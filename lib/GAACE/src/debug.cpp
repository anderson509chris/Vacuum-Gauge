#include "debug.h"

// =============================================================================
//  Module-level state
//
//  All command callbacks in this file are plain C functions stored as
//  CMDfunction pointers in the Command table.  They share state through these
//  file-scope statics rather than through class members.
//
//  LIMITATION: Only one debug instance may exist at a time.  Constructing a
//  second instance overwrites `cp` and the first instance's callbacks will
//  silently operate on the wrong commandProcessor.
// =============================================================================

/** commandProcessor that owns these commands. Set by debug::debug(). */
static commandProcessor *cp = NULL;

/** Base address used by SETADDRESS / READ / WRITE / DUMP. */
static uint32_t baseAddress = 0;

/**
 * Byte offset added to baseAddress for READ / WRITE / DUMP.
 * Reset to zero whenever baseAddress changes.
 */
static uint32_t addOffset = 0;

/** Optional user-supplied callback invoked by the DEBUG command. */
static void (*debugFunction)(void) = NULL;

// =============================================================================
//  Internal helpers
// =============================================================================

/** Invoke the user-registered debug callback (no-op if none registered). */
static void callDebug(void)
{
    if (debugFunction != NULL) debugFunction();
}

// =============================================================================
//  Platform-specific ADC helpers
// =============================================================================

#if SAMD21_SERIES
/**
 * @brief Spin-wait until the SAMD21 ADC synchronisation is complete.
 * Required before changing ADC registers or reading results on SAMD21.
 */
static inline void syncADC(void) __attribute__((always_inline, unused));
static inline void syncADC(void)
{
    while (ADC->STATUS.bit.SYNCBUSY == 1);
}

/**
 * @brief Perform a single-shot ADC conversion on the SAMD21.
 *
 * The first conversion after a reference change is discarded per the SAMD21
 * datasheet errata — two conversions are always performed and only the second
 * result is returned.
 *
 * @param ch  MUXPOS channel index.
 * @return    Raw ADC result (12-bit by default, normalised to 12 bits for
 *            8- and 10-bit resolution settings).
 */
static int readADC(int ch)
{
    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = ch;

    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x01;   // Enable ADC

    // First conversion (discarded — datasheet requirement after ref change)
    syncADC();
    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0);
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY; // Clear Data Ready flag

    // Second conversion — this is the valid result
    syncADC();
    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0);
    int valueRead = ADC->RESULT.reg;

    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x00;   // Disable ADC
    syncADC();

    return valueRead;
}
#endif // SAMD21_SERIES

#if SAMD51_SERIES
/**
 * @brief Perform a single-shot ADC conversion on the SAMD51.
 *
 * The first conversion is discarded as required by the SAMD51 datasheet.
 *
 * @param adc  Pointer to the ADC peripheral (ADC0 or ADC1).
 * @param ch   MUXPOS channel index.
 * @return     Raw ADC result.
 */
static int readADC(Adc *adc, int ch)
{
    while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL);
    adc->INPUTCTRL.bit.MUXPOS = ch;

    while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);
    adc->CTRLA.bit.ENABLE = 0x01;   // Enable ADC

    // First conversion (discarded — datasheet requirement)
    while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);
    adc->SWTRIG.bit.START = 1;
    adc->INTFLAG.reg = ADC_INTFLAG_RESRDY; // Clear Data Ready flag
    adc->SWTRIG.bit.START = 1;             // Second trigger (valid result)

    while (adc->INTFLAG.bit.RESRDY == 0);
    int valueRead = adc->RESULT.reg;

    while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);
    adc->CTRLA.bit.ENABLE = 0x00;   // Disable ADC
    while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);

    return valueRead;
}
#endif // SAMD51_SERIES

// =============================================================================
//  Analog I/O commands
// =============================================================================

/** ADCRES — set the ADC resolution in bits (2–16). */
static void setADCres(void)
{
    int i;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&i, 2, 16))
    {
        analogReadResolution(i);
        cp->sendACK();
        return;
    }
    cp->sendNAK();
}

/** DACRES — set the DAC (PWM) output resolution in bits (2–16). */
static void setDACres(void)
{
    int i;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&i, 2, 16))
    {
        analogWriteResolution(i);
        cp->sendACK();
        return;
    }
    cp->sendNAK();
}

/** ADC — read and print the raw value of an ADC input channel (0–100). */
static void getADC(void)
{
    int i;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&i, 0, 100))
    {
        cp->sendACK(false);
        cp->println(analogRead(i));
        return;
    }
    cp->sendNAK();
}

/**
 * DAC — write a value to an analog/PWM output channel.
 * Arguments: channel (0–100), value (0–65535).
 */
static void setDAC(void)
{
    int ch, i;
    if (cp->getNumArgs() != 2) { cp->sendNAK(); return; }
    if (cp->getValue(&ch, 0, 100))
    {
        if (cp->getValue(&i, 0, 65535))
        {
            analogWrite(ch, i);
            cp->sendACK();
            return;
        }
    }
    cp->sendNAK();
}

// =============================================================================
//  System information commands
// =============================================================================

/**
 * @brief CPUTEMP — read and report the on-chip temperature sensor in °C.
 *
 * Implementation is heavily platform-specific.  On unsupported platforms
 * `treal` is left at its sentinel value (-1001) and NAK is returned.
 *
 * SAMD21 / SAMD51: Uses the factory-programmed temperature calibration values
 * stored in NVM to compensate the raw ADC reading.
 *
 * SAM3X8: Uses the built-in temperature channel of the 12-bit ADC.
 *
 * Teensy 4.0: Delegates to the SDK helper `tempmonGetTemp()`.
 */
static void cpuTemp(void)
{
    float treal = -1001.0f; // sentinel — means "unsupported platform"

    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }

#if SAMD21_SERIES
    // Factory calibration row is at NVM address 0x00806030.
    // Word 0 encodes: roomTemp (8.4 fixed), hotTemp (8.4 fixed),
    //                 room1V correction, hot1V correction.
    // Word 1 encodes: room ADC count, hot ADC count.
    const uint32_t *tempLog = (const uint32_t *)0x00806030;

    float    roomTemp = (float)(tempLog[0] & 0xFF)
                      + (float)((tempLog[0] >> 8)  & 0x0F) / 10.0f;
    float    hotTemp  = (float)((tempLog[0] >> 12) & 0xFF)
                      + (float)((tempLog[0] >> 20) & 0x0F) / 10.0f;
    float    room1V   = 1.0f - (float)((int8_t)((tempLog[0] >> 24) & 0xFF)) / 1000.0f;
    float    hot1V    = 1.0f - (float)((int8_t)( tempLog[1] & 0xFF))        / 1000.0f;
    uint16_t roomADC  = (tempLog[1] >> 8)  & 0xFFF;
    uint16_t hotADC   = (tempLog[1] >> 20) & 0xFFF;

    // Save and override ADC gain / reference for temperature measurement.
    int savedGain = ADC->INPUTCTRL.bit.GAIN;
    int savedRef  = ADC->REFCTRL.bit.REFSEL;
    analogReference(AR_INTERNAL1V0);
    SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN;

    int valueRead = readADC(0x18); // channel 0x18 = temperature sensor

    // Normalise lower-resolution readings to 12-bit before applying formula.
    if (ADC->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_10BIT_Val) valueRead <<= 2;
    if (ADC->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_8BIT_Val)  valueRead <<= 4;

    // Restore original ADC settings.
    ADC->INPUTCTRL.bit.GAIN  = savedGain;
    ADC->REFCTRL.bit.REFSEL  = savedRef;

    // Interpolation formula from SAMD21 datasheet section 37.9.
    treal = roomTemp
          + (( (float)valueRead         / 4095.0f)
             - ((float)roomADC * room1V / 4095.0f))
          * (hotTemp - roomTemp)
          / ( ((float)hotADC  * hot1V  / 4095.0f)
             -((float)roomADC * room1V / 4095.0f));
#endif // SAMD21_SERIES

#if SAMD51_SERIES
    // Factory calibration row at 0x00800100.
    // Word 0: TL (low ref temp, 8.4), TH (high ref temp, 8.4).
    // Word 1: VPL (PTAT at low temp), VPH (PTAT at high temp).
    // Word 2: VCL (CTAT at low temp), VCH (CTAT at high temp).
    const uint32_t *tempLog = (const uint32_t *)0x00800100;

    float   TL  = (float)( tempLog[0]        & 0xFF)
                + (float)((tempLog[0] >>  8)  & 0x0F) / 10.0f;
    float   TH  = (float)((tempLog[0] >> 12) & 0xFF)
                + (float)((tempLog[0] >> 20)  & 0x0F) / 10.0f;
    int16_t VPL = (tempLog[1] >>  8) & 0xFFF;
    int16_t VPH = (tempLog[1] >> 20) & 0xFFF;
    int16_t VCL =  tempLog[2]        & 0xFFF;
    int16_t VCH = (tempLog[2] >> 12) & 0xFFF;

    // Save VREF state, then configure for temperature measurement.
    int savedOnDemand = SUPC->VREF.bit.ONDEMAND;
    int savedVrefOE   = SUPC->VREF.bit.VREFOE;
    SUPC->VREF.bit.ONDEMAND = 0;
    SUPC->VREF.bit.VREFOE   = 0;
    SUPC->VREF.bit.TSEN     = 1;

    Adc *adc = ADC0;

    // Read PTAT (Proportional To Absolute Temperature) value.
    SUPC->VREF.bit.TSSEL = 0; // 0 = PTAT
    int16_t TP = (int16_t)readADC(adc, 0x1C);
    if (adc->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_10BIT_Val) TP <<= 2;
    if (adc->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_8BIT_Val)  TP <<= 4;

    // Read CTAT (Complementary To Absolute Temperature) value.
    SUPC->VREF.bit.TSSEL = 1; // 1 = CTAT
    int16_t TC = (int16_t)readADC(adc, 0x1D);
    if (adc->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_10BIT_Val) TC <<= 2;
    if (adc->CTRLB.bit.RESSEL == ADC_CTRLB_RESSEL_8BIT_Val)  TC <<= 4;

    // Interpolation formula from SAMD51 datasheet section 45.6.3.1.
    treal = (TL * VPH * TC - VPL * TH * TC - TL * VCH * TP + TH * VCL * TP)
          / (VCL * TP - VCH * TP - VPL * TC + VPH * TC);

    // Restore VREF state.
    SUPC->VREF.bit.ONDEMAND = savedOnDemand;
    SUPC->VREF.bit.VREFOE   = savedVrefOE;
    SUPC->VREF.bit.TSEN     = 0;
#endif // SAMD51_SERIES

#if SAM3X8
    // Enable ADC channel 15 and activate the temperature sensor.
    ADC->ADC_CHER  = 1 << 15;
    ADC->ADC_ACR  |= ADC_ACR_TSON;

    // First conversion (discard result — warm-up read).
    ADC->ADC_CR = ADC_CR_START;
    // BUG FIX: original was (ADC_ISR & EOC15 == EOC15) which is
    // (ADC_ISR & 1) due to == binding tighter than &.  Parentheses added.
    while ((ADC->ADC_ISR & ADC_ISR_EOC15) != ADC_ISR_EOC15);
    delay(100); // Datasheet requires settling time after sensor enable
    (void)ADC->ADC_LCDR; // discard first reading

    // Second conversion — valid result.
    ADC->ADC_CR = ADC_CR_START;
    while ((ADC->ADC_ISR & ADC_ISR_EOC15) != ADC_ISR_EOC15); // FIXED precedence
    delay(100);
    int mV = ADC->ADC_LCDR;

    // Disable temperature channel.
    ADC->ADC_CHDR = 1 << 15;

    // Convert raw counts to millivolts then to °C (SAM3X8 datasheet formula).
    treal = (((3300 * mV) / 4096) - 800) * 0.37736f + 25.5f;
#endif // SAM3X8

#ifdef ARDUINO_TEENSY40
    treal = tempmonGetTemp();
#endif

    if (treal < -1000.0f) { cp->sendNAK(); return; } // unsupported platform
    cp->sendACK(false);
    cp->println(treal);
}

/**
 * @brief UUID — print the 128-bit factory-programmed unique ID in hex.
 *
 * Address of the UUID word array varies by platform:
 *   SAMD21: 0x0080A00C
 *   SAMD51: 0x008061FC
 *   SAM3X8: read via EEFC (Enhanced Embedded Flash Controller)
 */
static void UUID(void)
{
    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }

    const unsigned int *val = NULL;

#if SAMD21_SERIES
    val = (const unsigned int *)0x0080A00C;
#endif

#if SAMD51_SERIES
    val = (const unsigned int *)0x008061FC;
#endif

#if SAM3X8
    // On SAM3X8 the UUID must be read via the EEFC Read Unique Identifier
    // command; it is not memory-mapped.
    unsigned int adwUniqueID[4];
    _EEFC_ReadUniqueID(adwUniqueID);
    val = adwUniqueID;
#endif

    if (val == NULL) { cp->sendNAK(); return; } // unsupported platform

    cp->sendACK(false);
    cp->print("ID: ");
    for (byte b = 0; b < 4; b++) cp->print((uint32_t)val[b], HEX);
    cp->print();
}

/**
 * @brief RESET — perform an immediate software reset of the microcontroller.
 *
 * This is a static method so it can be stored as a plain function pointer in
 * the Command table.  The trailing `while(true)` is a no-return safety net
 * for compilers that do not recognise the platform reset intrinsics as
 * diverging.
 */
void debug::softwareReset(void)
{
    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }
    cp->sendACK();

#if SAMD21_SERIES || SAMD51_SERIES
    NVIC_SystemReset();

#elif SAM3X8
    const int RSTC_KEY = 0xA5;
    RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;

#elif defined(ARDUINO_TEENSY40)
    // Trigger Cortex-M7 software reset via the Application Interrupt and
    // Reset Control Register (AIRCR).
    static volatile uint32_t * const AIRCR    = (volatile uint32_t *)0xE000ED0C;
    static const    uint32_t         RESET_VAL = 0x5FA0004u; // VECTKEY | SYSRESETREQ
    *AIRCR = RESET_VAL;
#endif

    while (true); // should never be reached; silences "no-return" warnings
}

/** UPTIME — print elapsed time since last boot in minutes. */
static void upTime(void)
{
    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }
    cp->sendACK(false);
    float uptime = (float)millis() / 60000.0f;
    cp->print("System has been up for at least: ");
    cp->print(uptime);
    cp->println(" minutes");
}

// =============================================================================
//  Memory access commands
// =============================================================================

/**
 * @brief SETADDRESS — set the base address used by READ / WRITE / DUMP.
 *
 * Expects one hex argument.  Also resets the address offset to zero so that
 * a fresh SETADDRESS always starts from a clean state.
 */
static void setAddress(void)
{
    uint32_t i;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&i, 0, 0, HEX))
    {
        baseAddress = i;
        addOffset   = 0; // reset offset whenever the base changes
        cp->sendACK(false);
        cp->print("Base address, hex = ");
        cp->println(baseAddress, HEX);
    }
    else cp->sendNAK();
}

/**
 * @brief SETOFFSET — set the byte offset added to baseAddress for READ/WRITE/DUMP.
 *
 * Expects one hex argument.
 *
 * BUG FIX: The original function reset addOffset to 0 immediately after
 * storing it (copy-paste from setAddress), discarding the value just set.
 * The spurious `addOffset = 0` line has been removed.
 */
static void setOffset(void)
{
    uint32_t i;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&i, 0, 0, HEX))
    {
        addOffset = i; // BUG FIX: was followed by `addOffset = 0` — removed
        cp->sendACK(false);
        cp->print("Address offset, hex = ");
        cp->println(addOffset, HEX);
    }
    else cp->sendNAK();
}

/**
 * @brief WRITE — write a typed value to memory at (baseAddress + addOffset).
 *
 * Arguments: type, value
 *   type  — one of: BYTE (uint8), WORD (uint16), DWORD (uint32), INT, FLOAT
 *   value — value to write; BYTE/WORD/DWORD accept hex, INT/FLOAT accept decimal
 *
 * BUG FIX (original): `bool status` was declared but only assigned inside
 * each type branch and then used unconditionally in sendACK() below.  If no
 * branch matched, `status` was uninitialised.  Also, `sendACK()` was called
 * even when `status` was false (bad argument).  Fixed: ACK is sent only on
 * success, NAK on unknown type or bad argument.
 */
static void writeMemory(void)
{
    char    *res;
    void    *ptr = (void *)(baseAddress + addOffset);

    if (cp->getNumArgs() != 2) { cp->sendNAK(); return; }
    if (!cp->getValue(&res)) { cp->sendNAK(); return; }

    bool ok = false;

    if      (strcasecmp(res, "BYTE")  == 0)
    {
        uint32_t uval;
        ok = cp->getValue(&uval, 0, 0xFF, HEX);
        if (ok) *(uint8_t *)ptr = (uint8_t)uval;
    }
    else if (strcasecmp(res, "WORD")  == 0)
    {
        uint32_t uval;
        ok = cp->getValue(&uval, 0, 0xFFFF, HEX);
        if (ok) *(uint16_t *)ptr = (uint16_t)uval;
    }
    else if (strcasecmp(res, "DWORD") == 0)
    {
        uint32_t uval;
        // ll == ul == 0 means "no range check" in getValue
        ok = cp->getValue(&uval, 0, 0, HEX);
        if (ok) *(uint32_t *)ptr = uval;
    }
    else if (strcasecmp(res, "INT")   == 0)
    {
        int ival;
        ok = cp->getValue(&ival);
        if (ok) *(int *)ptr = ival;
    }
    else if (strcasecmp(res, "FLOAT") == 0)
    {
        float fval;
        ok = cp->getValue(&fval);
        if (ok) *(float *)ptr = fval;
    }
    // else: unknown type — ok stays false

    if (ok) cp->sendACK();
    else    cp->sendNAK();
}

/**
 * @brief READ — print a typed value from memory at (baseAddress + addOffset).
 *
 * Argument: type — one of BYTE, WORD, DWORD, INT, FLOAT
 */
static void readMemory(void)
{
    char *res;
    void *ptr = (void *)(baseAddress + addOffset);

    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (!cp->getValue(&res)) { cp->sendNAK(); return; }

    cp->sendACK(false);

    if      (strcasecmp(res, "BYTE")  == 0) cp->println((int)*(uint8_t  *)ptr, HEX);
    else if (strcasecmp(res, "WORD")  == 0) cp->println((int)*(uint16_t *)ptr, HEX);
    else if (strcasecmp(res, "DWORD") == 0) cp->println(    *(uint32_t  *)ptr, HEX);
    else if (strcasecmp(res, "INT")   == 0) cp->println(    *(int       *)ptr);
    else if (strcasecmp(res, "FLOAT") == 0) cp->println(    *(float     *)ptr);
    else cp->sendNAK(); // unknown type — ACK already sent; best we can do is NAK
}

/**
 * @brief RAM — report available heap/stack space.
 *
 * ESP32: reports minimum free heap since boot via the IDF API.
 *
 * Other: estimates free RAM as the distance between the current stack top
 * and the current heap break (sbrk(0)).  This is an approximation — it does
 * not account for heap fragmentation or memory reserved by the runtime.
 */
#ifdef ESP_PLATFORM
static void freeRam(void)
{
    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }
    cp->sendACK(false);
    cp->println((uint32_t)esp_get_minimum_free_heap_size());
}
#else
extern "C" char *sbrk(int incr);

static void freeRam(void)
{
    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }
    cp->sendACK(false);
    // Stack grows downward; heap break (sbrk(0)) grows upward.
    // The difference is an upper-bound estimate of free RAM between them.
    char top;
    int free = (int)(&top - (char *)sbrk(0));
    cp->println(free);
}
#endif

/**
 * @brief DUMP — print a 512-byte hex dump starting at (baseAddress + addOffset).
 *
 * Output format: 32 rows × 16 bytes, with a header row showing column offsets.
 *
 * BUG FIX: original passed literal 16 (int) as the fmt argument to println().
 * This works only because HEX == 16 on Arduino; changed to HEX for clarity.
 */
static void dumpMemory(void)
{
    char     sbuf[10];
    uint8_t *buf = (uint8_t *)(baseAddress + addOffset);

    if (cp->getNumArgs() != 0) { cp->sendNAK(); return; }
    cp->sendACK(false);

    cp->print("Memory dump at: ");
    cp->println(baseAddress + addOffset, HEX); // BUG FIX: was literal 16

    // Print column header (byte offsets 00–0F).
    cp->print("     ");
    for (int j = 0; j < 16; j++)
    {
        sprintf(sbuf, "%02X ", j);
        cp->print(sbuf);
    }
    cp->print();

    // Print 32 rows of 16 bytes each.
    for (int i = 0; i < 32; i++)
    {
        sprintf(sbuf, "%03X: ", i * 16);
        cp->print(sbuf);
        for (int j = 0; j < 16; j++)
        {
            sprintf(sbuf, "%02X ", buf[(i * 16) + j]);
            cp->print(sbuf);
        }
        cp->print();
    }
}

// =============================================================================
//  Digital I/O commands
// =============================================================================

/**
 * @brief PINMODE — configure a GPIO pin direction.
 *
 * Arguments: pin (0–100), mode (INPUT | OUTPUT | PULLUP)
 *
 * BUG FIX: original used `if(strcasecmp(mode,"INPUT"))` which is truthy when
 * the strings do NOT match (strcasecmp returns 0 on match).  Every branch was
 * therefore inverted — the wrong pinMode was called for every input string.
 * Fixed to `strcasecmp(...) == 0`.
 */
static void setPinMode(void)
{
    int  pin;
    char *mode;

    if (cp->getNumArgs() != 2) { cp->sendNAK(); return; }
    if (!cp->getValue(&pin, 0, 100)) { cp->sendNAK(); return; }
    if (!cp->getValue(&mode, "INPUT,OUTPUT,PULLUP")) { cp->sendNAK(); return; }

    // BUG FIX: all comparisons were inverted (missing == 0)
    if      (strcasecmp(mode, "INPUT")  == 0) pinMode(pin, INPUT);
    else if (strcasecmp(mode, "PULLUP") == 0) pinMode(pin, INPUT_PULLUP);
    else if (strcasecmp(mode, "OUTPUT") == 0) pinMode(pin, OUTPUT);

    cp->sendACK();
}

/**
 * @brief DOUT — drive a digital output pin.
 *
 * Arguments: pin (0–100), action (LOW | HIGH | PULSE | SPULSE)
 *   LOW    — drive pin low
 *   HIGH   — drive pin high
 *   PULSE  — toggle the pin instantaneously (no delay)
 *   SPULSE — toggle the pin with a 1 ms dwell in the active state
 *
 * BUG FIX 1: same inverted strcasecmp as setPinMode.
 * BUG FIX 2: options string had "SPLUSE" (typo) — corrected to "SPULSE".
 */
static void setPin(void)
{
    int  pin;
    char *mode;

    if (cp->getNumArgs() != 2) { cp->sendNAK(); return; }
    if (!cp->getValue(&pin, 0, 100)) { cp->sendNAK(); return; }
    // BUG FIX: "SPLUSE" corrected to "SPULSE"
    if (!cp->getValue(&mode, "LOW,HIGH,PULSE,SPULSE")) { cp->sendNAK(); return; }

    // BUG FIX: all comparisons were inverted (missing == 0)
    if (strcasecmp(mode, "LOW") == 0)
    {
        digitalWrite(pin, LOW);
    }
    else if (strcasecmp(mode, "HIGH") == 0)
    {
        digitalWrite(pin, HIGH);
    }
    else if (strcasecmp(mode, "PULSE") == 0)
    {
        // Instantaneous toggle with no dwell time.
        if (digitalRead(pin) == LOW) { digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
        else                         { digitalWrite(pin, LOW);  digitalWrite(pin, HIGH); }
    }
    else if (strcasecmp(mode, "SPULSE") == 0)
    {
        // Slow pulse: toggle with a 1 ms dwell before returning to the original state.
        if (digitalRead(pin) == LOW) { digitalWrite(pin, HIGH); delay(1); digitalWrite(pin, LOW); }
        else                         { digitalWrite(pin, LOW);  delay(1); digitalWrite(pin, HIGH); }
    }

    cp->sendACK();
}

/** DIN — read and print the digital state of a GPIO pin (HIGH or LOW). */
static void getPin(void)
{
    int pin;
    if (cp->getNumArgs() != 1) { cp->sendNAK(); return; }
    if (cp->getValue(&pin, 0, 100))
    {
        cp->sendACK(false);
        cp->println(digitalRead(pin) == HIGH ? "HIGH" : "LOW");
        return;
    }
    cp->sendNAK();
}

// =============================================================================
//  Command table and list node
//
//  Future additions to consider:
//    - ESP32 processor support (partial: ESP_PLATFORM freeRam already present)
//    - Counter with threshold and output trigger generation
//    - Frequency generation with burst capability
//    - Watchdog timer support (enable, pet, test-reset)
// =============================================================================

static Command debugCmds[] =
{
    // Memory access
    {"SETADDRESS", CMDfunction, -1, (void *)setAddress,           NULL, "Set base address in hex"},
    {"SETOFFSET",  CMDfunction, -1, (void *)setOffset,            NULL, "Set offset from base address in hex"},
    {"WRITE",      CMDfunction, -1, (void *)writeMemory,          NULL, "Write BYTE,WORD,DWORD,INT, or FLOAT to baseaddress+offset"},
    {"READ",       CMDfunction, -1, (void *)readMemory,           NULL, "Read BYTE,WORD,DWORD,INT, or FLOAT from baseaddress+offset"},
    {"DUMP",       CMDfunction, -1, (void *)dumpMemory,           NULL, "Hex dump 512 bytes from baseaddress+offset"},
    {"RAM",        CMDfunction, -1, (void *)freeRam,              NULL, "Display approximate free RAM in bytes"},
    // Digital I/O
    {"PINMODE",    CMDfunction, -1, (void *)setPinMode,           NULL, "Set pin mode: pin, INPUT|OUTPUT|PULLUP"},
    {"DOUT",       CMDfunction, -1, (void *)setPin,               NULL, "Drive output pin: pin, HIGH|LOW|PULSE|SPULSE"},
    {"DIN",        CMDfunction, -1, (void *)getPin,               NULL, "Read input pin state: pin -> HIGH or LOW"},
    // Analog I/O
    {"ADCRES",     CMDfunction, -1, (void *)setADCres,            NULL, "Set ADC resolution in bits (2–16)"},
    {"DACRES",     CMDfunction, -1, (void *)setDACres,            NULL, "Set DAC/PWM resolution in bits (2–16)"},
    {"ADC",        CMDfunction, -1, (void *)getADC,               NULL, "Read ADC input channel"},
    {"DAC",        CMDfunction, -1, (void *)setDAC,               NULL, "Write DAC output channel: channel, value"},
    // System / debug
    {"DEBUG",      CMDfunction, -1, (void *)callDebug,            NULL, "Invoke the user-registered debug callback"},
    {"UPTIME",     CMDfunction, -1, (void *)upTime,               NULL, "Report system uptime since last reboot"},
    {"RESET",      CMDfunction, -1, (void *)debug::softwareReset, NULL, "Reboot the system immediately"},
    {"UUID",       CMDfunction, -1, (void *)UUID,                 NULL, "Print the 128-bit factory UUID in hex"},
    {"CPUTEMP",    CMDfunction, -1, (void *)cpuTemp,              NULL, "Report CPU die temperature in °C"},
    {NULL} // sentinel
};

static CommandList debugList = {debugCmds, NULL};

// =============================================================================
//  debug class implementation
// =============================================================================

debug::debug(commandProcessor *cmdP)
{
    cp            = cmdP;
    debugFunction = NULL;
    // Note: debugList is populated at file-scope initialisation time;
    // no further setup is needed here.
}

CommandList *debug::debugCommands(void)
{
    return &debugList;
}

void debug::registerDebugFunction(void (*function)(void))
{
    debugFunction = function;
}

void debug::setAddress(uint32_t address)
{
    baseAddress = address;
    addOffset   = 0; // reset offset when base changes, consistent with SETADDRESS command
}
