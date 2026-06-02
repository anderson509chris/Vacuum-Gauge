#ifndef DEBUG_H_
#define DEBUG_H_

/**
 * @file debug.h
 * @brief Hardware diagnostic command module for Arduino-based systems.
 *
 * Provides a `debug` class that registers a block of serial commands with a
 * `commandProcessor` instance.  Once registered, the following command groups
 * are available over the serial interface:
 *
 *   Memory access   — SETADDRESS, SETOFFSET, WRITE, READ, DUMP, RAM
 *   Digital I/O     — PINMODE, DOUT, DIN
 *   Analog I/O      — ADCRES, DACRES, ADC, DAC
 *   System info     — UPTIME, RESET, UUID, CPUTEMP
 *   Development     — DEBUG (calls a user-registered callback)
 *
 * Supported targets (selected via preprocessor):
 *   SAMD21_SERIES, SAMD51_SERIES, SAM3X8, ARDUINO_TEENSY40, ESP_PLATFORM
 *
 * Usage
 * -----
 *   commandProcessor cp;
 *   debug dbg(&cp);
 *   cp.registerCommands(dbg.debugCommands());
 *
 * LIMITATION: Only one `debug` instance may exist at a time.  The
 * implementation uses a module-level static pointer (cp) that is overwritten
 * by each constructor call.
 */

#include <Arduino.h>
#include "commandProcessor.h"

class debug
{
public:
    /**
     * @brief Construct a debug module and bind it to a commandProcessor.
     *
     * Sets the module-level `cp` pointer used by all command callbacks.
     * Call `cp.registerCommands(debugCommands())` after construction to
     * make the commands available.
     *
     * @param cmdP  Pointer to the commandProcessor that will own these commands.
     *              Must remain valid for the lifetime of this object.
     */
    explicit debug(commandProcessor *cmdP);

    /**
     * @brief Return the CommandList node that holds all debug commands.
     *
     * Pass the returned pointer to commandProcessor::registerCommands().
     */
    CommandList *debugCommands(void);

    /**
     * @brief Register a user callback invoked by the DEBUG serial command.
     *
     * Useful during development to trigger a custom diagnostic routine over
     * the serial interface without adding a new command.
     *
     * @param function  Callback with signature `void fn(void)`, or NULL to
     *                  disable.
     */
    void registerDebugFunction(void (*function)(void));

    /**
     * @brief Override the memory base address used by READ/WRITE/DUMP.
     *
     * Equivalent to issuing SETADDRESS over the serial interface, but callable
     * from code.  Resets the address offset to zero.
     *
     * @param address  New base address (32-bit, interpreted as a raw pointer).
     */
    void setAddress(uint32_t address);

    /**
     * @brief Perform an immediate software reset of the microcontroller.
     *
     * Also exposed as the RESET serial command.  Declared static so it can be
     * stored as a plain function pointer in the Command table.
     */
    static void softwareReset(void);

private:
    // No private data — all state is held in module-level statics in debug.cpp.
    // A future refactor should move that state here.
};

#endif // DEBUG_H_
