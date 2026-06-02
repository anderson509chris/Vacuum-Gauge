/*
 * Button — a small library for Arduino to handle button debouncing
 *
 * MIT licensed.
 */

#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

/**
 * @file Button.h
 * @brief Debounced digital button driver for Arduino.
 *
 * Provides edge-detection (pressed / released / toggled) on top of a simple
 * time-based debounce filter.  The debounce window uses unsigned elapsed-time
 * arithmetic, so it is immune to the millis() 32-bit rollover (~49.7 days).
 *
 * Typical usage
 * -------------
 * @code
 *   Button btn(7);          // pin 7, 100 ms debounce (default)
 *
 *   void setup() { btn.begin(); }
 *
 *   void loop() {
 *       btn.read();                         // must be called every loop
 *       if (btn.pressed())  doSomething();
 *       if (btn.released()) doSomethingElse();
 *   }
 * @endcode
 *
 * Important: call read() once per loop iteration.  pressed(), released(), and
 * toggled() all operate on the state snapshot captured by the most recent
 * read() call — do NOT call read() again between them in the same iteration.
 * Calling read() multiple times per loop may miss or duplicate events.
 *
 * Event consumption
 * -----------------
 * pressed(), released(), and toggled() each consume the change flag the first
 * time they return true.  If you need to test more than one condition in the
 * same iteration, cache the result of read() and use the state constants:
 *
 * @code
 *   bool state   = btn.read();
 *   bool changed = btn.hasChanged();
 *   if (changed && state == Button::PRESSED)  ...
 *   if (changed && state == Button::RELEASED) ...
 * @endcode
 */
class Button
{
public:
    // -----------------------------------------------------------------------
    // State constants
    // -----------------------------------------------------------------------

    /** Pin state when the button is physically pressed (active-low wiring). */
    static const uint8_t PRESSED  = LOW;

    /** Pin state when the button is physically released. */
    static const uint8_t RELEASED = HIGH;

    // -----------------------------------------------------------------------
    // Construction / initialisation
    // -----------------------------------------------------------------------

    /**
     * @brief Construct a Button bound to @p pin.
     *
     * @param pin          Arduino pin number connected to the button.
     * @param debounce_ms  Debounce window in milliseconds.  State changes are
     *                     ignored for this long after each detected edge.
     *                     Default is 100 ms.
     */
    explicit Button(uint8_t pin, uint16_t debounce_ms = 100);

    /**
     * @brief Initialise the pin mode and sample the initial button state.
     *
     * Must be called from setup() before any other method.  Reads the actual
     * pin state so that no spurious pressed/released event fires at boot even
     * if the button is held down at startup.
     */
    void begin();

    // -----------------------------------------------------------------------
    // Core update — call exactly once per loop() iteration
    // -----------------------------------------------------------------------

    /**
     * @brief Sample the pin and update internal state.
     *
     * Applies the debounce filter.  Must be called once per loop() iteration
     * before testing pressed(), released(), or toggled().
     *
     * @return Current debounced pin state: Button::PRESSED or Button::RELEASED.
     */
    bool read();

    // -----------------------------------------------------------------------
    // Edge / state queries — valid after the most recent read() call
    // -----------------------------------------------------------------------

    /**
     * @brief Return true if the button state changed on the last read().
     *
     * This is a **destructive read**: the flag is cleared after the first call
     * that returns true.  Subsequent calls return false until the next change.
     *
     * Prefer pressed() / released() / toggled() for most use cases.
     */
    bool hasChanged();

    /**
     * @brief Return true if the button was pressed on the last read().
     *
     * Equivalent to: state == PRESSED && hasChanged().
     * Consumes the change flag.
     */
    bool pressed();

    /**
     * @brief Return true if the button was released on the last read().
     *
     * Equivalent to: state == RELEASED && hasChanged().
     * Consumes the change flag.
     */
    bool released();

    /**
     * @brief Return true if the button state changed on the last read(),
     *        regardless of direction.
     *
     * Equivalent to hasChanged().  Provided as a named alias for readability.
     * Consumes the change flag.
     */
    bool toggled();

private:
    uint8_t  _pin;             ///< Arduino pin number
    uint16_t _debounceMs;      ///< Debounce window in milliseconds
    bool     _state;           ///< Current debounced state (true = HIGH = RELEASED)
    uint32_t _lastChangeTime;  ///< millis() timestamp of the last detected edge
    bool     _hasChanged;      ///< True if state changed since last hasChanged() call
};

#endif // BUTTON_H
