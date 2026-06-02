/*
 * Button — a small library for Arduino to handle button debouncing
 *
 * MIT licensed.
 */

#include "Button.h"

// =============================================================================
//  Constructor
// =============================================================================

/**
 * Store configuration.  The actual pin mode and initial state are set in
 * begin() so that no hardware interaction happens before setup() runs.
 *
 * _lastChangeTime is initialised to 0.  Because the debounce check uses
 * unsigned elapsed-time arithmetic (millis() - _lastChangeTime >= _debounceMs),
 * a zero timestamp means the window has already expired by the time begin()
 * is called, so the first real read() sees a clean slate.
 *
 * _state is initialised to HIGH (RELEASED) as a safe default; begin() will
 * overwrite it with the real pin reading before any user code runs.
 */
Button::Button(uint8_t pin, uint16_t debounce_ms)
    : _pin(pin)
    , _debounceMs(debounce_ms)
    , _state(HIGH)
    , _lastChangeTime(0)
    , _hasChanged(false)
{
}

// =============================================================================
//  Initialisation
// =============================================================================

/**
 * Configure the pin with an internal pull-up resistor (button should connect
 * pin to GND) and sample the current pin state so that no spurious
 * pressed/released event fires if the button is held at boot.
 *
 * BUG FIX (original): the constructor set _state = HIGH unconditionally.
 * If the button was held down at power-on, the first read() detected a
 * HIGH→LOW "transition" and fired a spurious pressed() event.  Reading the
 * actual pin state here prevents that.
 */
void Button::begin()
{
    pinMode(_pin, INPUT_PULLUP);
    _state = (bool)digitalRead(_pin); // seed with real hardware state
}

// =============================================================================
//  Core update
// =============================================================================

/**
 * Sample the debounced pin state.
 *
 * Debounce algorithm
 * ------------------
 * Rather than storing an absolute future timestamp and comparing with >, we
 * record the time of the last detected edge (_lastChangeTime) and compute the
 * elapsed time as:
 *
 *   elapsed = millis() - _lastChangeTime
 *
 * Unsigned 32-bit subtraction wraps correctly at the millis() rollover
 * (~49.7 days), so elapsed is always a valid non-negative duration.  The
 * original approach stored an absolute deadline (_ignore_until = millis() + delay)
 * and tested (_ignore_until > millis()), which breaks catastrophically near
 * rollover: the deadline wraps to a small number, making the condition false
 * immediately and ignoring the debounce window entirely — or, in the opposite
 * case, locking the button for the entire next 49-day cycle.
 *
 * BUG FIX: replaced absolute-deadline comparison with elapsed-time arithmetic.
 *
 * @return Current debounced state: true = HIGH = RELEASED, false = LOW = PRESSED.
 */
bool Button::read()
{
    bool currentPin = (bool)digitalRead(_pin);

    // Only act on a pin change that has survived the debounce window.
    if (currentPin != _state)
    {
        uint32_t elapsed = millis() - _lastChangeTime; // safe across rollover

        if (elapsed >= _debounceMs)
        {
            _lastChangeTime = millis(); // record when this edge was accepted
            _state          = currentPin;
            _hasChanged     = true;
        }
        // If elapsed < _debounceMs, the glitch is within the debounce window —
        // ignore it and keep the previous stable state.
    }

    return _state;
}

// =============================================================================
//  Edge / state queries
// =============================================================================

/**
 * Destructive read of the change flag.
 *
 * Returns true once after each state change detected by read(), then resets
 * to false.  Subsequent calls return false until the next change.
 *
 * BUG FIX (original): the method was named has_changed() (public) and called
 * "mostly internal" in a comment.  Renamed to hasChanged() (camelCase,
 * consistent with Arduino style) and documented as destructive so callers are
 * not surprised.
 */
bool Button::hasChanged()
{
    if (_hasChanged)
    {
        _hasChanged = false;
        return true;
    }
    return false;
}

/**
 * Return true if the button transitioned to PRESSED on the last read().
 *
 * BUG FIX (original): the original called read() internally:
 *   return (read() == PRESSED && has_changed());
 * This meant pressed() advanced the debounce state machine itself.  If a
 * caller had already called read() in the same loop iteration, this second
 * read() could catch a new edge that the caller was not aware of, or (with
 * very short debounce) miss an event by re-entering the debounce window.
 * Additionally, if both pressed() and released() were called in the same
 * iteration, the second call's internal read() would find no new transition
 * and has_changed() would already be consumed, so the second condition always
 * returned false.
 *
 * Fixed: pressed() and released() now operate only on the state captured by
 * the most recent external read() call.  The caller is responsible for
 * calling read() once per loop, as documented in the header.
 */
bool Button::pressed()
{
    return (_state == PRESSED && hasChanged());
}

/**
 * Return true if the button transitioned to RELEASED on the last read().
 *
 * See pressed() for the full explanation of why internal read() calls were
 * removed.
 */
bool Button::released()
{
    return (_state == RELEASED && hasChanged());
}

/**
 * Return true if the button state changed on the last read() (either direction).
 *
 * Alias for hasChanged(); consumes the change flag.
 *
 * BUG FIX (original): the original called read() internally before
 * has_changed().  Same issue as pressed() and released() — removed.
 */
bool Button::toggled()
{
    return hasChanged();
}
