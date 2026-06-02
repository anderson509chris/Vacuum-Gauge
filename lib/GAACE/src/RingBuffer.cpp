#include "RingBuffer.h"

// =============================================================================
//  Construction / destruction
// =============================================================================

ringBuffer::ringBuffer(int rbSize)
    : Size(rbSize)
    , Tail(0)
    , Head(0)
    , Count(0)
    , Commands(0)
    , EOLchars(";\n\r")
    , Ignore("")
{
    Buffer = new char[rbSize]; // allocated as an array — freed with delete[] in destructor
}

ringBuffer::~ringBuffer()
{
    delete[] Buffer; // BUG FIX: original used scalar `delete`, UB for array allocations
}

// =============================================================================
//  Private helpers
// =============================================================================

/**
 * @brief Return true if @p ch is one of the configured EOL characters.
 *
 * Linear scan of EOLchars — acceptable because the string is very short
 * (typically 2–3 characters).  Using a dedicated method rather than
 * inlining the loop everywhere ensures a single match per character (the
 * original inline loops did not break after the first match, so a character
 * appearing twice in EOLchars would double-count).
 */
bool ringBuffer::isEOL(char ch) const
{
    for (int i = 0; i < (int)EOLchars.length(); i++)
        if (ch == EOLchars[i]) return true;
    return false;
}

/** @brief Return true if @p ch should be dropped by put(). */
bool ringBuffer::isIgnored(char ch) const
{
    for (int i = 0; i < (int)Ignore.length(); i++)
        if (ch == Ignore[i]) return true;
    return false;
}

// =============================================================================
//  Basic state queries
// =============================================================================

int ringBuffer::count(void) const
{
    return Count;
}

int ringBuffer::lines(void) const
{
    return Commands;
}

// =============================================================================
//  Single-character operations
// =============================================================================

/**
 * @brief Remove and return the next character from the buffer.
 *
 * BUG FIX (original): when the buffer was empty the original reset
 * `Commands = 0`.  That silently hid any count inconsistency instead of
 * simply returning the sentinel.  Now the function returns 0xFF without
 * touching any state.
 *
 * BUG FIX (original): the original decremented Commands by iterating
 * EOLchars without breaking after the first match.  If the same character
 * appeared twice in EOLchars it would decrement Commands twice per EOL
 * character, corrupting the count.  Fixed by using isEOL() which returns
 * on the first match.
 */
char ringBuffer::get(void)
{
    if (Count == 0) return (char)0xFF; // BUG FIX: no longer zeroes Commands

    char ch = Buffer[Head];
    Head = (Head + 1) % Size;
    Count--;

    if (isEOL(ch)) Commands--; // BUG FIX: single decrement guaranteed by isEOL()

    return ch;
}

/** @brief Return the next character without removing it. */
char ringBuffer::peek(void) const
{
    if (Count == 0) return (char)0xFF;
    return Buffer[Head];
}

/**
 * @brief Add a character to the buffer.
 *
 * BUG FIX (original): the original iterated the full EOLchars string without
 * breaking after the first match, so a character appearing twice in EOLchars
 * would increment Commands twice.  Fixed by using isEOL().
 *
 * @return  0 on success; -1 if the buffer is full.
 */
int ringBuffer::put(char ch)
{
    if (isIgnored(ch)) return 0;          // silently drop ignored characters
    if (Count >= Size) return -1;         // buffer full

    Buffer[Tail] = ch;
    Tail = (Tail + 1) % Size;
    Count++;

    if (isEOL(ch)) Commands++;            // BUG FIX: single increment guaranteed

    return 0;
}

// =============================================================================
//  Buffer management
// =============================================================================

/**
 * @brief Discard all buffered characters and reset all state counters.
 *
 * EOLchars and Ignore are not modified.
 */
void ringBuffer::clear(void)
{
    Tail = Head = Count = Commands = 0;
}

// =============================================================================
//  Line-oriented operations
// =============================================================================

/**
 * @brief Copy the next complete line into @p buf and consume it from the buffer.
 *
 * Reads characters one by one.  When get() consumes an EOL character the line
 * count (Commands) drops, which is the exit signal.  The EOL character is
 * consumed but not written to @p buf.
 *
 * The trailing `while` drain from the original is removed: because get() now
 * correctly decrements Commands on the first (and only) matching EOL character,
 * the loop exits immediately without needing a separate drain pass.
 *
 * @return Number of characters written (excluding NUL), or -1 if no complete
 *         line is available.
 */
int ringBuffer::getLine(char *buf, int maxLen)
{
    if (Commands == 0) return -1;

    int startCmds = Commands;
    int i         = 0;
    buf[0]        = '\0';

    while (i < maxLen - 1)
    {
        char ch = get();
        if (Commands != startCmds) break; // EOL consumed — line complete
        buf[i]     = ch;
        buf[i + 1] = '\0';
        i++;
    }

    // If we hit maxLen before the EOL, drain the remainder of the line so the
    // buffer stays consistent.  This is the only situation where a drain pass
    // is needed.
    while (Commands == startCmds) get();

    return i;
}

/**
 * @brief Return the character count of the next complete line (excluding EOL).
 *
 * Scans forward from Head without consuming any data.
 *
 * @return Count of characters before the first EOL, or -1 if no complete
 *         line is present.
 */
int ringBuffer::lineLength(void) const
{
    if (Commands == 0) return -1;

    for (int i = 0; i < Count; i++)
    {
        if (isEOL(Buffer[(Head + i) % Size])) return i;
    }

    // Should not be reached when Commands > 0, but guard against state
    // corruption rather than looping forever.
    return -1;
}

/**
 * @brief Count the delimiter-separated tokens in the next complete line.
 *
 * Scans forward from Head.  Each delimiter adds one token.  An initial count
 * of 1 handles the case of a line with no delimiters (one token).
 *
 * @return Token count >= 1, or -1 if no complete line is available.
 */
int ringBuffer::tokensInLine(char delim) const
{
    if (Commands == 0) return -1;

    int tokens = 1;
    for (int i = 0; i < Count; i++)
    {
        char ch = Buffer[(Head + i) % Size];
        if (isEOL(ch))    return tokens; // end of this line
        if (ch == delim)  tokens++;
    }

    return tokens; // reached end of buffer — Commands > 0 should prevent this
}

/**
 * @brief Discard all characters in the next complete line, including its EOL.
 *
 * @return Number of characters removed, or -1 if no complete line is available.
 */
int ringBuffer::clearLine(void)
{
    if (Commands == 0) return -1;

    int startCmds = Commands;
    int i         = 0;
    while (Commands == startCmds)
    {
        get();
        i++;
    }
    return i;
}

// =============================================================================
//  Token-oriented operations
// =============================================================================

/**
 * @brief Return true if a token boundary is visible in the buffer.
 *
 * Returns true immediately if a complete line is present (lines() > 0).
 * Otherwise scans the buffered data for @p delim.
 *
 * NOTE: the scan is bounded to Count bytes, so it may look into a partial
 * second line.  See the header comment for the design caveat about pairing
 * isToken() with getToken().
 */
bool ringBuffer::isToken(char delim) const
{
    if (Commands > 0) return true;
    for (int i = 0; i < Count; i++)
        if (Buffer[(Head + i) % Size] == delim) return true;
    return false;
}

/**
 * @brief Read and consume the next delimiter-separated token.
 *
 * Reads characters until @p delim or an EOL is consumed, or until
 * @p maxLen - 1 characters have been written.  The delimiter/EOL character
 * is consumed but not written to @p buf.
 *
 * If @p buf is NULL the characters are consumed and discarded (skip mode).
 *
 * The output is always NUL-terminated when buf != NULL.
 *
 * @return Number of characters written (excluding NUL), or -1 if no complete
 *         line is available.
 */
int ringBuffer::getToken(char *buf, char delim, int maxLen)
{
    if (Commands == 0) return -1;

    int startCmds = Commands;
    int i         = 0;

    if (buf != NULL) buf[0] = '\0';

    while (i < maxLen - 1)
    {
        char ch = get();

        // EOL consumed — token (and line) ended.
        if (Commands != startCmds) break;

        // Delimiter consumed — token ended, line continues.
        if (ch == delim) break;

        if (buf != NULL)
        {
            buf[i]     = ch;
            buf[i + 1] = '\0';
        }
        i++;
    }

    return i;
}

/**
 * @brief Return the byte length of the next token without consuming data.
 *
 * Scans forward from Head looking for @p delim or any EOL character.
 *
 * @return Number of characters before the first delimiter or EOL, or -1 if
 *         neither is found (token is incomplete / no line terminator present).
 */
int ringBuffer::tokenLength(char delim) const
{
    for (int i = 0; i < Count; i++)
    {
        char ch = Buffer[(Head + i) % Size];
        if (ch == delim)  return i;
        if (isEOL(ch))    return i;
    }
    return -1; // no delimiter or EOL found — token incomplete
}
