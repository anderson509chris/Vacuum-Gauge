#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

/**
 * @file RingBuffer.h
 * @brief A character ring buffer with line- and token-oriented parsing helpers.
 *
 * Design overview
 * ---------------
 * Characters are added one at a time via put().  The buffer tracks how many
 * complete "lines" are present — a line ends at any character found in the
 * EOLchars string (default: ";\n\r").  Characters in the Ignore string are
 * silently dropped by put() before they enter the buffer.
 *
 * The line count (Commands) drives higher-level parsing: getLine(),
 * getToken(), tokensInLine(), lineLength(), and clearLine() all require at
 * least one complete line before operating.
 *
 * Sentinel values
 * ---------------
 * get() and peek() return 0xFF when the buffer is empty.  0xFF is used only
 * as a sentinel — it is not a valid payload character in the ASCII command
 * protocol this buffer is designed for.  Callers that need to handle binary
 * data should call empty() or count() before get()/peek() to avoid ambiguity.
 *
 * Thread / interrupt safety
 * -------------------------
 * This class is NOT interrupt-safe.  Do not call put() from an ISR while any
 * other method is executing on the same instance, or vice versa, without
 * external synchronisation.
 */

#include <Arduino.h>   // Corrected case (Linux FS is case-sensitive)

class ringBuffer
{
public:
    // -----------------------------------------------------------------------
    // Construction / destruction
    // -----------------------------------------------------------------------

    /**
     * @param rbSize  Total byte capacity of the ring buffer.  Must be > 0.
     *                Default is 1024.
     */
    explicit ringBuffer(int rbSize = 1024);

    ~ringBuffer();

    // -----------------------------------------------------------------------
    // Basic state queries
    // -----------------------------------------------------------------------

    /** @brief Return true if the buffer contains no characters. */
    bool empty(void) const { return Count == 0; }

    /** @brief Return the number of characters currently in the buffer. */
    int  count(void) const;

    /** @brief Return the number of complete lines (terminated by an EOL char). */
    int  lines(void) const;

    // -----------------------------------------------------------------------
    // Single-character operations
    // -----------------------------------------------------------------------

    /**
     * @brief Read and remove the next character from the buffer.
     * @return The character, or 0xFF if the buffer is empty.
     */
    char get(void);

    /**
     * @brief Return the next character without removing it.
     * @return The character, or 0xFF if the buffer is empty.
     */
    char peek(void) const;

    /**
     * @brief Add a character to the buffer.
     *
     * Characters present in Ignore are silently dropped before insertion.
     * EOLchars characters increment the internal line counter.
     *
     * @return  0 on success; -1 if the buffer is full.
     */
    int put(char ch);

    // -----------------------------------------------------------------------
    // Buffer management
    // -----------------------------------------------------------------------

    /**
     * @brief Discard all buffered characters and reset all counters.
     *
     * Any data in the buffer is lost.  EOLchars and Ignore are not changed.
     */
    void clear(void);

    // -----------------------------------------------------------------------
    // Line-oriented operations  (require lines() > 0)
    // -----------------------------------------------------------------------

    /**
     * @brief Copy the next complete line into @p buf and consume it.
     *
     * Reads characters until an EOL character is consumed, or @p maxLen - 1
     * characters have been copied.  The EOL character is consumed but not
     * copied.  The output string is always NUL-terminated.
     *
     * @param buf     Destination buffer.  Must be at least @p maxLen bytes.
     * @param maxLen  Maximum bytes to write including the NUL terminator.
     *                Default is 128.
     * @return Number of characters copied (excluding NUL), or -1 if no
     *         complete line is available.
     */
    int getLine(char *buf, int maxLen = 128);

    /**
     * @brief Return the number of characters in the next complete line,
     *        not including the EOL terminator.
     * @return Character count, or -1 if no complete line is available.
     */
    int lineLength(void) const;

    /**
     * @brief Count the number of delimiter-separated tokens in the next line.
     *
     * A line with no delimiters contains one token.  The EOL character is not
     * counted as a token.
     *
     * @param delim  Field delimiter character (e.g. ',').
     * @return Token count >= 1, or -1 if no complete line is available.
     */
    int tokensInLine(char delim) const;

    /**
     * @brief Discard all characters up to and including the next EOL.
     * @return Number of characters removed, or -1 if no complete line is
     *         available.
     */
    int clearLine(void);

    // -----------------------------------------------------------------------
    // Token-oriented operations  (require lines() > 0)
    // -----------------------------------------------------------------------

    /**
     * @brief Return true if a delimiter-separated token is available.
     *
     * Returns true when either a complete line exists (lines() > 0) or when
     * a delimiter character is found anywhere in the currently buffered data.
     *
     * NOTE: when no complete line exists, isToken() may find a delimiter on a
     * partial line.  The companion function getToken() requires a complete
     * line; pairing isToken()/getToken() can produce a false-ready signal on
     * partial data.  Prefer checking lines() > 0 directly before getToken().
     *
     * @param delim  Delimiter to search for.
     * @return true if a token boundary is visible in the buffer.
     */
    bool isToken(char delim) const;

    /**
     * @brief Read and consume the next delimiter-separated token.
     *
     * Reads characters until @p delim or an EOL character is reached, or
     * @p maxLen - 1 characters have been copied.  The delimiter is consumed
     * but not copied.  The output string is always NUL-terminated.
     *
     * If @p buf is NULL the token is consumed and discarded (useful for
     * skipping fields).
     *
     * @param buf     Destination buffer, or NULL to discard.
     * @param delim   Field delimiter character.
     * @param maxLen  Maximum bytes to write including the NUL terminator.
     *                Default is 64.
     * @return Number of characters copied (excluding NUL), or -1 if no
     *         complete line is available.
     */
    int getToken(char *buf, char delim, int maxLen = 64);

    /**
     * @brief Return the number of characters in the next token without
     *        consuming anything.
     *
     * Scans forward from Head until a delimiter or EOL character is found.
     *
     * @param delim  Delimiter to search for.
     * @return Number of characters before the delimiter/EOL, or -1 if no
     *         delimiter or EOL is found within the buffered data (i.e. the
     *         token is incomplete).
     */
    int tokenLength(char delim) const;

    // -----------------------------------------------------------------------
    // Configuration  (set before use; do not change while data is in buffer)
    // -----------------------------------------------------------------------

    /**
     * @brief Characters that mark the end of a line (default ";\n\r").
     *
     * WARNING: modifying this string while the buffer contains data will
     * corrupt the internal line count (Commands).  Always call clear() first.
     */
    String EOLchars;

    /**
     * @brief Characters silently dropped by put() before insertion (default "").
     *
     * Useful for stripping carriage returns or other noise characters.
     * Safe to modify at any time — it only affects future put() calls.
     */
    String Ignore;

private:
    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /** Return true if @p ch is an EOL character. */
    bool isEOL(char ch) const;

    /** Return true if @p ch is in the Ignore set. */
    bool isIgnored(char ch) const;

    // -----------------------------------------------------------------------
    // Storage
    // -----------------------------------------------------------------------

    char *Buffer;   ///< Heap-allocated circular storage array
    int   Size;     ///< Total capacity in bytes
    int   Tail;     ///< Write index (next character goes here)
    int   Head;     ///< Read index (next character comes from here)
    int   Count;    ///< Number of characters currently in the buffer
    int   Commands; ///< Number of complete lines (EOL-terminated) in the buffer
};

#endif // RINGBUFFER_H_
