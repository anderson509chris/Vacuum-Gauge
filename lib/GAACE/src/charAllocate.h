#ifndef CHARALLOCATE_H_
#define CHARALLOCATE_H_

#include <Arduino.h>   // Corrected case (Linux file systems are case-sensitive)
#include <stdint.h>
#include <stdlib.h>    // for malloc / free

/**
 * @brief A fixed-size heap allocator for char arrays.
 *
 * Manages a single contiguous buffer using a singly-linked free-list of
 * variable-length blocks.  Each block is preceded by a 2-byte little-endian
 * header whose layout is:
 *
 *   Bit 15   : 1 = block is allocated, 0 = block is free
 *   Bits 14-0: payload size in bytes (i.e. NOT counting the 2-byte header)
 *
 * The buffer is initialised as one large free block that spans the entire
 * allocation minus the 2-byte header.
 *
 * Example (512-byte buffer):
 *   buffer[0..1] = 0x01FE  (free, payload = 510 bytes)
 *   buffer[2..511]          (510 bytes of usable payload)
 *
 * @note This class defines its own free(char*) method.  Internally it calls
 *       the C standard ::free() via the destructor, so the names do not
 *       collide at run-time, but be aware of the shadowing when reading the
 *       code.
 */
class charAllocate
{
public:
    /**
     * @param caSize Total number of bytes to allocate for the internal buffer,
     *               including the 2-byte header of the initial free block.
     *               Must be at least 3.
     */
    explicit charAllocate(int caSize = 512);

    ~charAllocate();

    /**
     * @brief  Allocate a block of at least @p num bytes from the buffer.
     * @return Pointer to the usable payload, or NULL if no suitable free
     *         block exists.
     */
    char *allocate(int num);

    /**
     * @brief  Mark the block whose payload starts at @p buf as free.
     * @note   @p buf must have been returned by allocate(); passing any other
     *         pointer is undefined behaviour.
     */
    void free(char *buf);

    /**
     * @brief  Reset the entire buffer to a single free block (discards all
     *         allocations).
     */
    void clear(void);

    /**
     * @brief  Merge adjacent free blocks to reduce fragmentation.
     *         Call this before allocate() when allocation fails but
     *         available() shows enough total free space.
     */
    void defrag(void);

    /**
     * @brief  Return the size of the largest single contiguous free block,
     *         in bytes (payload only, header not counted).
     */
    int available(void) const;

private:
    // -----------------------------------------------------------------------
    // Header bit-field constants
    // -----------------------------------------------------------------------
    static const uint16_t ALLOC_FLAG  = 0x8000u; ///< Set when block is in use
    static const uint16_t SIZE_MASK   = 0x7FFFu; ///< Masks the payload-size field
    static const int      HEADER_SIZE = 2;        ///< Bytes consumed by one header

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /** Read the 2-byte little-endian header at buffer[ptr]. */
    inline uint16_t readHeader(int ptr) const
    {
        return (uint8_t)buffer[ptr] | ((uint8_t)buffer[ptr + 1] << 8);
    }

    /** Write a 2-byte little-endian header at buffer[ptr]. */
    inline void writeHeader(int ptr, uint16_t h)
    {
        buffer[ptr]     = (char)(h & 0xFFu);
        buffer[ptr + 1] = (char)((h >> 8) & 0xFFu);
    }

    // -----------------------------------------------------------------------
    // Data members
    // -----------------------------------------------------------------------
    int   size;    ///< Total byte length of buffer[]
    char *buffer;  ///< Raw storage managed by this allocator
};

#endif // CHARALLOCATE_H_
