#include "charAllocate.h"

// =============================================================================
//  charAllocate — fixed-size heap allocator
//
//  Buffer layout (each row is one block):
//
//    Offset   Bytes  Contents
//    ------   -----  --------
//    ptr+0      1    Header low byte  (payload size bits 7-0, or'd with 0x80
//                    in bit 7 of the HIGH byte when allocated)
//    ptr+1      1    Header high byte (bit 7 = alloc flag; bits 6-0 = size[14:8])
//    ptr+2    size   Payload (returned to the caller)
//
//  A block whose payload size is 0 and whose alloc flag is clear is a
//  degenerate free block that can exist after freeing a minimum-size
//  allocation.  defrag() will coalesce it with its neighbour.
// =============================================================================

// -----------------------------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------------------------

charAllocate::charAllocate(int caSize)
    : size(caSize), buffer(NULL)
{
    // Require at least a header plus one payload byte.
    if (caSize < HEADER_SIZE + 1) return;

    buffer = (char *)malloc((size_t)caSize);
    if (buffer == NULL) return; // malloc failed; buffer stays NULL, all methods guard against this

    // Initialise the entire buffer as one large free block.
    // Payload size = total buffer size minus the 2-byte header.
    writeHeader(0, (uint16_t)(caSize - HEADER_SIZE));
}

charAllocate::~charAllocate()
{
    ::free(buffer); // call C standard free, not our member free(char*)
    buffer = NULL;
    size   = 0;
}

// -----------------------------------------------------------------------------
// clear — reset to a single free block (all previous allocations are invalid)
// -----------------------------------------------------------------------------

void charAllocate::clear(void)
{
    if (buffer == NULL) return;
    writeHeader(0, (uint16_t)(size - HEADER_SIZE));
}

// -----------------------------------------------------------------------------
// allocate — find the first free block large enough and carve out num bytes
// -----------------------------------------------------------------------------

char *charAllocate::allocate(int num)
{
    if (buffer == NULL || num <= 0) return NULL;

    int      ptr    = 0;
    uint16_t header = readHeader(ptr);

    // Walk the block list looking for a free block whose payload is >= num.
    while ((header & ALLOC_FLAG) || ((header & SIZE_MASK) < (uint16_t)num))
    {
        ptr += (int)(header & SIZE_MASK) + HEADER_SIZE;
        if (ptr >= size) return NULL; // reached end of buffer with no suitable block
        header = readHeader(ptr);
    }

    // Found a suitable free block.  Decide whether to split it.
    int blkSize = (int)(header & SIZE_MASK);

    // Only split if the remainder can hold a header PLUS at least one byte of
    // payload (i.e. remainder >= HEADER_SIZE + 1 = 3).  Otherwise hand the
    // entire block to the caller to avoid a zero- or negative-size free shard.
    if ((blkSize - num) < (HEADER_SIZE + 1))
    {
        num = blkSize; // absorb the extra bytes into this allocation
    }

    // Mark this block as allocated.
    writeHeader(ptr, (uint16_t)(num) | ALLOC_FLAG);
    char *payload = &buffer[ptr + HEADER_SIZE];

    // If there is room left over, write a new free-block header for it.
    if (blkSize > num)
    {
        int      splitPtr  = ptr + num + HEADER_SIZE;
        uint16_t splitSize = (uint16_t)(blkSize - num - HEADER_SIZE);
        writeHeader(splitPtr, splitSize); // no ALLOC_FLAG → free block
    }

    return payload;
}

// -----------------------------------------------------------------------------
// free — mark a previously allocated block as free
// -----------------------------------------------------------------------------

void charAllocate::free(char *buf)
{
    if (buffer == NULL || buf == NULL) return;

    // The header sits HEADER_SIZE (2) bytes before the payload pointer.
    // Use signed arithmetic to allow the bounds check below to work correctly
    // with ptrdiff_t rather than uint32_t (which can never be negative).
    ptrdiff_t offset = (char *)buf - buffer; // byte offset of payload from buffer start
    int       ptr    = (int)(offset - HEADER_SIZE); // byte offset of header

    // Validate that ptr points inside the buffer and is not the very start
    // (which would mean the caller passed buffer itself, not a payload pointer).
    if (ptr < 0 || ptr >= size - HEADER_SIZE) return;

    // Check that the alloc flag is actually set before clearing it, to guard
    // against double-free or freeing a pointer that was never allocated.
    if ((buffer[ptr + 1] & (char)(ALLOC_FLAG >> 8)) == 0) return; // already free

    // Clear the alloc flag in the high byte of the header to mark as free.
    buffer[ptr + 1] &= (char)~(ALLOC_FLAG >> 8);
}

// -----------------------------------------------------------------------------
// available — size of the largest single free block (payload bytes only)
// -----------------------------------------------------------------------------

int charAllocate::available(void) const
{
    if (buffer == NULL) return 0;

    int      ptr     = 0;
    int      largest = 0;
    uint16_t header  = readHeader(ptr);

    while (true)
    {
        if ((header & ALLOC_FLAG) == 0)
        {
            // Free block — track the largest payload seen.
            int payloadSize = (int)(header & SIZE_MASK);
            if (payloadSize > largest) largest = payloadSize;
        }

        ptr += (int)(header & SIZE_MASK) + HEADER_SIZE;
        if (ptr >= size) break; // past end of buffer
        header = readHeader(ptr);
    }

    return largest;
}

// -----------------------------------------------------------------------------
// defrag — coalesce adjacent free blocks into a single larger free block
// -----------------------------------------------------------------------------

void charAllocate::defrag(void)
{
    if (buffer == NULL) return;

    int      ptr    = 0;
    uint16_t header = readHeader(ptr);

    while (ptr < size)
    {
        if (header & ALLOC_FLAG)
        {
            // Allocated block — skip over it.
            ptr += (int)(header & SIZE_MASK) + HEADER_SIZE;
            if (ptr >= size) break;
            header = readHeader(ptr);
        }
        else
        {
            // Free block — try to merge with the immediately following block.
            int nextPtr = ptr + (int)(header & SIZE_MASK) + HEADER_SIZE;
            if (nextPtr >= size) break; // this is the last block; nothing to merge

            uint16_t nextHeader = readHeader(nextPtr);

            if ((nextHeader & ALLOC_FLAG) == 0)
            {
                // Next block is also free: merge by adding its payload size
                // plus its header size into the current block's payload size.
                // Do NOT advance ptr so that we re-examine this (now larger)
                // block against the block that follows the one we just merged,
                // enabling coalescing of 3+ consecutive free blocks in one pass.
                header = (uint16_t)((header & SIZE_MASK)
                                    + (nextHeader & SIZE_MASK)
                                    + HEADER_SIZE);
                writeHeader(ptr, header); // still free (no ALLOC_FLAG)
            }
            else
            {
                // Next block is allocated — advance past the current free block.
                ptr += (int)(header & SIZE_MASK) + HEADER_SIZE;
                if (ptr >= size) break;
                header = readHeader(ptr);
            }
        }
    }
}
