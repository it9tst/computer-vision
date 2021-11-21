/**
 * @file    loadepng_ex.cpp
 * @brief   Additional functions for the LodePNG library 
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include "lodepng_ex.h"

#include <kApi/Io/kMemory.h>

#define LODEPNG_MALLOC_EXTRA kALIGN_ANY_SIZE

void* lodepng_malloc(size_t size)
{
    void* allocation = kNULL;

    const auto blockSize = size + LODEPNG_MALLOC_EXTRA;
    if (kMemAlloc(blockSize, &allocation) != kOK)
    {
        return 0;
    }

    kPointer_WriteAs(allocation, size, kSize);
    return kPointer_ByteOffset(allocation, LODEPNG_MALLOC_EXTRA);
}

void lodepng_free(void* ptr)
{
    if (kIsNull(ptr))
        return;

    const auto allocation = (kByte*)(ptr) - LODEPNG_MALLOC_EXTRA;
    kMemFree(allocation);
}

void* lodepng_realloc(void* oldMemory, size_t newSize) 
{
    if (newSize == 0)
        return 0;

    if (kIsNull(oldMemory))
        return lodepng_malloc(newSize);

    const auto oldAllocation = (kByte*)(oldMemory) - LODEPNG_MALLOC_EXTRA;
    const kSize oldSize = kPointer_ReadAs(oldAllocation, kSize);

    const auto allocated = lodepng_malloc(newSize);

    if (!kIsNull(allocated))
    {
        if (oldSize <  newSize)
            kMemCopy(allocated, oldMemory, oldSize);
        else
            kMemCopy(allocated, oldMemory, newSize);
    }

    lodepng_free(oldMemory);

    return allocated;
}