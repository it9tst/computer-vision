/**
 * @file    loadepng_ex.h
 * @brief   Additions for the LodePNG library 
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef K_API_EXTERN_LODEPNG_EX_H
#define K_API_EXTERN_LODEPNG_EX_H

#include <stdlib.h>

#define LODEPNG_NO_COMPILE_ALLOCATORS
#define LODEPNG_NO_COMPILE_DISK
#define LODEPNG_NO_COMPILE_ANCILLARY_CHUNKS
#define LODEPNG_NO_COMPILE_ERROR_TEXT
#define LODEPNG_NO_COMPILE_CPP

void* lodepng_malloc(size_t size);
void lodepng_free(void* ptr);
void* lodepng_realloc(void* oldMemory, size_t newSize);

#endif

