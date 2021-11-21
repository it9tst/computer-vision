/** 
 * @file    kImageUtils.x.h
 * @brief   Save and load functions for the kImage class.  
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_IMAGE_UTILS_X_H
#define K_API_IMAGE_UTILS_X_H

#include <kApi/kApiDef.h>

kFx(kStatus) xkImageUtils_Save(kImage image, const kChar* fileName, kAlloc allocator = kNULL);

kFx(kStatus) xkImageUtils_Load(kImage* image, const kChar* fileName, kAlloc allocator = kNULL);

#endif
