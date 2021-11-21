/**
* @file    GoAlgorithm.h
* @brief   Contains various Algorithmic helper functions.
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_SDK_ALGORITHM_H
#define GO_SDK_ALGORITHM_H

#include <GoSdk/GoSdkDef.h>

/**
* Performs a demosaic operation on a bayer encoded image.
*
* @public                          @memberof GoAlgorithm
* @version                         Introduced in firmware 4.7.2.18
* @param    input                  kImage object.
* @param    output                 kImage pointer to store output at.
* @param    style                  GoDemosaicStyle.
* @param    allocator              kAlloc object (can be kNULL for fallback)
* @return                          Status of operation.
*/
GoFx(kStatus) GoAlgorithm_Demosaic(kImage input, kImage* output, GoDemosaicStyle style, kAlloc allocator);

#include <GoSdk/GoAlgorithm.x.h>

#endif
