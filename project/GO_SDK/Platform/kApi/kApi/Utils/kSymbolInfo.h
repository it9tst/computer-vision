/** 
 * @file    kSymbolInfo.h
 * @brief   Declares the kSymbolInfo class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_SYMBOL_TABLE_H
#define K_API_SYMBOL_TABLE_H

#include <kApi/kApiDef.h>
#include <kApi/Utils/kSymbolInfo.x.h>

/**
 * @class   kSymbolInfo
 * @extends kObject
 * @ingroup kApi-Utils
 * @brief   Collection of static functions related to debug symbol information.
 */
//typedef kObject kSymbolInfo;            --forward-declared in kApiDef.x.h 

/** 
 * Describes the function at the specified address.
 * 
 * This method may not be supported on all platforms.
 *
 * @public                  @memberof kSymbolInfo
 * @param   function        Function call address. 
 * @param   description     Receives function call description; use kObject_Destroy to free the string.
 * @return                  Operation status (kERROR_UNIMPLEMENTED if not supported).
 */
kFx(kStatus) kSymbolInfo_DescribeFunction(kPointer function, kString* description); 

#endif
