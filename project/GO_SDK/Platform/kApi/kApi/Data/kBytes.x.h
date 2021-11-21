/** 
 * @file    kBytes.x.h
 *
 * @internal
 * Copyright (C) 2012-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BYTES_X_H
#define K_API_BYTES_X_H

#define xkBYTES_CAPACITY          (1024)

/* 
* Private methods. 
*/

kFx(kStatus) xkBytes_AddTypes(kAssembly assembly); 
kFx(kStatus) xkBytes_Register(kAssembly assembly, const kChar* name); 

kFx(kBool) xkBytes_VEquals(kType type, const void* value, const void* other);
kFx(kSize) xkBytes_VHashCode(kType type, const void* value);

kFx(kStatus) xkBytes_Write(kType type, const void* values, kSize count, kSerializer serializer);      
kFx(kStatus) xkBytes_Read(kType type, void* values, kSize count, kSerializer serializer);     

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Define custom types rather than relying on kBytes.
#define kAddBytes()     xkBytes_AddTypes(output); 

//[Deprecated] Define custom types rather than relying on kBytes.
kFx(kType) kBytes_GetType(kSize size); 

#endif
