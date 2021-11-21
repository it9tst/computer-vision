/**
 * @file    GoSerializer.h
 * @brief   Declares the GoSerializer class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SERIALIZER_H
#define GO_SDK_SERIALIZER_H

#include <GoSdk/GoSdkDef.h>
#include <kApi/Io/kSerializer.h>

#define GO_SERIALIZATION_FORMAT_NAME        "godat"        //format name, used for version info lookup
#define GO_SERIALIZATION_FORMAT_VERSION     "6.0.0.0"      //format version, used for version info lookup

// Need a level of indirection to stringify a number.
// Must only call GoSerializerTypeIdStr() and DO NOT call GoSerializerTypeIdStr_().
#define GoSerializerTypeIdStr_(TYPE_ID)     #TYPE_ID
#define GoSerializerTypeIdStr(TYPE_ID)      GoSerializerTypeIdStr_(TYPE_ID)

/**
 * @class   GoSerializer
 * @extends kSerializer
 * @ingroup GoSdk-Internal
 * @brief   Serializes/deserializes objects to/from Gocator Data Protocol.
 */
typedef kSerializer GoSerializer;

/**
 * Constructs a GoSerializer object.
 *
 * @public              @memberof GoSerializer
 * @version             Introduced in firmware 4.0.10.27
 * @param   serializer  Receives the constructed object.
 * @param   stream      Stream for reading or writing.
 * @param   allocator   Memory allocator (or kNULL for default).
 * @return              Operation status.
 */
GoFx(kStatus) GoSerializer_Construct(GoSerializer* serializer, kStream stream, kAlloc allocator);

#include <GoSdk/Internal/GoSerializer.x.h>

#endif
