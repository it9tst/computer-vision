/**
 * @file    GoDataTypes.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_TYPES_X_H
#define GO_SDK_DATA_TYPES_X_H

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kArray2.h>
#include <kApi/Data/kImage.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>
#include <kApi/Io/kDat6Serializer.h>
#include <kApi/Threads/kLock.h>

typedef struct GoDataMsgVTable
{
    kObjectVTable base;
    kStatus (kCall* VInit)(GoDataMsg msg, kType type, kAlloc allocator);
} GoDataMsgVTable;

typedef struct GoDataMsgClass
{
    kObjectClass base;
    GoDataMessageType typeId;
    GoDataStep streamStep;
    k32s streamStepId;
    k32s arrayedCount;
    k32s arrayedIndex;
} GoDataMsgClass;

kDeclareVirtualClassEx(Go, GoDataMsg, kObject)

GoFx(kStatus) GoDataMsg_Construct(GoDataMsg* msg, kAlloc allocator);
GoFx(kStatus) GoDataMsg_VInit(GoDataMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoDataMsg_Init(GoDataMsg msg, kType type, GoDataMessageType typeId, kAlloc alloc);
GoFx(kStatus) GoDataMsg_VInitClone(GoDataMsg msg, GoDataMsg source, kAlloc alloc);
GoFx(kStatus) GoDataMsg_VRelease(GoDataMsg msg);


GoFx(kStatus) GoDataMsg_ReadStreamStepAndId(GoDataMsg msg, kSerializer serializer);
GoFx(kStatus) GoDataMsg_ReadArrayedCountAndIndex(GoDataMsg msg, kSerializer serializer);
GoFx(kStatus) GoDataMsg_WriteArrayedCountAndIndex(GoDataMsg msg, kSerializer serializer);

/*
 * GoStamp
 */

kDeclareValueEx(Go, GoStamp, kValue)

/*
 * GoStampMsg
 */

#define GO_STAMP_MSG_STAMP_SIZE_1_56         (56)
#define GO_STAMP_MSG_STAMP_SIZE_1_64         (64)

typedef struct GoStampMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;            // stamp source
    kArray1 stamps;                 // stamps (kArray1<GoStamp>)
} GoStampMsgClass;

kDeclareClassEx(Go, GoStampMsg, GoDataMsg)

GoFx(kStatus) GoStampMsg_Construct(GoStampMsg* msg, kAlloc allocator);
GoFx(kStatus) GoStampMsg_VInit(GoStampMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoStampMsg_Allocate(GoStampMsg msg, kSize count);
GoFx(kStatus) GoStampMsg_VInitClone(GoStampMsg msg, GoStampMsg source, kAlloc alloc);
GoFx(kStatus) GoStampMsg_VRelease(GoStampMsg msg);
GoFx(kSize) GoStampMsg_VSize(GoStampMsg msg);
GoFx(kStatus) GoStampMsg_WriteV1(GoStampMsg msg, kSerializer serializer);
GoFx(kStatus) GoStampMsg_ReadV1(GoStampMsg msg, kSerializer serializer, kAlloc alloc);

#define GoStampMsg_SetContent_(D, V)        (xGoStampMsg_CastRaw(D)->stamps = (V), kOK)
#define GoStampMsg_Content_(D)              (xGoStampMsg_CastRaw(D)->stamps)
#define GoStampMsg_SetSource_(D, V)         (xGoStampMsg_CastRaw(D)->source = (V), kOK)


/*
 * GoVideoMsg
 */

typedef struct GoVideoMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                  // image source
    kSize cameraIndex;                    // camera index
    kImage content;                       // image content
    kSize exposureIndex;                  // exposure index
    k32u exposure;                      // exposure (nS)
    kBool isFlippedX;                   // indicates whether the image needs to be flipped on the horizontally to match the profile data
    kBool isFlippedY;                   // indicates whether the image needs to be flipped on the vertically to match the profile data
    kBool isTransposed;                 // indicates whether the image needs to be transposed to match the profile data
} GoVideoMsgClass;

kDeclareClassEx(Go, GoVideoMsg, GoDataMsg)

GoFx(kStatus) GoVideoMsg_Construct(GoVideoMsg* msg, kAlloc allocator);
GoFx(kStatus) GoVideoMsg_VInit(GoVideoMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoVideoMsg_VInitClone(GoVideoMsg msg, GoVideoMsg source, kAlloc alloc);
GoFx(kStatus) GoVideoMsg_Allocate(GoVideoMsg msg, kType pixelType, kSize width, kSize height);
GoFx(kStatus) GoVideoMsg_VRelease(GoVideoMsg msg);
GoFx(kSize) GoVideoMsg_VSize(GoVideoMsg msg);
GoFx(kStatus) GoVideoMsg_WriteV2(GoVideoMsg msg, kSerializer serializer);
GoFx(kStatus) GoVideoMsg_ReadV2(GoVideoMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoVideoMsg_ReadV2AttrProtoVer2(GoVideoMsg msg, kSerializer serializer);
GoFx(kStatus) GoVideoMsg_ReadTranspose(GoVideoMsg msg, kSerializer serializer);

#define GoVideoMsg_SetContent_(D, V)                (xGoVideoMsg_CastRaw(D)->content = (V), kOK)
#define GoVideoMsg_Content_(D)                      (xGoVideoMsg_CastRaw(D)->content)
#define GoVideoMsg_SetSource_(D, V)                 (xGoVideoMsg_CastRaw(D)->source = (V), kOK)
#define GoVideoMsg_CameraIndex_(D, V)               (xGoVideoMsg_CastRaw(D)->cameraIndex = (V), kOK)
#define GoVideoMsg_SetExposureIndex_(D, V)          (xGoVideoMsg_CastRaw(D)->exposureIndex = (V), kOK)
#define GoVideoMsg_SetExposure_(D, V)               (xGoVideoMsg_CastRaw(D)->exposure = (V), kOK)
#define GoVideoMsg_SetFlippedX_(D, V)               (xGoVideoMsg_CastRaw(D)->isFlippedX = (V), kOK)
#define GoVideoMsg_SetFlippedY_(D, V)               (xGoVideoMsg_CastRaw(D)->isFlippedY = (V), kOK)
#define GoVideoMsg_SetTransposed_(D, V)             (xGoVideoMsg_CastRaw(D)->isTransposed = (V), kOK)

/*
 * GoRangeMsg
 */

typedef struct GoRangeMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // Range source
    k32u zResolution;                      // z-resolution (nm)
    k32s zOffset;                          // z-offset (um)
    kArray1 content;                       // Range content (kArray1<k16s>)
    k32u exposure;                      // exposure (nS)
} GoRangeMsgClass;

kDeclareClassEx(Go, GoRangeMsg, GoDataMsg)

GoFx(kStatus) GoRangeMsg_Construct(GoRangeMsg* msg, kAlloc allocator);
GoFx(kStatus) GoRangeMsg_VInit(GoRangeMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoRangeMsg_VInitClone(GoRangeMsg msg, GoRangeMsg source, kAlloc alloc);
GoFx(kStatus) GoRangeMsg_Allocate(GoRangeMsg msg, kSize count);
GoFx(kStatus) GoRangeMsg_VRelease(GoRangeMsg msg);
GoFx(kSize) GoRangeMsg_VSize(GoRangeMsg msg);
GoFx(kStatus) GoRangeMsg_WriteV3(GoRangeMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeMsg_ReadV3(GoRangeMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoRangeMsg_ReadV3Attr(GoRangeMsg msg, kSerializer serializer, k32u* count);
GoFx(kStatus) GoRangeMsg_ReadV3AttrProtoVer2(GoRangeMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeMsg_ReadV3AttrProtoVer3(GoRangeMsg msg, kSerializer serializer);

#define GoRangeMsg_SetContent_(D, V)            (xGoRangeMsg_CastRaw(D)->content = (V), kOK)
#define GoRangeMsg_Content_(D)                  (xGoRangeMsg_CastRaw(D)->content)
#define GoRangeMsg_SetSource_(D, V)             (xGoRangeMsg_CastRaw(D)->source = (V), kOK)
#define GoRangeMsg_SetZResolution_(D, V)        (xGoRangeMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoRangeMsg_SetZOffset_(D, V)            (xGoRangeMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoRangeMsg_SetExposure_(D, V)           (xGoRangeMsg_CastRaw(D)->exposure = (V), kOK)

/*
 * GoRangeIntensityMsg
 */

typedef struct GoRangeIntensityMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // RangeIntensity source
    kArray1 content;                       // RangeIntensity content (kArray1<k8u>)
    k32u exposure;
} GoRangeIntensityMsgClass;

kDeclareClassEx(Go, GoRangeIntensityMsg, GoDataMsg)

GoFx(kStatus) GoRangeIntensityMsg_Construct(GoRangeIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoRangeIntensityMsg_VInit(GoRangeIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoRangeIntensityMsg_VInitClone(GoRangeIntensityMsg msg, GoRangeIntensityMsg source, kAlloc alloc);
GoFx(kStatus) GoRangeIntensityMsg_Allocate(GoRangeIntensityMsg msg, kSize count);
GoFx(kStatus) GoRangeIntensityMsg_VRelease(GoRangeIntensityMsg msg);
GoFx(kSize) GoRangeIntensityMsg_VSize(GoRangeIntensityMsg msg);
GoFx(kStatus) GoRangeIntensityMsg_WriteV4(GoRangeIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeIntensityMsg_ReadV4(GoRangeIntensityMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoRangeIntensityMsg_ReadV4Attr(GoRangeIntensityMsg msg, kSerializer serializer, k32u* count);
GoFx(kStatus) GoRangeIntensityMsg_ReadV4AttrProtoVer2(GoRangeIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoRangeIntensityMsg_ReadV4AttrProtoVer3(GoRangeIntensityMsg msg, kSerializer serializer);

#define GoRangeIntensityMsg_SetContent_(D, V)            (xGoRangeIntensityMsg_CastRaw(D)->content = (V), kOK)
#define GoRangeIntensityMsg_Content_(D)                  (xGoRangeIntensityMsg_CastRaw(D)->content)
#define GoRangeIntensityMsg_SetSource_(D, V)             (xGoRangeIntensityMsg_CastRaw(D)->source = (V), kOK)
#define GoRangeIntensityMsg_SetExposure_(D, V)           (xGoRangeIntensityMsg_CastRaw(D)->exposure = (V), kOK)

/*
 * GoProfilePointCloudMsg
 */

typedef struct GoProfilePointCloudMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // profile content (kArray2<kPoint16s>)
    k32u exposure;                        // exposure (nS)
    k8u cameraIndex;                    // camera index (0 - Front, 1 - Back)
} GoProfilePointCloudMsgClass;

kDeclareClassEx(Go, GoProfilePointCloudMsg, GoDataMsg)

GoFx(kStatus) GoProfilePointCloudMsg_Construct(GoProfilePointCloudMsg* msg, kAlloc allocator);
GoFx(kStatus) GoProfilePointCloudMsg_VInit(GoProfilePointCloudMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoProfilePointCloudMsg_VInitClone(GoProfilePointCloudMsg msg, GoProfilePointCloudMsg source, kAlloc alloc);
GoFx(kStatus) GoProfilePointCloudMsg_Allocate(GoProfilePointCloudMsg msg, kSize count, kSize width);
GoFx(kStatus) GoProfilePointCloudMsg_VRelease(GoProfilePointCloudMsg msg);
GoFx(kSize) GoProfilePointCloudMsg_VSize(GoProfilePointCloudMsg msg);
GoFx(kStatus) GoProfilePointCloudMsg_WriteV5(GoProfilePointCloudMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfilePointCloudMsg_ReadV5(GoProfilePointCloudMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoProfilePointCloudMsg_ReadV5Attr(GoProfilePointCloudMsg msg, kSerializer serializer, k32u* count, k32u* width);
GoFx(kStatus) GoProfilePointCloudMsg_ReadV5AttrProtoVer2(GoProfilePointCloudMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfilePointCloudMsg_ReadV5AttrProtoVer3(GoProfilePointCloudMsg msg, kSerializer serializer);

#define GoProfilePointCloudMsg_SetContent_(D, V)            (xGoProfilePointCloudMsg_CastRaw(D)->content = (V), kOK)
#define GoProfilePointCloudMsg_Content_(D)                  (xGoProfilePointCloudMsg_CastRaw(D)->content)
#define GoProfilePointCloudMsg_SetSource_(D, V)             (xGoProfilePointCloudMsg_CastRaw(D)->source = (V), kOK)
#define GoProfilePointCloudMsg_SetXResolution_(D, V)        (xGoProfilePointCloudMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoProfilePointCloudMsg_SetZResolution_(D, V)        (xGoProfilePointCloudMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoProfilePointCloudMsg_SetXOffset_(D, V)            (xGoProfilePointCloudMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoProfilePointCloudMsg_SetZOffset_(D, V)            (xGoProfilePointCloudMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoProfilePointCloudMsg_SetExposure_(D, V)           (xGoProfilePointCloudMsg_CastRaw(D)->exposure = (V), kOK)
#define GoProfilePointCloudMsg_SetCameraIndex_(D, V)        (xGoProfilePointCloudMsg_CastRaw(D)->cameraIndex = (V), kOK)


/*
 * GoUniformProfileMsg
 */

typedef struct GoUniformProfileMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // profile content (kArray2<k16s>)
    k32u exposure;                         // exposure (nS)
} GoUniformProfileMsgClass;

kDeclareClassEx(Go, GoUniformProfileMsg, GoDataMsg)

GoFx(kStatus) GoUniformProfileMsg_Construct(GoUniformProfileMsg* msg, kAlloc allocator);
GoFx(kStatus) GoUniformProfileMsg_VInit(GoUniformProfileMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoUniformProfileMsg_VInitClone(GoUniformProfileMsg msg, GoUniformProfileMsg source, kAlloc alloc);
GoFx(kStatus) GoUniformProfileMsg_Allocate(GoUniformProfileMsg msg, kSize count, kSize width);
GoFx(kStatus) GoUniformProfileMsg_VRelease(GoUniformProfileMsg msg);
GoFx(kSize) GoUniformProfileMsg_VSize(GoUniformProfileMsg msg);
GoFx(kStatus) GoUniformProfileMsg_WriteV6(GoUniformProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoUniformProfileMsg_ReadV6(GoUniformProfileMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoUniformProfileMsg_ReadV6Attr(GoUniformProfileMsg msg, kSerializer serializer, k32u* count, k32u* width);
GoFx(kStatus) GoUniformProfileMsg_ReadV6AttrProtoVer1(GoUniformProfileMsg msg, kSerializer serializer, k32u* countPtr, k32u* widthPtr);
GoFx(kStatus) GoUniformProfileMsg_ReadV6AttrProtoVer2(GoUniformProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoUniformProfileMsg_ReadV6AttrProtoVer3(GoUniformProfileMsg msg, kSerializer serializer);

#define GoUniformProfileMsg_SetContent_(D, V)            (xGoUniformProfileMsg_CastRaw(D)->content = (V), kOK)
#define GoUniformProfileMsg_Content_(D)                  (xGoUniformProfileMsg_CastRaw(D)->content)
#define GoUniformProfileMsg_SetSource_(D, V)             (xGoUniformProfileMsg_CastRaw(D)->source = (V), kOK)
#define GoUniformProfileMsg_SetXResolution_(D, V)        (xGoUniformProfileMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoUniformProfileMsg_SetZResolution_(D, V)        (xGoUniformProfileMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoUniformProfileMsg_SetXOffset_(D, V)            (xGoUniformProfileMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoUniformProfileMsg_SetZOffset_(D, V)            (xGoUniformProfileMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoUniformProfileMsg_SetExposure_(D, V)           (xGoUniformProfileMsg_CastRaw(D)->exposure = (V), kOK)

/**
* Deprecated: use base class GoDataMsg_StreamStep()
* Gets the source of the data stream.
*
* @public             @memberof GoUniformProfileMsg()
* @version            Introduced in firmware 4.8.1.70
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoUniformProfileMsg_StreamStep(GoUniformProfileMsg msg);

/**
* Deprecated: use base class GoDataMsg_StreamStepId()
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoUniformProfileMsg
* @version            Introduced in firmware 4.8.1.70
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoUniformProfileMsg_StreamStepId(GoUniformProfileMsg msg);


/*
 * GoProfileIntensityMsg
 */

typedef struct GoProfileIntensityMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // profile source
    k32u xResolution;                      // x-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    kArray2 content;                       // intensity content (kArray2<k8u>)
    k32u exposure;                         // exposure (nS)
    k8u cameraIndex;                       // camera index (0 - Front, 1 - Back)
} GoProfileIntensityMsgClass;

kDeclareClassEx(Go, GoProfileIntensityMsg, GoDataMsg)

GoFx(kStatus) GoProfileIntensityMsg_Construct(GoProfileIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoProfileIntensityMsg_VInit(GoProfileIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoProfileIntensityMsg_VInitClone(GoProfileIntensityMsg msg, GoProfileIntensityMsg source, kAlloc alloc);
GoFx(kStatus) GoProfileIntensityMsg_Allocate(GoProfileIntensityMsg msg, kSize height, kSize width);
GoFx(kStatus) GoProfileIntensityMsg_VRelease(GoProfileIntensityMsg msg);
GoFx(kSize) GoProfileIntensityMsg_VSize(GoProfileIntensityMsg msg);
GoFx(kStatus) GoProfileIntensityMsg_WriteV7(GoProfileIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfileIntensityMsg_ReadV7(GoProfileIntensityMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoProfileIntensityMsg_ReadV7Attr(GoProfileIntensityMsg msg, kSerializer serializer, k32u* count, k32u* width);
GoFx(kStatus) GoProfileIntensityMsg_ReadV7AttrProtoVer2(GoProfileIntensityMsg  msg, kSerializer serializer);
GoFx(kStatus) GoProfileIntensityMsg_ReadV7AttrProtoVer3(GoProfileIntensityMsg  msg, kSerializer serializer);

#define GoProfileIntensityMsg_SetContent_(D, V)            (xGoProfileIntensityMsg_CastRaw(D)->content = (V), kOK)
#define GoProfileIntensityMsg_Content_(D)                  (xGoProfileIntensityMsg_CastRaw(D)->content)
#define GoProfileIntensityMsg_SetSource_(D, V)             (xGoProfileIntensityMsg_CastRaw(D)->source = (V), kOK)
#define GoProfileIntensityMsg_SetXResolution_(D, V)        (xGoProfileIntensityMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoProfileIntensityMsg_SetXOffset_(D, V)            (xGoProfileIntensityMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoProfileIntensityMsg_SetExposure_(D, V)           (xGoProfileIntensityMsg_CastRaw(D)->exposure = (V), kOK)
#define GoProfileIntensityMsg_SetCameraIndex_(D, V)        (xGoProfileIntensityMsg_CastRaw(D)->cameraIndex = (V), kOK)

/*
 * GoUniformSurfaceMsg
 */

typedef struct GoUniformSurfaceMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // surface source
    k32u xResolution;                      // x-resolution (nm)
    k32u yResolution;                      // y-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s yOffset;                          // y-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // surface content (kArray2<k16s>)
    k32u exposure;                         // exposure (nS)
} GoUniformSurfaceMsgClass;

kDeclareClassEx(Go, GoUniformSurfaceMsg, GoDataMsg)

GoFx(kStatus) GoUniformSurfaceMsg_Construct(GoUniformSurfaceMsg* msg, kAlloc allocator);
GoFx(kStatus) GoUniformSurfaceMsg_VInit(GoUniformSurfaceMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoUniformSurfaceMsg_VInitClone(GoUniformSurfaceMsg msg, GoUniformSurfaceMsg source, kAlloc alloc);
GoFx(kStatus) GoUniformSurfaceMsg_Allocate(GoUniformSurfaceMsg msg, kSize length, kSize width);
GoFx(kStatus) GoUniformSurfaceMsg_VRelease(GoUniformSurfaceMsg msg);
GoFx(kSize) GoUniformSurfaceMsg_VSize(GoUniformSurfaceMsg msg);
GoFx(kStatus) GoUniformSurfaceMsg_WriteV8(GoUniformSurfaceMsg msg, kSerializer serializer);
GoFx(kStatus) GoUniformSurfaceMsg_ReadV8(GoUniformSurfaceMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoUniformSurfaceMsg_ReadV8Attr(GoUniformSurfaceMsg msg, kSerializer serializer, k32u* length, k32u* width);
GoFx(kStatus) GoUniformSurfaceMsg_ReadV8AttrProtoVer1(GoUniformSurfaceMsg msg, kSerializer serializer, k32u* lenPtr, k32u* widthPtr);
GoFx(kStatus) GoUniformSurfaceMsg_ReadV8AttrProtoVer2(GoUniformSurfaceMsg msg, kSerializer serializer);
GoFx(kStatus) GoUniformSurfaceMsg_ReadV8AttrProtoVer3(GoUniformSurfaceMsg msg, kSerializer serializer);

#define GoUniformSurfaceMsg_SetContent_(D, V)            (xGoUniformSurfaceMsg_CastRaw(D)->content = (V), kOK)
#define GoUniformSurfaceMsg_Content_(D)                  (xGoUniformSurfaceMsg_CastRaw(D)->content)
#define GoUniformSurfaceMsg_SetSource_(D, V)             (xGoUniformSurfaceMsg_CastRaw(D)->source = (V), kOK)
#define GoUniformSurfaceMsg_SetXResolution_(D, V)        (xGoUniformSurfaceMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoUniformSurfaceMsg_SetYResolution_(D, V)        (xGoUniformSurfaceMsg_CastRaw(D)->yResolution = (V), kOK)
#define GoUniformSurfaceMsg_SetZResolution_(D, V)        (xGoUniformSurfaceMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoUniformSurfaceMsg_SetXOffset_(D, V)            (xGoUniformSurfaceMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoUniformSurfaceMsg_SetYOffset_(D, V)            (xGoUniformSurfaceMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoUniformSurfaceMsg_SetZOffset_(D, V)            (xGoUniformSurfaceMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoUniformSurfaceMsg_SetExposure_(D, V)           (xGoUniformSurfaceMsg_CastRaw(D)->exposure = (V), kOK)

/**
* Deprecated: use base class GoDataMsg_StreamStep()
* Gets the source of the data stream.
*
* @public             @memberof GoUniformSurfaceMsg
* @version            Introduced in firmware 4.8.1.70* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoUniformSurfaceMsg_StreamStep(GoUniformSurfaceMsg msg);

/**
* Deprecated: use base class GoDataMsg_StreamStepId()
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoUniformSurfaceMsg
* @version            Introduced in firmware 4.8.1.70* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoUniformSurfaceMsg_StreamStepId(GoUniformSurfaceMsg msg);

/*
* GoSurfacePointCloudMsg
*/

typedef struct GoSurfacePointCloudMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // surface source
    k32u xResolution;                      // x-resolution (nm)
    k32u yResolution;                      // y-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s yOffset;                          // y-offset (um)
    k32s zOffset;                          // z-offset (um)
    kArray2 content;                       // surface content (kArray2<k16s>)
    k32u exposure;                         // exposure (nS)
    kBool isAdjacent;
} GoSurfacePointCloudMsgClass;

kDeclareClassEx(Go, GoSurfacePointCloudMsg, GoDataMsg)

GoFx(kStatus) GoSurfacePointCloudMsg_Construct(GoSurfacePointCloudMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSurfacePointCloudMsg_VInit(GoSurfacePointCloudMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSurfacePointCloudMsg_VInitClone(GoSurfacePointCloudMsg msg, GoSurfacePointCloudMsg source, kAlloc alloc);
GoFx(kStatus) GoSurfacePointCloudMsg_Allocate(GoSurfacePointCloudMsg msg, kSize length, kSize width);
GoFx(kStatus) GoSurfacePointCloudMsg_VRelease(GoSurfacePointCloudMsg msg);
GoFx(kSize) GoSurfacePointCloudMsg_VSize(GoSurfacePointCloudMsg msg);
GoFx(kStatus) GoSurfacePointCloudMsg_ReadV8Attr(GoSurfacePointCloudMsg msg, kSerializer serializer, k32u* length, k32u* width);
GoFx(kStatus) GoSurfacePointCloudMsg_WriteV8(GoSurfacePointCloudMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfacePointCloudMsg_ReadV8(GoSurfacePointCloudMsg msg, kSerializer serializer, kAlloc alloc);

GoFx(kStatus) GoSurfacePointCloudMsg_ReadV8AttrProtoVer1(GoSurfacePointCloudMsg msg, kSerializer serializer, k32u* lenPtr, k32u* widthPtr);
GoFx(kStatus) GoSurfacePointCloudMsg_ReadV8AttrProtoVer2(GoSurfacePointCloudMsg msg, kSerializer serializer);

#define GoSurfacePointCloudMsg_SetContent_(D, V)            (xGoSurfacePointCloudMsg_CastRaw(D)->content = (V), kOK)
#define GoSurfacePointCloudMsg_Content_(D)                  (xGoSurfacePointCloudMsg_CastRaw(D)->content)
#define GoSurfacePointCloudMsg_SetSource_(D, V)             (xGoSurfacePointCloudMsg_CastRaw(D)->source = (V), kOK)
#define GoSurfacePointCloudMsg_SetXResolution_(D, V)        (xGoSurfacePointCloudMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoSurfacePointCloudMsg_SetYResolution_(D, V)        (xGoSurfacePointCloudMsg_CastRaw(D)->yResolution = (V), kOK)
#define GoSurfacePointCloudMsg_SetZResolution_(D, V)        (xGoSurfacePointCloudMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoSurfacePointCloudMsg_SetXOffset_(D, V)            (xGoSurfacePointCloudMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoSurfacePointCloudMsg_SetYOffset_(D, V)            (xGoSurfacePointCloudMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoSurfacePointCloudMsg_SetZOffset_(D, V)            (xGoSurfacePointCloudMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoSurfacePointCloudMsg_SetExposure_(D, V)           (xGoSurfacePointCloudMsg_CastRaw(D)->exposure = (V), kOK)
#define GoSurfacePointCloudMsg_SetIsAdjacent_(D, V)         (xGoSurfacePointCloudMsg_CastRaw(D)->isAdjacent = (V), kOK)

/**
* Deprecated: use base class GoDataMsg_StreamStep()
* Gets the source of the data stream.
*
* @public             @memberof GoSurfacePointCloudMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoSurfacePointCloudMsg_StreamStep(GoSurfacePointCloudMsg msg);

/**
* Deprecated: use base class GoDataMsg_StreamStepId()
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoSurfacePointCloudMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoSurfacePointCloudMsg_StreamStepId(GoSurfacePointCloudMsg msg);



/*
 * GoSurfaceIntensityMsg
 */

typedef struct GoSurfaceIntensityMsgClass
{
    GoDataMsgClass base;
    GoDataSource source;                   // surface intensity source
    k32u xResolution;                      // x-resolution (nm)
    k32u yResolution;                      // y-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s yOffset;                          // y-offset (um)
    kArray2 content;                       // intensity content (kArray2<k8u>)
    k32u exposure;                         // exposure (nS)
} GoSurfaceIntensityMsgClass;

kDeclareClassEx(Go, GoSurfaceIntensityMsg, GoDataMsg)

GoFx(kStatus) GoSurfaceIntensityMsg_Construct(GoSurfaceIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSurfaceIntensityMsg_VInit(GoSurfaceIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSurfaceIntensityMsg_VInitClone(GoSurfaceIntensityMsg msg, GoSurfaceIntensityMsg source, kAlloc alloc);
GoFx(kStatus) GoSurfaceIntensityMsg_Allocate(GoSurfaceIntensityMsg msg, kSize count, kSize width);
GoFx(kStatus) GoSurfaceIntensityMsg_VRelease(GoSurfaceIntensityMsg msg);
GoFx(kSize) GoSurfaceIntensityMsg_VSize(GoSurfaceIntensityMsg msg);
GoFx(kStatus) GoSurfaceIntensityMsg_WriteV9(GoSurfaceIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9(GoSurfaceIntensityMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9Attr(GoSurfaceIntensityMsg msg, kSerializer serializer, k32u* length, k32u* width);
GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9AttrProtoVer2(GoSurfaceIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceIntensityMsg_ReadV9AttrProtoVer3(GoSurfaceIntensityMsg msg, kSerializer serializer);

#define GoSurfaceIntensityMsg_SetContent_(D, V)            (xGoSurfaceIntensityMsg_CastRaw(D)->content = (V), kOK)
#define GoSurfaceIntensityMsg_Content_(D)                  (xGoSurfaceIntensityMsg_CastRaw(D)->content)
#define GoSurfaceIntensityMsg_SetSource_(D, V)             (xGoSurfaceIntensityMsg_CastRaw(D)->source = (V), kOK)
#define GoSurfaceIntensityMsg_SetXResolution_(D, V)        (xGoSurfaceIntensityMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoSurfaceIntensityMsg_SetYResolution_(D, V)        (xGoSurfaceIntensityMsg_CastRaw(D)->yResolution = (V), kOK)
#define GoSurfaceIntensityMsg_SetXOffset_(D, V)            (xGoSurfaceIntensityMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoSurfaceIntensityMsg_SetYOffset_(D, V)            (xGoSurfaceIntensityMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoSurfaceIntensityMsg_SetExposure_(D, V)           (xGoSurfaceIntensityMsg_CastRaw(D)->exposure = (V), kOK)

/*
 * GoSectionMsg
 */

typedef struct GoSectionMsgClass
{
    GoDataMsgClass base;
    k32u id;                               // Section ID
    GoDataSource source;                   // section source
    k32u xResolution;                      // x-resolution (nm)
    k32u zResolution;                      // z-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s zOffset;                          // z-offset (um)
    k32s xPose;                            // x-pose (um)
    k32s yPose;                            // y-pose (um)
    k32s anglePose;                        // angle-pose (um)
    kArray2 content;                       // section content (kArray2<k16s>)
    k32u exposure;                         // exposure (nS)
} GoSectionMsgClass;

kDeclareClassEx(Go, GoSectionMsg, GoDataMsg)

GoFx(kStatus) GoSectionMsg_Construct(GoSectionMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSectionMsg_VInit(GoSectionMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSectionMsg_VInitClone(GoSectionMsg msg, GoSectionMsg source, kAlloc alloc);
GoFx(kStatus) GoSectionMsg_Allocate(GoSectionMsg msg, kSize count, kSize width);
GoFx(kStatus) GoSectionMsg_VRelease(GoSectionMsg msg);
GoFx(kSize) GoSectionMsg_VSize(GoSectionMsg msg);
GoFx(kStatus) GoSectionMsg_WriteV20(GoSectionMsg msg, kSerializer serializer);
GoFx(kStatus) GoSectionMsg_ReadV20(GoSectionMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoSectionMsg_ReadV20Attr(GoSectionMsg msg, kSerializer serializer, k32u* count, k32u* width);
GoFx(kStatus) GoSectionMsg_ReadV20AttrProtoVer2(GoSectionMsg msg, kSerializer serializer);

#define GoSectionMsg_SetContent_(D, V)            (xGoSectionMsg_CastRaw(D)->content = (V), kOK)
#define GoSectionMsg_Content_(D)                  (xGoSectionMsg_CastRaw(D)->content)
#define GoSectionMsg_SetId_(D, V)                 (xGoSectionMsg_CastRaw(D)->id = (V), kOK)
#define GoSectionMsg_SetSource_(D, V)             (xGoSectionMsg_CastRaw(D)->source = (V), kOK)
#define GoSectionMsg_SetXResolution_(D, V)        (xGoSectionMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoSectionMsg_SetZResolution_(D, V)        (xGoSectionMsg_CastRaw(D)->zResolution = (V), kOK)
#define GoSectionMsg_SetXOffset_(D, V)            (xGoSectionMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoSectionMsg_SetZOffset_(D, V)            (xGoSectionMsg_CastRaw(D)->zOffset = (V), kOK)
#define GoSectionMsg_SetXPose_(D, V)              (xGoSectionMsg_CastRaw(D)->xPose = (V), kOK)
#define GoSectionMsg_SetYPose_(D, V)              (xGoSectionMsg_CastRaw(D)->yPose = (V), kOK)
#define GoSectionMsg_SetAnglePose_(D, V)          (xGoSectionMsg_CastRaw(D)->anglePose = (V), kOK)
#define GoSectionMsg_SetExposure_(D, V)           (xGoSectionMsg_CastRaw(D)->exposure = (V), kOK)

/*
 * GoSectionIntensityMsg
 */

typedef struct GoSectionIntensityMsgClass
{
    GoDataMsgClass base;
    k32u id;                               // Section ID
    GoDataSource source;                   // section source
    k32u xResolution;                      // x-resolution (nm)
    k32s xOffset;                          // x-offset (um)
    k32s xPose;                            // x-pose (um)
    k32s yPose;                            // y-pose (um)
    k32s anglePose;                        // angle-pose (um)
    kArray2 content;                       // section content (kArray2<k16s>)
    k32u exposure;                        // exposure (nS)
} GoSectionIntensityMsgClass;

kDeclareClassEx(Go, GoSectionIntensityMsg, GoDataMsg)

GoFx(kStatus) GoSectionIntensityMsg_Construct(GoSectionIntensityMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSectionIntensityMsg_VInit(GoSectionIntensityMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSectionIntensityMsg_VInitClone(GoSectionIntensityMsg msg, GoSectionIntensityMsg source, kAlloc alloc);
GoFx(kStatus) GoSectionIntensityMsg_Allocate(GoSectionIntensityMsg msg, kSize count, kSize width);
GoFx(kStatus) GoSectionIntensityMsg_VRelease(GoSectionIntensityMsg msg);
GoFx(kSize) GoSectionIntensityMsg_VSize(GoSectionIntensityMsg msg);
GoFx(kStatus) GoSectionIntensityMsg_WriteV21(GoSectionIntensityMsg msg, kSerializer serializer);
GoFx(kStatus) GoSectionIntensityMsg_ReadV21(GoSectionIntensityMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoSectionIntensityMsg_ReadV21Attr(GoSectionIntensityMsg msg, kSerializer serializer, k32u* count, k32u* width);
GoFx(kStatus) GoSectionIntensityMsg_ReadV21AttrProtoVer2(GoSectionIntensityMsg msg, kSerializer serializer);

#define GoSectionIntensityMsg_SetContent_(D, V)            (xGoSectionIntensityMsg_CastRaw(D)->content = (V), kOK)
#define GoSectionIntensityMsg_Content_(D)                  (xGoSectionIntensityMsg_CastRaw(D)->content)
#define GoSectionIntensityMsg_SetId_(D, V)                 (xGoSectionIntensityMsg_CastRaw(D)->id = (V), kOK)
#define GoSectionIntensityMsg_SetSource_(D, V)             (xGoSectionIntensityMsg_CastRaw(D)->source = (V), kOK)
#define GoSectionIntensityMsg_SetXResolution_(D, V)        (xGoSectionIntensityMsg_CastRaw(D)->xResolution = (V), kOK)
#define GoSectionIntensityMsg_SetXOffset_(D, V)            (xGoSectionIntensityMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoSectionIntensityMsg_SetXPose_(D, V)              (xGoSectionIntensityMsg_CastRaw(D)->xPose = (V), kOK)
#define GoSectionIntensityMsg_SetYPose_(D, V)              (xGoSectionIntensityMsg_CastRaw(D)->yPose = (V), kOK)
#define GoSectionIntensityMsg_SetAnglePose_(D, V)          (xGoSectionIntensityMsg_CastRaw(D)->anglePose = (V), kOK)
#define GoSectionIntensityMsg_SetExposure_(D, V)           (xGoSectionIntensityMsg_CastRaw(D)->exposure = (V), kOK)


/*
 * GoMeasurementData
 */

kDeclareValueEx(Go, GoMeasurementData, kValue)

/**
 * Deprecated: Gets the measurement data value.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement value.
 */
GoFx(k64f) GoMeasurementData_Value(GoMeasurementData data);

/**
 * Deprecated: Gets the measurement data decision.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement decision.
 */
GoFx(GoDecision) GoMeasurementData_Decision(GoMeasurementData data);

/**
 * Deprecated: Gets the measurement decision code.
 *
 * @public             @memberof GoMeasurementData
 * @version             Introduced in firmware 4.2.4.7
 * @param   data       Data object.
 * @return             Measurement decision code.
 */
GoFx(GoDecisionCode) GoMeasurementData_DecisionCode(GoMeasurementData data);

/*
 * GoMeasurementMsg
 */

typedef struct GoMeasurementMsgClass
{
    GoDataMsgClass base;
    k16u id;                        // measurement identifier
    kArray1 measurements;           // measurement (kArray1<GoMeasurementData>)
} GoMeasurementMsgClass;

kDeclareClassEx(Go, GoMeasurementMsg, GoDataMsg)

GoFx(kStatus) GoMeasurementMsg_Construct(GoMeasurementMsg* msg, kAlloc allocator);
GoFx(kStatus) GoMeasurementMsg_VInit(GoMeasurementMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoMeasurementMsg_Allocate(GoMeasurementMsg msg, kSize count);
GoFx(kStatus) GoMeasurementMsg_VInitClone(GoMeasurementMsg msg, GoMeasurementMsg source, kAlloc alloc);
GoFx(kStatus) GoMeasurementMsg_VRelease(GoMeasurementMsg msg);
GoFx(kSize) GoMeasurementMsg_VSize(GoMeasurementMsg msg);
GoFx(kStatus) GoMeasurementMsg_WriteV10(GoMeasurementMsg msg, kSerializer serializer);
GoFx(kStatus) GoMeasurementMsg_ReadV10(GoMeasurementMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoMeasurementMsg_WriteV31(GoMeasurementMsg msg, kSerializer serializer);
GoFx(kStatus) GoMeasurementMsg_ReadV31(GoMeasurementMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoMeasurementMsg_ReadV31Attr(GoMeasurementMsg msg, kSerializer serializer, k32u* count);

#define GoMeasurementMsg_SetContent_(D, V)        (xGoMeasurementMsg_CastRaw(D)->measurements = (V), kOK)
#define GoMeasurementMsg_Content_(D)              (xGoMeasurementMsg_CastRaw(D)->measurements)
#define GoMeasurementMsg_SetId_(D, V)             (xGoMeasurementMsg_CastRaw(D)->id = (V), kOK)


/*
 * GoCalMsg
 */

typedef struct GoAlignMsgClass
{
    GoDataMsgClass base;
    k32u opId;
    GoAlignmentStatus status;                 // alignment result
} GoAlignMsgClass;

kDeclareClassEx(Go, GoAlignMsg, GoDataMsg)

GoFx(kStatus) GoAlignMsg_Construct(GoAlignMsg* msg, kAlloc allocator);
GoFx(kStatus) GoAlignMsg_VInit(GoAlignMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoAlignMsg_VInitClone(GoAlignMsg msg, GoAlignMsg source, kAlloc alloc);
GoFx(kStatus) GoAlignMsg_VRelease(GoAlignMsg msg);
GoFx(kSize) GoAlignMsg_VSize(GoAlignMsg msg);
GoFx(kStatus) GoAlignMsg_WriteV11(GoAlignMsg msg, kSerializer serializer);
GoFx(kStatus) GoAlignMsg_ReadV11(GoAlignMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoAlignMsg_ReadV11Attr(GoAlignMsg msg, kSerializer serializer);

#define GoCalMsg_SetStatus_(D, V)         (xGoAlignMsg_CastRaw(D)->status = (V), kOK)

/*
 * GoExposureCalMsg
 */

typedef struct GoExposureCalMsgClass
{
    GoDataMsgClass base;
    k32u opId;
    kStatus status;                 // calibration result
    k32u exposure;                  // calibrated value
} GoExposureCalMsgClass;

kDeclareClassEx(Go, GoExposureCalMsg, GoDataMsg)

GoFx(kStatus) GoExposureCalMsg_Construct(GoExposureCalMsg* msg, kAlloc allocator);
GoFx(kStatus) GoExposureCalMsg_VInit(GoExposureCalMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoExposureCalMsg_VInitClone(GoExposureCalMsg msg, GoExposureCalMsg source, kAlloc alloc);
GoFx(kStatus) GoExposureCalMsg_VRelease(GoExposureCalMsg msg);
GoFx(kSize) GoExposureCalMsg_VSize(GoExposureCalMsg msg);
GoFx(kStatus) GoExposureCalMsg_WriteV12(GoExposureCalMsg msg, kSerializer serializer);
GoFx(kStatus) GoExposureCalMsg_ReadV12(GoExposureCalMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoExposureCalMsg_ReadV12Attr(GoExposureCalMsg msg, kSerializer serializer);

#define GoExposureCalMsg_SetStatus_(D, V)         (xGoExposureCalMsg_CastRaw(D)->status = (V), kOK)
#define GoExposureCalMsg_SetExposure_(D, V)         (xGoExposureCalMsg_CastRaw(D)->exposure = (V), kOK)


/*
 *  GoEdgeMatch
 */

typedef struct GoEdgeMatchMsgClass
{
    GoDataMsgClass base;

    k8u decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f qualityValue;
    k8u qualityDecision;
} GoEdgeMatchMsgClass;

kDeclareClassEx(Go, GoEdgeMatchMsg, GoDataMsg)

GoFx(kStatus) GoEdgeMatchMsg_Construct(GoEdgeMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoEdgeMatchMsg_VInit(GoEdgeMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoEdgeMatchMsg_VInitClone(GoEdgeMatchMsg msg, GoEdgeMatchMsg source, kAlloc alloc);
GoFx(kStatus) GoEdgeMatchMsg_VRelease(GoEdgeMatchMsg msg);
GoFx(kSize) GoEdgeMatchMsg_VSize(GoEdgeMatchMsg msg);
GoFx(kStatus) GoEdgeMatchMsg_WriteV16(GoEdgeMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoEdgeMatchMsg_ReadV16(GoEdgeMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoEdgeMatchMsg_SetDecision_(D, V)        (xGoEdgeMatchMsg_CastRaw(D)->decision = (V), kOK)
#define GoEdgeMatchMsg_SetXOffset_(D, V)         (xGoEdgeMatchMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoEdgeMatchMsg_SetYOffset_(D, V)         (xGoEdgeMatchMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoEdgeMatchMsg_SetZAngle_(D, V)          (xGoEdgeMatchMsg_CastRaw(D)->zAngle = (V), kOK)
#define GoEdgeMatchMsg_SetQualityValue_(D, V)    (xGoEdgeMatchMsg_CastRaw(D)->qualityValue = (V), kOK)
#define GoEdgeMatchMsg_SetQualityDecision_(D, V) (xGoEdgeMatchMsg_CastRaw(D)->qualityDecision = (V), kOK)

/*
 *  GoBoundingBoxMatch
 */

typedef struct GoBoundingBoxMatchMsgClass
{
    GoDataMsgClass base;

    k8s decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f lengthValue;
    k8u lengthDecision;
    k64f widthValue;
    k8u widthDecision;
} GoBoundingBoxMatchMsgClass;

kDeclareClassEx(Go, GoBoundingBoxMatchMsg, GoDataMsg)

GoFx(kStatus) GoBoundingBoxMatchMsg_Construct(GoBoundingBoxMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoBoundingBoxMatchMsg_VInit(GoBoundingBoxMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoBoundingBoxMatchMsg_VInitClone(GoBoundingBoxMatchMsg msg, GoBoundingBoxMatchMsg source, kAlloc alloc);
GoFx(kStatus) GoBoundingBoxMatchMsg_VRelease(GoBoundingBoxMatchMsg msg);
GoFx(kSize) GoBoundingBoxMatchMsg_VSize(GoBoundingBoxMatchMsg msg);
GoFx(kStatus) GoBoundingBoxMatchMsg_WriteV17(GoBoundingBoxMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoBoundingBoxMatchMsg_ReadV17(GoBoundingBoxMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoBoundingBoxMatchMsg_SetDecision_(D, V)        (xGoBoundingBoxMatchMsg_CastRaw(D)->decision = (V), kOK)
#define GoBoundingBoxMatchMsg_SetXOffset_(D, V)         (xGoBoundingBoxMatchMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoBoundingBoxMatchMsg_SetYOffset_(D, V)         (xGoBoundingBoxMatchMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoBoundingBoxMatchMsg_SetZAngle_(D, V)          (xGoBoundingBoxMatchMsg_CastRaw(D)->zAngle = (V), kOK)
#define GoBoundingBoxMatchMsg_SetLengthValue_(D, V)     (xGoBoundingBoxMatchMsg_CastRaw(D)->lengthValue = (V), kOK)
#define GoBoundingBoxMatchMsg_SetLengthDecision_(D, V)  (xGoBoundingBoxMatchMsg_CastRaw(D)->lengthDecision = (V), kOK)
#define GoBoundingBoxMatchMsg_SetWidthValue_(D, V)      (xGoBoundingBoxMatchMsg_CastRaw(D)->widthValue = (V), kOK)
#define GoBoundingBoxMatchMsg_SetWidthDecision_(D, V)   (xGoBoundingBoxMatchMsg_CastRaw(D)->widthDecision = (V), kOK)


/*
 *  GoEllipseMatch
 */

typedef struct GoEllipseMatchMsgClass
{
    GoDataMsgClass base;

    k8s decision;
    k64f xOffset;
    k64f yOffset;
    k64f zAngle;
    k64f majorValue;
    k8u majorDecision;
    k64f minorValue;
    k8u minorDecision;
} GoEllipseMatchMsgClass;

kDeclareClassEx(Go, GoEllipseMatchMsg, GoDataMsg)

GoFx(kStatus) GoEllipseMatchMsg_Construct(GoEllipseMatchMsg* msg, kAlloc allocator);
GoFx(kStatus) GoEllipseMatchMsg_VInit(GoEllipseMatchMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoEllipseMatchMsg_VInitClone(GoEllipseMatchMsg msg, GoEllipseMatchMsg source, kAlloc alloc);
GoFx(kStatus) GoEllipseMatchMsg_VRelease(GoEllipseMatchMsg msg);
GoFx(kSize) GoEllipseMatchMsg_VSize(GoEllipseMatchMsg msg);
GoFx(kStatus) GoEllipseMatchMsg_WriteV18(GoEllipseMatchMsg msg, kSerializer serializer);
GoFx(kStatus) GoEllipseMatchMsg_ReadV18(GoEllipseMatchMsg msg, kSerializer serializer, kAlloc alloc);

#define GoEllipseMatchMsg_SetDecision_(D, V)        (xGoEllipseMatchMsg_CastRaw(D)->decision = (V), kOK)
#define GoEllipseMatchMsg_SetXOffset_(D, V)         (xGoEllipseMatchMsg_CastRaw(D)->xOffset = (V), kOK)
#define GoEllipseMatchMsg_SetYOffset_(D, V)         (xGoEllipseMatchMsg_CastRaw(D)->yOffset = (V), kOK)
#define GoEllipseMatchMsg_SetZAngle_(D, V)          (xGoEllipseMatchMsg_CastRaw(D)->zAngle = (V), kOK)
#define GoEllipseMatchMsg_SetMajorValue_(D, V)      (xGoEllipseMatchMsg_CastRaw(D)->majorValue = (V), kOK)
#define GoEllipseMatchMsg_SetMajorDecision_(D, V)   (xGoEllipseMatchMsg_CastRaw(D)->majorDecision = (V), kOK)
#define GoEllipseMatchMsg_SetMinorValue_(D, V)      (xGoEllipseMatchMsg_CastRaw(D)->minorValue = (V), kOK)
#define GoEllipseMatchMsg_SetMinorDecision_(D, V)   (xGoEllipseMatchMsg_CastRaw(D)->minorDecision = (V), kOK)


/*
 *  GoEvent
 */

typedef struct GoEventMsgClass
{
    GoDataMsgClass base;

    GoEventType type;
} GoEventMsgClass;

kDeclareClassEx(Go, GoEventMsg, GoDataMsg)

GoFx(kStatus) GoEventMsg_Construct(GoEventMsg* msg, kAlloc allocator);
GoFx(kStatus) GoEventMsg_VInit(GoEventMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoEventMsg_VInitClone(GoEventMsg msg, GoEventMsg source, kAlloc alloc);
GoFx(kStatus) GoEventMsg_VRelease(GoEventMsg msg);
GoFx(kSize) GoEventMsg_VSize(GoEventMsg msg);
GoFx(kStatus) GoEventMsg_WriteV22(GoEventMsg msg, kSerializer serializer);
GoFx(kStatus) GoEventMsg_ReadV22(GoEventMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoEventMsg_ReadV22Attr(GoEventMsg msg, kSerializer serializer);

#define GoEventMsg_SetType_(D, V)            (xGoEventMsg_CastRaw(D)->type = (V), kOK)

/*
* GoTracheidEllipse
*/

kDeclareValueEx(Go, GoTracheidEllipse, kValue)

/*
* GoTracheidMsg
*/

typedef struct GoTracheidMsgClass
{
    GoDataMsgClass base;

    GoDataSource source;                   // Tracheid source
    k8u cameraIndex;                       // Camera index
    kArray2 moments;                       // Tracheid moments (kArray2<k32s>)

    kArray2 ellipses;                      // Tracheid ellipses, calculated (kArray2<GoTracheidEllipse>)

} GoTracheidMsgClass;

kDeclareClassEx(Go, GoTracheidMsg, GoDataMsg)

GoFx(kStatus) GoTracheidMsg_Construct(GoTracheidMsg* msg, kAlloc allocator);
GoFx(kStatus) GoTracheidMsg_VInit(GoTracheidMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoTracheidMsg_VInitClone(GoTracheidMsg msg, GoTracheidMsg source, kAlloc alloc);
GoFx(kStatus) GoTracheidMsg_VRelease(GoTracheidMsg msg);
GoFx(kSize) GoTracheidMsg_VSize(GoTracheidMsg msg);
GoFx(kStatus) GoTracheidMsg_WriteV23(GoTracheidMsg msg, kSerializer serializer);
GoFx(kStatus) GoTracheidMsg_ReadV23(GoTracheidMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoTracheidMsg_ReadV23Attr(GoTracheidMsg msg, kSerializer serializer, k32u* count, k32u* spotCount);

#define GO_TRACHEID_MOMENTS_PER_SPOT        (6)

GoFx(kStatus) GoTracheidMsg_GenerateEllipses(GoTracheidMsg msg, kAlloc alloc);

/*
* GoFeatureMsg
*/
///Utility functions for feature measages

GoFx(kStatus) GoFeatureMsg_WritePoint(kSerializer serializer, kPoint3d64f* point);
GoFx(kStatus) GoFeatureMsg_ReadPoint(kSerializer serializer, kPoint3d64f* point);

/*
* GoPointFeatureMsg
*/

typedef struct GoPointFeatureMsgClass
{
    GoDataMsgClass base;
    k16u id;                     // feature identifier
    kPoint3d64f point;           // feature data
} GoPointFeatureMsgClass;

kDeclareClassEx(Go, GoPointFeatureMsg, GoDataMsg)

GoFx(kStatus) GoPointFeatureMsg_Construct(GoPointFeatureMsg* msg, kAlloc allocator);
GoFx(kStatus) GoPointFeatureMsg_VInit(GoPointFeatureMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoPointFeatureMsg_VInitClone(GoPointFeatureMsg msg, GoPointFeatureMsg source, kAlloc alloc);
GoFx(kStatus) GoPointFeatureMsg_VRelease(GoPointFeatureMsg msg);
GoFx(kSize) GoPointFeatureMsg_VSize(GoPointFeatureMsg msg);
GoFx(kStatus) GoPointFeatureMsg_WriteV24(GoPointFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoPointFeatureMsg_ReadV24(GoPointFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoPointFeatureMsg_WriteV32(GoPointFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoPointFeatureMsg_ReadV32(GoPointFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoPointFeatureMsg_ReadV32Attr(GoPointFeatureMsg msg, kSerializer serializer);

#define GoPointFeatureMsg_SetId_(D, V)             (xGoPointFeatureMsg_CastRaw(D)->id = (V), kOK)

/*
* GoLineFeatureMsg
*/

typedef struct GoLineFeatureMsgClass
{
    GoDataMsgClass base;
    k16u id;                     // feature identifier
    kPoint3d64f point;           // feature data
    kPoint3d64f direction;       // feature data
} GoLineFeatureMsgClass;

kDeclareClassEx(Go, GoLineFeatureMsg, GoDataMsg)

GoFx(kStatus) GoLineFeatureMsg_Construct(GoLineFeatureMsg* msg, kAlloc allocator);
GoFx(kStatus) GoLineFeatureMsg_VInit(GoLineFeatureMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoLineFeatureMsg_VInitClone(GoLineFeatureMsg msg, GoLineFeatureMsg source, kAlloc alloc);
GoFx(kStatus) GoLineFeatureMsg_VRelease(GoLineFeatureMsg msg);
GoFx(kSize) GoLineFeatureMsg_VSize(GoLineFeatureMsg msg);
GoFx(kStatus) GoLineFeatureMsg_WriteV25(GoLineFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoLineFeatureMsg_ReadV25(GoLineFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoLineFeatureMsg_WriteV33(GoLineFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoLineFeatureMsg_ReadV33(GoLineFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoLineFeatureMsg_ReadV33Attr(GoLineFeatureMsg msg, kSerializer serializer);

#define GoLineFeatureMsg_SetId_(D, V)             (xGoLineFeatureMsg_CastRaw(D)->id = (V), kOK)

/*
* GoPlaneFeatureMsg
*/

typedef struct GoPlaneFeatureMsgClass
{
    GoDataMsgClass base;
    k16u id;                      // feature identifier
    kPoint3d64f normal;           // feature data
    k64f distanceToOrigin;        // feature data
} GoPlaneFeatureMsgClass;

kDeclareClassEx(Go, GoPlaneFeatureMsg, GoDataMsg)

GoFx(kStatus) GoPlaneFeatureMsg_Construct(GoPlaneFeatureMsg* msg, kAlloc allocator);
GoFx(kStatus) GoPlaneFeatureMsg_VInit(GoPlaneFeatureMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoPlaneFeatureMsg_VInitClone(GoPlaneFeatureMsg msg, GoPlaneFeatureMsg source, kAlloc alloc);
GoFx(kStatus) GoPlaneFeatureMsg_VRelease(GoPlaneFeatureMsg msg);
GoFx(kSize) GoPlaneFeatureMsg_VSize(GoPlaneFeatureMsg msg);
GoFx(kStatus) GoPlaneFeatureMsg_WriteV26(GoPlaneFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoPlaneFeatureMsg_ReadV26(GoPlaneFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoPlaneFeatureMsg_WriteV34(GoPlaneFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoPlaneFeatureMsg_ReadV34(GoPlaneFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoPlaneFeatureMsg_ReadV34Attr(GoPlaneFeatureMsg msg, kSerializer serializer);

#define GoPlaneFeatureMsg_SetId_(D, V)             (xGoPlaneFeatureMsg_CastRaw(D)->id = (V), kOK)

/*
* GoCircleFeatureMsg
*/

typedef struct GoCircleFeatureMsgClass
{
    GoDataMsgClass base;
    k16u id;                      // feature identifier
    kPoint3d64f center;           // feature data
    kPoint3d64f normal;           // feature data
    k64f radius;                  // feature data
} GoCircleFeatureMsgClass;

kDeclareClassEx(Go, GoCircleFeatureMsg, GoDataMsg)

GoFx(kStatus) GoCircleFeatureMsg_Construct(GoCircleFeatureMsg* msg, kAlloc allocator);
GoFx(kStatus) GoCircleFeatureMsg_VInit(GoCircleFeatureMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoCircleFeatureMsg_VInitClone(GoCircleFeatureMsg msg, GoCircleFeatureMsg source, kAlloc alloc);
GoFx(kStatus) GoCircleFeatureMsg_VRelease(GoCircleFeatureMsg msg);
GoFx(kSize) GoCircleFeatureMsg_VSize(GoCircleFeatureMsg msg);
GoFx(kStatus) GoCircleFeatureMsg_WriteV27(GoCircleFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoCircleFeatureMsg_ReadV27(GoCircleFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoCircleFeatureMsg_WriteV35(GoCircleFeatureMsg msg, kSerializer serializer);
GoFx(kStatus) GoCircleFeatureMsg_ReadV35(GoCircleFeatureMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoCircleFeatureMsg_ReadV35Attr(GoCircleFeatureMsg msg, kSerializer serializer);

#define GoCircleFeatureMsg_SetId_(D, V)             (xGoCircleFeatureMsg_CastRaw(D)->id = (V), kOK)


//Deprecated: 
/*
* GoProfileMsg
*/

#define GoProfileMsgClass GoProfilePointCloudMsgClass

//Deprecated Definitions
/**
* @class   GoProfileMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Deprecated: Represents a data message containing a set of profile arrays.
*/
#define GoProfileMsg GoProfilePointCloudMsg

GoFx(kStatus) GoProfileMsg_Construct(GoProfileMsg* msg, kAlloc allocator);
GoFx(kStatus) GoProfileMsg_VInit(GoProfileMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoProfileMsg_VInitClone(GoProfileMsg msg, GoProfileMsg source, kAlloc alloc);
GoFx(kStatus) GoProfileMsg_Allocate(GoProfileMsg msg, kSize count, kSize width);
GoFx(kStatus) GoProfileMsg_VRelease(GoProfileMsg msg);
GoFx(kSize) GoProfileMsg_VSize(GoProfileMsg msg);
GoFx(kStatus) GoProfileMsg_WriteV5(GoProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoProfileMsg_ReadV5(GoProfileMsg msg, kSerializer serializer, kAlloc alloc);

#define GoProfileMsg_(D)                          kCast(GoProfileMsgClass*, D)
#define GoProfileMsg_SetContent_(D, V)            (GoProfileMsg_(D)->content = (V), kOK)
#define GoProfileMsg_Content_(D)                  (GoProfileMsg_(D)->content)
#define GoProfileMsg_SetSource_(D, V)             (GoProfileMsg_(D)->source = (V), kOK)
#define GoProfileMsg_SetXResolution_(D, V)        (GoProfileMsg_(D)->xResolution = (V), kOK)
#define GoProfileMsg_SetZResolution_(D, V)        (GoProfileMsg_(D)->zResolution = (V), kOK)
#define GoProfileMsg_SetXOffset_(D, V)            (GoProfileMsg_(D)->xOffset = (V), kOK)
#define GoProfileMsg_SetZOffset_(D, V)            (GoProfileMsg_(D)->zOffset = (V), kOK)
#define GoProfileMsg_SetExposure_(D, V)           (GoProfileMsg_(D)->exposure = (V), kOK)
#define GoProfileMsg_SetCameraIndex_(D, V)        (GoProfileMsg_(D)->cameraIndex = (V), kOK)

/**
* Gets the profile source.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Profile source.
*/
GoFx(GoDataSource) GoProfileMsg_Source(GoProfileMsg msg);

/**
* Gets the count of profile arrays in this message.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Count of profile arrays.
*/
GoFx(kSize) GoProfileMsg_Count(GoProfileMsg msg);

/**
* Gets the count of ranges in each profile array.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Range count.
*/
GoFx(kSize) GoProfileMsg_Width(GoProfileMsg msg);

/**
* Gets the profile x-resolution, in nanometers.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X resolution (nm).
*/
GoFx(k32u) GoProfileMsg_XResolution(GoProfileMsg msg);

/**
* Gets the profile z-resolution, in nanometers.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z resolution (nm).
*/
GoFx(k32u) GoProfileMsg_ZResolution(GoProfileMsg msg);

/**
* Gets the profile x-offset, in micrometers.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X offset (um).
*/
GoFx(k32s) GoProfileMsg_XOffset(GoProfileMsg msg);

/**
* Gets the profile z-offset, in micrometers.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z offset (um).
*/
GoFx(k32s) GoProfileMsg_ZOffset(GoProfileMsg msg);

/**
* Gets a pointer to a profile array.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @param   index      Profile array index.
* @return             Profile pointer.
*/
GoFx(kPoint16s*) GoProfileMsg_At(GoProfileMsg msg, kSize index);

/**
* Gets the exposure.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Exposure in uS.
*/
GoFx(k32u) GoProfileMsg_Exposure(GoProfileMsg msg);


/**
* Gets the source camera index.
*
* @public             @memberof GoProfileMsg
* @version            Introduced in firmware 4.1.3.106
* @param   msg        Message object.
* @return             Camera index (0 - Front camera, 1 - Back camera).
*/
GoFx(k8u) GoProfileMsg_CameraIndex(GoProfileMsg msg);


/*
* GoResampledProfileMsg
*/

#define GoResampledProfileMsgClass GoUniformProfileMsgClass

/**
* @class   GoResampledProfileMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Deprecated: Represents a data message containing a set of re-sampled profile arrays.
*/
#define GoResampledProfileMsg GoUniformProfileMsg

GoFx(kStatus) GoResampledProfileMsg_Construct(GoResampledProfileMsg* msg, kAlloc allocator);
GoFx(kStatus) GoResampledProfileMsg_VInit(GoResampledProfileMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoResampledProfileMsg_VInitClone(GoResampledProfileMsg msg, GoResampledProfileMsg source, kAlloc alloc);
GoFx(kStatus) GoResampledProfileMsg_Allocate(GoResampledProfileMsg msg, kSize count, kSize width);
GoFx(kStatus) GoResampledProfileMsg_VRelease(GoResampledProfileMsg msg);
GoFx(kSize) GoResampledProfileMsg_VSize(GoResampledProfileMsg msg);
GoFx(kStatus) GoResampledProfileMsg_WriteV6(GoResampledProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoResampledProfileMsg_ReadV6(GoResampledProfileMsg msg, kSerializer serializer, kAlloc alloc);

GoFx(kStatus) GoResampledProfileMsg_ReadV6AttrProtoVer1(GoResampledProfileMsg msg, kSerializer serializer, k32u* countPtr, k32u* widthPtr);
GoFx(kStatus) GoResampledProfileMsg_ReadV6AttrProtoVer2(GoResampledProfileMsg msg, kSerializer serializer);
GoFx(kStatus) GoResampledProfileMsg_ReadV6AttrProtoVer3(GoResampledProfileMsg msg, kSerializer serializer);

#define GoResampledProfileMsg_(D)                          kCast(GoResampledProfileMsgClass*, D)
#define GoResampledProfileMsg_SetContent_(D, V)            (GoResampledProfileMsg_(D)->content = (V), kOK)
#define GoResampledProfileMsg_Content_(D)                  (GoResampledProfileMsg_(D)->content)
#define GoResampledProfileMsg_SetSource_(D, V)             (GoResampledProfileMsg_(D)->source = (V), kOK)
#define GoResampledProfileMsg_SetXResolution_(D, V)        (GoResampledProfileMsg_(D)->xResolution = (V), kOK)
#define GoResampledProfileMsg_SetZResolution_(D, V)        (GoResampledProfileMsg_(D)->zResolution = (V), kOK)
#define GoResampledProfileMsg_SetXOffset_(D, V)            (GoResampledProfileMsg_(D)->xOffset = (V), kOK)
#define GoResampledProfileMsg_SetZOffset_(D, V)            (GoResampledProfileMsg_(D)->zOffset = (V), kOK)
#define GoResampledProfileMsg_SetExposure_(D, V)           (GoResampledProfileMsg_(D)->exposure = (V), kOK)

/**
* Gets the profile source.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Profile source.
*/
GoFx(GoDataSource) GoResampledProfileMsg_Source(GoResampledProfileMsg msg);

/**
* Gets the count of re-sampled profile arrays in this message.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Count of profile arrays.
*/
GoFx(kSize) GoResampledProfileMsg_Count(GoResampledProfileMsg msg);

/**
* Gets the count of points in each re-sampled profile array.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Point count.
*/
GoFx(kSize) GoResampledProfileMsg_Width(GoResampledProfileMsg msg);

/**
* Gets the x-resolution, in nanometers.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X resolution (nm).
*/
GoFx(k32u) GoResampledProfileMsg_XResolution(GoResampledProfileMsg msg);

/**
* Gets the profile z-resolution, in nanometers.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z resolution (nm).
*/
GoFx(k32u) GoResampledProfileMsg_ZResolution(GoResampledProfileMsg msg);

/**
* Gets the profile x-offset, in micrometers.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X offset (um).
*/
GoFx(k32s) GoResampledProfileMsg_XOffset(GoResampledProfileMsg msg);

/**
* Gets the profile z-offset, in micrometers.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z offset (um).
*/
GoFx(k32s) GoResampledProfileMsg_ZOffset(GoResampledProfileMsg msg);

/**
* Gets a pointer to a re-sampled profile array.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @param   index      Profile array index.
* @return             Data pointer.
*/
GoFx(k16s*) GoResampledProfileMsg_At(GoResampledProfileMsg msg, kSize index);

/**
* Gets the exposure.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Exposure in uS.
*/
GoFx(k32u) GoResampledProfileMsg_Exposure(GoResampledProfileMsg msg);

/**
* Gets the source of the data stream.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoResampledProfileMsg_StreamStep(GoResampledProfileMsg msg);

/**
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoResampledProfileMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoResampledProfileMsg_StreamStepId(GoResampledProfileMsg msg);


/*
* GoSurfaceMsg
*/

#define GoSurfaceMsgClass GoUniformSurfaceMsgClass
/**
* @class   GoSurfaceMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Deprecated: Represents a data message containing a surface array.
*/
#define GoSurfaceMsg GoUniformSurfaceMsg

GoFx(kStatus) GoSurfaceMsg_Construct(GoSurfaceMsg* msg, kAlloc allocator);
GoFx(kStatus) GoSurfaceMsg_VInit(GoSurfaceMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoSurfaceMsg_VInitClone(GoSurfaceMsg msg, GoSurfaceMsg source, kAlloc alloc);
GoFx(kStatus) GoSurfaceMsg_Allocate(GoSurfaceMsg msg, kSize length, kSize width);
GoFx(kStatus) GoSurfaceMsg_VRelease(GoSurfaceMsg msg);
GoFx(kSize) GoSurfaceMsg_VSize(GoSurfaceMsg msg);
GoFx(kStatus) GoSurfaceMsg_WriteV8(GoSurfaceMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceMsg_ReadV8(GoSurfaceMsg msg, kSerializer serializer, kAlloc alloc);

GoFx(kStatus) GoSurfaceMsg_ReadV8AttrProtoVer1(GoSurfaceMsg msg, kSerializer serializer, k32u* lenPtr, k32u* widthPtr);
GoFx(kStatus) GoSurfaceMsg_ReadV8AttrProtoVer2(GoSurfaceMsg msg, kSerializer serializer);
GoFx(kStatus) GoSurfaceMsg_ReadV8AttrProtoVer3(GoSurfaceMsg msg, kSerializer serializer);

#define GoSurfaceMsg_(D)                          kCast(GoSurfaceMsgClass*, D)
#define GoSurfaceMsg_SetContent_(D, V)            (GoSurfaceMsg_(D)->content = (V), kOK)
#define GoSurfaceMsg_Content_(D)                  (GoSurfaceMsg_(D)->content)
#define GoSurfaceMsg_SetSource_(D, V)             (GoSurfaceMsg_(D)->source = (V), kOK)
#define GoSurfaceMsg_SetXResolution_(D, V)        (GoSurfaceMsg_(D)->xResolution = (V), kOK)
#define GoSurfaceMsg_SetYResolution_(D, V)        (GoSurfaceMsg_(D)->yResolution = (V), kOK)
#define GoSurfaceMsg_SetZResolution_(D, V)        (GoSurfaceMsg_(D)->zResolution = (V), kOK)
#define GoSurfaceMsg_SetXOffset_(D, V)            (GoSurfaceMsg_(D)->xOffset = (V), kOK)
#define GoSurfaceMsg_SetYOffset_(D, V)            (GoSurfaceMsg_(D)->yOffset = (V), kOK)
#define GoSurfaceMsg_SetZOffset_(D, V)            (GoSurfaceMsg_(D)->zOffset = (V), kOK)
#define GoSurfaceMsg_SetExposure_(D, V)           (GoSurfaceMsg_(D)->exposure = (V), kOK)

/**
* Gets the surface source.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Profile source.
*/
GoFx(GoDataSource) GoSurfaceMsg_Source(GoSurfaceMsg msg);

/**
* Gets the length of the surface (row count).
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Surface length.
*/
GoFx(kSize) GoSurfaceMsg_Length(GoSurfaceMsg msg);

/**
* Gets the width of the surface (column count).
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Surface width;
*/
GoFx(kSize) GoSurfaceMsg_Width(GoSurfaceMsg msg);

/**
* Gets the surface x-resolution, in nanometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X resolution (nm).
*/
GoFx(k32u) GoSurfaceMsg_XResolution(GoSurfaceMsg msg);

/**
* Gets the surface y-resolution, in nanometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Y resolution (nm).
*/
GoFx(k32u) GoSurfaceMsg_YResolution(GoSurfaceMsg msg);

/**
* Gets the surface z-resolution, in nanometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z resolution (nm).
*/
GoFx(k32u) GoSurfaceMsg_ZResolution(GoSurfaceMsg msg);

/**
* Gets the surface x-offset, in micrometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             X offset (um).
*/
GoFx(k32s) GoSurfaceMsg_XOffset(GoSurfaceMsg msg);

/**
* Gets the surface y-offset, in micrometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Y offset (um).
*/
GoFx(k32s) GoSurfaceMsg_YOffset(GoSurfaceMsg msg);

/**
* Gets the surface z-offset, in micrometers.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Z offset (um).
*/
GoFx(k32s) GoSurfaceMsg_ZOffset(GoSurfaceMsg msg);

/**
* Gets a pointer to a surface row.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @param   index      Surface row index.
* @return             Row pointer.
*/
GoFx(k16s*) GoSurfaceMsg_RowAt(GoSurfaceMsg msg, kSize index);

/**
* Gets the exposure.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.0.10.27
* @param   msg        Message object.
* @return             Exposure in uS.
*/
GoFx(k32u) GoSurfaceMsg_Exposure(GoSurfaceMsg msg);

/**
* Gets the source of the data stream.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoSurfaceMsg_StreamStep(GoSurfaceMsg msg);

/**
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoSurfaceMsg
* @version            Introduced in firmware 4.7.3.35
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoSurfaceMsg_StreamStepId(GoSurfaceMsg msg);

/// @endcond

/*
* GoGenericMsg
*/

typedef struct GoGenericMsgClass
{
    GoDataMsgClass base;
    k32u userType;
    kBool isObject;    
    kArray1 buffer;

    kObject objectPayload;
    kStatus serializerStatus;
} GoGenericMsgClass;

typedef struct GoGenericMsgStatic
{
    kLock generalLock;

    // Deserialization objects that can be shared by all GoGenericMsg instances.
    // This reduces overhead of each message.
    kLock serializerLock;
    kMemory serializerBuffer;
    kDat6Serializer serializer;
} GoGenericMsgStatic;

kDeclareFullClassEx(Go, GoGenericMsg, GoDataMsg)

GoFx(kStatus) xGoGenericMsg_InitStatic();
GoFx(kStatus) xGoGenericMsg_ReleaseStatic();
GoFx(kStatus) GoGenericMsg_InitSerializer();

GoFx(kStatus) GoGenericMsg_ReadObjectPayload(const void* data, kSize length, kObject* object, kAlloc alloc);
GoFx(kStatus) GoGenericMsg_WriteObjectPayload(kObject object, kArray1* data, kAlloc alloc);

GoFx(kStatus) GoGenericMsg_Construct(GoGenericMsg* msg, kAlloc allocator);
GoFx(kStatus) GoGenericMsg_VInit(GoGenericMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoGenericMsg_VInitClone(GoGenericMsg msg, GoGenericMsg source, kAlloc alloc);
GoFx(kStatus) GoGenericMsg_VRelease(GoGenericMsg msg);
GoFx(kSize) GoGenericMsg_VSize(GoGenericMsg msg);
GoFx(kStatus) GoGenericMsg_Write(GoGenericMsg msg, kSerializer serializer);
GoFx(kStatus) GoGenericMsg_Read(GoGenericMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoGenericMsg_ReadAttr(GoGenericMsg msg, kSerializer serializer, k32u* dataLength);

/**
* Deprecated: use base class GoDataMsg_StreamStep()
* Gets the data stream step.
*
* @public             @memberof GoGenericMsg
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoGenericMsg_StreamStep(GoGenericMsg msg);

/**
* Deprecated: use base class GoDataMsg_StreamStepId()
* Gets the data stream ID within the step.
*
* @public             @memberof GoGenericMsg
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoGenericMsg_StreamStepId(GoGenericMsg msg);

/*
* GoNullMsg
*/

typedef struct GoNullMsgClass
{
    GoDataMsgClass base;
    k32s errorStatus;
} GoNullMsgClass;

kDeclareFullClassEx(Go, GoNullMsg, GoDataMsg)

GoFx(kStatus) GoNullMsg_Construct(GoNullMsg* msg, kAlloc allocator);
GoFx(kStatus) GoNullMsg_VInit(GoNullMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoNullMsg_VInitClone(GoNullMsg msg, GoNullMsg source, kAlloc alloc);
GoFx(kStatus) GoNullMsg_VRelease(GoNullMsg msg);
GoFx(kSize) GoNullMsg_VSize(GoNullMsg msg);
GoFx(kStatus) GoNullMsg_Write(GoNullMsg msg, kSerializer serializer);
GoFx(kStatus) GoNullMsg_Read(GoNullMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoNullMsg_ReadAttr(GoNullMsg msg, kSerializer serializer);

/*
* GoMeshMsg
*/

typedef struct GoMeshMsgClass
{
    GoDataMsgClass base;
 
    GoDataSource source;                                                // source

    kBool hasData;                                                      //  Whether any channel buffer has been allocated
    kSize maxUserChannel;                                               //  Max number of user channels allowed
    kSize numOfUserChannel;                                             //  Number of additional (non-system) channels

    kPoint3d64f offset;                                                 //  Object center
    kPoint3d64f range;                                                  //  Object dimension
 
    Go3dTransform64f transform;                                         //  Transformation matrix
 
    GoMeshMsgChannel systemChannels[GO_MESH_MSG_NUM_OF_SYSTEM_CHANNEL]; //  System (default) channels
    
    // This buffer holds any additional user channel structured data
    kArray1 userChannels;   
} GoMeshMsgClass;

kDeclareFullClassEx(Go, GoMeshMsg, GoDataMsg)

GoFx(kStatus) GoMeshMsg_Construct(GoMeshMsg* msg, kAlloc allocator);
GoFx(kStatus) GoMeshMsg_VInit(GoMeshMsg msg, kType type, kAlloc alloc);
GoFx(kStatus) GoMeshMsg_VInitClone(GoMeshMsg msg, GoMeshMsg source, kAlloc alloc);
GoFx(kStatus) GoMeshMsg_VRelease(GoMeshMsg msg);
GoFx(kSize) GoMeshMsg_VSize(GoMeshMsg msg);
GoFx(kStatus) GoMeshMsg_Write(GoMeshMsg msg, kSerializer serializer);
GoFx(kStatus) GoMeshMsg_Read(GoMeshMsg msg, kSerializer serializer, kAlloc alloc);
GoFx(kStatus) GoMeshMsg_ReadAttr(GoMeshMsg msg, kSerializer serializer, k32s* numOfSystemChannel, k32s* numOfUserChannel, k32s* numOfTotalChannel);
GoFx(kStatus) GoMeshMsg_ReadChannelAttr(GoMeshMsg msg,
    kSerializer serializer,
    k32s* channelId,
    k32u* channelType,
    k32s* channelState,
    k32u* channelFlag,
    k32s* allocatedSize,
    k32s* usedSize);

GoFx(GoMeshMsgChannel*) GoMeshMsg_GetChannel(GoMeshMsg msg, GoMeshMsgChannelId id);
GoFx(kStatus) GoMeshMsg_AddUserChannel(GoMeshMsg msg, GoMeshMsgChannelId* id, GoMeshMsgChannelType type);
GoFx(kStatus) GoMeshMsg_AllocateChannelData(GoMeshMsg msg, GoMeshMsgChannelId id, kType type, kSize size, kAlloc alloc);
GoFx(kStatus) GoMeshMsg_SetUsedChannelDataCount(GoMeshMsg msg, GoMeshMsgChannelId id, kSize count);
GoFx(kStatus) GoMeshMsg_SetSource(GoMeshMsg msg, GoDataSource source);
GoFx(kStatus) GoMeshMsg_SetChannelFlag(GoMeshMsg msg, GoMeshMsgChannelId id, k32u flag);
GoFx(kStatus) GoMeshMsg_SetHasData(GoMeshMsg msg, kBool data);
GoFx(kStatus) GoMeshMsg_SetOffset(GoMeshMsg msg, kPoint3d64f offset);
GoFx(kStatus) GoMeshMsg_SetRange(GoMeshMsg msg, kPoint3d64f range);
GoFx(kStatus) GoMeshMsg_SetTransform(GoMeshMsg msg, Go3dTransform64f tranform);

#endif
