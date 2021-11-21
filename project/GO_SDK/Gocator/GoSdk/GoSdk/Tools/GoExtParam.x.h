/**
 * @file    GoExtParam.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_PARAM_X_H
#define GO_EXT_PARAM_X_H

#include <kApi/Data/kXml.h>

kDeclareValueEx(Go, GoExtParamIntOption, kValue)
kDeclareValueEx(Go, GoExtParamFloatOption, kValue)

typedef struct GoExtParamVTable
{
    kObjectVTable base;

    kStatus (kCall* VInit)(kObject object, kType type, kObject sensor, kAlloc alloc);
    kStatus (kCall* VRead)(kObject object, kXml xml, kXmlItem item);
    kStatus (kCall* VWrite)(kObject object, kXml xml, kXmlItem item);
} GoExtParamVTable;

kDeclareVirtualClassEx(Go, GoExtParam, kObject)
#define GoExtParam_Class_(PARAM)                     (kCastClass_(GoExtParam, PARAM))

typedef struct GoExtParamClass
{
    kObjectClass base;
    kObject sensor;

    kText64 paramType;
    kText128 label;
    kText64 id;
    kBool used;
    kString units;
} GoExtParamClass;

kDeclareClassEx(Go, GoExtParamBool, GoExtParam)
typedef struct GoExtParamBoolClass
{
    GoExtParamClass base;

    kBool value;
} GoExtParamBoolClass;

kDeclareClassEx(Go, GoExtParamInt, GoExtParam)
typedef struct GoExtParamIntClass
{
    GoExtParamClass base;

    k32s value;
    kArrayList options; //of options which consist of an integer value and name
    k32s valMin;
    k32s valMax;
    kBool valLimitsUsed;
} GoExtParamIntClass;

kDeclareClassEx(Go, GoExtParamFloat, GoExtParam)
typedef struct GoExtParamFloatClass
{
    GoExtParamClass base;

    k64f value;
    kArrayList options; //of options which consist of an integer value and name
    k64f valMin;
    k64f valMax;
    kBool valLimitsUsed;
} GoExtParamFloatClass;

kDeclareClassEx(Go, GoExtParamString, GoExtParam)
typedef struct GoExtParamStringClass
{
    GoExtParamClass base;

    kString value;
} GoExtParamStringClass;

kDeclareClassEx(Go, GoExtParamProfileRegion, GoExtParam)
typedef struct GoExtParamProfileRegionClass
{
    GoExtParamClass base;

    GoProfileRegion value;
} GoExtParamProfileRegionClass;

kDeclareClassEx(Go, GoExtParamSurfaceRegion2d, GoExtParam)
typedef struct GoExtParamSurfaceRegion2dClass
{
    GoExtParamClass base;

    GoSurfaceRegion2d value;
} GoExtParamSurfaceRegion2dClass;

kDeclareClassEx(Go, GoExtParamSurfaceRegion3d, GoExtParam)
typedef struct GoExtParamSurfaceRegion3dClass
{
    GoExtParamClass base;

    GoRegion3d value;
} GoExtParamSurfaceRegion3dClass;

kDeclareClassEx(Go, GoExtParamFeature, GoExtParam)
typedef struct GoExtParamFeatureClass
{
    GoExtParamClass base;
    k32s featureId;
    kArrayList options; //of options which consist of an integer value and name
    k32s valMin;
    k32s valMax;
    kBool valLimitsUsed;
} GoExtParamFeatureClass;

kDeclareClassEx(Go, GoExtParamDataInput, GoExtParam)
typedef struct GoExtParamDataInputClass
{
    GoExtParamClass base;
    GoDataStreamId streamId;
    kArrayList options; //of options which consist of an integer value and name
}GoExtParamDataInputClass;

kDeclareClassEx(Go, GoExtParamPointSetRegion, GoExtParam)
typedef struct GoExtParamPointSetRegionClass
{
    GoExtParamClass base;

    GoPointSetRegion value;
} GoExtParamPointSetRegionClass;

GoFx(kStatus) GoExtParam_Construct(GoExtParam* param, kType type, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParam_VInit(GoExtParam param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParam_VRelease(GoExtParam value);
GoFx(kStatus) GoExtParam_VRead(GoExtParam param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParam_VWrite(GoExtParam param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParam_Read(GoExtParam param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParam_Write(GoExtParam param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamBool_Construct(GoExtParamBool* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamBool_VInit(GoExtParamBool param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamBool_VRelease(GoExtParamBool value);
GoFx(kStatus) GoExtParamBool_VRead(GoExtParamBool param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamBool_VWrite(GoExtParamBool param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamInt_Construct(GoExtParamInt* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamInt_VInit(GoExtParamInt param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamInt_VRelease(GoExtParamInt value);
GoFx(kStatus) GoExtParamInt_VRead(GoExtParamInt param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamInt_VWrite(GoExtParamInt param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamFloat_Construct(GoExtParamFloat* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamFloat_VInit(GoExtParamFloat param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamFloat_VRelease(GoExtParamFloat value);
GoFx(kStatus) GoExtParamFloat_VRead(GoExtParamFloat param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamFloat_VWrite(GoExtParamFloat param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamString_Construct(GoExtParamString* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamString_VInit(GoExtParamString param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamString_VRelease(GoExtParamString value);
GoFx(kStatus) GoExtParamString_VRead(GoExtParamString param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamString_VWrite(GoExtParamString param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamProfileRegion_Construct(GoExtParamProfileRegion* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamProfileRegion_VInit(GoExtParamProfileRegion param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamProfileRegion_VRelease(GoExtParamProfileRegion value);
GoFx(kStatus) GoExtParamProfileRegion_VRead(GoExtParamProfileRegion param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamProfileRegion_VWrite(GoExtParamProfileRegion param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamSurfaceRegion2d_Construct(GoExtParamSurfaceRegion2d* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamSurfaceRegion2d_VInit(GoExtParamSurfaceRegion2d param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamSurfaceRegion2d_VRelease(GoExtParamSurfaceRegion2d value);
GoFx(kStatus) GoExtParamSurfaceRegion2d_VRead(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamSurfaceRegion2d_VWrite(GoExtParamSurfaceRegion2d param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamSurfaceRegion3d_Construct(GoExtParamSurfaceRegion3d* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamSurfaceRegion3d_VInit(GoExtParamSurfaceRegion3d param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamSurfaceRegion3d_VRelease(GoExtParamSurfaceRegion3d value);
GoFx(kStatus) GoExtParamSurfaceRegion3d_VRead(GoExtParamSurfaceRegion3d param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamSurfaceRegion3d_VWrite(GoExtParamSurfaceRegion3d param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamFeature_Construct(GoExtParamFeature* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamFeature_VInit(GoExtParamFeature param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamFeature_VRelease(GoExtParamFeature value);
GoFx(kStatus) GoExtParamFeature_VRead(GoExtParamFeature param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamFeature_VWrite(GoExtParamFeature param, kXml xml, kXmlItem item);

GoFx(kStatus) GoExtParamDataInput_Construct(GoExtParamDataInput* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamDataInput_VInit(GoExtParamDataInput param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamDataInput_VRelease(GoExtParamDataInput param);
GoFx(kStatus) GoExtParamDataInput_VRead(GoExtParamDataInput param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamDataInput_VWrite(GoExtParamDataInput param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamDataInput_ParseList(kXml xml, kXmlItem item, kArrayList list);

GoFx(kStatus) GoExtParamPointSetRegion_Construct(GoExtParamPointSetRegion* param, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoExtParamPointSetRegion_VInit(GoExtParamPointSetRegion param, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoExtParamPointSetRegion_VRelease(GoExtParamPointSetRegion value);
GoFx(kStatus) GoExtParamPointSetRegion_VRead(GoExtParamPointSetRegion param, kXml xml, kXmlItem item);
GoFx(kStatus) GoExtParamPointSetRegion_VWrite(GoExtParamPointSetRegion param, kXml xml, kXmlItem item);

GoFx(kType) GoExtUtils_GetKType(const kChar* paramType);

GoFx(kStatus) GoExtParamFloat_ParseList(kXml xml, kXmlItem item, kArrayList list);
GoFx(kStatus) GoExtParamInt_ParseList(kXml xml, kXmlItem item, kArrayList list);

GoFx(kBool) GoExtUtils_InParamOptionList32s(kArrayList list, k32s value);
GoFx(kBool) GoExtUtils_InParamOptionList64f(kArrayList list, k64f value);

#endif
