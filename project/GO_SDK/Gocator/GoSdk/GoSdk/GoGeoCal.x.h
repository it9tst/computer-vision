/**
* @file    GoGeoCal.x.h
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GO_GEOCAL_X_H
#define GO_GEOCAL_X_H

#include <kApi/Data/kXml.h>
#include <kApi/Data/kMath.h>

#define GO_GEO_CAL_VERSION                            (1)
/** @} */

typedef struct GoGeoCalWindow 
{
    k32u x;
    k32u y;
    k32u width;
    k32u height;
    k32u xSubsampling;
    k32u ySubsampling;

} GoGeoCalWindow;

typedef struct GoGeoCalCamera 
{
    GoGeoCalWindow calWindow;
    kArray1 xResCoeffs;
    kArray1 yResCoeffs;
    kArray1 xCenterCoeffs;
    kArray1 yCenterCoeffs;
    k64f roll;

} GoGeoCalCamera;

typedef struct GoGeoCalClass
{
    kObjectClass base;

    k32u id;
    kText256 timestamp;

    kArray1 cameras;         // Type: GoGeoCalCamera

} GoGeoCalClass;

kDeclareClassEx(Go, GoGeoCal, kObject)

GoFx(kStatus) GoGeoCal_Construct(GoGeoCal* geoCal, kXml xml, kAlloc alloc);

GoFx(kStatus) GoGeoCal_Init(GoGeoCal geoCal, kType type, kXml xml, kAlloc alloc);
GoFx(kStatus) GoGeoCal_VRelease(GoGeoCal geoCal);

#endif
