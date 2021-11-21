/**
 * @file    GoProfileToolUtils.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOL_UTILS_X_H
#define GO_PROFILE_TOOL_UTILS_X_H

#include <kApi/Data/kXml.h>

typedef struct GoProfileRegionClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    k64f x;
    k64f z;
    k64f width;
    k64f height;
} GoProfileRegionClass;

kDeclareClassEx(Go, GoProfileRegion, kObject)

GoFx(kStatus) GoProfileRegion_Construct(GoProfileRegion* region, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileRegion_Init(GoProfileRegion region, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileRegion_VRelease(GoProfileRegion region);

GoFx(kStatus) GoProfileRegion_Read(GoProfileRegion region, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRegion_Write(GoProfileRegion region, kXml xml, kXmlItem item);

typedef struct GoProfileFeatureClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoProfileFeatureType type;
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileFeatureClass;

kDeclareClassEx(Go, GoProfileFeature, kObject)

GoFx(kStatus) GoProfileFeature_Construct(GoProfileFeature* feature, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileFeature_Init(GoProfileFeature feature, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileFeature_VRelease(GoProfileFeature feature);

GoFx(kStatus) GoProfileFeature_Read(GoProfileFeature feature, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileFeature_Write(GoProfileFeature feature, kXml xml, kXmlItem item);

#define GO_PROFILE_LINE_REGION_MAX_FITTING_REGIONS      (2)     ///< The maximum number of line regions permitted for a profile tool.

typedef struct GoProfileLineFittingRegionClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    k32u regionCount;
    kBool regionEnabled[GO_PROFILE_LINE_REGION_MAX_FITTING_REGIONS];
    GoProfileRegion regions[GO_PROFILE_LINE_REGION_MAX_FITTING_REGIONS];
} GoProfileLineFittingRegionClass;

kDeclareClassEx(Go, GoProfileLineFittingRegion, kObject)
GoFx(kStatus) GoProfileLineFittingRegion_Construct(GoProfileLineFittingRegion* lineRegion, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileLineFittingRegion_Init(GoProfileLineFittingRegion lineRegion, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileLineFittingRegion_VRelease(GoProfileLineFittingRegion lineRegion);

GoFx(kStatus) GoProfileLineFittingRegion_Read(GoProfileLineFittingRegion lineRegion, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineFittingRegion_Write(GoProfileLineFittingRegion lineRegion, kXml xml, kXmlItem item);


typedef struct GoProfileEdgeClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoProfileEdgeType edgeType;
    kBool regionEnabled;
    GoProfileRegion region;
    k64f maxVoidWidth;
    k64f minDepth;
    k64f surfaceWidth;
    k64f surfaceOffset;
    k64f nominalRadius;
    k64f edgeAngle;
} GoProfileEdgeClass;

kDeclareClassEx(Go, GoProfileEdge, kObject)

GoFx(kStatus) GoProfileEdge_Construct(GoProfileEdge* profileEdge, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileEdge_Init(GoProfileEdge profileEdge, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileEdge_VRelease(GoProfileEdge profileEdge);

GoFx(kStatus) GoProfileEdge_Read(GoProfileEdge edge, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileEdge_Write(GoProfileEdge edge, kXml xml, kXmlItem item);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * @brief   Represents a profile line region used in various profile tools.
 */
#define GoProfileLineRegion GoProfileLineFittingRegion

GoFx(kStatus) GoProfileLineRegion_Construct(GoProfileLineRegion* lineRegion, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileLineRegion_Init(GoProfileLineRegion lineRegion, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileLineRegion_VRelease(GoProfileLineRegion lineRegion);

GoFx(kStatus) GoProfileLineRegion_Read(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLineRegion_Write(GoProfileLineRegion lineRegion, kXml xml, kXmlItem item);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * Returns the number of regions.
 *
 * @public               @memberof GoProfileLine
 * @version              Introduced in firmware 4.0.10.27
 * @param    lineRegion  GoProfileLineRegion object.
 * @return               The region count.
 */
GoFx(k32u) GoProfileLineRegion_RegionCount(GoProfileLineRegion lineRegion);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * Sets the number of regions in use.
 *
 * @public               @memberof GoProfileLine
 * @version              Introduced in firmware 4.7.11.65
 * @param    lineRegion  GoProfileLineRegion object.
 * @param    count       number of regions configured
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileLineRegion_SetRegionCount(GoProfileLineRegion lineRegion, kSize count);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * Gets the profile region based on the given index.
 *
 * @public               @memberof GoProfileLine
 * @version              Introduced in firmware 4.0.10.27
 * @param    lineRegion  GoProfileLineRegion object.
 * @param    index       The index of the profile region to return.
 * @return               The profile region.
 * @see                  GoProfileLineRegion_RegionCount
 */
GoFx(GoProfileRegion) GoProfileLineRegion_RegionAt(GoProfileLineRegion lineRegion, kSize index);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * Enables the region.
 *
 * @public               @memberof GoProfileRegion
 * @version              Introduced in firmware 4.4.4.14
 * @param    lineRegion  GoProfileLineRegion object.
 * @param    index       The region index of which to modify.
 * @param    enable      kTRUE to use the region, kFALSE otherwise.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileLineRegion_EnableRegionAt(GoProfileLineRegion lineRegion, kSize index, kBool enable);

/**
 * @deprecated   Use GoProfileLineFittingRegion instead
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileLineRegion
 * @version              Introduced in firmware 4.4.4.14
 * @param    lineRegion  GoProfileLineRegion object.
 * @param    index       The region index to retrieve the enabled state from.
 * @return               kTRUE if the region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileLineRegion_RegionEnabledAt(GoProfileLineRegion lineRegion, kSize index);

#endif
