/**
 * @file    GoProfileToolUtils.h
 * @brief   Declares shared profile tool configuration classes.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOL_UTILS_H
#define GO_PROFILE_TOOL_UTILS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoUtils.h>

/**
 * @class   GoProfileRegion
 * @extends kObject
 * @note    Supported with G1, G2
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a profile region used in various profile tools.
 */
typedef kObject GoProfileRegion;

/**
 * Sets the X position.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @param    x           The X-position to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileRegion_SetX(GoProfileRegion region, k64f x);

/**
 * Gets the X-position.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @return               The X-position of the region.
 */
GoFx(k64f) GoProfileRegion_X(GoProfileRegion region);

/**
 * Sets the Z-position.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @param    z           The Z-position to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileRegion_SetZ(GoProfileRegion region, k64f z);

/**
 * Gets the Z-position.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @return               The Z-position of the region.
 */
GoFx(k64f) GoProfileRegion_Z(GoProfileRegion region);

/**
 * Sets the width.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @param    width       The width to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileRegion_SetWidth(GoProfileRegion region, k64f width);

/**
 * Gets the width.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @return               The width of the region.
 */
GoFx(k64f) GoProfileRegion_Width(GoProfileRegion region);

/**
 * Sets the height.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @param    height      The height to set.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileRegion_SetHeight(GoProfileRegion region, k64f height);

/**
 * Gets the height.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    region      GoProfileRegion object.
 * @return               The height of the region.
 */
GoFx(k64f) GoProfileRegion_Height(GoProfileRegion region);

/**
* @class   GoProfileFeature
* @extends kObject
* @note    Supported with G1, G2
* @ingroup GoSdk-ProfileTools
* @brief   Represents a profile feature used in various profile tools.
*/
typedef kObject GoProfileFeature;

/**
 * Sets the feature type.
 *
 * @public               @memberof GoProfileFeature
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    feature     GoProfileFeature object.
 * @param    type        GoProfileFeatureType object.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileFeature_SetType(GoProfileFeature feature, GoProfileFeatureType type);

/**
 * Gets the profile feature type.
 *
 * @public               @memberof GoProfileFeature
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    feature     GoProfileFeature object.
 * @return               The feature type.
 */
GoFx(GoProfileFeatureType) GoProfileFeature_Type(GoProfileFeature feature);

/**
 * Gets the profile feature region.
 *
 * @public               @memberof GoProfileFeature
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    feature     GoProfileFeature object.
 * @return               A GoProfileRegion object.
 */
GoFx(GoProfileRegion) GoProfileFeature_Region(GoProfileFeature feature);

/**
 * Enables the region.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    feature     GoProfileFeature object.
 * @param    enable      kTRUE to use the region, kFALSE otherwise.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileFeature_EnableRegion(GoProfileFeature feature, kBool enable);

/**
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileFeature
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    feature     GoProfileFeature object.
 * @return               kTRUE if the region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileFeature_RegionEnabled(GoProfileFeature feature);

/**
* @class   GoProfileLineFittingRegion
* @extends kObject
* @ingroup GoSdk-ProfileTools
* @brief   Represents a profile line region used in various profile tools.
*/
typedef kObject GoProfileLineFittingRegion;

/**
 * Returns the number of regions.
 *
 * @public               @memberof GoProfileLine
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    lineRegion  GoProfileLineFittingRegion object.
 * @return               The region count.
 */
GoFx(k32u) GoProfileLineFittingRegion_RegionCount(GoProfileLineFittingRegion lineRegion);

/**
* Sets the number of regions in use.
*
* @public               @memberof GoProfileLine
* @version              Introduced in firmware 4.7.11.65
* @param    lineRegion  GoProfileLineFittingRegion object.
* @param    count       number of regions configured
* @return               Operation status.
*/
GoFx(kStatus) GoProfileLineFittingRegion_SetRegionCount(GoProfileLineFittingRegion lineRegion, kSize count);

/**
 * Gets the profile region based on the given index.
 *
 * @public               @memberof GoProfileLine
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    lineRegion  GoProfileLineFittingRegion object.
 * @param    index       The index of the profile region to return.
 * @return               The profile region.
 * @see                  GoProfileLineFittingRegion_RegionCount
 */
GoFx(GoProfileRegion) GoProfileLineFittingRegion_RegionAt(GoProfileLineFittingRegion lineRegion, kSize index);

/**
 * Enables the region.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    lineRegion  GoProfileLineFittingRegion object.
 * @param    index       The region index of which to modify.
 * @param    enable      kTRUE to use the region, kFALSE otherwise.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileLineFittingRegion_EnableRegionAt(GoProfileLineFittingRegion lineRegion, kSize index, kBool enable);

/**
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileLineFittingRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    lineRegion  GoProfileLineFittingRegion object.
 * @param    index       The region index to retrieve the enabled state from.
 * @return               kTRUE if the region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileLineFittingRegion_RegionEnabledAt(GoProfileLineFittingRegion lineRegion, kSize index);

/**
* @class   GoProfileEdge
* @extends kObject
* @note    Supported with G1, G2
* @ingroup GoSdk-ProfileTools
* @brief   Represents a profile edge used in various profile tools.
*/
typedef kObject GoProfileEdge;

/**
 * Sets the profile edge type.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    type        The type of the profile edge.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetType(GoProfileEdge edge, GoProfileEdgeType type);

/**
 * Gets the profile edge type.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The profile edge type.
 */
GoFx(GoProfileEdgeType) GoProfileEdge_Type(GoProfileEdge edge);

/**
 * Gets the profile region.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The profile region.
 */
GoFx(GoProfileRegion) GoProfileEdge_Region(GoProfileEdge edge);

/**
 * Enables the region.
 *
 * @public               @memberof GoProfileRegion
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    edge        GoProfileEdge object.
 * @param    enable      kTRUE to use the region, kFALSE otherwise.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_EnableRegion(GoProfileEdge edge, kBool enable);

/**
 * Indicates whether the region is enabled.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.4.4.14
 * @param    edge        GoProfileEdge object.
 * @return               kTRUE if the region is enabled and kFALSE otherwise.
 */
GoFx(kBool) GoProfileEdge_RegionEnabled(GoProfileEdge edge);

/**
 * Sets the maximum void width.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    width       The width to set for the maximum void width.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetVoidWidthMax(GoProfileEdge edge, k64f width);

/**
 * Gets the maximum void width.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               Maximum void width.
 */
GoFx(k64f) GoProfileEdge_VoidWidthMax(GoProfileEdge edge);

/**
 * Sets the minimum depth.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    depth       The minimum depth.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetDepthMin(GoProfileEdge edge, k64f depth);

/**
 * Gets the minimum depth.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The minimum depth.
 */
GoFx(k64f) GoProfileEdge_DepthMin(GoProfileEdge edge);

/**
 * Sets the surface width.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    width       The surface width.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetSurfaceWidth(GoProfileEdge edge, k64f width);

/**
 * Gets the surface width.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The surface width.
 */
GoFx(k64f) GoProfileEdge_SurfaceWidth(GoProfileEdge edge);

/**
 * Sets the surface offset.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    offset      The surface offset.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetSurfaceOffset(GoProfileEdge edge, k64f offset);

/**
 * Gets the surface offset.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The surface offset.
 */
GoFx(k64f) GoProfileEdge_SurfaceOffset(GoProfileEdge edge);

/**
 * Sets the nominal radius.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    radius      The nominal radius.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetNominalRadius(GoProfileEdge edge, k64f radius);

/**
 * Gets the nominal radius.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The nominal radius.
 */
GoFx(k64f) GoProfileEdge_NominalRadius(GoProfileEdge edge);

/**
 * Sets the edge angle.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @param    angle        k64f object.
 * @return               Operation status.
 */
GoFx(kStatus) GoProfileEdge_SetEdgeAngle(GoProfileEdge edge, k64f angle);

/**
 * Gets the edge angle.
 *
 * @public               @memberof GoProfileEdge
 * @note                 Supported with G1, G2
 * @version              Introduced in firmware 4.0.10.27
 * @param    edge        GoProfileEdge object.
 * @return               The edge angle.
 */
GoFx(k64f) GoProfileEdge_EdgeAngle(GoProfileEdge edge);

#include <GoSdk/Tools/GoProfileToolUtils.x.h>

#endif
