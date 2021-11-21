/** 
 * @file    GoSurfaceToolUtils.h
 * @brief   Declares all surface tools and their related classes. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef GO_SURFACE_TOOL_UTILS_H
#define GO_SURFACE_TOOL_UTILS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/Tools/GoFeature.h>

/**
* @class   GoSurfaceRegion2d
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a two dimensional surface tool region.
*/
typedef kObject GoSurfaceRegion2d; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    x          The X position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetX(GoSurfaceRegion2d region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The X position of the region.(mm)            
 */
GoFx(k64f) GoSurfaceRegion2d_X(GoSurfaceRegion2d region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    value      The Y position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetY(GoSurfaceRegion2d region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The Y position of the region.(mm)         
 */
GoFx(k64f) GoSurfaceRegion2d_Y(GoSurfaceRegion2d region);

/** 
 * Sets the width.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    width      The width to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetWidth(GoSurfaceRegion2d region, k64f width);

/** 
 * Gets the width.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The width of the region.(mm)           
 */
GoFx(k64f) GoSurfaceRegion2d_Width(GoSurfaceRegion2d region);

/** 
 * Sets the length.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @param    length     The length to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoSurfaceRegion2d_SetLength(GoSurfaceRegion2d region, k64f length);

/** 
 * Gets the length.
 *
 * @public              @memberof GoSurfaceRegion2d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoSurfaceRegion2d object.
 * @return              The length of the region.(mm)            
 */
GoFx(k64f) GoSurfaceRegion2d_Length(GoSurfaceRegion2d region);

/**
* Sets the length.
*
* @public              @memberof GoSurfaceRegion2d
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    region     GoSurfaceRegion2d object.
* @param    zAngle     The zAngle to set.(degrees)
* @return              Operation status.
*/
GoFx(kStatus) GoSurfaceRegion2d_SetZAngle(GoSurfaceRegion2d region, k64f zAngle);

/**
* Gets the length.
*
* @public              @memberof GoSurfaceRegion2d
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    region     GoSurfaceRegion2d object.
* @return              The zAngle of the region.(degrees)
*/
GoFx(k64f) GoSurfaceRegion2d_ZAngle(GoSurfaceRegion2d region);


/**
* @class   GoRegion3d
* @extends kObject
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a three dimensional surface region.
*/
typedef kObject GoRegion3d; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    x          The X position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetX(GoRegion3d region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The X position of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_X(GoRegion3d region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    value      The Y position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetY(GoRegion3d region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The Y position of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_Y(GoRegion3d region);

/** 
 * Sets the Z-position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    z          The Z-position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetZ(GoRegion3d region, k64f z);

/** 
 * Gets the Z-position.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The Z-position of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_Z(GoRegion3d region);

/** 
 * Sets the width.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    width      The width to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetWidth(GoRegion3d region, k64f width);

/** 
 * Gets the width.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The width of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_Width(GoRegion3d region);

/** 
 * Sets the length.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    length     The length to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetLength(GoRegion3d region, k64f length);

/** 
 * Gets the length.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The length of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_Length(GoRegion3d region);

/** 
 * Sets the height.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @param    height     The height to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoRegion3d_SetHeight(GoRegion3d region, k64f height);

/** 
 * Gets the height.
 *
 * @public              @memberof GoRegion3d
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoRegion3d object.
 * @return              The height of the region.(mm)            
 */
GoFx(k64f) GoRegion3d_Height(GoRegion3d region);

/**
* Sets the height.
*
* @public              @memberof GoRegion3d
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    region     GoRegion3d object.
* @param    zAngle     The zAngle to set.(degrees)
* @return              Operation status.
*/
GoFx(kStatus) GoRegion3d_SetZAngle(GoRegion3d region, k64f zAngle);

/**
* Gets the height.
*
* @public              @memberof GoRegion3d
* @note                Supported with G2, G3
* @version             Introduced in firmware 4.0.10.27
* @param    region     GoRegion3d object.
* @return              The zAngle of the region.(degrees)
*/
GoFx(k64f) GoRegion3d_ZAngle(GoRegion3d region);

/**
* @class   GoPointSetRegion
* @extends kObject
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a point set region.
*/
typedef kObject GoPointSetRegion; 

/**
* Sets constant-z value.
*
* @public              @memberof GoPointSetRegion
* @note                Supported with G2, G3
* @version             Introduced in firmware 5.2.18.3
* @param    region     GoPointSetRegion object.
* @param    constantZ  Constant-z value
* @return              Operation status
*/
GoFx(kStatus) GoPointSetRegion_SetConstantZ(GoPointSetRegion region, k64f constantZ);

GoFx(const kPoint3d64f*) GoPointSetRegion_PointAt(GoPointSetRegion region, kSize index);

GoFx(kStatus) GoPointSetRegion_SetPointAt(GoPointSetRegion region, kSize index, const kPoint3d64f* point);

/**
* Disables constant-z.
*
* @public              @memberof GoPointSetRegion
* @note                Supported with G2, G3
* @version             Introduced in firmware 5.2.18.3
* @param    region     GoPointSetRegion object.
* @return              Operation status
*/
GoFx(kStatus) GoPointSetRegion_DisableConstantZ(GoPointSetRegion region);

GoFx(kSize) GoPointSetRegion_PointCount(GoPointSetRegion region);


#if 0
/**
* Sets point values for the region.
*
* @public              @memberof GoPointSetRegion
* @note                Supported with G2, G3
* @version            Introduced in firmware 5.2.18.3
* @param    region     GoPointSetRegion object.
* @param    points     kArrayList of kPoint3d64f. Values are copied.
* @return              Operation status
*/
GoFx(kStatus) GoPointSetRegion_SetPoints(GoPointSetRegion region, const kArrayList* points);

/**
* Gets point values for the region.
*
* @public              @memberof GoPointSetRegion
* @note                Supported with G2, G3
* @version             Introduced in firmware 5.2.18.3
* @param    region     GoPointSetRegion object.
* @param    points     Pre-constructed kArrayList of kPoint3d64f. Values are copied.
* @return              Operation status
*/
GoFx(kStatus) GoPointSetRegion_Points(GoPointSetRegion region, kArrayList* points);
#endif

/**
* @class   GoCylinderRegion
* @extends kObject
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a cylinder region for select surface tools.
*/
typedef kObject GoCylinderRegion; 

/** 
 * Sets the X position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    x          The X position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetX(GoCylinderRegion region, k64f x);

/** 
 * Gets the X position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The X position of the region.(mm)            
 */
GoFx(k64f) GoCylinderRegion_X(GoCylinderRegion region);

/** 
 * Sets the Y position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The Y position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetY(GoCylinderRegion region, k64f value);

/** 
 * Gets the Y position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The Y position of the region.(mm)            
 */
GoFx(k64f) GoCylinderRegion_Y(GoCylinderRegion region);

/** 
 * Sets the Z-position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    z          The Z-position to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetZ(GoCylinderRegion region, k64f z);

/** 
 * Gets the Z-position.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The Z-position of the region.(mm)            
 */
GoFx(k64f) GoCylinderRegion_Z(GoCylinderRegion region);

/** 
 * Sets the radius.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The radius to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetRadius(GoCylinderRegion region, k64f value);

/** 
 * Gets the radius.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The radius of the region.(mm)            
 */
GoFx(k64f) GoCylinderRegion_Radius(GoCylinderRegion region);

/** 
 * Sets the height.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @param    value      The height to set.(mm)
 * @return              Operation status.            
 */
GoFx(kStatus) GoCylinderRegion_SetHeight(GoCylinderRegion region, k64f value);

/** 
 * Gets the height.
 *
 * @public              @memberof GoCylinderRegion
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.0.10.27
 * @param    region     GoCylinderRegion object.
 * @return              The height of the region.(mm)            
 */
GoFx(k64f) GoCylinderRegion_Height(GoCylinderRegion region);


/**
* @class   GoSurfaceFeature
* @extends kObject
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a surface feature for select surface tools.
*/
typedef kObject GoSurfaceFeature; 

/** 
 * Gets the surface feature type.
 *
 * @public                  @memberof GoSurfaceFeature
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  The surface feature type.
 */
GoFx(GoSurfaceFeatureType) GoSurfaceFeature_Type(GoSurfaceFeature feature);

/** 
 * Sets the surface feature type.
 *
 * @public                  @memberof GoSurfaceFeature
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @param   type            The feature type value to set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceFeature_SetType(GoSurfaceFeature feature, GoSurfaceFeatureType type);

/** 
 * Gets the current state the surface feature region.
 *
 * @public                  @memberof GoSurfaceFeature
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoSurfaceFeature_RegionEnabled(GoSurfaceFeature feature);

/** 
 * Enable or disable the surface feature region.
 *
 * @public                  @memberof GoSurfaceFeature
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @param   enable          kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceFeature_EnableRegion(GoSurfaceFeature feature, kBool enable);

/** 
 * Gets the 3d region for the feature.
 *
 * @public                  @memberof GoSurfaceFeature
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.0.10.27
 * @param   feature         GoSurfaceFeature object.
 * @return                  A GoRegion3d object.
 */
GoFx(GoRegion3d) GoSurfaceFeature_Region(GoSurfaceFeature feature);

#include <GoSdk/Tools/GoSurfaceToolUtils.x.h>

#endif
