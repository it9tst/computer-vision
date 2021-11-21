/** 
 * @file    GoFeatures.h
 * @brief   Declares the GoFeatures classes. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_FEATURES_H
#define GO_SDK_FEATURES_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoFeature.h>
#include <GoSdk/Tools/GoExtParams.h>

typedef GoFeature GoExtFeature;

/**
* Returns the number of custom parameters available for this extensible feature.
*
* @public                  @memberof GoExtFeature
* @version                 Introduced in firmware 4.6.3.142
* @param    feature        GoExtFeature object.
* @return                  Number of extensible parameters.
*/
GoFx(kSize) GoExtFeature_CustomParameterCount(GoExtFeature feature);

/**
* Returns the custom parameter at the specified index.
*
* @public                  @memberof GoExtFeature
* @version                 Introduced in firmware 4.6.3.142
* @param    feature        GoExtFeature object.
* @param    index          Index of parameter to retrieve.
* @return                  The custom feature parameter object.
*/
GoFx(GoExtParam) GoExtFeature_CustomParameterAt(GoExtFeature feature, kSize index);

/**
* @class   GoSurfaceEdgeEdgeLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents an edge line of a Surface Edge tool.
*/
typedef GoFeature GoSurfaceEdgeEdgeLine;



/**
* @class   GoSurfaceEdgeCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of a Surface Edge tool.
*/
typedef GoFeature GoSurfaceEdgeCenterPoint;

/**
* @class   GoSurfaceBoundingBoxCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of a Surface Bounding Box tool.
*/
typedef GoFeature GoSurfaceBoundingBoxCenterPoint;

/**
* @class   GoSurfaceBoundingBoxAxisLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the axis line of a Surface Bounding Box tool.
*/
typedef GoFeature GoSurfaceBoundingBoxAxisLine;

/**
* @class   GoSurfaceCountersunkHoleCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of a Surface Countersunk Hole tool.
*/
typedef GoFeature GoSurfaceCountersunkHoleCenterPoint;

/**
* @class   GoSurfaceDimensionCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point for a Surface Dimension tool.
*/
typedef GoFeature GoSurfaceDimensionCenterPoint;

/**
* @class   GoSurfaceEllipseCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of an Surface Ellipse tool.
*/
typedef GoFeature GoSurfaceEllipseCenterPoint;

/**
* @class   GoSurfaceEllipseMajorAxisLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the major axis line of an Surface Ellipse tool.
*/
typedef GoFeature GoSurfaceEllipseMajorAxisLine;

/**
* @class   GoSurfaceEllipseMinorAxisLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the minor axis line of an Surface Ellipse tool.
*/
typedef GoFeature GoSurfaceEllipseMinorAxisLine;

/**
* @class   GoSurfaceHoleCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of a Surface Hole tool.
*/
typedef GoFeature GoSurfaceHoleCenterPoint;

/**
* @class   GoSurfaceOpeningCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents the center point of a Surface Opening tool.
*/
typedef GoFeature GoSurfaceOpeningCenterPoint;

/**
* @class   GoSurfacePlanePlane
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a plane for a Surface Plane tool.
*/
typedef GoFeature GoSurfacePlanePlane;

/**
* @class   GoSurfacePositionPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of a Surface Position tool.
*/
typedef GoFeature GoSurfacePositionPoint;

/**
* @class   GoSurfaceStudTipPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of a Surface Stud Tip tool.
*/
typedef GoFeature GoSurfaceStudTipPoint;

/**
* @class   GoSurfaceStudBasePoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of a Surface Stud Base tool.
*/
typedef GoFeature GoSurfaceStudBasePoint;

/**
* @class   GoProfilePositionPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of a Profile Position tool.
*/
typedef GoFeature GoProfilePositionPoint;

/**
* @class   GoProfileLineLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of a Profile Line tool.
*/
typedef GoFeature GoProfileLineLine;

/**
* @class   GoProfileLineMinErrorPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of minimum error for a Profile Line tool.
*/
typedef GoFeature GoProfileLineMinErrorPoint;

/**
* @class   GoProfileLineMaxErrorPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of maximum error for a Profile Line tool.
*/
typedef GoFeature GoProfileLineMaxErrorPoint;

/**
* @class   GoProfileIntersectIntersectPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a point of intersection for a Profile Intersect tool.
*/
typedef GoFeature GoProfileIntersectIntersectPoint;

/**
* @class   GoProfileIntersectLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a line for a Profile Intersect tool.
*/
typedef GoFeature GoProfileIntersectLine;

/**
* @class   GoProfileIntersectBaseLine
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a base line feature of a Profile Intersect tool tool.
*/
typedef GoFeature GoProfileIntersectBaseLine;

/**
* @class   GoProfileBoundingBoxCornerPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a corner point for a Profile Bounding Box tool.
*/
typedef GoFeature GoProfileBoundingBoxCornerPoint;

/**
* @class   GoProfileBoundingBoxCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   ReprRepresents a center point for a Profile Bounding Box tool.
*/
typedef GoFeature GoProfileBoundingBoxCenterPoint;

/**
* @class   GoProfileAreaCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a center point of a Profile Area tool.
*/
typedef GoFeature GoProfileAreaCenterPoint;

/**
* @class   GoProfileCircleCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a center point of a Profile Circle tool.
*/
typedef GoFeature GoProfileCircleCenterPoint;

/**
* @class   GoProfileDimensionCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a center point of a Profile Dimension tool.
*/
typedef GoFeature GoProfileDimensionCenterPoint;

/**
* @class   GoProfilePanelLeftGapPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a left gap point of a Profile Panel tool.
*/
typedef GoFeature GoProfilePanelLeftGapPoint;

/**
* @class   GoProfilePanelLeftFlushPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a left flush point of a Profile Panel tool.
*/
typedef GoFeature GoProfilePanelLeftFlushPoint;

/**
* @class   GoProfilePanelRightGapPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a right gap point of a Profile Panel tool.
*/
typedef GoFeature GoProfilePanelRightGapPoint;

/**
* @class   GoProfilePanelRightFlushPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a right flush point of a Profile Panel tool.
*/
typedef GoFeature GoProfilePanelRightFlushPoint;

/**
* @class   GoProfileRoundCornerPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a round corner point of a Profile Round tool.
*/
typedef GoFeature GoProfileRoundCornerPoint;

/**
* @class   GoProfileRoundCornerEdgePoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a round corner edge point of a Profile Round tool.
*/
typedef GoFeature GoProfileRoundCornerEdgePoint;

/**
* @class   GoProfileRoundCornerCenterPoint
* @extends GoFeature
* @ingroup GoSdk
* @brief   Represents a round corner circle center point of a Profile Round tool.
*/
typedef GoFeature GoProfileRoundCornerCenterPoint;

typedef GoFeature GoBoundingBoxCenterPoint;

#include <GoSdk/Tools/GoFeatures.x.h>

#endif
