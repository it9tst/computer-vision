/** 
 * @file    GoMeasurements.h
 * @brief   Declares the GoMeasurement classes. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_MEASUREMENTS_H
#define GO_SDK_MEASUREMENTS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdk/Tools/GoExtMeasurement.h>
#include <GoSdk/Tools/GoExtParam.h>

/**
 * @class   GoRangePositionZ
 * @extends GoMeasurement
 * @note    Supported with G1
 * @ingroup GoSdk
 * @brief   Represents a position Z measurement of a Range Position tool.
 */
typedef GoMeasurement GoRangePositionZ;

/**
 * @class   GoRangeThicknessThickness
 * @extends GoMeasurement
 * @note    Supported with G1
 * @ingroup GoSdk
 * @brief   Represents a the thickness measurement of a Range Thickness tool.
 */
typedef GoMeasurement GoRangeThicknessThickness;

/**
 * @class   GoProfileAreaArea
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an area measurement for a Profile Area tool.
 */
typedef GoMeasurement GoProfileAreaArea;

/**
 * @class   GoProfileAreaCentroidX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a centroid X measurement for a Profile Area Tool.
 */
typedef GoMeasurement GoProfileAreaCentroidX;

/**
 * @class   GoProfileAreaCentroidZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a centroid Z measurement for a Profile Area Tool.
 */
typedef GoMeasurement GoProfileAreaCentroidZ;

/**
 * @class   GoProfileBoxX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxX;

/**
 * @class   GoProfileBoxZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxZ;

/**
 * @class   GoProfileBoxWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxWidth;

/**
 * @class   GoProfileBoxHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a height measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxHeight;

/**
 * @class   GoProfileBoxGlobalX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a global X measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxGlobalX;

/**
 * @class   GoProfileBoxGlobalY
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a global Y measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxGlobalY;

/**
 * @class   GoProfileBoxGlobalAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a global angle measurement for a Profile Bounding Box tool.
 */
typedef GoMeasurement GoProfileBoxGlobalAngle;

/**
* @class   GoProfileBridgeValueBridgeValue
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a bridge value measurement for a Profile Bridge Value tool.
*/
typedef GoMeasurement GoProfileBridgeValueBridgeValue;

/**
* @class   GoProfileBridgeValueAngle
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an angle measurement for a Profile Bridge Value tool.
*/
typedef GoMeasurement GoProfileBridgeValueAngle;

/**
* @class   GoProfileBridgeValueWindow
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an window measurement for a Profile Bridge Value tool.
*/
typedef GoMeasurement GoProfileBridgeValueWindow;

/**
* @class   GoProfileBridgeValueStdDev
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an standard deviation measurement for a Profile Bridge Value tool.
*/
typedef GoMeasurement GoProfileBridgeValueStdDev;

/**
 * @class   GoProfileCircleX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleX;

/**
 * @class   GoProfileCircleZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleZ;

/**
 * @class   GoProfileCircleRadius
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a radius value measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleRadius;

/**
 * @class   GoProfileCircleStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a standard deviation measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleStdDev;

/**
 * @class   GoProfileCircleMinError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a min error measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMinError;

/**
 * @class   GoProfileCircleMinErrorX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a min error X measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMinErrorX;

/**
 * @class   GoProfileCircleMinErrorZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a min error Z measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMinErrorZ;

/**
 * @class   GoProfileCircleMaxError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a max error measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMaxError;

/**
 * @class   GoProfileCircleMaxErrorX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a max error X measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMaxErrorX;

/**
 * @class   GoProfileCircleMaxErrorZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a max error Z measurement for a Profile Circle Tool.
 */
typedef GoMeasurement GoProfileCircleMaxErrorZ;

/**
 * @class   GoProfileDimWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimWidth;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileDimWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimWidth object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileDimWidth_AbsoluteEnabled(GoProfileDimWidth measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileDimWidth
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimWidth object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileDimWidth_EnableAbsolute(GoProfileDimWidth measurement, kBool absolute);

/**
 * @class   GoProfileDimHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a height value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimHeight;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoProfileDimHeight 
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimHeight object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileDimHeight_AbsoluteEnabled(GoProfileDimHeight measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoProfileDimHeight
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileDimHeight object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileDimHeight_EnableAbsolute(GoProfileDimHeight measurement, kBool absolute);

/**
 * @class   GoProfileDimDistance
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a distance value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimDistance;

/**
 * @class   GoProfileDimCenterX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a center X value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimCenterX;

/**
 * @class   GoProfileDimCenterZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a center Z value measurement for a Profile Dimension Tool.
 */
typedef GoMeasurement GoProfileDimCenterZ;

/**
 * @class   GoProfileIntersectX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect X measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectX;

/**
 * @class   GoProfileIntersectZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect Z measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectZ;

/**
 * @class   GoProfileIntersectAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an intersect angle measurement for a Profile Intersect Tool.
 */
typedef GoMeasurement GoProfileIntersectAngle;

/** 
 * Returns a boolean value representing whether a absolute measurement value 
 * in the range of 0 to 180 degrees instead of -90 to 90 will be returned.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoProfileIntersectAngle object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoProfileIntersectAngle_Range0to180Enabled(GoProfileIntersectAngle measurement);

/** 
 * Enables or disables a result in the range of 0 to 180 instead of -90 to 90 degrees for the given measurement.
 *
 * @public                  @memberof GoProfileIntersectAngle
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoProfileIntersectAngle object.
 * @param    enable         kTRUE to use a range of 0 to 180 and kFALSE to use a range of -90 to 90 degrees.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileIntersectAngle_EnableRange0to180(GoProfileIntersectAngle measurement, kBool enable);

/**
 * @class   GoProfileGrooveX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X value measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveX;

/** 
 * Gets the current groove location.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX object.
 * @return                  The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileGrooveX_Location(GoProfileGrooveX measurement);

/** 
 * Sets the groove location.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX  object.
 * @param    value          The groove location value to be set.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetLocation(GoProfileGrooveX measurement, GoProfileGrooveLocation value);


/** 
 * Gets the current groove selection type.
 *
 * @public                  @memberof GoProfileGrooveX
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveX object.
 * @return                  The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveX_SelectType(GoProfileGrooveX measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveX
 * @version                      Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveX object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetSelectType(GoProfileGrooveX measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveX object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveX_SelectIndex(GoProfileGrooveX measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveX object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveX_SetSelectIndex(GoProfileGrooveX measurement, k32u selectN);

/**
 * @class   GoProfileGrooveZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z value measurement for a Profile Groove tool.
 */
typedef GoMeasurement GoProfileGrooveZ;

/** 
 * Gets the current groove location type.
 *
 * @public                      @memberof GoProfileGrooveZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveZ object.
 * @return                      The profile groove location type.            
 */
GoFx(GoProfileGrooveLocation) GoProfileGrooveZ_Location(GoProfileGrooveZ measurement);

/** 
 * Sets the groove location type.
 *
 * @public                       @memberof GoProfileGrooveZ
 * @version                      Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveZ object.
 * @param    location            The profile groove location type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetLocation(GoProfileGrooveZ measurement, GoProfileGrooveLocation location);

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveZ object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveZ_SelectType(GoProfileGrooveZ measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveZ object.
 * @param    selectType          The profile groove selection type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetSelectType(GoProfileGrooveZ measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                  @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveZ object.
 * @return                  The current groove index.            
 */
GoFx(k32u) GoProfileGrooveZ_SelectIndex(GoProfileGrooveZ measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                  @memberof GoProfileGrooveZ
 * @version                 Introduced in firmware 4.0.10.27
 * @param    measurement    GoProfileGrooveZ object.
 * @param    selectN        The selected groove index.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoProfileGrooveZ_SetSelectIndex(GoProfileGrooveZ measurement, k32u selectN);


/**
 * @class   GoProfileGrooveWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveWidth;

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveWidth_SelectType(GoProfileGrooveWidth measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveWidth
 * @version                      Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveWidth object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveWidth_SetSelectType(GoProfileGrooveWidth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveWidth_SelectIndex(GoProfileGrooveWidth measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveWidth object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveWidth_SetSelectIndex(GoProfileGrooveWidth measurement, k32u selectN);

/**
 * @class   GoProfileGrooveDepth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a depth measurement for a Profile Groove Tool.
 */
typedef GoMeasurement GoProfileGrooveDepth;

/** 
 * Gets the current groove selection type.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @return                      The profile groove selection type.            
 */
GoFx(GoProfileGrooveSelectType) GoProfileGrooveDepth_SelectType(GoProfileGrooveDepth measurement);

/** 
 * Sets the groove selection type.
 *
 * @public                       @memberof GoProfileGrooveDepth
 * @version                      Introduced in firmware 4.0.10.27
 * @param    measurement         GoProfileGrooveDepth object.
 * @param    selectType          The profile groove type.
 * @return                       Operation status.            
 */
GoFx(kStatus) GoProfileGrooveDepth_SetSelectType(GoProfileGrooveDepth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the current selected groove index.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @return                      The current groove index.            
 */
GoFx(k32u) GoProfileGrooveDepth_SelectIndex(GoProfileGrooveDepth measurement);

/** 
 * Sets the selected groove index.
 *
 * @public                      @memberof GoProfileGrooveDepth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileGrooveDepth object.
 * @param    selectN            The selected groove index.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileGrooveDepth_SetSelectIndex(GoProfileGrooveDepth measurement, k32u selectN);


/**
 * @class   GoProfileLineStdDev
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a standard deviation measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineStdDev;

/**
 * @class   GoProfileLineMinError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a minimum error measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineMinError;

/**
 * @class   GoProfileLineMaxError
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a maximum error measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLineMaxError;

/**
 * @class   GoProfileLinePercentile
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a percentile measurement for a Profile Line Tool.
 */
typedef GoMeasurement GoProfileLinePercentile;

/**
* @class   GoProfileLineOffset
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an offset measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineOffset;

/**
* @class   GoProfileLineAngle
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an Angle measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineAngle;

/**
* @class   GoProfileLineMinErrorX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents an Minimum X Error measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineMinErrorX;

/**
* @class   GoProfileLineMinErrorZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Minimum Z Error measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineMinErrorZ;

/**
* @class   GoProfileLineMaxErrorX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Maximum X Error measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineMaxErrorX;

/**
* @class   GoProfileLineMaxErrorZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Maximum Z Error measurement for a Profile Line Tool.
*/
typedef GoMeasurement GoProfileLineMaxErrorZ;

/** 
 * Gets the percent threshold.
 *
 * @public                      @memberof GoProfileLinePercentile 
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileLinePercentile object.
 * @return                      The percent threshold.
 */
GoFx(k64f) GoProfileLinePercentile_Percent(GoProfileLinePercentile measurement);

/** 
 * Sets the percent threshold.
 *
 * @public                      @memberof GoProfileLinePercentile 
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileLinePercentile object.
 * @param    percent            The percent threshold to set.
 * @return                      Operation status.            
 */
GoFx(kStatus) GoProfileLinePercentile_SetPercent(GoProfileLinePercentile measurement, k64f percent);

/**
 * @class   GoProfilePanelGap
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a gap measurement for a Profile Panel Tool.
 */
typedef GoMeasurement GoProfilePanelGap;

/** 
 * Gets the gap axis.
 *
 * @public                      @memberof GoProfilePanelGap
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelGap object.
 * @return                      The gap axis.    
 */
GoFx(GoProfileGapAxis) GoProfilePanelGap_Axis(GoProfilePanelGap measurement);

/** 
 * Sets the gap axis.
 *
 * @public                      @memberof GoProfilePanelGap
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelGap object.
 * @param    axis               The gap axis value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfilePanelGap_SetAxis(GoProfilePanelGap measurement, GoProfileGapAxis axis);

/**
 * @class   GoProfilePanelFlush
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a flush measurement for a Profile Panel Tool.
 */
typedef GoMeasurement GoProfilePanelFlush;

/** 
 * Gets absolute value state.
 *
 * @public                      @memberof GoProfilePanelFlush
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelFlush object.
 * @return                      kTRUE if absolute value is enabled and kFALSE otherwise.    
 */
GoFx(kBool) GoProfilePanelFlush_AbsoluteEnabled(GoProfilePanelFlush measurement);

/** 
 * Enables or disables the absolute value state.
 *
 * @public                      @memberof GoProfilePanelFlush
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfilePanelFlush object.
 * @param    absolute           kTRUE to enable and kFALSE to disable.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfilePanelFlush_EnableAbsolute(GoProfilePanelFlush measurement, kBool absolute);

/**
* @class   GoProfilePanelLeftGapX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Left Gap X measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelLeftGapX;

/**
* @class   GoProfilePanelLeftGapZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Left Gap Z measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelLeftGapZ;

/**
* @class   GoProfilePanelLeftFlushX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Left Flush X measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelLeftFlushX;

/**
* @class   GoProfilePanelLeftFlushZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Left Flush Z measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelLeftFlushZ;

/**
* @class   GoProfilePanelLeftSurfaceAngle
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Left Surface Angle measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelLeftSurfaceAngle;

/**
* @class   GoProfilePanelRightGapX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Right Gap X measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelRightGapX;

/**
* @class   GoProfilePanelRightGapZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Right Gap Z measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelRightGapZ;

/**
* @class   GoProfilePanelRightFlushX
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Right Flush X measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelRightFlushX;

/**
* @class   GoProfilePanelRightFlushZ
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Right Flush Z measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelRightFlushZ;

/**
* @class   GoProfilePanelRightSurfaceAngle
* @extends GoMeasurement
* @ingroup GoSdk-ProfileTools
* @brief   Represents a Right Surface Angle measurement for a Profile Panel tool.
*/
typedef GoMeasurement GoProfilePanelRightSurfaceAngle;

/**
 * @class   GoProfileRoundCornerX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Round Corner tool.
 */
typedef GoMeasurement GoProfileRoundCornerX;

/**
 * @class   GoProfileRoundCornerZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an Z measurement for a Profile Round Corner tool.
 */
typedef GoMeasurement GoProfileRoundCornerZ;

/**
 * @class   GoProfileRoundCornerAngle
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an Angle measurement for a Profile Round Corner tool.
 */
typedef GoMeasurement GoProfileRoundCornerAngle;

/**
 * @class   GoProfilePositionX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Position tool.
 */
typedef GoMeasurement GoProfilePositionX;

/**
 * @class   GoProfilePositionZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an Z measurement for a Profile Position tool.
 */
typedef GoMeasurement GoProfilePositionZ;


/**
 * @class   GoProfileStripX
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an X measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripX;

/** 
 * Gets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripX_Location(GoProfileStripX measurement);

/** 
 * Sets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    location           The groove location value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetLocation(GoProfileStripX measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripX_SelectType(GoProfileStripX measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetSelectType(GoProfileStripX measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripX_SelectIndex(GoProfileStripX measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripX
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripX object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripX_SetSelectIndex(GoProfileStripX measurement, k32u selectIndex);


/**
 * @class   GoProfileStripZ
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a Z measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripZ;

/** 
 * Gets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The profile groove location.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripZ_Location(GoProfileStripZ measurement);

/** 
 * Sets the groove location configuration value.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    location           The profile groove location to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetLocation(GoProfileStripZ measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripZ_SelectType(GoProfileStripZ measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetSelectType(GoProfileStripZ measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripZ_SelectIndex(GoProfileStripZ measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripZ
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripZ object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripZ_SetSelectIndex(GoProfileStripZ measurement, k32u selectIndex);


/**
 * @class   GoProfileStripWidth
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents a width measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripWidth;

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripWidth_SelectType(GoProfileStripWidth measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @param    selectType         The select type to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripWidth_SetSelectType(GoProfileStripWidth measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripWidth_SelectIndex(GoProfileStripWidth measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripWidth
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripWidth object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripWidth_SetSelectIndex(GoProfileStripWidth measurement, k32u selectIndex);


/**
 * @class   GoProfileStripHeight
 * @extends GoMeasurement
 * @ingroup GoSdk-ProfileTools
 * @brief   Represents an height measurement for a Profile Strip Tool.
 */
typedef GoMeasurement GoProfileStripHeight;

/** 
 * Gets the groove location setting.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The groove location setting.
 */
GoFx(GoProfileGrooveLocation) GoProfileStripHeight_Location(GoProfileStripHeight measurement);

/** 
 * Sets the location.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    location           The location value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetLocation(GoProfileStripHeight measurement, GoProfileGrooveLocation location);

/** 
 * Gets the select type.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The select type.
 */
GoFx(GoProfileGrooveSelectType) GoProfileStripHeight_SelectType(GoProfileStripHeight measurement);

/** 
 * Sets the select type.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    selectType         The select type value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetSelectType(GoProfileStripHeight measurement, GoProfileGrooveSelectType selectType);

/** 
 * Gets the select index.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @return                      The select index.
 */
GoFx(k32u) GoProfileStripHeight_SelectIndex(GoProfileStripHeight measurement);

/** 
 * Sets the select index.
 *
 * @public                      @memberof GoProfileStripHeight
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoProfileStripHeight object.
 * @param    selectIndex        The select index value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoProfileStripHeight_SetSelectIndex(GoProfileStripHeight measurement, k32u selectIndex);

/**
 * @class   GoSurfaceBoxX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxX;

/**
 * @class   GoSurfaceBoxY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxY;

/**
 * @class   GoSurfaceBoxZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxZ;


/**
 * @class   GoSurfaceBoxZAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-angle measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxZAngle;

/**
 * @class   GoSurfaceBoxGlobalX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global X measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalX;

/**
 * @class   GoSurfaceBoxGlobalY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global Y measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalY;

/**
 * @class   GoSurfaceBoxGlobalZAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a global Z angle measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxGlobalZAngle;

/**
 * @class   GoSurfaceBoxLength
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a length measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxLength;

/**
 * @class   GoSurfaceBoxWidth
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a width measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxWidth;

/**
 * @class   GoSurfaceBoxHeight
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a height measurement for a Surface Bounding Box tool.
 */
typedef GoMeasurement GoSurfaceBoxHeight;

/**
 * @class   GoSurfaceDimWidth
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a width value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimWidth;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoSurfaceDimWidth
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimWidth object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceDimWidth_AbsoluteEnabled(GoSurfaceDimWidth measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoSurfaceDimWidth
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimWidth object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceDimWidth_EnableAbsolute(GoSurfaceDimWidth measurement, kBool absolute);

/**
 * @class   GoSurfaceDimHeight
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a height value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimHeight;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoSurfaceDimHeight 
 * @note                     Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimHeight object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceDimHeight_AbsoluteEnabled(GoSurfaceDimHeight measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoSurfaceDimHeight
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimHeight object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceDimHeight_EnableAbsolute(GoSurfaceDimHeight measurement, kBool absolute);

/**
 * @class   GoSurfaceDimLength
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Length value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimLength;

/** 
 * Returns a boolean value representing whether an absolute measurement value will be returned.
 *
 * @public                  @memberof GoSurfaceDimLength
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimLength object.
 * @return                  kTRUE if an absolute value will be returned. kFALSE otherwise.
 */
GoFx(kBool) GoSurfaceDimLength_AbsoluteEnabled(GoSurfaceDimLength measurement);

/** 
 * Enables or disables an absolute value result for the given measurement.
 *
 * @public                  @memberof GoSurfaceDimLength
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param    measurement    GoSurfaceDimLength object.
 * @param    absolute       kTRUE to enable absolute value and kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoSurfaceDimLength_EnableAbsolute(GoSurfaceDimLength measurement, kBool absolute);

/**
 * @class   GoSurfaceDimDistance
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a distance value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimDistance;

/**
 * @class   GoSurfaceDimPlaneDistance
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a distance value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimPlaneDistance;

/**
 * @class   GoSurfaceDimCenterX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a center X value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimCenterX;

/**
 * @class   GoSurfaceDimCenterY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a center Y value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimCenterY;


/**
 * @class   GoSurfaceDimCenterZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a center Z value measurement for a Surface Dimension Tool.
 */
typedef GoMeasurement GoSurfaceDimCenterZ;

/**
 * @class   GoSurfaceEllipseMajor
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a major value measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseMajor;

/**
 * @class   GoSurfaceEllipseMinor
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a minor value measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseMinor;

/**
 * @class   GoSurfaceEllipseRatio
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a ratio measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseRatio;

/**
 * @class   GoSurfaceEllipseZAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-angle measurement for a Surface Ellipse tool.
 */
typedef GoMeasurement GoSurfaceEllipseZAngle;

/**
 * @class   GoSurfaceHoleX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleX;

/**
 * @class   GoSurfaceHoleY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleY;

/**
 * @class   GoSurfaceHoleZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleZ;

/**
 * @class   GoSurfaceHoleRadius
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radius measurement for a Surface Hole Tool.
 */
typedef GoMeasurement GoSurfaceHoleRadius;


/**
 * @class   GoSurfaceOpeningX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningX;

/**
 * @class   GoSurfaceOpeningY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningY;

/**
 * @class   GoSurfaceOpeningZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningZ;

/**
 * @class   GoSurfaceOpeningWidth
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a width measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningWidth;

/**
 * @class   GoSurfaceOpeningLength
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a length measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningLength;

/**
 * @class   GoSurfaceOpeningAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an angle measurement for a Surface Opening Tool.
 */
typedef GoMeasurement GoSurfaceOpeningAngle;


/**
 * @class   GoSurfacePlaneXAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X-angle measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneXAngle;

/**
 * @class   GoSurfacePlaneYAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y-angle measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneYAngle;

/**
 * @class   GoSurfacePlaneZOffset
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z-offset measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneZOffset;

/**
 * @class   GoSurfacePlaneStdDev
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Standard Deviation measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneStdDev;

/**
 * @class   GoSurfacePlaneMinError
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Minimum Error measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneMinError;

/**
 * @class   GoSurfacePlaneMaxError
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Maximum Error measurement for a Surface Plane Tool.
 */
typedef GoMeasurement GoSurfacePlaneMaxError;

/**
* @class   GoSurfacePlaneXNormal
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents the X component of the normal measurement for a Surface Plane Tool.
*/
typedef GoMeasurement GoSurfacePlaneXNormal;

/**
* @class   GoSurfacePlaneYNormal
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents the Y component of the normal measurement for a Surface Plane Tool.
*/
typedef GoMeasurement GoSurfacePlaneYNormal;

/**
* @class   GoSurfacePlaneZNormal
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents the Z component of the normal measurement for a Surface Plane Tool.
*/
typedef GoMeasurement GoSurfacePlaneZNormal;

/**
* @class   GoSurfacePlaneDistance
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents the distance measurement for a Surface Plane Tool.
*/
typedef GoMeasurement GoSurfacePlaneDistance;

/**
* @class   GoSurfaceEdgeX
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents an X measurement for a Surface Edge Tool.
*/
typedef GoMeasurement  GoSurfaceEdgeX;

/**
* @class   GoSurfaceEdgeY
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Y measurement for a Surface Edge Tool.
*/
typedef GoMeasurement  GoSurfaceEdgeY;

/**
* @class   GoSurfaceEdgeZ
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Z measurement for a Surface Edge Tool.
*/
typedef GoMeasurement  GoSurfaceEdgeZ;

/**
* @class   GoSurfacePositionX
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents an X measurement for a Surface Position Tool.
*/
typedef GoMeasurement GoSurfacePositionX;

/**
* @class   GoSurfacePositionY
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Y measurement for a Surface Position Tool.
*/
typedef GoMeasurement GoSurfacePositionY;

/**
* @class   GoSurfacePositionZ
* @extends GoMeasurement
* @note    Supported with G2, G3
* @ingroup GoSdk-SurfaceTools
* @brief   Represents a Z measurement for a Surface Position Tool.
*/
typedef GoMeasurement GoSurfacePositionZ;

/**
 * @class   GoSurfaceStudBaseX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base X measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseX;

/**
 * @class   GoSurfaceStudBaseY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base Y measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseY;

/**
 * @class   GoSurfaceStudBaseZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a base Z measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudBaseZ;

/**
 * @class   GoSurfaceStudTipX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip X measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipX;

/**
 * @class   GoSurfaceStudTipY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip Y measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipY;

/**
 * @class   GoSurfaceStudTipZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a tip Z measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudTipZ;

/**
 * @class   GoSurfaceStudRadius
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a radius measurement for a Surface Stud Tool.
 */
typedef GoMeasurement GoSurfaceStudRadius;

/** 
 * Gets the radius offset.
 *
 * @public                      @memberof GoSurfaceStudRadius
 * @note                        Supported with G2, G3
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceStudRadius object.
 * @return                      The surface location.
 */
GoFx(k64f) GoSurfaceStudRadius_RadiusOffset(GoSurfaceStudRadius measurement);

/** 
 * Sets the radius offset.
 *
 * @public                      @memberof GoSurfaceStudRadius
 * @note                        Supported with G2, G3
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceStudRadius object.
 * @param    value              The offset value to set.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceStudRadius_SetRadiusOffset(GoSurfaceStudRadius measurement, k64f value);

/**
 * @class   GoSurfaceVolumeVolume
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a volume measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeVolume;

/**
 * @class   GoSurfaceVolumeArea
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an area measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeArea;

/**
 * @class   GoSurfaceVolumeThickness
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a thickness measurement for a Surface Volume Tool.
 */
typedef GoMeasurement GoSurfaceVolumeThickness;

/** 
 * Gets the location.
 *
 * @public                      @memberof GoSurfaceVolumeThickness
 * @note                        Supported with G2, G3
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceVolumeThickness object.
 * @return                      The surface location.
 */
GoFx(GoSurfaceLocation) GoSurfaceVolumeThickness_Location(GoSurfaceVolumeThickness measurement);

/** 
 * Sets the location.
 *
 * @public                      @memberof GoSurfaceVolumeThickness
 * @note                        Supported with G2, G3
 * @version                     Introduced in firmware 4.0.10.27
 * @param    measurement        GoSurfaceVolumeThickness object.
 * @param    location           The surface location.
 * @return                      Operation status.
 */
GoFx(kStatus) GoSurfaceVolumeThickness_SetLocation(GoSurfaceVolumeThickness measurement, GoSurfaceLocation location);

/**
 * @class   GoSurfaceCountersunkHoleX
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleX;

/**
 * @class   GoSurfaceCountersunkHoleY
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleY;

/**
 * @class   GoSurfaceCountersunkHoleZ
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Z position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleZ;

/**
 * @class   GoSurfaceCountersunkHoleOuterRadius
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an Outer Radius position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleOuterRadius;

/**
 * @class   GoSurfaceCountersunkHoleDepth
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Depth position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleDepth;

/**
 * @class   GoSurfaceCountersunkHoleBevelRadius
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an Bevel Radius position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleBevelRadius;

/**
 * @class   GoSurfaceCountersunkHoleBevelAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Bevel Angle measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleBevelAngle;

/**
 * @class   GoSurfaceCountersunkHoleXAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents an X Angle position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleXAngle;

/**
 * @class   GoSurfaceCountersunkHoleYAngle
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a Y Angle position measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleYAngle;

/**
 * @class   GoSurfaceCountersunkHoleCounterboreDepth
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a counterbore depth measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleCounterboreDepth;

/**
 * @class   GoSurfaceCountersunkHoleAxisTilt
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a axis tilt measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleAxisTilt;

/**
 * @class   GoSurfaceCountersunkHoleAxisOrientation
 * @extends GoMeasurement
 * @note    Supported with G2, G3
 * @ingroup GoSdk-SurfaceTools
 * @brief   Represents a axis orientation measurement for a Surface Counter Sunk Hole Tool.
 */
typedef GoMeasurement GoSurfaceCountersunkHoleAxisOrientation;


/**
 * @class   GoScriptOutput
 * @extends GoMeasurement
 * @ingroup GoSdk-Tools
 * @brief   Represents a script output for a Script Tool.
 */
typedef GoMeasurement GoScriptOutput;

#include <GoSdk/Tools/GoMeasurements.x.h>

#endif
