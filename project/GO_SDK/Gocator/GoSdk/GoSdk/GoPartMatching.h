/**
 * @file    GoPartMatching.h
 * @brief   Declares the GoPartMatching class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PART_MATCHING_H
#define GO_PART_MATCHING_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoPartMatching
 * @extends kObject
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents the part matching parameters of the surface mode configuration.
 */
typedef kObject GoPartMatching; 

/** 
 * Enables part matching.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @param   enable          kTRUE to enable, or kFALSE to disable.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoPartMatching_EnablePartMatching(GoPartMatching matching, kBool enable);

/** 
 * Gets the current state of part matching.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  kTRUE if enabled and kFALSE if disabled.
 */
GoFx(kBool) GoPartMatching_PartMatchingEnabled(GoPartMatching matching);

/** 
 * Returns the state of whether or not the user specified part matching value is used.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  kTRUE if used and kFALSE if not.
 */
GoFx(kBool) GoPartMatching_EnablePartMatchingUsed(GoPartMatching matching);

/**
 * Sets the desired part matching algorithm.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    algorithm       The algorithm to use for part matching.
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetAlgorithm(GoPartMatching matching, GoPartMatchAlgorithm algorithm);

/**
 * Gets the currently selected part matching algorithm.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The currently selected part matching algorithm.
 */
GoFx(GoPartMatchAlgorithm) GoParthMatching_Algorithm(GoPartMatching matching);

/**
 * Sets the current edge model name.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    name            The desired name to set for the edge model.
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEdgeModelName(GoPartMatching matching, const kChar* name);

/**
 * Gets the name of the currently selected part matching edge model.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The name of the current edge matching model.
 */
GoFx(const kChar*) GoPartMatching_EdgeModelName(GoPartMatching matching);

/**
 * Sets the edge matching decision minimum quality value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEdgeQualityDecisionMin(GoPartMatching matching, k64f value);

/**
 * Gets the minimum decision value for the edge part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The minimum decision value.
 */
GoFx(k64f) GoPartMatching_EdgeQualityDecisionMin(GoPartMatching matching);

/**
 * Sets the ellipse match major decision maximum value.
 * 
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseMajorMax(GoPartMatching matching, k64f value);

/**
 * Gets the maximum major decision value for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The maximum major decision value.(mm)
 */
GoFx(k64f) GoPartMatching_EllipseMajorMax(GoPartMatching matching);

/**
 * Sets the ellipse match major decision minimum value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseMajorMin(GoPartMatching matching, k64f value);

/**
 * Gets the minimum major decision value for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The minimum major decision value.(mm)
 */
GoFx(k64f) GoPartMatching_EllipseMajorMin(GoPartMatching matching);

/**
 * Sets the ellipse match minor decision maximum value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseMinorMax(GoPartMatching matching, k64f value);

/**
 * Gets the maximum minor decision value for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The maximum minor decision value.(mm)
 */
GoFx(k64f) GoPartMatching_EllipseMinorMax(GoPartMatching matching);

/**
 * Sets the ellipse match minor decision minimum value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseMinorMin(GoPartMatching matching, k64f value);

/**
 * Gets the minimum minor decision value for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The minimum minor decision value.(mm)
 */
GoFx(k64f) GoPartMatching_EllipseMinorMin(GoPartMatching matching);

/**
 * Sets the ellipse match Z angle value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set. (degrees)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseZAngle(GoPartMatching matching, k64f value);

/**
 * Gets the ellipse Z angle value for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The Z angle value.(degrees)
 */
GoFx(k64f) GoPartMatching_EllipseZAngle(GoPartMatching matching);

/**
 * Sets the ellipse match decision asymmetry detection type.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.4.4.14
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetEllipseAsymmetryDetectionType(GoPartMatching matching, GoEllipseAsymmetryType value);

/**
 * Gets the asymmetry detection type for the ellipse part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param   matching        GoPartMatching object.
 * @return                  The asymmetry detection type.
 */
GoFx(GoEllipseAsymmetryType) GoPartMatching_EllipseAsymmetryDetectionType(GoPartMatching matching);

/**
 * Sets the bounding box match decision maximum width value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxWidthMax(GoPartMatching matching, k64f value);

/**
 * Gets the maximum width decision value for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The maximum width decision value.(mm)
 */
GoFx(k64f) GoPartMatching_BoundingBoxWidthMax(GoPartMatching matching);

/**
 * Sets the bounding box match decision minimum width value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxWidthMin(GoPartMatching matching, k64f value);

/**
 * Gets the minimum width decision value for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The minimum width decision value.(mm)
 */
GoFx(k64f) GoPartMatching_BoundingBoxWidthMin(GoPartMatching matching);

/**
 * Sets the bounding box match decision maximum length value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxLengthMax(GoPartMatching matching, k64f value);

/**
 * Gets the maximum length decision value for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The maximum length decision value.(mm)
 */
GoFx(k64f) GoPartMatching_BoundingBoxLengthMax(GoPartMatching matching);

/**
 * Sets the bounding box match decision minimum length value.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.2.4.7
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(mm)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxLengthMin(GoPartMatching matching, k64f value);

/**
 * Gets the minimum length decision value for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The minimum length decision value.(mm)
 */
GoFx(k64f) GoPartMatching_BoundingBoxLengthMin(GoPartMatching matching);

/**
 * Sets the bounding box match decision Z angle value.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param    matching       GoPartMatching object.
 * @param    value          The value to set.(mm)
 * @return                  Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxZAngle(GoPartMatching matching, k64f value);

/**
 * Gets the Z angle value for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.2.4.7
 * @param   matching        GoPartMatching object.
 * @return                  The Z angle value.(degrees)
 */
GoFx(k64f) GoPartMatching_BoundingBoxZAngle(GoPartMatching matching);

/**
 * Sets the bounding box match decision asymmetry detection type.
 *
 * @public                   @memberof GoPartMatching
 * @note                     Supported with G2, G3
 * @version                  Introduced in firmware 4.4.4.14
 * @param    matching        GoPartMatching object.
 * @param    value           The value to set.(degrees)
 * @return                   Operation status.
 */
GoFx(kStatus) GoPartMatching_SetBoundingBoxAsymmetryDetectionType(GoPartMatching matching, GoBoxAsymmetryType value);

/**
 * Gets the asymmetry detection type for the bounding box part matching configuration.
 *
 * @public                  @memberof GoPartMatching
 * @note                    Supported with G2, G3
 * @version                 Introduced in firmware 4.4.4.14
 * @param   matching        GoPartMatching object.
 * @return                  The asymmetry detection type.
 */
GoFx(GoBoxAsymmetryType) GoPartMatching_BoundingBoxAsymmetryDetectionType(GoPartMatching matching);

#include <GoSdk/GoPartMatching.x.h>

#endif
