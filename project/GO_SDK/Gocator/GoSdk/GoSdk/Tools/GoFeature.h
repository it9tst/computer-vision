/** 
 * @file    GoFeature.h
 * @brief   Declares the GoFeature class. 
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_FEATURE_H
#define GO_FEATURE_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoFeature
 * @extends kObject
 * @ingroup GoSdk
 * @brief   Represents the base class for a tool Feature or script output.
 */
typedef kObject GoFeature; 

/** 
 * Returns whether or not the given Feature has a valid ID assigned to it.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  kTRUE if there is an ID; kFALSE otherwise.            
 */
GoFx(kBool) GoFeature_HasId(GoFeature feature);

/** 
 * Clears the assigned ID for the given Feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoFeature_ClearId(GoFeature feature);

/** 
 * Sets an ID number for the given Feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @param    id             The ID value to set for the feature.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoFeature_SetId(GoFeature feature, k32u id);

/** 
 * Gets the ID for the given feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  The ID value if there is one assigned.  Otherwise, -1 is returned.            
 */
GoFx(k32s) GoFeature_Id(GoFeature feature);

/** 
 * Gets the name for the given feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  A character array pointer for the feature name.
 */
GoFx(const kChar*) GoFeature_Name(GoFeature feature);

/** 
 * Sets the name for the given feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @param    name           The name to assign to the feature.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoFeature_SetName(GoFeature feature, const kChar* name);

/** 
 * Returns the source tool of the given feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  A pointer to the source tool for the feature.
 */
GoFx(kObject) GoFeature_SourceTool(GoFeature feature);

/** 
 * Enables the given feature for output.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @param    enable         Set to kTRUE to enable the feature, kFALSE to disable it.
 * @return                  Operation status.            
 */
GoFx(kStatus) GoFeature_Enable(GoFeature feature, kBool enable);

/** 
 * Returns a boolean value representing whether the given feature is enabled.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  kTRUE if enabled; kFALSE otherwise.            
 */
GoFx(kBool) GoFeature_Enabled(GoFeature feature);

/** 
 * Gets the feature type for the given feature.
 *
 * @public                  @memberof GoFeature
 * @version                 Introduced in firmware 4.6.3.27
 * @param    feature        GoFeature object.
 * @return                  A character array pointer for the feature type.      
 */
GoFx(const kChar*) GoFeature_Type(GoFeature feature);

/**
* Gets the feature data type for the given feature.
*
* @public                  @memberof GoFeature
* @version                 Introduced in firmware 4.6.3.27
* @param    feature        GoFeature object.
* @return                  The feature data type enumerator value.
*/
GoFx(GoFeatureDataType) GoFeature_DataType(GoFeature feature);

/**
* Gets the feature type id for the given feature.
*
* @public                  @memberof GoFeature
* @version                 Introduced in firmware 4.6.3.27
* @param    feature        GoFeature object.
* @return                  The feature type enumerator value.
*/
GoFx(GoFeatureType) GoFeature_TypeId(GoFeature feature);

#include <GoSdk/Tools/GoFeature.x.h>

#endif
