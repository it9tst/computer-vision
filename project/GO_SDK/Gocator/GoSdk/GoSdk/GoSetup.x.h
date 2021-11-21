/** 
 * @file    GoSetup.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_SETUP_X_H
#define GO_SDK_SETUP_X_H

#include <GoSdk/GoSetup.h>
#include <kApi/Data/kXml.h>

#define GO_MAX_RESOLUTION_COUNT         (3)
#define GO_MAX_EXPOSURE_COUNT           (5)
#define GO_ALIGNMENT_CORNERS_MINIMUM    (3)

typedef struct GoBackgroundSuppression
{
    kBool enabled;
    k64u ratio;

} GoBackgroundSuppression;


typedef struct GoTriggerClass
{
    kObjectClass base;

    GoTriggerSource source;
    kArrayList sourceOptions;
    k32s externalInputIndex;
    kBool externalInputIndexUsed;
    kArrayList externalInputIndexOptions;
    GoTriggerUnits unit;

    GoElement64f frameRate;
    kBool maxFrameRateEnabled;
    GoFrameRateMaxSource frameRateMaxSource;
    k64f tracheidRate;
    k64f frameDataRate;

    GoElement64f encoderSpacing;
    GoEncoderSpacingMinSource encoderSpacingMinSource;
    GoEncoderTriggerMode encoderTriggerMode;

    GoElement64f delay;
    kBool gateEnabled;
    kBool gateEnabledSystemValue;
    kBool gateEnabledUsed;
    k32u burstCount;
    kBool burstCountUsed;
    kBool burstEnabled;
    kBool burstEnabledUsed; 
    
    kBool laserSleepUsed;
    kBool laserSleepEnabled;
    k64u laserSleepIdleTime;
    k64u laserSleepWakeupEncoderTravel;

    kBool reversalDistanceAutoEnabled;
    kBool reversalDistanceAutoEnabledUsed;

    kBool reversalDistanceThresholdUsed;
    k64f reversalDistanceThreshold;
    k64f reversalDistanceThresholdActual;
} GoTriggerClass;

typedef struct GoFiltersClass
{
    kObjectClass base;

    GoFilter xGapFilling;
    GoFilter yGapFilling;
    GoFilter xMedian;
    GoFilter yMedian;
    GoFilter xSmoothing;
    GoFilter ySmoothing;
    GoFilter xDecimation;
    GoFilter yDecimation;
    GoFilter xSlope;
    GoFilter ySlope;
} GoFiltersClass;

typedef struct GoAlignmentClass
{
    kObjectClass base;

    kBool used;
    GoAlignmentType type;
    kArrayList typeOptions;
    GoAlignmentTarget stationaryTarget;
    kArrayList stationaryTargetOptions;
    GoAlignmentTarget movingTarget;
    kArrayList movingTargetOptions;
    kBool inputTriggerEnabled;
    kBool inputTriggerEnabledUsed;
    kBool inputTriggerEnabledSystemValue;
    kBool encoderCalibrateEnabled;
    k64f diskDiameter;
    k64f diskHeight;
    k64f barWidth;
    k64f barHeight;
    kSize barHoleCount;
    kSize barHoleCountValue;
    kBool barHoleCountUsed;
    kArrayList barHoleCountOptions;
    k64f barHoleDistance;
    kBool barHoleDistanceUsed;
    k64f barHoleDiameter;
    kBool barHoleDiameterUsed;
    GoAlignmentDegreesOfFreedom barDegreesOfFreedom;
    kBool barDegreesOfFreedomUsed;
    kArrayList barDegreesOfFreedomOptions;
    k64f plateHeight;
    kSize plateHoleCount;
    k64f plateRefHoleDiameter;
    k64f plateSecHoleDiameter;
    kArrayList polygonCorners;  // of type GoPolygonCornerParameters
} GoAlignmentClass;

typedef struct GoCameraConfig
{
    k32u x; 
    k32u y; 
    k32u width; 
    k32u height; 
} GoCameraConfig;

typedef struct GoTrackingConfig
{
    kBool used;
    kBool enabled;
    GoElement64f trackingAreaHeight;
    k64f searchThreshold;
} GoTrackingConfig;

typedef struct GoTransformConfig
{
    k64f xOffset;
    k64f yOffset;
    k64f zOffset;
    k64f xAngle;
    k64f yAngle;
    k64f zAngle;
} GoTransformConfig;

typedef struct GoIndependentExposuresConfig
{
    kBool used;
    kBool enabled;
    GoElement64f frontCameraExposure;
    GoElement64f backCameraExposure;
} GoIndependentExposuresConfig;

/*
 * GoSetupNode
 */

typedef kObject GoSetupNode; 

typedef struct GoSetupNodeClass
{
    kObjectClass base;   

    kXml configXml;
    kXmlItem configXmlItem;
    kXml transformXml;
    kXmlItem transformXmlItem;
    
    GoRole role;
    k32s index;

    kObject setup;          //setup object (parent)
    kBool hasConfig;
    kBool hasTransform;

    //Configuration elements
    GoActiveAreaConfig activeArea;
    GoTransformedDataRegion transformedDataRegion;

    //Grid used
    kBool gridUsed;
    GoElement32s row;
    GoElement32s column;
    GoElementBool direction;
    GoElement32s multiplexingBank;

    GoCameraConfig frontCamera;
    GoCameraConfig backCamera;
    kBool backCameraUsed;

    GoExposureMode exposureMode;
    kArrayList exposureModeOptions;
    GoElement64f exposure;
    k64f dynamicExposureMax; 
    k64f dynamicExposureMin; 
    k64f exposureSteps[GO_MAX_EXPOSURE_COUNT]; 
    kSize exposureStepCount; 
    GoIntensitySource intensitySource;
    GoIntensityMode intensityMode;
    kBool intensityModeAvail;
    kArrayList intensitySourceOptions;
    kSize intensityStepIndex;
    GoPatternSequenceType patternSequenceType;
    kArrayList patternSequenceTypeOptions;
    kBool patternSequenceTypeUsed;
    kSize patternSequenceCount;
    k32u patternSequenceIndex;
    kBool patternSequenceIndexUsed;
    k32u patternSequenceIndexMin;
    k32u patternSequenceIndexMax;

    GoElement64f spacingInterval;
    GoElement32u spacingIntervalType;
    k32u xSpacingCount;
    k32u ySpacingCount;

    k32u zSubsamplingOptions[GO_MAX_RESOLUTION_COUNT]; 
    kSize zSubsamplingOptionCount; 
    k32u zSubsampling; 
    k32u xSubsamplingOptions[GO_MAX_RESOLUTION_COUNT]; 
    kSize xSubsamplingOptionCount; 
    k32u xSubsampling; 

    GoTrackingConfig tracking;
    GoAdvanced advanced;
    
    GoTracheid tracheid;

    GoIndependentExposuresConfig independentExposures;

} GoSetupNodeClass; 

kDeclareClassEx(Go, GoSetupNode, kObject)

GoFx(kStatus) GoSetupNode_Construct(GoSetupNode* node, kObject setup, GoRole role, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSetupNode_Init(GoSetupNode node, kType type, kObject setup, GoRole role, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSetupNode_VRelease(GoSetupNode node);

GoFx(k32u) GoSetupNode_Index(GoSetupNode node);

GoFx(kStatus) GoSetupNode_ReadConfig(GoSetupNode node, kXml xml, kXmlItem item);
GoFx(kStatus) GoSetupNode_WriteConfig(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_ClearConfig(GoSetupNode node); 
GoFx(kStatus) GoSetupNode_ReadTransform(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_WriteTransform(GoSetupNode node, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetupNode_ClearTransform(GoSetupNode node);

GoFx(k32s) GoSetupNode_Role(GoSetupNode setup);
GoFx(kStatus) GoSetupNode_SetRole(GoSetupNode setup, k32s role);

GoFx(kBool) GoSetupNode_LayoutGridUsed(GoSetupNode node);
GoFx(kBool) GoSetupNode_LayoutGridDirectionSystemValue(GoSetupNode node);
GoFx(kBool) GoSetupNode_LayoutGridDirection(GoSetupNode node);
GoFx(kStatus) GoSetupNode_SetLayoutGridDirection(GoSetupNode node, kBool value);
GoFx(k32s) GoSetupNode_LayoutGridColumnSystemValue(GoSetupNode node);
GoFx(k32s) GoSetupNode_LayoutGridColumn(GoSetupNode node);
GoFx(kStatus) GoSetupNode_SetLayoutGridColumn(GoSetupNode node, k32s value);
GoFx(k32s) GoSetupNode_LayoutGridRowSystemValue(GoSetupNode node);
GoFx(k32s) GoSetupNode_LayoutGridRow(GoSetupNode node);
GoFx(kStatus) GoSetupNode_SetLayoutGridRow(GoSetupNode node, k32s value);
GoFx(kBool) GoSetupNode_LayoutMultiplexingBankUsed(GoSetupNode node);
GoFx(k32u) GoSetupNode_LayoutMultiplexingBankSystemValue(GoSetupNode node);
GoFx(k32u) GoSetupNode_LayoutMultiplexingBank(GoSetupNode node);
GoFx(kStatus) GoSetupNode_SetLayoutMultiplexingBank(GoSetupNode node, k32u value);

/* 
 * GoSetup
 */

typedef struct GoSetupClass
{
    kObjectClass base; 
    kObject sensor;             // sensor (parent object)
    kArrayList nodes;           // device-specific setup (kArrayList<GoSetupNode>
    
    kXml configXml;
    kXmlItem configXmlItem;
    kXml transformXml;
    kXmlItem transformXmlItem;

    //configuration elements
    kBool autoStartEnabled;

    kBool temperatureSafetyEnabled;
    kBool temperatureSafetyUsed;

    GoMode scanMode;
    kArrayList scanModeOptions;
    
    kBool intensityEnabled;
    kBool intensityAvail;
    kBool intensityEnabledSystemValue;

    kBool occlusionReductionEnabled;
    kBool occlusionReductionEnabledSystemValue;
    kBool occlusionReductionEnabledUsed;

    GoOcclusionReductionAlg occlusionReductionAlg;
    kBool occlusionReductionAlgUsed;

    kBool uniformSpacingEnabled;
    kBool uniformSpacingEnabledSystemValue;
    kBool uniformSpacingAvail;

    kBool flickerFreeModeEnabled;
    kBool flickerFreeModeAvail;
    GoBackgroundSuppression backgroundSuppression;

    GoTriggerClass trigger;
    GoFiltersClass filters;
    GoLayout layout;
    GoAlignmentClass alignment;

    GoProfileGeneration profileGeneration;      //profile generation module
    GoSurfaceGeneration surfaceGeneration;      //surface generation module
    GoPartDetection partDetection;              //part detection module
    GoPartMatching partMatching;              
    GoSections sections;                        //section management module

    kBool externalInputZPulseEnabled;
    k32u externalInputZPulseIndex;
    kBool externalInputZPulseIndexAvail;
    kBool preferMasterTimeEncoderEnabled;
} GoSetupClass; 

kDeclareClassEx(Go, GoSetup, kObject)

GoFx(kStatus) GoSetup_Construct(GoSetup* setup, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoSetup_Init(GoSetup setup, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoSetup_VRelease(GoSetup setup);

GoFx(GoSetupNode) GoSetup_FindNode(GoSetup setup, GoRole role);

GoFx(kStatus) GoSetup_ReadConfig(GoSetup setup, kXml xml, kXmlItem item);
GoFx(kStatus) GoSetup_WriteConfig(GoSetup setup, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetup_ReadTransform(GoSetup setup, kXml xml, kXmlItem item); 
GoFx(kStatus) GoSetup_WriteTransform(GoSetup setup, kXml xml, kXmlItem item); 

GoFx(kStatus) GoSetup_EnableTemperatureSafety( GoSetup setup, kBool enable);
GoFx(kBool) GoSetup_TemperatureSafetyEnabled(GoSetup setup);
GoFx(kBool) GoSetup_TemperatureSafetyValueUsed(GoSetup setup);

#endif
