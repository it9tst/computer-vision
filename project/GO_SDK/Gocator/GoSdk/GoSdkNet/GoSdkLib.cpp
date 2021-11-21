//
// GoSdkLib.cpp
//
// Copyright (C) 2017-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#include "GoSdkNet/GoSdkLib.h"
#include "GoSdkNet/GoSdkDef.h"

#include "GoSdkNet/GoAccelerator.h"
#include "GoSdkNet/GoAcceleratorMgr.h"
#include "GoSdkNet/GoAlgorithm.h"
#include "GoSdkNet/GoLayout.h"
#include "GoSdkNet/GoAdvanced.h"
#include "GoSdkNet/GoMaterial.h"
#include "GoSdkNet/GoMultiplexBank.h"
#include "GoSdkNet/GoPartDetection.h"
#include "GoSdkNet/GoPartMatching.h"
#include "GoSdkNet/GoPartModel.h"
#include "GoSdkNet/GoProfileGeneration.h"
#include "GoSdkNet/GoRecordingFilter.h"
#include "GoSdkNet/GoReplay.h"
#include "GoSdkNet/GoReplayCondition.h"
#include "GoSdkNet/GoSection.h"
#include "GoSdkNet/GoSections.h"
#include "GoSdkNet/GoSensor.h"
#include "GoSdkNet/GoSensorInfo.h"
#include "GoSdkNet/GoSetup.h"
#include "GoSdkNet/GoSurfaceGeneration.h"
#include "GoSdkNet/GoSystem.h"
#include "GoSdkNet/GoTracheid.h"
#include "GoSdkNet/GoTransform.h"
#include "GoSdkNet/GoUtils.h"

#include "GoSdkNet/Messages/GoDataSet.h"
#include "GoSdkNet/Messages/GoDataTypes.h"
#include "GoSdkNet/Messages/GoDiscoveryExtInfo.h"
#include "GoSdkNet/Messages/GoHealth.h"

#include "GoSdkNet/Outputs/GoAnalog.h"
#include "GoSdkNet/Outputs/GoDigital.h"
#include "GoSdkNet/Outputs/GoEthernet.h"
#include "GoSdkNet/Outputs/GoOutput.h"
#include "GoSdkNet/Outputs/GoSerial.h"

#include "GoSdkNet/Tools/GoExtParams.h"
#include "GoSdkNet/Tools/GoExtParam.h"
#include "GoSdkNet/Tools/GoExtTool.h"
#include "GoSdkNet/Tools/GoExtToolDataOutput.h"
#include "GoSdkNet/Tools/GoFeature.h"
#include "GoSdkNet/Tools/GoFeatures.h"
#include "GoSdkNet/Tools/GoMeasurement.h"
#include "GoSdkNet/Tools/GoMeasurements.h"
#include "GoSdkNet/Tools/GoProfileTools.h"
#include "GoSdkNet/Tools/GoProfileToolUtils.h"
#include "GoSdkNet/Tools/GoRangeTools.h"
#include "GoSdkNet/Tools/GoSurfaceTools.h"
#include "GoSdkNet/Tools/GoSurfaceToolUtils.h"
#include "GoSdkNet/Tools/GoTool.h"
#include "GoSdkNet/Tools/GoTools.h"

// GoSdkNet assembly properties
[assembly:AssemblyTitleAttribute(L"GoSdkNet")];
[assembly:AssemblyDescriptionAttribute(L"")];
[assembly:AssemblyConfigurationAttribute(L"")];
[assembly:AssemblyCompanyAttribute(L"")];
[assembly:AssemblyProductAttribute(L"GoSdkNet")];
[assembly:AssemblyCopyrightAttribute(L"Copyright (C) 2017 by LMI Technologies Inc. All rights reserved.")];
[assembly:AssemblyTrademarkAttribute(L"")];
[assembly:AssemblyCultureAttribute(L"")];
[assembly:AssemblyVersionAttribute("1.0.*")];
[assembly:ComVisible(false)];
[assembly:CLSCompliantAttribute(true)];

//FSS-817 / GOC-6674
//
//This library uses the 'debuggable' attribute to prevent serious bugs that
//occur when, due to aggressive JIT optimization, an object is finalized
//before the methods that use it have finished executing. This optimization
//is safe in pure .NET code, but it's disastrous for C++/CLI classes that
//handle unmanaged resources. Though there are other ways to solve this
//problem, the use of 'debuggable' is (by far) the most reliable. Given that
//this library implements lightweight wrappers rather than computationally
//intensive logic, the loss of performance caused by this attribute is
//considered acceptable. This attribute should be applied to all assemblies
//that implement kApiNet-based wrappers using C++/CLI.
//
[assembly:System::Diagnostics::DebuggableAttribute(true, true)];

// Definitions for types that do not have dedicated source files
KDefineEnum(Lmi3d::GoSdk::GoUser, GoUser)
KDefineEnum(Lmi3d::GoSdk::GoState, GoState)
KDefineEnum(Lmi3d::GoSdk::GoBuddyState, GoBuddyState)
KDefineEnum(Lmi3d::GoSdk::GoRole, GoRole)
KDefineEnum(Lmi3d::GoSdk::GoAcceleratorConnectionStatus, GoAcceleratorConnectionStatus)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentState, GoAlignmentState)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentRef, GoAlignmentRef)
KDefineEnum(Lmi3d::GoSdk::GoMode, GoMode)
KDefineEnum(Lmi3d::GoSdk::GoTrigger, GoTrigger)
KDefineEnum(Lmi3d::GoSdk::GoEncoderTriggerMode, GoEncoderTriggerMode)
KDefineEnum(Lmi3d::GoSdk::GoFrameRateMaxSource, GoFrameRateMaxSource)
KDefineEnum(Lmi3d::GoSdk::GoEncoderSpacingMinSource, GoEncoderSpacingMinSource)
KDefineEnum(Lmi3d::GoSdk::GoTriggerUnits, GoTriggerUnits)
KDefineEnum(Lmi3d::GoSdk::GoExposureMode, GoExposureMode)
KDefineEnum(Lmi3d::GoSdk::GoOrientation, GoOrientation)
KDefineEnum(Lmi3d::GoSdk::GoInputSource, GoInputSource)
KDefineEnum(Lmi3d::GoSdk::GoIntensitySource, GoIntensitySource)
KDefineEnum(Lmi3d::GoSdk::GoIntensityMode, GoIntensityMode)
KDefineEnum(Lmi3d::GoSdk::GoSeekDirection, GoSeekDirection)
KDefineEnum(Lmi3d::GoSdk::GoDataSource, GoDataSource)
KDefineEnum(Lmi3d::GoSdk::GoSpacingIntervalType, GoSpacingIntervalType)
KDefineEnum(Lmi3d::GoSdk::GoTriggerSource, GoTriggerSource)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentType, GoAlignmentType)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentTarget, GoAlignmentTarget)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentDegreesOfFreedom, GoAlignmentDegreesOfFreedom)
KDefineEnum(Lmi3d::GoSdk::GoReplayExportSourceType, GoReplayExportSourceType)
KDefineEnum(Lmi3d::GoSdk::GoFamily, GoFamily)
KDefineEnum(Lmi3d::GoSdk::GoDecision, GoDecision)
KDefineEnum(Lmi3d::GoSdk::GoDecisionCode, GoDecisionCode)
KDefineEnum(Lmi3d::GoSdk::GoDeviceState, GoDeviceState)
KDefineEnum(Lmi3d::GoSdk::GoAsciiOperation, GoAsciiOperation)
KDefineEnum(Lmi3d::GoSdk::GoAsciiStandardFormatMode, GoAsciiStandardFormatMode)
KDefineEnum(Lmi3d::GoSdk::GoSelcomFormat, GoSelcomFormat)
KDefineEnum(Lmi3d::GoSdk::GoSerialProtocol, GoSerialProtocol)
KDefineEnum(Lmi3d::GoSdk::GoAnalogTrigger, GoAnalogTrigger)
KDefineEnum(Lmi3d::GoSdk::GoDigitalPass, GoDigitalPass)
KDefineEnum(Lmi3d::GoSdk::GoDigitalSignal, GoDigitalSignal)
KDefineEnum(Lmi3d::GoSdk::GoDigitalEvent, GoDigitalEvent)
KDefineEnum(Lmi3d::GoSdk::GoAnalogEvent, GoAnalogEvent)
KDefineEnum(Lmi3d::GoSdk::GoEthernetProtocol, GoEthernetProtocol)
KDefineEnum(Lmi3d::GoSdk::GoEndianType, GoEndianType)
KDefineEnum(Lmi3d::GoSdk::GoOutputSource, GoOutputSource)
KDefineEnum(Lmi3d::GoSdk::GoDataStep, GoDataStep)
KDefineEnum(Lmi3d::GoSdk::GoOutputDelayDomain, GoOutputDelayDomain)
KDefineEnum(Lmi3d::GoSdk::GoPixelType, GoPixelType)
KDefineEnum(Lmi3d::GoSdk::GoToolType, GoToolType)
KDefineEnum(Lmi3d::GoSdk::GoMeasurementType, GoMeasurementType)
KDefineEnum(Lmi3d::GoSdk::GoDataMessageType, GoDataMessageType)
KDefineEnum(Lmi3d::GoSdk::GoAdvancedType, GoAdvancedType)
KDefineEnum(Lmi3d::GoSdk::GoMaterialType, GoMaterialType)
KDefineEnum(Lmi3d::GoSdk::GoReplayConditionType, GoReplayConditionType)
KDefineEnum(Lmi3d::GoSdk::GoReplayCombineType, GoReplayCombineType)
KDefineEnum(Lmi3d::GoSdk::GoReplayMeasurementResult, GoReplayMeasurementResult)
KDefineEnum(Lmi3d::GoSdk::GoReplayRangeCountCase, GoReplayRangeCountCase)
KDefineEnum(Lmi3d::GoSdk::GoSpotSelectionType, GoSpotSelectionType)
KDefineEnum(Lmi3d::GoSdk::GoTranslucentThreadingMode, GoTranslucentThreadingMode)
KDefineEnum(Lmi3d::GoSdk::GoProfileStripBaseType, GoProfileStripBaseType)
KDefineEnum(Lmi3d::GoSdk::GoProfileStripEdgeType, GoProfileStripEdgeType)
KDefineEnum(Lmi3d::GoSdk::GoProfileFeatureType, GoProfileFeatureType)
KDefineEnum(Lmi3d::GoSdk::GoProfileGapAxis, GoProfileGapAxis)
KDefineEnum(Lmi3d::GoSdk::GoProfileEdgeType, GoProfileEdgeType)
KDefineEnum(Lmi3d::GoSdk::GoProfileBaseline, GoProfileBaseline)
KDefineEnum(Lmi3d::GoSdk::GoProfileAreaType, GoProfileAreaType)
KDefineEnum(Lmi3d::GoSdk::GoProfilePanelSide, GoProfilePanelSide)
KDefineEnum(Lmi3d::GoSdk::GoProfileRoundCornerDirection, GoProfileRoundCornerDirection)
KDefineEnum(Lmi3d::GoSdk::GoProfileGrooveShape, GoProfileGrooveShape)
KDefineEnum(Lmi3d::GoSdk::GoProfileGrooveSelectType, GoProfileGrooveSelectType)
KDefineEnum(Lmi3d::GoSdk::GoProfileGrooveLocation, GoProfileGrooveLocation)
KDefineEnum(Lmi3d::GoSdk::GoProfileStripSelectType, GoProfileStripSelectType)
KDefineEnum(Lmi3d::GoSdk::GoProfileStripLocation, GoProfileStripLocation)
KDefineEnum(Lmi3d::GoSdk::GoProfileGenerationType, GoProfileGenerationType)
KDefineEnum(Lmi3d::GoSdk::GoProfileGenerationStartTrigger, GoProfileGenerationStartTrigger)
KDefineEnum(Lmi3d::GoSdk::GoPartFrameOfReference, GoPartFrameOfReference)
KDefineEnum(Lmi3d::GoSdk::GoPartHeightThresholdDirection, GoPartHeightThresholdDirection)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceGenerationType, GoSurfaceGenerationType)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceGenerationStartTrigger, GoSurfaceGenerationStartTrigger)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceLocation, GoSurfaceLocation)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceFeatureType, GoSurfaceFeatureType)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceCountersunkHoleShape, GoSurfaceCountersunkHoleShape)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceOpeningType, GoSurfaceOpeningType)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceRivetType, GoSurfaceRivetType)
KDefineEnum(Lmi3d::GoSdk::GoPartMatchAlgorithm, GoPartMatchAlgorithm)
KDefineEnum(Lmi3d::GoSdk::GoBoxAsymmetryType, GoBoxAsymmetryType)
KDefineEnum(Lmi3d::GoSdk::GoEllipseAsymmetryType, GoEllipseAsymmetryType)
KDefineEnum(Lmi3d::GoSdk::GoImageType, GoImageType)
KDefineEnum(Lmi3d::GoSdk::GoGammaType, GoGammaType)
KDefineEnum(Lmi3d::GoSdk::GoPatternSequenceType, GoPatternSequenceType)
KDefineEnum(Lmi3d::GoSdk::GoImplicitTriggerOverride, GoImplicitTriggerOverride)
KDefineEnum(Lmi3d::GoSdk::GoAlignmentStatus, GoAlignmentStatus)
KDefineEnum(Lmi3d::GoSdk::GoEventType, GoEventType)
KDefineEnum(Lmi3d::GoSdk::GoOcclusionReductionAlg, GoOcclusionReductionAlg)
KDefineEnum(Lmi3d::GoSdk::GoDemosaicStyle, GoDemosaicStyle)
KDefineEnum(Lmi3d::GoSdk::GoSurfaceEncoding, GoSurfaceEncoding)
KDefineEnum(Lmi3d::GoSdk::GoSurfacePhaseFilter, GoSurfacePhaseFilter)
KDefineEnum(Lmi3d::GoSdk::GoFeatureDataType, GoFeatureDataType)
KDefineEnum(Lmi3d::GoSdk::GoDataType, GoDataType)
KDefineEnum(Lmi3d::GoSdk::GoVoltageSetting, GoVoltageSetting)
KDefineEnum(Lmi3d::GoSdk::GoDiscoveryOpMode, GoDiscoveryOpMode)
KDefineEnum(Lmi3d::GoSdk::GoExtParamType, GoExtParamType)
KDefineEnum(Lmi3d::GoSdk::GoSensorAccelState, GoSensorAccelState)
KDefineEnum(Lmi3d::GoSdk::GoSecurityLevel, GoSecurityLevel)
KDefineEnum(Lmi3d::GoSdk::GoBrandingType, GoBrandingType)
KDefineEnum(Lmi3d::GoSdk::GoMeshMsgChannelId, GoMeshMsgChannelId)
KDefineEnum(Lmi3d::GoSdk::GoMeshMsgChannelType, GoMeshMsgChannelType)
KDefineEnum(Lmi3d::GoSdk::GoMeshMsgChannelState, GoMeshMsgChannelState)

KDefineEnum(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgrAccelEvents, GoAcceleratorMgrAccelEvents)

KDefineEnum(Lmi3d::GoSdk::Messages::GoHealthIndicatorId, GoHealthIndicatorId)

KDefineStruct(Lmi3d::GoSdk::GoStates, GoStates)
KDefineStruct(Lmi3d::GoSdk::GoAddressInfo, GoAddressInfo)
KDefineStruct(Lmi3d::GoSdk::GoPortInfo, GoPortInfo)
KDefineStruct(Lmi3d::GoSdk::GoBuddyInfo, GoBuddyInfo)
KDefineStruct(Lmi3d::GoSdk::GoElement64f, GoElement64f)
KDefineStruct(Lmi3d::GoSdk::GoElement32u, GoElement32u)
KDefineStruct(Lmi3d::GoSdk::GoElementBool, GoElementBool)
KDefineStruct(Lmi3d::GoSdk::GoFilter, GoFilter)
KDefineStruct(Lmi3d::GoSdk::GoActiveAreaConfig, GoActiveAreaConfig)
KDefineStruct(Lmi3d::GoSdk::GoTransformation, GoTransformation)
KDefineStruct(Lmi3d::GoSdk::GoTransformedDataRegion, GoTransformedDataRegion)
KDefineStruct(Lmi3d::GoSdk::GoOutputCompositeSource, GoOutputCompositeSource)
KDefineStruct(Lmi3d::GoSdk::GoDataStream, GoDataStream)
KDefineStruct(Lmi3d::GoSdk::GoDataStreamId, GoDataStreamId)
KDefineStruct(Lmi3d::GoSdk::GoFeatureOption, GoFeatureOption)
KDefineStruct(Lmi3d::GoSdk::GoMeasurementOption, GoMeasurementOption)
KDefineStruct(Lmi3d::GoSdk::GoToolDataOutputOption, GoToolDataOutputOption)
KDefineStruct(Lmi3d::GoSdk::GoPolygonCornerParameters, GoPolygonCornerParameters)
KDefineStruct(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgrAccelUpdate, GoAcceleratorMgrAccelUpdate)
KDefineStruct(Lmi3d::GoSdk::AcceleratorMgr::GoAccelSensorPortAllocPorts, GoAccelSensorPortAllocPorts)
KDefineStruct(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgrSensorParam, GoAcceleratorMgrSensorParam)
KDefineStruct(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgrSensorBackup, GoAcceleratorMgrSensorBackup)
KDefineStruct(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgrSensorInfo, GoAcceleratorMgrSensorInfo)

KDefineStruct(Lmi3d::GoSdk::Messages::GoDiscoveryProperty, GoDiscoveryProperty)
KDefineStruct(Lmi3d::GoSdk::Messages::GoStamp, GoStamp)
KDefineStruct(Lmi3d::GoSdk::Messages::GoIndicator, GoIndicator)
KDefineStruct(Lmi3d::GoSdk::Messages::GoMeasurementData, GoMeasurementData)
KDefineStruct(Lmi3d::GoSdk::Messages::GoTracheidEllipse, GoTracheidEllipse)
KDefineStruct(Lmi3d::GoSdk::Messages::Go3dTransform64f, Go3dTransform64f)

KDefineClass(Lmi3d::GoSdk::GoAccelerator, GoAccelerator)
KDefineClass(Lmi3d::GoSdk::AcceleratorMgr::GoAcceleratorMgr, GoAcceleratorMgr)
KDefineClass(Lmi3d::GoSdk::GoLayout, GoLayout)
KDefineClass(Lmi3d::GoSdk::GoAdvanced, GoAdvanced)
KDefineClass(Lmi3d::GoSdk::GoMultiplexBank, GoMultiplexBank)
KDefineClass(Lmi3d::GoSdk::GoPartDetection, GoPartDetection)
KDefineClass(Lmi3d::GoSdk::GoPartMatching, GoPartMatching)
KDefineClass(Lmi3d::GoSdk::GoPartModelEdge, GoPartModelEdge)
KDefineClass(Lmi3d::GoSdk::GoPartModel, GoPartModel)
KDefineClass(Lmi3d::GoSdk::GoProfileGeneration, GoProfileGeneration)
KDefineClass(Lmi3d::GoSdk::GoRecordingFilter, GoRecordingFilter)
KDefineClass(Lmi3d::GoSdk::GoReplay, GoReplay)
KDefineClass(Lmi3d::GoSdk::GoReplayCondition, GoReplayCondition)
KDefineClass(Lmi3d::GoSdk::GoReplayAnyMeasurement, GoReplayAnyMeasurement)
KDefineClass(Lmi3d::GoSdk::GoReplayAnyData, GoReplayAnyData)
KDefineClass(Lmi3d::GoSdk::GoReplayMeasurement, GoReplayMeasurement)
KDefineClass(Lmi3d::GoSdk::GoSection, GoSection)
KDefineClass(Lmi3d::GoSdk::GoSections, GoSections)
KDefineClass(Lmi3d::GoSdk::GoSensor, GoSensor)
KDefineClass(Lmi3d::GoSdk::GoSensorInfo, GoSensorInfo)
KDefineClass(Lmi3d::GoSdk::GoSetup, GoSetup)
KDefineClass(Lmi3d::GoSdk::GoSurfaceGeneration, GoSurfaceGeneration)
KDefineClass(Lmi3d::GoSdk::GoSystem, GoSystem)
KDefineClass(Lmi3d::GoSdk::GoTracheid, GoTracheid)
KDefineClass(Lmi3d::GoSdk::GoTransform, GoTransform)
KDefineClass(Lmi3d::GoSdk::GoGeoCal, GoGeoCal)

KDefineClass(Lmi3d::GoSdk::Messages::GoDataSet, GoDataSet)
KDefineClass(Lmi3d::GoSdk::Messages::GoDataMsg, GoDataMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoStampMsg, GoStampMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoVideoMsg, GoVideoMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoRangeMsg, GoRangeMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoRangeIntensityMsg, GoRangeIntensityMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoProfilePointCloudMsg, GoProfilePointCloudMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoUniformProfileMsg, GoUniformProfileMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoProfileIntensityMsg, GoProfileIntensityMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoUniformSurfaceMsg, GoUniformSurfaceMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoSurfaceIntensityMsg, GoSurfaceIntensityMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoSectionMsg, GoSectionMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoSectionIntensityMsg, GoSectionIntensityMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoMeasurementMsg, GoMeasurementMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoAlignMsg, GoAlignMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoExposureCalMsg, GoExposureCalMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoEdgeMatchMsg, GoEdgeMatchMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoEllipseMatchMsg, GoEllipseMatchMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoBoundingBoxMatchMsg, GoBoundingBoxMatchMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoEventMsg, GoEventMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoDiscoveryExtInfo, GoDiscoveryExtInfo)
KDefineClass(Lmi3d::GoSdk::Messages::GoHealthMsg, GoHealthMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoTracheidMsg, GoTracheidMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoSurfacePointCloudMsg, GoSurfacePointCloudMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoGenericMsg, GoGenericMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoCircleFeatureMsg, GoCircleFeatureMsg )
KDefineClass(Lmi3d::GoSdk::Messages::GoLineFeatureMsg, GoLineFeatureMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoPlaneFeatureMsg, GoPlaneFeatureMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoPointFeatureMsg, GoPointFeatureMsg)
KDefineClass(Lmi3d::GoSdk::Messages::GoMeshMsg, GoMeshMsg)

KDefineClass(Lmi3d::GoSdk::Outputs::GoAnalog, GoAnalog)
KDefineClass(Lmi3d::GoSdk::Outputs::GoDigital, GoDigital)
KDefineClass(Lmi3d::GoSdk::Outputs::GoEthernet, GoEthernet)
KDefineClass(Lmi3d::GoSdk::Outputs::GoOutput, GoOutput)
KDefineClass(Lmi3d::GoSdk::Outputs::GoSerial, GoSerial)

KDefineClass(Lmi3d::GoSdk::Tools::GoTool, GoTool)
KDefineClass(Lmi3d::GoSdk::Tools::GoToolOption, GoToolOption)
KDefineClass(Lmi3d::GoSdk::Tools::GoTools, GoTools)

KDefineClass(Lmi3d::GoSdk::Tools::GoExtTool, GoExtTool)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtToolDataOutput, GoExtToolDataOutput)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileTool, GoProfileTool)
KDefineClass(Lmi3d::GoSdk::Tools::GoRangeTool, GoRangeTool)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceTool, GoSurfaceTool)
KDefineClass(Lmi3d::GoSdk::Tools::GoScript, GoScript)

KDefineClass(Lmi3d::GoSdk::Tools::GoExtParams, GoExtParams)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParam, GoExtParam)

KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamFeature, GoExtParamFeature)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamBool, GoExtParamBool)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamInt, GoExtParamInt)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamFloat, GoExtParamFloat)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamString, GoExtParamString)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamProfileRegion, GoExtParamProfileRegion)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamSurfaceRegion2d, GoExtParamSurfaceRegion2d)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamSurfaceRegion3d, GoExtParamSurfaceRegion3d)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamDataInput, GoExtParamDataInput)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtParamPointSetRegion, GoExtParamPointSetRegion)

KDefineClass(Lmi3d::GoSdk::Tools::GoMeasurement, GoMeasurement)
KDefineClass(Lmi3d::GoSdk::Tools::GoExtMeasurement, GoExtMeasurement)

KDefineClass(Lmi3d::GoSdk::Tools::GoRangePositionZ, GoRangePositionZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoRangeThicknessThickness, GoRangeThicknessThickness)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileAreaArea, GoProfileAreaArea)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileAreaCentroidX, GoProfileAreaCentroidX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileAreaCentroidZ, GoProfileAreaCentroidZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxX, GoProfileBoxX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxZ, GoProfileBoxZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxWidth, GoProfileBoxWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxHeight, GoProfileBoxHeight)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxGlobalX, GoProfileBoxGlobalX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxGlobalY, GoProfileBoxGlobalY)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoxGlobalAngle, GoProfileBoxGlobalAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBridgeValueBridgeValue, GoProfileBridgeValueBridgeValue)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBridgeValueAngle, GoProfileBridgeValueAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBridgeValueWindow, GoProfileBridgeValueWindow)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBridgeValueStdDev, GoProfileBridgeValueStdDev)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleX, GoProfileCircleX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleZ, GoProfileCircleZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleRadius, GoProfileCircleRadius)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleStdDev, GoProfileCircleStdDev)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMinError, GoProfileCircleMinError)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMinErrorX, GoProfileCircleMinErrorX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMinErrorZ, GoProfileCircleMinErrorZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMaxError, GoProfileCircleMaxError)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMaxErrorX, GoProfileCircleMaxErrorX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleMaxErrorZ, GoProfileCircleMaxErrorZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimWidth, GoProfileDimWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimHeight, GoProfileDimHeight)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimDistance, GoProfileDimDistance)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimCenterX, GoProfileDimCenterX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimCenterZ, GoProfileDimCenterZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectX, GoProfileIntersectX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectZ, GoProfileIntersectZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectAngle, GoProfileIntersectAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileGrooveX, GoProfileGrooveX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileGrooveZ, GoProfileGrooveZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileGrooveWidth, GoProfileGrooveWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileGrooveDepth, GoProfileGrooveDepth)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineStdDev, GoProfileLineStdDev)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMinError, GoProfileLineMinError)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMaxError, GoProfileLineMaxError)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineOffset, GoProfileLineOffset)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineAngle, GoProfileLineAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMinErrorX, GoProfileLineMinErrorX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMinErrorZ, GoProfileLineMinErrorZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMaxErrorX, GoProfileLineMaxErrorX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMaxErrorZ, GoProfileLineMaxErrorZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLinePercentile, GoProfileLinePercentile)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelGap, GoProfilePanelGap)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelFlush, GoProfilePanelFlush)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftFlushX, GoProfilePanelLeftFlushX);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftFlushZ, GoProfilePanelLeftFlushZ);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftGapX, GoProfilePanelLeftGapX);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftGapZ, GoProfilePanelLeftGapZ);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftSurfaceAngle, GoProfilePanelLeftSurfaceAngle);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightFlushX, GoProfilePanelRightFlushX);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightFlushZ, GoProfilePanelRightFlushZ);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightGapX, GoProfilePanelRightGapX);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightGapZ, GoProfilePanelRightGapZ);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightSurfaceAngle, GoProfilePanelRightSurfaceAngle);
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerX, GoProfileRoundCornerX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerZ, GoProfileRoundCornerZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerAngle, GoProfileRoundCornerAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePositionX, GoProfilePositionX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePositionZ, GoProfilePositionZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileStripX, GoProfileStripX)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileStripZ, GoProfileStripZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileStripWidth, GoProfileStripWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileStripHeight, GoProfileStripHeight)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxX, GoSurfaceBoxX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxY, GoSurfaceBoxY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxZ, GoSurfaceBoxZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxZAngle, GoSurfaceBoxZAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxGlobalX, GoSurfaceBoxGlobalX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxGlobalY, GoSurfaceBoxGlobalY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxGlobalZAngle, GoSurfaceBoxGlobalZAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxLength, GoSurfaceBoxLength)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxWidth, GoSurfaceBoxWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoxHeight, GoSurfaceBoxHeight)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimWidth, GoSurfaceDimWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimHeight, GoSurfaceDimHeight)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimLength, GoSurfaceDimLength)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimDistance, GoSurfaceDimDistance)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimPlaneDistance, GoSurfaceDimPlaneDistance)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimCenterX, GoSurfaceDimCenterX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimCenterY, GoSurfaceDimCenterY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimCenterZ, GoSurfaceDimCenterZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseMajor, GoSurfaceEllipseMajor)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseMinor, GoSurfaceEllipseMinor)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseRatio, GoSurfaceEllipseRatio)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseZAngle, GoSurfaceEllipseZAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHoleX, GoSurfaceHoleX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHoleY, GoSurfaceHoleY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHoleZ, GoSurfaceHoleZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHoleRadius, GoSurfaceHoleRadius)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningX, GoSurfaceOpeningX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningY, GoSurfaceOpeningY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningZ, GoSurfaceOpeningZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningWidth, GoSurfaceOpeningWidth)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningLength, GoSurfaceOpeningLength)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningAngle, GoSurfaceOpeningAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneXAngle, GoSurfacePlaneXAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneYAngle, GoSurfacePlaneYAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneZOffset, GoSurfacePlaneZOffset)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneStdDev, GoSurfacePlaneStdDev)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneMinError, GoSurfacePlaneMinError)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneMaxError, GoSurfacePlaneMaxError)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneXNormal, GoSurfacePlaneXNormal)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneYNormal, GoSurfacePlaneYNormal)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneZNormal, GoSurfacePlaneZNormal)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlaneDistance, GoSurfacePlaneDistance)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePositionX, GoSurfacePositionX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePositionY, GoSurfacePositionY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePositionZ, GoSurfacePositionZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudBaseX, GoSurfaceStudBaseX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudBaseY, GoSurfaceStudBaseY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudBaseZ, GoSurfaceStudBaseZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudTipX, GoSurfaceStudTipX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudTipY, GoSurfaceStudTipY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudTipZ, GoSurfaceStudTipZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudRadius, GoSurfaceStudRadius)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceVolumeVolume, GoSurfaceVolumeVolume)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceVolumeArea, GoSurfaceVolumeArea)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceVolumeThickness, GoSurfaceVolumeThickness)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleX, GoSurfaceCountersunkHoleX)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleY, GoSurfaceCountersunkHoleY)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleZ, GoSurfaceCountersunkHoleZ)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleOuterRadius, GoSurfaceCountersunkHoleOuterRadius)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleDepth, GoSurfaceCountersunkHoleDepth)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleBevelRadius, GoSurfaceCountersunkHoleBevelRadius)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleBevelAngle, GoSurfaceCountersunkHoleBevelAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleXAngle, GoSurfaceCountersunkHoleXAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleYAngle, GoSurfaceCountersunkHoleYAngle)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleAxisTilt, GoSurfaceCountersunkHoleAxisTilt)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleAxisOrientation, GoSurfaceCountersunkHoleAxisOrientation)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleCounterboreDepth, GoSurfaceCountersunkHoleCounterboreDepth)
KDefineClass(Lmi3d::GoSdk::Tools::GoScriptOutput, GoScriptOutput)

KDefineClass(Lmi3d::GoSdk::Tools::GoProfileArea, GoProfileArea)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBox, GoProfileBox)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBridgeValue, GoProfileBridgeValue)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircle, GoProfileCircle)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDim, GoProfileDim)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileGroove, GoProfileGroove)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersect, GoProfileIntersect)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLine, GoProfileLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanel, GoProfilePanel)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCorner, GoProfileRoundCorner)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePosition, GoProfilePosition)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileStrip, GoProfileStrip)

KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRegion, GoProfileRegion)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileFeature, GoProfileFeature)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineFittingRegion, GoProfileLineFittingRegion)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileEdge, GoProfileEdge)

KDefineClass(Lmi3d::GoSdk::Tools::GoRangePosition, GoRangePosition)
KDefineClass(Lmi3d::GoSdk::Tools::GoRangeThickness, GoRangeThickness)

KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBox, GoSurfaceBox)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHole, GoSurfaceCountersunkHole)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDim, GoSurfaceDim)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipse, GoSurfaceEllipse)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHole, GoSurfaceHole)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpening, GoSurfaceOpening)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlane, GoSurfacePlane)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePosition, GoSurfacePosition)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStud, GoSurfaceStud)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceVolume, GoSurfaceVolume)

KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceRegion2d, GoSurfaceRegion2d)
KDefineClass(Lmi3d::GoSdk::Tools::GoRegion3d, GoRegion3d)
KDefineClass(Lmi3d::GoSdk::Tools::GoCylinderRegion, GoCylinderRegion)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceFeature, GoSurfaceFeature)
KDefineClass(Lmi3d::GoSdk::Tools::GoPointSetRegion, GoPointSetRegion)

KDefineClass(Lmi3d::GoSdk::Tools::GoExtFeature, GoExtFeature)
KDefineClass(Lmi3d::GoSdk::Tools::GoFeature, GoFeature)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoundingBoxAxisLine, GoSurfaceBoundingBoxAxisLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceBoundingBoxCenterPoint, GoSurfaceBoundingBoxCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceCountersunkHoleCenterPoint, GoSurfaceCountersunkHoleCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceDimensionCenterPoint, GoSurfaceDimensionCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseCenterPoint, GoSurfaceEllipseCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseMajorAxisLine, GoSurfaceEllipseMajorAxisLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceEllipseMinorAxisLine, GoSurfaceEllipseMinorAxisLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceHoleCenterPoint, GoSurfaceHoleCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceOpeningCenterPoint, GoSurfaceOpeningCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePlanePlane, GoSurfacePlanePlane)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfacePositionPoint, GoSurfacePositionPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudTipPoint, GoSurfaceStudTipPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoSurfaceStudBasePoint, GoSurfaceStudBasePoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePositionPoint, GoProfilePositionPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineLine, GoProfileLineLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMinErrorPoint, GoProfileLineMinErrorPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileLineMaxErrorPoint, GoProfileLineMaxErrorPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectIntersectPoint, GoProfileIntersectIntersectPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectLineFeature, GoProfileIntersectLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileIntersectBaseLineFeature, GoProfileIntersectBaseLine)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoundingBoxCornerPoint, GoProfileBoundingBoxCornerPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileBoundingBoxCenterPoint, GoProfileBoundingBoxCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileAreaCenterPoint, GoProfileAreaCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileCircleCenterPoint, GoProfileCircleCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileDimensionCenterPoint, GoProfileDimensionCenterPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftGapPoint, GoProfilePanelLeftGapPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelLeftFlushPoint, GoProfilePanelLeftFlushPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightGapPoint, GoProfilePanelRightGapPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfilePanelRightFlushPoint, GoProfilePanelRightFlushPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerPoint, GoProfileRoundCornerPoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerEdgePoint, GoProfileRoundCornerEdgePoint)
KDefineClass(Lmi3d::GoSdk::Tools::GoProfileRoundCornerCenterPoint, GoProfileRoundCornerCenterPoint)
