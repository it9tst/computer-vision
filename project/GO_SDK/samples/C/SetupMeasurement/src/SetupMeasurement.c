/*
* SetupMeasurement.c
*
* Gocator 2000 Sample
* Copyright (C) 2011-2021 by LMI Technologies Inc.
*
* Licensed under The MIT License.
* Redistributions of files must retain the above copyright notice.
*
* Purpose: Connect to Gocator system, setup measurement data under Profile Mode.
* Please refer to ReceiveMeasurement.c for receiving of the measurement data.
* This cover both the "Legacy" tools through the SetupProfilePositionTool function 
* and "Extensible" (GDK-Based) tools through the SetupFeatureDimensionTool function
*
* See ConsoleExample example for more details on configuring extensible tools. 
* 
* NOTE! This code requires the sensor to not have any conflicting measurments setup already.
*
*/
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define SENSOR_IP                "192.168.1.10"

kStatus SetupProfilePositionTool(GoTools tools, GoSetup setup, kChar* toolName, k32s measId, k32s featId, kChar* featName);
kStatus SetupFeatureDimensionTool(GoTools tools, GoSetup setup, kChar* toolName, k32s toolId, k32s featInId0, k32s featInId1);

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    //GoProfilePositionX positionX = kNULL;
    kIpAddress ipAddress;
    GoSetup setup = kNULL;
    GoTools tools = kNULL;
    GoOutput outputModule = kNULL;
    GoEthernet ethernetOutput = kNULL;

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct: %d\n", status);
        return;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct: %d\n", status);
        return;
    }

    // Parse IP address into address data structure
    if ((status = kIpAddress_Parse(&ipAddress, SENSOR_IP)) != kOK)
    {
        printf("Error: kIpAddress_Parse: %d\n", status);
        return;
    }

    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensorByIpAddress: %d\n", status);
        return;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect: %d\n", status);
        return;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData: %d\n", status);
        return;
    }

    // retrieve setup handle
    if ((setup = GoSensor_Setup(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Setup: Invalid Handle\n");
    }

    // retreive tools handle
    if ((tools = GoSensor_Tools(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Tools: Invalid Handle\n");
    }

    // Set up three tools on the sensor: 2 Profile Positions, and 1 Feature Dimension
    if ((status = SetupProfilePositionTool(tools, setup, "Profile Position Test 0", 10, 20, "Out Feature 0")) != kOK)
    {
        printf("Error: SetupProfilePositionTool: %d\n", status);
        return;
    }
    if ((status = SetupProfilePositionTool(tools, setup, "Profile Position Test 1", 11, 21, "Out Feature 0")) != kOK)
    {
        printf("Error: SetupProfilePositionTool: %d\n", status);
        return;
    }
    if ((status = SetupFeatureDimensionTool(tools, setup, "Feature Dimension Test", 12, 20, 21)) != kOK)
    {
        printf("Error: SetupFeatureDimensionTool: %d\n", status);
        return;
    }


    // enable Ethernet output for measurement tool    

    if ((outputModule = GoSensor_Output(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Output: Invalid Handle\n");
        return;
    }
    if ((ethernetOutput = GoOutput_Ethernet(outputModule)) == kNULL)
    {
        printf("Error: GoOutput_Ethernet: Invalid Handle\n");
        return;
    }
    if ((status = GoEthernet_ClearAllSources(ethernetOutput)) != kOK)
    {
        printf("Error: GoEthernet_ClearAllSources: %d\n", status);
        return;
    }

    // Output the three measrements

    if ((status = GoEthernet_AddSource(ethernetOutput, GO_OUTPUT_SOURCE_MEASUREMENT, 10)) != kOK)
    {
        printf("Error: GoEthernet_AddSource: %d\n", status);
        return;
    }
    
    if ((status = GoEthernet_AddSource(ethernetOutput, GO_OUTPUT_SOURCE_MEASUREMENT, 11)) != kOK)
    {
        printf("Error: GoEthernet_AddSource: %d\n", status);
        return;
    }

    if ((status = GoEthernet_AddSource(ethernetOutput, GO_OUTPUT_SOURCE_MEASUREMENT, 12)) != kOK)
    {
        printf("Error: GoEthernet_AddSource: %d\n", status);
        return;
    }

    // refer to ReceiveMeasurement.c for receiving of the measurement data.

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);

    printf("Press any key to continue...\n");
    getchar();
    return;
}

kStatus SetupProfilePositionTool(GoTools tools, GoSetup setup, kChar* toolName, k32s measId, k32s featId, kChar* featName)
{
    kStatus status = kOK;

    GoProfilePosition        profilePositionTool        = kNULL;
    GoProfilePositionZ        profileMeasurementTopZ    = kNULL;
    GoProfileFeature        profileFeatureTop        = kNULL;
    GoProfileRegion            regionTop                = kNULL;
    GoProfilePositionPoint    outFeaturePoint            = kNULL;

    // add ProfilePosition tool, retreive tool handle
    if ((status = GoTools_AddToolByName(tools, "ProfilePosition", &profilePositionTool)) != kOK)
    {
        printf("Error: GoTools_AddToolByName: %d\n", status);
        return status;
    }

    if ((status = GoTool_SetName(profilePositionTool, toolName)) != kOK)
    {
        printf("Error: GoTool_SetName: %d\n", status);
        return status;
    }

    // Set Feature Output
    if ((outFeaturePoint = GoProfilePosition_Point(profilePositionTool)) == kNULL)
    {
        printf("Error: GoProfilePosition_Point: Invalid Handle\n");
        return status;
    }
    
    if ((status = GoFeature_Enable(outFeaturePoint, kTRUE)) != kOK)
    {
        printf("Error: GoFeature_Enable: %d\n", status);
        return status;
    }
    
    if ((status = GoFeature_SetId(outFeaturePoint, featId)) != kOK)
    {
        printf("Error: GoFeature_SetId: %d\n", status);
        return status;
    }

    if ((status = GoFeature_SetName(outFeaturePoint, featName)) != kOK)
    {
        printf("Error: GoFeature_SetName: %d\n", status);
        return status;
    }

    // Set Z Measurement Output

    // add Z measurement for ProfilePosition tool
    if ((profileMeasurementTopZ = GoProfilePosition_ZMeasurement(profilePositionTool)) == kNULL)
    {
        printf("Error: GoProfilePosition_ZMeasurement: Invalid Handle\n");
        return status;
    }

    // enable zprofileMeasurementTop
    if ((status = GoMeasurement_Enable(profileMeasurementTopZ, kTRUE)) != kOK)
    {
        printf("Error: GoMeasurement_Enable: %d\n", status);
        return status;
    }

    // set the measurement ID for zprofileMeasurementTop
    if ((status = GoMeasurement_SetId(profileMeasurementTopZ, measId)) != kOK)
    {
        printf("Error: GoMeasurement_SetId: %d\n", status);
        return status;
    }

    // set the name for zprofileMeasuermentTop
    if ((status = GoMeasurement_SetName(profileMeasurementTopZ, "Profile Measurement Z")) != kOK)
    {
        printf("Error: GoMeasurement_SetName: %d\n", status);
    }

    // set ProfilePosition feature to top
    if ((profileFeatureTop = GoProfilePosition_Feature(profilePositionTool)) == kNULL)
    {
        printf("Error: GoProfilePosition_Feature: Invalid Handle\n");
    }

    if ((status = GoProfileFeature_SetType(profileFeatureTop, GO_PROFILE_FEATURE_TYPE_MAX_Z)) != kOK)
    {
        printf("Error: GoProfileFeature_SetType: %d\n", status);
    }

    // set the ROI to fill the entire active area
    if ((regionTop = GoProfileFeature_Region(profileFeatureTop)) == kNULL)
    {
        printf("Error: GoProfileFeature_Region: Invalid Handle\n");
    }
    if ((status = GoProfileRegion_SetX(regionTop, GoSetup_TransformedDataRegionX(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: GoProfileRegion_SetX: %d\n", status);
    }
    if ((status = GoProfileRegion_SetZ(regionTop, GoSetup_TransformedDataRegionZ(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: GoProfileRegion_SetZ: %d\n", status);
    }
    if ((status = GoProfileRegion_SetHeight(regionTop, GoSetup_TransformedDataRegionHeight(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: GoProfileRegion_SetHeight: %d\n", status);
    }
    if ((status = GoProfileRegion_SetWidth(regionTop, GoSetup_TransformedDataRegionWidth(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: GoProfileRegion_SetWidth: %d\n", status);
    }

    return kOK;
}
kStatus SetupFeatureDimensionTool(GoTools tools, GoSetup setup, kChar* toolName, k32s toolId, k32s featInId0, k32s featInId1)
{
    kStatus status = kOK;

    GoExtTool           extensibleTool                = kNULL;
    GoMeasurement       profileMeasurementDistance    = kNULL;
    GoExtParamFeature    paramInFeat0                = kNULL;
    GoExtParamFeature    paramInFeat1                = kNULL;

    // add ProfilePosition tool, retreive tool handle
    if ((status = GoTools_AddToolByName(tools, "FeatureDimension", &extensibleTool)) != kOK)
    {
        printf("Error: GoTools_AddToolByName: %d\n", status);
        return status;
    }

    if ((status = GoTool_SetName(extensibleTool, toolName)) != kOK)
    {
        printf("Error: GoTool_SetName: %d\n", status);
        return status;
    }

    // add "Distance" measurement for the extensible tool
    if ((profileMeasurementDistance = GoExtTool_FindMeasurementByName(extensibleTool, "Distance")) == kNULL)
    {
        printf("Error: GoExtTool_FindMeasurementByName: %d\n", status);
        return status;
    }

    // enable Distance measurement
    if ((status = GoMeasurement_Enable(profileMeasurementDistance, kTRUE)) != kOK)
    {
        printf("Error: GoMeasurement_Enable: %d\n", status);
        return status;
    }

    // set the measurement ID for Distance measurement
    if ((status = GoMeasurement_SetId(profileMeasurementDistance, toolId)) != kOK)
    {
        printf("Error: GoMeasurement_SetId: %d\n", status);
        return status;
    }

    // set the name for Distance measurement
    if ((status = GoMeasurement_SetName(profileMeasurementDistance, "Profile Measurement Z")) != kOK)
    {
        printf("Error: GoMeasurement_SetName: %d\n", status);
        return status;
    }

    // set the feature inputs
    if ((paramInFeat0 = GoExtTool_ParameterAt(extensibleTool, 0)) == kNULL)
    {
        printf("Error: GoExtTool_ParameterAt: %d\n", status);
        return status;
    }

    if ((status = GoExtParamFeature_SetFeatureId(paramInFeat0, featInId0)) != kOK)
    {
        printf("Error: GoExtParamFeature_SetFeatureId:%d\n", status);
        return status;
    }

    if ((paramInFeat1 = GoExtTool_ParameterAt(extensibleTool, 1)) == kNULL)
    {
        printf("Error: GoExtTool_ParameterAt: %d\n", status);
        return status;
    }

    if ((status = GoExtParamFeature_SetFeatureId(paramInFeat1, featInId1)) != kOK)
    {
        printf("Error: GoExtParamFeature_SetFeatureId:%d\n", status);
        return status;
    }

    return kOK;
}

