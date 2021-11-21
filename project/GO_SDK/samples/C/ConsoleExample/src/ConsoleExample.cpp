/*
 * ConsoleExample.c
 *
 * Gocator Sample
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 *
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Example of console application to configure and operate Gocator sensors
 * Includes: 
 *  - Setup sensor mode
 *  - Setup buddy sensors
 *  - Add measurement tools
 *  - Configue extensible tools (aka GDK-based tools)
 *    (See SetupMeasurment.c for an example for configuring legacy tools.)
 *  - Start scanning and recieve measurements
 */

#include <src/ConsoleExample.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    ConsoleExampleContext context;
    kAssembly assembly = kNULL;
    k32s selection;
    kBool done = kFALSE;
    kStatus exception = kOK;

    context.mainId = GDK_SAMPLE_CLIENT_DEFAULT_MAIN_ID;
    context.buddyId = GDK_SAMPLE_CLIENT_DEFAULT_BUDDY_ID;
    context.lastDataSet = kNULL;
    context.mainDevice = context.buddyDevice = kNULL;

    kCheck(GoSdk_Construct(&assembly));
    kCheck(GoSystem_Construct(&context.system, kNULL));
    
    printf("Please enter a Main device ID to connect to: ");
    scanf("%u", &selection);

    context.mainId = selection;

    kTry
    {
        if (!kSuccess(GoSystem_FindSensorById(context.system, context.mainId, &context.mainDevice)))
        {
            printf("** Error: Could not connect to main device. Exiting application. **");
            
            kThrow(kERROR);
        }

        kTest(GoSensor_Connect(context.mainDevice));

        if (GoSensor_IsRunning(context.mainDevice))
        {
            printf("Sensor is running. Stopping sensor.\n");
        }

        printf("Please enter a Buddy device ID to connect to or -1 to continue in a single device configuration: ");
        scanf("%u", &selection);

        context.buddyId = selection;

        if (context.buddyId != GDK_SAMPLE_CLIENT_DEFAULT_BUDDY_ID)
        {
            kTest(GdkClientSample_ConfigureBuddyDevice(&context));
        }

        while (!done)
        {
            printf("\nExtensible Tools SDK Sample Application\n");
            printf("===================================\n");
            printf("1 - Select a scan mode\n");
            printf("2 - Add a tool\n");
            printf("3 - Configure an extensible tool\n");
            printf("4 - Start/stop the sensor\n");
            printf("5 - Display last received measurement results\n");
            printf("6 - Exit application\n");
            printf("===================================\n");
            printf("Choose an option: ");

            scanf("%u", &selection);

            switch (selection)
            {
            case 1: kTest(GdkClientSample_ModeSelection(&context)); break;
            case 2: kTest(GdkClientSample_AddTools(&context)); break;
            case 3: kTest(GdkClientSample_ConfigureTools(&context)); break;
            case 4: kTest(GdkClientSample_StartStopSensor(&context)); break;
            case 5: kTest(GdkClientSample_DisplayLastMeasurementResults(&context)); break;
            case 6: done = kTRUE; break;
            default: printf("** Invalid selection. Please enter a valid option.** \n"); break;
            }
        }
    }
    kCatchEx(&exception)
    {
        printf("Operation failed: %s\n", kStatus_Name(exception));
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        if (!kIsNull(context.buddyDevice))
        {
            GoSensor_RemoveBuddy(context.mainDevice);
        }

        if (!kIsNull(context.buddyDevice))
        {
            GoSensor_Disconnect(context.mainDevice);
        }

        GoDestroy(context.lastDataSet);
        GoDestroy(context.system);
        GoDestroy(assembly);

        while ((selection = getchar()) != '\n' && selection != EOF);
        printf("\nPress any key to exit.\n");
        getchar(); 

        kEndFinallyEx();
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureBuddyDevice(ConsoleExampleContext* context)
{
    printf("Buddy device specified(ID: %u) - Connecting with buddy...", context->buddyId);
     
    if (kSuccess(GoSystem_FindSensorById(context->system, context->buddyId, &context->buddyDevice)))
    {
        if (!kSuccess(GoSensor_AddBuddy(context->mainDevice, context->buddyDevice)))
        {
            printf("** Error: Could not successfully connect the Buddy device to the Main. **");
            return kERROR;
        }
    }
    else
    {
        printf("** Error: Could not find specified buddy device(ID: %u). **", context->buddyId);
        return kERROR;
    }

    printf("Done!");

    return kOK;
}

kStatus GdkClientSample_ModeSelection(const ConsoleExampleContext* context)
{
    k32u selection;

    printf("\nMode Selection\n");
    printf("==============\n");
    printf("1 - Range mode\n");
    printf("2 - Profile mode(non - resampled)\n");
    printf("3 - Profile mode (resampled)\n");
    printf("4 - Surface mode\n");
    printf("5 - Return to previous menu\n");
    printf("==============\n");
    printf("Please select a mode to run the sensor in:\n");

    scanf("%u", &selection);
    
    switch (selection)
    {
    case 1: kCheck(GdkClientSample_ConfigureRangeMode(context));
        break;
    case 2: kCheck(GdkClientSample_ConfigureProfileMode(context, kFALSE));
        break;
    case 3: kCheck(GdkClientSample_ConfigureProfileMode(context, kTRUE));
        break;
    case 4: kCheck(GdkClientSample_ConfigureSurfaceMode(context));
        break;
    case 5: printf("No changes were made. Returning to main menu.\n"); break;
    default: printf("** Invalid selection. **\n"); break;
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureRangeMode(const ConsoleExampleContext* context)
{
    GoSetup setup = GoSensor_Setup(context->mainDevice);

    if (GoSensor_Family(context->mainDevice) == GO_FAMILY_1000)
    {
        kCheck(GoSetup_SetScanMode(setup, GO_MODE_RANGE));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_MAIN, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_BUDDY, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_MAIN, (GoSetup_ExposureLimitMax(setup, GO_ROLE_MAIN) - GoSetup_ExposureLimitMin(setup, GO_ROLE_MAIN)) / 10));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_BUDDY, (GoSetup_ExposureLimitMax(setup, GO_ROLE_BUDDY) - GoSetup_ExposureLimitMin(setup, GO_ROLE_BUDDY)) / 10));

        printf("Device configured for Range mode with single exposure(%f uS).\n", GoSetup_Exposure(setup, GO_ROLE_MAIN));
    }
    else
    {
        printf("** Invalid mode selected. **\n");
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureProfileMode(const ConsoleExampleContext* context, kBool isResampled)
{
    GoSetup setup = GoSensor_Setup(context->mainDevice);

    if (GoSensor_Family(context->mainDevice) != GO_FAMILY_3000)
    {
        kCheck(GoSetup_SetScanMode(setup, GO_MODE_PROFILE));
        kCheck(GoSetup_EnableUniformSpacing(setup, isResampled));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_MAIN, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_BUDDY, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_MAIN, (GoSetup_ExposureLimitMax(setup, GO_ROLE_MAIN) - GoSetup_ExposureLimitMin(setup, GO_ROLE_MAIN)) / 10));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_BUDDY, (GoSetup_ExposureLimitMax(setup, GO_ROLE_BUDDY) - GoSetup_ExposureLimitMin(setup, GO_ROLE_BUDDY)) / 10));

        printf("Device configured for Profile mode with single exposure(%f uS).\n", GoSetup_Exposure(setup, GO_ROLE_MAIN));
    }
    else
    {
        printf("** Invalid mode selected. **\n");
    }    
    
    return kOK;
}

kStatus GdkClientSample_ConfigureSurfaceMode(const ConsoleExampleContext* context)
{
    GoSetup setup = GoSensor_Setup(context->mainDevice);
    GoSurfaceGeneration surfGen = GoSetup_SurfaceGeneration(setup);
    GoPartDetection partDet = GoSetup_PartDetection(setup);

    if (GoSensor_Family(context->mainDevice) != GO_FAMILY_1000)
    {
        kCheck(GoSetup_SetScanMode(setup, GO_MODE_SURFACE));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_MAIN, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposureMode(setup, GO_ROLE_BUDDY, GO_EXPOSURE_MODE_SINGLE));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_MAIN, (GoSetup_ExposureLimitMax(setup, GO_ROLE_MAIN) - GoSetup_ExposureLimitMin(setup, GO_ROLE_MAIN)) / 10));
        kCheck(GoSetup_SetExposure(setup, GO_ROLE_BUDDY, (GoSetup_ExposureLimitMax(setup, GO_ROLE_BUDDY) - GoSetup_ExposureLimitMin(setup, GO_ROLE_BUDDY)) / 10));

        kCheck(GoSurfaceGeneration_SetGenerationType(surfGen, GO_SURFACE_GENERATION_TYPE_CONTINUOUS));
        kCheck(GoPartDetection_SetMinLength(partDet, GoPartDetection_MinLengthLimitMin(partDet)));
        kCheck(GoPartDetection_SetThreshold(partDet, GoPartDetection_ThresholdLimitMin(partDet)));

        printf("Device configured for Surface mode with single exposure(%f uS) and Continuous surface generation(Min length: %f, Threshold: %f).\n",
            GoSetup_Exposure(setup, GO_ROLE_MAIN),
            GoPartDetection_MinLength(partDet),
            GoPartDetection_Threshold(partDet));
    }
    else
    {
        printf("** Invalid mode selected. **\n");
    }
    
    return kOK;
}

kStatus GdkClientSample_AddTools(const ConsoleExampleContext* context)
{
    GoTools tools = GoSensor_Tools(context->mainDevice);
    kSize i;
    k32u selection;
    
    printf("\nAdd a tool\n");
    printf("==========\n");
    
    for (i = 0; i < GoTools_ToolOptionCount(tools); i++)
    {
        GoToolOption option = GoTools_ToolOptionAt(tools, i);
        printf("%u - %s\n", (k32u)i, GoToolOption_Name(option));
    }

    printf("%u - Return to previous menu.\n", (k32u)GoTools_ToolOptionCount(tools));
    printf("==========\n");

    printf("Please select a tool to add: ");
    scanf("%u", &selection);
    
    if (selection < GoTools_ToolOptionCount(tools) )
    {
        GoTool toolHandler = kNULL;
        if (kSuccess(GoTools_AddToolByName(tools, GoToolOption_Name(GoTools_ToolOptionAt(tools, selection)), &toolHandler)))
        {
            if (kObject_Is(toolHandler, kTypeOf(GoExtTool)))
                printf("Extensible tool added.\n");
            else
                printf("Legacy tool added.\n");
        }
        else
        {
            printf("** Invalid tool selected. **\n");
        }
    }
    else
    {
        if (selection == GoTools_ToolOptionCount(tools))
        {
            return kOK;
        }

        printf("** Failed to add tool. **\n");
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureTools(const ConsoleExampleContext* context)
{
    kSize i, extToolCount;
    k32u selection;
    GoTools tools = GoSensor_Tools(context->mainDevice);
    kBool isDone = kFALSE;

    extToolCount = 0;

    for (i = 0; i < GoTools_ToolCount(tools); i++)
    {
        if (kObject_Is(GoTools_ToolAt(tools, i), kTypeOf(GoExtTool)))
        {
            extToolCount++;
        }
    }

    if (extToolCount == 0)
    {
        printf("There are no extensible tools to configure.\n");
        return kOK;
    }

    while (!isDone)
    {
        printf("\nExtensible tool configuration\n");
        printf("=========================\n");

        for (i = 0; i < GoTools_ToolCount(tools); i++)
        {
            GoTool tool = GoTools_ToolAt(tools, i);

            if (kObject_Is(tool, kTypeOf(GoExtTool)))
            {
                printf("%u - %s\n", (k32u)i, GoExtTool_Type(tool));
            }
        }

        printf("%u - Return to previous menu\n", (k32u)GoTools_ToolCount(tools));
        printf("=========================\n");
        printf("Please select a tool to configure: ");
    
        scanf("%u", &selection);

        if (selection < GoTools_ToolCount(tools)
            && kObject_Is(GoTools_ToolAt(tools, selection), kTypeOf(GoExtTool)))
        {
            GoExtTool tool = GoTools_ToolAt(tools, selection);

            kCheck(GdkClientSample_ConfigureExtTool(context, tool));
        }
        else
        {
            if (selection == GoTools_ToolCount(tools))
            {
                isDone = kTRUE;
            }
            else
            {
                printf("** Invalid tool selection. **\n");
            }
        }
    }
    
    return kOK;
}

kStatus GdkClientSample_ConfigureExtTool(const ConsoleExampleContext* context, GoExtTool tool)
{
    kBool isDone = kFALSE;
    k32u selection = 0;

    while (!isDone)
    {
        printf("\nExtensible tool configuration - %s\n", GoExtTool_DisplayName(tool));
        printf("============================================================\n");
        printf("1 - Edit tool parameters\n");
        printf("2 - Edit measurements\n");
        printf("3 - Return to previous menu\n");
        printf("============================================================\n");
        printf("Choose an option: ");
        
        scanf("%u", &selection);

        switch (selection)
        {
        case 1: kCheck(GdkClientSample_ConfigureExtToolParams(context, tool)); break;
        case 2: kCheck(GdkClientSample_ConfigureMeasurements(context, tool)); break;
        case 3: isDone = kTRUE; break;
        default: printf("** Invalid selection. **\n");
        }
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureMeasurements(const ConsoleExampleContext* context, GoExtTool tool)
{
    kBool isDone = kFALSE;
    kSize i;
    kText32 enabledStr;
    GoExtMeasurement measurement = kNULL;
    k32u selection;
    
    while (!isDone)
    {
        printf("\nConfigure Measurements\n");
        printf("======================\n");

        for (i = 0; i < GoExtTool_MeasurementCount(tool); i++)
        {
            measurement = GoExtTool_MeasurementAt(tool, i);

            if (GoMeasurement_Enabled(measurement))
            {
                kStrCopy(enabledStr, 32, "Enabled");
            }
            else
            {
                kStrCopy(enabledStr, 32, "Disabled");
            }

            printf("%u - %s (%s)\n", (k32u)i, GoMeasurement_Name(measurement), enabledStr);
        }

        printf("%u - Return to previous menu\n", (k32u)GoExtTool_MeasurementCount(tool));
        printf("=======================\n");
        printf("Select a measurement to configure: ");
        scanf("%u", &selection);

        if (selection < GoExtTool_MeasurementCount(tool))
        {
            measurement = GoExtTool_MeasurementAt(tool, selection);
            kCheck(GdkClientSample_ConfigureMeasurement(context, measurement));
        }
        else
        {
            if (selection == GoExtTool_MeasurementCount(tool))
            {
                isDone = kTRUE;
            }
            else 
            {
                printf("** Invalid selection. **\n");
            }
        }
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureMeasurement(const ConsoleExampleContext* context, GoExtMeasurement measurement)
{
    kBool isDone = kFALSE;
    kSize i;
    GoExtParam parameter = kNULL;
    k32u selection;

    while (!isDone)
    {
        printf("\nConfigure %s Measurement\n", GoMeasurement_Name(measurement));
        printf("======================\n");
        printf("0 - %s the measurement\n", GoMeasurement_Enabled(measurement) ? "Disable" : "Enable");

        //standard measurement configuration
        //custom parameters
        for (i = 1; i - 1 < GoExtMeasurement_CustomParameterCount(measurement); i++)
        {
            parameter = GoExtMeasurement_CustomParameterAt(measurement, i - 1);

            printf("%u - %s\n", (k32u)i, GoExtParam_Label(parameter));
        }

        printf("%u - Return to previous menu\n", (k32u)GoExtMeasurement_CustomParameterCount(measurement) + 1);
        printf("=======================\n");
        printf("Select a field to configure: ");
        scanf("%u", &selection);

        switch (selection)
        {
        case 0: {
            kCheck(GoMeasurement_Enable(measurement, !GoMeasurement_Enabled(measurement)));
            kCheck(GoTools_AssignMeasurementId(GoSensor_Tools(context->mainDevice), measurement));
        }; break;
        default: 
            if (selection == (k32u)GoExtMeasurement_CustomParameterCount(measurement) + 1)
            {
                return kOK;
            }

            if (selection - 1 < GoExtMeasurement_CustomParameterCount(measurement))
            {
                parameter = GoExtMeasurement_CustomParameterAt(measurement, selection - 1);
                kCheck(GdkClientSample_ConfigureExtParam(context, parameter));
            }
            else
            {
                if (selection == GoExtMeasurement_CustomParameterCount(measurement))
                {
                    isDone = kTRUE;
                }
                else
                {
                    printf("** Invalid selection. **\n");
                }
            } break;
        }
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureExtToolParams(const ConsoleExampleContext* context, GoExtTool tool)
{
    kBool isDone = kFALSE;
    kSize i;
    GoExtParam param = kNULL;
    k32u selection;

    if (GoExtTool_ParameterCount(tool) == 0)
    {
        printf("No parameters to configure\n");

        return kOK;
    }

    while (!isDone)
    {
        printf("\nTool Parameter Selection\n");
        printf("==========================\n");

        for (i = 0; i < GoExtTool_ParameterCount(tool); i++)
        {
            param = GoExtTool_ParameterAt(tool, i);

            printf("%u - %s\n", (k32u)i, GoExtParam_Label(param));
        }

        printf("%u - Return to previous menu\n", (k32u)GoExtTool_ParameterCount(tool));
        printf("==========================\n");
        printf("Please select a parameter to configure: ");
        scanf("%u", &selection);

        if (selection < GoExtTool_ParameterCount(tool))
        {
            param = GoExtTool_ParameterAt(tool, selection);

            kCheck(GdkClientSample_ConfigureExtParam(context, param));
        }
        else
        {
            if (selection == GoExtTool_ParameterCount(tool))
            {
                isDone = kTRUE;
            }
            else
            {
                printf("** Invalid selection. **\n");
            }
        }
    }    

    return kOK;
}

kStatus GdkClientSample_ConfigureExtParam(const ConsoleExampleContext* context, GoExtParam param)
{
    kText256 limits;
    k32s selection;
    k64f floatSelection;
    kText256 stringSelection = "";
    kSize i;
    kText32 valString = "";
    GoDataStreamId dataStreamId;

    switch (GoExtParam_Type(param))
    {
    case GO_EXT_PARAM_TYPE_INT:
        sprintf(limits, "%s", GoExtParam_Label(param));
        
        if (GoExtParamInt_OptionCount(param) > 0)
        {
            kText128 options = "";

            kStrCat(limits, 256, "(");

            for (i = 0; i < GoExtParamInt_OptionCount(param); i++)
            {
                if (i > 0)
                {
                    kStrCat(options, 128, ", ");
                }
                
                kStrCat(options, 128, GoExtParamInt_OptionDescriptionAt(param, i));
                kStrCat(options, 128, "(");
                sprintf(valString, "%d", GoExtParamInt_OptionValueAt(param, i));
                kStrCat(options, 128, valString);
                kStrCat(options, 128, ")");
            }

            kStrCat(limits, 256, options);
            kStrCat(limits, 256, ")");
        }
        else
        {
            if (GoExtParamInt_IsValueLimitUsed(param))
            {
                kStrCat(limits, 128, " (Min: ");
                sprintf(valString, "%d", GoExtParamInt_ValueMin(param));
                kStrCat(limits, 128, valString);
                kStrCat(limits, 128, " Max: ");
                sprintf(valString, "%d", GoExtParamInt_ValueMax(param));
                kStrCat(limits, 128, valString);
                kStrCat(limits, 128, ")");
            }
        }

        printf("Please enter an integer value for %s (Currently %d): ", limits, GoExtParamInt_Value(param));
        scanf("%i", &selection);
        if (kSuccess(GoExtParamInt_SetValue(param, selection)))
        {
            printf("Value updated.\n");
        }
        else
        {
            printf("** Failed to update value. Please confirm that your input falls within valid bounds. **\n");
        }
        break;
    case GO_EXT_PARAM_TYPE_FLOAT:
        sprintf(limits, "%s", GoExtParam_Label(param));

        if (GoExtParamFloat_OptionCount(param) > 0)
        {
            kText128 options = "";

            kStrCat(limits, 256, "(");

            for (i = 0; i < GoExtParamFloat_OptionCount(param); i++)
            {
                if (i > 0)
                {
                    kStrCat(options, 128, ", ");
                }

                kStrCat(options, 128, GoExtParamFloat_OptionDescriptionAt(param, i));
                kStrCat(options, 128, "(");
                sprintf(valString, "%lf", GoExtParamFloat_OptionValueAt(param, i));
                kStrCat(options, 128, valString);
                kStrCat(options, 128, ")");
            }

            kStrCat(limits, 256, options);
            kStrCat(limits, 256, ")");
        }
        else
        {
            if (GoExtParamFloat_IsValueLimitUsed(param))
            {
                kStrCat(limits, 128, " (Min: ");
                sprintf(valString, "%lf", GoExtParamFloat_ValueMin(param));
                kStrCat(limits, 128, valString);
                kStrCat(limits, 128, " Max: ");
                sprintf(valString, "%lf", GoExtParamFloat_ValueMax(param));
                kStrCat(limits, 128, valString);
                kStrCat(limits, 128, ")");
            }
        }

        printf("Please enter a float value for %s (currently %lf): ", limits, GoExtParamFloat_Value(param));
        scanf("%lf", &floatSelection);
        if (kSuccess(GoExtParamFloat_SetValue(param, floatSelection)))
        {
            printf("Value updated.\n");
        }
        else
        {
            printf("** Failed to update value. Please confirm that your input falls within valid bounds. **\n");
        }
        break;
    case GO_EXT_PARAM_TYPE_BOOL:
        printf("Please enter a boolean value for %s (0 or 1 - Currently %d): ", GoExtParam_Label(param), GoExtParamBool_Value(param));
        scanf("%u", &selection);
        kCheck(GoExtParamBool_SetValue(param, (kBool)selection));
        break;
    case GO_EXT_PARAM_TYPE_STRING:
        printf("Please enter a string value for %s: ", GoExtParam_Label(param));
        scanf("%s", stringSelection);
        kCheck(kString_Set(GoExtParamString_Value(param), stringSelection));
        break;
    case GO_EXT_PARAM_TYPE_DATA_INPUT:
        sprintf(limits, "%s", GoExtParam_Label(param));

        printf("\nData Input Option Selection\n");
        printf("===========================\n");

        if (GoExtParamDataInput_OptionCount(param) > 0)
        {
            for (i = 0; i < GoExtParamDataInput_OptionCount(param); i++)
            {
                dataStreamId = GoExtParamDataInput_OptionValueAt(param, i);
                #if defined(WIN64)
                printf("%llu - %d_%d_%d\n", i, dataStreamId.step, dataStreamId.id, dataStreamId.source);
                #else
                printf("%u - %d_%d_%d\n", (k32u) i, dataStreamId.step, dataStreamId.id, dataStreamId.source);
                #endif
            }

            #if defined(WIN64)
            printf("%llu - Return to previous menu\n", i);
            #else
            printf("%u - Return to previous menu\n", (k32u) i);
            #endif
            
            printf("===========================\n");
        }
        else
        {
            dataStreamId = GoExtParamDataInput_Value(param);
            printf("No options available for parameter <%s> (currently %d_%d_%d)\n", GoExtParam_Label(param), dataStreamId.step, dataStreamId.id, dataStreamId.source);
            printf("===========================\n");
            break;
        }

        dataStreamId = GoExtParamDataInput_Value(param);
        printf("Please enter an option to set for parameter <%s> (currently %d_%d_%d): ", GoExtParam_Label(param), dataStreamId.step, dataStreamId.id, dataStreamId.source);
        scanf("%d", &selection);

        if (selection == (k32s) GoExtParamDataInput_OptionCount(param))
            break;

        if (selection < (k32s) GoExtParamDataInput_OptionCount(param) && kSuccess(GoExtParamDataInput_SetValue(param, GoExtParamDataInput_OptionValueAt(param, selection))))
        {
            printf("Value updated.\n");
        }
        else
        {
            printf("** Failed to update value. Please confirm that your input falls within valid bounds. **\n");
        }
        break;
    case GO_EXT_PARAM_TYPE_PROFILE_REGION: kCheck(GdkClientSample_ConfigureProfileRegion(context, param)); break;
    case GO_EXT_PARAM_TYPE_SURFACE_REGION_2D: kCheck(GdkClientSample_ConfigureSurfaceRegion2d(context, param));  break;
    case GO_EXT_PARAM_TYPE_SURFACE_REGION_3D: kCheck(GdkClientSample_ConfigureSurfaceRegion3d(context, param));  break;
    default: printf("** Error: Unimplemented parameter type. **\n"); return kERROR_PARAMETER;  break;
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureProfileRegion(const ConsoleExampleContext* context, GoExtParamProfileRegion param)
{
    GoProfileRegion region = GoExtParamProfileRegion_Value(param);
    k64f value = k64F_NULL;
    kBool isDone = kFALSE;
    k64u selection = k64U_NULL;
    
    while (!isDone)
    {
        printf("%s configuration\n", GoExtParam_Label(param));
        printf("================================\n");
        printf("1 - Edit X Value (%f)\n", GoProfileRegion_X(region));
        printf("2 - Edit Z Value (%f)\n", GoProfileRegion_Z(region));
        printf("3 - Edit Height Value (%f)\n", GoProfileRegion_Height(region));
        printf("4 - Edit Width Value (%f)\n", GoProfileRegion_Width(region));
        printf("5 - Return to the previous menu\n");
        printf("================================\n");
        printf("Please select a property to edit: ");
        
        scanf("%llu", &selection);
        
        if (selection >= 1 && selection < 5)
        {
            printf("Please enter a value for the property: ");
            scanf("%lf", &value);
        }

        switch (selection)
        {
        case 1: kCheck(GoProfileRegion_SetX(region, value)); break;
        case 2: kCheck(GoProfileRegion_SetZ(region, value)); break;
        case 3: kCheck(GoProfileRegion_SetHeight(region, value)); break;
        case 4: kCheck(GoProfileRegion_SetWidth(region, value)); break;
        case 5: isDone = kTRUE; break;
        default: printf("** Invalid selection. **\n"); break;
        }
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureSurfaceRegion2d(const ConsoleExampleContext* context, GoExtParamSurfaceRegion2d param)
{
    GoSurfaceRegion2d region = GoExtParamSurfaceRegion2d_Value(param);
    k64f value = k64F_NULL;
    kBool isDone = kFALSE;
    k64u selection = k64U_NULL;
    
    while (!isDone)
    {
        printf("%s configuration\n", GoExtParam_Label(param));
        printf("================================\n");
        printf("1 - Edit X Value (%f)\n", GoSurfaceRegion2d_X(region));
        printf("2 - Edit Y Value (%f)\n", GoSurfaceRegion2d_Y(region));
        printf("3 - Edit Length Value (%f)\n", GoSurfaceRegion2d_Length(region));
        printf("4 - Edit Width Value (%f)\n", GoSurfaceRegion2d_Width(region));
        printf("5 - Return to the previous menu\n");
        printf("================================\n");
        printf("Please select a property to edit: ");
        
        scanf("%llu", &selection);

        if (selection >= 1 && selection < 5)
        {
            printf("Please enter a value for the property: ");
            scanf("%lf", &value);
        }

        switch (selection)
        {
        case 1: kCheck(GoSurfaceRegion2d_SetX(region, value)); break;
        case 2: kCheck(GoSurfaceRegion2d_SetY(region, value)); break;
        case 3: kCheck(GoSurfaceRegion2d_SetLength(region, value)); break;
        case 4: kCheck(GoSurfaceRegion2d_SetWidth(region, value)); break;
        case 5: isDone = kTRUE; break;
        default: printf("** Invalid selection. **\n"); break;
        }
    }

    return kOK;
}

kStatus GdkClientSample_ConfigureSurfaceRegion3d(const ConsoleExampleContext* context, GoExtParamSurfaceRegion3d param)
{
    GoRegion3d region = GoExtParamSurfaceRegion3d_Value(param);
    k64f value = k64F_NULL;
    kBool isDone = kFALSE;
    k64u selection = k64U_NULL;
    
    while (!isDone)
    {
        printf("%s configuration\n", GoExtParam_Label(param));
        printf("================================\n");
        printf("1 - Edit X Value (%f)\n", GoRegion3d_X(region));
        printf("2 - Edit Y Value (%f)\n", GoRegion3d_Y(region));
        printf("3 - Edit Z Value (%f)\n", GoRegion3d_Z(region));
        printf("4 - Edit Length Value (%f)\n", GoRegion3d_Length(region));
        printf("5 - Edit Width Value (%f)\n", GoRegion3d_Width(region));
        printf("6 - Edit Height Value (%f)\n", GoRegion3d_Height(region));
        printf("7 - Edit ZAngle Value (%f)\n", GoRegion3d_ZAngle(region));
        printf("8 - Return to the previous menu\n");
        printf("================================\n");
        printf("Please select a property to edit: ");
        
        scanf("%llu", &selection);

        if (selection >= 1 && selection < 8)
        {
            printf("Please enter a value for the property: ");
            scanf("%lf", &value);
        }

        switch (selection)
        {
        case 1: kCheck(GoRegion3d_SetX(region, value)); break;
        case 2: kCheck(GoRegion3d_SetY(region, value)); break;
        case 3: kCheck(GoRegion3d_SetZ(region, value)); break;
        case 4: kCheck(GoRegion3d_SetLength(region, value)); break;
        case 5: kCheck(GoRegion3d_SetWidth(region, value)); break;
        case 6: kCheck(GoRegion3d_SetHeight(region, value)); break;
        case 7: kCheck(GoRegion3d_SetZAngle(region, value)); break;
        case 8: isDone = kTRUE; break;
        default: printf("** Invalid selection. **\n"); break;
        }
    }

    return kOK;
}

kStatus GdkClientSample_StartStopSensor(ConsoleExampleContext* context)
{
    kSize i;
    GoOutput output = GoSensor_Output(context->mainDevice);
    GoEthernet ethernet = GoOutput_Ethernet(output);

    if (GoSensor_IsRunning(context->mainDevice))
    {
        kCheck(GoSensor_Stop(context->mainDevice));
        printf("Sensor stopped.\n");

        return kOK;
    }

    kCheck(GoEthernet_ClearAllSources(ethernet));

    for (i = 0; i < GoEthernet_OptionCount(ethernet, GO_OUTPUT_SOURCE_MEASUREMENT); i++)
    {
        kCheck(GoEthernet_AddSource(ethernet, GO_OUTPUT_SOURCE_MEASUREMENT, GoEthernet_OptionAt(ethernet, GO_OUTPUT_SOURCE_MEASUREMENT, i)));
    }
    
    kCheck(GoSensor_SetDataHandler(context->mainDevice, GdkClientSample_DataHandler, context));
    kCheck(GoSystem_EnableData(context->system, kTRUE));
    kCheck(GoSensor_Start(context->mainDevice));
    printf("Sensor started.\n");

    return kOK;
}

kStatus kCall GdkClientSample_DataHandler(kPointer context, GoSensor sensor, GoDataSet dataSet)
{
    ConsoleExampleContext* obj = (ConsoleExampleContext*)context;

    if (!kIsNull(obj->lastDataSet))
    {
        kCheck(GoDestroy(obj->lastDataSet));
    }

    obj->lastDataSet = dataSet;
    
    return kOK;
}

kStatus GdkClientSample_DisplayLastMeasurementResults(ConsoleExampleContext* context)
{
    kSize i, j;
    kBool showedData = kFALSE;

    if (kIsNull(context->lastDataSet))
    {
        printf("No data to display.\n");
        return kOK;
    }

    for (i = 0; i < GoDataSet_Count(context->lastDataSet); i++)
    {
        kObject msg = GoDataSet_At(context->lastDataSet, i);

        if (kObject_Is(msg, kTypeOf(GoMeasurementMsg)))
        {
            for (j = 0; j < GoMeasurementMsg_Count(msg); j++)
            {
                const GoMeasurementData* data = GoMeasurementMsg_At(msg, j);

                if (data->value == k64F_NULL)
                {
                    printf("ID: %u\tValue: INVALID\tDecision: %u\n", (k32u)GoMeasurementMsg_Id(msg), data->decision);
                }
                else
                {
                    printf("ID: %u\tValue: %f\tDecision: %u\n", (k32u)GoMeasurementMsg_Id(msg), data->value, data->decision);
                }

                showedData = kTRUE;
            }
        }
    }

    if (!showedData)
    {
        printf("No data to display.\n");
    }

    return kOK;
}
