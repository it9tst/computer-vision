/*
 * AlignmentMoving.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and perform a moving alignment with a calibration disk
 *
 * NOTE! This code requires an encoder to be correctly connected to the system!
 *       Please verify correct encoder operation in the Dashboard page of the Gocator Web UI.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define RECEIVE_TIMEOUT         30000000                    // 30 second timeout for calibration to complete
#define SENSOR_IP               "192.168.1.10"  

void main(int argc, char **argv)
{
    kStatus status;
    unsigned int i;
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoSetup setup = kNULL;
    GoAlignMsg alignMsg = kNULL;
    GoDataSet dataset = kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress;

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return;
    }

    // Parse IP address into address data structure
    kIpAddress_Parse(&ipAddress, SENSOR_IP);

    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return;
    }

    // retrieve setup handle
    if ((setup = GoSensor_Setup(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Setup: Invalid Handle\n");
        return;
    }   

    // configure Gocator to be in travel alignment mode
    if ((status = GoSetup_SetAlignmentType(setup, GO_ALIGNMENT_TYPE_MOVING)) != kOK)
    {
        printf("Error: GoSetup_SetAlignmentType:%d\n", status);
        return;
    }

    // configure travel alignment target
    if ((status = GoSetup_SetAlignmentMovingTarget(setup, GO_ALIGNMENT_TARGET_DISK)) != kOK)
    {
        printf("Error: GoSetup_SetAlignmentMovingTarget:%d\n", status);
        return;
    }

    // set disk diameter
    if ((status = GoSetup_SetDiskDiameter(setup, 40)) != kOK)
    {
        printf("Error: GoSetup_SetDiskDiameter:%d\n", status);
        return;
    }

    //Set up for bar alignment target
    //GoSetup_SetBarWidth(setup, 25);
    //GoSetup_SetBarHeight(setup, 20);
    //GoSetup_SetBarHoleCount(setup, 1);
    //GoSetup_SetBarHoleDiameter(setup, 5);

    // set disk height
    if ((status = GoSetup_SetDiskHeight(setup, 6.25)) != kOK)
    {
        printf("Error: GoSetup_SetDiskHeight:%d\n", status);
        return;
    }

    // enable encoder calibration
    if ((status = GoSetup_EnableAlignmentEncoderCalibrate(setup, kTRUE)) != kOK)
    {
        printf("Error: GoSetup_EnableAlignmentEncoderCalibrate:%d\n", status);
        return;
    }
    
    // start Gocator alignment  
    if ((status = GoSystem_StartAlignment(system)) != kOK)
    {
        printf("Error: GoSystem_StartAlignment:%d\n", status);
        return;
    }

    printf("Waiting for calibration disk to be scanned...\n\n");

    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            dataObj = GoDataSet_At(dataset, i);
            if (GoDataMsg_Type(dataObj) == GO_DATA_MESSAGE_TYPE_ALIGNMENT)
            {
                alignMsg = dataObj;
                if (GoAlignMsg_Status(alignMsg) == kOK)
                {
                    printf("Travel alignment calibration complete.\n\n");
                }
                else
                {
                    printf("Travel alignment calibration failed.\n\n");
                }
            }
        }
        GoDestroy(dataset);
    }
    else
    {
        printf("Travel alignment calibration timed out.\n\n");
    }

    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSystem_Stop:%d\n", status);
        return;
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);

    printf("Press any key to continue...\n");
    getchar();

    return;
}
