/*
 * AlignmentStationary.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and perform stationary alignment to a flat surface
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define RECEIVE_TIMEOUT         30000000                    // 30 second timeout for calibration to complete
#define SENSOR_IP               "192.168.1.10"              // Sensor IP Address

void main(int argc, char **argv)
{
    kStatus status;
    unsigned int i;
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoSetup setup = kNULL;  
    GoDataSet dataset = kNULL;   
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

    // configure Gocator to be in stationary alignment mode
    if ((status = GoSetup_SetAlignmentType(setup, GO_ALIGNMENT_TYPE_STATIONARY)) != kOK)
    {
        printf("Error: GoSetup_SetAlignmentType:%d\n", status);
        return;
    }

    // configure stationary alignment target to be flat surface
    if ((status = GoSetup_SetAlignmentStationaryTarget(setup, GO_ALIGNMENT_TARGET_NONE)) != kOK)
    {
        printf("Error: GoSetup_SetAlignmentStationaryTarget:%d\n", status);
        return;
    }

    // start Gocator alignment  
    if ((status = GoSystem_StartAlignment(system)) != kOK)
    {
        printf("Error: GoSystem_StartAlignment:%d\n", status);
        return;
    }

    printf("Waiting for alignment calibration data to be collected...\n\n");

    if ((status = GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT)) == kOK)
    {
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);
            if (GoDataMsg_Type(dataObj) == GO_DATA_MESSAGE_TYPE_ALIGNMENT)
            {
                GoAlignMsg alignMsg = dataObj;
                if (GoAlignMsg_Status(alignMsg) == kOK)
                {
                    printf("Alignment calibration complete.\n\n");
                }
                else
                {
                    printf("Alignment calibration failed.\n\n");
                }
            }
        }
        GoDestroy(dataset);
    }
    else if (status == kERROR_TIMEOUT)
    {
        printf("Failed to collect date for calibration within timeout limit...\n");
    }
    else
    {
        printf("Error: GoSystem_ReceiveData:%d\n", status);
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
