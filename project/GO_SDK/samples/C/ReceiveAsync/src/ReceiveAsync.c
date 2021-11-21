/*
 * ReceiveASync.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive data using a callback 
 *          function.
 * 
 * Note:    Ethernet output for the desired data must be enabled.
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define SENSOR_IP           "192.168.1.10"

typedef struct DataContext 
{
    k32u count;
} DataContext;

// Data callback function
// This function is called from a separate thread spawned by the GoSDK library.
// Processing within this function should be minimal.
kStatus kCall onData(void* ctx, void* sys, void* dataset)
{
    unsigned int i, j;
    DataContext *context = ctx;

    printf("onData Callback:\n");
    printf("Data message received:\n"); 
    printf("Dataset message count: %u\n", (k32u)GoDataSet_Count(dataset));
    
    for (i = 0; i < GoDataSet_Count(dataset); ++i)
    {
        GoDataMsg dataObj = GoDataSet_At(dataset, i);
        // retrieve GoStamp message
        switch(GoDataMsg_Type(dataObj))
        {
        case GO_DATA_MESSAGE_TYPE_STAMP:    
            {
                GoStampMsg stampMsg = dataObj;
                for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                {
                    GoStamp *stamp = GoStampMsg_At(stampMsg, j);
                    printf("  Timestamp: %llu\n", stamp->timestamp);
                    printf("  Encoder: %lld\n", stamp->encoder); 
                    printf("  Frame index: %llu\n", stamp->frameIndex);                 
                    context->count++;
                }
            }
            break;
            // Refer to example ReceiveRange, ReceiveProfile, ReceiveMeasurement and ReceiveWholePart on how to receive data                
        }               
    }
    GoDestroy(dataset);
    return kOK;
}

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    kStatus status;
    kIpAddress ipAddress;

    DataContext contextPointer;
    
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

    // create connection to GoSystem object
    if ((status = GoSystem_Connect(system)) != kOK)
    {
        printf("Error: GoSystem_Connect:%d\n", status);
        return;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return;
    }
    
    // set data handler to receive data asynchronously
    if ((status = GoSystem_SetDataHandler(system, onData, &contextPointer)) != kOK)
    {
        printf("Error: GoSystem_SetDataHandler:%d\n", status);
        return;
    }   
    
    // start Gocator sensor
    // After this call, the GoSDK library will start receiving data from the 
    // sensor. The data handler function will be called by the SDK in a 
    // separate thread asynchonously.
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }

    printf("Press any key to stop sensor...\n");
    getchar();

    // stop Gocator sensor
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
