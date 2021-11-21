/*
 * ReceivePartSegments.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive data using a callback function.
 * Ethernet output for the desired data must be enabled.
 * Specific case demonstrated for receiving Parts from Part Segmentation tool
 * using Generic IOs.
 *
 */

#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "GenericBlob.h"

#define AUTOSTOP

#define SENSOR_IP           "192.168.1.10"

kBool stoppingSensor = kFALSE;


typedef struct DataContext 
{
    k32u count;
} DataContext;


// data callback function
kStatus kCall onData(void* ctx, void* sys, void* dataset)
{
    unsigned int i, j;
    DataContext *context = (DataContext*)ctx;


#ifdef AUTOSTOP
    //Autostop after first callback
    stoppingSensor = kTRUE;
#endif


    printf("----------\nDataset message count: %u\n", GoDataSet_Count(dataset));
    
    for (i = 0; i < GoDataSet_Count(dataset); ++i)
    {
        GoDataMsg dataObj = GoDataSet_At(dataset, i);
        GoDataMessageType type = GoDataMsg_Type(dataObj);
        printf("Type: %d\n", type);

        switch(type)
        {
        default:
            {
                printf("* Other Message Type: %d\n", type);
            }
            break;
        case GO_COMPACT_MESSAGE_GENERIC:
            {
                GoGenericMsg genMsg = dataObj;
                std::vector<BlobProperty> parts;
                DeserializeBlobs(genMsg, parts);

                // Process using Parts information

                k32u utype = GoGenericMsg_UserType(genMsg);
                printf("  Generic IO\n");
                printf("    Generic Type: %x\n", utype);
                printf("    Message Length: %u\n", GoGenericMsg_BufferSize(genMsg));
                printf("    Representing Parts: %s\n", GoGenericMsgTypeValid(genMsg) ? "yes": "no");

                if (GoGenericMsgTypeValid(genMsg))
                {
                    printf("    Count: %d\n", (int)parts.size());

                    int k = 0;
                    for (auto part : parts)
                    {
                        printf("    Part %d\n", k++);
                        printf("      Area: %f\n", part.GetArea());
                        printf("      Height: %f\n", part.GetCenterHeight());
                        printf("      Center X: %f\n", part.GetCenter().x);
                        printf("      Center Y: %f\n", part.GetCenter().y);
                    }
                }
            }
            break;
        case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
            {
                printf("  Surface\n");
            }
            break;
        case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj;
                printf("  Stamp\n");

                for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                {
                    GoStamp *stamp = GoStampMsg_At(stampMsg, j);
                    printf("    Timestamp: %llu\n", stamp->timestamp);
                    printf("    Encoder: %lld\n", stamp->encoder);
                    printf("    Frame index: %llu\n", stamp->frameIndex);
                    context->count++;
                }
            }
            break;
        case GO_DATA_MESSAGE_TYPE_MEASUREMENT:
            {
                GoMeasurementMsg measurementMsg = dataObj;
                GoMeasurementData *measurementData = kNULL;

                printf("  Measurement Message batch count: %u\n", GoMeasurementMsg_Count(measurementMsg));

                for (j = 0; j < GoMeasurementMsg_Count(measurementMsg); ++j)
                {
                    measurementData = GoMeasurementMsg_At(measurementMsg, j);
                    printf("    Measurement ID: %u\n", GoMeasurementMsg_Id(measurementMsg));
                    printf("    Measurement Value: %.1f\n", measurementData->value);
                    printf("    Measurement Decision: %d\n", measurementData->decision);
                }
            }
            break;
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
    if ((status = GoSensor_Connect(sensor)) != kOK)
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
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }

#ifdef AUTOSTOP
    k32s times = 0;
    while (!stoppingSensor)
    {
        kThread_Sleep(1000000);
        times++;
        if (times > 10)
            break;
    }
#else
    printf("Press any key to stop sensor...\n");
    getchar();
#endif

    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSystem_Stop:%d\n", status);
        return;
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);

    printf("\nPress any key to continue...\n");
    getchar();
    return;
}
