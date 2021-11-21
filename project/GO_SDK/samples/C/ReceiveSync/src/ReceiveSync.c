/*
 * ReceiveSync.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive data.
 * Ethernet output for the desired data must be enabled.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define NUM_PROFILES            10
#define RECEIVE_TIMEOUT         200000 
#define SENSOR_IP               "192.168.1.10"  

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    kStatus status;
    unsigned int i, j, n;           
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

    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }
    
    // loop until 10 results are received
    n = 0;
    while (n < NUM_PROFILES)
    {
        if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
        {
            printf("Callback:\n");
            printf("Data message received:\n"); 
            printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));

            // each result can have multiple data items
            // loop through all items in result message
            for (i = 0; i < GoDataSet_Count(dataset); ++i)
            {
                GoDataMsg dataObj = GoDataSet_At(dataset, i);
                // retrieve GoStamp message
                switch (GoDataMsg_Type(dataObj))
                {
                case GO_DATA_MESSAGE_TYPE_STAMP:                
                    {
                        GoStampMsg stampMsg = dataObj;
                        printf("  Stamp Message batch count: %u\n", (k32u)GoStampMsg_Count(stampMsg));

                        for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                        {
                            GoStamp *stamp = GoStampMsg_At(stampMsg, j);
                            printf("  Timestamp: %llu\n", stamp->timestamp);
                            printf("  Encoder: %lld\n", stamp->encoder); 
                            printf("  Frame index: %llu\n", stamp->frameIndex);                     
                        }
                    }
                    break;
                // Refer to example ReceiveRange, ReceiveProfile, ReceiveMeasurement and ReceiveWholePart on how to receive data                
                }               
            }
            GoDestroy(dataset);
            n++;
        }
        
    }
    
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
