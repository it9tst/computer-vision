/*
 * ReceiveHealth.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive data
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define NUM_MESSAGES    10
#define RECEIVE_TIMEOUT 2000000 
#define SENSOR_IP           "192.168.1.10"

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet healthData = kNULL;
    GoHealthMsg healthMsg =kNULL;
    GoIndicator *healthIndicator = kNULL;
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

    //loop until 10 results are received
    i = 0;
    while (i < NUM_MESSAGES)
    {
        if ((status = GoSystem_ReceiveHealth(system, &healthData, RECEIVE_TIMEOUT)) == kOK)
        {
            for (j = 0; j < GoDataSet_Count(healthData); ++j)
            {
                healthMsg = GoDataSet_At(healthData, j);
                printf("Health message received:\n"); 
                printf("  Number of indicators: %u\n", (k32u)GoHealthMsg_Count(healthMsg)); 
                for (k = 0; k < GoHealthMsg_Count(healthMsg); k++)
                {
                    healthIndicator = GoHealthMsg_At(healthMsg, k);
                    printf("  Indicator[%u]: Id:%u Instance:%u Value:%lld\n", k, healthIndicator->id, healthIndicator->instance, healthIndicator->value); 
                }
            }
            GoDestroy(healthMsg);
            i++;
        }
        else if (status == kERROR_TIMEOUT)
        {
            printf("Timeout in GoSystem_ReceiveHealth...\n");
        }
        else
        {
            printf("Error: GoSystem_ReceiveHealth:%d\n", status);
            return;
        }
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);


    printf("Press any key to continue...\n");
    getchar();

    return;
}