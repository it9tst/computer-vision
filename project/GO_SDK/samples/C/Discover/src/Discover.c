/*
 * Discover.c
 * 
 * Gocator Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and modify parameters
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>

void main(int argc, char **argv)
{
    kStatus status;
    kAssembly api = kNULL;
    kAssembly apiLib = kNULL;
    GoSystem system = kNULL;
    k32u i;

    if ((status = kApiLib_Construct(&apiLib)) != kOK)
    {
        printf("Error: kApiLib_Construct:%d\n", status);
        return;
    }

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return;
    }   

    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return;
    }

    for (i=0; i < GoSystem_SensorCount(system); i++)
    {
        GoAddressInfo addressInfo;
        GoSensor sensor = GoSystem_SensorAt(system, i);
                    
        if ((status = GoSensor_Address(sensor, &addressInfo)) == kOK)
        {
            kChar addressString[64];    
            kIpAddress_Format(addressInfo.address, addressString, 64);
            printf ("Discovers sensor %d with IP %s\n", GoSensor_Id(sensor), addressString);
        }
    }

    GoDestroy(system);

    // destroy handles
    GoDestroy(api);
    
    printf("Press any key to continue...\n");
    getchar();

    return;
}
