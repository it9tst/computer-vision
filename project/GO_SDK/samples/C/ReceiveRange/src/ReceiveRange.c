/*
 * ReceiveRange.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive range data in Range Mode and translate to engineering units (mm).
 * Ethernet output for range data must be enabled.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define RECEIVE_TIMEOUT         (20000000) 
#define INVALID_RANGE_16BIT     ((k16s)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define SENSOR_IP               "192.168.1.10"                      

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

typedef struct ProfilePoint 
{
    double x;   // x-coordinate in engineering units (mm) - position along laser line
    double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
    unsigned char intensity;
} ProfilePoint;

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    ProfilePoint* profileBuffer = NULL; 
    GoStamp *stamp =kNULL;
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

    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }

    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {           
        printf("Data message received:\n"); 
        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
        // each result can have multiple data items
        // loop through all items in result message
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {           
            dataObj = GoDataSet_At(dataset, i);
            //Retrieve GoStamp message
            switch(GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
                {
                    GoStampMsg stampMsg = dataObj;

                    printf("Stamp Message batch count: %u\n", (k32u)GoStampMsg_Count(stampMsg));
                    for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                    {
                        stamp = GoStampMsg_At(stampMsg, j);
                        printf("  Timestamp: %llu\n", stamp->timestamp);
                        printf("  Encoder: %lld\n", stamp->encoder); 
                        printf("  Frame index: %llu\n", stamp->frameIndex);                     
                    }
                }
                break;
            case GO_DATA_MESSAGE_TYPE_RANGE:            
                {                   
                    GoRangeMsg rangeMsg = dataObj;

                    printf("Range Message batch count: %u\n", (k32u)GoRangeMsg_Count(rangeMsg));

                    for (k = 0; k < GoRangeMsg_Count(rangeMsg); ++k)
                    {                       
                        k16s* data = GoRangeMsg_At(rangeMsg, k);                       
                        double ZResolution = NM_TO_MM(GoRangeMsg_ZResolution(rangeMsg));
                        double ZOffset = UM_TO_MM(GoRangeMsg_ZOffset(rangeMsg));
                        double pointZ; 
                        
                        if (*data != INVALID_RANGE_16BIT )
                        {
                            pointZ =  ZOffset + ZResolution * *data;
                        }
                        else
                        {
                            pointZ = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
                break;                      
            case GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY:          
                {
                    //kSize validPointCount = 0;
                    GoRangeIntensityMsg intensityMsg = dataObj;
                    printf("Intensity Message batch count: %u\n", (k32u)GoRangeIntensityMsg_Count(intensityMsg));

                    for (k = 0; k < GoRangeIntensityMsg_Count(intensityMsg); ++k)
                    {
                        //k8u* data = GoRangeIntensityMsg_At(intensityMsg, k);                      
                        //k8u intensity = *data;
                    }
                }
                break;
            }
        }
        GoDestroy(dataset);
    }
    else
    {
        printf ("Error: No data received during the waiting period\n");
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
    free(profileBuffer);

    printf("Press any key to continue...\n");
    getchar();
    return;
}