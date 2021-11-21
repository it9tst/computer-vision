/*
 * ReceiveMeasurement.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system, setup and receive measurement data under
 *          Profile Mode. Measurements should have been setup and have the 
 *          outputs enabled via Ethernet.
 * 
 * Please refer to SetupMeasurement.c for configuring the measurements.
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define RECEIVE_TIMEOUT         (20000000) 
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define SENSOR_IP               "192.168.1.10"                      

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    GoStamp *stamp = kNULL;
    //GoProfilePositionX positionX = kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress;
    GoMeasurementData *measurementData = kNULL;
    // Added
    GoMeasurement measurement = kNULL;
    GoTools collection_tools = kNULL;
    //GoTool base_tool = kNULL; 

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
        printf("Error: GoSystem_FindSensorByIpAddress:%d\n", status);
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
    
    // refer to SetupMeasurement for set up of the measurement tool

    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }
    // retrieve tools handle
    if ((collection_tools = GoSensor_Tools(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Tools: Invalid Handle\n");
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
            case GO_DATA_MESSAGE_TYPE_MEASUREMENT:
                {
                    GoMeasurementMsg measurementMsg = dataObj;

                    printf("Measurement Message batch count: %u\n", (k32u)GoMeasurementMsg_Count(measurementMsg));

                    for (k = 0; k < GoMeasurementMsg_Count(measurementMsg); ++k)
                    {
                        measurementData = GoMeasurementMsg_At(measurementMsg, k);

                        printf("Measurement ID: %u\n", GoMeasurementMsg_Id(measurementMsg));
                        // 1. Retrieve the measurement ID
                        int l = GoMeasurementMsg_Id(measurementMsg);
                        //2. Retrieve the measurement from the set of tools using measurement ID
                        measurement = GoTools_FindMeasurementById(collection_tools, l);
                        //3. Print the measurement name 
                        printf("Measurement Name: %s\n", GoMeasurement_Name(measurement));

                        printf("Measurement Value: %.1f\n", measurementData->value);
                        printf("Measurement Decision: %d\n", measurementData->decision);
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

    printf("Press any key to continue...\n");
    getchar();
    return;
}