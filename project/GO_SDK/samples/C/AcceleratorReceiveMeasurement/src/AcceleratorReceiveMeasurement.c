/*
 * AcceleratorReceiveMeasurement.c
 *
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 *
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Demonstrates the simple use of the Accelerator by connecting to a sensor and receiving a measurement.
 * This allows processing to be performed on the PC rather than on the sensor.
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
#define WEB_PORT                8081                            // ensure the port you specify is not already claimed and available, default port is 8080

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoAccelerator accelerator = kNULL;
    GoDataSet dataset = kNULL;
    GoStamp *stamp = kNULL;
    //GoProfilePositionX positionX = kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress;
    GoMeasurementData *measurementData = kNULL;

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

    // construct GoAccelerator object
    if ((status = GoAccelerator_Construct(&accelerator, kNULL)) != kOK)
    {
        printf("Error: GoAccelerator_Construct:%d\n", status);
        return;
    }

    // use this call to specify different Web port if 8080 is not available
    // otherwise you can skip next line....
    status = GoAccelerator_SetWebPort(accelerator, WEB_PORT);

    // start Accelerator service
    if ((status = GoAccelerator_Start(accelerator)) != kOK)
    {
        printf("Error: GoAccelerator_Start:%d\n", status);
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

    // attach accelerator
    if ((status = GoAccelerator_Attach(accelerator, sensor)) != kOK)
    {
        printf("Error: GoAccelerator_Attach:%d\n", status);
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
            case GO_DATA_MESSAGE_TYPE_MEASUREMENT:
                {
                    GoMeasurementMsg measurementMsg = dataObj;

                    printf("Measurement Message batch count: %u\n", (k32u)GoMeasurementMsg_Count(measurementMsg));

                    for (k = 0; k < GoMeasurementMsg_Count(measurementMsg); ++k)
                    {
                        measurementData = GoMeasurementMsg_At(measurementMsg, k);
                        printf("Measurement ID: %u\n", GoMeasurementMsg_Id(measurementMsg));
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

    // detach accelerator
    if ((status = GoAccelerator_Detach(accelerator, sensor)) != kOK)
    {
        printf("Error: GoAccelerator_Detach:%d\n", status);
        return;
    }

    // stop Accelerator service
    if ((status = GoAccelerator_Stop(accelerator)) != kOK)
    {
        printf("Error: GoAccelerator_Start:%d\n", status);
        return;
    }

    // destroy handles
    GoDestroy(accelerator);
    GoDestroy(system);
    GoDestroy(api);

    printf("Press any key to terminate...\n");
    getchar();
    return;
}
