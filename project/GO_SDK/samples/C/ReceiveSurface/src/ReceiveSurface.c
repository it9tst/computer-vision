/*
 * ReceiveSurface.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator system and receive Surface data and translate to engineering units. Gocator must be in Surface Mmode.
 * Ethernet output for the whole part and/or intensity data must be enabled.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#define RECEIVE_TIMEOUT 20000000
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.
#define SENSOR_IP               "192.168.1.10"  

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

typedef struct ProfilePoint 
{
    double x;   // x-coordinate in engineering units (mm) - position along laser line
    double y;   // y-coordinate in engineering units (mm) - position along the direction of travel
    double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
    unsigned char intensity;
} ProfilePoint;

void main(int argc, char **argv)
{
    kAssembly api = kNULL;
    kStatus status;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    unsigned int i, j;
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

    printf("Waiting for Whole Part data...\n\n");
    
    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {           
        //k16s* height_map_memory = NULL;
        //k8u* intensity_image_memory = NULL;
        ProfilePoint **surfaceBuffer = NULL;
        k32u surfaceBufferHeight = 0;
        
        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));

        // each result can have multiple data items
        // loop through all items in result message
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {           
            GoDataMsg dataObj = GoDataSet_At(dataset, i);

            switch(GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:            
                {
                    GoStampMsg stampMsg = dataObj;
                    printf("  Stamp Message batch count: %u\n\n", (k32u)GoStampMsg_Count(stampMsg));

                    for (j = 0; j < GoStampMsg_Count(stampMsg); j++)
                    {
                        GoStamp *stamp = GoStampMsg_At(stampMsg, j);
                        printf("  Timestamp: %llu\n", stamp->timestamp);
                        printf("  Encoder position at leading edge: %lld\n", stamp->encoder); 
                        printf("  Frame index: %llu\n", stamp->frameIndex);                 
                    }
                }
                break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
                {   
                    GoSurfaceMsg surfaceMsg = dataObj;      
                    unsigned int rowIdx, colIdx;

                    double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                    double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                    double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                    double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                    double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                    double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));                    

                    printf("  Surface data width: %lu\n", (k32u)GoSurfaceMsg_Width(surfaceMsg));
                    printf("  Surface data length: %lu\n", (k32u)GoSurfaceMsg_Length(surfaceMsg));

                    //allocate memory if needed
                    if (surfaceBuffer == NULL)
                    {   
                        surfaceBuffer = malloc(GoSurfaceMsg_Length(surfaceMsg) * sizeof(ProfilePoint *));

                        for (j=0; j < GoSurfaceMsg_Length(surfaceMsg); j++)
                        {
                            surfaceBuffer[j] = malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(ProfilePoint));
                        }

                        surfaceBufferHeight = (k32u)GoSurfaceMsg_Length(surfaceMsg);
                    }

                    for (rowIdx=0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++)
                    {
                        k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);
                        
                        for (colIdx=0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++)
                        {
                            // gocator transmits range data as 16-bit signed integers
                            // to translate 16-bit range data to engineering units, the calculation for each point is: 
                            //          X: XOffset + columnIndex * XResolution 
                            //          Y: YOffset + rowIndex * YResolution
                            //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

                            surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                            surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                                

                            if (data[colIdx] != INVALID_RANGE_16BIT)
                            {
                                surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];
                            }
                            else
                            {
                                surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                            }
                        }
                    }                   
                }       
                break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD:
                {
                    GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
                    unsigned int rowIdx, colIdx;

                    double XResolution = NM_TO_MM(GoSurfacePointCloudMsg_XResolution(surfacePointCloudMsg));
                    double YResolution = NM_TO_MM(GoSurfacePointCloudMsg_YResolution(surfacePointCloudMsg));
                    double ZResolution = NM_TO_MM(GoSurfacePointCloudMsg_ZResolution(surfacePointCloudMsg));
                    double XOffset = UM_TO_MM(GoSurfacePointCloudMsg_XOffset(surfacePointCloudMsg));
                    double YOffset = UM_TO_MM(GoSurfacePointCloudMsg_YOffset(surfacePointCloudMsg));
                    double ZOffset = UM_TO_MM(GoSurfacePointCloudMsg_ZOffset(surfacePointCloudMsg));

                    printf("  Surface Point Cloud data width: %lu\n", (k32u)GoSurfacePointCloudMsg_Width(surfacePointCloudMsg));
                    printf("  Surface Point Cloud data length: %lu\n", (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg));

                    //allocate memory if needed
                    if (surfaceBuffer == NULL)
                    {
                        surfaceBuffer = malloc(GoSurfacePointCloudMsg_Length(surfacePointCloudMsg) * sizeof(ProfilePoint *));

                        for (j = 0; j < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); j++)
                        {
                            surfaceBuffer[j] = malloc(GoSurfacePointCloudMsg_Width(surfacePointCloudMsg) * sizeof(ProfilePoint));
                        }

                        surfaceBufferHeight = (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg);
                    }

                    for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
                    {
                        kPoint3d16s *data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

                        for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
                        {
                            surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * data[colIdx].x;
                            surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * data[colIdx].y;

                            if (data[colIdx].z != INVALID_RANGE_16BIT)
                            {
                                surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx].z;
                            }
                            else
                            {
                                surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                            }
                        }
                    }
                }
                break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
                {   
                    GoSurfaceIntensityMsg surfaceIntMsg = dataObj;  
                    unsigned int rowIdx, colIdx;
                    double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
                    double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));                    
                    double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
                    double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));                    

                    printf("  Surface intensity width: %lu\n", (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg));
                    printf("  Surface intensity height: %lu\n", (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg));

                    //allocate memory if needed
                    if (surfaceBuffer == NULL)
                    {   
                        surfaceBuffer = malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) * sizeof(ProfilePoint *));

                        for (j=0; j < GoSurfaceIntensityMsg_Length(surfaceIntMsg); j++)
                        {
                            surfaceBuffer[j] = malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) * sizeof(ProfilePoint));
                        }

                        surfaceBufferHeight = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
                    }

                    for (rowIdx=0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++)
                    {
                        k8u *data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

                        // gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
                        for (colIdx=0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++)
                        {
                            surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                            surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                            surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
                        }
                    
                    }     
                }
                break;
            }
        }

        GoDestroy(dataset);
        //free memory arrays
        if (surfaceBuffer) 
        {
            unsigned int k;
            for (k=0; k < surfaceBufferHeight; k++)
            {
                free(surfaceBuffer[k]);
            }
        }
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
