/*
 * DualSensor.c
 *
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 *
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Connect to Gocator buddy system and receive range data in Profile Mode and translate to engineering units (mm). Gocator must be in Profile Mmode.
 * Ethernet output for the profile data must be enabled. Gocator buddy is removed at the end.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <windows.h>

#define RECEIVE_TIMEOUT         (20000000) 
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define SENSOR_IP               "192.168.1.10"
#define BUDDY_IP                "192.168.1.11"

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
    unsigned int i, j, k, arrayIndex;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoSensor buddy = kNULL;
    GoDataSet dataset = kNULL;
    ProfilePoint* profileBuffer = NULL;
    GoStamp *stamp = kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress, ipAddress_buddy;
    GoSetup setup = kNULL;
    //GoSetup buddy_setup = kNULL;
    k32u profilePointCount;

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

    // Parse IP address into address data structure for main sensor
    kIpAddress_Parse(&ipAddress, SENSOR_IP);

    // obtain GoSensor object by sensor IP address for main sensor
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return;
    }

    //NOTE: GoSystem_Connect() can be used on Gocator buddy systems already configured, the GoSystem() will be able to
    //recognize the buddy sensor as a paired buddy and not attempt to obtain a separate connection from the budfdy sensor. 
    //In this example, we are trying to pair two sensors as buddies, hence the need to use GoSensor_Connect() to retrieve sensor
    //handles for both Main and Buddy sensor.

    // create connection to GoSensor object for main sensor
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return;
    }

    // Check if buddy sensor has already being assigned, assign buddy if buddy has not being assigned
    if (!GoSensor_HasBuddy(sensor))
    {
        // Parse IP address into address data structure for buddy sensor
        kIpAddress_Parse(&ipAddress_buddy, BUDDY_IP);

        // obtain GoSensor object by sensor IP address for buddy sensor
        if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress_buddy, &buddy)) != kOK)
        {
            printf("Error: GoSystem_FindSensor:%d\n", status);
            return;
        }

        // create connection to GoSensor object for buddy sensor
        if ((status = GoSensor_Connect(buddy)) != kOK)
        {
            printf("Error: GoSensor_Connect:%d\n", status);
            return;
        }

        // Assign buddy sensor connection
        if ((status = GoSensor_AddBuddyBlocking(sensor, buddy)) != kOK)
        {
            printf("Error: GoSensor_AddBuddy:%d\n", status);
            return;
        }

        printf("Buddy sensor assigned.\n");
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return;
    }

    // retrieve setup handle for main sensor
    if ((setup = GoSensor_Setup(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Setup: Invalid Handle\n");
    }

    // reset main sensor Active Area X value to minimum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaX(setup, GO_ROLE_MAIN, GoSetup_ActiveAreaXLimitMin(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: Main Sensor - GoSetup_SetActiveAreaX:%d\n", status);
    }

    // reset main sensor Active Area Z value to minimum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaZ(setup, GO_ROLE_MAIN, GoSetup_ActiveAreaZLimitMin(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: Main Sensor - GoSetup_SetActiveAreaZ:%d\n", status);
    }

    // reset main sensor Active Area Height value to maximum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaHeight(setup, GO_ROLE_MAIN, GoSetup_ActiveAreaHeightLimitMax(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: Main Sensor - GoSetup_SetActiveAreaHeight:%d\n", status);
    }

    // reset main sensor Active Area Width value to maximum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaWidth(setup, GO_ROLE_MAIN, GoSetup_ActiveAreaWidthLimitMax(setup, GO_ROLE_MAIN))) != kOK)
    {
        printf("Error: Main Sensor - GoSetup_SetActiveAreaWidth:%d\n", status);
    }

    // reset buddy sensor Active Area X value to minimum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaX(setup, GO_ROLE_BUDDY, GoSetup_ActiveAreaXLimitMin(setup, GO_ROLE_BUDDY))) != kOK)
    {
        printf("Error: Buddy Sensor - GoSetup_SetActiveAreaX:%d\n", status);
    }

    // reset buddy sensor Active Area Z value to minimum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaZ(setup, GO_ROLE_BUDDY, GoSetup_ActiveAreaZLimitMin(setup, GO_ROLE_BUDDY))) != kOK)
    {
        printf("Error: Buddy Sensor - GoSetup_SetActiveAreaZ:%d\n", status);
    }

    // reset buddy sensor Active Area Height value to maximum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaHeight(setup, GO_ROLE_BUDDY, GoSetup_ActiveAreaHeightLimitMax(setup, GO_ROLE_BUDDY))) != kOK)
    {
        printf("Error: Buddy Sensor - GoSetup_SetActiveAreaHeight:%d\n", status);
    }

    // reset buddy sensor Active Area Width value to maximum in case of mixed model buddying
    if ((status = GoSetup_SetActiveAreaWidth(setup, GO_ROLE_BUDDY, GoSetup_ActiveAreaWidthLimitMax(setup, GO_ROLE_BUDDY))) != kOK)
    {
        printf("Error: Buddy Sensor - GoSetup_SetActiveAreaWidth:%d\n", status);
    }

    // GoSensorFlush() - immediately synchronizes configuration changes to the sensor
    // *The changes will be shown on the web browser GUI after the browser has been refreshed.
    // NOTE: Sensor is not automatically synchronized with every call to function that modifies a setting.
    // This allows for rapid configuring sensors without delay caused by synchronization after every call.
    // Generally functions that retreieve setting values causes automatic synchronization while functions that set values don't.
    // Synchronization is also always guranteed prior to sensor entering running state. The GoSensor_Flush() function
    // should only be used when configuration changes are needed to be seen immediately.
    GoSensor_Flush(sensor);

    // retrieve total number of profile points prior to starting the sensor
    if (GoSetup_UniformSpacingEnabled(setup))
    {
        // Uniform spacing is enabled. The number is based on the X Spacing setting
        profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN) + GoSetup_XSpacingCount(setup, GO_ROLE_BUDDY);
    }
    else
    {
        // non-uniform spacing is enabled. The max number is based on the number of columns used in the camera. 
        profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
    }

    if ((profileBuffer = malloc(profilePointCount * sizeof(ProfilePoint))) == kNULL)
    {
        printf("Error: Cannot allocate profileData, %d points\n", profilePointCount);
        return;
    }

    // start Gocator system
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
            switch (GoDataMsg_Type(dataObj))
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
            case GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE:
            {
                GoResampledProfileMsg profileMsg = dataObj;

                printf("Resampled Profile Message batch count: %u\n", (k32u)GoResampledProfileMsg_Count(profileMsg));

                for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                {
                    unsigned int validPointCount = 0;
                    short* data = GoResampledProfileMsg_At(profileMsg, k);
                    double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));

                    //translate 16-bit range data to engineering units and copy profiles to memory array
                    for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
                    {
                        if (data[arrayIndex] != INVALID_RANGE_16BIT)
                        {
                            profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                            profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
                            validPointCount++;
                        }
                        else
                        {
                            profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                            profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                    printf("  Profile Valid Point %d out of max %d\n", validPointCount, profilePointCount);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_PROFILE: // Note this is NON resampled profile            
            {
                GoProfileMsg profileMsg = dataObj;

                printf("Profile Message batch count: %u\n", (k32u)GoProfileMsg_Count(profileMsg));

                for (k = 0; k < GoProfileMsg_Count(profileMsg); ++k)
                {
                    kPoint16s* data = GoProfileMsg_At(profileMsg, k);
                    unsigned int validPointCount = 0;
                    double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
                    double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
                    double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
                    double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));

                    //translate 16-bit range data to engineering units and copy profiles to memory array
                    for (arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex)
                    {
                        if (data[arrayIndex].x != INVALID_RANGE_16BIT)
                        {
                            profileBuffer[arrayIndex].x = XOffset + XResolution * data[arrayIndex].x;
                            profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
                            validPointCount++;
                        }
                        else
                        {
                            profileBuffer[arrayIndex].x = INVALID_RANGE_DOUBLE;
                            profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                    printf("  Profile Valid Point %d out of max %d\n", validPointCount, profilePointCount);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY:
            {
                //kSize validPointCount = 0;
                GoProfileIntensityMsg intensityMsg = dataObj;
                printf("Intensity Message batch count: %u\n", (k32u)GoProfileIntensityMsg_Count(intensityMsg));

                for (k = 0; k < GoProfileIntensityMsg_Count(intensityMsg); ++k)
                {
                    unsigned char* data = GoProfileIntensityMsg_At(intensityMsg, k);
                    for (arrayIndex = 0; arrayIndex < GoProfileIntensityMsg_Width(intensityMsg); ++arrayIndex)
                    {
                        profileBuffer[arrayIndex].intensity = data[arrayIndex];
                    }
                }
            }
            break;
            }
        }
        GoDestroy(dataset);
    }
    else
    {
        printf("Error: No data received during the waiting period\n");
    }

    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSensor_Stop:%d\n", status);
        return;
    }

    // Remove buddy sensor assignment, first check if buddy is still assigned
    if (GoSensor_HasBuddy(sensor))
    {
        // Remove buddy sensor assignment
        if ((status = GoSensor_RemoveBuddy(sensor)) != kOK)
        {
            printf("Error: GoSensor_RemoveBuddy:%d\n", status);
            return;
        }
        printf("Buddy sensor removed.\n");
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);
    free(profileBuffer);

    printf("Press any key to continue...\n");
    getchar();
    return;
}
