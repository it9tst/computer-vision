/*
 * Multi Sensor Layout
 *
 * Gocator Sample
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 *
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Build N-Sensor system and receive data using a callback function.
 * Ethernet output for the desired data must be enabled.
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <memory.h>

// Replace with your sensors serial numbers.
#define Sensor_ID_Main              37064
#define Sensor_ID_Bud1              46796
#define Sensor_ID_Bud2              39902
#define MAIN                        1

// This example has 3 sensors added to a WIDE layout grid.
k32u sensors[2] = { Sensor_ID_Bud1, Sensor_ID_Bud2 };


typedef struct MyBuddy
{
    k32u ID;
    GoSensor sensor;
} MyBuddy;


typedef struct MyGocator
{
    k32u ID;
    GoSensor sensor;
    MyBuddy buddies[2];
    k32u buddiesCount;
    GoSetup setup;
    GoLayout layout;
} MyGocator;


int main(int argc, char **argv)
{
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    MyGocator gocator;
    kStatus status = kOK;

    // Construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return;
    }

    // Construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return;
    }

    // Find sensor
    if ((status = GoSystem_FindSensorById(system, Sensor_ID_Main, &gocator.sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return;
    }

    // Create connection to sensor
    if ((status = GoSensor_Connect(gocator.sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return;
    }

    // Enable sensor data channel
    if ((status = GoSensor_EnableData(gocator.sensor, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return;
    }

    if ((status = GoSensor_Stop(gocator.sensor)) != kOK)
    {
        printf("Error: GoSensor_Stop:%d\n", status);
        return;
    }

    // Acquire setup and layout handles
    gocator.setup = GoSensor_Setup(gocator.sensor);
    gocator.layout = GoSetup_Layout(gocator.setup);

    // Remove buddy sensor(s)
    if (GoSensor_HasBuddies(gocator.sensor))
    {
        k32u buddiesCount = (k32u)GoSensor_BuddiesCount(gocator.sensor);
        for (k32u i = 0; i < buddiesCount; i++)
        {
            // Remove one at a time.
            if ((status = GoSensor_RemoveBuddy(gocator.sensor)) != kOK)
            {
                printf("Error: GoSensor_RemoveBuddy:%d\n", status);
                return;
            }
            printf("\nBuddy sensor(s) removed.");
        }
    }

    // Number of sensors in the array
    k32u sensorCount = sizeof(sensors) / sizeof(k32u);

    // Set orientation
    if ((status = GoLayout_SetOrientation(gocator.layout, GO_ORIENTATION_WIDE)) != kOK)
    {
        printf("Error: GoLayout_SetOrientation:%d\n", status);
        return;
    }

    // Set number of columns
    if ((status = GoLayout_SetGridColumnCount(gocator.layout, MAIN + sensorCount)) != kOK)
    {
        printf("Error: GoLayout_SetGridColumnCount:%d\n", status);
        return;
    }

    // Add each buddy sensor to the main sensor
    for (k32u i = 0; i < sensorCount; i++)
    {
        // Obtain GoSensor object by sensor ID for buddy sensor
        if ((status = GoSystem_FindSensorById(system, sensors[i], &gocator.buddies[i].sensor)) != kOK)
        {
            printf("Error: GoSystem_FindSensorById:%d\n", status);
            return;
        }

        gocator.buddies[i].ID = sensors[i];

        // Create connection to GoSensor object for buddy sensor
        if ((status = GoSensor_Connect(gocator.buddies[i].sensor)) != kOK)
        {
            printf("Error: GoSensor_Connect:%d\n", status);
            return;
        }

        // Assign buddy sensor connection
        if ((status = GoSensor_AddBuddyBlocking(gocator.sensor, gocator.buddies[i].sensor)) != kOK)
        {
            printf("Error: GoSensor_AddBuddyBlocking:%d\n", status);
            return;
        }

        printf("\nBuddy sensor assigned..%u", gocator.buddies[i].ID);
    }
    gocator.buddiesCount = (k32u)GoSensor_BuddiesCount(gocator.sensor);

    if ((status = GoLayout_EnableMultiplexBuddy(gocator.layout, kTRUE)) != kOK)
    {
        printf("Error: GoLayout_EnableMultiplexBuddy:%d\n", status);
        return;
    }

    if ((status = GoSystem_Refresh(system)) != kOK)
    {
        printf("Error: GoSystem_Refresh:%d\n", status);
        return;
    }

    k32u timeBank = 1;

    // Assign sensors to the grid
    printf("\nSensor ID:\t%u", Sensor_ID_Main);
    if ((status = GoSetup_SetLayoutGridRow(gocator.setup, GO_ROLE_MAIN, 0)) != kOK)
    {
        printf("Error: GoSetup_SetLayoutGridRow:%d\n", status);
        return;
    }
    printf("\nRow\t%d", GoSetup_LayoutGridRow(gocator.setup, GO_ROLE_MAIN));

    if ((status = GoSetup_SetLayoutGridColumn(gocator.setup, GO_ROLE_MAIN, 0)) != kOK)
    {
        printf("Error: GoSetup_SetLayoutGridColumn:%d\n", status);
        return;
    }
    printf("\nCol\t%d", GoSetup_LayoutGridColumn(gocator.setup, GO_ROLE_MAIN));

    if ((status = GoSetup_SetLayoutGridDirection(gocator.setup, GO_ROLE_MAIN, 0)) != kOK)
    {
        printf("Error: GoSetup_SetLayoutGridDirection:%d\n", status);
        return;
    }
    printf("\nDir\t%d", GoSetup_LayoutGridDirection(gocator.setup, GO_ROLE_MAIN));

    if ((status = GoSetup_SetLayoutMultiplexingBank(gocator.setup, GO_ROLE_MAIN, timeBank)) != kOK)
    {
        printf("Error: GoSetup_SetLayoutMultiplexingBank:%d\n", status);
        return;
    }
    printf("\nBank\t%d\n", GoSetup_LayoutMultiplexingBank(gocator.setup, GO_ROLE_MAIN));

    for (k32u x = 0; x < gocator.buddiesCount; x++)
    {
        printf("\nG[%u]", gocator.buddies[x].ID);
        if ((status = GoSetup_SetLayoutGridRow(gocator.setup, GOROLE_BUDDYIDX(x), 0)) != kOK)
        {
            printf("Error: GoSetup_SetLayoutGridRow:%d\n", status);
            return;
        }
        printf("\nRow\t%d", GoSetup_LayoutGridRow(gocator.setup, GOROLE_BUDDYIDX(x)));

        if ((status = GoSetup_SetLayoutGridColumn(gocator.setup, GOROLE_BUDDYIDX(x), (MAIN + x))) != kOK)
        {
            printf("Error: GoSetup_SetLayoutGridColumn:%d\n", status);
            return;
        }
        printf("\nCol\t%d", GoSetup_LayoutGridColumn(gocator.setup, GOROLE_BUDDYIDX(x)));

        if ((status = GoSetup_SetLayoutGridDirection(gocator.setup, GOROLE_BUDDYIDX(x), 0)) != kOK)
        {
            printf("Error: GoSetup_SetLayoutGridDirection:%d\n", status);
            return;
        }
        printf("\nDir\t%d", GoSetup_LayoutGridDirection(gocator.setup, GOROLE_BUDDYIDX(x)));

        timeBank %= (MAIN + gocator.buddiesCount);
        timeBank += 1;
        if ((status = GoSetup_SetLayoutMultiplexingBank(gocator.setup, GOROLE_BUDDYIDX(x), timeBank++)) != kOK)
        {
            printf("Error: GoSetup_SetLayoutMultiplexingBank:%d\n", status);
            return;
        }
        printf("\nBank\t%d\n", GoSetup_LayoutMultiplexingBank(gocator.setup, GOROLE_BUDDYIDX(x)));
    }

    // Destroy handles
    GoDestroy(system);
    GoDestroy(api);

    printf("\nPress any key to continue...\n");
    getchar();
    return;
}
