/*
 * BackupRestore.c
 * 
 * Gocator 2000 Sample
 * Copyright (C) 2011-2021 by LMI Technologies Inc.
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * Purpose: Backup a Gocator system and restore it
 *
 */
#include <GoSdk/GoSdk.h>
#include <stdio.h>

#define SENSOR_IP           "192.168.1.10"                      

void main(int argc, char **argv)
{
    kStatus status;
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    char* backupFile = "SystemBackup.bin";
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
        printf("Error: GoSystem_FindSensorByIpAddress:%d\n", status);
        return;
    }
    
    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return;
    }

    printf("Creating backup file:%s\n", backupFile);
    printf("-------------------\n");
    if ((status = GoSensor_Backup(sensor, backupFile)) != kOK)
    {
        printf("Error: GoSystem_Backup:%d\n", status);
        return;
    }

    printf("Restoring from  backup file:%s\n", backupFile);
    printf("-------------------\n");
    if ((status = GoSensor_Restore(sensor, backupFile)) != kOK)
    {
        printf("Error: GoSystem_Restore:%d\n", status);
        return;
    }

    // destroy handles
    GoDestroy(system);
    GoDestroy(api);

    
    printf("Press any key to continue...\n");
    getchar();

    return;
}