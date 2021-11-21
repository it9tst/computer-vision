/**
 * @file    GoSensor.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoSystem.h>
#include <GoSdk/GoUtils.h>
#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Messages/GoHealth.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include <kApi/Utils/kUtils.h>
#include <kApi/Data/kImage.h>

kBeginClassEx(Go, GoSensor)
    kAddVMethod(GoSensor, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSensor_Construct(GoSensor* sensor, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSensor), sensor));
    if (!kSuccess(status = GoSensor_Init(*sensor, kTypeOf(GoSensor), system, discoveryInfo, alloc)))
    {
        kAlloc_FreeRef(alloc, sensor);
    }

    return status;
}

GoFx(kStatus) GoSensor_Init(GoSensor sensor, kType type, kPointer system, const GoDiscoveryInfo* discoveryInfo, kAlloc alloc)
{
    kObjR(GoSensor, sensor);
    kStatus status;


    kCheck(kObject_Init(sensor, type, alloc));
    obj->deviceId = 0;
    kZero(obj->protocolVersion);
    kZero(obj->discoveryHistory);
    obj->discoveryCount = 0;
    obj->healthCheckEnabled = kFALSE;
    kZero(obj->healthHistory);
    obj->healthCheckCount = 0;
    obj->healthMsgCount = 0;
    obj->previousHealthMsgCount = 0;
    obj->sensorInfoTime = k64U_NULL;
    kZero(obj->address);
    obj->role = GO_ROLE_INVALID;
    obj->buddyId = 0;
    obj->alignmentState = GO_ALIGNMENT_STATE_NOT_ALIGNED;
    obj->runState = GO_STATE_ONLINE;
    obj->user = GO_USER_NONE;
    obj->isResetting = kFALSE;
    obj->isCancelled = kFALSE;
    obj->isConnected = kFALSE;
    obj->isCompatible = kFALSE;
    obj->isUpgrading = kFALSE;
    kZero(obj->fileList);
    obj->fileListValid = kFALSE;
    kZero(obj->directoryList);
    obj->isDirectoryListValid = kFALSE;
    kZero(obj->partModelList);
    obj->isSyncPartModels = kFALSE;
    kZero(obj->configXml);
    kZero(obj->configXmlItem);
    obj->configValid = kFALSE;
    obj->configModified = kFALSE;
    obj->isSyncConfig = kFALSE;
    obj->isFlushConfig = kFALSE;
    kZero(obj->transformXml);
    kZero(obj->transformXmlItem);
    obj->transformValid = kFALSE;
    obj->transformModified = kFALSE;
    obj->isSyncTransform = kFALSE;
    kZero(obj->timer);
    obj->localSensorInfo = kNULL;
    kZero(obj->remoteSensorInfo);
    obj->infoValid = kFALSE;
    kZero(obj->buddySensorInfo);
    kZero(obj->resetTimer);
    obj->control = kNULL;
    obj->data = kNULL;
    obj->dataPort = GO_SENSOR_DATA_PORT;
    obj->onDataSet = kNULL;
    obj->onDataSetContext = kNULL;
    obj->health = kNULL;
    obj->healthPort = GO_SENSOR_HEALTH_PORT;
    obj->controlPort = GO_CONTROL_PORT_CONTROL;
    obj->upgradePort = GO_CONTROL_PORT_UPGRADE;
    obj->setup = kNULL;
    obj->tools = kNULL;
    obj->output = kNULL;
    obj->transform = kNULL;
    obj->replay = kNULL;
    kZero(obj->accelInfo);
    kZero(obj->discoveryInfo);

    obj->system = system;
    if (discoveryInfo != kNULL)
    {
        obj->deviceId = discoveryInfo->id;
        obj->address = discoveryInfo->address;
        obj->discoveryInfo.firmwareVersion = discoveryInfo->version;
    }

    kTry
    {
        kTest(GoSensorInfo_Construct(&obj->localSensorInfo, alloc));

        kTest(kArrayList_Construct(&obj->fileList, kTypeOf(kText64), 0, alloc));
        kTest(kArrayList_Construct(&obj->directoryList, kTypeOf(kText64), 0, alloc));
        kTest(kArrayList_Construct(&obj->partModelList, kTypeOf(GoPartModel), 0, alloc));
        kTest(kTimer_Construct(&obj->timer, alloc));
        kTest(kPeriodic_Construct(&obj->resetTimer, alloc));

        kTest(kArrayList_Construct(&obj->remoteSensorInfo, kTypeOf(GoSensorInfo), 0, alloc));
        kTest(kArrayList_Construct(&obj->buddySensorInfo, kTypeOf(GoBuddyInfo), 0, alloc));

        kTest(GoControl_Construct(&obj->control, alloc));
        if (discoveryInfo != kNULL)
        {
            kTest(GoControl_SetRemoteAddress(obj->control, discoveryInfo->address.address));
        }
        kTest(GoControl_SetCancelHandler(obj->control, GoSensor_OnCancelQuery, sensor));
    }
    kCatch(&status)
    {
        GoSensor_VRelease(sensor);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSensor_VRelease(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_Disconnect(sensor));
    kCheck(kDestroyRef(&obj->control));

    kCheck(kPeriodic_Stop(obj->resetTimer));
    kCheck(kDestroyRef(&obj->resetTimer));
    kCheck(kDisposeRef(&obj->remoteSensorInfo));
    kCheck(kDisposeRef(&obj->buddySensorInfo));
    kCheck(kDestroyRef(&obj->localSensorInfo));
    kCheck(kDisposeRef(&obj->partModelList));
    kCheck(kDisposeRef(&obj->directoryList));
    kCheck(kDisposeRef(&obj->fileList));
    kCheck(kDestroyRef(&obj->timer));

    kCheck(kDestroyRef(&obj->configXml));
    kCheck(kDestroyRef(&obj->transformXml));

    kCheck(kObject_VRelease(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_SetAddress(GoSensor sensor, const GoAddressInfo* info, kBool wait)
{
    kObj(GoSensor, sensor);
    GoDiscovery discovery = kNULL;
    kBool wasConnected = GoSensor_IsConnected(sensor);

    kTry
    {
        if (wasConnected)
        {
            kTest(GoSensor_Flush(sensor));
        }

        kTest(GoSensor_BeginReset(sensor));
        kTest(GoSensor_Disconnect(sensor));

        kTest(GoDiscovery_Construct(&discovery, kTRUE, kObject_Alloc(sensor)));
        kTest(GoDiscovery_SetAddress(discovery, obj->deviceId, info));

        if (wait)
        {
            if (wasConnected)
            {
                kTest(GoSensor_WaitForReconnect(sensor, GO_SYSTEM_RESET_TIMEOUT));
            }
            else
            {
                kTest(GoSensor_WaitForReboot(sensor, GO_SYSTEM_RESET_TIMEOUT));
            }
        }
    }
    kFinally
    {
        kDestroyRef(&discovery);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_Address(GoSensor sensor, GoAddressInfo* info)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        *info = obj->address;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_StartConnection(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kAlloc alloc = kObject_Alloc(sensor);
    kBool isCompatible = kFALSE;
    kStatus status = kERROR;

    kCheck(GoSensor_Disconnect(sensor));

    kCheckState(GoSensor_State(sensor) == GO_STATE_ONLINE);

    kTry
    {
        kTest(GoControl_Open(obj->control, obj->address.address, obj->controlPort, obj->upgradePort));

        isCompatible = GoControl_IsCompatible(obj->control);

        kTest(GoSensor_SetConnected(sensor, kTRUE, isCompatible));

        if (isCompatible)
        {
            kTest(GoSensor_EnableHealth(sensor, kTRUE));
        }

        kTest(GoSetup_Construct(&obj->setup, sensor, alloc));
        kTest(GoTools_Construct(&obj->tools, sensor, alloc));
        kTest(GoOutput_Construct(&obj->output, sensor, alloc));
        kTest(GoTransform_Construct(&obj->transform, sensor, alloc));
        kTest(GoReplay_Construct(&obj->replay, sensor, alloc));
    }
    kCatch(&status)
    {
        kCheck(GoSensor_SetConnected(sensor, kFALSE, isCompatible));
        kEndCatch(status);
    }
    return kOK;
}

GoFx(kStatus) GoSensor_CompleteConnection(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kBool isCompatible = kFALSE;
    kStatus status = kERROR;

    isCompatible = GoControl_IsCompatible(obj->control);

    kTry
    {
        if (isCompatible)
        {
            kTest(GoSensor_ReadInfo(sensor));
            obj->deviceId = GoSensorInfo_Id(obj->localSensorInfo);
            kTest(GoSensor_ReadConfig(sensor));
            kTest(GoSensor_ReadPartModels(sensor));
            kTest(GoSensor_ReadTransform(sensor));
            kTest(GoSensor_SetTimeDate(sensor));
        }
    }
    kCatch(&status)
    {
        kCheck(GoSensor_SetConnected(sensor, kFALSE, isCompatible));
        kEndCatch(status);
    }
    return kOK;
}

GoFx(kStatus) GoSensor_Connect(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kStatus status = kERROR;
    GoStates states;

    kTry
    {
        kTest(GoSensor_StartConnection(sensor));

        kTest(GoSensor_States(sensor, &states));

        if (GO_SECURITY_NONE != states.security && GO_USER_NONE == states.loginType)
        {
            kTest(GO_ERROR_AUTHENTICATION);
        }

        kTest(GoSensor_CompleteConnection(sensor));
    }
    kCatch(&status)
    {
        GoSensor_Disconnect(sensor);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ConnectAndLogin(GoSensor sensor, GoUser user, const kChar* password)
{
    kObj(GoSensor, sensor);
    kStatus status = kERROR;

    kTry
    {
        kTest(GoSensor_StartConnection(sensor));

        kTest(GoControl_Login(obj->control, user, password));

        kTest(GoSensor_CompleteConnection(sensor));

        kTest(GoSensor_InvalidateInfo(sensor));
    }
    kCatch(&status)
    {
        GoSensor_Disconnect(sensor);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSensor_Disconnect(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (GoSensor_IsConnected(sensor) && GoSensor_IsConfigurable(sensor))
    {
        GoSensor_Flush(sensor);
    }

    kTry
    {
        kTest(GoSensor_Cancel(sensor));
        kTest(GoControl_Close(obj->control));
    }
    kFinally
    {
        kCheck(kDisposeRef(&obj->replay));
        kCheck(kDisposeRef(&obj->transform));
        kCheck(kDisposeRef(&obj->output));
        kCheck(kDisposeRef(&obj->tools));
        kCheck(kDisposeRef(&obj->setup));
        kCheck(kDisposeRef(&obj->data));
        kCheck(kDisposeRef(&obj->health));

        kCheck(GoSensor_SetConnected(sensor, kFALSE, kFALSE));
        kCheck(GoSensor_Invalidate(sensor));

        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetConnected(GoSensor sensor, kBool isConnected, kBool isCompatible)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isConnected = isConnected;
        obj->isCompatible = isCompatible;

        if (!isConnected)
        {
            obj->isCancelled = kFALSE;
        }
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_Flush(GoSensor sensor)
{
    kCheck(GoSensor_FlushConfig(sensor));
    kCheck(GoSensor_FlushTransform(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_Invalidate(GoSensor sensor)
{
    kCheck(GoSensor_InvalidateRole(sensor));
    kCheck(GoSensor_InvalidateInfo(sensor));
    kCheck(GoSensor_InvalidateConfig(sensor));
    kCheck(GoSensor_InvalidateTransform(sensor));
    kCheck(GoSensor_InvalidateFileList(sensor));
    kCheck(GoSensor_InvalidateDirectoryList(sensor));
    kCheck(GoSensor_InvalidateBuddyId(sensor));
    kCheck(GoSensor_InvalidateBuddyInfoList(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_SyncConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->isSyncConfig && !obj->isFlushConfig)
    {
        kTry
        {
            obj->isSyncConfig = kTRUE;
            kTest(GoSensor_FlushConfig(sensor));
            kTest(GoSensor_CacheConfig(sensor));
        }
        kFinally
        {
            obj->isSyncConfig = kFALSE;
            kEndFinally();
        }
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SyncPartModels(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->isSyncPartModels)
    {
        kTry
        {
            obj->isSyncPartModels = kTRUE;
            kTest(GoSensor_FlushPartModels(sensor));
            kTest(GoSensor_CachePartModels(sensor));
        }
        kFinally
        {
            obj->isSyncPartModels = kFALSE;
            kEndFinally();
        }
    }

    return kOK;
}

GoFx(kStatus) GoSensor_FlushPartModels(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        kObjN(GoPartModel, modelObj, *kArrayList_AtT(obj->partModelList, i, GoPartModel));

        if (modelObj->isModified)
        {
            kCheck(GoSensor_WritePartModel(sensor, modelObj));
        }
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CachePartModels(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize i, j;
    kText64 tempText = "";
    kStatus exception = kOK;

    // Remove part models that are no longer present in the live job
    for (i = 0; i < kArrayList_Count(obj->partModelList); )
    {
        kObjN(GoPartModel, modelObj, *kArrayList_AtT(obj->partModelList, i, GoPartModel));
        kBool foundFile = kFALSE;
        GoPartModel partModel = kNULL;

        for (j = 0; j < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); j++)
        {
            kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, j, tempText, 64));
            strtok(tempText, ".");

            if (strcmp(tempText, modelObj->name) == 0)
            {
                foundFile = kTRUE;
            }
        }

        if (!foundFile)
        {
            kCheck(kArrayList_RemoveT(obj->partModelList, i, &partModel));
            kDestroyRef(&partModel);
        }
        else
        {
            i++;
        }
    }

    // Add or update part models in the live job
    for (i = 0; i < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); i++)
    {
        kBool foundModel = kFALSE;

        kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, i, tempText, 64));
        strtok(tempText, "."); //stripping out .mdl extension

        for (j = 0; j < kArrayList_Count(obj->partModelList); j++)
        {
            kObjN(GoPartModel, modelObj, *kArrayList_AtT(obj->partModelList, j, GoPartModel));

            if (strcmp(tempText, modelObj->name) == 0)
            {
                foundModel = kTRUE;

                if (!modelObj->isValid)
                {
                    kCheck(GoSensor_ReadPartModel(sensor, modelObj));
                    break;
                }
            }
        }

        if (!foundModel)
        {
            GoPartModel modelToAdd = kNULL;

            kTry
            {
                kTest(GoPartModel_Construct(&modelToAdd, sensor, tempText, kObject_Alloc(sensor)));
                kTest(GoSensor_ReadPartModel(sensor, modelToAdd));
                kTest(kArrayList_AddT(obj->partModelList, &modelToAdd));
            }
            kCatch(&exception)
            {
                kDestroyRef(&modelToAdd);
                kEndCatch(exception);
            }
        }
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidatePartModels(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        kObjNR(GoPartModel, modelObj, *kArrayList_AtT(obj->partModelList, i, GoPartModel));

        modelObj->isValid = kFALSE;
    }


    return kOK;
}

GoFx(kStatus) GoSensor_GetLivePartModel(GoSensor sensor, const kChar* name, kXml* xml, kAlloc allocator)
{
    kObj(GoSensor, sensor);
    kAlloc tempAlloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText256 configurationPath = "";

    kCheckState(GoSensor_IsReadable(sensor));

    kTry
    {
        kStrCopy(configurationPath, 256, "_live.job/");
        kStrCat(configurationPath, 256, name);
        kStrCat(configurationPath, 256, ".mdl/config.xml");
        kTest(GoControl_ReadFile(obj->control, configurationPath, &fileData, &fileSize, tempAlloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_GetLivePartModel-Debug.cfg", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLivePartModel(GoSensor sensor, const kChar* name, kXml xml)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText256 configurationPath = "";

    kCheckState(GoSensor_IsConfigurable(sensor));

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_SetLivePartModel-Debug.xml", fileData, fileSize));

        kStrCopy(configurationPath, 256, "_live.job/");
        kStrCat(configurationPath, 256, name);
        kStrCat(configurationPath, 256, ".mdl/config.xml");
        kTest(GoControl_WriteFile(obj->control, configurationPath, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free(alloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ReadPartModels(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
    {
        GoPartModel model = *kArrayList_AtT(obj->partModelList, i, GoPartModel);

        kCheck(GoSensor_ReadPartModel(sensor, model));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ReadPartModel(GoSensor sensor, GoPartModel model)
{
    kObj(GoSensor, sensor);
    kXml xml = kNULL;
    kXml root = kNULL;

    kCheck(GoSensor_GetLivePartModel(sensor, GoPartModel_Name(model), &xml, kObject_Alloc(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml)));

    kCheck(GoPartModel_Read(model, xml, root));

    return kOK;
}

GoFx(kStatus) GoSensor_WritePartModel(GoSensor sensor, GoPartModel model)
{
    kObj(GoSensor, sensor);
    kSize i;
    kText64 tempName = "";
    kXml xml = kNULL;
    kXmlItem root = kNULL;
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kText128 modelConfigString = "";

    kCheckState(GoSensor_IsConfigurable(sensor));

    for (i = 0; i < GoSensor_DirectoryFileCount(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE); i++)
    {
        kCheck(GoSensor_DirectoryFileNameAt(sensor, "mdl", GO_SENSOR_LIVE_JOB_NAME, kTRUE, i, tempName, 64));
        strtok(tempName, ".");

        if (strcmp(tempName, GoPartModel_Name(model)) == 0)
        {
            kTry
            {
                kObjN(GoPartModel, modelObj, model);

                kTest(kXml_Construct(&xml, kObject_Alloc(sensor)));
                kTest(kXml_AddItem(xml, kXml_Root(xml), "Configuration", &root));
                kTest(kXml_SetAttr32u(xml, root, "version", GO_PART_MODEL_CONFIG_VERSION));
                kTest(GoPartModel_Write(model, xml, root));
                kTest(kXml_SaveBytes(xml, &fileData, &fileSize, kObject_Alloc(sensor)));

                // Uncomment to save config to file (useful for debugging config problems)
                //kTest(kFile_Save("GoSensor_SetModel-Debug.xml", fileData, fileSize));

                kTest(kStrCat(modelConfigString, 256, "_live.job/"));
                kTest(kStrCat(modelConfigString, 256, GoPartModel_Name(model)));
                kTest(kStrCat(modelConfigString, 256, ".mdl/config.xml"));
                kTest(GoControl_WriteFile(obj->control, modelConfigString, fileData, fileSize));

                modelObj->isModified = kFALSE;
                modelObj->isValid = kFALSE;
            }
            kFinally
            {
                kDestroyRef(&xml);
                kAlloc_Free(kObject_Alloc(sensor), fileData);
                kEndFinally();
            }

            return kOK;
        }
    }

    //see if the model exists
    //if exists, set the live model

    return kERROR_NOT_FOUND;
}

GoFx(kSize) GoSensor_PartMatchModelCount(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!GoSensor_IsReadable(sensor))
    {
        return kSIZE_NULL;
    }

    if (!kSuccess(GoSensor_SyncPartModels(sensor)))
    {
        return 0;
    }

    return kArrayList_Count(obj->partModelList);
}

GoFx(GoPartModel) GoSensor_PartMatchModelAt(GoSensor sensor, kSize index)
{
    kObj(GoSensor, sensor);
    GoPartModel model = kNULL;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoSensor_SyncPartModels(sensor))
        && index < kArrayList_Count(obj->partModelList))
    {
        model = *kArrayList_AtT(obj->partModelList, index, GoPartModel);
    }

    return model;
}

GoFx(GoPartModel) GoSensor_PartMatchModel(GoSensor sensor, const kChar* name)
{
    kObj(GoSensor, sensor);
    GoPartModel model = kNULL;
    kSize i;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoSensor_SyncPartModels(sensor)))
    {
        for (i = 0; i < kArrayList_Count(obj->partModelList); i++)
        {
            GoPartModel partModel = *kArrayList_AtT(obj->partModelList, i, GoPartModel);

            if (strcmp(name, GoPartModel_Name(partModel)) == 0)
            {
                model = partModel;
            }
        }
    }

    return model;
}

GoFx(kStatus) GoSensor_FlushConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (obj->configModified && !obj->isFlushConfig)
    {
        kTry
        {
            obj->isFlushConfig = kTRUE;
            kTest(GoSensor_WriteConfig(sensor));
        }
        kFinally
        {
            obj->isFlushConfig = kFALSE;
            kEndFinally();
        }
        obj->configModified = kFALSE;
        obj->configValid = kFALSE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CacheConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->configValid)
    {
        kCheck(GoSensor_ReadConfig(sensor));
        obj->configModified = kFALSE;
        obj->configValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->configValid = kFALSE;

    return kOK;
}

GoFx(kBool) GoSensor_ConfigValid(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->configValid;
}

GoFx(kStatus) GoSensor_SetConfigModified(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->configModified = kTRUE;

    return kOK;
}

GoFx(kBool) GoSensor_ConfigModified(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->configModified;
}

GoFx(kStatus) GoSensor_ReadConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kXml xml = kNULL;
    kXml root = kNULL;
    k32u version;

    kCheck(GoSensor_GetLiveConfig(sensor, &xml, kObject_Alloc(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml)));

    kCheck(kDestroyRef(&obj->configXml));
    obj->configXml = xml;
    obj->configXmlItem = root;

    kCheck(kXml_Attr32u(xml, root, "version", &version));
    if (version != GO_SENSOR_CONFIG_SCHEMA_VERSION)
    {
        return kERROR_VERSION;
    }

    if (kXml_ChildExists(xml, root, "Setup"))
    {
        kCheck(GoSetup_ReadConfig(obj->setup, xml, kXml_Child(xml, root, "Setup")));
    }

    // Don't read the "Configuration/Streams" section. This section
    // is only used by the GUI client.

    if (kXml_ChildExists(xml, root, "Tools") && kXml_ChildExists(xml, root, "ToolOptions"))
    {
        kCheck(GoTools_Read(obj->tools, xml, kXml_Child(xml, root, "Tools"), kXml_Child(xml, root, "ToolOptions")));
    }

    if (kXml_ChildExists(xml, root, "Output"))
    {
        kCheck(GoOutput_Read(obj->output, xml, kXml_Child(xml, root, "Output")));
    }

    if (kXml_ChildExists(xml, root, "Replay"))
    {
        kCheck(GoReplay_Read(obj->replay, xml, kXml_Child(xml, root, "Replay")));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_WriteConfig(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kXml xml = kNULL;
    kXml root = kNULL;
    kXml item = kNULL;

    kCheckState(GoSensor_IsConfigurable(sensor));

    kTry
    {
        kTest(kXml_Construct(&xml, kObject_Alloc(sensor)));

        kTest(kXml_AddItem(xml, kNULL, "Configuration", &root));
        kTest(kXml_SetAttr32u(xml, root, "version", GO_SENSOR_CONFIG_SCHEMA_VERSION));

        kTest(kXml_AddItem(xml, root, "Setup", &item));
        kTest(GoSetup_WriteConfig(obj->setup, xml, item));

        // The "Configuration/Streams" section is sent only from sensor
        // to client, never from client to sensor. So don't write that
        // section.

        kTest(kXml_AddItem(xml, root, "Tools", &item));
        kTest(GoTools_Write(obj->tools, xml, item));

        kTest(kXml_AddItem(xml, root, "Output", &item));
        kTest(GoOutput_Write(obj->output, xml, item));

        kTest(kXml_AddItem(xml, root, "Replay", &item));
        kTest(GoReplay_Write(obj->replay, xml, item));

        kTest(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, root));

        kTest(GoSensor_SetLiveConfig(sensor, xml));
    }
    kFinally
    {
        kCheck(kDestroyRef(&xml));
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_GetLiveConfig(GoSensor sensor, kXml* xml, kAlloc allocator)
{
    kObj(GoSensor, sensor);
    kAlloc tempAlloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsReadable(sensor));

    kTry
    {
        kTest(GoControl_ReadFile(obj->control, GO_SENSOR_LIVE_CONFIG_NAME, &fileData, &fileSize, tempAlloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_GetLiveConfig-Debug.cfg", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLiveConfig(GoSensor sensor, kXml xml)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsConfigurable(sensor));

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save config to file (useful for debugging config problems)
        //kTest(kFile_Save("GoSensor_SetLiveConfig-Debug.cfg", fileData, fileSize));

        kTest(GoControl_WriteFile(obj->control, GO_SENSOR_LIVE_CONFIG_NAME, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free(alloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SyncTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->isSyncTransform)
    {
        kTry
        {
            obj->isSyncTransform = kTRUE;
            kTest(GoSensor_FlushTransform(sensor));
            kTest(GoSensor_CacheTransform(sensor));
        }
        kFinally
        {
            obj->isSyncTransform = kFALSE;
            kEndFinally();
        }
    }

    return kOK;
}

GoFx(kStatus) GoSensor_FlushTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (obj->transformModified)
    {
        kCheck(GoSensor_WriteTransform(sensor));
        obj->transformModified = kFALSE;
        obj->transformValid = kFALSE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CacheTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->transformValid)
    {
        kCheck(GoSensor_ReadTransform(sensor));
        obj->transformModified = kFALSE;
        obj->transformValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->transformValid = kFALSE;

    return kOK;
}

GoFx(kBool) GoSensor_TransformValid(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->transformValid;
}

GoFx(kStatus) GoSensor_SetTransformModified(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->transformModified = kTRUE;

    return kOK;
}

GoFx(kBool) GoSensor_TransformModified(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->transformModified;
}

GoFx(kStatus) GoSensor_ReadTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kXml xml = kNULL;
    kXml root = kNULL;
    k32u version;

    kCheck(GoSensor_GetLiveTransform(sensor, &xml, kObject_Alloc(sensor)));
    kCheck(!kIsNull(root = kXml_Root(xml)));

    kCheck(kXml_Attr32u(xml, root, "version", &version));
    if (version != GO_SENSOR_TRANSFORM_SCHEMA_VERSION)
    {
        return kERROR_VERSION;
    }

    kCheck(GoTransform_Read(obj->transform, xml, root));

    return kOK;
}

GoFx(kStatus) GoSensor_WriteTransform(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kXml xml = kNULL;
    kXml root = kNULL;

    kCheckState(GoSensor_IsConfigurable(sensor));

    kTry
    {
        kTest(kXml_Construct(&xml, kObject_Alloc(sensor)));

        kTest(kXml_AddItem(xml, kNULL, "Transform", &root));
        kTest(kXml_SetAttr32u(xml, root, "version", GO_SENSOR_TRANSFORM_SCHEMA_VERSION));

        kTest(GoTransform_Write(obj->transform, xml, root));

        kTest(GoSensor_SetLiveTransform(sensor, xml));
    }
    kFinally
    {
        kDestroyRef(&xml);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_GetLiveTransform(GoSensor sensor, kXml* xml, kAlloc allocator)
{
    kObj(GoSensor, sensor);
    kAlloc tempAlloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsReadable(sensor));

    kTry
    {
        kTest(GoControl_ReadFile(obj->control, GO_SENSOR_LIVE_TRANSFORM_NAME, &fileData, &fileSize, tempAlloc));

        // Uncomment to save transform to file (useful for debugging transform problems)
        //kTest(kFile_Save("GoSensor_GetLiveTransform-Debug.tfm", fileData, fileSize));

        kTest(kXml_LoadBytes(xml, fileData, fileSize, allocator));
    }
    kFinally
    {
        kAlloc_Free(tempAlloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetLiveTransform(GoSensor sensor, kXml xml)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;

    kCheckState(GoSensor_IsConfigurable(sensor));

    kTry
    {
        kTest(kXml_SaveBytes(xml, &fileData, &fileSize, alloc));

        // Uncomment to save transform to file (useful for debugging transform problems)
        //kTest(kFile_Save("GoSensor_SetLiveTransform-Debug.cfg", fileData, fileSize));

        kTest(GoControl_WriteFile(obj->control, GO_SENSOR_LIVE_TRANSFORM_NAME, fileData, fileSize));
    }
    kFinally
    {
        kAlloc_Free(alloc, fileData);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CacheFileList(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->fileListValid)
    {
        kCheck(GoSensor_ReadFileList(sensor));
        obj->fileListValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateFileList(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->fileListValid = kFALSE;

    return kOK;
}

GoFx(kBool) GoSensor_FileListValid(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->fileListValid;
}

GoFx(kStatus) GoSensor_ReadFileList(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReadable(sensor));

    kCheck(GoControl_ReadFileList(obj->control, obj->fileList, kNULL));

    return kOK;
}

GoFx(kStatus) GoSensor_ClearLog(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReadable(sensor));

    kCheck(GoControl_ClearLog(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_CacheInfo(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!obj->infoValid)
    {
        kCheck(GoSensor_ReadInfo(sensor));
        obj->infoValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateInfo(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->infoValid = kFALSE;

    kCheck(GoSensor_LockState(sensor));
    {
        obj->sensorInfoTime = k64U_NULL;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_InfoValid(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->infoValid;
}

GoFx(kStatus) GoSensor_ReadInfo(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoState state = GoSensor_KnownState(sensor);        //avoids communication
    k64u now = kTimer_Now();
    GoStates states;
    kVersion sensorInfoVersion = kVersion_Create(101, 9, 0, 0);
    kVersion currentSensorInfoVersion;

    kCheckState(GoState_IsResponsive(state));


    kCheck(GoControl_GetProtocolVersion(obj->control, &currentSensorInfoVersion));

    // Determine which method to use to get system info.
    if (kVersion_Compare(sensorInfoVersion, currentSensorInfoVersion) > 0)
    {
        kCheck(GoControl_GetSensorInfo(obj->control, obj->localSensorInfo, obj->remoteSensorInfo));
    }
    else
    {
        kCheck(GoControl_GetSensorInfoV2(obj->control, obj->localSensorInfo, obj->remoteSensorInfo, obj->buddySensorInfo));
    }

    if (kArrayList_Count(obj->buddySensorInfo) > 0 )
    {
        GoBuddyInfo* buddyInfo = kArrayList_AtT(obj->buddySensorInfo, 0, GoBuddyInfo);
        kObjN(GoSensorInfo, tempObj, obj->localSensorInfo);
        tempObj->buddyId = buddyInfo->id;
    }

    kCheck(GoControl_GetStates(obj->control, &states));

    kCheck(GoSensor_LockState(sensor));

    kTry
    {
        obj->user = states.loginType;
        obj->runState = GoSensorInfo_State(obj->localSensorInfo);
        obj->alignmentState = states.alignmentState;

        if (obj->role == GO_ROLE_INVALID)
        {
            obj->role = GoSensorInfo_Role(obj->localSensorInfo);
            obj->buddyId = GoSensorInfo_HasBuddy(obj->localSensorInfo) ? GoSensorInfo_BuddyId(obj->localSensorInfo) : k32U_NULL;
        }

        obj->sensorInfoTime = now;
    }
    kFinally
    {
        kCheck(GoSensor_UnlockState(sensor));
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateRole(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->role = GO_ROLE_INVALID;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateBuddyId(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kObjN(GoSensorInfo, tempObj, obj->localSensorInfo);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->buddyId = 0;
        tempObj->buddyId = 0;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateBuddyInfoList(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        kArrayList_Clear(obj->buddySensorInfo);
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_Refresh(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    if (state == GO_STATE_CANCELLED)
    {
        kCheck(GoSensor_Connect(sensor));
    }
    else if (state == GO_STATE_UNRESPONSIVE)
    {
        kCheck(GoSensor_Disconnect(sensor));
    }
    else
    {
        kCheck(GoSensor_Invalidate(sensor));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_RefreshSystem(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoSystem system = kNULL;

    if (kIsNull(obj->system)) { return kOK; } // no-op

    system = (GoSystem)obj->system;
    return GoSystem_Refresh(system);
}

GoFx(kStatus) GoSensor_CheckHealth(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (obj->healthCheckEnabled)
    {
        kSize historyMask = kCountOf(obj->healthHistory) - 1;
        k64u diff = obj->healthMsgCount - obj->previousHealthMsgCount;

        obj->healthHistory[obj->healthCheckCount & historyMask] = (diff > 0);
        obj->healthCheckCount++;

        obj->previousHealthMsgCount = obj->healthMsgCount;
    }

    return kOK;
}

GoFx(GoState) GoSensor_KnownState(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoState state;

    kCheck(GoSensor_LockState(sensor));
    {
        if (obj->isCancelled)
        {
            state = GO_STATE_CANCELLED;
        }
        else if (obj->isUpgrading)
        {
            state = GO_STATE_UPGRADING;
        }
        else if (obj->isResetting)
        {
            state = GO_STATE_RESETTING;
        }
        else if (!obj->isConnected)
        {
            state = GoSensor_IsDiscoveryOnline(sensor) ? GO_STATE_ONLINE : GO_STATE_OFFLINE;
        }
        else if (!GoSensor_IsDiscoveryOnline(sensor) && !GoSensor_IsHealthOnline(sensor))
        {
            state = GO_STATE_UNRESPONSIVE;
        }
        else if (!obj->isCompatible)
        {
            state = GO_STATE_INCOMPATIBLE;
        }
        else if ((obj->sensorInfoTime != k64U_NULL) && (obj->role == GO_ROLE_BUDDY))
        {
            state = GO_STATE_BUSY;
        }
        else if (obj->sensorInfoTime != k64U_NULL)
        {
            state = obj->runState;
        }
        else
        {
            state = GO_STATE_CONNECTED;
        }
    }
    kCheck(GoSensor_UnlockState(sensor));

    return state;
}

GoFx(kBool) GoSensor_IsDiscoveryOnline(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize discoveryMask = kCountOf(obj->discoveryHistory) - 1;
    kSize discoveryTotal = (kSize) obj->discoveryCount;
    kSize discoveryCount = (kSize) kMin_(obj->discoveryCount, GO_SENSOR_DISCOVERY_CHECK_COUNT);
    kBool discoveryOnline = (discoveryCount < GO_SENSOR_DISCOVERY_CHECK_COUNT);
    kSize i;

    for (i = 0; i < discoveryCount; ++i)
    {
        discoveryOnline = discoveryOnline || obj->discoveryHistory[(discoveryTotal-i-1) & discoveryMask];
    }

    return discoveryOnline;
}

GoFx(kBool) GoSensor_IsHealthOnline(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize healthMask = kCountOf(obj->healthHistory) - 1;
    kSize healthTotal = (kSize) obj->healthCheckCount;
    kSize healthCount = (kSize) kMin_(obj->healthCheckCount, GO_SENSOR_HEALTH_CHECK_COUNT);
    kBool healthOnline = (healthCount < GO_SENSOR_HEALTH_CHECK_COUNT);
    kSize i;

    for (i = 0; i < healthCount; ++i)
    {
        healthOnline = healthOnline || obj->healthHistory[(healthTotal-i-1) & healthMask];
    }

    return healthOnline;
}

GoFx(kBool) GoSensor_IsConnected(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor);

    return GoState_IsConnected(state);
}

GoFx(kBool) GoSensor_IsResponsive(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor);

    return GoState_IsResponsive(state);
}

GoFx(kBool) GoSensor_IsReadable(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    return GoState_IsReadable(state);
}

GoFx(kBool) GoSensor_IsConfigurable(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    return GoState_IsConfigurable(state);
}

GoFx(kBool) GoSensor_IsReady(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    return (state == GO_STATE_READY);
}

GoFx(kBool) GoSensor_IsRunning(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    return (state == GO_STATE_RUNNING);
}

GoFx(kBool) GoSensor_IsNormal(GoSensor sensor)
{
    GoState state = GoSensor_State(sensor);

    return GoState_IsNormal(state);
}

GoFx(kBool) GoSensor_IsCancelled(GoSensor sensor)
{
    GoState state = GoSensor_KnownState(sensor);

    return (state == GO_STATE_CANCELLED);
}

GoFx(kBool) GoSensor_ConsistencyIntervalElapsed(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    k64u now = kTimer_Now();

    return (obj->sensorInfoTime != k64U_NULL) && ((now - obj->sensorInfoTime) > GO_SENSOR_CONSISTENCY_INTERVAL);
}

GoFx(kStatus) GoSensor_Cancel(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isCancelled = kTRUE;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_OnCancelQuery(GoSensor sensor, kObject sender, kPointer args)
{
    GoState state = GoSensor_KnownState(sensor);        //use known state; avoids communication
    kBool isResponsive = GoState_IsResponsive(state);
    kBool isUpgrading = GoState_IsUpgrading(state);

    return (isResponsive || isUpgrading) ? kOK : kERROR_ABORT;
}

GoFx(kStatus) GoSensor_LockState(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    return GoSystem_LockState(obj->system);
}

GoFx(kStatus) GoSensor_UnlockState(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    return GoSystem_UnlockState(obj->system);
}

GoFx(kStatus) GoSensor_BeginReset(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isResetting = kTRUE;
    }
    kCheck(GoSensor_UnlockState(sensor));

    kCheck(kPeriodic_Start(obj->resetTimer, GO_SYSTEM_RESET_HOLD_INTERVAL, GoSensor_OnResetHoldComplete, sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_OnResetHoldComplete(GoSensor sensor, kPeriodic timer)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isResetting = kFALSE;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_WaitForReboot(GoSensor sensor, k64u timeout)
{
    kObj(GoSensor, sensor);

    kCheck(kTimer_Start(obj->timer, timeout));

    while ((GoSensor_State(sensor) != GO_STATE_ONLINE) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL));
    }

    return (GoSensor_State(sensor) == GO_STATE_ONLINE) ? kOK : kERROR_TIMEOUT;
}

GoFx(kStatus) GoSensor_WaitForReconnect(GoSensor sensor, k64u timeout)
{
    kObj(GoSensor, sensor);
    GoState state;

    kCheck(kTimer_Start(obj->timer, timeout));

    //wait for reconnection
    while (!kSuccess(GoSensor_Connect(sensor)) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL));
    }

    kCheck(kTimer_Start(obj->timer, kMin_(kTimer_Remaining(obj->timer), GO_SYSTEM_RESET_INCOMPLETE_TIMEOUT)));

    //wait for incomplete status to resolve (main booted, but buddy not yet detected)
    state = GoSensor_State(sensor);

    while (((state == GO_STATE_INCOMPLETE) || (state == GO_STATE_INCONSISTENT)) && !kTimer_IsExpired(obj->timer))
    {
        kCheck(kThread_Sleep(GO_SYSTEM_RESET_QUERY_INTERVAL));

        kCheck(GoSensor_Refresh(sensor));

        state = GoSensor_State(sensor);
    }

    return GoSensor_IsConnected(sensor) ? kOK : kERROR_TIMEOUT;
}

GoFx(kStatus) GoSensor_AddBuddy(GoSensor sensor, GoSensor buddy)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsConfigurable(sensor));
    kCheckState(GoSensor_IsConfigurable(buddy));
    kCheckState(!GoSensor_HasBuddy(buddy));

    kCheck(GoSensor_Flush(sensor));
    kCheck(GoSensor_Flush(buddy));

    kCheck(GoSensor_AppendBuddy(sensor, buddy));

    kCheck(GoSensor_Invalidate(sensor));
    kCheck(GoSensor_Invalidate(buddy));
    kCheck(GoSensor_SetConnected(buddy, kFALSE, kTRUE));

    // Sync the config to pick up the new <Device> config from sensor.
    // <Device> is writeen by sensor when a buddy is added and read by SDK.
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK;
}

// This works around command blocking on sensor during buddy connection.
GoFx(kStatus) GoSensor_WaitForCacheinfo(GoSensor sensor, k64u expiryTime)
{
    kCheck(GoSensor_InvalidateInfo(sensor));

    while (kTimer_Now() < expiryTime)
    {
        kStatus status = GoSensor_CacheInfo(sensor);

        if (status != kERROR_TIMEOUT)
        {
            return status;
        }
    }

    return kERROR_TIMEOUT;
}

GoFx(kStatus) GoSensor_WaitForBuddies(GoSensor sensor, k64u timeout)
{
    kObj(GoSensor, sensor);

    kSize i;
    kBool stillWaiting = kFALSE;
    const GoBuddyInfo* buddyInfo = kNULL;
    const k64u start = kTimer_Now();

    do
    {
        // if just checking with no timeout, need to pass small timeout into GoSensor_WaitForCacheinfo to succeed
        kCheck(GoSensor_WaitForCacheinfo(sensor, start + ((timeout ? timeout : 100)*1000)));

        if (!GoSensor_HasBuddies(sensor))
        {
            // no need to loop and check for buddies if system got no buddies
            return kOK;
        }

        for (i = 0; i < GoSensor_BuddiesCount(sensor); ++i)
        {
            buddyInfo = kArrayList_AtT(obj->buddySensorInfo, i, GoBuddyInfo);

            if (buddyInfo->state != GO_BUDDY_STATE_CONNECTED)
            {
                stillWaiting = kTRUE;
                break;
            }
        }
        if (!stillWaiting)
        {
            return kOK;
        }
        kCheck(kThread_Sleep(10000));

    } while (kTimer_Now() - start < (1000*timeout));

    return kERROR_TIMEOUT;
}

GoFx(kStatus) GoSensor_WaitForBuddyConnection(GoSensor sensor, k32u buddyId, k64u expiryTime)
{
    kObj(GoSensor, sensor);

    while (kTimer_Now() < expiryTime)
    {
        kSize i;
        const GoBuddyInfo* foundBuddyInfo = kNULL;

        kCheck(GoSensor_WaitForCacheinfo(sensor, expiryTime));

        for (i = 0; i < GoSensor_BuddiesCount(sensor); ++i)
        {
            const GoBuddyInfo* buddyInfo = kArrayList_AtT(obj->buddySensorInfo, i, GoBuddyInfo);

            if (buddyInfo->id == buddyId)
            {
                foundBuddyInfo = buddyInfo;
                break;
            }
        }

        kCheck(foundBuddyInfo != kNULL);

        if (foundBuddyInfo->state == GO_BUDDY_STATE_CONNECTED)
        {
            return kOK;
        }

        kCheck(kThread_Sleep(100000));
    }

    return kERROR_TIMEOUT;
}

GoFx(kStatus) GoSensor_AddBuddyBlocking(GoSensor sensor, GoSensor buddy)
{
    kObj(GoSensor, sensor);
    k32u buddyId = GoSensor_Id(buddy);
    static const k64u MAX_WAIT = 30 * 1000000;
    k64u expiryTime;

    kCheck(GoSensor_AddBuddy(sensor, buddy));

    expiryTime = kTimer_Now() + MAX_WAIT;

    // Special case: for older firmware without GetSystemInfov2,
    // there is no buddySensorInfo list. This also means the AddBuddy
    // command is synchronous so there's no need to check.
    // TODO: GoSensor_ReadInfo() should abstract away this difference.
    // Once that is done, we can remove this code.
    kCheck(GoSensor_WaitForCacheinfo(sensor, expiryTime));
    if (kArrayList_Count(obj->buddySensorInfo) == 0)
    {
        // Sync the config to pick up the new <Device> config from sensor.
        // <Device> is writeen by sensor when a buddy is added and read by SDK.
        kCheck(GoSensor_SyncConfig(sensor));
        return kOK;
    }

    kCheck(GoSensor_WaitForBuddyConnection(sensor, buddyId, expiryTime));

    // Sync the config to pick up the new <Device> config from sensor.
    // <Device> is writeen by sensor when a buddy is added and read by SDK.
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_RemoveBuddy(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoSensor buddy = GoSensor_Buddy(sensor);

    kCheckState(GoSensor_IsConfigurable(sensor));
    kCheckState(GoSensor_Role(sensor) == GO_ROLE_MAIN);

    if (GoSensor_HasBuddy(sensor))
    {
        kCheck(GoSensor_Flush(sensor));

        k32u buddyIndex = (k32u)(GoSensor_BuddiesCount(sensor) - 1);
        kCheck(GoControl_RemoveBuddies(obj->control, &buddyIndex, 1));

        kCheck(GoSensor_Invalidate(sensor));

        if (!kIsNull(buddy))
        {
            kCheck(GoSensor_Invalidate(buddy));
        }
    }

    return kOK;
}

GoFx(kBool) GoSensor_HasBuddy(GoSensor sensor)
{
    // Caching is done inside these functions so isn't needed here.
    if (GoSensor_Role(sensor) == GO_ROLE_MAIN)
    {
        return (GoSensor_HasBuddies(sensor));
    }
    else
    {
        return kFALSE;
    }
}

//gets last buddy
GoFx(GoSensor) GoSensor_Buddy(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoSensor buddy = kNULL;

    if (GoSensor_HasBuddy(sensor))
    {
        k32u buddyId = GoSensor_BuddyId(sensor);

        if (kSuccess(GoSystem_FindSensorById(obj->system, buddyId, &buddy)))
        {
            return buddy;
        }
    }

    return kNULL;
}

GoFx(k32u) GoSensor_BuddyId(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    k32u id = 0;

    kCheck(GoSensor_CacheInfo(sensor));

    kCheck(GoSensor_LockState(sensor));
    {
        if (kArrayList_Count(obj->buddySensorInfo) > 0)
        {
            GoBuddyInfo* buddyInfo = kArrayList_LastT(obj->buddySensorInfo, GoBuddyInfo);
            if (!kIsNull(buddyInfo))
            {
                id = buddyInfo->id;
            }
        }
    }
    kCheck(GoSensor_UnlockState(sensor));

    return id;
}

GoFx(kStatus) GoSensor_EndAlign(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndAlignment(obj->control));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginAlign(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kLogf("IsReady");

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginAlignment(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_Align(GoSensor sensor)
{
    kCheck(GoSensor_BeginAlign(sensor));
    kLogf("MidAlign");
    kCheck(GoSensor_EndAlign(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_ClearAlignment(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_ClearAlignment(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_SetAlignmentReference(GoSensor sensor, GoAlignmentRef reference)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_SetAlignmentReference(obj->control, reference));

    return kOK;
}

GoFx(kStatus) GoSensor_AlignmentReference(GoSensor sensor, GoAlignmentRef* reference)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReadable(sensor));
    kCheck(GoControl_GetAlignmentReference(obj->control, reference));

    return kOK;
}

GoFx(kStatus) GoSensor_EndExposureAutoSet(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndExposureAutoSet(obj->control));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginExposureAutoSet(GoSensor sensor, GoRole role)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_Refresh(sensor));
    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginExposureAutoSet(obj->control, role));

    return kOK;
}

GoFx(kStatus) GoSensor_ExposureAutoSet(GoSensor sensor, GoRole role)
{
    kCheck(GoSensor_BeginExposureAutoSet(sensor, role));
    kCheck(GoSensor_EndExposureAutoSet(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginStart(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginStart(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_EndStart(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndStart(obj->control));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_CanStart(GoSensor sensor)
{
    return (kIsError(GoSensor_WaitForBuddies(sensor, 0))) ? kFALSE : kTRUE;
}

GoFx(kStatus) GoSensor_Start(GoSensor sensor)
{
    kCheck(GoSensor_BeginStart(sensor));
    kCheck(GoSensor_EndStart(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginScheduledStart(GoSensor sensor, k64s value)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginScheduledStart(obj->control, value));

    return kOK;
}

GoFx(kStatus) GoSensor_EndScheduledStart(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndScheduledStart(obj->control));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_ScheduledStart(GoSensor sensor, k64s value)
{
    kCheck(GoSensor_BeginScheduledStart(sensor, value));
    kCheck(GoSensor_EndScheduledStart(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginStop(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_BeginStop(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_EndStop(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndStop(obj->control));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_Stop(GoSensor sensor)
{
    kCheck(GoSensor_BeginStop(sensor));
    kCheck(GoSensor_EndStop(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginSnapshot(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_BeginSnapshot(obj->control));

    return kOK;
}

GoFx(kStatus) GoSensor_EndSnapshot(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_EndSnapshot(obj->control));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_Snapshot(GoSensor sensor)
{
    kCheck(GoSensor_BeginSnapshot(sensor));
    kCheck(GoSensor_EndSnapshot(sensor));

    return kOK;
}

GoFx(GoState) GoSensor_State(GoSensor sensor)
{
    //return value intentionally not checked; success not required
    GoSensor_CacheInfo(sensor);

    return GoSensor_KnownState(sensor);
}

GoFx(kStatus) GoSensor_States(GoSensor sensor, GoStates* states)
{
    kObj(GoSensor, sensor);
    GoMode mode = GO_MODE_UNKNOWN;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetStates(obj->control, states)))
    {
        return kOK;
    }

    return kERROR;
}

GoFx(GoRole) GoSensor_Role(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_CacheInfo(sensor));

    return obj->role;
}

GoFx(GoMode) GoSensor_ScanMode(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoMode mode = GO_MODE_UNKNOWN;

    kCheck(GoSensor_Flush(sensor));

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetScanMode(obj->control, &mode)))
    {
        return mode;
    }

    return GO_MODE_UNKNOWN;
}

GoFx(kStatus) GoSensor_EmitDigital(GoSensor sensor, k16u index, k64s target, k8u value)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsRunning(sensor));

    kCheck(GoControl_ScheduleDigital(obj->control, index, target, value));

    return kOK;
}

GoFx(kStatus) GoSensor_EmitAnalog(GoSensor sensor, k16u index, k64s target, k32s value)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsRunning(sensor));

    kCheck(GoControl_ScheduleAnalog(obj->control, index, target, value));

    return kOK;
}

GoFx(kStatus) GoSensor_Reset(GoSensor sensor, kBool wait)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoSensor_Flush(sensor));
    kCheck(GoSensor_BeginReset(sensor));
    kCheck(GoControl_Reset(obj->control));
    kCheck(GoSensor_Disconnect(sensor));

    if (wait)
    {
        kCheck(GoSensor_WaitForReconnect(sensor, GO_SYSTEM_RESET_TIMEOUT));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ResetEncoder(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoSensor_Flush(sensor));
    kCheck(GoControl_ResetEncoder(obj->control));

    return kOK;
}

GoFx(k32u) GoSensor_Id(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->deviceId;
}

// DEPRECATED
GoFx(kStatus) GoSensor_Model(GoSensor sensor, kChar* model, kSize capacity)
{
    kObj(GoSensor, sensor);
    // modelName is deprecated (since it was ambiguous), provide the original behavior which was to return partNumber.
    return GoSensor_PartNumber(sensor, model, capacity);
}

GoFx(kStatus) GoSensor_PartNumber(GoSensor sensor, kChar* partNumber, kSize capacity)
{
    kObj(GoSensor, sensor);

    // If the value is still unknown, it means that the PartNumber was not in the discovery info, 
    // which means old sensor firmware that is older than ~2015, or really old Id.xml schemaVersion=1
    // that does not have PartNumber in the first place.
    // Fallback to the partNumber received via GetSystemInfo().
    if (kStrLength(obj->discoveryInfo.partNumber) == 0)
    {
        if (GoSensor_InfoValid(sensor))
        {
            kCheck(GoSensorInfo_PartNumber(obj->localSensorInfo, partNumber, capacity));
        }
        else
        {
            kStrCopy(partNumber, capacity, "");
        }
    }
    else
    { 
        // PartNumber learned via discovery.
        kCheck(kStrCopy(partNumber, capacity, obj->discoveryInfo.partNumber));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ModelNumber(GoSensor sensor, kChar* modelNumber, kSize capacity)
{
    kObj(GoSensor, sensor);
    // NOTE: ModelNumber is learned via discovery.
    return kStrCopy(modelNumber, capacity, obj->discoveryInfo.modelNumber);
}

GoFx(kStatus) GoSensor_ModelDisplayName(GoSensor sensor, kChar* modelDisplayName, kSize capacity)
{
    kObj(GoSensor, sensor);
    // NOTE: ModelDisplayName is learned via discovery.
    return kStrCopy(modelDisplayName, capacity, obj->discoveryInfo.modelDisplayName);
}

GoFx(GoUser) GoSensor_User(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!kSuccess(GoSensor_CacheInfo(sensor)))
    {
        return GO_USER_NONE;
    }

    return obj->user;
}

GoFx(kVersion) GoSensor_ProtocolVersion(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!GoSensor_IsConnected(sensor))
    {
        return 0;
    }

    return GoControl_ProtocolVersion(obj->control);
}

GoFx(kVersion) GoSensor_FirmwareVersion(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    // NOTE: Firmware version is learned via discovery.
    return obj->discoveryInfo.firmwareVersion;
}

GoFx(GoAlignmentState) GoSensor_AlignmentState(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!kSuccess(GoSensor_CacheInfo(sensor)))
    {
        return GO_ALIGNMENT_STATE_NOT_ALIGNED;
    }

    return obj->alignmentState;
}

GoFx(kStatus) GoSensor_Timestamp(GoSensor sensor, k64u* time)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(GoControl_GetTimestamp(obj->control, time));

    return kOK;
}

GoFx(kStatus) GoSensor_Encoder(GoSensor sensor, k64s* encoder)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(GoControl_GetEncoder(obj->control, encoder));

    return kOK;
}

GoFx(kStatus) GoSensor_Trigger(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsRunning(sensor));

    kCheck(GoControl_Trigger(obj->control));

    return kOK;
}

GoFx(kSize) GoSensor_FileCount(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    if (!kSuccess(GoSensor_CacheFileList(sensor)))
    {
        return 0;
    }

    return kArrayList_Count(obj->fileList);
}

GoFx(kStatus) GoSensor_FileNameAt(GoSensor sensor, kSize index, kChar* name, kSize capacity)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_CacheFileList(sensor));

    kCheckArgs(index < kArrayList_Count(obj->fileList));

    kCheck(kStrCopy(name, capacity, kArrayList_AtT(obj->fileList, index, kChar)));

    return kOK;
}

GoFx(kStatus) GoSensor_UploadFile(GoSensor sensor, const kChar* sourcePath, const kChar* destName)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* data = kNULL;
    kSize size = 0;

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(GoSensor_Flush(sensor));

    kTry
    {
        kTest(kFile_Load(sourcePath, &data, &size, alloc));

        kTest(GoControl_WriteFile(obj->control, destName, data, size));

        kTest(GoSensor_Invalidate(sensor));
    }
    kFinally
    {
        kAlloc_Free(alloc, data);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_DownloadFile(GoSensor sensor, const kChar* sourceName, const kChar* destPath)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));
    kCheck(GoControl_ReadFileStreamed(obj->control, sourceName, destPath, kFALSE));

    return kOK;
}

GoFx(kStatus) GoSensor_CopyFile(GoSensor sensor, const kChar* sourceName, const kChar* destName)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(GoSensor_Flush(sensor));
    
    // GOC-14330: if loading/switching a job file (by copying to GO_SENSOR_LIVE_JOB_NAME), first stop
    // a running sensor. After the job load, the sensor remains stopped, to match the behaviour
    // of switching jobs from the GUI.
    // If just making a copy of a file (ie. not loading/switching job file), no need to stop the sensor.
    if (kStrEquals(destName, GO_SENSOR_LIVE_JOB_NAME))
    {
        // Loading/switching job file. Check if sensor is running.
        if (GoSensor_IsRunning(sensor))
        {
            kCheck(GoSensor_Stop(sensor));
        }
    }

    kCheck(GoControl_CopyFile(obj->control, sourceName, destName));
    kCheck(GoSensor_Invalidate(sensor));
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_DeleteFile(GoSensor sensor, const kChar* name)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(GoSensor_Flush(sensor));
    kCheck(GoControl_DeleteFile(obj->control, name));
    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_FileExists(GoSensor sensor, const kChar* name)
{
    kSize fileCount = GoSensor_DirectoryFileCount(sensor, kNULL, "", kTRUE);
    kText128 fileName;
    kSize i;

    for (i = 0; i < fileCount; ++i)
    {
        kCheck(GoSensor_DirectoryFileNameAt(sensor, kNULL, "", kTRUE, i, fileName, kCountOf(fileName)));

        if (kStrEquals(fileName, name))
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

GoFx(k64u) GoSensor_UserStorageFree(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    k64u storageFree = 0;

    if (GoSensor_IsReadable(sensor))
    {
        GoControl_GetUserStorageFree(obj->control, &storageFree);
    }

    return storageFree;
}

GoFx(k64u) GoSensor_UserStorageUsed(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    k64u storageUsed = 0;

    if (GoSensor_IsReadable(sensor))
    {
        GoControl_GetUserStorageUsed(obj->control, &storageUsed);
    }

    return storageUsed;
}

GoFx(kStatus) GoSensor_SetDefaultJob(GoSensor sensor, const kChar* fileName)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_SetDefaultJob(obj->control, fileName));

    return kOK;
}

GoFx(kStatus) GoSensor_DefaultJob(GoSensor sensor, kChar* fileName, kSize capacity)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_GetDefaultJob(obj->control, fileName, capacity));

    return kOK;
}

GoFx(kStatus) GoSensor_LoadedJob(GoSensor sensor, kChar* fileName, kSize capacity, kBool* changed)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_GetLoadedJob(obj->control, fileName, capacity, changed));

    return kOK;
}

GoFx(kStatus) GoSensor_LogIn(GoSensor sensor, GoUser user, const kChar* password)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_Login(obj->control, user, password));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_ChangePassword(GoSensor sensor, GoUser user, const kChar* password)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_ChangePassword(obj->control, user, password));

    return kOK;
}

GoFx(kStatus) GoSensor_BeginUpgrade(GoSensor sensor, const kChar* sourcePath)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* data = kNULL;
    kSize size = 0;
    kStatus status;

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isUpgrading = kTRUE;
    }
    kCheck(GoSensor_UnlockState(sensor));

    kTry
    {
        kTest(kFile_Load(sourcePath, &data, &size, alloc));

        kTest(GoControl_BeginUpgrade(obj->control, data, size));
    }
    kCatchEx(&status)
    {
        GoSensor_EndUpgrade(sensor);
        kEndCatchEx(status);
    }
    kFinallyEx
    {
        kAlloc_Free(alloc, data);
        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_EndUpgrade(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        obj->isUpgrading = kFALSE;
        obj->isCancelled = kFALSE;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_PostUpgrade(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_BeginReset(sensor));
    kCheck(GoSensor_WaitForReconnect(sensor, GO_SYSTEM_RESET_TIMEOUT));

    return kOK;
}

GoFx(kStatus) GoSensor_Upgrade(GoSensor sensor, const kChar* sourcePath, GoUpgradeFx callback, kPointer receiver)
{
    kObj(GoSensor, sensor);
    k32s progress, responseStage;
    GoUpgradeFxArgs args;
    kStatus status;
    kBool upgradeComplete = kFALSE;
    kBool upgradeSucceeded = kFALSE;
    kBool upgradeReapply = kFALSE;

    //success optional
    GoSensor_Flush(sensor);

    //disable health; health service may not be reliable during upgrade
    kCheck(GoSensor_EnableHealth(sensor, kFALSE));

    kCheck(kTimer_Start(obj->timer, GO_SENSOR_UPGRADE_TIMEOUT));

    kCheck(GoSensor_BeginUpgrade(sensor, sourcePath));
    {
        do
        {
            kTry
            {
                kTest(kThread_Sleep(GO_SYSTEM_QUIT_QUERY_INTERVAL));

                kTest(GoControl_GetUpgradeStatusEx(obj->control, &responseStage, &progress));

                if (callback)
                {
                    args.progress = (k64f) progress;
                    kTest(callback(receiver, sensor, &args));
                }

                upgradeComplete = (responseStage < 1 || responseStage == 2);
                upgradeSucceeded = (responseStage == 0);
                upgradeReapply = (responseStage == 2);
            }
            kCatch(&status)
            {
                upgradeComplete = kTRUE;
                upgradeSucceeded = kTRUE; /* speculative */
                kEndCatch(kOK);
            }
        }
        while (!upgradeComplete && !obj->isCancelled && !kTimer_IsExpired(obj->timer));
    }
    kCheck(GoSensor_EndUpgrade(sensor));

    if (!upgradeComplete)
    {
        return kERROR_TIMEOUT;
    }

    if (upgradeReapply)
    {
        return kERROR_INCOMPLETE;
    }

    if (!upgradeSucceeded)
    {
        return kERROR;
    }

    return GoSensor_PostUpgrade(sensor);
}

GoFx(kStatus) GoSensor_Backup(GoSensor sensor, const kChar* destPath)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* data = kNULL;
    kSize size = 0;

    kCheckState(GoSensor_IsReady(sensor));

    kTry
    {
        kTest(GoControl_Backup(obj->control, &data, &size, alloc));
        kTest(kFile_Save(destPath, data, size));
    }
    kFinally
    {
        kAlloc_Free(alloc, data);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_Restore(GoSensor sensor, const kChar* sourcePath)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* data = kNULL;
    kSize size = 0;

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Invalidate(sensor));

    kTry
    {
        kTest(kFile_Load(sourcePath, &data, &size, alloc));
        kTest(GoControl_Restore(obj->control, data, size));
    }
    kFinally
    {
        kAlloc_Free(alloc, data);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_RestoreDefaults(GoSensor sensor, kBool restoreAddress)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsConnected(sensor));

    kCheck(GoSensor_Invalidate(sensor));

    kCheck(GoControl_RestoreFactory(obj->control, restoreAddress));

    kCheck(GoSensor_CacheInfo(sensor));
    kCheck(GoSensor_ReadConfig(sensor));
    kCheck(GoSensor_ReadTransform(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_EnableData(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));

    kCheck(kDestroyRef(&obj->data));

    if (enable)
    {
        kCheck(GoReceiver_Construct(&obj->data, kObject_Alloc(sensor)));
        kCheck(GoReceiver_SetBuffers(obj->data, GO_SENSOR_DATA_SOCKET_BUFFER, GO_SENSOR_DATA_STREAM_BUFFER));
        kCheck(GoReceiver_SetCancelHandler(obj->data, GoSensor_OnCancelQuery, sensor));
        kCheck(GoReceiver_SetMessageHandler(obj->data, GoSensor_OnData, sensor));

        kCheck(GoReceiver_Open(obj->data, obj->address.address, obj->dataPort));
    }

    return kOK;
}

GoFx(kBool) GoSensor_DataEnabled(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    return !kIsNull(obj->data);
}

GoFx(kStatus) GoSensor_OnData(GoSensor sensor, GoReceiver receiver, kSerializer reader)
{
    kObj(GoSensor, sensor);
    kAlloc msgAlloc = kAlloc_App();    //use the default system allocator for now; revisit later (performance)
    GoDataSet dataSet = kNULL;
    kStatus status = kNULL;

    kTry
    {
        kTest(kSerializer_ReadObject(reader, &dataSet, msgAlloc));
        kTest(GoDataSet_SetSenderId_(dataSet, obj->deviceId));
    }
    kCatch(&status)
    {
        kObject_Dispose(dataSet);
        kEndCatch(status);
    }

    if (obj->onDataSet == kNULL)
    {
        kCheck(GoSystem_OnData(obj->system, sensor, dataSet));
    }
    else
    {
        kCheck(obj->onDataSet(obj->onDataSetContext, sensor, dataSet));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_EnableHealth(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheck(kDestroyRef(&obj->health));

    if (enable)
    {
        kCheck(GoReceiver_Construct(&obj->health, kObject_Alloc(sensor)));
        kCheck(GoReceiver_SetBuffers(obj->health, GO_SENSOR_HEALTH_SOCKET_BUFFER, GO_SENSOR_HEALTH_STREAM_BUFFER));
        kCheck(GoReceiver_SetCancelHandler(obj->health, GoSensor_OnCancelQuery, sensor));
        kCheck(GoReceiver_SetMessageHandler(obj->health, GoSensor_OnHealth, sensor));

        kCheck(GoReceiver_Open(obj->health, obj->address.address, obj->healthPort));
    }

    kCheck(GoSensor_LockState(sensor));
    {
        obj->healthCheckEnabled = enable;
        obj->previousHealthMsgCount = 0;
        obj->healthCheckCount = 0;
        obj->healthMsgCount = 0;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_OnHealth(GoSensor sensor, GoReceiver receiver, kSerializer reader)
{
    kObj(GoSensor, sensor);
    kAlloc msgAlloc = kAlloc_App();
    GoDataSet healthSet = kNULL;
    kStatus status = kNULL;

    kTry
    {
        kTest(kSerializer_ReadObject(reader, &healthSet, msgAlloc));
        kTest(GoDataSet_SetSenderId_(healthSet, obj->deviceId));

        kTest(GoSensor_UpdateHealthInfo(sensor, healthSet));
    }
    kCatch(&status)
    {
        kObject_Dispose(healthSet);
        kEndCatch(status);
    }

    kCheck(GoSystem_OnHealth(obj->system, sensor, healthSet));

    return kOK;
}

GoFx(kStatus) GoSensor_UpdateHealthInfo(GoSensor sensor, GoDataSet healthSet)
{
    kObj(GoSensor, sensor);
    kSize i, j;

    kCheck(GoSensor_LockState(sensor));
    {
        kBool consistencyIntervalElapsed = GoSensor_ConsistencyIntervalElapsed(sensor);
        kSize deviceCount = GoDataSet_Count(healthSet);

        for (i = 0; i < deviceCount; ++i)
        {
            GoHealthMsg health = GoDataSet_At(healthSet, i);
            kSize indicatorCount = GoHealthMsg_Count(health);
            k32u source = GoDataSet_SenderId(healthSet);

            if (source == GoSensor_Id(sensor))
            {
                for (j = 0; j < indicatorCount; ++j)
                {
                    const GoIndicator* indicator = GoHealthMsg_At(health, j);

                    switch (indicator->id)
                    {
                    case GO_HEALTH_STATE:
                        {
                            if (consistencyIntervalElapsed)
                            {
                                switch (indicator->value)
                                {
                                case -1:    obj->runState = GO_STATE_INCOMPLETE;       break;
                                case 0:     obj->runState = GO_STATE_READY;            break;
                                case 1:     obj->runState = GO_STATE_RUNNING;          break;
                                default:    obj->runState = GO_STATE_CONNECTED;        break;
                                }
                            }
                            break;
                        }

                    default:
                        break;
                    }

                }
            }
        }

        obj->healthMsgCount++;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(GoSetup) GoSensor_Setup(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->setup;
}

GoFx(GoTools) GoSensor_Tools(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->tools;
}

GoFx(GoOutput) GoSensor_Output(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->output;
}

GoFx(kStatus) GoSensor_EnableRecording(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetRecordingEnabled(obj->control, enable));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_RecordingEnabled(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetRecordingEnabled(obj->control, &enabled)))
    {
        return enabled;
    }

    return enabled;
}

GoFx(kStatus) GoSensor_SetInputSource(GoSensor sensor, GoInputSource source)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetInputSource(obj->control, source));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(GoInputSource) GoSensor_InputSource(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoInputSource source = GO_INPUT_SOURCE_LIVE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetInputSource(obj->control, &source)))
    {
        return source;
    }

    return source;
}

GoFx(kStatus) GoSensor_Simulate(GoSensor sensor, kBool* isBufferValid)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_Simulate(obj->control, isBufferValid));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_PlaybackStep(GoSensor sensor, GoSeekDirection direction)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_PlaybackStep(obj->control, direction));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_PlaybackSeek(GoSensor sensor, kSize position)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_PlaybackSeek(obj->control, position));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_PlaybackPosition(GoSensor sensor, kSize* position, kSize* count)
{
    kObj(GoSensor, sensor);

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_PlaybackPosition(obj->control, position, count)))
    {
        return kOK;
    }

    return kERROR;
}

GoFx(kStatus) GoSensor_ClearReplayData(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_ClearReplayData(obj->control));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_ClearMeasurementStats(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_ClearMeasurementStats(obj->control));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_ExportBitmap(GoSensor sensor,
                                    GoReplayExportSourceType type,
                                    GoDataSource source,
                                    const kChar* dstFileName)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_ExportBitmap(obj->control,
                                 type,
                                 source,
                                 dstFileName));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}


GoFx(kStatus) GoSensor_ExportCsv(GoSensor sensor, const kChar* dstFileName)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_ExportCsv(obj->control, dstFileName));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(GoFamily) GoSensor_Family(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoFamily returnType = GO_FAMILY_UNKNOWN;

    if (kStrLength(obj->discoveryInfo.family) == 0)
    {
        // If the value is still unknown, it means that the Family was not in the discovery info
        // so try to fall back to other sources.
        returnType = GoSensor_FillFamily(sensor);
    }
    // The following code mimics what is filled in for "Family" in GsDevice_FillExtendedIdGaps().
    else if (kStrCompareLower(obj->discoveryInfo.family, "G1x00") == 0)
    {
        returnType = GO_FAMILY_1000;
    }
    else if (kStrCompareLower(obj->discoveryInfo.family, "G2x00") == 0)
    {
        returnType = GO_FAMILY_2000;
    }
    else if (kStrCompareLower(obj->discoveryInfo.family, "G3x00") == 0)
    {
        returnType = GO_FAMILY_3000;
    }
    else
    {
        // NOTE: GoSdk currently does not handle "G2XX".
        // Wait until customer requests support for it.
        // Original implementation simply returns GO_FAMILY_2000 for it.
        kLogf("%s: Warning: could not determine family of '%s'. Falling back to G2x00.", 
              __FUNCTION__, obj->discoveryInfo.family);
        returnType = GO_FAMILY_2000; 
    }

    return returnType;
}

GoFx(GoFamily) GoSensor_FillFamily(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    GoFamily returnType = GO_FAMILY_UNKNOWN;

    kText32 partNumber;
    const char* pos = kNULL;
    kString tempString = kNULL;
    kArrayList tokens = kNULL;

    kTry
    {
        // If the value is still unknown, it means that the Family was not in the discovery info, 
        // which means old sensor firmware older than ~2015, so attempt to fallback 
        // to parsing the partNumber received via GetSystemInfo().
        if (GoSensor_InfoValid(sensor))
        {
            kTest(GoSensorInfo_PartNumber(obj->localSensorInfo, partNumber, kCountOf(partNumber)));
            kTest(kString_Construct(&tempString, partNumber, kObject_Alloc(sensor)));

            kTest(kString_Split(tempString, "-", &tokens, kObject_Alloc(sensor)));

            if(kArrayList_Count(tokens) == 0)
            { 
                returnType = GO_FAMILY_2000;
            }
            else
            {
                switch(atoi(kString_Chars(kArrayList_AsT(tokens, 0, kString)) + 2) / 1000)
                {
                    case 1:     returnType = GO_FAMILY_1000; break;
                    case 2:     returnType = GO_FAMILY_2000; break;
                    case 3:     returnType = GO_FAMILY_3000; break;
                    default:    returnType = GO_FAMILY_2000; break;
                }
            }
        }
        else
        {
            // NOTE: If localSensorInfo is not available, we do not explicitly trigger another 
            // GoSensor_CacheInfo() since the normal operation of this function should be able 
            // to obtain family via discovery.
            returnType = GO_FAMILY_UNKNOWN;
        }
    }
    kFinally
    {
        kDestroyRef(&tempString);
        kDisposeRef(&tokens);
        kEndFinally();
    }

    return returnType;
}

GoFx(GoTransform) GoSensor_Transform(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->transform;
}

GoFx(GoReplay) GoSensor_Replay(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->replay;
}

GoFx(kStatus) GoSensor_EnableAutoStart(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetAutoStartEnabled(obj->control, enable));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_AutoStartEnabled(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetAutoStartEnabled(obj->control, &enabled)))
    {
        return enabled;
    }

    return enabled;
}

GoFx(kStatus) GoSensor_SetVoltage(GoSensor sensor, GoVoltageSetting voltage, k64f cableLength)
{
    kObj(GoSensor, sensor);
    k32u cableLength_mm = 0;

    if (voltage != GO_VOLTAGE_48 && voltage != GO_VOLTAGE_24) return kERROR_PARAMETER;
    if (voltage == GO_VOLTAGE_24 && (cableLength < 0.0 || cableLength > 25.0)) return kERROR_PARAMETER;

    cableLength_mm = (k32u)(cableLength * 1000);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetVoltage(obj->control, voltage, cableLength_mm));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_GetVoltage(GoSensor sensor, GoVoltageSetting *voltage, k64f *cableLength)
{
    kObj(GoSensor, sensor);

    if (GoSensor_IsReadable(sensor))
    {
        k32u cableLength_mm = 0;
        kCheck(GoControl_GetVoltage(obj->control, voltage, &cableLength_mm));
        if (cableLength) *cableLength = (k64f)cableLength_mm / 1000.0;
    }
    else
    {
        return kERROR_COMMAND;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_EnableQuickEdit(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetQuickEditEnabled(obj->control, enable));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_QuickEditEnabled(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetQuickEditEnabled(obj->control, &enabled)))
    {
        return enabled;
    }

    return enabled;
}

GoFx(kSize) GoSensor_RemoteInfoCount(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return kArrayList_Count(obj->remoteSensorInfo);
}

GoFx(GoSensorInfo) GoSensor_RemoteInfoAt(GoSensor sensor, kSize index)
{
    kObj(GoSensor, sensor);

    kAssert(index < kArrayList_Count(obj->remoteSensorInfo));

    return kArrayList_AsT(obj->remoteSensorInfo, index, GoSensorInfo);
}

GoFx(kStatus) GoSensor_PartMatchCreateModel(GoSensor sensor, const kChar* name)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_CreateModel(obj->control, name));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_PartMatchDetectModelEdges(GoSensor sensor, const kChar* name, k16u sensitivity)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_DetectModelEdges(obj->control, name, sensitivity));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_CacheDirectoryList(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive)
{
    kObj(GoSensor, sensor);

    if (!obj->isDirectoryListValid)
    {
        kCheck(GoSensor_ListDirectory(sensor, extensionFilter, path, isRecursive, obj->directoryList));
        obj->isDirectoryListValid = kTRUE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_ListDirectory(GoSensor sensor, const kChar* extension, const kChar* root, kBool isRecursive, kArrayList fileList)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReadable(sensor));

    kCheck(GoControl_ListDirectory(obj->control, extension, root, isRecursive, fileList));

    return kOK;
}

GoFx(kSize) GoSensor_DirectoryFileCount(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_CacheDirectoryList(sensor, extensionFilter, path, isRecursive));

    return kArrayList_Count(obj->directoryList);
}

GoFx(kStatus) GoSensor_DirectoryFileNameAt(GoSensor sensor, const kChar* extensionFilter, const kChar* path, kBool isRecursive, kSize index, kChar* fileName, kSize capacity)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_CacheDirectoryList(sensor, extensionFilter, path, isRecursive));

    kCheckArgs(index < kArrayList_Count(obj->directoryList));

    kCheck(kStrCopy(fileName, capacity, kArrayList_AtT(obj->directoryList, index, kChar)));

    return kOK;
}

GoFx(kStatus) GoSensor_InvalidateDirectoryList(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->isDirectoryListValid = kFALSE;

    return kOK;
}

GoFx(kBool) GoSensor_DirectoryListValid(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->isDirectoryListValid;
}

GoFx(kStatus) GoSensor_SetDataHandler(GoSensor sensor, GoSensorDataSetFx function, kPointer context)
{
    kObj(GoSensor, sensor);

    obj->onDataSet = function;
    obj->onDataSetContext = context;

    return kOK;
}

GoFx(kStatus) GoSensor_AddTool(GoSensor sensor, const kChar* type, const kChar* name)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_AddTool(obj->control, type, name));

    kCheck(GoSensor_Invalidate(sensor));
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_AddMeasurement(GoSensor sensor, kSize index, const kChar* type, const kChar* name)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_AddMeasurement(obj->control, index, type, name));

    kCheck(GoSensor_Invalidate(sensor));
    kCheck(GoSensor_SyncConfig(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_IsCompatible(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return GoControl_IsCompatible(obj->control);
}

GoFx(kStatus) GoSensor_EnableRamImage(GoSensor sensor, kBool enable)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsReady(sensor));

    kCheck(GoSensor_Flush(sensor));

    kCheck(GoControl_SetRamImageEnabled(obj->control, enable));

    kCheck(GoSensor_Invalidate(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_RamImageEnabled(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kBool enabled = kFALSE;

    if (GoSensor_IsReadable(sensor)
        && kSuccess(GoControl_GetRamImageEnabled(obj->control, &enabled)))
    {
        return enabled;
    }

    return enabled;
}

GoFx(kStatus) GoSensor_WriteRamImage(GoSensor sensor, GoRole role, kBool resizeActiveArea, kSize cameraIndex, kSize stateIndex, kSize frameIndex, const kChar* imagePath)
{
    kObj(GoSensor, sensor);
    kImage tempImage = kNULL;
    k64f xRes = 0.0, yRes = 0.0, interval = 0.05;

    kCheckState(GoSensor_IsConfigurable(sensor));
    kCheckState(kFile_Exists(imagePath));

    kTry
    {
        kTest(kImage_Import(&tempImage, imagePath, kObject_Alloc(sensor)));

        if (resizeActiveArea)
        {
            xRes = GoSetup_FrontCameraWidth(obj->setup, role) / GoSetup_ActiveAreaWidth(obj->setup, role);
            yRes = GoSetup_FrontCameraHeight(obj->setup, role) / GoSetup_ActiveAreaHeight(obj->setup, role);
            kTest(GoSetup_SetActiveAreaHeight(obj->setup, role, kImage_Height(tempImage) / yRes));
            kTest(GoSetup_SetActiveAreaWidth(obj->setup, role, kImage_Width(tempImage) / xRes));

            kTest(GoSensor_Flush(sensor));

            while (GoSetup_FrontCameraWidth(obj->setup, role) > kImage_Width(tempImage))
            {
                kTest(GoSetup_SetActiveAreaWidth(obj->setup, role, GoSetup_ActiveAreaWidth(obj->setup, role) - interval));
            }

            while (GoSetup_FrontCameraHeight(obj->setup, role) > kImage_Height(tempImage))
            {
                kTest(GoSetup_SetActiveAreaHeight(obj->setup, role, GoSetup_ActiveAreaHeight(obj->setup, role) - interval));
            }

            //kTest(GoSetup_SetActiveAreaHeight(obj->setup, role, GoSetup_ActiveAreaHeight(obj->setup, role) + interval)); //we want the smallest possible active area that results in a matching pixel height
            //kTest(GoSetup_SetActiveAreaHeight(obj->setup, role, interval));
            kTest(GoSetup_SetActiveAreaZ(obj->setup, role, GoSetup_ActiveAreaZLimitMin(obj->setup, role)));

            kTest(GoSensor_Start(sensor));
            kTest(GoSensor_Stop(sensor));
        }

        kTest(GoControl_WriteRamImage(obj->control, role, cameraIndex, stateIndex, frameIndex, tempImage));
    }
    kFinally
    {
        kDisposeRef(&tempImage);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetRuntimeVariables(GoSensor sensor, kSize startIndex, kSize length, k32s* values)
{
    kObj(GoSensor, sensor);

    //no state check or flush is done, as runtime variables can be set at any time
    kCheckArgs(!kIsNull(values));

    kCheck(GoControl_SetRuntimeVariables(obj->control, startIndex, length, values));

    return kOK;
}

GoFx(kSize) GoSensor_GetRuntimeVariableCount(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kSize value;

    if (!kSuccess(GoControl_RuntimeVariableCount(obj->control, &value)))
    {
        return 0;
    }

    return value;
}

GoFx(kStatus) GoSensor_GetRuntimeVariables(GoSensor sensor, kSize startIndex, kSize length, k32s* values)
{
    kObj(GoSensor, sensor);

    kCheckArgs(!kIsNull(values));

    kCheck(GoControl_GetRuntimeVariables(obj->control, startIndex, length, values));

    return kOK;
}

GoFx(kStatus) GoSensor_GetRuntimeVariableAt(GoSensor sensor, kSize index, k32s* value)
{
    kCheckArgs(!kIsNull(value));

    kCheck(GoSensor_GetRuntimeVariables(sensor, index, 1, value));

    return kOK;
}

// For virtual and accelerated sensors, the hosted sensor's discovery server returns
// the loopback address (127.0.0.1). This limits connectivity to the
// hosted sensor by local SDK application. Remote SDK application
// cannot use this address to connect to the sensor.
// To support remote SDK applications connecting to accelerated (but not virtual) sensors,
// replace the sensor object's loopback address value with the IP of the
// discovery server  that is running in the hosted sensor.
//
// Virtual sensors are never seen outside the platform on which they run so they are
// not connectable by a remote SDK application. However, virtual sensor MUST
// be connectable by a local SDK application.
// Virtual sensors created by the GoEmulator program binds sockets to the
// loopback address. Therefore these virtual sensor object's address must
// remain the loopback address.
// Virtual sensors created by developers using kFramework.exe directly
// binds to the 0.0.0.0 address. Therefore these virtual sensor object's address
// can be changed to the discovery server's address, or left as the loopback
// address.
//
// NOTE:
// Before release 5.1, the discovery server for virtual sensor always
// returned the loopback address.
// For 5.1 and newer releases, the discovery server returns interface addresses
// for all sensors (virtual or standalone or accelerated).
// The 5.1 and newer releases also support the sensor OpMode field which can be used to see
// if the sensor is a virtual sensor or not.
GoFx(kBool) GoSensor_UseDiscoveryServerAddr(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    kBool useServerAddr = kTRUE;

    if (GoSensor_AccelOpMode(sensor) == GO_DISCOVERY_OP_MODE_VIRTUAL)
    {
        // 5.1 and newer sensor software's virtual sensor. Keep the virtual
        // sensor loopback address as is.
        useServerAddr = kFALSE;
    }
    else if (!kIpAddress_IsLoopback(obj->address.address))
    {
        // Not a loopback address. Might have been set by SDK client, so
        // don't change it.
        useServerAddr = kFALSE;
    }

    return useServerAddr;
}

GoFx(kStatus) GoSensor_UpdateDiscoveryInfo(GoSensor sensor, const GoDiscoveryInfo* info)
{
    kObj(GoSensor, sensor);
    kSize historyMask;
    kSize historyIndex;
    GoPortInfo portInfo;

    // Just an efficient way to do modulus
    historyMask = kCountOf(obj->discoveryHistory) - 1;
    historyIndex = obj->discoveryCount & historyMask;

    // There is technically a race condition here i.e. the parameters are modified
    // while they are used in Connect. However the values almost never change
    // so it's unlikely to be a problem. It has never been observed to be a problem
    // before this refactoring.
    if (info != kNULL)
    {
        portInfo = GoDiscoveryInfo_Ports(info);
        obj->address = GoDiscoveryInfo_Address(info);

        if (info->ports.controlPort != k16U_NULL)
        {
            kCheck(GoSensor_SetControlPort(sensor, portInfo.controlPort));
        }

        if (info->ports.upgradePort != k16U_NULL)
        {
            kCheck(GoSensor_SetUpgradePort(sensor, portInfo.upgradePort));
        }

        if (info->ports.healthPort != k16U_NULL)
        {
            kCheck(GoSensor_SetHealthPort(sensor, portInfo.healthPort));
        }

        if (info->ports.dataPort != k16U_NULL)
        {
            kCheck(GoSensor_SetDataPort(sensor, portInfo.dataPort));
        }

        kCheck(GoSensor_UpdateAccelInfo(sensor, info));

        // Update basic sensor information learned via discovery.
        obj->discoveryInfo.firmwareVersion = GoDiscoveryInfo_Version(info);
        kCheck(GoDiscoveryInfo_PartNumber(info, obj->discoveryInfo.partNumber, kCountOf(obj->discoveryInfo.partNumber)));
        kCheck(GoDiscoveryInfo_ModelNumber(info, obj->discoveryInfo.modelNumber, kCountOf(obj->discoveryInfo.modelNumber)));
        kCheck(GoDiscoveryInfo_ModelDisplayName(info, obj->discoveryInfo.modelDisplayName, kCountOf(obj->discoveryInfo.modelDisplayName)));
        kCheck(GoDiscoveryInfo_Family(info, obj->discoveryInfo.family, kCountOf(obj->discoveryInfo.family)));

        // Do this only after the sensor acceleraton information has been
        // updated so that the acceleraton op mode value is valid.
        if (GoSensor_UseDiscoveryServerAddr(sensor))
        {
            // Make sensor accessible from local and remote SDK clients.
            obj->address.address = GoDiscoveryInfo_DiscoveryServerAddress(info);
        }

        obj->discoveryHistory[historyIndex] = kTRUE;
    }
    else
    {
        obj->discoveryHistory[historyIndex] = kFALSE;
    }

    return kOK;
}

GoFx(kStatus) GoSensor_UpdateDiscoveryCycle(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    obj->discoveryCount++;

    return kOK;
}

GoFx(GoAddressInfo*) GoSensor_AddressInfo(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return &obj->address;
}

GoFx(kStatus) GoSensor_SetUpgradePort(GoSensor sensor, k32u port)
{
    kObj(GoSensor, sensor);

    obj->upgradePort = port;

    return kOK;
}

GoFx(k32u) GoSensor_UpgradePort(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->upgradePort;
}

GoFx(kStatus) GoSensor_SetControlPort(GoSensor sensor, k32u port)
{
    kObj(GoSensor, sensor);

    obj->controlPort = port;

    return kOK;
}

GoFx(k32u) GoSensor_ControlPort(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->controlPort;
}

GoFx(GoControl) GoSensor_Control(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->control;
}

GoFx(kStatus) GoSensor_SetDataPort(GoSensor sensor, k32u port)
{
    kObj(GoSensor, sensor);

    obj->dataPort = port;

    return kOK;
}

GoFx(k32u) GoSensor_DataPort(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->dataPort;
}

GoFx(kStatus) GoSensor_SetHealthPort(GoSensor sensor, k32u port)
{
    kObj(GoSensor, sensor);

    obj->healthPort = port;

    return kOK;
}

GoFx(k32u) GoSensor_HealthPort(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->healthPort;
}

GoFx(kStatus) GoSensor_StartRecordingStream(GoSensor sensor, kChar* destPath)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsNormal(sensor));
    kCheck(GoControl_ReadFileStreamed(obj->control, GO_SENSOR_LIVE_REPLAY_STREAM, destPath, kTRUE));

    return kOK;
}

GoFx(kStatus) GoSensor_StopRecordingStream(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kCheck(GoControl_StopStreaming(obj->control));

    // disconnect and reconnect to re-establish the control connection
    kCheck(GoSensor_Disconnect(sensor));
    kCheck(GoSensor_Connect(sensor));

    return kOK;
}

GoFx(kBool) GoSensor_IsRecordingStreaming(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return GoControl_IsStreamingStopped(obj->control);
}

GoFx(GoSensorInfo) GoSensor_BuddiesAt(GoSensor sensor, k32u index)
{
    kObj(GoSensor, sensor);

    if (index >= kArrayList_Count(obj->buddySensorInfo))
    {
        return kNULL;
    }

    return *kArrayList_AtT(obj->remoteSensorInfo, index, GoSensorInfo);
}

GoFx(kSize) GoSensor_BuddiesCount(GoSensor sensor)
{
    kObj(GoSensor, sensor);
    // Should we set to something "invalid" here?
    kSize buddyCount = 0;

    kCheck(GoSensor_LockState(sensor));
    if (kSuccess(GoSensor_CacheInfo(sensor)))
    {
        if (GoSensor_Role(sensor) == GO_ROLE_MAIN)
        {
            buddyCount = kArrayList_Count(obj->buddySensorInfo);
        }
    }
    kCheck(GoSensor_UnlockState(sensor));

    return buddyCount;
}

GoFx(kBool) GoSensor_HasBuddies(GoSensor sensor)
{
    return GoSensor_BuddiesCount(sensor) > 0;
}

GoFx(kStatus) GoSensor_GeoCal(GoSensor sensor, GoGeoCal *geoCal)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kByte* fileData = kNULL;
    kSize fileSize = 0;
    kXml xml = kNULL;
    GoGeoCal newGeoCal = kNULL;
    kStatus exception, readOK;

    readOK = GoControl_ReadFile(obj->control, GO_FILE_GEOCAL, &fileData, &fileSize, alloc);

    if (kOK != readOK)
    {
        //Couldn't read the file, probably because it doesnt exist on this sensor.
        *geoCal = kNULL;
        return kOK;
    }

    kTry
    {
        kTest(kXml_LoadBytes(&xml, fileData, fileSize, kNULL));

        kTest(GoGeoCal_Construct(&newGeoCal, xml, kNULL));

        *geoCal = newGeoCal;
    }
    kCatchEx(&exception)
    {
        //Read the geocal from the filesystem but couldn't parse it properly?
        kObject_Destroy(newGeoCal);
        *geoCal = kNULL;
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kAlloc_Free(alloc, fileData);
        kObject_Destroy(xml);
        kEndFinallyEx();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_UpdateAccelState(GoSensor sensor, const GoDiscoveryInfo* info, GoDiscoveryOpMode opMode)
{
    kObj(GoSensor, sensor);
    GoSensorAccelState accelState;

    // Accelerator (SDK) software version must match exactly with
    // sensor/accelerator host software version.
    // This also handles the case where the operational mode is unknown, because this
    // means the sensor/accelerator host software pre-dates this current SDK code.
    if (kVersion_Compare(GoDiscoveryInfo_Version(info), GoSdk_Version()) != 0)
    {
        accelState = GO_SENSOR_ACCEL_STATE_FW_MISMATCH;

        // Don't need to fill out the rest of the field since this sensor
        // cannot be accelerated.
    }
    else
    {
        // Discovered sensor is either standalone or accelerated, and has the same
        // software version as this current SDK code.
        if (opMode == GO_DISCOVERY_OP_MODE_STANDALONE)
        {
            accelState = GO_SENSOR_ACCEL_STATE_AVAILABLE;
        }
        else
        {
            // Main controller responding must be the sensor's accelerator host.
            // From the given information, sensor class cannot determine at this
            // point whether sensor is accelerated by this host or another host.
            // Leave this decision up to the accelerator code to fine tune the
            // acceleration state.
            accelState = GO_SENSOR_ACCEL_STATE_ACCELERATED;
        }
    }

    kCheck(GoSensor_SetAccelState(sensor, accelState));

    return kOK;
}

// This function stores the accelerated sensor's IP address. This address is
// only available if the sensor is being accelerated, regardless of which
// host is accelerating it.
GoFx(kStatus) GoSensor_UpdateAccelSensorIpAddr(GoSensor sensor, const GoDiscoveryInfo* info, GoDiscoveryOpMode opMode)
{
    kObj(GoSensor, sensor);
    kIpAddress ipAddress;

    // Accelerated sensor IP address is meaningful only if sensor is accelerated,
    // regardless of host. So copy/set the address only if sensor is accelerated.
    if (opMode == GO_DISCOVERY_OP_MODE_ACCELERATOR)
    {
        ipAddress = GoDiscoveryInfo_AccelSensorIpAddress(info);
    }
    else
    {
        ipAddress = GoDiscoveryInfo_Address(info).address;
    }

    kCheck(GoSensor_SetAccelSensorIpAddress(sensor, &ipAddress));

    return kOK;
}

GoFx(kStatus) GoSensor_UpdateAccelInfo(GoSensor sensor, const GoDiscoveryInfo* info)
{
    kObj(GoSensor, sensor);
    GoDiscoveryOpMode opMode;
    GoPortInfo portInfo;

    kCheck(GoSensor_LockState(sensor));

    kTry
    {
        // Save the operational mode always, and do it first so it is set even
        // if mode is virtual or unknown.
        obj->accelInfo.opMode = GoDiscoveryInfo_OpMode(info);
        opMode = GoSensor_AccelOpMode(sensor);

        // Ignore virtual sensors because they cannot be accelerated.
        // Only track standalone or accelerated sensors.
        // For older sensor software, the operational mode is unknown, so
        // accept those. However, these sensors would be marked as firmware mismatch.
        if (opMode != GO_DISCOVERY_OP_MODE_VIRTUAL)
        {
            kTest(GoSensor_UpdateAccelState(sensor, info, opMode));

            // Ports used by sensor is always valid.
            portInfo = GoDiscoveryInfo_Ports(info);
            kTest(GoSensor_SetAccelPortInfo(sensor, &portInfo));

            // Get accelerated sensor IP only after determining the acceleration state.
            // Want to filter out software version incompatible sensors and unaccelerated sensor.
            kTest(GoSensor_UpdateAccelSensorIpAddr(sensor, info, GoSensor_AccelOpMode(sensor)));
        }

    }
    kFinally
    {
        kCheck(GoSensor_UnlockState(sensor));

        kEndFinally();
    }

    return kOK;
}

GoFx(GoSensorAccelState) GoSensor_AccelState(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->accelInfo.state;
}

GoFx(kStatus) GoSensor_SetAccelState(GoSensor sensor, GoSensorAccelState accelState)
{
    kObj(GoSensor, sensor);

    obj->accelInfo.state = accelState;

    return kOK;
}

GoFx(GoPortInfo) GoSensor_AccelPortInfo(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->accelInfo.portInfo;
}

GoFx(kStatus) GoSensor_SetAccelPortInfo(GoSensor sensor, GoPortInfo* portInfo)
{
    kObj(GoSensor, sensor);

    obj->accelInfo.portInfo = *portInfo;

    return kOK;
}

GoFx(GoDiscoveryOpMode) GoSensor_AccelOpMode(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    return obj->accelInfo.opMode;
}

GoFx(kStatus) GoSensor_SetAccelOpMode(GoSensor sensor, GoDiscoveryOpMode opMode)
{
    kObj(GoSensor, sensor);

    obj->accelInfo.opMode = opMode;

    return kOK;
}

GoFx(kStatus) GoSensor_AccelSensorIpAddress(GoSensor sensor, kIpAddress* ipAddress)
{
    kObj(GoSensor, sensor);

    kCheck(GoSensor_LockState(sensor));
    {
        *ipAddress = obj->accelInfo.accelSensorIp;
    }
    kCheck(GoSensor_UnlockState(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_SetAccelSensorIpAddress(GoSensor sensor, kIpAddress* ipAddress)
{
    kObj(GoSensor, sensor);

    obj->accelInfo.accelSensorIp = *ipAddress;

    return kOK;
}

GoFx(kStatus) GoSensor_SetSecurityLevel(GoSensor sensor, GoSecurityLevel security)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_SetSecurityLevel(obj->control, security));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_SetFlag(GoSensor sensor, const kChar* name, const kChar* value)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_SetFlag(obj->control, name, value));

    kCheck(GoSensor_InvalidateInfo(sensor));

    return kOK;
}

GoFx(kStatus) GoSensor_GetFlag(GoSensor sensor, const kChar* name, kString value)
{
    kObj(GoSensor, sensor);

    kCheckState(GoSensor_IsResponsive(sensor));

    kCheck(GoControl_GetFlag(obj->control, name, value));

    return kOK;
}

GoFx(kStatus) GoSensor_AppendBuddy(GoSensor sensor, GoSensor buddy)
{
    kObj(GoSensor, sensor);
    kAlloc alloc = kObject_Alloc(sensor);
    kArrayList buddyIds = kNULL;

    kCheck(kArrayList_Construct(&buddyIds, kTypeOf(k32u), 16, alloc));

    kTry
    {
        kTest(GoSensor_CreateBuddyList(sensor, &buddyIds));

        kSize buddyCount = GoSensor_BuddiesCount(sensor);

        k32u buddyId = GoSensor_Id(buddy);
        kTest(kArrayList_AddT(buddyIds, &buddyId));
        buddyCount++;

        kTest(GoControl_AssignBuddies(obj->control, kArrayList_DataT(buddyIds, k32u), buddyCount));
    }
    kFinally
    {
        kDestroyRef(&buddyIds);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSensor_CreateBuddyList(GoSensor sensor, kArrayList* buddyIds)
{
    kObj(GoSensor, sensor);
    kSize buddyCount = kArrayList_Count(obj->buddySensorInfo);

    for (kSize i = 0; i < buddyCount; ++i)
    {
        kCheck(kArrayList_AddT(*buddyIds, &kArrayList_AtT(obj->buddySensorInfo, i, GoBuddyInfo)->id));
    }

    return kOK;
}

GoFx(kStatus) GoSensor_SetTimeDate(GoSensor sensor)
{
    kObj(GoSensor, sensor);

    kDateTime dateTime = kDateTime_Now();
    GoControl_SetDataTime(obj->control, dateTime);

    return kOK;
}