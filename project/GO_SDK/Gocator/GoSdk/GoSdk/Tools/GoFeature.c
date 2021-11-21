/** 
 * @file    GoFeature.c
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Tools/GoFeatures.h>
#include <GoSdk/Tools/GoFeature.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoFeature)
    kAddVMethod(GoFeature, kObject, VRelease)
    kAddVMethod(GoFeature, GoFeature, VInit)
    kAddVMethod(GoFeature, GoFeature, VRead)
    kAddVMethod(GoFeature, GoFeature, VWrite)
kEndClassEx()

GoFx(kStatus) GoFeature_Construct(GoFeature* feature, 
                                    kType type, 
                                    kObject sensor, 
                                    kObject srcTool, 
                                    kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, feature)); 

    if (!kSuccess(status = kType_VTableT(type, GoFeature)->VInit(*feature, type, sensor, srcTool, alloc)))
    {
        kAlloc_FreeRef(alloc, feature); 
    }

    return status;
} 

GoFx(kStatus) GoFeature_VInit(GoFeature feature, kType type, kObject sensor, kObject srcTool, kAlloc alloc)
{
    kObjR(GoFeature, feature);

    kCheck(kObject_Init(feature, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->name[0] = 0;
    obj->type[0] = 0;

    obj->sensor = sensor;
    obj->id = GO_FEATURE_UNASSIGNED_ID;
    obj->srcTool = srcTool;
    obj->enabled = kFALSE;
    obj->typeId = GO_FEATURE_EXTENSIBLE; //to Discuss H.Ware
    obj->dataType = GO_FEATURE_DATA_UNKNOWN;

    return kOK;//TO discuss H.Ware kERROR_UNIMPLEMENTED;  //this function must be overriden by every feature
}

GoFx(kStatus) GoFeature_Init(GoFeature feature, kType type, GoFeatureType typeId, kObject sensor, kObject srcTool, GoFeatureDataType dataType, kAlloc alloc)
{
    kObjR(GoFeature, feature);
    
    kCheck(kObject_Init(feature, type, alloc));
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->name[0] = 0;
    obj->type[0] = 0;

    obj->sensor = sensor;
    obj->id = GO_FEATURE_UNASSIGNED_ID;
    obj->srcTool = srcTool;
    obj->enabled = kFALSE;
    obj->typeId = typeId;
    obj->dataType = dataType;

    return kOK; 
}

GoFx(kStatus) GoFeature_VRelease(GoFeature feature)
{
    kObj(GoFeature, feature); 
            
    kCheck(kObject_VRelease(feature)); 

    return kOK; 
}

GoFx(kStatus) GoFeature_VRead(GoFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoFeature, feature); 
    kText64 tempStr;
    obj->xml = xml;
    obj->xmlItem = item;

    if(!kSuccess(kXml_Attr32s(xml, item, "id", &obj->id)))
    {
        obj->id = GO_FEATURE_UNASSIGNED_ID;
    }
    
    kCheck(kXml_ChildText(xml, item, "Name", obj->name, kCountOf(obj->name)));    

    kCheck(kXml_ChildBool(xml, item, "Enabled", &obj->enabled));
    
    kCheck(kXml_AttrText(xml, item, "dataType", tempStr, kCountOf(tempStr)));
    kCheck(GoFeatures_ParseDataType(tempStr, &obj->dataType));

    kCheck(kStrCopy(obj->type, kCountOf(obj->type), kXml_ItemName(xml, item)));

    return kOK; 
}

GoFx(kStatus) GoFeature_VWrite(GoFeature feature, kXml xml, kXmlItem item)
{
    kObj(GoFeature, feature); 
    kText64 tempStr;

    kCheck(kXml_SetAttr32s(xml, item, "id", obj->id));
    kCheck(kXml_SetChildText(xml, item, "Name", obj->name));

    kCheck(kXml_SetChildBool(xml, item, "Enabled", obj->enabled));
    
    if (obj->dataType != GO_FEATURE_DATA_UNKNOWN)
    {
        kCheck(GoFeatures_FormatDataType(obj->dataType, tempStr, kCountOf(tempStr)));
        kCheck(kXml_SetAttrText(xml, item, "dataType", tempStr));
    }

    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}


GoFx(kStatus) GoFeature_Read(GoFeature feature, kXml xml, kXmlItem item)
{
    return kCast(GoFeatureVTable*, xkObject_VTable(feature))->VRead(feature, xml, item);
}

GoFx(kStatus) GoFeature_Write(GoFeature feature, kXml xml, kXmlItem item)
{
    return kCast(GoFeatureVTable*, xkObject_VTable(feature))->VWrite(feature, xml, item);
}

GoFx(kBool) GoFeature_HasId(GoFeature feature)
{
    kObj(GoFeature, feature); 

    GoSensor_SyncConfig(obj->sensor);

    return (obj->id != GO_FEATURE_UNASSIGNED_ID);
}

GoFx(kStatus) GoFeature_ClearId(GoFeature feature)
{
    kObj(GoFeature, feature); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->id = GO_FEATURE_UNASSIGNED_ID;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoFeature_SetId(GoFeature feature, k32u id)
{
    kObj(GoFeature, feature); 

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    obj->id = (k32s) id;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoFeature_Id(GoFeature feature)
{
    kObj(GoFeature, feature);

    GoSensor_SyncConfig(obj->sensor); 

    return obj->id;
}

GoFx(const kChar*) GoFeature_Name(GoFeature feature)
{
    kObj(GoFeature, feature);

    GoSensor_SyncConfig(obj->sensor);

    return obj->name;
}

GoFx(kStatus) GoFeature_SetName(GoFeature feature, const kChar* name)
{
    kObj(GoFeature, feature);

    kCheckState(GoSensor_IsConfigurable(GoFeature_Sensor(feature)));
    kCheck(GoSensor_CacheConfig(GoFeature_Sensor(feature)));
    kCheck(kStrCopy(obj->name, kCountOf(obj->name), name));
    kCheck(GoSensor_SetConfigModified(GoFeature_Sensor(feature)));

    return kOK;
}

GoFx(kObject) GoFeature_Sensor(GoFeature feature)
{
    kObj(GoFeature, feature);

    return obj->sensor;
}

GoFx(kObject) GoFeature_SourceTool(GoFeature feature)
{
    kObj(GoFeature, feature);

    GoSensor_SyncConfig(GoFeature_Sensor(feature));

    return obj->srcTool;
}

GoFx(kStatus) GoFeature_Enable(GoFeature feature, kBool enable)
{
    kObj(GoFeature, feature); 

    kCheckState(GoSensor_IsConfigurable(GoFeature_Sensor(feature)));
    kCheck(GoSensor_CacheConfig(GoFeature_Sensor(feature)));
    obj->enabled = enable;
    kCheck(GoSensor_SetConfigModified(GoFeature_Sensor(feature)));

    return kOK;
}

GoFx(kBool) GoFeature_Enabled(GoFeature feature)
{
    kObj(GoFeature, feature); 

    GoSensor_SyncConfig(GoFeature_Sensor(feature));

    return obj->enabled;
}

GoFx(const kChar*) GoFeature_Type(GoFeature feature)
{
    kObj(GoFeature, feature);

    GoSensor_SyncConfig(GoFeature_Sensor(feature));

    return obj->type;
}


GoFx(GoFeatureDataType) GoFeature_DataType(GoFeature feature)
{
    kObj(GoFeature, feature); 

    GoSensor_SyncConfig(GoFeature_Sensor(feature));

    return obj->dataType;
}

GoFx(GoFeatureType) GoFeature_TypeId(GoFeature feature)
{
    kObj(GoFeature, feature);

    GoSensor_SyncConfig(GoFeature_Sensor(feature));

    return obj->typeId;
}
