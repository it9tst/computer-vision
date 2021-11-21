/** 
 * @file    GoPartModel.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoPartModel.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoPartModelEdge)
    kAddVMethod(GoPartModelEdge, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoPartModelEdge_Construct(GoPartModelEdge* partModelEdge, kObject parentModel, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartModelEdge), partModelEdge)); 

    if (!kSuccess(status = GoPartModelEdge_Init(*partModelEdge, kTypeOf(GoPartModelEdge), parentModel, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, partModelEdge); 
    }

    return status; 
} 

GoFx(kStatus) GoPartModelEdge_Init(GoPartModelEdge partModelEdge, kType type, kObject parentModel, kObject sensor, kAlloc alloc)
{
    kObjR(GoPartModelEdge, partModelEdge); 
    kStatus exception = kOK;

    kCheck(kObject_Init(partModelEdge, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->imageType = 0;
    obj->imageSource = GO_DATA_SOURCE_TOP;
    kZero(obj->removedPoints);

    obj->partModelParent = parentModel;
    obj->sensor = sensor; 

    kTry
    {
        kTest(kArrayList_Construct(&obj->removedPoints, kTypeOf(k32u), 0, alloc));
    }
    kCatch(&exception)
    {
        GoPartModelEdge_VRelease(partModelEdge);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoPartModelEdge_VRelease(GoPartModelEdge partModelEdge)
{
    kObj(GoPartModelEdge, partModelEdge); 

    kDisposeRef(&obj->removedPoints);

    return kObject_VRelease(partModelEdge); 
}

GoFx(kStatus) GoPartModelEdge_Read(GoPartModelEdge modelEdge, kXml xml, kXmlItem item)
{
    kObj(GoPartModelEdge, modelEdge);
    kXmlItem enabledItem = kNULL;
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kString tempStr = kNULL;

    obj->xml = xml;
    obj->xmlItem = item;

    kTry
    {
        kTest(kString_Construct(&tempStr, kNULL, kObject_Alloc(modelEdge)));

        kTest(kXml_Child32s(xml, item, "ImageType", &obj->imageType));
        kTest(kXml_Child32s(xml, item, "ImageSource", &obj->imageSource));
        kTest(kXml_ChildString(xml, item, "RemovedPoints", tempStr));

        kTest(GoOptionList_ParseList32u(kString_Chars(tempStr), obj->removedPoints));
    }
    kFinally
    {
        kDestroyRef(&tempStr);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoPartModelEdge_Write(GoPartModelEdge partModelEdge, kXml xml, kXmlItem item)
{
    kObj(GoPartModelEdge, partModelEdge); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kString tempStr = kNULL;

    kTry
    {
        kTest(kString_Construct(&tempStr, kNULL, kObject_Alloc(partModelEdge)));

        kTest(GoOptionList_Format32uString(
            kArrayList_DataT(obj->removedPoints, k32u),
            kArrayList_Count(obj->removedPoints),
            tempStr));
        kTest(kXml_SetChildText(xml, item, "RemovedPoints", kString_Chars(tempStr)));

        //Forwards Compatibility
        kTest(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));
    }
    kFinally
    {
        kDestroyRef(&tempStr);
        kEndFinally();
    }

    return kOK; 
}

GoFx(GoImageType) GoPartModelEdge_ImageType(GoPartModelEdge edge)
{
    kObj(GoPartModelEdge, edge); 

    kCheck(GoSensor_SyncPartModels(obj->sensor));

    return obj->imageType;
}

GoFx(GoDataSource) GoPartModelEdge_ImageSource(GoPartModelEdge edge)
{
    kObj(GoPartModelEdge, edge); 

    kCheck(GoSensor_SyncPartModels(obj->sensor));

    return obj->imageSource;
}

GoFx(kStatus) GoPartModelEdge_SetRemovedPoints(GoPartModelEdge edge, const k32u* points, kSize length)
{
    kObj(GoPartModelEdge, edge);
    kSize i;

    kCheck(kArrayList_Purge(obj->removedPoints));

    for (i = 0; i < length; i++)
    {
        kCheck(kArrayList_AddT(obj->removedPoints, &points[i]));
    }

    kCheck(GoPartModel_SetModified(obj->partModelParent));

    return kOK;
}

GoFx(kStatus) GoPartModelEdge_RemovedPoints(GoPartModelEdge edge, const k32u* points, kSize* length)
{
    kObj(GoPartModelEdge, edge); 

    GoSensor_SyncPartModels(obj->sensor);

    points = kArrayList_DataT(obj->removedPoints, k32u);
    *length = kArrayList_Count(obj->removedPoints);

    return kOK;
}

GoFx(kSize) GoPartModelEdge_RemovedPointsLength(GoPartModelEdge edge)
{
    kObj(GoPartModelEdge, edge); 

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->removedPoints);
}

GoFx(k32u) GoPartModelEdge_RemovedPointAt(GoPartModelEdge edge, kSize index)
{
    kObj(GoPartModelEdge, edge); 

    GoSensor_SyncPartModels(obj->sensor);

    kAssert(index < GoPartModelEdge_RemovedPointsLength(edge));

    return *kArrayList_AtT(obj->removedPoints, index, k32u);
}


kBeginClassEx(Go, GoPartModel)
    kAddVMethod(GoPartModel, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoPartModel_Construct(GoPartModel* partModel, kObject sensor, const kChar* name, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoPartModel), partModel)); 

    if (!kSuccess(status = GoPartModel_Init(*partModel, kTypeOf(GoPartModel), sensor, name, alloc)))
    {
        kAlloc_FreeRef(alloc, partModel); 
    }

    return status; 
} 

GoFx(kStatus) GoPartModel_Init(GoPartModel partModel, kType type, kObject sensor, const kChar* name, kAlloc alloc)
{
    kObjR(GoPartModel, partModel); 
    kStatus exception = kOK;

    kCheck(kObject_Init(partModel, type, alloc)); 
    kZero(obj->sensor);
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->name[0] = 0;
    kZero(obj->edges);
    obj->transformedDataRegion = kNULL;
    obj->imageType = 0;
    kZero(obj->imageTypeOptions);
    obj->targetEdgeSensitivity = 0.0;
    obj->edgeSensitivity = 0.0;
    obj->zAngle = 0.0;
    obj->isValid = kFALSE;
    obj->isModified = kFALSE;

    kCheck(kStrCopy(obj->name, 64, name));
    obj->sensor = sensor; 
    
    kTry
    {
        kTest(kArrayList_Construct(&obj->edges, kTypeOf(GoPartModelEdge), 0, alloc));
        kTest(kArrayList_Construct(&obj->imageTypeOptions, kTypeOf(k32s), 0, alloc));
        kTest(GoRegion3d_Construct(&obj->transformedDataRegion, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoPartModel_VRelease(partModel);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoPartModel_VRelease(GoPartModel partModel)
{
    kObj(GoPartModel, partModel); 

    kDestroyRef(&obj->xml);
    kDestroyRef(&obj->transformedDataRegion);
    kDisposeRef(&obj->edges);
    kDestroyRef(&obj->imageTypeOptions);

    return kObject_VRelease(partModel); 
}

GoFx(kStatus) GoPartModel_Read(GoPartModel model, kXml xml, kXmlItem item)
{
    kObj(GoPartModel, model);
    kXmlItem tempItem = kNULL;
    kSize i;
    kText256 tempText;
    kObjN(GoRegion3d, region, obj->transformedDataRegion);
    kStatus exception = kOK;
    GoPartModelEdge constructedModelEdge = kNULL;

    kCheck(kDestroyRef(&obj->xml));

    obj->xml = xml;
    obj->xmlItem = item;

    tempItem = kXml_Child(xml, item, "Edges");
    if (!kIsNull(tempItem))
    {
        kTry
        {
            for (i = 0; i < kXml_ChildCount(xml, tempItem); i++)
            {
                kXmlItem tempItemInner = kXml_ChildAt(xml, tempItem, i);
                GoPartModelEdge modelEdge;

                if (kArrayList_Count(obj->edges) <= i)
                {
                    kTest(GoPartModelEdge_Construct(&constructedModelEdge, model, obj->sensor, kObject_Alloc(model)));
                    modelEdge = constructedModelEdge;

                    kTest(kArrayList_AddT(obj->edges, &constructedModelEdge));
                    constructedModelEdge = kNULL;
                }
                else
                {
                    modelEdge = *kArrayList_AtT(obj->edges, i, GoPartModelEdge);
                }

                kTestArgs(strcmp("Edge", kXml_ItemName(xml, tempItemInner)) == 0);
                kTest(GoPartModelEdge_Read(modelEdge, xml, tempItemInner));
            }
        }
        kCatch(&exception)
        {
            kDestroyRef(&constructedModelEdge);
            kEndCatch(exception);
        }
    }
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/X", &region->x));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Y", &region->y));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Z", &region->z));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Width", &region->width));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Length", &region->length));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Height", &region->height));

    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "ImageType")));
    kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
    kCheck(GoOptionList_ParseList32u(tempText, obj->imageTypeOptions));
    kCheck(kXml_Item32s(xml, tempItem, &obj->imageType));

    kCheck(kXml_Child64f(xml, item, "TargetEdgeSensitivity", &obj->targetEdgeSensitivity));
    kCheck(kXml_Child64f(xml, item, "EdgeSensitivity", &obj->edgeSensitivity));
    kCheck(kXml_Child64f(xml, item, "ZAngle", &obj->zAngle));

    obj->isModified = kFALSE;
    obj->isValid = kTRUE; 

    return kOK;
}

GoFx(kStatus) GoPartModel_Write(GoPartModel partModel, kXml xml, kXmlItem item)
{
    kObj(GoPartModel, partModel); 
    kXmlItem tempItem = kNULL;
    kXmlItem tempItemInner = kNULL;
    kSize i;

    kCheck(kXml_AddItem(xml, item, "Edges", &tempItem));
    
    for (i = 0; i < GoPartModel_EdgeCount(partModel); i++)
    {
        kCheck(kXml_AddItem(xml, tempItem, "Edge", &tempItemInner));
        kCheck(GoPartModelEdge_Write(GoPartModel_EdgeAt(partModel, i), xml, tempItemInner));
    }

    kCheck(kXml_SetChild32s(xml, item, "ImageType", obj->imageType));
    kCheck(kXml_SetChild64f(xml, item, "TargetEdgeSensitivity", obj->targetEdgeSensitivity));
    kCheck(kXml_SetChild64f(xml, item, "EdgeSensitivity", obj->edgeSensitivity));
    kCheck(kXml_SetChild64f(xml, item, "ZAngle", obj->zAngle));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(const kChar*) GoPartModel_Name(GoPartModel model)
{
    kObj(GoPartModel, model);

    return obj->name;
}

GoFx(kSize) GoPartModel_EdgeCount(GoPartModel model)
{
    kObj(GoPartModel, model);

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->edges);
}

GoFx(GoPartModelEdge) GoPartModel_EdgeAt(GoPartModel model, kSize index)
{
    kObj(GoPartModel, model);

    GoSensor_SyncPartModels(obj->sensor);

    kAssert(index < GoPartModel_EdgeCount(model));

    return *kArrayList_AtT(obj->edges, index, GoPartModelEdge);
}

GoFx(k64f) GoPartModel_TransformedDataRegionX(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_X(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionY(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Y(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionZ(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Z(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionWidth(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Width(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionLength(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Length(obj->transformedDataRegion);
}

GoFx(k64f) GoPartModel_TransformedDataRegionHeight(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return GoRegion3d_Height(obj->transformedDataRegion);
}

GoFx(GoImageType) GoPartModel_ImageType(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->imageType;
}

GoFx(kStatus) GoPartModel_SetImageType(GoPartModel partModel, GoImageType value)
{
    kObj(GoPartModel, partModel);

    obj->imageType = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(GoImageType) GoPartModel_ImageTypeOptionAt(GoPartModel partModel, kSize index)
{
    kObj(GoPartModel, partModel);

    kCheckArgs(index < kArrayList_Count(obj->imageTypeOptions));
    
    return *kArrayList_AtT(obj->imageTypeOptions, index, GoImageType);
}

GoFx(kSize) GoPartModel_ImageTypeOptionCount(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return kArrayList_Count(obj->imageTypeOptions);
}

GoFx(k64f) GoPartModel_TargetEdgeSensitivity(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->targetEdgeSensitivity;
}

GoFx(kStatus) GoPartModel_SetTargetEdgeSensitivity(GoPartModel partModel, k64f value)
{
    kObj(GoPartModel, partModel);

    obj->targetEdgeSensitivity = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(k64f) GoPartModel_EdgeSensitivity(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->edgeSensitivity;
}

GoFx(k64f) GoPartModel_ZAngle(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    GoSensor_SyncPartModels(obj->sensor);

    return obj->zAngle;
}

GoFx(kStatus) GoPartModel_SetZAngle(GoPartModel partModel, k64f value)
{
    kObj(GoPartModel, partModel);

    obj->zAngle = value;
    kCheck(GoPartModel_SetModified(partModel));

    return kOK;
}

GoFx(kStatus) GoPartModel_SetModified(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    obj->isModified = kTRUE;

    return kOK;
}

GoFx(kBool) GoPartModel_IsModified(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    return obj->isModified;
}

GoFx(kBool) GoPartModel_IsValid(GoPartModel partModel)
{
    kObj(GoPartModel, partModel);

    return obj->isValid;
}
