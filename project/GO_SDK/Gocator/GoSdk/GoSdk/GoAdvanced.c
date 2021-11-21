/** 
 * @file    GoAdvanced.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoAdvanced.h>
#include <GoSdk/GoSensor.h>
#include <GoSdk/GoUtils.h>

#define GO_SPOT_THRESHOLD_MIN               1
#define GO_SPOT_THRESHOLD_MAX               255

#define GO_SURFACE_CONTRAST_THRESHOLD_MIN   1
#define GO_SURFACE_CONTRAST_THRESHOLD_MAX   50

kBeginClassEx(Go, GoAdvanced)
    kAddVMethod(GoAdvanced, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoAdvanced_Construct(GoAdvanced* advanced, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoAdvanced), advanced)); 

    if (!kSuccess(status = GoAdvanced_Init(*advanced, kTypeOf(GoAdvanced), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, advanced); 
    }

    return status; 
} 

GoFx(kStatus) GoAdvanced_Init(GoAdvanced advanced, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoAdvanced, advanced); 
    kStatus exception = kOK;

    kCheck(kObject_Init(advanced, type, alloc)); 
    kZero(obj->xml);
    kZero(obj->xmlItem);
    obj->typeSystemValue = GO_ADVANCED_TYPE_CUSTOM;
    obj->typeUsed = kFALSE;
    kZero(obj->spotThreshold);
    obj->spotThreshold.min = GO_SPOT_THRESHOLD_MIN;
    obj->spotThreshold.max = GO_SPOT_THRESHOLD_MAX;
    kZero(obj->spotWidthMax);
    obj->systemSpotSelectionType = GO_SPOT_SELECTION_TYPE_BEST;
    obj->spotSelectionTypeUsed = kFALSE;
    obj->spotSelectionTypeOptions = kNULL;
    obj->spotContinuitySortingMinimumSegmentSize = 0;
    obj->spotContinuitySortingSearchWindowX = 0;
    obj->spotContinuitySortingSearchWindowY = 0;

    kZero(obj->spotTranslucentSortingOpaqueWidth);
    kZero(obj->spotTranslucentSortingTranslucentWidth);
    kZero(obj->spotTranslucentSortingMinLength);
    kZero(obj->spotTranslucentSortingThreadingMode);
    obj->spotTranslucentSortingThreadingModeOptions = kNULL;

    kZero(obj->spotWidthMin);
    kZero(obj->widthThreshold);
    kZero(obj->spotSumMin);
    kZero(obj->sobelEdgeWindow);
    kZero(obj->cameraGainAnalog);
    kZero(obj->cameraGainDigital);
    kZero(obj->dynamicSensitivity);
    kZero(obj->dynamicThreshold);
    kZero(obj->gammaType);
    kZero(obj->sensitivityCompensationEnabled);
    kZero(obj->encoding);
    obj->encoding.value = GO_SURFACE_ENCODING_STANDARD;
    obj->phaseFilter = GO_SURFACE_PHASE_FILTER_NONE;

    obj->sensor = sensor; 
    
    obj->type = GO_ADVANCED_TYPE_DIFFUSE;
    obj->spotSelectionType = GO_SPOT_SELECTION_TYPE_BEST;

    kZero(obj->contrastThreshold);
    obj->contrastThreshold.min = GO_SURFACE_CONTRAST_THRESHOLD_MIN;
    obj->contrastThreshold.max = GO_SURFACE_CONTRAST_THRESHOLD_MAX;

    kTry
    {
        kTest(kArrayList_Construct(&obj->spotSelectionTypeOptions, kTypeOf(GoSpotSelectionType), 0, alloc));
        kTest(kArrayList_Construct(&obj->spotTranslucentSortingThreadingModeOptions, kTypeOf(GoTranslucentThreadingMode), 0, alloc));
    }
    kCatch(&exception)
    {
        GoAdvanced_VRelease(advanced);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoAdvanced_VRelease(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced); 

    kCheck(kDisposeRef(&obj->spotSelectionTypeOptions));
    kCheck(kDisposeRef(&obj->spotTranslucentSortingThreadingModeOptions));

    return kObject_VRelease(advanced);
}

GoFx(kStatus) GoAdvanced_ReadTunableParams(GoAdvanced advanced, kChar* paramStr, GoElement32u* element, kXml xml, kXmlItem item)
{
    kXmlItem tempItem = kNULL;
    kBool tempBool;

    if (kXml_ChildExists(xml, item, paramStr))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, paramStr)));
        kCheck(kXml_Item32u(xml, tempItem, &element->value));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &element->enabled));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &element->min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &element->max));
        if (element->enabled && kXml_AttrExists(xml, tempItem, "readonly"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "readonly", &tempBool));
            element->enabled = !tempBool;
        }
    }

    return kOK;
}

GoFx(kStatus) GoAdvanced_Read(GoAdvanced advanced, kXml xml, kXmlItem item)
{
    kObj(GoAdvanced, advanced); 
    kXmlItem tempItem = kNULL;
    kText256 tempText;
    kBool tempBool;

    obj->xml = xml;
    obj->xmlItem = item;

    // Type -------------------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Type")));    
    kCheck(kXml_Item32s(xml, tempItem, &obj->type)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->typeUsed));
    kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->typeSystemValue));
    
    // SpotThreshold ----------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotThreshold")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->spotThreshold.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotThreshold.enabled));
    // Enabled if used AND writeable.
    if (obj->spotThreshold.enabled && kXml_AttrExists(xml, tempItem, "readonly"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "readonly", &tempBool));
        obj->spotThreshold.enabled = !tempBool;
    }
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotThreshold.systemValue));
    if (kXml_AttrExists(xml, tempItem, "min") && kXml_AttrExists(xml, tempItem, "max"))
    {
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotThreshold.min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotThreshold.max));
    }

    // SpotWidthMax -----------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotWidthMax")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->spotWidthMax.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotWidthMax.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotWidthMax.systemValue));
    kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotWidthMax.min));
    kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotWidthMax.max));

    // SpotSelectionType ------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotSelectionType")));
    kCheck(kXml_Item32s(xml, tempItem, &obj->spotSelectionType)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotSelectionTypeUsed));
    kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->systemSpotSelectionType));

    if (kXml_AttrExists(xml, tempItem, "options"))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(kArrayList_Clear(obj->spotSelectionTypeOptions));
        kCheck(GoOptionList_ParseList32u(tempText, obj->spotSelectionTypeOptions));
    }

    // Tunable parameters -----------------------------------------------------------
    kCheck(GoAdvanced_ReadTunableParams(advanced, "SpotWidthMin", &obj->spotWidthMin, xml, item));
    kCheck(GoAdvanced_ReadTunableParams(advanced, "WidthThreshold", &obj->widthThreshold, xml, item));
    kCheck(GoAdvanced_ReadTunableParams(advanced, "SpotSumMin", &obj->spotSumMin, xml, item));
    kCheck(GoAdvanced_ReadTunableParams(advanced, "SobelEdgeWindow", &obj->sobelEdgeWindow, xml, item));

    // SpotContinuitySorting --------------------------------------------------
    if (kXml_ChildExists(xml, item, "SpotContinuitySorting"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotContinuitySorting")));
        kCheck(kXml_Child32u(xml, tempItem, "MinimumSegmentSize", &obj->spotContinuitySortingMinimumSegmentSize));
        kCheck(kXml_Child32u(xml, tempItem, "SearchWindow/X", &obj->spotContinuitySortingSearchWindowX));
        kCheck(kXml_Child32u(xml, tempItem, "SearchWindow/Y", &obj->spotContinuitySortingSearchWindowY));
    }

    // SpotTranslucentSorting --------------------------------------------------
    if (kXml_ChildExists(xml, item, "SpotTranslucentSorting/OpaqueWidth"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotTranslucentSorting/OpaqueWidth")));
        kCheck(kXml_Item32u(xml, tempItem, &obj->spotTranslucentSortingOpaqueWidth.value)); 
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotTranslucentSortingOpaqueWidth.enabled));
        kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotTranslucentSortingOpaqueWidth.systemValue));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotTranslucentSortingOpaqueWidth.min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotTranslucentSortingOpaqueWidth.max));
    }

    if (kXml_ChildExists(xml, item, "SpotTranslucentSorting/TranslucentWidth"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotTranslucentSorting/TranslucentWidth")));
        kCheck(kXml_Item32u(xml, tempItem, &obj->spotTranslucentSortingTranslucentWidth.value)); 
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotTranslucentSortingTranslucentWidth.enabled));
        kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotTranslucentSortingTranslucentWidth.systemValue));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotTranslucentSortingTranslucentWidth.min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotTranslucentSortingTranslucentWidth.max));
    }

    if (kXml_ChildExists(xml, item, "SpotTranslucentSorting/MinLength"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotTranslucentSorting/MinLength")));
        kCheck(kXml_Item32u(xml, tempItem, &obj->spotTranslucentSortingMinLength.value)); 
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotTranslucentSortingMinLength.enabled));
        kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->spotTranslucentSortingMinLength.systemValue));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->spotTranslucentSortingMinLength.min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->spotTranslucentSortingMinLength.max));
    }

    if (kXml_ChildExists(xml, item, "SpotTranslucentSorting/ThreadingMode"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SpotTranslucentSorting/ThreadingMode")));
        kCheck(kXml_Item32s(xml, tempItem, &obj->spotTranslucentSortingThreadingMode.value)); 
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spotTranslucentSortingThreadingMode.enabled));
        kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->spotTranslucentSortingThreadingMode.systemValue));
        if (kXml_AttrExists(xml, tempItem, "options"))
        {
            kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
            kCheck(kArrayList_Clear(obj->spotTranslucentSortingThreadingModeOptions));
            kCheck(GoOptionList_ParseList32s(tempText, obj->spotTranslucentSortingThreadingModeOptions));
        }
    }

    // CameraGainAnalog -------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "CameraGainAnalog")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->cameraGainAnalog.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->cameraGainAnalog.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->cameraGainAnalog.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->cameraGainAnalog.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->cameraGainAnalog.max));

    // CameraGainDigital ------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "CameraGainDigital")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->cameraGainDigital.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->cameraGainDigital.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->cameraGainDigital.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->cameraGainDigital.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->cameraGainDigital.max));
    
    // DynamicSensitivity -----------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "DynamicSensitivity")));
    kCheck(kXml_Item64f(xml, tempItem, &obj->dynamicSensitivity.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->dynamicSensitivity.enabled));
    kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->dynamicSensitivity.systemValue));
    kCheck(kXml_Attr64f(xml, tempItem, "min", &obj->dynamicSensitivity.min));
    kCheck(kXml_Attr64f(xml, tempItem, "max", &obj->dynamicSensitivity.max));

    // DynamicThreshold -------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "DynamicThreshold")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->dynamicThreshold.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->dynamicThreshold.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->dynamicThreshold.systemValue));
    kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->dynamicThreshold.min));
    kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->dynamicThreshold.max));

    // GammaType --------------------------------------------------------------
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "GammaType")));
    kCheck(kXml_Item32u(xml, tempItem, &obj->gammaType.value)); 
    kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->gammaType.enabled));
    kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->gammaType.systemValue));

    // SensitivityCompensationEnabled -----------------------------------------
    if (kXml_ChildExists(xml, item, "SensitivityCompensationEnabled"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "SensitivityCompensationEnabled")));
        kCheck(kXml_ItemBool(xml, tempItem, &obj->sensitivityCompensationEnabled.value));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->sensitivityCompensationEnabled.enabled));
        kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->sensitivityCompensationEnabled.systemValue));
    }

    // SurfaceEncoding --------------------------------------------------------
    if (kXml_ChildExists(xml, item, "SurfaceEncoding"))
    {
        tempItem = kXml_Child(xml, item, "SurfaceEncoding"); 
        kCheck(kXml_Item32u(xml, tempItem, &obj->encoding.value)); 
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->encoding.enabled));
        // Enabled if used && writeable.
        if (obj->encoding.enabled && kXml_AttrExists(xml, tempItem, "readonly"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "readonly", &tempBool));
            obj->encoding.enabled = !tempBool;
    }
        kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->encoding.systemValue));
    }

    // SurfacePhaseFilter -----------------------------------------------------
    if (kXml_ChildExists(xml, item, "SurfacePhaseFilter"))
    {
        kCheck(kXml_Child32s(xml, item, "SurfacePhaseFilter", &obj->phaseFilter));
    }

    // ContrastThreshold ------------------------------------------------------
    if (kXml_ChildExists(xml, item, "ContrastThreshold"))
    {
        // Contrast Threshold only became available in 5.3.12.6 when GOC-5534 was implemented.
        // Earlier versions should still proceed with the read, and calls to
        // GoAdvanced_IsContrastThresholdUsed() will return false.
        tempItem = kXml_Child(xml, item, "ContrastThreshold");
        kCheck(kXml_Item32u(xml, tempItem, &obj->contrastThreshold.value)); 

        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->contrastThreshold.enabled));
        // Enabled if used && writeable.
        if (obj->contrastThreshold.enabled && kXml_AttrExists(xml, tempItem, "readonly"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "readonly", &tempBool));
            obj->contrastThreshold.enabled = !tempBool;
        }
        kCheck(kXml_Attr32u(xml, tempItem, "value", &obj->contrastThreshold.systemValue));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->contrastThreshold.min));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->contrastThreshold.max));
    }

    return kOK; 
}

GoFx(kStatus) GoAdvanced_Write(GoAdvanced advanced, kXml xml, kXmlItem item)
{
    kObj(GoAdvanced, advanced); 
    kXmlItem tempItem = kNULL;

    kCheck(kXml_SetChild32s(xml, item, "Type", obj->type));
    kCheck(kXml_SetChild32u(xml, item, "SpotThreshold", obj->spotThreshold.value));
    kCheck(kXml_SetChild32u(xml, item, "SpotWidthMax", obj->spotWidthMax.value));
    kCheck(kXml_SetChild32s(xml, item, "SpotSelectionType", obj->spotSelectionType));

    kCheck(kXml_SetChild32u(xml, item, "SpotWidthMin", obj->spotWidthMin.value));
    kCheck(kXml_SetChild32u(xml, item, "WidthThreshold", obj->widthThreshold.value));
    kCheck(kXml_SetChild32u(xml, item, "SpotSumMin", obj->spotSumMin.value));
    kCheck(kXml_SetChild32u(xml, item, "SobelEdgeWindow", obj->sobelEdgeWindow.value));

    kCheck(kXml_AddItem(xml, item, "SpotContinuitySorting", &tempItem));
    kCheck(kXml_SetChild32u(xml, tempItem, "MinimumSegmentSize", obj->spotContinuitySortingMinimumSegmentSize));
    kCheck(kXml_SetChild32u(xml, tempItem, "SearchWindow/X", obj->spotContinuitySortingSearchWindowX));
    kCheck(kXml_SetChild32u(xml, tempItem, "SearchWindow/Y", obj->spotContinuitySortingSearchWindowY));

    kCheck(kXml_AddItem(xml, item, "SpotTranslucentSorting", &tempItem));
    kCheck(kXml_SetChild32u(xml, tempItem, "OpaqueWidth", obj->spotTranslucentSortingOpaqueWidth.value));
    kCheck(kXml_SetChild32u(xml, tempItem, "TranslucentWidth", obj->spotTranslucentSortingTranslucentWidth.value));
    kCheck(kXml_SetChild32u(xml, tempItem, "MinLength", obj->spotTranslucentSortingMinLength.value));
    kCheck(kXml_SetChild32s(xml, tempItem, "ThreadingMode", obj->spotTranslucentSortingThreadingMode.value));

    kCheck(kXml_SetChild64f(xml, item, "CameraGainAnalog", obj->cameraGainAnalog.value));
    kCheck(kXml_SetChild64f(xml, item, "CameraGainDigital", obj->cameraGainDigital.value));
    kCheck(kXml_SetChild64f(xml, item, "DynamicSensitivity", obj->dynamicSensitivity.value));
    kCheck(kXml_SetChild32u(xml, item, "DynamicThreshold", obj->dynamicThreshold.value));
    kCheck(kXml_SetChild32u(xml, item, "GammaType", obj->gammaType.value));
    kCheck(kXml_SetChildBool(xml, item, "SensitivityCompensationEnabled", obj->sensitivityCompensationEnabled.value));

    kCheck(kXml_SetChild32u(xml, item, "SurfaceEncoding", obj->encoding.value));
    kCheck(kXml_SetChild32s(xml, item, "SurfacePhaseFilter", obj->phaseFilter));
    
    kCheck(kXml_SetChild32u(xml, item, "ContrastThreshold", obj->contrastThreshold.value));
    
    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));

    return kOK; 
}

GoFx(kStatus) GoAdvanced_SetType(GoAdvanced advanced, GoAdvancedType type)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->type = type;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAdvancedType) GoAdvanced_Type(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->type;
}

GoFx(GoAdvancedType) GoAdvanced_TypeSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->typeSystemValue;
}

GoFx(kBool) GoAdvanced_IsTypeUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->typeUsed;
}

GoFx(kStatus) GoAdvanced_SetSpotThreshold(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotThreshold.min && value <= obj->spotThreshold.max);

    obj->spotThreshold.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotThreshold(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.value;
}

GoFx(k32u) GoAdvanced_SpotThresholdLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.min;
}

GoFx(k32u) GoAdvanced_SpotThresholdLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.max;
}

GoFx(kBool) GoAdvanced_IsSpotThresholdUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.enabled;
}

GoFx(k32u) GoAdvanced_SpotThresholdSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotThreshold.systemValue;
}

GoFx(kStatus) GoAdvanced_SetSpotWidthMax(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotWidthMax.min && value <= obj->spotWidthMax.max);

    obj->spotWidthMax.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotWidthMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.value;
}

GoFx(k32u) GoAdvanced_SpotWidthMaxLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.min;
}

GoFx(k32u) GoAdvanced_SpotWidthMaxLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.max;
}

GoFx(kBool) GoAdvanced_IsSpotWidthMaxUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.enabled;
}

GoFx(k32u) GoAdvanced_SpotWidthMaxSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotWidthMax.systemValue;
}

GoFx(kStatus) GoAdvanced_SetSpotSelectionType(GoAdvanced advanced, GoSpotSelectionType type)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->spotSelectionType = type;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionType(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotSelectionType;
}

GoFx(kBool) GoAdvanced_IsSpotSelectionTypeUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotSelectionTypeUsed;
}

GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionTypeSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->systemSpotSelectionType;
}

GoFx(k32u) GoAdvanced_SpotContinuityMinimumSegmentSize(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotContinuitySortingMinimumSegmentSize;
}

GoFx(kStatus) GoAdvanced_SetSpotContinuityMinimumSegmentSize(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->spotContinuitySortingMinimumSegmentSize = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotContinuitySearchWindowX(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotContinuitySortingSearchWindowX;
}

GoFx(kStatus) GoAdvanced_SetSpotContinuitySearchWindowX(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->spotContinuitySortingSearchWindowX = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotContinuitySearchWindowY(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotContinuitySortingSearchWindowY;
}

GoFx(kStatus) GoAdvanced_SetSpotContinuitySearchWindowY(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->spotContinuitySortingSearchWindowY = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoAdvanced_SetSpotTranslucentOpaqueWidth(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotTranslucentSortingOpaqueWidth.min && value <= obj->spotTranslucentSortingOpaqueWidth.max);

    obj->spotTranslucentSortingOpaqueWidth.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidth(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingOpaqueWidth.value;
}

GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingOpaqueWidth.min;
}

GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingOpaqueWidth.max;
}

GoFx(kBool) GoAdvanced_IsSpotTranslucentOpaqueWidthUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingOpaqueWidth.enabled;
}

GoFx(k32u) GoAdvanced_SpotTranslucentOpaqueWidthSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingOpaqueWidth.systemValue;
}

GoFx(kStatus) GoAdvanced_SetSpotTranslucentTranslucentWidth(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotTranslucentSortingTranslucentWidth.min && value <= obj->spotTranslucentSortingTranslucentWidth.max);

    obj->spotTranslucentSortingTranslucentWidth.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidth(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingTranslucentWidth.value;
}

GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingTranslucentWidth.min;
}

GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingTranslucentWidth.max;
}

GoFx(kBool) GoAdvanced_IsSpotTranslucentTranslucentWidthUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingTranslucentWidth.enabled;
}

GoFx(k32u) GoAdvanced_SpotTranslucentTranslucentWidthSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingTranslucentWidth.systemValue;
}

GoFx(kStatus) GoAdvanced_SetSpotTranslucentMinimumLength(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->spotTranslucentSortingMinLength.min && value <= obj->spotTranslucentSortingMinLength.max);

    obj->spotTranslucentSortingMinLength.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLength(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingMinLength.value;
}

GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingMinLength.min;
}

GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingMinLength.max;
}

GoFx(kBool) GoAdvanced_IsSpotTranslucentMinimumLengthUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingMinLength.enabled;
}

GoFx(k32u) GoAdvanced_SpotTranslucentMinimumLengthSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingMinLength.systemValue;
}

GoFx(kSize) GoAdvanced_SpotTranslucentThreadingModeOptionCount(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->spotTranslucentSortingThreadingModeOptions);
}

GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingModeOptionAt(GoAdvanced advanced, kSize index)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoAdvanced_SpotTranslucentThreadingModeOptionCount(advanced));

    return kArrayList_AsT(obj->spotTranslucentSortingThreadingModeOptions, index, GoTranslucentThreadingMode);
}

GoFx(kStatus) GoAdvanced_SetSpotTranslucentThreadingMode(GoAdvanced advanced, GoTranslucentThreadingMode mode)
{
    kObj(GoAdvanced, advanced);

    kBool isModeValid = kFALSE;
    GoTranslucentThreadingMode* threadingMode;

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    for (kSize i = 0; i < kArrayList_Count(obj->spotTranslucentSortingThreadingModeOptions); i++)
    {
        threadingMode = kArrayList_AtT(obj->spotTranslucentSortingThreadingModeOptions, i, GoTranslucentThreadingMode);
        if (*threadingMode == mode)
        {
            isModeValid = kTRUE;
            break;
        }
    }

    kCheckArgs(isModeValid);

    obj->spotTranslucentSortingThreadingMode.value = mode;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingMode(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingThreadingMode.value;
}

GoFx(kBool) GoAdvanced_IsSpotTranslucentThreadingModeUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingThreadingMode.enabled;
}

GoFx(GoTranslucentThreadingMode) GoAdvanced_SpotTranslucentThreadingModeSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->spotTranslucentSortingThreadingMode.systemValue;
}

GoFx(kStatus) GoAdvanced_SetCameraGainAnalog(GoAdvanced advanced, k64f value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->cameraGainAnalog.min && value <= obj->cameraGainAnalog.max);

    obj->cameraGainAnalog.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoAdvanced_CameraGainAnalog(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.value;
}

GoFx(k64f) GoAdvanced_CameraGainAnalogLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.min;
}

GoFx(k64f) GoAdvanced_CameraGainAnalogLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.max;
}

GoFx(kBool) GoAdvanced_IsCameraGainAnalogUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.enabled;
}

GoFx(k64f) GoAdvanced_CameraGainAnalogSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainAnalog.systemValue;
}

GoFx(kStatus) GoAdvanced_SetCameraGainDigital(GoAdvanced advanced, k64f value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->cameraGainDigital.min && value <= obj->cameraGainDigital.max);

    obj->cameraGainDigital.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoAdvanced_CameraGainDigital(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.value;
}

GoFx(k64f) GoAdvanced_CameraGainDigitalLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.min;
}

GoFx(k64f) GoAdvanced_CameraGainDigitalLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.max;
}

GoFx(kBool) GoAdvanced_IsCameraGainDigitalUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.enabled;
}

GoFx(k64f) GoAdvanced_CameraGainDigitalSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->cameraGainDigital.systemValue;
}

GoFx(kStatus) GoAdvanced_SetDynamicSensitivity(GoAdvanced advanced, k64f value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->dynamicSensitivity.min && value <= obj->dynamicSensitivity.max);

    obj->dynamicSensitivity.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoAdvanced_DynamicSensitivity(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.value;
}

GoFx(k64f) GoAdvanced_DynamicSensitivityLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.min;
}

GoFx(k64f) GoAdvanced_DynamicSensitivityLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.max;
}

GoFx(kBool) GoAdvanced_IsDynamicSensitivityUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.enabled;
}

GoFx(k64f) GoAdvanced_DynamicSensitivitySystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicSensitivity.systemValue;
}

GoFx(kStatus) GoAdvanced_SetDynamicThreshold(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->dynamicThreshold.min && value <= obj->dynamicThreshold.max);

    obj->dynamicThreshold.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_DynamicThreshold(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.value;
}

GoFx(k32u) GoAdvanced_DynamicThresholdLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.min;
}

GoFx(k32u) GoAdvanced_DynamicThresholdLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.max;
}

GoFx(kBool) GoAdvanced_IsDynamicThresholdUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.enabled;
}

GoFx(k32u) GoAdvanced_DynamicThresholdSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->dynamicThreshold.systemValue;
}

GoFx(kStatus) GoAdvanced_SetGammaType(GoAdvanced advanced, GoGammaType value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->gammaType.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoGammaType) GoAdvanced_GammaType(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.value;
}

GoFx(kBool) GoAdvanced_IsGammaTypeUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.enabled;
}

GoFx(GoGammaType) GoAdvanced_GammaTypeSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->gammaType.systemValue;
}

GoFx(kStatus) GoAdvanced_EnableSensitivityCompensation(GoAdvanced advanced, kBool value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->sensitivityCompensationEnabled.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoAdvanced_SensitivityCompensationEnabled(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->sensitivityCompensationEnabled.value;
}

GoFx(kBool) GoAdvanced_IsSensitivityCompensationEnabledUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->sensitivityCompensationEnabled.enabled;
}

GoFx(kBool) GoAdvanced_SensitivityCompensationEnabledSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->sensitivityCompensationEnabled.systemValue;
}

GoFx(kSize) GoAdvanced_SpotSelectionTypeOptionCount(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);
    return kArrayList_Count(obj->spotSelectionTypeOptions);
}

GoFx(GoSpotSelectionType) GoAdvanced_SpotSelectionTypeOptionAt(GoAdvanced advanced, kSize index)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoAdvanced_SpotSelectionTypeOptionCount(advanced));

    return kArrayList_AsT(obj->spotSelectionTypeOptions, index, GoSpotSelectionType);
}

GoFx(kStatus) GoAdvanced_SetSurfaceEncoding(GoAdvanced advanced, GoSurfaceEncoding encoding)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->encoding.value = encoding;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfaceEncoding) GoAdvanced_SurfaceEncoding(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->encoding.value;
}

GoFx(kBool) GoAdvanced_IsSurfaceEncodingUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->encoding.enabled;
}

GoFx(GoSurfaceEncoding) GoAdvanced_SurfaceEncodingSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->encoding.systemValue;
}

GoFx(kStatus) GoAdvanced_SetSurfacePhaseFilter(GoAdvanced advanced, GoSurfacePhaseFilter phaseFilter)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->phaseFilter = phaseFilter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoSurfacePhaseFilter) GoAdvanced_SurfacePhaseFilter(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->phaseFilter;
}

GoFx(kStatus) GoAdvanced_SetContrastThreshold(GoAdvanced advanced, k32u value)
{
    kObj(GoAdvanced, advanced);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(value >= obj->contrastThreshold.min && value <= obj->contrastThreshold.max);

    obj->contrastThreshold.value = value;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoAdvanced_ContrastThreshold(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->contrastThreshold.value;
}

GoFx(k32u) GoAdvanced_ContrastThresholdLimitMin(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->contrastThreshold.min;
}

GoFx(k32u) GoAdvanced_ContrastThresholdLimitMax(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->contrastThreshold.max;
}

GoFx(kBool) GoAdvanced_IsContrastThresholdUsed(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->contrastThreshold.enabled;
}

GoFx(k32u) GoAdvanced_ContrastThresholdSystemValue(GoAdvanced advanced)
{
    kObj(GoAdvanced, advanced);

    GoSensor_SyncConfig(obj->sensor);

    return obj->contrastThreshold.systemValue;
}

