/**
 * @file    GoSetup.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/GoSetup.h>
#include <GoSdk/GoSystem.h>
#include <GoSdk/GoUtils.h>

/*
 * GoSetupNode
 */

kBeginClassEx(Go, GoSetupNode)
    kAddVMethod(GoSetupNode, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSetupNode_Construct(GoSetupNode* node, kObject setup, GoRole role, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSetupNode), node));

    if (!kSuccess(status = GoSetupNode_Init(*node, kTypeOf(GoSetupNode), setup, role, sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, node);
    }

    return status;
}

GoFx(kStatus) GoSetupNode_Init(GoSetupNode node, kType type, kObject setup, GoRole role, kObject sensor, kAlloc alloc)
{
    kObjR(GoSetupNode, node);
    kStatus exception = kOK;

    kCheck(kObject_Init(node, type, alloc));
    kZero(obj->configXml);
    kZero(obj->configXmlItem);
    kZero(obj->transformXml);
    kZero(obj->transformXmlItem);
    obj->hasConfig = kFALSE;
    obj->hasTransform = kFALSE;
    kZero(obj->activeArea);
    kZero(obj->transformedDataRegion);
    obj->gridUsed = kFALSE;
    kZero(obj->row);
    kZero(obj->column);
    kZero(obj->direction);
    kZero(obj->multiplexingBank);
    kZero(obj->frontCamera);
    kZero(obj->backCamera);
    obj->backCameraUsed = kFALSE;
    obj->exposureMode = GO_EXPOSURE_MODE_SINGLE;
    kZero(obj->exposureModeOptions);
    kZero(obj->exposure);
    obj->dynamicExposureMax = 0.0;
    obj->dynamicExposureMin = 0.0;
    kZero(obj->exposureSteps);
    obj->exposureStepCount = 0;
    obj->intensitySource = GO_INTENSITY_SOURCE_BOTH;
    obj->intensityMode = GO_INTENSITY_MODE_AUTO;
    obj->intensityModeAvail = kFALSE;
    kZero(obj->intensitySourceOptions);
    obj->intensityStepIndex = 0;
    obj->patternSequenceType = GO_PATTERN_SEQUENCE_TYPE_DEFAULT;
    kZero(obj->patternSequenceTypeOptions);
    obj->patternSequenceTypeUsed = kFALSE;
    obj->patternSequenceCount = 0;
    kZero(obj->spacingInterval);
    kZero(obj->spacingIntervalType);
    obj->xSpacingCount = 0;
    obj->ySpacingCount = 0;
    kZero(obj->zSubsamplingOptions);
    obj->zSubsamplingOptionCount = 0;
    obj->zSubsampling = 0;
    kZero(obj->xSubsamplingOptions);
    obj->xSubsamplingOptionCount = 0;
    obj->xSubsampling = 0;
    kZero(obj->tracking);
    obj->advanced = kNULL;
    obj->tracheid = kNULL;
    kZero(obj->independentExposures);

    obj->role = role;
    obj->setup = setup;
    obj->index = GO_INVALID_ROLE_INDEX;

    kTry
    {
        kTest(kArrayList_Construct(&obj->exposureModeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->intensitySourceOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->patternSequenceTypeOptions, kTypeOf(k32s), 0, alloc));
        kTest(GoAdvanced_Construct(&obj->advanced, sensor, alloc));
        kTest(GoTracheid_Construct(&obj->tracheid, sensor, alloc));
    }
    kCatch(&exception)
    {
        GoSetupNode_VRelease(node);
        kEndCatch(exception);
    }

    return kOK;
}

GoFx(kStatus) GoSetupNode_VRelease(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    kCheck(kDestroyRef(&obj->tracheid));
    kCheck(kDestroyRef(&obj->advanced));
    kCheck(kDisposeRef(&obj->intensitySourceOptions));
    kCheck(kDisposeRef(&obj->exposureModeOptions));
    kCheck(kDisposeRef(&obj->patternSequenceTypeOptions));

    return kObject_VRelease(node);
}

GoFx(k32u) GoSetupNode_Index(GoSetupNode node)
{
    kObj(GoSetupNode, node);
    return obj->index;
}

GoFx(kStatus) GoSetupNode_ReadConfig(GoSetupNode node, kXml xml, kXmlItem item)
{
    kObj(GoSetupNode, node);

    kXmlItem exposureStepItem, tempItem, layoutItem;
    k32u role;
    kText256 tempText;

    obj->configXml = xml;
    obj->configXmlItem = item;

    kCheck(kXml_Attr32u(xml, item, "role", &role));
    if(role == 0)
    {
        obj->role = GO_ROLE_MAIN;
    }
    else
    {
        obj->role = GO_ROLE_BUDDY;
    }
    //check for index
    if (kXml_AttrExists(xml, item, "index"))
    {
        kCheck(kXml_Attr32s(xml, item, "index", &obj->index));
    }
    else
    {
        //for backwwards compatibilty make index invalid
        obj->index = GO_INVALID_ROLE_INDEX;
    }
    //Active Area
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/X", &obj->activeArea.x));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Y", &obj->activeArea.y));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Z", &obj->activeArea.z));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Width", &obj->activeArea.width));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Length", &obj->activeArea.length));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "ActiveArea/Height", &obj->activeArea.height));

    //Transformed Data Region
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/X", &obj->transformedDataRegion.x));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Y", &obj->transformedDataRegion.y));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Z", &obj->transformedDataRegion.z));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Width", &obj->transformedDataRegion.width));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Length", &obj->transformedDataRegion.length));
    kCheck(kXml_Child64f(xml, item, "TransformedDataRegion/Height", &obj->transformedDataRegion.height));

    //Front Camera
    kCheck(kXml_Child32u(xml, item, "FrontCamera/X", &obj->frontCamera.x));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Y", &obj->frontCamera.y));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Width", &obj->frontCamera.width));
    kCheck(kXml_Child32u(xml, item, "FrontCamera/Height", &obj->frontCamera.height));

    //multplexing bank
    tempItem = kXml_Child(xml, item, "Layout/MultiplexingBank");
    if (!kIsNull(tempItem))
    {
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->multiplexingBank.enabled));
        }
        if (kXml_AttrExists(xml, tempItem, "value"))
        {
            kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->multiplexingBank.systemValue));
        }
        kCheck(kXml_Item32s(xml, tempItem, &obj->multiplexingBank.value));
    }
    //buddy layout
    layoutItem = kXml_Child(xml, item, "Layout/Grid");
    if (!kIsNull(layoutItem))
    {
        if (kXml_AttrExists(xml, layoutItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, layoutItem, "used", &obj->gridUsed));
        }
    }
    else
    {
        // Force to false as there is no grid used flag.
        obj->gridUsed = kFALSE;
    }

    if (obj->gridUsed)
    {
        tempItem = kXml_Child(xml, layoutItem, "Row");
        if (!kIsNull(tempItem))
        {
            kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->row.systemValue));
            kCheck(kXml_Item32s(xml, tempItem, &obj->row.value));
        }
        tempItem = kXml_Child(xml, layoutItem, "Column");
        if (!kIsNull(tempItem))
        {
            kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->column.systemValue));
            kCheck(kXml_Item32s(xml, tempItem, &obj->column.value));
        }
        tempItem = kXml_Child(xml, layoutItem, "Direction");
        if (!kIsNull(tempItem))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->direction.systemValue));
            kCheck(kXml_ItemBool(xml, tempItem, &obj->direction.value));
        }
    }

    //Back Camera
    tempItem = kXml_Child(xml, item, "BackCamera");
    if (!kIsNull(tempItem) && kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->backCameraUsed));
    }
    else
    {
        obj->backCameraUsed = kFALSE;
    }
    kCheck(kXml_Child32u(xml, item, "BackCamera/X", &obj->backCamera.x));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Y", &obj->backCamera.y));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Width", &obj->backCamera.width));
    kCheck(kXml_Child32u(xml, item, "BackCamera/Height", &obj->backCamera.height));

    tempItem = kXml_Child(xml, item, "ExposureMode");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(kArrayList_Clear(obj->exposureModeOptions));
        kCheck(GoOptionList_ParseList32u(tempText, obj->exposureModeOptions));
        kCheck(kXml_Item32u(xml, tempItem, (k32u*) &obj->exposureMode));
    }

    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Exposure", &obj->exposure));

    kCheck(kXml_Child64f(xml, item, "DynamicExposureMin", &obj->dynamicExposureMin));
    kCheck(kXml_Child64f(xml, item, "DynamicExposureMax", &obj->dynamicExposureMax));

    exposureStepItem = kXml_Child(xml, item, "ExposureSteps");
    if (!kIsNull(exposureStepItem))
    {
        kCheck(kXml_ItemText(xml, exposureStepItem, tempText, kCountOf(tempText)));
        kCheck(GoOptionList_Parse64f(tempText, obj->exposureSteps, kCountOf(obj->exposureSteps), &obj->exposureStepCount));
    }

    kCheck(kXml_ChildSize(xml, item, "IntensityStepIndex", &obj->intensityStepIndex));

    tempItem = kXml_Child(xml, item, "PatternSequenceType");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->patternSequenceTypeUsed));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->patternSequenceTypeOptions));
        kCheck(kXml_Item32s(xml, tempItem, &obj->patternSequenceType));
    }
    kCheck(GoConfig_ReadSizeOptional(xml, item, "PatternSequenceCount", 0, &obj->patternSequenceCount));

    tempItem = kXml_Child(xml, item, "PatternSequenceIndex");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->patternSequenceIndexUsed));
        kCheck(kXml_Attr32u(xml, tempItem, "min", &obj->patternSequenceIndexMin));
        kCheck(kXml_Attr32u(xml, tempItem, "max", &obj->patternSequenceIndexMax));
        kCheck(kXml_Item32u(xml, tempItem, &obj->patternSequenceIndex));
    }
    else
    {
        obj->patternSequenceIndex = 0;
        obj->patternSequenceIndexUsed = 0;
        obj->patternSequenceIndexMax = 0;
        obj->patternSequenceIndexMin = 0;
    }

    tempItem = kXml_Child(xml, item, "XSubsampling");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_Parse32u(tempText, obj->xSubsamplingOptions, (kSize) kCountOf(obj->xSubsamplingOptions), &obj->xSubsamplingOptionCount));
        kCheck(kXml_Item32u(xml, tempItem, &obj->xSubsampling));
    }

    tempItem = kXml_Child(xml, item, "ZSubsampling");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_Parse32u(tempText, obj->zSubsamplingOptions, (kSize) kCountOf(obj->zSubsamplingOptions), &obj->zSubsamplingOptionCount));
        kCheck(kXml_Item32u(xml, tempItem, &obj->zSubsampling));
    }

    tempItem = kXml_Child(xml, item, "SpacingInterval");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spacingInterval.enabled));
        kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->spacingInterval.systemValue));
        kCheck(GoConfig_ReadRangeElement64f(xml, item, "SpacingInterval", &obj->spacingInterval));
    }

    tempItem = kXml_Child(xml, item, "SpacingIntervalType");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32u(xml, tempItem, &obj->spacingIntervalType.value));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->spacingIntervalType.enabled));
    }

    kCheck(kXml_Child32u(xml, item, "XSpacingCount", &obj->xSpacingCount));
    kCheck(kXml_Child32u(xml, item, "YSpacingCount", &obj->ySpacingCount));

    tempItem = kXml_Child(xml, item, "Tracking/Enabled");
    if (!kIsNull(tempItem) && kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->tracking.used));
    }
    else
    {
        obj->tracking.used = kFALSE;
    }

    kCheck(kXml_Child32s(xml, item, "Tracking/Enabled", &obj->tracking.enabled));
    kCheck(kXml_Child64f(xml, item, "Tracking/SearchThreshold", &obj->tracking.searchThreshold));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Tracking/Height", &obj->tracking.trackingAreaHeight));

    // Material
    if (kXml_ChildExists(xml, item, "Material"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Material")));
        kCheck(GoAdvanced_Read(obj->advanced, xml, tempItem));
    }

    // Tracheid
    if (kXml_ChildExists(xml, item, "Tracheid"))
    {
        kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Tracheid")));
        kCheck(GoTracheid_Read(obj->tracheid, xml, tempItem));
    }

    // Independent Exposures
    if (kXml_ChildExists(xml, item, "IndependentExposures"))
    {
        tempItem = kXml_Child(xml, item, "IndependentExposures");
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->independentExposures.used));
        kCheck(kXml_ChildBool(xml, tempItem, "Enabled", &obj->independentExposures.enabled));
        kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "FrontCameraExposure", &obj->independentExposures.frontCameraExposure));
        kCheck(GoConfig_ReadRangeElement64f(xml, tempItem, "BackCameraExposure", &obj->independentExposures.backCameraExposure));
    }

    // Intensity Source
    tempItem = kXml_Child(xml, item, "IntensitySource");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->intensitySourceOptions));
        kCheck(kXml_Item32u(xml, tempItem, (k32u*)&obj->intensitySource));
    }

    tempItem = kXml_Child(xml, item, "IntensityMode");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, &obj->intensityMode));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->intensityModeAvail));
    }

    obj->hasConfig = kTRUE;

    return kOK;
}

GoFx(kStatus) GoSetupNode_WriteConfig(GoSetupNode node, kXml xml, kXmlItem item)
{
    kObj(GoSetupNode, node);
    kXmlItem activeAreaItem, independentExposuresItem, materialItem, tracheidItem, trackingItem, layoutItem;
    kXmlItem griditem, rowItem, columnItem, directionItem, multiplexbank;
    kXmlItem tempItem;
    kText256 tempText;

    if(obj->role == GO_ROLE_MAIN)
    {
        kCheck(kXml_SetAttr32u(xml, item, "role", GO_ROLE_MAIN));
    }
    else
    {
        kCheck(kXml_SetAttr32u(xml, item, "role", GO_ROLE_BUDDY));
    }
    //Never update attributes/system values such as 'value', 'used', 'min', 'max', etc. Those are read-only

    kCheck(kXml_AddItem(xml, item, "Layout", &layoutItem));

    kCheck(kXml_AddItem(xml, layoutItem, "Grid", &griditem));
    kCheck(kXml_SetAttrBool(xml, griditem, "used", obj->gridUsed));

    kCheck(kXml_AddItem(xml, griditem, "Row", &rowItem));
    kCheck(kXml_SetItem32s(xml, rowItem, obj->row.value));
    kCheck(kXml_AddItem(xml, griditem, "Column", &columnItem));
    kCheck(kXml_SetItem32s(xml, columnItem, obj->column.value));
    kCheck(kXml_AddItem(xml, griditem, "Direction", &directionItem));
    kCheck(kXml_SetItemBool(xml, directionItem, obj->direction.value));

    kCheck(kXml_AddItem(xml, layoutItem, "MultiplexingBank", &multiplexbank));
    kCheck(kXml_SetItem32s(xml, multiplexbank, obj->multiplexingBank.value));

    kCheck(kXml_AddItem(xml, item, "ActiveArea", &activeAreaItem));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "X", obj->activeArea.x));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Y", obj->activeArea.y));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Z", obj->activeArea.z));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Width", obj->activeArea.width));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Length", obj->activeArea.length));
    kCheck(GoConfig_WriteRangeElement64f(xml, activeAreaItem, "Height", obj->activeArea.height));

    kCheck(kXml_SetChild32s(xml, item, "ExposureMode", obj->exposureMode));
    kCheck(GoConfig_WriteRangeElement64f(xml, item, "Exposure", obj->exposure));
    kCheck(kXml_SetChild64f(xml, item, "DynamicExposureMin", obj->dynamicExposureMin));
    kCheck(kXml_SetChild64f(xml, item, "DynamicExposureMax", obj->dynamicExposureMax));

    kCheck(GoOptionList_Format64f(obj->exposureSteps, obj->exposureStepCount, tempText, kCountOf(tempText)));
    kCheck(kXml_SetChildText(xml, item, "ExposureSteps", tempText));

    kCheck(kXml_SetChildSize(xml, item, "IntensityStepIndex", obj->intensityStepIndex));

    kCheck(kXml_SetChild32s(xml, item, "PatternSequenceType", obj->patternSequenceType));
    kCheck(kXml_SetChild32u(xml, item, "PatternSequenceIndex", obj->patternSequenceIndex));

    kCheck(kXml_SetChild32u(xml, item, "XSubsampling", obj->xSubsampling));
    kCheck(kXml_SetChild32u(xml, item, "ZSubsampling", obj->zSubsampling));

    kCheck(kXml_SetChild64f(xml, item, "SpacingInterval", obj->spacingInterval.value));
    kCheck(kXml_SetChild32u(xml, item, "SpacingIntervalType", obj->spacingIntervalType.value));

    kCheck(kXml_AddItem(xml, item, "Tracking", &tempItem));
    kCheck(kXml_SetChild32u(xml, tempItem, "Enabled", obj->tracking.enabled));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "Height", obj->tracking.trackingAreaHeight));
    kCheck(kXml_SetChild64f(xml, tempItem, "SearchThreshold", obj->tracking.searchThreshold));
    trackingItem = kXml_Child(xml, item, "Tracking");

    kCheck(kXml_AddItem(xml, item, "Material", &tempItem));
    kCheck(GoAdvanced_Write(obj->advanced, xml, tempItem));
    materialItem = kXml_Child(xml, item, "Material");

    kCheck(kXml_AddItem(xml, item, "Tracheid", &tempItem));
    kCheck(GoTracheid_Write(obj->tracheid, xml, tempItem));
    tracheidItem = kXml_Child(xml, item, "Tracheid");

    kCheck(kXml_AddItem(xml, item, "IndependentExposures", &tempItem));
    kCheck(kXml_SetChildBool(xml, tempItem, "Enabled", obj->independentExposures.enabled));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "FrontCameraExposure", obj->independentExposures.frontCameraExposure));
    kCheck(GoConfig_WriteRangeElement64f(xml, tempItem, "BackCameraExposure", obj->independentExposures.backCameraExposure));
    independentExposuresItem = kXml_Child(xml, item, "IndependentExposures");

    kCheck(kXml_SetChild32s(xml, item, "IntensitySource", obj->intensitySource));
    kCheck(kXml_SetChild32s(xml, item, "IntensityMode", obj->intensityMode));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "ActiveArea"), xml, activeAreaItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Tracking"), xml, trackingItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Material"), xml, materialItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Tracheid"), xml, tracheidItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "IndependentExposures"), xml, independentExposuresItem));
    kCheck(GoUtils_XmlMerge(obj->configXml, kXml_Child(obj->configXml, obj->configXmlItem, "Device"), xml, item));
    kCheck(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, item));

    return kOK;
}

GoFx(kStatus) GoSetupNode_ClearConfig(GoSetupNode node)
{
    kObj(GoSetupNode, node);
    obj->hasConfig = kFALSE;
    obj->configXml = kNULL;
    obj->configXmlItem = kNULL;
    return kOK;
}

GoFx(k32s) GoSetupNode_Role(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->role;
}

GoFx(kBool) GoSetupNode_LayoutGridUsed(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->gridUsed;
}

GoFx(kBool) GoSetupNode_LayoutGridDirectionSystemValue(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->direction.systemValue;
}

GoFx(kBool) GoSetupNode_LayoutGridDirection(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->direction.value;
}

GoFx(kStatus) GoSetupNode_SetLayoutGridDirection(GoSetupNode node, kBool value)
{
    kObj(GoSetupNode, node);

    obj->direction.value = value;

    return kOK;
}

GoFx(k32s) GoSetupNode_LayoutGridColumnSystemValue(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->column.systemValue;
}

GoFx(k32s) GoSetupNode_LayoutGridColumn(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->column.value;
}

GoFx(kStatus) GoSetupNode_SetLayoutGridColumn(GoSetupNode node, k32s value)
{
    kObj(GoSetupNode, node);

    obj->column.value = value;

    return kOK;
}

GoFx(k32s) GoSetupNode_LayoutGridRowSystemValue(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->row.systemValue;
}

GoFx(k32s) GoSetupNode_LayoutGridRow(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->row.value;
}

GoFx(kStatus) GoSetupNode_SetLayoutGridRow(GoSetupNode node, k32s value)
{
    kObj(GoSetupNode, node);

    obj->row.value = value;

    return kOK;
}

GoFx(kBool) GoSetupNode_LayoutMultiplexingBankUsed(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->multiplexingBank.enabled;
}

GoFx(k32u) GoSetupNode_LayoutMultiplexingBankSystemValue(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->multiplexingBank.systemValue;
}

GoFx(k32u) GoSetupNode_LayoutMultiplexingBank(GoSetupNode node)
{
    kObj(GoSetupNode, node);

    return obj->multiplexingBank.value;
}

GoFx(kStatus) GoSetupNode_SetLayoutMultiplexingBank(GoSetupNode node, k32u value)
{
    kObj(GoSetupNode, node);

    obj->multiplexingBank.value = value;

    return kOK;
}

/*
 * GoSetup
 */

kBeginClassEx(Go, GoSetup)
    kAddVMethod(GoSetup, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoSetup_Construct(GoSetup* setup, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoSetup), setup));

    if (!kSuccess(status = GoSetup_Init(*setup, kTypeOf(GoSetup), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, setup);
    }

    return status;
}

GoFx(kStatus) GoSetup_Init(GoSetup setup, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoSetup, setup);
    kStatus status;
    k32s options[] = { 0, 1, 2 }; 

    kCheck(kObject_Init(setup, type, alloc));
    kZero(obj->nodes);
    kZero(obj->configXml);
    kZero(obj->configXmlItem);
    kZero(obj->transformXml);
    kZero(obj->transformXmlItem);
    obj->autoStartEnabled = kFALSE;
    obj->temperatureSafetyEnabled = kFALSE;
    obj->temperatureSafetyUsed = kFALSE;
    kZero(obj->scanModeOptions);
    obj->intensityAvail = kFALSE;
    obj->intensityEnabledSystemValue = kFALSE;
    obj->occlusionReductionEnabled = kFALSE;
    obj->occlusionReductionEnabledSystemValue = kFALSE;
    obj->occlusionReductionEnabledUsed = kFALSE;
    obj->occlusionReductionAlg = 0;
    obj->occlusionReductionAlgUsed = kFALSE;
    obj->uniformSpacingEnabled = kFALSE;
    obj->uniformSpacingEnabledSystemValue = kFALSE;
    obj->uniformSpacingAvail = kFALSE;
    obj->flickerFreeModeEnabled = kFALSE;
    obj->flickerFreeModeAvail = kFALSE;
    kZero(obj->backgroundSuppression);
    kZero(obj->trigger);
    kZero(obj->filters);
    obj->layout = kNULL;
    kZero(obj->alignment);
    obj->profileGeneration = kNULL;
    obj->surfaceGeneration = kNULL;
    obj->partDetection = kNULL;
    obj->partMatching = kNULL;
    obj->sections = kNULL;
    obj->externalInputZPulseEnabled = kFALSE;
    obj->externalInputZPulseIndex = 0;
    obj->externalInputZPulseIndexAvail = kFALSE;
    obj->preferMasterTimeEncoderEnabled = kFALSE;

    obj->sensor = sensor;

    obj->scanMode = GO_MODE_UNKNOWN;

    obj->trigger.source = GO_TRIGGER_TIME;
    obj->trigger.delay.value = 0;
    obj->trigger.delay.min = 0;
    obj->trigger.delay.max = 0;
    obj->trigger.gateEnabled = kFALSE;

    obj->trigger.maxFrameRateEnabled = kFALSE;
    obj->trigger.frameRate.value = 0;
    obj->trigger.frameRate.min = 0;
    obj->trigger.frameRate.max = 0;
    obj->trigger.tracheidRate = 0;
    obj->trigger.frameDataRate = 0;
    //Could put frameRateLimitMaxSource in here, it's in the XML

    obj->trigger.encoderTriggerMode = GO_ENCODER_TRIGGER_MODE_TRACK_REVERSE;
    obj->trigger.encoderSpacing.value = 0;
    obj->trigger.encoderSpacing.min = 0;
    obj->trigger.encoderSpacing.max = 0;
    //Could put trigger.encoderSpacing.minSource in here, it's in the XML

    obj->trigger.reversalDistanceThresholdUsed = 0;
    obj->trigger.reversalDistanceThreshold = 0;

    obj->trigger.laserSleepEnabled = 0;
    obj->trigger.laserSleepIdleTime = 0;
    obj->trigger.laserSleepWakeupEncoderTravel = 0;

    obj->intensityEnabled = kFALSE;

    obj->alignment.stationaryTarget = GO_ALIGNMENT_TARGET_NONE;
    obj->alignment.movingTarget = GO_ALIGNMENT_TARGET_DISK;
    obj->alignment.diskDiameter = 0;
    obj->alignment.diskHeight = 0;
    obj->alignment.barWidth = 0;
    obj->alignment.barHeight = 0;
    obj->alignment.barHoleCount = 0;
    obj->alignment.barHoleCountValue = 1;
    obj->alignment.barHoleCountUsed = kTRUE;
    obj->alignment.barHoleDistanceUsed = kTRUE;
    obj->alignment.barHoleDistance = 0;
    obj->alignment.barHoleDiameter = 0;
    obj->alignment.barHoleDiameterUsed = kTRUE;
    obj->alignment.barDegreesOfFreedom = GO_ALIGNMENT_3DOF_XZ_Y;
    obj->alignment.barDegreesOfFreedomUsed = kFALSE;
    kTry
    {
        kTest(kArrayList_Construct(&obj->nodes, kTypeOf(GoSetupNode), 0, alloc));
        kTest(kArrayList_Construct(&obj->scanModeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->trigger.sourceOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->trigger.externalInputIndexOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.stationaryTargetOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.movingTargetOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.typeOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.barDegreesOfFreedomOptions, kTypeOf(k32s), 0, alloc));
        kTest(kArrayList_Construct(&obj->alignment.polygonCorners, kTypeOf(GoPolygonCornerParameters), 0, alloc));
        kTest(GoLayout_Construct(&obj->layout, sensor, alloc));
        kTest(GoProfileGeneration_Construct(&obj->profileGeneration, sensor, alloc));
        kTest(GoSurfaceGeneration_Construct(&obj->surfaceGeneration, sensor, alloc));
        kTest(GoPartDetection_Construct(&obj->partDetection, sensor, alloc));
        kTest(GoPartMatching_Construct(&obj->partMatching, sensor, alloc));
        kTest(GoSections_Construct(&obj->sections, sensor, alloc));
        kTest(kArrayList_Construct(&obj->alignment.barHoleCountOptions, kTypeOf(k32s), 0, alloc));
        // GOC-14090 Populate the option list for barHoleCount with default 0,1,2 hole count.
        kTest(kArrayList_AddT(obj->alignment.barHoleCountOptions, &options[0]));
        kTest(kArrayList_AddT(obj->alignment.barHoleCountOptions, &options[1]));
        kTest(kArrayList_AddT(obj->alignment.barHoleCountOptions, &options[2]));

    }
    kCatch(&status)
    {
        GoSetup_VRelease(setup);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoSetup_VRelease(GoSetup setup)
{
    kObj(GoSetup, setup);

    kCheck(kDisposeRef(&obj->alignment.barHoleCountOptions));
    kCheck(kDestroyRef(&obj->sections));
    kCheck(kDestroyRef(&obj->partMatching));
    kCheck(kDestroyRef(&obj->partDetection));
    kCheck(kDestroyRef(&obj->profileGeneration));
    kCheck(kDestroyRef(&obj->surfaceGeneration));
    kCheck(kDestroyRef(&obj->layout));
    kCheck(kDisposeRef(&obj->trigger.sourceOptions));
    kCheck(kDisposeRef(&obj->trigger.externalInputIndexOptions));
    kCheck(kDisposeRef(&obj->alignment.stationaryTargetOptions));
    kCheck(kDisposeRef(&obj->alignment.movingTargetOptions));
    kCheck(kDisposeRef(&obj->alignment.typeOptions));
    kCheck(kDisposeRef(&obj->alignment.barDegreesOfFreedomOptions));
    kCheck(kDisposeRef(&obj->scanModeOptions));
    kCheck(kDisposeRef(&obj->nodes));

    kCheck(GoSetup_ClearPolygonCorners(setup));
    kCheck(kDisposeRef(&obj->alignment.polygonCorners));

    kCheck(kObject_VRelease(setup));

    return kOK;
}

GoFx(kStatus) GoSetup_ReadFilterConfig(kXml xml, kXmlItem item, const kChar* filterPath, GoFilter* filter)
{
    kXmlItem filterItem = kXml_Child(xml, item, filterPath);

    kCheckArgs(!kIsNull(filterItem));

    kCheck(GoConfig_ReadAttrBoolOptional(xml, filterItem, "used", kFALSE, &filter->used));
    kCheck(GoConfig_ReadBoolOptional(xml, filterItem, "Enabled", kFALSE, &filter->value.enabled));
    kCheck(GoConfig_ReadRangeElement64f(xml, filterItem, "Window", &filter->value));

    return kOK;
}

GoFx(kStatus) GoSetup_ReadConfig(GoSetup setup, kXml xml, kXmlItem item)
{
    kObj(GoSetup, setup);
    kText128 tempText;
    kXmlItem devicesItem = kNULL;
    kXmlItem deviceItem = kNULL;
    kXmlItem tempItem = kNULL;
    k32u tempInt;
    k32u tempNodeIndex;
    GoRole sensorRole;
    GoSetupNode node;
    k32u i;

    obj->configXml = xml;
    obj->configXmlItem = item;

    // System settings
    kCheck(kXml_ChildBool(xml, item, "TemperatureSafetyEnabled", &obj->temperatureSafetyEnabled));
    tempItem = kXml_Child(xml, item, "TemperatureSafetyEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->temperatureSafetyUsed));
    }

    // Mode and datatype selection
    tempItem = kXml_Child(xml, item, "ScanMode");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(kArrayList_Clear(obj->scanModeOptions));
        kCheck(GoOptionList_ParseList32u(tempText, obj->scanModeOptions));
        kCheck(kXml_Item32u(xml, tempItem, (k32u*) &obj->scanMode));
    }

    tempItem = kXml_Child(xml, item, "UniformSpacingEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ItemBool(xml, tempItem, &obj->uniformSpacingEnabled));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->uniformSpacingAvail));
        kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->uniformSpacingEnabledSystemValue));
    }

    tempItem = kXml_Child(xml, item, "IntensityEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ItemBool(xml, tempItem, &obj->intensityEnabled));
        kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "used", kFALSE, &obj->intensityAvail));
        kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "value", kFALSE, &obj->intensityEnabledSystemValue));
    }

    if (kXml_ChildExists(xml, item, "BackgroundSuppression"))
    {
        tempItem = kXml_Child(xml, item, "BackgroundSuppression");
        kCheck(kXml_ChildBool(xml, tempItem, "Enabled", &obj->backgroundSuppression.enabled));
        kCheck(kXml_Child64u(xml, tempItem, "FrameRatio", &obj->backgroundSuppression.ratio));
    }

    if (kXml_ChildExists(xml, item, "PreferMasterTimeEncoderEnabled"))
    {
        tempItem = kXml_Child(xml, item, "PreferMasterTimeEncoderEnabled");
        kCheck(kXml_ItemBool(xml, tempItem, &obj->preferMasterTimeEncoderEnabled));
    }

    if (kXml_ChildExists(xml, item, "ExternalInputZPulseEnabled"))
    {
        tempItem = kXml_Child(xml, item, "ExternalInputZPulseEnabled");
        kCheck(kXml_ItemBool(xml, tempItem, &obj->externalInputZPulseEnabled));
    }

    if (kXml_ChildExists(xml, item, "ExternalInputZPulseIndex"))
    {
        tempItem = kXml_Child(xml, item, "ExternalInputZPulseIndex");
        kCheck(kXml_Item32u(xml, tempItem, &obj->externalInputZPulseIndex));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->externalInputZPulseIndexAvail));
    }

    if (kXml_ChildExists(xml, item, "FlickerFreeModeEnabled"))
    {
        tempItem = kXml_Child(xml, item, "FlickerFreeModeEnabled");
        kCheck(kXml_ItemBool(xml, tempItem, &obj->flickerFreeModeEnabled));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->flickerFreeModeAvail));
    }

    tempItem = kXml_Child(xml, item, "OcclusionReductionEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ItemBool(xml, tempItem, &obj->occlusionReductionEnabled));

        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->occlusionReductionEnabledUsed));
        }

        if (kXml_AttrExists(xml, tempItem, "value"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "value", &obj->occlusionReductionEnabledSystemValue));
        }
    }

    if (kXml_ChildExists(xml, item, "OcclusionReductionAlg"))
    {
        tempItem = kXml_Child(xml, item, "OcclusionReductionAlg");
        kCheck(kXml_Item32s(xml, tempItem, &obj->occlusionReductionAlg));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->occlusionReductionEnabledUsed));
    }


    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/XGapFilling", &obj->filters.xGapFilling));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/YGapFilling", &obj->filters.yGapFilling));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/XMedian", &obj->filters.xMedian));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/YMedian", &obj->filters.yMedian));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/XSmoothing", &obj->filters.xSmoothing));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/YSmoothing", &obj->filters.ySmoothing));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/XDecimation", &obj->filters.xDecimation));
    kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/YDecimation", &obj->filters.yDecimation));

    if (kXml_ChildExists(xml, item, "Filters/XSlope"))
    {
        kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/XSlope", &obj->filters.xSlope));
    }

    if (kXml_ChildExists(xml, item, "Filters/YSlope"))
    {
        kCheck(GoSetup_ReadFilterConfig(xml, item, "Filters/YSlope", &obj->filters.ySlope));
    }

    //Trigger
    tempItem = kXml_Child(xml, item, "Trigger/Source");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, (k32s*) &obj->trigger.source));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32s(tempText, obj->trigger.sourceOptions));
    }

    tempItem = kXml_Child(xml, item, "Trigger/ExternalInputIndex");
    if (tempItem)
    {
        kCheck(kXml_Item32s(xml, tempItem, (k32s*)&obj->trigger.externalInputIndex));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.externalInputIndexUsed));
        kCheck(GoOptionList_ParseList32s(tempText, obj->trigger.externalInputIndexOptions));
    }

    kCheck(kXml_Child32s(xml, item, "Trigger/Units", (k32s*) &obj->trigger.unit));

    tempItem = kXml_Child(xml, item, "Trigger/FrameRate");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Attr32s(xml, tempItem, "maxSource", (k32s*) &obj->trigger.frameRateMaxSource));
        kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/FrameRate", &obj->trigger.frameRate));
    }
    kCheck(kXml_ChildBool(xml, item, "Trigger/MaxFrameRateEnabled", &obj->trigger.maxFrameRateEnabled));
    
    if (kXml_ChildExists(xml, item, "Trigger/TracheidRate"))
    {
        kCheck(kXml_Child64f(xml, item, "Trigger/TracheidRate", &obj->trigger.tracheidRate));
    }
    else
    {
        obj->trigger.tracheidRate = 0.0;
    }
    if (kXml_ChildExists(xml, item, "Trigger/FrameDataRate"))
    {
        kCheck(kXml_Child64f(xml, item, "Trigger/FrameDataRate", &obj->trigger.frameDataRate));
    }
    else
    {
        obj->trigger.frameDataRate = obj->trigger.frameRate.value;
    }

    tempItem = kXml_Child(xml, item, "Trigger/EncoderSpacing");
    if (!kIsNull(tempItem))
    {
        kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/EncoderSpacing", &obj->trigger.encoderSpacing));
        kCheck(kXml_Attr32s(xml, tempItem, "minSource", (k32s*) &obj->trigger.encoderSpacingMinSource));
    }

    kCheck(kXml_Child32s(xml, item, "Trigger/EncoderTriggerMode", (k32s*) &obj->trigger.encoderTriggerMode));
    kCheck(GoConfig_ReadRangeElement64f(xml, item, "Trigger/Delay", &obj->trigger.delay));
    tempItem = kXml_Child(xml, item, "Trigger/GateEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, &obj->trigger.gateEnabled));
        kCheck(kXml_Attr32s(xml, tempItem, "used", &obj->trigger.gateEnabledUsed));
        kCheck(kXml_Attr32s(xml, tempItem, "value", &obj->trigger.gateEnabledSystemValue));
    }

    if (kXml_ChildExists(xml, item, "Trigger/BurstEnabled"))
    {
        tempItem = kXml_Child(xml, item, "Trigger/BurstEnabled");
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.burstEnabledUsed));
            if (obj->trigger.burstEnabledUsed == kTRUE)
            {
                kCheck(kXml_Item32s(xml, tempItem, &obj->trigger.burstEnabled));
            }
        }
    }

    if (kXml_ChildExists(xml, item, "Trigger/BurstCount"))
    {
        tempItem = kXml_Child(xml, item, "Trigger/BurstCount");
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.burstCountUsed));
            if (obj->trigger.burstCountUsed == kTRUE)
            {
                kCheck(kXml_Item32u(xml, tempItem, &obj->trigger.burstCount));
            }
        }
    }

    if (kXml_ChildExists(xml, item, "Trigger/ReversalDistanceAutoEnabled"))
    {
        tempItem = kXml_Child(xml, item, "Trigger/ReversalDistanceAutoEnabled");
        kCheck(kXml_ItemBool(xml, tempItem, &obj->trigger.reversalDistanceAutoEnabled));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.reversalDistanceAutoEnabledUsed));
    }

    if (kXml_ChildExists(xml, item, "Trigger/ReversalDistance"))
    {
        kCheck(kXml_Child64f(xml, item, "Trigger/ReversalDistance", &obj->trigger.reversalDistanceThreshold));
        tempItem = kXml_Child(xml, item, "Trigger/ReversalDistance");
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.reversalDistanceThresholdUsed));
        kCheck(kXml_Attr64f(xml, tempItem, "value", &obj->trigger.reversalDistanceThresholdActual));
    }

    if (kXml_ChildExists(xml, item, "Trigger/LaserSleepMode"))
    {
        tempItem = kXml_Child(xml, item, "Trigger/LaserSleepMode");
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->trigger.laserSleepUsed));
        }
        kCheck(kXml_ChildBool(xml, tempItem, "Enabled", &obj->trigger.laserSleepEnabled));
        kCheck(kXml_Child64u(xml, tempItem, "IdleTime", &obj->trigger.laserSleepIdleTime));
        kCheck(kXml_Child64u(xml, tempItem, "WakeupEncoderTravel", &obj->trigger.laserSleepWakeupEncoderTravel));
    }

    //Layout
    kCheckArgs(!kIsNull(tempItem = kXml_Child(xml, item, "Layout")));
    kCheck(GoLayout_Read(obj->layout, xml, tempItem));

    //Alignment
    tempItem = kXml_Child(xml, item, "Alignment");
    if (!kIsNull(tempItem) && kXml_AttrExists(xml, tempItem, "used"))
    {
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->alignment.used));
    }

    tempItem = kXml_Child(xml, item, "Alignment/InputTriggerEnabled");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ItemBool(xml, tempItem, &obj->alignment.inputTriggerEnabled));
        kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "used", kFALSE, &obj->alignment.inputTriggerEnabledUsed));
        kCheck(GoConfig_ReadAttrBoolOptional(xml, tempItem, "value", kFALSE, &obj->alignment.inputTriggerEnabledSystemValue));
    }

    tempItem = kXml_Child(xml, item, "Alignment/Type");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, (k32s*) &obj->alignment.type));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.typeOptions));
    }

    tempItem = kXml_Child(xml, item, "Alignment/StationaryTarget");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, (k32s*) &obj->alignment.stationaryTarget));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.stationaryTargetOptions));
    }

    tempItem = kXml_Child(xml, item, "Alignment/MovingTarget");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item32s(xml, tempItem, (k32s*) &obj->alignment.movingTarget));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.movingTargetOptions));
    }

    kCheck(kXml_ChildBool(xml, item, "Alignment/EncoderCalibrateEnabled", &obj->alignment.encoderCalibrateEnabled));

    kCheck(kXml_Child64f(xml, item, "Alignment/Disk/Diameter", &obj->alignment.diskDiameter));
    kCheck(kXml_Child64f(xml, item, "Alignment/Disk/Height", &obj->alignment.diskHeight));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/Width", &obj->alignment.barWidth));
    kCheck(kXml_Child64f(xml, item, "Alignment/Bar/Height", &obj->alignment.barHeight));

    tempItem = kXml_Child(xml, item, "Alignment/Bar/HoleCount");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_ItemSize(xml, tempItem, &obj->alignment.barHoleCount));
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrSize(xml, tempItem, "value", &obj->alignment.barHoleCountValue));
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->alignment.barHoleCountUsed));
            // GOC-14090 Populate the option list for barHoleCount, from config (SDK access).
            // New config format is  <HoleCount used="1" value="1" options="0,1,N">1</HoleCount>.
            if (kXml_AttrExists(xml, tempItem, "options"))
            {
                kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
                kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.barHoleCountOptions));
            }
        }
    }

    tempItem = kXml_Child(xml, item, "Alignment/Bar/HoleDistance");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item64f(xml, tempItem, &obj->alignment.barHoleDistance));
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->alignment.barHoleDistanceUsed));
        }
    }
    tempItem = kXml_Child(xml, item, "Alignment/Bar/HoleDiameter");
    if (!kIsNull(tempItem))
    {
        kCheck(kXml_Item64f(xml, tempItem, &obj->alignment.barHoleDiameter));
        if (kXml_AttrExists(xml, tempItem, "used"))
        {
            kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->alignment.barHoleDiameterUsed));
        }
    }

    if (kXml_ChildExists(xml, item, "Alignment/Bar/DegreesOfFreedom"))
    {
        tempItem = kXml_Child(xml, item, "Alignment/Bar/DegreesOfFreedom");
        kCheck(kXml_Item32s(xml, tempItem, &obj->alignment.barDegreesOfFreedom));
        kCheck(kXml_AttrBool(xml, tempItem, "used", &obj->alignment.barDegreesOfFreedomUsed));
        kCheck(kXml_AttrText(xml, tempItem, "options", tempText, kCountOf(tempText)));
        kCheck(GoOptionList_ParseList32u(tempText, obj->alignment.barDegreesOfFreedomOptions));
    }

    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/Height", &obj->alignment.plateHeight));
    kCheck(kXml_ChildSize(xml, item, "Alignment/Plate/HoleCount", &obj->alignment.plateHoleCount));
    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/RefHoleDiameter", &obj->alignment.plateRefHoleDiameter));
    kCheck(kXml_Child64f(xml, item, "Alignment/Plate/SecHoleDiameter", &obj->alignment.plateSecHoleDiameter));

    if (kXml_ChildExists(xml, item, "Polygon"))
    {
        kXmlItem parentItem = kNULL;
        kSize minCount = 0;

        // This child item already checked to exist.
        tempItem = kXml_Child(xml, item, "Polygon");

        parentItem = kXml_Child(xml, tempItem, "Corners");
        if (!kIsNull(parentItem))
        {
            kCheck(kXml_AttrSize(xml, parentItem, "minCount", &minCount));

            GoSetup_ClearPolygonCorners(setup);

            for (item = kXml_FirstChild(xml, parentItem); item != kNULL; item = kXml_NextSibling(xml, item))
            {
                GoPolygonCornerParameters cornerParams;
                kXmlItem parentItem2 = kNULL;
                kText32 tempText;

                kCheck(kXml_Child64f(xml, item, "X", &cornerParams.point.x));
                kCheck(kXml_Child64f(xml, item, "Z", &cornerParams.point.y));

                parentItem2 = kXml_Child(xml, item, "Devices");
                if (!kIsNull(parentItem2))
                {
                    kCheck(kArrayList_Construct(&cornerParams.deviceIdxs, kTypeOf(k32u), 0, kObject_Alloc(setup)));

                    kCheck(kXml_ItemText(xml, parentItem2, tempText, kCountOf(tempText)));
                    kCheck(GoOptionList_ParseList32u(tempText, cornerParams.deviceIdxs));

                    kCheck(kArrayList_AddT(obj->alignment.polygonCorners, &cornerParams));
                }
            }
        }
    }

    for (i = 0; i < kArrayList_Count(obj->nodes); ++i)
    {
        kCheck(GoSetupNode_ClearConfig(kArrayList_AsT(obj->nodes, i, GoSetupNode)));
    }

    kCheck(!kIsNull(devicesItem = kXml_Child(xml, item, "Devices")));
    deviceItem = kXml_FirstChild(xml, devicesItem);

    while (!kIsNull(deviceItem))
    {
        kCheck(kXml_Attr32u(xml, deviceItem, "role", &tempInt));

        if(tempInt == 0)
        {
            sensorRole = GO_ROLE_MAIN;
        }
        else
        {
            sensorRole = GO_ROLE_BUDDY;
        }

        if (!kSuccess(kXml_Attr32u(xml, deviceItem, "index", &tempNodeIndex)))
        {
            // Old firmware does not have index attribute, use role instead.
            tempNodeIndex = (sensorRole == GO_ROLE_MAIN) ? 0 : 1;
        }
        node = GoSetup_FindNode(setup, tempNodeIndex);

        if (kIsNull(node))
        {
            kCheck(GoSetupNode_Construct(&node, setup, sensorRole, obj->sensor, kObject_Alloc(setup)));
            kCheck(kArrayList_AddT(obj->nodes, &node));
        }

        kCheck(GoSetupNode_ReadConfig(node, xml, deviceItem));

        deviceItem = kXml_NextSibling(xml, deviceItem);
    }

    if (kXml_ChildExists(xml, item, "ProfileGeneration"))
    {
        kCheck(GoProfileGeneration_Read(obj->profileGeneration, xml, kXml_Child(xml, item, "ProfileGeneration")));
    }

    if (kXml_ChildExists(xml, item, "SurfaceGeneration"))
    {
        kCheck(GoSurfaceGeneration_Read(obj->surfaceGeneration, xml, kXml_Child(xml, item, "SurfaceGeneration")));
    }

    if (kXml_ChildExists(xml, item, "PartDetection"))
    {
        kCheck(GoPartDetection_Read(obj->partDetection, xml, kXml_Child(xml, item, "PartDetection")));
    }

    if (kXml_ChildExists(xml, item, "PartMatching"))
    {
        kCheck(GoPartMatching_Read(obj->partMatching, xml, kXml_Child(xml, item, "PartMatching")));
    }

    if (kXml_ChildExists(xml, item, "SurfaceSections"))
    {
        kCheck(GoSections_Read(obj->sections, xml, kXml_Child(xml, item, "SurfaceSections")));
    }

    return kOK;
}

GoFx(kStatus) GoSetup_WriteConfig(GoSetup setup, kXml xml, kXmlItem item)
{
    kObj(GoSetup, setup);
    kXmlItem setupItem = kNULL;
    kXmlItem devicesItem = kNULL;
    kXmlItem deviceItem = kNULL;
    kXmlItem alignmentItem = kNULL;
    kXmlItem alignmentBarItem = kNULL;
    kXmlItem triggerItem = kNULL;
    kXmlItem tempItem = kNULL;
    GoSetupNode node = kNULL;
    k32u i;
    kText256 text;

    kCheck(kXml_SetChildBool(xml, item, "TemperatureSafetyEnabled", obj->temperatureSafetyEnabled));

    kCheck(kXml_SetChild32u(xml, item, "ScanMode", obj->scanMode));
    kCheck(kXml_SetChildBool(xml, item, "UniformSpacingEnabled", obj->uniformSpacingEnabled));
    kCheck(kXml_SetChildBool(xml, item, "IntensityEnabled", obj->intensityEnabled));
    kCheck(kXml_SetChildBool(xml, item, "FlickerFreeModeEnabled", obj->flickerFreeModeEnabled));

    kCheck(kXml_SetChildBool(xml, item, "OcclusionReductionEnabled", obj->occlusionReductionEnabled));
    kCheck(kXml_SetChildBool(xml, item, "OcclusionReductionAlg", obj->occlusionReductionAlg));
    kCheck(kXml_SetChildBool(xml, item, "ExternalInputZPulseEnabled", obj->externalInputZPulseEnabled));
    kCheck(kXml_SetChild32u(xml, item, "ExternalInputZPulseIndex", obj->externalInputZPulseIndex));
    kCheck(kXml_SetChildBool(xml, item, "PreferMasterTimeEncoderEnabled", obj->preferMasterTimeEncoderEnabled));
    kCheck(kXml_SetChildBool(xml, item, "BackgroundSuppression/Enabled", obj->backgroundSuppression.enabled));
    kCheck(kXml_SetChild64u(xml, item, "BackgroundSuppression/FrameRatio", obj->backgroundSuppression.ratio));

    kCheck(kXml_SetChild32s(xml, item, "Filters/XGapFilling/Enabled", obj->filters.xGapFilling.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XGapFilling/Window", obj->filters.xGapFilling.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YGapFilling/Enabled", obj->filters.yGapFilling.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YGapFilling/Window", obj->filters.yGapFilling.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XMedian/Enabled", obj->filters.xMedian.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XMedian/Window", obj->filters.xMedian.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YMedian/Enabled", obj->filters.yMedian.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YMedian/Window", obj->filters.yMedian.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XSmoothing/Enabled", obj->filters.xSmoothing.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XSmoothing/Window", obj->filters.xSmoothing.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YSmoothing/Enabled", obj->filters.ySmoothing.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YSmoothing/Window", obj->filters.ySmoothing.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XDecimation/Enabled", obj->filters.xDecimation.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XDecimation/Window", obj->filters.xDecimation.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YDecimation/Enabled", obj->filters.yDecimation.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YDecimation/Window", obj->filters.yDecimation.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/XSlope/Enabled", obj->filters.xSlope.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/XSlope/Window", obj->filters.xSlope.value.value));
    kCheck(kXml_SetChild32s(xml, item, "Filters/YSlope/Enabled", obj->filters.ySlope.value.enabled));
    kCheck(kXml_SetChild64f(xml, item, "Filters/YSlope/Window", obj->filters.ySlope.value.value));

    kCheck(kXml_AddItem(xml, item, "Trigger", &triggerItem));

    kCheck(kXml_AddItem(xml, triggerItem, "Source", &tempItem));
    kCheck(GoOptionList_Format32s(kArrayList_DataT(obj->trigger.sourceOptions, k32s), kArrayList_Count(obj->trigger.sourceOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->trigger.source));

    kCheck(kXml_AddItem(xml, triggerItem, "ExternalInputIndex", &tempItem));
    kCheck(GoOptionList_Format32s(kArrayList_DataT(obj->trigger.externalInputIndexOptions, k32s), kArrayList_Count(obj->trigger.externalInputIndexOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->trigger.externalInputIndex));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->trigger.externalInputIndexUsed));

    kCheck(kXml_SetChild32u(xml, item, "Trigger/Units", obj->trigger.unit));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/FrameRate", obj->trigger.frameRate.value));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/MaxFrameRateEnabled", obj->trigger.maxFrameRateEnabled));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/EncoderSpacing", obj->trigger.encoderSpacing.value));
    kCheck(kXml_SetChild32s(xml, item, "Trigger/EncoderTriggerMode", obj->trigger.encoderTriggerMode));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/Delay", obj->trigger.delay.value));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/GateEnabled", obj->trigger.gateEnabled));

    kCheck(kXml_AddItem(xml, triggerItem, "BurstEnabled", &tempItem));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->trigger.burstEnabledUsed));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/BurstEnabled", obj->trigger.burstEnabled));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/BurstCount", obj->trigger.burstCount));

    kCheck(kXml_SetChildBool(xml, item, "Trigger/ReversalDistanceAutoEnabled", obj->trigger.reversalDistanceAutoEnabled));
    kCheck(kXml_SetChild64f(xml, item, "Trigger/ReversalDistance", obj->trigger.reversalDistanceThreshold));

    kCheck(kXml_AddItem(xml, triggerItem, "LaserSleepMode", &tempItem));
    kCheck(kXml_SetChildBool(xml, item, "Trigger/LaserSleepMode/Enabled", obj->trigger.laserSleepEnabled));
    kCheck(kXml_SetChild64u(xml, item, "Trigger/LaserSleepMode/IdleTime", obj->trigger.laserSleepIdleTime));
    kCheck(kXml_SetChild64u(xml, item, "Trigger/LaserSleepMode/WakeupEncoderTravel", obj->trigger.laserSleepWakeupEncoderTravel));

    //tranformed data region read only?
    kCheck(kXml_AddItem(xml, item, "Layout", &tempItem));
    kCheck(GoLayout_Write(obj->layout, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "Alignment", &alignmentItem));
    kCheck(kXml_SetChildBool(xml, alignmentItem, "InputTriggerEnabled", obj->alignment.inputTriggerEnabled));

    kCheck(kXml_AddItem(xml, alignmentItem, "Type", &tempItem));
    kCheck(GoOptionList_Format32u(kArrayList_DataT(obj->alignment.typeOptions, k32u), kArrayList_Count(obj->alignment.typeOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.type));

    kCheck(kXml_AddItem(xml, alignmentItem, "StationaryTarget", &tempItem));
    kCheck(GoOptionList_Format32u(kArrayList_DataT(obj->alignment.stationaryTargetOptions, k32u), kArrayList_Count(obj->alignment.stationaryTargetOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.stationaryTarget));

    kCheck(kXml_AddItem(xml, alignmentItem, "MovingTarget", &tempItem));
    kCheck(GoOptionList_Format32u(kArrayList_DataT(obj->alignment.movingTargetOptions, k32u), kArrayList_Count(obj->alignment.movingTargetOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.movingTarget));

    kCheck(kXml_SetChildBool(xml, alignmentItem, "EncoderCalibrateEnabled", obj->alignment.encoderCalibrateEnabled));

    kCheck(kXml_SetChild64f(xml, alignmentItem, "Disk/Diameter", obj->alignment.diskDiameter));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Disk/Height", obj->alignment.diskHeight));

    kCheck(kXml_AddItem(xml, alignmentItem, "Bar", &alignmentBarItem));
    kCheck(kXml_SetChild64f(xml, alignmentBarItem, "Width", obj->alignment.barWidth));
    kCheck(kXml_SetChild64f(xml, alignmentBarItem, "Height", obj->alignment.barHeight));

    kCheck(kXml_AddItem(xml, alignmentBarItem, "HoleCount", &tempItem));
    kCheck(kXml_SetItemSize(xml, tempItem, obj->alignment.barHoleCount));
    kCheck(kXml_SetAttrSize(xml, tempItem, "value", obj->alignment.barHoleCountValue));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->alignment.barHoleCountUsed));
    // GOC-14090 Write the option list for barHoleCount to the config (SDK access).
    kCheck(GoOptionList_Format32u(kArrayList_DataT(obj->alignment.barHoleCountOptions, k32u), kArrayList_Count(obj->alignment.barHoleCountOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));
    kCheck(kXml_AddItem(xml, alignmentBarItem, "HoleDistance", &tempItem));
    kCheck(kXml_SetItem64f(xml, tempItem, obj->alignment.barHoleDistance));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->alignment.barHoleDistanceUsed));

    kCheck(kXml_AddItem(xml, alignmentBarItem, "HoleDiameter", &tempItem));
    kCheck(kXml_SetItem64f(xml, tempItem, obj->alignment.barHoleDiameter));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->alignment.barHoleDiameterUsed));

    kCheck(kXml_AddItem(xml, alignmentBarItem, "DegreesOfFreedom", &tempItem));
    kCheck(kXml_SetItem32s(xml, tempItem, obj->alignment.barDegreesOfFreedom));
    kCheck(kXml_SetAttrBool(xml, tempItem, "used", obj->alignment.barDegreesOfFreedomUsed));
    kCheck(GoOptionList_Format32u(kArrayList_DataT(obj->alignment.barDegreesOfFreedomOptions, k32u),
            kArrayList_Count(obj->alignment.barDegreesOfFreedomOptions), text, kCountOf(text)));
    kCheck(kXml_SetAttrText(xml, tempItem, "options", text));

    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/Height", obj->alignment.plateHeight));
    kCheck(kXml_SetChildSize(xml, alignmentItem, "Plate/HoleCount", obj->alignment.plateHoleCount));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/RefHoleDiameter", obj->alignment.plateRefHoleDiameter));
    kCheck(kXml_SetChild64f(xml, alignmentItem, "Plate/SecHoleDiameter", obj->alignment.plateSecHoleDiameter));

    kCheck(kXml_AddItem(xml, alignmentItem, "Polygon", &tempItem));
    kCheck(kXml_AddItem(xml, tempItem, "Corners", &tempItem));
    kCheck(kXml_SetAttrSize(xml, tempItem, "minCount", GO_ALIGNMENT_CORNERS_MINIMUM));

    for (i = 0; i < kArrayList_Count(obj->alignment.polygonCorners); i++)
    {
        GoPolygonCornerParameters *cornerParams;
        kXmlItem tempItem2 = kNULL;
        kXmlItem tempItem3 = kNULL;

        kCheck(kXml_AddItem(xml, tempItem, "Corner", &tempItem2));
        cornerParams = kArrayList_AtT(obj->alignment.polygonCorners, i, GoPolygonCornerParameters);
        kCheck(kXml_SetChild64f(xml, tempItem2, "X", cornerParams->point.x));
        kCheck(kXml_SetChild64f(xml, tempItem2, "Z", cornerParams->point.y));

        kCheck(GoOptionList_Format32u(kArrayList_DataT(cornerParams->deviceIdxs, k32u),
            kArrayList_Count(cornerParams->deviceIdxs), text, kCountOf(text)));
        kCheck(kXml_SetChildText(xml, tempItem2, "Devices", text));
    }

    kCheck(kXml_AddItem(xml, item, "Devices", &devicesItem));

    for (i = 0; i < kArrayList_Count(obj->nodes); i++)
    {
        node = kArrayList_AsT(obj->nodes, i, GoSetupNode);

        kCheck(kXml_AddItem(xml, devicesItem, "Device", &deviceItem));
        kCheck(kXml_SetAttr32u(xml, deviceItem, "index", i));

        kCheck(GoSetupNode_WriteConfig(node, xml, deviceItem));
    }

    kCheck(kXml_AddItem(xml, item, "ProfileGeneration", &tempItem));
    kCheck(GoProfileGeneration_Write(obj->profileGeneration, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "SurfaceGeneration", &tempItem));
    kCheck(GoSurfaceGeneration_Write(obj->surfaceGeneration, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "PartDetection", &tempItem));
    kCheck(GoPartDetection_Write(obj->partDetection, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "PartMatching", &tempItem));
    kCheck(GoPartMatching_Write(obj->partMatching, xml, tempItem));

    kCheck(kXml_AddItem(xml, item, "SurfaceSections", &tempItem));
    kCheck(GoSections_Write(obj->sections, xml, tempItem));

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->configXml, obj->configXmlItem, xml, item));

    return kOK;
}

GoFx(GoSetupNode) GoSetup_FindNode(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);
    GoSetupNode* node;
    k32u i;

    for (i = 0; i < kArrayList_Count(obj->nodes); ++i)
    {
        k32s nodeIdx;
        node = kArrayList_AtT(obj->nodes, i, GoSetupNode);
        nodeIdx = GoSetupNode_Index(*node);

        if (nodeIdx != GO_INVALID_ROLE_INDEX)
        {
            //has index check against role
            if (nodeIdx == role)
            {
                return *node;
            }
        }
        else
        {
            //for legacy
            if (GoSetupNode_Role(*node) == role)
            {
                return *node;
            }
        }
    }

    return kNULL;
}

GoFx(kBool) GoSetup_LayoutGridUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridUsed(GoSetup_FindNode(setup, role));
}

GoFx(kBool) GoSetup_LayoutGridDirectionSystemValue(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridDirectionSystemValue(GoSetup_FindNode(setup, role));
}

GoFx(kBool) GoSetup_LayoutGridDirection(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridDirection(GoSetup_FindNode(setup, role));
}

GoFx(kStatus) GoSetup_SetLayoutGridDirection(GoSetup setup, GoRole role, kBool value)
{
    kObj(GoSetup, setup);

    kCheck(GoSetupNode_SetLayoutGridDirection(GoSetup_FindNode(setup, role), value));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoSetup_LayoutGridColumnSystemValue(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridColumnSystemValue(GoSetup_FindNode(setup, role));
}

GoFx(k32s) GoSetup_LayoutGridColumn(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridColumn(GoSetup_FindNode(setup, role));
}

GoFx(kStatus) GoSetup_SetLayoutGridColumn(GoSetup setup, GoRole role, k32s value)
{
    kObj(GoSetup, setup);

    kCheck(GoSetupNode_SetLayoutGridColumn(GoSetup_FindNode(setup, role), value));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoSetup_LayoutGridRowSystemValue(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridRowSystemValue(GoSetup_FindNode(setup, role));
}

GoFx(k32s) GoSetup_LayoutGridRow(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutGridRow(GoSetup_FindNode(setup, role));
}

GoFx(kStatus) GoSetup_SetLayoutGridRow(GoSetup setup, GoRole role, k32s value)
{
    kObj(GoSetup, setup);

    kCheck(GoSetupNode_SetLayoutGridRow(GoSetup_FindNode(setup, role), value));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_LayoutMultiplexingBankUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutMultiplexingBankUsed(GoSetup_FindNode(setup, role));
}

GoFx(k32u) GoSetup_LayoutMultiplexingBankSystemValue(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutMultiplexingBankSystemValue(GoSetup_FindNode(setup, role));
}

GoFx(k32u) GoSetup_LayoutMultiplexingBank(GoSetup setup, GoRole role)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return GoSetupNode_LayoutMultiplexingBank(GoSetup_FindNode(setup, role));
}

GoFx(kStatus) GoSetup_SetLayoutMultiplexingBank(GoSetup setup, GoRole role, k32u value)
{
    kObj(GoSetup, setup);

    kCheck(GoSetupNode_SetLayoutMultiplexingBank(GoSetup_FindNode(setup, role), value));

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_EnableTemperatureSafety( GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->temperatureSafetyEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_TemperatureSafetyEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->temperatureSafetyEnabled;
}


GoFx(kBool) GoSetup_TemperatureSafetyValueUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->temperatureSafetyUsed;
}


GoFx(kStatus) GoSetup_SetScanMode( GoSetup setup, GoMode mode )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    if (GoOptionList_Check32u(kArrayList_DataT(obj->scanModeOptions, const k32u), kArrayList_Count(obj->scanModeOptions), mode) != kTRUE)
    {
        return kERROR_PARAMETER;
    }

    obj->scanMode = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoMode) GoSetup_ScanMode( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->scanMode;
}

GoFx(kSize) GoSetup_ScanModeOptionCount(GoSetup setup)
{
   kObj(GoSetup, setup);

   GoSensor_SyncConfig(obj->sensor);

   return kArrayList_Count(obj->scanModeOptions);
}

GoFx(GoMode) GoSetup_ScanModeOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_ScanModeOptionCount(setup));

    return kArrayList_AsT(obj->scanModeOptions, index, GoMode);
}

GoFx(kBool) GoSetup_UniformSpacingEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->uniformSpacingEnabled;
}

GoFx(kBool) GoSetup_UniformSpacingEnabledSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->uniformSpacingEnabledSystemValue;
}

GoFx(kStatus) GoSetup_EnableUniformSpacing(GoSetup setup, kBool enable)
{
   kObj(GoSetup, setup);

   kCheckState(GoSensor_IsConfigurable(obj->sensor));
   kCheck(GoSensor_CacheConfig(obj->sensor));

   obj->uniformSpacingEnabled = enable;
   kCheck(GoSensor_SetConfigModified(obj->sensor));

   return kOK;
}

GoFx(kBool) GoSetup_UniformSpacingAvailable(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->uniformSpacingAvail;
}

GoFx(kBool) GoSetup_OcclusionReductionEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionEnabled;
}

GoFx(kStatus) GoSetup_EnableOcclusionReduction(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->occlusionReductionEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_OcclusionReductionEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionEnabledUsed;
}

GoFx(kBool) GoSetup_OcclusionReductionEnabledSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionEnabledSystemValue;
}

GoFx(GoOcclusionReductionAlg) GoSetup_OcclusionReductionAlg(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionAlg;
}

GoFx(kStatus) GoSetup_SetOcclusionReductionAlg(GoSetup setup, GoOcclusionReductionAlg alg)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->occlusionReductionAlg = alg;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_OcclusionReductionAlgUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->occlusionReductionAlgUsed;
}

GoFx(kStatus) GoSetup_EnableFlickerFreeMode(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    if (!GoSetup_FlickerFreeModeAvailable(setup))
    {
        return kERROR_UNIMPLEMENTED;
    }

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->flickerFreeModeEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_FlickerFreeModeEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->flickerFreeModeEnabled;
}

GoFx(kBool) GoSetup_FlickerFreeModeAvailable(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->flickerFreeModeAvail;
}

GoFx(kStatus) GoSetup_SetTriggerUnit( GoSetup setup, GoTriggerUnits unit )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.unit = unit;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoTriggerUnits) GoSetup_TriggerUnit(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.unit;
}

GoFx(kStatus) GoSetup_SetTriggerSource(GoSetup setup, GoTrigger source)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.source = source;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoTrigger) GoSetup_TriggerSource(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.source;
}

GoFx(kSize) GoSetup_TriggerSourceOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->trigger.sourceOptions);
}

GoFx(GoAlignmentTarget) GoSetup_TriggerSourceOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_TriggerSourceOptionCount(setup));

    return kArrayList_AsT(obj->trigger.sourceOptions, index, k32s);
}

GoFx(kStatus) GoSetup_SetTriggerExternalInputIndex(GoSetup setup, k32s index)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.externalInputIndex = index;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32s) GoSetup_TriggerExternalInputIndex(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.externalInputIndex;
}

GoFx(kBool) GoSetup_TriggerExternalInputIndexUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.externalInputIndexUsed;
}

GoFx(kSize) GoSetup_TriggerExternalInputIndexOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->trigger.externalInputIndexOptions);
}

GoFx(GoAlignmentTarget) GoSetup_TriggerExternalInputIndexOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_TriggerExternalInputIndexOptionCount(setup));

    return kArrayList_AsT(obj->trigger.externalInputIndexOptions, index, k32s);
}

GoFx(kStatus) GoSetup_SetTriggerDelay(GoSetup setup, k64f delay)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(delay, GoSetup_TriggerDelayLimitMin(setup), GoSetup_TriggerDelayLimitMax(setup)));

    obj->trigger.delay.value = delay;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_TriggerDelay(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.delay.value;
}

GoFx(k64f) GoSetup_TriggerDelayLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.delay.min;
}

GoFx(k64f) GoSetup_TriggerDelayLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.delay.max;
}

GoFx(kStatus) GoSetup_EnableTriggerGate(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.gateEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_TriggerGateEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabled;
}

GoFx(kBool) GoSetup_TriggerGateEnabledSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabledSystemValue;
}

GoFx(kBool) GoSetup_TriggerGateEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.gateEnabledUsed;
}

GoFx(kStatus) GoSetup_EnableMaxFrameRate(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.maxFrameRateEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_MaxFrameRateEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.maxFrameRateEnabled;
}

GoFx(kStatus) GoSetup_SetFrameRate(GoSetup setup, k64f frameRate)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(frameRate, GoSetup_FrameRateLimitMin(setup), GoSetup_FrameRateLimitMax(setup)));

    if (frameRate < GoSetup_FrameRateLimitMax(setup))
    {
        GoSetup_EnableMaxFrameRate(setup, kFALSE);
    }

    obj->trigger.frameRate.value = frameRate;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_FrameRate(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.frameRate.value;
}

GoFx(k64f) GoSetup_TracheidRate(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.tracheidRate;
}

GoFx(k64f) GoSetup_FrameDataRate(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.frameDataRate;
}

GoFx(k64f) GoSetup_FrameRateLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.frameRate.min;
}

GoFx(k64f) GoSetup_FrameRateLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.frameRate.max;
}

GoFx(kStatus) GoSetup_SetEncoderTriggerMode( GoSetup setup, GoEncoderTriggerMode mode )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.encoderTriggerMode = mode;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoEncoderTriggerMode) GoSetup_EncoderTriggerMode( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.encoderTriggerMode;
}

GoFx(kStatus) GoSetup_SetEncoderSpacing(GoSetup setup, k64f period)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(period, GoSetup_EncoderSpacingLimitMin(setup), GoSetup_EncoderSpacingLimitMax(setup)));

    obj->trigger.encoderSpacing.value = period;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_EncoderSpacing(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.encoderSpacing.value;
}

GoFx(k64f) GoSetup_EncoderSpacingLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.encoderSpacing.min;
}

GoFx(k64f) GoSetup_EncoderSpacingLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.encoderSpacing.max;
}

GoFx(kBool) GoSetup_ReversalDistanceAutoEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.reversalDistanceAutoEnabledUsed;
}

GoFx(kBool) GoSetup_ReversalDistanceAutoEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.reversalDistanceAutoEnabled;
}

GoFx(kStatus) GoSetup_SetReversalDistanceAutoEnabled(GoSetup setup, kBool enabled)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.reversalDistanceAutoEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_ReversalDistanceUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.reversalDistanceThresholdUsed;
}

GoFx(kStatus) GoSetup_SetReversalDistance(GoSetup setup, k64f threshold)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.reversalDistanceThreshold = threshold;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ReversalDistance(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.reversalDistanceThreshold;
}

GoFx(k64f) GoSetup_ReversalDistanceSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->trigger.reversalDistanceThresholdActual;
}

GoFx(kStatus) GoSetup_SetTriggerBurstEnabled(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.burstEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    return kOK;
}

GoFx(kBool) GoSetup_TriggerBurstEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.burstEnabled;
}

GoFx(kBool) GoSetup_TriggerBurstEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.burstEnabledUsed;
}

GoFx(kStatus) GoSetup_SetTriggerBurstCount(GoSetup setup, k32u count)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.burstCount = count;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    return kOK;
}

GoFx(k32u) GoSetup_TriggerBurstCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.burstCount;
}

GoFx(kBool) GoSetup_TriggerBurstCountUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.burstCountUsed;

}

GoFx(kBool) GoSetup_LaserSleepUsed(GoSetup setup)
{
    kObj(GoSetup, setup);
    
    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.laserSleepUsed;
}

GoFx(kStatus) GoSetup_SetLaserSleepModeEnabled(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.laserSleepEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    return kOK;
}

GoFx(kBool) GoSetup_LaserSleepModeEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.laserSleepEnabled;
}

GoFx(kStatus) GoSetup_SetLaserIdleTime(GoSetup setup, k64u time)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.laserSleepIdleTime = time;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    return kOK;
}

GoFx(k64u) GoSetup_LaserIdleTime(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.laserSleepIdleTime;
}

GoFx(kStatus) GoSetup_SetLaserWakeupEncoderTravel(GoSetup setup, k64u distance)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->trigger.laserSleepWakeupEncoderTravel = distance;
    kCheck(GoSensor_SetConfigModified(obj->sensor));
    return kOK;
}

GoFx(k64u) GoSetup_LaserWakeupEncoderTravel(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->trigger.laserSleepWakeupEncoderTravel;
}



GoFx(kStatus) GoSetup_EnableIntensity(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->intensityEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_IntensityEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityEnabled;
}

GoFx(kBool) GoSetup_IntensityEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityAvail;
}

GoFx(kBool) GoSetup_IntensityEnabledSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->intensityEnabledSystemValue;
}

GoFx(kBool) GoSetup_AlignmentUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.used;
}

GoFx(kStatus) GoSetup_SetAlignmentType( GoSetup setup, GoAlignmentType type)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->alignment.typeOptions, const k32u),
                            kArrayList_Count(obj->alignment.typeOptions),
                            type));

    obj->alignment.type = type;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAlignmentType) GoSetup_AlignmentType( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.type;
}

GoFx(kSize) GoSetup_AlignmentTypeOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.typeOptions);
}

GoFx(GoAlignmentType) GoSetup_AlignmentTypeOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentTypeOptionCount(setup));

    return kArrayList_AsT(obj->alignment.typeOptions, index, GoAlignmentType);
}

GoFx(kStatus) GoSetup_EnableAlignmentEncoderCalibrate(GoSetup setup, kBool enabled)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.encoderCalibrateEnabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_AlignmentEncoderCalibrateEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.encoderCalibrateEnabled;
}

GoFx(kBool) GoSetup_InputTriggerEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.inputTriggerEnabled;
}

GoFx(kStatus) GoSetup_EnableInputTrigger(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.inputTriggerEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_InputTriggerEnabledUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.inputTriggerEnabledUsed;
}

GoFx(kBool) GoSetup_InputTriggerEnabledSystemValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.inputTriggerEnabledSystemValue;
}


GoFx(kStatus) GoSetup_SetAlignmentStationaryTarget( GoSetup setup, GoAlignmentTarget target )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->alignment.stationaryTargetOptions, const k32u),
                                kArrayList_Count(obj->alignment.stationaryTargetOptions),
                                target));

    obj->alignment.stationaryTarget = target;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTarget( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.stationaryTarget;
}

GoFx(kSize) GoSetup_AlignmentStationaryTargetOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.stationaryTargetOptions);
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentStationaryTargetOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentStationaryTargetOptionCount(setup));

    return kArrayList_AsT(obj->alignment.stationaryTargetOptions, index, GoAlignmentTarget);
}


GoFx(kStatus) GoSetup_SetAlignmentMovingTarget( GoSetup setup, GoAlignmentTarget target )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->alignment.movingTargetOptions, const k32u),
        kArrayList_Count(obj->alignment.movingTargetOptions),
        target));

    obj->alignment.movingTarget = target;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTarget( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.movingTarget;
}

GoFx(kSize) GoSetup_AlignmentMovingTargetOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.movingTargetOptions);
}

GoFx(GoAlignmentTarget) GoSetup_AlignmentMovingTargetOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_AlignmentMovingTargetOptionCount(setup));

    return kArrayList_AsT(obj->alignment.movingTargetOptions, index, GoAlignmentTarget);
}

GoFx(kStatus) GoSetup_SetDiskDiameter(GoSetup setup, k64f diameter)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.diskDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_DiskDiameter(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.diskDiameter;
}

GoFx(kStatus) GoSetup_SetDiskHeight(GoSetup setup, k64f height)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.diskHeight = height;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_DiskHeight(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.diskHeight;
}

GoFx(kStatus) GoSetup_SetBarWidth(GoSetup setup, k64f width)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barWidth = width;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_BarWidth(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barWidth;
}

GoFx(kStatus) GoSetup_SetBarHeight(GoSetup setup, k64f height)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.barHeight = height;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_BarHeight(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHeight;
}

GoFx(kBool) GoSetup_BarHoleCountUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleCountUsed;
}

GoFx(kSize) GoSetup_BarHoleCountValue(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleCountValue;
}

GoFx(kStatus) GoSetup_SetBarHoleCount(GoSetup setup, kSize count)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(GoSetup_BarHoleCountUsed(setup));
    obj->alignment.barHoleCount = count;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_BarHoleCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleCount;
}

GoFx(kBool) GoSetup_BarHoleDistanceUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDistanceUsed;
}

GoFx(kStatus) GoSetup_SetBarHoleDistance(GoSetup setup, k64f distance)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(GoSetup_BarHoleDistanceUsed(setup));
    obj->alignment.barHoleDistance = distance;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_BarHoleDistance(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDistance;
}

GoFx(kBool) GoSetup_BarHoleDiameterUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDiameterUsed;
}

GoFx(kStatus) GoSetup_SetBarHoleDiameter(GoSetup setup, k64f diameter)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    kCheckArgs(GoSetup_BarHoleDiameterUsed(setup));
    obj->alignment.barHoleDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_BarHoleDiameter(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barHoleDiameter;
}

GoFx(kStatus) GoSetup_SetBarDegreesOfFreedom( GoSetup setup, GoAlignmentDegreesOfFreedom dof )
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->alignment.barDegreesOfFreedomOptions, const k32u),
        kArrayList_Count(obj->alignment.barDegreesOfFreedomOptions),
        dof));

    obj->alignment.barDegreesOfFreedom = dof;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoAlignmentDegreesOfFreedom) GoSetup_BarDegreesOfFreedom( GoSetup setup )
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barDegreesOfFreedom;
}

GoFx(kBool) GoSetup_BarDegreesOfFreedomUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.barDegreesOfFreedomUsed;
}

GoFx(kSize) GoSetup_BarDegreesOfFreedomOptionCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.barDegreesOfFreedomOptions);
}

GoFx(GoAlignmentDegreesOfFreedom) GoSetup_BarDegreesOfFreedomOptionAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < GoSetup_BarDegreesOfFreedomOptionCount(setup));

    return kArrayList_AsT(obj->alignment.barDegreesOfFreedomOptions, index, GoAlignmentDegreesOfFreedom);
}

GoFx(kStatus) GoSetup_SetPlateHeight(GoSetup setup, k64f height)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateHeight = height;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_PlateHeight(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateHeight;
}

GoFx(kStatus) GoSetup_SetPlateHoleCount(GoSetup setup, kSize count)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateHoleCount = count;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_PlateHoleCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateHoleCount;
}

GoFx(kStatus) GoSetup_SetPlateRefHoleDiameter(GoSetup setup, k64f diameter)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateRefHoleDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_PlateRefHoleDiameter(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateRefHoleDiameter;
}

GoFx(kStatus) GoSetup_SetPlateSecHoleDiameter(GoSetup setup, k64f diameter)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->alignment.plateSecHoleDiameter = diameter;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_PlateSecHoleDiameter(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->alignment.plateSecHoleDiameter;
}

GoFx(kBool) GoSetup_XSmoothingUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSmoothing.used;
}

GoFx(kStatus) GoSetup_AddPolygonCorner(GoSetup setup, GoPolygonCornerParameters* corner)
{
    kObj(GoSetup, setup);

    kCheck(kArrayList_AddT(obj->alignment.polygonCorners, corner));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_AddPolygonCornerParams(GoSetup setup, k64f pointx, k64f pointz, const kChar *devices)
{
    kObj(GoSetup, setup);
    GoPolygonCornerParameters cornerParams;
    kSize i;
    kString str = kNULL;
    kArrayList tokens = kNULL;

    cornerParams.point.x = pointx;
    cornerParams.point.y = pointz;
        
    kTry
    {
        kTest(kArrayList_Construct(&cornerParams.deviceIdxs,kTypeOf(k32u), 0, kObject_Alloc(setup)));
        kTest(kString_Construct(&str, devices, kObject_Alloc(setup)));
        kTest(kString_Split(str, ",", &tokens, kObject_Alloc(setup)));

        for (i = 0; i < kArrayList_Count(tokens); i++)
        {
            kChar* deviceStr = kString_Chars(kArrayList_AsT(tokens, i, kString));

            if (deviceStr[0])   // if not empty string
            {
                k32u deviceIdx = (k32u)atoi(deviceStr);
                kArrayList_AddT(cornerParams.deviceIdxs, &deviceIdx);
            }
        }
        kTest(kArrayList_AddT(obj->alignment.polygonCorners, &cornerParams));
        kTest(GoSensor_SetConfigModified(obj->sensor));
    }
    kFinally
    {
        kDisposeRef(&tokens);
        kDestroyRef(&str);
        kEndFinally();
    }

    return kOK;
}

GoFx(kStatus) GoSetup_ClearPolygonCorners(GoSetup setup)
{
    kObj(GoSetup, setup);
    GoPolygonCornerParameters* cornerParams;
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->alignment.polygonCorners); i++)
    {
        cornerParams = kArrayList_AtT(obj->alignment.polygonCorners, i, GoPolygonCornerParameters);
        kCheck(kDestroyRef(&cornerParams->deviceIdxs));
    }

    kCheck(kArrayList_Clear(obj->alignment.polygonCorners));
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(GoPolygonCornerParameters*) GoSetup_PolygonCornerAt(GoSetup setup, kSize index)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    kAssert(index < kArrayList_Count(obj->alignment.polygonCorners));

    return kArrayList_AtT(obj->alignment.polygonCorners, index, GoPolygonCornerParameters);
}

GoFx(kSize) GoSetup_PolygonCornerCount(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return kArrayList_Count(obj->alignment.polygonCorners);
}

GoFx(kStatus) GoSetup_EnableXSmoothing(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xSmoothing.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_XSmoothingEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSmoothing.value.enabled;
}

GoFx(kStatus) GoSetup_SetXSmoothingWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XSmoothingWindowLimitMin(setup), GoSetup_XSmoothingWindowLimitMax(setup)));

    obj->filters.xSmoothing.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_XSmoothingWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSmoothing.value.value;
}

GoFx(k64f) GoSetup_XSmoothingWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSmoothing.value.min;
}

GoFx(k64f) GoSetup_XSmoothingWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSmoothing.value.max;
}

GoFx(kBool) GoSetup_YSmoothingUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySmoothing.used;
}

GoFx(kStatus) GoSetup_EnableYSmoothing(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.ySmoothing.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_YSmoothingEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySmoothing.value.enabled;
}

GoFx(kStatus) GoSetup_SetYSmoothingWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YSmoothingWindowLimitMin(setup), GoSetup_YSmoothingWindowLimitMax(setup)));

    obj->filters.ySmoothing.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_YSmoothingWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySmoothing.value.value;
}

GoFx(k64f) GoSetup_YSmoothingWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySmoothing.value.min;
}

GoFx(k64f) GoSetup_YSmoothingWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySmoothing.value.max;
}

GoFx(kBool) GoSetup_XMedianUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xMedian.used;
}

GoFx(kStatus) GoSetup_EnableXMedian(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xMedian.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_XMedianEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xMedian.value.enabled;
}

GoFx(kStatus) GoSetup_SetXMedianWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XMedianWindowLimitMin(setup), GoSetup_XMedianWindowLimitMax(setup)));

    obj->filters.xMedian.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_XMedianWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xMedian.value.value;
}

GoFx(k64f) GoSetup_XMedianWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xMedian.value.min;
}

GoFx(k64f) GoSetup_XMedianWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xMedian.value.max;
}

GoFx(kBool) GoSetup_YMedianUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yMedian.used;
}

GoFx(kStatus) GoSetup_EnableYMedian(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yMedian.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_YMedianEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yMedian.value.enabled;
}

GoFx(kStatus) GoSetup_SetYMedianWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YMedianWindowLimitMin(setup), GoSetup_YMedianWindowLimitMax(setup)));

    obj->filters.yMedian.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_YMedianWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yMedian.value.value;
}

GoFx(k64f) GoSetup_YMedianWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yMedian.value.min;
}

GoFx(k64f) GoSetup_YMedianWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yMedian.value.max;
}

GoFx(kBool) GoSetup_XDecimationUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xDecimation.used;
}

GoFx(kStatus) GoSetup_EnableXDecimation(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xDecimation.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_XDecimationEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xDecimation.value.enabled;
}

GoFx(kStatus) GoSetup_SetXDecimationWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XDecimationWindowLimitMin(setup), GoSetup_XDecimationWindowLimitMax(setup)));

    obj->filters.xDecimation.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_XDecimationWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xDecimation.value.value;
}

GoFx(k64f) GoSetup_XDecimationWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xDecimation.value.min;
}

GoFx(k64f) GoSetup_XDecimationWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xDecimation.value.max;
}

GoFx(kBool) GoSetup_YDecimationUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yDecimation.used;
}

GoFx(kStatus) GoSetup_EnableYDecimation(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yDecimation.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_YDecimationEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yDecimation.value.enabled;
}

GoFx(kStatus) GoSetup_SetYDecimationWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YDecimationWindowLimitMin(setup), GoSetup_YDecimationWindowLimitMax(setup)));

    obj->filters.yDecimation.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_YDecimationWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yDecimation.value.value;
}

GoFx(k64f) GoSetup_YDecimationWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yDecimation.value.min;
}

GoFx(k64f) GoSetup_YDecimationWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yDecimation.value.max;
}

GoFx(kBool) GoSetup_XGapFillingUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xGapFilling.used;
}

GoFx(kStatus) GoSetup_EnableXGapFilling(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xGapFilling.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_XGapFillingEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xGapFilling.value.enabled;
}

GoFx(kStatus) GoSetup_SetXGapFillingWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XGapFillingWindowLimitMin(setup), GoSetup_XGapFillingWindowLimitMax(setup)));

    obj->filters.xGapFilling.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_XGapFillingWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xGapFilling.value.value;
}

GoFx(k64f) GoSetup_XGapFillingWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xGapFilling.value.min;
}

GoFx(k64f) GoSetup_XGapFillingWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xGapFilling.value.max;
}

GoFx(kBool) GoSetup_YGapFillingUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yGapFilling.used;
}

GoFx(kStatus) GoSetup_EnableYGapFilling(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.yGapFilling.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_YGapFillingEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yGapFilling.value.enabled;
}

GoFx(kStatus) GoSetup_SetYGapFillingWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YGapFillingWindowLimitMin(setup), GoSetup_YGapFillingWindowLimitMax(setup)));

    obj->filters.yGapFilling.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_YGapFillingWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.yGapFilling.value.value;
}

GoFx(k64f) GoSetup_YGapFillingWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yGapFilling.value.min;
}

GoFx(k64f) GoSetup_YGapFillingWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.yGapFilling.value.max;
}

GoFx(kBool) GoSetup_XSlopeUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSlope.used;
}

GoFx(kStatus) GoSetup_EnableXSlope(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.xSlope.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_XSlopeEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSlope.value.enabled;
}

GoFx(kStatus) GoSetup_SetXSlopeWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_XSlopeWindowLimitMin(setup), GoSetup_XSlopeWindowLimitMax(setup)));

    obj->filters.xSlope.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_XSlopeWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.xSlope.value.value;
}

GoFx(k64f) GoSetup_XSlopeWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSlope.value.min;
}

GoFx(k64f) GoSetup_XSlopeWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.xSlope.value.max;
}

GoFx(kBool) GoSetup_YSlopeUsed(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySlope.used;
}

GoFx(kStatus) GoSetup_EnableYSlope(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));

    obj->filters.ySlope.value.enabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_YSlopeEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySlope.value.enabled;
}

GoFx(kStatus) GoSetup_SetYSlopeWindow(GoSetup setup, k64f window)
{
    kObj(GoSetup, setup);

    kCheckState(GoSensor_IsConfigurable(obj->sensor));
    kCheck(GoSensor_CacheConfig(obj->sensor));
    kCheckArgs(GoUtils_MinMax_(window, GoSetup_YSlopeWindowLimitMin(setup), GoSetup_YSlopeWindowLimitMax(setup)));

    obj->filters.ySlope.value.value = window;

    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_YSlopeWindow(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->filters.ySlope.value.value;
}

GoFx(k64f) GoSetup_YSlopeWindowLimitMin(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySlope.value.min;
}

GoFx(k64f) GoSetup_YSlopeWindowLimitMax(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);
    return obj->filters.ySlope.value.max;
}


//*****  ROLE SPECIFIC API  *****//


GoFx(k64f) GoSetup_ExposureLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->exposure.max;
}

GoFx(k64f) GoSetup_ExposureLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->exposure.min;
}

GoFx(kStatus) GoSetup_SetExposure(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->exposure.value = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_Exposure(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposure.value;
}

GoFx(GoExposureMode) GoSetup_ExposureMode(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposureMode;
}

GoFx(kStatus) GoSetup_SetExposureMode(GoSetup setup, GoRole role, GoExposureMode mode)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->exposureModeOptions, const k32u),
        kArrayList_Count(obj->exposureModeOptions),
        mode));

    obj->exposureMode = mode;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_ExposureModeOptionCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return kArrayList_Count(obj->exposureModeOptions);
}

GoFx(GoExposureMode) GoSetup_ExposureModeOptionAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_ExposureModeOptionCount(setup, role));

    return kArrayList_AsT(obj->exposureModeOptions, index, GoExposureMode);
}

GoFx(kStatus) GoSetup_AddExposureStep(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);
    kSize exposureStepCount = obj->exposureStepCount;   //the limit function calls below will trigger a config read

    if(exposureStepCount < GO_MAX_EXPOSURE_COUNT)
    {
        kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));
    }
    else
    {
        return kERROR;
    }

    obj->exposureSteps[exposureStepCount] = exposure;
    obj->exposureStepCount = exposureStepCount + 1;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_ClearExposureSteps(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    obj->exposureStepCount = 0;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ExposureStepAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_ExposureStepCount(setup, role));

    return obj->exposureSteps[index];
}

GoFx(kSize) GoSetup_ExposureStepCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->exposureStepCount;
}

GoFx(k64f) GoSetup_DynamicExposureMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->dynamicExposureMax;
}

GoFx(kStatus) GoSetup_SetDynamicExposureMax(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->dynamicExposureMax = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_DynamicExposureMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->dynamicExposureMin;
}

GoFx(kStatus) GoSetup_SetDynamicExposureMin(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_ExposureLimitMin(setup, role), GoSetup_ExposureLimitMax(setup, role)));

    obj->dynamicExposureMin = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kStatus) GoSetup_SetIntensityStepIndex(GoSetup setup, GoRole role, kSize stepIndex)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheckState(stepIndex < GoSetup_ExposureStepCount(setup, role));

    obj->intensityStepIndex = stepIndex;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kSize) GoSetup_IntensityStepIndex(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->intensityStepIndex;
}

GoFx(kStatus) GoSetup_SetPatternSequenceType(GoSetup setup, GoRole role, GoPatternSequenceType type)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->patternSequenceTypeOptions, const k32u),
        kArrayList_Count(obj->patternSequenceTypeOptions),
        type));

    obj->patternSequenceType = type;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(GoPatternSequenceType) GoSetup_PatternSequenceType(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceType;
}

GoFx(kBool) GoSetup_PatternSequenceTypeUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceTypeUsed;
}

GoFx(kSize) GoSetup_PatternSequenceTypeOptionCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return kArrayList_Count(obj->patternSequenceTypeOptions);
}

GoFx(GoPatternSequenceType) GoSetup_PatternSequenceTypeOptionAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_PatternSequenceTypeOptionCount(setup, role));

    return kArrayList_AsT(obj->patternSequenceTypeOptions, index, GoPatternSequenceType);
}


GoFx(kSize) GoSetup_PatternSequenceCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceCount;
}

GoFx(k32u) GoSetup_PatternSequenceIndex(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceIndex;
}
GoFx(kStatus) GoSetup_SetPatternSequenceIndex(GoSetup setup, GoRole role, k32u index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kCheckTrue(index <= obj->patternSequenceIndexMax, kERROR_PARAMETER);

    obj->patternSequenceIndex = index;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}
GoFx(kBool) GoSetup_PatternSequenceIndexUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceIndexUsed;
}
GoFx(k32u) GoSetup_PatternSequenceIndexMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceIndexMin;
}
GoFx(k32u) GoSetup_PatternSequenceIndexMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->patternSequenceIndexMax;
}


//transformed data region read only functions?
GoFx(k64f) GoSetup_TransformedDataRegionX(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.x;
}

GoFx(k64f) GoSetup_TransformedDataRegionY(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.y;
}

GoFx(k64f) GoSetup_TransformedDataRegionZ(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.z;
}

GoFx(k64f) GoSetup_TransformedDataRegionWidth(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.width;
}

GoFx(k64f) GoSetup_TransformedDataRegionLength(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.length;
}

GoFx(k64f) GoSetup_TransformedDataRegionHeight(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->transformedDataRegion.height;
}

GoFx(k64f) GoSetup_ActiveAreaHeightLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.min;
}

GoFx(k64f) GoSetup_ActiveAreaHeightLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaHeight(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaHeightLimitMin(setup, role), GoSetup_ActiveAreaHeightLimitMax(setup, role)));

    obj->activeArea.height.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaHeight(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.height.value;
}

GoFx(k64f) GoSetup_ActiveAreaWidthLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.width.min;
}

GoFx(k64f) GoSetup_ActiveAreaWidthLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.width.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaWidth(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaWidthLimitMin(setup, role), GoSetup_ActiveAreaWidthLimitMax(setup, role)));

    obj->activeArea.width.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaWidth(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.width.value;
}


GoFx(k64f) GoSetup_ActiveAreaLengthLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.min;
}

GoFx(k64f) GoSetup_ActiveAreaLengthLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaLength(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaLengthLimitMin(setup, role), GoSetup_ActiveAreaLengthLimitMax(setup, role)));

    obj->activeArea.length.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaLength(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.length.value;
}

GoFx(k64f) GoSetup_ActiveAreaXLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.x.min;
}

GoFx(k64f) GoSetup_ActiveAreaXLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.x.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaX(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaXLimitMin(setup, role), GoSetup_ActiveAreaXLimitMax(setup, role)));

    obj->activeArea.x.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaX(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.x.value;
}


GoFx(k64f) GoSetup_ActiveAreaYLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.y.min;
}

GoFx(k64f) GoSetup_ActiveAreaYLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.y.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaY(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaYLimitMin(setup, role), GoSetup_ActiveAreaYLimitMax(setup, role)));

    obj->activeArea.y.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaY(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.y.value;
}


GoFx(k64f) GoSetup_ActiveAreaZLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.z.min;
}

GoFx(k64f) GoSetup_ActiveAreaZLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->activeArea.z.max;
}

GoFx(kStatus) GoSetup_SetActiveAreaZ(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(value, GoSetup_ActiveAreaZLimitMin(setup, role), GoSetup_ActiveAreaZLimitMax(setup, role)));

    obj->activeArea.z.value = value;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_ActiveAreaZ(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->activeArea.z.value;
}


GoFx(kSize) GoSetup_XSubsamplingOptionCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->xSubsamplingOptionCount;
}

GoFx(k32u) GoSetup_XSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    if (index >= obj->xSubsamplingOptionCount)
    {
        return k32U_NULL;
    }

    return obj->xSubsamplingOptions[index];
}

GoFx(kStatus) GoSetup_SetXSubsampling( GoSetup setup, GoRole role, k32u xSubsampling )
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);
    GoSensor_SyncConfig(setupObj->sensor);
    kCheck(GoOptionList_Check32u(obj->xSubsamplingOptions, obj->xSubsamplingOptionCount, xSubsampling));

    obj->xSubsampling = xSubsampling;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_XSubsampling(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->xSubsampling;
}

GoFx(kSize) GoSetup_ZSubsamplingOptionCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->zSubsamplingOptionCount;
}

GoFx(k32u) GoSetup_ZSubsamplingOptionAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    if (index >= obj->zSubsamplingOptionCount)
    {
        return k32U_NULL;
    }

    return obj->zSubsamplingOptions[index];
}

GoFx(kStatus) GoSetup_SetZSubsampling(GoSetup setup, GoRole role, k32u zSubsampling)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);
    GoSensor_SyncConfig(setupObj->sensor);
    kCheck(GoOptionList_Check32u(obj->zSubsamplingOptions, obj->zSubsamplingOptionCount, zSubsampling));

    obj->zSubsampling = zSubsampling;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_ZSubsampling(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->zSubsampling;
}

GoFx(k64f) GoSetup_SpacingIntervalSystemValue(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.systemValue;
}

GoFx(k64f) GoSetup_SpacingInterval(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.value;
}

GoFx(kBool) GoSetup_SpacingIntervalUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.enabled;
}

GoFx(kStatus) GoSetup_SetSpacingInterval(GoSetup setup, GoRole role, k64f value)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheckArgs(GoUtils_MinMax_(value, GoSetup_SpacingIntervalLimitMin(setup, role), GoSetup_SpacingIntervalLimitMax(setup, role)));

    obj->spacingInterval.value = value;

    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_SpacingIntervalLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.min;
}

GoFx(k64f) GoSetup_SpacingIntervalLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingInterval.max;
}

GoFx(GoSpacingIntervalType) GoSetup_SpacingIntervalType(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingIntervalType.value;
}

GoFx(kBool) GoSetup_SpacingIntervalTypeUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->spacingIntervalType.enabled;
}

GoFx(kStatus) GoSetup_SetSpacingIntervalType(GoSetup setup, GoRole role, GoSpacingIntervalType type)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    obj->spacingIntervalType.value = type;

    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_XSpacingCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->xSpacingCount;
}

GoFx(k32u) GoSetup_YSpacingCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->ySpacingCount;
}

GoFx(k32u) GoSetup_FrontCameraX(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->frontCamera.x;
}

GoFx(k32u) GoSetup_FrontCameraY(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->frontCamera.y;
}

GoFx(k32u) GoSetup_FrontCameraWidth(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->frontCamera.width;
}

GoFx(k32u) GoSetup_FrontCameraHeight(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->frontCamera.height;
}


GoFx(k32u) GoSetup_BackCameraX(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCamera.x;
}

GoFx(k32u) GoSetup_BackCameraY(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCamera.y;
}

GoFx(k32u) GoSetup_BackCameraWidth(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCamera.width;
}

GoFx(k32u) GoSetup_BackCameraHeight(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCamera.height;
}

GoFx(k32u) GoSetup_BackCameraUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->backCameraUsed;
}

GoFx(kStatus) GoSetup_EnableTracking(GoSetup setup, GoRole role, kBool enable)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    obj->tracking.enabled = enable;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_TrackingEnabled(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.enabled;
}

GoFx(kBool) GoSetup_TrackingUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.used;
}

GoFx(kStatus) GoSetup_SetTrackingAreaHeight(GoSetup setup, GoRole role, k64f height)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    setupObj = obj->setup;
    kCheckArgs(GoUtils_MinMax_(height, GoSetup_TrackingAreaHeightLimitMin(setup, role), GoSetup_TrackingAreaHeightLimitMax(setup, role)));

    obj->tracking.trackingAreaHeight.value = height;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_TrackingAreaHeight(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.trackingAreaHeight.value;
}

GoFx(k64f) GoSetup_TrackingAreaHeightLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->tracking.trackingAreaHeight.min;
}

GoFx(k64f) GoSetup_TrackingAreaHeightLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->tracking.trackingAreaHeight.max;
}

GoFx(kStatus) GoSetup_SetTrackingSearchThreshold(GoSetup setup, GoRole role, k64f threshold)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);
    kCheckArgs(GoUtils_MinMax_(threshold, 0, 100));

    obj->tracking.searchThreshold = threshold;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_TrackingSearchThreshold(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracking.searchThreshold;
}

GoFx(GoLayout) GoSetup_Layout(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->layout;
}

GoFx(GoProfileGeneration) GoSetup_ProfileGeneration(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->profileGeneration;
}

GoFx(GoSurfaceGeneration) GoSetup_SurfaceGeneration(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->surfaceGeneration;
}

GoFx(GoPartDetection) GoSetup_PartDetection(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->partDetection;
}

GoFx(GoPartMatching) GoSetup_PartMatching(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->partMatching;
}

GoFx(kStatus) GoSetup_EnableExternalInputZPulse(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    obj->externalInputZPulseEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_ExternalInputZPulseEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->externalInputZPulseEnabled;
}

GoFx(kStatus) GoSetup_SetExternalInputZPulseIndex(GoSetup setup, k32u index)
{
    kObj(GoSetup, setup);

    obj->externalInputZPulseIndex = index;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(k32u) GoSetup_ExternalInputZPulseIndex(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->externalInputZPulseIndex;
}

GoFx(kBool) GoSetup_ExternalInputZPulseIndexAvailable(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->externalInputZPulseIndexAvail;
}

GoFx(kStatus) GoSetup_EnablePreferMasterTimeEncoderEnabled(GoSetup setup, kBool enable)
{
    kObj(GoSetup, setup);

    obj->preferMasterTimeEncoderEnabled = enable;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_PreferMasterTimeEncoderEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->preferMasterTimeEncoderEnabled;
}

GoFx(GoMaterial) GoSetup_Advanced(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->advanced;
}

GoFx(GoMaterial) GoSetup_Material(GoSetup setup, GoRole role)
{
    return GoSetup_Advanced(setup,role);
}

GoFx(GoSections) GoSetup_Sections(GoSetup setup)
{
    kObj(GoSetup, setup);

    return obj->sections;
}

GoFx(GoTracheid) GoSetup_Tracheid(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->tracheid;
}

GoFx(kStatus) GoSetup_EnableIndependentExposures(GoSetup setup, GoRole role, kBool enable)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    obj->independentExposures.enabled = enable;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_IndependentExposuresEnabled(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->independentExposures.enabled;
}

GoFx(kBool) GoSetup_IndependentExposuresUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->independentExposures.used;
}

GoFx(k64f) GoSetup_FrontCameraExposureLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->independentExposures.frontCameraExposure.max;
}

GoFx(k64f) GoSetup_FrontCameraExposureLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->independentExposures.frontCameraExposure.min;
}

GoFx(kStatus) GoSetup_SetFrontCameraExposure(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_FrontCameraExposureLimitMin(setup, role), GoSetup_FrontCameraExposureLimitMax(setup, role)));

    obj->independentExposures.frontCameraExposure.value = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_FrontCameraExposure(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->independentExposures.frontCameraExposure.value;
}

GoFx(k64f) GoSetup_BackCameraExposureLimitMax(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->independentExposures.backCameraExposure.max;
}

GoFx(k64f) GoSetup_BackCameraExposureLimitMin(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return obj->independentExposures.backCameraExposure.min;
}

GoFx(kStatus) GoSetup_SetBackCameraExposure(GoSetup setup, GoRole role, k64f exposure)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheckArgs(GoUtils_MinMax_(exposure, GoSetup_BackCameraExposureLimitMin(setup, role), GoSetup_BackCameraExposureLimitMax(setup, role)));

    obj->independentExposures.backCameraExposure.value = exposure;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(k64f) GoSetup_BackCameraExposure(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->independentExposures.backCameraExposure.value;
}

GoFx(GoIntensitySource) GoSetup_IntensitySource(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->intensitySource;
}

GoFx(kStatus) GoSetup_SetIntensitySource(GoSetup setup, GoRole role, GoIntensitySource source)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    kCheck(GoOptionList_Check32u(kArrayList_DataT(obj->intensitySourceOptions, const k32u),
        kArrayList_Count(obj->intensitySourceOptions),
        source));

    obj->intensitySource = source;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(GoIntensityMode) GoSetup_IntensityMode(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->intensityMode;
}

GoFx(kStatus) GoSetup_SetIntensityMode(GoSetup setup, GoRole role, GoIntensityMode mode)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    obj->intensityMode = mode;
    kCheck(GoSensor_SetConfigModified(setupObj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_IntensityModeUsed(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    return obj->intensityModeAvail;
}

GoFx(kSize) GoSetup_IntensitySourceOptionCount(GoSetup setup, GoRole role)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);
    return kArrayList_Count(obj->intensitySourceOptions);
}

GoFx(GoExposureMode) GoSetup_IntensitySourceOptionAt(GoSetup setup, GoRole role, kSize index)
{
    kObj(GoSetupNode, GoSetup_FindNode(setup, role));
    kObjN(GoSetup, setupObj, obj->setup);

    GoSensor_SyncConfig(setupObj->sensor);

    kAssert(index < GoSetup_IntensitySourceOptionCount(setup, role));

    return kArrayList_AsT(obj->intensitySourceOptions, index, GoIntensitySource);
}


GoFx(kStatus) GoSetup_EnableBackgroundSuppression(GoSetup setup, kBool enabled)
{
    kObj(GoSetup, setup);
   
    obj->backgroundSuppression.enabled = enabled;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}

GoFx(kBool) GoSetup_BackgroundSuppressionEnabled(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->backgroundSuppression.enabled;
}

GoFx(kStatus) GoSetup_SetBackgroundSuppressionRatio(GoSetup setup, k64u ratio)
{
    kObj(GoSetup, setup);

    obj->backgroundSuppression.ratio = ratio;
    kCheck(GoSensor_SetConfigModified(obj->sensor));

    return kOK;
}
GoFx(k64u) GoSetup_BackgroundSuppressionRatio(GoSetup setup)
{
    kObj(GoSetup, setup);

    GoSensor_SyncConfig(obj->sensor);

    return obj->backgroundSuppression.ratio;
}