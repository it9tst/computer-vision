/** 
 * @file    GoAdvanced.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_ADVANCED_X_H
#define GO_ADVANCED_X_H

#include <kApi/Data/kXml.h>

typedef struct GoAdvancedClass
{
    kObjectClass base;
    
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    GoAdvancedType type;
    GoAdvancedType typeSystemValue;
    kBool typeUsed;
    
    GoElement32u spotThreshold;
    GoElement32u spotWidthMax;

    GoElement32u spotWidthMin;
    GoElement32u widthThreshold;
    GoElement32u spotSumMin;
    GoElement32u sobelEdgeWindow;

    GoSpotSelectionType spotSelectionType;
    GoSpotSelectionType systemSpotSelectionType;
    kBool spotSelectionTypeUsed;
    kArrayList spotSelectionTypeOptions; //of type GoSpotSelectionType

    k32u spotContinuitySortingMinimumSegmentSize;
    k32u spotContinuitySortingSearchWindowX;
    k32u spotContinuitySortingSearchWindowY;

    GoElement32u spotTranslucentSortingOpaqueWidth;
    GoElement32u spotTranslucentSortingTranslucentWidth;
    GoElement32u spotTranslucentSortingMinLength;
    GoElement32s spotTranslucentSortingThreadingMode;
    kArrayList spotTranslucentSortingThreadingModeOptions; //of type GoTranslucentThreadingMode

    GoElement64f cameraGainAnalog;
    GoElement64f cameraGainDigital;
    GoElement64f dynamicSensitivity;
    GoElement32u dynamicThreshold;
    GoElement32u gammaType;
    GoElementBool sensitivityCompensationEnabled;

    GoElement32u encoding;
    GoSurfacePhaseFilter phaseFilter;

    GoElement32u contrastThreshold;

} GoAdvancedClass;

kDeclareClassEx(Go, GoAdvanced, kObject)

GoFx(kStatus) GoAdvanced_Construct(GoAdvanced* layout, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoAdvanced_Init(GoAdvanced layout, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoAdvanced_VRelease(GoAdvanced layout);

GoFx(kStatus) GoAdvanced_Read(GoAdvanced layout, kXml xml, kXmlItem item);
GoFx(kStatus) GoAdvanced_Write(GoAdvanced layout, kXml xml, kXmlItem item); 

GoFx(kStatus) GoAdvanced_ReadTunableParams(GoAdvanced advanced, kChar* paramStr, GoElement32u* element, kXml xml, kXmlItem item);

#endif
