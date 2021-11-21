/**
 * @file    GoProfileTools.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_PROFILE_TOOLS_X_H
#define GO_PROFILE_TOOLS_X_H

#include <kApi/Data/kXml.h>

// Define Profile tool names, ordered alphabetically.
#define GO_PROFILE_TOOL_NAME_AREA           "ProfileArea"
#define GO_PROFILE_TOOL_NAME_BOUNDING_BOX   "ProfileBoundingBox"
#define GO_PROFILE_TOOL_NAME_BRIDGE_VALUE   "ProfileBridgeValue"
#define GO_PROFILE_TOOL_NAME_CIRCLE         "ProfileCircle"
#define GO_PROFILE_TOOL_NAME_DIMENSION      "ProfileDimension"
#define GO_PROFILE_TOOL_NAME_GROOVE         "ProfileGroove"
#define GO_PROFILE_TOOL_NAME_INTERSECT      "ProfileIntersect"
#define GO_PROFILE_TOOL_NAME_LINE           "ProfileLine"
#define GO_PROFILE_TOOL_NAME_PANEL          "ProfilePanel"
#define GO_PROFILE_TOOL_NAME_POSITION       "ProfilePosition"
#define GO_PROFILE_TOOL_NAME_ROUND_CORNER   "ProfileRoundCorner"
#define GO_PROFILE_TOOL_NAME_STRIP          "ProfileStrip"

typedef struct GoProfileToolClass
{
    GoToolClass base;

    kArrayList streamOptions;   //of GoStreamOption
    GoDataStream stream;

    kArrayList sourceOptions;
    GoDataSource source;

    kArrayList xAnchorOptions;   //of k32u
    k32s xAnchor;
    kArrayList zAnchorOptions;   //of k32u
    k32s zAnchor;
} GoProfileToolClass;

kDeclareClassEx(Go, GoProfileTool, GoTool)

GoFx(kStatus) GoProfileTool_Init(GoProfileTool tool, kType type, GoToolType typeId, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileTool_VRelease(GoProfileTool tool);
GoFx(kStatus) GoProfileTool_Read(GoProfileTool tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileTool_Write(GoProfileTool tool, kXml xml, kXmlItem item);


typedef struct GoProfileAreaClass
{
    GoProfileToolClass base;

    GoProfileAreaType type;
    kBool typeUsed;

    GoProfileBaseline baseline;
    kBool baselineUsed;

    GoProfileRegion region;
    kBool regionEnabled;
    GoProfileLineRegion line;
} GoProfileAreaClass;

kDeclareClassEx(Go, GoProfileArea, GoProfileTool)

GoFx(kStatus) GoProfileArea_Construct(GoProfileArea* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileArea_VInit(GoProfileArea tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileArea_VRelease(GoProfileArea tool);
GoFx(kStatus) GoProfileArea_VRead(GoProfileArea tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileArea_VWrite(GoProfileArea tool, kXml xml, kXmlItem item);


typedef struct GoProfileBoxClass
{
    GoProfileToolClass base;
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileBoxClass;

kDeclareClassEx(Go, GoProfileBox, GoProfileTool)

GoFx(kStatus) GoProfileBox_Construct(GoProfileBox* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileBox_VInit(GoProfileBox tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileBox_VRelease(GoProfileBox tool);
GoFx(kStatus) GoProfileBox_VRead(GoProfileBox tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileBox_VWrite(GoProfileBox tool, kXml xml, kXmlItem item);


typedef struct GoProfileBridgeValueClass
{
    GoProfileToolClass base;

    kBool regionEnabled;
    GoProfileRegion region;
    kBool normalizeEnabled;
    k64f windowSize;
    k64f windowSkip;
    k64f maxInvalid;
    GoElement64f maxDifferential;
} GoProfileBridgeValueClass;

kDeclareClassEx(Go, GoProfileBridgeValue, GoProfileTool)

GoFx(kStatus) GoProfileBridgeValue_Construct(GoProfileBridgeValue* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileBridgeValue_VInit(GoProfileBridgeValue tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileBridgeValue_VRelease(GoProfileBridgeValue tool);
GoFx(kStatus) GoProfileBridgeValue_VRead(GoProfileBridgeValue tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileBridgeValue_VWrite(GoProfileBridgeValue tool, kXml xml, kXmlItem item);


typedef struct GoProfileCircleClass
{
    GoProfileToolClass base;
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileCircleClass;

kDeclareClassEx(Go, GoProfileCircle, GoProfileTool)

GoFx(kStatus) GoProfileCircle_Construct(GoProfileCircle* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileCircle_VInit(GoProfileCircle tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileCircle_VRelease(GoProfileCircle tool);
GoFx(kStatus) GoProfileCircle_VRead(GoProfileCircle tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileCircle_VWrite(GoProfileCircle tool, kXml xml, kXmlItem item);


typedef struct GoProfileDimClass
{
    GoProfileToolClass base;
    GoProfileFeature refFeature;
    GoProfileFeature feature;
} GoProfileDimClass;

kDeclareClassEx(Go, GoProfileDim, GoProfileTool)

GoFx(kStatus) GoProfileDim_Construct(GoProfileDim* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileDim_VInit(GoProfileDim tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileDim_VRelease(GoProfileDim tool);
GoFx(kStatus) GoProfileDim_VRead(GoProfileDim tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileDim_VWrite(GoProfileDim tool, kXml xml, kXmlItem item);


typedef struct GoProfileGrooveClass
{
    GoProfileToolClass base;
    GoProfileGrooveShape shape;
    k64f minWidth;
    k64f maxWidth;
    k64f minDepth;
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileGrooveClass;

kDeclareClassEx(Go, GoProfileGroove, GoProfileTool)

GoFx(kStatus) GoProfileGroove_Construct(GoProfileGroove* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileGroove_VInit(GoProfileGroove tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileGroove_VRelease(GoProfileGroove tool);
GoFx(kStatus) GoProfileGroove_VRead(GoProfileGroove tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileGroove_VWrite(GoProfileGroove tool, kXml xml, kXmlItem item);


typedef struct GoProfileIntersectClass
{
    GoProfileToolClass base;
    GoProfileBaseline refLineType;
    GoProfileLineRegion refLineRegion;
    GoProfileLineRegion lineRegion;
} GoProfileIntersectClass;

kDeclareClassEx(Go, GoProfileIntersect, GoProfileTool)

GoFx(kStatus) GoProfileIntersect_Construct(GoProfileIntersect* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileIntersect_VInit(GoProfileIntersect tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileIntersect_VRelease(GoProfileIntersect tool);
GoFx(kStatus) GoProfileIntersect_VRead(GoProfileIntersect tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileIntersect_VWrite(GoProfileIntersect tool, kXml xml, kXmlItem item);

#define GO_PROFILE_LINE_MAX_FIT_REGIONS (2)

typedef struct GoProfileLineClass
{
    GoProfileToolClass base;
    kBool regionEnabled;
    GoProfileRegion region;
    kBool fittingRegionsEnabled;
    GoProfileLineRegion fittingRegions;
} GoProfileLineClass;

kDeclareClassEx(Go, GoProfileLine, GoProfileTool)

GoFx(kStatus) GoProfileLine_Construct(GoProfileLine* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileLine_VInit(GoProfileLine tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileLine_VRelease(GoProfileLine tool);
GoFx(kStatus) GoProfileLine_VRead(GoProfileLine tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileLine_VWrite(GoProfileLine tool, kXml xml, kXmlItem item);


typedef struct GoProfilePanelClass
{
    GoProfileToolClass base;
    GoProfilePanelSide refEdgeSide;
    k64f maxGapWidth;
    GoProfileEdge leftEdge;
    GoProfileEdge rightEdge;
} GoProfilePanelClass;

kDeclareClassEx(Go, GoProfilePanel, GoProfileTool)

GoFx(kStatus) GoProfilePanel_Construct(GoProfilePanel* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfilePanel_VInit(GoProfilePanel tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfilePanel_VRelease(GoProfilePanel tool);
GoFx(kStatus) GoProfilePanel_VRead(GoProfilePanel tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePanel_VWrite(GoProfilePanel tool, kXml xml, kXmlItem item);


typedef struct GoProfilePositionClass
{
    GoProfileToolClass base;
    GoProfileFeature feature;
} GoProfilePositionClass;

kDeclareClassEx(Go, GoProfilePosition, GoProfileTool)

GoFx(kStatus) GoProfilePosition_Construct(GoProfilePosition* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfilePosition_VInit(GoProfilePosition tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfilePosition_VRelease(GoProfilePosition tool);
GoFx(kStatus) GoProfilePosition_VRead(GoProfilePosition tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfilePosition_VWrite(GoProfilePosition tool, kXml xml, kXmlItem item);


typedef struct GoProfileStripClass
{
    GoProfileToolClass base;
    GoProfileStripBaseType baseType;
    GoProfileStripEdgeType leftEdge;
    GoProfileStripEdgeType rightEdge;
    kBool tiltEnabled;
    k64f supportWidth;
    GoElement64f transitionWidth;
    k64f minWidth;
    k64f minHeight;
    k64f maxVoidWidth;
    kBool regionEnabled;
    GoProfileRegion region;
} GoProfileStripClass;

kDeclareClassEx(Go, GoProfileStrip, GoProfileTool)

GoFx(kStatus) GoProfileStrip_Construct(GoProfileStrip* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileStrip_VInit(GoProfileStrip tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileStrip_VRelease(GoProfileStrip tool);
GoFx(kStatus) GoProfileStrip_VRead(GoProfileStrip tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileStrip_VWrite(GoProfileStrip tool, kXml xml, kXmlItem item);

typedef struct GoProfileRoundCornerClass
{
    GoProfileToolClass base;
    GoProfileRoundCornerDirection refDirection;
    GoProfileEdge edge;
} GoProfileRoundCornerClass;

kDeclareClassEx(Go, GoProfileRoundCorner, GoProfileTool)

GoFx(kStatus) GoProfileRoundCorner_Construct(GoProfileRoundCorner* tool, kObject sensor, kAlloc allocator);
GoFx(kStatus) GoProfileRoundCorner_VInit(GoProfileRoundCorner tool, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoProfileRoundCorner_VRelease(GoProfileRoundCorner tool);
GoFx(kStatus) GoProfileRoundCorner_VRead(GoProfileRoundCorner tool, kXml xml, kXmlItem item);
GoFx(kStatus) GoProfileRoundCorner_VWrite(GoProfileRoundCorner tool, kXml xml, kXmlItem item);

#endif
