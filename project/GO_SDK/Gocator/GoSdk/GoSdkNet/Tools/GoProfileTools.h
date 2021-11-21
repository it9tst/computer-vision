//
// GoProfileTools.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_PROFILE_TOOLS_H
#define GO_SDK_NET_PROFILE_TOOLS_H

#include <GoSdk/Tools/GoProfileTools.h>
#include <GoSdkNet/Tools/GoFeatures.h>
#include <GoSdkNet/Tools/GoFeature.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a base profile tool.</summary>
            public ref class GoProfileTool abstract : public GoTool
            {
                KDeclareClass(GoProfileTool, GoProfileTool)

            public:
                /// <summary>Default GoProfileTool constructor.</summary>
                GoProfileTool() {}

                /// <summary>Initializes a new instance of the GoProfileTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileTool(IntPtr handle)
                    : GoTool(handle)
                {}

                /// <summary>The data stream.</summary>
                /// <remarks>Note that stream validation will only occur if tool is in the tool options list.</remarks>
                property GoDataStream Stream
                {
                    GoDataStream get()           { return (GoDataStream) ::GoProfileTool_Stream(Handle); }
                    void set(GoDataStream stream)  { KCheck(::GoProfileTool_SetStream(Handle, ::GoDataStream(stream))); }
                }

                /// <summary>The data stream option list count.</summary>
                /// <returns>The current profile tool data stream option list count.</returns>
                property k64s StreamOptionCount
                {
                    k64s get()           { return (k64s) ::GoProfileTool_StreamOptionCount(Handle); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The profile tool data stream option at the given index, or k32U_MAX if an invalid index is given.</returns>
                GoDataStream GetStreamOptionAt(k64s index)
                {
                   return (GoDataStream) ::GoProfileTool_StreamOptionAt(Handle, (kSize)index);
                }

                /// <summary>The data source.</summary>
                /// <remarks>Note that source validation will only occur if tool is in the tool options list.</remarks>
                property GoDataSource Source
                {
                    GoDataSource get()           { return (GoDataSource) ::GoProfileTool_Source(Handle); }
                    void set(GoDataSource source)  { KCheck(::GoProfileTool_SetSource(Handle, source)); }
                }

                /// <summary>The data source option list count.</summary>
                property k64s SourceOptionCount
                {
                    k64s get()           { return (k64s) ::GoProfileTool_SourceOptionCount(Handle); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The profile tool data source option at the given index, or k32U_MAX if an invalid index is given.</returns>
                GoDataSource GetSourceOption(k64s index)
                {
                   return (GoDataSource) ::GoProfileTool_SourceOptionAt(Handle, (kSize)index);
                }

                /// <summary>The X-anchoring option list count.</summary>
                property k64s XAnchorOptionCount
                {
                    k64s get()           { return (k64s) ::GoProfileTool_XAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the X-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The X-anchoring option at the given index or k32U_MAX if invalid.</returns>
                virtual k32u GetXAnchorOptionAt(k64s index) 
                {
                   return (k32u) ::GoProfileTool_XAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The X-anchoring source.</summary>
                property k32s XAnchor
                {
                    k32s get()           { return (k32s) ::GoProfileTool_XAnchor(Handle); }
                    void set(k32s source)  { KCheck(::GoProfileTool_SetXAnchor(Handle, source)); }
                }

                /// <summary>A boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.</summary>
                property bool XAnchorEnabled
                {
                    bool get()           { return KToBool(::GoProfileTool_XAnchorEnabled(Handle)); }
                }

                /// <summary>The Z-anchoring option list count.</summary>
                property k64s ZAnchorOptionCount
                {
                    k64s get()           { return (k64s) ::GoProfileTool_ZAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the Z-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The Z-anchoring option at the given index or k32U_MAX if invalid.</returns>
                virtual k32u GetZAnchorOptionAt(k64s index) 
                {
                   return (k32u) ::GoProfileTool_ZAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The Z-anchoring source.</summary>
                property k32s ZAnchor
                {
                    k32s get()           { return (k32s) ::GoProfileTool_ZAnchor(Handle); }
                    void set(k32s source)  { KCheck(::GoProfileTool_SetZAnchor(Handle, source)); }
                }

                /// <summary>A boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.</summary>
                property bool ZAnchorEnabled
                {
                    bool get()           { return KToBool(::GoProfileTool_ZAnchorEnabled(Handle)); }
                }
            };

            /// <summary>Represents a profile area tool.</summary>
            public ref class GoProfileArea : public GoProfileTool
            {
                KDeclareClass(GoProfileArea, GoProfileArea)

                /// <summary>Initializes a new instance of the GoProfileArea class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileArea(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileArea class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileArea(GoSensor^ sensor)
                {
                    ::GoProfileArea handle = kNULL;

                    KCheck(::GoProfileArea_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileArea(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileArea(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileArea handle = kNULL;

                    KCheck(::GoProfileArea_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The profile area baseline.</summary>
                property GoProfileBaseline Baseline
                {
                    GoProfileBaseline get()           { return (GoProfileBaseline) ::GoProfileArea_Baseline(Handle); }
                    void set(GoProfileBaseline type)  { KCheck(::GoProfileArea_SetBaseline(Handle, type)); }
                }

                /// <summary>A boolean value representing whether the profile area baseline is used.</summary>
                property bool BaselineUsed
                {
                    bool get()           { return KToBool(::GoProfileArea_BaselineUsed(Handle)); }
                }

                /// <summary>The reference profile line.</summary>
                property GoProfileLineRegion^ LineRegion
                {
                    GoProfileLineRegion^ get()           { return KToObject<GoProfileLineRegion^>(::GoProfileArea_LineRegion(Handle)); }
                }

                /// <summary>The profile area type.</summary>
                property GoProfileAreaType AreaType
                {
                    GoProfileAreaType get()           { return (GoProfileAreaType) ::GoProfileArea_Type(Handle); }
                    void set(GoProfileAreaType type)  { KCheck(::GoProfileArea_SetType(Handle, type)); }
                }

                /// <summary>A boolean value representing whether the area type is used.</summary>
                property bool TypeUsed
                {
                    bool get()           { return KToBool(::GoProfileArea_TypeUsed(Handle)); }
                }

                /// <summary>The profile region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileArea_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileArea_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileArea_EnableRegion(Handle, enable)); }
                }

                /// <summary>The GoProfileArea area measurement object.</summary>
                property GoProfileAreaArea^ AreaMeasurement
                {
                    GoProfileAreaArea^ get()           { return KToObject<GoProfileAreaArea^>(::GoProfileArea_AreaMeasurement(Handle)); }
                }

                /// <summary>The GoProfileArea centroid X measurement object.</summary>
                property GoProfileAreaCentroidX^ CentroidXMeasurement
                {
                    GoProfileAreaCentroidX^ get()           { return KToObject<GoProfileAreaCentroidX^>(::GoProfileArea_CentroidXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileArea centroid Z measurement object.</summary>
                property GoProfileAreaCentroidZ^ CentroidZMeasurement
                {
                    GoProfileAreaCentroidZ^ get()           { return KToObject<GoProfileAreaCentroidZ^>(::GoProfileArea_CentroidZMeasurement(Handle)); }
                }
                /// <summary>The GoProfileArea center point feature object.</summary>
                property GoProfileAreaCenterPoint^ CenterPoint
                {
                    GoProfileAreaCenterPoint^ get()           { return KToObject<GoProfileAreaCenterPoint^>(::GoProfileArea_CenterPoint(Handle)); }
                }
            };

            /// <summary>Represents a profile bounding box tool.</summary>
            public ref class GoProfileBox : public GoProfileTool
            {
                KDeclareClass(GoProfileBox, GoProfileBox)

                /// <summary>Initializes a new instance of the GoProfileBox class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBox(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileBox class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileBox(GoSensor^ sensor)
                {
                    ::GoProfileBox handle = kNULL;

                    KCheck(::GoProfileBox_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileBox(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileBox(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileBox handle = kNULL;

                    KCheck(::GoProfileBox_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileBox_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileBox_EnableRegion(Handle, enable)); }
                }

                /// <summary>The profile bounding box region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileBox_Region(Handle)); }
                }

                /// <summary>The GoProfileBox X measurement object.</summary>
                property GoProfileBoxX^ XMeasurement
                {
                    GoProfileBoxX^ get()           { return KToObject<GoProfileBoxX^>(::GoProfileBox_XMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox Z measurement object.</summary>
                property GoProfileBoxZ^ ZMeasurement
                {
                    GoProfileBoxZ^ get()           { return KToObject<GoProfileBoxZ^>(::GoProfileBox_ZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox width measurement object.</summary>
                property GoProfileBoxWidth^ WidthMeasurement
                {
                    GoProfileBoxWidth^ get()           { return KToObject<GoProfileBoxWidth^>(::GoProfileBox_WidthMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox height measurement object.</summary>
                property GoProfileBoxHeight^ HeightMeasurement
                {
                    GoProfileBoxHeight^ get()           { return KToObject<GoProfileBoxHeight^>(::GoProfileBox_HeightMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox global X measurement object.</summary>
                property GoProfileBoxGlobalX^ GlobalXMeasurement
                {
                    GoProfileBoxGlobalX^ get()           { return KToObject<GoProfileBoxGlobalX^>(::GoProfileBox_GlobalXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox global Y measurement object.</summary>
                property GoProfileBoxGlobalY^ GlobalYMeasurement
                {
                    GoProfileBoxGlobalY^ get()           { return KToObject<GoProfileBoxGlobalY^>(::GoProfileBox_GlobalYMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox global angle measurement object.</summary>
                property GoProfileBoxGlobalAngle^ GlobalAngleMeasurement
                {
                    GoProfileBoxGlobalAngle^ get()           { return KToObject<GoProfileBoxGlobalAngle^>(::GoProfileBox_GlobalAngleMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBox global angle measurement object.</summary>
                property GoProfileBoundingBoxCornerPoint^ CornerPoint
                {
                    GoProfileBoundingBoxCornerPoint^ get()           { return KToObject<GoProfileBoundingBoxCornerPoint^>(::GoProfileBox_CornerPoint(Handle)); }
                }

                /// <summary>The GoProfileBox global angle measurement object.</summary>
                property GoProfileBoundingBoxCenterPoint^ CenterPoint
                {
                    GoProfileBoundingBoxCenterPoint^ get()           { return KToObject<GoProfileBoundingBoxCenterPoint^>(::GoProfileBox_CenterPoint(Handle)); }
                }
            };

            /// <summary>Represents a profile bridge value tool.</summary>
            public ref class GoProfileBridgeValue : public GoProfileTool
            {
                KDeclareClass(GoProfileBridgeValue, GoProfileBridgeValue)

                /// <summary>Initializes a new instance of the GoProfileBridgeValue class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBridgeValue(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileBridgeValue class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileBridgeValue(GoSensor^ sensor)
                {
                    ::GoProfileBridgeValue handle = kNULL;

                    KCheck(::GoProfileBridgeValue_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileBridgeValue(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileBridgeValue(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileBridgeValue handle = kNULL;

                    KCheck(::GoProfileBridgeValue_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileBridgeValue_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileBridgeValue_EnableRegion(Handle, enable)); }
                }

                /// <summary>The profile bounding box region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileBridgeValue_Region(Handle)); }
                }

                /// <summary>Enables or disables normalization.</summary>
                property bool NormalizeEnabled
                {
                    bool get()           { return KToBool(::GoProfileBridgeValue_NormalizeEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileBridgeValue_EnableNormalize(Handle, enable)); }
                }

                /// <summary>The profile X-Line tool window size percentage.</summary>
                property k64f WindowSize
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_WindowSize(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileBridgeValue_SetWindowSize(Handle, value)); }
                }

                /// <summary>The profile X-Line tool window skip percentage.</summary>
                property k64f WindowSkip
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_WindowSkip(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileBridgeValue_SetWindowSkip(Handle, value)); }
                }

                /// <summary>The profile X-Line tool max invalid percentage.</summary>
                property k64f MaxInvalid
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_MaxInvalid(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileBridgeValue_SetMaxInvalid(Handle, value)); }
                }

                /// <summary>The profile X-Line tool max differential.</summary>
                property k64f MaxDifferential
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_MaxDifferential(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileBridgeValue_SetMaxDifferential(Handle, value)); }
                }

                /// <summary>The profile X-Line tool max differential maximum value limit.</summary>
                property k64f MaxDifferentialLimitMax
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_MaxDifferentialLimitMax(Handle); }
                }

                /// <summary>The profile X-Line tool max differential minimum value limit.</summary>
                /// <remarks>NOTE: A value of 0 will result in an automated differential determination.</remarks>
                property k64f MaxDifferentialLimitMin
                {
                    k64f get()           { return (k64f) ::GoProfileBridgeValue_MaxDifferentialLimitMin(Handle); }
                }

                /// <summary>The GoProfileBridgeValue bridge value measurement object.</summary>
                property GoProfileBridgeValueBridgeValue^ BridgeValueMeasurement
                {
                    GoProfileBridgeValueBridgeValue^ get()           { return KToObject<GoProfileBridgeValueBridgeValue^>(::GoProfileBridgeValue_BridgeValueMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBridgeValue angle measurement object.</summary>
                property GoProfileBridgeValueAngle^ AngleMeasurement
                {
                    GoProfileBridgeValueAngle^ get()           { return KToObject<GoProfileBridgeValueAngle^>(::GoProfileBridgeValue_AngleMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBridgeValue window measurement object.</summary>
                property GoProfileBridgeValueWindow^ WindowMeasurement
                {
                    GoProfileBridgeValueWindow^ get()           { return KToObject<GoProfileBridgeValueWindow^>(::GoProfileBridgeValue_WindowMeasurement(Handle)); }
                }

                /// <summary>The GoProfileBridgeValue standard deviation measurement object.</summary>
                property GoProfileBridgeValueStdDev^ StdDevMeasurement
                {
                    GoProfileBridgeValueStdDev^ get()           { return KToObject<GoProfileBridgeValueStdDev^>(::GoProfileBridgeValue_StdDevMeasurement(Handle)); }
                }
            };

            /// <summary>Represents a profile circle tool.</summary>
            public ref class GoProfileCircle : public GoProfileTool
            {
                KDeclareClass(GoProfileCircle, GoProfileCircle)

                /// <summary>Initializes a new instance of the GoProfileCircle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircle(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileCircle class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileCircle(GoSensor^ sensor)
                {
                    ::GoProfileCircle handle = kNULL;

                    KCheck(::GoProfileCircle_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileCircle(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileCircle(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileCircle handle = kNULL;

                    KCheck(::GoProfileCircle_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The profile region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileCircle_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileCircle_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileCircle_EnableRegion(Handle, enable)); }
                }

                /// <summary>The GoProfileCircle X measurement object.</summary>
                property GoProfileCircleX^ XMeasurement
                {
                    GoProfileCircleX^ get()           { return KToObject<GoProfileCircleX^>(::GoProfileCircle_XMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle Z measurement object.</summary>
                property GoProfileCircleZ^ ZMeasurement
                {
                    GoProfileCircleZ^ get()           { return KToObject<GoProfileCircleZ^>(::GoProfileCircle_ZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle radius measurement object.</summary>
                property GoProfileCircleRadius^ RadiusMeasurement
                {
                    GoProfileCircleRadius^ get()           { return KToObject<GoProfileCircleRadius^>(::GoProfileCircle_RadiusMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle StdDev measurement object.</summary>
                property GoProfileCircleStdDev^ StdDevMeasurement
                {
                    GoProfileCircleStdDev^ get()           { return KToObject<GoProfileCircleStdDev^>(::GoProfileCircle_StdDevMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle min error measurement object.</summary>
                property GoProfileCircleMinError^ MinErrorMeasurement
                {
                    GoProfileCircleMinError^ get()           { return KToObject<GoProfileCircleMinError^>(::GoProfileCircle_MinErrorMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle min error X measurement object.</summary>
                property GoProfileCircleMinErrorX^ MinErrorXMeasurement
                {
                    GoProfileCircleMinErrorX^ get()           { return KToObject<GoProfileCircleMinErrorX^>(::GoProfileCircle_MinErrorXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle min error Z measurement object.</summary>
                property GoProfileCircleMinErrorZ^ MinErrorZMeasurement
                {
                    GoProfileCircleMinErrorZ^ get()           { return KToObject<GoProfileCircleMinErrorZ^>(::GoProfileCircle_MinErrorZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle max error measurement object.</summary>
                property GoProfileCircleMaxError^ MaxErrorMeasurement
                {
                    GoProfileCircleMaxError^ get()           { return KToObject<GoProfileCircleMaxError^>(::GoProfileCircle_MaxErrorMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle max error X measurement object.</summary>
                property GoProfileCircleMaxErrorX^ MaxErrorXMeasurement
                {
                    GoProfileCircleMaxErrorX^ get()           { return KToObject<GoProfileCircleMaxErrorX^>(::GoProfileCircle_MaxErrorXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle max error Z measurement object.</summary>
                property GoProfileCircleMaxErrorZ^ MaxErrorZMeasurement
                {
                    GoProfileCircleMaxErrorZ^ get()           { return KToObject<GoProfileCircleMaxErrorZ^>(::GoProfileCircle_MaxErrorZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileCircle radius measurement object.</summary>
                property GoProfileCircleCenterPoint^ CenterPoint
                {
                    GoProfileCircleCenterPoint^ get()           { return KToObject<GoProfileCircleCenterPoint^>(::GoProfileCircle_CenterPoint(Handle)); }
                }

            };

            /// <summary>Represents a profile dimension tool.</summary>
            public ref class GoProfileDim : public GoProfileTool
            {
                KDeclareClass(GoProfileDim, GoProfileDim)

                /// <summary>Initializes a new instance of the GoProfileDim class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDim(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileDim class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileDim(GoSensor^ sensor)
                {
                    ::GoProfileDim handle = kNULL;

                    KCheck(::GoProfileDim_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileDim(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileDim(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileDim handle = kNULL;

                    KCheck(::GoProfileDim_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The reference profile feature.</summary>
                property GoProfileFeature^ RefFeature
                {
                    GoProfileFeature^ get()           { return KToObject<GoProfileFeature^>(::GoProfileDim_RefFeature(Handle)); }
                }

                /// <summary>The non-reference profile feature.</summary>
                property GoProfileFeature^ Feature
                {
                    GoProfileFeature^ get()           { return KToObject<GoProfileFeature^>(::GoProfileDim_Feature(Handle)); }
                }

                /// <summary>The GoProfileDim width measurement object.</summary>
                property GoProfileDimWidth^ WidthMeasurement
                {
                    GoProfileDimWidth^ get()           { return KToObject<GoProfileDimWidth^>(::GoProfileDim_WidthMeasurement(Handle)); }
                }

                /// <summary>The GoProfileDim height measurement object.</summary>
                property GoProfileDimHeight^ HeightMeasurement
                {
                    GoProfileDimHeight^ get()           { return KToObject<GoProfileDimHeight^>(::GoProfileDim_HeightMeasurement(Handle)); }
                }

                /// <summary>The GoProfileDim distance measurement object.</summary>
                property GoProfileDimDistance^ DistanceMeasurement
                {
                    GoProfileDimDistance^ get()           { return KToObject<GoProfileDimDistance^>(::GoProfileDim_DistanceMeasurement(Handle)); }
                }

                /// <summary>The GoProfileDim center X measurement object.</summary>
                property GoProfileDimCenterX^ CenterXMeasurement
                {
                    GoProfileDimCenterX^ get()           { return KToObject<GoProfileDimCenterX^>(::GoProfileDim_CenterXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileDim center Z measurement object.</summary>
                property GoProfileDimCenterZ^ CenterZMeasurement
                {
                    GoProfileDimCenterZ^ get()           { return KToObject<GoProfileDimCenterZ^>(::GoProfileDim_CenterZMeasurement(Handle)); }
                }
                /// <summary>The GoProfileDim center Z measurement object.</summary>
                property GoProfileDimensionCenterPoint^ CenterPoint
                {
                    GoProfileDimensionCenterPoint^ get()           { return KToObject<GoProfileDimensionCenterPoint^>(::GoProfileDim_CenterPoint(Handle)); }
                }

            };

            /// <summary>Represents a profile groove tool.</summary>
            public ref class GoProfileGroove : public GoProfileTool
            {
                KDeclareClass(GoProfileGroove, GoProfileGroove)

                /// <summary>Initializes a new instance of the GoProfileGroove class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileGroove(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileGroove class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileGroove(GoSensor^ sensor)
                {
                    ::GoProfileGroove handle = kNULL;

                    KCheck(::GoProfileGroove_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileGroove(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileGroove(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileGroove handle = kNULL;

                    KCheck(::GoProfileGroove_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Adds an additional profile groove tool measurement.</summary>
                /// <param name="type">The measurement type to add. It must be a valid profile groove tool type.</param>
                /// <returns>The new GoMeasurement.</returns>
                GoMeasurement^ AddMeasurement(GoMeasurementType type)
                {
                   ::GoMeasurement measurement;

                   KCheck(::GoProfileGroove_AddMeasurement(Handle, type, &measurement));

                   return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>Adds an additional profile groove tool measurement.</summary>
                /// <param name="index">The index with which to remove a measurement.</param>
                void RemoveMeasurement(k64s index) override
                {
                   KCheck(::GoProfileGroove_RemoveMeasurement(Handle, (kSize)index));
                }

                /// <summary>The measurement count for the given tool.</summary>
                property k64s MeasurementCount
                {
                    virtual k64s get() override           { return (k64s) ::GoProfileGroove_MeasurementCount(Handle); }
                }

                /// <summary>Gets a measurement object at the given index.</summary>
                /// <param name="index">The index with which to return a measurement object.</param>
                /// <returns>A profile groove tool measurement or null if the index is invalid.</returns>
                virtual GoMeasurement^ GetMeasurement(k64s index) override
                {
                    ::GoMeasurement measurement = ::GoProfileGroove_MeasurementAt(Handle, (kSize)index);

                    if (kIsNull(measurement)) { return nullptr; }

                    return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>The groove determination shape.</summary>
                property GoProfileGrooveShape Shape
                {
                    GoProfileGrooveShape get()           { return (GoProfileGrooveShape) ::GoProfileGroove_Shape(Handle); }
                    void set(GoProfileGrooveShape shape)  { KCheck(::GoProfileGroove_SetShape(Handle, shape)); }
                }

                /// <summary>The groove depth minimum.</summary>
                property k64f MinDepth
                {
                    k64f get()           { return (k64f) ::GoProfileGroove_MinDepth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileGroove_SetMinDepth(Handle, value)); }
                }

                /// <summary>The groove width maximum.</summary>
                property k64f MaxWidth
                {
                    k64f get()           { return (k64f) ::GoProfileGroove_MaxWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileGroove_SetMaxWidth(Handle, value)); }
                }

                /// <summary>The groove width minimum.</summary>
                property k64f MinWidth
                {
                    k64f get()           { return (k64f) ::GoProfileGroove_MinWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileGroove_SetMinWidth(Handle, value)); }
                }

                /// <summary>The profile region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileGroove_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileGroove_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileGroove_EnableRegion(Handle, enable)); }
                }
            };

            /// <summary>Represents a profile intersect tool.</summary>
            public ref class GoProfileIntersect : public GoProfileTool
            {
                KDeclareClass(GoProfileIntersect, GoProfileIntersect)

                /// <summary>Initializes a new instance of the GoProfileIntersect class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileIntersect(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileIntersect class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileIntersect(GoSensor^ sensor)
                {
                    ::GoProfileIntersect handle = kNULL;

                    KCheck(::GoProfileIntersect_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileIntersect(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileIntersect(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileIntersect handle = kNULL;

                    KCheck(::GoProfileIntersect_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The reference profile line type.</summary>
                property GoProfileBaseline RefLineType
                {
                    GoProfileBaseline get()           { return (GoProfileBaseline) ::GoProfileIntersect_RefLineType(Handle); }
                    void set(GoProfileBaseline type)  { KCheck(::GoProfileIntersect_SetRefLineType(Handle, type)); }
                }

                /// <summary>The reference profile line.</summary>
                property GoProfileLineRegion^ RefLine
                {
                    GoProfileLineRegion^ get()           { return KToObject<GoProfileLineRegion^>(::GoProfileIntersect_RefLine(Handle)); }
                }

                /// <summary>The non-reference profile line.</summary>
                property GoProfileLineRegion^ Line
                {
                    GoProfileLineRegion^ get()           { return KToObject<GoProfileLineRegion^>(::GoProfileIntersect_Line(Handle)); }
                }

                /// <summary>The GoProfileIntersect X measurement object.</summary>
                property GoProfileIntersectX^ XMeasurement
                {
                    GoProfileIntersectX^ get()           { return KToObject<GoProfileIntersectX^>(::GoProfileIntersect_XMeasurement(Handle)); }
                }

                /// <summary>The GoProfileIntersect Z measurement object.</summary>
                property GoProfileIntersectZ^ ZMeasurement
                {
                    GoProfileIntersectZ^ get()           { return KToObject<GoProfileIntersectZ^>(::GoProfileIntersect_ZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileIntersect angle measurement object.</summary>
                property GoProfileIntersectAngle^ AngleMeasurement
                {
                    GoProfileIntersectAngle^ get()           { return KToObject<GoProfileIntersectAngle^>(::GoProfileIntersect_AngleMeasurement(Handle)); }
                }

                /// <summary>The GoProfileIntersect angle measurement object.</summary>
                property GoProfileIntersectIntersectPoint^ Point
                {
                    GoProfileIntersectIntersectPoint^ get()           { return KToObject<GoProfileIntersectIntersectPoint^>(::GoProfileIntersect_PointFeature(Handle)); }
                }

                /// <summary>The GoProfileIntersect angle measurement object.</summary>
                property GoProfileIntersectLineFeature^ LineFeature
                {
                    GoProfileIntersectLineFeature^ get()           { return KToObject<GoProfileIntersectLineFeature^>(::GoProfileIntersect_LineFeature(Handle)); }
                }
                /// <summary>The GoProfileIntersect angle measurement object.</summary>
                property GoProfileIntersectBaseLineFeature^ BaseLineFeature
                {
                    GoProfileIntersectBaseLineFeature^ get()           { return KToObject<GoProfileIntersectBaseLineFeature^>(::GoProfileIntersect_BaseLineFeature(Handle)); }
                }
            };

            /// <summary>Represents a profile line tool.</summary>
            public ref class GoProfileLine : public GoProfileTool
            {
                KDeclareClass(GoProfileLine, GoProfileLine)

                /// <summary>Initializes a new instance of the GoProfileLine class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLine(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileLine class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileLine(GoSensor^ sensor)
                {
                    ::GoProfileLine handle = kNULL;

                    KCheck(::GoProfileLine_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileLine(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileLine(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileLine handle = kNULL;

                    KCheck(::GoProfileLine_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The measurement region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileLine_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileLine_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileLine_EnableRegion(Handle, enable)); }
                }

                /// <summary>The fitting regions.</summary>
                property GoProfileLineRegion^ FittingRegions
                {
                    GoProfileLineRegion^ get()           { return KToObject<GoProfileLineRegion^>(::GoProfileLine_FittingRegions(Handle)); }
                }

                property bool FittingRegionsEnabled
                {
                    bool get()  { return KToBool(::GoProfileLine_FittingRegionsEnabled(Handle)); }
                    void set(bool enable) { KCheck(::GoProfileLine_EnableFittingRegions(Handle, enable)); }
                }

                /// <summary>The GoProfileLine standard deviation measurement object.</summary>
                property GoProfileLineStdDev^ StdDevMeasurement
                {
                    GoProfileLineStdDev^ get()           { return KToObject<GoProfileLineStdDev^>(::GoProfileLine_StdDevMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine maximum error measurement object.</summary>
                property GoProfileLineMaxError^ MaxErrorMeasurement
                {
                    GoProfileLineMaxError^ get()           { return KToObject<GoProfileLineMaxError^>(::GoProfileLine_MaxErrorMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineMinError^ MinErrorMeasurement
                {
                    GoProfileLineMinError^ get()           { return KToObject<GoProfileLineMinError^>(::GoProfileLine_MinErrorMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineOffset^ OffsetMeasurement
                {
                    GoProfileLineOffset^ get()           { return KToObject<GoProfileLineOffset^>(::GoProfileLine_OffsetMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineAngle^ AngleMeasurement
                {
                    GoProfileLineAngle^ get()           { return KToObject<GoProfileLineAngle^>(::GoProfileLine_AngleMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineMinErrorX^ MinErrorXMeasurement
                {
                    GoProfileLineMinErrorX^ get()           { return KToObject<GoProfileLineMinErrorX^>(::GoProfileLine_MinErrorXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineMinErrorZ^ MinErrorZMeasurement
                {
                    GoProfileLineMinErrorZ^ get()           { return KToObject<GoProfileLineMinErrorZ^>(::GoProfileLine_MinErrorZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineMaxErrorX^ MaxErrorXMeasurement
                {
                    GoProfileLineMaxErrorX^ get()           { return KToObject<GoProfileLineMaxErrorX^>(::GoProfileLine_MaxErrorXMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine minimum error measurement object.</summary>
                property GoProfileLineMaxErrorZ^ MaxErrorZMeasurement
                {
                    GoProfileLineMaxErrorZ^ get()           { return KToObject<GoProfileLineMaxErrorZ^>(::GoProfileLine_MaxErrorZMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine percentile measurement object.</summary>
                property GoProfileLinePercentile^ PercentileMeasurement
                {
                    GoProfileLinePercentile^ get()           { return KToObject<GoProfileLinePercentile^>(::GoProfileLine_PercentileMeasurement(Handle)); }
                }

                /// <summary>The GoProfileLine percentile measurement object.</summary>
                property GoProfileLineLine^ Line
                {
                    GoProfileLineLine^ get()           { return KToObject<GoProfileLineLine^>(::GoProfileLine_Line(Handle)); }
                }

                /// <summary>The GoProfileLine percentile measurement object.</summary>
                property GoProfileLineMinErrorPoint^ MinErrorPoint
                {
                    GoProfileLineMinErrorPoint^ get()           { return KToObject<GoProfileLineMinErrorPoint^>(::GoProfileLine_MinErrorPoint(Handle)); }
                }

                /// <summary>The GoProfileLine percentile measurement object.</summary>
                property GoProfileLineMaxErrorPoint^ MaxErrorPoint
                {
                    GoProfileLineMaxErrorPoint^ get()           { return KToObject<GoProfileLineMaxErrorPoint^>(::GoProfileLine_MaxErrorPoint(Handle)); }
                }
            };

            /// <summary>Represents a profile panel tool.</summary>
            public ref class GoProfilePanel : public GoProfileTool
            {
                KDeclareClass(GoProfilePanel, GoProfilePanel)

                /// <summary>Initializes a new instance of the GoProfilePanel class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePanel(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfilePanel class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfilePanel(GoSensor^ sensor)
                {
                    ::GoProfilePanel handle = kNULL;

                    KCheck(::GoProfilePanel_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfilePanel(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfilePanel(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfilePanel handle = kNULL;

                    KCheck(::GoProfilePanel_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The maximum gap width.</summary>
                property k64f MaxGapWidth
                {
                    k64f get()           { return (k64f) ::GoProfilePanel_MaxGapWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfilePanel_SetMaxGapWidth(Handle, value)); }
                }

                /// <summary>The reference edge side.</summary>
                property GoProfilePanelSide RefEdgeSide
                {
                    GoProfilePanelSide get()           { return (GoProfilePanelSide) ::GoProfilePanel_RefEdgeSide(Handle); }
                    void set(GoProfilePanelSide side)  { KCheck(::GoProfilePanel_SetRefEdgeSide(Handle, side)); }
                }

                /// <summary>The left profile edge.</summary>
                property GoProfileEdge^ LeftEdge
                {
                    GoProfileEdge^ get()           { return KToObject<GoProfileEdge^>(::GoProfilePanel_LeftEdge(Handle)); }
                }

                /// <summary>The right profile edge.</summary>
                property GoProfileEdge^ Right
                {
                    GoProfileEdge^ get()           { return KToObject<GoProfileEdge^>(::GoProfilePanel_RightEdge(Handle)); }
                }

                /// <summary>The GoProfilePanel gap measurement object.</summary>
                property GoProfilePanelGap^ GapMeasurement
                {
                    GoProfilePanelGap^ get()           { return KToObject<GoProfilePanelGap^>(::GoProfilePanel_GapMeasurement(Handle)); }
                }

                /// <summary>The GoProfilePanel flush measurement object.</summary>
                property GoProfilePanelFlush^ FlushMeasurement
                {
                    GoProfilePanelFlush^ get()           { return KToObject<GoProfilePanelFlush^>(::GoProfilePanel_FlushMeasurement(Handle)); }
                }

                /// <summary>Returns a GoProfilePanel Left Flush X measurement object.</summary>
                property GoProfilePanelLeftFlushX^ LeftFlushXMeasurement
                {
                    GoProfilePanelLeftFlushX^ get()  { return KToObject<GoProfilePanelLeftFlushX^>(::GoProfilePanel_LeftFlushXMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Left Flush Z measurement object.</summary>
                property GoProfilePanelLeftFlushZ^ LeftFlushZMeasurement
                {
                    GoProfilePanelLeftFlushZ^ get()  { return KToObject<GoProfilePanelLeftFlushZ^>(::GoProfilePanel_LeftFlushZMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Left Gap X measurement object.</summary>
                property GoProfilePanelLeftGapX^ LeftGapXMeasurement
                {
                    GoProfilePanelLeftGapX^ get()  { return KToObject<GoProfilePanelLeftGapX^>(::GoProfilePanel_LeftGapXMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Left Gap Z measurement object.</summary>
                property GoProfilePanelLeftGapZ^ LeftGapZMeasurement
                {
                    GoProfilePanelLeftGapZ^ get()  { return KToObject<GoProfilePanelLeftGapZ^>(::GoProfilePanel_LeftGapZMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Left Surface Angle measurement object.</summary>
                property GoProfilePanelLeftSurfaceAngle^ LeftSurfaceAngleMeasurement
                {
                    GoProfilePanelLeftSurfaceAngle^ get()  { return KToObject<GoProfilePanelLeftSurfaceAngle^>(::GoProfilePanel_LeftSurfaceAngleMeasurement(Handle)); }
                }
                /// <summaryReturns a GoProfilePanel Right Flush Z measurement object.></summary>
                property GoProfilePanelRightFlushZ^ RightFlushZMeasurement
                {
                    GoProfilePanelRightFlushZ^ get()  { return KToObject<GoProfilePanelRightFlushZ^>(::GoProfilePanel_RightFlushZMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Right Gap X measurement object.</summary>
                property GoProfilePanelRightGapX^ RightGapXMeasurement
                {
                    GoProfilePanelRightGapX^ get()  { return KToObject<GoProfilePanelRightGapX^>(::GoProfilePanel_RightGapXMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Right Gap Z measurement object.</summary>
                property GoProfilePanelRightGapZ^ RightGapZMeasurement
                {
                    GoProfilePanelRightGapZ^ get()  { return KToObject<GoProfilePanelRightGapZ^>(::GoProfilePanel_RightGapZMeasurement(Handle)); }
                }
                /// <summary>Returns a GoProfilePanel Right Surface Angle measurement object.</summary>
                property GoProfilePanelRightSurfaceAngle^ RightSurfaceAngleMeasurement
                {
                    GoProfilePanelRightSurfaceAngle^ get()  { return KToObject<GoProfilePanelRightSurfaceAngle^>(::GoProfilePanel_RightSurfaceAngleMeasurement(Handle)); }
                }
            };

            /// <summary>Represents a profile panel tool.</summary>
            public ref class GoProfileRoundCorner : public GoProfileTool
            {
                KDeclareClass(GoProfileRoundCorner, GoProfileRoundCorner)

                /// <summary>Initializes a new instance of the GoProfileRoundCorner class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileRoundCorner(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileRoundCorner class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileRoundCorner(GoSensor^ sensor)
                {
                    ::GoProfileRoundCorner handle = kNULL;

                    KCheck(::GoProfileRoundCorner_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileRoundCorner(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileRoundCorner(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileRoundCorner handle = kNULL;

                    KCheck(::GoProfileRoundCorner_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Returns a GoProfileRoundCorner X measurement object.</summary>
                property GoProfileRoundCornerX^ XMeasurement
                {
                    GoProfileRoundCornerX^ get() { return KToObject<GoProfileRoundCornerX^>(::GoProfileRoundCorner_XMeasurement(Handle)); }
                }

                /// <summary>Returns a GoProfileRoundCorner Z measurement object.</summary>
                property GoProfileRoundCornerZ^ ZMeasurement
                {
                    GoProfileRoundCornerZ^ get() { return KToObject<GoProfileRoundCornerZ^>(::GoProfileRoundCorner_ZMeasurement(Handle)); }
                }

                /// <summary>Returns a GoProfileRoundCorner Angle measurement object.</summary>
                property GoProfileRoundCornerAngle^ AngleMeasurement
                {
                    GoProfileRoundCornerAngle^ get() { return KToObject<GoProfileRoundCornerAngle^>(::GoProfileRoundCorner_AngleMeasurement(Handle)); }
                }

                /// <summary>Edge.</summary>
                property GoProfileEdge^ Edge
                {
                    GoProfileEdge^ get()            { return KToObject<GoProfileEdge^>(::GoProfileRoundCorner_Edge(Handle)); }
                    void set(GoProfileEdge^ edge)   { ::GoProfileRoundCorner_SetEdge(Handle, KToHandle(edge)); }
                }

                /// <summary>Reference direction.</summary>
                property GoProfileRoundCornerDirection ReferenceDirection
                {
                    GoProfileRoundCornerDirection get()                 { return ::GoProfileRoundCorner_RefDirection(Handle); }
                    void set(GoProfileRoundCornerDirection direction)   { ::GoProfileRoundCorner_SetRefDirection(Handle, direction); }
                }

                /// <summary>Gets the round corner point.</summary>
                property GoProfileRoundCornerPoint^ Point
                {
                    GoProfileRoundCornerPoint^ get()                 { return KToObject<GoProfileRoundCornerPoint^>(::GoProfileRoundCorner_Point(Handle)); }
                }

                /// <summary>Gets the round corner edge point.</summary>
                property GoProfileRoundCornerPoint^ EdgePoint
                {
                    GoProfileRoundCornerPoint^ get()  { return KToObject<GoProfileRoundCornerPoint^>(::GoProfileRoundCorner_EdgePoint(Handle)); }
                }
                
                /// <summary>Gets the round corner center point.</summary>
                property GoProfileRoundCornerCenterPoint^ CenterPoint
                {
                    GoProfileRoundCornerCenterPoint^ get()           { return KToObject<GoProfileRoundCornerCenterPoint^>(::GoProfileRoundCorner_CenterPoint(Handle)); }
                }
            };

            /// <summary>Represents a profile position tool.</summary>
            public ref class GoProfilePosition : public GoProfileTool
            {
                KDeclareClass(GoProfilePosition, GoProfilePosition)

                    /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get() { return KToBool(::GoProfilePosition_RegionEnabled(Handle)); }
                    void set(bool enable) { KCheck(::GoProfilePosition_EnableRegion(Handle, enable)); }
                }
                /// <summary>Initializes a new instance of the GoProfilePosition class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePosition(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfilePosition class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfilePosition(GoSensor^ sensor)
                {
                    ::GoProfilePosition handle = kNULL;

                    KCheck(::GoProfilePosition_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfilePosition(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfilePosition(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfilePosition handle = kNULL;

                    KCheck(::GoProfilePosition_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The profile feature.</summary>
                property GoProfileFeature^ Feature
                {
                    GoProfileFeature^ get()           { return KToObject<GoProfileFeature^>(::GoProfilePosition_Feature(Handle)); }
                }

                /// <summary>The GoProfilePosition X measurement object.</summary>
                property GoProfilePositionX^ XMeasurement
                {
                    GoProfilePositionX^ get()           { return KToObject<GoProfilePositionX^>(::GoProfilePosition_XMeasurement(Handle)); }
                }

                /// <summary>The GoProfilePosition Z measurement object.</summary>
                property GoProfilePositionZ^ ZMeasurement
                {
                    GoProfilePositionZ^ get()           { return KToObject<GoProfilePositionZ^>(::GoProfilePosition_ZMeasurement(Handle)); }
                }
                /// <summary>The GoProfilePosition Z measurement object.</summary>
                property GoProfilePositionPoint^ Point
                {
                    GoProfilePositionPoint^ get()           { return KToObject<GoProfilePositionPoint^>(::GoProfilePosition_Point(Handle)); }
                }
            };

            /// <summary>Represents a profile strip tool.</summary>
            public ref class GoProfileStrip : public GoProfileTool
            {
                KDeclareClass(GoProfileStrip, GoProfileStrip)

                /// <summary>Initializes a new instance of the GoProfileStrip class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileStrip(IntPtr handle)
                    : GoProfileTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileStrip class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileStrip(GoSensor^ sensor)
                {
                    ::GoProfileStrip handle = kNULL;

                    KCheck(::GoProfileStrip_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileStrip(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileStrip(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileStrip handle = kNULL;

                    KCheck(::GoProfileStrip_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The strip base type.</summary>
                property GoProfileStripBaseType BaseType
                {
                    GoProfileStripBaseType get()           { return (GoProfileStripBaseType) ::GoProfileStrip_BaseType(Handle); }
                    void set(GoProfileStripBaseType type)  { KCheck(::GoProfileStrip_SetBaseType(Handle, type)); }
                }

                /// <summary>The left edge value.</summary>
                property GoProfileStripEdgeType LeftEdge
                {
                    GoProfileStripEdgeType get()           { return (GoProfileStripEdgeType) ::GoProfileStrip_LeftEdge(Handle); }
                    void set(GoProfileStripEdgeType type)  { KCheck(::GoProfileStrip_SetLeftEdge(Handle, type)); }
                }

                /// <summary>The right edge value.</summary>
                property GoProfileStripEdgeType RightEdge
                {
                    GoProfileStripEdgeType get()           { return (GoProfileStripEdgeType) ::GoProfileStrip_RightEdge(Handle); }
                    void set(GoProfileStripEdgeType type)  { KCheck(::GoProfileStrip_SetRightEdge(Handle, type)); }
                }

                /// <summary>Enables or disables tilt.</summary>
                property bool TiltEnabled
                {
                    bool get()           { return KToBool(::GoProfileStrip_TiltEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileStrip_EnableTilt(Handle, enable)); }
                }

                /// <summary>The support width (in mm).</summary>
                property k64f SupportWidth
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_SupportWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileStrip_SetSupportWidth(Handle, value)); }
                }

                /// <summary>The minimum permissible transition width (in mm).</summary>
                property k64f TransitionWidthLimitMin
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_TransitionWidthLimitMin(Handle); }
                }

                /// <summary>The maximum permissible transition width (in mm).</summary>
                property k64f TransitionWidthLimitMax
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_TransitionWidthLimitMax(Handle); }
                }

                /// <summary>The transition width (in mm).</summary>
                property k64f TransitionWidth
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_TransitionWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileStrip_SetTransitionWidth(Handle, value)); }
                }

                /// <summary>The minimum width (in mm).</summary>
                property k64f MinWidth
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_MinWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileStrip_SetMinWidth(Handle, value)); }
                }

                /// <summary>The minimum height (in mm).</summary>
                property k64f MinHeight
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_MinHeight(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileStrip_SetMinHeight(Handle, value)); }
                }

                /// <summary>The maximum void width (in mm).</summary>
                property k64f MaxVoidWidth
                {
                    k64f get()           { return (k64f) ::GoProfileStrip_MaxVoidWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileStrip_SetMaxVoidWidth(Handle, value)); }
                }

                /// <summary>The profile strip tool region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileStrip_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileStrip_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileStrip_EnableRegion(Handle, enable)); }
                }

                /// <summary>Adds an additional profile strip tool measurement.</summary>
                /// <param name="type">The measurement type to add. It must be a valid profile strip tool type.</param>
                /// <returns>The new GoMeasurement.</returns>
                GoMeasurement^ AddMeasurement(GoMeasurementType type)
                {
                   ::GoMeasurement measurement;

                   KCheck(::GoProfileStrip_AddMeasurement(Handle, type, &measurement));

                   return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>Adds an additional profile groove tool measurement.</summary>
                /// <param name="index">The index with which to remove a measurement.</param>
                void RemoveMeasurement(k64s index) override 
                {
                   KCheck(::GoProfileStrip_RemoveMeasurement(Handle, (kSize)index));
                }

                /// <summary>The measurement count for the given tool.</summary>
                property k64s MeasurementCount
                {
                    virtual k64s get() override { return (k64s) ::GoProfileStrip_MeasurementCount(Handle); }
                }

                /// <summary>Gets a measurement object at the given index.</summary>
                /// <param name="index">The index with which to return a measurement object.</param>
                /// <returns>A profile strip tool measurement or null if the index is invalid.</returns>
                virtual GoMeasurement^ GetMeasurement(k64s index) override
                {
                    ::GoMeasurement measurement = ::GoProfileStrip_MeasurementAt(Handle, (kSize)index);

                    if (kIsNull(measurement)) { return nullptr; }

                    return GoMeasurements::GetMeasurementInstance(measurement);
                }
            };

            private ref class GoProfileTools abstract sealed
            {
            internal:
                static GoProfileTool^ GetProfileToolInstance(::GoProfileTool tool)
                {
                    if (kObject_Is(tool, kTypeOf(GoProfileArea))) { return KToObject<GoProfileArea^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileBox))) { return KToObject<GoProfileBox^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileBridgeValue))) { return KToObject<GoProfileBridgeValue^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileCircle))) { return KToObject<GoProfileCircle^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileDim))) { return KToObject<GoProfileDim^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileGroove))) { return KToObject<GoProfileGroove^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileIntersect))) { return KToObject<GoProfileIntersect^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileLine))) { return KToObject<GoProfileLine^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfilePanel))) { return KToObject<GoProfilePanel^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileRoundCorner))) { return KToObject<GoProfileRoundCorner^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfilePosition))) { return KToObject<GoProfilePosition^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileStrip))) { return KToObject<GoProfileStrip^>(tool); }

                    return nullptr;
                }
            };
        }
    }
}

#endif
