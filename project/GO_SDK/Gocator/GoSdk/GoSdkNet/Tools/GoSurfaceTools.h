//
// GoSurfaceTools.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_SURFACE_TOOLS_H
#define GO_SDK_NET_SURFACE_TOOLS_H

#include <GoSdk/Tools/GoSurfaceTools.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/tools/GoFeature.h>
#include <GoSdkNet/tools/GoFeatures.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a base surface tool.</summary>
            public ref class GoSurfaceTool abstract : public GoTool
            {
                KDeclareClass(GoSurfaceTool, GoSurfaceTool)

            public:
                /// <summary>Default GoSurfaceTool constructor.</summary>
                GoSurfaceTool() {}

                /// <summary>Initializes a new instance of the GoSurfaceTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceTool(IntPtr handle)
                    : GoTool(handle)
                {}

                /// <summary>The data stream.</summary>
                /// <remarks>Note that stream validation will only occur if tool is in the tool options list.</remarks>
                property GoDataStream Stream
                {
                    GoDataStream get()           { return (GoDataStream) ::GoSurfaceTool_Stream(Handle); }
                    void set(GoDataStream stream)  { KCheck(::GoSurfaceTool_SetStream(Handle, ::GoDataStream(stream))); }
                }

                /// <summary>The data stream option list count.</summary>
                /// <returns>The current surface tool data stream option list count.</returns>
                property k64s StreamOptionCount
                {
                    k64s get()           { return (k64s) ::GoSurfaceTool_StreamOptionCount(Handle); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The surface tool data stream option at the given index, or k32U_MAX if an invalid index is given.</returns>
                GoDataStream GetStreamOptionAt(k64s index)
                {
                   return (GoDataStream) ::GoSurfaceTool_StreamOptionAt(Handle, (kSize)index);
                }

                /// <summary>Gets/Sets the data source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource)::GoSurfaceTool_Source(Handle); }
                    void set(GoDataSource value) { KCheck(::GoSurfaceTool_SetSource(Handle, value)); }
                }

                /// <summary>Gets/Sets the current X-anchoring source.</summary>
                property k32s XAnchor
                {
                    k32s get() { return (k32s)::GoSurfaceTool_XAnchor(Handle); }
                    void set(k32s value) { KCheck(::GoSurfaceTool_SetXAnchor(Handle, value)); }
                }

                /// <summary>Gets/Sets the current Y-anchoring source.</summary>
                property k32s YAnchor
                {
                    k32s get() { return (k32s)::GoSurfaceTool_YAnchor(Handle); }
                    void set(k32s value) { KCheck(::GoSurfaceTool_SetYAnchor(Handle, value)); }
                }

                /// <summary>Gets/Sets the current Z-anchoring source.</summary>
                property k32s ZAnchor
                {
                    k32s get() { return (k32s)::GoSurfaceTool_ZAnchor(Handle); }
                    void set(k32s value) { KCheck(::GoSurfaceTool_SetZAnchor(Handle, value)); }
                }

                /// <summary>Gets/Sets the ZAngle-anchoring source.</summary>
                property k32s ZAngleAnchor
                {
                    k32s get()  { return (k32s)::GoSurfaceTool_ZAngleAnchor(Handle); }
                    void set(k32s value)  { KCheck(::GoSurfaceTool_SetZAngleAnchor(Handle, value)); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the source option to retrieve.</param>
                /// <returns>A source option.</returns>
                k32u GetSourceOption(k64s index) 
                {
                    return (k32u)::GoSurfaceTool_SourceOptionAt(Handle, (kSize)index); 
                }

                /// <summary>Gets the data source option list count.</summary>
                property kSize SourceOptionCount
                {
                    kSize get()  { return (kSize)::GoSurfaceTool_SourceOptionCount(Handle); }
                }

                /// <summary>Returns a boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.</summary>
                property bool XAnchorEnabled
                {
                    bool get()  { return KToBool(::GoSurfaceTool_XAnchorEnabled(Handle)); }
                }

                /// <summary>Gets the X-anchoring option list count.</summary>
                property kSize XAnchorOptionCount
                {
                    kSize get()  { return (kSize)::GoSurfaceTool_XAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the X-anchoring option at the given index.</summary>
                /// <param name="index">The index of the X-anchoring option to retrieve.</param>
                /// <returns>A X-anchoring option.</returns>
                k32u GetXAnchorOptionAt(k64s index) 
                {
                     return (k32u)::GoSurfaceTool_XAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>Returns a boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.</summary>
                property bool YAnchorEnabled
                {
                    bool get()  { return KToBool(::GoSurfaceTool_YAnchorEnabled(Handle)); }
                }

                /// <summary>Gets the Y-anchoring option list count.</summary>
                property kSize YAnchorOptionCount
                {
                    kSize get()  { return (kSize)::GoSurfaceTool_YAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the Y-anchoring option at the given index.</summary>
                /// <param name="index">The index of the Y-anchoring option to retrieve.</param>
                /// <returns>A Y-anchoring option.</returns>
                k32u GetYAnchorOptionAt(k64s index) 
                {
                     return (k32u)::GoSurfaceTool_YAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>Returns a boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.</summary>
                property bool ZAnchorEnabled
                {
                    bool get()  { return KToBool(::GoSurfaceTool_ZAnchorEnabled(Handle)); }
                }

                /// <summary>Gets the Z-anchoring option list count.</summary>
                property kSize ZAnchorOptionCount
                {
                    kSize get()  { return (kSize)::GoSurfaceTool_ZAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the Z-anchoring option at the given index.</summary>
                /// <param name="index">The index of the Z-anchoring option to retrieve.</param>
                /// <returns>A Z-anchoring option.</returns>
                k32u GetZAnchorOptionAt(k64s index) 
                {
                     return (k32u)::GoSurfaceTool_ZAnchorOptionAt(Handle, (kSize)index); 
                }

                /// <summary>Returns a boolean value representing whether or not a valid ZAngle - anchoring source has been set for ZAngle - anchoring.</summary>
                property bool ZAngleAnchorEnabled
                {
                    bool get()  { return KToBool(::GoSurfaceTool_ZAngleAnchorEnabled(Handle)); }
                }

                /// <summary>Gets the Z-anchoring option list count.</summary>
                property kSize ZAngleAnchorOptionCount
                {
                    kSize get()  { return (kSize)::GoSurfaceTool_ZAngleAnchorOptionCount(Handle); }
                }

                /// <summary></summary>
                /// <param name="index">The index of the Z-anchoring option to retrieve.</param>
                /// <returns>A Z-anchoring option.</returns>
                k32u GetZAngleAnchorOptionAt(k64s index) 
                {
                     return (k32u)::GoSurfaceTool_ZAngleAnchorOptionAt(Handle, (kSize)index);
                }
            };

            /// <summary>Represents a surface bounding box tool.</summary>
            public ref class GoSurfaceBox : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceBox, GoSurfaceBox)

                /// <summary>Initializes a new instance of the GoSurfaceBox class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBox(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceBox class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceBox(GoSensor^ sensor)
                {
                    ::GoSurfaceBox handle = kNULL;

                    KCheck(::GoSurfaceBox_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceBox(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceBox(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceBox handle = kNULL;

                    KCheck(::GoSurfaceBox_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables Z-rotation.</summary>
                property bool ZRotationEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceBox_ZRotationEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceBox_EnableZRotation(Handle, enable)); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceBox_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceBox_EnableRegion(Handle, enable)); }
                }

                /// <summary>The surface bounding box region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceBox_Region(Handle)); }
                }

                /// <summary>The asymmetry detection type.</summary>
                property GoBoxAsymmetryType AsymmetryDetectionType
                {
                    GoBoxAsymmetryType get()           { return (GoBoxAsymmetryType) ::GoSurfaceBox_AsymmetryDetectionType(Handle); }
                    void set(GoBoxAsymmetryType value)  { KCheck(::GoSurfaceBox_SetAsymmetryDetectionType(Handle, value)); }
                }

                /// <summary>A GoSurfaceBox X measurement object.</summary>
                property GoSurfaceBoxX^ XMeasurement
                {
                    GoSurfaceBoxX^ get()           { return KToObject<GoSurfaceBoxX^>(::GoSurfaceBox_XMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox Y measurement object.</summary>
                property GoSurfaceBoxY^ YMeasurement
                {
                    GoSurfaceBoxY^ get()           { return KToObject<GoSurfaceBoxY^>(::GoSurfaceBox_YMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox Z measurement object.</summary>
                property GoSurfaceBoxZ^ ZMeasurement
                {
                    GoSurfaceBoxZ^ get()           { return KToObject<GoSurfaceBoxZ^>(::GoSurfaceBox_ZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox width measurement object.</summary>
                property GoSurfaceBoxWidth^ WidthMeasurement
                {
                    GoSurfaceBoxWidth^ get()           { return KToObject<GoSurfaceBoxWidth^>(::GoSurfaceBox_WidthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox length measurement object.</summary>
                property GoSurfaceBoxLength^ LengthMeasurement
                {
                    GoSurfaceBoxLength^ get()           { return KToObject<GoSurfaceBoxLength^>(::GoSurfaceBox_LengthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox height measurement object.</summary>
                property GoSurfaceBoxHeight^ HeightMeasurement
                {
                    GoSurfaceBoxHeight^ get()           { return KToObject<GoSurfaceBoxHeight^>(::GoSurfaceBox_HeightMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox Z angle measurement object.</summary>
                property GoSurfaceBoxZAngle^ ZAngleMeasurement
                {
                    GoSurfaceBoxZAngle^ get()           { return KToObject<GoSurfaceBoxZAngle^>(::GoSurfaceBox_ZAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox global X measurement object.</summary>
                property GoSurfaceBoxGlobalX^ GlobalXMeasurement
                {
                    GoSurfaceBoxGlobalX^ get()           { return KToObject<GoSurfaceBoxGlobalX^>(::GoSurfaceBox_GlobalXMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox global Y measurement object.</summary>
                property GoSurfaceBoxGlobalY^ GlobalYMeasurement
                {
                    GoSurfaceBoxGlobalY^ get()           { return KToObject<GoSurfaceBoxGlobalY^>(::GoSurfaceBox_GlobalYMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox global Z measurement object.</summary>
                property GoSurfaceBoxGlobalZAngle^ GlobalZAngleMeasurement
                {
                    GoSurfaceBoxGlobalZAngle^ get()           { return KToObject<GoSurfaceBoxGlobalZAngle^>(::GoSurfaceBox_GlobalZAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceBox center point feature.</summary>
                property GoSurfaceBoundingBoxCenterPoint^ CenterPoint
                {
                    GoSurfaceBoundingBoxCenterPoint^ get()           { return KToObject<GoSurfaceBoundingBoxCenterPoint^>(::GoSurfaceBox_CenterPoint(Handle)); }
                }

                /// <summary>A GoSurfaceBox axis line feature.</summary>
                property GoSurfaceBoundingBoxAxisLine^ AxisLine
                {
                    GoSurfaceBoundingBoxAxisLine^ get()           { return KToObject<GoSurfaceBoundingBoxAxisLine^>(::GoSurfaceBox_AxisLine(Handle)); }
                }
            };

            /// <summary>Represents a Surface Counter Sunk Hole tool.</summary>
            public ref class GoSurfaceCountersunkHole : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceCountersunkHole, GoSurfaceCountersunkHole)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHole class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHole(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHole class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceCountersunkHole(GoSensor^ sensor)
                {
                    ::GoSurfaceCountersunkHole handle = kNULL;

                    KCheck(::GoSurfaceCountersunkHole_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceCountersunkHole(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceCountersunkHole(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceCountersunkHole handle = kNULL;

                    KCheck(::GoSurfaceCountersunkHole_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The shape.</summary>
                property GoSurfaceCountersunkHoleShape Shape
                {
                    GoSurfaceCountersunkHoleShape get()           { return (GoSurfaceCountersunkHoleShape) ::GoSurfaceCountersunkHole_Shape(Handle); }
                    void set(GoSurfaceCountersunkHoleShape value)  { KCheck(::GoSurfaceCountersunkHole_SetShape(Handle, value)); }
                }

                /// <summary>The nominal bevel angle.</summary>
                property k64f NominalBevelAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_NominalBevelAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetNominalBevelAngle(Handle, value)); }
                }

                /// <summary>The nominal outer radius.</summary>
                property k64f NominalOuterRadius
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_NominalOuterRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetNominalOuterRadius(Handle, value)); }
                }

                /// <summary>The nominal inner radius.</summary>
                property k64f NominalInnerRadius
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_NominalInnerRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetNominalInnerRadius(Handle, value)); }
                }

                /// <summary>The bevel radius offset.</summary>
                property k64f BevelRadiusOffset
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_BevelRadiusOffset(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetBevelRadiusOffset(Handle, value)); }
                }

                /// <summary>Enables or disables partial counter sunk hole detection.</summary>
                property bool PartialDetectionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_PartialDetectionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnablePartialDetection(Handle, enable)); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceCountersunkHole_Region(Handle)); }
                }

                /// <summary>Enables or disables reference regions.</summary>
                property bool RefRegionsEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_RefRegionsEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnableRefRegions(Handle, enable)); }
                }

                /// <summary>The maximum number of reference regions permitted for this tool.</summary>
                literal k64s MaxRefRegions = GO_SURFACE_COUNTERSUNK_HOLE_MAX_REF_REGIONS;

                /// <summary>The reference region count.</summary>
                property k64s RefRegionCount
                {
                    k64s get()           { return (k64s) ::GoSurfaceCountersunkHole_RefRegionCount(Handle); }
                    void set(k64s value)  { KCheck(::GoSurfaceCountersunkHole_SetRefRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the reference region at the given index.</summary>
                /// <param name="index">The index of the reference region to retrieve.</param>
                /// <returns>A reference region.</returns>
                GoSurfaceRegion2d^ GetRefRegion(k64s index)
                {
                    return KToObject<GoSurfaceRegion2d^>(::GoSurfaceCountersunkHole_RefRegionAt(Handle, (kSize)index));
                }

                /// <summary>Enables or disables automatic tilt.</summary>
                property bool AutoTiltEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_AutoTiltEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnableAutoTilt(Handle, enable)); }
                }

                /// <summary>The tilt X angle.</summary>
                property k64f TiltXAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_TiltXAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetTiltXAngle(Handle, value)); }
                }

                /// <summary>The tilt Y angle.</summary>
                property k64f TiltYAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_TiltYAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetTiltYAngle(Handle, value)); }
                }

                /// <summary>Enables or disables curve fitting.</summary>
                property bool CurveFitEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_CurveFitEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnableCurveFit(Handle, enable)); }
                }

                /// <summary>The curve orientation angle.</summary>
                property k64f CurveOrientation
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_CurveOrientation(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetCurveOrientation(Handle, value)); }
                }

                /// <summary>The plane fit range.</summary>
                property k64f PlaneFitRange
                {
                    k64f get()           { return (k64f) ::GoSurfaceCountersunkHole_PlaneFitRange(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceCountersunkHole_SetPlaneFitRange(Handle, value)); }
                }

                /// <summary>Enables or disables plane fitting.</summary>
                property bool PlaneFitRangeEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceCountersunkHole_PlaneFitRangeEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceCountersunkHole_EnablePlaneFitRange(Handle, enable)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool X position measurement object.</summary>
                property GoSurfaceCountersunkHoleX^ XMeasurement
                {
                    GoSurfaceCountersunkHoleX^ get()           { return KToObject<GoSurfaceCountersunkHoleX^>(::GoSurfaceCountersunkHole_XMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool Y position measurement object.</summary>
                property GoSurfaceCountersunkHoleY^ YMeasurement
                {
                    GoSurfaceCountersunkHoleY^ get()           { return KToObject<GoSurfaceCountersunkHoleY^>(::GoSurfaceCountersunkHole_YMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool Z position measurement object.</summary>
                property GoSurfaceCountersunkHoleZ^ ZMeasurement
                {
                    GoSurfaceCountersunkHoleZ^ get()           { return KToObject<GoSurfaceCountersunkHoleZ^>(::GoSurfaceCountersunkHole_ZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool outer radius measurement object.</summary>
                property GoSurfaceCountersunkHoleOuterRadius^ OuterRadiusMeasurement
                {
                    GoSurfaceCountersunkHoleOuterRadius^ get()           { return KToObject<GoSurfaceCountersunkHoleOuterRadius^>(::GoSurfaceCountersunkHole_OuterRadiusMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool depth measurement object.</summary>
                property GoSurfaceCountersunkHoleDepth^ DepthMeasurement
                {
                    GoSurfaceCountersunkHoleDepth^ get()           { return KToObject<GoSurfaceCountersunkHoleDepth^>(::GoSurfaceCountersunkHole_DepthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool counterbore depth measurement object.</summary>
                property GoSurfaceCountersunkHoleCounterboreDepth^ CounterboreDepthMeasurement
                {
                    GoSurfaceCountersunkHoleCounterboreDepth^ get()           { return KToObject<GoSurfaceCountersunkHoleCounterboreDepth^>(::GoSurfaceCountersunkHole_CounterboreDepth(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool bevel radius measurement object.</summary>
                property GoSurfaceCountersunkHoleBevelRadius^ BevelRadiusMeasurement
                {
                    GoSurfaceCountersunkHoleBevelRadius^ get()           { return KToObject<GoSurfaceCountersunkHoleBevelRadius^>(::GoSurfaceCountersunkHole_BevelRadiusMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool bevel angle measurement object.</summary>
                property GoSurfaceCountersunkHoleBevelAngle^ BevelAngleMeasurement
                {
                    GoSurfaceCountersunkHoleBevelAngle^ get()           { return KToObject<GoSurfaceCountersunkHoleBevelAngle^>(::GoSurfaceCountersunkHole_BevelAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool X angle measurement object.</summary>
                property GoSurfaceCountersunkHoleXAngle^ XAngleMeasurement
                {
                    GoSurfaceCountersunkHoleXAngle^ get()           { return KToObject<GoSurfaceCountersunkHoleXAngle^>(::GoSurfaceCountersunkHole_XAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool Y angle measurement object.</summary>
                property GoSurfaceCountersunkHoleYAngle^ YAngleMeasurement
                {
                    GoSurfaceCountersunkHoleYAngle^ get()           { return KToObject<GoSurfaceCountersunkHoleYAngle^>(::GoSurfaceCountersunkHole_YAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool axis tilt measurement object.</summary>
                property GoSurfaceCountersunkHoleAxisTilt^ AxisTilt
                {
                    GoSurfaceCountersunkHoleAxisTilt^ get()           { return KToObject<GoSurfaceCountersunkHoleAxisTilt^>(::GoSurfaceCountersunkHole_AxisTilt(Handle)); }
                }

                /// <summary>A GoSurfaceCountersunkHole tool axis orientation measurement object.</summary>
                property GoSurfaceCountersunkHoleAxisOrientation^ AxisOrientation
                {
                    GoSurfaceCountersunkHoleAxisOrientation^ get()           { return KToObject<GoSurfaceCountersunkHoleAxisOrientation^>(::GoSurfaceCountersunkHole_AxisOrientation(Handle)); }
                }
                /// <summary>A GoSurfaceCountersunkHole tool axis orientation measurement object.</summary>
                property GoSurfaceCountersunkHoleCenterPoint^ CenterPoint
                {
                    GoSurfaceCountersunkHoleCenterPoint^ get()           { return KToObject<GoSurfaceCountersunkHoleCenterPoint^>(::GoSurfaceCountersunkHole_CenterPoint(Handle)); }
                }

            };

            /// <summary>Represents a Surface dimension tool.</summary>
            public ref class GoSurfaceDim : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceDim, GoSurfaceDim)

                /// <summary>Initializes a new instance of the GoSurfaceDim class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDim(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceDim class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceDim(GoSensor^ sensor)
                {
                    ::GoSurfaceDim handle = kNULL;

                    KCheck(::GoSurfaceDim_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceDim(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceDim(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceDim handle = kNULL;

                    KCheck(::GoSurfaceDim_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The reference Surface feature.</summary>
                property GoSurfaceFeature^ RefFeature
                {
                    GoSurfaceFeature^ get()           { return KToObject<GoSurfaceFeature^>(::GoSurfaceDim_RefFeature(Handle)); }
                }

                /// <summary>The non-reference Surface feature.</summary>
                property GoSurfaceFeature^ Feature
                {
                    GoSurfaceFeature^ get()           { return KToObject<GoSurfaceFeature^>(::GoSurfaceDim_Feature(Handle)); }
                }

                /// <summary>A GoSurfaceDim width measurement object.</summary>
                property GoSurfaceDimWidth^ WidthMeasurement
                {
                    GoSurfaceDimWidth^ get()           { return KToObject<GoSurfaceDimWidth^>(::GoSurfaceDim_WidthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim height measurement object.</summary>
                property GoSurfaceDimHeight^ HeightMeasurement
                {
                    GoSurfaceDimHeight^ get()           { return KToObject<GoSurfaceDimHeight^>(::GoSurfaceDim_HeightMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim length measurement object.</summary>
                property GoSurfaceDimLength^ LengthMeasurement
                {
                    GoSurfaceDimLength^ get()           { return KToObject<GoSurfaceDimLength^>(::GoSurfaceDim_LengthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim distance measurement object.</summary>
                property GoSurfaceDimDistance^ DistanceMeasurement
                {
                    GoSurfaceDimDistance^ get()           { return KToObject<GoSurfaceDimDistance^>(::GoSurfaceDim_DistanceMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim plane distance measurement object.</summary>
                property GoSurfaceDimPlaneDistance^ PlaneDistanceMeasurement
                {
                    GoSurfaceDimPlaneDistance^ get()           { return KToObject<GoSurfaceDimPlaneDistance^>(::GoSurfaceDim_PlaneDistanceMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim center X measurement object.</summary>
                property GoSurfaceDimCenterX^ CenterXMeasurement
                {
                    GoSurfaceDimCenterX^ get()           { return KToObject<GoSurfaceDimCenterX^>(::GoSurfaceDim_CenterXMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim center Y measurement object.</summary>
                property GoSurfaceDimCenterY^ CenterYMeasurement
                {
                    GoSurfaceDimCenterY^ get()           { return KToObject<GoSurfaceDimCenterY^>(::GoSurfaceDim_CenterYMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceDim center Z measurement object.</summary>
                property GoSurfaceDimCenterZ^ CenterZMeasurement
                {
                    GoSurfaceDimCenterZ^ get()           { return KToObject<GoSurfaceDimCenterZ^>(::GoSurfaceDim_CenterZMeasurement(Handle)); }
                }

                /// <summary>Returns a GoSurfaceDim center point feature object.</summary>
                property GoSurfaceDimensionCenterPoint^ CenterPoint
                {
                    GoSurfaceDimensionCenterPoint^ get() { return KToObject<GoSurfaceDimensionCenterPoint^>(::GoSurfaceDim_CenterPoint(Handle)); }
                }
            };

            /// <summary>Represents a surface ellipse tool.</summary>
            public ref class GoSurfaceEllipse : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceEllipse, GoSurfaceEllipse)

                /// <summary>Initializes a new instance of the GoSurfaceEllipse class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceEllipse(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceEllipse class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceEllipse(GoSensor^ sensor)
                {
                    ::GoSurfaceEllipse handle = kNULL;

                    KCheck(::GoSurfaceEllipse_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceEllipse(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceEllipse(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceEllipse handle = kNULL;

                    KCheck(::GoSurfaceEllipse_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceEllipse_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceEllipse_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceEllipse_Region(Handle)); }
                }

                /// <summary>The asymmetry detection type.</summary>
                property GoBoxAsymmetryType AsymmetryDetectionType
                {
                    GoBoxAsymmetryType get()           { return (GoBoxAsymmetryType) ::GoSurfaceEllipse_AsymmetryDetectionType(Handle); }
                    void set(GoBoxAsymmetryType value)  { KCheck(::GoSurfaceEllipse_SetAsymmetryDetectionType(Handle, value)); }
                }

                /// <summary>A GoSurfaceEllipse major measurement object.</summary>
                property GoSurfaceEllipseMajor^ MajorMeasurement
                {
                    GoSurfaceEllipseMajor^ get()           { return KToObject<GoSurfaceEllipseMajor^>(::GoSurfaceEllipse_MajorMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse minor measurement object.</summary>
                property GoSurfaceEllipseMinor^ MinorMeasurement
                {
                    GoSurfaceEllipseMinor^ get()           { return KToObject<GoSurfaceEllipseMinor^>(::GoSurfaceEllipse_MinorMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse ratio measurement object.</summary>
                property GoSurfaceEllipseRatio^ RatioMeasurement
                {
                    GoSurfaceEllipseRatio^ get()           { return KToObject<GoSurfaceEllipseRatio^>(::GoSurfaceEllipse_RatioMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse Z angle measurement object.</summary>
                property GoSurfaceEllipseZAngle^ ZAngleMeasurement
                {
                    GoSurfaceEllipseZAngle^ get()           { return KToObject<GoSurfaceEllipseZAngle^>(::GoSurfaceEllipse_ZAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse point feature.</summary>
                property GoSurfaceEllipseCenterPoint^ Point
                {
                    GoSurfaceEllipseCenterPoint^ get()           { return KToObject<GoSurfaceEllipseCenterPoint^>(::GoSurfaceEllipse_CenterPoint(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse axis line feature object.</summary>
                property GoSurfaceEllipseMajorAxisLine^ MajorAxisLine
                {
                    GoSurfaceEllipseMajorAxisLine^ get()           { return KToObject<GoSurfaceEllipseMajorAxisLine^>(::GoSurfaceEllipse_MajorAxisLine(Handle)); }
                }

                /// <summary>A GoSurfaceEllipse axis line feature object.</summary>
                property GoSurfaceEllipseMinorAxisLine^ MinorAxisLine
                {
                    GoSurfaceEllipseMinorAxisLine^ get()           { return KToObject<GoSurfaceEllipseMinorAxisLine^>(::GoSurfaceEllipse_MinorAxisLine(Handle)); }
                }
            };

            /// <summary>Represents a surface hole tool.</summary>
            public ref class GoSurfaceHole : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceHole, GoSurfaceHole)

                /// <summary>Initializes a new instance of the GoSurfaceHole class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceHole(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceHole class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceHole(GoSensor^ sensor)
                {
                    ::GoSurfaceHole handle = kNULL;

                    KCheck(::GoSurfaceHole_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceHole(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceHole(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceHole handle = kNULL;

                    KCheck(::GoSurfaceHole_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The nominal radius value.</summary>
                property k64f NominalRadius
                {
                    k64f get()           { return (k64f) ::GoSurfaceHole_NominalRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceHole_SetNominalRadius(Handle, value)); }
                }

                /// <summary>The radius tolerance value.</summary>
                property k64f RadiusTolerance
                {
                    k64f get()           { return (k64f) ::GoSurfaceHole_RadiusTolerance(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceHole_SetRadiusTolerance(Handle, value)); }
                }

                /// <summary>Enables or disables partial detection.</summary>
                property bool PartialDetectionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceHole_PartialDetectionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceHole_EnablePartialDetection(Handle, enable)); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceHole_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceHole_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceHole_Region(Handle)); }
                }

                /// <summary>Enables or disables reference regions.</summary>
                property bool RefRegionsEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceHole_RefRegionsEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceHole_EnableRefRegions(Handle, enable)); }
                }

                /// <summary>The maximum number of reference regions permitted for this tool.</summary>
                literal k64s MaxRefRegions = GO_SURFACE_HOLE_MAX_REF_REGIONS;

                /// <summary>The reference region count.</summary>
                property k64s RefRegionCount
                {
                    k64s get()           { return (k64s) ::GoSurfaceHole_RefRegionCount(Handle); }
                    void set(k64s value)  { KCheck(::GoSurfaceHole_SetRefRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the reference region at the given index.</summary>
                /// <param name="index">The index of the reference region to retrieve.</param>
                /// <returns>A reference region.</returns>
                GoSurfaceRegion2d^ GetRefRegion(k64s index)
                {
                    return KToObject<GoSurfaceRegion2d^>(::GoSurfaceHole_RefRegionAt(Handle, (kSize)index));
                }

                /// <summary>Enables or disables automatic tilt.</summary>
                property bool AutoTiltEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceHole_AutoTiltEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceHole_EnableAutoTilt(Handle, enable)); }
                }

                /// <summary>The tilt X angle.</summary>
                property k64f TiltXAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceHole_TiltXAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceHole_SetTiltXAngle(Handle, value)); }
                }

                /// <summary>The tilt Y angle.</summary>
                property k64f TiltYAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceHole_TiltYAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceHole_SetTiltYAngle(Handle, value)); }
                }

                /// <summary>The enabled state of the depth limit.</summary>
                property bool DepthLimitEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceHole_DepthLimitEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceHole_EnableDepthLimit(Handle, enable)); }
                }

                /// <summary>The depth limit value.</summary>
                property k64f DepthLimit
                {
                    k64f get()           { return (k64f) ::GoSurfaceHole_DepthLimit(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceHole_SetDepthLimit(Handle, value)); }
                }

                /// <summary>A GoSurfaceHole tool X position measurement object.</summary>
                property GoSurfaceHoleX^ XMeasurement
                {
                    GoSurfaceHoleX^ get()           { return KToObject<GoSurfaceHoleX^>(::GoSurfaceHole_XMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceHole tool Y position measurement object.</summary>
                property GoSurfaceHoleY^ YMeasurement
                {
                    GoSurfaceHoleY^ get()           { return KToObject<GoSurfaceHoleY^>(::GoSurfaceHole_YMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceHole tool Z position measurement object.</summary>
                property GoSurfaceHoleZ^ ZMeasurement
                {
                    GoSurfaceHoleZ^ get()           { return KToObject<GoSurfaceHoleZ^>(::GoSurfaceHole_ZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceHole tool radius measurement object.</summary>
                property GoSurfaceHoleRadius^ RadiusMeasurement
                {
                    GoSurfaceHoleRadius^ get()           { return KToObject<GoSurfaceHoleRadius^>(::GoSurfaceHole_RadiusMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceHole tool radius measurement object.</summary>
                property GoSurfaceHoleCenterPoint^ Point
                {
                    GoSurfaceHoleCenterPoint^ get()           { return KToObject<GoSurfaceHoleCenterPoint^>(::GoSurfaceHole_Point(Handle)); }
                }

            };

            /// <summary>Represents a surface opening tool.</summary>
            public ref class GoSurfaceOpening : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceOpening, GoSurfaceOpening)

                /// <summary>Initializes a new instance of the GoSurfaceOpening class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpening(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceOpening class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceOpening(GoSensor^ sensor)
                {
                    ::GoSurfaceOpening handle = kNULL;

                    KCheck(::GoSurfaceOpening_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceOpening(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceOpening(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceOpening handle = kNULL;

                    KCheck(::GoSurfaceOpening_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The surface opening type.</summary>
                property GoSurfaceOpeningType OpeningType
                {
                    GoSurfaceOpeningType get()           { return (GoSurfaceOpeningType) ::GoSurfaceOpening_Type(Handle); }
                    void set(GoSurfaceOpeningType type)  { KCheck(::GoSurfaceOpening_SetType(Handle, type)); }
                }

                /// <summary>The nominal width.</summary>
                property k64f NominalWidth
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_NominalWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetNominalWidth(Handle, value)); }
                }

                /// <summary>The nominal length.</summary>
                property k64f NominalLength
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_NominalLength(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetNominalLength(Handle, value)); }
                }

                /// <summary>The nominal angle.</summary>
                property k64f NominalAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_NominalAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetNominalAngle(Handle, value)); }
                }

                /// <summary>The nominal radius.</summary>
                property k64f NominalRadius
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_NominalRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetNominalRadius(Handle, value)); }
                }

                /// <summary>The width tolerance.</summary>
                property k64f WidthTolerance
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_WidthTolerance(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetWidthTolerance(Handle, value)); }
                }

                /// <summary>The width tolerance.</summary>
                property k64f LengthTolerance
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_LengthTolerance(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetLengthTolerance(Handle, value)); }
                }

                /// <summary>The angle tolerance.</summary>
                property k64f AngleTolerance
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_AngleTolerance(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetAngleTolerance(Handle, value)); }
                }

                /// <summary>The enabled state of partial detection.</summary>
                property bool PartialDetectionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceOpening_PartialDetectionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceOpening_EnablePartialDetection(Handle, enable)); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceOpening_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceOpening_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceOpening_Region(Handle)); }
                }

                /// <summary>Enables or disables reference regions.</summary>
                property bool RefRegionsEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceOpening_RefRegionsEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceOpening_EnableRefRegions(Handle, enable)); }
                }

                /// <summary>The maximum number of reference regions permitted for this tool.</summary>
                literal k64s MaxRefRegions = GO_SURFACE_OPENING_MAX_REF_REGIONS;

                /// <summary>The reference region count.</summary>
                property k64s RefRegionCount
                {
                    k64s get()           { return (k64s) ::GoSurfaceOpening_RefRegionCount(Handle); }
                    void set(k64s value)  { KCheck(::GoSurfaceOpening_SetRefRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the reference region at the given index.</summary>
                /// <param name="index">The index of the reference region to retrieve.</param>
                /// <returns>A reference region.</returns>
                GoSurfaceRegion2d^ GetRefRegion(k64s index)
                {
                    return KToObject<GoSurfaceRegion2d^>(::GoSurfaceOpening_RefRegionAt(Handle, (kSize)index));
                }

                /// <summary>Enables or disables automatic tilt.</summary>
                property bool AutoTiltEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceOpening_AutoTiltEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceOpening_EnableAutoTilt(Handle, enable)); }
                }

                /// <summary>The tilt X angle.</summary>
                property k64f TiltXAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_TiltXAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetTiltXAngle(Handle, value)); }
                }

                /// <summary>The tilt Y angle.</summary>
                property k64f TiltYAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_TiltYAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetTiltYAngle(Handle, value)); }
                }

                /// <summary>The enabled state of the depth limit.</summary>
                property bool DepthLimitEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceOpening_DepthLimitEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceOpening_EnableDepthLimit(Handle, enable)); }
                }

                /// <summary>The depth limit value.</summary>
                property k64f DepthLimit
                {
                    k64f get()           { return (k64f) ::GoSurfaceOpening_DepthLimit(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceOpening_SetDepthLimit(Handle, value)); }
                }

                /// <summary>A GoSurfaceOpening tool X position measurement object.</summary>
                property GoSurfaceOpeningX^ XMeasurement
                {
                    GoSurfaceOpeningX^ get()           { return KToObject<GoSurfaceOpeningX^>(::GoSurfaceOpening_XMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceOpening tool Y position measurement object.</summary>
                property GoSurfaceOpeningY^ YMeasurement
                {
                    GoSurfaceOpeningY^ get()           { return KToObject<GoSurfaceOpeningY^>(::GoSurfaceOpening_YMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceOpening tool Z position measurement object.</summary>
                property GoSurfaceOpeningZ^ ZMeasurement
                {
                    GoSurfaceOpeningZ^ get()           { return KToObject<GoSurfaceOpeningZ^>(::GoSurfaceOpening_ZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceOpening tool width measurement object.</summary>
                property GoSurfaceOpeningWidth^ WidthMeasurement
                {
                    GoSurfaceOpeningWidth^ get()           { return KToObject<GoSurfaceOpeningWidth^>(::GoSurfaceOpening_WidthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceOpening tool length measurement object.</summary>
                property GoSurfaceOpeningLength^ LengthMeasurement
                {
                    GoSurfaceOpeningLength^ get()           { return KToObject<GoSurfaceOpeningLength^>(::GoSurfaceOpening_LengthMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceOpening tool angle measurement object.</summary>
                property GoSurfaceOpeningAngle^ AngleMeasurement
                {
                    GoSurfaceOpeningAngle^ get()           { return KToObject<GoSurfaceOpeningAngle^>(::GoSurfaceOpening_AngleMeasurement(Handle)); }
                }
                /// <summary>A GoSurfaceOpening tool center point object.</summary>
                property GoSurfaceOpeningCenterPoint^ CenterPoint
                {
                    GoSurfaceOpeningCenterPoint^ get()           { return KToObject<GoSurfaceOpeningCenterPoint^>(::GoSurfaceOpening_CenterPoint(Handle)); }
                }
            };

            /// <summary>Represents a surface plane tool.</summary>
            public ref class GoSurfacePlane : public GoSurfaceTool
            {
                KDeclareClass(GoSurfacePlane, GoSurfacePlane)

                /// <summary>Initializes a new instance of the GoSurfacePlane class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlane(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfacePlane class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfacePlane(GoSensor^ sensor)
                {
                    ::GoSurfacePlane handle = kNULL;

                    KCheck(::GoSurfacePlane_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfacePlane(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfacePlane(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfacePlane handle = kNULL;

                    KCheck(::GoSurfacePlane_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables reference regions.</summary>
                property bool RegionsEnabled
                {
                    bool get()           { return KToBool(::GoSurfacePlane_RegionsEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfacePlane_EnableRegions(Handle, enable)); }
                }

                /// <summary>The maximum number of reference regions permitted for this tool.</summary>
                literal k64s MaxRegions = GO_SURFACE_PLANE_MAX_REGIONS;

                /// <summary>The tool's region count.</summary>
                property k64s RegionCount
                {
                    k64s get()           { return (k64s) ::GoSurfacePlane_RegionCount(Handle); }
                    void set(k64s value)  { KCheck(::GoSurfacePlane_SetRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the region at the specified index.</summary>
                /// <param name="index">The index with which to return a tool region.</param>
                /// <returns>A GoRegion3d object or null if the index is in invalid.</returns>
                GoRegion3d^ GetRegion(k64s index)
                {
                    ::GoRegion3d region = ::GoSurfacePlane_RegionAt(Handle, (kSize)index);

                    if(kIsNull(region)) { return nullptr; }

                    return KToObject<GoRegion3d^>(region);
                }

                /// <summary>A GoSurfacePlane X angle measurement object.</summary>
                property GoSurfacePlaneXAngle^ XAngleMeasurement
                {
                    GoSurfacePlaneXAngle^ get()           { return KToObject<GoSurfacePlaneXAngle^>(::GoSurfacePlane_XAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePlane Y angle measurement object.</summary>
                property GoSurfacePlaneYAngle^ YAngleMeasurement
                {
                    GoSurfacePlaneYAngle^ get()           { return KToObject<GoSurfacePlaneYAngle^>(::GoSurfacePlane_YAngleMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePlane Z offset measurement object.</summary>
                property GoSurfacePlaneZOffset^ ZOffsetMeasurement
                {
                    GoSurfacePlaneZOffset^ get()           { return KToObject<GoSurfacePlaneZOffset^>(::GoSurfacePlane_ZOffsetMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePlane standard deviation measurement object.</summary>
                property GoSurfacePlaneStdDev^ StdDevMeasurement
                {
                    GoSurfacePlaneStdDev^ get()           { return KToObject<GoSurfacePlaneStdDev^>(::GoSurfacePlane_StdDevMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePlane minimum error measurement object.</summary>
                property GoSurfacePlaneMinError^ MinErrorMeasurement
                {
                    GoSurfacePlaneMinError^ get()           { return KToObject<GoSurfacePlaneMinError^>(::GoSurfacePlane_MinErrorMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePlane maximum error measurement object.</summary>
                property GoSurfacePlaneMaxError^ MaxErrorMeasurement
                {
                    GoSurfacePlaneMaxError^ get()           { return KToObject<GoSurfacePlaneMaxError^>(::GoSurfacePlane_MaxErrorMeasurement(Handle)); }
                }
                /// <summary>A GoSurfacePlane x normal measurement object.</summary>
                property GoSurfacePlaneXNormal^ XNormalMeasurement
                {
                    GoSurfacePlaneXNormal^ get()           { return KToObject<GoSurfacePlaneXNormal^>(::GoSurfacePlane_XNormalMeasurement(Handle)); }
                }
                /// <summary>A GoSurfacePlane y normal measurement object.</summary>
                property GoSurfacePlaneYNormal^ YNormalMeasurement
                {
                    GoSurfacePlaneYNormal^ get()           { return KToObject<GoSurfacePlaneYNormal^>(::GoSurfacePlane_YNormalMeasurement(Handle)); }
                }
                /// <summary>A GoSurfacePlane z normal measurement object.</summary>
                property GoSurfacePlaneZNormal^ ZNormalMeasurement
                {
                    GoSurfacePlaneZNormal^ get()           { return KToObject<GoSurfacePlaneZNormal^>(::GoSurfacePlane_ZNormalMeasurement(Handle)); }
                }
                /// <summary>A GoSurfacePlane distance measurement object.</summary>
                property GoSurfacePlaneDistance^ DistanceMeasurement
                {
                    GoSurfacePlaneDistance^ get()           { return KToObject<GoSurfacePlaneDistance^>(::GoSurfacePlane_DistanceMeasurement(Handle)); }
                }
                /// <summary>A GoSurfacePlane tool plane object.</summary>
                property GoSurfacePlanePlane^ Plane
                {
                    GoSurfacePlanePlane^ get()           { return KToObject<GoSurfacePlanePlane^>(::GoSurfacePlane_Plane(Handle)); }
                }
            };

            /// <summary>Represents a surface position tool.</summary>
            public ref class GoSurfacePosition : public GoSurfaceTool
            {
                KDeclareClass(GoSurfacePosition, GoSurfacePosition)

                /// <summary>Initializes a new instance of the GoSurfacePosition class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePosition(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfacePosition class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfacePosition(GoSensor^ sensor)
                {
                    ::GoSurfacePosition handle = kNULL;

                    KCheck(::GoSurfacePosition_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfacePosition(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfacePosition(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfacePosition handle = kNULL;

                    KCheck(::GoSurfacePosition_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The tool feature.</summary>
                property GoSurfaceFeature^ Feature
                {
                    GoSurfaceFeature^ get()           { return KToObject<GoSurfaceFeature^>(::GoSurfacePosition_Feature(Handle)); }
                }

                /// <summary>A GoSurfacePosition X position measurement object.</summary>
                property GoSurfacePositionX^ XMeasurement
                {
                    GoSurfacePositionX^ get()           { return KToObject<GoSurfacePositionX^>(::GoSurfacePosition_XMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePosition Y position measurement object.</summary>
                property GoSurfacePositionY^ YMeasurement
                {
                    GoSurfacePositionY^ get()           { return KToObject<GoSurfacePositionY^>(::GoSurfacePosition_YMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePosition Z position measurement object.</summary>
                property GoSurfacePositionZ^ ZMeasurement
                {
                    GoSurfacePositionZ^ get()           { return KToObject<GoSurfacePositionZ^>(::GoSurfacePosition_ZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfacePosition Z position measurement object.</summary>
                property GoSurfacePositionPoint^ Point
                {
                    GoSurfacePositionPoint^ get()           { return KToObject<GoSurfacePositionPoint^>(::GoSurfacePosition_Point(Handle)); }
                }

            };

            /// <summary>Represents a surface stud tool.</summary>
            public ref class GoSurfaceStud : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceStud, GoSurfaceStud)

                /// <summary>Initializes a new instance of the GoSurfaceStud class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStud(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceStud class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceStud(GoSensor^ sensor)
                {
                    ::GoSurfaceStud handle = kNULL;

                    KCheck(::GoSurfaceStud_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceStud(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceStud(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceStud handle = kNULL;

                    KCheck(::GoSurfaceStud_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The stud radius value.</summary>
                property k64f StudRadius
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_StudRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetStudRadius(Handle, value)); }
                }

                /// <summary>The stud height value.</summary>
                property k64f StudHeight
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_StudHeight(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetStudHeight(Handle, value)); }
                }

                /// <summary>The stud base height value.</summary>
                property k64f BaseHeight
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_BaseHeight(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetBaseHeight(Handle, value)); }
                }

                /// <summary>The stud tip height value.</summary>
                property k64f TipHeight
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_TipHeight(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetTipHeight(Handle, value)); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceStud_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceStud_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceStud_Region(Handle)); }
                }

                /// <summary>Enables or disables reference regions.</summary>
                property bool RefRegionsEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceStud_RefRegionsEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceStud_EnableRefRegions(Handle, enable)); }
                }

                /// <summary>The maximum number of reference regions permitted for this tool.</summary>
                literal k64s MaxRefRegions = GO_SURFACE_STUD_MAX_REF_REGIONS;

                /// <summary>The reference region count.</summary>
                property k64s RefRegionCount
                {
                    k64s get()           { return (k64s) ::GoSurfaceStud_RefRegionCount(Handle); }
                    void set(k64s value)  { KCheck(::GoSurfaceStud_SetRefRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the reference region at the given index.</summary>
                /// <param name="index">The index of the reference region to retrieve.</param>
                /// <returns>A reference region.</returns>
                GoSurfaceRegion2d^ GetRefRegion(k64s index)
                {
                    return KToObject<GoSurfaceRegion2d^>(::GoSurfaceStud_RefRegionAt(Handle, (kSize)index));
                }

                /// <summary>Enables or disables automatic tilt.</summary>
                property bool AutoTiltEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceStud_AutoTiltEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceStud_EnableAutoTilt(Handle, enable)); }
                }

                /// <summary>The tilt X angle.</summary>
                property k64f TiltXAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_TiltXAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetTiltXAngle(Handle, value)); }
                }

                /// <summary>The tilt Y angle.</summary>
                property k64f TiltYAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceStud_TiltYAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStud_SetTiltYAngle(Handle, value)); }
                }

                /// <summary>A GoSurfaceStud tool base X measurement object.</summary>
                property GoSurfaceStudBaseX^ BaseXMeasurement
                {
                    GoSurfaceStudBaseX^ get()           { return KToObject<GoSurfaceStudBaseX^>(::GoSurfaceStud_BaseXMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool base Y measurement object.</summary>
                property GoSurfaceStudBaseY^ BaseYMeasurement
                {
                    GoSurfaceStudBaseY^ get()           { return KToObject<GoSurfaceStudBaseY^>(::GoSurfaceStud_BaseYMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool base Z measurement object.</summary>
                property GoSurfaceStudBaseZ^ BaseZMeasurement
                {
                    GoSurfaceStudBaseZ^ get()           { return KToObject<GoSurfaceStudBaseZ^>(::GoSurfaceStud_BaseZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool tip X measurement object.</summary>
                property GoSurfaceStudTipX^ TipXMeasurement
                {
                    GoSurfaceStudTipX^ get()           { return KToObject<GoSurfaceStudTipX^>(::GoSurfaceStud_TipXMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool tip Y measurement object.</summary>
                property GoSurfaceStudTipY^ TipYMeasurement
                {
                    GoSurfaceStudTipY^ get()           { return KToObject<GoSurfaceStudTipY^>(::GoSurfaceStud_TipYMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool tip Z measurement object.</summary>
                property GoSurfaceStudTipZ^ TipZMeasurement
                {
                    GoSurfaceStudTipZ^ get()           { return KToObject<GoSurfaceStudTipZ^>(::GoSurfaceStud_TipZMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceStud tool radius measurement object.</summary>
                property GoSurfaceStudRadius^ RadiusMeasurement
                {
                    GoSurfaceStudRadius^ get()           { return KToObject<GoSurfaceStudRadius^>(::GoSurfaceStud_RadiusMeasurement(Handle)); }
                }

                // <summary>A GoSurfaceStud tool radius measurement object.</summary>
                property GoSurfaceStudTipPoint^ TipPoint
                {
                    GoSurfaceStudTipPoint^ get()           { return KToObject<GoSurfaceStudTipPoint^>(::GoSurfaceStud_TipPoint(Handle)); }
                }

                // <summary>A GoSurfaceStud tool radius measurement object.</summary>
                property GoSurfaceStudBasePoint^ BasePoint
                {
                    GoSurfaceStudBasePoint^ get()           { return KToObject<GoSurfaceStudBasePoint^>(::GoSurfaceStud_BasePoint(Handle)); }
                }


            };

            /// <summary>Represents a surface volume tool.</summary>
            public ref class GoSurfaceVolume : public GoSurfaceTool
            {
                KDeclareClass(GoSurfaceVolume, GoSurfaceVolume)

                /// <summary>Initializes a new instance of the GoSurfaceVolume class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceVolume(IntPtr handle)
                    : GoSurfaceTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceVolume class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceVolume(GoSensor^ sensor)
                {
                    ::GoSurfaceVolume handle = kNULL;

                    KCheck(::GoSurfaceVolume_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceVolume(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceVolume(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceVolume handle = kNULL;

                    KCheck(::GoSurfaceVolume_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceVolume_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceVolume_EnableRegion(Handle, enable)); }
                }

                /// <summary>The tool region.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceVolume_Region(Handle)); }
                }

                /// <summary>A GoSurfaceVolume volume measurement object.</summary>
                property GoSurfaceVolumeVolume^ VolumeMeasurement
                {
                    GoSurfaceVolumeVolume^ get()           { return KToObject<GoSurfaceVolumeVolume^>(::GoSurfaceVolume_VolumeMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceVolume area measurement object.</summary>
                property GoSurfaceVolumeArea^ AreaMeasurement
                {
                    GoSurfaceVolumeArea^ get()           { return KToObject<GoSurfaceVolumeArea^>(::GoSurfaceVolume_AreaMeasurement(Handle)); }
                }

                /// <summary>A GoSurfaceVolume thickness measurement object.</summary>
                property GoSurfaceVolumeThickness^ ThicknessMeasurement
                {
                    GoSurfaceVolumeThickness^ get()           { return KToObject<GoSurfaceVolumeThickness^>(::GoSurfaceVolume_ThicknessMeasurement(Handle)); }
                }
            };

            private ref class GoSurfaceTools abstract sealed
            {
            internal:
                static GoSurfaceTool^ GetSurfaceToolInstance(::GoSurfaceTool tool)
                {
                    if (kObject_Is(tool, kTypeOf(GoSurfaceBox))) { return KToObject<GoSurfaceBox^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceCountersunkHole))) { return KToObject<GoSurfaceCountersunkHole^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceDim))) { return KToObject<GoSurfaceDim^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceEllipse))) { return KToObject<GoSurfaceEllipse^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceHole))) { return KToObject<GoSurfaceHole^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceOpening))) { return KToObject<GoSurfaceOpening^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfacePlane))) { return KToObject<GoSurfacePlane^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfacePosition))) { return KToObject<GoSurfacePosition^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceStud))) { return KToObject<GoSurfaceStud^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoSurfaceVolume))) { return KToObject<GoSurfaceVolume^>(tool); }
                    return nullptr;
                }
            };
        }
    }
}

#endif
