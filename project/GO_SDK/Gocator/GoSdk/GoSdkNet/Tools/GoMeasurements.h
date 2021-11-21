//
// GoMeasurements.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_MEASUREMENTS_H
#define GO_SDK_NET_MEASUREMENTS_H

#include <GoSdk/Tools/GoMeasurements.h>
#include <GoSdk/Tools/GoExtParam.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            public ref class GoExtMeasurement : public GoMeasurement
            {
                KDeclareClass(GoExtMeasurement, GoExtMeasurement)

            public:
                /// <summary>Default GoMeasurement constructor.</summary>
                GoExtMeasurement() {}

                /// <summary>Initializes a new instance of the GoMeasurement class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtMeasurement(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Gets type name of the extensible measurement</summary>
                String^ Type()
                {
                    return KToString(::GoExtMeasurement_Type(Handle));
                }

                /// <summary>Gets a count of the custom pararmeters</summary>
                kSize GetParameterCount()
                {
                    return ::GoExtMeasurement_CustomParameterCount(Handle);
                }

                /// <summary>Returns a custom pararmater by index</summary>
                GoExtParam^ GetCustomParameterAt(k64s index)
                {
                    return KToObject<GoExtParam^>(::GoExtMeasurement_CustomParameterAt(Handle, (kSize)index));
                }

            };

            /// <summary>Represents a position Z measurement of a Range Position tool.</summary>
            public ref class GoRangePositionZ : public GoMeasurement
            {
                KDeclareClass(GoRangePositionZ, GoRangePositionZ)

                /// <summary>Initializes a new instance of the GoRangePositionZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangePositionZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a the thickness measurement of a Range Thickness tool.</summary>
            public ref class GoRangeThicknessThickness : public GoMeasurement
            {
                KDeclareClass(GoRangeThicknessThickness, GoRangeThicknessThickness)

                /// <summary>Initializes a new instance of the GoRangeThicknessThickness class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangeThicknessThickness(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an area measurement for a Profile Area tool.</summary>
            public ref class GoProfileAreaArea : public GoMeasurement
            {
                KDeclareClass(GoProfileAreaArea, GoProfileAreaArea)

                /// <summary>Initializes a new instance of the GoProfileAreaArea class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileAreaArea(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a centroid X measurement for a Profile Area Tool.</summary>
            public ref class GoProfileAreaCentroidX : public GoMeasurement
            {
                KDeclareClass(GoProfileAreaCentroidX, GoProfileAreaCentroidX)

                /// <summary>Initializes a new instance of the GoProfileAreaCentroidX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileAreaCentroidX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a centroid Z measurement for a Profile Area Tool.</summary>
            public ref class GoProfileAreaCentroidZ : public GoMeasurement
            {
                KDeclareClass(GoProfileAreaCentroidZ, GoProfileAreaCentroidZ)

                /// <summary>Initializes a new instance of the GoProfileAreaCentroidZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileAreaCentroidZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxX : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxX, GoProfileBoxX)

                /// <summary>Initializes a new instance of the GoProfileBoxX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxZ : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxZ, GoProfileBoxZ)

                /// <summary>Initializes a new instance of the GoProfileBoxZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a width measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxWidth : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxWidth, GoProfileBoxWidth)

                /// <summary>Initializes a new instance of the GoProfileBoxWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a height measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxHeight : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxHeight, GoProfileBoxHeight)

                /// <summary>Initializes a new instance of the GoProfileBoxHeight class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxHeight(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global X measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxGlobalX : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxGlobalX, GoProfileBoxGlobalX)

                /// <summary>Initializes a new instance of the GoProfileBoxGlobalX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxGlobalX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global Y measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxGlobalY : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxGlobalY, GoProfileBoxGlobalY)

                /// <summary>Initializes a new instance of the GoProfileBoxGlobalY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxGlobalY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global angle measurement for a Profile Bounding Box tool.</summary>
            public ref class GoProfileBoxGlobalAngle : public GoMeasurement
            {
                KDeclareClass(GoProfileBoxGlobalAngle, GoProfileBoxGlobalAngle)

                /// <summary>Initializes a new instance of the GoProfileBoxGlobalAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBoxGlobalAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a bridge value measurement for a Profile Bridge Value tool.</summary>
            public ref class GoProfileBridgeValueBridgeValue : public GoMeasurement
            {
                KDeclareClass(GoProfileBridgeValueBridgeValue, GoProfileBridgeValueBridgeValue)

                /// <summary>Initializes a new instance of the GoProfileBridgeValueBridgeValue class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBridgeValueBridgeValue(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an angle measurement for a Profile Bridge Value tool.</summary>
            public ref class GoProfileBridgeValueAngle : public GoMeasurement
            {
                KDeclareClass(GoProfileBridgeValueAngle, GoProfileBridgeValueAngle)

                /// <summary>Initializes a new instance of the GoProfileBridgeValueAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileBridgeValueAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an window measurement for a Profile Bridge Value tool.</summary>
            public ref class GoProfileBridgeValueWindow : public GoMeasurement
            {
                KDeclareClass(GoProfileBridgeValueWindow, GoProfileBridgeValueWindow)

                    /// <summary>Initializes a new instance of the GoProfileBridgeValueWindow class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileBridgeValueWindow(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an standard deviation measurement for a Profile Bridge Value tool.</summary>
            public ref class GoProfileBridgeValueStdDev : public GoMeasurement
            {
                KDeclareClass(GoProfileBridgeValueStdDev, GoProfileBridgeValueStdDev)

                    /// <summary>Initializes a new instance of the GoProfileBridgeValueStdDev class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileBridgeValueStdDev(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X value measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleX : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleX, GoProfileCircleX)

                /// <summary>Initializes a new instance of the GoProfileCircleX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z value measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleZ : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleZ, GoProfileCircleZ)

                /// <summary>Initializes a new instance of the GoProfileCircleZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a radius value measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleRadius : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleRadius, GoProfileCircleRadius)

                /// <summary>Initializes a new instance of the GoProfileCircleRadius class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleRadius(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a StdDev measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleStdDev : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleStdDev, GoProfileCircleStdDev)

                /// <summary>Initializes a new instance of the GoProfileCircleStdDev class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleStdDev(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a min error measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMinError : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMinError, GoProfileCircleMinError)

                /// <summary>Initializes a new instance of the GoProfileCircleMinError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMinError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a min error X measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMinErrorX : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMinErrorX, GoProfileCircleMinErrorX)

                /// <summary>Initializes a new instance of the GoProfileCircleMinErrorX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMinErrorX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a min error Z measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMinErrorZ : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMinErrorZ, GoProfileCircleMinErrorZ)

                /// <summary>Initializes a new instance of the GoProfileCircleMinErrorZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMinErrorZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a max error measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMaxError : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMaxError, GoProfileCircleMaxError)

                /// <summary>Initializes a new instance of the GoProfileCircleMaxError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMaxError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a max error X measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMaxErrorX : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMaxErrorX, GoProfileCircleMaxErrorX)

                /// <summary>Initializes a new instance of the GoProfileCircleMaxErrorX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMaxErrorX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a max error Z measurement for a Profile Circle Tool.</summary>
            public ref class GoProfileCircleMaxErrorZ : public GoMeasurement
            {
                KDeclareClass(GoProfileCircleMaxErrorZ, GoProfileCircleMaxErrorZ)

                /// <summary>Initializes a new instance of the GoProfileCircleMaxErrorZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileCircleMaxErrorZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a width value measurement for a Profile Dimension Tool.</summary>
            public ref class GoProfileDimWidth : public GoMeasurement
            {
                KDeclareClass(GoProfileDimWidth, GoProfileDimWidth)

                /// <summary>Initializes a new instance of the GoProfileDimWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDimWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables an absolute value result for the given measurement.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoProfileDimWidth_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileDimWidth_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a height value measurement for a Profile Dimension Tool.</summary>
            public ref class GoProfileDimHeight : public GoMeasurement
            {
                KDeclareClass(GoProfileDimHeight, GoProfileDimHeight)

                /// <summary>Initializes a new instance of the GoProfileDimHeight class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDimHeight(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables an absolute value result for the given measurement.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoProfileDimHeight_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileDimHeight_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a distance value measurement for a Profile Dimension Tool.</summary>
            public ref class GoProfileDimDistance : public GoMeasurement
            {
                KDeclareClass(GoProfileDimDistance, GoProfileDimDistance)

                /// <summary>Initializes a new instance of the GoProfileDimDistance class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDimDistance(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a center X value measurement for a Profile Dimension Tool.</summary>
            public ref class GoProfileDimCenterX : public GoMeasurement
            {
                KDeclareClass(GoProfileDimCenterX, GoProfileDimCenterX)

                /// <summary>Initializes a new instance of the GoProfileDimCenterX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDimCenterX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a center Z value measurement for a Profile Dimension Tool.</summary>
            public ref class GoProfileDimCenterZ : public GoMeasurement
            {
                KDeclareClass(GoProfileDimCenterZ, GoProfileDimCenterZ)

                /// <summary>Initializes a new instance of the GoProfileDimCenterZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileDimCenterZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an intersect X measurement for a Profile Intersect Tool.</summary>
            public ref class GoProfileIntersectX : public GoMeasurement
            {
                KDeclareClass(GoProfileIntersectX, GoProfileIntersectX)

                /// <summary>Initializes a new instance of the GoProfileIntersectX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileIntersectX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an intersect Z measurement for a Profile Intersect Tool.</summary>
            public ref class GoProfileIntersectZ : public GoMeasurement
            {
                KDeclareClass(GoProfileIntersectZ, GoProfileIntersectZ)

                /// <summary>Initializes a new instance of the GoProfileIntersectZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileIntersectZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an intersect angle measurement for a Profile Intersect Tool.</summary>
            public ref class GoProfileIntersectAngle : public GoMeasurement
            {
                KDeclareClass(GoProfileIntersectAngle, GoProfileIntersectAngle)

                /// <summary>Initializes a new instance of the GoProfileIntersectAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileIntersectAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables a result in the range of 0 to 180 instead of -90 to 90 degrees for the given measurement.</summary>
                property bool Range0to180Enabled
                {
                    bool get()           { return KToBool(::GoProfileIntersectAngle_Range0to180Enabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileIntersectAngle_EnableRange0to180(Handle, enable)); }
                }
            };

            /// <summary>Represents an X value measurement for a Profile Groove Tool.</summary>
            public ref class GoProfileGrooveX : public GoMeasurement
            {
                KDeclareClass(GoProfileGrooveX, GoProfileGrooveX)

                /// <summary>Initializes a new instance of the GoProfileGrooveX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileGrooveX(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The groove location.</summary>
                property GoProfileGrooveLocation Location
                {
                    GoProfileGrooveLocation get()           { return (GoProfileGrooveLocation) ::GoProfileGrooveX_Location(Handle); }
                    void set(GoProfileGrooveLocation value)  { KCheck(::GoProfileGrooveX_SetLocation(Handle, value)); }
                }

                /// <summary>The groove selection type.</summary>
                property GoProfileGrooveSelectType SelectType
                {
                    GoProfileGrooveSelectType get()           { return (GoProfileGrooveSelectType) ::GoProfileGrooveX_SelectType(Handle); }
                    void set(GoProfileGrooveSelectType value)  { KCheck(::GoProfileGrooveX_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected groove index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileGrooveX_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileGrooveX_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a Z value measurement for a Profile Groove tool.</summary>
            public ref class GoProfileGrooveZ : public GoMeasurement
            {
                KDeclareClass(GoProfileGrooveZ, GoProfileGrooveZ)

                /// <summary>Initializes a new instance of the GoProfileGrooveZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileGrooveZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The groove location.</summary>
                property GoProfileGrooveLocation Location
                {
                    GoProfileGrooveLocation get()           { return (GoProfileGrooveLocation) ::GoProfileGrooveZ_Location(Handle); }
                    void set(GoProfileGrooveLocation value)  { KCheck(::GoProfileGrooveZ_SetLocation(Handle, value)); }
                }

                /// <summary>The groove selection type.</summary>
                property GoProfileGrooveSelectType SelectType
                {
                    GoProfileGrooveSelectType get()           { return (GoProfileGrooveSelectType) ::GoProfileGrooveZ_SelectType(Handle); }
                    void set(GoProfileGrooveSelectType value)  { KCheck(::GoProfileGrooveZ_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected groove index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileGrooveZ_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileGrooveZ_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a width measurement for a Profile Groove tool.</summary>
            public ref class GoProfileGrooveWidth : public GoMeasurement
            {
                KDeclareClass(GoProfileGrooveWidth, GoProfileGrooveWidth)

                /// <summary>Initializes a new instance of the GoProfileGrooveWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileGrooveWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The groove selection type.</summary>
                property GoProfileGrooveSelectType SelectType
                {
                    GoProfileGrooveSelectType get()           { return (GoProfileGrooveSelectType) ::GoProfileGrooveWidth_SelectType(Handle); }
                    void set(GoProfileGrooveSelectType value)  { KCheck(::GoProfileGrooveWidth_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected groove index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileGrooveWidth_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileGrooveWidth_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a depth measurement for a Profile Groove tool.</summary>
            public ref class GoProfileGrooveDepth : public GoMeasurement
            {
                KDeclareClass(GoProfileGrooveDepth, GoProfileGrooveDepth)

                /// <summary>Initializes a new instance of the GoProfileGrooveDepth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileGrooveDepth(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The groove selection type.</summary>
                property GoProfileGrooveSelectType SelectType
                {
                    GoProfileGrooveSelectType get()           { return (GoProfileGrooveSelectType) ::GoProfileGrooveDepth_SelectType(Handle); }
                    void set(GoProfileGrooveSelectType value)  { KCheck(::GoProfileGrooveDepth_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected groove index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileGrooveDepth_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileGrooveDepth_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a standard deviation measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineStdDev : public GoMeasurement
            {
                KDeclareClass(GoProfileLineStdDev, GoProfileLineStdDev)

                /// <summary>Initializes a new instance of the GoProfileLineStdDev class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLineStdDev(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a minimum error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMinError : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMinError, GoProfileLineMinError)

                /// <summary>Initializes a new instance of the GoProfileLineMinError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLineMinError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a maximum error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMaxError : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMaxError, GoProfileLineMaxError)

                /// <summary>Initializes a new instance of the GoProfileLineMaxError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLineMaxError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an offset measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineOffset : public GoMeasurement
            {
                KDeclareClass(GoProfileLineOffset, GoProfileLineOffset)

                    /// <summary>Initializes a new instance of the GoProfileLineOffset class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineOffset(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an angle measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineAngle : public GoMeasurement
            {
                KDeclareClass(GoProfileLineAngle, GoProfileLineAngle)

                    /// <summary>Initializes a new instance of the GoProfileLineAngle class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a minimum X error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMinErrorX : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMinErrorX, GoProfileLineMinErrorX)

                    /// <summary>Initializes a new instance of the GoProfileLineMinErrorX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMinErrorX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a minimum Z error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMinErrorZ : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMinErrorZ, GoProfileLineMinErrorZ)

                    /// <summary>Initializes a new instance of the GoProfileLineMinErrorZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMinErrorZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a maximum X error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMaxErrorX : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMaxErrorX, GoProfileLineMaxErrorX)

                    /// <summary>Initializes a new instance of the GoProfileLineMaxErrorX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMaxErrorX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a maximum Z error measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLineMaxErrorZ : public GoMeasurement
            {
                KDeclareClass(GoProfileLineMaxErrorZ, GoProfileLineMaxErrorZ)

                    /// <summary>Initializes a new instance of the GoProfileLineMaxErrorZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMaxErrorZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a percentile measurement for a Profile Line Tool.</summary>
            public ref class GoProfileLinePercentile : public GoMeasurement
            {
                KDeclareClass(GoProfileLinePercentile, GoProfileLinePercentile)

                /// <summary>Initializes a new instance of the GoProfileLinePercentile class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLinePercentile(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The percent threshold.</summary>
                property k64f Percent
                {
                    k64f get()           { return (k64f) ::GoProfileLinePercentile_Percent(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileLinePercentile_SetPercent(Handle, value)); }
                }
            };

            /// <summary>Represents a gap measurement for a Profile Panel Tool.</summary>
            public ref class GoProfilePanelGap : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelGap, GoProfilePanelGap)

                /// <summary>Initializes a new instance of the GoProfilePanelGap class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePanelGap(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The gap axis.</summary>
                property GoProfileGapAxis Axis
                {
                    GoProfileGapAxis get()           { return (GoProfileGapAxis) ::GoProfilePanelGap_Axis(Handle); }
                    void set(GoProfileGapAxis value)  { KCheck(::GoProfilePanelGap_SetAxis(Handle, value)); }
                }
            };

            /// <summary>Represents a flush measurement for a Profile Panel Tool.</summary>
            public ref class GoProfilePanelFlush : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelFlush, GoProfilePanelFlush)

                /// <summary>Initializes a new instance of the GoProfilePanelFlush class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePanelFlush(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables the absolute value state.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoProfilePanelFlush_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfilePanelFlush_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a Left Gap X measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftGapX : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelLeftGapX, GoProfilePanelLeftGapX)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftGapX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftGapX(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Left Gap Z measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftGapZ : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelLeftGapZ, GoProfilePanelLeftGapZ)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftGapZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftGapZ(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Left Flush X measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftFlushX : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelLeftFlushX, GoProfilePanelLeftFlushX)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftFlushX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftFlushX(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Left Flush Z measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftFlushZ : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelLeftFlushZ, GoProfilePanelLeftFlushZ)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftFlushZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftFlushZ(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Left Surface Angle measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftSurfaceAngle : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelLeftSurfaceAngle, GoProfilePanelLeftSurfaceAngle)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftFlushZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftSurfaceAngle(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Right Gap X measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelRightGapX : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelRightGapX, GoProfilePanelRightGapX)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightGapX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightGapX(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Right Gap Z measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelRightGapZ : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelRightGapZ, GoProfilePanelRightGapZ)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightGapZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightGapZ(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Right Flush X measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelRightFlushX : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelRightFlushX, GoProfilePanelRightFlushX)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightFlushX class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightFlushX(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Right Flush Z measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelRightFlushZ : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelRightFlushZ, GoProfilePanelRightFlushZ)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightFlushZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightFlushZ(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents a Right Surface Angle measurement for a Profile Panel tool.</summary>
            public ref class GoProfilePanelRightSurfaceAngle : public GoMeasurement
            {
                KDeclareClass(GoProfilePanelRightSurfaceAngle, GoProfilePanelRightSurfaceAngle)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightFlushZ class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightSurfaceAngle(IntPtr handle) :GoMeasurement(handle){}
            };

            /// <summary>Represents an X measurement for a Profile Round Corner tool.</summary>
            public ref class GoProfileRoundCornerX : public GoMeasurement
            {
                KDeclareClass(GoProfileRoundCornerX, GoProfileRoundCornerX)

                /// <summary>Initializes a new instance of the GoProfileRoundCornerX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileRoundCornerX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an Z measurement for a Profile Round Corner tool.</summary>
            public ref class GoProfileRoundCornerZ : public GoMeasurement
            {
                KDeclareClass(GoProfileRoundCornerZ, GoProfileRoundCornerZ)

                /// <summary>Initializes a new instance of the GoProfileRoundCornerZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileRoundCornerZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an Angle measurement for a Profile Round Corner tool.</summary>
            public ref class GoProfileRoundCornerAngle : public GoMeasurement
            {
                KDeclareClass(GoProfileRoundCornerAngle, GoProfileRoundCornerAngle)

                /// <summary>Initializes a new instance of the GoProfileRoundCornerAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileRoundCornerAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Profile Position tool.</summary>
            public ref class GoProfilePositionX : public GoMeasurement
            {
                KDeclareClass(GoProfilePositionX, GoProfilePositionX)

                /// <summary>Initializes a new instance of the GoProfilePositionX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePositionX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an Z measurement for a Profile Position tool.</summary>
            public ref class GoProfilePositionZ : public GoMeasurement
            {
                KDeclareClass(GoProfilePositionZ, GoProfilePositionZ)

                /// <summary>Initializes a new instance of the GoProfilePositionZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePositionZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Profile Strip Tool.</summary>
            public ref class GoProfileStripX : public GoMeasurement
            {
                KDeclareClass(GoProfileStripX, GoProfileStripX)

                /// <summary>Initializes a new instance of the GoProfileStripX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileStripX(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The strip location.</summary>
                property GoProfileStripLocation Location
                {
                    GoProfileStripLocation get()           { return (GoProfileStripLocation) ::GoProfileStripX_Location(Handle); }
                    void set(GoProfileStripLocation value)  { KCheck(::GoProfileStripX_SetLocation(Handle, value)); }
                }

                /// <summary>The strip selection type.</summary>
                property GoProfileStripSelectType SelectType
                {
                    GoProfileStripSelectType get()           { return (GoProfileStripSelectType) ::GoProfileStripX_SelectType(Handle); }
                    void set(GoProfileStripSelectType value)  { KCheck(::GoProfileStripX_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected strip index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileStripX_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileStripX_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents an Z measurement for a Profile Strip Tool.</summary>
            public ref class GoProfileStripZ : public GoMeasurement
            {
                KDeclareClass(GoProfileStripZ, GoProfileStripZ)

                /// <summary>Initializes a new instance of the GoProfileStripZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileStripZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The strip location.</summary>
                property GoProfileStripLocation Location
                {
                    GoProfileStripLocation get()           { return (GoProfileStripLocation) ::GoProfileStripZ_Location(Handle); }
                    void set(GoProfileStripLocation value)  { KCheck(::GoProfileStripZ_SetLocation(Handle, value)); }
                }

                /// <summary>The strip selection type.</summary>
                property GoProfileStripSelectType SelectType
                {
                    GoProfileStripSelectType get()           { return (GoProfileStripSelectType) ::GoProfileStripZ_SelectType(Handle); }
                    void set(GoProfileStripSelectType value)  { KCheck(::GoProfileStripZ_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected strip index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileStripZ_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileStripZ_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a width measurement for a Profile Strip Tool.</summary>
            public ref class GoProfileStripWidth : public GoMeasurement
            {
                KDeclareClass(GoProfileStripWidth, GoProfileStripWidth)

                /// <summary>Initializes a new instance of the GoProfileStripWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileStripWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The strip selection type.</summary>
                property GoProfileStripSelectType SelectType
                {
                    GoProfileStripSelectType get()           { return (GoProfileStripSelectType) ::GoProfileStripWidth_SelectType(Handle); }
                    void set(GoProfileStripSelectType value)  { KCheck(::GoProfileStripWidth_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected strip index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileStripWidth_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileStripWidth_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents a height measurement for a Profile Strip Tool.</summary>
            public ref class GoProfileStripHeight : public GoMeasurement
            {
                KDeclareClass(GoProfileStripHeight, GoProfileStripHeight)

                /// <summary>Initializes a new instance of the GoProfileStripHeight class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileStripHeight(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The strip location.</summary>
                property GoProfileStripLocation Location
                {
                    GoProfileStripLocation get()           { return (GoProfileStripLocation) ::GoProfileStripHeight_Location(Handle); }
                    void set(GoProfileStripLocation value)  { KCheck(::GoProfileStripHeight_SetLocation(Handle, value)); }
                }

                /// <summary>The strip selection type.</summary>
                property GoProfileStripSelectType SelectType
                {
                    GoProfileStripSelectType get()           { return (GoProfileStripSelectType) ::GoProfileStripHeight_SelectType(Handle); }
                    void set(GoProfileStripSelectType value)  { KCheck(::GoProfileStripHeight_SetSelectType(Handle, value)); }
                }

                /// <summary>The selected strip index.</summary>
                property k32u SelectIndex
                {
                    k32u get()           { return (k32u) ::GoProfileStripHeight_SelectIndex(Handle); }
                    void set(k32u value)  { KCheck(::GoProfileStripHeight_SetSelectIndex(Handle, value)); }
                }
            };

            /// <summary>Represents an X measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxX, GoSurfaceBoxX)

                /// <summary>Initializes a new instance of the GoSurfaceBoxX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxY, GoSurfaceBoxY)

                /// <summary>Initializes a new instance of the GoSurfaceBoxY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxZ, GoSurfaceBoxZ)

                /// <summary>Initializes a new instance of the GoSurfaceBoxZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z-angle measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxZAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxZAngle, GoSurfaceBoxZAngle)

                /// <summary>Initializes a new instance of the GoSurfaceBoxZAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxZAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global X measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxGlobalX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxGlobalX, GoSurfaceBoxGlobalX)

                /// <summary>Initializes a new instance of the GoSurfaceBoxGlobalX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxGlobalX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global Y measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxGlobalY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxGlobalY, GoSurfaceBoxGlobalY)

                /// <summary>Initializes a new instance of the GoSurfaceBoxGlobalY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxGlobalY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a global Z angle measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxGlobalZAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxGlobalZAngle, GoSurfaceBoxGlobalZAngle)

                /// <summary>Initializes a new instance of the GoSurfaceBoxGlobalZAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxGlobalZAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a length measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxLength : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxLength, GoSurfaceBoxLength)

                /// <summary>Initializes a new instance of the GoSurfaceBoxLength class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxLength(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a width measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxWidth : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxWidth, GoSurfaceBoxWidth)

                /// <summary>Initializes a new instance of the GoSurfaceBoxWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a height measurement for a Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoxHeight : public GoMeasurement
            {
                KDeclareClass(GoSurfaceBoxHeight, GoSurfaceBoxHeight)

                /// <summary>Initializes a new instance of the GoSurfaceBoxHeight class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceBoxHeight(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a width value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimWidth : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimWidth, GoSurfaceDimWidth)

                /// <summary>Initializes a new instance of the GoSurfaceDimWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables an absolute value result for the given measurement.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceDimWidth_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceDimWidth_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a height value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimHeight : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimHeight, GoSurfaceDimHeight)

                /// <summary>Initializes a new instance of the GoSurfaceDimHeight class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimHeight(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables an absolute value result for the given measurement.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceDimHeight_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceDimHeight_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a length value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimLength : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimLength, GoSurfaceDimLength)

                /// <summary>Initializes a new instance of the GoSurfaceDimLength class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimLength(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>Enables or disables an absolute value result for the given measurement.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceDimLength_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceDimLength_EnableAbsolute(Handle, enable)); }
                }
            };

            /// <summary>Represents a distance value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimDistance : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimDistance, GoSurfaceDimDistance)

                /// <summary>Initializes a new instance of the GoSurfaceDimDistance class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimDistance(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a distance value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimPlaneDistance : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimPlaneDistance, GoSurfaceDimPlaneDistance)

                /// <summary>Initializes a new instance of the GoSurfaceDimPlaneDistance class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimPlaneDistance(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a center X value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimCenterX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimCenterX, GoSurfaceDimCenterX)

                /// <summary>Initializes a new instance of the GoSurfaceDimCenterX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimCenterX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a center Y value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimCenterY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimCenterY, GoSurfaceDimCenterY)

                /// <summary>Initializes a new instance of the GoSurfaceDimCenterY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimCenterY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a center Z value measurement for a Surface Dimension Tool.</summary>
            public ref class GoSurfaceDimCenterZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceDimCenterZ, GoSurfaceDimCenterZ)

                /// <summary>Initializes a new instance of the GoSurfaceDimCenterZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceDimCenterZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a major value measurement for a Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseMajor : public GoMeasurement
            {
                KDeclareClass(GoSurfaceEllipseMajor, GoSurfaceEllipseMajor)

                /// <summary>Initializes a new instance of the GoSurfaceEllipseMajor class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceEllipseMajor(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a minor value measurement for a Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseMinor : public GoMeasurement
            {
                KDeclareClass(GoSurfaceEllipseMinor, GoSurfaceEllipseMinor)

                /// <summary>Initializes a new instance of the GoSurfaceEllipseMinor class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceEllipseMinor(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a ratio measurement for a Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseRatio : public GoMeasurement
            {
                KDeclareClass(GoSurfaceEllipseRatio, GoSurfaceEllipseRatio)

                /// <summary>Initializes a new instance of the GoSurfaceEllipseRatio class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceEllipseRatio(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z-angle measurement for a Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseZAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceEllipseZAngle, GoSurfaceEllipseZAngle)

                /// <summary>Initializes a new instance of the GoSurfaceEllipseZAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceEllipseZAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Surface Hole Tool.</summary>
            public ref class GoSurfaceHoleX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceHoleX, GoSurfaceHoleX)

                /// <summary>Initializes a new instance of the GoSurfaceHoleX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceHoleX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y measurement for a Surface Hole Tool.</summary>
            public ref class GoSurfaceHoleY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceHoleY, GoSurfaceHoleY)

                /// <summary>Initializes a new instance of the GoSurfaceHoleY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceHoleY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z measurement for a Surface Hole Tool.</summary>
            public ref class GoSurfaceHoleZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceHoleZ, GoSurfaceHoleZ)

                /// <summary>Initializes a new instance of the GoSurfaceHoleZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceHoleZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a radius measurement for a Surface Hole Tool.</summary>
            public ref class GoSurfaceHoleRadius : public GoMeasurement
            {
                KDeclareClass(GoSurfaceHoleRadius, GoSurfaceHoleRadius)

                /// <summary>Initializes a new instance of the GoSurfaceHoleRadius class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceHoleRadius(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningX, GoSurfaceOpeningX)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningY, GoSurfaceOpeningY)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningZ, GoSurfaceOpeningZ)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a width measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningWidth : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningWidth, GoSurfaceOpeningWidth)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningWidth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningWidth(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a length measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningLength : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningLength, GoSurfaceOpeningLength)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningLength class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningLength(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an angle measurement for a Surface Opening Tool.</summary>
            public ref class GoSurfaceOpeningAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceOpeningAngle, GoSurfaceOpeningAngle)

                /// <summary>Initializes a new instance of the GoSurfaceOpeningAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceOpeningAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X-angle measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneXAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneXAngle, GoSurfacePlaneXAngle)

                /// <summary>Initializes a new instance of the GoSurfacePlaneXAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneXAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y-angle measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneYAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneYAngle, GoSurfacePlaneYAngle)

                /// <summary>Initializes a new instance of the GoSurfacePlaneYAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneYAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z-offset measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneZOffset : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneZOffset, GoSurfacePlaneZOffset)

                /// <summary>Initializes a new instance of the GoSurfacePlaneZOffset class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneZOffset(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Standard Deviation measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneStdDev : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneStdDev, GoSurfacePlaneStdDev)

                /// <summary>Initializes a new instance of the GoSurfacePlaneStdDev class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneStdDev(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Minimum Error measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneMinError : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneMinError, GoSurfacePlaneMinError)

                /// <summary>Initializes a new instance of the GoSurfacePlaneMinError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneMinError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Maximum Error measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneMaxError : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneMaxError, GoSurfacePlaneMaxError)

                /// <summary>Initializes a new instance of the GoSurfacePlaneMaxError class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneMaxError(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a X Normal measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneXNormal : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneXNormal, GoSurfacePlaneXNormal)

                /// <summary>Initializes a new instance of the GoSurfacePlaneXNormal class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePlaneXNormal(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };
            /// <summary>Represents a X Normal measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneYNormal : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneYNormal, GoSurfacePlaneYNormal)

                    /// <summary>Initializes a new instance of the GoSurfacePlaneXNormal class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfacePlaneYNormal(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };
            /// <summary>Represents a X Normal measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneZNormal : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneZNormal, GoSurfacePlaneZNormal)

                    /// <summary>Initializes a new instance of the GoSurfacePlaneXNormal class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfacePlaneZNormal(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };
            /// <summary>Represents a X Normal measurement for a Surface Plane Tool.</summary>
            public ref class GoSurfacePlaneDistance : public GoMeasurement
            {
                KDeclareClass(GoSurfacePlaneDistance, GoSurfacePlaneDistance)

                    /// <summary>Initializes a new instance of the GoSurfacePlaneDistance class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfacePlaneDistance(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X measurement for a Surface Position Tool.</summary>
            public ref class GoSurfacePositionX : public GoMeasurement
            {
                KDeclareClass(GoSurfacePositionX, GoSurfacePositionX)

                /// <summary>Initializes a new instance of the GoSurfacePositionX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePositionX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y measurement for a Surface Position Tool.</summary>
            public ref class GoSurfacePositionY : public GoMeasurement
            {
                KDeclareClass(GoSurfacePositionY, GoSurfacePositionY)

                /// <summary>Initializes a new instance of the GoSurfacePositionY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePositionY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z measurement for a Surface Position Tool.</summary>
            public ref class GoSurfacePositionZ : public GoMeasurement
            {
                KDeclareClass(GoSurfacePositionZ, GoSurfacePositionZ)

                /// <summary>Initializes a new instance of the GoSurfacePositionZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePositionZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a base X measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudBaseX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudBaseX, GoSurfaceStudBaseX)

                /// <summary>Initializes a new instance of the GoSurfaceStudBaseX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudBaseX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a base Y measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudBaseY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudBaseY, GoSurfaceStudBaseY)

                /// <summary>Initializes a new instance of the GoSurfaceStudBaseY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudBaseY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a base Z measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudBaseZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudBaseZ, GoSurfaceStudBaseZ)

                /// <summary>Initializes a new instance of the GoSurfaceStudBaseZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudBaseZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a tip X measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudTipX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudTipX, GoSurfaceStudTipX)

                /// <summary>Initializes a new instance of the GoSurfaceStudTipX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudTipX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a tip Y measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudTipY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudTipY, GoSurfaceStudTipY)

                /// <summary>Initializes a new instance of the GoSurfaceStudTipY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudTipY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a tip Z measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudTipZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudTipZ, GoSurfaceStudTipZ)

                /// <summary>Initializes a new instance of the GoSurfaceStudTipZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudTipZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a radius measurement for a Surface Stud Tool.</summary>
            public ref class GoSurfaceStudRadius : public GoMeasurement
            {
                KDeclareClass(GoSurfaceStudRadius, GoSurfaceStudRadius)

                /// <summary>Initializes a new instance of the GoSurfaceStudRadius class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceStudRadius(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The radius offset.</summary>
                property k64f RadiusOffset
                {
                    k64f get()           { return (k64f) ::GoSurfaceStudRadius_RadiusOffset(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceStudRadius_SetRadiusOffset(Handle, value)); }
                }
            };

            /// <summary>Represents a volume measurement for a Surface Volume Tool.</summary>
            public ref class GoSurfaceVolumeVolume : public GoMeasurement
            {
                KDeclareClass(GoSurfaceVolumeVolume, GoSurfaceVolumeVolume)

                /// <summary>Initializes a new instance of the GoSurfaceVolumeVolume class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceVolumeVolume(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an area measurement for a Surface Volume Tool.</summary>
            public ref class GoSurfaceVolumeArea : public GoMeasurement
            {
                KDeclareClass(GoSurfaceVolumeArea, GoSurfaceVolumeArea)

                /// <summary>Initializes a new instance of the GoSurfaceVolumeArea class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceVolumeArea(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a thickness measurement for a Surface Volume Tool.</summary>
            public ref class GoSurfaceVolumeThickness : public GoMeasurement
            {
                KDeclareClass(GoSurfaceVolumeThickness, GoSurfaceVolumeThickness)

                /// <summary>Initializes a new instance of the GoSurfaceVolumeThickness class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceVolumeThickness(IntPtr handle)
                    : GoMeasurement(handle)
                {}

                /// <summary>The location.</summary>
                property GoSurfaceLocation Location
                {
                    GoSurfaceLocation get()           { return (GoSurfaceLocation) ::GoSurfaceVolumeThickness_Location(Handle); }
                    void set(GoSurfaceLocation value)  { KCheck(::GoSurfaceVolumeThickness_SetLocation(Handle, value)); }
                }
            };

            /// <summary>Represents an X position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleX : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleX, GoSurfaceCountersunkHoleX)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleX class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleX(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleY : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleY, GoSurfaceCountersunkHoleY)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleY class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleY(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Z position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleZ : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleZ, GoSurfaceCountersunkHoleZ)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleZ class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleZ(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an Outer Radius position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleOuterRadius : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleOuterRadius, GoSurfaceCountersunkHoleOuterRadius)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleOuterRadius class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleOuterRadius(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Depth position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleDepth : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleDepth, GoSurfaceCountersunkHoleDepth)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleDepth class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleDepth(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an Bevel Radius position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleBevelRadius : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleBevelRadius, GoSurfaceCountersunkHoleBevelRadius)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleBevelRadius class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleBevelRadius(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Bevel Angle measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleBevelAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleBevelAngle, GoSurfaceCountersunkHoleBevelAngle)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleBevelAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleBevelAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents an X Angle position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleXAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleXAngle, GoSurfaceCountersunkHoleXAngle)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleXAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleXAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a Y Angle position measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleYAngle : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleYAngle, GoSurfaceCountersunkHoleYAngle)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleYAngle class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleYAngle(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a counterbore depth measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleCounterboreDepth : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleCounterboreDepth, GoSurfaceCountersunkHoleCounterboreDepth)

                    /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleCounterboreDepth class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceCountersunkHoleCounterboreDepth(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a axis tilt measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleAxisTilt : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleAxisTilt, GoSurfaceCountersunkHoleAxisTilt)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleAxisTilt class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleAxisTilt(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a axis orientation measurement for a Surface Counter Sunk Hole Tool.</summary>
            public ref class GoSurfaceCountersunkHoleAxisOrientation : public GoMeasurement
            {
                KDeclareClass(GoSurfaceCountersunkHoleAxisOrientation, GoSurfaceCountersunkHoleAxisOrientation)

                /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleAxisOrientation class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceCountersunkHoleAxisOrientation(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            /// <summary>Represents a script output for a Script Tool.</summary>
            public ref class GoScriptOutput : public GoMeasurement
            {
                KDeclareClass(GoScriptOutput, GoScriptOutput)

                /// <summary>Initializes a new instance of the GoScriptOutput class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoScriptOutput(IntPtr handle)
                    : GoMeasurement(handle)
                {}
            };

            private ref class GoMeasurements abstract sealed
            {
            internal:
                static GoMeasurement^ GetMeasurementInstance(::GoMeasurement measurement)
                {
                    if (kObject_Is(measurement, kTypeOf(GoExtMeasurement))) { return KToObject<GoExtMeasurement^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoRangePositionZ))) { return KToObject<GoRangePositionZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoRangeThicknessThickness))) { return KToObject<GoRangeThicknessThickness^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileAreaArea))) { return KToObject<GoProfileAreaArea^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileAreaCentroidX))) { return KToObject<GoProfileAreaCentroidX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileAreaCentroidZ))) { return KToObject<GoProfileAreaCentroidZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxX))) { return KToObject<GoProfileBoxX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxZ))) { return KToObject<GoProfileBoxZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxWidth))) { return KToObject<GoProfileBoxWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxHeight))) { return KToObject<GoProfileBoxHeight^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxGlobalX))) { return KToObject<GoProfileBoxGlobalX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxGlobalY))) { return KToObject<GoProfileBoxGlobalY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBoxGlobalAngle))) { return KToObject<GoProfileBoxGlobalAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBridgeValueBridgeValue))) { return KToObject<GoProfileBridgeValueBridgeValue^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBridgeValueAngle))) { return KToObject<GoProfileBridgeValueAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBridgeValueWindow))) { return KToObject<GoProfileBridgeValueWindow^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileBridgeValueStdDev))) { return KToObject<GoProfileBridgeValueStdDev^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleX))) { return KToObject<GoProfileCircleX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleZ))) { return KToObject<GoProfileCircleZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleRadius))) { return KToObject<GoProfileCircleRadius^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleStdDev))) { return KToObject<GoProfileCircleStdDev^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMinError))) { return KToObject<GoProfileCircleMinError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMinErrorX))) { return KToObject<GoProfileCircleMinErrorX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMinErrorZ))) { return KToObject<GoProfileCircleMinErrorZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMaxError))) { return KToObject<GoProfileCircleMaxError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMaxErrorX))) { return KToObject<GoProfileCircleMaxErrorX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileCircleMaxErrorZ))) { return KToObject<GoProfileCircleMaxErrorZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileDimWidth))) { return KToObject<GoProfileDimWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileDimHeight))) { return KToObject<GoProfileDimHeight^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileDimDistance))) { return KToObject<GoProfileDimDistance^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileDimCenterX))) { return KToObject<GoProfileDimCenterX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileDimCenterZ))) { return KToObject<GoProfileDimCenterZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileIntersectX))) { return KToObject<GoProfileIntersectX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileIntersectZ))) { return KToObject<GoProfileIntersectZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileIntersectAngle))) { return KToObject<GoProfileIntersectAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileGrooveX))) { return KToObject<GoProfileGrooveX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileGrooveZ))) { return KToObject<GoProfileGrooveZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileGrooveWidth))) { return KToObject<GoProfileGrooveWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileGrooveDepth))) { return KToObject<GoProfileGrooveDepth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineStdDev))) { return KToObject<GoProfileLineStdDev^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMinError))) { return KToObject<GoProfileLineMinError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMaxError))) { return KToObject<GoProfileLineMaxError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLinePercentile))) { return KToObject<GoProfileLinePercentile^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineOffset))) { return KToObject<GoProfileLineOffset^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineAngle))) { return KToObject<GoProfileLineAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMinErrorX))) { return KToObject<GoProfileLineMinErrorX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMaxErrorX))) { return KToObject<GoProfileLineMaxErrorX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMinErrorZ))) { return KToObject<GoProfileLineMinErrorZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileLineMaxErrorZ))) { return KToObject<GoProfileLineMaxErrorZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelGap))) { return KToObject<GoProfilePanelGap^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelFlush))) { return KToObject<GoProfilePanelFlush^>(measurement); }
                    // GOC-14386 Because the following 10 measurements accessor functions were not defined it was not possible to retrieve them.
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelLeftGapX))) { return KToObject<GoProfilePanelLeftGapX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelLeftGapZ))) { return KToObject<GoProfilePanelLeftGapZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelLeftFlushX))) { return KToObject<GoProfilePanelLeftFlushX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelLeftFlushZ))) { return KToObject<GoProfilePanelLeftFlushZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelLeftSurfaceAngle))) { return KToObject<GoProfilePanelLeftSurfaceAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelRightGapX))) { return KToObject<GoProfilePanelRightGapX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelRightGapZ))) { return KToObject<GoProfilePanelRightGapZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelRightFlushX))) { return KToObject<GoProfilePanelRightFlushX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelRightFlushZ))) { return KToObject<GoProfilePanelRightFlushZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePanelRightSurfaceAngle))) { return KToObject<GoProfilePanelRightSurfaceAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileRoundCornerX))) { return KToObject<GoProfileRoundCornerX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileRoundCornerZ))) { return KToObject<GoProfileRoundCornerZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileRoundCornerAngle))) { return KToObject<GoProfileRoundCornerAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePositionX))) { return KToObject<GoProfilePositionX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfilePositionZ))) { return KToObject<GoProfilePositionZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileStripX))) { return KToObject<GoProfileStripX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileStripZ))) { return KToObject<GoProfileStripZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileStripWidth))) { return KToObject<GoProfileStripWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoProfileStripHeight))) { return KToObject<GoProfileStripHeight^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxX))) { return KToObject<GoSurfaceBoxX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxY))) { return KToObject<GoSurfaceBoxY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxZ))) { return KToObject<GoSurfaceBoxZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxZAngle))) { return KToObject<GoSurfaceBoxZAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalX))) { return KToObject<GoSurfaceBoxGlobalX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalY))) { return KToObject<GoSurfaceBoxGlobalY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxGlobalZAngle))) { return KToObject<GoSurfaceBoxGlobalZAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxLength))) { return KToObject<GoSurfaceBoxLength^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxWidth))) { return KToObject<GoSurfaceBoxWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceBoxHeight))) { return KToObject<GoSurfaceBoxHeight^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimWidth))) { return KToObject<GoSurfaceDimWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimHeight))) { return KToObject<GoSurfaceDimHeight^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimLength))) { return KToObject<GoSurfaceDimLength^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimDistance))) { return KToObject<GoSurfaceDimDistance^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimPlaneDistance))) { return KToObject<GoSurfaceDimPlaneDistance^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimCenterX))) { return KToObject<GoSurfaceDimCenterX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimCenterY))) { return KToObject<GoSurfaceDimCenterY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceDimCenterZ))) { return KToObject<GoSurfaceDimCenterZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseMajor))) { return KToObject<GoSurfaceEllipseMajor^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseMinor))) { return KToObject<GoSurfaceEllipseMinor^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseRatio))) { return KToObject<GoSurfaceEllipseRatio^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceEllipseZAngle))) { return KToObject<GoSurfaceEllipseZAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceHoleX))) { return KToObject<GoSurfaceHoleX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceHoleY))) { return KToObject<GoSurfaceHoleY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceHoleZ))) { return KToObject<GoSurfaceHoleZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceHoleRadius))) { return KToObject<GoSurfaceHoleRadius^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningX))) { return KToObject<GoSurfaceOpeningX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningY))) { return KToObject<GoSurfaceOpeningY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningZ))) { return KToObject<GoSurfaceOpeningZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningWidth))) { return KToObject<GoSurfaceOpeningWidth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningLength))) { return KToObject<GoSurfaceOpeningLength^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceOpeningAngle))) { return KToObject<GoSurfaceOpeningAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneXAngle))) { return KToObject<GoSurfacePlaneXAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneYAngle))) { return KToObject<GoSurfacePlaneYAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneZOffset))) { return KToObject<GoSurfacePlaneZOffset^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneStdDev))) { return KToObject<GoSurfacePlaneStdDev^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneMinError))) { return KToObject<GoSurfacePlaneMinError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneMaxError))) { return KToObject<GoSurfacePlaneMaxError^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneXNormal))) { return KToObject<GoSurfacePlaneXNormal^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneYNormal))) { return KToObject<GoSurfacePlaneYNormal^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneZNormal))) { return KToObject<GoSurfacePlaneZNormal^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePlaneDistance))) { return KToObject<GoSurfacePlaneDistance^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePositionX))) { return KToObject<GoSurfacePositionX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePositionY))) { return KToObject<GoSurfacePositionY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfacePositionZ))) { return KToObject<GoSurfacePositionZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseX))) { return KToObject<GoSurfaceStudBaseX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseY))) { return KToObject<GoSurfaceStudBaseY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudBaseZ))) { return KToObject<GoSurfaceStudBaseZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipX))) { return KToObject<GoSurfaceStudTipX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipY))) { return KToObject<GoSurfaceStudTipY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudTipZ))) { return KToObject<GoSurfaceStudTipZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceStudRadius))) { return KToObject<GoSurfaceStudRadius^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeVolume))) { return KToObject<GoSurfaceVolumeVolume^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeArea))) { return KToObject<GoSurfaceVolumeArea^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceVolumeThickness))) { return KToObject<GoSurfaceVolumeThickness^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleX))) { return KToObject<GoSurfaceCountersunkHoleX^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleY))) { return KToObject<GoSurfaceCountersunkHoleY^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleZ))) { return KToObject<GoSurfaceCountersunkHoleZ^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleOuterRadius))) { return KToObject<GoSurfaceCountersunkHoleOuterRadius^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleDepth))) { return KToObject<GoSurfaceCountersunkHoleDepth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleBevelRadius))) { return KToObject<GoSurfaceCountersunkHoleBevelRadius^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleBevelAngle))) { return KToObject<GoSurfaceCountersunkHoleBevelAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleXAngle))) { return KToObject<GoSurfaceCountersunkHoleXAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleYAngle))) { return KToObject<GoSurfaceCountersunkHoleYAngle^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleCounterboreDepth))) { return KToObject<GoSurfaceCountersunkHoleCounterboreDepth^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleAxisTilt))) { return KToObject<GoSurfaceCountersunkHoleAxisTilt^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoSurfaceCountersunkHoleAxisOrientation))) { return KToObject<GoSurfaceCountersunkHoleAxisOrientation^>(measurement); }
                    if (kObject_Is(measurement, kTypeOf(GoScriptOutput))) { return KToObject<GoScriptOutput^>(measurement); }
                    return nullptr;
                }
            };
        }
    }
}

#endif
