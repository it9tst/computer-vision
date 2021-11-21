//
// GoProfileToolUtils.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_TOOL_UTILS_H
#define GO_SDK_NET_TOOL_UTILS_H

#include <GoSdk/Tools/GoProfileToolUtils.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a profile region used in various profile tools.</summary>
            public ref class GoProfileRegion : public KObject
            {
                KDeclareClass(GoProfileRegion, GoProfileRegion)

                /// <summary>Initializes a new instance of the GoProfileRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileRegion(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileRegion(GoSensor^ sensor)
                {
                    ::GoProfileRegion handle = kNULL;

                    KCheck(::GoProfileRegion_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileRegion(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileRegion handle = kNULL;

                    KCheck(::GoProfileRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The X position.</summary>
                property k64f X
                {
                    k64f get()           { return (k64f) ::GoProfileRegion_X(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileRegion_SetX(Handle, value)); }
                }

                /// <summary>The Z position.</summary>
                property k64f Z
                {
                    k64f get()           { return (k64f) ::GoProfileRegion_Z(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileRegion_SetZ(Handle, value)); }
                }

                /// <summary>The width position.</summary>
                property k64f Width
                {
                    k64f get()           { return (k64f) ::GoProfileRegion_Width(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileRegion_SetWidth(Handle, value)); }
                }

                /// <summary>The height position.</summary>
                property k64f Height
                {
                    k64f get()           { return (k64f) ::GoProfileRegion_Height(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileRegion_SetHeight(Handle, value)); }
                }
            };

            /// <summary>Represents a profile feature used in various profile tools.</summary>
            public ref class GoProfileFeature : public KObject
            {
                KDeclareClass(GoProfileFeature, GoProfileFeature)

                /// <summary>Initializes a new instance of the GoProfileFeature class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileFeature(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileFeature class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileFeature(GoSensor^ sensor)
                {
                    ::GoProfileFeature handle = kNULL;

                    KCheck(::GoProfileFeature_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileFeature(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileFeature(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileFeature handle = kNULL;

                    KCheck(::GoProfileFeature_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The profile feature type.</summary>
                property GoProfileFeatureType FeatureType
                {
                    GoProfileFeatureType get()           { return (GoProfileFeatureType) ::GoProfileFeature_Type(Handle); }
                    void set(GoProfileFeatureType type)  { KCheck(::GoProfileFeature_SetType(Handle, type)); }
                }

                /// <summary>The profile feature region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return gcnew GoProfileRegion(IntPtr(::GoProfileFeature_Region(Handle))); }
                }

                /// <summary>Enables or disables the tool region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileFeature_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileFeature_EnableRegion(Handle, enable)); }
                }
            };

            /// <summary>Represents a profile line region used in various profile tools.</summary>
            public ref class GoProfileLineFittingRegion : public KObject
            {
                KDeclareClass(GoProfileLineFittingRegion , GoProfileLineFittingRegion )

                /// <summary>Initializes a new instance of the GoProfileLineFittingRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileLineFittingRegion (IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileLineFittingRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileLineFittingRegion (GoSensor^ sensor)
                {
                    ::GoProfileLineFittingRegion  handle = kNULL;

                    KCheck(::GoProfileLineFittingRegion_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileLineFittingRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileLineFittingRegion (GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileLineFittingRegion  handle = kNULL;

                    KCheck(::GoProfileLineFittingRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The tool's region count.</summary>
                property k32u RegionCount
                {
                    k32u get()           { return (k32u) ::GoProfileLineFittingRegion_RegionCount(Handle); }
                    void set(k32u value) {KCheck(::GoProfileLineFittingRegion_SetRegionCount(Handle, (kSize) value)); }
                }

                /// <summary>Gets the profile region based on the given index.</summary>
                /// <param name="index">The index of the profile region to return.</param>
                /// <returns>The profile region.</returns>
                GoProfileRegion^ GetRegion(k64s index)
                {
                    return KToObject<GoProfileRegion^>(::GoProfileLineFittingRegion_RegionAt(Handle, (kSize)index));
                }

                /// <summary>Enables or disables the region.</summary>
                /// <param name="index">The region index of which to modify.</param>
                /// <param name="enable">true to use the region, false otherwise.</param>
                void SetRegionEnabled(k64s index, bool enable)
                {
                    KCheck(::GoProfileLineFittingRegion_EnableRegionAt(Handle, (kSize)index, enable));
                }

                /// <summary>Indicates whether the region is enabled.</summary>
                /// <param name="index">The region index of which to modify.</param>
                /// <returns>true if the region is enabled and false otherwise.</returns>
                bool GetRegionEnabled(k64s index)
                {
                    return KToBool(::GoProfileLineFittingRegion_RegionEnabledAt(Handle, (kSize)index));
                }
            };

            /// <summary>Represents a profile edge used in various profile tools.</summary>
            public ref class GoProfileEdge : public KObject
            {
                KDeclareClass(GoProfileEdge, GoProfileEdge)

                /// <summary>Initializes a new instance of the GoProfileEdge class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileEdge(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileEdge class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoProfileEdge(GoSensor^ sensor)
                {
                    ::GoProfileEdge handle = kNULL;

                    KCheck(::GoProfileEdge_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileEdge(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileEdge(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoProfileEdge handle = kNULL;

                    KCheck(::GoProfileEdge_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The profile edge type.</summary>
                property GoProfileEdgeType EdgeType
                {
                    GoProfileEdgeType get()           { return (GoProfileEdgeType) ::GoProfileEdge_Type(Handle); }
                    void set(GoProfileEdgeType type)  { KCheck(::GoProfileEdge_SetType(Handle, type)); }
                }

                /// <summary>The profile region.</summary>
                property GoProfileRegion^ Region
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoProfileEdge_Region(Handle)); }
                }

                /// <summary>Enables or disables the region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoProfileEdge_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoProfileEdge_EnableRegion(Handle, enable)); }
                }

                /// <summary>The maximum void width.</summary>
                property k64f VoidWidthMax
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_VoidWidthMax(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetVoidWidthMax(Handle, value)); }
                }

                /// <summary>The minimum depth.</summary>
                property k64f DepthMin
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_DepthMin(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetDepthMin(Handle, value)); }
                }

                /// <summary>The surface width.</summary>
                property k64f SurfaceWidth
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_SurfaceWidth(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetSurfaceWidth(Handle, value)); }
                }

                /// <summary>The surface offset.</summary>
                property k64f SurfaceOffset
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_SurfaceOffset(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetSurfaceOffset(Handle, value)); }
                }

                /// <summary>The nominal radius.</summary>
                property k64f NominalRadius
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_NominalRadius(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetNominalRadius(Handle, value)); }
                }

                /// <summary>The edge angle.</summary>
                property k64f EdgeAngle
                {
                    k64f get()           { return (k64f) ::GoProfileEdge_EdgeAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoProfileEdge_SetEdgeAngle(Handle, value)); }
                }
            };
        }
    }
}

#endif
