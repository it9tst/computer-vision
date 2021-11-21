// 
// GoSurfaceToolUtils.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SURFACE_TOOL_UTILS_H
#define GO_SDK_NET_SURFACE_TOOL_UTILS_H

#include <GoSdk/Tools/GoSurfaceToolUtils.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a two dimensional surface tool region.</summary>
            public ref class GoSurfaceRegion2d : public KObject
            {
                KDeclareClass(GoSurfaceRegion2d, GoSurfaceRegion2d)
            
                /// <summary>Initializes a new instance of the GoSurfaceRegion2d class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceRegion2d(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoSurfaceRegion2d class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceRegion2d(GoSensor^ sensor)
                {
                    ::GoSurfaceRegion2d handle = kNULL;
            
                    KCheck(::GoSurfaceRegion2d_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoSurfaceRegion2d(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceRegion2d(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceRegion2d handle = kNULL;
            
                    KCheck(::GoSurfaceRegion2d_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The X position.</summary>
                property k64f X
                {
                    k64f get()           { return (k64f) ::GoSurfaceRegion2d_X(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceRegion2d_SetX(Handle, value)); }
                }
            
                /// <summary>The Y position.</summary>
                property k64f Y
                {
                    k64f get()           { return (k64f) ::GoSurfaceRegion2d_Y(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceRegion2d_SetY(Handle, value)); }
                }
            
                /// <summary>The width.</summary>
                property k64f Width
                {
                    k64f get()           { return (k64f) ::GoSurfaceRegion2d_Width(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceRegion2d_SetWidth(Handle, value)); }
                }
            
                /// <summary>The length.</summary>
                property k64f Length
                {
                    k64f get()           { return (k64f) ::GoSurfaceRegion2d_Length(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceRegion2d_SetLength(Handle, value)); }
                }
                /// <summary>The ZAngle.</summary>
                property k64f ZAngle
                {
                    k64f get()           { return (k64f) ::GoSurfaceRegion2d_ZAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoSurfaceRegion2d_SetZAngle(Handle, value)); }
                }
            };
            
            /// <summary>Represents a three dimensional surface region.</summary>
            public ref class GoRegion3d : public KObject
            {
                KDeclareClass(GoRegion3d, GoRegion3d)
            
                /// <summary>Initializes a new instance of the GoRegion3d class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRegion3d(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoRegion3d class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoRegion3d(GoSensor^ sensor)
                {
                    ::GoRegion3d handle = kNULL;
            
                    KCheck(::GoRegion3d_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoRegion3d(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoRegion3d(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoRegion3d handle = kNULL;
            
                    KCheck(::GoRegion3d_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The X position.</summary>
                property k64f X
                {
                    k64f get()           { return (k64f) ::GoRegion3d_X(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetX(Handle, value)); }
                }
            
                /// <summary>The Y position.</summary>
                property k64f Y
                {
                    k64f get()           { return (k64f) ::GoRegion3d_Y(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetY(Handle, value)); }
                }
            
                /// <summary>The Z position.</summary>
                property k64f Z
                {
                    k64f get()           { return (k64f) ::GoRegion3d_Z(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetZ(Handle, value)); }
                }
            
                /// <summary>The width.</summary>
                property k64f Width
                {
                    k64f get()           { return (k64f) ::GoRegion3d_Width(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetWidth(Handle, value)); }
                }
            
                /// <summary>The length.</summary>
                property k64f Length
                {
                    k64f get()           { return (k64f) ::GoRegion3d_Length(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetLength(Handle, value)); }
                }
            
                /// <summary>The height.</summary>
                property k64f Height
                {
                    k64f get()           { return (k64f) ::GoRegion3d_Height(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetHeight(Handle, value)); }
                }
                /// <summary>The ZAngle.</summary>
                property k64f ZAngle
                {
                    k64f get()           { return (k64f) ::GoRegion3d_ZAngle(Handle); }
                    void set(k64f value)  { KCheck(::GoRegion3d_SetZAngle(Handle, value)); }
                }
            };

            /// <summary>Represents a point set region.</summary>
            public ref class GoPointSetRegion : public KObject
            {
                KDeclareClass(GoPointSetRegion, GoPointSetRegion)
                /// <summary>Initializes a new instance of the GoPointSetRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoPointSetRegion(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoPointSetRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoPointSetRegion(GoSensor^ sensor)
                {
                    ::GoPointSetRegion handle = kNULL;
            
                    KCheck(::GoPointSetRegion_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoPointSetRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoPointSetRegion(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoPointSetRegion handle = kNULL;
            
                    KCheck(::GoPointSetRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }

                property k64f ConstantZ
                {
                    void set(k64f value) { KCheck(::GoPointSetRegion_SetConstantZ(Handle, value)); }
                }

                void DisableConstantZ()
                {
                    KCheck(::GoPointSetRegion_DisableConstantZ(Handle));
                }

                KPoint3d64f PointAt(KSize index)
                {
                    return (KPoint3d64f)(*::GoPointSetRegion_PointAt(Handle, index));
                }

                void SetPointAt(KSize index, KPoint3d64f point)
                {
                    KCheck(::GoPointSetRegion_SetPointAt(Handle, index, (kPoint3d64f*) &point));
                }

                property kSize PointCount
                {
                    kSize get() { return ::GoPointSetRegion_PointCount(Handle); }
                }
            };

            /// <summary>Represents a cylinder region for select surface tools.</summary>
            public ref class GoCylinderRegion : public KObject
            {
                KDeclareClass(GoCylinderRegion, GoCylinderRegion)
            
                /// <summary>Initializes a new instance of the GoCylinderRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoCylinderRegion(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoCylinderRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoCylinderRegion(GoSensor^ sensor)
                {
                    ::GoCylinderRegion handle = kNULL;
            
                    KCheck(::GoCylinderRegion_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoCylinderRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoCylinderRegion(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoCylinderRegion handle = kNULL;
            
                    KCheck(::GoCylinderRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The X position.</summary>
                property k64f X
                {
                    k64f get()           { return (k64f) ::GoCylinderRegion_X(Handle); }
                    void set(k64f value)  { KCheck(::GoCylinderRegion_SetX(Handle, value)); }
                }
            
                /// <summary>The Y position.</summary>
                property k64f Y
                {
                    k64f get()           { return (k64f) ::GoCylinderRegion_Y(Handle); }
                    void set(k64f value)  { KCheck(::GoCylinderRegion_SetY(Handle, value)); }
                }
            
                /// <summary>The Z position.</summary>
                property k64f Z
                {
                    k64f get()           { return (k64f) ::GoCylinderRegion_Z(Handle); }
                    void set(k64f value)  { KCheck(::GoCylinderRegion_SetZ(Handle, value)); }
                }
            
                /// <summary>The radius.</summary>
                property k64f Radius
                {
                    k64f get()           { return (k64f) ::GoCylinderRegion_Radius(Handle); }
                    void set(k64f value)  { KCheck(::GoCylinderRegion_SetRadius(Handle, value)); }
                }
            
                /// <summary>The height.</summary>
                property k64f Height
                {
                    k64f get()           { return (k64f) ::GoCylinderRegion_Height(Handle); }
                    void set(k64f value)  { KCheck(::GoCylinderRegion_SetHeight(Handle, value)); }
                }
            };
            
            /// <summary>Represents a surface feature for select surface tools.</summary>
            public ref class GoSurfaceFeature : public KObject
            {
                KDeclareClass(GoSurfaceFeature, GoSurfaceFeature)
            
                /// <summary>Initializes a new instance of the GoSurfaceFeature class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceFeature(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoSurfaceFeature class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSurfaceFeature(GoSensor^ sensor)
                {
                    ::GoSurfaceFeature handle = kNULL;
            
                    KCheck(::GoSurfaceFeature_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoSurfaceFeature(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceFeature(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSurfaceFeature handle = kNULL;
            
                    KCheck(::GoSurfaceFeature_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The surface feature type.</summary>
                property GoSurfaceFeatureType FeatureType
                {
                    GoSurfaceFeatureType get()           { return (GoSurfaceFeatureType) ::GoSurfaceFeature_Type(Handle); }
                    void set(GoSurfaceFeatureType type)  { KCheck(::GoSurfaceFeature_SetType(Handle, type)); }
                }
            
                /// <summary>Enables or disables the surface feature region.</summary>
                property bool RegionEnabled
                {
                    bool get()           { return KToBool(::GoSurfaceFeature_RegionEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoSurfaceFeature_EnableRegion(Handle, enable)); }
                }
            
                /// <summary>The 3d region for the feature.</summary>
                property GoRegion3d^ Region
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoSurfaceFeature_Region(Handle)); }
                }
            };
        }
    }
}

#endif
