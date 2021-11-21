//
// GoExtParam.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_EXT_PARAM_H
#define GO_SDK_NET_EXT_PARAM_H

#include <GoSdk/Tools/GoExtParam.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/Tools/GoProfileToolUtils.h>
#include <GoSdkNet/Tools/GoSurfaceToolUtils.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {


            public ref class GoExtParam abstract : public KObject
            {
                KDeclareClass(GoExtParam, GoExtParam)

            public:
                /// <summary>Default GoExtParam constructor.</summary>
                GoExtParam() {}

                /// <summary>Initializes a new instance of the GoExtParam class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParam(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Gets the extensible parameter label name.</summary>
                property String^ Label
                {
                    String^ get()
                    {
                        return KToString(::GoExtParam_Label(Handle));
                    }
                }

                /// <summary>Gets the extensible parameter name.</summary>
                property String^ Id
                {
                    String^ get()
                    {
                        return KToString(::GoExtParam_Id(Handle));
                    }
                }

                /// <summary>Gets the extensible parameter type.</summary>
                property GoExtParamType Type
                {
                    GoExtParamType get()
                    {
                        return (GoExtParamType) ::GoExtParam_Type(Handle);
                    }
                }

                /// <summary>Gets the usage status of the exensible parametre.</summary>
                property bool Used
                {
                    bool get()
                    {
                        return KToBool(::GoExtParam_Used(Handle));
                    }
                }

                /// <summary>Gets the extensible parameter measurement type.</summary>
                property String^ UnitType
                {
                    String^ get()
                    {
                        return KToString(::GoExtParam_UnitType(Handle));
                    }
                }
            };


            public ref class GoExtParamFeature : public GoExtParam
            {
                KDeclareClass(GoExtParamFeature, GoExtParamFeature)

            public:
                /// <summary>Initializes a new instance of the GoExtParamFeature class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamFeature(IntPtr handle)
                : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamFeature class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamFeature(GoSensor^ sensor)
                {
                    ::GoExtParamFeature handle = kNULL;

                    KCheck(::GoExtParamFeature_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamFeature(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamFeature(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamFeature handle = kNULL;

                    KCheck(::GoExtParamFeature_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the identifier of the extensible feature parameter.</summary>
                property k32s FeatureId
                {
                    k32s get()
                    {
                        return (k32s) ::GoExtParamFeature_FeatureId(Handle);
                    }
                    void set(k32s id)
                    {
                        KCheck(::GoExtParamFeature_SetFeatureId(Handle, id));
                    }
                }

                /// <summary>Gets the minimum value of the extensible feature parameter.</summary>
                property k32s ValueMin
                {
                    k32s get()
                    {
                        return (k32s) ::GoExtParamFeature_ValueMin(Handle);
                    }
                }

                /// <summary>Gets the maximum value of the extensible feature parameter.</summary>
                property k32s ValueMax
                {
                    k32s get()
                    {
                        return (k32s) ::GoExtParamFeature_ValueMax(Handle);
                    }
                }

                /// <summary>Gets the number of options for the extensible feature parameter.</summary>
                property kSize OptionCount
                {
                    kSize get()
                    {
                        return (kSize) ::GoExtParamFeature_OptionCount(Handle);
                    }
                }

                /// <summary>Gets the specified option for the extensible feature parameter.</summary>
                k32s GetOption(k64s index)
                {
                    return (k32s) ::GoExtParamFeature_OptionValueAt(Handle, (kSize) index);
                }

                /// <summary>Gets the specified option's description text for the extensible feature parameter.</summary>
                String^ GetOptionDescription(k64s index)
                {
                    return KToString(::GoExtParamFeature_OptionDescriptionAt(Handle, (kSize) index));
                }
            };

            public ref class GoExtParamBool : public GoExtParam
            {
                KDeclareClass(GoExtParamBool, GoExtParamBool)

                /// <summary>Initializes a new instance of the GoExtParamBool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamBool(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamBool class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamBool(GoSensor^ sensor)
                {
                    ::GoExtParamBool handle = kNULL;

                    KCheck(::GoExtParamBool_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamBool(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamBool(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamBool handle = kNULL;

                    KCheck(::GoExtParamBool_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property bool Value
                {
                    bool get()           { return KToBool(::GoExtParamBool_Value(Handle)); }
                    void set(bool value)  { KCheck(::GoExtParamBool_SetValue(Handle, value)); }
                }
            };

            public ref class GoExtParamInt : public GoExtParam
            {
                KDeclareClass(GoExtParamInt, GoExtParamInt)

                /// <summary>Initializes a new instance of the GoExtParamInt class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamInt(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamInt class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamInt(GoSensor^ sensor)
                {
                    ::GoExtParamInt handle = kNULL;

                    KCheck(::GoExtParamInt_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamInt(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamInt(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamInt handle = kNULL;

                    KCheck(::GoExtParamInt_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property k32s Value
                {
                    k32s get()           { return (k32s) ::GoExtParamInt_Value(Handle); }
                    void set(k32s value)  { KCheck(::GoExtParamInt_SetValue(Handle, value)); }
                }

                property bool IsValueLimitUsed
                {
                    bool get()           { return KToBool(::GoExtParamInt_IsValueLimitUsed(Handle)); }
                }

                property k32s ValueMin
                {
                    k32s get()           { return (k32s) ::GoExtParamInt_ValueMin(Handle); }
                }

                property k32s ValueMax
                {
                    k32s get()           { return (k32s) ::GoExtParamInt_ValueMax(Handle); }
                }

                property kSize OptionCount
                {
                    kSize get()           { return (kSize) ::GoExtParamInt_OptionCount(Handle); }
                }

                k32s GetOption(k64s index)
                {
                    return (k32s) ::GoExtParamInt_OptionValueAt(Handle, (kSize) index);
                }

                String^ GetOptionDescription(k64s index)
                {
                   return KToString(::GoExtParamInt_OptionDescriptionAt(Handle, (kSize) index));
                }
            };

            public ref class GoExtParamFloat : public GoExtParam
            {
                KDeclareClass(GoExtParamFloat, GoExtParamFloat)

                /// <summary>Initializes a new instance of the GoExtParamFloat class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamFloat(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamFloat class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamFloat(GoSensor^ sensor)
                {
                    ::GoExtParamFloat handle = kNULL;

                    KCheck(::GoExtParamFloat_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamFloat(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamFloat(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamFloat handle = kNULL;

                    KCheck(::GoExtParamFloat_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property k64f Value
                {
                    k64f get()           { return (k64f) ::GoExtParamFloat_Value(Handle); }
                    void set(k64f value)  { KCheck(::GoExtParamFloat_SetValue(Handle, value)); }
                }

                property bool IsValueLimitUsed
                {
                    bool get()           { return KToBool(::GoExtParamFloat_IsValueLimitUsed(Handle)); }
                }

                property k64f ValueMin
                {
                    k64f get()           { return (k64f) ::GoExtParamFloat_ValueMin(Handle); }
                }

                property k64f ValueMax
                {
                    k64f get()           { return (k64f) ::GoExtParamFloat_ValueMax(Handle); }
                }

                property kSize OptionCount
                {
                    kSize get()           { return (kSize) ::GoExtParamFloat_OptionCount(Handle); }
                }

                k64f GetOption(k64s index)
                {
                    return (k64f) ::GoExtParamFloat_OptionValueAt(Handle, (kSize) index);
                }

                String^ GetOptionDescription(k64s index)
                {
                   return KToString(::GoExtParamFloat_OptionDescriptionAt(Handle, (kSize) index));
                }
            };

            public ref class GoExtParamString : public GoExtParam
            {
                KDeclareClass(GoExtParamString, GoExtParamString)

                /// <summary>Initializes a new instance of the GoExtParamString class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamString(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamString class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamString(GoSensor^ sensor)
                {
                    ::GoExtParamString handle = kNULL;

                    KCheck(::GoExtParamString_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamString(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamString(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamString handle = kNULL;

                    KCheck(::GoExtParamString_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property String^ Value
                {
                    String^ get()           { return KToString(kString_Chars(::GoExtParamString_Value(Handle))); }
                }
            };

            public ref class GoExtParamProfileRegion : public GoExtParam
            {
                KDeclareClass(GoExtParamProfileRegion, GoExtParamProfileRegion)

                /// <summary>Initializes a new instance of the GoExtParamProfileRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamProfileRegion(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamProfileRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamProfileRegion(GoSensor^ sensor)
                {
                    ::GoExtParamProfileRegion handle = kNULL;

                    KCheck(::GoExtParamProfileRegion_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamProfileRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamProfileRegion(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamProfileRegion handle = kNULL;

                    KCheck(::GoExtParamProfileRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property GoProfileRegion^ Value
                {
                    GoProfileRegion^ get()           { return KToObject<GoProfileRegion^>(::GoExtParamProfileRegion_Value(Handle)); }
                }
            };

            public ref class GoExtParamSurfaceRegion2d : public GoExtParam
            {
                KDeclareClass(GoExtParamSurfaceRegion2d, GoExtParamSurfaceRegion2d)

                /// <summary>Initializes a new instance of the GoExtParamSurfaceRegion2d class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamSurfaceRegion2d(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamSurfaceRegion2d class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamSurfaceRegion2d(GoSensor^ sensor)
                {
                    ::GoExtParamSurfaceRegion2d handle = kNULL;

                    KCheck(::GoExtParamSurfaceRegion2d_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamSurfaceRegion2d(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamSurfaceRegion2d(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamSurfaceRegion2d handle = kNULL;

                    KCheck(::GoExtParamSurfaceRegion2d_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property GoSurfaceRegion2d^ Value
                {
                    GoSurfaceRegion2d^ get()           { return KToObject<GoSurfaceRegion2d^>(::GoExtParamSurfaceRegion2d_Value(Handle)); }
                }
            };

            public ref class GoExtParamSurfaceRegion3d : public GoExtParam
            {
                KDeclareClass(GoExtParamSurfaceRegion3d, GoExtParamSurfaceRegion3d)

                /// <summary>Initializes a new instance of the GoExtParamSurfaceRegion3d class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamSurfaceRegion3d(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamSurfaceRegion3d class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamSurfaceRegion3d(GoSensor^ sensor)
                {
                    ::GoExtParamSurfaceRegion3d handle = kNULL;

                    KCheck(::GoExtParamSurfaceRegion3d_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamSurfaceRegion3d(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamSurfaceRegion3d(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamSurfaceRegion3d handle = kNULL;

                    KCheck(::GoExtParamSurfaceRegion3d_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property GoRegion3d^ Value
                {
                    GoRegion3d^ get()           { return KToObject<GoRegion3d^>(::GoExtParamSurfaceRegion3d_Value(Handle)); }
                }
            };

            public ref class GoExtParamDataInput : public GoExtParam
            {
                KDeclareClass(GoExtParamDataInput, GoExtParamDataInput)

                /// <summary>Initializes a new instance of the GoExtParamDataInput class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamDataInput(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamDataInput class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamDataInput(GoSensor^ sensor)
                {
                    ::GoExtParamDataInput handle = kNULL;

                    KCheck(::GoExtParamDataInput_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamDataInput(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamDataInput(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamDataInput handle = kNULL;

                    KCheck(::GoExtParamDataInput_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property GoDataStreamId Stream
                {
                    GoDataStreamId get()           { return (GoDataStreamId) ::GoExtParamDataInput_Value(Handle); }
                    void set(GoDataStreamId stream)  { KCheck(::GoExtParamDataInput_SetValue(Handle, ::GoDataStreamId(stream))); }
                }

                property kSize OptionCount
                {
                    kSize get()           { return (kSize) ::GoExtParamDataInput_OptionCount(Handle); }
                }

                GoDataStreamId GetOption(k64s index)
                {
                    return (GoDataStreamId) ::GoExtParamDataInput_OptionValueAt(Handle, (kSize) index);
                }
            };


            public ref class GoExtParamPointSetRegion : public GoExtParam
            {
                KDeclareClass(GoExtParamPointSetRegion, GoExtParamPointSetRegion)

                /// <summary>Initializes a new instance of the GoExtParamPointSetRegion class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParamPointSetRegion(IntPtr handle)
                    : GoExtParam(handle)
                {}

                /// <summary>Initializes a new instance of the GoExtParamPointSetRegion class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoExtParamPointSetRegion(GoSensor^ sensor)
                {
                    ::GoExtParamPointSetRegion handle = kNULL;

                    KCheck(::GoExtParamPointSetRegion_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExtParamPointSetRegion(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoExtParamPointSetRegion(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoExtParamPointSetRegion handle = kNULL;

                    KCheck(::GoExtParamPointSetRegion_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                property GoPointSetRegion^ Value
                {
                    GoPointSetRegion^ get()           { return KToObject<GoPointSetRegion^>(::GoExtParamPointSetRegion_Value(Handle)); }
                }
            };

        }
    }
}

#endif
