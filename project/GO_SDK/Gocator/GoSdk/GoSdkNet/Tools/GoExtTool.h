//
// GoExtTool.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_EXT_TOOL_H
#define GO_SDK_NET_EXT_TOOL_H

#include <GoSdk/Tools/GoExtTool.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/Tools/GoTool.h>
#include <GoSdkNet/Tools/GoExtParam.h>
#include <GoSdkNet/Tools/GoExtToolDataOutput.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents an extensible tool.</summary>
            public ref class GoExtTool : public GoTool
            {
                KDeclareClass(GoExtTool, GoExtTool)

            public:
                /// <summary>Default GoSurfaceTool constructor.</summary>
                GoExtTool()
                {}
                /// <summary>Initializes a new instance of the GoExtTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtTool(IntPtr handle)
                         : GoTool(handle)
                {}

                /// <summary>The tool configuration version string.</summary>
                property String^ Version
                {
                    String^ get()           { return KToString(::GoExtTool_Version(Handle)); }
                }

                /// <summary>The measurement count.</summary>
                property k64s MeasurementCount
                {
                    virtual k64s get() override     { return (k64s) ::GoExtTool_MeasurementCount(Handle); }
                }

                /// <summary>The measurement at the given index.</summary>
                /// <param name="index">The index of the measurement.</param>
                /// <returns>The measurement at the given index or null if an invalid index is provided.</returns>
                virtual GoMeasurement^ GetMeasurement(k64s index) override
                {
                    ::GoMeasurement measurement = ::GoExtTool_MeasurementAt(Handle, (kSize)index);

                    if (kIsNull(measurement)) { return nullptr; }

                    return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>The display name of the tool.</summary>
                property String^ DisplayName
                {
                    String^ get() { return KToString(::GoExtTool_DisplayName(Handle)); }

                    void set(String^ name)
                    {
                        KString str(name);

                        KCheck(::GoExtTool_SetDisplayName(Handle, str.CharPtr));
                    }
                }

                /// <summary>The type of the tool.</summary>
                property String^ ToolTypeName
                {
                    String^ get() { return KToString(::GoExtTool_Type(Handle)); }
                }

                /// <summary>The data source.</summary>
                property GoDataSource Source
                {
                    virtual GoDataSource get()           { return (GoDataSource) ::GoExtTool_Source(Handle); }
                    virtual void set(GoDataSource source)  { KCheck(::GoExtTool_SetSource(Handle, source)); }
                }

                /// <summary>The data source option list count.</summary>
                property kSize SourceOptionCount
                {
                    virtual kSize get()           { return (kSize) ::GoExtTool_SourceOptionCount(Handle); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The tool data source option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual k32u GetSourceOption(k64s index)
                {
                   return (k32u) ::GoExtTool_SourceOptionAt(Handle, (kSize)index);
                }

                /// <summary>A boolean value representing whether the tool supports x anchoring.</summary>
                property bool XAnchorSupportEnabled
                {
                    bool get()           { return KToBool(::GoExtTool_XAnchorSupportEnabled(Handle)); }
                }

                /// <summary>The X-anchoring option list count.</summary>
                property kSize XAnchorOptionCount
                {
                    virtual kSize get()           { return (kSize) ::GoExtTool_XAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the X-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The X-anchoring option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual k32u GetXAnchorOptionAt(k64s index)
                {
                   return (k32u) ::GoExtTool_XAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The X-anchoring source.</summary>
                /// <remarks>-1 if no source is currently set.</remarks>
                property k32s XAnchor
                {
                    virtual k32s get()           { return (k32s) ::GoExtTool_XAnchor(Handle); }
                    virtual void set(k32s value)  { KCheck(::GoExtTool_SetXAnchor(Handle, value)); }
                }

                /// <summary>A boolean value representing whether or not a valid X-anchoring source has been set for X-anchoring.</summary>
                property bool XAnchorEnabled
                {
                    virtual bool get()           { return KToBool(::GoExtTool_XAnchorEnabled(Handle)); }
                }

                /// <summary>A boolean value representing whether the tool supports y anchoring.</summary>
                property bool YAnchorSupportEnabled
                {
                    bool get()           { return KToBool(::GoExtTool_YAnchorSupportEnabled(Handle)); }
                }

                /// <summary>The Y-anchoring option list count.</summary>
                property kSize YAnchorOptionCount
                {
                    virtual kSize get()           { return (kSize) ::GoExtTool_YAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the Y-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The Y-anchoring option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual k32u GetYAnchorOptionAt(k64s index)
                {
                   return (k32u) ::GoExtTool_YAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The Y-anchoring source.</summary>
                /// <remarks>-1 if no source is currently set.</remarks>
                property k32s YAnchor
                {
                    virtual k32s get()           { return (k32s) ::GoExtTool_YAnchor(Handle); }
                    virtual void set(k32s value)  { KCheck(::GoExtTool_SetYAnchor(Handle, value)); }
                }

                /// <summary>A boolean value representing whether or not a valid Y-anchoring source has been set for Y-anchoring.</summary>
                property bool YAnchorEnabled
                {
                    virtual bool get()           { return KToBool(::GoExtTool_YAnchorEnabled(Handle)); }
                }

                /// <summary>A boolean value representing whether the tool supports z anchoring.</summary>
                property bool ZAnchorSupportEnabled
                {
                    bool get()           { return KToBool(::GoExtTool_ZAnchorSupportEnabled(Handle)); }
                }

                /// <summary>The Z-anchoring option list count.</summary>
                property kSize ZAnchorOptionCount
                {
                    virtual kSize get()           { return (kSize) ::GoExtTool_ZAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the Z-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The Z-anchoring option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual k32u GetZAnchorOptionAt(k64s index)
                {
                   return (k32u) ::GoExtTool_ZAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The Z-anchoring source.</summary>
                /// <remarks>-1 if no source is currently set.</remarks>
                property k32s ZAnchor
                {
                    virtual k32s get()           { return (k32s) ::GoExtTool_ZAnchor(Handle); }
                    virtual void set(k32s value)  { KCheck(::GoExtTool_SetZAnchor(Handle, value)); }
                }

                /// <summary>A boolean value representing whether or not a valid Z-anchoring source has been set for Z-anchoring.</summary>
                property bool ZAnchorEnabled
                {
                    virtual bool get() { return KToBool(::GoExtTool_ZAnchorEnabled(Handle)); }
                }

                /// <summary>A boolean value representing whether the tool supports z angle anchoring.</summary>
                property bool ZAngleAnchorSupportEnabled
                {
                    bool get()           { return KToBool(::GoExtTool_ZAngleAnchorSupportEnabled(Handle)); }
                }

                /// <summary>The ZAngle-anchoring option list count.</summary>
                property kSize ZAngleAnchorOptionCount
                {
                    virtual kSize get()           { return (kSize) ::GoExtTool_ZAngleAnchorOptionCount(Handle); }
                }

                /// <summary>Gets the ZAngle-anchoring option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The Z-anchoring option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual k32u GetZAngleAnchorOptionAt(k64s index)
                {
                    return (k32u) ::GoExtTool_ZAngleAnchorOptionAt(Handle, (kSize)index);
                }

                /// <summary>The ZAngle-anchoring source.</summary>
                /// <remarks>-1 if no source is currently set.</remarks>
                property k32s ZAngleAnchor
                {
                    virtual k32s get()           { return (k32s) ::GoExtTool_ZAngleAnchor(Handle); }
                    virtual void set(k32s value)  { KCheck(::GoExtTool_SetZAngleAnchor(Handle, value)); }
                }

                /// <summary>A boolean value representing whether or not a valid ZAngle-anchoring source has been set for Z-anchoring.</summary>
                property bool ZAngleAnchorEnabled
                {
                    virtual bool get()           { return KToBool(::GoExtTool_ZAngleAnchorEnabled(Handle)); }
                }


                /// <summary>Retrieves the first found instance of a measurement for a given enumeration type.</summary>
                /// <param name="name">The name of the measurement to retrieve.</param>
                /// <returns>A measurement object if one is found, otherwise null.</returns>
                GoMeasurement^ FindMeasurementByName(String^ name)
                {
                   KString str(name);

                   ::GoMeasurement measurement = ::GoExtTool_FindMeasurementByName(Handle, str.CharPtr);

                   if (kIsNull(measurement)) { return nullptr; }

                   return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>The number of parameters available for the given tool.</summary>
                property k64s ParameterCount
                {
                    k64s get()           { return (k64s) ::GoExtTool_ParameterCount(Handle); }
                }

                /// <summary>Gets the parameter at the given index.</summary>
                /// <param name="index">The index of the parameter to retrieve.</param>
                /// <returns>The custom tool parameter object or null if the index is invalid.</returns>
                GoExtParam^ GetParameter(k64s index)
                {
                   ::GoExtParam param = ::GoExtTool_ParameterAt(Handle, (kSize)index);

                   if(kIsNull(param)) { return nullptr; }

                   return GetExtParamInstance(param);
                }

                /// <summary>Gets the parameter which matches the given label.</summary>
                /// <param name="label">The label of the parameter to retrieve.</param>
                /// <returns>The parameter matching the label or null if no match is found.</returns>
                GoExtParam^ FindParameterById(String^ label)
                {
                   KString str(label);

                   ::GoExtParam param = ::GoExtTool_FindParameterById(Handle, str.CharPtr);

                   if(kIsNull(param)) { return nullptr; }

                   return GetExtParamInstance(param);
                }

                /// <summary>The tool data output list count.</summary>
                property k64s ToolDataOutputCount
                {
                    k64s get()
                    {
                        return (k64s) ::GoExtTool_ToolDataOutputCount(Handle);
                    }
                }

                /// <summary>Gets the tool data output at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The tool data output at the given index, or null if an invalid index is given.</returns>
                GoExtToolDataOutput^ GetToolDataOutput(k64s index)
                {
                    ::GoExtToolDataOutput output = ::GoExtTool_ToolDataOutputAt(Handle, (kSize)index);

                    return KToObject<GoExtToolDataOutput^>(output);
                }

                /// <summary>The data stream.</summary>
                /// <remarks>Note that stream validation will only occur if tool is in the tool options list.</remarks>
                property GoDataStream Stream
                {
                    virtual GoDataStream get()              { return (GoDataStream) ::GoExtTool_Stream(Handle); }
                    virtual void set(GoDataStream stream)  { KCheck(::GoExtTool_SetStream(Handle, (::GoDataStream)stream)); }
                }

                /// <summary>Gets the data stream option list count.</summary>
                /// <returns>The current Ext tool data stream option list count.</returns>
                property k64s StreamOptionCount
                {
                    k64s get()      { return (k64s) ::GoExtTool_StreamOptionCount(Handle); }
                }

                /// <summary>Gets the data stream option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The Ext tool data stream option at the given index, or k32U_MAX if an invalid index is given.</returns>
                virtual GoDataStream GetStreamOptionAt(kSize index)
                {
                   return (GoDataStream) ::GoExtTool_StreamOptionAt(Handle, (kSize)index);
                }

            internal:
                GoExtParam^ GetExtParamInstance(::GoExtParam param)
                {
                    if (kObject_Is(param, kTypeOf(GoExtParamBool))) { return KToObject<GoExtParamBool^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamInt))) { return KToObject<GoExtParamInt^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamFloat))) { return KToObject<GoExtParamFloat^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamFeature))) { return KToObject<GoExtParamFeature^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamString))) { return KToObject<GoExtParamString^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamProfileRegion))) { return KToObject<GoExtParamProfileRegion^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamSurfaceRegion2d))) { return KToObject<GoExtParamSurfaceRegion2d^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamSurfaceRegion3d))) { return KToObject<GoExtParamSurfaceRegion3d^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamDataInput))) { return KToObject<GoExtParamDataInput^>(param); }
                    if (kObject_Is(param, kTypeOf(GoExtParamPointSetRegion))) { return KToObject<GoExtParamPointSetRegion^>(param); }

                    return nullptr;
                }
            };
        }
    }
}

#endif
