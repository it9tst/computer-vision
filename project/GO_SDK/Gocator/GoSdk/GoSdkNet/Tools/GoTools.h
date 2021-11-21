//
// GoTools.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_TOOLS_H
#define GO_SDK_NET_TOOLS_H

#include <GoSdk/Tools/GoTools.h>
#include <GoSdkNet/Tools/GoTool.h>
#include <GoSdkNet/Tools/GoExtTool.h>
#include <GoSdkNet/Tools/GoProfileTools.h>
#include <GoSdkNet/Tools/GoRangeTools.h>
#include <GoSdkNet/Tools/GoSurfaceTools.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a tool option.</summary>
            public ref class GoToolOption : public KObject
            {
                KDeclareClass(GoToolOption, GoToolOption)

                /// <summary>Initializes a new instance of the GoToolOption class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoToolOption(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoToolOption class.</summary>
                GoToolOption()
                {
                    ::GoToolOption handle = kNULL;

                    KCheck(::GoToolOption_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoToolOption()" />
                /// <param name="allocator">Memory allocator</param>
                GoToolOption(KAlloc^ allocator)
                {
                    ::GoToolOption handle = kNULL;

                    KCheck(::GoToolOption_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The name associated with the given tool option.</summary>
                property String^ Name
                {
                    String^ get()           { return KToString(::GoToolOption_Name(Handle)); }
                }

                /// <summary>The number of measurement options available for the given tool option.</summary>
                property kSize MeasurementOptionCount
                {
                    kSize get()           { return (kSize) ::GoToolOption_MeasurementOptionCount(Handle); }
                }

                /// <summary>Gets the measurement option at the given index of the tool option.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The measurement option associated with the given index.</returns>
                GoMeasurementOption GetMeasurementOption(k64s index)
                {
                    return (GoMeasurementOption)(*::GoToolOption_MeasurementOptionAt(Handle, (kSize)index));
                }

                /// <summary>Returns the number of feature options available for the given tool option.</summary>
                property kSize FeatureOptionCount
                {
                    kSize get()           { return (kSize) ::GoToolOption_FeatureOptionCount(Handle); }
                }

                /// <summary>Retrieves the feature option at the given index of the tool option.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The feature option associated with the given index.</returns>
                GoFeatureOption^ GetFeatureOption(k64s index)
                {
                    const ::GoFeatureOption* option = ::GoToolOption_FeatureOptionAt(Handle, (kSize)index);
                    return (kIsNull(option)) ? nullptr : gcnew GoFeatureOption(option);
               }

                /// <summary>Returns the number of tool data output options available for the given tool option.</summary>
                property kSize ToolDataOutputOptionCount
                {
                    kSize get()           { return (kSize) ::GoToolOption_ToolDataOutputOptionCount(Handle); }
                }

                /// <summary>Retrieves the tool data output option at the given index of the tool option.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The tool data output option associated with the given index.</returns>
                GoToolDataOutputOption^ GetToolDataOutputOption(k64s index)
                {
                    const ::GoToolDataOutputOption* option = ::GoToolOption_ToolDataOutputOptionAt(Handle, (kSize)index);
                    return (kIsNull(option)) ? nullptr : gcnew GoToolDataOutputOption(option);
                }
            };

            /// <summary>Represents a script tool.</summary>
            public ref class GoScript : public GoTool
            {
                KDeclareClass(GoScript, GoScript)

                    /// <summary>Initializes a new instance of the GoScript class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoScript(IntPtr handle)
                    : GoTool(handle)
                {}

                /// <summary>Initializes a new instance of the GoTools class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoScript(GoSensor^ sensor)
                {
                    ::GoScript handle = kNULL;

                    KCheck(::GoScript_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoScript(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoScript(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoScript handle = kNULL;

                    KCheck(::GoScript_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The code for the script.</summary>
                property String^ Code
                {
                    String^ get()
                    {
                        String^ strCode;
                        kChar* code = kNULL;

                        KCheck(::GoScript_Code(Handle, &code));

                        strCode = KToString(code);

                        kMemFreeRef(&code);

                        return strCode;
                    }

                    void set(String^ code)
                    {
                        KString str(code);

                        KCheck(::GoScript_SetCode(Handle, str.CharPtr));
                    }
                }

                /// <summary>Adds a script output.</summary>
                /// <param name="id">An ID (must not already be used by other measurements) of the script output to add.</param>
                void AddOutput(k32u id)
                {
                    KCheck(::GoScript_AddOutput(Handle, id));
                }

                /// <summary>Removes a script output with the specific ID.</summary>
                /// <param name="id">An ID of the script output to remove.</param>
                void RemoveOutput(k32u id)
                {
                    KCheck(::GoScript_RemoveOutput(Handle, id));
                }

                /// <summary>The count of script tool outputs.</summary>
                property k64s OutputCount
                {
                    k64s get()           { return (k64s) ::GoScript_OutputCount(Handle); }
                }

                /// <summary>Gets a handle to a script output at the given index.</summary>
                /// <param name="index">The index with which to return a corresponding script output.</param>
                /// <returns>A GoScriptOutput object or null if the index is invalid.</returns>
                GoScriptOutput^ GetOutput(k64s index)
                {
                    return KToObject<GoScriptOutput^>(::GoScript_OutputAt(Handle, (kSize)index));
                }
            };

            /// <summary>Represents a collection of tools.</summary>
            public ref class GoTools : public KObject
            {
                KDeclareClass(GoTools, GoTools)

                /// <summary>Initializes a new instance of the GoTools class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoTools(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoTools class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoTools(GoSensor^ sensor)
                {
                    ::GoTools handle = kNULL;

                    KCheck(::GoTools_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoTools(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoTools(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoTools handle = kNULL;

                    KCheck(::GoTools_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The current number of tools.</summary>
                property k64s ToolCount
                {
                    k64s get()           { return (k64s) ::GoTools_ToolCount(Handle); }
                }

                /// <summary>Gets a tool at the given index.</summary>
                /// <param name="index">The index with which to retrieve a tool.</param>
                /// <returns>A tool at the given index or null if the index is invalid.</returns>
                GoTool^ GetTool(k64s index)
                {
                    ::GoTool tool = ::GoTools_ToolAt(Handle, (kSize)index);

                    if (kIsNull(tool)) { return nullptr; }

                    return GetToolInstance(tool);
                }

                /// <summary>Moves a tool from one index to another.</summary>
                /// <param name="index">The index with which tool to move.</param>
                /// <param name="newIndex"> The new index in which to move the tool to. </param>
                void MoveTool(k64s index, k64s newIndex)
                {
                    KCheck(::GoTools_MoveTool(Handle, (kSize)index, (kSize)newIndex));
                }

                /// <summary>Adds a tool. Requires SDK client to define all aspects of the tool (eg. name, measurements etc).
                /// DEPRECATED: this API is deprecated as of 4.7.11.72. Use the AddToolByName() API.
                /// instead.</summary>
                /// <param name="type">The tool type enumerator value representing the type of tool to add.</param>
                /// <returns>A newly added tool.</returns>
                GoTool^ AddTool(GoToolType type)
                {
                    ::GoTool tool;

                    KCheck(::GoTools_AddTool(Handle, type, &tool));

                    return GetToolInstance(tool);
                }

                /// <summary>Removes a tool at the given index.</summary>
                /// <param name="index">The index with which to remove a tool.</param>
                void RemoveTool(k64s index)
                {
                    KCheck(::GoTools_RemoveTool(Handle, (kSize)index));
                }

                /// <summary>Removes all tools in this GoTools instance.</summary>
                void ClearTools()
                {
                    KCheck(::GoTools_ClearTools(Handle));
                }

                /// <summary>Gets an enabled measurement if the specified ID is valid.</summary>
                /// <param name="id">The measurement ID to search for.</param>
                /// <returns>A GoMeasurement object if an enabled measurement with the ID is found, otherwise null.</returns>
                GoMeasurement^ FindMeasurementById(k32u id)
                {
                    ::GoMeasurement measurement = ::GoTools_FindMeasurementById(Handle, id);

                    if(kIsNull(measurement)) { return nullptr; }

                    return GoMeasurements::GetMeasurementInstance(measurement);
                }

                /// <summary>Automatically update a given measurement to use a valid ID within the set of tools contained in the GoTools object.</summary>
                /// <param name="measurement">GoMeasurement object to update.</param>
                void AssignMeasurementId(GoMeasurement^ measurement)
                {
                    KCheck(::GoTools_AssignMeasurementId(Handle, KToHandle(measurement)));
                }

                /// <summary>The number of tool options available for the current configuration.</summary>
                property k64s ToolOptionCount
                {
                    k64s get()           { return (k64s) ::GoTools_ToolOptionCount(Handle); }
                }

                /// <summary>Gets the tool option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The tool option associated with the given index.</returns>
                GoToolOption^ GetToolOption(k64s index)
                {
                    return KToObject<GoToolOption^>(::GoTools_ToolOptionAt(Handle, (kSize)index));
                }

                /// <summary>Adds an existing tool by name. After calling this API, the tool can be used immediately.
                /// This API is the commonly used API to select a tool for the sensor.</summary>
                /// <param name="optionName">The name of the tool to add.</param>
                /// <returns>The newly constructed tool.</returns>
                GoTool^ AddToolByName(String^ optionName)
                {
                    KString str(optionName);
                    ::GoTool tool;

                    KCheck(::GoTools_AddToolByName(Handle, str.CharPtr, &tool));

                    return GetToolInstance(tool);
                }

                /// <summary>Adds a measurement to a tool via a string representing the type.</summary>
                /// <param name="tool">The tool of which the measurement is being added.</param>
                /// <param name="type">The measurement type represented as a string to be added.</param>
                /// <param name="name">The name to assign the new measurement.</param>
                /// <returns>The newly added measurement.</returns>
                GoMeasurement^ AddMeasurementByName(GoTool^ tool, String^ type, String^ name)
                {
                    KString strType(type);
                    KString strName(name);
                    ::GoMeasurement measurement;

                    KCheck(::GoTools_AddMeasurementByName(Handle, KToHandle(tool), strType.CharPtr, strName.CharPtr, &measurement));

                    return GoMeasurements::GetMeasurementInstance(measurement);
                }

            internal:
                GoTool^ GetToolInstance(::GoTool tool)
                {
                    // Note that GoSurfaceTool is derived from GoExtTool so it
                    // will also match the GoExtTool check.
                    if (kObject_Is(tool, kTypeOf(GoSurfaceTool))) { return GoSurfaceTools::GetSurfaceToolInstance(tool); }
                    if (kObject_Is(tool, kTypeOf(GoProfileTool))) { return GoProfileTools::GetProfileToolInstance(tool); }
                    if (kObject_Is(tool, kTypeOf(GoRangeTool))) { return GoRangeTools::GetRangeToolInstance(tool); }
                    if (kObject_Is(tool, kTypeOf(GoScript))) { return KToObject<GoScript^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoExtTool))) { return KToObject<GoExtTool^>(tool); }

                    return nullptr;
                }
            };
        }
    }
}

#endif
