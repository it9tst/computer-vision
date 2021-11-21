// 
// GoRangeTools.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_RANGE_TOOLS_H
#define GO_SDK_NET_RANGE_TOOLS_H

#include <GoSdk/Tools/GoRangeTools.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a base range tool.</summary>
            public ref class GoRangeTool abstract : public GoTool
            {
                KDeclareClass(GoRangeTool, GoRangeTool)

            public:
                /// <summary>Default GoRangeTool constructor.</summary>
                GoRangeTool() {}
            
                /// <summary>Initializes a new instance of the GoRangeTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangeTool(IntPtr handle)
                    : GoTool(handle)
                {}
            
                /// <summary>The data stream.</summary>
                /// <remarks>Note that stream validation will only occur if tool is in the tool options list.</remarks>
                property GoDataStream Stream
                {
                    GoDataStream get()           { return (GoDataStream) ::GoRangeTool_Stream(Handle); }
                    void set(GoDataStream stream)  { KCheck(::GoRangeTool_SetStream(Handle, ::GoDataStream(stream))); }
                }

                /// <summary>The data stream option list count.</summary>
                /// <returns>The current range tool data stream option list count.</returns>
                property k64s StreamOptionCount
                {
                    k64s get()           { return (k64s) ::GoRangeTool_StreamOptionCount(Handle); }
                }

                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The range tool data stream option at the given index, or k32U_MAX if an invalid index is given.</returns>
                GoDataStream GetStreamOptionAt(k64s index)
                {
                   return (GoDataStream) ::GoRangeTool_StreamOptionAt(Handle, (kSize)index);
                }

                /// <summary>The data source.</summary>
                /// <remarks>Note that source validation will only occur if tool is in the tool options list.</remarks>
                property GoDataSource Source
                {
                    GoDataSource get()           { return (GoDataSource) ::GoRangeTool_Source(Handle); }
                    void set(GoDataSource source)  { KCheck(::GoRangeTool_SetSource(Handle, source)); }
                }
            
                /// <summary>The data source option list count.</summary>
                property k64s SourceOptionCount
                {
                    k64s get()           { return (k64s) ::GoRangeTool_SourceOptionCount(Handle); }
                }
                
                /// <summary>Gets the data source option at the given index.</summary>
                /// <param name="index">The index of the option list to access.</param>
                /// <returns>The tool data source option at the given index, or k32U_MAX if an invalid index is given.</returns>
                GoDataSource GetSourceOption(k64s index)
                {
                   return (GoDataSource) ::GoRangeTool_SourceOptionAt(Handle, (kSize)index);
                }
            };
            
            /// <summary>Represents a range position tool.</summary>
            public ref class GoRangePosition : public GoRangeTool
            {
                KDeclareClass(GoRangePosition, GoRangePosition)
            
                /// <summary>Initializes a new instance of the GoRangePosition class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangePosition(IntPtr handle)
                    : GoRangeTool(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoRangePosition class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoRangePosition(GoSensor^ sensor)
                {
                    ::GoRangePosition handle = kNULL;
            
                    KCheck(::GoRangePosition_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoRangePosition(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoRangePosition(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoRangePosition handle = kNULL;
            
                    KCheck(::GoRangePosition_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>A range position tool Z measurement object.</summary>
                property GoRangePositionZ^ ZMeasurement
                {
                    GoRangePositionZ^ get()           { return KToObject<GoRangePositionZ^>(::GoRangePosition_ZMeasurement(Handle)); }
                }
            };
            
            /// <summary>Represents a range thickness tool.</summary>
            public ref class GoRangeThickness : public GoRangeTool
            {
                KDeclareClass(GoRangeThickness, GoRangeThickness)
            
                /// <summary>Initializes a new instance of the GoRangeThickness class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangeThickness(IntPtr handle)
                    : GoRangeTool(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoRangeThickness class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoRangeThickness(GoSensor^ sensor)
                {
                    ::GoRangeThickness handle = kNULL;
            
                    KCheck(::GoRangeThickness_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoRangeThickness(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoRangeThickness(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoRangeThickness handle = kNULL;
            
                    KCheck(::GoRangeThickness_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>Enable or disable absolute value measurement output.</summary>
                property bool AbsoluteEnabled
                {
                    bool get()           { return KToBool(::GoRangeThickness_AbsoluteEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoRangeThickness_EnableAbsolute(Handle, enable)); }
                }
            
                /// <summary>A range thickness tool thickness measurement object.</summary>
                property GoRangeThickness^ ThicknessMeasurement
                {
                    GoRangeThickness^ get()           { return KToObject<GoRangeThickness^>(::GoRangeThickness_ThicknessMeasurement(Handle)); }
                }
            };

            private ref class GoRangeTools abstract sealed
            {
            internal:
                static GoRangeTool^ GetRangeToolInstance(::GoRangeTool tool)
                {
                    if (kObject_Is(tool, kTypeOf(GoRangePosition))) { return KToObject<GoRangePosition^>(tool); }
                    if (kObject_Is(tool, kTypeOf(GoRangeThickness))) { return KToObject<GoRangeThickness^>(tool); }

                    return nullptr;
                }
            };
        }
    }
}

#endif
