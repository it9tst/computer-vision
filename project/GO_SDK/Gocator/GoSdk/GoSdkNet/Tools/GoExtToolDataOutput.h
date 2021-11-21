//
// GoExtToolDataOutput.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_TOOL_DATA_OUTPUT_H
#define GO_SDK_NET_TOOL_DATA_OUTPUT_H

#include <GoSdk/Tools/GoExtToolDataOutput.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents a tool data output.</summary>
            public ref class GoExtToolDataOutput : public KObject
            {
                KDeclareClass(GoExtToolDataOutput, GoExtToolDataOutput)

            public:
                /// <summary>Initializes a new instance of the GoExtTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtToolDataOutput(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>The tool data output ID.</summary>
                property k32s Id
                {
                    k32s get()          { return (k32s) ::GoExtToolDataOutput_Id(Handle); }
                    void set(k32s id)   { KCheck(::GoExtToolDataOutput_SetId(Handle, id)); }
                }

                /// <summary>The measurement name.</summary>
                property String^ Name
                {
                    String^ get() { return KToString(::GoExtToolDataOutput_Name(Handle)); }

                    void set(String^ name)
                    {
                        KString str(name);

                        KCheck(::GoExtToolDataOutput_SetName(Handle, str.CharPtr));
                    }
                }

                /// <summary>Enables or disables output.</summary>
                property bool Enabled
                {
                    bool get()           { return KToBool(::GoExtToolDataOutput_Enabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoExtToolDataOutput_SetEnable(Handle, enable)); }
                }

                /// <summary>The measurement value offset.</summary>
                property GoDataType DataType
                {
                    GoDataType get()    { return (GoDataType) ::GoExtToolDataOutput_DataType(Handle); }
                }
                property String^ Type
                {
                    String^ get()
                    {
                        return KToString(::GoExtToolDataOutput_Type(Handle));
                    }
                }
            };
        }
    }
}

#endif




































