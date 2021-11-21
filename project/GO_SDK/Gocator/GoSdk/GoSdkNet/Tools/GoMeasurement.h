// 
// GoMeasurement.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_MEASUREMENT_H
#define GO_SDK_NET_MEASUREMENT_H

#include <GoSdk/Tools/GoMeasurement.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents the base class for a tool measurement or script output.</summary>
            public ref class GoMeasurement abstract : public KObject
            {
                KDeclareClass(GoMeasurement, GoMeasurement)

            public:
                /// <summary>Default GoMeasurement constructor.</summary>
                GoMeasurement() {}
            
                /// <summary>Initializes a new instance of the GoMeasurement class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoMeasurement(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Whether or not the measurement has a valid ID assigned to it.</summary>
                property bool HasId
                {
                    bool get()           { return KToBool(::GoMeasurement_HasId(Handle)); }
                }
                
                /// <summary>Clears the assigned measurement ID.</summary>
                void ClearId()
                {
                   KCheck(::GoMeasurement_ClearId(Handle));
                }
            
                /// <summary>The measurement ID.</summary>
                property k32s Id
                {
                    k32s get()           { return (k32s) ::GoMeasurement_Id(Handle); }
                    void set(k32s id)  { KCheck(::GoMeasurement_SetId(Handle, id)); }
                }
            
                /// <summary>The user configured measurement name.</summary>
                property String^ Name
                {
                    String^ get() { return KToString(::GoMeasurement_Name(Handle)); }
                    
                    void set(String^ name)
                    {
                        KString str(name);
                        
                        KCheck(::GoMeasurement_SetName(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The source tool.</summary>
                property KObject^ SourceTool
                {
                    KObject^ get()           { return KToObject<KObject^>(::GoMeasurement_SourceTool(Handle)); }
                }
            
                /// <summary>Enables or disables output.</summary>
                property bool Enabled
                {
                    bool get()           { return KToBool(::GoMeasurement_Enabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoMeasurement_Enable(Handle, enable)); }
                }
            
                /// <summary>The minimum decision value.</summary>
                property k64f DecisionMin
                {
                    k64f get()           { return (k64f) ::GoMeasurement_DecisionMin(Handle); }
                    void set(k64f value)  { KCheck(::GoMeasurement_SetDecisionMin(Handle, value)); }
                }
            
                /// <summary>The maximum decision value.</summary>
                property k64f DecisionMax
                {
                    k64f get()           { return (k64f) ::GoMeasurement_DecisionMax(Handle); }
                    void set(k64f value)  { KCheck(::GoMeasurement_SetDecisionMax(Handle, value)); }
                }
            
                /// <summary>Enables or disables the measurement value hold.</summary>
                property bool HoldEnabled
                {
                    bool get()           { return KToBool(::GoMeasurement_HoldEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoMeasurement_EnableHold(Handle, enable)); }
                }
            
                /// <summary>Enables or disables the measurement value smoothing.</summary>
                property bool SmoothingEnabled
                {
                    bool get()           { return KToBool(::GoMeasurement_SmoothingEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoMeasurement_EnableSmoothing(Handle, enable)); }
                }
                /// <summary>Enables or disables the measurement value smoothing.</summary>
                property bool SmoothingPreserveEnabled
                {
                    bool get()           { return KToBool(::GoMeasurement_XSmoothingPreserveInvalidEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoMeasurement_SetXSmoothingPreserveInvalidEnabled(Handle, enable)); }
                }
                
                /// <summary>The measurement value smoothing window.</summary>
                property k64s SmoothingWindow
                {
                    k64s get()           { return (k64s) ::GoMeasurement_SmoothingWindow(Handle); }
                    void set(k64s value)  { KCheck(::GoMeasurement_SetSmoothingWindow(Handle, value)); }
                }
            
                /// <summary>The measurement value scaling factor.</summary>
                property k64f Scale
                {
                    k64f get()           { return (k64f) ::GoMeasurement_Scale(Handle); }
                    void set(k64f value)  { KCheck(::GoMeasurement_SetScale(Handle, value)); }
                }
            
                /// <summary>The measurement value offset.</summary>
                property k64f Offset
                {
                    k64f get()           { return (k64f) ::GoMeasurement_Offset(Handle); }
                    void set(k64f value)  { KCheck(::GoMeasurement_SetOffset(Handle, value)); }
                }
            
                /// <summary>The measurement type.</summary>
                property GoMeasurementType MeasurementType
                {
                    GoMeasurementType get()           { return (GoMeasurementType) ::GoMeasurement_Type(Handle); }
                }
            };
        }
    }
}

#endif




































