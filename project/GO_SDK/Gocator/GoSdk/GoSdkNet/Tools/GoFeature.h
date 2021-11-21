// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_FEATURE_H
#define GO_SDK_NET_FEATURE_H

#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            
            
            
            /// <summary>Represents the base class for a tool feature or script output.</summary>
            public ref class GoFeature abstract : public KObject
            {
                KDeclareClass(GoFeature, GoFeature)

            public:
                /// <summary>Default GoFeature constructor.</summary>
                GoFeature() {}

                /// <summary>Initializes a new instance of the GoFeature class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoFeature(IntPtr handle)
                    : KObject(handle)
                {}
               
                /// <summary>The feature ID.</summary>
                property k32s Id
                {
                    k32s get()           { return (k32s) ::GoFeature_Id(Handle); }
                    void set(k32s id)  { KCheck(::GoFeature_SetId(Handle, id)); }
                }
                
                /// <summary>Whether or not the feature has a valid ID assigned to it.</summary>
                property bool HasId
                {
                    bool get()           { return KToBool(::GoFeature_HasId(Handle)); }
                }

                /// <summary>Clears the assigned feature ID.</summary>
                void ClearId()
                {
                    KCheck(::GoFeature_ClearId(Handle));
                }

                /// <summary>The feature name.</summary>
                property String^ Name
                {
                    String^ get() { return KToString(::GoFeature_Name(Handle)); }

                    void set(String^ name)
                    {
                        KString str(name);

                        KCheck(::GoFeature_SetName(Handle, str.CharPtr));
                    }
                }

                /// <summary>The source tool.</summary>
                property KObject^ SourceTool
                {
                    KObject^ get()           { return KToObject<KObject^>(::GoFeature_SourceTool(Handle)); }
                }

                /// <summary>Enables or disables output.</summary>
                property bool Enabled
                {
                    bool get()           { return KToBool(::GoFeature_Enabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoFeature_Enable(Handle, enable)); }
                }

                /// <summary>The feature type.</summary>
                property String^ FeatureType
                {
                    String^ get()           { return KToString(::GoFeature_Type(Handle)); }
                }

                /// <summary>Gets the feature data type for the given feature.</summary>
                property GoFeatureDataType DataType
                {
                    GoFeatureDataType get() { return (GoFeatureDataType)(::GoFeature_DataType(Handle)); }
                }

                /// <summary>Gets the feature type id for the given feature.</summary>
                property GoFeatureType TypeId
                {
                    GoFeatureType get()     { return GoFeatureType(::GoFeature_TypeId(Handle)); }
                }
            };
           
        }
       
    }
}

#endif



