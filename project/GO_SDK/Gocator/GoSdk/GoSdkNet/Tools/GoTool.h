// 
// GoTool.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_TOOL_H
#define GO_SDK_NET_TOOL_H

#include <GoSdk/Tools/GoTool.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/Tools/goFeature.h>
#include <GoSdkNet/Tools/GoMeasurement.h>
#include <GoSdkNet/Tools/GoMeasurements.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            /// <summary>Represents the base tool class.</summary>
            public ref class GoTool abstract : public KObject
            {
                KDeclareClass(GoTool, GoTool)

            public:
                /// <summary>Default GoTool constructor.</summary>
                GoTool() {}
            
                /// <summary>Initializes a new instance of the GoTool class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoTool(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>The feature count.</summary>
                property k64s FeatureCount
                {
                    virtual k64s get()           { return (k64s) ::GoTool_FeatureOutputCount(Handle); }
                }

                /// <summary>Gets the feature at the given index.</summary>
                /// <param name="index">The index of the feature.</param>
                /// <returns>The feature at the given index or null if an invalid index is provided.</returns>
                virtual GoFeature^ GetFeature(k64s index)
                {
                    ::GoFeature feature = ::GoTool_FeatureOutputAt(Handle, (kSize)index);

                    if (kIsNull(feature)) { return nullptr; }

                    return KToObject<GoFeature^>(feature);
                }
                /// <summary>The measurement count.</summary>
                property k64s MeasurementCount
                {
                    virtual k64s get()           { return (k64s) ::GoTool_MeasurementCount(Handle); }
                }
                
                /// <summary>Gets the measurement at the given index.</summary>
                /// <param name="index">The index of the measurement.</param>
                /// <returns>The measurement at the given index or null if an invalid index is provided.</returns>
                virtual GoMeasurement^ GetMeasurement(k64s index)
                {
                   ::GoMeasurement measurement = ::GoTool_MeasurementAt(Handle, (kSize)index);
                   
                   if (kIsNull(measurement)) { return nullptr; }

                   return GoMeasurements::GetMeasurementInstance(measurement);
                }
               
                /// <summary>The name of the tool.</summary>
                property String^ Name
                {
                    String^ get() 
                    {
                        kText256 name;
                        
                        KCheck(::GoTool_Name(Handle, name, kCountOf(name)));
                        
                        return KToString(name);
                    }
                    
                    void set(String^ name)
                    {
                        KString str(name);
                        
                        KCheck(::GoTool_SetName(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The tool type enumeration value of the tool.</summary>
                property GoToolType ToolType
                {
                    GoToolType get()           { return (GoToolType) ::GoTool_Type(Handle); }
                }
                
                /// <summary>The id of the instance of the tool.</summary>
                property k32s ToolId
                {
                    k32s get()           { return (k32s) ::GoTool_Id(Handle); }
                }
                
                /// <summary>Gets the first found instance of a measurement for a given enumeration type.</summary>
                /// <param name="type">A GoMeasurementType representing the measurement type to find in the given tool.</param>
                /// <returns>A measurement object if one is found, otherwise null.</returns>
                GoMeasurement^ FindMeasurementByType(GoMeasurementType type)
                {
                   ::GoMeasurement measurement = ::GoTool_FindMeasurementByType(Handle, type);
                   
                   if (kIsNull(measurement)) { return nullptr; }

                   return GoMeasurements::GetMeasurementInstance(measurement);
                }
            
                /// <summary>Adds the given measurement to the tool set.</summary>
                /// <param name="type">The type of the measurement.</param>
                /// <param name="isFilterable">Indicates whether the measurement can be toggled and filtered;</param>
                /// <returns>A pointer to hold a reference to the created measurement (can be null).</returns>
                virtual GoMeasurement^ AddMeasurement(KType type, bool isFilterable)
                {
                    ::GoMeasurement measurement;
                    KCheck(GoTool_AddMeasurement(Handle, (kType)type.ToHandle(), isFilterable, &measurement));
                    return KToObject<GoMeasurement^>(measurement);
                }

                /// <summary>Removes all measurements for the given tool.</summary>
                virtual void ClearMeasurements()
                {
                    return KCheck(GoTool_ClearMeasurements(Handle));
                }

                /// <summary>Removes a measurement at a given index.</summary>
                /// <param name="index">The index of the measurement to remove.</param>
                virtual void RemoveMeasurement(k64s index)
                {
                    return KCheck(::GoTool_RemoveMeasurement(Handle, (kSize)index));
                }

                /// <summary>Retrieves the instance of a feature output for a given enumeration type.</summary>
                /// <param name="type">A GoFeatureType representing the feature output type to find in the given tool.
                /// <returns>A feature object if one is found, otherwise kNULL. Returns the first found.</returns>
                GoFeature^ FindFeatureOutputByType(GoFeatureType type)
                {
                    ::GoFeature feature = ::GoTool_FindFeatureOutputByType(Handle, type);

                    if (kIsNull(feature)) 
                    { 
                        return nullptr; 
                    }

                    return KToObject<GoFeature^>(feature);
                }

                /// <summary>Adds the given feature to the tool set.</summary>
                /// <param name="type">The type of the feature.</param>
                /// <returns>A pointer to hold a reference to the created feature (can be null).</returns>
                virtual GoFeature^ AddFeatureOutput(KType type)
                {
                    ::GoFeature featureOutput;
                    KCheck(GoTool_AddFeatureOutput(Handle, (kType)type.ToHandle(), &featureOutput));
                    return KToObject<GoFeature^>(featureOutput);
                }

                /// <summary>Removes all features for the given tool.</summary>
                virtual void ClearFeatureOutputs()
                {
                    return KCheck(GoTool_ClearFeatureOutputs(Handle));
                }

                /// <summary>Removes a feature at a given index.</summary>
                /// <param name="index">The index of the feature to remove.</param>
                virtual void RemoveFeatureOutput(k64s index)
                {
                    return KCheck(::GoTool_RemoveFeatureOutput(Handle, (kSize)index));
                }


            protected:
                /// <summary>Initializes a new instance of the GoTool class.</summary>
                /// <param name="type">The tool instance type.</param>
                /// <param name="sensor">A GoSensor instance.</param>
                GoTool(KType^ type, GoSensor^ sensor)
                {
                    ::GoTool handle = kNULL;
            
                    KCheck(::GoTool_Construct(&handle, KToHandle(type), KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoTool(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoTool(KType^ type, GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoTool handle = kNULL;
            
                    KCheck(::GoTool_Construct(&handle, KToHandle(type), KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            };
        }
    }
}

#endif
