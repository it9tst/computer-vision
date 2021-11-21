// 
// GoAnalog.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_ANALOG_H
#define GO_SDK_NET_ANALOG_H

#include <GoSdk/Outputs/GoAnalog.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Outputs
        {
            /// <summary>Represents Analog output settings.</summary>
            public ref class GoAnalog : public KObject
            {
                KDeclareClass(GoAnalog, GoAnalog)
            
                /// <summary>Initializes a new instance of the GoAnalog class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoAnalog(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoAnalog class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoAnalog(GoSensor^ sensor)
                {
                    ::GoAnalog handle = kNULL;
            
                    KCheck(::GoAnalog_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoAnalog(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoAnalog(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoAnalog handle = kNULL;
            
                    KCheck(::GoAnalog_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The event which triggers this output to fire.</summary>
                property GoAnalogEvent Event
                {
                    GoAnalogEvent get()           { return (GoAnalogEvent) ::GoAnalog_Event(Handle); }
                    void set(GoAnalogEvent analogEvent)  { KCheck(::GoAnalog_SetEvent(Handle, analogEvent)); }
                }
            
                /// <summary>The number of measurement value source options.</summary>
                property k64s OptionCount
                {
                    k64s get()           { return (k64s) ::GoAnalog_OptionCount(Handle); }
                }
            
                /// <summary>Gets the measurement value source option at the specified index.</summary>
                /// <param name="index">Source option index.</param>
                /// <returns>Source option.</returns>
                k32u GetOption(k64s index)
                {
                    return (k32u) ::GoAnalog_OptionAt(Handle, (kSize)index);
                }
            
                /// <summary>A source type and source identifier for output.</summary>
                property k32u Source
                {
                    k32u get()           { return (k32u) ::GoAnalog_Source(Handle); }
                    void set(k32u sourceId)  { KCheck(::GoAnalog_SetSource(Handle, sourceId)); }
                }
            
                /// <summary>Clears the currently selected source identifier.</summary>
                void ClearSource()
                {
                    KCheck(::GoAnalog_ClearSource(Handle));
                }
            
                /// <summary>The minimum valid value for CurrentMin, CurrentMax and CurrentInvalid settings.</summary>
                property k64f CurrentLimitMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentLimitMin(Handle); }
                }
            
                /// <summary>The maximum valid value for CurrentMin, CurrentMax and CurrentInvalid settings.</summary>
                property k64f CurrentLimitMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentLimitMax(Handle); }
                }
            
                /// <summary>The minimum current output level (mA).</summary>
                property k64f CurrentMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMin(Handle); }
                    void set(k64f min)  { KCheck(::GoAnalog_SetCurrentMin(Handle, min)); }
                }
            
                /// <summary>The minimum allowable current output level to be set for the minimum current.</summary>
                property k64f CurrentMinLimitMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMinLimitMin(Handle); }
                }
            
                /// <summary>The maximum allowable current output level to be set for the minimum current.</summary>
                property k64f CurrentMinLimitMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMinLimitMax(Handle); }
                }
            
                /// <summary>The maximum current output level (mA).</summary>
                property k64f CurrentMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMax(Handle); }
                    void set(k64f max)  { KCheck(::GoAnalog_SetCurrentMax(Handle, max)); }
                }
            
                /// <summary>The minimum allowable current output level to be set for the maximum current.</summary>
                property k64f CurrentMaxLimitMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMaxLimitMin(Handle); }
                }
            
                /// <summary>The maximum allowable current output level to be set for the maximum current.</summary>
                property k64f CurrentMaxLimitMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentMaxLimitMax(Handle); }
                }
            
                /// <summary>The status of the invalid current enabled option.</summary>
                /// <remarks>When this is disabled, the output value will be held constant on an invalid measurement.</remarks>
                property bool CurrentInvalidEnabled
                {
                    bool get()           { return KToBool(::GoAnalog_CurrentInvalidEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoAnalog_EnableCurrentInvalid(Handle, enable)); }
                }
            
                /// <summary>The current output level associated with an invalid measurement.</summary>
                property k64f CurrentInvalid
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentInvalid(Handle); }
                    void set(k64f invalid)  { KCheck(::GoAnalog_SetCurrentInvalid(Handle, invalid)); }
                }
            
                /// <summary>The maximum allowable current output level to be set for the invalid value current.</summary>
                property k64f CurrentInvalidLimitMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentInvalidLimitMax(Handle); }
                }
            
                /// <summary>The minimum allowable current output level to be set for the invalid value current.</summary>
                property k64f CurrentInvalidLimitMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_CurrentInvalidLimitMin(Handle); }
                }
            
                /// <summary>The measurement value associated with the minimum output current value.</summary>
                property k64f DataScaleMin
                {
                    k64f get()           { return (k64f) ::GoAnalog_DataScaleMin(Handle); }
                    void set(k64f min)  { KCheck(::GoAnalog_SetDataScaleMin(Handle, min)); }
                }
            
                /// <summary>The measurement value associated with the maximum output current value.</summary>
                property k64f DataScaleMax
                {
                    k64f get()           { return (k64f) ::GoAnalog_DataScaleMax(Handle); }
                    void set(k64f max)  { KCheck(::GoAnalog_SetDataScaleMax(Handle, max)); }
                }
            
                /// <summary>
                /// The delay from exposure until output is triggered, in units based on GoDomain
                /// mm when GoDomain is distance, uS when GoDomain is time.
                /// </summary>
                /// <remarks>The delay is ignored when GoDomain is Immediate or when output is Software triggered.</remarks>
                property k64s Delay
                {
                    k64s get()           { return (k64s) ::GoAnalog_Delay(Handle); }
                    void set(k64s delay)  { KCheck(::GoAnalog_SetDelay(Handle, delay)); }
                }
            
                /// <summary>The output delay domain.</summary>
                property GoOutputDelayDomain DelayDomain
                {
                    GoOutputDelayDomain get()           { return (GoOutputDelayDomain) ::GoAnalog_DelayDomain(Handle); }
                    void set(GoOutputDelayDomain delayDomain)  { KCheck(::GoAnalog_SetDelayDomain(Handle, delayDomain)); }
                }
            
                /// <summary>Enables or disables the scheduler for this output.</summary>
                property bool ScheduleEnabled
                {
                    bool get()           { return KToBool(::GoAnalog_ScheduleEnabled(Handle)); }
                    void set(bool enable)  { KCheck(::GoAnalog_EnableSchedule(Handle, enable)); }
                }
            };
        }
    }
}

#endif





















