// 
// GoDigital.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_DIGITAL_H
#define GO_SDK_NET_DIGITAL_H

#include <GoSdk/Outputs/GoDigital.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Outputs
        {
            /// <summary>Represents Digital output settings.</summary>
            public ref class GoDigital : public KObject
            {
                KDeclareClass(GoDigital, GoDigital)
            
                /// <summary>Initializes a new instance of the GoDigital class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoDigital(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoDigital class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoDigital(GoSensor^ sensor)
                {
                    ::GoDigital handle = kNULL;
            
                    KCheck(::GoDigital_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoDigital(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoDigital(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoDigital handle = kNULL;
            
                    KCheck(::GoDigital_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The event which triggers this output to fire.</summary>
                property GoDigitalEvent Event
                {
                    GoDigitalEvent get()           { return (GoDigitalEvent) ::GoDigital_Event(Handle); }
                    void set(GoDigitalEvent digitalEvent)  { KCheck(::GoDigital_SetEvent(Handle, digitalEvent)); }
                }
            
                /// <summary>The number of available decision source options.</summary>
                property k64s OptionCount
                {
                    k64s get()           { return (k64s) ::GoDigital_OptionCount(Handle); }
                }
                
                /// <summary>Gets the decision source option at the specified index.</summary>
                /// <param name="index">Source option index.</param>
                /// <returns>Source option.</returns>
                k32u GetOption(k64s index)
                {
                   return (k64s) ::GoDigital_OptionAt(Handle, (kSize)index);
                }
            
                /// <summary>The number of decision sources that are currently selected for determining pass/fail state.</summary>
                property k64s SourceCount
                {
                    k64s get()           { return (k64s) ::GoDigital_SourceCount(Handle); }
                }
                
                /// <summary>Gets the identifier of the selected output source at the specified index.</summary>
                /// <param name="index">Selected source index.</param>
                /// <returns>Source identifier.</returns>
                k32u GetSource(k64s index)
                {
                   return (k64s) ::GoDigital_SourceAt(Handle, (kSize)index);
                }
                
                /// <summary>Selects the specified decision source for use in determining pass/fail status.</summary>
                /// <param name="sourceId">Index of the source to be added/selected.</param>
                void AddSource(k32u sourceId)
                {
                   KCheck(::GoDigital_AddSource(Handle, sourceId));
                }
                
                /// <summary>Removes (deselects) the decision source at the specified index.</summary>
                /// <param name="index">Index of the source to be removed.</param>
                void RemoveSource(k64s index)
                {
                   KCheck(::GoDigital_RemoveSource(Handle, (kSize)index));
                }
                
                /// <summary>Removes all selected decision sources.</summary>
                void ClearSources()
                {
                   KCheck(::GoDigital_ClearSources(Handle));
                }
            
                /// <summary>The pass/fail mode for the digital output.</summary>
                property GoDigitalPass PassMode
                {
                    GoDigitalPass get()           { return (GoDigitalPass) ::GoDigital_PassMode(Handle); }
                    void set(GoDigitalPass pass)  { KCheck(::GoDigital_SetPassMode(Handle, pass)); }
                }
            
                /// <summary>The minimum valid value for the Pulse Width setting.</summary>
                property k32u PulseWidthLimitMin
                {
                    k32u get()           { return (k32u) ::GoDigital_PulseWidthLimitMin(Handle); }
                }
            
                /// <summary>The maximum valid value for the Pulse Width setting.</summary>
                property k32u PulseWidthLimitMax
                {
                    k32u get()           { return (k32u) ::GoDigital_PulseWidthLimitMax(Handle); }
                }
            
                /// <summary>The width of digital output pulses.</summary>
                property k32u PulseWidth
                {
                    k32u get()           { return (k32u) ::GoDigital_PulseWidth(Handle); }
                    void set(k32u width)  { KCheck(::GoDigital_SetPulseWidth(Handle, width)); }
                }
            
                /// <summary>The signal type of output.</summary>
                property GoDigitalSignal SignalType
                {
                    GoDigitalSignal get()           { return (GoDigitalSignal) ::GoDigital_SignalType(Handle); }
                    void set(GoDigitalSignal width)  { KCheck(::GoDigital_SetSignalType(Handle, width)); }
                }
            
                /// <summary>The delay from exposure until output is triggered, in units based on GoDomain.</summary>
                /// <remarks>mm units when GoDomain is distance, uS units when GoDomain is time.</remarks>
                property k64s Delay
                {
                    k64s get()           { return (k64s) ::GoDigital_Delay(Handle); }
                    void set(k64s delay)  { KCheck(::GoDigital_SetDelay(Handle, delay)); }
                }
            
                /// <summary>The output delay domain.</summary>
                property GoOutputDelayDomain DelayDomain
                {
                    GoOutputDelayDomain get()           { return (GoOutputDelayDomain) ::GoDigital_DelayDomain(Handle); }
                    void set(GoOutputDelayDomain delayDomain)  { KCheck(::GoDigital_SetDelayDomain(Handle, delayDomain)); }
                }
            
                /// <summary>Enables or disables the scheduler for this output.</summary>
                property bool ScheduleEnabled
                {
                    bool get()           { return KToBool(::GoDigital_ScheduleEnabled(Handle)); }
                    void set(bool enabled)  { KCheck(::GoDigital_EnableSchedule(Handle, enabled)); }
                }

                /// <summary>Enables or disables the output signal inversion.</summary>
                property bool OutputInverted
                {
                    bool get()           { return KToBool(::GoDigital_IsOutputInverted(Handle)); }
                    void set(bool enabled)  { KCheck(::GoDigital_SetOutputInverted(Handle, enabled)); }
                }
            };
        }
    }
}
#endif



























