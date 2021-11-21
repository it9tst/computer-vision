// 
// GoSerial.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SERIAL_H
#define GO_SDK_NET_SERIAL_H

#include <GoSdk/Outputs/GoSerial.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Outputs
        {
            /// <summary></summary>
            public ref class GoSerial : public KObject
            {
                KDeclareClass(GoSerial, GoSerial)
            
                /// <summary>Initializes a new instance of the GoSerial class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSerial(IntPtr handle)
                    : KObject(handle)
                {}
            
                /// <summary>Initializes a new instance of the GoSerial class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoSerial(GoSensor^ sensor)
                {
                    ::GoSerial handle = kNULL;
            
                    KCheck(::GoSerial_Construct(&handle, KToHandle(sensor), kNULL));
            
                    Handle = handle;
                }
            
                /// <inheritdoc cref="GoSerial(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoSerial(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoSerial handle = kNULL;
            
                    KCheck(::GoSerial_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The count of serial protocol options.</summary>
                property k64s ProtocolOptionCount
                {
                    k64s get()           { return (k64s) ::GoSerial_ProtocolOptionCount(Handle); }
                }
                            
                /// <summary>Gets the serial protocol option at the specified index.</summary>
                /// <param name="index">Protocol option index.</param>
                /// <returns>An integer corresponding to the current Serial output protocol.</returns>
                k32u GetProtocolOption(k64s index)
                {
                   return (k32u) ::GoSerial_ProtocolOptionAt(Handle, (kSize)index);
                }
            
                /// <summary>The serial output protocol.</summary>
                property k32u Protocol
                {
                    k32u get()               { return (k32u) ::GoSerial_Protocol(Handle); }
                    void set(k32u protocol)  { KCheck(::GoSerial_SetProtocol(Handle, protocol)); }
                }
            
                /// <summary>The number of source options.</summary>
                property k64s OptionCount
                {
                    k64s get()           { return (k64s) ::GoSerial_OptionCount(Handle); }
                }
                            
                /// <summary>Gets the source option at the specified index.</summary>
                /// <param name="index">Source option index.</param>
                /// <returns>Source option.</returns>
                k32u GetOption(k64s index)
                {
                   return (k32u) ::GoSerial_OptionAt(Handle, (kSize)index);
                }
            
                /// <summary>The number of sources of the specified output type that are currently selected for transmission.</summary>
                property k64s SourceCount
                {
                    k64s get()           { return (k64s) ::GoSerial_SourceCount(Handle); }
                }
                            
                /// <summary>Gets the identifier of the selected output source at the specified index.</summary>
                /// <param name="index">Selected source index.</param>
                /// <returns>Source identifier.</returns>
                k32u GetSource(k64s index)
                {
                   return (k32u) ::GoSerial_SourceAt(Handle, (kSize)index);
                }
                            
                /// <summary>Selects the specified source for transmission.</summary>
                /// <param name="sourceId">Source identifier.</param>
                void AddSource(k32u sourceId)
                {
                   KCheck(::GoSerial_AddSource(Handle, sourceId));
                }
                            
                /// <summary>Removes (de-selects) the source at the specified index.</summary>
                /// <param name="index">Index of the source to be removed.</param>
                void RemoveSource(k64s index)
                {
                   KCheck(::GoSerial_RemoveSource(Handle, (kSize)index));
                }
                            
                /// <summary>Removes all selected sources for the specified output type.</summary>
                void ClearSources()
                {
                   KCheck(::GoSerial_ClearSources(Handle));
                }
            
                /// <summary>The index of the currently selected Selcom output rate.</summary>
                property k32u SelcomRate
                {
                    k32u get()               { return (k32u) ::GoSerial_SelcomRate(Handle); }
                    void set(k32u rate)  { KCheck(::GoSerial_SetSelcomRate(Handle, rate)); }
                }
            
                /// <summary>The list of Selcom rate options.</summary>
                property KArrayList^ SelcomRateOptionList
                {
                    KArrayList^ get()               { return KToObject<KArrayList^>(::GoSerial_SelcomRateOptionList(Handle)); }
                }
            
                /// <summary>The count of available Selcom protocol output rate options.</summary>
                property k64s SelcomRateOptionCount
                {
                    k64s get()               { return (k64s) ::GoSerial_SelcomRateOptionCount(Handle); }
                }
                            
                /// <summary>Gets the Selcom rate option at the specified index.</summary>
                /// <param name="index">Selcom rate option index.</param>
                /// <returns>Selcom rate option value.</returns>
                k32u GetSelcomRateOption(k64s index)
                {
                   return (k32u) ::GoSerial_SelcomRateOptionAt(Handle, (kSize)index);
                }
            
                /// <summary>The Selcom protocol output format.</summary>
                property GoSelcomFormat SelcomFormat
                {
                    GoSelcomFormat get()               { return (GoSelcomFormat) ::GoSerial_SelcomFormat(Handle); }
                    void set(GoSelcomFormat format)  { KCheck(::GoSerial_SetSelcomFormat(Handle, format)); }
                }
            
                /// <summary>The list of Selcom format options.</summary>
                property KArrayList^ SelcomFormatOptionList
                {
                    KArrayList^ get()               { return KToObject<KArrayList^>(::GoSerial_SelcomFormatOptionList(Handle)); }
                }
            
                /// <summary>The count of available Selcom protocol output format options.</summary>
                property k64s SelcomFormatOptionCount
                {
                    k64s get()               { return (k64s) ::GoSerial_SelcomFormatOptionCount(Handle); }
                }
                            
                /// <summary>Gets the Selcom format option at the specified index.</summary>
                /// <param name="index">Selcom format option index.</param>
                /// <returns>Selcom format option value.</returns>
                GoSelcomFormat GetSelcomFormatOption(k64s index)
                {
                   return (GoSelcomFormat) ::GoSerial_SelcomFormatOptionAt(Handle, (kSize)index);
                }
            
                /// <summary>The Selcom protocol maximum data scale value.</summary>
                property k64f SelcomDataScaleMax
                {
                    k64f get()               { return (k64f) ::GoSerial_SelcomDataScaleMax(Handle); }
                    void set(k64f value)  { KCheck(::GoSerial_SetSelcomDataScaleMax(Handle, value)); }
                }
            
                /// <summary>The Selcom protocol minimum data scale value.</summary>
                property k64f SelcomDataScaleMin
                {
                    k64f get()               { return (k64f) ::GoSerial_SelcomDataScaleMin(Handle); }
                    void set(k64f value)  { KCheck(::GoSerial_SetSelcomDataScaleMin(Handle, value)); }
                }
            
                /// <summary>The delay from exposure until output is triggered, in units based on GoDomain.</summary>
                /// <remarks>mm units when GoDomain is distance, uS units when GoDomain is time.</remarks>
                property k64u SelcomDelay
                {
                   k64u get()               { return (k64u) ::GoSerial_SelcomDelay(Handle); }
                    void set(k64u delay)  { KCheck(::GoSerial_SetSelcomDelay(Handle, delay)); }
                }
            
                /// <summary>The ASCII protocol output delimiter string.</summary>
                property String^ AsciiDelimiter
                {
                    String^ get() 
                    {
                        return KToString(::GoSerial_AsciiDelimiter(Handle));
                    }
                    
                    void set(String^ delimiter)
                    {
                        KString str(delimiter);
                        
                        KCheck(::GoSerial_SetAsciiDelimiter(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The ASCII protocol output terminator string.</summary>
                property String^ AsciiTerminator
                {
                    String^ get() 
                    {
                        return KToString(::GoSerial_AsciiTerminator(Handle));
                    }
                    
                    void set(String^ terminator)
                    {
                        KString str(terminator);
                        
                        KCheck(::GoSerial_SetAsciiTerminator(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The ASCII protocol output invalid value string.</summary>
                property String^ AsciiInvalidValue
                {
                    String^ get() 
                    {
                        return KToString(::GoSerial_AsciiInvalidValue(Handle));
                    }
                    
                    void set(String^ invalidValue)
                    {
                        KString str(invalidValue);
                        
                        KCheck(::GoSerial_SetAsciiInvalidValue(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The ASCII protocol output custom data format string.</summary>
                property String^ AsciiCustomDataFormat
                {
                    String^ get() 
                    {
                        return KToString(::GoSerial_AsciiCustomDataFormat(Handle));
                    }
                    
                    void set(String^ customDataFormat)
                    {
                        KString str(customDataFormat);
                        
                        KCheck(::GoSerial_SetAsciiCustomDataFormat(Handle, str.CharPtr));
                    }
                }
            
                /// <summary>The value of whether the ASCII protocol custom data format is enabled or disabled.</summary>
                property bool AsciiCustomFormatEnabled
                {
                    bool get()              { return KToBool(::GoSerial_AsciiCustomFormatEnabled(Handle)); }
                    void set(bool enabled)  { KCheck(::GoSerial_EnableAsciiCustomFormat(Handle, enabled)); }
                }

                /// <summary>The ASCII operation mode.</summary>
                property GoAsciiStandardFormatMode StandardFormatMode
                {
                    GoAsciiStandardFormatMode get()           { return (GoAsciiStandardFormatMode) ::GoSerial_AsciiStandardFormat(Handle); }
                    void set(GoAsciiStandardFormatMode mode)  { KCheck(::GoSerial_SetAsciiStandardFormat(Handle, mode)); }
                }
            };
        }
    }
}

#endif
