//
// GoEthernet.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_ETHERNET_H
#define GO_SDK_NET_ETHERNET_H

#include <GoSdk/Outputs/GoEthernet.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Outputs
        {
            /// <summary>Represents Ethernet output settings.</summary>
            public ref class GoEthernet : public KObject
            {
                KDeclareClass(GoEthernet, GoEthernet)

                /// <summary>Initializes a new instance of the GoEthernet class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoEthernet(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>Initializes a new instance of the GoEthernet class.</summary>
                /// <param name="sensor">A GoSensor instance.</param>
                GoEthernet(GoSensor^ sensor)
                {
                    ::GoEthernet handle = kNULL;

                    KCheck(::GoEthernet_Construct(&handle, KToHandle(sensor), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoEthernet(GoSensor^ sensor)" />
                /// <param name="allocator">Memory allocator</param>
                GoEthernet(GoSensor^ sensor, KAlloc^ allocator)
                {
                    ::GoEthernet handle = kNULL;

                    KCheck(::GoEthernet_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>
                /// Indicates whether the ethernet connection timeout is enabled. If it is
                /// enabled, the configured timeout value will be used.
                /// </summary>
                property bool TimeoutEnabled
                {
                    bool get()           { return KToBool(::GoEthernet_TimeoutEnabled(Handle)); }
                    void set(bool value)  { KCheck(::GoEthernet_EnableTimeout(Handle, value)); }
                }

                /// <summary>
                /// Indicates whether the ethernet connection timeout is
                /// supported or not by the ethernet protocol. If connection
                /// timeout is not supported by the protocol, then
                /// connection timeout enable/disable and the timeout value
                /// are not meaningful.
                /// </summary>
                property bool TimeoutEnabledIsAvailable
                {
                    bool get()
                    {
                        return KToBool(::GoEthernet_TimeoutEnabledIsAvailable(Handle));
                    }
                }

                /// <summary>The connection timeout value.</summary>
                property k64f Timeout
                {
                    k64f get()           { return (k64f) ::GoEthernet_Timeout(Handle); }
                    void set(k64f value)  { KCheck(::GoEthernet_SetTimeout(Handle, value)); }
                }

                /// <summary>Gets the number of source options for the specified output type.</summary>
                /// <param name="type">Output source type.</param>
                /// <returns>Count of source options.</returns>
                k64s GetOptionCount(GoOutputSource type)
                {
                   return (k64s) ::GoEthernet_OptionCount(Handle, type);
                }

                /// <summary>Gets the source option at the specified index.</summary>
                /// <param name="type">Output source type.</param>
                /// <param name="index">Source option index.</param>
                /// <returns>Source option.</returns>
                k32u GetOption(GoOutputSource type, k64s index)
                {
                   return (k32u) ::GoEthernet_OptionAt(Handle, type, (kSize)index);
                }

                /// <summary>
                /// Gets the number of sources of the specified output type that have been selected for transmission.
                /// The selected output source continues to be included in the count even if the source is no longer valid,
                /// such as by deleting the tool that generated the source.
                /// The count decrements only when an output source is unselected.
                /// </summary>
                /// <param name="type">Output source type.</param>
                /// <returns>Count of selected sources.</returns>
                k64s GetSourceCount(GoOutputSource type)
                {
                   return (k64s) ::GoEthernet_SourceCount(Handle, type);
                }

                /// <summary>Gets the source option at the specified index.</summary>
                /// <param name="type">Output source type.</param>
                /// <param name="index">Source option index.</param>
                /// <returns>Source identifier.</returns>
                k32u GetSource(GoOutputSource type, k64s index)
                {
                   return (k64s) ::GoEthernet_SourceAt(Handle, type, (kSize)index);
                }

                /// <summary>Selects the specified source for transmission.</summary>
                /// <param name="type">Output source type.</param>
                /// <param name="sourceId">Output source identifier.</param>
                void AddSource(GoOutputSource type, k32u sourceId)
                {
                   KCheck(::GoEthernet_AddSource(Handle, type, sourceId));
                }

                /// <summary>Removes (deselects) the source at the specified index.</summary>
                /// <param name="type">Output source type.</param>
                /// <param name="index">Index of the source to be removed.</param>
                void RemoveSource(GoOutputSource type, k64s index)
                {
                   KCheck(::GoEthernet_RemoveSource(Handle, type, (kSize)index));
                }

                /// <summary>Removes (deselects) all selected sources for the specified output type.</summary>
                /// <param name="type">Output source type.</param>
                void ClearSources(GoOutputSource type)
                {
                   KCheck(::GoEthernet_ClearSources(Handle, type));
                }

                /// <summary>Removes (deselects) all selected sources for all possible ethernet output types.</summary>
                void ClearAllSources()
                {
                   KCheck(::GoEthernet_ClearAllSources(Handle));
                }

                /// <summary>Gets the number of composite source options for the specified output type.</summary>
                /// <param name="type">Output source type.</param>
                /// <returns>Count of composite source options.</returns>
                k64s GetCompositeOptionCount(GoOutputSource type)
                {
                   return (k64s) ::GoEthernet_CompositeOptionCount(Handle, type);
                }

                /// <summary>Gets the number of composite source options for the specified output type.</summary>
                /// <param name="type">Output source type.</param>
                /// <param name="index">List index</param>
                /// <returns>Composite source option.</returns>
                GoOutputCompositeSource GetCompositeOption(GoOutputSource type, k64s index)
                {
                   return (GoOutputCompositeSource) ::GoEthernet_CompositeOptionAt(Handle, type, (kSize)index);
                }

                /// <summary>Gets the number of composite sources of the specified output type that are currently selected for transmission.</summary>
                /// <param name="type">Output CompositeSource type.</param>
                /// <returns>Count of selected composite data sources.</returns>
                k64s GetCompositeSourceCount(GoOutputSource type)
                {
                   return (k64s) ::GoEthernet_CompositeSourceCount(Handle, type);
                }

                /// <summary>Gets the identifier of the selected output at the specified index.</summary>
                /// <param name="type">Output CompositeSource type.</param>
                /// <param name="index">Selected index.</param>
                /// <returns>Composite source value.</returns>
                GoOutputCompositeSource GetCompositeSource(GoOutputSource type, k64s index)
                {
                   return (GoOutputCompositeSource) ::GoEthernet_CompositeSourceAt(Handle, type, (kSize)index);
                }

                /// <summary>Selects the specified composite source for transmission.</summary>
                /// <param name="type">Output CompositeSource type.</param>
                /// <param name="source">Composite source identifier.</param>
                void AddCompositeSource(GoOutputSource type, GoOutputCompositeSource source)
                {
                    KCheck(::GoEthernet_AddCompositeSource(Handle, type, (::GoOutputCompositeSource)source));
                }

                /// <summary>Removes (deselects) the composite source at the specified index.</summary>
                /// <param name="type">Output type.</param>
                /// <param name="index">Index of the composite source to be removed.</param>
                void RemoveCompositeSource(GoOutputSource type, k64s index)
                {
                    KCheck(::GoEthernet_RemoveCompositeSource(Handle, type, (kSize)index));
                }

                /// <summary>The number of event options.</summary>
                property k64s EventOptionCount
                {
                    k64s get()           { return (k64s) ::GoEthernet_EventOptionCount(Handle); }
                }

                /// <summary>Gets the event option at the specified index.</summary>
                /// <param name="index">Event EventOption index.</param>
                /// <returns>Event EventOption.</returns>
                GoEventType GetEventOption(k64s index)
                {
                    return (GoEventType) ::GoEthernet_EventOptionAt(Handle, (kSize)index);
                }

                /// <summary>The number of events selected for Gocator protocol output.</summary>
                property k64s EventCount
                {
                    k64s get()           { return (k64s) ::GoEthernet_EventCount(Handle); }
                }

                /// <summary>Gets the output event associated with the specified index.</summary>
                /// <param name="index">Selected event index.</param>
                /// <returns>Event identifier.</returns>
                GoEventType GetEvent(k64s index)
                {
                    return (GoEventType) ::GoEthernet_EventAt(Handle, (kSize)index);
                }

                /// <summary>Selects the specified event for transmission.</summary>
                /// <param name="type">The type of event to add/select.</param>
                void AddEvent(GoEventType type)
                {
                    KCheck(::GoEthernet_AddEvent(Handle, type));
                }

                /// <summary>Removes (deselects) the Event at the specified index.</summary>
                /// <param name="index">Index of the event to be removed.</param>
                void RemoveEvent(k64s index)
                {
                    KCheck(::GoEthernet_RemoveEvent(Handle, (kSize)index));
                }

                /// <summary>Removes (deselects) all selected events.</summary>
                void ClearEvents()
                {
                    KCheck(::GoEthernet_ClearEvents(Handle));
                }

                /// <summary>The protocol that the ethernet utilizes for output.</summary>
                property GoEthernetProtocol Protocol
                {
                    GoEthernetProtocol get()           { return (GoEthernetProtocol) ::GoEthernet_Protocol(Handle); }
                    void set(GoEthernetProtocol protocol)  { KCheck(::GoEthernet_SetProtocol(Handle, protocol)); }
                }

                /// <summary>The ASCII operation mode.</summary>
                property GoAsciiOperation AsciiOperation
                {
                    GoAsciiOperation get()           { return (GoAsciiOperation) ::GoEthernet_AsciiOperation(Handle); }
                    void set(GoAsciiOperation mode)  { KCheck(::GoEthernet_SetAsciiOperation(Handle, mode)); }
                }

                /// <summary>The ASCII operation mode.</summary>
                property GoAsciiStandardFormatMode StandardFormatMode
                {
                    GoAsciiStandardFormatMode get()           { return (GoAsciiStandardFormatMode) ::GoEthernet_AsciiStandardFormat(Handle); }
                    void set(GoAsciiStandardFormatMode mode)  { KCheck(::GoEthernet_SetAsciiStandardFormat(Handle, mode)); }
                }

                /// <summary>The ASCII protocol control channel port number.</summary>
                property k32u AsciiControlPort
                {
                    k32u get()           { return (k32u) ::GoEthernet_AsciiControlPort(Handle); }
                    void set(k32u port)  { KCheck(::GoEthernet_SetAsciiControlPort(Handle, port)); }
                }

                /// <summary>The ASCII protocol health channel port number.</summary>
                property k32u AsciiHealthPort
                {
                    k32u get()           { return (k32u) ::GoEthernet_AsciiHealthPort(Handle); }
                    void set(k32u port)  { KCheck(::GoEthernet_SetAsciiHealthPort(Handle, port)); }
                }

                /// <summary>The ASCII protocol data channel port number.</summary>
                property k32u AsciiDataPort
                {
                    k32u get()           { return (k32u) ::GoEthernet_AsciiDataPort(Handle); }
                    void set(k32u port)  { KCheck(::GoEthernet_SetAsciiDataPort(Handle, port)); }
                }

                /// <summary>The ASCII protocol output delimiter string.</summary>
                property String^ AsciiDelimiter
                {
                    String^ get()
                    {
                        return KToString(::GoEthernet_AsciiDelimiter(Handle));
                    }

                    void set(String^ delimiter)
                    {
                        KString str(delimiter);

                        KCheck(::GoEthernet_SetAsciiDelimiter(Handle, str.CharPtr));
                    }
                }

                /// <summary>The ASCII protocol output terminator string.</summary>
                property String^ AsciiTerminator
                {
                    String^ get()
                    {
                        return KToString(::GoEthernet_AsciiTerminator(Handle));
                    }

                    void set(String^ terminator)
                    {
                        KString str(terminator);

                        KCheck(::GoEthernet_SetAsciiTerminator(Handle, str.CharPtr));
                    }
                }

                /// <summary>The ASCII protocol output invalid value string.</summary>
                property String^ AsciiInvalidValue
                {
                    String^ get()
                    {
                        return KToString(::GoEthernet_AsciiInvalidValue(Handle));
                    }

                    void set(String^ invalidValue)
                    {
                        KString str(invalidValue);

                        KCheck(::GoEthernet_SetAsciiInvalidValue(Handle, str.CharPtr));
                    }
                }

                /// <summary>The ASCII protocol output custom data format string.</summary>
                property String^ AsciiCustomDataFormat
                {
                    String^ get()
                    {
                        return KToString(::GoEthernet_AsciiCustomDataFormat(Handle));
                    }

                    void set(String^ customDataFormat)
                    {
                        KString str(customDataFormat);

                        KCheck(::GoEthernet_SetAsciiCustomDataFormat(Handle, str.CharPtr));
                    }
                }

                /// <summary>The value of whether the ASCII protocol custom data format is enabled or disabled.</summary>
                property bool AsciiCustomFormatEnabled
                {
                    bool get()              { return KToBool(::GoEthernet_AsciiCustomFormatEnabled(Handle)); }
                    void set(bool enabled)  { KCheck(::GoEthernet_EnableAsciiCustomFormat(Handle, enabled)); }
                }

                /// <summary>Enables or disables EthernetIP protocol output buffering.</summary>
                property bool EIPBufferingEnabled
                {
                    bool get()              { return KToBool(::GoEthernet_EIPBufferingEnabled(Handle)); }
                    void set(bool enabled)  { KCheck(::GoEthernet_SetEIPBufferingEnabled(Handle, enabled)); }
                }

                /// <summary>The value of the EthernetIP protocol endian output type.</summary>
                property GoEndianType EIPEndianOutputType
                {
                    GoEndianType get()           { return (GoEndianType) ::GoEthernet_EIPEndianOutputType(Handle); }
                    void set(GoEndianType type)  { KCheck(::GoEthernet_SetEIPEndianOutputType(Handle, type)); }
                }

                /// <summary>The value of the EthernetIP protocol endian output type.</summary>
                property GoImplicitTriggerOverride EIPImplicitTriggerOverride
                {
                    GoImplicitTriggerOverride get()           { return (GoImplicitTriggerOverride) ::GoEthernet_EIPImplicitTriggerOverride(Handle); }
                    void set(GoImplicitTriggerOverride type)  { KCheck(::GoEthernet_SetEIPImplicitTriggerOverride(Handle, type)); }
                }

                /// <summary>The value of whether Modbus protocol output buffering is enabled or disabled.</summary>
                property bool ModbusBufferingEnabled
                {
                    bool get()              { return KToBool(::GoEthernet_ModbusBufferingEnabled(Handle)); }
                    void set(bool enabled)  { KCheck(::GoEthernet_SetModbusBufferingEnabled(Handle, enabled)); }
                }

                /// <summary>The value of the Profinet device's IP address.</summary>
                property KIpAddress ProfinetIpAddress
                {
                    KIpAddress get()
                    {
                        return (KIpAddress) *::GoEthernet_ProfinetIpAddress(Handle);
                    }
                }

                /// <summary>The value of the Profinet device's IP subnet mask.</summary>
                property KIpAddress ProfinetSubnetMask
                {
                    KIpAddress get()
                    {
                        return (KIpAddress) *::GoEthernet_ProfinetSubnetMask(Handle);
                    }
                }

                /// <summary>The value of the Profinet device's IP gateway.</summary>
                property KIpAddress ProfinetGateway
                {
                    KIpAddress get()
                    {
                        return (KIpAddress) *::GoEthernet_ProfinetGateway(Handle);
                    }
                }

                /// <summary>The value of the Profinet device's name.</summary>
                property String^ ProfinetDeviceName
                {
                    String^ get()
                    {
                        return KToString(::GoEthernet_ProfinetDeviceName(Handle));
                    }
                }
            };
        }
    }
}

#endif
