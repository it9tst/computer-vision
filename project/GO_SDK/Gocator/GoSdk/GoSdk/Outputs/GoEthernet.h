/**
 * @file    GoEthernet.h
 * @brief   Declares the GoEthernet class.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_ETHERNET_H
#define GO_SDK_ETHERNET_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoEthernet
 * @extends kObject
 * @ingroup GoSdk-Ethernet
 * @brief   Represents Ethernet output settings.
 */
typedef kObject GoEthernet;

/**
 * Indicates whether the ethernet connection timeout is enabled. If it is
 * enabled, the configured timeout value will be used.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.3.3.124
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if enabled and kFALSE otherwise.
 * @see                     GoEthernet_Timeout, GoEthernet_SetTimeout
 */
GoFx(kBool) GoEthernet_TimeoutEnabled(GoEthernet ethernet);

/**
 * Enables or disables the ethernet connection timeout.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.3.3.124
 * @param   ethernet        GoEthernet object.
 * @param   value           kTRUE to enable the connection timeout and
 *                          kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_EnableTimeout(GoEthernet ethernet, kBool value);

/**
* Indicates whether ethernet connection timeout is supported or not by
* the ethernet protocol. If connection timeout is not supported, then
* enabling/disabling the connection timeout and the timeout value are
* not meaningful.
*
* @public                  @memberof GoEthernet
* @version                 Introduced in firmware 4.7.11.27.
* @param   ethernet        GoEthernet object.
* @return                  True means the ethernet protocol supports connection
*                          timeout.
*                          False means otherwise.
*/
GoFx(kBool) GoEthernet_TimeoutEnabledIsAvailable(GoEthernet ethernet);

/**
 * Returns the connection timeout value in minutes.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.3.3.124
 * @param   ethernet        GoEthernet object.
 * @return                  Connection timeout value in minutes.
 */
GoFx(k64f) GoEthernet_Timeout(GoEthernet ethernet);

/**
 * Sets the connection timeout value.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.3.3.124
 * @param   ethernet        GoEthernet object.
 * @param   value           The timeout value (in minutes).
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetTimeout(GoEthernet ethernet, k64f value);

/**
 * Gets the number of source options for the specified output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Count of source options.
 */
GoFx(kSize) GoEthernet_OptionCount(GoEthernet ethernet, GoOutputSource type);

/**
 * Gets the source option at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Source option index.
 * @return                  Source option.
 */
GoFx(k32u) GoEthernet_OptionAt(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Gets the number of sources of the specified output type that have been selected for transmission.
 * The selected output source continues to be included in the count even if the source is no longer valid,
 * such as by deleting the tool that generated the source.
 * The count decrements only when an output source is unselected.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Count of selected sources.
 */
GoFx(kSize) GoEthernet_SourceCount(GoEthernet ethernet, GoOutputSource type);

/**
 * Gets the identifier of the selected output at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Selected source index.
 * @return                  Source identifier.
 */
GoFx(k32u) GoEthernet_SourceAt(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Selects the specified source for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   sourceId        Output source identifier.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_AddSource(GoEthernet ethernet, GoOutputSource type, k32s sourceId);

/**
 * Removes (deselects) the source at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           Index of the source to be removed.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_RemoveSource(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Removes (deselects) all selected sources for the specified output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_ClearSources(GoEthernet ethernet, GoOutputSource type);

/**
 * Removes (deselects) all selected sources for all possible ethernet output types.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_ClearAllSources(GoEthernet ethernet);


/**
 * Gets the number of composite source options for the specified output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14

 * @param   ethernet        GoEthernet object.
 * @param   type            Output data type.
 * @return                  Count of composite source options.
 */
GoFx(kSize) GoEthernet_CompositeOptionCount(GoEthernet ethernet, GoOutputSource type);

/**
 * Gets the composite source option at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14
 * @param   ethernet        GoEthernet object.
 * @param   type            Output source type.
 * @param   index           List index.
 * @return                  Composite source option.
 */
GoFx(GoOutputCompositeSource) GoEthernet_CompositeOptionAt(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Gets the number of composite sources of the specified output type that are currently selected for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14
 * @param   ethernet        GoEthernet object.
 * @param   type            Output CompositeSource type.
 * @return                  Count of selected composite data sources.
 */
GoFx(kSize) GoEthernet_CompositeSourceCount(GoEthernet ethernet, GoOutputSource type);

/**
 * Gets the identifier of the selected output at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14
 * @param   ethernet        GoEthernet object.
 * @param   type            Output CompositeSource type.
 * @param   index           Selected index.
 * @return                  Composite source value.
 */
GoFx(GoOutputCompositeSource) GoEthernet_CompositeSourceAt(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Selects the specified composite source for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14
 * @param   ethernet        GoEthernet object.
 * @param   type            Output CompositeSource type.
 * @param   source          Composite source identifier.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_AddCompositeSource(GoEthernet ethernet, GoOutputSource type, GoOutputCompositeSource source);

/**
 * Removes (deselects) the composite source at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.4.4.14
 * @param   ethernet        GoEthernet object.
 * @param   type            Output type.
 * @param   index           Index of the composite source to be removed.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_RemoveCompositeSource(GoEthernet ethernet, GoOutputSource type, kSize index);

/**
 * Gets the number of event options.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @return                  Count of event options.
 */
GoFx(kSize) GoEthernet_EventOptionCount(GoEthernet ethernet);

/**
 * Gets the event option at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @param   index           Event EventOption index.
 * @return                  Event EventOption.
 */
GoFx(GoEventType) GoEthernet_EventOptionAt(GoEthernet ethernet, kSize index);

/**
 * Gets the number of events selected for Gocator protocol output.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @return                  Count of selected Events.
 */
GoFx(kSize) GoEthernet_EventCount(GoEthernet ethernet);

/**
 * Gets the output event associated with the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @param   index           Selected event index.
 * @return                  Event identifier.
 */
GoFx(GoEventType) GoEthernet_EventAt(GoEthernet ethernet, kSize index);

/**
 * Selects the specified event for transmission.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @param   type            The type of event to add/select.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_AddEvent(GoEthernet ethernet, GoEventType type);

/**
 * Removes (deselects) the Event at the specified index.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @param   index           Index of the event to be removed.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_RemoveEvent(GoEthernet ethernet, kSize index);

/**
 * Removes (deselects) all selected events.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_ClearEvents(GoEthernet ethernet);

/**
 * Gets the protocol that the ethernet utilizes for output.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  The ethernet protocol.
 */
GoFx(GoEthernetProtocol) GoEthernet_Protocol(GoEthernet ethernet);

/**
 * Sets the protocol which will be output via ethernet.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   protocol        The selected ethernet protocol.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetProtocol(GoEthernet ethernet, GoEthernetProtocol protocol);

/**
 * Gets the ASCII protocol operational mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Operation mode.
 */
GoFx(GoAsciiOperation) GoEthernet_AsciiOperation(GoEthernet ethernet);

/**
 * Sets the ASCII operation mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   mode            The selected ASCII operation mode.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiOperation(GoEthernet ethernet, GoAsciiOperation mode);

/**
 * Gets the ASCII protocol control channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Control channel port number.
 */
GoFx(k32u) GoEthernet_AsciiControlPort(GoEthernet ethernet);

/**
 * Sets the port number of the ASCII control port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII control port value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiControlPort(GoEthernet ethernet, k32u port);

/**
 * Gets the ASCII protocol health channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Health channel port number.
 */
GoFx(k32u) GoEthernet_AsciiHealthPort(GoEthernet ethernet);

/**
 * Sets the port number of the ASCII health port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII health port value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiHealthPort(GoEthernet ethernet, k32u port);

/**
 * Gets the ASCII protocol data channel port number.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  Data channel port number.
 */
GoFx(k32u) GoEthernet_AsciiDataPort(GoEthernet ethernet);

/**
 * Sets the port number of the ASCII data port.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   port            The selected ASCII data port value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiDataPort(GoEthernet ethernet, k32u port);

/**
 * Gets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoEthernet_AsciiDelimiter(GoEthernet ethernet);

/**
 * Sets the ASCII protocol output delimiter string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output delimiter.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiDelimiter(GoEthernet ethernet, const kChar* string);

/**
 * Gets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the terminator representing the ASCII protocol output delimiter.
 */
GoFx(kChar*) GoEthernet_AsciiTerminator(GoEthernet ethernet);

/**
 * Sets the ASCII protocol output terminator string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output terminator.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiTerminator(GoEthernet ethernet, const kChar* string);

/**
 * Gets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output invalid value.
 */
GoFx(kChar*) GoEthernet_AsciiInvalidValue(GoEthernet ethernet);

/**
 * Sets the ASCII protocol output invalid value string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output invalid value.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiInvalidValue(GoEthernet ethernet, const kChar* string);

/**
 * Gets the ASCII protocol output custom data format string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  A pointer to the string representing the ASCII protocol output custom data format.
 */
GoFx(kChar*) GoEthernet_AsciiCustomDataFormat(GoEthernet ethernet);

/**
 * Sets the ASCII protocol output custom data format string.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   string          A pointer to the string representing the ASCII protocol output custom data format (10,000 character limit).
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiCustomDataFormat(GoEthernet ethernet, const kChar* string);

/**
 * Enables or disables the ASCII protocol output custom data format.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable custom data format output. kFALSE to use the default output format.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_EnableAsciiCustomFormat(GoEthernet ethernet, kBool enabled);

/**
 * Returns the value of whether the ASCII protocol custom data format is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if the custom data format is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_AsciiCustomFormatEnabled(GoEthernet ethernet);

/**
 * Sets the current standard format mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @param   mode            The new StandardFormatMode.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetAsciiStandardFormat(GoEthernet ethernet, GoAsciiStandardFormatMode mode);

/**
 * Gets the current standard format mode.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.5.3.57
 * @param   ethernet        GoEthernet object.
 * @return                  The StandardFormatMode.
 */
GoFx(kBool) GoEthernet_AsciiStandardFormat(GoEthernet ethernet);

/**
 * Enables or disables EthernetIP protocol output buffering.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable buffering. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPBufferingEnabled(GoEthernet ethernet, kBool enabled);

/**
 * Returns the value of whether the EthernetIP protocol output buffering is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if buffering is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_EIPBufferingEnabled(GoEthernet ethernet);

/**
 * Sets the EthernetIP protocol endian output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.1.3.106
 * @param   ethernet        GoEthernet object.
 * @param   type            The endian output type to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPEndianOutputType(GoEthernet ethernet, GoEndianType type);

/**
 * Returns the value of the EthernetIP protocol endian output type.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.1.3.106
 * @param   ethernet        GoEthernet object.
 * @return                  Endian output type.
 */
GoFx(kBool) GoEthernet_EIPEndianOutputType(GoEthernet ethernet);

/**
 * Sets the EthernetIP protocol implicit trigger override.
 *
 * @public                  @memberof GoEthernet
 * @version             Introduced in firmware 4.2.4.7
 * @param   ethernet        GoEthernet object.
 * @param   value           The implicit trigger override value to set.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPImplicitTriggerOverride(GoEthernet ethernet, GoImplicitTriggerOverride value);

/**
 * Returns the value of the EthernetIP protocol implicit trigger override.
 *
 * @public                  @memberof GoEthernet
 * @version             Introduced in firmware 4.2.4.7
 * @param   ethernet        GoEthernet object.
 * @return                  Implicit trigger override value.
 */
GoFx(GoImplicitTriggerOverride) GoEthernet_EIPImplicitTriggerOverride(GoEthernet ethernet);

/**
 * Enables or disables Modbus protocol output buffering.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable buffering. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetModbusBufferingEnabled(GoEthernet ethernet, kBool enabled);

/**
 * Returns the value of whether Modbus protocol output buffering is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @version                 Introduced in firmware 4.0.10.27
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if buffering is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_ModbusBufferingEnabled(GoEthernet ethernet);

/**
* Returns the IP address of the Profinet protocol device.
*
* @public                  @memberof GoEthernet
* @version                 Introduced in firmware 5.2.18.3
* @param   ethernet        GoEthernet object.
* @return                  Pointer to Profinet device IP address.
*/
GoFx(const kIpAddress*) GoEthernet_ProfinetIpAddress(GoEthernet ethernet);

/**
* Returns the IP subnet mask of the Profinet protocol device.
*
* @public                  @memberof GoEthernet
* @version                 Introduced in firmware 5.2.18.3
* @param   ethernet        GoEthernet object.
* @return                  Pointer to Profinet device IP subnet mask.
*/
GoFx(const kIpAddress*) GoEthernet_ProfinetSubnetMask(GoEthernet ethernet);

/**
* Returns the IP gateway address of the Profinet protocol device.
*
* @public                  @memberof GoEthernet
* @version                 Introduced in firmware 5.2.18.3
* @param   ethernet        GoEthernet object.
* @return                  Pointer to Profinet device IP gateway address.
*/
GoFx(const kIpAddress*) GoEthernet_ProfinetGateway(GoEthernet ethernet);

/**
* Returns the Profinet protocol device name.
*
* @public                  @memberof GoEthernet
* @version                 Introduced in firmware 5.2.18.3
* @param   ethernet        GoEthernet object.
* @return                  Pointer to the Profinet device name.
*/
GoFx(const kChar*) GoEthernet_ProfinetDeviceName(GoEthernet ethernet);

#include <GoSdk/Outputs/GoEthernet.x.h>

#endif
