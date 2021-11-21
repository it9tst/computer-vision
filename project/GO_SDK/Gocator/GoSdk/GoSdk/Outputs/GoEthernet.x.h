/**
 * @file    GoEthernet.x.h
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_ETHERNET_X_H
#define GO_SDK_ETHERNET_X_H

#include <GoSdk/Outputs/GoEthernet.h>
#include <kApi/Data/kXml.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kString.h>

// GOC-14410 - Increase GO_OUTPUT_SOURCE_TEXT_CAPACITY from 2048 to 8192 to 
// fix out of memory error (kERROR_MEMORY) with large number of measurements.
#define GO_OUTPUT_SOURCE_TEXT_CAPACITY          (8192)

typedef struct GoValueNamePair
{
    k32s         value;
    const kChar* name;
} GoValueNamePair;

typedef struct GoEthernetClass
{
    kObjectClass base;
    kObject sensor;

    kXml xml;
    kXmlItem xmlItem;

    kArrayList videoOptions;            //of type k32s
    kArrayList videoSources;            //of type k32s
    kArrayList rangeOptions;            //of type k32s
    kArrayList rangeSources;            //of type k32s
    kArrayList rangeIntensityOptions;   //of type k32s
    kArrayList rangeIntensitySources;   //of type k32s
    kArrayList profileOptions;          //of type k32s
    kArrayList profileSources;          //of type k32s
    kArrayList profileIntensityOptions; //of type k32s
    kArrayList profileIntensitySources; //of type k32s
    kArrayList surfaceIntensityOptions; //of type k32s
    kArrayList surfaceIntensitySources; //of type k32s
    kArrayList surfaceOptions;          //of type k32s
    kArrayList surfaceSources;          //of type k32s
    kArrayList sectionIntensityOptions; //of type GoOutputCompositeSource
    kArrayList sectionIntensitySources; //of type GoOutputCompositeSource
    kArrayList sectionOptions;          //of type GoOutputCompositeSource
    kArrayList sectionSources;          //of type GoOutputCompositeSource
    kArrayList measurementOptions;      //of type k32s
    kArrayList measurementSources;      //of type k32s
    kArrayList eventOptions;            //of type k32s
    kArrayList events;                  //of type k32s
    kArrayList tracheidOptions;         //of type k32s
    kArrayList tracheidSources;         //of type k32s
    kArrayList featureOptions;          //of type k32s
    kArrayList featureSources;          //of type k32s
    kArrayList toolDataOptions;         //of type k32s
    kArrayList toolDataSources;         //of type k32s

    // The Avail flag indicates if the feature is available for (supported by)
    // the selected configured Ethernet output protocol. If feature is supported,
    // then the enable/disable and timeout value are meaningful.
    kBool timeoutEnabledAvail;
    kBool timeoutEnabled;
    k64f timeout;

    kBool outputUsed;

    GoEthernetProtocol protocol;
    GoEipConfig eip;
    GoModbusConfig modbus;
    GoAsciiConfig ascii;
    GoProfinetConfig profinet;
    GoPtpConfig ptp;
} GoEthernetClass;

kDeclareClassEx(Go, GoEthernet, kObject)

GoFx(kStatus) GoEthernet_Construct(GoEthernet* ethernet, kObject sensor, kAlloc allocator);

GoFx(kStatus) GoEthernet_Init(GoEthernet ethernet, kType type, kObject sensor, kAlloc alloc);
GoFx(kStatus) GoEthernet_VRelease(GoEthernet ethernet);

GoFx(kStatus) GoEthernet_Read(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_ReadAllOutputConfig(GoEthernet ethernet);

GoFx(kStatus) GoEthernet_Write(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_WriteAllOutputConfig(GoEthernet ethernet, kXml xml, kXmlItem item);

GoFx(kStatus) GoEthernet_ParseOutputSourceList(GoEthernet ethernet, GoOutputSource outputSource, kChar* text);
GoFx(kStatus) GoEthernet_ParseOutputOptionList(GoEthernet ethernet, GoOutputSource outputSource, kChar* text);
GoFx(kStatus) GoEthernet_ParseOutputList(GoOutputSource outputSource, kChar* text, kArrayList outputList);

GoFx(kStatus) GoEthernet_ParseCompositeOptionList(const kChar* text, kArrayList list);
GoFx(kStatus) GoEthernet_ParseHelperComposite(const kChar* text, kSize length, GoOutputCompositeSource* value);

GoFx(kArrayList) GoEthernet_SourceList(GoEthernet ethernet, GoOutputSource type);
GoFx(kArrayList) GoEthernet_OptionList(GoEthernet ethernet, GoOutputSource type);

GoFx(kStatus) GoEthernet_ReadAscii(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_ReadEip(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_ReadModbus(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_ReadProfinet(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_ReadPtp(GoEthernet ethernet, kXml xml, kXmlItem item);

GoFx(kStatus) GoEthernet_WriteAscii(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_WriteEip(GoEthernet ethernet, kXml xml, kXmlItem item);
GoFx(kStatus) GoEthernet_WriteModbus(GoEthernet ethernet, kXml xml, kXmlItem item);

/**
 * Enables or disables EthernetIP protocol implicit output.
 *
 * @public                  @memberof GoEthernet
 * @param   ethernet        GoEthernet object.
 * @param   enabled         kTRUE to enable implicit output. kFALSE to disable it.
 * @return                  Operation status.
 */
GoFx(kStatus) GoEthernet_SetEIPImplicitOutputEnabled(GoEthernet ethernet, kBool enabled);

/**
 * Returns the value of whether the EthernetIP protocol implicit output is enabled or disabled.
 *
 * @public                  @memberof GoEthernet
 * @param   ethernet        GoEthernet object.
 * @return                  kTRUE if implicit output is enabled. kFALSE otherwise.
 */
GoFx(kBool) GoEthernet_EIPImplicitOutputEnabled(GoEthernet ethernet);

#endif
