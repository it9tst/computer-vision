/** 
 * @file    GoOutput.c
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <GoSdk/Outputs/GoOutput.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoOutput)
    kAddVMethod(GoOutput, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoOutput_Construct(GoOutput* output, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoOutput), output)); 

    if (!kSuccess(status = GoOutput_Init(*output, kTypeOf(GoOutput), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, output); 
    }

    return status; 
} 

GoFx(kStatus) GoOutput_Init(GoOutput output, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoOutput, output); 
    k32u i;
    kStatus exception;

    kCheck(kObject_Init(output, type, alloc)); 

    obj->sensor = sensor; 
    obj->xml = obj->xmlItem = obj->ethernet = obj->serial = obj->analog = kNULL;
    for (i = 0; i < kCountOf(obj->digital); ++i) obj->digital[i] = kNULL;

    kTry
    {
        kTest(GoEthernet_Construct(&obj->ethernet, sensor, alloc)); 
        kTest(GoSerial_Construct(&obj->serial, sensor, alloc)); 
        kTest(GoAnalog_Construct(&obj->analog, sensor, alloc)); 

        for (i = 0; i < kCountOf(obj->digital); ++i)
        {
            kTest(GoDigital_Construct(&obj->digital[i], sensor, alloc)); 
        }
    }
    kCatch(&exception)
    {
        GoOutput_VRelease(output);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoOutput_VRelease(GoOutput output)
{
    kObjR(GoOutput, output); 
    k32u i;

    kCheck(kDestroyRef(&obj->ethernet)); 
    kCheck(kDestroyRef(&obj->serial)); 
    kCheck(kDestroyRef(&obj->analog)); 

    for (i = 0; i < kCountOf(obj->digital); ++i)
    {
        kCheck(kDestroyRef(&obj->digital[i])); 
    }

    kCheck(kObject_VRelease(output)); 

    return kOK; 
}

GoFx(kStatus) GoOutput_Read(GoOutput output, kXml xml, kXmlItem item)
{
    kObjR(GoOutput, output);
    kXmlItem ethernetItem = kNULL; 
    kXmlItem serialItem = kNULL; 
    kXmlItem digitalItem = kNULL; 
    kXmlItem analogItem = kNULL; 
    k32u i; 
    kString digitalString = kNULL;
    
    obj->xml = xml;
    obj->xmlItem = item;

    kCheck(!kIsNull(ethernetItem = kXml_Child(xml, item, "Ethernet"))); 
    kCheck(GoEthernet_Read(obj->ethernet, xml, ethernetItem)); 
    
    kCheck(!kIsNull(serialItem = kXml_Child(xml, item, "Serial"))); 
    kCheck(GoSerial_Read(obj->serial, xml, serialItem)); 

    kCheck(!kIsNull(analogItem = kXml_Child(xml, item, "Analog"))); 
    kCheck(GoAnalog_Read(obj->analog, xml, analogItem)); 

    kTry
    {
        kTest(kString_Construct(&digitalString, "", kObject_Alloc(output)));

        for (i = 0; i < kCountOf(obj->digital); ++i)
        {
            kTest(kString_Setf(digitalString, "Digital%u", i));
            digitalItem = kXml_Child(xml, item, kString_Chars(digitalString)); 
            kTest(!kIsNull(digitalItem)); 
            kTest(GoDigital_Read(obj->digital[i], xml, digitalItem)); 
        }
    }
    kFinally
    {
        kDestroyRef(&digitalString);
        kEndFinally();
    }

    return kOK; 
}

GoFx(kStatus) GoOutput_Write(GoOutput output, kXml xml, kXmlItem item)
{
    kObjR(GoOutput, output);
    kXmlItem ethernetItem = kNULL; 
    kXmlItem serialItem = kNULL; 
    kXmlItem digitalItem = kNULL; 
    kXmlItem analogItem = kNULL; 
    k32u i; 
    kString digitalString = kNULL;

    if (kIsNull(obj->xml) || kIsNull(obj->xmlItem)) return kERROR_STATE;

    kCheck(kXml_AddItem(xml, item, "Ethernet", &ethernetItem)); 
    kCheck(GoEthernet_Write(obj->ethernet, xml, ethernetItem)); 

    kCheck(kXml_AddItem(xml, item, "Serial", &serialItem)); 
    kCheck(GoSerial_Write(obj->serial, xml, serialItem)); 

    kCheck(kXml_AddItem(xml, item, "Analog", &analogItem)); 
    kCheck(GoAnalog_Write(obj->analog, xml, analogItem)); 

    kTry
    {
        kTest(kString_Construct(&digitalString, "", kObject_Alloc(output)));

        for (i = 0; i < kCountOf(obj->digital); ++i)
        {
            kTest(kString_Setf(digitalString, "Digital%u", i));
            kTest(kXml_AddItem(xml, item, kString_Chars(digitalString), &digitalItem)); 
            kTest(GoDigital_Write(obj->digital[i], xml, digitalItem)); 
        }
    }
    kFinally
    {
        kDestroyRef(&digitalString);
        kEndFinally();
    }

    //Forwards Compatibility
    kCheck(GoUtils_XmlMerge(obj->xml, obj->xmlItem, xml, item));
        
    return kOK; 
}

GoFx(GoEthernet) GoOutput_Ethernet(GoOutput output)
{
    kObjR(GoOutput, output);
    return obj->ethernet; 
}

GoFx(k32u) GoOutput_EthernetCount(GoOutput output)
{
    kObjR(GoOutput, output);
    kObjNR(GoEthernet, ethernet, obj->ethernet);
    return (k32u) ethernet->outputUsed; 
}

GoFx(GoEthernet) GoOutput_EthernetAt(GoOutput output, kSize index)
{
    kObjR(GoOutput, output);
    kAssert(index < GoOutput_EthernetCount(output));
    return obj->ethernet; 
}

GoFx(GoSerial) GoOutput_Serial(GoOutput output)
{
    kObjR(GoOutput, output);
    return obj->serial; 
}

GoFx(k32u) GoOutput_SerialCount(GoOutput output)
{
    kObjR(GoOutput, output);
    kObjNR(GoSerial, serial, obj->serial);
    return (k32u) serial->outputUsed;  
}

GoFx(GoSerial) GoOutput_SerialAt(GoOutput output, kSize index)
{
    kObjR(GoOutput, output);
    kAssert(index < GoOutput_SerialCount(output));
    return obj->serial; 
}

GoFx(k32u) GoOutput_DigitalCount(GoOutput output)
{
    kObjR(GoOutput, output);
    kObjNR(GoDigital, digital, kNULL);
    k32u i, numEnabled = 0;

    for (i = 0; i < kCountOf(obj->digital); i++){
        digital = obj->digital[i];
        if (digital->outputUsed){
            numEnabled++;
        }
    }
    return numEnabled; 
}

GoFx(GoDigital) GoOutput_DigitalAt(GoOutput output, kSize index)
{
    kObjR(GoOutput, output);
    kAssert(index < GoOutput_DigitalCount(output));
    return obj->digital[index]; 
}

GoFx(GoAnalog) GoOutput_Analog(GoOutput output)
{
    kObjR(GoOutput, output);
    return obj->analog; 
}

GoFx(k32u) GoOutput_AnalogCount(GoOutput output)
{
    kObjR(GoOutput, output);
    kObjNR(GoAnalog, analog, obj->analog);
    return (k32u) analog->outputUsed; 
}

GoFx(GoAnalog) GoOutput_AnalogAt(GoOutput output, kSize index)
{
    kObjR(GoOutput, output);
    kAssert(index < GoOutput_AnalogCount(output));
    return obj->analog; 
}
