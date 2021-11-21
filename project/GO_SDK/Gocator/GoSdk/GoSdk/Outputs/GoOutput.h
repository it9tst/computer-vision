/** 
 * @file    GoOutput.h
 * @brief   Declares the GoOutput class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_OUTPUT_H
#define GO_SDK_OUTPUT_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Outputs/GoAnalog.h>
#include <GoSdk/Outputs/GoDigital.h>
#include <GoSdk/Outputs/GoEthernet.h>
#include <GoSdk/Outputs/GoSerial.h>

/**
 * @class   GoOutput
 * @extends kObject
 * @ingroup GoSdk-Output
 * @brief   Represents output configuration.
 */
typedef kObject GoOutput; 

/** 
 * Gets the Ethernet output configuration object.
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 4.0.10.27
 * @param   output      GoOutput object.
 * @return              Ethernet configuration object.            
 */
GoFx(GoEthernet) GoOutput_Ethernet(GoOutput output);

/** 
* [Deprecated]
*
* Gets the Serial output configuration object.
*
* @deprecated
* @public              @memberof GoOutput
* @version             Introduced in firmware 4.0.10.27
* @param   output      GoOutput object.
* @return              Serial configuration object.            
*/
GoFx(GoSerial) GoOutput_Serial(GoOutput output);

/** 
 * Gets the count of Serial output configuration objects. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @return              Count of Serial output configuration objects.            
 */
GoFx(k32u) GoOutput_SerialCount(GoOutput output);

/** 
 * Gets the Serial output configuration object at the specified index. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @param   index       Serial output index.
 * @return              Serial output configuration object.            
 */
GoFx(GoSerial) GoOutput_SerialAt(GoOutput output, kSize index);

/** 
 * Gets the count of Digital output configuration objects. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @return              Count of digital output configuration objects.            
 */
GoFx(k32u) GoOutput_DigitalCount(GoOutput output);

/** 
 * Gets the Digital output configuration object at the specified index. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @param   index       Digital output index.
 * @return              Digital output configuration object.            
 */
GoFx(GoDigital) GoOutput_DigitalAt(GoOutput output, kSize index);

/** 
* [Deprecated] 
*
* Gets the Analog output configuration object.
*
* @deprecated 
* @public              @memberof GoOutput
* @version             Introduced in firmware 4.0.10.27
* @param   output      GoOutput object.
* @return              Analog configuration object.            
*/
GoFx(GoAnalog) GoOutput_Analog(GoOutput output);

/** 
 * Gets the count of Analog output configuration objects. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @return              Count of analog output configuration objects.            
 */
GoFx(k32u) GoOutput_AnalogCount(GoOutput output);

/** 
 * Gets the Analog output configuration object at the specified index. 
 *
 * @public              @memberof GoOutput
 * @version             Introduced in firmware 5.2.18.3
 * @param   output      GoOutput object.
 * @param   index       Analog output index.
 * @return              Analog output configuration object.            
 */
GoFx(GoAnalog) GoOutput_AnalogAt(GoOutput output, kSize index);

#include <GoSdk/Outputs/GoOutput.x.h>

#endif
