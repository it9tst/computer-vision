//
// GoMultiplexBank.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_MULTIPLEX_BANK_H
#define GO_SDK_NET_MULTIPLEX_BANK_H

#include <GoSdk/GoMultiplexBank.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoSensor.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a bank of related sensors to be used in multiplexing.</summary>
        public ref class GoMultiplexBank : public KObject
        {
            KDeclareClass(GoMultiplexBank, GoMultiplexBank)

            /// <summary>Initializes a new instance of the GoMultiplexBank class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoMultiplexBank(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoMultiplexBank class.</summary>
            GoMultiplexBank(k32u id)
            {
                ::GoMultiplexBank handle = kNULL;

                KCheck(::GoMultiplexBank_Construct(&handle, id, kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoMultiplexBank(k32u id)" />
            /// <param name="allocator">Memory allocator</param>
            GoMultiplexBank(k32u id, KAlloc^ allocator)
            {
                ::GoMultiplexBank handle = kNULL;

                KCheck(::GoMultiplexBank_Construct(&handle, id, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Adds a sensor to the given multiplexing bank.</summary>
            /// <param name="sensor">A connected handle to the sensor to add.</param>
            void AddSensor(GoSensor^ sensor)
            {
                KCheck(::GoMultiplexBank_AddSensor(Handle, KToHandle(sensor)));
            }

            /// <summary>Adds a sensor to the given multiplexing bank.</summary>
            /// <param name="id">The ID of the sensor in the bank to the remove.</param>
            void RemoveSensor(k32u id)
            {
                KCheck(::GoMultiplexBank_RemoveSensor(Handle, id));
            }

            /// <summary>The count corresponding to the number of sensors in the multiplexing bank.</summary>
            property k64s SensorCount
            {
                k64s get()           { return (k64s) ::GoMultiplexBank_SensorCount(Handle); }
            }

            /// <summary>Gets a sensor handle from the given index.</summary>
            /// <param name="index">Multiplexing bank index from which to obtain a sensor handle.</param>
            /// <returns>A section.</returns>
            GoSensor^ GetSensor(k64s index)
            {
                return KToObject<GoSensor^>(::GoMultiplexBank_SensorAt(Handle, (kSize)index));
            }

            /// <summary>Adds a sensor to the given multiplexing bank.</summary>
            /// <param name="id">The sensor ID to search for.</param>
            /// <returns>true if the sensor is in the multiplexing bank, false otherwise.</returns>
            bool HasSensor(k32u id)
            {
                return KToBool(::GoMultiplexBank_HasSensor(Handle, id));
            }

            /// <summary>The ID of the given multiplexing bank.</summary>
            property k32u Id
            {
                k32u get()           { return ::GoMultiplexBank_Id(Handle); }
            }
        };
    }
}

#endif
