// 
// GoReplayCondition.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_REPLAY_CONDITION_H
#define GO_SDK_NET_REPLAY_CONDITION_H

#include <GoSdk/GoReplayCondition.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents the base class for replay conditions.</summary>
        public ref class GoReplayCondition abstract : public KObject
        {
            KDeclareClass(GoReplayCondition, GoReplayCondition)

            /// <summary>Default GoMeasurement constructor.</summary>
            GoReplayCondition() {}

            /// <summary>Initializes a new instance of the GoReplayCondition class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoReplayCondition(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>The enabled state of a replay condition.</summary>
            property bool Enabled
            {
                bool get()           { return KToBool(::GoReplayCondition_Enabled(Handle)); }
                void set(bool enable)  { KCheck(::GoReplayCondition_Enable(Handle, enable)); }
            }
        };

        /// <summary>Represents the any measurement replay condition.</summary>
        public ref class GoReplayAnyMeasurement : public GoReplayCondition
        {
            KDeclareClass(GoReplayAnyMeasurement, GoReplayAnyMeasurement)

            /// <summary>Initializes a new instance of the GoReplayAnyMeasurement class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoReplayAnyMeasurement(IntPtr handle)
                : GoReplayCondition(handle)
            {}

            /// <summary>The required measurement result.</summary>
            property GoReplayMeasurementResult Result
            {
                GoReplayMeasurementResult get()           { return (GoReplayMeasurementResult) ::GoReplayAnyMeasurement_Result(Handle); }
                void set(GoReplayMeasurementResult result)  { KCheck(::GoReplayAnyMeasurement_SetResult(Handle, result)); }
            }
        };

        /// <summary>Represents the any data replay condition.</summary>
        public ref class GoReplayAnyData : public GoReplayCondition
        {
            KDeclareClass(GoReplayAnyData, GoReplayAnyData)

            /// <summary>Initializes a new instance of the GoReplayAnyData class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoReplayAnyData(IntPtr handle)
                : GoReplayCondition(handle)
            {}

            /// <summary>The required range count case.</summary>
            property GoReplayRangeCountCase RangeCountCase
            {
                GoReplayRangeCountCase get()           { return (GoReplayRangeCountCase) ::GoReplayAnyData_RangeCountCase(Handle); }
                void set(GoReplayRangeCountCase rangeCase)  { KCheck(::GoReplayAnyData_SetRangeCountCase(Handle, rangeCase)); }
            }

            /// <summary>The required range count threshold.</summary>
            property k32u RangeCountThreshold
            {
                k32u get()           { return (k32u) ::GoReplayAnyData_RangeCountThreshold(Handle); }
                void set(k32u threshold)  { KCheck(::GoReplayAnyData_SetRangeCountThreshold(Handle, threshold)); }
            }
        };

        /// <summary>Represents the measurement replay condition.</summary>
        public ref class GoReplayMeasurement : public GoReplayCondition
        {
            KDeclareClass(GoReplayMeasurement, GoReplayMeasurement)

            /// <summary>Initializes a new instance of the GoReplayMeasurement class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoReplayMeasurement(IntPtr handle)
                : GoReplayCondition(handle)
            {}

            /// <summary>The required measurement result.</summary>
            property GoReplayMeasurementResult Result
            {
                GoReplayMeasurementResult get()           { return (GoReplayMeasurementResult) ::GoReplayMeasurement_Result(Handle); }
                void set(GoReplayMeasurementResult result)  { KCheck(::GoReplayMeasurement_SetResult(Handle, result)); }
            }

            /// <summary>The ID count.</summary>
            property k64s IdCount
            {
                k64s get()           { return (k64s) ::GoReplayMeasurement_IdCount(Handle); }
            }

            /// <summary>Gets the ID at the specified index.</summary>
            /// <param name="index">The index of the ID.</param>
            /// <returns>The ID at the specified index.</returns>
            k32u GetId(k64s index)
            {
                return ::GoReplayMeasurement_IdAt(Handle, (kSize)index);
            }

            /// <summary>Sets the ID at the specified index.</summary>
            /// <param name="index">The index of the ID.</param>
            /// <param name="id">The ID.</param>
            void SetId(k64s index, k32u id)
            {
                KCheck(::GoReplayMeasurement_SetIdAt(Handle, (kSize)index, id));
            }
        };
    }
}

#endif
