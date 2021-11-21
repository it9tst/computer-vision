// 
// GoProfileGeneration.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_PROFILE_GENERATION_H
#define GO_SDK_NET_PROFILE_GENERATION_H

#include <GoSdk/GoProfileGeneration.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoSensor.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a profile generation configuration.</summary>
        public ref class GoProfileGeneration : public KObject
        {
            KDeclareClass(GoProfileGeneration, GoProfileGeneration)

            /// <summary>Initializes a new instance of the GoProfileGeneration class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoProfileGeneration(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoProfileGeneration class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoProfileGeneration(GoSensor^ sensor)
            {
                ::GoProfileGeneration handle = kNULL;

                KCheck(::GoProfileGeneration_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoProfileGeneration(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoProfileGeneration(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoProfileGeneration handle = kNULL;

                KCheck(::GoProfileGeneration_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The profile generation type.</summary>
            property GoProfileGenerationType GenerationType
            {
                GoProfileGenerationType get()           { return (GoProfileGenerationType) ::GoProfileGeneration_GenerationType(Handle); }
                void set(GoProfileGenerationType type)  { KCheck(::GoProfileGeneration_SetGenerationType(Handle, type)); }
            }

            /// <summary>The fixed length profile generation start trigger.</summary>
            property GoProfileGenerationStartTrigger FixedLengthStartTrigger
            {
                GoProfileGenerationStartTrigger get()           { return (GoProfileGenerationStartTrigger) ::GoProfileGenerationFixedLength_StartTrigger(Handle); }
                void set(GoProfileGenerationStartTrigger trigger)  { KCheck(::GoProfileGenerationFixedLength_SetStartTrigger(Handle, trigger)); }
            }

            /// <summary>The external input to use for profile generation - only applicable if Start Trigger is DigitalInput.</summary>
            property k32s FixedLengthTriggerExternalInputIndex
            {
                k32s get()           { return ::GoProfileGenerationFixedLength_TriggerExternalInputIndex(Handle); }
                void set(k32s source)  { KCheck(::GoProfileGenerationFixedLength_SetTriggerExternalInputIndex(Handle, source)); }
            }

            /// <summary>Returns a boolean value representing whether the external input trigger index is used.</summary>
            property bool FixedLengthTriggerExternalInputIndexUsed
            {
                bool get()           { return ::GoProfileGenerationFixedLength_TriggerExternalInputIndexUsed(Handle); }
            }

            /// <summary>The trigger external input index option count.</summary>
            property k64s FixedLengthTriggerExternalInputIndexOptionCount
            {
                k64s get()           { return (k64s) ::GoProfileGenerationFixedLength_TriggerExternalInputIndexOptionCount(Handle); }
            }

            /// <summary>The fixed length profile generation profile length.</summary>
            property k64f FixedLengthLength
            {
                k64f get()           { return ::GoProfileGenerationFixedLength_Length(Handle); }
                void set(k64f length)  { KCheck(::GoProfileGenerationFixedLength_SetLength(Handle, length)); }
            }

            /// <summary>The fixed length profile generation circumference limit maximum value.</summary>
            property k64f FixedLengthLengthLimitMax
            {
                k64f get()           { return ::GoProfileGenerationFixedLength_LengthLimitMax(Handle); }
            }

            /// <summary>The fixed length profile generation circumference limit minimum value.</summary>
            property k64f FixedLengthLengthLimitMin
            {
                k64f get()           { return ::GoProfileGenerationFixedLength_LengthLimitMin(Handle); }
            }

            /// <summary>The variable length profile generation maximum length.</summary>
            property k64f VariableLengthMaxLength
            {
                k64f get()           { return ::GoProfileGenerationVariableLength_MaxLength(Handle); }
                void set(k64f length)  { KCheck(::GoProfileGenerationVariableLength_SetMaxLength(Handle, length)); }
            }

            /// <summary>The variable length profile generation circumference limit maximum value.</summary>
            property k64f VariableLengthMaxLengthLimitMax
            {
                k64f get()           { return ::GoProfileGenerationVariableLength_MaxLengthLimitMax(Handle); }
            }

            /// <summary>The variable length profile generation circumference limit minimum value.</summary>
            property k64f VariableLengthMaxLengthLimitMin
            {
                k64f get()           { return ::GoProfileGenerationVariableLength_MaxLengthLimitMin(Handle); }
            }

            /// <summary>The rotational profile generation circumference.</summary>
            property k64f RotationalCircumference
            {
                k64f get()           { return ::GoProfileGenerationRotational_Circumference(Handle); }
                void set(k64f value)  { KCheck(::GoProfileGenerationRotational_SetCircumference(Handle, value)); }
            }

            /// <summary>The rotational profile generation circumference limit maximum value.</summary>
            property k64f RotationalCircumferenceLimitMax
            {
                k64f get()           { return ::GoProfileGenerationRotational_CircumferenceLimitMax(Handle); }
            }

            /// <summary>The rotational length profile generation circumference limit minimum value.</summary>
            property k64f RotationalCircumferenceLimitMin
            {
                k64f get()           { return ::GoProfileGenerationRotational_CircumferenceLimitMin(Handle); }
            }
        };
    }
}

#endif
























