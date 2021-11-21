// 
// GoSurfaceGeneration.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SURFACE_GENERATION_H
#define GO_SDK_NET_SURFACE_GENERATION_H

#include <GoSdk/GoSurfaceGeneration.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoSensor.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a surface generation configuration.</summary>
        public ref class GoSurfaceGeneration : public KObject
        {
            KDeclareClass(GoSurfaceGeneration, GoSurfaceGeneration)

            /// <summary>Initializes a new instance of the GoSurfaceGeneration class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSurfaceGeneration(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSurfaceGeneration class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoSurfaceGeneration(GoSensor^ sensor)
            {
                ::GoSurfaceGeneration handle = kNULL;

                KCheck(::GoSurfaceGeneration_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoSurfaceGeneration(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoSurfaceGeneration(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoSurfaceGeneration handle = kNULL;

                KCheck(::GoSurfaceGeneration_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The surface generation type.</summary>
            property GoSurfaceGenerationType GenerationType
            {
                GoSurfaceGenerationType get()           { return (GoSurfaceGenerationType) ::GoSurfaceGeneration_GenerationType(Handle); }
                void set(GoSurfaceGenerationType type)  { KCheck(::GoSurfaceGeneration_SetGenerationType(Handle, type)); }
            }

            /// <summary>The fixed length surface generation start trigger.</summary>
            property GoSurfaceGenerationStartTrigger FixedLengthStartTrigger
            {
                GoSurfaceGenerationStartTrigger get()           { return (GoSurfaceGenerationStartTrigger) ::GoSurfaceGenerationFixedLength_StartTrigger(Handle); }
                void set(GoSurfaceGenerationStartTrigger trigger)  { KCheck(::GoSurfaceGenerationFixedLength_SetStartTrigger(Handle, trigger)); }
            }

            /// <summary>The external input to use for surface generation - only applicable if Start Trigger is DigitalInput.</summary>
            property k32s FixedLengthTriggerExternalInputIndex
            {
                k32s get()           { return ::GoSurfaceGenerationFixedLength_TriggerExternalInputIndex(Handle); }
                void set(k32s source)  { KCheck(::GoSurfaceGenerationFixedLength_SetTriggerExternalInputIndex(Handle, source)); }
            }

            /// <summary>Returns a boolean value representing whether the external input trigger index is used.</summary>
            property bool FixedLengthTriggerExternalInputIndexUsed
            {
                bool get()           { return ::GoSurfaceGenerationFixedLength_TriggerExternalInputIndexUsed(Handle); }
            }

            /// <summary>The trigger external input index option count.</summary>
            property k64s FixedLengthTriggerExternalInputIndexOptionCount
            {
                k64s get()           { return (k64s) ::GoSurfaceGenerationFixedLength_TriggerExternalInputIndexOptionCount(Handle); }
            }

            /// <summary>The fixed length surface generation surface length.</summary>
            property k64f FixedLengthLength
            {
                k64f get()           { return  ::GoSurfaceGenerationFixedLength_Length(Handle); }
                void set(k64f length)  { KCheck(::GoSurfaceGenerationFixedLength_SetLength(Handle, length)); }
            }

            /// <summary>The fixed length surface generation circumference limit maximum value.</summary>
            property k64f FixedLengthLengthLimitMax
            {
                k64f get()           { return  ::GoSurfaceGenerationFixedLength_LengthLimitMax(Handle); }
            }

            /// <summary>The fixed length surface generation circumference limit minimum value.</summary>
            property k64f FixedLengthLengthLimitMin
            {
                k64f get()           { return  ::GoSurfaceGenerationFixedLength_LengthLimitMin(Handle); }
            }

            /// <summary>The variable length surface generation maximum length.</summary>
            property k64f VariableLengthMaxLength
            {
                k64f get()           { return  ::GoSurfaceGenerationVariableLength_MaxLength(Handle); }
                void set(k64f length)  { KCheck(::GoSurfaceGenerationVariableLength_SetMaxLength(Handle, length)); }
            }

            /// <summary>The variable length surface generation circumference limit maximum value.</summary>
            property k64f VariableLengthLengthLimitMax
            {
                k64f get()           { return  ::GoSurfaceGenerationVariableLength_MaxLengthLimitMax(Handle); }
            }

            /// <summary>The variable length surface generation circumference limit minimum value.</summary>
            property k64f VariableLengthLengthLimitMin
            {
                k64f get()           { return  ::GoSurfaceGenerationVariableLength_MaxLengthLimitMin(Handle); }
            }

            /// <summary>The rotational surface generation surface circumference.</summary>
            property k64f RotationalCircumference
            {
                k64f get()           { return  ::GoSurfaceGenerationRotational_Circumference(Handle); }
                void set(k64f value)  { KCheck(::GoSurfaceGenerationRotational_SetCircumference(Handle, value)); }
            }

            /// <summary>The rotational surface generation circumference limit maximum value.</summary>
            property k64f RotationalCircumferenceLimitMax
            {
                k64f get()           { return  ::GoSurfaceGenerationRotational_CircumferenceLimitMax(Handle); }
            }

            /// <summary>The rotational surface generation circumference limit minimum value.</summary>
            property k64f RotationalCircumferenceLimitMin
            {
                k64f get()           { return  ::GoSurfaceGenerationRotational_CircumferenceLimitMin(Handle); }
            }

            /// <summary>Gets/sets the rotational surface generation encoder resolution in ticks/rev.</summary>
            property k64f EncoderResolution
            {
                k64f get()  { return ::GoSurfaceGenerationRotational_EncoderResolution(Handle); }
                void set(k64f value)    { KCheck(::GoSurfaceGenerationRotational_SetEncoderResolution(Handle, value)); }
            }
        };
    }
}

#endif


























