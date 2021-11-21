// 
// GoTransform.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_TRANSFORM_H
#define GO_SDK_NET_TRANSFORM_H

#include <GoSdk/GoTransform.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a sensor transformation.</summary>
        public ref class GoTransform : public KObject
        {
            KDeclareClass(GoTransform, GoTransform)

            /// <summary>Initializes a new instance of the GoTransform class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoTransform(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoTransform class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoTransform(GoSensor^ sensor)
            {
                ::GoTransform handle = kNULL;

                KCheck(::GoTransform_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoTransform(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoTransform(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoTransform handle = kNULL;

                KCheck(::GoTransform_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The encoder resolution.</summary>
            property k64f EncoderResolution
            {
                k64f get()           { return ::GoTransform_EncoderResolution(Handle); }
                void set(k64f value)  { KCheck(::GoTransform_SetEncoderResolution(Handle, value)); }
            }

            /// <summary>The encoder speed.</summary>
            property k64f Speed
            {
                k64f get()           { return ::GoTransform_Speed(Handle); }
                void set(k64f value)  { KCheck(::GoTransform_SetSpeed(Handle, value)); }
            }
            
            /// <summary>Gets the X component of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation X component.</returns>
            k64f GetX(GoRole role)
            {
                return ::GoTransform_X(Handle, role);
            }
            
            /// <summary>Sets the transformation X component.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation X component to set.</param>
            void SetX(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetX(Handle, role, offset));
            }
            
            /// <summary>Gets the Y component of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation Y component.</returns>
            k64f GetY(GoRole role)
            {
                return ::GoTransform_Y(Handle, role);
            }
            
            /// <summary>Sets the transformation Y component.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation Y component to set.</param>
            void SetY(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetY(Handle, role, offset));
            }
            
            /// <summary>Gets the Z component of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation Z component.</returns>
            k64f GetZ(GoRole role)
            {
                return ::GoTransform_Z(Handle, role);
            }
            
            /// <summary>Sets the transformation Z component.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation Z component to set.</param>
            void SetZ(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetZ(Handle, role, offset));
            }
            
            /// <summary>Gets the X-angle of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation X-angle.</returns>
            k64f GetXAngle(GoRole role)
            {
                return ::GoTransform_XAngle(Handle, role);
            }
            
            /// <summary>Sets the transformation X-angle.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation X-angle to set.</param>
            void SetXAngle(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetXAngle(Handle, role, offset));
            }
            
            /// <summary>Gets the Y-angle of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation Y-angle.</returns>
            k64f GetYAngle(GoRole role)
            {
                return ::GoTransform_YAngle(Handle, role);
            }
            
            /// <summary>Sets the transformation Y-angle.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation Y-angle to set.</param>
            void SetYAngle(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetYAngle(Handle, role, offset));
            }
            
            /// <summary>Gets the Z-angle of the transformation.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <returns>The transformation Z-angle.</returns>
            k64f GetZAngle(GoRole role)
            {
                return ::GoTransform_ZAngle(Handle, role);
            }
            
            /// <summary>Sets the transformation Z-angle.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details. </param>
            /// <param name="offset">The transformation Z-angle to set.</param>
            void SetZAngle(GoRole role, k64f offset)
            {
                KCheck(::GoTransform_SetZAngle(Handle, role, offset));
            }
        };
    }
}

#endif
























