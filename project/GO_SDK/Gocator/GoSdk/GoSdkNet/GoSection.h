// 
// GoSection.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SECTION_H
#define GO_SDK_NET_SECTION_H

#include <GoSdk/GoSection.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents sections to be applied on surface data.</summary>
        public ref class GoSection : public KObject
        {
            KDeclareClass(GoSection, GoSection)

            /// <summary>Initializes a new instance of the GoSection class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSection(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSection class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            /// <param name="id">The section ID.</param>
            GoSection(GoSensor^ sensor, k16s id)
            {
                ::GoSection handle = kNULL;

                KCheck(::GoSection_Construct(&handle, KToHandle(sensor), id, kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoSection(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoSection(GoSensor^ sensor, k16s id, KAlloc^ allocator)
            {
                ::GoSection handle = kNULL;

                KCheck(::GoSection_Construct(&handle, KToHandle(sensor), id, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The ID of the section.</summary>
            property k16s Id
            {
                k16s get()           { return ::GoSection_Id(Handle); }
            }

            /// <summary>The name of the section.</summary>
            property String^ Name
            {
                String^ get() 
                {
                    kText256 name;
                    
                    KCheck(::GoSection_Name(Handle, name, kCountOf(name)));
                    
                    return KToString(name);
                }
                
                void set(String^ name)
                {
                    KString str(name);
                    
                    KCheck(::GoSection_SetName(Handle, str.CharPtr));
                }
            }

            /// <summary>The start point of the section.</summary>
            property KPoint64f StartPoint
            {
                KPoint64f get()
                {
                    kPoint64f point = ::GoSection_StartPoint(Handle);                    
                    return KPoint64f(&point);
                }
                void set(KPoint64f point) { KCheck(::GoSection_SetStartPoint(Handle, point.ToNative())); }
            }

            /// <summary>The end point of the section.</summary>
            property KPoint64f EndPoint
            {
                KPoint64f get()
                {
                    kPoint64f point = ::GoSection_EndPoint(Handle);
                    return KPoint64f(&point);
                }
                void set(KPoint64f point) { KCheck(::GoSection_SetEndPoint(Handle, point.ToNative())); }
            }

            /// <summary>Enables or disables the custom spacing interval.</summary>
            property bool CustomSpacingIntervalEnabled
            {
                bool get()           { return KToBool(::GoSection_CustomSpacingIntervalEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoSection_EnableCustomSpacingInterval(Handle, enable)); }
            }

            /// <summary>The spacing interval of the section.</summary>
            property k64f SpacingInterval
            {
                k64f get()           { return ::GoSection_SpacingInterval(Handle); }
                void set(k64f value)  { KCheck(::GoSection_SetSpacingInterval(Handle, value)); }
            }

            /// <summary>The current spacing interval minimum limit.</summary>
            property k64f SpacingIntervalLimitMin
            {
                k64f get()           { return ::GoSection_SpacingIntervalLimitMin(Handle); }
            }
            
            /// <summary>The current spacing interval maximum limit.</summary>
            property k64f SpacingIntervalLimitMax
            {
                k64f get()           { return ::GoSection_SpacingIntervalLimitMax(Handle); }
            }

            /// <summary>The spacing interval system value.</summary>
            property k64f SpacingIntervalSystemValue
            {
                k64f get()           { return ::GoSection_SpacingIntervalSystemValue(Handle); }
            }
        };
    }
}

#endif
