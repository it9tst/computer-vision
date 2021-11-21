// 
// GoDataSet.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_DATA_SET_H
#define GO_SDK_NET_DATA_SET_H

#include <GoSdk/Messages/GoDataSet.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Messages
        {
            /// <summary>Represents a collection of data channel or health channel messages.</summary>
            public ref class GoDataSet : public KObject, IDisposable
            {
                KDeclareClass(GoDataSet, GoDataSet)
            
                /// <summary>Initializes a new instance of the GoDataSet class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoDataSet(IntPtr handle)
                    : KObject(handle,KRefStyle::Auto)
                {}
            
                /// <summary>Initializes a new instance of the GoDataSet class.</summary>
                GoDataSet()
                {
                    ::GoDataSet handle = kNULL;
            
                    KCheck(::GoDataSet_Construct(&handle, kNULL));
            
                    Handle = handle;
                }
                
                /// <inheritdoc cref="GoDataSet()" />
                /// <param name="allocator">Memory allocator</param>
                GoDataSet(KAlloc^ allocator)
                {
                    ::GoDataSet handle = kNULL;
            
                    KCheck(::GoDataSet_Construct(&handle, KToHandle(allocator)));
            
                    Handle = handle;
                }
            
                /// <summary>The sender ID (serial number) associated with this message collection.</summary>
                property k32u SenderId
                {
                    k32u get()           { return ::GoDataSet_SenderId(Handle); }
                }
            
                /// <summary>The message count in this collection.</summary>
                property k64s Count
                {
                    k64s get()           { return (k64s) ::GoDataSet_Count(Handle); }
                }
                
                /// <summary>Gets the message at the specified index.</summary>
                /// <param name="index">Message index.</param>
                KObject^ Get(k64s index)
                {
                   return KToObject<KObject^>(GoDataSet_At(Handle, (kSize)index));
                }
            };
        }
    }
}

#endif
