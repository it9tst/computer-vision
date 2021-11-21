//
// KBox.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_BOX_H
#define K_API_NET_BOX_H

#include <kApi/Data/kBox.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents an instance of a Zen value type as a Zen object.</summary>
            /// 
            /// <remarks>
            /// <para>KBox provides a simple way to represent a single value (e.g. K32s) using a KObject-derived class instance.
            /// This approach can sometimes be used to reduce duplicated effort when both reference types and value types
            /// must be supported in native code.</para>
            /// 
            /// <para>KBox supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KBox supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KBox : public KObject
            {
                KDeclareAutoClass(KBox, kBox)

            public:
                /// <summary>Initializes a new instance of the KBox class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KBox(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KBox(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KBox(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KBox class without specifying an item type.</summary>
                KBox()
                    : KObject(DefaultRefStyle)
                {
                    kBox handle = kNULL;

                    KCheck(kBox_Construct(&handle, kTypeOf(kVoid), kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KBox class with the specified item type.</summary>
                /// 
                /// <param name="itemType">Type of box element (value types only).</param>
                KBox(KType^ itemType)
                    : KObject(DefaultRefStyle)
                {
                    kBox handle = kNULL;

                    KCheck(kBox_Construct(&handle, KToHandle(itemType), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KBox(KType^)" />
                /// <param name="allocator">Memory allocator.</param>
                KBox(KType^ itemType, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kBox handle = kNULL;

                    KCheck(kBox_Construct(&handle, KToHandle(itemType), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KBox(KType^, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KBox(KType^ itemType, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kBox handle = kNULL;

                    KCheck(kBox_Construct(&handle, KToHandle(itemType), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Reallocates the internal box item buffer.</summary>
                /// 
                /// <param name="itemType">Type of box element.</param>
                void Allocate(KType^ itemType)
                {
                    KCheck(kBox_Allocate(Handle, KToHandle(itemType)));
                }

                /// <summary>Copies the value contained within the source box into this box.</summary>
                /// 
                /// <param name="source">Source box to be copied.</param>
                void Assign(KBox^ source)
                {
                    KCheck(kBox_Assign(Handle, KToHandle(source)));
                }

                /// <summary>Sets all box element bits to zero.</summary>
                void Zero()
                {
                    KCheck(kBox_Zero(Handle));
                }

                /// <summary>Sets the box value.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="item">Item to be copied into the box.</param>
                generic <typename T>
                void Set(T item)
                {
                    KCheckArgs(sizeof(T) == kBox_ItemSize(Handle));

                    KCheck(kBox_SetItem(Handle, &item));
                }

                /// <summary>Gets the box value.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <returns>Box item.</returns>
                generic <typename T>
                T Get()
                {
                    void* item = kBox_Data(Handle);
                    T value;

                    KCheckArgs(sizeof(T) == kBox_ItemSize(Handle));

                    kItemCopy(&value, item, sizeof(T));

                    return value;
                }

                /// <summary>Gets a pointer to the box item buffer.</summary>
                /// 
                /// <returns>Pointer to box item buffer.</returns>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kBox_Data(Handle)); }
                }

                /// <summary>Gets the box item type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kBox_ItemType(Handle)); }
                }

                /// <summary>Gets the box item size.</summary>
                property k64s ItemSize
                {
                    k64s get() { return (k64s)kBox_ItemSize(Handle); }
                }
            };
        }
    }
}

#endif
