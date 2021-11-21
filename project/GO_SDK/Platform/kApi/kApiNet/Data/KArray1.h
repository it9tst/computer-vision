//
// KArray1.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ARRAY1_H
#define K_API_NET_ARRAY1_H

#include <kApi/Data/kArray1.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a 1D array.</summary>
            /// 
            /// <remarks>
            /// <para>The KArray1 class represents a 1D array of objects or values. The KArray1 constructor accepts arguments
            /// that determine the array item type (KType) and array length.</para>
            /// 
            /// <para>KArray1 supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KArray1 supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KArray1 : public KObject
            {
                KDeclareAutoClass(KArray1, kArray1)

            public:
                /// <summary>Initializes a new instance of the KArray1 class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KArray1(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KArray1(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KArray1(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KArray1 class without specifying an item type.</summary>
                KArray1()
                    : KObject(DefaultRefStyle)
                {
                    kArray1 handle = kNULL;

                    KCheck(kArray1_Construct(&handle, kTypeOf(kVoid), 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KArray1 class with the specified item type and length.</summary>
                /// 
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length">Length of array.</param>
                KArray1(KType^ itemType, k64s length)
                    : KObject(DefaultRefStyle)
                {
                    kArray1 handle = kNULL;

                    KCheck(kArray1_Construct(&handle, KToHandle(itemType), (kSize)length, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray1(KType^, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KArray1(KType^ itemType, k64s length, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kArray1 handle = kNULL;

                    KCheck(kArray1_Construct(&handle, KToHandle(itemType), (kSize)length, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray1(KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KArray1(KType^ itemType, k64s length, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kArray1 handle = kNULL;

                    KCheck(kArray1_Construct(&handle, KToHandle(itemType), (kSize)length, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray1(KType^, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <param name="alignment">Memory alignment for data.</param>
                KArray1(KType^ itemType, k64s length, KAlloc^ allocator, KRefStyle refStyle, KMemoryAlignment alignment)
                    : KObject(refStyle)
                {
                    kArray1 handle = kNULL;

                    KCheck(kArray1_ConstructEx(&handle, KToHandle(itemType), (kSize)length, KToHandle(allocator), KToHandle(allocator), (kMemoryAlignment)alignment));

                    Handle = handle;
                }

                /// <summary>Reallocates the internal array item buffer.</summary>
                /// 
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length">Length of array.</param>
                void Allocate(KType^ itemType, k64s length)
                {
                    Allocate(itemType, length, Nullable<KRefStyle>());
                }
                
                /// <inheritdoc cref="Allocate(KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Allocate(KType^ itemType, k64s length, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArray1_Allocate(Handle, KToHandle(itemType), (kSize)length));
                }

                /// <summary>Attaches the array to an external item buffer.</summary>
                /// 
                /// <remarks>Attached item buffers are not freed when the array is destroyed.</remarks>
                /// 
                /// <param name="items">External item buffer.</param>
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length">Length of array.</param>
                void Attach(IntPtr items, KType^ itemType, k64s length)
                {
                    Attach(items, itemType, length, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Attach(IntPtr, KType^, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Attach(IntPtr items, KType^ itemType, k64s length, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, RefStyle);

                    KCheck(kArray1_Attach(Handle, items.ToPointer(), KToHandle(itemType), (kSize)length));
                }

                /// <summary>Performs a shallow copy of the source array.</summary>
                /// 
                /// <remarks>Source items are copied by value; if the source array contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">Source array to be copied.</param>
                void Assign(KArray1^ source)
                {
                    Assign(source, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Assign(KArray1^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KArray1^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArray1_Assign(Handle, KToHandle(source)));

                    AddRefRange(0, Count, refStyle);
                }

                /// <summary>Sets all array element bits to zero.</summary>
                void Zero()
                {
                    Zero(Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Zero()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Zero(Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Count, refStyle);

                    KCheck(kArray1_Zero(Handle)); 
                }

                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Array item index.</param>
                /// <param name="item">Item to be copied into the array.</param>
                generic <typename T>
                void Set(k64s index, T item)
                {
                    Set(index, item, Nullable<KRefStyle>()); 
                    
                }

                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index">Array item index.</param>
                /// <param name="item">Item to be copied into the array.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Set(k64s index, T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArray1_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kArray1_ItemSize(Handle));

                        KCheck(kArray1_SetItem(Handle, (kSize)index, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item); 

                        RemoveRefRange(index, 1, refStyle); 

                        KCheck(kArray1_SetItem(Handle, (kSize)index, &object));

                        AddRefRange(index, 1, refStyle);
                    }
                }

                /// <summary>Gets the item at the specified index.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="index">Item index.</param>
                /// <returns>Array item.</returns>
                generic <typename T>
                T Get(k64s index)
                {
                    return Get<T>(index, Nullable<KRefStyle>()); 
                }

                /// <inheritdoc cref="Get(k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Get(k64s index, Nullable<KRefStyle> refStyle)
                {
                    KCheckArgs((kSize)index < kArray1_Count(Handle));

                    if (kType_IsValue(kArray1_ItemType(Handle)))
                    {
                        void* item = kArray1_At(Handle, (kSize)index);
                        T value;

                        KCheckArgs(sizeof(T) == kArray1_ItemSize(Handle));

                        kItemCopy(&value, item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kArray1_At(Handle, (kSize)index);

                        AddRefRange(index, 1, refStyle);

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Gets a pointer to the array item buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kArray1_Data(Handle)); }
                }

                /// <summary>Gets the size, in bytes, of the array item buffer.</summary>
                property k64s DataSize
                {
                    k64s get() { return (k64s)kArray1_DataSize(Handle); }
                }

                /// <summary>Gets the array item type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kArray1_ItemType(Handle)); }
                }

                /// <summary>Gets the array item size, in bytes.</summary>
                property k64s ItemSize
                {
                    k64s get() { return (k64s)kArray1_ItemSize(Handle); }
                }

                /// <summary>Gets the array length, in elements.</summary>
                property k64s Length
                {
                    k64s get() { return (k64s)kArray1_Length(Handle); }
                }

                /// <summary>Gets the array item count, in elements.</summary>
                /// 
                /// <remarks>This method is provided for symmetry with KArray2/KArray3.  In practice,
                /// the item count for a 1D array is always equal to the length.</remarks>
                property k64s Count
                {
                    k64s get() { return (k64s)kArray1_Count(Handle); }
                }

            private:
                void AddRefRange(k64s start, k64s count, Nullable<KRefStyle> refStyle)
                {
                    AdjustRefRange(start, count, kTRUE, refStyle); 
                }

                void RemoveRefRange(k64s start, k64s count, Nullable<KRefStyle> refStyle)
                { 
                    AdjustRefRange(start, count, kFALSE, refStyle);
                }

                /// <summary>Adjusts the ref count of a range of objects, based on the refStyle parameter or the objects' default ref style.</summary>
                void AdjustRefRange(k64s start, k64s count, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kArray1_ItemType(Handle)))
                    {
                        KCheckArgs(start + count <= Count);

                        for (k64s i = start; i < (start + count); ++i)
                        { 
                            kObject object = kNULL;

                            KCheck(kArray1_Item(Handle, (kSize)i, &object));

                            KAdjustRef(object, add, refStyle);
                        }
                    }
                }
            };
        }
    }
}

#endif
