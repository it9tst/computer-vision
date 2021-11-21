//
// KArray3.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ARRAY3_H
#define K_API_NET_ARRAY3_H

#include <kApi/Data/kArray3.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a 3D array.</summary>
            /// 
            /// <remarks>
            /// <para>The KArray3 class represents a 3D array of objects or values. The KArray3 constructor accepts arguments
            /// that determine the array item type (KType) and array dimension lengths.</para>
            /// 
            /// <para>KArray3 supports the KObject.Clone and KObject.Size methods.</para>
            /// 
            /// <para>KArray3 supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KArray3 : public KObject
            {
                KDeclareAutoClass(KArray3, kArray3)

            public:
                /// <summary>Initializes a new instance of the KArray3 class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KArray3(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KArray3(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KArray3(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KArray3 class without specifying an item type.</summary>
                KArray3()
                    : KObject(DefaultRefStyle)
                {
                    kArray3 handle = kNULL;

                    KCheck(kArray3_Construct(&handle, kTypeOf(kVoid), 0, 0, 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KArray3 class with the specified item type and dimension lengths.</summary>
                /// 
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length0">Length of first array dimension (outermost).</param>
                /// <param name="length1">Length of second array dimension.</param>
                /// <param name="length2">Length of third array dimension (innermost).</param>
                KArray3(KType^ itemType, k64s length0, k64s length1, k64s length2)
                    : KObject(DefaultRefStyle)
                {
                    kArray3 handle = kNULL;

                    KCheck(kArray3_Construct(&handle, KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray3(KType^, k64s, k64s, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KArray3(KType^ itemType, k64s length0, k64s length1, k64s length2, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kArray3 handle = kNULL;

                    KCheck(kArray3_Construct(&handle, KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray3(KType^, k64s, k64s, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KArray3(KType^ itemType, k64s length0, k64s length1, k64s length2, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kArray3 handle = kNULL;

                    KCheck(kArray3_Construct(&handle, KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KArray3(KType^, k64s, k64s, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                /// <param name="alignment">Memory alignment for data.</param>
                KArray3(KType^ itemType, k64s length0, k64s length1, k64s length2, KAlloc^ allocator, KRefStyle refStyle, KMemoryAlignment alignment)
                    : KObject(refStyle)
                {
                    kArray3 handle = kNULL;

                    KCheck(kArray3_ConstructEx(&handle, KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2, 
                        KToHandle(allocator), KToHandle(allocator), (kMemoryAlignment)alignment));

                    Handle = handle;
                }

                /// <summary>Reallocates the internal array item buffer.</summary>
                /// 
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length0">Length of first array dimension (outermost).</param>
                /// <param name="length1">Length of second array dimension.</param>
                /// <param name="length2">Length of third array dimension (innermost).</param>
                void Allocate(KType^ itemType, k64s length0, k64s length1, k64s length2)
                {
                    Allocate(itemType, length0, length1, length2, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Allocate(KType^, k64s, k64s, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Allocate(KType^ itemType, k64s length0, k64s length1, k64s length2, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Length0, 0, Length1, 0, Length2, refStyle);

                    KCheck(kArray3_Allocate(Handle, KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2)); 
                }

                /// <summary>Attaches the array to an external item buffer.</summary>
                /// 
                /// <remarks>Attached item buffers are not freed when the array is destroyed.</remarks>
                /// 
                /// <param name="items">External item buffer.</param>
                /// <param name="itemType">Type of array element.</param>
                /// <param name="length0">Length of first array dimension (outermost).</param>
                /// <param name="length1">Length of second array dimension.</param>
                /// <param name="length2">Length of third array dimension (innermost).</param>
                void Attach(IntPtr items, KType^ itemType, k64s length0, k64s length1, k64s length2)
                {
                    Attach(items, itemType, length0, length1, length2, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Attach(IntPtr, KType^, k64s, k64s, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Attach(IntPtr items, KType^ itemType, k64s length0, k64s length1, k64s length2, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, length0, 0, length1, 0, length2, refStyle);

                    KCheck(kArray3_Attach(Handle, items.ToPointer(), KToHandle(itemType), (kSize)length0, (kSize)length1, (kSize)length2)); 
                }

                /// <summary>Performs a shallow copy of the source array.</summary>
                /// 
                /// <remarks>Source items are copied by value; if the source array contains objects, the object
                /// handles are copied but the objects are not cloned.</remarks>
                /// 
                /// <param name="source">Source array to be copied.</param>
                void Assign(KArray3^ source)
                {
                    Assign(source, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Assign(KArray3^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                void Assign(KArray3^ source, Nullable<KRefStyle> refStyle)
                {
                    RemoveRefRange(0, Length0, 0, Length1, 0, Length2, refStyle);

                    KCheck(kArray3_Assign(Handle, KToHandle(source))); 

                    AddRefRange(0, Length0, 0, Length1, 0, Length2, refStyle);
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
                    RemoveRefRange(0, Length0, 0, Length1, 0, Length2, refStyle);

                    KCheck(kArray3_Zero(Handle));
                }

                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index0">First dimension index.</param>
                /// <param name="index1">Second dimension index.</param>
                /// <param name="index2">Third dimension index.</param>
                /// <param name="item">Item to be copied into the array.</param>
                generic <typename T>
                void Set(k64s index0, k64s index1, k64s index2, T item)
                {
                    Set(index0, index1, index2, item, Nullable<KRefStyle>());
                }
                
                /// <summary>Sets the value of an item.</summary>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="index0">First dimension index.</param>
                /// <param name="index1">Second dimension index.</param>
                /// <param name="index2">Third dimension index.</param>
                /// <param name="item">Item to be copied into the array.</param>
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                void Set(k64s index0, k64s index1, k64s index2, T item, Nullable<KRefStyle> refStyle)
                {
                    if (kType_IsValue(kArray3_ItemType(Handle)))
                    {
                        KCheckArgs(sizeof(T) == kArray3_ItemSize(Handle));

                        KCheck(kArray3_SetItem(Handle, (kSize)index0, (kSize)index1, (kSize)index2, &item));
                    }
                    else
                    {
                        kObject object = KToHandle(item);

                        RemoveRefRange(index0, 1, index1, 1, index2, 1, refStyle);

                        KCheck(kArray3_SetItem(Handle, (kSize)index0, (kSize)index1, (kSize)index2, &object));

                        AddRefRange(index0, 1, index1, 1, index2, 1, refStyle);
                    }
                }

                /// <summary>Gets the item at the specified indices.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="index0">First dimension index.</param>
                /// <param name="index1">Second dimension index.</param>
                /// <param name="index2">Third dimension index.</param>
                /// <returns>Array item.</returns>
                generic <typename T>
                T Get(k64s index0, k64s index1, k64s index2)
                {
                    return Get<T>(index0, index1, index2, Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="Get(k64s, k64s, k64s)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T Get(k64s index0, k64s index1, k64s index2, Nullable<KRefStyle> refStyle)
                {
                    KCheckArgs(((kSize)index0 < kArray3_Length(Handle, 0)) &&
                        ((kSize)index1 < kArray3_Length(Handle, 1)) &&
                        ((kSize)index2 < kArray3_Length(Handle, 2)));

                    if (kType_IsValue(kArray3_ItemType(Handle)))
                    {
                        void* item = kArray3_At(Handle, (kSize)index0, (kSize)index1, (kSize)index2);
                        T value;

                        KCheckArgs(sizeof(T) == kArray3_ItemSize(Handle));

                        kItemCopy(&value, item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kArray3_At(Handle, (kSize)index0, (kSize)index1, (kSize)index2);
                        
                        AddRefRange(index0, 1, index1, 1, index2, 1, refStyle);

                        return KToObject<T>(object, refStyle);
                    }
                }

                /// <summary>Gets a pointer to the array item buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kArray3_Data(Handle)); }
                }

                /// <summary>Gets the size, in bytes, of the array item buffer.</summary>
                property k64s DataSize
                {
                    k64s get() { return (k64s)kArray3_DataSize(Handle); }
                }

                /// <summary>Gets the array item type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kArray3_ItemType(Handle)); }
                }

                /// <summary>Gets the array item size.</summary>
                property k64s ItemSize
                {
                    k64s get() { return (k64s)kArray3_ItemSize(Handle); }
                }

                /// <summary>Gets the length of the specified array dimension, in elements.</summary>
                /// 
                /// <param name="dimension">Array dimension index.</param>
                /// <returns>Array dimension length (in elements).</returns>
                k64s GetLength(k64s dimension)
                {
                    return (k64s)kArray3_Length(Handle, (kSize)dimension); 
                }

                /// <summary>Gets the length of the outermost array dimension, in elements.</summary>
                property k64s Length0
                {
                    k64s get() { return (k64s)kArray3_Length(Handle, 0); }
                }

                /// <summary>Gets the length of the centermost array dimension, in elements.</summary>
                property k64s Length1
                {
                    k64s get() { return (k64s)kArray3_Length(Handle, 1); }
                }

                /// <summary>Gets the length of the innermost array dimension, in elements.</summary>
                property k64s Length2
                {
                    k64s get() { return (k64s)kArray3_Length(Handle, 2); }
                }

                /// <summary>Gets the array item count, in elements.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s)kArray3_Count(Handle); }
                }

            private:
                void AddRefRange(k64s start0, k64s count0, k64s start1, k64s count1, k64s start2, k64s count2, Nullable<KRefStyle> refStyle)
                {
                    AdjustRefRange(start0, count0, start1, count1, start2, count2, kTRUE, refStyle); 
                }

                void RemoveRefRange(k64s start0, k64s count0, k64s start1, k64s count1, k64s start2, k64s count2, Nullable<KRefStyle> refStyle)
                { 
                    AdjustRefRange(start0, count0, start1, count1, start2, count2, kFALSE, refStyle);
                }

                /// <summary>Adjusts the ref count of a range of objects, based on the refStyle parameter or the objects' default ref style.</summary>
                void AdjustRefRange(k64s start0, k64s count0, k64s start1, k64s count1, k64s start2, k64s count2, kBool add, Nullable<KRefStyle> refStyle)
                {
                    if (!kType_IsValue(kArray3_ItemType(Handle)))
                    {
                        KCheckArgs((start0 + count0) * (start1 + count1) * (start2 + count2) <= Count);

                        for (k64s i = start0; i < (start0 + count0); ++i)
                        { 
                            for (k64s j = start1; j < (start1 + count1); ++j)
                            {
                                for (k64s k = start2; k < (start2 + count2); ++k)
                                {
                                    kObject object = kNULL;

                                    KCheck(kArray3_Item(Handle, (kSize)i, (kSize)j, (kSize)k, &object));
                                                                
                                    KAdjustRef(object, add, refStyle);
                                }
                            }
                        }
                    }
                }
            };
        }
    }
}

#endif
