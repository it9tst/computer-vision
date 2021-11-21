//  
// KUtils.h
//  
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//  
#ifndef K_API_NET_UTILS_H
#define K_API_NET_UTILS_H

#include <kApi/Utils/kUtils.h>
#include "kApiNet/KObject.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Collection of static utility methods</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KUtils : public KObject
            {
                KDeclareNoneClass(KUtils, kUtils)

            public:      
                /// <summary>Calls Dispose on an object if the object is not null.</summary>
                ///
                /// <remarks>If the object is null, this method has no effect.</remarks>
                ///
                /// <param name="object">Object to be disposed.</param>
                static void Dispose(KObject^ object)
                {
                    if (object != nullptr)
                    {
                        delete object;
                    }
                }

                /// <summary>Calls Dispose on an object if the object is not null and sets the object reference to null.</summary>
                ///
                /// <remarks>If the object is null, this method has no effect.</remarks>
                ///
                /// <typeparam name="T">Object type.</typeparam>
                /// <param name="object">Object to be disposed.</param>
                generic <typename T> where T : ref class
                static void DisposeRef(T% object)
                {
                    if (object != nullptr)
                    {
                        delete object;
                    }

                    object = T(); 
                }

                /// <summary>Calls DisposeAll on an object if the object is not null.</summary>
                ///
                /// <para>This method is obsolete. Is it equivalent to the Dispose method.</para>
                ///
                /// <param name="object">Object to be recursively disposed.</param>
                [Obsolete]
                static void DisposeAll(KObject^ object)
                {
                    if (object != nullptr)
                    {
                        delete object;
                    }
                }

                /// <summary>Calls DisposeAll on an object if the object is not null and sets the object reference to null.</summary>
                ///
                /// <para>This method is obsolete. Is it equivalent to the DisposeRef method.</para>
                ///
                /// <typeparam name="T">Object type.</typeparam>
                /// <param name="object">Object to be recursively disposed.</param>
                generic <typename T> where T : KObject
                [Obsolete]
                static void DisposeAllRef(T% object)
                {
                    if (object != nullptr)
                    {
                        delete object;
                    }

                    object = T(); 
                }

                /// <summary>Allocates a block of memory from the application heap.</summary>
                /// 
                /// <remarks>
                /// Memory allocated with this method should be freed with the MemFree function.
                /// </remarks>
                /// 
                /// <param name="size">Size of memory to allocate, in bytes.</param>
                /// <returns>Pointer to the memory block.</returns>
                static IntPtr MemAlloc(k64s size)
                {
                    kPointer mem = kNULL; 

                    KCheck(kMemAlloc(K64sToSize(size), &mem)); 

                    return IntPtr(mem); 
                }

                /// <summary>Allocates and zero-initializes block of memory from the application heap.</summary>
                /// 
                /// <remarks>
                /// Memory allocated with this method should be freed with the MemFree function.
                /// </remarks>
                /// 
                /// <param name="size">Size of memory to allocate, in bytes.</param>
                /// <returns>Pointer to the memory block.</returns>
                static IntPtr MemAllocZero(k64s size)
                {
                    kPointer mem = kNULL; 

                    KCheck(kMemAllocZero(K64sToSize(size), &mem)); 

                    return IntPtr(mem); 
                }

                /// <summary>Frees a block of memory that was allocated using MemAlloc or MemAllocZero.</summary>
                /// 
                /// <param name="mem">Pointer to memory to free.</param>
                static void MemFree(IntPtr mem)
                {
                    KCheck(kMemFree(mem.ToPointer()));
                }

                /// <summary>Frees a block of memory that was allocated using KMemAlloc or KMemAllocZero and resets the memory pointer to null.</summary>
                /// 
                /// <param name="mem">Pointer to pointer to memory to free.</param>
                static void MemFreeRef(IntPtr% mem)
                {
                    KCheck(kMemFree(mem.ToPointer()));

                    mem = IntPtr::Zero;
                }

                /// <summary>Sets a block of memory to the given byte value.</summary>
                /// 
                /// <param name="dest">Destination for the memory set operation.</param>
                /// <param name="fill">Value to be set.</param>
                /// <param name="size">Size of memory block to be set, in bytes.</param>
                static void MemSet(IntPtr dest, kByte fill, k64s size)
                {
                    KCheck(kMemSet(dest.ToPointer(), fill, K64sToSize(size))); 
                }

                /// <summary>Copies memory from a source buffer to a non-overlapping destination.</summary>
                /// 
                /// <param name="dest">Destination for the memory copy.</param>
                /// <param name="src">Source for the memory copy.</param>
                /// <param name="size">Size of memory block to be copied, in bytes.</param>
                static void MemCopy(IntPtr dest, IntPtr src, k64s size)
                {
                    KCheck(kMemCopy(dest.ToPointer(), src.ToPointer(), K64sToSize(size)));
                }

                /// <summary>Copies memory from a source buffer to a potentially-overlapping destination.</summary>
                /// 
                /// <param name="dest">Destination for the memory copy.</param>
                /// <param name="src">Source for the memory copy.</param>
                /// <param name="size">Size of memory block to be copied, in bytes.</param>
                static void MemMove(IntPtr dest, IntPtr src, k64s size)
                {
                    KCheck(kMemMove(dest.ToPointer(), src.ToPointer(), K64sToSize(size)));
                }

                /// <summary>Compares one memory buffer with another.</summary>
                /// 
                /// <param name="a">First buffer.</param>
                /// <param name="b">Second buffer.</param>
                /// <param name="size">Size of memory buffers to be compared, in bytes.</param>
                /// <returns>true if the memory buffers are equal; otherwise, false.</returns>
                static bool MemEquals(IntPtr a, IntPtr b, k64s size)
                {
                    return KToBool(kMemEquals(a.ToPointer(), b.ToPointer(), K64sToSize(size)));
                }

                /// <summary>Copies UTF-8-encoded characters from source to destination.</summary>
                /// 
                /// <param name="dest">Destination for the string copy (KChar pointer).</param>
                /// <param name="capacity">Capacity of destination buffer, in characters.</param>
                /// <param name="src">Source for the string copy (KChar pointer).</param>
                /// <exception cref="KException">Thrown is destination buffer is insufficient.</exception>
                static void StrCopy(IntPtr dest, k64s capacity, IntPtr src)
                {
                    KCheck(kStrCopy((kChar*)dest.ToPointer(),  K64sToSize(capacity), (kChar*)src.ToPointer()));
                }

                /// <summary>Appends UTF-8-encoded characters from source to destination.</summary>
                /// 
                /// <param name="dest">Destination string to append to (KChar pointer).</param>
                /// <param name="capacity">Capacity of destination buffer, in characters.</param>
                /// <param name="src">Source string to append (KChar pointer).</param>
                /// <exception cref="KException">Thrown is destination buffer is insufficient.</exception>
                static void StrCat(IntPtr dest, k64s capacity, IntPtr src)
                {
                    KCheck(kStrCat((kChar*)dest.ToPointer(),  K64sToSize(capacity), (kChar*)src.ToPointer()));
                }

                /// <summary>Converts UTF-8-encoded characters in the given sequence to lower case.</summary>
                /// 
                /// <remarks>
                /// This method currently supports conversion of characters only within the ASCII character range.
                /// </remarks>
                /// 
                /// <param name="str">Character sequence to convert (KChar pointer).</param>
                /// <returns>Operation status.</returns>
                static void StrToLower(IntPtr str)
                {
                    KCheck(kStrToLower((kChar*)str.ToPointer()));
                }

                /// <summary>Tests a pair of UTF-8-encoded character sequences for equality.</summary>
                /// 
                /// <param name="a">First string (KChar pointer).</param>
                /// <param name="b">Second string (KChar pointer).</param>
                /// <returns>true if the character sequences are equal; otherwise, false.</returns>
                static bool StrEquals(IntPtr a, IntPtr b)
                {
                    return KToBool(kStrEquals((kChar*) a.ToPointer(), (kChar*)b.ToPointer()));
                }

                /// <summary>Tests a pair of UTF-8-encoded character sequences for equality, up to a maximum number of characters.</summary>
                /// 
                /// <param name="a">First string (KChar pointer).</param>
                /// <param name="b">Second string (KChar pointer).</param>
                /// <param name="maxCount">Maximum number of characters to compare.</param>
                /// <returns>true if the character sequences are equal; otherwise, false.</returns>
                static bool StrnEquals(IntPtr a, IntPtr b, KSize maxCount)
                {
                    return KToBool(kStrnEquals((kChar*) a.ToPointer(), (kChar*)b.ToPointer(), maxCount));
                }

                /// <summary>Compares one UTF-8-encoded string to another.</summary>
                /// 
                /// <remarks>
                /// <para>The result is negative if the string a is lexically less than string b, positive
                /// if string a is lexically greater than string b, and zero if they are equal.</para>
                /// 
                /// <para>This method performs comparison of UTF-8 encoded characters by Unicode code point.</para>
                /// </remarks>
                /// 
                /// <param name="a">First string (KChar pointer).</param>
                /// <param name="b">Second string (KChar pointer).</param>
                /// <returns>Positive if a is greater than b, negative if b is greater than a; otherwise zero.</returns>
                static k32s StrCompare(IntPtr a, IntPtr b)
                {
                    return kStrCompare((kChar*)a.ToPointer(), (kChar*)b.ToPointer());
                }

                /// <summary>Compares one UTF-8-encoded string to another, only up to a maximum number of characters.</summary>
                /// 
                /// <remarks>
                /// <para>The result is negative if the string a is lexically less than string b, positive
                /// if string a is lexically greater than string b, and zero if they are equal.</para>
                /// 
                /// <para>This method performs comparison of UTF-8 encoded characters by Unicode code point.</para>
                /// </remarks>
                /// 
                /// <param name="a">First string (KChar pointer).</param>
                /// <param name="b">Second string (KChar pointer).</param>
                /// <param name="maxCount">Maximum number of characters to compare.</param>
                /// <returns>Positive if a is greater than b, negative if b is greater than a; otherwise zero.</returns>
                static k32s StrnCompare(IntPtr a, IntPtr b, KSize maxCount)
                {
                    return kStrCompareN((kChar*)a.ToPointer(), (kChar*)b.ToPointer(), maxCount);
                }

                /// <summary>Performs a case-insenstive comparison of two UTF-8-encoded strings.</summary>
                /// 
                /// <remarks>
                /// This method currently supports comparison of characters only within the ASCII character range.
                /// </remarks>
                /// 
                /// <param name="a">First string (KChar pointer).</param>
                /// <param name="b">Second string (KChar pointer).</param>
                /// <returns>Positive if a is greater than b, negative if b is greater than a; otherwise zero.</returns>
                static k32s StrCompareLower(IntPtr a, IntPtr b)
                {
                    return kStrCompareLower((kChar*)a.ToPointer(), (kChar*)b.ToPointer());
                }

                // <summary>Determines the number of bytes in a UTF-8-encoded characater sequence.</summary>
                // 
                // <param name="str">Input string (KChar pointer).</param>
                // <returns>Number of kChar units in sequence.</returns>
                static k64s StrLength(IntPtr str)
                {
                    return (k64s)kStrLength((const kChar*)str.ToPointer());
                }

                /// <summary>Finds the first occurrence of a character sequence in a UTF-8-encoded string.</summary>
                /// 
                /// <param name="str">Input string to be searched (KChar pointer).</param>
                /// <param name="subStr">Substring to find (KChar pointer).</param>
                /// <returns>Pointer to first occurrence, or IntPtr.Zero.</returns>
                static IntPtr StrFindFirst(IntPtr str, IntPtr subStr)
                {
                    return IntPtr((kPointer)kStrFindFirst((kChar*)str.ToPointer(), (kChar*)subStr.ToPointer())); 
                }

                /// <summary>Finds the last occurrence of a character sequence in a UTF-8-encoded string.</summary>
                /// 
                /// <param name="str">Input string to be searched (KChar pointer).</param>
                /// <param name="subStr">Substring to find (KChar pointer).</param>
                /// <returns>Pointer to last occurrence, or IntPtr.Zero.</returns>
                static IntPtr StrFindLast(IntPtr str, IntPtr  subStr)
                {
                    return IntPtr((kPointer)kStrFindLast((kChar*)str.ToPointer(), (kChar*)subStr.ToPointer())); 
                }

                /// <summary>Writes to logging handler (if registered). The behaviour of the logging handler is system-specific.
                /// The logging handler is guaranteed to be thread-safe.</summary>
                /// 
                /// <param name="message">Message to log.</param>
                static void Log(String^ message)
                {
                    KString messageStr(message); 

                    KCheck(kLogf("%s", messageStr.CharPtr)); 
                }             

                /// <summary>Generates a random 32-bit number.</summary>
                /// 
                /// <returns>Random 32-bit number.</returns>
                static k32u GetRandom32u()
                {
                    return kRandom32u(); 
                }

                /// <summary>Generates a random 32-bit number.</summary>
                /// 
                /// <returns>Random 32-bit number.</returns>
                static k32s GetRandom32s()
                {
                    return (k32s)kRandom32u(); 
                }

                /// <summary>Generates a random 64-bit number.</summary>
                /// 
                /// <returns>Random 64-bit number.</returns>
                static k64u GetRandom64u()
                {
                    return kRandom64u();
                }

                /// <summary>Generates a random 64-bit number.</summary>
                /// 
                /// <returns>Random 64-bit number.</returns>
                static k64s GetRandom64s()
                {
                    return (k64s) kRandom64u();
                }

                /// <summary>Generates a random number of type KSize.</summary>
                /// 
                /// <returns>Random number.</returns>
                static KSize GetRandomSize()
                {
                    return KSize(kRandomSize()); 
                }

            private:
                KUtils() : KObject(DefaultRefStyle){}
            };
        }
    }
}

#endif
