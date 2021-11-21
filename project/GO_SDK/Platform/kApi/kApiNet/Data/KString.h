//
// KString.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_STRING_H
#define K_API_NET_STRING_H

#include <kApi/Data/kString.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KArrayList.h"

using namespace System::Text; 

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a character string.</summary>
            /// 
            /// <remarks>
            /// <para>The KString class represents a variable-length, null-terminated sequence of KChar elements.</para>
            /// 
            /// <para>KString supports the KObject.Clone, KObject.Size, KObject.Equals, and KObject.GetHashCode methods.</para>
            /// 
            /// <para>KString supports the kdat5 and kdat6 serialization protocols.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KString : public KObject
            {
                KDeclareAutoClass(KString, kString)

            public:
                /// <summary>Initializes a new instance of the KString class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KString(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KString(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KString(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new, empty instance of the KString class.</summary>
                KString()
                    : KObject(DefaultRefStyle)
                {
                    kString handle = kNULL;

                    KCheck(kString_Construct(&handle, kNULL, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KString class with the specified string content.</summary>
                /// 
                /// <param name="content">Initial string content (or null).</param>
                KString(String^ content)
                    : KObject(DefaultRefStyle)
                {
                    kString handle = kNULL;

                    KCheck(kString_Construct(&handle, kNULL, kNULL));

                    Handle = handle;

                    if (content != nullptr)
                    {
                        try
                        {
                            this->Set(content);
                        }
                        catch (...)
                        {
                            kObject_Destroy(Handle);
                            Handle = kNULL;

                            throw;
                        }
                    }
                }


                /// <inheritdoc cref="KString(String^)" />
                ///
                /// <param name="allocator">Memory allocator.</param>
                KString(String^ content, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kString handle = kNULL;

                    KCheck(kString_Construct(&handle, kNULL, KToHandle(allocator))); 

                    Handle = handle;

                    if (content != nullptr)
                    {
                        try
                        {
                            this->Set(content);
                        }
                        catch (...)
                        {
                            kObject_Destroy(Handle);
                            Handle = kNULL;

                            throw;
                        }
                    }
                }

                /// <inheritdoc cref="KString(String^, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KString(String^ content, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kString handle = kNULL;

                    KCheck(kString_Construct(&handle, kNULL, KToHandle(allocator))); 

                    Handle = handle;

                    if (content != nullptr)
                    {
                        try
                        {
                            this->Set(content);
                        }
                        catch (...)
                        {
                            kObject_Destroy(Handle);
                            Handle = kNULL;

                            throw;
                        }
                    }
                }

                /// <summary>Copies the source string content.</summary>
                /// 
                /// <param name="source">Source string to be copied.</param>
                void Assign(KString^ source)
                {
                    KCheck(kString_Assign(Handle, KToHandle(source))); 
                }

                /// <summary>Sets the length of the string to zero.</summary>
                void Clear()
                {
                    KCheck(kString_Clear(Handle)); 
                }

                /// <summary>Ensures that capacity is reserved for at least the specified number of character units (excluding null terminator).</summary>
                /// 
                /// <remarks>Any string content within the original capacity is preserved.</remarks>
                /// 
                /// <param name="minimumCapacity">Minimum string capacity, in character units.</param>
                void Reserve(k64s minimumCapacity)
                {
                    KCheck(kString_Reserve(Handle, (kSize)minimumCapacity)); 
                }

                /// <summary>Explicitly sets the length of the string.</summary>
                /// 
                /// <remarks>
                /// <para>This method can be used to specify the string length if the string buffer has been directly
                /// manipulated.</para>
                /// 
                /// <para>If the current capacity is less than the specified length, the capacity will be automatically increased. A null
                /// terminator will be written at the end of the string buffer.</para>
                /// 
                /// <para>Use with caution; no error-checking is performed to ensure that the string does not contain extra null
                /// terminators prior to the specified length.</para>
                /// </remarks>
                /// 
                /// <param name="length">New string length, in character units (excluding null-terminator).</param>
                void SetLength(k64s length)
                {
                    KCheck(kString_SetLength(Handle, (kSize)length));
                }

                /// <summary>Sets the content of the string.</summary>
                /// 
                /// <param name="content">String content to copy.</param>
                void Set(String^ content)
                {
                    if (content->Length == 0)
                    {
                        KCheck(kString_Clear(Handle)); 
                    }
                    else
                    {
                        array<Byte>^ bytes = Encoding::UTF8->GetBytes(content);
                        pin_ptr<Byte> data = &bytes[0];
                        kSize length = (kSize)bytes->LongLength;

                        KCheck(kString_SetLength(Handle, length));
                        KCheck(kMemCopy(kString_Chars(Handle), data, length));
                    }
                }

                /// <summary>Appends content to the string.</summary>
                /// 
                /// <param name="content">String content to append.</param>
                void Add(String^ content)
                {
                    if (content->Length > 0)
                    {
                        kSize originalLength = kString_Length(Handle);
                        array<Byte>^ bytes = Encoding::UTF8->GetBytes(content);
                        pin_ptr<Byte> data = &bytes[0];
                        kSize inputLength = (kSize)bytes->LongLength;
                        kSize totalLength = originalLength + inputLength;
                        kChar* strBuffer = kNULL;

                        KCheck(kString_SetLength(Handle, totalLength));
                        strBuffer = kString_Chars(Handle);

                        KCheck(kMemCopy(&strBuffer[originalLength], data, inputLength));
                    }
                }

                /// <summary>Removes leading and trailing whitespace.</summary>
                void Trim()
                {
                    KCheck(kString_Trim(Handle)); 
                }

                /// <summary>Splits this string into substrings using the supplied delimiters.</summary>
                /// 
                /// <remarks>This method currently supports delimiters only within the ASCII character range.</remarks>
                /// 
                /// <param name="delimiters">Null-terminated string containing delimiter characters.</param>
                /// <param name="allocator">Memory allocator (or null for default).</param>
                /// <returns>List of substrings.</returns>
                KArrayList^ Split(String^ delimiters, KAlloc^ allocator)
                {
                    KString delim(delimiters, nullptr); 
                    kArrayList tokens = kNULL; 

                    KCheck(kString_Split(Handle, (kChar*)delim.Chars.ToPointer(), &tokens, KToHandle(allocator)));

                    return gcnew KArrayList(IntPtr(tokens));
                }

                /// <summary>Gets a pointer (IntPtr) to the internal character buffer.</summary>
                property IntPtr Chars
                {
                    IntPtr get() { return IntPtr(kString_Chars(Handle)); }
                }

                /// <summary>Gets a pointer (char*) to the internal character buffer.</summary>
                property char* CharPtr
                {
                    char* get() { return kString_Chars(Handle); }
                }

                /// <summary>Gets the number of character units in the string buffer (excluding null-terminator).</summary>
                /// 
                /// <remarks>kString assumes UTF8-encoded data; the string length refers to the number of encoded
                /// character units (bytes), rather than the number of characters.</remarks>
                property k64s Length
                {
                    k64s get() { return (k64s)kString_Length(Handle); }
                }

                /// <summary>Get the number of character units that can be stored without reallocation.</summary>
                property k64s Capacity
                {
                    k64s get() { return kString_Capacity(Handle); }
                }

                static explicit operator kChar*(KString^ str)
                {
                    return (str == nullptr) ? kNULL : kString_Chars(str->Handle); 
                }
               
                virtual String^ ToString() override
                {
                    return KToString(kString_Chars(Handle)); 
                }

                virtual bool Equals(Object^ other) override
                {
                    KString^ kstr = dynamic_cast<KString^>(other);
                    String^ nstr = dynamic_cast<String^>(other);

                    if (kstr != nullptr)
                    {
                        return KToBool(kObject_Equals(Handle, KToHandle(kstr)));
                    }
                    else if (nstr != nullptr)
                    {
                        return this->ToString()->Equals(other); 
                    }
                    else
                    {
                        return kFALSE; 
                    }
                }           
            };
        }
    }
}

#endif
