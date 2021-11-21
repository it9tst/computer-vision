// 
// KApiNet.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_H
#define K_API_NET_H

using namespace System;
using namespace System::ComponentModel;
using namespace System::Reflection;
using namespace System::Runtime::InteropServices;


namespace Lmi3d
{
    namespace Zen
    {
        /*
        * Forward declarations.
        */
        ref class KObject;
        ref class KAlloc;
        ref class KType;
        ref class KAssembly;
        ref class KException;

        namespace Crypto {}
        namespace Data {}
        namespace Io {}
        namespace Threads {}
        namespace Utils
        {
            ref class KObjectPool;
            namespace Internal {}
        }
    }
}

using namespace Lmi3d;
using namespace Lmi3d::Zen;
using namespace Lmi3d::Zen::Crypto;
using namespace Lmi3d::Zen::Data;
using namespace Lmi3d::Zen::Io;
using namespace Lmi3d::Zen::Threads;
using namespace Lmi3d::Zen::Utils;
using namespace Lmi3d::Zen::Utils::Internal;

/*
* Zen conversion/utilty macros
*/

#define KCheck          KInternalUtils::Check
#define KCheckArgs      KInternalUtils::CheckArgs
#define KCheckState     KInternalUtils::CheckState
#define KCheckErr       KInternalUtils::CheckErr
#define KToBool         KInternalUtils::ToBool
#define KToText         KInternalUtils::StringToText
#define KToString       KInternalUtils::TextToString
#define KToObject       KInternalUtils::ToObject
#define KAsObject       KInternalUtils::AsObject
#define KToHandle       KInternalUtils::ToHandle
#define KToKZenGeneric  KInternalUtils::ToKZenGeneric
#define KToNetGeneric   KInternalUtils::ToNetGeneric
#define KToGcObject     KInternalUtils::ToGcObject
#define KSizeTo64s      KInternalUtils::SizeTo64s
#define K64sToSize      KInternalUtils::SizeFrom64s
#define KAdjustRef      KInternalUtils::AdjustRef

/*
* Zen type definition helper macros.
*/


/*
* Enum wrapper declaration helper.
*/
#define KDeclareEnum(CLR_T, C_T)                                            \
                                                                            \
public:                                                                     \
                                                                            \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
property k32s Value;                                                        \
                                                                            \
CLR_T(k32s v)                                                               \
{                                                                           \
    Value = v;                                                              \
}                                                                           \
                                                                            \
static operator CLR_T(k32s v)                                               \
{                                                                           \
    return CLR_T(v);                                                        \
}                                                                           \
                                                                            \
static operator C_T(CLR_T v)                                                \
{                                                                           \
    return C_T(v.Value);                                                    \
}                                                                           \
                                                                            \
virtual String^ ToString() override;                                        \
                                                                            \
virtual int GetHashCode() override                                          \
{                                                                           \
    return Value;                                                           \
}                                                                           \
                                                                            \
virtual bool Equals(Object^ obj) override                                   \
{                                                                           \
    CLR_T^ enumRef = dynamic_cast<CLR_T^>(obj);                             \
                                                                            \
    if (enumRef != nullptr)                                                 \
    {                                                                       \
        return Equals((CLR_T)enumRef);                                      \
    }                                                                       \
                                                                            \
    int^ intRef = dynamic_cast<int^>(obj);                                  \
                                                                            \
    if (intRef != nullptr)                                                  \
    {                                                                       \
        return Equals((int)intRef);                                         \
    }                                                                       \
                                                                            \
    return false;                                                           \
}                                                                           \
                                                                            \
bool Equals(CLR_T other)                                                    \
{                                                                           \
    return Value == other.Value;                                            \
}                                                                           \
                                                                            \
bool Equals(int other)                                                      \
{                                                                           \
    return Value == other;                                                  \
}                                                                           \
                                                                            \
static bool operator ==(CLR_T a, CLR_T b)                                   \
{                                                                           \
    return a.Equals(b);                                                     \
}                                                                           \
                                                                            \
static bool operator ==(CLR_T a, int b)                                     \
{                                                                           \
    return a.Equals(b);                                                     \
}                                                                           \
                                                                            \
static bool operator !=(CLR_T a, CLR_T b)                                   \
{                                                                           \
    return !a.Equals(b);                                                    \
}                                                                           \
                                                                            \
static bool operator !=(CLR_T a, int b)                                     \
{                                                                           \
    return !a.Equals(b);                                                    \
}


/*
* Enum wrapper definition helper.
*/
#define KDefineEnum(CLR_T, C_T)                                             \
                                                                            \
KType^ CLR_T::KTypeId::get()                                                \
{                                                                           \
    return gcnew Lmi3d::Zen::KType(IntPtr(kTypeOf(C_T)));                   \
}                                                                           \
                                                                            \
String^ CLR_T::ToString()                                                   \
{                                                                           \
    kText128 text;                                                          \
    kType_FormatEnumerator(kTypeOf(C_T), Value, text, kCountOf(text));      \
    return KToString(text);                                                 \
}


/*
* Value wrapper declaration helper (used to help build Struct, Text, 
* and Primative helpers).
*/
#define KDeclareValue(CLR_T, C_T)                                           \
                                                                            \
private:                                                                    \
                                                                            \
static CLR_T()                                                              \
{                                                                           \
    if (sizeof(CLR_T) != sizeof(::C_T))                                     \
        {                                                                   \
        throw gcnew System::InvalidOperationException(                      \
            System::String::Format(                                         \
                "{0} size does not match underlying native structure size", \
                #CLR_T));                                                   \
        }                                                                   \
}                                                                           \
                                                                            \
public:                                                                     \
                                                                            \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
virtual k32s GetHashCode() override                                         \
{                                                                           \
    CLR_T self = *this;                                                     \
    return (k32s) kValue_HashCode(kTypeOf(C_T), &self);                     \
}                                                                           \
                                                                            \
virtual bool Equals(Object^ obj) override                                   \
{                                                                           \
    if (dynamic_cast<CLR_T^>(obj) == nullptr)                               \
        return false;                                                       \
                                                                            \
    return Equals(safe_cast<CLR_T>(obj));                                   \
}                                                                           \
                                                                            \
bool Equals(CLR_T other)                                                    \
{                                                                           \
    CLR_T self = *this;                                                     \
    return KToBool(kValue_Equals(kTypeOf(C_T), &self, &other));             \
}                                                                           \
                                                                            \
static bool operator ==(CLR_T a, CLR_T b)                                   \
{                                                                           \
    return a.Equals(b);                                                     \
}                                                                           \
                                                                            \
static bool operator !=(CLR_T a, CLR_T b)                                   \
{                                                                           \
    return !a.Equals(b);                                                    \
}   


/*
* Value wrapper definition helper (used to help build Struct, Text, 
* and Primative helpers).
*/
#define KDefineValue(CLR_T, C_T)                                            \
                                                                            \
KType^ CLR_T::KTypeId::get()                                                \
{                                                                           \
    return gcnew Lmi3d::Zen::KType(IntPtr(kTypeOf(C_T)));                   \
}

/*
 * Struct wrapper declaration helper.
 */

#define KDeclareStruct(CLR_T, C_T)                                          \
                                                                            \
    KDeclareValue(CLR_T, C_T)                                               \
                                                                            \
internal:                                                                   \
                                                                            \
[EditorBrowsable(EditorBrowsableState::Never)]                              \
CLR_T(const ::C_T* native)                                                  \
{                                                                           \
    pin_ptr<CLR_T> addr = this;                                             \
                                                                            \
    kItemCopy(addr, native, sizeof(CLR_T));                                \
}                                                                           \
                                                                            \
[EditorBrowsable(EditorBrowsableState::Never)]                              \
::C_T ToNative()                                                            \
{                                                                           \
    pin_ptr<CLR_T> addr = this;                                             \
                                                                            \
    return kPointer_ReadAs(addr, ::C_T);                                               \
}                                                                           \
                                                                            \
[EditorBrowsable(EditorBrowsableState::Never)]                              \
static explicit operator CLR_T(const ::C_T& native)                         \
{                                                                           \
    return kPointer_ReadAs(&native, CLR_T);                                            \
}                                                                           \
                                                                            \
[EditorBrowsable(EditorBrowsableState::Never)]                              \
static explicit operator ::C_T(CLR_T clr)                                   \
{                                                                           \
    return kPointer_ReadAs(&clr, ::C_T);                                               \
}                                                                           \
                                                                            \
public:

/*
 * Struct wrapper definition helper.
 */

#define KDefineStruct(CLR_T, C_T)                                           \
                                                                            \
    KDefineValue(CLR_T, C_T)


/*
* Primative value wrapper declaration helper.
*/
#define KDeclarePrimative(CLR_T, C_T)                                       \
                                                                            \
KDeclareValue(CLR_T, C_T)                                                   \
                                                                            \
CLR_T(::C_T v)                                                              \
{                                                                           \
    Value = v;                                                              \
}                                                                           \
                                                                            \
::C_T Value;                                                                \
                                                                            \
static operator CLR_T(::C_T v)                                              \
{                                                                           \
    return CLR_T(v);                                                        \
}                                                                           \
                                                                            \
static operator ::C_T(CLR_T v)                                              \
{                                                                           \
    return ::C_T(v.Value);                                                  \
}                                                                           \
                                                                            \
virtual String^ ToString() override                                         \
{                                                                           \
    return Value.ToString();                                                \
}


/*
* Primative value wrapper definition helper.
*/
#define KDefinePrimative(CLR_T, C_T)                                        \
                                                                            \
KDefineValue(CLR_T, C_T)

/*
* Text value wrapper declaration helper.
*/
#define KDeclareText(CLR_T, C_T)                                            \
                                                                            \
KDeclareValue(CLR_T, C_T)                                                   \
                                                                            \
CLR_T(String^ s)                                                            \
{                                                                           \
    pin_ptr<kChar> c = &m_bytes;                                            \
                                                                            \
    KToText(s, c, (int)sizeof(CLR_T));                                      \
}                                                                           \
                                                                            \
static operator String^ (CLR_T t)                                           \
{                                                                           \
    pin_ptr<kChar> c = &t.m_bytes;                                          \
                                                                            \
    return KToString(c);                                                    \
}                                                                           \
                                                                            \
static operator CLR_T(String^ s)                                            \
{                                                                           \
    CLR_T text;                                                             \
                                                                            \
    KToText(s, &text.m_bytes, (int)sizeof(text));                           \
                                                                            \
    return text;                                                            \
}                                                                           \
                                                                            \
virtual String^ ToString() override                                         \
{                                                                           \
    pin_ptr<kChar> c = &m_bytes;                                            \
                                                                            \
    return KToString(c);                                                    \
}                                                                           \
                                                                            \
private:                                                                    \
    [FieldOffset(0)]                                                        \
    kChar m_bytes;


/*
* Text value wrapper definition helper.
*/
#define KDefineText(CLR_T, C_T)                                             \
                                                                            \
KDefineValue(CLR_T, C_T)



/*
* Declaration helper for KVoid
*/
#define KDeclareVoid(CLR_T, C_T)                                            \
                                                                            \
public:                                                                     \
                                                                            \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
virtual bool Equals(Object^ obj) override                                   \
{                                                                           \
    return false;                                                           \
}                                                                           \
                                                                            \
bool Equals(CLR_T other)                                                    \
{                                                                           \
    return false;                                                           \
}                                                                       


/*
* Definition helper for KVoid.
*/
#define KDefineVoid(CLR_T, C_T)                                             \
                                                                            \
KType^ CLR_T::KTypeId::get()                                                \
{                                                                           \
    return gcnew Lmi3d::Zen::KType(IntPtr(kTypeOf(C_T)));                   \
}

/*
* Class wrapper declaration helper.
*/
#define KDeclareClass(CLR_T, C_T)                                           \
protected:                                                                  \
virtual property Lmi3d::Zen::KRefStyle InternalDefaultRefStyle              \
{                                                                           \
    virtual Lmi3d::Zen::KRefStyle get() override                            \
    {                                                                       \
        return DefaultRefStyle;                                             \
    }                                                                       \
}                                                                           \
                                                                            \
public:                                                                     \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
static property Lmi3d::Zen::KRefStyle DefaultRefStyle                       \
{                                                                           \
    Lmi3d::Zen::KRefStyle get()                                             \
    {                                                                       \
        return Lmi3d::Zen::KRefStyle::Manual;                               \
    }                                                                       \
}   

/*
* Class wrapper definition helper.
*/
#define KDefineClass(CLR_T, C_T)                                            \
                                                                            \
Lmi3d::Zen::KType^ CLR_T::KTypeId::get()                                    \
{                                                                           \
    return gcnew Lmi3d::Zen::KType(IntPtr(kTypeOf(C_T)));                   \
}

/*
* Automatic ref-counting class wrapper declaration helper.
*/
#define KDeclareAutoClass(CLR_T, C_T)                                       \
protected:                                                                  \
virtual property Lmi3d::Zen::KRefStyle InternalDefaultRefStyle              \
{                                                                           \
    virtual Lmi3d::Zen::KRefStyle get() override                            \
    {                                                                       \
        return DefaultRefStyle;                                             \
    }                                                                       \
}                                                                           \
                                                                            \
public:                                                                     \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
static property Lmi3d::Zen::KRefStyle DefaultRefStyle                       \
{                                                                           \
    Lmi3d::Zen::KRefStyle get()                                             \
    {                                                                       \
        return Lmi3d::Zen::KRefStyle::Auto;                                 \
    }                                                                       \
}    

/*
* Automatic ref-counting class wrapper declaration helper.
*/
#define KDeclareNoneClass(CLR_T, C_T)                                       \
protected:                                                                  \
virtual property Lmi3d::Zen::KRefStyle InternalDefaultRefStyle              \
{                                                                           \
    virtual Lmi3d::Zen::KRefStyle get() override                            \
    {                                                                       \
        return DefaultRefStyle;                                             \
    }                                                                       \
}                                                                           \
                                                                            \
public:                                                                     \
static property Lmi3d::Zen::KType^ KTypeId                                  \
{                                                                           \
    Lmi3d::Zen::KType^ get();                                               \
}                                                                           \
                                                                            \
static property Lmi3d::Zen::KRefStyle DefaultRefStyle                       \
{                                                                           \
    Lmi3d::Zen::KRefStyle get()                                             \
    {                                                                       \
        return Lmi3d::Zen::KRefStyle::None;                                 \
    }                                                                       \
}

/*
* Interface wrapper declaration helper.
*/
#define KDeclareInterface(CLR_T, C_T)                                       \
    KDeclareClass(CLR_T, C_T)

#define KDeclareAutoInterface(CLR_T, C_T)                                   \
    KDeclareAutoClass(CLR_T, C_T)

/*
* Interface wrapper definition helper.
*/
#define KDefineInterface(CLR_T, C_T)                                        \
    KDefineClass(CLR_T, C_T)

#endif
