// 
// KInternalUtils.cpp
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#include "kApiNet/KApiDef.h"
#include "kApiNet/KApiLib.h"
#include "kApiNet/KException.h"
#include "kApiNet/KObject.h"
#include "kApiNet/Utils/Internal/KInternalUtils.h"
#include "kApiNet/Utils/KObjectPool.h"
#include <vcclr.h>

void KInternalUtils::Check(kStatus status)
{
    if (status != kOK)
    {
        throw gcnew KException(status);
    }
}

void KInternalUtils::CheckArgs(kBool predicate)
{
    if (!predicate)
    {
        throw gcnew KException(kERROR_PARAMETER);
    }
}

void KInternalUtils::CheckState(kBool predicate)
{
    if (!predicate)
    {
        throw gcnew KException(kERROR_STATE);
    }
}

void KInternalUtils::CheckErr(kBool predicate, kStatus status)
{
    if (!predicate)
    {
        throw gcnew KException(status);
    }
}

bool KInternalUtils::ToBool(k32s value)
{
    return (value == kFALSE) ? false : true; 
}

kObject KInternalUtils::ToHandle(KObject^ object)
{
    return (object == nullptr) ? kNULL : object->ToHandle().ToPointer();
}

kObject KInternalUtils::ToHandle(Object^ object)
{
    return (object == nullptr) ? kNULL : safe_cast<KObject^>(object)->ToHandle().ToPointer();
}

void KInternalUtils::StringToText(System::String^ str, kChar* buffer, k32s length)
{
    pin_ptr<const wchar_t> pinned = PtrToStringChars(str);
    k32s bytesWritten = 0; 

    CheckArgs(length > 0); 

    bytesWritten = System::Text::Encoding::UTF8->GetBytes((wchar_t*)pinned, str->Length, (kByte*)buffer, length - 1);

    buffer[bytesWritten] = 0; 
}

System::String^ KInternalUtils::TextToString(const kChar* buffer)
{
    if (!kIsNull(buffer))
    {
        return gcnew System::String(buffer, 0, (int)kStrLength(buffer), System::Text::Encoding::UTF8);
    }
    else
    {
        return nullptr;
    }
}

Type^ KInternalUtils::FindType(IntPtr type)
{
    return KApiLib::toType[type];
}

IntPtr KInternalUtils::FindType(Type^ type)
{
    return KApiLib::toKType[type];
}

generic <typename T>
T KInternalUtils::ToObject(kPointer handle)
{
    if (kIsNull(handle))
    {
        return T(); 
    }
    else
    {
        kType typeHandle = kObject_Type(handle);

        while (!kIsNull(typeHandle))
        {
            Type^ type = KInternalUtils::FindType(IntPtr(typeHandle)); 

            if (type)
            {
                array<Object^>^ ctorArgs = gcnew array < Object^ > { IntPtr(handle) };

                return safe_cast<T> (type->InvokeMember(nullptr, BindingFlags::Public | BindingFlags::Instance | BindingFlags::CreateInstance, nullptr, nullptr, ctorArgs));
            }
                
            typeHandle = kType_Base(typeHandle);
        }

        throw gcnew KException(kERROR_NOT_FOUND); 
    }
}

generic <typename T> 
T KInternalUtils::ToObject(kPointer handle, Nullable<KRefStyle> refStyle)
{
    T object  = KInternalUtils::ToObject<T>(handle);

    if (refStyle.HasValue)
    {
        ((KObject^)object)->RefStyle = refStyle.Value;
    }

    return object; 
}

generic <typename T>
T KInternalUtils::AsObject(kPointer handle)
{
    if (!kIsNull(handle))
    {
        kType typeHandle = kObject_Type(handle);
        Type^ otherType = T::typeid; 
        kType otherTypeHandle = KInternalUtils::FindType(otherType).ToPointer();

        if (kType_Is(typeHandle, otherTypeHandle))
        {
            array<Object^>^ ctorArgs = gcnew array < Object^ > { IntPtr(handle) };

            KRefStyle style = KInternalUtils::DetermineRefStyle(Nullable<KRefStyle>(), handle);
            if (style == KRefStyle::Auto)
            {
                KCheck(kObject_Share(handle));
            }

            return safe_cast<T> (otherType->InvokeMember(nullptr, BindingFlags::Public | BindingFlags::Instance | BindingFlags::CreateInstance, nullptr, nullptr, ctorArgs));
        }
    }

    return T();
}

generic <typename T>
void* KInternalUtils::ToKZenGeneric(T% arg, kType type, kObject* tempHandle)
{
    if (kType_IsReference(type))
    {
        *tempHandle = KToHandle(arg);

        return tempHandle;
    }
    else
    {
        KCheckArgs(kType_Size(type) == sizeof(T));

        pin_ptr<T> ptr = &arg;

        return ptr;
    }
}

generic <typename T>
T KInternalUtils::ToNetGeneric(const void* arg, kType type)
{
    if (kType_IsReference(type))
    {
        kObject object = *(kObject*)arg;

        return KToObject<T>(object);
    }
    else
    {
        T value;

        KCheckArgs(sizeof(T) == kType_Size(type));

        kItemCopy(&value, arg, sizeof(T));

        return value;
    }
}

generic <typename T>
T KInternalUtils::ToGcObject(void* gcPointer)
{
    GCHandle h = (GCHandle)IntPtr(gcPointer);

    return (T) h.Target;
}

array<kObject>^ KInternalUtils::ToHandleArray(array<KObject^>^ objects)
{
    if (objects == nullptr)
    {
        return nullptr;
    }
    else
    {
        array<kObject>^ output = gcnew array<kObject>(objects->Length);

        for (int i = 0; i < objects->Length; ++i)
        {
            output[i] = KToHandle(objects[i]);
        }

        return output;
    }
}

kSize KInternalUtils::SizeFrom64s(k64s value)
{
    if (value == k64S_MAX)
    {
        return kSIZE_MAX;
    }
    else if (value > kSIZE_MAX)
    {
        throw gcnew KException(kERROR_PARAMETER);
    }
    else if (value < 0)
    {
        throw gcnew KException(kERROR_PARAMETER);
    }
    else
    {
        return (kSize)value;
    }
}

k64s KInternalUtils::SizeTo64s(kSize value)
{
    if (value == kSIZE_MAX)
    {
        return k64S_MAX;
    }
    else
    {
        return (k64s)value;
    }
}

KRefStyle KInternalUtils::DetermineRefStyle(Nullable<KRefStyle> refStyle, kObject handle)
{
    if (refStyle.HasValue)
    {
        return refStyle.Value; 
    }
    else if (!kIsNull(handle))
    {
        kType typeHandle = kObject_Type(handle);

        while (!kIsNull(typeHandle))
        {
            Type^ type = KInternalUtils::FindType(IntPtr(typeHandle));

            if (type)
            {
                return safe_cast<KRefStyle> (type->InvokeMember("DefaultRefStyle", BindingFlags::Public | BindingFlags::Static | BindingFlags::GetProperty, nullptr, nullptr, nullptr));
            }

            typeHandle = kType_Base(typeHandle);
        }
    }

    return KRefStyle::Manual;
}

void KInternalUtils::AdjustRef(kObject object, kBool add, Nullable<KRefStyle> refStyle)
{
    KRefStyle style = KInternalUtils::DetermineRefStyle(refStyle, object);

    if (style == KRefStyle::Auto)
    {
        if (add)
        {
            KCheck(kObject_Share(object));
        }
        else
        {
            KCheck(kObject_Dispose(object));
        }
    }
}