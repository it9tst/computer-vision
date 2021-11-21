// 
// KInternalUtils.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_INTERNAL_UTILS_H
#define K_API_NET_INTERNAL_UTILS_H

#include <kApi/kApiDef.h>
#include "kApiNet/KApiNet.h"

namespace Lmi3d
{
    namespace Zen
    {
        /// <summary>Represents how multiple references of an object are managed.</summary>
        public enum class KRefStyle
        {
            /// <summary>The user must manually manage multiple references and object disposal.</summary>
            Manual,
            /// <summary>Multiple references and disposal are automatically handled.</summary>
            Auto,
            /// <summary>The object should not be shared or disposed. It is managed by a 'parent' object.</summary>
            None
        };

        namespace Utils
        {
            namespace Internal 
            {
                /// <summary>A collection of static utility methods used in the implementation of Zen.NET types.</summary>
                public ref class KInternalUtils abstract sealed
                {
                public:         
                    static void Check(kStatus status);
                    static void CheckArgs(kBool predicate);
                    static void CheckState(kBool predicate);
                    static void CheckErr(kBool predicate, kStatus status);

                    static bool ToBool(k32s value); 

                    static void StringToText(System::String^ str, kChar* buffer, k32s length); 

                    static System::String^ TextToString(const kChar* buffer);

                    static kObject ToHandle(KObject^ object); 
                    static kObject ToHandle(Object^ object);

                    generic <typename T>
                    static T ToObject(kPointer handle); 

                    generic <typename T> 
                    static T ToObject(kPointer handle, Nullable<KRefStyle> refStyle);

                    generic <typename T>
                    static T ToObject(IntPtr handle)
                    {
                        return ToObject<T>(handle.ToPointer()); 
                    }

                    generic <typename T>
                    static T AsObject(kPointer handle);

                    generic <typename T>
                    static T AsObject(IntPtr handle)
                    {
                        return AsObject<T>(handle.ToPointer()); 
                    }

                    static Type^ FindType(IntPtr type);
                    static IntPtr FindType(Type^ type);

                    generic <typename T>
                    static void* ToKZenGeneric(T% arg, kType type, kObject* tempHandle);

                    generic <typename T>
                    static T ToNetGeneric(const void* arg, kType type);              

                    generic <typename T>
                    static T ToGcObject(void* gcHandle);

                    static array<kObject>^ ToHandleArray(array<KObject^>^ objects); 

                    static kSize SizeFrom64s(k64s value);

                    static k64s SizeTo64s(kSize value);

                    static KRefStyle DetermineRefStyle(Nullable<KRefStyle> refStyle, kObject handle);

                    static void KInternalUtils::AdjustRef(kObject object, kBool add, Nullable<KRefStyle> refStyle);
                };
            }
        }
    }
}

#endif
