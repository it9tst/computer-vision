// 
// KAssemblyManager.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ASSEMBLY_MANAGER_H
#define K_API_NET_ASSEMBLY_MANAGER_H

#include <kApi/Utils/kPlugin.h>
#include "kApiNet/KAssembly.h"

using namespace System::Threading;
using namespace System::Collections::Generic;

namespace Lmi3d
{
    namespace Zen
    {
        /// <summary>Abstract base class for assembly managers.</summary>
        public ref class KAssemblyManager abstract
        {
        public:

            /// <summary>Gets zen assembly object.</summary>
            property KAssembly^ Assembly
            {
                KAssembly^ get()
                {
                    return gcnew KAssembly(IntPtr(m_assembly));
                }
            }

        protected:

            /// <summary>Registers .NET types from the calling assembly.</summary>
            void Register()
            {
                AddAssembly(System::Reflection::Assembly::GetCallingAssembly());
            }

            /// <summary>Unregisters .NET types from the calling assembly.</summary>
            void Unregister()
            {
                GC::Collect(); 
                GC::WaitForPendingFinalizers(); 

                RemoveAssembly(System::Reflection::Assembly::GetCallingAssembly());
            }

            /// <summary>Zen assembly handle. </summary>
            kAssembly m_assembly;

            /// <summary>Reference count for this instance. </summary>
            int m_refCount;

        internal:

            /// <summary>Maps from native Zen type (kType) to .NET type (Type).</summary>
            static System::Collections::Generic::Dictionary<IntPtr, System::Type^>^ toType;

            /// <summary>Maps from .NET type (Type) to native Zen type (kType).</summary>
            static System::Collections::Generic::Dictionary<System::Type^, IntPtr>^ toKType;

        private:

            static KAssemblyManager()
            {
                toType = gcnew Dictionary<IntPtr, Type^>();
                toKType = gcnew Dictionary<Type^, IntPtr>();
            }

            static void AddAssembly(System::Reflection::Assembly^ assembly)
            {
                array<Type^>^ types = assembly->GetTypes();

                for each(Type^ t in types)
                {
                    PropertyInfo^ prop = t->GetProperty("KTypeId");

                    if (prop)
                    {
                        MethodInfo^ method = prop->GetGetMethod();

                        if (method)
                        {
                            KType^ ktype = dynamic_cast<KType^>(method->Invoke(nullptr, nullptr));

                            if (ktype)
                            {
                                toType->Add(ktype->ToHandle(), t);
                                toKType->Add(t, ktype->ToHandle());
                            }
                        }
                    }
                }
            }

            static void RemoveAssembly(System::Reflection::Assembly^ assembly)
            {
                array<Type^>^ types = assembly->GetTypes();

                for each(Type^ t in types)
                {
                    PropertyInfo^ prop = t->GetProperty("KTypeId");

                    if (prop)
                    {
                        MethodInfo^ method = prop->GetGetMethod();

                        if (method)
                        {
                            KType^ ktype = dynamic_cast<KType^>(method->Invoke(nullptr, nullptr));

                            if (ktype)
                            {
                                toType->Remove(ktype->ToHandle());
                                toKType->Remove(t);
                            }
                        }
                    }
                }
            }
        };

    }
}

#endif
