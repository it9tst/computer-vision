// 
// KAssembly.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_ASSEMBLY_H
#define K_API_NET_ASSEMBLY_H

#include <kApi/kAssembly.h>
#include "kApiNet/KObject.h"
#include "kApiNet/Data/KArrayList.h"

namespace Lmi3d
{
    namespace Zen 
    {     
        ref class KType;

        /// <summary>Represents a native Zen-based type library.</summary>
        ///
        /// <remarks>
        /// A KAssembly represents a collection of underlying native types, rather than a .NET assembly. 
        /// Typically, one KAssembly is defined per Zen-based library or application. The Enumerate method can 
        /// be used to get a list of all loaded Zen assemblies.
        /// </remarks>
        ///
        /// <example>This example prints a list of supplemental assemblies that have been loaded (i.e., assemblies other than KApiLib)
        /// <code language="C#" source="examples/KAssembly_Enumerate.cs" />
        /// </example>    
        public ref class KAssembly : public KObject
        {
        public:                  
            KDeclareClass(KAssembly, kAssembly)

            /// <summary>Initializes a new instance of the KAssembly class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            KAssembly(IntPtr handle)
                : KObject(handle, DefaultRefStyle)
            {}

            /// <summary>Gets a list of the currently-loaded assemblies.</summary>
            /// 
            /// <remarks>Each assembly returned by this method is a reference-counted instance that should be disposed
            /// when no longer needed. Use the Dispose method on the returned list to destroy both the list and the 
            /// and the assembly objects. </remarks>
            /// 
            /// <returns>List of loaded assemblies.</returns>
            static KArrayList^ Enumerate()
            {
                KArrayList^ list = gcnew KArrayList(KAssembly::KTypeId, 0, nullptr);
                kStatus status;

                if (!kSuccess(status = kAssembly_Enumerate(KToHandle(list))))
                {
                    delete list;
                    KCheck(status);
                }

                return list;
            }

            /// <summary>Gets the assembly version.</summary>
            property KVersion Version 
            { 
                KVersion get() { return kAssembly_Version(Handle); } 
            }

            /// <summary>Gets the platform version.</summary>
            property KVersion PlatformVersion 
            { 
                KVersion get() { return kAssembly_PlatformVersion(Handle); } 
            }

            /// <summary>Gets the assembly name.</summary>
            property String^ Name 
            { 
                String^ get() { return KToString(kAssembly_Name(Handle)); } 
            }

            /// <summary>Gets the number of types in an assembly.</summary>
            property k32s TypeCount 
            { 
                k32s get() { return (k32s) kAssembly_TypeCount(Handle); } 
            }

            /// <summary>Gets the type at a particular index within an assembly.</summary>
            /// <param name="index">Index of type.</param>
            /// <returns>Type object.</returns>
            KType^ TypeAt(k32s index) 
            { 
                return gcnew KType(kAssembly_TypeAt(Handle, (kSize)index)); 
            }

            /// <summary>Finds a type by name.</summary>
            /// 
            /// <param name="name">Type name.</param>
            /// <returns>Type object.</returns>
            /// <exception cref="KException">Thrown when type not found.</exception>
            KType^ FindType(String^ name)
            {
                KText64 textName = name; 
                kType type = kNULL; 

                KCheck(kAssembly_FindType(Handle, (kChar*)&textName, &type)); 

                return gcnew KType(type); 
            }

            /// <summary>Gets the count of assembly dependencies.</summary>
            property k32s DependencyCount
            {
                k32s get() { return (k32s)kAssembly_DependencyCount(Handle); }
            }

            /// <summary>Gets the assembly dependency at the specified index.</summary>
            /// <param name="index">Index of assembly dependency.</param>
            /// <returns>Assembly dependency.</returns>
            KAssembly^ DependencyAt(k32s index)
            {
                return gcnew KAssembly(IntPtr(kAssembly_DependencyAt(Handle, (kSize)index))); 
            }


        protected:
            //KAssembly() {}
        };
    }
}

#endif
