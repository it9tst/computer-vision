// 
// GoMaterial.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_MATERIAL_H
#define GO_SDK_NET_MATERIAL_H

#include <GoSdk/GoMaterial.h>
#include <GoSdk/GoAdvanced.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents configurable material acquisition settings.</summary>
        [Obsolete("GoMaterial has been renamed to GoAdvanced", false)]
        public ref class GoMaterial sealed : public GoAdvanced
        {
        public:

            /// <summary>Initializes a new instance of the GoAdvanced class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoMaterial(IntPtr handle)
                : GoAdvanced(handle)
            {}

            /// <summary>Initializes a new instance of the GoAdvanced class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoMaterial(GoSensor^ sensor)
                : GoAdvanced(sensor)
            {}

            /// <inheritdoc cref="GoAdvanced(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoMaterial(GoSensor^ sensor, KAlloc^ allocator)
                : GoAdvanced(sensor, allocator)
            {}

            /// <summary>The material acquisition type.</summary>
            property GoMaterialType MaterialType
            {
                GoMaterialType get()           { return (GoMaterialType) ::GoAdvanced_Type(Handle); }
                void set(GoMaterialType type)  { KCheck(::GoAdvanced_SetType(Handle, type)); }
            }

            /// <summary>The material acquisition type to be used by the system.</summary>
            property GoMaterialType MaterialTypeSystemValue
            {
                GoMaterialType get()           { return (GoMaterialType) ::GoAdvanced_TypeSystemValue(Handle); }
            }
        };
    }
}

#endif
