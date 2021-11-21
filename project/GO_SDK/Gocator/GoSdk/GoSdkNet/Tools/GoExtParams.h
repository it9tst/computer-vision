//
// GoExtParams.h
//
// This class should not be confused with a similarly named GoExtParm.h class.
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_EXT_PARAMS_H
#define GO_SDK_NET_EXT_PARAMS_H

#include <GoSdk/Tools/GoExtParams.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/Tools/GoExtParam.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            public ref class GoExtParams abstract : public KObject
            {
                KDeclareClass(GoExtParams, GoExtParams)

            public:
                /// <summary>Default GoExtParams constructor.</summary>
                GoExtParams() {}

                /// <summary>Initializes a new instance of the GoExtParams class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExtParams(IntPtr handle)
                    : KObject(handle)
                {}

                property kSize Count
                {
                    kSize get()
                    {
                        return (kSize) ::GoExtParams_ParameterCount(Handle);
                    }
                }

                GoExtParam^ Value(kSize index)
                {
                    return KToObject<GoExtParam^>(::GoExtParams_ParameterAt(Handle, (kSize) index));
                }

                GoExtParam^ FindById(String^ id)
                {
                    KString str(id);

                    return KToObject<GoExtParam^>(::GoExtParams_FindParameterById(Handle, (const kChar*) str.CharPtr));
                }
            };
        }
    }
}

#endif
