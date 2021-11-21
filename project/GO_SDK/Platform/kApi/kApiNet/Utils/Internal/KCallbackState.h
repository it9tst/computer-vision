// 
// KAssemblyManager.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_CALLBACK_STATE_H
#define K_API_NET_CALLBACK_STATE_H

#include <kApi/kApiDef.h>
#include "kApiNet/KApiNet.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Utils
        {
            namespace Internal 
            {
                /// <summary>Represents context for a managed callback that can be invoked from native code.</summary>
                ///            
                /// <remarks>
                /// <para>The Marshal.GetFunctionPointerForDelegate technique implemented in this class is required to support
                /// unmanaged callbacks in the presence of multiple app domains. See notes at: 
                ///  http://lambert.geek.nz/2007/05/unmanaged-appdomain-callback/ </para>
                ///
                /// <para> Excercise caution when using this class. In particular, the signature of the managed delegate 
                /// must exactly match the sigature expected for the native callback. This constraint is not enforced via 
                /// static type checking.</para>
                ///
                /// <para>Take care to ensure that each instance of this class is disposed when no longer needed. Otherwise, CLR
                /// object leaks can occur.</para>
                /// 
                /// </remarks>
                public ref class KCallbackState
                {
                public:
                    KCallbackState(Delegate^ thunk, Delegate^ handler)
                        : Thunk(thunk), Handler(handler)
                    {
                        SelfHandle = GCHandle::Alloc(this);
                        NativeContext = ((IntPtr)SelfHandle).ToPointer();
                        NativeFunction = (kFunction)Marshal::GetFunctionPointerForDelegate(Thunk).ToPointer();
                    }

                    ~KCallbackState()
                    {
                        if (!kIsNull(NativeContext))
                        {
                            SelfHandle.Free();

                            NativeContext = kNULL;
                            NativeFunction = kNULL;
                            Thunk = Handler = nullptr;
                        }
                    }

                    /// <summary>Gets the KCallbackState object associated with the specified native context pointer.</summary>
                    static KCallbackState^ FromNativeContext(kPointer nativeContext)
                    {
                        return KToGcObject<KCallbackState^>(nativeContext);
                    }

                    /// <summary>Frees the KCallbackState object associated with the specified native context pointer.</summary>
                    static void Dispose(kPointer nativeContext)
                    {
                        if (!kIsNull(nativeContext))
                        {
                            KCallbackState^ context = KToGcObject<KCallbackState^>(nativeContext);
                            delete context;
                        }
                    }

                    /// <summary>Delegate representing managed method that is callable by native code.</summary>
                    Delegate^ Thunk;

                    /// <summary>User event handler, typically invoked by thunk. </summary>
                    Delegate^ Handler;

                    ///<summary>Handle to pinning pointer to this object.</summary>
                    GCHandle SelfHandle;

                    ///<summary>Function pointer that can be passed to native code; will invoke thunk.</summary>
                    kPointer NativeFunction;

                    ///<summary>Context pointer that can be passed to native code; pointer representation of GcHandle.</summary>
                    kPointer NativeContext;
                };
            }
        }
    }
}

#endif
