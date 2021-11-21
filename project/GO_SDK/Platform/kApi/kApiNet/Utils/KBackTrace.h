// 
// KBackTrace.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_BACK_TRACE_H
#define K_API_NET_BACK_TRACE_H

#include <kApi/Utils/kBackTrace.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KArrayList.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {
            /// <summary>Represents a snapshot of the active functions on a call stack.</summary>
            /// 
            /// <remarks>
            /// <para>The KBackTrace class does not currently support managed (CLR) callstacks.</para>
            ///
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KBackTrace : public KObject
            {
                KDeclareAutoClass(KBackTrace, kBackTrace)

            public:
                /// <summary>Initializes a new instance of the KBackTrace class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KBackTrace(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KBackTrace(IntPtr)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KBackTrace(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KBackTrace class.</summary>   
                ///
                /// <remarks>
                /// The constructed back trace object will be initially empty; use the Capture method to acquire
                /// trace information.
                /// </remarks>
                /// 
                /// <param name="allocator">Memory allocator (or null for default).</param>
                KBackTrace(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kBackTrace handle = kNULL;

                    KCheck(kBackTrace_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KBackTrace(KAlloc^)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KBackTrace(KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kBackTrace handle = kNULL;

                    KCheck(kBackTrace_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Captures back trace information.</summary>
                /// 
                /// <remarks>
                /// If supported by the underlying platform, the set of active functions on the call stack
                /// will be captured. For platforms that do not support back trace, the capture operation will
                /// succeed but the resulting trace will be empty.
                /// </remarks>
                /// 
                /// <param name="skip">Count of recent functions to omit from trace.</param>
                void Capture(k64s skip)
                {
                    KCheck(kBackTrace_Capture(Handle, (kSize)skip)); 
                }

                /// <summary>Count of function calls in the captured trace.</summary>
                /// 
                /// <returns>Count of function calls in trace.</returns>
                property k64s Depth
                {
                    k64s get() { return (k64s)kBackTrace_Depth(Handle); }
                }

                /// <summary>Creates a list of descriptive strings, one for each line in the trace.</summary>
                /// 
                /// <remarks>The list returned by this function should be freed using KObject.Dispose</remarks>
                ///
                /// <param name="allocator">Memory allocator for trace list (or null for default).</param>
                /// <returns>List of descriptive strings.</returns>
                KArrayList^ Describe(KAlloc^ allocator)
                {
                    kArrayList list = kNULL; 

                    KCheck(kBackTrace_Describe(Handle, &list, KToHandle(allocator))); 

                    return KToObject<KArrayList^>(list);
                }
              
            protected:
                KBackTrace() : KObject(DefaultRefStyle){}
            };
        }
    }
}

#endif
