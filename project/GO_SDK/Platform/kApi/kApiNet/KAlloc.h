//
// KAlloc.h
//
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef K_API_NET_ALLOC_H
#define K_API_NET_ALLOC_H

#include <kApi/kAlloc.h>
#include "kApiNet/KObject.h"
#include "kApiNet/KType.h"

namespace Lmi3d
{
    namespace Zen 
    {                
        /// <summary>Abstract base class for memory allocators.</summary>        
        public ref class KAlloc abstract : public KObject
        {
            KDeclareClass(KAlloc, kAlloc)

        public:
            /// <summary>Gets the allocator that should normally be used by applications to request memory.</summary> 
            static property KAlloc^ App
            {
                KAlloc^ get()
                {
                    return KToObject<KAlloc^>(kAlloc_App());
                }
            }

            /// <summary>Gets an allocator that can be used to allocate directly from main system memory.</summary> 
            static property KAlloc^ System
            {
                KAlloc^ get()
                {
                    return KToObject<KAlloc^>(kAlloc_System());
                }
            }

            /// <summary>Allocates a block of memory.</summary>
            ///
            /// <param name="size">Size of memory to be allocated.</param>
            /// <returns>Pointer to allocated memory.</returns>
            IntPtr Get(k64s size)
            {
                void* mem = kNULL;

                KCheck(kAlloc_Get(Handle, (kSize)size, &mem));

                return IntPtr(mem);
            }

            /// <summary> Allocates a block of memory and zero-initializes the block.</summary>
            ///
            /// <param name="size">Size of memory to be allocated.</param>
            /// <returns>Pointer to allocated memory.</returns>
            IntPtr GetZero(k64s size)
            {
                void* mem = kNULL;

                KCheck(kAlloc_GetZero(Handle, (kSize)size, &mem));

                return IntPtr(mem);
            }

            /// <summary>Frees a block of memory.</summary>
            /// 
            /// <param name="mem">Pointer to memory to be freed.</param>
            void Free(IntPtr mem)
            {
                KCheck(kAlloc_Free(Handle, mem.ToPointer()));
            }

            /// <summary>Returns the passed allocator, or if null, the App allocator. </summary>
            ///
            /// <param name="alloc">Allocator.</param>
            /// <returns>Allocator.</returns>
            KAlloc^ Fallback(KAlloc^ alloc)
            {
                return (alloc != nullptr) ? alloc : KAlloc::App; 
            }

        protected:
            /// <summary>Initializes a new instance of the KAlloc class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            KAlloc(IntPtr handle)
                : KObject(handle, DefaultRefStyle) {}

            /// <summary>Initializes a new instance of the KAlloc class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            /// <param name="refStyle">Reference managment style.</param>
            KAlloc(IntPtr handle, KRefStyle refStyle)
                : KObject(handle, refStyle) {}

            KAlloc(KRefStyle refStyle) : KObject(refStyle){}

            virtual void OnDisposing() override
            {
                // Not fool-proof, but can help prevent errors from members of the allocator that are scheduled for 
                // finalization. Disposing the allocator while objects that used it still exist can crash or cause serious memory issues.
                GC::Collect();
                GC::WaitForPendingFinalizers();
            }
        };
    }
}

#endif
