// 
// KDebugAlloc.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_DEBUG_ALLOC_H
#define K_API_NET_DEBUG_ALLOC_H

#include <kApi/Utils/kDebugAlloc.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KArrayList.h"
#include "kApiNet/Data/KString.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Utils
        {
            /// <summary>Allocation record used by KDebugAlloc.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kDebugAllocation))]
            public value struct KDebugAllocation
            {
                KDeclareStruct(KDebugAllocation, kDebugAllocation)

                /// <summary>Gets user data pointer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(m_data); }
                }

                /// <summary>Gets size of memory allocation.</summary>
                property k64s Size
                {
                    k64s get() { return (k64s)m_size; }
                }

                /// <summary>Gets allocation index.</summary>
                property k64s Index
                {
                    k64s get() { return (k64s)m_index; }
                }

                /// <summary>Gets backtrace output at point of allocation.</summary>
                property KArrayList^ Trace
                {
                    KArrayList^ get() { return (m_trace) ? gcnew KArrayList(IntPtr(m_trace), KRefStyle::Manual) : nullptr; }
                }

            private:
               
                [FieldOffset(offsetof(kDebugAllocation, data))]
                kByte* m_data;            
                               
                [FieldOffset(offsetof(kDebugAllocation, size))]
                kSize m_size;                
               
                [FieldOffset(offsetof(kDebugAllocation, index))]
                k64u m_index;                
                
                [FieldOffset(offsetof(kDebugAllocation, trace))]
                kArrayList m_trace;
            };

            /// <summary>Debug memory allocator that can track allocations and report leaks. <para/> Requires manual disposal.</summary>
            public ref class KDebugAlloc : public KAlloc
            {
                KDeclareClass(KDebugAlloc, kDebugAlloc)

            public:
                /// <summary>Initializes a new instance of the KDebugAlloc class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KDebugAlloc(IntPtr handle)
                    : KAlloc(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KDebugAlloc class.</summary> 
                /// 
                /// <param name="name">Descriptive name for this memory allocator.</param>
                KDebugAlloc(String^ name)
                    : KAlloc(DefaultRefStyle)
                {
                    KString strName(name);
                    kDebugAlloc handle = kNULL;

                    KCheck(kDebugAlloc_Construct(&handle, strName.CharPtr, kNULL, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KDebugAlloc(String^)" />
                /// <param name="allocator">Memory allocator to use for internal allocations.</param>
                KDebugAlloc(String^ name, KAlloc^ innerAlloc, KAlloc^ allocator)
                    : KAlloc(DefaultRefStyle)
                {
                    KString strName(name); 
                    kDebugAlloc handle = kNULL;

                    KCheck(kDebugAlloc_Construct(&handle, strName.CharPtr, KToHandle(innerAlloc), KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Clears all outstanding allocations (resets allocator). </summary>
                void Clear()
                {
                    KCheck(kDebugAlloc_Clear(Handle)); 
                }

                /// <summary>Gets the total amount of memory that has been allocated and not yet freed.</summary>
                property k64s Allocated
                {
                    k64s get() { return (k64s)kDebugAlloc_Allocated(Handle); }
                }

                /// <summary>Gets a memory checkpoint value, used in conjunction with allocation logging methods.</summary>
                property k64s Checkpoint
                {
                    k64s get() { return (k64s)kDebugAlloc_Checkpoint(Handle); }
                }

                /// <summary>Gets a list of all outstanding memory allocations performed after the given checkpoint.</summary>
                /// 
                /// <param name="since">Memory checkpoint (or zero for beginning).</param>
                /// <param name="allocator">Allocator for history list (or null for default).</param>
                KArrayList^ GetAllocations(k64s since, KAlloc^ allocator)
                {
                    kArrayList history = kNULL;

                    KCheck(kDebugAlloc_Allocations(Handle, (k64u)since, &history, KToHandle(allocator))); 

                    return gcnew KArrayList(IntPtr(history)); 
                }

                /// <summary>Logs all outstanding memory allocations performed after the given checkpoint (using KUtils.Log).</summary>
                /// 
                /// <param name="since">Memory checkpoint (or zero for beginning).</param>
                void LogAllocations(k64s since)
                {
                    KCheck(kDebugAlloc_LogAllocations(Handle, (k64u)since)); 
                }

                /// <summary>Makes note of outstanding allocations that appear to be objects from any currently-loaded assembly.</summary>
                /// 
                /// <remarks>
                /// This method records information about outstanding allocations that appear to be objects, for later use
                /// when logging leak information.
                /// </remarks>
                /// 
                /// <param name="since">Memory checkpoint (or zero for beginning).</param>
                void DetectLeakedObjects(k64s since)
                {
                    KCheck(kDebugAlloc_DetectLeakedObjects(Handle, (k64u)since)); 
                }

                /// <summary>Makes note of outstanding allocations that appear to be objects associated with the specified Zen assembly.</summary>
                /// 
                /// <remarks>
                /// This method records information about outstanding allocations that appear to be objects, for later use
                /// when logging leak information.
                /// </remarks>
                /// 
                /// <param name="since">Memory checkpoint (or zero for beginning).</param>
                /// <param name="assembly">Zen (native) assembly associated with objects.</param>
                void DetectLeakedAssemblyObjects(k64s since, KAssembly^ assembly)
                {
                    KCheck(kDebugAlloc_DetectLeakedAssemblyObjects(Handle, (k64u)since, KToHandle(assembly))); 
                }

            protected:
                KDebugAlloc(KRefStyle refStyle) : KAlloc(refStyle) {}
            };
        }
    }
}

#endif
