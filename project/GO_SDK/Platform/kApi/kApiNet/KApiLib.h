// 
// KApiLib.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_LIB_H
#define K_API_NET_LIB_H

#include <kApi/kApiLib.h>
#include "kApiNet/KAssembly.h"
#include "kApiNet/KAssemblyManager.h"

namespace Lmi3d
{
    namespace Zen
    {

        kStatus kCall KApiLib_OnLog(const kChar* format, va_list args); 
        kStatus kCall KApiLib_OnAssert(const kChar* file, k32u line);

        kStatus kCall KApiLib_OnAlloc(kPointer provider, kSize size, void* mem, kMemoryAlignment alignment);
        kStatus kCall KApiLib_OnFree(kPointer provider, void* mem); 

        /// <summary>Represents a KApiLib log message handler.</summary>
        public delegate void KLogHandler(String^ message);
         
        /// <summary>Singleton assembly manager for the Zen API native type assembly (kApiLib).</summary>
        ///
        /// <remarks>
        /// <para>The KApiLib class manages access to the underying Zen API native type assembly. A KApiLib instance 
        /// must be constructed in order to use the underlying native library, and should be destroyed when no 
        /// longer needed. </para>
        ///
        /// <para>The KApiLib class also provides static methods and properties to support logging and leak detection. 
        /// Refer to the example below.</para>
        /// </remarks>
        ///
        /// <example>This example illustrates use of the MessageLogged static event and the LeaksDetected static property
        /// to detect and report memory leaks.
        /// <code language="C#" source="examples/KApiLib_Logging.cs" />
        /// </example>
        public ref class KApiLib : public KAssemblyManager
        {
        public:

            /// <summary>Constructs the KApiLib assembly manager.</summary>
            ///
            /// <remarks>
            /// <para>If an instance of the KApiLib assembly manager does not currently exist, the Construct method 
            /// will create a new instance and return it. If an instance of the KApiLib assembly manager already exists, 
            /// the Construct method will increment its reference count and return the existing instance.</para>
            ///
            /// <para>The KApiLib instance returned by this method should be disposed when no longer needed.</para>
            /// 
            /// <para>This method is thread safe.</para>
            /// </remarks>
            static KApiLib^ Construct()
            {
                Monitor::Enter(m_lock);

                try
                {
                    //create new ref-counted instance only if necessary
                    if (m_instance == nullptr)
                    {
                        m_instance = gcnew KApiLib();
                    }
                    else
                    {
                        m_instance->m_refCount++;
                    }

                    return m_instance;
                }
                finally
                {
                    Monitor::Exit(m_lock);
                }
            }

            /// <summary>Disposes the KApiLib assembly manager.</summary>
            ///
            /// <remarks>
            /// <para>This method decrements the KApiLib reference count. If the reference count reaches zero, the 
            /// KApiLib instance will be disposed.</para>
            /// 
            /// <para>This method is thread safe.</para>
            /// </remarks>
            ~KApiLib()
            {

                Monitor::Enter(m_lock);

                try
                {
                    m_refCount--;

                    if (m_refCount == 0)
                    {
                        //unregister .NET types in this assembly
                        KAssemblyManager::Unregister();

                        //destroy native assembly
                        kObject_Destroy(m_assembly);

                        m_assembly = kNULL;
                        m_instance = nullptr;
                    }
                }
                finally
                {
                    Monitor::Exit(m_lock);
                }
            }

            /// <summary>Occurs when the Zen library emits a log message.</summary>
            ///
            /// <remarks>
            /// <para>This event can be used to capture diagnostic messages such as memory leak information.</para>
            /// 
            /// <para>To ensure that all messages are captured, an event handler should normally be added prior to constructing 
            /// the KApiLib instance, and removed after disposing the KApiLib instance.</para>
            /// </remarks>
            static event KLogHandler^ MessageLogged;

            /// <summary>Occurs when the Zen library triggers an assert.</summary>
            ///
            /// <remarks>
            /// <para>This event can be used to capture assert information.</para>
            /// 
            /// <para>To ensure that all messages are captured, an event handler should normally be added prior to constructing 
            /// the KApiLib instance, and removed after disposing the KApiLib instance.</para>
            /// </remarks>
            static event KLogHandler^ OnAssert;

            /// <summary>Reports memory leaks detected after disposing KApiLib.</summary>
            ///
            /// <remarks>Memory leaks can only be detected in debug builds.</remarks>
            static property k64s LeaksDetected 
            { 
                k64s get()
                {
                    return (k64s)kApiLib_LeaksDetected();
                }
            }

            /// <summary>Gets a reference to the KApiLib singleton instance.</summary>
            static property KApiLib^ Instance { KApiLib^ get() { return m_instance; } }

        internal:

            //header size used for memory allocation; enables the allocation size to be written 
            //into a header at the beginning of the allocation, without violating kApi's requirement 
            //that allocated memory must be aligned to kALIGN_ANY_SIZE
            static kSize MemHeaderSize = sizeof(kSize) * 2;;

            delegate kStatus KAllocFx(kPointer provider, kSize size, void* mem, kMemoryAlignment alignment);
            delegate kStatus KFreeFx(kPointer provider, void* mem);

            KApiLib()
            {
                kAssembly handle = kNULL;

                Monitor::Enter(m_lock);

                try
                {
                    KCheck(kApiLib_SetLogfHandler(KApiLib_OnLog));
                    KCheck(kApiLib_SetAssertHandler(KApiLib_OnAssert));

                    // Thunk the alloc and free callbacks to avoid app-domain errors
                    KAllocFx^ allocThunk = gcnew KAllocFx(&Lmi3d::Zen::KApiLib_OnAlloc);
                    m_AllocContext = gcnew KCallbackState(allocThunk, nullptr);

                    KFreeFx^ freeThunk = gcnew KFreeFx(&Lmi3d::Zen::KApiLib_OnFree);
                    m_FreeContext = gcnew KCallbackState(freeThunk, nullptr);

                    KCheck(kApiLib_SetMemAllocHandlers(
                        (kApiMemAllocFx)m_AllocContext->NativeFunction, 
                        (kApiMemFreeFx)m_FreeContext->NativeFunction, 
                        kNULL));

                    //construct native assembly
                    KCheck(kApiLib_Construct(&handle));

                    //register .NET types in this assembly
                    KAssemblyManager::Register();

                    m_assembly = handle;
                    m_refCount = 1;
                }
                catch (...)
                {
                    kObject_Destroy(handle);
                    throw;
                }
                finally
                {
                    Monitor::Exit(m_lock);
                }
            }

            static void OnMessageLogged(String^ arg)
            {
                MessageLogged(arg);
            }

            static void OnAssertHandler(String^ arg)
            {
                OnAssert(arg);
            }

            ///<summary>Lock object to provide thread-safe access.</summary>
            static Object^ m_lock = gcnew Object();

            ///<summary>Singleton instance.</summary>
            static KApiLib^ m_instance;

            static KCallbackState^ m_AllocContext;
            static KCallbackState^ m_FreeContext;
        };
    }
}

#endif
