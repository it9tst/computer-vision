// 
// GoSdkLib.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//  
#ifndef GO_SDK_NET_LIB_H
#define GO_SDK_NET_LIB_H

#include <GoSdk/GoSdkLib.h>
#include <GoSdkNet/GoSdkDef.h>

using namespace System::Threading;

namespace Lmi3d
{
    namespace GoSdk
    {

        /// <summary>Assembly manager for the SensorApi API native type assembly (kSxLib).</summary>
        public ref class GoSdkLib : public KAssemblyManager
        {
        public:
 
            /// <summary>Constructs the GoSdkLib assembly manager.</summary>
            ///
            /// <remarks>
            /// <para>If an instance of the GoSdkLib assembly manager does not currently exist, the Construct method 
            /// will create a new instance and return it. If an instance of the GoSdkLib assembly manager already exists, 
            /// the Construct method will increment its reference count and return the existing instance.</para>
            ///
            /// <para>The GoSdkLib instance returned by this method should be disposed when no longer needed.</para>
            /// 
            /// <para>This method is thread safe.</para>
            /// </remarks>
            static GoSdkLib^ Construct()
            {
                Monitor::Enter(m_lock);

                try
                {
                    //create new ref-counted instance only if necessary
                    if (m_instance == nullptr)
                    {
                        m_instance = gcnew GoSdkLib();
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

            /// <summary>Disposes the GoSdkLib assembly manager.</summary>
            ///
            /// <remarks>
            /// <para>This method decrements the GoSdkLib reference count. If the reference count reaches zero, the 
            /// GoSdkLib instance will be disposed.</para>
            /// 
            /// <para>This method is thread safe.</para>
            /// </remarks>
            ~GoSdkLib()
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

                        //free any dependent assembly managers
                        delete m_zenLib;

                        m_assembly = kNULL;
                        m_instance = nullptr;
                    }
                }
                finally
                {
                    Monitor::Exit(m_lock);
                }
            }

            /// <summary>Gets a reference to the GoSdkLib singleton instance.</summary>
            static property GoSdkLib^ Instance 
            { 
                GoSdkLib^ get() 
                { 
                    return m_instance; 
                } 
            }

        internal:

            GoSdkLib()
            {
                kAssembly handle = kNULL;

                Monitor::Enter(m_lock);

                try
                {
                    //construct any dependent assembly managers
                    m_zenLib = KApiLib::Construct();

                    //construct native assembly
                    KCheck(GoSdk_Construct(&handle));

                    //register .NET types in this assembly
                    KAssemblyManager::Register();

                    m_assembly = handle;
                    m_refCount = 1;
                }
                catch (...)
                {
                    if (m_zenLib) delete m_zenLib;

                    kObject_Destroy(handle);

                    throw;
                }
                finally
                {
                    Monitor::Exit(m_lock);
                }
            }

            ///<summary>Lock object to provide thread-safe access.</summary>
            static Object^ m_lock = gcnew Object();

            ///<summary>Singleton instance.</summary>
            static GoSdkLib^ m_instance;

            ///<summary>Reference to dependent assembly manager.</summary>
            KApiLib^ m_zenLib;
        };
    }
}

#endif 
