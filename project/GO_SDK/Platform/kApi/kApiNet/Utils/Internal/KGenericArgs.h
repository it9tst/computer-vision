// 
// KAssemblyManager.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_GENERIC_ARGS_H
#define K_API_NET_GENERIC_ARGS_H

#include <kApi/kApiDef.h>
#include <kApi/kType.h>
#include "kApiNet/Utils/Internal/KInternalUtils.h"

namespace Lmi3d
{
    namespace Zen
    {
        namespace Utils
        {
            namespace Internal 
            {
                /// <summary>Helper class that can be used when translating Zen generics to CLR generics.</summary>
                generic <typename T>
                private value class KGenericArg
                {
                public:
                    KGenericArg(kType type) 
                        : m_handle(kNULL), m_type(type)
                    {}

                    KGenericArg(kType type, T% clrData)
                        : m_handle(kNULL), m_type(type)
                    {
                        if (kType_IsReference(type))
                        {
                            m_handle = KToHandle(clrData);
                        }
                        else
                        {
                            KCheckArgs(kType_Size(type) == sizeof(T));

                            m_value = clrData;
                        }
                    }

                    KGenericArg(kType type, const void* zenData)
                        : m_handle(kNULL), m_type(type)
                    {
                        if (kType_IsReference(type))
                        {
                            m_handle = *(kObject*)zenData;
                        }
                        else
                        {
                            pin_ptr<T> valuePointer = &m_value;

                            KCheckArgs(sizeof(T) == kType_Size(type));

                            kItemCopy(valuePointer, zenData, sizeof(T));
                        }
                    }

                    property void* ZenData
                    {
                        void* get()
                        {
                            if (kType_IsReference(m_type))
                            {
                                pin_ptr<kObject> p = &m_handle;
                                return (void*)p;
                            }
                            else
                            {
                                pin_ptr<T> p = &m_value;
                                return (void*)p;
                            }
                        }
                    }

                    property T ClrData
                    {
                        T get()
                        {
                            if (kType_IsReference(m_type))
                            {
                                m_value = KToObject<T>(m_handle);
                            }

                            return m_value;
                        }
                    }

                    property kType ZenType
                    {
                        kType get()
                        {
                            return m_type;
                        }
                    }

                private:
                    kType m_type;
                    kObject m_handle; 
                    T m_value; 
                };
            }
        }
    }
}

#endif
