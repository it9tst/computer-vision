//
// KCollection.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_COLLECTION_H
#define K_API_NET_COLLECTION_H

#include <kApi/Data/kCollection.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents an iterator used with a KCollection instance.</summary>           
            public ref struct KIterator
            {          
            public:               
                /// <summary>Determines whether the collection has another item.</summary>
                /// 
                /// <param name="iterator">Collection iterator.</param>
                /// <returns>true if the collection has a next element.</returns>                
                property bool HasNext
                {
                    bool get()
                    {
                        return KToBool(kCollection_HasNext(m_collection, m_iterator));
                    }
                }

                /// <summary>Gets the next collection element and then advances the iterator.</summary>
                /// 
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <returns>Next collection element.</returns>
                generic <typename T>
                T GetNext()
                {
                    return GetNext<T>(Nullable<KRefStyle>());
                }

                /// <inheritdoc cref="GetNext()" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                generic <typename T>
                T GetNext(Nullable<KRefStyle> refStyle)
                {
                    kType itemType = kCollection_ItemType(m_collection);
                    kIterator it = m_iterator;
                    
                    if (kType_IsValue(itemType))
                    {
                        void* item = kCollection_Next(m_collection, &it);
                        T value;

                        m_iterator = it;

                        KCheckArgs(sizeof(T) == kType_Size(itemType)); 

                        kItemCopy(&value, item, sizeof(T));

                        return value;
                    }
                    else
                    {
                        kObject object = *(kObject*)kCollection_Next(m_collection, &it);

                        m_iterator = it;
                        ShareRef(object, refStyle);

                        return KToObject<T>(object, refStyle);
                    }
                }

                virtual bool Equals(Object^ other) override
                {
                    KIterator^ it = dynamic_cast<KIterator^>(other); 

                    if (it != nullptr)
                    {
                        return (it->m_collection == this->m_collection) && (it->m_iterator == this->m_iterator);
                    }

                    return false;
                }
           
            internal:
                KIterator(Object^ root, kCollection collection, kIterator it)
                    : m_root(root), m_collection(collection), m_iterator(it) {}

                static kPointer ToHandle(KIterator^ it)
                {
                    return (it == nullptr) ? kNULL : it->m_iterator;
                }

                static KIterator^ ToObject(Object^ root, kCollection collection, kIterator it)
                {
                    return (it == kNULL) ? nullptr : gcnew KIterator(root, collection, it);
                }

            private:
                Object^ m_root;               //prevents premature finalization of parent object
                kCollection m_collection;
                kIterator m_iterator;

                void ShareRef(kObject object, Nullable<KRefStyle> refStyle)
                {
                    if (object == kNULL || kType_IsValue(kObject_Type(object)))
                    {
                        return;
                    }
                    
                    KAdjustRef(object, kTRUE, refStyle);
                }
            };      
          
            /// <summary>Represents a collection of elements.</summary>
            ///
            /// <remarks>KCollection represent the underlying Zen kCollection interface, which supports forward iteration over a 
            /// collection of elements. Typically, the classes that implement this interface provide alternative, class-specific 
            /// accessor methods with better performance. The KCollection interface can be used to reduce the amount 
            /// of container-specific code required to iterate over a variety of collections, in contexts where performance is not 
            /// paramount.
            ///
            /// <para>Default KRefStyle: Auto</para>
            ///</remarks>
            ///
            /// <example>This example demonstrates use of KCollection with a KArrayList of K32s elements.
            /// <code language="C#" source="examples/Data/KCollection_Iteration.cs" />
            /// </example>
            public ref class KCollection : public KObject
            {
                KDeclareAutoInterface(KCollection, kCollection)

            public:
                /// <summary>Initializes a new instance of the KCollection class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KCollection(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Gets the collection element type.</summary>
                property KType^ ItemType
                {
                    KType^ get() { return gcnew KType(kCollection_ItemType(Handle)); }
                }

                /// <summary>Gets the collection element count.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) kCollection_Count(Handle); }
                }

                /// <summary>Gets an iterator to the first element in the collection.</summary>
                property KIterator^ Iterator
                {
                    KIterator^ get() 
                    { 
                        kIterator it = kCollection_GetIterator(Handle); 

                        return KIterator::ToObject(this, Handle, it); 
                    }
                }

            protected:
                KCollection() 
                    : KObject(DefaultRefStyle)
                {}
            };
        }
    }
}

#endif
