//
// KObject.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef K_API_NET_OBJECT_H
#define K_API_NET_OBJECT_H

#include "kApiNet/KApiDef.h"

namespace Lmi3d
{
    namespace Zen 
    {
        /// <summary>Root of all classes in the Zen.NET type system.</summary>       
        ///
        /// <remarks>
        /// <para>The KObject class provides infrastructure to support object destruction, reference counting, and 
        /// native type introspection.</para>
        ///
        /// <para>Classes derived from KObject that use KRefStyle::Auto will be automatically disposed by the 
        /// garbage collector. They should still be disposed of manually where possible to reduce overhead and ensure
        /// proper lifetime.</para>
        ///
        /// <para>The Destroy method can be used to destroy an instance of any class. The Dispose method provides 
        /// support for recursive destruction, often used in conjunction with Zen data collections.</para>
        ///
        /// <para>The Clone method can be used to create a deep copy of an object. The implementation of this method 
        /// requires additional support from derived types; refer to the documentation for a specific derived type to determine 
        /// whether cloning is supported.</para>
        ///
        /// <para>All KObject-derived instances are reference counted. The Share method can optionally be used to increment an object's 
        /// reference count, while the Destroy/Dispose methods are always used to decrement an object's reference count.
        /// Objects that use KRefStyle::Auto should not be shared manually.</para>
        ///
        /// <para>The GetKType method can be used to access native type information for any KObject instance. The KType object returned
        /// by this method can be used to learn about the underlying Zen object's class, including its name, base classes, 
        /// implemented interfaces, and methods.</para>
        ///
        /// <para>Refer to the documentation for the native Zen library (kApi) for more information on object destruction or 
        /// reference counting.</para>
        /// </remarks>
        /// 
        /// <example>This example illustrates use of the Clone and Dispose methods to manage collections of data objects:
        /// <code language="C#" source="examples/KObject_Clone_Dispose.cs" />
        /// </example>
        ///
        public ref class KObject
        {
        public:
            static property Lmi3d::Zen::KType^ KTypeId                                  
            {                                                                           
                Lmi3d::Zen::KType^ get();                                               
            }                                                                           
                 
            /// <summary>Reports the default ref style for the object.</summary>
            static property Lmi3d::Zen::KRefStyle DefaultRefStyle                       
            {                                                                           
                Lmi3d::Zen::KRefStyle get()                                             
                {                                                                       
                    return Lmi3d::Zen::KRefStyle::Manual;                               
                }                                                                       
            } 

            /// <summary>Initializes a new instance of the KObject class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            KObject(IntPtr handle, KRefStyle refStyle)
                : m_handle((kPointer)handle), m_refStyle(refStyle)
            {
                Init();
            }
                        
            /// <summary>Destroys this object and any aggregated child elements.</summary>
            ///
            /// <remarks>
            ///
            /// <para>This method invokes the kObject_Dispose method from the native Zen API. To destroy this object without 
            /// also destroying any aggregated child objects, use the Destroy method instead.</para>
            ///
            /// <para>This method sets the underlying object handle to null.</para>
            ///
            /// </remarks>
            virtual ~KObject()
            {
                if (!kIsNull(m_handle) && (RefStyle != KRefStyle::None))
                {
                    OnDisposing();

                    KCheck(kObject_Dispose(m_handle));

                    m_handle = kNULL;
                }
            }
            
            // Finalizer
            !KObject()
            {
                if (!kIsNull(m_handle) && RefStyle == KRefStyle::Auto)
                {
                    OnDisposing();

                    kObject_Dispose(Handle);

                    m_handle = kNULL;
                }
            }

            /// <summary>Destroys this object.</summary>
            ///
            /// <remarks>
            /// 
            /// <para>This method invokes the kObject_Destroy method from the underlying Zen API. To destroy both the object 
            /// and any aggregated child objects, use the Dispose method instead.</para>
            ///
            /// <para>This method sets the underlying object handle to null.</para>
            ///
            /// </remarks>
            void Destroy()
            {
                if (!kIsNull(m_handle) && (RefStyle != KRefStyle::None))
                {
                    OnDisposing();

                    KCheck(kObject_Destroy(m_handle));

                    m_handle = kNULL;
                }

                GC::SuppressFinalize(this); 
            }

            /// <summary>Destroys this object and any aggregated child elements.</summary>
            ///
            /// <remarks>
            /// <para>This method is obsolete. Is it equivalent to the Dispose method.</para>
            /// </remarks>
            [Obsolete]
            void DisposeAll()
            {
                if (!kIsNull(m_handle) && (RefStyle != KRefStyle::None))
                {
                    OnDisposing();

                    KCheck(kObject_Dispose(m_handle));
                    m_handle = kNULL;
                }

                GC::SuppressFinalize(this);
            }

            ///
            /// <summary>Constructs a new object by copying this object, including any aggregated child elements.</summary>
            ///
            /// <remarks>
            /// <para>If the source object is an object collection (e.g. KArrayList of KString elements), any aggregated child objects
            /// are also cloned. In this case, the Dispose method can be used to free the cloned collection and its associated elements.</para>
            ///
            /// <para>This method will fail if the source object (or an aggregated child element) does not support cloning.</para>
            /// </remarks>
            ///
            /// <typeparam name="T">Cloned object will be cast to this type.</typeparam>
            /// <returns>The cloned object.</returns>
            ///
            /// <seealso cref="Dispose" />         
            generic <typename T> where T : KObject
            T Clone()
            {
                return Clone<T>(nullptr);
            }

            /// <inheritdoc cref="Clone()" />     
            /// <param name="allocator">Optional memory allocator.</param>
            generic <typename T> where T : KObject
            T Clone(KAlloc^ allocator)
            {
                kObject clone = kNULL;

                KCheck(kObject_Clone(&clone, m_handle, KToHandle(allocator)));

                try
                {
                    return KToObject<T>(clone);
                }
                catch (...)
                {
                    kObject_Dispose(clone);
                    throw;
                }
            }

            /// <inheritdoc cref="Clone(KAlloc^)" />     
            /// <param name="refStyle">Ref style of cloned object.</param>
            generic <typename T> where T : KObject
            T Clone(KAlloc^ allocator, KRefStyle refStyle)
            {
                T object = Clone<T>(allocator); 

                object->RefStyle = refStyle; 

                return object; 
            }

            /// <summary>Increments the reference count associated with this object.</summary>
            ///
            /// <remarks>This method is thread-safe.</remarks>
            void Share()
            {
                KCheck(kObject_Share(m_handle));
            }

            /// <summary>Sets the object pool associated with this object.</summary>
            ///
            /// <remarks>Object pools can be used to implement custom lifecycle management. If an object has an assigned pool,
            /// then the KObjectPool.Reclaim method will be called just prior to destruction, to provide an opportunity
            /// for the object to be reclaimed.</remarks>
            /// 
            /// <param name="pool">Pool object (or null to clear the pool assignment).</param>           
            void SetPool(Utils::KObjectPool^ pool)
            {
                KCheck(kObject_SetPool(m_handle, KToHandle(pool)));
            }
            
            /// <summary>Gets type metadata for this object from the underlying Zen type system.</summary>           
            ///
            /// <returns>Type metadata, describing the underlying native Zen object.</returns>
            KType^ GetKType();
            
            /// <summary>Implements equality comparison for the underlying Zen objects.</summary>           
            /// <param name="other">The object to compare with the current object.</param>
            /// <returns>true if the specified object is equal to the current object; otherwise, false.</returns>
            virtual bool Equals(Object^ other) override
            {
                KObject^ kother = dynamic_cast<KObject^>(other);

                if (kother == nullptr)
                {
                    return false;
                }
                else
                {
                    return KToBool(kObject_Equals(m_handle, KToHandle(kother)));
                }
            }
            /// <summary>Gets a hash code for the underlying Zen object. </summary>
            /// <returns>A hash code for the current object.</returns>
            virtual int GetHashCode() override
            {
                return (int)kObject_HashCode(m_handle);
            }

            ///<summary>Attempts to convert this object to an instance of the specified generic type parameter.</summary>
            /// 
            /// <remarks>The underlying Zen type system implements a single-inheritance/multiple-interface class
            /// model that is similar to the .NET object system. However, interfaces from the underlying type system are
            /// represented as .NET classes. The As method allows an obect to be converted to one of the underlying interfaces
            /// that it implements (and back again). 
            /// </remarks>
            ///
            /// <typeparam name="T">Type to which object should be converted.</typeparam> 
            /// <returns>The converted object, or null if the object cannot be converted.</returns>
            generic <typename T>
            T As()
            {
                return KAsObject<T>(m_handle);
            }

            /// <summary>Gets the memory allocator associated with this object.</summary>
            /// 
            /// <remarks>Most objects are constructed with an allocator, which is used to allocate the memory required by the
            /// object. Objects retain a reference to this allocator to enable further allocations and to free memory when
            /// the object is destroyed.</remarks>           
            property KAlloc^ Alloc 
            { 
                KAlloc^ get()
                {
                    kAlloc alloc = kObject_Alloc(m_handle);

                    return KToObject<KAlloc^>(alloc);
                }
            }

            /// <summary>Estimates the memory consumed by this object, including any aggregated child elements.</summary>
            /// 
            /// <remarks>This method can be optionally overridden by KObject-derived classes to report the amount of memory consumed.
            /// The default implementation reports only the size of the class instance (additional allocations performed by the
            /// class are excluded).</remarks>           
            property k64s Size 
            { 
                k64s get()
                {
                    return (k64s)kObject_Size(m_handle);
                }
            }

            /// <summary>Reports whether the object is currently shared (reference count greater than one).</summary>
            /// 
            /// <remarks>
            /// <para>Objects are initialized with a reference count of one. The Share method can be used to increment
            /// the reference count. The Dispose and Destroy methods decrease the reference count, and when the reference count 
            /// reaches zero, the object is actually destroyed/disposed.</para>            
            ///
            /// <para>This method can be used to determine if the reference count of an object is currently greater than one.</para>           
            ///
            /// <para>This method is thread-safe.</para>
            /// </remarks>
            property bool IsShared 
            { 
                bool get()
                {
                    return KToBool(kObject_IsShared(m_handle));
                }
            }

            /// <summary>Reference style of the object.</summary>
            ///
            /// <remarks>
            /// <para>Manual RefStyle means the object is not reference counted and must be manually disposed of. Auto RefStyle
            /// objects will be finalized (disposed) by the garbage collector, but may still be manually disposed of 
            /// without issue.</para>
            /// </remarks>
            property KRefStyle RefStyle
            {
                KRefStyle get()
                {
                    return m_refStyle;
                }

                void set(KRefStyle refStyle)
                {
                    if ((m_refStyle == KRefStyle::Auto) && (refStyle == KRefStyle::Manual))
                    {
                        GC::SuppressFinalize(this); 
                    }
                    else if ((refStyle == KRefStyle::Auto) && (m_refStyle == KRefStyle::Manual))
                    {
                        GC::ReRegisterForFinalize(this); 
                    }

                    m_refStyle = refStyle;
                }
            }

            /// <summary>Gets an IntPtr representing the underlying handle associated with this object.</summary>
            IntPtr ToHandle()
            {
                return IntPtr(m_handle);
            }

            /// <summary>Gets the handle (native pointer) corresponding to a KObject instance.</summary>            
            /// 
            /// <param name="object">Zen.NET object.</param>
            /// <returns>Underlying Zen object handle.</returns>
            static explicit operator kObject(KObject^ object)
            {
                return (object == nullptr) ? kNULL : object->m_handle;
            }

        protected:
            KObject()
                : m_handle(kNULL), m_refStyle(InternalDefaultRefStyle)
            {
                Init();
            }

            KObject(IntPtr handle)
                : m_handle((kPointer)handle), m_refStyle(InternalDefaultRefStyle)
            {
                Init();
            }

            KObject(KRefStyle refStyle)
                : m_handle(kNULL), m_refStyle(refStyle)
            {
                Init();
            }

            /// <summary>This property refers to the static DefaultRefStyle property.</summary>
            ///
            /// <remarks>
            /// <para>This is overriden by subclass objects through the KDeclareClass or KDeclareAutoClass macros. 
            /// This allows the KObject default constructor(s) to reference the static property of an object during construction. This allows
            /// existing code to avoid having to implement new constructors if they are macro'd as Auto classes.</para>
            /// </remarks>
            virtual property Lmi3d::Zen::KRefStyle InternalDefaultRefStyle                       
            {                                                                           
                virtual Lmi3d::Zen::KRefStyle get()                                             
                {                                                                       
                    return DefaultRefStyle;                               
                }                                                                       
            } 

            void Init()
            {
                if (m_refStyle == KRefStyle::Manual)
                {
                    GC::SuppressFinalize(this); 
                }
            }

            /// <summary>Override to implement any special actions that should be undertaken just prior to dispose.</summary>            
            virtual void OnDisposing() {}

            property kPointer Handle
            {
                kPointer get() { return m_handle; }

                void set(kPointer value) 
                { 
                    m_handle = value; 
                }
            }

        private:
            kPointer m_handle;
            KRefStyle m_refStyle; 
        };
    }
}

#endif
