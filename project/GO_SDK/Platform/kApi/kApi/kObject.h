/** 
 * @file    kObject.h
 * @brief   Declares the kObject class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/kApiDef.h>   //--inclusion order controlled by kApiDef

#ifndef K_API_OBJECT_H
#define K_API_OBJECT_H

#include <kApi/kObject.x.h>

/**
 * @class   kObject
 * @ingroup kApi
 * @brief   Root of all Zen classes. 
 * 
 * The kObject class provides infrastructure to support object destruction, reference counting, and type introspection. 
 *
 * The kObject_Destroy method can be used to destroy an instance of any class. The kObject_Dispose method provides 
 * support for recursive destruction, often used in conjunction with Zen data collections. Refer to @ref kApi-Destruction 
 * for more information. 
 * 
 * The kObject_Clone method can be used to create a deep copy of an object. The implementation of this method 
 * requires additional support from derived types; refer to the documentation for a specific derived type to determine 
 * whether cloning is supported. 
 * 
 * The kObject_Clone and kObject_Dispose methods are often used together to manage collections of data objects: 
 * 
 * @code {.c}
 * 
 * kList list = kNULL; 
 * kList copy = kNULL; 
 * kImage image = kNULL;
 * kSize imageCount = 10; 
 *
 * //create a list of image objects
 * kCheck(kList_Construct(&list, kTypeOf(kImage), imageCount, kNULL)); 
 * 
 * for (kSize i = 0; i < imageCount; ++i)
 * {
 *     kCheck(kImage_Construct(&image, kTypeOf(kArgb), 640, 480, kNULL)); 
 *     kList_Add(list, &image); 
 * }
 * 
 * //make a deep copy of the list; copies both the list and the images contained in the list
 * kCheck(kObject_Clone(&copy, list, kNULL)); 
 * //...
 * 
 * //clean up; destroy the lists and the images contained in the lists
 * kObject_Dispose(list); 
 * kObject_Dispose(copy);
 *
 * @endcode
 *
 * All kObject-derived instances are reference counted. The kObject_Share method can optionally be used to increment an object's 
 * reference count, while the kObject_Destroy/kObject_Dispose methods are always used to decrement an object's reference 
 * count. Refer to @ref kApi-Reference-Counting for more information.
 *
 * The kObject_Type method can be used to access type information for any kObject instance. The kType object returned
 * by this method can be used to learn about the object's class, including its name, base classes, implemented interfaces, 
 * and methods.  The kObject_Is method provides a convenient way to determine whether an object derives from a specific 
 * base class or implements an interface. 
 * 
 * The kObject_Equals and kObject_HashCode methods can be helpful for object comparisons, but require support from 
 * derived types and are infrequently overridden. The kString class overrides both of these methods to support the use of 
 * kString objects as hash keys in kMap data collections. 
 */
//typedef kPointer kObject;   --forward-declared in kApiDef.x.h

/*
* Public 
*/

/** 
 * Constructs a new object by copying an existing object, including any aggregated child elements. 
 *
 * If the source object is an object collection (e.g. kArrayList<kString>), any aggregated child objects 
 * are also cloned. In this case, the kObject_Dispose method can be used to free the cloned collection and 
 * its associated elements. 
 * 
 * This method will fail if the source object (or an aggregated child element) does not support cloning.
 * 
 * To support cloning, derived classes should override the kObject_VClone method. 
 * 
 * @public                  @memberof kObject
 * @param   object          Receives the constructed object. 
 * @param   source          Source object. 
 * @param   objectAllocator Object memory allocator (or kNULL for default). 
 * @return                  Operation status. 
 * @see                     kObject_Dispose
 */
kInlineFx(kStatus) kObject_Clone(kObject* object, kObject source, kAlloc objectAllocator)
{
    if (!kIsNull(source))
    {
        return xkObject_CloneImpl(object, source, objectAllocator, objectAllocator, kNULL);
    }
    
    *object = kNULL;
    
    return kOK;  
}

/** 
 * Constructs a new object by copying an existing object, including any aggregated child elements. 
 *
 * If the source object is an object collection (e.g. kArrayList<kString>), any aggregated child objects 
 * are also cloned. In this case, the kObject_Dispose method can be used to free the cloned collection and 
 * its associated elements. 
 * 
 * This method will fail if the source object (or an aggregated child element) does not support cloning.
 * 
 * The valueAllocator argument can be used to specify a separate memory allocator to be used for 
 * for value-array allocations. This feature can be used to allocate primitive arrays with special requirements 
 * (e.g., Cuda device memory). Support for this feature is typically provided only by classes that implement 
 * contiguous array-based collections (e.g., kArray2); other classes will ignore the valueAllocator argument 
 * and use the objectAllocator argument for all allocations. Collections that do support the valueAllocator 
 * agument will only use this allocator when allocating primitive value arrays; reference arrays 
 * (e.g., kArray2<kObject>) will be allocated with the objectAllocator argument. Consult the documentation 
 * for individual collections classes to determine whether separate valueAllocator arguments are supported.
 * 
 * To support cloning, derived classes should override the kObject_VClone method. 
 * 
 * @public                      @memberof kObject
 * @param   object              Receives the constructed object. 
 * @param   source              Source object. 
 * @param   objectAllocator     Object memory allocator (or kNULL for default). 
 * @param   valueAllocator      Value memory allocator (or kNULL for default). 
 * @param   context             Context for copy operation (allocator specific; not usually required).
 * @return                      Operation status. 
 * @see                         kObject_Dispose
 */
#if defined (K_CPP)
kInlineFx(kStatus) kObject_Clone(kObject* object, kObject source, kAlloc objectAllocator, kAlloc valueAllocator, kObject context = kNULL)
{
    if (!kIsNull(source))
    {
        return xkObject_CloneImpl(object, source, objectAllocator, valueAllocator, context);
    }
    
    *object = kNULL;
    
    return kOK;  
}
#endif

/** 
 * Increments the reference count associated with this object. 
 *
 * This method is thread-safe.
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Operation status. 
 * @see                 reference-counting
 */
kInlineFx(kStatus) kObject_Share(kObject object)
{
    kObj(kObject, object);

    kAtomic32s_Increment(&obj->refCount); 
    
    return kOK; 
}

/** 
 * Sets the object pool associated with this object. 
 *
 * Object pools can be used to implement custom lifetime management. If an object has an assigned pool, 
 * then the kObjectPool_Reclaim method will be called just prior to destruction, to provide an opportunity
 * for the object to be reclaimed. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   pool        Pool object (or kNULL to clear the pool assignment). 
 * @return              Operation status. 
 */
kInlineFx(kStatus) kObject_SetPool(kObject object, kObjectPool pool)
{
    kObj(kObject, object);

    obj->pool = pool; 

    return kOK; 
}

/** 
 * Destroys the object. 
 * 
 * The kObject_Destroy method destroys the object itself and any resources that are owned by the object.  
 * See @ref kApi-Destruction for more information. 
 * 
 * When an object is destroyed (or disposed), its reference count is decremented. The object is only truly 
 * destroyed when the reference count reaches zero. See @ref kApi-Reference-Counting for more information.
 * 
 * To support releasing resources, derived classes should override the kObject_VRelease method. 
 * 
 * @public              @memberof kObject
 * @param   object      Object (or kNULL). 
 * @return              Operation status. 
 * @see                 @ref kApi-Destruction, @ref kApi-Reference-Counting
 */
kInlineFx(kStatus) kObject_Destroy(kObject object)
{
    if (!kIsNull(object))
    {
        return xkObject_DestroyImpl(object, kFALSE);
    }
    
    return kOK; 
}

/** 
 * Destroys the object and any aggregated child elements. 
 * 
 * The kObject_Dispose method destroys the object itself, any resources that are owned by the object, and 
 * if the object represents a collection of objects, any child objects in the collection. See @ref kApi-Destruction 
 * for more information. 
 * 
 * When an object is destroyed (or disposed), its reference count is decremented. The object is only truly 
 * destroyed when the reference count reaches zero. See @ref kApi-Reference-Counting for more information.
 * 
 * To support destroying aggregated child elements during kObject_Dispose, derived classes should override the 
 * kObject_VDisposeItems method. 
 * 
 * @public              @memberof kObject
 * @param   object      Object (or kNULL). 
 * @return              Operation status. 
 * @see                 @ref kApi-Destruction, @ref kApi-Reference-Counting
 */
kInlineFx(kStatus) kObject_Dispose(kObject object)
{
    if (!kIsNull(object))
    {
        return xkObject_DestroyImpl(object, kTRUE);
    }
    
    return kOK; 
}

/**
 * Returns the type of the object.
 *
 * Each object is an instance of a specific class type. The type handle returned by this function can be 
 * used to learn about the class. 
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Type. 
 * @see                 kType
 */      
kInlineFx(kType) kObject_Type(kObject object)
{
    kObj(kObject, object);

    return obj->type; 
}

/**
 * Determines whether this object is an instance of the specified type.
 * 
 * This function compares the type of this object with the given type. An object is considered to be an 
 * instance of a given type if a) the type represents a class and this object inherits from (or instantiates) 
 * that class, or b) the type represents an interface and this object implements the interface. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   type        Type. 
 * @return              kTRUE if the object is of the specified type; otherwise kFALSE. 
 * @see                 kType_Is
 */
kInlineFx(kBool) kObject_Is(kObject object, kType type)
{
    kObjR(kObject, object);

    return !kIsNull(object) && !kIsNull(type) && !kIsNull(obj->type) &&
           xkObject_RawVerifyTag(object) && kType_Is(obj->type, type); 
}

/**
 * Determines whether the object is equal to another object. 
 * 
 * By default, objects are compared by reference; objects are considered equal if the given handles refer to 
 * the same object instance. The kObject_VEquals method can optionally be overridden to provide a more meaningful 
 * equality comparison. 
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @param   other       Object for comparison. 
 * @return              kTRUE if the objects are equal.
 */
kInlineFx(kBool) kObject_Equals(kObject object, kObject other)
{
    return xkObject_VTable(object)->VEquals(object, other); 
}

/**
 * Gets a hash code representing the state of this object.
 * 
 * By default, objects return a hash code based on the object handle value. The kObject_VHashCode method can optionally 
 * be overridden to provide a more useful hash. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Hash code. 
 */
kInlineFx(kSize) kObject_HashCode(kObject object)
{
    return xkObject_VTable(object)->VHashCode(object); 
}

/**
 * Gets the memory allocator associated with this object.  
 * 
 * Most objects are constructed with an allocator, which is used to allocate the memory required by the 
 * object. Objects retain a reference to this allocator to enable further allocations and to free memory when 
 * the object is destroyed. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Memory allocator. 
 * @see                 kAlloc
*/
kInlineFx(kAlloc) kObject_Alloc(kObject object)
{
    kObj(kObject, object);

    return obj->alloc;
}

/** 
 * Gets the bitset of allocator traits for any allocators used within this object, including aggregated child elements.
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Bitset of alloator traits.
 */
kInlineFx(kAllocTrait) kObject_AllocTraits(kObject object)
{
    return xkObject_VTable(object)->VAllocTraits(object);
}

/** 
 * Reports whether the object, including aggregated child elements, contains any foreign memory references.   
 * 
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              kTRUE if the object references memory allocated in a foreign address space. 
 */
kInlineFx(kBool) kObject_HasForeignData(kObject object)
{
    return (kObject_AllocTraits(object) & kALLOC_TRAIT_FOREIGN) != 0;
}

/**
 * Estimates the memory consumed by this object, including any aggregated child elements. 
 * 
 * The default implementation of this method reports only the size of the class instance (additional allocations 
 * performed by the class are excluded). Derived classes can override kObject_VSize to provide type-specific 
 * size estimates. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              Object size, in bytes.  
*/
kInlineFx(kSize) kObject_Size(kObject object)
{    
    return xkObject_VTable(object)->VSize(object);
}

/**
 * Reports whether the object is currently shared (reference count greater than one). 
 * 
 * Objects are initialized with a reference count of one. The kObject_Share method can be used to increment 
 * the reference count. The kObject_Destroy and kObject_Dispose methods decrease the reference count, and 
 * when the reference count reaches zero, the object is actually destroyed/disposed. 
 * 
 * This method can be used to determine if the reference count of an object is currently greater than one. 
 * 
 * This method is thread-safe. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              kTRUE if the object is shared. 
 * @see                 @ref kApi-Reference-Counting, kObject_Share, kObject_HasShared
*/
kInlineFx(kBool) kObject_IsShared(kObject object)
{
    kObj(kObject, object);

    return kAtomic32s_Get(&obj->refCount) > 1; 
}

/**
 * Reports whether an object or any of its aggregated child elements has a reference count greater than one.
 *  
 * This method is thread-safe. 
 *
 * @public              @memberof kObject
 * @param   object      Object. 
 * @return              kTRUE if the object contains shared objects. 
 * @see                 @ref kApi-Reference-Counting, kObject_Share, kObject_IsShared
*/
kInlineFx(kBool) kObject_HasShared(kObject object)
{    
    return xkObject_VTable(object)->VHasShared(object);
}

/*
* Protected 
*/

/**
 * Protected method called by derived classes to initialize the kObject base class. 
 * 
 * This method should typically be called as the first statement in derived initializer methods. 
 *
 * @protected           @memberof kObject
 * @param   object      Object instance (not yet initialized). 
 * @param   type        Object type (required).
 * @param   alloc       Memory allocator (required).
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_Init(kObject object, kType type, kAlloc alloc) 
{ 
    kObjR(kObject, object);

    obj->type = type; 
    obj->alloc = alloc; 
    obj->pool = kNULL; 
    obj->tag = xkOBJECT_TAG; 

    kAtomic32s_Init(&obj->refCount, 1);

    return kOK; 
}

/**
 * Protected method called by derived classes to allocate memory using the object's allocator. 
 * 
 * This method is provided for convenience; alternatively, the allocator provided by kObject_Alloc can also be used directly.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives pointer to allocated memory (pointer to a pointer).
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_GetMem(kObject object, kSize size, void* mem)
{
    return kAlloc_Get(kObject_Alloc(object), size, mem);
}

/**
 * Protected method called by derived classes to allocate and zero memory using the object's allocator. 
 * 
 * This method is provided for convenience; alternatively, the allocator provided by kObject_Alloc can also be used directly.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @param   size        Size of memory to allocate, in bytes.
 * @param   mem         Receives pointer to allocated memory (pointer to a pointer).
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_GetMemZero(kObject object, kSize size, void* mem)
{
    return kAlloc_GetZero(kObject_Alloc(object), size, mem); 
}

/**
 * Protected method called by derived classes to free memory using the object's allocator. 
 * 
 * This method is provided for convenience; alternatively, the allocator provided by kObject_Alloc can also be used directly.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @param   mem         Pointer to memory to free (or kNULL). 
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_FreeMem(kObject object, void* mem)
{
    return kAlloc_Free(kObject_Alloc(object), mem);
}

/**
 * Protected method called by derived classes to free memory (and reset the provided memory pointer to kNULL) using the object's allocator. 
 * 
 * This method is provided for convenience; alternatively, the allocator provided by kObject_Alloc can also be used directly.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @param   mem         Pointer to pointer to memory to free (or pointer to kNULL). 
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_FreeMemRef(kObject object, void* mem)
{
    return kAlloc_FreeRef(kObject_Alloc(object), mem);
}

/**
 * Protected virtual method that deallocates any resources owned by the object. 
 * 
 * This method can be overridden in derived classes to free any resources owned by 
 * derived instances. 
 * 
 * Each derived class should call the VRelease method of its parent class as the 
 * last statement in the derived VRelease method. 
 * 
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_VRelease(kObject object)                  
{
    kObjR(kObject, object);

    obj->tag = 0;
    
    return kOK;
}

/**
 * Protected virtual method that clones (makes a deep copy of) the specified source object.
 * 
 * This method can be overridden in derived classes to support kObject_Clone.
 * 
 * There is no requirement to call kObject_VClone from derived VClone methods.
 * 
 * Types that override this virtual method should also provide a framework constructor via the 
 * @ref kAddFrameworkConstructor or @ref kAddPrivateFrameworkConstructor macros.
 *
 * @protected               @memberof kObject
 * @param   object          Object instance (initialized via framework constructor). 
 * @param   source          Object to be cloned (required).
 * @param   valueAllocator  Value memory allocator; can optionally be used by types that support a separate allocator for value content (e.g., kArray1).
 * @param   context         Context for copy operation (allocator specific; not usually provided).
 * @return                  Operation status.  
 * @see                     kObject_Clone
*/
kInlineFx(kStatus) kObject_VClone(kObject object, kObject source, kAlloc valueAllocator, kObject context)
{
    return kERROR_UNIMPLEMENTED;
}

/**
 * Protected virtual method that destroys any aggregated child objects associated with a collection.
 * 
 * This method can be overridden in derived collection classes to support kObject_Dispose.
 * 
 * There is no requirement to call kObject_VDisposeItems from derived VDisposeItems methods.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @return              Operation status.  
*/
kInlineFx(kStatus) kObject_VDisposeItems(kObject object)
{
    return kOK;
}

/** 
 * Protected virtual method that reports whether an object or any of its aggregated child elements has a reference count greater than one.
 * 
 * @protected           @memberof kObject
 * @param   object      Object. 
 * @return              Bitset of alloator traits.
 */
kInlineFx(kBool) kObject_VHasShared(kObject object)
{    
    kObj(kObject, object);

    return kObject_IsShared(object); 
}

/** 
 * Protected virtual method that gets the bitset of allocator traits for any allocators used within this object, including aggregated child elements.
 * 
 * @protected           @memberof kObject
 * @param   object      Object. 
 * @return              Bitset of alloator traits.
 */
kInlineFx(kAllocTrait) kObject_VAllocTraits(kObject object)
{    
    kObj(kObject, object);

    return kAlloc_Traits(obj->alloc); 
}

/**
 * Protected virtual method that calculates the total size (in bytes) of the object instance.
 * 
 * This method can be overridden in derived collection classes to support kObject_Size.
 * 
 * Collection classes should report the size of any aggregated child elements. 
 * 
 * There is no requirement to call kObject_VSize from derived VSize methods. However, derived
 * classes have the option of doing so, if it simplifies implementation.
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @return              Operation status.  
*/
kInlineFx(kSize) kObject_VSize(kObject object)
{
    return kType_InnerSize(kObject_Type(object)); 
}

/**
 * Protected virtual method that calculates a hash code representing the object instance. 
 * 
 * This method can be overridden in derived collection classes to support kObject_HashCode.
 * 
 * The purpose of this method is to generate a hash code that can be used with hash-based 
 * collections such as kMap. 
 *
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @return              Hash code. 
*/
kInlineFx(kSize) kObject_VHashCode(kObject object)
{
    return xkHashPointer(object); 
}

/**
 * Protected virtual method that compares two objects for equality. 
 * 
 * This method can be overridden in derived collection classes to support kObject_Equals.
 * 
 * The default implementation of this method compares objects according to pointer/handle 
 * equality. In many cases, this is adequate/appropriate. This method can be overidden 
 * to implement a more specialized/type-specific representation of equality. 
 * 
 * @protected           @memberof kObject
 * @param   object      Object instance. 
 * @param   other       Object for comparison. 
 * @return              kTRUE if the objects are equal.
*/
kInlineFx(kBool) kObject_VEquals(kObject object, kObject other)
{
    return (object == other); 
}

#endif 
