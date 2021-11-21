/** 
 * @file    kObject.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_OBJECT_X_H
#define K_API_OBJECT_X_H

#define xkOBJECT_TAG           (0x44332211)       //used for object validity check (debug feature)

typedef struct kObjectClass
{
    /*
    * Private Fields
    */
    kType type;                      //[Private] instance type   
    kAlloc alloc;                    //[Private] memory allocator   

    kObjectPool pool;                //[Private] object pool (or kNULL)

    k32u tag;                        //[Private] used for validity check (lowest bit reserved for legacy 'init' bit)
    kAtomic32s refCount;             //[Private] reference count (initially one, destroy when zero)

} kObjectClass;

typedef struct kObjectVTable
{
    kStatus (kCall* VRelease)(kObject object); 
    kStatus (kCall* VDisposeItems)(kObject object); 
    kStatus (kCall* VInitClone)(kObject object, kObject source, kAlloc allocator);   //deprecated
    kStatus (kCall* VClone)(kObject object, kObject source, kAlloc valueAllocator, kObject context); 
    kBool (kCall* VHasShared)(kObject object); 
    kSize (kCall* VHashCode)(kObject object); 
    kBool (kCall* VEquals)(kObject object, kObject other); 
    kSize (kCall* VSize)(kObject object); 
    kAllocTrait (kCall* VAllocTraits)(kObject object); 
} kObjectVTable; 

kDeclareVirtualClassEx(k, kObject, kNull)

/* 
* Private methods. 
*/

kFx(kStatus) xkObject_CloneImpl(kObject* object, kObject source, kAlloc objectAllocator, kAlloc valueAllocator, kObject context);

kFx(kStatus) xkObject_DestroyImpl(kObject object, kBool dispose);

kInlineFx(kBool) xkObject_RawVerifyTag(kObject object)
{
    kObjR(kObject, object);

    return obj->tag == xkOBJECT_TAG; 
}

kInlineFx(kType) xkObject_RawType(kObject object)
{
    kObjR(kObject, object);

    return obj->type; 
}

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Replace with class-specific approach
kInlineFx(kStatus) kObject_Release(kObject object)                    
{
    return xkObject_VTable(object)->VRelease(object); 
}

//[Deprecated] Replace with class-specific approach
kInlineFx(kStatus) kObject_DisposeItems(kObject object)
{
    return xkObject_VTable(object)->VDisposeItems(object); 
}

//[Deprecated] Implement VClone instead; note VClone receives an object that has been pre-initialized via the framework constructor.
kInlineFx(kStatus) kObject_VInitClone(kObject object, kObject source, kAlloc allocator)
{
    return kERROR_UNIMPLEMENTED;
}

//[Deprecated] Convert to kObjectInit
#define kObject_InitClone(OBJ, SRC, ALLOC)  kObject_Init((OBJ), kObject_Type((SRC)), ALLOC)

#endif
