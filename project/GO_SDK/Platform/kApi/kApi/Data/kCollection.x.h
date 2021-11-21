/** 
 * @file    kCollection.x.h
 *
 * @internal
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_COLLECTION_X_H
#define K_API_COLLECTION_X_H

typedef struct kCollectionVTable
{   
    kIterator (kCall* VGetIterator)(kCollection collection);
    kType (kCall* VItemType)(kCollection collection);
    kSize (kCall* VCount)(kCollection collection); 
    kBool (kCall* VHasNext)(kCollection collection, kIterator iterator); 
    void* (kCall* VNext)(kCollection collection, kIterator* iterator);  
} kCollectionVTable;

kDeclareInterfaceEx(k, kCollection, kNull) 

/*
* Forard declarations.
*/

kInlineFx(kType) kCollection_ItemType(kCollection collection); 
kInlineFx(void*) kCollection_Next(kCollection collection, kIterator* iterator);

/* 
* Private methods. 
*/

kInlineFx(kIterator) xkCollection_VGetIterator(kCollection collection)  { return kNULL; }
kInlineFx(kType) xkCollection_VItemType(kCollection collection) { return kNULL; }
kInlineFx(kSize) xkCollection_VCount(kCollection collection) { return 0; }
kInlineFx(kBool) xkCollection_VHasNext(kCollection collection, kIterator iterator) { return kFALSE; }
kInlineFx(void*) xkCollection_VNext(kCollection collection, kIterator* iterator) { return kNULL; }

kInlineFx(void*) xkCollection_NextT(kCollection collection, kIterator* iterator, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kCollection_ItemType(collection), itemSize)); 

    return kCollection_Next(collection, iterator);
} 

#endif
