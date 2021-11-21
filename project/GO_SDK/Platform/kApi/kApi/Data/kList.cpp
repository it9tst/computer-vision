/** 
 * @file    kList.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kList.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kList) 

    //serialization versions
    kAddPrivateVersionEx(kList, "kdat6", "6.0.0.0", "kList-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kList, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kList, kObject, VRelease)
    kAddPrivateVMethod(kList, kObject, VDisposeItems)
    kAddPrivateVMethod(kList, kObject, VClone)
    kAddPrivateVMethod(kList, kObject, VHasShared)
    kAddPrivateVMethod(kList, kObject, VSize)
    kAddPrivateVMethod(kList, kObject, VAllocTraits)

    //kCollection interface
    kAddInterface(kList, kCollection); 
    kAddPrivateIVMethod(kList, kCollection, VGetIterator, CollectionGetIterator)
    kAddPrivateIVMethod(kList, kCollection, VItemType, CollectionItemType)
    kAddIVMethod(kList, kCollection, VCount, Count)
    kAddPrivateIVMethod(kList, kCollection, VHasNext, CollectionHasNext)
    kAddPrivateIVMethod(kList, kCollection, VNext, CollectionNext)

kEndClassEx() 

kFx(kStatus) kList_Construct(kList* list, kType itemType, kSize initialCapacity, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kList), list)); 

    if (!kSuccess(status = xkList_Init(*list, kTypeOf(kList), itemType, initialCapacity, alloc)))
    {
        kAlloc_FreeRef(alloc, list); 
    }

    return status; 
} 

kFx(kStatus) xkList_ConstructFramework(kList* list, kAlloc allocator)
{
    return kList_Construct(list, kNULL, 0, allocator);
}

kFx(kStatus) xkList_Init(kList list, kType classType, kType itemType, kSize initialCapacity, kAlloc alloc)
{
    kObjR(kList, list);
    kStatus status = kOK; 

    kCheck(kObject_Init(list, classType, alloc)); 

    obj->blocks = kNULL;
    obj->free = kNULL;
    obj->first = kNULL;
    obj->last = kNULL;
    obj->count = 0;
    obj->capacity = 0;
    kZero(obj->contentField);
    obj->itemSize = 0;
    
    kTry
    {
        kTest(xkList_Layout(list, itemType)); 
        kTest(kList_Reserve(list, initialCapacity));       
    }
    kCatch(&status)
    {
        xkList_VRelease(list); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkList_VClone(kList list, kList source, kAlloc valueAlloc, kObject context)
{ 
    kObj(kList, list);
    kAlloc objectAlloc = kObject_Alloc(list);
    kObject contentClone = kNULL; 
    kStatus status;

    kCheck(kList_Allocate(list, kList_ItemType(source), kList_Count(source))); 

    kTry
    {
        kListItem item = kList_First(source); 

        while (!kIsNull(item))
        {
            const void* content = xkList_ItemContent(source, item); 
            
            if (xkList_ItemIsRef(source))
            {
                kTest(kObject_Clone(&contentClone, *(kObject*)content, objectAlloc, valueAlloc, context)); 
                content = &contentClone; 
            }

            kTest(kList_Add(list, content, kNULL)); 
            contentClone = kNULL; 

            item = kList_Next(source, item); 
        }
    }
    kCatch(&status)
    {
        kObject_Dispose(contentClone); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkList_VRelease(kList list)
{
    kCheck(xkList_Deallocate(list)); 

    kCheck(kObject_VRelease(list)); 

    return kOK; 
}

kFx(kStatus) kList_Allocate(kList list, kType itemType, kSize initialCapacity)
{
    kObj(kList, list);   
    
    kCheck(xkList_Deallocate(list)); 

    obj->free = kNULL; 
    obj->first = kNULL; 
    obj->last = kNULL; 
    obj->count = 0; 
    obj->capacity = 0; 
    obj->contentField.type = kNULL; 

    kCheck(xkList_Layout(list, itemType)); 
    kCheck(kList_Reserve(list, initialCapacity));   

    return kOK; 
}

kFx(kStatus) kList_Assign(kList list, kList source)
{
    kObjN(kList, sourceObj, source);
    kListItem item  = kNULL; 

    kCheck(kList_Allocate(list, sourceObj->contentField.type, sourceObj->count)); 

    item = kList_First(source); 

    while (!kIsNull(item))
    {
        kCheck(kList_Add(list, xkList_ItemContent(source, item), kNULL)); 
        
        item = kList_Next(source, item); 
    }

    return kOK; 
}

kFx(kStatus) xkList_Layout(kList list, kType itemType)
{
    kObj(kList, list);   
    xkStructField nextField, previousField; 
    xkStructField* fields[3];  

    nextField.type = kTypeOf(kPointer); 
    nextField.offset = offsetof(kListItemStruct, next); 
    nextField.count = 1; 
    fields[0] = &nextField; 

    previousField.type = kTypeOf(kPointer); 
    previousField.offset = offsetof(kListItemStruct, previous); 
    previousField.count = 1; 
    fields[1] = &previousField; 

    obj->contentField.type = kIsNull(itemType) ? kTypeOf(kVoid) : itemType;
    obj->contentField.offset = kSIZE_NULL; 
    obj->contentField.count = 1; 
    fields[2] = &obj->contentField; 

    kCheck(xkType_LayoutStruct(fields, kCountOf(fields), &obj->itemSize)); 
              
    return kOK; 
}

kFx(kStatus) xkList_Deallocate(kList list)
{
    kObj(kList, list);   

    while (!kIsNull(obj->blocks))
    {
        kListItemBlock* next = obj->blocks->next; 

        kCheck(kAlloc_Free(kObject_Alloc(list), obj->blocks)); 

        obj->blocks = next; 
    }

    return kOK; 
}

kFx(kBool) xkList_VHasShared(kList list)
{
    kObj(kList, list);

    if (kObject_IsShared(list))
    {
        return kTRUE;
    }
    else if (xkList_ItemIsRef(list))
    {
        kListItem item = kList_First(list); 
        
        while (!kIsNull(item))
        {
            kObject content = kList_AsT(list, item, kObject); 

            if (!kIsNull(content) && kObject_HasShared(content))
            {
                return kTRUE;
            }

            item = kList_Next(list, item); 
        }
    }

    return kFALSE;
}

kFx(kSize) xkList_VSize(kList list)
{
    kObj(kList, list);   
    kSize size = sizeof(kListClass);
    kListItemBlock* blockObj = obj->blocks; 

    while (!kIsNull(blockObj))
    {
        size += (kByte*)blockObj->items - (kByte*)blockObj; 
        size += obj->itemSize*blockObj->itemCount;

        blockObj = blockObj->next; 
    }

    if (xkList_ItemIsRef(list))
    {
        kListItem item = kList_First(list); 
        
        while (!kIsNull(item))
        {
            kObject content = kList_AsT(list, item, kObject); 

            if (!kIsNull(content))
            {
                size += kObject_Size(content); 
            }

            item = kList_Next(list, item); 
        }
    }

    return size; 
}

kFx(kAllocTrait) xkList_VAllocTraits(kList list)
{
    kObj(kList, list);
    kAllocTrait traits = kAlloc_Traits(kObject_Alloc(list));

    if (xkList_ItemIsRef(list))
    {
        kListItem item = kList_First(list); 
        
        while (!kIsNull(item))
        {
            kObject content = kList_AsT(list, item, kObject); 

            if (!kIsNull(content))
            {
                traits |= kObject_AllocTraits(content);
            }

            item = kList_Next(list, item); 
        }
    }

    return traits;
}

kFx(kStatus) xkList_WriteDat6V0(kList list, kSerializer serializer)
{
    kObj(kList, list);   
    kListItem item = kList_First(list); 
    kTypeVersion contentVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->contentField.type, &contentVersion)); 
    kCheck(kSerializer_WriteSize(serializer, obj->count)); 

    while (!kIsNull(item))
    {
        kCheck(kSerializer_WriteItems(serializer, obj->contentField.type, contentVersion, xkList_ItemContent(list, item), 1)); 

        item = kList_Next(list, item); 
    }

    return kOK; 
}

kFx(kStatus) xkList_ReadDat6V0(kList list, kSerializer serializer)
{
    kObj(kList, list); 
    kListItemStruct* itemObj = kNULL; 
    kType contentType; 
    kTypeVersion contentVersion; 
    kSize count = 0;  

    kCheck(kSerializer_ReadType(serializer, &contentType, &contentVersion)); 
    kCheck(kSerializer_ReadSize(serializer, &count)); 

    kCheck(kList_Allocate(list, contentType, count)); 

    for (kSize i = 0; i < count; ++i)
    {   
        itemObj = obj->free; 
        obj->free = itemObj->next; 

        itemObj->next = kNULL; 
        itemObj->previous = obj->last; 

        if (!kIsNull(itemObj->previous))
        {
            itemObj->previous->next = itemObj; 
        }

        obj->last = itemObj; 
            
        if (kIsNull(obj->first))
        {
            obj->first = itemObj; 
        }

        kItemZero((void*)xkList_ItemContent(list, itemObj), obj->contentField.fieldSize); 

        kCheck(kSerializer_ReadItems(serializer, obj->contentField.type, contentVersion, (void*)xkList_ItemContent(list, itemObj), 1)); 
    }

    obj->count = count; 

    return kOK; 
}

kFx(kStatus) kList_Reserve(kList list, kSize capacity)
{
    kObj(kList, list);   

    if (capacity > obj->capacity)
    {
        kSize itemCount = kMax_(capacity - obj->capacity, xkLIST_MIN_ITEMS_PER_BLOCK); 
        kSize itemOffset = kSize_Align(sizeof(kListItemBlock), kALIGN_ANY); 
        kSize blockSize = kSize_Align(itemOffset + itemCount*obj->itemSize, kALIGN_ANY); 
        kListItemBlock* block = kNULL; 
        kSize i; 

        kCheck(kAlloc_Get(kObject_Alloc(list), blockSize, &block));

        block->itemCount = itemCount; 
        block->items = (kListItemStruct*) kPointer_ByteOffset(block, (kSSize)itemOffset); 

        block->next = obj->blocks; 
        obj->blocks = block; 

        for (i = 0; i < itemCount; ++i)
        {
            kListItemStruct* item = (kListItemStruct*) kPointer_ItemOffset(&block->items[0], (kSSize)i, obj->itemSize); 

            item->next = obj->free; 
            obj->free = item; 
        }

        obj->capacity += itemCount; 
    }

    return kOK; 
}

kFx(kStatus) kList_Add(kList list, const void* itemContent, kListItem* item)
{
    kObj(kList, list);   
    kListItemStruct* itemObj = kNULL; 
    void* contentSlot = kNULL; 

    if (obj->count == obj->capacity)
    {
        kCheck(kList_Reserve(list, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 

    itemObj->next = kNULL; 
    itemObj->previous = obj->last; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj; 
    }

    obj->last = itemObj; 
                
    if (kIsNull(obj->first))
    {
        obj->first = itemObj; 
    }

    obj->count++; 

    contentSlot = (void*)xkList_ItemContent(list, itemObj); 

    if (!kIsNull(itemContent))
    {
        kValue_Import(obj->contentField.type, contentSlot, itemContent); 
    }
    
    if (!kIsNull(item))
    {
        *item = itemObj; 
    }

    return kOK; 
}

kFx(kStatus) kList_Insert(kList list, kListItem before, const void* itemContent, kListItem* item)
{
    kObj(kList, list);   
    kListItemStruct* itemObj = kNULL; 
    kListItemStruct* beforeObj = (kListItemStruct*) before; 
    void* contentSlot = kNULL; 

    if (kIsNull(before))
    {
        return kList_Add(list, itemContent, item); 
    }

    if (obj->count == obj->capacity)
    {
        kCheck(kList_Reserve(list, obj->capacity+1)); 
    }

    itemObj = obj->free; 
    obj->free = itemObj->next; 

    itemObj->next = beforeObj; 
    itemObj->previous = beforeObj->previous; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj; 
    }

    beforeObj->previous = itemObj; 

    if (obj->first == beforeObj)
    {
        obj->first = itemObj; 
    }

    obj->count++; 

    contentSlot = (void*)xkList_ItemContent(list, itemObj); 

    if (!kIsNull(itemContent))
    {
        kValue_Import(obj->contentField.type, contentSlot, itemContent); 
    }

    if (!kIsNull(item))
    {
        *item = itemObj; 
    }
    
    return kOK; 
}

kFx(kStatus) kList_Remove(kList list, kListItem item)
{
    kObj(kList, list);   
    kListItemStruct* itemObj = (kListItemStruct*) item; 

    if (!kIsNull(itemObj->previous))
    {
        itemObj->previous->next = itemObj->next; 
    }
    
    if (!kIsNull(itemObj->next))
    {
        itemObj->next->previous = itemObj->previous; 
    }

    if (obj->first == itemObj)
    {
        obj->first = itemObj->next; 
    }

    if (obj->last == itemObj)
    {
        obj->last = itemObj->previous; 
    }

    itemObj->next = obj->free; 
    itemObj->previous = kNULL;
    obj->free = itemObj; 

    obj->count--; 
    
    return kOK; 
}

kFx(kStatus) kList_Clear(kList list)
{
    kObj(kList, list);   

    if (obj->count > 0)
    {
        obj->last->next = obj->free; 
        obj->free = obj->first; 

        obj->first = kNULL; 
        obj->last = kNULL; 

        obj->count = 0; 
    }

    return kOK; 
}

kFx(kStatus) kList_Purge(kList list)
{    
    kObj(kList, list);   

    if (!kIsNull(obj->contentField.type) && xkList_ItemIsRef(list))
    {
        kListItem item = kList_First(list); 

        while (!kIsNull(item))
        {
            kCheck(kObject_Dispose(kList_AsT(list, item, kObject))); 

            item = kList_Next(list, item); 
        }
    }

    kCheck(kList_Clear(list)); 

    return kOK; 
}

kFx(kStatus) kList_SetItem(kList list, kListItem item, const void* content)
{
    kObj(kList, list);   
    void* contentSlot = xkList_ItemContent(list, item); 

    kValue_Import(obj->contentField.type, contentSlot, content); 

    return kOK; 
}

kFx(kStatus) kList_Item(kList list, kListItem item, void* content) 
{
    kObj(kList, list);   
    void* contentSlot = xkList_ItemContent(list, item); 

    kItemCopy(content, contentSlot, obj->contentField.fieldSize); 

    return kOK; 
}

kFx(kListItem) kList_FindIndex(kList list, kSize index)
{
    kObj(kList, list);   

    if (index > obj->count)
    {
        return kNULL; 
    }
    else
    {
        kListItem it = kList_First(list); 
        kSize i; 

        for (i = 0; i < index; ++i)
        {
            it = kList_Next(list, it); 
        }

        return it; 
    }
}


kFx(void*) kList_AtIndex(kList list, kSize index)
{
    kListItem item = kList_FindIndex(list, index); 

    return kIsNull(item) ? kNULL : kList_At(list, item); 
}
