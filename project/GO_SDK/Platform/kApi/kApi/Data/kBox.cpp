/** 
 * @file    kBox.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kBox.h>
#include <kApi/Data/kCollection.h>
#include <kApi/Io/kSerializer.h>

kBeginClassEx(k, kBox) 
    
    //serialization versions
    kAddPrivateVersionEx(kBox, "kdat5", "5.0.0.0", "29-1", WriteDat5V1, ReadDat5V1)
    kAddPrivateVersionEx(kBox, "kdat6", "5.7.1.0", "kBox-0", WriteDat6V0, ReadDat6V0)

    //special constructors
    kAddPrivateFrameworkConstructor(kBox, ConstructFramework)

    //virtual methods
    kAddPrivateVMethod(kBox, kObject, VRelease)
    kAddPrivateVMethod(kBox, kObject, VClone)
    kAddPrivateVMethod(kBox, kObject, VSize)

kEndClassEx() 

kFx(kStatus) kBox_Construct(kBox* box, kType itemType, kObject allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, kTypeOf(kBox), box)); 

    if (!kSuccess(status = xkBox_Init(*box, kTypeOf(kBox), itemType, alloc)))
    {
        kAlloc_FreeRef(alloc, box); 
    }

    return status; 
} 

kFx(kStatus) xkBox_ConstructFramework(kBox* box, kAlloc allocator)
{
    return kBox_Construct(box, kNULL, allocator);
}

kFx(kStatus) xkBox_Init(kBox box, kType classType, kType itemType, kAlloc alloc)
{
    kObjR(kBox, box);
    kStatus status = kOK; 

    kCheck(kObject_Init(box, classType, alloc)); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 
    obj->allocSize = 0; 
    obj->item = kNULL;

    kTry
    {
        kTest(xkBox_Realloc(box)); 
    }
    kCatch(&status)
    {
        xkBox_VRelease(box); 
        kEndCatch(status); 
    }

    return kOK; 
}

kFx(kStatus) xkBox_VClone(kBox box, kBox source, kAlloc valueAlloc, kObject context)
{
    kObj(kBox, box);

    obj->itemType = kBox_ItemType(source); 
    obj->itemSize = kBox_ItemSize(source); 

    kCheck(xkBox_Realloc(box)); 

    kItemCopy(obj->item, kBox_Data(source), obj->itemSize); 

    return kOK; 
}

kFx(kStatus) xkBox_VRelease(kBox box)
{
    kObj(kBox, box); 

    kCheck(kAlloc_Free(kObject_Alloc(box), obj->item)); 
    
    kCheck(kObject_VRelease(box)); 

    return kOK; 
}

kFx(kStatus) xkBox_WriteDat5V1(kBox box, kSerializer serializer)
{
    kObj(kBox, box); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteSize(serializer, 1)); 
    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->item, 1)); 

    return kOK; 
}

kFx(kStatus) xkBox_ReadDat5V1(kBox box, kSerializer serializer)
{
    kObj(kBox, box);
    kSize length; 
    kTypeVersion itemVersion;
    kType itemType = kNULL;            
 
    kCheck(kSerializer_ReadSize(serializer, &length));
    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion)); 

    kCheck(kBox_Allocate(box, itemType)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->item, 1));

    return kOK; 
}

kFx(kStatus) xkBox_WriteDat6V0(kBox box, kSerializer serializer)
{
    kObj(kBox, box); 
    kTypeVersion itemVersion; 

    kCheck(kSerializer_WriteType(serializer, obj->itemType, &itemVersion));
    kCheck(kSerializer_WriteItems(serializer, obj->itemType, itemVersion, obj->item, 1)); 

    return kOK; 
}

kFx(kStatus) xkBox_ReadDat6V0(kBox box, kSerializer serializer)
{
    kObj(kBox, box);
    kType itemType = kNULL;            
    kTypeVersion itemVersion; 

    kCheck(kSerializer_ReadType(serializer, &itemType, &itemVersion));

    kCheck(kBox_Allocate(box, itemType)); 

    kCheck(kSerializer_ReadItems(serializer, itemType, itemVersion, obj->item, 1)); 

    return kOK; 
}

kFx(kStatus) xkBox_Realloc(kBox box)
{
    kObj(kBox, box); 
    kSize newSize = kMax_(obj->allocSize, obj->itemSize); 

    if (newSize > obj->allocSize)
    {
        void* oldItem = obj->item; 
        void* newItem = kNULL; 

        kCheck(kAlloc_Get(kObject_Alloc(box), newSize, &newItem));
        
        obj->item = newItem; 
        obj->allocSize = newSize; 

        kCheck(kAlloc_Free(kObject_Alloc(box), oldItem));
    }
    
    return kOK; 
}

kFx(kStatus) kBox_Allocate(kBox box, kType itemType)
{
    kObj(kBox, box); 

    obj->itemType = kIsNull(itemType) ? kTypeOf(kVoid) : itemType; 
    obj->itemSize = kType_Size(obj->itemType); 

    kCheck(xkBox_Realloc(box)); 

    return kOK; 
}

kFx(kStatus) kBox_Assign(kBox box, kBox source)
{
    kObj(kBox, box); 
    kObjN(kBox, sourceObj, source);  

    kCheck(kBox_Allocate(box, sourceObj->itemType)); 

    kItemCopy(obj->item, sourceObj->item, obj->itemSize); 

    return kOK;   
}

kFx(kStatus) kBox_Zero(kBox box)
{
    kObj(kBox, box); 

    kMemZero(obj->item, obj->itemSize); 

    return kOK; 
}

kFx(kStatus) kBox_SetItem(kBox box, const void* item)
{
    kObj(kBox, box); 

    kValue_Import(obj->itemType, obj->item, item); 

    return kOK; 
}

kFx(kStatus) kBox_Item(kBox box, void* item)
{
    kObj(kBox, box); 

    kItemCopy(item, obj->item, obj->itemSize); 

    return kOK; 
}

