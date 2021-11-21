/** 
 * @file    kBox.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BOX_X_H
#define K_API_BOX_X_H

typedef struct kBoxClass
{
    kObjectClass base; 
    kType itemType;             //item type
    kSize itemSize;             //item size, in bytes
    kSize allocSize;            //size of allocated array memory, in bytes
    void* item;                 //array memory 
} kBoxClass;
    
kDeclareClassEx(k, kBox, kObject) 

/* 
* Forward declarations. 
*/

kFx(kStatus) kBox_SetItem(kBox box, const void* item); 
kFx(kStatus) kBox_Item(kBox box, void* item); 
kInlineFx(kType) kBox_ItemType(kBox box);
kInlineFx(kSize) kBox_ItemSize(kBox box);
kInlineFx(void*) kBox_Data(kBox box);

/* 
* Private methods. 
*/

kFx(kStatus) xkBox_ConstructFramework(kBox* box, kAlloc allocator); 

kFx(kStatus) xkBox_Init(kBox box, kType classType, kType itemType, kAlloc alloc);
kFx(kStatus) xkBox_VClone(kBox box, kBox source, kAlloc valueAlloc, kObject context); 

kFx(kStatus) xkBox_VRelease(kBox box); 

kInlineFx(kSize) xkBox_VSize(kBox box)
{
    kObj(kBox, box); 
    kSize size = sizeof(kBoxClass) + obj->allocSize; 

    return size; 
}

kFx(kStatus) xkBox_WriteDat5V1(kBox box, kSerializer serializer); 
kFx(kStatus) xkBox_ReadDat5V1(kBox box, kSerializer serializer); 
kFx(kStatus) xkBox_WriteDat6V0(kBox box, kSerializer serializer); 
kFx(kStatus) xkBox_ReadDat6V0(kBox box, kSerializer serializer); 

kFx(kStatus) xkBox_Realloc(kBox box); 


kInlineFx(kStatus) xkBox_SetItemT(kBox box, const void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kBox_ItemType(box), itemSize)); 

    return kBox_SetItem(box, item);
} 

kInlineFx(kStatus) xkBox_ItemT(kBox box, void* item, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kBox_ItemType(box), itemSize)); 

    return kBox_Item(box, item);
} 

kInlineFx(kSize) xkBox_ItemSizeT(kBox box, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kBox_ItemType(box), itemSize)); 

    return kBox_ItemSize(box);
} 

kInlineFx(void*) xkBox_DataT(kBox box, kSize itemSize)
{
    kAssert(xkType_IsPointerCompatible(kBox_ItemType(box), itemSize)); 

    return kBox_Data(box);
} 

kInlineFx(void*) xkBox_AsT(kBox box, kSize itemSize)
{
    kAssert(itemSize == kBox_ItemSize(box)); 

    return kBox_Data(box);
}

#endif
