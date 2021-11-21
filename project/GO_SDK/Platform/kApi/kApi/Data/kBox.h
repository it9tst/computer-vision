/** 
 * @file    kBox.h
 * @brief   Declares the kBox class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_BOX_H
#define K_API_BOX_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kBox.x.h>

/**
 * @class   kBox
 * @extends kObject
 * @ingroup kApi-Data
 * @brief   Represents an instance of a value type as an object. 
 * 
 * kBox provides a simple way to represent a single value (e.g. k32s) using a kObject-derived class instance. 
 * This approach can sometimes be used to reduce duplicated effort when both reference types and value types
 * must be supported.
 * 
 * kBox supports the kObject_Clone and kObject_Size methods.
 *
 * kBox supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kBox;    --forward-declared in kApiDef.x.h 

/** 
 * Constructs a kBox object.
 *
 * @public              @memberof kBox
 * @param   box         Receives the constructed object.  
 * @param   itemType    Type of box element (value types only). 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Construct(kBox* box, kType itemType, kAlloc allocator);

/** 
 * Reallocates the internal box item buffer. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   itemType    Type of box element.
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Allocate(kBox box, kType itemType);

/** 
 * Copies the value contained within the source box into this box. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   source      Source box to be copied. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Assign(kBox box, kBox source); 

/** 
 * Sets all box element bits to zero.
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Zero(kBox box); 

/** 
 * Sets the box value. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   item        Pointer to item that will be copied into the box.
 * @return              Operation status. 
 */
kFx(kStatus) kBox_SetItem(kBox box, const void* item); 

/** 
 * Sets the box value. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates             kBox
 * @param   kBox_box    Box object. 
 * @param   TPtr_item   Strongly-typed pointer to item that will be copied into the box.
 * @return              Operation status. 
 */
#define kBox_SetItemT(kBox_box, TPtr_item) \
    xkBox_SetItemT(kBox_box, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Gets the box value. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @param   item        Destination for value copied from the box. 
 * @return              Operation status. 
 */
kFx(kStatus) kBox_Item(kBox box, void* item); 

/** 
* Sets the box value.
* 
* A debug assertion will be raised if the size of the specified data type does not match the size of the box type.
*
* @relates                  kBox
* @param   kBox_box         Box object. 
* @param   T_value          Item value.  
* @param   T                Item type identifier (e.g., k32s).
* @return                   Item value.
*/
#define kBox_SetAsT(kBox_box, T_value, T) \
    (kPointer_WriteAs(xkBox_AsT(kBox_box, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the box value.
* 
* A debug assertion will be raised if the size of the specified data type does not match the size of the box type.
*
* @relates                  kBox
* @param   kBox_box         Box object. 
* @param   T                Item type identifier (e.g., k32s).
* @return                   Item value.
*/
#define kBox_AsT(kBox_box, T) \
    kPointer_ReadAs(xkBox_AsT(kBox_box, sizeof(T)), T)

/** 
 * Gets the box value. 
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates             kBox
 * @param   kBox_box    Box object. 
 * @param   TPtr_item   Strongly-typed destination pointer for value copied from the box. 
 * @return              Operation status. 
 */
#define kBox_ItemT(kBox_box, TPtr_item) \
    xkBox_ItemT(kBox_box, TPtr_item, sizeof(*(TPtr_item)))

/** 
 * Returns a pointer to the box item buffer.  
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Pointer to box item buffer. 
 */
kInlineFx(void*) kBox_Data(kBox box)
{
    kObj(kBox, box); 

    return obj->item;
}

/** 
 * Returns a strongly-typed pointer to the box item buffer.  
 *
 * A debug assertion will be raised if the size of the specified item type is not equal to the 
 * size of the collection item type.
 * 
 * @relates             kBox
 * @param   kBox_box    Box object. 
 * @param   T           Item type identifier (e.g., k32s).
 * @return              Strongly-typed pointer to box item buffer. 
 */
#define kBox_DataT(kBox_box, T) \
    kCast(T*, xkBox_DataT(kBox_box, sizeof(T)))

/** 
 * Returns the box item type. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Box item type. 
 */
kInlineFx(kType) kBox_ItemType(kBox box)
{
    kObj(kBox, box); 

    return obj->itemType; 
}

/** 
 * Returns the box item size. 
 *
 * @public              @memberof kBox
 * @param   box         Box object. 
 * @return              Box item size. 
 */
kInlineFx(kSize) kBox_ItemSize(kBox box)
{
    kObj(kBox, box); 

    return obj->itemSize;
}

#endif
