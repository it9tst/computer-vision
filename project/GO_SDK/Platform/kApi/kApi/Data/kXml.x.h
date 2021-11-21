/** 
 * @file    kXml.x.h
 *
 * @internal
 * Copyright (C) 2004-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_XML_X_H
#define K_API_XML_X_H

#define xkXML_FILE_READ_BUFFER_SIZE      (16384)
#define xkXML_FILE_WRITE_BUFFER_SIZE     (16384)
#define xkXML_FORMAT_BUFFER_SIZE         (256)
#define xkXML_STRING_INIT_SIZE           (100*1024)
#define xkXML_DEFAULT_TEXT_SIZE          (64)
#define xkXML_DEFAULT_ITEM_BLOCK_COUNT   (128)
#define xkXML_DEFAULT_ATTR_BLOCK_COUNT   (128)

#define xkXML_PATH_DELIMITERS            "\\/"
#define xkXML_DEFAULT_HEADER             "?xml version=\"1.0\" encoding=\"UTF-8\"?"
#define xkXML_NEW_LINE                   "\r\n"
#define xkXML_INDENTATION                "  "

typedef struct
{
    kChar text[xkXML_DEFAULT_TEXT_SIZE];
    kChar* buffer;
    kSize capacity;
} xkXmlTextField;

typedef struct xkXmlAttrClassTag xkXmlAttrClass;
typedef struct xkXmlItemClassTag xkXmlItemClass;

struct xkXmlAttrClassTag
{
    xkXmlTextField name;
    xkXmlTextField value;
    xkXmlAttrClass* next;
};

struct xkXmlItemClassTag
{
    xkXmlTextField name;
    xkXmlTextField value;

    xkXmlAttrClass* firstAttr;
    xkXmlAttrClass* lastAttr;

    xkXmlItemClass* firstChild;
    xkXmlItemClass* lastChild;

    xkXmlItemClass* parent;
    xkXmlItemClass* prev;
    xkXmlItemClass* next;
};

typedef struct
{
    xkXmlItemClass* items;
} xkXmlItemBlock;

kDeclareValueEx(k, xkXmlItemBlock, kValue)

typedef struct
{
    xkXmlAttrClass* attrs;
} xkXmlAttrBlock;

kDeclareValueEx(k, xkXmlAttrBlock, kValue)

typedef enum
{
    TAG_OPEN,
    TAG_CLOSE,
    TAG_EMPTY
} xkXmlTagType;

typedef struct
{
    kXml xml;
    xkXmlItemClass* currentItem;
    const kChar* position;
    kBool isStream;
    kChar* buffer;

    // stream info
    kStream stream;
    kSize capacity;
    kSize size;
    kBool eof;
} xkXmlParseContext;

typedef struct kXmlClass
{
    kObjectClass base; 
    
    xkXmlItemClass* root;

    kSize itemBlockCount;
    kArrayList itemBlocks;
    xkXmlItemClass* freeItems;

    kSize attrBlockCount;
    kArrayList attrBlocks;
    xkXmlAttrClass* freeAttrs;

    kChar formatBuffer[xkXML_FORMAT_BUFFER_SIZE];
    kArrayList pathParseBuffer;
} kXmlClass; 

kDeclareClassEx(k, kXml, kObject)

kFx(kStatus) xkXml_VRelease(kXml xml); 

kFx(kStatus) xkXml_WriteDat6V0(kXml xml, kSerializer serializer); 
kFx(kStatus) xkXml_ReadDat6V0(kXml xml, kSerializer serializer); 

kStatus xkXml_AllocItemBlock(kXml xml);
kStatus xkXml_AllocAttrBlock(kXml xml);
kStatus xkXml_FreeItemBlocks(kXml xml);
kStatus xkXml_FreeAttrBlocks(kXml xml);

kStatus xkXml_AllocItem(kXml xml, xkXmlItemClass** item);
kStatus xkXml_FreeItem(kXml xml, xkXmlItemClass* item);
kStatus xkXml_AllocAttr(kXml xml, xkXmlAttrClass** attr);
kStatus xkXml_FreeAttr(kXml xml, xkXmlAttrClass* attr);

kStatus xkXml_LinkItemUnder(kXml xml, xkXmlItemClass* parent, xkXmlItemClass* item);
kStatus xkXml_LinkItemBefore(kXml xml, xkXmlItemClass* next, xkXmlItemClass* item);
kStatus xkXml_LinkItemAfter(kXml xml, xkXmlItemClass* prev, xkXmlItemClass* item);
kStatus xkXml_UnlinkItem(kXml xml, xkXmlItemClass* item);
kStatus xkXml_LinkAttr(kXml xml, xkXmlItemClass* item, xkXmlAttrClass* attr);

xkXmlAttrClass* xkXml_FindItemAttributeObject(kXml xml, kXmlItem item, const kChar *name);
xkXmlItemClass* xkXml_FindChildByName(kXml xml, xkXmlItemClass* parent, const kChar* name, kSize index);
xkXmlAttrClass* xkXml_FindAttrByName(kXml xml, xkXmlItemClass* item, const kChar* name);

kStatus xkXml_ItemToString(kXml xml, kString str, xkXmlItemClass* item, kSize level);
const kChar* xkXml_ItemValue(kXml xml, kXmlItem item);
const kChar* xkXml_AttrValue(kXml xml, kXmlItem item, const kChar* name);

kStatus xkXml_TextFieldInit(kXml xml, xkXmlTextField* field);
kStatus xkXml_TextFieldRelease(kXml xml, xkXmlTextField* field);
kStatus xkXml_TextFieldReserve(kXml xml, xkXmlTextField* field, kSize size);
kStatus xkXml_TextFieldSetString(kXml xml, xkXmlTextField* field, const kChar* str);
kChar* xkXml_TextFieldString(kXml xml, xkXmlTextField* field);

const kChar* xkXml_Format16u(kXml xml, k16u val);
const kChar* xkXml_Format16s(kXml xml, k16s val);
const kChar* xkXml_Format32u(kXml xml, k32u val);
const kChar* xkXml_Format32s(kXml xml, k32s val);
const kChar* xkXml_FormatBool(kXml xml, kBool val);
const kChar* xkXml_Format64u(kXml xml, k64u val);
const kChar* xkXml_Format64s(kXml xml, k64s val);
const kChar* xkXml_FormatSize(kXml xml, kSize val);
const kChar* xkXml_Format32f(kXml xml, k32f val);
const kChar* xkXml_Format64f(kXml xml, k64f val);
kStatus xkXml_Parse16u(kXml xml, const kChar *string, k16u *val);
kStatus xkXml_Parse16s(kXml xml, const kChar *string, k16s *val);
kStatus xkXml_Parse32u(kXml xml, const kChar *string, k32u *val);
kStatus xkXml_Parse32s(kXml xml, const kChar *string, k32s *val);
kStatus xkXml_ParseBool(kXml xml, const kChar *string, kBool *val);
kStatus xkXml_Parse64u(kXml xml, const kChar *string, k64u *val);
kStatus xkXml_Parse64s(kXml xml, const kChar *string, k64s *val);
kStatus xkXml_ParseSize(kXml xml, const kChar *string, kSize *val);
kStatus xkXml_Parse32f(kXml xml, const kChar *string, k32f *val);
kStatus xkXml_Parse64f(kXml xml, const kChar *string, k64f *val);

kStatus xkXml_Substr(kXml xml, const kChar* input, kSize begin, kSize end, xkXmlTextField* str);
kStatus xkXml_UnescapeSubstr(kXml xml, const kChar* input, kSize begin, kSize end, xkXmlTextField* str);
const kChar* xkXml_FindDelimiter(const kChar* str, const kChar* delims);
kSize xkXml_ParseNameSubstrIndex(const kChar* str, kSize size, kSize* adjustedSize);

kStatus xkXml_SetPathParseBuffer(kXml xml, const kChar* text, kSize size);
const kChar* xkXml_PathParseBuffer(kXml xml);

kBool xkXml_ValidateName(const kChar *name);
kBool xkXml_IsNameChar(kChar ch);
kStatus xkXml_AddEscapedString(kXml xml, const kChar* in, kString out);

// Parser code

kStatus xkXml_ParseAttributes(kXml xml, xkXmlParseContext* context);
kStatus xkXml_ParseTag(kXml xml, xkXmlParseContext* context, xkXmlTagType* tagType);
kStatus xkXml_ParseComment(kXml xml, xkXmlParseContext* context);
kStatus xkXml_ParseDeclaration(kXml xml, xkXmlParseContext* context);
kStatus xkXml_Parse(kXml xml, xkXmlParseContext* context);
kStatus xkXml_AdvanceParser(xkXmlParseContext* context);
kStatus xkXml_SkipWhitespace(xkXmlParseContext* context);
kStatus xkXml_SkipToken(xkXmlParseContext* context);

kFx(kStatus) xkXml_Init(kXml xml, kAlloc allocator);
kFx(kStatus) xkXml_VClone(kXml xml, kXml source, kAlloc valueAlloc, kObject context);

kInlineFx(xkXmlItemClass*) xkXmlItem_Cast(kXmlItem item)
{
    return kCast(xkXmlItemClass*, item);
}

#endif
