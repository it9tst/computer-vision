//
// KXml.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_XML_H
#define K_API_NET_XML_H

#include <kApi/Data/kXml.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents an XML element within an XML document.</summary>           
            public ref struct KXmlElement
            {          
            public:

                /// <summary>Gets the parent element.</summary>
                property KXmlElement^ Parent
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(m_root, m_xml, kXml_Parent(m_xml, m_element)); }
                }

                /// <summary>Gets the first child element.</summary>
                property KXmlElement^ FirstChild
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(m_root, m_xml, kXml_FirstChild(m_xml, m_element)); }
                }

                /// <summary>Gets the last child element.</summary>
                property KXmlElement^ LastChild
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(m_root, m_xml, kXml_LastChild(m_xml, m_element)); }
                }

                /// <summary>Gets the next sibling element.</summary>
                property KXmlElement^ NextSibling
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(m_root, m_xml, kXml_NextSibling(m_xml, m_element)); }
                }

                /// <summary>Gets the previous sibling element.</summary>
                property KXmlElement^ PreviousSibling
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(m_root, m_xml, kXml_PreviousSibling(m_xml, m_element)); }
                }

                /// <summary>Gets the number of child elements.</summary>
                property k64s ChildCount
                {
                    k64s get() { return (k64s) kXml_ChildCount(m_xml, m_element); }
                }

                /// <summary>Gets the child element at the specified index within the list of child elements. </summary>
                /// 
                /// <param name="index">Child element index.</param>
                /// <returns>Child element.</returns>
                KXmlElement^ GetChildAt(k64s index)
                {
                    return KXmlElement::ToObject(m_root, m_xml, kXml_ChildAt(m_xml, m_element, (kSize)index)); 
                }

                /// <summary>Gets or sets the element name.</summary>
                property String^ Name
                {
                    String^ get() 
                    { 
                        return KToString(kXml_ItemName(m_xml, m_element));  
                    }

                    void set(String^ name)
                    {
                        KString str(name); 

                        KCheck(kXml_SetItemName(m_xml, m_element, (const kChar*)%str)); 
                    }
                }
         
                /// <summary>Gets element content as a text string.</summary>
                /// <returns>Element value.</returns>
                String^ GetString()
                {
                    KString str;

                    KCheck(kXml_ItemString(m_xml, m_element, KToHandle(%str)));

                    return str.ToString();
                }
                
                /// <summary>Gets element content as a UInt16 value.</summary>
                /// <returns>Element value.</returns>
                k16u Get16u()
                {
                    k16u value; 

                    KCheck(kXml_Item16u(m_xml, m_element, &value)); 

                    return value;
                }

                /// <summary>Gets element content as an Int16 value.</summary>
                /// <returns>Element value.</returns>
                k16s Get16s()
                {
                    k16s value;

                    KCheck(kXml_Item16s(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Gets element content as a UInt32 value.</summary>
                /// <returns>Element value.</returns>
                k32u Get32u()
                {
                    k32u value;

                    KCheck(kXml_Item32u(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Gets element content as an Int32 value.</summary>
                /// <returns>Element value.</returns>
                k32s Get32s()
                {
                    k32s value;

                    KCheck(kXml_Item32s(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Gets element content as a UInt64 value.</summary>
                /// <returns>Element value.</returns>
                k64u Get64u()
                {
                    k64u value;

                    KCheck(kXml_Item64u(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Gets element content as an Int64 value.</summary>
                /// <returns>Element value.</returns>
                k64s Get64s()
                {
                    k64s value;

                    KCheck(kXml_Item64s(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Gets element content as a Bool value.</summary>
                /// <returns>Element value.</returns>
                bool GetBool()
                {
                    kBool value;
                    
                    KCheck(kXml_ItemBool(m_xml, m_element, &value));

                    return KToBool(value); 
                }

                /// <summary>Gets element content as a Single value.</summary>
                /// <returns>Element value.</returns>
                k32f Get32f()
                {
                    k32f value;

                    KCheck(kXml_Item32f(m_xml, m_element, &value));

                    return value; 
                }

                /// <summary>Gets element content as a Double value.</summary>
                /// <returns>Element value.</returns>
                k64f Get64f()
                {
                    k64f value;

                    KCheck(kXml_Item64f(m_xml, m_element, &value));

                    return value;
                }

                /// <summary>Sets element content to a string value.</summary>
                /// <param name="value">Element value.</param>
                void SetString(String^ value)
                {
                    KString str;

                    str.Set(value);

                    KCheck(kXml_SetItemText(m_xml, m_element, (const kChar*) % str));
                }

                /// <summary>Sets element content to a UInt16 value.</summary>
                /// <param name="value">Element value.</param>
                void Set16u(k16u value)
                {
                    KCheck(kXml_SetItem16u(m_xml, m_element, value));
                }

                /// <summary>Sets element content to an Int32 value.</summary>
                /// <param name="value">Element value.</param>
                void Set16s(k16s value)
                {
                    KCheck(kXml_SetItem16s(m_xml, m_element, value));
                }

                /// <summary>Sets element content to a UInt32 value.</summary>
                /// <param name="value">Element value.</param>
                void Set32u(k32u value)
                {
                    KCheck(kXml_SetItem32u(m_xml, m_element, value));
                }

                /// <summary>Sets element content to an Int32 value.</summary>
                /// <param name="value">Element value.</param>
                void Set32s(k32s value)
                {
                    KCheck(kXml_SetItem32s(m_xml, m_element, value));
                }

                /// <summary>Sets element content to a UInt64 value.</summary>
                /// <param name="value">Element value.</param>
                void Set64u(k64u value)
                {
                    KCheck(kXml_SetItem64u(m_xml, m_element, value));
                }

                /// <summary>Sets element content to an Int32 value.</summary>
                /// <param name="value">Element value.</param>
                void Set64s(k64s value)
                {
                    KCheck(kXml_SetItem64s(m_xml, m_element, value));
                }

                /// <summary>Sets element content to a Bool value.</summary>
                /// <param name="value">Element value.</param>
                void SetBool(bool value)
                {
                    KCheck(kXml_SetItemBool(m_xml, m_element, value));
                }

                /// <summary>Sets element content to a Single value.</summary>
                /// <param name="value">Element value.</param>
                void Set32f(k32f value)
                {
                    KCheck(kXml_SetItem32f(m_xml, m_element, value));
                }

                /// <summary>Sets element content to a Double value.</summary>
                /// <param name="value">Element value.</param>
                void Set64f(k64f value)
                {
                    KCheck(kXml_SetItem64f(m_xml, m_element, value));
                }

                /// <summary>Reports whether an attribute exists.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>true if attribute exists; otherwise false.</returns>
                bool AttrExists(String^ name)
                {
                    KString nameStr(name); 

                    return KToBool(kXml_AttrExists(m_xml, m_element, (kChar*) % nameStr));
                }

                /// <summary>Gets the number of attributes.</summary>
                property k64s AttrCount
                {
                    k64s get() { return (k64s)kXml_AttrCount(m_xml, m_element); }
                }

                /// <summary>Gets the attribute name at the specified index within the attribute list.</summary>
                ///
                /// <param name="index">Attribute index.</param>
                /// <returns>Attribute name.</returns>
                String^ GetAttrNameAt(k64s index)
                {
                    return KToString(kXml_AttrNameAt(m_xml, m_element, (kSize)index));
                }

                /// <summary>Deletes an attribute.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                void DeleteAttr(String^ name)
                {
                    KString nameStr(name); 

                    KCheck(kXml_DeleteAttr(m_xml, m_element, (kChar*) % nameStr));
                }

                /// <summary>Deletes all attributes.</summary>
                void DeleteAttrs()
                {
                    KCheck(kXml_DeleteAttrs(m_xml, m_element));
                }
            
                /// <summary>Gets attribute content as a string.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attibute value.</returns>
                String^ GetAttrString(String^ name)
                {
                    KString nameStr(name); 
                    KString valueStr; 

                    KCheck(kXml_AttrString(m_xml, m_element, (kChar*) % nameStr, valueStr.ToHandle().ToPointer()));

                    return valueStr.ToString(); 
                }
             
                /// <summary>Gets attribute content as a UInt16 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k16u GetAttr16u(String^ name)
                {
                    KString nameStr(name); 
                    k16u value; 

                    KCheck(kXml_Attr16u(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as an Int16 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k16s GetAttr16s(String^ name)
                {
                    KString nameStr(name);
                    k16s value;

                    KCheck(kXml_Attr16s(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as a UInt32 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k32u GetAttr32u(String^ name)
                {
                    KString nameStr(name);
                    k32u value;

                    KCheck(kXml_Attr32u(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as an Int32 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k32s GetAttr32s(String^ name)
                {
                    KString nameStr(name);
                    k32s value;

                    KCheck(kXml_Attr32s(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as a UInt64 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k64u GetAttr64u(String^ name)
                {
                    KString nameStr(name);
                    k64u value;

                    KCheck(kXml_Attr64u(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as an Int64 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k64s GetAttr64s(String^ name)
                {
                    KString nameStr(name);
                    k64s value;

                    KCheck(kXml_Attr64s(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as a Bool value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                bool GetAttrBool(String^ name)
                {
                    KString nameStr(name);
                    kBool value;

                    KCheck(kXml_AttrBool(m_xml, m_element, nameStr.CharPtr, &value));

                    return KToBool(value);
                }

                /// <summary>Gets attribute content as a Single value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k32f GetAttr32f(String^ name)
                {
                    KString nameStr(name);
                    k32f value;

                    KCheck(kXml_Attr32f(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets attribute content as a Double value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <returns>Attribute value.</returns>
                k64f GetAttr64f(String^ name)
                {
                    KString nameStr(name);
                    k64f value;

                    KCheck(kXml_Attr64f(m_xml, m_element, nameStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Sets attribute content to a string value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttrString(String^ name, String^ value)
                {
                    KString nameStr(name);
                    KString valueStr(value);

                    KCheck(kXml_SetAttrText(m_xml, m_element, nameStr.CharPtr, valueStr.CharPtr));
                }

                /// <summary>Sets attribute content to a UInt16 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr16u(String^ name, k16u value)
                {
                    KString nameStr(name);
                
                    KCheck(kXml_SetAttr16u(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to an Int16 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr16s(String^ name, k16s value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr16s(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to a UInt32 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr32u(String^ name, k32u value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr32u(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to an Int32 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr32s(String^ name, k32s value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr32s(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to a UInt64 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr64u(String^ name, k64u value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr64u(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to an Int64 value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr64s(String^ name, k64s value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr64s(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to a Bool value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttrBool(String^ name, bool value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttrBool(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to a Single value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr32f(String^ name, k32f value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr32f(m_xml, m_element, nameStr.CharPtr, value));
                }

                /// <summary>Sets attribute content to a Double value.</summary>
                ///
                /// <param name="name">Attribute name.</param>
                /// <param name="value">Attribute value.</param>
                void SetAttr64f(String^ name, k64f value)
                {
                    KString nameStr(name);

                    KCheck(kXml_SetAttr64f(m_xml, m_element, nameStr.CharPtr, value));
                }

                virtual bool Equals(Object^ other) override
                {
                    KXmlElement^ element = dynamic_cast<KXmlElement^>(other);

                    if (element != nullptr)
                    {
                        return (element->m_xml == this->m_xml) && (element->m_element == this->m_element);
                    }

                    return false;
                }

            internal:

                KXmlElement(Object^ root, kXml xml, kXmlItem element)
                    : m_root(root), m_xml(xml), m_element(element) {}

                static kXmlItem ToHandle(KXmlElement^ it)
                {
                    return (it == nullptr) ? kNULL : it->m_element;
                }

                static KXmlElement^ ToObject(Object^ root, kXml xml, kXmlItem it)
                {
                    return (it == kNULL) ? nullptr : gcnew KXmlElement(root, xml, it);
                }

                Object^ m_root;               //prevents finalization of document
                kXml m_xml; 
                kXmlItem m_element; 
            };      

            /// <summary>Represents an XML document.</summary>
            /// 
            /// <remarks>
            /// <para>KXml^ supports the KObject.Clone method.</para>
            /// 
            /// <para>KXml^ supports the kdat6 serialization protocol.</para>
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KXml : public KObject
            {
                KDeclareAutoClass(KXml, kXml)

            public:
                /// <summary>Initializes a new instance of the KXml class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KXml(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KXml(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KXml(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KXml class.</summary>           
                /// 
                /// The KXml object is initially empty; use KXml.AddItem with parent=null to create a root element.
                KXml()
                    : KObject(DefaultRefStyle)
                {
                    kXml handle = kNULL;

                    KCheck(kXml_Construct(&handle, kNULL));

                    Handle = handle;
                }


                /// <inheritdoc cref="KXml()" />
                /// <param name="allocator">Memory allocator.</param>
                KXml(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kXml handle = kNULL;

                    KCheck(kXml_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <inheritdoc cref="KXml(KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KXml(KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kXml handle = kNULL;

                    KCheck(kXml_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Loads an XML document from file.</summary>
                /// 
                /// <param name="fileName">Path to file.</param>
                /// <returns>Loaded document object.</returns>
                static KXml^ Load(String^ fileName)
                {
                    return Load(fileName, nullptr);
                }

                /// <summary>Loads an XML document from file.</summary>
                /// 
                /// <param name="fileName">Path to file.</param>
                /// <param name="allocator">Memory allocator.</param>
                /// <returns>Loaded document object.</returns>
                static KXml^ Load(String^ fileName, KAlloc^ allocator)
                {
                    KString fileStr(fileName);
                    kXml xml = kNULL;

                    KCheck(kXml_Load(&xml, fileStr.CharPtr, KToHandle(allocator)));

                    return gcnew KXml(IntPtr(xml));
                }

                /// <summary>Loads an XML document from a KStream object.</summary>
                /// 
                /// <param name="stream">Stream for reading.</param>
                static KXml^ Load(KStream^ stream, KAlloc^ allocator)
                {
                    KXml^ xml = gcnew KXml(allocator);

                    try
                    {
                        KCheck(kXml_Read(xml->Handle, stream->ToHandle().ToPointer()));

                        return xml;
                    }
                    catch (...)
                    {
                        delete xml;
                        throw;
                    }
                }

                /// <summary>Saves the XML document to file.</summary>
                /// 
                /// <param name="fileName">File path.</param>
                void Save(String^ fileName)
                {
                    KString fileStr(fileName);

                    KCheck(kXml_Save(Handle, fileStr.CharPtr));
                }

                /// <summary>Saves the XML document to a KStream.</summary>
                /// 
                /// <param name="stream">Stream for writing.</param>
                void Save(KStream^ stream)
                {
                    KCheck(kXml_Write(Handle, KToHandle(stream))); 
                }

                /// <summary>Creates an XML document from a string.</summary>
                /// 
                /// <param name="content">XML-formatted string.</param>
                static KXml^ Parse(String^ content)
                {
                    return Parse(content, nullptr); 
                }

                /// <summary>Creates an XML document from a string.</summary>
                /// 
                /// <param name="content">XML-formatted string.</param>
                /// <param name="allocator">Memory allocator.</param>
                static KXml^ Parse(String^ content, KAlloc^ allocator)
                {
                    KString str(content);
                    KXml^ xml = gcnew KXml(allocator); 

                    try
                    {
                        KCheck(kXml_FromString(xml->Handle, str.ToHandle().ToPointer())); 

                        return xml; 
                    }
                    catch (...)
                    {
                        delete xml; 
                        throw; 
                    }
                }

                /// <summary>Writes XML content to a string.</summary>
                /// 
                /// <returns>XML-formatted string.</returns>
                virtual String^ ToString() override
                {
                    KString str;

                    KCheck(kXml_ToString(Handle, str.ToHandle().ToPointer()));

                    return str.ToString(); 
                }
               
                /// <summary>Compacts the XML object for minimum memory usage.</summary>
                /// 
                /// <remarks>Compacting invalidates existing KXmlElement objects.</remarks>
                void Compact()
                {
                    KCheck(kXml_Compact(Handle)); 
                }
           
                /// <summary>Removes all elements from the XML document.</summary>
                void Clear()
                {
                    KCheck(kXml_Clear(Handle));
                }

                /// <summary>Copies the source document.</summary>
                /// 
                /// <param name="source">Source document to be copied.</param>
                void Assign(KXml^ source)
                {
                    KCheck(kXml_Assign(Handle, KToHandle(source)));
                }

                /// <summary>Gets the root element of the XML document.</summary>
                property KXmlElement^ Root
                {
                    KXmlElement^ get() { return KXmlElement::ToObject(this, Handle, kXml_Root(Handle)); }
                }

                /// <summary>Gets the root element, if it exists.</summary>
                /// 
                /// <returns>Root element, if it exists.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                KXmlElement^ FindRoot()
                {
                    kXmlItem root = kNULL; 

                    KCheck(kXml_FindRoot(Handle, &root)); 

                    return KXmlElement::ToObject(this, Handle, root); 
                }

                /// <summary>Ensures that a child element exists at the specified path.</summary>
                ///
                /// <remarks>This method creates any missing elements that are necessary for the path to be complete. 
                /// If a child already exists at the specified path, the existing child is returned.</remarks>
                /// 
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Child element.</returns>
                KXmlElement^ EnsureChildExists(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path); 
                    kXmlItem childItem = kNULL; 

                    KCheck(kXml_EnsureChildExists(Handle, KXmlElement::ToHandle(parent), (kChar*) %pathStr, &childItem)); 

                    return KXmlElement::ToObject(this, Handle, childItem); 
                }

                /// <summary>Deletes all children of the specified parent element.</summary>
                /// 
                /// <param name="parent">Parent element.</param>
                void DeleteChildren(KXmlElement^ parent)
                {
                    KCheck(kXml_DeleteChildren(Handle, KXmlElement::ToHandle(parent))); 
                }

                /// <summary>Adds a new child element to the specified parent element.</summary>
                /// 
                /// <param name="parent">Parent element (null to add root element).</param>
                /// <param name="name">Name of the new element.</param>
                /// <returns>New element</returns>
                KXmlElement^ AddElement(KXmlElement^ parent, String^ name)
                {
                    KString nameStr(name); 
                    kXmlItem item = kNULL; 

                    KCheck(kXml_AddItem(Handle, KXmlElement::ToHandle(parent), (kChar*) %nameStr, &item)); 

                    return KXmlElement::ToObject(this, Handle, item); 
                }

                /// <summary>Inserts a new element before the specified sibling element.</summary>
                /// 
                /// <param name="before">Sibling element.</param>
                /// <param name="name">Name of the new element.</param>
                /// <returns>New element.</returns>
                KXmlElement^ InsertElement(KXmlElement^ before, String^ name)
                {
                    KString nameStr(name);
                    kXmlItem item = kNULL;

                    KCheck(kXml_InsertItem(Handle, KXmlElement::ToHandle(before), nameStr.CharPtr, &item));

                    return KXmlElement::ToObject(this, Handle, item);
                }

                /// <summary>Copies an element from another XML document to this XML document, inserting a new element at the specified location.</summary>
                /// 
                /// <param name="parent">Destination parent element.</param>
                /// <param name="before">Destination sibling element (if null, appends to end of child list).</param>
                /// <param name="srcXml">Source XML document.</param>
                /// <param name="srcElement">Source XML element.</param>
                /// <returns>New element.</returns>
                KXmlElement^ CopyElement(KXmlElement^ parent, KXmlElement^ before, KXml^ srcXml, KXmlElement^ srcElement)
                {
                    kXmlItem item = kNULL;

                    KCheck(kXml_CopyItem(Handle, KXmlElement::ToHandle(parent), KXmlElement::ToHandle(before), KToHandle(srcXml), KXmlElement::ToHandle(srcElement), &item)); 

                    return KXmlElement::ToObject(this, Handle, item); 
                }

                /// <summary>Copies an element from another XML document to this XML document, overwriting an existing element.</summary>
                /// 
                /// <param name="destElement">XML destination element.</param>
                /// <param name="srcXml">Source XML document.</param>
                /// <param name="srcElement">Source XML element.</param>
                void OverwriteElement(KXmlElement^ destElement, KXml^ srcXml, KXmlElement^ srcElement)
                {
                    KCheck(kXml_OverwriteItem(Handle, KXmlElement::ToHandle(destElement), KToHandle(srcXml), KXmlElement::ToHandle(srcElement)));
                }

                /// <summary>Removes all children, attributes, and value from the XML element.</summary>
                /// 
                /// <param name="element">Element to be cleared.</param>
                void ClearElement(KXmlElement^ element)
                {
                    KCheck(kXml_ClearItem(Handle, KXmlElement::ToHandle(element))); 
                }
 
                /// <summary>Deletes an XML element.</summary>
                /// 
                /// <param name="element">Element to be deleted.</param>
                void DeleteElement(KXmlElement^ element)
                {
                    KCheck(kXml_DeleteItem(Handle, KXmlElement::ToHandle(element)));
                }

                /// <summary>Finds the child element at the given relative path.</summary>
                /// 
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Child element.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                KXmlElement^ FindChild(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path); 
                    kXmlItem childItem = kNULL; 

                    KCheck(kXml_FindChild(Handle, KXmlElement::ToHandle(parent), (kChar*) %pathStr, &childItem)); 

                    return KXmlElement::ToObject(this, Handle, childItem); 
                }

                /// <summary>Attempts to find the child element at the given relative path.</summary>
                /// 
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Child element, or null if not found.</returns>
                KXmlElement^ TryFindChild(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    kXmlItem childItem = kNULL;

                    if (kSuccess(kXml_FindChild(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &childItem)))
                    {
                        return KXmlElement::ToObject(this, Handle, childItem);
                    }

                    return nullptr; 
                }

                /// <summary>Reports whether a child element exists at the specified relative path.</summary>
                /// 
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>true if child exists; otherwise false.</returns>
                bool ChildExists(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);

                    return KToBool(kXml_ChildExists(Handle, KXmlElement::ToHandle(parent), (kChar*) %pathStr));
                }
            
                /// <summary>Gets child element content as a string.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Child content.</returns>
                String^ GetChildString(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    KString content; 

                    KCheck(kXml_ChildString(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, content.ToHandle().ToPointer()));

                    return content.ToString(); 
                }

                /// <summary>Gets child element content as a UInt16 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k16u GetChild16u(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k16u value;

                    KCheck(kXml_Child16u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value; 
                }

                /// <summary>Gets child element content as an Int16 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k16s GetChild16s(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k16s value;

                    KCheck(kXml_Child16s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as a UInt32 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k32u GetChild32u(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k32u value;

                    KCheck(kXml_Child32u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as an Int32 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k32s GetChild32s(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k32s value;

                    KCheck(kXml_Child32s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as a UInt64 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k64u GetChild64u(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k64u value;

                    KCheck(kXml_Child64u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as an Int64 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k64s GetChild64s(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k64s value;

                    KCheck(kXml_Child64s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as a Bool value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                bool GetChildBool(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    kBool value;

                    KCheck(kXml_ChildBool(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return KToBool(value);
                }

                /// <summary>Gets child element content as a Single value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k32f GetChild32f(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k32f value;

                    KCheck(kXml_Child32f(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Gets child element content as a Double value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Element value.</returns>
                k64f GetChild64f(KXmlElement^ parent, String^ path)
                {
                    KString pathStr(path);
                    k64f value;

                    KCheck(kXml_Child64f(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, &value));

                    return value;
                }

                /// <summary>Sets child element content to a string.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <returns>Child content.</returns>
                void SetChildString(KXmlElement^ parent, String^ path, String^ value)
                {
                    KString pathStr(path);
                    KString contentStr(value);

                    KCheck(kXml_SetChildText(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, contentStr.CharPtr));
                }

                /// <summary>Sets child element content to a UInt16 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild16u(KXmlElement^ parent, String^ path, k16u value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild16u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value)); 
                }

                /// <summary>Sets child element content to an Int16 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild16s(KXmlElement^ parent, String^ path, k16s value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild16s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to a UInt32 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild32u(KXmlElement^ parent, String^ path, k32u value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild32u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to an Int32 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild32s(KXmlElement^ parent, String^ path, k32s value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild32s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to a UInt64 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild64u(KXmlElement^ parent, String^ path, k64u value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild64u(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to an Int64 value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild64s(KXmlElement^ parent, String^ path, k64s value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild64s(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to a Bool value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChildBool(KXmlElement^ parent, String^ path, bool value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChildBool(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to a Single value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild32f(KXmlElement^ parent, String^ path, k32f value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild32f(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }

                /// <summary>Sets child element content to a Double value.</summary>
                ///
                /// <param name="parent">Parent element.</param>
                /// <param name="path">Path relative to the parent element.</param>
                /// <param name="value">Element value.</param>
                void SetChild64f(KXmlElement^ parent, String^ path, k64f value)
                {
                    KString pathStr(path);

                    KCheck(kXml_SetChild64f(Handle, KXmlElement::ToHandle(parent), pathStr.CharPtr, value));
                }
            };

        }
    }
}

#endif
