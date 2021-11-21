//
// KType.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_TYPE_H
#define K_API_NET_TYPE_H

#include "kApiNet/KObject.h"

namespace Lmi3d
{
    namespace Zen 
    {
        /// <summary>Represents type method information.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kMethodInfo))]
        public value struct KMethodInfo
        {
            /// <summary>Method name (e.g. "Clone").</summary>
            property String^ MethodName
            {
                String^ get() { return m_methodName.ToString(); }
            }

            /// <summary>Full function name (e.g. "kObject_Clone").</summary>
            property String^ FunctionName
            {
                String^ get() { return m_functionName.ToString(); }
            }

            /// <summary>Pointer to function.</summary>
            property IntPtr Function
            {
                IntPtr get() { return IntPtr((void*)m_function.Value); }
            }

        internal:
            KMethodInfo(const kMethodInfo* info)
            {
                pin_ptr<KMethodInfo> self = this;
                kItemCopy(self, info, sizeof(KMethodInfo));
            }

        private:
            [FieldOffset(offsetof(kMethodInfo, methodName))]
            KText64 m_methodName;

            [FieldOffset(offsetof(kMethodInfo, functionName))]
            KText64 m_functionName;

            [FieldOffset(offsetof(kMethodInfo, function))]
            KPointer m_function;
        };

        /// <summary> Represents type field information. </summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kFieldInfo))]
        public value struct KFieldInfo
        {
        public:
            /// <summary>Field name.</summary>
            property String^ Name
            {
                String^ get() { return m_name.ToString(); }
            }

            property KType^ Type
            {
                KType^ get(); 
            }

            /// <summary>Offset of field within structure (bytes).</summary>
            property k64s Offset
            {
                k64s get() { return (k64s) m_offset.Value; }
            }

            /// <summary>Count of values in this field (typically 1; can be higher for "array value" fields, e.g. kText32). </summary>
            property k64s Count
            {
                k64s get() { return (k64s)m_count.Value; }
            }

        internal:
            KFieldInfo(const kFieldInfo* info)
            {
                pin_ptr<KFieldInfo> self = this;
                kItemCopy(self, info, sizeof(KFieldInfo));
            }

        private:

            [FieldOffset(offsetof(kFieldInfo, type))]
            kType m_type;

            [FieldOffset(offsetof(kFieldInfo, name))]
            KText64 m_name;

            [FieldOffset(offsetof(kFieldInfo, offset))]
            KSize m_offset;

            [FieldOffset(offsetof(kFieldInfo, count))]
            KSize m_count;
        };

        /// <summary> Represents enumerator information. </summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kEnumeratorInfo))]
        public value struct KEnumeratorInfo
        {
        public:

            /// <summary>Enumerator numeric value.</summary>
            [FieldOffset(offsetof(kEnumeratorInfo, value))]
            K32s Value;                         

            /// <summary>Enumerator name (e.g. "kPIXEL_FORMAT_8BPP_GREYSCALE").</summary>
            property String^ Name
            {
                String^ get() { return m_name; }
            }

            /// <summary>Formatted display name (e.g. "8bpp Greyscale").</summary>
            property String^ DisplayName
            {
                String^ get() { return m_displayName; }
            }

        internal:

            KEnumeratorInfo(const kEnumeratorInfo* info)
            {
                pin_ptr<KEnumeratorInfo> self = this;
                kItemCopy(self, info, sizeof(KEnumeratorInfo));
            }

        private:
            [FieldOffset(offsetof(kEnumeratorInfo, name))]
            KText64 m_name;

            [FieldOffset(offsetof(kEnumeratorInfo, displayName))]
            KText64 m_displayName;
        };

        /// <summary>Represents an opaque reference to type version information (used in object serialization).</summary>
        public value struct KTypeVersion
        {
        public:
            /// <summary>Gets the GUID associated with the specified type version.</summary>
            property String^ Guid
            {
                String^ get() { return KToString(kType_VersionGuid(Type, Handle)); }
            }

            /// <summary>Gets the serialization format name associated with the type version.</summary>
            property String^ Format
            {
                String^ get() { return KToString(kType_VersionFormat(Type, Handle)); }
            }

            /// <summary>Gets the serialization format version associated with the type version..</summary>
            property KVersion FormatVersion
            {
                KVersion get() { return KVersion(kType_VersionFormatVersion(Type, Handle)); }
            }

        internal:
            KTypeVersion(kType type, kPointer handle)
                :Type(type), Handle(handle)
            {
            }

            kType Type; 
            kPointer Handle;
        };


        /// <summary>Represents metadata describing a native type.</summary>
        /// 
        /// <remarks>
        /// <para>When a Zen native assembly is constructed, one kType instance is created for every type defined in the assembly.
        /// KAssembly methods can be used to discover the types in the assembly, and KType methods can be used to
        /// learn about individual types (e.g., type name, base class).</para>
        /// 
        /// <para>The KObject.Type method can be used to obtain type information for any object derived from KObject.
        /// The KTypeId static property can be used to obtain type information for any class, interface, or value.</para>
        /// </remarks>
        /// 
        /// <example>This example illustrates the use of various KType methods.
        /// <code language="C#" source="examples/KType_Introspect.cs" />
        /// </example>
        public ref class KType : public KObject
        {
        public:
            KDeclareClass(KType, kType)

            /// <summary>Initializes a new instance of the KType class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            KType(kType handle)
                : KObject(IntPtr(handle), DefaultRefStyle)
            {}

            /// <summary>Initializes a new instance of the KType class with the specified Zen object handle.</summary>           
            /// <param name="handle">Zen object handle.</param>
            KType(IntPtr handle)
                : KObject(handle, DefaultRefStyle) 
            {}

            /// <summary>Gets the assembly to which the type belongs.</summary>
            property KAssembly^ Assembly 
            { 
                KAssembly^ get(); 
            }

            /// <summary>Gets the name of the type.</summary>
            property String^ Name 
            { 
                String^ get() { return KToString(kType_Name(Handle)); }
            }

            /// <summary>Determines whether a type is equivalent to another type.</summary>
            /// 
            /// <remarks>Checks type equality, inheritance, and interfaces.</remarks>
            /// 
            /// <param name="other">Type to which type is compared.</param>
            /// <returns>kTRUE if type is equivalent.</returns>
            bool Is(KType^ other)
            {
                return KToBool(kType_Is(Handle, other->Handle)); 
            }

            /// <summary>Determines whether a type represents a value (primitive, struct, enum).</summary>
            property bool IsValue
            {
                bool get() { return KToBool(kType_IsValue(Handle)); }
            }

            /// <summary>Determines whether a type represents a class.</summary>
            property bool IsClass
            {
                bool get() { return KToBool(kType_IsClass(Handle)); }
            }

            /// <summary>Determines whether a type represents an interface.</summary>
            property bool IsInterface
            {
                bool get() { return KToBool(kType_IsInterface(Handle)); }
            }

            /// <summary>Determines whether a type represents a class or interface.</summary>
            property bool IsReference
            {
                bool get() { return KToBool(kType_IsReference(Handle)); }
            }

            /// <summary>Determines whether a type represents an abstract class.</summary>
            property bool IsAbstract
            {
                bool get() { return KToBool(kType_IsAbstract(Handle)); }
            }

            /// <summary>Reports whether the type is an enumeration.</summary>
            property bool IsEnum
            {
                bool get() { return KToBool(kType_IsEnum(Handle)); }
            }

            /// <summary>Reports whether the type is an enumeration bitset.</summary>
            property bool IsBitEnum
            {
                bool get() { return KToBool(kType_IsBitEnum(Handle)); }
            }

            /// <summary>Reports whether the type is an 'array-value' type (e.g., kText32)</summary>
            property bool IsArrayValue
            {
                bool get() { return KToBool(kType_IsArrayValue(Handle)); }
            }

            /// <summary>Reports whether the type is a primitive value (single-valued; lacks fields).</summary>
            property bool IsPrimitive
            {
                bool get() { return KToBool(kType_IsPrimitive(Handle)); }
            }

            /// <summary>Reports whether the fields of a value type are tightly packed (free from structure padding).</summary>
            property bool IsPacked
            {
                bool get() { return KToBool(kType_IsPacked(Handle)); }
            }

            /// <summary>Determines whether a type implements a specific interface.</summary>
            /// 
            /// <param name="interfaceType">Interface type.</param>
            /// <returns>true if type implements interface.</returns>
            bool Implements(KType^ interfaceType)
            {
                return KToBool(kType_Implements(Handle, interfaceType->Handle));
            }

            /// <summary>Determines whether a type extends another type.</summary>
            /// 
            /// <param name="baseType">Base type.</param>
            /// <returns>true if type extends base type.</returns>
            bool Extends(KType^ baseType)
            {
                return KToBool(kType_Extends(Handle, baseType->Handle));
            }

            /// <summary>Gets the base of a class or interface.</summary>
            property KType^ Base
            {
                KType^ get() { return kIsNull(kType_Base(Handle)) ? nullptr : gcnew KType(kType_Base(Handle)); }
            }

            /// <summary>Reports count of implemented interfaces.</summary>
            property k32s InterfaceCount
            {
                k32s get() { return (k32s)kType_InterfaceCount(Handle); }
            }

            /// <summary>Gets the implemented interface at the specified index.</summary>
            /// 
            /// <param name="index">Interface index.</param>
            /// <returns>Interface type.</returns>
            KType^ InterfaceAt(k32s index)
            {
                return gcnew KType(kType_InterfaceAt(Handle, (kSize)index)); 
            }

            /// <summary>Gets the external size of a type.</summary>
            /// 
            /// <remarks>The external size of a value type reflects the size of the struct, value or enum. The external size of a reference
            /// type is equal to the size of a pointer.</remarks>
            property k64s Size 
            {
                k64s get() new { return (k64s) kType_Size(Handle); }
            }

            /// <summary>Gets the internal size of a type.</summary>
            /// 
            /// <remarks>The internal size of a value type reflects the size of the struct, value or enum. The internal size of a reference
            /// type is equal to the size of its class structure.</remarks>
            property k64s InnerSize
            {
                k64s get() { return (k64s)kType_InnerSize(Handle); }
            }

            /// <summary>Gets the size of a type's static data.</summary>
            property k64s StaticSize
            {
                k64s get() { return (k64s)kType_StaticSize(Handle); }
            }

            /// <summary>Gets a pointer to the type's primary virtual method table.</summary>
            property KPointer VTable
            {
                KPointer get() { return KPointer(kType_VTable(Handle)); }
            }

            /// <summary>Gets a pointer to the type's virtual method table corresponding to the specified interface type.</summary>
            /// 
            /// <param name="interfaceType">Interface type.</param>
            /// <returns>Interface virtual table pointer.</returns>
            KPointer IVTable(KType^ interfaceType)
            {
                return KPointer(kType_IVTable(Handle, interfaceType->Handle)); 
            }

            /// <summary>Gets a pointer to the type's static data structure.</summary>
            property KPointer Static
            {
                KPointer get() { return KPointer(kType_Static(Handle)); }
            }

            /// <summary>Reports whether the type's static data structure has been successfully initialized.</summary>
            /// 
            /// <remarks>This property can be helpful during startup, to determine whether the static data for a particular type
            /// has been initialized. This property is not thread-safe.</remarks>
            property bool StaticInitialized
            {
                bool get() { return KToBool(kType_StaticInitialized(Handle)); }
            }

            /// <summary>Reports count of non-virtual methods.</summary>
            /// 
            /// <remarks>Most types do not register non-virtual methods. However, non-virtual methods can optionally be registered
            /// using the kAddMethod macro. This can be useful in specific scenarios requiring non-virtual method reflection.</remarks>
            property k32s MethodCount
            {
                k32s get() { return (k32s)kType_MethodCount(Handle); }
            }

            /// <summary>Gets metadata for the non-virtual method at the specified index.</summary>
            /// 
            /// <param name="index">Method index.</param>
            /// <returns>Method metadata.</returns>
            KMethodInfo MethodInfoAt(k32s index)
            {
                return KMethodInfo(kType_MethodInfoAt(Handle, (kSize)index)); 
            }

            /// <summary>Finds metadata for the non-virtual method with the specified name.</summary>
            /// 
            /// <param name="name">Method name.</param>
            /// <returns>Method metadata.</returns>
            /// <exception cref="KException">Thrown if not found.</exception>
            KMethodInfo FindMethodInfo(String^ name)
            {
                KText64 textName = name; 
                const kMethodInfo* info = kNULL; 

                KCheck(kType_FindMethodInfo(Handle, (kChar*)&textName, &info)); 

                return KMethodInfo(info); 
            }

            /// <summary>Reports count of virtual methods.</summary>
            property k32s VMethodCount
            {
                k32s get() { return (k32s)kType_VMethodCount(Handle); }
            }

            /// <summary>Gets metadata for the virtual method at the specified index.</summary>
            /// 
            /// <param name="index">Method index.</param>
            /// <returns>Method metadata.</returns>
            KMethodInfo VMethodInfoAt(k32s index)
            {
                return KMethodInfo(kType_VMethodInfoAt(Handle, (kSize)index));
            }

            /// <summary>Reports count of interface methods for the given interface.</summary>
            /// 
            /// <param name="interfaceType">Interface type.</param>
            /// <returns>Count of interface methods.</returns>
            k32s IMethodCount(KType^ interfaceType)
            {
                return (k32s) kType_IMethodCount(Handle, interfaceType->Handle); 
            }

            /// <summary>Gets metadata for the interface method at the specified index.</summary>
            /// 
            /// <param name="interfaceType">Interface type.</param>
            /// <param name="index">Method index.</param>
            /// <returns>Method metadata.</returns>
            KMethodInfo IMethodInfoAt(KType^ interfaceType, k32s index)
            {
                return KMethodInfo(kType_IMethodInfoAt(Handle, interfaceType->Handle, (kSize)index));
            }

            /// <summary>Reports count of registered fields for the given type.</summary>
            /// 
            /// <remarks>Value fields can optionally be registered using the kAddField macro.</remarks>
            property k32s FieldCount
            {
                k32s get() { return (k32s)kType_FieldCount(Handle); }
            }

            /// <summary>Gets metadata for the field at the specified index.</summary>
            /// 
            /// <param name="index">Field index.</param>
            /// <returns>Field metadata.</returns>
            KFieldInfo FieldInfoAt(k32s index)
            {
                return KFieldInfo(kType_FieldInfoAt(Handle, (kSize)index));
            }

            /// <summary>Reports count of registered enumerators for the given enumeration type.</summary>
            /// 
            /// <remarks>Enumerators can optionally be registered using the kAddEnumerator macro.</remarks>
            property k32s EnumeratorCount
            {
                k32s get() { return (k32s)kType_EnumeratorCount(Handle); }
            }

            /// <summary>Gets metadata for the enumerator at the specified index.</summary>
            /// 
            /// <param name="index">Enumerator index.</param>
            /// <returns>Enumerator metadata.</returns>
            KEnumeratorInfo EnumeratorInfoAt(k32s index)
            {
                return KEnumeratorInfo(kType_EnumeratorInfoAt(Handle, (kSize)index));
            }

            /// <summary>Finds enumerator metadata for the enumerator with the specified value.</summary>
            /// 
            /// <param name="value">Enumerator value.</param>
            /// <returns>Enumerator metadata.</returns>
            /// <exception cref="KException">Thrown if not found</exception>
            KEnumeratorInfo FindEnumeratorInfo(k32s value)
            {
                const kEnumeratorInfo* info = kNULL; 

                KCheck(kType_FindEnumeratorInfo(Handle, value, &info)); 

                return KEnumeratorInfo(info); 
            }

            /// <summary>Formats an enumerator value to a text buffer using the enumerator display name.</summary>
            /// 
            /// <remarks>
            /// <para>If the enumerator value isn't known, the enumerator will be formatted as "[Unknown]".</para>
            /// 
            /// <para>Exercise caution when using formatted enumerator names in file formats or communication protocols;
            /// enumerator text names may change if the enumerator source code is modified. Consider using explicit and
            /// stable format/parse methods, or using numeric values instead.</para>
            /// </remarks>
            /// 
            /// <param name="value">Enumerator value.</param>
            /// <returns>Formatted enumertor name.</returns>
            String^ FormatEnumerator(k32s value)
            {
                KText64 textName; 

                kType_FormatEnumerator(Handle, value, (kChar*)&textName, sizeof(textName)); 

                return textName; 
            }

            /// <summary>Parses an enumerator value from a text buffer using the enumerator display name.</summary>
            /// 
            /// <remarks>If a matching enumerator display name cannot be found, 0 will be returned.</remarks>
            /// 
            /// <param name="formattedName">Enumerator display name.</param>
            /// <returns>Enumerator value.</returns>
            k32s ParseEnumerator(String^ formattedName)
            {
                KText64 textName = formattedName;
                k32s value; 

                kType_ParseEnumerator(Handle, &value, (kChar*)&textName); 

                return value; 
            }

            /// <summary>Reports count of registered serialization versions.</summary>
            property k32s VersionCount
            {
                k32s get() { return (k32s)kType_VersionCount(Handle); }
            }

            /// <summary>Gets metadata for the serialization version at the specified index.</summary>
            /// 
            /// <param name="index">Serialization version index.</param>
            /// <returns>Serialization version metadata.</returns>
            KTypeVersion VersionInfoAt(k32s index)
            {
                return KTypeVersion(Handle, kType_VersionAt(Handle, (kSize)index));
            }

            String^ ToString() override
            {
                return Name;
            }

        protected:
            KType() : KObject(DefaultRefStyle) {}
        };
    }
}

#endif
