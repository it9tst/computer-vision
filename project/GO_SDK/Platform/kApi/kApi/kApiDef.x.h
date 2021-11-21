/** 
 * @file    kApiDef.x.h
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_API_DEF_X_H
#define K_API_API_DEF_X_H

#if defined(K_COMPAT_5)
    kWarn("FireSync 5 compatibility support has been removed from the FireSync Platform. Refer to Platform/scripts/Port/FireSync5.h")
#endif

/*
 * Type system support macros.  
 */

#define xkAssemblyOf(SYMBOL) \
    (xkAssemblyVar(SYMBOL))

#define xkStaticOf(SYMBOL) \
    (kCast(struct SYMBOL##Static*, kType_Static(kTypeOf(SYMBOL))))

#define xkInitFields(SYMBOL, OBJECT)                                                            \
    (memset(kPointer_ByteOffset(OBJECT, sizeof(kCast(SYMBOL##Class*, OBJECT)->base)), 0,        \
        sizeof(SYMBOL##Class) - sizeof(kCast(SYMBOL##Class*, OBJECT)->base)), kOK)

#define xkTypeVar(SYMBOL) \
    xx##SYMBOL##_type

#define xkAssemblyVar(SYMBOL) \
    xx##SYMBOL##_assembly

#define xkRegisterMethod(SYMBOL) \
    xx##SYMBOL##_Register

#define xkStaticInitMethod(SYMBOL) \
    SYMBOL##_InitStatic

#define xkStaticReleaseMethod(SYMBOL) \
    SYMBOL##_ReleaseStatic

#define xkStaticInitPrivateMethod(SYMBOL)            \
    x##SYMBOL##_InitStatic

#define xkStaticReleasePrivateMethod(SYMBOL)         \
    x##SYMBOL##_ReleaseStatic

#define xkBaseMethod(SYMBOL) \
    xx##SYMBOL##_Base

#define xkBaseNameMethod(SYMBOL) \
    xx##SYMBOL##_BaseName

#define xkBaseTypePointer(SYMBOL) \
   xx##SYMBOL##_BaseTypePtr

#define xkDeclareAssemblyEx(PREFIX, SYMBOL)                                                 \
    xkDeclareAssembly(PREFIX, SYMBOL)  

#define xkDeclareAssembly(PREFIX, SYMBOL)                                                   \
    kExtern PREFIX##Dx(kAssembly) xkAssemblyVar(SYMBOL);                                    \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly);                            \
    PREFIX##Fx(kAssembly) SYMBOL##_Instance();

#define xkBeginAssemblyEx(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)                        \
    xkBeginAssembly(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)

#define xkBeginAssembly(PREFIX, SYMBOL, VERSION, PLATFORM_VERSION)                          \
    PREFIX##Dx(kAssembly) xkAssemblyVar(SYMBOL) = kNULL;                                    \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly);                            \
    xkDefinePlugin(SYMBOL, VERSION, PLATFORM_VERSION)                                       \
    PREFIX##Fx(kStatus) SYMBOL##_Construct(kAssembly* assembly)                             \
    {                                                                                       \
        kAssembly output = kNULL;                                                           \
        kStatus status;                                                                     \
        kVersion version, platformVersion;                                                  \
        k32u priority = 0;                                                                  \
        kMemZero(&priority, sizeof(priority));  /* silence Xcode */                         \
        if (!kIsNull(xkAssemblyVar(SYMBOL)))                                                \
        {                                                                                   \
            kCheck(kObject_Share(xkAssemblyVar(SYMBOL)));                                   \
            *assembly = xkAssemblyVar(SYMBOL);                                              \
        }                                                                                   \
        else                                                                                \
        {                                                                                   \
            kTry                                                                            \
            {                                                                               \
                kTest(kVersion_Parse(&version, VERSION));                                   \
                kTest(kVersion_Parse(&platformVersion, PLATFORM_VERSION));                  \
                kTest(xkAssembly_ConstructInternal(&output,                                 \
                    &xkAssemblyVar(SYMBOL), #SYMBOL, version, platformVersion)); 


#define xkEndAssemblyEx()                                                                   \
    xkEndAssembly()

#define xkEndAssembly()                                                                     \
                kTest(xkAssembly_Finalize(output));                                         \
                *assembly = output;                                                         \
            }                                                                               \
            kCatch(&status)                                                                 \
            {                                                                               \
                if (!kIsNull(output)) xkAssembly_VRelease(output);                          \
                kEndCatch(status);                                                          \
            }                                                                               \
        }                                                                                   \
        return kOK;                                                                         \
    }


#if defined(K_PLUGIN)

#define xkDefinePlugin(SYMBOL, ASSEMBLY_VERSION, PLATFORM_VERSION)                          \
                                                                                            \
    kExportCx(const kChar*) kPlugin_AssemblyVersion()                                       \
    {                                                                                       \
        return ASSEMBLY_VERSION;                                                            \
    }                                                                                       \
                                                                                            \
    kExportCx(const kChar*) kPlugin_PlatformVersion()                                       \
    {                                                                                       \
        return PLATFORM_VERSION;                                                            \
    }                                                                                       \
                                                                                            \
    kExportCx(kStatus) kPlugin_ConstructAssembly(kAssembly* assembly)                       \
    {                                                                                       \
        return SYMBOL##_Construct(assembly);                                                \
    }
#else

#define xkDefinePlugin(SYMBOL, ASSEMBLY_VERSION, PLATFORM_VERSION)

#endif

#define kForwardDeclareType(PREFIX, SYMBOL, BASE)                                           \
    kExtern PREFIX##Dx(kType) xkTypeVar(SYMBOL);                                            \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(void* assembly, const kChar* name); 

#define kForwardDeclareClass(PREFIX, SYMBOL, BASE)                                          \
    typedef BASE SYMBOL;                                                                    \
    struct SYMBOL##Class;                                                                   \
    kForwardDeclareType(PREFIX, SYMBOL, BASE)

#define xkDeclareVirtualValue(PREFIX, SYMBOL, BASE)                                         \
    xkDeclareValueType(PREFIX, SYMBOL)                                                      \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareValueVirtualCast(PREFIX, SYMBOL)                                               \

#define xkBeginVirtualValueEx(PREFIX, SYMBOL)                                               \
    xkDefineValueType(PREFIX, SYMBOL)                                                       \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name)     \
    {                                                                                       \
        kCheck(xkAssembly_AddValue(assembly, &xkTypeVar(SYMBOL),                            \
            #SYMBOL, xkBaseMethod(SYMBOL)(), xkBaseNameMethod(SYMBOL)(), 0,                 \
            kTYPE_FLAGS_VALUE));

#define xkEndVirtualValueEx()                                                               \
    xkEndValueRegister()

#define xkDeclareVoidValue(PREFIX, SYMBOL, BASE)                                            \
    xkDeclareValueVTable(PREFIX, SYMBOL, BASE)                                              \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareValueType(PREFIX, SYMBOL)

#define xkBeginVoidValueEx(PREFIX, SYMBOL)                                                  \
    xkDefineValueType(PREFIX, SYMBOL)                                                       \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name)     \
    {                                                                                       \
        kCheck(xkAssembly_AddValue(assembly, &xkTypeVar(SYMBOL),                            \
            #SYMBOL, xkBaseMethod(SYMBOL)(), xkBaseNameMethod(SYMBOL)(), 0,                 \
            kTYPE_FLAGS_VALUE));

#define xkEndVoidValueEx()                                                                  \
    xkEndValueRegister()

#define xkDeclareValueEx(PREFIX, SYMBOL, BASE)                                              \
    kDeclareValue(PREFIX, SYMBOL, BASE)

#define xkDeclareValue(PREFIX, SYMBOL, BASE)                                                \
  /*  xkDeclareValueVTable(PREFIX, SYMBOL, BASE)     FSS-808; may need to restore   */      \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareValueType(PREFIX, SYMBOL)

#define xkBeginValue(PREFIX, SYMBOL, BASE)                                                  \
    kBeginValueEx(PREFIX, SYMBOL)

#define xkBeginValueEx(PREFIX, SYMBOL)                                                      \
    xkDefineValueType(PREFIX, SYMBOL)                                                       \
    xkBeginValueRegister(PREFIX, SYMBOL, kTYPE_FLAGS_VALUE)

#define xkEndValue()                                                                        \
    kEndValueEx()

#define xkEndValueEx()                                                                      \
    xkEndValueRegister()

#define xkDeclareEnumEx(PREFIX, SYMBOL, BASE)                                               \
    kDeclareEnum(PREFIX, SYMBOL, BASE)

#define xkDeclareEnum(PREFIX, SYMBOL, BASE)                                                 \
    xkDeclareValueVTable(PREFIX, SYMBOL, BASE)                                              \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareValueType(PREFIX, SYMBOL)

#define xkBeginEnum(PREFIX, SYMBOL, BASE)                                                   \
    kBeginEnumEx(PREFIX, SYMBOL)

#define xkBeginEnumEx(PREFIX, SYMBOL)                                                       \
    xkDefineValueType(PREFIX, SYMBOL)                                                       \
    xkBeginValueRegister(PREFIX, SYMBOL,                                                    \
        kTYPE_FLAGS_VALUE | kTYPE_FLAGS_PRIMITIVE | kTYPE_FLAGS_ENUM)                       \
    kCheck(xkType_AddField(xkTypeVar(SYMBOL), xkTypeVar(k32s), "k32s", sizeof(k32s),        \
        0, 1, "value")); 

#define xkEndEnum()                                                                         \
    kEndEnumEx()

#define xkEndEnumEx()                                                                       \
    xkEndValueRegister()

#define xkDeclareArrayValueEx(PREFIX, SYMBOL, BASE)                                         \
    kDeclareArrayValue(PREFIX, SYMBOL, BASE)

#define xkDeclareArrayValue(PREFIX, SYMBOL, BASE)                                           \
    xkDeclareValueVTable(PREFIX, SYMBOL, BASE)                                              \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareValueType(PREFIX, SYMBOL)

#define xkBeginArrayValue(PREFIX, SYMBOL, TYPE, BASE)                                       \
    kBeginArrayValueEx(PREFIX, SYMBOL, TYPE)

#define xkBeginArrayValueEx(PREFIX, SYMBOL, TYPE)                                           \
    xkDefineValueType(PREFIX, SYMBOL)                                                       \
    xkBeginValueRegister(PREFIX, SYMBOL, kTYPE_FLAGS_VALUE | kTYPE_FLAGS_ARRAY_VALUE)       \
    kCheck(xkType_AddField(xkTypeVar(SYMBOL), xkTypeVar(TYPE), #TYPE,                       \
        sizeof(SYMBOL), 0, sizeof(SYMBOL)/sizeof(TYPE), "elements")); 

#define xkEndArrayValue()                                                                   \
    kEndArrayValueEx()

#define xkEndArrayValueEx()                                                                 \
    xkEndValueRegister()

#define xkDeclareValueVTable(PREFIX, SYMBOL, BASE)                                          \
    typedef struct SYMBOL##VTable { BASE##VTable base; } SYMBOL##VTable;

#define xkDeclareValueType(PREFIX, SYMBOL)                                                  \
    kExtern PREFIX##Dx(kType) xkTypeVar(SYMBOL);                                            \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name);
    
#define xkDefineValueType(PREFIX, SYMBOL)                                                   \
    PREFIX##Dx(kType) xkTypeVar(SYMBOL) = kNULL;

#define xkBeginValueRegister(PREFIX, SYMBOL, FLAGS)                                         \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name)     \
    {                                                                                       \
        kCheck(xkAssembly_AddValue(assembly, &xkTypeVar(SYMBOL),                            \
            #SYMBOL, xkBaseMethod(SYMBOL)(), xkBaseNameMethod(SYMBOL)(), sizeof(SYMBOL),    \
            FLAGS));

#define xkEndValueRegister()                                                                \
        return kOK;                                                                         \
    }

#define xkDefineBaseType(PREFIX, SYMBOL, BASE)                                              \
    kInlineFx(kType) xkBaseMethod(SYMBOL)()                                                 \
    {                                                                                       \
        return xkTypeVar(BASE);                                                             \
    }                                                                                       \
    kInlineFx(const kChar*) xkBaseNameMethod(SYMBOL)()                                      \
    {                                                                                       \
        return #BASE;                                                                       \
    }                                                                                       \
    static struct BASE##Class* xkBaseTypePointer(SYMBOL) K_ATTRIBUTE_UNUSED;

#define xkDeclareInterfaceEx(PREFIX, SYMBOL, BASE)                                          \
    xkDeclareInterface(PREFIX, SYMBOL, BASE)                                                \
    xkDeclareInterfaceCast(PREFIX, SYMBOL) 

#define xkDeclareInterface(PREFIX, SYMBOL, BASE)                                            \
    kExtern PREFIX##Dx(kType) xkTypeVar(SYMBOL);                                            \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name);

#define xkBeginInterface(PREFIX, SYMBOL, BASE)                                              \
    kBeginInterfaceEx(PREFIX, SYMBOL)   

#define xkBeginInterfaceEx(PREFIX, SYMBOL)                                                  \
    xkDefineInterfaceType(PREFIX, SYMBOL)                                                   \
    xkBeginInterfaceRegister(PREFIX, SYMBOL)

#define xkEndInterface()                                                                    \
    kEndInterfaceEx()

#define xkEndInterfaceEx()                                                                  \
    xkEndInterfaceRegister()

#define xkDefineInterfaceType(PREFIX, SYMBOL)                                               \
    PREFIX##Dx(kType) xkTypeVar(SYMBOL) = kNULL;

#define xkBeginInterfaceRegister(PREFIX, SYMBOL)                                            \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name)     \
    {                                                                                       \
        kCheck(xkAssembly_AddInterface(assembly, &xkTypeVar(SYMBOL),                        \
            #SYMBOL, xkBaseMethod(SYMBOL)(), xkBaseNameMethod(SYMBOL)())); 

#define xkEndInterfaceRegister()                                                            \
        return kOK;                                                                         \
    }

#define xkDeclareFullClassEx(PREFIX, SYMBOL, BASE)                                          \
    xkDeclareFullClass(PREFIX, SYMBOL, BASE)                                                \
    xkDeclareClassCast(PREFIX, SYMBOL)                                                      \
    xkDeclareVirtualCast(PREFIX, SYMBOL)                                                    \
    xkDeclareStaticCast(PREFIX, SYMBOL)

#define xkDeclareFullClass(PREFIX, SYMBOL, BASE)                                            \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareClassType(PREFIX, SYMBOL)

#define xkBeginFullClassEx(PREFIX, SYMBOL)                                                  \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)                                                    \
    xkAddStatic(SYMBOL)
      
#define xkBeginFullClass(PREFIX, SYMBOL, BASE)                                              \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)                                                    \
    xkCheckBase(#PREFIX, #SYMBOL, #BASE, xkBaseNameMethod(SYMBOL)());                       \
    xkAddStaticLegacy(SYMBOL)

#define xkEndFullClass()                                                                    \
    kEndFullClassEx()

#define xkEndFullClassEx()                                                                   \
    xkEndClassRegister()

#define xkDeclareVirtualClassEx(PREFIX, SYMBOL, BASE)                                       \
    xkDeclareVirtualClass(PREFIX, SYMBOL, BASE)                                             \
    xkDeclareClassCast(PREFIX, SYMBOL)                                                      \
    xkDeclareVirtualCast(PREFIX, SYMBOL) 

#define xkDeclareVirtualClass(PREFIX, SYMBOL, BASE)                                         \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareClassType(PREFIX, SYMBOL) 

#define xkBeginVirtualClass(PREFIX, SYMBOL, BASE)                                           \
    kBeginVirtualClassEx(PREFIX, SYMBOL)                                                    \
    xkCheckBase(#PREFIX, #SYMBOL, #BASE, xkBaseNameMethod(SYMBOL)());
    
#define xkBeginVirtualClassEx(PREFIX, SYMBOL)                                               \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)

#define xkEndVirtualClass()                                                                 \
    kEndVirtualClassEx()

#define xkEndVirtualClassEx()                                                               \
    xkEndClassRegister()

#define xkDeclareStaticClassEx(PREFIX, SYMBOL)                                              \
    xkDeclareStaticClass(PREFIX, SYMBOL)                                                    \
    xkDeclareStaticCast(PREFIX, SYMBOL)    

#define xkDeclareStaticClass(PREFIX, SYMBOL)                                                \
    xkDeclareClassInstance(PREFIX, SYMBOL, kObject)                                         \
  /*  xkDeclareClassVTable(PREFIX, SYMBOL, kObject)       FSS-808; may need to restore  */  \
    xkDefineBaseType(PREFIX, SYMBOL, kObject)                                               \
    xkDeclareClassType(PREFIX, SYMBOL) 

#define xkBeginStaticClassEx(PREFIX, SYMBOL)                                                \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)                                                    \
    xkAddStatic(SYMBOL)

#define xkBeginStaticClass(PREFIX, SYMBOL)                                                  \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)                                                    \
    xkAddStaticLegacy(SYMBOL)

#define xkEndStaticClass()                                                                  \
    kEndStaticClassEx()

#define xkEndStaticClassEx()                                                                \
    xkEndClassRegister()

#define xkDeclareClassEx(PREFIX, SYMBOL, BASE)                                              \
    xkDeclareClass(PREFIX, SYMBOL, BASE)                                                    \
    xkDeclareClassCast(PREFIX, SYMBOL)

#define xkDeclareClass(PREFIX, SYMBOL, BASE)                                                \
  /*  xkDeclareClassVTable(PREFIX, SYMBOL, BASE)     FSS-808; may need to restore  */       \
    xkDefineBaseType(PREFIX, SYMBOL, BASE)                                                  \
    xkDeclareClassType(PREFIX, SYMBOL)

#define xkBeginClass(PREFIX, SYMBOL, BASE)                                                  \
    kBeginClassEx(PREFIX, SYMBOL)                                                           \
    xkCheckBase(#PREFIX, #SYMBOL, #BASE, xkBaseNameMethod(SYMBOL)());

#define xkBeginClassEx(PREFIX, SYMBOL)                                                      \
    xkDefineClassType(PREFIX, SYMBOL)                                                       \
    xkBeginClassRegister(PREFIX, SYMBOL)

#define xkEndClass()                                                                        \
    kEndClassEx()

#define xkEndClassEx()                                                                       \
    xkEndClassRegister()

#define xkDeclareClassType(PREFIX, SYMBOL)                                                  \
    kExtern PREFIX##Dx(kType) xkTypeVar(SYMBOL);                                            \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name);

#define xkDeclareClassInstance(PREFIX, SYMBOL, BASE)                                        \
    typedef struct SYMBOL##Class { BASE##Class base; } SYMBOL##Class;

#define xkDeclareClassVTable(PREFIX, SYMBOL, BASE)                                          \
    typedef struct SYMBOL##VTable { BASE##VTable base; } SYMBOL##VTable;

#define xkDeclareClassStatic(PREFIX, SYMBOL)                                                \
    typedef struct SYMBOL##Static { kByte placeholder; } SYMBOL##Static;

#define xkDeclareValueVirtualCast(PREFIX, SYMBOL)                                           \
    kInlineFx(struct SYMBOL##VTable*) SYMBOL##_VTable(kType type)                           \
    {                                                                                       \
        return kCast(kValueVTable*, kType_VTable(type));                                    \
    }   

#define xkDeclareClassCast(PREFIX, SYMBOL)                                                  \
    /* Performs a type-checked cast */                                                      \
    kInlineFx(struct SYMBOL##Class*) x##SYMBOL##_Cast(SYMBOL object)                        \
    {                                                                                       \
        return xkCastTag_(SYMBOL, object, __FILE__, __LINE__);                              \
    }                                                                                       \
    /* Performs a type-checked cast with file/line info */                                  \
    kInlineFx(struct SYMBOL##Class*) xx##SYMBOL##_Cast(SYMBOL object,                       \
                                                        const kChar* file, k32s line)       \
    {                                                                                       \
        return xkCastTag_(SYMBOL, object, file, line);                                      \
    }                                                                                       \
    /* Performs a non-type-checked cast */                                                  \
    kInlineFx(struct SYMBOL##Class*) x##SYMBOL##_CastRaw(SYMBOL object)                     \
    {                                                                                       \
        return (struct SYMBOL##Class*) object;                                              \
    }

#define xkDeclareVirtualCast(PREFIX, SYMBOL)                                                \
    kInlineFx(struct SYMBOL##VTable*) x##SYMBOL##_VTable(SYMBOL object)                     \
    {                                                                                       \
        kAssertType(object, SYMBOL);                                                        \
        return kCast(struct SYMBOL##VTable*, kType_VTable(kObject_Type(object)));           \
    }   

#define xkDeclareInterfaceCast(PREFIX, SYMBOL)                                              \
    kInlineFx(struct SYMBOL##VTable*) x##SYMBOL##_VTable(SYMBOL object)                     \
    {                                                                                       \
        kAssertType(object, SYMBOL);                                                        \
        return kCast(struct SYMBOL##VTable*, kType_IVTable(kObject_Type(object),            \
            kTypeOf(SYMBOL)));                                                              \
    }   

#define xkDeclareStaticCast(PREFIX, SYMBOL)                                                 \
    kInlineFx(struct SYMBOL##Static*) x##SYMBOL##_Static()                                  \
    {                                                                                       \
        return kCast(struct SYMBOL##Static*, kStaticOf(SYMBOL));                            \
    }   

#define xkDefineClassType(PREFIX, SYMBOL)                                                   \
    PREFIX##Dx(kType) xkTypeVar(SYMBOL) = kNULL;

#define xkBeginClassRegister(PREFIX, SYMBOL)                                                \
    PREFIX##Fx(kStatus) xkRegisterMethod(SYMBOL)(kAssembly assembly, const kChar* name)     \
    {                                                                                       \
        kCheck(xkAssembly_AddClass(assembly, &xkTypeVar(SYMBOL),                            \
            #SYMBOL, xkBaseMethod(SYMBOL)(), xkBaseNameMethod(SYMBOL)(),                    \
            sizeof(SYMBOL##Class)));

#define xkEndClassRegister()                                                                \
        return kOK;                                                                         \
    }

#define xkAddDependency(SYMBOL)                                                             \
    kTest(xkAssembly_AddDependency(output, SYMBOL##_Construct)); 

#define xkAddType(SYMBOL)                                                                   \
    kTest(xkAssembly_AddType(output, xkRegisterMethod(SYMBOL), #SYMBOL)); 

#define xkAddPriority(SYMBOL)                                                               \
    kTest(xkAssembly_AddPriority(output, xkRegisterMethod(SYMBOL), priority++)); 

#define xkAddStaticLegacy(SYMBOL)                                                           \
    kCheck(xkType_AddStatic(xkTypeVar(SYMBOL), sizeof(SYMBOL##Static),                      \
            xkStaticInitMethod(SYMBOL), xkStaticReleaseMethod(SYMBOL)));

#define xkAddStatic(SYMBOL)                                                                 \
    kCheck(xkType_AddStatic(xkTypeVar(SYMBOL), sizeof(SYMBOL##Static),                      \
            xkStaticInitPrivateMethod(SYMBOL), xkStaticReleasePrivateMethod(SYMBOL)));

#define xkAddInterface(SYMBOL, IFACE)                                                       \
    kCheck(xkType_ImplementInterface(xkTypeVar(SYMBOL), xkTypeVar(IFACE),                   \
        #IFACE, sizeof(IFACE##VTable))); 

#define xkAddMethod(CLASS, METHOD)                                                          \
    kCheck(xkType_AddMethod(xkTypeVar(CLASS),                                               \
        (kFunction)CLASS##_##METHOD, #METHOD));

#define xkAddFrameworkConstructor(CLASS, CTOR)                                                \
    kCheck(xkType_AddFrameworkConstructor(xkTypeVar(CLASS), (kFunction)CLASS##_##CTOR));

#define xkAddPrivateFrameworkConstructor(CLASS, CTOR)                                          \
    kCheck(xkType_AddFrameworkConstructor(xkTypeVar(CLASS), (kFunction)x##CLASS##_##CTOR));

#define xkAddPrivateVMethod(IN_CLASS, FROM_CLASS, METHOD)                                   \
    if (0) (((FROM_CLASS##VTable*)0)->METHOD) = x##IN_CLASS##_##METHOD;  /* type check */   \
    kCheck(xkType_AddVMethod(xkTypeVar(IN_CLASS),                                           \
        offsetof(FROM_CLASS##VTable, METHOD)/sizeof(kPointer),                              \
        (kFunction)x##IN_CLASS##_##METHOD, #METHOD));

#define xkAddVMethod(IN_CLASS, FROM_CLASS, METHOD)                                          \
    if (0) (((FROM_CLASS##VTable*)0)->METHOD) = IN_CLASS##_##METHOD;  /* type check */      \
    kCheck(xkType_AddVMethod(xkTypeVar(IN_CLASS),                                           \
        offsetof(FROM_CLASS##VTable, METHOD)/sizeof(kPointer),                              \
        (kFunction)IN_CLASS##_##METHOD, #METHOD));

#define xkAddPrivateIVMethod(IN_CLASS, FROM_IFACE, IMETHOD, CMETHOD)                        \
    if (0) (((FROM_IFACE##VTable*)0)->IMETHOD) = x##IN_CLASS##_##CMETHOD;  /* type check */ \
    kCheck(xkType_AddIVMethod(xkTypeVar(IN_CLASS), xkTypeVar(FROM_IFACE),                   \
        offsetof(FROM_IFACE##VTable, IMETHOD)/sizeof(kPointer),                             \
        (kFunction) x##IN_CLASS##_##CMETHOD, #IMETHOD, #CMETHOD)); 

#define xkAddIVMethod(IN_CLASS, FROM_IFACE, IMETHOD, CMETHOD)                               \
    if (0) (((FROM_IFACE##VTable*)0)->IMETHOD) = IN_CLASS##_##CMETHOD;  /* type check */    \
    kCheck(xkType_AddIVMethod(xkTypeVar(IN_CLASS), xkTypeVar(FROM_IFACE),                   \
        offsetof(FROM_IFACE##VTable, IMETHOD)/sizeof(kPointer),                             \
        (kFunction) IN_CLASS##_##CMETHOD, #IMETHOD, #CMETHOD)); 

#define xkAddField(VALUE, FIELD_TYPE, FIELD)                                                \
    kCheck(xkType_AddField(xkTypeVar(VALUE), xkTypeVar(FIELD_TYPE), #FIELD_TYPE,            \
        sizeof(((VALUE*)0)->FIELD), offsetof(VALUE, FIELD),                                 \
        sizeof(((VALUE*)0)->FIELD)/sizeof(FIELD_TYPE),                                      \
        #FIELD)); 

#define xkAddEnumerator(SYMBOL, ENUMERATOR)                                                 \
    kCheck(xkType_AddEnumerator(xkTypeVar(SYMBOL), ENUMERATOR, #ENUMERATOR)); 

#define xkAddPrivateVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)    \
    kCheck(xkType_AddVersion(xkTypeVar(TYPE), FORMAT, FORMAT_VER, GUID,                     \
    (kFunction)x##TYPE##_##WRITE_METHOD, (kFunction)x##TYPE##_##READ_METHOD, kFALSE));

#define xkAddPrivateVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)      \
    kCheck(xkType_AddVersion(xkTypeVar(TYPE), FORMAT, FORMAT_VER, GUID,                     \
    (kFunction)x##TYPE##_##WRITE_METHOD, (kFunction)x##TYPE##_##READ_METHOD, kTRUE));

#define xkAddVersionEx(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)           \
    kCheck(xkType_AddVersion(xkTypeVar(TYPE), FORMAT, FORMAT_VER, GUID,                     \
    (kFunction)TYPE##_##WRITE_METHOD, (kFunction)TYPE##_##READ_METHOD, kFALSE));

#define xkAddVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)             \
    kCheck(xkType_AddVersion(xkTypeVar(TYPE), FORMAT, FORMAT_VER, GUID,                     \
    (kFunction)TYPE##_##WRITE_METHOD, (kFunction)TYPE##_##READ_METHOD, kTRUE));

#define xkAddAbstractVersion(TYPE, FORMAT, FORMAT_VER, GUID)                                \
    kCheck(xkType_AddVersion(xkTypeVar(TYPE), FORMAT, FORMAT_VER, GUID,                     \
    kNULL, kNULL, kFALSE));

#define xkAddFlags(TYPE, FLAGS)                                                             \
    kCheck(xkType_AddFlags(xkTypeVar(TYPE), FLAGS)); 

/*
 * Support for "debug hints".  See kDefineDebugHints. 
 */

#define xkAddClassDebugHint(TYPE)                                \
    static TYPE##Class* TYPE##_debugHint = kNULL;

#define xkAddValueDebugHint(TYPE)                                \
    static TYPE* TYPE##_debugHint = kNULL;

#if defined(K_DEBUG) && defined(K_MSVC)

//types that require K_PLATFORM (e.g., kThread) should be excluded from this list
#define xkDefineDebugHints()                                     \
    xkAddClassDebugHint(kAlloc)                                  \
    xkAddClassDebugHint(kAssembly)                               \
    xkAddClassDebugHint(kObject)                                 \
    xkAddClassDebugHint(kType)                                   \
                                                                 \
    xkAddClassDebugHint(kArray1)                                 \
    xkAddClassDebugHint(kArray2)                                 \
    xkAddClassDebugHint(kArray3)                                 \
    xkAddClassDebugHint(kArrayList)                              \
    xkAddClassDebugHint(kBitArray)                               \
    xkAddClassDebugHint(kBox)                                    \
    xkAddClassDebugHint(kImage)                                  \
    xkAddClassDebugHint(kList)                                   \
    xkAddClassDebugHint(kMap)                                    \
    xkAddClassDebugHint(kQueue)                                  \
    xkAddClassDebugHint(kString)                                 \
    xkAddClassDebugHint(kXml)                                    \
                                                                 \
    xkAddClassDebugHint(kDat5Serializer)                         \
    xkAddClassDebugHint(kDat6Serializer)                         \
    xkAddClassDebugHint(kHttpServer)                             \
    xkAddClassDebugHint(kHttpServerChannel)                      \
    xkAddClassDebugHint(kHttpServerRequest)                      \
    xkAddClassDebugHint(kHttpServerResponse)                     \
    xkAddClassDebugHint(kMemory)                                 \
    xkAddClassDebugHint(kSerializer)                             \
    xkAddClassDebugHint(kStream)                                 \
    xkAddClassDebugHint(kTcpClient)                              \
    xkAddClassDebugHint(kTcpServer)                              \
    xkAddClassDebugHint(kUdpClient)                              \
    xkAddClassDebugHint(kWebSocket)                              \
                                                                 \
    xkAddClassDebugHint(kMsgQueue)                               \
    xkAddClassDebugHint(kPeriodic)                               \
    xkAddClassDebugHint(kTimer)                                  \
                                                                 \
    xkAddClassDebugHint(kBackTrace)                              \
    xkAddClassDebugHint(kDebugAlloc)                             \
    xkAddClassDebugHint(kDynamicLib)                             \
    xkAddClassDebugHint(kEvent)                                  \
    xkAddClassDebugHint(kPoolAlloc)                              \
    xkAddClassDebugHint(kUserAlloc)                              \
                                                                 \
    xkAddValueDebugHint(kEnumeratorInfo)                         \
    xkAddValueDebugHint(kFieldInfo)                              \
    xkAddValueDebugHint(xkInterfaceInfo)                         \
    xkAddValueDebugHint(kMethodInfo)                             \
    xkAddValueDebugHint(xkStructField)                           \
    xkAddValueDebugHint(kTypeVersionInfo)                        \
                                                                 \
    xkAddValueDebugHint(kListItemStruct)                         \
    xkAddValueDebugHint(kListItemBlock)                          \
                                                                 \
    xkAddValueDebugHint(kHash)                                   \
    xkAddValueDebugHint(kSha1Hash)                               \
                                                                 \
    xkAddValueDebugHint(kArgb)                                   \
    xkAddValueDebugHint(kPoint16s)                               \
    xkAddValueDebugHint(kPoint32s)                               \
    xkAddValueDebugHint(kPoint32f)                               \
    xkAddValueDebugHint(kPoint64f)                               \
    xkAddValueDebugHint(kPoint3d16s)                             \
    xkAddValueDebugHint(kPoint3d32s)                             \
    xkAddValueDebugHint(kPoint3d32f)                             \
    xkAddValueDebugHint(kPoint3d64f)                             \
    xkAddValueDebugHint(kRect16s)                                \
    xkAddValueDebugHint(kRect32s)                                \
    xkAddValueDebugHint(kRect32f)                                \
    xkAddValueDebugHint(kRect64f)                                \
    xkAddValueDebugHint(kRect3d64f)                              \
    xkAddValueDebugHint(kRgb)                                    \
    xkAddValueDebugHint(kRotatedRect32s)                         \
    xkAddValueDebugHint(kRotatedRect32f)

#else

#define xkDefineDebugHints()

#endif

/*
 * Error-handling and exception support macros. 
 */

#define xkCheckTrue(EXPRESSION, STATUS)                                 \
    do                                                                  \
    {                                                                   \
        if (!(EXPRESSION))                                              \
        {                                                               \
            kCheckTrace("kCheck failed");                               \
            return STATUS;                                              \
        }                                                               \
    } while(0)    


#define xkCheck(EXPRESSION)                                             \
    do                                                                  \
    {                                                                   \
        kStatus kStatus_var = (kStatus)(EXPRESSION);                    \
        if (kIsError(kStatus_var))                                      \
        {                                                               \
            kCheckTrace("kCheck failed");                               \
            return kStatus_var;                                         \
        }                                                               \
    } while(0)

#define xkCheckArgs(EXPRESSION)                                         \
    kCheckTrue(EXPRESSION, kERROR_PARAMETER)

#define xkCheckState(EXPRESSION)                                        \
    kCheckTrue(EXPRESSION, kERROR_STATE)

#define xkTry                                                           \
    {                                                                   \
        kStatus kExcept_value = kOK;                                    \
                                                                        \
        /* eliminates compiler warnings when kTest isn't used */        \
        kTest(kOK);


#define xkThrow(EXPRESSION)                                             \
        do                                                              \
        {                                                               \
            kExcept_value = (EXPRESSION);                               \
            kCheckTrace("kThrow exception");                            \
            goto kEXCEPT_CATCH_LABEL;                                   \
        } while (kFALSE)

#define xkTest(EXPRESSION)                                              \
        do                                                              \
        {                                                               \
            kStatus kExcept_temp = (kStatus)(EXPRESSION);               \
            if (!kSuccess(kExcept_temp))                                \
            {                                                           \
                kThrow(kExcept_temp);                                   \
            }                                                           \
        } while (kFALSE)

#define xkTestTrue(EXPRESSION, STATUS)                                  \
        do                                                              \
        {                                                               \
            if (!(EXPRESSION))                                          \
            {                                                           \
                kThrow(STATUS);                                         \
            }                                                           \
        } while (kFALSE)

#define xkTestArgs(EXPRESSION)                                          \
        kTestTrue(EXPRESSION, kERROR_PARAMETER)

#define xkTestState(EXPRESSION)                                         \
        kTestTrue(EXPRESSION, kERROR_STATE)

#define xkCatch(STATUS_POINTER)                                         \
kEXCEPT_CATCH_LABEL:                                                    \
        {                                                               \
            *(STATUS_POINTER) = kExcept_value;                          \
            if (kSuccess(kExcept_value))                                \
            {                                                           \
                goto kEXCEPT_CATCH_END_LABEL;                           \
            }                                                           \

#define xkEndCatch(status)                                              \
            kCheck(status);                                             \
        }                                                               \
    }                                                                   \
kEXCEPT_CATCH_END_LABEL:                                                \
    (void)0


#define xkFinally                                                       \
kEXCEPT_CATCH_LABEL:

#define xkEndFinally()                                                  \
        kCheck(kExcept_value);                                          \
    } (void)0

#define xkCatchEx(STATUS_POINTER)                                       \
kEXCEPT_CATCH_LABEL:                                                    \
        {                                                               \
            *(STATUS_POINTER) = kExcept_value;                          \
            if (kSuccess(kExcept_value))                                \
            {                                                           \
                goto kEXCEPT_FINALLY_LABEL;                             \
            }

#define xkEndCatchEx(STATUS)                                            \
    kExcept_value = (STATUS)

#define xkFinallyEx                                                     \
        }                                                               \
kEXCEPT_FINALLY_LABEL:

#define xkEndFinallyEx()                                                \
        kCheck(kExcept_value);                                          \
    } (void)0

#if (defined(K_DEBUG) || defined(K_ASSERT)) && !defined(K_NO_ASSERT)

#   define xkAssert(EXPRESSION)                                         \
        do                                                              \
        {                                                               \
            kBool kAssert_result = (EXPRESSION);                        \
            if (!kAssert_result)                                        \
            {                                                           \
                kApiAssertFx handler = kApiLib_AssertHandler();         \
                if (!kIsNull(handler))                                  \
                {                                                       \
                    handler(__FILE__, __LINE__);                        \
                }                                                       \
            }                                                           \
        } while(0)

#   define xkAssertOk(EXPRESSION)                                       \
        kAssert(kSuccess(EXPRESSION))

#   define xkAssertType(EXPRESSION, SYMBOL)                             \
        kAssert(kObject_Is(EXPRESSION, kTypeOf(SYMBOL)))

    kFx(kBool) xkCastClass_ObjectIs(kPointer object, kPointer type); 
    kFx(kBool) xkCastClass_ObjectIsValid(kPointer object, kPointer type); 

#   define xkCastTag_(SYMBOL, OBJ, FILENAME, LINE)                                          \
        (xkCastClass_ObjectIs(OBJ, kTypeOf(SYMBOL)) ?                                       \
            kCast(struct SYMBOL##Class*, OBJ) :                                             \
            kCast(struct SYMBOL##Class*, xkApiLib_CastFailHandler(FILENAME, LINE)))

#   define xkCastClass_(SYMBOL, OBJ)                                                        \
        (xkCastClass_ObjectIs(OBJ, kTypeOf(SYMBOL)) ?                                       \
            kCast(SYMBOL##Class*, OBJ) :                                                    \
            kCast(SYMBOL##Class*, xkApiLib_CastFailHandler(__FILE__, __LINE__)))

//only used in the implementation of kType; provides weaker checking 
#   define xkTypeCastClass_(SYMBOL, OBJ)                                                    \
        (xkCastClass_ObjectIsValid(OBJ, xkTypeVar(SYMBOL)) ?                                \
            kCast(SYMBOL##Class*, OBJ) :                                                    \
            kCast(SYMBOL##Class*, xkApiLib_CastFailHandler(__FILE__, __LINE__)))

#   define xkTypeOf(SYMBOL)                                                                 \
        (!kIsNull(xkTypeVar(SYMBOL)) ?                                                      \
            (xkTypeVar(SYMBOL)) :                                                           \
            xkApiLib_CastFailHandler(__FILE__, __LINE__))

#else

#   define xkAssert(EXPRESSION)                     ((void)0)
#   define xkAssertOk(EXPRESSION)                   ((void)0)
#   define xkAssertType(EXPRESSION, SYMBOL)         ((void)0)

#   define xkCastTag_(SYMBOL, OBJ, FILENAME, LINE) \
        kCast(struct SYMBOL##Class*, OBJ)  

#   define xkCastClass_(SYMBOL, OBJ) \
        kCast(SYMBOL##Class*, OBJ)

//only used in the implementation of kType; provides weaker checking 
#   define xkTypeCastClass_(SYMBOL, OBJ)        \
        kCast(SYMBOL##Class*, OBJ)

#   define xkTypeOf(SYMBOL)                     \
        (xkTypeVar(SYMBOL))


#endif

#define xkCastVTable_(SYMBOL, OBJ)              \
    kCast(SYMBOL##VTable*, kType_VTable(kObject_Type(OBJ)))

#define xkCastIVTable_(SYMBOL, OBJ)             \
    kCast(SYMBOL##VTable*, kType_IVTable(kObject_Type(OBJ), kTypeOf(SYMBOL)))


#if !defined(K_NO_TRACE)

#   define xkTrace(TAG)                                                             \
        if (!kIsNull(kApiLib_TraceHandler()))                                       \
        {                                                                           \
            kApiLib_TraceHandler()(TAG, __FILE__, __LINE__);                        \
        }

#else

#   define xkTrace(TAG)

#endif

#if defined(K_CHECK_TRACE)

//Deprecated: do not use in new code. 
#   define kCheckTrace(TAG)                                                         \
        do                                                                          \
        {                                                                           \
            kApiTraceFx handler = kApiLib_TraceHandler();                           \
                                                                                    \
            if (!kIsNull(handler) && kApiLib_CheckTraceEnabled())                   \
            {                                                                       \
                handler(TAG, __FILE__, __LINE__);                                   \
            }                                                                       \
        } while (0)

#else

//Deprecated: do not use in new code. 
#   define kCheckTrace(TAG)                     \
        (void)0

#endif

/* 
 * Class forward declarations
 */

typedef void* kIterator; 

kForwardDeclareClass(k, kApiLog, kObject)
kForwardDeclareClass(k, kArray1, kObject)
kForwardDeclareClass(k, kArray2, kObject)
kForwardDeclareClass(k, kArray3, kObject)
kForwardDeclareClass(k, kArrayList, kObject)
kForwardDeclareClass(k, kArrayProvider, kObject)
kForwardDeclareClass(k, kAssembly, kObject)
kForwardDeclareClass(k, kBackTrace, kObject)
kForwardDeclareClass(k, kBitArray, kObject)
kForwardDeclareClass(k, kBox, kObject)
kForwardDeclareClass(k, kCipher, kObject)
kForwardDeclareClass(k, kCollection, kObject)
kForwardDeclareClass(k, kCompressor, kObject)
kForwardDeclareClass(k, xkDateTimeManager, kObject)
kForwardDeclareClass(k, kDirectory, kObject)
kForwardDeclareClass(k, kDynamicLib, kObject)
kForwardDeclareClass(k, kEvent, kObject)
kForwardDeclareClass(k, kHash, kObject)
kForwardDeclareClass(k, kHttpServer, kObject)
kForwardDeclareClass(k, kHttpServerChannel, kObject)
kForwardDeclareClass(k, kHttpServerRequest, kObject)
kForwardDeclareClass(k, kHttpServerResponse, kObject)
kForwardDeclareClass(k, kImage, kObject)
kForwardDeclareClass(k, kList, kObject)
kForwardDeclareClass(k, kLock, kObject)
kForwardDeclareClass(k, kMap, kObject)
kForwardDeclareClass(k, kMsgQueue, kObject)
kForwardDeclareClass(k, kNetwork, kObject)
kForwardDeclareClass(k, kNetworkAdapter, kObject)
kForwardDeclareClass(k, kNetworkInfo, kObject)
kForwardDeclareClass(k, kNetworkInterface, kObject)
kForwardDeclareClass(k, kParallel, kObject)
kForwardDeclareClass(k, kParallelJob, kObject)
kForwardDeclareClass(k, kPeriodic, kObject)
kForwardDeclareClass(k, kPlugin, kObject)
kForwardDeclareClass(k, kProcess, kObject)
kForwardDeclareClass(k, kObjectPool, kObject)
kForwardDeclareClass(k, kQueue, kObject)
kForwardDeclareClass(k, kSemaphore, kObject)
kForwardDeclareClass(k, kSerializer, kObject)
kForwardDeclareClass(k, kSocket, kObject)
kForwardDeclareClass(k, kStream, kObject)
kForwardDeclareClass(k, kString, kObject)
kForwardDeclareClass(k, kSymbolInfo, kObject)
kForwardDeclareClass(k, kTcpServer, kObject)
kForwardDeclareClass(k, kThread, kObject)
kForwardDeclareClass(k, kThreadPool, kObject)
kForwardDeclareClass(k, kTimer, kObject)
kForwardDeclareClass(k, kXml, kObject)

kForwardDeclareClass(k, kDebugAlloc, kAlloc)
kForwardDeclareClass(k, kUserAlloc, kAlloc)
kForwardDeclareClass(k, kPoolAlloc, kAlloc)

kForwardDeclareClass(k, kBlowfishCipher, kCipher)

kForwardDeclareClass(k, kDat5Serializer, kSerializer)
kForwardDeclareClass(k, kDat6Serializer, kSerializer)

kForwardDeclareClass(k, kCipherStream, kStream)
kForwardDeclareClass(k, kFile, kStream)
kForwardDeclareClass(k, kMemory, kStream)
kForwardDeclareClass(k, kPipeStream, kStream)
kForwardDeclareClass(k, kTcpClient, kStream)
kForwardDeclareClass(k, kUdpClient, kStream)
kForwardDeclareClass(k, kWebSocket, kStream)

kForwardDeclareClass(k, kSha1Hash, kHash)

typedef k64s kDateTime;

typedef kPointer kXmlItem;

typedef xkAtomic32s kAtomic32s;
typedef xkAtomicPointer kAtomicPointer;

//Function forward declarations, to resolve core header dependency issues

kInlineFx(void) kAtomic32s_Init(kAtomic32s* atomic, k32s value); 
kInlineFx(k32s) kAtomic32s_Increment(kAtomic32s* atomic); 
kInlineFx(k32s) kAtomic32s_Decrement(kAtomic32s* atomic); 
kInlineFx(k32s) kAtomic32s_Get(kAtomic32s* atomic); 

kInlineFx(kAllocTrait) kAlloc_Traits(kAlloc alloc);

#if defined(K_CPP)
kInlineFx(kStatus) kAlloc_Get(kAlloc alloc, kSize size, void* mem, kMemoryAlignment alignment);
#endif
kInlineFx(kStatus) kAlloc_Get(kAlloc alloc, kSize size, void* mem);

#if defined(K_CPP)
kInlineFx(kStatus) kAlloc_GetZero(kAlloc alloc, kSize size, void* mem, kMemoryAlignment alignment);
#endif
kInlineFx(kStatus) kAlloc_GetZero(kAlloc alloc, kSize size, void* mem);

kInlineFx(kStatus) kAlloc_Free(kAlloc alloc, void* mem);
kInlineFx(kStatus) kAlloc_FreeRef(kAlloc alloc, void* mem);

kFx(kStatus) kMemSet(void* dest, kByte fill, kSize size);
kInlineFx(kStatus) kMemZero(void* dest, kSize size);
kFx(kStatus) kMemCopy(void* dest, const void* src, kSize size);
kFx(kStatus) kMemMove(void* dest, const void* src, kSize size);
kFx(kStatus) kMemReverseCopy(void* dest, const void* src, kSize size); 
kInlineFx(kSize) xkHashPointer(kPointer pointer);

kInlineFx(kBool) kObject_Is(kObject object, kType type);
kInlineFx(kType) kObject_Type(kObject object);
kInlineFx(kAlloc) kObject_Alloc(kObject object); 

kInlineFx(kFunction*) kType_VTable(kType type);
kInlineFx(kFunction*) kType_IVTable(kType type, kType interfaceType);
kInlineFx(kBool) kType_Is(kType type, kType other);
kInlineFx(kSize) kType_InnerSize(kType type);
kInlineFx(kBool) kType_Extends(kType type, kType baseType);
kInlineFx(kBool) kType_Implements(kType type, kType interfaceType);
kInlineFx(kType) kType_Base(kType type); 
kInlineFx(kBool) kType_IsArrayValue(kType type);
kInlineFx(kSize) kType_Size(kType type); 
kInlineFx(kBool) xkType_IsPointerCompatible(kType type, kSize size); 
kInlineFx(kFrameworkConstructorFx) kType_FrameworkConstructor(kType type);

kExtern kDx(kType) xkTypeVar(kNull);

kFx(k64u) xkApiLib_TimerStub();

kFx(void) xkCheckBase(const kChar* prefix, const kChar* typeName, const kChar* definitionBase, const kChar* declarationBase); 

/* 
 * Some core headers are included here, both for convenience, and to enforce a 
 * specific include order. For these headers, #include <kApi/kApiDef.h> should 
 * appear *before* the normal include guard, which ensures that kApiDef has 
 * complete control over include order.
 */

#include <kApi/kApiLib.h>
#include <kApi/kValue.h>
#include <kApi/kObject.h>
#include <kApi/kType.h>
#include <kApi/kAssembly.h>
#include <kApi/kAlloc.h>
#include <kApi/Utils/kUtils.h>
#include <kApi/Threads/kAtomic.h>

/* 
 * Core value types
 */

xkDeclareVoidValue(k, kVoid, kValue) 

kFx(kStatus) xkVoid_Write(kType type, void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkVoid_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k8u, kValue) 

kFx(kBool) xk8u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk8u_VHashCode(kType type, const void* value); 
kFx(kStatus) xk8u_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk8u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k16u, kValue) 

kFx(kBool) xk16u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk16u_VHashCode(kType type, const void* value); 
kFx(kStatus) xk16u_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk16u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k32u, kValue)

kFx(kBool) xk32u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk32u_VHashCode(kType type, const void* value); 
kFx(kStatus) xk32u_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk32u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k64u, kValue)

kFx(kBool) xk64u_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk64u_VHashCode(kType type, const void* value); 
kFx(kStatus) xk64u_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk64u_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k8s, kValue)

kFx(kBool) xk8s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk8s_VHashCode(kType type, const void* value); 
kFx(kStatus) xk8s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk8s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k16s, kValue)

kFx(kBool) xk16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk16s_VHashCode(kType type, const void* value); 
kFx(kStatus) xk16s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k32s, kValue)

kFx(kBool) xk32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk32s_VHashCode(kType type, const void* value); 
kFx(kStatus) xk32s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k64s, kValue) 

kFx(kBool) xk64s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk64s_VHashCode(kType type, const void* value); 
kFx(kStatus) xk64s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk64s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k32f, kValue)

kFx(kBool) xk32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk32f_VHashCode(kType type, const void* value); 
kFx(kStatus) xk32f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, k64f, kValue)

kFx(kBool) xk64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xk64f_VHashCode(kType type, const void* value); 
kFx(kStatus) xk64f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xk64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kByte, kValue)

kFx(kBool) xkByte_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkByte_VHashCode(kType type, const void* value); 
kFx(kStatus) xkByte_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkByte_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kChar, kValue)

kFx(kBool) xkChar_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkChar_VHashCode(kType type, const void* value); 
kFx(kStatus) xkChar_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkChar_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kBool, kValue)

kFx(kBool) xkBool_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkBool_VHashCode(kType type, const void* value); 
kFx(kStatus) xkBool_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkBool_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kSize, kValue)

kFx(kBool) xkSize_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkSize_VHashCode(kType type, const void* value); 
kFx(kStatus) xkSize_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkSize_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kSSize, kValue)

kFx(kBool) xkSSize_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkSSize_VHashCode(kType type, const void* value); 
kFx(kStatus) xkSSize_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkSSize_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPointer, kValue)

kFx(kBool) xkPointer_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPointer_VHashCode(kType type, const void* value); 

kDeclareValueEx(k, kFunction, kValue) 

kFx(kBool) xkFunction_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkFunction_VHashCode(kType type, const void* value); 

kDeclareArrayValueEx(k, kText16, kValue)

kFx(kBool) xkText16_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkText16_VHashCode(kType type, const void* value); 
kFx(void) xkText16_VImport(kType type, void* value, const void* source); 
kFx(kStatus) xkText16_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkText16_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValueEx(k, kText32, kValue)

kFx(kBool) xkText32_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkText32_VHashCode(kType type, const void* value); 
kFx(void) xkText32_VImport(kType type, void* value, const void* source); 
kFx(kStatus) xkText32_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkText32_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValueEx(k, kText64, kValue)

kFx(kBool) xkText64_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkText64_VHashCode(kType type, const void* value); 
kFx(void) xkText64_VImport(kType type, void* value, const void* source); 
kFx(kStatus) xkText64_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkText64_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValueEx(k, kText128, kValue)

kFx(kBool) xkText128_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkText128_VHashCode(kType type, const void* value); 
kFx(void) xkText128_VImport(kType type, void* value, const void* source); 
kFx(kStatus) xkText128_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkText128_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareArrayValueEx(k, kText256, kValue)

kFx(kBool) xkText256_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkText256_VHashCode(kType type, const void* value); 
kFx(void) xkText256_VImport(kType type, void* value, const void* source); 
kFx(kStatus) xkText256_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkText256_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareEnumEx(k, kStatus, kValue)

kFx(kBool) xkStatus_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkStatus_VHashCode(kType type, const void* value); 
kFx(kStatus) xkStatus_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkStatus_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kVersion, kValue)

kFx(kBool) xkVersion_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkVersion_VHashCode(kType type, const void* value); 
kFx(kStatus) xkVersion_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkVersion_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareEnumEx(k, kEndianness, kValue)

kDeclareValueEx(k, kPoint16s, kValue)

kFx(kBool) xkPoint16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint16s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint16s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint32s, kValue)

kFx(kBool) xkPoint32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint32s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint32s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint32f, kValue)

kFx(kBool) xkPoint32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint32f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint32f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint64f, kValue)

kFx(kBool) xkPoint64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint64f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint64f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkPoint_Init_(POINT, X, Y)          \
    ((POINT)->x = X, (POINT)->y = (Y))

kDeclareValueEx(k, kPoint3d16s, kValue)

kFx(kBool) xkPoint3d16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint3d16s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint3d16s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint3d16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint3d32s, kValue)

kFx(kBool) xkPoint3d32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint3d32s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint3d32s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint3d32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint3d32f, kValue)

kFx(kBool) xkPoint3d32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint3d32f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint3d32f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint3d32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kPoint3d64f, kValue)

kFx(kBool) xkPoint3d64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint3d64f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint3d64f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint3d64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkPoint3d_Init_(POINT, X, Y, Z)                         \
    ((POINT)->x = X, (POINT)->y = (Y), (POINT)->z = (Z))

kDeclareValueEx(k, kPoint4d16s, kValue)

kFx(kBool) xkPoint4d16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPoint4d16s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPoint4d16s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPoint4d16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkPoint4d_Init_(POINT, X, Y, Z, W)                      \
    ((POINT)->x = X, (POINT)->y = (Y), (POINT)->z = (Z), (POINT)->w = (W))

kDeclareValueEx(k, kRect16s, kValue)

kFx(kBool) xkRect16s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRect16s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRect16s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRect16s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRect32s, kValue)

kFx(kBool) xkRect32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRect32s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRect32s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRect32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRect32f, kValue)

kFx(kBool) xkRect32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRect32f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRect32f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRect32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRect64f, kValue)

kFx(kBool) xkRect64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRect64f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRect64f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRect64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRect3d64f, kValue)

kFx(kBool) xkRect3d64f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRect3d64f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRect3d64f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRect3d64f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkRect_Init_(RECT, X, Y, W, H)                          \
    ((RECT)->x = X, (RECT)->y = (Y),                            \
     (RECT)->width = (W), (RECT)->height = (H))

#define xkRect3d_Init_(RECT, X, Y, Z, W, H, D)                   \
    ((RECT)->x = X, (RECT)->y = (Y), (RECT)->z = (Z),            \
     (RECT)->width = (W), (RECT)->height = (H),                  \
     (RECT)->depth = (D))

kDeclareValueEx(k, kRotatedRect32s, kValue)

kFx(kBool) xkRotatedRect32s_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRotatedRect32s_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRotatedRect32s_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRotatedRect32s_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRotatedRect32f, kValue)

kFx(kBool) xkRotatedRect32f_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRotatedRect32f_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRotatedRect32f_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRotatedRect32f_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkRotatedRect_Init_(RECT, XC, YC, W, H, A)              \
    ((RECT)->xc = XC, (RECT)->yc = (YC),                        \
     (RECT)->width = (W), (RECT)->height = (H),                 \
     (RECT)->angle = (A))

kDeclareEnumEx(k, kPixelFormat, kValue)

kFx(kBool) xkPixelFormat_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkPixelFormat_VHashCode(kType type, const void* value); 
kFx(kStatus) xkPixelFormat_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkPixelFormat_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareEnumEx(k, kCfa, kValue)

kFx(kBool) xkCfa_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkCfa_VHashCode(kType type, const void* value); 
kFx(kStatus) xkCfa_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkCfa_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kRgb, kValue)

kFx(kBool) xkRgb_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkRgb_VHashCode(kType type, const void* value); 
kFx(kStatus) xkRgb_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkRgb_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkRgb_Init_(RGB, R, G, B)                               \
    ((RGB)->b = (B), (RGB)->g = (G), (RGB)->r = (R)) 

kDeclareValueEx(k, kArgb, kValue)

kFx(kBool) xkArgb_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkArgb_VHashCode(kType type, const void* value); 
kFx(kStatus) xkArgb_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkArgb_Read(kType type, void* values, kSize count, kSerializer serializer); 

#define xkArgb_Init_(ARGB, A, R, G, B)                          \
    ((ARGB)->b = (B), (ARGB)->g = (G),                          \
     (ARGB)->r = (R), (ARGB)->a = (A))

kDeclareValueEx(k, kMacAddress, kValue)

kFx(kBool) xkMacAddress_VEquals(kType type, const void* value, const void* other);

kDeclareEnumEx(k, kComparison, kValue)

kFx(kBool) xkComparison_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkComparison_VHashCode(kType type, const void* value); 
kFx(kStatus) xkComparison_Write(kType type, const void* values, kSize count, kSerializer serializer); 
kFx(kStatus) xkComparison_Read(kType type, void* values, kSize count, kSerializer serializer); 

kDeclareValueEx(k, kCallbackFx, kValue) 

kFx(kBool) xkCallbackFx_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkCallbackFx_VHashCode(kType type, const void* value); 

kDeclareValueEx(k, kCallback, kValue)

kFx(kBool) xkCallback_VEquals(kType type, const void* value, const void* other); 
kFx(kSize) xkCallback_VHashCode(kType type, const void* value); 

kDeclareEnumEx(k, kFileMode, kValue)

kDeclareEnumEx(k, kSeekOrigin, kValue)

kDeclareEnumEx(k, kCompressionType, kValue)

kDeclareEnumEx(k, kCompressionPreset, kValue)

kDeclareEnumEx(k, kMemoryAlignment, kValue)

kDeclareEnumEx(k, kLogOption, kValue)

kDeclareValueEx(k, kLogArgs, kValue)

kDeclareEnumEx(k, kAllocTrait, kValue)

kDeclareValueEx(k, kThreadId, kValue)

kDeclareValueEx(k, kThreadPriorityClass, kValue)

kInlineFx(void) xkItemCopy1(void* dest, const void* src)
{
    kItemCopy(dest, src, 1); 
}

kInlineFx(void) xkItemCopy2(void* dest, const void* src)
{
    kItemCopy(dest, src, 2); 
}

kInlineFx(void) xkItemCopy4(void* dest, const void* src)
{
    kItemCopy(dest, src, 4); 
}

kInlineFx(void) xkItemCopy8(void* dest, const void* src)
{
    kItemCopy(dest, src, 8); 
}

kInlineFx(void) xkItemReverseCopy2(void* dest, const void* src)
{
    kByte* d = (kByte*)dest; 
    const kByte* s = (const kByte*) src; 

    d[0] = s[1]; 
    d[1] = s[0]; 
}

kInlineFx(void) xkItemReverseCopy4(void* dest, const void* src)
{
    kByte* d = (kByte*)dest; 
    const kByte* s = (const kByte*) src; 

    d[0] = s[3]; 
    d[1] = s[2]; 
    d[2] = s[1]; 
    d[3] = s[0]; 
}

kInlineFx(void) xkItemReverseCopy8(void* dest, const void* src)
{
    kByte* d = (kByte*)dest; 
    const kByte* s = (const kByte*) src; 

    d[0] = s[7]; 
    d[1] = s[6]; 
    d[2] = s[5]; 
    d[3] = s[4]; 
    d[4] = s[3]; 
    d[5] = s[2]; 
    d[6] = s[1]; 
    d[7] = s[0]; 
}
 
/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Replace with kDeclareEnum
#define kDeclareBitEnum kDeclareEnum

//[Deprecated] Replace with kBeginEnum
#define kBeginBitEnum kBeginEnum

//[Deprecated] Replace with kEndEnum
#define kEndBitEnum kEndEnum 

//[Deprecated] Convert to kValue_Import 
kInlineFx(void) kItemImport_(void* dest, const void* src, kType type)         
{
    kValue_Import(type, dest, src);
}

//[Deprecated] Replace with individual field initializations.
#define kInitFields(TYPE, OBJECT) \
    xkInitFields(TYPE, OBJECT)    

//[Deprecated] Replace with kAddVersionEx; requires updating deserialization method; refer to kType_VersionDeserializeFx. 
#define kAddVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD) \
    xkAddVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)   

//[Deprecated] Replace with kAddPrivateVersionEx; requires updating deserialization method; refer to kType_VersionDeserializeFx. 
#define kAddPrivateVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD) \
    xkAddPrivateVersion(TYPE, FORMAT, FORMAT_VER, GUID, WRITE_METHOD, READ_METHOD)   

//[Deprecated] For most use cases, replace with kMin. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kMin_(A, B) (((A) < (B)) ? (A) : (B))

//[Deprecated] For most use cases, replace with kMax. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kMax_(A, B) (((A) > (B)) ? (A) : (B))

//[Deprecated] For most use cases, replace with kClamp. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kClamp_(V, VMIN, VMAX) (kMin_(kMax_((V), (VMIN)), (VMAX)))

//[Deprecated] For most use cases, replace with kAbs. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kAbs_(A) (((A) >= 0) ? (A) : -(A))

//[Deprecated] For most use cases, replace with kDivideFloor. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kDivideFloorInt_(A, B) (((A) >= 0) ? (A) / (B) : ((A) - (B) + 1) / (B))

//[Deprecated] For most use cases, replace with kDivideCeil. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kDivideCeilInt_(A, B) (((A) >= 0) ? ((A) + (B) - 1) / (B) : (A) / (B))

//[Deprecated] For most use cases, replace with kDivideFloor. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kDivideFloorUInt_(A, B) ((A) / (B))

//[Deprecated] For most use cases, replace with kDivideCeil. If used in the definition of an inline function that may be consumed 
//by C or C++ 98 compilers, replace with equivalent C expression.
#define kDivideCeilUInt_(A, B) (((A) + (B) - 1) / (B))

#endif
