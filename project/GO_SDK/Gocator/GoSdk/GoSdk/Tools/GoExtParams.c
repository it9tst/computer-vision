/** 
 * @file    GoExtParams.c
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include <GoSdk/Tools/GoExtParams.h>
#include <GoSdk/GoUtils.h>

kBeginClassEx(Go, GoExtParams)
    kAddVMethod(GoExtParams, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoExtParams_Construct(GoExtParams* params, kObject sensor, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoExtParams), params));

    if (!kSuccess(status = GoExtParams_Init(*params, kTypeOf(GoExtParams), sensor, alloc)))
    {
        kAlloc_FreeRef(alloc, params);
    }

    return status;
}

GoFx(kStatus) GoExtParams_Init(GoExtParams params, kType type, kObject sensor, kAlloc alloc)
{
    kObjR(GoExtParams, params);
    kStatus exception;

    kCheck(kObject_Init(params, type, alloc));
    kZero(obj->params);
    kZero(obj->xml);
    kZero(obj->nodesToMerge);

    obj->sensor = sensor;

    kTry
    {
        kTest(kArrayList_Construct(&obj->params, kTypeOf(GoExtParam), 0, alloc));
        kTest(kArrayList_Construct(&obj->nodesToMerge, kTypeOf(kPointer), 0, alloc));
    }
    kCatch(&exception)
    {
        GoExtParams_VRelease(params);
        kEndCatch(exception);
    }

    return kOK; 
}

GoFx(kStatus) GoExtParams_VRelease(GoExtParams params)
{
    kObj(GoExtParams, params);
    kSize i;
    for (i = 0; i < kArrayList_Count(obj->params); i++)
    {
        GoExtParam param = *kArrayList_AtT(obj->params, i, GoExtParam);
        kCheck(kDestroyRef(&param));
    }

    kCheck(kDestroyRef(&obj->params));
    kCheck(kDestroyRef(&obj->nodesToMerge));
    kCheck(kObject_VRelease(params));

    return kOK;
}

GoFx(kStatus) GoExtParams_Read(GoExtParams params, kXml xml, kXmlItem item)
{
    kObj(GoExtParams, params);
    kXmlItem paramItem = kNULL;
    kType paramType;
    kText32 typeVal;
    kStatus exception;
    kSize i;

    obj->xml = xml;

    kCheck(kArrayList_Clear(obj->nodesToMerge));

    for (i = 0; i < kXml_ChildCount(xml, item); i++)
    {
        GoExtParam param = kNULL;
        kBool insertParam = kFALSE;
        
        paramItem = kXml_ChildAt(xml, item, i);

        if (kSuccess(kXml_AttrText(xml, paramItem, "type", typeVal, kCountOf(typeVal))) && 
                !kIsNull((paramType = GoExtUtils_GetKType(typeVal))))
        {
            kTry
            {
                if (i >= kArrayList_Count(obj->params)
                || kIsNull(param = kArrayList_AsT(obj->params, i, GoExtParam))
                || !kObject_Is(param, paramType))
                {
                    if (!kIsNull(param))
                    {
                        kTest(kArrayList_RemoveT(obj->params, i, &param));
                        kDisposeRef(&param);
                    }
                    kTest(GoExtParam_Construct(&param, paramType, obj->sensor, kObject_Alloc(params)));
                    insertParam = kTRUE;
                }

                kTest(GoExtParam_Read(param, xml, paramItem));

                if (insertParam)
                {
                    if (i >= kArrayList_Count(obj->params))
                        kTest(kArrayList_AddT(obj->params, &param));
                    else
                        kTest(kArrayList_InsertT(obj->params, i, &param));
                }
            }
            kCatch(&exception)
            {
                kDestroyRef(&param);
                kEndCatch(exception);
            }
        }
        else
        {
            kCheck(kArrayList_AddT(obj->nodesToMerge, &paramItem));
        }
    }

    return kOK;
}

GoFx(kStatus) GoExtParams_Write(GoExtParams params, kXml xml, kXmlItem item)
{
    kObj(GoExtParams, params);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->params); i++)
    {
        GoExtParam param = *kArrayList_AtT(obj->params, i, GoExtParam);
        kXmlItem valueItem = kNULL;

        kCheck(kXml_AddItem(xml, item, GoExtParam_Id(param), &valueItem));
        kCheck(GoExtParam_Write(param, xml, valueItem));
    }

    // XML merge intentionally omitted due to the dynamic nature of the Tools element. Replaced
    //  with unrecognized node insertion
    for (i = 0; i < kArrayList_Count(obj->nodesToMerge); i++)
    {
        kXmlItem mergeItem = *kArrayList_AtT(obj->nodesToMerge, i, kXmlItem);

        kCheck(kXml_CopyItem(xml, item, kNULL, obj->xml, mergeItem, kNULL));
    }

    return kOK;
}

GoFx(kSize) GoExtParams_ParameterCount(GoExtParams params)
{
    kObj(GoExtParams, params);
    
    return kArrayList_Count(obj->params);
}

GoFx(GoExtParam) GoExtParams_ParameterAt(GoExtParams params, kSize index)
{
    kObj(GoExtParams, params);

    if (index >= kArrayList_Count(obj->params))
    {
        return kNULL;
    }

    return *kArrayList_AtT(obj->params, index, GoExtParam);
}

GoFx(GoExtParam) GoExtParams_FindParameterById(GoExtParams params, const kChar* id)
{
    kObj(GoExtParams, params);
    kSize i;

    for (i = 0; i < kArrayList_Count(obj->params); i++)
    {
        GoExtParam value = *kArrayList_AtT(obj->params, i, GoExtParam);

        if (strcmp(id, GoExtParam_Id(value)) == 0)
        {
            return value;
        }
    }

    return kNULL;
}
