/** 
 * @file    kHttpServerRequest.cpp
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Io/kHttpServerRequest.h>
#include <kApi/Data/kArrayList.h>
#include <kApi/Data/kList.h>
#include <kApi/Data/kMap.h>
#include <kApi/Data/kString.h>
#include <kApi/Io/kHttpServer.h>
#include <kApi/Io/kHttpServerChannel.h>
#include <kApi/Io/kTcpClient.h>
#include <stdio.h>

kBeginClassEx(k, kHttpServerRequest)
    kAddPrivateVMethod(kHttpServerRequest, kObject, VRelease)
kEndClassEx()

kFx(kStatus) xkHttpServerRequest_Construct(kHttpServerRequest* request, kHttpServerChannel channel, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kType type = kTypeOf(kHttpServerRequest); 
    kStatus status; 

    kCheck(kAlloc_GetObject(alloc, type, request)); 

    if (!kSuccess(status = xkHttpServerRequest_Init(*request, type, channel, alloc)))
    {
        kAlloc_FreeRef(alloc, request); 
    }

    return status; 
} 

kFx(kStatus) xkHttpServerRequest_Init(kHttpServerRequest request, kType type, kHttpServerChannel channel, kAlloc alloc)
{
    kObjR(kHttpServerRequest, request); 
    kStatus status; 

    kCheck(kObject_Init(request, type, alloc)); 

    obj->channel = channel; 
    obj->client = xkHttpServerChannel_Client(channel); 
    obj->headerLines = kNULL;
    obj->headers = kNULL;
    obj->method = kNULL;
    obj->uri = kNULL;
    obj->uriPath = kNULL;
    obj->versionStr = kNULL;
    obj->version = 0;
    obj->contentLength = 0;
    obj->isChunkCoded = kFALSE;
    obj->expectContinue = kFALSE;
    obj->contentComplete = kFALSE;
    obj->chunkIndex = 0;
    obj->chunkLine = kNULL;
    obj->findName = kNULL;
    obj->tempStr = kNULL;
    
    kTry
    {
        kTest(kList_Construct(&obj->headerLines, kTypeOf(kString), xkHTTP_SERVER_REQUEST_HEADER_CAPACITY, alloc));    
        kTest(kMap_Construct(&obj->headers, kTypeOf(kString), kTypeOf(kString), xkHTTP_SERVER_REQUEST_HEADER_CAPACITY, alloc));    

        kTest(kString_Construct(&obj->findName, kNULL, alloc)); 
        kTest(kString_Construct(&obj->chunkLine, kNULL, alloc)); 
        kTest(kString_Construct(&obj->tempStr, kNULL, alloc)); 
    }
    kCatch(&status)
    {
        xkHttpServerRequest_VRelease(request); 
        kEndCatch(status); 
    }

    return kOK;     
}

kFx(kStatus) xkHttpServerRequest_VRelease(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    kCheck(kObject_Dispose(obj->headerLines));  
    kCheck(kObject_Dispose(obj->headers));  

    kCheck(kObject_Destroy(obj->method)); 
    kCheck(kObject_Destroy(obj->uri)); 
    kCheck(kObject_Destroy(obj->uriPath)); 
    kCheck(kObject_Destroy(obj->versionStr)); 

    kCheck(kObject_Destroy(obj->findName));  
    kCheck(kObject_Destroy(obj->chunkLine));  
    kCheck(kObject_Destroy(obj->tempStr));  

    kCheck(kObject_VRelease(request)); 

    return kOK;   
}

kFx(kStatus) xkHttpServerRequest_Begin(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    kCheck(xkHttpServerRequest_Clear(request)); 

    kCheck(xkHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 

    kCheck(xkHttpServerRequest_ParseRequestLine(request)); 

    kCheck(xkHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 

    kCheck(xkHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
    kCheck(xkHttpServerRequest_ParseKnownHeaders(request)); 

    return kOK;   
}

kFx(kStatus) xkHttpServerRequest_Clear(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    kCheck(kList_Purge(obj->headerLines)); 
    kCheck(kMap_Purge(obj->headers)); 
    
    kCheck(kDisposeRef(&obj->method)); 
    kCheck(kDisposeRef(&obj->uri)); 
    kCheck(kDisposeRef(&obj->uriPath)); 
    kCheck(kDisposeRef(&obj->versionStr)); 

    obj->contentLength = -1; 
    obj->isChunkCoded = kFALSE; 
    obj->contentComplete = kFALSE; 
    obj->chunkIndex = 0; 
    obj->expectContinue = kFALSE; 
    
    return kOK;
}

kFx(kStatus) xkHttpServerRequest_ReadHeaderLines(kHttpServerRequest request, kList headerLines)
{
    kString line = kNULL; 

    kCheck(kList_Purge(headerLines)); 

    do 
    {
        kCheck(kString_Construct(&line, kNULL, kObject_Alloc(request))); 
        kCheck(kList_AddT(headerLines, &line, kNULL)); 

        kCheck(xkHttpServerRequest_ReadLine(request, line)); 
    }
    while ((kString_Length(line) != 0) && (kList_Count(headerLines) < xkHTTP_SERVER_REQUEST_HEADER_CAPACITY)); 

    if (kString_Length(line) == 0)
    {
        kCheck(kList_Remove(headerLines, kList_Last(headerLines))); 
        kCheck(kObject_Destroy(line)); 
        return kOK; 
    }
    else
    {
        return kERROR_INCOMPLETE; 
    }
}

kFx(kStatus) xkHttpServerRequest_CoalesceHeaderLines(kHttpServerRequest request, kList headerLines)
{
    kListItem first = kList_First(headerLines); 
    kListItem second = kIsNull(first) ? kNULL : kList_Next(headerLines, first); 

    while (!kIsNull(second))
    {
        kString firstStr = kList_AsT(headerLines, first, kString); 
        kString secondStr = kList_AsT(headerLines, second, kString); 
        kChar* secondChars = kString_Chars(secondStr); 
       
        if ((kString_Length(secondStr) > 0) && kChar_IsSpace(secondChars[0]))
        {
            kCheck(kString_Add(firstStr, secondChars)); 
            
            kCheck(kList_Remove(headerLines, second)); 
            kCheck(kObject_Destroy(secondStr)); 
        }
        else
        {
            first = second; 
        }

        second = kList_Next(headerLines, first); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ParseRequestLine(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 
    kListItem lineItem = kList_First(obj->headerLines); 
    kString line = kIsNull(lineItem) ? kNULL : kList_AsT(obj->headerLines, lineItem, kString); 
    kArrayList tokens = kNULL; 
    const kChar* versionNumStr = kNULL; 
    k32u versionMajor, versionMinor; 

    kCheckTrue(!kIsNull(line), kERROR_FORMAT); 

    kTry
    {
        kTest(kList_Remove(obj->headerLines, lineItem)); 

        kTest(kString_Split(line, " ", &tokens, kObject_Alloc(request))); 
        kTestTrue(kArrayList_Count(tokens) == 3, kERROR_FORMAT); 

        obj->method = kArrayList_AsT(tokens, 0, kString); 
        obj->uri = kArrayList_AsT(tokens, 1, kString); 
        obj->versionStr = kArrayList_AsT(tokens, 2, kString); 

        kArrayList_Clear(tokens);           
        
        kTest(kString_Construct(&obj->uriPath, kNULL, kObject_Alloc(request))); 
        kTest(xkHttpServerRequest_ParseUriPath(request, obj->uri, obj->uriPath));       

        versionNumStr = kStrFindFirst(kString_Chars(obj->versionStr), "/"); 

        kTestTrue(!kIsNull(versionNumStr), kERROR_FORMAT); 

        if (sscanf(++versionNumStr, "%u.%u", &versionMajor, &versionMinor) != 2)          
        {
            kThrow(kERROR_FORMAT); 
        }

        obj->version = kVersion_Create(versionMajor, versionMinor, 0, 0); 
    }
    kFinally
    {
        kObject_Destroy(line); 
        kObject_Dispose(tokens); 
        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ParseUriPath(kHttpServerRequest request, kString uri, kString uriPath)
{
    const kChar* separator; 

    if (!kIsNull(separator = kStrFindFirst(kString_Chars(uri), "://")))
    {
        if (!kIsNull(separator = kStrFindFirst(separator + 3, "/")))
        {
            //uri was in absolute form
            kCheck(kString_Set(uriPath, separator)); 
        }
        else
        {
            //uri was root in absolute form, but missing root slash
            kCheck(kString_Set(uriPath, "/")); 
        }
    }
    else
    {
        //uri was already in path-form
        kCheck(kString_Assign(uriPath, uri)); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ParseHeaders(kHttpServerRequest request, kList headerLines, kMap headerMap)
{
    kListItem listIt = kList_First(headerLines);

    while (!kIsNull(listIt))
    {
        kString str = kList_AsT(headerLines, listIt, kString); 
       
        kCheck(xkHttpServerRequest_ParseHeader(request, str, headerMap)); 

        listIt = kList_Next(headerLines, listIt); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ParseHeader(kHttpServerRequest request, kString str, kMap headerMap)
{
    kChar* delim = (kChar*) kStrFindFirst(kString_Chars(str), ":"); 
    kMapItem mapItem = kNULL; 
    kString name = kNULL; 
    kString value = kNULL; 

    kCheckTrue(!kIsNull(delim), kERROR_FORMAT); 
    delim[0] = 0; 

    kTry
    {
        kTest(kString_Construct(&name, kString_Chars(str), kObject_Alloc(request))); 
        kTest(kString_Trim(name)); 
        kTest(xkHttpServerChannel_NormalizeHeaderCaps(kString_Chars(name))); 

        kTest(kString_Construct(&value, ++delim, kObject_Alloc(request))); 
        kTest(kString_Trim(value)); 

        if (kSuccess(kMap_FindItemT(headerMap, &name, &mapItem)))
        {
            kString existing = kMap_ValueAsT(headerMap, mapItem, kString); 

            kTest(kString_Addf(existing, ",%s", kString_Chars(value))); 
        }
        else
        {
            kTest(kMap_AddT(headerMap, &name, &value)); 
            name = value = kNULL; 
        }
    }
    kFinally
    {
        kObject_Destroy(name); 
        kObject_Destroy(value); 

        kEndFinally(); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ParseKnownHeaders(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 
    const kChar* str = kNULL; 

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Content-Length")))
    {
        sscanf(str, "%lld", &obj->contentLength); 
    }

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Transfer-Encoding")))
    {
        obj->isChunkCoded = kTRUE; 
    }

    if (!kIsNull(str = kHttpServerRequest_FindHeaderValue(request, "Expect")))
    {
        kCheck(kString_Set(obj->tempStr, str)); 
        kCheck(kStrToLower(kString_Chars(obj->tempStr))); 

        obj->expectContinue = !kIsNull(kStrFindFirst(kString_Chars(obj->tempStr), "100-continue")); 
    }

    return kOK; 
}

kFx(kStatus) xkHttpServerRequest_ReadLine(kHttpServerRequest request, kString line)
{
    kObj(kHttpServerRequest, request); 
    const kSize EOL_LENGTH = 2; 
    kSize length = 0; 
    kChar* buffer = 0; 
    kBool isComplete = kFALSE; 

    kCheck(kString_Clear(line)); 
    kCheck(kString_Reserve(line, 64)); 
    buffer = kString_Chars(line); 

    do
    {
        if (length == kString_Capacity(line))
        {
            kCheck(kString_Reserve(line, 2*length)); 
            buffer = kString_Chars(line); 
        }

        kCheck(kStream_Read(obj->client, &buffer[length], 1)); 
        length++; 

        isComplete = (length >= EOL_LENGTH) && (buffer[length-EOL_LENGTH] == '\r') && (buffer[length-EOL_LENGTH+1] == '\n'); 
    }
    while (!isComplete && (length < xkHTTP_SERVER_REQUEST_LINE_CAPACITY)); 
    
    kCheckTrue(isComplete, kERROR_INCOMPLETE); 
    
    kCheck(kString_SetLength(line, length - EOL_LENGTH));
   
    return kOK; 
}
    
kFx(const kChar*) kHttpServerRequest_Method(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return kString_Chars(obj->method); 
}

kFx(const kChar*) kHttpServerRequest_Uri(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return kString_Chars(obj->uri); 
}

kFx(const kChar*) kHttpServerRequest_UriPath(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return kString_Chars(obj->uriPath); 
}

kFx(kVersion) kHttpServerRequest_Version(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return obj->version; 
}

kFx(kSize) kHttpServerRequest_HeaderCount(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return kMap_Count(obj->headers); 
}

kFx(kPointer) kHttpServerRequest_FirstHeader(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return kMap_First(obj->headers); 
}

kFx(kPointer) kHttpServerRequest_NextHeader(kHttpServerRequest request, kPointer header)
{
    kObj(kHttpServerRequest, request); 

    return kMap_Next(obj->headers, header); 
}

kFx(const kChar*) kHttpServerRequest_HeaderName(kHttpServerRequest request, kPointer header)
{
    kObj(kHttpServerRequest, request); 
    kString name = kMap_KeyAsT(obj->headers, header, kString); 

    return kString_Chars(name); 
}

kFx(const kChar*) kHttpServerRequest_HeaderValue(kHttpServerRequest request, kPointer header)
{
    kObj(kHttpServerRequest, request); 
    kString value = kMap_ValueAsT(obj->headers, header, kString); 

    return kString_Chars(value); 
}

kFx(const kChar*) kHttpServerRequest_FindHeaderValue(kHttpServerRequest request, const kChar* name)
{
    kObj(kHttpServerRequest, request); 
    kString valueStr = kNULL; 

    if (kSuccess(kString_Set(obj->findName, name)))
    {
        xkHttpServerChannel_NormalizeHeaderCaps(kString_Chars(obj->findName)); 

        if (kSuccess(kMap_FindT(obj->headers, &obj->findName, &valueStr)))
        {
            return kString_Chars(valueStr); 
        }
    }

    return kNULL; 
}

kFx(kBool) kHttpServerRequest_HasBody(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return (obj->contentLength >= 0) || obj->isChunkCoded; 
}

kFx(k64s) kHttpServerRequest_ContentLength(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return obj->contentLength; 
}

kFx(kBool) kHttpServerRequest_IsChunkCoded(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return obj->isChunkCoded; 
}

kFx(kStatus) xkHttpServerRequest_ExpectsContinue(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 

    return obj->expectContinue; 
}

kFx(kStatus) xkHttpServerRequest_ParseWebSocketUpgrade(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 
    const kChar* value = kNULL;
   
    kCheckTrue(kStrEquals(kHttpServerRequest_Method(request), "GET"), kERROR_FORMAT); 

    kCheckTrue(kVersion_Compare(kHttpServerRequest_Version(request), kVersion_Create(1, 1, 0, 0)) == 0, kERROR_FORMAT); 

    kCheckTrue(!kIsNull(value = kHttpServerRequest_FindHeaderValue(request, "Upgrade")), kERROR_FORMAT); 

    kCheckTrue(kStrCompareLower(value, "websocket") == 0, kERROR_FORMAT); 

    kCheckTrue(!kIsNull(value = kHttpServerRequest_FindHeaderValue(request, "Connection")), kERROR_FORMAT); 

    kCheck(kString_Set(obj->tempStr, value)); 
    kCheck(kStrToLower(kString_Chars(obj->tempStr))); 

    kCheckTrue(kStrFindFirst(kString_Chars(obj->tempStr), "upgrade") != kNULL, kERROR_FORMAT); 

    return kTRUE;
}

kFx(kBool) kHttpServerRequest_IsWebSocketUpgrade(kHttpServerRequest request)
{
    kObj(kHttpServerRequest, request); 
   
    return kSuccess(xkHttpServerRequest_ParseWebSocketUpgrade(request));
}

kFx(kStatus) kHttpServerRequest_BeginRead(kHttpServerRequest request, k64u* length, kStream* stream)
{
    kObj(kHttpServerRequest, request); 

    *length = 0; 
    *stream = obj->client; 

    kCheckState(kHttpServerRequest_HasBody(request)); 
    
    if (!obj->contentComplete)
    {
        if (!obj->isChunkCoded)
        {
            *length = (k64u)obj->contentLength; 
            obj->contentComplete = kTRUE; 
        }
        else
        {
            if (obj->chunkIndex != 0)
            {
                //read terminating CRLF from previous chunk
                kCheck(xkHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            }

            //read chunk descriptor; ignore extensions
            kCheck(xkHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            kCheckTrue(sscanf(kString_Chars(obj->chunkLine), "%llX", length) == 1, kERROR_FORMAT); 

            if (*length == 0)
            {
                obj->contentComplete = kTRUE; 
                
                //read optional trailing headers
                kCheck(xkHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 
                kCheck(xkHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 
                kCheck(xkHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
            }

            obj->chunkIndex++; 
        }
    }

    return kOK; 
}

kFx(kStatus) kHttpServerRequest_BeginReadChunk(kHttpServerRequest request, k64u* length, kStream* stream)
{
    kObj(kHttpServerRequest, request); 

    *length = 0; 
    *stream = obj->client; 

    kCheckState(kHttpServerRequest_HasBody(request)); 
    
    if (!obj->contentComplete)
    {
        if (!obj->isChunkCoded)
        {
            *length = (k64u)obj->contentLength; 
            obj->contentComplete = kTRUE; 
        }
        else
        {
            if (obj->chunkIndex != 0)
            {
                //read terminating CRLF from previous chunk
                kCheck(xkHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            }

            //read chunk descriptor; ignore extensions
            kCheck(xkHttpServerRequest_ReadLine(request, obj->chunkLine)); 
            kCheckTrue(sscanf(kString_Chars(obj->chunkLine), "%llX", length) == 1, kERROR_FORMAT); 

            if (*length == 0)
            {
                obj->contentComplete = kTRUE; 

                //read terminating CRLF from final chunk
                kCheck(xkHttpServerRequest_ReadLine(request, obj->chunkLine)); 
                
                //read optional trailing headers
                kCheck(xkHttpServerRequest_ReadHeaderLines(request, obj->headerLines)); 
                kCheck(xkHttpServerRequest_CoalesceHeaderLines(request, obj->headerLines)); 
                kCheck(xkHttpServerRequest_ParseHeaders(request, obj->headerLines, obj->headers)); 
            }

            obj->chunkIndex++; 
        }
    }

    return kOK; 
}

