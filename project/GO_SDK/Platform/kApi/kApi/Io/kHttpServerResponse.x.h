/** 
 * @file    kHttpServerResponse.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_RESPONSE_X_H
#define K_API_HTTP_SERVER_RESPONSE_X_H

kDeclareEnumEx(k, kHttpStatus, kValue)

typedef struct kHttpServerResponseClass
{
    kObjectClass base; 
    kHttpServerChannel channel;         //Parent. 
    kTcpClient client;                  //TCP connection (belongs to parent channel). 

    kMap headers;                       //Maps header field names to values -- kMap<kString, kString>.

    kVersion version;                   //HTTP version.  
    kHttpStatus status;                 //HTTP status code.  
    kString reason;                     //HTTP status reason. 
    kBool shouldClose;                  //Should the connection be closed after this response?
    kBool isChunkCoded;                 //Is the response body chunk-coded?
    k64s contentLength;                 //Content length header value (or -1, if not present). 

    kBool messageStarted;               //Have headers been written?
    kSize chunkIndex;                   //Current chunk index, used during body formatting.

    kString line;                       //Temp variable used for output formatting.

} kHttpServerResponseClass;

kDeclareClassEx(k, kHttpServerResponse, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkHttpServerResponse_Construct(kHttpServerResponse* response, kHttpServerChannel channel, kAlloc allocator); 

kFx(kStatus) xkHttpServerResponse_Init(kHttpServerResponse response, kType type, kHttpServerChannel channel, kAlloc alloc); 
kFx(kStatus) xkHttpServerResponse_VRelease(kHttpServerResponse response);

kFx(kStatus) xkHttpServerResponse_Begin(kHttpServerResponse response);
kFx(kStatus) xkHttpServerResponse_Clear(kHttpServerResponse response);

kFx(kStatus) xkHttpServerResponse_SetContentLength(kHttpServerResponse response, k64u length); 
kFx(kStatus) xkHttpServerResponse_EnableChunkCoding(kHttpServerResponse response, kBool enabled); 

kFx(kStatus) xkHttpServerResponse_BeginWriteMessage(kHttpServerResponse response); 

kFx(kStatus) xkHttpServerResponse_AddKnownHeaders(kHttpServerResponse response); 
const kChar* xkHttpServerResponse_DefaultReason(kHttpServerResponse response, kHttpStatus status); 

kFx(kStatus) xkHttpServerResponse_FormatStatusLine(kHttpServerResponse response); 
kFx(kStatus) kHttpServerResponse_FormatHeaders(kHttpServerResponse response, kMap headerMap); 

kFx(kStatus) xkHttpServerResponse_End(kHttpServerResponse response);

kFx(kBool) xkHttpServerResponse_MessageStarted(kHttpServerResponse response);
kFx(kBool) xkHttpServerResponse_Closed(kHttpServerResponse response); 

#endif
