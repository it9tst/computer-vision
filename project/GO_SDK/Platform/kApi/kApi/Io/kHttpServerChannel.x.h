/** 
 * @file    kHttpServerChannel.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_CHANNEL_X_H
#define K_API_HTTP_SERVER_CHANNEL_X_H

typedef struct kHttpServerChannelClass
{
    kObjectClass base; 
    kHttpServer server;                 //Parent. 
    kTcpClient client;                  //TCP connection. 
    kLock clientLock;                   //Provides atomic cancel/destroy/attach operations. 
    kThread thread;                     //Processing thread.
    kHttpServerRequest request;         //Request processor.
    kHttpServerResponse response;       //Response processor. 
} kHttpServerChannelClass;

kDeclareClassEx(k, kHttpServerChannel, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkHttpServerChannel_Construct(kHttpServerChannel* channel, kHttpServer server, kTcpClient client, kAlloc allocator); 

kFx(kStatus) xkHttpServerChannel_Init(kHttpServerChannel channel, kType type, kHttpServer server, kTcpClient client, kAlloc allocator); 
kFx(kStatus) xkHttpServerChannel_VRelease(kHttpServerChannel channel);

kFx(kStatus) xkHttpServerChannel_ThreadEntry(kHttpServerChannel channel); 
kFx(kStatus) xkHttpServerChannel_ProcessMessage(kHttpServerChannel channel); 
kFx(kStatus) xkHttpServerChannel_SendContinue(kHttpServerChannel channel); 

kFx(kStatus) xkHttpServerChannel_DestroyClient(kHttpServerChannel channel); 
kFx(kStatus) xkHttpServerChannel_CancelClient(kHttpServerChannel channel); 

kFx(kBool) xkHttpServerChannel_IsClosed(kHttpServerChannel channel); 
kFx(kTcpClient) xkHttpServerChannel_Client(kHttpServerChannel channel); 

kFx(kStatus) xkHttpServerChannel_NormalizeHeaderCaps(kChar* str); 

/* 
* Deprecated (Stage 1): not recommended for further use, but not yet announced via kDeprecate
*/

//[Deprecated] Was never a public method. Unclear why it would be needed by external code.
#define kHttpServerChannel_Cast_ xkHttpServerChannel_Cast 



#endif
