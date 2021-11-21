/** 
 * @file    kHttpServer.x.h
 *
 * @internal
 * Copyright (C) 2013-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_HTTP_SERVER_X_H
#define K_API_HTTP_SERVER_X_H

#define xkHTTP_SERVER_DEFAULT_CHANNEL_CAPACITY           (10)            //Default maximum simultaneous connections.
#define xkHTTP_SERVER_DEFAULT_LISTEN_BACKLOG             (16)            //Default backlog parameter used with tcp listen function.
#define xkHTTP_SERVER_DEFAULT_PORT                       (80)            //Default server port.
#define xkHTTP_SERVER_QUIT_QUERY_PERIOD                  (50000)         //Interval to check for quit flag. 

#define xkHTTP_SERVER_DEFAULT_SOCKET_READ_BUFFER         (32*1024)          //Connection socket read buffer size. 
#define xkHTTP_SERVER_DEFAULT_CLIENT_READ_BUFFER         (32*1024)          //Connection stream read buffer size.
#define xkHTTP_SERVER_DEFAULT_SOCKET_WRITE_BUFFER        (32*1024)          //Connection socket write buffer size. 
#define xkHTTP_SERVER_DEFAULT_CLIENT_WRITE_BUFFER        (32*1024)          //Connection stream write buffer size.

typedef struct kHttpServerClass
{
    kObjectClass base; 
    kIpEndPoint localEndPoint;          //Local end-point information.
    kSize channelCapacity;              //Maximum simultaneous connections. 
    kSSize socketWriteBufferSize;       //Socket write buffer size for accepted sockets.
    kSSize clientWriteBufferSize;       //Client write buffer size for accepted sockets.
    kSSize socketReadBufferSize;        //Socket read buffer size for accepted sockets.
    kSSize clientReadBufferSize;        //Client read buffer size for accepted sockets.
    kCallback handler;                  //Request callback. 
    kTcpServer listener;                //TCP server socket. 
    kThread thread;                     //Thread to accept connections.
    kAtomic32s shouldQuit;              //Thread quit flag. 
    kList channels;                     //Remote connections -- kList<kHttpServerChannel>.
    kSize backlog;                      //Backlog size.
} kHttpServerClass;

kDeclareClassEx(k, kHttpServer, kObject)

/* 
* Private methods. 
*/

kFx(kStatus) xkHttpServer_Init(kHttpServer server, kType type, kAlloc allocator); 
kFx(kStatus) xkHttpServer_VRelease(kHttpServer server);

kFx(kStatus) xkHttpServer_ThreadEntry(kHttpServer server); 

kFx(kStatus) xkHttpServer_AddChannel(kHttpServer server, kHttpServerChannel channel); 
kFx(kStatus) xkHttpServer_CullChannels(kHttpServer server); 

kFx(kCallback) xkHttpServer_Handler(kHttpServer server); 

#endif
