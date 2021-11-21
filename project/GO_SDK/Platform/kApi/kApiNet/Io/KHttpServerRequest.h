// 
// KHttpServerRequest.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_HTTP_SERVER_REQUEST_H
#define K_API_NET_HTTP_SERVER_REQUEST_H

#include <kApi/Io/kHttpServerRequest.h>
#include "kApiNet/KAlloc.h"
#include "kApiNet/Data/KString.h"
#include "kApiNet/Io/KStream.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents a node within a kHttpServerRequest header list.</summary>           
            public ref struct KHttpServerRequestHeader
            {
            public:
                /// <summary> Gets a reference to the next list item.</summary>
                property KHttpServerRequestHeader^ Next
                {
                    KHttpServerRequestHeader^ get() { return KHttpServerRequestHeader::ToObject(m_request, kHttpServerRequest_NextHeader(m_request, m_header)); }
                }
                
                /// <summary>Gets the header field name.</summary>
                /// 
                /// <remarks>
                /// HTTP header field names are case-insensitive. To avoid ambiguity, the kHttpServerRequest
                /// class converts all header names to Pascal caps (e.g. "Content-Length").
                /// </remarks>
                property String^ Name
                {
                    String^ get() { return KToString(kHttpServerRequest_HeaderName(m_request, m_header)); }
                }

                /// <summary>Gets the header field value.</summary>
                property String^ Value
                {
                    String^ get() { return KToString(kHttpServerRequest_HeaderValue(m_request, m_header)); }
                }

            internal:

                KHttpServerRequestHeader(kHttpServerRequest request, kPointer header)
                    : m_request(request), m_header(header) {}

                static kPointer ToHandle(KHttpServerRequestHeader^ it)
                {
                    return (it == nullptr) ? kNULL : it->m_header;
                }

                static KHttpServerRequestHeader^ ToObject(kHttpServerRequest request, kPointer header)
                {
                    return (header == kNULL) ? nullptr : gcnew KHttpServerRequestHeader(request, header);
                }

            private:
                kHttpServerRequest m_request;
                kPointer m_header;
            };

            /// <summary>Supports HTTP server request parsing.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: None</para>
            /// </remarks>
            public ref class KHttpServerRequest : public KObject
            {
                KDeclareNoneClass(KHttpServerRequest, kHttpServerRequest)

            public:
                /// <summary>Initializes a new instance of the KHttpServerRequest class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KHttpServerRequest(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// Gets a string representing the HTTP request method (e.g. "GET", "POST").
                property String^ Method
                {
                    String^ get() { return KToString(kHttpServerRequest_Method(Handle)); }
                }

                /// <summary>Gets the HTTP request URI (e.g., /resources/page.html).</summary>
                /// 
                /// <remarks>
                /// The URI can be in absolute URI form (e.g., http://www.example.com/index.html) or absolute path form (e.g., /index.html).
                /// Use the UriPath property to access the URI in absolute path form.
                /// </remarks>
                property String^ Uri
                {
                    String^ get() { return KToString(kHttpServerRequest_Uri(Handle)); }
                }

                /// <summary>Gets the HTTP request URI in absolute path form (e.g., /resources/page.html).</summary>
                property String^ UriPath
                {
                    String^ get() { return KToString(kHttpServerRequest_UriPath(Handle)); }
                }

                /// <summary>Gets the HTTP version associated with this request.</summary>
                property KVersion Version
                {
                    KVersion get() { return (KVersion)kHttpServerRequest_Version(Handle); }
                }

                /// <summary>Gets the total count of headers parsed from this request.</summary>
                /// 
                /// <remarks>
                /// The headers reported by this property can include both leading and trailing headers. However, trailing headers
                /// are only reported after the final segment of a chunk-encoded message is parsed.
                /// </remarks>
                property k32s HeaderCount
                {
                    k32s get() { return (k32s)kHttpServerRequest_HeaderCount(Handle); }
                }

                /// <summary>Gets the first header.</summary>
                property KHttpServerRequestHeader^ FirstHeader
                {
                    KHttpServerRequestHeader^ get() { return KHttpServerRequestHeader::ToObject(Handle, kHttpServerRequest_FirstHeader(Handle));  }
                }

                /// <summary>Finds the header field value associated with the given header field name, if present.</summary>
                /// 
                /// <param name="name">Header field name.</param>
                /// <returns>Header field value.</returns>
                /// <exception cref="KException">Thrown if not found.</exception>
                String^ FindHeaderValue(String^ name)
                {
                    String^ value = TryFindHeaderValue(name); 

                    KCheckErr((value != nullptr), kERROR_NOT_FOUND); 

                    return value; 
                }

                /// <summary>Attempts to find the header field value associated with the given header field name. </summary>
                /// 
                /// <param name="name">Header field name.</param>
                /// <returns>Header field value, or null if not found.</returns>
                String^ TryFindHeaderValue(String^ name)
                {
                    KString nameStr(name);

                    return KToString(kHttpServerRequest_FindHeaderValue(Handle, nameStr.CharPtr)); 
                }

                /// <summary>Reports whether the request has an associated message body.</summary>
                property bool HasBody
                {
                    bool get() { return KToBool(kHttpServerRequest_HasBody(Handle)); }
                }

                /// <summary>Reports whether the message body is chunk-encoded.</summary>
                property bool IsChunkCoded
                {
                    bool get() { return KToBool(kHttpServerRequest_IsChunkCoded(Handle)); }
                }

                /// <summary>Reports the total message length for a simple (non-chunk-encoded) message.</summary>
                property k64s ContentLength
                {
                    k64s get() { return kHttpServerRequest_ContentLength(Handle); }
                }

                /// <summary>Begins reading the message body.</summary>
                /// 
                /// <remarks>
                /// <para>For simple messages, call this method once to receive the total message length and a reference to
                /// a stream object that can be used to read the entire message.</para>
                /// 
                /// <para>For chunk-encoded messages, call this method to receive the length of the next chunk and a reference to
                /// a stream object that can be used to read the chunk. Each individual chunk must be read out before this
                /// method can be used to learn about the next chunk.  Reading is complete when a length of zero is reported
                /// by this method.</para>
                ///
                /// <para>The returned stream is owned by this object and should not be explicitly disposed.</para>
                /// </remarks>
                /// 
                /// <param name="length">Receives the amount of data, in bytes, that should be read from the stream.</param>
                /// <return>KStream object that should be used to read message content.</return>
                KStream^ BeginRead([Out] k64s% length)
                {
                    kStream stream = kNULL; 
                    k64u len; 

                    KCheck(kHttpServerRequest_BeginRead(Handle, &len, &stream)); 

                    length = (k64s)len; 

                    return KToObject<KStream^>(stream, KRefStyle::None);
                }

                /// <summary>Reports whether an upgrade to websocket protocol is requested.</summary>
                property bool IsWebSocketUpgrade
                {
                    bool get() {return KToBool(kHttpServerRequest_IsWebSocketUpgrade(Handle));};
                }

            protected:
                KHttpServerRequest() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif
