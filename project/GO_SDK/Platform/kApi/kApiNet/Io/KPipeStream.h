// 
// KPipeStream.h
// 
// Copyright (C) 2018-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_PIPE_STREAM_H
#define K_API_NET_PIPE_STREAM_H

#include <kApi/Io/kPipeStream.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Represents standard streams as KStream object.</summary>
            public ref class KPipeStream : public KStream
            {
                KDeclareClass(KPipeStream, kProcess)

            public:
                /// <summary>Initializes a new instance of the KPipeStream class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KPipeStream(IntPtr handle)
                    : KStream(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KPipeStream(IntPtr)" />
                ///
                /// <param name="refStyle">Ref style.</param>
                KPipeStream(IntPtr handle, KRefStyle refStyle)
                    : KStream(handle, refStyle)
                {}

                /// <summary>Return stdin stream as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                static property KStream^ StdIn
                {
                    KStream^ get()
                    {
                        kStream stream = kStdIn();
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }

                /// <summary>Return stdout stream as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                static property KStream^ StdOut
                {
                    KStream^ get()
                    {
                        kStream stream = kStdOut();
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }

                /// <summary>Return stderr stream as KStream.</summary>
                /// 
                /// <returns>Stream object.</returns>
                static property KStream^ StdErr
                {
                    KStream^ get()
                    {
                        kStream stream = kStdErr();
                        KAdjustRef(stream, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KStream^>(stream);
                    }
                }
            };
        }
    }
}

#endif
