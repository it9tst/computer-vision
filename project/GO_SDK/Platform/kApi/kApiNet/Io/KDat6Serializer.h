// 
// KDat6Serializer.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_DAT6_SERIALIZER_H
#define K_API_NET_DAT6_SERIALIZER_H

#include <kApi/Io/kDat6Serializer.h>
#include "kApiNet/Data/KString.h"
#include "kApiNet/Io/KSerializer.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Io
        {
            /// <summary>Serializes/deserializes objects using kDat6 format.</summary>
            ///
            /// <remarks>
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            public ref class KDat6Serializer : public KSerializer
            {
                KDeclareAutoClass(KDat6Serializer, kDat6Serializer)

            public:
                /// <summary>Initializes a new instance of the KDat6Serializer class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KDat6Serializer(IntPtr handle)
                    : KSerializer(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KDat6Serializer(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KDat6Serializer(IntPtr handle, KRefStyle refStyle)
                    : KSerializer(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KDat6Serializer class.</summary>           
                /// 
                /// <param name="stream">Stream for reading or writing.</param>
                KDat6Serializer(KStream^ stream)
                    : KSerializer(DefaultRefStyle)
                {
                    kDat6Serializer handle = kNULL;

                    KCheck(kDat6Serializer_Construct(&handle, KToHandle(stream), kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KDat6Serializer(KStream^)" />
                /// <param name="allocator">Memory allocator.</param>
                KDat6Serializer(KStream^ stream, KAlloc^ allocator)
                    : KSerializer(DefaultRefStyle)
                {
                    kDat6Serializer handle = kNULL;

                    KCheck(kDat6Serializer_Construct(&handle, KToHandle(stream), KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <inheritdoc cref="KDat6Serializer(KStream^, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KDat6Serializer(KStream^ stream, KAlloc^ allocator, KRefStyle refStyle)
                    : KSerializer(DefaultRefStyle)
                {
                    kDat6Serializer handle = kNULL;

                    KCheck(kDat6Serializer_Construct(&handle, KToHandle(stream), KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <summary>Loads an object from file using kDat6 format.</summary>
                /// 
                /// <typeparam name="T">Deserialized object will be cast to this type.</typeparam>
                /// <param name="filePath">Path of the file to load.</param>
                /// <returns>Loaded object.</returns>
                generic <typename T>
                static T LoadObject(String^ filePath)
                {
                    return LoadObject<T>(filePath, nullptr); 
                }

                /// <inheritdoc cref="LoadObject(String^)" />
                /// <param name="allocator">Memory allocator for loaded object.</param>
                generic <typename T> 
                static T LoadObject(String^ filePath, KAlloc^ allocator)
                {
                    KString pathStr(filePath); 
                    kObject object = kNULL; 

                    KCheck(kSerializer_LoadObject(&object, kTypeOf(kDat6Serializer), pathStr.CharPtr, KToHandle(allocator))); 

                    try
                    {
                        return KToObject<T>(object);
                    }
                    catch (...)
                    {
                        kObject_Dispose(object);
                        throw;
                    }
                }

                /// <summary>Saves an object to file using kDat6 format.</summary>
                /// 
                /// <param name="object">Object to be serialized.</param>
                /// <param name="filePath">Path of the file to save.</param>
                /// <returns>Operation status.</returns>
                static void SaveObject(KObject^ object, String^ filePath)
                {
                    KString pathStr(filePath);

                    KCheck(kSerializer_SaveObject(KToHandle(object), kTypeOf(kDat6Serializer), pathStr.CharPtr));
                }

                /// <summary>
                /// Enables the use of compression in serialization. 
                /// </summary>
                /// <param name="algorithm">Compression algorithm type(null to disable).</param>
                /// <param name="level">Compression level; accepts kCompressionPreset value or compressor - specific value.</param>
                void EnableCompression(KCompressionType algorithm, k32s level)
                {
                    KCheck(kDat6Serializer_EnableCompression(Handle, algorithm, level));
                }

            protected:
                KDat6Serializer() : KSerializer(DefaultRefStyle) {}
            };

        }
    }
}

#endif
