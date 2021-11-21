//
// KImage.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_IMAGE_H
#define K_API_NET_IMAGE_H

#include <kApi/Data/kImage.h>
#include <kApi/Io/kPath.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Represents a 2D collection of pixels.</summary>
            /// 
            /// <remarks>
            /// <para>The KImage class is similar to the KArray2 class, with some important differences:
            /// <list type="bullet">
            ///   <item><description>By default, image memory is allocated such that rows are aligned to 8-byte boundaries.</description></item>
            ///   <item><description>Pixels are limited to value types.</description></item>
            ///   <item><description>Image indices/dimensions are given in (column, row) rather than (row, column) order.</description></item>
            ///   <item><description>kImage contains additional image-specific attributes (e.g. color filter array type).</description></item>
            /// </list>
            /// </para>
            ///
            /// <para>KImage supports the KObject.Clone and KObject.Size methods.</para>
            ///
            /// <para>KImage supports the kdat5 and kdat6 serialization protocols.</para>        
            ///
            /// <para>Default KRefStyle: Auto</para>
            /// </remarks>
            ///
            /// <example>This example illustrates iterating over the pixels in an image.
            /// <code language="C#" source="examples/Data/KImage_Get.cs" />
            /// </example> 
            public ref class KImage : public KObject
            {
                KDeclareAutoClass(KImage, kImage)

            public:
                /// <summary>Initializes a new instance of the KImage class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KImage(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <inheritdoc cref="KImage(IntPtr)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KImage(IntPtr handle, KRefStyle refStyle)
                    : KObject(handle, refStyle)
                {}

                /// <summary>Initializes a new instance of the KImage class without specifying a pixel type.</summary>
                KImage()
                    : KObject(DefaultRefStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_Construct(&handle, kTypeOf(kVoid), 0, 0, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KImage class with the specified pixel type and dimensions.</summary>
                /// 
                /// <param name="pixelType">Pixel type (must be a value type).</param>
                /// <param name="width">Image width.</param>
                /// <param name="height">Image height.</param>
                KImage(KType^ pixelType, k64s width, k64s height)
                    : KObject(DefaultRefStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_Construct(&handle, KToHandle(pixelType), (kSize)width, (kSize)height, kNULL));

                    Handle = handle;
                }

                /// <summary>Initializes a new instance of the KImage class with the specified pixel type and dimensions.</summary>
                /// 
                /// <param name="pixelType">Pixel type (must be a value type).</param>
                /// <param name="width">Image width.</param>
                /// <param name="height">Image height.</param>
                /// <param name="alignment">Memory alignment for data.</param>
                KImage(KType^ pixelType, k64s width, k64s height, KMemoryAlignment alignment)
                    : KObject(DefaultRefStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_ConstructEx(&handle, KToHandle(pixelType), (kSize)width, (kSize)height, kNULL, kNULL, (kMemoryAlignment)alignment));

                    Handle = handle;
                }

                /// <inheritdoc cref="KImage(KType^, k64s, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                KImage(KType^ pixelType, k64s width, k64s height, KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_Construct(&handle, KToHandle(pixelType), (kSize)width, (kSize)height, KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <inheritdoc cref="KImage(KType^, k64s, k64s)" />
                /// <param name="allocator">Memory allocator.</param>
                /// <param name="alignment">Memory alignment for data.</param>
                KImage(KType^ pixelType, k64s width, k64s height, KAlloc^ allocator, KMemoryAlignment alignment)
                    : KObject(DefaultRefStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_ConstructEx(&handle, KToHandle(pixelType), (kSize)width, (kSize)height, KToHandle(allocator), KToHandle(allocator), (kMemoryAlignment)alignment));

                    Handle = handle;
                }

                /// <inheritdoc cref="KImage(KType^, k64s, k64s, KAlloc^)" />
                ///
                /// <param name="refStyle">RefStyle for this object.</param>
                KImage(KType^ pixelType, k64s width, k64s height, KAlloc^ allocator, KRefStyle refStyle)
                    : KObject(refStyle)
                {
                    kImage handle = kNULL;

                    KCheck(kImage_Construct(&handle, KToHandle(pixelType), (kSize)width, (kSize)height, KToHandle(allocator))); 

                    Handle = handle;
                }

                /// <summary>Loads an image from file.</summary>
                /// 
                /// <remarks>The file extension is used to determine to image format. Currently, only the BMP (.bmp) format is supported.</remarks>
                /// 
                /// <param name="fileName">File path.</param>
                static KImage^ Load(String^ fileName)
                {
                    return Load(fileName, nullptr); 
                }

                /// <summary>Loads an image from file.</summary>
                /// 
                /// <remarks>The file extension is used to determine to image format. Currently, only the BMP (.bmp) format is supported.</remarks>
                /// 
                /// <param name="fileName">File path.</param>
                /// <param name="allocator">Memory allocator.</param>
                static KImage^ Load(String^ fileName, KAlloc^ allocator)
                {
                    kChar textFileName[kPATH_MAX]; 
                    KImage^ image = gcnew KImage();
                    kImage handle = kNULL;

                    KToText(fileName, textFileName, kCountOf(textFileName)); 

                    KCheck(kImage_Import(&handle, textFileName, KToHandle(allocator))); 

                    image->Handle = handle;

                    return image; 
                }

                /// <summary>Saves an image to file.</summary>
                /// 
                /// <remarks>The file extension is used to determine to image format. Currently, only the BMP (.bmp) format is supported.</remarks>
                /// 
                /// <param name="fileName">File path.</param>
                void Save(String^ fileName)
                {
                    kChar textFileName[kPATH_MAX];

                    KToText(fileName, textFileName, kCountOf(textFileName));

                    KCheck(kImage_Export(Handle, textFileName)); 
                }

                /// <summary>Reallocates the internal pixel buffer.</summary>
                /// 
                /// <param name="pixelType">Type of pixel (values types only).</param>
                /// <param name="width">Image width, in pixels.</param>
                /// <param name="height">Image height, in pixels.</param>
                void Allocate(KType^ pixelType, k64s width, k64s height)
                {
                    KCheck(kImage_Allocate(Handle, KToHandle(pixelType), (kSize)width, (kSize)height)); 
                }

                /// <summary>Attaches the image to an external pixel buffer.</summary>
                /// 
                /// <remarks>Attached pixel buffers are not freed when the image is destroyed.</remarks>
                /// 
                /// <param name="pixels">Pointer to external pixel buffer.</param>
                /// <param name="pixelType">Type of pixel (values types only).</param>
                /// <param name="width">Image width, in pixels.</param>
                /// <param name="height">Image height, in pixels.</param>
                /// <param name="stride">Image stride (row size), in bytes.</param>
                void Attach(IntPtr pixels, KType^ pixelType, k64s width, k64s height, k64s stride)
                {
                    KCheck(kImage_Attach(Handle, pixels.ToPointer(), KToHandle(pixelType), (kSize)width, (kSize)height, (kSize)stride)); 
                }

                /// <summary>Copies a given source image into this image.</summary>
                /// 
                /// <param name="source">Source image to be copied.</param>
                void Assign(KImage^ source)
                {
                    KCheck(kImage_Assign(Handle, KToHandle(source))); 
                }

                /// <summary>Sets all pixel bits to zero.</summary>
                void Zero()
                {
                    KCheck(kImage_Zero(Handle)); 
                }

                /// <summary>Gets or sets the optional pixel format descriptor associated with this image.</summary>
                property KPixelFormat PixelFormat
                {
                    KPixelFormat get() { return kImage_PixelFormat(Handle); }

                    void set(KPixelFormat format) { KCheck(kImage_SetPixelFormat(Handle, format)); }
                }

                /// <summary>Gets or sets the color filter array type associated with this image.</summary>
                property KCfa Cfa
                {
                    KCfa get() { return kImage_Cfa(Handle); }

                    void set(KCfa cfa) { KCheck(kImage_SetCfa(Handle, cfa)); }
                }

                /// <summary>Sets the value of a pixel.</summary>
                /// 
                /// <remarks>This method is convenient in some contexts, but is not an efficient method to manipulate
                /// many pixels.  In performance-critical code, use kImage_RowAt to directly access the pixel buffer.</remarks>
                /// 
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="x">Column index.</param>
                /// <param name="y">Row index.</param>
                /// <param name="pixel">Pixel value to be copied into the image.</param>
                generic <typename T>
                void Set(k64s x, k64s y, T pixel)
                {
                    KCheckArgs(sizeof(T) == kImage_PixelSize(Handle));

                    KCheck(kImage_SetPixel(Handle, (kSize)x, (kSize) y, &pixel));
                }
                                
                /// <summary>Gets the specified pixel value.</summary>
                /// 
                /// <example>This example illustrates iterating over the pixels in an image.
                /// <code language="C#" source="examples/Data/KImage_Get.cs" />
                /// </example> 
                ///
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="x">Column index.</param>
                /// <param name="y">Row index.</param>
                /// <returns>Pixel value.</returns>
                generic <typename T>
                T Get(k64s x, k64s y)
                {
                    KCheckArgs(((kSize)x < kImage_Width(Handle)) && ((kSize)y < kImage_Height(Handle)));

                    void* item = kImage_At(Handle, (kSize)x, (kSize)y);
                    T pixel;

                    KCheckArgs(sizeof(T) == kImage_PixelSize(Handle));

                    kItemCopy(&pixel, item, sizeof(T));

                    return pixel;
                }

                /// <summary>Gets a pointer to the first row in the pixel buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(kImage_Data(Handle)); }
                }

                /// <summary>Gets the size, in bytes, of the pixel buffer.</summary>
                property k64s DataSize
                {
                    k64s get() { return (k64s)kImage_DataSize(Handle); }
                }

                /// <summary>Gets a pointer to the specified row in the pixel buffer.</summary>
                /// 
                /// <example>This example illustrates iterating over the pixels in an image.
                /// <code language="C#" source="examples/Data/KImage_RowAt.cs" />
                /// </example> 
                ///
                /// <param name="y">Row index.</param>
                /// <returns>Pointer to first pixel in row.</returns>
                IntPtr RowAt(k64s y)
                {
                    return IntPtr(kImage_RowAt(Handle, (kSize)y));
                }

                /// <summary>Gets the pixel type.</summary>
                property KType^ PixelType
                {
                    KType^ get() { return gcnew KType(kImage_PixelType(Handle)); }
                }

                /// <summary>Gets the pixel size.</summary>
                property k64s PixelSize
                {
                    k64s get() { return (k64s)kImage_PixelSize(Handle); }
                }

                /// <summary>Gets the width of the image, in pixels.</summary>
                property k64s Width
                {
                    k64s get() { return (k64s)kImage_Width(Handle); }
                }

                /// <summary>Gets the height of the image, in pixels.</summary>
                property k64s Height
                {
                    k64s get() { return (k64s)kImage_Height(Handle); }
                }

                /// <summary>Gets the area of the image, in pixels.</summary>
                property k64s Area
                {
                    k64s get() { return (k64s)kImage_Area(Handle); }
                }

                /// <summary>Gets the size of an image row, including alignment padding bytes, in bytes.</summary>
                property k64s Stride
                {
                    k64s get() { return (k64s)kImage_Stride(Handle); }
                }

            };
        }
    }
}

#endif
