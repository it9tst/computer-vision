/** 
 * @file    kImage.h
 * @brief   Declares the kImage class. 
 *
 * @internal
 * Copyright (C) 2003-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_IMAGE_H
#define K_API_IMAGE_H

#include <kApi/kApiDef.h>
#include <kApi/Data/kImage.x.h>

/**
 * @class       kImage
 * @extends     kObject
 * @implements  kCollection
 * @implements  kArrayProvider
 * @ingroup     kApi-Data
 * @brief       Represents a 2D collection of pixels.
 * 
 * The kImage class is similar to the kArray2 class, with some important differences:
 * - By default, image memory is allocated such that rows are aligned to 8-byte boundaries. 
 * - Pixels are limited to value types. 
 * - Image indices/dimensions are given in (column, row) rather than (row, column) order.  
 * - kImage contains additional image-specific attributes (e.g. color filter array type). 
 * 
 * In particular, the row-alignment behaviour means that image memory should typically be accessed
 * by getting a pointer to the beginning of a row, and then iterating over pixels. E.g. 
 * 
 * @code {.c}
 * kSize height = kImage_Height(image); 
 * kSize width = kImage_Width(image); 
 * kSize i, j; 
 * k32u sum = 0; 
 * 
 * kAssert(kImage_PixelType(image, kTypeOf(k8u))); 
 * 
 * for (i = 0; i < height; ++i)
 * {
 *     k8u* row = kImage_RowAt(image, i); 
 * 
 *     for (j = 0; j < width; ++j)
 *     {
 *         sum += row[j]; 
 *     }
 * }
 *
 * @endcode
 *
 * kImage supports the kObject_Clone and kObject_Size methods.
 *
 * kImage supports the kdat5 and kdat6 serialization protocols.
 */
//typedef kObject kImage; --forward-declared in kApiDef.x.h 

/**
 * Constructs a kImage object.
 *
 * @public                  @memberof kImage
 * @param   image           Receives the constructed image object. 
 * @param   pixelType       Pixel type (must be a value type). 
 * @param   width           Image width. 
 * @param   height          Image height. 
 * @param   allocator       Memory allocator. 
 * @return                  Operation status.
 */
kFx(kStatus) kImage_Construct(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator);

#if defined(K_CPP)
/**
 * Constructs a kImage object using a separate allocator for image array memory.
 *
 * @public                  @memberof kImage
 * @param   image           Receives the constructed image object.
 * @param   pixelType       Pixel type (must be a value type).
 * @param   width           Image width.
 * @param   height          Image height.
 * @param   allocator       Primary memory allocator (or kNULL for default).
 * @param   valueAllocator  Data array allocator (or kNULL for default).
 * @param   valueAlignment  Memory alignment for image data.
 * @return                  Operation status.
 */
kFx(kStatus) kImage_ConstructEx(kImage* image, kType pixelType, kSize width, kSize height, kAlloc allocator, kAlloc valueAllocator, kMemoryAlignment valueAlignment = kALIGN_ANY);
#endif

/** 
 * Loads an image from file.
 * 
 * The file extension is used to determine to image format. The BMP (.bmp) and PNG (.png) formats are supported.
 *
 * @public              @memberof kImage
 * @param   image       Receives the constructed image object. 
 * @param   fileName    File path. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Import(kImage* image, const kChar* fileName, kAlloc allocator); 

/** 
 * Saves an image to file.
 * 
 * The file extension is used to determine to image format. The BMP (.bmp) and PNG (.png) formats are supported.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   fileName    File path. 
 * @return              Operation status.
 */
kFx(kStatus) kImage_Export(kImage image, const kChar* fileName); 

/** 
 * Reallocates the internal pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   pixelType   Type of pixel (values types only). 
 * @param   width       Image width, in pixels. 
 * @param   height      Image height, in pixels. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Allocate(kImage image, kType pixelType, kSize width, kSize height); 

/** 
 * Attaches the image to an external pixel buffer. 
 *
 * Attached pixel buffers are not freed when the image is destroyed. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   pixels      Pointer to external pixel buffer. 
 * @param   pixelType   Type of pixel (values types only). 
 * @param   width       Image width, in pixels. 
 * @param   height      Image height, in pixels. 
 * @param   stride      Image stride (row size), in bytes. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Attach(kImage image, void* pixels, kType pixelType, kSize width, kSize height, kSize stride); 

/** 
 * Attaches the image to an external pixel buffer. 
 *
 * Attached pixel buffers are not freed when the image is destroyed. 
 *
 * A debug assertion will be raised if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   TPtr_pixels     Strongly-typed pointer to external pixel buffer. 
 * @param   kType_pixelType Type of pixel (values types only). 
 * @param   kSize_width     Image width, in pixels. 
 * @param   kSize_height    Image height, in pixels. 
 * @param   kSize_stride    Image stride (row size), in bytes. 
 * @return                  Operation status. 
 */
#define kImage_AttachT(kImage_image, TPtr_pixels, kType_pixelType, kSize_width, kSize_height, kSize_stride) \
    xkImage_AttachT(kImage_image, TPtr_pixels, kType_pixelType, kSize_width, kSize_height, kSize_stride, sizeof(*(TPtr_pixels)))

/** 
 * Copies a given source image into this image. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   source      Source image to be copied.
 * @return              Operation status. 
 */
kInlineFx(kStatus) kImage_Assign(kImage image, kImage source)
{
    return xkImage_Assign(image, source, kNULL);
}

/** 
 * Copies a given source image into this image. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   source      Source image to be copied.
 * @param   context     Context for copy operation (allocator specific; not usually required).
 * @return              Operation status. 
 */
#if defined (K_CPP)
kInlineFx(kStatus) kImage_Assign(kImage image, kImage source, kObject context)
{
    return xkImage_Assign(image, source, context);
}
#endif

/** 
 * Sets all pixel bits to zero.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Zero(kImage image); 

/** 
 * Sets the optional pixel format descriptor associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   format      Pixel format.  
 * @return              Operation status. 
 */
kInlineFx(kStatus) kImage_SetPixelFormat(kImage image, kPixelFormat format)
{
    kObj(kImage, image);

    obj->format = format; 

    return kOK; 
}

/** 
 * Gets the optional pixel format descriptor associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel format. 
 */
kInlineFx(kPixelFormat) kImage_PixelFormat(kImage image)
{
    kObj(kImage, image);

    return obj->format;
}

/** 
 * Sets the color filter array type associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   cfa         Color filter array type.  
 * @return              Operation status. 
 */
kInlineFx(kStatus) kImage_SetCfa(kImage image, kCfa cfa)
{
    kObj(kImage, image);

    obj->cfa = cfa; 

    return kOK; 
}

/** 
 * Gets the color filter array type associated with this image.
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Color filter array type.  
 */
kInlineFx(kCfa) kImage_Cfa(kImage image)
{
    kObj(kImage, image);

    return obj->cfa;
}

/** 
 * Sets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to manipulate
 * many pixels.  In performance-critical code, use kImage_RowAt to directly access the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @param   pixel       Pointer to pixel value that will be copied into the image.
 * @return              Operation status. 
 */
kFx(kStatus) kImage_SetPixel(kImage image, kSize x, kSize y, const void* pixel); 

/** 
 * Sets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to manipulate
 * many pixels. In performance-critical code, use kImage_RowAtT to directly access the pixel buffer. 
 *
 * A debug assertion will be raised if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   kSize_x         Column index. 
 * @param   kSize_y         Row index.  
 * @param   TPtr_pixel      Strongly-typed pointer to pixel value that will be copied into the image.
 * @return                  Operation status. 
 */
#define kImage_SetPixelT(kImage_image, kSize_x, kSize_y, TPtr_pixel) \
    xkImage_SetPixelT(kImage_image, kSize_x, kSize_y, TPtr_pixel, sizeof(*(TPtr_pixel)))

/** 
 * Gets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to access
 * many pixels.  In performance-critical code, use kImage_RowAt to directly access the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @param   pixel       Destination for pixel copied from the image. 
 * @return              Operation status. 
 */
kFx(kStatus) kImage_Pixel(kImage image, kSize x, kSize y, void* pixel); 

/** 
 * Gets the value of a pixel. 
 * 
 * This method is convenient in some contexts, but is not an efficient method to access
 * many pixels.  In performance-critical code, use kImage_RowAtT to directly access the pixel buffer. 
 *
 * A debug assertion will be raised if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   kSize_x         Column index. 
 * @param   kSize_y         Row index.  
 * @param   TPtr_pixel      Strongly-typed destination pointer for pixel copied from the image. 
 * @return                  Operation status. 
 */
#define kImage_PixelT(kImage_image, kSize_x, kSize_y, TPtr_pixel) \
    xkImage_PixelT(kImage_image, kSize_x, kSize_y, TPtr_pixel, sizeof(*(TPtr_pixel)))

/** 
* Sets the value of a pixel. 
* 
* A debug assertion will be raised if the row or column index arguments are greater than or equal to 
* the current image dimensions, or if the size of the specified item type is not equal to the 
* size of the image pixel type.
* 
* @relates                  kImage 
* @param   kImage_image     Image object. 
* @param   kSize_x          Column index. 
* @param   kSize_y          Row index.  
* @param   T_value          Pixel value.  
* @param   T                Pixel type identifier (e.g., kRgb).
* @return                   pixel value. 
*/
#define kImage_SetAsT(kImage_image, kSize_x, kSize_y, T_value, T) \
    (kPointer_WriteAs(xkImage_AsT(kImage_image, kSize_x, kSize_y, sizeof(T)), T_value, T), (void)0)

/** 
* Gets the value of a pixel. 
* 
* A debug assertion will be raised if the row or column index arguments are greater than or equal to 
* the current image dimensions, or if the size of the specified item type is not equal to the 
* size of the image pixel type.
* 
* @relates                  kImage 
* @param   kImage_image     Image object. 
* @param   kSize_x          Column index. 
* @param   kSize_y          Row index.  
* @param   T                Pixel type identifier (e.g., kRgb).
* @return                   pixel value. 
*/
#define kImage_AsT(kImage_image, kSize_x, kSize_y, T) \
    kPointer_ReadAs(xkImage_AsT(kImage_image, kSize_x, kSize_y, sizeof(T)), T)

/** 
 * Returns a pointer to the first row in the pixel buffer.  
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pointer to pixel buffer. 
 */
kInlineFx(void*) kImage_Data(kImage image)
{
    kObj(kImage, image);

    return obj->pixels;
}

/** 
 * Returns a strongly-typed pointer to the first row in the pixel buffer.  
 *
 * A debug assertion will be raised if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   T               Pixel type identifier (e.g., k8u).
 * @return                  Strongly-typed pointer to pixel buffer. 
 */
#define kImage_DataT(kImage_image, T) \
    kCast(T*, xkImage_DataT(kImage_image, sizeof(T)))

/** 
 * Calculates an address relative to the start of the pixel buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index arguments. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @return              Pointer to pixel. 
 */
kInlineFx(void*) kImage_DataAt(kImage image, kSSize x, kSSize y)
{
    kObj(kImage, image);
    kSSize byteOffset = y*(kSSize)obj->stride + x*(kSSize)obj->pixelSize; 

    return kPointer_ByteOffset(obj->pixels, byteOffset);
}

/** 
 * Calculates an address relative to the start of the pixel buffer.
 *
 * This method is similar to the At method, but does not bounds-check the index arguments. 
 *
 * A debug assertion will be raised if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   kSSize_x        Column index. 
 * @param   kSSize_y        Row index.  
 * @param   T               Pixel type identifier (e.g., k8u).
 * @return                  Calculated pointer.
 */
#define kImage_DataAtT(kImage_image, kSSize_x, kSSize_y, T) \
    kCast(T*, xkImage_DataAtT(kImage_image, kSSize_x, kSSize_y, sizeof(T)))

/** 
 * Reports the size, in bytes, of the pixel buffer. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Size of pixel buffer (bytes). 
 */
kInlineFx(kSize) kImage_DataSize(kImage image)
{
    kObj(kImage, image);

    return obj->height * obj->stride;
}

/** 
 * Returns a pointer to the specified pixel in the pixel buffer. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current image dimensions. 
 * 
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   x           Column index. 
 * @param   y           Row index.  
 * @return              Pointer to pixel. 
 */
kInlineFx(void*) kImage_At(kImage image, kSize x, kSize y)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(x < kImage_Width(image));
        kAssert(y < kImage_Height(image));
    }
#   endif

    return kImage_DataAt(image, (kSSize)x, (kSSize)y);
}

/** 
 * Returns a strongly-typed pointer to the specified pixel in the pixel buffer. 
 *
 * A debug assertion will be raised if the index arguments are greater than or equal to 
 * the current image dimensions, or if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   kSize_x         Column index. 
 * @param   kSize_y         Row index.  
 * @param   T               Pixel type identifier (e.g., k8u).
 * @return                  Strongly-typed pointer to pixel. 
 */
#define kImage_AtT(kImage_image, kSize_x, kSize_y, T) \
    kCast(T*, xkImage_AtT(kImage_image, kSize_x, kSize_y, sizeof(T)))

/** 
 * Returns a pointer to the specified row in the pixel buffer. 
 *
 * A debug assertion will be raised if the row index argument is greater than or equal to 
 * the current image height. 
 * 
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @param   y           Row index.  
 * @return              Pointer to first pixel in row. 
 */
kInlineFx(void*) kImage_RowAt(kImage image, kSize y)
{    
#   if !defined(K_FSS_912_DISABLE_BOUNDS_CHECK)
    {
        kAssert(y < kImage_Height(image));
    }
#   endif

    return kImage_DataAt(image, (kSSize)0, (kSSize)y);
}

/** 
 * Returns a strongly-typed pointer to the specified row in the pixel buffer. 
 *
 * A debug assertion will be raised if the row index argument is greater than or equal to 
 * the current image height, or if the size of the specified pixel type is not equal to the 
 * size of the image pixel type.
 * 
 * @relates                 kImage
 * @param   kImage_image    Image object. 
 * @param   kSize_y         Row index.  
 * @param   T               Pixel type identifier (e.g., k8u).
 * @return                  Strongly-typed pointer to first pixel in row. 
 */
#define kImage_RowAtT(kImage_image, kSize_y, T) \
    kCast(T*, xkImage_RowAtT(kImage_image, kSize_y, sizeof(T)))

/** 
 * Returns the pixel type. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel type. 
 */
kInlineFx(kType) kImage_PixelType(kImage image)
{
    kObj(kImage, image); 

    return obj->pixelType;
}

/** 
 * Returns the pixel size. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Pixel size, in bytes. 
 */
kInlineFx(kSize) kImage_PixelSize(kImage image)
{
    kObj(kImage, image); 

    return obj->pixelSize;
}

/** 
 * Returns the width of the image, in pixels. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image width (pixels). 
 */
kInlineFx(kSize) kImage_Width(kImage image)
{
    kObj(kImage, image); 

    return obj->width;
}

/** 
 * Returns the height of the image, in pixels. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image height (pixels). 
 */
kInlineFx(kSize) kImage_Height(kImage image)
{
    kObj(kImage, image); 

    return obj->height;
}

/** 
 * Returns the area of the image, in pixels. 
 * 
 * Image area is the product of image width and image height. 
 *
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image area (pixels). 
 */
kInlineFx(kSize) kImage_Area(kImage image)
{
    kObj(kImage, image); 

    return obj->width * obj->height; 
}

/** 
 * Returns the size of an image row, including alignment padding bytes, in bytes. 
 * 
 * @public              @memberof kImage
 * @param   image       Image object. 
 * @return              Image stride (bytes). 
 */
kInlineFx(kSize) kImage_Stride(kImage image)
{
    kObj(kImage, image); 

    return obj->stride;
}

/** 
 * Reports the allocator used for the internal pixel array.
 *
 * @public          @memberof kArray1
 * @param   image   Image object. 
 * @return          Pixel array allocator.
 */
kInlineFx(kAlloc) kImage_DataAlloc(kImage image)
{
    kObj(kImage, image);

    return obj->dataAlloc; 
}

#endif
