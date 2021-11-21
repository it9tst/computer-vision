/** 
 * @file    kImageUtils.cpp
 *
 * @internal
 * Copyright (C) 2020-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#include <kApi/Utils/kImageUtils.x.h>

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kImage.h>
#include <kApi/Io/kFile.h>
#include <kApi/Io/kPath.h>
#include <kApi/Io/kSerializer.h>

#include <kApi.extern/lodepng/lodepng.h>

// Png read functions

kType xkImageUtils_LodePngStateToType(const LodePNGState& state)
{
    switch (state.info_png.color.colortype)
    {
    case LCT_GREY:
        if (state.info_png.color.bitdepth == 16)
        {
            return kTypeOf(k16u);
        }
        if (state.info_png.color.bitdepth == 8)
        {
            return kTypeOf(k8u);
        }
        break;

    case LCT_RGB:
        if (state.info_png.color.bitdepth == 8)
            return kTypeOf(kRgb);
        break;

    case LCT_RGBA:
        if (state.info_png.color.bitdepth == 8)
            return kTypeOf(kArgb);
        break;

    case LCT_PALETTE:
        return kTypeOf(kArgb);

    default:
        break;
    }

    return kNULL;
}

static kStatus xkImageUtils_FillData8uToImage(const kImage image, const unsigned char* pixelReader)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        const auto rowWriter = kImage_RowAt(image, y);

        kMemCopy(rowWriter, pixelReader, width);
        pixelReader += width;
    }

    return kOK;
}

static kStatus xkImageUtils_FillData16uToImage(const kImage image, const unsigned char* pixelReader)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowWriter = kImage_RowAtT(image, y, k16u);
        const auto rowWriterEnd = rowWriter + width;

        while (rowWriter != rowWriterEnd)
        {
            *rowWriter++ = (((k16u)pixelReader[0]) << 8) | (k16u)pixelReader[1];
            pixelReader += 2;
        }
    }

    return kOK;
}

static kStatus xkImageUtils_FillDataArgbToImage(const kImage image, const unsigned char* pixelReader)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowWriter = kImage_RowAtT(image, y, kArgb);
        const auto rowWriterEnd = rowWriter + width;

        for (; rowWriter != rowWriterEnd; rowWriter++)
        {
            rowWriter->r = *pixelReader++;
            rowWriter->g = *pixelReader++;
            rowWriter->b = *pixelReader++;
            rowWriter->a = *pixelReader++;
        }
    }

    return kOK;
}

static kStatus xkImageUtils_FillDataRgbToImage(const kImage image, const unsigned char* pixelReader)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowWriter = kImage_RowAtT(image, y, kRgb);
        const auto rowWriterEnd = rowWriter + width;

        for (; rowWriter != rowWriterEnd; rowWriter++)
        {
            rowWriter->r = *pixelReader++;
            rowWriter->g = *pixelReader++;
            rowWriter->b = *pixelReader++;
            pixelReader++;
        }
    }

    return kOK;
}

static kStatus xkImageUtils_FillDataToImage(LodePNGColorType colortype, kType type, const kImage image, const unsigned char* pixelReader)
{
    if (colortype == LCT_GREY)
    {
        if (type == kTypeOf(k8u))
        {
            return xkImageUtils_FillData8uToImage(image, pixelReader);
        }

        if (type == kTypeOf(k16u))
        {
            return xkImageUtils_FillData16uToImage(image, pixelReader);
        }
    }

    if (colortype == LCT_PALETTE || colortype == LCT_RGBA)
    {
        return xkImageUtils_FillDataArgbToImage(image, pixelReader);
    }

    if (colortype == LCT_RGB)
    {
        return xkImageUtils_FillDataRgbToImage(image, pixelReader);
    }

    return kERROR;
}

kStatus xkImageUtils_LoadAndDecodePng(const kChar* fileName, LodePNGState* state, unsigned int* width, unsigned int* height, unsigned char** dataPng, kAlloc allocator)
{
    unsigned char* png = kNULL;
    kSize pngSize = 0;
    *dataPng = kNULL;

    kCheck(kFile_Load(fileName, &png, &pngSize, allocator));

    lodepng_state_init(state);
    auto lodepngError = lodepng_inspect(width, height, state, png, pngSize);

    if (lodepngError == 0)
    {
        if (state->info_png.color.colortype == LCT_GREY)
        {
            lodepngError = lodepng_decode_memory(dataPng, width, height, png, pngSize, state->info_png.color.colortype, state->info_png.color.bitdepth);
        }
        else
        {
            lodepngError = lodepng_decode32(dataPng, width, height, png, pngSize);
        }
    }

    if (lodepngError)
    {
        lodepng_free(*dataPng);
        *dataPng = kNULL;
        lodepng_state_cleanup(state);
        kCheck(kAlloc_Free(allocator, png));

        return kERROR;
    }

    kCheck(kAlloc_Free(allocator, png));
    return kOK;
}

kFx(kStatus) xkImageUtils_LoadPng(kImage* image, const kChar* fileName, kAlloc allocator)
{
    LodePNGState state;
    unsigned char* dataPng = kNULL;
    unsigned int width = 0, height = 0;

    *image = kNULL;
    kCheck(xkImageUtils_LoadAndDecodePng(fileName, &state, &width, &height, &dataPng, allocator));

    kImage out = kNULL;

    kTry
    {
        const auto type = xkImageUtils_LodePngStateToType(state);
        kTestTrue(type != kNULL, kERROR_FORMAT);

        kTest(kImage_Construct(&out, type, width, height, allocator));
        kTest(xkImageUtils_FillDataToImage(state.info_png.color.colortype, type, out, dataPng));
        *image = out;
        out = kNULL;
    }
    kFinally
    {
        lodepng_free(dataPng);
        lodepng_state_cleanup(&state);
        kDestroyRef(&out);
        kEndFinally();
    }

    return kOK;
}

// Png write functions

static kStatus xkImageUtils_TypeToLodePngType(const kType type, LodePNGColorType& lodePNGColorType, kSize& outPixelSize, unsigned int& bitDepth)
{
    if (type == kTypeOf(k8u))
    {
        outPixelSize = 1;
        lodePNGColorType = LCT_GREY;
    }
    else if (type == kTypeOf(k16u))
    {
        outPixelSize = 2;
        lodePNGColorType = LCT_GREY;
        bitDepth = 16;
    }
    else if (type == kTypeOf(kRgb))
    {
        outPixelSize = 3;
        lodePNGColorType = LCT_RGB;
    }
    else if (type == kTypeOf(kArgb))
    {
        outPixelSize = 4;
        lodePNGColorType = LCT_RGBA;
    }
    else
    {
        return kERROR_PARAMETER;
    }

    return kOK;
}

static void xkImageUtils_FillData8uFromImage(const kImage image, k8u* pixelWriter)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        const auto rowReader = kImage_RowAt(image, y);

        kMemCopy(pixelWriter, rowReader, width);
        pixelWriter += width;
    }
}

static void xkImageUtils_FillData16uFromImage(const kImage image, k8u* pixelWriter)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowReader = kImage_RowAtT(image, y, k16u);
        const auto rowReaderEnd = rowReader + width;

        for (; rowReader != rowReaderEnd; rowReader++)
        {
            *pixelWriter++ = (k8u)(*rowReader >> 8);
            *pixelWriter++ = (k8u)(*rowReader & 0xff);
        }
    }
}

static void xkImageUtils_FillDataRgbFromImage(const kImage image, k8u* pixelWriter)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowReader = kImage_RowAtT(image, y, kRgb);
        const auto rowReaderEnd = rowReader + width;

        for (; rowReader != rowReaderEnd; rowReader++)
        {
            *pixelWriter++ = rowReader->r;
            *pixelWriter++ = rowReader->g;
            *pixelWriter++ = rowReader->b;
        }
    }
}

static void xkImageUtils_FillDataArgbFromImage(const kImage image, k8u* pixelWriter)
{
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);

    for (kSize y = 0; y < height; y++)
    {
        auto rowReader = kImage_RowAtT(image, y, kArgb);
        const auto rowReaderEnd = rowReader + width;

        for (; rowReader != rowReaderEnd; rowReader++)
        {
            *pixelWriter++ = rowReader->r;
            *pixelWriter++ = rowReader->g;
            *pixelWriter++ = rowReader->b;
            *pixelWriter++ = rowReader->a;
        }
    }
}

static kStatus xkImageUtils_FillDataFromImage(kImage image, kArray1 imageArray)
{
    const auto data = kArray1_DataT(imageArray, k8u);
    const auto type = kImage_PixelType(image);

    if (type == kTypeOf(k8u))
    {
        xkImageUtils_FillData8uFromImage(image, data);
    }
    else if (type == kTypeOf(k16u))
    {
        xkImageUtils_FillData16uFromImage(image, data);
    }
    else if (type == kTypeOf(kRgb))
    {
        xkImageUtils_FillDataRgbFromImage(image, data);
    }
    else if (type == kTypeOf(kArgb))
    {
        xkImageUtils_FillDataArgbFromImage(image, data);
    }
    else
    {
        return kERROR;
    }

    return kOK;
}

kFx(kStatus) xkImageUtils_SavePng(kImage image, const kChar* fileName, kAlloc allocator)
{
    const auto type = kImage_PixelType(image);
    const auto width = kImage_Width(image);
    const auto height = kImage_Height(image);
    kSize outPixelSize = 1;
    LodePNGColorType lodePNGColorType = LCT_GREY;
    unsigned int bitDepth = 8;

    kCheck(xkImageUtils_TypeToLodePngType(type, lodePNGColorType, outPixelSize, bitDepth));

    kArray1 imageArray = kNULL;
    kCheck(kArray1_Construct(&imageArray, kTypeOf(k8u), width * height * outPixelSize, allocator));

    if (xkImageUtils_FillDataFromImage(image, imageArray) != kOK)
    {
        kCheck(kDestroyRef(&imageArray));
        return kERROR;
    }

    unsigned char* png = kNULL;
    size_t pngSize = 0;
    kStatus fileStatus = kOK;
    LodePNGState state;

    lodepng_state_init(&state);
    state.encoder.auto_convert = 0; 
    state.info_raw.colortype = lodePNGColorType;
    state.info_raw.bitdepth = bitDepth;
    state.info_png.color.colortype = lodePNGColorType;
    state.info_png.color.bitdepth = bitDepth;

    const auto status = lodepng_encode(&png, &pngSize, kArray1_DataT(imageArray, const unsigned char),
                                       (unsigned int)(width), (unsigned int)(height), &state);

    if (!status)
    {        
        fileStatus = kFile_Save(fileName, png, pngSize);
    }

    lodepng_state_cleanup(&state);
    lodepng_free(png);
    kCheck(kDestroyRef(&imageArray));

    return status ? kERROR : fileStatus;
}

// Bitmap functions

kFx(kStatus) xkImageUtils_SaveBmp(kImage image, const kChar* fileName, kAlloc allocator)
{
    kFile file = kNULL;
    kSerializer serializer = kNULL;
    kType type = kImage_PixelType(image);
    k32u width = (k32u)kImage_Width(image);
    k32u height = (k32u)kImage_Height(image);
    const k32u paletteEntryCount = (type == kTypeOf(k8u)) ? 256u : 0u;
    const k32u offBitCount = 54 + 4 * paletteEntryCount;
    static const k32u infoHeaderSize = 40;
    static const k32u bmType = 19778;
    k32u outPixelSize, outWidth, outSize, backPorch;
    k32u i;
    k32s x, y;

    if (type == kTypeOf(k8u))      outPixelSize = 1;
    else if (type == kTypeOf(kRgb))     outPixelSize = 3;
    else if (type == kTypeOf(kArgb))    outPixelSize = 4;
    else                                return kERROR_PARAMETER;

    kTry
    {
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_WRITE, kNULL));
        kTest(kSerializer_Construct(&serializer, file, kNULL, kNULL));

        backPorch = (4 - ((width * outPixelSize) % 4)) % 4;
        outWidth = width + backPorch;
        outSize = (width * outPixelSize + backPorch) * height;

        kTest(kSerializer_Write16u(serializer, (k16u)bmType));
        kTest(kSerializer_Write32u(serializer, offBitCount + outSize));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, offBitCount));

        kTest(kSerializer_Write32u(serializer, infoHeaderSize));
        kTest(kSerializer_Write32u(serializer, width));
        kTest(kSerializer_Write32u(serializer, height));
        kTest(kSerializer_Write16u(serializer, 1));
        kTest(kSerializer_Write16u(serializer, (k16u)(outPixelSize * 8)));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, outSize));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));
        kTest(kSerializer_Write32u(serializer, 0));

        /* This is a grayscale palette */
        for (i = 0; i < paletteEntryCount; i++)
        {
            kTest(kSerializer_Write32u(serializer, i * 0x00010101));
        }

        for (y = (k32s)height - 1; y >= 0; y--)
        {
            for (x = 0; x < (k32s)width; x++)
            {
                kTest(kSerializer_WriteByteArray(serializer, kImage_At(image, (kSize)x, (kSize)y), outPixelSize));
            }
            for (i = 0; i < outWidth - width; i++)
            {
                kTest(kSerializer_Write8u(serializer, 0));
            }
        }

        kTest(kSerializer_Flush(serializer));
    }
    kFinally
    {
        kObject_Destroy(serializer);
        kObject_Destroy(file);
        kEndFinally();
    }

    return kOK;
}

kFx(kStatus) xkImageUtils_LoadBmp(kImage* image, const kChar* fileName, kAlloc allocator)
{
    kFile file = kNULL;
    kSerializer serializer = kNULL;
    kImage out = kNULL;
    kRgb* palette = kNULL;
    k16u bmType, bppIn, pixelSizeIn, backPorch, planes;
    k32u fileEnd, reserved, offBitCount, infoHeaderSize, width;
    k32u height, compression, paletteSize;
    k32s x, y;
    kType type = kNULL;
    kStatus exception;

    kTry
    {
        kTest(kFile_Construct(&file, fileName, kFILE_MODE_READ, kNULL));
        kTest(kFile_SetReadBuffer(file, 16384));

        kTest(kSerializer_Construct(&serializer, file, kNULL, kNULL));

        kTest(kSerializer_Read16u(serializer, &bmType));
        kTest(kSerializer_Read32u(serializer, &fileEnd));
        kTest(kSerializer_Read32u(serializer, &reserved));
        kTest(kSerializer_Read32u(serializer, &offBitCount));

        kTest(kSerializer_Read32u(serializer, &infoHeaderSize));

        kTest(kFile_Length(file) == fileEnd); /* corrupted file? */

        if (infoHeaderSize == 40)
        {
            kTest(kSerializer_Read32u(serializer, &width));
            kTest(kSerializer_Read32u(serializer, &height));
            kTest(kSerializer_Read16u(serializer, &planes));
            kTest(kSerializer_Read16u(serializer, &bppIn));
            kTest(kSerializer_Read32u(serializer, &compression));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
            kTest(kSerializer_Read32u(serializer, &reserved));
        }
        else
        {
            /* TODO: add support for other header sizes */
            kThrow(kERROR);
        }

        kTest(planes == 1);
        kTest(compression == 0);

        kTest(width != 0);
        kTest(height != 0);

        paletteSize = offBitCount - infoHeaderSize - 14;

        if (paletteSize > 0)
        {
            kTest(kMemAlloc(paletteSize, &palette));
            kTest(kSerializer_ReadByteArray(serializer, (kByte*)palette, paletteSize));
        }

        switch (bppIn)
        {
        case 8:
            type = kTypeOf(k8u);
            break;
        case 24:
            type = kTypeOf(kRgb);
            break;
        case 32:
            type = kTypeOf(kArgb);
            break;
        default:
            kThrow(kERROR);
        }

        kTest(bppIn % 8 == 0); /* does not support packed pixels */
        pixelSizeIn = (k16u)(bppIn / 8);

        backPorch = (4 - ((width * pixelSizeIn) % 4)) % 4;

        kTest(kImage_Construct(&out, type, width, height, allocator));
        kTest(kImage_Zero(out));

        for (y = (k32s)height - 1; y >= 0; y--)
        {
            for (x = 0; x < (k32s)width; x++)
            {
                if (paletteSize && (type == kTypeOf(k8u)))
                {
                    k32u pixel = 0;
                    kRgb* entry;

                    kTest(kSerializer_ReadByteArray(serializer, (kByte*)&pixel, pixelSizeIn));

                    if (pixel > paletteSize / sizeof(kRgb))
                    {
                        kThrow(kERROR);
                    }

                    entry = &palette[pixel];

                    kImage_SetAsT(out, (kSize)x, (kSize)y, (entry->r + entry->g + entry->b) / 3, k8u);
                }
                else
                {
                    kTest(kSerializer_ReadByteArray(serializer, kImage_At(out, (kSize)x, (kSize)y), pixelSizeIn));
                }
            }

            kTest(kSerializer_AdvanceRead(serializer, backPorch));
        }
        *image = out;
    }
    kCatchEx(&exception)
    {
        kDestroyRef(&out);
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kMemFreeRef(&palette);
        kDestroyRef(&serializer);
        kDestroyRef(&file);
        kEndFinallyEx();
    }

    return kOK;
}

kFx(kStatus) xkImageUtils_Load(kImage* image, const kChar* fileName, kAlloc allocator)
{
    const auto alloc = kAlloc_Fallback(allocator);
    kText64 extension;

    kCheck(kPath_Extension(fileName, extension, kCountOf(extension)));

    if (kStrCompareLower(extension, "png") == 0)
    {
        return xkImageUtils_LoadPng(image, fileName, alloc);
    }

    return xkImageUtils_LoadBmp(image, fileName, alloc);
}

kFx(kStatus) xkImageUtils_Save(kImage image, const kChar* fileName, kAlloc allocator)
{
    const auto alloc = kAlloc_Fallback(allocator);
    kText64 extension;

    kCheck(kPath_Extension(fileName, extension, kCountOf(extension)));

    if (kStrCompareLower(extension, "png") == 0)
    {
        return xkImageUtils_SavePng(image, fileName, alloc);
    }

    return xkImageUtils_SaveBmp(image, fileName, alloc);
}
