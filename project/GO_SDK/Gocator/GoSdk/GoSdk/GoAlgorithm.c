/**
* @file    GoAlgorithm.c
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/GoAlgorithm.h>

#include <kApi/Data/kImage.h>

#define COPY_BAYER_ALGORITHM_FROM_KVISION 1
#ifdef COPY_BAYER_ALGORITHM_FROM_KVISION
kStatus GoAlgorithm_BayerDemosaicReduceBggr(kImage input, kImage output)
{
    k32u outWidth = (k32u)kImage_Width(output);
    k32u outHeight = (k32u)kImage_Height(output);
    k64u* reader[2];
    k64u v0, v1;
    k8u* v0r = (void*)&v0;
    k8u* v1r = (void*)&v1;
    k32u* writer;
    k32u* writerEnd;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kArgb)) ||
        (kImage_Width(input) != 2 * outWidth) ||
        (kImage_Height(input) != 2 * outHeight) ||
        (kImage_Width(input) % 8) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < outHeight; ++i)
    {
        reader[0] = (k64u*)kImage_RowAt(input, 2 * i);
        reader[1] = (k64u*)kImage_RowAt(input, 2 * i + 1);
        writer = (k32u*)kImage_RowAt(output, i);
        writerEnd = writer + outWidth;

        while (writer != writerEnd)
        {
            v0 = *reader[0]++;
            v1 = *reader[1]++;

            writer[0] = (v1r[1] << 16) | (((v0r[1] + v1r[0]) >> 1) << 8) | v0r[0];
            writer[1] = (v1r[3] << 16) | (((v0r[3] + v1r[2]) >> 1) << 8) | v0r[2];
            writer[2] = (v1r[5] << 16) | (((v0r[5] + v1r[4]) >> 1) << 8) | v0r[4];
            writer[3] = (v1r[7] << 16) | (((v0r[7] + v1r[6]) >> 1) << 8) | v0r[6];

            writer += 4;
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicReduceGbrg(kImage input, kImage output)
{
    k32u outWidth = (k32u)kImage_Width(output);
    k32u outHeight = (k32u)kImage_Height(output);
    k64u* reader[2];
    k64u v0, v1;
    k8u* v0r = (void*)&v0;
    k8u* v1r = (void*)&v1;
    k32u* writer;
    k32u* writerEnd;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kArgb)) ||
        (kImage_Width(input) != 2 * outWidth) ||
        (kImage_Height(input) != 2 * outHeight) ||
        (kImage_Width(input) % 8) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < outHeight; ++i)
    {
        reader[0] = (k64u*)kImage_RowAt(input, 2 * i);
        reader[1] = (k64u*)kImage_RowAt(input, 2 * i + 1);
        writer = (k32u*)kImage_RowAt(output, i);
        writerEnd = writer + outWidth;

        while (writer != writerEnd)
        {
            v0 = *reader[0]++;
            v1 = *reader[1]++;

            writer[0] = (v1r[0] << 16) | (((v0r[0] + v1r[1]) >> 1) << 8) | v0r[1];
            writer[1] = (v1r[2] << 16) | (((v0r[2] + v1r[3]) >> 1) << 8) | v0r[3];
            writer[2] = (v1r[4] << 16) | (((v0r[4] + v1r[5]) >> 1) << 8) | v0r[5];
            writer[3] = (v1r[6] << 16) | (((v0r[6] + v1r[7]) >> 1) << 8) | v0r[7];

            writer += 4;
        }
    }
    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicReduceRggb(kImage input, kImage output)
{
    k32u outWidth = (k32u)kImage_Width(output);
    k32u outHeight = (k32u)kImage_Height(output);
    k64u* reader[2];
    k64u v0, v1;
    k8u* v0r = (void*)&v0;
    k8u* v1r = (void*)&v1;
    k32u* writer;
    k32u* writerEnd;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kArgb)) ||
        (kImage_Width(input) != 2 * outWidth) ||
        (kImage_Height(input) != 2 * outHeight) ||
        (kImage_Width(input) % 8) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < outHeight; ++i)
    {
        reader[0] = (k64u*)kImage_RowAt(input, 2 * i);
        reader[1] = (k64u*)kImage_RowAt(input, 2 * i + 1);
        writer = (k32u*)kImage_RowAt(output, i);
        writerEnd = writer + outWidth;

        while (writer != writerEnd)
        {
            v0 = *reader[0]++;
            v1 = *reader[1]++;

            writer[0] = (v0r[0] << 16) | (((v0r[1] + v1r[0]) >> 1) << 8) | v1r[1];
            writer[1] = (v0r[2] << 16) | (((v0r[3] + v1r[2]) >> 1) << 8) | v1r[3];
            writer[2] = (v0r[4] << 16) | (((v0r[5] + v1r[4]) >> 1) << 8) | v1r[5];
            writer[3] = (v0r[6] << 16) | (((v0r[7] + v1r[6]) >> 1) << 8) | v1r[7];

            writer += 4;
        }
    }
    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicReduceGrbg(kImage input, kImage output)
{
    k32u outWidth = (k32u)kImage_Width(output);
    k32u outHeight = (k32u)kImage_Height(output);
    k64u* reader[2];
    k64u v0, v1;
    k8u* v0r = (void*)&v0;
    k8u* v1r = (void*)&v1;
    k32u* writer;
    k32u* writerEnd;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kArgb)) ||
        (kImage_Width(input) != 2 * outWidth) ||
        (kImage_Height(input) != 2 * outHeight) ||
        (kImage_Width(input) % 8) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < outHeight; ++i)
    {
        reader[0] = (k64u*)kImage_RowAt(input, 2 * i);
        reader[1] = (k64u*)kImage_RowAt(input, 2 * i + 1);
        writer = (k32u*)kImage_RowAt(output, i);
        writerEnd = writer + outWidth;

        while (writer != writerEnd)
        {
            v0 = *reader[0]++;
            v1 = *reader[1]++;

            writer[0] = (v0r[1] << 16) | (((v0r[0] + v1r[1]) >> 1) << 8) | v1r[0];
            writer[1] = (v0r[3] << 16) | (((v0r[2] + v1r[3]) >> 1) << 8) | v1r[2];
            writer[2] = (v0r[5] << 16) | (((v0r[4] + v1r[5]) >> 1) << 8) | v1r[4];
            writer[3] = (v0r[7] << 16) | (((v0r[6] + v1r[7]) >> 1) << 8) | v1r[6];

            writer += 4;
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicBilinearBggr(kImage input, kImage output)
{
    k8u* srcp;
    k8u* r0p, *r1p, *b0p, *b1p;
    k32u *dstp;
    k32u r, g, b;
    k32s i, j;
    k32s width = (k32s)kImage_Width(input);
    k32s height = (k32s)kImage_Height(input);

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        ((k32s)kImage_Width(output) != width) ||
        ((k32s)kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    srcp = kImage_AtT(input, 0, 0, k8u);
    dstp = kImage_AtT(output, 0, 0, k32u);

    for (i = 0; i<height; i++)
    {
        if ((i & 1) == 0)
        { /* Blue row */
            b0p = srcp + i*width;
            if (i)
            {
                if (i<height - 1)
                {
                    r1p = b0p + width;
                    r0p = b0p - width;
                }
                else
                {
                    r0p = r1p = b0p - width;
                }
            }
            else
            {
                r0p = r1p = b0p + width;
            }

            /* First pixel */
            r = (*(r0p + 1) + *(r1p + 1)) / 2;
            g = (*r0p + *r1p) / 2;
            b = *b0p;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; r1p++; b0p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                { /* Green Pixel */
                    r = (*r0p + *r1p) / 2;
                    g = *b0p;
                    b = (*(b0p - 1) + *(b0p + 1)) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
                else
                { /* Blue pixel */
                    r = (*(r0p - 1) + *(r0p + 1) + *(r1p - 1) + *(r1p + 1) + 1) / 4;
                    g = (*(b0p - 1) + *(b0p + 1) + *r0p + *r1p + 1) / 4;
                    b = *b0p;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            { /* Green Pixel */
                r = (*r0p + *r1p) / 2;
                g = *b0p;
                b = *(b0p - 1);
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            { /* Blue pixel */
                r = (*(r0p - 1) + *(r1p - 1)) / 2;
                g = (*(b0p - 1) + *(b0p - 1) + *r0p + *r1p + 1) / 4;
                b = *b0p;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
        else
        { /* Red row */
            r0p = srcp + i*width;
            b1p = r0p + width;
            if (i)
            {
                if (i<height - 1)
                {
                    b1p = r0p + width;
                    b0p = r0p - width;
                }
                else
                {
                    b0p = b1p = r0p - width;
                }
            }
            else
            {
                b0p = b1p = r0p + width;
            }

            /* First pixel */
            r = *(r0p + 1);
            g = *r0p;
            b = (*b0p + *b1p) / 2;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; b0p++; b1p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                { /* Red pixel */
                    r = *r0p;
                    g = (*(r0p - 1) + *(r0p + 1) + *b0p + *b1p + 1) / 4;
                    b = (*(b0p - 1) + *(b0p + 1) + *(b1p - 1) + *(b1p + 1) + 1) / 4;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
                else
                { /* Green Pixel */
                    r = (*(r0p - 1) + *(r0p + 1)) / 2;
                    g = *r0p;
                    b = (*b0p + *b1p) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            { /* Red pixel */
                r = *r0p;
                g = (*(r0p - 1) + *(r0p - 1) + *b0p + *b1p + 1) / 4;
                b = (*(b0p - 1) + *(b1p - 1)) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            { /* Green Pixel */
                r = *(r0p - 1);
                g = *r0p;
                b = (*b0p + *b1p) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicBilinearGbrg(kImage input, kImage output)
{
    k8u* srcp;
    k8u* r0p, *r1p, *b0p, *b1p;
    k32u *dstp;
    k32u r, g, b;
    k32s i, j;
    k32s width = (k32s)kImage_Width(input);
    k32s height = (k32s)kImage_Height(input);

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        ((k32s)kImage_Width(output) != width) ||
        ((k32s)kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    srcp = kImage_AtT(input, 0, 0, k8u);
    dstp = kImage_AtT(output, 0, 0, k32u);

    for (i = 0; i<height; i++)
    {
        if ((i & 1) == 0)
        { /* Blue row */
            b0p = srcp + i*width;
            if (i)
            {
                if (i<height - 1)
                {
                    r1p = b0p + width;
                    r0p = b0p - width;
                }
                else
                {
                    r0p = r1p = b0p - width;
                }
            }
            else
            {
                r0p = r1p = b0p + width;
            }

            /* First pixel */
            r = (*r0p + *r1p) / 2;
            g = *b0p;
            b = *(b0p + 1);
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; r1p++; b0p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                {
                    /* Blue pixel */
                    r = (*(r0p - 1) + *(r0p + 1) + *(r1p - 1) + *(r1p + 1) + 1) / 4;
                    g = (*(b0p - 1) + *(b0p + 1) + *r0p + *r1p + 1) / 4;
                    b = *b0p;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
                else
                {
                    /* Green Pixel */
                    r = (*r0p + *r1p) / 2;
                    g = *b0p;
                    b = (*(b0p - 1) + *(b0p + 1)) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            {
                /* Blue pixel */
                r = (*(r0p - 1) + *(r1p - 1)) / 2;
                g = (*(b0p - 1) + *(b0p - 1) + *r0p + *r1p + 1) / 4;
                b = *b0p;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            {
                /* Green Pixel */
                r = (*r0p + *r1p) / 2;
                g = *b0p;
                b = *(b0p - 1);
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
        else
        { /* Red row */
            r0p = srcp + i*width;
            b1p = r0p + width;
            if (i)
            {
                if (i<height - 1)
                {
                    b1p = r0p + width;
                    b0p = r0p - width;
                }
                else
                {
                    b0p = b1p = r0p - width;
                }
            }
            else
            {
                b0p = b1p = r0p + width;
            }

            /* First pixel */
            r = *r0p;
            g = (*b0p + *b1p) / 2; // Should we take the adjacent one as well?
            b = (*(b0p + 1) + *(b1p + 1)) / 2;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; b0p++; b1p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                {
                    /* Green Pixel */
                    r = (*(r0p - 1) + *(r0p + 1)) / 2;
                    g = *r0p;
                    b = (*b0p + *b1p) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
                else
                {
                    /* Red pixel */
                    r = *r0p;
                    g = (*(r0p - 1) + *(r0p + 1) + *b0p + *b1p + 1) / 4;
                    b = (*(b0p - 1) + *(b0p + 1) + *(b1p - 1) + *(b1p + 1) + 1) / 4;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            {
                /* Green Pixel */
                r = *(r0p - 1);
                g = *r0p;
                b = (*b0p + *b1p) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            {
                /* Red pixel */
                r = *r0p;
                g = (*(r0p - 1) + *(r0p - 1) + *b0p + *b1p + 1) / 4;
                b = (*(b0p - 1) + *(b1p - 1)) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicBilinearRggb(kImage input, kImage output)
{
    k8u* srcp;
    k8u* r0p, *r1p, *b0p, *b1p;
    k32u *dstp;
    k32u r, g, b;
    k32s i, j;
    k32s width = (k32s)kImage_Width(input);
    k32s height = (k32s)kImage_Height(input);

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        ((k32s)kImage_Width(output) != width) ||
        ((k32s)kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    srcp = kImage_AtT(input, 0, 0, k8u);
    dstp = kImage_AtT(output, 0, 0, k32u);

    for (i = 0; i<height; i++)
    {
        if ((i & 1) == 1)
        { /* Blue row */
            b0p = srcp + i*width;
            if (i)
            {
                if (i<height - 1)
                {
                    r1p = b0p + width;
                    r0p = b0p - width;
                }
                else
                {
                    r0p = r1p = b0p - width;
                }
            }
            else
            {
                r0p = r1p = b0p + width;
            }

            /* First pixel */
            r = (*r0p + *r1p) / 2;
            g = *b0p;
            b = *(b0p + 1);
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; r1p++; b0p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                {
                    /* Blue pixel */
                    r = (*(r0p - 1) + *(r0p + 1) + *(r1p - 1) + *(r1p + 1) + 1) / 4;
                    g = (*(b0p - 1) + *(b0p + 1) + *r0p + *r1p + 1) / 4;
                    b = *b0p;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
                else
                {
                    /* Green Pixel */
                    r = (*r0p + *r1p) / 2;
                    g = *b0p;
                    b = (*(b0p - 1) + *(b0p + 1)) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            {
                /* Blue pixel */
                r = (*(r0p - 1) + *(r1p - 1)) / 2;
                g = (*(b0p - 1) + *(b0p - 1) + *r0p + *r1p + 1) / 4;
                b = *b0p;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            {
                /* Green Pixel */
                r = (*r0p + *r1p) / 2;
                g = *b0p;
                b = *(b0p - 1);
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
        else
        { /* Red row */
            r0p = srcp + i*width;
            b1p = r0p + width;
            if (i)
            {
                if (i<height - 1)
                {
                    b1p = r0p + width;
                    b0p = r0p - width;
                }
                else
                {
                    b0p = b1p = r0p - width;
                }
            }
            else
            {
                b0p = b1p = r0p + width;
            }

            /* First pixel */
            r = *r0p;
            g = (*b0p + *b1p) / 2;
            b = (*(b0p + 1) + *(b1p + 1)) / 2;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; b0p++; b1p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                {
                    /* Green Pixel */
                    r = (*(r0p - 1) + *(r0p + 1)) / 2;
                    g = *r0p;
                    b = (*b0p + *b1p) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
                else
                {
                    /* Red pixel */
                    r = *r0p;
                    g = (*(r0p - 1) + *(r0p + 1) + *b0p + *b1p + 1) / 4;
                    b = (*(b0p - 1) + *(b0p + 1) + *(b1p - 1) + *(b1p + 1) + 1) / 4;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            {
                /* Green Pixel */
                r = *(r0p - 1);
                g = *r0p;
                b = (*b0p + *b1p) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            {
                /* Red pixel */
                r = *r0p;
                g = (*(r0p - 1) + *(r0p - 1) + *b0p + *b1p + 1) / 4;
                b = (*(b0p - 1) + *(b1p - 1)) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicBilinearGrbg(kImage input, kImage output)
{
    k8u* srcp;
    k8u* r0p, *r1p, *b0p, *b1p;
    k32u *dstp;
    k32u r, g, b;
    k32s i, j;
    k32s width = (k32s)kImage_Width(input);
    k32s height = (k32s)kImage_Height(input);

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        ((k32s)kImage_Width(output) != width) ||
        ((k32s)kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2))
    {
        return kERROR_PARAMETER;
    }

    srcp = kImage_AtT(input, 0, 0, k8u);
    dstp = kImage_AtT(output, 0, 0, k32u);

    for (i = 0; i<height; i++)
    {
        if ((i & 1) == 1)
        { /* Blue row */
            b0p = srcp + i*width;
            if (i)
            {
                if (i<height - 1)
                {
                    r1p = b0p + width;
                    r0p = b0p - width;
                }
                else
                {
                    r0p = r1p = b0p - width;
                }
            }
            else
            {
                r0p = r1p = b0p + width;
            }

            /* First pixel */
            r = (*(r0p + 1) + *(r1p + 1)) / 2;
            g = (*r0p + *r1p) / 2;
            b = *b0p;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; r1p++; b0p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                { /* Green Pixel */
                    r = (*r0p + *r1p) / 2;
                    g = *b0p;
                    b = (*(b0p - 1) + *(b0p + 1)) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
                else
                { /* Blue pixel */
                    r = (*(r0p - 1) + *(r0p + 1) + *(r1p - 1) + *(r1p + 1) + 1) / 4;
                    g = (*(b0p - 1) + *(b0p + 1) + *r0p + *r1p + 1) / 4;
                    b = *b0p;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; r1p++; b0p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            { /* Green Pixel */
                r = (*r0p + *r1p) / 2;
                g = *b0p;
                b = *(b0p - 1);
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            { /* Blue pixel */
                r = (*(r0p - 1) + *(r1p - 1)) / 2;
                g = (*(b0p - 1) + *(b0p - 1) + *r0p + *r1p + 1) / 4;
                b = *b0p;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
        else
        { /* Red row */
            r0p = srcp + i*width;
            b1p = r0p + width;
            if (i)
            {
                if (i<height - 1)
                {
                    b1p = r0p + width;
                    b0p = r0p - width;
                }
                else
                {
                    b0p = b1p = r0p - width;
                }
            }
            else
            {
                b0p = b1p = r0p + width;
            }

            /* First pixel */
            r = *(r0p + 1);
            g = *r0p;
            b = (*b0p + *b1p) / 2;
            *dstp++ = b | (g << 8) | (r << 16);
            r0p++; b0p++; b1p++;

            for (j = 1; j<width - 1; j++)
            {
                if (j % 2)
                { /* Red pixel */
                    r = *r0p;
                    g = (*(r0p - 1) + *(r0p + 1) + *b0p + *b1p + 1) / 4;
                    b = (*(b0p - 1) + *(b0p + 1) + *(b1p - 1) + *(b1p + 1) + 1) / 4;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
                else
                { /* Green Pixel */
                    r = (*(r0p - 1) + *(r0p + 1)) / 2;
                    g = *r0p;
                    b = (*b0p + *b1p) / 2;
                    *dstp++ = b | (g << 8) | (r << 16);
                    r0p++; b0p++; b1p++;
                }
            }

            /* Last pixel */
            if (j % 2)
            { /* Red pixel */
                r = *r0p;
                g = (*(r0p - 1) + *(r0p - 1) + *b0p + *b1p + 1) / 4;
                b = (*(b0p - 1) + *(b1p - 1)) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
            else
            { /* Green Pixel */
                r = *(r0p - 1);
                g = *r0p;
                b = (*b0p + *b1p) / 2;
                *dstp++ = b | (g << 8) | (r << 16);
            }
        }
    }

    return kOK;
}


#define BAYER_GRADIENT_X(in, i)                                          (in[2][2+i])

#define BAYER_GRADIENT_L_G(in, i)                                    ((-1*in[0][2+i]   +                                            \
                                                                        2*in[1][2+i]   +                                            \
                               -1*in[2][2+i]   +   2*in[2][3+i]   +     4*in[2][2+i]   +   2*in[2][3+i]   +   -1*in[2][4+i]   +     \
                                                                        2*in[3][2+i]   +                                            \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_L_RBH(in, i)                                   ((1*in[0][2+i]/2 +                                            \
                                                  -1*in[1][3+i]   +                       -1*in[1][3+i]   +                         \
                               -1*in[2][2+i]   +   4*in[2][3+i]   +     5*in[2][2+i]   +   4*in[2][3+i]   +   -1*in[2][4+i]   +     \
                                                  -1*in[3][3+i]   +                       -1*in[3][3+i]   +                         \
                                                                        1*in[4][2+i]/2) /8) 

#define BAYER_GRADIENT_L_RBV(in, i)                                  ((-1*in[0][2+i]   +                                            \
                                                  -1*in[1][3+i]   +     4*in[1][2+i]   +  -1*in[1][3+i]   +                         \
                                1*in[2][2+i]/2 +                        5*in[2][2+i]   +                       1*in[2][4+i]/2 +     \
                                                  -1*in[3][3+i]   +     4*in[3][2+i]   +  -1*in[3][3+i]   +                         \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_L_RBD(in, i)                                  ((-3*in[0][2+i]/2 +                                            \
                                                   2*in[1][3+i]   +                        2*in[1][3+i]   +                         \
                               -3*in[2][2+i]/2 +                        6*in[2][2+i]   +                      -3*in[2][4+i]/2 +     \
                                                   2*in[3][3+i]   +                        2*in[3][3+i]   +                         \
                                                                       -3*in[4][2+i]/2) /8)

#define BAYER_GRADIENT_C_G(in, i)                                    ((-1*in[0][2+i]   +                                            \
                                                                        2*in[1][2+i]   +                                            \
                               -1*in[2][0+i]   +   2*in[2][1+i]   +     4*in[2][2+i]   +   2*in[2][3+i]   +   -1*in[2][4+i]   +     \
                                                                        2*in[3][2+i]   +                                            \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_C_RBH(in, i)                                   ((1*in[0][2+i]/2 +                                            \
                                                  -1*in[1][1+i]   +                       -1*in[1][3+i]   +                         \
                               -1*in[2][0+i]   +   4*in[2][1+i]   +     5*in[2][2+i]   +   4*in[2][3+i]   +   -1*in[2][4+i]   +     \
                                                  -1*in[3][1+i]   +                       -1*in[3][3+i]   +                         \
                                                                        1*in[4][2+i]/2) /8) 

#define BAYER_GRADIENT_C_RBV(in, i)                                  ((-1*in[0][2+i]   +                                            \
                                                  -1*in[1][1+i]   +     4*in[1][2+i]   +  -1*in[1][3+i]   +                         \
                                1*in[2][0+i]/2 +                        5*in[2][2+i]   +                       1*in[2][4+i]/2 +     \
                                                  -1*in[3][1+i]   +     4*in[3][2+i]   +  -1*in[3][3+i]   +                         \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_C_RBD(in, i)                                  ((-3*in[0][2+i]/2 +                                            \
                                                   2*in[1][1+i]   +                        2*in[1][3+i]   +                         \
                               -3*in[2][0+i]/2 +                        6*in[2][2+i]   +                      -3*in[2][4+i]/2 +     \
                                                   2*in[3][1+i]   +                        2*in[3][3+i]   +                         \
                                                                       -3*in[4][2+i]/2) /8)

#define BAYER_GRADIENT_R_G(in, i)                                    ((-1*in[0][2+i]   +                                            \
                                                                        2*in[1][2+i]   +                                            \
                               -1*in[2][0+i]   +   2*in[2][1+i]   +     4*in[2][2+i]   +   2*in[2][1+i]   +   -1*in[2][2+i]   +     \
                                                                        2*in[3][2+i]   +                                            \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_R_RBH(in, i)                                   ((1*in[0][2+i]/2 +                                            \
                                                  -1*in[1][1+i]   +                       -1*in[1][1+i]   +                         \
                               -1*in[2][0+i]   +   4*in[2][1+i]   +     5*in[2][2+i]   +   4*in[2][1+i]   +   -1*in[2][2+i]   +     \
                                                  -1*in[3][1+i]   +                       -1*in[3][1+i]   +                         \
                                                                        1*in[4][2+i]/2) /8) 

#define BAYER_GRADIENT_R_RBV(in, i)                                  ((-1*in[0][2+i]   +                                            \
                                                  -1*in[1][1+i]   +     4*in[1][2+i]   +  -1*in[1][1+i]   +                         \
                                1*in[2][0+i]/2 +                        5*in[2][2+i]   +                       1*in[2][2+i]/2 +     \
                                                  -1*in[3][1+i]   +     4*in[3][2+i]   +  -1*in[3][1+i]   +                         \
                                                                       -1*in[4][2+i])   /8)

#define BAYER_GRADIENT_R_RBD(in, i)                                  ((-3*in[0][2+i]/2 +                                            \
                                                   2*in[1][1+i]   +                        2*in[1][1+i]   +                         \
                               -3*in[2][0+i]/2 +                        6*in[2][2+i]   +                      -3*in[2][2+i]/2 +     \
                                                   2*in[3][1+i]   +                        2*in[3][1+i]   +                         \
                                                                       -3*in[4][2+i]/2) /8)

#define BAYER_GRADIENT_ASSIGN(out, i, r, g, b)                                      \
            out[i].r = (k8u) kClamp_(r, 0, k8U_MAX);                            \
            out[i].g = (k8u) kClamp_(g, 0, k8U_MAX);                            \
            out[i].b = (k8u) kClamp_(b, 0, k8U_MAX)

#define BAYER_GRADIENT_ADVANCE(in, out, i)                                          \
            in[0] += i, in[1] += i, in[2] += i, in[3] += i, in[4] += i; out += i

kStatus GoAlgorithm_BayerDemosaicGradientBggr(kImage input, kImage output)
{
    k32u width = (k32u)kImage_Width(input);
    k32u height = (k32u)kImage_Height(input);
    k32u pitch = (k32u)(kImage_Stride(input)/kImage_PixelSize(input));
    k8u* base = (kImage_DataT(input, k8u)) - 2;
    k8u* in[5];
    k8u* end0;
    kRgb* out;
    k32s r, g, b;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        (kImage_Width(output) != width) ||
        (kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2) ||
        (kImage_Width(input) < 4) ||
        (kImage_Height(input) < 4))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < height; ++i)
    {
        in[0] = (i > 1) ? &base[(i - 2)*pitch] : &base[(i)*pitch];
        in[1] = (i > 0) ? &base[(i - 1)*pitch] : &base[(i + 1)*pitch];
        in[2] = &base[(i)*pitch];
        in[3] = (i < (height - 1)) ? &base[(i + 1)*pitch] : &base[(i - 1)*pitch];
        in[4] = (i < (height - 2)) ? &base[(i + 2)*pitch] : &base[(i)*pitch];
        end0 = in[0] + width - 2;
        out = kImage_AtT(output, 0, i, kRgb);

        if ((i & 1) == 0)
        {
            r = BAYER_GRADIENT_L_RBD(in, 0);
            g = BAYER_GRADIENT_L_G(in, 0);
            b = BAYER_GRADIENT_X(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBV(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_L_RBH(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBD(in, 0);
                g = BAYER_GRADIENT_C_G(in, 0);
                b = BAYER_GRADIENT_X(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBV(in, 1);
                g = BAYER_GRADIENT_X(in, 1);
                b = BAYER_GRADIENT_C_RBH(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBD(in, 0);
            g = BAYER_GRADIENT_R_G(in, 0);
            b = BAYER_GRADIENT_X(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBV(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_R_RBH(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
        else
        {
            r = BAYER_GRADIENT_L_RBH(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_L_RBV(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_X(in, 1);
            g = BAYER_GRADIENT_L_G(in, 1);
            b = BAYER_GRADIENT_L_RBD(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBH(in, 0);
                g = BAYER_GRADIENT_X(in, 0);
                b = BAYER_GRADIENT_C_RBV(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_X(in, 1);
                g = BAYER_GRADIENT_C_G(in, 1);
                b = BAYER_GRADIENT_C_RBD(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBH(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_R_RBV(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_X(in, 1);
            g = BAYER_GRADIENT_R_G(in, 1);
            b = BAYER_GRADIENT_R_RBD(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicGradientGbrg(kImage input, kImage output)
{
    k32u width = (k32u)kImage_Width(input);
    k32u height = (k32u)kImage_Height(input);
    k32u pitch = (k32u)(k32u)(kImage_Stride(input)/kImage_PixelSize(input));
    k8u* base = (kImage_DataT(input, k8u)) - 2;
    k8u* in[5];
    k8u* end0;
    kRgb* out;
    k32s r, g, b;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        (kImage_Width(output) != width) ||
        (kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2) ||
        (kImage_Width(input) < 4) ||
        (kImage_Height(input) < 4))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < height; ++i)
    {
        in[0] = (i > 1) ? &base[(i - 2)*pitch] : &base[(i)*pitch];
        in[1] = (i > 0) ? &base[(i - 1)*pitch] : &base[(i + 1)*pitch];
        in[2] = &base[(i)*pitch];
        in[3] = (i < (height - 1)) ? &base[(i + 1)*pitch] : &base[(i - 1)*pitch];
        in[4] = (i < (height - 2)) ? &base[(i + 2)*pitch] : &base[(i)*pitch];
        end0 = in[0] + width - 2;
        out = kImage_AtT(output, 0, i, kRgb);

        if ((i & 1) == 1)
        {
            r = BAYER_GRADIENT_X(in, 0);
            g = BAYER_GRADIENT_L_G(in, 0);
            b = BAYER_GRADIENT_L_RBD(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBH(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_L_RBV(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_X(in, 0);
                g = BAYER_GRADIENT_C_G(in, 0);
                b = BAYER_GRADIENT_C_RBD(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBH(in, 1);
                g = BAYER_GRADIENT_X(in, 1);
                b = BAYER_GRADIENT_C_RBV(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_X(in, 0);
            g = BAYER_GRADIENT_R_G(in, 0);
            b = BAYER_GRADIENT_R_RBD(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBH(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_R_RBV(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
        else
        {
            r = BAYER_GRADIENT_L_RBV(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_L_RBH(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBD(in, 1);
            g = BAYER_GRADIENT_L_G(in, 1);
            b = BAYER_GRADIENT_X(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBV(in, 0);
                g = BAYER_GRADIENT_X(in, 0);
                b = BAYER_GRADIENT_C_RBH(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBD(in, 1);
                g = BAYER_GRADIENT_C_G(in, 1);
                b = BAYER_GRADIENT_X(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBV(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_R_RBH(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBD(in, 1);
            g = BAYER_GRADIENT_R_G(in, 1);
            b = BAYER_GRADIENT_X(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicGradientRggb(kImage input, kImage output)
{
    k32u width = (k32u)kImage_Width(input);
    k32u height = (k32u)kImage_Height(input);
    k32u pitch = (k32u)(kImage_Stride(input)/kImage_PixelSize(input));
    k8u* base = (kImage_DataT(input, k8u)) - 2;
    k8u* in[5];
    k8u* end0;
    kRgb* out;
    k32s r, g, b;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        (kImage_Width(output) != width) ||
        (kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2) ||
        (kImage_Width(input) < 4) ||
        (kImage_Height(input) < 4))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < height; ++i)
    {
        in[0] = (i > 1) ? &base[(i - 2)*pitch] : &base[(i)*pitch];
        in[1] = (i > 0) ? &base[(i - 1)*pitch] : &base[(i + 1)*pitch];
        in[2] = &base[(i)*pitch];
        in[3] = (i < (height - 1)) ? &base[(i + 1)*pitch] : &base[(i - 1)*pitch];
        in[4] = (i < (height - 2)) ? &base[(i + 2)*pitch] : &base[(i)*pitch];
        end0 = in[0] + width - 2;
        out = kImage_AtT(output, 0, i, kRgb);

        if ((i & 1) == 0)
        {
            r = BAYER_GRADIENT_X(in, 0);
            g = BAYER_GRADIENT_L_G(in, 0);
            b = BAYER_GRADIENT_L_RBD(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBH(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_L_RBV(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_X(in, 0);
                g = BAYER_GRADIENT_C_G(in, 0);
                b = BAYER_GRADIENT_C_RBD(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBH(in, 1);
                g = BAYER_GRADIENT_X(in, 1);
                b = BAYER_GRADIENT_C_RBV(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_X(in, 0);
            g = BAYER_GRADIENT_R_G(in, 0);
            b = BAYER_GRADIENT_R_RBD(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBH(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_R_RBV(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
        else
        {
            r = BAYER_GRADIENT_L_RBV(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_L_RBH(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBD(in, 1);
            g = BAYER_GRADIENT_L_G(in, 1);
            b = BAYER_GRADIENT_X(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBV(in, 0);
                g = BAYER_GRADIENT_X(in, 0);
                b = BAYER_GRADIENT_C_RBH(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBD(in, 1);
                g = BAYER_GRADIENT_C_G(in, 1);
                b = BAYER_GRADIENT_X(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBV(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_R_RBH(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBD(in, 1);
            g = BAYER_GRADIENT_R_G(in, 1);
            b = BAYER_GRADIENT_X(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicGradientGrbg(kImage input, kImage output)
{
    k32u width = (k32u)kImage_Width(input);
    k32u height = (k32u)kImage_Height(input);
    k32u pitch = (k32u)(kImage_Stride(input)/kImage_PixelSize(input));
    k8u* base = (kImage_DataT(input, k8u)) - 2;
    k8u* in[5];
    k8u* end0;
    kRgb* out;
    k32s r, g, b;
    k32u i;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_PixelSize(output) != sizeof(kRgb)) ||
        (kImage_Width(output) != width) ||
        (kImage_Height(output) != height) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2) ||
        (kImage_Width(input) < 4) ||
        (kImage_Height(input) < 4))
    {
        return kERROR_PARAMETER;
    }

    for (i = 0; i < height; ++i)
    {
        in[0] = (i > 1) ? &base[(i - 2)*pitch] : &base[(i)*pitch];
        in[1] = (i > 0) ? &base[(i - 1)*pitch] : &base[(i + 1)*pitch];
        in[2] = &base[(i)*pitch];
        in[3] = (i < (height - 1)) ? &base[(i + 1)*pitch] : &base[(i - 1)*pitch];
        in[4] = (i < (height - 2)) ? &base[(i + 2)*pitch] : &base[(i)*pitch];
        end0 = in[0] + width - 2;
        out = kImage_AtT(output, 0, i, kRgb);

        if ((i & 1) == 1)
        {
            r = BAYER_GRADIENT_L_RBD(in, 0);
            g = BAYER_GRADIENT_L_G(in, 0);
            b = BAYER_GRADIENT_X(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_L_RBV(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_L_RBH(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBD(in, 0);
                g = BAYER_GRADIENT_C_G(in, 0);
                b = BAYER_GRADIENT_X(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_C_RBV(in, 1);
                g = BAYER_GRADIENT_X(in, 1);
                b = BAYER_GRADIENT_C_RBH(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBD(in, 0);
            g = BAYER_GRADIENT_R_G(in, 0);
            b = BAYER_GRADIENT_X(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_R_RBV(in, 1);
            g = BAYER_GRADIENT_X(in, 1);
            b = BAYER_GRADIENT_R_RBH(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
        else
        {
            r = BAYER_GRADIENT_L_RBH(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_L_RBV(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_X(in, 1);
            g = BAYER_GRADIENT_L_G(in, 1);
            b = BAYER_GRADIENT_L_RBD(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);

            while (in[0] != end0)
            {
                r = BAYER_GRADIENT_C_RBH(in, 0);
                g = BAYER_GRADIENT_X(in, 0);
                b = BAYER_GRADIENT_C_RBV(in, 0);
                BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

                r = BAYER_GRADIENT_X(in, 1);
                g = BAYER_GRADIENT_C_G(in, 1);
                b = BAYER_GRADIENT_C_RBD(in, 1);
                BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

                BAYER_GRADIENT_ADVANCE(in, out, 2);
            }

            r = BAYER_GRADIENT_R_RBH(in, 0);
            g = BAYER_GRADIENT_X(in, 0);
            b = BAYER_GRADIENT_R_RBV(in, 0);
            BAYER_GRADIENT_ASSIGN(out, 0, r, g, b);

            r = BAYER_GRADIENT_X(in, 1);
            g = BAYER_GRADIENT_R_G(in, 1);
            b = BAYER_GRADIENT_R_RBD(in, 1);
            BAYER_GRADIENT_ASSIGN(out, 1, r, g, b);

            BAYER_GRADIENT_ADVANCE(in, out, 2);
        }
    }

    return kOK;
}

kStatus GoAlgorithm_BayerDemosaicReduce(kImage input, kImage output)
{
    switch (kImage_Cfa(input))
    {
    case kCFA_BAYER_BGGR:           return GoAlgorithm_BayerDemosaicReduceBggr(input, output);
    case kCFA_BAYER_GBRG:           return GoAlgorithm_BayerDemosaicReduceGbrg(input, output);
    case kCFA_BAYER_RGGB:           return GoAlgorithm_BayerDemosaicReduceRggb(input, output);
    case kCFA_BAYER_GRBG:           return GoAlgorithm_BayerDemosaicReduceGrbg(input, output);
    default:                        return kERROR_PARAMETER;
    }
}

kStatus GoAlgorithm_BayerDemosaicBilinear(kImage input, kImage output)
{
    switch (kImage_Cfa(input))
    {
    case kCFA_BAYER_BGGR:           return GoAlgorithm_BayerDemosaicBilinearBggr(input, output);
    case kCFA_BAYER_GBRG:           return GoAlgorithm_BayerDemosaicBilinearGbrg(input, output);
    case kCFA_BAYER_RGGB:           return GoAlgorithm_BayerDemosaicBilinearRggb(input, output);
    case kCFA_BAYER_GRBG:           return GoAlgorithm_BayerDemosaicBilinearGrbg(input, output);
    default:                        return kERROR_PARAMETER;
    }
}

kStatus GoAlgorithm_BayerDemosaicGradient(kImage input, kImage output)
{
    switch (kImage_Cfa(input))
    {
    case kCFA_BAYER_BGGR:           return GoAlgorithm_BayerDemosaicGradientBggr(input, output);
    case kCFA_BAYER_GBRG:           return GoAlgorithm_BayerDemosaicGradientGbrg(input, output);
    case kCFA_BAYER_RGGB:           return GoAlgorithm_BayerDemosaicGradientRggb(input, output);
    case kCFA_BAYER_GRBG:           return GoAlgorithm_BayerDemosaicGradientGrbg(input, output);
    default:                        return kERROR_PARAMETER;
    }
}

#endif //COPY_BAYER_ALGORITHM_FROM_KVISION

GoFx(kStatus) GoAlgorithm_Demosaic(kImage input, kImage* output, GoDemosaicStyle style, kAlloc allocator)
{
    kImage newImage;
    kStatus exception;
    kSize height, width;

    if ((kImage_PixelSize(input) != sizeof(k8u)) ||
        (kImage_Width(input) % 2) ||
        (kImage_Height(input) % 2) ||
        (kImage_Width(input) < 4) ||
        (kImage_Height(input) < 4))
    {
        return kERROR_PARAMETER;
    }

    if (GO_DEMOSAIC_STYLE_REDUCE == style)
    {
        height = kImage_Height(input) / 2;
        width = kImage_Width(input) / 2;
    }
    else
    {
        height = kImage_Height(input);
        width = kImage_Width(input);
    }

    kCheck(kImage_Construct(&newImage, kTypeOf(kRgb), width, height, allocator));

    kTry
    {
        switch (style)
        {
        case GO_DEMOSAIC_STYLE_REDUCE:            kTest(GoAlgorithm_BayerDemosaicReduce(input, newImage)); break;
        case GO_DEMOSAIC_STYLE_BILINEAR:          kTest(GoAlgorithm_BayerDemosaicBilinear(input, newImage)); break;
        case GO_DEMOSAIC_STYLE_GRADIENT:          kTest(GoAlgorithm_BayerDemosaicGradient(input, newImage)); break;
        default:                                    kThrow(kERROR_PARAMETER);
        }

        kDestroyRef(output); //Free the image stored at the output pointer if there is already one there
        //This allows them to call GoAlgorithm_Demosaic(myImage, &myImage, ...)
        //Without memory leaks.

    }
    kCatch(&exception)
    {
        kDestroyRef(&newImage);
        *output = kNULL;
        kEndCatch(exception);
    }

    *output = newImage;

    return kOK;
}
