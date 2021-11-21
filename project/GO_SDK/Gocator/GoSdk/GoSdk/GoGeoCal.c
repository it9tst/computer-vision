/**
* @file    GoGeoCal.c
*
* @internal
* Copyright (C) 2017-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include <GoSdk/GoGeoCal.h>

#include <kApi/Data/kArray1.h>
#include <kApi/Data/kBytes.h>

kBeginClassEx(Go, GoGeoCal)
kAddVMethod(GoGeoCal, kObject, VRelease)
kEndClassEx()

GoFx(kStatus) GoGeoCal_Construct(GoGeoCal* geoCal, kXml xml, kAlloc allocator)
{
    kAlloc alloc = kAlloc_Fallback(allocator);
    kStatus status;

    kCheck(kAlloc_GetObject(alloc, kTypeOf(GoGeoCal), geoCal));

    if (!kSuccess(status = GoGeoCal_Init(*geoCal, kTypeOf(GoGeoCal), xml, alloc)))
    {
        kAlloc_FreeRef(alloc, geoCal);
    }

    return status;
}

GoFx(kStatus) GoGeoCal_Init(GoGeoCal geoCal, kType type, kXml xml, kAlloc alloc)
{
    kObjR(GoGeoCal, geoCal);
    kXmlItem root = kNULL;
    kXmlItem camerasItem = kNULL;
    kXmlItem cameraItem = kNULL;
    kText256 buffer;
    k32u version;
    kStatus status;
    kSize i = 0, j;

    kCheck(kObject_Init(geoCal, type, alloc));
    obj->id = 0;
    obj->timestamp[0] = 0;
    kZero(obj->cameras);

    root = kXml_Root(xml);

    kTry
    {
        kTest(kXml_Attr32u(xml, root, "version", &version));
        kTest(version == GO_GEO_CAL_VERSION);

        kTest(kXml_Child32u(xml, root, "Attributes\\Id", &obj->id));
        kTest(kXml_ChildText(xml, root, "Attributes\\Timestamp", obj->timestamp, kCountOf(obj->timestamp)));

        camerasItem = kXml_Child(xml, root, "Cameras");
        if (!kIsNull(camerasItem))
        {
            kTest(kArray1_Construct(&obj->cameras, kBytes_GetType(sizeof(GoGeoCalCamera)), kXml_ChildCount(xml, camerasItem), alloc));
            kTest(kArray1_Zero(obj->cameras));

            for (cameraItem = kXml_FirstChild(xml, camerasItem); cameraItem != kNULL; cameraItem = kXml_NextSibling(xml, cameraItem))
            {
                GoGeoCalCamera* camera = kArray1_AtT(obj->cameras, i, GoGeoCalCamera);
                kXmlItem xResItem = kNULL;
                kXmlItem yResItem = kNULL;

                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\X", &camera->calWindow.x));
                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\Y", &camera->calWindow.y));
                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\Width", &camera->calWindow.width));
                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\Height", &camera->calWindow.height));
                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\XSubsampling", &camera->calWindow.xSubsampling));
                kTest(kXml_Child32u(xml, cameraItem, "CalWindow\\YSubsampling", &camera->calWindow.ySubsampling));

                if (kXml_ChildExists(xml, cameraItem, "XCenterAsZ"))
                {
                    kTest(kArray1_Construct(&camera->xCenterCoeffs, kTypeOf(k64f), kXml_ChildCount(xml, kXml_Child(xml, cameraItem, "XCenterAsZ")), alloc));
                    for (j = 0; j < kArray1_Length(camera->xCenterCoeffs); j++)
                    {
                        kTest(kStrPrintf(buffer, kCountOf(buffer), "XCenterAsZ\\C%d", j));
                        kTest(kXml_Child64f(xml, cameraItem, buffer, kArray1_AtT(camera->xCenterCoeffs, j, k64f)));
                    }
                }

                if (kXml_ChildExists(xml, cameraItem, "YCenterAsZ"))
                {
                    kTest(kArray1_Construct(&camera->yCenterCoeffs, kTypeOf(k64f), kXml_ChildCount(xml, kXml_Child(xml, cameraItem, "YCenterAsZ")), alloc));
                    for (j = 0; j < kArray1_Length(camera->yCenterCoeffs); j++)
                    {
                        kTest(kStrPrintf(buffer, kCountOf(buffer), "YCenterAsZ\\C%d", j));
                        kTest(kXml_Child64f(xml, cameraItem, buffer, kArray1_AtT(camera->yCenterCoeffs, j, k64f)));
                    }
                }

                xResItem = kXml_Child(xml, cameraItem, "XResolutionAsZ");
                if (!kIsNull(xResItem)) /* Optional */
                {
                    kTest(kArray1_Construct(&camera->xResCoeffs, kTypeOf(k64f), kXml_ChildCount(xml, xResItem), alloc));
                    for (j = 0; j < kArray1_Length(camera->xResCoeffs); j++)
                    {
                        kTest(kStrPrintf(buffer, kCountOf(buffer), "XResolutionAsZ\\C%d", j));
                        kTest(kXml_Child64f(xml, cameraItem, buffer, kArray1_AtT(camera->xResCoeffs, j, k64f)));
                    }
                }

                yResItem = kXml_Child(xml, cameraItem, "YResolutionAsZ");
                if (!kIsNull(yResItem)) /* Optional */
                {
                    kTest(kArray1_Construct(&camera->yResCoeffs, kTypeOf(k64f), kXml_ChildCount(xml, yResItem), alloc));
                    for (j = 0; j < kArray1_Length(camera->yResCoeffs); j++)
                    {
                        kTest(kStrPrintf(buffer, kCountOf(buffer), "YResolutionAsZ\\C%d", j));
                        kTest(kXml_Child64f(xml, cameraItem, buffer, kArray1_AtT(camera->yResCoeffs, j, k64f)));
                    }
                }

                kTest(kXml_Child64f(xml, cameraItem, "Roll", &camera->roll));
                i++;
            }
        }
    }
    kCatch(&status)
    {
        GoGeoCal_VRelease(geoCal);
        kEndCatch(status);
    }

    return kOK;
}

GoFx(kStatus) GoGeoCal_VRelease(GoGeoCal geoCal)
{
    kObj(GoGeoCal, geoCal);
    kSize i;

    if (!kIsNull(obj->cameras))
    {
        for (i = 0; i < kArray1_Length(obj->cameras); i++)
        {
            GoGeoCalCamera* camera = kArray1_AtT(obj->cameras, i, GoGeoCalCamera);
            kCheck(kDestroyRef(&camera->xCenterCoeffs));
            kCheck(kDestroyRef(&camera->yCenterCoeffs));
            kCheck(kDestroyRef(&camera->xResCoeffs));
            kCheck(kDestroyRef(&camera->yResCoeffs));
        }
        kCheck(kDestroyRef(&obj->cameras));
    }

    return kObject_VRelease(geoCal);
}

GoFx(k32u) GoGeoCal_Id(GoGeoCal cal)
{
    kObj(GoGeoCal, cal);

    return obj->id;
}

GoFx(kStatus) GoGeoCal_SetId(GoGeoCal cal, k32u id)
{
    kObj(GoGeoCal, cal);

    obj->id = id;

    return kOK;
}

GoFx(const kChar*) GoGeoCal_Timestamp(GoGeoCal cal)
{
    kObj(GoGeoCal, cal);

    return &obj->timestamp[0];
}

GoFx(kStatus) GoGeoCal_SetTimestamp(GoGeoCal cal, const kChar* timestamp)
{
    kObj(GoGeoCal, cal);

    kCheck(kStrCopy(obj->timestamp, kCountOf(obj->timestamp), timestamp));

    return kOK;
}

GoFx(kSize) GoGeoCal_CameraCount(GoGeoCal cal)
{
    kObj(GoGeoCal, cal);

    return kArray1_Length(obj->cameras);
}

GoFx(kStatus) GoGeoCal_CalWindow(GoGeoCal cal, kSize cameraIndex, k32u* x, k32u* y, k32u* width, k32u* height, k32u* xSubsampling, k32u* ySubsampling)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    if (!kIsNull(x))                *x = camera->calWindow.x;
    if (!kIsNull(y))                *y = camera->calWindow.y;
    if (!kIsNull(width))            *width = camera->calWindow.width;
    if (!kIsNull(height))           *height = camera->calWindow.height;
    if (!kIsNull(xSubsampling))     *xSubsampling = camera->calWindow.xSubsampling;
    if (!kIsNull(ySubsampling))     *ySubsampling = camera->calWindow.ySubsampling;

    return kOK;
}

GoFx(kStatus) GoGeoCal_SetCalWindow(GoGeoCal cal, kSize cameraIndex, k32u x, k32u y, k32u width, k32u height, k32u xSubsampling, k32u ySubsampling)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    camera->calWindow.x = x;
    camera->calWindow.y = y;
    camera->calWindow.width = width;
    camera->calWindow.height = height;
    camera->calWindow.xSubsampling = xSubsampling;
    camera->calWindow.ySubsampling = ySubsampling;

    return kOK;
}

GoFx(kArray1) GoGeoCal_XResolutionFit(GoGeoCal cal, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kNULL;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    return camera->xResCoeffs;
}

GoFx(kStatus) GoGeoCal_SetXResolutionFit(GoGeoCal cal, kSize cameraIndex, kArray1 coefficients)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    kCheck(kObject_Clone(&camera->xResCoeffs, coefficients, kObject_Alloc(cal)));

    return kOK;
}

GoFx(kArray1) GoGeoCal_YResolutionFit(GoGeoCal cal, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kNULL;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    return camera->yResCoeffs;
}

GoFx(kStatus) GoGeoCal_SetYResolutionFit(GoGeoCal cal, kSize cameraIndex, kArray1 coefficients)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    kCheck(kObject_Clone(&camera->yResCoeffs, coefficients, kObject_Alloc(cal)));

    return kOK;
}

GoFx(kArray1) GoGeoCal_XCenterFit(GoGeoCal cal, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kNULL;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    return camera->xCenterCoeffs;
}

GoFx(kStatus) GoGeoCal_SetXCenterFit(GoGeoCal cal, kSize cameraIndex, kArray1 coefficients)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    kCheck(kObject_Clone(&camera->xCenterCoeffs, coefficients, kObject_Alloc(cal)));

    return kOK;
}

GoFx(kArray1) GoGeoCal_YCenterFit(GoGeoCal cal, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kNULL;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    return camera->yCenterCoeffs;
}

GoFx(kStatus) GoGeoCal_SetYCenterFit(GoGeoCal cal, kSize cameraIndex, kArray1 coefficients)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    kCheck(kObject_Clone(&camera->yCenterCoeffs, coefficients, kObject_Alloc(cal)));

    return kOK;
}

GoFx(kStatus) GoGeoCal_SetRoll(GoGeoCal cal, kSize cameraIndex, k64f roll)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);
    camera->roll = roll;

    return kOK;
}

GoFx(k64f) GoGeoCal_Roll(GoGeoCal cal, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return k64F_NULL;

    camera = kArray1_AtT(obj->cameras, cameraIndex, GoGeoCalCamera);

    return camera->roll;
}

GoFx(kStatus) GoGeoCal_ApplyActiveArea(GoGeoCal cal, k32u x, k32u y, k32u width, k32u height, k32u xSubsampling, k32u ySubsampling, kSize cameraIndex)
{
    kObj(GoGeoCal, cal);
    GoGeoCalCamera* camera;
    k32u calX, calY, calWidth, calHeight, calXSubsampling, calYSubsampling;
    k64f xScale, yScale;
    k64f xDifference, yDifference;
    k64f xCenterNominal, xResNominal, xCenterShiftedNominal, yCenterNominal, yResNominal, yCenterShiftedNominal;

    if (cameraIndex >= kArray1_Length(obj->cameras))
        return kERROR_PARAMETER;

    camera = (GoGeoCalCamera*)kArray1_At(obj->cameras, cameraIndex);

    kCheck(GoGeoCal_CalWindow(cal, cameraIndex, &calX, &calY, &calWidth, &calHeight, &calXSubsampling, &calYSubsampling));

    xScale = xSubsampling / calXSubsampling;
    yScale = ySubsampling / calYSubsampling;

    xDifference = ((k64f)x - calX) + (((k64f)width * xScale - calWidth) / 2);
    yDifference = ((k64f)y - calY) + (((k64f)height * yScale - calHeight) / 2);

    xCenterNominal = *(k64f*)kArray1_At(camera->xCenterCoeffs, 1) + *(k64f*)kArray1_At(camera->xCenterCoeffs, 0);
    xResNominal =  *(k64f*)kArray1_At(camera->xResCoeffs, 1) + *(k64f*)kArray1_At(camera->xResCoeffs, 0);
    xCenterShiftedNominal = xCenterNominal + xDifference * xResNominal;

    yCenterNominal = *(k64f*)kArray1_At(camera->yCenterCoeffs, 1) + *(k64f*)kArray1_At(camera->yCenterCoeffs, 0);
    yResNominal =  *(k64f*)kArray1_At(camera->yResCoeffs, 1) + *(k64f*)kArray1_At(camera->yResCoeffs, 0);
    yCenterShiftedNominal = yCenterNominal + yDifference * yResNominal;

    *(k64f*)kArray1_At(camera->xCenterCoeffs, 0) += xDifference * (*(k64f*)kArray1_At(camera->xResCoeffs, 0));
    *(k64f*)kArray1_At(camera->yCenterCoeffs, 0) += yDifference * (*(k64f*)kArray1_At(camera->yResCoeffs, 0));

    *(k64f*)kArray1_At(camera->xCenterCoeffs, 1) = xCenterShiftedNominal - (*(k64f*)kArray1_At(camera->xCenterCoeffs, 0));
    *(k64f*)kArray1_At(camera->yCenterCoeffs, 1) = yCenterShiftedNominal - (*(k64f*)kArray1_At(camera->yCenterCoeffs, 0));

    if (!kIsNull(camera->xResCoeffs))
    {
        kCheck(kMath_MulC64f(kArray1_Data(camera->xResCoeffs), kArray1_Data(camera->xResCoeffs), kArray1_Length(camera->xResCoeffs), xScale));
    }
    if (!kIsNull(camera->yResCoeffs))
    {
        kCheck(kMath_MulC64f(kArray1_Data(camera->yResCoeffs), kArray1_Data(camera->yResCoeffs), kArray1_Length(camera->yResCoeffs), yScale));
    }

    return kOK;
}