/**
* @file    GdkSurfaceTrackInspectionResult.cpp
*
* @internal
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#include "GdkSurfaceTrackInspectionResult.h"

#include <kApi/Data/kArray1.h>
#include <kApi/Io/kMemory.h>
#include <kApi/Io/kSerializer.h>

kStatus GdkSurfaceTrackInspectionResult_SerializeGenericOutput(const std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum, kArray1* outputBuffer)
{
    if (!kIsNull(outputBuffer))
    {
        kMemory memory = kNULL;
        kSerializer serializer = kNULL;
        kStatus exception = kOK;

        kTry
        {
            kTest(kMemory_Construct(&memory, kNULL));
            kTest(kSerializer_Construct(&serializer, memory, kNULL, kNULL));

            // version == 1 or 2: Old serializer, only one measured data section with kTypeOf(k16u) is sent
            k32u version = 3;

            // The first measured data section with kTypeOf(k16u) format is used to pass a version information
            // By doing so, the old deserializer using kTypeOf(k16u) can retrieve the version
            kTest(kSerializer_BeginWrite(serializer, kTypeOf(k16u), kFALSE));
            kTest(kSerializer_WriteSize(serializer, 1));            // A non-zero value ensures the old deserializer can read the version information
            kTest(kSerializer_Write32u(serializer, version));
            kTest(kSerializer_EndWrite(serializer));

            // The second measured data section package measurements to transfer
            // Previously, we used kTypeOf(k16u) to record the size of the writing measured data section.
            // It is inadequate in some cases and thus fail to finish writing the data section.
            // Use kTypeOf(k32u) as suggested by Rex Wen to cover most cases
            kTest(kSerializer_BeginWrite(serializer, kTypeOf(k32u), kFALSE));
            kTest(kSerializer_WriteSize(serializer, elementNum));
            if (elementNum)
            {
                for (kSize i = 0; i < elementNum; i++)
                {
                    kTest(kSerializer_Write32u(serializer, elements[i].trackID));
                    kTest(kSerializer_Write32u(serializer, elements[i].segmentID));
                    kTest(kSerializer_Write64f(serializer, elements[i].width));
                    kTest(kSerializer_Write64f(serializer, elements[i].peakHeight));
                    kTest(kSerializer_Write64f(serializer, elements[i].offset));
                    kTest(kSerializer_Write64f(serializer, elements[i].centerX));
                    kTest(kSerializer_Write64f(serializer, elements[i].centerY));
                    kTest(kSerializer_Write64f(serializer, elements[i].area));
                }
            }
            kTest(kSerializer_EndWrite(serializer));

            kTest(kSerializer_Flush(serializer));

            auto len = (kSize)kMemory_Length(memory);
            kTest(kArray1_Construct(outputBuffer, kTypeOf(k8u), len, kNULL));
            kTest(kMemCopy(kArray1_Data(*outputBuffer), kMemory_At(memory, 0), len));
        }
        kCatchEx(&exception)
        {
            kDestroyRef(outputBuffer);
            kEndCatchEx(exception);
        }
        kFinallyEx
        {
            kDestroyRef(&memory);
            kDestroyRef(&serializer);
            kEndFinallyEx();
        }
    }

    return kOK;
}

kStatus GdkSurfaceTrackInspectionResult_DeserializeGenericOutput(std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum, void* inputBuffer, kSize length, kAlloc alloc)
{
    kSerializer serializer = kNULL;
    kMemory memory = kNULL;
    kStatus exception = kOK;

    kTry
    {
        kTest(kMemory_Construct(&memory, alloc));
        kTest(kMemory_Attach(memory, inputBuffer, 0, length, length));
        kTest(kSerializer_Construct(&serializer, memory, kNULL, alloc));

        // Read the first measured data section in kTypeOf(k16u) format
        kTest(kSerializer_BeginRead(serializer, kTypeOf(k16u), kFALSE));
        kTest(kSerializer_ReadSize(serializer, &elementNum));

        // only when the elements is not empty, read the following information
        if (elementNum != 0)
        {
            k32u version = 0;
            kTest(kSerializer_Read32u(serializer, &version));

            switch (version)
            {
            default:
                kThrow(kERROR);
            case 1:
                for (kSize i = 0; i < elementNum; ++i)
                {
                    GdkSurfaceTrackInspectionResultStruct output;
                    kZero(output);

                    kTest(kSerializer_Read32u(serializer, &output.trackID));
                    kTest(kSerializer_Read32u(serializer, &output.segmentID));
                    kTest(kSerializer_Read64f(serializer, &output.width));
                    kTest(kSerializer_Read64f(serializer, &output.peakHeight));
                    kTest(kSerializer_Read64f(serializer, &output.offset));
                    kTest(kSerializer_Read64f(serializer, &output.centerX));
                    kTest(kSerializer_Read64f(serializer, &output.centerY));
                    elements.push_back(output);
                }
                break;
            case 2:
                for (kSize i = 0; i < elementNum; ++i)
                {
                    GdkSurfaceTrackInspectionResultStruct output;
                    kZero(output);

                    kTest(kSerializer_Read32u(serializer, &output.trackID));
                    kTest(kSerializer_Read32u(serializer, &output.segmentID));
                    kTest(kSerializer_Read64f(serializer, &output.width));
                    kTest(kSerializer_Read64f(serializer, &output.peakHeight));
                    kTest(kSerializer_Read64f(serializer, &output.offset));
                    kTest(kSerializer_Read64f(serializer, &output.centerX));
                    kTest(kSerializer_Read64f(serializer, &output.centerY));
                    kTest(kSerializer_Read64f(serializer, &output.area));
                    elements.push_back(output);
                }
                break;
            case 3:
                // End reading the first measured data section containing the version information
                kTest(kSerializer_EndRead(serializer));

                // Start reading the second measured data section in kTypeOf(k32u) format
                kTest(kSerializer_BeginRead(serializer, kTypeOf(k32u), kFALSE));
                kTest(kSerializer_ReadSize(serializer, &elementNum));
                for (kSize i = 0; i < elementNum; ++i)
                {
                    GdkSurfaceTrackInspectionResultStruct output;
                    kZero(output);

                    kTest(kSerializer_Read32u(serializer, &output.trackID));
                    kTest(kSerializer_Read32u(serializer, &output.segmentID));
                    kTest(kSerializer_Read64f(serializer, &output.width));
                    kTest(kSerializer_Read64f(serializer, &output.peakHeight));
                    kTest(kSerializer_Read64f(serializer, &output.offset));
                    kTest(kSerializer_Read64f(serializer, &output.centerX));
                    kTest(kSerializer_Read64f(serializer, &output.centerY));
                    kTest(kSerializer_Read64f(serializer, &output.area));
                    elements.push_back(output);
                }
                break;
            }
        }

        kTest(kSerializer_EndRead(serializer));
    }
    kCatchEx(&exception)
    {
        kEndCatchEx(exception);
    }
    kFinallyEx
    {
        kDestroyRef(&serializer);
        kDestroyRef(&memory);
        kEndFinallyEx();
    }

    return kOK;
}

#ifndef GDKAPP

kBool GdkSurfaceTrackInspectionResult_TypeValid(GoGenericMsg genMsg)
{
    if (genMsg)
    {
        k32u utype = GoGenericMsg_UserType(genMsg);
        if (utype == GDK_SURFACE_TRACK_INSPECTION_DATA_TYPE)
        {
            return kTRUE;
        }
    }

    return kFALSE;
}

kStatus GdkSurfaceTrackInspectionResult_DeserializeElements(GoGenericMsg genMsg, std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum)
{
    if (GdkSurfaceTrackInspectionResult_TypeValid(genMsg))
    {
        void* ptr = (void*)GoGenericMsg_BufferData(genMsg);
        GdkSurfaceTrackInspectionResult_DeserializeGenericOutput(elements, elementNum, ptr, GoGenericMsg_BufferSize(genMsg), kNULL);

        return kOK;
    }

    return kERROR;
}

#endif
