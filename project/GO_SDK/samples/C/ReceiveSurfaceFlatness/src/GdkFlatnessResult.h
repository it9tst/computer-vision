/**
* @file    GdkFlatnessResult.h
* @brief   Declares the GdkFlatnessResult class.
*
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GDK_FLATNESS_RESULT_H
#define GDK_FLATNESS_RESULT_H

#include <kApi/kApiDef.h>
#include <vector>

// Include for deserialization
#ifndef GDKAPP
#include <GoSdk/GoSdk.h>
#endif

// Include for type ID
#ifndef GDKAPP
#define GDK_DATA_TYPE_GENERIC_BASE                  (0x80000000)    ///< Generic data base
#else
#include <Gdk/GdkDef.h>
#endif

const k32u GDK_FLATNESS_DATA_TYPE = GDK_DATA_TYPE_GENERIC_BASE + 0x00009002;

/**
* @class   GdkFlatnessResult
* @brief   Represents output for grid elements.
*/


struct GdkFlatnessResultStruct
{
    k64f minValue;
    k64f maxValue;
    k64f flatnessValue;

    kPoint3d64f avePoint;
    kPoint3d64f minPoint;
    kPoint3d64f maxPoint;
    kPoint3d64f medianPoint;

    GdkFlatnessResultStruct() :
        minValue{ k64F_NULL }, maxValue{ k64F_NULL }, flatnessValue{ k64F_NULL }
    {
        kPoint3d_Init_(&avePoint, k64F_NULL, k64F_NULL, k64F_NULL);
        kPoint3d_Init_(&minPoint, k64F_NULL, k64F_NULL, k64F_NULL);
        kPoint3d_Init_(&maxPoint, k64F_NULL, k64F_NULL, k64F_NULL);
        kPoint3d_Init_(&medianPoint, k64F_NULL, k64F_NULL, k64F_NULL);
    }
};

typedef GdkFlatnessResultStruct* GdkFlatnessResult;


#define GdkFlatnessResult_Cast_(CONTEXT)    (GdkFlatnessResultStruct*) CONTEXT

kStatus GdkSurfaceFlatness_SerializeGenericOutput(const std::vector<GdkFlatnessResultStruct>& gridElements, kSize & ElementNum, kArray1* outputBuffer);
kStatus GdkFlatnessResult_DeserializeGenericOutput(std::vector<GdkFlatnessResultStruct>& gridElements, kSize & localElementNum, void* inputBuffer, kSize length, kAlloc alloc);

#ifndef GDKAPP
kBool   GdkFlatnessResult_TypeValid(GoGenericMsg genMsg);
kStatus GdkFlatnessResult_DeserializeGridElements(GoGenericMsg genMsg, std::vector<GdkFlatnessResultStruct>& gridElements, kSize & localElementNum);
#endif

#endif
