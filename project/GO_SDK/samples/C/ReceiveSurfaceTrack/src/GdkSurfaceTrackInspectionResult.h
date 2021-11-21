/**
* @file    GdkSurfaceTrackInspectionResult.h
* @brief   Declares the GdkSurfaceTrackInspectionResult class.
*
* Copyright (C) 2016-2021 by LMI Technologies Inc.
* Licensed under the MIT License.
* Redistributed files must retain the above copyright notice.
*/
#ifndef GDK_SURFACE_TRACK_INSPECTION_RESULT_H
#define GDK_SURFACE_TRACK_INSPECTION_RESULT_H

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

using namespace std;

const k32u GDK_SURFACE_TRACK_INSPECTION_DATA_TYPE = GDK_DATA_TYPE_GENERIC_BASE + 0x00009003;

/**
* @class   GdkSurfaceTrackInspectionResult
* @brief   Represents output for elements.
*/


struct GdkSurfaceTrackInspectionResultStruct
{
    k32u trackID;
    k32u segmentID;
    k64f width;
    k64f peakHeight;
    k64f offset;
    k64f centerX;
    k64f centerY;
    k64f area;
};
typedef GdkSurfaceTrackInspectionResultStruct* GdkSurfaceTrackInspectionResult;

#define GdkSurfaceTrackInspectionResult_Cast_(CONTEXT)    (GdkSurfaceTrackInspectionResultStruct*) CONTEXT

kStatus GdkSurfaceTrackInspectionResult_SerializeGenericOutput(const std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum, kArray1* outputBuffer);
kStatus GdkSurfaceTrackInspectionResult_DeserializeGenericOutput(std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum, void* inputBuffer, kSize length, kAlloc alloc);

#ifndef GDKAPP
kBool   GdkSurfaceTrackInspectionResult_TypeValid(GoGenericMsg genMsg);
kStatus GdkSurfaceTrackInspectionResult_DeserializeElements(GoGenericMsg genMsg, std::vector<GdkSurfaceTrackInspectionResultStruct>& elements, kSize & elementNum);
#endif

#endif
