// 
// GoPartDetection.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_PART_DETECTION_H
#define GO_SDK_NET_PART_DETECTION_H

#include <GoSdk/GoPartDetection.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoSensor.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents the part detection parameters of the surface mode configuration.</summary>
        public ref class GoPartDetection : public KObject
        {
            KDeclareClass(GoPartDetection, GoPartDetection)

            /// <summary>Initializes a new instance of the Go class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoPartDetection(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the Go class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoPartDetection(GoSensor^ sensor)
            {
                ::GoPartDetection handle = kNULL;

                KCheck(::GoPartDetection_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoPartDetection(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoPartDetection(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoPartDetection handle = kNULL;

                KCheck(::GoPartDetection_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Enables part detection.</summary>
            property bool PartDetectionEnabled
            {
                bool get()           { return KToBool(::GoPartDetection_PartDetectionEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoPartDetection_EnablePartDetection(Handle, enable)); }
            }

            /// <summary>Returns the state of whether or not the user specified part detection value is used.</summary>
            property bool EnablePartDetectionUsed
            {
                bool get()           { return KToBool(::GoPartDetection_EnablePartDetectionUsed(Handle)); }
            }

            /// <summary>Gets the current part detection system state.</summary>
            property bool PartDetectionEnabledSystemValue
            {
                bool get()           { return KToBool(::GoPartDetection_PartDetectionEnabledSystemValue(Handle)); }
            }

            /// <summary>Gets the part detection minimum threshold value.</summary>
            property k64f ThresholdLimitMin
            {
                k64f get()           { return ::GoPartDetection_ThresholdLimitMin(Handle); }
            }

            /// <summary>Gets the part detection maximum threshold value.</summary>
            property k64f ThresholdLimitMax
            {
                k64f get()           { return ::GoPartDetection_ThresholdLimitMax(Handle); }
            }

            /// <summary>The part detection threshold value.</summary>
            property k64f Threshold
            {
                k64f get()           { return ::GoPartDetection_Threshold(Handle); }
                void set(k64f height)  { KCheck(::GoPartDetection_SetThreshold(Handle, height)); }
            }

            /// <summary>The part detection height threshold direction.</summary>
            property GoPartHeightThresholdDirection ThresholdDirection
            {
                GoPartHeightThresholdDirection get()           { return (GoPartHeightThresholdDirection) ::GoPartDetection_ThresholdDirection(Handle); }
                void set(GoPartHeightThresholdDirection direction)  { KCheck(::GoPartDetection_SetThresholdDirection(Handle, direction)); }
            }

            /// <summary>The part detection flag for including one sided data when in opposite layout.</summary>
            property bool IncludeSinglePoints
            {
                bool get()           { return KToBool(::GoPartDetection_IncludeSinglePoints(Handle)); }
                void set(bool flag)  { KCheck(::GoPartDetection_SetIncludeSinglePoints(Handle, flag)); }
            }

            /// <summary>Returns the state of whether or not one sided data flag is used.</summary>
            property bool IncludeSinglePointsUsed
            {
                bool get() { return KToBool(::GoPartDetection_IncludeSinglePointsUsed(Handle)); }
            }

            /// <summary>The part detection frame of reference.</summary>
            property GoPartFrameOfReference FrameOfReference
            {
                GoPartFrameOfReference get()           { return (GoPartFrameOfReference) ::GoPartDetection_FrameOfReference(Handle); }
                void set(GoPartFrameOfReference frameOfRef)  { KCheck(::GoPartDetection_SetFrameOfReference(Handle, frameOfRef)); }
            }

            /// <summary>Returns the state of whether or not the user specified frame of reference value is used.</summary>
            property bool FrameOfReferenceUsed
            {
                bool get()           { return KToBool(::GoPartDetection_FrameOfReferenceUsed(Handle)); }
            }

            /// <summary>Gets the part detection gap width minimum limit value.</summary>
            property k64f GapWidthLimitMin
            {
                k64f get()           { return ::GoPartDetection_GapWidthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection gap width maximum limit value.</summary>
            property k64f GapWidthLimitMax
            {
                k64f get()           { return ::GoPartDetection_GapWidthLimitMax(Handle); }
            }

            /// <summary>The part detection gap width.</summary>
            property k64f GapWidth
            {
                k64f get()           { return ::GoPartDetection_GapWidth(Handle); }
                void set(k64f width)  { KCheck(::GoPartDetection_SetGapWidth(Handle, width)); }
            }

            /// <summary>Returns the state of whether or not the user specified gap width value is used.</summary>
            property bool GapWidthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_GapWidthUsed(Handle)); }
            }

            /// <summary>Gets the part detection gap length minimum limit value.</summary>
            property k64f GapLengthLimitMin
            {
                k64f get()           { return ::GoPartDetection_GapLengthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection gap length maximum limit value.</summary>
            property k64f GapLengthLimitMax
            {
                k64f get()           { return ::GoPartDetection_GapLengthLimitMax(Handle); }
            }

            /// <summary>The part detection gap length.</summary>
            property k64f GapLength
            {
                k64f get()           { return ::GoPartDetection_GapLength(Handle); }
                void set(k64f length)  { KCheck(::GoPartDetection_SetGapLength(Handle, length)); }
            }

            /// <summary>Returns the state of whether or not the user specified gap length value is used.</summary>
            property bool GapLengthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_GapLengthUsed(Handle)); }
            }

            /// <summary>Gets the part detection padding width minimum limit value.</summary>
            property k64f PaddingWidthLimitMin
            {
                k64f get()           { return ::GoPartDetection_PaddingWidthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection padding width maximum limit value.</summary>
            property k64f PaddingWidthLimitMax
            {
                k64f get()           { return ::GoPartDetection_PaddingWidthLimitMax(Handle); }
            }

            /// <summary>The part detection padding width.</summary>
            property k64f PaddingWidth
            {
                k64f get()           { return ::GoPartDetection_PaddingWidth(Handle); }
                void set(k64f padding)  { KCheck(::GoPartDetection_SetPaddingWidth(Handle, padding)); }
            }

            /// <summary>Returns the state of whether or not the user specified padding width value is used.</summary>
            property bool PaddingWidthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_PaddingWidthUsed(Handle)); }
            }
            
            /// <summary>Gets the part detection padding width minimum limit value.</summary>
            property k64f PaddingLengthLimitMin
            {
                k64f get()           { return ::GoPartDetection_PaddingLengthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection padding width maximum limit value.</summary>
            property k64f PaddingLengthLimitMax
            {
                k64f get()           { return ::GoPartDetection_PaddingLengthLimitMax(Handle); }
            }

            /// <summary>The part detection padding width.</summary>
            property k64f PaddingLength
            {
                k64f get()           { return ::GoPartDetection_PaddingLength(Handle); }
                void set(k64f padding)  { KCheck(::GoPartDetection_SetPaddingLength(Handle, padding)); }
            }

            /// <summary>Returns the state of whether or not the user specified padding width value is used.</summary>
            property bool PaddingLengthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_PaddingLengthUsed(Handle)); }
            }

            /// <summary>Gets the part detection minimum length minimum limit value.</summary>
            property k64f MinLengthLimitMin
            {
                k64f get()           { return ::GoPartDetection_MinLengthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection minimum length maximum limit value.</summary>
            property k64f MinLengthLimitMax
            {
                k64f get()           { return ::GoPartDetection_MinLengthLimitMax(Handle); }
            }

            /// <summary>The part detection minimum length.</summary>
            property k64f MinLength
            {
                k64f get()           { return ::GoPartDetection_MinLength(Handle); }
                void set(k64f length)  { KCheck(::GoPartDetection_SetMinLength(Handle, length)); }
            }

            /// <summary>Returns the state of whether or not the user specified gap minimum length value is used.</summary>
            property bool MinLengthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_MinLengthUsed(Handle)); }
            }
            
            /// <summary>Gets the part detection maximum length minimum limit value.</summary>
            property k64f MaxLengthLimitMin
            {
                k64f get()           { return ::GoPartDetection_MaxLengthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection maximum length maximum limit value.</summary>
            property k64f MaxLengthLimitMax
            {
                k64f get()           { return ::GoPartDetection_MaxLengthLimitMax(Handle); }
            }

            /// <summary>The part detection gap maximum length.</summary>
            property k64f MaxLength
            {
                k64f get()           { return ::GoPartDetection_MaxLength(Handle); }
                void set(k64f length)  { KCheck(::GoPartDetection_SetMaxLength(Handle, length)); }
            }

            /// <summary>Returns the state of whether or not the user specified maximum length value is used.</summary>
            property bool MaxLengthUsed
            {
                bool get()           { return KToBool(::GoPartDetection_MaxLengthUsed(Handle)); }
            }

            /// <summary>Gets the part detection gap minimum area minimum limit value.</summary>
            property k64f MinAreaLimitMin
            {
                k64f get()           { return ::GoPartDetection_MinAreaLimitMin(Handle); }
            }

            /// <summary>Gets the part detection minimum area maximum limit value.</summary>
            property k64f MinAreaLimitMax
            {
                k64f get()           { return ::GoPartDetection_MinAreaLimitMax(Handle); }
            }

            /// <summary>The part detection gap minimum area.</summary>
            property k64f MinArea
            {
                k64f get()           { return ::GoPartDetection_MinArea(Handle); }
                void set(k64f area)  { KCheck(::GoPartDetection_SetMinArea(Handle, area)); }
            }

            /// <summary>Returns the state of whether or not the user specified gap minimum area value is used.</summary>
            property bool GapMinAreaUsed
            {
                bool get()           { return KToBool(::GoPartDetection_MinAreaUsed(Handle)); }
            }

            /// <summary>Returns the state of whether or not the user specified edge filtering configuration is used.</summary>
            property bool EdgeFilterUsed
            {
                bool get()           { return KToBool(::GoPartDetection_EdgeFilterUsed(Handle)); }
            }

            /// <summary>Enables or disables edge filtering.</summary>
            property bool EdgeFilterEnabled
            {
                bool get()           { return KToBool(::GoPartDetection_EdgeFilterEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoPartDetection_EnableEdgeFilter(Handle, enable)); }
            }

            /// <summary>Gets the part detection edge filtering width minimum limit value.</summary>
            property k64f EdgeFilterWidthLimitMin
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterWidthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection edge filtering width maximum limit value.</summary>
            property k64f EdgeFilterWidthLimitMax
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterWidthLimitMax(Handle); }
            }

            /// <summary>The part detection edge filtering width value.</summary>
            property k64f EdgeFilterWidth
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterWidth(Handle); }
                void set(k64f value)  { KCheck(::GoPartDetection_SetEdgeFilterWidth(Handle, value)); }
            }

            /// <summary>Gets the part detection edge filtering length minimum limit value.</summary>
            property k64f EdgeFilterLengthLimitMin
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterLengthLimitMin(Handle); }
            }

            /// <summary>Gets the part detection edge filtering length maximum limit value.</summary>
            property k64f EdgeFilterLengthLimitMax
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterLengthLimitMax(Handle); }
            }

            /// <summary>The part detection edge filtering length value.</summary>
            property k64f EdgeFilterLength
            {
                k64f get()           { return ::GoPartDetection_EdgeFilterLength(Handle); }
                void set(k64f value)  { KCheck(::GoPartDetection_SetEdgeFilterLength(Handle, value)); }
            }

            /// <summary>Gets/Sets the part detection edge filtering anterior preservation.</summary>
            property bool EdgeFilterAnteriorPreservation
            {
                bool get() { return KToBool(::GoPartDetection_EdgeFilterAnteriorPreservationEnabled(Handle)); }
                void set(bool value) { KCheck(::GoPartDetection_EnableEdgeFilterAnteriorPreservation(Handle, value)); }
            }
        };
    }
}

#endif























