// 
// GoPartMatching.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_PART_MATCHING_H
#define GO_SDK_NET_PART_MATCHING_H

#include <GoSdk/GoPartMatching.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents the part matching parameters of the surface mode configuration.</summary>
        public ref class GoPartMatching : public KObject
        {
            KDeclareClass(GoPartMatching, GoPartMatching)

            /// <summary>Initializes a new instance of the GoPartMatching class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoPartMatching(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoPartMatching class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoPartMatching(GoSensor^ sensor)
            {
               ::GoPartMatching handle = kNULL;

                KCheck(::GoPartMatching_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="Go(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoPartMatching(GoSensor^ sensor, KAlloc^ allocator)
            {
               ::GoPartMatching handle = kNULL;

                KCheck(::GoPartMatching_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Enables part matching.</summary>
            property bool PartMatchingEnabled
            {
                bool get()           { return KToBool(::GoPartMatching_PartMatchingEnabled(Handle)); }
                void set(bool enable)  { KCheck(::GoPartMatching_EnablePartMatching(Handle, enable)); }
            }

            /// <summary>The state of whether or not the user specified part matching value is used.</summary>
            property bool EnablePartMatchingUsed
            {
                bool get()           { return KToBool(::GoPartMatching_EnablePartMatchingUsed(Handle)); }
            }

            /// <summary>The part matching algorithm.</summary>
            property GoPartMatchAlgorithm Algorithm
            {
                GoPartMatchAlgorithm get()           { return (GoPartMatchAlgorithm) ::GoParthMatching_Algorithm(Handle); }
                void set(GoPartMatchAlgorithm algorithm)  { KCheck(::GoPartMatching_SetAlgorithm(Handle, algorithm)); }
            }

            /// <summary>The edge model name.</summary>
            property String^ EdgeModelName
            {
                String^ get()           { return KToString(::GoPartMatching_EdgeModelName(Handle)); }
                void set(String^ name) 
                { 
                    KString str(name);
                    
                    KCheck(::GoPartMatching_SetEdgeModelName(Handle, str.CharPtr));
                }
            }

            /// <summary>The edge matching decision minimum quality value.</summary>
            property k64f EdgeQualityDecisionMin
            {
                k64f get()           { return ::GoPartMatching_EdgeQualityDecisionMin(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEdgeQualityDecisionMin(Handle, value)); }
            }

            /// <summary>The ellipse match major decision maximum value.</summary>
            property k64f EllipseMajorMax
            {
                k64f get()           { return ::GoPartMatching_EllipseMajorMax(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEllipseMajorMax(Handle, value)); }
            }

            /// <summary>The ellipse match major decision minimum value.</summary>
            property k64f EllipseMajorMin
            {
                k64f get()           { return ::GoPartMatching_EllipseMajorMin(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEllipseMajorMin(Handle, value)); }
            }

            /// <summary>The ellipse match minor decision maximum value.</summary>
            property k64f EllipseMinorMax
            {
                k64f get()           { return ::GoPartMatching_EllipseMinorMax(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEllipseMinorMax(Handle, value)); }
            }

            /// <summary>The ellipse match minor decision minimum value.</summary>
            property k64f EllipseMinorMin
            {
                k64f get()           { return ::GoPartMatching_EllipseMinorMin(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEllipseMinorMin(Handle, value)); }
            }

            /// <summary>The ellipse match Z angle value.</summary>
            property k64f EllipseZAngle
            {
                k64f get()           { return ::GoPartMatching_EllipseZAngle(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetEllipseZAngle(Handle, value)); }
            }

            /// <summary>The ellipse match decision asymmetry detection type.</summary>
            property GoEllipseAsymmetryType EllipseAsymmetryDetectionType
            {
                GoEllipseAsymmetryType get()           { return (GoEllipseAsymmetryType) ::GoPartMatching_EllipseAsymmetryDetectionType(Handle); }
                void set(GoEllipseAsymmetryType value)  { KCheck(::GoPartMatching_SetEllipseAsymmetryDetectionType(Handle, value)); }
            }

            /// <summary>The bounding box match decision maximum width value.</summary>
            property k64f BoundingBoxWidthMax
            {
                k64f get()           { return ::GoPartMatching_BoundingBoxWidthMax(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetBoundingBoxWidthMax(Handle, value)); }
            }

            /// <summary>The bounding box match decision minimum width value.</summary>
            property k64f BoundingBoxWidthMin
            {
                k64f get()           { return ::GoPartMatching_BoundingBoxWidthMin(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetBoundingBoxWidthMin(Handle, value)); }
            }

            /// <summary>The bounding box match decision maximum length value.</summary>
            property k64f BoundingBoxLengthMax
            {
                k64f get()           { return ::GoPartMatching_BoundingBoxLengthMax(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetBoundingBoxLengthMax(Handle, value)); }
            }

            /// <summary>The bounding box match decision minimum length value.</summary>
            property k64f BoundingBoxLengthMin
            {
                k64f get()           { return ::GoPartMatching_BoundingBoxLengthMin(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetBoundingBoxLengthMin(Handle, value)); }
            }

            /// <summary>The bounding box match decision Z angle value.</summary>
            property k64f BoundingBoxZAngle
            {
                k64f get()           { return ::GoPartMatching_BoundingBoxZAngle(Handle); }
                void set(k64f value)  { KCheck(::GoPartMatching_SetBoundingBoxZAngle(Handle, value)); }
            }

            /// <summary>The bounding box match decision asymmetry detection type.</summary>
            property GoBoxAsymmetryType BoundingBoxAsymmetryDetectionType
            {
                GoBoxAsymmetryType get()           { return (GoBoxAsymmetryType) ::GoPartMatching_BoundingBoxAsymmetryDetectionType(Handle); }
                void set(GoBoxAsymmetryType value)  { KCheck(::GoPartMatching_SetBoundingBoxAsymmetryDetectionType(Handle, value)); }
            }
        };
    }
}

#endif























