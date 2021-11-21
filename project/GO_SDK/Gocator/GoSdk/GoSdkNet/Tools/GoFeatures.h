// 
// GoFeaturess.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_FEATURES_H
#define GO_SDK_NET_FEATURES_H

#include <GoSdkNet/Tools/GoFeature.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Tools
        {
            // <summary> Represents an external feature for GDK and custom tools </summary>
            public ref class GoExtFeature : public GoFeature
            {
                KDeclareClass(GoExtFeature, GoExtFeature)

                    /// <summary>Default GoFeature constructor.</summary>
                    GoExtFeature() {}

                    /// <summary>Initializes a new instance of the GoFeature class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoExtFeature(IntPtr handle)
                        : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoundingBoxCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceBoundingBoxCenterPoint, GoSurfaceBoundingBoxCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceBoundingBoxCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceBoundingBoxCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents an axis line for Surface Bounding Box tool.</summary>
            public ref class GoSurfaceBoundingBoxAxisLine : public GoFeature
            {
                KDeclareClass(GoSurfaceBoundingBoxAxisLine, GoSurfaceBoundingBoxAxisLine)

                    /// <summary>Initializes a new instance of the GoSurfaceBoundingBoxCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceBoundingBoxAxisLine(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Countersunk Hole tool.</summary>
            public ref class GoSurfaceCountersunkHoleCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceCountersunkHoleCenterPoint, GoSurfaceCountersunkHoleCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceCountersunkHoleCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceCountersunkHoleCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Dimension tool.</summary>
            public ref class GoSurfaceDimensionCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceDimensionCenterPoint, GoSurfaceDimensionCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceDimensionCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceDimensionCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceEllipseCenterPoint, GoSurfaceEllipseCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceEllipseCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceEllipseCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a major axis line for Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseMajorAxisLine : public GoFeature
            {
                KDeclareClass(GoSurfaceEllipseMajorAxisLine, GoSurfaceEllipseMajorAxisLine)

                    /// <summary>Initializes a new instance of the GoSurfaceEllipseMajorAxisLine class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceEllipseMajorAxisLine(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a minor axis line for Surface Ellipse tool.</summary>
            public ref class GoSurfaceEllipseMinorAxisLine : public GoFeature
            {
                KDeclareClass(GoSurfaceEllipseMinorAxisLine, GoSurfaceEllipseMinorAxisLine)

                    /// <summary>Initializes a new instance of the GoSurfaceEllipseMinorAxisLine class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceEllipseMinorAxisLine(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Hole tool.</summary>
            public ref class GoSurfaceHoleCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceHoleCenterPoint, GoSurfaceHoleCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceHoleCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceHoleCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Surface Opening tool.</summary>
            public ref class GoSurfaceOpeningCenterPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceOpeningCenterPoint, GoSurfaceOpeningCenterPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceOpeningCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceOpeningCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a plane for the Surface Plane tool.</summary>
            public ref class GoSurfacePlanePlane : public GoFeature
            {
                KDeclareClass(GoSurfacePlanePlane, GoSurfacePlanePlane)

                    /// <summary>Initializes a new instance of the GoSurfacePlanePlane class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfacePlanePlane(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a point for Surface Position tool.</summary>
            public ref class GoSurfacePositionPoint : public GoFeature
            {
                KDeclareClass(GoSurfacePositionPoint, GoSurfacePositionPoint)

                    /// <summary>Initializes a new instance of the GoSurfacePositionPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfacePositionPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a tip point for Surface Stud tool.</summary>
            public ref class GoSurfaceStudTipPoint : public GoFeature
            {
                KDeclareClass(GoSurfaceStudTipPoint, GoSurfaceStudTipPoint)

                    /// <summary>Initializes a new instance of the GoSurfaceStudTipPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceStudTipPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a base point for Surface Stud tool.</summary>
            public ref class GoSurfaceStudBasePoint : public GoFeature
            {
                KDeclareClass(GoSurfaceStudBasePoint, GoSurfaceStudBasePoint)

                    /// <summary>Initializes a new instance of the GoSurfaceStudBasePoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoSurfaceStudBasePoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a point for Profile Position tool.</summary>
            public ref class GoProfilePositionPoint : public GoFeature
            {
                KDeclareClass(GoProfilePositionPoint, GoProfilePositionPoint)

                    /// <summary>Initializes a new instance of the GoProfilePositionPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePositionPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a line for Profile Line tool.</summary>
            public ref class GoProfileLineLine : public GoFeature
            {
                KDeclareClass(GoProfileLineLine, GoProfileLineLine)

                    /// <summary>Initializes a new instance of the GoProfileLineLine class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineLine(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a point of minimum error for Profile Line tool.</summary>
            public ref class GoProfileLineMinErrorPoint : public GoFeature
            {
                KDeclareClass(GoProfileLineMinErrorPoint, GoProfileLineMinErrorPoint)

                    /// <summary>Initializes a new instance of the GoProfileLineMinErrorPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMinErrorPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a point of maximum error for Profile Line tool.</summary>
            public ref class GoProfileLineMaxErrorPoint : public GoFeature
            {
                KDeclareClass(GoProfileLineMaxErrorPoint, GoProfileLineMaxErrorPoint)

                    /// <summary>Initializes a new instance of the GoProfileLineMaxErrorPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileLineMaxErrorPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents an intersect point for Profile Intersect tool.</summary>
            public ref class GoProfileIntersectIntersectPoint : public GoFeature
            {
                KDeclareClass(GoProfileIntersectIntersectPoint, GoProfileIntersectIntersectPoint)

                    /// <summary>Initializes a new instance of the GoProfileIntersectIntersectPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileIntersectIntersectPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a line for Profile Intersect tool.</summary>
            public ref class GoProfileIntersectLineFeature : public GoFeature
            {
                KDeclareClass(GoProfileIntersectLineFeature, GoProfileIntersectLine)

                    /// <summary>Initializes a new instance of the GoProfileIntersectLineFeature class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileIntersectLineFeature(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a base line for Profile Intersect tool.</summary>
            public ref class GoProfileIntersectBaseLineFeature : public GoFeature
            {
                KDeclareClass(GoProfileIntersectBaseLineFeature, GoProfileIntersectBaseLine)

                    /// <summary>Initializes a new instance of the GoProfileIntersectBaseLineFeature class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileIntersectBaseLineFeature(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a corner point for Profile Bounding Box tool.</summary>
            public ref class GoProfileBoundingBoxCornerPoint : public GoFeature
            {
                KDeclareClass(GoProfileBoundingBoxCornerPoint, GoProfileBoundingBoxCornerPoint)

                    /// <summary>Initializes a new instance of the GoProfileBoundingBoxCornerPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileBoundingBoxCornerPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Profile Bounding Box tool.</summary>
            public ref class GoProfileBoundingBoxCenterPoint : public GoFeature
            {
                KDeclareClass(GoProfileBoundingBoxCenterPoint, GoProfileBoundingBoxCenterPoint)

                    /// <summary>Initializes a new instance of the GoProfileBoundingBoxCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileBoundingBoxCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point of Profile Area tool.</summary>
            public ref class GoProfileAreaCenterPoint : public GoFeature
            {
                KDeclareClass(GoProfileAreaCenterPoint, GoProfileAreaCenterPoint)

                    /// <summary>Initializes a new instance of the GoProfileAreaCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileAreaCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Profile Circle tool.</summary>
            public ref class GoProfileCircleCenterPoint : public GoFeature
            {
                KDeclareClass(GoProfileCircleCenterPoint, GoProfileCircleCenterPoint)

                    /// <summary>Initializes a new instance of the GoProfileCircleCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileCircleCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a center point for Profile Dimension tool.</summary>
            public ref class GoProfileDimensionCenterPoint : public GoFeature
            {
                KDeclareClass(GoProfileDimensionCenterPoint, GoProfileDimensionCenterPoint)

                    /// <summary>Initializes a new instance of the GoProfileDimensionCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileDimensionCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a left gap point for Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftGapPoint : public GoFeature
            {
                KDeclareClass(GoProfilePanelLeftGapPoint, GoProfilePanelLeftGapPoint)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftGapPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftGapPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a left flush point for Profile Panel tool.</summary>
            public ref class GoProfilePanelLeftFlushPoint : public GoFeature
            {
                KDeclareClass(GoProfilePanelLeftFlushPoint, GoProfilePanelLeftFlushPoint)

                    /// <summary>Initializes a new instance of the GoProfilePanelLeftFlushPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelLeftFlushPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a right gap point for Profile Panel tool.</summary>
            public ref class GoProfilePanelRightGapPoint : public GoFeature
            {
                KDeclareClass(GoProfilePanelRightGapPoint, GoProfilePanelRightGapPoint)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightGapPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightGapPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a right flush point for Profile Panel tool.</summary>
            public ref class GoProfilePanelRightFlushPoint : public GoFeature
            {
                KDeclareClass(GoProfilePanelRightFlushPoint, GoProfilePanelRightFlushPoint)

                    /// <summary>Initializes a new instance of the GoProfilePanelRightFlushPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfilePanelRightFlushPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a corner point for Profile Round tool.</summary>
            public ref class GoProfileRoundCornerPoint : public GoFeature
            {
                KDeclareClass(GoProfileRoundCornerPoint, GoProfileRoundCornerPoint)

                    /// <summary>Initializes a new instance of the GoProfileRoundCornerPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileRoundCornerPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a corner edge point for Profile Round tool.</summary>
            public ref class GoProfileRoundCornerEdgePoint : public GoFeature
            {
                KDeclareClass(GoProfileRoundCornerEdgePoint, GoProfileRoundCornerEdgePoint)

                    /// <summary>Initializes a new instance of the GoProfileRoundCornerEdgePoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileRoundCornerEdgePoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
            // <summary>Represents a corner center point for Profile Round tool.</summary>
            public ref class GoProfileRoundCornerCenterPoint : public GoFeature
            {
                KDeclareClass(GoProfileRoundCornerCenterPoint, GoProfileRoundCornerCenterPoint)

                    /// <summary>Initializes a new instance of the GoProfileRoundCornerCenterPoint class with the specified Zen object handle.</summary>
                    /// <param name="handle">Zen object handle.</param>
                    GoProfileRoundCornerCenterPoint(IntPtr handle)
                    : GoFeature(handle)
                {}
            };
        }
    }
}

#endif
