// 
// GoPartModel.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_PART_MODEL_H
#define GO_SDK_NET_PART_MODEL_H

#include <GoSdk/GoPartModel.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a part model edge configuration.</summary>
        public ref class GoPartModelEdge : public KObject
        {
            KDeclareClass(GoPartModelEdge, GoPartModelEdge)

            /// <summary>Initializes a new instance of the GoPartModelEdge class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoPartModelEdge(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoPartModelEdge class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            /// <param name="parentModel">The part model parent.</param>
            GoPartModelEdge(KObject^ parentModel, GoSensor^ sensor)
            {
                ::GoPartModelEdge handle = kNULL;

                KCheck(::GoPartModelEdge_Construct(&handle, KToHandle(parentModel), KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="Go(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoPartModelEdge(KObject^ parentModel, GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoPartModelEdge handle = kNULL;

                KCheck(::GoPartModelEdge_Construct(&handle, KToHandle(parentModel), KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The edge model image type used at the time of model creation.</summary>
            property GoImageType ImageType
            {
                GoImageType get()           { return (GoImageType) ::GoPartModelEdge_ImageType(Handle); }
            }

            /// <summary>The edge model image data source used at the time of model creation.</summary>
            property GoDataSource ImageSource
            {
                GoDataSource get()           { return (GoDataSource) ::GoPartModelEdge_ImageSource(Handle); }
            }

            /// <summary>The count of removed edge model points.</summary>
            property k64s RemovedPointsLength
            {
                k64s get()           { return (kSize) ::GoPartModelEdge_RemovedPointsLength(Handle); }
            }
        };
        
        /// <summary>Represents a part model configuration.</summary>
        public ref class GoPartModel : public KObject
        {
            KDeclareClass(GoPartModel, GoPartModel)

            /// <summary>Initializes a new instance of the GoPartModel class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoPartModel(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoPartModel class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            /// <param name="name">The name of the part model.</param>
            GoPartModel(GoSensor^ sensor, String^ name)
            {
                ::GoPartModel handle = kNULL;
                KString str(name);

                KCheck(::GoPartModel_Construct(&handle, KToHandle(sensor), str.CharPtr, kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="Go(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoPartModel(GoSensor^ sensor, String^ name, KAlloc^ allocator)
            {
                ::GoPartModel handle = kNULL;
                KString str(name);

                KCheck(::GoPartModel_Construct(&handle, KToHandle(sensor), str.CharPtr, KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The name of the part model.</summary>
            property String^ Name
            {
                String^ get()           { return KToString(::GoPartModel_Name(Handle)); }
            }

            /// <summary>The number of edges for the given part model.</summary>
            property k64s EdgeCount
            {
                k64s get()           { return (kSize) ::GoPartModel_EdgeCount(Handle); }
            }

            /// <summary>Gets the edge for the given index.</summary>
            /// <param name="index">The index of the part model edge list to access.</param>
            /// <returns>The edge at the given index.</returns>
            GoPartModelEdge^ GetEdge(k64s index)
            {
                return KToObject<GoPartModelEdge^>(::GoPartModel_EdgeAt(Handle, (kSize)index));
            }

            /// <summary>The edge sensitivity used at the time of model creation.</summary>
            property k64f EdgeSensitivity
            {
                k64f get()           { return ::GoPartModel_EdgeSensitivity(Handle); }
            }

            /// <summary>The transformed data region X-component value.</summary>
            property k64f TransformedDataRegionX
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionX(Handle); }
            }

            /// <summary>The transformed data region Y-component value.</summary>
            property k64f TransformedDataRegionY
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionY(Handle); }
            }

            /// <summary>The transformed data region Z-component value.</summary>
            property k64f TransformedDataRegionZ
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionZ(Handle); }
            }

            /// <summary>The transformed data region width value.</summary>
            property k64f TransformedDataRegionWidth
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionWidth(Handle); }
            }

            /// <summary>The transformed data region length value.</summary>
            property k64f TransformedDataRegionLength
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionLength(Handle); }
            }

            /// <summary>The transformed data region height value.</summary>
            property k64f TransformedDataRegionHeight
            {
                k64f get()           { return ::GoPartModel_TransformedDataRegionHeight(Handle); }
            }

            /// <summary>The Z angle for the given part model.</summary>
            property k64f ZAngle
            {
                k64f get()           { return ::GoPartModel_ZAngle(Handle); }
                void set(k64f value)  { KCheck(::GoPartModel_SetZAngle(Handle, value)); }
            }

            /// <summary>The image type for the given part model.</summary>
            property GoImageType ImageType
            {
                GoImageType get()           { return (GoImageType) ::GoPartModel_ImageType(Handle); }
                void set(GoImageType value)  { KCheck(::GoPartModel_SetImageType(Handle, value)); }
            }

            /// <summary>The image type option count.</summary>
            property k64s ImageTypeOptionCount
            {
                k64s get()           { return (k64s) ::GoPartModel_ImageTypeOptionCount(Handle); }
            }

            /// <summary>Gets the image type option at the given index.</summary>
            /// <param name="index">The image type option index to access.</param>
            /// <returns>The image type option.</returns>
            GoImageType GetImageTypeOption(k64s index)
            {
                return (GoImageType) ::GoPartModel_ImageTypeOptionAt(Handle, (kSize)index);
            }

            /// <summary>The target edge sensitivity.</summary>
            property k64f TargetEdgeSensitivity
            {
                k64f get()           { return ::GoPartModel_TargetEdgeSensitivity(Handle); }
                void set(k64f value)  { KCheck(::GoPartModel_SetTargetEdgeSensitivity(Handle, value)); }
            }
        };
    }
}

#endif
