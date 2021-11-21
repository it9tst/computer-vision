//
// GoDataTypes.h
//
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
//
#ifndef GO_SDK_NET_DATA_TYPES_H
#define GO_SDK_NET_DATA_TYPES_H

#include <GoSdk/Messages/GoDataTypes.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        namespace Messages
        {
            /// <summary>FireSync distributed control namespace.</summary>
            [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
            public ref class NamespaceDoc { };

            /// <summary>Represents an acquisition stamp.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoStamp))]
            public value struct GoStamp
            {
                KDeclareStruct(GoStamp, GoStamp)

                /// <summary>Frame index (counts up from zero).</summary>
                [FieldOffset(offsetof(::GoStamp, frameIndex))]
                k64u FrameIndex;

                /// <summary>Capture time (approximates nanoseconds; actual time = timestamp / 1.024).</summary>
                [FieldOffset(offsetof(::GoStamp, timestamp))]
                k64u Timestamp;

                /// <summary>Capture position (encoder ticks).</summary>
                [FieldOffset(offsetof(::GoStamp, encoder))]
                k64s Encoder;

                /// <summary>Encoder value latched at z-index mark (encoder ticks).</summary>
                [FieldOffset(offsetof(::GoStamp, encoderAtZ))]
                k64s EncoderAtZ;

                /// <summary>Bit mask containing frame information.</summary>
                ///
                /// <remarks>
                /// - Bit 0: Represents sensor digital input state.
                /// - Bit 4: Represents master digital input state.
                /// - Bits 8 and 9: Represents inter-frame digital pulse trigger (master digital input if a master is connected,
                ///   otherwise sensor digital input). Value is cleared after each frame and saturates at 3.
                /// </remarks>
                ///
                [FieldOffset(offsetof(::GoStamp, status))]
                k64u Status;

                /// <summary>Source device ID.</summary>
                [FieldOffset(offsetof(::GoStamp, id))]
                k32u Id;

                /// <summary>Reserved.</summary>
                [FieldOffset(offsetof(::GoStamp, reserved32u))]
                k32u Reserved32u;

                /// <summary>Reserved.</summary>
                [FieldOffset(offsetof(::GoStamp, reserved64u))]
                k64u Reserved64u;

                /// <summary>Ptp time corresponding to Firesync time.</summary>
                [FieldOffset(offsetof(::GoStamp, ptpTime))]
                k64u PtpTimestamp;
            };

            /// <summary>Represents an 3d transformation matrix.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::Go3dTransform64f))]
            public value struct Go3dTransform64f
            {
                KDeclareStruct(Go3dTransform64f, Go3dTransform64f)

                /// <summary>xx.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, xx))]
                k64f xx;

                /// <summary>xy.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, xy))]
                k64f xy;

                /// <summary>xz.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, xz))]
                k64f xz;

                /// <summary>xt.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, xt))]
                k64f xt;

                /// <summary>yx.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, yx))]
                k64f yx;

                /// <summary>yy.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, yy))]
                k64f yy;

                /// <summary>yz.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, yz))]
                k64f yz;

                /// <summary>yt.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, yt))]
                k64f yt;

                /// <summary>zx.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, zx))]
                k64f zx;

                /// <summary>zy.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, zy))]
                k64f zy;

                /// <summary>zz.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, zz))]
                k64f zz;

                /// <summary>zt.</summary>
                [FieldOffset(offsetof(::Go3dTransform64f, zt))]
                k64f zt;

                String^ ToString() override
                {
                    return "{" + xx + "," + xy + "," + xz + "," + xt + "}, {" +
                                 yx + "," + yy + "," + yz + "," + yt + "}, {" +
                                 zx + "," + zy + "," + zz + "," + zt + "}";
                }
            };

            /// <summary>Represents a base message sourced from the data channel.</summary>
            public ref class GoDataMsg abstract : public KObject
            {
                KDeclareClass(GoDataMsg, GoDataMsg)

            public:
                /// <summary>Default GoDataMsg constructor.</summary>
                GoDataMsg() {}

                /// <summary>Initializes a new instance of the GoDataMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoDataMsg(IntPtr handle)
                    : KObject(handle)
                {}

                /// <summary>The message type for a data channel message given in a GoDataSet.</summary>
                property GoDataMessageType MessageType
                {
                    GoDataMessageType get() { return (GoDataMessageType) ::GoDataMsg_Type(Handle); }
                }

                property GoDataStep StreamStep
                {
                    GoDataStep get() { return ::GoDataMsg_StreamStep(Handle); }
                }

                property k32s StreamStepId
                {
                    k32s get() { return ::GoDataMsg_StreamStepId(Handle); }
                }
            };

            /// <summary>Represents a message containing a set of acquisition stamps.</summary>
            public ref class GoStampMsg : public GoDataMsg
            {
                KDeclareClass(GoStampMsg, GoStampMsg)

            public:
                /// <summary>Initializes a new instance of the GoStampMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoStampMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoStampMsg class.</summary>
                GoStampMsg()
                {
                    ::GoStampMsg handle = kNULL;

                    KCheck(::GoStampMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoStampMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoStampMsg(KAlloc^ allocator)
                {
                    ::GoStampMsg handle = kNULL;

                    KCheck(::GoStampMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The stamp source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoStampMsg_Source(Handle); }
                }

                /// <summary>Returns the number of stamps contained in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoStampMsg_Count(Handle); }
                }

                /// <summary>Sets the stamp at the specified index.</summary>
                /// <param name="index">Stamp index.</param>
                /// <param name="stamp">Stamp object.</param>
                void Set(k64s index, GoStamp stamp)
                {
                    *(::GoStamp*) ::GoStampMsg_At(Handle, (kSize)index) = stamp.ToNative();
                }

                /// <summary>Gets the stamp at the specified index.</summary>
                /// <param name="index">Stamp index.</param>
                /// <returns>Stamp at index.</returns>
                GoStamp Get(k64s index)
                {
                    return GoStamp(::GoStampMsg_At(Handle, (kSize)index));
                }

                /// <summary>Gets a pointer to the internal data buffer buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoStampMsg_At(Handle, 0)); }
                }
            };


            /// <summary>Represents a data message containing a video image.</summary>
            public ref class GoVideoMsg : public GoDataMsg
            {
                KDeclareClass(GoVideoMsg, GoVideoMsg)

            public:
                /// <summary>Initializes a new instance of the GoVideoMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoVideoMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoVideoMsg class.</summary>
                GoVideoMsg()
                {
                    ::GoVideoMsg handle = kNULL;

                    KCheck(::GoVideoMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoVideoMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoVideoMsg(KAlloc^ allocator)
                {
                    ::GoVideoMsg handle = kNULL;

                    KCheck(::GoVideoMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The video source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoVideoMsg_Source(Handle); }
                }

                /// <summary>Gets the camera index that the video data originates from.</summary>
                property k32s CameraIndex
                {
                    k32s get() { return (k32s) ::GoVideoMsg_CameraIndex(Handle); }
                }

                /// <summary>Gets the image width, in pixels.</summary>
                property k32s Width
                {
                    k32s get() { return (k32s) ::GoVideoMsg_Width(Handle); }
                }

                /// <summary>Gets the image height, in pixels.</summary>
                property k32s Height
                {
                    k32s get() { return (k32s) ::GoVideoMsg_Height(Handle); }
                }

                /// <summary>Gets the data type used to represent an image pixel.</summary>
                property GoPixelType PixelType
                {
                    GoPixelType get() { return (GoPixelType) ::GoVideoMsg_PixelType(Handle); }
                }

                /// <summary>Gets the image pixel size, in bytes.</summary>
                property k32s PixelSize
                {
                    k32s get() { return (k32s) ::GoVideoMsg_PixelSize(Handle); }
                }

                /// <summary>Gets the pixel format descriptor.</summary>
                property KPixelFormat PixelFormat
                {
                    KPixelFormat get() { return (KPixelFormat) ::GoVideoMsg_PixelFormat(Handle); }
                }

                /// <summary>Gets the image color filter array.</summary>
                property KCfa Cfa
                {
                    KCfa get() { return (KCfa) ::GoVideoMsg_Cfa(Handle); }
                }

                /// <summary>Gets the specified pixel value.</summary>
                ///
                /// <typeparam name="T">Type of item to be accessed.</typeparam>
                /// <param name="x">Column index.</param>
                /// <param name="y">Row index.</param>
                /// <returns>Pixel value.</returns>
                generic <typename T>
                T Get(k32s x, k32s y)
                {
                    KCheckArgs(((kSize)x < ::GoVideoMsg_Width(Handle)) && ((kSize)y < ::GoVideoMsg_Height(Handle)));
                    kSize pixelSize = ::GoVideoMsg_PixelSize(Handle);

                    void* item = (kByte*)::GoVideoMsg_RowAt(Handle, (kSize)y) + (kSize)x * pixelSize;
                    T pixel;

                    KCheckArgs(sizeof(T) == pixelSize);

                    kItemCopy(&pixel, item, sizeof(T));

                    return pixel;
                }

                /// <summary>Sets the value of a pixel.</summary>
                ///
                /// <typeparam name="T">Type of item to be modified.</typeparam>
                /// <param name="x">Column index.</param>
                /// <param name="y">Row index.</param>
                /// <param name="pixel">Pixel value to be copied into the image.</param>
                generic <typename T>
                void Set(k64s x, k64s y, T pixel)
                {
                    kSize pixelSize = ::GoVideoMsg_PixelSize(Handle);

                    KCheckArgs(sizeof(T) == pixelSize);
                    KCheckArgs(((kSize)x < ::GoVideoMsg_Width(Handle)) && ((kSize)y < ::GoVideoMsg_Height(Handle)));

                    void* item = (kByte*)::GoVideoMsg_RowAt(Handle, (kSize)y) + (kSize)x * pixelSize;

                    kItemCopy(item, &pixel, sizeof(T));
                }

                /// <summary>Gets a pointer to a row within the image.</summary>
                IntPtr RowAt(k32s y)
                {
                    return IntPtr(::GoVideoMsg_RowAt(Handle, (kSize)y));
                }

                /// <summary>Gets the exposure index.</summary>
                property k32s ExposureIndex
                {
                    k32s get() { return (k32s) ::GoVideoMsg_ExposureIndex(Handle); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32u Exposure
                {
                    k32u get() { return (k32u) ::GoVideoMsg_Exposure(Handle); }
                }

                /// <summary>Indicates whether the video message data requires horizontal flipping to match up with profile data.</summary>
                property bool IsFlippedX
                {
                    bool get() { return KToBool(::GoVideoMsg_IsFlippedX(Handle)); }
                }

                /// <summary>Indicates whether the video message data requires vertical flipping to match up with profile data.</summary>
                property bool IsFlippedY
                {
                    bool get() { return KToBool(::GoVideoMsg_IsFlippedY(Handle)); }
                }

                /// <summary>Indicates whether the video message data requires transpose to match up with profile data.</summary>
                property bool IsTransposed
                {
                    bool get() { return KToBool(::GoVideoMsg_IsTransposed(Handle)); }
                }
            };

            /// <summary>Represents a data message containing a set of range data.</summary>
            public ref class GoRangeMsg : public GoDataMsg
            {
                KDeclareClass(GoRangeMsg, GoRangeMsg)

            public:
                /// <summary>Initializes a new instance of the GoRangeMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangeMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoRangeMsg class.</summary>
                GoRangeMsg()
                {
                    ::GoRangeMsg handle = kNULL;

                    KCheck(::GoRangeMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoRangeMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoRangeMsg(KAlloc^ allocator)
                {
                    ::GoRangeMsg handle = kNULL;

                    KCheck(::GoRangeMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The range source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoRangeMsg_Source(Handle); }
                }

                /// <summary>Returns the count of range data in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoRangeMsg_Count(Handle); }
                }

                /// <summary>Gets the range z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoRangeMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the range z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoRangeMsg_ZOffset(Handle); }
                }

                /// <summary>Sets the range value at the specified index.</summary>
                /// <param name="index">Range index.</param>
                /// <param name="value">Range value.</param>
                void Set(k64s index, k16s value)
                {
                    *(::GoRangeMsg_At(Handle, (kSize)index)) = value;
                }

                /// <summary>Gets the range value at the specified index.</summary>
                /// <param name="index">Range index.</param>
                /// <returns>Range value at given index.</returns>
                k16s Get(k64s index)
                {
                    return *(::GoRangeMsg_At(Handle, (kSize)index));
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoRangeMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure.</summary>
                property k32u Exposure
                {
                    k32u get() { return (k32u) GoRangeMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a data message containing a set of range intensity data.</summary>
            public ref class GoRangeIntensityMsg : public GoDataMsg
            {
                KDeclareClass(GoRangeIntensityMsg, GoRangeIntensityMsg)

            public:
                /// <summary>Initializes a new instance of the GoRangeIntensityMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoRangeIntensityMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoRangeIntensityMsg class.</summary>
                GoRangeIntensityMsg()
                {
                    ::GoRangeIntensityMsg handle = kNULL;

                    KCheck(::GoRangeIntensityMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoRangeIntensityMsg_Construct()" />
                /// <param name="allocator">Memory allocator</param>
                GoRangeIntensityMsg(KAlloc^ allocator)
                {
                    ::GoRangeIntensityMsg handle = kNULL;

                    KCheck(::GoRangeIntensityMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The range intensity source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoRangeIntensityMsg_Source(Handle); }
                }

                /// <summary>Returns the count of range intensity data in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoRangeIntensityMsg_Count(Handle); }
                }

                /// <summary>Sets the range intensity value at the specified index.</summary>
                /// <param name="index">Range intensity index.</param>
                /// <param name="value">Range intensity value.</param>
                void Set(k64s index, k8u value)
                {
                    *(::GoRangeIntensityMsg_At(Handle, (kSize)index)) = value;
                }

                /// <summary>Gets the range intensity at the specified index.</summary>
                /// <param name="index">Range intensity index.</param>
                /// <returns>Range intensity at given index.</returns>
                k8u Get(k64s index)
                {
                    return *(::GoRangeIntensityMsg_At(Handle, (kSize)index));
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoRangeIntensityMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure.</summary>
                property k32u Exposure
                {
                    k32u get() { return (k32u) GoRangeIntensityMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a data message containing a set of profile arrays.</summary>
            public ref class GoProfilePointCloudMsg : public GoDataMsg
            {
                KDeclareClass(GoProfilePointCloudMsg, GoProfilePointCloudMsg)

            public:
                /// <summary>Initializes a new instance of the GoProfilePointCloudMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfilePointCloudMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfilePointCloudMsg class.</summary>
                GoProfilePointCloudMsg()
                {
                    ::GoProfilePointCloudMsg handle = kNULL;

                    KCheck(::GoProfilePointCloudMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfilePointCloudMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoProfilePointCloudMsg(KAlloc^ allocator)
                {
                    ::GoProfilePointCloudMsg handle = kNULL;

                    KCheck(::GoProfilePointCloudMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the count of profile arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoProfilePointCloudMsg_Count(Handle); }
                }

                /// <summary>Gets the count of ranges in each profile array.</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoProfilePointCloudMsg_Width(Handle); }
                }

                /// <summary> Gets the profile source.</summary>
                property GoDataSource Source 
                { 
                    GoDataSource get() { return (GoDataSource) ::GoProfilePointCloudMsg_Source(Handle); } 
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoProfilePointCloudMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoProfilePointCloudMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoProfilePointCloudMsg_XOffset(Handle); }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoProfilePointCloudMsg_ZOffset(Handle); }
                }

                /// <summary>Sets the point at the specified index for a specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="point">Point object.</param>
                void Set(k64s profileIndex, k64s pointIndex, KPoint16s point)
                {
                    kPoint16s* points = GoProfilePointCloudMsg_At(Handle, (kSize) profileIndex);

                    points[pointIndex].x = point.X;
                    points[pointIndex].y = point.Y;
                }

                /// <summary>Gets the point at the specified index for a specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point at given indices.</returns>
                KPoint16s Get(k64s profileIndex, k64s pointIndex)
                {
                    kPoint16s* points = ::GoProfilePointCloudMsg_At(Handle, (kSize) profileIndex);

                    return KPoint16s(points[pointIndex].x, points[pointIndex].y);
                }

                /// <summary>Sets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="point">Point object.</param>
                void Set(k64s pointIndex, KPoint16s point)
                {
                    return Set(0, pointIndex, point);
                }

                /// <summary>Gets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point at given indices.</returns>
                KPoint16s Get(k64s pointIndex)
                {
                    return Get(0, pointIndex);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoProfilePointCloudMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoProfilePointCloudMsg_Exposure(Handle); }
                }

                /// <summary>Gets the source camera index.</summary>
                property k8u CameraIndex
                {
                    k8u get() { return (k8u) ::GoProfilePointCloudMsg_CameraIndex(Handle); }
                }
            };

            /// <summary>Represents a data message containing a set of re-sampled profile arrays.</summary>
            public ref class GoUniformProfileMsg : public GoDataMsg
            {
                KDeclareClass(GoUniformProfileMsg, GoUniformProfileMsg)

            public:
                /// <summary>Initializes a new instance of the GoUniformProfileMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoUniformProfileMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoUniformProfileMsg class.</summary>
                GoUniformProfileMsg()
                {
                    ::GoUniformProfileMsg handle = kNULL;

                    KCheck(::GoUniformProfileMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoUniformProfileMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoUniformProfileMsg(KAlloc^ allocator)
                {
                    ::GoUniformProfileMsg handle = kNULL;

                    KCheck(::GoUniformProfileMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the count of profile arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoUniformProfileMsg_Count(Handle); }
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoUniformProfileMsg_Source(Handle); }
                }

                /// <summary>Gets the count of ranges in each profile array.</summary>
                property k32s Width
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_Width(Handle); }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_XOffset(Handle); }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_ZOffset(Handle); }
                }

                /// <summary>Sets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s profileIndex, k64s pointIndex, k16s value)
                {
                    k16s* points = GoUniformProfileMsg_At(Handle, (kSize) profileIndex);

                    points[pointIndex] = value;
                }

                /// <summary>Gets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k16s Get(k64s profileIndex, k64s pointIndex)
                {
                    k16s* points = ::GoUniformProfileMsg_At(Handle, (kSize) profileIndex);

                    return points[pointIndex];
                }

                /// <summary>Sets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s pointIndex, k16s value)
                {
                    return Set(0, pointIndex, value);
                }

                /// <summary>Gets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given index.</returns>
                k16s Get(k64s pointIndex)
                {
                    return Get(0, pointIndex);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoUniformProfileMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoUniformProfileMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a data message containing a set of profile intensity arrays.</summary>
            public ref class GoProfileIntensityMsg : public GoDataMsg
            {
                KDeclareClass(GoProfileIntensityMsg, GoProfileIntensityMsg)

            public:
                /// <summary>Initializes a new instance of the GoProfileIntensityMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoProfileIntensityMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoProfileIntensityMsg class.</summary>
                GoProfileIntensityMsg()
                {
                    ::GoProfileIntensityMsg handle = kNULL;

                    KCheck(::GoProfileIntensityMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoProfileIntensityMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoProfileIntensityMsg(KAlloc^ allocator)
                {
                    ::GoProfileIntensityMsg handle = kNULL;

                    KCheck(::GoProfileIntensityMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoProfileIntensityMsg_Source(Handle); }
                }

                /// <summary>Gets the count of profile intensity arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoProfileIntensityMsg_Count(Handle); }
                }

                /// <summary>Gets the count of intensity values in each profile intensity array.</summary>
                property k32s Width
                {
                    k32s get() { return (k32s) ::GoProfileIntensityMsg_Width(Handle); }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoProfileIntensityMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoProfileIntensityMsg_XOffset(Handle); }
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s profileIndex, k64s pointIndex, k8u value)
                {
                    k8u* points = ::GoProfileIntensityMsg_At(Handle, (kSize) profileIndex);

                    points[pointIndex] = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k8u Get(k64s profileIndex, k64s pointIndex)
                {
                    k8u* points = ::GoProfileIntensityMsg_At(Handle, (kSize) profileIndex);

                    return points[pointIndex];
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoProfileIntensityMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoProfileIntensityMsg_Exposure(Handle); }
                }

                /// <summary>Gets the source camera index.</summary>
                property k8u CameraIndex
                {
                    k8u get() { return (k8u) ::GoProfileIntensityMsg_CameraIndex(Handle); }
                }
            };

            /// <summary>Represents a data message containing a surface array.</summary>
            public ref class GoUniformSurfaceMsg : public GoDataMsg
            {
                KDeclareClass(GoUniformSurfaceMsg, GoUniformSurfaceMsg)

            public:
                /// <summary>Initializes a new instance of the GoUniformSurfaceMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoUniformSurfaceMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoUniformSurfaceMsg class.</summary>
                GoUniformSurfaceMsg()
                {
                    ::GoUniformSurfaceMsg handle = kNULL;

                    KCheck(::GoUniformSurfaceMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoUniformSurfaceMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoUniformSurfaceMsg(KAlloc^ allocator)
                {
                    ::GoUniformSurfaceMsg handle = kNULL;

                    KCheck(::GoUniformSurfaceMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoUniformSurfaceMsg_Source(Handle); }
                }

                /// <summary>Gets the length of the surface (row count).</summary>
                property k64s Length
                {
                    k64s get() { return (k64s) ::GoUniformSurfaceMsg_Length(Handle); }
                }

                /// <summary>Gets the width of the surface (column count).</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoUniformSurfaceMsg_Width(Handle); }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile y-resolution, in nanometers.</summary>
                property k32s YResolution
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_YResolution(Handle); }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_XOffset(Handle); }
                }

                /// <summary>Gets the profile y-offset, in micrometers.</summary>
                property k32s YOffset
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_YOffset(Handle); }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_ZOffset(Handle); }
                }

                /// <summary>Gets a pointer to a surface row.</summary>
                IntPtr RowAt(k32s y)
                {
                    return IntPtr(::GoUniformSurfaceMsg_RowAt(Handle, (kSize)y));
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s rowIndex, k64s colIndex, k16s value)
                {
                    k16s* row = ::GoUniformSurfaceMsg_RowAt(Handle, (kSize) rowIndex);

                    row[colIndex] = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k16s Get(k64s rowIndex, k64s colIndex)
                {
                    k16s* row = ::GoUniformSurfaceMsg_RowAt(Handle, (kSize) rowIndex);

                    return row[colIndex];
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoUniformSurfaceMsg_RowAt(Handle, 0)); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoUniformSurfaceMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a generic message containing a buffer or an kObject.</summary>
            public ref class GoGenericMsg : public GoDataMsg
            {
                KDeclareClass(GoGenericMsg, GoGenericMsg)

            public:
                /// <summary>Initializes a new instance of the GoGenericMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoGenericMsg(IntPtr handle) : GoDataMsg(handle)
                {
                }
                
                /// <summary>Initializes a new instance of the GoGenericMsg class.</summary>
                GoGenericMsg() : GoGenericMsg(nullptr)
                {
                }
                
                /// <inheritdoc cref="GoGenericMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoGenericMsg(KAlloc^ allocator)
                {
                    ::GoGenericMsg handle = kNULL;

                    KCheck(::GoGenericMsg_Construct(&handle, KToHandle(allocator)));
                    Handle = handle;
                }
                
                /// <summary>User defined data type ID.</summary>
                property k32u UserType
                {
                    k32u get()
                    {
                        return ::GoGenericMsg_UserType(Handle);
                    }
                }

                /// <summary>Whether or not the content is an kObject.</summary>
                property bool IsObject
                {
                    bool get()
                    {
                        return KToBool(::GoGenericMsg_IsObject(Handle));
                    }
                }

                /// <summary>Buffer length in bytes.</summary>
                property kSize BufferSize
                {
                    kSize get()
                    {
                        return ::GoGenericMsg_BufferSize(Handle);
                    }
                }

                /// <summary>Pointer to buffer.</summary>
                property IntPtr BufferData
                {
                    IntPtr get()
                    {
                        return IntPtr((void*)::GoGenericMsg_BufferData(Handle));
                    }
                }

                /// <summary>Content object, if content is an kObject instance.</summary>
                property KObject^ Object
                {
                    KObject^ get()
                    {
                        kObject child = ::GoGenericMsg_Object(Handle);

                        KAdjustRef(child, kTRUE, Nullable<KRefStyle>());

                        return KToObject<KObject^>(child);
                    }
                }

                /// <summary>Serializer status after deserializing kObject.</summary>
                property kStatus SerializerStatus
                {
                    kStatus get()
                    {
                        return ::GoGenericMsg_SerializerStatus(Handle);
                    }
                }
            };

            /// <summary>Represents a data message containing an unresampled surface array.</summary>
            public ref class GoSurfacePointCloudMsg : public GoDataMsg
            {
                KDeclareClass(GoSurfacePointCloudMsg, GoSurfacePointCloudMsg)

            public:
                /// <summary>Initializes a new instance of the GoSurfacePointCloudMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfacePointCloudMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfacePointCloudMsg class.</summary>
                GoSurfacePointCloudMsg()
                {
                    ::GoSurfacePointCloudMsg handle = kNULL;

                    KCheck(::GoSurfacePointCloudMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfacePointCloudMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfacePointCloudMsg(KAlloc^ allocator)
                {
                    ::GoSurfacePointCloudMsg handle = kNULL;

                    KCheck(::GoSurfacePointCloudMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoSurfacePointCloudMsg_Source(Handle); }
                }

                /// <summary>Gets the length of the surface (row count).</summary>
                property k64s Length
                {
                    k64s get() { return (k64s) ::GoSurfacePointCloudMsg_Length(Handle); }
                }

                /// <summary>Gets the width of the surface (column count).</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoSurfacePointCloudMsg_Width(Handle); }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile y-resolution, in nanometers.</summary>
                property k32s YResolution
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_YResolution(Handle); }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_XOffset(Handle); }
                }

                /// <summary>Gets the profile y-offset, in micrometers.</summary>
                property k32s YOffset
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_YOffset(Handle); }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_ZOffset(Handle); }
                }

                /// <summary>Gets a pointer to a surface row.</summary>
                IntPtr RowAt(k32s y)
                {
                    return IntPtr(::GoSurfacePointCloudMsg_RowAt(Handle, (kSize)y));
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s rowIndex, k64s colIndex, kPoint3d16s value)
                {
                    kPoint3d16s* row = ::GoSurfacePointCloudMsg_RowAt(Handle, (kSize)rowIndex);

                    row[colIndex] = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                kPoint3d16s Get(k64s rowIndex, k64s colIndex)
                {
                    kPoint3d16s* row = ::GoSurfacePointCloudMsg_RowAt(Handle, (kSize)rowIndex);

                    return row[colIndex];
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoSurfacePointCloudMsg_RowAt(Handle, 0)); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoSurfacePointCloudMsg_Exposure(Handle); }
                }

                /// <summary>Is the unresampled surface data adjacent/sorted?</summary>
                property bool IsAdjacent
                {
                    bool get() { return KToBool(::GoSurfacePointCloudMsg_IsAdjacent(Handle))
                    ; }
                }
            };
            
            /// <summary>Represents a data message containing a surface intensity array.</summary>
            public ref class GoSurfaceIntensityMsg : public GoDataMsg
            {
                KDeclareClass(GoSurfaceIntensityMsg, GoSurfaceIntensityMsg)

            public:
                /// <summary>Initializes a new instance of the GoSurfaceIntensityMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSurfaceIntensityMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoSurfaceIntensityMsg class.</summary>
                GoSurfaceIntensityMsg()
                {
                    ::GoSurfaceIntensityMsg handle = kNULL;

                    KCheck(::GoSurfaceIntensityMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSurfaceIntensityMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoSurfaceIntensityMsg(KAlloc^ allocator)
                {
                    ::GoSurfaceIntensityMsg handle = kNULL;

                    KCheck(::GoSurfaceIntensityMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoSurfaceIntensityMsg_Source(Handle); }
                }

                /// <summary>Gets the length of the surface (row count).</summary>
                property k64s Length
                {
                    k64s get() { return (k64s) ::GoSurfaceIntensityMsg_Length(Handle); }
                }

                /// <summary>Gets the width of the surface (column count).</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoSurfaceIntensityMsg_Width(Handle); }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoSurfaceIntensityMsg_XResolution(Handle); }
                }

                /// <summary>Gets the profile y-resolution, in nanometers.</summary>
                property k32s YResolution
                {
                    k32s get() { return (k32s) ::GoSurfaceIntensityMsg_YResolution(Handle); }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoSurfaceIntensityMsg_XOffset(Handle); }
                }

                /// <summary>Gets the profile y-offset, in micrometers.</summary>
                property k32s YOffset
                {
                    k32s get() { return (k32s) ::GoSurfaceIntensityMsg_YOffset(Handle); }
                }

                /// <summary>Gets a pointer to a surface row.</summary>
                IntPtr RowAt(k64s y)
                {
                    return IntPtr(::GoSurfaceIntensityMsg_RowAt(Handle, (kSize)y));
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s rowIndex, k64s colIndex, k8u value)
                {
                    k8u* row = ::GoSurfaceIntensityMsg_RowAt(Handle, (kSize) rowIndex);

                    row[colIndex] = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k8u Get(k64s rowIndex, k64s colIndex)
                {
                    k8u* row = ::GoSurfaceIntensityMsg_RowAt(Handle, (kSize) rowIndex);

                    return row[colIndex];
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoSurfaceIntensityMsg_RowAt(Handle, 0)); }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoSurfaceIntensityMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a data message containing mesh data.</summary>
            public ref class GoMeshMsg : public GoDataMsg
            {
                KDeclareClass(GoMeshMsg, GoMeshMsg)

            public:
                /// <summary>Initializes a new instance of the GoMeshMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoMeshMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoMeshMsg class.</summary>
                GoMeshMsg()
                {
                    ::GoMeshMsg handle = kNULL;

                    KCheck(::GoMeshMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoMeshMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoMeshMsg(KAlloc^ allocator)
                {
                    ::GoMeshMsg handle = kNULL;

                    KCheck(::GoMeshMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets HasData.</summary>
                property KBool HasData
                {
                    KBool get() { return (KBool) ::GoMeshMsg_HasData(Handle); }
                }

                /// <summary>Gets Offset.</summary>
                property KPoint3d64f Offset
                {
                    KPoint3d64f get() { return (KPoint3d64f) ::GoMeshMsg_Offset(Handle); }
                }

                /// <summary>Gets Range.</summary>
                property KPoint3d64f Range
                {
                    KPoint3d64f get() { return (KPoint3d64f) ::GoMeshMsg_Range(Handle); }
                }

                /// <summary>Gets Transform.</summary>
                property Go3dTransform64f Transform
                {
                    Go3dTransform64f get() { return (Go3dTransform64f) ::GoMeshMsg_Transform(Handle); }
                }


                /// <summary>Gets Channel Count.</summary>
                property k64u ChannelCount
                {
                    k64u get() { return (k64u) ::GoMeshMsg_ChannelCount(Handle); }
                }

                /// <summary>Gets Channel Type.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                GoMeshMsgChannelType GetChannelType(GoMeshMsgChannelId id)
                {
                    return (GoMeshMsgChannelType) (::GoMeshMsg_ChannelType(Handle, id));
                }

                /// <summary>Gets Channel State.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                GoMeshMsgChannelState GetChannelState(GoMeshMsgChannelId id)
                {
                    return (GoMeshMsgChannelState) (::GoMeshMsg_ChannelState(Handle, id));
                }

                /// <summary>Gets Channel Flag.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                k32u GetChannelFlag(GoMeshMsgChannelId id)
                {
                    return (K32u) (::GoMeshMsg_ChannelFlag(Handle, id));
                }

                /// <summary>Gets Channel Data Allocation Count.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                kSize GetAllocatedChannelDataCount(GoMeshMsgChannelId id)
                {
                    return (kSize) (::GoMeshMsg_AllocatedChannelDataCount(Handle, id));
                }

                /// <summary>Gets Channel Data Used Count.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                kSize GetUsedChannelDataCount(GoMeshMsgChannelId id)
                {
                    return (kSize) (::GoMeshMsg_UsedChannelDataCount(Handle, id));
                }

                /// <summary>Gets Channel Data Type.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                kType GetChannelDataType(GoMeshMsgChannelId id)
                {
                    return (kType) (::GoMeshMsg_ChannelDataType(Handle, id));
                }

                /// <summary>Gets Channel Data Buffer.</summary>
                /// <param name="id">Channel Id. Fist 6 channels are reserved for system usage</param>
                KArray1^ GetChannelData(GoMeshMsgChannelId id)
                {
                    kArray1 arr = ::GoMeshMsg_ChannelData(Handle, id);
                    return KToObject<KArray1^>(arr);
                }
            };

            /// <summary>Represents a data message containing a set of section arrays.</summary>
            public ref class GoSectionMsg : public GoDataMsg
            {
                KDeclareClass(GoSectionMsg, GoSectionMsg)

            public:
                /// <summary>Initializes a new instance of the GoSectionMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSectionMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoSectionMsg class.</summary>
                GoSectionMsg()
                {
                    ::GoSectionMsg handle = kNULL;

                    KCheck(::GoSectionMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSectionMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoSectionMsg(KAlloc^ allocator)
                {
                    ::GoSectionMsg handle = kNULL;

                    KCheck(::GoSectionMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the section ID.</summary>
                property k32u Id
                {
                    k32u get() { return (k32u) ::GoSectionMsg_Id(Handle); }
                }

                /// <summary>Gets the section source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoSectionMsg_Source(Handle); }
                }

                /// <summary>Gets the count of section arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoSectionMsg_Count(Handle); }
                }

                /// <summary>Gets the count of points in each section array.</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoSectionMsg_Width(Handle); }
                }

                /// <summary>Gets the x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoSectionMsg_XResolution(Handle); }
                }

                /// <summary>Gets the z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) ::GoSectionMsg_ZResolution(Handle); }
                }

                /// <summary>Gets the x-pose, in micrometers.</summary>
                property k32s XPose
                {
                    k32s get() { return (k32s) ::GoSectionMsg_XPose(Handle); }
                }

                /// <summary>Gets the y-pose, in micrometers.</summary>
                property k32s YPose
                {
                    k32s get() { return (k32s) ::GoSectionMsg_YPose(Handle); }
                }

                /// <summary>Gets the pose angle, in microdegrees.</summary>
                property k32s AnglePose
                {
                    k32s get() { return (k32s) ::GoSectionMsg_AnglePose(Handle); }
                }

                /// <summary>Gets the x-offset, in nanometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoSectionMsg_XOffset(Handle); }
                }

                /// <summary>Gets the z-offset, in nanometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) ::GoSectionMsg_ZOffset(Handle); }
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="index">Section index.</param>
                /// <param name="value">Section value.</param>
                void Set(k64s index, k16s value)
                {
                    *::GoSectionMsg_At(Handle, (kSize)index) = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="index">Section index.</param>
                /// <returns>Section value at given indices.</returns>
                k16s Get(k64s index)
                {
                    return *::GoSectionMsg_At(Handle, (kSize)index);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoSectionMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoSectionMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a data message containing a set of section arrays.</summary>
            public ref class GoSectionIntensityMsg : public GoDataMsg
            {
                KDeclareClass(GoSectionIntensityMsg, GoSectionIntensityMsg)

            public:
                /// <summary>Initializes a new instance of the GoSectionIntensityMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoSectionIntensityMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoSectionIntensityMsg class.</summary>
                GoSectionIntensityMsg()
                {
                    ::GoSectionIntensityMsg handle = kNULL;

                    KCheck(::GoSectionIntensityMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoSectionIntensityMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoSectionIntensityMsg(KAlloc^ allocator)
                {
                    ::GoSectionIntensityMsg handle = kNULL;

                    KCheck(::GoSectionIntensityMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Gets the section ID.</summary>
                property k32u Id
                {
                    k32u get() { return (k32u) ::GoSectionIntensityMsg_Id(Handle); }
                }

                /// <summary>Gets the section source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) ::GoSectionIntensityMsg_Source(Handle); }
                }

                /// <summary>Gets the count of section arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoSectionIntensityMsg_Count(Handle); }
                }

                /// <summary>Gets the count of points in each section array.</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoSectionIntensityMsg_Width(Handle); }
                }

                /// <summary>Gets the x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_XResolution(Handle); }
                }

                /// <summary>Gets the x-pose, in micrometers.</summary>
                property k32s XPose
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_XPose(Handle); }
                }

                /// <summary>Gets the y-pose, in micrometers.</summary>
                property k32s YPose
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_YPose(Handle); }
                }

                /// <summary>Gets the pose angle, in microdegrees.</summary>
                property k32s AnglePose
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_AnglePose(Handle); }
                }

                /// <summary>Gets the x-offset, in nanometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_XOffset(Handle); }
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="index">Section intensity index.</param>
                /// <param name="value">Section intensity value.</param>
                void Set(k64s index, k8u value)
                {
                    *::GoSectionIntensityMsg_At(Handle, (kSize)index) = value;
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="index">Section intensity index.</param>
                /// <returns>Section value at given indices.</returns>
                k8u Get(k64s index)
                {
                    return *::GoSectionIntensityMsg_At(Handle, (kSize)index);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoSectionIntensityMsg_At(Handle, 0)); }
                }

                /// <summary>Gets the exposure.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) ::GoSectionIntensityMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a measurement result.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoMeasurementData))]
            public value struct GoMeasurementData
            {
                KDeclareStruct(GoMeasurementData, GoMeasurementData)

                /// <summary>Measurement value.</summary>
                [FieldOffset(offsetof(::GoMeasurementData, value))]
                k64f Value;

                /// <summary>Measurement decision value.</summary>
                property GoDecision Decision
                {
                    GoDecision get() { return (GoDecision)decision; }
                    void set(GoDecision d) { decision = (k8u)d; }
                }

                /// <summary>Measurement decision code - relevant only when the value represents a failure.</summary>
                property GoDecisionCode DecisionCode
                {
                    GoDecisionCode get() { return (GoDecisionCode)decisionCode; }
                    void set(GoDecisionCode d) { decisionCode = (k8u)d; }
                }

            private:
                [FieldOffset(offsetof(::GoMeasurementData, decision))]
                k8u decision;

                [FieldOffset(offsetof(::GoMeasurementData, decisionCode))]
                k8u decisionCode;
            };

            /// <summary>Represents a message containing a set of GoMeasurementData.</summary>
            public ref class GoMeasurementMsg : public GoDataMsg
            {
                KDeclareClass(GoMeasurementMsg, GoMeasurementMsg)

            public:
                /// <summary>Initializes a new instance of the GoMeasurementMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoMeasurementMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoMeasurementMsg class.</summary>
                GoMeasurementMsg()
                {
                    ::GoMeasurementMsg handle = kNULL;

                    KCheck(::GoMeasurementMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoMeasurementMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoMeasurementMsg(KAlloc^ allocator)
                {
                    ::GoMeasurementMsg handle = kNULL;

                    KCheck(::GoMeasurementMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The measurement identifier.</summary>
                property k16u Id
                {
                    k16u get() { return (k16u) ::GoMeasurementMsg_Id(Handle); }
                }

                /// <summary>Count of measurements in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoMeasurementMsg_Count(Handle); }
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="index">Section intensity index.</param>
                /// <param name="value">Section intensity value.</param>
                void Set(k64s index, GoMeasurementData value)
                {
                    *::GoMeasurementMsg_At(Handle, (kSize)index) = value.ToNative();
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="index">Section intensity index.</param>
                /// <returns>Section value at given indices.</returns>
                GoMeasurementData Get(k64s index)
                {
                   return (GoMeasurementData)(*::GoMeasurementMsg_At(Handle, (kSize)index));
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoMeasurementMsg_At(Handle, 0)); }
                }
            };

            /// <summary>Represents a message containing an alignment result.</summary>
            public ref class GoAlignMsg : public GoDataMsg
            {
                KDeclareClass(GoAlignMsg, GoAlignMsg)

            public:
                /// <summary>Initializes a new instance of the GoAlignMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoAlignMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoAlignMsg class.</summary>
                GoAlignMsg()
                {
                    ::GoAlignMsg handle = kNULL;

                    KCheck(::GoAlignMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoAlignMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoAlignMsg(KAlloc^ allocator)
                {
                    ::GoAlignMsg handle = kNULL;

                    KCheck(::GoAlignMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The alignment result.</summary>
                property KStatus Status
                {
                    KStatus get() { return KStatus(::GoAlignMsg_Status(Handle)); }
                }
            };

            /// <summary>Represents a message containing exposure calibration results.</summary>
            public ref class GoExposureCalMsg : public GoDataMsg
            {
                KDeclareClass(GoExposureCalMsg, GoExposureCalMsg)

            public:
                /// <summary>Initializes a new instance of the GoExposureCalMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoExposureCalMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoExposureCalMsg class.</summary>
                GoExposureCalMsg()
                {
                    ::GoExposureCalMsg handle = kNULL;

                    KCheck(::GoExposureCalMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoExposureCalMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoExposureCalMsg(KAlloc^ allocator)
                {
                    ::GoExposureCalMsg handle = kNULL;

                    KCheck(::GoExposureCalMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The exposure calibration result.</summary>
                property KStatus Status
                {
                    KStatus get() { return KStatus(::GoExposureCalMsg_Status(Handle)); }
                }

                /// <summary>The calibrated exposure.</summary>
                property k64f Exposure
                {
                    k64f get() { return (k64f) ::GoExposureCalMsg_Exposure(Handle); }
                }
            };

            /// <summary>Represents a message containing edge based part matching results.</summary>
            public ref class GoEdgeMatchMsg : public GoDataMsg
            {
                KDeclareClass(GoEdgeMatchMsg, GoEdgeMatchMsg)

            public:
                /// <summary>Initializes a new instance of the GoEdgeMatchMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoEdgeMatchMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoEdgeMatchMsg class.</summary>
                GoEdgeMatchMsg()
                {
                    ::GoEdgeMatchMsg handle = kNULL;

                    KCheck(::GoEdgeMatchMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoEdgeMatchMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoEdgeMatchMsg(KAlloc^ allocator)
                {
                    ::GoEdgeMatchMsg handle = kNULL;

                    KCheck(::GoEdgeMatchMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The edge match decision.</summary>
                property k8u Decision
                {
                    k8u get() { return (k8u) ::GoEdgeMatchMsg_Decision(Handle); }
                }

                /// <summary>The edge match X offset.</summary>
                property k64f XOffset
                {
                    k64f get() { return (k64f) ::GoEdgeMatchMsg_XOffset(Handle); }
                }

                /// <summary>The edge match Y offset.</summary>
                property k64f YOffset
                {
                    k64f get() { return (k64f) ::GoEdgeMatchMsg_YOffset(Handle); }
                }

                /// <summary>The edge match Z angle.</summary>
                property k64f ZAngle
                {
                    k64f get() { return (k64f) ::GoEdgeMatchMsg_ZAngle(Handle); }
                }

                /// <summary>The edge match quality value.</summary>
                property k64f QualityValue
                {
                    k64f get() { return (k64f) ::GoEdgeMatchMsg_QualityValue(Handle); }
                }

                /// <summary>The edge match quality decision.</summary>
                property k8u QualityDecision
                {
                    k8u get() { return (k8u) ::GoEdgeMatchMsg_QualityDecision(Handle); }
                }
            };

            /// <summary>Represents a message containing ellipse based part matching results.</summary>
            public ref class GoEllipseMatchMsg : public GoDataMsg
            {
                KDeclareClass(GoEllipseMatchMsg, GoEllipseMatchMsg)

            public:
                /// <summary>Initializes a new instance of the GoEllipseMatchMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoEllipseMatchMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoEllipseMatchMsg class.</summary>
                GoEllipseMatchMsg()
                {
                    ::GoEllipseMatchMsg handle = kNULL;

                    KCheck(::GoEllipseMatchMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoEllipseMatchMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoEllipseMatchMsg(KAlloc^ allocator)
                {
                    ::GoEllipseMatchMsg handle = kNULL;

                    KCheck(::GoEllipseMatchMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The ellipse match decision.</summary>
                property k8u Decision
                {
                    k8u get() { return (k8u) ::GoEllipseMatchMsg_Decision(Handle); }
                }

                /// <summary>The ellipse match X offset.</summary>
                property k64f XOffset
                {
                    k64f get() { return (k64f) ::GoEllipseMatchMsg_XOffset(Handle); }
                }

                /// <summary>The ellipse match Y offset.</summary>
                property k64f YOffset
                {
                    k64f get() { return (k64f) ::GoEllipseMatchMsg_YOffset(Handle); }
                }

                /// <summary>The ellipse match Z angle.</summary>
                property k64f ZAngle
                {
                    k64f get() { return (k64f) ::GoEllipseMatchMsg_ZAngle(Handle); }
                }

                /// <summary>The ellipse match minor value.</summary>
                property k64f MinorValue
                {
                    k64f get() { return (k64f) ::GoEllipseMatchMsg_MinorValue(Handle); }
                }

                /// <summary>The ellipse match minor decision.</summary>
                property k8u MinorDecision
                {
                    k8u get() { return (k8u) ::GoEllipseMatchMsg_MinorDecision(Handle); }
                }

                /// <summary>The ellipse match major value.</summary>
                property k64f MajorValue
                {
                    k64f get() { return (k64f) ::GoEllipseMatchMsg_MajorValue(Handle); }
                }

                /// <summary>The ellipse match major decision.</summary>
                property k8u MajorDecision
                {
                    k8u get() { return (k8u) ::GoEllipseMatchMsg_MajorDecision(Handle); }
                }
            };

            /// <summary>Represents a message containing bounding box based part matching results.</summary>
            public ref class GoBoundingBoxMatchMsg : public GoDataMsg
            {
                KDeclareClass(GoBoundingBoxMatchMsg, GoBoundingBoxMatchMsg)

            public:
                /// <summary>Initializes a new instance of the GoBoundingBoxMatchMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoBoundingBoxMatchMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoBoundingBoxMatchMsg class.</summary>
                GoBoundingBoxMatchMsg()
                {
                    ::GoBoundingBoxMatchMsg handle = kNULL;

                    KCheck(::GoBoundingBoxMatchMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoBoundingBoxMatchMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoBoundingBoxMatchMsg(KAlloc^ allocator)
                {
                    ::GoBoundingBoxMatchMsg handle = kNULL;

                    KCheck(::GoBoundingBoxMatchMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The bounding box match decision.</summary>
                property k8u Decision
                {
                    k8u get() { return (k8u) ::GoBoundingBoxMatchMsg_Decision(Handle); }
                }

                /// <summary>The bounding box match X offset.</summary>
                property k64f XOffset
                {
                    k64f get() { return (k64f) ::GoBoundingBoxMatchMsg_XOffset(Handle); }
                }

                /// <summary>The bounding box match Y offset.</summary>
                property k64f YOffset
                {
                    k64f get() { return (k64f) ::GoBoundingBoxMatchMsg_YOffset(Handle); }
                }

                /// <summary>The bounding box match Z angle.</summary>
                property k64f ZAngle
                {
                    k64f get() { return (k64f) ::GoBoundingBoxMatchMsg_ZAngle(Handle); }
                }

                /// <summary>The bounding box match length value.</summary>
                property k64f LengthValue
                {
                    k64f get() { return (k64f) ::GoBoundingBoxMatchMsg_LengthValue(Handle); }
                }

                /// <summary>The bounding box match length decision.</summary>
                property k8u LengthDecision
                {
                    k8u get() { return (k8u) ::GoBoundingBoxMatchMsg_LengthDecision(Handle); }
                }

                /// <summary>The bounding box match width value.</summary>
                property k64f WidthValue
                {
                    k64f get() { return (k64f) ::GoBoundingBoxMatchMsg_WidthValue(Handle); }
                }

                /// <summary>The bounding box match width decision.</summary>
                property k8u WidthDecision
                {
                    k8u get() { return (k8u) ::GoBoundingBoxMatchMsg_WidthDecision(Handle); }
                }
            };

            /// <summary>Represents a message containing an event.</summary>
            public ref class GoEventMsg : public GoDataMsg
            {
                KDeclareClass(GoEventMsg, GoEventMsg)

            public:
                /// <summary>Initializes a new instance of the GoEventMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoEventMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoEventMsg class.</summary>
                GoEventMsg()
                {
                    ::GoEventMsg handle = kNULL;

                    KCheck(::GoEventMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoEventMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoEventMsg(KAlloc^ allocator)
                {
                    ::GoEventMsg handle = kNULL;

                    KCheck(::GoEventMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The event type.</summary>
                property GoEventType EventType
                {
                    GoEventType get() { return (GoEventType)::GoEventMsg_Type(Handle); }
                }
            };

            /// <summary>Represents a tracheid ellipse.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(::GoTracheidEllipse))]
            public value struct GoTracheidEllipse
            {
                KDeclareStruct(GoTracheidEllipse, GoTracheidEllipse)

                [FieldOffset(offsetof(::GoTracheidEllipse, area))]
                k64f Area;

                [FieldOffset(offsetof(::GoTracheidEllipse, angle))]
                k64f Angle;

                [FieldOffset(offsetof(::GoTracheidEllipse, scatter))]
                k64f Scatter;

                [FieldOffset(offsetof(::GoTracheidEllipse, minor))]
                k64f Minor;

                [FieldOffset(offsetof(::GoTracheidEllipse, major))]
                k64f Major;
            };

            /// <summary>Represents a message containing a tracheid result.</summary>
            public ref class GoTracheidMsg : public GoDataMsg
            {
                KDeclareClass(GoTracheidMsg, GoTracheidMsg)

            public:
                /// <summary>Initializes a new instance of the GoTracheidMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoTracheidMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoTracheidMsg class.</summary>
                GoTracheidMsg()
                {
                    ::GoTracheidMsg handle = kNULL;

                    KCheck(::GoTracheidMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoTracheidMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoTracheidMsg(KAlloc^ allocator)
                {
                    ::GoTracheidMsg handle = kNULL;

                    KCheck(::GoTracheidMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>The tracheid source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource)::GoTracheidMsg_Source(Handle); }
                }

                /// <summary>Gets the count of tracheid arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return (k64s) ::GoTracheidMsg_Count(Handle); }
                }

                /// <summary>Gets the count of ellipses in each tracheid array.</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) ::GoTracheidMsg_Width(Handle); }
                }

                /// <summary>The camera index.</summary>
                property k8u CameraIndex
                {
                    k8u get() { return (k8u)::GoTracheidMsg_CameraIndex(Handle); }
                }

                /// <summary>Sets the ellipse at the specified index.</summary>
                /// <param name="tracheidIndex">Tracheid index.</param>
                /// <param name="ellipseIndex">Ellipse index.</param>
                /// <param name="ellipse">Ellipse object.</param>
                void Set(k64s tracheidIndex, k64s ellipseIndex, GoTracheidEllipse ellipse)
                {
                    ::GoTracheidEllipse* ellipses = GoTracheidMsg_At(Handle, (kSize)tracheidIndex);

                    ellipses[ellipseIndex] = (::GoTracheidEllipse)ellipse;
                }

                /// <summary>Gets the ellipse at the specified index.</summary>
                /// <param name="tracheidIndex">Tracheid index.</param>
                /// <param name="ellipseIndex">Ellipse index.</param>
                /// <returns>Ellipse at given index.</returns>
                GoTracheidEllipse Get(k64s tracheidIndex, k64s ellipseIndex)
                {
                    ::GoTracheidEllipse* ellipses = ::GoTracheidMsg_At(Handle, (kSize)tracheidIndex);

                    return (GoTracheidEllipse)ellipses[ellipseIndex];
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return IntPtr(::GoTracheidMsg_At(Handle, 0)); }
                }
            };

            /// <summary>Represents a message containing circular feature data.</summary>
            public ref class GoCircleFeatureMsg : public GoDataMsg
            {
                KDeclareClass(GoCircleFeatureMsg, GoCircleFeatureMsg)

            public:
                /// <summary>Initializes a new instance of the GoCircleFeatureMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoCircleFeatureMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoCircleFeatureMsg class.</summary>
                GoCircleFeatureMsg()
                {
                    ::GoCircleFeatureMsg handle = kNULL;

                    KCheck(::GoCircleFeatureMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoCircleFeatureMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoCircleFeatureMsg(KAlloc^ allocator)
                {
                    ::GoCircleFeatureMsg handle = kNULL;

                    KCheck(::GoCircleFeatureMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }
                
                /// <summary>Gets the circular feature identifier.</summary>
                property k16u Id
                {
                    k16u get() { return (k16u)::GoCircleFeatureMsg_Id(Handle); }
                }

                /// <summary>Gets the position of the center of the circular feature in nanometers.</summary>
                property KPoint3d64f Position
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoCircleFeatureMsg_Position(Handle);}
                }

                /// <summary>Gets the normal vector to the plane of the circular feature.</summary>
                property KPoint3d64f Normal
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoCircleFeatureMsg_Normal(Handle);}
                }

                /// <summary>Gets the radius of the circular feature in nanometers.</summary>
                property k64f Radius
                {
                    k64f get() { return (k64f)::GoCircleFeatureMsg_Radius(Handle);}
                }
            };
            
            /// <summary>Represents a message containing Linear Feature data.</summary>
            public ref class GoLineFeatureMsg : public GoDataMsg
            {
                KDeclareClass(GoLineFeatureMsg, GoLineFeatureMsg)

            public:
                /// <summary>Initializes a new instance of the GoLineFeatureMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoLineFeatureMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoLineFeatureMsg class.</summary>
                GoLineFeatureMsg()
                {
                    ::GoLineFeatureMsg handle = kNULL;

                    KCheck(::GoLineFeatureMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoLineFeatureMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoLineFeatureMsg(KAlloc^ allocator)
                {
                    ::GoLineFeatureMsg handle = kNULL;

                    KCheck(::GoLineFeatureMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }
                
                /// <summary>Gets the linear feature identifier.</summary>
                property k16u Id
                {
                    k16u get() { return (k16u)::GoLineFeatureMsg_Id(Handle); }
                }
                
                /// <summary> a point on the linear feature.</summary>
                property KPoint3d64f Position
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoLineFeatureMsg_Position(Handle); }
                }
                
                /// <summary>The direction vector of the linear feature.</summary>
                property KPoint3d64f Direction
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoLineFeatureMsg_Direction(Handle); }
                }
            };

            /// <summary>Represents a message containing Planar feature data.</summary>
            public ref class GoPlaneFeatureMsg : public GoDataMsg
            {
                KDeclareClass(GoPlaneFeatureMsg, GoPlaneFeatureMsg)

            public:
                /// <summary>Initializes a new instance of the GoPlaneFeatureMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoPlaneFeatureMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoPlaneFeatureMsg class.</summary>
                GoPlaneFeatureMsg()
                {
                    ::GoPlaneFeatureMsg handle = kNULL;

                    KCheck(::GoPlaneFeatureMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoPlaneFeatureMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoPlaneFeatureMsg(KAlloc^ allocator)
                {
                    ::GoPlaneFeatureMsg handle = kNULL;

                    KCheck(::GoPlaneFeatureMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }
                
                /// <summary>Gets the planear feature identifier.</summary>
                property k16u Id
                {
                    k16u get() { return (k16u)::GoPlaneFeatureMsg_Id(Handle); }
                }
                
                /// <summary>Gets the normal vector of the planear feature.</summary>
                property KPoint3d64f Normal
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoPlaneFeatureMsg_Normal(Handle); }
                }
                
                /// <summary>Gets shortest distance from the origin to the plane. Is parallel to the normal vector.</summary>
                property k64f DistanceToOrigin
                {
                    k64f get() { return (k64f)::GoPlaneFeatureMsg_DistanceToOrigin(Handle); }
                }
            };

            /// <summary>Represents a message containing Point feature data.</summary>
            public ref class GoPointFeatureMsg : public GoDataMsg
            {
                KDeclareClass(GoPointFeatureMsg, GoPointFeatureMsg)

            public:
                /// <summary>Initializes a new instance of the GoPointFeatureMsg class with the specified Zen object handle.</summary>
                /// <param name="handle">Zen object handle.</param>
                GoPointFeatureMsg(IntPtr handle)
                    : GoDataMsg(handle)
                {}

                /// <summary>Initializes a new instance of the GoPointFeatureMsg class.</summary>
                GoPointFeatureMsg()
                {
                    ::GoPointFeatureMsg handle = kNULL;

                    KCheck(::GoPointFeatureMsg_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="GoPointFeatureMsg()" />
                /// <param name="allocator">Memory allocator</param>
                GoPointFeatureMsg(KAlloc^ allocator)
                {
                    ::GoPointFeatureMsg handle = kNULL;

                    KCheck(::GoPointFeatureMsg_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }
                
                /// <summary>Gets the point feature identifier.</summary>
                property k16u Id
                {
                    k16u get() { return (k16u)::GoPointFeatureMsg_Id(Handle); }
                }
                
                /// <summary>The point data in this message.</summary>
                property KPoint3d64f Position
                {
                    KPoint3d64f get() { return (KPoint3d64f)::GoPointFeatureMsg_Position(Handle); }
                }
            };

#undef GoProfileMsg
#undef GoResampledProfileMsg
#undef GoSurfaceMsg
#pragma warning( push )
#pragma warning(disable: 4947)
            /// <summary>Deprecated, use GoProfilePointCloudMsg instead</summary>
            [Obsolete("Use GoProfilePointCloudMsg instead.")]
            public ref class GoProfileMsg
            {
            private:
                GoProfilePointCloudMsg ^ msg;
            public:
                property GoDataMessageType MessageType
                {
                    GoDataMessageType get() { return msg->MessageType; }
                }

                GoProfileMsg(GoProfilePointCloudMsg^ other)
                {
                    msg = other;
                }

                GoProfileMsg()
                {
                    msg = gcnew GoProfilePointCloudMsg();
                }

                GoProfileMsg(KAlloc^ allocator)
                {
                    msg = gcnew GoProfilePointCloudMsg(allocator);
                }

                static explicit operator GoProfileMsg ^ (GoProfilePointCloudMsg^ other)
                {
                    return gcnew GoProfileMsg(other);
                }

                /// <summary>Gets the count of profile arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return msg->Count; }
                }

                /// <summary>Gets the count of ranges in each profile array.</summary>
                property k32s Width
                {
                    k32s get() { return (k32s)msg->Width; }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s)msg->XResolution; }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s)msg->ZResolution; }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s)msg->XOffset; }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s)msg->ZOffset; }
                }

                /// <summary>Sets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s profileIndex, k64s pointIndex, KPoint16s value)
                {
                    msg->Set(profileIndex, pointIndex, value);
                }

                /// <summary>Gets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                KPoint16s Get(k64s profileIndex, k64s pointIndex)
                {
                    return msg->Get(profileIndex, pointIndex);
                }

                /// <summary>Sets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s pointIndex, KPoint16s value)
                {
                    return Set(0, pointIndex, value);
                }

                /// <summary>Gets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given index.</returns>
                KPoint16s Get(k64s pointIndex)
                {
                    return Get(0, pointIndex);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return msg->Data; }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return msg->Exposure; }
                }
            };

            /// <summary>Deprecated, use GoUniformProfileMsg instead</summary>
            [Obsolete("Use GoUniformProfileMsg instead.")]
            public ref class GoResampledProfileMsg
            {
            private:
                GoUniformProfileMsg ^msg;
            public:
                property GoDataMessageType MessageType
                {
                    GoDataMessageType get() { return msg->MessageType; }
                }

                GoResampledProfileMsg(GoUniformProfileMsg^ other)
                {
                    msg = other;
                }

                GoResampledProfileMsg()
                {
                    msg = gcnew GoUniformProfileMsg();
                }

                GoResampledProfileMsg(KAlloc^ allocator)
                {
                    msg = gcnew GoUniformProfileMsg(allocator);
                }

                static explicit operator GoResampledProfileMsg^(GoUniformProfileMsg^ other)
                {
                    return gcnew GoResampledProfileMsg(other);
                }

                /// <summary>Gets the count of profile arrays in this message.</summary>
                property k64s Count
                {
                    k64s get() { return msg->Count; }
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) msg->Source; }
                }

                /// <summary>Gets the count of ranges in each profile array.</summary>
                property k32s Width
                {
                    k32s get() { return (k32s) msg->Width; }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) msg->XResolution; }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) msg->ZResolution; }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) msg->XOffset; }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) msg->ZOffset; }
                }

                /// <summary>Gets the stream step of the stream data source.</summary>
                property GoDataStep StreamStep
                {
                    GoDataStep get()
                    {
                        return msg->StreamStep;
                    }
                }

                /// <summary>Gets the id of the stream data.</summary>
                property k32s StreamStepId
                {
                    k32s get()
                    {
                        return msg->StreamStepId;
                    }
                }

                /// <summary>Sets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s profileIndex, k64s pointIndex, k16s value)
                {
                    msg->Set(profileIndex, pointIndex, value);
                }

                /// <summary>Gets the value at the specified index for the specified frame.</summary>
                /// <param name="profileIndex">Profile index.</param>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k16s Get(k64s profileIndex, k64s pointIndex)
                {
                    return msg->Get(profileIndex, pointIndex);
                }

                /// <summary>Sets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s pointIndex, k16s value)
                {
                    return Set(0, pointIndex, value);
                }

                /// <summary>Gets the point at the specified index.</summary>
                /// <param name="pointIndex">Point index.</param>
                /// <returns>Point value at given index.</returns>
                k16s Get(k64s pointIndex)
                {
                    return Get(0, pointIndex);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return msg->Data; }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return msg->Exposure; }
                }

            };
            /// <summary>Deprecated, use GoUniformSurfaceMsg instead</summary>
            [Obsolete("Use GoUniformSurfaceMsg instead.")]
            public ref class GoSurfaceMsg
            {
            private:
                GoUniformSurfaceMsg ^ msg;
            public:
                property GoDataMessageType MessageType
                {
                    GoDataMessageType get() { return msg->MessageType; }
                }

                GoSurfaceMsg(GoUniformSurfaceMsg^ other)
                {
                    msg = other;
                }

                GoSurfaceMsg()
                {
                    msg = gcnew GoUniformSurfaceMsg();
                }

                GoSurfaceMsg(KAlloc^ allocator)
                {
                    msg = gcnew GoUniformSurfaceMsg(allocator);
                }

                static explicit operator GoSurfaceMsg ^ (GoUniformSurfaceMsg^ other)
                {
                    return gcnew GoSurfaceMsg(other);
                }

                /// <summary>Gets the profile source.</summary>
                property GoDataSource Source
                {
                    GoDataSource get() { return (GoDataSource) msg->Source; }
                }

                /// <summary>Gets the length of the surface (row count).</summary>
                property k64s Length
                {
                    k64s get() { return (k64s) msg->Length; }
                }

                /// <summary>Gets the width of the surface (column count).</summary>
                property k64s Width
                {
                    k64s get() { return (k64s) msg->Width; }
                }

                /// <summary>Gets the profile x-resolution, in nanometers.</summary>
                property k32s XResolution
                {
                    k32s get() { return (k32s) msg->XResolution; }
                }

                /// <summary>Gets the profile y-resolution, in nanometers.</summary>
                property k32s YResolution
                {
                    k32s get() { return (k32s) msg->YResolution; }
                }

                /// <summary>Gets the profile z-resolution, in nanometers.</summary>
                property k32s ZResolution
                {
                    k32s get() { return (k32s) msg->ZResolution; }
                }

                /// <summary>Gets the profile x-offset, in micrometers.</summary>
                property k32s XOffset
                {
                    k32s get() { return (k32s) msg->XOffset; }
                }

                /// <summary>Gets the profile y-offset, in micrometers.</summary>
                property k32s YOffset
                {
                    k32s get() { return (k32s) msg->YOffset; }
                }

                /// <summary>Gets the profile z-offset, in micrometers.</summary>
                property k32s ZOffset
                {
                    k32s get() { return (k32s) msg->ZOffset; }
                }

                /// <summary>Gets the stream step of the stream data source.</summary>
                property GoDataStep StreamStep
                {
                    GoDataStep get() { return msg->StreamStep; }
                }

                /// <summary>Gets the id of the stream data.</summary>
                property k32s StreamStepId
                {
                    k32s get()
                    {
                        return msg->StreamStepId;
                    }
                }

                /// <summary>Gets a pointer to a surface row.</summary>
                IntPtr RowAt(k32s y)
                {
                    return msg->RowAt(y);
                }

                /// <summary>Sets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <param name="value">Point value.</param>
                void Set(k64s rowIndex, k64s colIndex, k16s value)
                {
                    msg->Set(rowIndex, colIndex, value);
                }

                /// <summary>Gets the value at the specified index.</summary>
                /// <param name="rowIndex">Profile index.</param>
                /// <param name="colIndex">Point index.</param>
                /// <returns>Point value at given indices.</returns>
                k16s Get(k64s rowIndex, k64s colIndex)
                {
                    return msg->Get(rowIndex, colIndex);
                }

                /// <summary>Gets a pointer to the internal data buffer.</summary>
                property IntPtr Data
                {
                    IntPtr get() { return msg->Data; }
                }

                /// <summary>Gets the exposure in uS.</summary>
                property k32s Exposure
                {
                    k32s get() { return (k32s) msg->Exposure; }
                }
            };
#pragma warning( pop )       
        }
    }
}

#endif
