/**
 * @file    GoDataTypes.h
 * @brief   Declares Gocator data message classes and related types.
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SDK_DATA_TYPES_H
#define GO_SDK_DATA_TYPES_H

#include <GoSdk/GoSdkDef.h>

/**
 * @class   GoDataMsg
 * @extends kObject
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a base message sourced from the data channel.
 */
typedef kObject GoDataMsg;

/**
 * Returns the message type for a data channel message given in a GoDataSet.
 *
 * @public             @memberof GoDataMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   message    A data channel message.
 * @return             A GoDataMessageType value.
 */
GoFx(GoDataMessageType) GoDataMsg_Type(GoDataMsg message);

/**
* Gets the source of the data stream.
*
* @public             @memberof GoDataMsg
* @version            Introduced in firmware 5.2
* @param   message    Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoDataMsg_StreamStep(GoDataMsg message);

/**
* Gets the identifier of the data stream from the source.
*
* @public             @memberof GoDataMsg
* @version            Introduced in firmware 5.2
* @param   message    Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoDataMsg_StreamStepId(GoDataMsg message);

/**
* Sets the source of the data stream.
*
* @public             @memberof GoDataMsg
* @version            Introduced in firmware 5.2
* @param   message    Message object.
* @param   streamStep streamStep value
* @return             Result status.
*/
GoFx(kStatus) GoDataMsg_SetStreamStep(GoDataMsg message, GoDataStep streamStep);

/**
* Sets the identifier of the data stream from the source.
*
* @public             @memberof GoDataMsg
* @version            Introduced in firmware 5.2
* @param   message        Message object.
* @param   streamStepId streamStepId value
* @return             Result status.
*/
GoFx(kStatus) GoDataMsg_SetStreamStepId(GoDataMsg message, k32s streamStepId);

/**
 * Gets the count of the data messages (in an arrayed context).
 *
 * @public              @memberof GoDataMsg
 * @version             Introduced in firmware 6.0
 * @param   message     Message object.
 * @return              The number of arrayed messages.
 */
GoFx(k32s) GoDataMsg_ArrayedCount(GoDataMsg message);

/**
 * Gets the index of the data message (in an arrayed context).
 *
 * @public              @memberof GoDataMsg
 * @version             Introduced in firmware 6.0
 * @param   message     Message object.
 * @return              The index of the arrayed message.
 */
GoFx(k32s) GoDataMsg_ArrayedIndex(GoDataMsg message);

/**
 * Sets the count of the data messages (in an arrayed context).
 *
 * @public                  @memberof GoDataMsg
 * @version                 Introduced in firmware 6.0
 * @param   message         Message object.
 * @param   arrayedCount    arrayedCount value to set.
 * @return                  Result status.
 */
GoFx(kStatus) GoDataMsg_SetArrayedCount(GoDataMsg message, k32s arrayedCount);

/**
 * Sets the index of the data message (in an arrayed context).
 *
 * @public                  @memberof GoDataMsg
 * @version                 Introduced in firmware 6.0
 * @param   message         Message object.
 * @param   arrayedIndex    arrayedIndex value to set.
 * @return                  Result status.
 */
GoFx(kStatus) GoDataMsg_SetArrayedIndex(GoDataMsg message, k32s arrayedIndex);

/**
 * @struct  GoStamp
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Represents an acquisition stamp.
 */
typedef struct GoStamp
{
    k64u frameIndex;            ///< Frame index (counts up from zero).
    k64u timestamp;             ///< Timestamp in internal units approximating microseconds where the true time in us = timestamp value / 1.024.
    k64s encoder;               ///< Position (encoder ticks).
    k64s encoderAtZ;            ///< Encoder value latched at z-index mark (encoder ticks).
    k64u status;
    /**<
    * Bit mask containing frame information:
    *
    * - Bit 0: Represents sensor digital input state.
    * - Bit 4: Represents Master digital input state.
    * - Bits 8 and 9: Represents inter-frame digital pulse trigger
                    (Master digital input if a Master is connected, otherwise
                    sensor digital input. Value is cleared after each frame
                    and clamped at 3 if more than 3 pulses are received).
    */
    k32u id;                    ///< Source device ID.
    k32u reserved32u;           ///< Reserved.
    k64u reserved64u;           ///< Reserved.
    k64u ptpTime;               ///< PTP time of the stamp. us since the PTP epoch (usually TAI)
} GoStamp;

/**
 * @class   GoStampMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing a set of acquisition stamps.
 */
typedef GoDataMsg GoStampMsg;

/**
 * Gets the stamp source.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Stamp source.
 */
GoFx(GoDataSource) GoStampMsg_Source(GoStampMsg msg);

/**
 * Returns the number of stamps contained in this message.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Count of stamps.
 */
GoFx(kSize) GoStampMsg_Count(GoStampMsg msg);

/**
 * Gets the stamp at the specified index.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Stamp index.
 * @return             Stamp pointer.
 */
GoFx(GoStamp*) GoStampMsg_At(GoStampMsg msg, kSize index);

/**
 * @class   GoVideoMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a video image.
 */
typedef GoDataMsg GoVideoMsg;

/**
 * Gets the video source.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Video source.
 */
GoFx(GoDataSource) GoVideoMsg_Source(GoVideoMsg msg);


/**
 * Gets the camera index that the video data originates from.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Camera index.
 */
GoFx(kSize) GoVideoMsg_CameraIndex(GoVideoMsg msg);


/**
 * Gets the image width, in pixels.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Image width, in pixels.
 */
GoFx(kSize) GoVideoMsg_Width(GoVideoMsg msg);

/**
 * Gets the image height, in pixels.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Image height, in pixels.
 */
GoFx(kSize) GoVideoMsg_Height(GoVideoMsg msg);

/**
 * Gets the data type used to represent an image pixel.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Pixel type.
 */
GoFx(GoPixelType) GoVideoMsg_PixelType(GoVideoMsg msg);

/**
 * Gets the image pixel size, in bytes.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Pixel size, in bytes.
 */
GoFx(kSize) GoVideoMsg_PixelSize(GoVideoMsg msg);

/**
 * Gets the pixel format descriptor.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Pixel format.
 */
GoFx(kPixelFormat) GoVideoMsg_PixelFormat(GoVideoMsg msg);

/**
 * Gets the image color filter array.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Color filter array.
 */
GoFx(kCfa) GoVideoMsg_Cfa(GoVideoMsg msg);

/**
 * Gets a pointer to a row within the image.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   rowIndex   Row index.
 * @return             Row pointer.
 */
GoFx(void*) GoVideoMsg_RowAt(GoVideoMsg msg, kSize rowIndex);

/**
 * Gets the exposure index.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure index.
 */
GoFx(kSize) GoVideoMsg_ExposureIndex(GoVideoMsg msg);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoVideoMsg_Exposure(GoVideoMsg msg);

/**
 * Indicates whether the video message data requires horizontal flipping to match up with profile data.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             kTRUE if rotation is required, kFALSE otherwise.
 */
GoFx(kBool) GoVideoMsg_IsFlippedX(GoVideoMsg msg);

/**
 * Indicates whether the video message data requires vertical flipping to match up with profile data.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             kTRUE if rotation is required, kFALSE otherwise.
 */
GoFx(kBool) GoVideoMsg_IsFlippedY(GoVideoMsg msg);

/**
 * Indicates whether the video message data requires transpose to match up with profile data.
 *
 * @public             @memberof GoVideoMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             kTRUE if transpose is required, kFALSE otherwise.
 */
GoFx(kBool) GoVideoMsg_IsTransposed(GoVideoMsg msg);

/**
 * @class   GoRangeMsg
 * @extends GoDataMsg
 * @note    Supported with G1
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of range data.
 */
typedef GoDataMsg GoRangeMsg;

/**
 * Gets the Range source.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Range source.
 */
GoFx(GoDataSource) GoRangeMsg_Source(GoRangeMsg msg);

/**
 * Gets the count of Range data in this message.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Count of Range data.
 */
GoFx(kSize) GoRangeMsg_Count(GoRangeMsg msg);


/**
 * Gets the Range z-resolution, in nanometers.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Z resolution (nm).
 */
GoFx(k32u) GoRangeMsg_ZResolution(GoRangeMsg msg);

/**
 * Gets the Range z-offset, in micrometers.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Z offset (um).
 */
GoFx(k32s) GoRangeMsg_ZOffset(GoRangeMsg msg);

/**
 * Gets a pointer to Range data.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Range array index.
 * @return             Range pointer.
 */
GoFx(k16s*) GoRangeMsg_At(GoRangeMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoRangeMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoRangeMsg_Exposure(GoRangeMsg msg);


/**
 * @class   GoRangeIntensityMsg
 * @extends GoDataMsg
 * @note    Supported with G1
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of range intensity data.
 */
typedef GoDataMsg GoRangeIntensityMsg;

/**
 * Gets the range intensity source.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Range intensity source.
 */
GoFx(GoDataSource) GoRangeIntensityMsg_Source(GoRangeIntensityMsg msg);

/**
 * Gets the count of range intensity data in this message.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Count of range intensity data.
 */
GoFx(kSize) GoRangeIntensityMsg_Count(GoRangeIntensityMsg msg);

/**
 * Gets a pointer to range intensity data.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Range intensity array index.
 * @return             Range intensity data pointer.
 */
GoFx(k8u*) GoRangeIntensityMsg_At(GoRangeIntensityMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoRangeIntensityMsg
 * @note               Supported with G1
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoRangeIntensityMsg_Exposure(GoRangeIntensityMsg msg);

/**
 * @class   GoProfilePointCloudMsg
 * @extends GoDataMsg
 * @note    Supported with G1, G2
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of profile arrays.
 */
typedef GoDataMsg GoProfilePointCloudMsg;

/**
 * Gets the profile source.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Profile source.
 */
GoFx(GoDataSource) GoProfilePointCloudMsg_Source(GoProfilePointCloudMsg msg);

/**
 * Gets the count of profile arrays in this message.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoProfilePointCloudMsg_Count(GoProfilePointCloudMsg msg);

/**
 * Gets the count of ranges in each profile array.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Range count.
 */
GoFx(kSize) GoProfilePointCloudMsg_Width(GoProfilePointCloudMsg msg);

/**
 * Gets the profile x-resolution, in nanometers.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoProfilePointCloudMsg_XResolution(GoProfilePointCloudMsg msg);

/**
 * Gets the profile z-resolution, in nanometers.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z resolution (nm).
 */
GoFx(k32u) GoProfilePointCloudMsg_ZResolution(GoProfilePointCloudMsg msg);

/**
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoProfilePointCloudMsg_XOffset(GoProfilePointCloudMsg msg);

/**
 * Gets the profile z-offset, in micrometers.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z offset (um).
 */
GoFx(k32s) GoProfilePointCloudMsg_ZOffset(GoProfilePointCloudMsg msg);

/**
 * Gets a pointer to a profile array.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @param   index      Profile array index.
 * @return             Profile pointer.
 */
GoFx(kPoint16s*) GoProfilePointCloudMsg_At(GoProfilePointCloudMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoProfilePointCloudMsg_Exposure(GoProfilePointCloudMsg msg);


/**
 * Gets the source camera index.
 *
 * @public             @memberof GoProfilePointCloudMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Camera index (0 - Front camera, 1 - Back camera).
 */
GoFx(k8u) GoProfilePointCloudMsg_CameraIndex(GoProfilePointCloudMsg msg);


/**
 * @class   GoUniformProfileMsg
 * @extends GoDataMsg
 * @note    Supported with G1, G2
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of re-sampled profile arrays.
 */
typedef GoDataMsg GoUniformProfileMsg;

/**
 * Gets the profile source.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Profile source.
 */
GoFx(GoDataSource) GoUniformProfileMsg_Source(GoUniformProfileMsg msg);

/**
 * Gets the count of re-sampled profile arrays in this message.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoUniformProfileMsg_Count(GoUniformProfileMsg msg);

/**
 * Gets the count of points in each re-sampled profile array.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Point count.
 */
GoFx(kSize) GoUniformProfileMsg_Width(GoUniformProfileMsg msg);

/**
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoUniformProfileMsg_XResolution(GoUniformProfileMsg msg);

/**
 * Gets the profile z-resolution, in nanometers.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z resolution (nm).
 */
GoFx(k32u) GoUniformProfileMsg_ZResolution(GoUniformProfileMsg msg);

/**
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoUniformProfileMsg_XOffset(GoUniformProfileMsg msg);

/**
 * Gets the profile z-offset, in micrometers.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z offset (um).
 */
GoFx(k32s) GoUniformProfileMsg_ZOffset(GoUniformProfileMsg msg);

/**
 * Gets a pointer to a re-sampled profile array.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @param   index      Profile array index.
 * @return             Data pointer.
 */
GoFx(k16s*) GoUniformProfileMsg_At(GoUniformProfileMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoUniformProfileMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Exposure in nS.
 */
GoFx(k32u) GoUniformProfileMsg_Exposure(GoUniformProfileMsg msg);

/**
 * @class   GoProfileIntensityMsg
 * @extends GoDataMsg
 * @note    Supported with G1, G2
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of profile intensity arrays.
 */
typedef GoDataMsg GoProfileIntensityMsg;

/**
 * Gets the profile source.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Profile source.
 */
GoFx(GoDataSource) GoProfileIntensityMsg_Source(GoProfileIntensityMsg msg);

/**
 * Gets the count of profile intensity arrays in this message.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Count of profile arrays.
 */
GoFx(kSize) GoProfileIntensityMsg_Count(GoProfileIntensityMsg msg);

/**
 * Gets the count of intensity values in each profile intensity array.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Intensity count.
 */
GoFx(kSize) GoProfileIntensityMsg_Width(GoProfileIntensityMsg msg);

/**
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoProfileIntensityMsg_XResolution(GoProfileIntensityMsg msg);

/**
 * Gets the profile x-offset, in micrometers.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoProfileIntensityMsg_XOffset(GoProfileIntensityMsg msg);

/**
 * Gets a pointer to a profile intensity array.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Profile intensity array index.
 * @return             Data pointer.
 */
GoFx(k8u*) GoProfileIntensityMsg_At(GoProfileIntensityMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoProfileIntensityMsg_Exposure(GoProfileIntensityMsg msg);

/**
 * Gets the source camera index.
 *
 * @public             @memberof GoProfileIntensityMsg
 * @note               Supported with G1, G2
 * @version            Introduced in firmware 4.1.3.106
 * @param   msg        Message object.
 * @return             Camera index (0 - Front camera, 1 - Back camera).
 */
GoFx(k8u) GoProfileIntensityMsg_CameraIndex(GoProfileIntensityMsg msg);

/**
 * @class   GoUniformSurfaceMsg
 * @extends GoDataMsg
 * @note    Supported with G2, G3
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a surface array.
 */
typedef GoDataMsg GoUniformSurfaceMsg;

/**
 * Gets the profile source.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Profile source.
 */
GoFx(GoDataSource) GoUniformSurfaceMsg_Source(GoUniformSurfaceMsg msg);

/**
 * Gets the length of the surface (row count).
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Surface length.
 */
GoFx(kSize) GoUniformSurfaceMsg_Length(GoUniformSurfaceMsg msg);

/**
 * Gets the width of the surface (column count).
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Surface width;
 */
GoFx(kSize) GoUniformSurfaceMsg_Width(GoUniformSurfaceMsg msg);

/**
 * Gets the surface x-resolution, in nanometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoUniformSurfaceMsg_XResolution(GoUniformSurfaceMsg msg);

/**
 * Gets the surface y-resolution, in nanometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Y resolution (nm).
 */
GoFx(k32u) GoUniformSurfaceMsg_YResolution(GoUniformSurfaceMsg msg);

/**
 * Gets the surface z-resolution, in nanometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z resolution (nm).
 */
GoFx(k32u) GoUniformSurfaceMsg_ZResolution(GoUniformSurfaceMsg msg);

/**
 * Gets the surface x-offset, in micrometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoUniformSurfaceMsg_XOffset(GoUniformSurfaceMsg msg);

/**
 * Gets the surface y-offset, in micrometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Y offset (um).
 */
GoFx(k32s) GoUniformSurfaceMsg_YOffset(GoUniformSurfaceMsg msg);

/**
 * Gets the surface z-offset, in micrometers.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Z offset (um).
 */
GoFx(k32s) GoUniformSurfaceMsg_ZOffset(GoUniformSurfaceMsg msg);

/**
 * Gets a pointer to a surface row.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @param   index      Surface row index.
 * @return             Row pointer.
 */
GoFx(k16s*) GoUniformSurfaceMsg_RowAt(GoUniformSurfaceMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoUniformSurfaceMsg
 * @note               Supported with G2, G3
 * @version            Introduced in firmware 4.8.1.70
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoUniformSurfaceMsg_Exposure(GoUniformSurfaceMsg msg);

/**
* @class   GoSurfacePointCloudMsg
* @extends GoDataMsg
* @note    Supported with G3
* @ingroup GoSdk-DataChannel
* @brief   Represents a data message containing a surface array.
*/
typedef GoDataMsg GoSurfacePointCloudMsg;

/**
* Gets the profile source.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Profile source.
*/
GoFx(GoDataSource) GoSurfacePointCloudMsg_Source(GoSurfacePointCloudMsg msg);

/**
* Gets the length of the surface (row count).
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Surface length.
*/
GoFx(kSize) GoSurfacePointCloudMsg_Length(GoSurfacePointCloudMsg msg);

/**
* Gets the width of the surface (column count).
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Surface width;
*/
GoFx(kSize) GoSurfacePointCloudMsg_Width(GoSurfacePointCloudMsg msg);

/**
* Gets the surface x-resolution, in nanometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             X resolution (nm).
*/
GoFx(k32u) GoSurfacePointCloudMsg_XResolution(GoSurfacePointCloudMsg msg);

/**
* Gets the surface y-resolution, in nanometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Y resolution (nm).
*/
GoFx(k32u) GoSurfacePointCloudMsg_YResolution(GoSurfacePointCloudMsg msg);

/**
* Gets the surface z-resolution, in nanometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Z resolution (nm).
*/
GoFx(k32u) GoSurfacePointCloudMsg_ZResolution(GoSurfacePointCloudMsg msg);

/**
* Gets the surface x-offset, in micrometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             X offset (um).
*/
GoFx(k32s) GoSurfacePointCloudMsg_XOffset(GoSurfacePointCloudMsg msg);

/**
* Gets the surface y-offset, in micrometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Y offset (um).
*/
GoFx(k32s) GoSurfacePointCloudMsg_YOffset(GoSurfacePointCloudMsg msg);

/**
* Gets the surface z-offset, in micrometers.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Z offset (um).
*/
GoFx(k32s) GoSurfacePointCloudMsg_ZOffset(GoSurfacePointCloudMsg msg);

/**
* Gets a pointer to a surface row.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @param   index      Surface row index.
* @return             Row pointer.
*/
GoFx(kPoint3d16s*) GoSurfacePointCloudMsg_RowAt(GoSurfacePointCloudMsg msg, kSize index);

/**
* Gets the exposure.
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.1.66
* @param   msg        Message object.
* @return             Exposure in uS.
*/
GoFx(k32u) GoSurfacePointCloudMsg_Exposure(GoSurfacePointCloudMsg msg);

/**
* Is the unresampled surface data adjacent/sorted?
*
* @public             @memberof GoSurfacePointCloudMsg
* @note               Supported with G3
* @version            Introduced in firmware 4.8.0.100
* @param   msg        Message object.
* @return             Is the data adjacent?
*/
GoFx(kBool) GoSurfacePointCloudMsg_IsAdjacent(GoSurfacePointCloudMsg msg);

/**
 * @class   GoSurfaceIntensityMsg
 * @extends GoDataMsg
 * @note    Supported with G3
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a surface intensity array.
 */
typedef GoDataMsg GoSurfaceIntensityMsg;

/**
 * Gets the profile source.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Profile source.
 */
GoFx(GoDataSource) GoSurfaceIntensityMsg_Source(GoSurfaceIntensityMsg msg);

/**
 * Gets the length of the surface (row count).
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Surface length.
 */
GoFx(kSize) GoSurfaceIntensityMsg_Length(GoSurfaceIntensityMsg msg);

/**
 * Gets the width of the surface (column count).
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Surface width;
 */
GoFx(kSize) GoSurfaceIntensityMsg_Width(GoSurfaceIntensityMsg msg);

/**
 * Gets the surface x-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoSurfaceIntensityMsg_XResolution(GoSurfaceIntensityMsg msg);

/**
 * Gets the surface y-resolution, in nanometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Y resolution (nm).
 */
GoFx(k32u) GoSurfaceIntensityMsg_YResolution(GoSurfaceIntensityMsg msg);

/**
 * Gets the surface x-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoSurfaceIntensityMsg_XOffset(GoSurfaceIntensityMsg msg);

/**
 * Gets the surface y-offset, in micrometers.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Y offset (um).
 */
GoFx(k32s) GoSurfaceIntensityMsg_YOffset(GoSurfaceIntensityMsg msg);

/**
 * Gets a pointer to a surface intensity row.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Surface intensity row index.
 * @return             Data pointer.
 */
GoFx(k8u*) GoSurfaceIntensityMsg_RowAt(GoSurfaceIntensityMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoSurfaceIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoSurfaceIntensityMsg_Exposure(GoSurfaceIntensityMsg msg);


/**
 * @class   GoSectionMsg
 * @extends GoDataMsg
 * @note    Supported with G3
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of section arrays.
 */
typedef GoDataMsg GoSectionMsg;

/**
 * Gets the section ID.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Section ID.
 */
GoFx(k32u) GoSectionMsg_Id(GoSectionMsg msg);

/**
 * Gets the section source.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Section source.
 */
GoFx(GoDataSource) GoSectionMsg_Source(GoSectionMsg msg);

/**
 * Gets the count of section arrays in this message.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Count of section arrays.
 */
GoFx(kSize) GoSectionMsg_Count(GoSectionMsg msg);

/**
 * Gets the count of points in each section array.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Point count.
 */
GoFx(kSize) GoSectionMsg_Width(GoSectionMsg msg);

/**
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoSectionMsg_XResolution(GoSectionMsg msg);

/**
 * Gets the z-resolution, in nanometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Z resolution (nm).
 */
GoFx(k32u) GoSectionMsg_ZResolution(GoSectionMsg msg);

/**
 * Gets the X-Pose, in micrometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X Pose (um).
 */
GoFx(k32s) GoSectionMsg_XPose(GoSectionMsg msg);

/**
 * Gets the Y-Pose, in micrometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Y Pose (um).
 */
GoFx(k32s) GoSectionMsg_YPose(GoSectionMsg msg);

/**
 * Gets the Pose Angle, in microdegrees.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Angle Pose (microdegrees).
 */
GoFx(k32s) GoSectionMsg_AnglePose(GoSectionMsg msg);

/**
 * Gets the x-offset, in micrometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoSectionMsg_XOffset(GoSectionMsg msg);

/**
 * Gets the z-offset, in micrometers.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Z offset (um).
 */
GoFx(k32s) GoSectionMsg_ZOffset(GoSectionMsg msg);

/**
 * Gets a pointer to a section array.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @param   index      Section array index.
 * @return             Data pointer.
 */
GoFx(k16s*) GoSectionMsg_At(GoSectionMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoSectionMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoSectionMsg_Exposure(GoSectionMsg msg);


/**
 * @class   GoSectionIntensityMsg
 * @extends GoDataMsg
 * @note    Supported with G3
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of profile intensity arrays.
 */
typedef GoDataMsg GoSectionIntensityMsg;

/**
 * Gets the section ID.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Section ID.
 */
GoFx(k32u) GoSectionIntensityMsg_Id(GoSectionIntensityMsg msg);

/**
 * Gets the section source.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Section source.
 */
GoFx(GoDataSource) GoSectionIntensityMsg_Source(GoSectionIntensityMsg msg);

/**
 * Gets the count of section arrays in this message.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Count of section arrays.
 */
GoFx(kSize) GoSectionIntensityMsg_Count(GoSectionIntensityMsg msg);

/**
 * Gets the count of points in each section array.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Point count.
 */
GoFx(kSize) GoSectionIntensityMsg_Width(GoSectionIntensityMsg msg);

/**
 * Gets the x-resolution, in nanometers.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X resolution (nm).
 */
GoFx(k32u) GoSectionIntensityMsg_XResolution(GoSectionIntensityMsg msg);

/**
 * Gets the X-Pose, in micrometers.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X Pose (um).
 */
GoFx(k32s) GoSectionIntensityMsg_XPose(GoSectionIntensityMsg msg);

/**
 * Gets the Y-Pose, in micrometers.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Y Pose (um).
 */
GoFx(k32s) GoSectionIntensityMsg_YPose(GoSectionIntensityMsg msg);

/**
 * Gets the Pose Angle, in microdegrees.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Angle Pose (microdegrees).
 */
GoFx(k32s) GoSectionIntensityMsg_AnglePose(GoSectionIntensityMsg msg);

/**
 * Gets the x-offset, in micrometers.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             X offset (um).
 */
GoFx(k32s) GoSectionIntensityMsg_XOffset(GoSectionIntensityMsg msg);

/**
 * Gets a pointer to a Section intensity array.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @param   index      Section intensity array index.
 * @return             Data pointer.
 */
GoFx(k8u*) GoSectionIntensityMsg_At(GoSectionIntensityMsg msg, kSize index);

/**
 * Gets the exposure.
 *
 * @public             @memberof GoSectionIntensityMsg
 * @note               Supported with G3
 * @version            Introduced in firmware 4.4.4.14
 * @param   msg        Message object.
 * @return             Exposure in uS.
 */
GoFx(k32u) GoSectionIntensityMsg_Exposure(GoSectionIntensityMsg msg);

/**
 * @struct  GoMeasurementData
 * @extends kValue
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a measurement result.
 */
typedef struct GoMeasurementData
{
    k64f value;                      ///< Measurement value.
    GoDecision decision;             ///< Measurement decision value.
    GoDecisionCode decisionCode;     ///< Measurement decision code - relevant only when the value represents a failure.
} GoMeasurementData;

/**
 * @class   GoMeasurementMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing a set of GoMeasurementData.
 */
typedef GoDataMsg GoMeasurementMsg;

/**
 * Gets the measurement identifier.
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Measurement identifier.
 */
GoFx(k16u) GoMeasurementMsg_Id(GoMeasurementMsg msg);

/**
 * Count of measurements in this message.
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Count of measurements.
 */
GoFx(kSize) GoMeasurementMsg_Count(GoMeasurementMsg msg);

/**
 * Gets the measurement at the specified index.
 *
 * @public             @memberof GoMeasurementMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @param   index      Measurement index.
 * @return             Measurement pointer.
 */
GoFx(GoMeasurementData*) GoMeasurementMsg_At(GoMeasurementMsg msg, kSize index);


/**
 * @class   GoAlignMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing an alignment result.
 */
typedef GoDataMsg GoAlignMsg;

/**
 * Gets the alignment result.
 *
 * @public             @memberof GoAlignMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Alignment result.
 */
GoFx(kStatus) GoAlignMsg_Status(GoAlignMsg msg);

/**
 * @class   GoExposureCalMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing exposure calibration results.
 */
typedef GoDataMsg GoExposureCalMsg;

/**
 * Gets the exposure calibration result.
 *
 * @public             @memberof GoExposureCalMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Exposure calibration result.
 */
GoFx(kStatus) GoExposureCalMsg_Status(GoExposureCalMsg msg);

/**
 * Gets the calibrated exposure.
 *
 * @public             @memberof GoExposureCalMsg
 * @version            Introduced in firmware 4.0.10.27
 * @param   msg        Message object.
 * @return             Calibrated exposure value in uS if the result is kOK.
 */
GoFx(k64f) GoExposureCalMsg_Exposure(GoExposureCalMsg msg);


/**
 * @class   GoEdgeMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing edge based part matching results.
 */
typedef GoDataMsg GoEdgeMatchMsg;

/**
 * Gets the edge match decision.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match decision.
 */
GoFx(k8u) GoEdgeMatchMsg_Decision(GoEdgeMatchMsg msg);

/**
 * Gets the edge match X offset.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match X offset.
 */
GoFx(k64f) GoEdgeMatchMsg_XOffset(GoEdgeMatchMsg msg);

/**
 * Gets the edge match Y offset.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match Y offset.
 */
GoFx(k64f) GoEdgeMatchMsg_YOffset(GoEdgeMatchMsg msg);

/**
 * Gets the edge match Z angle.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match Z angle.
 */
GoFx(k64f) GoEdgeMatchMsg_ZAngle(GoEdgeMatchMsg msg);

/**
 * Gets the edge match quality value.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match quality value.
 */
GoFx(k64f) GoEdgeMatchMsg_QualityValue(GoEdgeMatchMsg msg);

/**
 * Gets the edge match quality decision.
 *
 * @public             @memberof GoEdgeMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Edge match quality decision.
 */
GoFx(k8u) GoEdgeMatchMsg_QualityDecision(GoEdgeMatchMsg msg);


/**
 * @class   GoEllipseMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing ellipse based part matching results.
 */
typedef GoDataMsg GoEllipseMatchMsg;

/**
 * Gets the ellipse match decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match quality decision.
 */
GoFx(k8u) GoEllipseMatchMsg_Decision(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match X offset.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match X offset.
 */
GoFx(k64f) GoEllipseMatchMsg_XOffset(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match Y offset.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match Y offset.
 */
GoFx(k64f) GoEllipseMatchMsg_YOffset(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match Z angle.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match Z angle.
 */
GoFx(k64f) GoEllipseMatchMsg_ZAngle(GoEllipseMatchMsg msg);

/**
* Gets the ellipse match minor value.
*
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match minor value.
 */
GoFx(k64f) GoEllipseMatchMsg_MinorValue(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match minor decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match minor decision.
 */
GoFx(k8u) GoEllipseMatchMsg_MinorDecision(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match major value.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match major value.
 */
GoFx(k64f) GoEllipseMatchMsg_MajorValue(GoEllipseMatchMsg msg);

/**
 * Gets the ellipse match major decision.
 *
 * @public             @memberof GoEllipseMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Ellipse match major decision.
 */
GoFx(k8u) GoEllipseMatchMsg_MajorDecision(GoEllipseMatchMsg msg);


/**
 * @class   GoBoundingBoxMatchMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing bounding box based part matching results.
 */
typedef GoDataMsg GoBoundingBoxMatchMsg;

/**
 * Gets the bounding box match major value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match major value.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_Decision(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match X offset.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match X offset.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_XOffset(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match Y offset.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match Y offset.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_YOffset(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match Z angle.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match Z angle.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_ZAngle(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match length value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match length value.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_LengthValue(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match length decision.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match length decision.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_LengthDecision(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match width value.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match width value.
 */
GoFx(k64f) GoBoundingBoxMatchMsg_WidthValue(GoBoundingBoxMatchMsg msg);

/**
 * Gets the bounding box match width decision.
 *
 * @public             @memberof GoBoundingBoxMatchMsg
 * @version             Introduced in firmware 4.2.4.7
 * @param   msg        Message object.
 * @return             Bounding box match width decision.
 */
GoFx(k8u) GoBoundingBoxMatchMsg_WidthDecision(GoBoundingBoxMatchMsg msg);


/**
 * @class   GoEventMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a message containing an event (See GoEventType).
 */
typedef GoDataMsg GoEventMsg;

/**
 * Gets the event type.
 *
 * @public             @memberof GoEventMsg
 * @version            Introduced in firmware 4.5.3.57
 * @param   msg        Message object.
 * @return             Event type.
 */
GoFx(GoEventType) GoEventMsg_Type(GoEventMsg msg);


/**
 * @class   GoTracheidMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing a set of tracheid data.
 */
typedef GoDataMsg GoTracheidMsg;

/**
* @struct  GoTracheidEllipse
* @extends kValue
* @ingroup GoSdk-DataChannel
* @brief   Represents a tracheid ellipse.
*/
typedef struct GoTracheidEllipse
{
    k64f area;                      ///< Ellipse area
    k64f angle;                     ///< Ellipse angle
    k64f scatter;                   ///< Ellipse scatter
    k64f minor;                     ///< Ellipse minor
    k64f major;                     ///< Ellipse major

} GoTracheidEllipse;

/**
 * Gets the tracheid source.
 *
 * @public             @memberof GoTracheidMsg
 * @version            Introduced in firmware 4.5.3.57
 * @param   msg        Message object.
 * @return             Tracheid source.
 */
GoFx(GoDataSource) GoTracheidMsg_Source(GoTracheidMsg msg);

/**
 * Gets the count of tracheid arrays in this message.
 *
 * @public             @memberof GoTracheidMsg
 * @version             Introduced in firmware 4.6.4.66
 * @param   msg        Message object.
 * @return             Count of tracheid arrays.
 */
GoFx(kSize) GoTracheidMsg_Count(GoTracheidMsg msg);

/**
* Gets the count of ellipses in each tracheid array.
*
* @public             @memberof GoTracheidMsg
 * @version             Introduced in firmware 4.6.4.66
* @param   msg        Message object.
* @return             Ellipse count.
*/
GoFx(kSize) GoTracheidMsg_Width(GoTracheidMsg msg);

/**
 * Gets the tracheid camera index.
 *
 * @public             @memberof GoTracheidMsg
 * @version            Introduced in firmware 4.5.3.57
 * @param   msg        Message object.
 * @return             Tracheid camera index.
 */
GoFx(k8u) GoTracheidMsg_CameraIndex(GoTracheidMsg msg);

/**
 * Gets a pointer to a tracheid ellipse array.
 *
 * @public             @memberof GoTracheidMsg
 * @version             Introduced in firmware 4.6.4.66
 * @param   msg        Message object.
 * @param   index      Tracheid ellipse array index.
 * @return             Tracheid ellipse array.
 */
GoFx(GoTracheidEllipse*) GoTracheidMsg_At(GoTracheidMsg msg, kSize index);

/**
* @class   GoPointFeatureMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Represents a message containing Point Feature data.
*/
typedef GoDataMsg GoPointFeatureMsg;

/**
* Gets the point feature identifier.
*
* @public             @memberof GoPointFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Point Feature identifier.
*/
GoFx(k16u) GoPointFeatureMsg_Id(GoPointFeatureMsg msg);

/**
* The point data in this message.
*
* @public             @memberof GoPointFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Corrdinates of point feature in nanometers.
*/
GoFx(kPoint3d64f) GoPointFeatureMsg_Position(GoPointFeatureMsg msg);

/**
* @class   GoLineFeatureMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Represents a message containing Linear Feature data.
*/
typedef GoDataMsg GoLineFeatureMsg;

/**
* Gets the linear feature identifier.
*
* @public             @memberof GoLineFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Line Feature identifier.
*/
GoFx(k16u) GoLineFeatureMsg_Id(GoLineFeatureMsg msg);

/**
* Gets a point on the linear feature.
*
* @public             @memberof GoLineFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Position of the linear feature in nanometers.
*/
GoFx(kPoint3d64f) GoLineFeatureMsg_Position(GoLineFeatureMsg msg);

/**
* The direction vector of the linear feature.
*
* @public             @memberof GoLineFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Direction of the linear feature.
*/
GoFx(kPoint3d64f) GoLineFeatureMsg_Direction(GoLineFeatureMsg msg);

/**
* @class   GoPlaneFeatureMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Represents a message containing Planear Feature data.
*/
typedef GoDataMsg GoPlaneFeatureMsg;

/**
* Gets the planear feature identifier.
*
* @public             @memberof GoPlaneFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Plane Feature identifier.
*/
GoFx(k16u) GoPlaneFeatureMsg_Id(GoPlaneFeatureMsg msg);

/**
* Gets the normal vector of the planear feature.
*
* @public             @memberof GoPlaneFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             The vector normal to the plane.
*/
GoFx(kPoint3d64f) GoPlaneFeatureMsg_Normal(GoPlaneFeatureMsg msg);

/**
* Gets shortest distance from the origin to the plane. Is parallel to the normal vector.
*
* @public             @memberof GoPlaneFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Shortest distance to the origin in nanometers.
*/
GoFx(k64f) GoPlaneFeatureMsg_DistanceToOrigin(GoPlaneFeatureMsg msg);

/**
* @class   GoCircleFeatureMsg
* @extends GoDataMsg
* @ingroup GoSdk-DataChannel
* @brief   Represents a message containing circular feature data.
*/
typedef GoDataMsg GoCircleFeatureMsg;

/**
* Gets the circular feature identifier.
*
* @public             @memberof GoCircleFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Circle Feature identifier.
*/
GoFx(k16u) GoCircleFeatureMsg_Id(GoCircleFeatureMsg msg);

/**
* Gets the position of the center of the circular feature.
*
* @public             @memberof GoCircleFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Position of the circular feature in nanometers
*/
GoFx(kPoint3d64f) GoCircleFeatureMsg_Position(GoCircleFeatureMsg msg);

/**
* Gets the normal vector of the circular feature.
*
* @public             @memberof GoCircleFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             The vector normal to the plane.
*/
GoFx(kPoint3d64f) GoCircleFeatureMsg_Normal(GoCircleFeatureMsg msg);

/**
* Gets the radius of the circular feature.
*
* @public             @memberof GoCircleFeatureMsg
* @version            Introduced in firmware 4.6.3.43
* @param   msg        Message object.
* @return             Radius of the circle in nanometers.
*/
GoFx(k64f) GoCircleFeatureMsg_Radius(GoCircleFeatureMsg msg);

/**
 * @class   GoGenericMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing generic data.
 *
 * A generic message may contain either a raw byte buffer, or a kObject.
 * If the content is a kObject, then the buffer contains the byte stream containing
 * the serialized object, using the standard serialization schem (kDat6).
 */
typedef GoDataMsg GoGenericMsg;

/**
* Gets the user defined data type.
*
* @public             @memberof GoGenericMsg
* @version            Introduced in firmware 5.2.18.3
* @param   msg        Message object.
* @return             Type ID.
*/
GoFx(k32u) GoGenericMsg_UserType(GoGenericMsg msg);

/**
 * Returns whether or not the content is an kObject. If false,
 * the content is a raw byte buffer.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 5.2.18.3
 * @param   msg        Message object.
 * @return             kTRUE if content is kObject. kFALSE if raw buffer.
 */
GoFx(kBool) GoGenericMsg_IsObject(GoGenericMsg msg);

/**
 * Returns the size of the raw buffer.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 5.2.18.3
 * @param   msg        Message object.
 * @return             Size of the buffer in bytes.
 */
GoFx(kSize) GoGenericMsg_BufferSize(GoGenericMsg msg);

/**
 * Returns a pointer to the raw buffer.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 5.2.18.3
 * @param   msg        Message object.
 * @return             Address of the raw buffer.
 */
GoFx(const void*) GoGenericMsg_BufferData(GoGenericMsg msg);

/**
 * Returns the kObject content, if available.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 5.2.18.3
 * @param   msg        Message object.
 * @return             Content object. kNULL if the content is a raw
 *                     buffer or if the data cannot be deserialized.
 */
GoFx(kObject) GoGenericMsg_Object(GoGenericMsg msg);

/**
* Returns the serializer status after deserializing kObject content.
*
* Note that deserialization does not occur until GoGenericMsg_Object is
* called for the first time.
*
* @public             @memberof GoGenericMsg
* @version            Introduced in firmware 5.2.18.3
* @param   msg        Message object.
* @return             Serializer status. kOK if not applicable.
*/
GoFx(kStatus) GoGenericMsg_SerializerStatus(GoGenericMsg msg);

/* Internal Use Only*/

/**
 * @class   GoNullMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing null data.
 *
 * This is a reserved class and currently only for internal use.
 * A Null message is used to explicitly inform that there is no valid result
 * e.g. when a measurement fails to find a feature
 */
typedef GoDataMsg GoNullMsg;

/**
* Gets the error status related to the null message.
*
* @public             @memberof GoNullMsg
* @version            Introduced in firmware 6.0.0.0 // subject to change
* @param   msg        Message object.
* @return             Error status.
*/
GoFx(kStatus) GoNullMsg_Status(GoNullMsg msg);

/**
 * @class   GoMeshMsg
 * @extends GoDataMsg
 * @ingroup GoSdk-DataChannel
 * @brief   Represents a data message containing mesh data.
 */
typedef GoDataMsg GoMeshMsg;

/**
 * Gets the source.
 *
 * @public             @memberof GoStampMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             A GoDataSource value.
 */
GoFx(GoDataSource) GoMeshMsg_Source(GoMeshMsg msg);

/**
* Gets the source of the mesh data stream.
*
* @public             @memberof GoMeshMsg
* @version            Introduced in firmware 6.0.18.7
* @param   msg        Message object.
* @return             A GoDataStep value.
*/
GoFx(GoDataStep) GoMeshMsg_StreamStep(GoMeshMsg msg);

/**
* Gets the identifier of the mesh data stream from the source.
*
* @public             @memberof GoMeshMsg
* @version            Introduced in firmware 6.0.18.7
* @param   msg        Message object.
* @return             Stream step identifier number.
*/
GoFx(k32s) GoMeshMsg_StreamStepId(GoMeshMsg msg);

/**
 * Gets hasData.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Whether any buffer has been allocated for any channel.
 */
GoFx(kBool) GoMeshMsg_HasData(GoMeshMsg msg);

/**
 * Gets offset.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Offset parameter.
 */
GoFx(kPoint3d64f) GoMeshMsg_Offset(GoMeshMsg msg);

/**
 * Gets range.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Range parameter.
 */
GoFx(kPoint3d64f) GoMeshMsg_Range(GoMeshMsg msg);

/**
 * Gets transform.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Transform parameter.
 */
GoFx(Go3dTransform64f) GoMeshMsg_Transform(GoMeshMsg msg);

/**
 * Gets total channel count.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Number of channels. This include both of all system channels and any additional user channel.
 */
GoFx(kSize) GoMeshMsg_ChannelCount(GoMeshMsg msg);

/**
 * Gets maximum user channel count.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Maximum number of user channel that can be used.
 */
GoFx(kSize) GoMeshMsg_MaxUserChannelCount(GoMeshMsg msg);

/**
 * Gets user channel count.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @return             Number of user channel used.
 */
GoFx(kSize) GoMeshMsg_UserChannelCount(GoMeshMsg msg);

/**
 * Gets channel type.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             Type of channel.
 */
GoFx(GoMeshMsgChannelType) GoMeshMsg_ChannelType(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel state.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             State of channel.
 */
GoFx(GoMeshMsgChannelState) GoMeshMsg_ChannelState(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel flag.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             Flag of channel. Flag is typcically an user defined field that allows user to customize its usage.
 */
GoFx(k32u) GoMeshMsg_ChannelFlag(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel allocated data count.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             Number of allocated elements within channel data buffer.
 */
GoFx(kSize) GoMeshMsg_AllocatedChannelDataCount(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel used data count.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             Number of actual used elements within channel data buffer.
 */
GoFx(kSize) GoMeshMsg_UsedChannelDataCount(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel data type.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             kType of each of element within channel data. Note that for any user channel, data type is always k8U.
 */
GoFx(kType) GoMeshMsg_ChannelDataType(GoMeshMsg msg, GoMeshMsgChannelId id);

/**
 * Gets channel data buffer.
 *
 * @public             @memberof GoGenericMsg
 * @version            Introduced in firmware 6.0.18.7
 * @param   msg        Message object.
 * @param   id         Channel ID.
 * @return             Channel data buffer.
 */
GoFx(kArray1) GoMeshMsg_ChannelData(GoMeshMsg msg, GoMeshMsgChannelId id);

#include <GoSdk/Messages/GoDataTypes.x.h>

#endif
