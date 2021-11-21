// 
// GoSetup.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SETUP_H
#define GO_SDK_NET_SETUP_H

#include <GoSdk/GoSetup.h>
#include <GoSdkNet/GoLayout.h>
#include <GoSdkNet/GoSurfaceGeneration.h>
#include <GoSdkNet/GoProfileGeneration.h>
#include <GoSdkNet/GoPartDetection.h>
#include <GoSdkNet/GoPartMatching.h>
#include <GoSdkNet/GoAdvanced.h>
#include <GoSdkNet/GoMaterial.h>
#include <GoSdkNet/GoSections.h>
#include <GoSdkNet/GoSdkDef.h>
#include <GoSdkNet/GoTracheid.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents a device configuration.</summary>
        public ref class GoSetup : public KObject
        {
            KDeclareClass(GoSetup, GoSetup)

            /// <summary>Initializes a new instance of the GoSetup class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSetup(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSetup class.</summary>
            GoSetup(GoSensor^ sensor)
            {
                ::GoSetup handle = kNULL;

                KCheck(::GoSetup_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoSetup(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoSetup(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoSetup handle = kNULL;

                KCheck(::GoSetup_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>Whether grid seup is used in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetLayoutGridUsed(GoRole role)
            {
                return KToBool(::GoSetup_LayoutGridUsed(Handle, role));
            }

            /// <summary>Device direction system value being used on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetLayoutGridDirectionSystemValue(GoRole role)
            {
                return KToBool(::GoSetup_LayoutGridDirectionSystemValue(Handle, role));
            }

            /// <summary>Device direction being configured (may not be applicable or used) on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetLayoutGridDirection(GoRole role)
            {
                return KToBool(::GoSetup_LayoutGridDirection(Handle, role));
            }

            /// <summary>Set device direction in n-buddy system.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="direction">Whether direction is reversed.</param>
            void SetLayoutGridDirection(GoRole role, kBool direction)  
            { 
                KCheck(::GoSetup_SetLayoutGridDirection(Handle, role, direction)); 
            }

            /// <summary>Device column index system value being used on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetLayoutGridColumnSystemValue(GoRole role)
            {
                return ::GoSetup_LayoutGridColumnSystemValue(Handle, role);
            }

            /// <summary>Device column index being configured (may not be applicable or used) on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetLayoutGridColumn(GoRole role)
            {
                return ::GoSetup_LayoutGridColumn(Handle, role);
            }

            /// <summary>Set device column index in n-buddy system.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="direction">Device column index.</param>
            void SetLayoutGridColumn(GoRole role, k32s col)  
            { 
                KCheck(::GoSetup_SetLayoutGridColumn(Handle, role, col)); 
            }
            
            /// <summary>Device row index system value being used on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetLayoutGridRowSystemValue(GoRole role)
            {
                return ::GoSetup_LayoutGridRowSystemValue(Handle, role);
            }

            /// <summary>Device row index being configured (may not be applicable or used) on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetLayoutGridRow(GoRole role)
            {
                return ::GoSetup_LayoutGridRow(Handle, role);
            }

            /// <summary>Set device row index in n-buddy system.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="direction">Device row index.</param>
            void SetLayoutGridRow(GoRole role, k32s row)  
            { 
                KCheck(::GoSetup_SetLayoutGridRow(Handle, role, row)); 
            }

            /// <summary>Device multiplexing bank in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetMultiplexingBankUsed(GoRole role)
            {
                return KToBool(::GoSetup_LayoutMultiplexingBankUsed(Handle, role));
            }

            /// <summary>Device multiplexing bank system value being used on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetMultiplexingBankSystemValue(GoRole role)
            {
                return ::GoSetup_LayoutMultiplexingBankSystemValue(Handle, role);
            }

            /// <summary>Device multiplexing bank value being configured (may not be applicable or used) on device in n-buddy system.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32s GetMultiplexingBank(GoRole role)
            {
                return ::GoSetup_LayoutMultiplexingBank(Handle, role);
            }

            /// <summary>Set device multiplexing bank value in n-buddy system.</summary>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Multiplexing bank value.</param>
            void SetMultiplexingBank(GoRole role, k32s value)  
            { 
                KCheck(::GoSetup_SetLayoutMultiplexingBank(Handle, role, value)); 
            }

            /// <summary>The default operation mode of the sensor system.</summary>
            property GoMode ScanMode
            {
                GoMode get()           { return (GoMode) ::GoSetup_ScanMode(Handle); }
                void set(GoMode scanMode)  { KCheck(::GoSetup_SetScanMode(Handle, scanMode)); }
            }

            /// <summary>Gets the scan mode option at the specified index.</summary>
            ///
            /// <param name="index">The index with which to retrieve a mode option.</param>
            GoMode GetScanModeOption(k64s index)
            {
                return (GoMode) ::GoSetup_ScanModeOptionAt(Handle, (kSize)index);
            }

            /// <summary>The scan mode option count.</summary>
            property k64s ScanModeOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_ScanModeOptionCount(Handle); }
            }

            /// <summary>The user specified Uniform Spacing enabled state.</summary>
            property bool UniformSpacingEnabled
            {
                bool get()           { return KToBool(::GoSetup_UniformSpacingEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableUniformSpacing(Handle, enabled)); }
            }

            /// <summary>A boolean representing whether or not the user specified Uniform Spacing setting is being used at the moment.</summary>
            property bool UniformSpacingAvailable
            {
                bool get()           { return KToBool(::GoSetup_UniformSpacingAvailable(Handle)); }
            }

            /// <summary>The Uniform Spacing enabled system value.</summary>
            property bool UniformSpacingEnabledSystemValue
            {
                bool get()           { return KToBool(::GoSetup_UniformSpacingEnabledSystemValue(Handle)); }
            }

            /// <summary>The state of the external input triggered encoder Z-pulse feature.</summary>
            property bool ExternalInputZPulseEnabled
            {
                bool get()           { return KToBool(::GoSetup_ExternalInputZPulseEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableExternalInputZPulse(Handle, enabled)); }
            }
            
            /// <summary>The external input index for triggering encoder Z-pulse.</summary>
            property k32u ExternalInputZPulseIndex
            {
                k32u get()          { return ::GoSetup_ExternalInputZPulseIndex(Handle); }
                void set(k32u val)  { KCheck(::GoSetup_SetExternalInputZPulseIndex(Handle, val)); }
            }

            /// <summary>If the external input index for triggering encoder Z-pulse is used.</summary>
            property bool ExternalInputZPulseIndexAvailable
            {
                bool get()          { return KToBool(::GoSetup_ExternalInputZPulseIndexAvailable(Handle)); }
            }

            /// <summary>The occlusion reduction enabled state.</summary>
            property bool OcclusionReductionEnabled
            {
                bool get()           { return KToBool(::GoSetup_OcclusionReductionEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableOcclusionReduction(Handle, enabled)); }
            }

            /// <summary>A boolean representing whether the user occlusion reduction configuration is used.</summary>
            property bool OcclusionReductionEnabledUsed
            {
                bool get()           { return KToBool(::GoSetup_OcclusionReductionEnabledUsed(Handle)); }
            }

            /// <summary>The occlusion reduction system value.</summary>
            property bool OcclusionReductionEnabledSystemValue
            {
                bool get()           { return KToBool(::GoSetup_OcclusionReductionEnabledSystemValue(Handle)); }
            }

            /// <summary>The occlusion reduction algorithm.</summary>
            property GoOcclusionReductionAlg OcclusionReductionAlg
            {
                GoOcclusionReductionAlg get()           { return (GoOcclusionReductionAlg) ::GoSetup_OcclusionReductionAlg(Handle); }
                void set(GoOcclusionReductionAlg alg)  { KCheck(::GoSetup_SetOcclusionReductionAlg(Handle, alg)); }
            }

            /// <summary>A boolean representing whether the user occlusion reduction algorithm is used.</summary>
            property bool OcclusionReductionAlgUsed
            {
                bool get()           { return KToBool(::GoSetup_OcclusionReductionAlgUsed(Handle)); }
            }

            /// <summary>A boolean representing whether flicker reduction mode is enabled.</summary>
            property bool FlickerReductionModeEnabled
            {
                bool get()                      { return KToBool(::GoSetup_FlickerFreeModeEnabled(Handle)); }
                void set(bool enabled)          { KCheck(::GoSetup_EnableFlickerFreeMode(Handle, enabled)); }
            }
            /// <summary>A boolean representing whether the flicker reduction mode is available for use.</summary>
            property bool FlickerReductionModeAvailable
            {
                bool get()                      { return KToBool(::GoSetup_FlickerFreeModeAvailable(Handle)); }
            }

            /// <summary>The system trigger units.</summary>
            /// 
            /// <remarks>Ignored if GoSetup_TriggerSource is time or encoder</remarks>
            property GoTriggerUnits TriggerUnit
            {
                GoTriggerUnits get()           { return (GoTriggerUnits) ::GoSetup_TriggerUnit(Handle); }
                void set(GoTriggerUnits unit)  { KCheck(::GoSetup_SetTriggerUnit(Handle, unit)); }
            }

            /// <summary>The trigger source for profile triggering.</summary>
            property GoTrigger TriggerSource
            {
                GoTrigger get()           { return (GoTrigger) ::GoSetup_TriggerSource(Handle); }
                void set(GoTrigger source)  { KCheck(::GoSetup_SetTriggerSource(Handle, source)); }
            }

            /// <summary>The trigger source option count.</summary>
            property k64s TriggerSourceOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_TriggerSourceOptionCount(Handle); }
            }

            /// <summary>Gets the trigger source option at the given index.</summary>
            ///
            /// <param name="index">The index with which to retrieve a trigger source.</param>
            GoTrigger GetTriggerSourceOptionAt(k64s index)
            {
                return (GoTrigger) ::GoSetup_TriggerSourceOptionAt(Handle, (kSize)index);
            }

           /// <summary>The external input to use for profile triggering - only applicable if trigger source is DigitalInput.</summary>
            property k32s TriggerExternalInputIndex
            {
                k32s get()           { return (GoTrigger) ::GoSetup_TriggerExternalInputIndex(Handle); }
                void set(k32s source)  { KCheck(::GoSetup_SetTriggerExternalInputIndex(Handle, source)); }
            }

            /// <summary>Returns a boolean value representing whether the external input trigger index is used.</summary>
            property bool TriggerExternalInputIndexUsed
            {
                bool get()           { return KToBool(::GoSetup_TriggerExternalInputIndexUsed(Handle)); }
            }

            /// <summary>The trigger external input index option count.</summary>
            property k64s TriggerExternalInputIndexOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_TriggerExternalInputIndexOptionCount(Handle); }
            }

            /// <summary>Gets the trigger external input index option at the given index.</summary>
            ///
            /// <param name="index">The index with which to retrieve a external input index.</param>
            GoTrigger GetTriggerExternalInputIndexOptionAt(k64s index)
            {
                return (GoTrigger) ::GoSetup_TriggerExternalInputIndexOptionAt(Handle, (kSize)index);
            }

            /// <summary>The trigger delay. Depending on GoDomain, units are uS or mm.</summary>
            property k64f TriggerDelay
            {
                k64f get()           { return ::GoSetup_TriggerDelay(Handle); }
                void set(k64f delay)  { KCheck(::GoSetup_SetTriggerDelay(Handle, delay)); }
            }

            /// <summary>The minimum trigger delay, based on current settings. Depending on GoDomain, units are uS or mm.</summary>
            property k64f TriggerDelayLimitMin
            {
                k64f get()           { return ::GoSetup_TriggerDelayLimitMin(Handle); }
            }

            /// <summary>The maximum trigger delay, based on current settings. Depending on GoDomain, units are uS or mm.</summary>
            property k64f TriggerDelayLimitMax
            {
                k64f get()           { return ::GoSetup_TriggerDelayLimitMax(Handle); }
            }

            /// <summary>The trigger gate feature.</summary>
            property bool TriggerGateEnabled
            {
                bool get()           { return KToBool(::GoSetup_TriggerGateEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableTriggerGate(Handle, enabled)); }
            }

            /// <summary>The system value representing whether or not the user specified trigger gate enabled setting is being used at the moment.</summary>
            property bool TriggerGateEnabledUsed
            {
                bool get()           { return KToBool(::GoSetup_TriggerGateEnabledUsed(Handle)); }
            }

            /// <summary>The trigger gate enabled system value.</summary>
            property bool TriggerGateEnabledSystemValue
            {
                bool get()           { return KToBool(::GoSetup_TriggerGateEnabledSystemValue(Handle)); }
            }

            /// <summary>Enables or disables operation at full frame rate (ignoring frame rate setting).</summary>
            property bool MaxFrameRateEnabled
            {
                bool get()           { return KToBool(::GoSetup_MaxFrameRateEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableMaxFrameRate(Handle, enabled)); }
            }

            /// <summary>Enables or disables burst triggering.</summary>
            property bool TriggerBurstEnabled
            {
                bool get()           { return KToBool(::GoSetup_TriggerBurstEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_SetTriggerBurstEnabled(Handle, enabled)); }
            }

            /// <summary>Whether or not burst triggering can be enabled.</summary>
            property bool TriggerBurstEnabledUsed
            {
                bool get()           { return KToBool(::GoSetup_TriggerBurstEnabledUsed(Handle)); }
            }

            /// <summary>Gets or sets the burst count for the burst triggering feature.</summary>
            property k64s TriggerBurstCount
            {
                k64s get()           { return KSizeTo64s(::GoSetup_TriggerBurstCount(Handle)); }
                void set(k64s count)    { KCheck(::GoSetup_SetTriggerBurstCount(Handle, (k32s)count)); }
            }

            /// <summary>Whether or not the trigger burst count can be set.</summary>
            property bool TriggerBurstCountUsed
            {
                bool get()           { return KToBool(::GoSetup_TriggerBurstCountUsed(Handle)); }
            }

            /// <summary>Sets the laser sleep mode and timeout and threshold values to activate/deactivate</summary>
            /// 
            /// <summary>The laser sleep mode used value</summary>
            property bool LaserSleepUsed
            {
                bool get() { return KToBool(::GoSetup_LaserSleepUsed(Handle)); }
            }

            /// <summary>The laser sleep mode enable system value</summary>
            property bool LaserSleepEnable
            {
                bool get() { return KToBool(::GoSetup_LaserSleepModeEnabled(Handle)); }
                void set(bool enable) { KCheck(::GoSetup_SetLaserSleepModeEnabled(Handle, enable)); }
            }

            /// <summary>The idle time before the laser sleeps.</summary>
            property k64u LaserSleepIdleTime
            {
                k64u get() { return (k64u)::GoSetup_LaserIdleTime(Handle); }
                void set(k64u time) { KCheck(::GoSetup_SetLaserIdleTime(Handle, (k64u)time)); }
            }

            /// <summary>The trigger gate enabled system value.</summary>
            property k64u LaserSleepWakeupEncoderTravel
            {
                k64u get() { return (k64u)::GoSetup_LaserWakeupEncoderTravel(Handle); }
                void set(k64u distance) { KCheck(::GoSetup_SetLaserWakeupEncoderTravel(Handle, (k64u)distance)); }
            }

            /// <summary>Sets the current frame rate for time-based triggering.</summary>
            /// 
            /// <remarks>
            /// The maximum frame rate option must be disabled to use the value set in this function.
            /// </remarks>
            property k64f FrameRate
            {
                k64f get()           { return ::GoSetup_FrameRate(Handle); }
                void set(k64f frameRate)  { KCheck(::GoSetup_SetFrameRate(Handle, frameRate)); }
            }

            /// <summary>Gets the current Tracheid frame rate </summary
            property k64f TracheidRate
            {
                k64f get() { return ::GoSetup_TracheidRate(Handle); }
            }

            /// <summary>Gets the current data (range/profile/surface) frame rate </summary
            property k64f FrameDataRate
            {
                k64f get() { return ::GoSetup_FrameDataRate(Handle); }
            }

            /// <summary>Constraint for the minimum valid value of the Frame Rate setting.</summary>
            property k64f FrameRateLimitMin
            {
                k64f get()           { return ::GoSetup_FrameRateLimitMin(Handle); }
            }

            /// <summary>Constraint for the maximum valid value of the Frame Rate setting.</summary>
            property k64f FrameRateLimitMax
            {
                k64f get()           { return ::GoSetup_FrameRateLimitMax(Handle); }
            }

            /// <summary>The encoder trigger mode.</summary>
            property GoEncoderTriggerMode EncoderTriggerMode
            {
                GoEncoderTriggerMode get()           { return (GoEncoderTriggerMode) ::GoSetup_EncoderTriggerMode(Handle); }
                void set(GoEncoderTriggerMode mode)  { KCheck(::GoSetup_SetEncoderTriggerMode(Handle, mode)); }
            }

            /// <summary>The current encoder period for encoder-based triggering.</summary>
            property k64f EncoderSpacing
            {
                k64f get()           { return ::GoSetup_EncoderSpacing(Handle); }
                void set(k64f period)  { KCheck(::GoSetup_SetEncoderSpacing(Handle, period)); }
            }

            /// <summary>Constraint for the minimum valid value of the Encoder Period setting..</summary>
            property k64f EncoderSpacingLimitMin
            {
                k64f get()           { return ::GoSetup_EncoderSpacingLimitMin(Handle); }
            }

            /// <summary>Constraint for the maximum valid value of the Encoder Period setting.</summary>
            property k64f EncoderSpacingLimitMax
            {
                k64f get()           { return ::GoSetup_EncoderSpacingLimitMax(Handle); }
            }

            /// <summary>Indicates whether or not auto encoder reversal distance can be enabled or disabled.</summary>
            property bool ReversalDistanceAutoEnabledUsed
            {
                bool get() { return KToBool(::GoSetup_ReversalDistanceAutoEnabledUsed(Handle)); }
            }

            /// <summary>Auto encoder reversal distance enabled status.</summary>
            property bool ReversalDistanceAutoEnabled
            {
                bool get() { return KToBool(::GoSetup_ReversalDistanceAutoEnabled(Handle)); }
                void set(bool enabled) { KCheck(::GoSetup_SetReversalDistanceAutoEnabled(Handle, (bool)enabled)); }
            }

            /// <summary>A boolean representing whether the encoder reversal distance threshold is used.</summary>
            property bool ReversalDistanceUsed
            {
                bool get() { return KToBool(::GoSetup_ReversalDistanceUsed(Handle)); }
            }

            /// <summary>The current system value of the encoder reversal distance threshold.</summary>
            property k64f ReversalDistanceSystemValue
            {
                k64f get() { return ::GoSetup_ReversalDistanceSystemValue(Handle); }
            }

            /// <summary>The encoder reversal distance threshold.</summary>
            property k64f ReversalDistance
            {
                k64f get() { return ::GoSetup_ReversalDistance(Handle); }
                void set(k64f threshold) { KCheck(::GoSetup_SetReversalDistance(Handle, threshold)); }
            }

            /// <summary>Enables profile intensity collection.</summary>
            property bool IntensityEnabled
            {
                bool get()           { return KToBool(::GoSetup_IntensityEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableIntensity(Handle, enabled)); }
            }

            /// <summary>The input trigger enabled state.</summary>
            property bool InputTriggerEnabled
            {
                bool get()           { return KToBool(::GoSetup_InputTriggerEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableInputTrigger(Handle, enabled)); }
            }

            /// <summary>A boolean representing whether the user input trigger configuration is used.</summary>
            property bool InputTriggerEnabledUsed
            {
                bool get()           { return KToBool(::GoSetup_InputTriggerEnabledUsed(Handle)); }
            }

            /// <summary>The input trigger system value.</summary>
            property bool InputTriggerEnabledSystemValue
            {
                bool get()           { return KToBool(::GoSetup_InputTriggerEnabledSystemValue(Handle)); }
            }

            /// <summary>The type used for alignment.</summary>
            property GoAlignmentType AlignmentType
            {
                GoAlignmentType get()           { return (GoAlignmentType) ::GoSetup_AlignmentType(Handle); }
                void set(GoAlignmentType type)  { KCheck(::GoSetup_SetAlignmentType(Handle, type)); }
            }

            /// <summary>Gets the alignment type option at the given index.</summary>
            ///
            /// <param name="index">The index with which to retrieve an alignment type option.</param>
            GoAlignmentType GetAlignmentTypeOption(k64s index)
            {
                return (GoAlignmentType) ::GoSetup_AlignmentTypeOptionAt(Handle, (kSize)index);
            }

            /// <summary>The alignment type option count.</summary>
            property k64s AlignmentTypeOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_AlignmentTypeOptionCount(Handle); }
            }

            /// <summary>Enables encoder calibration after alignment.</summary>
            property bool AlignmentEncoderCalibrateEnabled
            {
                bool get()           { return KToBool(::GoSetup_AlignmentEncoderCalibrateEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableAlignmentEncoderCalibrate(Handle, enabled)); }
            }

            /// <summary>The target type used for stationary alignment calibration.</summary>
            property GoAlignmentTarget AlignmentStationaryTarget
            {
                GoAlignmentTarget get()           { return (GoAlignmentTarget) ::GoSetup_AlignmentStationaryTarget(Handle); }
                void set(GoAlignmentTarget target)  { KCheck(::GoSetup_SetAlignmentStationaryTarget(Handle, target)); }
            }

            /// <summary>The stationary alignment target option count.</summary>
            property k64s AlignmentStationaryTargetOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_AlignmentStationaryTargetOptionCount(Handle); }
            }

            /// <summary>Gets the stationary alignment target option at the given index.</summary>
            ///
            /// <param name="index">The index with which to retrieve an alignment target.</param>
            GoAlignmentTarget GetAlignmentStationaryTargetOption(k64s index)
            {
                return (GoAlignmentTarget) ::GoSetup_AlignmentStationaryTargetOptionAt(Handle, (kSize)index);
            }

            /// <summary>The target type used for moving alignment calibration.</summary>
            property GoAlignmentTarget AlignmentMovingTarget
            {
                GoAlignmentTarget get()           { return (GoAlignmentTarget) ::GoSetup_AlignmentMovingTarget(Handle); }
                void set(GoAlignmentTarget target)  { KCheck(::GoSetup_SetAlignmentMovingTarget(Handle, target)); }
            }

            /// <summary>The moving alignment target option count.</summary>
            property k64s AlignmentMovingTargetOptionCount
            {
                k64s get()           { return (k64s) ::GoSetup_AlignmentMovingTargetOptionCount(Handle); }
            }

            /// <summary>Gets the moving alignment target option at the given index.</summary>
            ///
            /// <param name="index">The index with which to retrieve an alignment target.</param>
            GoAlignmentTarget GetAlignmentMovingTargetOptionAt(k64s index)
            {
                return (GoAlignmentTarget) ::GoSetup_AlignmentMovingTargetOptionAt(Handle, (kSize)index);
            }

            /// <summary>The diameter of the disk used for travel calibration (mm).</summary>
            property k64f DiskDiameter
            {
                k64f get()           { return ::GoSetup_DiskDiameter(Handle); }
                void set(k64f diameter)  { KCheck(::GoSetup_SetDiskDiameter(Handle, diameter)); }
            }

            /// <summary>The height of the disk used for travel calibration (mm).</summary>
            property k64f DiskHeight
            {
                k64f get()           { return ::GoSetup_DiskHeight(Handle); }
                void set(k64f height)  { KCheck(::GoSetup_SetDiskHeight(Handle, height)); }
            }

            /// <summary>The width of the bar used for travel calibration (mm).</summary>
            property k64f BarWidth
            {
                k64f get()           { return ::GoSetup_BarWidth(Handle); }
                void set(k64f width)  { KCheck(::GoSetup_SetBarWidth(Handle, width)); }
            }

            /// <summary>The height of the bar used for travel calibration (mm).</summary>
            property k64f BarHeight
            {
                k64f get()           { return ::GoSetup_BarHeight(Handle); }
                void set(k64f height)  { KCheck(::GoSetup_SetBarHeight(Handle, height)); }
            }

            /// <summary>Indicates if the bar hole count can be modified by the user.</summary>
            property bool BarHoleCountUsed
            {
                bool get()           { return KToBool(::GoSetup_BarHoleCountUsed(Handle)); }
            }

            /// <summary>Gets the system value of the bar hole count.</summary>
            property k64s BarHoleCountValue
            {
                k64s get()           { return (k64s) ::GoSetup_BarHoleCountValue(Handle); }
            }

            /// <summary>The number of holes that are defined on the calibration bar.</summary>
            property k64s BarHoleCount
            {
                k64s get()           { return (k64s) ::GoSetup_BarHoleCount(Handle); }
                void set(k64s count)  { KCheck(::GoSetup_SetBarHoleCount(Handle, (kSize) count)); }
            }

            /// <summary>Indicates if the bar hole distance can be modified by the user.</summary>
            property bool BarHoleDistanceUsed
            {
                bool get()           { return KToBool(::GoSetup_BarHoleDistanceUsed(Handle)); }
            }

            /// <summary>The distance between holes that are defined on the calibration bar (mm).</summary>
            property k64f BarHoleDistance
            {
                k64f get()           { return ::GoSetup_BarHoleDistance(Handle); }
                void set(k64f distance)  { KCheck(::GoSetup_SetBarHoleDistance(Handle, distance)); }
            }

            /// <summary>Indicates if the hole diameter of the bar can be modified by the user.</summary>
            property bool BarHoleDiameterUsed
            {
                bool get()           { return KToBool(::GoSetup_BarHoleDiameterUsed(Handle)); }
            }

            /// <summary>The diameter of holes that are defined on the calibration bar (mm).</summary>
            property k64f BarHoleDiameter
            {
                k64f get()           { return ::GoSetup_BarHoleDiameter(Handle); }
                void set(k64f diameter)  { KCheck(::GoSetup_SetBarHoleDiameter(Handle, diameter)); }
            }

            /// <summary>The degrees of freedom to use in performing bar alignment.</summary>
            property GoAlignmentDegreesOfFreedom BarDegreesOfFreedom
            {
                GoAlignmentDegreesOfFreedom get()           { return ::GoSetup_BarDegreesOfFreedom(Handle); }
                void set(GoAlignmentDegreesOfFreedom dof)  { KCheck(::GoSetup_SetBarDegreesOfFreedom(Handle, dof)); }
            }

            /// <summary>Indicates if the degrees of freedom for bar alignment can be modified by the user.</summary>
            property bool BarDegreesOfFreedomUsed
            {
                bool get()           { return KToBool(::GoSetup_BarDegreesOfFreedomUsed(Handle)); }
            }

            /// <summary>The height of the plate used for travel calibration (mm).</summary>
            property k64f PlateHeight
            {
                k64f get()           { return ::GoSetup_PlateHeight(Handle); }
                void set(k64f height)  { KCheck(::GoSetup_SetPlateHeight(Handle, height)); }
            }

            /// <summary>The number of holes that are defined on the calibration plate.</summary>
            property k64s PlateHoleCount
            {
                k64s get()           { return (k64s) ::GoSetup_PlateHoleCount(Handle); }
                void set(k64s count)  { KCheck(::GoSetup_SetPlateHoleCount(Handle, (kSize) count)); }
            }

            /// <summary>The diameter of the reference hole defined on the calibration plate (mm).</summary>
            property k64f PlateRefHoleDiameter
            {
                k64f get()           { return ::GoSetup_PlateRefHoleDiameter(Handle); }
                void set(k64f diameter)  { KCheck(::GoSetup_SetPlateRefHoleDiameter(Handle, diameter)); }
            }

            /// <summary>The diameter of the secondary hole defined on the calibration plate (mm).</summary>
            property k64f PlateSecHoleDiameter
            {
                k64f get()           { return ::GoSetup_PlateSecHoleDiameter(Handle); }
                void set(k64f diameter)  { KCheck(::GoSetup_SetPlateSecHoleDiameter(Handle, diameter)); }
            }

            /// <summary>Adds corner parameters to the polygon collection</summary>
            ///
            /// <param name="pointx">X coordinate of corner parameter.</param>
            /// <param name="pointz">Z coordinate of corner parameter.</param>
            /// <param name="devices">List of device indexes assigned to the corner (csv).</param>
            void AddPolygonCornerParams(k64f pointx, k64f pointz, String^ devices)
            {
                KString str(devices);

                KCheck(::GoSetup_AddPolygonCornerParams(Handle, pointx, pointz, str.CharPtr));
            }

            /// <summary>Clears all corner parameter objects from the polygon collection.</summary>
            ///
            void ClearPolygonCorners()
            {
                KCheck(::GoSetup_ClearPolygonCorners(Handle));
            }

            /// <summary>Get the number of corner parameter objects defined in the polygon collection</summary>
            ///
            k64s GetPolygonCornerCount()
            {
                return (k64s) ::GoSetup_PolygonCornerCount(Handle);
            }

            /// <summary>Indicates whether X smoothing can be used for the current scan mode and device family.</summary>
            property bool XSmoothingUsed
            {
                bool get()           { return KToBool(::GoSetup_XSmoothingUsed(Handle)); }
            }

            /// <summary>The status of x-direction smoothing.</summary>
            property bool XSmoothingEnabled
            {
                bool get()           { return KToBool(::GoSetup_XSmoothingEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableXSmoothing(Handle, enabled)); }
            }

            /// <summary>The x-direction smoothing window.</summary>
            property k64f XSmoothingWindow
            {
                k64f get()           { return ::GoSetup_XSmoothingWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetXSmoothingWindow(Handle, window)); }
            }

            /// <summary>The x-direction smoothing window minimum.</summary>
            property k64f XSmoothingWindowLimitMin
            {
                k64f get()           { return ::GoSetup_XSmoothingWindowLimitMin(Handle); }
            }

            /// <summary>The x-direction smoothing window maximum.</summary>
            property k64f XSmoothingWindowLimitMax
            {
                k64f get()           { return ::GoSetup_XSmoothingWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether Y smoothing can be used for the current scan mode and device family.</summary>
            property bool YSmoothingUsed
            {
                bool get()           { return KToBool(::GoSetup_YSmoothingUsed(Handle)); }
            }

            /// <summary>The status of y-direction smoothing.</summary>
            property bool YSmoothingEnabled
            {
                bool get()           { return KToBool(::GoSetup_YSmoothingEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableYSmoothing(Handle, enabled)); }
            }

            /// <summary>The y-direction smoothing window.</summary>
            property k64f YSmoothingWindow
            {
                k64f get()           { return ::GoSetup_YSmoothingWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetYSmoothingWindow(Handle, window)); }
            }

            /// <summary>The y-direction smoothing window minimum.</summary>
            property k64f YSmoothingWindowLimitMin
            {
                k64f get()           { return ::GoSetup_YSmoothingWindowLimitMin(Handle); }
            }

            /// <summary>The y-direction smoothing window maximum.</summary>
            property k64f YSmoothingWindowLimitMax
            {
                k64f get()           { return ::GoSetup_YSmoothingWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether X median can be used for the current scan mode and device family.</summary>
            property bool XMedianUsed
            {
                bool get()           { return KToBool(::GoSetup_XMedianUsed(Handle)); }
            }

            /// <summary>The status of x-direction median.</summary>
            property bool XMedianEnabled
            {
                bool get()           { return KToBool(::GoSetup_XMedianEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableXMedian(Handle, enabled)); }
            }

            /// <summary>The x-direction median window.</summary>
            property k64f XMedianWindow
            {
                k64f get()           { return ::GoSetup_XMedianWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetXMedianWindow(Handle, window)); }
            }

            /// <summary>The x-direction median window minimum.</summary>
            property k64f XMedianWindowLimitMin
            {
                k64f get()           { return ::GoSetup_XMedianWindowLimitMin(Handle); }
            }

            /// <summary>The x-direction median window maximum.</summary>
            property k64f XMedianWindowLimitMax
            {
                k64f get()           { return ::GoSetup_XMedianWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether Y median can be used for the current scan mode and device family.</summary>
            property bool YMedianUsed
            {
                bool get()           { return KToBool(::GoSetup_YMedianUsed(Handle)); }
            }

            /// <summary>The status of y-direction median.</summary>
            property bool YMedianEnabled
            {
                bool get()           { return KToBool(::GoSetup_YMedianEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableYMedian(Handle, enabled)); }
            }

            /// <summary>The y-direction median window.</summary>
            property k64f YMedianWindow
            {
                k64f get()           { return ::GoSetup_YMedianWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetYMedianWindow(Handle, window)); }
            }

            /// <summary>The y-direction median window minimum.</summary>
            property k64f YMedianWindowLimitMin
            {
                k64f get()           { return ::GoSetup_YMedianWindowLimitMin(Handle); }
            }

            /// <summary>The y-direction median window maximum.</summary>
            property k64f YMedianWindowLimitMax
            {
                k64f get()           { return ::GoSetup_YMedianWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether X decimation can be used for the current scan mode and device family.</summary>
            property bool XDecimationUsed
            {
                bool get()           { return KToBool(::GoSetup_XDecimationUsed(Handle)); }
            }

            /// <summary>The status of x-direction decimation.</summary>
            property bool XDecimationEnabled
            {
                bool get()           { return KToBool(::GoSetup_XDecimationEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableXDecimation(Handle, enabled)); }
            }

            /// <summary>The x-direction decimation window.</summary>
            property k64f XDecimationWindow
            {
                k64f get()           { return ::GoSetup_XDecimationWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetXDecimationWindow(Handle, window)); }
            }

            /// <summary>The x-direction decimation window minimum.</summary>
            property k64f XDecimationWindowLimitMin
            {
                k64f get()           { return ::GoSetup_XDecimationWindowLimitMin(Handle); }
            }

            /// <summary>The x-direction decimation window maximum.</summary>
            property k64f XDecimationWindowLimitMax
            {
                k64f get()           { return ::GoSetup_XDecimationWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether Y decimation can be used for the current scan mode and device family.</summary>
            property bool YDecimationUsed
            {
                bool get()           { return KToBool(::GoSetup_YDecimationUsed(Handle)); }
            }

            /// <summary>The status of y-direction decimation.</summary>
            property bool YDecimationEnabled
            {
                bool get()           { return KToBool(::GoSetup_YDecimationEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableYDecimation(Handle, enabled)); }
            }

            /// <summary>The y-direction decimation window.</summary>
            property k64f YDecimationWindow
            {
                k64f get()           { return ::GoSetup_YDecimationWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetYDecimationWindow(Handle, window)); }
            }

            /// <summary>The y-direction decimation window minimum.</summary>
            property k64f YDecimationWindowLimitMin
            {
                k64f get()           { return ::GoSetup_YDecimationWindowLimitMin(Handle); }
            }

            /// <summary>The y-direction decimation window maximum.</summary>
            property k64f YDecimationWindowLimitMax
            {
                k64f get()           { return ::GoSetup_YDecimationWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether X gap filling can be used for the current scan mode and device family.</summary>
            property bool XGapFillingUsed
            {
                bool get()           { return KToBool(::GoSetup_XGapFillingUsed(Handle)); }
            }

            /// <summary>The status of x-direction gap-filling.</summary>
            property bool XGapFillingEnabled
            {
                bool get()           { return KToBool(::GoSetup_XGapFillingEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableXGapFilling(Handle, enabled)); }
            }

            /// <summary>The x-direction gap-filling window.</summary>
            property k64f XGapFillingWindow
            {
                k64f get()           { return ::GoSetup_XGapFillingWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetXGapFillingWindow(Handle, window)); }
            }

            /// <summary>The x-direction gap-filling window minimum.</summary>
            property k64f XGapFillingWindowLimitMin
            {
                k64f get()           { return ::GoSetup_XGapFillingWindowLimitMin(Handle); }
            }

            /// <summary>The x-direction gap-filling window maximum.</summary>
            property k64f XGapFillingWindowLimitMax
            {
                k64f get()           { return ::GoSetup_XGapFillingWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether Y gap filling can be used for the current scan mode and device family.</summary>
            property bool YGapFillingUsed
            {
                bool get()           { return KToBool(::GoSetup_YGapFillingUsed(Handle)); }
            }

            /// <summary>The status of y-direction gap-filling.</summary>
            property bool YGapFillingEnabled
            {
                bool get()           { return KToBool(::GoSetup_YGapFillingEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableYGapFilling(Handle, enabled)); }
            }

            /// <summary>The y-direction gap-filling window.</summary>
            property k64f YGapFillingWindow
            {
                k64f get()           { return ::GoSetup_YGapFillingWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetYGapFillingWindow(Handle, window)); }
            }

            /// <summary>The y-direction gap-filling window minimum.</summary>
            property k64f YGapFillingWindowLimitMin
            {
                k64f get()           { return ::GoSetup_YGapFillingWindowLimitMin(Handle); }
            }

            /// <summary>The y-direction gap-filling window maximum.</summary>
            property k64f YGapFillingWindowLimitMax
            {
                k64f get()           { return ::GoSetup_YGapFillingWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether X slope can be used for the current scan mode and device family.</summary>
            property bool XSlopeUsed
            {
                bool get()           { return KToBool(::GoSetup_XSlopeUsed(Handle)); }
            }

            /// <summary>The status of x-direction slope.</summary>
            property bool XSlopeEnabled
            {
                bool get()           { return KToBool(::GoSetup_XSlopeEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableXSlope(Handle, enabled)); }
            }

            /// <summary>The x-direction slope window.</summary>
            property k64f XSlopeWindow
            {
                k64f get()           { return ::GoSetup_XSlopeWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetXSlopeWindow(Handle, window)); }
            }

            /// <summary>The x-direction slope window minimum.</summary>
            property k64f XSlopeWindowLimitMin
            {
                k64f get()           { return ::GoSetup_XSlopeWindowLimitMin(Handle); }
            }

            /// <summary>The x-direction slope window maximum.</summary>
            property k64f XSlopeWindowLimitMax
            {
                k64f get()           { return ::GoSetup_XSlopeWindowLimitMax(Handle); }
            }

            /// <summary>Indicates whether Y slope can be used for the current scan mode and device family.</summary>
            property bool YSlopeUsed
            {
                bool get()           { return KToBool(::GoSetup_YSlopeUsed(Handle)); }
            }

            /// <summary>The status of y-direction slope.</summary>
            property bool YSlopeEnabled
            {
                bool get()           { return KToBool(::GoSetup_YSlopeEnabled(Handle)); }
                void set(bool enabled)  { KCheck(::GoSetup_EnableYSlope(Handle, enabled)); }
            }

            /// <summary>The y-direction slope window.</summary>
            property k64f YSlopeWindow
            {
                k64f get()           { return ::GoSetup_YSlopeWindow(Handle); }
                void set(k64f window)  { KCheck(::GoSetup_SetYSlopeWindow(Handle, window)); }
            }

            /// <summary>The y-direction slope window minimum.</summary>
            property k64f YSlopeWindowLimitMin
            {
                k64f get()           { return ::GoSetup_YSlopeWindowLimitMin(Handle); }
            }

            /// <summary>The y-direction slope window maximum.</summary>
            property k64f YSlopeWindowLimitMax
            {
                k64f get()           { return ::GoSetup_YSlopeWindowLimitMax(Handle); }
            }

            /// <summary>Gets the maximum valid value for the Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetExposureLimitMax(GoRole role)
            {
                return ::GoSetup_ExposureLimitMax(Handle, role);
            }

            /// <summary>Gets the minimum valid value for the Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetExposureLimitMin(GoRole role)
            {
                return ::GoSetup_ExposureLimitMin(Handle, role);
            }

            /// <summary>Sets the exposure value (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="exposure">Intended exposure value.</param>
            void SetExposure(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_SetExposure(Handle, role, exposure));
            }

            /// <summary>Gets the exposure value (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetExposure(GoRole role)
            {
                return ::GoSetup_Exposure(Handle, role);
            }

            /// <summary>Gets the exposure mode.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            GoExposureMode GetExposureMode(GoRole role)
            {
                return (GoExposureMode) ::GoSetup_ExposureMode(Handle, role);
            }

            /// <summary>Sets the exposure mode.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="mode">The exposure mode to use.</param>
            void SetExposureMode(GoRole role, GoExposureMode mode)
            {
                KCheck(::GoSetup_SetExposureMode(Handle, role, mode));
            }

            /// <summary>Gets the exposure mode option at the given index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The index with which to retrieve an exposure mode option.</param>
            GoExposureMode GetExposureModeOption(GoRole role, k64s index)
            {
                return (GoExposureMode) ::GoSetup_ExposureModeOptionAt(Handle, role, (kSize)index);
            }

            /// <summary>Gets the exposure mode option count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetExposureModeOptionCount(GoRole role)
            {
                return (k64s) ::GoSetup_ExposureModeOptionCount(Handle, role);
            }

            /// <summary>Adds an exposure step</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="exposure">Exposure value (microseconds).</param>
            void AddExposureStep(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_AddExposureStep(Handle, role, exposure));
            }

            /// <summary>Removes all exposure steps.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            void ClearExposureSteps(GoRole role)
            {
                KCheck(::GoSetup_ClearExposureSteps(Handle, role));
            }

            /// <summary>Get the exposure step value specified by index</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The index of the exposure step to get.</param>
            k64f GetExposureStep(GoRole role, k64s index)
            {
                return ::GoSetup_ExposureStepAt(Handle, role, (kSize)index);
            }

            /// <summary>Get the number of exposure steps defined</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetExposureStepCount(GoRole role)
            {
                return (k64s) ::GoSetup_ExposureStepCount(Handle, role);
            }

            /// <summary>Gets the maximum value for the Dynamic Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetDynamicExposureMax(GoRole role)
            {
                return ::GoSetup_DynamicExposureMax(Handle, role);
            }

            /// <summary>Sets the maximum value for the Dynamic Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            void SetDynamicExposureMax(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_SetDynamicExposureMax(Handle, role, exposure));
            }

            /// <summary>Gets the minimum value for the Dynamic Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetDynamicExposureMin(GoRole role)
            {
                return ::GoSetup_DynamicExposureMin(Handle, role);
            }

            /// <summary>Sets the minimum value for the Dynamic Exposure setting (microseconds).</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            void SetDynamicExposureMin(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_SetDynamicExposureMin(Handle, role, exposure));
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaHeight setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaHeightLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaHeightLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaHeight setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaHeightLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaHeightLimitMax(Handle, role);
            }

            /// <summary>Gets the active area height (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaHeight(GoRole role)
            {
                return ::GoSetup_ActiveAreaHeight(Handle, role);
            }

            /// <summary>Sets the active area height.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area height (mm).</param>
            void SetActiveAreaHeight(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaHeight(Handle, role, value));
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaLength setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaLengthLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaLengthLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaLength setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaLengthLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaLengthLimitMax(Handle, role);
            }

            /// <summary>Sets the active area length.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area length (mm).</param>
            void SetActiveAreaLength(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaLength(Handle, role, value));
            }

            /// <summary>Gets the active area length (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaLength(GoRole role)
            {
                return ::GoSetup_ActiveAreaLength(Handle, role);
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaWidth setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaWidthLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaWidthLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaWidth setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaWidthLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaWidthLimitMax(Handle, role);
            }

            /// <summary>Sets the active area width.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area width (mm).</param>
            void SetActiveAreaWidth(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaWidth(Handle, role, value));
            }

            /// <summary>Gets the active area width (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaWidth(GoRole role)
            {
                return ::GoSetup_ActiveAreaWidth(Handle, role);
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaX setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaXLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaXLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaX setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaXLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaXLimitMax(Handle, role);
            }

            /// <summary>Sets the active area y origin.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area y origin (mm).</param>
            void SetActiveAreaX(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaX(Handle, role, value));
            }

            /// <summary>Gets the active area y origin (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaX(GoRole role)
            {
                return ::GoSetup_ActiveAreaX(Handle, role);
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaY setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaYLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaYLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaY setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaYLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaYLimitMax(Handle, role);
            }

            /// <summary>Sets the active area y origin.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area y origin (mm).</param>
            void SetActiveAreaY(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaY(Handle, role, value));
            }

            /// <summary>Gets the active area y origin (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaY(GoRole role)
            {
                return ::GoSetup_ActiveAreaY(Handle, role);
            }

            /// <summary>Gets the minimum valid value for the ActiveAreaZ setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaZLimitMin(GoRole role)
            {
                return ::GoSetup_ActiveAreaZLimitMin(Handle, role);
            }

            /// <summary>Gets the maximum valid value for the ActiveAreaZ setting (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaZLimitMax(GoRole role)
            {
                return ::GoSetup_ActiveAreaZLimitMax(Handle, role);
            }

            /// <summary>Sets the active area z origin.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="value">Active area z origin (mm).</param>
            void SetActiveAreaZ(GoRole role, k64f value)
            {
                KCheck(::GoSetup_SetActiveAreaZ(Handle, role, value));
            }

            /// <summary>Gets the active area z origin (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetActiveAreaZ(GoRole role)
            {
                return ::GoSetup_ActiveAreaZ(Handle, role);
            }

            /// <summary>Gets the transformed data region X value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionX(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionX(Handle, role);
            }

            /// <summary>Gets the transformed data region Y value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionY(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionY(Handle, role);
            }

            /// <summary>Gets the transformed data region Z value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionZ(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionZ(Handle, role);
            }

            /// <summary>Gets the transformed data region width value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionWidth(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionWidth(Handle, role);
            }

            /// <summary>Gets the transformed data region length value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionLength(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionLength(Handle, role);
            }

            /// <summary>Gets the transformed data region height value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTransformedDataRegionHeight(GoRole role)
            {
                return ::GoSetup_TransformedDataRegionHeight(Handle, role);
            }

            /// <summary>Gets the count of valid x-resolution options.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetXSubsamplingOptionCount(GoRole role)
            {
                return (k64s) ::GoSetup_XSubsamplingOptionCount(Handle, role);
            }

            /// <summary>Gets the x-resolution option at the specified index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">Index of the desired resolution option.</param>
            k32u GetXSubsamplingOption(GoRole role, k64s index)
            {
                return ::GoSetup_XSubsamplingOptionAt(Handle, role, (kSize)index);
            }

            /// <summary>Sets the current x-resolution divider.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="xSubsampling">X subsampling divider (e.g. 1 - full res, 2 - half res).</param>
            void SetXSubsampling(GoRole role, k32u xSubsampling)
            {
                KCheck(::GoSetup_SetXSubsampling(Handle, role, xSubsampling));
            }

            /// <summary>Gets the current x-resolution divider.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetXSubsampling(GoRole role)
            {
                return ::GoSetup_XSubsampling(Handle, role);
            }

            /// <summary>Gets the count of valid z-resolution options.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetZSubsamplingOptionCount(GoRole role)
            {
                return (k64s) ::GoSetup_ZSubsamplingOptionCount(Handle, role);
            }

            /// <summary>Gets the z-resolution option at the specified index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">Index of the desired resolution option.</param>
            k32u GetZSubsamplingOption(GoRole role, k64s index)
            {
                return ::GoSetup_ZSubsamplingOptionAt(Handle, role, (kSize)index);
            }

            /// <summary>Sets the current z-resolution divider.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="zSubsampling">Z subsampling divider (e.g. 1 - full res, 2 - half res).</param>
            void SetZSubsampling(GoRole role, k32u zSubsampling)
            {
                KCheck(::GoSetup_SetZSubsampling(Handle, role, zSubsampling));
            }

            /// <summary>Gets the current z-resolution divider.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetZSubsampling(GoRole role)
            {
                return ::GoSetup_ZSubsampling(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest x origin (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetFrontCameraX(GoRole role)
            {
                return ::GoSetup_FrontCameraX(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest y origin (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetFrontCameraY(GoRole role)
            {
                return ::GoSetup_FrontCameraY(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest width (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetFrontCameraWidth(GoRole role)
            {
                return ::GoSetup_FrontCameraWidth(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest height (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetFrontCameraHeight(GoRole role)
            {
                return ::GoSetup_FrontCameraHeight(Handle, role);
            }

            /// <summary>Returns a boolean representing whether the back camera element is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetBackCameraUsed(GoRole role)
            {
                return KToBool(::GoSetup_BackCameraUsed(Handle, role));
            }

            /// <summary>Gets the camera region-of-interest x origin (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetBackCameraX(GoRole role)
            {
                return ::GoSetup_BackCameraX(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest y origin (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetBackCameraY(GoRole role)
            {
                return ::GoSetup_BackCameraY(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest width (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetBackCameraWidth(GoRole role)
            {
                return ::GoSetup_BackCameraWidth(Handle, role);
            }

            /// <summary>Gets the camera region-of-interest height (pixels).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetBackCameraHeight(GoRole role)
            {
                return ::GoSetup_BackCameraHeight(Handle, role);
            }

            /// <summary>Enables tracking.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="enable">true to enable, or false to disable.</param>
            void EnableTracking(GoRole role, bool enable)
            {
                KCheck(::GoSetup_EnableTracking(Handle, role, enable));
            }

            /// <summary>Determines if tracking is enabled.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetTrackingEnabled(GoRole role)
            {
                return KToBool(::GoSetup_TrackingEnabled(Handle, role));
            }

            /// <summary>Returns a boolean value representing whether the Tracking Enabled field is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetTrackingUsed(GoRole role)
            {
                return KToBool(::GoSetup_TrackingUsed(Handle, role));
            }

            /// <summary>Sets the tracking window height.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="height">Tracking window height (mm).</param>
            void SetTrackingAreaHeight(GoRole role, k64f height)
            {
                KCheck(::GoSetup_SetTrackingAreaHeight(Handle, role, height));
            }

            /// <summary>Gets the tracking window height (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTrackingAreaHeight(GoRole role)
            {
                return ::GoSetup_TrackingAreaHeight(Handle, role);
            }

            /// <summary>Gets the tracking window height minimum limit (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTrackingAreaHeightLimitMin(GoRole role)
            {
                return ::GoSetup_TrackingAreaHeightLimitMin(Handle, role);
            }

            /// <summary>Gets the tracking window height maximum limit (mm).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTrackingAreaHeightLimitMax(GoRole role)
            {
                return ::GoSetup_TrackingAreaHeightLimitMax(Handle, role);
            }

            /// <summary>Sets the tracking window search threshold.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="height">Tracking window search threshold (%)</param>
            void SetTrackingSearchThreshold(GoRole role, k64f threshold)
            {
                KCheck(::GoSetup_SetTrackingSearchThreshold(Handle, role, threshold));
            }

            /// <summary>Gets the tracking window search threshold (%).</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetTrackingSearchThreshold(GoRole role)
            {
                return ::GoSetup_TrackingSearchThreshold(Handle, role);
            }

            /// <summary>Returns the state of whether the user specified spacing interval is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetSpacingIntervalUsed(GoRole role)
            {
                return KToBool(::GoSetup_SpacingIntervalUsed(Handle, role));
            }

            /// <summary>Gets the spacing interval system value.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetSpacingIntervalSystemValue(GoRole role)
            {
                return ::GoSetup_SpacingIntervalSystemValue(Handle, role);
            }

            /// <summary>Sets the spacing interval.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="interval">Spacing interval</param>
            void SetSpacingInterval(GoRole role, k64f interval)
            {
                KCheck(::GoSetup_SetSpacingInterval(Handle, role, interval));
            }

            /// <summary>Gets the spacing interval.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetSpacingInterval(GoRole role)
            {
                return ::GoSetup_SpacingInterval(Handle, role);
            }

            /// <summary>Gets the spacing interval value limit minimum.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetSpacingIntervalLimitMin(GoRole role)
            {
                return ::GoSetup_SpacingIntervalLimitMin(Handle, role);
            }

            /// <summary>Gets the spacing interval value limit maximum.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetSpacingIntervalLimitMax(GoRole role)
            {
                return ::GoSetup_SpacingIntervalLimitMax(Handle, role);
            }

            /// <summary>Gets the spacing interval type.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            GoSpacingIntervalType GetSpacingIntervalType(GoRole role)
            {
                return (GoSpacingIntervalType) ::GoSetup_SpacingIntervalType(Handle, role);
            }

            /// <summary>Sets the spacing interval type.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="type">The spacing interval type.</param>
            void SetTrackingSearchThreshold(GoRole role, GoSpacingIntervalType type)
            {
                KCheck(::GoSetup_SetTrackingSearchThreshold(Handle, role, type));
            }

            /// <summary>Gets the system value representing whether or not the user specified spacing interval type setting is being used at the moment.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetSpacingIntervalTypeUsed(GoRole role)
            {
                return KToBool(::GoSetup_SpacingIntervalTypeUsed(Handle, role));
            }

            /// <summary>Gets the X spacing count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetXSpacingCount(GoRole role)
            {
                return ::GoSetup_XSpacingCount(Handle, role);
            }

            /// <summary>Gets the Y spacing count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetYSpacingCount(GoRole role)
            {
                return ::GoSetup_YSpacingCount(Handle, role);
            }

            /// <summary>Gets the intensity step index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetIntensityStepIndex(GoRole role)
            {
                return (k64s) ::GoSetup_IntensityStepIndex(Handle, role);
            }

            /// <summary>Sets the intensity step index.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The exposure step index to use for intensity acquisition.</param>
            void SetIntensityStepIndex(GoRole role, k64s index)
            {
                KCheck(::GoSetup_SetIntensityStepIndex(Handle, role, (kSize)index));
            }

            /// <summary>Gets the pattern sequence type option count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetPatternSequenceTypeOptionCount(GoRole role)
            {
                return (k64s) ::GoSetup_PatternSequenceTypeOptionCount(Handle, role);
            }

            /// <summary>Gets the pattern sequence type option at the given index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The index with which to retrieve a sequence type option.</param>
            GoPatternSequenceType GetPatternSequenceTypeOption(GoRole role, k64s index)
            {
                return (GoPatternSequenceType) ::GoSetup_PatternSequenceTypeOptionAt(Handle, role, (kSize)index);
            }

            /// <summary>Gets the pattern sequence type.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            GoPatternSequenceType GetPatternSequenceType(GoRole role)
            {
                return (GoPatternSequenceType) ::GoSetup_PatternSequenceType(Handle, role);
            }

            /// <summary>Sets the pattern sequence type.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="type">The pattern sequence type to set.</param>
            void SetPatternSequenceType(GoRole role, GoPatternSequenceType type)
            {
                KCheck(::GoSetup_SetPatternSequenceType(Handle, role, type));
            }

            /// <summary>Returns a boolean value representing whether the pattern sequence type is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetPatternSequenceTypeUsed(GoRole role)
            {
                return KToBool(::GoSetup_PatternSequenceTypeUsed(Handle, role));
            }

            /// <summary>Gets the current pattern sequence count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetPatternSequenceCount(GoRole role)
            {
                return (k64s) ::GoSetup_PatternSequenceCount(Handle, role);
            }

            /// <summary>Gets the phase pattern index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetPatternSequenceIndex(GoRole role)
            {
                return ::GoSetup_PatternSequenceIndex(Handle, role);
            }

            /// <summary>Sets the phase pattern index.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The phase pattern index to set.</param>
            void SetPatternSequenceIndex(GoRole role, k32u index)
            {
                KCheck(::GoSetup_SetPatternSequenceIndex(Handle, role, index));
            }

            /// <summary>Returns a boolean value representing whether the phase pattern index is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetPatternSequenceIndexUsed(GoRole role)
            {
                return KToBool(::GoSetup_PatternSequenceIndexUsed(Handle, role));
            }

            /// <summary>Returns the minimum phase pattern index</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetPatternSequenceIndexMin(GoRole role)
            {
                return ::GoSetup_PatternSequenceIndexMin(Handle, role);
            }

            /// <summary>Returns the maximum phase pattern index</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k32u GetPatternSequenceIndexMax(GoRole role)
            {
                return ::GoSetup_PatternSequenceIndexMax(Handle, role);
            }

            /// <summary>Gets the layout configuration module.</summary>
            /// <returns>The layout configuration module.</returns>
            GoLayout^ GetLayout()
            {
                return KToObject<GoLayout^>(::GoSetup_Layout(Handle));
            }

            /// <summary>Gets the profile generation module, used for profile generation configuration.</summary>
            /// <returns>The profile generation module.</returns>
            GoProfileGeneration^ GetProfileGeneration()
            {
                return KToObject<GoProfileGeneration^>(::GoSetup_ProfileGeneration(Handle));
            }

            /// <summary>Gets the surface generation module, used for surface generation configuration.</summary>
            /// <returns>The surface generation module.</returns>
            GoSurfaceGeneration^ GetSurfaceGeneration()
            {
                return KToObject<GoSurfaceGeneration^>(::GoSetup_SurfaceGeneration(Handle));
            }

            /// <summary>Gets the part detection module, used for part detection configuration.</summary>
            /// <returns>The part detection module.</returns>
            GoPartDetection^ GetPartDetection()
            {
                return KToObject<GoPartDetection^>(::GoSetup_PartDetection(Handle));
            }

            /// <summary>Gets the part matching module, used for part matching configuration.</summary>
            /// <returns>The part matching module.</returns>
            GoPartMatching^ GetPartMatching()
            {
                return KToObject<GoPartMatching^>(::GoSetup_PartMatching(Handle));
            }
            #pragma warning(disable:4947)

            /// <summary>Gets the advanced acquisition module, used for advanced acquisition configuration.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <returns>The advanced acquisition module.</returns>
            GoAdvanced^ GetAdvanced(GoRole role)
            {
                return KToObject<GoAdvanced^>(::GoSetup_Advanced(Handle, role));
            }

            /// <summary>Depricated. Gets the material acquisition module, used for material acquisition configuration.</summary>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <returns>The material acquisition module.</returns>
            [Obsolete("GoMaterial has been renamed to GoAdvanced", false)]
            GoMaterial^ GetMaterial(GoRole role)
            {
                ::GoAdvanced advanced = ::GoSetup_Advanced(Handle, role);
                if (advanced == nullptr)
                {
                    return nullptr;
                }
                else
                {
                    return gcnew GoMaterial(IntPtr(advanced));
                }
            }
            #pragma warning(default:4947)

            /// <summary>Gets the section configuration module, used for surface sections.</summary>
            /// <returns>The section configuration module.</returns>
            GoSections^ GetSections()
            {
                return KToObject<GoSections^>(::GoSetup_Sections(Handle));
            }

            /// <summary>Returns a boolean value representing whether the independent camera exposures feature is used.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetIndependentExposuresUsed(GoRole role)
            {
                return KToBool(::GoSetup_IndependentExposuresUsed(Handle, role));
            }

            /// <summary>Enables independent camera exposures feature.</summary>
            ///
            /// <remarks>
            /// This feature utilizes the front and back camera exposure values.
            /// </remarks>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="enable">true to enable, or false to disable.</param>
            void EnableIndependentExposures(GoRole role, bool enable)
            {
                KCheck(::GoSetup_EnableIndependentExposures(Handle, role, enable));
            }

            /// <summary>Determines if independent camera exposures feature is enabled.</summary>
            ///
            /// <remarks>
            /// This feature utilizes the front and back camera exposure values.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool GetIndependentExposuresEnabled(GoRole role)
            {
                return KToBool(::GoSetup_IndependentExposuresEnabled(Handle, role));
            }

            /// <summary>Gets the maximum value for the front camera's exposure setting (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetFrontCameraExposureLimitMax(GoRole role)
            {
                return ::GoSetup_FrontCameraExposureLimitMax(Handle, role);
            }

            /// <summary>Gets the minimum value for the front camera's exposure setting (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetFrontCameraExposureLimitMin(GoRole role)
            {
                return ::GoSetup_FrontCameraExposureLimitMin(Handle, role);
            }

            /// <summary>Sets the front camera's exposure value (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="exposure">Intended exposure value.</param>
            void SetFrontCameraExposure(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_SetFrontCameraExposure(Handle, role, exposure));
            }

            /// <summary>Gets the front camera's exposure value (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetFrontCameraExposure(GoRole role)
            {
                return ::GoSetup_FrontCameraExposure(Handle, role);
            }

            /// <summary>Gets the maximum value for the back camera's exposure setting (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetBackCameraExposureLimitMax(GoRole role)
            {
                return ::GoSetup_BackCameraExposureLimitMax(Handle, role);
            }

            /// <summary>Gets the minimum value for the back camera's exposure setting (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetBackCameraExposureLimitMin(GoRole role)
            {
                return ::GoSetup_BackCameraExposureLimitMin(Handle, role);
            }

            /// <summary>Sets the back camera's exposure value (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="exposure">Intended exposure value.</param>
            void SetBackCameraExposure(GoRole role, k64f exposure)
            {
                KCheck(::GoSetup_SetBackCameraExposure(Handle, role, exposure));
            }

            /// <summary>Gets the back camera's exposure value (microseconds).</summary>
            ///
            /// <remarks>
            /// NOTE: Utilized when independent camera exposures feature is enabled.
            /// </remarks>
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64f GetBackCameraExposure(GoRole role)
            {
                return ::GoSetup_BackCameraExposure(Handle, role);
            }

            /// <summary>Gets the intensity generation mode.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            GoIntensityMode GetIntensityMode(GoRole role)
            {
                return (GoIntensityMode) ::GoSetup_IntensityMode(Handle, role);
            }

            /// <summary>Sets the intensity generation mode.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="mode">New intensity generation mode.</param>
            void SetIntensityMode(GoRole role, GoIntensityMode mode)
            {
                KCheck(::GoSetup_SetIntensityMode(Handle, role, mode));
            }

            /// <summary>Gets the availability of the intensity generation mode.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            bool IntensityModeUsed(GoRole role)
            {
                return KToBool(::GoSetup_IntensityModeUsed(Handle, role));
            }

            /// <summary>Gets the source used for generating intensity data.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            GoIntensitySource GetIntensitySource(GoRole role)
            {
                return (GoIntensitySource) ::GoSetup_IntensitySource(Handle, role);
            }

            /// <summary>Sets the source to be used for generating intensity data.</summary>
            ///
            /// <param name="role">Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="source">The intensity source to use.</param>
            void SetIntensitySource(GoRole role, GoIntensitySource source)
            {
                KCheck(::GoSetup_SetIntensitySource(Handle, role, source));
            }

            /// <summary>Gets the intensity source option at the given index.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name="index">The index with which to retrieve an intensity source option.</param>
            GoIntensitySource GetIntensitySourceOption(GoRole role, k64s index)
            {
                return (GoIntensitySource) ::GoSetup_IntensitySourceOptionAt(Handle, role, (kSize)index);
            }

            /// <summary>Gets the intensity source option count.</summary>
            ///
            /// <param name="role">Determines which device to retrieve the value from. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            k64s GetIntensitySourceOptionCount(GoRole role)
            {
                return (k64s) ::GoSetup_IntensitySourceOptionCount(Handle, role);
            }

            /// <summary>Adds a corner parameters object to the collection of polygon corners for alignment.</summary>
            /// <param name=corner>Reference to GoPolygonCornerParameters object.</param>
            /// <return>Operation status.</return>
            void AddPolygonCorner(GoPolygonCornerParameters corner) 
            {
                ::GoPolygonCornerParameters params = (::GoPolygonCornerParameters)corner;
                KCheck(::GoSetup_AddPolygonCorner(Handle, &params));
            }

            /// <summary>Indicates whether alignment can be used.</summary>
            /// <return>True if alignment is available for use.</return>
            property bool AlignmentUsed 
            {
                bool get() { return KToBool(::GoSetup_AlignmentUsed(Handle)); }
            }

            /// <summary>Gets the degrees of freedom bar alignment target option count.</summary>
            property kSize BarDegreesOfFreedomOptionCount
            {
                kSize get()   { return (kSize)::GoSetup_BarDegreesOfFreedomOptionCount(Handle); }
            }

            /// <summary>Gets the bar alignment degrees of freedom option at the given index.</summary>
            /// <param name=index>The index with which to retrieve the degree of freedom.</param>
            /// <return>The degrees of freedom option set at the index.</return>
            GoAlignmentDegreesOfFreedom GetBarDegreesOfFreedomOptionAt(k64s index) 
            {
                return (GoAlignmentDegreesOfFreedom)::GoSetup_BarDegreesOfFreedomOptionAt(Handle, (kSize)index);
            }

            /// <summary>Gets/Sets the state (enabled/disabled) of the Master time encoder.</summary>
            property bool PreferMasterTimeEncoderEnabled
            {
                bool get() { return KToBool(::GoSetup_PreferMasterTimeEncoderEnabled(Handle)); }
                void set(bool value)   { ::GoSetup_EnablePreferMasterTimeEncoderEnabled(Handle, value); }
            }

            /// <summary>Enables the temperature saftey  of the sensor system.</summary>
            /// <param name=enable>bool.</param>
            /// <return>Operation status.</return>
            property bool TemperatureSafety
            {
                void set(bool value)   { ::GoSetup_EnableTemperatureSafety(Handle, value); }
            }

            /// <summary>Retrieves the reference to the corner parameters object from the collection of polygon corners.</summary>
            /// <param name=index>Index in the collection of polygon corners.</param>
            /// <return>Reference to corresponding corner parameters.</return>
            GoPolygonCornerParameters GetPolygonCornerAt(k64s index) 
            {
                ::GoPolygonCornerParameters* params = ::GoSetup_PolygonCornerAt(Handle, (kSize)index);
                return GoPolygonCornerParameters(params);
            }

            /// <summary>Sets the spacing interval type.</summary>
            /// <param name=role>Determines which device to apply changes to. Use GoRole.Main or GoRole.Buddy + buddyidx. See GoRole for more details.</param>
            /// <param name=type>The spacing interval type.</param>
            /// <return>Operation status.</return>
            void SetSpacingIntervalType(GoRole role, GoSpacingIntervalType type) 
            {
                KCheck(::GoSetup_SetSpacingIntervalType(Handle, role, type));
            }
            
            /// <summary>Gets the tracheid configuration module.</summary>
            /// <param name=role>Determines which device to retrieve the value from.</param>
            /// <return>Tracheid configuration module.</return>
            GoTracheid^ GetTracheid(GoRole role)
            {
                ::GoTracheid id = ::GoSetup_Tracheid(Handle, role);

                return (kIsNull(id)) ? nullptr : KToObject<GoTracheid^>(id);
            }

            /// <summary>The background supression feature enabled.</summary>
            property bool BackgroundSuppressionEnabled
            {
                bool get() { return KToBool(::GoSetup_BackgroundSuppressionEnabled(Handle)); }
                void set(bool enabled) { KCheck(::GoSetup_EnableBackgroundSuppression(Handle, enabled)); }
            }

            /// <summary>The background supression feature ratio.</summary>
            property k64u BackgroundSuppressionRatio
            {
                k64u get() { return (k64u)::GoSetup_BackgroundSuppressionRatio(Handle); }
                void set(k64u ratio) { KCheck(::GoSetup_SetBackgroundSuppressionRatio(Handle, (k64u)ratio)); }
            }

        };
    }
}

#endif
