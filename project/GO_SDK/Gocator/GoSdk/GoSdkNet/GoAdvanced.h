// 
// GoAdvanced.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_ADVANCED_H
#define GO_SDK_NET_ADVANCED_H

#include <GoSdk/GoAdvanced.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents configurable advanced acquisition settings.</summary>
        public ref class GoAdvanced : public KObject
        {
            KDeclareClass(GoAdvanced, GoAdvanced)

            /// <summary>Initializes a new instance of the GoAdvanced class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoAdvanced(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoAdvanced class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoAdvanced(GoSensor^ sensor)
            {
                ::GoAdvanced handle = kNULL;

                KCheck(::GoAdvanced_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoAdvanced(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoAdvanced(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoAdvanced handle = kNULL;

                KCheck(::GoAdvanced_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The advanced acquisition type.</summary>
            property GoAdvancedType AdvancedType
            {
                GoAdvancedType get()           { return (GoAdvancedType) ::GoAdvanced_Type(Handle); }
                void set(GoAdvancedType type)  { KCheck(::GoAdvanced_SetType(Handle, type)); }
            }

            /// <summary>A boolean relating to whether the user defined advanced acquisition type value will be used by the system.</summary>
            property bool IsTypeUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsTypeUsed(Handle)); }
            }

            /// <summary>The advanced acquisition type to be used by the system.</summary>
            property GoAdvancedType AdvancedTypeSystemValue
            {
                GoAdvancedType get()           { return (GoAdvancedType) ::GoAdvanced_TypeSystemValue(Handle); }
            }

            /// <summary>The spot threshold.</summary>
            property k32u SpotThreshold
            {
                k32u get()           { return ::GoAdvanced_SpotThreshold(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotThreshold(Handle, value)); }
            }

            /// <summary>The minimum spot threshold limit.</summary>
            property k32u SpotThresholdLimitMin
            {
                k32u get()           { return ::GoAdvanced_SpotThresholdLimitMin(Handle); }
            }

            /// <summary>The maximum spot threshold limit.</summary>
            property k32u SpotThresholdLimitMax
            {
                k32u get()           { return ::GoAdvanced_SpotThresholdLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the user specified spot threshold value is used.</summary>
            property bool IsSpotThresholdUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotThresholdUsed(Handle)); }
            }

            /// <summary>The system spot threshold value.</summary>
            property k32u SpotThresholdSystemValue
            {
                k32u get()           { return ::GoAdvanced_SpotThresholdSystemValue(Handle); }
            }

            /// <summary>The maximum spot width.</summary>
            property k32u SpotWidthMax
            {
                k32u get()           { return ::GoAdvanced_SpotWidthMax(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotWidthMax(Handle, value)); }
            }

            /// <summary>The maximum spot width minimum limit.</summary>
            property k32u SpotWidthMaxLimitMin
            {
                k32u get()           { return ::GoAdvanced_SpotWidthMaxLimitMin(Handle); }
            }

            /// <summary>The maximum spot width maximum limit.</summary>
            property k32u SpotWidthMaxLimitMax
            {
                k32u get()           { return ::GoAdvanced_SpotWidthMaxLimitMax(Handle); }
            }

            /// <summary>A boolean relating to whether the user defined spot width max value will be used by the system.</summary>
            property bool IsSpotWidthMaxUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotWidthMaxUsed(Handle)); }
            }

            /// <summary>The maximum spot width system value.</summary>
            property k32u SpotWidthMaxSystemValue
            {
                k32u get()           { return ::GoAdvanced_SpotWidthMaxSystemValue(Handle); }
            }

            /// <summary>The number of spot selection type options.</summary>
            property k64s SpotSelectionTypeOptionCount
            {
                k64s get()           { return (k64s) ::GoAdvanced_SpotSelectionTypeOptionCount(Handle); }
            }

            /// <summary>Returns the spot selection type option at the given index.</summary>
            /// <param name="index">The option list index to access.</param>
            GoSpotSelectionType GetSpotSelectionTypeOption(k64s index)
            {
                return (GoSpotSelectionType) ::GoAdvanced_SpotSelectionTypeOptionAt(Handle, (kSize)index);
            }

            /// <summary>The spot selection type.</summary>
            property GoSpotSelectionType SpotSelectionType
            {
                GoSpotSelectionType get()           { return ::GoAdvanced_SpotSelectionType(Handle); }
                void set(GoSpotSelectionType type)  { KCheck(::GoAdvanced_SetSpotSelectionType(Handle, type)); }
            }

            /// <summary>A boolean relating to whether the user defined spot selection type will be used by the system.</summary>
            property bool IsSpotSelectionTypeUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotSelectionTypeUsed(Handle)); }
            }

            /// <summary>The system spot selection type.</summary>
            property GoSpotSelectionType SpotSelectionTypeSystemValue
            {
                GoSpotSelectionType get()           { return ::GoAdvanced_SpotSelectionTypeSystemValue(Handle); }
            }

            /// <summary>The minimum segment size used in continuity sorting spot detection.</summary>
            property k32u SpotContinunityMinimumSegmentSize
            {
                k32u get()           { return ::GoAdvanced_SpotContinuityMinimumSegmentSize(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotContinuityMinimumSegmentSize(Handle, value)); }
            }

            /// <summary>The search window size in the X direction used in continuity sortinging spot detection.</summary>
            property k32u SpotContinunitySearchWindowX
            {
                k32u get()           { return ::GoAdvanced_SpotContinuitySearchWindowX(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotContinuitySearchWindowX(Handle, value)); }
            }

            /// <summary>The search window size in the Y direction used in continuity sortinging spot detection.</summary>
            property k32u SpotContinunitySearchWindowY
            {
                k32u get()           { return ::GoAdvanced_SpotContinuitySearchWindowY(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotContinuitySearchWindowY(Handle, value)); }
            }

            /// <summary>The spot width threshold below which spots are considered to be in an opaque section of the profile. This is in number of pixels in the video image Y direction.</summary>
            property k32u SpotTranslucentOpaqueWidth
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentOpaqueWidth(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotTranslucentOpaqueWidth(Handle, value)); }
            }

            /// <summary>The system value of the spot width threshold below which spots are considered to be in an opaque section of the profile.</summary>
            property k32u SpotTranslucentOpaqueWidthSystemValue
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentOpaqueWidthSystemValue(Handle); }
            }

            /// <summary>The minimum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.</summary>
            property k32u SpotTranslucentOpaqueWidthLimitMin
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentOpaqueWidthLimitMin(Handle); }
            }

            /// <summary>The maximum limit of the spot width threshold below which spots are considered to be in an opaque section of the profile.</summary>
            property k32u SpotTranslucentOpaqueWidthLimitMax
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentOpaqueWidthLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the spot width threshold below which spots are considered to be in an opaque section of the profile is used in translucent sorting algorithm.</summary>
            property bool IsSpotTranslucentOpaqueWidthUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotTranslucentOpaqueWidthUsed(Handle)); }
            }

            /// <summary>The spot width required to activate a translucent section in the profile. This is in number of pixels in the video image Y direction.</summary>
            property k32u SpotTranslucentTranslucentWidth
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentTranslucentWidth(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotTranslucentTranslucentWidth(Handle, value)); }
            }

            /// <summary>The system value of the spot width required to activate a translucent section in the profile.</summary>
            property k32u SpotTranslucentTranslucentWidthSystemValue
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentTranslucentWidthSystemValue(Handle); }
            }

            /// <summary>The minimum limit of the spot width required to activate a translucent section in the profile.</summary>
            property k32u SpotTranslucentTranslucentWidthLimitMin
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentTranslucentWidthLimitMin(Handle); }
            }

            /// <summary>The maximum limit of the spot width required to activate a translucent section in the profile.</summary>
            property k32u SpotTranslucentTranslucentWidthLimitMax
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentTranslucentWidthLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the spot width required to activate a translucent section in the profile is used in translucent sorting algorithm.</summary>
            property bool IsSpotTranslucentTranslucentWidthUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotTranslucentTranslucentWidthUsed(Handle)); }
            }

            /// <summary>The minimum length of a translucent section. This is in number of pixels in the video image X direction.</summary>
            property k32u SpotTranslucentMinimumLength
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentMinimumLength(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetSpotTranslucentMinimumLength(Handle, value)); }
            }

            /// <summary>The system value of the minimum length of a translucent section.</summary>
            property k32u SpotTranslucentMinimumLengthSystemValue
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentMinimumLengthSystemValue(Handle); }
            }

            /// <summary>The minimum limit of the minimum length of a translucent section.</summary>
            property k32u SpotTranslucentMinimumLengthLimitMin
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentMinimumLengthLimitMin(Handle); }
            }

            /// <summary>The maximum limit of the minimum length of a translucent section.</summary>
            property k32u SpotTranslucentMinimumLengthLimitMax
            {
                k32u get()           { return ::GoAdvanced_SpotTranslucentMinimumLengthLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the minimum length of a translucent section is used in translucent sorting algorithm.</summary>
            property bool IsSpotTranslucentMinimumLengthUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotTranslucentMinimumLengthUsed(Handle)); }
            }

            /// <summary>The threading mode options count used in translucent sorting algorithm.</summary>
            property k64s SpotTranslucentThreadingModeOptionCount
            {
                k64s get()           { return (kSize) ::GoAdvanced_SpotTranslucentThreadingModeOptionCount(Handle); }
            }

            /// <summary>Returns the threading mode option at the given index used in translucent sorting algorithm.</summary>
            /// <param name="index">The option list index to access.</param>
            GoTranslucentThreadingMode GetSpotTranslucentThreadingModeOptionAt(k64s index)
            {
                return (GoTranslucentThreadingMode) ::GoAdvanced_SpotTranslucentThreadingModeOptionAt(Handle, (kSize)index);
            }
            
            /// <summary>The threading mode used in translucent sorting algorithm.</summary>
            property GoTranslucentThreadingMode SpotTranslucentThreadingMode
            {
                GoTranslucentThreadingMode get()           { return ::GoAdvanced_SpotTranslucentThreadingMode(Handle); }
                void set(GoTranslucentThreadingMode mode)  { KCheck(::GoAdvanced_SetSpotTranslucentThreadingMode(Handle, mode)); }
            }

            /// <summary>The system value of threading mode used in translucent sorting algorithm.</summary>
            property GoTranslucentThreadingMode SpotTranslucentThreadingModeSystemValue
            {
                GoTranslucentThreadingMode get()           { return ::GoAdvanced_SpotTranslucentThreadingModeSystemValue(Handle); }
            }
            
            /// <summary>A boolean value representing whether threading mode is used in translucent sorting algorithm.</summary>
            property bool IsSpotTranslucentThreadingModeUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSpotTranslucentThreadingModeUsed(Handle)); }
            }

            /// <summary>The user defined analog camera gain value.</summary>
            property k64f CameraGainAnalog
            {
                k64f get()           { return ::GoAdvanced_CameraGainAnalog(Handle); }
                void set(k64f value)  { KCheck(::GoAdvanced_SetCameraGainAnalog(Handle, value)); }
            }

            /// <summary>The analog camera gain minimum value limit.</summary>
            property k64f CameraGainAnalogLimitMin
            {
                k64f get()           { return ::GoAdvanced_CameraGainAnalogLimitMin(Handle); }
            }

            /// <summary>The analog camera gain maximum value limit.</summary>
            property k64f CameraGainAnalogLimitMax
            {
                k64f get()           { return ::GoAdvanced_CameraGainAnalogLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the user defined analog camera gain is used.</summary>
            property bool IsCameraGainAnalogUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsCameraGainAnalogUsed(Handle)); }
            }

            /// <summary>The analog camera gain system value.</summary>
            property k64f CameraGainAnalogSystemValue
            {
                k64f get()           { return ::GoAdvanced_CameraGainAnalogSystemValue(Handle); }
            }

            /// <summary>The user defined digital camera gain value.</summary>
            property k64f CameraGainDigital
            {
                k64f get()           { return ::GoAdvanced_CameraGainDigital(Handle); }
                void set(k64f value)  { KCheck(::GoAdvanced_SetCameraGainDigital(Handle, value)); }
            }

            /// <summary>The digital camera gain minimum value limit.</summary>
            property k64f CameraGainDigitalLimitMin
            {
                k64f get()           { return ::GoAdvanced_CameraGainDigitalLimitMin(Handle); }
            }

            /// <summary>The digital camera gain maximum value limit.</summary>
            property k64f CameraGainDigitalLimitMax
            {
                k64f get()           { return ::GoAdvanced_CameraGainDigitalLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the user defined digital camera gain is used.</summary>
            property bool IsCameraGainDigitalUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsCameraGainDigitalUsed(Handle)); }
            }

            /// <summary>The digital camera gain system value.</summary>
            property k64f CameraGainDigitalSystemValue
            {
                k64f get()           { return ::GoAdvanced_CameraGainDigitalSystemValue(Handle); }
            }

            /// <summary>The dynamic sensitivity.</summary>
            property k64f DynamicSensitivity
            {
                k64f get()           { return ::GoAdvanced_DynamicSensitivity(Handle); }
                void set(k64f value)  { KCheck(::GoAdvanced_SetDynamicSensitivity(Handle, value)); }
            }

            /// <summary>The dynamic sensitivity minimum value limit.</summary>
            property k64f DynamicSensitivityLimitMin
            {
                k64f get()           { return ::GoAdvanced_DynamicSensitivityLimitMin(Handle); }
            }

            /// <summary>The dynamic sensitivity maximum value limit.</summary>
            property k64f DynamicSensitivityLimitMax
            {
                k64f get()           { return ::GoAdvanced_DynamicSensitivityLimitMax(Handle); }
            }

            /// <summary>A boolean representing whether the user defined dynamic sensitivity value is used.</summary>
            property bool IsDynamicSensitivityUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsDynamicSensitivityUsed(Handle)); }
            }

            /// <summary>The dynamic sensitivity system value.</summary>
            property k64f DynamicSensitivitySystemValue
            {
                k64f get()           { return ::GoAdvanced_DynamicSensitivitySystemValue(Handle); }
            }

            /// <summary>The dynamic threshold.</summary>
            property k32u DynamicThreshold
            {
                k32u get()           { return ::GoAdvanced_DynamicThreshold(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetDynamicThreshold(Handle, value)); }
            }

            /// <summary>The dynamic threshold minimum value limit.</summary>
            property k32u DynamicThresholdLimitMin
            {
                k32u get()           { return ::GoAdvanced_DynamicThresholdLimitMin(Handle); }
            }

            /// <summary>The dynamic threshold maximum value limit.</summary>
            property k32u DynamicThresholdLimitMax
            {
                k32u get()           { return ::GoAdvanced_DynamicThresholdLimitMax(Handle); }
            }

            /// <summary>A boolean representing whether the user defined dynamic threshold value is used.</summary>
            property bool IsDynamicThresholdUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsDynamicThresholdUsed(Handle)); }
            }

            /// <summary>The dynamic threshold system value.</summary>
            property k32u DynamicThresholdSystemValue
            {
                k32u get()           { return ::GoAdvanced_DynamicThresholdSystemValue(Handle); }
            }

            /// <summary>The gamma type.</summary>
            property GoGammaType GammaType
            {
                GoGammaType get()           { return (GoGammaType) ::GoAdvanced_GammaType(Handle); }
                void set(GoGammaType value)  { KCheck(::GoAdvanced_SetGammaType(Handle, value)); }
            }

            /// <summary>A boolean representing whether the user defined gamma type is used by the system.</summary>
            property bool IsGammaTypeUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsGammaTypeUsed(Handle)); }
            }

            /// <summary>The system's gamma type value.</summary>
            property GoGammaType GammaTypeSystemValue
            {
                GoGammaType get()           { return (GoGammaType) ::GoAdvanced_GammaTypeSystemValue(Handle); }
            }

            /// <summary>Enables or disables senstivity compensation.</summary>
            /// <remarks>NOTE: This is only applicable to 2300 B series sensors.</remarks>
            property bool SensitivityCompensationEnabled
            {
                bool get()           { return KToBool(::GoAdvanced_SensitivityCompensationEnabled(Handle)); }
                void set(bool value)  { KCheck(::GoAdvanced_EnableSensitivityCompensation(Handle, value)); }
            }

            /// <summary>A boolean representing whether the user defined sensitivity compensation is used by the system.</summary>
            property bool IsSensitivityCompensationEnabledUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSensitivityCompensationEnabledUsed(Handle)); }
            }

            /// <summary>The system's sensitivity compensation value.</summary>
            property bool SensitivityCompensationEnabledSystemValue
            {
                bool get()           { return KToBool(::GoAdvanced_SensitivityCompensationEnabledSystemValue(Handle)); }
            }

            /// <summary>The Surface Encoding type.</summary>
            property GoSurfaceEncoding SurfaceEncoding
            {
                GoSurfaceEncoding get() { return (GoSurfaceEncoding) ::GoAdvanced_SurfaceEncoding(Handle); }
                void set(GoSurfaceEncoding value) { KCheck(::GoAdvanced_SetSurfaceEncoding(Handle, value)); }
            }

            /// <summary>A boolean value representing whether the user specified surface encoding value is used.</summary>
            property bool IsSurfaceEncodingUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsSurfaceEncodingUsed(Handle)); }
            }

            /// <summary>The system surface encoding value.</summary>
            property GoSurfaceEncoding SurfaceEncodingSystemValue
            {
                GoSurfaceEncoding get()           { return (GoSurfaceEncoding) ::GoAdvanced_SurfaceEncodingSystemValue(Handle); }
            }

            /// <summary>The Surface Phase Filter type.</summary>
            property GoSurfacePhaseFilter SurfacePhaseFilter
            {
                GoSurfacePhaseFilter get() { return (GoSurfacePhaseFilter) ::GoAdvanced_SurfacePhaseFilter(Handle); }
                void set(GoSurfacePhaseFilter value) { KCheck(::GoAdvanced_SetSurfacePhaseFilter(Handle, value)); }
            }

            /// <summary>The contrast threshold.</summary>
            property k32u ContrastThreshold
            {
                k32u get()           { return ::GoAdvanced_ContrastThreshold(Handle); }
                void set(k32u value)  { KCheck(::GoAdvanced_SetContrastThreshold(Handle, value)); }
            }

            /// <summary>The minimum contrast threshold limit.</summary>
            property k32u ContrastThresholdLimitMin
            {
                k32u get()           { return ::GoAdvanced_ContrastThresholdLimitMin(Handle); }
            }

            /// <summary>The maximum contrast threshold limit.</summary>
            property k32u ContrastThresholdLimitMax
            {
                k32u get()           { return ::GoAdvanced_ContrastThresholdLimitMax(Handle); }
            }

            /// <summary>A boolean value representing whether the user specified contrast threshold value is used.</summary>
            property bool IsContrastThresholdUsed
            {
                bool get()           { return KToBool(::GoAdvanced_IsContrastThresholdUsed(Handle)); }
            }

            /// <summary>The system contrast threshold value.</summary>
            property k32u ContrastThresholdSystemValue
            {
                k32u get()           { return ::GoAdvanced_ContrastThresholdSystemValue(Handle); }
            }
        };
    }
}

#endif




























