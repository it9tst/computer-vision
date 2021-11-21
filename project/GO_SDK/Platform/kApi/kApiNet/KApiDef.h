// 
// KApiDef.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_DEF_H
#define K_API_NET_DEF_H

#include <kApi/kApiDef.h>
#include <kApi/Io/kHttpServerResponse.h>
#include "kApiNet/KApiNet.h"
#include "kApiNet/Utils/Internal/KInternalUtils.h"
#include "kApiNet/Utils/Internal/KCallbackState.h"
#include "kApiNet/Utils/Internal/KGenericArgs.h"

namespace Lmi3d
{
    namespace Zen
    {
        /// <summary>Zen API namespace.</summary>
        [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
        public ref class NamespaceDoc { }; 

        /// <summary>Represents a status code.</summary>       
        public value struct KStatus
        {
            KDeclareEnum(KStatus, kStatus)

            /// <summary>Invalid state.</summary>
            literal k32s ErrorState = kERROR_STATE;                         

            /// <summary>Item is not found.</summary>
            literal k32s ErrorNotFound = kERROR_NOT_FOUND;              

            /// <summary>Command not recognized.</summary>
            literal k32s ErrorCommand = kERROR_COMMAND;

            /// <summary>Parameter is invalid.</summary>
            literal k32s ErrorParamter = kERROR_PARAMETER;

            /// <summary>Feature not implemented.</summary>
            literal k32s ErrorUnimplemented = kERROR_UNIMPLEMENTED;

            /// <summary>Out of memory.</summary>
            literal k32s ErrorMemory = kERROR_MEMORY;

            /// <summary>Action timed out.</summary>
            literal k32s ErrorTimeout = kERROR_TIMEOUT;

            /// <summary>Buffer insufficient for data.</summary>
            literal k32s ErrorIncomplete = kERROR_INCOMPLETE;

            /// <summary>Error in stream.</summary>
            literal k32s ErrorStream = kERROR_STREAM;

            /// <summary>Resource is no longer available.</summary>
            literal k32s ErrorClosed = kERROR_CLOSED;

            /// <summary>Incompatible version.</summary>
            literal k32s ErrorVersion = kERROR_VERSION;

            /// <summary>Operation aborted.</summary>
            literal k32s ErrorAbort = kERROR_ABORT;

            /// <summary>Conflicts with existing item.</summary>
            literal k32s ErrorAlreadyExists = kERROR_ALREADY_EXISTS;

            /// <summary>Network setup/resource error.</summary>
            literal k32s ErrorNetwork = kERROR_NETWORK;

            /// <summary>Heap error (leak/double-free).</summary>
            literal k32s ErrorHeap = kERROR_HEAP;

            /// <summary>Data parsing/formatting error.</summary>
            literal k32s ErrorFormat = kERROR_FORMAT;

            /// <summary>Object is read-only (cannot be written).</summary>
            literal k32s ErrorReadOnly = kERROR_READ_ONLY;

            /// <summary>Object is write-only (cannot be read). </summary>
            literal k32s ErrorWriteOnly = kERROR_WRITE_ONLY;

            /// <summary>Agent is busy (cannot service request).</summary>
            literal k32s ErrorBusy = kERROR_BUSY;

            /// <summary>State conflicts with another object.</summary>
            literal k32s ErrorConflict = kERROR_CONFLICT;

            /// <summary>Generic error reported by underlying OS.</summary>
            literal k32s ErrorOs = kERROR_OS;

            /// <summary>Hardware device error.</summary>
            literal k32s ErrorDevice = kERROR_DEVICE;

            /// <summary>Resource is already fully utilized.</summary>
            literal k32s ErrorFull = kERROR_FULL;

            /// <summary>Operation is in progress, but not yet complete.</summary>
            literal k32s ErrorInProgress = kERROR_IN_PROGRESS;

            /// <summary>General error.</summary>
            literal k32s Error = kERROR;

            /// <summary>Operation successful.</summary>
            literal k32s Ok = kOK;

            property String^ Name
            {
                String^ get() { return KToString(kStatus_Name(Value)); }
            }
        };     
  
        /// <summary>Represents an 8-bit unsigned integer.</summary>
        public value struct K8u
        {
            KDeclarePrimative(K8u, k8u)

            /// <summary>K8u minimum value.</summary>
            literal k8u Min = k8U_MIN; 

            /// <summary>K8u maximum value.</summary>
            literal k8u Max = k8U_MAX;

            /// <summary>K8u invalid value.</summary>
            literal k8u Null = k8U_NULL;
        };

        /// <summary>Represents an 8-bit signed integer.</summary>
        public value struct K8s
        {
            KDeclarePrimative(K8s, k8s)

            /// <summary>K8s minimum value.</summary>
            literal k8s Min = k8S_MIN; 

            /// <summary>K8s maximum value.</summary>
            literal k8s Max = k8S_MAX;

            /// <summary>K8s invalid value.</summary>
            literal k8s Null = k8S_NULL;

        };

        /// <summary>Represents a 16-bit unsigned integer.</summary>
        public value struct K16u
        {
            KDeclarePrimative(K16u, k16u)

            /// <summary>K16u minimum value.</summary>
            literal k16u Min = k16U_MIN; 

            /// <summary>K16u maximum value.</summary>
            literal k16u Max = k16U_MAX;

            /// <summary>K16u invalid value.</summary>
            literal k16u Null = k16U_NULL;
        };

        /// <summary>Represents a 16-bit signed integer.</summary>
        public value struct K16s
        {
            KDeclarePrimative(K16s, k16s)

            /// <summary>K16s minimum value.</summary>
            literal k16s Min = k16S_MIN; 

            /// <summary>K16s maximum value.</summary>
            literal k16s Max = k16S_MAX;

            /// <summary>K16s invalid value.</summary>
            literal k16s Null = k16S_NULL;
        };

        /// <summary>Represents a 32-bit unsigned integer.</summary>
        public value struct K32u
        {
            KDeclarePrimative(K32u, k32u)

            /// <summary>K32u minimum value.</summary>
            literal k32u Min = k32U_MIN; 

            /// <summary>K32u maximum value.</summary>
            literal k32u Max = k32U_MAX;

            /// <summary>K32u invalid value.</summary>
            literal k32u Null = k32U_NULL;
        };

        /// <summary>Represents a 32-bit signed integer.</summary>
        public value struct K32s
        {
            KDeclarePrimative(K32s, k32s)

            /// <summary>K32s minimum value.</summary>
            literal k32s Min = k32S_MIN; 

            /// <summary>K32s maximum value.</summary>
            literal k32s Max = k32S_MAX;

            /// <summary>K32s invalid value.</summary>
            literal k32s Null = k32S_NULL;
        };

        /// <summary>Represents a 64-bit unsigned integer.</summary>
        public value struct K64u
        {
            KDeclarePrimative(K64u, k64u)

            /// <summary>K64u minimum value.</summary>
            literal k64u Min = k64U_MIN; 

            /// <summary>K64u maximum value.</summary>
            literal k64u Max = k64U_MAX;

            /// <summary>K64u invalid value.</summary>
            literal k64u Null = k64U_NULL;
        };

        /// <summary>Represents a 64-bit signed integer.</summary>
        public value struct K64s
        {
            KDeclarePrimative(K64s, k64s)

            /// <summary>K64s minimum value.</summary>
            literal k64s Min = k64S_MIN; 

            /// <summary>K64s maximum value.</summary>
            literal k64s Max = k64S_MAX;

            /// <summary>K64s invalid value.</summary>
            literal k64s Null = k64S_NULL;
        };

        /// <summary>Represents an unsigned integer that can hold a pointer address.</summary>
        public value struct KSize
        {
            KDeclarePrimative(KSize, kSize)

            /// <summary>KSize minimum value.</summary>
            literal kSize Min = 0; 

            /// <summary>KSize maximum value.</summary>
            literal kSize Max = kSIZE_MAX;

            /// <summary>KSize invalid value.</summary>
            literal kSize Null = kSIZE_NULL;
        };

        /// <summary>Represents a signed integer that can hold a pointer address.</summary>
        public value struct KSSize
        {
            KDeclarePrimative(KSSize, kSSize)

            /// <summary>KSSize minimum value.</summary>
            literal kSSize Min = kSSIZE_MIN; 

            /// <summary>KSSize maximum value.</summary>
            literal kSSize Max = kSSIZE_MAX;

            /// <summary>KSSize invalid value.</summary>
            literal kSSize Null = kSSIZE_NULL;
        };

        /// <summary>Represents a 32-bit floating-point value.</summary>
        public value struct K32f
        {
            KDeclarePrimative(K32f, k32f)

            /// <summary>K32f minimum value.</summary>
            literal k32f Min = k32F_MIN; 

            /// <summary>K32f maximum value.</summary>
            literal k32f Max = k32F_MAX;

            /// <summary>K32f invalid value.</summary>
            literal k32f Null = k32F_NULL;
        };

        /// <summary>Represents a 32-bit floating-point value.</summary>
        public value struct K64f
        {
            KDeclarePrimative(K64f, k64f)          

            /// <summary>K64f minimum value.</summary>
            literal k64f Min = k64F_MIN; 

            /// <summary>K64f maximum value.</summary>
            literal k64f Max = k64F_MAX;

            /// <summary>K64f invalid value.</summary>
            literal k64f Null = k64F_NULL;
        };

        /// <summary>Represents a byte on the current platform.</summary>
        public value struct KByte
        {
            KDeclarePrimative(KByte, kByte)
        };

        /// <summary>Represents a single unit (byte) in a UTF-8 character. </summary>
        public value struct KChar
        {
            KDeclarePrimative(KChar, kChar)
        };

        /// <summary>Represents a boolean value.</summary>
        public value struct KBool
        {
            KDeclarePrimative(KBool, kBool)

            /// <summary>KBool false.</summary>
            literal kBool False = kFALSE;

            /// <summary>KBool true.</summary>
            literal kBool True = kTRUE; 

            static operator bool(KBool b)
            {
                return (b.Value == True) ? true : false;
            }

            static operator KBool(bool b)
            {
                return b ? KBool(True) : KBool(False);
            }
        };

        /// <summary>Static collection of special constants related to timeouts.</summary>
        public ref class KTimeout
        {
        public:
            literal k64s Infinite = kINFINITE;

        private:
            KTimeout()
            { }

        };

        /// <summary>Represents a version number. </summary>
        public value struct KVersion
        {
            KDeclarePrimative(KVersion, kVersion)

            KVersion(k32s major, k32s minor, k32s release, k32s build)
            {
                Value = kVersion_Create(major, minor, release, build); 
            }

            /// <summary>Gets the major part of a version number.</summary>
            property k32s Major { k32s get() { return kVersion_Major(Value); } }

            /// <summary>Gets the minor part of a version number.</summary>
            property k32s Minor { k32s get() { return kVersion_Minor(Value); } }

            /// <summary>Gets the release part of a version number.</summary>
            property k32s Release { k32s get() { return kVersion_Release(Value); } }

            /// <summary>Gets the build part of a version number.</summary>
            property k32s Build { k32s get() { return kVersion_Build(Value); } }

            /// <summary>Parses a version from a formatted string.</summary>
            /// <param name="string">Formatted string (e.g., "1.2.3.4")</param>
            /// <returns>Version number.</returns>
            static KVersion Parse(String^ string)
            {
                kText64 text; 
                kVersion version; 

                KToText(string, text, kCountOf(text)); 
                KCheck(kVersion_Parse(&version, text));

                return KVersion(version);
            }

            /// <summary>Formats a version to a string.</summary>
            /// <returns>Formatted string (e.g., "1.2.3.4")</returns>
            String^ Format()
            {
                kText64 text;        

                KCheck(kVersion_Format(Value, text, kCountOf(text)));
                
                return KToString(text);
            }
        };

       /// <summary>Represents the byte-ordering of primative data types.</summary>       
        public value struct KEndianness
        {
            KDeclareEnum(KEndianness, kEndianness)

            /// <summary>Item is not found.</summary>
            literal k32s Little = kENDIANNESS_LITTLE;              

            /// <summary>Command not recognized.</summary>
            literal k32s Big = kENDIANNESS_BIG;    

            /// <summary>Reports the endianness of the current platform.</summary>
            KEndianness Host()
            {
                return (KEndianness) kEndianness_Host(); 
            }

            /// <summary>Reports whether byte ordering must be reversed to be consistent with the current platform.</summary>
            bool ShouldSwap(KEndianness endianness)
            {
                return KToBool(kEndianness_ShouldReverse((kEndianness)endianness)); 
            }
        };     

        /// <summary>Represents a pointer.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kPointer))]
        public value struct KPointer
        {
            KDeclareStruct(KPointer, kPointer)

            KPointer(kPointer value)
            {
                Value = (kSize)value; 
            }

            /// <summary>Pointer value.</summary>
            [FieldOffset(0)]
            kSize Value;
        };

        /// <summary>Represents a value type with zero size.</summary>        
        public value struct KVoid
        {
            KDeclareVoid(KVoid, kVoid)
        };

        /// <summary>Represents a string encoded as a null-terminated, UTF-8 character sequence of up to 16-bytes.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kText16))]
        public value struct KText16
        {
            KDeclareText(KText16, kText16)
        };

        /// <summary>Represents a string encoded as a null-terminated, UTF-8 character sequence of up to 32-bytes.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kText32))]
        public value struct KText32
        {
            KDeclareText(KText32, kText32)
        };

        /// <summary>Represents a string encoded as a null-terminated, UTF-8 character sequence of up to 64-bytes.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kText64))]
        public value struct KText64
        {
            KDeclareText(KText64, kText64)
        };

        /// <summary>Represents a string encoded as a null-terminated, UTF-8 character sequence of up to 128-bytes.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kText128))]
        public value struct KText128
        {
            KDeclareText(KText128, kText128)          
        };

        /// <summary>Represents a string encoded as a null-terminated, UTF-8 character sequence of up to 256-bytes.</summary>
        [StructLayout(LayoutKind::Explicit, Size = sizeof(kText256))]
        public value struct KText256
        {
            KDeclareText(KText256, kText256)
        }; 

        /// <summary>A delegate that matches the signature of kCallbackFx from the underlying Zen library.</summary>
        public delegate kStatus KCallbackFx(kPointer receiver, kPointer sender, kPointer args);

        namespace Data
        {
            /// <summary>Zen data namespace.</summary>
            [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
            public ref class NamespaceDoc { };

            /// <summary>32-bit color pixel structure (B/G/R/A).</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kArgb))]
            public value struct KArgb
            {
                KDeclareStruct(KArgb, kArgb)

                /// <summary>Initializes a new KArgb instance.</summary>
                KArgb(k8u a, k8u r, k8u g, k8u b)
                : A(a), R(r), G(g), B(b)
                {}

                /// <summary>Blue component value.</summary>
                [FieldOffset(offsetof(kArgb, b))]
                k8u B;      

                /// <summary>Green component value.</summary>
                [FieldOffset(offsetof(kArgb, g))]
                k8u G;

                /// <summary>Red component value.</summary>
                [FieldOffset(offsetof(kArgb, r))]
                k8u R;

                /// <summary>Alpha component value.</summary>
                [FieldOffset(offsetof(kArgb, a))]
                k8u A;
            };

            /// <summary>32-bit color pixel structure (B/G/R/X).</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRgb))]
            public value struct KRgb
            {
                KDeclareStruct(KRgb, kRgb)

                /// <summary>Initializes a new KRgb instance.</summary>
                KRgb(k8u r, k8u g, k8u b)
                : R(r), G(g), B(b), X(0)
                {}

                /// <summary>Blue component value.</summary>
                [FieldOffset(offsetof(kRgb, b))]
                k8u B;      

                /// <summary>Green component value.</summary>
                [FieldOffset(offsetof(kRgb, g))]
                k8u G;

                /// <summary>Red component value.</summary>
                [FieldOffset(offsetof(kRgb, r))]
                k8u R;

                /// <summary>Alpha component value.</summary>
                [FieldOffset(offsetof(kRgb, x))]
                k8u X;
            };

            /// <summary>Image color filter array type.</summary>
            public value struct KCfa
            {
                KDeclareEnum(KCfa, kCfa)

                /// <summary>No color filter array.</summary>
                literal k32s Null = kCFA_NONE;

                /// <summary>Bayer filter, BG/GR.</summary>
                literal k32s BayerBggr = kCFA_BAYER_BGGR;

                /// <summary>Bayer filter, GB/RG.</summary>
                literal k32s BayerGbrg = kCFA_BAYER_GBRG;

                /// <summary>Bayer filter, RG/GB.</summary>
                literal k32s BayerRggb = kCFA_BAYER_RGGB;

                /// <summary>Bayer filter, GR/BG.</summary>
                literal k32s BayerGrbg = kCFA_BAYER_GRBG;
            };

            /// <summary>Comparison type.</summary>
            public value struct KComparison
            {
                KDeclareEnum(KComparison, kComparison)

                /// <summary>Is equal.</summary>
                literal k32s Equal = kCOMPARISON_EQ;

                /// <summary>Is not equal.</summary>
                literal k32s NotEqual = kCOMPARISON_NEQ;

                /// <summary>Is less than.</summary>
                literal k32s LessThan = kCOMPARISON_LT;

                /// <summary>Is less than or equal.</summary>
                literal k32s LessThanOrEqual = kCOMPARISON_LTE;

                /// <summary>Is greater than.</summary>
                literal k32s GreaterThan = kCOMPARISON_GT;

                /// <summary>Is greater than or equal.</summary>
                literal k32s GreaterThanOrEqual = kCOMPARISON_GTE;              
            };

            /// <summary>Pixel format descriptor.</summary>
            public value struct KPixelFormat
            {
                KDeclareEnum(KPixelFormat, kPixelFormat)

                /// <summary>Unknown pixel format.</summary>
                literal k32s Null = kPIXEL_FORMAT_NULL;

                /// <summary>8-bit greyscale (K8u).</summary>
                literal k32s Format8bppGreyscale = kPIXEL_FORMAT_8BPP_GREYSCALE;

                /// <summary>8-bit with color filter array (K8u).</summary>
                literal k32s Format8bppCfa= kPIXEL_FORMAT_8BPP_CFA;

                /// <summary>8-bits-per-channel color with 4 channels (blue/green/red/unused) (KRgb).</summary>
                literal k32s Format8bpcBgrx = kPIXEL_FORMAT_8BPC_BGRX;

                /// <summary>16-bit greyscale (K16u).</summary>
                literal k32s Format16bppGreyscale = kPIXEL_FORMAT_16BPP_GREYSCALE;
            };

            /// <summary>2D point structure with 16-bit signed integer fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint16s))]
            public value struct KPoint16s
            {
                KDeclareStruct(KPoint16s, kPoint16s)

                /// <summary>Initializes a new KPoint16s instance.</summary>
                KPoint16s(k16s x, k16s y)
                    : X(x), Y(y)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint16s, x))]
                k16s X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint16s, y))]
                k16s Y;

                String^ ToString() override
                {
                    return X + "," + Y;
                }
            };

            /// <summary>2D point structure with 32-bit signed integer fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint32s))]
            public value struct KPoint32s
            {
                KDeclareStruct(KPoint32s, kPoint32s)

                /// <summary>Initializes a new KPoint32s instance.</summary>
                KPoint32s(k32s x, k32s y)
                    : X(x), Y(y)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint32s, x))]
                k32s X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint32s, y))]
                k32s Y;

                String^ ToString() override
                {
                    return X + "," + Y;
                }
            };

            /// <summary>2D point structure with 32-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint32f))]
            public value struct KPoint32f
            {
                KDeclareStruct(KPoint32f, kPoint32f)

                /// <summary>Initializes a new KPoint32f instance.</summary>
                KPoint32f(k32f x, k32f y)
                    : X(x), Y(y)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint32f, x))]
                k32f X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint32f, y))]
                k32f Y;

                String^ ToString() override
                {
                    return X + "," + Y;
                }
            };

            /// <summary>2D point structure with 64-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint64f))]
            public value struct KPoint64f
            {
                KDeclareStruct(KPoint64f, kPoint64f)

                /// <summary>Initializes a new KPoint64f instance.</summary>
                KPoint64f(k64f x, k64f y)
                    : X(x), Y(y)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint64f, x))]
                k64f X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint64f, y))]
                k64f Y;

                String^ ToString() override
                {
                    return X + "," + Y;
                }
            };

            /// <summary>3D point structure with 16-bit signed integer fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint3d16s))]
            public value struct KPoint3d16s
            {
                KDeclareStruct(KPoint3d16s, kPoint3d16s)

                /// <summary>Initializes a new KPoint3d16s instance.</summary>
                KPoint3d16s(k16s x, k16s y, k16s z)
                    : X(x), Y(y), Z(z)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d16s, x))]
                k16s X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d16s, y))]
                k16s Y;

                /// <summary>Z-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d16s, z))]
                k16s Z;

                String^ ToString() override
                {
                    return X + "," + Y + "," + Z;
                }
            };

            /// <summary>3D point structure with 32-bit signed integer fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint3d32s))]
            public value struct KPoint3d32s
            {
                KDeclareStruct(KPoint3d32s, kPoint3d32s)

                /// <summary>Initializes a new KPoint3d32s instance.</summary>
                KPoint3d32s(k32s x, k32s y, k32s z)
                    : X(x), Y(y), Z(z)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32s, x))]
                k32s X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32s, y))]
                k32s Y;

                /// <summary>Z-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32s, z))]
                k32s Z;

                String^ ToString() override
                {
                    return X + "," + Y + "," + Z;
                }  
            };

            /// <summary>3D point structure with 32-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint3d32f))]
            public value struct KPoint3d32f
            {
                KDeclareStruct(KPoint3d32f, kPoint3d32f)

                /// <summary>Initializes a new KPoint3d32f instance.</summary>
                KPoint3d32f(k32f x, k32f y, k32f z)
                    : X(x), Y(y), Z(z)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32f, x))]
                k32f X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32f, y))]
                k32f Y;

                /// <summary>Z-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d32f, z))]
                k32f Z;

                String^ ToString() override
                {
                    return X + "," + Y + "," + Z;
                }
            };

            /// <summary>3D point structure with 64-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kPoint3d64f))]
            public value struct KPoint3d64f
            {
                KDeclareStruct(KPoint3d64f, kPoint3d64f)

                /// <summary>Initializes a new KPoint3d64f instance.</summary>
                KPoint3d64f(k64f x, k64f y, k64f z)
                    : X(x), Y(y), Z(z)
                {}

                /// <summary>X-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d64f, x))]
                k64f X;

                /// <summary>Y-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d64f, y))]
                k64f Y;

                /// <summary>Z-coordinate value.</summary>
                [FieldOffset(offsetof(kPoint3d64f, z))]
                k64f Z;

                String^ ToString() override
                {
                    return X + "," + Y + "," + Z;
                }
            };

            /// <summary>Rectangle structure with 16-bit signed integer fields. </summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRect16s))]
            public value struct KRect16s
            {
                KDeclareStruct(KRect16s, kRect16s)

                /// <summary>Initializes a new KRect16s instance.</summary>
                KRect16s(k16s x, k16s y, k16s width, k16s height)
                    : X(x), Y(y), Width(width), Height(height)
                {}

                /// <summary>X-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect16s, x))]
                k16s X;

                /// <summary>Y-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect16s, y))]
                k16s Y;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRect16s, width))]
                k16s Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRect16s, height))]
                k16s Height;
            };

            /// <summary>Rectangle structure with 32-bit signed integer fields. </summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRect32s))]
            public value struct KRect32s
            {
                KDeclareStruct(KRect32s, kRect32s)

                /// <summary>Initializes a new KRect32s instance.</summary>
                KRect32s(k32s x, k32s y, k32s width, k32s height)
                    : X(x), Y(y), Width(width), Height(height)
                {}

                /// <summary>X-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect32s, x))]
                k32s X;

                /// <summary>Y-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect32s, y))]
                k32s Y;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRect32s, width))]
                k32s Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRect32s, height))]
                k32s Height;
            };

            /// <summary>Rectangle structure with 32-bit floating-point fields. </summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRect32f))]
            public value struct KRect32f
            {
                KDeclareStruct(KRect32f, kRect32f)

                /// <summary>Initializes a new KRect32f instance.</summary>
                KRect32f(k32f x, k32f y, k32f width, k32f height)
                    : X(x), Y(y), Width(width), Height(height)
                {}

                /// <summary>X-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect32f, x))]
                k32f X;

                /// <summary>Y-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect32f, y))]
                k32f Y;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRect32f, width))]
                k32f Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRect32f, height))]
                k32f Height;
            };

            /// <summary>Rectangle structure with 64-bit floating-point fields. </summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRect64f))]
            public value struct KRect64f
            {
                KDeclareStruct(KRect64f, kRect64f)

                /// <summary>Initializes a new KRect64f instance.</summary>
                KRect64f(k64f x, k64f y, k64f width, k64f height)
                    : X(x), Y(y), Width(width), Height(height)
                {}

                /// <summary>X-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect64f, x))]
                k64f X;

                /// <summary>Y-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect64f, y))]
                k64f Y;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRect64f, width))]
                k64f Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRect64f, height))]
                k64f Height;
            };

            /// <summary>Rectangular cuboid structure with 64-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRect3d64f))]
            public value struct KRect3d64f
            {
                KDeclareStruct(KRect3d64f, kRect3d64f)

                /// <summary>Initializes a new KRect3d64f instance.</summary>
                KRect3d64f(k64f x, k64f y, k64f z, k64f width, k64f height, k64f depth)
                    : X(x), Y(y), Z(z), Width(width), Height(height), Depth(depth)
                {}

                /// <summary>X-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect3d64f, x))]
                k64f X;

                /// <summary>Y-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect3d64f, y))]
                k64f Y;

                /// <summary>Z-coordinate of the origin.</summary>
                [FieldOffset(offsetof(kRect3d64f, z))]
                k64f Z;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRect3d64f, width))]
                k64f Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRect3d64f, height))]
                k64f Height;

                /// <summary>Depth of the rectangle.</summary>
                [FieldOffset(offsetof(kRect3d64f, depth))]
                k64f Depth;
            };

            /// <summary>Rotated rectangle structure with 32-bit signed integer fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRotatedRect32s))]
            public value struct KRotatedRect32s
            {
                KDeclareStruct(KRotatedRect32s, kRotatedRect32s)

                /// <summary>Initializes a new KRotatedRect32s instance.</summary>
                KRotatedRect32s(k32s xc, k32s yc, k32s width, k32s height, k32s angle)
                    : Xc(xc), Yc(yc), Width(width), Height(height), Angle(angle)
                {}

                /// <summary>X-coordinate of the center.</summary>
                [FieldOffset(offsetof(kRotatedRect32s, xc))]
                k32s Xc;

                /// <summary>Y-coordinate of the center.</summary>
                [FieldOffset(offsetof(kRotatedRect32s, yc))]
                k32s Yc;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32s, width))]
                k32s Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32s, height))]
                k32s Height;

                /// <summary>Rotation angle of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32s, angle))]
                k32s Angle;
            };

            /// <summary>Rotated rectangle structure with 32-bit floating-point fields.</summary>
            [StructLayout(LayoutKind::Explicit, Size = sizeof(kRotatedRect32f))]
            public value struct KRotatedRect32f
            {
                KDeclareStruct(KRotatedRect32f, kRotatedRect32f)

                /// <summary>Initializes a new KRotatedRect32f instance.</summary>
                KRotatedRect32f(k32f xc, k32f yc, k32f width, k32f height, k32f angle)
                    : Xc(xc), Yc(yc), Width(width), Height(height), Angle(angle)
                {}

                /// <summary>X-coordinate of the center.</summary>
                [FieldOffset(offsetof(kRotatedRect32f, xc))]
                k32f Xc;

                /// <summary>Y-coordinate of the center.</summary>
                [FieldOffset(offsetof(kRotatedRect32f, yc))]
                k32f Yc;

                /// <summary>Width of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32f, width))]
                k32f Width;

                /// <summary>Height of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32f, height))]
                k32f Height;

                /// <summary>Rotation angle of the rectangle.</summary>
                [FieldOffset(offsetof(kRotatedRect32f, angle))]
                k32f Angle;
            };
        }


        namespace Io
        {
            /// <summary>Zen I/O namespace.</summary>
            [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
            public ref class NamespaceDoc { };

            /// <summary>Represents the point of reference for a stream seek operation.</summary>
            public value struct KSeekOrigin
            {
                KDeclareEnum(KSeekOrigin, kSeekOrigin)

                /// <summary>Seek relative to the start of stream.</summary>
                literal k32s Begin = kSEEK_ORIGIN_BEGIN;

                /// <summary>Seek relative to the current position.</summary>
                literal k32s Current = kSEEK_ORIGIN_CURRENT;

                /// <summary>Seek relative to the end of stream.</summary>
                literal k32s End = kSEEK_ORIGIN_END;
            };

            /// <summary>Flags that control how a file is opened.</summary>
            public value struct KFileMode
            {
                KDeclareEnum(KFileMode, kFileMode)

                /// <summary>Open the file with permission to read.</summary>
                literal k32s Read = kFILE_MODE_READ;

                /// <summary>Open the file with permission to write.</summary>
                literal k32s Write = kFILE_MODE_WRITE;

                /// <summary>Preserve contents when opened for writing.</summary>
                literal k32s Update = kFILE_MODE_UPDATE;
            };

            /// <summary>HTTP status code.</summary>
            public value struct KHttpStatus
            {
                KDeclareEnum(KHttpStatus, kHttpStatus)

                /// <summary>Continue.</summary>
                literal k32s Continue = kHTTP_STATUS_CONTINUE;
                
                ///<summary>Switching protocols.</summary>
                literal k32s SwitchingProtocols = kHTTP_STATUS_SWITCHING_PROTOCOLS;
                
                ///<summary>OK.</summary>
                literal k32s Ok = kHTTP_STATUS_OK;
                
                ///<summary>Created.</summary>
                literal k32s Created = kHTTP_STATUS_CREATED;
                
                ///<summary>Accepted.</summary>
                literal k32s Accepted = kHTTP_STATUS_ACCEPTED;
                
                ///<summary>Non-authoritative information.</summary>
                literal k32s NonAuthoritative = kHTTP_STATUS_NON_AUTHORITATIVE;
                
                ///<summary>No content.</summary>
                literal k32s NoContent = kHTTP_STATUS_NO_CONTENT;
                
                ///<summary>Reset content.</summary>
                literal k32s ResetContent = kHTTP_STATUS_RESET_CONTENT;
                
                ///<summary>Partial content.</summary>
                literal k32s PartialContent = kHTTP_STATUS_PARTIAL_CONTENT;
                
                ///<summary>Multiple choices.</summary>
                literal k32s MultipleChoices = kHTTP_STATUS_MULTIPLE_CHOICES;
                
                ///<summary>Moved permanently.</summary>
                literal k32s MovedPermanently = kHTTP_STATUS_MOVED_PERMANENTLY;
                
                ///<summary>Found.</summary>
                literal k32s Found = kHTTP_STATUS_FOUND;
                
                ///<summary>See other.</summary>
                literal k32s SeeOther = kHTTP_STATUS_SEE_OTHER;
                
                ///<summary>Not modified.</summary>
                literal k32s NotModified = kHTTP_STATUS_NOT_MODIFIED;
                
                ///<summary>Use proxy.</summary>
                literal k32s UseProxy = kHTTP_STATUS_USE_PROXY;
                
                ///<summary>Temporary redirect.</summary>
                literal k32s TemporaryRedirect = kHTTP_STATUS_TEMPORARY_REDIRECT;
                
                ///<summary>Bad request.</summary>
                literal k32s BadRequest = kHTTP_STATUS_BAD_REQUEST;
                
                ///<summary>Unauthorized.</summary>
                literal k32s Unauthorized = kHTTP_STATUS_UNAUTHORIZED;
                
                ///<summary>Payment required.</summary>
                literal k32s PaymentRequired = kHTTP_STATUS_PAYMENT_REQUIRED;
                
                ///<summary>Forbidden.</summary>
                literal k32s Forbidden = kHTTP_STATUS_FORBIDDEN;
                
                ///<summary>Not found.</summary>
                literal k32s NotFound = kHTTP_STATUS_NOT_FOUND;
                
                ///<summary>Method not allowed.</summary>
                literal k32s MethodNotAllowed = kHTTP_STATUS_METHOD_NOT_ALLOWED;
                
                ///<summary>Not acceptable.</summary>
                literal k32s NotAcceptable = kHTTP_STATUS_NOT_ACCEPTABLE;
                
                ///<summary>Proxy authentication required.</summary>
                literal k32s ProxyAuthRequired = kHTTP_STATUS_PROXY_AUTH_REQUIRED;
                
                ///<summary>Request timeout.</summary>
                literal k32s RequestTimeout = kHTTP_STATUS_REQUEST_TIMEOUT;
                
                ///<summary>Conflict.</summary>
                literal k32s Conflict = kHTTP_STATUS_CONFLICT;
                
                ///<summary>Gone.</summary>
                literal k32s Gone = kHTTP_STATUS_GONE;
                
                ///<summary>Length required.</summary>
                literal k32s LengthRequired = kHTTP_STATUS_LENGTH_REQUIRED;
                
                ///<summary>Precondition failed.</summary>
                literal k32s PreconditionFailed = kHTTP_STATUS_PRECONDITION_FAILED;
                
                ///<summary>Request entity too large.</summary>
                literal k32s RequestEntitySize = kHTTP_STATUS_REQUEST_ENTITY_SIZE;
                
                ///<summary>Request URI size too large.</summary>
                literal k32s RequestUriSize = kHTTP_STATUS_REQUEST_URI_SIZE;
                
                ///<summary>Unsupported media type.</summary>
                literal k32s UnsupportedMediaType = kHTTP_STATUS_UNSUPPORTED_MEDIA_TYPE;
                
                ///<summary>Requested range not satisfiable.</summary>
                literal k32s InvalidRange = kHTTP_STATUS_INVALID_RANGE;
                
                ///<summary>Expectation failed.</summary>
                literal k32s ExpectationFailed = kHTTP_STATUS_EXPECTATION_FAILED;
                
                ///<summary>Internal server error.</summary>
                literal k32s InternalServerError = kHTTP_STATUS_INTERNAL_SERVER_ERROR;
                
                ///<summary>Not implemented.</summary>
                literal k32s NotImplemented = kHTTP_STATUS_NOT_IMPLEMENTED;
                
                ///<summary>Bad gateway.</summary>
                literal k32s BadGateway = kHTTP_STATUS_BAD_GATEWAY;
                
                ///<summary>Service unavailable.</summary>
                literal k32s ServiceUnavailable = kHTTP_STATUS_SERVICE_UNAVAILABLE;
                
                ///<summary>Gateway timeout.</summary>
                literal k32s GatewayTimeout = kHTTP_STATUS_GATEWAY_TIMEOUT;
                
                ///<summary>HTTP version not supported.</summary>
                literal k32s UnsupportedVersion = kHTTP_STATUS_UNSUPPORTED_VERSION;

            };

            /// <summary>Type of compression algorithm.</summary>
            public value struct KCompressionType
            {
                KDeclareEnum(KCompressionType, kCompressionType)

                /// <summary>
                /// None. 
                /// </summary>
                literal k32s Null = kCOMPRESSION_TYPE_NULL;

                /// <summary>
                /// Zstandard compression.
                /// </summary>
                literal k32s ZStd = kCOMPRESSION_TYPE_ZSTD;
            };

            /// <summary>Preset compression levels.</summary>
            public value struct KCompressionPreset
            {
                KDeclareEnum(KCompressionPreset, kCompressionPreset)

                /// <summary>
                /// Minimum compression density supported by algorithm (not recommend). 
                /// </summary>
                literal k32s Min = kCOMPRESSION_PRESET_MIN;

                /// <summary>
                /// Recommended setting for fast compression.
                /// </summary>
                literal k32s Fast = kCOMPRESSION_PRESET_FAST;
                    
                /// <summary>
                /// Recommended setting for a balance of speed and density.
                /// </summary>
                literal k32s Default = kCOMPRESSION_PRESET_DEFAULT;

                /// <summary>
                /// Recommended setting for dense compression.
                /// </summary>
                literal k32s Dense = kCOMPRESSION_PRESET_DENSE;

                /// <summary>
                /// Maximum compression density supported by algorithm (not recommended).
                /// </summary>
                literal k32s Max = kCOMPRESSION_PRESET_MAX;
            };

            /// <summary>Represents alignment options for allocations.</summary>
            public value struct KMemoryAlignment
            {
                KDeclareEnum(KMemoryAlignment, kMemoryAlignment)

                /// <summary>
                /// 8 bytes alignment.
                /// </summary>
                literal k32s Bytes8 = kMEMORY_ALIGNMENT_8;

                /// <summary>
                /// 16 bytes alignment.
                /// </summary>
                literal k32s Bytes16 = kMEMORY_ALIGNMENT_16;

                /// <summary>
                /// 32 bytes alignment.
                /// </summary>
                literal k32s Bytes32 = kMEMORY_ALIGNMENT_32;

                /// <summary>
                /// 64 bytes alignment.
                /// </summary>
                literal k32s Bytes64 = kMEMORY_ALIGNMENT_64;

                /// <summary>
                /// 128 bytes alignment.
                /// </summary>
                literal k32s Bytes128 = kMEMORY_ALIGNMENT_128;

                /// <summary>
                /// 256 bytes alignment.
                /// </summary>
                literal k32s Bytes256 = kMEMORY_ALIGNMENT_256;

                /// <summary>
                /// 512 bytes alignment.
                /// </summary>
                literal k32s Bytes512 = kMEMORY_ALIGNMENT_512;

                /// <summary>
                /// 1024 bytes alignment.
                /// </summary>
                literal k32s Bytes1024 = kMEMORY_ALIGNMENT_1024;

                /// <summary>
                /// 2048 bytes alignment.
                /// </summary>
                literal k32s Bytes2048 = kMEMORY_ALIGNMENT_2048;

                /// <summary>
                /// 4096 bytes alignment.
                /// </summary>
                literal k32s Bytes4096 = kMEMORY_ALIGNMENT_4096;
            };
        }

        namespace Threads
        {
            /// <summary>Zen threads namespace.</summary>
            [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
            public ref class NamespaceDoc { };
        }

        namespace Utils
        {
            /// <summary>Zen utilities namespace.</summary>
            [System::Runtime::CompilerServices::CompilerGeneratedAttribute()]
            public ref class NamespaceDoc { };
        }     
     }
}

#endif
