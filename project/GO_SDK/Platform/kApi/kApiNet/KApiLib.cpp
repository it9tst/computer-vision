//
// KApiLib.cpp
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#include "kApiNet/KApiLib.h"
#include "kApiNet/KAlloc.h"
#include "kApiNet/KApiDef.h"
#include "kApiNet/KAssembly.h"
#include "kApiNet/KException.h"
#include "kApiNet/KType.h"
#include "kApiNet/KValue.h"
#include "kApiNet/Crypto/KCipher.h"
#include "kApiNet/Crypto/KCipherStream.h"
#include "kApiNet/Crypto/KBlowfishCipher.h"
#include "kApiNet/Crypto/KHash.h"
#include "kApiNet/Crypto/KSha1Hash.h"
#include "kApiNet/Data/KArray1.h"
#include "kApiNet/Data/KArray2.h"
#include "kApiNet/Data/KArray3.h"
#include "kApiNet/Data/KArrayList.h"
#include "kApiNet/Data/KBox.h"
#include "kApiNet/Data/KCollection.h"
#include "kApiNet/Data/KImage.h"
#include "kApiNet/Data/KList.h"
#include "kApiNet/Data/KMap.h"
#include "kApiNet/Data/KMath.h"
#include "kApiNet/Data/KQueue.h"
#include "kApiNet/Data/KString.h"
#include "kApiNet/Data/KXml.h"
#include "kApiNet/Io/KDat5Serializer.h"
#include "kApiNet/Io/KDat6Serializer.h"
#include "kApiNet/Io/KDirectory.h"
#include "kApiNet/Io/KFile.h"
#include "kApiNet/Io/KHttpServer.h"
#include "kApiNet/Io/KHttpServerChannel.h"
#include "kApiNet/Io/KHttpServerRequest.h"
#include "kApiNet/Io/KHttpServerResponse.h"
#include "kApiNet/Io/KMemory.h"
#include "kApiNet/Io/KNetwork.h"
#include "kApiNet/Io/KPath.h"
#include "kApiNet/Io/KPipeStream.h"
#include "kApiNet/Io/KSerializer.h"
#include "kApiNet/Io/KSocket.h"
#include "kApiNet/Io/KStream.h"
#include "kApiNet/Io/KTcpClient.h"
#include "kApiNet/Io/KTcpServer.h"
#include "kApiNet/Io/KUdpClient.h"
#include "kApiNet/Io/KWebSocket.h"
#include "kApiNet/Threads/KAtomic.h"
#include "kApiNet/Threads/KLock.h"
#include "kApiNet/Threads/KMsgQueue.h"
#include "kApiNet/Threads/KParallel.h"
#include "kApiNet/Threads/KPeriodic.h"
#include "kApiNet/Threads/KSemaphore.h"
#include "kApiNet/Threads/KThread.h"
#include "kApiNet/Threads/KThreadPool.h"
#include "kApiNet/Threads/KTimer.h"
#include "kApiNet/Utils/KBackTrace.h"
#include "kApiNet/Utils/KDebugAlloc.h"
#include "kApiNet/Utils/KDynamicLib.h"
#include "kApiNet/Utils/KEvent.h"
#include "kApiNet/Utils/KObjectPool.h"
#include "kApiNet/Utils/KPlugin.h"
#include "kApiNet/Utils/KPoolAlloc.h"
#include "kApiNet/Utils/KProcess.h"
#include "kApiNet/Utils/KRoot.h"
#include "kApiNet/Utils/KSymbolInfo.h"
#include "kApiNet/Utils/KUserAlloc.h"
#include "kApiNet/Utils/KUtils.h"

// kApiNet assembly properties
[assembly:AssemblyTitleAttribute(L"kApiNet")];
[assembly:AssemblyDescriptionAttribute(L"")];
[assembly:AssemblyConfigurationAttribute(L"")];
[assembly:AssemblyCompanyAttribute(L"")];
[assembly:AssemblyProductAttribute(L"kApiNet")];
[assembly:AssemblyCopyrightAttribute(L"Copyright (C) 2014 by LMI Technologies Inc.")];
[assembly:AssemblyTrademarkAttribute(L"")];
[assembly:AssemblyCultureAttribute(L"")];
[assembly:AssemblyVersionAttribute("1.0.*")];
[assembly:ComVisible(false)];
[assembly:CLSCompliantAttribute(true)];

//FSS-817
//
//This library uses the 'debuggable' attribute to prevent serious bugs that 
//occur when, due to aggressive JIT optimization, an object is finalized 
//before the methods that use it have finished executing. This optimization
//is safe in pure .NET code, but it's disastrous for C++/CLI classes that 
//handle unmanaged resources. Though there are other ways to solve this 
//problem, the use of 'debuggable' is (by far) the most reliable. Given that 
//this library implements lightweight wrappers rather than computationally 
//intensive logic, the loss of performance caused by this attribute is 
//considered acceptable. This attribute should be applied to all assemblies
//that implement kApiNet-based wrappers using C++/CLI.
//
[assembly:System::Diagnostics::DebuggableAttribute(true, true)];

// Friend assemblies - required for C++/CLI libraries which need to convert between .NET and native structures
[assembly:System::Runtime::CompilerServices::InternalsVisibleTo("GoSdkNet,PublicKey="
    "0024000004800000940000000602000000240000525341310004000001000100e536cdaee4362c"
    "403b4e0ad9f6bef85821a7d72af0cab512ca9eb2f88555d59149b734fa5cf3babd17d8c549784f"
    "953d1d8f88dcb2a12cc651e53aacf915fb0fd44a340f3cdf5a3c368816df44a89febb82a98dc91"
    "0f3e4ccbb732971f4a801bf8409efbac859ec5d24912756a90060553ec72f8bfe3d1c8de8278b3"
    "818690cf")];
[assembly:System::Runtime::CompilerServices::InternalsVisibleTo("kFireSyncNet,PublicKey="
    "002400000480000094000000060200000024000052534131000400000100010049e3493a2a4e89"
    "45d70fcf79b4ac530e8cb1ba6cb5d6c423749e7155b2412fb7354e31167279a257adb5d629f83f"
    "4beb0e9b9a72d2ab65a081edfd78eef78410d0e15127bed320ea419ccb1aebc9d0cb88630759b6"
    "71ceb237bdab4f1943a8f057e17f3c3af34ea64925928d7e8322c4ad2efa4689df74a47c7530ce"
    "780594c2")];
[assembly:System::Runtime::CompilerServices::InternalsVisibleTo("kDeviceNet,PublicKey="
    "00240000048000009400000006020000002400005253413100040000010001004930d6f70f0873"
    "da733e126595ec0c3cfc452b5301ee1c159fa7a941086fffc614f699fd2b90c42300852d416a1b"
    "1e330ffeeaca3eadd6bf1be84a30235e8eee74eb6411aad0b0b2b02b26f0aab5fca617d247c831"
    "7d459f29e4a42d68c700ac3a71c5f1ce119ca6f7c786c710c1a4ebcb381ec4e20f45794b753bc3"
    "100b89d9")];
[assembly:System::Runtime::CompilerServices::InternalsVisibleTo("kVisionNet,PublicKey="
    "00240000048000009400000006020000002400005253413100040000010001002160bde81e509e"
    "7d0db7e6568cbf0cde6d3855e540a3281c0298f91491459c4bbc8e3932b423d5da55ca35388c10"
    "c443ef2f1de1560c134a7e239c7adb203a33e99420ccbfb5817c231aa87c2c6cd36da209edc194"
    "715f286be7cddb62dfc838d005abefd6b510d019d4b4241d752178cf0e8fd3f653d5183bbd6bf3"
    "2a6edcea")];

// Definitions for types that do not have dedicated source files
KDefineClass(KAlloc, kAlloc)
KDefineClass(KArray1, kArray1)
KDefineClass(KArray2, kArray2)
KDefineClass(KArray3, kArray3)
KDefineClass(KArrayList, kArrayList)
KDefineClass(KAssembly, kAssembly)
KDefineClass(KBackTrace, kBackTrace)
KDefineClass(KBox, kBox)
KDefineClass(KBlowfishCipher, kBlowfishCipher)
KDefineClass(KCipher, kCipher)
KDefineClass(KCipherStream, kCipherStream)
KDefineClass(KHash, kHash)
KDefineClass(KSha1Hash, kSha1Hash)
KDefineClass(KDat5Serializer, kDat5Serializer)
KDefineClass(KDat6Serializer, kDat6Serializer)
KDefineClass(KDebugAlloc, kDebugAlloc)
KDefineClass(KDirectory, kDirectory)
KDefineClass(KDynamicLib, kDynamicLib)
KDefineClass(KEvent, kEvent)
KDefineClass(KFile, kFile)
KDefineClass(KHttpServer, kHttpServer)
KDefineClass(KHttpServerChannel, kHttpServerChannel)
KDefineClass(KHttpServerRequest, kHttpServerRequest)
KDefineClass(KHttpServerResponse, kHttpServerResponse)
KDefineClass(KImage, kImage)
KDefineClass(KList, kList)
KDefineClass(KLock, kLock)
KDefineClass(KMap, kMap)
KDefineClass(KMath, kMath)
KDefineClass(KMemory, kMemory)
KDefineClass(KMsgQueue, kMsgQueue)
KDefineClass(KNetwork, kNetwork)
KDefineClass(KPath, kPath)
KDefineClass(KParallel, kParallel)
KDefineClass(KPeriodic, kPeriodic)
KDefineClass(KPoolAlloc, kPoolAlloc)
KDefineClass(KProcess, kProcess)
KDefineClass(KPipeStream, kPipeStream)
KDefineClass(KPlugin, kPlugin)
KDefineClass(KQueue, kQueue)
KDefineClass(KSemaphore, kSemaphore)
KDefineClass(KSerializer, kSerializer)
KDefineClass(KSocket, kSocket)
KDefineClass(KStream, kStream)
KDefineClass(KString, kString)
KDefineClass(KSymbolInfo, kSymbolInfo)
KDefineClass(KTcpClient, kTcpClient)
KDefineClass(KTcpServer, kTcpServer)
KDefineClass(KThread, kThread)
KDefineClass(KThreadPool, kThreadPool)
KDefineClass(KTimer, kTimer)
KDefineClass(KUdpClient, kUdpClient)
KDefineClass(KWebSocket, kWebSocket)
KDefineClass(KUserAlloc, kUserAlloc)
KDefineClass(KUtils, kUtils)
KDefineClass(KXml, kXml)

KDefineInterface(KCollection, kCollection)
KDefineInterface(KObjectPool, kObjectPool)

KDefinePrimative(K8u, k8u)
KDefinePrimative(K8s, k8s)
KDefinePrimative(K16u, k16u)
KDefinePrimative(K16s, k16s)
KDefinePrimative(K32u, k32u)
KDefinePrimative(K32s, k32s)
KDefinePrimative(K64u, k64u)
KDefinePrimative(K64s, k64s)
KDefinePrimative(K32f, k32f)
KDefinePrimative(K64f, k64f)
KDefinePrimative(KSize, kSize)
KDefinePrimative(KSSize, kSSize)
KDefinePrimative(KChar, kChar)
KDefinePrimative(KByte, kByte)
KDefinePrimative(KBool, kBool)
KDefinePrimative(KVersion, kVersion)

KDefineStruct(KArgb, kArgb)
KDefineStruct(KDebugAllocation, kDebugAllocation)
KDefineStruct(KIpAddress, kIpAddress)
KDefineStruct(KIpEndPoint, kIpEndPoint)
KDefineStruct(KIpEntry, kIpEntry)
KDefineStruct(KPoint16s, kPoint16s)
KDefineStruct(KPoint32s, kPoint32s)
KDefineStruct(KPoint32f, kPoint32f)
KDefineStruct(KPoint64f, kPoint64f)
KDefineStruct(KPoint3d16s, kPoint3d16s)
KDefineStruct(KPoint3d32s, kPoint3d32s)
KDefineStruct(KPoint3d32f, kPoint3d32f)
KDefineStruct(KPoint3d64f, kPoint3d64f)
KDefineStruct(KPointer, kPointer)
KDefineStruct(KRect16s, kRect16s)
KDefineStruct(KRect32s, kRect32s)
KDefineStruct(KRect32f, kRect32f)
KDefineStruct(KRect64f, kRect64f)
KDefineStruct(KRect3d64f, kRect3d64f)
KDefineStruct(KRotatedRect32s, kRotatedRect32s)
KDefineStruct(KRotatedRect32f, kRotatedRect32f)
KDefineStruct(KRgb, kRgb)

KDefineEnum(KCfa, kCfa)
KDefineEnum(KCipherMode, kCipherMode)
KDefineEnum(KCipherPadding, kCipherPadding)
KDefineEnum(KComparison, kComparison)
KDefineEnum(KEndianness, kEndianness)
KDefineEnum(KFileMode, kFileMode)
KDefineEnum(KHttpStatus, kHttpStatus)
KDefineEnum(KIpVersion, kIpVersion)
KDefineEnum(KPixelFormat, kPixelFormat)
KDefineEnum(KSeekOrigin, kSeekOrigin)
KDefineEnum(KSocketEvent, kSocketEvent)
KDefineEnum(KSocketType, kSocketType)
KDefineEnum(KStatus, kStatus)
KDefineEnum(KWebSocketDataType, kWebSocketDataType)
KDefineEnum(KCompressionType, kCompressionType)
KDefineEnum(KCompressionPreset, kCompressionPreset)
KDefineEnum(KMemoryAlignment, kMemoryAlignment)

KDefineText(KText16, kText16)
KDefineText(KText32, kText32)
KDefineText(KText64, kText64)
KDefineText(KText128, kText128)
KDefineText(KText256, kText256)

KDefineVoid(KVoid, kVoid)


kStatus kCall Lmi3d::Zen::KApiLib_OnLog(const kChar* format, va_list args)
{
    kText256 buffer;

    kStrPrintvf(buffer, kCountOf(buffer), format, args);

    try
    {
        KApiLib::OnMessageLogged(KToString(buffer));

        return kOK;
    }
    catch (...)
    {
        return kERROR; 
    }
}

kStatus kCall Lmi3d::Zen::KApiLib_OnAssert(const kChar* file, k32u line)
{
    try
    {
        String^ assert = gcnew String(file);
        KApiLib::OnAssertHandler(assert + " : " + line);

        return kOK;
    }
    catch (...)
    {
        return kERROR; 
    }
}

kStatus kCall Lmi3d::Zen::KApiLib_OnAlloc(kPointer provider, kSize size, void* mem, kMemoryAlignment alignment)
{
    kStatus status = kOK;
    
    try
    {
        kSize headerSize = KApiLib::MemHeaderSize;

        if (alignment > kALIGN_ANY)
        {
            headerSize = (kSize)1 << alignment;
        }

        kSize totalAllocationSize = headerSize + size;

        *(void**)mem = kNULL;

        if (size > 0)
        {
            void* allocatedMem = kNULL;

            KCheck(xkDefaultMemAlloc(provider, totalAllocationSize, &allocatedMem, alignment));

            GC::AddMemoryPressure(totalAllocationSize);

            //write the size of the allocated memory into a header area, just before the 
            //location that we'll return to the user; this allows us to retrieve the size 
            //of the allocation in KApiLib_OnFree
            //additionally start of the allocation is written before size. This is because of memory alignment.
            const auto* userAllocation = kPointer_ByteOffset(allocatedMem, headerSize);

            ((kSize*)userAllocation)[-1] = size;
            ((kPointer*)userAllocation)[-2] = allocatedMem;
            *(void**)mem = (kByte*)userAllocation;
        }

        return kOK;     
    }
    catch (KException^ e)
    {
        status = e->Status;
    }
    catch (...)
    {
        status = kERROR;
    }
    
    return status;
}

kStatus kCall Lmi3d::Zen::KApiLib_OnFree(kPointer provider, void* mem)
{
    kStatus status = kOK;

    try
    {
        if (!kIsNull(mem))
        {
            const auto size = ((kSize*)mem)[-1];
            auto* const allocatedMem = ((kPointer*)mem)[-2];
            kSize totalAllocationSize = size + kPointer_Diff(mem, allocatedMem);

            KCheck(xkDefaultMemFree(provider, allocatedMem));

            //inform the garbage collector about native memory that we have deallocated
            GC::RemoveMemoryPressure(totalAllocationSize);
        }

        return kOK;
    }
    catch (KException^ e)
    {
        status = e->Status;
    }
    catch (...)
    {
        status = kERROR;
    }
    
    return status;
}
