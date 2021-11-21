/**
 * Copyright (C) 2008-2021 by LMI Technologies Inc.
 */
#ifndef GENERIC_BLOB_H
#define GENERIC_BLOB_H

#include <kApi/kApiDef.h>
#include <vector>
#include <array>

 // Include for deserialization
#if !defined(GDK_HELPER_LIBRARY_EMIT) && !defined(GDKAPP)
#include <GoSdk/GoSdk.h>
#endif

// Include for type ID
#if !defined(GDK_HELPER_LIBRARY_EMIT) && !defined(GDKAPP)
#define GDK_DATA_TYPE_GENERIC_BASE                  (0x80000000)    ///< Generic data base
#else
#include <Gdk/GdkDef.h>
#endif

const k32u GENERIC_BLOB_DATA_TYPE = GDK_DATA_TYPE_GENERIC_BASE + 0x00009001;

/**
* @class   BlobProperty
* @brief   Represents output for segmented parts and blobs.
*/

class kExportDx(BlobProperty)
{
private:
    k64f _area;
    k64f _aspect;
    k64f _centerHeight;
    k64f _angle;
    kPoint32f _center; // Center of minAreaRect
    kPoint32f _centerHoriz; // Center of up-right bounding box
    kPoint32f _centerContourPts; // Average point of contour points
    k64f _width;
    k64f _length;
    kSize _contourPtCount;
    std::array<kPoint32f, 4> _minAreaRectCorners;
    kRect32s _boundingBox;

    std::vector<kPoint64f> *_pContourPoints;

public:
    BlobProperty();
    ~BlobProperty();

    // Copy
    BlobProperty(const BlobProperty& other);
    BlobProperty& operator=(const BlobProperty& other);

    // Move
    BlobProperty(BlobProperty&& other) noexcept;
    BlobProperty& operator=(BlobProperty&& other);

    // Getter and Setter
    k64f GetArea() const;
    void SetArea(k64f area);

    k64f GetAspect() const;
    void SetAspect(k64f aspect);

    k64f GetCenterHeight() const;
    void SetCenterHeight(k64f height);

    k64f GetAngle() const;
    void SetAngle(k64f angle);

    const kPoint32f& GetCenter() const;
    void SetCenter(const kPoint32f& pt);

    const kPoint32f& GetCenterHoriz() const;
    void SetCenterHoriz(const kPoint32f& pt);

    const kPoint32f& GetCenterContourPts() const;
    void SetCenterContourPts(const kPoint32f& pt);

    k64f GetWidth()const;
    void SetWidth(k64f width);

    k64f GetLength() const;
    void SetLength(k64f length);

    kSize GetContourPtCount() const;
    void SetContourPtCount(kSize count);

    const std::array<kPoint32f, 4>& GetMinAreaRectCorners() const;
    void SetMinAreaRectCorners(const std::array<kPoint32f, 4>& minAreaRectCorners);

    const kRect32s& GetBoundingBox() const;
    void SetBoundingBox(const kRect32s& rect);

    const std::vector<kPoint64f>& GetContourPoints() const;
    void SetContourPoints(const std::vector<kPoint64f>& contourPoints);

};


kExportDx(kStatus) SerializeBlobs(const std::vector<BlobProperty>& blobs, kArray1* outputBuffer);
kExportDx(kStatus) DeserializeBlobs(std::vector<BlobProperty>& blobs, void* inputBuffer, kSize length, kAlloc alloc);


// For SDK usage
#if !defined(GDK_HELPER_LIBRARY_EMIT) && !defined(GDKAPP)
kBool   GoGenericMsgTypeValid(GoGenericMsg genMsg);
kStatus DeserializeBlobs(GoGenericMsg genMsg, std::vector<BlobProperty>& blobs);
#endif

#endif
