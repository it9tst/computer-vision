// 
// GoSections.h
// 
// Copyright (C) 2016-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef GO_SDK_NET_SECTIONS_H
#define GO_SDK_NET_SECTIONS_H

#include <GoSdk/GoSections.h>
#include <GoSdkNet/GoSection.h>
#include <GoSdkNet/GoSdkDef.h>

namespace Lmi3d
{
    namespace GoSdk
    {
        /// <summary>Represents the collection of sections and limits for defining them.</summary>
        public ref class GoSections : public KObject
        {
            KDeclareClass(GoSections, GoSections)

            /// <summary>Initializes a new instance of the GoSections class with the specified Zen object handle.</summary>
            /// <param name="handle">Zen object handle.</param>
            GoSections(IntPtr handle)
                : KObject(handle)
            {}

            /// <summary>Initializes a new instance of the GoSections class.</summary>
            /// <param name="sensor">A GoSensor instance.</param>
            GoSections(GoSensor^ sensor)
            {
                ::GoSections handle = kNULL;

                KCheck(::GoSections_Construct(&handle, KToHandle(sensor), kNULL));

                Handle = handle;
            }

            /// <inheritdoc cref="GoSections(GoSensor^ sensor)" />
            /// <param name="allocator">Memory allocator</param>
            GoSections(GoSensor^ sensor, KAlloc^ allocator)
            {
                ::GoSections handle = kNULL;

                KCheck(::GoSections_Construct(&handle, KToHandle(sensor), KToHandle(allocator)));

                Handle = handle;
            }

            /// <summary>The X axis minimum for section definition.</summary>
            property k64f XLimitMin
            {
                k64f get()           { return ::GoSections_XLimitMin(Handle); }
            }

            /// <summary>The X axis maximum for section definition.</summary>
            property k64f XLimitMax
            {
                k64f get()           { return ::GoSections_XLimitMax(Handle); }
            }

            /// <summary>The Y axis minimum for section definition.</summary>
            property k64f YLimitMin
            {
                k64f get()           { return ::GoSections_YLimitMin(Handle); }
            }

            /// <summary>The Y axis maximum for section definition.</summary>
            property k64f YLimitMax
            {
                k64f get()           { return ::GoSections_YLimitMax(Handle); }
            }

            /// <summary>The number of added sections.</summary>
            property k64s SectionCount
            {
                k64s get()           { return (k64s) ::GoSections_SectionCount(Handle); }
            }

            /// <summary>Gets the section at the specified index.</summary>
            /// <param name="index">The index of the section to retrieve.</param>
            /// <returns>A section.</returns>
            GoSection^ GetSection(k64s index)
            {
                return KToObject<GoSection^>(::GoSections_SectionAt(Handle, (kSize)index));
            }

            /// <summary>Adds a section to the configuration.</summary>
            /// <returns>The added section.</returns>
            GoSection^ AddSection()
            {
                ::GoSection section;
                
                KCheck(GoSections_AddSection(Handle, &section));
                
                return KToObject<GoSection^>(section);
            }

            /// <summary>Clears the section list.</summary>
            void Clear()
            {
                KCheck(::GoSections_Clear(Handle));
            }

            /// <summary>Removes the section at the specified index.</summary>
            /// <param name=index>The index of the section to remove.</param>
            void RemoveSection(k64s index)
            {
                KCheck(::GoSections_RemoveSection(Handle, (kSize)index));
            }

        };
    }
}

#endif
