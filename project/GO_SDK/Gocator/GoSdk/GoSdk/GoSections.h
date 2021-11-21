/**
 * @file    GoSections.h
 * @brief   Declares the GoSections class. 
 *
 * @internal
 * Copyright (C) 2016-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_SECTIONS_H
#define GO_SECTIONS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/GoSection.h>

/**
 * @class   GoSections
 * @extends kObject
 * @note    Supported with G2, G3
 * @ingroup GoSdk-Surface
 * @brief   Represents the collection of sections and limits for defining them.
 */
typedef kObject GoSections; 

/** 
 * Gets the X axis minimum for section definition.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              The minimum X value.
 */
GoFx(k64f) GoSections_XLimitMin(GoSections sections);

/** 
 * Gets the X axis maximum for section definition.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              The maximum X value.
 */
GoFx(k64f) GoSections_XLimitMax(GoSections sections);

/** 
 * Gets the Y axis minimum for section definition.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              The minimum Y value.
 */
GoFx(k64f) GoSections_YLimitMin(GoSections sections);

/** 
 * Gets the Y axis maximum for section definition.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              The maximum Y value.
 */
GoFx(k64f) GoSections_YLimitMax(GoSections sections);

/** 
 * Returns the number of added sections.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              The section count.
 */
GoFx(kSize) GoSections_SectionCount(GoSections sections);

/** 
 * Retrieves the section at the specified index.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @param   index       The index of the section to retrieve.
 * @return              A section.
 * @see                 GoSections_SectionCount
 */
GoFx(GoSection) GoSections_SectionAt(GoSections sections, kSize index);

/** 
 * Adds a specified section to the configuration.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @param   section     The GoSection reference to add.
 * @return              Operation status.
 */
GoFx(kStatus) GoSections_AddSection(GoSections sections, GoSection* section);

/** 
 * Removes the section at the specified index.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @param   index       The index of the section to remove.
 * @return              Operation status.
 */
GoFx(kStatus) GoSections_RemoveSection(GoSections sections, kSize index);

/** 
 * Clears the section list.
 *
 * @public              @memberof GoSection
 * @note                Supported with G2, G3
 * @version             Introduced in firmware 4.4.4.14
 * @param   sections    GoSections object.
 * @return              Operation status.
 */
GoFx(kStatus) GoSections_Clear(GoSections sections);

#include <GoSdk/GoSections.x.h>

#endif
