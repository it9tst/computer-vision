///@cond private

/** 
 * @file    GoExtParams.h
 * @brief   Declares the GoExtParams class.
 *
 * @internal
 * Copyright (C) 2017-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef GO_EXT_PARAMS_H
#define GO_EXT_PARAMS_H

#include <GoSdk/GoSdkDef.h>
#include <GoSdk/Tools/GoExtParam.h>
#include <kApi/Data/kXml.h>

typedef kObject GoExtParams;

GoFx(kSize) GoExtParams_ParameterCount(GoExtParams params);
GoFx(GoExtParam) GoExtParams_ParameterAt(GoExtParams params, kSize index);
GoFx(GoExtParam) GoExtParams_FindParameterById(GoExtParams params, const kChar* id);

#include <GoSdk/Tools/GoExtParams.x.h>

#endif
///@endcond
