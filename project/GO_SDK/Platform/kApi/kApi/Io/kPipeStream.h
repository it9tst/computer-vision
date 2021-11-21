/** 
 * @file    kPipeStream.h
 * @brief   Declares the kPipeStream class. 
 *
 * @internal
 * Copyright (C) 2018-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */

#ifndef K_API_PIPE_STREAM_H
#define K_API_PIPE_STREAM_H

#include <kApi/kApiDef.h>

/**
 * @class   kPipeStream
 * @extends kStream
 * @ingroup kApi-Utils
 * @brief   Represents standard streams as kStream object.
  * 
 */
//typedef kObject kPipeStream;        --forward-declared in kApiDef.x.h

/** 
 * Return stdin stream as kStream.
 *
 * @public                  @memberof kPipeStream
 * @return                  kStream object. 
 */
kFx(kStream) kStdIn();

/** 
 * Return stdout stream as kStream.
 *
 * @public                  @memberof kPipeStream
 * @return                  kStream object. 
 */
kFx(kStream) kStdOut();

/** 
 * Return stderr stream as kStream.
 *
 * @public                  @memberof kPipeStream
 * @return                  kStream object. 
 */
kFx(kStream) kStdErr();

#include <kApi/Io/kPipeStream.x.h>

#endif
