/** 
 * @file    kLock.h
 * @brief   Declares the kLock class. 
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#ifndef K_API_LOCK_H
#define K_API_LOCK_H

#include <kApi/kApiDef.h>

/**
 * @class   kLockOption
 * @extends kValue
 * @ingroup kApi-Threads
 * @brief   Represents a lock construction option.
 */
typedef k32s kLockOption; 

/** @relates kLockOption @{ */
#define kLOCK_OPTION_NONE                  (0x0)       ///< No options
#define kLOCK_OPTION_TIMEOUT               (0x1)       ///< Can wait with timeout.
/** @} */

#include <kApi/Threads/kLock.x.h>

/**
 * @class   kLock
 * @extends kObject
 * @ingroup kApi-Threads
 * @brief   Represents a recursive, mutual exclusion lock.
 * 
 * The kLock class optionally supports lock timeouts. Use the kLock_ConstructEx
 * method with kLOCK_OPTION_TIMEOUT to construct a lock that can support the use of finite 
 * timeout values in the kLock_EnterEx method. 
 */

/** 
 * Constructs a lock object.
 *
 * The constructed object does not support timeouts. When used with the kLock_EnterEx method, 
 * kINFINITE is the only supported timeout value. 
 *
 * @public              @memberof kLock
 * @param   lock        Destination for the constructed object handle.
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Construct(kLock* lock, kAlloc allocator);

/** 
 * Constructs a lock object with support for behavioral options.
 *
 * If kLOCK_OPTION_TIMEOUT is specified, a finite timeout value can be used with kLock_EnterEx.
 *
 * @public              @memberof kLock
 * @param   lock        Destination for the constructed object handle.
 * @param   options     Bitset of lock options. 
 * @param   allocator   Memory allocator (or kNULL for default). 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_ConstructEx(kLock* lock, kLockOption options, kAlloc allocator);

/** 
 * Blocks until exclusive ownership of the lock is acquired.
 *
 * kLock ownership is recursive; the same thread can acquire the lock multiple 
 * times without blocking. The lock will be relinquished when the number of 
 * calls to kLock_Exit balances the number of successful calls to kLockEnter/kLockEnterEx.  
 *
 * @public              @memberof kLock
 * @param   lock        Lock object. 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Enter(kLock lock);

/** 
 * Blocks until exclusive ownership of the lock is acquired or the timeout interval has elapsed.
 *
 * kLock ownership is recursive; the same thread can acquire the lock multiple 
 * times without blocking. The lock will be relinquished when the number of 
 * calls to kLock_Exit balances the number of successful calls to kLockEnter/kLockEnterEx.  
 *
 * kERROR_TIMEOUT is returned if the timeout elapses before the lock can be acquired. 
 * 
 * A debug assertion may be raised if a finite timeout is specified and the lock does not support
 * timeouts.
 *
 * @public              @memberof kLock
 * @param   lock        Lock object. 
 * @param   timeout     Timeout in microseconds.
 * @return              Operation status. 
 */
kFx(kStatus) kLock_EnterEx(kLock lock, k64u timeout);

/** 
 * Relinquishes ownership of the lock.
 *
 * kLock ownership is recursive; the same thread can acquire the lock multiple 
 * times without blocking. The lock will be relinquished when the number of 
 * calls to kLock_Exit balances the number of successful calls to kLockEnter/kLockEnterEx.  
 *
 * @public              @memberof kLock
 * @param   lock        Lock object. 
 * @return              Operation status. 
 */
kFx(kStatus) kLock_Exit(kLock lock); 

#endif
