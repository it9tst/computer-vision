// 
// KEvent.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_EVENT_H
#define K_API_NET_EVENT_H

#include <kApi/Utils/kEvent.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {
        namespace Utils
        {           
            /// <summary>Delegate for a KEvent event handler.</summary>
            public delegate void KEventFx(IntPtr sender, IntPtr args);

            /// <summary>Represents a list of native callbacks. <para/> Requires manual disposal.</summary>
            /// <remarks> 
            /// This type is not likely to be useful in the context of managed applications, but an implementation is provided
            /// here for completeness. 
            /// </remarks>
            public ref class KEvent : public KObject
            {
                KDeclareClass(KEvent, kEvent)

            public:
                /// <summary>Initializes a new instance of the KEvent class with the specified Zen object handle.</summary>           
                /// <param name="handle">Zen object handle.</param>
                KEvent(IntPtr handle)
                    : KObject(handle, DefaultRefStyle)
                {}

                /// <summary>Initializes a new instance of the KEvent class.</summary>           
                KEvent()
                    : KObject(DefaultRefStyle)
                {
                    kEvent handle = kNULL;

                    KCheck(kEvent_Construct(&handle, kNULL));

                    Handle = handle;
                }

                /// <inheritdoc cref="KEvent()" />
                /// <param name="allocator">Memory allocator.</param>
                KEvent(KAlloc^ allocator)
                    : KObject(DefaultRefStyle)
                {
                    kEvent handle = kNULL;

                    KCheck(kEvent_Construct(&handle, KToHandle(allocator)));

                    Handle = handle;
                }

                /// <summary>Adds a listener to the event.</summary>
                /// 
                /// <param name="handler">Handler.</param>
                void Add(KEventFx^ handler)
                {
                    KCallbackFx^ thunk = gcnew KCallbackFx(this, &KEvent::OnEvent);
                    KCallbackState^ session = gcnew KCallbackState(thunk, handler);
                    kStatus status; 

                    if (!kSuccess(status = kEvent_Add(Handle, (kCallbackFx)session->NativeFunction, session->NativeContext)))
                    {
                        delete session;
                        throw gcnew KException(status);
                    }
                }

                /// <summary>Removes a listener from the event.</summary>
                /// 
                /// <param name="handler">Handler.</param>
                void Remove(KEventFx^ handler)
                {
                    kList list = kEvent_Listeners(Handle); 
                    kListItem it = kList_First(list);

                    while (!kIsNull(it))
                    {
                        kCallback callback = kList_AsT(list, it, kCallback); 

                        //this might be dangerous, if the event has multiple listeners and some are not managed listeners
                        KCallbackState^ session = KCallbackState::FromNativeContext(callback.receiver); 

                        if (session->Handler == handler)
                        {
                            KCheck(kEvent_Remove(Handle, callback.function, callback.receiver)); 

                            delete session; 

                            return; 
                        }

                        it = kList_Next(list, it); 
                    }

                    throw gcnew KException(kERROR_NOT_FOUND); 
                }
 
                /// <summary>Removes all listeners from the event.</summary>
                void Clear()
                {
                    while (kEvent_Count(Handle) > 0)
                    {
                        kList list = kEvent_Listeners(Handle);
                        kListItem it = kList_First(list);

                        kCallback callback = kList_AsT(list, it, kCallback);

                        //this might be dangerous, if the event has multiple listeners and some are not managed listeners
                        KCallbackState^ session = KCallbackState::FromNativeContext(callback.receiver);

                        KCheck(kEvent_Remove(Handle, callback.function, callback.receiver));

                        delete session;
                    }                   
                }

                /// <summary>Notifies all event listeners.</summary>
                /// 
                /// <param name="sender">Sender of event notification.</param>
                /// <param name="args">Arguments for event notification.</param>
                /// <exception cref="KException">Thrown if one or more event handlers generates an exception.</exception>
                void Notify(IntPtr sender, IntPtr args)
                {
                    KCheck(kEvent_Notify(Handle, sender.ToPointer(), args.ToPointer())); 
                }

                /// <summary>Gets count of event listeners.</summary>
                property k64s Count
                {
                    k64s get() { return kEvent_Count(Handle); }
                }

           protected:

               //ensures that callbacks are unhooked
               virtual void OnDisposing() override
               {
                   Clear();
               }

            private:

                kStatus OnEvent(kPointer receiver, kPointer sender, kPointer args)
                {
                    kStatus status = kOK;

                    try
                    {
                        KCallbackState^ context = KCallbackState::FromNativeContext(receiver);
                        KEventFx^ handler = (KEventFx^)context->Handler;
                        
                        handler(IntPtr(sender), IntPtr(args));
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
            };
        }
    }
}

#endif
