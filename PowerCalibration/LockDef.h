#pragma once
/*
//	LockDef.h
//
//	(C) Power. 2020 ~
*/

#define semTake(x, t)					WaitForSingleObject(x, t);
#define semGive(x)						ReleaseMutex(x);
#define semFlush(x)						WaitForSingleObject(x, NO_WAIT);

#define SEM_LOCK(x, t)					semTake(x, t)
#define SEM_UNLOCK(x)					semGive(x)
#define SEM_FLUSH(x)					semFlush(x)

#define LISTCTRL_LOCK(x, t)				semTake(x, t)
#define LISTCTRL_UNLOCK(x)				semGive(x)
#define LISTCTRL_FLUSH(x)				semFlush(x)

#define MSGQ_LOCK(x, t)					semTake(x, t)
#define MSGQ_UNLOCK(x)					semGive(x)
#define MSGQ_FLUSH(x)					semFlush(x)

#define SENDCLIENT_LOCK(x, t)			semTake(x, t)
#define SENDCLIENT_UNLOCK(x)			semGive(x)
#define SENDCLIENT_FLUSH(x)				semFlush(x)

#define ADDSOCKET_LOCK(x, t)			semTake(x, t)
#define ADDSOCKET_UNLOCK(x)				semGive(x)
#define ADDSOCKET_FLUSH(x)				semFlush(x)

#define READINPUT_LOCK(x, t)			semTake(x, t)
#define READINPUT_UNLOCK(x)				semGive(x)
#define READINPUT_FLUSH(x)				semFlush(x)

#define HOMESTATUS_LOCK(x, t)			semTake(x, t)
#define HOMESTATUS_UNLOCK(x)			semGive(x)
#define HOMESTATUS_FLUSH(x)				semFlush(x)

#define MASTERSTATUS_LOCK(x, t)			semTake(x, t)
#define MASTERSTATUS_UNLOCK(x)			semGive(x)
#define MASTERSTATUS_FLUSH(x)			semFlush(x)

#define CALIBRATION_LOCK(x, t)			semTake(x, t)
#define CALIBRATION_UNLOCK(x)			semGive(x)
#define CALIBRATION_FLUSH(x)			semFlush(x)

#define HOSTMSG_LOCK(x, t)				semTake(x, t)
#define HOSTMSG_UNLOCK(x)				semGive(x)
#define HOSTMSG_FLUSH(x)				semFlush(x)

#define AXIS_LOCK(x, t)					semTake(x, t)
#define AXIS_UNLOCK(x)					semGive(x)
#define AXIS_FLUSH(x)					semFlush(x)

#define SLAVESTATUS_LOCK(x, t)			semTake(x, t)
#define SLAVESTATUS_UNLOCK(x)			semGive(x)
#define SLAVESTATUS_FLUSH(x)			semFlush(x)

#define PCVISLOCK(x,t)					semTake(x,t)
#define PCVISUNLOCK(t) 					semGive(x)

#define MOTION_LOCK(x, t)				semTake(x, t)
#define MOTION_UNLOCK(x)				semGive(x)
#define MOTION_FLUSH(x)					semFlush(x)
