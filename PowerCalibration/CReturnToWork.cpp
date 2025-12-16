#include "pch.h"
#include "CReturnToWork.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "DefineThreadLoopTime.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
//#include "ErrorCode.h"

CReturnToWork* gcReturnToWork;
CReturnToWork::CReturnToWork()
{
}

CReturnToWork::~CReturnToWork()
{
}

UINT CReturnToWork::StartReturnToWork(LPVOID wParam)
{
    bool bLoop = true, bWork = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    CReturnToWork* pThis = reinterpret_cast<CReturnToWork*>(wParam);
    CApplicationTime* pTime = new CApplicationTime();
    CString strPusherZ = GetPusherZName(FRONT_CONV);
    double Position = ReadPosition(strPusherZ), PCBThickness = 1.0, DownOffset = 0.0, Offset = 10.0, Ratio = 0.5;
    long TimeOut = TIME5000MS, Err = 0, SpeedTime = 0;
    long EntryPrevInBeltSpd, EntryNextOutBeltSpd, WorkPrevInBeltSpd, WorkNextOutBeltSpd, ExitPrevInBeltSpd, ExitNextOutBeltSpd;
    EntryPrevInBeltSpd = EntryNextOutBeltSpd = WorkPrevInBeltSpd = WorkNextOutBeltSpd = ExitPrevInBeltSpd = ExitNextOutBeltSpd = BELT_SPEED_MID;
    EntryPrevInBeltSpd = GetConveyorProfileSpeedPrevIn(ENTRY_CONV);
    EntryNextOutBeltSpd = GetConveyorProfileSpeedNextOut(ENTRY_CONV);
    WorkPrevInBeltSpd = GetConveyorProfileSpeedPrevIn(WORK1_CONV);
    WorkNextOutBeltSpd = GetConveyorProfileSpeedNextOut(WORK1_CONV);
    ExitPrevInBeltSpd = GetConveyorProfileSpeedPrevIn(EXIT_CONV);
    ExitNextOutBeltSpd = GetConveyorProfileSpeedNextOut(EXIT_CONV);
    PCBThickness = gcWorkConveyor->GetPcbThickness();
    DownOffset = gcWorkConveyor->GetPcbStandByZOffset();
    if (Position < (GetPusherByZ(FRONT_GANTRY) + DownOffset))
    {
        Offset = (GetPusherByZ(FRONT_GANTRY) + DownOffset) - Position;
    }
    else
    {
        Offset = 0.0;
    }
    while (pThis->GetRunning())
    {
        bWork = false;
        if (pThis->IsExistSet(EXIT_CONV) == true)
        {
            gcEntryConveyor->SetStep(ConveyorStep::STOP);
            gcWorkConveyor->SetStep(ConveyorStep::STOP);
            gcExitConveyor->SetStep(ConveyorStep::STOP);
            gcEntryConveyor->BeltOff(BELT_SPEED_MID);
            gcWorkConveyor->BeltOff(BELT_SPEED_MID);
            gcExitConveyor->BeltOff(BELT_SPEED_MID);
            SetWorkPcbReady(FRONT_CONV, false);
            pTime->TimeGet();
            while (1)
            {
                gcWorkConveyor->DownStopper();
                if (gcWorkConveyor->IsStopperDn(WORK1_CONV) == true)
                {
                    Err = 0;
                    ThreadSleep(TIME50MS);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    Err = WORK1_STOPPER_DN_TIMEOUT;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            TRACE(_T("[PWR] Stopper Down Elapsed:%d[ms]\n"), pTime->TimeElapsed());
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error1 Quit Err:%d\n"), pThis->GetThreadID(), Err);
                break;
            }
            pTime->TimeGet();
            if (abs(Offset) > 0.001 && GetOnlyConveyorMode() == false)
            {
                Ratio = gcWorkConveyor->GetPusherDownRatio();
                Err = StartPosWaitMotion(strPusherZ, Ratio, TimeOut, Position + Offset, true);
                if (Err != NO_ERR)
                {
                    TRACE(_T("[PWR] CReturnToWork(0x%X) Error2 Quit\n"), pThis->GetThreadID());
                    break;
                }
                WritePusherZ(FRONT_CONV, WORK1_CONV, Position + Offset);
            }
            TRACE(_T("[PWR] PusherZ Down Elapsed:%d[ms]\n"), pTime->TimeElapsed());
            gcWorkConveyor->SetReverse(true);
            gcExitConveyor->SetReverse(true);

            gcWorkConveyor->BeltOn(WorkPrevInBeltSpd);
            gcExitConveyor->BeltOn(ExitPrevInBeltSpd);
            pTime->TimeGet();
            while (1)
            {
                if (gcEntryConveyor->IsExistSet(WORK1_CONV) == true)
                {
                    bWork = true;
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error3 Quit\n"), pThis->GetThreadID());
                break;
            }
            pTime->TimeGet();
            while (1)
            {
                if (gcEntryConveyor->IsExistSet(WORK1_CONV) == false)
                {
                    bWork = false;
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error4 Quit\n"), pThis->GetThreadID());
                break;
            }
            pTime->TimeGet();
            while (1)
            {
                if (gcEntryConveyor->IsExistLow(WORK1_CONV) == false)
                {
                    bWork = false;
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error5 Quit\n"), pThis->GetThreadID());
                break;
            }
            gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
            gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
            pTime->TimeGet();
            while (1)
            {
                gcWorkConveyor->UpStopper();
                if (gcWorkConveyor->IsStopperDn(WORK1_CONV) == true)
                {
                    Err = 0;
                    ThreadSleep(TIME50MS);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error5 Quit\n"), pThis->GetThreadID());
                break;
            }
            TRACE(_T("[PWR] Stopper Down Elapsed:%d[ms]\n"), pTime->TimeElapsed());
            gcWorkConveyor->SetReverse(false);
            gcExitConveyor->SetReverse(false);
            gcWorkConveyor->BeltOn(WorkPrevInBeltSpd);
            pTime->TimeGet();
            while (1)
            {
                if (gcEntryConveyor->IsExistLow(WORK1_CONV) == true || gcEntryConveyor->IsExistSet(WORK1_CONV) == true)
                {
                    SpeedTime = gcWorkConveyor->GetMiddleTime();
                    ThreadSleep(SpeedTime);
                    gcWorkConveyor->BeltOn(BELT_SPEED_LOW);
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error6 Quit\n"), pThis->GetThreadID());
                break;
            }
            pTime->TimeGet();
            while (1)
            {
                if (gcEntryConveyor->IsExistSet(WORK1_CONV) == true)
                {
                    SpeedTime = gcWorkConveyor->GetLowTime();
                    ThreadSleep(SpeedTime);
                    bWork = true;
                    gcWorkConveyor->BeltOff(BELT_SPEED_LOW);
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            if (Err > 0)
            {
                TRACE(_T("[PWR] CReturnToWork(0x%X) Error7 Quit\n"), pThis->GetThreadID());
                break;
            }
            pTime->TimeGet();
			if (GetOnlyConveyorMode() == false)
			{
                Ratio = gcWorkConveyor->GetPusherUpRatio();
				Err = StartPosWaitMotion(strPusherZ, Ratio, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
				if (Err == NO_ERR)
				{
					WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
					TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), pTime->TimeElapsed());
					SetWorkPcbReady(FRONT_CONV, true);
				}
				else
				{
					TRACE(_T("[PWR] CReturnToWork(0x%X) Error8 Quit\n"), pThis->GetThreadID());
					break;
				}
			}
			else
			{
				SetWorkPcbReady(FRONT_CONV, true);

			}
        }
        if (Err > 0)
        {
            gcEntryConveyor->BeltOff(EntryPrevInBeltSpd);
            gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
            gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
            TRACE(_T("[PWR] CReturnToWork(0x%X) Error2 Quit\n"), pThis->GetThreadID());
            break;
        }
        if (bWork == true)
        {
            break;
        }
    };
    gcEntryConveyor->BeltOff(EntryPrevInBeltSpd);
    gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
    gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
    pThis->SetEnd(true);
    TRACE(_T("[PWR] CReturnToWork(0x%x) Quit Elapsed:%d[ms]\n"), pThis->GetThreadID(), pTime->TimeElapsed());
    delete pTime;
    return 0;
}

void CReturnToWork::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thReturnToEnt"));
    lpStartAddress = (_beginthreadex_proc_type)StartReturnToWork;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    TRACE(_T("[PWR] CReturnToWork Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] CReturnToWork Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}