#include "pch.h"
#include "CLoadingToWork.h"
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

CLoadingToWork* gcLoadingToWork;
CLoadingToWork::CLoadingToWork()
{
}

CLoadingToWork::~CLoadingToWork()
{
}

UINT CLoadingToWork::StartLoadingToWork(LPVOID wParam)
{
    bool bLoop = true, bWorkSet = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    CLoadingToWork* pThis = reinterpret_cast<CLoadingToWork*>(wParam);
    ULONGLONG GetTime = 0, Elapsed = 0;
    CString strPusherZ = GetPusherZName(FRONT_CONV);
    double Position = ReadPosition(strPusherZ), PCBThickness = 1.0, DownOffset = 10.0, Ratio = 0.5, Offset = 0.0;
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
        if (pThis->IsTerminated(THREAD_LOADINGTOWORK_READTIME) == true)
        {
            TRACE(_T("[PWR] StartLoadingToWork(0x%X) Terminated\n"), pThis->GetThreadID());
            break;
        }
        bWorkSet = false;
        if (pThis->IsExistEnt(ENTRY_CONV) == true || pThis->IsExistSet(ENTRY_CONV) == true)
        {
            gcEntryConveyor->SetStep(ConveyorStep::STOP);
            gcWorkConveyor->SetStep(ConveyorStep::STOP);
            gcExitConveyor->SetStep(ConveyorStep::STOP);
            gcEntryConveyor->BeltOff(BELT_SPEED_MID);
            gcWorkConveyor->BeltOff(BELT_SPEED_MID);
            gcExitConveyor->BeltOff(BELT_SPEED_MID);
            SetWorkPcbReady(FRONT_CONV, false);
            GetTime = _time_get();
            if (abs(Offset) > 0.001 && GetOnlyConveyorMode() == false)
            {
                Ratio = gcWorkConveyor->GetPusherDownRatio();
                Err = StartPosWaitMotion(strPusherZ, Ratio, TimeOut, Position + Offset, true);
                if (Err == NO_ERR)
                {
                    WritePusherZ(FRONT_CONV, WORK1_CONV, Position + Offset);
                }                
                else
                {
                    TRACE(_T("[PWR] StartLoadingToWork(0x%X) Pusher Down Error:%d Quit\n"), pThis->GetThreadID(), Err);
                    break;
                }
            }
            TRACE(_T("[PWR] PusherZ Down Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
            GetTime = _time_get();
            while (1)
            {
                gcWorkConveyor->UpStopper();
                if (gcWorkConveyor->IsStopperUp(WORK1_CONV) == true)
                {
                    Err = 0;
                    ThreadSleep(TIME50MS);
                    break;
                }
                if (_time_elapsed(GetTime) > TIME10000MS)
                {
                    Err = WORK1_STOPPER_UP_TIMEOUT;
                    break;
                }
                ThreadSleep(THREAD_LOADINGTOWORK_READTIME);
            };
            TRACE(_T("[PWR] Stopper Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
            if (Err > 0)
            {
                TRACE(_T("[PWR] StartLoadingToWork(0x%X) Error1 Quit\n"), pThis->GetThreadID());
                break;
            }
            gcEntryConveyor->SetReverse(false);
            gcWorkConveyor->SetReverse(false);
            gcExitConveyor->SetReverse(false);

            gcEntryConveyor->BeltOn(EntryNextOutBeltSpd);
            gcWorkConveyor->BeltOn(WorkPrevInBeltSpd);
            GetTime = _time_get();
            while (1)
            {
                if (pThis->IsExistLow(WORK1_CONV) == true || pThis->IsExistSet(WORK1_CONV) == true)
                {
                    SpeedTime = gcWorkConveyor->GetMiddleTime();
                    ThreadSleep(SpeedTime);
                    gcWorkConveyor->BeltOn(BELT_SPEED_LOW);
                    break;
                }
                if (_time_elapsed(GetTime) > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_LOADINGTOWORK_READTIME);
            }
            TRACE(_T("[PWR] IsExistLow Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
            if (Err > 0)
            {
                gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
                gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
                TRACE(_T("[PWR] StartLoadingToWork(0x%X) Error2 Quit\n"), pThis->GetThreadID());
                break;
            }
            GetTime = _time_get();
            while (1)
            {
                if (pThis->IsExistSet(WORK1_CONV) == true)
                {
                    SpeedTime = gcWorkConveyor->GetLowTime();
                    ThreadSleep(SpeedTime);
                    gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
                    gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
                    bWorkSet = true;
                    break;
                }
                if (_time_elapsed(GetTime) > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_LOADINGTOWORK_READTIME);
            };
            TRACE(_T("[PWR] IsExistSet Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
            if (Err > 0)
            {
                gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
                gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
                TRACE(_T("[PWR] StartLoadingToWork(0x%X) Error3 Quit\n"), pThis->GetThreadID());
                break;
            }
            GetTime = _time_get();
			if (GetOnlyConveyorMode() == true)
			{
				SetWorkPcbReady(FRONT_CONV, true);
			}
			else
			{
                Ratio = gcWorkConveyor->GetPusherUpRatio();
				Err = StartPosWaitMotion(strPusherZ, Ratio, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
				if (Err == NO_ERR)
				{
					WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
					TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
					SetWorkPcbReady(FRONT_CONV, true);
				}
			}

        }
        if (Err > 0)
        {
            gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
            gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
            TRACE(_T("[PWR] StartLoadingToWork(0x%X) Error4 Quit\n"), pThis->GetThreadID());
            break;
        }
        if (bWorkSet == true)
        {
            break;
        }
    };
    pThis->SetEnd(true);
    gcEntryConveyor->BeltOff(BELT_SPEED_MID);
    gcWorkConveyor->BeltOff(BELT_SPEED_MID);
    gcExitConveyor->BeltOff(BELT_SPEED_MID);
    TRACE(_T("[PWR] StartLoadingToWork(0x%x) Quit Elapsed:%d[ms]\n"), pThis->GetThreadID(), _time_elapsed(GetTime));
    return 0;
}

void CLoadingToWork::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thLoadingToWork"));    
    lpStartAddress = (_beginthreadex_proc_type)StartLoadingToWork;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    TRACE(_T("[PWR] CLoadingToWork Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] CLoadingToWork Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}