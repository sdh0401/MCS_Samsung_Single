#include "pch.h"
#include "COutToExit.h"
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

COutToExit* gcOutToExit;
COutToExit::COutToExit()
{
}

COutToExit::~COutToExit()
{
}

UINT COutToExit::StartOutToExit(LPVOID wParam)
{
    bool bLoop = true, bExitSet = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    COutToExit* pThis = reinterpret_cast<COutToExit*>(wParam);
    //CApplicationTime* pTime = new CApplicationTime();
    ULONGLONG GetTime = 0, Elapsed = 0;
    ULONGLONG TotalGetTime = 0, TotalElapsed = 0;
    CString strPusherZ = GetPusherZName(FRONT_CONV);
    double Position = ReadPosition(strPusherZ), PCBThickness = 1.0, DownOffset = 10.0, Ratio = 0.5, Offset = 0.0;
    long TimeOut = TIME5000MS, Err = 0, SpeedTime = 0, From = pThis->GetFromPcb(), StartConvPos = 0;
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
    if (From == 0) // Entry
        StartConvPos = ENTRY_CONV;
    else // Work
        StartConvPos = WORK1_CONV;
    TotalGetTime = _time_get();
    while (pThis->GetRunning())
    {
        if (pThis->IsTerminated(THREAD_OUTTOEXIT_READTIME) == true)
        {
            TRACE(_T("[PWR] StartOutToExit(0x%X) Terminated\n"), pThis->GetThreadID());
            break;
        }
        bExitSet = false;
        if (pThis->IsExist(StartConvPos) == true)
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
                WritePusherZ(FRONT_CONV, WORK1_CONV, Position + Offset);
            }
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] StartOutToExit(0x%X) Error0 Quit\n"), pThis->GetThreadID());
                break;
            }
            Elapsed = _time_elapsed(GetTime);
            TRACE(_T("[PWR] PusherZ Down Elapsed:%d[ms]\n"), Elapsed);

            GetTime = _time_get();
            while (1)
            {
                gcWorkConveyor->DownStopper();
                if (gcWorkConveyor->IsStopperDn(WORK1_CONV) == true)
                {
                    Err = 0;
                    ThreadSleep(TIME50MS);
                    break;
                }
                if (_time_elapsed(GetTime) > TIME10000MS)
                {
                    Err = WORK1_STOPPER_DN_TIMEOUT;
                    break;
                }
                ThreadSleep(THREAD_OUTTOEXIT_READTIME);
            };
            Elapsed = _time_elapsed(GetTime);
            TRACE(_T("[PWR] Stopper Dn Elapsed:%d[ms]\n"), Elapsed);
            if (Err > 0)
            {
                TRACE(_T("[PWR] StartOutToExit(0x%X) Error1 Quit Err:%d\n"), pThis->GetThreadID(), Err);
                break;
            }
            gcEntryConveyor->SetReverse(false);
            gcWorkConveyor->SetReverse(false);
            gcExitConveyor->SetReverse(false);

            if (StartConvPos == ENTRY_CONV)
            {
                gcEntryConveyor->BeltOn(EntryNextOutBeltSpd);
            }
            gcWorkConveyor->BeltOn(WorkNextOutBeltSpd);
            gcExitConveyor->BeltOn(ExitPrevInBeltSpd);
            GetTime = _time_get();
            while (1)
            {
                if (pThis->IsExistSet(EXIT_CONV) == true)
                {
                    //SpeedTime = gcExitConveyor->GetHighTime();
                    //ThreadSleep(SpeedTime);
                    //SpeedTime = gcExitConveyor->GetMiddleTime();
                    //ThreadSleep(SpeedTime);
                    //SpeedTime = gcExitConveyor->GetLowTime();
                    //ThreadSleep(SpeedTime);
                    gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
                    gcWorkConveyor->BeltOff(WorkNextOutBeltSpd);
                    gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
                    bExitSet = true;
                    break;
                }
                Elapsed = _time_elapsed(GetTime);
                if (Elapsed > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_OUTTOEXIT_READTIME);
            };
            Elapsed = _time_elapsed(GetTime);
            TRACE(_T("[PWR] IsExistSet Elapsed:%d[ms]\n"), Elapsed);
            if (Err > 0)
            {
                gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
                gcWorkConveyor->BeltOff(WorkNextOutBeltSpd);
                gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
                TRACE(_T("[PWR] StartOutToExit(0x%X) Error2 Quit\n"), pThis->GetThreadID());
                break;
            }
        }
        if (Err > 0)
        {
            gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
            gcWorkConveyor->BeltOff(WorkNextOutBeltSpd);
            gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
            TRACE(_T("[PWR] StartOutToExit(0x%X) Error3 Quit\n"), pThis->GetThreadID());
            break;
        }
        if (bExitSet == true)
        {
            break;
        }
    };
    pThis->SetEnd(true);
    gcEntryConveyor->BeltOff(EntryNextOutBeltSpd);
    gcWorkConveyor->BeltOff(WorkNextOutBeltSpd);
    gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
    TotalElapsed = _time_elapsed(TotalGetTime);
    TRACE(_T("[PWR] StartOutToExit(0x%x) Quit Elapsed:%d[ms]\n"), pThis->GetThreadID(), TotalElapsed);
    return 0;
}

void COutToExit::Run(long From)
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetEnd(false);
    SetFromPcb(From);
    SetThreadName(_T("thOutToExit"));
    lpStartAddress = (_beginthreadex_proc_type)StartOutToExit;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    TRACE(_T("[PWR] COutToExit Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] COutToExit Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}