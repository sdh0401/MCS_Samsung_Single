#include "pch.h"
#include "CReturnToEntrance.h"
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

CReturnToEntrance* gcReturnToEntrance;
CReturnToEntrance::CReturnToEntrance()
{
}

CReturnToEntrance::~CReturnToEntrance()
{
}

UINT CReturnToEntrance::StartReturnToEntrance(LPVOID wParam)
{
    bool bLoop = true, bEnt = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    CReturnToEntrance* pThis = reinterpret_cast<CReturnToEntrance*>(wParam);
    CApplicationTime* pTime = new CApplicationTime();
    CString strPusherZ = GetPusherZName(FRONT_CONV);
    double Position = ReadPosition(strPusherZ), PCBThickness = 1.0, DownOffset = 0.0, Offset = 10.0, Ratio = 0.5;
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
    if (From == 1) // Work
        StartConvPos = WORK1_CONV;
    else // Exit
        StartConvPos = EXIT_CONV;
    while (pThis->GetRunning())
    {
        bEnt = false;
        if (pThis->IsExist(StartConvPos) == true)
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
                TRACE(_T("[PWR] CReturnToEntrance(0x%X) Error0 Quit Err:%d\n"), pThis->GetThreadID(), Err);
                break;
            }
            pTime->TimeGet();
            if (abs(Offset) > 0.001 && GetOnlyConveyorMode() == false)
            {
                Ratio = gcWorkConveyor->GetPusherDownRatio();
                Err = StartPosWaitMotion(strPusherZ, Ratio, TimeOut, Position + Offset, true);
                WritePusherZ(FRONT_CONV, WORK1_CONV, Position + Offset);
            }
            TRACE(_T("[PWR] PusherZ Down Elapsed:%d[ms]\n"), pTime->TimeElapsed());
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CReturnToEntrance(0x%X) Error1 Quit Err:%d\n"), pThis->GetThreadID(), Err);
                break;
            }            
            gcEntryConveyor->SetReverse(true);
            gcWorkConveyor->SetReverse(true);
            gcExitConveyor->SetReverse(true);

            gcEntryConveyor->BeltOn(EntryPrevInBeltSpd);
            gcWorkConveyor->BeltOn(WorkPrevInBeltSpd);
            if (StartConvPos == EXIT_CONV)
            {
                gcExitConveyor->BeltOn(ExitPrevInBeltSpd);
            }
            pTime->TimeGet();
            while (1)
            {                
                if (gcEntryConveyor->IsExistEnt(ENTRY_CONV) == true)
                {
                    //SpeedTime = gcEntryConveyor->GetHighTime();
                    //ThreadSleep(SpeedTime);
                    //SpeedTime = gcEntryConveyor->GetMiddleTime();
                    //ThreadSleep(SpeedTime);
                    //SpeedTime = gcEntryConveyor->GetLowTime();
                    //ThreadSleep(SpeedTime);
                    gcEntryConveyor->BeltOff(EntryPrevInBeltSpd);
                    gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
                    gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
                    bEnt = true;
                    break;
                }
                if (pTime->TimeElapsed() > TIME10000MS)
                {
                    Err = 1;
                    break;
                }
                ThreadSleep(THREAD_RETURN_TO_ENTRANCE_READTIME);
            };
            TRACE(_T("[PWR] Entry Entrance Sensor Elapsed:%d[ms]\n"), pTime->TimeElapsed());
        }
        if (Err > 0)
        {
            gcEntryConveyor->BeltOff(EntryPrevInBeltSpd);
            gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
            gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
            TRACE(_T("[PWR] CReturnToEntrance(0x%X) Error2 Quit\n"), pThis->GetThreadID());
            break;
        }
        if (bEnt == true)
        {
            break;
        }
    };
    pThis->SetEnd(true);
    gcEntryConveyor->BeltOff(EntryPrevInBeltSpd);
    gcWorkConveyor->BeltOff(WorkPrevInBeltSpd);
    gcExitConveyor->BeltOff(ExitPrevInBeltSpd);
    TRACE(_T("[PWR] CReturnToEntrance(0x%x) Quit Elapsed:%d[ms]\n"), pThis->GetThreadID(), pTime->TimeElapsed());
    delete pTime;
    return 0;
}

void CReturnToEntrance::Run(long From)
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetEnd(false);
    SetFromPcb(From);
    SetThreadName(_T("thReturnToEnt"));
    lpStartAddress = (_beginthreadex_proc_type)StartReturnToEntrance;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] CReturnToEntrance Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] CReturnToEntrance Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}