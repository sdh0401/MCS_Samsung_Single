#include "pch.h"
#include "CExitConveyor.h"
#include "CPcbExist.h"
#include "CSmemaControl.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "GlobalIODefine.h"
#include "LockDef.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
//#include "ErrorCode.h"

CExitConveyor* gcExitConveyor;
CExitConveyor::CExitConveyor(bool bReverse)
{
    SetInfo(FRONT_CONV, EXIT_CONV);
    SetStopperIO(IO_NOUSE, IO_NOUSE, IO_NOUSE, IO_NOUSE);
    SetPusherPlateIO(PusherZType::None, IO_NOUSE, IO_NOUSE, IO_NOUSE, IO_NOUSE);
    SetConveyorWidthMotorType(ConveyorMotorType::None);
    SetBeltMotorType(BeltControl::Motor);
    if (bReverse == true)
        SetBeltMotor(_T("FBTIN"));
    else
        SetBeltMotor(_T("FBTOT"));
    JogInfo LowSpd, MidSpd, HighSpd;
    MidSpd.MaxVel = 30000.0;
    MidSpd.Acc = 900000.0;
    MidSpd.Dec = 900000.0;
    MidSpd.JerkRatio = 0.5;
    LowSpd.MaxVel = 10000.0;
    LowSpd.Acc = 300000.0;
    LowSpd.Dec = 300000.0;
    LowSpd.JerkRatio = 0.5;
    HighSpd = MidSpd;
    SetBeltMotorSpeedLow(LowSpd);
    SetBeltMotorSpeedMid(MidSpd);
    SetBeltMotorSpeedHigh(HighSpd);
    SetStep(ConveyorStep::STOP);
    SetHighTime(0);
    SetMiddleTime(0);
    SetLowTime(0);
}

CExitConveyor::~CExitConveyor()
{
}

bool CExitConveyor::IsSaftyStep(ConveyorStep Step)
{
    bool bSafty = false;
    switch (Step)
    {
    case ConveyorStep::STOP:
    case ConveyorStep::START:
    case ConveyorStep::WAIT_PREV_SMEMA:
    case ConveyorStep::SEND_PREV_SMEMA:
        bSafty = true;
        break;
    case ConveyorStep::WAIT_POS:
        bSafty = false;
        break;
    case ConveyorStep::SEND_NEXT_SMEMA:
    case ConveyorStep::WAIT_NEXT_SMEMA:
        bSafty = true;
        break;
    case ConveyorStep::WAIT_OUT_POS:
    case ConveyorStep::WAIT_NEXT_SMEMA_BUSY:
        bSafty = false;
        break;
    }
    return bSafty;
}

UINT CExitConveyor::StartExitConveyor(LPVOID wParam)
{
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    CExitConveyor* pThis = reinterpret_cast<CExitConveyor*>(wParam);
    long Conv = pThis->GetConv();
    long Err = NO_ERR;
    bool MsgShow = true, bFirstGetFreeTime = true;
    bool bLineOfBalance = true;
    ConveyorStep OldStep = ConveyorStep::STOP;
    long MaxFreeTime = GetGlobalTowerYellowLampTime() * TIME1000MS;
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    ULONGLONG NextMachineFreeTime = 0, NextMachineFreeElapsedTime = 0;
    long PrevInBeltSpd, NextOutBeltSpd;
    PrevInBeltSpd = NextOutBeltSpd = BELT_SPEED_MID;
    PrevInBeltSpd = pThis->GetPrevInBeltSpd();
    NextOutBeltSpd = pThis->GetNextOutBeltSpd();
    if (pThis->GetProdRunMode() == RUN_BYPASS)
    {
        PrevInBeltSpd = NextOutBeltSpd = BELT_SPEED_MID;
    }
    pThis->BeltOff(PrevInBeltSpd);
    pThis->InitSmema(Conv);
    pThis->SetReverse(false);
    TRACE(_T("[PWR] StartExitConveyor GetProdRunMode():%d\n"), pThis->GetProdRunMode());

	ULONGLONG PrevLoadingStartTime = 0;
	ULONGLONG PrevLoadingTimeOut = pThis->GetPrevTimeOut();
	ULONGLONG NextUnloadingStartTime = 0;
	ULONGLONG NextUnloadingTimeOut = pThis->GetNextTimeOut();

    while (pThis->GetRunning() == true)
    {
		if (pThis->GetProdRunMode() == RUN_DRY) // Only Dry
		{
			TRACE(_T("[PWR] StartExitConveyor Dryrun Terminated\n"));
			break;
		}

        if (pThis->IsTerminated(THREAD_EXITCONV_READTIME) == true)
        {
            TRACE(_T("[PWR] CExitConveyor(0x%X) LastStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
            if (pThis->IsSaftyStep(pThis->GetStep()) == true)
            {
                TRACE(_T("[PWR] CExitConveyor(0x%X) SaftyStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
                break;
            }
        }
        if ((OldStep != pThis->GetStep()) && (MsgShow == true))
        {
            Elapsed = _time_elapsed(GetTime);
            if (gcPowerLog->IsShowConveyorLog() == true)
            {
                TRACE(_T("[PWR] (%d) StartExitConveyor(%d) Step:%d Elapsed:%d[ms]\n"), pThis->GetPos(), Conv, pThis->GetStep(), Elapsed);
            }
            OldStep = pThis->GetStep();
            GetTime = _time_get();
        }
        if (GetGlobalStatusError() == true)
        {
            //TRACE(_T("[PWR] StartExitConveyor(0x%X) GetGlobalStatusError(%d)\n"), pThis->GetTaskID(), GetGlobalStatusError());
            if (pThis->IsSaftyStep(pThis->GetStep()) == true)
            {
                TRACE(_T("[PWR] StartExitConveyor(0x%X) GetGlobalStatusError(%d) SaftyStep:%d\n"), pThis->GetTaskID(), GetGlobalStatusError(), pThis->GetStep());
                break;
            }
        }
        switch (pThis->GetStep())
        {
        case ConveyorStep::STOP:
            break;

        case ConveyorStep::START:
            if (pThis->IsExistEnt(Conv) == true)
            {
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                ThreadSleep(TIME20MS);
                bFirstGetFreeTime = true;
                pThis->BeltOn(PrevInBeltSpd);
				PrevLoadingStartTime = _time_get();
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_POS);
                }
            }
            else if (pThis->IsExistSet(Conv) == true)
            {
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                ThreadSleep(TIME20MS);
                bFirstGetFreeTime = true;
				PrevLoadingStartTime = _time_get();
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_POS);
                }
            }
            else
            {
                pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
                pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
                ThreadSleep(TIME20MS);
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_PREV_SMEMA);
                }
            }
            break;

        case ConveyorStep::WAIT_PREV_SMEMA:
            if (pThis->CheckPrevSmema(Conv) == SMEMA_READY)
            {
                if (bLoop != false)
                {
                    MsgShow = true;
                    pThis->SetStep(ConveyorStep::SEND_PREV_SMEMA);
                }
            }
            //else
            //{
            //    if (bLoop != false)
            //    {
            //        MsgShow = false;
            //        pThis->SetStep(ConveyorStep::START);
            //    }
            //}
            break;

        case ConveyorStep::SEND_PREV_SMEMA:
            bFirstGetFreeTime = true;
            pThis->BeltOn(PrevInBeltSpd);
            pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
            ThreadSleep(TIME20MS);            
			PrevLoadingStartTime = _time_get();
            if (bLoop != false)
            {
                pThis->SetStep(ConveyorStep::WAIT_POS);
            }
            break;

        case ConveyorStep::WAIT_POS:     

            if (pThis->IsExistSet(Conv) == true)
            {
                if (bLineOfBalance == true)
                {
                    StartLineOfBalanceConveyor();
                    bLineOfBalance = false;
                }
                else
                {
                    EndLineOfBalanceConveyor();
                    StartLineOfBalanceConveyor();
                }
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                pThis->BeltOff(NextOutBeltSpd);
                StartFreeTimeConveyor(EXIT_CONV);                        
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::SEND_NEXT_SMEMA);
                }
            }
			else if (_time_elapsed(PrevLoadingStartTime) > PrevLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Work To Outlet) TimeOut:%d"), PrevLoadingTimeOut);
				SendAlarm(CONV_WORK_TO_EXIT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To Outlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            
            break;

        case ConveyorStep::SEND_NEXT_SMEMA:
            pThis->NextSmemaOut(Conv, SMEMA_READY);
            ThreadSleep(TIME20MS);
            if (bLoop != false)
            {
                pThis->SetStep(ConveyorStep::WAIT_NEXT_SMEMA);
            }
            break;

        case ConveyorStep::WAIT_NEXT_SMEMA:
            if (pThis->CheckNextSmema(Conv) == SMEMA_NOT_BUSY)
            {
                bFirstGetFreeTime = true;
                pThis->BeltOn(NextOutBeltSpd);
                TowerLampOutPcb();
				NextUnloadingStartTime = _time_get();
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_OUT_POS);
                }
            }
            else
            {
                if (pThis->IsExistSet(Conv) == false) // 손으로 뺐어? 누구여? 다시 첨으로 가자
                {
                    pThis->SetStep(ConveyorStep::START);
                }
                else
                {
                    if (pThis->GetProdRunMode() == RUN_DRY) // Only Dry
                    {
                        bFirstGetFreeTime = true;
                        NextMachineFreeTime = _time_get();
                    }
                    else
                    {
                        if (pThis->IsStartGetFreeTime() == true)
                        {
                            if (bFirstGetFreeTime == true)
                            {
                                NextMachineFreeTime = _time_get();
                                bFirstGetFreeTime = false;
                                if (gcPowerLog->IsShowTowerLampLog() == true)
                                {
                                    TRACE(_T("[PWR] ***** Conv:%d Start FreeTimer On *****\n"), Conv);
                                }
                            }
                            else
                            {
                                NextMachineFreeElapsedTime = _time_elapsed(NextMachineFreeTime);
                                if (NextMachineFreeElapsedTime > MaxFreeTime)
                                {
                                    if (gcPowerLog->IsShowTowerLampLog() == true)
                                    {
                                        TRACE(_T("[PWR] ***** Conv:%d TowerLampWaitPcb On *****\n"), Conv);
                                    }
                                    TowerLampWaitPcb(Conv, CONV_EXIT_WAIT_NEXT_REQUEST(FRONT_CONV));
                                    NextMachineFreeTime = _time_get();
                                }
                            }
                        }
                    }
                }
            }
            break;

        case ConveyorStep::WAIT_OUT_POS:
            if (pThis->IsExistSet(Conv) == false)
            {
                //pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
                //ThreadSleep(TIME20MS);
                //pThis->BeltOff(NextOutBeltSpd);
                bFirstGetFreeTime = true;
                if (bLoop != false)
                {
                    //pThis->SetStep(ConveyorStep::START);
                    pThis->SetStep(ConveyorStep::WAIT_NEXT_SMEMA_BUSY);
                }
            }
			else if (_time_elapsed(NextUnloadingStartTime) > NextUnloadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Outlet To NextMachine) TimeOut:%d"), NextUnloadingTimeOut);
				SendAlarm(CONV_EXIT_TO_NEXT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To NextMachine)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        case ConveyorStep::WAIT_NEXT_SMEMA_BUSY:
            if (pThis->CheckNextSmema(Conv) == SMEMA_BUSY)
            {
                pThis->BeltOff(NextOutBeltSpd);
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::START);
                }
            }
			else if (_time_elapsed(NextUnloadingStartTime) > NextUnloadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Outlet To NextMachine) TimeOut:%d"), NextUnloadingTimeOut);
				SendAlarm(CONV_EXIT_TO_NEXT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To NextMachine)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        default:
            break;
        }
        ThreadSleep(THREAD_EXITCONV_READTIME);
    };
	pThis->BeltOff(NextOutBeltSpd);

    pThis->InitSmema(Conv);
    pThis->SetEnd(true);
    TRACE(_T("[PWR] CExitConveyor(0x%X) LastStep(%d) Quit\n"), pThis->GetTaskID(), pThis->GetStep());
    return 0;
}

void CExitConveyor::Run(long ProdMode)
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetProdRunMode(ProdMode);
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thExitConveyor"));
    lpStartAddress = (_beginthreadex_proc_type)StartExitConveyor;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    SetTaskID(nID);    
    TRACE(_T("[PWR] Exit Conveyor Thread ID:0x%04X(%s)\n"), GetTaskID(), GetThreadName());
    strLog.Format(_T("[PWR] Exit Conveyor Thread ID:0x%04X(%s)"), GetTaskID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
    SetStep(ConveyorStep::STOP);
}


