#include "pch.h"
#include "CEntryConveyor.h"
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

CEntryConveyor* gcEntryConveyor;
CEntryConveyor::CEntryConveyor(bool bReverse)
{
    SetInfo(FRONT_CONV, ENTRY_CONV);
    SetStopperIO(IO_NOUSE, IO_NOUSE, IO_NOUSE, IO_NOUSE);
    SetPusherPlateIO(PusherZType::None, IO_NOUSE, IO_NOUSE, IO_NOUSE, IO_NOUSE);
    SetConveyorWidthMotorType(ConveyorMotorType::None);
    SetBeltMotorType(BeltControl::Motor);
    if(bReverse == true)
        SetBeltMotor(_T("FBTOT"));
    else
        SetBeltMotor(_T("FBTIN"));
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

CEntryConveyor::~CEntryConveyor()
{
}

bool CEntryConveyor::IsSaftyStep(ConveyorStep Step)
{
    bool bSafty = false;
    switch (Step)
    {
    case ConveyorStep::STOP:
    case ConveyorStep::START:
    case ConveyorStep::WAIT_PREV_SMEMA:
    case ConveyorStep::INSPECT_BARCODE:
    case ConveyorStep::SEND_BARCODE:
    case ConveyorStep::WAIT_BARCODE_RESULT:
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

UINT CEntryConveyor::StartEntryConveyor(LPVOID wParam)
{
    CEntryConveyor* pThis = reinterpret_cast<CEntryConveyor*>(wParam);
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    long Conv = pThis->GetConv();
    long Err = NO_ERR;
    bool MsgShow = true, bFirstGetFreeTime = true, bSendFreeTime = true;
    ConveyorStep OldStep = ConveyorStep::STOP;
    long MaxFreeTime = GetGlobalTowerYellowLampTime() * TIME1000MS;
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    ULONGLONG PrevMachineFreeTime = 0, PrevMachineFreeElapsedTime = 0;
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

	long PrevLoadingTimeOut = pThis->GetPrevTimeOut();
	long NextLoadingTimeOut = pThis->GetNextTimeOut();
	ULONGLONG PrevLoadingStartTime = 0;
	ULONGLONG NextLoadingStartTime = 0;

    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_ENTRYCONV_READTIME) == true)
        {
            TRACE(_T("[PWR] CEntryConveyor(0x%X) LastStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
            if (pThis->IsSaftyStep(pThis->GetStep()) == true)
            {
                TRACE(_T("[PWR] StartEntryConveyor(0x%X) SaftyStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
                break;
            }
        }
        if ((OldStep != pThis->GetStep()) && (MsgShow == true))
        {
            Elapsed = _time_elapsed(GetTime);
            if (gcPowerLog->IsShowConveyorLog() == true)
            {
                TRACE(_T("[PWR] (%d) StartEntryConveyor(%d) Step:%d Elapsed:%d[ms]\n"), pThis->GetPos(), Conv, pThis->GetStep(), Elapsed);
            }
            OldStep = pThis->GetStep();
            GetTime = _time_get();
        }
        if (GetGlobalStatusError() == true)
        {
            //TRACE(_T("[PWR] StartEntryConveyor(0x%X) GetGlobalStatusError(%d)\n"), pThis->GetTaskID(), GetGlobalStatusError());
            if (pThis->IsSaftyStep(pThis->GetStep()) == true)
            {
                TRACE(_T("[PWR] StartEntryConveyor(0x%X) GetGlobalStatusError(%d) SaftyStep:%d\n"), pThis->GetTaskID(), GetGlobalStatusError(), pThis->GetStep());
                break;
            }
        }


		if (GetRunModeNoLog() == PAUSE_MODE)
		{
			if (bFirstGetFreeTime == true)
			{
				PrevMachineFreeTime = _time_get();
				bFirstGetFreeTime = false;
				if (gcPowerLog->IsShowTowerLampLog() == true)
				{
					TRACE(_T("[PWR] ***** Conv:%d Start FreeTimer On Pause*****\n"), Conv);
				}
			}
			else
			{
				PrevMachineFreeElapsedTime = _time_elapsed(PrevMachineFreeTime);
				if (PrevMachineFreeElapsedTime > TIME1000MS)
				{
					if (gcPowerLog->IsShowTowerLampLog() == true)
					{
						TRACE(_T("[PWR] ***** Conv:%d TowerLampWaitPcb On Pause*****\n"), Conv);
					}
					TowerLampWaitPcb(Conv, CONV_ENTRY_WAIT_PREV_READY(FRONT_CONV));
					PrevMachineFreeTime = _time_get();
				}
			}
		}


        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_ENTRYCONV_READTIME);
        //    continue;
        //}
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
                TowerLampInPcb();
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
                if (pThis->GetProdRunMode() == RUN_DRY) // Only Dry
                {
                    pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                    if (bLoop != false)
                    {
                        pThis->SetStep(ConveyorStep::START);
                    }
                }
                else
                {
                    pThis->SetPcbReady(false);
                    pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
                    pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
                    ThreadSleep(TIME20MS);
                    if (pThis->IsStartGetFreeTime() == true)
                    {
                        if (bFirstGetFreeTime == true)
                        {
                            PrevMachineFreeTime = _time_get();
                            bFirstGetFreeTime = false;
                            if (gcPowerLog->IsShowTowerLampLog() == true)
                            {
                                TRACE(_T("[PWR] ***** Conv:%d Start FreeTimer On *****\n"), Conv);
                            }
                        }
                        else
                        {
                            PrevMachineFreeElapsedTime = _time_elapsed(PrevMachineFreeTime);
                            if (PrevMachineFreeElapsedTime > MaxFreeTime)
                            {
                                if (gcPowerLog->IsShowTowerLampLog() == true)
                                {
                                    TRACE(_T("[PWR] ***** Conv:%d TowerLampWaitPcb On *****\n"), Conv);
                                }
                                TowerLampWaitPcb(Conv, CONV_ENTRY_WAIT_PREV_READY(FRONT_CONV));
                                PrevMachineFreeTime = _time_get();
                            }
                        }
                    }
                    else
                    {
                        bFirstGetFreeTime = true;
                        PrevMachineFreeTime = _time_get();
                    }
                    if (bLoop != false)
                    {
                        pThis->SetStep(ConveyorStep::WAIT_PREV_SMEMA);
                    }
                }
            }
            break;

        case ConveyorStep::WAIT_PREV_SMEMA:
            if (pThis->CheckPrevSmema(Conv) == SMEMA_READY)
            {
                //SetEntryPcbReady(FRONT_CONV, false);
                if (bLoop != false)
                {
                    MsgShow = true;
                    pThis->SetStep(ConveyorStep::SEND_PREV_SMEMA);
                }
            }
            else
            {
                if (bLoop != false)
                {
                    MsgShow = false;
                    pThis->SetStep(ConveyorStep::START);
                }
            }
            break;

        case ConveyorStep::SEND_PREV_SMEMA:
            bFirstGetFreeTime = true;
            pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
            ThreadSleep(TIME20MS);
            pThis->BeltOn(PrevInBeltSpd);
            TowerLampInPcb();
			PrevLoadingStartTime = _time_get();
            if (bLoop != false)
            {
                pThis->SetStep(ConveyorStep::WAIT_POS);
            }
            break;

        case ConveyorStep::WAIT_POS:
            if (pThis->IsExistSet(Conv) == true)
            {
                pThis->SetPcbReady(true);
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                pThis->BeltOff(PrevInBeltSpd);
                pThis->NextSmemaOut(Conv, SMEMA_READY);                
                ThreadSleep(TIME20MS);
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::SEND_NEXT_SMEMA);
                }
            }
			else if (_time_elapsed(PrevLoadingStartTime) > PrevLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(PrevMachine To Inlet) TimeOut:%d"), PrevLoadingTimeOut);
				SendAlarm(CONV_PREV_TO_ENTRY_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(PrevMachine To Inlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            //else
            //{
            //    pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
            //}
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
                pThis->EntryLoadingStart();
				NextLoadingStartTime = _time_get();
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_OUT_POS);
                }
            }
            break;

        case ConveyorStep::WAIT_OUT_POS:
            if (pThis->IsExistSet(Conv) == false)
            {
                StartFreeTimeConveyor(ENTRY_CONV);
                ThreadSleep(TIME20MS);
                if (bLoop != false)
                {
                    //pThis->SetStep(ConveyorStep::START);
                    pThis->SetStep(ConveyorStep::WAIT_NEXT_SMEMA_BUSY);
                }
            }
			else if (_time_elapsed(NextLoadingStartTime) > NextLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Inlet To Work) TimeOut:%d"), NextLoadingTimeOut);
				SendAlarm(CONV_PREV_TO_ENTRY_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Inlet To Work)"));
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
			else if (_time_elapsed(NextLoadingStartTime) > NextLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Inlet To Work) TimeOut:%d"), NextLoadingTimeOut);
				SendAlarm(CONV_PREV_TO_ENTRY_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Inlet To Work)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        default:
            break;
        }
        ThreadSleep(THREAD_ENTRYCONV_READTIME);
    };
	pThis->BeltOff(NextOutBeltSpd);

    pThis->InitSmema(Conv);
    pThis->SetEnd(true);
    TRACE(_T("[PWR] CEntryConveyor(0x%X) LastStep(%d) Quit\n"), pThis->GetTaskID(), pThis->GetStep());
    return 0;
}

void CEntryConveyor::Run(long ProdMode)
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetProdRunMode(ProdMode);
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thEntryConveyor"));
    lpStartAddress = (_beginthreadex_proc_type)StartEntryConveyor;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    SetTaskID(nID);    
    TRACE(_T("[PWR] Entry Conveyor Thread ID:0x%04X(%s)\n"), GetTaskID(), GetThreadName());
    strLog.Format(_T("[PWR] Entry Conveyor Thread ID:0x%04X(%s)"), GetTaskID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
    SetStep(ConveyorStep::STOP);
}


