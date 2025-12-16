#include "pch.h"
#include "CWorkConveyor.h"
#include "CPcbExist.h"
#include "CSmemaControl.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
//#include "ErrorCode.h"
#include "CReadJobFile.h"

CWorkConveyor* gcWorkConveyor;
CWorkConveyor::CWorkConveyor(bool bReverse)
{
    SetInfo(FRONT_CONV, WORK1_CONV);
    SetStopperIO(OUT_FCONV_WORK1_STOP_UP, IN_FCONV_WORK1_STOP_UP, IO_NOUSE, IO_NOUSE);
    SetPusherPlateIO(PusherZType::Motor, IO_NOUSE, IO_NOUSE, IO_NOUSE, IO_NOUSE);
    SetConveyorWidthMotorType(ConveyorMotorType::Motor);
    SetBeltMotorType(BeltControl::Motor);
    SetBeltMotor(_T("FBTWK"));
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
    SetPusherPlateZMotor(GetPusherZName(FRONT_CONV));
    SetConveyorWidthMotor(GetConvName(FRONT_CONV));
    SetStep(ConveyorStep::STOP);
    SetHighTime(0);
    SetMiddleTime(TIME500MS);
    SetLowTime(TIME300MS);
}

CWorkConveyor::~CWorkConveyor()
{
}

bool CWorkConveyor::IsSaftyStep(ConveyorStep Step, long SimulLoadType)
{
    bool bSafty = false;
    switch (Step)
    {
    case ConveyorStep::STOP:
    case ConveyorStep::START:
    case ConveyorStep::WAIT_PREV_SMEMA:
    case ConveyorStep::PREPARE_PUSHERZ:
    case ConveyorStep::CHECK_PUSHERZ_PREPARE:
    case ConveyorStep::SEND_PREV_SMEMA:
        bSafty = true;
        break;
    case ConveyorStep::WAIT_LOW:
    case ConveyorStep::WAIT_POS:
        bSafty = false;
        break;
    case ConveyorStep::UP_PUSHERZ:
    case ConveyorStep::CHECK_PUSHERZ_UP:
    case ConveyorStep::PCB_READY:
    case ConveyorStep::DOWN_STOPPER:
    case ConveyorStep::WAIT_WORK_DONE:
    case ConveyorStep::DOWN_PUSHERZ:
    case ConveyorStep::CHECK_PUSHERZ_DOWN:
    case ConveyorStep::DOWN_STOPPER_AFTER_DONE:
    case ConveyorStep::SEND_NEXT_SMEMA:
    case ConveyorStep::WAIT_NEXT_SMEMA:
        bSafty = true;
        break;
    case ConveyorStep::WAIT_OUT_POS:
    case ConveyorStep::WAIT_PCBOUT:
    case ConveyorStep::WAIT_OUT_PCBOUT:
    case ConveyorStep::WAIT_NEXT_SMEMA_BUSY:
        bSafty = false;
        break;

	case ConveyorStep::UP_STOPPER:
		if (SimulLoadType == 0)
		{
			bSafty = true;
		}
		else
		{
			bSafty = false;
		}
		break;
    }
    return bSafty;
}

UINT CWorkConveyor::StartWorkConveyor(LPVOID wParam)
{
    bool bLoop = true, bEntryPcbReady = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    CWorkConveyor* pThis = reinterpret_cast<CWorkConveyor*>(wParam);
    long Conv = pThis->GetConv();
    long TimeChk = 0, TimeOut = TIME3000MS, SpeedTime = 0, Err = NO_ERR;
    bool MsgShow = true;
    const double pusherZUpRatio = gcReadJobFile->GetPcb().PusherRatioUp / 100.0;
    const double pusherZDnRatio = gcReadJobFile->GetPcb().PusherRatioDown / 100.0;
    double PusherZ = GetPusherByZ(FRONT_CONV);

    //double PcbThickness = pThis->GetPcbThickness();
    //double PcbStandByZOffset = pThis->GetPcbStandByZOffset();
	double PcbThickness = gcReadJobFile->GetPcb().Thickness;
    double PcbStandByZOffset = gcReadJobFile->GetPcb().StandByPusherZOffsetHeight;
    long PusherRatioUp = gcReadJobFile->GetPcb().PusherRatioUp / 10;
    long PusherRatioDown = gcReadJobFile->GetPcb().PusherRatioDown / 10;
    pThis->SetPusherUpRatio(PusherRatioUp);
    pThis->SetPusherDownRatio(PusherRatioDown);
    TRACE(_T("[PWR] PusherZ,PcbThickness,PcbStandByZOffset,%.3f,%.3f,%.3f Pusher Up/Down %03d%%/%03d%%\n"), PusherZ, PcbThickness, PcbStandByZOffset, PusherRatioUp * 10, PusherRatioDown * 10);
    ConveyorStep OldStep = ConveyorStep::STOP;
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    ULONGLONG PusherZGetTime = _time_get(), PusherZElapsed = 0;
    CString strPusherZ = pThis->GetPusherPlateZMotor();
    long PrevInBeltSpd, NextOutBeltSpd, LowBeltSpd;
    LowBeltSpd = BELT_SPEED_LOW;
    PrevInBeltSpd = NextOutBeltSpd = BELT_SPEED_MID;
    PrevInBeltSpd = pThis->GetPrevInBeltSpd();
    NextOutBeltSpd = pThis->GetNextOutBeltSpd();    

    long UseSimulLoadingType = gcReadJobFile->GetPcb().SimulLoadType;
	bool LatchOutOn = false;
	bool LatchLowOn = false;
	bool LatchSetOff = false;

	long PrevLoadingTimeOut = pThis->GetPrevTimeOut();
	long NextLoadingTimeOut = pThis->GetNextTimeOut();
	ULONGLONG PrevLoadingStartTime = 0;
	ULONGLONG NextLoadingStartTime = 0;

	bool bIsWaitPrev = false;
	bool bIsWaitNext = false;

	if (pThis->GetProdRunMode() == RUN_BYPASS && UseSimulLoadingType > 0)
	{
		UseSimulLoadingType = 0;
		TRACE(_T("[PWR] SimulLoad Off (bypass)"));
	}

    if (UseSimulLoadingType > 0)
    {
        TRACE(_T("[PWR] SimulLoad Type(%d)"), UseSimulLoadingType);
        if (PrevInBeltSpd < NextOutBeltSpd)
        {
            TRACE(_T("[PWR] SimulLoad Work NextOut Speed Down(%d,%d)"), PrevInBeltSpd, NextOutBeltSpd);
            NextOutBeltSpd = PrevInBeltSpd;
        }
        else if (PrevInBeltSpd > NextOutBeltSpd)
        {
            TRACE(_T("[PWR] SimulLoad Work PrevIn Speed Down"), PrevInBeltSpd, NextOutBeltSpd);
            PrevInBeltSpd = NextOutBeltSpd;
        }
    }


    if (pThis->GetProdRunMode() == RUN_BYPASS)
    {
        PrevInBeltSpd = NextOutBeltSpd = BELT_SPEED_MID;
    }
    pThis->BeltOff(PrevInBeltSpd);
    pThis->InitSmema(Conv);
    pThis->SetReverse(false);
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_WORKCONV_READTIME) == true)
        {
            TRACE(_T("[PWR] CWorkConveyor(0x%X) LastStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
            if (pThis->IsSaftyStep(pThis->GetStep(), UseSimulLoadingType) == true)
            {
                TRACE(_T("[PWR] StartWorkConveyor(0x%X) SaftyStep(%d) Terminated\n"), pThis->GetTaskID(), pThis->GetStep());
            }

        }
        if ((OldStep != pThis->GetStep()) && (MsgShow == true))
        {
            Elapsed = _time_elapsed(GetTime);
            if (gcPowerLog->IsShowConveyorLog() == true)
            {
                TRACE(_T("[PWR] (%d) StartWorkConveyor(%d) Step:%d Elapsed:%d[ms]\n"), pThis->GetPos(), Conv, pThis->GetStep(), Elapsed);
            }
            OldStep = pThis->GetStep();
            GetTime = _time_get();
        }
        if (GetGlobalStatusError() == true)
        {
            if (pThis->IsSaftyStep(pThis->GetStep(), UseSimulLoadingType) == true)
            {
                TRACE(_T("[PWR] StartWorkConveyor(0x%X) GetGlobalStatusError(%d) SaftyStep:%d\n"), pThis->GetTaskID(), GetGlobalStatusError(), pThis->GetStep());
                break;
            }
        }

		if (GetOnlyConveyorMode() == true)
		{
			long err = CheckMachineSafety();

			if (err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Bypass Error"));
			}
		}

        switch (pThis->GetStep())
        {
        case ConveyorStep::STOP:
            break;

        case ConveyorStep::START:
            if (pThis->IsSimulationOn() == true)
            {
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
				PrevLoadingStartTime = _time_get();

                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::WAIT_POS);
                }
            }
            else if (pThis->IsExistEnt(Conv) == true)
            {
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                ThreadSleep(TIME20MS);
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
            pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
            pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
            ThreadSleep(TIME20MS);

            if (pThis->CheckPrevSmema(Conv) == SMEMA_READY)
            {
                SetWorkPcbReady(FRONT_CONV, false); // WORK로 PCB가 투입될 때
                SetInsertDone(0); // WORK로 PCB가 투입될 때
                SetHeightMeasureDone(0);

				if (bIsWaitPrev == true)
				{
					SendToChangeMachineState(TimeStatics::ENTRY_WAIT_END);
				}
				bIsWaitPrev = false;

                if (bLoop != false)
                {
                    MsgShow = true;
                    pThis->SetStep(ConveyorStep::PREPARE_PUSHERZ);
                }
            }
			else
			{
				if (bIsWaitPrev == false)
				{
					bIsWaitPrev = true;
					SendToChangeMachineState(TimeStatics::ENTRY_WAIT_START);
				}
			}
            break;

        case ConveyorStep::PREPARE_PUSHERZ:
			if (pThis->GetProdRunMode() == RUN_BYPASS)
			{
				pThis->SetStep(ConveyorStep::UP_STOPPER);
			}
            else if (CheckServoOn(strPusherZ) == false) // 20210420 HarkDo
            {
				Err = GetServoOnError(strPusherZ);
                Err = SendAlarm(Err, _T("Pusher Z Servo Off(Prepare Error)"));
                break;
            }
            else
            {
                PusherZGetTime = _time_get();
                Err = StartPosWaitMotion(strPusherZ, pusherZDnRatio, TimeOut, PusherZ + PcbStandByZOffset, true);
                TRACE(_T("[PWR] PREPARE_PUSHERZ %s Height:%.3f Elapsed,%d\n"), strPusherZ, PusherZ + PcbStandByZOffset, _time_elapsed(PusherZGetTime));
                if (Err != NO_ERR)
                {
                    Err = SendAlarm(Err, _T("Pusher Z Prepare Error"));
                    break;
                }
                if (bLoop != false)
                {
                    if (GetInfiniteDryRun() == 1 && pThis->GetProdRunMode() == RUN_DRY)
                    {
                        pThis->SetStep(ConveyorStep::SEND_PREV_SMEMA);
                    }
                    else
                    {
                        pThis->SetStep(ConveyorStep::UP_STOPPER);
                    }
                }
            }
            break;

        case ConveyorStep::UP_STOPPER:
            if (pThis->IsStopperUp(Conv) == true)
            {
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::SEND_PREV_SMEMA);
                }
            }
            else
            {
                pThis->UpStopper();
            }
            break;

        case ConveyorStep::SEND_PREV_SMEMA:
            pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
            ThreadSleep(TIME20MS);
            pThis->BeltOn(PrevInBeltSpd);
			PrevLoadingStartTime = _time_get();

            if (bLoop != false)
            {
                pThis->SetStep(ConveyorStep::WAIT_LOW);
            }
            break;

        case ConveyorStep::WAIT_LOW:
            if (pThis->IsExistLow(Conv) == true || pThis->IsExistSet(Conv) == true)
            {
                if (bLoop != false)
                {
                    SpeedTime = pThis->GetMiddleTime();
                    ThreadSleep(SpeedTime);
                    pThis->BeltOn(BELT_SPEED_LOW);
                    pThis->SetStep(ConveyorStep::WAIT_POS);
                }
            }
			else if (_time_elapsed(PrevLoadingStartTime) > PrevLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Inlet To Work) TimeOut:%d"), PrevLoadingTimeOut);
				SendAlarm(CONV_ENTRY_TO_WORK_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(PrevMachine To Inlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        case ConveyorStep::WAIT_POS:
            if (pThis->IsExistSet(Conv) == true)
            {
                //SetWorkPcbOut(FRONT_CONV, false);
                //SetPcbOutDone(0);
                pThis->PrevSmemaOut(Conv, SMEMA_BUSY);
                //SpeedTime = pThis->GetHighTime();
                //ThreadSleep(SpeedTime);
                SpeedTime = pThis->GetLowTime();
                ThreadSleep(SpeedTime);
                pThis->BeltOff(BELT_SPEED_LOW);
                if (GetMaxBoardCount() == 1) {                    
                    SetLoadable(false);
                }
                if (bLoop != false)
                {
                    if (pThis->GetProdRunMode() == RUN_BYPASS)
                    {
                        pThis->SetStep(ConveyorStep::DOWN_STOPPER);
                    }
                    else
                    {
                        pThis->SetStep(ConveyorStep::UP_PUSHERZ);
                    }
                }
            }
			else if (_time_elapsed(PrevLoadingStartTime) > PrevLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Inlet To Work) TimeOut:%d"), PrevLoadingTimeOut);
				SendAlarm(CONV_ENTRY_TO_WORK_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(PrevMachine To Inlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        case ConveyorStep::UP_PUSHERZ:
            if (CheckServoOn(strPusherZ) == false) // 20210420 HarkDo
            {
				Err = GetServoOnError(strPusherZ);

                Err = SendAlarm(Err, _T("Pusher Z Servo Off(Up Error)"));
                break;
            }
            else
            {
                PusherZGetTime = _time_get();
                Err = StartPosWaitMotion(strPusherZ, pusherZUpRatio, TimeOut, PusherZ + PcbThickness, true);
                TRACE(_T("[PWR] UP_PUSHERZ %s Height:%.3f Elapsed,%d\n"), strPusherZ, PusherZ + PcbThickness, _time_elapsed(PusherZGetTime));
                if (Err != NO_ERR)
                {
                    Err = SendAlarm(Err, _T("Pusher Z Up Error"));
                    break;
                }
                if (bLoop != false)
                {
					if (GetInfiniteDryRun() == 1 && pThis->GetProdRunMode() == RUN_DRY)
					{
						ThreadSleep(TIME1000MS);
					}
                    pThis->SetStep(ConveyorStep::PCB_READY);
                }
            }
            break;

        case ConveyorStep::PCB_READY:
            if (bLoop != false)
            {
                pThis->WorkLoadingElapsed();
                SetWorkPcbReady(FRONT_CONV, true);
                pThis->SetStep(ConveyorStep::DOWN_STOPPER);
            }
            break;
        
        case ConveyorStep::DOWN_STOPPER:
            if (pThis->IsStopperDn(Conv) == true)
            {
                if (bLoop != false)
                {
					if (GetGlobalSimulationMode() == true)
					{

					}
					else if (GetInfiniteDryRun() == 1 && pThis->GetProdRunMode() == RUN_DRY)
					{
						pThis->SetStep(ConveyorStep::DOWN_PUSHERZ);
					}
                    else if (pThis->GetProdRunMode() == RUN_BYPASS)
                    {
                        pThis->SetStep(ConveyorStep::DOWN_STOPPER_AFTER_DONE);
                    }
                    else
                    {
                        pThis->SetStep(ConveyorStep::WAIT_WORK_DONE);
                    }
                }
            }
            else
            {
                pThis->DownStopper();
            }

			if (GetGlobalSimulationMode() == true)
			{
				pThis->SetStep(ConveyorStep::WAIT_WORK_DONE);
			}

            break;

        case ConveyorStep::WAIT_WORK_DONE:
            if (GetInsertDone(FRONT_CONV) == 1)
            {
				if (GetConveyorRunMode() == LOCATION_STAY_WORK)
				{
					//SendAlarm(BOARD_STOP, _T("Stop by PCB location:Stop"));
					SendPopupMessage(_T("Stop by PCB location:Stop"));
					pThis->SetStep(ConveyorStep::STOP);
					break;
				}
				else if (GetStopMode() == HMI_RUN_CONTROL_STOPCYCLE || GetStopMode() == HMI_RUN_CONTROL_STOPBOARD)
				{

				}
                else if (bLoop != false)
                {
					pThis->SetStep(ConveyorStep::DOWN_PUSHERZ);
                }
            }
            else if (pThis->IsExistSet(Conv) == false)
            {
                ThreadSleep(TIME100MS);
                if (pThis->IsExistSet(Conv) == false)
                {
                    TRACE(_T("[PWR] WorkConv Set Sensor PCB Lost"));
                    SendAlarm(CONV_WORK_PCB_LOST(pThis->GetPos()), _T("WorkConv Set Sensor PCB Lost"));
                    pThis->SetStep(ConveyorStep::STOP);
                }
            }
            else
            {
                if (CheckServoOn(strPusherZ) == false) // 20210420 HarkDo
                {
					Err = GetServoOnError(strPusherZ);

                    Err = SendAlarm(Err, _T("Pusher Z Servo Off(Move Done But Inposition error over 3 Seconds)"));
                    break;
                }
            }
            break;

        case ConveyorStep::DOWN_PUSHERZ:
            if (CheckServoOn(strPusherZ) == false) // 20210420 HarkDo
            {
				Err = GetServoOnError(strPusherZ);

                Err = SendAlarm(Err, _T("Pusher Z Servo Off(Down Error)"));
                break;
            }
            else
            {
                PusherZGetTime = _time_get();
                Err = StartPosWaitMotion(strPusherZ, pusherZDnRatio, TimeOut, PusherZ + PcbStandByZOffset, true);
                TRACE(_T("[PWR] DOWN_PUSHERZ %s Height:%.3f Elapsed,%d\n"), strPusherZ, PusherZ + PcbStandByZOffset, _time_elapsed(PusherZGetTime));
                if (Err != NO_ERR)
                {
                    Err = SendAlarm(Err, _T("Pusher Z Down Error"));
                    break;
                }
                if (bLoop != false)
                {
					if (GetInfiniteDryRun() == 1 && pThis->GetProdRunMode() == RUN_DRY)
					{
						ThreadSleep(TIME1000MS);

						pThis->SetStep(ConveyorStep::UP_PUSHERZ);
					}
					else
					{
						pThis->SetStep(ConveyorStep::DOWN_STOPPER_AFTER_DONE);
					}
                }
            }
            break;

        case ConveyorStep::DOWN_STOPPER_AFTER_DONE:
            if (pThis->IsStopperDn(Conv) == true)
            {
                if (bLoop != false)
                {
                    pThis->SetStep(ConveyorStep::SEND_NEXT_SMEMA);
                }
            }
            else
            {
                pThis->DownStopper();
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
                SetWorkPcbReady(FRONT_CONV, false);
                pThis->BeltOn(NextOutBeltSpd);
				NextLoadingStartTime = _time_get();
				if (bIsWaitNext == true)
				{
					SendToChangeMachineState(TimeStatics::EXIT_WAIT_END);
				}
				bIsWaitNext = false;
                if (UseSimulLoadingType > 0 && pThis->CheckPrevSmema(Conv) == SMEMA_READY)
                {
                    if (UseSimulLoadingType == SIMULLOAD_SAMETIME)
                    {
                        pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
                    }

					LatchOutOn = false;
					LatchLowOn = false;
					LatchSetOff = false;

                    pThis->SetStep(ConveyorStep::WAIT_PCBOUT);
                }
                else
                {
                    if (bLoop != false)
                    {
                        pThis->SetStep(ConveyorStep::WAIT_OUT_POS);
                    }
                }
            }
			else
			{
				if (bIsWaitNext == false)
				{
					bIsWaitNext = true;
					SendToChangeMachineState(TimeStatics::EXIT_WAIT_START);
				}
			}
            break;

        case ConveyorStep::WAIT_OUT_POS:
            if (pThis->IsExistSet(Conv) == false)
            {
                pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
                ThreadSleep(TIME20MS);
                if (bLoop != false)
                {
                    ThreadSleep(TIME100MS);

                 //   pThis->BeltOff(NextOutBeltSpd);

                    pThis->SetStep(ConveyorStep::WAIT_NEXT_SMEMA_BUSY);
                    SetPcbOutDone(1);
                    //pThis->SetStep(ConveyorStep::START);
                }
            }
			else if (_time_elapsed(NextLoadingStartTime) > NextLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Work To Outlet) TimeOut:%d"), NextLoadingTimeOut);
				SendAlarm(CONV_WORK_TO_EXIT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To Outlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        case ConveyorStep::WAIT_PCBOUT:

			if (LatchOutOn == false && pThis->IsExistOut(Conv) == true)
			{
				LatchOutOn = true;
				if (UseSimulLoadingType == SIMULLOAD_OUTON)
				{
					pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
				}
				TRACE(_T("[PWR] WorkConveyor LatchOutOn true"));

			}

			if (LatchSetOff == false && pThis->GetStateSet(Conv, INOFF) == true)
			{
				TRACE(_T("[PWR] WorkConveyor LatchSetOff true"));

				LatchSetOff = true;
			}

			if (LatchSetOff == true) // 워크에서 나간거 확실히.
			{
				pThis->SetStep(ConveyorStep::WAIT_OUT_PCBOUT);
			}
			else if (_time_elapsed(NextLoadingStartTime) > NextLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Work To Outlet) TimeOut:%d"), NextLoadingTimeOut);
				SendAlarm(CONV_WORK_TO_EXIT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To Outlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
			break;

        case ConveyorStep::WAIT_OUT_PCBOUT:

			if (LatchOutOn == false && pThis->IsExistOut(Conv) == true)
			{
				LatchOutOn = true;
				if (UseSimulLoadingType == SIMULLOAD_OUTON)
				{
					pThis->PrevSmemaOut(Conv, SMEMA_NOT_BUSY);
				}
				TRACE(_T("[PWR] WorkConveyor LatchOutOn true"));

			}

			if (LatchLowOn == false && pThis->IsExistLow(Conv) == true)
			{
				LatchLowOn = true;
				pThis->BeltOn(BELT_SPEED_LOW);
				TRACE(_T("[PWR] WorkConveyor LatchLowOn true"));

				// 뒷놈이 벌써 온거 같은데 우선 슬로우
			}

			// 아웃 한번 켜졌고, 지금은 아웃, 워크 셋 둘다 오프라면 정상
			if (LatchOutOn == true && pThis->IsExistOut(Conv) == false && pThis->GetStateSet(Conv, INOFF) == true)
			{
				pThis->NextSmemaOut(Conv, SMEMA_NOT_READY);
				SetWorkPcbReady(FRONT_CONV, false); // WORK로 SIMUL PCB가 투입될 때
				SetInsertDone(0); // WORK로 SIMUL PCB가 투입될 때
				SetPcbOutDone(1);
                SetHeightMeasureDone(0);
				pThis->SetStep(ConveyorStep::UP_STOPPER);
			}
			else if (pThis->GetStateSet(Conv, INOFF) == false)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Simulloading Work To Outlet)"));
				SendAlarm(CONV_WORK_TO_EXIT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Simulloading Work To Outlet))"));
				pThis->SetStep(ConveyorStep::STOP);
			}
			else if (_time_elapsed(NextLoadingStartTime) > NextLoadingTimeOut)
			{
				pThis->BeltOff(PrevInBeltSpd);
				TRACE(_T("[PWR] Pcb Tranfer Error(Work To Outlet) TimeOut:%d"), NextLoadingTimeOut);
				SendAlarm(CONV_WORK_TO_EXIT_TIMEOUT(pThis->GetPos()), _T("Pcb Tranfer Error(Work To Outlet)"));
				pThis->SetStep(ConveyorStep::STOP);
			}
            break;

        case ConveyorStep::WAIT_NEXT_SMEMA_BUSY:
            if (pThis->CheckNextSmema(Conv) == SMEMA_BUSY)
            {
                pThis->BeltOff(NextOutBeltSpd);
                if (bLoop != false)
                {
                    //pThis->SetStep(ConveyorStep::START);
                    pThis->SetStep(ConveyorStep::WAIT_PREV_SMEMA);
                }
            }
            break;

        default:
            break;
        }
        ThreadSleep(THREAD_WORKCONV_READTIME);
    };
	pThis->BeltOff(NextOutBeltSpd);

    pThis->InitSmema(Conv);
    pThis->SetEnd(true);
    TRACE(_T("[PWR] CWorkConveyor(0x%X) LastStep(%d) Quit\n"), pThis->GetTaskID(), pThis->GetStep());
    return 0;
}

void CWorkConveyor::Run(long ProdMode)
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetProdRunMode(ProdMode);
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thWorkConveyor"));
    lpStartAddress = (_beginthreadex_proc_type)StartWorkConveyor;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    SetTaskID(nID);
    TRACE(_T("[PWR] Work Conveyor Thread ID:0x%04X(%s)\n"), GetTaskID(), GetThreadName());
    strLog.Format(_T("[PWR] Work Conveyor Thread ID:0x%04X(%s)"), GetTaskID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
    SetStep(ConveyorStep::STOP);
}


