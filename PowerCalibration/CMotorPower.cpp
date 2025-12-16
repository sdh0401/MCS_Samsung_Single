#include "pch.h"
#include "CMotorPower.h"
#include "GlobalIODefine.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "AxisInformation.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CPowerHMI.h"
//#include "ErrorCode.h"
#include "CPowerBuzzer.h"

CMotorPower* gcMotorPower;
CMotorPower::CMotorPower()
{
    m_ShowID = 0;
    m_MotorPowerStep = MotorPowerStep::INIT;
}

CMotorPower::~CMotorPower()
{
}

const char* CMotorPower::GetStepName(const MotorPowerStep& step)
{
    switch (step)
    {
    case MotorPowerStep::INIT:
        return "INIT";
    case MotorPowerStep::STOP:
        return "STOP";
    case MotorPowerStep::START:
        return "START";
    case MotorPowerStep::EMERGENCY_PUSHDN:
        return "EMERGENCY_PUSHDN";
    case MotorPowerStep::EMERGENCY_PUSHUP:
        return "EMERGENCY_PUSHUP";
    case MotorPowerStep::DOOR_KEY_OUT:
        return "DOOR_KEY_OUT";
    case MotorPowerStep::DOOR_KEY_IN:
        return "DOOR_KEY_IN";
    case MotorPowerStep::DOOR_UNLOCK:
        return "DOOR_UNLOCK";
    case MotorPowerStep::DOOR_LOCK:
        return "DOOR_LOCK";
    case MotorPowerStep::DOOR_LOCK_OFF:
        return "DOOR_LOCK_OFF";
    case MotorPowerStep::DOOR_LOCK_ON:
        return "DOOR_LOCK_ON";
    case MotorPowerStep::LOTO_KEY_OFF:
        return "LOTO_KEY_OFF";
    case MotorPowerStep::LOTO_KEY_ON:
        return "LOTO_KEY_ON";
    case MotorPowerStep::TEMP_HIGH_X:
        return "TEMP_HIGH_X";
    case MotorPowerStep::TEMP_HIGH_Y1:
        return "TEMP_HIGH_Y1";
    case MotorPowerStep::TEMP_HIGH_Y2:
        return "TEMP_HIGH_Y2";
    case MotorPowerStep::MOTOR_POWER_ON:
        return "MOTOR_POWER_ON";
    case MotorPowerStep::WAIT_COMM:
        return "WAIT_COMM";
    case MotorPowerStep::SEND_HMI:
        return "SEND_HMI";
    case MotorPowerStep::WAIT_RESET_KEY:
        return "WAIT_RESET_KEY";
    case MotorPowerStep::WAIT_POWER_ON:
        return "WAIT_POWER_ON";
    case MotorPowerStep::READY_MACHINE:
        return "READY_MACHINE";
    case MotorPowerStep::IDLE:
        return "IDLE";
    case MotorPowerStep::SELFQUIT:
        return "SELFQUIT";
    case MotorPowerStep::QUIT:
        return "QUIT";
    case MotorPowerStep::TTF_DOOR_LOCK_POWER_ON:
        return "TTF_DOOR_LOCK_POWER_ON";
    case MotorPowerStep::TTF_RETRY_SERVON:
        return "TTF_RETRY_SERVON";
    case MotorPowerStep::DETECT_AREASENSOR:
        return "DETECT_AREASENSOR";
    default:
        return "**UNDEFINED";
    }
}

MotorPowerStep CMotorPower::GetStep()
{
    return m_MotorPowerStep;
}

void CMotorPower::SetStep(MotorPowerStep MotorPowerStep)
{
    m_MotorPowerStep = MotorPowerStep;
}

UINT CMotorPower::MotorPowerOn(LPVOID wParam)
{
    CString strLog;
    long Err = ErrorCode::None;
    bool bLoop = true;
    CMotorPower* pThis = reinterpret_cast<CMotorPower*>(wParam);
    MotorPowerStep OldStep = MotorPowerStep::STOP;
    MotorPowerStep ErrorStep = MotorPowerStep::STOP;
    //CApplicationTime* pTime = new CApplicationTime();
    ULONGLONG GetTime = 0, Elapsed = 0;
    long TimeChk = 0;
    bool bFirst = true, bPushEmg = true;
    bool bFirstResetButton = true;
	bool bError = false;
	bool exeDoorLock = false;

	bool emgPush = false;
	bool lotoKeyOn = false;
	bool safetyOff = false;
	bool doorUnlock = false;

    //pThis->SetMsgQueueStatus(THREAD_RUN);
    GetTime = _time_get();
    SendEmergencyStatus(1);
	SendPopupClose();

    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_MOTOR_READTIME) == true)
        {
            DoorLockingControl(false);
            MotorPowerControl(false);
            ThreadSleep(TIME1000MS);
            TRACE(_T("[PWR] CMotorPower(0x%x) Terminated\n"), pThis->m_ShowID);
            strLog.Format(_T("[PWR] CMotorPower(0x%x) Terminated\n"), pThis->m_ShowID);
            gcPowerLog->Logging(strLog);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_MOTOR_READTIME);
        //    continue;
        //}
        if (OldStep != pThis->GetStep())
        {
            Elapsed = _time_elapsed(GetTime);

			if (ErrorStep == MotorPowerStep::STOP)
			{
				//TRACE(_T("[PWR] CMotorPower Step:%d Time:%d[ms]\n"), pThis->GetStep(), Elapsed);
                CString temp; temp.Format(L"[%s](%d)->[%s](%d) %llu(ms)", (LPCTSTR)CString(CMotorPower::GetStepName(OldStep)), static_cast<int>(OldStep), (LPCTSTR)CString(CMotorPower::GetStepName(pThis->GetStep())), static_cast<int>(pThis->GetStep()), Elapsed);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
			}
            OldStep = pThis->GetStep();
            GetTime = _time_get();
        }
        switch (pThis->GetStep())
        {
        case MotorPowerStep::INIT:
            break;
        case MotorPowerStep::STOP:
            DoorLockingControl(false);
            MotorPowerControl(true);
            ThreadSleep(TIME5000MS);
            pThis->SetStep(MotorPowerStep::START);
            break;
        case MotorPowerStep::START:
            pThis->SetStep(MotorPowerStep::EMERGENCY_PUSHDN);
            break;
        case MotorPowerStep::EMERGENCY_PUSHDN:
            if (InputTimeOne(IN_FEMERGENCY, INOFF, TIME100MS) == true && InputTimeOne(IN_REMERGENCY, INOFF, TIME100MS) == true)
            {
                pThis->SetStep(MotorPowerStep::EMERGENCY_PUSHUP);
            }
            else
            {
                if (bPushEmg == true)
                {
                    SendAlarm(PUSH_EMERGENCY, _T("Emergency Emergency Emergency"));
                    //TowerLampAlarm(PUSH_EMERGENCY);
                    //BuzzerOn(0);
                    SetGlobalStatusError(true);
                }

				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Emergency"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            break;
        case MotorPowerStep::EMERGENCY_PUSHUP:
            if (InputTimeOne(IN_FEMERGENCY, INON, TIME100MS) == true || InputTimeOne(IN_REMERGENCY, INON, TIME100MS) == true)
            {
				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Emergency"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            else
            {
                bPushEmg = false;
                //BuzzerOff();
                SetGlobalStatusError(false);
                pThis->SetStep(MotorPowerStep::DOOR_KEY_OUT);
            }
            break;
        case MotorPowerStep::DOOR_KEY_OUT:
            if (InputTimeOne(IN_FDOOR_KEY_OUT, INOFF, TIME100MS) == true && InputTimeOne(IN_RDOOR_KEY_OUT, INOFF, TIME100MS) == true)
            {
                pThis->SetStep(MotorPowerStep::DOOR_KEY_IN);
            }
            else
            {
                TowerLampAlarm(OPEN_DOOR);
				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Door Open"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            break;
        case MotorPowerStep::DOOR_KEY_IN:
            if (InputOne(IN_FDOOR_KEY_OUT) == INON || InputOne(IN_RDOOR_KEY_OUT) == INON)
            {
				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Door Open"));
				}
				ErrorStep = pThis->GetStep(); 
				pThis->SetStep(MotorPowerStep::START);
                g_bUserDoorClose = false;
            }
            else
            {
				exeDoorLock = false;
                g_bUserDoorClose = true;
                if (GetMachineState() == STATE_INIT)
                {
                    DoorLockingControl(true);
					exeDoorLock = true;
                }
                else if (GetMachineState() == STATE_IDLE && GetRunModeNoLog() == NORMAL_MODE)
                {
                    if (g_bUserDoorPush == true)
                    {
                        g_UserOpenDoorElapsed = _time_elapsed(g_UserOpenDoorGetTime);
                        if (g_UserOpenDoorElapsed > TIME1000MS * 10) 
                        {
                            DoorLockingControl(true);
                            g_bUserDoorPush = false;
							exeDoorLock = true;
                        }
                    }
                    else
                    {
                        DoorLockingControl(true);
						exeDoorLock = true;
                    }
                }

				if (exeDoorLock == true)
				{
					ThreadSleep(TIME1000MS);
					pThis->SetStep(MotorPowerStep::DOOR_UNLOCK);
				}
            }
            break;
        case MotorPowerStep::DOOR_UNLOCK:
            if (InputTimeOne(IN_FDOOR_KEY_UNLOCK, INOFF, TIME100MS) == true && InputTimeOne(IN_RDOOR_KEY_UNLOCK, INOFF, TIME100MS) == true)
            {
                pThis->SetStep(MotorPowerStep::DOOR_LOCK);
            }
            else
            {
				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Door Lock"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            break;
        case MotorPowerStep::DOOR_LOCK:
            if (InputOne(IN_FDOOR_KEY_UNLOCK) == INON || InputOne(IN_RDOOR_KEY_UNLOCK) == INON)
            {
				if (ErrorStep != pThis->GetStep())
				{
					SendPopupMessage(_T("Check Door Lock"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            else
            {
                pThis->SetStep(MotorPowerStep::LOTO_KEY_ON);
            }
            break;
        case MotorPowerStep::LOTO_KEY_ON:
            if (InputOne(IN_LOTO_KEY_ON) == INOFF)
            {
                pThis->SetStep(MotorPowerStep::LOTO_KEY_OFF);
            }
            else
            {
				if (ErrorStep != pThis->GetStep())
				{
					//SendPopupMessage(_T("Check LOTO Key"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            break;
        case MotorPowerStep::LOTO_KEY_OFF:
            if (InputOne(IN_LOTO_KEY_ON) == INON)
            {
				if (ErrorStep != pThis->GetStep())
				{
					//SendPopupMessage(_T("Check LOTO Key"));
				}
				ErrorStep = pThis->GetStep();
                pThis->SetStep(MotorPowerStep::START);
            }
            else
            {
				if (GetOnlyConveyorMode() == true)
				{
					pThis->SetStep(MotorPowerStep::READY_MACHINE);
				}
				else
				{
					pThis->SetStep(MotorPowerStep::TEMP_HIGH_X);
				}
            }
            break;            
        case MotorPowerStep::TEMP_HIGH_X:
            if (GetUseRTDSensorFX() == 1)
            {
                if (GetTemperature(_T("FX")) > HIGH_TEMPERATURE)
                {
					if (ErrorStep != pThis->GetStep())
					{
						TRACE(_T("[PWR] GetTemperature FX:%.1f\n"), GetTemperature(GetAxisX(FRONT_GANTRY)));

						SendPopupMessage(_T("Check Temperature FX"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::TEMP_HIGH_Y1);
                }
            }
            else
            {
                if (InputOne(IN_FX_LMTEMP_LOW) == INOFF)
                {
					if (ErrorStep != pThis->GetStep())
					{
						SendPopupMessage(_T("Check Temperature FX"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::TEMP_HIGH_Y1);
                }
            }
            break;
        case MotorPowerStep::TEMP_HIGH_Y1:
            if (GetUseRTDSensorFY1() == 1)
            {
                if (GetTemperature(_T("FY1")) > HIGH_TEMPERATURE)
                {
					if (ErrorStep != pThis->GetStep())
					{
						TRACE(_T("[PWR] GetTemperature FY1:%.1f\n"), GetTemperature(GetAxisY1(FRONT_GANTRY)));

						SendPopupMessage(_T("Check Temperature FY1"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::TEMP_HIGH_Y2);
                }
            }
            else
            {
                if (InputOne(IN_FY1_LMTEMP_LOW) == INOFF)
                {
					if (ErrorStep != pThis->GetStep())
					{
						SendPopupMessage(_T("Check Temperature FY1"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::TEMP_HIGH_Y2);
                }
            }
            break;
        case MotorPowerStep::TEMP_HIGH_Y2:
            if (GetUseRTDSensorFY2() == 1)
            {
                if (GetTemperature(_T("FY2")) > HIGH_TEMPERATURE)
                {
					if (ErrorStep != pThis->GetStep())
					{
						TRACE(_T("[PWR] GetTemperature FY2:%.1f\n"), GetTemperature(GetAxisY2(FRONT_GANTRY)));

						SendPopupMessage(_T("Check Temperature FY2"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::MOTOR_POWER_ON);
                }
            }
            else
            {
                if (InputOne(IN_FY2_LMTEMP_LOW) == INOFF)
                {
					if (ErrorStep != pThis->GetStep())
					{
						SendPopupMessage(_T("Check Temperature FY2"));
					}
					ErrorStep = pThis->GetStep();
                    pThis->SetStep(MotorPowerStep::START);
                }
                else
                {
                    pThis->SetStep(MotorPowerStep::MOTOR_POWER_ON);
                }
            }
            break;
        //case MotorPowerStep::MOTOR_POWER_OFF:
        //    if (InputOne(IN_SAFETY_ON) == INOFF)
        //    {
        //        pThis->SetStep(MotorPowerStep::MOTOR_POWER_ON);
        //    }
        //    else
        //    {
        //        pThis->SetStep(MotorPowerStep::START);
        //    }
        //    break;

        case MotorPowerStep::MOTOR_POWER_ON:
			ErrorStep = MotorPowerStep::STOP;

            if (bFirst == true)
            {
				SendPopupClose();

				//if (ReadOutputOne(OUT_MOTOR_POWER) == OUTON && GetMotorOffDoorOpen() == false)
				if (ReadOutputOne(OUT_MOTOR_POWER) == OUTON)
				{
					TRACE(_T("[PWR] Skip Charging wait.\n"));
				}
				else
				{
					MotorPowerControl(true);
					ThreadSleep(TIME1000MS);
					for (long indx = 0; indx < 5; ++indx)
					{
						if (InputOne(IN_SAFETY_ON) == INON)
						{
							TRACE(_T("[PWR] Skip Wating Power Charging...(%d)\n"), indx + 1);
							strLog.Format(_T("[PWR] Skip Wating Power Charging...(%d)\n"), indx + 1);
							gcPowerLog->Logging(strLog);
							break;
						}
						ThreadSleep(TIME1000MS + TIME200MS);
						TRACE(_T("[PWR] Please Wating Power Charging...(%d)\n"), indx + 1);
						strLog.Format(_T("[PWR] Please Wating Power Charging...(%d)\n"), indx + 1);
						gcPowerLog->Logging(strLog);
					}
				}

                SendRequestResetButton();
                bFirst = false;
                bFirstResetButton = true;
            }
            pThis->SetStep(MotorPowerStep::WAIT_COMM);
            break;
        case MotorPowerStep::WAIT_COMM:
            pThis->SetStep(MotorPowerStep::SEND_HMI);
            break;
        case MotorPowerStep::SEND_HMI:
            pThis->SetStep(MotorPowerStep::WAIT_RESET_KEY);
            break;
        case MotorPowerStep::WAIT_RESET_KEY:
            //if (InputTimeOne(IN_RESET_PANEL_KEY, INON, TIME10MS) == true)
            {
                pThis->SetStep(MotorPowerStep::WAIT_POWER_ON);
            }
            break;
        case MotorPowerStep::WAIT_POWER_ON:
            OutputOne(OUT_RESET_PANEL_LED, OUTON);
            if (InputOne(IN_SAFETY_ON) == INON)
            {
                ThreadSleep(TIME1000MS);
                OutputOne(OUT_RESET_PANEL_LED, OUTOFF);
                pThis->SetStep(MotorPowerStep::READY_MACHINE);
                if (GetMachineState() == STATE_IDLE && GetRunModeNoLog() == NORMAL_MODE)
                {
                    ThreadSleep(TIME1000MS);
                    gServoAllOn();
                    SetGlobalStatusError(false);
                    (void)gcPowerBuzzer->SetLastSafetyAlarmCode(NO_ERR);
                    TowerLampNormal();
                    BuzzerOff();
                }
            }
            else
            {
                bFirst = true;
                pThis->SetStep(MotorPowerStep::START);
            }
            break;

        case MotorPowerStep::READY_MACHINE:

			if (GetOnlyConveyorMode() == true)
			{
				if (ErrorStep != MotorPowerStep::STOP)
				{
					SendPopupClose();
					ErrorStep = MotorPowerStep::STOP;

				}
				gServoAllOn();
				bFirstResetButton = true;
			}

            if (bFirstResetButton == true)
            {
                SendResetDone();
                ThreadSleep(TIME2000MS);
                bFirstResetButton = false;
            }
            pThis->SetStep(MotorPowerStep::IDLE);
            break;

        case MotorPowerStep::IDLE:
			emgPush = false;
			lotoKeyOn = false;
			safetyOff = false;
			doorUnlock = false;

			if (InputOne(IN_FEMERGENCY) == INON || InputOne(IN_REMERGENCY) == INON)
			{
				emgPush = true;
			}

			if (InputOne(IN_LOTO_KEY_ON) == INON)
			{
				lotoKeyOn = true;
			}

			if (InputOne(IN_SAFETY_ON) == INOFF && GetOnlyConveyorMode() == false)
			{
				safetyOff = true;
			}

			if (ReadOutputOne(OUT_FDOOR_KEY_LOCK) == INOFF || ReadOutputOne(OUT_RDOOR_KEY_LOCK) == INOFF
				|| InputOne(IN_FDOOR_KEY_UNLOCK) == INON || InputOne(IN_RDOOR_KEY_UNLOCK) == INON)
			{
				doorUnlock = true;
			}

			if (emgPush == true)
			{
				bFirst = true;
				gServoAllOff();
				//MotorPowerControl(false);
				DoorLockingControl(false);
				pThis->SetStep(MotorPowerStep::START);
				TRACE(_T("[PWR] CMotorPower Emergency push\n"));
			}
			else if (lotoKeyOn == true)
			{
				bFirst = true;
				gServoAllOff();
				//MotorPowerControl(false);
				DoorLockingControl(false);
				pThis->SetStep(MotorPowerStep::START);
				TRACE(_T("[PWR] CMotorPower Lotokey On\n"));

			}
			else if (doorUnlock == true)
			{
				bFirst = true;
				gServoAllOffWithoutConv();
				DoorLockingControl(false);
				pThis->SetStep(MotorPowerStep::START);
				TRACE(_T("[PWR] CMotorPower Door unlock\n"));

			}
			else if (safetyOff == true && GetOnlyConveyorMode() == false)
			{
				bFirst = true;
				gServoAllOffWithoutConv();
				//MotorPowerControl(false);
				DoorLockingControl(false);
				pThis->SetStep(MotorPowerStep::START);
				TRACE(_T("[PWR] CMotorPower Safety relay Off\n"));

			}
            break;

        case MotorPowerStep::SELFQUIT:
            bLoop = false;
            break;

        case MotorPowerStep::QUIT:
            break;
        }
        if (bLoop == false)
        {
            break;
        }
        ThreadSleep(THREAD_MOTOR_READTIME);
    }
    DoorLockingControl(false);
    MotorPowerControl(false);
    TRACE(_T("[PWR] CMotorPower(0x%x) Quit\n"), pThis->m_ShowID);
    strLog.Format(_T("[PWR] CMotorPower(0x%x) Quit\n"), pThis->m_ShowID);
    gcPowerLog->Logging(strLog);
    //delete pTime;
    return 0;
}

void CMotorPower::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thMotorPowerOn"));
    SetRunning(true);
    lpStartAddress = (_beginthreadex_proc_type)MotorPowerOn;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] MotorPower IO Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] MotorPower IO Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

long CMotorPower::WaitMotorPowerOn(long TimeOut)
{
    CApplicationTime* pTime = new CApplicationTime();
    long Err = NO_ERR;
    CString strLog;
    while (1)
    {
        ThreadSleep(TIME100MS);
        if (IsTerminated(THREAD_MOTOR_READTIME) == true)
        {
            TRACE(_T("[PWR] WaitMotorPowerOn Terminated\n"));
            break;
        }
        if (GetStep() == MotorPowerStep::READY_MACHINE)
        {            
            //SetStep(MotorPowerStep::IDLE);
            break;
        }
        if (pTime->TimeElapsed() > TimeOut)
        {
            TRACE(_T("[PWR] WaitMotorPowerOn TimeOut Elapsed:%d\n"), pTime->TimeElapsed());
            strLog.Format(_T("[PWR] WaitMotorPowerOn TimeOut Elapsed:%d[ms]"), pTime->TimeElapsed());
            gcPowerLog->Logging(strLog);
            break;
        }
    }
    TRACE(_T("[PWR] WaitMotorPowerOn Elapsed:%d\n"), pTime->TimeElapsed());
    return Err;
}

