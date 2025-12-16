#include "pch.h"
#include "CMachineInformation.h"
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
#include <deque>
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"
#include "CSqlite3File.h"
#include "CTowerLamp.h"

using namespace std;
CMachineInformation* gcMachineInformation;
CMachineInformation::CMachineInformation()
{
    m_ShowID = 0;
	//m_BowlFeederOffTime = TIME1MINUTE;
	ZeroMemory(&m_BowlFeederOff, sizeof(m_BowlFeederOff));
}

CMachineInformation::~CMachineInformation()
{
}

UINT CMachineInformation::MonitoringMachineInformation(LPVOID wParam)
{
    CString strLog, strSendMsg, strSendMsgOld;
    long Err = ErrorCode::None, ReadyIONo = 0, ReadyIO = 0, Length = 0;
    bool bLoop = true, bReady = false;
    CMachineInformation* pThis = reinterpret_cast<CMachineInformation*>(wParam);
    ULONGLONG GetTime = 0, Elapsed = 0;
    long AxisNo = 0, ServoOnAxis = 0;
    GetTime = _time_get();
    bool bEmg = false, bEmgOld = false;
    bool bFrontDoorOpen = false, bFrontDoorOpenOld = false;
    bool bRearDoorOpen = false, bRearDoorOpenOld = false;
    bool bLotoKeyOn = false, bLotoKeyOnOld = false;
    long bHighFX = 1, bHighFXOld = 2;
    long bHighFY1 = 1, bHighFY1Old = 2;
    long bHighFY2 = 1, bHighFY2Old = 2;
    long bHighRX = 1, bHighRXOld = 2;
    long bHighRY1 = 1, bHighRY1Old = 2;
    long bHighRY2 = 1, bHighRY2Old = 2;
    bool bServoOn = false, bServoOnOld = false;
    bool bExist = false;
    long bEntry = 0, bEntryOld = 2;
    long bWork = 0, bWorkOld = 2;
    long bExit = 0, bExitOld = 2;
    bool bSmemaPrevIn = false, bSmemaPrevOut = false, bSmemaPrevInOld = false, bSmemaPrevOutOld = false;
    bool bSmemaNextIn = false, bSmemaNextOut = false, bSmemaNextInOld = false, bSmemaNextOutOld = false;
    bool bFrAncLockIn = false, bFrAncUnlockIn = false, bFrAncLockInOld = false, bFrAncUnlockInOld = false;
	std::deque<IO_MONITORING> InputStatusCurrnet;
	std::deque<IO_MONITORING> InputStatusOld;
	std::deque<IO_MONITORING> OutputStatusCurrnet;
	std::deque<IO_MONITORING> OutputStatusOld;
	IO_MONITORING init;

	init.No = IN_FANC_BASE_UP;			init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FANC_BASE_DN;			init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FORMING_1ST_LOCK;		init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FORMING_1ST_UNLOCK;	init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FORMING_2ND_LOCK;		init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FORMING_2ND_UNLOCK;	init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);
	init.No = IN_FORMING_EXIST;			init.Status = false;	InputStatusCurrnet.push_back(init);	InputStatusOld.push_back(init);

	init.No = OUT_FHEAD1_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD2_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD3_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD4_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD5_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD6_SUC;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD1_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD2_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD3_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD4_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD5_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FHEAD6_BLO;			init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FORMING_1ST_LOCK;		init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);
	init.No = OUT_FORMING_2ND_UNLOCK;	init.Status = false;	OutputStatusCurrnet.push_back(init);	OutputStatusOld.push_back(init);

	bool homingComplete = false;
	pThis->InitialMotorAlarmHistory();
	
	std::deque<bool> ServoOnOld(MAXGANTRYAXISNO, false);
	long ServoOffAxisCount = 0;

    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_MACHINE_MONITORINGTIME) == true)
        {
            TRACE(_T("[PWR] CMachineInformation(0x%x) Terminated\n"), pThis->m_ShowID);
            strLog.Format(_T("[PWR] CMachineInformation(0x%x) Terminated\n"), pThis->m_ShowID);
            gcPowerLog->Logging(strLog);
            break;
        }

        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_MACHINE_MONITORINGTIME);
        //    continue;
        //}

		if (homingComplete == true || IsAllAxisHomingComplete() == true)
		{
			homingComplete = true;
			pThis->SaveAlarmCode();
		}


		if (InputStatusCurrnet.size() == InputStatusOld.size())
		{
			long NoCurr, NoOld;
			for (long idx = 0; idx < InputStatusCurrnet.size(); idx++)
			{
				NoCurr = InputStatusCurrnet.at(idx).No;
				NoOld = InputStatusOld.at(idx).No;

				if (NoCurr == NoOld)
				{

					InputStatusCurrnet.at(idx).Status = InputElapsedTimeOne(NoCurr, INON, TIME50MS);

					if (InputStatusCurrnet.at(idx).Status != InputStatusOld.at(idx).Status)
					{
						if (InputStatusCurrnet.at(idx).Status == true)
						{
							strSendMsg.Format(_T("%d,%d,%d,0"), NO_ERR, NoCurr, INON);
						}
						else
						{
							strSendMsg.Format(_T("%d,%d,%d,0"), NO_ERR, NoCurr, INOFF);
						}

						SendIOStatus(strSendMsg);
						InputStatusOld.at(idx).Status = InputStatusCurrnet.at(idx).Status;
					}
				}
			}
		}

		if (OutputStatusCurrnet.size() == OutputStatusOld.size())
		{
			long NoCurr, NoOld;
			for (long idx = 0; idx < OutputStatusCurrnet.size(); idx++)
			{
				NoCurr = OutputStatusCurrnet.at(idx).No;
				NoOld = OutputStatusOld.at(idx).No;

				if (NoCurr == NoOld)
				{
					OutputStatusCurrnet.at(idx).Status = ReadOutputOne(NoCurr);

					if (OutputStatusCurrnet.at(idx).Status != OutputStatusOld.at(idx).Status)
					{
						if (OutputStatusCurrnet.at(idx).Status == OUTON)
						{
							strSendMsg.Format(_T("%d,%d,%d,0"), NO_ERR, NoCurr, OUTON);
						}
						else
						{
							strSendMsg.Format(_T("%d,%d,%d,0"), NO_ERR, NoCurr, INOFF);
						}

						SendIOStatus(strSendMsg);
						OutputStatusOld.at(idx).Status = OutputStatusCurrnet.at(idx).Status;
					}
				}
			}
		}

        if (InputElapsedTimeOne(IN_FEMERGENCY, INOFF, TIME100MS) == true && InputElapsedTimeOne(IN_REMERGENCY, INOFF, TIME100MS) == true)
        {
            bEmg = false;
        }
        else
        {
            bEmg = true;
        }

		if (InputElapsedTimeOne(IN_LOTO_KEY_ON, INOFF, TIME100MS) == true)
		{
			bLotoKeyOn = false;
		}
		else
		{
			bLotoKeyOn = true;
		}

		if (bEmgOld == true || bLotoKeyOnOld == true)
		{
			if (bEmg == false && bLotoKeyOn == false)
			{
				TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
				TRACE(_T("[PWR] !!!!! Emergency & LotoKey Clear!!!!!\n"));
				TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));

				//BuzzerOff();
			}
		}

        if (bEmgOld != bEmg)
        {
            bEmgOld = bEmg;
            if (bEmg == true)
            {
                TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!\n"));
                TRACE(_T("[PWR] !!!!! Emergency !!!!!\n"));
                TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!\n"));
                SendEmergencyStatus(0);
            }
            else
            {
                TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
                TRACE(_T("[PWR] !!!!! Release Emergency !!!!!\n"));
                TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
                SendEmergencyStatus(1);
            }
        }

		//if (bEmg == false)
		{
			if (bLotoKeyOnOld != bLotoKeyOn)
			{
				bLotoKeyOnOld = bLotoKeyOn;
				if (bLotoKeyOn == true)
				{
					TRACE(_T("[PWR] ###########################\n"));
					TRACE(_T("[PWR] ##### LOTO Key   Lock #####\n"));
					TRACE(_T("[PWR] ###########################\n"));
					SendLotoKeyStatus(LOTO_KEY_LOCK); // Lock
				}
				else
				{
					TRACE(_T("[PWR] ###########################\n"));
					TRACE(_T("[PWR] ##### LOTO Key UnLock #####\n"));
					TRACE(_T("[PWR] ###########################\n"));
					SendLotoKeyStatus(LOTO_KEY_UNLOCK); // Unlock
				}
			}
		}

		if (bEmg == true || bLotoKeyOn == true)
		{
			if (ReadOutputOne(OUT_FBUZZER) == OUTOFF)
			{
				//BuzzerOn(1000000);
			}

			gcTowerLamp->Alarm();
		}

        if (InputElapsedTimeOne(IN_FDOOR_KEY_OUT, INOFF, TIME100MS) == true )
        {
            bFrontDoorOpen = false;
        }
        else
        {
            bFrontDoorOpen = true;
        }
        if (InputElapsedTimeOne(IN_RDOOR_KEY_OUT, INOFF, TIME100MS) == true)
        {
            bRearDoorOpen = false;
        }
        else
        {
            bRearDoorOpen = true;
        }
        if (bFrontDoorOpenOld != bFrontDoorOpen)
        {
            bFrontDoorOpenOld = bFrontDoorOpen;
            if (bFrontDoorOpen == true)
            {
                TRACE(_T("[PWR] ||||||||||||||||||||||||||||\n"));
                TRACE(_T("[PWR] ||||| Front Open  Door |||||\n"));
                TRACE(_T("[PWR] ||||||||||||||||||||||||||||\n"));
                SendDoorStatus(0, FRONT_GANTRY); // Open
            }
            else
            {
                TRACE(_T("[PWR] ----------------------------\n"));
                TRACE(_T("[PWR] ----- Front Close Door -----\n"));
                TRACE(_T("[PWR] ----------------------------\n"));
                SendDoorStatus(1, FRONT_GANTRY); // Close
            }
        }
        if (bRearDoorOpenOld != bRearDoorOpen)
        {
            bRearDoorOpenOld = bRearDoorOpen;
            if (bRearDoorOpen == true)
            {
                TRACE(_T("[PWR] ||||||||||||||||||||||||||||\n"));
                TRACE(_T("[PWR] ||||| Rear Open  Door |||||\n"));
                TRACE(_T("[PWR] ||||||||||||||||||||||||||||\n"));
                SendDoorStatus(0, REAR_GANTRY); // Open
            }
            else
            {
                TRACE(_T("[PWR] ----------------------------\n"));
                TRACE(_T("[PWR] ----- Rear Close Door -----\n"));
                TRACE(_T("[PWR] ----------------------------\n"));
                SendDoorStatus(1, REAR_GANTRY); // Close
            }
        }

        if (GetUseRTDSensorFX() == 0)
        {
            if (InputOne(IN_FX_LMTEMP_LOW) == INOFF)
            {
                bHighFX = 1;
            }
            else
            {
                bHighFX = 0;
            }
        }
        else 
        {
            if (GetTemperature(_T("FX")) > HIGH_TEMPERATURE)
            {
                bHighFX = 1;
            }
            else
            {
                bHighFX = 0;
            }
        }
        if (bHighFXOld != bHighFX)
        {
            bHighFXOld = bHighFX;
            strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d"), bHighFX, bHighFY1, bHighFY2, bHighRX, bHighRY1, bHighRY2);
            SendGantryTempStatus(strSendMsg);
            ThreadSleep(TIME5MS);
        }
        if (GetUseRTDSensorFY1() == 0)
        {
            if (InputOne(IN_FY1_LMTEMP_LOW) == INOFF)
            {
                bHighFY1 = 1;
            }
            else
            {
                bHighFY1 = 0;
            }
        }
        else
        {
            if (GetTemperature(_T("FY1")) > HIGH_TEMPERATURE)
            {
                bHighFY1 = 1;
            }
            else
            {
                bHighFY1 = 0;
            }
        }
        if (bHighFY1Old != bHighFY1)
        {
            bHighFY1Old = bHighFY1;
            strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d"), bHighFX, bHighFY1, bHighFY2, bHighRX, bHighRY1, bHighRY2);
            SendGantryTempStatus(strSendMsg);
            ThreadSleep(TIME5MS);
        }
        if (GetUseRTDSensorFY2() == 0)
        {
            if (InputOne(IN_FY2_LMTEMP_LOW) == INOFF)
            {
                bHighFY2 = 1;
            }
            else
            {
                bHighFY2 = 0;
            }
        }
        else
        {
            if (GetTemperature(_T("FY2")) > HIGH_TEMPERATURE)
            {
                bHighFY2 = 1;
            }
            else
            {
                bHighFY2 = 0;
            }
        }
        if (bHighFY2Old != bHighFY2)
        {
            bHighFY2Old = bHighFY2;
            strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d"), bHighFX, bHighFY1, bHighFY2, bHighRX, bHighRY1, bHighRY2);
            SendGantryTempStatus(strSendMsg);
            ThreadSleep(TIME5MS);
        }
        strSendMsg.Empty();
        AxisNo = 0, ServoOnAxis = 0;
        for (AxisNo = 0; AxisNo < MAXGANTRYAXISNO; ++AxisNo)
        {
            if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
            {
                ServoOnAxis++;
                continue;
            }
            if(CheckServoOn(PowerAxisAliasName[AxisNo]) == false)
            {
                break;
            }
            else
            {
                ServoOnAxis++;
            }
        }
        if (ServoOnAxis == 0)
        {
            bServoOn = false;
        }
        else
        {
            if (ServoOnAxis == MAXGANTRYAXISNO)
            {
                bServoOn = true;
            }
            else
            {
                bServoOn = false;
            }
        }
        if (bServoOnOld != bServoOn)
        {
            bServoOnOld = bServoOn;
            if (bServoOn == true)
            {
                //TRACE(_T("[PWR] $$$$$$$$$$$$$$$$$$$$$$$$$$$\n"));
                //TRACE(_T("[PWR] $$$$$$ All Servo  ON $$$$$$\n"));
                //TRACE(_T("[PWR] $$$$$$$$$$$$$$$$$$$$$$$$$$$\n"));
                SendServoStatus(ALLSERVO_ON); // On
            }
            else
            {
                //TRACE(_T("[PWR] $$$$$$$$$$$$$$$$$$$$$$$$$$$\n"));
                //TRACE(_T("[PWR] $$$$$$ All Servo OFF $$$$$$\n"));
                //TRACE(_T("[PWR] $$$$$$$$$$$$$$$$$$$$$$$$$$$\n"));
                SendServoStatus(ALLSERVO_OFF); // Off
            }
        }

		for (AxisNo = 0; AxisNo < ServoOnOld.size(); AxisNo++)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				continue;
			}

			bServoOn = CheckServoOn(PowerAxisAliasName[AxisNo]);
			if (bServoOn != ServoOnOld.at(AxisNo))
			{
				ServoOnOld.at(AxisNo) = bServoOn;
				if (bServoOn == true)
				{
					TRACE(_T("[PWR] Servo status OFF -> ON (%s)\n"), GetAxisNameByAxisIndex(AxisNo));
				}
				else
				{
					TRACE(_T("[PWR] Servo status ON -> OFF (%s)\n"), GetAxisNameByAxisIndex(AxisNo));
				}
			}
		}

        bExist = IsExistAll(ENTRY_CONV);
        if (bExist == true) bEntry = 1;
        else bEntry = 0;
        bExist = IsExistAll(WORK1_CONV);
        if (bExist == true) bWork = 1;
        else bWork = 0;
        bExist = IsExistAll(EXIT_CONV);
        if (bExist == true) bExit = 1;
        else bExit = 0;
        if (bEntryOld != bEntry)
        {
            bEntryOld = bEntry;
            strSendMsg.Format(_T("%d,%d,%d"), bEntry, bWork, bExit);
            SendPcbSensorStatus(strSendMsg);
        }
        if (bWorkOld != bWork)
        {
            bWorkOld = bWork;
            strSendMsg.Format(_T("%d,%d,%d"), bEntry, bWork, bExit);
            SendPcbSensorStatus(strSendMsg);
        }
        if (bExitOld != bExit)
        {
            bExitOld = bExit;
            strSendMsg.Format(_T("%d,%d,%d"), bEntry, bWork, bExit);
            SendPcbSensorStatus(strSendMsg);
        }

        strSendMsg.Empty();
        for (ReadyIONo = 0; ReadyIONo < MAX_READYIO_NO; ++ReadyIONo)
        {
            bReady = false;
            ReadyIO = GetReadyIONoFromReadyNo(ReadyIONo + 1);
            if (ReadyIO != IO_NOUSE)
            {
                bReady = InputTimeOne(ReadyIO, INON, TIME5MS);
            }
            if (bReady == true)
            {
                strSendMsg.AppendFormat(_T("%d,"), INON);
            }
            else
            {
                strSendMsg.AppendFormat(_T("%d,"), INOFF);
            }
        }
        for (ReadyIONo = MAX_FEEDER_READY_NO; ReadyIONo < MAX_FEEDER_READY_NO + MAX_READYIO_NO; ++ReadyIONo)
        {
            bReady = false;
            ReadyIO = GetReadyIONoFromReadyNo(ReadyIONo + 1);
            if (ReadyIO != IO_NOUSE)
            {
                bReady = InputElapsedTimeOne(ReadyIO, INON, TIME5MS);
            }
            if (bReady == true)
            {
                strSendMsg.AppendFormat(_T("%d,"), INON);
            }
            else
            {
                strSendMsg.AppendFormat(_T("%d,"), INOFF);
            }
        }
        if (strSendMsgOld.CompareNoCase(strSendMsg) == 0)
        {
            strSendMsg.Empty();
        }
        else
        {
            strSendMsgOld = strSendMsg;
            SendReadyIOStatus(strSendMsg);
            strSendMsg.Empty();
        }
        bSmemaPrevIn = InputElapsedTimeOne(IN_FCONV_PREV_IN, INON, TIME5MS);
        if (bSmemaPrevInOld != bSmemaPrevIn)
        {
            bSmemaPrevInOld = bSmemaPrevIn;
            if (bSmemaPrevIn == true)
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, IN_FCONV_PREV_IN, INON);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, IN_FCONV_PREV_IN, INOFF);
            }
            SendIOStatus(strSendMsg);
        }
        bSmemaPrevOut = ReadOutputTimeOne(OUT_FCONV_PREV_OUT, OUTON, TIME5MS);
        if (bSmemaPrevOutOld != bSmemaPrevOut)
        {
            bSmemaPrevOutOld = bSmemaPrevOut;
            if (bSmemaPrevOut == true)
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, OUT_FCONV_PREV_OUT, OUTON);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, OUT_FCONV_PREV_OUT, OUTOFF);
            }
            SendIOStatus(strSendMsg);
        }
        bSmemaNextIn = InputElapsedTimeOne(IN_FCONV_NEXT_IN, INON, TIME5MS);
        if (bSmemaNextInOld != bSmemaNextIn)
        {
            bSmemaNextInOld = bSmemaNextIn;
            if (bSmemaNextIn == true)
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, IN_FCONV_NEXT_IN, INON);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, IN_FCONV_NEXT_IN, INOFF);
            }
            SendIOStatus(strSendMsg);
        }
        bSmemaNextOut = ReadOutputTimeOne(OUT_FCONV_NEXT_OUT, OUTON, TIME5MS);
        if (bSmemaNextOutOld != bSmemaNextOut)
        {
            bSmemaNextOutOld = bSmemaNextOut;
            if (bSmemaNextOut == true)
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, OUT_FCONV_NEXT_OUT, OUTON);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d"), NO_ERR, OUT_FCONV_NEXT_OUT, OUTOFF);
            }
            SendIOStatus(strSendMsg);
        }

        bFrAncLockIn = InputElapsedTimeOne(IN_FANC_CLAMP_LOCK, INON, TIME5MS);
        if (bFrAncLockIn != bFrAncLockInOld)
        {
            bFrAncLockInOld = bFrAncLockIn;

            if (bFrAncLockIn == true)
            {
                strSendMsg.Format(_T("%d,%d,%d,%d"), NO_ERR, IN_FANC_CLAMP_LOCK, INON, 0);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d,%d"), NO_ERR, IN_FANC_CLAMP_LOCK, INOFF, 0);
            }
            SendIOStatus(strSendMsg);
        }

        bFrAncUnlockIn = InputElapsedTimeOne(IN_FANC_CLAMP_UNLOCK, INON, TIME5MS);
        if (bFrAncUnlockIn != bFrAncUnlockInOld)
        {
            bFrAncUnlockInOld = bFrAncUnlockIn;

            if (bFrAncUnlockIn == true)
            {
                strSendMsg.Format(_T("%d,%d,%d,%d"), NO_ERR, IN_FANC_CLAMP_UNLOCK, INON, 0);
            }
            else
            {
                strSendMsg.Format(_T("%d,%d,%d,%d"), NO_ERR, IN_FANC_CLAMP_UNLOCK, INOFF, 0);
            }
            SendIOStatus(strSendMsg);
        }

		pThis->ExeBowlFeederOff();


        ThreadSleep(THREAD_MACHINE_MONITORINGTIME);
    }
    TRACE(_T("[PWR] CMachineInformation(0x%x) Quit\n"), pThis->m_ShowID);
    strLog.Format(_T("[PWR] CMachineInformation(0x%x) Quit\n"), pThis->m_ShowID);
    gcPowerLog->Logging(strLog);

	pThis->SetEnd(true);

    return 0;
}

void CMachineInformation::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thMachineInformation"));
    SetRunning(true);
    SetEnd(false);
    lpStartAddress = (_beginthreadex_proc_type)MonitoringMachineInformation;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] MachineInformation Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] MachineInformation Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

void CMachineInformation::InitialMotorAlarmHistory()
{
	bool openReuslt = false;
	CString fileName = _T("C:\\Power\\i6.0\\MCS\\MotorAlarm.db");
	std::string strSql;
	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;

	CSqlite3File* m_MotorAlarmHistory = new CSqlite3File();

	db = m_MotorAlarmHistory->OpenCreateDB(fileName, &openReuslt);
	if (openReuslt == false)
	{
		delete m_MotorAlarmHistory;
		return;
	}

	CString strTable;
	strTable.Format(_T("CREATE TABLE IF NOT EXISTS 'History' (timestamp DATE DEFAULT (datetime('now','localtime')), 'axis' TEXT, 'alarmcode' TEXT);"));

	strSql = CT2CA(strTable);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		delete m_MotorAlarmHistory;
		return;
	}

	Cwmx3Axis* pAxis = NULL;
	long code;
	for (long axis = 0; axis < GetWmx3AxisCount(); axis++)
	{
		pAxis = g_pWmx3AxisArray.GetAt(axis);

		code = pAxis->CheckAmpAlarmCode();
		m_AlarmCodeCurrent.push_back(code);
		m_AlarmCodeOld.push_back(code);
	}

	sqlite3_close(db);
	delete m_MotorAlarmHistory;

}

void CMachineInformation::InsertAlarmcode(CString strAxis, long alarmCode)
{
	bool openReuslt = false;
	CString fileName = _T("C:\\Power\\i6.0\\MCS\\MotorAlarm.db");
	std::string strSql;
	char* zErrMsg = 0;
	int rc;
	sqlite3* db;
	char** results;
	char* error;

	CSqlite3File* m_MotorAlarmHistory = new CSqlite3File();

	db = m_MotorAlarmHistory->OpenCreateDB(fileName, &openReuslt);
	if (openReuslt == false)
	{
		delete m_MotorAlarmHistory;
		return;
	}

	CString strTable;

	strTable.Format(_T("select count(*) from History;"));
	strSql = CT2CA(strTable);	

	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);
	if (rc == 0 && results[1] != NULL)
	{
		if (atoi(results[1]) >= 10000)
		{
			strTable.Format(_T("DELETE FROM	History WHERE ROWID IN(SELECT ROWID FROM History LIMIT 1);"));
			strSql = CT2CA(strTable);

			rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
			if (rc != SQLITE_OK)
			{
				sqlite3_close(db);
				delete m_MotorAlarmHistory;
				return;
			}
		}		
	}

	strTable.Format(_T("INSERT INTO 'History' ('axis', 'alarmcode') VALUES('%s', '0x%X');"), (LPCTSTR)strAxis, alarmCode);
	strSql = CT2CA(strTable);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		delete m_MotorAlarmHistory;
		return;
	}

	sqlite3_close(db);
	delete m_MotorAlarmHistory;

}

void CMachineInformation::SaveAlarmCode()
{
	Cwmx3Axis* pAxis = NULL;
	long code;
	bool alarmstate;

	for (long axis = 0; axis < m_AlarmCodeOld.size(); axis++)
	{
		pAxis = g_pWmx3AxisArray.GetAt(axis);
		alarmstate = pAxis->CheckAmpAlarm();
		code = pAxis->CheckAmpAlarmCode();

		//if (m_AlarmCodeOld.at(axis) != code && alarmstate == true)
		if (m_AlarmCodeOld.at(axis) != code)
		{
			m_AlarmCodeOld.at(axis) = code;
			InsertAlarmcode(pAxis->GetAxisName(), code);
		}
	}

}

void CMachineInformation::SetBowlFeederOffTime(BOWLFEEDER_OFFTIME data)
{
	Lock();
	m_BowlFeederOff = data;

	CString strMsg;
	strMsg.Format(_T("[PWR] SetBowlFeederOff"));

	strMsg.AppendFormat(_T(" Front:%d"), m_BowlFeederOff.ReadyOnTimeFront[0]);
	for (long idx = 1; idx < MAX_READYIO_NO; idx++)
	{
		strMsg.AppendFormat(_T(",%d"), m_BowlFeederOff.ReadyOnTimeFront[idx]);
	}

	strMsg.AppendFormat(_T(" Rear:%d"), m_BowlFeederOff.ReadyOnTimeRear[0]);
	for (long idx = 1; idx < MAX_READYIO_NO; idx++)
	{
		strMsg.AppendFormat(_T(",%d"), m_BowlFeederOff.ReadyOnTimeRear[idx]);
	}

	TRACE(_T("%s\n"), strMsg);
	Unlock();
}

BOWLFEEDER_OFFTIME CMachineInformation::GetBowlFeederOffTime()
{
	Lock();

	BOWLFEEDER_OFFTIME data = m_BowlFeederOff;

	Unlock();

	return data;
}

void CMachineInformation::ExeBowlFeederOff()
{
	BOWLFEEDER_OFFTIME data = GetBowlFeederOffTime();
	long readyIONo;
	long realeaseIONo;
	ULONGLONG onTime;
	long readyOffCountFront = 0;
	long readyOffCountRear = 0;

	for (long idx = 0; idx < MAX_READYIO_NO; idx++)
	{
		readyIONo = GetReadyIONoFromReadyNo(idx + 1);
		realeaseIONo = GetReleaseIONoFromReleaseNo(idx + 1);
		onTime = (ULONGLONG)data.ReadyOnTimeFront[idx];
		if (onTime == 0)
		{
			continue;
		}

		if (InputElapsedTimeOne(readyIONo, INON, onTime) == true)
		{

		}
		else
		{
			readyOffCountFront++;
		}
	}

	for (long idx = 0; idx < MAX_READYIO_NO; idx++)
	{
		readyIONo = GetReadyIONoFromReadyNo(idx + 1);
		realeaseIONo = GetReleaseIONoFromReleaseNo(idx + 1);
		onTime = (ULONGLONG)data.ReadyOnTimeFront[idx];

		if (onTime == 0)
		{
			continue;
		}

		if (readyOffCountFront == 0)
		{
			OutputOne(realeaseIONo, OUTON);
		}
		else
		{
			OutputOne(realeaseIONo, OUTOFF);
		}
	}

	for (long idx = 0; idx < MAX_READYIO_NO; idx++)
	{
		readyIONo = GetReadyIONoFromReadyNo(idx + 1 + MAX_FEEDER_READY_NO);
		realeaseIONo = GetReleaseIONoFromReleaseNo(idx + 1 + MAX_FEEDER_READY_NO);
		onTime = (ULONGLONG)data.ReadyOnTimeRear[idx];

		if (onTime == 0)
		{
			continue;
		}

		if (InputElapsedTimeOne(readyIONo, INON, onTime) == true)
		{

		}
		else
		{
			readyOffCountRear++;
		}
	}

	for (long idx = 0; idx < MAX_READYIO_NO; idx++)
	{
		readyIONo = GetReadyIONoFromReadyNo(idx + 1 + MAX_FEEDER_READY_NO);
		realeaseIONo = GetReleaseIONoFromReleaseNo(idx + 1 + MAX_FEEDER_READY_NO);
		onTime = (ULONGLONG)data.ReadyOnTimeRear[idx];

		if (onTime == 0)
		{
			continue;
		}

		if (readyOffCountRear == 0)
		{
			OutputOne(realeaseIONo, OUTON);
		}
		else
		{
			OutputOne(realeaseIONo, OUTOFF);
		}
	}
}
