#include "pch.h"
#include "CDecoding1.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "AxisInformation.h"
#include "CPowerHMI.h"
//#include "ErrorCode.h"
#include "VisionData.h"
#include "CReadJobFile.h"
#include "CMachineInit.h"
#include "CPowerLog.h"
#include "CHomeStatus.h"
#include "CMeasureHeight.h"
#include "CMachineConfig.h"
#include "CCamDropCheck.h"
#include "CCollisionMonitoring.h"
#include "CTrayDumpBox.h"

CDecoding1* gcDecoding1;
CDecoding1::CDecoding1()
{
	GetId(&m_id);
}

CDecoding1::~CDecoding1()
{
}

BOOL CDecoding1::OnTask()
{
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding1::OnTask Thread(0x%04X)\n", m_id);
	}
	return TRUE;
}

BOOL CDecoding1::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	long Ret = 0;
	CString strMsg, strSendMsg;
	ThreadId_t id;
	strSendMsg.Format(_T("0"));
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CDecoding1 GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}

		strSendMsg.Format(_T(STRING_UNDEFINED_MESSAGE));

		switch (nSubMsg[1])
		{
			case HMI_CMD2ND_01:
				if (nSubMsg[2] == HMI_CMD3RD_00)
				{
					TRACE(_T("[PWR] Initialize Machine\n"));
					strSendMsg = InitializeMachine(strMsg);
				}
				else if (nSubMsg[2] == HMI_CMD3RD_04)
				{
					TRACE(_T("[PWR] Prepare to quit Machine\n"));
					strSendMsg = PrepareToQuitMachine(strMsg);
				}
				else if (nSubMsg[2] == HMI_CMD3RD_05)
				{
					TRACE(_T("[PWR] Quit Machine\n"));
					strSendMsg = QuitMachine(strMsg);
				}
				break;
			case HMI_CMD2ND_02:
				if (gcPowerLog->IsShowCommunicationLog() == true)
				{
					TRACE(_T("[PWR] Check to initialized Machine Status\n"));
				}
				strSendMsg = CheckInitalizedMachine(strMsg);
				break;
			case HMI_CMD2ND_21:
				if (gcPowerLog->IsShowCommunicationLog() == true)
				{
					TRACE(_T("[PWR] Feeder Refill\n"));
				}
				strSendMsg = FeederRefill(strMsg);
				break;
			case HMI_CMD2ND_22:
				if (gcPowerLog->IsShowCommunicationLog() == true)
				{
					TRACE(_T("[PWR] Feeder Refill Done\n"));
				}
				strSendMsg = FeederRefillDone(strMsg);
				break;

			case HMI_CMD2ND_31:
				if (nSubMsg[2] == HMI_CMD3RD_00)
				{
					TRACE(_T("[PWR] ReceiveHeadSkipConfig\n"));
					strSendMsg = ReceiveHeadSkipConfig(strMsg);
				}
				break;
			case HMI_CMD2ND_33:
				if (nSubMsg[2] == HMI_CMD3RD_00)
				{
					TRACE(_T("[PWR] ReceiveMES_Disconnect\n"));
					strSendMsg = ReceiveMES_Disconnect(strMsg);
				}
				break;

		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 ReturnMsg(%s)\n", (LPCTSTR)strSendMsg);
		}
		if (strSendMsg.CompareNoCase(_T(STRING_UNDEFINED_MESSAGE)) == 0)
		{
			TRACE("[PWR] CDecoding1 STRING_UNDEFINED_MESSAGE(%s)\n", (LPCTSTR)(strSendMsg));
		}
		else if (strSendMsg.GetLength() > 0)
		{
			PowerThreadMessage* msgSend = new PowerThreadMessage();
			msgSend->SetThreadMsg(strSendMsg);
			msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			if (gcCPowerHMI)
			{
				gcCPowerHMI->GetId(&id);
				msgSend->SetID(id);
				if (gcCPowerHMI->PingThread(TIME1MS))
				{
					gcCPowerHMI->Event((LPVOID)msgSend);
				}
			}
		}
		delete msgReceived;
	}
	return TRUE;
}

CString CDecoding1::InitializeMachine(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	bool bRerverse = false;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 InitializeMachine TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
        constexpr int ARRAY_SIZE = 100;
		int iValue[ARRAY_SIZE];
		double dValue[ARRAY_SIZE];
		bool bSimulationMode = false, bHomingFail = false;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
                if (dCnt < ARRAY_SIZE)
                {
                    dValue[dCnt] = cTokenizer->GetDouble(i);
                }
				dCnt++;
			}
			else
			{
                if (iCnt < ARRAY_SIZE)
                {
                    iValue[iCnt] = cTokenizer->GetInt(i);
                }
				iCnt++;
			}
		}

		gCMachineConfig = new CMachineConfig();
		Err = gCMachineConfig->ReadConfigFile();
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Initial fail. Config & Cal file error"));
			ThreadSleep(TIME1000MS);
			KillMySelf();
		}

		gCMachineConfig->GetHWOptionCameraLaser();

		gcLastPickFront = new CLastPick(FRONT_GANTRY);
        Err = CLastPick::initializeDiscardInfo();
        if (Err != NO_ERR)
        {
            Err = SendAlarm(Err, _T("Initial fail. Config & Cal file error 2"));
            ThreadSleep(TIME1000MS);
            KillMySelf();
        }

        gcTrayDumpBox = new CTrayDumpBox();

		TRACE(_T("[PWR] *********************************************************"));
		TRACE(_T("[PWR] ******* InitializeMachine Option *******\n"));
		TRACE(_T("[PWR] ******* Conveyor Reverse:%d(0:Forward, 1:Reverse) *******\n"), iValue[0]);
		TRACE(_T("[PWR] ******* Rear Feeder Use :%d(0:Use, 1:NoUse)       *******\n"), iValue[1]);
		TRACE(_T("[PWR] ******* Rear Camera Use :%d(0:Use, 1:NoUse)       *******\n"), iValue[2]);
		TRACE(_T("[PWR] ******* Ball Screw  Type:%d(0:8mm, 1:10mm)        *******\n"), iValue[3]);
		TRACE(_T("[PWR] ******* Gantry X Type   :%d(0:1.5, 1:2.0)         *******\n"), iValue[4]);
		TRACE(_T("[PWR] ******* Area Sensor Use :%d(0:Use, 1:NoUse)       *******\n"), iValue[5]);
		TRACE(_T("[PWR] ******* RTD FX  Use     :%d(0:Use, 1:NoUse)       *******\n"), iValue[6]);
		TRACE(_T("[PWR] ******* RTD FY1 Use     :%d(0:Use, 1:NoUse)       *******\n"), iValue[7]);
		TRACE(_T("[PWR] ******* RTD FY2 Use     :%d(0:Use, 1:NoUse)       *******\n"), iValue[8]);
		TRACE(_T("[PWR] ******* TTF Use         :%d(1:Use, 0:NoUse)       *******\n"), iValue[9]);
		TRACE(_T("[PWR] ******* Front ANC Use   :%d(1:Use, 0:NoUse)       *******\n"), iValue[10]);
		TRACE(_T("[PWR] ******* Rear ANC Use    :%d(1:Use, 0:NoUse)       *******\n"), iValue[11]);
		TRACE(_T("[PWR] ******* Manual Conveyor :%d(1:Use, 0:NoUse)       *******\n"), iValue[12]);
		TRACE(_T("[PWR] ******* Front Cam Count :%d                       *******\n"), iValue[13]);
		TRACE(_T("[PWR] ******* Front LED Type  :%d(1:3Ch, 2:Bar Light)   *******\n"), iValue[14]);
		TRACE(_T("[PWR] ******* Front Laser Use :%d(1:Use, 2:NoUse)       *******\n"), iValue[15]);
		TRACE(_T("[PWR] ******* Rear Cam Count  :%d                       *******\n"), iValue[16]);
		TRACE(_T("[PWR] ******* Rear LED Type   :%d(1:3Ch, 2:Bar Light)   *******\n"), iValue[17]);
		TRACE(_T("[PWR] ******* Rear Laser Use  :%d(1:Use, 2:NoUse)       *******\n"), iValue[18]);
		TRACE(_T("[PWR] ******* Exit Stopper Use:%d(1:Use, 2:NoUse)       *******\n"), iValue[19]);
		TRACE(_T("[PWR] ******* NG    Buffer Use:%d(1:Use, 0:NoUse)       *******\n"), iValue[20]);
		TRACE(_T("[PWR] ******* Front Backup Block Exist Sensor Use:%d(1:Use, 0:NoUse)       *******\n"), iValue[21]);
		TRACE(_T("[PWR] ******* Rear  Backup Block Exist Sensor Use:%d(1:Use, 0:NoUse)       *******\n"), iValue[22]);
		TRACE(_T("[PWR] ******* ANC UpDown Type:%d(1:UpDown, 0:Fix)       *******\n"), iValue[23]);
		TRACE(_T("[PWR] ******* Rear Area Sensor Use:%d(1:Use, 0:NoUse)   *******\n"), iValue[24]);
		TRACE(_T("[PWR] ******* Rear Safety Cover Use:%d(1:Use, 0:NoUse)  *******\n"), iValue[25]);
		TRACE(_T("[PWR] ******* Ionizer Use:%d(1:Use, 0:NoUse)	          *******\n"), iValue[26]);
		TRACE(_T("[PWR] ******* Only Conveyor Use:%d(1:Use, 0:NoUse)	  *******\n"), iValue[27]);
		TRACE(_T("[PWR] ******* Y2 Shift Auto Init:%d,%.1f   	          *******\n"), iValue[28], dValue[0]);
		TRACE(_T("[PWR] ******* Area Sensor2nd Use :%d(1:Use, 0:NoUse)    *******\n"), iValue[29]);
		TRACE(_T("[PWR] ******* Forming Type :%d(0:Standard, 1:2Step)     *******\n"), iValue[37]);
		TRACE(_T("[PWR] *********************************************************"));
		if (iValue[0] == 1)
		{
			bRerverse = true;
		}
		SetRecvInitializeMachine(true);
		if (GetInitializedMachine() == false)
		{
			gcMachineInit = new CMachineInit(bSimulationMode);
			gcPowerLog->Logging(_T("[PWR] gcMachineInit->VariableInit"));

			TRACE(_T("[PWR] gcMachineInit->VariableInit\n"));
			gcMachineInit->VariableInit();

			if (iValue[2] == 0)	
			{
				SetUseRearCamera(1);
			}
			else
			{
				SetUseRearCamera(0);
			}

			if (iValue[5] == 0)
			{
				SetUseAreaSensor(1);
			}
			else
			{
				SetUseAreaSensor(0);
			}

			if (iValue[29] == 1)
			{
				SetUseAreaSensor2nd(1);

				if (GetUseAreaSensor() == 0)
				{
					SetUseAreaSensor(1);
					TRACE(_T("[PWR] SetUseAreaSensor forced\n"));
				}
			}
			else
			{
				SetUseAreaSensor2nd(0);
			}

			if (iValue[6] == 0)
			{
				SetUseRTDSensorFX(1);
			}
			else
			{
				SetUseRTDSensorFX(0);
			}

			if (iValue[7] == 0)
			{
				SetUseRTDSensorFY1(1);
			}
			else
			{
				SetUseRTDSensorFY1(0);
			}

			if (iValue[8] == 0)
			{
				SetUseRTDSensorFY2(1);
			}
			else
			{
				SetUseRTDSensorFY2(0);
			}

			if (iValue[9] == 0) // no use
			{
				SetTTFUse(false);
			}
			else
			{
				//	SetUseTTFAxis();
				SetTTFUse(true);
			}

			if (iValue[10] == 0)
			{
				SetUseANC(FRONT_STAGE, 0);

			}
			else
			{
				SetUseANC(FRONT_STAGE, 1);
			}

			if (iValue[11] == 0)
			{
				SetUseANC(REAR_STAGE, 0);

			}
			else
			{
				SetUseANC(REAR_STAGE, 1);
			}

			if (iValue[12] == 0)
			{
				SetManualConvUse(false);

			}
			else
			{
				SetManualConvUse(true);
				//	SetClearConveyorAxis();
			}

			if (iValue[37] == 0)
			{
				SetFormingDRBCoilUse(false);
			}
			else
			{
				SetFormingDRBCoilUse(true);
			}

			TRACE(_T("[PWR] ******* Front Cam Count :%d                       *******\n"), iValue[13]);
			SetCameraCount(FRONT_VISION, iValue[13]);
			TRACE(_T("[PWR] ******* Front LED Type  :%d(1:3Ch, 2:Bar Light)   *******\n"), iValue[14]);
			SetModuleCamType(FRONT_VISION, iValue[14]);
			if (iValue[14] == 2) // Bar Light
			{
				SetModuleCamLedChannel(FRONT_VISION, 1);
			}
			else // 3 Channel
			{
				SetModuleCamLedChannel(FRONT_VISION, 3);
			}
			TRACE(_T("[PWR] ******* Front Laser Use :%d(1:Use, 2:NoUse)       *******\n"), iValue[15]);
			SetModuleCamLaserUse(FRONT_VISION, iValue[15]);
			
			TRACE(_T("[PWR] ******* Rear Cam Count  :%d                       *******\n"), iValue[16]);
			SetCameraCount(REAR_VISION, iValue[16]);
			TRACE(_T("[PWR] ******* Rear LED Type   :%d(1:3Ch, 2:Bar Light)   *******\n"), iValue[17]);
			SetModuleCamType(REAR_VISION, iValue[17]);
			if (iValue[17] == 2) // Bar Light
			{
				SetModuleCamLedChannel(REAR_VISION, 1);
			}
			else // 3 Channel
			{
				SetModuleCamLedChannel(REAR_VISION, 3);
			}
			TRACE(_T("[PWR] ******* Rear Laser Use  :%d(1:Use, 2:NoUse)       *******\n"), iValue[18]);
			SetModuleCamLaserUse(REAR_VISION, iValue[18]);

			if (iValue[19] == 1)
			{
				SetUseExitConvStopper(true);
			}
			else
			{
				SetUseExitConvStopper(false);
			}

			if (iValue[27] == 1)
			{
				SetOnlyConveyorMode(true);
			}
			else
			{
				SetOnlyConveyorMode(false);

			}

			Y2SHIFT_AUTOINIT y2shift;
			if (iValue[28] == 1)
			{
				y2shift.Use = true;
				y2shift.TorqueLimit = abs(dValue[0]);
			}
			else
			{
				y2shift.Use = false;
				y2shift.TorqueLimit = abs(dValue[0]);
			}
			SetY2ShiftAutoInit(y2shift);

			SetSimulationForming(false);

			SetMachineManualActionRunninSign(false);

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->CalibrationDataInit"));
			TRACE(_T("[PWR] gcMachineInit->CalibrationDataInit\n"));
			gcMachineInit->CalibrationDataInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->ConveyorDataInit"));
			TRACE(_T("[PWR] gcMachineInit->ConveyorDataInit\n"));
			gcMachineInit->ConveyorDataInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->SystemFileInit"));
			TRACE(_T("[PWR] gcMachineInit->SystemFileInit\n"));
			gcMachineInit->SystemFileInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->MotionInit"));
			TRACE(_T("[PWR] gcMachineInit->MotionInit\n"));
			Err = gcMachineInit->MotionInit(bRerverse);
			if (Err != NO_ERR)
			{
				strRetMsg.Format(_T("%d"), Err);
				return strRetMsg;
			}

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->VisionInit"));
			TRACE(_T("[PWR] gcMachineInit->VisionInit\n"));
			gcMachineInit->VisionInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->ConveyorInit"));
			TRACE(_T("[PWR] gcMachineInit->ConveyorInit\n"));
			gcMachineInit->ConveyorInit(bRerverse);

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->FeederInit"));
			TRACE(_T("[PWR] gcMachineInit->FeederInit\n"));
			gcMachineInit->FeederInit();

			TRACE(_T("[PWR] gcMeasureHeight Create\n"));
			gcMeasureHeight = new CMeasureHeight();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->MainInit"));
			TRACE(_T("[PWR] gcMachineInit->MainInit\n"));
			gcMachineInit->MainInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->TeachBoxInit"));
			TRACE(_T("[PWR] gcMachineInit->TeachBoxInit\n"));
			gcMachineInit->TeachBoxInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->GantryInit"));
			TRACE(_T("[PWR] gcMachineInit->GantryInit\n"));
			gcMachineInit->GantryInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->LedControlInit"));
			TRACE(_T("[PWR] gcMachineInit->LedControlInit\n"));
			gcMachineInit->LedControlInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->BarcoodeControlInit"));
			TRACE(_T("[PWR] gcMachineInit->BarcoodeControlInit\n"));
			gcMachineInit->BarcoodeControlInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->CleanerInit"));
			TRACE(_T("[PWR] gcMachineInit->CleanerInit\n"));
			gcMachineInit->CleanerInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->MachineFileInit"));
			TRACE(_T("[PWR] gcMachineInit->MachineFileInit\n"));
			gcMachineInit->MachineFileInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->SwitchPanelInit"));
			TRACE(_T("[PWR] gcMachineInit->SwitchPanelInit\n"));
			gcMachineInit->SwitchPanelInit();

			gcPowerLog->Logging(_T("[PWR] gcMachineInit->TowerLampInit"));
			TRACE(_T("[PWR] gcMachineInit->TowerLampInit\n"));
			gcMachineInit->TowerLampInit();
			
			gcPowerLog->Logging(_T("[PWR] gcMachineInit->BuzzerInit"));
			TRACE(_T("[PWR] gcMachineInit->BuzzerInit\n"));
			gcMachineInit->BuzzerInit();
			
			//ClearCameraRecognitionPosition(FRONT_GANTRY);
			gcPowerLog->Logging(_T("[PWR] gcMachineInit->MonitoringMachineInformationInit"));
			TRACE(_T("[PWR] gcMachineInit->MonitoringMachineInformationInit\n"));
			gcMachineInit->MonitoringMachineInformationInit();

			//gcPowerLog->Logging(_T("[PWR] gcMachineInit->VirtualSerialInit"));
			//TRACE(_T("[PWR] gcMachineInit->VirtualSerialInit\n"));
			//gcMachineInit->VirtualSerialInit();

			InitCStep(FRONT_GANTRY);

			gcCamDropCheck[FRONT_STAGE] = new CCamDropCheck(FRONT_STAGE);
			gcCamDropCheck[REAR_STAGE] = new CCamDropCheck(REAR_STAGE);


			if (GetSkipMotorPower() == true)
			{
				gcPowerLog->Logging(_T("[PWR] Skip gcMachineInit->MotorPowerInit"));
				TRACE(_T("[PWR] Skip gcMachineInit->MotorPowerInit\n"));
				ThreadSleep(TIME1000MS);
			}
			else
			{
				gcPowerLog->Logging(_T("[PWR] gcMachineInit->MotorPowerInit"));
				TRACE(_T("[PWR] gcMachineInit->MotorPowerInit\n"));
				gcMachineInit->MotorPowerInit();
				Err = gcMachineInit->WaitMotorPowerOn(TIME10000MS * TIME10000MS);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("Motor Power Enable error"));
					ThreadSleep(TIME1000MS);
					KillMySelf();
				}
			}

			if (GetInitY2Shift() == true)
			{
				ClearY2Shift(FRONT_GANTRY);
			}

			TowerLampNormal();
			HeightMeasurementControl(false);

			SetAxisSkip();

			SetBarcodeSimulation(false);

			if (GetAutoHomingUse() == true && GetOnlyConveyorMode() == false)
			{
				ThreadSleep(TIME1000MS);
				SetRunMode(ORIG_MODE);
				gcHomeStatus = new CHomeStatus();
				SendInitializeHome(0);
				gcHomeStatus->Run();
				while (IsAllAxisHomingComplete() == false)
				{
					if (IsAllAxisHomingFail() == true)
					{
						bHomingFail = true;
						break;
					}
					ThreadSleep(TIME1000MS);
				}
				if (bHomingFail == false)
				{
					TRACE(_T("[PWR] ########## Wait to All axis(%d) homing complete    ########## \n"), GetWmx3AxisCount());
				}
				ThreadSleep(TIME1000MS);
				SetRunMode(NORMAL_MODE);
			}
			if (bHomingFail == false)
			{
				TRACE(_T("[PWR] Get1DCompensation:%s\n"), Get1DCompensationUse() == true ? _T("Use") : _T("UnUse"));
				if (Get1DCompensationUse() == true)
				{
					oneDCompensationOn();
				}
				else
				{
					oneDCompensationOff();
				}
				TRACE(_T("[PWR] Get2DCompensation:%s\n"), Get2DCompensationUse() == true ? _T("Use") : _T("UnUse"));
				if (Get2DCompensationUse() == true)
				{
					twoDCompensationOn();
				}
				else
				{
					twoDCompensationOff();
				}
				TRACE(_T("[PWR] GetZCompensation:%s\n"), GetZCompensationUse() == true ? _T("Use") : _T("UnUse"));
				if (GetZCompensationUse() == true)
				{
					AllZCompensationOn(FRONT_GANTRY);
				}
				else
				{
					AllZCompensationOff(FRONT_GANTRY);
				}
				gUpdateVisionData(FRONT_VISION, 0, 1);
				SendCameraRecognitionOffset(FRONT_GANTRY);
				gLiveOn(FHCAM);
				SendInitializeHome(1);
				ThreadSleep(TIME500MS);
				SetInitializedMachine(true);
				ReadAllPosition(FRONT_GANTRY);
				SetMachineState(STATE_IDLE);
				SendInitializeHMI(HMI_INITIALIZE_COMPLETE);
				TowerLampNormal();
			}
			else
			{
				SendInitializeHome(1);
				ThreadSleep(TIME500MS);
				SetInitializedMachine(true);
				ReadAllPosition(FRONT_GANTRY);
				SetMachineState(STATE_IDLE);
				SendInitializeHMI(HMI_INITIALIZE_COMPLETE);
				TowerLampNormal();
			}

			gcCollisionMonitoring = new CCollisionMonitoring();
			gcCollisionMonitoring->Run();
			SendToChangeMachineState(TimeStatics::STOP);

			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ALREADY_INITIALIZED_MACHINE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding1::CheckInitalizedMachine(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 CheckInitalizedMachine TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[10];
		double dValue[10];
		bool InitializedMachine = false;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		InitializedMachine = GetInitializedMachine();
		if(InitializedMachine == true)
			strRetMsg.Format(_T("%d"), INITIALIED_MACHINE_OK);
		else
			strRetMsg.Format(_T("%d"), INITIALIED_MACHINE_NG);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding1::PrepareToQuitMachine(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 PrepareToQuitMachine TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[10];
		double dValue[10];
		double Ratio = 0.5;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (GetRunMode() == NORMAL_MODE)
		{
			if (GetOnlyConveyorMode() == false)
			{
				Err = MoveZStandySkipServoOff(FRONT_GANTRY, GetStandByZ(FRONT_GANTRY), Ratio);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] PrepareToQuitMachine Front MoveZStandy Err:%d\n"), Err);
				}
				ThreadSleep(TIME500MS);

				Err = gServoAllOff();
			}
			ThreadSleep(TIME100MS);

			DoorLockingControl(false); // Prepare to shutdown
			MotorPowerControl(false);
			gQuitMachine();
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}


CString CDecoding1::QuitMachine(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 QuitMachine TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[10];
		double dValue[10];
		bool m_bSimulationMode = false;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		if (GetRunMode() == NORMAL_MODE)
		{			
			strRetMsg.Format(_T("%d"), Err);			
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding1::FeederRefill(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 FeederRefill TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		long FeederNo = 0;
		CString strValue, strAxis;
		int iValue[10];
		double dValue[10];
		bool m_bSimulationMode = false;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		TRACE("[PWR] CDecoding1 FeederRefill FeederNo:%d\n", iValue[0]);
		FeederNo = iValue[0];
		if (GetRunMode() == PROD_RUN || GetRunMode() == PAUSE_MODE)
		{
			if (FeederNo > 0)
			{
				gFeederRefill(FeederNo);
			}
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_PRODRUNMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding1::FeederRefillDone(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 FeederRefillDone TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[10];
		double dValue[10];
		bool m_bSimulationMode = false;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		if (GetRunMode() == PROD_RUN || GetRunMode() == PAUSE_MODE)
		{
			gFeederRefillDone();
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_PRODRUNMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

void CDecoding1::KillMySelf()
{
	TRACE(_T("[PWR] CDecoding1::KillMySelf \n"));

	DWORD pid = GetCurrentProcessId();
	HANDLE hnd;
	hnd = OpenProcess(SYNCHRONIZE | PROCESS_TERMINATE, TRUE, pid);
	TerminateProcess(hnd, 0);
}

CString CDecoding1::ReceiveHeadSkipConfig(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 SetHeadSkip TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[20];
		double dValue[20];
		long headIdx;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}

		headIdx = 1;
		for (long idx = 0; idx < MAXUSEDHEADNO; idx++)
		{
			if (iValue[idx] == 1)
			{
				SetHeadSkip(FRONT_GANTRY, headIdx, true);
			}
			headIdx++;
		}

		headIdx = 1;
		for (long idx = MAXUSEDHEADNO; idx < MAXUSEDHEADNO*2; idx++)
		{
			if (iValue[idx] == 1)
			{
				SetHeadSkip(REAR_GANTRY, headIdx, true);
			}
			headIdx++;
		}

		strRetMsg.Format(_T("%d"), NO_ERR);

		//TRACE("[PWR] CDecoding1 SetHeadSkip Front:%d,%d,%d,%d,%d,%d\n", g_HeadSkipFront[0], g_HeadSkipFront[1], g_HeadSkipFront[2], g_HeadSkipFront[3], g_HeadSkipFront[4], g_HeadSkipFront[5]);
		//TRACE("[PWR] CDecoding1 SetHeadSkip Rear:%d,%d,%d,%d,%d,%d\n", g_HeadSkipRear[0], g_HeadSkipRear[1], g_HeadSkipRear[2], g_HeadSkipRear[3], g_HeadSkipRear[4], g_HeadSkipRear[5]);

		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

void CDecoding1::SetAxisSkip()
{
	CString strAxisZ;
	CString strAxisR;
	Cwmx3Axis* pAxisZ;
	Cwmx3Axis* pAxisR;

	for (long HeadNo = 1; HeadNo <= MAXUSEDHEADNO; HeadNo++)
	{
		if (GetHeadSkip(FRONT_GANTRY, HeadNo) == true)
		{
			strAxisZ = GetZAxisFromHeadNo(FRONT_GANTRY, HeadNo);
			strAxisR = GetRAxisFromHeadNo(FRONT_GANTRY, HeadNo);

			pAxisZ = GetWmx3AxisByName(strAxisZ);
			pAxisR = GetWmx3AxisByName(strAxisR);

			pAxisZ->SetAxisSkip(true);
			pAxisR->SetAxisSkip(true);
		}

		//if (m_HeadSkipRear[idx] == true)
		//{
		//	strAxisZ = GetZAxisFromHeadNo(REAR_GANTRY, TBL_HEAD1 + idx);
		//	strAxisR = GetRAxisFromHeadNo(REAR_GANTRY, TBL_HEAD1 + idx);

		//	pAxisZ = GetWmx3AxisByName(strAxisZ);
		//	pAxisR = GetWmx3AxisByName(strAxisR);

		//	pAxisZ->SetAxisSkip(true);
		//	pAxisR->SetAxisSkip(true);
		//}

	}
}

CString CDecoding1::ReceiveMES_Disconnect(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding1 ReceiveMES_Disconnect TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0, Err = NO_ERR;
		CString strValue, strAxis;
		int iValue[20];
		double dValue[20];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}

		gMainMES_Disconnect();

		strRetMsg.Format(_T("%d"), NO_ERR);

		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}