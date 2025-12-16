#include "pch.h"
#include "CDecoding2.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "AxisInformation.h"
#include "CPowerHMI.h"
//#include "ErrorCode.h"
#include "CReadJobFile.h"
#include "CAdvancedMotionFile.h"
#include "CPowerMain.h"
#include "CPowerLog.h"
#include "GlobalIODefine.h"
#include "LockDef.h"
#include "CInsertEndFile.h"

CDecoding2* gcDecoding2;
CDecoding2::CDecoding2()
{
	m_HMIRunMode = HMI_RUN_CONTROL_INIT;
	GetId(&m_id);
}

CDecoding2::~CDecoding2()
{
}

BOOL CDecoding2::OnTask()
{
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding2::OnTask Thread(0x%04X)\n", m_id);
	}
	return TRUE;
}

BOOL CDecoding2::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	long Ret = 0;
	CString strMsg, strSendMsg;
	strSendMsg.Format(_T("0"));
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CDecoding2 GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}

		strSendMsg.Format(_T(STRING_UNDEFINED_MESSAGE));

		switch (nSubMsg[1])
		{
		case HMI_CMD2ND_00:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadRecipeFile(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = RunControl(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = FeedBackRunControl(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03)
				strSendMsg = ReadRunMode(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = GetReadyPrepareRun(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_06)
				strSendMsg = GetReadyRun(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_07)
				strSendMsg = RunBypass(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_13)
				strSendMsg = ResumeMoveOnce(strMsg);
			break;
		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 ReturnMsg(%s)\n", (LPCTSTR)strSendMsg);
		}

		if (strSendMsg.CompareNoCase(_T(STRING_UNDEFINED_MESSAGE)) == 0)
		{
			TRACE("[PWR] CDecoding2 STRING_UNDEFINED_MESSAGE(%s)\n", (LPCTSTR)(strSendMsg));
		}
		else if (strSendMsg.GetLength() > 0)
		{
			SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
		}
		delete msgReceived;
	}
	return TRUE;
}

long CDecoding2::SetHMIRunMode(long RunMode)
{
	m_HMIRunMode = RunMode;
	return NO_ERR;
}

long CDecoding2::GetHMIRunMode()
{
	return m_HMIRunMode;
}

CString CDecoding2::ReadRecipeFile(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	if (GetRunMode() == NORMAL_MODE)
	{
		Err = gcReadJobFile->ReadFile();
		if (Err == NO_ERR)
		{
			Err = gcAdvancedMotionFile->ReadFile();
		}
		strRetMsg.Format(_T("%d"), Err);
	}
	else
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
	}
	return strRetMsg;
}

CString CDecoding2::RunControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 RunControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue, strRetValue;
		int iValue[10];
		double dValue[10];
		long Control = 0;
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
		//if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] RunControl:%d\n"), iValue[0]);
		}
		Control = iValue[0];
		SetHMIRunMode(Control);
		if (Control == HMI_RUN_CONTROL_START)
		{
			strRetValue = StartRun();
			SetMachineManualActionRunninSign(false);
		}
		else if (Control == HMI_RUN_CONTROL_PAUSE)
			strRetValue = PauseRun();
		else if (Control == HMI_RUN_CONTROL_RESUME)
			strRetValue = ResumeRun();
		else if (Control == HMI_RUN_CONTROL_STOPNOW)
			strRetValue = StopNow();
		else if (Control == HMI_RUN_CONTROL_STOPSTEP)
			strRetValue = StopStep();
		else if (Control == HMI_RUN_CONTROL_STOPBLOCK)
			strRetValue = StopBlock();
		else if (Control == HMI_RUN_CONTROL_STOPBOARD)
			strRetValue = StopBoard();
		else if (Control == HMI_RUN_CONTROL_STOPCYCLE)
			strRetValue = StopCycle();
		else
			strRetValue = _T("20000");
		TRACE(_T("[PWR] RunControl Ret:%s\n"), strRetValue);
		strRetMsg.Format(_T("%s"), (LPCTSTR)strRetValue);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding2::FeedBackRunControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 FeedBackRunControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, State = HMI_RUN_CONTROL_INIT;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Control = 0;
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
		//if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] FeedBackRunControl:%d\n"), iValue[0]);
		}
		Err = iValue[0];
		State = iValue[1];
		TRACE(_T("[PWR] FeedBackRunControl Err:%d Recv:%d Send:%d\n"), Err, State, GetMachineState());
		//if (State == GetMachineState())
		//{
		//	StopConveyor();
		//	SetRunMode(NORMAL_MODE);
		//}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding2::ReadRunMode(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 RunControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Control = 0;
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
		Control = GetHMIRunMode();
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ReadRunMode Control:%d\n"), Control);
		}
		strRetMsg.Format(_T("%d"), Control);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}


CString CDecoding2::StartRun()
{
	CString strRetMsg;
	long Err = NO_ERR;

	if (IsAliveCStep(FRONT_GANTRY) == true)
	{
		DeleteCStep(FRONT_GANTRY);
	}
	CreateCStep(FRONT_GANTRY);

	if (GetWorkExistSkip() == 1 && GetInfiniteDryRun() == 1)
	{
		SetWorkPcbReady(FRONT_CONV, true);
		SetInsertDone(0);
		SetPcbOutDone(1);

		if (gcInsertEndFile)
		{
			TRACE(_T("[PWR] ***************************************** ClearInsertEnd Front Conveyor *****************************************\n"));
			gcInsertEndFile->ClearInsertEnd(FRONT_CONV);
			SaveInsertEnd();
		}
	}

	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (GetRunMode() == NORMAL_MODE)
	{
        Err = gcReadJobFile->AddBlockSequence();
        if (Err != NO_ERR)
        {
            strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
            return strRetMsg;
        }
		long FeederAll = 0;
		long ProdRunMode = 0;
		long ConveyorRunMode = 0;
		PRODUCTION Prod = gcReadJobFile->GetProduction();
		PCB Pcb = gcReadJobFile->GetPcb();
		BARCODE Barcode = gcReadJobFile->GetBarcode();
		long LastBlockNo = 0, LastInsertNo = 0, InsertDone = 0, ContinueRun = 0;
		long SettlingDelay = TIME200MS;
		ProdRunMode = Prod.ProdMode;
		ConveyorRunMode = Prod.BoardLocation;
		SendCameraRecognitionOffset(FRONT_GANTRY);
		//if (ProdRunMode == RUN_DRY)
		//{
		//	if (GetGlobalSettlingDelay() < TIME50MS)
		//	{
		//		SettlingDelay = TIME50MS; // Dry Run Z Height -10.0
		//		TRACE(_T("[PWR] Settling Add Delay,%d\n"), SettlingDelay);
		//		SetGlobalSettlingDelay(SettlingDelay);
		//	}
		//}
		TRACE(_T("[PWR] RunControl Step1\n"));
		SetProdRunMode(ProdRunMode, ConveyorRunMode);
		TowerLampRun();
		SetRunMode(PROD_RUN);
		SetStopMode(HMI_RUN_CONTROL_START);
		SetMachineState(STATE_RUN);
		SetGlobalStatusError(false);
		SetFeederProdRunMode(ProdRunMode);
		TRACE(_T("[PWR] RunControl Step2\n"));
		StartFeeder(FeederAll, 0, 0);
		TRACE(_T("[PWR] RunControl Step3\n"));
		StartConveyor(Barcode.Use, Barcode.Mes.Use, 0);
		TRACE(_T("[PWR] RunControl Step4\n"));
		StartSyncGantryConveyor();
		TRACE(_T("[PWR] RunControl Step5\n"));
		StartMain();
		TRACE(_T("[PWR] RunControl Step6\n"));
		PrepareMain();
		TRACE(_T("[PWR] RunControl Step7\n"));
		SetRunInfoConveyor(Barcode.Use, Barcode.Mes.Use, 0);
		TRACE(_T("[PWR] RunControl Step8\n"));
		//SetPcbInfoConveyor(Pcb.Thickness, Pcb.StandByPusherZOffsetHeight, Pcb.UseSimultaneousLoading);
		SetPcbInfoConveyor(Pcb.Thickness, Pcb.StandByPusherZOffsetHeight, Pcb.SimulLoadType);
		TRACE(_T("[PWR] RunControl Step9\n"));
		LastBlockNo = GetRemainFirstBlock(Pcb.MaxBlockCount, Prod.TotalInsertCount);
		LastInsertNo = GetRemainFirstNo(Pcb.MaxBlockCount, Prod.TotalInsertCount);
		InsertDone = GetInsertDone(FRONT_CONV);
		if (Pcb.UseHeightMeasurement == 0)
		{
			SetHeightMeasureDone(1);
		}

		if (IsExistSet(WORK1_CONV) == true)
		{
			TRACE(_T("[PWR] RunControl Last Block:%d Point:%d InsertDone(%d)\n"), LastBlockNo, LastInsertNo, InsertDone);
			if (LastBlockNo == MAXINSERTDONECOUNT || LastInsertNo == MAXINSERTDONECOUNT || InsertDone == 1)
			{
				if (GetHeightMeasureDone() == 0)
				{
					ContinueRun = 0;
				}
				else
				{
					ContinueRun = 1;
				}
			}
			else
			{
				ContinueRun = 0;
			}
		}
		else
		{
			ContinueRun = 0;
		}
		ThreadSleep(TIME10MS);
		RunConveyor(ContinueRun, Pcb.Width);
		TRACE(_T("[PWR] RunControl Step9\n"));
		RunFeeder(FeederAll);
		TRACE(_T("[PWR] RunControl Step10\n"));
		RunSyncGantryConveyor();
		TRACE(_T("[PWR] RunControl Step11\n"));
		RunMain(false);
		TRACE(_T("[PWR] RunControl Step12\n"));
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_START);
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, HMI_RUN_CONTROL_START);
	}
	return strRetMsg;
}

CString CDecoding2::PauseRun()
{
	long Err = NO_ERR;
	CString strRetMsg;
	if (GetRunMode() == PROD_RUN)
	{
		SetZRMoveOnce(false);
		SendPopupMessage(_T("Pause..."));

		BuzzerOff();
		Err = gPauseMachine();
		SetRunMode(PAUSE_MODE);
		MoveOneTimeLockWait(TIME1MS);

		SendMachineStartStopStatus(HMI_RUN_CONTROL_PAUSE);
		ThreadSleep(TIME500MS);

		WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
		WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
		WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_PAUSE);
		TRACE(_T("[PWR] PauseRun %s\n"), (LPCTSTR)strRetMsg);
		SendPopupClose();
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, HMI_RUN_CONTROL_PAUSE);
		TRACE(_T("[PWR] PauseRun NOT RUN %s\n"), (LPCTSTR)strRetMsg);
	}
	return strRetMsg;
}

CString CDecoding2::ResumeRun()
{
	long Err = NO_ERR;
	CString strRetMsg;
	if (GetRunMode() == PAUSE_MODE)
	{
		Err = gResumeMachine();
		SetRunMode(PROD_RUN);
		SendMachineStartStopStatus(HMI_RUN_CONTROL_RESUME);
		SetStopMode(HMI_RUN_CONTROL_RESUME);
		CycleStopUnlock(REAR_GANTRY);
		CycleStopUnlock(FRONT_GANTRY);
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_RESUME);
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PAUSEMODE, HMI_RUN_CONTROL_RESUME);
	}
	return strRetMsg;
}


CString CDecoding2::ResumeMoveOnce(CString strHostMsg)
{
	long Err = NO_ERR;
	CString strRetMsg, strMsg = strHostMsg;

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding2 ResumeMoveOnce TokenCount:%d\n"), cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, ZROneTime = NO_ERR, State = HMI_RUN_CONTROL_INIT;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Control = 0;
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

		ZROneTime = iValue[0];
		if (ZROneTime == 0)
		{
			SetZRMoveOnce(false);
		}
		else
		{
			SetZRMoveOnce(true);
		}

		if (GetRunMode() == PAUSE_MODE)
		{
			TRACE(_T("[PWR] CDecoding2 ResumeMoveOnce SetZR:%d\n"), GetZRMoveOnce());
			//ResetAxisPauseState();

			ResetLockOKAxisName();
			Err = gResumeMachine();
			ThreadSleep(TIME500MS);

			WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
			WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
			WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
			//	WaitAxisPauseStateByLock(TIME5000MS);
			WaitLockOKAxisIdle(TIME5000MS);
			CycleStopUnlock(REAR_GANTRY);
			CycleStopUnlock(FRONT_GANTRY);
			SetStopMode(HMI_RUN_CONTROL_RESUME);
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			TRACE(_T("[PWR] CDecoding2 ResumeMoveOnce not PauseMode:%d\n"), GetRunMode());
			strRetMsg.Format(_T("%d,%d"), ISNOT_PAUSEMODE, HMI_RUN_CONTROL_RESUME);
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


CString CDecoding2::StopNow()
{
	long Err = NO_ERR, FeederAll = 0;
	long TimeOut = TIME5000MS;
	CString strRetMsg;
	ULONGLONG GetTime = 0, Elapsed = 0;
	if (GetRunMode() == PAUSE_MODE)
	{
		SendPopupMessage(_T("Stop..."));

		TRACE(_T("[PWR] StopNow Step0\n"));
		SetMachineState(STATE_STOPNOW);
		TRACE(_T("[PWR] StopNow Step1\n"));
		gReleaseMachine();
		TRACE(_T("[PWR] StopNow Step2\n"));

		GetTime = _time_get();
		TRACE(_T("[PWR] Stop Conveyor Belt Stop Wait..\n"));
		while (GetConveyorBeltStop() == false)
		{
			Elapsed = _time_elapsed(GetTime);
			if (Elapsed > TIME10000MS)
			{
				TRACE(_T("[PWR] Stop Conveyor Belt Stop TimeOut\n"));
				break;
			}
			ThreadSleep(TIME100MS);
		}
		StopSyncGantryConveyor();
		TRACE(_T("[PWR] StopNow Step3\n"));
		EndConveyor();
		TRACE(_T("[PWR] StopNow Step4\n"));
		StopMain(HMI_RUN_CONTROL_STOPNOW);
		TRACE(_T("[PWR] StopNow Step5\n"));
		StopFeeder(FeederAll);
		TRACE(_T("[PWR] StopNow Step6\n"));
		CycleStopUnlock(REAR_GANTRY);
		CycleStopUnlock(FRONT_GANTRY);
		GetTime = _time_get();
		//if (GetConveyorRunMode() == LOCATION_OUT_NEXT)
		{
			while (GetConveyorEnd() == false)
			{
				Elapsed = _time_elapsed(GetTime);
				if (Elapsed > TimeOut)
				{
					TRACE(_T("[PWR] StopNow Conveyor TimeOut\n"));
					break;
				}
				ThreadSleep(TIME100MS);
			}
		}
		TRACE(_T("[PWR] StopNow Step7\n"));
		GetTime = _time_get();

		while (GetMainEnd() == false)
		{
			if (GetOnlyConveyorMode() == true)
			{
				break;
			}

			Elapsed = _time_elapsed(GetTime);
			if (Elapsed > TimeOut)
			{
				TRACE(_T("[PWR] StopNow Gantry TimeOut\n"));
				break;
			}
			ThreadSleep(TIME100MS);
		}
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] StopNow Step8\n"));
		StopConveyor();
		TRACE(_T("[PWR] StopNow Step9\n"));
		SetMachineState(STATE_IDLE);
		TRACE(_T("[PWR] StopNow Step10\n"));
		SetRunMode(NORMAL_MODE);
		TRACE(_T("[PWR] StopNow Step11\n"));
		TowerLampNormal();
		TRACE(_T("[PWR] StopNow Step12\n"));
		if (gcAdvancedMotionFile)
		{
			gcAdvancedMotionFile->ClearAdvanceMotion();
		}
		TRACE(_T("[PWR] StopNow Step13\n"));
		//for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO; ++AxisNo)
		//{
		//	SEM_FLUSH(gMOTION_LOCK[AxisNo]);
		//}
		//TRACE(_T("[PWR] StopNow Step14\n"));
		//for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO; ++AxisNo)
		//{
		//	SEM_UNLOCK(gMOTION_LOCK[AxisNo]);
		//}
		TRACE(_T("[PWR] StopNow Step15\n"));
		BuzzerOff();
		TRACE(_T("[PWR] StopNow Step16\n"));
		StopMonitor(_T("FZ1"));
		ThreadSleep(TIME500MS);

		SetStopMode(HMI_RUN_CONTROL_STOPNOW);

		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		SendPopupClose();

	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PRODRUNMODE, HMI_RUN_CONTROL_STOPNOW);
	}
	//delete pTime;
	return strRetMsg;
}

CString CDecoding2::StopStep()
{
	long Err = NO_ERR, FeederAll = 0;
	long TimeOut = TIME1000MS;
	CString strRetMsg;
	ULONGLONG GetTime = 0, Elapsed = 0;
	if (GetRunMode() == PAUSE_MODE)
	{
		SendPopupMessage(_T("Pause..."));

		TRACE(_T("[PWR] StopStep Step0\n"));
		SetMachineState(STATE_STOPSTEP);
		TRACE(_T("[PWR] StopStep Step1\n"));
		gReleaseMachine();
		TRACE(_T("[PWR] StopStep Step2\n"));
		StopSyncGantryConveyor();
		TRACE(_T("[PWR] StopStep Step3\n"));
		EndConveyor();
		TRACE(_T("[PWR] StopStep Step4\n"));
		StopMain(HMI_RUN_CONTROL_STOPSTEP);
		TRACE(_T("[PWR] StopStep Step5\n"));
		StopFeeder(FeederAll);
		TRACE(_T("[PWR] StopStep Step6\n"));
		GetTime = _time_get();
		//if (GetConveyorRunMode() == LOCATION_OUT_NEXT)
		{
			while (GetConveyorEnd() == false)
			{
				Elapsed = _time_elapsed(GetTime);
				if (Elapsed > TimeOut)
				{
					TRACE(_T("[PWR] StopStep Conveyor TimeOut\n"));
					break;
				}
				ThreadSleep(TIME100MS);
			}
		}
		TRACE(_T("[PWR] StopStep Step7\n"));
		GetTime = _time_get();
		while (GetMainEnd() == false)
		{
			Elapsed = _time_elapsed(GetTime);
			if (Elapsed > TimeOut)
			{
				TRACE(_T("[PWR] StopStep Gantry TimeOut\n"));
				break;
			}
			ThreadSleep(TIME100MS);
		}
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] StopStep Step8\n"));
		StopConveyor();
		TRACE(_T("[PWR] StopStep Step9\n"));
		SetMachineState(STATE_IDLE);
		TRACE(_T("[PWR] StopStep Step10\n"));
		SetRunMode(NORMAL_MODE);
		TRACE(_T("[PWR] StopStep Step11\n"));
		TowerLampNormal();
		TRACE(_T("[PWR] StopStep Step12\n"));
		if (gcAdvancedMotionFile)
		{
			gcAdvancedMotionFile->ClearAdvanceMotion();
		}
		TRACE(_T("[PWR] StopStep Step13\n"));
		//for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
		//{
		//	SEM_FLUSH(gMOTION_LOCK[AxisNo]);
		//}
		//TRACE(_T("[PWR] StopStep Step14\n"));
		//for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
		//{
		//	SEM_UNLOCK(gMOTION_LOCK[AxisNo]);
		//}
		TRACE(_T("[PWR] StopStep Step15\n"));
		BuzzerOff();
		TRACE(_T("[PWR] StopStep Step16\n"));
		ShowPCBSensor();
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPSTEP);
		SendPopupClose();

	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PRODRUNMODE, HMI_RUN_CONTROL_STOPSTEP);
	}
	return strRetMsg;
}

CString CDecoding2::StopBlock()
{
	long Err = NO_ERR, FeederAll = 0;
	long TimeOut = TIME1000MS;
	CString strRetMsg;
	ULONGLONG GetTime = 0, Elapsed = 0;
	if (GetRunMode() == PAUSE_MODE)
	{
		TRACE(_T("[PWR] StopBlock Step0\n"));
		SetMachineState(STATE_STOPBLOCK);
		TRACE(_T("[PWR] StopBlock Step1\n"));
		gReleaseMachine();
		TRACE(_T("[PWR] StopBlock Step2\n"));
		StopSyncGantryConveyor();
		TRACE(_T("[PWR] StopBlock Step3\n"));
		EndConveyor();
		TRACE(_T("[PWR] StopBlock Step4\n"));
		StopMain(HMI_RUN_CONTROL_STOPBLOCK);
		TRACE(_T("[PWR] StopBlock Step5\n"));
		StopFeeder(FeederAll);
		TRACE(_T("[PWR] StopBlock Step6\n"));
		GetTime = _time_get();
		//if (GetConveyorRunMode() == LOCATION_OUT_NEXT)
		{
			while (GetConveyorEnd() == false)
			{
				Elapsed = _time_elapsed(GetTime);
				if (Elapsed > TimeOut)
				{
					TRACE(_T("[PWR] StopBlock Conveyor TimeOut\n"));
					break;
				}
				ThreadSleep(TIME100MS);
			}
		}
		TRACE(_T("[PWR] StopBlock Step7\n"));
		GetTime = _time_get();
		while (GetMainEnd() == false)
		{
			Elapsed = _time_elapsed(GetTime);
			if (Elapsed > TimeOut)
			{
				TRACE(_T("[PWR] StopBlock Gantry TimeOut\n"));
				break;
			}
			ThreadSleep(TIME100MS);
		}
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] StopBlock Step8\n"));
		StopConveyor();
		TRACE(_T("[PWR] StopBlock Step9\n"));
		SetMachineState(STATE_IDLE);
		TRACE(_T("[PWR] StopBlock Step10\n"));
		SetRunMode(NORMAL_MODE);
		TRACE(_T("[PWR] StopBlock Step11\n"));
		TowerLampNormal();
		TRACE(_T("[PWR] StopBlock Step12\n"));
		if (gcAdvancedMotionFile)
		{
			gcAdvancedMotionFile->ClearAdvanceMotion();
		}
		TRACE(_T("[PWR] StopBlock Step13\n"));
		//for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
		//{
		//	SEM_FLUSH(gMOTION_LOCK[AxisNo]);
		//}
		//TRACE(_T("[PWR] StopBlock Step14\n"));
		//for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
		//{
		//	SEM_UNLOCK(gMOTION_LOCK[AxisNo]);
		//}
		TRACE(_T("[PWR] StopBlock Step15\n"));
		BuzzerOff();
		TRACE(_T("[PWR] StopBlock Step16\n"));
		ShowPCBSensor();
		CycleStopUnlock(REAR_GANTRY);
		CycleStopUnlock(FRONT_GANTRY);
		SetStopMode(HMI_RUN_CONTROL_STOPBLOCK);
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPBLOCK);
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PRODRUNMODE, HMI_RUN_CONTROL_STOPBLOCK);
	}
	return strRetMsg;
}

CString CDecoding2::StopBoard()
{
	long Err = NO_ERR, FeederAll = 0;
	long TimeOut = TIME1000MS;
	CString strRetMsg;
	ULONGLONG GetTime = 0, Elapsed = 0;
	if (GetRunMode() == PAUSE_MODE)
	{
		TRACE(_T("[PWR] ResumeRun by StopBoard\n"));
		ResumeRun();
		//StopMachine(STATE_STOPBOARD, HMI_RUN_CONTROL_STOPBOARD);
		SetStopMode(HMI_RUN_CONTROL_STOPBOARD);

		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPBOARD);
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PRODRUNMODE, HMI_RUN_CONTROL_STOPBOARD);
	}
	return strRetMsg;
}

CString CDecoding2::StopCycle()
{
	long Err = NO_ERR, FeederAll = 0;
	long TimeOut = TIME1000MS;
	CString strRetMsg;
	ULONGLONG GetTime = 0, Elapsed = 0;
	if (GetRunMode() == PAUSE_MODE)
	{
		TRACE(_T("[PWR] ResumeRun by StopCycle\n"));
		ResumeRun();

		//StopMachine(STATE_STOPBOARD, HMI_RUN_CONTROL_STOPBOARD);
		SetStopMode(HMI_RUN_CONTROL_STOPCYCLE);
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPCYCLE);
	}
	else
	{
		strRetMsg.Format(_T("%d,%d"), ISNOT_PRODRUNMODE, HMI_RUN_CONTROL_STOPBOARD);
	}
	return strRetMsg;
}


CString CDecoding2::GetReadyPrepareRun(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 GetReadyPrepareRun TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Err = 0;
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
		Err = iValue[0];
		if (Err == NO_ERR)
		{
			SetHMIReadyToRun(1);
		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] GetReadyPrepareRun Err:%d\n"), Err);
		}
		strRetMsg.Empty();
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding2::GetReadyRun(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 GetReadyRun TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Err = 0;
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
		Err = iValue[0];
		if (Err == NO_ERR)
		{
			SetHMIReadyToRun(0);
		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] GetReadyRun Err:%d\n"), Err);
		}
		strRetMsg.Empty();
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding2::RunBypass(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;

	if (GetOnlyConveyorMode() == true)
	{
		Err = CheckMachineSafety();

	}
	else
	{
		Err = CheckReadyToMachine(false);
	}

	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding2 RunBypass TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long ProdRunMode = 0;
		long ConveyorRunMode = 0;
		long Interval = 1, PcbLoadingCount = 0;
		double Width = 194.000, Length = 250.000, Thickness = 1.0, PuhserZOffset = 10.0;
		long WorkBeltMidDelay = TIME300MS;
		long WorkBeltLowDelay = TIME300MS;
		long SimultaneousLoading = 0;
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
		if (GetRunMode() == NORMAL_MODE)
		{
			ProdRunMode = RUN_BYPASS;
			ConveyorRunMode = LOCATION_OUT_NEXT;
			Interval = iValue[0] * TIME1000MS;
			Width = dValue[0];
			Length = dValue[1];
			Thickness = dValue[2];
			PuhserZOffset = dValue[3];
			PcbLoadingCount = iValue[1];
			WorkBeltMidDelay = iValue[2];
			WorkBeltLowDelay = iValue[3];
			SimultaneousLoading = iValue[4];
			TRACE(_T("[PWR] RunBypass Step1\n"));
			SetProdRunMode(ProdRunMode, ConveyorRunMode);
			TowerLampRun();
			SetRunMode(PROD_RUN);
			SetMachineState(STATE_RUN);
			SetGlobalStatusError(false);
			TRACE(_T("[PWR] RunBypass Step2\n"));
			StartConveyor(0, 0, 0);
			TRACE(_T("[PWR] RunBypass Step3\n"));
			SetPcbInfoConveyor(Thickness, PuhserZOffset, SimultaneousLoading);
			TRACE(_T("[PWR] RunBypass Step4\n"));
			ThreadSleep(TIME10MS);
			TRACE(_T("[PWR] RunBypass Step5\n"));
			RunConveyor(0, Width);
			TRACE(_T("[PWR] RunBypass Step6\n"));
			strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_START);
			SendMachineStartStopStatus(HMI_RUN_CONTROL_START);
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, HMI_RUN_CONTROL_START);
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
