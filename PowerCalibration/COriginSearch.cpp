#include "pch.h"
#include "COriginSearch.h"
#include "Cwmx3Axis.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "CPowerConveyorData.h"
#include "CPowerCalibrationData.h"
#include "AxisInformation.h"
#include "CPowerLog.h"
//#include "ErrorCode.h"

COriginSearch::COriginSearch(CString strAxis)
{
	m_HomingErr = 0;
	m_Axis = strAxis;
	m_pAxis = GetWmx3AxisByName(strAxis);
	m_HomingStep = HomingStep::INIT;
}

COriginSearch::~COriginSearch()
{
}

CString COriginSearch::GetAxisName()
{
	return m_Axis;
}

long COriginSearch::GetHomingErr()
{
	return m_HomingErr;
}

bool COriginSearch::isHomingFail()
{
	bool result = false;
	result = m_pAxis->GetInitializeFail();
	return result;
}

long COriginSearch::SetHomingErr(long HomingErr)
{
	m_HomingErr = HomingErr;
	return NO_ERR;
}

HomingStep COriginSearch::GetStep()
{
	return m_HomingStep;
}

void COriginSearch::Run()
{
	CString strLog, threadName;
	HANDLE nHandle;
	DWORD nID = NULL;
	_beginthreadex_proc_type lpStartAddress;
	SetRunning(true);
	lpStartAddress = (_beginthreadex_proc_type)StartOriginSearch;
	nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	if (nHandle != INVALID_HANDLE_VALUE)
		SetThreadHandle(nHandle);
	if (nID != NULL)
		SetThreadID(nID);
	threadName.Format(_T("thAxis%d"), m_pAxis->GetAxisIndex());
	SetThreadName(threadName);
	TRACE(_T("[PWR] Homing Thread Axis(%d) ID:0x%04X(%s)\n"), m_pAxis->GetAxisIndex(), GetThreadID(), GetThreadName());
	strLog.Format(_T("[PWR] Homing Thread Axis(%d) ID:0x%04X(%s)"), m_pAxis->GetAxisIndex(), GetThreadID(), (LPCTSTR)GetThreadName());
	gcPowerLog->Logging(strLog);
}

UINT COriginSearch::StartOriginSearch(LPVOID wParam)
{
	COriginSearch* pThis = reinterpret_cast<COriginSearch*>(wParam);
	bool bLoop = true, bRet = false;;
	signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	CString strHostMsg;
	int err = ErrorCode::None;
	long lTimeChk = 0;
	double dblDir = -1.0, dblPos = 0.0, HomeShift = 0.0, LoadPosition = 0.0, LoadPulse = 0.0, HomePosition = 0.0;
	HomingStep nOldHomingStep = HomingStep::MAX;
	Config::HomeParam* homeParam = new Config::HomeParam();
	Config::HomeParam* slaveHomeParam = new Config::HomeParam();
	Config::LimitParam* limitParam = new Config::LimitParam();
	CString strErrMsg, strAxisName;
	//CApplicationTime* pTime = new CApplicationTime();
	ULONGLONG GetTime = 0, Elapsed = 0;
	ULONGLONG GetHomingTime = 0, HomingElapsed = 0;
	double PusherZ = 0.0, Ratio = 0.1, Inpos = 0.010;
	long TimeOut = TIME5000MS, Ms = TIME100MS;
	bool bSkipConvWidthHome = false;
	long Y1Encoder = 0, Y2Encoder = 0, Y1Y2Diff = 0;
	bool firstTimeCatchShift = true;
	bool firstTimeShiftInit = true;
	long maxWaitTime = TIME10000MS;

	pThis->m_HomingStep = pThis->GetStartHomingStep();
	pThis->SetHomingErr(NO_ERR);
	TRACE(_T("[PWR] StartOriginSearch(%s)\n"), pThis->GetThreadName());
	//pThis->SetMsgQueueStatus(THREAD_RUN);
	GetTime = _time_get();

	if (GetGlobalSimulationMode() == true)
	{
		pThis->m_pAxis->GetHomeParam(homeParam);
		homeParam->homeType = Config::HomeType::CurrentPos;
		pThis->m_pAxis->SetHomeParam(homeParam);
		TRACE(_T("[PWR] StartOriginSearch(%s) SimulationMode HomeType::CurrentPos\n"), pThis->GetThreadName());

	}

	while (pThis->GetRunning() == true)
	{
		if (nOldHomingStep != pThis->m_HomingStep)
		{
			Elapsed = _time_elapsed(GetTime);
			//if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] StartOriginSearch Axis(%d,%d,%s) Step:%d Time:%d[ms]\n"), pThis->m_pAxis->GetAxisIndex(), pThis->m_pAxis->GetAxisMap(), pThis->m_pAxis->GetAxisName(), pThis->m_HomingStep, Elapsed);
			}
			nOldHomingStep = pThis->m_HomingStep;
			GetTime = _time_get();
		}
		else
		{
			if (pThis->m_pAxis->IsGantryAxis() == true)
			{
				maxWaitTime = TIME50000MS;
			}
			else if (pThis->m_pAxis->IsConveyorAxis() == true)
			{
				maxWaitTime = TIME30000MS;
			}
			else
			{
				maxWaitTime = TIME10000MS;
			}

			Elapsed = _time_elapsed(GetTime);
			if (Elapsed > maxWaitTime)
			{
				GetTime = _time_get();
				TRACE(_T("[PWR] StartOriginSearch Axis(%d,%d,%s) Step:%d TimeOut:%d[ms]\n"),
					pThis->m_pAxis->GetAxisIndex(),
					pThis->m_pAxis->GetAxisMap(),
					pThis->m_pAxis->GetAxisName(),
					pThis->m_HomingStep, Elapsed);

				pThis->m_HomingStep = HomingStep::SELFQUIT;
				pThis->m_pAxis->SetInitializeFail(true);
				SendAlarm(HOMING_TIMOUT(pThis->m_pAxis->GetAxisIndex()), _T("Homing TimeOut"));
			}
		}
		if (pThis->IsTerminated(THREAD_ORIGIN_SEARCH_TIME) == true)
		{
			TRACE(_T("[PWR] StartOriginSearch Axis(%d,%d) Terminated\n"), pThis->m_pAxis->GetAxisIndex(), pThis->m_pAxis->GetAxisMap());
			break;
		}
		switch (pThis->m_HomingStep)
		{
		case HomingStep::INIT:

			//if (pThis->m_pAxis->GetAxisSkip() == true)
			//{
			//	pThis->m_HomingStep = HomingStep::SELFQUIT;
			//	pThis->m_pAxis->SetInitializeEnd(true);
			//	pThis->m_pAxis->SetInitializeFail(false);
			//}
			//else
			{
				pThis->m_HomingStep = HomingStep::START;
				pThis->m_pAxis->SetInitializeEnd(false);
				pThis->m_pAxis->SetInitializeFail(false);
			}

			break;

		case HomingStep::START:
			pThis->m_HomingStep = HomingStep::CLEARALARM;
			break;

		case HomingStep::CLEARALARM:
			if (pThis->m_pAxis->CheckAmpAlarm() == true)
			{
				err = pThis->m_pAxis->ClearAmpAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearAmpAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
				err = pThis->m_pAxis->ClearAxisAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearAxisAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_HomingStep = HomingStep::AXISSERVO_OFF;
			}
			break;

		case HomingStep::AXISSERVO_OFF:
			if (pThis->m_pAxis->CheckServoOn() == true)
			{
				err = pThis->m_pAxis->ServoOff();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ServoServoOffOn Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				if (pThis->m_pAxis->GetUseSlaveAxis() == true) // Use Master and Slave
				{
					pThis->m_HomingStep = HomingStep::SLAVEAXISSERVO_OFF;
				}
				else
				{
					pThis->m_HomingStep = HomingStep::ALARM;
				}
			}
			break;

		case HomingStep::SLAVEAXISSERVO_OFF:
			if (pThis->m_pAxis->CheckSlaveServoOn() == true)
			{
				err = pThis->m_pAxis->SlaveServoOff();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d SlaveServoOff Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_HomingStep = HomingStep::ALARM;
			}
			break;

		case HomingStep::MINUSLIMIT:
			err = pThis->m_pAxis->GetHWLimitParam(limitParam);
			if (err != ErrorCode::None)
			{
				pThis->SetHomingErr(err);
				strErrMsg.Format(_T("[PWR] HomigStep:%d GetHWLimitParam Error:%d"), pThis->m_HomingStep, err);
				pThis->m_HomingStep = HomingStep::SELFQUIT;
				break;
			}
			err = pThis->m_pAxis->SetHWLimitParam(limitParam, WMX3AxisLimitSwitchType::Dec, WMX3AxisLimitSwitchType::Dec);
			if (err != ErrorCode::None)
			{
				pThis->SetHomingErr(err);
				strErrMsg.Format(_T("[PWR] HomigStep:%d SetHWLimitParam Error:%d"), pThis->m_HomingStep, err);
				pThis->m_HomingStep = HomingStep::SELFQUIT;
				break;
			}
			pThis->m_HomingStep = HomingStep::ALARM;
			break;

		case HomingStep::ALARM:
			if (pThis->m_pAxis->CheckAmpAlarm() == true)
			{
				err = pThis->m_pAxis->ClearAmpAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearAmpAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
				err = pThis->m_pAxis->ClearAxisAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearAxisAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				if (pThis->m_pAxis->GetUseSlaveAxis() == true) // Use Master and Slave
				{
					pThis->m_HomingStep = HomingStep::SLAVE_ALARM;
				}
				else
				{
					pThis->m_HomingStep = HomingStep::AXISSERVO_ON;
				}
			}
			break;

		case HomingStep::SLAVE_ALARM:
			if (pThis->m_pAxis->CheckSlaveAmpAlarm() == true)
			{
				err = pThis->m_pAxis->ClearSlaveAmpAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearSlaveAmpAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
				err = pThis->m_pAxis->ClearSlaveAxisAlarm();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ClearSlaveAxisAlarm Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_HomingStep = HomingStep::AXISSERVO_ON;
			}
			break;

		case HomingStep::AXISSERVO_ON:
			if (pThis->m_pAxis->CheckServoOn() == false)
			{
				err = pThis->m_pAxis->ServoOn();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d ServoOn Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				if (pThis->m_pAxis->GetUseSlaveAxis() == true) // Use Master and Slave
				{
					pThis->m_HomingStep = HomingStep::SLAVEAXISSERVO_ON;
				}
				else
				{
					pThis->m_HomingStep = HomingStep::ISIDLE;
				}
			}
			break;

		case HomingStep::SLAVEAXISSERVO_ON:
			if (pThis->m_pAxis->CheckSlaveServoOn() == false)
			{
				err = pThis->m_pAxis->SlaveServoOn();
				if (err != ErrorCode::None)
				{
					pThis->SetHomingErr(err);
					strErrMsg.Format(_T("[PWR] HomigStep:%d SlaveServoOn Error:%d"), pThis->m_HomingStep, err);
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_HomingStep = HomingStep::ISIDLE;
			}
			break;

		case HomingStep::ISIDLE:
			if (pThis->m_pAxis->IsAxisIdle() == true)
			{
				if (pThis->m_pAxis->GetUseSlaveAxis() == true)
				{
					pThis->m_HomingStep = HomingStep::SLAVE_ISIDLE;
				}
				else if (pThis->m_pAxis->IsConveyorAxis() == true)
				{
					pThis->m_HomingStep = HomingStep::LOAD_POSITION;
				}
				else
				{
					pThis->m_HomingStep = HomingStep::SEARCH_ORIGIN;
				}
			}
			break;

		case HomingStep::LOAD_POSITION:
			pThis->m_pAxis->GetHomeParam(homeParam);
			if (pThis->m_pAxis->GetForceOriginOffset() > 0.0)
			{
				CString strZ = GetPusherZName(FRONT_CONV);
				long puserZErr = NO_ERR;
				puserZErr = PrepareMoveConvWidth();

				if (puserZErr != NO_ERR)
				{
					pThis->m_pAxis->SetInitializeFail(true);
					SendAlarm(HOMING_TIMOUT(pThis->m_pAxis->GetAxisIndex()), _T("TimeOut PusherZ Move Standby"));
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}

				LoadPosition = pThis->m_pAxis->GetForceOriginOffset();
				homeParam->homeType = Config::HomeType::LSReverseZPulse;
				homeParam->homeDirection = Config::HomeDirection::Positive;
				pThis->m_pAxis->SetForceOriginOffset(0.000);

			}
			else
			{
				LoadPosition = gcPowerConveyorData->GetWidth(FRONT_CONV, WORK1_CONV);
				TRACE(_T("[PWR] Load Conveyor Width:%.1f\n"), LoadPosition);
				bSkipConvWidthHome = true;
				homeParam->homeType = Config::HomeType::CurrentPos;
			}
			LoadPulse = pThis->m_pAxis->GetUnResol() * LoadPosition;
			homeParam->homePosition = LoadPulse;
			////	if (IsExistAll(ENTRY_CONV) == true || IsExistAll(WORK1_CONV) == true || IsExistAll(EXIT_CONV) == true)
			//	{
			//		homeParam->homeType = Config::HomeType::CurrentPos;
			//		bSkipConvWidthHome = true;
			//	}
			pThis->m_pAxis->SetHomeParam(homeParam);
			pThis->m_HomingStep = HomingStep::SEARCH_ORIGIN;
			break;

		case HomingStep::SLAVE_ISIDLE:
			if (pThis->m_pAxis->IsSlaveAxisIdle() == true)
			{
				pThis->m_HomingStep = HomingStep::SYNC_ON;
			}
			else
			{
				TRACE(_T("[PWR] HomingStep Slave axis(%d,%d) is not IDLE\n"), pThis->m_pAxis->GetSlaveAxisIndex(), pThis->m_pAxis->GetSlaveAxisSlaveID());
			}
			break;

		case HomingStep::SYNC_ON:
			err = pThis->m_pAxis->SetSyncMasterSlave();
			if (err != ErrorCode::None)
			{
				pThis->SetHomingErr(err);
				strErrMsg.Format(_T("[PWR] HomigStep:%d SetSyncMasterSlave Error:%d"), pThis->m_HomingStep, err);
				pThis->m_HomingStep = HomingStep::SELFQUIT;
				break;
			}
			else
			{
				pThis->m_HomingStep = HomingStep::CHECK_HOME_DISTANCE;
				ThreadSleep(TIME100MS);
			}
			break;
		case HomingStep::CHECK_HOME_DISTANCE:
			HomeShift = gcPowerCalibrationData->GetHomeShiftDistance(FRONT_GANTRY);
			//if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d Before StartHome(%s) HomeShift:%.3f\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), HomeShift * pThis->m_pAxis->GetResol());
			}
			if (abs(HomeShift) < EPSILON)
			{
				//if (gcPowerLog->IsShowHomingLog() == true)
				{
					TRACE(_T("[PWR] %s Home 1st Shift Distance is zero\n"), pThis->m_pAxis->GetAxisName());
				}
				pThis->m_HomingStep = HomingStep::CHANGE_HOME_PARA;
			}
			else
			{
				//if (gcPowerLog->IsShowHomingLog() == true)
				{
					TRACE(_T("[PWR] %s Home 1st Shift Distance is %.3f\n"), 
						pThis->m_pAxis->GetAxisName(), HomeShift * pThis->m_pAxis->GetResol());
				}
				pThis->m_HomingStep = HomingStep::CHECK_HOME_PARA;
			}
			break;

		case HomingStep::CHANGE_HOME_PARA:
			pThis->m_pAxis->GetHomeParam(homeParam);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d StartHome(%s) (1) Use Slave Z Pulse(%s) RetainSyncOffset(%s) Err:%d\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(),
					homeParam->gantryHomingUseSlaveZPulse == true ? _T("Enable") : _T("Disable"),
					homeParam->gantryHomingRetainSyncOffset == true ? _T("Enable") : _T("Disable"),
					err);
			}
			homeParam->gantryHomingUseSlaveZPulse = true;
			homeParam->gantryHomingRetainSyncOffset = true;
			homeParam->immediateStopAtLS = true;
			pThis->m_pAxis->SetHomeParam(homeParam);
			pThis->m_pAxis->GetHomeParam(homeParam);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d StartHome(%s) (2) Use Slave Z Pulse(%s) RetainSyncOffset(%s) Err:%d\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(),
					homeParam->gantryHomingUseSlaveZPulse == true ? _T("Enable") : _T("Disable"),
					homeParam->gantryHomingRetainSyncOffset == true ? _T("Enable") : _T("Disable"),
					err);
			}
			pThis->m_HomingStep = HomingStep::SEARCH_ORIGIN;
			break;

		case HomingStep::CHECK_HOME_PARA:
			pThis->m_pAxis->GetHomeParam(homeParam);
			pThis->m_pAxis->GetSlaveHomeParam(slaveHomeParam);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d StartHome(%s) (3) Use Slave Z Pulse(%s) RetainSyncOffset(%s) Err:%d\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(),
					homeParam->gantryHomingUseSlaveZPulse == true ? _T("Enable") : _T("Disable"),
					homeParam->gantryHomingRetainSyncOffset == true ? _T("Enable") : _T("Disable"),
					err);
			}
			homeParam->gantryHomingUseSlaveZPulse = true;
			homeParam->gantryHomingRetainSyncOffset = false;
			homeParam->immediateStopAtLS = true;
			if (homeParam->homeDirection == 1) // Negative
			{
				HomeShift = HomeShift * -1;
			}
			else
			{
				HomeShift = HomeShift * 1;
			}
			slaveHomeParam->homeShiftDistance = HomeShift;
			//if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomingStep:%d(%s) HomeShiftDistance:%.3f\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), HomeShift * pThis->m_pAxis->GetResol());
			}
			pThis->m_pAxis->SetHomeParam(homeParam);
			pThis->m_pAxis->SetSlaveHomeParam(slaveHomeParam);
			pThis->m_pAxis->GetHomeParam(homeParam);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d StartHome(%s) (4) Use Slave Z Pulse(%s) RetainSyncOffset(%s) Err:%d\n"),
					pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(),
					homeParam->gantryHomingUseSlaveZPulse == true ? _T("Enable") : _T("Disable"),
					homeParam->gantryHomingRetainSyncOffset == true ? _T("Enable") : _T("Disable"),
					err);
			}
			pThis->m_HomingStep = HomingStep::SEARCH_ORIGIN;
			break;

		case HomingStep::SEARCH_ORIGIN:
			pThis->m_pAxis->GetHomeParam(homeParam);
			if (pThis->m_pAxis->IsConveyorAxis() == true)
			{
				if (bSkipConvWidthHome == false)
				{
					LoadPosition = HomePosition = ReadHomePosition(pThis->m_pAxis->GetAxisName());
				}
				else
				{
					HomePosition = gcPowerConveyorData->GetWidth(FRONT_GANTRY, WORK1_CONV);
				}
				TRACE(_T("[PWR] Conveyor Get Home Position:%.3f\n"), HomePosition * pThis->m_pAxis->GetUnResol());
			}
			else
			{
				HomePosition = ReadHomePosition(pThis->m_pAxis->GetAxisName());
			}
			homeParam->homePosition = HomePosition * pThis->m_pAxis->GetUnResol();
			pThis->m_pAxis->SetHomeParam(homeParam);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d SetHomeParam(%s)(%.3f %.3f) Err:%d\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), HomePosition,
					pThis->m_pAxis->GetUnResol(), err);
			}
			err = pThis->m_pAxis->StartHome();
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d StartHome(%s)(%.3f) Err:%d\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), homeParam->homePosition, err);
			}
			if (pThis->m_pAxis->GetMotorType() == CMotorType::StepMotor || homeParam->homeType == Config::HomeType::CurrentPos)
			{
				pThis->m_HomingStep = HomingStep::HOME_DONE;
			}
			else
			{
				pThis->m_HomingStep = HomingStep::IS_HOMING;
			}
			break;

		case HomingStep::IS_HOMING:
			if (pThis->m_pAxis->IsAxisHoming() == true)
			{
				if (gcPowerLog->IsShowHomingLog() == true)
				{
					TRACE(_T("[PWR] HomigStep:%d Homing(%s)\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName());
				}
				GetHomingTime = _time_get();
				pThis->m_HomingStep = HomingStep::HOME_DONE;
			}
			break;

		case HomingStep::HOME_DONE:
			if (pThis->m_pAxis->IsAxisHomeDone() == true)
			{
				if (pThis->m_pAxis->IsConveyorAxis() == true)
				{
					gcPowerConveyorData->SetWidth(FRONT_GANTRY, WORK1_CONV, LoadPosition);
				}
				else if (pThis->m_pAxis->GetUseSlaveAxis() == true)
				{
					ThreadSleep(THREAD_HOMING_DONE_WAITTIME);
					if (gcPowerLog->IsShowHomingLog() == true)
					{
						TRACE(_T("[PWR] HomigStep:%d Home Done(%s) Torque(%f,%f) Shift(%.3f)\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), pThis->m_pAxis->ReadActualTorque(),
							pThis->m_pAxis->ReadSlaveActualTorque(),
							gcPowerCalibrationData->GetHomeShiftDistance(FRONT_GANTRY));
					}
				}
				else
				{
					if (gcPowerLog->IsShowHomingLog() == true)
					{
						TRACE(_T("[PWR] HomigStep:%d Home Done(%s) Torque(%f)\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), pThis->m_pAxis->ReadActualTorque());
					}
				}
				if (pThis->m_pAxis->GetUseSlaveAxis() == true)
				{
					HomeShift = gcPowerCalibrationData->GetHomeShiftDistance(FRONT_GANTRY);
					if (gcPowerLog->IsShowHomingLog() == true)
					{
						TRACE(_T("[PWR] HomigStep:%d Before StartHome(%s) HomeShift:%.3f\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), HomeShift);
					}

					ThreadSleep(TIME100MS);
					Y1Encoder = pThis->m_pAxis->GetEncoderFeedBack();
					Y2Encoder = pThis->m_pAxis->GetSlaveEncoderFeedBack();
					Y1Y2Diff = Y1Encoder - Y2Encoder;
					TRACE(_T("[PWR] %s Y1Encoder:%d Y2Encoder:%d Diff:%d \n"), pThis->m_pAxis->GetAxisName(), Y1Encoder, Y2Encoder, Y1Y2Diff);
					gcPowerCalibrationData->SetYHomeEncoderDifferent(Y1Y2Diff);

					Y2SHIFT_AUTOINIT y2ShiftAutoInit = GetY2ShiftAutoInit();
					double torqueY1 = 0.0;
					double torqueY2 = 0.0;
					bool torqueOver = false;

					if (y2ShiftAutoInit.Use == true && firstTimeShiftInit == true)
					{
						firstTimeShiftInit = false;
						if (Get1DCompensationUse() == true)
						{
							oneDCompensationOn();
						}

						ThreadSleep(TIME500MS);

						for (long readCnt = 0; readCnt < 10; readCnt++)
						{
							torqueY1 = pThis->m_pAxis->ReadActualTorque();
							torqueY2 = pThis->m_pAxis->ReadSlaveActualTorque();

							if (abs(torqueY1) > y2ShiftAutoInit.TorqueLimit || abs(torqueY2) > y2ShiftAutoInit.TorqueLimit)
							{
								TRACE(_T("[PWR] %s Torque Over & Auto Init Start. (Torque Y1:%.1f, Y2:%.1f)\n"), pThis->m_pAxis->GetAxisName(), torqueY1, torqueY2);

								torqueOver = true;
								break;
							}
							else
							{
								TRACE(_T("[PWR] %s Torque pass(%d). (Torque Y1:%.1f, Y2:%.1f)\n"), pThis->m_pAxis->GetAxisName(), readCnt, torqueY1, torqueY2);
							}
							ThreadSleep(TIME10MS);
						}

					}

					if (torqueOver == true)
					{
						if (Get1DCompensationUse() == true)
						{
							oneDCompensationOff();
						}

						ThreadSleep(TIME500MS);

						ClearY2Shift(FRONT_GANTRY);
						pThis->m_HomingStep = HomingStep::INIT;
						TRACE(_T("[PWR] %s Torque Over ClearY2Shift\n"), pThis->m_pAxis->GetAxisName());
					}
					else if (abs(HomeShift) < EPSILON && firstTimeCatchShift == true)
					{
						firstTimeCatchShift = false;
						if (gcPowerLog->IsShowHomingLog() == true)
						{
							TRACE(_T("[PWR] %s Home 2nd Shift Distance is zero\n"), pThis->m_pAxis->GetAxisName());
						}
						pThis->m_HomingStep = HomingStep::CATCH_HOME_SHIFT;
					}
					else
					{
						if (gcPowerLog->IsShowHomingLog() == true)
						{
							TRACE(_T("[PWR] %s Home 2nd Shift Distance is %.3f\n"), pThis->m_pAxis->GetAxisName(), HomeShift);
						}
						pThis->m_pAxis->SetInitializeEnd(true);
						pThis->m_HomingStep = HomingStep::SELFQUIT;
					}
				}
				else
				{
					pThis->m_pAxis->SetInitializeEnd(true);
					pThis->m_HomingStep = HomingStep::MOVE_LAST_POSITION;
				}
			}
			else
			{
				HomingElapsed = _time_elapsed(GetTime);
				if (HomingElapsed > pThis->m_pAxis->GetHomingMaxTimeOut())
				{
					err = pThis->m_pAxis->StopMotion();
					pThis->m_pAxis->SetInitializeFail(true);
					SendAlarm(HOMING_TIMOUT(pThis->m_pAxis->GetAxisIndex()), _T("TimeOut Origin Search"));
					pThis->m_HomingStep = HomingStep::SELFQUIT;
					break;
				}
				if (gcPowerLog->IsShowHomingLog() == true)
				{
					TRACE(_T("[PWR] HomigStep:%d Home Done Yet(%s)\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName());
				}
			}
			break;

		case HomingStep::CATCH_HOME_SHIFT:
			HomeShift = pThis->m_pAxis->GetHomeDiffZPulse();
			//if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] %s DiffZPulse:%.3f\n"), pThis->m_pAxis->GetAxisName(), HomeShift * pThis->m_pAxis->GetResol());
			}
			gcPowerCalibrationData->SetHomeShiftDistance(FRONT_GANTRY, HomeShift);
			gcPowerCalibrationData->WriteHomeShiftDistance(FRONT_GANTRY);

			TRACE(_T("[PWR] %s Homing retry for HomeShiftDistnace\n"), pThis->m_pAxis->GetAxisName());
			pThis->m_HomingStep = HomingStep::INIT;

			//pThis->m_pAxis->SetInitializeEnd(true);
			//pThis->m_HomingStep = HomingStep::SELFQUIT;
			break;

		case HomingStep::SELF_HOME_DONE:
			pThis->m_pAxis->SetInitializeEnd(true);
			if (gcPowerLog->IsShowHomingLog() == true)
			{
				TRACE(_T("[PWR] HomigStep:%d Home Self Done(%s)\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName());
			}
			pThis->m_HomingStep = HomingStep::SELFQUIT;
			break;

		case HomingStep::MOVE_LAST_POSITION:
			if (pThis->m_pAxis->IsPusherZAxis() == true)
			{
				//PusherZ = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
				//if (gcPowerLog->IsShowHomingLog() == true)
				//{
				//	TRACE(_T("[PWR] HomingStep::%d %s Move recently Position:%.3f"), pThis->m_HomingStep, pThis->GetAxisName(), PusherZ);
				//}
				//err = StartPosWaitDelayedInposition(pThis->GetAxisName(), Ratio, TimeOut, PusherZ, Inpos, Ms, true);
				//if (err != NO_ERR)
				//{
				//	TRACE(_T("[PWR] HomigStep:%d (%s) MOVE_LAST_POSITION StartPosWaitDelayedInposition Err:%d\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), err);
				//}
			}
			else if (pThis->m_pAxis->IsRAxis() == true)
			{
				err = StartPosWaitDelayedInposition(pThis->GetAxisName(), Ratio, TimeOut, 0.000, Inpos, Ms, true);
				if (err != NO_ERR)
				{
					TRACE(_T("[PWR] HomigStep:%d (%s) MOVE_LAST_POSITION StartPosWaitDelayedInposition Err:%d\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), err);
				}
			}
			else if (pThis->m_pAxis->IsZAxis() == true)
			{
				err = StartPosWaitDelayedInposition(pThis->GetAxisName(), Ratio, TimeOut, GetTorqueOverSafetyZ(), Inpos, Ms, true);
				if (err != NO_ERR)
				{
					TRACE(_T("[PWR] HomigStep:%d (%s) MOVE_LAST_POSITION StartPosWaitDelayedInposition Err:%d\n"), pThis->m_HomingStep, pThis->m_pAxis->GetAxisName(), err);
				}
			}
			else if (pThis->m_pAxis->IsConveyorAxis() == true)
			{
				TRACE(_T("[PWR] Conveyor CurWidth:%.1f\n"), ReadPosition(pThis->GetAxisName()));
			}
			pThis->m_HomingStep = HomingStep::SELFQUIT;
			break;

		case HomingStep::SELFQUIT:
			bLoop = false;
			break;

		case HomingStep::MAX:
			Sleep(TIME100MS);
			break;

		default:
			break;
		}
		ThreadSleep(THREAD_ORIGIN_SEARCH_TIME);
		if (bLoop == false)
		{
			break;
		}
	}
	//pThis->SetMsgQueueStatus(INITIALZE);
	//SetEvent(pThis->GetThreadTerminate());
	if (gcPowerLog->IsShowHomingLog() == true)
	{
		TRACE(_T("[PWR] COriginSearch(%d) ID:0x%04X(%s) HomeStep:%d Quit\n"), pThis->m_pAxis->GetAxisIndex(),
			pThis->GetThreadID(), pThis->GetThreadName(), pThis->m_HomingStep);
	}
	delete homeParam;
	delete slaveHomeParam;
	delete limitParam;
	//AddCleanThread(pThis);
	return 0;
}

HomingStep COriginSearch::GetStartHomingStep()
{
	int err = ErrorCode::None;
	CString strErrMsg;
	HomingStep nStartStep = HomingStep::INIT;
	if (m_pAxis->GetAxisIndex() == static_cast<int>(PowerAxis::FY2) || m_pAxis->GetAxisIndex() == static_cast<int>(PowerAxis::RY2))
	{
		err = m_pAxis->Disable1D();
		if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsDisabled)
		{
			TRACE(_T("[PWR] Disable1D(%s) Err:%d\n"), m_pAxis->GetAxisName(), err);
			SetThreadStatusError(true);
			nStartStep = HomingStep::SELFQUIT;
		}
		else
		{
			nStartStep = HomingStep::SELF_HOME_DONE;
		}
	}
	return nStartStep;
}

