#include "pch.h"
#include "CStart1D.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "CPowerCalibrationData.h"
#include "CMachineFile.h"
#include "Trace.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "CPowerLog.h"
//#include "ErrorCode.h"

CStart1D* gcStart1D;
CStart1D::CStart1D()
{
	ClearStep();
	Clear1DCompen();
	Set1DCompenMode(true);
	m_Gantry = FRONT_GANTRY;
	m_StrX = GetAxisX(m_Gantry);
	m_StrY1 = GetAxisY1(m_Gantry);
	m_StrY2 = GetAxisY2(m_Gantry);
}

CStart1D::CStart1D(HANDLE h_Terminate)
{
	ClearStep();
	Clear1DCompen();
	Set1DCompenMode(true);
	m_Gantry = FRONT_GANTRY;
	m_StrX = GetAxisX(m_Gantry);
	m_StrY1 = GetAxisY1(m_Gantry);
	m_StrY2 = GetAxisY2(m_Gantry);
}

CStart1D::~CStart1D()
{
	Clear1DCompen();
}

void CStart1D::ClearStep()
{
	m_1DStep = Calibration1DStep::INIT;
}

void CStart1D::Clear1DCompen()
{
	ZeroMemory(dblMean, sizeof(dblMean));
	ZeroMemory(dblPlusCompen, sizeof(dblPlusCompen));
	ZeroMemory(dblMinusCompen, sizeof(dblMinusCompen));
	ZeroMemory(dblPlusTorque, sizeof(dblPlusTorque));
	ZeroMemory(dblMinusTorque, sizeof(dblMinusTorque));
}

void CStart1D::Set1DCompenMode(bool bApply)
{
	m_bApply = bApply;
	TRACE(_T("[PWR] Set1DCompenMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStart1D::Get1DCompenMode()
{
	return m_bApply;
}

void CStart1D::SetPlus1DDiff(unsigned indx, double dblDiff)
{
	if (Get1DCompenMode() == true)
	{
		dblPlusCompen[indx] = dblDiff;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetPlus1DDiff(%d,%.3f)"), indx, dblPlusCompen[indx]);
		}
	}
}

void CStart1D::SetMinus1DDiff(unsigned indx, double dblDiff)
{
	if (Get1DCompenMode() == true)
	{
		dblMinusCompen[indx] = dblDiff;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetMinus1DDiff(%d,%.3f)"), indx, dblMinusCompen[indx]);
		}
	}
}

void CStart1D::SetPlusTorque(unsigned indx, double dblDiff)
{
	dblPlusTorque[indx] = dblDiff;
}

void CStart1D::SetMinusTorque(unsigned indx, double dblDiff)
{
	dblMinusTorque[indx] = dblDiff;
}

void CStart1D::Show1DMeanData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_1D_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Mean Diff %.3f\n"), indx, dblMean[indx]);
	}
}

void CStart1D::Show1DRawData()
{
	for (unsigned indx = CAL_1D_INIT; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Plus,Minus Diff %.3f,%.3f=%.3f\n"), indx, dblPlusCompen[indx], dblMinusCompen[indx], dblPlusCompen[indx] - dblMinusCompen[indx]);
	}
}

void CStart1D::Show1DRawData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_1D_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Plus,Minus Diff %.3f,%.3f=%.3f\n"), indx, dblPlusCompen[indx], dblMinusCompen[indx], dblPlusCompen[indx] - dblMinusCompen[indx]);
	}
}

void CStart1D::ShowTorqueData()
{
	for (unsigned indx = CAL_1D_INIT; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Plus,Minus Torque %.3f,%.3f=%.3f\n"), indx, dblPlusTorque[indx], dblMinusTorque[indx], dblPlusTorque[indx] - dblMinusTorque[indx]);
	}
}

void CStart1D::ShowTorqueData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_1D_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Plus,Minus Torque %.3f,%.3f=%.3f\n"), indx, dblPlusTorque[indx], dblMinusTorque[indx], dblPlusTorque[indx] - dblMinusTorque[indx]);
	}
}

int CStart1D::CopyGlobalCompensationData()
{
	for (long indx = 0; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		dblMean[indx] = ((dblPlusCompen[indx] + dblMinusCompen[indx]) / 2);
	}
	for (long indx = 0; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		gcPowerCalibrationData->Set1DCompensationData(m_Gantry, indx, dblMean[indx]);
	}
	return 0;
}

int CStart1D::CopyGlobalCompensationData(long MaxCnt)
{
	int indx = 0;
	double Init = 0.0;
	for (indx = CAL_1D_INIT; indx < MaxCnt; ++indx)
	{
		dblMean[indx] = ((dblPlusCompen[indx] + dblMinusCompen[indx]) / 2);
		Init = dblMean[indx];
	}
	//Init = dblMean[MaxCnt - 1];
	for (indx = MaxCnt; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		dblMean[indx] = Init;
	}
	for (indx = 0; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		gcPowerCalibrationData->Set1DCompensationData(m_Gantry, indx, dblMean[indx]);
	}
	return 0;
}

int CStart1D::PasteGlobalCompensationData()
{
	return 0;
}

UINT CStart1D::Start1DCalibration(LPVOID wParam)
{
	CStart1D* pThis = reinterpret_cast<CStart1D*>(wParam);
	bool bLoop = true, bRet = false;
	unsigned nCompenCount = 0;
	int err = ErrorCode::None;
	long lTimeChk = 0, Err = NO_ERR, Gantry = pThis->m_Gantry, nCompenMax = 0;
	double dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff, dblPositiveLimit, dblTargetX, dblTargetY;
	double StartPos = 0.0;
	Calibration1DStep nOld1DStep = Calibration1DStep::MAX;
	signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg, strErrMsg;
	Limit limit;
	CApplicationTime* pTime = new CApplicationTime();
	dblTargetX = dblTargetY = dblPositionY1 = dblPositionY2 = dblTorqueY1 = dblTorqueY2 = dblDiff = 0.0;
	limit = GetLimit(GetAxisIndexFromAliasName(pThis->m_StrY1));
	dblPositiveLimit = limit.plus * GetUnResol(pThis->m_StrY1); //ReadVirtualPositiveLimit(pThis->m_StrY1);		
	nCompenMax = (int)(dblPositiveLimit / CAL_1D_PITCH_PULSE) - 1;
	//nCompenMax = 10; // TEST
	TRACE(_T("[PWR] Y1 Positive Limit:%.1f Pitch:%d Max Count:%d\n"), dblPositiveLimit, CAL_1D_PITCH_PULSE, nCompenMax);
	if (nCompenMax < 0)
	{
		Err = UNDERZERO_1DCOUNT;
		TRACE(_T("[PWR] Start1DCalibration CompenMax under 0 Err:%d\n"), Err);
		return Err;
	}
	Point_XY xy;
	ZeroMemory(&xy, sizeof(xy));
	xy.x = xy.y = 0.0;
    double Ratio = 0.5, Inpos = 0.010;
    long TimeOut = TIME10000MS, Ms = TIME300MS;
    if (pThis->Get1DCompenMode() == false)
    {
        Ratio = 0.5;
    }
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, TBL_CAMERA, xy, Ratio, Inpos, Ms, TimeOut);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Start1DCalibration LinearIntplPosWaitDelayedPosSet Err:%d\n"));
		return Err;
	}
    while (bLoop)
    {
		if (nOld1DStep != pThis->m_1DStep)
		{
			lTimeChk = pTime->TimeElapsed();
			if (gcPowerLog->IsShowCalibrationLog() == true)
			{
				TRACE(_T("[PWR] Start1DCalibration Axis(%s) Step:%d Time:%d[ms]\n"), _T("FY1"), pThis->m_1DStep, lTimeChk);
			}
			nOld1DStep = pThis->m_1DStep;
			pTime->TimeGet();
		}
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_1D_CALIBRATION_READTIME);
        //    continue;
        //}
		switch (pThis->m_1DStep)
		{
		case Calibration1DStep::INIT:
			nCompenCount = 0;
			pThis->m_1DStep = Calibration1DStep::START;
			break;

		case Calibration1DStep::START:
			pThis->m_1DStep = Calibration1DStep::CHECK_ORIGIN;
			break;

		case Calibration1DStep::CHECK_ORIGIN:
			if (IsOneAxisHomingComplete(pThis->m_StrX) == true && IsOneAxisHomingComplete(pThis->m_StrY1) == true)
			{
				pThis->m_1DStep = Calibration1DStep::CLEARALARM;
			}
			else
			{
				TRACE(_T("[PWR] CStart1D Homing Failed %s(%d) %s(%d)\n"), pThis->m_StrX, IsOneAxisHomingComplete(pThis->m_StrX), pThis->m_StrY1, IsOneAxisHomingComplete(pThis->m_StrY1));
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			break;

		case Calibration1DStep::CLEARALARM:
			if (CheckAmpAlarm(pThis->m_StrX) == true || CheckAmpAlarm(pThis->m_StrY1) == true)
			{
				bRet = AlarmClear(pThis->m_StrX);
				if (bRet == false)
				{
					strErrMsg.Format(_T("[PWR] CStart1D:%d AlarmClear Error:%d"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
				bRet = AlarmClear(pThis->m_StrY1);
				if (bRet == false)
				{
					strErrMsg.Format(_T("[PWR] CStart1D:%d AlarmClear Error:%d"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::AXISSERVO_OFF;
			}
			break;

		case Calibration1DStep::AXISSERVO_OFF:
			if (CheckServoOn(pThis->m_StrY1) == true)
			{
				bRet = ServoOff(pThis->m_StrY1);
				if (bRet == false)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d ServoOff Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::SLAVEAXISSERVO_OFF;
			}
			break;

		case Calibration1DStep::SLAVEAXISSERVO_OFF:
			if (CheckSlaveServoOn(pThis->m_StrY1) == true)
			{
				bRet = SlaveServoOff(pThis->m_StrY1);
				if (bRet == false)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d SlaveServoOff Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::ALARM;
			}
			break;

		case Calibration1DStep::ALARM:
			if (CheckAmpAlarm(pThis->m_StrY1) == true)
			{
				if(AlarmClear(pThis->m_StrY1) == false)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d ClearAmpAlarm Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::SLAVE_ALARM;
			}
			break;

		case Calibration1DStep::SLAVE_ALARM:
			if (CheckSlaveAmpAlarm(pThis->m_StrY1) == true)
			{
				if(ClearSlaveAmpAlarm(pThis->m_StrY1) == false)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d ClearSlaveAmpAlarm Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::DISABLE_COMPEN_1ST;
			}
			break;

		case Calibration1DStep::DISABLE_COMPEN_1ST:
			gDisable1DCompensation();
			ThreadSleep(TIME1000MS);
			twoDCompensationOff();
			ThreadSleep(TIME1000MS);
			pThis->m_1DStep = Calibration1DStep::AXISSERVO_ON;
			break;

		case Calibration1DStep::AXISSERVO_ON:
			if (CheckServoOn(pThis->m_StrY1) == false)
			{
				bRet = ServoOn(pThis->m_StrY1);
				if (bRet == false)
				{
					TRACE(_T("[PWR] HomigStep:%d ServoOn Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::ISIDLE;
			}
			break;

		case Calibration1DStep::ISIDLE:
			Err = WaitOneIdle(pThis->m_StrY1, TIME3000MS);
			if (Err == NO_ERR)
			{
				pThis->m_1DStep = Calibration1DStep::SLAVE_OFF;
			}
			break;

		case Calibration1DStep::SLAVE_OFF:
			if (CheckSlaveServoOn(pThis->m_StrY1) == false)
			{
				if (pThis->Get1DCompenMode() == true)
				{
					pThis->m_1DStep = Calibration1DStep::MOVE_ORIGIN;
				}
				else
				{
					pThis->m_1DStep = Calibration1DStep::SLAVEAXISSERVO_ON;
				}				
			}
			else
			{
				TRACE(_T("[PWR] Calibration1DStep Slave axis(%d,%d) is not Off\n"), GetSlaveAxisIndex(pThis->m_StrY1), GetSlaveAxisSlaveID(pThis->m_StrY1));
			}
			break;

		case Calibration1DStep::MOVE_ORIGIN:
			Err = StartPosWaitDelayedInpositionWihtoutSlave(pThis->m_StrY1, Ratio, TimeOut, StartPos, Inpos, Ms, true);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d StartPosWaitDelayedInpositionWihtoutSlave StartPos Error:%d\n"), pThis->m_1DStep, Err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			dblTorqueY1 = ReadActualTorque(pThis->m_StrY1);
			dblTorqueY2 = ReadActualTorque(pThis->m_StrY2);
			dblPositionY1 = ReadPosition(pThis->m_StrY1);
			dblPositionY2 = ReadPosition(pThis->m_StrY2);
			dblDiff = dblPositionY2 - dblPositionY1;
			if (abs(dblDiff) < MAX_1D_VALID_RANGE)
			{
				pThis->SetPlus1DDiff(nCompenCount, dblDiff);
				pThis->SetPlusTorque(nCompenCount, dblTorqueY2);
				TRACE(_T("[PWR] Position(+)(%04d)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f\n"), nCompenCount,
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			else
			{
				TRACE(_T("[PWR] Position(+)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f(Range Over)\n"),
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			nCompenCount++;
			pThis->m_1DStep = Calibration1DStep::FIND_PLUS_COMPEN;
			break;

		case Calibration1DStep::FIND_PLUS_COMPEN:
			dblTargetY = (double)(CAL_1D_PITCH_POSITION * 1.0);
			Err = StartMoveWaitDelayedInpositionWithoutSlave(pThis->m_StrY1, Ratio, TimeOut, dblTargetY, Inpos, Ms, true);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d StartMoveWaitDelayedInpositionWithoutSlave (+) Error:%d\n"), pThis->m_1DStep, Err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			pThis->m_1DStep = Calibration1DStep::CATCH_Y2_PLUS_COMPEN;
			ThreadSleep(THREAD_1D_CATCH_DELAY);
			break;

		case Calibration1DStep::CATCH_Y2_PLUS_COMPEN:
			dblTorqueY1 = ReadActualTorque(pThis->m_StrY1);
			dblTorqueY2 = ReadActualTorque(pThis->m_StrY2);
			dblPositionY1 = ReadPosition(pThis->m_StrY1);
			dblPositionY2 = ReadPosition(pThis->m_StrY2);
			dblDiff = dblPositionY2 - dblPositionY1;
			if (abs(dblDiff) < MAX_1D_VALID_RANGE)
			{
				pThis->SetPlus1DDiff(nCompenCount, dblDiff);
				pThis->SetPlusTorque(nCompenCount, dblTorqueY2);
				TRACE(_T("[PWR] Position(+)(%04d)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f\n"), nCompenCount,
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			else
			{
				TRACE(_T("[PWR] Position(+)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f(Range Over)\n"),
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			pThis->m_1DStep = Calibration1DStep::CHECK_PLUS_COMPEN_COUNT;
			break;

		case Calibration1DStep::CHECK_PLUS_COMPEN_COUNT:
			if ((long)nCompenCount >= (nCompenMax - 1))
			{
				pThis->m_1DStep = Calibration1DStep::CATCH_Y2_MINUS_COMPEN;
			}
			else
			{
				nCompenCount++;
				pThis->m_1DStep = Calibration1DStep::FIND_PLUS_COMPEN;
			}
			break;

		case Calibration1DStep::CATCH_Y2_MINUS_COMPEN:
			dblTorqueY1 = ReadActualTorque(pThis->m_StrY1);
			dblTorqueY2 = ReadActualTorque(pThis->m_StrY2);
			dblPositionY1 = ReadPosition(pThis->m_StrY1);
			dblPositionY2 = ReadPosition(pThis->m_StrY2);
			dblDiff = dblPositionY2 - dblPositionY1;
			if (abs(dblDiff) < MAX_1D_VALID_RANGE)
			{
				pThis->SetMinus1DDiff(nCompenCount, dblDiff);
				pThis->SetMinusTorque(nCompenCount, dblTorqueY2);
				TRACE(_T("[PWR] Position(-)(%04d)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f\n"), nCompenCount,
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			else
			{
				TRACE(_T("[PWR] Position(-)Y1:%.3f,Y2:%.3f TorqueY1:%.3f,Y2:%.3f Diff:%.3f(Range Over)\n"),
					dblPositionY1, dblPositionY2, dblTorqueY1, dblTorqueY2, dblDiff);
			}
			pThis->m_1DStep = Calibration1DStep::CHECK_MINUS_COMPEN_COUNT;
			break;

		case Calibration1DStep::CHECK_MINUS_COMPEN_COUNT:
			if (nCompenCount <= CAL_1D_INIT)
			{
				nCompenCount = 0;
				pThis->m_1DStep = Calibration1DStep::MOVE_STANDBY;
			}
			else
			{
				nCompenCount--;
				pThis->m_1DStep = Calibration1DStep::FIND_MINUS_COMPEN;
			}
			break;

		case Calibration1DStep::FIND_MINUS_COMPEN:
			dblTargetY = (double)(CAL_1D_PITCH_POSITION * -1.0);
			Err = StartMoveWaitDelayedInpositionWithoutSlave(pThis->m_StrY1, Ratio, TimeOut, dblTargetY, Inpos, Ms, true);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d StartMoveWaitDelayedInpositionWithoutSlave StartPos Error:%d\n"), pThis->m_1DStep, Err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			pThis->m_1DStep = Calibration1DStep::CATCH_Y2_MINUS_COMPEN;
			ThreadSleep(THREAD_1D_CATCH_DELAY);
			break;

		case Calibration1DStep::MOVE_STANDBY:
			dblTargetY = 0.0f;
			Err = StartPosWaitDelayedInpositionWihtoutSlave(pThis->m_StrY1, Ratio, TimeOut, dblTargetY, Inpos, Ms, true);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d StartPosWaitDelayedInpositionWihtoutSlave Error:%d\n"), pThis->m_1DStep, Err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			pThis->m_1DStep = Calibration1DStep::SHOW_1D;
			break;

		case Calibration1DStep::SHOW_1D:
			pThis->Show1DRawData(nCompenMax);
			pThis->ShowTorqueData(nCompenMax);
			if (pThis->Get1DCompenMode() == true)
			{
				pThis->m_1DStep = Calibration1DStep::SET_1D;
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
			}
			break;

		case Calibration1DStep::SET_1D:
			pThis->CopyGlobalCompensationData(nCompenMax);
			err = gSet1DCompensation();
			pThis->Show1DMeanData(nCompenMax);
			if (err != ErrorCode::None)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d Set1DCompensationData Error:%d\n"), pThis->m_1DStep, err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::DISABLE_1D;
			}
			break;

		case Calibration1DStep::DISABLE_1D:
			err = gDisable1DCompensation();
			if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsEnabled &&
				err != CompensationErrorCode::PitchErrorCompensationIsDisabled)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d Disable1DCompensationData Error:%d\n"), pThis->m_1DStep, err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::ENABLE_1D;
			}
			break;

		case Calibration1DStep::ENABLE_1D:
			err = gEnable1DCompensation();
			if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsEnabled)
			{
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::WRITEFILE_1D;
			}
			break;

		case Calibration1DStep::WRITEFILE_1D:
			if (gcPowerCalibrationData)
			{
				if (gcPowerCalibrationData->Write1DCompensationData(FRONT_GANTRY) == true)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d gcPowerCalibrationData Write1DCompensationData is complete\n"), pThis->m_1DStep);
					gcMachineFile->SaveFile();
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;				
				}
				else
				{
					TRACE(_T("[PWR] Calibration1DStep:%d gcPowerCalibrationData Write1DCompensationData is fail\n"), pThis->m_1DStep);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				TRACE(_T("[PWR] Calibration1DStep:%d gcPowerCalibrationData is null Error:%d\n"), pThis->m_1DStep, err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			break;

		case Calibration1DStep::SLAVEAXISSERVO_ON:
			if (CheckSlaveServoOn(pThis->m_StrY1) == false)
			{
				bRet = SlaveServoOn(pThis->m_StrY1);
				if (bRet == false)
				{
					TRACE(_T("[PWR] Calibration1DStep:%d SetSyncMasterSlave Error:%d\n"), pThis->m_1DStep, err);
					pThis->SetThreadStatusError(true);
					pThis->m_1DStep = Calibration1DStep::SELFQUIT;
					break;
				}
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::SYNC_ON;
			}
			break;

		case Calibration1DStep::SYNC_ON:
			bRet = SetSyncMasterSlave();
			if (bRet == false)
			{
				TRACE(_T("[PWR] Calibration1DStep:%d SetSyncMasterSlave Error:%d\n"), pThis->m_1DStep, err);
				pThis->SetThreadStatusError(true);
				pThis->m_1DStep = Calibration1DStep::SELFQUIT;
				break;
			}
			else
			{
				pThis->m_1DStep = Calibration1DStep::FIND_PLUS_COMPEN;
			}
			break;

		case Calibration1DStep::SELFQUIT:
			bLoop = false;
			break;

		case Calibration1DStep::MAX:
			Sleep(100);
			break;
		}
		if (pThis->IsThreadQuitMsg(strHostMsg) == false)
		{
			ZeroMemory(nArg, sizeof(nArg));
			strHostMsg.Empty();
		}
        ThreadSleep(THREAD_1D_CALIBRATION_READTIME);
    }
	//pThis->SetMsgQueueStatus(INITIALZE);
	SetEvent(pThis->GetThreadTerminate());
	InitOneRatio(pThis->m_StrX);
	InitOneRatio(pThis->m_StrY1);
	delete pTime;
	pTime = NULL;
	TRACE(_T("[PWR] CStart1D(0x%x) Quit\n"), pThis->GetThreadID());
	return 0;
}

void CStart1D::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
	SetThreadName(_T("th1DCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)Start1DCalibration;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] 1D Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] 1D Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
	gcPowerLog->Logging(strLog);
}

