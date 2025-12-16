#include "pch.h"
#include "CStartZCompen.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "CPowerCalibrationData.h"
#include "CMachineFile.h"
#include "Trace.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "CPowerLog.h"

CStartZCompen* gcStartZCompen;
CStartZCompen::CStartZCompen()
{
	m_strAxisZ.Empty();
	ClearStep();
	ClearZCompen();
	SetZCompenMode(true);
}

CStartZCompen::CStartZCompen(HANDLE h_Terminate)
{
	ClearStep();
	ClearZCompen();
	SetZCompenMode(true);
}

CStartZCompen::~CStartZCompen()
{
	ClearZCompen();
}

void CStartZCompen::SetZAxis(CString strAxisZ)
{
	m_strAxisZ = strAxisZ;
}

CString CStartZCompen::GetZAxis()
{
	CString strAxisZ = _T("NON");
	if (m_strAxisZ.IsEmpty() == false)
	{
		strAxisZ = m_strAxisZ;
	}
	return strAxisZ;
}

void CStartZCompen::ClearStep()
{
	m_ZCompenStep = CalibrationZCompenStep::INIT;
}

void CStartZCompen::ClearZCompen()
{
	ZeroMemory(dblMean, sizeof(dblMean));
	ZeroMemory(dblPlusCompen, sizeof(dblPlusCompen));
	ZeroMemory(dblMinusCompen, sizeof(dblMinusCompen));
}

void CStartZCompen::SetZCompenMode(bool bApply)
{
	m_bApply = bApply;
	TRACE(_T("[PWR] SetZCompenMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStartZCompen::GetZCompenMode()
{
	return m_bApply;
}

void CStartZCompen::SetPlusZDiff(unsigned indx, double dblDiff)
{
	if (GetZCompenMode() == true)
	{
		dblPlusCompen[indx] = dblDiff;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetPlusZDiff(%d,%.3f)"), indx, dblPlusCompen[indx]);
		}
	}
}

void CStartZCompen::SetMinusZDiff(unsigned indx, double dblDiff)
{
	if (GetZCompenMode() == true)
	{
		dblMinusCompen[indx] = dblDiff;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetMinusZDiff(%d,%.3f)"), indx, dblMinusCompen[indx]);
		}
	}
}

void CStartZCompen::ShowZMeanData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_Z_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d Mean Diff %.3f\n"), indx, dblMean[indx]);
	}
}

void CStartZCompen::ShowZRawData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_Z_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] ZCompen indx:%d Plus,Minus Diff %.3f,%.3f=%.3f\n"), indx, dblPlusCompen[indx], dblMinusCompen[indx], dblPlusCompen[indx] - dblMinusCompen[indx]);
	}
}

long CStartZCompen::CopyGlobalZCompensationData(long MaxCnt, double StartCompensation, double StartSensorHeight)
{
	long indx = 0, HeadNo = 0;
	double Init = 0.0;
	if (MaxCnt < 1)
	{
		return 1;
	}
	for (indx = CAL_Z_INIT; indx < MaxCnt; ++indx)
	{
		dblMean[indx] = (dblPlusCompen[MaxCnt - (indx + 1)]);
	}
	Init = dblMean[0];
	TRACE(_T("[PWR] CopyGlobalZCompensationData %.3f MaxCnt:%d\n"), Init, MaxCnt);
	for (indx = MaxCnt; indx < BUFSIZE; ++indx)
	{
		dblMean[indx] = 0.000;
	}
	HeadNo = GetZAxisIndexByZName(GetZAxis());
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		TRACE(_T("[PWR] CopyGlobalZCompensationData %s HeadNo:%d\n"), GetZAxis(), HeadNo);
		gcPowerCalibrationData->SetZCompensationStartData(FRONT_GANTRY, HeadNo - 1, StartCompensation);
		for (indx = 0; indx < BUFSIZE; ++indx)
		{
			gcPowerCalibrationData->SetZCompensationData(FRONT_GANTRY, HeadNo - 1, indx, dblMean[indx]);
		}
		if (GetZAxis() == _T("FZ1") || GetZAxis() == _T("RZ1"))
		{
			gcPowerCalibrationData->SetSensorOriginHeight(FRONT_GANTRY, StartSensorHeight);
		}
		gcPowerCalibrationData->WriteZCompensationData(FRONT_GANTRY);
		gcPowerCalibrationData->WriteSensorOriginHeight(FRONT_GANTRY);
	}
	return 0;
}

UINT CStartZCompen::StartZCompenCalibration(LPVOID wParam)
{
	bool bLoop = true, bRet = false;
	unsigned nCompenCount = 0, nCompenMax = CAL_Z_MAXCOUNT;
	int err = ErrorCode::None;
	long lTimeChk = 0;
	double StartPosZ = 0.0, StartHeigthMeasurement = 0.0;
	double MovePosZ = 0.0, MoveHeigthMeasurement = 0.1, Sum = 0.0;
	double DistPosZ = 0.1, Diff = 0.0, Read = 0.0;
	CStartZCompen* pThis = reinterpret_cast<CStartZCompen*>(wParam);
	CApplicationTime* pTime = new CApplicationTime();
	double Ratio = 0.3;
	double Inpos = 0.001;
	long TimeOut = TIME10000MS, Ms = TIME300MS;
	if (pThis->GetZCompenMode() == false)
	{
		Ratio = 0.5;
	}
	StartPosZ = ReadPosition(pThis->GetZAxis());
	if (IsOneAxisHomingComplete(pThis->GetZAxis()) == false)
	{
		return 1;
	}
	if (pThis->GetZCompenMode() ==  true)
	{
		AxisZCompensationOff(pThis->GetZAxis());
		ServoOff(pThis->GetZAxis());
		ServoOn(pThis->GetZAxis());
		StartPosWaitDelayedInposition(pThis->GetZAxis(), Ratio, TimeOut, StartPosZ, Inpos, Ms, true);
		ThreadSleep(TIME100MS);
	}
	StartPosZ = ReadPosition(pThis->GetZAxis());
	StartHeigthMeasurement = GetHeight(FRONT_GANTRY);
	TRACE(_T("[PWR] (%s) Start Pos,Sensor,%.3f,%.3f\n"), pThis->GetZAxis(), StartPosZ, StartHeigthMeasurement);
	for (nCompenCount = CAL_Z_INIT; nCompenCount < nCompenMax; ++nCompenCount)
	{
		Sum = 0.0;
		MovePosZ = StartPosZ - (DistPosZ * nCompenCount);
		StartPosWaitDelayedInposition(pThis->GetZAxis(), Ratio, TimeOut, MovePosZ, Inpos, Ms, true);
		Read = ReadPosition(pThis->GetZAxis());
		for (long indx = 0; indx < CAL_Z_FILTERCOUNT; ++indx)
		{
			Sum += GetHeight(FRONT_GANTRY);
			ThreadSleep(TIME10MS);
		}
		MoveHeigthMeasurement = Sum / CAL_Z_FILTERCOUNT;
		Diff = MoveHeigthMeasurement - (StartHeigthMeasurement + (DistPosZ * nCompenCount));
		if (abs(Diff) < MAX_Z_VALID_RANGE)
		{
			pThis->SetPlusZDiff(nCompenCount, Diff);
		}
		TRACE(_T("[PWR] (%s)(%04d) Cmd,Read,Default,Sensor,Diff,%.3f,%.3f,%.3f,%.3f,%.3f\n"), pThis->GetZAxis(), nCompenCount, MovePosZ, Read, (StartHeigthMeasurement + (DistPosZ * nCompenCount)), MoveHeigthMeasurement, Diff);
	}
	if (pThis->GetZCompenMode() == true)
	{
		pThis->CopyGlobalZCompensationData(nCompenMax, MovePosZ, StartHeigthMeasurement);
	}
	delete pTime;
	pTime = NULL;
	TRACE(_T("[PWR] CStartZCompen(0x%x) Quit\n"), pThis->GetThreadID());
	return 0;
}

void CStartZCompen::Run()
{
	CString strLog;
	HANDLE nHandle;
	DWORD nID = NULL;
	_beginthreadex_proc_type lpStartAddress;
	SetThreadName(_T("thZCompenCalibration"));
	lpStartAddress = (_beginthreadex_proc_type)StartZCompenCalibration;
	nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	if (nHandle != INVALID_HANDLE_VALUE)
		SetThreadHandle(nHandle);
	if (nID != NULL)
		SetThreadID(nID);	
	TRACE(_T("[PWR] ZCompen Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
	strLog.Format(_T("[PWR] ZCompen Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
	gcPowerLog->Logging(strLog);
}


