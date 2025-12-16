#include "pch.h"
#include "CPowerGantry.h"
#include "AxisInformation.h"
#include "Cwmx3Axis.h"
#include "GlobalData.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "LockDef.h"
#include "Trace.h"
#include "CApplicationTime.h"
#include "DefineThreadLoopTime.h"
#include "DefineMotionLoopTime.h"
#include "CTokenizer.h"
#include "Cwmx3LinearIntplPos.h"
#include "COriginSearch.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorData.h"
#include "CPowerLog.h"
#include "Vision.h"
#include "VisionData.h"
//#include "ErrorCode.h"
#include "LockDef.h"
#include "CRunFile.h"
#include "CReadJobFile.h"

CPowerGantry* gcPowerGantry;
CPowerGantry::CPowerGantry()
{
	InitializeValue();
	SetTable(FRONT_GANTRY);
	m_CmdLock = CreateMutex(NULL, FALSE, NULL);
	for (int indx = 0; indx < MAXGANTRYAXISNO; ++indx)
	{
		m_bPause[indx] = false;
		m_MotionLock[indx] = CreateMutex(NULL, FALSE, NULL);
	}
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		m_bUseZAxis[indx] = true;
		m_bUseRAxis[indx] = true;
		m_bSuction[indx] = false;
		m_bBlow[indx] = false;
	}
	SetWriteDisk(false);

	LoadValue();
	SetUseZAxis(_T("FZ1"), true);
	SetUseZAxis(_T("FZ2"), true);
	SetUseZAxis(_T("FZ3"), true);
	SetUseZAxis(_T("FZ4"), true);
	SetUseZAxis(_T("FZ5"), true);
	SetUseZAxis(_T("FZ6"), true);
	AddAllZAxis();

	SetUseRAxis(_T("FW1"), true);
	SetUseRAxis(_T("FW2"), true);
	SetUseRAxis(_T("FW3"), true);
	SetUseRAxis(_T("FW4"), true);
	SetUseRAxis(_T("FW5"), true);
	SetUseRAxis(_T("FW6"), true);
	AddAllRAxis();

	NozzleNoPerHeadStruct nzl = gcRunFile->ReadNozzleNoPerHead(FRONT_GANTRY);

	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; HeadNo++)
	{
		TRACE(_T("[PWR] Init Nzl Head:%d Hole:%d"), HeadNo + 1, nzl.Head[HeadNo]);
		m_GlobalNozzleNo[HeadNo] = nzl.Head[HeadNo];
	}
	m_MoveOnceZREnable = false;
	m_LockOKAxis.Format(_T("NULL"));

}

CPowerGantry::~CPowerGantry()
{
}

long CPowerGantry::GetAxisMap(CString strAxis)
{
	long AxisMap = NON;
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis != NULL)
		AxisMap = pAxis->GetAxisMap();
	return AxisMap;
}

bool CPowerGantry::Lock(CString strAxis)
{
	long AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		return false;
	SEM_LOCK(m_MotionLock[AxisNo], INFINITE);
	return true;
}

bool CPowerGantry::Unlock(CString strAxis)
{
	long AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		return false;
	SEM_UNLOCK(m_MotionLock[AxisNo]);
	return true;
}

bool CPowerGantry::Release(CString strAxis)
{
	long AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		return false;
	SEM_UNLOCK(m_MotionLock[AxisNo]);
	return true;
}

bool CPowerGantry::Lock(long AxisNo)
{
	SEM_LOCK(m_MotionLock[AxisNo], INFINITE);
	return true;
}

bool CPowerGantry::Unlock(long AxisNo)
{
	SEM_UNLOCK(m_MotionLock[AxisNo]);
	return true;
}

bool CPowerGantry::Release(long AxisNo)
{
	SEM_UNLOCK(m_MotionLock[AxisNo]);
	return true;
}

long CPowerGantry::GetAxisNullError(CString strAxis)
{
	long Err = NO_ERR, AxisNo = NON;
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		Err = UNDEFINED_AXIS_NULL;
	else
		Err = AXIS_NULL(AxisNo);
	return Err;
}

long CPowerGantry::GetTimeOutError(CString strAxis)
{
	long Err = NO_ERR, AxisNo = NON;
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		Err = UNDEFINED_AXIS_TIMEOUT;
	else
		Err = MOTION_TIMEOUT(AxisNo);
	return Err;
}

long CPowerGantry::GetMinusLimitError(CString strAxis)
{
	long Err = NO_ERR, AxisNo = NON;
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		Err = UNDEFINED_MINUS_LIMIT;
	else
		Err = MINUS_LIMIT(AxisNo);
	return Err;
}

long CPowerGantry::GetPlusLimitError(CString strAxis)
{
	long Err = NO_ERR, AxisNo = NON;
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		Err = UNDEFINED_PLUS_LIMIT;
	else
		Err = PLUS_LIMIT(AxisNo);
	return Err;
}

long CPowerGantry::GetServoOnError(CString strAxis)
{
	long Err = NO_ERR, AxisNo = NON;
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo == NON)
		Err = UNDEFINED_AXIS_NULL;
	else
		Err = SERVO_ON_TIMEOUT(AxisNo);
	return Err;
}

void CPowerGantry::SetShortDist(CString strAxis, long index, double ShortDist)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		if (abs(ShortDist) > MAX_MOTION_VALID_RANGE)
		{
			TRACE(_T("[PWR] SetShortDist index:%d ShortDist:%.3f"), index, ShortDist);
		}
		if (pAxis != NULL)
		{
			pAxis->SetShortDist(index, ShortDist);
			if (gcPowerLog->IsShowShortDistLog() == true)
			{
				TRACE(_T("[PWR] ShortDistNo:%02d Dist:%.3f\n"), index + 1, ShortDist);
			}
		}
	}
}

void CPowerGantry::SetShortDistVel(CString strAxis, long index, double ShortVel)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		if (pAxis != NULL)
		{
			pAxis->SetShortDistVel(index, ShortVel);
			if (gcPowerLog->IsShowShortDistLog() == true)
			{
				TRACE(_T("[PWR] ShortDistNo:%02d ShortVel:%.3f\n"), index + 1, ShortVel);
			}
		}
	}
}

void CPowerGantry::SetShortDistAccDec(CString strAxis, long index, double ShortAccDec)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		if (pAxis != NULL)
		{
			pAxis->SetShortDistAccDec(index, ShortAccDec);
			if (gcPowerLog->IsShowShortDistLog() == true)
			{
				TRACE(_T("[PWR] SetShortDistAccDec:%02d ShortVel:%.3f\n"), index + 1, ShortAccDec);
			}
		}
	}
}

void CPowerGantry::SetMoveProfile(CString strAxis, WMX3_AXIS_POSCOMMANDPROFILE profile)
{
	Cwmx3Axis* pAxis = NULL;
	Profile* pProfile = new Profile();
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis != NULL)
	{
		memcpy(pProfile, &profile, sizeof(Profile));
		pAxis->SetMoveProfile(*pProfile);
		//if (gcPowerLog->IsShowShortDistLog() == true)
		{
			TRACE(_T("[PWR] SetMoveProfile(%s) Vel:%.1f Acc:%.1f Dec:%.1f\n"), strAxis,
				pProfile->velocity,
				pProfile->acc,
				pProfile->dec);
		}
	}
	delete pProfile;
}

void CPowerGantry::SetInsertByZ(long Gantry, double InsertByZ)
{
	m_InsertByZ = InsertByZ;
}

double CPowerGantry::GetInsertByZ(long Gantry)
{
	return m_InsertByZ;
}

void CPowerGantry::SetPusherByZ(long Gantry, double PusherByZ)
{
	m_PusherByZ = PusherByZ;
}

double CPowerGantry::GetPusherByZ(long Conveyor)
{
	return m_PusherByZ;
}

void CPowerGantry::SetStandByZ(long Gantry, double StandByZ)
{
	m_StandByZ = StandByZ;
}

double CPowerGantry::GetStandByZ(long Gantry)
{
	return m_StandByZ;
}

void CPowerGantry::SetMaxZTorqueLimit(long Gantry, long Head, double MaxTorqueLimit)
{
	if (0 < Head && Head <= MAXUSEDHEADNO)
	{
		TRACE(_T("[PWR] SetMaxZTorqueLimit Gantry:%d Head:%d Torque:%.3f -> %.3f\n"), GetTable(), Head, m_ZMaxTorqueLimit[Head - 1], MaxTorqueLimit);
		m_ZMaxTorqueLimit[Head - 1] = MaxTorqueLimit;
	}
}

double CPowerGantry::GetMaxZTorqueLimit(long Gantry, long Head)
{
	if (0 < Head && Head <= MAXUSEDHEADNO)
	{
		TRACE(_T("[PWR] GetMaxZTorqueLimit Gantry:%d Head:%d Torque:%.3f\n"), GetTable(), Head, m_ZMaxTorqueLimit[Head - 1], m_ZMaxTorqueLimit[Head - 1]);
		return m_ZMaxTorqueLimit[Head - 1];
	}

	return 200.0;
}

void CPowerGantry::SetStandByR(long Gantry, double StandByR)
{
	m_StandByR = StandByR;
}

double CPowerGantry::GetStandByR(long Gantry)
{
	return m_StandByR;
}

void CPowerGantry::SetTable(long Table)
{
	m_Table = Table;
}

long CPowerGantry::GetTable()
{
	return m_Table;
}

double CPowerGantry::GetDryRunZHeightOffset()
{
	return m_DryRunZHeightOffset;
}

Point_XY CPowerGantry::ReadGantryPosition(long Gantry)
{
	Point_XY pt;
	pt.x = ReadOnePosition(GetAxisX(Gantry));
	pt.y = ReadOnePosition(GetAxisY1(Gantry));
	return pt;
}

void CPowerGantry::ReadAllPosition(long Gantry)
{
	Cwmx3Axis* pAxis = NULL;
	for (long indx = 0; indx < MAXGANTRYAXISNO; ++indx)
	{
		pAxis = GetWmx3AxisByAxisNo(PowerAxisArray[indx]);
		if (pAxis != NULL)
		{
			m_Position[PowerAxisArray[indx]] = pAxis->ReadMotorPosition();
			TRACE(_T("[PWR] (%02d,%10s) %7.3f Servo(%3s) Home(%5s) Torque(%.3f) Origin(%.3f)\n"),
				PowerAxisArray[indx],
				pAxis->GetAxisName(),
				m_Position[PowerAxisArray[indx]],
				pAxis->CheckServoOn() == true ? _T("On") : _T("Off"),
				pAxis->IsAxisHomeDone() == true ? _T("Done") : _T("Yet "),
				pAxis->ReadActualTorque(),
				gGetHomePosition(indx));
		}
	}
}

double CPowerGantry::ReadPosition(long AxisNo)
{
	double Pos = 0.0;
	if (AxisNo < MAXGANTRYAXISNO)
	{
		Pos = m_Position[AxisNo];
	}
	return Pos;
}

long CPowerGantry::GetCameraHeadFromHeadNo(long HeadNo)
{
	long RetHeadNo = HeadNo;
	switch (HeadNo)
	{
	case TBL_HEAD1:
		RetHeadNo = TBL_HEAD1;
		break;
	case TBL_HEAD2:
		RetHeadNo = TBL_HEAD2;
		break;
	case TBL_HEAD3:
		RetHeadNo = TBL_HEAD3;
		break;
	case TBL_HEAD4:
		RetHeadNo = TBL_HEAD1;
		break;
	case TBL_HEAD5:
		RetHeadNo = TBL_HEAD2;
		break;
	case TBL_HEAD6:
		RetHeadNo = TBL_HEAD3;
		break;
	}
	return RetHeadNo;
}

long CPowerGantry::GetCamera6HeadFromHeadNo(long HeadNo)
{
	long RetHeadNo = HeadNo;
	switch (HeadNo)
	{
	case TBL_HEAD1:
	case TBL_HEAD2:
	case TBL_HEAD3:
	case TBL_HEAD4:
	case TBL_HEAD5:
	case TBL_HEAD6:
		RetHeadNo = TBL_HEAD1;
		break;
	}
	return RetHeadNo;
}

long CPowerGantry::GetCameraNoByHead(long Gantry, long HeadNo)
{
	long CameraNo = MAXCAMNO;
	switch (HeadNo)
	{
	case TBL_HEAD1:
	case TBL_HEAD2:
	case TBL_HEAD3:
		if (Gantry == FRONT_GANTRY)
			CameraNo = CAM1;
		else
			CameraNo = RCAM1;
		break;
	case TBL_HEAD4:
	case TBL_HEAD5:
	case TBL_HEAD6:
		if (Gantry == FRONT_GANTRY)
			CameraNo = CAM2;
		else
			CameraNo = RCAM2;
		break;
	}
	return CameraNo;
}

long CPowerGantry::GetCamera6NoByHead(long Gantry, long HeadNo)
{
	long CameraNo = MAXCAMNO;
	switch (HeadNo)
	{
	case TBL_HEAD1:
		CameraNo = CAM1;
		break;
	case TBL_HEAD2:
		CameraNo = CAM2;
		break;
	case TBL_HEAD3:
		CameraNo = CAM3;
		break;
	case TBL_HEAD4:
		CameraNo = CAM4;
		break;
	case TBL_HEAD5:
		CameraNo = CAM5;
		break;
	case TBL_HEAD6:
		CameraNo = CAM6;
		break;
	}
	return CameraNo;
}

long CPowerGantry::GetCameraChkPosByHead(long HeadNo)
{
	long ChkPos = 0;
	switch (HeadNo)
	{
	case TBL_HEAD1:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD2:	ChkPos = CHK_CENT2;		break;
	case TBL_HEAD3:	ChkPos = CHK_CENT3;		break;
	case TBL_HEAD4:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD5:	ChkPos = CHK_CENT2;		break;
	case TBL_HEAD6:	ChkPos = CHK_CENT3;		break;
	}
	return ChkPos;
}

long CPowerGantry::GetCamera6ChkPosByHead(long HeadNo)
{
	long ChkPos = 0;
	switch (HeadNo)
	{
	case TBL_HEAD1:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD2:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD3:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD4:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD5:	ChkPos = CHK_CENT;		break;
	case TBL_HEAD6:	ChkPos = CHK_CENT;		break;
	}
	return ChkPos;
}

bool CPowerGantry::IsCameraCenterPositionByHeadNo(long HeadNo)
{
	bool bRet = false;
	switch (HeadNo)
	{
	case TBL_HEAD1:
	case TBL_HEAD2:
	case TBL_HEAD3:
		bRet = true;
		break;
	case TBL_HEAD4:
	case TBL_HEAD5:
	case TBL_HEAD6:
		bRet = false;
		break;
	}
	return bRet;
}

bool CPowerGantry::IsCamera6CenterPositionByHeadNo(long HeadNo)
{
	bool bRet = false;
	switch (HeadNo)
	{
	case TBL_HEAD1:
		bRet = true;
		break;
	case TBL_HEAD2:
	case TBL_HEAD3:
	case TBL_HEAD4:
	case TBL_HEAD5:
	case TBL_HEAD6:
		bRet = false;
		break;
	}
	return bRet;
}

void CPowerGantry::InitialNozzle()
{
	ZeroMemory(&m_GlobalNozzleNo, sizeof(m_GlobalNozzleNo));
	ZeroMemory(&m_MachineNozzle, sizeof(m_MachineNozzle));
}

void CPowerGantry::InitialPosition()
{
	m_GlobalYellowTowerLampTime = 60; // Sec
	m_StandByZ = 20.0;	// Safty Position
	m_StandByR = 0.0;
	m_InsertByZ = 120.0;
	m_PusherByZ = 120.0;
	for (long i = 0; i < MAXUSEDHEADNO; i++)
	{
		m_ZMaxTorqueLimit[i] = 200.0;
	}
	m_DryRunZHeightOffset = 10.0;
	ZeroMemory(&m_Discard, sizeof(m_Discard));
	ZeroMemory(&m_Position, sizeof(m_Position));
}

void CPowerGantry::InitializeCamPosMech()
{
	m_CameraRecogPosition[0].x = 420.000;
	m_CameraRecogPosition[0].y = 260.000;

	m_CameraRecogPosition[1].x = 420.000;
	m_CameraRecogPosition[1].y = 260.000;

	m_CameraRecogPosition[2].x = 420.000;
	m_CameraRecogPosition[2].y = 260.000;

	m_CameraRecogPosition[3].x = 288.735;
	m_CameraRecogPosition[3].y = 260.000;

	m_CameraRecogPosition[4].x = 288.735;
	m_CameraRecogPosition[4].y = 260.000;

	m_CameraRecogPosition[5].x = 288.735;
	m_CameraRecogPosition[5].y = 260.000;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		SetCameraRecognitionPosition(TBL_HEAD1 + indx, m_CameraRecogPosition[indx]);
	}
}

void CPowerGantry::InitializeCamOffsetMech()
{
	m_CameraRecogOffset[0].x = 0.000;
	m_CameraRecogOffset[0].y = 0.000;

	m_CameraRecogOffset[1].x = 0.000;
	m_CameraRecogOffset[1].y = 0.000;

	m_CameraRecogOffset[2].x = 0.000;
	m_CameraRecogOffset[2].y = 0.000;

	m_CameraRecogOffset[3].x = 0.000;
	m_CameraRecogOffset[3].y = 0.000;

	m_CameraRecogOffset[4].x = 0.000;
	m_CameraRecogOffset[4].y = 0.000;

	m_CameraRecogOffset[5].x = 0.000;
	m_CameraRecogOffset[5].y = 0.000;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		SetCameraRecognitionOffset(TBL_HEAD1 + indx, m_CameraRecogOffset[indx]);
	}
}

void CPowerGantry::InitializeRearCamPosMech()
{
	// 구형 i-6.0
	//m_RearCameraRecogPosition[0].x = 386.600;
	//m_RearCameraRecogPosition[0].y = 901.1265; // 1st:898.500, 2nd:903.753

	//m_RearCameraRecogPosition[1].x = 346.600;
	//m_RearCameraRecogPosition[1].y = 901.1265;

	//m_RearCameraRecogPosition[2].x = 306.600;
	//m_RearCameraRecogPosition[2].y = 901.1265;

	//m_RearCameraRecogPosition[3].x = 386.600;
	//m_RearCameraRecogPosition[3].y = 901.1265;

	//m_RearCameraRecogPosition[4].x = 346.600;
	//m_RearCameraRecogPosition[4].y = 901.1265;

	//m_RearCameraRecogPosition[5].x = 306.600;
	//m_RearCameraRecogPosition[5].y = 901.1265;

	m_RearCameraRecogPosition[0].x = 348.700;// 386.600;
	m_RearCameraRecogPosition[0].y = 923.126;//901.126;// 1st:898.500, 2nd:903.753

	m_RearCameraRecogPosition[1].x = 306.700;// 346.600;
	m_RearCameraRecogPosition[1].y = 923.126;

	m_RearCameraRecogPosition[2].x = 266.700;//306.600;
	m_RearCameraRecogPosition[2].y = 923.126;

	m_RearCameraRecogPosition[3].x = 348.700;
	m_RearCameraRecogPosition[3].y = 923.126;

	m_RearCameraRecogPosition[4].x = 306.700;
	m_RearCameraRecogPosition[4].y = 923.126;

	m_RearCameraRecogPosition[5].x = 266.700;
	m_RearCameraRecogPosition[5].y = 923.126;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		SetRearCameraRecognitionPosition(TBL_HEAD1 + indx, m_RearCameraRecogPosition[indx]);
	}
}

void CPowerGantry::InitializeRearCamOffsetMech()
{
	m_RearCameraRecogOffset[0].x = 0.000;
	m_RearCameraRecogOffset[0].y = 0.000;

	m_RearCameraRecogOffset[1].x = 0.000;
	m_RearCameraRecogOffset[1].y = 0.000;

	m_RearCameraRecogOffset[2].x = 0.000;
	m_RearCameraRecogOffset[2].y = 0.000;

	m_RearCameraRecogOffset[3].x = 0.000;
	m_RearCameraRecogOffset[3].y = 0.000;

	m_RearCameraRecogOffset[4].x = 0.000;
	m_RearCameraRecogOffset[4].y = 0.000;

	m_RearCameraRecogOffset[5].x = 0.000;
	m_RearCameraRecogOffset[5].y = 0.000;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		SetRearCameraRecognitionOffset(TBL_HEAD1 + indx, m_RearCameraRecogOffset[indx]);
	}
}

void CPowerGantry::InitializeHeadMech()
{
	m_HeadOffset[0].x = 126.000;
	m_HeadOffset[0].y = -3.000;

	m_HeadOffset[1].x = 87.000;
	m_HeadOffset[1].y = -3.000;

	m_HeadOffset[2].x = 48.000;
	m_HeadOffset[2].y = -3.000;

	m_HeadOffset[3].x = -48.000;
	m_HeadOffset[3].y = -3.000;

	m_HeadOffset[4].x = -87.000;
	m_HeadOffset[4].y = -3.000;

	m_HeadOffset[5].x = -126.000;
	m_HeadOffset[5].y = -3.000;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		SetHeadOffset(TBL_HEAD1 + indx, m_HeadOffset[indx]);
	}
}

void CPowerGantry::InitializeHeadMech(long HeadNo)
{
	switch (HeadNo)
	{
	case TBL_HEAD1:
		m_HeadOffset[0].x = 126.000;
		m_HeadOffset[0].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[0]);
		break;
	case TBL_HEAD2:
		m_HeadOffset[1].x = 87.000;
		m_HeadOffset[1].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[1]);
		break;
	case TBL_HEAD3:
		m_HeadOffset[2].x = 48.000;
		m_HeadOffset[2].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[2]);
		break;
	case TBL_HEAD4:
		m_HeadOffset[3].x = -48.000;
		m_HeadOffset[3].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[3]);
		break;
	case TBL_HEAD5:
		m_HeadOffset[4].x = -87.000;
		m_HeadOffset[4].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[4]);
		break;
	case TBL_HEAD6:
		m_HeadOffset[5].x = -126.000;
		m_HeadOffset[5].y = -3.000;
		SetHeadOffset(HeadNo, m_HeadOffset[5]);
		break;
	}
}

void CPowerGantry::InitializeHeightSensorMech()
{
	m_HeightMeasurementOffset.x = m_HeightMeasurementOffset.y = 0.000;
}

void CPowerGantry::InitializeValue()
{
	InitialNozzle();
	InitialPosition();
	InitializeHeadMech();
	InitializeCamPosMech();
	InitializeCamOffsetMech();
	InitializeRearCamPosMech();
	InitializeRearCamOffsetMech();
	InitializeHeightSensorMech();
}

long CPowerGantry::SendCameraRecognitionOffset(long Gantry)
{
	MachineReferenceMark RefMark = GetGlobalMachineReferenceMark();

	Point_XY ptCamToHd1, ptCamToHd2, ptCamToHd3, ptCamToHd4, ptCamToHd5, ptCamToHd6;
	ZeroMemory(&ptCamToHd1, sizeof(ptCamToHd1));
	ZeroMemory(&ptCamToHd2, sizeof(ptCamToHd2));
	ZeroMemory(&ptCamToHd3, sizeof(ptCamToHd3));
	ZeroMemory(&ptCamToHd4, sizeof(ptCamToHd4));
	ZeroMemory(&ptCamToHd5, sizeof(ptCamToHd5));
	ZeroMemory(&ptCamToHd6, sizeof(ptCamToHd6));

	if (RefMark.Use == 1)
	{
		ptCamToHd1 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD1);
		ptCamToHd2 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD2);
		ptCamToHd3 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD3);
		ptCamToHd4 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD4);
		ptCamToHd5 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD5);
		ptCamToHd6 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(FRONT_GANTRY, TBL_HEAD6);
	}
	else
	{
		ptCamToHd1 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD1);
		ptCamToHd2 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD2);
		ptCamToHd3 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD3);
		ptCamToHd4 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD4);
		ptCamToHd5 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD5);
		ptCamToHd6 = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, TBL_HEAD6);
	}

	if (GetCameraCount(FRONT_GANTRY) == CAMERA_COUNT_6)
	{
		gSendCameraRecognitionOffset(CAM1, ptCamToHd1, ptCamToHd1, ptCamToHd1);
		gSendCameraRecognitionOffset(CAM2, ptCamToHd2, ptCamToHd2, ptCamToHd2);
		gSendCameraRecognitionOffset(CAM3, ptCamToHd3, ptCamToHd3, ptCamToHd3);
		gSendCameraRecognitionOffset(CAM4, ptCamToHd4, ptCamToHd4, ptCamToHd4);
		gSendCameraRecognitionOffset(CAM5, ptCamToHd5, ptCamToHd5, ptCamToHd5);
		gSendCameraRecognitionOffset(CAM6, ptCamToHd6, ptCamToHd6, ptCamToHd6);
	}
	else if (GetCameraCount(FRONT_GANTRY) == CAMERA_COUNT_2)
	{
		gSendCameraRecognitionOffset(CAM1, ptCamToHd1, ptCamToHd2, ptCamToHd3);
		gSendCameraRecognitionOffset(CAM2, ptCamToHd4, ptCamToHd5, ptCamToHd6);
	}

	if (RefMark.Use == 1)
	{
		ptCamToHd1 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD1);
		ptCamToHd2 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD2);
		ptCamToHd3 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD3);
		ptCamToHd4 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD4);
		ptCamToHd5 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD5);
		ptCamToHd6 = gcPowerCalibrationData->GetCameraRecognitionOffsetRefMarkCompen(REAR_GANTRY, TBL_HEAD6);
	}
	else
	{
		ptCamToHd1 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD1);
		ptCamToHd2 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD2);
		ptCamToHd3 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD3);
		ptCamToHd4 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD4);
		ptCamToHd5 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD5);
		ptCamToHd6 = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, TBL_HEAD6);
	}

	if (GetCameraCount(REAR_GANTRY) == CAMERA_COUNT_6)
	{
		gSendCameraRecognitionOffset(RCAM1, ptCamToHd1, ptCamToHd1, ptCamToHd1);
		gSendCameraRecognitionOffset(RCAM2, ptCamToHd2, ptCamToHd2, ptCamToHd2);
		gSendCameraRecognitionOffset(RCAM3, ptCamToHd3, ptCamToHd3, ptCamToHd3);
		gSendCameraRecognitionOffset(RCAM4, ptCamToHd4, ptCamToHd4, ptCamToHd4);
		gSendCameraRecognitionOffset(RCAM5, ptCamToHd5, ptCamToHd5, ptCamToHd5);
		gSendCameraRecognitionOffset(RCAM6, ptCamToHd6, ptCamToHd6, ptCamToHd6);
	}
	else if (GetCameraCount(REAR_GANTRY) == CAMERA_COUNT_2)
	{
		gSendCameraRecognitionOffset(RCAM1, ptCamToHd1, ptCamToHd2, ptCamToHd3);
		gSendCameraRecognitionOffset(RCAM2, ptCamToHd4, ptCamToHd5, ptCamToHd6);
	}

	TRACE(_T("[PWR] SendCameraRecognitionOffset Front Camera(%d) Rear Camera(%d)\n"), GetCameraCount(FRONT_GANTRY), GetCameraCount(REAR_GANTRY));
	return 0;
}

void CPowerGantry::SetHeadOffset(long HeadNo, Point_XY Offset)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_HeadOffset[HeadNo - 1].x = Offset.x;
		m_HeadOffset[HeadNo - 1].y = Offset.y;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetHeadOffset HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, Offset.x, Offset.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetHeadOffset invalid head no:%d\n"), HeadNo);
	}
}

Point_XY CPowerGantry::GetHeadOffset(long HeadNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		pt = m_HeadOffset[HeadNo - 1];
	}
	return pt;
}

Point_XY CPowerGantry::GetHMOffset()
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	pt = m_HeightMeasurementOffset;
	return pt;
}

void CPowerGantry::SetCameraRecognitionPosition(long HeadNo, Point_XY Position)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_CameraRecogPosition[HeadNo - 1] = Position;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetCameraRecognitionPosition HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, Position.x, Position.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetCameraRecognitionPosition invalid head no:%d\n"), HeadNo);
	}
}

Point_XY CPowerGantry::GetCameraRecognitionPosition(long HeadNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		pt = m_CameraRecogPosition[HeadNo - 1];
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] GetCameraRecognitionPosition HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetCameraRecognitionPosition invalid head no:%d\n"), HeadNo);
	}
	return pt;
}

void CPowerGantry::SetCameraRecognitionOffset(long HeadNo, Point_XY Position)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_CameraRecogOffset[HeadNo - 1] = Position;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetCameraRecognitionOffset HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, Position.x, Position.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetCameraRecognitionOffset invalid head no:%d\n"), HeadNo);
	}
}

Point_XY CPowerGantry::GetCameraRecognitionOffset(long HeadNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		pt = m_CameraRecogOffset[HeadNo - 1];
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] GetCameraRecognitionOffset HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetCameraRecognitionOffset invalid head no:%d\n"), HeadNo);
	}
	return pt;
}

void CPowerGantry::SetRearCameraRecognitionPosition(long HeadNo, Point_XY Position)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_RearCameraRecogPosition[HeadNo - 1] = Position;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetRearCameraRecognitionPosition HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, Position.x, Position.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetRearCameraRecognitionPosition invalid head no:%d\n"), HeadNo);
	}
}

Point_XY CPowerGantry::GetRearCameraRecognitionPosition(long HeadNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		pt = m_RearCameraRecogPosition[HeadNo - 1];
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] GetRearCameraRecognitionPosition HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetRearCameraRecognitionPosition invalid head no:%d\n"), HeadNo);
	}
	return pt;
}

void CPowerGantry::SetRearCameraRecognitionOffset(long HeadNo, Point_XY Position)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_RearCameraRecogOffset[HeadNo - 1] = Position;
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetRearCameraRecognitionOffset HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, Position.x, Position.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetRearCameraRecognitionOffset invalid head no:%d\n"), HeadNo);
	}
}

Point_XY CPowerGantry::GetRearCameraRecognitionOffset(long HeadNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		pt = m_RearCameraRecogOffset[HeadNo - 1];
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] GetRearCameraRecognitionOffset HeadNo:%d X,Y,%.3f,%.3f\n"), HeadNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetRearCameraRecognitionOffset invalid head no:%d\n"), HeadNo);
	}
	return pt;
}

void CPowerGantry::SetGlobalDiscardPosition(Point_XYRZ Discard)
{
	m_Discard = Discard;
	TRACE(_T("[PWR] SetGlobalDiscardPosition XYRZ:%.3f %.3f %.3f %.3f\n"),
		Discard.x, Discard.y, Discard.r, Discard.z);
}

Point_XYRZ CPowerGantry::GetGlobalDiscardPosition()
{
	Point_XYRZ Discard;
	Discard = m_Discard;
	return Discard;
}

void CPowerGantry::SetGlobalNozzleNo(long HeadNo, long NozzleNo)
{
	NozzleNoPerHeadStruct Nzl;

	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		if (m_GlobalNozzleNo[HeadNo - 1] != NozzleNo)
		{
			gcLastPickFront->ClearHeadData(HeadNo);
		}

		m_GlobalNozzleNo[HeadNo - 1] = NozzleNo;

		for (long i = 0; i < MAXUSEDHEADNO; i++)
		{
			Nzl.Head[i] = m_GlobalNozzleNo[i];
		}

		//	Nzl.Head[HeadNo - 1] = NozzleNo;

		gcRunFile->WriteNozzleNoPerHead(FRONT_GANTRY, Nzl);

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetGlobalNozzleNo HeadNo:%d NozzleNo:%d\n"), HeadNo, NozzleNo);
		}

		gcRunFile->SaveFile();
	}
}

long CPowerGantry::GetGlobalNozzleNo(long HeadNo)
{
	long NozzleNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		NozzleNo = m_GlobalNozzleNo[HeadNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetGlobalNozzleNo HeadNo:%d NozzleNo:%d\n"), HeadNo, NozzleNo);
		}
	}
	return NozzleNo;
}

void CPowerGantry::SetGlobalNozzleInformation(long StationNo, NOZZLE NozzleInfo)
{
	if (StationNo > 0 && StationNo <= MAXSTATIONNO)
	{
		m_MachineNozzle[StationNo - 1] = NozzleInfo;
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetGlobalNozzleInformation StationNo:%d No:%d Use:%d Type:%d Tip:%.3f Pusher:%.3f\n"),
				StationNo, NozzleInfo.No, NozzleInfo.Use, NozzleInfo.Type, NozzleInfo.TipHeight, NozzleInfo.PusherHeight);
		}
	}
}

NOZZLE CPowerGantry::GetGlobalNozzleInformation(long StationNo)
{
	NOZZLE NozzleInfo;
	ZeroMemory(&NozzleInfo, sizeof(NozzleInfo));
	if (StationNo > 0 && StationNo <= MAXSTATIONNO)
	{
		NozzleInfo = m_MachineNozzle[StationNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetGlobalNozzleInformation StationNo:%d No:%d Use:%d Type:%d Tip:%.3f Pusher:%.3f\n"),
				StationNo, NozzleInfo.No, NozzleInfo.Use, NozzleInfo.Type, NozzleInfo.TipHeight, NozzleInfo.PusherHeight);
		}
	}
	return NozzleInfo;
}

void CPowerGantry::SetGlobalTowerYellowLampTime(long YellowTowerLampTime)
{
	m_GlobalYellowTowerLampTime = YellowTowerLampTime;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetGlobalTowerYellowLampTime %d[Sec]\n"), YellowTowerLampTime);
	}
}

long CPowerGantry::GetGlobalTowerYellowLampTime()
{
	long YellowTowerLampTime = 60;
	YellowTowerLampTime = m_GlobalYellowTowerLampTime;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetGlobalTowerYellowLampTime %d[Sec]\n"), YellowTowerLampTime);
	}
	return YellowTowerLampTime;
}

void CPowerGantry::SetGlobalEmptyBuzzerTime(long EmptyBuzzerTime)
{
	m_GlobalEmptyBuzzerTime = EmptyBuzzerTime;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetGlobalEmptyBuzzerTime %d[Sec]\n"), EmptyBuzzerTime);
	}
}

long CPowerGantry::GetGlobalEmptyBuzzerTime()
{
	long EmptyBuzzerTime = 60;
	EmptyBuzzerTime = m_GlobalEmptyBuzzerTime;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetGlobalEmptyBuzzerTime %d[Sec]\n"), EmptyBuzzerTime);
	}
	return EmptyBuzzerTime;
}

void CPowerGantry::SetGlobalMachineReferenceMark(MachineReferenceMark ReferenceMark)
{
	m_MachineReferenceMark = ReferenceMark;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetGlobalMachineReferenceMark Use:%d CycleTime:%d XY Mark1,%.3f,%.3f Mark2,%.3f,%.3f Mark3,%.3f,%.3f Mark4,%.3f,%.3f\n"),
			ReferenceMark.Use,
			ReferenceMark.CycleTime,
			ReferenceMark.Mark[0].x, ReferenceMark.Mark[0].y,
			ReferenceMark.Mark[1].x, ReferenceMark.Mark[1].y,
			ReferenceMark.Mark[2].x, ReferenceMark.Mark[2].y, 
			ReferenceMark.Mark[3].x, ReferenceMark.Mark[3].y);
	}
}

MachineReferenceMark CPowerGantry::GetGlobalMachineReferenceMark()
{
	MachineReferenceMark ReferenceMark;
	ReferenceMark = m_MachineReferenceMark;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetGlobalMachineReferenceMark Use:%d\n"), ReferenceMark.Use);
	}
	return ReferenceMark;
}

void CPowerGantry::SetWriteDisk(bool bWrite)
{
	m_bWriteDisk = bWrite;
}

void CPowerGantry::SetUseZAxis(CString strZAxis, bool bUse)
{
	long indexZ = 0;
	if (strZAxis.GetLength() > 0)
	{
		strZAxis.Delete(0, 2);
		indexZ = ConvertCStringToInt(strZAxis);
		if (indexZ > 0)
		{
			if (m_bUseZAxis[indexZ - 1] != bUse)
			{
				m_bUseZAxis[indexZ - 1] = bUse;
				SetWriteDisk(true);
			}
		}
	}
}

void CPowerGantry::SetUseRAxis(CString strRAxis, bool bUse)
{
	long indexR = 0;
	if (strRAxis.GetLength() > 0)
	{
		strRAxis.Delete(0, 2);
		indexR = ConvertCStringToInt(strRAxis);
		if (indexR > 0)
		{
			if (m_bUseRAxis[indexR - 1] != bUse)
			{
				m_bUseRAxis[indexR - 1] = bUse;
				SetWriteDisk(true);
			}
		}
	}
}

void CPowerGantry::LoadValue()
{
	Point_XY pt;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetHeadOffset(FRONT_GANTRY, indx + TBL_HEAD1);
		m_HeadOffset[indx] = pt;
		pt = gcPowerCalibrationData->GetCameraRecognitionPosition(FRONT_GANTRY, indx + TBL_HEAD1);
		m_CameraRecogPosition[indx] = pt;
		pt = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, indx + TBL_HEAD1);
		m_CameraRecogOffset[indx] = pt;
		pt = gcPowerCalibrationData->GetCameraRecognitionPosition(REAR_GANTRY, indx + TBL_HEAD1);
		m_RearCameraRecogPosition[indx] = pt;
		pt = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, indx + TBL_HEAD1);
		m_RearCameraRecogOffset[indx] = pt;
	}
	pt = gcPowerCalibrationData->GetHMOffset(FRONT_GANTRY);
	m_HeightMeasurementOffset = pt;
}

void CPowerGantry::SaveValue()
{
	Point_XY pt;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = m_HeadOffset[indx];
		gcPowerCalibrationData->SetHeadOffset(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		pt = m_CameraRecogPosition[indx];
		gcPowerCalibrationData->SetCameraRecognitionPosition(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		pt = m_CameraRecogOffset[indx];
		gcPowerCalibrationData->SetCameraRecognitionOffset(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		pt = m_RearCameraRecogPosition[indx];
		gcPowerCalibrationData->SetCameraRecognitionPosition(REAR_GANTRY, indx + TBL_HEAD1, pt);
		pt = m_RearCameraRecogOffset[indx];
		gcPowerCalibrationData->SetCameraRecognitionOffset(REAR_GANTRY, indx + TBL_HEAD1, pt);
	}
	pt = m_HeightMeasurementOffset;
	gcPowerCalibrationData->SetHMOffset(FRONT_GANTRY, pt);
}

void CPowerGantry::AddAllZAxis()
{
	CString StrAxis[MAXUSEDHEADNO];
	CString strNo;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		if (m_bUseZAxis[indx] == false) continue;
		strNo.Format(_T("%d"), indx + 1);
		StrAxis[indx] = _T("FZ") + strNo;
		AddZAxis(StrAxis[indx]);
	}
	TRACE(_T("[PWR] AddAllZAxis GetZAxisCount():%d\n"), GetZAxisCount());
}

void CPowerGantry::AddAllRAxis()
{
	CString StrAxis[MAXUSEDHEADNO];
	CString strNo;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		if (m_bUseRAxis[indx] == false) continue;
		strNo.Format(_T("%d"), indx + 1);
		StrAxis[indx] = _T("FW") + strNo;
		AddRAxis(StrAxis[indx]);
	}
	TRACE(_T("[PWR] AddAllRAxis GetRAxisCount():%d\n"), GetRAxisCount());
}

void CPowerGantry::AddZAxis(CString pStrAxis)
{
	m_ZaxisArray.Add(pStrAxis);
}

bool CPowerGantry::RemoveZAxis(INT_PTR indx)
{
	CString StrAxis;
	StrAxis = m_ZaxisArray.GetAt(indx);
	if (StrAxis.IsEmpty() != false)
	{
		m_ZaxisArray.RemoveAt(indx);
		delete StrAxis;
	}
	return true;
}

INT_PTR CPowerGantry::GetZAxisCount()
{
	INT_PTR retCount = 0;
	retCount = m_ZaxisArray.GetCount();
	return retCount;
}

CString CPowerGantry::GetZAxisByIndex(INT_PTR indx)
{
	CString StrAxis;
	ASSERT(indx >= 0 && indx < m_ZaxisArray.GetCount());
	StrAxis = m_ZaxisArray.GetAt(indx);
	return StrAxis;
}

long CPowerGantry::GetZAxisIndexByZName(CString strZAxis)
{
	long ZAxisNo = 0;
	if (strZAxis.CompareNoCase(_T("FZ1")) == 0)
	{
		ZAxisNo = TBL_HEAD1;
	}
	else if (strZAxis.CompareNoCase(_T("FZ2")) == 0)
	{
		ZAxisNo = TBL_HEAD2;
	}
	else if (strZAxis.CompareNoCase(_T("FZ3")) == 0)
	{
		ZAxisNo = TBL_HEAD3;
	}
	else if (strZAxis.CompareNoCase(_T("FZ4")) == 0)
	{
		ZAxisNo = TBL_HEAD4;
	}	
	else if (strZAxis.CompareNoCase(_T("FZ5")) == 0)
	{
		ZAxisNo = TBL_HEAD5;
	}
	else if (strZAxis.CompareNoCase(_T("FZ6")) == 0)
	{
		ZAxisNo = TBL_HEAD6;
	}
	else
	{
		ZAxisNo = NON;
	}
	return ZAxisNo;
}

void CPowerGantry::AddRAxis(CString pStrAxis)
{
	m_RaxisArray.Add(pStrAxis);
}

bool CPowerGantry::RemoveRAxis(INT_PTR indx)
{
	CString StrAxis = NULL;
	StrAxis = m_RaxisArray.GetAt(indx);
	if (StrAxis.IsEmpty() != false)
	{
		m_RaxisArray.RemoveAt(indx);
		delete StrAxis;
	}
	return true;
}

INT_PTR CPowerGantry::GetRAxisCount()
{
	INT_PTR retCount = 0;
	retCount = m_RaxisArray.GetCount();
	return retCount;
}

CString CPowerGantry::GetRAxisByIndex(INT_PTR indx)
{
	CString StrAxis;
	ASSERT(indx >= 0 && indx < m_RaxisArray.GetCount());
	StrAxis = m_RaxisArray.GetAt(indx);
	return StrAxis;
}

HANDLE CPowerGantry::GetThreadLock()
{
	ASSERT(m_CmdLock != INVALID_HANDLE_VALUE);
	return m_CmdLock;
}

bool CPowerGantry::Lock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_LOCK(GetThreadLock(), INFINITE);
	return true;
}

bool CPowerGantry::Unlock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_UNLOCK(GetThreadLock());
	return true;
}

long CPowerGantry::SetInitializeEnd(CString strAxis, bool bEnd)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		pAxis->SetInitializeEnd(bEnd);
	}
	return Err;
}

long CPowerGantry::OneOriginSearch(CString strAxis, double forceOffset)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	THREAD_STRUCT threadInfo;
	pAxis = GetWmx3AxisByName(strAxis);
	COriginSearch* pOriginSearch = new COriginSearch(strAxis);
	if (pAxis != NULL)
	{
		pAxis->SetInitializeEnd(false);
		pAxis->SetInitializeFail(false);
		pAxis->SetForceOriginOffset(forceOffset);
		pOriginSearch->Run();
	}
	CApplicationTime* pTime = new CApplicationTime();
	while (true)
	{
		if (IsOneAxisHomingComplete(strAxis) == true)
		{
			break;
		}
		if (pOriginSearch->GetStep() == HomingStep::SELFQUIT)
		{
			if (IsOneAxisHomingComplete(strAxis) == false)
			{
				Err = pOriginSearch->GetHomingErr();
			}
			break;
		}
		if (pTime->TimeElapsed() > TIME30000MS)
		{
			Err = HOMING_TIMOUT(pAxis->GetAxisIndex());
			break;
		}
		ThreadSleep(TIME100MS);
	}
	TRACE(_T("[PWR] OneOriginSearch(%s) Force Offset:%.3f Elapsed:%d[ms]\n"), strAxis, forceOffset, pTime->TimeElapsed());
	delete pTime;
	return Err;
}

long CPowerGantry::GetAllAxisHomingInComplete()
{
	INT_PTR indx = 0;
	long nHomedCount = 0, nHomedYetCount = 0;
	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		if (IsOneAxisHomingComplete(indx) == true)
		{
			nHomedCount++;
		}
		else
		{
			nHomedYetCount++;
		}
	}
	return nHomedYetCount;
}

bool CPowerGantry::IsAllAxisHomingComplete()
{
	INT_PTR indx = 0;
	long nHomedCount = 0, nHomedYetCount = 0, nHomedYetAxis = 0;
	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		if (IsOneAxisHomingComplete(indx) == true)
		{
			nHomedCount++;
		}
		else
		{
			nHomedYetCount++;
			nHomedYetAxis = (long)indx;
		}
	}
	//TRACE(_T("[PWR] IsAllAxisHomingComplete(%d) Done(%d) Yet(%d) Axis(%d)\n"), GetWmx3AxisCount(), nHomedCount, nHomedYetCount, nHomedYetAxis);
	if (nHomedCount == GetWmx3AxisCount())
		return true;
	else
		return false;
}

bool CPowerGantry::IsAllZAxisHomingComplete()
{
	INT_PTR indx = 0;
	int nZHomedCount = 0;
	CString StrAxis;
	Cwmx3Axis* pAxis;
	for (indx = 0; indx < GetZAxisCount(); ++indx)
	{
		StrAxis = GetZAxisByIndex(indx);
		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetInitializeEnd() == true)
		{
			nZHomedCount++;
		}
	}
	if (GetZAxisCount() > 0)
	{
		if (nZHomedCount == GetZAxisCount())
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool CPowerGantry::IsAllRAxisHomingComplete()
{
	INT_PTR indx = 0;
	int nRHomedCount = 0;
	CString StrAxis;
	Cwmx3Axis* pAxis;
	for (indx = 0; indx < GetRAxisCount(); ++indx)
	{
		StrAxis = GetRAxisByIndex(indx);
		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetInitializeEnd() == true)
		{
			nRHomedCount++;
		}
	}
	if (GetRAxisCount() > 0)
	{
		if (nRHomedCount == GetRAxisCount())
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool CPowerGantry::IsOneRAxisHomingComplete(INT_PTR indx)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByIndex(indx);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->IsRAxis() == true)
	{
		if (pAxis->GetInitializeEnd() == true)
		{
			return true;
		}
	}
	return false;
}

bool CPowerGantry::IsOneZAxisHomingComplete(INT_PTR indx)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByIndex(indx);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->IsZAxis() == true)
	{
		if (pAxis->GetInitializeEnd() == true)
		{
			return true;
		}
	}
	return false;
}

bool CPowerGantry::IsOneAxisHomingComplete(INT_PTR indx)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByIndex(indx);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->GetInitializeEnd() == true)
		return true;
	return false;
}

bool CPowerGantry::IsOneAxisHomingComplete(CString strAxis)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->GetInitializeEnd() == true)
		return true;
	return false;
}

CString CPowerGantry::GetAxisX(long Gantry)
{
	CString strAxis = _T("NON");
	if (Gantry == FRONT_GANTRY)
	{
		strAxis = _T("FX");
	}
	else if (Gantry == REAR_GANTRY)
	{
		strAxis = _T("RX");
	}
	return strAxis;
}

CString CPowerGantry::GetAxisY1(long Gantry)
{
	CString strAxis = _T("NON");
	if (Gantry == FRONT_GANTRY)
	{
		strAxis = _T("FY1");
	}
	else if (Gantry == REAR_GANTRY)
	{
		strAxis = _T("RY1");
	}
	return strAxis;
}

CString CPowerGantry::GetAxisY2(long Gantry)
{
	CString strAxis = _T("NON");
	if (Gantry == FRONT_GANTRY)
	{
		strAxis = _T("FY2");
	}
	else if (Gantry == REAR_GANTRY)
	{
		strAxis = _T("RY2");
	}
	return strAxis;
}

CString CPowerGantry::GetConvName(long Conveyor)
{
	CString strConv = _T("NON");
	if (Conveyor == FRONT_CONV)
	{
		strConv = _T("FCONV");
	}
	else if (Conveyor == REAR_CONV)
	{
		strConv = _T("RCONV");
	}
	return strConv;
}

CString CPowerGantry::GetPusherZName(long Conveyor)
{
	CString strPusherZ = _T("NON");
	if (Conveyor == FRONT_CONV)
	{
		strPusherZ = _T("FPUZ");
	}
	else if (Conveyor == REAR_CONV)
	{
		strPusherZ = _T("RPUZ");
	}
	return strPusherZ;
}

CString CPowerGantry::GetRAxisFromHeadNo(long Gantry, long HeadNo)
{
	CString strRAxis = _T("NON");
	if (HeadNo == TBL_HEAD1)
	{
		strRAxis = _T("FW1");
	}
	else if (HeadNo == TBL_HEAD2)
	{
		strRAxis = _T("FW2");
	}
	else if (HeadNo == TBL_HEAD3)
	{
		strRAxis = _T("FW3");
	}
	else if (HeadNo == TBL_HEAD4)
	{
		strRAxis = _T("FW4");
	}
	else if (HeadNo == TBL_HEAD5)
	{
		strRAxis = _T("FW5");
	}
	else if (HeadNo == TBL_HEAD6)
	{
		strRAxis = _T("FW6");
	}
	return strRAxis;
}

CString CPowerGantry::GetZAxisFromHeadNo(long Gantry, long HeadNo)
{
	CString strZAxis = _T("NON");
	if (HeadNo == TBL_HEAD1)
	{
		strZAxis = _T("FZ1");
	}
	else if (HeadNo == TBL_HEAD2)
	{
		strZAxis = _T("FZ2");
	}
	else if (HeadNo == TBL_HEAD3)
	{
		strZAxis = _T("FZ3");
	}
	else if (HeadNo == TBL_HEAD4)
	{
		strZAxis = _T("FZ4");
	}
	else if (HeadNo == TBL_HEAD5)
	{
		strZAxis = _T("FZ5");
	}
	else if (HeadNo == TBL_HEAD6)
	{
		strZAxis = _T("FZ6");
	}
	return strZAxis;
}

CString CPowerGantry::GetHeadToRotateAxis(long Gantry, CString strZAxis)
{
	CString strRAxis = _T("NON");
	if (strZAxis.CompareNoCase(_T("FZ1")) == 0)
	{
		strRAxis = _T("FW1");
	}
	else if (strZAxis.CompareNoCase(_T("FZ2")) == 0)
	{
		strRAxis = _T("FW2");
	}
	else if (strZAxis.CompareNoCase(_T("FZ3")) == 0)
	{
		strRAxis = _T("FW3");
	}
	else if (strZAxis.CompareNoCase(_T("FZ4")) == 0)
	{
		strRAxis = _T("FW4");
	}
	else if (strZAxis.CompareNoCase(_T("FZ5")) == 0)
	{
		strRAxis = _T("FW5");
	}
	else if (strZAxis.CompareNoCase(_T("FZ6")) == 0)
	{
		strRAxis = _T("FW6");
	}
	return strRAxis;
}

CString CPowerGantry::GetRotateAxisToHead(long Gantry, CString strRAxis)
{
	CString strZAxis = _T("NON");
	if (strRAxis.CompareNoCase(_T("FW1")) == 0)
	{
		strZAxis = _T("FZ1");
	}
	else if (strRAxis.CompareNoCase(_T("FW2")) == 0)
	{
		strZAxis = _T("FZ2");
	}
	else if (strRAxis.CompareNoCase(_T("FW3")) == 0)
	{
		strZAxis = _T("FZ3");
	}
	else if (strRAxis.CompareNoCase(_T("FW4")) == 0)
	{
		strZAxis = _T("FZ4");
	}
	else if (strRAxis.CompareNoCase(_T("FW5")) == 0)
	{
		strZAxis = _T("FZ5");
	}
	else if (strRAxis.CompareNoCase(_T("FW6")) == 0)
	{
		strZAxis = _T("FZ6");
	}
	return strZAxis;
}

void CPowerGantry::InitOneRatio(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] InitOneRatio %s is NULL\n"), strAxis);
	}
	else
	{
		pAxis->InitializeRatio();
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Init to Ratio %.3f\n"), strAxis, pAxis->GetRatio());
		}
	}
}

void CPowerGantry::SetOneRatio(CString strAxis, double Ratio)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetOneRatio %s is NULL\n"), strAxis);
	}
	else
	{
		pAxis->SetRatio(Ratio);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Set to Ratio %.3f\n"), strAxis, Ratio);
		}
	}
}

void CPowerGantry::SetOne2ndRatio(CString strAxis, double Ratio2nd)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetOne2ndRatio %s is NULL\n"), strAxis);
	}
	else
	{
		pAxis->Set2ndRatio(Ratio2nd);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Set to 2ndRatio %.3f\n"), strAxis, Ratio2nd);
		}
	}
}

long CPowerGantry::SetOnePosSet(CString strAxis, double PosSetWidth)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetOnePosSet %s is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->SetPosSet(PosSetWidth);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Set to PosSet %.3f\n"), strAxis, PosSetWidth);
		}
	}
	return Err;
}

long CPowerGantry::SetOneInPos(CString strAxis, double PosSetWidth)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetOneInPos %s is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->SetInPos(PosSetWidth);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Set to InPosSet %.3f\n"), strAxis, PosSetWidth);
		}
	}
	return Err;
}

long CPowerGantry::SetOneDelayedPosSet(CString strAxis, double PosSetWidth, long ms)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetOneDelayedPosSet %s is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->SetOneDelayedPosSet(PosSetWidth, ms);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Set to DelayedPosSet %.3f[mm] Ms:%d[ms]\n"), strAxis, PosSetWidth, ms);
		}
	}
	return Err;
}

long CPowerGantry::StopOne(CString strAxis)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StopOne(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Profile profile = pAxis->GetMoveProfile();
		Err = pAxis->StopMotion(profile.dec);
	}
	return Err;
}

long CPowerGantry::StopOne(CString strAxis, double dec)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StopOne(%s,%.1f) is NULL\n"), strAxis, dec);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->StopMotion(dec);
	}
	return Err;
}

long CPowerGantry::GetTorqueLimit(CString strAxis, double* pTorqueLimit)
{
	long Err = NO_ERR;
	double TorqueLimit = 0.0;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetTorqueLimit(%s,%.1f) is NULL\n"), strAxis, *pTorqueLimit);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		TorqueLimit = pAxis->GetMaxTrqLimit();
	}
	*pTorqueLimit = TorqueLimit;
	return Err;
}

long CPowerGantry::SetTorqueLimit(CString strAxis, double Torque)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetTorqueLimit(%s,%.1f) is NULL\n"), strAxis, Torque);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->SetMaxTrqLimit(Torque);
	}
	return Err;
}

long CPowerGantry::SetEventToStopByOverTorque(long Eventid, CString strAxis, double Torque, double SaftyPosition)
{
	long Err = NO_ERR;
	double SetTorque = Torque;
	int EventID = 0;
	long AxisMap = 0;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetEventToStopByOverTorque(%s,EventID:%d,%.1f,%.1f) is NULL\n"), strAxis, Eventid, Torque, SaftyPosition);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		AxisMap = pAxis->GetAxisMap();
		EventControl::Event* Evt = new EventControl::Event();
		Evt->inputFunction = EventControl::EventInputFunction::GreaterTrq;
		Evt->input.greaterTrq.axis = AxisMap;
		Evt->input.greaterTrq.trq = SetTorque;
		Evt->input.greaterTrq.enableUnsigned = 1; // 0 : signed, 1 : absolute
		Evt->outputFunction = EventControl::EventOutputFunction::StartSinglePos;
		Evt->output.startSinglePos.axis = AxisMap;
		Evt->output.startSinglePos.disableAfterActivate = 1;
		Evt->output.startSinglePos.acc = pAxis->GetMoveProfile().acc;
		Evt->output.startSinglePos.dec = pAxis->GetMoveProfile().dec;
		Evt->output.startSinglePos.velocity = pAxis->GetMoveProfile().velocity;
		Evt->output.startSinglePos.target = pAxis->GetUnResol() * SaftyPosition;
		Evt->output.startSinglePos.jerkAccRatio = 0.3;
		Evt->output.startSinglePos.jerkDecRatio = 0.3;
		Evt->output.startSinglePos.type = ProfileType::JerkRatio;
		Evt->enabled = 1;
		Err = GetEventControl()->SetEvent(&EventID, Evt, Eventid);
		if (Err != ErrorCode::None)
		{
			EventControlErrorToString(Err, _T("SetEventToStopByOverTorque"));
		}
		else
		{
			TRACE(_T("[PWR] SetEventToStopByOverTorque(%s,EventID:%d,%.1f,%.1f) Complete\n"), strAxis, Eventid, Torque, SaftyPosition);
		}
		delete Evt;
	}
	return Err;
}

long CPowerGantry::SetEventToOverrideVelByPosition(long Eventid, CString strAxis, double Position, double Ratio)
{
	long Err = NO_ERR;
	long AxisMap = 0;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetEventToOverrideVelByPosition(%s,%.1f,%.1f) is NULL\n"), strAxis, Position, Ratio);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		int err = ErrorCode::None;
		double SetPosition = Position * pAxis->GetUnResol();
		Profile profile = pAxis->GetMoveProfile();
		double Vel = profile.velocity * Ratio;
		AxisMap = pAxis->GetAxisMap();
		int EventID = AxisMap;
		EventControl::Event* Evt = new EventControl::Event();
		Evt->inputFunction = EventControl::EventInputFunction::GreaterPos;
		Evt->input.greaterPos.axis = AxisMap;
		Evt->input.greaterPos.pos = SetPosition;
		Evt->outputFunction = EventControl::EventOutputFunction::OverrideVelSingleAxis;
		Evt->output.overrideVelSingleAxis.axis = AxisMap;
		Evt->output.overrideVelSingleAxis.velocity = Vel;
		Evt->enabled = 1;
		err = GetEventControl()->SetEvent(&EventID, Evt);
		TRACE(_T("[PWR] SetEventToOverrideVelByPosition EvtID:%d Pos:%.3f Ratio:%.1f Vel:%.1f\n"), EventID, SetPosition, Ratio, Vel);
		if (err != ErrorCode::None)
		{
			EventControlErrorToString(err, _T("SetEventToOverrideVelByPosition"));
		}
		delete Evt;
		return err;
	}
	return Err;
}

long CPowerGantry::SetEventToStopByAreaSensor(long Eventid, long AreaSensor, CString strAxis)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetEventToStopByAreaSensor(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		long AreaSensorIO = AreaSensor;
		int EventID = 0;
		EventControl::Event* Evt = new EventControl::Event();
		Evt->inputFunction = EventControl::EventInputFunction::DelayIOBit;
		Evt->input.delayIOBit.ioSourceType = IOSourceType::Input;
		Evt->input.delayIOBit.byteAddress = GetIOAddr(AreaSensor);
		Evt->input.delayIOBit.bitAddress = GetIOBit(AreaSensor);
		Evt->input.delayIOBit.delayTime = TIME10MS;
		Evt->input.delayIOBit.invert = 1;
		Evt->outputFunction = EventControl::EventOutputFunction::ExecQuickStopSingleAxis;
		Evt->output.execQuickStopSingleAxis.axis = pAxis->GetAxisMap();
		Evt->output.execQuickStopSingleAxis.disableAfterActivate = 1;
		Evt->enabled = 1;
		Err = GetEventControl()->SetEvent(&EventID, Evt, Eventid);
		if (Err != ErrorCode::None)
		{
			EventControlErrorToString(Err, _T("SetEventToStopByAreaSensor"));
		}
		delete Evt;
	}
	return Err;
}

bool CPowerGantry::GetEventAvailable(long Eventid)
{
	long Err = NO_ERR;
	bool bRet = false;
	EventControl::Event* Evt = new EventControl::Event();
	int EventID = Eventid;
	Err = GetEventControl()->GetEvent(EventID, Evt);
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("GetEventAvailable"));
	}
	if (Evt->enabled == 0)
	{
		bRet = true;
	}
	else
	{
		bRet = false;
	}
	delete Evt;
	return bRet;
}

long CPowerGantry::RemoveEvent(long Eventid)
{
	long Err = NO_ERR;
	int EventID = Eventid;
	Err = GetEventControl()->RemoveEvent(EventID);
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("RemoveEvent"));
	}
	return Err;
}

long CPowerGantry::EnableEvent(long Eventid)
{
	long Err = NO_ERR;
	int EventID = Eventid;
	Err = GetEventControl()->EnableEvent(EventID, 1);
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("EnableEvent"));
	}
	return Err;
}

long CPowerGantry::ClearAllEvent()
{
	long Err = NO_ERR;
	Err = GetEventControl()->ClearAllEvent();
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("ClearAllEvent"));
	}
	TRACE(_T("[PWR] ClearAllEvent Err:%d\n"), Err);
	return Err;
}

long CPowerGantry::GetAllEventCount()
{
	long Err = NO_ERR;
	long Count = 0;
	AllEventID* pEventIdData = new AllEventID;
	Err = GetEventControl()->GetAllEventID(pEventIdData);
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("GetAllEventCount"));
	}
	Count = pEventIdData->count;
	delete pEventIdData;
	return Count;
}

long CPowerGantry::StartWmx3Monitor(CString strAxis, long BoardCount, long BlockNo, long InsertNo, CString Action)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartWmx3Monitor(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->StartWmx3Monitor(BoardCount, BlockNo, InsertNo, Action);
	}
	return Err;
}

bool CPowerGantry::WaitStopMonitor(CString strAxis)
{
	bool Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitMonitor(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->WaitStopWmx3Monitor(TIME100MS);
	}
	return Err;
}

long CPowerGantry::ResetMonitor(CString strAxis)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ResetMonitor(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->ResetWmx3Monitor();
	}
	return Err;
}

CString CPowerGantry::GetTorqueMonitorFileName(CString strAxis)
{
	CString strFileName;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetTorqueMonitorFileName(%s) is NULL\n"), strAxis);
		//Err = GetAxisNullError(strAxis);
		strFileName.Empty();
	}
	else
	{
		strFileName = pAxis->GetTorqueMonitorFileName();
	}
	return strFileName;
}

long CPowerGantry::StopWmx3Monitor(CString strAxis)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StopWmx3Monitor(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->StopWmx3Monitor();
	}
	return Err;
}

long CPowerGantry::WaitOneIdle(CString strAxis, long TimeOut)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	CoreMotionAxisStatus status;
	long remaintime = 0;
	CString strMsg;

	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitOneIdle(%s,%.1f) is NULL\n"), strAxis, dec);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		GetTime = _time_get();
		while (pAxis->IsAxisIdle() == false && pAxis->IsAxisMotionComplete() == false)
		{
			if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
			{
				if (status.profileRemainingMilliseconds > TimeOut)
				{
					remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
					TRACE(_T("[PWR] WaitOneIdle TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
					TimeOut = remaintime;
				}
			}

			if (CheckServoOn(strAxis) == false)
			{
				Err = GetServoOnError(strAxis);
				TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
				strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				break;
			}

			ThreadSleep(WAIT_IDLE_READTIME);
			ElapsedTime = _time_elapsed(GetTime);
			if(ElapsedTime > TimeOut)
			{
				TRACE(_T("[PWR] WaitOneIdle(%s) TimeOut(%d)\n"), strAxis, TimeOut);
				Err = GetTimeOutError(strAxis);
				break;
			}
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] WaitOneIdle(%s) Elapsed:%.3f[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::StartOneJog(CString strAxis, JogInfo jogInfo)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::JogCommand* jogCmd = new Motion::JogCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOneJog %s is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		if (pAxis->IsConveyorAxis() == false && pAxis->IsConveyorBeltAxis() == false && pAxis->IsPusherZAxis() == false)
		{
			Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		}
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOneJog CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOneJog Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOneJog SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			jogCmd->profile = pAxis->GetMoveProfile();
			jogCmd->profile.acc = jogInfo.Acc;
			jogCmd->profile.dec = jogInfo.Dec;
			jogCmd->profile.velocity = jogInfo.MaxVel;
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] StartOneJog %s Acc:%.3f Dec:%.3f Vel:%.3f\n"), strAxis,
					jogInfo.Acc, jogInfo.Dec, jogInfo.MaxVel);
			}
			Err = pAxis->StartJog(jogCmd);
		}
	}
	delete jogCmd;
	return Err;
}

long CPowerGantry::StartOnePosition(CString strAxis, double Cmd)
{
	if (GetOnlyConveyorMode() == true)
	{
		return NO_ERR;
	}


	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOnePosition(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		if (pAxis->IsConveyorAxis() == false && pAxis->IsConveyorBeltAxis() == false && pAxis->IsPusherZAxis() == false)
		{
			Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		}
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOnePosition CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOnePosition GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOnePosition Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOnePosition SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOnePosition AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/							

				Err = MoveOnceLockPauseMode(strAxis, Cmd);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOnePosition STOP_NOW\n"), strAxis);
					return Err;
				}

				posCmd->profile = pAxis->GetMoveProfile();
				Err = pAxis->StartPos(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOnePosition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOnePositionSkipLimitCheck(CString strAxis, double Cmd)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOnePositionSkipLimitCheck(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		if (pAxis->IsConveyorAxis() == false && pAxis->IsConveyorBeltAxis() == false && pAxis->IsPusherZAxis() == false)
		{
			Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		}
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOnePositionSkipLimitCheck CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOnePositionSkipLimitCheck GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOnePositionSkipLimitCheck Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOnePositionSkipLimitCheck SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}

			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOnePosition AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/

				Err = MoveOnceLockPauseMode(strAxis, Cmd);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOnePosition STOP_NOW\n"), strAxis);
					return Err;
				}

				posCmd->profile = pAxis->GetMoveProfile();
				Err = pAxis->StartPos(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOnePositionSkipLimitCheck Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOne2StepPosition(CString strAxis, double Cmd, double Cmd2nd)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOne2StepPosition(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOne2StepPosition CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOne2StepPosition GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOne2StepPosition Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOne2StepPosition SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOne2StepPosition AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/

				Err = MoveOnceLockPauseMode(strAxis);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOne2StepPosition STOP_NOW\n"), strAxis);
					return Err;
				}
				posCmd->profile = pAxis->GetMoveProfile();
				Err = pAxis->StartPosWithTriggerPos(posCmd, Cmd, Cmd2nd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOne2StepPosition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOnePositionX(CString strAxis, double Cmd, double Ratio)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOnePositionX(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOnePositionX CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOnePositionX GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOnePositionX Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOnePositionX SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOnePositionX AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis, Cmd);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOnePositionX STOP_NOW\n"), strAxis);
					return Err;
				}

				posCmd->profile = pAxis->GetMoveProfile();
				TRACE(_T("[PWR] StartOnePositionX Cmd:%.3f Velocity:%.3f Ratio:%.3f\n"), Cmd, posCmd->profile.velocity, Ratio);
				Err = pAxis->StartavsMotionX(Cmd, posCmd->profile.velocity * Ratio);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOnePositionX Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOnePositionY(CString strAxis, double Cmd, double Ratio)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOnePositionY(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOnePositionY CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOnePositionY GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOnePositionY Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOnePositionY SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOnePositionY AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis, Cmd);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOnePositionY STOP_NOW\n"), strAxis);
					return Err;
				}
				posCmd->profile = pAxis->GetMoveProfile();
				TRACE(_T("[PWR] StartavsMotionY Cmd:%.3f Velocity:%.3f Ratio:%.3f\n"), Cmd, posCmd->profile.velocity, Ratio);
				Err = pAxis->StartavsMotionY(Cmd, posCmd->profile.velocity * Ratio);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOnePositionY Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOnePositionWithoutSlave(CString strAxis, double Cmd)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOnePositionWithoutSlave(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOnePositionWithoutSlave CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOnePositionWithoutSlave GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
			}
			Err = CheckLimitOver(strAxis, Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOnePositionWithoutSlave AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis, Cmd);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOnePositionWithoutSlave STOP_NOW\n"), strAxis);
					return Err;
				}
				posCmd->profile = pAxis->GetMoveProfile();
				Err = pAxis->StartPos(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOnePositionWithoutSlave Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOneMove(CString strAxis, double Cmd)
{
	long Err = NO_ERR;
	double Position = ReadOnePosition(strAxis);
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOneMove(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		if (pAxis->IsConveyorAxis() == false && pAxis->IsConveyorBeltAxis() == false && pAxis->IsPusherZAxis() == false)
		{
			Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		}
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOneMove CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOneMove GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOneMove Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOneMove SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Position + Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOneMove AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOneMove STOP_NOW\n"), strAxis);
					return Err;
				}
				posCmd->profile = pAxis->GetMoveProfile(Cmd);
				pAxis->StartMove(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOneMove Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOneMoveWithoutSlave(CString strAxis, double Cmd)
{
	long Err = NO_ERR;
	double Position = ReadOnePosition(strAxis);
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	pAxis = GetWmx3AxisByName(strAxis);
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOneMoveWithoutSlave(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		if (pAxis->GetInitializeEnd() == false)
		{
			Err = HOMING_FAIL(pAxis->GetAxisIndex());
			TRACE(_T("[PWR] StartOneMoveWithoutSlave GetInitializeEnd Err:%d\n"), Err);
			CString strMsg;
			strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
			Err = SendAlarm(Err, strMsg);
			return Err;
		}
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOneMoveWithoutSlave CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
			}
			Err = CheckLimitOver(strAxis, Position + Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOneMoveWithoutSlave AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOneMoveWithoutSlave STOP_NOW\n"), strAxis);
					return Err;
				}
				posCmd->profile = pAxis->GetMoveProfile(Cmd);
				pAxis->StartMove(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOneMoveWithoutSlave Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::StartOneTeachMove(CString strAxis, double Cmd)
{
	long Err = NO_ERR;
	double Position = ReadOnePosition(strAxis);
	Cwmx3Axis* pAxis;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	WMX3AxisSyncMode SyncMode = WMX3AxisSyncMode::NoSync;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] StartOneTeachMove(%s,%.1f) is NULL\n"), strAxis, Cmd);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartOneTeachMove CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			if (pAxis->GetInitializeEnd() == false)
			{
				Err = HOMING_FAIL(pAxis->GetAxisIndex());
				TRACE(_T("[PWR] StartOneTeachMove GetInitializeEnd Err:%d\n"), Err);
				CString strMsg;
				strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)strAxis);
				Err = SendAlarm(Err, strMsg);
				return Err;
			}
			if (pAxis->GetUseSlaveAxis() == true)
			{
				SyncMode = pAxis->GetAxisSyncMode();
				if (SyncMode == WMX3AxisSyncMode::NoSync)
				{
					TRACE(_T("[PWR] %s StartOneTeachMove Not Ready SyncMode(%d)\n"), strAxis, SyncMode);
					Err = pAxis->SetSyncMasterSlave();
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] StartOneTeachMove SetSyncMasterSlave %s Err:%d\n"), pAxis->GetAxisName(), Err);
						return Err;
					}
					ThreadSleep(TIME100MS);
					SyncMode = pAxis->GetAxisSyncMode();
				}
			}
			Err = CheckLimitOver(strAxis, Position + Cmd);
			if (Err == NO_ERR)
			{
				/*if (pAxis->IsGantryAxis() == true)
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartOneTeachMove AxisIndex(%d) Lock-1\n"), pAxis->GetAxisIndex());
					}
					SEM_LOCK(gMOTION_LOCK[pAxis->GetAxisIndex()], INFINITE);
				}*/
				Err = MoveOnceLockPauseMode(strAxis);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] %s StartOneTeachMove STOP_NOW\n"), strAxis);
					return Err;
				}

				posCmd->profile = pAxis->GetTeachBoxMoveProfile();
				pAxis->StartMove(posCmd, Cmd);
			}
			else
			{
				TRACE(_T("[PWR] %s StartOneTeachMove Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
			}
		}
	}
	delete posCmd;
	return Err;
}

bool CPowerGantry::IsAllEventAvailable(long* Err, bool UseSendAlarm)
{
	bool bEventAvailable = false;
	bool result = false;
	CString strMsg;
	*Err = NO_ERR;
	AllEventID* pEventIdData = new AllEventID;
	EventControl::Event* EventInfo = new EventControl::Event();
	double eventTrq = 0.0;

	long ErrEvent = GetEventControl()->GetAllEventID(pEventIdData);
	long EventID;
	if (ErrEvent != ErrorCode::None)
	{
		EventControlErrorToString(ErrEvent, _T("GetAllEventCount"));
	}

	for (long cnt = 0; cnt < pEventIdData->count; ++cnt)
	{
		EventID = pEventIdData->id[cnt];
		bEventAvailable = GetEventAvailable(EventID);

		if (bEventAvailable == true)
		{
			GetEventInfo(EventID, EventInfo);
			eventTrq = EventInfo->input.greaterTrq.trq;

			if (EventID == EVENTID_FZ1_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ1;
				strMsg.Format(_T("FZ1 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FZ2_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ2;
				strMsg.Format(_T("FZ2 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FZ3_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ3;
				strMsg.Format(_T("FZ3 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FZ4_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ4;
				strMsg.Format(_T("FZ4 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FZ5_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ5;
				strMsg.Format(_T("FZ5 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FZ6_OVER_TORQUE && GetEventAvailable(EventID) == true)
			{
				*Err = MAX_TORQUE_LIMIT_FZ6;
				strMsg.Format(_T("FZ6 Over Torque Limit(%.1f)%%"), eventTrq);
			}
			else if (EventID == EVENTID_FX_FRONT_AREA_SENSOR && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_FRONT;
				//EnableEvent(EVENTID_FY1_FRONT_AREA_SENSOR);

				strMsg.Format(_T("Front Area Sensor Detected"));
			}
			else if (EventID == EVENTID_FY1_FRONT_AREA_SENSOR && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_FRONT;
				//EnableEvent(EVENTID_FX_FRONT_AREA_SENSOR);

				strMsg.Format(_T("Front Area Sensor Detected"));
			}
			else if (EventID == EVENTID_FX_REAR_AREA_SENSOR && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_REAR;
				//EnableEvent(EVENTID_FY1_REAR_AREA_SENSOR);

				strMsg.Format(_T("Rear Area Sensor Detected"));
			}
			else if (EventID == EVENTID_FY1_REAR_AREA_SENSOR && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_REAR;
				//EnableEvent(EVENTID_FX_REAR_AREA_SENSOR);

				strMsg.Format(_T("Rear Area Sensor Detected"));
			}
			else if (EventID == EVENTID_FX_FRONT_AREA_SENSOR_2ND && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_FRONT;
				//EnableEvent(EVENTID_FY1_FRONT_AREA_SENSOR);

				strMsg.Format(_T("Front Area Sensor2nd Detected"));
			}
			else if (EventID == EVENTID_FY1_FRONT_AREA_SENSOR_2ND && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_FRONT;
				//EnableEvent(EVENTID_FX_FRONT_AREA_SENSOR);

				strMsg.Format(_T("Front Area Sensor2nd Detected"));
			}
			else if (EventID == EVENTID_FX_REAR_AREA_SENSOR_2ND && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_REAR;
				//EnableEvent(EVENTID_FY1_REAR_AREA_SENSOR);

				strMsg.Format(_T("Rear Area Sensor2nd Detected"));
			}
			else if (EventID == EVENTID_FY1_REAR_AREA_SENSOR_2ND && GetEventAvailable(EventID) == true)
			{
				*Err = DETECTED_AREA_SENSOR_REAR;
				//EnableEvent(EVENTID_FX_REAR_AREA_SENSOR);

				strMsg.Format(_T("Rear Area Sensor2nd Detected"));
			}
			//EnableEvent(EventID);
		}
	}

	if (*Err == NO_ERR)
	{
		result = false;
	}
	else
	{
		//bool existEnable = false;
		//for (long retry = 0; retry < 20; retry++)
		//{
		//	for (long cnt = 0; cnt < pEventIdData->count; ++cnt)
		//	{
		//		EventID = pEventIdData->id[cnt];
		//		if (GetEventAvailable(EventID) == true)
		//		{
		//			TRACE(_T("[PWR] Try EnableEvent:%d\n"), EventID);

		//			existEnable = true;
		//			EnableEvent(EventID);
		//		}
		//	}
		//	if (existEnable == false)
		//	{
		//		break;
		//	}
		//	ThreadSleep(TIME10MS);
		//}

		TRACE(_T("[PWR] %s\n"), strMsg);
		if (UseSendAlarm == true)
		{
			*Err = SendAlarm(*Err, strMsg);
		}

		GantryEventEnable(FRONT_GANTRY);

		result = true;
	}
	delete pEventIdData;
	delete EventInfo;

	return result;

}

long CPowerGantry::GantryEventEnable(long Gantry)
{
	bool ExistAvailable = false;
	CString strMsg;
	AllEventID* pEventIdData = new AllEventID;
	long ErrEvent = GetEventControl()->GetAllEventID(pEventIdData);
	if (ErrEvent != ErrorCode::None)
	{
		EventControlErrorToString(ErrEvent, _T("GetAllEventCount"));
	}
	long EventID;

	for (long retry = 0; retry < 20; retry++)
	{
		ExistAvailable = false;
		for (long cnt = 0; cnt < pEventIdData->count; ++cnt)
		{
			EventID = pEventIdData->id[cnt];
			if (IsGantryEvent(Gantry, EventID) == true && GetEventAvailable(EventID) == true)
			{
				TRACE(_T("[PWR] GantryEventEnable Gantry:%d id:%d\n"), Gantry, EventID);
				EnableEvent(EventID);
				ExistAvailable = true;
			}
		}

		if (ExistAvailable == false)
		{
			TRACE(_T("[PWR] GantryEventEnable Gantry:%d clear(%d)\n"), Gantry, retry);
			break;
		}

		ThreadSleep(TIME10MS);
	}
	delete pEventIdData;

	return 0;
}

bool CPowerGantry::GetEventInfo(long Eventid, EventControl::Event* EventInfo)
{
	long Err = ErrorCode::None;
	int EventID = Eventid;

	Err = GetEventControl()->GetEvent(EventID, EventInfo);
	if (Err != ErrorCode::None)
	{
		EventControlErrorToString(Err, _T("GetEventInfo"));
		return false;
	}

	return true;
}

bool CPowerGantry::IsGantryEvent(long Gantry, long Eventid)
{
	bool result = false;

	if (Gantry == FRONT_GANTRY)
	{
		switch (Eventid)
		{
		case EVENTID_FZ1_OVER_TORQUE:
		case EVENTID_FZ2_OVER_TORQUE:
		case EVENTID_FZ3_OVER_TORQUE:
		case EVENTID_FZ4_OVER_TORQUE:
		case EVENTID_FZ5_OVER_TORQUE:
		case EVENTID_FZ6_OVER_TORQUE:
		case EVENTID_DUAL_FX_FRONT_AREA_SENSOR:
		case EVENTID_DUAL_FY1_FRONT_AREA_SENSOR:
		case EVENTID_DUAL_FX_REAR_AREA_SENSOR:
		case EVENTID_DUAL_FY1_REAR_AREA_SENSOR:
			result = true;
			break;
		default:
			break;
		}
	}
	else if (Gantry == REAR_GANTRY)
	{
		switch (Eventid)
		{
		case EVENTID_RZ1_OVER_TORQUE:
		case EVENTID_RZ2_OVER_TORQUE:
		case EVENTID_RZ3_OVER_TORQUE:
		case EVENTID_RZ4_OVER_TORQUE:
		case EVENTID_RZ5_OVER_TORQUE:
		case EVENTID_RZ6_OVER_TORQUE:
		case EVENTID_DUAL_RX_FRONT_AREA_SENSOR:
		case EVENTID_DUAL_RY1_FRONT_AREA_SENSOR:
		case EVENTID_DUAL_RX_REAR_AREA_SENSOR:
		case EVENTID_DUAL_RY1_REAR_AREA_SENSOR:
			result = true;
			break;
		default:
			break;
		}
	}

	return result;
}

long CPowerGantry::IsWaitOnePos(CString strAxis, double Cmd, double Inpos, long TimeOut, bool UseSendAlarm)
{
	Cwmx3Axis* pAxis;
	double Read = ReadOnePosition(strAxis);
	long Err = NO_ERR;
	bool bEventAvailable = false;
	CString strMsg;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] IsWaitOnePos(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
		return Err;
	}
	if (GetOnlyConveyorMode() == true)
	{
		return NO_ERR;
	}

	CoreMotionAxisStatus status;
	long remaintime = 0;
	
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();
	while (1)
	{
		if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
		{
			if (status.profileRemainingMilliseconds > TimeOut)
			{
				remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
				TRACE(_T("[PWR] IsWaitOnePos TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
				TimeOut = remaintime;
			}
		}
		
		if (pAxis->IsGantryAxis() == true || pAxis->IsZAxis() == true)
		{
			if (IsAllEventAvailable(&Err, UseSendAlarm) == true)
			{
				break;
			}
		}

		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			if (UseSendAlarm == true)
			{
				Err = SendAlarm(Err, strMsg);
			}
			break;
		}

		Read = ReadOnePosition(strAxis);
		if (abs(Cmd - Read) < Inpos)
		{
			Err = NO_ERR;
			break;
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] IsWaitOnePos(%s) Cmd:%.3f Read:%.3f Diff:%.3f\n"), strAxis, Cmd, Read, Cmd-Read);
		}
		ElapsedTime = _time_elapsed(GetTime);
		if(ElapsedTime > TimeOut)
		{
			Err = GetTimeOutError(strAxis);
			break;
		}
		ThreadSleep(WAIT_POSITIONSET_READTIME);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] IsWaitOnePos(%s) Elapsed:%d[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::WaitOneMotion(CString strAxis, double CmdPos, long TimeOut)
{
	Cwmx3Axis* pAxis;
	long Err = NO_ERR;
	double InPos = 0.1;
	bool bEventAvailable = false;
	CString strMsg;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitOneMotion(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
		return Err;
	}
	CoreMotionAxisStatus status;
	long remaintime = 0;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();
	pAxis->WaitMotion();
	while (pAxis->IsAxisIdle() == false && pAxis->IsAxisMotionComplete() == false)
	{
		if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
		{
			if (status.profileRemainingMilliseconds > TimeOut)
			{
				remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
				TRACE(_T("[PWR] WaitOneMotion TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
				TimeOut = remaintime;
			}
		}
		if (pAxis->IsGantryAxis() == true || pAxis->IsZAxis() == true)
		{
			if (IsAllEventAvailable(&Err) == true)
			{
				break;
			}
		}
		
		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			Err = SendAlarm(Err, strMsg);
			break;
		}
		ThreadSleep(WAIT_MOTION_READTIME);
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitOneMotion(%s) TimeOut\n"), pAxis->GetAxisName());
			Err = GetTimeOutError(strAxis);
			break;
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] WaitOneMotion(%s) Elapsed:%d[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::WaitOnePosSet(CString strAxis, double CmdPos, long TimeOut)
{
	Cwmx3Axis* pAxis;
	long Err = NO_ERR;
	bool bEventAvailable = false;
	CString strMsg;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitOnePosSet(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
		return Err;
	}
	CoreMotionAxisStatus status;
	long remaintime = 0;

	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();
	while (pAxis->IsPosSet() == false && pAxis->IsAxisMotionComplete() == false)
	{
		if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
		{
			if (status.profileRemainingMilliseconds > TimeOut)
			{
				remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
				TRACE(_T("[PWR] WaitOnePosSet TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
				TimeOut = remaintime;
			}
		}

		if (pAxis->IsGantryAxis() == true || pAxis->IsZAxis() == true)
		{
			if (IsAllEventAvailable(&Err) == true)
			{
				break;
			}
		}

		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			Err = SendAlarm(Err, strMsg);
			break;
		}
		ThreadSleep(WAIT_POSITIONSET_READTIME);
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitPosSet(%s) TimeOut(%d)\n"), strAxis, TimeOut);
			Err = GetTimeOutError(strAxis);
			break;
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] WaitPosSet(%s) Elapsed:%d[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::WaitOneInPos(CString strAxis, double CmdPos, long TimeOut)
{
	Cwmx3Axis* pAxis;
	long Err = NO_ERR;
	bool bEventAvailable = false;
	CString strMsg;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitOneInPos(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
		return Err;
	}
	if (GetOnlyConveyorMode() == true)
	{
		return NO_ERR;
	}
	CoreMotionAxisStatus status;
	long remaintime = 0;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();
	while (pAxis->IsInpos() == false)
	{
		if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
		{
			if (status.profileRemainingMilliseconds > TimeOut)
			{
				remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
				TRACE(_T("[PWR] WaitOneInPos TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
				TimeOut = remaintime;
			}
		}

		if (pAxis->IsGantryAxis() == true || pAxis->IsZAxis() == true)
		{
			if (IsAllEventAvailable(&Err) == true)
			{
				break;
			}
		}

		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			Err = SendAlarm(Err, strMsg);
			break;
		}
		ThreadSleep(WAIT_POSITIONSET_READTIME);
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitOneInPos(%s) TimeOut(%d)\n"), strAxis, TimeOut);
			Err = GetTimeOutError(strAxis);
			break;
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] WaitOneInPos(%s) Elapsed:%d[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::WaitOneDelayedPosSet(CString strAxis, double CmdPos, long TimeOut, bool UseSendAlarm)
{
	Cwmx3Axis* pAxis;
	long Err = NO_ERR;
	bool bEventAvailable = false;
	CString strMsg;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WaitOneDelayedPosSet(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
		return Err;
	}
	CoreMotionAxisStatus status;
	long remaintime = 0;

	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();
	while (pAxis->IsDelayedPosSet() == false && pAxis->IsAxisMotionComplete() == false)
	{
		if (remaintime == 0 && pAxis->GetCoreMotionAxisStatus(&status) == true)
		{
			if (status.profileRemainingMilliseconds > TimeOut)
			{
				remaintime = (long)(status.profileRemainingMilliseconds * 1.2);
				TRACE(_T("[PWR] WaitOneDelayedPosSet TimeOut Change(%s, %d->%d)\n"), strAxis, TimeOut, remaintime);
				TimeOut = remaintime;
			}
		}

		if (pAxis->IsGantryAxis() == true || pAxis->IsZAxis() == true)
		{
			if (IsAllEventAvailable(&Err, UseSendAlarm) == true)
			{
				break;
			}
		}

		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			if (UseSendAlarm == true)
			{
				Err = SendAlarm(Err, strMsg);
			}
			break;
		}
		ThreadSleep(WAIT_DELAYEDPOSITIONSET_READTIME);
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitOneDelayedPosSet(%s) TimeOut\n"), pAxis->GetAxisName());
			Err = GetTimeOutError(strAxis);
			break;
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] WaitOneDelayedPosSet(%s) Elapsed:%d[ms]\n"), strAxis, ElapsedTime);
	}
	return Err;
}

long CPowerGantry::WaitAllZIdle(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	CString strAxis;
	for (INT_PTR index = 0; index < GetZAxisCount(); ++index)
	{
		strAxis = GetZAxisByIndex(index);
		Err = WaitOneIdle(strAxis, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] WaitAllZIdle %s WaitOneIdle Err:%d\n"), strAxis, Err);
			return Err;
		}
	}
	return Err;
}

long CPowerGantry::WaitAllRIdle(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	CString strAxis;
	for (INT_PTR index = 0; index < GetRAxisCount(); ++index)
	{
		strAxis = GetRAxisByIndex(index);
		Err = WaitOneIdle(strAxis, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] WaitAllRIdle %s WaitOneIdle Err:%d\n"), strAxis, Err);
			return Err;
		}
	}
	return Err;
}

long CPowerGantry::WaitGantryIdle(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneIdle(GetAxisX(Gantry), TimeOut);
	Err = WaitOneIdle(GetAxisY1(Gantry), TimeOut);
	return Err;
}

bool CPowerGantry::AlarmClear(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] AlarmClear(%s) is NULL\n"), strAxis);
		return false;
	}
	long lTimeChk = 0;
	CApplicationTime* pTime = new CApplicationTime();
	while (1)
	{
		if (pAxis->CheckAmpAlarm() == true)
		{
			pAxis->ClearAmpAlarm();
			ThreadSleep(TIME10MS);
			pAxis->ClearAxisAlarm();
		}
		lTimeChk = pTime->TimeElapsed();
		if(lTimeChk > TIME3000MS)
		{
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Alarm Clear(%s) Elapsed:%d[ms]\n"), strAxis, lTimeChk);
	}
	delete pTime;
	return true;
}

bool CPowerGantry::ClearSlaveAmpAlarm(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ClearSlaveAmpAlarm(%s) is NULL\n"), strAxis);
		return false;
	}
	long lTimeChk = 0;
	CApplicationTime* pTime = new CApplicationTime();
	while (1)
	{
		if (pAxis->CheckSlaveAmpAlarm() == true)
		{
			pAxis->ClearSlaveAmpAlarm();
			pAxis->ClearSlaveAxisAlarm();
		}
		lTimeChk = pTime->TimeElapsed();
		if (lTimeChk > TIME3000MS)
		{
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Slave Alarm Clear(%s) Elapsed:%d[ms]\n"), strAxis, lTimeChk);
	}
	delete pTime;
	return true;
}

bool CPowerGantry::CheckAmpAlarm(CString strAxis)
{
	Cwmx3Axis* pAxis;
	bool bRet = false;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] CheckAmpAlarm(%s) is NULL\n"), strAxis);
		return bRet;
	}
	bRet = pAxis->CheckAmpAlarm();
	return bRet;
}

bool CPowerGantry::CheckSlaveAmpAlarm(CString strAxis)
{
	Cwmx3Axis* pAxis;
	bool bRet = false;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] CheckSlaveAmpAlarm(%s) is NULL\n"), strAxis);
		return bRet;
	}
	bRet = pAxis->CheckSlaveAmpAlarm();
	return bRet;
}

bool CPowerGantry::CheckServoOn(CString strAxis)
{
	Cwmx3Axis* pAxis;
	bool bRet = false;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] CheckServoOn(%s) is NULL\n"), strAxis);
		return bRet;
	}
	bRet = pAxis->CheckServoOn();
	return bRet;
}

bool CPowerGantry::CheckSlaveServoOn(CString strAxis)
{
	Cwmx3Axis* pAxis;
	bool bRet = false;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] CheckSlaveServoOn(%s) is NULL\n"), strAxis);
		return bRet;
	}
	bRet = pAxis->CheckSlaveServoOn();
	return bRet;
}

long CPowerGantry::GetSlaveAxisIndex(CString strAxis)
{
	Cwmx3Axis* pAxis;
	long index = 0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetSlaveAxisIndex(%s) is NULL\n"), strAxis);
		return index;
	}
	index = pAxis->GetSlaveAxisIndex();
	return index;
}

long CPowerGantry::GetAxisID(CString strAxis)
{
	Cwmx3Axis* pAxis;
	long axisID = false;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetAxisID(%s) is NULL\n"), strAxis);
		return axisID;
	}
	axisID = pAxis->GetAxisMap();
	return axisID;
}

long CPowerGantry::GetSlaveAxisSlaveID(CString strAxis)
{
	Cwmx3Axis* pAxis;
	long slaveAxisID = 0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetSlaveAxisSlaveID(%s) is NULL\n"), strAxis);
		return slaveAxisID;
	}
	slaveAxisID = pAxis->GetSlaveAxisSlaveID();
	return slaveAxisID;
}

bool CPowerGantry::ServoOn(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ServoOn(%s) is NULL\n"), strAxis);
		return false;
	}
	long lTimeChk = 0;
	CApplicationTime* pTime = new CApplicationTime();
	while (1)
	{
		if (pAxis->CheckServoOn() == false)
		{
			pAxis->ServoOn();
		}
		else
		{
			lTimeChk = pTime->TimeElapsed();			
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Servo On(%s) Elapsed:%d[ms]\n"), strAxis, lTimeChk);
	}
	delete pTime;
	return true;
}

bool CPowerGantry::SlaveServoOn(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SlaveServoOn(%s) is NULL\n"), strAxis);
		return false;
	}
	long lTimeChk = 0;
	CApplicationTime* pTime = new CApplicationTime();
	while (1)
	{
		if (pAxis->CheckSlaveServoOn() == false)
		{
			pAxis->SlaveServoOn();
		}
		else
		{
			lTimeChk = pTime->TimeElapsed();
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] SlaveServoOn Off(%s) Elapsed:%d[ms]\n"), strAxis, lTimeChk);
	}
	delete pTime;
	return true;
}

void CPowerGantry::TryAlarmClearAllAxis()
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;

	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);

		pAxis->ClearAmpAlarm();
		pAxis->ClearAxisAlarm();
	}
}

bool CPowerGantry::IsAlarmClearAllAxis(long* clearFailAxisNo)
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	long alarmCode = 0;
	bool alarmStatus = false;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		alarmCode = pAxis->CheckAmpAlarmCode();
		alarmStatus = pAxis->CheckAmpAlarm();

		if (alarmStatus == true || alarmCode != ErrorCode::None)
		{
			*clearFailAxisNo = pAxis->GetAxisIndex();
			return false;
		}
	}

	return true;
}

void CPowerGantry::TrayServoOnAllAxis()
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;

	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		pAxis->ServoOn();
		ThreadSleep(TIME2MS);
	}
}

bool CPowerGantry::IsServoOnAllAxis(long* servoOnFailAxisNo)
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;

	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis->CheckServoOn() == false)
		{
			*servoOnFailAxisNo = pAxis->GetAxisIndex();
			return false;
		}
	}

	return true;
}

bool CPowerGantry::CheckOverloadAlarm()
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	long alarmCode = 0;
	bool alarmStatus = false;

	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);

		if (pAxis->IsConveyorBeltAxis() == true || pAxis->IsPusherZAxis() == true)
		{
			continue;
		}

		alarmCode = pAxis->CheckAmpAlarmCode();
		alarmStatus = pAxis->CheckAmpAlarm();

		if (alarmCode == GetAlarmCodeOverload()) // 파나소닉 드라이버 구분이 필요한데 이거....
		{
			return true;
		}
	}

	return false;
}

long CPowerGantry::ServoAllOn()
{
	long Err = NO_ERR;
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	long TimeOut = TIME10000MS;
	ULONGLONG GetTime = 0, Elapsed = 0;
	bool AllServoOn = true;
	long maxRetryClear = 10;
	long maxRetryServoOn = 20;
	CString strMsg = _T("");
	long TimeOutAlarmClear = TIME2000MS;

	if (GetOnlyConveyorMode() == true)
	{
		for (index = 0; index < GetWmx3AxisCount(); ++index)
		{
			pAxis = g_pWmx3AxisArray.GetAt(index);
			if (pAxis->IsConveyorBeltAxis() == true)
			{
				pAxis->ClearAmpAlarm();
				ThreadSleep(TIME10MS);
				pAxis->ClearAxisAlarm();
				ThreadSleep(TIME200MS);
				pAxis->ServoOn();
			}
		}

		return Err;
	}
	
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);

		if (pAxis->IsGantryAxis() == false || pAxis->CheckServoOn() == true)
		{
			continue;
		}

		//if (IsNearSWLimitMinus(pAxis->GetAxisName()) == true || IsNearSWLimitPlus(pAxis->GetAxisName()) == false)
		if (IsDangerSWLimit(pAxis->GetAxisName()) == true)
		{
			TRACE(_T("[PWR] %s Dangerous Position.\n"), pAxis->GetAxisName());
			strMsg.Format(_T("%s Dangerous Position"), (LPCTSTR)pAxis->GetAxisName());
			Err = SERVO_ON_DANGER_POSITION(pAxis->GetAxisIndex());
			Err = SendAlarm(Err, strMsg);
			return Err;
		}		
	}


	if (Get1DCompensationUse() == true)
	{
		oneDCompensationOff();
	}
	if (Get2DCompensationUse() == true)
	{
		twoDCompensationOff();
	}
	ThreadSleep(TIME100MS);

	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis->GetInitializeEnd() == false)
		{
			Err = HOMING_TIMOUT(pAxis->GetAxisIndex());
			TRACE(_T("[PWR] %s Homing Failed\n"), pAxis->GetAxisName());
			strMsg.Format(_T("%s Homing Failed"), (LPCTSTR)pAxis->GetAxisName());
			Err = SendAlarm(Err, strMsg);
			return Err;
		}
	}

	TRACE(_T("[PWR] Alarm Clear Start.\n"));

	int alarmCode = 0;
	bool alarmStatus = false;
	long ClearTry = 0;
	bool alarmExist = false;
	long maxTimeAlarmClear = false;
	long clearFailAxisNo = 0; 
	long servoOnFailAxisNo = 0; 

	if (CheckOverloadAlarm() == true)
	{
		SendPopupMessage(_T("Wait Alarm Clear"));
		TimeOutAlarmClear = TIME10000MS + TIME1000MS;
		TRACE(_T("[PWR] Alarm Clear TimeOut %dms. (Overload Alarm)\n"), TimeOutAlarmClear);
	}

	GetTime = _time_get();

	while (1)
	{
		TryAlarmClearAllAxis();

		ThreadSleep(TIME100MS);		

		if (IsAlarmClearAllAxis(&clearFailAxisNo) == true)
		{
			break;
		}

		Err = ALARM_CLEAR_FAIL(clearFailAxisNo);		

		if (_time_elapsed(GetTime) > TimeOutAlarmClear)
		{
			TRACE(_T("[PWR] Alarm Clear Fail %s\n"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			strMsg.Format(_T("Alarm Clear Fail. (%s)"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			Err = SendAlarm(Err, strMsg);
			return Err;
		}
	}

	TRACE(_T("[PWR] Alarm Clear Complete, Servo On Start\n"));

	SendPopupMessage(_T("Wait Servo On."));

	GetTime = _time_get();

	while (1) 
	{
		Err = NO_ERR;
		TrayServoOnAllAxis();

		ThreadSleep(TIME100MS);

		if (IsServoOnAllAxis(&servoOnFailAxisNo) == true)
		{
			break;
		}	

		if (IsAlarmClearAllAxis(&clearFailAxisNo) == false)
		{
			TRACE(_T("[PWR] Alarm Clear Fail %s\n"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			strMsg.Format(_T("Alarm Clear Fail. (%s)"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			Err = ALARM_CLEAR_FAIL(clearFailAxisNo);
			Err = SendAlarm(Err, strMsg);
			return Err;
		}

		if (_time_elapsed(GetTime) > TIME5000MS)
		{
			TRACE(_T("[PWR] Servo On Fail %s\n"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			strMsg.Format(_T("Servo On Fail. (%s)"), (LPCTSTR)GetAxisNameByAxisIndex(clearFailAxisNo));
			Err = SERVO_ON_TIMEOUT(servoOnFailAxisNo);
			Err = SendAlarm(Err, strMsg);
			return Err;
		}
	}

	ThreadSleep(TIME100MS);	// 신규 추가

	if (GetY2CompensationUse() == true)
	{
		Err = RestoreY1Y2DifferentByPosCmd();
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Y2 Restore Fail\n"));
			strMsg.Format(_T("Y2 Restore Fail"));
			Err = SendAlarm(Err, strMsg);
			return Err;
		}
	}

	ThreadSleep(TIME100MS); // 신규 추가

	GetTime = _time_get();
	Err = SetSyncMasterSlave();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		Elapsed = _time_elapsed(GetTime);
		TRACE(_T("[PWR] SetSyncMasterSlave Elapsed:%d[ms]\n"), Elapsed);
	}

	ThreadSleep(TIME100MS);

	if (Get1DCompensationUse() == true)
	{
		oneDCompensationOn();
	}
	if (Get2DCompensationUse() == true)
	{
		twoDCompensationOn();
	}

	return Err;
}

bool CPowerGantry::ServoOnWithAlarmClear(CString strAxis)
{
	long Err = NO_ERR;
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	long TimeOut = TIME10000MS;
	ULONGLONG GetTime = 0, Elapsed = 0, RetryTime = 0;

	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ServoOnWithAlarmClear(%s) is NULL\n"), strAxis);
		return false;
	}

	pAxis->ClearAmpAlarm();
	ThreadSleep(TIME2MS);
	pAxis->ClearAxisAlarm();
	ThreadSleep(WAIT_SERVOON_WAITTIME);
	pAxis->ServoOn();

	GetTime = _time_get();
	RetryTime = _time_get();

	while (1)
	{
		if (pAxis->CheckServoOn() == true)
		{
			break;
		}
		else
		{
			pAxis->ClearAmpAlarm();
			ThreadSleep(TIME2MS);
			pAxis->ClearAxisAlarm();

			ThreadSleep(WAIT_SERVOON_WAITTIME);
			pAxis->ServoOn();

		}

		ThreadSleep(WAIT_SERVOON_WAITTIME);
		Elapsed = _time_elapsed(GetTime);
		if (TimeOut < Elapsed)
		{
			Err = SERVO_ON_TIMEOUT(pAxis->GetAxisIndex());
			TRACE(_T("[PWR] %s Servo On TimeOut(%d)\n"), pAxis->GetAxisName(), Elapsed);
			break;
		}
	}

	if (Err != NO_ERR)
	{
		return Err;
	}

	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] %s Servo On Elapsed:%d[ms]\n"), strAxis, Elapsed);
	}

	return Err;
}

bool CPowerGantry::ServoOff(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ServoOff(%s) is NULL\n"), strAxis);
		return false;
	}
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	while (1)
	{
		if (pAxis->CheckServoOn() == true)
		{
			pAxis->ServoOff();
		}
		else
		{
			Elapsed = _time_elapsed(GetTime);
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Servo Off(%s) Elapsed:%d[ms]\n"), strAxis, Elapsed);
	}
	return true;
}

bool CPowerGantry::SlaveServoOff(CString strAxis)
{
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SlaveServoOff(%s) is NULL\n"), strAxis);
		return false;
	}
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	while (1)
	{
		if (pAxis->CheckSlaveServoOn() == true)
		{
			pAxis->SlaveServoOff();
		}
		else
		{
			Elapsed = _time_elapsed(GetTime);
			break;
		}
		ThreadSleep(TIME10MS);
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] SlaveServoOff Off(%s) Elapsed:%d[ms]\n"), strAxis, Elapsed);
	}
	return true;

}

long CPowerGantry::ServoAllOff()
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	double Ratio = 0.5;
	long TimeOut = TIME5000MS, Err = NO_ERR;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis != NULL)
		{
			pAxis->ServoOff();
		}
	}
	ULONGLONG AllGetTime = _time_get(), GetTime = _time_get(), Elapsed = 0;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		GetTime = _time_get();
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis != NULL)
		{
			while (1)
			{
				ThreadSleep(WAIT_SERVOOFF_WAITTIME);
				if (pAxis->CheckServoOn() == false)
				{
					break;
				}
				Elapsed = _time_elapsed(GetTime);
				if (TimeOut < Elapsed)
				{
					Err = SERVO_OFF_TIMEOUT(pAxis->GetAxisIndex());
					TRACE(_T("[PWR] %s Servo Off TimeOut(%d)\n"), pAxis->GetAxisName(), Elapsed);
					break;
				}
			}
		}
		if (Err != NO_ERR)
		{
			break;
		}
	}
	if (Err != NO_ERR)
	{
		return Err;
	}
	Elapsed = _time_elapsed(AllGetTime);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Servo All Off Elapsed:%d[ms]\n"), Elapsed);
	}
	return Err;
}

long CPowerGantry::ServoAllOffWithoutConvAxis()
{
	INT_PTR index = 0;
	Cwmx3Axis* pAxis = NULL;
	double Ratio = 0.5;
	long TimeOut = TIME5000MS, Err = NO_ERR;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis != NULL)
		{
			if (pAxis->IsConveyorAxis() == true || pAxis->IsPusherZAxis() == true)
			{
				TRACE(_T("[PWR] %s ConvAxis Servo Off Skip\n"), pAxis->GetAxisName());
				continue;
			}

			pAxis->ServoOff();
		}
	}
	ULONGLONG AllGetTime = _time_get(), GetTime = _time_get(), Elapsed = 0;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		GetTime = _time_get();
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis != NULL)
		{
			if (pAxis->IsConveyorAxis() == true || pAxis->IsPusherZAxis() == true) // 20210429 HarkDo
			{
				TRACE(_T("[PWR] %s ConvAxis Servo Off Skip to Check\n"), pAxis->GetAxisName());
				continue;
			}

			while (1)
			{
				ThreadSleep(WAIT_SERVOOFF_WAITTIME);
				if (pAxis->CheckServoOn() == false)
				{
					break;
				}
				Elapsed = _time_elapsed(GetTime);
				if (TimeOut < Elapsed)
				{
					Err = SERVO_OFF_TIMEOUT(pAxis->GetAxisIndex());
					TRACE(_T("[PWR] %s Servo Off TimeOut(%d)\n"), pAxis->GetAxisName(), Elapsed);
					break;
				}
			}
		}
		if (Err != NO_ERR)
		{
			break;
		}
	}
	if (Err != NO_ERR)
	{
		return Err;
	}
	Elapsed = _time_elapsed(AllGetTime);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Servo All Off Elapsed:%d[ms]\n"), Elapsed);
	}
	return Err;
}

bool CPowerGantry::IsGantryAxis(CString strAxis)
{
	INT_PTR indx = 0;
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis->IsGantryAxis() == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CPowerGantry::IsUseSlaveAxis(CString strAxis)
{
	INT_PTR indx = 0;
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis->GetUseSlaveAxis() == true)
	{
		return true;
	}
	else
	{
		return false;
	}
}

long CPowerGantry::SetSyncMasterSlave()
{
	long err[MAXAXISNO], Err = NO_ERR;
	ZeroMemory(&err, sizeof(err));
	INT_PTR indx = 0;
	Cwmx3Axis* pAxis = NULL;
	WMX3AxisSyncMode syncMode = WMX3AxisSyncMode::NoSync;
	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		pAxis = GetWmx3AxisByIndex(indx);
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			TRACE(_T("[PWR] GetUseSlaveAxis-1 %s SyncMode:%d\n"), pAxis->GetAxisName(), syncMode);
			if (syncMode == WMX3AxisSyncMode::NoSync)
			{
				err[indx] = pAxis->SetSyncMasterSlave();
				TRACE(_T("[PWR] SetSyncMasterSlave %s Ret:%d\n"), pAxis->GetAxisName(), err[indx]);
				ThreadSleep(TIME100MS);
				syncMode = pAxis->GetAxisSyncMode();
				TRACE(_T("[PWR] GetUseSlaveAxis-2 %s SyncMode:%d\n"), pAxis->GetAxisName(), syncMode);
			}
		}
	}
	for (indx = 0; indx < MAXAXISNO; indx++)
	{
		if (err[indx] != NO_ERR)
		{
			return Err;
		}
	}
	return Err;
}

long CPowerGantry::RestoreY1Y2Different()
{
	long err[MAXAXISNO], Err = NO_ERR;
	ZeroMemory(&err, sizeof(err));
	INT_PTR indx = 0;
	Cwmx3Axis* pAxis = NULL;
	WMX3AxisSyncMode syncMode = WMX3AxisSyncMode::NoSync;
	long Y1, Y2, HomeDiff, FinalDiff;
	double pNumerator, pDenominator;
	double Y2TartPosition, Y2CurrentPosition, Y2MoveDelta;
	Motion::PosCommand* posCmd = new Motion::PosCommand();

	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		pAxis = GetWmx3AxisByIndex(indx);
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			TRACE(_T("[PWR] RestoreY1Y2Different %s start\n"), pAxis->GetAxisName());
			if (syncMode == WMX3AxisSyncMode::NoSync && pAxis->CheckServoOn() == true && pAxis->CheckSlaveServoOn() == true)
			{
				Y1 = pAxis->GetEncoderFeedBack();
				Y2 = pAxis->GetSlaveEncoderFeedBack();
				HomeDiff = gcPowerCalibrationData->GetYHomeEncoderDifferent();
				FinalDiff = (Y1 - Y2) - HomeDiff;

				TRACE(_T("[PWR] RestoreY1Y2Different %s Y1,Y2,Home,Final %d %d %d %d \n"), pAxis->GetAxisName(), Y1, Y2, HomeDiff, FinalDiff);

				GetCoreMotion()->config->GetGearRatio(pAxis->GetAxisMap(), &pNumerator, &pDenominator);
				TRACE(_T("[PWR] RestoreY1Y2Different %s Gear Numerator Denominator %.3f %.3f \n"), pAxis->GetAxisName(), pNumerator, pDenominator);
				Y2MoveDelta = ((FinalDiff * pDenominator) / pNumerator) * pAxis->GetResol();
				pAxis->SetRatio(0.1);
				posCmd->axis = pAxis->GetSlaveAxisSlaveID();
				posCmd->profile.velocity = 10000.0;
				posCmd->profile.acc = 10000.0;
				posCmd->profile.dec = 10000.0;

				posCmd->target = (Y2MoveDelta);

				TRACE(_T("[PWR] RestoreY1Y2Different %s SlaveMap %d Relative Move %.3f\n"), pAxis->GetAxisName(), posCmd->axis, posCmd->target);

				if (fabs(Y2MoveDelta) < 0.5)
				{
					Err = pAxis->StartSlaveMove(posCmd, posCmd->target);
					Y2TartPosition = pAxis->ReadSlsaveMotorPosition();
					TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d Start:%.3f\n"), pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2TartPosition);
					while (true)
					{
						Y2CurrentPosition = pAxis->ReadSlsaveMotorPosition();
						if (fabs(((Y2TartPosition + Y2MoveDelta) - Y2CurrentPosition)) <= 0.002)
						{
							break;
						}
						else
						{
							TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d Current:%.3f Target:%.3f\n"), pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2CurrentPosition,
								Y2TartPosition + Y2MoveDelta);
						}
						ThreadSleep(TIME10MS);
					};
					TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d End:%.3f\n"), pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2CurrentPosition);
					ThreadSleep(TIME200MS);
					TRACE(_T("[PWR] %s StartMove Target:%.3f Err:%d\n"), pAxis->GetAxisName(), posCmd->target, Err);
					Y1 = pAxis->GetEncoderFeedBack();
					Y2 = pAxis->GetSlaveEncoderFeedBack();

					TRACE(_T("[PWR] RestoreY1Y2Different %s Y1,Y2,Home,Final %d %d %d %d \n"), pAxis->GetAxisName(), Y1, Y2, HomeDiff, FinalDiff);
				}
				else
				{
					TRACE(_T("[PWR] RestoreY1Y2Different %s Skip. Y2 Delta %.3f is too big. \n"), pAxis->GetAxisName(), Y2MoveDelta);
				}
			}
		}
	}
	delete posCmd;
	return Err;

}

long CPowerGantry::RestoreY1Y2DifferentByPosCmd()
{
	long err[MAXAXISNO], Err = NO_ERR;
	ZeroMemory(&err, sizeof(err));
	INT_PTR indx = 0;
	Cwmx3Axis* pAxis = NULL;
	WMX3AxisSyncMode syncMode = WMX3AxisSyncMode::NoSync;
	double Y1, Y2, FinalDiff;
	double Y2StartPosition, Y2CurrentPosition;
	Motion::PosCommand* posCmd = new Motion::PosCommand();
	ULONGLONG GetTime = 0;

	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		pAxis = GetWmx3AxisByIndex(indx);
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s start\n"), pAxis->GetAxisName());
			if (syncMode == WMX3AxisSyncMode::NoSync && pAxis->CheckServoOn() == true && pAxis->CheckSlaveServoOn() == true)
			{
				Y1 = pAxis->ReadCommandPosition();
				Y2 = pAxis->ReadSlaveCommandPosition();
				FinalDiff = (Y1 - Y2);

				TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s Y1,Y2,Diff,%.3f,%.3f,%.3f\n"), pAxis->GetAxisName(), Y1, Y2, FinalDiff);

				//GetCoreMotion()->config->GetGearRatio(pAxis->GetAxisMap(), &pNumerator, &pDenominator);
				//TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s Gear Numerator Denominator %.3f %.3f \n"), pAxis->GetAxisName(), pNumerator, pDenominator);
				//Y2MoveDelta = ((FinalDiff * pDenominator) / pNumerator) * pAxis->GetResol();
				pAxis->SetRatio(0.1);
				posCmd->axis = pAxis->GetSlaveAxisSlaveID();
				posCmd->profile.velocity = 10000.0;
				posCmd->profile.acc = 100000.0;
				posCmd->profile.dec = 100000.0;

				posCmd->target = (Y1);

				TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s SlaveMap %d Pos Move %.3f\n"), pAxis->GetAxisName(), posCmd->axis, posCmd->target);

				if (fabs(FinalDiff) < 0.5)
				{
					Err = pAxis->StartSlavePos(posCmd, posCmd->target);
					Y2StartPosition = pAxis->ReadSlaveCommandPosition();
					TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d Start:%.3f\n"), pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2StartPosition);
					
					GetTime = _time_get();
					while (true)
					{
						Y2CurrentPosition = pAxis->ReadSlaveCommandPosition();
						if (fabs((Y1 - Y2CurrentPosition)) <= 0.002)
						{
							break;
						}
						else
						{
							TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d Current:%.3f Target:%.3f\n"),
								pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2CurrentPosition, Y1);
						}

						if (_time_elapsed(GetTime) > TIME10000MS)
						{
							Err = TIMEOUT_FRONTY2COMPEN;
							delete posCmd;
							return Err;
						}

						ThreadSleep(TIME10MS);
					};
					TRACE(_T("[PWR] %s IsSlaveAxisIdle:%d IsSlaveAxisMotionComplete:%d End:%.3f\n"), pAxis->GetAxisName(), pAxis->IsSlaveAxisIdle(), pAxis->IsSlaveAxisMotionComplete(), Y2CurrentPosition);
					ThreadSleep(TIME200MS);
					TRACE(_T("[PWR] %s StartMove Target:%.3f Err:%d\n"), pAxis->GetAxisName(), posCmd->target, Err);
					Y1 = pAxis->ReadCommandPosition();
					Y2 = pAxis->ReadSlaveCommandPosition();
					TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s Y1,Y2,Final %.3f %.3f %.3f\n"), pAxis->GetAxisName(), Y1, Y2, Y1-Y2);
				}
				else
				{
					TRACE(_T("[PWR] RestoreY1Y2DifferentByPosCmd %s Skip. Y2 Delta %.3f is too big. \n"), pAxis->GetAxisName(), FinalDiff);
				}
			}
		}
	}
	delete posCmd;
	return Err;
}

long CPowerGantry::GetAlarmCodeOverload()
{
	// 파나소닉
	return 0xFF10;
}

double CPowerGantry::GetUnResol(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] GetUnResol %s is NON\n"), strAxis);
		return Position;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetUnResol(%s) is NULL\n"), strAxis);
		return Position;
	}
	Position = pAxis->GetUnResol();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] GetUnResol %s %.4f\n"), strAxis, Position);
	}
	return Position;
}

double CPowerGantry::ReadOnePosition(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0, Compensation = 0.0;
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] ReadOnePosition %s is NON\n"), strAxis);
		return Position;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadOnePosition %s is NULL\n"), strAxis);
		return Position;
	}
	Position = pAxis->ReadMotorPosition();
	if (pAxis->IsGantryAxis() == true)
	{
		Compensation = Read2DCompensationData(strAxis);
		//Position = Position + Compensation;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] ReadOnePosition %s %.3f Compensation(%.3f)\n"), strAxis, Position, Compensation);
		}
	}
	else
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] ReadOnePosition %s %.3f\n"), strAxis, Position);
		}
	}
	return Position;
}

double CPowerGantry::ReadCommandPosition(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] ReadCommandPosition %s is NON\n"), strAxis);
		return Position;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadCommandPosition %s is NULL\n"), strAxis);
		return Position;
	}
	Position = pAxis->ReadCommandPosition();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadCommandPosition %s %.3f\n"), strAxis, Position);
	}
	return Position;
}

double CPowerGantry::ReadOneCommandVelocity(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Velocity = 0.0;

	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] ReadOneCommandVelocity %s is NON\n"), strAxis);
		return Velocity;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadOneCommandVelocity %s is NULL\n"), strAxis);
		return Velocity;
	}
	Velocity = pAxis->ReadCommandVelocity();

	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadOneCommandVelocity %s %.3f\n"), strAxis, Velocity);
	}

	return Velocity;
}

double CPowerGantry::ReadProfileTargetPosition(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] ReadProfileTargetPosition %s is NON\n"), strAxis);
		return Position;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadProfileTargetPosition %s is NULL\n"), strAxis);
		return Position;
	}
	Position = pAxis->ReadProfileTargetPosition();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadProfileTargetPosition %s %.3f\n"), strAxis, Position);
	}
	return Position;
}

long CPowerGantry::WriteOnePosition(CString strAxis, double Position)
{
	Cwmx3Axis* pAxis;
	long Err = ErrorCode::None;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] WriteOnePosition %s is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else
	{
		Err = pAxis->SetMotorPosition(Position);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WriteOnePosition %s %.3f Err:%d\n"), strAxis, Position, Err);
		}

		if (pAxis->IsConveyorAxis() == true)
		{
			WriteWidth(FRONT_CONV, WORK1_CONV, Position);
		}
	}
	return Err;
}

double CPowerGantry::ReadMotorActualPosition(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadMotorActualPosition %s is NULL\n"), strAxis);
		return Position;
	}
	Position = pAxis->ReadMotorActualPosition();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadMotorActualPosition %s %.3f\n"), strAxis, Position);
	}
	return Position;
}

double CPowerGantry::ReadActualTorque(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Torque = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadActualTorque %s is NULL\n"), strAxis);
		return Torque;
	}
	Torque = pAxis->ReadActualTorque();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadActualTorque %s %.3f\n"), strAxis, Torque);
	}
	return Torque;
}

double CPowerGantry::Read1DCompensationData(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Compensation = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] Read1DCompensationData %s is NULL\n"), strAxis);
		return Compensation;
	}
	Compensation = pAxis->Read1DCompensationData() * pAxis->GetResol();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Read1DCompensationData %s %.3f Resol %.4f\n"), strAxis, Compensation, pAxis->GetResol());
	}
	return Compensation;
}

double CPowerGantry::Read2DCompensationData(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Compensation = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] Read2DCompensationData %s is NULL\n"), strAxis);
		return Compensation;
	}
	Compensation = pAxis->Read2DCompensationData() * pAxis->GetResol();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] Read2DCompensationData %s %.3f Resol %.4f\n"), strAxis, Compensation, pAxis->GetResol());
	}
	return Compensation;
}

double CPowerGantry::ReadVirtualPositiveLimit(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadVirtualPositiveLimit(%s) is NULL\n"), strAxis);
		return Position;
	}
	pAxis->ReadVirtualPositiveLimit(&Position);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadVirtualPositiveLimit %s %.3f\n"), strAxis, Position);
	}
	return Position;
}

double CPowerGantry::ReadVirtualNegativeLimit(CString strAxis)
{
	Cwmx3Axis* pAxis;
	double Position = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] ReadVirtualNegativeLimit(%s) is NULL\n"), strAxis);
		return Position;
	}
	pAxis->ReadVirtualNegativeLimit(&Position);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadVirtualNegativeLimit %s %.3f\n"), strAxis, Position);
	}
	return Position;
}

bool CPowerGantry::IsNegativeLimitSwitchOn(CString strAxis)
{
	bool bRet = false;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] IsNegativeLimitSwitchOn(%s) is NULL\n"), strAxis);
		return false;
	}
	bRet = pAxis->IsNegativeLimitSwitchOn();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] IsNegativeLimitSwitchOn %s %d\n"), strAxis, bRet);
	}
	return bRet;
}

bool CPowerGantry::IsPositiveLimitSwitchOn(CString strAxis)
{
	bool bRet = false;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] IsPositiveLimitSwitchOn(%s) is NULL\n"), strAxis);
		return false;
	}
	bRet = pAxis->IsPositiveLimitSwitchOn();
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] IsPositiveLimitSwitchOn %s %d\n"), strAxis, bRet);
	}
	return bRet;
}

long CPowerGantry::GetOneOnlyIdle(CString strAxis)
{
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] GetOneIdle(%s) is NULL\n"), strAxis);
		Err = GetAxisNullError(strAxis);
	}
	else if (pAxis->IsAxisIdle() == false)
	{
		Err = GetTimeOutError(strAxis);
	}

	return Err;
}
long CPowerGantry::StartMultiPosition(SomeTarget Target)
{
	long Err = NO_ERR, AxisIndex = 0;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartMultiPosition Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		Err = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		Cwmx3Axis* pAxis;
		for (long AxisNo = 0; AxisNo < Target.MaxAxisCount; ++AxisNo)
		{
			pAxis = GetWmx3AxisByName(Target.Axis[AxisNo]);
			if (pAxis == NULL)
			{
				TRACE(_T("[PWR] StartMultiPosition(%s,%.1f) is NULL\n"), Target.Axis[AxisNo], Target.Command[AxisNo]);
				Err = GetAxisNullError(Target.Axis[AxisNo]);
			}
			else
			{
				if (pAxis->GetInitializeEnd() == false)
				{
					Err = HOMING_FAIL(pAxis->GetAxisIndex());
					TRACE(_T("[PWR] StartMultiPosition GetInitializeEnd Err:%d\n"), Err);
					CString strMsg;
					strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)Target.Axis[AxisNo]);
					Err = SendAlarm(Err, strMsg);
					return Err;
				}
			}
		}
		
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPosition CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			//for (long AxisNo = 0; AxisNo < Target.MaxAxisCount; ++AxisNo)
			//{
			//	AxisIndex = GetAxisIndexFromAliasName(Target.Axis[AxisNo]);
			//	if (AxisIndex == static_cast<int>(PowerAxis::FX) || AxisIndex == static_cast<int>(PowerAxis::RX) || AxisIndex == static_cast<int>(PowerAxis::FY1) || AxisIndex == static_cast<int>(PowerAxis::RY1))
			//	{
			//		if (gcPowerLog->IsShowMotionLockLog() == true)
			//		{
			//			TRACE(_T("[PWR] StartMultiPosition AxisIndex(%d) Lock-1\n"), AxisIndex);
			//		}
			//		SEM_LOCK(gMOTION_LOCK[AxisIndex], INFINITE);
			//	}
			//}

			for (long AxisNo = 0; AxisNo < Target.MaxAxisCount; ++AxisNo)
			{
				if (IsMoveOnceAxis(Target.Axis[AxisNo]) == true)
				{
					Err = MoveOnceLockPauseMode(Target.Axis[AxisNo], Target.Command[AxisNo]);
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] %s StartMultiPosition STOP_NOW\n"), Target.Axis[AxisNo]);
						return Err;
					}
					break;
				}
			}
			
			Err = gcWmx3LinearIntplPos->StartLinearIntplPos();
		}
	}
	return Err;
}

long CPowerGantry::StartMultiPosition(long Target)
{
	long Err = NO_ERR, AxisIndex = 0;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	StartPosStruct* pStartPos;
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartMultiPosition Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		Err = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		Cwmx3Axis* pAxis;
		for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);
			pAxis = GetWmx3AxisByName(pStartPos->StrAxis);
			if (pAxis == NULL)
			{
				TRACE(_T("[PWR] StartMultiPosition(%s,%.1f) is NULL\n"), pStartPos->StrAxis, pStartPos->CommandPos);
				Err = GetAxisNullError(pStartPos->StrAxis);
			}
			else
			{
				if (pAxis->GetInitializeEnd() == false)
				{
					Err = HOMING_FAIL(pAxis->GetAxisIndex());
					TRACE(_T("[PWR] StartMultiPosition GetInitializeEnd Err:%d\n"), Err);
					CString strMsg;
					strMsg.Format(_T("%s is NOT ready to Move cause failed origin search\n"), (LPCTSTR)pStartPos->StrAxis);
					Err = SendAlarm(Err, strMsg);
					return Err;
				}
			}
		}
		Err = CheckLinearMotorTemperatureHigh(GetTable());
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPosition CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			//for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			//{
			//	pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);
			//	AxisIndex = GetAxisIndexFromAliasName(pStartPos->StrAxis);
			//	if (AxisIndex == static_cast<int>(PowerAxis::FX) || AxisIndex == static_cast<int>(PowerAxis::RX) || AxisIndex == static_cast<int>(PowerAxis::FY1) || AxisIndex == static_cast<int>(PowerAxis::RY1))
			//	{
			//		if (gcPowerLog->IsShowMotionLockLog() == true)
			//		{
			//			TRACE(_T("[PWR] StartMultiPosition AxisIndex(%d) Lock-1\n"), AxisIndex);
			//		}
			//		SEM_LOCK(gMOTION_LOCK[AxisIndex], INFINITE);
			//	}
			//}

			for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			{
				pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);

				if (IsMoveOnceAxis(pStartPos->StrAxis) == true)
				{
					Err = MoveOnceLockPauseMode(pStartPos->StrAxis, pStartPos->CommandPos);
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] %s StartMultiPosition STOP_NOW\n"), pStartPos->StrAxis);
						return Err;
					}
					break;
				}
			}
			Err = gcWmx3LinearIntplPos->StartLinearIntplPos();
		}
	}
	return Err;
}

long CPowerGantry::WaitMultiMotion(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiMotion Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneMotion(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f WaitOneMotion Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiMotion Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiMotion(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	bool bEventAvailable = false;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiMotion Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		ULONGLONG GetTime = _time_get(), ElapsedTime = 0;
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneMotion(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f WaitOneMotion Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiMotion %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		ElapsedTime = _time_elapsed(GetTime);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiMotion Elapsed:%d[ms] Err:%d\n"), ElapsedTime, ReturnErr);
		}
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiPosSet(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOnePosSet(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f WaitOnePosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiPosSet(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOnePosSet(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f WaitOnePosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiDelayedPosSet(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiDelayedPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneDelayedPosSet(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f WaitOneDelayedPosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiDelayedPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiDelayedPosSet(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiDelayedPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneDelayedPosSet(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f WaitOneDelayedPosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiDelayedPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiDelayedPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

void CPowerGantry::ReadMultiPosition()
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0;
	CApplicationTime* pTime = new CApplicationTime();
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		CmdPos = pStartPos->CommandPos;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->FeedbackPos = ReadPos;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] ReadMultiPosition %s Cmd:%.3f FeedBack:%.3f Diff:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos));
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadMultiPosition Elapsed:%d[ms]\n"), pTime->TimeElapsed());
	}
	delete pTime;
	pTime = NULL;
}

long CPowerGantry::StartMultiPositionR(SomeTarget Target)
{
	long Err = NO_ERR, AxisIndex = 0;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartMultiPositionR Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		Err = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPositionR CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			/*for (long AxisNo = 0; AxisNo < Target.MaxAxisCount; ++AxisNo)
			{
				AxisIndex = GetAxisIndexFromAliasName(Target.Axis[AxisNo]);
				if (gcPowerLog->IsShowMotionLockLog() == true)
				{
					TRACE(_T("[PWR] StartMultiPositionR AxisIndex(%d) Lock-1\n"), AxisIndex);
				}
				SEM_LOCK(gMOTION_LOCK[AxisIndex], INFINITE);
			}*/

			for (long AxisNo = 0; AxisNo < Target.MaxAxisCount; ++AxisNo)
			{
				if (IsMoveOnceAxis(Target.Axis[AxisNo]) == true)
				{
					Err = MoveOnceLockPauseMode(Target.Axis[AxisNo], Target.Command[AxisNo]);
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] %s StartMultiPosition STOP_NOW\n"), Target.Axis[AxisNo]);
						return Err;
					}
					break;
				}
			}

			Err = gcWmx3LinearIntplPos->StartLinearIntplPosR();
		}
	}
	return Err;
}

long CPowerGantry::StartMultiPositionR(long Target)
{
	long Err = NO_ERR, AxisIndex = 0;
	INT_PTR LinearIntpCount = 0;
	StartPosStruct* pStartPos;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartMultiPositionR Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		Err = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPositionR CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			/*for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			{
				pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);
				AxisIndex = GetAxisIndexFromAliasName(pStartPos->StrAxis);
				if (gcPowerLog->IsShowMotionLockLog() == true)
				{
					TRACE(_T("[PWR] StartMultiPositionR AxisIndex(%d) Lock-1\n"), AxisIndex);
				}
				SEM_LOCK(gMOTION_LOCK[AxisIndex], INFINITE);
			}*/

			for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			{
				pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);

				if (IsMoveOnceAxis(pStartPos->StrAxis) == true)
				{
					Err = MoveOnceLockPauseMode(pStartPos->StrAxis, pStartPos->CommandPos);
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] %s StartMultiPosition STOP_NOW\n"), pStartPos->StrAxis);
						return Err;
					}

					break;
				}
			}

			Err = gcWmx3LinearIntplPos->StartLinearIntplPosR();
		}
	}
	return Err;
}

long CPowerGantry::WaitMultiMotionR(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiMotion Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneMotion(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f WaitOneMotion Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiMotionR Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiMotionR(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiMotionR Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneMotion(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f WaitOneMotion Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiMotionR %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiMotionR Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiRPosSet(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOnePosSet(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f WaitOnePosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiRPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiRPosSet(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOnePosSet(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f WaitOnePosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiRPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiRPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiDelayedRPosSet(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiDelaWaitMultiDelayedRPosSetyedPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneDelayedPosSet(strAxis, CmdPos, Target.TimeOut[index]);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f WaitOneDelayedPosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiDelayedRPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

long CPowerGantry::WaitMultiDelayedRPosSet(long TimeOut)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0, Torque = 0.0, InPos = 0.1;
	long Err[MAXAXISNO], ReturnErr = NO_ERR;
	ZeroMemory(&Err, sizeof(Err));
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] WaitMultiDelayedRPosSet Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		ReturnErr = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		CApplicationTime* pTime = new CApplicationTime();
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			CmdPos = pStartPos->CommandPos;
			InPos = pStartPos->Inposition;
			ReturnErr = Err[index] = IsWaitOnePos(strAxis, CmdPos, InPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f IsWaitOnePos Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReturnErr = Err[index] = WaitOneDelayedPosSet(strAxis, CmdPos, TimeOut);
			if (ReturnErr != NO_ERR)
			{
				if (gcPowerLog->IsShowMotionLog() == true)
				{
					TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f WaitOneDelayedPosSet Err:%d\n"), strAxis, CmdPos, ReturnErr);
				}
				break;
			}
			ReadPos = ReadOnePosition(strAxis);
			Torque = ReadActualTorque(strAxis);
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] WaitMultiDelayedRPosSet %s Cmd:%.3f FeedBack:%.3f Diff:%.3f Torque:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos), Torque);
			}
		}
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] WaitMultiDelayedRPosSet Elapsed:%d[ms] Err:%d\n"), pTime->TimeElapsed(), ReturnErr);
		}
		delete pTime;
		pTime = NULL;
	}
	return ReturnErr;
}

void CPowerGantry::ReadMultiPositionR()
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos = 0.0, CmdPos = 0.0;
	CApplicationTime* pTime = new CApplicationTime();
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		CmdPos = pStartPos->CommandPos;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->FeedbackPos = ReadPos;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] ReadMultiPositionR %s Cmd:%.3f FeedBack:%.3f Diff:%.3f\n"), strAxis, CmdPos, ReadPos, (ReadPos - CmdPos));
		}
	}
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] ReadMultiPositionR Elapsed:%d[ms]\n"), pTime->TimeElapsed());
	}
	delete pTime;
	pTime = NULL;
}


long CPowerGantry::Disable1DCompensation()
{
	int err = ErrorCode::None;
	Cwmx3Axis* pAxisY2;
	pAxisY2 = GetWmx3AxisByName(_T("FY2"));
	if (pAxisY2 == NULL)
	{
		TRACE(_T("[PWR] Disable1DCompensationData pAxisY2 is NULL Err:%d\n"), err);
		return err;
	}
	else
	{
		err = pAxisY2->Disable1D();
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] Disable1DCompensationData Err:%d\n"), err);
		}
	}
	return err;
}

long CPowerGantry::Enable1DCompensation()
{
	int err = ErrorCode::None;
	Cwmx3Axis* pAxisY2;
	pAxisY2 = GetWmx3AxisByName(_T("FY2"));
	if (pAxisY2 == NULL)
	{
		TRACE(_T("[PWR] Enable1DCompensationData pAxisY2 is NULL Err:%d\n"), err);
		return err;
	}
	err = pAxisY2->Enable1D();
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Enable1DCompensationData Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::Set1DCompensation()
{
	PitchErrorCompensationData oneDCompData;
	int err = ErrorCode::None;
	double oneDComp = 0.0;
	Cwmx3Axis* pAxisY2;
	pAxisY2 = GetWmx3AxisByName(_T("FY2"));
	if (pAxisY2 == NULL)
	{
		TRACE(_T("[PWR] Set1DCompensation pAxisY2 is NULL Err:%d\n"), err);
		return err;
	}
	oneDCompData = PitchErrorCompensationData();
	oneDCompData.enable = 1;
	oneDCompData.pitchOriginPosition = 0.0;
	oneDCompData.pitchOriginIndex = 0; //Based on the current position value of axis, the variable should be different.
	oneDCompData.pitchCount = CAL_1D_MAXCOUNT;
	oneDCompData.pitchInterval = CAL_1D_PITCH_PULSE;
	oneDCompData.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;
	oneDCompData.options.catchUpVelocity = 1000.0;
	oneDCompData.options.catchUpAcc = 1000.0;

	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		oneDComp = gcPowerCalibrationData->Get1DCompensationData(FRONT_GANTRY, indx);
		if (GetGantryCalibrationMethod() == 0 || GetGantryCalibrationMethod() == 2)
		{
			oneDCompData.pitchCompensationValue[indx] = oneDComp * pAxisY2->GetUnResol();
		}
		else
		{
			oneDCompData.pitchCompensationValue[indx] = oneDComp * pAxisY2->GetUnResol() * (-1.0);
		}
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] pitchCompensationValue %s (%04d,%.3f)\n"), pAxisY2->GetAxisName(), indx, oneDCompData.pitchCompensationValue[indx]);
		}
	}
	err = pAxisY2->Set1DCompensationData(&oneDCompData);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set1DCompensationData Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::Disable2DCompensation(long Channel)
{
	long err = ErrorCode::None;
	err = Disable2D(Channel);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Disable2DCompensation Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::Enable2DCompensation(long Channel)
{
	long err = ErrorCode::None;
	err = Enable2D(Channel);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Enable2DCompensation Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::Set2DCompensation(long Gantry)
{
	long err = ErrorCode::None;
	Point_XY Start;
	Point_XYRE pt;
	long MaxPitch = 0, indx = 0;
	Cwmx3Axis* pAxisX;
	Cwmx3Axis* pAxisY1;
	Cwmx3Axis* pAxisY2;
	pAxisX = GetWmx3AxisByName(GetAxisX(Gantry));
	if (pAxisX == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisX is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisX(Gantry));
		return err;
	}
	pAxisY1 = GetWmx3AxisByName(GetAxisY1(Gantry));
	if (pAxisY1 == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisY1 is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisY1(Gantry));
		return err;
	}
	pAxisY2 = GetWmx3AxisByName(GetAxisY2(Gantry));
	if (pAxisY2 == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisY2 is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisY2(Gantry));
		return err;
	}
	long MarkXCount, MarkYCount;
	double catchUpAcc = 50000.0;
	double catchUpVelocity = 50000.0;

	MarkXCount = (long)(CAL_2D_MAX_XPOS / CAL_2D_PITCH) + 1;
	MarkYCount = (long)(CAL_2D_MAX_YPOS / CAL_2D_PITCH) + 1;
	Start = gcPowerCalibrationData->Get2DStartPosition(FRONT_GANTRY);
	TRACE(_T("[PWR] Set2DCompensation(%04d) XY(%d,%d) StartXY:%.3f %.3f\n"), MarkXCount * MarkYCount, MarkXCount, MarkYCount, Start.x, Start.y);
	g_2DCompDataX.enable = 0;
	g_2DCompDataX.axis = GetAxisID(GetAxisX(Gantry));
	g_2DCompDataX.pitchCount[0] = MarkXCount;
	g_2DCompDataX.pitchCount[1] = MarkYCount;
	g_2DCompDataX.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataX.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataX.pitchOriginIndex[0] = 0;
	g_2DCompDataX.pitchOriginIndex[1] = 0;
	g_2DCompDataX.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataX.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataX.referenceAxis[0] = GetAxisID(GetAxisX(Gantry));
	g_2DCompDataX.referenceAxis[1] = GetAxisID(GetAxisY1(Gantry));
	g_2DCompDataX.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;
	g_2DCompDataX.options.catchUpAcc = catchUpAcc;
	g_2DCompDataX.options.catchUpVelocity = catchUpVelocity;

	g_2DCompDataY1.enable = 0;
	g_2DCompDataY1.axis = GetAxisID(GetAxisY1(Gantry));
	g_2DCompDataY1.pitchCount[0] = MarkXCount;
	g_2DCompDataY1.pitchCount[1] = MarkYCount;
	g_2DCompDataY1.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataY1.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataY1.pitchOriginIndex[0] = 0;
	g_2DCompDataY1.pitchOriginIndex[1] = 0;
	g_2DCompDataY1.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataY1.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataY1.referenceAxis[0] = GetAxisID(GetAxisX(Gantry));
	g_2DCompDataY1.referenceAxis[1] = GetAxisID(GetAxisY1(Gantry));
	g_2DCompDataY1.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;
	g_2DCompDataY1.options.catchUpAcc = catchUpAcc;
	g_2DCompDataY1.options.catchUpVelocity = catchUpVelocity;

	g_2DCompDataY2.enable = 0;
	g_2DCompDataY2.axis = GetAxisID(GetAxisY2(Gantry));
	g_2DCompDataY2.pitchCount[0] = MarkXCount;
	g_2DCompDataY2.pitchCount[1] = MarkYCount;
	g_2DCompDataY2.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataY2.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataY2.pitchOriginIndex[0] = 0;
	g_2DCompDataY2.pitchOriginIndex[1] = 0;
	g_2DCompDataY2.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataY2.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataY2.referenceAxis[0] = GetAxisID(GetAxisX(Gantry));
	g_2DCompDataY2.referenceAxis[1] = GetAxisID(GetAxisY1(Gantry));
	g_2DCompDataY2.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;
	g_2DCompDataY2.options.catchUpAcc = catchUpAcc;
	g_2DCompDataY2.options.catchUpVelocity = catchUpVelocity;

	for (long y = 0; y < MarkYCount; ++y)
	{
		for (long x = 0; x < MarkXCount; ++x)
		{
			pt = gcPowerCalibrationData->Get2DCompensationData(FRONT_GANTRY, indx);
			if (GetGantryCalibrationMethod() == 0 || GetGantryCalibrationMethod() == 1)
			{
				if (GetGantry2DMethod() == 0)
				{
					g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol();
					g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
					g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
				}
				else if (GetGantry2DMethod() == 1)
				{
					g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol() * (-1.0);
					g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
					g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
				}
				else if (GetGantry2DMethod() == 2)
				{
					g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol();
					g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);;
					g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);;
				}
				else if (GetGantry2DMethod() == 3)
				{
					g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol() * (-1.0);;
					g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);;
					g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);;
				}
			}
			else
			{
				g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol() * (-1.0);
				g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);
				g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);
			}
			if (gcPowerLog->IsShowCalibrationLog() == true)
			{
				TRACE(_T("[PWR] 2D(%04d) X:%04d Y:%04d %.3f %.3f %.3f\n"), indx, x, y, 
					g_2DCompDataX.pitchCompensationValue[x][y], g_2DCompDataY1.pitchCompensationValue[x][y], g_2DCompDataY2.pitchCompensationValue[x][y]);
			}
			indx++;
		}
	}
	err = Set2DCompensationData(CHANNEL_1, &g_2DCompDataX);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_1 Err:%d\n"), err);
	}
	err = Set2DCompensationData(CHANNEL_2, &g_2DCompDataY1);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_2 Err:%d\n"), err);
	}
	err = Set2DCompensationData(CHANNEL_3, &g_2DCompDataY2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_3 Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::Set2DCompensation(long Gantry, long MarkXCount, long MarkYCount)
{
	long err = ErrorCode::None;
	Point_XY Start;
	Point_XYRE pt;
	long MaxPitch = 0, indx = 0;
	Cwmx3Axis* pAxisX;
	Cwmx3Axis* pAxisY1;
	Cwmx3Axis* pAxisY2;
	pAxisX = GetWmx3AxisByName(GetAxisX(Gantry));
	if (pAxisX == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisX is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisX(Gantry));
		return err;
	}
	pAxisY1 = GetWmx3AxisByName(GetAxisY1(Gantry));
	if (pAxisY1 == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisY1 is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisY1(Gantry));
		return err;
	}
	pAxisY2 = GetWmx3AxisByName(GetAxisY2(Gantry));
	if (pAxisY2 == NULL)
	{
		TRACE(_T("[PWR] Set2DCompensation pAxisY2 is NULL Err:%d\n"), err);
		err = GetAxisNullError(GetAxisY2(Gantry));
		return err;
	}
	long lMarkXCount, lMarkYCount;
	lMarkXCount = MarkXCount + 1;
	lMarkYCount = MarkYCount + 1;
	Start = gcPowerCalibrationData->Get2DStartPosition(FRONT_GANTRY);
	TRACE(_T("[PWR] Set2DCompensation(%04d) XY(%d,%d) StartXY:%.3f %.3f\n"), MarkXCount * MarkYCount, MarkXCount, MarkYCount, Start.x, Start.y);
	g_2DCompDataX.enable = 0;
	g_2DCompDataX.axis = GetAxisID(_T("FX"));
	g_2DCompDataX.pitchCount[0] = lMarkXCount;
	g_2DCompDataX.pitchCount[1] = lMarkYCount;
	g_2DCompDataX.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataX.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataX.pitchOriginIndex[0] = 0;
	g_2DCompDataX.pitchOriginIndex[1] = 0;
	g_2DCompDataX.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataX.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataX.referenceAxis[0] = GetAxisID(_T("FX"));
	g_2DCompDataX.referenceAxis[1] = GetAxisID(_T("FY1"));
	g_2DCompDataX.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;

	g_2DCompDataY1.enable = 0;
	g_2DCompDataY1.axis = GetAxisID(_T("FY1"));
	g_2DCompDataY1.pitchCount[0] = lMarkXCount;
	g_2DCompDataY1.pitchCount[1] = lMarkYCount;
	g_2DCompDataY1.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataY1.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataY1.pitchOriginIndex[0] = 0;
	g_2DCompDataY1.pitchOriginIndex[1] = 0;
	g_2DCompDataY1.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataY1.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataY1.referenceAxis[0] = GetAxisID(_T("FX"));
	g_2DCompDataY1.referenceAxis[1] = GetAxisID(_T("FY1"));
	g_2DCompDataY1.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;

	g_2DCompDataY2.enable = 0;
	g_2DCompDataY2.axis = GetAxisID(_T("FY2"));
	g_2DCompDataY2.pitchCount[0] = lMarkXCount;
	g_2DCompDataY2.pitchCount[1] = lMarkYCount;
	g_2DCompDataY2.pitchInterval[0] = CAL_2D_PITCH * pAxisX->GetUnResol();
	g_2DCompDataY2.pitchInterval[1] = CAL_2D_PITCH * pAxisY1->GetUnResol();
	g_2DCompDataY2.pitchOriginIndex[0] = 0;
	g_2DCompDataY2.pitchOriginIndex[1] = 0;
	g_2DCompDataY2.pitchOriginPosition[0] = Start.x * pAxisX->GetUnResol();
	g_2DCompDataY2.pitchOriginPosition[1] = Start.y * pAxisY1->GetUnResol();
	g_2DCompDataY2.referenceAxis[0] = GetAxisID(_T("FX"));
	g_2DCompDataY2.referenceAxis[1] = GetAxisID(_T("FY1"));
	g_2DCompDataY2.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;

	for (long y = 0; y < lMarkYCount; ++y)
	{
		for (long x = 0; x < lMarkXCount; ++x)
		{
			pt = gcPowerCalibrationData->Get2DCompensationData(FRONT_GANTRY, indx);
			if (GetGantryCalibrationMethod() == 0 || GetGantryCalibrationMethod() == 1)
			{
				g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol();
				g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
				g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol();
			}
			else
			{
				g_2DCompDataX.pitchCompensationValue[x][y] = pt.x * pAxisX->GetUnResol() * (-1.0);
				g_2DCompDataY1.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);
				g_2DCompDataY2.pitchCompensationValue[x][y] = pt.y * pAxisY1->GetUnResol() * (-1.0);
			}
			if (gcPowerLog->IsShowCalibrationLog() == true)
			{
				TRACE(_T("[PWR] Set2DCompensation(%04d) X:%04d Y:%04d %.3f %.3f\n"), indx, x, y, g_2DCompDataX.pitchCompensationValue[x][y], g_2DCompDataY1.pitchCompensationValue[x][y]);
			}
			indx++;
		}
	}
	err = Set2DCompensationData(CHANNEL_1, &g_2DCompDataX);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_1 Err:%d\n"), err);
	}
	err = Set2DCompensationData(CHANNEL_2, &g_2DCompDataY1);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_2 Err:%d\n"), err);
	}
	err = Set2DCompensationData(CHANNEL_3, &g_2DCompDataY2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] Set2DCompensationData CHANNEL_3 Err:%d\n"), err);
	}
	return err;
}
long CPowerGantry::PauseGantry()
{
	//StepMoveLock();
	//g_PauseMoveOneTime = false;
	return 0;
}

long CPowerGantry::ResumeGantry()
{
	MoveOneTimeUnlock();
	//g_PauseMoveOneTime = true;
	return 0;
}

long CPowerGantry::ReleaseGantry()
{
	MoveOneTimeUnlock();
	//g_PauseMoveOneTime = true;
	return 0;
}
//long CPowerGantry::PauseGantry()
//{
//	Cwmx3Axis* pAxis;
//	for (int indx = 0; indx < MAXGANTRYAXISNO; ++indx)
//	{
//		pAxis = GetWmx3AxisByName(PowerAxisAliasName[indx]);
//		if (pAxis == NULL) continue;
//		if (pAxis->IsSlaveAxis() == true) continue;
//		if (pAxis->IsZAxis() == true) continue;
//		if (pAxis->IsRAxis() == true) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepMotor) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepServoMotor) continue;
//		TRACE(_T("[PWR] (%s,%d) Start to Lock()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//		SEM_LOCK(gMOTION_LOCK[indx], INFINITE);
//		//pAxis->Lock();
//		m_bPause[pAxis->GetAxisIndex()] = true;
//		TRACE(_T("[PWR] (%s,%d) Complete to Lock()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//	}
//	return 0;
//}
//
//long CPowerGantry::ResumeGantry()
//{
//	Cwmx3Axis* pAxis;
//	for (int indx = 0; indx < MAXGANTRYAXISNO; ++indx)
//	{
//		pAxis = GetWmx3AxisByName(PowerAxisAliasName[indx]);
//		if (pAxis == NULL) continue;
//		if (pAxis->IsSlaveAxis() == true) continue;
//		if (pAxis->IsZAxis() == true) continue;
//		if (pAxis->IsRAxis() == true) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepMotor) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepServoMotor) continue;
//		if (m_bPause[pAxis->GetAxisIndex()] == true)
//		{
//			TRACE(_T("[PWR] (%s,%d) Start to UnLock()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//			SEM_UNLOCK(gMOTION_LOCK[indx]);
//			//pAxis->Unlock();
//			m_bPause[pAxis->GetAxisIndex()] = false;
//			TRACE(_T("[PWR] (%s,%d) Complete to Unlock()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//		}
//		else
//		{
//			TRACE(_T("[PWR] (%s,%d) Skip to UnLock()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//		}
//	}
//	return 0;
//}
//
//long CPowerGantry::ReleaseGantry()
//{
//	Cwmx3Axis* pAxis;
//	for (int indx = 0; indx < MAXGANTRYAXISNO; ++indx)
//	{
//		pAxis = GetWmx3AxisByName(PowerAxisAliasName[indx]);
//		if (pAxis == NULL) continue;
//		if (pAxis->IsSlaveAxis() == true) continue;
//		if (pAxis->IsZAxis() == true) continue;
//		if (pAxis->IsRAxis() == true) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepMotor) continue;
//		if (pAxis->GetMotorType() == CMotorType::StepServoMotor) continue;
//		if (m_bPause[pAxis->GetAxisIndex()] == true)
//		{
//			TRACE(_T("[PWR] (%s,%d) Start to Release()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//			SEM_UNLOCK(gMOTION_LOCK[indx]);
//			//SEM_FLUSH(gMOTION_LOCK[indx]);
//			//pAxis->Unlock();
//			m_bPause[pAxis->GetAxisIndex()] = false;
//			TRACE(_T("[PWR] (%s,%d) Complete to Release()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//		}
//		else
//		{
//			TRACE(_T("[PWR] (%s,%d) Skip to Release()\n"), pAxis->GetAxisName(), pAxis->GetAxisIndex());
//		}
//	}
//	return 0;
//}



INT_PTR CPowerGantry::AddLinearIntpAxis(long Gantry)
{
	StartPosStruct* pStartPosX = new StartPosStruct;
	StartPosStruct* pStartPosY = new StartPosStruct;
	pStartPosX->StrAxis = GetAxisX(Gantry);
	pStartPosX->CommandPos = 0.0;
	pStartPosX->Distance = 0.0;
	pStartPosY->StrAxis = GetAxisY1(Gantry);
	pStartPosY->CommandPos = 0.0;
	pStartPosY->Distance = 0.0;
	AddWmx3LinearIntpAxis(pStartPosX);
	AddWmx3LinearIntpAxis(pStartPosY);
	return GetWmx3LinearIntpAxisCount();
}

INT_PTR CPowerGantry::AddSomeRIntpAxis(SomeTarget Target)
{
	StartPosStruct* pStartPosR[MAXUSEDHEADNO];
	CString StrAxis;
	Cwmx3Axis* pAxis;

	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] AddSomeRIntpAxis Count:%d\n"), Target.MaxAxisCount);
	}
	for (INT_PTR indx = 0; indx < Target.MaxAxisCount; ++indx)
	{
		pStartPosR[indx] = new StartPosStruct;
		pStartPosR[indx]->CommandPos = 0.0;
		pStartPosR[indx]->Distance = 0.0;
		StrAxis = Target.Axis[indx];
		pStartPosR[indx]->StrAxis = StrAxis;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] AddSomeRIntpAxis indx:%d(%s)\n"), indx, StrAxis);
		}

		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetAxisSkip() == true)
		{
			TRACE(_T("[PWR] AddSomeRIntpAxis indx:%d(%s) Skip\n"), indx, StrAxis);
			continue;
		}

		AddWmx3LinearIntpAxisR(pStartPosR[indx]);
	}
	return GetWmx3LinearIntpAxisRCount();
}


INT_PTR CPowerGantry::AddAllRIntpAxis()
{
	StartPosStruct* pStartPosR[MAXUSEDHEADNO];
	CString StrAxis;
	Cwmx3Axis* pAxis;

	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] AddAllRIntpAxis Count:%d\n"), GetRAxisCount());
	}
	for (INT_PTR indx = 0; indx < GetRAxisCount(); ++indx)
	{
		pStartPosR[indx] = new StartPosStruct;
		pStartPosR[indx]->CommandPos = 0.0;
		pStartPosR[indx]->Distance = 0.0;
		StrAxis = GetRAxisByIndex(indx);
		pStartPosR[indx]->StrAxis = StrAxis;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] AddAllRIntpAxis indx:%d(%s)\n"), indx, StrAxis);
		}

		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetAxisSkip() == true)
		{
			TRACE(_T("[PWR] AddAllRIntpAxis indx:%d(%s) Skip\n"), indx, StrAxis);
			continue;
		}

		AddWmx3LinearIntpAxisR(pStartPosR[indx]);
	}
	return GetWmx3LinearIntpAxisRCount();
}

INT_PTR CPowerGantry::AddSomeZIntpAxis(SomeTarget Target)
{
	StartPosStruct* pStartPosZ[MAXUSEDHEADNO];
	CString StrAxis;
	Cwmx3Axis* pAxis;

	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] AddSomeZIntpAxis Count:%d\n"), Target.MaxAxisCount);
	}
	for (INT_PTR indx = 0; indx < Target.MaxAxisCount; ++indx)
	{
		pStartPosZ[indx] = new StartPosStruct;
		pStartPosZ[indx]->CommandPos = 0.0;
		pStartPosZ[indx]->Distance = 0.0;
		StrAxis = Target.Axis[indx];
		pStartPosZ[indx]->StrAxis = StrAxis;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] AddSomeZIntpAxis indx:%d(%s)\n"), indx, StrAxis);
		}

		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetAxisSkip() == true)
		{
			TRACE(_T("[PWR] AddSomeZIntpAxis indx:%d(%s) Skip\n"), indx, StrAxis);
			continue;
		}

		AddWmx3LinearIntpAxis(pStartPosZ[indx]);
	}
	return GetWmx3LinearIntpAxisCount();
}

INT_PTR CPowerGantry::AddAllZIntpAxis()
{
	StartPosStruct* pStartPosZ[MAXUSEDHEADNO];
	CString StrAxis;
	Cwmx3Axis* pAxis;

	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] AddAllZIntpAxis Count:%d\n"), GetZAxisCount());
	}
	for (INT_PTR indx = 0; indx < GetZAxisCount(); ++indx)
	{
		pStartPosZ[indx] = new StartPosStruct;
		pStartPosZ[indx]->CommandPos = 0.0;
		pStartPosZ[indx]->Distance = 0.0;
		StrAxis = GetZAxisByIndex(indx);
		pStartPosZ[indx]->StrAxis = StrAxis;
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] AddAllZIntpAxis indx:%d(%s)\n"), indx, StrAxis);
		}

		pAxis = GetWmx3AxisByName(StrAxis);
		if (pAxis->GetAxisSkip() == true)
		{
			TRACE(_T("[PWR] AddAllZIntpAxis indx:%d(%s) Skip\n"), indx, StrAxis);
			continue;
		}

		AddWmx3LinearIntpAxis(pStartPosZ[indx]);
	}
	return GetWmx3LinearIntpAxisCount();
}

void CPowerGantry::InitMultiRatio()
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] InitMultiRatio Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		InitOneRatio(strAxis);
	}
}

void CPowerGantry::InitMultiRatioR()
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] InitMultiRatioR Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		InitOneRatio(strAxis);
	}
}

void CPowerGantry::SetMultiRatio(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiRatio Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Ratio = Target.Ratio[index];
		SetOneRatio(strAxis, pStartPos->Ratio);
	}
}

void CPowerGantry::SetMultiRatio(double Ratio)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiRatio Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Ratio = Ratio;
		SetOneRatio(strAxis, Ratio);
	}
}

long CPowerGantry::SetMultiPosSet(SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Target.Inpos[index];
		Err = SetOnePosSet(strAxis, pStartPos->Inposition);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiPosSet SomeTarget Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}


long CPowerGantry::SetMultiPosSet(double Inpos)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Inpos;
		Err = SetOnePosSet(strAxis, Inpos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiPosSet All Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiDelayedPosSet(SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Target.Inpos[index];
		pStartPos->InpositionTime = Target.InposMs[index];
		Err = SetOneDelayedPosSet(strAxis, pStartPos->Inposition, pStartPos->InpositionTime);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiDelayedPosSet SomeTarget Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiDelayedPosSet(double Inpos, long Ms)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Inpos;
		pStartPos->InpositionTime = Ms;
		Err = SetOneDelayedPosSet(strAxis, Inpos, Ms);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiDelayedPosSet All Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

void CPowerGantry::SetMultiRatioR(SomeTarget Target)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiRatio Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Ratio = Target.Ratio[index];
		SetOneRatio(strAxis, pStartPos->Ratio);
	}
}

void CPowerGantry::SetMultiRatioR(double Ratio)
{
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiRatio Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Ratio = Ratio;
		SetOneRatio(strAxis, Ratio);
	}
}

long CPowerGantry::SetMultiPosSetR(SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Target.Inpos[index];
		Err = SetOnePosSet(strAxis, pStartPos->Inposition);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiPosSet SomeTarget Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}


long CPowerGantry::SetMultiPosSetR(double Inpos)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Inpos;
		Err = SetOnePosSet(strAxis, Inpos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiPosSet All Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiDelayedPosSetR(SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Target.Inpos[index];
		pStartPos->InpositionTime = Target.InposMs[index];
		Err = SetOneDelayedPosSet(strAxis, pStartPos->Inposition, pStartPos->InpositionTime);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiDelayedPosSet SomeTarget Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiDelayedPosSetR(double Inpos, long Ms)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	CString strAxis;
	StartPosStruct* pStartPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pStartPos->Inposition = Inpos;
		pStartPos->InpositionTime = Ms;
		Err = SetOneDelayedPosSet(strAxis, Inpos, Ms);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s SetMultiDelayedPosSet All Err(%d)\n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiCommand(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommand Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->CommandPos = Target.Command[index];
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
		}
		pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
		pStartPos->linearIntplProfileCalcMode = 0;
	}
	return Err;
}

long CPowerGantry::SetMultiCommand(long Gantry, long Target, double x, double y)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommand Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] SetMultiCommand X Y %.3f %.3f Target:%d\n"), x, y, Target);
	}
	if (Target >= TBL_HEAD1 && Target <= TBL_HEAD10)
	{
		Point_XY offset = GetHeadOffset(Target);
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] SetMultiCommand X Y %.3f %.3f Target:%d OffsetXY,%.3f,%.3f\n"), x, y, Target, offset.x, offset.y);
		}
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.x;
			}
			else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.y;
			}
		}
	}
	else if (Target == TBL_HEIGHTDEVICE)
	{
		Point_XY offset = GetHMOffset();
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] SetMultiCommand X Y %.3f %.3f HeightDevice:%d OffsetXY,%.3f,%.3f\n"), x, y, Target, offset.x, offset.y);
		}
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.x;
			}
			else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.y;
			}
		}
	}
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
		{
			pStartPos->CommandPos += x;
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
			}
			pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		}
		else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
		{
			pStartPos->CommandPos += y;
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
			}
			pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		}
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
		pStartPos->linearIntplProfileCalcMode = 0;
	}
	return Err;
}

long CPowerGantry::SetMultiCommand(long Gantry, double pos)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommand Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->CommandPos = pos;
		pStartPos->Distance = abs(pos - ReadPos);
		pStartPos->linearIntplProfileCalcMode = 0;
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetMultiCommandR(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommand Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->CommandPos = Target.Command[index];
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
		}
		pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
		pStartPos->linearIntplProfileCalcMode = 0;
	}
	return Err;
}

long CPowerGantry::SetMultiCommandR(long Gantry, long Target, double x, double y)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommandR Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] SetMultiCommandR X Y %.3f %.3f\n"), x, y);
	}
	if (Target >= TBL_HEAD1 && Target <= TBL_HEAD10)
	{
		Point_XY offset = GetHeadOffset(Target);
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
			strAxis = pStartPos->StrAxis;
			if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.x;
			}
			else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.y;
			}
		}
	}
	else if (Target == TBL_HEIGHTDEVICE)
	{
		Point_XY offset = GetHMOffset();
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] SetMultiCommandR X Y %.3f %.3f HeightDevice:%d OffsetXY,%.3f,%.3f\n"), x, y, Target, offset.x, offset.y);
		}
		for (INT_PTR index = 0; index < LinearIntpCount; ++index)
		{
			pStartPos = GetWmx3LinearIntpAxisByIndex(index);
			strAxis = pStartPos->StrAxis;
			if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.x;
			}
			else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
			{
				pStartPos->CommandPos = offset.y;
			}
		}
	}
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		if (strAxis.CompareNoCase(GetAxisX(Gantry)) == 0)
		{
			pStartPos->CommandPos += x;
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
			}
			pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		}
		else if (strAxis.CompareNoCase(GetAxisY1(Gantry)) == 0)
		{
			pStartPos->CommandPos += y;
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] %s Command:%.3f\n"), strAxis, pStartPos->CommandPos);
			}
			pStartPos->Distance = abs(pStartPos->CommandPos - ReadPos);
		}
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
		pStartPos->linearIntplProfileCalcMode = 0;
	}
	return Err;
}

long CPowerGantry::SetMultiCommandR(long Gantry, double pos)
{
	long Err = NO_ERR;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] SetMultiCommand Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	CString strAxis;
	StartPosStruct* pStartPos;
	double ReadPos;
	for (INT_PTR index = 0; index < LinearIntpCount; ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		ReadPos = ReadOnePosition(strAxis);
		pStartPos->CommandPos = pos;
		pStartPos->Distance = abs(pos - ReadPos);
		pStartPos->linearIntplProfileCalcMode = 0;
		Err = CheckLimitOver(strAxis, pStartPos->CommandPos);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] %s Command:%.3f Limit Error(%d) **************** \n"), strAxis, Err);
			break;
		}
	}
	return Err;
}

void CPowerGantry::RemoveLinearIntpAxis()
{
	INT_PTR MaxLinearIntpAxis = GetWmx3LinearIntpAxisCount();
	while (GetWmx3LinearIntpAxisCount() > 0)
	{
		RemoveWmx3LinearIntpAxis(0);
		ThreadSleep(WAIT_REMOVE_ARRAY_WAITTIME);
	}
	ASSERT(GetWmx3LinearIntpAxisCount() == 0);
}

void CPowerGantry::RemoveLinearIntpAxisR()
{
	INT_PTR MaxLinearIntpAxis = GetWmx3LinearIntpAxisRCount();
	while (GetWmx3LinearIntpAxisRCount() > 0)
	{
		RemoveWmx3LinearIntpAxisR(0);
		ThreadSleep(WAIT_REMOVE_ARRAY_WAITTIME);
	}
	ASSERT(GetWmx3LinearIntpAxisRCount() == 0);
}

void CPowerGantry::WriteHomePosition(CString strAxis, double HomePosition)
{
	Cwmx3Axis* pAxis;
	THREAD_STRUCT threadInfo;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis != NULL)
	{
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteHomePosition (%s,%d) HomePosition:%.3f\n"), strAxis, pAxis->GetAxisIndex(), HomePosition);
		}
		gcPowerCalibrationData->SetHomePosition(pAxis->GetAxisIndex(), HomePosition);
		gcPowerCalibrationData->WriteHomePosition(FRONT_GANTRY);
	}
}

double CPowerGantry::ReadHomePosition(CString strAxis)
{
	Cwmx3Axis* pAxis;
	THREAD_STRUCT threadInfo;
	double HomePosition = 0.0;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis != NULL)
	{
		HomePosition = gcPowerCalibrationData->GetHomePosition(pAxis->GetAxisIndex());
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadHomePosition %s HomePosition:%.3f\n"), strAxis, HomePosition);
		}
	}
	return HomePosition;
}

void CPowerGantry::WriteCameraAlignPosition(long Gantry, Point_XY CameraAlign)
{
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteCameraAlignPosition Front SetCameraAlignPosition:%.3f %.3f\n"), CameraAlign.x, CameraAlign.y);
	}
	gcPowerCalibrationData->SetCameraAlignPosition(Gantry, CameraAlign);
	gcPowerCalibrationData->WriteCameraAlignPosition(Gantry);
}

Point_XY CPowerGantry::ReadCameraAlignPosition(long Gantry)
{
	Point_XY CameraAlign;
	CameraAlign = gcPowerCalibrationData->GetCameraAlignPosition(Gantry);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadCameraAlignPosition Front GetCameraAlignPosition:%.3f %.3f\n"), CameraAlign.x, CameraAlign.y);
	}
	return CameraAlign;
}

void CPowerGantry::WritePcbFixPosition(long Conveyor, Point_XY PcbFix)
{
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WritePcbFixPosition Conveyor(%d) SetPcbFixPosition:%.3f %.3f\n"), Conveyor, PcbFix.x, PcbFix.y);
	}
	gcPowerCalibrationData->SetPcbFixPosition(Conveyor, PcbFix);
	gcPowerCalibrationData->WritePcbFixPosition(Conveyor);
}

Point_XY CPowerGantry::ReadPcbFixPosition(long Conveyor)
{
	Point_XY PcbFix;
	PcbFix = gcPowerCalibrationData->GetPcbFixPosition(Conveyor);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadPcbFixPosition Conveyor(%d) GetPcbFixPosition:%.3f %.3f\n"), Conveyor, PcbFix.x, PcbFix.y);
	}
	return PcbFix;
}

void CPowerGantry::WriteReferenceFeederPosition(long Stage, long RefFdNo, Point_XY RefFdPos)
{
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteReferenceFeederPosition Stage(%d) SetReferenceFeederNo(%03d):%.3f %.3f\n"), Stage, RefFdNo, RefFdPos.x, RefFdPos.y);
	}
	gcPowerCalibrationData->SetReferenceFeederNo(Stage, RefFdNo);
	gcPowerCalibrationData->SetReferenceFeederPosition(Stage, RefFdPos);
	gcPowerCalibrationData->WriteReferenceFeeder(Stage);
}

void CPowerGantry::WriteFeederPitch(long Stage, double Pitch)
{
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteFeederPitch Stage(%d) SetFeederPitch:%.3f\n"), Stage, Pitch);
	}
	gcPowerCalibrationData->SetFeederPitch(Stage, Pitch);
	gcPowerCalibrationData->WriteReferenceFeeder(Stage);
}

Point_XY CPowerGantry::ReadReferenceFeederPosition(long Stage)
{
	Point_XY RefFdPos;
	RefFdPos = gcPowerCalibrationData->GetReferenceFeederPosition(Stage);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadFeederReferencePosition Stage(%d) GetReferenceFeederPosition:%.3f %.3f\n"), Stage, RefFdPos.x, RefFdPos.y);
	}
	return RefFdPos;
}

long CPowerGantry::ReadReferenceFeederNo(long Stage)
{
	long RefFdNo = 0;
	RefFdNo = gcPowerCalibrationData->GetReferenceFeederNo(Stage);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadReferenceFeederNo Stage(%d) GetReferenceFeederNo:%d\n"), Stage, RefFdNo);
	}
	return RefFdNo;
}

double CPowerGantry::ReadFeederPitch(long Stage)
{
	double Pitch = 0.0;
	Pitch = gcPowerCalibrationData->GetFeederPitch(Stage);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadFeederPitch Stage(%d) GetFeederPitch:%.3f\n"), Stage, Pitch);
	}
	return Pitch;
}

Point_XY CPowerGantry::ReadOriginDist(Point_XY Origin)
{
	Point_XY Result, PcbFix;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	Result.x = Origin.x - PcbFix.x;
	Result.y = Origin.y - PcbFix.y;
	return Result;
}

Point_XY CPowerGantry::ReadMarkDist(Point_XY MarkPt, Point_XY Origin)
{
	Point_XY Result, PcbFix;
	//Point_XYRZE Origin;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	//Origin = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_POINT_NO);
	Result.x = MarkPt.x - (PcbFix.x + Origin.x);
	Result.y = MarkPt.y - (PcbFix.y + Origin.y);
	return Result;
}

Point_XY CPowerGantry::ReadInsertDist(Point_XY InsertPt, Point_XY Origin)
{
	Point_XY Result, PcbFix;
	//Point_XYRZE Origin;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	//Origin = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_POINT_NO);
	Result.x = InsertPt.x - (PcbFix.x + Origin.x);
	Result.y = InsertPt.y - (PcbFix.y + Origin.y);
	return Result;
}

Point_XY CPowerGantry::ReadOriginTarget(Point_XY Origin)
{
	Point_XY Result, PcbFix;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	Result.x = PcbFix.x + Origin.x;
	Result.y = PcbFix.y + Origin.y;
	return Result;
}

Point_XY CPowerGantry::ReadMarkTarget(Point_XY MarkPt, Point_XY Origin, const bool& isSkipReadBlockTarget)
{
	Point_XY Result, PcbFix;
	//Point_XYRZE Origin;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	//Origin = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_POINT_NO);
	Result.x = PcbFix.x + Origin.x + MarkPt.x;
	Result.y = PcbFix.y + Origin.y + MarkPt.y;

    const long useBlockType = gcReadJobFile->GetPcb().UseBlockType;
    if (useBlockType == PCB_MAXTRIX || useBlockType == PCB_NON_MAXTRIX)
    {
        Result = (isSkipReadBlockTarget == true) ? Result : ReadBlockTarget(MarkPt, Origin);
    }

	return Result;
}

Point_XY CPowerGantry::ReadInsertTarget(Point_XY InsertPt, Point_XY Origin)
{
	Point_XY Result, PcbFix;
	//Point_XYRZE Origin;
	long Conveyor = FRONT_CONV;
	PcbFix = ReadPcbFixPosition(Conveyor);
	//Origin = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_POINT_NO);
	Result.x = PcbFix.x + Origin.x + InsertPt.x;
	Result.y = PcbFix.y + Origin.y + InsertPt.y;

    const long useBlockType = gcReadJobFile->GetPcb().UseBlockType;
    if (useBlockType == PCB_MAXTRIX || useBlockType == PCB_NON_MAXTRIX)
    {
        Result = ReadBlockTarget(InsertPt, Origin);
    }

	return Result;
}

void CPowerGantry::WriteConfirmInsertBeforePosition(Point_XY Before)
{
	m_ConfirmInsertBeforePosition.x = Before.x;
	m_ConfirmInsertBeforePosition.y = Before.y;
	TRACE(_T("[PWR] WriteConfirmInsertBeforePosition X,Y,%.3f,%.3f"), m_ConfirmInsertBeforePosition.x, m_ConfirmInsertBeforePosition.y);
}

Point_XY CPowerGantry::ReadConfirmInsertBeforePosition()
{
	Point_XY Result;
	Result = m_ConfirmInsertBeforePosition;
	TRACE(_T("[PWR] ReadConfirmInsertBeforePosition X,Y,%.3f,%.3f"), Result.x, Result.y);
	return Result;
}

void CPowerGantry::WriteConfirmMeasureHeightBeforePosition(Point_XYT Before)
{
	m_ConfirmMeasureHeightBeforePosition.x = Before.x;
	m_ConfirmMeasureHeightBeforePosition.y = Before.y;
	m_ConfirmMeasureHeightBeforePosition.t = Before.t;
	TRACE(_T("[PWR] WriteConfirmMeasureHeightBeforePosition X,Y,T,%.3f,%.3f,%.3f"), m_ConfirmMeasureHeightBeforePosition.x, m_ConfirmMeasureHeightBeforePosition.y, m_ConfirmMeasureHeightBeforePosition.t);
}

Point_XYT CPowerGantry::ReadConfirmMeasureHeightBeforePosition()
{
	Point_XYT Result;
	Result = m_ConfirmMeasureHeightBeforePosition;
	TRACE(_T("[PWR] ReadConfirmMeasureHeightBeforePosition X,Y,T,%.3f,%.3f,%.3f"), Result.x, Result.y, Result.t);
	return Result;
}


long CPowerGantry::DisableZCompensation(CString strAxisZ)
{
	int err = ErrorCode::None;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxisZ);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] DisableZCompensation pAxis is NULL Err:%d\n"), err);
		return err;
	}
	err = pAxis->Disable1D();
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] DisableZCompensation Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::EnableZCompensation(CString strAxisZ)
{
	int err = ErrorCode::None;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxisZ);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] EnableZCompensation pAxis is NULL Err:%d\n"), err);
		return err;
	}
	err = pAxis->Enable1D();
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] EnableZCompensation Err:%d\n"), err);
	}
	return err;
}

long CPowerGantry::SetZCompensation(CString strAxisZ)
{
	PitchErrorCompensationData oneDCompData;
	int err = ErrorCode::None;
	long HeadNo = GetZAxisIndexByZName(strAxisZ);
	double zComp = 0.0, StartPos = 0.0;
	Cwmx3Axis* pAxis;
	pAxis = GetWmx3AxisByName(strAxisZ);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetZCompensation pAxis is NULL Err:%d\n"), err);
		return err;
	}
	if (HeadNo != NON)
	{
		StartPos = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, HeadNo - 1);
		TRACE(_T("[PWR] SetZCompensation %s StartPos:%.3f\n"), strAxisZ, StartPos);
		oneDCompData = PitchErrorCompensationData();
		oneDCompData.enable = 1;
		oneDCompData.pitchOriginPosition = StartPos * pAxis->GetUnResol();
		oneDCompData.pitchOriginIndex = 0; //Based on the current position value of axis, the variable should be different.
		oneDCompData.pitchCount = BUFSIZE;
		oneDCompData.pitchInterval = CAL_Z_PITCH_POSITION * pAxis->GetUnResol();
		oneDCompData.options.originPositionType = PitchErrorCompensationOriginPositionType::Absolute;
		for (int indx = 0; indx < BUFSIZE; ++indx)
		{
			zComp = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, HeadNo - 1, indx);
			oneDCompData.pitchCompensationValue[indx] = zComp * pAxis->GetUnResol();
			if (gcPowerLog->IsShowCalibrationLog() == true)
			{
				TRACE(_T("[PWR] pitchCompensationValue %s (%04d,%.3f)\n"), pAxis->GetAxisName(), indx, oneDCompData.pitchCompensationValue[indx]);
			}
		}
		err = pAxis->Set1DCompensationData(&oneDCompData);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] SetZCompensation(%s) Err:%d\n"), pAxis->GetAxisName(), err);
		}
	}
	return err;
}

long CPowerGantry::CheckLimitOver(CString strAxis, double Command)
{
	long AxisNo = NON, Err = NO_ERR;
	CString strMsg;
	Limit Limit;
	//TRACE(_T("[PWR] Monitoring CheckLimitOver(%s) Command:%.3f \n"), strAxis, Command);
	AxisNo = GetAxisIndexFromAliasName(strAxis);
	if (AxisNo != NON)
	{
		if (CheckServoOn(strAxis) == false)
		{
			Err = GetServoOnError(strAxis);
			TRACE(_T("[PWR] Servo Off Error(%s)\n"), strAxis);
			strMsg.Format(_T("Servo Off Error(%s)\n"), (LPCTSTR)strAxis);
			Err = SendAlarm(Err, strMsg);

			return Err;
		}

		if (AxisNo == (unsigned)PowerAxis::FX || AxisNo == (unsigned)PowerAxis::FY1 || AxisNo == (unsigned)PowerAxis::FY2)
		{
			Err = ANCDownBeforeMove(FRONT_GANTRY);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] ANC Down Check Err:%d\n"), Err);
				return Err;
			}
		}

		Limit = gcPowerCalibrationData->GetLimit(AxisNo);
		if (Command < Limit.minus)
		{
			TRACE(_T("[PWR] CheckLimitOver(%s) Command:%.3f (-)Limit:%.3f\n"), strAxis, Command, Limit.minus);
			Err = GetMinusLimitError(strAxis);
			strMsg.Format(_T("CheckLimitOver(%s) Command:%.3f (-)Limit:%.3f"), (LPCTSTR)strAxis, Command, Limit.minus);
			Err = SendAlarm(Err, strMsg);
		}
		else if (Command > Limit.plus)
		{
			TRACE(_T("[PWR] CheckLimitOver(%s) Command:%.3f (+)Limit:%.3f\n"), strAxis, Command, Limit.plus);
			Err = GetPlusLimitError(strAxis);
			strMsg.Format(_T("CheckLimitOver(%s) Command:%.3f (+)Limit:%.3f"), (LPCTSTR)strAxis, Command, Limit.plus);
			Err = SendAlarm(Err, strMsg);
		}
		else
		{
			if (gcPowerLog->IsShowMotionLog() == true)
			{
				TRACE(_T("[PWR] CheckLimitOver AxisNo:%d Minus:%.3f (%.3f) Plus:%.3f\n"), AxisNo, Limit.minus, Command, Limit.plus);
			}
			Err = NO_ERR;
		}
	}
	else
	{
		TRACE(_T("[PWR] CheckLimitOver(%s) Undefined Axis\n"), strAxis);
		Err = UNDEFINED_AXIS_NULL;
	}
	return Err;
}

long CPowerGantry::GetSuctionIONo(long Gantry, long HeadNo)
{
	long SuctionIONo = IO_NOUSE;
	if (Gantry == FRONT_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1:
			SuctionIONo = OUT_FHEAD1_SUC;
			break;
		case TBL_HEAD2:
			SuctionIONo = OUT_FHEAD2_SUC;
			break;
		case TBL_HEAD3:
			SuctionIONo = OUT_FHEAD3_SUC;
			break;
		case TBL_HEAD4:
			SuctionIONo = OUT_FHEAD4_SUC;
			break;
		case TBL_HEAD5:
			SuctionIONo = OUT_FHEAD5_SUC;
			break;
		case TBL_HEAD6:
			SuctionIONo = OUT_FHEAD6_SUC;
			break;
		default:
			break;
		}
	}
	else if (Gantry == REAR_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1:
			SuctionIONo = OUT_RHEAD1_SUC;
			break;
		case TBL_HEAD2:
			SuctionIONo = OUT_RHEAD2_SUC;
			break;
		case TBL_HEAD3:
			SuctionIONo = OUT_RHEAD3_SUC;
			break;
		case TBL_HEAD4:
			SuctionIONo = OUT_RHEAD4_SUC;
			break;
		case TBL_HEAD5:
			SuctionIONo = OUT_RHEAD5_SUC;
			break;
		case TBL_HEAD6:
			SuctionIONo = OUT_RHEAD6_SUC;
			break;
		default:
			break;
		}
	}
	return SuctionIONo;
}

long CPowerGantry::GetBlowIONo(long Gantry, long HeadNo)
{
	long BlowIONo = IO_NOUSE;
	if (Gantry == FRONT_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1:
			BlowIONo = OUT_FHEAD1_BLO;
			break;
		case TBL_HEAD2:
			BlowIONo = OUT_FHEAD2_BLO;
			break;
		case TBL_HEAD3:
			BlowIONo = OUT_FHEAD3_BLO;
			break;
		case TBL_HEAD4:
			BlowIONo = OUT_FHEAD4_BLO;
			break;
		case TBL_HEAD5:
			BlowIONo = OUT_FHEAD5_BLO;
			break;
		case TBL_HEAD6:
			BlowIONo = OUT_FHEAD6_BLO;
			break;
		default:
			break;
		}
	}
	else if (Gantry == REAR_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1:
			BlowIONo = OUT_RHEAD1_BLO;
			break;
		case TBL_HEAD2:
			BlowIONo = OUT_RHEAD2_BLO;
			break;
		case TBL_HEAD3:
			BlowIONo = OUT_RHEAD3_BLO;
			break;
		case TBL_HEAD4:
			BlowIONo = OUT_RHEAD4_BLO;
			break;
		case TBL_HEAD5:
			BlowIONo = OUT_RHEAD5_BLO;
			break;
		case TBL_HEAD6:
			BlowIONo = OUT_RHEAD6_BLO;
			break;
		default:
			break;
		}
	}
	return BlowIONo;
}

long CPowerGantry::SuctionOne(long Gantry, long HeadNo, bool bSuction)
{
	long OutputIO = IO_NOUSE, Err = NO_ERR, OnOff = OUTOFF;
	if (bSuction == true)
	{
		OnOff = OUTON;
	}
	else
	{
		OnOff = OUTOFF;
	}
	OutputIO = GetSuctionIONo(Gantry, HeadNo);
	if (OutputIO != IO_NOUSE)
	{
		SetOneSuction(Gantry, HeadNo,  bSuction);
		OutputOne(OutputIO, OnOff);

		if (OnOff == OUTOFF)
		{
			gcLastPickFront->SetHeadDataExist(HeadNo, false);
		}

		Err = NO_ERR;
	}
	else
	{
		Err = UNDEIFNED_IO;
	}
	return Err;
}

long CPowerGantry::BlowOne(long Gantry, long HeadNo, bool bBlow)
{
	long OutputIO = IO_NOUSE, Err = NO_ERR, OnOff = OUTOFF;
	if (bBlow == true)
	{
		OnOff = OUTON;
	}
	else
	{
		OnOff = OUTOFF;
	}
	OutputIO = GetBlowIONo(Gantry, HeadNo);
	if (OutputIO != IO_NOUSE)
	{
		SetOneBlow(Gantry, HeadNo, bBlow);
		OutputOne(OutputIO, OnOff);
		Err = NO_ERR;
	}
	else
	{
		Err = UNDEIFNED_IO;
	}
	return Err;
}

long CPowerGantry::SuctionAll(long Gantry, bool bSuction)
{
	long Err = NO_ERR;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		Err = SuctionOne(Gantry, TBL_HEAD1 + indx, bSuction);
		if (Err != NO_ERR)
		{
			break;
		}
	}
	return Err;
}

long CPowerGantry::BlowAll(long Gantry, bool bBlow)
{
	long Err = NO_ERR;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		Err = BlowOne(Gantry, TBL_HEAD1 + indx, bBlow);
		if (Err != NO_ERR)
		{
			break;
		}
	}
	return Err;
}

long CPowerGantry::SetOneSuction(long Gantry, long HeadNo, bool bSuction)
{
	long Err = NO_ERR;
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_bSuction[HeadNo - 1] = bSuction;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetOneSuction Gantry:%d HeadNo:%d bSuction:%d\n"), Gantry, HeadNo, bSuction);
		}
		Err = NO_ERR;
	}
	else
	{
		Err = INVALID_HEADNO;
	}
	return Err;
}

long CPowerGantry::SetOneBlow(long Gantry, long HeadNo, bool bBlow)
{
	long Err = NO_ERR;
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		m_bBlow[HeadNo - 1] = bBlow;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetOneBlow Gantry:%d HeadNo:%d bBlow:%d\n"), Gantry, HeadNo, bBlow);
		}
		Err = NO_ERR;
	}
	else
	{
		Err = INVALID_HEADNO;
	}
	return Err;
}

bool CPowerGantry::GetOneSuction(long Gantry, long HeadNo)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetOneSuction Gantry:%d HeadNo:%d bSuction:%d\n"), Gantry, HeadNo, m_bSuction[HeadNo - 1]);
		}
		return m_bSuction[HeadNo - 1];
	}
	else
	{
		return false;
	}
}

bool CPowerGantry::GetOneBlow(long Gantry, long HeadNo)
{
	if (HeadNo >= TBL_HEAD1 && HeadNo <= TBL_HEAD6)
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetOneBlow Gantry:%d HeadNo:%d bBlow:%d\n"), Gantry, HeadNo, m_bBlow[HeadNo - 1]);
		}
		return m_bBlow[HeadNo - 1];
	}
	else
	{
		return false;
	}
}


bool CPowerGantry::GetAllSuction(long Gantry)
{
	long HeadNo = 0;
	for (HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		if (GetOneSuction(Gantry, TBL_HEAD1 + HeadNo) == true)
		{
			return true;
		}
	}
	return false;
}

bool CPowerGantry::GetAllBlow(long Gantry)
{
	long HeadNo = 0;
	for (HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		if (GetOneBlow(Gantry, TBL_HEAD1 + HeadNo) == true)
		{
			return true;
		}
	}
	return false;
}

long CPowerGantry::CheckAirPressureLow()
{
	long Err = NO_ERR;

	if (GetSkipMotorPower() == true)
	{
		return Err;
	}

	if (InputTimeOne(IN_AIR_PRESS_LOW, INOFF, TIME10MS) == true)
	{
		Err = LOW_AIR;
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] CheckAirPressureLow Err:%d\n"), Err);
	}
	return Err;
}

long CPowerGantry::CheckLinearMotorTemperatureHigh(long Gantry)
{
	long TempX, TempY1, TempY2, Err = NO_ERR;
	
	CString strMsg;

	strMsg.Format(_T("NO_ERR"));

	if (Gantry == FRONT_GANTRY)
	{
		TempX = IN_FX_LMTEMP_LOW;
		TempY1 = IN_FY1_LMTEMP_LOW;
		TempY2 = IN_FY2_LMTEMP_LOW;
	}
	else
	{
		TempX = IN_RX_LMTEMP_LOW;
		TempY1 = IN_RY1_LMTEMP_LOW;
		TempY2 = IN_RY2_LMTEMP_LOW;
	}
	if (GetUseRTDSensorFX() == 0)
	{
		if (InputOne(TempX) == INOFF) // X Temp High
		{
			Err = LM_TEMP_HIGH_FX + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisX(Gantry));
		}
	}
	else
	{
		if (GetTemperature(_T("FX")) > HIGH_TEMPERATURE)
		{
			Err = LM_TEMP_HIGH_FX + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisX(Gantry));

		}
		//else if (GetTemperature(_T("FX")) < LOW_TEMPERATURE)
		//{
		//	Err = LM_TEMP_LOW_FX + (Gantry * 3);
		//	strMsg.Format(_T("%s Temperature low"), (LPCTSTR)GetAxisX(Gantry));

		//}
	}
	if (GetUseRTDSensorFY1() == 0)
	{
		if (InputOne(TempY1) == INOFF) // Y1 Temp High
		{
			Err = LM_TEMP_HIGH_FY1 + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY1(Gantry));

		}
	}
	else
	{
		if (GetTemperature(_T("FY1")) > HIGH_TEMPERATURE)
		{
			Err = LM_TEMP_HIGH_FY1 + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY1(Gantry));

		}
		//else if (GetTemperature(_T("FY1")) < LOW_TEMPERATURE)
		//{
		//	Err = LM_TEMP_LOW_FY1 + (Gantry * 3);
		//	strMsg.Format(_T("%s Temperature low"), (LPCTSTR)GetAxisY1(Gantry));

		//}
	}
	if (GetUseRTDSensorFY2() == 0)
	{
		if (InputOne(TempY2) == INOFF) // Y2 Temp High
		{
			Err = LM_TEMP_HIGH_FY2 + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY2(Gantry));
		}
	}
	else
	{
		if (GetTemperature(_T("FY2")) > HIGH_TEMPERATURE)
		{
			Err = LM_TEMP_HIGH_FY2 + (Gantry * 3);
			strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY2(Gantry));

		}
		//else if (GetTemperature(_T("FY2")) < LOW_TEMPERATURE)
		//{
		//	Err = LM_TEMP_LOW_FY2 + (Gantry * 3);
		//	strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY2(Gantry));

		//}
	}
	if (GetUseAreaSensor() == 1)
	{
		if (InputElapsedTimeOne(IN_FRONT_AREA_SENSOR, INOFF, TIME20MS) == true)
		{
			Err = DETECTED_AREA_SENSOR_FRONT;
			strMsg.Format(_T("Front Area Sensor Detected"));

		}
		if (InputElapsedTimeOne(IN_REAR_AREA_SENSOR, INOFF, TIME20MS) == true)
		{
			Err = DETECTED_AREA_SENSOR_REAR;
			strMsg.Format(_T("Rear Area Sensor Detected"));
		}
	}

	if (GetUseAreaSensor2nd() == 1)
	{
		if (InputElapsedTimeOne(IN_FRONT_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
		{
			Err = DETECTED_AREA_SENSOR_FRONT;
			strMsg.Format(_T("Front Area Sensor2nd Detected"));

		}
		if (InputElapsedTimeOne(IN_REAR_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
		{
			Err = DETECTED_AREA_SENSOR_REAR;
			strMsg.Format(_T("Rear Area Sensor2nd Detected"));
		}
	}

	if (GetSkipMotorPower() == true) // Simulation
	{
		Err = NO_ERR;
	}

	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] CheckLinearMotorTemperatureHigh Err:%d\n"), Err);

		Err = SendAlarm(Err, strMsg);
	}
	return Err;
}

long CPowerGantry::StartMultiPosition(Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal)
{
	long Err = NO_ERR, AxisIndex = 0;
	INT_PTR LinearIntpCount = 0;
	LinearIntpCount = GetWmx3LinearIntpAxisCount();
	StartPosStruct* pStartPos;
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartMultiPosition Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		Err = UNDER_MIN_LINEAR_INTP_AXIS;
	}
	else
	{
		Err = CheckLinearMotorTemperatureHigh(FRONT_GANTRY);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPosition CheckLinearMotorTemperatureHigh Err:%d\n"), Err);
		}
		else
		{
			/*for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			{
				pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);
				AxisIndex = GetAxisIndexFromAliasName(pStartPos->StrAxis);
				if (AxisIndex == static_cast<int>(PowerAxis::FX) || AxisIndex == static_cast<int>(PowerAxis::RX) || AxisIndex == static_cast<int>(PowerAxis::FY1) || AxisIndex == static_cast<int>(PowerAxis::RY1))
				{
					if (gcPowerLog->IsShowMotionLockLog() == true)
					{
						TRACE(_T("[PWR] StartMultiPosition AxisIndex(%d) Lock-1\n"), AxisIndex);
					}
					SEM_LOCK(gMOTION_LOCK[AxisIndex], INFINITE);
				}
			}*/

			for (long AxisNo = 0; AxisNo < GetWmx3LinearIntpAxisCount(); ++AxisNo)
			{
				pStartPos = GetWmx3LinearIntpAxisByIndex(AxisNo);

				if (IsMoveOnceAxis(pStartPos->StrAxis) == true)
				{
					Err = MoveOnceLockPauseMode(pStartPos->StrAxis, pStartPos->CommandPos);

					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] %s StartMultiPosition STOP_NOW\n"), pStartPos->StrAxis);
						return Err;
					}

					break;
				}
			}

			Err = gcWmx3LinearIntplPos->StartPathLinearIntplPos(Linear, StartCenter, EndCenter, Goal);
		}
	}
	return Err;
}

bool CPowerGantry::IsOneAxisHomingFail(CString strAxis)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->GetInitializeFail() == true)
		return true;
	return false;
}

bool CPowerGantry::IsOneAxisHomingFail(INT_PTR indx)
{
	Cwmx3Axis* pAxis = NULL;
	pAxis = GetWmx3AxisByIndex(indx);
	if (pAxis->GetAxisMap() == NON)
		return false;
	if (pAxis->GetInitializeFail() == true)
		return true;
	return false;
}

bool CPowerGantry::IsAllAxisHomingFail()
{
	INT_PTR indx = 0;
	long nHomedCount = 0, nHomedYetCount = 0;
	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		if (IsOneAxisHomingFail(indx) == true)
		{
			break;
		}
	}
	if (indx == GetWmx3AxisCount())
		return false;
	else
		return true;
}


bool CPowerGantry::IsMoveOnceAxis(CString strAxis)
{
	bool result = false;
	Cwmx3Axis* pAxis;

	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] IsMoveOnceAxis %s is NON\n"), strAxis);
		return result;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] IsMoveOnceAxis %s is NULL\n"), strAxis);
		return result;
	}

	if (pAxis->IsGantryAxis() == true)
	{
		result = true;
	}
	else if (GetMoveOnceAxisRZ() == true && pAxis->IsZAxis() == true)
	{
		result = true;
	}
	else if (GetMoveOnceAxisRZ() == true && pAxis->IsRAxis() == true)
	{
		result = true;
	}
	else if (pAxis->IsTTFZAxis() == true)
	{
		result = true;
	}

	return result;
}

void CPowerGantry::SetMoveOnceAxisRZ(bool Set)
{
	m_MoveOnceZREnable = Set;
}

bool CPowerGantry::GetMoveOnceAxisRZ()
{
	return m_MoveOnceZREnable;
}

void CPowerGantry::SetAxisPause(CString strAxis, bool Pause)
{
	bool result = false;
	Cwmx3Axis* pAxis;

	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] SetAxisPause %s is NON\n"), strAxis);
		return;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetAxisPause %s is NULL\n"), strAxis);
		return;
	}

	if (m_bPause[pAxis->GetAxisMap()] != Pause)
	{
		TRACE(_T("[PWR] SetAxisPause %s %d->%d\n"), strAxis, m_bPause[pAxis->GetAxisMap()], Pause);
	}

	m_bPause[pAxis->GetAxisMap()] = Pause;
}

bool CPowerGantry::GetAxisPause(CString strAxis)
{
	bool result = false;
	Cwmx3Axis* pAxis;

	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		TRACE(_T("[PWR] SetAxisPause %s is NON\n"), strAxis);
		return result;
	}
	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] SetAxisPause %s is NULL\n"), strAxis);
		return result;
	}

	return m_bPause[pAxis->GetAxisMap()];
}

bool CPowerGantry::WaitAxisPauseState(long TimeOut)
{
	bool result = false;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();

	while (1)
	{
		for (unsigned indx = 0; indx < MAXGANTRYAXISNO; ++indx)
		{
			if (m_bPause[indx] == true)
			{
				TRACE(_T("[PWR] WaitAxisPauseState OK(%s)\n"), GetAxisNameByAxisIndex(indx));
				return true;
			}
		}

		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitAxisPauseState TimeOut(%d)\n"), TimeOut);
			return false;
		}

		ThreadSleep(TIME100MS);
	}

	return false;
}

void CPowerGantry::ResetAxisPause()
{
	for (int indx = 0; indx < MAXGANTRYAXISNO; ++indx)
	{
		m_bPause[indx] = false;
	}
}

void CPowerGantry::SetLockOKAxisName(CString strAxis)
{
	m_LockOKAxis = strAxis;
}

CString CPowerGantry::GetLockOKAxisName()
{
	return m_LockOKAxis;
}

bool CPowerGantry::IsAxisStatusJog(CString strAxis)
{
	CString strFunc(__func__);
	long Err = NO_ERR;
	Cwmx3Axis* pAxis;
	CoreMotionAxisStatus status;
	bool bRet = false;

	pAxis = GetWmx3AxisByName(strAxis);
	if (pAxis == NULL)
	{
		TRACE(_T("[PWR] %s %s is NULL\n"), strFunc, strAxis);
		return bRet;
	}

	bRet = pAxis->IsAxisJog();
	return bRet;
}
