#include "pch.h"
#include "Cwmx3Axis.h"
#include "Cwmx3Init.h"
#include "GlobalDefine.h"
#include "Wmx3MotorDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CApplicationTime.h"
#include "DefineThreadLoopTime.h"
#include "CSlaveMotorStatus.h"
//#include "ErrorCode.h"
#include "CThreadException.h"
#include "CPowerLog.h"
#include "LockDef.h"

#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <AdvancedMotionApi.h>
#include <ecApi.h>
#include <IOApi.h>
#include <EventApi.h>
#include <mutex>

using namespace wmx3Api;
using namespace std;

Cwmx3Axis::Cwmx3Axis(bool bSimulation)
{
	TRACE(_T("[PWR] Cwmx3Axis size:%d\n"), sizeof(Cwmx3Axis));

	m_nAxisMap = NON;
	m_nIndex = INITIALZE;
	m_bSimulation = bSimulation;
	m_dblVirtualMinusLimit = 0.0;
	m_dblVirtualPlusLimit = 0.0;
	m_dblResol = 0.0;
	m_dblUnResol = 0.0;
	m_bInitializeEnd = false;
	m_bErrorStatus = false;
	m_OpState = 0;
	m_Ratio = 1.0;		// 0.1 is 10% ~ 1 is 100%
	m_Ratio2nd = 1.0;	// 0.1 is 10% ~ 1 is 100%
	ZeroMemory(m_ErrString, sizeof(m_ErrString));
	m_LinearIntp = 0.0;
	m_ForceOriginOffset = 0.0;
	m_CmdLock = CreateMutex(NULL, FALSE, NULL);
	m_HomingMaxTimeOut = 30000;
	m_AxisSkip = false;
	m_LastCommandPosition = 0.000;
	m_LastCommandServoOn = true;
}

Cwmx3Axis::~Cwmx3Axis(void)
{
	TRACE(_T("[PWR] ~Cwmx3Axis Start Axis(%02d,%02d) Done\n"), GetAxisIndex(), GetAxisMap());
	Lock();
	TRACE(_T("[PWR] ~Cwmx3Axis Ending Axis(%02d,%02d) Done\n"), GetAxisIndex(), GetAxisMap());
	CloseHandle(m_CmdLock);
	TRACE(_T("[PWR] ~Cwmx3Axis End(%02d,%02d) Done\n"), GetAxisIndex(), GetAxisMap());
}

bool Cwmx3Axis::Lock()
{
	SEM_LOCK(m_CmdLock, INFINITE);
	return true;

}
bool Cwmx3Axis::Unlock()
{
	SEM_UNLOCK(m_CmdLock);
	return true;
}

void Cwmx3Axis::SetAxisSkip(bool set)
{
	TRACE(_T("[PWR] SetAxisSkip %s:%d\n"), GetAxisName(), set);

	m_AxisSkip = set;
}

bool Cwmx3Axis::GetAxisSkip()
{
	return m_AxisSkip;
}

void Cwmx3Axis::SetLastCommandPosition(double position)
{
	TRACE(_T("[PWR] SetLastCommandPosition %s:%.3f\n"), GetAxisName(), position);
	m_LastCommandPosition = position;
}

double Cwmx3Axis::GetLastCommandPosition()
{
	return m_LastCommandPosition;
}

void Cwmx3Axis::SetLastCommandServoOn(bool on)
{
	TRACE(_T("[PWR] SetLastCommandServoOn %s:%.3f\n"), GetAxisName(), on);
	m_LastCommandServoOn = on;
}

bool Cwmx3Axis::GetLastCommandServoOn()
{
	return m_LastCommandServoOn;
}

void Cwmx3Axis::InitAxisData()
{
}

void Cwmx3Axis::SetInitializeEnd(bool bEnd)
{
	m_bInitializeEnd = bEnd;
}

bool Cwmx3Axis::GetInitializeEnd()
{
	return m_bInitializeEnd;
}

void Cwmx3Axis::SetMotionControlType(CMotionControlType motionControlType)
{
	m_nMotionControlType = motionControlType;
}

void Cwmx3Axis::SetMotorType(CMotorType motorType)
{
	m_nMotorType = motorType;
}

CMotionControlType Cwmx3Axis::GetMotionControlType()
{
	return m_nMotionControlType;
}

CMotorType Cwmx3Axis::GetMotorType()
{
	return m_nMotorType;
}

void Cwmx3Axis::Initialize(void)
{
}

void Cwmx3Axis::SetAxisName(CString strAxisName)
{
	m_AxisName = strAxisName;
}

CString Cwmx3Axis::GetAxisName()
{
	return m_AxisName;
}

CString Cwmx3Axis::GetSlaveAxisName()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FY1))
	{
		return _T("FY2");
	}

	if (GetAxisIndex() == static_cast<int>(PowerAxis::RY1))
	{
		return _T("RY2");
	}

    return m_AxisName;

}

int Cwmx3Axis::GetAxisIndex()
{
	return m_nIndex;
}

void Cwmx3Axis::SetAxisInformation(unsigned nSlaveId, unsigned nIndex)
{
	m_nAxisMap = nSlaveId;
	m_nIndex = nIndex;
}

int Cwmx3Axis::GetAxisMap()
{
	return m_nAxisMap;
}

bool Cwmx3Axis::GetUseSlaveAxis()
{
	return m_bUseSlave;
}

void Cwmx3Axis::SetUseSlaveAxis(bool bSlave)
{
	m_bUseSlave = bSlave;
}

int Cwmx3Axis::GetSlaveAxisIndex()
{
	return m_nSlaveAxis_Index;
}

int Cwmx3Axis::GetSlaveAxisSlaveID()
{
	return GetWmx3AxisByIndex(m_nSlaveAxis_Index)->GetAxisMap();
}

void Cwmx3Axis::SetSlaveAxisIndex(unsigned nSlaveAxisIndex)
{
	m_nSlaveAxis_Index = nSlaveAxisIndex;
}

void Cwmx3Axis::SetResol(double dblResol)
{
	m_dblResol = dblResol;
}

double Cwmx3Axis::GetResol()
{
	return m_dblResol;
}

void Cwmx3Axis::SetUnResol(double dblUnResol)
{
	m_dblUnResol = dblUnResol;
}

double Cwmx3Axis::GetUnResol()
{
	return m_dblUnResol;
}

void Cwmx3Axis::SetLinearIntp(double LinearIntp)
{
	m_LinearIntp = LinearIntp;
}

double Cwmx3Axis::GetLinearIntp()
{
	return m_LinearIntp;
}

void Cwmx3Axis::SetForceOriginOffset(double ForceOffset) 
{
	m_ForceOriginOffset = ForceOffset;
}

double Cwmx3Axis::GetForceOriginOffset()
{
	return m_ForceOriginOffset;
}

void Cwmx3Axis::SetRatio(double Ratio)
{
	Lock();
	m_Ratio = Ratio;
	Unlock();
}

double Cwmx3Axis::GetRatio(void)
{
	double Ratio = m_Ratio;
	return Ratio;
}

void Cwmx3Axis::Set2ndRatio(double Ratio)
{
	Lock();
	m_Ratio2nd = Ratio;
	Unlock();
}

double Cwmx3Axis::Get2ndRatio(void)
{
	double Ratio = m_Ratio2nd;
	return Ratio;
}


void Cwmx3Axis::InitializeRatio(void)
{
	SetRatio(1.0);
	Set2ndRatio(1.0);
}

void Cwmx3Axis::SetMovingDir(signed Dir)
{
	m_MovingDir = Dir;
}

signed Cwmx3Axis::GetMovingDir()
{
	return m_MovingDir;
}
void Cwmx3Axis::SetAxisSlaveNo(unsigned SlaveNo)
{
	m_nSlaveNo = SlaveNo;
}
int Cwmx3Axis::GetAxisSlaveNo()
{
	return m_nSlaveNo;
}

bool Cwmx3Axis::IsSimulationMode()
{
	return m_bSimulation;
}

void Cwmx3Axis::SetAsSimulation()
{
	m_bSimulation = TRUE;
}

bool Cwmx3Axis::IsGantryAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FX) || GetAxisIndex() == static_cast<int>(PowerAxis::RX))
	{
		return true;
	}
	else if (GetAxisIndex() == static_cast<int>(PowerAxis::FY1) || GetAxisIndex() == static_cast<int>(PowerAxis::RY1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsYAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FY1) || GetAxisIndex() == static_cast<int>(PowerAxis::RY1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsXAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FX) || GetAxisIndex() == static_cast<int>(PowerAxis::RX))
	{
		return true;
	}
	else
	{
		return false;
	}
}


bool Cwmx3Axis::IsTTFZAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::TTF1_Z) || GetAxisIndex() == static_cast<int>(PowerAxis::TTF2_Z))
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool Cwmx3Axis::IsTTFYAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::TTF1_Y) || GetAxisIndex() == static_cast<int>(PowerAxis::TTF2_Y))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsTTFAxis(long TTFNumber)
{
	bool result = false;
	if (TTFNumber == TTF1)
	{
		if (GetAxisIndex() == static_cast<int>(PowerAxis::TTF1_Y) || GetAxisIndex() == static_cast<int>(PowerAxis::TTF1_Z))
		{
			return true;
		}
	}
	else if (TTFNumber == TTF2)
	{
		if (GetAxisIndex() == static_cast<int>(PowerAxis::TTF2_Y) || GetAxisIndex() == static_cast<int>(PowerAxis::TTF2_Z))
		{
			return true;
		}
	}

	return result;
}

bool Cwmx3Axis::IsSlaveAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FY2) || GetAxisIndex() == static_cast<int>(PowerAxis::RY2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsZAxis()
{
	if (GetAxisIndex() >= static_cast<int>(PowerAxis::FZ1) && GetAxisIndex() <= static_cast<int>(PowerAxis::FZ6))
	{
		return true;
	}
	else if (GetAxisIndex() >= static_cast<int>(PowerAxis::RZ1) && GetAxisIndex() <= static_cast<int>(PowerAxis::RZ6))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsRAxis()
{
	if (GetAxisIndex() >= static_cast<int>(PowerAxis::FW1) && GetAxisIndex() <= static_cast<int>(PowerAxis::FW6))
	{
		return true;
	}
	else if (GetAxisIndex() >= static_cast<int>(PowerAxis::RW1) && GetAxisIndex() <= static_cast<int>(PowerAxis::RW6))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsConveyorAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FCONV) || GetAxisIndex() == static_cast<int>(PowerAxis::RCONV))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsConveyorBeltAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FBTIN) || GetAxisIndex() == static_cast<int>(PowerAxis::FBTWK) || GetAxisIndex() == static_cast<int>(PowerAxis::FBTOT))
	{
		return true;
	}
	else if (GetAxisIndex() == static_cast<int>(PowerAxis::RBTIN) || GetAxisIndex() == static_cast<int>(PowerAxis::RBTWK) || GetAxisIndex() == static_cast<int>(PowerAxis::RBTOT))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsPusherZAxis()
{
	if (GetAxisIndex() == static_cast<int>(PowerAxis::FPUZ) || GetAxisIndex() == static_cast<int>(PowerAxis::RPUZ))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Cwmx3Axis::IsY1Axis(long Gantry)
{
	if (Gantry == FRONT_GANTRY && GetAxisIndex() == static_cast<int>(PowerAxis::FY1))
	{
		return true;
	}
	else if (Gantry == REAR_GANTRY && GetAxisIndex() == static_cast<int>(PowerAxis::RY1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

int Cwmx3Axis::SetFeedbackParam(Config::FeedbackParam* feedParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->config->SetFeedbackParam(GetAxisMap(), feedParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetFeedbackParam"));
	}
	Unlock();
	return err;
}

int Cwmx3Axis::GetFeedbackParam(Config::FeedbackParam* pFeedParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::FeedbackParam feedParam;
	err = GetCoreMotion()->config->GetFeedbackParam(GetAxisMap(), &feedParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetHomeParam"));
	}
	*pFeedParam = feedParam;
	return err;
}

int Cwmx3Axis::SetHomeParam(Config::HomeParam* homeParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->config->SetHomeParam(GetAxisMap(), homeParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetHomeParam"));
	}
	Unlock();
	return err;
}

int Cwmx3Axis::GetHomeParam(Config::HomeParam* pHomeParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::HomeParam homeParam;
	err = GetCoreMotion()->config->GetHomeParam(GetAxisMap(), &homeParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetHomeParam"));
	}
	*pHomeParam = homeParam;
	return err;
}

int Cwmx3Axis::SetSlaveHomeParam(Config::HomeParam* homeParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->config->SetHomeParam(GetSlaveAxisSlaveID(), homeParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetHomeParam"));
	}
	Unlock();
	return err;
}

int Cwmx3Axis::GetSlaveHomeParam(Config::HomeParam* pHomeParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::HomeParam homeParam;
	err = GetCoreMotion()->config->GetHomeParam(GetSlaveAxisSlaveID(), &homeParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetHomeParam"));
	}
	*pHomeParam = homeParam;
	return err;
}

int Cwmx3Axis::SetHWLimitParam(Config::LimitParam* limitParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->config->SetLimitParam(GetAxisMap(), limitParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetLimitParam(HW)"));
	}
	Unlock();
	return err;
}

int Cwmx3Axis::SetHWLimitParam(Config::LimitParam* CurLimitParam, WMX3AxisLimitSwitchType posLimitType, WMX3AxisLimitSwitchType negLimitType)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	Config::LimitParam limitParamError;
	Config::LimitSwitchType limitType;

	limitParamError = *CurLimitParam;
	limitParam = *CurLimitParam;
	switch (posLimitType)
	{
	case WMX3AxisLimitSwitchType::None:							limitParam.positiveLSType = limitType.None;							break;
	case WMX3AxisLimitSwitchType::ServoOff:						limitParam.positiveLSType = limitType.ServoOff;						break;
	case WMX3AxisLimitSwitchType::DecServoOff:					limitParam.positiveLSType = limitType.DecServoOff;					break;
	case WMX3AxisLimitSwitchType::Dec:							limitParam.positiveLSType = limitType.Dec;							break;
	case WMX3AxisLimitSwitchType::SlowDecServoOff:				limitParam.positiveLSType = limitType.SlowDecServoOff;				break;
	case WMX3AxisLimitSwitchType::SlowDec:						limitParam.positiveLSType = limitType.SlowDec;						break;
	case WMX3AxisLimitSwitchType::SeparatePositiveLSNegativeLS: limitParam.positiveLSType = limitType.SeparatePositiveLSNegativeLS;	break;
	default:
		break;
	}
	switch (negLimitType)
	{
	case WMX3AxisLimitSwitchType::None:							limitParam.negativeLSType = limitType.None;							break;
	case WMX3AxisLimitSwitchType::ServoOff:						limitParam.negativeLSType = limitType.ServoOff;						break;
	case WMX3AxisLimitSwitchType::DecServoOff:					limitParam.negativeLSType = limitType.DecServoOff;					break;
	case WMX3AxisLimitSwitchType::Dec:							limitParam.negativeLSType = limitType.Dec;							break;
	case WMX3AxisLimitSwitchType::SlowDecServoOff:				limitParam.negativeLSType = limitType.SlowDecServoOff;				break;
	case WMX3AxisLimitSwitchType::SlowDec:						limitParam.negativeLSType = limitType.SlowDec;						break;
	case WMX3AxisLimitSwitchType::SeparatePositiveLSNegativeLS: limitParam.negativeLSType = limitType.SeparatePositiveLSNegativeLS;	break;
	default:
		break;
	}
	Lock();
	err = GetCoreMotion()->config->SetLimitParam(GetAxisMap(), &limitParam, &limitParamError);
	Unlock();
	if (err != ErrorCode::None)
	{
		if (err == CoreMotionErrorCode::ParameterSettingsInvalid)
		{
			CoreMotionErrorToString(err, _T("SetLimitParam(HW) ParameterSettingsInvalid"));
		}
		else
		{
			CoreMotionErrorToString(err, _T("SetLimitParam(HW)"));
		}
	}
	return err;
}

int Cwmx3Axis::GetHWLimitParam(Config::LimitParam* pLimitParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	err = GetCoreMotion()->config->GetLimitParam(GetAxisMap(), &limitParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetLimitParam(HW)"));
	}
	*pLimitParam = limitParam;
	return err;
}

int Cwmx3Axis::SetVirtualLimitParam(Config::LimitParam limitParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->config->SetLimitParam(GetAxisMap(), &limitParam);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetLimitParam(SW)"));
	}
	return err;
}

int Cwmx3Axis::SetVirtualLimitParam(Config::LimitParam CurLimitParam, WMX3AxisLimitSwitchType posLimitType, double positivePos, WMX3AxisLimitSwitchType negLimitType, double negativePos)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	Config::LimitParam limitParamError;
	Config::LimitSwitchType limitType;

	limitParamError = CurLimitParam;
	limitParam = CurLimitParam;
	switch (posLimitType)
	{
	case WMX3AxisLimitSwitchType::None:							limitParam.positiveSoftLimitType = limitType.None;							break;
	case WMX3AxisLimitSwitchType::ServoOff:						limitParam.positiveSoftLimitType = limitType.ServoOff;						break;
	case WMX3AxisLimitSwitchType::DecServoOff:					limitParam.positiveSoftLimitType = limitType.DecServoOff;					break;
	case WMX3AxisLimitSwitchType::Dec:							limitParam.positiveSoftLimitType = limitType.Dec;							break;
	case WMX3AxisLimitSwitchType::SlowDecServoOff:				limitParam.positiveSoftLimitType = limitType.SlowDecServoOff;				break;
	case WMX3AxisLimitSwitchType::SlowDec:						limitParam.positiveSoftLimitType = limitType.SlowDec;						break;
	case WMX3AxisLimitSwitchType::SeparatePositiveLSNegativeLS: limitParam.positiveSoftLimitType = limitType.SeparatePositiveLSNegativeLS;	break;
	default:
		break;
	}
	limitParam.softLimitPositivePos = positivePos * m_dblUnResol;
	switch (negLimitType)
	{
	case WMX3AxisLimitSwitchType::None:							limitParam.negativeSoftLimitType = limitType.None;							break;
	case WMX3AxisLimitSwitchType::ServoOff:						limitParam.negativeSoftLimitType = limitType.ServoOff;						break;
	case WMX3AxisLimitSwitchType::DecServoOff:					limitParam.negativeSoftLimitType = limitType.DecServoOff;					break;
	case WMX3AxisLimitSwitchType::Dec:							limitParam.negativeSoftLimitType = limitType.Dec;							break;
	case WMX3AxisLimitSwitchType::SlowDecServoOff:				limitParam.negativeSoftLimitType = limitType.SlowDecServoOff;				break;
	case WMX3AxisLimitSwitchType::SlowDec:						limitParam.negativeSoftLimitType = limitType.SlowDec;						break;
	case WMX3AxisLimitSwitchType::SeparatePositiveLSNegativeLS: limitParam.negativeSoftLimitType = limitType.SeparatePositiveLSNegativeLS;	break;
	default:
		break;
	}
	limitParam.softLimitNegativePos = negativePos * m_dblUnResol;
	Lock();
	err = GetCoreMotion()->config->SetLimitParam(GetAxisMap(), &limitParam, &limitParamError);
	Unlock();
	if (err != ErrorCode::None)
	{
		if (err == CoreMotionErrorCode::ParameterSettingsInvalid)
		{
			CoreMotionErrorToString(err, _T("SetLimitParam(SW) ParameterSettingsInvalid"));
		}
		else
		{
			CoreMotionErrorToString(err, _T("SetLimitParam(SW)"));
		}
	}
	return err;
}

int Cwmx3Axis::GetVirtualLimitParam(Config::LimitParam* pLimitParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	err = GetCoreMotion()->config->GetLimitParam(GetAxisMap(), &limitParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetLimitParam(SW)"));
	}
	*pLimitParam = limitParam;
	return err;
}

int Cwmx3Axis::ReadVirtualPositiveLimit(double* pLimit)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	err = GetCoreMotion()->config->GetLimitParam(GetAxisMap(), &limitParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetLimitParam(SW)"));
	}
	*pLimit = limitParam.softLimitPositivePos;
	return err;
}

int Cwmx3Axis::ReadVirtualNegativeLimit(double* nLimit)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::LimitParam limitParam;
	err = GetCoreMotion()->config->GetLimitParam(GetAxisMap(), &limitParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetLimitParam(SW)"));
	}
	*nLimit = limitParam.softLimitNegativePos;
	return err;
}

int Cwmx3Axis::StartHome()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err  = GetCoreMotion()->home->StartHome(GetAxisMap());
	Unlock();
	return err;
}

int Cwmx3Axis::ContinueHome()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return ErrorCode::None;
	Lock();
	err  = GetCoreMotion()->home->Continue(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Continue"));
	}
	return err;
}

int Cwmx3Axis::CancelHome()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return ErrorCode::None;
	Lock();
	err = GetCoreMotion()->home->Cancel(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Cancel"));
	}
	return err;
}

int Cwmx3Axis::SetHomeDone(unsigned char value)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return ErrorCode::None;
	Lock();
	err = GetCoreMotion()->home->SetHomeDone(GetAxisMap(), value);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetHomeDone"));
	}
	return err;
}

int Cwmx3Axis::GetHomeData()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return ErrorCode::None;
	Home::HomeData HomeData = Home::HomeData();
	Lock();
	err = GetCoreMotion()->home->GetHomeData(&HomeData);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetHomeData"));
	}
	return err;
}

double Cwmx3Axis::GetHomeDiffZPulse()
{
	if (GetGlobalSimulationMode() == true)
	{
		return 11.0;
	}

	double Value = 0.0f;
	int err = ErrorCode::None;
	if (m_bSimulation)	return Value;
	Home::HomeData HomeData = Home::HomeData();
	err = GetCoreMotion()->home->GetHomeData(&HomeData);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetHomeData"));
	}
	Value = HomeData.axesHomeData[GetSlaveAxisSlaveID()].distZPulseToMasterZPulse;
	return Value;
}

int Cwmx3Axis::ServoOn()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;

	m_LastCommandServoOn = true;
	if (GetAxisSkip() == true)
	{
		return err;
	}

	Lock();
	err = GetCoreMotion()->axisControl->SetServoOn(GetAxisMap(), SERVO_ON);
	if (gcPowerLog->IsShowMotionLog() == true)
	{
		TRACE(_T("[PWR] SetServoOn Axis(%d,%d) %s\n"), GetAxisIndex(), GetAxisMap(), err);
	}
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetServoOn ON"));
	}
	return err;
}

int Cwmx3Axis::SlaveServoOn()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisMap() == GetSlaveAxisSlaveID()) // Same Axis
	{
		return err;
	}
	m_LastCommandServoOn = true;

	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->SetServoOn(GetSlaveAxisSlaveID(), SERVO_ON);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetServoOn Slave ON"));
	}
	return err;
}

int Cwmx3Axis::ServoOff()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;

	m_LastCommandServoOn = false;

	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->SetServoOn(GetAxisMap(), SERVO_OFF);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetServoOn OFF"));
	}
	return err;
}

int Cwmx3Axis::SlaveServoOff()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	m_LastCommandServoOn = false;

	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->SetServoOn(GetSlaveAxisSlaveID(), SERVO_OFF);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetServoOn Slave OFF"));
	}
	return err;
}

int Cwmx3Axis::ClearAmpAlarm()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->ClearAmpAlarm(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ClearAmpAlarm"));
	}
	return err;
}

int Cwmx3Axis::ClearSlaveAmpAlarm()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->ClearAmpAlarm(GetSlaveAxisSlaveID());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ClearAmpAlarm Slave"));
	}
	return err;
}

int Cwmx3Axis::ClearAxisAlarm()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->ClearAxisAlarm(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ClearAxisAlarm"));
	}
	return err;
}

int Cwmx3Axis::ClearSlaveAxisAlarm()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->axisControl->ClearAxisAlarm(GetSlaveAxisSlaveID());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ClearAxisAlarm Slave"));
	}
	return err;
}

double Cwmx3Axis::ReadCommandPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->axisControl->GetPosCommand(GetAxisMap(), &dblPosition);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetPosCommand"));
	}
	dblPosition = dblPosition * m_dblResol;
	return dblPosition;
}
double Cwmx3Axis::ReadProfileTargetPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->GetStatus(g_CoreMotionStatus);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetStatus"));
	}

	dblPosition = g_CoreMotionStatus->axesStatus[GetAxisMap()].profileTargetPos * m_dblResol;

	return dblPosition;
}

double Cwmx3Axis::ReadSlaveCommandPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->axisControl->GetPosCommand(GetSlaveAxisSlaveID(), &dblPosition);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetPosCommand"));
	}
	dblPosition = dblPosition * m_dblResol;
	return dblPosition;
}

double Cwmx3Axis::ReadMotorPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->axisControl->GetPosFeedback(GetAxisMap(), &dblPosition);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetPosFeedback"));
	}
	dblPosition = dblPosition * m_dblResol;
	return dblPosition;
}

double Cwmx3Axis::ReadSlsaveMotorPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->axisControl->GetPosFeedback(GetSlaveAxisSlaveID(), &dblPosition);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetPosFeedback"));
	}
	dblPosition = dblPosition * m_dblResol;
	return dblPosition;
}

double Cwmx3Axis::ReadMotorActualPosition()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return GetLastCommandPosition() * m_dblResol;
	}
	double dblPosition = 0.0;
	err = GetCoreMotion()->axisControl->GetPosFeedback(GetAxisMap(), &dblPosition);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetPosFeedbackRaw"));
	}
	return dblPosition;
}

double Cwmx3Axis::ReadCommandVelocity()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	double dblVelocity = 0.0;
	err = GetCoreMotion()->axisControl->GetVelCommand(GetAxisMap(), &dblVelocity);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetVelCommand"));
	}
	return dblVelocity;
}

double Cwmx3Axis::ReadMotorVelocity()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return 0.0;
	}
	double dblVelocity = 0.0;
	err = GetCoreMotion()->axisControl->GetVelFeedback(GetAxisMap(), &dblVelocity);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetVelFeedback"));
	}
	return dblVelocity;
}

int Cwmx3Axis::SetPosSet(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->posSetWidth = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetInPos(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->inPosWidth = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetInPos2(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->inPosWidth2 = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetInPos3(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->inPosWidth3 = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetInPos4(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->inPosWidth4 = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetInPos5(double posSet)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->inPosWidth5 = posSet * GetUnResol();
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

int Cwmx3Axis::SetOneDelayedPosSet(double DelayedPosSetWidth, long ms)
{
	int err = ErrorCode::None;
	Config::FeedbackParam* feedParam = new Config::FeedbackParam();
	feedParam->delayedPosSetWidth = DelayedPosSetWidth * GetUnResol();
	feedParam->delayedPosSetMilliseconds = ms;
	err = SetFeedbackParam(feedParam);
	delete feedParam;
	return err;
}

// Memory에서 읽어온다~
WMX3HomeState Cwmx3Axis::GetHomeState()
{
	WMX3HomeState state = WMX3HomeState::Idle;
	if (m_bSimulation)	return state;
	state = gcSlaveMotorStatus->GetWmx3HomeState(GetAxisMap());
	return state;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::CheckAmpAlarm()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	if (GetAxisSkip() == true)
	{
		return false;
	}
	bRet = gcSlaveMotorStatus->GetWmx3AmpAlarm(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::CheckSlaveAmpAlarm()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	if (GetAxisSkip() == true)
	{
		return false;
	}
	bRet = gcSlaveMotorStatus->GetWmx3AmpAlarm(GetSlaveAxisSlaveID());
	return bRet;
}

// Memory에서 읽어온다~
int Cwmx3Axis::CheckAmpAlarmCode()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	err = gcSlaveMotorStatus->GetAmpAlarmCode(GetAxisMap());
	return err;
}

// Memory에서 읽어온다~
int Cwmx3Axis::CheckSlaveAmpAlarmCode()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	err = gcSlaveMotorStatus->GetAmpAlarmCode(GetSlaveAxisSlaveID());
	return err;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsInpos()
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	bRet = gcSlaveMotorStatus->GetWmx3IsInpos(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsPosSet()
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	bRet = gcSlaveMotorStatus->GetWmx3IsPosSet(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsDelayedPosSet()
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	bRet = gcSlaveMotorStatus->GetWmx3IsDelayedPosSet(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::CheckServoOn()
{
	bool bRet = false;
	int err = ErrorCode::None;

	if (m_bSimulation)	return true;

	if (GetAxisSkip() == true)
	{
		return m_LastCommandServoOn;
	}

	err = GetCoreMotion()->GetStatus(g_CoreMotionStatus);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetStatus"));
		return false;
	}

	bRet = g_CoreMotionStatus->axesStatus[GetAxisMap()].servoOn;
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::CheckSlaveServoOn()
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return m_LastCommandServoOn;
	}
	bRet = gcSlaveMotorStatus->GetWmx3CheckServoOn(GetSlaveAxisSlaveID());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsAxisIdle(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	if (gcSlaveMotorStatus->GetWmx3IsAxisIdle(GetAxisMap()) == true)
	{
		bRet = true;
	}
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsSlaveAxisIdle(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	if (gcSlaveMotorStatus->GetWmx3IsAxisIdle(GetSlaveAxisSlaveID()) == true)
	{
		bRet = true;
	}
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsAxisHoming(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	if (gcSlaveMotorStatus->GetWmx3IsAxisHoming(GetAxisMap()) == true)
	{
		bRet = true;
	}
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsAxisHomeDone(void)
{
	bool bRet = false;
	int err = ErrorCode::None;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	//if (GetUseSlaveAxis() == true)
	//{
	//	bRet = IsSyncGroupHomeDone();
	//}
	//else
	{
		bRet = gcSlaveMotorStatus->GetWmx3IsAxisHomeDone(GetAxisMap());
	}
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsAxisMotionComplete(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	bRet = gcSlaveMotorStatus->GetWmx3IsAxisMotionComplete(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsSlaveAxisMotionComplete(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return true;
	}
	bRet = gcSlaveMotorStatus->GetWmx3IsAxisMotionComplete(GetSlaveAxisSlaveID());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsAxisBusy(void)
{
	bool bRet = false;
	if (m_bSimulation)	return true;
	if (GetAxisSkip() == true)
	{
		return false;
	}
	if (gcSlaveMotorStatus->GetWmx3IsAxisBusy(GetAxisMap()) == true)
	{
		bRet = true;
	}
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsNegativeLimitSwitchOn()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	bRet = gcSlaveMotorStatus->GetWmx3IsNegativeLimitSwitchOn(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsPositiveLimitSwitchOn()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	bRet = gcSlaveMotorStatus->GetWmx3IsPositiveLimitSwitchOn(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsNegativeVirtualLimitSwitchOn()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	bRet = gcSlaveMotorStatus->GetWmx3IsNegativeVirtualLimitSwitchOn(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsPositiveVirtualLimitSwitchOn()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	bRet = gcSlaveMotorStatus->GetWmx3IsPositiveVirtualLimitSwitchOn(GetAxisMap());
	return bRet;
}

// Memory에서 읽어온다~
bool Cwmx3Axis::IsHomeSwitchOn()
{
	bool bRet = false;
	if (m_bSimulation)	return false;
	bRet = gcSlaveMotorStatus->GetWmx3IsHomeSwitchOn(GetAxisMap());
	return bRet;
}

bool Cwmx3Axis::IsAxisJog()
{
	bool bRet = false;
	int err = ErrorCode::None;

	if (m_bSimulation)	return true;

	err = GetCoreMotion()->GetStatus(g_CoreMotionStatus);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetStatus"));
		return false;
	}

	if (g_CoreMotionStatus->axesStatus[GetAxisMap()].opState == OperationState::Jog)
	{
		bRet = true;
	}

	return bRet;
}


// Memory에서 읽어온다~
double Cwmx3Axis::ReadActualTorque()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	if (GetAxisSkip() == true)
	{
		return 0.0;
	}
	Value = gcSlaveMotorStatus->GetWmx3ActualTorque(GetAxisMap());
	return Value;
}

// Memory에서 읽어온다~
double Cwmx3Axis::ReadSlaveActualTorque()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	if (GetAxisSkip() == true)
	{
		return 0.0;
	}
	Value = gcSlaveMotorStatus->GetWmx3ActualTorque(GetSlaveAxisSlaveID());
	return Value;
}

// Memory에서 읽어온다~
double Cwmx3Axis::Read1DCompensationData()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	Value = gcSlaveMotorStatus->GetWmx3oneDCompensation(GetAxisMap());
	return Value;
}

// Memory에서 읽어온다~
double Cwmx3Axis::Read2DCompensationData()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	Value = gcSlaveMotorStatus->GetWmx3twoDCompensation(GetAxisMap());
	return Value;
}

WMX3AxisSyncMode Cwmx3Axis::GetAxisSyncMode()
{
	WMX3AxisSyncMode syncMode = WMX3AxisSyncMode::NoSync;
	if (m_bSimulation)	return WMX3AxisSyncMode::NoSync;
	syncMode = gcSlaveMotorStatus->GetAxisSyncMode(GetSlaveAxisSlaveID());
	return syncMode;
}

// Memory에서 읽어온다~
double Cwmx3Axis::GetSyncOffset()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	Value = gcSlaveMotorStatus->GetWmx3SyncOffset(GetSlaveAxisSlaveID());
	return Value;
}

// Memory에서 읽어온다~
double Cwmx3Axis::GetSyncPhaseOffset()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	Value = gcSlaveMotorStatus->GetWmx3SyncPhaseOffset(GetSlaveAxisSlaveID());
	return Value;
}

// Memory에서 읽어온다~
double Cwmx3Axis::GetHomeOffset()
{
	double Value = 0.0f;
	if (m_bSimulation)	return Value;
	Value = gcSlaveMotorStatus->GetWmx3HomeOffset(GetSlaveAxisSlaveID());
	return Value;
}

int Cwmx3Axis::GetEncoderFeedBack()
{
	int Value = 0;
	if (m_bSimulation)	return Value;
	if (GetAxisSkip() == true)
	{
		return Value;
	}
	Value = gcSlaveMotorStatus->GetWmx3EncoderFeedback(GetAxisMap());
	return Value;

}

int Cwmx3Axis::GetSlaveEncoderFeedBack()
{
	int Value = 0;
	if (m_bSimulation)	return Value;
	if (GetAxisSkip() == true)
	{
		return Value;
	}
	Value = gcSlaveMotorStatus->GetWmx3EncoderFeedback(GetSlaveAxisSlaveID());
	return Value;
}

/*
Set the current feedback position of the axis to the specified value.
*/
int Cwmx3Axis::SetMotorPosition(double dblPosition)
{
	int err = ErrorCode::None;
	double Pulse = dblPosition * GetUnResol();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(dblPosition);
		return err;
	}
	Lock();
	err = GetCoreMotion()->home->SetFeedbackPos(GetAxisMap(), Pulse);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetFeedbackPos"));
	}
	return err;
}

/*
Start a blocking wait command, returning only when the axis becomes idle.
*/
int Cwmx3Axis::WaitMotion()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->Wait(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(GetAxisMap(), err, _T("Wait"));
	}
	return err;
}

/*
Pause the execution of a position command or interpolation command for an axis.
*/
int Cwmx3Axis::PauseMotion()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->Pause(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Pause"));
	}
	return err;
}

/*
Resume the execution of a paused position command or interpolation command for an axis.
*/
int Cwmx3Axis::ResumeMotion()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->Resume(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Resume"));
	}
	return err;
}

/*
Override the target position of an axis currently executing a position command.The target position is specified as an absolute position.
*/
int Cwmx3Axis::OverridePosition(double dblPosition, WMX3_AXIS_POSCOMMANDPROFILE profile)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	double dPulse = m_dblUnResol * dblPosition;
	Motion::PosCommand posCommand = Motion::PosCommand();
	memcpy_s(&posCommand.profile, sizeof(posCommand.profile), &profile, sizeof(profile));
	posCommand.axis = GetAxisMap();
	posCommand.target = dPulse;	
	Lock();
	err = GetCoreMotion()->motion->OverridePos(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("OverridePos"));
	}
	return err;
}

/*
Override the velocity of an axis currently executing a position, jog, or velocity command.
*/
int Cwmx3Axis::OverrideVelocity(double dblVelocity, WMX3_AXIS_POSCOMMANDPROFILE profile)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	memcpy_s(&posCommand.profile, sizeof(posCommand.profile), &profile, sizeof(profile));
	posCommand.profile.velocity = dblVelocity;
	posCommand.axis = GetAxisMap();
	Lock();
	err = GetCoreMotion()->motion->OverrideVel(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("OverrideVel"));
	}
	return err;
}

/*
Override the entire profile of an axis currently executing a position, jog, or velocity command.
*/
int Cwmx3Axis::OverrideProfile(WMX3_AXIS_POSCOMMANDPROFILE profile)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	memcpy_s(&posCommand.profile, sizeof(posCommand.profile), &profile, sizeof(profile));
	posCommand.axis = GetAxisMap();
	Lock();
	err = GetCoreMotion()->motion->OverrideProfile(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("OverrideProfile"));
	}
	return err;
}

int Cwmx3Axis::StartJog(Motion::JogCommand* jogCmd)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::JogCommand jogCommand = Motion::JogCommand();
	jogCommand = *jogCmd;
	jogCommand.axis = GetAxisMap();
	jogCommand.profile.velocity = GetMovingDir() * jogCommand.profile.velocity;
	Lock();
	err = GetCoreMotion()->motion->StartJog(&jogCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartJog with profile"));
	}
	return err;
}

/*
Stop the motion of an axis.
*/
int Cwmx3Axis::StopMotion()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->Stop(GetAxisMap());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Stop"));
	}
	return err;
}

/*
Stop the motion of an axis using the specified deceleration and a trapezoidal profile.
*/
int Cwmx3Axis::StopMotion(double deceleration)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisSkip() == true)
	{
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->Stop(GetAxisMap(), deceleration);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("Stop with deceleration"));
	}
	return err;
}

/*
Start a relative position motion command for a single axis.
*/
int Cwmx3Axis::StartMove(Motion::PosCommand* moveCmd, double position)
{
	unsigned err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	double dPulse = m_dblUnResol * position;
	posCommand = GetProfileByRatio(moveCmd);
	posCommand.target = dPulse * GetMovingDir();
	posCommand.axis = GetAxisMap();

	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}

	Lock();
	err = GetCoreMotion()->motion->StartMov(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartMov with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartMove AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

/*
Start a relative position motion command for a single axis.
*/
int Cwmx3Axis::StartMove(Motion::PosCommand* moveCmd)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	posCommand = GetProfileByRatio(moveCmd);
	posCommand.target = posCommand.target * GetMovingDir();
	posCommand.axis = GetAxisMap();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->StartMov(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartMov with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartMove AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

int Cwmx3Axis::StartSlaveMove(Motion::PosCommand* moveCmd, double position)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	double dPulse = m_dblUnResol * position;
	SetRatio(0.1);
	posCommand = GetProfileByRatio(moveCmd);
	posCommand.target = dPulse * GetMovingDir();
	posCommand.axis = GetSlaveAxisSlaveID();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->StartMov(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartMov with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartMove AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

/*
Start an absolute position motion command for a single axis.
*/
int Cwmx3Axis::StartPos(Motion::PosCommand* posCmd, double position)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	double dPulse = m_dblUnResol * position;
	Motion::PosCommand posCommand = Motion::PosCommand();
	posCommand = GetProfileByRatio(posCmd);
	posCommand.target = dPulse * GetMovingDir();
	posCommand.axis = GetAxisMap();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}

	if (abs(ReadProfileTargetPosition() - position) > 0.0001)
	{
		TRACE(_T("[PWR] MotionCommand StartPos Start (%s %.3f -> %.3f %.1f)\n"), GetAxisName(), ReadProfileTargetPosition(), position, GetRatio() * 100.0);
	}

	Lock();
	err = GetCoreMotion()->motion->StartPos(&posCommand);
	Unlock();

	if (err == ErrorCode::StartingPreviousCommand)
	{
		TRACE(_T("[PWR] StartPos AxisIndex(%d) StartingPreviousCommand Retry Start\n"), GetAxisIndex());

		if (WaitIdle() == true)
		{
			Lock();
			err = GetCoreMotion()->motion->StartPos(&posCommand);
			Unlock();
		}
		else
		{
			TRACE(_T("[PWR] StartPos AxisIndex(%d) WaitIdle TimeOut\n"), GetAxisIndex());
			Lock();
			err = GetCoreMotion()->motion->StartPos(&posCommand);
			Unlock();
		}
	}

	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartPos with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartPos AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

/*
Start an absolute position motion command for a single axis.
*/
int Cwmx3Axis::StartPos(Motion::PosCommand* posCmd)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Motion::PosCommand posCommand = Motion::PosCommand();
	posCommand = GetProfileByRatio(posCmd);
	posCommand.target = posCommand.target * GetMovingDir();
	posCommand.axis = GetAxisMap();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->StartPos(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartPos with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartPos AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

/*
Start an absolute position motion command for a single axis.
*/
int Cwmx3Axis::StartSlavePos(Motion::PosCommand* posCmd, double position)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	double dPulse = m_dblUnResol * position;
	Motion::PosCommand posCommand = Motion::PosCommand();
	posCommand = GetProfileByRatio(posCmd);
	posCommand.target = dPulse * GetMovingDir();
	posCommand.axis = GetSlaveAxisSlaveID();
	if (GetAxisSkip() == true)
	{
		SetLastCommandPosition(posCommand.target);
		return err;
	}
	Lock();
	err = GetCoreMotion()->motion->StartPos(&posCommand);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartPos with profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartPos AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

/*
Start an absolute position motion command for a single axis.
*/
int Cwmx3Axis::StartPosWithTriggerPos(Motion::PosCommand* posCmd, double position, double TriggerPos)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	double dPulse = m_dblUnResol * position;
	double dTriggerPulse = m_dblUnResol * TriggerPos;
	Motion::PosCommand posCommand = Motion::PosCommand();
	Motion::TriggerPosCommand triggerCommand = Motion::TriggerPosCommand();

	posCommand = GetProfileByRatio(posCmd);
	posCommand.target = dPulse * GetMovingDir();
	posCommand.axis = GetAxisMap();

	triggerCommand = GetProfileByRatioTrigger(posCmd);
	triggerCommand.profile.velocity = posCommand.profile.velocity * Get2ndRatio();
	triggerCommand.profile.acc = posCommand.profile.acc * 1.6;
	triggerCommand.profile.dec = posCommand.profile.dec * 1.6;
	triggerCommand.target = dPulse * GetMovingDir();
	triggerCommand.axis = GetAxisMap();

	triggerCommand.trigger.triggerAxis = GetAxisMap();
	triggerCommand.trigger.triggerType = TriggerType::RemainingDistance;
	triggerCommand.trigger.triggerValue = dTriggerPulse * GetMovingDir();

	TRACE(_T("[PWR] StartPosWithTriggerPos Vel:%.1f Trigger Vel:%.1f\n"), posCommand.profile.velocity, triggerCommand.profile.velocity);
	TRACE(_T("[PWR] StartPosWithTriggerPos Acc&Dec:%.1f Trigger Acc&Dec:%.1f\n"), posCommand.profile.acc, triggerCommand.profile.acc);
	TRACE(_T("[PWR] StartPosWithTriggerPos Target:%.1f Trigger Target:%.1f Value:%.1f\n"), posCommand.target, triggerCommand.target, triggerCommand.trigger.triggerValue);

	Lock();
	err = GetCoreMotion()->motion->StartPos(&posCommand);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartPosWithTriggerPos with profile"));
	}
	err = GetCoreMotion()->motion->StartPos(&triggerCommand);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("StartPosWithTriggerPos with trigger"));
	}
	Unlock();
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartPosWithTriggerPos AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return err;
}

int Cwmx3Axis::StartavsMotionX(double targetPos, double Velocity)
{
	AdvMotion::PVTCommand pvtCmd;
	double currentPos = 0;
	double velRatio = 0;
	double movingDist = 0;
	double constDist = 0;
	double constT = 0;
	int Err = ErrorCode::None;
	double dPulse = m_dblUnResol * targetPos;
	TRACE(_T("[PWR] StartavsMotionX Cur:%.1f Pulse:%.1f Velocity:%.1f"), currentPos, dPulse, Velocity);
	Err = GetCoreMotion()->axisControl->GetPosCommand(GetAxisMap(), &currentPos);
	if (Err != ErrorCode::None)
	{
		TRACE(_T("[PWR] StartavsMotionX Cur:%.1f Pulse:%.1f Velocity:%.1f Err:%d"), currentPos, dPulse, Velocity, Err);
		return Err;
	}	
	movingDist = dPulse - currentPos;
	velRatio = Velocity / 1100000.00000;
	constDist = abs(movingDist) - 130890.59772 * velRatio;
	if (constDist <= 0)
	{
		velRatio = abs(movingDist) / 0.11899 / 1100000.00000 * 0.9;
		Velocity = velRatio * 1100000.00000;
	}
	constDist = abs(movingDist) - 130890.59772 * velRatio;
	constT = constDist / abs(Velocity) * 1000;
	pvtCmd.axis = GetAxisMap();
	pvtCmd.pointCount = 8;
	pvtCmd.points[0].timeMilliseconds = 0.00000;
	pvtCmd.points[1].timeMilliseconds = 32.64922;
	pvtCmd.points[2].timeMilliseconds = 86.34223;
	pvtCmd.points[3].timeMilliseconds = 118.99145;
	pvtCmd.points[4].timeMilliseconds = constT + 118.99145;
	pvtCmd.points[5].timeMilliseconds = constT + 151.64068;
	pvtCmd.points[6].timeMilliseconds = constT + 205.33368;
	pvtCmd.points[7].timeMilliseconds = constT + 237.98290;
	pvtCmd.points[0].pos = currentPos + 0.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[1].pos = currentPos + 3395.12012 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[2].pos = currentPos + 32926.27338 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[3].pos = currentPos + 65445.29886 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[4].pos = currentPos + (constDist + 65445.29886 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[5].pos = currentPos + (constDist + 97964.32434 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[6].pos = currentPos + (constDist + 127495.47760 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[7].pos = currentPos + (constDist + 130890.59772 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[0].velocity = 0.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[1].velocity = 207975.55226 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[2].velocity = 892024.44774 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[3].velocity = 1100000.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[4].velocity = 1100000.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[5].velocity = 892024.44774 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[6].velocity = 207975.55226 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[7].velocity = 0.00000 * velRatio * (abs(movingDist) / movingDist);
	Err = GetAdvancedMotion()->advMotion->StartPVT(&pvtCmd);
	TRACE(_T("[PWR] StartavsMotionX Cur:%.1f Pulse:%.1f Velocity:%.1f StartPVT Err:%d"), currentPos, dPulse, Velocity, Err);
	if (Err != ErrorCode::None)
	{
		AdvancedMotionErrorToString(Err, _T("StartavsMotionX with private profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartavsMotionX AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return Err;
}

int Cwmx3Axis::StartavsMotionY(double targetPos, double Velocity)
{
	AdvMotion::PVTCommand pvtCmd;
	double currentPos = 0;
	double velRatio = 0;
	double movingDist = 0;
	double constDist = 0;
	double constT = 0;
	int Err = ErrorCode::None;
	double dPulse = m_dblUnResol * targetPos;
	Err = GetCoreMotion()->axisControl->GetPosCommand(GetAxisMap(), &currentPos);
	TRACE(_T("[PWR] StartavsMotionX Cur:%.1f Pulse:%.1f Velocity:%.1f"), currentPos, dPulse, Velocity);
	if (Err != ErrorCode::None)
	{
		TRACE(_T("[PWR] StartavsMotionY Cur:%.1f Pulse:%.1f Velocity:%.1f Err:%d"), currentPos, dPulse, Velocity, Err);
		return Err;
	}	
	movingDist = dPulse - currentPos;
	velRatio = Velocity / 1100000.00000;
	constDist = abs(movingDist) - 114176.94364 * velRatio;
	if (constDist <= 0)
	{
		velRatio = abs(movingDist) / 0.10380 / 1100000.00000 * 0.9;
		Velocity = velRatio * 1100000.00000;
	}
	constDist = abs(movingDist) - 114176.94364 * velRatio;
	constT = constDist / abs(Velocity) * 1000;
	pvtCmd.axis = GetAxisMap();
	pvtCmd.pointCount = 8;
	pvtCmd.points[0].timeMilliseconds = 0.00000;
	pvtCmd.points[1].timeMilliseconds = 17.45499;
	pvtCmd.points[2].timeMilliseconds = 86.34223;
	pvtCmd.points[3].timeMilliseconds = 103.79722;
	pvtCmd.points[4].timeMilliseconds = constT + 103.79722;
	pvtCmd.points[5].timeMilliseconds = constT + 121.25221;
	pvtCmd.points[6].timeMilliseconds = constT + 190.13945;
	pvtCmd.points[7].timeMilliseconds = constT + 207.59444;
	pvtCmd.points[0].pos = currentPos + 0.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[1].pos = currentPos + 970.39547 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[2].pos = currentPos + 38858.37576 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[3].pos = currentPos + 57088.47182 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[4].pos = currentPos + (constDist + 57088.47182 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[5].pos = currentPos + (constDist + 75318.56788 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[6].pos = currentPos + (constDist + 113206.54817 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[7].pos = currentPos + (constDist + 114176.94364 * velRatio) * (abs(movingDist) / movingDist);
	pvtCmd.points[0].velocity = 0.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[1].velocity = 111188.30097 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[2].velocity = 988811.69903 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[3].velocity = 1100000.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[4].velocity = 1100000.00000 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[5].velocity = 988811.69903 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[6].velocity = 111188.30097 * velRatio * (abs(movingDist) / movingDist);
	pvtCmd.points[7].velocity = 0.00000 * velRatio * (abs(movingDist) / movingDist);
	Err = GetAdvancedMotion()->advMotion->StartPVT(&pvtCmd);
	TRACE(_T("[PWR] StartavsMotionY Cur:%.1f Pulse:%.1f Velocity:%.1f StartPVT Err:%d"), currentPos, dPulse, Velocity, Err);
	if (Err != ErrorCode::None)
	{
		AdvancedMotionErrorToString(Err, _T("StartavsMotionY with private profile"));
	}
	if (IsGantryAxis() == true)
	{
		//if (gcPowerLog->IsShowMotionLockLog() == true)
		//{
		//	TRACE(_T("[PWR] StartavsMotionY AxisIndex(%d) Unlock\n"), GetAxisIndex());
		//}
		//SEM_UNLOCK(gMOTION_LOCK[GetAxisIndex()]);
	}
	return Err;
}


/*
Establish synchronous control between a master axis and a slave axis.
*/
int Cwmx3Axis::SetSyncMasterSlave()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	if (GetAxisMap() == GetSlaveAxisSlaveID())
		return err;
	Lock();
	err = GetCoreMotion()->sync->SetSyncMasterSlave(GetAxisMap(), GetSlaveAxisSlaveID());
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetSyncMasterSlave"));
	}
	return err;
}

int Cwmx3Axis::Set1DCompensationData(PitchErrorCompensationData* pitchErrCompData)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCompensation()->SetPitchErrorCompensation(GetAxisMap(), pitchErrCompData);
	Unlock();
	if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsEnabled)
	{
		CompensationErrorToString(err, _T("SetPitchErrorCompensation"));
	}
	return err;
}

int Cwmx3Axis::Enable1D()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCompensation()->EnablePitchErrorCompensation(GetAxisMap());
	Unlock();
	if (err != CompensationErrorCode::PitchErrorCompensationIsDisabled && err != CompensationErrorCode::PitchErrorCompensationIsEnabled)
	{
		CompensationErrorToString(err, _T("EnablePitchErrorCompensation"));
	}
	return err;
}

int Cwmx3Axis::Disable1D()
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCompensation()->DisablePitchErrorCompensation(GetAxisMap());
	Unlock();
	if (err != CompensationErrorCode::PitchErrorCompensationIsDisabled && err != CompensationErrorCode::PitchErrorCompensationIsEnabled)
	{
		CompensationErrorToString(err, _T("DisablePitchErrorCompensation"));
	}
	return err;
}

bool Cwmx3Axis::WaitIdle()
{
	ULONGLONG GetTime = _time_get();

	while (1)
	{
		if (IsAxisIdle() == true && IsAxisMotionComplete() == true)
		{
			return true;
		}
		else if (_time_elapsed(GetTime) > TIME1000MS)
		{
			return false;
		}
		ThreadSleep(TIME1MS);
	}

	return true;
}


Profile Cwmx3Axis::GetTeachBoxMoveProfile()
{
	return m_TeachMoveProfile;
}	

void Cwmx3Axis::SetTeachBoxMoveProfile(Profile profile)
{
	m_TeachMoveProfile = profile;
}

Profile Cwmx3Axis::GetTeachBoxJogProfile()
{
	return m_TeachJogProfile;
}

void Cwmx3Axis::SetTeachBoxJogProfile(Profile profile)
{
	m_TeachJogProfile = profile;
}

long Cwmx3Axis::GetShortDistanceLevel(double Dist)
{
	for (int inx = 0; inx < MAX_SHORT_DIST_LEVEL; ++inx)
	{
		if (abs(Dist) < GetShortDist(inx))
		{
			return inx;
		}
	}
	return MAX_SHORT_DIST_LEVEL;
}

Profile Cwmx3Axis::GetMoveProfile()
{
	return m_MoveProfile;
}

Profile Cwmx3Axis::GetMoveProfile(double Dist)
{
	Profile MoveProfile;
	Lock();
	MoveProfile = m_MoveProfile;
	if (IsGantryAxis() == true)
	{
		long ShortDistLvl = MAX_SHORT_DIST_LEVEL;
		ShortDistLvl = GetShortDistanceLevel(Dist);
		if (ShortDistLvl < MAX_SHORT_DIST_LEVEL)
		{
			MoveProfile.velocity = GetShortDistVel(ShortDistLvl);
			MoveProfile.acc = GetShortDistAccDec(ShortDistLvl);
			MoveProfile.dec = GetShortDistAccDec(ShortDistLvl);
			if (gcPowerLog->IsShowShortDistLog() == true)
			{
				TRACE(_T("[PWR] %s ShortDist:%.3f Level:%d Vel:%.3f AccDec:%.3f\n"), GetAxisName(), Dist, ShortDistLvl, MoveProfile.velocity, MoveProfile.acc);
			}
		}
	}
	Unlock();
	return MoveProfile;
}

void Cwmx3Axis::SetShortDist(long index, double ShortDist)
{
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		if (abs(ShortDist) > MAX_MOTION_VALID_RANGE)
		{
			TRACE(_T("[PWR] SetShortDist index:%d ShortDist:%.3f"), index, ShortDist);
		}
		m_ShortDist[index] = ShortDist;
	}
}

void Cwmx3Axis::SetShortDistVel(long index, double ShortVel)
{
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		m_ShortDistVel[index] = ShortVel;
	}
}

void Cwmx3Axis::SetShortDistAccDec(long index, double ShortAccDec)
{
	if (index < MAX_SHORT_DIST_LEVEL)
	{
		m_ShortDistAccDec[index] = ShortAccDec;
	}
}

double Cwmx3Axis::GetShortDist(long index)
{
	ASSERT(index < MAX_SHORT_DIST_LEVEL);
	return m_ShortDist[index];
}

double Cwmx3Axis::GetShortDistVel(long index)
{
	ASSERT(index < MAX_SHORT_DIST_LEVEL);
	return m_ShortDistVel[index];
}

double Cwmx3Axis::GetShortDistAccDec(long index)
{
	ASSERT(index < MAX_SHORT_DIST_LEVEL);
	return m_ShortDistAccDec[index];
}

void Cwmx3Axis::SetMoveProfile(Profile profile)
{
	m_MoveProfile = profile;
}

Motion::PosCommand Cwmx3Axis::GetProfileByRatio(Motion::PosCommand* Cmd)
{
	Motion::PosCommand NewCmd = Motion::PosCommand();
	Lock();
	NewCmd = *Cmd;
	NewCmd.profile.velocity = Cmd->profile.velocity * GetRatio();
	NewCmd.profile.acc = Cmd->profile.acc * GetRatio();
	NewCmd.profile.dec = Cmd->profile.dec * GetRatio();
	if (gcPowerLog->IsShowShortDistLog() == true)
	{
		TRACE(_T("[PWR] Axis(%d) Ratio:%.1f%%\n"), GetAxisIndex(), GetRatio());
		TRACE(_T("[PWR] Axis(%d) Old Vel:%.3f Acc:%.3f Dec:%.3f\n"), GetAxisIndex(), Cmd->profile.velocity, Cmd->profile.acc, Cmd->profile.dec);
		TRACE(_T("[PWR] Axis(%d) New Vel:%.3f Acc:%.3f Dec:%.3f\n"), GetAxisIndex(), NewCmd.profile.velocity, NewCmd.profile.acc, NewCmd.profile.dec);
	}
	Unlock();
	return NewCmd;
}

Motion::TriggerPosCommand Cwmx3Axis::GetProfileByRatioTrigger(Motion::PosCommand* Cmd)
{
	Motion::TriggerPosCommand NewCmd = Motion::TriggerPosCommand();
	Lock();
	//NewCmd = *Cmd;
	NewCmd.profile.type = Cmd->profile.type;
	NewCmd.profile.jerkAcc = Cmd->profile.jerkAcc;
	NewCmd.profile.jerkDec = Cmd->profile.jerkDec;
	NewCmd.profile.jerkAccRatio = Cmd->profile.jerkAccRatio;
	NewCmd.profile.jerkDecRatio = Cmd->profile.jerkDecRatio;
	NewCmd.profile.accTimeMilliseconds = Cmd->profile.accTimeMilliseconds;
	NewCmd.profile.decTimeMilliseconds = Cmd->profile.decTimeMilliseconds;
	NewCmd.profile.velocity = Cmd->profile.velocity * Get2ndRatio();
	NewCmd.profile.acc = Cmd->profile.acc * GetRatio();
	NewCmd.profile.dec = Cmd->profile.dec * GetRatio();
	//if (gcPowerLog->IsShowShortDistLog() == true)
	{
		TRACE(_T("[PWR] Axis(%d) Ratio:%.1f%% 2ndRatio:%.1f%%\n"), GetAxisIndex(), GetRatio(), Get2ndRatio());
		TRACE(_T("[PWR] Axis(%d) Old Vel:%.3f Acc:%.3f Dec:%.3f\n"), GetAxisIndex(), Cmd->profile.velocity, Cmd->profile.acc, Cmd->profile.dec);
		TRACE(_T("[PWR] Axis(%d) New Vel:%.3f Acc:%.3f Dec:%.3f\n"), GetAxisIndex(), NewCmd.profile.velocity, NewCmd.profile.acc, NewCmd.profile.dec);
	}
	Unlock();
	return NewCmd;
}

int Cwmx3Axis::GetMotionParam(Config::MotionParam* pMotionParam)
{
	int err = ErrorCode::None;
	Config::MotionParam motionParam;
	if (m_bSimulation)	return err;
	Lock();	
	err = GetCoreMotion()->config->GetMotionParam(GetAxisMap(), &motionParam);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetMotionParam"));
	}
	*pMotionParam = motionParam;
	return err;
}

int Cwmx3Axis::SetMotionParam(Config::MotionParam* pMotionParam)
{
	int err = ErrorCode::None;
	if (m_bSimulation)	return err;
	Config::MotionParam motionParam;
	motionParam = *pMotionParam;
	Lock();
	err = GetCoreMotion()->config->SetMotionParam(GetAxisMap(), &motionParam);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetMotionParam"));
	}
	return err;
}

void Cwmx3Axis::SetLinearIntplProfileCalcMode(long CalMode, bool apiWaitUntilMotionStart, double quickStopDec)
{
	int err = ErrorCode::None;
	Config::MotionParam* motionParam = new Config::MotionParam();
	err = GetMotionParam(motionParam);
	if(CalMode == 1)
		motionParam->linearIntplProfileCalcMode = Config::LinearIntplProfileCalcMode::MatchSlowestAxis;
	else if(CalMode == 2)
		motionParam->linearIntplProfileCalcMode = Config::LinearIntplProfileCalcMode::MatchFarthestAxis;
	else
		motionParam->linearIntplProfileCalcMode = Config::LinearIntplProfileCalcMode::AxisLimit;
	motionParam->apiWaitUntilMotionStart = false;
	motionParam->linearIntplOverrideType = Config::LinearIntplOverrideType::FastBlending;
	motionParam->quickStopDec = quickStopDec;
	err = SetMotionParam(motionParam);
	delete motionParam;
}

bool Cwmx3Axis::IsSyncGroupHomeDone(void)
{
	bool bRet = false;
	int err = ErrorCode::None;
	if (m_bSimulation)	return true;
	Sync::SyncGroupStatus* pSyncGroupStatus = new Sync::SyncGroupStatus();
	err = GetCoreMotion()->sync->GetSyncGroupStatus(GetAxisMap(), pSyncGroupStatus);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetSyncGroupStatus"));
		bRet = false;
	}
	else
	{
		if (pSyncGroupStatus->homeDone == 1)
		{
			bRet = true;
		}
		else
		{
			bRet = false;
		}
	}
	delete pSyncGroupStatus;
	return bRet;
}

double Cwmx3Axis::GetMaxTrqLimit(void)
{
	int err = ErrorCode::None;
	double Torque = 0.0;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->torque->GetMaxTrqLimit(GetAxisMap(), &Torque);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetMaxTrqLimit"));
	}
	return Torque;
}

long Cwmx3Axis::SetMaxTrqLimit(double Torque)
{
	int err = ErrorCode::None;
	double SetTorque = Torque;
	if (m_bSimulation)	return err;
	Lock();
	err = GetCoreMotion()->torque->SetMaxTrqLimit(GetAxisMap(), Torque);
	Unlock();
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("SetMaxTrqLimit"));
	}
	return err;
}

long Cwmx3Axis::StartWmx3Monitor(long BoardCount, long BlockNo, long InsertNo, CString Action)
{
	int err = ErrorCode::None;
	long CreateDir = false;
	CoreMotionLogInput in;
	LogChannelOptions opt;
	LogFilePathW path;
	CString strFileName;
	unsigned int chnl = 0;
	wchar_t fileName[256];
	wchar_t Time[256];
	wchar_t year[256], date[256];
	ZeroMemory(&fileName, sizeof(fileName));
	ZeroMemory(&Time, sizeof(Time));
	ZeroMemory(&year, sizeof(year));
	ZeroMemory(&date, sizeof(date));

	GetYear(year);
	GetDate(date);
	//Set the log output file path
	swprintf_s(path.dirPath, _T("D:\\PowerMotion\\TorqueZ\\%s\\%s\\"), year, date);
	TRACE(_T("[PWR] _CreateDirectory:%s\n"), path.dirPath);
	CreateDir = _CreateDirectory(path.dirPath);

	//Set the CoreMotion module to log the position command data of axis 0
	in.axisSelection.axisCount = 1;
	in.axisSelection.axis[0] = GetAxisMap();
	in.axisOptions.feedbackPos = 1;
	in.axisOptions.feedbackVelocity = 1;
	in.axisOptions.feedbackTrq = 1;
	err = GetWmx3Log()->SetLog(chnl, &in);
	//TRACE(_T("[PWR] SetLog Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("SetLog"));
		//return err;
	}
	//Set the log options
	opt.samplingTimeMilliseconds = 0;
	opt.stopLoggingOnBufferOverflow = 1;
	opt.precision = 1;
	err = GetWmx3Log()->SetLogOption(chnl, &opt);
	//TRACE(_T("[PWR] SetLogOption Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("SetLogOption"));
		//return err;
	}
	GetDateTimeMil(Time);
	swprintf_s(path.fileName, _T("%s_Board%d_Block%d_Idx%d_%s_%s.Monitor"), Time, BoardCount, BlockNo, InsertNo, (LPCTSTR)Action, GetAxisName().GetBuffer());	TRACE(_T("[PWR] SetTorqueMonitorFileName:%s\n"), path.fileName);
	strFileName.Format(_T("%s%s"), path.dirPath, path.fileName);
	SetTorqueMonitorFileName(strFileName);
	err = GetWmx3Log()->SetLogFilePath(chnl, &path);
	//TRACE(_T("[PWR] SetLogFilePath Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("SetLogFilePath"));
		return err;
	}
	//Start Log operation
	err = GetWmx3Log()->StartLog(chnl);
	//TRACE(_T("[PWR] StartLog Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("StartLog"));
		return err;
	}
	return err;
}

long Cwmx3Axis::SetTorqueMonitorFileName(CString strFileName)
{
	int err = ErrorCode::None;
	m_TorqueMonitorFileName.Format(_T("%s"), (LPCTSTR)strFileName);
	return err;
}

CString Cwmx3Axis::GetTorqueMonitorFileName()
{
	int err = ErrorCode::None;
	CString strFileName;
	strFileName.Format(_T("%s"), (LPCTSTR)m_TorqueMonitorFileName);
	return strFileName;
}

long Cwmx3Axis::StopWmx3Monitor()
{
	int err = ErrorCode::None;
	unsigned int chnl = 0;
	//When the event or error occurs, stop logging data
	err = GetWmx3Log()->StopLog(chnl);
	//TRACE(_T("[PWR] StopLog Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("StopLog"));
	}
	return err;
}


bool Cwmx3Axis::WaitStopWmx3Monitor(long timeOut)
{
	ULONGLONG time = _time_get();
	int err = ErrorCode::None;
	unsigned int chnl = 0;
	DetailLogStatus pStatus;
	ZeroMemory(&pStatus, sizeof(pStatus));
	bool waitfail = false;

	while (1)
	{
		err = GetWmx3Log()->GetDetailLogStatus(chnl, &pStatus);

		if (err == ErrorCode::None)
		{
			if (pStatus.state == DetailLogState::Idle || pStatus.state == DetailLogState::Finished)
			{
				return true;
			}
		}

		if (timeOut < _time_elapsed(time))
		{
			return false;
		}

		ThreadSleep(TIME1MS);
	}

	return false;
}

long Cwmx3Axis::ResetWmx3Monitor()
{
	int err = ErrorCode::None;
	unsigned int chnl = 0;
	//When the event or error occurs, stop logging data
	err = GetWmx3Log()->ResetLog(chnl);
	//TRACE(_T("[PWR] StopLog Err:%d\n"), err);
	if (err != ErrorCode::None)
	{
		LogErrorToString(err, _T("ResetLog"));
	}
	return err;
}

long Cwmx3Axis::GetDateTime(wchar_t* Time)
{
	struct tm newtime;
	__time64_t long_time;
	errno_t err;
	// Get time as 64-bit integer.
	_time64(&long_time);
	// Convert to local time.
	err = _localtime64_s(&newtime, &long_time);
	if (err)
	{
		TRACE(_T("[PWR] Invalid argument to _localtime64_s. err:%d\n"), err);
	}
	swprintf_s(Time, 256, _T("%02d%02d%02d"), newtime.tm_hour, newtime.tm_min, newtime.tm_sec);
	return err;
}

long Cwmx3Axis::GetDateTimeMil(wchar_t* Time)
{
	SYSTEMTIME cur_time;
	//CString strTime;

	GetLocalTime(&cur_time);
	//strTime.Format(_T("%02d%02d%02d%03d"), (int)cur_time.wHour, (int)cur_time.wMinute, (int)cur_time.wSecond, (int)cur_time.wMilliseconds);

	swprintf_s(Time, 256, _T("%02d%02d%02d%03d"), (int)cur_time.wHour, (int)cur_time.wMinute, (int)cur_time.wSecond, (int)cur_time.wMilliseconds);

	return 0;
}

long Cwmx3Axis::GetYear(wchar_t* year)
{
	struct tm newtime;
	__time64_t long_time;
	errno_t err;
	// Get time as 64-bit integer.
	_time64(&long_time);
	// Convert to local time.
	err = _localtime64_s(&newtime, &long_time);
	if (err)
	{
		TRACE(_T("[PWR] Invalid argument to _localtime64_s. err:%d\n"), err);
	}
	swprintf_s(year, 256, _T("%04d"), newtime.tm_year + 1900);
	return err;
}


long Cwmx3Axis::GetDate(wchar_t* date)
{
	struct tm newtime;
	__time64_t long_time;
	errno_t err;
	// Get time as 64-bit integer.
	_time64(&long_time);
	// Convert to local time.
	err = _localtime64_s(&newtime, &long_time);
	if (err)
	{
		TRACE(_T("[PWR] Invalid argument to _localtime64_s. err:%d\n"), err);
	}
	swprintf_s(date, 256, _T("%02d%02d"), newtime.tm_mon + 1, newtime.tm_mday);
	return err;
}

bool Cwmx3Axis::_CreateDirectory(LPCWSTR lpszPath)
{
	wchar_t szPathBuffer[MAX_PATH];
	size_t len = wcslen(lpszPath);
	for (size_t i = 0; i < len; i++)
	{
		szPathBuffer[i] = *(lpszPath + i);
		if (szPathBuffer[i] == _T('\\') || szPathBuffer[i] == _T('/'))
		{
			szPathBuffer[i + 1] = NULL;
			if (!PathFileExists(szPathBuffer))
			{
				if (!::CreateDirectory(szPathBuffer, NULL))
				{
					if (GetLastError() != ERROR_ALREADY_EXISTS)
						return false;
				}
			}
		}
	}
	return true;
}

void Cwmx3Axis::SetHomingMaxTimeOut(ULONGLONG MaxTimeOut)
{
	m_HomingMaxTimeOut = MaxTimeOut;
}

ULONGLONG Cwmx3Axis::GetHomingMaxTimeOut()
{
	return m_HomingMaxTimeOut;
}

void Cwmx3Axis::SetInitializeFail(bool bFail)
{
	m_bErrorStatus = bFail;
}

bool Cwmx3Axis::GetInitializeFail()
{
	return m_bErrorStatus;
}

bool Cwmx3Axis::GetCoreMotionAxisStatus(CoreMotionAxisStatus* status)
{
	bool bRet = false;
	int err = ErrorCode::None;

	//if (m_bSimulation)	return false;

	err = GetCoreMotion()->GetStatus(g_CoreMotionStatus);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("GetStatus"));
		return false;
	}

	*status = g_CoreMotionStatus->axesStatus[GetAxisMap()];
	return true;
}
