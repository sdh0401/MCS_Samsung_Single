#include "pch.h"
#include "Cwmx3LinearIntplPos.h"

#include "Cwmx3Init.h"
#include "GlobalDefine.h"
#include "AxisInformation.h"
#include "Wmx3MotorDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CApplicationTime.h"
#include "DefineThreadLoopTime.h"
#include "CSlaveMotorStatus.h"
//#include "ErrorCode.h"
#include "CThreadException.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorData.h"
#include "CPowerLog.h"

#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <AdvancedMotionApi.h>
#include <ecApi.h>
#include <IOApi.h>
#include <mutex>

using namespace wmx3Api;
using namespace std;

Cwmx3LinearIntplPos* gcWmx3LinearIntplPos;
Cwmx3LinearIntplPos::Cwmx3LinearIntplPos()
{
}

Cwmx3LinearIntplPos::~Cwmx3LinearIntplPos()
{
}

/*
A linear interpolation command interpolates the cyclic position commands of one or more axes to move in a straight line in 2D, 3D, or other dimensional space.
There is no limit to the number of axes that may be commanded by a single linear interpolation command.
*/
long Cwmx3LinearIntplPos::StartLinearIntplPos()
{
	long Err = ErrorCode::None;
	unsigned int LinearIntpCount = 0;
	LinearIntpCount = (unsigned int)GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartLinearIntplPos Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	Motion::LinearIntplCommand linearIntplCommand = Motion::LinearIntplCommand();
	StartPosStruct* pStartPos;
	Cwmx3Axis* pAxis;
	CString strAxis;
	linearIntplCommand.axisCount = LinearIntpCount;
	Config::MotionParam motionParam = Config::MotionParam();
	Motion::PosCommand posCommand = Motion::PosCommand();
	Motion::PosCommand retPosCommand = Motion::PosCommand();
	double Dist = 0.0;
	long linearIntplProfileCalcMode = 0;
	long MoveAxisCnt = 0, AxisIndex = 0;
	WMX3AxisSyncMode syncMode;
	for (INT_PTR index = 0; index < GetWmx3LinearIntpAxisCount(); ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pAxis = GetWmx3AxisByName(strAxis);
		if (pAxis == NULL)
		{
			TRACE(_T("[PWR] StartLinearIntplPos(%s) is NULL\n"), strAxis);
			continue;
		}
		Dist = pStartPos->Distance;
		linearIntplProfileCalcMode = pStartPos->linearIntplProfileCalcMode;
		posCommand.profile = pAxis->GetMoveProfile(Dist);
		linearIntplCommand.axis[index] = pAxis->GetAxisMap();
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			if (syncMode == WMX3AxisSyncMode::NoSync)
			{
				TRACE(_T("[PWR] %s Not Ready SyncMode(%d)\n"), strAxis, syncMode);
				return NOT_READY_SYNC_Y;
			}
		}
		linearIntplCommand.target[index] = pStartPos->CommandPos * pAxis->GetUnResol() * pAxis->GetMovingDir();
		retPosCommand = pAxis->GetProfileByRatio(&posCommand);
		pAxis->SetLinearIntplProfileCalcMode(linearIntplProfileCalcMode, false, retPosCommand.profile.dec);

		linearIntplCommand.maxVelocity[index] = retPosCommand.profile.velocity;
		linearIntplCommand.maxAcc[index] = retPosCommand.profile.acc;
		linearIntplCommand.maxDec[index] = retPosCommand.profile.dec;
		MoveAxisCnt++;

		if (abs(pAxis->ReadProfileTargetPosition() - pStartPos->CommandPos) > 0.0001)
		{
			TRACE(_T("[PWR] MotionCommand StartLinearIntplPos Start (%s %.3f -> %.3f %.1f))\n"), 
				pAxis->GetAxisName(), pAxis->ReadProfileTargetPosition(), pStartPos->CommandPos, pAxis->GetRatio() * 100.0);
		}
	}
	if (MoveAxisCnt > 0)
	{
		Err = GetCoreMotion()->motion->StartLinearIntplPos(&linearIntplCommand);
		if (Err != ErrorCode::None)
		{
			CoreMotionErrorToString(Err, _T("StartLinearIntplPos with profile"));
		}
	}
	for (INT_PTR index = 0; index < GetWmx3LinearIntpAxisCount(); ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		AxisIndex = GetAxisIndexFromAliasName(strAxis);
		pAxis = GetWmx3AxisByName(strAxis);
		if (pAxis == NULL)
		{
			TRACE(_T("[PWR] StartLinearIntplPos(%s) is NULL\n"), strAxis);
			continue;
		}
		if (pAxis->IsGantryAxis() == true)
		{
			//if (gcPowerLog->IsShowMotionLockLog() == true)
			//{
			//	TRACE(_T("[PWR] StartLinearIntplPos AxisIndex(%d) Unlock\n"), AxisIndex);
			//}
			//SEM_UNLOCK(gMOTION_LOCK[AxisIndex]);
		}
	}
	return Err;
}

long Cwmx3LinearIntplPos::StartLinearIntplPosR()
{
	long Err = ErrorCode::None;
	unsigned int LinearIntpCount = 0;
	LinearIntpCount = (unsigned int)GetWmx3LinearIntpAxisRCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartLinearIntplPos Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	Motion::LinearIntplCommand linearIntplCommand = Motion::LinearIntplCommand();
	StartPosStruct* pStartPos;
	Cwmx3Axis* pAxis;
	CString strAxis;
	linearIntplCommand.axisCount = LinearIntpCount;
	Config::MotionParam motionParam = Config::MotionParam();
	Motion::PosCommand posCommand = Motion::PosCommand();
	Motion::PosCommand retPosCommand = Motion::PosCommand();
	double Dist = 0.0;
	long linearIntplProfileCalcMode = 0;
	long MoveAxisCnt = 0, AxisIndex = 0;
	WMX3AxisSyncMode syncMode;
	for (INT_PTR index = 0; index < GetWmx3LinearIntpAxisRCount(); ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisRByIndex(index);
		strAxis = pStartPos->StrAxis;
		pAxis = GetWmx3AxisByName(strAxis);
		if (pAxis == NULL)
		{
			TRACE(_T("[PWR] StartLinearIntplPos(%s) is NULL\n"), strAxis);
			continue;
		}
		Dist = pStartPos->Distance;
		linearIntplProfileCalcMode = pStartPos->linearIntplProfileCalcMode;
		posCommand.profile = pAxis->GetMoveProfile(Dist);
		linearIntplCommand.axis[index] = pAxis->GetAxisMap();
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			if (syncMode == WMX3AxisSyncMode::NoSync)
			{
				TRACE(_T("[PWR] %s Not Ready SyncMode(%d)\n"), strAxis, syncMode);
				return NOT_READY_SYNC_Y;
			}
		}
		linearIntplCommand.target[index] = pStartPos->CommandPos * pAxis->GetUnResol() * pAxis->GetMovingDir();
		retPosCommand = pAxis->GetProfileByRatio(&posCommand);
		pAxis->SetLinearIntplProfileCalcMode(linearIntplProfileCalcMode, false, retPosCommand.profile.dec);

		linearIntplCommand.maxVelocity[index] = retPosCommand.profile.velocity;
		linearIntplCommand.maxAcc[index] = retPosCommand.profile.acc;
		linearIntplCommand.maxDec[index] = retPosCommand.profile.dec;
		MoveAxisCnt++;
	}
	if (MoveAxisCnt > 0)
	{
		Err = GetCoreMotion()->motion->StartLinearIntplPos(&linearIntplCommand);
		if (Err != ErrorCode::None)
		{
			CoreMotionErrorToString(Err, _T("StartLinearIntplPos with profile"));
		}
	}
	for (INT_PTR index = 0; index < GetWmx3LinearIntpAxisCount(); ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		AxisIndex = GetAxisIndexFromAliasName(strAxis);
		pAxis = GetWmx3AxisByName(strAxis);
		if (pAxis->IsGantryAxis() == true)
		{
			//if (gcPowerLog->IsShowMotionLockLog() == true)
			//{
			//	TRACE(_T("[PWR] StartLinearIntplPosR AxisIndex(%d) Unlock\n"), AxisIndex);
			//}
			//SEM_UNLOCK(gMOTION_LOCK[AxisIndex]);
		}
	}
	return Err;
}

long Cwmx3LinearIntplPos::StartPathLinearIntplPos(Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal)
{
	bool bFirst = true;
	long Err = ErrorCode::None;
	unsigned int LinearIntpCount = 0;
	LinearIntpCount = (unsigned int)GetWmx3LinearIntpAxisCount();
	if (LinearIntpCount < MIN_LINEAR_INTP_AXIS)
	{
		TRACE(_T("[PWR] StartPathLinearIntplPos Axis count under %d\n"), MIN_LINEAR_INTP_AXIS);
		return UNDER_MIN_LINEAR_INTP_AXIS;
	}
	StartPosStruct* pStartPos;
	Cwmx3Axis* pAxis;
	CString strAxis;
	Config::MotionParam motionParam = Config::MotionParam();
	Motion::PosCommand posCommand = Motion::PosCommand();
	Motion::PosCommand retPosCommand = Motion::PosCommand();
	double Dist = 200.0;
	long linearIntplProfileCalcMode = 0;
	long MoveAxisCnt = 0, AxisIndex = 0;
	WMX3AxisSyncMode syncMode;
	AdvMotion::PathIntplCommand path = AdvMotion::PathIntplCommand();
	
	for (INT_PTR index = 0; index < GetWmx3LinearIntpAxisCount(); ++index)
	{
		pStartPos = GetWmx3LinearIntpAxisByIndex(index);
		strAxis = pStartPos->StrAxis;
		pAxis = GetWmx3AxisByName(strAxis);
		if (pAxis == NULL)
		{
			TRACE(_T("[PWR] StartPathLinearIntplPos(%s) is NULL\n"), strAxis);
			continue;
		}
		Dist = pStartPos->Distance;
		posCommand.profile = pAxis->GetMoveProfile(Dist);
		retPosCommand = pAxis->GetProfileByRatio(&posCommand);
		path.axis[index] = pAxis->GetAxisMap();
		if (pAxis->GetUseSlaveAxis() == true)
		{
			syncMode = pAxis->GetAxisSyncMode();
			if (syncMode == WMX3AxisSyncMode::NoSync)
			{
				TRACE(_T("[PWR] %s Not Ready SyncMode(%d)\n"), strAxis, syncMode);
				return NOT_READY_SYNC_Y;
			}
		}
		if (bFirst == true)
		{
			path.profile[0] = retPosCommand.profile;
			bFirst = false;
		}
		if (index == 0) // X
		{
			path.target[0][0] = Linear.x * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.target[0][1] = EndCenter.x * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.centerPos[0][1] = StartCenter.x * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.target[0][2] = Goal.x * pAxis->GetUnResol() * pAxis->GetMovingDir();
		}
		else // Y 
		{
			path.target[1][0] = Linear.y * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.target[1][1] = EndCenter.y * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.centerPos[1][1] = StartCenter.y * pAxis->GetUnResol() * pAxis->GetMovingDir();
			path.target[1][2] = Goal.y * pAxis->GetUnResol() * pAxis->GetMovingDir();
		}
	}
	//Specify one motion profile for entire path
	path.enableConstProfile = 1;
	//Define linear and circular segments
	path.numPoints = 3;
	path.type[0] = AdvMotion::PathIntplSegmentType::Linear;
	path.type[1] = AdvMotion::PathIntplSegmentType::Circular;
	path.direction[1] = 1;
	path.type[2] = AdvMotion::PathIntplSegmentType::Linear;
	Err = GetAdvancedMotion()->advMotion->StartPathIntplPos(&path);
	TRACE(_T("[PWR] StartPathLinearIntplPos LinearXY,%.3f,%.3f CenterXY Start,%.3f,%.3f End,%.3f,%.3f GoalXY,%.3f,%.3f Err:%d\n"), 
		Linear.x, Linear.y, StartCenter.x, StartCenter.y, EndCenter.x, EndCenter.y, Goal.x, Goal.y, Err);
	if (Err != ErrorCode::None)
	{
		AdvancedMotionErrorToString(Err, _T("StartPathLinearIntplPos"));
	}
	return Err;
}
