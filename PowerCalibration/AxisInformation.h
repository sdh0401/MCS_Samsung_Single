#pragma once
#include "GlobalDefine.h"
#include "Wmx3MotorDefine.h"
#include "CFeeder.h"
#include <iostream>

#include <WMX3Api.h>
#include <EcApi.h>
#include <CoreMotionApi.h>

using namespace wmx3Api;
using namespace ecApi;

Point_XY ReadBlockTarget(const Point_XY& targetPoint, const Point_XY& originPoint);

/// <returns>degree * M_PI / 180.0</returns>
constexpr double convertDegreeToRadian(const double& degree);

/// <returns>radian * 180.0 / M_PI</returns>
constexpr double convertRadianToDegree(const double& radian);

/// <summary>
/// source 좌표를 반시계방향으로 angle만큼 회전시키고, origin 만큼 평행이동한 결과를 return.
/// </summary>
/// <param name="source"></param>
/// <param name="rotateAngleInDegree"></param>
/// <param name="origin"></param>
/// <returns></returns>
Point_XY transformCoordinate(const Point_XY& source, const double& rotateAngleInDegree, const Point_XY& origin);

/// <summary>
/// 장비 좌표계에서 x축방향으로 return.x 만큼, y축방향으로 return.y 만큼 이동하면 origin을 원점으로하고 rotateAngleInDegree만큼 회전한 [변환된 frame]좌표계에서 x축방향으로 source.x 만큼, y축방향으로 source.y 만큼 움직이게 됩니다.
/// </summary>
/// <param name="source">[변환된 frame]에서의 coordinate</param>
/// <param name="origin">장비 기준 [변환된 frame]의 원점 좌표</param>
/// <param name="rotateAngleInDegree">[단위 : degree] 장비 기준 변환된 frame이 위에서보았을때 CCW방향으로 몇도 회전했는지.</param>
/// <param name="isFromEquipmentToBlockBased"></param>
/// <returns>장비 좌표계 좌표</returns>
Point_XY transformCoordinateFromBlockBasedToEquipment(const Point_XY& source, const Point_XY& origin, const double& rotateAngleInDegree);

/// <summary>
/// 각도만큼 돌아간 블록원점기준으로 return.x, return.y 위치로 움직이면 장비기준 source.x, source.y 로 움직인것과 동일하다.
/// </summary>
/// <param name="source">장비기준 좌표</param>
/// <param name="origin">장비기준 블록 원점 위치</param>
/// <param name="rotateAngleInDegree">장비기준 블록 반시계방향 회전 각도</param>
/// <returns></returns>
Point_XY transformCoordinateFromEquipmentToBlockBased(const Point_XY& source, const Point_XY& origin, const double& rotateAngleInDegree);

/// <summary>
/// </summary>
/// <param name="strMsg"></param>
void getDegree(const CString& strMsg);

/// <summary>
/// blockNo블록위에서 initialRotation각도는 장비기준에서 실제로 return각도로 돌아야 그 각도입니다.
/// ex) R이 180인 블록에서 R Position 90(=initialRotation) 으로 회전하면 실제로는 270(=return) 도 회전.
/// </summary>
/// <param name="initialRotation">블록 기준으로 R축 변위</param>
/// <param name="blockNo"></param>
/// <returns>initialRotation + [blockNo블록의 rotation값] (단순 덧셈)</returns>
double calculateBlockRotationOffset(const double& initialRotation, const long& blockNo);

CString getFileNameFromFullPath(const std::string& fileName);
#define TRACE_FILE_FUNC_LINE_ TRACE("[PWR] %s::%s(%d)->%s", (CStringA)getFileNameFromFullPath(__FILE__), __func__, __LINE__, 

extern unsigned PowerAxisArray[MAXAXISNO];
extern unsigned PowerAxisMap[MAXAXISNO];
extern unsigned PowerAxisSlaveID[MAXAXISNO];
extern signed PowerAxisMovingDir[MAXAXISNO];
extern CString PowerAxisAliasName[MAXAXISNO];
extern CString PowerAxisName[MAXAXISNO];
extern double PowerAxisResol[MAXAXISNO];
extern double PowerAxisUnResol[MAXAXISNO];
extern bool PowerAxisUseMasterSlave[MAXAXISNO];
extern unsigned PowerAxisWhatMasterSlave[MAXAXISNO];
extern CMotionControlType PowerAxisMotionControlType[MAXAXISNO];
extern CMotorType PowerAxisMotorType[MAXAXISNO];
extern WMX3_AXIS_POSCOMMANDPROFILE PowerAxisMoveParam[MAXAXISNO];
extern WMX3_AXIS_POSCOMMANDPROFILE PowerAxisTeachMoveParam[MAXAXISNO];
extern WMX3_AXIS_POSCOMMANDPROFILE PowerAxisTeachJogParam[MAXAXISNO];
extern double gShortDist[MAX_SHORT_DIST_LEVEL];
extern double gShortDistVel[MAX_SHORT_DIST_LEVEL];
extern double gShortDistAccDec[MAX_SHORT_DIST_LEVEL];
extern ULONGLONG PowerHomingMaxTimeOut[MAXAXISNO];

extern long GetAxisID(CString strAxis);
extern bool CheckServoOn(CString strAxis);
extern long GetServoOnError(CString strAxis);
extern bool CheckSlaveServoOn(CString strAxis);
extern long GetSlaveAxisIndex(CString strAxis);
extern long GetSlaveAxisSlaveID(CString strAxis);
extern bool AlarmClear(CString strAxis);
extern bool ClearSlaveAmpAlarm(CString strAxis);
extern bool CheckAmpAlarm(CString strAxis);
extern bool CheckSlaveAmpAlarm(CString strAxis);
extern bool ServoOn(CString strAxis);
extern bool ServoOff(CString strAxis);
extern bool SlaveServoOn(CString strAxis);
extern bool SlaveServoOff(CString strAxis);
extern long CheckLimitOver(CString strAxis, double Command);
extern long GetSuctionIONo(long Gantry, long HeadNo);
extern long GetBlowIONo(long Gantry, long HeadNo);
extern long CheckAirPressureLow();
extern double GetDryRunZHeightOffset();
extern void SetInsertByZ(long Gantry, double InsertByZ);
extern double GetInsertByZ(long Gantry);
extern void SetPusherByZ(long Gantry, double PusherByZ);
extern double GetPusherByZ(long Gantry);
extern void SetStandByZ(long Gantry, double StandByZ);
extern double GetStandByZ(long Gantry);
extern double GetRuntimeSafetyZ(long Gantry);
extern double GetTorqueOverSafetyZ();
extern void SetMaxZTorqueLimit(long Gantry, long Head, double MaxZTorqueLimit);
extern double GetMaxZTorqueLimit(long Gantry, long Head);
extern void SetStandByR(long Gantry, double StandByR);
extern double GetStandByR(long Gantry);
extern double GetUnResol(CString strAxis);
extern Point_XY gReadGantryPosition(long Gantry);
extern long WaitGantryRZIdle(long TimeOut);
extern long WaitAllZIdle(long Gantry, long TimeOut);
extern long WaitAllRIdle(long Gantry, long TimeOut);
extern long WaitGantryIdle(long Gantry, long TimeOut);
extern void SetHeadOffset(long Gantry, long HeadNo, Point_XY Offset);
extern Point_XY GetHeadOffset(long Gantry, long HeadNo);
extern Point_XY GetCameraRecognitionPosition(long Gantry, long HeadNo);
extern void SetCameraRecognitionPosition(long Gantry, long HeadNo, Point_XY Position);
extern void SetCameraRecognitionOffset(long Gantry, long HeadNo, Point_XY Offset);
extern bool IsCameraCenterPositionByHeadNo(long HeadNo);
extern bool IsCamera6CenterPositionByHeadNo(long HeadNo);
extern long GetCameraHeadFromHeadNo(long HeadNo);
extern long GetCamera6HeadFromHeadNo(long HeadNo);
extern long GetCameraNoByHead(long Gantry, long HeadNo);
extern long GetCamera6NoByHead(long Gantry, long HeadNo);
extern long GetCameraChkPosByHead(long HeadNo);
extern long GetCamera6ChkPosByHead(long HeadNo);
extern double ReadPosition(CString strAxis);
extern double ReadPosition(long AxisNo);
extern void ReadAllPosition(long Gantry);
extern double ReadCommandPosition(CString strAxis);
extern double ReadOneCommandVelocity(CString strAxis);
extern double ReadProfileTargetPosition(CString strAxis);
extern long WritePosition(CString strAxis, double Position);
extern double ReadMotorActualPulse(CString strAxis);
extern double ReadActualTorque(CString strAxis);
extern double Read1DCompensationData(CString strAxis);
extern double Read2DCompensationData(CString strAxis);
extern double ReadVirtualPositiveLimit(CString strAxis);
extern double ReadVirtualNegativeLimit(CString strAxis);
extern bool IsNegativeLimitSwitchOn(CString strAxis);
extern bool IsPositiveLimitSwitchOn(CString strAxis);

extern long GetAxisMap(CString strAxis);
extern CString GetAxisX(long Gantry);
extern CString GetAxisY1(long Gantry);
extern CString GetAxisY2(long Gantry);
extern CString GetConvName(long Conveyor);
extern CString GetPusherZName(long Conveyor);
extern CString GetRAxisFromHeadNo(long Gantry, long HeadNo);
extern CString GetZAxisFromHeadNo(long Gantry, long HeadNo);
extern CString GetHeadToRotateAxis(long Gantry, CString strZAxis);
extern CString GetRotateAxisToHead(long Gantry, CString strRAxis);
extern void SetOneRatio(CString strAxis, double Ratio);
extern void SetOne2ndRatio(CString strAxis, double Ratio2nd);
extern void InitOneRatio(CString strAxis);
extern long SetOnePosSet(CString strAxis, double Inpos);
extern long SetOneInPos(CString strAxis, double Inpos);
extern long SetOneDelayedPosSet(CString strAxis, double Inpos, long Time);
extern long StartOneJog(CString strAxis, JogInfo jogInfo);
extern long StartOnePosition(CString strAxis, double Position);
extern long StartOnePositionSkipLimitCheck(CString strAxis, double Position);
extern long StartOne2StepPosition(CString strAxis, double Position, double Position2nd);
//extern long StartOnePositionX(CString strAxis, double Position, double Ratio);
//extern long StartOnePositionY(CString strAxis, double Position, double Ratio);
extern long StartOnePositionWithoutSlave(CString strAxis, double Position);
extern long StartOneMove(CString strAxis, double Offset);
extern long StartOneMoveWithoutSlave(CString strAxis, double Offset);
extern long StartOneTeachMove(CString strAxis, double Offset);
extern long StopOne(CString strAxis);
extern long StopOne(CString strAxis, double Dec);
extern long GetOneOnlyIdle(CString strAxis);
extern long WaitOneIdle(CString strAxis, long TimeOut);
extern long WaitOneMotion(CString strAxis, double CmdPos, double Inpos, long TimeOut);
extern long WaitOnePosSet(CString strAxis, double CmdPos, double Inpos, long TimeOut);
extern long WaitOneInPos(CString strAxis, double CmdPos, double Inpos, long TimeOut);
extern long WaitOneDelayedPosSet(CString strAxis, double CmdPos, double Inpos, long TimeOut, bool UseSendAlarm = true);
extern long OneOriginSearch(CString strAxis, double forceOffset);
extern long SetInitializeEnd(CString strAxis, bool bEnd);
extern long GetTorqueLimit(CString strAxis, double* pTorqueLimit);
extern long SetTorqueLimit(CString strAxis, double Torque);
extern long SetEventToStopByOverTorque(long EventId, CString strAxis, double Torque, double SaftyPosition);
extern long SetEventToOverrideVelByPosition(long EventId, CString strAxis, double position, double ratio);
extern long SetEventToStopByAreaSensor(long EventID, long AreaSensor, CString strAxis);
extern long ClearAllEvent();
extern long RemoveEvent(long EventId);
extern long EnableEvent(long EventId);
extern long StartMonitor(CString strAxis, long BoardCount, long BlockNo, long InsertNo, CString Action);
extern CString GetTorqueMonitorFileName(CString strAxis);
extern long StopMonitor(CString strAxis);
extern long WaitStopMonitor(CString strAxis);
extern long ResetMonitor(CString strAxis);
extern void SetShortDist(CString strAxis, long index, double ShortDist);
extern void SetShortDistVel(CString strAxis, long index, double ShortDistMaxVel);
extern void SetShortDistAccDec(CString strAxis, long index, double ShortDistMaxAccDec);
extern void SetMoveProfile(CString strAxis, WMX3_AXIS_POSCOMMANDPROFILE profile);

extern long StartPosWaitMotion(CString strAxis, double Ratio, long TimeOut, double Pos, bool Wait);
extern long StartPosWaitMotionSkipLimitCheck(CString strAxis, double Ratio, long TimeOut, double Pos, bool Wait);
extern long Start2StepPosWaitMotion(CString strAxis, double Ratio, double Ratio2nd, long TimeOut, double Pos, double Pos2nd, bool Wait);
extern long StartPosWaitInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, bool Wait);
extern long StartPosWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
extern long Start2StepPosWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait, double Ratio2nd, double Pos2nd);
extern long StartPosWaitDelayedInpositionWihtoutSlave(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
extern long StartPosWaitInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
extern long StartMoveWaitMotion(CString strAxis, double Ratio, long TimeOut, double Offset, bool Wait);
extern long StartMoveWaitInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, bool Wait);
extern long StartMoveWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait);
extern long StartMoveWaitDelayedInpositionWithoutSlave(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait);
extern long StartTeachMoveWaitMotion(CString strAxis, double Ratio, long TimeOut, double Offset, bool Wait);
extern long StartTeachMoveWaitInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, bool Wait);
extern long StartTeachMoveWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait);

extern long gServoAllOn();
extern long gServoOnWithAlarmClear(CString strAxis);
extern long gServoAllOff();
extern long gServoAllOffWithoutConv();
extern long gMoveAllLastPosition();
extern long GetAllAxisHomingInComplete();
extern bool IsAllAxisHomingComplete();
extern long GetHomingCompleteError();
extern bool IsAllZAxisHomingComplete();
extern bool IsAllRAxisHomingComplete();
extern bool IsOneAxisHomingComplete(INT_PTR indx);
extern bool IsOneAxisHomingComplete(CString strAxis);
extern bool IsAllAxisHomingFail();
extern bool IsOneAxisHomingFail(CString strAxis);
extern bool SetSyncMasterSlave();
extern bool IsGantryAxis(CString strAxis);
extern bool IsUseSlaveAxis(CString strAxis);
extern long GetZAxisCount();
extern CString GetZAxisByIndex(INT_PTR indx);
extern long GetZAxisIndexByZName(CString strZAxis);
extern long GetRAxisCount();
extern CString GetRAxisByIndex(INT_PTR indx);

extern INT_PTR AddLinearIntpAxis(long Gantry);
extern INT_PTR AddAllRAxis();
extern INT_PTR AddSomeRAxis(SomeTarget Target);
extern INT_PTR AddAllZAxis();
extern INT_PTR AddSomeZAxis(SomeTarget Target);

extern long SetMultiCommand(long Gantry, double pos);
extern long SetMultiCommand(long Gantry, SomeTarget Target);
extern long SetMultiCommand(long Gantry, long Target, double x, double y);

extern void SetMultiRatio(double Ratio);
extern void InitMultiRatio();
extern long StartMultiPosition(long Gantry, SomeTarget Target, bool Wait);
extern long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, bool Wait);
extern long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, bool Wait);
extern long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, long Ms, bool Wait);
extern long StartMultiPosition(long Gantry, Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal, long TimeOut, bool Wait);
extern long WaitMultiMotion(long Gantry, SomeTarget Target);
extern long WaitMultiMotion(long Gantry, long TimeOut);
extern long WaitMultiPosSet(long Gantry, SomeTarget Target);
extern long WaitMultiPosSet(long Gantry, long TimeOut);
extern long WaitMultiDelayedPosSet(long Gantry, SomeTarget Target);
extern long WaitMultiDelayedPosSet(long Gantry, long TimeOut);

extern long SetMultiCommandR(long Gantry, double pos);
extern long SetMultiCommandR(long Gantry, SomeTarget Target);
extern long SetMultiCommandR(long Gantry, long Target, double x, double y);

extern void SetMultiRatioR(double Ratio);
extern void InitMultiRatioR();
extern long StartMultiPositionR(long Gantry, SomeTarget Target, bool Wait);
extern long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, bool Wait);
extern long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, bool Wait);
extern long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, long Ms, bool Wait);
extern long WaitMultiMotionR(long Gantry, SomeTarget Target);
extern long WaitMultiMotionR(long Gantry, long TimeOut);
extern long WaitMultiPosSetR(long Gantry, SomeTarget Target);
extern long WaitMultiPosSetR(long Gantry, long TimeOut);
extern long WaitMultiDelayedPosSetR(long Gantry, SomeTarget Target);
extern long WaitMultiDelayedPosSetR(long Gantry, long TimeOut);

extern void ReadMultiPosition();
extern void ReadMultiPositionR();
extern void RemoveLinearIntpAxis();
extern void RemoveLinearIntpAxisR();
extern long SetMultiPosSet(double Inpos);
extern long SetMultiPosSet(SomeTarget Target);
extern long SetMultiDelayedPosSet(double Inpos, long Ms);
extern long SetMultiDelayedPosSet(SomeTarget Target);

extern long LinearIntplPosWaitMotion(long Gantry, long Target, Point_XY pt, double Ratio, long TimeOut);
extern long LinearIntplPosWaitPosSet(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long TimeOut);
extern long LinearIntplPosWaitDelayedPosSet(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
extern long LinearPathIntplPos(long Gantry, long Target, Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal, double Ratio, long TimeOut);

extern long StartSomeRAxisWaitMotion(long Gantry, SomeTarget Target, bool Wait);
extern long StartSomeRAxisWaitPosSet(long Gantry, SomeTarget Target, bool Wait);
extern long StartSomeRAxisWaitDelayedPosSet(long Gantry, SomeTarget Target, bool Wait);

extern long StartAllRAxisWaitMotion(long Gantry, double pt, double Ratio, long TimeOut);
extern long StartAllRAxisWaitPosSet(long Gantry, double pt, double Ratio, double Inpos, long TimeOut);
extern long StartAllRAxisWaitDelayedPosSet(long Gantry, double pt, double Ratio, double Inpos, long Ms, long TimeOut);

extern long StartSomeZAxisWaitMotion(long Gantry, SomeTarget Target);
extern long StartSomeZAxisWaitPosSet(long Gantry, SomeTarget Target);
extern long StartSomeZAxisWaitDelayedPosSet(long Gantry, SomeTarget Target);

extern long StartAllZAxisWaitMotion(long Gantry, double pt, double Ratio, long TimeOut);
extern long StartAllZAxisWaitPosSet(long Gantry, double pt, double Ratio, double Inpos, long TimeOut);
extern long StartAllZAxisWaitDelayedPosSet(long Gantry, double pt, double Ratio, double Inpos, long Ms, long TimeOut);
extern long PrepareMoveConvWidth();
extern long MoveConvWidth(long Conv, double Width);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern UBYTE ReadOutputOne(long no);
extern bool ReadOutputTimeOne(long port, long onoff, long dwTime);
extern void OutputOne(long no, long onoff);
extern UBYTE InputOne(long no);
extern bool InputTimeOne(long port, long onoff, long dwTime);
extern bool InputElapsedTimeOne(long port, long onoff, ULONGLONG dwTime);
extern bool InputElapsedTimeWait(long port, long onoff, ULONGLONG dwTime, ULONGLONG timeOut);
extern bool OutputElapsedTimeOne(long port, long onoff, ULONGLONG dwTime);
extern double GetTemperature(CString strAxis);
extern double GetHeight(long Gantry);
extern long GetAnalogLevel(long Gantry, long HeadNo);
extern long SuctionOne(long Gantry, long HeadNo, bool bSuction);
extern long BlowOne(long Gantry, long HeadNo, bool bBlow);
extern long SuctionAll(long Gantry, bool bSuction);
extern long BlowAll(long Gantry, bool bBlow);
extern bool GetOneSuction(long Gantry, long HeadNo);
extern bool GetOneBlow(long Gantry, long HeadNo);
extern bool GetAllSuction(long Gantry);
extern bool GetAllBlow(long Gantry);
extern long GetReadyIONoFromReadyNo(long ReadyNo);
extern long GetReleaseIONoFromReleaseNo(long ReleaseNo);
extern long GetIOAddr(long IONum);
extern char GetIOBit(long IONum);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern bool WriteHomePosition(CString strAxis, double HomePosition);
extern double ReadHomePosition(CString strAxis);
extern bool WriteCameraAlignPosition(long Gantry, Point_XY pt);
extern Point_XY ReadCameraAlignPosition(long Gantry);
extern bool WritePcbFixPosition(long Conveyor, Point_XY pt);
extern Point_XY ReadPcbFixPosition(long Conveyor);
extern void WriteReferenceFeederPosition(long Stage, long RefFdNo, Point_XY RefFdPos);
extern void WriteFeederPitch(long Stage, double Pitch);
extern Point_XY ReadReferenceFeederPosition(long Stage);
extern long ReadReferenceFeederNo(long Stage);
extern double ReadFeederPitch(long Stage);
extern Point_XY GetFeederPosition(long Stage, long FdNo);
extern Point_XY ReadDistOriginFromPcbFix(Point_XY Origin);
extern Point_XY ReadDistMarkFromOrigin(Point_XY MarkPt, Point_XY Origin);
extern Point_XY ReadDistInsertFromOrigin(Point_XY Insert, Point_XY Origin);
extern Point_XY ReadOriginFromPcbFix(Point_XY Origin);
extern Point_XY ReadMarkFromOrigin(Point_XY MarkPt, Point_XY Origin, const bool& isSkipReadBlockTarget = false);
extern Point_XY ReadInsertFromOrigin(Point_XY Insert, Point_XY Origin);
extern bool ReadBlockMarkFromOrigin(long BlockNo, Point_XY* Mark1Pt, Point_XY* Mark2Pt);
extern Point_XYR GetInsertXYFromJobfile(long BlockNo, long InsertNo);
extern Point_XYR GetOriginXYRFromJobfile(long BlockNo);
extern Point_XY ReadInsertFromOrigin(Point_XY Insert, Point_XY Origin);
//extern Point_XYR ReadInsertFromOrigin(Point_XYR Insert, Point_XYR Origin);
extern void WriteConfirmInsertBeforePosition(Point_XY Before);
void WriteConfirmInsertBeforePosition(const long& gantry, const Point_XY& Before);
extern Point_XY ReadConfirmInsertBeforePosition();
extern void WriteConfirmMeasureHeightBeforePosition(Point_XYT Before);
extern Point_XYT ReadConfirmMeasureHeightBeforePosition();

//////////////////////////////////////////////////////////////////////////////////////////////////

extern void WritePusherZ(long Conveyor, long Type, double Position);
extern void WriteWidth(long Conveyor, long Type, double Position);

//////////////////////////////////////////////////////////////////////////////////////////////////
extern long gDisable1DCompensation();
extern long gEnable1DCompensation();
extern long gSet1DCompensation();
extern void gShow1DData();
extern void gShow1DData(unsigned nCompenMax);
extern long oneDCompensationOn();
extern long oneDCompensationOff();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long gDisable2DCompensation(long Channel);
extern long gEnable2DCompensation(long Channel);
extern long gSet2DCompensation_ForXY(long Gantry);
extern long gSet2DCompensation_ForXY(long Gantry, long MarkXCount, long MarYCount);
extern long twoDCompensationOn();
extern long twoDCompensationOff();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern double gGetHomePosition(long AxisNo);
extern void gSetHomePosition(long AxisNo, double Origin);
extern void gWriteHomePosition(long Gantry);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long gDisableZCompensation(CString strAxisZ);
extern long gEnableZCompensation(CString strAxisZ);
extern long gSetZCompensation(CString strAxisZ);
extern long AxisZCompensationOn(CString strAxisZ);
extern long AxisZCompensationOff(CString strAxisZ);
extern long AllZCompensationOn(long Gantry);
extern long AllZCompensationOff(long Gantry);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long gPauseMachine();
extern long gResumeMachine();
extern long gReleaseMachine();
extern long gStopMachine();

extern void SetRunMode(long Mode);
extern long GetRunMode();
extern long GetRunModeNoLog();
extern void SetStopMode(long Mode);
extern long GetStopMode();
extern bool WaitRunMode(long Mode, long TimeOut);
extern void InitCStep(long Gantry);
extern void DeleteCStep(long Gantry);
extern void CreateCStep(long Gantry);
extern bool IsAliveCStep(long Gantry);

extern long SetTowerLamp(TowerLampLed UserLamp);
extern TowerLampLed GetTowerLamp();

extern void SetUseRearCamera(long UseRearCamera);
extern long GetUseRearCamera();

extern long CallbackHMI(unsigned Msg1, unsigned Msg2, unsigned Msg3, CString strMsg);
extern void CallbackHMI_Pause();
extern void CallbackHMI_Resume();
extern void CallbackHMI_StopNow();
extern void CallbackHMI_BoardStop();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long InitializeHeadMech(long Gantry);
extern long InitializeHeadMech(long Gantry, long HeadNo);
extern long InitializeCamPosMech(long Gantry);
extern long InitializeRearCamPosMech(long Gantry);
extern long InitializeRearCamOffsetMech(long Gantry);
extern long InitializeCamOffsetMech(long Gantry);
extern long SaveHeadOffsetCamPosOffsetValue(long Gantry);
extern long LoadHeadOffsetCamPosOffsetValue(long Gantry);
extern long SendCameraRecognitionOffset(long Gantry);
extern void SetGlobalDiscardPosition(Point_XYRZ Discard);
extern Point_XYRZ GetGlobalDiscardPosition();
extern void SetGlobalNozzleNo(long HeadNo, long StationNo);
extern long GetGlobalNozzleNo(long HeadNo);
extern void SetGlobalNozzleInformation(long StationNo, NOZZLE NozzleInfo);
extern NOZZLE GetGlobalNozzleInformation(long StationNo);
extern void SetGlobalTowerYellowLampTime(long YellowTowerLampTime);
extern long GetGlobalTowerYellowLampTime();
extern void SetGlobalEmptyBuzzerTime(long EmptyBuzzerTime);
extern long GetGlobalEmptyBuzzerTime();
extern void SetGlobalMachineReferenceMark(MachineReferenceMark ReferenceMark);
extern MachineReferenceMark GetGlobalMachineReferenceMark();
extern bool IsEmergencyStop();
extern double GetRotateAngle(double dest);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void SetEntryPcbReady(long Conveyor, bool bReady);
extern void SetWorkPcbReady(long Conveyor, bool bReady);
extern void SetWorkPcbOut(long Conveyor, bool Out);
extern bool IsEntryPcbReady(long Conveyor);
extern bool IsWorkPcbReady(long Conveyor);
extern bool IsWorkPcbOut(long Conveyor);
extern void SetInsertDone(long InsertDone);
extern long GetInsertDone(long Conveyor);
extern void SetPcbOutDone(long OutDone);
extern long GetPcbOutDone(long Conveyor);
extern bool IsStopperUp(long Conveyor);
extern bool IsStopperDn(long Conveyor);
extern long GetConveyorRunMode();
extern void SetConveyorRunMode(long ConveyorRunMode);
extern void SetProdRunMode(long ProdRunMode, long ConveyorRunMode);
extern long GetManualConveyorRunMode();
extern void SetManualConveyorRunMode(long ConveyorRunMode);
extern void StartConveyor(unsigned Msg1, unsigned Msg2, unsigned Msg3);
extern void SetPcbInfoConveyor(double Thickness, double StandByZOffset, long SimultaneousLoading);
extern void SetRunInfoConveyor(long BarcodeType, long Reserved1, long Reserved2);
extern void SetBarcodeResultConveyor(long Conveyor, long BarcodeResult, long Reserved1);
extern void RunConveyor(long ContinueRun, long From, long To);
extern void RunConveyor(long ContinueRun, double Width);
extern void EndConveyor();
extern void StopConveyor();
extern void StartFreeTimeConveyor(long Conv);
extern void StopFreeTimeConveyor(long Conv);
extern void StartLoadingTimeConveyor();
extern void EndLoadingTimeConveyor();
extern void StartLineOfBalanceConveyor();
extern void EndLineOfBalanceConveyor();
extern ULONGLONG GetLoadingTime();
extern ULONGLONG GetLineOfBalance();
extern void SetWorkConveyorStopMidDelay(long WorkStopMidDelay);
extern void SetWorkConveyorStopLowDelay(long WorkStopLowDelay);
extern void SetConveyorProfileLow(double Vel, double Acc, double Dec);
extern void SetConveyorProfileMid(double Vel, double Acc, double Dec);
extern void SetConveyorProfileHigh(double Vel, double Acc, double Dec);
extern void SetConveyorProfileSpeed(long Conv, long PrevInBeltSpd, long NextOutBeltSpd);
extern long GetConveyorProfileSpeedPrevIn(long Conv);
extern long GetConveyorProfileSpeedNextOut(long Conv);
extern long SetConveyorTransferTimeOut(long Table, long Conv, long PrevInBeltSpd, long NextOutBeltSpd);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void StartSyncGantryConveyor();
extern void RunSyncGantryConveyor();
extern void StopSyncGantryConveyor();
extern void StartManualLocationConveyor(long From, long To);
extern void SetPcbInfoManualLocationConveyor(double PcbThickness, double PcbStandByZOffset, long PusherzRatio);
extern void RunManualLocationConveyor(long From, long To);
extern void StopManualLocationConveyor(long From, long To);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void StartMain();
extern void PrepareMain();
extern void RunMain(bool ContinueRun);
extern void StopMain(long StopTiming);
extern void gFeederRefill(long FeederNo);
extern void gFeederRefillDone();
extern void gMainMoveStandBy();
extern bool GetMainEnd();
extern bool GetConveyorEnd();
extern bool GetConveyorBeltStop();
extern bool GetManualLocationConveyorEnd();
extern long SendFeederAutoRefill(long FeederNo);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern bool IsExistEnt(long Conv);
extern bool IsExistLow(long Conv);
extern bool IsExistSet(long Conv);
extern bool IsExistOut(long Conv);
extern bool IsExistExit(long Conv);
extern bool IsExistAll(long Conv);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void SaveInsertEnd();
extern void ClearInsertEnd(long Conv);
extern void SetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point);
extern void SetInsertClear(int Conv, int Gantry, unsigned Block, unsigned Point);
extern unsigned GetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point);
extern long GetRemainFirstBlock(long MaxBlockNo, long MaxInsertNo);
extern long GetRemainFirstNo(long MaxBlockNo, long MaxInsertNo);
extern bool GetAllInertEnd();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long GetAxisIndexFromAliasName(CString strAxis);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long SendToHMI(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg);
extern long SendInitializeHMI(long Status);
extern long SendAlarm(long AlarmCode, CString AlarmMsg);
extern long SendEmpty(long Feeder, long Type, CString EmptyMsg);
extern long SendAlarmForNormal(long AlarmCode, CString AlarmMsg);
extern long SendAlarmOnlyBuzzer(long AlarmCode, CString AlarmMsg);
extern long GetZTorqueAlarmcode(long Gantry, long HeadNo);
extern long GetZTorqueInsertFailAlarmcode(long Gantry, long HeadNo);
extern long GetZTorquePickFailAlarmcode(long Gantry, long HeadNo);
extern long SendRequestResetButton();
extern long SendResetDone();
extern long SendInitializeHome(long Timing);
extern long AlarmSelfQuit(long StartMode);
extern long SendDoorStatus(long Status, long FrontOrRear);
extern long SendServoStatus(long Status);
extern long SendEmergencyStatus(long Status);
extern long SendLotoKeyStatus(long Status);
extern long SendMachineStartStopStatus(long Status);
extern long SendProdInfo(long ProdCompleteCount, long ProdTime, long TactTime, long ChipPerHour, long ProdTimeWithLoadingTime, long TactTimeWithLoadingTime);
extern long SendProdInfo(long ProdCompleteCount, long ProdTime, long TactTime, long ChipPerHour, long ProdTimeWithLoadingTime, long TactTimeWithLoadingTime, CString strBarcode);
extern long SendBlockProdInfo(long ProdCompleteCount, long BlockProdCompleteCount);
extern long SendReadyIOStatus(CString ReadyIOStatus);
extern long SendGantryTempStatus(CString TempStatus);
extern long SendPcbSensorStatus(CString PcbSensor);
extern long SendToPrepareRun();
extern long SendToRun();
extern long SendToInsertComplete(long BlockNo, long InsertNo);
extern long SendToNewBoard();
extern long SendToZTorqueMonitorFile(CString strFileName, long TorqueLimit);
extern long SendInformation(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg);
extern long SendIOStatus(CString strMsg);
extern long SendTrayLastPocket(long TrayNo, long LastPocket);

extern long SendNozzleNoStatus();
extern long SendToPickupStatisticsComplete(long FeederNo, long NozzleNo);
extern long SendToInsertStatisticsComplete(long InsertNo, long FeederNo, long NozzleNo);
extern long SendToNoComponentStatistics(long FeederNo, long NozzleNo);
extern long SendToRunTimeDiscardStatistics(long FeederNo, long NozzleNo);
extern long SendToPickSkipByLastPick(long FeederNo);
extern long SendToVisionResult(long Gantry, long InsertNo, long ErrorCode);

extern long SendPopupMessage(CString Message);
extern long SendPopupClose();
extern long SendToForming1Statistics(long FeederNo);
extern long SendToForming2Statistics(long FeederNo);
extern long SendSuctionDifferentLevel(long FeederNo, long Level);
extern long SendToLedRetrytatistics(long FeederNo);
extern long SendToChangeMachineState(TimeStatics state);
extern CString GetTimeStaticsName(TimeStatics set);
extern long SendManualHeightMeasureEnd();
extern long SendLampStatus(long Green, long Yellow, long Red, long AlarmCode);

//////////////////////////////////////////////////////////////////////////////////////////////////
extern long CheckReadyToMachine(bool ServoCheck);
extern long CheckMachineSafety();
extern long GantryEventEnable();
//////////////////////////////////////////////////////////////////////////////////////////////////
//extern void BlinkingIO(bool blink);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void gLedOn(int CameraNo, int iValue1, int iValue2, int iValue3);
extern void gLedAllOff();
extern void gLedAllOn();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void gLaserOn(int CameraNo);
extern void gLaserOff(int CameraNo);
extern void gLaserAllOff();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void StartFeeder(long FeederNo, long ReadyIONo, long ReleaseIONo);
extern void SetInfoFeeder(long FeederNo, long ReadyIONo, long ReleaseIONo);
extern void RunFeeder(long FeederNo);
extern void StopFeeder(long FeederNo);
extern CFeeder* GetFeeder(long FeederNo);
extern void SetFeederProdRunMode(long RunMode);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void ClearY2Shift(long Gantry);
extern void ClearCameraRecognitionPosition(long Gantry);
extern void ClearCameraRecognitionOffset(long Gantry);
extern Limit GetLimit(long AxisNo);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void HeightMeasurementControl(bool On);
extern void MotorPowerControl(bool On);
extern void DoorLockingControl(bool Lock);
extern void TowerLampRed(bool On);
extern void TowerLampYel(bool On);
extern void TowerLampGrn(bool On);
extern long GetTowerLampRed();
extern long GetTowerLampYel();
extern long GetTowerLampGrn();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void gQuitMachine();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void TowerLampMsg(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg);
extern void TowerLampAlarm(long AlarmCode);
extern void TowerLampEmpty(long AlarmCode);
extern void TowerLampWaitPcb(long Conv, long AlarmCode);
extern void TowerLampInPcb();
extern void TowerLampOutPcb();
extern void TowerLampRun();
extern void TowerLampNormal();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void ShowSmemaIO();
extern void ShowPCBSensor();
extern void ShowSuctionIO();
extern void ShowBlowIO();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long MoveZStandy(long Gantry, double StandByZ, double Ratio);
extern long MoveZStandyOneToJig(long Gantry, long Target, double StandByZ, double Ratio);
extern long MoveZStandySkipServoOff(long Gantry, double StandByZ, double Ratio);
extern long WaitZStandy(long Gantry, double StandByZ);
extern long MoveRStandy(long Gantry, double TargetR, double Ratio);
extern long WaitRStandBy(long Gantry, double TargetR);
extern long MoveStandByXY(long Gantry, long Target, Point_XY pt, double Ratio, long TimeOut);
extern long MoveZAllUpLimitWithOutOneHead(long Gantry, double Ratio, CString WithoutHead);
extern long MoveZAllUpLimit(long Gantry, double Ratio);
extern long MoveZAllUpLimitWait(long Gantry, long TimeOut);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void BuzzerOn(long BuzzerOnTime);
extern void BuzzerOff();
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long GantryMoveStandByZ(long Gantry, double Ratio);
//////////////////////////////////////////////////////////////////////////////////////////////////
extern long SetDefaultTorqueLimitEvent(long Gantry);
extern long SetDefaultAreaSensorEvent();
extern long RemoveHeadTorqueLimitEvent(long Gantry, long HeadNo);
extern long SetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit);
extern long SetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit, double SafetyPosition);
extern long RemoveSetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit, double SafetyPosition);
extern long InitialHeadTorqueLimit(long Gantry, long HeadNo);
extern void MoveOneTimeUnlock();
//extern bool StepMoveLock();
extern bool MoveOneTimeLockWait(long WaitTime);
extern long MoveOnceLockPauseMode(CString strAxis);
extern long MoveOnceLockPauseMode(CString strAxis, double CmdPosition);
extern long MoveOnceLockPauseModeTTF(CString TTFName);
extern void SetZRMoveOnce(bool Set);
extern bool GetZRMoveOnce();
extern bool WaitAxisPauseState(long TimeOut);
extern void ResetAxisPauseState();
extern bool WaitAxisPauseStateByLock(long TimeOut);
extern void ResetLockOKAxisName();
extern bool WaitLockOKAxisIdle(long TimeOut);
extern long ANCDownBeforeMove(long Gantry);

extern bool IsTrayTypeFeeder(long FeederType);
extern bool IsLabelTypeFeeder(long FeederType);

extern void gPartDropLedOn(long CamTable);
extern void gPartDropProcess(long CamTable);
extern long gPartDropGetResult(long CamTable);
extern double GetManualSafetyDistanceSWLimit();
extern bool IsNearSWLimitMinus(CString strAxis);
extern bool IsNearSWLimitPlus(CString strAxis);
extern bool IsDangerSWLimit(CString strAxis);
extern bool IsAxisStatusJog(CString strAxis);

extern void SetHeightMeasureDone(long done);
extern long GetHeightMeasureDone();

extern void gBarcodeTriggerOn();
extern void gBarcodeTriggerOff();
extern CString gGetBarcodeString();
extern void gSetBarcodeString(CString str);
extern void gInitBarcodeString();

extern void EIMES_SendProtocol(long SubMsg2, long SubMsg3, CString strMsg);
extern void MES_SendBarcode(CString strBarcode);
extern void MES_SendBarcode(CString strBarcode, CString strBarcodeBlock1, CString strBarcodeBlock2);
extern void MES_SendBarcodeSNTMotive(CString strBarcode);
extern void MES_SendResult(long Result, CString strPath);
extern void MES_SendProtocol(long SubMsg2, long SubMsg3, CString strMsg);

extern void gSetBarcode(long Conv, CString strBarcode);
extern CString gGetBarcode(long Conv);
extern void gCopyBarcode(long SrcConv, long DestConv);

extern void SendConveyorBarcode(long Conv, CString strBarcode);
extern void SendOutBufferConveyorBarcode(CString strBarcode);
extern void SendNextMachineConveyorBarcode(CString strBarcode);

extern void gMainMESResultOK();
extern void gMainMESResultNG();

extern long GetCustomerSiteFromBarcodeSetting(BARCODE Barcode);

extern void gMainMES_Disconnect();
extern void ptRot(double* x, double* y, double rad);
extern void ptRotDeg(double* x, double* y, double deg);
extern long GetMaxBoardCount();
extern void SetMaxBoardCount(long MaxBoardCount);
extern bool IsLoadable();
extern void SetLoadable(bool Loadable);