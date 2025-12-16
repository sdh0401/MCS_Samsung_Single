#pragma once
//#include "Thread.h"
#include "Cwmx3Init.h"
#include "Wmx3MotorDefine.h"
#include "CMessageQueue.h"
#include "CPowerClient.h"
#include "CApplicationTime.h"
using namespace std;
class Cwmx3Axis// : public CThread
{
public:
	Cwmx3Axis(bool bSimulation);
	~Cwmx3Axis(void);
	/*
	UTIL FUNCTION
	*/
	void SetAxisInformation(unsigned nSlaveId, unsigned nIndex);
	int GetAxisMap();
	bool IsSimulationMode();
	void SetAsSimulation();
	void SetResol(double dblResol);
	double GetResol(void);
	void SetUnResol(double dblUnResol);
	double GetUnResol(void);
	void SetLinearIntp(double LinearIntp);
	void SetForceOriginOffset(double ForceOffset);
	double GetForceOriginOffset();
	double GetLinearIntp();
	void SetRatio(double Ratio);
	double GetRatio(void);
	void Set2ndRatio(double Ratio);
	double Get2ndRatio(void);
	void InitializeRatio(void);
	void SetMovingDir(signed Dir);
	signed GetMovingDir();
	void SetAxisSlaveNo(unsigned SlaveNo);
	int GetAxisSlaveNo();

	void InitAxisData();
	bool IsConveyorAxis();
	bool IsConveyorBeltAxis();
	bool IsGantryAxis();
	bool IsYAxis();
	bool IsXAxis();
	bool IsTTFZAxis();
	bool IsTTFYAxis();
	bool IsTTFAxis(long TTFNumber);
	bool IsSlaveAxis();
	bool IsZAxis();
	bool IsRAxis();
	bool IsPusherZAxis();
	bool IsY1Axis(long Gantry);
	/*
	MOTION PROFILE FUNCTION
	*/
	Profile GetTeachBoxMoveProfile();
	void SetTeachBoxMoveProfile(Profile profile);
	Profile GetTeachBoxJogProfile();
	void SetTeachBoxJogProfile(Profile profile);
	Profile GetMoveProfile();
	Profile GetMoveProfile(double Dist);
	void SetMoveProfile(Profile profile);
	Motion::PosCommand GetProfileByRatio(Motion::PosCommand* Cmd);
	Motion::TriggerPosCommand GetProfileByRatioTrigger(Motion::PosCommand* Cmd);
	long GetShortDistanceLevel(double Dist);
	void SetShortDist(long index, double ShortDist);
	void SetShortDistVel(long index, double ShortVel);
	void SetShortDistAccDec(long index, double ShortAccDec);
	double GetShortDist(long index);
	double GetShortDistVel(long index);
	double GetShortDistAccDec(long index);
	int GetMotionParam(Config::MotionParam* pMotionParam);
	int SetMotionParam(Config::MotionParam* pMotionParam);
	void SetLinearIntplProfileCalcMode(long CalMode, bool apiWaitUntilMotionStart, double quickStopDec);
	
	/* 
	HOME FUNCTION 
	*/
	int SetFeedbackParam(Config::FeedbackParam* feedParam);
	int GetFeedbackParam(Config::FeedbackParam* feedParam);
	int SetHomeParam(Config::HomeParam* homeParam);
	int GetHomeParam(Config::HomeParam* pHomeParam);
	int SetSlaveHomeParam(Config::HomeParam* homeParam);
	int GetSlaveHomeParam(Config::HomeParam* pHomeParam);
	int StartHome();
	int ContinueHome();
	int CancelHome();
	int SetHomeDone(unsigned char value);
	int GetHomeData();
	/*
	LIMIT FUNCTION
	*/
	int SetHWLimitParam(Config::LimitParam* limitParam);
	int SetHWLimitParam(Config::LimitParam* CurLimitParam, WMX3AxisLimitSwitchType posLimitType, WMX3AxisLimitSwitchType negLimitType);
	int GetHWLimitParam(Config::LimitParam* pLimitParam);

	int SetVirtualLimitParam(Config::LimitParam limitParam);
	int SetVirtualLimitParam(Config::LimitParam CurLimitParam, WMX3AxisLimitSwitchType posLimitType, double positivePos, WMX3AxisLimitSwitchType negLimitType, double negativePos);
	int GetVirtualLimitParam(Config::LimitParam* pLimitParam);
	int ReadVirtualPositiveLimit(double* pLimit);
	int ReadVirtualNegativeLimit(double* nLimit);
	/*
	MOTION FUNCTION 
	*/
	int ServoOn();
	int SlaveServoOn();
	int ServoOff();
	int SlaveServoOff();
	int ClearAmpAlarm();
	int ClearSlaveAmpAlarm();
	int ClearAxisAlarm();
	int ClearSlaveAxisAlarm();
	int WaitMotion();
	int PauseMotion();
	int ResumeMotion();
	int StopMotion();
	int StopMotion(double deceleration);
	int SetMotorPosition(double dblPosition);
	double ReadMotorPosition();
	double ReadSlsaveMotorPosition();
	double ReadCommandPosition();
	double ReadProfileTargetPosition();
	double ReadSlaveCommandPosition();
	double ReadMotorVelocity();
	int SetPosSet(double posSet);
	int SetInPos(double posSet);
	int SetInPos2(double posSet);
	int SetInPos3(double posSet);
	int SetInPos4(double posSet);
	int SetInPos5(double posSet);
	int SetOneDelayedPosSet(double DelayedPosSetWidth, long ms);
	double ReadCommandVelocity();
	double ReadMotorActualPosition();
	int OverridePosition(double dblPosition, WMX3_AXIS_POSCOMMANDPROFILE profile);
	int OverrideVelocity(double dblVelocity, WMX3_AXIS_POSCOMMANDPROFILE profile);
	int OverrideProfile(WMX3_AXIS_POSCOMMANDPROFILE profile);
	int StartJog(Motion::JogCommand* jogCmd);
	int StartMove(Motion::PosCommand* moveCmd, double position);
	int StartMove(Motion::PosCommand* moveCmd);
	int StartSlaveMove(Motion::PosCommand* moveCmd, double position);
	int StartPos(Motion::PosCommand* posCmd);
	int StartPos(Motion::PosCommand* posCmd, double position);
	int StartSlavePos(Motion::PosCommand* posCmd, double position);
	int StartPosWithTriggerPos(Motion::PosCommand* posCmdposCmd, double position, double TriggerPos);
	int StartavsMotionX(double targetPos, double Velocity);
	int StartavsMotionY(double targetPos, double Velocity);
	///////////////////////////////////////////////////////////////////////////////////////////// Gantry Homing
	int SetSyncMasterSlave();
	///////////////////////////////////////////////////////////////////////////////////////////// 1D Compensation
	int Set1DCompensationData(PitchErrorCompensationData* pitchErrCompData);
	int Enable1D();
	int Disable1D();
	bool WaitIdle();
	///////////////////////////////////////////////////////////////////////////////////////////// 2D Compensation
	/*
	Axis Status FUNCTION 
	*/
	bool CheckAmpAlarm();
	bool CheckSlaveAmpAlarm();
	WMX3HomeState GetHomeState();	
	int CheckAmpAlarmCode();
	int CheckSlaveAmpAlarmCode();
	bool IsInpos();
	bool IsPosSet();
	bool IsDelayedPosSet();
	bool CheckServoOn();	
	bool CheckSlaveServoOn();
	bool IsAxisBusy();
	bool IsAxisIdle();
	bool IsSlaveAxisIdle(void);
	bool IsAxisHoming();
	bool IsAxisHomeDone();
	bool IsSyncGroupHomeDone(void);
	bool IsAxisMotionComplete();
	bool IsSlaveAxisMotionComplete(void);
	bool IsNegativeLimitSwitchOn();
	bool IsNegativeVirtualLimitSwitchOn();
	bool IsPositiveLimitSwitchOn();
	bool IsPositiveVirtualLimitSwitchOn();
	bool IsHomeSwitchOn();
	bool IsAxisJog();
	double ReadActualTorque();
	double ReadSlaveActualTorque();
	double Read1DCompensationData();
	double Read2DCompensationData();
	WMX3AxisSyncMode GetAxisSyncMode();
	double GetSyncOffset();
	double GetSyncPhaseOffset();
	double GetHomeOffset();
	int GetEncoderFeedBack();
	int GetSlaveEncoderFeedBack();
	double GetHomeDiffZPulse();
	double GetMaxTrqLimit(void);
	long SetMaxTrqLimit(double Torque);
	long StartWmx3Monitor(long BoardCount, long BlockNo, long InsertNo, CString Action);
	long SetTorqueMonitorFileName(CString strFileName);
	CString GetTorqueMonitorFileName();
	long StopWmx3Monitor();
	bool WaitStopWmx3Monitor(long timeOut);
	long ResetWmx3Monitor();
	long GetDateTime(wchar_t* datetime);
	long GetDateTimeMil(wchar_t* Time);
	long GetYear(wchar_t* year);
	long GetDate(wchar_t* date);
	bool _CreateDirectory(LPCWSTR lpszPath);
	void SetHomingMaxTimeOut(ULONGLONG MaxTimeOut);
	ULONGLONG GetHomingMaxTimeOut();
	void SetInitializeFail(bool bEnd);
	bool GetInitializeFail();
	bool GetCoreMotionAxisStatus(CoreMotionAxisStatus* status);

public:
	void Initialize(void);
	void SetInitializeEnd(bool bEnd);
	bool GetInitializeEnd();
	CMotionControlType GetMotionControlType();
	void SetMotionControlType(CMotionControlType motionControlType);
	CMotorType GetMotorType();
	void SetMotorType(CMotorType motorType);
	void SetAxisName(CString strAxisName);
	CString GetAxisName();
	CString GetSlaveAxisName();
	int GetAxisIndex();
	bool GetUseSlaveAxis();
	void SetUseSlaveAxis(bool bSlave);
	int GetSlaveAxisIndex();
	void SetSlaveAxisIndex(unsigned nSlave);
	int GetSlaveAxisSlaveID();
	bool Lock();
	bool Unlock();

	void SetAxisSkip(bool set);
	bool GetAxisSkip();
	void SetLastCommandPosition(double position);
	double GetLastCommandPosition();
	void SetLastCommandServoOn(bool on);
	bool GetLastCommandServoOn();

private:
	HANDLE m_CmdLock;
	int m_nAxisMap;				// Axis Hardware Mapping Number
	int m_nIndex;				// Axis Index
	int m_nSlaveAxis_Index;		// Axis Slave Axis Index
	CString m_AxisName;			// Axis String
	CString m_TorqueMonitorFileName;
	double m_Ratio;				// Axis Speed Ratio, Default:100
	double m_Ratio2nd;			// Axis Speed Ratio, Default:100
	signed m_MovingDir;			// Axis Moving Direction
	CMotorType m_nMotorType;	// Axis Motor Type
	CMotionControlType m_nMotionControlType; // Axis Motion Control Type
	int m_nSlaveNo;				// Slave ID
	bool m_bInitializeEnd;		// Axis Homing Flag
	bool m_bSimulation;			// Axis Simulation Flag
	bool m_bErrorStatus;		// Axis Error Flag
	bool m_bUseSlave;			// Axis Use Slave Flag
	int m_OpState;				// Axis Operation State
	double m_dblVirtualPlusLimit;	// Software + Limit
	double m_dblVirtualMinusLimit;	// Software - Limit
	double m_dblResol;			// Axis Motor Unresolution
	double m_dblUnResol;		// Axis Motor Resolution
	char m_ErrString[BUFSIZE];	// Axis Error String
	Profile m_TeachMoveProfile;	// Axis Relative Profile
	Profile m_TeachJogProfile;	// Axis Velocity Profile
	Profile m_MoveProfile;		// Axis Relative Profile
	double m_inPosWidth;		// Axis Inpositon value1
	double m_inPosWidth2;		// Axis Inpositon value2
	double m_inPosWidth3;		// Axis Inpositon value3
	double m_inPosWidth4;		// Axis Inpositon value4
	double m_inPosWidth5;		// Axis Inpositon value5
	double m_LinearIntp;		// Axis Linear interpolation position
	double m_ForceOriginOffset; // Axis Origin Offset

	double m_ShortDist[MAX_SHORT_DIST_LEVEL];
	double m_ShortDistVel[MAX_SHORT_DIST_LEVEL];
	double m_ShortDistAccDec[MAX_SHORT_DIST_LEVEL];

	ULONGLONG m_HomingMaxTimeOut; // 
	bool m_AxisSkip;
	bool m_LastCommandServoOn;
	double m_LastCommandPosition;
};

