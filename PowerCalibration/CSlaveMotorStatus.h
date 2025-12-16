#pragma once
#include "CPowerThread.h"
#include "Wmx3MotorDefine.h"

class CSlaveMotorStatus : public CPowerThread
{
public:
	CSlaveMotorStatus();
	CSlaveMotorStatus(HANDLE h_Terminate);
	CSlaveMotorStatus(HANDLE h_Terminate, HANDLE h_HostMutex);
	CSlaveMotorStatus(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	~CSlaveMotorStatus();
	void InitialMemory();
	static UINT StartSlaveMotorStatus(LPVOID wParam);
	void Run();
	bool IsDiffMotorStatus(int iSelAxis = 99);
	bool GetWmx3MasterInvalidLicenseError();
	WMX3EngineState GetWmx3MasterEngineState();
	double GetWmx3MasterCycleTimeMilliseconds(long no);
	long long GetWmx3MasterCycleCounter(long no);
	bool GetWmx3MasterEmergencyStop();
	WMX3EmgStopLevel GetWmx3MasterEmgStopLevel();
	WMX3HomeState GetWmx3HomeState(int nIndex);
	bool GetWmx3AmpAlarm(int nIndex);
	int GetAmpAlarmCode(int nIndex);
	bool GetWmx3IsInpos(int nIndex);
	bool GetWmx3IsPosSet(int nAxis);
	bool GetWmx3IsDelayedPosSet(int nAxis);
	bool GetWmx3CheckServoOn(int nIndex);
	bool GetWmx3IsAxisIdle(int nIndex);
	bool GetWmx3IsAxisHoming(int nIndex);
	bool GetWmx3IsAxisHomeDone(int nIndex);
	bool GetWmx3IsAxisMotionComplete(int nIndex);
	bool GetWmx3IsAxisBusy(int nIndex);
	bool GetWmx3IsNegativeLimitSwitchOn(int nIndex);
	bool GetWmx3IsPositiveLimitSwitchOn(int nIndex);
	bool GetWmx3IsNegativeVirtualLimitSwitchOn(int nIndex);
	bool GetWmx3IsPositiveVirtualLimitSwitchOn(int nIndex);
	bool GetWmx3IsHomeSwitchOn(int nIndex);
	double GetWmx3ActualTorque(int nAxis);
	double GetWmx3oneDCompensation(int nAxis);
	double GetWmx3twoDCompensation(int nAxis);
	WMX3AxisSyncMode GetAxisSyncMode(int nAxis);
	double GetWmx3SyncOffset(int nAxis);
	double GetWmx3SyncPhaseOffset(int nAxis);
	double GetWmx3HomeOffset(int nAxis);
	int GetWmx3EncoderFeedback(int nAxis);

private:
	void CopyMotorStatus();
	void CopyMasterStatus();
	WMX3_AXIS_STATUS m_MotionStatus[MAXAXISNO];
	WMX3MASTER_INFO m_MasterInfo;
	long m_ShowID;
};

extern CSlaveMotorStatus* gcSlaveMotorStatus;
