#pragma once
#include "CPowerThread.h"
#include "Wmx3MotorDefine.h"

class Cwmx3Motor : public CPowerThread
{
public:
	Cwmx3Motor(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	~Cwmx3Motor();
	static UINT StartWmx3MotorStatus(LPVOID wParam);
	void Run();
	int GetWmx3MotorStatus();
	int EStop();
	int EStopHigher();
	int ReleaseEStop();
	bool GetEmergencyStop();
	bool WaitEStopStatus(bool bStatus);
	WMX3EmgStopLevel GetEmergencyStopLevel();
	bool CopyWmx3MotorStatus(LPVOID wParam);
	bool CopyWmx3MasterStatus(LPVOID wParam);
private:
	long m_ShowID;
};

extern Cwmx3Motor* gcWmx3Motor;