#pragma once
#include "CPowerThread.h"

class CTorqueMonitor : public CPowerThread
{
public:
	CTorqueMonitor();
	~CTorqueMonitor();
	void InitializeValue();
	void Run();
	static UINT TorqueMonitorControl(LPVOID wParam);
	void SetZAxis(CString strZAxis);
	CString GetZAxis();
	long GetZAxisMap();
	TorqueMonitorStep GetStep();
	void SetStep(TorqueMonitorStep BuzzerStep);
	void SetPosition(long index, double Position);
	void SetTorque(long index, double Torque);
	void SetVelocity(long index, double Velocity);
	double GetPosition(long index);
	double GetTorque(long index);
	double GetVelocity(long index);
	void SetMaxCount(long MaxCount);
	long GetMaxCount();
	long ShowValue();
private:
	long m_ShowID;
	CString m_StrZAxis;
	TorqueMonitorStep m_TorqueMonitorStep;
	double m_Position[MAX_BUFFER_SIZE];
	double m_Veloctiy[MAX_BUFFER_SIZE];
	double m_Torque[MAX_BUFFER_SIZE];
	long m_MaxCount;
};

extern CTorqueMonitor* gcTorqueMonitor;