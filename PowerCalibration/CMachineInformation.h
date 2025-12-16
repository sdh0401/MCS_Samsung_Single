#pragma once
#include "CPowerThread.h"
class CMachineInformation : public CPowerThread
{
public:
	CMachineInformation();
	~CMachineInformation();
	static UINT MonitoringMachineInformation(LPVOID wParam);
	void Run();
	
	void InitialMotorAlarmHistory();
	void InsertAlarmcode(CString strAxis, long alarmCode);
	void SaveAlarmCode();
	void SetBowlFeederOffTime(BOWLFEEDER_OFFTIME data);
	BOWLFEEDER_OFFTIME GetBowlFeederOffTime();
	void ExeBowlFeederOff();
private:
	long m_ShowID;
	std::deque<long> m_AlarmCodeCurrent;
	std::deque<long> m_AlarmCodeOld;
	BOWLFEEDER_OFFTIME m_BowlFeederOff;

};

extern CMachineInformation* gcMachineInformation;