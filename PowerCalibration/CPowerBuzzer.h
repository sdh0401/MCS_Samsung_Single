#pragma once
#include "GlobalDefine.h"
//#include "Thread.h"
#include "CPowerThread.h"
#include "sqlite3.h"

class CPowerBuzzer : public CPowerThread
{
public:
	CPowerBuzzer();
	~CPowerBuzzer();
    int GetLastSafetyAlarmCode();
    int SetLastSafetyAlarmCode(const int& safetyAlarmCode);
    //LONG_MAX (=2147483647) seconds is about 68.049 years.
    static constexpr long GlobalBuzzerTimeInSeconds = LONG_MAX;
	//long GetThreadID();
	void Run();
	static UINT BuzzerControl(LPVOID wParam);
	long SetBuzzerOnTime(long BuzzerOnTime);
	long GetBuzzerOnTime();
	BuzzerStep GetStep();
	void SetStep(BuzzerStep BuzzerStep);
	void SetBuzzserNoUse(bool nouse);
private:
    int _lastSafetyAlarmCode;
	void SetBuzzerStop(bool BuzzerStop);
	bool GetBuzzerStop();
	void BuzzerOn();
	void BuzzerOff();
	//virtual BOOL OnTask();
	//virtual BOOL OnTask(LPVOID lpv);
	//ThreadId_t m_id;
	bool m_bBuzzerStop;
	long m_BuzzerOnTime;
	long m_ShowID;
	BuzzerStep m_BuzzerStep;
	bool m_buzzserNoUse;
};

extern CPowerBuzzer* gcPowerBuzzer;