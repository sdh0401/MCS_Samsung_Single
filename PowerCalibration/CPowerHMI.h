#pragma once
#include "Thread.h"
class CPowerHMI : public CThread
{
public:
	CPowerHMI();
	~CPowerHMI();
private:
	bool SendCommand(long Sub1, long Sub2, long Sub3, CString strCmd);
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
};

extern CPowerHMI* gcCPowerHMI;