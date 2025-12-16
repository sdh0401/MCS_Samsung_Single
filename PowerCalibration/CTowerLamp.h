#pragma once
#include "Thread.h"
class CTowerLamp : public CThread
{
public:
	CTowerLamp();
	~CTowerLamp();
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	void Alarm();

private:
	ThreadId_t m_id;
	void Normal();
	void Empty();
	void Run(long Msg2);
	void WaitPcb(long Msg2);
	void RedOn();
	void YelOn();
	void YelOn(long Msg2);
	void GrnOn();
	void GrnOn(long Msg2);
	void AllOff();
	long m_EntryFree, m_WorkFree, m_ExitFree;
	HANDLE m_Lock;
	signed m_OldLampMsg;

};

extern CTowerLamp* gcTowerLamp;