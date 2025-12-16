#pragma once
#include "Thread.h"

class CDecoding1 : public CThread
{
public:
	CDecoding1();
	~CDecoding1();

private:
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	CString InitializeMachine(CString strHostMsg);
	CString PrepareToQuitMachine(CString strHostMsg);
	CString QuitMachine(CString strHostMsg);
	CString FeederRefill(CString strHostMsg);
	CString FeederRefillDone(CString strHostMsg);
	CString CheckInitalizedMachine(CString strHostMsg);
	void KillMySelf();
	CString ReceiveHeadSkipConfig(CString strHostMsg);
	void SetAxisSkip();
	CString ReceiveMES_Disconnect(CString strHostMsg);
	ThreadId_t m_id;
};

extern CDecoding1* gcDecoding1;