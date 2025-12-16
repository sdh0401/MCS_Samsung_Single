#pragma once
#include "GlobalDefine.h"
#include "CApplicationTime.h"
#include "Thread.h"

class CPowerFeederControl : public CThread
{
public:
	CPowerFeederControl();
	~CPowerFeederControl();
	void StartFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void SetInfoFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void RunFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void FeederAutoControl(FeederControlStep CalStep, int nSub1, int nSub2, int nSub3);
	FeederControlStep CheckFeederControlStep(CString strHostMsg);
	void SetStep(FeederControlStep nStep);
	FeederControlStep GetStep();
	long GetThreadID();
	long GetProdRunMode();
	void SetProdRunMode(long ProdRunMode);

private:
	FeederControlStep m_Step;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	long m_ProdRunMode;
};

extern CPowerFeederControl* gcPowerFeederControl;