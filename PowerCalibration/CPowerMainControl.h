#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"
#include "CApplicationTime.h"
#include "Thread.h"

class CPowerMainControl : public CThread
{
public:
	CPowerMainControl();
	~CPowerMainControl();
	void StartMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void PrepareMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void RunMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void FeederRefillControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void FeederRefillDoneControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void MoveStandByControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	MainRunCtrlStep CheckMainControlStep(CString strHostMsg);
	void SetStep(MainRunCtrlStep nStep);
	MainRunCtrlStep GetStep();
	void MainRunAutoControl(MainRunCtrlStep CalStep, int nSub1, int nSub2, int nSub3);
	long GetThreadID();
	long GetProdRunMode();
	long GetConveyorRunMode();
	void SetProdRunMode(long ProdRunMode, long ConveyorRunMode);
	void GetResultMES_OK(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void GetResultMES_NG(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void GetMES_Disconnect(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);

private:
	MainRunCtrlStep m_Step;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	long m_ProdRunMode;
	long m_ConveyorRunMode;
};

extern CPowerMainControl* gcPowerMainControl;