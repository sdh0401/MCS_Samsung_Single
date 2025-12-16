#pragma once
#pragma once
#include "GlobalDefine.h"
#include "CApplicationTime.h"
#include "Thread.h"

class CPowerConveyorManualControl : public CThread
{
public:
	CPowerConveyorManualControl();
	~CPowerConveyorManualControl();
	void StartConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void SetPcbInfoManualConveyorLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void RunConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void ConveyorManualLocationControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3);
	ConveyorCtrlStep CheckCalibrationStep(CString strHostMsg);
	void SetStep(ConveyorCtrlStep nStep);
	ConveyorCtrlStep GetStep();
	long GetThreadID();
	long GetConveyorRunMode();
	void SetConveyorRunMode(long ConveyorRunMode);
	bool IsAllConveyorEnd();

private:
	ConveyorCtrlStep m_Step;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	long m_EntryPcbCount, m_WorkPcbCount, m_ExitPcbCount;
	long m_ProdRunMode;
	long m_ConveyorRunMode;
};

extern CPowerConveyorManualControl* gcPowerConveyorManualControl;