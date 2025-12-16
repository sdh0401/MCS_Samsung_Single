#pragma once
#include "GlobalDefine.h"
#include "CApplicationTime.h"
#include "Thread.h"

class CPowerConveyorControl : public CThread
{
public:
	CPowerConveyorControl();
	~CPowerConveyorControl();
	void StartConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void SetPcbInfoConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StartFreeTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopFreeTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StartLoadingTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void EndLoadingTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StartLineOfBalance(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void EndLineOfBalance(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	ULONGLONG GetLoadingTime();
	ULONGLONG GetLineOfBalance();
	void RunConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void EndConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void ConveyorAutoControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void SetPcbInfoManualConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void RunConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void StopConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void EndConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3);
	void ConveyorManualControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3);
	ConveyorCtrlStep CheckCalibrationStep(CString strHostMsg);
	void SetStep(ConveyorCtrlStep nStep);
	ConveyorCtrlStep GetStep();
	long GetThreadID();
	void SetEntryPcbReady(bool bReady);
	void SetWorkPcbReady(bool bReady);
	void SetWorkPcbOut(bool bReady);
	bool IsEntryPcbReady(long Conveyor);
	bool IsWorkPcbReady(long Conveyor);
	bool IsWorkPcbOut(long Conveyor);
	void SetInsertDone(long InsertDone);
	long GetInsertDone(long Conveyor);
	void SetPcbOutDone(long OutDone);
	long GetPcbOutDone(long Conveyor);
	bool IsStopperUp(long Conveyor);
	bool IsStopperDn(long Conveyor);
	long GetProdRunMode();
	long GetConveyorRunMode();
	void SetConveyorRunMode(long ProdRunMode, long ConveyorRunMode);
	bool IsEntryConveyorEnd();
	bool IsWorkConveyorEnd();
	bool IsExitConveyorEnd();
	bool IsAllConveyorEnd();
	bool IsAllConveyorBeltStop();
	long SetWorkConveyorStopMidDelay(long WorkStopMidDelay);
	long SetWorkConveyorStopLowDelay(long WorkStopLowDelay);
	long SetProfileLow(double MaxVel, double Acc, double Dec);
	long SetProfileMid(double MaxVel, double Acc, double Dec);
	long SetProfileHigh(double MaxVel, double Acc, double Dec);
	long SetConveyorSpeed(long Conv, long In, long Out);
	long GetConveyorSpeedPrevIn(long Conv);
	long GetConveyorSpeedNextOut(long Conv);
	long SetPcbTransferTimeOut(long ConvTable, long Conv, long LoadingTimeOut, long UnloadingTimeOut);
	void SetHeightMeasureDone(long done);
	long GetHeightMeasureDone();

private:
	ConveyorCtrlStep m_Step;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	long m_EntryPcbCount, m_WorkPcbCount, m_ExitPcbCount;
	long m_ProdRunMode;
	long m_ConveyorRunMode;
	ULONGLONG m_EntryLoadingStartTimeGet;
	ULONGLONG m_WorkLoadingElapsed;
	ULONGLONG m_LineOfBalanceTimeGet;
	ULONGLONG m_LineOfBalanceElapsed;
	JogInfo m_HighSpd, m_MidSpd, m_LowSpd;
};

extern CPowerConveyorControl* gcPowerConveyorControl;