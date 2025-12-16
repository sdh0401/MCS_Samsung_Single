#pragma once
#include "GlobalDefine.h"

class CPick
{
public:
	CPick(long Gantry);
	~CPick();
	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	long GetPickupHeadNo(long PickOrd);
	long GetMaxPickOrder();
	void SetMaxPickOrder(long MaxPickOrd);
	long GetFdNoFromPickOrder(long PickOrd);
	long GetPickDelayFromFdNo(long FdNo);
	Ratio_XYRZ GetRatioFromFdNo(long FdNo);
	Point_XYRZ GetPickOffsetFromFdNo(long FdNo);
	long GetReadyNoFromFeederNo(long FeederNo);
	long GetReleaseNoFromFeederNo(long FeederNo);
	long GetReadyTimeOutFromFeederNo(long FeederNo);
	long GetReadyWaitDelayFromFeederNo(long FeederNo);
	long GetReadyTimeOutEmpty(long FeederNo);
	long SetReadyTimeOutEmpty(long FeederNo, long ReadyTimeOutEmpty);
	long GetReadyTimeOutEmptyByHeadNo(long HeadNo);
	long SetReadyTimeOutEmptyByHeadNo(long HeadNo, long ReadyTimeOutEmpty);
	long ClearReadyTimeOutEmptyByHeadNo(long HeadNo);
	bool GetSendEmpty(long FeederNo);
	long SetSendEmpty(long FeederNo, bool bSendEmpty);
	long SetEmptyError(long InsertOrd, long ReadyTimeOutEmpty);
	double GetComponentHeight(long FeederNo);
	double GetComponentLeadHeight(long FeederNo);
	TwoStepMotion GetTwoStepPick(long FeederNo);
	double GetMaxComponentHeight();
	double GetPickupZStandBy();
	long GetFeederReadyIOType(long FeederNo);
	TRAY_INFO GetTrayInfo(long FeederNo);
	long GetFeederType(long FeederNo);
	long GetTrayMaxPocket(long FeederNo);
	long GetTrayNowPocket(long FeederNo);
	long Run(bool bFirst, bool bManualRecog);
	long WaitGantry(long Gantry, long TimeOut);
	long FirstMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long MoveXy(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut, long UseSendAlarm);
	long MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut);
	Ratio_XYRZ GetMinRatio();
	Ratio_XYRZ GetPickRatio(long PickOrder);
	long SetTrayNowPocket(long FeederNo, long NowPocket);
	bool WaitTargetLevel(long HeadNo, long TargetLevel, long LessMore, long WaitTime);
	//void PartDropLedOn(long CamTable);
	//void PartDropProcess(long CamTable);
	//long PartDropGetResult(long CamTable);
	long GetInsertNoFromInsertOrder(long insertOrd);
	long GetRecognitionTable(long insertNo);
	void SetFeederEmptyDisplay(long FeederNo, bool set);
	bool GetFeederEmptyDisplay(long FeederNo);
	DIVIDE_INSPECT GetDivideInspect(long FeederNo);
	Ratio_XYRZ GetMinRatioLastPickData();
private:
	long m_Gantry;
	long m_MaxPickOrder;
	long m_ProdRunMode;
};

extern CPick* gcPick;
