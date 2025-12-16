#pragma once
#include "GlobalDefine.h"

class CInsert
{
public:
	CInsert(long Gantry);
	~CInsert();
	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	long GetMaxInsertOrder();
	long Run(const bool& manualMode = false);
	long GetFdNoFromInsertOrder(long insertOrd);
	long GetHeadNoFromInsertOrder(long insertOrd);
	Ratio_XYRZ GetComponentRatioByFdNo(long FdNo);
	double GetVAAngleFromFdNo(long insertOrd);
	double GetComponentHeight(long FdNo);
	double GetComponentLeadHeight(long FdNo);
	Point_XYRZ GetInsertPoint(long insertNo);
	long GetBlowDelayFromInsertOrder(long FdNo);
	long GetReleaseDelayFromInsertOrder(long FdNo);
	ORIGIN GetOrigin();
	long GetInsertNoFromInsertOrder(long insertOrd);
	double GetPickupZStandBy();
	double GetMaxComponentHeight();
	long GetReadyTimeOutEmpty(long FeederNo);
	long GetReadyTimeOutEmptyByHeadNo(long HeadNo);
	long GetRecognitionTable(long insertNo);
	TwoStepMotion GetTwoStepInsert(long FeederNo);
	TwoStepMotion GetTwoStepInsertUp(long FeederNo);
	double GetInsertZOffset(long FeederNo);
	long GetBoardNo();
	long GetAvoidCount(long insertNo);
	long WaitGantry(long Gantry, long TimeOut);
	long MoveZUpBeforeInsert(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZUpBeforeInsert(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut, long UseSendAlarm);
	long MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	Ratio_XYRZ GetMinRatio();
	Ratio_XYRZ GetInsertRatio(long InsertOrder);
	//void PartDropLedOn(long CamTable);
	//void PartDropProcess(long CamTable);
	//long PartDropGetResult(long CamTable);
	void SetPartDrop(bool set);
	bool GetPartDrop();
private:
	long m_Gantry;
	long m_MaxInsertOrder;
	long m_ProdRunMode;
	bool m_ExePartDrop;
	long m_InsertRetryRcv;
};

extern CInsert* gcInsert;
