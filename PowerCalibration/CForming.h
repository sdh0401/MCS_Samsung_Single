#pragma once

#include "GlobalDefine.h"

class CForming
{

public:
	CForming(long Gantry);
	~CForming();
	long Run(long ManualMode, FORMING_COMPONENT Forming);
	ORIGIN GetOrigin();
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZUp(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveStandBy();
	void SetRatio(Ratio_XYRZ ratio);
	void SetDelay(long delay);
	long GetMaxInsertOrder();
	long GetTable();
	long GetHeadNoFromInsertOrder(long insertOrd);
	long GetFdNoFromInsertOrder(long insertOrd);
	double GetComponentHeight(long FdNo);
	double GetComponentLeadHeight(long FdNo);
	double GetMaxComponentHeight();
	Ratio_XYRZ GetComponentRatioByFdNo(long FdNo);
	FORMING_COMPONENT GetFormingComponentFromFeederNo(long FdNo);
	void SetFormingMotionDone(long insertOrd, bool bDone);
	long GetPickupErrorUse(long FeederNo);
	void AddPickupErrorCount(long FeederNo, long VisionError);
	void RemovePickupErrorCount(long FeederNo);
	long GetPickupErrorReferenceCount(long FeederNo);
	long GetPickupErrorAlarmCount(long FeederNo);
	long GetPickupErrorRawCount(long FeederNo);
	long GetPickupErrorCount(long FeederNo);

private:
	long m_Gantry;
};

extern CForming* gcForming;
