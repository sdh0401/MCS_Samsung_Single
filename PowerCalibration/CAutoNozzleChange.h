#pragma once
#include "GlobalDefine.h"

class CAutoNozzleChange
{
public:
	CAutoNozzleChange();
	~CAutoNozzleChange();
	long GetTable();
	void InitANC();
	//bool GetANCHolePosition(long Base, long HoleNo, Point_XYRZ* ptHole);
	bool GetANCHolePosition(long HoleNo, Point_XYRZ* ptHole);
	long NozzleRelease(long Gantry, long Head, bool Recovery);
	long NozzleHold(long Gantry, long HeadNo, long NozzleNo, bool Recovery);
	//long GetIONum_OUT_CLAMP_UNLOCK(long Base);
	//long GetIONum_IN_CLAMP_LOCK(long Base);
	//long GetIONum_IN_CLAMP_UNLOCK(long Base);
	long GetBaseFromNozzleNo(long Nozzle);
	long GetMaxInsertOrder();
	long GetNozzleNoFromInsertNo(long InsertNo);
	long GetInsertNoFromInsertOrder(long insertOrd);
	long GetHeadNoFromInsertOrder(long insertOrd);
	long GetNozzleNoFromANCPrepare(long Gantry, long HeadNo);
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZ(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZ(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long Run();
	//long RunOptmize();
	long RunPrepare();
	void ActBaseUpDown(long UpDown);
	long GetBaseUpDown(long UpDown, long Time);
	void ActBaseLock(long LockUnlock);
	long GetBaseLock(long LockUnlock, long Time);
	bool GetUseANC();
	bool IsUpDownType();
private:
	bool m_SkipIOCheck;
	long m_Gantry;
};

extern CAutoNozzleChange* gcCAutoNozzleChange;