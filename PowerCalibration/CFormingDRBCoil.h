#pragma once

#include "GlobalDefine.h"

enum class FormingType
{
	Forming1st,
	Forming2nd
};

class CFormingDRBCoil
{

public:
	//CFormingDRBCoil(long Gantry, long OutputFormingClose, long InputStandbyFormingLock, long InputStandbyFormingUnlock, long InputFormingLock, long InputFormingUnlock);
	CFormingDRBCoil(long Gantry);
	~CFormingDRBCoil();
	long Run(long ManualMode, FORMING_COMPONENT Forming, FormingType Method);
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

	void FormingPrepare(FormingType Method);
	long FormingPrepareCheck(FormingType Method);
	long FormingStart(FormingType Method);
	long EmptyCheck();
private:
	long m_Gantry;

	long m_Out1stFormingLock;	// unlock 상태에서 헤드 내림 -> lock 동작으로 포밍 -> unlock 후 헤드 올림
	long m_Input1stFormingLock;
	long m_Input1stFormingUnlock;

	long m_Out2ndFormingUnlock;	// lock 상태에서 헤드 내림 ->  unlock 동작으로 1차 포밍 -> lock 동작으로 2차 포밍 -> lock 후 헤드 올림
	long m_Input2ndFormingLock;
	long m_Input2ndFormingUnlock;

	long m_InputPartExist;
};

extern CFormingDRBCoil* gcFormingDRBCoil;
