#pragma once
#include "GlobalDefine.h"

class CAvoidMotion
{

public:
	CAvoidMotion(long Gantry);
	~CAvoidMotion();
	long Run(long insertNo, long OrderNo, long ManualMove);
	CString MoveAvoidSequenceGantry(CString strHostMsg);
	CString MoveAvoidGantry(CString strHostMsg);
	CString TeachAvoidPosition(CString strHostMsg);
	ORIGIN GetOrigin();
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveStandBy();
	void SetRatio(Ratio_XYRZ ratio);
	void SetDelay(long delay);
	long GetTable();
private:
	long m_Gantry;
};

extern CAvoidMotion* gcAvoidMotion;
