#pragma once
#include "GlobalDefine.h"

class CRecognitionRetry
{
public:
	CRecognitionRetry(long Gantry);
	~CRecognitionRetry();
	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	long Run(bool bManualRecog);
	long GetMaxInsertOrder();
	long GetFdNoFromInsertOrder(long insertOrd);
	long GetHeadNoFromInsertOrder(long insertOrd);
	Ratio_XYRZ GetRatioByFdNo(long FdNo);
	double GetVAAngleFromFdNo(long insertOrd);
	double GetComponentHeight(long FdNo);
	double GetComponentLeadHeight(long FdNo);
	double GetVAOffsetHeight(long FdNo);
	MODULE_LED GetLed(long FdNo);
	long GetManualCompensationUse();
	Point_XYRZ GetManualVisionResult();
	long GetVisionErrorEmpty(long FeederNo);
	long ClearVisionErrorEmpty(long FeederNo);
	long SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty);
	long SetVisionError(long InsertOrd, long VisionError);
	long GetEmptyError(long InsertOrd);
	bool GetSendEmpty(long FeederNo);
	long SetSendEmpty(long FeederNo, bool bSendEmpty);
	long GetReadyTimeOutEmpty(long FeederNo);
	long WaitGantry(long Gantry, long TimeOut);
	long RecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut);
	long ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut);
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	long MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long MoveOneZDn(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long MoveSomeZDn(long Gantry, SomeTarget Target);
	Ratio_XYRZ GetMinRatio();
	long GetLaserControl(long FeederNo);
	long GetCatchDelay(long FeederNo);
	long GetMaxRetryCount(long FeederNo);
	MODULE_LED GetMaxRetryLed(long FeederNo, long RetryLed);
	DIVIDE_INSPECT GetDivideInspect(long FeederNo);
	FORMING_COMPONENT GetForming(long FeederNo);
private:
	long m_Gantry;
	long m_MaxInsertOrder;
	long m_ProdRunMode;
	double m_VAAngleOffset[MAXUSEDHEADNO];
	long ClearManualVisionOffset();
	long SetManualVisionOffset(long insertOrd, double OffsetAngle);
	double GetManualVisionOffset(long insertOrd);
	long GetFeederNoFromHeadNo(long HeadNo);
	long GetInsertOrderFromHeadNo(long HeadNo);
	long GetRecognitionTableFromHeadNo(long HeadNo);
	long GetReadyTimeOutEmptyByHeadNo(long HeadNo);
	long SetFeederNoFromHeadNo(long HeadNo, long FeederNo);	
	long SetInsertOrderFromHeadNo(long HeadNo, long InsertOrder);
	long SetRecognitionTableFromHeadNo(long HeadNo, long RecogTable);
	long GetInsertNoFromInsertOrder(long insertOrd);
	long GetRecognitionTable(long insertNo);
	long m_FeederNo[MAXUSEDHEADNO];
	long m_InsertOrder[MAXUSEDHEADNO];
	long m_RecognitionTable[MAXUSEDHEADNO];
};

extern CRecognitionRetry* gcRecognitionRetry;
