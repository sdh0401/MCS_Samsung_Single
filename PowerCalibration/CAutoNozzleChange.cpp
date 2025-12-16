#include "pch.h"
#include "CAutoNozzleChange.h"
#include "AxisInformation.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "Trace.h"
#include "CStep.h"
#include "GlobalIODefine.h"
#include "CRunFile.h"
#include "CMachineConfig.h"
#include "CMachineFileDB.h"

CAutoNozzleChange* gcCAutoNozzleChange;
CAutoNozzleChange::CAutoNozzleChange()
{
	m_SkipIOCheck = 0;
}

CAutoNozzleChange::~CAutoNozzleChange()
{

}

long CAutoNozzleChange::GetTable()
{
	return FRONT_STAGE;
}

void CAutoNozzleChange::InitANC()
{
	if (gCMachineConfig->IsExistANCData() == true)
	{
		TRACE(_T("[PWR] InitANC Skip."));

		return;
	}

	gcPowerCalibrationData->InitANCHoleCadPosition();
	gcPowerCalibrationData->CalculateANCHoleRealXYRZ(FRONT_STAGE);
	gcPowerCalibrationData->CalculateANCHoleRealXYRZ(REAR_STAGE);

	Point_XYRZ pt;
	for (long i = 1; i <= MAX_ANC_HOLE; i++)
	{
		GetANCHolePosition(i, &pt);

		TRACE(_T("[PWR] InitANC Front Hole:%d XYRZ %.3f %.3f %.3f %.3f"), i, pt.x, pt.y, pt.r, pt.z);
	}

	for (long i = REAR_ANC_1ST; i < REAR_ANC_1ST + MAX_ANC_HOLE; i++)
	{
		GetANCHolePosition(i, &pt);

		TRACE(_T("[PWR] InitANC Rear Hole:%d XYRZ %.3f %.3f %.3f %.3f"), i, pt.x, pt.y, pt.r, pt.z);
	}
}

//bool CAutoNozzleChange::GetANCHolePosition(long Base, long HoleNo, Point_XYRZ* ptHole)
//{
//	bool Err;
//
//	Err = gcPowerCalibrationData->GetANCHoleRealPosition(Base, HoleNo, ptHole);
//	return true;
//}

bool CAutoNozzleChange::GetANCHolePosition(long HoleNo, Point_XYRZ* ptHole)
{
	bool Err;
	long Base;

	if (1 <= HoleNo && HoleNo <= MAX_ANC_HOLE)
	{
		Base = FRONT_STAGE;
	}
	else
	{
		Base = REAR_STAGE;
	}

	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		Err = gcPowerCalibrationData->GetANCHoleRealPosition(HoleNo, ptHole);

	}
	else if (gCMachineConfig->IsExistANCData() == true)
	{
		ANC_HOLE_STRUCT holeCal;
		ANC_HOLE_STRUCT holeConfig;

		if (gCMachineConfig->GetConfigANCHole(Base, HoleNo, &holeConfig) == true && gCMachineConfig->GetCalANCHole(Base, HoleNo, &holeCal) == true)
		{
			*ptHole = holeCal.pt;

			if (holeConfig.Use == true)
			{
				return true;
			}
			else
			{
				TRACE(_T("[PWR] ANC base %d hole %d can not use"), Base, HoleNo);
				return false;
			}
		}
		else
		{
			TRACE(_T("[PWR] ANC base %d hole %d position data is empty"), Base, HoleNo);
			return false;
		}
	}
	else
	{
		Err = gcPowerCalibrationData->GetANCHoleRealPosition(HoleNo, ptHole);
	}
	return Err;
}

//long CAutoNozzleChange::GetIONum_OUT_CLAMP_UNLOCK(long Base)
//{
//	return 1;
//}
//long CAutoNozzleChange::GetIONum_IN_CLAMP_LOCK(long Base)
//{
//	return 1;
//}
//long CAutoNozzleChange::GetIONum_IN_CLAMP_UNLOCK(long Base)
//{
//	return 1;
//}
long CAutoNozzleChange::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	SetMoveCheckANCDown(Gantry, false);
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	SetMoveCheckANCDown(Gantry, true);
	return Err;
}

long CAutoNozzleChange::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}
long CAutoNozzleChange::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CAutoNozzleChange::MoveZ(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}
long CAutoNozzleChange::WaitZ(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}
long CAutoNozzleChange::GetBaseFromNozzleNo(long NozzleNo)
{
	if (1 <= NozzleNo && NozzleNo <= MAX_ANC_HOLE)
	{
		return FRONT_STAGE;
	}
	else 
	{
		return REAR_STAGE;
	}

	TRACE(_T("[PWR] Invalid Nozzle Number. Nzl:%d "), NozzleNo);
	return -1;
}

long CAutoNozzleChange::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}
long CAutoNozzleChange::GetNozzleNoFromInsertNo(long InsertNo)
{
	long Ret;
	Ret = gcStep->GetNozzleNoFromInsertNo(InsertNo);

	return Ret;
}

long CAutoNozzleChange::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CAutoNozzleChange::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CAutoNozzleChange::GetNozzleNoFromANCPrepare(long Gantry, long HeadNo)
{
	long NozzleNo = 0;
	NozzleNo = gcStep->GetANCPrepare(Gantry, HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetNozzleNoFromANCPrepare:%d HeadNo:%d Nozzle:%d\n"), Gantry, HeadNo, NozzleNo);
	}
	return NozzleNo;
}

long CAutoNozzleChange::NozzleRelease(long Gantry, long HeadNo, bool Recovery)
{
	long Err = NO_ERR;
	long Base, NozzleNo, target;
	long IO_InClampLock, IO_InClampUnock,IO_OutClampUnlock;
	Point_XYRZ ptANCxyrz;
	Point_XY ptANCxy;

	double InposXY = 0.10, InposR = 1.000, InposZ = 0.050, RatioXY = 1.0, RatioZ = 1.0, RatioR = 1.0;
	double UpposZ, AncZTorqueLimit = 120.0;
	long MsXy = TIME30MS, TimeOut = TIME5000MS;
	long ErrCode;

	ULONGLONG TimeANC,TimeUnLock, TimeAct;
	
	CString strZAxis, strRAxis;
	NOZZLE Nozzle;
	ANC_STRUCT ancXML;

	if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] NozzleRelease GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
		return STOP_NOW;
	}
	if (GetMachineState() == STATE_STOPNOW)
	{
		TRACE(_T("[PWR] NozzleRelease GetMachineState(%d)\n"), GetMachineState());
		return STOP_NOW;
	}

	TimeANC = _time_get();

	if (Gantry == FRONT_GANTRY)
	{
		ErrCode = ANC_HOLD_FZ1 + HeadNo - 1;
	}
	else
	{
		ErrCode = ANC_HOLD_RZ1 + HeadNo - 1;
	}

	if (TBL_HEAD1 <= HeadNo && HeadNo <= MAXUSEDHEADNO)
	{

	}
	else
	{
		TRACE(_T("[PWR] Invalid NozzleRelease Head:%d"), HeadNo);
		return ANC_HOLD_FZ1;
	}

	NozzleNo = GetGlobalNozzleNo(HeadNo);
	Nozzle = GetGlobalNozzleInformation(NozzleNo);

	if (NozzleNo == 0)
	{
		TRACE(_T("[PWR] Already Nozzle Empty. %s Nzl:%d "), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
		return NO_ERR;
	}

	Base = GetBaseFromNozzleNo(NozzleNo);

	if (gCMachineConfig->GetConfigANC(Base, &ancXML) == true)
	{
		AncZTorqueLimit = ancXML.HeadTorque;
	}

	if (Base == FRONT_STAGE)
	{
		IO_InClampLock = IN_FANC_CLAMP_LOCK;
		IO_InClampUnock = IN_FANC_CLAMP_UNLOCK;
		IO_OutClampUnlock = OUT_FANC_CLAMP_UNLOCK;
	}
	else
	{
		IO_InClampLock = IN_RANC_CLAMP_LOCK;
		IO_InClampUnock = IN_RANC_CLAMP_UNLOCK;
		IO_OutClampUnlock = OUT_RANC_CLAMP_UNLOCK;
	}

	TRACE(_T("[PWR] ANC Release %s Nzl:%d Unlock & Z Standby"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);

	Err = MoveZStandy(Gantry, GetStandByZ(Gantry), 1.0);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	
	if (GetANCHolePosition(NozzleNo, &ptANCxyrz) != true)
	{
		return ErrCode;
	}

	TRACE(_T("[PWR] ANC Release %s Nzl:%d XYRZ %.3f,%.3f,%.3f,%.3f"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo, ptANCxyrz.x, ptANCxyrz.y, ptANCxyrz.r, ptANCxyrz.z);

	ptANCxy.x = ptANCxyrz.x;
	ptANCxy.y = ptANCxyrz.y;
	target = HeadNo;

	strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);

	ActBaseUpDown(BASE_UP);

	TRACE(_T("[PWR] ANC Release %s Nzl:%d MoveStart"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);

	Err = MoveR(strRAxis, RatioR, TimeOut, ptANCxyrz.r, InposR, MsXy, false);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	Err = MoveXY(Gantry, target, ptANCxy, RatioXY, InposXY, MsXy, TimeOut);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}

	TimeAct = _time_get();
	while (1)
	{
		Err = GetBaseUpDown(BASE_UP, TIME100MS);
		if (Err == NO_ERR)
		{
			break;
		}
		else if (_time_elapsed(TimeAct) > TIME3000MS)
		{
			TRACE(_T("[PWR] ANC Release %s Nzl:%d BaseUp TimeOut"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			return ErrCode;
		}

		ThreadSleep(TIME1MS);
	}


	TimeUnLock = _time_get();

	ActBaseLock(BASE_UNLOCK);

	while (1)
	{
		if (m_SkipIOCheck)
		{
			TRACE(_T("[PWR] ANC Release IO Skip %s Nzl:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			break;
		}

		Err = GetBaseLock(BASE_UNLOCK, TIME500MS);
		if (Err == NO_ERR)
		{
			break;
		}
		else if(_time_elapsed(TimeUnLock) > TIME3000MS)
		{
			TRACE(_T("[PWR] ANC Release %s Nzl:%d Unlock TimeOut"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			return ErrCode;
		}
		
		ThreadSleep(TIME1MS);
	}

	Err = WaitR(strRAxis, ptANCxyrz.r, InposR, TimeOut);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	TRACE(_T("[PWR] ANC Release %s Nzl:%d Z Down"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
	Err = RemoveHeadTorqueLimitEvent(Gantry, HeadNo);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	Err = SetHeadTorqueLimitEvent(Gantry, HeadNo, AncZTorqueLimit);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	Err = MoveZ(strZAxis, RatioZ, TimeOut, ptANCxyrz.z, InposZ, MsXy, true);
	if (Err != NO_ERR)
	{
		InitialHeadTorqueLimit(Gantry, HeadNo);
		return ErrCode;
	}

	TRACE(_T("[PWR] ANC Release %s Nzl:%d Lock"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);

	TimeAct = _time_get();
	ActBaseLock(BASE_LOCK);
	while (1)
	{
		if (m_SkipIOCheck)
		{
			TRACE(_T("[PWR] ANC Release IO Skip %s Nzl:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			break;
		}

		Err = GetBaseLock(BASE_LOCK, TIME500MS);
		if (Err == NO_ERR)
		{
			break;
		}
		else if (_time_elapsed(TimeAct) > TIME3000MS)
		{
			TRACE(_T("[PWR] ANC Release %s Nzl:%d Unlock TimeOut"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			InitialHeadTorqueLimit(Gantry, HeadNo);
			return ErrCode;
		}

		ThreadSleep(TIME1MS);
	}

	
	UpposZ = GetStandByZ(Gantry);

	Err = MoveZ(strZAxis, RatioZ, TimeOut, UpposZ, InposZ, MsXy, false);
	if (Err != NO_ERR)
	{
		InitialHeadTorqueLimit(Gantry, HeadNo);
		return ErrCode;
	}

	SetGlobalNozzleNo(HeadNo, 0);

	Err = WaitZ(strZAxis, UpposZ, InposZ, TimeOut);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}

	if (Recovery == true)
	{
		ActBaseLock(BASE_LOCK);
		ThreadSleep(TIME100MS);
		ActBaseUpDown(BASE_DOWN);
	}

	TRACE(_T("[PWR] ANC Release %s Nzl:%d Complete. %d[ms]"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo, _time_elapsed(TimeANC));

	return Err;
}



long CAutoNozzleChange::NozzleHold(long Gantry, long HeadNo, long NozzleNo, bool Recovery)
{
	long Err = NO_ERR, ErrZ = NO_ERR;
	long Base, target, NozzleNoCurrent;
	long IO_InClampLock, IO_InClampUnock, IO_OutClampUnlock;
	Point_XYRZ ptANCxyrz;
	Point_XY ptANCxy;

	double InposXY = 0.10, InposR = 1.000, InposZ = 0.050, RatioXY = 1.0, RatioZ = 1.0, RatioR = 1.0;
	double UpposZ, AncZTorqueLimit = 120.0;
	long MsXy = TIME30MS, TimeOut = TIME5000MS;
	long ErrCode;

	ULONGLONG TimeANC, TimeLock, TimeAct;

	CString strZAxis, strRAxis;
	NOZZLE Nozzle;
	ANC_STRUCT ancXML;

	if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] NozzleHold GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
		return STOP_NOW;
	}
	if (GetMachineState() == STATE_STOPNOW)
	{
		TRACE(_T("[PWR] NozzleHold GetMachineState(%d)\n"), GetMachineState());
		return STOP_NOW;
	}


	if (Gantry == FRONT_GANTRY)
	{
		ErrCode = ANC_HOLD_FZ1 + HeadNo - 1;
	}
	else
	{
		ErrCode = ANC_HOLD_RZ1 + HeadNo - 1;
	}

	TimeANC = _time_get();
	if (TBL_HEAD1 <= HeadNo && HeadNo <= MAXUSEDHEADNO)
	{

	}
	else
	{
		TRACE(_T("[PWR] Invalid NozzleHold Head:%d)"), HeadNo);
		return ANC_HOLD_FZ1;
	}

	NozzleNoCurrent = GetGlobalNozzleNo(HeadNo);
	Nozzle = GetGlobalNozzleInformation(NozzleNo);

	if (NozzleNoCurrent != 0)
	{
		TRACE(_T("[PWR] Already Nozzle Hold. %s Nzl:%d "), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoCurrent);

		Err = NozzleRelease(Gantry, HeadNo, Recovery);
		if (Err != NO_ERR)
		{
			return Err;
		}
	}

	Base = GetBaseFromNozzleNo(NozzleNo);


	if (gCMachineConfig->GetConfigANC(Base, &ancXML) == true)
	{
		AncZTorqueLimit = ancXML.HeadTorque;
	}


	if (Base == FRONT_STAGE)
	{
		IO_InClampLock = IN_FANC_CLAMP_LOCK;
		IO_InClampUnock = IN_FANC_CLAMP_UNLOCK;
		IO_OutClampUnlock = OUT_FANC_CLAMP_UNLOCK;
	}
	else
	{
		IO_InClampLock = IN_RANC_CLAMP_LOCK;
		IO_InClampUnock = IN_RANC_CLAMP_UNLOCK;
		IO_OutClampUnlock = OUT_RANC_CLAMP_UNLOCK;
	}

	TRACE(_T("[PWR] ANC Hold %s Nzl:%d lock & Z Standby"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);

	Err = MoveZStandy(Gantry, GetStandByZ(Gantry), 1.0);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}

	if (GetANCHolePosition(NozzleNo, &ptANCxyrz) != true)
	{
		return ErrCode;
	}

	TRACE(_T("[PWR] ANC Hold %s Nzl:%d XYRZ %.3f,%.3f,%.3f,%.3f"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo, ptANCxyrz.x, ptANCxyrz.y, ptANCxyrz.r, ptANCxyrz.z);

	ptANCxy.x = ptANCxyrz.x;
	ptANCxy.y = ptANCxyrz.y;
	target = HeadNo;

	strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
	ActBaseUpDown(BASE_UP);
	TRACE(_T("[PWR] ANC Hold %s Nzl:%d MoveStart"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
	Err = MoveR(strRAxis, RatioR, TimeOut, ptANCxyrz.r, InposR, MsXy, false);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	Err = MoveXY(Gantry, target, ptANCxy, RatioXY, InposXY, MsXy, TimeOut);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	TimeLock = _time_get();


	TimeAct = _time_get();
	while (1)
	{
		Err = GetBaseUpDown(BASE_UP, TIME100MS);
		if (Err == NO_ERR)
		{
			break;
		}
		else if (_time_elapsed(TimeAct) > TIME3000MS)
		{
			ActBaseUpDown(BASE_DOWN);
			TRACE(_T("[PWR] ANC Hold %s Nzl:%d BaseUp TimeOut"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			return ErrCode;
		}

		ThreadSleep(TIME1MS);
	}

	Err = WaitR(strRAxis, ptANCxyrz.r, InposR, TimeOut);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	TRACE(_T("[PWR] ANC Hold %s Nzl:%d Z Down"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
	Err = RemoveHeadTorqueLimitEvent(Gantry, HeadNo);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	Err = SetHeadTorqueLimitEvent(Gantry, HeadNo, AncZTorqueLimit);
	if (Err != NO_ERR)
	{
		return ErrCode;
	}
	ErrZ = MoveZ(strZAxis, RatioZ, TimeOut, ptANCxyrz.z, InposZ, MsXy, false);
	if (ErrZ != NO_ERR)
	{
		InitialHeadTorqueLimit(Gantry, HeadNo);
		return ErrCode;
	}

	SetGlobalNozzleNo(HeadNo, NozzleNo);

	Err = WaitZ(strZAxis, ptANCxyrz.z, InposZ, TimeOut);
	if (Err != NO_ERR)
	{
		InitialHeadTorqueLimit(Gantry, HeadNo);
		return ErrCode;
	}

	TRACE(_T("[PWR] ANC Hold %s Nzl:%d UnLock"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);

	ActBaseLock(BASE_UNLOCK);
	TimeAct = _time_get();
	while (1)
	{
		Err = GetBaseLock(BASE_UNLOCK, TIME500MS);
		if (Err == NO_ERR)
		{
			break;
		}
		else if (_time_elapsed(TimeAct) > TIME3000MS)
		{
			TRACE(_T("[PWR] ANC Hold %s Nzl:%d Unlock TimeOut"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo);
			return ErrCode;
		}

		ThreadSleep(TIME1MS);
	}

	//UpposZ = GetStandByZ(Gantry) - Nozzle.TipHeight;
	UpposZ = GetStandByZ(Gantry);

	Err = MoveZ(strZAxis, RatioZ, TimeOut, UpposZ, InposZ, MsXy, true);
	if (Err != NO_ERR)
	{
		InitialHeadTorqueLimit(Gantry, HeadNo);
		return ErrCode;
	}

	if (Recovery == true)
	{
		ActBaseLock(BASE_LOCK);
		ThreadSleep(TIME100MS);
		ActBaseUpDown(BASE_DOWN);
	}

	InitialHeadTorqueLimit(Gantry, HeadNo);

	TRACE(_T("[PWR] ANC Hold %s Nzl:%d Complete. %d[ms]"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNo, _time_elapsed(TimeANC));

	return Err;
}


long CAutoNozzleChange::Run()
{
	long Err = NO_ERR;
	long InsertNo, NexHead, NozzleNoJobfile, NozzleNoCurrent;
	//	long InsertNoSort[MAXUSEDHEADNO];
	//	long HeadNoSort[MAXUSEDHEADNO];
	bool Recovery = false;
	long Base = GetTable();
	long Gantry = FRONT_GANTRY;

	if (gCMachineConfig->IsExistANCData() == false)
	{
		return NO_ERR;
	}

	for (long CurrentHead = 1; CurrentHead <= MAXUSEDHEADNO; CurrentHead++)
	{
		NozzleNoCurrent = GetGlobalNozzleNo(CurrentHead);

		if (NozzleNoCurrent == 0)
		{
			continue;
		}

		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			InsertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);
			NexHead = GetHeadNoFromInsertOrder(InsertOrd + 1);
			NozzleNoJobfile = GetNozzleNoFromInsertNo(InsertNo);

			if (CurrentHead == NexHead)
			{
				if (NozzleNoJobfile != NozzleNoCurrent)
				{
					Err = NozzleRelease(Gantry, CurrentHead, Recovery);
					if (Err != NO_ERR)
					{
						return Err;
					}

					break;
				}
			}
			else if (NozzleNoJobfile == NozzleNoCurrent)
			{
				Err = NozzleRelease(Gantry, CurrentHead, Recovery);
				if (Err != NO_ERR)
				{
					return Err;
				}

				break;
			}
		}

	}

	for (long CurrentHead = MAXUSEDHEADNO; CurrentHead >= 1; CurrentHead--)
	{
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			InsertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);
			NexHead = GetHeadNoFromInsertOrder(InsertOrd + 1);
			NozzleNoJobfile = GetNozzleNoFromInsertNo(InsertNo);
			NozzleNoCurrent = GetGlobalNozzleNo(NexHead);

			if (CurrentHead != NexHead)
			{
				continue;
			}

			if (NozzleNoJobfile == NozzleNoCurrent)
			{
				TRACE(_T("[PWR] ANC Skip - Same Nozzle %s Nzl:%d"), GetZAxisFromHeadNo(Gantry, NexHead), NozzleNoJobfile);
				continue;
			}

			Err = NozzleHold(Gantry, NexHead, NozzleNoJobfile, Recovery);
			if (Err != NO_ERR)
			{
				return Err;
			}
			break;
		}
	}

	ActBaseLock(BASE_LOCK);
	ActBaseUpDown(BASE_DOWN);

	return Err;
}


//long CAutoNozzleChange::Run()
//{
//	long Err = NO_ERR, Gantry = FRONT_GANTRY;
//	long InsertNo, HeadNo, NozzleNoJobfile, NozzleNoCurrent;
////	long InsertNoSort[MAXUSEDHEADNO];
////	long HeadNoSort[MAXUSEDHEADNO];
//
//	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
//	{
//		InsertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);
//		HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
//		NozzleNoJobfile = GetNozzleNoFromInsertNo(InsertNo);
//		NozzleNoCurrent = GetGlobalNozzleNo(HeadNo);
//
//		if (NozzleNoJobfile == NozzleNoCurrent)
//		{
//			TRACE(_T("[PWR] ANC Skip - Same Nozzle %s Nzl:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoJobfile);
//			continue;
//		}
//
//		for (long head = 1; head <= MAXUSEDHEADNO; head++)
//		{
//			if (head == HeadNo)
//			{
//				continue;
//			}
//			else if (NozzleNoJobfile == GetGlobalNozzleNo(head))
//			{
//				Err = NozzleRelease(Gantry, head);
//				if (Err != NO_ERR)
//				{
//					return Err;
//				}
//
//				break;
//			}
//		}
//
//		if (NozzleNoCurrent != 0)
//		{
//			Err = NozzleRelease(Gantry, HeadNo);
//			if (Err != NO_ERR)
//			{
//				return Err;
//			}
//		}
//
//		Err = NozzleHold(Gantry, HeadNo, NozzleNoJobfile);
//		if (Err != NO_ERR)
//		{
//			return Err;
//		}
//	}
//
//	return Err;
//}


//long CAutoNozzleChange::RunOptmize()
//{
//	long Err = NO_ERR, Gantry = FRONT_GANTRY;
//	long InsertNo, HeadNo, NozzleNoJobfile, NozzleNoCurrent;
//	//	long InsertNoSort[MAXUSEDHEADNO];
//	//	long HeadNoSort[MAXUSEDHEADNO];
//
//	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
//	{
//		InsertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);
//		HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
//		NozzleNoJobfile = GetNozzleNoFromInsertNo(InsertNo);
//		NozzleNoCurrent = GetGlobalNozzleNo(HeadNo);
//
//		if (NozzleNoJobfile == NozzleNoCurrent)
//		{
//			TRACE(_T("[PWR] ANC Skip - Same Nozzle %s Nzl:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoJobfile);
//			continue;
//		}
//
//		for (long head = 1; head <= MAXUSEDHEADNO; head++)
//		{
//			if (head == HeadNo)
//			{
//				continue;
//			}
//			else if (NozzleNoJobfile == GetGlobalNozzleNo(head))
//			{
//				NozzleRelease(Gantry, head);
//				break;
//			}
//		}
//
//		if (NozzleNoCurrent != 0)
//		{
//			NozzleRelease(Gantry, HeadNo);
//		}
//
//		NozzleHold(Gantry, HeadNo, NozzleNoJobfile);
//	}
//
//	return Err;
//}

long CAutoNozzleChange::RunPrepare()
{
	long Err = NO_ERR, Gantry = FRONT_GANTRY;
	long NozzleNoCurrent, NozzleNoPrepare;
	bool Recovery = false;

	for(long HeadNo = 1; HeadNo <= GetZAxisCount(); HeadNo++)
	{
		NozzleNoCurrent = GetGlobalNozzleNo(HeadNo);
		NozzleNoPrepare = GetNozzleNoFromANCPrepare(Gantry, HeadNo);

		if (NozzleNoPrepare == 0 || NozzleNoCurrent == NozzleNoPrepare)
		{
			TRACE(_T("[PWR] ANC Prepare Skip - %s Current:%d Prepare:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoCurrent, NozzleNoPrepare);
			continue;
		}

		for (long ReleaseHeadNo = 1; ReleaseHeadNo <= GetZAxisCount(); ReleaseHeadNo++)
		{
			if (ReleaseHeadNo == HeadNo)
			{
				continue;
			}
			else if (NozzleNoPrepare == GetGlobalNozzleNo(ReleaseHeadNo))
			{
				TRACE(_T("[PWR] ANC Prepare Other Head Release - %s Current:%d "), GetZAxisFromHeadNo(Gantry, ReleaseHeadNo), NozzleNoCurrent, NozzleNoPrepare);

				Err = NozzleRelease(Gantry, ReleaseHeadNo, Recovery);
				if (Err != NO_ERR)
				{
					return Err;
				}

				break;
			}
		}

		if (NozzleNoCurrent != 0)
		{
			TRACE(_T("[PWR] ANC Prepare Target Head Release - %s Current:%d "), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoCurrent);

			Err = NozzleRelease(Gantry, HeadNo, Recovery);
			if (Err != NO_ERR)
			{
				return Err;
			}
		}

		TRACE(_T("[PWR] ANC Prepare Target Head Hold - %s Prepare:%d"), GetZAxisFromHeadNo(Gantry, HeadNo), NozzleNoPrepare);

		Err = NozzleHold(Gantry, HeadNo, NozzleNoPrepare, Recovery);
		if (Err != NO_ERR)
		{
			return Err;
		}
	}

	ActBaseLock(BASE_LOCK);
	ActBaseUpDown(BASE_DOWN);

	return Err;
}

void CAutoNozzleChange::ActBaseUpDown(long UpDown)
{
	long Base = GetTable();

	if (gCMachineConfig->IsANCUpType(Base) == false)
	{
		return;
	}

	CString strFunc(__func__);
	long OnOff;

	if (UpDown == BASE_UP)
	{
		TRACE(_T("[PWR] ANC %s Base:%d Act:%d BaseUp"), strFunc, Base, UpDown);
		OnOff = OUTON;
	}
	else
	{
		TRACE(_T("[PWR] ANC %s Base:%d Act:%d BaseDown"), strFunc, Base, UpDown);

		OnOff = OUTOFF;
	}

	if (Base == FRONT_STAGE)
	{
		OutputOne(OUT_FANC_BASE_UP, OnOff);
	}
	else
	{
		OutputOne(OUT_RANC_BASE_UP, OnOff);
	}
}

long CAutoNozzleChange::GetBaseUpDown(long UpDown, long Time)
{
	long Base = GetTable();

	if (gCMachineConfig->IsANCUpType(Base) == false || m_SkipIOCheck == true)
	{
		return NO_ERR;
	}
	CString strFunc(__func__);

	long OnOffUp, OnOffDown;
	long InputUp, InputDown;
	long Err;

	if (UpDown == BASE_UP)
	{
		OnOffUp = INON;
		OnOffDown = INOFF;
		Err = ANC_BASE_UP_TIMEOUT(Base);
	}
	else
	{
		OnOffUp = INOFF;
		OnOffDown = INON;
		Err = ANC_BASE_DN_TIMEOUT(Base);
	}

	if (Base == FRONT_STAGE)
	{
		InputUp = IN_FANC_BASE_UP;
		InputDown = IN_FANC_BASE_DN;
	}
	else
	{
		InputUp = IN_RANC_BASE_UP;
		InputDown = IN_RANC_BASE_DN;
	}

	if (InputElapsedTimeOne(InputUp, OnOffUp, Time) == false || InputElapsedTimeOne(InputDown, OnOffDown, Time) == false)
	{
		//TRACE(_T("[PWR] ANC %s Base:%d Act:%d Fail:%d"), strFunc, Base, UpDown, Err);

		return Err;

	}

	//TRACE(_T("[PWR] ANC %s Base:%d Act:%d OK"), strFunc, Base, UpDown);

	return NO_ERR;

}


void CAutoNozzleChange::ActBaseLock(long LockUnlock)
{
	long Base = GetTable();
	long OnOff;
	CString strFunc(__func__);

	TRACE(_T("[PWR] ANC %s Base:%d Act:%d"), strFunc, Base, LockUnlock);

	if (LockUnlock == BASE_LOCK)
	{
		OnOff = OUTOFF;
	}
	else
	{
		OnOff = OUTON;
	}

	if (Base == FRONT_STAGE)
	{
		OutputOne(OUT_FANC_CLAMP_UNLOCK, OnOff);
	}
	else
	{
		OutputOne(OUT_RANC_CLAMP_UNLOCK, OnOff);
	}
}

long CAutoNozzleChange::GetBaseLock(long LockUnlock, long Time)
{
	long Base = GetTable();

	if (m_SkipIOCheck == true)
	{
		return NO_ERR;
	}

	CString strFunc(__func__);

	long OnOffLock, OnOffUnlock;
	long InputLock, InputUnlock;
	long Err;

	if (LockUnlock == BASE_LOCK)
	{
		OnOffLock = INON;
		OnOffUnlock = INOFF;
		Err = ANC_BASE_LOCK_TIMEOUT(Base);
		Time = TIME300MS;

	}
	else
	{
		OnOffLock = INOFF;
		OnOffUnlock = INON;
		Err = ANC_BASE_UNLOCK_TIMEOUT(Base);
		Time = TIME150MS;

	}

	if (Base == FRONT_STAGE)
	{
		InputLock = IN_FANC_CLAMP_LOCK;
		InputUnlock = IN_FANC_CLAMP_UNLOCK;
	}
	else
	{
		InputLock = IN_RANC_CLAMP_LOCK;
		InputUnlock = IN_RANC_CLAMP_UNLOCK;
	}


	if (InputElapsedTimeOne(InputLock, OnOffLock, Time) == true && InputElapsedTimeOne(InputUnlock, OnOffUnlock, Time) == true)
	{
		TRACE(_T("[PWR] ANC %s Base:%d Act:%d OK"), strFunc, Base, LockUnlock);

		return NO_ERR;
	}

	return Err;
}

bool CAutoNozzleChange::GetUseANC()
{
	ANC_STRUCT data;
	long Base = GetTable();

	if (gCMachineConfig->GetConfigANC(Base, &data) == true)
	{
		return data.Use;
	}

	return false;
}


bool CAutoNozzleChange::IsUpDownType()
{
	ANC_STRUCT data;
	long Base = GetTable();

	if (gCMachineConfig->GetConfigANC(Base, &data) == true)
	{
		return data.UpdownType;
	}

	return false;
}