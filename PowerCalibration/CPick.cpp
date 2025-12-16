#include "pch.h"
#include "CPick.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
#include "CFeeder.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CAdvancedMotionFile.h"
#include "Trace.h"
#include "CStep.h"
#include "CReadJobFile.h"
#include "CCamDropCheck.h"
#include "CRecognitionDivide.h"
CPick* gcPick;
CPick::CPick(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxPickOrder = 0;
	m_ProdRunMode = RUN_REAL;
}

CPick::~CPick()
{
}

void CPick::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CPick::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CPick::GetPickupHeadNo(long PickOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetPickupHeadNo(PickOrd);
	return HeadNo;
}

long CPick::GetMaxPickOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxPickOrder();
	return RetMaxOrder;
}

void CPick::SetMaxPickOrder(long MaxPickOrd)
{
	//if (MaxPickOrd > 0 && MaxPickOrd <= MAXUSEDHEADNO)
	//{
	//	m_MaxPickOrder = MaxPickOrd;
	//}
}

long CPick::GetFdNoFromPickOrder(long PickOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromPickOrder(PickOrd);
	return FdNo;
}

long CPick::GetPickDelayFromFdNo(long FdNo)
{
	long PickDelay;
	PickDelay = gcStep->GetPickDelayFromFdNo(FdNo);
	return PickDelay;
}

Ratio_XYRZ CPick::GetRatioFromFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

Point_XYRZ CPick::GetPickOffsetFromFdNo(long FdNo)
{
	Point_XYRZ pt;
	pt = gcStep->GetPickOffsetFromFdNo(FdNo);
	if (FdNo <= MAXHALFFEEDNO)
	{
		pt.r += 0.0;
	}
	else
	{
		pt.r += 180.0;
		TRACE(_T("[PWR] FdNo%03d Add PickOffsetR:%.3f\n"), FdNo, pt.r);
	}
	return pt;
}

long CPick::GetReadyNoFromFeederNo(long FeederNo)
{
	long ReadyNo = 0;
	ReadyNo = gcStep->GetReadyNoFromFeederNo(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyNoFromFeederNo FeederNo:%d ReadyNo:%d\n"), FeederNo, ReadyNo);
	}
	return ReadyNo;
}

long CPick::GetReleaseNoFromFeederNo(long FeederNo)
{
	long ReleaseNo = 0;
	ReleaseNo = gcStep->GetReleaseNoFromFeederNo(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReleaseNoFromFeederNo FeederNo:%d ReleaseNo:%d\n"), FeederNo, ReleaseNo);
	}
	return ReleaseNo;
}

long CPick::GetReadyTimeOutFromFeederNo(long FeederNo)
{
	long TimeOut = 0;
	TimeOut = gcStep->GetReadyTimeOutFromFeederNo(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutFromFeederNo FeederNo:%d TimeOut:%d\n"), FeederNo, TimeOut);
	}
	return TimeOut;
}

long CPick::GetReadyWaitDelayFromFeederNo(long FeederNo)
{
	long ReadyWaitDelay = 0;
	ReadyWaitDelay = gcStep->GetReadyWaitDelayFromFeederNo(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyWaitDelayFromFeederNo FeederNo:%d ReadyWaitDelay:%d\n"), FeederNo, ReadyWaitDelay);
	}
	return ReadyWaitDelay;
}

long CPick::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmpty FeederNo:%d ReadyTimeOutEmpty:%d\n"), FeederNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CPick::SetReadyTimeOutEmpty(long FeederNo, long ReadyTimeOutEmpty)
{
	long Err = NO_ERR;
	gcStep->SetReadyTimeOutEmpty(FeederNo, ReadyTimeOutEmpty);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetReadyTimeOutEmpty FeederNo:%d ReadyTimeOutEmpty:%d\n"), FeederNo, ReadyTimeOutEmpty);
	}
	return Err;
}

long CPick::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmptyByHeadNo HeadNo:%d ReadyTimeOutEmpty:%d\n"), HeadNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CPick::SetReadyTimeOutEmptyByHeadNo(long HeadNo, long ReadyTimeOutEmpty)
{
	long Err = NO_ERR;
	gcStep->SetReadyTimeOutEmptyByHeadNo(HeadNo, ReadyTimeOutEmpty);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetReadyTimeOutEmptyByHeadNo HeadNo:%d ReadyTimeOutEmpty:%d\n"), HeadNo, ReadyTimeOutEmpty);
	}
	return Err;
}

long CPick::ClearReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long Err = NO_ERR;
	gcStep->SetReadyTimeOutEmptyByHeadNo(HeadNo, 0);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] ClearReadyTimeOutEmptyByHeadNo HeadNo:%d\n"), HeadNo);
	}
	return Err;
}

bool CPick::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	bSendEmpty = gcStep->GetSendEmpty(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] CPick GetSendEmpty FeederNo:%d bSendEmpty:%d\n"), FeederNo, bSendEmpty);
	}
	return bSendEmpty;
}

long CPick::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	long Err = NO_ERR;
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] CPick SetSendEmpty FeederNo:%d ReadyTimeOutEmpty:%d\n"), FeederNo, bSendEmpty);
	}
	return Err;
}

long CPick::SetEmptyError(long InsertOrd, long ReadyTimeOutEmpty)
{
	long Err = NO_ERR;
	gcStep->SetEmptyError(InsertOrd, ReadyTimeOutEmpty);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetEmptyError InsertOrd:%d ReadyTimeOutEmpty:%d\n"), InsertOrd, ReadyTimeOutEmpty);
	}
	return Err;
}

double CPick::GetComponentHeight(long FeederNo)
{
	double ComponentHeight = 10.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FeederNo);
	return ComponentHeight;
}

double CPick::GetComponentLeadHeight(long FeederNo)
{
	double ComponentLeadHeight = 10.0;
	ComponentLeadHeight = gcStep->GetComponentLeadHeightFromFdNo(FeederNo);
	return ComponentLeadHeight;
}

TwoStepMotion CPick::GetTwoStepPick(long FeederNo)
{
	TwoStepMotion TwoStepPick;
	TwoStepPick = gcStep->GetTwoStepPick(FeederNo);
	return TwoStepPick;
}

double CPick::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	return MaxComponentHeight;
}

double CPick::GetPickupZStandBy()
{
	double PickupZStandBy = 0.0;
	PickupZStandBy = gcStep->GetPickupZStandBy();
	return PickupZStandBy;
}

long CPick::GetFeederReadyIOType(long FeederNo)
{
	long FeederReadyIOType = 0;
	FeederReadyIOType = gcStep->GetFeederReadyIOType(FeederNo);
	return FeederReadyIOType;
}

TRAY_INFO CPick::GetTrayInfo(long FeederNo)
{
	TRAY_INFO Tray;
	Tray = gcStep->GetTray(FeederNo);
	return Tray;
}

long CPick::GetTrayMaxPocket(long FeederNo)
{
	long TrayMaxPocket = 1;
	TrayMaxPocket = gcStep->GetTrayMaxPocket(FeederNo);
	return TrayMaxPocket;
}

long CPick::GetTrayNowPocket(long FeederNo)
{
	long TrayNowPocket = 1;
	TrayNowPocket = gcStep->GetTrayNowPocket(FeederNo);
	return TrayNowPocket;
}

long CPick::GetFeederType(long FeederNo)
{
	long FeederType;
	FeederType = gcStep->GetFeederType(FeederNo);
	return FeederType;
}


long CPick::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CPick::FirstMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CPick::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CPick::MoveXy(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CPick::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CPick::MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CPick::WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut, long UseSendAlarm)
{
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return GetZTorqueAlarmcode(FRONT_GANTRY, TBL_HEAD1);
	//}

	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut, UseSendAlarm);
	InitOneRatio(strAxis);
	return Err;
}

long CPick::MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR;
	Err = StartPosWaitInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CPick::ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CPick::Run(bool bFirstPick, bool bManualRecog)
{
	long Err = NO_ERR, Gantry = m_Gantry, Stage = FRONT_STAGE;
	CString strZAxis, strRAxis, strOtherZAxis, strFileName;
	Ratio_XYRZ CompRatio[MAXUSEDHEADNO], OtherRatio, StepMinRatio, MoveRatio;
	long MsXy, MsZDn, MsZUp, MsR;
	MsXy = MsZDn = MsZUp = MsR = TIME30MS;
	long TimeOut = TIME5000MS, HeadNo[MAXUSEDHEADNO], JobPickDelay[MAXUSEDHEADNO], FdNo[MAXUSEDHEADNO];
	long OtherHeadNo = 0, OtherFdNo = 0;
	double InposXY = 0.10, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	double RatioR = 1.0, RatioZ = 1.0, Pitch = 1.0, ComponentBodyHeight = 0.0, Angle = 0.0, ComponentLeadHeight = 0.0;
	double OtherComponentBodyHeight = 0.0, OtherComponentLeadHeight = 0.0;
	double PickupWithComponentZ = GetStandByZ(Gantry), PickupZStandBy = GetPickupZStandBy(), OtherPickupZStand = GetPickupZStandBy();
	double CurPos = 0.0, TargetPos = 0.0;
	long NozzleType = 0, ReadyTimeOutEmpty = 0, ReleaseIO = IO_NOUSE, NozzleNo = 0, OtherNozzleNo = 0;
	bool bFirstMoveR = true, bFirstMoveXY = true;
	long FeederReadyIOType = 0;
	Point_XY pt, RefPt, CurPt;
	Point_XYRZ PickOffset[MAXUSEDHEADNO];
	NOZZLE Nozzle[MAXUSEDHEADNO], OtherNozzle;
	CFeeder* pFeeder[MAXUSEDHEADNO];
	CFeeder* pRetFeeder;
	long ReadyNo[MAXUSEDHEADNO];
	long ReleaseNo[MAXUSEDHEADNO];
	long ReadyTimeOut[MAXUSEDHEADNO];
	long ReadyWaitDelay[MAXUSEDHEADNO];
	TwoStepMotion TwoStepPick;
	bool bFirstMoveXYRatio = true;
	long FeederType = 0;
	long MaxPocket = 1, NowPocket = 1;
	TRAY_INFO Tray;
	PICK_LEVEL_CHECK PickLevelInfo;
	ULONGLONG GetTimeSuction = _time_get();
	long LevelBefore = 0;
	long LevelAfter = 0;
	double MeasureDifferntLevel = 0;
	double EmptyLevelMargin = 0.1;
	double DifferntLevelMargin = 0.5;
	double ZDownRatio = 1.0;
	bool PickHead[MAXUSEDHEADNO];
	long ErrTorque;

	ZeroMemory(&FdNo, sizeof(FdNo));
	ZeroMemory(&CurPt, sizeof(CurPt));
	ZeroMemory(&HeadNo, sizeof(HeadNo));
	ZeroMemory(&Nozzle, sizeof(Nozzle));
	ZeroMemory(&ReadyNo, sizeof(ReadyNo));
	ZeroMemory(&ReleaseNo, sizeof(ReleaseNo));
	ZeroMemory(&PickOffset, sizeof(PickOffset));
	ZeroMemory(&TwoStepPick, sizeof(TwoStepPick));
	ZeroMemory(&JobPickDelay, sizeof(JobPickDelay));
	ZeroMemory(&ReadyTimeOut, sizeof(ReadyTimeOut));
	ZeroMemory(&ReadyWaitDelay, sizeof(ReadyWaitDelay));
	ZeroMemory(&PickHead, sizeof(PickHead));

	//CApplicationTime* pTime = new CApplicationTime();
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	ULONGLONG PickupReadyGetTime = _time_get(), PickupReadyElapsed = 0;
	ULONGLONG TorqueMonitorGetTime = _time_get(), TorqueMonitorElapsed = 0;

	OtherRatio.xy = OtherRatio.r = OtherRatio.z = 1.0;
	StepMinRatio = GetMinRatio();
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] Pick Run Step1\n"));
	}

	if (bManualRecog == true)
	{
		//gcLastPickFront->ClearAllHeadData();
	}

	if (bFirstPick == true)
	{
		RatioZ = GetMinRatioLastPickData().z;
		Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), RatioZ);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Picking FirstMoveAllZUp Err"));
			TRACE(_T("[PWR] Picking FirstMoveAllZUp Err:%d\n"), Err);
			return Err;
		}
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] Pick Run ProdRunMode:%d\n"), GetProdRunMode());
		TRACE(_T("[PWR] Pick Run Step2 Err:%d GetMaxPickOrder(%d)\n"), Err, GetMaxPickOrder());
	}
	GetTime = _time_get();
	Err = WaitGantry(FRONT_GANTRY, TIME5000MS);
	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowRunLog() == true|| Elapsed > 0)
	{
		TRACE(_T("[PWR] CPick WaitGantry Elasped,%d"), Elapsed);
	}
	if (gcAdvancedMotionFile->UseAdvanceMotion() == 1)
	{
		MsXy = gcAdvancedMotionFile->GetPickDelayedMsXY();
		MsR = gcAdvancedMotionFile->GetPickDelayedMsR();
		MsZDn = gcAdvancedMotionFile->GetPickDelayedMsZDn();
		MsZUp = gcAdvancedMotionFile->GetPickDelayedMsZUp();
		InposXY = gcAdvancedMotionFile->GetPickInposXY();
		InposR = gcAdvancedMotionFile->GetPickInposR();
		InposZDn = gcAdvancedMotionFile->GetPickInposZDn();
		InposZUp = gcAdvancedMotionFile->GetPickInposZUp();
		TRACE(_T("[PWR] Pick Ms XY,%d R,%d ZDn,%d ZUp,%d Inpos XY,%.3f R,%.3f ZDn,%.3f ZUp,%.3f\n"),
			MsXy, MsR, MsZDn, MsZUp, InposXY, InposR, InposZDn, InposZUp);
	}
	for (long Head = 0; Head < MAXUSEDHEADNO; ++Head)
	{
		ClearReadyTimeOutEmptyByHeadNo(Head + TBL_HEAD1);
	}

	if (GetProdRunMode() != RUN_DRY)
	{
		for (long PickOrd = 0; PickOrd < GetMaxPickOrder(); ++PickOrd)
		{
			long Head = GetPickupHeadNo(PickOrd + 1);
			long Fd = GetFdNoFromPickOrder(PickOrd + 1);
			if (Head == 0) continue;
			if (Fd == 0) continue;


			NozzleNo = GetGlobalNozzleNo(Head);
			if (GetGlobalNozzleInformation(NozzleNo).Type != 0) continue;

			//if (gcStep->GetPickLevelCheck(Fd).Use == false) continue;
			SuctionOne(Gantry, Head, true);
			TRACE(_T("[PWR] PickLabel PreSuctionOn(%s)\n"), GetZAxisFromHeadNo(Gantry, Head));
			GetTimeSuction = _time_get();
            //PreSuction 동작 시 set DiscardInfo 해야함.
            Err = gcLastPickFront->setDiscardInfo(Head, CLastPick::DiscardInfo{ Fd, gcReadJobFile->GetDiscard(Fd).pt }, true);
            if (Err != NO_ERR)
            {
                Err = SendAlarm(Err, L"setDiscardInfo for PreSuction failed..");
                return Err;
            }
		}
	}

	for (long PickOrd = 0; PickOrd < GetMaxPickOrder(); ++PickOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] Pick GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] Pick GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		HeadNo[PickOrd] = GetPickupHeadNo(PickOrd + 1);
		FdNo[PickOrd] = GetFdNoFromPickOrder(PickOrd + 1);
		if (HeadNo[PickOrd] == 0) continue;
		if (FdNo[PickOrd] == 0) continue;
		SetReadyTimeOutEmpty(FdNo[PickOrd], 0);
		ReadyNo[PickOrd] = GetReadyNoFromFeederNo(FdNo[PickOrd]);
		ReleaseNo[PickOrd] = GetReleaseNoFromFeederNo(FdNo[PickOrd]);
		ReadyTimeOut[PickOrd] = GetReadyTimeOutFromFeederNo(FdNo[PickOrd]);
		ReadyWaitDelay[PickOrd] = GetReadyWaitDelayFromFeederNo(FdNo[PickOrd]);
		pRetFeeder = GetFeeder(FdNo[PickOrd]);
		ComponentBodyHeight = GetComponentHeight(FdNo[PickOrd]);
		ComponentLeadHeight = GetComponentLeadHeight(FdNo[PickOrd]);
		TwoStepPick = GetTwoStepPick(FdNo[PickOrd]);
		FeederReadyIOType = GetFeederReadyIOType(FdNo[PickOrd]);
		FeederType = GetFeederType(FdNo[PickOrd]);

		if (gcLastPickFront->GetHeadData(HeadNo[PickOrd]).Enable == true)
		{
			TRACE(_T("[PWR] LastPickData Pick Skip %s\n"), GetZAxisFromHeadNo(Gantry, HeadNo[PickOrd]));
			SendToPickSkipByLastPick(gcLastPickFront->GetHeadData(HeadNo[PickOrd]).Insert.FeederNo);
			continue;
		}

		if (IsLabelTypeFeeder(FeederType) == true)
		{
			Tray = GetTrayInfo(FdNo[PickOrd]);
			MaxPocket = GetTrayMaxPocket(FdNo[PickOrd]);
			NowPocket = GetTrayNowPocket(FdNo[PickOrd]);

			TRACE(_T("[PWR] PickLabel FeederNo(%03d) Pocket Max:%d Now:%d\n"), FdNo[PickOrd], MaxPocket, NowPocket);

			if (MaxPocket <= NowPocket)
			{
				NowPocket = 0;
				SetTrayNowPocket(FdNo[PickOrd], 0);
				SendTrayLastPocket(FdNo[PickOrd], 0);
			}
		}
		else if (IsTrayTypeFeeder(FeederType) == true)
		{
			Tray = GetTrayInfo(FdNo[PickOrd]);
			MaxPocket = GetTrayMaxPocket(FdNo[PickOrd]);
			NowPocket = GetTrayNowPocket(FdNo[PickOrd]);
			TRACE(_T("[PWR] Pick Tray FeederNo(%03d) Pocket Max:%d Now:%d\n"), FdNo[PickOrd], MaxPocket, NowPocket);
			if (MaxPocket <= NowPocket)
			{
				SetEmptyError(PickOrd + 1, 1);
				SetReadyTimeOutEmpty(FdNo[PickOrd], 1);
				SetReadyTimeOutEmptyByHeadNo(HeadNo[PickOrd], 1);
				SendEmpty(FdNo[PickOrd], EMPTY_READY_TIMEOUT, _T("Tray Pocket is out of range"));
				SetSendEmpty(FdNo[PickOrd], true);
				continue;				
			}
		}

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] Pick FeederNo(%03d) BodyHeight:%.3f LeadHeight:%.3f\n"), FdNo[PickOrd], ComponentBodyHeight, ComponentLeadHeight);
		}
		if (ReadyNo[PickOrd] > 0)
		{
			if (pRetFeeder != NULL)
			{
				pFeeder[PickOrd] = pRetFeeder;
			}
			else
			{
				TRACE(_T("[PWR] Pick FeederNo(%03d) ReadyIO(%d) pRetFeeder is NULL\n"), FdNo[PickOrd], ReadyNo[PickOrd]);
				continue;
			}
		}
		strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo[PickOrd]);
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo[PickOrd]);
		if (FdNo[PickOrd] <= MAXHALFFEEDNO)
			Stage = FRONT_STAGE;
		else
			Stage = REAR_STAGE;
		RefPt = GetFeederPosition(Stage, FdNo[PickOrd]);
		PickOffset[PickOrd] = GetPickOffsetFromFdNo(FdNo[PickOrd]);

		if (bManualRecog == false)
		{
			if (IsTrayTypeFeeder(FeederType) == true || IsLabelTypeFeeder(FeederType) == true)
			{
				PickOffset[PickOrd].x += Tray.pt[NowPocket].x;
				PickOffset[PickOrd].y += Tray.pt[NowPocket].y;
			}
		}


		CompRatio[PickOrd] = GetRatioFromFdNo(FdNo[PickOrd]);
		JobPickDelay[PickOrd] = GetPickDelayFromFdNo(FdNo[PickOrd]);
		//Nozzle[PickOrd] = GetNozzle(FdNo[PickOrd]);
		NozzleNo = GetGlobalNozzleNo(HeadNo[PickOrd]);
		Nozzle[PickOrd] = GetGlobalNozzleInformation(NozzleNo);
		pt.x = RefPt.x + PickOffset[PickOrd].x;
		pt.y = RefPt.y + PickOffset[PickOrd].y;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] PickOffsetXYRZ,%.3f,%.3f,%.3f,%.3f\n"), PickOffset[PickOrd].x,
				PickOffset[PickOrd].y, PickOffset[PickOrd].r, PickOffset[PickOrd].z);
			TRACE(_T("[PWR] PickOrd(%d)(%s,%s) Run FeederNo:%d HeadNo:%d Delay:%d\n"),
				PickOrd, strRAxis, strZAxis,
				FdNo[PickOrd], HeadNo[PickOrd], JobPickDelay[PickOrd]);
			TRACE(_T("[PWR] PickOrd(%d) RatioXYRZ,%.3f,%.3f,%.3f\n"), PickOrd,
				CompRatio[PickOrd].xy,
				CompRatio[PickOrd].r, CompRatio[PickOrd].z);
		}
		NozzleType = Nozzle[PickOrd].Type;
		GetTime = _time_get();
		Err = MoveR(strRAxis, /*CompRatio[PickOrd].r*/1.0, TimeOut, PickOffset[PickOrd].r, InposR, MsR, false);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Pick MoveR Err"));
			TRACE(_T("[PWR] Pick(%s) MoveR Err:%d\n"), strRAxis, Err);
			return Err;
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			CurPt = gReadGantryPosition(Gantry);
		}

		if (bFirstMoveXYRatio == true)
		{
			bFirstMoveXYRatio = false;
			MoveRatio.xy = GetMinRatioLastPickData().xy;
			//MoveRatio.xy = 1.0;
			//	TRACE(_T("[PWR] Pick PickOrd:%d First Pick XY Spd,%03d%%"), PickOrd + 1, (long)(MoveRatio.xy * 100));
		}
		else
		{
			// HarkDo 20210127-1
			//for (long SearchPick = PickOrd; SearchPick < GetMaxPickOrder(); SearchPick++) // 동시 픽업할 때 나중에 확인
			{
				if (MoveRatio.xy > GetPickRatio(PickOrd + 1).xy)
				{
					MoveRatio.xy = GetPickRatio(PickOrd + 1).xy;
					TRACE(_T("[PWR] Pick PickOrd:%d Change XY Spd,%03d%%"), PickOrd + 1, (long)(MoveRatio.xy * 100));
				}
			}
		}

		if (bFirstMoveXY == true && gcStep->GetRecogTableBy1stInsertOrder() == FRONT_STAGE)
		{
			gPartDropLedOn(FRONT_STAGE);
		}

		TRACE(_T("[PWR] Pick MoveXY %s, %.3f %.3f"), strZAxis, pt.x, pt.y);

		Err = MoveXy(Gantry, HeadNo[PickOrd], pt, MoveRatio.xy, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Pick MoveXy Feeder Err"));
			TRACE(_T("[PWR] Pick MoveXy Feeder(%03d) Err:%d\n"), FdNo[PickOrd], Err);
			return Err;
		}

		if (IsAccTest() == true)
		{
			continue;
		}

		if (bFirstMoveXY == true && gcStep->GetRecogTableBy1stInsertOrder() == FRONT_STAGE)
		{
			gPartDropProcess(FRONT_STAGE);
		}

		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TRACE(_T("[PWR] Pick Feeder(%03d) Dist (%.3f,%.3f) XY Elapsed,%d\n"), FdNo[PickOrd], pt.x - CurPt.x, pt.y - CurPt.y, Elapsed);
		}
		GetTime = _time_get();
		if (GetProdRunMode() == RUN_REAL)
		{
			if (NozzleType == 0) // Suction
			{
				Err = SuctionOne(Gantry, HeadNo[PickOrd], true);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] Pick NozzleType(%d) SuctionOne Err:%d\n"), NozzleType, Err);
					return Err;
				}
			}
		}
		PickOffset[PickOrd].z = PickOffset[PickOrd].z - Nozzle[PickOrd].TipHeight + Nozzle[PickOrd].PusherHeight;
		if (GetProdRunMode() == RUN_DRY)
		{
			PickOffset[PickOrd].z -= GetDryRunZHeightOffset();
		}
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] PickOrd(%d) Run TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
				PickOrd,
				Nozzle[PickOrd].TipHeight, Nozzle[PickOrd].PusherHeight, PickOffset[PickOrd].z);
		}
		if (GetProdRunMode() == RUN_REAL)
		{
			if (ReadyNo[PickOrd] > 0)
			{
				PickupReadyGetTime = _time_get();
				while (1)
				{
					if (bManualRecog == true)
					{
						if (GetGlobalStatusError() == true)
						{
							TRACE(_T("[PWR] Pick Feeder(%03d) Ready Wait StopNow\n"), FdNo[PickOrd]);
							return STOP_NOW;
						}
					}
					else
					{
						if (GetGlobalStatusError() == true || GetMachineState() == STATE_STOPNOW || GetRunModeNoLog() == NORMAL_MODE)
						{
							TRACE(_T("[PWR] Pick Feeder(%03d) Ready Wait StopNow\n"), FdNo[PickOrd]);
							return STOP_NOW;
						}
					}

					if (pFeeder[PickOrd]->GetStep() == FeederStep::READY)
					{
						SetEmptyError(PickOrd + 1, 0);
						SetReadyTimeOutEmpty(FdNo[PickOrd], 0);
						SetReadyTimeOutEmptyByHeadNo(HeadNo[PickOrd], 0);
						pFeeder[PickOrd]->SetEmtpy(false);
						break;
					}
					PickupReadyElapsed = _time_elapsed(PickupReadyGetTime);
					if (PickupReadyElapsed > ReadyTimeOut[PickOrd])
					{
						SetEmptyError(PickOrd + 1, 1);
						SetReadyTimeOutEmpty(FdNo[PickOrd], 1);
						SetReadyTimeOutEmptyByHeadNo(HeadNo[PickOrd], 1);

						if (GetFeederEmptyDisplay(FdNo[PickOrd]) == false)
						{
							SetFeederEmptyDisplay(FdNo[PickOrd], true);
							SendEmpty(FdNo[PickOrd], EMPTY_READY_TIMEOUT, _T("Ready IO TimeOut"));
						}

						pFeeder[PickOrd]->SetEmtpy(true);
						break;
					}
					ThreadSleep(TIME1MS);
				}
				if (GetReadyTimeOutEmpty(FdNo[PickOrd]) == 1)
				{
					TRACE(_T("[PWR] Pick Feeder(%03d) ReadyIO TimeOut(%d)\n"), FdNo[PickOrd], ReadyTimeOut[PickOrd]);
					Err = WaitR(strRAxis, PickOffset[PickOrd].r, InposR, TimeOut);
					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] Pick(%s) WaitR Err:%d\n"), strRAxis, Err);
					}
					continue;
				}
			}
		}
		Err = WaitR(strRAxis, PickOffset[PickOrd].r, InposR, TimeOut);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Pick WaitR Err"));
			TRACE(_T("[PWR] Pick(%s) WaitR Err,%d\n"), strRAxis, Err);
			return Err;
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TRACE(_T("[PWR] Pick(%s) R Elapsed,%d\n"), strRAxis, Elapsed);
		}

		GetTime = _time_get();
		if (bFirstMoveXY == true) // 첫번째 Pickup 시에 XY도착하고 다른 Head도 미리 Z축을 내린다.
		{
			double lastPickMaxComp = gcLastPickFront->GetMaxHeight();

			for (long OtherZ = 0; OtherZ < GetMaxPickOrder(); ++OtherZ)
			{
				OtherHeadNo = GetPickupHeadNo(OtherZ + 1);
				OtherFdNo = GetFdNoFromPickOrder(OtherZ + 1);
				if (OtherHeadNo == 0) continue;
				if (OtherFdNo == 0) continue;
				if (GetReadyTimeOutEmpty(OtherFdNo) == 1) continue;

				PickHead[OtherHeadNo - 1] = true;
				TRACE(_T("[PWR] Pick Move Safety Head(%d)\n"), OtherHeadNo);

				strOtherZAxis = GetZAxisFromHeadNo(Gantry, OtherHeadNo);
				if (strOtherZAxis.CompareNoCase(strZAxis) == 0) continue;
				OtherNozzleNo = GetGlobalNozzleNo(OtherHeadNo);
				OtherNozzle = GetGlobalNozzleInformation(OtherNozzleNo);
				OtherPickupZStand = PickupZStandBy - OtherNozzle.TipHeight - lastPickMaxComp;

				Limit limitOtherZ = GetLimit(GetAxisIndexFromAliasName(strOtherZAxis));
				if (limitOtherZ.minus > OtherPickupZStand)
				{
					OtherPickupZStand = limitOtherZ.minus + 1.0;
				}

				Err = MoveZDown(strOtherZAxis, ZDownRatio, TimeOut, OtherPickupZStand, InposZDn, MsZDn, false);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("Pick OtherZPreDn Err"));
					TRACE(_T("[PWR] Pick(%s) OtherZPreDn Err,%d\n"), strOtherZAxis, Err);
					return Err;
				}
			}

			if (bManualRecog == false)
			{
				for (long head = 0; head < MAXUSEDHEADNO; head++)
				{
					if (PickHead[head] == true)
					{
						continue;
					}
					strOtherZAxis = GetZAxisFromHeadNo(Gantry, head + 1);
					OtherNozzleNo = GetGlobalNozzleNo(OtherHeadNo);
					OtherNozzle = GetGlobalNozzleInformation(OtherNozzleNo);
					OtherPickupZStand = gcReadJobFile->GetStandyBy().pt.z - OtherNozzle.TipHeight;
					CurPos = ReadPosition(strOtherZAxis);

					if (CurPos > OtherPickupZStand)
					{
						TRACE(_T("[PWR] Pick Move Safety Position(%s,%.3f->%.3f)\n"), strOtherZAxis, CurPos, OtherPickupZStand);
						Err = MoveZDown(strOtherZAxis, ZDownRatio, TimeOut, OtherPickupZStand, InposZDn, MsZDn, false);
					}
					else
					{
						TRACE(_T("[PWR] Pick Move Safety Skip Position(%s,%.3f->%.3f)\n"), strOtherZAxis, CurPos, OtherPickupZStand);

					}
				}
			}

			bFirstMoveXY = false;
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			CurPos = ReadPosition(strZAxis);
		}

		PickLevelInfo = gcStep->GetPickLevelCheck(FdNo[PickOrd]);
		//StartMonitor(strZAxis, 0, 0, 0);

		double torqueLimit = GetMaxZTorqueLimit(Gantry, HeadNo[PickOrd]);
		if (gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown.Use == true)
		{
			torqueLimit = gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown.TorqueLimit;
			RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo[PickOrd], torqueLimit, GetTorqueOverSafetyZ());
		}

		long insertNo = gcStep->GetUserPickOrderFromIndex(PickOrd + 1).InsertCurNo;
		long Block = gcReadJobFile->GetInsert(Gantry, insertNo).BlockNo;
		long index = gcReadJobFile->GetInsert(Gantry, insertNo).index;

		StartMonitor(strZAxis, GetProductionQuantity() + 1, Block, index, _T("Pick"));
		strFileName = GetTorqueMonitorFileName(strZAxis);
		//if (GetGlobalSimulationMode() == true)
		//{
		//	strFileName = _T("D:\\PowerMotion\\TorqueZ\\121951142_Board1_Block0_Idx1_Pick_FZ1.Monitor");
		//}
		ErrTorque = GetZTorqueAlarmcode(Gantry, HeadNo[PickOrd]);

		if (TwoStepPick.Use == 1)
		{
            const double currentPosition = ReadPosition(strZAxis);
            const bool startFast = TwoStepPick.Dist < (PickOffset[PickOrd].z - currentPosition);//현재 위치에서 픽업 위치까지의 거리가 TwoStepPick.Dist보다 멀 때만 빠르게 하강 시작(true).
			Err = startFast ? MoveZDown(strZAxis, ZDownRatio, TimeOut, (PickOffset[PickOrd].z - TwoStepPick.Dist), InposZDn, MsZDn, false) : Err;
			if (Err != NO_ERR)
			{
				StopMonitor(strZAxis);

				Err = SendAlarm(Err, _T("Pick ZDn1st Err"));
				TRACE(_T("[PWR] Pick(%s) ZDn1st Err,%d\n"), strZAxis, Err);
				return Err;
			}

			Err = startFast ? WaitZDown(strZAxis, (PickOffset[PickOrd].z - TwoStepPick.Dist), InposZDn, TimeOut, false) : Err;
			if (Err != NO_ERR)
			{
				if (Err == ErrTorque)
				{
					Err = GetZTorquePickFailAlarmcode(Gantry, HeadNo[PickOrd]);
				}

				StopMonitor(strZAxis);
				ThreadSleep(TIME50MS);
				WaitStopMonitor(strZAxis);
				SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);

				ThreadSleep(TIME50MS);

				Err = SendAlarm(Err, _T("Insert ZDn1st Err"));
				TRACE(_T("[PWR] Insert(%s) ZDn1st Err,%d\n"), strZAxis, Err);
				return Err;
			}

			if (IsLabelTypeFeeder(FeederType) == true && Nozzle[PickOrd].Type == 0 && PickLevelInfo.Use == true)
			{
				while (_time_elapsed(GetTimeSuction) < TIME500MS)
				{
					ThreadSleep(TIME1MS);
				}

				LevelBefore = GetAnalogLevel(FRONT_GANTRY, HeadNo[PickOrd]);
			}			

			if (gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown2nd.Use == true)
			{
				torqueLimit = gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown2nd.TorqueLimit;
				RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo[PickOrd], torqueLimit, GetTorqueOverSafetyZ());
			}

			Err = MoveZDown(strZAxis, TwoStepPick.Ratio, TimeOut, PickOffset[PickOrd].z, InposZDn, MsZDn, false);
			if (Err != NO_ERR)
			{
				StopMonitor(strZAxis);

				Err = SendAlarm(Err, _T("Pick ZDn2nd Err"));
				TRACE(_T("[PWR] Pick(%s) ZDn2nd Err,%d\n"), strZAxis, Err);
				return Err;
			}

			Err = WaitZDown(strZAxis, PickOffset[PickOrd].z, InposZDn, TimeOut, false);
			if (Err != NO_ERR)
			{
				StopMonitor(strZAxis);

				if (Err == ErrTorque)
				{
					Err = GetZTorquePickFailAlarmcode(Gantry, HeadNo[PickOrd]);
				}

				StopMonitor(strZAxis);
				ThreadSleep(TIME50MS);
				WaitStopMonitor(strZAxis);
				SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);

				Err = SendAlarm(Err, _T("Pick ZDn2nd Err"));
				TRACE(_T("[PWR] Pick(%s) ZDn2nd Err,%d\n"), strZAxis, Err);
				return Err;
			}
		}
		else
		{
			if (IsLabelTypeFeeder(FeederType) == true && Nozzle[PickOrd].Type == 0 && PickLevelInfo.Use == true)
			{
				while (_time_elapsed(GetTimeSuction) < TIME500MS)
				{
					ThreadSleep(TIME1MS);
				}

				LevelBefore = GetAnalogLevel(FRONT_GANTRY, HeadNo[PickOrd]);				
			}

			Err = MoveZDown(strZAxis, ZDownRatio, TimeOut, PickOffset[PickOrd].z, InposZDn, MsZDn, false);
			if (Err != NO_ERR)
			{
				StopMonitor(strZAxis);
				Err = SendAlarm(Err, _T("Pick ZDn Err"));
				TRACE(_T("[PWR] Pick(%s) ZDn Err,%d\n"), strZAxis, Err);
				return Err;
			}

			Err = WaitZDown(strZAxis, PickOffset[PickOrd].z, InposZDn, TimeOut, false);
			if (Err != NO_ERR)
			{
				if (Err == ErrTorque)
				{
					Err = GetZTorquePickFailAlarmcode(Gantry, HeadNo[PickOrd]);
				}

				StopMonitor(strZAxis);
				ThreadSleep(TIME50MS);
				WaitStopMonitor(strZAxis);
				SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);

				ThreadSleep(TIME50MS);

				Err = SendAlarm(Err, _T("Pick ZDn Err"));
				TRACE(_T("[PWR] Pick(%s) ZDn Err,%d\n"), strZAxis, Err);
				return Err;
			}
		}
		StopMonitor(strZAxis);
		if (gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown.Use == true || gcStep->GetPartTorqueLimit(FdNo[PickOrd]).PickDown2nd.Use == true)
		{
			RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo[PickOrd], GetMaxZTorqueLimit(Gantry, HeadNo[PickOrd]), GetTorqueOverSafetyZ());
		}

		//StopMonitor(strZAxis);
		//strFileName = GetTorqueMonitorFileName(strZAxis);
		//SendToZTorqueMonitorFile(strFileName);
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TRACE(_T("[PWR] Pick(%s) Dist:%.1f ZDn Elapsed,%d\n"), strZAxis, PickOffset[PickOrd].z - CurPos, Elapsed);
		}
		if (GetProdRunMode() == RUN_REAL)
		{
			if (NozzleType == 1) // Gripper
			{
				Err = SuctionOne(Gantry, HeadNo[PickOrd], true);
				if (Err != NO_ERR)
				{
					TRACE(_T("[PWR] Pick NozzleType(%d) SuctionOne Err,%d\n"), NozzleType, Err);
					return Err;
				}
			}
		}
        //집었을때는 반드시 해야함.
        Err = gcLastPickFront->setDiscardInfo(HeadNo[PickOrd], CLastPick::DiscardInfo{ FdNo[PickOrd], gcReadJobFile->GetDiscard(FdNo[PickOrd]).pt }, true);
        if (Err != NO_ERR)
        {
            Err = SendAlarm(Err, L"setDiscardInfo failed..");
            return Err;
        }
		ThreadSleep(JobPickDelay[PickOrd]);
		ReleaseIO = GetReleaseIONoFromReleaseNo(ReleaseNo[PickOrd]);
		if (GetProdRunMode() == RUN_REAL)
		{
			if (ReleaseNo[PickOrd] > 0)
			{
				TRACE(_T("[PWR] Pick Fd:%d ReleaseNo:%d OUTON\n"), FdNo[PickOrd], ReleaseNo[PickOrd]);
				OutputOne(ReleaseIO, OUTON);

				bool readyOff = false;

				for (long waitCnt = 0; waitCnt < 1000; waitCnt++)
				{
					if (pFeeder[PickOrd]->GetStep() == FeederStep::NOTREADY)
					{
						TRACE(_T("[PWR] Pick Fd:%d FeederStep::NOTREADY OK\n"), FdNo[PickOrd]);
						readyOff = true;
						break;
					}
					ThreadSleep(TIME1MS);
				}

				if (readyOff == false)
				{
					TRACE(_T("[PWR] Pick Fd:%d FeederStep::NOTREADY Error\n"), FdNo[PickOrd]);
				}
				//ThreadSleep(ReadyWaitDelay[PickOrd]);
			}
		}
		PickupWithComponentZ = GetInsertByZ(Gantry) - GetMaxComponentHeight() - Nozzle[PickOrd].TipHeight + Nozzle[PickOrd].PusherHeight - ComponentBodyHeight - ComponentLeadHeight;
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] Pick Pickup(%s) Target,%.3f MaxCompT,%.3f Tip,%.3f Pusher,%.3f Body,%.3f LeadHeight,%.3f\n"), strZAxis,
				PickupWithComponentZ, GetMaxComponentHeight(), Nozzle[PickOrd].TipHeight, Nozzle[PickOrd].PusherHeight, ComponentBodyHeight, ComponentLeadHeight);
		}
		if (PickupWithComponentZ > SAFTY_ZHEIGHT)
		{
			PickupWithComponentZ = PickupZStandBy;
			TRACE(_T("[PWR] Pick Pickup(%s) Safty Pickup,%.3f\n"), strZAxis, PickupWithComponentZ);
		}
		if (GetProdRunMode() == RUN_DRY)
		{
			PickupWithComponentZ -= GetDryRunZHeightOffset();
		}
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > PickupWithComponentZ)
		{
			TRACE(_T("[PWR] Pick ZUp Target position,%.3f is under minus limit,%.3f\n"), PickupWithComponentZ, limit.minus);
			PickupWithComponentZ = limit.minus + 1.0;			
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			CurPos = ReadPosition(strZAxis);
		}
		GetTime = _time_get();

		if (bManualRecog == true)
		{
			if (GetDivideInspect(FdNo[PickOrd]).Use == true)
			{
				TRACE(_T("[PWR] Manual Pick Divid ZUp %.3f\n"), PickupWithComponentZ);

				Err = MoveZUp(strZAxis, CompRatio[PickOrd].z, TimeOut, PickupWithComponentZ, InposZUp, MsZUp, true);
			}
			else
			{
				Err = ManualRecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), CompRatio[PickOrd].z, TIME5000MS);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("Pick ManualRecognitionMoveAllZUp Err"));
					TRACE(_T("[PWR] Pick ManualRecognitionMoveAllZUp Err,%d\n"), Err);
					return Err;
				}
			}

			LevelAfter = GetAnalogLevel(FRONT_GANTRY, HeadNo[PickOrd]);
			MeasureDifferntLevel = (LevelAfter - LevelBefore) * DifferntLevelMargin;
			SendSuctionDifferentLevel(FdNo[PickOrd], (long)MeasureDifferntLevel);
			TRACE(_T("[PWR] PickLabel %s Feeder:%d MeasureLevel Before:%d After:%d Result:%d\n"), (LPCTSTR)strZAxis, FdNo[PickOrd], LevelBefore, LevelAfter, (long)MeasureDifferntLevel);

		}
		else
		{
			Err = MoveZUp(strZAxis, CompRatio[PickOrd].z, TimeOut, PickupWithComponentZ, InposZUp, MsZUp, true);

			if (IsLabelTypeFeeder(FeederType) == true && GetProdRunMode() != RUN_DRY)
			{
				NowPocket++;
				SetTrayNowPocket(FdNo[PickOrd], NowPocket);
				SendTrayLastPocket(FdNo[PickOrd], NowPocket);
				double DiffEmpty = PickLevelInfo.DifferentLevel * EmptyLevelMargin;
				if (PickLevelInfo.Use == true && NozzleType == 0)
				{
					LevelAfter = GetAnalogLevel(FRONT_GANTRY, HeadNo[PickOrd]);

					if (LevelAfter > LevelBefore + PickLevelInfo.DifferentLevel)
					{
						PickLevelInfo.FirstPocketFind = true;
						gcStep->SetPickLevelCheck(FdNo[PickOrd], PickLevelInfo);
						TRACE(_T("[PWR] PickLabel %s Feeder:%d PickOK Before:%d After:%d\n"), (LPCTSTR)strZAxis, FdNo[PickOrd], LevelBefore, LevelAfter);

						//if (MaxPocket <= NowPocket)
						//{
						//	pFeeder[PickOrd]->SetStep(FeederStep::WAIT_READYOFF);
						//}

					}
					else if (PickLevelInfo.FirstPocketFind == false && NowPocket < MaxPocket && LevelAfter < (LevelBefore + (long)DiffEmpty))
					{
						TRACE(_T("[PWR] PickLabel %s Feeder:%d TryNext Before:%d After:%d Empty:%d\n"), (LPCTSTR)strZAxis, FdNo[PickOrd], LevelBefore, LevelAfter, (long)DiffEmpty);

						// 정말 픽업 실패 && 다음꺼 픽업해보자
						PickOrd--;
						continue;
					}
					else
					{
						TRACE(_T("[PWR] PickLabel %s Feeder:%d PickUp NG Before:%d After:%d\n"), (LPCTSTR)strZAxis, FdNo[PickOrd], LevelBefore, LevelAfter);

						CString strMsg;
						strMsg.Format(_T("%s Feeder%d Suction Level Error"), (LPCTSTR)strZAxis, FdNo[PickOrd]);
						Err = PICK_DIFFERENT_LEVEL_ERROR(FdNo[PickOrd]);
						Err = SendAlarm(Err, strMsg);
						return Err;
					}
				}
			}
			else if (IsTrayTypeFeeder(FeederType) == true && GetProdRunMode() != RUN_DRY)
			{
				NowPocket++;
				SetTrayNowPocket(FdNo[PickOrd], NowPocket);

				if (bManualRecog == false)
				{
					SendTrayLastPocket(FdNo[PickOrd], NowPocket);
				}
			}

		}
		SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);
		if (bManualRecog == false)
		{
			SendToPickupStatisticsComplete(FdNo[PickOrd], NozzleNo);

			User_PickOrder pickInfo = gcStep->GetUserPickOrderFromIndex(PickOrd + 1);
			INSERT insertInfo = gcReadJobFile->GetInsert(pickInfo.InsertCurNo);
			long package = gcReadJobFile->GetPackgeIndexFromFeederNo(insertInfo.FeederNo);

			LASTPICK lastPick;

			if (gcReadJobFile->GetPcb().UseLastPickReUse == 1)
			{
				lastPick.Enable = true;
			}
			else
			{
				lastPick.Enable = false;
			}

			lastPick.Insert = insertInfo;
			lastPick.Package = gcReadJobFile->GetPackage(package);
			gcLastPickFront->SetHeadData(HeadNo[PickOrd], lastPick);
		}

		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Pick ZUp Err"));
			TRACE(_T("[PWR] Pick(%s) ZUp Err,%d\n"), strZAxis, Err);
			return Err;
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TargetPos = ReadPosition(strZAxis);
			TRACE(_T("[PWR] Pick(%s) Dist:%.3f ZUp Elapsed,%d\n"), strZAxis, TargetPos - CurPos, Elapsed);
		}
		if (GetProdRunMode() == RUN_REAL)
		{
			if (ReleaseNo[PickOrd] > 0)
			{
				OutputOne(ReleaseIO, OUTOFF);
				//ThreadSleep(ReadyWaitDelay[PickOrd]);
			}
		}
	}

	if (gcStep->GetRecogTableBy1stInsertOrder() == FRONT_STAGE)
	{
		Err = gPartDropGetResult(FRONT_STAGE);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Drop object on the camera"));
			return Err;
		}
	}

	if (Err != NO_ERR)
	{
		for (long PickOrd = 0; PickOrd < GetMaxPickOrder(); ++PickOrd)
		{
			HeadNo[PickOrd] = GetPickupHeadNo(PickOrd + 1);
			FdNo[PickOrd] = GetFdNoFromPickOrder(PickOrd + 1);
			if (HeadNo[PickOrd] == 0) continue;
			if (FdNo[PickOrd] == 0) continue;
			ReadyTimeOutEmpty = GetReadyTimeOutEmpty(FdNo[PickOrd]);
			if (ReadyTimeOutEmpty > 0)
			{
				Err = READYTIMEOUTEMPTY(FdNo[PickOrd]);
				break;
			}
		}
	}
	return Err;
}

Ratio_XYRZ CPick::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio = gcStep->GetMinRatio();
	return MinRatio;
}


Ratio_XYRZ CPick::GetPickRatio(long PickOrder)
{
	Ratio_XYRZ PickRatio;
	PickRatio = gcStep->GetPickRatio(PickOrder);
	return PickRatio;
}

long CPick::SetTrayNowPocket(long FeederNo, long NowPocket)
{
	long Err = NO_ERR;
	gcStep->SetTrayNowPocket(FeederNo, NowPocket);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetTrayNowPocket FeederNo:%d NowPocket:%d\n"), FeederNo, NowPocket);
	}
	return Err;
}

bool CPick::WaitTargetLevel(long HeadNo, long TargetLevel, long LowHigh, long WaitTime)
{
	long level = 0;
	long time = 0;

	while(1)
	{
		level = GetAnalogLevel(FRONT_GANTRY, HeadNo);

		if (LowHigh == 0 && level < TargetLevel) // low
		{
			TRACE(_T("[PWR] WaitTargetLevel Head:%d Target:%d LowHigh:%d Result:%d Wait:%d\n"), HeadNo, TargetLevel, LowHigh, level, time);

			return true;
		}
		else if (LowHigh == 1 && level > TargetLevel) // high
		{
			TRACE(_T("[PWR] WaitTargetLevel Head:%d Target:%d LowHigh:%d Result:%d Wait:%d\n"), HeadNo, TargetLevel, LowHigh, level, time);

			return true;
		}

		time++;
		if (time > WaitTime)
		{
			break;
		}
		ThreadSleep(TIME1MS);
		
	}

	TRACE(_T("[PWR] WaitTargetLevel fail. Head:%d Target:%d LowHigh:%d Result:%d Wait:%d\n"), HeadNo, TargetLevel, LowHigh, level, time);

	return false;
}

//void CPick::PartDropLedOn(long CamTable)
//{
//	if (gcReadJobFile->GetPartDrop().Use == false) return;
//
//	MODULE_LED led = gcReadJobFile->GetPartDrop().led;
//
//	if (CamTable == FRONT_GANTRY)
//	{
//		gLedOn(CAM1, led.Top, led.Mid, led.Bot);
//		gLedOn(CAM2, led.Top, led.Mid, led.Bot);
//	}
//	else
//	{
//		gLedOn(RCAM1, led.Top, led.Mid, led.Bot);
//		gLedOn(RCAM2, led.Top, led.Mid, led.Bot);
//	}
//
//	TRACE(_T("[PWR] PartDropLedOn(%d) Led:%d,%d,%d\n"), CamTable, led.Top, led.Mid, led.Bot);
//
//}
//
//
//void CPick::PartDropProcess(long CamTable)
//{	
//	if (gcReadJobFile->GetPartDrop().Use == false) return;
//
//	PowerThreadMessage* msgSend = new PowerThreadMessage();
//	ThreadId_t id;
//	msgSend->SetThreadMsg(_T("0"));
//	msgSend->SetThreadSubMsg(0, 0, 0);
//	if (gcCamDropCheck[CamTable])
//	{
//		gcCamDropCheck[CamTable]->InitDropResult();
//
//		gcCamDropCheck[CamTable]->GetId(&id);
//		msgSend->SetID(id);
//		if (gcCamDropCheck[CamTable]->PingThread(TIME1MS))
//		{
//			gcCamDropCheck[CamTable]->Event((LPVOID)msgSend);
//			TRACE(_T("[PWR] PartDropProcess(%d) Send ProcessMsg\n"), CamTable);
//		}
//	}
//}
//
//long CPick::PartDropGetResult(long CamTable)
//{
//	if (gcReadJobFile->GetPartDrop().Use == false) return NO_ERR;
//
//	ULONGLONG time = _time_get();
//	long result = NO_ERR;
//	long timeOut = TIME1000MS;
//
//	while (1)
//	{
//		if (_time_elapsed(time) > timeOut)
//		{
//			TRACE(_T("[PWR] PartDropGetResult(%d) TimeOut\n"), CamTable);
//
//			return FCAMERA_DROP_ERROR + CamTable;
//		}
//
//		if (gcCamDropCheck[CamTable]->GetDropResult(&result) == true)
//		{
//			TRACE(_T("[PWR] PartDropGetResult(%d) Result:%d WaitTime:%d\n"), CamTable, result, _time_elapsed(time));
//
//			return result;
//		}
//
//		ThreadSleep(TIME1MS);
//	}
//
//	return NO_ERR;
//}

long CPick::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CPick::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

void CPick::SetFeederEmptyDisplay(long FeederNo, bool set)
{
	gcStep->SetFeederEmptyDisplay(FeederNo, set);
}

bool CPick::GetFeederEmptyDisplay(long FeederNo)
{
	return gcStep->GetFeederEmptyDisplay(FeederNo);
}

DIVIDE_INSPECT CPick::GetDivideInspect(long FeederNo)
{
	return gcStep->GetDivideInspect(FeederNo);
}

Ratio_XYRZ CPick::GetMinRatioLastPickData()
{
	long Gantry = m_Gantry;
	LASTPICK data;
	long insertNo = MAXINSERTNO;
	Ratio_XYRZ MinRatio;
	Ratio_XYRZ tempRatio;

	MinRatio.xy = MinRatio.z = MinRatio.r = 1.0;

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
		data = gcLastPickFront->GetHeadData(head);
		insertNo = gcReadJobFile->GetInsertNo(data.Insert.BlockNo, data.Insert.index);

		if (data.Enable == true && insertNo > 0)
		{
			tempRatio = gcStep->GetRatioFromFdNo(data.Insert.FeederNo);
			if (tempRatio.xy < MinRatio.xy)
			{
				MinRatio.xy = tempRatio.xy;
			}

			if (tempRatio.r < MinRatio.r)
			{
				MinRatio.r = tempRatio.r;
			}

			if (tempRatio.z < MinRatio.z)
			{
				MinRatio.z = tempRatio.z;
			}
		}
	}

	TRACE(_T("[PWR] LastPickData Pick MinRatio XY %.1f R %.1f Z %.1f\n"), MinRatio.xy * 100.0, MinRatio.r * 100.0, MinRatio.z * 100.0);
	return MinRatio;
}