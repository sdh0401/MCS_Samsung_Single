#include "pch.h"
#include "CRecognitionDivide.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CAdvancedMotionFile.h"
#include "CStep.h"
#include "Trace.h"
#include "CReadJobFile.h"

CRecognitionDivide* gcRecognitionDivide;
CRecognitionDivide::CRecognitionDivide(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxInsertOrder = 0;
	m_ProdRunMode = RUN_REAL;
	ZeroMemory(&m_FeederNo, sizeof(m_FeederNo));
	ZeroMemory(&m_InsertOrder, sizeof(m_InsertOrder));
	ZeroMemory(&m_RecognitionTable, sizeof(m_RecognitionTable));
	ClearManualVisionOffset();
}

CRecognitionDivide::~CRecognitionDivide()
{
}

void CRecognitionDivide::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CRecognitionDivide::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CRecognitionDivide::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CRecognitionDivide::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CRecognitionDivide::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CRecognitionDivide::GetFeederNoFromHeadNo(long HeadNo)
{
	long FeederNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		FeederNo = m_FeederNo[HeadNo - 1];
	}
	return FeederNo;
}

long CRecognitionDivide::GetInsertOrderFromHeadNo(long HeadNo)
{
	long InsertOder = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		InsertOder = m_InsertOrder[HeadNo - 1];
	}
	return InsertOder;
}

long CRecognitionDivide::GetRecognitionTableFromHeadNo(long HeadNo)
{
	long RecogTable = FRONT_GANTRY;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		RecogTable = m_RecognitionTable[HeadNo - 1];
	}
	RecogTable = FRONT_GANTRY;
	return RecogTable;
}

long CRecognitionDivide::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	return ReadyTimeOutEmpty;
}

Ratio_XYRZ CRecognitionDivide::GetRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

double CRecognitionDivide::GetVAAngleFromFdNo(long insertOrd)
{
	double RecognitionAngle = 0.0;
	RecognitionAngle = gcStep->GetVAAngleFromFdNo(insertOrd);
	return RecognitionAngle;
}

double CRecognitionDivide::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionDivide::GetComponentLeadHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionDivide::GetVAOffsetHeight(long FdNo)
{
	double VARecognitionOffsetHeight = 0.0;
	VARecognitionOffsetHeight = gcStep->GetVAOffsetHeight(FdNo);
	return VARecognitionOffsetHeight;
}

double CRecognitionDivide::GetPickupZStandBy() {
	double PickupZStandBy = 0.0;
	PickupZStandBy = gcStep->GetPickupZStandBy();
	return PickupZStandBy;
}

MODULE_LED CRecognitionDivide::GetLed(long FdNo)
{
	MODULE_LED Led;
	Led = gcStep->GetLed(FdNo);
	return Led;
}

long CRecognitionDivide::GetManualCompensationUse()
{
	long ManualCompenUse = 0;
	ManualCompenUse = gcStep->GetManualCompensationUse();
	return ManualCompenUse;
}

Point_XYRZ CRecognitionDivide::GetManualVisionResult()
{
	Point_XYRZ ManualVisionResult;
	ManualVisionResult = gcStep->GetManualVisionResult();
	return ManualVisionResult;
}

long CRecognitionDivide::ClearManualVisionOffset()
{
	ZeroMemory(&m_VAAngleOffset, sizeof(m_VAAngleOffset));
	return NO_ERR;
}

long CRecognitionDivide::SetManualVisionOffset(long insertOrd, double OffsetAngle)
{
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_VAAngleOffset[insertOrd - 1] = OffsetAngle;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetManualVisionOffset insertOrd:%d OffsetAngle:%.3f\n"), insertOrd, OffsetAngle);
		}
	}
	return NO_ERR;
}

double CRecognitionDivide::GetManualVisionOffset(long insertOrd)
{
	double OffsetAngle = 0.0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		OffsetAngle = m_VAAngleOffset[insertOrd - 1];
	}
	return OffsetAngle;
}

long CRecognitionDivide::SetFeederNoFromHeadNo(long HeadNo, long FeederNo)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_FeederNo[HeadNo - 1] = FeederNo;
	}
	return NO_ERR;
}

long CRecognitionDivide::SetInsertOrderFromHeadNo(long HeadNo, long InsertOrder)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_InsertOrder[HeadNo - 1] = InsertOrder;
	}
	return NO_ERR;
}

long CRecognitionDivide::SetRecognitionTableFromHeadNo(long HeadNo, long RecogTable)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_RecognitionTable[HeadNo - 1] = RecogTable;
	}
	return NO_ERR;
}

long CRecognitionDivide::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CRecognitionDivide::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

long CRecognitionDivide::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
	return VisionErrorEmpty;
}

long CRecognitionDivide::ClearVisionErrorEmpty(long FeederNo)
{
	long Err = NO_ERR;
	gcStep->ClearVisionErrorEmpty(FeederNo);
	return Err;
}

long CRecognitionDivide::SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty)
{
	long Err = NO_ERR;
	gcStep->SetVisionErrorEmpty(FeederNo, VisionErrorEmpty);
	return Err;
}

long CRecognitionDivide::SetVisionError(long InsertOrd, long VisionError)
{
	long Err = NO_ERR;
	gcStep->SetVisionError(InsertOrd, VisionError);
	return Err;
}

long CRecognitionDivide::GetEmptyError(long InsertOrd)
{
	long EmptyError = 0;
	EmptyError = gcStep->GetEmptyError(InsertOrd);
	return EmptyError;
}

bool CRecognitionDivide::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	bSendEmpty = gcStep->GetSendEmpty(FeederNo);
	return bSendEmpty;
}

long CRecognitionDivide::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	long Err = NO_ERR;
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	return Err;
}

long CRecognitionDivide::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	return ReadyTimeOutEmpty;
}

long CRecognitionDivide::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CRecognitionDivide::ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionDivide::RecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionDivide::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, FHCAM, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CRecognitionDivide::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionDivide::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CRecognitionDivide::MoveOneZDn(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionDivide::MoveSomeZDn(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = StartSomeZAxisWaitDelayedPosSet(Gantry, Target);
	return Err;
}

long CRecognitionDivide::Run(bool bManualRecog)
{
	long Gantry = m_Gantry, insertNo = 0, RecogTable = FRONT_STAGE, OldRecogTable = FRONT_STAGE;
	Point_XY pt, CurPt;
	Point_XY ptDivide;
	Point_XYT ResPartSize;
	Point_XYRE res;
	long UseVA[MAXVAHEAD], FrameNo[MAXFRAMENO], CamNo[MAXVAHEAD], VisChk[MAXVAHEAD], VisDB[MAXVAHEAD];
	double AngleVA[MAXVAHEAD];
	long Forming[MAXVAHEAD];
	long Ret = 0, err = 0, FeederNo = 0, NozzleNo = 0;
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	Ratio_XYRZ CompRatio[MAXUSEDHEADNO], MinRatio, MoveRatio;
	SomeTarget Target;
	NOZZLE Nozzle[MAXUSEDHEADNO];
	long CatchOrder[MAXUSEDHEADNO], CatchIndex = 0, CatchUnit = 0;
	long CatchUseVA[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchCamNo[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchHeadNo[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchVisChk[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchVisDB[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	MODULE_LED Led[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	double CatchAngleVA[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchFormingVA[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	Ratio_XYRZ CatchCompRatio[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	double CatchComponentHeight[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	double CatchComponentLeadHeight[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	double CatchComponentVARecognitionOffsetHeight[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	NOZZLE CatchNozzle[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchDelay[MAXUSEDHEADNO][MAXCATCH_SIMUL];
	long CatchLaserControl[MAXCATCHNO][MAXCATCH_SIMUL];

	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Err = NO_ERR, insertOrd = 0, LedOrd = 0, CatchOrd = 0, MaxCatchOrd = 0, CatchMoveZOrd = 0, CatchMoveZIndex = 0, CatchMoveROrd = 0, CatchMoveRIndex = 0;
	double RatioZ = 1.0, Pitch = 1.0, ComponentHeight[MAXUSEDHEADNO], JobAngle[MAXUSEDHEADNO], ComponentLeadHeight[MAXUSEDHEADNO], ComponentVARecognitionOffsetHeight[MAXUSEDHEADNO];
	double AlignCheckingZ = GetInsertByZ(Gantry);
	long Cam[MAXUSEDHEADNO], ChkPos[MAXUSEDHEADNO], /*TargetHeadNo, */JobHeadNo[MAXUSEDHEADNO], JobFdNo[MAXUSEDHEADNO], JobDBNo[MAXUSEDHEADNO], RecognitionTable[MAXUSEDHEADNO];
	long CatchUnit1st = 0, CatchUnit2nd = 0, CatchUnit3rd = 0;
	long CatchUnit1stCatchOrd = 0, CatchUnit2ndCatchOrd = 0, CatchUnit3rdCatchOrd = 0;
	bool bFirstMoveR = true, bFirstMoveZ = true, bSaftyZUp = false;
	long FirstRecogTable = FRONT_GANTRY;
	long MaxCatchDelay = 0;
	double PitchResult = 0.0;
	CString HeadPackageName[MAXUSEDHEADNO];
	bool HeadUseVA[MAXUSEDHEADNO];
	long HeadCatchIndex[MAXUSEDHEADNO];
	long HeadCatchUnit[MAXUSEDHEADNO];
	long CatchIndexManual = 0, CatchUnitManual = 0;
	bool CatchIndexFirst = true;

	ZeroMemory(&UseVA, sizeof(UseVA));
	ZeroMemory(&res, sizeof(res));
	ZeroMemory(&CamNo, sizeof(CamNo));
	ZeroMemory(&CurPt, sizeof(CurPt));
	ZeroMemory(&VisChk, sizeof(VisChk));
	ZeroMemory(&VisDB, sizeof(VisDB));
	ZeroMemory(&FrameNo, sizeof(FrameNo));
	ZeroMemory(&AngleVA, sizeof(AngleVA));
	ZeroMemory(&Nozzle, sizeof(Nozzle));
	ZeroMemory(&ResPartSize, sizeof(ResPartSize));
	ZeroMemory(&Cam, sizeof(Cam));
	ZeroMemory(&Led, sizeof(Led));
	ZeroMemory(&ChkPos, sizeof(ChkPos));
	ZeroMemory(&JobFdNo, sizeof(JobFdNo));
	ZeroMemory(&JobDBNo, sizeof(JobDBNo));
	ZeroMemory(&JobAngle, sizeof(JobAngle));
	ZeroMemory(&JobHeadNo, sizeof(JobHeadNo));	
	ZeroMemory(&CompRatio, sizeof(CompRatio));
	ZeroMemory(&ComponentHeight, sizeof(ComponentHeight));
	ZeroMemory(&CatchOrder, sizeof(CatchOrder));
	ZeroMemory(&CatchUseVA, sizeof(CatchUseVA));
	ZeroMemory(&CatchCamNo, sizeof(CatchCamNo));
	ZeroMemory(&CatchHeadNo, sizeof(CatchHeadNo));
	ZeroMemory(&CatchVisChk, sizeof(CatchVisChk));
	ZeroMemory(&CatchVisDB, sizeof(CatchVisDB));
	ZeroMemory(&CatchAngleVA, sizeof(CatchAngleVA));
	ZeroMemory(&CatchCompRatio, sizeof(CatchCompRatio));
	ZeroMemory(&RecognitionTable, sizeof(RecognitionTable));
	ZeroMemory(&CatchComponentHeight, sizeof(CatchComponentHeight));
	ZeroMemory(&CatchComponentLeadHeight, sizeof(CatchComponentLeadHeight));
	ZeroMemory(&ComponentVARecognitionOffsetHeight, sizeof(ComponentVARecognitionOffsetHeight));
	ZeroMemory(&CatchComponentVARecognitionOffsetHeight, sizeof(CatchComponentVARecognitionOffsetHeight));
	ZeroMemory(&CatchNozzle, sizeof(CatchNozzle));
	ZeroMemory(&CatchDelay, sizeof(CatchDelay));
	ZeroMemory(&Forming, sizeof(Forming));
	ZeroMemory(&CatchFormingVA, sizeof(CatchFormingVA));
	ZeroMemory(&HeadUseVA, sizeof(HeadUseVA));
	ZeroMemory(&HeadCatchIndex, sizeof(HeadCatchIndex));
	ZeroMemory(&HeadCatchUnit, sizeof(HeadCatchUnit));
	ZeroMemory(&CatchLaserControl, sizeof(CatchLaserControl));

	//gClearRunVisionResult(FRONT_STAGE);
	//gClearRunVisionResult(REAR_STAGE);

	for (long i = 0; i < MAXUSEDHEADNO; i++)
	{
		HeadPackageName[i].Format(_T("Temp%d"),i);
	}

	if (gcAdvancedMotionFile->UseAdvanceMotion() == 1)
	{
		MsXy = gcAdvancedMotionFile->GetRecogDelayedMsXY();
		MsZDn = gcAdvancedMotionFile->GetRecogDelayedMsZDn();
		MsZUp = gcAdvancedMotionFile->GetRecogDelayedMsZUp();
		MsR = gcAdvancedMotionFile->GetRecogDelayedMsR();
		InposXY = gcAdvancedMotionFile->GetRecogInposXY();
		InposR = gcAdvancedMotionFile->GetRecogInposR();
		InposZDn = gcAdvancedMotionFile->GetRecogInposZDn();
		InposZUp = gcAdvancedMotionFile->GetRecogInposZUp();
		TRACE(_T("[PWR] Recognition Ms XY,%d R,%d ZDn,%d ZUp,%d Inpos XY,%.3f R,%.3f ZDn,%.3f ZUp,%.3f\n"),
			MsXy, MsR, MsZDn, MsZUp, InposXY, InposR, InposZDn, InposZUp);
	}
	for (insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		JobHeadNo[insertOrd] = GetHeadNoFromInsertOrder(insertOrd + 1);

		if (GetReadyTimeOutEmptyByHeadNo(JobHeadNo[insertOrd]) == 1)
		{
			TRACE(_T("[PWR] ************************************************************\n"));
			TRACE(_T("[PWR] VisSkip Hd:%02d Fd:%03d NG ReadyIO TimeOut(%d)\n"), JobHeadNo[insertOrd], GetFdNoFromInsertOrder(insertOrd + 1), GetReadyTimeOutEmptyByHeadNo(JobHeadNo[insertOrd]));
			TRACE(_T("[PWR] ************************************************************\n"));
			JobHeadNo[insertOrd] = 0;
			continue;
		}

		JobFdNo[insertOrd] = GetFdNoFromInsertOrder(insertOrd + 1);

		if (GetDivideInspect(JobFdNo[insertOrd]).Use == false)
		{
			TRACE(_T("[PWR] SingleInspectSkip Hd:%02d Fd:%03d\n"), JobHeadNo[insertOrd], JobFdNo[insertOrd]);
			JobHeadNo[insertOrd] = 0;
			JobFdNo[insertOrd] = 0;
			continue;
		}

		JobDBNo[insertOrd] = JobFdNo[insertOrd];
		SetInsertOrderFromHeadNo(JobHeadNo[insertOrd], insertOrd + 1);
		SetFeederNoFromHeadNo(JobHeadNo[insertOrd], JobDBNo[insertOrd]);
		ChkPos[insertOrd] = GetCameraChkPosByHead(JobHeadNo[insertOrd]);
		CompRatio[insertOrd] = GetRatioByFdNo(JobFdNo[insertOrd]);
		JobAngle[insertOrd] = GetVAAngleFromFdNo(JobFdNo[insertOrd]);
		ComponentHeight[insertOrd] = GetComponentHeight(JobFdNo[insertOrd]);
		ComponentLeadHeight[insertOrd] = GetComponentLeadHeight(JobFdNo[insertOrd]);
		ComponentVARecognitionOffsetHeight[insertOrd] = GetVAOffsetHeight(JobFdNo[insertOrd]);

		HeadPackageName[JobHeadNo[insertOrd] - 1] = GetPackageName(JobFdNo[insertOrd]);
		HeadUseVA[JobHeadNo[insertOrd] - 1] = true;

		insertNo = GetInsertNoFromInsertOrder(insertOrd + 1);
		if (insertOrd == 0)
		{
			FirstRecogTable = GetRecognitionTable(insertNo);
		}
		else
		{
			if (FirstRecogTable != GetRecognitionTable(insertNo))
			{
				Err = RECOGTABLE_FAIL;
				SendAlarm(Err, _T("Recognition Table Error"));
				TRACE(_T("[PWR] Recognition Table Error. Insert:%d FirstRecogTable:%d CurrRecogTable:%d\n"), insertNo, FirstRecogTable, GetRecognitionTable(insertNo));
				return Err;
			}
		}

		RecognitionTable[insertOrd] = FirstRecogTable;
		SetRecognitionTableFromHeadNo(JobHeadNo[insertOrd], RecognitionTable[insertOrd]);
		Cam[insertOrd] = GetCameraNoByHead(RecognitionTable[insertOrd], JobHeadNo[insertOrd]) + 1;
		NozzleNo = GetGlobalNozzleNo(JobHeadNo[insertOrd]);
		Nozzle[insertOrd] = GetGlobalNozzleInformation(NozzleNo);

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] InsertOrd:%d Head:%d Fd:%d Db:%d Cam:%d ChkPos:%d ComponentT:%.3f LeadT:%.3f RatioXY,R,Z:%03d%%,%03d%%,%03d%%,Angle:%.3f\n"),
				insertOrd, JobHeadNo[insertOrd], JobFdNo[insertOrd], JobDBNo[insertOrd], Cam[insertOrd],
				ChkPos[insertOrd], ComponentHeight[insertOrd], ComponentLeadHeight[insertOrd],
				(long)(CompRatio[insertOrd].xy * 100), (long)(CompRatio[insertOrd].r * 100), (long)(CompRatio[insertOrd].z * 100),
				JobAngle[insertOrd]);
			TRACE(_T("[PWR] InsertNo:%d RecognitionTable:%d\n"), insertNo, RecognitionTable[insertOrd]);
			TRACE(_T("[PWR] Nozzle(%d,%d,%.3f,%.3f)(%d,%d,%d,%d)\n"),
				Nozzle[insertOrd].No, Nozzle[insertOrd].Type,
				Nozzle[insertOrd].TipHeight, Nozzle[insertOrd].PusherHeight,
				Nozzle[insertOrd].Empty, Nozzle[insertOrd].EmptyDiff,
				Nozzle[insertOrd].Exist, Nozzle[insertOrd].ExistDiff);
		}
		gInitRunVisionAngle(Gantry, insertOrd);
		gSetRunVisionAngle(Gantry, insertOrd, JobAngle[insertOrd]);
		gSetRunVisionResult(Gantry, insertOrd, res);
	}
	MinRatio = GetMinRatio();
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;

	long sort1st, sort2nd, min, buf;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < GetMaxInsertOrder(); ++sort1st)
		{
			TRACE(_T("[PWR] Before Sorting Index(%d) HeadNo:%d\n"), sort1st + 1, JobHeadNo[sort1st]);
		}
	}
	for (sort1st = 0; sort1st < GetMaxInsertOrder() - 1; ++sort1st)
	{
		min = sort1st;
		for (sort2nd = sort1st + 1; sort2nd < GetMaxInsertOrder(); ++sort2nd)
		{
			if (JobHeadNo[sort2nd] < JobHeadNo[sort1st])
			{
				min = sort2nd;
				SWAP(JobHeadNo[sort1st], JobHeadNo[sort2nd], buf);
			}
		}
	}

	long JobHeadNoInitial[MAXUSEDHEADNO], JobHeadNo2nd[MAXUSEDHEADNO];
	long TargetHeadNo = 0, JobHeadIndex = 0;

	ZeroMemory(&JobHeadNoInitial, sizeof(JobHeadNoInitial));
	ZeroMemory(&JobHeadNo2nd, sizeof(JobHeadNo2nd));

	JobHeadNoInitial[0] = TBL_HEAD1;
	JobHeadNoInitial[1] = TBL_HEAD4;
	JobHeadNoInitial[2] = TBL_HEAD2;
	JobHeadNoInitial[3] = TBL_HEAD5;
	JobHeadNoInitial[4] = TBL_HEAD3;
	JobHeadNoInitial[5] = TBL_HEAD6;

	for (sort1st = 0; sort1st < MAXUSEDHEADNO; ++sort1st)
	{
		TargetHeadNo = JobHeadNoInitial[sort1st];

		for (sort2nd = 0; sort2nd < GetMaxInsertOrder(); ++sort2nd)
		{
			if (JobHeadNo[sort2nd] == TargetHeadNo)
			{
				JobHeadNo2nd[JobHeadIndex] = TargetHeadNo;
				JobHeadIndex++;
				break;
			}
		}
	}

	CopyMemory(JobHeadNo, JobHeadNo2nd, sizeof(JobHeadNo));

	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < GetMaxInsertOrder(); ++sort1st)
		{
			TRACE(_T("[PWR] After  Sorting Index(%d) HeadNo:%d\n"), sort1st + 1, JobHeadNo[sort1st]);
		}
	}

	long ReadyTimeOutEmpty = 0;
	for (insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		if (GetEmptyError(insertOrd + 1) == 1)
		{
			ReadyTimeOutEmpty++;
		}
	}
	if (ReadyTimeOutEmpty > 0)
	{
		if (ReadyTimeOutEmpty == GetMaxInsertOrder())
		{
			return Err;
		}
	}
	GetTime = _time_get();
	Err = WaitGantry(FRONT_GANTRY, TIME5000MS);
	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowRunLog() == true || Elapsed > 0)
	{
		TRACE(_T("[PWR] Recognition WaitGantry Elasped,%d"), Elapsed);
	}
	//if (bManualRecog == true)
	//{
	//	Err = ManualRecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), RatioZ, TIME5000MS);
	//	if (Err != NO_ERR)
	//	{
	//		SendAlarm(Err, _T("ManualRecognitionMoveAllZUp Err"));
	//		TRACE(_T("[PWR] ManualRecognitionMoveAllZUp Err,%d\n"), Err);
	//		return Err;
	//	}
	//}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] CRecognitionDivide Run ProdRunMode:%d\n"), GetProdRunMode());
	}

	for (long TblHead = 0; TblHead < TBL_HEAD3; TblHead++)
	{
		long Cam1Head = TblHead;
		long Cam2Head = TblHead + TBL_HEAD3 ;

		if (HeadUseVA[Cam1Head] == true && HeadUseVA[Cam2Head] == true)
		{
			if (HeadPackageName[Cam1Head].CompareNoCase(HeadPackageName[Cam2Head]) == 0)
			{
				HeadCatchIndex[Cam1Head] = HeadCatchIndex[Cam2Head] = CatchIndex;
				CatchIndex++;

				HeadCatchUnit[Cam1Head] = 0;
				HeadCatchUnit[Cam2Head] = 1;
			}
			else
			{
				HeadCatchIndex[Cam1Head] = CatchIndex;
				CatchIndex++;
				HeadCatchIndex[Cam2Head] = CatchIndex;
				CatchIndex++;

				HeadCatchUnit[Cam1Head] = 0;
				HeadCatchUnit[Cam2Head] = 0;
			}
		}
		else if (HeadUseVA[Cam1Head] == true)
		{
			HeadCatchIndex[Cam1Head] = CatchIndex;
			CatchIndex++;
			HeadCatchUnit[Cam1Head] = 0;

		}
		else if (HeadUseVA[Cam2Head] == true)
		{
			HeadCatchIndex[Cam2Head] = CatchIndex;
			CatchIndex++;
			HeadCatchUnit[Cam2Head] = 0;
		}
	}

	MaxCatchOrd = CatchIndex;

	TRACE(_T("[PWR] CRecognitionDivide HeadCatchIndex %d %d %d %d %d %d"), HeadCatchIndex[0], HeadCatchIndex[1], HeadCatchIndex[2], HeadCatchIndex[3], HeadCatchIndex[4], HeadCatchIndex[5]);
	TRACE(_T("[PWR] CRecognitionDivide HeadCatchUnit %d %d %d %d %d %d"), HeadCatchUnit[0], HeadCatchUnit[1], HeadCatchUnit[2], HeadCatchUnit[3], HeadCatchUnit[4], HeadCatchUnit[5]);

	for (insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		if (JobHeadNo[insertOrd] == 0)
		{
			continue;
		}

		CatchIndex = HeadCatchIndex[JobHeadNo[insertOrd] - 1];
		CatchUnit1st = HeadCatchUnit[JobHeadNo[insertOrd] - 1];
		CatchOrder[CatchIndex] = GetCameraHeadFromHeadNo(JobHeadNo[insertOrd]);
		CatchUseVA[CatchIndex][CatchUnit1st] = USE_VA;
		RecogTable = GetRecognitionTableFromHeadNo(JobHeadNo[insertOrd]);
		CatchCamNo[CatchIndex][CatchUnit1st] = GetCameraNoByHead(RecogTable, JobHeadNo[insertOrd]);
		CatchVisChk[CatchIndex][CatchUnit1st] = GetCameraChkPosByHead(JobHeadNo[insertOrd]);
		CatchVisDB[CatchIndex][CatchUnit1st] = GetFeederNoFromHeadNo(JobHeadNo[insertOrd]);
		Led[CatchIndex][CatchUnit1st] = GetLed(CatchVisDB[CatchIndex][CatchUnit1st]);
		CatchCompRatio[CatchIndex][CatchUnit1st] = GetRatioByFdNo(CatchVisDB[CatchIndex][CatchUnit1st]);
		CatchHeadNo[CatchIndex][CatchUnit1st] = JobHeadNo[insertOrd];
		CatchAngleVA[CatchIndex][CatchUnit1st] = GetVAAngleFromFdNo(CatchVisDB[CatchIndex][CatchUnit1st]);
		CatchComponentHeight[CatchIndex][CatchUnit1st] = GetComponentHeight(CatchVisDB[CatchIndex][CatchUnit1st]);
		CatchComponentLeadHeight[CatchIndex][CatchUnit1st] = GetComponentLeadHeight(CatchVisDB[CatchIndex][CatchUnit1st]);
		CatchComponentVARecognitionOffsetHeight[CatchIndex][CatchUnit1st] = GetVAOffsetHeight(CatchVisDB[CatchIndex][CatchUnit1st]);
		NozzleNo = GetGlobalNozzleNo(JobHeadNo[insertOrd]);
		CatchNozzle[CatchIndex][CatchUnit1st] = GetGlobalNozzleInformation(NozzleNo);
		CatchDelay[CatchIndex][CatchUnit1st] = GetCatchDelay(CatchVisDB[CatchIndex][CatchUnit1st]);
		//CatchFormingVA[CatchIndex][CatchUnit1st] = GetForming(CatchVisDB[CatchIndex][CatchUnit1st]).Use;
		CatchLaserControl[CatchIndex][CatchUnit1st] = GetLaserControl(CatchVisDB[CatchIndex][CatchUnit1st]);

		if (CatchIndexFirst == true)
		{
			CatchIndexFirst = false;
			CatchIndexManual = CatchIndex;
			CatchUnitManual = CatchUnit1st;
		}
	}

	if (bManualRecog == true)
	{
		CatchIndex = CatchIndexManual;
		CatchUnit1st = CatchUnitManual;

		if (GetDivideInspect(CatchVisDB[CatchIndex][CatchUnit1st]).Use == true)
		{			
			Err = MoveZAllUpLimitWithOutOneHead(Gantry, RatioZ, GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[0][0]));
			if (Err != NO_ERR)
			{
				SendAlarm(Err, _T("MoveZAllUpLimitWithOutOneHead Err"));
				TRACE(_T("[PWR] MoveZAllUpLimitWithOutOneHead Err,%d\n"), Err);
				return Err;
			}

			Target.Axis[0] = GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[0][0]);
			Target.Command[0] = GetPickupZStandBy() - (CatchComponentHeight[CatchMoveZOrd][CatchUnit] + CatchComponentLeadHeight[CatchMoveZOrd][CatchUnit]) - CatchNozzle[CatchMoveZOrd][CatchUnit].TipHeight + CatchNozzle[CatchMoveZOrd][CatchUnit].PusherHeight;

			Limit limit = GetLimit(GetAxisIndexFromAliasName(Target.Axis[0]));
			if (limit.minus > Target.Command[0])
			{
				TRACE(_T("[PWR] Divide MoveOneZDn position(%.3f) is under minus limit(%.3f)\n"), Target.Command[0], limit.minus);
				Target.Command[0] = limit.minus + 1.0;
			}

			Target.Ratio[0] = CatchCompRatio[CatchOrd][CatchUnit].z;
			Target.Inpos[0] = InposZDn;
			Target.InposMs[0] = MsZDn;
			Target.TimeOut[0] = TimeOut;

			TRACE(_T("[PWR] Divide MoveOneZDn %s %.3f\n"), Target.Axis[0], Target.Command[0]);

			Err = MoveOneZDn(Target.Axis[0], Target.Ratio[0], Target.TimeOut[0], Target.Command[0], Target.Inpos[0], Target.InposMs[0], true);
			if (Err != NO_ERR)
			{
				SendAlarm(Err, _T("Divide MoveOneZDn Err"));
				TRACE(_T("[PWR] Divide MoveOneZDn Err,%d\n"), Err);
				return Err;
			}

		}
		else
		{
			Err = ManualRecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), RatioZ, TIME5000MS);
			if (Err != NO_ERR)
			{
				SendAlarm(Err, _T("ManualRecognitionMoveAllZUp Err"));
				TRACE(_T("[PWR] ManualRecognitionMoveAllZUp Err,%d\n"), Err);
				return Err;
			}
		}

		WaitAllZIdle(Gantry, TIME5000MS);
	}

	//if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] CRecognitionDivide MaxCatchOrd:%d\n"), MaxCatchOrd);
		for (CatchOrd = 0; CatchOrd < MaxCatchOrd; ++CatchOrd)
		{
			for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
			{
				if (CatchUseVA[CatchOrd][CatchUnit] != USE_VA) continue;
				TRACE(_T("[PWR] CRecognitionDivide CatchOrd(%d) CatchUnit(%d) UseVA:%d CamNo:%d HeadNo:%d(%s,%s)\n"), 
					CatchOrd, CatchUnit,
					CatchUseVA[CatchOrd][CatchUnit], CatchCamNo[CatchOrd][CatchUnit], CatchHeadNo[CatchOrd][CatchUnit],
					GetRAxisFromHeadNo(Gantry, CatchHeadNo[CatchOrd][CatchUnit]),
					GetZAxisFromHeadNo(Gantry, CatchHeadNo[CatchOrd][CatchUnit])
				);
			}
		}
	}

	for (CatchOrd = 0; CatchOrd < MaxCatchOrd; ++CatchOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] CRecognitionDivide GetGlobalStatusError-1(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] CRecognitionDivide GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		for (long indx = 0; indx < MAXVAHEAD; ++indx)
		{
			UseVA[indx] = NON;
			CamNo[indx] = 0;
			VisChk[indx] = 0;
			VisDB[indx] = 0;
		}
		CatchIndex = 0;
		RecogTable = FRONT_STAGE;
		for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
		{
			if (CatchUseVA[CatchOrd][CatchUnit] != USE_VA) continue;
			UseVA[CatchIndex] = CatchUseVA[CatchOrd][CatchUnit];
			CamNo[CatchIndex] = CatchCamNo[CatchOrd][CatchUnit];
			VisChk[CatchIndex] = CatchVisChk[CatchOrd][CatchUnit];
			VisDB[CatchIndex] = CatchVisDB[CatchOrd][CatchUnit];
			AngleVA[CatchIndex] = CatchAngleVA[CatchOrd][CatchUnit];
			Forming[CatchIndex] = CatchFormingVA[CatchOrd][CatchUnit];
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] CatchOrd(%d) CatchUnit(%d) index(%d) Use(%d) Cam(%d) VisChk(%d) DB(%d) Angle(%.3f) Forming(%d)\n"),
					CatchOrd, CatchUnit, CatchIndex, UseVA[CatchIndex], CamNo[CatchIndex], VisChk[CatchIndex], VisDB[CatchIndex], AngleVA[CatchIndex], Forming[CatchIndex]);
			}
			CatchIndex++;
		}
		if (GetProdRunMode() == RUN_REAL)
		{
			GetTime = _time_get();
			Ret = gPrepareCommand(Gantry, CamNo, CatchOrd + 1, VisChk, UseVA, VisDB, AngleVA, Forming);
			Elapsed = _time_elapsed(GetTime);
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				TRACE(_T("[PWR] gPrepareCommand Ret(%d) Elapsed,%d\n"), Ret, Elapsed);
			}
		}
		else // Simulation
		{
			ThreadSleep(TIME30MS);
		}
		//TargetHeadNo = GetCameraHeadFromHeadNo(JobHeadNo[insertOrd]);
		if (CatchCamNo[CatchOrd][0] < CAM5)
			RecogTable = FRONT_STAGE;
		else
			RecogTable = REAR_STAGE;
		
		RecogTable = FRONT_STAGE;

		if (CatchOrd == 0)
		{
			OldRecogTable = RecogTable;
			bSaftyZUp = false;
		}
		else
		{
			if (RecogTable != OldRecogTable) // HarkDo Check 올렸으니깐 다시 내려야 한다.
			{
				TRACE(_T("[PWR] RecogTable New,%d Old,%d Safty Z Up\n"), RecogTable, OldRecogTable);
				Err = RecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), RatioZ, TIME5000MS);
				if (Err != NO_ERR)
				{
					SendAlarm(Err, _T("Recognition RecognitionMoveAllZUp Err"));
					TRACE(_T("[PWR] Recognition RecognitionMoveAllZUp Err,%d\n"), Err);
					return Err;
				}
				OldRecogTable = RecogTable;
				bSaftyZUp = true;
			}
			else
			{
				bSaftyZUp = false;
			}
		}
		pt = GetCameraRecognitionPosition(RecogTable, CatchOrder[CatchOrd]);
		if (GetManualCompensationUse() == 1)
		{
			Point_XYRZ ManualVisionResult;
			ManualVisionResult = GetManualVisionResult();
			pt.x += ManualVisionResult.x;
			pt.y += ManualVisionResult.y;
			SetManualVisionOffset(CatchOrd + 1, ManualVisionResult.r);
			ClearManualVisionOffset();
		}
		for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
		{
			if (CatchUseVA[CatchOrd][CatchUnit] != USE_VA) continue;
			gLedOn(CatchCamNo[CatchOrd][CatchUnit], Led[CatchOrd][CatchUnit].Top, Led[CatchOrd][CatchUnit].Mid, Led[CatchOrd][CatchUnit].Bot);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] CatchOrd:%d CatchUnit:%d Cam:%d Led(Top,Mid,Bot)(%d,%d,%d)\n"),
					CatchMoveROrd, CatchUnit,
					CatchCamNo[CatchOrd][CatchUnit], Led[CatchOrd][CatchUnit].Top, Led[CatchOrd][CatchUnit].Mid, Led[CatchOrd][CatchUnit].Bot);
			}

			if (CatchLaserControl[CatchOrd][CatchUnit] == 1)
			{
				gLaserOn(CatchCamNo[CatchOrd][CatchUnit]);
			}
			else
			{
				gLaserOff(CatchCamNo[CatchOrd][CatchUnit]);
			}
		}
		if (bFirstMoveR == true)
		{
			CatchMoveRIndex = 0;
			Target.MaxAxisCount = 0;
			for (CatchMoveROrd = 0; CatchMoveROrd < MaxCatchOrd; ++CatchMoveROrd)
			{
				for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
				{
					if (CatchUseVA[CatchMoveROrd][CatchUnit] != USE_VA) continue;
					if (gcPowerLog->IsShowRunLog() == true)
					{
						TRACE(_T("[PWR] CatchMoveROrd:%d CatchUnit:%d HeadNo:%d(%s)\n"), CatchMoveROrd, CatchUnit,
							CatchHeadNo[CatchMoveROrd][CatchUnit], GetRAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveROrd][CatchUnit]));
					}
					Target.Axis[CatchMoveRIndex] = GetRAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveROrd][CatchUnit]);
					Target.Command[CatchMoveRIndex] = CatchAngleVA[CatchMoveROrd][CatchUnit] + GetManualVisionOffset(CatchMoveROrd + 1);
					Target.Ratio[CatchMoveRIndex] = CatchCompRatio[CatchMoveROrd][CatchUnit].r;
					Target.Inpos[CatchMoveRIndex] = InposR;
					Target.InposMs[CatchMoveRIndex] = MsR;
					Target.TimeOut[CatchMoveRIndex] = TimeOut;
					if (gcPowerLog->IsShowRunLog() == true)
					{
						TRACE(_T("[PWR] RMoveIndex(%d)(%s) Command:%.3f\n"),
							CatchMoveRIndex, Target.Axis[CatchMoveRIndex], Target.Command[CatchMoveRIndex]);
					}
					CatchMoveRIndex++;
					Target.MaxAxisCount++;
				}
			}
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] Target.MaxAxisCount,%d\n"), Target.MaxAxisCount);
			}
			for (long MoveUnit = 0; MoveUnit < Target.MaxAxisCount; ++MoveUnit)
			{
				Err = MoveR(Target.Axis[MoveUnit], Target.Ratio[MoveUnit], Target.TimeOut[MoveUnit], 
					Target.Command[MoveUnit], Target.Inpos[MoveUnit], Target.InposMs[MoveUnit], false);
				if (Err != NO_ERR)
				{
					SendAlarm(Err, _T("Recognition MoveR"));
					TRACE(_T("[PWR] Recognition MoveR Err,%d\n"), Err);
					return Err;
				}
			}
		}
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			CurPt = gReadGantryPosition(Gantry);
		}
		MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
		TRACE(_T("[PWR] Recognition MinRatio XYRZ,%.1f,%.1f,%.1f MoveRatio XYRZ,%.1f,%.1f,%.1f\n"), MinRatio.xy, MinRatio.r, MinRatio.z, MoveRatio.xy, MoveRatio.r, MoveRatio.z);
		if (MinRatio.xy < MoveRatio.xy)
			MoveRatio = MinRatio;
		GetTime = _time_get();

		DIVIDE_INSPECT DivideData = GetDivideInspect(VisDB[0]);

		for (long divideCnt = 0; divideCnt < DivideData.Count; divideCnt++)
		{
			ptDivide.x = pt.x + DivideData.Offset[divideCnt].x;
			ptDivide.y = pt.y + DivideData.Offset[divideCnt].y;

			TRACE(_T("[PWR] DivideRecognition(%d) XY %.3f,%.3f -> %.3f,%.3f\n"), divideCnt, pt.x, pt.y, ptDivide.x, ptDivide.y);

			Err = MoveXY(Gantry, FHCAM, ptDivide, MoveRatio.xy, InposXY, MsXy, TimeOut);
			if (Err != NO_ERR)
			{
				SendAlarm(Err, _T("Recognition MoveXY"));
				TRACE(_T("[PWR] Recognition MoveXY Err,%d\n"), Err);
				return Err;
			}

			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				Elapsed = _time_elapsed(GetTime);
				TRACE(_T("[PWR] Recognition Dist (%.3f, %.3f) XY Elapsed,%d\n"), pt.x - CurPt.x, pt.y - CurPt.y, Elapsed);
			}

			if (bFirstMoveR == true)
			{
				GetTime = _time_get();
				for (long MoveUnit = 0; MoveUnit < Target.MaxAxisCount; ++MoveUnit)
				{
					Err = WaitR(Target.Axis[MoveUnit], Target.Command[MoveUnit], Target.Inpos[MoveUnit], Target.TimeOut[MoveUnit]);
					if (Err != NO_ERR)
					{
						SendAlarm(Err, _T("Recognition WaitR"));
						TRACE(_T("[PWR] Recognition WaitR(%s) Err,%d\n"), Target.Axis[CatchUnit], Err);
						return Err;
					}
				}
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] Recognition R Elapsed,%d\n"), Elapsed);
				}
				bFirstMoveR = false;
			}

			if (bFirstMoveZ == true || bSaftyZUp == true)
			{
				CatchMoveZIndex = 0;
				Target.MaxAxisCount = 0;
				//LaserOn(2);
				for (CatchMoveZOrd = 0; CatchMoveZOrd < MaxCatchOrd; ++CatchMoveZOrd)
				{
					for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
					{
						if (CatchUseVA[CatchMoveZOrd][CatchUnit] != USE_VA) continue;
						if (gcPowerLog->IsShowRunLog() == true)
						{
							TRACE(_T("[PWR] CatchMoveZOrd:%d CatchUnit:%d HeadNo:%d(%s)\n"),
								CatchMoveZOrd, CatchUnit,
								CatchHeadNo[CatchMoveZOrd][CatchUnit],
								GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveZOrd][CatchUnit]));
						}
						Target.Axis[CatchMoveZIndex] = GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveZOrd][CatchUnit]);
						Target.Command[CatchMoveZIndex] = GetInsertByZ(FRONT_GANTRY) - (CatchComponentHeight[CatchMoveZOrd][CatchUnit] + CatchComponentLeadHeight[CatchMoveZOrd][CatchUnit]) - CatchNozzle[CatchMoveZOrd][CatchUnit].TipHeight + CatchNozzle[CatchMoveZOrd][CatchUnit].PusherHeight + CatchComponentVARecognitionOffsetHeight[CatchMoveZOrd][CatchUnit];
						if (GetProdRunMode() == RUN_DRY)
						{
							Target.Command[CatchMoveZIndex] -= GetDryRunZHeightOffset();
						}
						Target.Ratio[CatchMoveZIndex] = CatchCompRatio[CatchOrd][CatchUnit].z;
						Target.Inpos[CatchMoveZIndex] = InposZDn;
						Target.InposMs[CatchMoveZIndex] = MsZDn;
						Target.TimeOut[CatchMoveZIndex] = TimeOut;
						if (gcPowerLog->IsShowRunLog() == true)
						{
							TRACE(_T("[PWR] ZMoveIndex(%d)(%s) ComponeneHeight(%.3f) LeadHeight(%.3f) Nozzle No,Type(%d,%d) TipHeight:%.3f PusherHeight:%.3f RecognitionHeightOffset:%.3f Target:%.3f\n"),
								CatchMoveZIndex,
								Target.Axis[CatchMoveZIndex],
								CatchComponentHeight[CatchMoveZOrd][CatchUnit],
								CatchComponentLeadHeight[CatchMoveZOrd][CatchUnit],
								CatchNozzle[CatchMoveZOrd][CatchUnit].No,
								CatchNozzle[CatchMoveZOrd][CatchUnit].Type,
								CatchNozzle[CatchMoveZOrd][CatchUnit].TipHeight,
								CatchNozzle[CatchMoveZOrd][CatchUnit].PusherHeight,
								CatchComponentVARecognitionOffsetHeight[CatchMoveZOrd][CatchUnit],
								Target.Command[CatchUnit]);
						}
						CatchMoveZIndex++;
						Target.MaxAxisCount++;
					}
				}
				if (gcPowerLog->IsShowRunLog() == true)
				{
					TRACE(_T("[PWR] Target.MaxAxisCount:%d\n"), Target.MaxAxisCount);
				}
				GetTime = _time_get();
				if (Target.MaxAxisCount < MIN_LINEAR_INTP_AXIS)
				{
					CatchUnit = 0;
					Err = MoveOneZDn(Target.Axis[CatchUnit], Target.Ratio[CatchUnit], Target.TimeOut[CatchUnit], Target.Command[CatchUnit], Target.Inpos[CatchUnit], Target.InposMs[CatchUnit], true);
				}
				else
				{
					Err = MoveSomeZDn(Gantry, Target);
				}
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] Recognition One or Some Z Dn Elapsed,%d\n"), Elapsed);
				}
				if (Err != NO_ERR)
				{
					SendAlarm(Err, _T("Recognition MoveOne or MoveSome ZDn"));
					TRACE(_T("[PWR] Recognition MoveOne or MoveSome ZDn Err,%d\n"), Err);
					return Err;
				}
				bFirstMoveZ = false;
			}

			MaxCatchDelay = 0;
			for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
			{
				if (CatchUseVA[CatchOrd][CatchUnit] != USE_VA) continue;
				if (MaxCatchDelay < CatchDelay[CatchOrd][CatchUnit])
				{
					MaxCatchDelay = CatchDelay[CatchOrd][CatchUnit];
				}
			}
			if (MaxCatchDelay > 0)
			{
				ThreadSleep(MaxCatchDelay);
				TRACE(_T("[PWR] Delay(%d)[ms] Before Catch image by Camera\n"), MaxCatchDelay);
			}

			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gImageCatch(Gantry, CamNo, CatchOrd + 1, VisChk, UseVA, 1 + divideCnt);
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] gImageCatch CatchOrd(%d) Ret(%d) Elapsed,%d\n"), CatchOrd, Ret, Elapsed);
				}
			}
			else // Simulation
			{
				ThreadSleep(TIME30MS);
			}
		}
	}

	for (CatchOrd = 0; CatchOrd < MaxCatchOrd; ++CatchOrd)
	{
		if (GetProdRunMode() == RUN_REAL)
		{
			GetTime = _time_get();
			Ret = gStartProcess(CatchCamNo[CatchOrd][0], CatchOrd + 1);
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				Elapsed = _time_elapsed(GetTime);
				TRACE(_T("[PWR] gStartProcess Ret(%d) Elapsed,%d\n"), Ret, Elapsed);
			}
		}
		else // Simulation
		{
			ThreadSleep(TIME30MS);
		}
		for (CatchUnit = 0; CatchUnit < MAXCATCH_SIMUL; ++CatchUnit)
		{
			if (CatchUseVA[CatchOrd][CatchUnit] != USE_VA) continue;
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gGetRecognitionResult(CatchCamNo[CatchOrd][CatchUnit], CatchVisChk[CatchOrd][CatchUnit], CatchOrd + 1, &err, &res, FrameNo, &ResPartSize);
				insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[CatchOrd][CatchUnit]);
				FeederNo = GetFdNoFromInsertOrder(insertOrd);
				if (insertOrd > 0)
				{
					if (GetSkipVision() == 1) // Simulation
					{
						res.exe = 1;
						res.x = res.y = res.r = 0.000;
						err = NO_ERR;
						TRACE(_T("[PWR] Gantry(%d) Set Run Vision Simulation Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f PitchXY:%.3f,%.3f\n"), Gantry, insertOrd, res.exe, res.x, res.y, res.r, ResPartSize.x, ResPartSize.y);
						gSetPartRecognitionResult(CatchCamNo[CatchOrd][CatchUnit], CatchVisChk[CatchOrd][CatchUnit], res);
						gSetRunVisionResult(Gantry, insertOrd - 1, res);
						gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);

						SetVisionError(insertOrd, 0);
						ClearVisionErrorEmpty(FeederNo);
					}
					else
					{
						TRACE(_T("[PWR] Gantry(%d) Set Run Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f PitchXY:%.3f,%.3f\n"), Gantry, insertOrd, res.exe, res.x, res.y, res.r, ResPartSize.x, ResPartSize.y);
						gSetPartRecognitionResult(CatchCamNo[CatchOrd][CatchUnit], CatchVisChk[CatchOrd][CatchUnit], res);
						gSetRunVisionResult(Gantry, insertOrd - 1, res);
						gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);

						if (bManualRecog == false && err == 503 && GetMaxRetryCount(FeederNo) == 0) // 미흡창 통계 제외
						{
							SendToNoComponentStatistics(FeederNo, CatchNozzle[CatchOrd][CatchUnit].No);
						}
						if (res.exe != 1)
						{
							SetVisionError(insertOrd, 1);
							//if (GetMaxRetryCount(FeederNo) == 0)
							{
								gcLastPickFront->SetHeadDataEnable(GetHeadNoFromInsertOrder(insertOrd), false);
								SetVisionErrorEmpty(FeederNo, 1);

								if (bManualRecog == false)
								{
									insertNo = GetInsertNoFromInsertOrder(insertOrd);
									SendToVisionResult(Gantry, insertNo, err);
								}
							}
						}
						else
						{
							SetVisionError(insertOrd, 0);
							ClearVisionErrorEmpty(FeederNo);
						}
					}
				}
				Elapsed = _time_elapsed(GetTime);
				if (gcPowerLog->IsShowVisionLog() == true)
				{
					TRACE(_T("[PWR] gGetRecognitionResult CatchOrd(%d) VisChk(%d) Ret(%d) Err(%d) Res:%.3f %.3f %.3f SizeXY %.3f %.3f Elapsed,%d\n"),
						CatchCamNo[CatchOrd][CatchUnit], CatchVisChk[CatchOrd][CatchUnit], Ret, err, res.x, res.y, res.r,
						ResPartSize.x, ResPartSize.y, Elapsed);
				}
			}
			else
			{
				ThreadSleep(TIME30MS);
				insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[CatchOrd][CatchUnit]);
				res.exe = 1;
				res.x = 0.0;
				res.y = 0.0;
				res.r = 0.0;
				err = 0;
				ResPartSize.x = ResPartSize.y = 0.0;
				gSetRunVisionResult(Gantry, insertOrd - 1, res);
				gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
				TRACE(_T("[PWR] Set DryRun Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f PitchXY:%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r, ResPartSize.x, ResPartSize.y);
			}
		}
	}
	gLedAllOff();
	gLaserAllOff();
	return Err;
}

long CRecognitionDivide::GetMaxRetryCount(long FeederNo)
{
	long MaxRetry = 0;
	MaxRetry = gcStep->GetRetryLed(FeederNo).MaxRetry;
	return MaxRetry;
}

long CRecognitionDivide::GetCatchDelay(long FeederNo)
{
	long CatchDelay = 0;
	CatchDelay = gcStep->GetCatchDelay(FeederNo);
	return CatchDelay;
}


Ratio_XYRZ CRecognitionDivide::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio = gcStep->GetMinRatio();
	TRACE(_T("[PWR] CRecognitionDivide MinRatioXYRZ,%.1f,%.1f,%.1f"), MinRatio.xy, MinRatio.r, MinRatio.z);
	return MinRatio;
}

CString CRecognitionDivide::GetPackageName(long FeederNo)
{
	return gcReadJobFile->GetFeeder(FeederNo).PackageName;
}
DIVIDE_INSPECT CRecognitionDivide::GetDivideInspect(long FeederNo)
{
	return gcStep->GetDivideInspect(FeederNo);
}

double CRecognitionDivide::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	return MaxComponentHeight;
}

long CRecognitionDivide::GetLaserControl(long FeederNo)
{
	long LaserControl = 0;
	LaserControl = gcStep->GetLaserControl(FeederNo);
	return LaserControl;
}
