#include "pch.h"
#include "CRecognitionCamera6.h"
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

CRecognitionCamera6* gcRecognitionCamera6;
CRecognitionCamera6::CRecognitionCamera6(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxInsertOrder = 0;
	m_ProdRunMode = RUN_REAL;
	ZeroMemory(&m_FeederNo, sizeof(m_FeederNo));
	ZeroMemory(&m_InsertOrder, sizeof(m_InsertOrder));
	ZeroMemory(&m_RecognitionTable, sizeof(m_RecognitionTable));
	ClearManualVisionOffset();
}

CRecognitionCamera6::~CRecognitionCamera6()
{
}

void CRecognitionCamera6::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CRecognitionCamera6::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CRecognitionCamera6::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CRecognitionCamera6::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CRecognitionCamera6::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CRecognitionCamera6::GetFeederNoFromHeadNo(long HeadNo)
{
	long FeederNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		FeederNo = m_FeederNo[HeadNo - 1];
	}
	return FeederNo;
}

long CRecognitionCamera6::GetInsertOrderFromHeadNo(long HeadNo)
{
	long InsertOder = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		InsertOder = m_InsertOrder[HeadNo - 1];
	}
	return InsertOder;
}

long CRecognitionCamera6::GetRecognitionTableFromHeadNo(long HeadNo)
{
	long RecogTable = FRONT_GANTRY;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		RecogTable = m_RecognitionTable[HeadNo - 1];
	}
	return RecogTable;
}

Ratio_XYRZ CRecognitionCamera6::GetRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

double CRecognitionCamera6::GetVAAngleFromFdNo(long insertOrd)
{
	double RecognitionAngle = 0.0;
	RecognitionAngle = gcStep->GetVAAngleFromFdNo(insertOrd);
	return RecognitionAngle;
}

double CRecognitionCamera6::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionCamera6::GetComponentLeadHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionCamera6::GetVAOffsetHeight(long FdNo)
{
	double VARecognitionOffsetHeight = 0.0;
	VARecognitionOffsetHeight = gcStep->GetVAOffsetHeight(FdNo);
	return VARecognitionOffsetHeight;
}

MODULE_LED CRecognitionCamera6::GetLed(long FdNo)
{
	MODULE_LED Led;
	Led = gcStep->GetLed(FdNo);
	return Led;
}

long CRecognitionCamera6::GetManualCompensationUse()
{
	long ManualCompenUse = 0;
	ManualCompenUse = gcStep->GetManualCompensationUse();
	return ManualCompenUse;
}

Point_XYRZ CRecognitionCamera6::GetManualVisionResult()
{
	Point_XYRZ ManualVisionResult;
	ManualVisionResult = gcStep->GetManualVisionResult();
	return ManualVisionResult;
}

Ratio_XYRZ CRecognitionCamera6::GetComponentRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ZeroMemory(&ratio, sizeof(ratio));
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

long CRecognitionCamera6::ClearManualVisionOffset()
{
	ZeroMemory(&m_VAAngleOffset, sizeof(m_VAAngleOffset));
	return NO_ERR;
}

long CRecognitionCamera6::SetManualVisionOffset(long insertOrd, double OffsetAngle)
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

double CRecognitionCamera6::GetManualVisionOffset(long insertOrd)
{
	double OffsetAngle = 0.0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		OffsetAngle = m_VAAngleOffset[insertOrd - 1];
	}
	return OffsetAngle;
}

long CRecognitionCamera6::SetFeederNoFromHeadNo(long HeadNo, long FeederNo)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_FeederNo[HeadNo - 1] = FeederNo;
	}
	return NO_ERR;
}

long CRecognitionCamera6::SetInsertOrderFromHeadNo(long HeadNo, long InsertOrder)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_InsertOrder[HeadNo - 1] = InsertOrder;
	}
	return NO_ERR;
}

long CRecognitionCamera6::SetRecognitionTableFromHeadNo(long HeadNo, long RecogTable)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_RecognitionTable[HeadNo - 1] = RecogTable;
	}
	return NO_ERR;
}

long CRecognitionCamera6::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CRecognitionCamera6::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

long CRecognitionCamera6::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
	return VisionErrorEmpty;
}

long CRecognitionCamera6::ClearVisionErrorEmpty(long FeederNo)
{
	long Err = NO_ERR;
	gcStep->ClearVisionErrorEmpty(FeederNo);
	return Err;
}

long CRecognitionCamera6::SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty)
{
	long Err = NO_ERR;
	gcStep->SetVisionErrorEmpty(FeederNo, VisionErrorEmpty);
	return Err;
}

long CRecognitionCamera6::SetVisionError(long InsertOrd, long VisionError)
{
	long Err = NO_ERR;
	gcStep->SetVisionError(InsertOrd, VisionError);
	return Err;
}

long CRecognitionCamera6::GetEmptyError(long InsertOrd)
{
	long EmptyError = 0;
	EmptyError = gcStep->GetEmptyError(InsertOrd);
	return EmptyError;
}

bool CRecognitionCamera6::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	bSendEmpty = gcStep->GetSendEmpty(FeederNo);
	return bSendEmpty;
}

long CRecognitionCamera6::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	long Err = NO_ERR;
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	return Err;
}

long CRecognitionCamera6::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	return ReadyTimeOutEmpty;
}

double CRecognitionCamera6::GetPickupZStandBy()
{
	double PickupZStandBy = 0.0;
	PickupZStandBy = gcStep->GetPickupZStandBy();
	return PickupZStandBy;
}

double CRecognitionCamera6::GetHighSpeedZOffset()
{
	double HighSpeedZOffset = 0.0;
	HighSpeedZOffset = gcStep->GetHighSpeedZOffset();
	return HighSpeedZOffset;
}

double CRecognitionCamera6::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	return MaxComponentHeight;
}

long CRecognitionCamera6::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CRecognitionCamera6::ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionCamera6::RecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionCamera6::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, FHCAM, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CRecognitionCamera6::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionCamera6::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CRecognitionCamera6::MoveOneZDn(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionCamera6::MoveSomeZDn(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = StartSomeZAxisWaitDelayedPosSet(Gantry, Target);
	return Err;
}

long CRecognitionCamera6::MoveZUpBeforeInsert(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionCamera6::WaitZUpBeforeInsert(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneInPos(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CRecognitionCamera6::Run(bool bManualRecog)
{
	TRACE(_T("[PWR] CRecognitionCamera6 Run\n"));
	long Gantry = m_Gantry, insertNo = 0, RecogTable = FRONT_STAGE, OldRecogTable = FRONT_STAGE;
	Point_XY pt, CurPt;
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
	long CatchOrder[MAXCATCH_CAMERA6], CatchIndex = 0, CatchUnit = 0;
	long CatchUseVA[MAXCATCH_CAMERA6];
	long CatchCamNo[MAXCATCH_CAMERA6];
	long CatchHeadNo[MAXCATCH_CAMERA6];
	long CatchVisChk[MAXCATCH_CAMERA6];
	long CatchVisDB[MAXCATCH_CAMERA6];
	MODULE_LED Led[MAXCATCH_CAMERA6];
	double CatchAngleVA[MAXCATCH_CAMERA6];
	Ratio_XYRZ CatchCompRatio[MAXCATCH_CAMERA6];
	double CatchComponentHeight[MAXCATCH_CAMERA6];
	double CatchComponentLeadHeight[MAXCATCH_CAMERA6];
	double CatchVARecognitionOffsetHeight[MAXCATCH_CAMERA6];
	NOZZLE CatchNozzle[MAXCATCH_CAMERA6];
	long CatchLaserControl[MAXCATCH_CAMERA6];
	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Err = NO_ERR, insertOrd = 0, LedOrd = 0, CatchOrd = 0, MaxCatchOrd = 0, CatchMoveZOrd = 0, CatchMoveZIndex = 0, CatchMoveROrd = 0, CatchMoveRIndex = 0;
	double RatioZ = 1.0, Pitch = 1.0, ComponentHeight[MAXUSEDHEADNO], JobAngle[MAXUSEDHEADNO], ComponentLeadHeight[MAXUSEDHEADNO];
	double AlignCheckingZ = GetInsertByZ(Gantry);
	long Cam[MAXUSEDHEADNO], ChkPos[MAXUSEDHEADNO], /*TargetHeadNo, */JobHeadNo[MAXUSEDHEADNO], JobFdNo[MAXUSEDHEADNO], JobDBNo[MAXUSEDHEADNO], RecognitionTable[MAXUSEDHEADNO];
	long CatchUnit1st = 0, CatchUnit2nd = 0, CatchUnit3rd = 0;
	long CatchUnit1stCatchOrd = 0, CatchUnit2ndCatchOrd = 0, CatchUnit3rdCatchOrd = 0;
	bool bFirstMoveR = true, bFirstMoveZ = true, bSaftyZUp = false;
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
	ZeroMemory(&CatchNozzle, sizeof(CatchNozzle));
	ZeroMemory(&CatchLaserControl, sizeof(CatchLaserControl));
	ZeroMemory(&Forming, sizeof(Forming));
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
	gClearRunVisionResult(FRONT_STAGE);
	gClearRunVisionResult(REAR_STAGE);

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
		JobFdNo[insertOrd] = GetFdNoFromInsertOrder(insertOrd + 1);
		JobDBNo[insertOrd] = JobFdNo[insertOrd];
		SetInsertOrderFromHeadNo(JobHeadNo[insertOrd], insertOrd + 1);
		SetFeederNoFromHeadNo(JobHeadNo[insertOrd], JobDBNo[insertOrd]);
		ChkPos[insertOrd] = GetCamera6ChkPosByHead(JobHeadNo[insertOrd]);
		CompRatio[insertOrd] = GetRatioByFdNo(JobFdNo[insertOrd]);
		JobAngle[insertOrd] = GetVAAngleFromFdNo(JobFdNo[insertOrd]);
		ComponentHeight[insertOrd] = GetComponentHeight(JobFdNo[insertOrd]);
		ComponentLeadHeight[insertOrd] = GetComponentLeadHeight(JobFdNo[insertOrd]);
		insertNo = GetInsertNoFromInsertOrder(insertOrd + 1);
		RecognitionTable[insertOrd] = GetRecognitionTable(insertNo);
		SetRecognitionTableFromHeadNo(JobHeadNo[insertOrd], RecognitionTable[insertOrd]);
		Cam[insertOrd] = GetCamera6NoByHead(RecognitionTable[insertOrd], JobHeadNo[insertOrd]) + 1;
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
			if (JobHeadNo[sort2nd] < JobHeadNo[min])
			{
				min = sort2nd;
				SWAP(JobHeadNo[sort1st], JobHeadNo[min], buf);
			}
		}
	}
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
	if (bManualRecog == true)
	{
		Err = ManualRecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), RatioZ, TIME5000MS);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("ManualRecognitionMoveAllZUp Err"));
			TRACE(_T("[PWR] ManualRecognitionMoveAllZUp Err,%d\n"), Err);
			return Err;
		}
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] Recognition Run ProdRunMode:%d MaxInsertOrd:%d\n"), GetProdRunMode(), GetMaxInsertOrder());
	}
	for (insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		CatchOrder[CatchIndex] = GetCamera6HeadFromHeadNo(JobHeadNo[insertOrd]);
		CatchUseVA[CatchIndex] = USE_VA;
		RecogTable = GetRecognitionTableFromHeadNo(JobHeadNo[insertOrd]);
		CatchCamNo[CatchIndex] = GetCamera6NoByHead(RecogTable, JobHeadNo[insertOrd]);
		CatchVisChk[CatchIndex] = GetCamera6ChkPosByHead(JobHeadNo[insertOrd]);
		CatchVisDB[CatchIndex] = GetFeederNoFromHeadNo(JobHeadNo[insertOrd]);
		Led[CatchIndex] = GetLed(CatchVisDB[CatchIndex]);
		CatchCompRatio[CatchIndex] = GetRatioByFdNo(CatchVisDB[CatchIndex]);
		CatchHeadNo[CatchIndex] = JobHeadNo[insertOrd];
		AngleVA[CatchIndex] = CatchAngleVA[CatchIndex] = GetVAAngleFromFdNo(CatchVisDB[CatchIndex]);
		CatchComponentHeight[CatchIndex] = GetComponentHeight(CatchVisDB[CatchIndex]);
		CatchComponentLeadHeight[CatchIndex] = GetComponentLeadHeight(CatchVisDB[CatchIndex]);
		CatchVARecognitionOffsetHeight[CatchIndex] = GetVAOffsetHeight(CatchVisDB[CatchIndex]);
		NozzleNo = GetGlobalNozzleNo(JobHeadNo[insertOrd]);
		CatchNozzle[CatchIndex] = GetGlobalNozzleInformation(NozzleNo);
		CatchLaserControl[CatchIndex] = GetLaserControl(CatchVisDB[CatchIndex]);
		CatchIndex++;
		MaxCatchOrd++;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] CRecognition MaxCatchOrd:%d\n"), MaxCatchOrd);
		for (CatchUnit = 0; CatchUnit < MaxCatchOrd; ++CatchUnit)
		{
			if (CatchUseVA[CatchUnit] != USE_VA) continue;
			TRACE(_T("[PWR] CRecognition CatchOrd(%d) CatchUnit(%d) UseVA:%d CamNo:%d HeadNo:%d(%s,%s)\n"), 
				CatchOrd, CatchUnit,
				CatchUseVA[CatchUnit], CatchCamNo[CatchUnit], CatchHeadNo[CatchUnit],
				GetRAxisFromHeadNo(Gantry, CatchHeadNo[CatchUnit]),
				GetZAxisFromHeadNo(Gantry, CatchHeadNo[CatchUnit])
			);
		}
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
	for (CatchUnit = 0; CatchUnit < MAXCATCH_CAMERA6; ++CatchUnit)
	{
		if (CatchUseVA[CatchUnit] != USE_VA) continue;
		UseVA[CatchIndex] = CatchUseVA[CatchUnit];
		CamNo[CatchIndex] = CatchCamNo[CatchUnit];
		VisChk[CatchIndex] = CatchVisChk[CatchUnit];
		VisDB[CatchIndex] = CatchVisDB[CatchUnit];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] CatchUnit(%d) index(%d) Use(%d) Cam(%d) VisChk(%d) DB(%d)\n"), CatchUnit, CatchIndex, UseVA[CatchIndex], CamNo[CatchIndex], VisChk[CatchIndex], VisDB[CatchIndex]);
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
		if (Ret == -2)
		{
			TRACE(_T("[PWR] Vision Prepare Err id:%d Ret:%d\n"), CatchOrd + 1, Ret);
			Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Prepare Error"));
			return Ret;
		}
	}
	else // Simulation
	{
		ThreadSleep(TIME30MS);
	}
	//TargetHeadNo = GetCameraHeadFromHeadNo(JobHeadNo[insertOrd]);
	if (CatchCamNo[CatchOrd] < CAM7)
		RecogTable = FRONT_STAGE;
	else
		RecogTable = REAR_STAGE;
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
				Err = SendAlarm(Err, _T("Recognition RecognitionMoveAllZUp Err"));
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
	}
	for (CatchUnit = 0; CatchUnit < MAXCATCH_CAMERA6; ++CatchUnit)
	{
		if (CatchUseVA[CatchUnit] != USE_VA) continue;
		gLedOn(CatchCamNo[CatchUnit], Led[CatchUnit].Top, Led[CatchUnit].Mid, Led[CatchUnit].Bot);
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] CatchUnit:%d Cam:%d Led(Top,Mid,Bot)(%d,%d,%d)\n"), CatchUnit, CatchCamNo[CatchUnit], Led[CatchUnit].Top, Led[CatchUnit].Mid, Led[CatchUnit].Bot);
		}
		if (Led[CatchUnit].Bot != 0)
		{
			gLaserOn(CatchCamNo[CatchUnit]);
		}
		else if (CatchLaserControl[CatchUnit] == 1)
		{
			gLaserOn(CatchCamNo[CatchUnit]);
		}
		else
		{
			gLaserOff(CatchCamNo[CatchUnit]);
		}
	}
	if (bFirstMoveR == true)
	{
		CatchMoveRIndex = 0;
		Target.MaxAxisCount = 0;
		for (CatchMoveROrd = 0; CatchMoveROrd < MaxCatchOrd; ++CatchMoveROrd)
		{
			if (CatchUseVA[CatchMoveROrd] != USE_VA) continue;
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] CatchMoveROrd:%d CatchUnit:%d HeadNo:%d(%s)\n"), CatchMoveROrd, CatchUnit,
					CatchHeadNo[CatchMoveROrd], GetRAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveROrd]));
			}
			Target.Axis[CatchMoveRIndex] = GetRAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveROrd]);
			Target.Command[CatchMoveRIndex] = CatchAngleVA[CatchMoveROrd] + GetManualVisionOffset(CatchMoveROrd + 1);
			Target.Ratio[CatchMoveRIndex] = CatchCompRatio[CatchMoveROrd].r;
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
				Err = SendAlarm(Err, _T("Recognition MoveR"));
				TRACE(_T("[PWR] Recognition MoveR Err,%d\n"), Err);
				return Err;
			}
		}
	}
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		CurPt = gReadGantryPosition(Gantry);
	}
	MinRatio = GetMinRatio();
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
	TRACE(_T("[PWR] Recognition6 CatchOrd:%d MinRatio XYRZ,%.1f,%.1f,%.1f MoveRatio XYRZ,%.1f,%.1f,%1f\n"), CatchOrd, MinRatio.xy, MinRatio.r, MinRatio.z, MoveRatio.xy, MoveRatio.r, MoveRatio.z);
	if (MinRatio.xy < MoveRatio.xy)
	{
		MoveRatio = MinRatio;
	}
	GetTime = _time_get();
	Err = MoveXY(Gantry, FHCAM, pt, MoveRatio.xy, InposXY, MsXy, TimeOut);
	if (Err != NO_ERR)
	{
		Err = SendAlarm(Err, _T("Recognition MoveXY"));
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
				Err = SendAlarm(Err, _T("Recognition WaitR"));
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
	ClearManualVisionOffset();
	if (bFirstMoveZ == true || bSaftyZUp == true)
	{
		CatchMoveZIndex = 0;
		Target.MaxAxisCount = 0;
		for (CatchMoveZOrd = 0; CatchMoveZOrd < MaxCatchOrd; ++CatchMoveZOrd)
		{
			if (CatchUseVA[CatchMoveZOrd] != USE_VA) continue;
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] CatchMoveZOrd:%d CatchUnit:%d HeadNo:%d(%s)\n"), 
					CatchMoveZOrd, CatchUnit,
					CatchHeadNo[CatchMoveZOrd], 
					GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveZOrd]));
			}
			Target.Axis[CatchMoveZIndex] = GetZAxisFromHeadNo(FRONT_GANTRY, CatchHeadNo[CatchMoveZOrd]);
			Target.Command[CatchMoveZIndex] = GetInsertByZ(Gantry) - (CatchComponentHeight[CatchMoveZOrd] + CatchComponentLeadHeight[CatchMoveZOrd]) - CatchNozzle[CatchMoveZOrd].TipHeight + CatchNozzle[CatchMoveZOrd].PusherHeight + CatchVARecognitionOffsetHeight[CatchMoveZOrd];
			if (GetProdRunMode() == RUN_DRY)
			{
				Target.Command[CatchMoveZIndex] -= GetDryRunZHeightOffset();
			}
			Target.Ratio[CatchMoveZIndex] = CatchCompRatio[CatchOrd].z;
			Target.Inpos[CatchMoveZIndex] = InposZDn;
			Target.InposMs[CatchMoveZIndex] = MsZDn;
			Target.TimeOut[CatchMoveZIndex] = TimeOut;
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] ZMoveIndex(%d)(%s) ComponeneHeight(%.3f) LeadHeight(%.3f) Nozzle No,Type(%d,%d) TipHeight:%.3f PusherHeight:%.3f VARecogOffsetHeight:%.3f Target:%.3f\n"),
					CatchMoveZIndex,
					Target.Axis[CatchMoveZIndex],
					CatchComponentHeight[CatchMoveZOrd],
					CatchComponentLeadHeight[CatchMoveZOrd],
					CatchNozzle[CatchMoveZOrd].No,
					CatchNozzle[CatchMoveZOrd].Type,
					CatchNozzle[CatchMoveZOrd].TipHeight,
					CatchNozzle[CatchMoveZOrd].PusherHeight,
					CatchVARecognitionOffsetHeight[CatchMoveZOrd],
					Target.Command[CatchUnit]);
			}
			CatchMoveZIndex++;
			Target.MaxAxisCount++;
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
			Err = SendAlarm(Err, _T("Recognition MoveOne or MoveSome ZDn"));
			TRACE(_T("[PWR] Recognition MoveOne or MoveSome ZDn Err,%d\n"), Err);
			return Err;
		}
		bFirstMoveZ = false;
	}
	if (GetProdRunMode() == RUN_REAL)
	{
		GetTime = _time_get();
		Ret = gImageCatch(Gantry, CamNo, CatchOrd + 1, VisChk, UseVA, 1);
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TRACE(_T("[PWR] gImageCatch CatchOrd(%d) Ret(%d) Elapsed,%d\n"), CatchOrd, Ret, Elapsed);
		}
		if (Ret == -2)
		{
			TRACE(_T("[PWR] Vision Catch Err Ret:%d\n"), Ret);
			Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Catch Error"));
			return Ret;
		}
	}
	else // Simulation
	{
		ThreadSleep(TIME30MS);
	}
	double BodyHeight = 5.0, LeadHeight = 5.0;
	TimeOut = TIME5000MS;
	long HeadNo = 0;
	double RecognitionZUp = GetStandByZ(FRONT_GANTRY), MaxComponentHeight = GetMaxComponentHeight();
	double Dist = 0.0, InsertZOffset = 0.0;
	Limit limit;
	CString strZAxis;
	GetTime = _time_get();
	if (bManualRecog == false)
	{
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
			JobFdNo[InsertOrd] = GetFdNoFromInsertOrder(InsertOrd + 1);
			BodyHeight = GetComponentHeight(JobFdNo[InsertOrd]);
			LeadHeight = GetComponentLeadHeight(JobFdNo[InsertOrd]);
			CompRatio[InsertOrd] = GetComponentRatioByFdNo(JobFdNo[InsertOrd]);
			NozzleNo = GetGlobalNozzleNo(HeadNo);
			Nozzle[InsertOrd] = GetGlobalNozzleInformation(NozzleNo);
			RecognitionZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle[InsertOrd].TipHeight + Nozzle[InsertOrd].PusherHeight - BodyHeight - LeadHeight;
			strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			if (RecognitionZUp > SAFTY_ZHEIGHT)
			{
				RecognitionZUp = GetPickupZStandBy() - MaxComponentHeight - Nozzle[InsertOrd].TipHeight + Nozzle[InsertOrd].PusherHeight - BodyHeight - LeadHeight;
				TRACE(_T("[PWR] RecognitionZUp Safty,%.3f\n"), RecognitionZUp);
			}
			if (limit.minus > RecognitionZUp)
			{
				TRACE(_T("[PWR] RecognitionZUp Target position(%.3f) is under minus limit(%.3f)\n"), RecognitionZUp, limit.minus);
				RecognitionZUp = limit.minus + 1.0;
			}
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] RecognitionZUp:%.3f MaxCompoT:%.3f TipHeight:%.3f Body:%.3f Lead:%.3f\n"),
					RecognitionZUp, MaxComponentHeight, Nozzle[InsertOrd].TipHeight, BodyHeight, LeadHeight);
			}
			Err = MoveZUpBeforeInsert(strZAxis, CompRatio[InsertOrd].z, TimeOut, RecognitionZUp, InposZUp, MsZUp, false);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Insert MoveZUpBeforeInsert Err"));
				TRACE(_T("[PWR] Insert MoveZUpBeforeInsert(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
	}
	if (GetProdRunMode() == RUN_REAL)
	{
		GetTime = _time_get();
		Ret = gStartProcess(CatchCamNo[CatchOrd], CatchOrd + 1);
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			Elapsed = _time_elapsed(GetTime);
			TRACE(_T("[PWR] gStartProcess Ret(%d) Elapsed,%d\n"), Ret, Elapsed);
		}
		if (Ret == -2)
		{
			TRACE(_T("[PWR] Vision Process Err Cam:%d id:%d Ret:%d\n"), CatchCamNo[CatchOrd], CatchOrd + 1, Ret);
			Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Process Error"));
			return Ret;
		}
	}
	else // Simulation
	{
		ThreadSleep(TIME30MS);
	}
	if (bManualRecog == false)
	{
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
			JobFdNo[InsertOrd] = GetFdNoFromInsertOrder(InsertOrd + 1);
			BodyHeight = GetComponentHeight(JobFdNo[InsertOrd]);
			LeadHeight = GetComponentLeadHeight(JobFdNo[InsertOrd]);
			CompRatio[InsertOrd] = GetComponentRatioByFdNo(JobFdNo[InsertOrd]);
			NozzleNo = GetGlobalNozzleNo(HeadNo);
			Nozzle[InsertOrd] = GetGlobalNozzleInformation(NozzleNo);
			RecognitionZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle[InsertOrd].TipHeight + Nozzle[InsertOrd].PusherHeight - BodyHeight - LeadHeight;
			strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			if (RecognitionZUp > SAFTY_ZHEIGHT)
			{
				RecognitionZUp = GetPickupZStandBy() - MaxComponentHeight - Nozzle[InsertOrd].TipHeight + Nozzle[InsertOrd].PusherHeight - BodyHeight - LeadHeight;
				TRACE(_T("[PWR] RecognitionZUp Safty,%.3f\n"), RecognitionZUp);
			}
			if (limit.minus > RecognitionZUp)
			{
				TRACE(_T("[PWR] RecognitionZUp Target position(%.3f) is under minus limit(%.3f)\n"), RecognitionZUp, limit.minus);
				RecognitionZUp = limit.minus + 1.0;
			}
			Err = WaitZUpBeforeInsert(strZAxis, RecognitionZUp, InposZUp, TimeOut);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Insert WaitZUpBeforeInsert Err"));
				TRACE(_T("[PWR] Insert WaitZUpBeforeInsert(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
	}
	for (CatchOrd = 0; CatchOrd < MAXCATCH_CAMERA6; ++CatchOrd)
	{
		if (CatchUseVA[CatchOrd] != USE_VA) continue;
		if (GetProdRunMode() == RUN_REAL)
		{
			GetTime = _time_get();
			Ret = gGetRecognitionResult(CatchCamNo[CatchOrd], CatchVisChk[CatchOrd], 0 + 1, &err, &res, FrameNo, &ResPartSize);
			if (Ret == -2)
			{
				TRACE(_T("[PWR] Vision Recognition Err:%d Cam:%d loc:%d id:%d Frame:%d Ret:%d\n"), err, CatchCamNo[CatchOrd], CatchVisChk[CatchOrd], 1, FrameNo, Ret);
				Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Result Error"));
				return Ret;
			}

			insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[CatchOrd]);
			FeederNo = GetFdNoFromInsertOrder(insertOrd);
			if (insertOrd > 0)
			{
				if (GetSkipVision() == 1) // Simulation
				{
					res.exe = 1;
					res.x = res.y = res.r = 0.000;
					err = NO_ERR;
					TRACE(_T("[PWR] Set Run Vision Simulation Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r);
					gSetPartRecognitionResult(CatchCamNo[CatchOrd], CatchVisChk[CatchOrd], res);
					gSetRunVisionResult(Gantry, insertOrd - 1, res);
					gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
					SetVisionError(insertOrd, 0);
					ClearVisionErrorEmpty(FeederNo);
				}
				else
				{
					TRACE(_T("[PWR] Set Run Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r);
					gSetPartRecognitionResult(CatchCamNo[CatchOrd], CatchVisChk[CatchOrd], res);
					gSetRunVisionResult(Gantry, insertOrd - 1, res);
					gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
					if (res.exe != 1)
					{
						SetVisionError(insertOrd, 1);
						SetVisionErrorEmpty(FeederNo, 1);
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
					CatchCamNo[CatchOrd], CatchVisChk[CatchOrd], Ret, err, res.x, res.y, res.r,
					ResPartSize.x, ResPartSize.y, Elapsed);
			}
		}
		else
		{
			ThreadSleep(TIME5MS);
			insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[CatchOrd]);
			res.exe = 1;
			res.x = 0.0;
			res.y = 0.0;
			res.r = 0.0;
			err = 0;
			gSetRunVisionResult(Gantry, insertOrd - 1, res);
			gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
		}
	}
	gLedAllOff();
	gLaserAllOff();
	return Err;
}

Ratio_XYRZ CRecognitionCamera6::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio = gcStep->GetMinRatio();
	TRACE(_T("[PWR] CRecognitionCamera6 MinRatioXYRZ,%.1f,%.1f,%.1f"), MinRatio.xy, MinRatio.r, MinRatio.z);
	return MinRatio;
}

long CRecognitionCamera6::GetLaserControl(long FeederNo)
{
	long LaserControl = 0;
	LaserControl = gcStep->GetLaserControl(FeederNo);
	return LaserControl;
}