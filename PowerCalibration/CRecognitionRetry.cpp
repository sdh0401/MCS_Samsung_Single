#include "pch.h"
#include "CRecognitionRetry.h"
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

CRecognitionRetry* gcRecognitionRetry;
CRecognitionRetry::CRecognitionRetry(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxInsertOrder = 0;
	m_ProdRunMode = RUN_REAL;
	ZeroMemory(&m_FeederNo, sizeof(m_FeederNo));
	ZeroMemory(&m_InsertOrder, sizeof(m_InsertOrder));
	ZeroMemory(&m_RecognitionTable, sizeof(m_RecognitionTable));
	ClearManualVisionOffset();
}

CRecognitionRetry::~CRecognitionRetry()
{
}

void CRecognitionRetry::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CRecognitionRetry::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CRecognitionRetry::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CRecognitionRetry::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CRecognitionRetry::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CRecognitionRetry::GetFeederNoFromHeadNo(long HeadNo)
{
	long FeederNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		FeederNo = m_FeederNo[HeadNo - 1];
	}
	return FeederNo;
}

long CRecognitionRetry::GetInsertOrderFromHeadNo(long HeadNo)
{
	long InsertOder = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		InsertOder = m_InsertOrder[HeadNo - 1];
	}
	return InsertOder;
}

long CRecognitionRetry::GetRecognitionTableFromHeadNo(long HeadNo)
{
	long RecogTable = FRONT_GANTRY;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		RecogTable = m_RecognitionTable[HeadNo - 1];
	}
	return RecogTable;
}

long CRecognitionRetry::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	return ReadyTimeOutEmpty;
}

Ratio_XYRZ CRecognitionRetry::GetRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

double CRecognitionRetry::GetVAAngleFromFdNo(long insertOrd)
{
	double RecognitionAngle = 0.0;
	RecognitionAngle = gcStep->GetVAAngleFromFdNo(insertOrd);
	return RecognitionAngle;
}

double CRecognitionRetry::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionRetry::GetComponentLeadHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionRetry::GetVAOffsetHeight(long FdNo)
{
	double VARecognitionOffsetHeight = 0.0;
	VARecognitionOffsetHeight = gcStep->GetVAOffsetHeight(FdNo);
	return VARecognitionOffsetHeight;
}

MODULE_LED CRecognitionRetry::GetLed(long FdNo)
{
	MODULE_LED Led;
	Led = gcStep->GetLed(FdNo);
	return Led;
}

long CRecognitionRetry::GetManualCompensationUse()
{
	long ManualCompenUse = 0;
	ManualCompenUse = gcStep->GetManualCompensationUse();
	return ManualCompenUse;
}

Point_XYRZ CRecognitionRetry::GetManualVisionResult()
{
	Point_XYRZ ManualVisionResult;
	ManualVisionResult = gcStep->GetManualVisionResult();
	return ManualVisionResult;
}

long CRecognitionRetry::ClearManualVisionOffset()
{
	ZeroMemory(&m_VAAngleOffset, sizeof(m_VAAngleOffset));
	return NO_ERR;
}

long CRecognitionRetry::SetManualVisionOffset(long insertOrd, double OffsetAngle)
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

double CRecognitionRetry::GetManualVisionOffset(long insertOrd)
{
	double OffsetAngle = 0.0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		OffsetAngle = m_VAAngleOffset[insertOrd - 1];
	}
	return OffsetAngle;
}

long CRecognitionRetry::SetFeederNoFromHeadNo(long HeadNo, long FeederNo)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_FeederNo[HeadNo - 1] = FeederNo;
	}
	return NO_ERR;
}

long CRecognitionRetry::SetInsertOrderFromHeadNo(long HeadNo, long InsertOrder)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_InsertOrder[HeadNo - 1] = InsertOrder;
	}
	return NO_ERR;
}

long CRecognitionRetry::SetRecognitionTableFromHeadNo(long HeadNo, long RecogTable)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_RecognitionTable[HeadNo - 1] = RecogTable;
	}
	return NO_ERR;
}

long CRecognitionRetry::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CRecognitionRetry::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

long CRecognitionRetry::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
	return VisionErrorEmpty;
}

long CRecognitionRetry::ClearVisionErrorEmpty(long FeederNo)
{
	long Err = NO_ERR;
	gcStep->ClearVisionErrorEmpty(FeederNo);
	return Err;
}

long CRecognitionRetry::SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty)
{
	long Err = NO_ERR;
	gcStep->SetVisionErrorEmpty(FeederNo, VisionErrorEmpty);
	return Err;
}

long CRecognitionRetry::SetVisionError(long InsertOrd, long VisionError)
{
	long Err = NO_ERR;
	gcStep->SetVisionError(InsertOrd, VisionError);
	return Err;
}

long CRecognitionRetry::GetEmptyError(long InsertOrd)
{
	long EmptyError = 0;
	EmptyError = gcStep->GetEmptyError(InsertOrd);
	return EmptyError;
}

bool CRecognitionRetry::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	bSendEmpty = gcStep->GetSendEmpty(FeederNo);
	return bSendEmpty;
}

long CRecognitionRetry::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	long Err = NO_ERR;
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	return Err;
}

long CRecognitionRetry::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	return ReadyTimeOutEmpty;
}

long CRecognitionRetry::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CRecognitionRetry::ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionRetry::RecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionRetry::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, FHCAM, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CRecognitionRetry::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionRetry::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CRecognitionRetry::MoveOneZDn(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionRetry::MoveSomeZDn(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = StartSomeZAxisWaitDelayedPosSet(Gantry, Target);
	return Err;
}

long CRecognitionRetry::Run(bool bManualRecog)
{
	long Gantry = m_Gantry, insertNo = 0, RecogTable = FRONT_STAGE, OldRecogTable = FRONT_STAGE;
	Point_XY pt, CurPt;
	Point_XYT ResPartSize;
	Point_XYRE res;
	long UseVA[MAXVAHEAD], FrameNo[MAXVAHEAD], CamNo[MAXVAHEAD], VisChk[MAXVAHEAD], VisDB[MAXVAHEAD];
	double AngleVA[MAXVAHEAD];
	long Forming[MAXVAHEAD];
	long Ret = 0, err = 0, FeederNo = 0, NozzleNo = 0, HeadNo = 0, Point = 0;
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	Ratio_XYRZ CompRatio[MAXUSEDHEADNO], MinRatio, MoveRatio;
	SomeTarget Target;
	NOZZLE Nozzle[MAXUSEDHEADNO];
	long CatchOrder[MAXUSEDHEADNO];
	long CatchUseVA[MAXUSEDHEADNO];
	long CatchCamNo[MAXUSEDHEADNO];
	long CatchHeadNo[MAXUSEDHEADNO];
	long CatchVisChk[MAXUSEDHEADNO];
	long CatchVisDB[MAXUSEDHEADNO];
	MODULE_LED RetryLed[MAXUSEDHEADNO][MAXRETRYCNT_5];
	double CatchAngleVA[MAXUSEDHEADNO];
	Ratio_XYRZ CatchCompRatio[MAXUSEDHEADNO];
	double CatchComponentHeight[MAXUSEDHEADNO];
	double CatchComponentLeadHeight[MAXUSEDHEADNO];
	double CatchComponentVARecognitionOffsetHeight[MAXUSEDHEADNO];
	NOZZLE CatchNozzle[MAXUSEDHEADNO];
	long CatchLaserControl[MAXUSEDHEADNO];
	long CatchDelay[MAXUSEDHEADNO];
	long CatchRetry[MAXUSEDHEADNO];
	long CatchForming[MAXUSEDHEADNO];
	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Err = NO_ERR, insertOrd = 0, LedOrd = 0, MaxCatchOrd = 0;
	double RatioZ = 1.0, Pitch = 1.0, ComponentHeight[MAXUSEDHEADNO], JobAngle[MAXUSEDHEADNO], ComponentLeadHeight[MAXUSEDHEADNO], ComponentVARecognitionOffsetHeight[MAXUSEDHEADNO];
	double AlignCheckingZ = GetInsertByZ(Gantry);
	long Cam[MAXUSEDHEADNO], ChkPos[MAXUSEDHEADNO], /*TargetHeadNo, */JobHeadNo[MAXUSEDHEADNO], JobFdNo[MAXUSEDHEADNO], JobDBNo[MAXUSEDHEADNO], RecognitionTable[MAXUSEDHEADNO];
	long CatchUnit1st = 0, CatchUnit2nd = 0, CatchUnit3rd = 0;
	long CatchUnit1stCatchOrd = 0, CatchUnit2ndCatchOrd = 0, CatchUnit3rdCatchOrd = 0;
	bool bFirstMoveR = true, bFirstMoveZ = true, bSaftyZUp = false;
	long MaxCatchDelay = 0, MaxRetryCount = 0, RetryOrd = 0, RetryRecog = 0;
	long RetryUse[MAXUSEDHEADNO];

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
	ZeroMemory(&CatchLaserControl, sizeof(CatchLaserControl));
	ZeroMemory(&CatchDelay, sizeof(CatchDelay));
	ZeroMemory(&CatchRetry, sizeof(CatchRetry));
	ZeroMemory(&RetryUse, sizeof(RetryUse));
	ZeroMemory(&RetryLed, sizeof(RetryLed));
	ZeroMemory(&Forming, sizeof(Forming));
	ZeroMemory(&CatchForming, sizeof(CatchForming));

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
		res = gGetRunVisionResult(Gantry, insertOrd);
		HeadNo = GetHeadNoFromInsertOrder(insertOrd + 1);
		Point = GetInsertNoFromInsertOrder(insertOrd + 1);
		JobFdNo[insertOrd] = GetFdNoFromInsertOrder(insertOrd + 1);
		if (GetProdRunMode() == RUN_REAL)
		{
			if (GetReadyTimeOutEmptyByHeadNo(HeadNo) == 1)
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] Retry InsertNo:%03d Hd:%02d Fd:%03d NG ReadyIO TimeOut(%d)\n"), Point, HeadNo, JobFdNo[insertOrd], GetReadyTimeOutEmptyByHeadNo(HeadNo));
				TRACE(_T("[PWR] ************************************************************\n"));
				continue;
			}
			else if (GetDivideInspect(JobFdNo[insertOrd]).Use == true)
			{
				TRACE(_T("[PWR] SingleRecogRetrySkip Hd:%02d Fd:%03d\n"), JobHeadNo[insertOrd], JobFdNo[insertOrd]);
				continue;
			}
			else if (res.exe != 1)
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] Retry InsertNo:%03d Hd:%02d Fd:%03d NG ErrorCode:%d\n"), Point, HeadNo, JobFdNo[insertOrd], gGetRunVisionErrorCode(Gantry, insertOrd));
				TRACE(_T("[PWR] ************************************************************\n"));
				JobHeadNo[insertOrd] = GetHeadNoFromInsertOrder(insertOrd + 1);				
				JobDBNo[insertOrd] = JobFdNo[insertOrd];
				SetInsertOrderFromHeadNo(JobHeadNo[insertOrd], insertOrd + 1);
				SetFeederNoFromHeadNo(JobHeadNo[insertOrd], JobDBNo[insertOrd]);
				insertNo = GetInsertNoFromInsertOrder(insertOrd + 1);
				RecognitionTable[insertOrd] = GetRecognitionTable(insertNo);
				if(GetCameraCount(RecognitionTable[insertOrd]) == CAMERA_COUNT_6)
					ChkPos[insertOrd] = GetCamera6ChkPosByHead(JobHeadNo[insertOrd]);
				else
					ChkPos[insertOrd] = GetCameraChkPosByHead(JobHeadNo[insertOrd]);
				CompRatio[insertOrd] = GetRatioByFdNo(JobFdNo[insertOrd]);
				JobAngle[insertOrd] = GetVAAngleFromFdNo(JobFdNo[insertOrd]);
				ComponentHeight[insertOrd] = GetComponentHeight(JobFdNo[insertOrd]);
				ComponentLeadHeight[insertOrd] = GetComponentLeadHeight(JobFdNo[insertOrd]);
				ComponentVARecognitionOffsetHeight[insertOrd] = GetVAOffsetHeight(JobFdNo[insertOrd]);
				SetRecognitionTableFromHeadNo(JobHeadNo[insertOrd], RecognitionTable[insertOrd]);
				if (GetCameraCount(RecognitionTable[insertOrd]) == CAMERA_COUNT_6)
					Cam[insertOrd] = GetCamera6NoByHead(RecognitionTable[insertOrd], JobHeadNo[insertOrd]) + 1;
				else
					Cam[insertOrd] = GetCameraNoByHead(RecognitionTable[insertOrd], JobHeadNo[insertOrd]) + 1;
				NozzleNo = GetGlobalNozzleNo(JobHeadNo[insertOrd]);
				Nozzle[insertOrd] = GetGlobalNozzleInformation(NozzleNo);
				gInitRunVisionAngle(Gantry, insertOrd);
				gSetRunVisionAngle(Gantry, insertOrd, JobAngle[insertOrd]);
				gSetRunVisionResult(Gantry, insertOrd, res);
				RetryUse[MaxRetryCount] = HeadNo;
				MaxRetryCount++;
			}
			else
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] Retry InsertNo:%03d Hd:%02d Fd:%03d OK(1) ResultXYR:%.3f,%.3f,%.3f\n"), Point, HeadNo, JobFdNo[insertOrd], res.x, res.y, res.r);
				TRACE(_T("[PWR] ************************************************************\n"));
				continue;
			}
		}
	}
	MinRatio = GetMinRatio();
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
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
	if (MaxRetryCount == 0)
	{
		return NO_ERR;
	}
	GetTime = _time_get();
	Err = WaitGantry(Gantry, TIME5000MS);
	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowRunLog() == true || Elapsed > 0)
	{
		TRACE(_T("[PWR] RecognitionRetry WaitGantry Elasped,%d"), Elapsed);
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] RecognitionRetry Run ProdRunMode:%d\n"), GetProdRunMode());
	}
	for (RetryOrd = 0; RetryOrd < MaxRetryCount; ++RetryOrd)
	{
		CatchOrder[RetryOrd] = GetCameraHeadFromHeadNo(RetryUse[RetryOrd]);
		CatchUseVA[RetryOrd] = USE_VA;
		RecogTable = GetRecognitionTableFromHeadNo(RetryUse[RetryOrd]);
		if (GetCameraCount(RecogTable) == CAMERA_COUNT_6)
		{
			CatchCamNo[RetryOrd] = GetCamera6NoByHead(RecogTable, RetryUse[RetryOrd]);
			CatchVisChk[RetryOrd] = GetCamera6ChkPosByHead(RetryUse[RetryOrd]);
		}
		else
		{
			CatchCamNo[RetryOrd] = GetCameraNoByHead(RecogTable, RetryUse[RetryOrd]);
			CatchVisChk[RetryOrd] = GetCameraChkPosByHead(RetryUse[RetryOrd]);
		}		
		CatchVisDB[RetryOrd] = GetFeederNoFromHeadNo(RetryUse[RetryOrd]);
		CatchCompRatio[RetryOrd] = GetRatioByFdNo(CatchVisDB[RetryOrd]);
		CatchHeadNo[RetryOrd] = RetryUse[RetryOrd];
		CatchAngleVA[RetryOrd] = GetVAAngleFromFdNo(CatchVisDB[RetryOrd]);
		CatchComponentHeight[RetryOrd] = GetComponentHeight(CatchVisDB[RetryOrd]);
		CatchComponentLeadHeight[RetryOrd] = GetComponentLeadHeight(CatchVisDB[RetryOrd]);
		CatchComponentVARecognitionOffsetHeight[RetryOrd] = GetVAOffsetHeight(CatchVisDB[RetryOrd]);
		NozzleNo = GetGlobalNozzleNo(RetryUse[RetryOrd]);
		CatchNozzle[RetryOrd] = GetGlobalNozzleInformation(NozzleNo);
		CatchLaserControl[RetryOrd] = GetLaserControl(CatchVisDB[RetryOrd]);
		CatchDelay[RetryOrd] = GetCatchDelay(CatchVisDB[RetryOrd]);
		CatchRetry[RetryOrd] = GetMaxRetryCount(CatchVisDB[RetryOrd]);
		CatchForming[RetryOrd] = GetForming(CatchVisDB[RetryOrd]).Use;
		for (long retry = 0; retry < CatchRetry[RetryOrd]; ++retry)
		{
			RetryLed[RetryOrd][retry] = GetMaxRetryLed(CatchVisDB[RetryOrd], retry);
			TRACE(_T("[PWR] RecognitionRetry(%d) Fd:%03d Retry:%02d LED Top:%03d Mid:%03d Bot:%03d\n"), RetryOrd, CatchVisDB[RetryOrd], retry, RetryLed[RetryOrd][retry].Top, RetryLed[RetryOrd][retry].Mid, RetryLed[RetryOrd][retry].Bot);
		}
	}

	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] RecognitionRetry MaxRetryCount:%d\n"), MaxRetryCount);
		for (RetryOrd = 0; RetryOrd < MaxRetryCount; ++RetryOrd)
		{
			if (CatchUseVA[RetryOrd] != USE_VA) continue;
			TRACE(_T("[PWR] RecognitionRetry RetryOrd(%d) UseVA:%d CamNo:%d HeadNo:%d(%s,%s)\n"),
				RetryOrd, CatchUseVA[RetryOrd], CatchCamNo[RetryOrd], CatchHeadNo[RetryOrd],
				GetRAxisFromHeadNo(Gantry, CatchHeadNo[RetryOrd]),
				GetZAxisFromHeadNo(Gantry, CatchHeadNo[RetryOrd]));
		}
	}
	for (RetryOrd = 0; RetryOrd < MaxRetryCount; ++RetryOrd)
	{
		MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] CRecognition GetGlobalStatusError-1(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] CRecognition GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		for (long indx = 0; indx < MAXVAHEAD; ++indx)
		{
			UseVA[indx] = NON;
			CamNo[indx] = 0;
			VisChk[indx] = 0;
			VisDB[indx] = 0;
			AngleVA[indx] = 0.0;
			Forming[indx] = 0;
		}
		RecogTable = FRONT_STAGE;
		UseVA[RetryOrd] = CatchUseVA[RetryOrd];
		CamNo[RetryOrd] = CatchCamNo[RetryOrd];
		VisChk[RetryOrd] = CatchVisChk[RetryOrd];
		VisDB[RetryOrd] = CatchVisDB[RetryOrd];
		AngleVA[RetryOrd] = CatchAngleVA[RetryOrd];
		Forming[RetryOrd] = CatchForming[RetryOrd];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] RetryOrd(%d) Use(%d) Cam(%d) VisChk(%d) DB(%d)\n"), RetryOrd, UseVA[RetryOrd], CamNo[RetryOrd], VisChk[RetryOrd], VisDB[RetryOrd]);
		}
		Err = RecognitionMoveAllZUp(Gantry, GetStandByZ(Gantry), RatioZ, TIME5000MS);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("RecognitionRetry RecognitionMoveAllZUp Err"));
			TRACE(_T("[PWR] RecognitionRetry RecognitionMoveAllZUp Err,%d\n"), Err);
			return Err;
		}
		for (RetryRecog = 0; RetryRecog < CatchRetry[RetryOrd]; ++RetryRecog)
		{
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gPrepareCommand(Gantry, CamNo, RetryOrd + 1, VisChk, UseVA, VisDB, AngleVA, Forming);
				Elapsed = _time_elapsed(GetTime);
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					TRACE(_T("[PWR] gPrepareCommand RetryOrd(%d,%d) Ret(%d) Elapsed,%d\n"), RetryOrd, RetryRecog, Ret, Elapsed);
				}
				if (Ret == -2)
				{
					TRACE(_T("[PWR] Vision Prepare Err id:%d Ret:%d\n"), RetryOrd + 1, Ret);
					Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Prepare Error"));
					return Ret;
				}
			}
			else // Simulation
			{
				ThreadSleep(TIME30MS);
			}
			//TargetHeadNo = GetCameraHeadFromHeadNo(JobHeadNo[insertOrd]);
			if (CatchCamNo[RetryOrd] < RCAM1)
				RecogTable = FRONT_STAGE;
			else
				RecogTable = REAR_STAGE;
			pt = GetCameraRecognitionPosition(RecogTable, CatchOrder[RetryOrd]);
			gLedOn(CatchCamNo[RetryOrd], RetryLed[RetryOrd][RetryRecog].Top, RetryLed[RetryOrd][RetryRecog].Mid, RetryLed[RetryOrd][RetryRecog].Bot);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] RetryOrd(%d,%d) Cam:%d Led(Top,Mid,Bot)(%d,%d,%d) LaserControl:%d\n"), RetryOrd, RetryRecog, CatchCamNo[RetryOrd], RetryLed[RetryOrd][RetryRecog].Top, RetryLed[RetryOrd][RetryRecog].Mid, RetryLed[RetryOrd][RetryRecog].Bot, CatchLaserControl[RetryOrd]);
			}
			//if (RetryLed[RetryOrd][RetryRecog].Bot != 0)
			//{
			//	gLaserOn(CatchCamNo[RetryOrd]);
			//}
			//else if (CatchLaserControl[RetryOrd] == 1)
			if (CatchLaserControl[RetryOrd] == 1)
			{
				gLaserOn(CatchCamNo[RetryOrd]);
			}
			else
			{
				gLaserOff(CatchCamNo[RetryOrd]);
			}
			Target.MaxAxisCount = 0;
			Target.Axis[RetryOrd] = GetRAxisFromHeadNo(Gantry, CatchHeadNo[RetryOrd]);
			Target.Command[RetryOrd] = CatchAngleVA[RetryOrd] + GetManualVisionOffset(RetryOrd + 1);
			Target.Ratio[RetryOrd] = CatchCompRatio[RetryOrd].r;
			Target.Inpos[RetryOrd] = InposR;
			Target.InposMs[RetryOrd] = MsR;
			Target.TimeOut[RetryOrd] = TimeOut;
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] RetryOrd(%d,%d) RMoveIndex(%s) Command:%.3f\n"), RetryOrd, RetryRecog, Target.Axis[RetryOrd], Target.Command[RetryOrd]);
			}
			Err = MoveR(Target.Axis[RetryOrd], Target.Ratio[RetryOrd], Target.TimeOut[RetryOrd], Target.Command[RetryOrd], Target.Inpos[RetryOrd], Target.InposMs[RetryOrd], false);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("RecognitionRetry MoveR"));
				TRACE(_T("[PWR] RecognitionRetry MoveR Err,%d\n"), Err);
				return Err;
			}
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				CurPt = gReadGantryPosition(Gantry);
			}
			MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
			//TRACE(_T("[PWR] RecognitionRetry MinRatio XYRZ,%.1f,%.1f,%.1f MoveRatio XYRZ,%.1f,%.1f,%1f\n"), MinRatio.xy, MinRatio.r, MinRatio.z, MoveRatio.xy, MoveRatio.r, MoveRatio.z);
			if (MinRatio.xy < MoveRatio.xy)
				MoveRatio = MinRatio;
			GetTime = _time_get();
			Err = MoveXY(Gantry, FHCAM, pt, MoveRatio.xy, InposXY, MsXy, TimeOut);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("RecognitionRetry MoveXY"));
				TRACE(_T("[PWR] RecognitionRetry MoveXY Err,%d\n"), Err);
				return Err;
			}
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				Elapsed = _time_elapsed(GetTime);
				TRACE(_T("[PWR] RecognitionRetry Dist (%.3f, %.3f) XY Elapsed,%d\n"), pt.x - CurPt.x, pt.y - CurPt.y, Elapsed);
			}
			GetTime = _time_get();
			Err = WaitR(Target.Axis[RetryOrd], Target.Command[RetryOrd], Target.Inpos[RetryOrd], Target.TimeOut[RetryOrd]);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("RecognitionRetry WaitR"));
				TRACE(_T("[PWR] RecognitionRetry WaitR(%s) Err,%d\n"), Target.Axis[RetryOrd], Err);
				return Err;
			}
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				Elapsed = _time_elapsed(GetTime);
				TRACE(_T("[PWR] RecognitionRetry R Elapsed,%d\n"), Elapsed);
			}
			ClearManualVisionOffset();
			//if (bFirstMoveZ == true || bSaftyZUp == true)
			{
				Target.MaxAxisCount = 0;
				if (gcPowerLog->IsShowRunLog() == true)
				{
					TRACE(_T("[PWR] RetryOrd:%d(%d) HeadNo:%d(%s)\n"),
						RetryOrd, RetryRecog,
						CatchHeadNo[RetryOrd],
						GetZAxisFromHeadNo(Gantry, CatchHeadNo[RetryOrd]));
				}
				Target.Axis[RetryOrd] = GetZAxisFromHeadNo(Gantry, CatchHeadNo[RetryOrd]);
				Target.Command[RetryOrd] = GetInsertByZ(Gantry) - (CatchComponentHeight[RetryOrd] + CatchComponentLeadHeight[RetryOrd]) - CatchNozzle[RetryOrd].TipHeight + CatchNozzle[RetryOrd].PusherHeight + CatchComponentVARecognitionOffsetHeight[RetryOrd];
				if (GetProdRunMode() == RUN_DRY)
				{
					Target.Command[RetryOrd] -= GetDryRunZHeightOffset();
				}
				Target.Ratio[RetryOrd] = CatchCompRatio[RetryOrd].z;
				Target.Inpos[RetryOrd] = InposZDn;
				Target.InposMs[RetryOrd] = MsZDn;
				Target.TimeOut[RetryOrd] = TimeOut;
				if (gcPowerLog->IsShowRunLog() == true)
				{
					TRACE(_T("[PWR] RetryOrd(%d,%d)(%s) ComponeneHeight(%.3f) LeadHeight(%.3f) Nozzle No,Type(%d,%d) TipHeight:%.3f PusherHeight:%.3f RecognitionHeightOffset:%.3f Target:%.3f\n"),
						RetryOrd, RetryRecog,
						Target.Axis[RetryOrd],
						CatchComponentHeight[RetryOrd],
						CatchComponentLeadHeight[RetryOrd],
						CatchNozzle[RetryOrd].No,
						CatchNozzle[RetryOrd].Type,
						CatchNozzle[RetryOrd].TipHeight,
						CatchNozzle[RetryOrd].PusherHeight,
						CatchComponentVARecognitionOffsetHeight[RetryOrd],
						Target.Command[RetryOrd]);
				}
				GetTime = _time_get();
				if (Target.MaxAxisCount < MIN_LINEAR_INTP_AXIS)
				{
					Err = MoveOneZDn(Target.Axis[RetryOrd], Target.Ratio[RetryOrd], Target.TimeOut[RetryOrd], Target.Command[RetryOrd], Target.Inpos[RetryOrd], Target.InposMs[RetryOrd], true);
				}
				else
				{
					Err = MoveSomeZDn(Gantry, Target);
				}
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] RecognitionRetry One or Some Z Dn Elapsed,%d\n"), Elapsed);
				}
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("RecognitionRetry MoveOne or MoveSome ZDn"));
					TRACE(_T("[PWR] RecognitionRetry MoveOne or MoveSome ZDn Err,%d\n"), Err);
					return Err;
				}
				bFirstMoveZ = false;
			}
			MaxCatchDelay = CatchDelay[RetryOrd];
			if (MaxCatchDelay > 0)
			{
				ThreadSleep(MaxCatchDelay);
				TRACE(_T("[PWR] Delay(%d)[ms] Before Catch image by Camera\n"), MaxCatchDelay);
			}
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gImageCatch(Gantry, CamNo, RetryOrd + 1, VisChk, UseVA, 1);
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] gImageCatch RetryOrd(%d,%d) Ret(%d) Elapsed,%d\n"), RetryOrd, RetryRecog, Ret, Elapsed);
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
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gStartProcess(CatchCamNo[RetryOrd], RetryOrd + 1);
				if (Ret == -2)
				{
					TRACE(_T("[PWR] Vision Process Err Cam:%d id:%d Ret:%d\n"), CatchCamNo[RetryOrd], RetryOrd + 1, Ret);
					Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Process Error"));
					return Ret;
				}
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] gStartProcess RetryOrd(%d,%d) Ret(%d) Elapsed,%d\n"), RetryOrd, RetryRecog, Ret, Elapsed);
				}
			}
			else // Simulation
			{
				ThreadSleep(TIME30MS);
			}
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gGetRecognitionResult(CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], RetryOrd + 1, &err, &res, FrameNo, &ResPartSize);
				if (Ret == -2)
				{
					TRACE(_T("[PWR] Vision Recognition Err:%d Cam:%d loc:%d id:%d Frame:%d Ret:%d\n"), err, CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], RetryOrd + 1, FrameNo, Ret);
					Ret = SendAlarm(VISION_RECEIVE_TIMEOUT, _T("Vision Result Error"));
					return Ret;
				}
				insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[RetryOrd]);
				FeederNo = GetFdNoFromInsertOrder(insertOrd);
				SendToLedRetrytatistics(FeederNo);

				if (insertOrd > 0)
				{
					if (GetSkipVision() == 1) // Simulation
					{
						res.exe = 1;
						res.x = res.y = res.r = 0.000;
						err = NO_ERR;
						TRACE(_T("[PWR] Set Run Vision Simulation Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r);
						gSetPartRecognitionResult(CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], res);
						gSetRunVisionResult(Gantry, insertOrd - 1, res);
						gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
						SetVisionError(insertOrd, 0);
						ClearVisionErrorEmpty(FeederNo);
						if (GetSimulationForming() == true)
						{
							ResPartSize.x = ResPartSize.y = 26.0;
							gSetRunVisionPitchResult(Gantry, insertOrd - 1, ResPartSize);
						}
						else
						{
							gSetRunVisionPitchResult(Gantry, insertOrd - 1, ResPartSize);
						}
					}
					else
					{
						TRACE(_T("[PWR] Set Run Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r);
						gSetPartRecognitionResult(CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], res);
						gSetRunVisionResult(Gantry, insertOrd - 1, res);
						gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);
						if (GetSimulationForming() == true)
						{
							ResPartSize.x = ResPartSize.y = 26.0;
							gSetRunVisionPitchResult(Gantry, insertOrd - 1, ResPartSize);
						}
						else
						{
							gSetRunVisionPitchResult(Gantry, insertOrd - 1, ResPartSize);
						}
						if (bManualRecog == false && err == 503 && (RetryRecog + 1 == CatchRetry[RetryOrd])) // 미흡창 통계 제외
						{
							SendToNoComponentStatistics(FeederNo, CatchNozzle[RetryOrd].No);
						}

						if (res.exe != 1)
						{
							SetVisionError(insertOrd, 1);
							if (RetryRecog + 1 == CatchRetry[RetryOrd])
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
							break; // Retry
						}
					}
				}
				Elapsed = _time_elapsed(GetTime);
				if (gcPowerLog->IsShowVisionLog() == true)
				{
					TRACE(_T("[PWR] gGetRecognitionResult Cam(%d) VisChk(%d) Ret(%d) Err(%d) Res:%.3f %.3f %.3f SizeXY %.3f %.3f Elapsed,%d\n"),
						CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], Ret, err, res.x, res.y, res.r,
						ResPartSize.x, ResPartSize.y, Elapsed);
				}
			}
			else
			{
				ThreadSleep(TIME30MS);
				insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[RetryOrd]);
				res.exe = 1;
				res.x = 0.0;
				res.y = 0.0;
				res.r = 0.0;
				err = 0;
				gSetRunVisionResult(Gantry, insertOrd - 1, res);
				gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);

				if (GetSimulationForming() == true)
				{
					ResPartSize.x = ResPartSize.y = 26.0;
					gSetRunVisionPitchResult(Gantry, insertOrd - 1, ResPartSize);
				}
				TRACE(_T("[PWR] Set DryRun Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f PitchXY:%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r, ResPartSize.x, ResPartSize.y);


			}
		}
	}
	gLedAllOff();
	gLaserAllOff();
	return Err;
}

Ratio_XYRZ CRecognitionRetry::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio.xy = MinRatio.r = MinRatio.z = 1.0;
	MinRatio = gcStep->GetMinRatio();
	TRACE(_T("[PWR] CRecognitionRetry MinRatioXYRZ,%.1f,%.1f,%.1f"), MinRatio.xy, MinRatio.r, MinRatio.z);
	return MinRatio;
}

long CRecognitionRetry::GetLaserControl(long FeederNo)
{
	long LaserControl = 0;
	LaserControl = gcStep->GetLaserControl(FeederNo);
	return LaserControl;
}

long CRecognitionRetry::GetCatchDelay(long FeederNo)
{
	long CatchDelay = 0;
	CatchDelay = gcStep->GetCatchDelay(FeederNo);
	return CatchDelay;
}

long CRecognitionRetry::GetMaxRetryCount(long FeederNo)
{
	long MaxRetry = 0;
	MaxRetry = gcStep->GetRetryLed(FeederNo).MaxRetry;
	return MaxRetry;
}

MODULE_LED CRecognitionRetry::GetMaxRetryLed(long FeederNo, long Retry)
{
	MODULE_LED Led;
	Led = gcStep->GetRetryLed(FeederNo).Led[Retry];
	return Led;
}

DIVIDE_INSPECT CRecognitionRetry::GetDivideInspect(long FeederNo)
{
	return gcStep->GetDivideInspect(FeederNo);
}

FORMING_COMPONENT CRecognitionRetry::GetForming(long FeederNo)
{
	FORMING_COMPONENT Forming;
	ZeroMemory(&Forming, sizeof(Forming));
	Forming = gcStep->GetForming(FeederNo);
	return Forming;
}