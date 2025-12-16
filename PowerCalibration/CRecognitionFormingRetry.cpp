#include "pch.h"
#include "CRecognitionFormingRetry.h"
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

CRecognitionFormingRetry* gcRecognitionFormingRetry;
CRecognitionFormingRetry::CRecognitionFormingRetry(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxInsertOrder = 0;
	m_ProdRunMode = RUN_REAL;
	ZeroMemory(&m_FeederNo, sizeof(m_FeederNo));
	ZeroMemory(&m_InsertOrder, sizeof(m_InsertOrder));
	ZeroMemory(&m_RecognitionTable, sizeof(m_RecognitionTable));
	ClearManualVisionOffset();
}

CRecognitionFormingRetry::~CRecognitionFormingRetry()
{
}

long CRecognitionFormingRetry::GetTable()
{
	return m_Gantry;
}

void CRecognitionFormingRetry::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CRecognitionFormingRetry::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CRecognitionFormingRetry::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CRecognitionFormingRetry::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CRecognitionFormingRetry::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CRecognitionFormingRetry::GetFeederNoFromHeadNo(long HeadNo)
{
	long FeederNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		FeederNo = m_FeederNo[HeadNo - 1];
	}
	return FeederNo;
}

long CRecognitionFormingRetry::GetInsertOrderFromHeadNo(long HeadNo)
{
	long InsertOder = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		InsertOder = m_InsertOrder[HeadNo - 1];
	}
	return InsertOder;
}

long CRecognitionFormingRetry::GetRecognitionTableFromHeadNo(long HeadNo)
{
	long RecogTable = FRONT_GANTRY;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		RecogTable = m_RecognitionTable[HeadNo - 1];
	}
	return RecogTable;
}

long CRecognitionFormingRetry::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	return ReadyTimeOutEmpty;
}

Ratio_XYRZ CRecognitionFormingRetry::GetRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

double CRecognitionFormingRetry::GetVAAngleFromFdNo(long insertOrd)
{
	double RecognitionAngle = 0.0;
	RecognitionAngle = gcStep->GetVAAngleFromFdNo(insertOrd);
	return RecognitionAngle;
}

double CRecognitionFormingRetry::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionFormingRetry::GetComponentLeadHeight(long FdNo)
{
	double ComponentHeight = 5.0;
	ComponentHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CRecognitionFormingRetry::GetVAOffsetHeight(long FdNo)
{
	double VARecognitionOffsetHeight = 0.0;
	VARecognitionOffsetHeight = gcStep->GetVAOffsetHeight(FdNo);
	return VARecognitionOffsetHeight;
}

MODULE_LED CRecognitionFormingRetry::GetLed(long FdNo)
{
	MODULE_LED Led;
	Led = gcStep->GetLed(FdNo);
	return Led;
}

long CRecognitionFormingRetry::GetManualCompensationUse()
{
	long ManualCompenUse = 0;
	ManualCompenUse = gcStep->GetManualCompensationUse();
	return ManualCompenUse;
}

Point_XYRZ CRecognitionFormingRetry::GetManualVisionResult()
{
	Point_XYRZ ManualVisionResult;
	ManualVisionResult = gcStep->GetManualVisionResult();
	return ManualVisionResult;
}

long CRecognitionFormingRetry::ClearManualVisionOffset()
{
	ZeroMemory(&m_VAAngleOffset, sizeof(m_VAAngleOffset));
	return NO_ERR;
}

long CRecognitionFormingRetry::SetManualVisionOffset(long insertOrd, double OffsetAngle)
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

double CRecognitionFormingRetry::GetManualVisionOffset(long insertOrd)
{
	double OffsetAngle = 0.0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		OffsetAngle = m_VAAngleOffset[insertOrd - 1];
	}
	return OffsetAngle;
}

long CRecognitionFormingRetry::SetFeederNoFromHeadNo(long HeadNo, long FeederNo)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_FeederNo[HeadNo - 1] = FeederNo;
	}
	return NO_ERR;
}

long CRecognitionFormingRetry::SetInsertOrderFromHeadNo(long HeadNo, long InsertOrder)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_InsertOrder[HeadNo - 1] = InsertOrder;
	}
	return NO_ERR;
}

long CRecognitionFormingRetry::SetRecognitionTableFromHeadNo(long HeadNo, long RecogTable)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_RecognitionTable[HeadNo - 1] = RecogTable;
	}
	return NO_ERR;
}

long CRecognitionFormingRetry::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

long CRecognitionFormingRetry::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

long CRecognitionFormingRetry::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
	return VisionErrorEmpty;
}

long CRecognitionFormingRetry::ClearVisionErrorEmpty(long FeederNo)
{
	long Err = NO_ERR;
	gcStep->ClearVisionErrorEmpty(FeederNo);
	return Err;
}

long CRecognitionFormingRetry::SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty)
{
	long Err = NO_ERR;
	gcStep->SetVisionErrorEmpty(FeederNo, VisionErrorEmpty);
	return Err;
}

long CRecognitionFormingRetry::SetVisionError(long InsertOrd, long VisionError)
{
	long Err = NO_ERR;
	gcStep->SetVisionError(InsertOrd, VisionError);
	return Err;
}

long CRecognitionFormingRetry::GetEmptyError(long InsertOrd)
{
	long EmptyError = 0;
	EmptyError = gcStep->GetEmptyError(InsertOrd);
	return EmptyError;
}

bool CRecognitionFormingRetry::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	bSendEmpty = gcStep->GetSendEmpty(FeederNo);
	return bSendEmpty;
}

long CRecognitionFormingRetry::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	long Err = NO_ERR;
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	return Err;
}

long CRecognitionFormingRetry::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	return ReadyTimeOutEmpty;
}

long CRecognitionFormingRetry::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CRecognitionFormingRetry::ManualRecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionFormingRetry::RecognitionMoveAllZUp(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, pt, Ratio);
	return Err;
}

long CRecognitionFormingRetry::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, FHCAM, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CRecognitionFormingRetry::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionFormingRetry::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CRecognitionFormingRetry::MoveOneZDn(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CRecognitionFormingRetry::MoveSomeZDn(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = StartSomeZAxisWaitDelayedPosSet(Gantry, Target);
	return Err;
}

long CRecognitionFormingRetry::Run(bool bManualRecog)
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
	MODULE_LED Led[MAXUSEDHEADNO];
	double CatchAngleVA[MAXUSEDHEADNO];
	Ratio_XYRZ CatchCompRatio[MAXUSEDHEADNO];
	double CatchComponentHeight[MAXUSEDHEADNO];
	double CatchComponentLeadHeight[MAXUSEDHEADNO];
	double CatchComponentVARecognitionOffsetHeight[MAXUSEDHEADNO];
	NOZZLE CatchNozzle[MAXUSEDHEADNO];
	long CatchLaserControl[MAXUSEDHEADNO];
	long CatchDelay[MAXUSEDHEADNO];
	long CatchRetry[MAXUSEDHEADNO];
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
	ZeroMemory(&Led, sizeof(Led));
	ZeroMemory(&Forming, sizeof(Forming));

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
		JobHeadNo[insertOrd] = GetHeadNoFromInsertOrder(insertOrd + 1);

		//if (GetProdRunMode() == RUN_REAL)
		{
			if (GetDivideInspect(JobFdNo[insertOrd]).Use == true)
			{
				TRACE(_T("[PWR] SingleRecogRetrySkip Hd:%02d Fd:%03d\n"), JobHeadNo[insertOrd], JobFdNo[insertOrd]);
				continue;
			}

			if (GetFormingMotionDone(insertOrd + 1) == true)
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] FormingRetry InsertNo:%03d Hd:%02d Fd:%03d FormingMotionDone:%d\n"), Point, HeadNo, JobFdNo[insertOrd], GetFormingMotionDone(insertOrd + 1));
				TRACE(_T("[PWR] ************************************************************\n"));
				JobDBNo[insertOrd] = JobFdNo[insertOrd];
				SetInsertOrderFromHeadNo(JobHeadNo[insertOrd], insertOrd + 1);
				SetFeederNoFromHeadNo(JobHeadNo[insertOrd], JobDBNo[insertOrd]);
				ChkPos[insertOrd] = GetCameraChkPosByHead(JobHeadNo[insertOrd]);
				CompRatio[insertOrd] = GetRatioByFdNo(JobFdNo[insertOrd]);
				JobAngle[insertOrd] = GetVAAngleFromFdNo(JobFdNo[insertOrd]);
				ComponentHeight[insertOrd] = GetComponentHeight(JobFdNo[insertOrd]);
				ComponentLeadHeight[insertOrd] = GetComponentLeadHeight(JobFdNo[insertOrd]);
				ComponentVARecognitionOffsetHeight[insertOrd] = GetVAOffsetHeight(JobFdNo[insertOrd]);
				insertNo = GetInsertNoFromInsertOrder(insertOrd + 1);
				RecognitionTable[insertOrd] = GetRecognitionTable(insertNo);
				SetRecognitionTableFromHeadNo(JobHeadNo[insertOrd], RecognitionTable[insertOrd]);
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
				TRACE(_T("[PWR] FormingRetry Skip InsertNo:%03d Hd:%02d Fd:%03d FormingMotionDone:%d\n"), Point, HeadNo, JobFdNo[insertOrd], GetFormingMotionDone(insertOrd + 1));
				TRACE(_T("[PWR] ************************************************************\n"));
				continue;
			}
		}
	}
	MinRatio = GetMinRatio();
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
	//long ReadyTimeOutEmpty = 0;
	//for (insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	//{
	//	if (GetEmptyError(insertOrd + 1) == 1)
	//	{
	//		ReadyTimeOutEmpty++;
	//	}
	//}
	//if (ReadyTimeOutEmpty > 0)
	//{
	//	if (ReadyTimeOutEmpty == GetMaxInsertOrder())
	//	{
	//		return Err;
	//	}
	//}
	if (MaxRetryCount == 0)
	{
		return NO_ERR;
	}
	GetTime = _time_get();
	Err = WaitGantry(Gantry, TIME5000MS);
	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowRunLog() == true || Elapsed > 0)
	{
		TRACE(_T("[PWR] RecognitionFormingRetry WaitGantry Elasped,%d"), Elapsed);
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] RecognitionFormingRetry Run ProdRunMode:%d\n"), GetProdRunMode());
	}
	for (RetryOrd = 0; RetryOrd < MaxRetryCount; ++RetryOrd)
	{
		CatchOrder[RetryOrd] = GetCameraHeadFromHeadNo(RetryUse[RetryOrd]);
		CatchUseVA[RetryOrd] = USE_VA;
		RecogTable = GetRecognitionTableFromHeadNo(RetryUse[RetryOrd]);
		CatchCamNo[RetryOrd] = GetCameraNoByHead(RecogTable, RetryUse[RetryOrd]);
		CatchVisChk[RetryOrd] = GetCameraChkPosByHead(RetryUse[RetryOrd]);
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
		Led[RetryOrd] = GetLed(CatchVisDB[RetryOrd]);
		//for (long retry = 0; retry < CatchRetry[RetryOrd]; ++retry)
		{
			long retry = 0;
			//RetryLed[RetryOrd][retry] = GetMaxRetryLed(CatchVisDB[RetryOrd], retry);
			TRACE(_T("[PWR] RecognitionFormingRetry(%d) Fd:%03d Retry:%02d LED Top:%03d Mid:%03d Bot:%03d\n"), RetryOrd, CatchVisDB[RetryOrd], retry, Led[RetryOrd].Top, Led[RetryOrd].Mid, Led[RetryOrd].Bot);
		}
	}

	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] RecognitionFormingRetry MaxRetryCount:%d\n"), MaxRetryCount);
		for (RetryOrd = 0; RetryOrd < MaxRetryCount; ++RetryOrd)
		{
			if (CatchUseVA[RetryOrd] != USE_VA) continue;
			TRACE(_T("[PWR] RecognitionFormingRetry RetryOrd(%d) UseVA:%d CamNo:%d HeadNo:%d(%s,%s)\n"),
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
			TRACE(_T("[PWR] RecognitionFormingRetry GetGlobalStatusError-1(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] RecognitionFormingRetry GetMachineState(%d)\n"), GetMachineState());
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
		RecogTable = FRONT_STAGE;
		UseVA[RetryOrd] = CatchUseVA[RetryOrd];
		CamNo[RetryOrd] = CatchCamNo[RetryOrd];
		VisChk[RetryOrd] = CatchVisChk[RetryOrd];
		VisDB[RetryOrd] = CatchVisDB[RetryOrd];
		AngleVA[RetryOrd] = CatchAngleVA[RetryOrd];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] RecognitionFormingRetry(%d) Use(%d) Cam(%d) VisChk(%d) DB(%d)\n"), RetryOrd, UseVA[RetryOrd], CamNo[RetryOrd], VisChk[RetryOrd], VisDB[RetryOrd]);
		}
		Err = GantryMoveStandByZ(Gantry, MinRatio.z);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("RecognitionFormingRetry GantryMoveStandByZ Err"));
			TRACE(_T("[PWR] RecognitionFormingRetry GantryMoveStandByZ Err,%d\n"), Err);
			return Err;
		}
		//for (RetryRecog = 0; RetryRecog < CatchRetry[RetryOrd]; ++RetryRecog)
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
			gLedOn(CatchCamNo[RetryOrd], Led[RetryOrd].Top, Led[RetryOrd].Mid, Led[RetryOrd].Bot);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] RecognitionFormingRetry(%d,%d) Cam:%d Led(Top,Mid,Bot)(%d,%d,%d) LaserControl:%d\n"), RetryOrd, RetryRecog, CatchCamNo[RetryOrd], Led[RetryOrd].Top, Led[RetryOrd].Mid, Led[RetryOrd].Bot, CatchLaserControl[RetryOrd]);
			}
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
				TRACE(_T("[PWR] RecognitionFormingRetry(%d,%d) RMoveIndex(%s) Command:%.3f\n"), RetryOrd, RetryRecog, Target.Axis[RetryOrd], Target.Command[RetryOrd]);
			}
			Err = MoveR(Target.Axis[RetryOrd], Target.Ratio[RetryOrd], Target.TimeOut[RetryOrd], Target.Command[RetryOrd], Target.Inpos[RetryOrd], Target.InposMs[RetryOrd], false);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("RecognitionFormingRetry MoveR"));
				TRACE(_T("[PWR] RecognitionFormingRetry MoveR Err,%d\n"), Err);
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
				Err = SendAlarm(Err, _T("RecognitionFormingRetry MoveXY"));
				TRACE(_T("[PWR] RecognitionFormingRetry MoveXY Err,%d\n"), Err);
				return Err;
			}
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				Elapsed = _time_elapsed(GetTime);
				TRACE(_T("[PWR] RecognitionFormingRetry Dist (%.3f, %.3f) XY Elapsed,%d\n"), pt.x - CurPt.x, pt.y - CurPt.y, Elapsed);
			}
			GetTime = _time_get();
			Err = WaitR(Target.Axis[RetryOrd], Target.Command[RetryOrd], Target.Inpos[RetryOrd], Target.TimeOut[RetryOrd]);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("RecognitionFormingRetry WaitR"));
				TRACE(_T("[PWR] RecognitionFormingRetry WaitR(%s) Err,%d\n"), Target.Axis[RetryOrd], Err);
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
					TRACE(_T("[PWR] RecognitionFormingRetry:%d(%d) HeadNo:%d(%s)\n"),
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
					TRACE(_T("[PWR] RecognitionFormingRetry(%d,%d)(%s) ComponeneHeight(%.3f) LeadHeight(%.3f) Nozzle No,Type(%d,%d) TipHeight:%.3f PusherHeight:%.3f RecognitionHeightOffset:%.3f Target:%.3f\n"),
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
					TRACE(_T("[PWR] RecognitionFormingRetry One or Some Z Dn Elapsed,%d\n"), Elapsed);
				}
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("RecognitionFormingRetry MoveOne or MoveSome ZDn"));
					TRACE(_T("[PWR] RecognitionFormingRetry MoveOne or MoveSome ZDn Err,%d\n"), Err);
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
			}
			else // Simulation
			{
				ThreadSleep(TIME30MS);
			}
			if (GetProdRunMode() == RUN_REAL)
			{
				GetTime = _time_get();
				Ret = gStartProcess(CatchCamNo[RetryOrd], RetryOrd + 1);
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
				insertOrd = GetInsertOrderFromHeadNo(CatchHeadNo[RetryOrd]);
				FeederNo = GetFdNoFromInsertOrder(insertOrd);
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
					}
					else
					{
						TRACE(_T("[PWR] Set Run Vision Result insertOrd(%d) Exe(%d) X,Y,R,%.3f,%.3f,%.3f\n"), insertOrd, res.exe, res.x, res.y, res.r);
						gSetPartRecognitionResult(CatchCamNo[RetryOrd], CatchVisChk[RetryOrd], res);
						gSetRunVisionResult(Gantry, insertOrd - 1, res);
						gSetRunVisionErrorCode(Gantry, insertOrd - 1, err);

						if (bManualRecog == false && err == 503 && (RetryRecog + 1 == CatchRetry[RetryOrd])) // 미흡창 통계 제외
						{
							SendToNoComponentStatistics(FeederNo, CatchNozzle[RetryOrd].No);
						}

						if (res.exe != 1)
						{
							SetVisionError(insertOrd, 1);
							if (RetryRecog + 1 == CatchRetry[RetryOrd])
							{
								SetVisionErrorEmpty(FeederNo, 1);
								//if (GetPickupErrorUse(FeederNo) > 0)
								//{
								//	AddPickupErrorCount(FeederNo, 1);
								//	if (GetPickupErrorRawCount(FeederNo) > GetPickupErrorReferenceCount(FeederNo)) // 알람 취합 수량 > 알람 참조 수량
								//	{
								//		RemovePickupErrorCount(FeederNo);
								//	}
								//}

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
							//if (GetPickupErrorUse(FeederNo) > 0)
							//{
							//	AddPickupErrorCount(FeederNo, 0);
							//	if (GetPickupErrorRawCount(FeederNo) > GetPickupErrorReferenceCount(FeederNo)) // 알람 취합 수량 > 알람 참조 수량
							//	{
							//		RemovePickupErrorCount(FeederNo);
							//	}
							//}
							//break; // Retry
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
			}
		}
	}
	if (Gantry == FRONT_GANTRY)
	{
		gLedOn(CAM1, 0, 0, 0);
		gLedOn(CAM2, 0, 0, 0);
		gLedOn(CAM3, 0, 0, 0);
		gLedOn(CAM4, 0, 0, 0);
		gLedOn(CAM5, 0, 0, 0);
		gLedOn(CAM6, 0, 0, 0);

		gLaserOff(CAM1);
		gLaserOff(CAM2);
		gLaserOff(CAM3);
		gLaserOff(CAM4);
		gLaserOff(CAM5);
		gLaserOff(CAM6);
	}
	else
	{
		gLedOn(RCAM1, 0, 0, 0);
		gLedOn(RCAM2, 0, 0, 0);
		gLedOn(RCAM3, 0, 0, 0);
		gLedOn(RCAM4, 0, 0, 0);
		gLedOn(RCAM5, 0, 0, 0);
		gLedOn(RCAM6, 0, 0, 0);


		gLaserOff(RCAM1);
		gLaserOff(RCAM2);
		gLaserOff(RCAM3);
		gLaserOff(RCAM4);
		gLaserOff(RCAM5);
		gLaserOff(RCAM6);
	}
	return Err;
}

Ratio_XYRZ CRecognitionFormingRetry::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio.xy = MinRatio.r = MinRatio.z = 1.0;
	MinRatio = gcStep->GetMinRatio();
	TRACE(_T("[PWR] CRecognitionFormingRetry(%d) MinRatioXYRZ,%.1f,%.1f,%.1f"), GetTable(), MinRatio.xy, MinRatio.r, MinRatio.z);
	return MinRatio;
}

long CRecognitionFormingRetry::GetLaserControl(long FeederNo)
{
	long LaserControl = 0;
	LaserControl = gcStep->GetLaserControl(FeederNo);
	return LaserControl;
}

long CRecognitionFormingRetry::GetCatchDelay(long FeederNo)
{
	long CatchDelay = 0;
	CatchDelay = gcStep->GetCatchDelay(FeederNo);
	return CatchDelay;
}

long CRecognitionFormingRetry::GetMaxRetryCount(long FeederNo)
{
	long MaxRetry = 0;
	MaxRetry = gcStep->GetRetryLed(FeederNo).MaxRetry;
	return MaxRetry;
}

MODULE_LED CRecognitionFormingRetry::GetMaxRetryLed(long FeederNo, long Retry)
{
	MODULE_LED Led;
	Led = gcStep->GetRetryLed(FeederNo).Led[Retry];
	return Led;
}

bool CRecognitionFormingRetry::GetFormingMotionDone(long insertOrd)
{
	bool bMotionDone = false;
	bMotionDone = gcStep->GetFormingMotionDone(insertOrd);
	return bMotionDone;
}
//
//long CRecognitionFormingRetry::GetPickupErrorUse(long FeederNo)
//{
//	long ReferenceCount = 0;
//	ReferenceCount = gcStep->GetPickupErrorUse(FeederNo);
//	return ReferenceCount;
//}
//
//void CRecognitionFormingRetry::AddPickupErrorCount(long FeederNo, long VisionError)
//{
//	gcStep->AddPickupErrorCount(FeederNo, VisionError);
//}
//
//void CRecognitionFormingRetry::RemovePickupErrorCount(long FeederNo)
//{
//	gcStep->RemovePickupErrorCount(FeederNo);
//}
//
//long CRecognitionFormingRetry::GetPickupErrorReferenceCount(long FeederNo)
//{
//	long ReferenceCount = 0;
//	ReferenceCount = gcStep->GetPickupErrorReferenceCount(FeederNo);
//	return ReferenceCount;
//}
//
//long CRecognitionFormingRetry::GetPickupErrorAlarmCount(long FeederNo)
//{
//	long ReferenceCount = 0;
//	ReferenceCount = gcStep->GetPickupErrorAlarmCount(FeederNo);
//	return ReferenceCount;
//}
//
//long CRecognitionFormingRetry::GetPickupErrorRawCount(long FeederNo)
//{
//	long RawCount = 0;
//	RawCount = gcStep->GetPickupErrorRawCount(FeederNo);
//	return RawCount;
//}
//
//long CRecognitionFormingRetry::GetPickupErrorCount(long FeederNo)
//{
//	long ErrorCount = 0;
//	ErrorCount = gcStep->GetPickupErrorCount(FeederNo);
//	return ErrorCount;
//}

DIVIDE_INSPECT CRecognitionFormingRetry::GetDivideInspect(long FeederNo)
{
	return gcStep->GetDivideInspect(FeederNo);
}