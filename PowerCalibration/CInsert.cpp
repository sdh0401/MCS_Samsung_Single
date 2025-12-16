#include "pch.h"
#include "CInsert.h"
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
#include "Trace.h"
#include "CStep.h"
#include "CAvoidMotion.h"
#include "CReadJobFile.h"
#include "CCamDropCheck.h"

CInsert* gcInsert;
CInsert::CInsert(long Gantry)
{
	m_Gantry = Gantry;
	m_MaxInsertOrder = 0;
	m_ProdRunMode = RUN_REAL;
}

CInsert::~CInsert()
{
}

void CInsert::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CInsert::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CInsert::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 0;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CInsert::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CInsert::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

Ratio_XYRZ CInsert::GetComponentRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ZeroMemory(&ratio, sizeof(ratio));
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

double CInsert::GetVAAngleFromFdNo(long FdNo)
{
	double RecognitionAngle = 0.0;
	RecognitionAngle = gcStep->GetVAAngleFromFdNo(FdNo);
	return RecognitionAngle;
}

double CInsert::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 10.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CInsert::GetComponentLeadHeight(long FdNo)
{
	double ComponentLeadHeight = 10.0;
	ComponentLeadHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentLeadHeight;
}

Point_XYRZ CInsert::GetInsertPoint(long insertNo)
{
	Point_XYRZ ptRet;
	ptRet = gcStep->GetInsertPoint(insertNo);
	return ptRet;
}

long CInsert::GetBlowDelayFromInsertOrder(long FdNo)
{
	CString str;
	long BlowDelay = TIME20MS;
	BlowDelay = gcStep->GetBlowDelayFromFdNo(FdNo);
	return BlowDelay;
}

long CInsert::GetReleaseDelayFromInsertOrder(long FdNo)
{
	CString str;
	long ReleaseDelay = TIME20MS;
	ReleaseDelay = gcStep->GetReleaseDelayFromFdNo(FdNo);
	return ReleaseDelay;
}

ORIGIN CInsert::GetOrigin()
{
	ORIGIN Origin;
	Origin = gcStep->GetOrigin();
	return Origin;
}

long CInsert::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
	return InsertNo;
}

double CInsert::GetPickupZStandBy()
{
	double PickupZStandBy = 0.0;
	PickupZStandBy = gcStep->GetPickupZStandBy();
	return PickupZStandBy;
}

double CInsert::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	return MaxComponentHeight;
}

long CInsert::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmpty FeederNo:%d ReadyTimeOutEmpty:%d\n"), FeederNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CInsert::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmptyByHeadNo HeadNo:%d ReadyTimeOutEmpty:%d\n"), HeadNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CInsert::GetRecognitionTable(long insertNo)
{
	long RecognitionTable = FRONT_GANTRY;
	RecognitionTable = gcStep->GetRecognitionTable(insertNo);
	return RecognitionTable;
}

TwoStepMotion CInsert::GetTwoStepInsert(long FeederNo)
{
	TwoStepMotion TwoStepInsert;
	TwoStepInsert = gcStep->GetTwoStepInsert(FeederNo);
	return TwoStepInsert;
}

TwoStepMotion CInsert::GetTwoStepInsertUp(long FeederNo)
{
	TwoStepMotion TwoStepInsert;
	TwoStepInsert = gcStep->GetTwoStepInsertUp(FeederNo);
	return TwoStepInsert;
}

double CInsert::GetInsertZOffset(long FeederNo)
{
	double InsertZOffset;
	InsertZOffset = gcStep->GetInsertZOffset(FeederNo);
	return InsertZOffset;
}

long CInsert::GetBoardNo()
{
	long BoardNo = 0;
	BoardNo = gcStep->GetBoardNo();
	return BoardNo;
}

long CInsert::GetAvoidCount(long insertNo)
{
	long AvoidCount = NO_ERR;
	AvoidCount = gcStep->GetAvoidCount(insertNo);
	return AvoidCount;
}

long CInsert::WaitGantry(long Gantry, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitGantryIdle(Gantry, TimeOut);
	return Err;
}

long CInsert::MoveZUpBeforeInsert(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = StartPosWaitInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CInsert::WaitZUpBeforeInsert(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = WaitOneInPos(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CInsert::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CInsert::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CInsert::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CInsert::MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CInsert::WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut, long UseSendAlarm)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut, UseSendAlarm);
	InitOneRatio(strAxis);
	return Err;
}

long CInsert::MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
	long Err = NO_ERR;
	Err = StartPosWaitInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CInsert::Run(const bool& manualMode)
{
	long Gantry = m_Gantry;
	CString strFileName;
	Point_XY ptXY, CurPt;
	Point_XYR ptXYR;
	Point_XYRZ ptXYRZ;
	CString strZAxis, strRAxis;
	Ratio_XYRZ CompRatio, MinRatio, MoveRatio;
	Point_XYRE res, CamRes;
	Point_XY Goal, OriginXY;
	ORIGIN Origin;
	NOZZLE Nozzle;
	SomeTarget TargetGroup;
	ULONGLONG GetTime = 0, Elapsed = 0;
	ULONGLONG TorqueMonitorGetTime = 0, TorqueMonitorElapsed = 0;
	ULONGLONG ReleaseGetTime = 0, ReleaseElapsed = 0;
	Limit limit;
	double TorqueLimit = 0.0, Position = 0.0;
	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	double RatioZ = 1.0, BodyHeight = 5.0, LeadHeight = 5.0;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Ret = 0, Err = NO_ERR, Err2 = NO_ERR;
	long Target, ChkPos = 0, HeadNo = 0, JobFdNo = 0, Block = 0, Point = 0, NozzleType = 0, index = 0;
	long BlowDelay = 0, ReleaseDelay = 0, NozzleNo = 0;
	long CamNo = 0, CamChk = 0, InsertNo = 0, RecogTable = 0, AvoidCount = 0, ReleaseLevel = 0;
	double InsertZ = GetInsertByZ(FRONT_GANTRY), InsertZUp = GetStandByZ(FRONT_GANTRY);
	double RecognitionZUp = GetStandByZ(FRONT_GANTRY), MaxComponentHeight = 10.0;
	double Dist = 0.0, InsertZOffset = 0.0;
	TwoStepMotion TwoStepInsert;
	TwoStepMotion TwoStepInsertUp;
	PCB Pcb = gcReadJobFile->GetPcb();
	bool bFirstMoveXY = true;
	double ZUpRatio = 1.0;
	bool SendTorqueFileName = false;
	ZeroMemory(&limit, sizeof(limit));
	ZeroMemory(&CamRes, sizeof(CamRes));
	ZeroMemory(&CurPt, sizeof(CurPt));
	ZeroMemory(&TwoStepInsert, sizeof(TwoStepInsert));

	GetTime = _time_get();
	Err = WaitGantry(FRONT_GANTRY, TIME5000MS);
	Elapsed = _time_elapsed(GetTime);
	if (gcPowerLog->IsShowRunLog() == true|| Elapsed > 0)
	{
		TRACE(_T("[PWR] Insert WaitGantry Elasped,%d"), _time_elapsed(GetTime));
	}
	Origin = GetOrigin();
	OriginXY.x = Origin.pt.x;
	OriginXY.y = Origin.pt.y;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] Insert Run ProdRunMode:%d\n"), GetProdRunMode());
	}
	if (gcAdvancedMotionFile->UseAdvanceMotion() == 1)
	{
		MsXy = gcAdvancedMotionFile->GetInsertDelayedMsXY();
		MsZDn = gcAdvancedMotionFile->GetInsertDelayedMsZDn();
		MsZUp = gcAdvancedMotionFile->GetInsertDelayedMsZUp();
		MsR = gcAdvancedMotionFile->GetInsertDelayedMsR();
		InposXY = gcAdvancedMotionFile->GetInsertInposXY();
		InposR = gcAdvancedMotionFile->GetInsertInposR();
		InposZDn = gcAdvancedMotionFile->GetInsertInposZDn();
		InposZUp = gcAdvancedMotionFile->GetInsertInposZUp();
		TRACE(_T("[PWR] Insert Ms XY,%d R,%d ZDn,%d ZUp,%d Inpos XY,%.3f R,%.3f ZDn,%.3f ZUp,%.3f\n"),
			MsXy, MsR, MsZDn, MsZUp, InposXY, InposR, InposZDn, InposZUp);
	}
	MoveRatio.xy = MoveRatio.r = MoveRatio.z = 1.0;
	MinRatio = GetMinRatio();
	GetTime = _time_get();
	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
		JobFdNo = GetFdNoFromInsertOrder(InsertOrd + 1);
		BodyHeight = GetComponentHeight(JobFdNo);
		LeadHeight = GetComponentLeadHeight(JobFdNo);
		CompRatio = GetComponentRatioByFdNo(JobFdNo);
		MaxComponentHeight = GetMaxComponentHeight();
		NozzleNo = GetGlobalNozzleNo(HeadNo);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		RecognitionZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
		limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (RecognitionZUp > SAFTY_ZHEIGHT)
		{
			RecognitionZUp = GetStandByZ(m_Gantry);
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
				RecognitionZUp, MaxComponentHeight, Nozzle.TipHeight, BodyHeight, LeadHeight);
		}
		Err = MoveZUpBeforeInsert(strZAxis, CompRatio.z, TimeOut, RecognitionZUp, InposZUp, MsZUp, false);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Insert MoveZUpBeforeInsert Err"));
			TRACE(_T("[PWR] Insert MoveZUpBeforeInsert(%s) Err:%d\n"), strZAxis, Err);
			return Err;
		}
	}
	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
		JobFdNo = GetFdNoFromInsertOrder(InsertOrd + 1);
		BodyHeight = GetComponentHeight(JobFdNo);
		LeadHeight = GetComponentLeadHeight(JobFdNo);
		CompRatio = GetComponentRatioByFdNo(JobFdNo);
		MaxComponentHeight = GetMaxComponentHeight();
		NozzleNo = GetGlobalNozzleNo(HeadNo);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		RecognitionZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
		limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (RecognitionZUp > SAFTY_ZHEIGHT)
		{
			RecognitionZUp = GetStandByZ(m_Gantry);
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
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		TRACE(_T("[PWR] Insert Z Up Before Insert Elasped,%d\n"), _time_elapsed(GetTime));
	}
	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] Insert GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] Insert GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
		JobFdNo = GetFdNoFromInsertOrder(InsertOrd + 1);
		BodyHeight = GetComponentHeight(JobFdNo);
		LeadHeight = GetComponentLeadHeight(JobFdNo);
		Point = GetInsertNoFromInsertOrder(InsertOrd + 1);
        Block = gcReadJobFile->GetInsert(Gantry, Point).BlockNo;
		index = gcReadJobFile->GetInsert(Gantry, Point).index;
        if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
        {
            const Point_XYR BlockOrigin = GetOriginXYRFromJobfile(Block);
            OriginXY = Point_XY{ BlockOrigin.x, BlockOrigin.y };
        }
		ptXYRZ = GetInsertPoint(Point);
		strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
		BlowDelay = GetBlowDelayFromInsertOrder(JobFdNo);
		ReleaseDelay = GetReleaseDelayFromInsertOrder(JobFdNo);
		NozzleNo = GetGlobalNozzleNo(HeadNo);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		InsertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);
		RecogTable = GetRecognitionTable(InsertNo);
		CamNo = GetCameraNoByHead(RecogTable, HeadNo);
		CamChk = GetCameraChkPosByHead(HeadNo);
		TwoStepInsert = GetTwoStepInsert(JobFdNo);
		TwoStepInsertUp = GetTwoStepInsertUp(JobFdNo);
		InsertZOffset = GetInsertZOffset(JobFdNo);
		MaxComponentHeight = GetMaxComponentHeight();
		ptXY.x = ptXYRZ.x;
		ptXY.y = ptXYRZ.y;
		CompRatio = GetComponentRatioByFdNo(JobFdNo);
		res = gGetRunVisionResult(Gantry, InsertOrd);
		CamRes = gGetPartRecognitionResult(CamNo, CamChk);
		if (GetProdRunMode() == RUN_REAL)
		{
			if (GetReadyTimeOutEmptyByHeadNo(HeadNo) == 1)
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] InsertNo:%03d Hd:%02d Fd:%03d NG ReadyIO TimeOut(%d)\n"), Point, HeadNo, JobFdNo, GetReadyTimeOutEmptyByHeadNo(HeadNo));
				TRACE(_T("[PWR] ************************************************************\n"));
				continue;
			}
			else if (res.exe != 1)
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] InsertNo:%03d Hd:%02d Fd:%03d NG ErrorCode:%d\n"), Point, HeadNo, JobFdNo, gGetRunVisionErrorCode(Gantry, InsertOrd));
				TRACE(_T("[PWR] ************************************************************\n"));
				continue;
			}
			else
			{
				TRACE(_T("[PWR] ************************************************************\n"));
				TRACE(_T("[PWR] InsertNo:%03d Hd:%02d Fd:%03d OK(1) ResultXYR:%.3f,%.3f,%.3f\n"), Point, HeadNo, JobFdNo, res.x, res.y, res.r);
				TRACE(_T("[PWR] InsertNo:%03d Hd:%02d Fd:%03d OK(2) ResultXYR:%.3f,%.3f,%.3f\n"), Point, HeadNo, JobFdNo, CamRes.x, CamRes.y, CamRes.r);
				TRACE(_T("[PWR] ************************************************************\n"));
			}
		}
		MoveRatio = CompRatio;
		TRACE(_T("[PWR] Insert InsertOrd:%d Original XY Spd,%03d%%\n"), InsertOrd, (long)(MoveRatio.xy * 100));
		for (long InsertSpd = InsertOrd + 1; InsertSpd < GetMaxInsertOrder(); InsertSpd++)
		{
			TRACE(_T("[PWR] Insert InsertOrd:%d XY Spd,%03d%% NextOrd:%d XY Spd,%03d%%\n"), InsertOrd, (long)(MoveRatio.xy * 100), InsertSpd, (long)(GetInsertRatio(InsertSpd).xy * 100));
			if (GetInsertRatio(InsertSpd).xy < MoveRatio.xy)
			{
				MoveRatio.xy = GetInsertRatio(InsertSpd).xy;
				TRACE(_T("[PWR] Insert InsertOrd:%d New XY Spd,%03d%%\n"), InsertSpd, (long)(MoveRatio.xy * 100));
				if (MoveRatio.xy < MinRatio.xy)
				{
					MinRatio = MoveRatio;
				}
			}
		}

		double torqueEventZup = ReadCommandPosition(strZAxis);

		AvoidCount = GetAvoidCount(InsertNo);
		TRACE(_T("[PWR] InsertNo(%03d) AvoidCount:%d\n"), InsertNo, AvoidCount);
		if (AvoidCount > 0)
		{
			CAvoidMotion* Avoid = new CAvoidMotion(Gantry);
			Err = Avoid->Run(Point, 0, 0);
			delete Avoid;
		}
		Goal = ReadInsertFromOrigin(ptXY, OriginXY);
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] Insert Ord(%d)(%s,%s) X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), InsertOrd, strZAxis, strRAxis, ptXYRZ.x, ptXYRZ.y, ptXYRZ.r, ptXYRZ.z);
		}
		ptXYRZ.x = Goal.x;
		ptXYRZ.y = Goal.y;
        ptXYRZ.r = calculateBlockRotationOffset(ptXYRZ.r, Block);
		ChkPos = GetCameraChkPosByHead(HeadNo);
        if (Pcb.UseBlockType == PCB_SINGLE)
        {
            ptXYRZ = gComponentCompensation(Gantry, ChkPos, ptXYRZ, InsertOrd, MK_PWB);		
        }
        else if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
        {
            ptXYRZ = gComponentCompensation(Gantry, ChkPos, ptXYRZ, InsertOrd, gcReadJobFile->GetInsert(Gantry, Point).BlockNo);
        }
		ptXYR.x = ptXYRZ.x;
		ptXYR.y = ptXYRZ.y;
		ptXYR.r = ptXYRZ.r;

        if (GetProdRunMode() == RUN_DRY || Pcb.UseFiducial == FIDUCIAL_NOUSE)
        {

        }
        else
        {
            //절댓값 360안쪽으로 조정 후 피듀셜 마크 보정하니까 가끔 360 넘는 문제점 발생해서 위로 올림.
            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
            {
                ptXYR = gMarkCompensation(Gantry, ptXYR, MK_PWB);
            }
            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
            {
                ptXYR = gMarkCompensation(Gantry, ptXYR, Block);
            }
        }

        if ((int)(ptXYR.r * 100) < -36000 || +36000 < (int)(ptXYR.r * 100))
        //2차 수정. 소수점 계산이 정확하지 않으니까 (0.1이 사실은 0.1이 아닌 문제) 소수점 아래 2자리 까지 사용한다는 점을 토대로 정수로 잠시 바꿔서 쓰고난 뒤 다시 원래 자릿수로 돌려주기.
        //-180.0 이상 ~ +180.0 이하의 값으로..
        //3차 수정. 회피장착사용시 Z축 내린 상태에서 돌면서 충돌 가능성 발생. -> -360 이상 ~ +360 이하의 조건으로 수정.
        {
            const double oldRPosition = ptXYR.r;
            
            int rPositionInInteger = (int)(oldRPosition * 100);
            
            if (rPositionInInteger < -36000)
            {
                while (true)
                {
                    if (-36000 <= rPositionInInteger && rPositionInInteger <= +36000)
                    {
                        break;
                    }
                    rPositionInInteger += 36000;
                    continue;
                }
            }
            else if (+36000 < rPositionInInteger)
            {
                while (true)
                {
                    if (-36000 <= rPositionInInteger && rPositionInInteger <= +36000)
                    {
                        break;
                    }
                    rPositionInInteger -= 36000;
                    continue;
                }
            }
            const double newRPosition = (double)(rPositionInInteger / 100.0);
            ptXYR.r = newRPosition;
            
            CString temp;
            temp.Format(L"Move R Position adjusted from '%lf' to '%lf'", oldRPosition, newRPosition);
            TRACE("[PWR] %s::%s -> %s", typeid(CInsert).name(), __func__, (CStringA)temp);
        }

		Goal.x = ptXYR.x;
		Goal.y = ptXYR.y;
		Target = HeadNo;

		if (GetProdRunMode() == RUN_DRY)
		{
			Goal = ReadInsertFromOrigin(ptXY, OriginXY);
		}

		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			CurPt = gReadGantryPosition(Gantry);
		}

		if (IsAccTest() == true)
		{

		}
		else if (Target >= TBL_HEAD1 && Target <= TBL_HEAD6)
		{
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] Insert Move(%s) Angle:%.3f\n"), strRAxis, ptXYR.r);
			}
			Err = MoveR(strRAxis, CompRatio.r, TimeOut, ptXYR.r, InposR, MsR, false);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Insert MoveR Err"));
				TRACE(_T("[PWR] Insert MoveR(%s) Err,%d\n"), strRAxis, Err);
				return Err;
			}
		}

		if (bFirstMoveXY == true && gcStep->GetRecogTableBy1stInsertOrder() == REAR_STAGE)
		{
			SetPartDrop(true);
			gPartDropLedOn(REAR_STAGE);
		}

		Point_XY insertOffsetXY = gcPowerCalibrationData->GetInsertOffset4532(RecogTable, HeadNo, gDegreeToIndex(ptXYRZ.r));
		TRACE(_T("[PWR] InsertOffset4532 %s %.3f %.3f\n"), strZAxis, insertOffsetXY.x, insertOffsetXY.y);

		Goal.x = Goal.x + insertOffsetXY.x;
		Goal.y = Goal.y + insertOffsetXY.y;

		TRACE(_T("[PWR] InsertOrder:%d RatioXYRZ,%.1f,%.1f,%.1f\n"), InsertOrd, MoveRatio.xy, MoveRatio.r, MoveRatio.z);
		GetTime = _time_get();
		Err = MoveXY(Gantry, Target, Goal, MoveRatio.xy, InposXY, MsXy, TimeOut);
		if (gcPowerLog->IsShowElapsedLog() == true)
		{
			TRACE(_T("[PWR] InsertOrder:%d Dist (%.3f,%.3f) MoveXY Elapsed,%d\n"), InsertOrd, Goal.x - CurPt.x, Goal.y - CurPt.y, _time_elapsed(GetTime));
		}

		if (bFirstMoveXY == true && gcStep->GetRecogTableBy1stInsertOrder() == REAR_STAGE)
		{
			gPartDropProcess(REAR_STAGE);
		}

		bFirstMoveXY = false;

		if (Err == NO_ERR)
		{
			if (IsAccTest() == true)
			{
				SetInsertEnd(FRONT_CONV, Gantry, Block, Point);
				SendToInsertComplete(Block, Point);
				continue;
			}

			if (Target >= TBL_HEAD1 && Target <= TBL_HEAD6)
			{
				GetTime = _time_get();
				Err = WaitR(strRAxis, ptXYR.r, InposR, TimeOut);
				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					TRACE(_T("[PWR] InsertOrder:%d WaitR Elapsed,%d\n"), InsertOrd, _time_elapsed(GetTime));
				}
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("Insert WaitR Err"));
					TRACE(_T("[PWR] Insert WaitR(%s) Err,%d\n"), strRAxis, Err);
					return Err;
				}
				InsertZ = ptXYRZ.z - BodyHeight - Nozzle.TipHeight + Nozzle.PusherHeight + InsertZOffset;
				if (GetProdRunMode() == RUN_DRY)
				{
					InsertZ -= GetDryRunZHeightOffset();
				}
				if (gcPowerLog->IsShowRunLog() == true)
				{
					TRACE(_T("[PWR] InsertOrd(%d) InsertZ(%.3f) CompoT:%.3f Tip:%.3f PusherHeight:%.3f Target:%.3f\n"),
						InsertOrd, ptXYRZ.z, BodyHeight, Nozzle.TipHeight, Nozzle.PusherHeight, InsertZ);
				}
				Dist = ReadPosition(strZAxis);
				Dist = InsertZ - Dist;

				//double torqueEventZup = GetInsertByZ(Gantry) - GetMaxComponentHeight() - Nozzle.TipHeight;
				//double torqueEventZup = ReadCommandPosition(strZAxis);
				double torqueLimit = GetMaxZTorqueLimit(Gantry, HeadNo);

				if (gcStep->GetPartTorqueLimit(JobFdNo).InsertDown.Use == true)
				{
					torqueLimit = gcStep->GetPartTorqueLimit(JobFdNo).InsertDown.TorqueLimit;
					RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo, torqueLimit, torqueEventZup);
				}


				GetTime = _time_get();
				TorqueMonitorGetTime = _time_get();
				StartMonitor(strZAxis, GetProductionQuantity() + 1, Block, index, _T("Insert"));
				TorqueMonitorElapsed = _time_elapsed(TorqueMonitorGetTime);
				SendTorqueFileName = false;
				long ErrTorque = GetZTorqueAlarmcode(Gantry, HeadNo);

				if (TwoStepInsert.Use == 1)
				{
                    //Dist : 현재위치에서 조립위치(일반적으로 PCB상면)까지의 거리, TwoStepInsert.Dist : 조립위치에서 느리게 이동하기 시작하는 위치까지의 거리.
                    const bool startFast = TwoStepInsert.Dist < Dist;//느리게 이동하기 시작하는 높이보다 지금 높이가 더 크면 빠르게 하강 시작(true).
					Err = startFast ? MoveZDown(strZAxis, CompRatio.z, TimeOut, (InsertZ - TwoStepInsert.Dist), InposZDn, MsZDn, false) : Err;
					if (Err != NO_ERR)
					{
						StopMonitor(strZAxis);
						Err = SendAlarm(Err, _T("Insert ZDn1st Err"));
						TRACE(_T("[PWR] Insert(%s) ZDn1st Err,%d\n"), strZAxis, Err);
						return Err;
					}

					//gcLastPickFront->ClearHeadData(HeadNo);

					Err = startFast ? WaitZDown(strZAxis, (InsertZ - TwoStepInsert.Dist), InposZDn, TimeOut, false) : Err;
					if (Err == ErrTorque)
					{
						StopMonitor(strZAxis);
						TRACE(_T("[PWR] Insert(%s) ZDn1st TorqueOver Err,%d\n"), strZAxis, Err);
					}
					else if (Err != NO_ERR)
					{
						StopMonitor(strZAxis);
						Err = SendAlarm(Err, _T("Insert ZDn2nd Err"));
						TRACE(_T("[PWR] Insert(%s) ZDn2nd Err,%d\n"), strZAxis, Err);
						return Err;
					}
					else
					{
						if (gcStep->GetPartTorqueLimit(JobFdNo).InsertDown2nd.Use == true)
						{
							torqueLimit = gcStep->GetPartTorqueLimit(JobFdNo).InsertDown2nd.TorqueLimit;
							RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo, torqueLimit, torqueEventZup);
						}

						Err = MoveZDown(strZAxis, TwoStepInsert.Ratio, TimeOut, InsertZ, InposZDn, MsZDn, false);
						if (Err != NO_ERR)
						{
							StopMonitor(strZAxis);
							Err = SendAlarm(Err, _T("Insert MoveZDown Err"));
							TRACE(_T("[PWR] Insert MoveZDown(%s) Err,%d\n"), strZAxis, Err);
							return Err;
						}

						(manualMode == false) ? SetInsertEnd(FRONT_CONV, Gantry, Block, Point) : __noop;
						Err = WaitZDown(strZAxis, InsertZ, InposZDn, TimeOut, false);
						if (Err != NO_ERR && manualMode == false)
						{
							SetInsertClear(FRONT_CONV, Gantry, Block, Point);
						}
					}
				}
				else
				{
					Err = MoveZDown(strZAxis, CompRatio.z, TimeOut, InsertZ, InposZDn, MsZDn, false);
					if (Err != NO_ERR)
					{
						StopMonitor(strZAxis);
						Err = SendAlarm(Err, _T("Insert MoveZDown Err"));
						TRACE(_T("[PWR] Insert MoveZDown(%s) Err,%d\n"), strZAxis, Err);
						return Err;
					}

					//gcLastPickFront->ClearHeadData(HeadNo);

					(manualMode == false) ? SetInsertEnd(FRONT_CONV, Gantry, Block, Point) : __noop;
					Err = WaitZDown(strZAxis, InsertZ, InposZDn, TimeOut, false);
					if (Err != NO_ERR && manualMode == false)
					{
						SetInsertClear(FRONT_CONV, Gantry, Block, Point);
					}

				}

				TorqueMonitorGetTime = _time_get();
				StopMonitor(strZAxis);
				strFileName = GetTorqueMonitorFileName(strZAxis);
				//SendToZTorqueMonitorFile(strFileName);
				TorqueMonitorElapsed = _time_elapsed(TorqueMonitorGetTime);
				TRACE(_T("[PWR] StopMonitor(%s) Elapsed,%d\n"), strZAxis, TorqueMonitorElapsed);

				bool bSkipInsertDone = false;
				bool bTorqueAlarmStatus = false;

				if (gcStep->GetPartTorqueLimit(JobFdNo).InsertDown.Use == true || gcStep->GetPartTorqueLimit(JobFdNo).InsertDown2nd.Use == true)
				{
					RemoveSetHeadTorqueLimitEvent(Gantry, HeadNo, GetMaxZTorqueLimit(Gantry, HeadNo), GetTorqueOverSafetyZ());
				}

				bool SuctionOffSkip = false;

				if (Err != NO_ERR)
				{
					if (SendTorqueFileName == false)
					{
						SendTorqueFileName = true;
						WaitStopMonitor(strZAxis);
						SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);
					}

					if (Err == ErrTorque && manualMode == false)
					{
						ThreadSleep(TIME100MS);
						bTorqueAlarmStatus = true;
						Err = GetZTorqueInsertFailAlarmcode(Gantry, HeadNo);
						SendAlarmOnlyBuzzer(Err, _T("Insert WaitZDown Err"));
						SetInsertTorqueAlarm(false, INSERT_ALARM);

						INSERT_ALARM_ACTION RcvAction;
						while (1)
						{
							ThreadSleep(TIME10MS);

							RcvAction = GetInsertTorqueAlarm();
							if (RcvAction.Rcv == true)
							{
								TRACE(_T("[PWR] InsertTorqueAlarm %s Action:%d\n"),strZAxis, RcvAction.Action);

								if (RcvAction.Action == INSERT_DONE)
								{
									TowerLampRun();
									gcStep->SetVisionError(InsertOrd + 1, 1);
									SuctionOffSkip = true;
									Err = NO_ERR;
								}
								else if (RcvAction.Action == INSERT_RETRY)
								{
									TowerLampRun();
									gcStep->SetVisionError(InsertOrd + 1, 1);
									bSkipInsertDone = true;
									Err = NO_ERR;

								}
								else
								{
									SetGlobalStatusError(true);
									return Err;
								}
								break;
							}							

							if (GetMachineState() == STATE_STOPNOW || GetProdRunMode() == NORMAL_MODE)
							{
								TRACE(_T("[PWR] InsertTorqueAlarm GetMachineState(%d)\n"), GetMachineState());
								Err = STOP_NOW;
								return Err;
							}
						}
					}
					else
					{
						Err = SendAlarm(Err, _T("Insert WaitZDown Err"));
						TRACE(_T("[PWR] Insert WaitZDown(%s) Err,%d\n"), strZAxis, Err);
						return Err;
					}
				}

				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] InsertOrder:%d %.3f Z(%s) Dn Elapsed,%d\n"), InsertOrd, Dist, strZAxis, Elapsed);
				}

				if (bSkipInsertDone == true)
				{
					if (manualMode == false && GetInsertEnd(FRONT_CONV, Gantry, Block, Point) == INSERT_END)
					{
						SetInsertClear(FRONT_CONV, Gantry, Block, Point);
					}
				}
				else
				{
					if (manualMode == false)
					{
						if (GetInsertEnd(FRONT_CONV, Gantry, Block, Point) != INSERT_END)
						{
							SetInsertEnd(FRONT_CONV, Gantry, Block, Point);
						}

						SendToInsertComplete(Block, index);
						SendToInsertStatisticsComplete(Point, JobFdNo, NozzleNo);
						if (gcPowerLog->IsShowInsertEndLog() == true)
						{
							TRACE(_T("[PWR] Insert Gantry:%d Block:%d Point:%d GetInsertEnd(%d)\n"), Gantry, Block, Point, GetInsertEnd(FRONT_CONV, Gantry, Block, Point));
						}
					}

					if (GetProdRunMode() == RUN_REAL && SuctionOffSkip == false)
					{
						Err = SuctionOne(Gantry, HeadNo, false);
						if (Err != NO_ERR)
						{
							TRACE(_T("[PWR] Insert SuctionOne Err:%d\n"), Err);
							return Err;
						}
						if (NozzleType == 0) // Suction Type
						{
							TRACE(_T("[PWR] Insert Gantry:%d HeadNo:%d Before Blow Level:%d\n"), Gantry, HeadNo, GetAnalogLevel(Gantry, HeadNo));
							Err = BlowOne(Gantry, HeadNo, true);
							if (Err != NO_ERR)
							{
								TRACE(_T("[PWR] Insert BlowOne Err:%d\n"), Err);
								return Err;
							}
							ThreadSleep(BlowDelay);
							//Err = BlowOne(Gantry, HeadNo, false);
							//if (Err != NO_ERR)
							//{
							//	TRACE(_T("[PWR] Insert BlowOne Err:%d\n"), Err);
							//	return Err;
							//}
							TRACE(_T("[PWR] Insert Gantry:%d HeadNo:%d After  Blow Level:%d\n"), Gantry, HeadNo, GetAnalogLevel(Gantry, HeadNo));
							ReleaseGetTime = _time_get();
							while (true)
							{
								ReleaseElapsed = _time_elapsed(ReleaseGetTime);
								if (ReleaseElapsed > ReleaseDelay)
								{
									TRACE(_T("[PWR] Insert Gantry:%d HeadNo:%d After Release Level:%d\n"), Gantry, HeadNo, GetAnalogLevel(Gantry, HeadNo));
									break;
								}
								ThreadSleep(TIME1MS);
								ReleaseLevel = GetAnalogLevel(Gantry, HeadNo);
								if (gcPowerLog->IsShowAnalogLog() == true)
								{
									TRACE(_T("[PWR] Insert Gantry:%d HeadNo:%d Elapsed:%d Release Level:%d\n"), Gantry, HeadNo, ReleaseElapsed, ReleaseLevel);
								}
							}
						}
						else
						{
							Err = BlowOne(Gantry, HeadNo, true);
							if (Err != NO_ERR)
							{
								TRACE(_T("[PWR] Insert BlowOne Err:%d\n"), Err);
								return Err;
							}
							ThreadSleep(ReleaseDelay);
						}
					}
					else
					{
						ThreadSleep(BlowDelay);
						ThreadSleep(ReleaseDelay);
					}
				}


				InsertZUp = GetInsertByZ(Gantry) - GetMaxComponentHeight() - Nozzle.TipHeight;
				limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
				if (InsertZUp > SAFTY_ZHEIGHT)
				{
					InsertZUp = GetStandByZ(m_Gantry);
					TRACE(_T("[PWR] StandByZ Safty,%.3f\n"), InsertZUp);
				}
				if (limit.minus > InsertZUp)
				{
					TRACE(_T("[PWR] Insert Z Up Target position(%.3f) is under minus limit(%.3f)\n"), InsertZUp, limit.minus);
					InsertZUp = limit.minus + 1.0;
				}
				if (gcPowerLog->IsShowRunLog() == true)
				{
					TRACE(_T("[PWR] InsertOrder:%d MaxCompT:%.3f TipHeight:%.3f InsertZUp:%.3f\n"), InsertOrd,
						GetMaxComponentHeight(), Nozzle.TipHeight, InsertZUp);
				}
				GetTime = _time_get();

				if (TwoStepInsertUp.Use == 1 && bTorqueAlarmStatus == false)
				{
					TRACE(_T("[PWR] TwoStepInsertUp(%s) Dist:%.3f Ratio:%.1f Target:%.3f\n"), strZAxis, TwoStepInsertUp.Dist, TwoStepInsert.Ratio, InsertZ - TwoStepInsertUp.Dist);

					Err = MoveZUp(strZAxis, TwoStepInsert.Ratio, TimeOut, (InsertZ - TwoStepInsertUp.Dist), InposZDn, MsZDn, true);
					if (Err != NO_ERR)
					{
						if (SendTorqueFileName == false)
						{
							SendTorqueFileName = true;
							WaitStopMonitor(strZAxis);
							SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);
						}

						Err = SendAlarm(Err, _T("Insert ZUp2nd Err"));
						TRACE(_T("[PWR] Insert(%s) ZUp2nd Err,%d\n"), strZAxis, Err);
						return Err;
					}
				}

				if (bTorqueAlarmStatus == false)
				{
					Err = MoveZUp(strZAxis, ZUpRatio, TimeOut, InsertZUp, InposZUp, MsZUp, true);
				}

				if (SendTorqueFileName == false)
				{
					SendTorqueFileName = true;
					WaitStopMonitor(strZAxis);
					//if (GetGlobalSimulationMode() == true)
					//{
					//	strFileName = _T("D:\\PowerMotion\\TorqueZ\\121951152_Board1_Block0_Idx1_Insert_FZ1.Monitor");
					//}

					SendToZTorqueMonitorFile(strFileName, (long)torqueLimit);
				}

				if (gcPowerLog->IsShowElapsedLog() == true)
				{
					Elapsed = _time_elapsed(GetTime);
					TRACE(_T("[PWR] InsertOrder:%d Z(%s) Up Elapsed,%d\n"), InsertOrd, strZAxis, Elapsed);
				}
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("Insert MoveZUp Err"));
					TRACE(_T("[PWR] Insert MoveZUp(%s) Err:%d\n"), strZAxis, Err);
					return Err;
				}
				if (GetProdRunMode() == RUN_REAL)
				{
					if (NozzleType == 1) // Gripper Type
					{
						Err = BlowOne(Gantry, HeadNo, false);
						if (Err != NO_ERR)
						{
							TRACE(_T("[PWR] Insert BlowOne Err:%d\n"), Err);
							return Err;
						}
					}
					else
					{
						Err = BlowOne(Gantry, HeadNo, false);
						if (Err != NO_ERR)
						{
							TRACE(_T("[PWR] Insert BlowOne Err:%d\n"), Err);
							return Err;
						}
					}
				}
			}
		}
		else
		{
			Err = SendAlarm(Err, _T("Insert MoveXY Err"));
			TRACE(_T("[PWR] Insert MoveXY Err:%d\n"), Err);
			Err2 = WaitR(strRAxis, ptXYR.r, InposR, TimeOut);
			if (Err2 != NO_ERR)
			{
				Err = SendAlarm(Err2, _T("Insert WaitR Err"));
				TRACE(_T("[PWR] Insert WaitR Err:%d\n"), Err2);
			}
			return Err;
		}
	}

	if (gcStep->GetRecogTableBy1stInsertOrder() == REAR_STAGE)
	{
		Err = gPartDropGetResult(REAR_STAGE);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Drop object on the camera"));
			return Err;
		}
	}

	return Err;
}

Ratio_XYRZ CInsert::GetMinRatio()
{
	Ratio_XYRZ MinRatio;
	MinRatio = gcStep->GetMinRatio();
	return MinRatio;
}

Ratio_XYRZ CInsert::GetInsertRatio(long InsertOrder)
{
	Ratio_XYRZ InsertRatio;
	InsertRatio = gcStep->GetInsertRatio(InsertOrder);
	return InsertRatio;
}

//void CInsert::PartDropLedOn(long CamTable)
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
//}
//
//void CInsert::PartDropProcess(long CamTable)
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
//long CInsert::PartDropGetResult(long CamTable)
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

void CInsert::SetPartDrop(bool set)
{
	m_ExePartDrop = set;

}

bool CInsert::GetPartDrop()
{
	return m_ExePartDrop;
}
