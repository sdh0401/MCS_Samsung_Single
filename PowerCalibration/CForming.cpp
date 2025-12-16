#include "pch.h"
#include "CForming.h"
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
#include "Trace.h"
#include "CStep.h"
#include "CApplicationTime.h"
#include "CReadJobFile.h"
#include "CAdvancedMotionFile.h"
#include "CTokenizer.h"

CForming* gcForming;
CForming::CForming(long Gantry)
{
	m_Gantry = Gantry;
}

CForming::~CForming()
{
}

long CForming::GetTable()
{
	return m_Gantry;
}

long CForming::MoveStandBy()
{
	gMainMoveStandBy();
	return NO_ERR;
}

ORIGIN CForming::GetOrigin()
{
	ORIGIN Origin;
	Origin = gcStep->GetOrigin();
	return Origin;
}

long CForming::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CForming::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CForming::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CForming::MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CForming::WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

long CForming::MoveZUp(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CForming::WaitZUp(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

void CForming::SetRatio(Ratio_XYRZ ratio)
{
}

void CForming::SetDelay(long delay)
{
}

long CForming::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CForming::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CForming::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 0;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

double CForming::GetComponentHeight(long FdNo)
{
	double ComponentHeight = 10.0;
	ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
	return ComponentHeight;
}

double CForming::GetComponentLeadHeight(long FdNo)
{
	double ComponentLeadHeight = 10.0;
	ComponentLeadHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
	return ComponentLeadHeight;
}

double CForming::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	return MaxComponentHeight;
}

Ratio_XYRZ CForming::GetComponentRatioByFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ZeroMemory(&ratio, sizeof(ratio));
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

FORMING_COMPONENT CForming::GetFormingComponentFromFeederNo(long FdNo)
{
	FORMING_COMPONENT Forming;
	ZeroMemory(&Forming, sizeof(Forming));
	Forming = gcStep->GetForming(FdNo);
	return Forming;
}

void CForming::SetFormingMotionDone(long insertOrd, bool bDone)
{
	gcStep->SetFormingMotionDone(insertOrd, bDone);
}

long CForming::Run(long RunMode, FORMING_COMPONENT Forming)
{
	long Gantry = m_Gantry, HeadNo = TBL_CAMERA, FeederNo = 0, PackageNo = 0, PackageIndex = 0, NozzleNo = 0;
	Point_XY goalXY;
	Ratio_XYRZ Ratio;
	PCB Pcb = gcReadJobFile->GetPcb();
	double MaxComponentHeight = Pcb.MaxComponentHeight;
	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050, ZUpHeight = 0.0, ZDnHeight = 0.0;
	double RatioZ = 1.0, BodyHeight = 5.0, LeadHeight = 5.0, Angle = 0.0;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Ret = 0, Err = NO_ERR, Err2 = NO_ERR;
	ULONGLONG GetTime = 0, Elapsed = 0;
	CString strRAxis, strZAxis, PackageName, strAxis;
	NOZZLE Nozzle;
	Limit limit;
	FORMING_COMPONENT FormingRun;
	GetTime = _time_get();
	Err = WaitGantryIdle(Gantry, TIME5000MS);
	TRACE(_T("[PWR] Forming Run WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Forming Run WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}
	HeadNo = Forming.HeadNo;
	FeederNo = Forming.FeederNo;
	NozzleNo = GetGlobalNozzleNo(HeadNo);
	Nozzle = GetGlobalNozzleInformation(NozzleNo);	
	strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
	Ratio = Forming.Ratio;;
	BodyHeight = Forming.BodyHeight;
	LeadHeight = Forming.LeadHeight;
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
		TRACE(_T("[PWR] CForming Ms XY,%d R,%d ZDn,%d ZUp,%d Inpos XY,%.3f R,%.3f ZDn,%.3f ZUp,%.3f\n"), MsXy, MsR, MsZDn, MsZUp, InposXY, InposR, InposZDn, InposZUp);
	}

	if (RunMode == 0) // Manual Mode
	{
		for (long indx = 0; indx < GetZAxisCount(); ++indx)
		{
			strAxis = GetZAxisByIndex(indx);
			NozzleNo = GetGlobalNozzleNo(indx);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			if (strZAxis.CompareNoCase(strAxis) == 0)
			{
				ZUpHeight = GetStandByZ(Gantry) - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
			}
			else
			{
				ZUpHeight = GetStandByZ(Gantry) - Nozzle.TipHeight + Nozzle.PusherHeight;
			}
			limit = GetLimit(GetAxisIndexFromAliasName(strAxis));

			if (limit.minus > ZUpHeight)
			{
				TRACE(_T("[PWR] Forming ZUpHeight position(%.3f) is under minus limit(%.3f)\n"), ZUpHeight, limit.minus);
				ZUpHeight = limit.minus + 1.0;
			}

			Err = MoveZUp(strAxis, Ratio.z, TimeOut, ZUpHeight, InposZUp, MsZUp, false);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Forming MoveZUp(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
		for (long indx = 0; indx < GetZAxisCount(); ++indx)
		{
			strAxis = GetZAxisByIndex(indx);
			NozzleNo = GetGlobalNozzleNo(indx);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			if (strZAxis.CompareNoCase(strAxis) == 0)
			{
				ZUpHeight = GetStandByZ(Gantry) - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
			}
			else
			{
				ZUpHeight = GetStandByZ(Gantry) - Nozzle.TipHeight + Nozzle.PusherHeight;
			}
			limit = GetLimit(GetAxisIndexFromAliasName(strAxis));

			if (limit.minus > ZUpHeight)
			{
				TRACE(_T("[PWR] Forming ZUpHeight position(%.3f) is under minus limit(%.3f)\n"), ZUpHeight, limit.minus);
				ZUpHeight = limit.minus + 1.0;
			}

			Err = WaitZUp(strAxis, ZUpHeight, InposZUp, TimeOut);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Forming WaitZUp(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
	}
	else // Run Mode
	{
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
			FeederNo = GetFdNoFromInsertOrder(InsertOrd + 1);
			BodyHeight = GetComponentHeight(FeederNo);
			LeadHeight = GetComponentLeadHeight(FeederNo);
			Ratio = GetComponentRatioByFdNo(FeederNo);
			MaxComponentHeight = GetMaxComponentHeight();
			NozzleNo = GetGlobalNozzleNo(HeadNo);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			ZUpHeight = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
			strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			if (ZUpHeight > SAFTY_ZHEIGHT)
			{
				ZUpHeight = GetStandByZ(m_Gantry);
				TRACE(_T("[PWR] Forming ZUpHeight Safty,%.3f\n"), ZUpHeight);
			}
			if (limit.minus > ZUpHeight)
			{
				TRACE(_T("[PWR] Forming ZUpHeight position(%.3f) is under minus limit(%.3f)\n"), ZUpHeight, limit.minus);
				ZUpHeight = limit.minus + 1.0;
			}
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] Forming ZUpHeight:%.3f MaxCompoT:%.3f TipHeight:%.3f Body:%.3f Lead:%.3f\n"), ZUpHeight, MaxComponentHeight, Nozzle.TipHeight, BodyHeight, LeadHeight);
			}
			Err = MoveZUp(strZAxis, Ratio.z, TimeOut, ZUpHeight, InposZUp, MsZUp, false);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Forming MoveZUp Err"));
				TRACE(_T("[PWR] Forming MoveZUp(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
			FeederNo = GetFdNoFromInsertOrder(InsertOrd + 1);
			BodyHeight = GetComponentHeight(FeederNo);
			LeadHeight = GetComponentLeadHeight(FeederNo);
			Ratio = GetComponentRatioByFdNo(FeederNo);
			MaxComponentHeight = GetMaxComponentHeight();
			NozzleNo = GetGlobalNozzleNo(HeadNo);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			ZUpHeight = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
			strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			if (ZUpHeight > SAFTY_ZHEIGHT)
			{
				ZUpHeight = GetStandByZ(m_Gantry);
				TRACE(_T("[PWR] Forming TargetPos Safty,%.3f\n"), ZUpHeight);
			}
			if (limit.minus > ZUpHeight)
			{
				TRACE(_T("[PWR] Forming ZUpHeight position(%.3f) is under minus limit(%.3f)\n"), ZUpHeight, limit.minus);
				ZUpHeight = limit.minus + 1.0;
			}
			Err = WaitZUp(strZAxis, ZUpHeight, InposZUp, TimeOut);
			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("Forming WaitZUp Err"));
				TRACE(_T("[PWR] Forming WaitZUp(%s) Err:%d\n"), strZAxis, Err);
				return Err;
			}
		}
	}

	if (RunMode == 0) // Manual Mode
	{
		long InsertOrd = 0;
		NozzleNo = GetGlobalNozzleNo(HeadNo);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
		TRACE(_T("[PWR] Forming (1)Min~Max:%.3f~%.3f (2)Min~Max:%.3f~%.3f Pitch:%.3f\n"),
			Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
			Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
			Forming.VisionResultPitch);
		if (Forming.VisionResultPitch >= Forming.CaseNo[0].Min && Forming.VisionResultPitch < Forming.CaseNo[0].Max)
		{
			TRACE(_T("[PWR] FormingXYRZ(1),%.3f,%.3f,%.3f,%.3f VisionResultXYR,%.3f,%.3f,%.3f RecogAnlge:%.3f\n"),
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.VisionResult.x, Forming.VisionResult.y, Forming.VisionResult.r,
				Forming.RecognitionAngle);
			Angle = (Forming.ptNo[0].r - Forming.RecognitionAngle);
			ptRotDeg(&Forming.VisionResult.x, &Forming.VisionResult.y, Angle);
			goalXY.x = Forming.ptNo[0].x + Forming.VisionResult.x;
			goalXY.y = Forming.ptNo[0].y + Forming.VisionResult.y;
			Angle = Forming.ptNo[0].r + Forming.VisionResult.r;
			TRACE(_T("[PWR] Compen Forming(1) XY,%.3f,%.3f\n"), goalXY.x, goalXY.y);
			ZDnHeight = Forming.ptNo[0].z - Nozzle.TipHeight + Nozzle.PusherHeight/* - BodyHeight - LeadHeight*/;
			TRACE(_T("[PWR] Forming(1) ZDn Height:%.3f\n"), ZDnHeight);
		}
		else if (Forming.VisionResultPitch > Forming.CaseNo[1].Min && Forming.VisionResultPitch <= Forming.CaseNo[1].Max)
		{
			TRACE(_T("[PWR] FormingXYRZ(2),%.3f,%.3f,%.3f,%.3f VisionResultXYR,%.3f,%.3f,%.3f RecogAnlge:%.3f\n"),
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z,
				Forming.VisionResult.x, Forming.VisionResult.y, Forming.VisionResult.r,
				Forming.RecognitionAngle);
			Angle = (Forming.ptNo[1].r - Forming.RecognitionAngle);
			ptRotDeg(&Forming.VisionResult.x, &Forming.VisionResult.y, Angle);
			goalXY.x = Forming.ptNo[1].x + Forming.VisionResult.x;
			goalXY.y = Forming.ptNo[1].y + Forming.VisionResult.y;
			Angle = Forming.ptNo[1].r + Forming.VisionResult.r;
			TRACE(_T("[PWR] Compen Forming(2) XY,%.3f,%.3f\n"), goalXY.x, goalXY.y);
			ZDnHeight = Forming.ptNo[1].z - Nozzle.TipHeight + Nozzle.PusherHeight/* - BodyHeight - LeadHeight*/;
			TRACE(_T("[PWR] Forming(2) ZDn Height:%.3f\n"), ZDnHeight);
		}
		else if (Forming.VisionResultPitch >= Forming.InsertCase.Min && Forming.VisionResultPitch <= Forming.InsertCase.Max)
		{
			TRACE(_T("[PWR] Forming(No need)\n"));
			return NO_NEED_FORMING;
		}
		else
		{
			TRACE(_T("[PWR] Forming(Nothing)\n"));
			return NO_FORMING_JIG;
		}
		Err = MoveR(strRAxis, Ratio.r, TimeOut, Angle, InposR, MsR, false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d MoveR Err:%d\n"), HeadNo, Err);
			return Err;
		}
		Err = MoveXY(Gantry, HeadNo, goalXY, Ratio.xy, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d MoveXY Err:%d\n"), HeadNo, Err);
			return Err;
		}
		Err = WaitR(strRAxis, Angle, InposR, MsR);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d WaitR Err:%d\n"), HeadNo, Err);
			return Err;
		}
		limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > ZDnHeight)
		{
			TRACE(_T("[PWR] CForming ZDnHeight position,%.3f is under minus limit,%.3f\n"), ZDnHeight, limit.minus);
			ZDnHeight = limit.minus + 1.0;
		}
		Err = MoveZDown(strZAxis, Ratio.z, TimeOut, ZDnHeight, InposZDn, MsZDn, false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d MoveZDown Err:%d\n"), HeadNo, Err);
			return Err;
		}
		Err = WaitZDown(strZAxis, ZDnHeight, InposZDn, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d WaitZDown Err:%d\n"), HeadNo, Err);
			return Err;
		}
		ThreadSleep(Forming.Delay);
		Err = MoveZUp(strZAxis, Ratio.z, TimeOut, ZUpHeight, InposZUp, MsZUp, false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d MoveZUp Err:%d\n"), HeadNo, Err);
			return Err;
		}
		Err = WaitZUp(strZAxis, ZUpHeight, InposZUp, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] CForming HeadNo:%d WaitZUp Err:%d\n"), HeadNo, Err);
			return Err;
		}
		SetFormingMotionDone(InsertOrd + 1, true);
	}
	else // Run Mode
	{
		for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
		{
			SetFormingMotionDone(InsertOrd + 1, false);
			HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
			FeederNo = GetFdNoFromInsertOrder(InsertOrd + 1);
			FormingRun = GetFormingComponentFromFeederNo(FeederNo);
			FormingRun.VisionResult = gGetRunVisionResult(Gantry, InsertOrd);
			if (FormingRun.Use == 0 || FormingRun.VisionResult.exe != 1)
			{
				TRACE(_T("[PWR] Run Forming InsertOrder:%d FeederNo:%d Skip Use:%d VisionResult:%d\n"), InsertOrd + 1, FeederNo, FormingRun.Use, FormingRun.VisionResult.exe);
				continue;
			}
			TRACE(_T("[PWR] Run Forming InsertOrder:%d FeederNo:%d Use:%d VisionResult:%d\n"), InsertOrd + 1, FeederNo, FormingRun.Use, FormingRun.VisionResult.exe);
			if (GetSimulationForming() == true)
			{
				FormingRun.VisionResultPitch = 26.0;
			}
			else
			{
				FormingRun.VisionResultPitch = gGetRunVisionPitchResult(Gantry, InsertOrd).x;
			}
			TRACE(_T("[PWR] Run Forming InsertOrder:%d FeederNo:%d (1)Min~Max:%.3f~%.3f (2)Min~Max:%.3f~%.3f Pitch:%.3f\n"), InsertOrd + 1, FeederNo,
				FormingRun.CaseNo[0].Min, FormingRun.CaseNo[0].Max,
				FormingRun.CaseNo[1].Min, FormingRun.CaseNo[1].Max,
				FormingRun.VisionResultPitch);
			BodyHeight = GetComponentHeight(FeederNo);
			LeadHeight = GetComponentLeadHeight(FeederNo);
			Ratio = GetComponentRatioByFdNo(FeederNo);
			MaxComponentHeight = GetMaxComponentHeight();
			NozzleNo = GetGlobalNozzleNo(HeadNo);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			ZUpHeight = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
			strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			goalXY.x = FormingRun.ptNo[0].x;
			goalXY.y = FormingRun.ptNo[0].y;
			if (FormingRun.VisionResultPitch >= FormingRun.CaseNo[0].Min && FormingRun.VisionResultPitch < FormingRun.CaseNo[0].Max)
			{
				TRACE(_T("[PWR] Run FormingXYRZ(1),%.3f,%.3f,%.3f,%.3f VisionResultXYR,%.3f,%.3f,%.3f RecogAnlge:%.3f\n"),
					FormingRun.ptNo[0].x, FormingRun.ptNo[0].y, FormingRun.ptNo[0].r, FormingRun.ptNo[0].z,
					FormingRun.VisionResult.x, FormingRun.VisionResult.y, FormingRun.VisionResult.r,
					FormingRun.RecognitionAngle);
				Angle = (FormingRun.ptNo[0].r - FormingRun.RecognitionAngle);
				ptRotDeg(&FormingRun.VisionResult.x, &FormingRun.VisionResult.y, Angle);
				goalXY.x = FormingRun.ptNo[0].x + FormingRun.VisionResult.x;
				goalXY.y = FormingRun.ptNo[0].y + FormingRun.VisionResult.y;
				Angle = FormingRun.ptNo[0].r + FormingRun.VisionResult.r;
				TRACE(_T("[PWR] Run Compen Forming(1) XY,%.3f,%.3f\n"), goalXY.x, goalXY.y);
				ZDnHeight = FormingRun.ptNo[0].z - Nozzle.TipHeight + Nozzle.PusherHeight/* - BodyHeight - LeadHeight*/;
				TRACE(_T("[PWR] Run Forming(1) ZDn Height:%.3f\n"), ZDnHeight);
				SendToForming1Statistics(FeederNo);
			}
			else if (FormingRun.VisionResultPitch > FormingRun.CaseNo[1].Min && FormingRun.VisionResultPitch <= FormingRun.CaseNo[1].Max)
			{
				TRACE(_T("[PWR] Run FormingXYRZ(2),%.3f,%.3f,%.3f,%.3f VisionResultXYR,%.3f,%.3f,%.3f RecogAnlge:%.3f\n"),
					FormingRun.ptNo[1].x, FormingRun.ptNo[1].y, FormingRun.ptNo[1].r, FormingRun.ptNo[1].z,
					FormingRun.VisionResult.x, FormingRun.VisionResult.y, FormingRun.VisionResult.r,
					FormingRun.RecognitionAngle);
				Angle = (FormingRun.ptNo[1].r - FormingRun.RecognitionAngle);
				ptRotDeg(&FormingRun.VisionResult.x, &FormingRun.VisionResult.y, Angle);
				goalXY.x = FormingRun.ptNo[1].x + FormingRun.VisionResult.x;
				goalXY.y = FormingRun.ptNo[1].y + FormingRun.VisionResult.y;
				Angle = FormingRun.ptNo[1].r + FormingRun.VisionResult.r;
				TRACE(_T("[PWR] Run Compen Forming(2) XY,%.3f,%.3f\n"), goalXY.x, goalXY.y);
				ZDnHeight = FormingRun.ptNo[1].z - Nozzle.TipHeight + Nozzle.PusherHeight/* - BodyHeight - LeadHeight*/;
				TRACE(_T("[PWR] Run Forming(2) ZDn Height:%.3f\n"), ZDnHeight);
				SendToForming2Statistics(FeederNo);
			}
			else if (FormingRun.VisionResultPitch >= FormingRun.InsertCase.Min && FormingRun.VisionResultPitch <= FormingRun.InsertCase.Max)
			{
				//if (GetPickupErrorUse(FeederNo) > 0)
				//{
				//	AddPickupErrorCount(FeederNo, 0);
				//	if (GetPickupErrorRawCount(FeederNo) > GetPickupErrorReferenceCount(FeederNo)) // 알람 취합 수량 > 알람 참조 수량
				//	{
				//		RemovePickupErrorCount(FeederNo);
				//	}
				//}
				TRACE(_T("[PWR] Run Forming(No need) InsertOrder:%d FeederNo:%d\n"), InsertOrd + 1, FeederNo);
				continue;
			}
			else
			{
				//if (GetPickupErrorUse(FeederNo) > 0)
				//{
				//	AddPickupErrorCount(FeederNo, 0);
				//	if (GetPickupErrorRawCount(FeederNo) > GetPickupErrorReferenceCount(FeederNo)) // 알람 취합 수량 > 알람 참조 수량
				//	{
				//		RemovePickupErrorCount(FeederNo);
				//	}
				//}
				TRACE(_T("[PWR] Run Forming(Nothing) InsertOrder:%d FeederNo:%d\n"), InsertOrd + 1, FeederNo);
				continue;
			}

			Err = MoveR(strRAxis, Ratio.r, TimeOut, Angle, InposR, MsR, false);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d MoveR Err:%d\n"), HeadNo, Err);
				return Err;
			}
			Err = MoveXY(Gantry, HeadNo, goalXY, Ratio.xy, InposXY, MsXy, TimeOut);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d MoveXY Err:%d\n"), HeadNo, Err);
				return Err;
			}
			Err = WaitR(strRAxis, Angle, InposR, MsR);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d WaitR Err:%d\n"), HeadNo, Err);
				return Err;
			}
			limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
			if (limit.minus > ZDnHeight)
			{
				TRACE(_T("[PWR] CForming ZDnHeight position,%.3f is under minus limit,%.3f\n"), ZDnHeight, limit.minus);
				ZDnHeight = limit.minus + 1.0;
			}
			Err = MoveZDown(strZAxis, Ratio.z, TimeOut, ZDnHeight, InposZDn, MsZDn, false);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d MoveZDown Err:%d\n"), HeadNo, Err);
				return Err;
			}
			Err = WaitZDown(strZAxis, ZDnHeight, InposZDn, TimeOut);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d WaitZDown Err:%d\n"), HeadNo, Err);
				return Err;
			}
			ThreadSleep(FormingRun.Delay);
			Err = MoveZUp(strZAxis, Ratio.z, TimeOut, ZUpHeight, InposZUp, MsZUp, false);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d MoveZUp Err:%d\n"), HeadNo, Err);
				return Err;
			}
			Err = WaitZUp(strZAxis, ZUpHeight, InposZUp, TimeOut);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] CForming HeadNo:%d WaitZUp Err:%d\n"), HeadNo, Err);
				return Err;
			}
			SetFormingMotionDone(InsertOrd + 1, true);
		}
	}
	return Err;
}

long CForming::GetPickupErrorUse(long FeederNo)
{
	long ReferenceCount = 0;
	//ReferenceCount = gcStep->GetPickupErrorUse(FeederNo);
	return ReferenceCount;
}

void CForming::AddPickupErrorCount(long FeederNo, long VisionError)
{
	//gcStep->AddPickupErrorCount(FeederNo, VisionError);
}

void CForming::RemovePickupErrorCount(long FeederNo)
{
	//gcStep->RemovePickupErrorCount(FeederNo);
}

long CForming::GetPickupErrorReferenceCount(long FeederNo)
{
	long ReferenceCount = 0;
	//ReferenceCount = gcStep->GetPickupErrorReferenceCount(FeederNo);
	return ReferenceCount;
}

long CForming::GetPickupErrorAlarmCount(long FeederNo)
{
	long ReferenceCount = 0;
	//ReferenceCount = gcStep->GetPickupErrorAlarmCount(FeederNo);
	return ReferenceCount;
}

long CForming::GetPickupErrorRawCount(long FeederNo)
{
	long RawCount = 0;
	//RawCount = gcStep->GetPickupErrorRawCount(FeederNo);
	return RawCount;
}

long CForming::GetPickupErrorCount(long FeederNo)
{
	long ErrorCount = 0;
	//ErrorCount = gcStep->GetPickupErrorCount(FeederNo);
	return ErrorCount;
}