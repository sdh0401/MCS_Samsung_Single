#include "pch.h"
#include "CAvoidMotion.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CStep.h"
#include "CApplicationTime.h"
#include "CReadJobFile.h"
#include "CAdvancedMotionFile.h"
#include "CTokenizer.h"

CAvoidMotion* gcAvoidMotion;
CAvoidMotion::CAvoidMotion(long Gantry)
{
	m_Gantry = Gantry;
}

CAvoidMotion::~CAvoidMotion()
{
}

long CAvoidMotion::MoveStandBy()
{
	gMainMoveStandBy();
	return NO_ERR;
}

ORIGIN CAvoidMotion::GetOrigin()
{
	ORIGIN Origin;
	Origin = gcStep->GetOrigin();
	return Origin;
}

long CAvoidMotion::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}

long CAvoidMotion::MoveR(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CAvoidMotion::WaitR(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	return Err;
}

long CAvoidMotion::MoveZDown(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
	return Err;
}

long CAvoidMotion::WaitZDown(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = WaitOneDelayedPosSet(strAxis, CmdPos, Inpos, TimeOut);
	InitOneRatio(strAxis);
	return Err;
}

void CAvoidMotion::SetRatio(Ratio_XYRZ ratio)
{
}

void CAvoidMotion::SetDelay(long delay)
{
}

long CAvoidMotion::GetTable()
{
	return m_Gantry;
}

long CAvoidMotion::Run(long insertNo, long OrderNo, long ManualMove)
{
	long Gantry = GetTable(), HeadNo = TBL_CAMERA, FeederNo = 0, PackageNo = 0, PackageIndex = 0, NozzleNo = 0;
	AVOIDMOTION avoid;
	ORIGIN Origin;
	Point_XY goalXY, originXY, cadXY;
	Point_XYR compenXYR, cadXYR;
	Ratio_XYRZ Ratio;
	double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
	double RatioZ = 1.0, BodyHeight = 5.0, LeadHeight = 5.0, AvoidZ = 100.0;
	long MsXy = TIME30MS, MsR = TIME30MS, MsZDn = TIME30MS, MsZUp = TIME30MS;
	long TimeOut = TIME5000MS, Ret = 0, Err = NO_ERR, Err2 = NO_ERR;
	ULONGLONG GetTime = 0, Elapsed = 0;
	PCB Pcb = gcReadJobFile->GetPcb();
	CString strRAxis, strZAxis, PackageName;
	NOZZLE Nozzle;
	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] CAvoidMotion Run WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] CAvoidMotion Run WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}
	Origin = gcReadJobFile->GetOrigin();
	originXY.x = Origin.pt.x;
	originXY.y = Origin.pt.y;
	avoid = gcReadJobFile->GetAvoidMotion(insertNo);

    const long BlockNo = gcReadJobFile->GetInsert(this->m_Gantry, insertNo).BlockNo;
    if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
    {
        const Point_XYR BlockOrigin = GetOriginXYRFromJobfile(BlockNo);
        originXY = Point_XY{ BlockOrigin.x, BlockOrigin.y };
    }

	TRACE(_T("[PWR] CAvoidMotion insertNo(%03d) MaxCount:%d\n"), insertNo, avoid.Count);

	HeadNo = gcReadJobFile->GetInsert(insertNo).HeadNo;
	NozzleNo = GetGlobalNozzleNo(HeadNo);
	Nozzle = GetGlobalNozzleInformation(NozzleNo);
	FeederNo = gcReadJobFile->GetInsert(insertNo).FeederNo;
	strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
	PackageName = gcReadJobFile->GetFeeder(FeederNo).PackageName;
	TRACE(_T("[PWR] Fd:%d PackageName:%s Start Find\n"), FeederNo, PackageName);
	for (PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
	{
		if (PackageName.CompareNoCase(gcReadJobFile->GetPackage(PackageNo).Name) == 0)
		{
			PackageIndex = PackageNo;
			TRACE(_T("[PWR] Fd:%d PackageName:%s Found Index(%d)\n"), FeederNo, PackageName, PackageIndex);
			break;
		}
	}
	if (PackageNo == MAXPACKAGENO)
	{
		TRACE(_T("[PWR] CANNOT find PackageName:%s from feeder No(%d)\n"), PackageName, FeederNo);
		return INVALID_PACKAGENAME(FeederNo + 1);
	}
	Ratio = gcReadJobFile->GetPackage(PackageIndex).Ratio;
	BodyHeight = gcReadJobFile->GetPackage(PackageIndex).Height;
	LeadHeight = gcReadJobFile->GetPackage(PackageIndex).LeadHeight;
	TRACE(_T("[PWR] Avoid Count,%d FeederNo,%d Height Body,%.3f Lead,%.3f\n"), avoid.Count, FeederNo, BodyHeight, LeadHeight);
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
		TRACE(_T("[PWR] Avoid Ms XY,%d R,%d ZDn,%d ZUp,%d Inpos XY,%.3f R,%.3f ZDn,%.3f ZUp,%.3f\n"), MsXy, MsR, MsZDn, MsZUp, InposXY, InposR, InposZDn, InposZUp);
	}
	for (long order = 0; order < MAX_AVOID_COUNT; ++order)
	{
		if (ManualMove == 1)
		{
			if (OrderNo > 0) // Manual Avoid Move 시에 특정 Order만 동작
			{
				if (order != (OrderNo - 1))
				{
					TRACE(_T("[PWR] insertNo(%03d) Manual Order:%d Skip Use:%d\n"), insertNo, order, avoid.Use[order]);
					continue;
				}
			}
		}
		if (avoid.Use[order] == 0)
		{
			TRACE(_T("[PWR] insertNo(%03d) Order:%d Use:%d\n"), insertNo, order, avoid.Use[order]);
			continue;
		}
		cadXY.x = avoid.pt[order].x;
		cadXY.y = avoid.pt[order].y;
		cadXY = ReadInsertFromOrigin(cadXY, originXY);
		cadXYR.x = cadXY.x;
		cadXYR.y = cadXY.y;
		cadXYR.r = 0.0;
		if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, MK_PWB);
		}
        else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
        {
            compenXYR = gMarkCompensation(Gantry, cadXYR, BlockNo);
        }
		else
		{
			compenXYR = cadXYR;
		}
		goalXY.x = compenXYR.x;
		goalXY.y = compenXYR.y;
		if (ManualMove == 1)
		{
			Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio.z);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] Avoid Order:%d MoveZStandy Err:%d\n"), order, Err);
				break;
			}
		}
        avoid.pt[order].r = calculateBlockRotationOffset(avoid.pt[order].r, BlockNo);
		Err = MoveR(strRAxis, Ratio.r, TimeOut, avoid.pt[order].r, InposR, MsR, false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Avoid Order:%d MoveR Err:%d\n"), order, Err);
			break;
		}
		Err = MoveXY(Gantry, HeadNo, goalXY, Ratio.xy, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Avoid Order:%d MoveXY Err:%d\n"), order, Err);
			break;
		}
		Err = WaitR(strRAxis, avoid.pt[order].r, InposR, TimeOut);
		AvoidZ = GetInsertByZ(Gantry) - BodyHeight - LeadHeight - Nozzle.TipHeight + Nozzle.PusherHeight - avoid.Height[order];
		TRACE(_T("[PWR] Avoid,%.3f InsertZ,%.3f Height Body,%.3f Lead,%.3f Nozzle Tip,%.3f Pusher,%.3f User Height,%.3f\n"),
			AvoidZ, GetInsertByZ(Gantry), BodyHeight, LeadHeight, Nozzle.TipHeight, Nozzle.PusherHeight, avoid.Height[order]);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > AvoidZ)
		{
			TRACE(_T("[PWR] Avoid Target position,%.3f is under minus limit,%.3f\n"), AvoidZ, limit.minus);
			AvoidZ = limit.minus + 1.0;
		}
		Err = MoveZDown(strZAxis, Ratio.z, TimeOut, AvoidZ, InposZDn, MsZDn, false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Avoid Order:%d MoveZDown Err:%d\n"), order, Err);
			break;
		}
		Err = WaitZDown(strZAxis, AvoidZ, InposZDn, TimeOut);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Avoid WaitZDown Err"));
			TRACE(_T("[PWR] Avoid Order:%d WaitZDown Err:%d\n"), order, Err);
			break;
		}
	}
	return Err;
}

CString CAvoidMotion::TeachAvoidPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;

	if (GetRunMode() != NORMAL_MODE)
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}

		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, order = 0, Ret = NO_ERR, InsertNo = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.003;
		double ratio = 0.5;
		long Target = TBL_HEIGHTDEVICE, TimeOut = TIME5000MS, Ms = TIME30MS;

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Gantry = iValue[0] - 1;
		order = iValue[1] - 1;
		InsertNo = iValue[2];
		TRACE(_T("[PWR] TeachAvoidPosition Gantry:%d InsertNo:%d Order:%d\n"), Gantry, InsertNo, order + 1);
		if (order >= 0)
		{
			if (gcReadJobFile)
			{
				AVOIDMOTION Avoid = gcReadJobFile->GetAvoidMotion(InsertNo);
				Point_XY ptCurrentGantryXY;
				Point_XYR Diff, Loss;
				Point_XY ptOld, ptJobFile, ptReuslt, ptNew;

				if (Avoid.Use[order] == 0)
				{
					TRACE(_T("[PWR] InsertNo:%d Order:%d No Use\n"), InsertNo + 1, order + 1);
					SetMachineState(STATE_IDLE);
					strRetMsg.Format(_T("%d"), Err);
					return strRetMsg;
				}

				ptJobFile.x = Avoid.pt[order].x;
				ptJobFile.y = Avoid.pt[order].y;
				ptCurrentGantryXY = gReadGantryPosition(Gantry);	// Origin + CAD + 
				ptOld = ReadConfirmInsertBeforePosition();

				Diff.x = ptCurrentGantryXY.x - ptOld.x;
				Diff.y = ptCurrentGantryXY.y - ptOld.y;
				PCB Pcb = gcReadJobFile->GetPcb();
                const long BlockNo = gcReadJobFile->GetInsert(this->m_Gantry, InsertNo).BlockNo;
				if (Pcb.UseFiducial > 0)
				{
					Loss = gMarkLoss(FRONT_GANTRY, &Diff, MK_PWB);
				}
                else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                {
                    Loss = gMarkLoss(FRONT_GANTRY, &Diff, BlockNo);
                }
				else
				{
					Loss = Diff;
				}
				TRACE(_T("[PWR] Avoid Teach No(%d) LossXY,%.3f,%.3f\n"), order + 1, Loss.x, Loss.y);
				if (abs(Loss.x) > 0.003 || abs(Loss.y) > 0.003)
				{
                    ptNew.x = ptCurrentGantryXY.x + (Diff.x - Loss.x);
                    ptNew.y = ptCurrentGantryXY.y + (Diff.y - Loss.y);
                    WriteConfirmInsertBeforePosition(Gantry, ptNew);
                    TRACE(_T("[PWR] Avoid(%03d) (%d) OldJobXY,%.3f,%.3f NewJobXY,%.3f,%.3f\n"),
                          InsertNo, order + 1, ptJobFile.x, ptJobFile.y, ptNew.x, ptNew.y);

                    if (gcReadJobFile->GetBlockOrigin(BlockNo).pt.r != 0.0)
                    {
                        const Point_XY transformedLoss = transformCoordinateFromEquipmentToBlockBased(Point_XY{ Loss.x, Loss.y }, Point_XY{ 0, 0 }, gcReadJobFile->GetBlockOrigin(BlockNo).pt.r);
                        Loss.x = transformedLoss.x;
                        Loss.y = transformedLoss.y;
                    }

					ptReuslt.x = ptJobFile.x + Loss.x;
					ptReuslt.y = ptJobFile.y + Loss.y;
					strRetMsg.Format(_T("%.3f,%.3f,%.3f,%.3f"), ptReuslt.x, ptReuslt.y, Avoid.pt[order].r, Avoid.Height[order]);
				}
				else
				{
					strRetMsg.Format(_T("%.3f,%.3f,%.3f,%.3f"), ptJobFile.x, ptJobFile.y, Avoid.pt[order].r, Avoid.Height[order]);
				}
			}
		}
		else
		{
			strRetMsg.Format(_T("10000"));
		}


		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CAvoidMotion::MoveAvoidGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}

	if (GetRunMode() != NORMAL_MODE)
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		ULONGLONG GetTime = 0, Elapsed = 0;
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MoveAvoidGantry TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, order = 0, Ret = NO_ERR, InsertNo = 0;
		Ratio_XYRZ Ratio;
		Point_XY ptGoal, Cur;
		Point_XYR ptXYR;
		Point_XYRZ pt;
		long Target = TBL_CAMERA, TimeOut = TIME5000MS, Ms = TIME30MS, NozzleNo = 0, PackageNo = 0, PackageIndex = 0, FeederNo = 0;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue, strRAxis, strZAxis, PackageName;
		int iValue[10];
		double dValue[10];
		double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
		double PCBThickness = 0.0, RatioPusherZ = 0.5, AvoidZ = GetStandByZ(FRONT_GANTRY), BodyHeight = 5.0, LeadHeight = 2.0;
		NOZZLE Nozzle;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Gantry = iValue[0] - 1;
		Gantry = FRONT_GANTRY;
		order = iValue[1];
		InsertNo = iValue[2];
		Target = iValue[3];
		if (Target != TBL_CAMERA)
		{
			strRAxis = GetRAxisFromHeadNo(Gantry, Target);
			strZAxis = GetZAxisFromHeadNo(Gantry, Target);
			NozzleNo = GetGlobalNozzleNo(Target);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
		}
		TRACE(_T("[PWR] MoveAvoidGantry Gantry:%d InsertNo:%d Order:%d Target:%d\n"), Gantry, InsertNo, order, Target);
		Ratio.xy = Ratio.r = Ratio.z = 0.5;
		TRACE(_T("[PWR] MoveAvoidGantry Step1\n"));
		SetMachineState(STATE_RUN);
		TRACE(_T("[PWR] MoveAvoidGantry Step2\n"));
		SetGlobalStatusError(false);
		TRACE(_T("[PWR] MoveAvoidGantry Step3\n"));
		if (InsertNo > 0)
		{
			TRACE(_T("[PWR] MoveAvoidGantry Step4\n"));
			if (gcReadJobFile)
			{
				TRACE(_T("[PWR] MoveAvoidGantry Step5\n"));
				PCBThickness = gcReadJobFile->GetPcb().Thickness;
				if (IsExistSet(WORK1_CONV) == true)
				{
					GetTime = _time_get();
					Err = StartPosWaitMotion(GetPusherZName(FRONT_CONV), RatioPusherZ, TimeOut, GetPusherByZ(Gantry) + PCBThickness, true);
					if (Err == NO_ERR)
					{
						WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(Gantry) + PCBThickness);
						TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
					}
				}
				TRACE(_T("[PWR] MoveAvoidGantry Step6\n"));
				Err = GantryMoveStandByZ(Gantry, Ratio.z);
				//Err = StartAllZAxisWaitMotion(Gantry, GetStandByZ(Gantry), Ratio.z, TimeOut);
				TRACE(_T("[PWR] MoveAvoidGantry Step7\n"));
				if (Err == NO_ERR)
				{
					TRACE(_T("[PWR] MoveAvoidGantry Step8\n"));
					const PCB Pcb = gcReadJobFile->GetPcb();
					TRACE(_T("[PWR] MoveAvoidGantry Step9\n"));
					const ORIGIN Origin = gcReadJobFile->GetOrigin();
					TRACE(_T("[PWR] MoveAvoidGantry Step10\n"));
					AVOIDMOTION Avoid = gcReadJobFile->GetAvoidMotion(InsertNo);
					FeederNo = gcReadJobFile->GetInsert(InsertNo).FeederNo;
					PackageName = gcReadJobFile->GetFeeder(FeederNo).PackageName;
					TRACE(_T("[PWR] Fd:%d PackageName:%s Start Find\n"), FeederNo, PackageName);
					for (PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
					{
						if (PackageName.CompareNoCase(gcReadJobFile->GetPackage(PackageNo).Name) == 0)
						{
							PackageIndex = PackageNo;
							TRACE(_T("[PWR] Fd:%d PackageName:%s Found Index(%d)\n"), FeederNo, PackageName, PackageIndex);
							break;
						}
					}
					if (PackageNo == MAXPACKAGENO)
					{
						TRACE(_T("[PWR] CANNOT find PackageName:%s from feeder No(%d)\n"), PackageName, FeederNo);
						SetMachineState(STATE_IDLE);
						strRetMsg.Format(_T("%d"), Err);
						return strRetMsg;
					}
					BodyHeight = gcReadJobFile->GetPackage(PackageIndex).Height;
					LeadHeight = gcReadJobFile->GetPackage(PackageIndex).LeadHeight;
					TRACE(_T("[PWR] Avoid Count,%d FeederNo,%d Height Body,%.3f Lead,%.3f\n"), Avoid.Count, FeederNo, BodyHeight, LeadHeight);
					TRACE(_T("[PWR] MoveAvoidGantry Step11\n"));
					if (Avoid.Count == 0)
					{
						TRACE(_T("[PWR] No(%03d) Avoid Motion No Use!!!!\n"), InsertNo);
						SetMachineState(STATE_IDLE);
						strRetMsg.Format(_T("%d"), Err);
						return strRetMsg;
					}
					TRACE(_T("[PWR] MoveAvoidGantry Step12\n"));
					Point_XY OriginXY, InsertXY;
					OriginXY.x = Origin.pt.x;
					OriginXY.y = Origin.pt.y;
					InsertXY.x = Avoid.pt[order - 1].x;
					InsertXY.y = Avoid.pt[order - 1].y;
					TRACE(_T("[PWR] MoveAvoidGantry Step13\n"));
                    const long BlockNo = gcReadJobFile->GetInsert(Gantry, InsertNo).BlockNo;
                    if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
                    {
                        Point_XYR BlockOrigin = GetOriginXYRFromJobfile(BlockNo);
                        OriginXY = Point_XY{ BlockOrigin.x, BlockOrigin.y };
                    }
					if (Pcb.UseFiducial > 0)
					{
						FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
						Point_XY Mark1, Mark2;
                        if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                        {
                            Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                            Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                        }
                        else//Pcb.UseFiduaial == FIDUCIAL_BLOCK
                        {
                            Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
                            Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);
                        }
						gSetMarkLed(FiducialMark.MarkNo[0], FiducialMark.Led[0]);
						gSetMarkLed(FiducialMark.MarkNo[1], FiducialMark.Led[1]);
						TRACE(_T("[PWR] MarkRecognition WaitZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
						Ret = gMarkPairRecognition(Gantry, FiducialMark.MarkNo[0], FiducialMark.MarkNo[1], Mark1, Mark2, Ratio);
						if (Ret == NO_ERR)
						{
                            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                            {
                                Ret = gGetMarkDelta(Gantry, MK_1, MK_2, Mark1, Mark2, MK_PWB);
                            }
                            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                            {
                                Ret = gGetMarkDelta(Gantry, MK_1, MK_2, Mark1, Mark2, BlockNo);
                            }
							ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
							TRACE(_T("[PWR] InsertNo(%03d) Goal X,Y,%.3f,%.3f\n"), InsertNo, ptGoal.x, ptGoal.y);
							ptXYR.x = ptGoal.x;
							ptXYR.y = ptGoal.y;
							ptXYR.r = 0.000;
                            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                            {
                                ptXYR = gMarkCompensation(Gantry, ptXYR, MK_PWB);
                            }
                            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                            {
                                ptXYR = gMarkCompensation(Gantry, ptXYR, BlockNo);
                            }
							ptGoal.x = ptXYR.x;
							ptGoal.y = ptXYR.y;
						}
						else
						{
							Ret = RECOGNITION_MARK_FAIL;
							Ret = SendAlarm(Ret, _T("Mark Recognition Fail before Avoid Motion"));
						}
					}
					else
					{
						ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
					}
					if (Ret == NO_ERR)
					{
						if (Target != TBL_CAMERA)
						{
                            Avoid.pt[order - 1].r = calculateBlockRotationOffset(Avoid.pt[order - 1].r, BlockNo);
							Err = MoveR(strRAxis, Ratio.r, TimeOut, Avoid.pt[order - 1].r, InposR, Ms, false);
							if (Err != NO_ERR)
							{
								TRACE(_T("[PWR] Avoid Order:%d MoveR Err:%d\n"), order, Err);
							}
						}
						Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, InposXY, Ms, TimeOut);
						if (Err == NO_ERR)
						{
							Cur = gReadGantryPosition(Gantry);
							WriteConfirmInsertBeforePosition(Cur);
							if (Target == TBL_CAMERA)
							{
								if (Gantry == FRONT_GANTRY)
								{
									gLedOn(FHCAM, 10, 10, 0);
									gLiveOn(FHCAM);
								}
								else
								{
									gLedOn(RHCAM, 10, 10, 0);
									gLiveOn(RHCAM);
								}
							}
							TRACE(_T("[PWR] Avoid MoveXY InsertNo(%03d) Order(%02d) AvoidXY,%.3f,%.3f\n"),
								InsertNo, order, Avoid.pt[order - 1].x, Avoid.pt[order - 1].y);
						}
					}
					if (Target != TBL_CAMERA)
					{
						Err = WaitR(strRAxis, Avoid.pt[order - 1].r, InposR, TimeOut);
						AvoidZ = GetInsertByZ(Gantry) - BodyHeight - LeadHeight - Nozzle.TipHeight + Nozzle.PusherHeight - Avoid.Height[order - 1];
						TRACE(_T("[PWR] Avoid,%.3f InsertZ,%.3f Height Body,%.3f Lead,%.3f Nozzle Tip,%.3f Pusher,%.3f User Height,%.3f\n"),
							AvoidZ, GetInsertByZ(Gantry), BodyHeight, LeadHeight, Nozzle.TipHeight, Nozzle.PusherHeight, Avoid.Height[order - 1]);
						Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
						if (limit.minus > AvoidZ)
						{
							TRACE(_T("[PWR] Avoid Target position,%.3f is under minus limit,%.3f\n"), AvoidZ, limit.minus);
							AvoidZ = limit.minus + 1.0;
						}
						Err = MoveZDown(strZAxis, Ratio.z, TimeOut, AvoidZ, InposZDn, Ms, false);
						if (Err != NO_ERR)
						{
							TRACE(_T("[PWR] Avoid Order:%d MoveZDown Err:%d\n"), order, Err);
						}
						Err = WaitZDown(strZAxis, AvoidZ, InposZDn, TimeOut);
						if (Err != NO_ERR)
						{
							TRACE(_T("[PWR] Avoid Order:%d WaitZDown Err:%d\n"), order, Err);
						}
					}
				}
			}
			SetMachineState(STATE_IDLE);
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("10000"));
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CAvoidMotion::MoveAvoidSequenceGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}

	if (GetRunMode() != NORMAL_MODE)
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		ULONGLONG GetTime = 0, Elapsed = 0;
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MoveAvoidSequenceGantry TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Ret = NO_ERR, InsertNo = 0, OtherGantry = REAR_GANTRY;
		Ratio_XYRZ Ratio;
		Point_XY ptGoal, Cur;
		Point_XYR ptXYR;
		Point_XYRZ pt;
		long Target = TBL_CAMERA, TimeOut = TIME5000MS, Ms = TIME30MS, NozzleNo = 0, PackageNo = 0, PackageIndex = 0, FeederNo = 0;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue, strRAxis, strZAxis, PackageName;
		int iValue[10];
		double dValue[10];
		double InposXY = 0.010, InposR = 1.000, InposZDn = 0.050, InposZUp = 0.050;
		double PCBThickness = 0.0, RatioPusherZ = 0.5, AvoidZ = GetStandByZ(FRONT_GANTRY), BodyHeight = 5.0, LeadHeight = 2.0;
		bool bFirst = true;
		NOZZLE Nozzle;

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Gantry = iValue[0] - 1;
		Gantry = OtherGantry = FRONT_GANTRY;
		InsertNo = iValue[1];
		Target = iValue[2];
		if (Target != TBL_CAMERA)
		{
			strRAxis = GetRAxisFromHeadNo(Gantry, Target);
			strZAxis = GetZAxisFromHeadNo(Gantry, Target);
			NozzleNo = GetGlobalNozzleNo(Target);
			Nozzle = GetGlobalNozzleInformation(NozzleNo);
			TRACE(_T("[PWR] Avoid Z(%s) R(%s) NozzleNo:%d\n"), strRAxis, strZAxis, NozzleNo);
		}
		else
		{
			TRACE(_T("[PWR] Avoid Target is Offset Camera"));
		}
		Ratio.xy = Ratio.r = Ratio.z = 0.5;
		SetMachineState(STATE_RUN);
		SetGlobalStatusError(false);
		if (InsertNo >= 0)
		{
			if (gcReadJobFile)
			{
				PCBThickness = gcReadJobFile->GetPcb().Thickness;
				if (IsExistSet(WORK1_CONV) == true)
				{
					GetTime = _time_get();
					Err = StartPosWaitMotion(GetPusherZName(FRONT_CONV), RatioPusherZ, TimeOut, GetPusherByZ(Gantry) + PCBThickness, true);
					if (Err == NO_ERR)
					{
						WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(Gantry) + PCBThickness);
						TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
					}
				}
				if (Err == NO_ERR)
				{
					GetTime = _time_get();
					Err = GantryMoveStandByZ(Gantry, Ratio.z);
					TRACE(_T("[PWR] MoveAvoidSequenceGantry MoveGantryToSafty Err:%d Elapsed,%d\n"), Err, _time_elapsed(GetTime));
				}
				//Err = StartAllZAxisWaitMotion(Gantry, GetStandByZ(Gantry), Ratio.z, TimeOut);
				if (Err == NO_ERR)
				{
					const PCB Pcb = gcReadJobFile->GetPcb();
					const ORIGIN Origin = gcReadJobFile->GetOrigin();
					AVOIDMOTION Avoid = gcReadJobFile->GetAvoidMotion(InsertNo);
					FeederNo = gcReadJobFile->GetInsert(InsertNo).FeederNo;
					PackageName = gcReadJobFile->GetFeeder(FeederNo).PackageName;
					TRACE(_T("[PWR] Fd:%d PackageName:%s Start Find\n"), FeederNo, PackageName);
					for (PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
					{
						if (PackageName.CompareNoCase(gcReadJobFile->GetPackage(PackageNo).Name) == 0)
						{
							PackageIndex = PackageNo;
							TRACE(_T("[PWR] Fd:%d PackageName:%s Found Index(%d)\n"), FeederNo, PackageName, PackageIndex);
							break;
						}
					}
					if (PackageNo == MAXPACKAGENO)
					{
						TRACE(_T("[PWR] CANNOT find PackageName:%s from feeder No(%d)\n"), PackageName, FeederNo);
						SetMachineState(STATE_IDLE);
						strRetMsg.Format(_T("%d"), Err);
						return strRetMsg;
					}
					BodyHeight = gcReadJobFile->GetPackage(PackageIndex).Height;
					LeadHeight = gcReadJobFile->GetPackage(PackageIndex).LeadHeight;
					TRACE(_T("[PWR] Avoid Count,%d FeederNo,%d Height Body,%.3f Lead,%.3f\n"), Avoid.Count, FeederNo, BodyHeight, LeadHeight);
					if (Avoid.Count == 0)
					{
						TRACE(_T("[PWR] No(%03d) Avoid Motion No Use!!!!\n"), InsertNo);
						SetMachineState(STATE_IDLE);
						strRetMsg.Format(_T("%d"), Err);
						return strRetMsg;
					}
					Point_XY OriginXY, InsertXY;
					OriginXY.x = Origin.pt.x;
					OriginXY.y = Origin.pt.y;
                    const long BlockNo = gcReadJobFile->GetInsert(Gantry, InsertNo).BlockNo;
                    if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
                    {
                        const Point_XYR blockOriginXYR = GetOriginXYRFromJobfile(BlockNo);
                        OriginXY = Point_XY{ blockOriginXYR.x, blockOriginXYR.y };
                    }
					for (long OrderNo = 0; OrderNo < MAX_AVOID_COUNT; ++OrderNo)
					{
						if (Avoid.Use[OrderNo] == 0) continue;
						InsertXY.x = Avoid.pt[OrderNo].x;
						InsertXY.y = Avoid.pt[OrderNo].y;
						if (bFirst == true)
						{
							if (Pcb.UseFiducial > 0)
							{
								FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
								Point_XY Mark1, Mark2;
                                if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                                {
                                    Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], Point_XY{ Origin.pt.x, Origin.pt.y });
                                    Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], Point_XY{ Origin.pt.x, Origin.pt.y });
                                }
                                else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                                {
                                    Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
                                    Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);
                                }
								gSetMarkLed(FiducialMark.MarkNo[0], FiducialMark.Led[0]);
								gSetMarkLed(FiducialMark.MarkNo[1], FiducialMark.Led[1]);
								TRACE(_T("[PWR] MarkRecognition WaitZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
								Ret = gMarkPairRecognition(Gantry, FiducialMark.MarkNo[0], FiducialMark.MarkNo[1], Mark1, Mark2, Ratio);
								if (Ret == NO_ERR)
								{
                                    if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                                    {
                                        Ret = gGetMarkDelta(Gantry, MK_1, MK_2, Mark1, Mark2, MK_PWB);
                                    }
                                    else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                                    {
                                        Ret = gGetMarkDelta(Gantry, MK_1, MK_2, Mark1, Mark2, BlockNo);
                                    }
									ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
									TRACE(_T("[PWR] InsertNo(%03d) Fid Order(%02d) Goal X,Y,%.3f,%.3f\n"), InsertNo, OrderNo + 1, ptGoal.x, ptGoal.y);
									ptXYR.x = ptGoal.x;
									ptXYR.y = ptGoal.y;
									ptXYR.r = 0.000;
                                    if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                                    {
                                        ptXYR = gMarkCompensation(Gantry, ptXYR, MK_PWB);
                                    }
                                    else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                                    {
                                        ptXYR = gMarkCompensation(Gantry, ptXYR, BlockNo);
                                    }
									ptXYR = gMarkCompensation(Gantry, ptXYR, MK_PWB);
									ptGoal.x = ptXYR.x;
									ptGoal.y = ptXYR.y;
								}
								else
								{
									Ret = RECOGNITION_MARK_FAIL;
									Ret = SendAlarm(RECOGNITION_MARK_FAIL, _T("Mark Recognition Fail before Confirm measureHeight"));
								}
							}
							else
							{
								ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
								TRACE(_T("[PWR] InsertNo(%03d) NoFid Order(%02d) Goal X,Y,%.3f,%.3f\n"), InsertNo, OrderNo + 1, ptGoal.x, ptGoal.y);
							}
							bFirst = false;
						}
						else
						{
							ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
							TRACE(_T("[PWR] InsertNo(%03d) Order(%02d) Goal X,Y,%.3f,%.3f\n"), InsertNo, OrderNo + 1, ptGoal.x, ptGoal.y);
							ptXYR.x = ptGoal.x;
							ptXYR.y = ptGoal.y;
							ptXYR.r = 0.000;
							if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
							{
								ptXYR = gMarkCompensation(Gantry, ptXYR, MK_PWB);
							}
                            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                            {
                                ptXYR = gMarkCompensation(Gantry, ptXYR, BlockNo);
                            }
							ptGoal.x = ptXYR.x;
							ptGoal.y = ptXYR.y;
						}
						if (Ret == NO_ERR)
						{
							if (Target != TBL_CAMERA)
							{
                                Avoid.pt[OrderNo].r = calculateBlockRotationOffset(Avoid.pt[OrderNo].r, BlockNo);
								Err = MoveR(strRAxis, Ratio.r, TimeOut, Avoid.pt[OrderNo].r, InposR, Ms, false);
								if (Err != NO_ERR)
								{
									TRACE(_T("[PWR] Avoid Order:%d MoveR Err:%d\n"), OrderNo, Err);
									break;
								}
							}
							Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, InposXY, Ms, TimeOut);
							if (Err == NO_ERR)
							{
								Cur = gReadGantryPosition(Gantry);
								WriteConfirmInsertBeforePosition(Cur);
								if (Target == TBL_CAMERA)
								{
									if (Gantry == FRONT_GANTRY)
									{
										gLedOn(FHCAM, 10, 10, 0);
										gLiveOn(FHCAM);
									}
									else
									{
										gLedOn(RHCAM, 10, 10, 0);
										gLiveOn(RHCAM);
									}
								}
								TRACE(_T("[PWR] Avoid MoveXY InsertNo(%03d) Order(%02d) AvoidXY,%.3f,%.3f\n"), InsertNo, OrderNo + 1, Avoid.pt[OrderNo].x, Avoid.pt[OrderNo].y);
							}
							if (Target != TBL_CAMERA)
							{
								Err = WaitR(strRAxis, Avoid.pt[OrderNo].r, InposR, TimeOut);
								AvoidZ = GetInsertByZ(Gantry) - BodyHeight - LeadHeight - Nozzle.TipHeight + Nozzle.PusherHeight - Avoid.Height[OrderNo];
								TRACE(_T("[PWR] Avoid,%.3f InsertZ,%.3f Height Body,%.3f Lead,%.3f Nozzle Tip,%.3f Pusher,%.3f User Height,%.3f\n"),
									AvoidZ, GetInsertByZ(Gantry), BodyHeight, LeadHeight, Nozzle.TipHeight, Nozzle.PusherHeight, Avoid.Height[OrderNo]);
								Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
								if (limit.minus > AvoidZ)
								{
									TRACE(_T("[PWR] Avoid Target position,%.3f is under minus limit,%.3f\n"), AvoidZ, limit.minus);
									AvoidZ = limit.minus + 1.0;
								}
								Err = MoveZDown(strZAxis, Ratio.z, TimeOut, AvoidZ, InposZDn, Ms, false);
								if (Err != NO_ERR)
								{
									TRACE(_T("[PWR] Avoid Order:%d MoveZDown Err:%d\n"), OrderNo, Err);
									break;
								}
								Err = WaitZDown(strZAxis, AvoidZ, InposZDn, TimeOut);
								if (Err != NO_ERR)
								{
									TRACE(_T("[PWR] Avoid Order:%d WaitZDown Err:%d\n"), OrderNo, Err);
									break;
								}
							}
							else
							{
								TRACE(_T("[PWR] Avoid InsertNo(%03d) Order(%02d) Target:%d"), InsertNo, OrderNo + 1, Target);
							}
						}
					}
				}
			}
			SetMachineState(STATE_IDLE);
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("10000"));
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}
