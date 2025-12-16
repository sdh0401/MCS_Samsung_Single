#include "pch.h"
#include "CMark.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
#include "CStep.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CReadJobFile.h"
#include "Trace.h"
#include "CPowerGantry.h"

CMark* gcMark;
CMark::CMark(long Gantry)
{
    m_Gantry = Gantry;
}

CMark::~CMark()
{
}

long CMark::blockFiducialMarkChecking(const long& blockNo)
{
    const PCB Pcb = gcReadJobFile->GetPcb();

    if (Pcb.UseFiducial != FIDUCIAL_BLOCK)
    {
        TRACE_FILE_FUNC_LINE_"early return due to (Pcb.UseFiducial != FIDUCIAL_BLOCK).");
        return NO_ERR;
    }

    if (blockNo <= 0 || Pcb.MaxBlockCount < blockNo)
    {
        TRACE_FILE_FUNC_LINE_"early return due to invalid blockNo. (blockNo <= 0 || Pcb.MaxBlockCount < blockNo)");
        return RECOGNITION_MARK_FAIL;
    }

    const ORIGIN Origin = gcReadJobFile->GetOrigin();
    const ORIGIN OriginBlock = gcReadJobFile->GetBlockOrigin(blockNo);
    const FIDUCIAL Fiducial = gcReadJobFile->GetMark();
    const Point_XY OriginXY = Point_XY{ Origin.pt.x + OriginBlock.pt.x, Origin.pt.y + OriginBlock.pt.y };
    const Point_XY Mark1 = ReadMarkFromOrigin(Fiducial.pt[0], OriginXY);
    const Point_XY Mark2 = ReadMarkFromOrigin(Fiducial.pt[1], OriginXY);
    (void)gSetMarkLed(Fiducial.MarkNo[0], Fiducial.Led[0]);
    (void)gSetMarkLed(Fiducial.MarkNo[1], Fiducial.Led[1]);
    const Ratio_XYRZ Ratio = GetMinRatio();

    //해당 갠트리에서 하나라도 석션 ON 이면 true. -> 그러면 Ratio 최솟값 사용. 아닌경우 1.0 Ratio로 동작.
    const bool isSuctionOn =
        (
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD1) == true ||
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD2) == true ||
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD3) == true ||
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD4) == true ||
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD5) == true ||
            gcPowerGantry->GetOneSuction(FRONT_GANTRY, TBL_HEAD6) == true
            );

    long Ret = NO_ERR;

    Ret = gMarkPairRecognition(this->m_Gantry, Fiducial.MarkNo[0], Fiducial.MarkNo[1], Mark1, Mark2, (isSuctionOn == true) ? Ratio : Ratio_XYRZ{ 1.0, 1.0, 1.0 });
    if (Ret != NO_ERR)
    {
        return RECOGNITION_MARK_FAIL;
    }

    Ret = gGetMarkDelta(this->m_Gantry, MK_1, MK_2, Mark1, Mark2, blockNo);
    if (Ret != NO_ERR)
    {
        return RECOGNITION_MARK_FAIL;
    }

    return Ret;
}

ORIGIN CMark::GetOrigin()
{
	ORIGIN Origin;
	Origin = gcStep->GetOrigin();
	return Origin;
}

Point_XY CMark::GetMarkPosition(long MarkNo)
{
	Point_XY pt, Goal, OriginXY;
	pt.x = 0.000;
	pt.y = 0.000;
	ORIGIN Origin = GetOrigin();
	OriginXY.x = Origin.pt.x;
	OriginXY.y = Origin.pt.y;
	Goal = ReadMarkFromOrigin(pt, OriginXY);
	return Goal;
}

long CMark::FiducialMarkChecking()
{
    long Ret = NO_ERR;
    PCB Pcb = gcReadJobFile->GetPcb();
    ORIGIN Origin = gcStep->GetOrigin();
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] OriginXY,%.3f,%.3f\n"), Origin.pt.x, Origin.pt.y);
    }
    FIDUCIAL Fiducial = gcReadJobFile->GetMark();
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] Fiducial Mark1XY,%.3f,%.3f Mark2XY,%.3f,%.3f\n"), Fiducial.pt[0].x, Fiducial.pt[0].y, Fiducial.pt[1].x, Fiducial.pt[1].y);
    }
    if (Pcb.UseFiducial == FIDUCIAL_NOUSE)
    {
        return NO_ERR;
    }
    if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
    {
        for (int i = 1; i <= Pcb.MaxBlockCount; i++)
        {
            if (gcReadJobFile->GetBlockOrigin(i).Use == 0)
            {
                CString temp; temp.Format(L"skipping block(%d) fiducial mark checking due to use(%d) option.", i, gcReadJobFile->GetBlockOrigin(i).Use);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
            Ret = blockFiducialMarkChecking(i);
            if (Ret != NO_ERR)
            {
                return Ret;
            }
        }
        return Ret;
    }
    Point_XY OriginXY;
    OriginXY.x = Origin.pt.x;
    OriginXY.y = Origin.pt.y;
    Point_XY Mark1, Mark2;
    Mark1 = ReadMarkFromOrigin(Fiducial.pt[0], OriginXY, true);
    Mark2 = ReadMarkFromOrigin(Fiducial.pt[1], OriginXY, true);
    gSetMarkLed(Fiducial.MarkNo[0], Fiducial.Led[0]);
    gSetMarkLed(Fiducial.MarkNo[1], Fiducial.Led[1]);
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    Ratio_XYRZ Ratio = GetMinRatio();
    Ret = gMarkPairRecognition(m_Gantry, Fiducial.MarkNo[0], Fiducial.MarkNo[1], Mark1, Mark2, Ratio);
    if (Ret != NO_ERR)
    {
        return Ret;
    }
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] gMarkPairRecognition Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    Ret = gGetMarkDelta(m_Gantry, MK_1, MK_2, Mark1, Mark2, MK_PWB);
    return Ret;
}

long CMark::MachineReferenceMarkChecking(MachineReferenceMark MachineRefMark)
{
    long Ret = NO_ERR, Gantry = m_Gantry;
    double TargetZ = m_StandBy.pt.z, Ratio = 1.0, InposXY = 0.010;
    ULONGLONG GetTime = 0, Elapsed = 0;
    long Ms = TIME30MS, TimeOut = TIME5000MS, Cam = FHCAM, MarkType = MACHREFMARK;
    Point_XYRE Res[MACHINE_REFERENCE_MARKNO];
    Point_XY Pos[MACHINE_REFERENCE_MARKNO];
    double Compen1D_Y2[MACHINE_REFERENCE_MARKNO];
    double Compen2D_X[MACHINE_REFERENCE_MARKNO];
    double Compen2D_Y1[MACHINE_REFERENCE_MARKNO];
    double Compen2D_Y2[MACHINE_REFERENCE_MARKNO];
    GetTime = _time_get();
    ZeroMemory(&Res, sizeof(Res));
    ZeroMemory(&Pos, sizeof(Pos));
    ZeroMemory(&Compen1D_Y2, sizeof(Compen1D_Y2));
    ZeroMemory(&Compen2D_X, sizeof(Compen2D_X));
    ZeroMemory(&Compen2D_Y1, sizeof(Compen2D_Y1));
    ZeroMemory(&Compen2D_Y2, sizeof(Compen2D_Y2));
    MoveZStandy(Gantry, TargetZ, Ratio);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] MachineReferenceMarkChecking MoveZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    for (long MarkNo = 0; MarkNo < MACHINE_REFERENCE_MARKNO; ++MarkNo)
    {
        if (GetGlobalStatusError() == true)
        {
            TRACE(_T("[PWR] MachineReferenceMarkChecking GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
            Ret = STOP_NOW;
            break;
        }
        if (GetMachineState() == STATE_STOPNOW)
        {
            TRACE(_T("[PWR] MachineReferenceMarkChecking GetMachineState(%d)\n"), GetMachineState());
            Ret = STOP_NOW;
            break;
        }
        GetTime = _time_get();
        Ret = WaitGantryIdle(Gantry, TIME5000MS);
        if (_time_elapsed(GetTime) > 0)
        {
            TRACE(_T("[PWR] MachineReferenceMarkChecking WaitGantryIdle Mark(%d) Elasped,%d\n"), MarkNo + 1, _time_elapsed(GetTime));
        }
        if (Ret != NO_ERR)
        {
            return Ret;
        }
        gLedOn(Cam, 100, 0, 0);
        Ret = LinearIntplPosWaitDelayedPosSet(Gantry, Cam, MachineRefMark.Mark[MarkNo], Ratio, InposXY, Ms, TimeOut);
        if (Ret != NO_ERR)
        {
            return Ret;
        }        
        Pos[MarkNo] = gReadGantryPosition(Gantry);
        Compen1D_Y2[MarkNo] = Read1DCompensationData(_T("FY2"));
        Compen2D_X[MarkNo] = Read2DCompensationData(_T("FX"));
        Compen2D_Y1[MarkNo] = Read2DCompensationData(_T("FY1"));
        Compen2D_Y2[MarkNo] = Read2DCompensationData(_T("FY2"));
        Res[MarkNo] = gCatchMachRefMark(Cam, MarkType);
    }
    gLedAllOff();
    TRACE(_T("[PWR] MachineReferenceMark 1D-Y2 Pos1,%.3f Pos2,%.3f Pos3,%.3f Pos4,%.3f\n"), Compen1D_Y2[0], Compen1D_Y2[1], Compen1D_Y2[2], Compen1D_Y2[3]);
    TRACE(_T("[PWR] MachineReferenceMark 2D-X  Pos1,%.3f Pos2,%.3f Pos3,%.3f Pos4,%.3f\n"), Compen2D_X[0], Compen2D_X[1], Compen2D_X[2], Compen2D_X[3]);
    TRACE(_T("[PWR] MachineReferenceMark 2D-Y1 Pos1,%.3f Pos2,%.3f Pos3,%.3f Pos4,%.3f\n"), Compen2D_Y1[0], Compen2D_Y1[1], Compen2D_Y1[2], Compen2D_Y1[3]);
    TRACE(_T("[PWR] MachineReferenceMark 2D-Y2 Pos1,%.3f Pos2,%.3f Pos3,%.3f Pos4,%.3f\n"), Compen2D_Y2[0], Compen2D_Y2[1], Compen2D_Y2[2], Compen2D_Y2[3]);
    TRACE(_T("[PWR] MachineReferenceMark XY Pos1,%.3f,%.3f Pos2,%.3f,%.3f Pos3,%.3f,%.3f Pos4,%.3f,%.3f\n"),
        Pos[0].x, Pos[0].y, Pos[1].x, Pos[1].y, Pos[2].x, Pos[2].y, Pos[3].x, Pos[3].y);
    TRACE(_T("[PWR] MachineReferenceMark XY Mark1,%.3f,%.3f Mark2,%.3f,%.3f Mark3,%.3f,%.3f Mark4,%.3f,%.3f\n"),
        Res[0].x, Res[0].y, Res[1].x, Res[1].y, Res[2].x, Res[2].y, Res[3].x, Res[3].y);
    return Ret;
}

long CMark::MachineReferenceMarkCheckingAutoCompen(MachineReferenceMark MachineRefMark, Ratio_XYRZ ratio)
{
	long Ret = NO_ERR, Gantry = m_Gantry;
	double InposXY = 0.005;
	long MarkNo = 0;
	long Ms = TIME100MS, TimeOut = TIME5000MS, Cam = FHCAM, MarkType = MACHREFMARK;
	Point_XYRE MarkRes;
	double ratioMax = 0.7;

	if (ratio.xy > ratioMax)
	{
		ratio.xy = ratioMax;
	}
	if (ratio.z > ratioMax)
	{
		ratio.z = ratioMax;
	}
	if (ratio.r > ratioMax)
	{
		ratio.r = ratioMax;
	}

	if (Gantry == FRONT_GANTRY)
	{
		Cam = FHCAM;
		MarkNo = 0;
	}
	else
	{
		Cam = RHCAM;
		MarkNo = 2;
	}
	gLedOn(Cam, 100, 0, 0);

	Ret = MoveZAllUpLimit(Gantry, ratio.z);
	if (Ret != NO_ERR)
	{
		return Ret;
	}

    Ret = MoveZAllUpLimitWait(Gantry, TIME5000MS);
    if (Ret != NO_ERR)
    {
        return Ret;
    }

	Ret = WaitGantryIdle(Gantry, TIME5000MS);
	if (Ret != NO_ERR)
	{
		SendAlarm(Ret, _T("TimeOut WaitGantryIdle"));
		return Ret;
	}

	Ret = LinearIntplPosWaitDelayedPosSet(Gantry, Cam, MachineRefMark.Mark[MarkNo], ratio.xy, InposXY, Ms, TimeOut);
	if (Ret != NO_ERR)
	{
		return Ret;
	}

	ThreadSleep(TIME100MS);

	MarkRes = gCatchMachRefMark(Cam, MarkType);
	gLedOn(Cam, 0, 0, 0);

	if (GetGlobalSimulationMode() == true)
	{
		MarkRes.x = -0.48;
		MarkRes.y = 0.179;
		MarkRes.exe = 1;
	}

	if (MarkRes.exe != 1)
	{
		Ret = REFERENCE_MARK_FAIL;
		SendAlarm(Ret, _T("Reference Mark Recog Fail"));
		return Ret;
	}

	gcPowerCalibrationData->CalcRecogOffsetRefMarkCompen(FRONT_GANTRY, FRONT_STAGE, MachineRefMark.Mark[MarkNo], MarkRes);
	gcPowerCalibrationData->CalcRecogOffsetRefMarkCompen(FRONT_GANTRY, REAR_STAGE, MachineRefMark.Mark[MarkNo], MarkRes);
	//gcPowerCalibrationData->CalcRecogOffsetMarkCompen(REAR_GANTRY, MachineRefMark.Mark[MarkNo], MarkRes);

	return NO_ERR;
}

long CMark::SetStandBy(STANDBY StandBy)
{
    m_StandBy = StandBy;
    TRACE(_T("[PWR] CMark SetStandBy XYRZ:%.3f,%.3f,%.3f,%.3f\n"), StandBy.pt.x, StandBy.pt.y, StandBy.pt.r, StandBy.pt.z);
    return NO_ERR;
}

Ratio_XYRZ CMark::GetMinRatio()
{
    Ratio_XYRZ Ratio;
    Ratio = gcStep->GetMinRatio();
    return Ratio;
}
