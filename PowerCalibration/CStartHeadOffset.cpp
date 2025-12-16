#include "pch.h"
#include "CStartHeadOffset.h"
#include "CPowerCalibrationData.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "VisionData.h"
//#include "ErrorCode.h"
#include "CMachineFileDB.h"

CStartHeadOffset* gcStartHeadOffset;
CStartHeadOffset::CStartHeadOffset()
{
    m_CalibrationHead = NON;
    m_RepeatCount = 0;
    m_bApply = false;
    ClearStep();
}

CStartHeadOffset::~CStartHeadOffset()
{
}

void CStartHeadOffset::SetCalibrationHead(long HeadNo)
{
    m_CalibrationHead = HeadNo;
    TRACE(_T("[PWR] SetCalibrationHead(%d)\n"), HeadNo);
}

long CStartHeadOffset::GetCalibrationHead()
{
    return m_CalibrationHead;
}

void CStartHeadOffset::SetRepeatCount(int RepeatCount)
{
    m_RepeatCount = RepeatCount;
    TRACE(_T("[PWR] SetRepeatCount(%d)\n"), RepeatCount);
}

long CStartHeadOffset::GetRepeatCount()
{
    return m_RepeatCount;
}

void CStartHeadOffset::ClearStep()
{
    m_Step = CalibrationHeadOffsetStep::INIT;
}

void CStartHeadOffset::SetHeadOffsetMode(bool bApply)
{
    m_bApply = bApply;
    TRACE(_T("[PWR] SetHeadOffsetMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStartHeadOffset::GetHeadOffsetMode()
{
    return m_bApply;
}

Point_XY CStartHeadOffset::GetCameraAlignPosition(long Gantry)
{
    Point_XY pt;
    pt = gcPowerCalibrationData->GetCameraAlignPosition(Gantry);
    return pt;
}

long CStartHeadOffset::GetAlignCameraNo(long Gantry)
{
    long AlignCamera = NON;
    AlignCamera = gcPowerCalibrationData->GetAlignCamera(Gantry);
    return AlignCamera;
}

void CStartHeadOffset::SetHeadOffset(long Gantry, long ZAxisNo, Point_XY pt)
{
    if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
    {
        if (Gantry == FRONT_GANTRY)
        {
            m_FrontHeadOffset[ZAxisNo - 1] = pt;
        }
        else if (Gantry == REAR_GANTRY)
        {
            m_RearHeadOffset[ZAxisNo - 1] = pt;
        }
    }
}

void CStartHeadOffset::SetCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY pt)
{
    if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
    {
        if (Gantry == FRONT_GANTRY)
        {
            m_FrontCameraRecogPosition[ZAxisNo - 1] = pt;
        }
        else if (Gantry == REAR_GANTRY)
        {
            m_RearCameraRecogPosition[ZAxisNo - 1] = pt;
        }
    }
}

void CStartHeadOffset::SetCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY pt)
{
    if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
    {
        if (Gantry == FRONT_GANTRY)
        {
            m_FrontCameraRecogOffset[ZAxisNo - 1] = pt;
        }
        else if (Gantry == REAR_GANTRY)
        {
            m_RearCameraRecogOffset[ZAxisNo - 1] = pt;
        }
    }
}

bool CStartHeadOffset::CopyHeadOffset(long Gantry)
{
    if (GetHeadOffsetMode() == true)
    {
        for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
        {
            if (GetCalibrationHead() != NON && indx + TBL_HEAD1 != GetCalibrationHead()) continue;
            gcPowerCalibrationData->SetHeadOffset(Gantry, indx + TBL_HEAD1, m_FrontHeadOffset[indx]);
            TRACE(_T("[PWR] SetHeadOffset indx:%d XY:%.3f %.3f\n"), indx, m_FrontHeadOffset[indx].x, m_FrontHeadOffset[indx].y);
            gcPowerCalibrationData->SetCameraRecognitionPosition(Gantry, indx + TBL_HEAD1, m_FrontCameraRecogPosition[indx]);
            TRACE(_T("[PWR] SetCameraRecognitionPosition indx:%d XY:%.3f %.3f\n"), indx, m_FrontCameraRecogPosition[indx].x, m_FrontCameraRecogPosition[indx].y);
            gcPowerCalibrationData->SetCameraRecognitionOffset(Gantry, indx + TBL_HEAD1, m_FrontCameraRecogOffset[indx]);
            TRACE(_T("[PWR] SetCameraRecognitionOffset indx:%d XY:%.3f %.3f\n"), indx, m_FrontCameraRecogOffset[indx].x, m_FrontCameraRecogOffset[indx].y);

            gcPowerCalibrationData->SetCameraRecognitionPosition(Gantry + 1, indx + TBL_HEAD1, m_RearCameraRecogPosition[indx]);
            TRACE(_T("[PWR] SetCameraRecognitionPosition Rear indx:%d XY:%.3f %.3f\n"), indx, m_RearCameraRecogPosition[indx].x, m_RearCameraRecogPosition[indx].y);
            gcPowerCalibrationData->SetCameraRecognitionOffset(Gantry + 1, indx + TBL_HEAD1, m_RearCameraRecogOffset[indx]);
            TRACE(_T("[PWR] SetCameraRecognitionOffset Rear indx:%d XY:%.3f %.3f\n"), indx, m_RearCameraRecogOffset[indx].x, m_RearCameraRecogOffset[indx].y);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void CStartHeadOffset::WriteHeadOffset(long Gantry)
{
    gcPowerCalibrationData->WriteHeadOffset(Gantry);
    gcPowerCalibrationData->WriteCameraRecognitionPosition(Gantry);
    gcPowerCalibrationData->WriteCameraRecognitionOffset(Gantry);
}

long CStartHeadOffset::StartHeadOffsetCalibration(void)
{
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    long TimeChk = 0, RetVis = 0, Err = NO_ERR;
    CalibrationHeadOffsetStep OldStep = CalibrationHeadOffsetStep::MAX, ErrorStep = CalibrationHeadOffsetStep::MAX;
    Point_XY Align, Dist, Cur, Offset, Set;
    CString StrZAxis;
    double Ratio = 0.3, Inpos = 0.001;
    long Gantry = FRONT_GANTRY, TimeOut = TIME10000MS, Ms = TIME300MS, ZAxisNo = 0, TargetHeadNo = NON;
    double cx, cy, Res;
    INT_PTR indx = 0, indy = 0;
    long retry = 0, AlignCamera = MAXCAMNO, RecogCamera = MAXCAMNO, RecogTable = FRONT_GANTRY;
    long LedLevel = 100;
	double radius = 0.000;

    gLedAllOff();
    LoadHeadOffsetCamPosOffsetValue(Gantry);
    if (IsAllAxisHomingComplete() == false)
    {
        TRACE(_T("[PWR] CStartHeadOffset All Axis Homing NOT Complate\n"));
        return GetHomingCompleteError();
    }
    AlignCamera = GetAlignCameraNo(Gantry);
    Align = GetCameraAlignPosition(Gantry);
    TRACE(_T("[PWR] CStartHeadOffset Align PositionXY:%.3f,%.3f\n"), Align.x, Align.y);
    if (AlignCamera < CAM1 || AlignCamera > CAM8)
    {
        TRACE(_T("[PWR] CStartHeadOffset Align Camera(%d) is OVER\n"), AlignCamera);
        return 1;
    }
    if (abs(Align.x) > MAX_MOTION_VALID_RANGE || abs(Align.y) > MAX_MOTION_VALID_RANGE)
    {
        TRACE(_T("[PWR] CStartHeadOffset Camera Align Position is OVER X,Y %.3f %.3f\n"), Align.x, Align.y);
        return 1;
    }
    TRACE(_T("[PWR] Calibration Head All:%s\n"), GetCalibrationHead()==NON?_T("ALL"):_T("ONE"));
    if (GetCalibrationHead() != NON)
    {
        TRACE(_T("[PWR] Calibration Head One(%s)\n"), GetZAxisFromHeadNo(Gantry, GetCalibrationHead()));
    }
    TRACE(_T("[PWR] CStartHeadOffset All ZAxis Count:%d\n"), GetZAxisCount());
    // Find Head Offset
    for (indx = 0; indx < GetZAxisCount(); ++indx)
    {
        cx = cy = Res = 0.0;
        StrZAxis = GetZAxisByIndex(indx);
        ZAxisNo = GetZAxisIndexByZName(StrZAxis);
        if (ZAxisNo == NON) continue;
        if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead()) continue;
        Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
        if (Err != NO_ERR)
        {
            TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
            return Err;
        }
        Err = StartAllRAxisWaitDelayedPosSet(Gantry, 0.000, Ratio, Inpos, Ms, TIME5000MS);
        if (Err != NO_ERR)
        {
            TRACE(_T("[PWR] CStartHeadOffset StartAllRAxisWaitDelayedPosSet Err:%d\n"), Err);
            return Err;
        }
        Cur = Align;
        for (retry = 0; retry < MAX_RETRY_HEADOFFSET; ++retry)
        {
            Err = LinearIntplPosWaitDelayedPosSet(Gantry, ZAxisNo, Cur, Ratio, Inpos, Ms, TimeOut);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
                return Err;
            }
            Err = StartPosWaitDelayedInposition(StrZAxis, Ratio, TimeOut, GetInsertByZ(Gantry), Inpos, Ms, true);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset StartPosWaitDelayedInposition(%s) Err:%d\n"), StrZAxis, Err);
                return Err;
            }
            //gLedOn(AlignCamera, LedLevel, LedLevel, 0);
            ThreadSleep(TIME50MS);
            RetVis = findRotateCenterWithRes(Gantry, StrZAxis, AlignCamera, &cx, &cy, &Res);
			radius = Res;
            if (RetVis != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset findRotateCenterWithRes Ret:%d\n"), RetVis);
                return RetVis;
            }
            if (abs(cx) < 0.005 && abs(cy) < 0.005)
            {
                TRACE(_T("[PWR] (%s) Head Offset Result X,Y %.3f %.3f\n"), StrZAxis, cx, cy);
                break;
            }
            else
            {
                TRACE(_T("[PWR] (%s) Try:%02d Head Offset Result X,Y %.3f %.3f\n"), StrZAxis, retry + 1, cx, cy);
            }
            Cur.x += cx;
            Cur.y += cy;
        }
        if (retry < 5)
        {
            Cur = gReadGantryPosition(Gantry);
            Dist.x = Cur.x - Align.x;
            Dist.y = Cur.y - Align.y;
            TRACE(_T("[PWR] (%s) Set Head Offset Result X,Y %.3f %.3f\n"), StrZAxis, Dist.x, Dist.y);
            SetHeadOffset(Gantry, ZAxisNo, Dist);
			gcMachineFileDB->UpdateHeadRadius(Gantry, ZAxisNo, radius);

			for (long degreeIndex = 0; degreeIndex < MAX_DEGREE_INDEX; degreeIndex++)
			{
				Point_XY offset;
				offset.x = offset.y = 0.000;
				gcPowerCalibrationData->SetInsertOffset4532(Gantry, ZAxisNo, degreeIndex, offset);
			}
			gcMachineFileDB->SaveInsertOffset4532FromDB();

        }        
        Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
        if (Err != NO_ERR)
        {
            TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
            return Err;
        }
    }
    TRACE(_T("[PWR] CStartHeadOffset Recognition All ZAxis Count:%d\n"), GetZAxisCount());
    // Find Recognition Position & Offset
    for (indx = 0; indx < GetZAxisCount(); ++indx)
    {
        cx = cy = Res = 0.0;
        StrZAxis = GetZAxisByIndex(indx);
        ZAxisNo = GetZAxisIndexByZName(StrZAxis);
        if (ZAxisNo == NON) continue;
        if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
        {
            TRACE(_T("[PWR] CStartHeadOffset Recognition Skip-0 ZAxis:%s ZAxisNo:%d CalHead:%d\n"), StrZAxis, ZAxisNo, GetCalibrationHead());
            continue;
        }
        if (GetCalibrationHead() == NON) // All Head
        {
            Cur = Align;
        }
        else
        {
            if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
            {
                TargetHeadNo = GetCamera6HeadFromHeadNo(ZAxisNo);
                if (IsCamera6CenterPositionByHeadNo(ZAxisNo) == true)
                {
                    Cur.x = Align.x + GetHeadOffset(Gantry, ZAxisNo).x;
                    Cur.y = Align.y + GetHeadOffset(Gantry, ZAxisNo).y;
                    TRACE(_T("[PWR] CStartHeadOffset(Center) CurHeadNo:%d TargetNo:%d RecogXY:%.3f,%.3f\n"), ZAxisNo, TargetHeadNo, Cur.x, Cur.y);
                }
                else
                {
                    Cur = GetCameraRecognitionPosition(Gantry, TargetHeadNo);
                    TRACE(_T("[PWR] CStartHeadOffset(Offset) CurHeadNo:%d TargetNo:%d RecogXY:%.3f,%.3f\n"), ZAxisNo, TargetHeadNo, Cur.x, Cur.y);
                }
            }
            else // 2 Camera 
            {
                TargetHeadNo = GetCameraHeadFromHeadNo(ZAxisNo);
                if (IsCameraCenterPositionByHeadNo(ZAxisNo) == true)
                {
                    Cur.x = Align.x + GetHeadOffset(Gantry, ZAxisNo).x;
                    Cur.y = Align.y + GetHeadOffset(Gantry, ZAxisNo).y;
                    TRACE(_T("[PWR] CStartHeadOffset(Center) CurHeadNo:%d TargetNo:%d RecogXY:%.3f,%.3f\n"), ZAxisNo, TargetHeadNo, Cur.x, Cur.y);
                }
                else
                {
                    Cur = GetCameraRecognitionPosition(Gantry, TargetHeadNo);
                    TRACE(_T("[PWR] CStartHeadOffset(Offset) CurHeadNo:%d TargetNo:%d RecogXY:%.3f,%.3f\n"), ZAxisNo, TargetHeadNo, Cur.x, Cur.y);
                }
            }

        }
        // Find Recognition Position
        for (retry = 0; retry < MAX_RETRY_HEADOFFSET; ++retry)
        {
            if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition Skip-1 ZAxis:%s ZAxisNo:%d CalHead:%d\n"), StrZAxis, ZAxisNo, GetCalibrationHead());
                continue;
            }
            Err = LinearIntplPosWaitDelayedPosSet(Gantry, TBL_CAMERA, Cur, Ratio, Inpos, Ms, TimeOut);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
                return Err;
            }
            Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetInsertByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
                return Err;
            }
            if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
            {
                if (IsCamera6CenterPositionByHeadNo(ZAxisNo) == false)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Recognition Position Skip(%d)\n"), ZAxisNo);
                    break;
                }
                RecogCamera = GetCamera6NoByHead(RecogTable, ZAxisNo);
            }
            else
            {
                if (IsCameraCenterPositionByHeadNo(ZAxisNo) == false)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Recognition Position Skip(%d)\n"), ZAxisNo);
                    break;
                }
                RecogCamera = GetCameraNoByHead(RecogTable, ZAxisNo);
            }
            //gLedOn(RecogCamera, LedLevel, LedLevel, 0);
            ThreadSleep(TIME50MS);
            RetVis = findRotateCenterWithRes(Gantry, StrZAxis, RecogCamera, &cx, &cy, &Res);
            if (RetVis != 0)
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition-1 findRotateCenterWithRes Ret:%d\n"), RetVis);
                return RetVis;
            }
            if (abs(cx) < 0.005 && abs(cy) < 0.005)
            {
                TRACE(_T("[PWR] (%s) Camera(%d) Recognition Position Result X,Y %.3f %.3f R %.3f\n"), StrZAxis, RecogCamera, cx, cy, Res);
                break;
            }
            else
            {
                TRACE(_T("[PWR] (%s) Try:%02d Camera(%d) Recognition Position Result X,Y %.3f %.3f\n"), StrZAxis, RecogCamera, retry + 1, cx, cy);
            }
            Cur.x += cx;
            Cur.y += cy;
        }
        if (retry < MAX_RETRY_HEADOFFSET)
        {
            Cur = gReadGantryPosition(Gantry);
            TRACE(_T("[PWR] (%s) Camera Recognition Position X,Y %.3f %.3f\n"), StrZAxis, Cur.x, Cur.y);
            SetCameraRecognitionPosition(Gantry, ZAxisNo, Cur);
        }
        // Find Recognition Offset
        for (indy = 0; indy < GetZAxisCount(); ++indy)
        {
            Set.x = Set.y = Offset.x = Offset.y = Res = 0.0;
            StrZAxis = GetZAxisByIndex(indy);
            ZAxisNo = GetZAxisIndexByZName(StrZAxis);
            if (ZAxisNo == NON) continue;
            if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
            {
                if (IsCamera6CenterPositionByHeadNo(ZAxisNo) == true) continue;
            }
            else
            {
                if (IsCameraCenterPositionByHeadNo(ZAxisNo) == true) continue;
            }
            if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition Skip-2 ZAxis:%s\n"), StrZAxis);
                continue;
            }
            if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
            {
                RecogCamera = GetCamera6NoByHead(RecogTable, ZAxisNo);
            }
            else
            {
                RecogCamera = GetCameraNoByHead(RecogTable, ZAxisNo);
            }
            TRACE(_T("[PWR] Find Recognition Offset Cam:%d Head:%d\n"), RecogCamera, ZAxisNo);
            //gLedOn(RecogCamera, LedLevel, LedLevel, 0);
            ThreadSleep(TIME50MS);
            RetVis = findRotateCenterWithRes(Gantry, StrZAxis, RecogCamera, &Offset.x, &Offset.y, &Res);
            if (RetVis != 0)
            {
                TRACE(_T("[PWR] CStartHeadOffset Recognition-2 findRotateCenterWithRes Ret:%d\n"), RetVis);
                return RetVis;
            }
            Set.x = 0.0 - Offset.x;
            Set.y = 0.0 - Offset.y;
            TRACE(_T("[PWR] (%s) Camera(%d) Recognition Offset X,Y %.3f %.3f R %.3f\n"), StrZAxis, RecogCamera, Set.x, Set.y, Res);
            SetCameraRecognitionPosition(Gantry, ZAxisNo, Cur);
            SetCameraRecognitionOffset(Gantry, ZAxisNo, Set);
        }
        Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
        if (Err != NO_ERR)
        {
            TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
            return Err;
        }
    }

    Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
    if (Err != NO_ERR)
    {
        TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
        return Err;
    }

    if (GetUseRearCamera() == 1) // Rear는 All 2 Camera
    {
        // Find Rear Recognition Position & Offset
        for (indx = 0; indx < GetZAxisCount(); ++indx)
        {
            cx = cy = Res = 0.0;
            StrZAxis = GetZAxisByIndex(indx);
            ZAxisNo = GetZAxisIndexByZName(StrZAxis);
            if (ZAxisNo == NON) continue;
            if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
            {
                TRACE(_T("[PWR] CStartHeadOffset Rear Recognition Skip-0 ZAxis:%s ZAxisNo:%d CalHead:%d\n"), StrZAxis, ZAxisNo, GetCalibrationHead());
                continue;
            }
            TargetHeadNo = GetCameraHeadFromHeadNo(ZAxisNo);
            Cur = GetCameraRecognitionPosition(RecogTable + 1, TargetHeadNo);
            TRACE(_T("[PWR] CStartHeadOffset(Offset) Rear CurHeadNo:%d TargetNo:%d RecogXY:%.3f,%.3f\n"), ZAxisNo, TargetHeadNo, Cur.x, Cur.y);
            // Find Rear Recognition Position
            for (retry = 0; retry < MAX_RETRY_HEADOFFSET; ++retry)
            {
                if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition Skip-1 ZAxis:%s ZAxisNo:%d CalHead:%d\n"), StrZAxis, ZAxisNo, GetCalibrationHead());
                    continue;
                }
                Err = LinearIntplPosWaitDelayedPosSet(Gantry, TBL_CAMERA, Cur, Ratio, Inpos, Ms, TimeOut);
                if (Err != NO_ERR)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
                    return Err;
                }
                Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetInsertByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
                if (Err != NO_ERR)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
                    return Err;
                }
                if (IsCameraCenterPositionByHeadNo(ZAxisNo) == false)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition Position Skip(%d)\n"), ZAxisNo);
                    break;
                }
                RecogCamera = GetCameraNoByHead(RecogTable + 1, ZAxisNo);
                TRACE(_T("[PWR] Find Rear Recognition Position Cam:%d Head:%d\n"), RecogCamera, ZAxisNo);
                //gLedOn(RecogCamera, LedLevel, LedLevel, 0);
                ThreadSleep(TIME50MS);
                RetVis = findRotateCenterWithRes(Gantry, StrZAxis, RecogCamera, &cx, &cy, &Res);
                if (RetVis != 0)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition-1 findRotateCenterWithRes Ret:%d\n"), RetVis);
                    return RetVis;
                }
                if (abs(cx) < 0.005 && abs(cy) < 0.005)
                {
                    TRACE(_T("[PWR] (%s) Camera(%d) Rear Recognition Position Result X,Y %.3f %.3f R %.3f\n"), StrZAxis, RecogCamera, cx, cy, Res);
                    break;
                }
                else
                {
                    TRACE(_T("[PWR] (%s) Try:%02d Camera(%d) Rear Recognition Position Result X,Y %.3f %.3f\n"), StrZAxis, RecogCamera, retry + 1, cx, cy);
                }
                Cur.x += cx;
                Cur.y += cy;
            }
            if (retry < MAX_RETRY_HEADOFFSET)
            {
                Cur = gReadGantryPosition(Gantry);
                TRACE(_T("[PWR] (%s) Camera Recognition Rear Position X,Y %.3f %.3f\n"), StrZAxis, Cur.x, Cur.y);
                SetCameraRecognitionPosition(RecogTable + 1, ZAxisNo, Cur);
            }
            // Find Rear Recognition Offset
            for (indy = 0; indy < GetZAxisCount(); ++indy)
            {
                Set.x = Set.y = Offset.x = Offset.y = Res = 0.0;
                StrZAxis = GetZAxisByIndex(indy);
                ZAxisNo = GetZAxisIndexByZName(StrZAxis);
                if (ZAxisNo == NON) continue;
                if (IsCameraCenterPositionByHeadNo(ZAxisNo) == true) continue;
                if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead())
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition Skip-2 ZAxis:%s\n"), StrZAxis);
                    continue;
                }
                RecogCamera = GetCameraNoByHead(RecogTable + 1, ZAxisNo);
                TRACE(_T("[PWR] Find Rear Recognition Offset Cam:%d Head:%d\n"), RecogCamera, ZAxisNo);
                //gLedOn(RecogCamera, LedLevel, LedLevel, 0);
                ThreadSleep(TIME50MS);
                RetVis = findRotateCenterWithRes(Gantry, StrZAxis, RecogCamera, &Offset.x, &Offset.y, &Res);
                if (RetVis != 0)
                {
                    TRACE(_T("[PWR] CStartHeadOffset Rear Recognition-2 findRotateCenterWithRes Ret:%d\n"), RetVis);
                    return RetVis;
                }
                Set.x = 0.0 - Offset.x;
                Set.y = 0.0 - Offset.y;
                TRACE(_T("[PWR] (%s) Camera(%d) Rear Recognition Offset X,Y %.3f %.3f R %.3f\n"), StrZAxis, RecogCamera, Set.x, Set.y, Res);
                SetCameraRecognitionPosition(RecogTable + 1, ZAxisNo, Cur);
                SetCameraRecognitionOffset(RecogTable + 1, ZAxisNo, Set);
            }
            Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
                return Err;
            }
        }
        Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
        if (Err != NO_ERR)
        {
            TRACE(_T("[PWR] CStartHeadOffset StartAllZAxisWaitDelayedPosSet Err:%d\n"), Err);
            return Err;
        }
    }
    CopyHeadOffset(Gantry);
    WriteHeadOffset(Gantry);

    TRACE(_T("[PWR] CStartHeadOffset(0x%x) Quit\n"), GetThreadID());

	Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
	gLedAllOff();

    return NO_ERR;
}

void CStartHeadOffset::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thHeadOffsetCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)RunThread;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] HeadOffset Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] HeadOffset Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

UINT CStartHeadOffset::RunThread(LPVOID wParam)
{
	CStartHeadOffset* pThis = reinterpret_cast<CStartHeadOffset*>(wParam);
	long Err = NO_ERR;

    /*
    * 20250530_중복 동작 방지
    * NEW_DEVELOP, Machine manual Action Running Sign, Lock-UnLock
    */
    if (GetMachineManualActionRunninSign() == true) {
        TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
    }
    else {
        SetMachineManualActionRunninSign(true);
        Err = pThis->StartHeadOffsetCalibration();
        SetMachineManualActionRunninSign(false);
    }
    /*Err = pThis->StartHeadOffsetCalibration();*/

	CString strMsg;
	strMsg.Format(_T("%d,%d,%d,%d"), Err, FRONT_GANTRY, pThis->GetCalibrationHead(), 0);
	SendToHMI(HMI_CMD1ST_3, HMI_CMD2ND_110, HMI_CMD3RD_02, strMsg);

	return Err;
}
