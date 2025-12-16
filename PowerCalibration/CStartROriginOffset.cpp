#include "pch.h"
#include "CStartROriginOffset.h"
#include "CPowerCalibrationData.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "VisionData.h"
//#include "ErrorCode.h"
#include "COriginSearch.h"

CStartROriginOffset* gcStartROriginOffset;
CStartROriginOffset::CStartROriginOffset()
{
    m_CalibrationHead = NON;
    m_RepeatCount = 0;
    m_bApply = false;
    ClearStep();
}

CStartROriginOffset::~CStartROriginOffset()
{
}

void CStartROriginOffset::SetCalibrationHead(long HeadNo)
{
    m_CalibrationHead = HeadNo;
    TRACE(_T("[PWR] SetCalibrationHead(%d)\n"), HeadNo);
}

long CStartROriginOffset::GetCalibrationHead()
{
    return m_CalibrationHead;
}

void CStartROriginOffset::ClearStep()
{

}

void CStartROriginOffset::SetHeadOffsetMode(bool bApply)
{
    m_bApply = bApply;
    TRACE(_T("[PWR] SetHeadOffsetMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStartROriginOffset::GetHeadOffsetMode()
{
    return m_bApply;
}

Point_XY CStartROriginOffset::GetCameraAlignPosition(long Gantry)
{
    Point_XY pt;
    pt = gcPowerCalibrationData->GetCameraAlignPosition(Gantry);
    return pt;
}

long CStartROriginOffset::GetAlignCameraNo(long Gantry)
{
    long AlignCamera = NON;
    AlignCamera = gcPowerCalibrationData->GetAlignCamera(Gantry);
    return AlignCamera;
}

bool CStartROriginOffset::SetROriginOffset(long Gantry)
{
    if (GetHeadOffsetMode() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CStartROriginOffset::WriteROriginOffset(long Gantry)
{

}

long CStartROriginOffset::StartHeadROffsetCalibration()
{
    bool bLoop = true, bErr = false;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    long TimeChk = 0, RetVis = 0, Err = NO_ERR;
    CalibrationHeadOffsetStep OldStep = CalibrationHeadOffsetStep::MAX, ErrorStep = CalibrationHeadOffsetStep::MAX;

    Point_XY Align, Cur;// Dist, Offset, Set;
    Point_XYRE ptXYRE;
    CString StrZAxis, StrRAxis;
    double Ratio = 0.3, Inpos = 0.001;
    long Gantry = FRONT_GANTRY, TimeOut = TIME10000MS, Ms = TIME300MS, ZAxisNo = 0, TargetHeadNo = NON;
    double cx, cy, Res, Origin = 0.0;
    INT_PTR indx = 0, indy = 0;
    long retry = 0, AlignCamera = MAXCAMNO, RecogCamera = MAXCAMNO, RecogTable = FRONT_GANTRY, AxisNo = NON;
    ULONGLONG GetTime = 0, Elapsed = 0;
    COriginSearch* pOriginSearch;
    gLedAllOff();
    if (IsAllAxisHomingComplete() == false)
    {
        TRACE(_T("[PWR] CStartROriginOffset All Axis Homing NOT Complate\n"));
        return GetHomingCompleteError();
    }
    AlignCamera = GetAlignCameraNo(Gantry);
    Align = GetCameraAlignPosition(Gantry);
    TRACE(_T("[PWR] CStartROriginOffset Align PositionXY:%.3f,%.3f\n"), Align.x, Align.y);
    if (AlignCamera < CAM1 || AlignCamera > CAM8)
    {
        TRACE(_T("[PWR] CStartROriginOffset Align Camera(%d) is OVER\n"), AlignCamera);
        return 1;
    }
    if (abs(Align.x) > MAX_MOTION_VALID_RANGE || abs(Align.y) > MAX_MOTION_VALID_RANGE)
    {
        TRACE(_T("[PWR] CStartROriginOffset Camera Align Position is OVER X,Y %.3f %.3f\n"), Align.x, Align.y);
        return 1;
    }
    TRACE(_T("[PWR] CStartROriginOffset Head All:%s\n"), GetCalibrationHead() == NON ? _T("ALL") : _T("ONE"));
    if (GetCalibrationHead() != NON)
    {
        TRACE(_T("[PWR] CStartROriginOffset Head One(%s)\n"), GetZAxisFromHeadNo(Gantry, GetCalibrationHead()));
    }
    TRACE(_T("[PWR] CStartROriginOffset All ZAxis Count:%d\n"), GetZAxisCount());

    MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio);
    MoveRStandy(Gantry, GetStandByR(Gantry), Ratio);
    WaitRStandBy(Gantry, GetStandByR(Gantry));

    // Find Head Offset
    for (indx = 0; indx < GetZAxisCount(); ++indx)
    {
        cx = cy = Res = 0.0;
        StrZAxis = GetZAxisByIndex(indx);
        StrRAxis = GetRAxisByIndex(indx);
        AxisNo = GetAxisIndexFromAliasName(StrRAxis);
        Origin = gGetHomePosition(AxisNo);
        TRACE(_T("[PWR] CStartROriginOffset %s(%d) Old Origin:%.3f\n"), StrRAxis, AxisNo, Origin);
        ZAxisNo = GetZAxisIndexByZName(StrZAxis);
        if (ZAxisNo == NON) continue;
        if (GetCalibrationHead() != NON && ZAxisNo != GetCalibrationHead()) continue;
        Cur = Align;
        for (retry = 0; retry < MAX_RETRY_HEADOFFSET; ++retry)
        {
			ThreadSleep(TIME500MS);
			WaitAllRIdle(Gantry, TIME2000MS);

            Err = LinearIntplPosWaitDelayedPosSet(Gantry, ZAxisNo, Cur, Ratio, Inpos, Ms, TimeOut);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartROriginOffset LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
                return Err;
            }
            Err = StartPosWaitDelayedInposition(StrZAxis, Ratio, TimeOut, GetInsertByZ(Gantry), Inpos, Ms, true);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartROriginOffset StartPosWaitDelayedInposition(%s) Err:%d\n"), StrZAxis, Err);
                return Err;
            }
            gLedOn(AlignCamera, 60, 60, 0);
			ThreadSleep(TIME500MS);
            ptXYRE = ginspectROriginMark(AlignCamera, 0);
            RetVis = gGetVisionCommandResult(FRONT_VISION);
            Err = gGetVisionErrorCode(FRONT_VISION);
            bErr = true;
            TRACE(_T("[PWR] CStartROriginOffset AxisNo(%d) RetVis:%d Err:%d ResXYR,%.3f,%.3f,%.3f\n"), AxisNo, RetVis, Err, ptXYRE.x, ptXYRE.y, ptXYRE.r);
            if (RetVis != NO_ERR)
            {
                TRACE(_T("[PWR] CStartROriginOffset inspectROriginMark Ret:%d\n"), Err);
                return Err;
            }
            else
            {
                if (abs(ptXYRE.r) < 0.02)
                {
                    TRACE(_T("[PWR] CStartROriginOffset RetVis:%d Err:%d R,%.3f\n"), RetVis, Err, abs(ptXYRE.r));
                    break;
                }
                else
                {
                    Origin = Origin - ptXYRE.r;
                    TRACE(_T("[PWR] CStartROriginOffset %s(%d) New Origin:%.3f\n"), StrRAxis, AxisNo, Origin);
                    gSetHomePosition(AxisNo, Origin);
                    gWriteHomePosition(Gantry);
                    pOriginSearch = new COriginSearch(StrRAxis);
                    pOriginSearch->Run();
                    GetTime = 0, Elapsed = 0;
                    GetTime = _time_get();
                    ThreadSleep(TIME500MS);
                    while (1)
                    {
                        if (IsOneAxisHomingComplete(StrRAxis) == true)
                        {
							ThreadSleep(TIME100MS);
                            bErr = false;
                            break;
                        }
                        Elapsed = _time_elapsed(GetTime);
                        if (Elapsed > TIME10000MS)
                        {
                            TRACE(_T("[PWR] CStartROriginOffset(%s) TimeOut\n"), StrRAxis);
                            bErr = true;
                            break;
                        }
                        ThreadSleep(TIME10MS);
                    }
                    TRACE(_T("[PWR] CStartROriginOffset(%s) delete\n"), StrRAxis);
                    if (bErr == true)
                    {
                        break;
                    }

                    /*
                    * 20250529_r축 캘 중 stanby 파손 문제 수정
                    */
                    if (pOriginSearch->isHomingFail() == true) {
                        TRACE(_T("[PWR] CStartROriginOffset(%s) OriginSearch Fail. \n"), StrRAxis);
                        break;
                    }
                }
            }
        }
    }
    MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio);
    MoveRStandy(Gantry, GetStandByR(Gantry), Ratio);
    WaitRStandBy(Gantry, GetStandByR(Gantry));
	gLedAllOff();
    TRACE(_T("[PWR] CStartROriginOffset(0x%x) Quit\n"), GetThreadID());
    return NO_ERR;
}

void CStartROriginOffset::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thROriginOffsetCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)RunThread;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    TRACE(_T("[PWR] ROriginOffset Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] ROriginOffset Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

UINT CStartROriginOffset::RunThread(LPVOID wParam)
{
	CStartROriginOffset* pThis = reinterpret_cast<CStartROriginOffset*>(wParam);
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
        Err = pThis->StartHeadROffsetCalibration();
        SetMachineManualActionRunninSign(false);
    }
    /*Err = pThis->StartHeadROffsetCalibration();*/

	CString strMsg;
	strMsg.Format(_T("%d,%d,%d,%d"), Err, FRONT_GANTRY, pThis->GetCalibrationHead(), 0);
	SendToHMI(HMI_CMD1ST_3, HMI_CMD2ND_110, HMI_CMD3RD_02, strMsg);

	return Err;
}