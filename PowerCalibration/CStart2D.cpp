#include "pch.h"
#include "CStart2D.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "CPowerCalibrationData.h"
#include "CMachineFile.h"
#include "Trace.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "vision.h"
#include "VisionData.h"
#include "CPowerLog.h"
//#include "ErrorCode.h"

CStart2D* gcStart2D;
CStart2D::CStart2D()
{
    InitThreadData();
    m_Gantry = FRONT_GANTRY;
    m_StrX = GetAxisX(m_Gantry);
    m_StrY = GetAxisY1(m_Gantry);
    ClearStep();
    Clear2DCompen();
}

CStart2D::CStart2D(HANDLE h_Terminate)
{
    InitThreadData(h_Terminate);
    m_Gantry = FRONT_GANTRY;
    m_StrX = GetAxisX(m_Gantry);
    m_StrY = GetAxisY1(m_Gantry);
    ClearStep();
    Clear2DCompen();
}

CStart2D::~CStart2D()
{
}

void CStart2D::SetStartPosition(Point_XY pt)
{
    if (Get2DCompenMode() == true)
    {
        m_2DStartPosition = pt;
        if (gcPowerLog->IsShowCalibrationLog() == true)
        {
            TRACE(_T("[PWR] SetStartPosition 2D XY %.3f %.3f\n"), pt.x, pt.y);
        }
    }
}

Point_XY CStart2D::GetStartPosition()
{
    return m_2DStartPosition;
}

void CStart2D::Set2DBackup(long indx, Point_XYRE pt)
{
    m_2DBackupXY[indx] = pt;
}

Point_XYRE CStart2D::Get2DBackup(long indx)
{
    return m_2DBackupXY[indx];
}

void CStart2D::Clear2DCompen()
{
    ZeroMemory(m_2DBackupXY, sizeof(m_2DBackupXY));
    ZeroMemory(&m_2DStartPosition, sizeof(m_2DStartPosition));
    ZeroMemory(m_2DPositionXY, sizeof(m_2DPositionXY));
    ZeroMemory(m_2DOffsetXY, sizeof(m_2DOffsetXY));
}

void CStart2D::SetStep(Calibration2DStep StartStep)
{
    m_2DStep = StartStep;
}

void CStart2D::ClearStep()
{
    m_2DStep = Calibration2DStep::INIT;
}

void CStart2D::Set2DCompenMode(bool bApply)
{
    m_bApply = bApply;
    TRACE(_T("[PWR] Set2DCompenMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStart2D::Get2DCompenMode()
{
    return m_bApply;
}

void CStart2D::Set2DMethod(long Method)
{
    m_Method = Method;
    TRACE(_T("[PWR] Set2DCompenMode(%s)\n"), Method == 0 ? _T("WMX3") : _T("Software"));
    Set2DCompensationMethod(Method);
}

long CStart2D::Get2DMethod()
{
    return m_Method;
}

bool CStart2D::Set2D(unsigned x, unsigned y, Point_XYRE pt)
{
    if (Get2DCompenMode() == true)
    {
        if (Get2DMethod() == 1)
        {
            Set2DPosition(x, y, pt);
        }
        else
        {
            Set2DOffset(x, y, pt);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void CStart2D::Set2DPosition(unsigned x, unsigned y, Point_XYRE pt)
{
    m_2DPositionXY[x][y] = pt;
    if (gcPowerLog->IsShowCalibrationLog() == true)
    {
        TRACE(_T("[PWR] Set2DPosition 2D(X:%04d Y:%04d) XY %.3f %.3f\n"), x, y, m_2DPositionXY[x][y].x, m_2DPositionXY[x][y].y);
    }
}

void CStart2D::Set2DOffset(unsigned x, unsigned y, Point_XYRE pt)
{
    m_2DOffsetXY[x][y] = pt;
    if (gcPowerLog->IsShowCalibrationLog() == true)
    {
        TRACE(_T("[PWR] Set2DOffset 2D(X:%04d Y:%04d) XY %.3f %.3f\n"), x, y, m_2DOffsetXY[x][y].x, m_2DOffsetXY[x][y].y);
    }
}

bool CStart2D::Show2D(unsigned MaxCntX, unsigned MaxCntY)
{
    if (Get2DCompenMode() == true)
    {
        if (Get2DMethod() == 1)
        {
            Show2DPosition(MaxCntX, MaxCntY);
        }
        else
        {
            Show2DOffset(MaxCntX, MaxCntY);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void CStart2D::Show2DPosition(long MaxCntX, long MaxCntY)
{
    long indx = 0;
    for (long y = CAL_2D_INIT; y < MaxCntY; ++y)
    {
        for (long x = CAL_2D_INIT; x < MaxCntX; ++x)
        {
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] 2D(X:%04d Y:%04d) indx:%d Position XY %.3f %.3f\n"), x, y, indx, m_2DPositionXY[x][y].x, m_2DPositionXY[x][y].y);
            }
            indx++;
        }
    }
}

void CStart2D::Show2DOffset(long MaxCntX, long MaxCntY)
{
    long indx = 0;
    for (long y = CAL_2D_INIT; y < MaxCntY; ++y)
    {
        for (long x = CAL_2D_INIT; x < MaxCntX; ++x)
        {
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] 2D(X:%04d Y:%04d) indx:%d Offset XY %.3f %.3f\n"), x, y, indx, m_2DOffsetXY[x][y].x, m_2DOffsetXY[x][y].y);
            }
            indx++;
        }
    }
}

bool CStart2D::CopyGlobal2DCompensationData(long MaxCntX, long MaxCntY)
{
    long indx = 0;
    Point_XYRE pt;
    Point_XY Start;
    if (Get2DCompenMode() == true)
    {
        Start = m_2DStartPosition;
        gcPowerCalibrationData->Set2DStartPosition(FRONT_GANTRY, Start);
        for (long y = CAL_2D_INIT; y < MaxCntY; ++y)
        {
            for (long x = CAL_2D_INIT; x < MaxCntX; ++x)
            {
                if (Get2DMethod() == 1)
                {
                    pt = m_2DPositionXY[x][y];
                }
                else
                {
                    pt = m_2DOffsetXY[x][y];
                    gcPowerCalibrationData->Set2DCompensationData(FRONT_GANTRY, indx, pt);
                }
                indx++;
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool CStart2D::Write2D()
{
    if (Get2DCompenMode() == true)
    {
        if (Get2DMethod() == 1)
        {
        }
        else
        {
            gcPowerCalibrationData->Write2DCompensationData(FRONT_GANTRY);
        }
        return true;
    }
    else
    {
        return false;
    }
}

void CStart2D::Disable2D()
{
    if (Get2DMethod() == 1)
    {
        Set2DCompensationUse(false);
    }
    else
    {
        gDisable2DCompensation(CHANNEL_1);
        gDisable2DCompensation(CHANNEL_2);
        gDisable2DCompensation(CHANNEL_3);
    }
}

void CStart2D::Set2D(long Gantry, long MaxCntX, long MaxCntY)
{
    if (Get2DMethod() == 1)
    {

    }
    else
    {
        gSet2DCompensation_ForXY(Gantry, MaxCntX, MaxCntY);
    }
}

void CStart2D::Enable2D()
{
    if (Get2DMethod() == 1)
    {
        Set2DCompensationUse(true);
    }
    else
    {
        gEnable2DCompensation(CHANNEL_1);
        gEnable2DCompensation(CHANNEL_2);
        gEnable2DCompensation(CHANNEL_3);
    }
}

UINT CStart2D::Start2DCalibration(LPVOID wParam)
{
    bool bLoop = true, bRet = false;
    int CameraNo = FHCAM;
    long lTimeChk = 0, MarkType = MCALMARK, MaxMarkX = 0, MaxMarkY = 0, MaxMark = 0;
    long CurX = 0, CurY = 0, CurNo = 0, Err = NO_ERR;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    Calibration2DStep nOld2DStep = Calibration2DStep::MAX, ErrorStep = Calibration2DStep::INIT;
    CStart2D* pThis = reinterpret_cast<CStart2D*>(wParam);
    CApplicationTime* pTime = new CApplicationTime();
    CApplicationTime* pErrorTime = new CApplicationTime();
    Point_XY Start;
    Point_XY Cur;
    double dblTargetX, dblTargetY, dblPitch, oneDOffset = 0.0;
    dblTargetX = CAL_2D_MAX_XPOS;
    dblTargetY = CAL_2D_MAX_YPOS;
    dblPitch = CAL_2D_PITCH;

    MaxMarkX = (long)(dblTargetX / dblPitch) + 1; // 29
    MaxMarkY = (long)(dblTargetY / dblPitch) + 1; // 33
    MaxMark = MaxMarkX * MaxMarkY;
    if (gcPowerLog->IsShowCalibrationLog() == true)
    {
        TRACE(_T("[PWR] Start2DCalibration MaxXY %d %d Total:%d\n"), MaxMarkX, MaxMarkY, MaxMark);
    }
    ZeroMemory(&Start, sizeof(Start));
    Start = gReadGantryPosition(FRONT_GANTRY);
    double Ratio = 0.5;
    double Inpos = 0.001;
    long TimeOut = TIME10000MS, Ms = TIME300MS;
    if (pThis->Get2DCompenMode() == false)
    {
        Ratio = 0.7;
    }
    Point_XYRE res, LastOk, Apply;
    Point_XY pt, ptMax;
    long n = 0, retry = 0, before_x = 0, before_y = 0;
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (bLoop)
    {
        if (nOld2DStep != pThis->m_2DStep)
        {
            lTimeChk = pTime->TimeElapsed();
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] Start2DCalibration Front Step:%d Time:%d[ms]\n"), pThis->m_2DStep, lTimeChk);
            }
            nOld2DStep = pThis->m_2DStep;
            pTime->TimeGet();
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_2D_CALIBRATION_READTIME);
        //    continue;
        //}
        switch (pThis->m_2DStep)
        {
        case Calibration2DStep::INIT:
            pThis->m_2DStep = Calibration2DStep::START;
            break;

        case Calibration2DStep::START:
            if (pThis->Get2DCompenMode() == true)
            {
                twoDCompensationOff();
                gcMachineFile->InitToDiskBlock2();
                gcMachineFile->InitToDiskBlock4();            
                ThreadSleep(TIME500MS);
            }
            pThis->m_2DStep = Calibration2DStep::CHECK_ORIGIN;
            break;

        case Calibration2DStep::CHECK_ORIGIN:
            if (IsOneAxisHomingComplete(pThis->m_StrX) == true && IsOneAxisHomingComplete(pThis->m_StrY) == true)
            {
                pThis->m_2DStep = Calibration2DStep::CHECK_MAX_MARK;
            }
            else
            {
                TRACE(_T("[PWR] CStart2D Homing Failed %s(%d) %s(%d)\n"), pThis->m_StrX, IsOneAxisHomingComplete(pThis->m_StrX), pThis->m_StrY, IsOneAxisHomingComplete(pThis->m_StrY));
                pErrorTime->TimeGet();
                ErrorStep = pThis->m_2DStep;
                pThis->SetThreadStatusError(true);                
                pThis->m_2DStep = Calibration2DStep::ERROR_2D;
                break;
            }
            break;

        case Calibration2DStep::CHECK_MAX_MARK:
            ptMax.x = Start.x + dblTargetX;
            ptMax.y = Start.y + dblTargetY;
            Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, ptMax, Ratio, Inpos, Ms, TimeOut);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] Start2DCalibration Step:%d LinearIntplPosWaitDelayedPosSet Max Error:%d\n"), pThis->m_2DStep, Err);
                pThis->SetThreadStatusError(true);
                pThis->m_2DStep = Calibration2DStep::SELFQUIT;
                break;
            }            ThreadSleep(TIME100MS);
            res = gCatchMachCalMark(CameraNo, MarkType);
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] MaxNo:%04d XY %.3f %.3f Exe:%d\n"), MaxMark, res.x, res.y, res.exe);
            }
            if (res.exe == 1)
            {
                pThis->m_2DStep = Calibration2DStep::START_XY;
            }
            else
            {
                pErrorTime->TimeGet();
                ErrorStep = pThis->m_2DStep;
                pThis->SetThreadStatusError(true);
                pThis->m_2DStep = Calibration2DStep::ERROR_2D;
                break;
            }
            Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, Start, Ratio, Inpos, Ms, TimeOut);
            if (Err != NO_ERR)
            {
                TRACE(_T("[PWR] Start2DCalibration Step:%d LinearIntplPosWaitDelayedPosSet Start Error:%d\n"), pThis->m_2DStep, Err);
                pThis->SetThreadStatusError(true);
                pThis->m_2DStep = Calibration2DStep::SELFQUIT;
                break;
            }
            ThreadSleep(TIME100MS);
            res = gCatchMachCalMark(CameraNo, MarkType);
            Cur.x = Start.x;
            Cur.y = Start.y;
            pThis->SetStartPosition(Cur);
            TRACE(_T("[PWR] StartNo:%04d XY %.3f %.3f Vision Result XY %.3f %.3f Exe:%d\n"), CurNo, Cur.x, Cur.y, res.x, res.y, res.exe);
            if (res.exe == 1)
            {
                pThis->m_2DStep = Calibration2DStep::START_XY;
            }
            else
            {
                pErrorTime->TimeGet();
                ErrorStep = pThis->m_2DStep;
                pThis->SetThreadStatusError(true);
                pThis->m_2DStep = Calibration2DStep::ERROR_2D;
                break;
            }
            break;

        case Calibration2DStep::START_XY:
            for (int indy = 0; indy < MaxMarkY; indy++)
            {
                for (int inx = 0; inx < MaxMarkX; inx++)
                {
                    if ((indy % 2) == 0) // ----> 방향으로 검사.
                        n = inx;
                    else // <------ 방향으로 검사.
                        n = (MaxMarkX - 1) - inx;
                    CurNo = n + (indy * MaxMarkX);
                    pt.x = Start.x + (n * dblPitch);
                    pt.y = Start.y + (indy * dblPitch);
                    Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, pt, Ratio, Inpos, Ms, TimeOut);
                    if (Err != NO_ERR)
                    {
                        TRACE(_T("[PWR] Start2DCalibration Step:%d LinearIntplPosWaitDelayedPosSet Pt Error:%d\n"), pThis->m_2DStep, Err);
                        pThis->SetThreadStatusError(true);
                        pThis->m_2DStep = Calibration2DStep::SELFQUIT;
                        break;
                    }
                    ThreadSleep(TIME300MS);
                    for (retry = 0; retry < 5; ++retry)
                    {                        
                        res = gCatchMachCalMark(CameraNo, MarkType);
                        oneDOffset = Read1DCompensationData(_T("FY2"));
                        if (res.exe == 1)
                        {
                            LastOk = res;
                            pThis->Set2DBackup(CurNo, res);
                            break;
                        }
                    }
                    if (retry < 5)
                    {
                        if (pThis->Get2DMethod() == 1)
                        {
                            Apply.x = pt.x + res.x;
                            Apply.y = pt.y + res.y;
                        }
                        else
                        {
                            Apply = res;
                        }
                        pThis->Set2D(n, indy, Apply);
                        TRACE(_T("[PWR] X:%04d Y:%04d CurNo:%04d Pt(%.3f %.3f) ResXY %.3f %.3f Exe:%d Y2:%.3f\n"), n, indy, CurNo, pt.x, pt.y, res.x, res.y, res.exe, oneDOffset);
                    }
                    else // All Vision Error
                    {
                        if ((indy % 2) == 0) // ----> 방향으로 검사.
                        {	
                            if (n == 0) { // X 위치가 처음 값일 때 비전 에러 발생
                                before_x = CurNo - MaxMarkX;
                                // 미리 검사한 X Table이 있는 경우
                                if (before_x >= 0) 
                                {
                                    Apply = pThis->Get2DBackup(before_x);
                                    TRACE(_T("[PWR] => X:%04d Y:%04d CurNo:%04d BeforeNo:%04d Pt(%.3f %.3f) PreResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, before_x, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                                }
                                else {	// 첫번째 X Table인 경우, 바로 전 데이터를 사용한다.
                                    Apply = LastOk;
                                    TRACE(_T("[PWR] => X:%04d Y:%04d CurNo:%04d Pt(%.3f %.3f) FirstX BesideResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                                }
                            }
                            else {	// 그 전 값으로 더해준다.
                                Apply = LastOk;
                                TRACE(_T("[PWR] => X:%04d Y:%04d CurNo:%04d Pt(%.3f %.3f) BesideResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                            }
                        }
                        else // <------ 방향으로 검사.
                        {
                            if (n == (MaxMarkX - 1)) { // X 위치가 마지막 값일 때 비전 에러 발생
                                before_x = CurNo - MaxMarkX;
                                // 미리 검사한 X Table이 있는 경우
                                if (before_x >= 0) 
                                {
                                    Apply = pThis->Get2DBackup(before_x);
                                    TRACE(_T("[PWR] <= X:%04d Y:%04d CurNo:%04d BeforeNo:%04d Pt(%.3f %.3f) PreResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, before_x, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                                }
                                else {	// 첫번째 X Table인 경우, 바로 전 데이터를 사용한다.
                                    Apply = LastOk;
                                    TRACE(_T("[PWR] <= X:%04d Y:%04d CurNo:%04d Pt(%.3f %.3f) FirstX BesideResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                                }
                            }
                            else {	// 그 전 값으로 더해준다.
                                Apply = LastOk;
                                TRACE(_T("[PWR] <= X:%04d Y:%04d CurNo:%04d Pt(%.3f %.3f) BesideResXY %.3f %.3f Y2:%.3f\n"), n, indy, CurNo, pt.x, pt.y, Apply.x, Apply.y, oneDOffset);
                            }
                        }
                        pThis->Set2D(n, indy, Apply);
                    }
                }
            }
            if (Err == NO_ERR)
            {
                pThis->m_2DStep = Calibration2DStep::WRITE_2D;
            }
            break;

        case Calibration2DStep::WRITE_2D:
            pThis->Show2D(MaxMarkX, MaxMarkY);
            TRACE(_T("[PWR] End Show2D()\n"));
            pThis->CopyGlobal2DCompensationData(MaxMarkX, MaxMarkY);
            TRACE(_T("[PWR] End CopyGlobal2DCompensationData()\n"));
            pThis->Write2D();
            TRACE(_T("[PWR] End gcPowerCalibrationData->Write2DCompensationData()\n"));
            pThis->m_2DStep = Calibration2DStep::DISABLE_2D;
            break;

        case Calibration2DStep::DISABLE_2D:
            //pThis->Disable2D();
            //ThreadSleep(TIME100MS);
            pThis->m_2DStep = Calibration2DStep::SET_2D;
            break;

        case Calibration2DStep::SET_2D:
            //pThis->Set2D(MaxMarkX, MaxMarkY);
            //ThreadSleep(TIME100MS);
            pThis->m_2DStep = Calibration2DStep::ENABLE_2D;
            break;

        case Calibration2DStep::ENABLE_2D:
            //pThis->Enable2D();
            //ThreadSleep(TIME100MS);
            pThis->m_2DStep = Calibration2DStep::SELFQUIT;
            break;

        case Calibration2DStep::SELFQUIT:
            bLoop = false;
            break;

        case Calibration2DStep::ERROR_2D:
            if (pThis->GetThreadStatusError() == true)
            {
                if (pErrorTime->TimeElapsed() > TIME1000MS)
                {
                    pErrorTime->TimeGet();
                    TRACE(_T("[PWR] GetStatusError Over 1[Sec] Error Step:%d\n"), ErrorStep);                    
                }
            }
            else
            {
                pThis->m_2DStep = Calibration2DStep::WAIT_INIT;
            }
            break;

        case Calibration2DStep::WAIT_INIT:            
            break;

        case Calibration2DStep::MAX:
            Sleep(100);
            break;
        }
        if (pThis->IsThreadQuitMsg(strHostMsg) == false)
        {
            ZeroMemory(nArg, sizeof(nArg));
            strHostMsg.Empty();
        }
        ThreadSleep(THREAD_2D_CALIBRATION_READTIME);
    }
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    InitOneRatio(pThis->m_StrX);
    InitOneRatio(pThis->m_StrY);
	gLedAllOff();
    delete pTime;
    delete pErrorTime;
    pTime = NULL;
    TRACE(_T("[PWR] CStart2D(0x%x) Quit\n"), pThis->GetThreadID());
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CStart2D::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("th2DCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)Start2DCalibration;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] 2D Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] 2D Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

