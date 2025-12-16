#include "pch.h"
#include "CStartOffsetCamera.h"
#include "EthernetVision.h"
#include "AxisInformation.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CMachineConfig.h"

CStartOffsetCamera* gcStartOffsetCamera;
CStartOffsetCamera::CStartOffsetCamera()
{
    ClearStep();
    InitializeValue();
    m_Gantry = FRONT_GANTRY;
    m_StrX = GetAxisX(m_Gantry);
    m_StrY = GetAxisY1(m_Gantry);
}

CStartOffsetCamera::CStartOffsetCamera(HANDLE h_Terminate)
{
    ClearStep();
    InitializeValue();
    m_Gantry = FRONT_GANTRY;
    m_StrX = GetAxisX(m_Gantry);
    m_StrY = GetAxisY1(m_Gantry);
}

void CStartOffsetCamera::InitializeValue()
{
    m_Bdno = FRONT_VISION;
    m_Cam = FHCAM;	
	m_FovX = 18;        // mm
	m_FovY = 14;        // mm
	m_PixelX = 1280.0;  // pixel
	m_PixelY = 960.0;   // pixel
    m_FovPerPixelX = m_FovX / m_PixelX;
    m_FovPerPixelY = m_FovY / m_PixelY;
    m_AngleErr = 0.0;
    Point_XY ptCamJigPitch;
    ptCamJigPitch.x = 1.8 * -1.0;
    ptCamJigPitch.y = 1.8;
    SetJigPitchXY(ptCamJigPitch);
	SetCameraNo(FHCAM);

}

CStartOffsetCamera::~CStartOffsetCamera()
{
    ClearStep();
}

void CStartOffsetCamera::ClearStep()
{
    m_Step = CalibrationCameraStep::INIT;
}

void CStartOffsetCamera::SetCameraNo(long CamNo)
{
    m_Cam = CamNo;
    TRACE(_T("[PWR] CStartOffsetCamera SetCameraNo:%d\n"), m_Cam);

	int ver = 0;
	if (gCMachineConfig->GetCameraVersion(CamNo, &ver) == NO_ERR)
	{
		if (ver == 1)
		{
			m_FovX = 19.04;        // mm
			m_FovY = 15.23;        // mm
			m_PixelX = 1280.0;  // pixel
			m_PixelY = 1024.0;   // pixel
			m_FovPerPixelX = m_FovX / m_PixelX;
			m_FovPerPixelY = m_FovY / m_PixelY;
		}
	}

}

long CStartOffsetCamera::GetCameraNo()
{
    return m_Cam;
}

void CStartOffsetCamera::SetJigPitchXY(Point_XY xy)
{
    m_CalJigPitch.x = xy.x;
    m_CalJigPitch.y = xy.y;
}

double CStartOffsetCamera::GetJigPitchX()
{
    return m_CalJigPitch.x;
}

double CStartOffsetCamera::GetJigPitchY()
{
    return m_CalJigPitch.y;
}

void CStartOffsetCamera::MakeCalibrationJigInfo()
{
    unsigned inx, iny;
    m_MaxCalJigNo = 0;
    for (inx = 0; inx < 7; inx++) {	/* y dir */
        for (iny = 0; iny < 9; iny++) {	/* x dir */
            m_CalJigPos[inx * 9 + iny].x = GetJigPitchX() * (double)(4.0 - iny);
            m_CalJigPos[inx * 9 + iny].y = GetJigPitchY() * (double)(3.0 - inx);
            m_MaxCalJigNo++;
        }
    }
    TRACE(_T("[PWR] Max Calibration Jig No:%d\n"), m_MaxCalJigNo);
}

void CStartOffsetCamera::RotationPoint(Point_XY* pt, double AngleErr)
{
    Point_XY tmp;
    double ca, sa, angle;
    angle = AngleErr;
    angle = (PIE_180)*angle;
    ca = cos(angle);
    sa = sin(angle);
    tmp.x = pt->x;
    tmp.y = pt->y;
    pt->x = tmp.x * ca - tmp.y * sa;
    pt->y = tmp.x * sa + tmp.y * ca;
}

void CStartOffsetCamera::ReadStartPosition()
{
    m_CurPos = gReadGantryPosition(FRONT_GANTRY);
    TRACE(_T("[PWR] Start Calibration Position XY:%.3f %.3f\n"), m_CurPos.x, m_CurPos.y);
}

UINT CStartOffsetCamera::RunOffsetCameraCalibration(LPVOID wParam)
{
    bool bLoop = true;
    int err = ErrorCode::None;
    long lTimeChk = 0;
    int retVis = 0, ImageCatch = 1, VisErr = 0;
    long lMarkNo = 0;
    double CalRmsErr = 0.0, dblPosition = 0.0;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    CStartOffsetCamera* pThis = reinterpret_cast<CStartOffsetCamera*>(wParam);
    CalibrationCameraStep nOldStep = CalibrationCameraStep::MAX;
    CApplicationTime* pTime = new CApplicationTime();
    Point_XYRE res, delta;
    Point_XY Move;
    double Ratio = 0.1, Inpos = 0.001;
    long Ms = TIME300MS, TimeOut = TIME5000MS;
    long Cam = pThis->GetCameraNo();
    Move.x = Move.y = delta.x = delta.y = 0.000;
    do
    {
        if (nOldStep != pThis->m_Step)
        {
            lTimeChk = pTime->TimeElapsed();
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] StartOffsetCameraCalibration Step:%d Time:%d[ms]\n"), pThis->m_Step, lTimeChk);
            }
            nOldStep = pThis->m_Step;
            pTime->TimeGet();
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_OFFSETCAMERA_CALIBRATION_READTIME);
        //    continue;
        //}
        switch (pThis->m_Step)
        {
        case CalibrationCameraStep::INIT:
            pThis->m_Step = CalibrationCameraStep::START;
            break;

        case CalibrationCameraStep::START:
            pThis->m_Step = CalibrationCameraStep::CHECK_ORIGIN;
            break;

        case CalibrationCameraStep::CHECK_ORIGIN:
            if (IsOneAxisHomingComplete(pThis->m_StrX) == true && IsOneAxisHomingComplete(pThis->m_StrY) == true)
            {
                pThis->m_Step = CalibrationCameraStep::SET_CENTERMARK;
            }
            else
            {
                TRACE(_T("[PWR] Homing Failed %s(%d) %s(%d)\n"), pThis->m_StrX, IsOneAxisHomingComplete(pThis->m_StrX), pThis->m_StrY, IsOneAxisHomingComplete(pThis->m_StrY));
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            break;

        case CalibrationCameraStep::SET_CENTERMARK:
            lMarkNo = SetMarkCameraCalibrationJigCenterMark(Cam);
            pThis->m_Step = CalibrationCameraStep::CATCH_CENTERMARK;
            break;

        case CalibrationCameraStep::CATCH_CENTERMARK:
            res = markTrainingWithoutCamCal(Cam, lMarkNo, 0, 0, 1);
            retVis = gGetVisionCommandResult(FRONT_VISION);
            if (retVis != 0)
            {
                VisErr = gGetVisionErrorCode(FRONT_VISION);
                TRACE(_T("[PWR] markTrainingWithoutCamCal Ret:%d Err:%d\n"), retVis, VisErr);
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            else
            {
                res = GetVisionResult(pThis->m_Bdno, 0);
                delta.x = res.x * ((pThis->m_FovX * (pThis->m_PixelX / pThis->m_PixelY)) / pThis->m_PixelX);
                delta.y = res.y * (pThis->m_FovY / pThis->m_PixelY);
                TRACE(_T("[PWR] Mark ResultXY Old %.3f,%.3f\n"), delta.x, delta.y);
                ThreadSleep(TIME500MS);
                pThis->m_Step = CalibrationCameraStep::MOVE_CENTERMARK;
            }
            break;

        case CalibrationCameraStep::MOVE_CENTERMARK:
            pThis->ReadStartPosition();
            TRACE(_T("[PWR] Mark ResultXY New %.3f,%.3f\n"), delta.x, delta.y);
            if (abs(delta.x) > 0.200 || abs(delta.y) > 0.200)
            {
                TRACE(_T("[PWR] CalibrationOffsetCameraStep:%d Mark Error:%d\n"), pThis->m_Step, err);
                pThis->SetThreadStatusError(true);
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            else if (abs(delta.x) > 0.005 || abs(delta.y) > 0.005)
            {
                Move.x = pThis->m_CurPos.x + delta.x;
                Move.y = pThis->m_CurPos.y - delta.y;
                LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, Move, Ratio, Inpos, Ms, TimeOut);
                pThis->m_Step = CalibrationCameraStep::CATCH_CENTERMARK;
            }
            else
            {
                pThis->m_Step = CalibrationCameraStep::MAKE_JIGINFO;
            }
            break;

        case CalibrationCameraStep::MAKE_JIGINFO:
            pThis->MakeCalibrationJigInfo();
            pThis->m_Step = CalibrationCameraStep::READ_STARTPOSITION;
            break;

        case CalibrationCameraStep::READ_STARTPOSITION:
            pThis->ReadStartPosition();
            pThis->m_Step = CalibrationCameraStep::MOVE_STARTPOSITION;
            break;

        case CalibrationCameraStep::MOVE_STARTPOSITION:
            Move.x = pThis->m_CurPos.x;
            Move.y = pThis->m_CurPos.y;
            LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, Move, Ratio, Inpos, Ms, TimeOut);
            pThis->m_Step = CalibrationCameraStep::ROTATE_JIGPOINT;
            break;

        case CalibrationCameraStep::ROTATE_JIGPOINT:
            TRACE(_T("[PWR] CalJigPos m_AngleErr:%.3f\n"), pThis->m_AngleErr);
            for (unsigned inx = 0; inx < pThis->m_MaxCalJigNo; ++inx)
            {
                TRACE(_T("[PWR] CalJigPos Old indx(%02d,%.3f,%.3f)\n"), inx, pThis->m_CalJigPos[inx].x, pThis->m_CalJigPos[inx].y);
                pThis->RotationPoint(&pThis->m_CalJigPos[inx], pThis->m_AngleErr);
                TRACE(_T("[PWR] CalJigPos New indx(%02d,%.3f,%.3f)\n"), inx, pThis->m_CalJigPos[inx].x, pThis->m_CalJigPos[inx].y);
            }
            pThis->m_Step = CalibrationCameraStep::CATCH_JIGPOINT;
            break;

        case CalibrationCameraStep::CATCH_JIGPOINT:
            retVis = CameraCalibration(Cam, pThis->m_MaxCalJigNo, pThis->m_CalJigPos);
            pThis->m_Step = CalibrationCameraStep::GET_CALJIGRESULT;
            break;

        case CalibrationCameraStep::GET_CALJIGRESULT:
            CalRmsErr = GetVisionResultDouble(FRONT_VISION, 3);
            TRACE(_T("[PWR] CalRmsErr:%f\n"), CalRmsErr);
            pThis->m_Step = CalibrationCameraStep::GET_CALJIGINFO;
            break;

        case CalibrationCameraStep::GET_CALJIGINFO:
            for (int inx = 0; inx < 32; inx++)
            {
                Camera[Cam].NewCamCal[inx] = gcEthernetVision->GetVisionResultDouble(FRONT_VISION, 4 + inx);
            }
            pThis->m_Step = CalibrationCameraStep::UPDATE_CALJIGINFO;
            break;

        case CalibrationCameraStep::UPDATE_CALJIGINFO:
            ThreadSleep(THREAD_VISION_UPDATE_DATE_TIME);
            pThis->m_Step = CalibrationCameraStep::MOVE_ROTATESTARTPOINT;
            break;

        case CalibrationCameraStep::MOVE_ROTATESTARTPOINT:
            dblPosition = (2.0 * pThis->GetJigPitchX());
            StartMoveWaitDelayedInposition(pThis->m_StrX, Ratio, TimeOut, dblPosition, Inpos, Ms, true);
            //TRACE(_T("[PWR] Start to MOVE_X_JIGPITCH_X %.3f\n"), dblPosition);
            //err = pThis->pAxisX->StartMove(posCommandX, dblPosition);
            //if (err != ErrorCode::None)
            //{
            //    TRACE(_T("[PWR] CalibrationOffsetCameraStep:%d StartMove Error:%d\n"), pThis->m_Step, err);
            //    pThis->SetThreadStatusError(true);
            //    pThis->m_Step = CalibrationOffsetCameraStep::SELFQUIT;
            //    break;
            //}
            //pThis->pAxisX->WaitMotion();
            ThreadSleep(THREAD_VISION_CALIBRATION_WAITTIME);
            pThis->m_Step = CalibrationCameraStep::READY_ROTATEJIGPOINT;
            break;

        case CalibrationCameraStep::READY_ROTATEJIGPOINT:
            retVis = CameraRotateCalibrationPrepare(Cam, pThis->GetJigPitchX());
            if (retVis != 0)
            {
                VisErr = gGetVisionErrorCode(FRONT_VISION);
                TRACE(_T("[PWR] CameraRotateCalibrationPrepare Ret:%d Err:%d\n"), retVis, VisErr);
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            else
            {
                pThis->m_Step = CalibrationCameraStep::CATCH_ROTATEJIGPOINT;
            }
            break;

        case CalibrationCameraStep::CATCH_ROTATEJIGPOINT:
            retVis = CameraRotateCalibration(Cam, ImageCatch);
            if (retVis != 0)
            {
                VisErr = gGetVisionErrorCode(FRONT_VISION);
                TRACE(_T("[PWR] CameraRotateCalibration Catch(%d) Ret:%d Err:%d\n"), ImageCatch, retVis, VisErr);
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            else
            {
                pThis->m_Step = CalibrationCameraStep::MOVE_X_JIGPITCH_X;
            }
            break;

        case CalibrationCameraStep::MOVE_X_JIGPITCH_X:
            dblPosition = pThis->GetJigPitchX() * -1;
            StartMoveWaitDelayedInposition(pThis->m_StrX, Ratio, TimeOut, dblPosition, Inpos, Ms, true);
            ThreadSleep(THREAD_VISION_CALIBRATION_WAITTIME);
            ImageCatch++;            
            if (ImageCatch > 5)
            {
                pThis->m_Step = CalibrationCameraStep::GET_ROTATEJIGRESULT;
            }
            else
            {
                pThis->m_Step = CalibrationCameraStep::CATCH_ROTATEJIGPOINT;
            }
            break;

        case CalibrationCameraStep::GET_ROTATEJIGRESULT:
            retVis = gcEthernetVision->CameraRotateCalibration(FHCAM, ImageCatch);
            if (retVis != 0)
            {
                VisErr = gGetVisionErrorCode(FRONT_VISION);
                TRACE(_T("[PWR] CameraRotateCalibration Catch(%d) Ret:%d Err:%d\n"), ImageCatch, retVis, VisErr);
                pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            }
            else
            {
                pThis->m_AngleErr = GetVisionResultDouble(FRONT_VISION, 3);
                if (pThis->m_AngleErr >= 0.005 || pThis->m_AngleErr <= -0.005)
                {
                    ImageCatch = 1;
                    TRACE(_T("[PWR] Angle Error:%f\n"), pThis->m_AngleErr);
                    pThis->m_Step = CalibrationCameraStep::MOVE_STARTPOSITION;
                }
                else
                {
                    TRACE(_T("[PWR] CameraNo:%d Spec In:%.3f\n"), Cam, pThis->m_AngleErr);
                    pThis->m_Step = CalibrationCameraStep::MOVE_ENDPOSITION;
                }
            }
            break;

        case CalibrationCameraStep::MOVE_ENDPOSITION:
            Move.x = pThis->m_CurPos.x;
            Move.y = pThis->m_CurPos.y;
            LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, Move, Ratio, Inpos, Ms, TimeOut);
            pThis->m_Step = CalibrationCameraStep::SELFQUIT;
            break;

        case CalibrationCameraStep::SELFQUIT:
            bLoop = false;
            break;

        case CalibrationCameraStep::MAX:
            Sleep(TIME100MS);
            break;
        }
        if (bLoop == false)
        {
            break;
        }
    } while (pThis->IsTerminated(THREAD_OFFSETCAMERA_CALIBRATION_READTIME) == false);
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    InitOneRatio(pThis->m_StrX);
    InitOneRatio(pThis->m_StrY);
    delete pTime;
    pTime = NULL;
    TRACE(_T("[PWR] CStartOffsetCamera(0x%x) Quit\n"), pThis->GetThreadID());
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CStartOffsetCamera::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thOffsetCameraCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)RunOffsetCameraCalibration;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] OffsetCamera Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] OffsetCamera Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
