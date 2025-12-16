#include "pch.h"
#include "CStartAlignOffset.h"
#include "GlobalDefine.h"
#include "vision.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "CPowerLog.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
//#include "ErrorCode.h"

CStartAlignOffset* gcStartAlignOffset;
CStartAlignOffset::CStartAlignOffset()
{
    m_Gantry = FRONT_GANTRY;
    m_StrX = GetAxisX(m_Gantry);
    m_StrY = GetAxisY1(m_Gantry);
    m_RepeatCount = 0;
    m_bApply = false;
    ClearStep();
    SetAlignOffsetCamera(CAM1);    
}

CStartAlignOffset::~CStartAlignOffset()
{
}

void CStartAlignOffset::SetRepeatCount(int RepeatCount)
{
    m_RepeatCount = RepeatCount;
    TRACE(_T("[PWR] SetRepeatCount(%d)\n"), RepeatCount);
}

long CStartAlignOffset::GetRepeatCount()
{
    return m_RepeatCount;
}

void CStartAlignOffset::ClearStep()
{
    m_Step = CalibrationAlignOffsetStep::INIT;
}

void CStartAlignOffset::SetAlignOffsetMode(bool bApply)
{
    m_bApply = bApply;
    TRACE(_T("[PWR] SetAlignOffsetMode(%s)\n"), bApply == true ? _T("Real") : _T("Simulation"));
}

bool CStartAlignOffset::GetAlignOffsetMode()
{
    return m_bApply;
}

void CStartAlignOffset::SetAlignOffsetCamera(long CameraNo)
{
    m_Camera = CameraNo;
    TRACE(_T("[PWR] SetAlignOffsetCamera(%d)\n"), m_Camera);
}

long CStartAlignOffset::GetAlignOffsetCamera()
{
    return m_Camera;
}

void CStartAlignOffset::SetCameraAlignPosition(Point_XY pt)
{
    m_CameraAlign = pt;
}

bool CStartAlignOffset::CopyCameraAlignPosition()
{
    Point_XY CameraAlign;
    long AlignCameraNo;
    if (GetAlignOffsetMode() == true)
    {
        CameraAlign = m_CameraAlign;
        AlignCameraNo = m_Camera;
        gcPowerCalibrationData->SetCameraAlignPosition(FRONT_GANTRY, CameraAlign);
        gcPowerCalibrationData->SetAlignCamera(FRONT_GANTRY, AlignCameraNo);
        TRACE(_T("[PWR] CopyCameraAlignPosition Cam:%d XY:%.3f %.3f\n"), AlignCameraNo, CameraAlign.x, CameraAlign.y);
        return true;
    }
    else
    {
        return false;
    }
}

bool CStartAlignOffset::WriteCameraAlignPosition()
{
    if (GetAlignOffsetMode() == true)
    {
        gcPowerCalibrationData->WriteCameraAlignPosition(FRONT_GANTRY);
        return true;
    }
    else
    {
        return false;
    }
}

long CStartAlignOffset::StartAlignOffsetCalibration()
{
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    long TimeChk = 0, indx = 0, Count = 0, RepeatCount = 0, Err = NO_ERR;
    CString strHostMsg;

    CalibrationAlignOffsetStep OldStep = CalibrationAlignOffsetStep::MAX, ErrorStep = CalibrationAlignOffsetStep::MAX;

    Point_XY Start, Compen;
    Point_XYRE Res, Sum, Mean;
    Point_XY Save;
    long LedVal = 50;
    ZeroMemory(&Start, sizeof(Start));
    ZeroMemory(&Compen, sizeof(Compen));
    ZeroMemory(&Res, sizeof(Res));
    ZeroMemory(&Sum, sizeof(Sum));
    ZeroMemory(&Mean, sizeof(Mean));
    ZeroMemory(&Save, sizeof(Save));

	long HeadCam = FHCAM;
	long AlignCam = GetAlignOffsetCamera();
	Save = gcPowerCalibrationData->GetCameraAlignPosition(FRONT_GANTRY);

    Start = gReadGantryPosition(FRONT_GANTRY);
    double Ratio = 0.3, Inpos = 0.001;
    long TimeOut = TIME10000MS, Ms = TIME300MS;
    if (GetAlignOffsetMode() == false)
    {
        Ratio = 0.5;
    }
    RepeatCount = GetRepeatCount();
    if (RepeatCount < 1)
    {
        RepeatCount = 1;
    }
    if (IsOneAxisHomingComplete(m_StrX) == false || IsOneAxisHomingComplete(m_StrY) == false)
    {
		long axis = 0;
		if (IsOneAxisHomingComplete(m_StrX) == false)
		{
			axis = GetAxisIndexFromAliasName(m_StrX);
		}
		else
		{
			axis = GetAxisIndexFromAliasName(m_StrY);
		}
		Err = HOMING_FAIL(axis);

		TRACE(_T("[PWR] CStartAlignOffset Homing Failed %s(%d) %s(%d)\n"), m_StrX, IsOneAxisHomingComplete(m_StrX), m_StrY, IsOneAxisHomingComplete(m_StrY));
		return Err;
    }
    //gLedOn(FHCAM, LedVal, LedVal, 0);
    //gLedOn(pThis->GetAlignOffsetCamera(), LedVal, LedVal, LedVal);
    //ThreadSleep(TIME500MS);
    ReliabilityInit();
    TRACE(_T("[PWR] ReliabilityInit Count:%d\n"), RepeatCount);
    for (long cnt = 0; cnt < RepeatCount; ++cnt)
    {
		gLedOn(HeadCam, 100, 150, 0);
		gLedOn(AlignCam, 0, 0, 0);
		ThreadSleep(TIME500MS);

        TRACE(_T("[PWR] ReliabilityInit cnt:%d\n"), cnt);
        Compen = Start;
        for (indx = 0; indx < CAL_ALIGNOFFSET_TRY_COUNT; ++indx)
        {
            Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, TBL_CAMERA, Compen, Ratio, Inpos, Ms, TimeOut);
            if(Err != NO_ERR)
            {
                TRACE(_T("[PWR] CStartAlignOffset LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
                return 1;
            }
            ThreadSleep(TIME100MS);
            Res = gCatchMark(FHCAM, ALIGNMARKWHT);
            if (Res.exe == 1)
            {
                Compen.x += Res.x;
                Compen.y += Res.y;
            }
            if ((fabs(Res.x) < CAL_ALIGNOFFSET_MAX) && (fabs(Res.y) < CAL_ALIGNOFFSET_MAX))
            {
                if (gcPowerLog->IsShowCalibrationLog() == true)
                {
                    TRACE(_T("[PWR] Count(%02d) : Align Jig Position of Head Cam (%.4f,%.4f) - OK\n"), indx + 1, Compen.x, Compen.y);
                }
                break;
            }
            else
            {
                if (gcPowerLog->IsShowCalibrationLog() == true)
                {
                    TRACE(_T("[PWR] Count(%02d) : Align Jig Position of Head Cam (%.4f,%.4f) - NG\n"), indx + 1, Compen.x, Compen.y);
                }
            }
        }
        if (indx == CAL_ALIGNOFFSET_TRY_COUNT)
        {
			TRACE(_T("[PWR] Align Error Head Cam Recog NG\n"));
			return PCB_FIDUCIALMARK_RECOGNITION;
        }
        Count = 0;
        Sum.x = Sum.y = 0.0;

		gLedOn(HeadCam, 0, 0, 0);
		gLedOn(AlignCam, 100, 100, 0);
		ThreadSleep(TIME500MS);

        for (indx = 0; indx < CAL_ALIGNOFFSET_MODULE_COUNT; ++indx)
        {
            Res = gCatchMark(GetAlignOffsetCamera(), ALIGNMARKWHTCAM2);
            if (Res.exe == 1)
            {
                Sum.x += Res.x;
                Sum.y += Res.y;
                Count++;
            }
            if (gcPowerLog->IsShowCalibrationLog() == true)
            {
                TRACE(_T("[PWR] Count(%02d) Cam%02d has recognized the result %.3f %.3f\n"), indx + 1, GetAlignOffsetCamera(), Res.x, Res.y);
            }
			ThreadSleep(TIME100MS);
        }

		if (Count < (CAL_ALIGNOFFSET_MODULE_COUNT / 2))
		{
			TRACE(_T("[PWR] Align Error Module Cam Recog NG\n"));

			return PCB_FIDUCIALMARK_RECOGNITION;
		}

        Mean.x = Sum.x / Count; // CAL_ALIGNOFFSET_MODULE_COUNT;
        Mean.y = Sum.y / Count; // CAL_ALIGNOFFSET_MODULE_COUNT;
        if (gcPowerLog->IsShowCalibrationLog() == true)
        {
            TRACE(_T("[PWR] CAM%02d Avr DeltaXY %.3f %.3f"), GetAlignOffsetCamera(), Mean.x, Mean.y);
        }
        Save.x = Compen.x + Mean.x;
        Save.y = Compen.y + Mean.y;
        if (gcPowerLog->IsShowCalibrationLog() == true)
        {
            TRACE(_T("[PWR] *******************************************************\n"));
            TRACE(_T("[PWR] [ALIGN] Camera Align Position(%7.3f,%7.3f) is GOOD !!!\n"), Save.x, Save.y);
            TRACE(_T("[PWR] [ALIGN] Offset was saved !!!\n"));
            TRACE(_T("[PWR] *******************************************************\n"));
        }
        ReliabilityRaw2(cnt, Save.x, Save.y);
        SetAlignOffsetCamera(GetAlignOffsetCamera());
        SetCameraAlignPosition(Save);                
        CopyCameraAlignPosition();
        WriteCameraAlignPosition();
    }
    if (RepeatCount > 1)
    {
        ReliabilityMakeStdev(_T("RepeatCameraAlign"), RepeatCount);
    }
    gLedAllOff();
    TRACE(_T("[PWR] CStartAlignOffset(0x%X) Quit\n"), GetThreadID());

    return NO_ERR;
}

void CStartAlignOffset::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetThreadName(_T("thAlignOffsetCalibration"));
    lpStartAddress = (_beginthreadex_proc_type)RunThread;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] AlignOffset Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] AlignOffset Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
    
}

UINT CStartAlignOffset::RunThread(LPVOID wParam)
{
	CStartAlignOffset* pThis = reinterpret_cast<CStartAlignOffset*>(wParam);
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
        Err = pThis->StartAlignOffsetCalibration();
        SetMachineManualActionRunninSign(false);
    }
    /*Err = pThis->StartAlignOffsetCalibration();*/

	CString strMsg;

	strMsg.Format(_T("%d,%d"), Err, pThis->m_Gantry);
	SendToHMI(HMI_CMD1ST_3, HMI_CMD2ND_110, HMI_CMD3RD_01, strMsg);

	return Err;

}