#include "pch.h"
#include "CStartCalibrationFunc.h"
#include "AxisInformation.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CHomeStatus.h"
#include "CStart1D.h"
#include "CStart2D.h"
#include "CStartAlignOffset.h"
#include "CStartHeadOffset.h"
#include "CStartModuleCamera.h"
#include "CStartOffsetCamera.h"
#include "COriginSearch.h"
#include "CStartZCompen.h"
#include "CStartROriginOffset.h"
#include "CPowerLog.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CStartCalibrationFunc* gcStartCalibrationFunc;
CStartCalibrationFunc::CStartCalibrationFunc()
{
    m_Step = CalibrationStep::CAL_INIT;
    ZeroMemory(&m_id, sizeof(m_id));
    GetId(&m_id);
}

void CStartCalibrationFunc::SetStep(CalibrationStep nStep)
{
    m_Step = nStep;
    TRACE(_T("[PWR] CStartCalibrationFunc::SetStep:%d\n"), nStep);
}

long CStartCalibrationFunc::GetThreadID()
{
    long retID;
    GetId(&m_id);
    retID = (long)m_id;
    return retID;
}

CalibrationStep CStartCalibrationFunc::GetStep()
{
    return m_Step;
}

CStartCalibrationFunc::~CStartCalibrationFunc()
{
    TRACE(_T("[PWR] ~CStartCalibrationFunc(0x%x)\n"), m_id);
    StopCalibrationFunc();
    TRACE(_T("[PWR] ~End CStartCalibrationFunc\n"));
}

BOOL CStartCalibrationFunc::OnTask()
{
    TRACE("[PWR] CStartCalibrationFunc::OnTask Thread(0x%04X)\n", m_id);
    return TRUE;
}

BOOL CStartCalibrationFunc::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    CalibrationStep CalStep = CalibrationStep::CAL_INIT;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CStartCalibrationFunc GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        CalStep = CheckCalibrationStep(strMsg);
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        //dont' use cout here, output could be broken up due to threading
        TRACE("[PWR] CStartCalibrationFunc(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);

        switch (CalStep)
        {
        case CalibrationStep::CAL_INIT:
            break;
        case CalibrationStep::CAL_ORIGINSEARCH_START:
        case CalibrationStep::CAL_ORIGINSEARCH_DOING:
        case CalibrationStep::CAL_ORIGINSEARCH_PAUSE:
        case CalibrationStep::CAL_ORIGINSEARCH_ERROR:
        case CalibrationStep::CAL_ORIGINSEARCH_END:
            SetStep(CalStep);
            StartOriginSearch(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_1D_START:
        case CalibrationStep::CAL_1D_DOING:
        case CalibrationStep::CAL_1D_PAUSE:
        case CalibrationStep::CAL_1D_ERROR:
        case CalibrationStep::CAL_1D_END:
        case CalibrationStep::CAL_1D_ON:
        case CalibrationStep::CAL_1D_OFF:
            SetStep(CalStep);
            Start1DCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_OFFSET_CAM_START:
        case CalibrationStep::CAL_OFFSET_CAM_DOING:
        case CalibrationStep::CAL_OFFSET_CAM_PAUSE:
        case CalibrationStep::CAL_OFFSET_CAM_ERROR:
        case CalibrationStep::CAL_OFFSET_CAM_END:
            SetStep(CalStep);
            StartOffsetCameraCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_MODULE_CAM_START:
        case CalibrationStep::CAL_MODULE_CAM_DOING:
        case CalibrationStep::CAL_MODULE_CAM_PAUSE:
        case CalibrationStep::CAL_MODULE_CAM_ERROR:
        case CalibrationStep::CAL_MODULE_CAM_END:
            SetStep(CalStep);
            StartModuleCameraCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_2D_START:
        case CalibrationStep::CAL_2D_DOING:
        case CalibrationStep::CAL_2D_PAUSE:
        case CalibrationStep::CAL_2D_ERROR:
        case CalibrationStep::CAL_2D_ON:
        case CalibrationStep::CAL_2D_OFF:
        case CalibrationStep::CAL_2D_END:
            SetStep(CalStep);
            Start2DCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_ALIGN_START:
        case CalibrationStep::CAL_ALIGN_DOING:
        case CalibrationStep::CAL_ALIGN_PAUSE:
        case CalibrationStep::CAL_ALIGN_ERROR:
        case CalibrationStep::CAL_ALIGN_END:
            SetStep(CalStep);
            StartAlignOffsetCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_HEADOFFSET_START:
        case CalibrationStep::CAL_HEADOFFSET_DOING:
        case CalibrationStep::CAL_HEADOFFSET_PAUSE:
        case CalibrationStep::CAL_HEADOFFSET_ERROR:
        case CalibrationStep::CAL_HEADOFFSET_END:
            SetStep(CalStep);
            StartHeadOffsetCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_Z_START:
        case CalibrationStep::CAL_Z_DOING:
        case CalibrationStep::CAL_Z_PAUSE:
        case CalibrationStep::CAL_Z_ERROR:
        case CalibrationStep::CAL_Z_END:
        case CalibrationStep::CAL_Z_ON:
        case CalibrationStep::CAL_Z_OFF:
            SetStep(CalStep);
            StartZCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        case CalibrationStep::CAL_R_START:
        case CalibrationStep::CAL_R_DOING:
        case CalibrationStep::CAL_R_PAUSE:
        case CalibrationStep::CAL_R_ERROR:
        case CalibrationStep::CAL_R_END:
        case CalibrationStep::CAL_R_ON:
        case CalibrationStep::CAL_R_OFF:
            SetStep(CalStep);
            StartROriginOffsetCalibration(CalStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
            break;

        default:
            break;
        }
        delete msgReceived;
    }
    return TRUE;
}

void CStartCalibrationFunc::Run()
{
}

CalibrationStep CStartCalibrationFunc::CheckCalibrationStep(CString strHostMsg)
{
    CalibrationStep retCalStep = CalibrationStep::CAL_INIT;
    if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_1D_START;
    else if(strHostMsg.CompareNoCase(_T(STRING_1D_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_1D_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_1D_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_1D_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_1D_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_1D_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_1D_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_1D_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_OFFSETCAM_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_OFFSET_CAM_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_MODULECAM_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_MODULE_CAM_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_2D_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_2D_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_2D_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_2D_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_2D_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_2D_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_2D_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_2D_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_ALIGN_OFFSET_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_ALIGN_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_HEAD_OFFSET_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_HEADOFFSET_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_Z_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_Z_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_Z_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_Z_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_Z_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_Z_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_Z_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_Z_OFF;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_START)) == 0)
        retCalStep = CalibrationStep::CAL_R_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_PAUSE)) == 0)
        retCalStep = CalibrationStep::CAL_R_PAUSE;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_RESUME)) == 0)
        retCalStep = CalibrationStep::CAL_R_DOING;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_ERROR)) == 0)
        retCalStep = CalibrationStep::CAL_R_ERROR;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_END)) == 0)
        retCalStep = CalibrationStep::CAL_R_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_ON)) == 0)
        retCalStep = CalibrationStep::CAL_R_ON;
    else if (strHostMsg.CompareNoCase(_T(STRING_R_CAL_OFF)) == 0)
        retCalStep = CalibrationStep::CAL_R_OFF;
    return retCalStep;
}

void CStartCalibrationFunc::StopCalibrationFunc()
{
    TRACE(_T("[PWR] StopCalibrationFunc\n"));
}

void CStartCalibrationFunc::StartOriginSearch(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    THREAD_STRUCT threadInfo;
    Cwmx3Axis* pAxis;
    CString strAxis;
    int iTable = nSub1;
    int iAxis = nSub2;
    int iOption = nSub3;
    pAxis = GetWmx3AxisByIndex(iAxis);            // Each Axis
    strAxis = pAxis->GetAxisName();
    COriginSearch* pOriginSearch = new COriginSearch(strAxis);
    switch (CalStep)
    {
    case CalibrationStep::CAL_ORIGINSEARCH_START:
        if (pAxis)
        {
            pAxis->SetInitializeEnd(false);
            pAxis->SetInitializeFail(false);
            pOriginSearch->Run();
        }
        break;
    //case CalibrationStep::CAL_ORIGINSEARCH_DOING:
    //    if (pAxis)
    //    {
    //        pOriginSearch->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_ORIGINSEARCH_PAUSE:
    //    if (pAxis)
    //    {
    //        pOriginSearch->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_ORIGINSEARCH_ERROR:
        if (pAxis)
        {
            pOriginSearch->SetThreadStatusError(true);
        }
        break;
    case CalibrationStep::CAL_ORIGINSEARCH_END:
        if (pAxis)
        {
            TRACE(_T("[PWR] %s Origin Searching End\n"), strAxis);
        }
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::Start1DCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_1D_START:
        if (gcStart1D != NULL)
        {
            gcStart1D = NULL;
        }
        gcStart1D = new CStart1D();
        if (nSub3 == 1) // Simulation
        {
            gcStart1D->Set1DCompenMode(false); // Don't apply
        }
        else
        {
            gcStart1D->Set1DCompenMode(true); // Apply
        }
        gcStart1D->Run();
        break;
    //case CalibrationStep::CAL_1D_DOING:
    //    if (gcStart1D)
    //    {
    //        gcStart1D->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_1D_PAUSE:
    //    if (gcStart1D)
    //    {
    //        gcStart1D->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_1D_END:
        if (gcStart1D)
        {
            delete gcStart1D;
            gcStart1D = NULL;
        }
        break;
    case CalibrationStep::CAL_1D_ON:
        oneDCompensationOn();
        break;
    case CalibrationStep::CAL_1D_OFF:
        oneDCompensationOff();
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::StartOffsetCameraCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_OFFSET_CAM_START:
        if (gcStartOffsetCamera != NULL)
        {
            gcStartOffsetCamera = NULL;
        }
        gcStartOffsetCamera = new CStartOffsetCamera();
        gcStartOffsetCamera->Run();
        break;
    //case CalibrationStep::CAL_OFFSET_CAM_DOING:
    //    if (gcStartOffsetCamera)
    //    {
    //        gcStartOffsetCamera->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_OFFSET_CAM_PAUSE:
    //    if (gcStartOffsetCamera)
    //    {
    //        gcStartOffsetCamera->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_OFFSET_CAM_END:
        if (gcStartOffsetCamera)
        {
            delete gcStartOffsetCamera;
            gcStartOffsetCamera = NULL;
        }
        break;
    case CalibrationStep::CAL_OFFSET_CAM_ON:
        break;
    case CalibrationStep::CAL_OFFSET_CAM_OFF:
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::Start2DCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_2D_START:
        if (gcStart2D != NULL)
        {
            gcStart2D = NULL;
        }
        gcStart2D = new CStart2D();
        if (nSub2 == 1)
        {
            gcStart2D->Set2DMethod(1);  // Software
        }
        else
        {
            gcStart2D->Set2DMethod(0);  // WMX3
        }
        if (nSub3 == 1) // Simulation
        {
            gcStart2D->Set2DCompenMode(false); // Don't apply
        }
        else
        {
            gcStart2D->Set2DCompenMode(true); // Apply
        }
        gcStart2D->Run();
        break;
    //case CalibrationStep::CAL_2D_DOING:
    //    if (gcStart2D)
    //    {
    //        gcStart2D->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_2D_PAUSE:
    //    if (gcStart2D)
    //    {
    //        gcStart2D->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_2D_END:
        if (gcStart2D)
        {
            delete gcStart2D;
            gcStart2D = NULL;
        }
        break;
    case CalibrationStep::CAL_2D_ON:
        if (nSub2 == 1)
        {
            Set2DCompensationMethod(1);  // Software
        }
        else
        {
            Set2DCompensationMethod(0);  // WMX3
        }
        twoDCompensationOn();
        break;
    case CalibrationStep::CAL_2D_OFF:
        if (nSub2 == 1)
        {
            Set2DCompensationMethod(1);  // Software
        }
        else
        {
            Set2DCompensationMethod(0);  // WMX3
        }
        twoDCompensationOff();
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::StartModuleCameraCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_MODULE_CAM_START:
        if (gcStartModuleCamera != NULL)
        {
            gcStartModuleCamera = NULL;
        }
        gcStartModuleCamera = new CStartModuleCamera();
        gcStartModuleCamera->SetCameraNo(nSub2);
        gcStartModuleCamera->Run();
        break;
    //case CalibrationStep::CAL_MODULE_CAM_DOING:
    //    if (gcStartModuleCamera)
    //    {
    //        gcStartModuleCamera->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_MODULE_CAM_PAUSE:
    //    if (gcStartModuleCamera)
    //    {
    //        gcStartModuleCamera->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_MODULE_CAM_END:
        if (gcStartModuleCamera)
        {
            delete gcStartModuleCamera;
            gcStartModuleCamera = NULL;
        }
        break;
    case CalibrationStep::CAL_MODULE_CAM_ON:
        break;
    case CalibrationStep::CAL_MODULE_CAM_OFF:
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::StartAlignOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_ALIGN_START:
        if (gcStartAlignOffset != NULL)
        {
            gcStartAlignOffset = NULL;
        }
        gcStartAlignOffset = new CStartAlignOffset();
        gcStartAlignOffset->SetRepeatCount(nSub3);
        gcStartAlignOffset->SetAlignOffsetMode(true);
        gcStartAlignOffset->Run();
        break;
    //case CalibrationStep::CAL_ALIGN_DOING:
    //    if (gcStartAlignOffset)
    //    {
    //        gcStartAlignOffset->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_ALIGN_PAUSE:
    //    if (gcStartAlignOffset)
    //    {
    //        gcStartAlignOffset->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_ALIGN_END:
        if (gcStartAlignOffset)
        {
            delete gcStartAlignOffset;
            gcStartAlignOffset = NULL;
        }
        break;
    case CalibrationStep::CAL_ALIGN_ON:
        break;
    case CalibrationStep::CAL_ALIGN_OFF:
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::StartHeadOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_HEADOFFSET_START:
        if (gcStartHeadOffset != NULL)
        {
            gcStartHeadOffset = NULL;
        }
        gcStartHeadOffset = new CStartHeadOffset();
        gcStartHeadOffset->SetRepeatCount(nSub3);
        gcStartHeadOffset->SetHeadOffsetMode(true);
        gcStartHeadOffset->SetHeadOffsetMode(true);
        gcStartHeadOffset->SetCalibrationHead(nSub2);
        gcStartHeadOffset->Run();
        break;
    //case CalibrationStep::CAL_HEADOFFSET_DOING:
    //    if (gcStartHeadOffset)
    //    {
    //        gcStartHeadOffset->Resume();
    //    }
    //    break;
    //case CalibrationStep::CAL_HEADOFFSET_PAUSE:
    //    if (gcStartHeadOffset)
    //    {
    //        gcStartHeadOffset->Pause();
    //    }
    //    break;
    case CalibrationStep::CAL_HEADOFFSET_END:
        if (gcStartHeadOffset)
        {
            delete gcStartHeadOffset;
            gcStartHeadOffset = NULL;
        }
        break;
    case CalibrationStep::CAL_HEADOFFSET_ON:
        break;
    case CalibrationStep::CAL_HEADOFFSET_OFF:
        break;
    default:
        break;
    }
}

void CStartCalibrationFunc::StartZCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_Z_START:
        if (gcStartZCompen != NULL)
        {
            gcStartZCompen = NULL;
        }
        gcStartZCompen = new CStartZCompen();
        if (nSub3 == 1) // Simulation
        {
            gcStartZCompen->SetZCompenMode(false); // Don't apply
        }
        else
        {
            gcStartZCompen->SetZCompenMode(true); // Apply
        }
        if (nSub2 == TBL_HEAD1)
            gcStartZCompen->SetZAxis(_T("FZ1"));
        else if (nSub2 == TBL_HEAD2)
            gcStartZCompen->SetZAxis(_T("FZ2"));
        else if (nSub2 == TBL_HEAD3)
            gcStartZCompen->SetZAxis(_T("FZ3"));
        else if (nSub2 == TBL_HEAD4)
            gcStartZCompen->SetZAxis(_T("FZ4"));
        else if (nSub2 == TBL_HEAD5)
            gcStartZCompen->SetZAxis(_T("FZ5"));
        else if (nSub2 == TBL_HEAD6)
            gcStartZCompen->SetZAxis(_T("FZ6"));
        gcStartZCompen->Run();
        break;
    case CalibrationStep::CAL_Z_END:
        if (gcStartZCompen)
        {
            delete gcStartZCompen;
            gcStartZCompen = NULL;
        }
        break;
    case CalibrationStep::CAL_Z_ON:
        break;
    case CalibrationStep::CAL_Z_OFF:
        break;
    default:
        break;
    }
}

UINT CStartCalibrationFunc::StartCalibrationFunc(LPVOID wParam)
{
    return 0;
}

void CStartCalibrationFunc::StartROriginOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case CalibrationStep::CAL_R_START:
        if (gcStartROriginOffset != NULL)
        {
            gcStartROriginOffset = NULL;
        }
        gcStartROriginOffset = new CStartROriginOffset();
        gcStartROriginOffset->SetHeadOffsetMode(true);
        gcStartROriginOffset->SetCalibrationHead(nSub2);
        gcStartROriginOffset->Run();
        break;
    case CalibrationStep::CAL_R_END:
        if (gcStartROriginOffset)
        {
            delete gcStartROriginOffset;
            gcStartROriginOffset = NULL;
        }
        break;
    case CalibrationStep::CAL_R_ON:
        break;
    case CalibrationStep::CAL_R_OFF:
        break;
    default:
        break;
    }
}
