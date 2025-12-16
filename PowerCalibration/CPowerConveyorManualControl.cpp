#include "pch.h"
#include "CPowerConveyorManualControl.h"

#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "CSmemaControl.h"
#include "CLoadingToWork.h"
#include "CReturnToEntrance.h"
#include "CReturnToWork.h"
#include "COutToExit.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CPowerConveyorData.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CPowerConveyorManualControl* gcPowerConveyorManualControl;
CPowerConveyorManualControl::CPowerConveyorManualControl()
{
    m_ProdRunMode = 0;
    m_ConveyorRunMode = 0;
    m_EntryPcbCount = m_WorkPcbCount = m_ExitPcbCount = 0;
    GetId(&m_id);
    SetStep(ConveyorCtrlStep::CONVEYOR_INIT);
}

void CPowerConveyorManualControl::SetStep(ConveyorCtrlStep nStep)
{
    m_Step = nStep;
    TRACE(_T("[PWR] CPowerConveyorManualControl::SetStep:%d\n"), nStep);
}

ConveyorCtrlStep CPowerConveyorManualControl::GetStep()
{
    return m_Step;
}

long CPowerConveyorManualControl::GetThreadID()
{
    long retID;
    GetId(&m_id);
    retID = (long)m_id;
    return retID;
}

BOOL CPowerConveyorManualControl::OnTask()
{
    TRACE("[PWR] CPowerConveyorManualControl::OnTask Thread(0x%04X)\n", m_id);
    return TRUE;
}

BOOL CPowerConveyorManualControl::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    ConveyorCtrlStep CtrlStep;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CPowerConveyorManualControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        //dont' use cout here, output could be broken up due to threading
        TRACE("[PWR] CPowerConveyorManualControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        CtrlStep = CheckCalibrationStep(strMsg);
        SetStep(CtrlStep);
        if (GetStep() >= ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_INIT && GetStep() <= ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_END)
        {
            ConveyorManualLocationControl(CtrlStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        }
        delete msgReceived;
    }
    return TRUE;
}

CPowerConveyorManualControl::~CPowerConveyorManualControl()
{
}

ConveyorCtrlStep CPowerConveyorManualControl::CheckCalibrationStep(CString strHostMsg)
{
    ConveyorCtrlStep retCalStep = ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_INIT;
    if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_LOCATION_CONVEYOR_START)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_LOCATION_CONVEYOR_PCBINFO)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_PCBINFO;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_LOCATION_CONVEYOR_RUN)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_RUN;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_LOCATION_CONVEYOR_END)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_END;
    return retCalStep;
}

void CPowerConveyorManualControl::StartConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] StartConveyorManualLocationCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long From = SubMsg1, To = SubMsg2;
    if (From == 0 && To == 1)
        gcLoadingToWork = new CLoadingToWork();
    else if (From == 2 && To == 1)
        gcReturnToWork = new CReturnToWork();
    else if (To == 0)
        gcReturnToEntrance = new CReturnToEntrance();
    else if (To == 2)
        gcOutToExit = new COutToExit();
    delete pTime;
}

void CPowerConveyorManualControl::SetPcbInfoManualConveyorLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    if (gcWorkConveyor)
    {
        TRACE(_T("[PWR] SetPcbInfoManualConveyorLocationCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
        double PcbThickness = (double)(SubMsg1 / 10.0);
        double PcbStandByZOffset = (double)(SubMsg2 / 10.0);
        long PusherzRatio = SubMsg3;
        long UpRatio = 5, DownRatio = 5;
        UpRatio = (PusherzRatio >> 4);
        DownRatio = (PusherzRatio & 0x0F);
        TRACE(_T("[PWR] SetPcbInfoManualConveyorLocationCtrl PcbThickness,%.3f,PcbStandByZOffset,%.3f PusherZ Up:%03d%% Down:%03d%%\n"), PcbThickness, PcbStandByZOffset, UpRatio * 10, DownRatio * 10);
        gcWorkConveyor->SetPcbThickness(PcbThickness);
        gcWorkConveyor->SetPcbStandByZOffset(PcbStandByZOffset);
        gcWorkConveyor->SetPusherUpRatio(UpRatio);
        gcWorkConveyor->SetPusherDownRatio(DownRatio);
    }
    delete pTime;
}

void CPowerConveyorManualControl::RunConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] RunConveyorManualLocationCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long From = SubMsg1, To = SubMsg2;
    // InBuffer-0, Work-1, Exit-2
    if (From == 0 && To == 1)
        gcLoadingToWork->Run();
    else if (From == 2 && To == 1)
        gcReturnToWork->Run();
    else if (To == 0)
        gcReturnToEntrance->Run(From);
    else if (To == 2)
        gcOutToExit->Run(From);
    delete pTime;
}

void CPowerConveyorManualControl::StopConveyorManualLocationCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CString strLog;
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] StopConveyorManualLocationCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long From = SubMsg1, To = SubMsg2;
    if (From == 0 && To == 1)
    {
        if (gcLoadingToWork)
        {
            gcLoadingToWork->ExitThreadLoop();
            while (1)
            {
                if (gcLoadingToWork->IsEnd() == true)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl LoadingToWork Complete"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl LoadingToWork TimeOut"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
            };
        }
    }
    else if (From == 2 && To == 1)
    {
        pTime->TimeGet();
        if (gcReturnToWork)
        {
            gcReturnToWork->ExitThreadLoop();
            while (1)
            {
                if (gcReturnToWork->IsEnd() == true)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl ReturnToWork Complete"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl ReturnToWork TimeOut"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
            };
        }
    }
    else if (To == 0)
    {
        pTime->TimeGet();
        if (gcReturnToEntrance)
        {
            gcReturnToEntrance->ExitThreadLoop();
            while (1)
            {
                if (gcReturnToEntrance->IsEnd() == true)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl ReturnToEntrance Complete"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl ReturnToEntrance TimeOut"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
            };
        }
    }
    else if (To == 2)
    {
        pTime->TimeGet();
        if (gcOutToExit)
        {
            gcOutToExit->ExitThreadLoop();
            while (1)
            {
                if (gcOutToExit->IsEnd() == true)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl OutToExit Complete"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
                if (pTime->TimeElapsed() > TIME5000MS)
                {
                    strLog.Format(_T("[PWR] StopConveyorManualLocationCtrl OutToExit TimeOut"));
                    gcPowerLog->Logging(strLog);
                    break;
                }
            };
        }
    }
    TRACE(_T("[PWR] StopConveyorManualLocationCtrl Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerConveyorManualControl::ConveyorManualLocationControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_START:
        StartConveyorManualLocationCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_PCBINFO:
        SetPcbInfoManualConveyorLocationCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_RUN:
        RunConveyorManualLocationCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_LOCATION_CONVEYOR_END:
        StopConveyorManualLocationCtrl(nSub1, nSub2, nSub3);
        break;
    default:
        break;
    }
}

long CPowerConveyorManualControl::GetConveyorRunMode()
{
    return m_ConveyorRunMode;
}

void CPowerConveyorManualControl::SetConveyorRunMode(long ConveyorRunMode)
{
    m_ConveyorRunMode = ConveyorRunMode;
    TRACE(_T("[PWR] SetConveyorRunMode ConveyorRunMode:%d\n"), m_ConveyorRunMode);
}

bool CPowerConveyorManualControl::IsAllConveyorEnd()
{
    bool bEnd = false;
    if (gcLoadingToWork)
    {
        bEnd = gcLoadingToWork->IsEnd();
        if (bEnd == false)
            return bEnd;
    }
    if (gcReturnToWork)
    {
        bEnd = gcReturnToWork->IsEnd();
        if (bEnd == false)
            return bEnd;
    }
    if (gcReturnToEntrance)
    {
        bEnd = gcReturnToEntrance->IsEnd();
        if (bEnd == false)
            return bEnd;
    }
    if (gcOutToExit)
    {
        bEnd = gcOutToExit->IsEnd();
        if (bEnd == false)
            return bEnd;
    }
    return true;
}