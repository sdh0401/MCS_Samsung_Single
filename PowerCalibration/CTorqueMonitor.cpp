#include "pch.h"
#include "CTorqueMonitor.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CPowerMain.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CReadJobFile.h"
//#include "ErrorCode.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CTorqueMonitor* gcTorqueMonitor;
CTorqueMonitor::CTorqueMonitor()
{
    m_ShowID = 0;
    m_TorqueMonitorStep = TorqueMonitorStep::INIT;
    InitializeValue();
}

CTorqueMonitor::~CTorqueMonitor()
{
}

void CTorqueMonitor::InitializeValue()
{
    m_MaxCount = 0;
    ZeroMemory(&m_Position, sizeof(m_Position));
    ZeroMemory(&m_Torque, sizeof(m_Torque));
    ZeroMemory(&m_Veloctiy, sizeof(m_Veloctiy));
}

TorqueMonitorStep CTorqueMonitor::GetStep()
{
    return m_TorqueMonitorStep;
}

void CTorqueMonitor::SetStep(TorqueMonitorStep TorqueMonitorStep)
{
    m_TorqueMonitorStep = TorqueMonitorStep;
}

void CTorqueMonitor::SetZAxis(CString strZAxis)
{
    m_StrZAxis = strZAxis;
}

CString CTorqueMonitor::GetZAxis()
{
    return m_StrZAxis;
}

long CTorqueMonitor::GetZAxisMap()
{
    long AxisMap = NON;
    AxisMap = GetAxisMap(GetZAxis());
    return AxisMap;
}

void CTorqueMonitor::SetPosition(long index, double Position)
{
    m_Position[index] = Position;
}

void CTorqueMonitor::SetTorque(long index, double Torque)
{
    m_Torque[index] = Torque;
}

void CTorqueMonitor::SetVelocity(long index, double Velocity)
{
    m_Veloctiy[index] = Velocity;
}

double CTorqueMonitor::GetPosition(long index)
{
    return m_Position[index];
}

double CTorqueMonitor::GetTorque(long index)
{
    return m_Torque[index];
}

double CTorqueMonitor::GetVelocity(long index)
{
    return m_Veloctiy[index];
}

void CTorqueMonitor::SetMaxCount(long MaxCount)
{
    m_MaxCount = MaxCount;
    TRACE(_T("[PWR] CTorqueMonitor SetMaxCount:%d\n"), MaxCount);
}

long CTorqueMonitor::GetMaxCount()
{
    return m_MaxCount;
}

long CTorqueMonitor::ShowValue()
{
    TRACE(_T("[PWR] CTorqueMonitor GetMaxCount:%d\n"), GetMaxCount());
    for (long Count = 0; Count < GetMaxCount(); ++Count)
    {
        TRACE(_T("[PWR] Count:%d Position:%.3f Torque:%.3f Velocity:%.3f\n"), Count,
            GetPosition(Count), GetTorque(Count), GetVelocity(Count));
    }
    return NO_ERR;
}

UINT CTorqueMonitor::TorqueMonitorControl(LPVOID wParam)
{
    CString strLog;
    bool bLoop = true;
    long Err = ErrorCode::None, index = 0, AxisMap = NON;
    CTorqueMonitor* pThis = reinterpret_cast<CTorqueMonitor*>(wParam);
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    ULONGLONG BuzzerOnGetTime = _time_get(), BuzzerOnElapsed = 0;
    TorqueMonitorStep TorqueMonitorStep = TorqueMonitorStep::INIT, TorqueMonitorOldStep = TorqueMonitorStep::STOP;
    CoreMotionStatus CoreMotionStatus;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_TORQUE_MONITOR_READTIME) == true)
        {
            TRACE(_T("[PWR] CTorqueMonitor(0x%x) Terminated\n"), pThis->m_ShowID);
            strLog.Format(_T("[PWR] CTorqueMonitor(0x%x) Terminated\n"), pThis->m_ShowID);
            gcPowerLog->Logging(strLog);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_TORQUE_MONITOR_READTIME);
        //    continue;
        //}
        if (TorqueMonitorOldStep != pThis->GetStep())
        {
            Elapsed = _time_elapsed(GetTime);
            //if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] CTorqueMonitor Step:%d Time:%d[ms]\n"), pThis->GetStep(), Elapsed);
            }
            TorqueMonitorStep = pThis->GetStep();
            GetTime = _time_get();
        }
        switch (pThis->GetStep())
        {
        case TorqueMonitorStep::INIT:
            break;
        case TorqueMonitorStep::STOP:
            break;
        case TorqueMonitorStep::START:
            index = 0;
            AxisMap = pThis->GetZAxisMap();
            pThis->InitializeValue();
            if (AxisMap != NON)
            {
                pThis->SetStep(TorqueMonitorStep::GATHERING);
            }
            else
            {
                pThis->SetStep(TorqueMonitorStep::SELFQUIT);
            }
            break;
        case TorqueMonitorStep::GATHERING:
            GetCoreMotion()->GetStatus(&CoreMotionStatus);
            pThis->SetPosition(index, CoreMotionStatus.axesStatus[AxisMap].actualPos);
            pThis->SetTorque(index, CoreMotionStatus.axesStatus[AxisMap].actualTorque);
            pThis->SetVelocity(index, CoreMotionStatus.axesStatus[AxisMap].actualVelocity);
            index++;
            break;
        case TorqueMonitorStep::END:
            pThis->SetMaxCount(index);
            pThis->SetStep(TorqueMonitorStep::SELFQUIT);
            pThis->ShowValue();
            break;
        case TorqueMonitorStep::SELFQUIT:
            bLoop = false;
            break;
        case TorqueMonitorStep::QUIT:
            break;
        }
        if (bLoop == false)
        {
            break;
        }
        ThreadSleep(THREAD_TORQUE_MONITOR_READTIME);
    }
    TRACE(_T("[PWR] CTorqueMonitor(0x%x) Quit\n"), pThis->m_ShowID);
    strLog.Format(_T("[PWR] CTorqueMonitor(0x%x) Quit\n"), pThis->m_ShowID);
    gcPowerLog->Logging(strLog);
    return 0;
}

void CTorqueMonitor::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thTorqueMonitor"));
    SetRunning(true);
    SetEnd(false);
    lpStartAddress = (_beginthreadex_proc_type)TorqueMonitorControl;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] TorqueMonitorControl Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] TorqueMonitorControl Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
