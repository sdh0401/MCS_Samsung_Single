#include "pch.h"
#include "CSyncGantryConveyor.h"
#include "AxisInformation.h"
#include "DefineThreadLoopTime.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "Trace.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

CSyncGantryConveyor* gcSyncGantryConveyor;
CSyncGantryConveyor::CSyncGantryConveyor()
{
    m_ShowID = 0;
    m_SyncGantryConveyorStep = SyncGantryConveyorStep::STOP;
}

CSyncGantryConveyor::~CSyncGantryConveyor()
{
}

void CSyncGantryConveyor::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetThreadName(_T("thSyncGantryConveyor"));
    lpStartAddress = (_beginthreadex_proc_type)StartSyncGantryConveyor;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] CSyncGantryConveyor Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CSyncGantryConveyor Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

SyncGantryConveyorStep CSyncGantryConveyor::GetStep()
{
    return m_SyncGantryConveyorStep;
}

UINT CSyncGantryConveyor::StartSyncGantryConveyor(LPVOID wParam)
{
    CSyncGantryConveyor* pThis = reinterpret_cast<CSyncGantryConveyor*>(wParam);
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    long TimeChk = 0, Conveyor = FRONT_CONV;
    bool MsgShow = true;
    SyncGantryConveyorStep SyncStep = SyncGantryConveyorStep::STOP;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_SYNC_READTIME) == true)
        {
            TRACE(_T("[PWR] CSyncGantryConveyor(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_SYNC_READTIME);
        //    continue;
        //}
        switch (SyncStep)
        {
        case SyncGantryConveyorStep::STOP:
            break;

        case SyncGantryConveyorStep::START:
            SyncStep = SyncGantryConveyorStep::PCB_NOTREADY;
            //PCBWaitReset();
            break;

        case SyncGantryConveyorStep::PCB_NOTREADY:
            if (IsWorkPcbReady(Conveyor) == true)
            {
                SyncStep = SyncGantryConveyorStep::PCB_READY;
            }
            break;

        case SyncGantryConveyorStep::PCB_READY:
            if (IsWorkPcbReady(Conveyor) == false)
            {
                //PCBWaitUnLock();
                SyncStep = SyncGantryConveyorStep::START;
            }
            break;

        case SyncGantryConveyorStep::SELFQUIT:
            break;
        }
    };
    TRACE(_T("[PWR] CSyncGantryConveyor(0x%X) Quit\n"), pThis->m_ShowID);
    return 0;
}
