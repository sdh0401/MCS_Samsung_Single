#include "pch.h"
#include "CAnalogStatus.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CPowerStackWalker.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CPowerIO.h"

#include <WMX3Api.h>
#include <EcApi.h>
#include <IOApi.h>
#include <CoreMotionApi.h>
/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace ecApi;
using namespace wmx3Api;
using namespace std;

CAnalogStatus* gcAnalogStatus;
CAnalogStatus::CAnalogStatus()
{
    InitialMemory();
}

void CAnalogStatus::InitialMemory()
{
}

CAnalogStatus::~CAnalogStatus()
{
}

UINT CAnalogStatus::ReadAnalogStatus(LPVOID wParam)
{
    int err = ErrorCode::None;
    bool bLoop = true;
    CString strHostMsg;
    unsigned short nLevel = 0, nOldLevel = 0xFFFF;
    //unsigned short nLevel1 = 0, nOldLevel1 = 0xFFFF;
    //unsigned short nLevel2 = 0, nOldLevel2 = 0xFFFF;
    long IOAddr = IO_NOUSE;
    CAnalogStatus* pThis = reinterpret_cast<CAnalogStatus*>(wParam);
    AddThread(pThis);
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (pThis->GetRunning() == true)
    //do
    {        
        if (pThis->IsTerminated(THREAD_ANALOG_READTIME) == true)
        {
            TRACE(_T("[PWR] CAnalogStatus(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_ANALOG_READTIME);
        //    continue;
        //}
        for (long No = 0; No < MAXIODEFINE; ++No)
        {
            if (gcPowerIO->GetIOUsage(No) == YES_USE)
            {
                if (gcPowerIO->GetIOType(No) == IO_WMX3_ANALOG_SICK || gcPowerIO->GetIOType(No) == IO_WMX3_ANALOG_EJECTOR || gcPowerIO->GetIOType(No) == IO_WMX3_ANALOG_RTD)
                {
                    IOAddr = gcPowerIO->GetIOAddr(No);
                    if (IOAddr != IO_NOUSE)
                    {
                        err = GetIO()->GetInAnalogDataUShort(IOAddr, &nLevel);
                        gcPowerIO->SetAnalogInput(No, nLevel);
                        if (abs(nLevel - nOldLevel) > 50)
                        {
                            if (gcPowerLog->IsShowIoLog() == true)
                            {
                                TRACE(_T("[PWR] IO(%d) Change Old:0x%04X New:0x%04X\n"), No, nOldLevel, nLevel);
                            }
                            nOldLevel = nLevel;
                        }
                    }
                }
            }
        }
        //err = GetIO()->GetInAnalogDataUShort(0x68, &nLevel1);
        //err = GetIO()->GetInAnalogDataUShort(0x6A, &nLevel2);
        //if (abs(nLevel1 - nOldLevel1) > 50)
        //{
        //    TRACE(_T("[PWR] Level1 Change Old:0x%04X New:0x%04X\n"), nOldLevel1, nLevel1);
        //    nOldLevel1 = nLevel1;
        //}
        //if (abs(nLevel2 - nOldLevel2) > 50)
        //{
        //    TRACE(_T("[PWR] Level2 Change Old:0x%04X New:0x%04X\n"), nOldLevel2, nLevel2);
        //    nOldLevel2 = nLevel2;
        //}
    };// while (pThis->IsTerminated(THREAD_ANALOG_READTIME) == false);
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    TRACE(_T("[PWR] CAnalogStatus(0x%x) Quit\n"), pThis->m_ShowID);
    pThis->SetRun(false);
    return 0;
}

void CAnalogStatus::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetThreadName(_T("thAnalogStatus"));
    lpStartAddress = (_beginthreadex_proc_type)ReadAnalogStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;    
    TRACE(_T("[PWR] Analog IO Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CAnalogStatus Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
