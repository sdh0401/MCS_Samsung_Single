#include "pch.h"
#include "Cwmx3IO.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CPowerStackWalker.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

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

Cwmx3IO* gcwmx3IO;
Cwmx3IO::Cwmx3IO(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    InitialMemory();
}

void Cwmx3IO::InitialMemory()
{
    ZeroMemory(&gioMapStruct, sizeof(IOMAPSTRUCT));
    ZeroMemory(&inData, sizeof(inData));
    //ZeroMemory(&GetOutData, sizeof(GetOutData));
    //ZeroMemory(&SetOutData, sizeof(SetOutData));
}

Cwmx3IO::~Cwmx3IO()
{
}

UINT Cwmx3IO::ReadWMX3IOStatus(LPVOID wParam)
{
    int err = ErrorCode::None;
    long wait_ticks;
    bool bLoop = true, bFirst = true;
    CString strHostMsg;
    Cwmx3IO* pThis = reinterpret_cast<Cwmx3IO*>(wParam);
    wait_ticks = 0;
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_WMX3_IO_READTIME) == true)
        {
            TRACE(_T("[PWR] Cwmx3IO(0x%x) Terminated\n"), pThis->GetThreadID());
            break;
        }
        err = GetIO()->GetInBytes(0x00, WMX3_HW_MAXIO, &pThis->inData[0]);
        memcpy(gioMapStruct.inputAll, pThis->inData, WMX3_HW_MAXIO);
		err = GetIO()->GetOutBytes(0x00, WMX3_HW_MAXIO, &pThis->outData[0]);
		memcpy(gioMapStruct.outAll, pThis->outData, WMX3_HW_MAXIO);

        ThreadSleep(THREAD_WMX3_IO_READTIME);
        if (bFirst == true)
        {
            TRACE(_T("[PWR] IO Task Synchronize Set Event\n"));
            SetEvent(pThis->GetThreadStartSync());
            bFirst = false;
        }
    }
    TRACE(_T("[PWR] Cwmx3IO(0x%x) Quit\n"), pThis->GetThreadID());
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void Cwmx3IO::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thWmx3IO"));
    SetRunning(true);
	SetEnd(false);

    lpStartAddress = (_beginthreadex_proc_type)ReadWMX3IOStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] WMX3 IO Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] WMX3 IO Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
