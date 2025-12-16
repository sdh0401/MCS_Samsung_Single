#include "pch.h"
#include "CioStatus.h"

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

CioStatus* gcIOStatus;
CioStatus::CioStatus(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    m_ShowID = 0x0;
}

CioStatus::~CioStatus()
{
}

//void CioStatus::SetDirectOutput(unsigned index, unsigned mindx, unsigned char offset)
//{
//    unsigned char ucSetOutput = WMX3IO_OUT_OFF;
//    unsigned char ucGetOutput = WMX3IO_OUT_OFF;
//    unsigned nindx = 0;
//    ASSERT(index < WMX3_HW_MAXIO);
//    ASSERT(mindx < WMX3IO_MAX_MODULE);
//    ASSERT(offset <= WMX3IO_MODULE_BYTE);
//    for (nindx = 0; nindx < WMX3IO_MODULE_BYTE; nindx++)
//    {
//        ucSetOutput = gioMapStruct.outputOut[mindx][nindx + offset];
//        ucGetOutput = gioMapStruct.outputIn[mindx][nindx + offset];
//        if (ucGetOutput != ucSetOutput)
//        {
//            if (ucSetOutput == WMX3IO_OUT_ON)
//            {
//                SET_BIT(gioMapStruct.outputOutAll[index], nindx);
//            }
//            else
//            {
//                CLR_BIT(gioMapStruct.outputOutAll[index], nindx);
//            }
//        }
//    }
//}

void CioStatus::ReadDirectInput(unsigned index)
{
    unsigned nindx = 0;
    unsigned char ioBit = 0x0;
	CString strDescription;

    if (index < WMX3_HW_MAXIO)
    {
		ioBit = gioMapStruct.inputAll[index];

        for (nindx = 0; nindx < WMX3IO_MODULE_BYTE; nindx++)
        {
            if (GET_BIT(ioBit, nindx))
            {
				if (gioMapStruct.input[index][nindx] == false)
				{
					if (gcPowerIO->IsUsedIO(0, index, nindx, &strDescription) == true)
					{
						TRACE(_T("[PWR] IOStatus Input %d.%d OFF(%dms)->ON (%s)\n"), index, nindx, _time_elapsed(gioMapStruct.changedTimeInOff[index][nindx]), strDescription);
					}

					gioMapStruct.changedTimeInOn[index][nindx] = _time_get();
				}

                gioMapStruct.input[index][nindx] = true;
            }
            else
            {
				if (gioMapStruct.input[index][nindx] == true)
				{
					if (gcPowerIO->IsUsedIO(0, index, nindx, &strDescription) == true)
					{
						TRACE(_T("[PWR] IOStatus Input %d.%d ON(%dms)->OFF (%s)\n"), index, nindx, _time_elapsed(gioMapStruct.changedTimeInOn[index][nindx]), strDescription);
					}

					gioMapStruct.changedTimeInOff[index][nindx] = _time_get();
				}

                gioMapStruct.input[index][nindx] = false;
            }
        }
    }
}


void CioStatus::ReadDirectOutput(unsigned index)
{
	unsigned nindx = 0;
	unsigned char ioBit = 0x0;
	bool oldData = false;
	long ioNum = 0;
	IOStruct data;
	CString strDescription;

	if (index < WMX3_HW_MAXIO)
	{
		ioBit = gioMapStruct.outAll[index];
		for (nindx = 0; nindx < WMX3IO_MODULE_BYTE; nindx++)
		{
			oldData = gioMapStruct.output[index][nindx];

			if (GET_BIT(ioBit, nindx))
			{
				if (gioMapStruct.output[index][nindx] == false)
				{
					if (gcPowerIO->IsUsedIO(1, index, nindx, &strDescription) == true)
					{
						TRACE(_T("[PWR] IOStatus Output %d.%d OFF(%dms)->ON (%s)\n"), index, nindx, _time_elapsed(gioMapStruct.changedTimeOutOff[index][nindx]), strDescription);
					}

					gioMapStruct.changedTimeOutOn[index][nindx] = _time_get();
				}

				gioMapStruct.output[index][nindx] = true;
			}
			else
			{
				if (gioMapStruct.output[index][nindx] == true)
				{
					if (gcPowerIO->IsUsedIO(1, index, nindx, &strDescription) == true)
					{
						TRACE(_T("[PWR] IOStatus Output %d.%d ON(%dms)->OFF (%s)\n"), index, nindx, _time_elapsed(gioMapStruct.changedTimeOutOn[index][nindx]), strDescription);
					}

					gioMapStruct.changedTimeOutOff[index][nindx] = _time_get();
				}

				gioMapStruct.output[index][nindx] = false;
			}
		}
	}
}

void CioStatus::InitInputTime()
{
	unsigned index, nindx = 0;
	unsigned char ioBit = 0x0;

	for (index = 0; index < WMX3_HW_MAXIO; ++index)
	{
		for (nindx = 0; nindx < WMX3IO_MODULE_BYTE; nindx++)
		{
			gioMapStruct.time[index][nindx] = 0;

			gioMapStruct.changedTimeInOn[index][nindx] = _time_get();
			gioMapStruct.changedTimeInOff[index][nindx] = _time_get();
			gioMapStruct.changedTimeOutOn[index][nindx] = _time_get();
			gioMapStruct.changedTimeOutOff[index][nindx] = _time_get();
		}
	}
}

//void CioStatus::ReadDirectInput(unsigned index, unsigned mindx, unsigned char offset)
//{
//    unsigned nindx = 0;
//    unsigned char ioBit = 0x0;
//    ASSERT(index < WMX3_HW_MAXIO);
//    ASSERT(mindx < WMX3IO_MAX_MODULE);
//    ASSERT(offset <= WMX3IO_MODULE_BYTE);
//    for (nindx = 0; nindx < WMX3IO_MODULE_BYTE; nindx++)
//    {
//        ioBit = gioMapStruct.inputAll[index];
//        if (GET_BIT(ioBit, nindx)) 
//        {
//            gioMapStruct.input[mindx][nindx + offset] = true;
//        }
//        else
//        {
//            gioMapStruct.input[mindx][nindx + offset] = false;
//        }
//    }
//}

UINT CioStatus::ReadIOStatus(LPVOID wParam)
{
    bool bLoop = true;
    unsigned index = 0, mindx = 0, nindx = 0;
    unsigned char ucOutput = WMX3IO_OUT_OFF;
    CString strHostMsg;
    CioStatus* pThis = reinterpret_cast<CioStatus*>(wParam);
    AddThread(pThis);
    TRACE(_T("[PWR] IO Task Synchronize Wait Event\n"));
    WaitForSingleObject(pThis->GetThreadStartSync(), INFINITE);
    TRACE(_T("[PWR] IO Task Synchronize Recv Event\n"));
    //pThis->SetMsgQueueStatus(THREAD_RUN);
	pThis->InitInputTime();
    while (pThis->GetRunning() == true)
    {
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_IO_READTIME);
        //    continue;
        //}
        index = mindx = nindx = 0;
        for (index = 0; index < WMX3_HW_MAXIO; ++index)
        {
            // Output
            //pThis->SetDirectOutput(index, mindx, 0);
            // Input
            pThis->ReadDirectInput(index);
            pThis->ReadDirectOutput(index);
            //pThis->ReadDirectInput(index, mindx, 0);
            //index++;
        }
        //ThreadSleep(THREAD_IO_READTIME);
        if (pThis->IsTerminated(THREAD_IO_READTIME) == true)
        {
            TRACE(_T("[PWR] CioStatus(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
    }
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    TRACE(_T("[PWR] CioStatus(0x%X) Quit\n"), pThis->m_ShowID);
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CioStatus::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);
    SetThreadName(_T("thIOStatus"));
    lpStartAddress = (_beginthreadex_proc_type)ReadIOStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;    
    TRACE(_T("[PWR] IO Status Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CioStatus Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}