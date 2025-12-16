#include "pch.h"
#include "CHomeStatus.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "Cwmx3Axis.h"
#include "DefineThreadLoopTime.h"
#include "AxisInformation.h"
#include "VisionData.h"
#include "GlobalDefine.h"
#include "COriginSearch.h"

#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

#include <thread>
#include <chrono>

using namespace std;

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
CHomeStatus* gcHomeStatus;
CHomeStatus::CHomeStatus()
{
    m_ShowID = 0;
}

CHomeStatus::~CHomeStatus()
{
}

void CHomeStatus::WaitAllAxisHomingThread()
{
    INT_PTR indx;
    THREAD_STRUCT* pThreadInfo = NULL;
    bool bLoop = true;
    //long lTimeChk = 0;
    //CApplicationTime* pTime = new CApplicationTime();
    ULONGLONG GetTime = 0, Elapsed = 0;
    TRACE(_T("[PWR] ~CHomeStatus WaitAxisThread %d\n"), GetAxisThreadCount());
    for (indx = 0; indx < GetAxisThreadCount(); indx++)
    {
        bLoop = true;
        GetTime = _time_get();
        pThreadInfo = GetAxisThreadByIndex(indx);
        while (bLoop)
        {
            if (IsAliveThread(pThreadInfo) == false)
            {
                Elapsed = _time_elapsed(GetTime);
                TRACE(_T("[PWR] CHomeStatus Axis Thread ID:0x%04X(%s) Exit Elapsed:%d[ms]\n"), pThreadInfo->m_ID, pThreadInfo->m_StrName, Elapsed);
                ThreadSleep(THREAD_EXIT_SAFETIME);
                bLoop = false;
            }
            ThreadSleep(THREAD_ALIVE_CHECKTIME);
        }
    }
    //delete pTime;
    //pTime = NULL;
}

void CHomeStatus::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetThreadName(_T("thHomeStatus"));
    lpStartAddress = (_beginthreadex_proc_type)StartAllHoming;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] All Homing Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] All Homing Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

UINT CHomeStatus::StartAllHoming(LPVOID wParam)
{
    INT_PTR indx = 0;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strAxis;
    Cwmx3Axis* pAxis;
    CString StrAxis;
    bool bLoop = true, bFirst = true, bHomingFail = false;
    CHomeStatus* pThis = reinterpret_cast<CHomeStatus*>(wParam);
    //CApplicationTime* pTime = new CApplicationTime();
    ULONGLONG GetTime = 0, Elapsed = 0;
    COriginSearch* pOriginSearch[MAXAXISNO];
    long InCompleteCount = 0;
    for (indx = 0; indx < GetWmx3AxisCount(); indx++)
    {
        pAxis = GetWmx3AxisByIndex(indx);
        pAxis->SetInitializeEnd(false);
        pAxis->SetInitializeFail(false);
    }
    TRACE(_T("[PWR] ########## All Z axis(%d) Start ########## \n"), GetZAxisCount());
    GetTime = _time_get();
    for (indx = 0; indx < GetZAxisCount(); indx++)
    {
        StrAxis = GetZAxisByIndex(indx);
        pAxis = GetWmx3AxisByName(StrAxis);
        TRACE(_T("[PWR] %s,%d Homing Start\n"), StrAxis, pAxis->GetAxisIndex());
        pOriginSearch[pAxis->GetAxisIndex()] = new COriginSearch(StrAxis);
        pOriginSearch[pAxis->GetAxisIndex()]->Run();
        ThreadSleep(TIME10MS);
    }
    while (IsAllZAxisHomingComplete() == false)
    {
        bHomingFail = false;
        for (indx = 0; indx < GetZAxisCount(); indx++)
        {
            StrAxis = GetZAxisByIndex(indx);
            bHomingFail = IsOneAxisHomingFail(StrAxis);
            if (bHomingFail == true)
            {
                TRACE(_T("[PWR] %s Homing Failed\n"), StrAxis);
                break;
            }
        }
        if (bHomingFail == true) break;
        ThreadSleep(TIME1000MS);
    }
    if (bHomingFail == false)
    {
        TRACE(_T("[PWR] ########## All Z axis(%d) End ########## \n"), GetZAxisCount());
        //TRACE(_T("[PWR] ########## All R axis(%d) Start ########## \n"), GetRAxisCount());
        //for (indx = 0; indx < GetRAxisCount(); indx++)
        //{
        //    StrAxis = GetRAxisByIndex(indx);
        //    pAxis = GetWmx3AxisByName(StrAxis);
        //    TRACE(_T("[PWR] %s,%d Homing Start\n"), StrAxis, pAxis->GetAxisIndex());
        //    pOriginSearch[pAxis->GetAxisIndex()] = new COriginSearch(StrAxis);
        //    pOriginSearch[pAxis->GetAxisIndex()]->Run();
        //    ThreadSleep(TIME10MS);
        //}
        //while (IsAllRAxisHomingComplete() == false)
        //{
        //    ThreadSleep(TIME1000MS);
        //}
        //TRACE(_T("[PWR] ########## All Z & R axis(%d,%d) homing complete %d[ms]   ########## \n"), GetZAxisCount(), GetRAxisCount(), _time_elapsed(GetTime));
        TRACE(_T("[PWR] ########## All Z (%d) homing complete %d[ms]   ########## \n"), GetZAxisCount(), _time_elapsed(GetTime));
        TRACE(_T("[PWR] ########## All axis(%d) Start ########## \n"), GetWmx3AxisCount());
        GetTime = _time_get();
        for (indx = 0; indx < GetWmx3AxisCount(); indx++)
        {
            pAxis = GetWmx3AxisByIndex(indx);
            if (pAxis->GetAxisMap() == NON) continue;
            if (pAxis->IsZAxis() == true) continue;
            strAxis = pAxis->GetAxisName();
            TRACE(_T("[PWR] %s,%d Homing Start\n"), strAxis, pAxis->GetAxisIndex());
            pOriginSearch[pAxis->GetAxisIndex()] = new COriginSearch(strAxis);
            pOriginSearch[pAxis->GetAxisIndex()]->Run();
            ThreadSleep(TIME1MS);
        }
        while (IsAllAxisHomingComplete() == false)
        {
            InCompleteCount = GetAllAxisHomingInComplete();
            for (indx = 0; indx < GetWmx3AxisCount(); indx++)
            {
                pAxis = GetWmx3AxisByIndex(indx);
                strAxis = pAxis->GetAxisName();
                bHomingFail = IsOneAxisHomingFail(strAxis);
                if (bHomingFail == true)
                {
                    TRACE(_T("[PWR] %s Homing Failed\n"), strAxis);
                    break;
                }
            }
            if (bHomingFail == true) break;
            ThreadSleep(TIME100MS);
        }
        if (bHomingFail == true)
        {
            TRACE(_T("[PWR] ########## All axis(%d) homing fail %d[ms]   ########## \n"), GetWmx3AxisCount(), _time_elapsed(GetTime));
        }
        else
        {
            TRACE(_T("[PWR] ########## All axis(%d) homing complete %d[ms]   ########## \n"), GetWmx3AxisCount(), _time_elapsed(GetTime));
        }
    }
    
    // Homing Fail
    if(bHomingFail == true)
    {
        for (indx = 0; indx < GetWmx3AxisCount(); indx++)
        {
            pAxis = GetWmx3AxisByIndex(indx);
            pAxis->SetInitializeEnd(false);
            pAxis->SetInitializeFail(true);
        }
    }
	else
	{
		//pThis->Y2ShiftAutoInit(FRONT_GANTRY);
	}



    gUpdateVisionData(FRONT_VISION, 0, 1);
    SendCameraRecognitionOffset(FRONT_GANTRY);
    //delete pTime;
    TRACE(_T("[PWR] CHomeStatus(0x%x) Quit\n"), pThis->GetThreadID());
    return 0;
}

void CHomeStatus::AddAxisThreadID(THREAD_STRUCT* pThreadInfo)
{
    m_HomingAxisIDArray.Add(pThreadInfo);
}

THREAD_STRUCT* CHomeStatus::GetAxisThreadByIndex(INT_PTR indx)
{
    THREAD_STRUCT* pThreadInfo = NULL;
    ASSERT(indx < m_HomingAxisIDArray.GetCount());
    pThreadInfo = m_HomingAxisIDArray.GetAt(indx);
    return pThreadInfo;
}

INT_PTR CHomeStatus::GetAxisThreadCount()
{
    return m_HomingAxisIDArray.GetCount();
}

void CHomeStatus::RemoveAxisThread(INT_PTR indx)
{
    ASSERT(indx < m_HomingAxisIDArray.GetCount());
    m_HomingAxisIDArray.RemoveAt(indx);
}

int CHomeStatus::GetAxisThreadIDByIndex(INT_PTR indx)
{
    THREAD_STRUCT* pThreadInfo = NULL;
    pThreadInfo = GetAxisThreadByIndex(indx);
    return pThreadInfo->m_ID;
}

long CHomeStatus::Y2ShiftAutoInit(long Gantry)
{
	Y2SHIFT_AUTOINIT y2ShiftAutoInit = GetY2ShiftAutoInit();
	CString strAxis;
	Cwmx3Axis* pAxis;
	INT_PTR indx = 0;
	bool Y1HomingComplete = false;
	bool Y1HomingFail = false;

	if (y2ShiftAutoInit.Use == false)
	{
		TRACE(_T("[PWR] Y2ShiftAutoInit %s Nouse\n"), strAxis);
		return NO_ERR;
	}

	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		pAxis = GetWmx3AxisByIndex(indx);
		if (pAxis->GetAxisMap() == NON) continue;
		if (pAxis->IsY1Axis(Gantry) == false) continue;
		strAxis = pAxis->GetAxisName();

		Y1HomingComplete = false;
		Y1HomingFail = false;

		while (1)
		{
			Y1HomingComplete = IsOneAxisHomingComplete(strAxis);
			if (Y1HomingComplete == true)
			{
				break;
			}

			Y1HomingFail = IsOneAxisHomingFail(strAxis);
			if (Y1HomingFail == true)
			{
				TRACE(_T("[PWR] Y2ShiftAutoInit %s Homing Failed\n"), strAxis);
				return HOMING_FAIL(pAxis->GetAxisIndex());
			}

			ThreadSleep(TIME10MS);
		}

		ThreadSleep(TIME100MS);

		bool torqueOver = false;
		double y1torque = 0.0;
		double y2torque = 0.0;

		for (long cnt = 0; cnt < 10; cnt++)
		{
			y1torque = ReadActualTorque(strAxis);
			y2torque = ReadActualTorque(pAxis->GetSlaveAxisName());
			if (y2ShiftAutoInit.TorqueLimit < abs(y1torque) || y2ShiftAutoInit.TorqueLimit < abs(y2torque))
			{
				TRACE(_T("[PWR] Y2ShiftAutoInit %s Torque Over Y1:%.1f Y2:%.1f\n"), strAxis, y1torque, y2torque);
				torqueOver = true;
				break;
			}
			ThreadSleep(TIME10MS);
		}

		if (torqueOver == false)
		{
			TRACE(_T("[PWR] Y2ShiftAutoInit %s Torque pass\n"), strAxis, y1torque, y2torque);
			return NO_ERR;
		}

		TRACE(_T("[PWR] Y2ShiftAutoInit %s Init Shift Start Y1:%.1f Y2:%.1f\n"), strAxis, y1torque, y2torque);

		ClearY2Shift(FRONT_GANTRY);
		std::unique_ptr<COriginSearch> pOriginSearch(new COriginSearch(strAxis));
		pOriginSearch->Run();			
		
		ThreadSleep(TIME100MS);

		Y1HomingComplete = false;
		Y1HomingFail = false;

		while (1)
		{
			Y1HomingComplete = IsOneAxisHomingComplete(strAxis);
			if (Y1HomingComplete == true)
			{
				break;
			}

			Y1HomingFail = IsOneAxisHomingFail(strAxis);
			if (Y1HomingFail == true)
			{
				TRACE(_T("[PWR] Y2ShiftAutoInit %s Homing Failed\n"), strAxis);
				return HOMING_FAIL(pAxis->GetAxisIndex());
			}

			ThreadSleep(TIME10MS);
		}

		ThreadSleep(TIME100MS);

	}
	TRACE(_T("[PWR] Y2ShiftAutoInit %s Complete\n"), strAxis);

	return NO_ERR;
}

void CHomeStatus::RemoveAllAxis()
{
    while(GetAxisThreadCount() > 0)
    {
        RemoveAxisThread(0);
        ThreadSleep(TIME1MS);
    }
}
