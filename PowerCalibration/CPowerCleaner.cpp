#include "pch.h"
#include "CPowerCleaner.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "CPowerLog.h"

CPowerCleaner* gcPowerCleaner;
CPowerCleaner::CPowerCleaner()
{
    m_bLoop = true;
}

CPowerCleaner::~CPowerCleaner()
{
    m_bLoop = false;
}

UINT CPowerCleaner::StartCleaner(LPVOID wParam)
{
    CPowerCleaner* pThis = reinterpret_cast<CPowerCleaner*>(wParam);
    INT_PTR Count = 0;
    CPowerThread* pThread;
    do
    {
        Count = GetCleanThreadCount();
        if (Count > 0)
        {
            pThread = GetCleanThread(0);
            if (pThread != NULL)
            {
                DestroyCleanThread(0);
            }
        }
        ThreadSleep(THREAD_CLEANER_READTIME);
    } while (pThis->m_bLoop == true);// while (pThis->IsTerminated(THREAD_CLEANER_READTIME) == false);
    return 0;
}

void CPowerCleaner::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    lpStartAddress = (_beginthreadex_proc_type)StartCleaner;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    TRACE(_T("[PWR] Cleaner Thread ID:0x%04X\n"), nID);
    strLog.Format(_T("[PWR] Cleaner Thread ID:0x%04X"), nID);
    gcPowerLog->Logging(strLog);
}


