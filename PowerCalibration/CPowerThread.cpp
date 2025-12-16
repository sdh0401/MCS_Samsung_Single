#include "pch.h"
#include "CPowerThread.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"
#include "DefineThreadLoopTime.h"
//#include "ErrorCode.h"

CPowerThread::CPowerThread()
{
    InitThreadData();
    msgQInfo = new PowerThreadMessageQInfo();
}

CPowerThread::CPowerThread(HANDLE h_Terminate)
{
    InitThreadData(h_Terminate);
    msgQInfo = new PowerThreadMessageQInfo();
}

CPowerThread::CPowerThread(HANDLE h_Terminate, HANDLE h_HostMutex)
{
    InitThreadData(h_Terminate, h_HostMutex);
    msgQInfo = new PowerThreadMessageQInfo();
}

CPowerThread::CPowerThread(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    InitThreadData(h_Terminate, h_HostMutex, h_RunStartSync);
    msgQInfo = new PowerThreadMessageQInfo();
}

CPowerThread::~CPowerThread()
{
    ExitThreadLoop();
    ThreadSleep(TIME10MS);
	delete msgQInfo;
	msgQInfo = NULL;
}

void CPowerThread::SetThreadHandle(HANDLE nHandle)
{
    ASSERT(nHandle != INVALID_HANDLE_VALUE);
    threadInfo.m_Handle = nHandle;
}

HANDLE CPowerThread::GetThreadHandle()
{
    ASSERT(threadInfo.m_Handle != INVALID_HANDLE_VALUE);
    return threadInfo.m_Handle;
}

void CPowerThread::SetThreadLock(HANDLE nLock)
{
    ASSERT(nLock != INVALID_HANDLE_VALUE);
    threadInfo.m_CmdLock = nLock;
}

HANDLE CPowerThread::GetThreadLock()
{
    ASSERT(threadInfo.m_CmdLock != INVALID_HANDLE_VALUE);
    return threadInfo.m_CmdLock;
}

void CPowerThread::SetThreadTerminate(HANDLE nTerminated)
{
    ASSERT(nTerminated != INVALID_HANDLE_VALUE);
    threadInfo.m_Terminate = nTerminated;
}

HANDLE CPowerThread::GetThreadTerminate()
{
    ASSERT(threadInfo.m_Terminate != INVALID_HANDLE_VALUE);
    return threadInfo.m_Terminate;
}

void CPowerThread::SetThreadExternalTerminate(HANDLE nTerminated)
{
    ASSERT(nTerminated != INVALID_HANDLE_VALUE);
    threadInfo.m_ExternalTerminate = nTerminated;
}

HANDLE CPowerThread::GetThreadExternalTerminate()
{
    ASSERT(threadInfo.m_ExternalTerminate != INVALID_HANDLE_VALUE);
    return threadInfo.m_ExternalTerminate;
}

void CPowerThread::SetThreadStartSync(HANDLE nStartSync)
{
    ASSERT(nStartSync != INVALID_HANDLE_VALUE);
    threadInfo.m_StartSync = nStartSync;
}

HANDLE CPowerThread::GetThreadStartSync()
{
    ASSERT(threadInfo.m_StartSync != INVALID_HANDLE_VALUE);
    return threadInfo.m_StartSync;
}

HANDLE CPowerThread::GetThreadLockComplete()
{
    ASSERT(threadInfo.m_LockComplete != INVALID_HANDLE_VALUE);
    return threadInfo.m_LockComplete;
}

void CPowerThread::SetThreadID(int nID)
{
    ASSERT(nID != NULL);
    threadInfo.m_ID = nID;
}

unsigned CPowerThread::GetThreadID()
{
    if (threadInfo.m_ID == NULL)
    {
        return NULL;
    }
    return threadInfo.m_ID;
}

void CPowerThread::SetThreadName(CString strName)
{
    ASSERT(strName.GetLength() > 0);
    threadInfo.m_StrName = strName;
}

CString CPowerThread::GetThreadName()
{
    CString strName;
    strName = threadInfo.m_StrName;
    if (strName.GetLength() > 0)
    {
        return strName;
    }
    else
    {
        return _T("NULL");
    }
}

void CPowerThread::InitThreadVariable()
{
    for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        threadInfo.m_SubMsg[indx] = INITIALZE;
    threadInfo.m_Handle = NULL;
    threadInfo.m_ID = NULL;
    threadInfo.m_StrName.Empty();
    threadInfo.m_bEnd = true;
    threadInfo.m_bCreate = true;
    threadInfo.m_bStart = false;
    threadInfo.m_bSuspend = false;
    threadInfo.m_bRun = false;
    threadInfo.m_bErrorStatus = false;
    threadInfo.m_bRunning = false;
}

void CPowerThread::InitThreadData()
{
    InitThreadVariable();
    threadInfo.m_Terminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_CmdLock = CreateMutex(NULL, FALSE, NULL);
    threadInfo.m_LockComplete = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_ExternalTerminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_StartSync = NULL;
    threadInfo.m_StrName.Empty();
}

void CPowerThread::InitThreadData(HANDLE h_Terminate)
{
    InitThreadVariable();
    threadInfo.m_Terminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_CmdLock = CreateMutex(NULL, FALSE, NULL);
    threadInfo.m_LockComplete = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_ExternalTerminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_StartSync = NULL;
    threadInfo.m_StrName.Empty();
}

void CPowerThread::InitThreadData(HANDLE h_Terminate, HANDLE h_HostMutex)
{
    InitThreadVariable();
    threadInfo.m_Terminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_CmdLock = CreateMutex(NULL, FALSE, NULL);
    threadInfo.m_LockComplete = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_ExternalTerminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_StartSync = NULL;
    threadInfo.m_StrName.Empty();
}

void CPowerThread::InitThreadData(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    InitThreadVariable();
    threadInfo.m_Terminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_CmdLock = CreateMutex(NULL, FALSE, NULL);
    threadInfo.m_LockComplete = CreateEvent(NULL, TRUE, FALSE, NULL);
    threadInfo.m_ExternalTerminate = CreateEvent(NULL, TRUE, FALSE, NULL);
    SetThreadStartSync(h_RunStartSync);
    threadInfo.m_StrName.Empty();
}

void CPowerThread::WaitThreadLoop()
{
    //bool bLoop = true;
    //long lTimeChk = 0;
    //CApplicationTime* pTime = new CApplicationTime();
    //while (bLoop)
    //{
    //    lTimeChk = pTime->TimeElapsed();
    //    if (IsAliveThread() == false)
    //    {            
    //        ThreadSleep(THREAD_EXIT_SAFETIME);
    //        bLoop = false;
    //        break;
    //    }
    //    if (WaitLockComplete() == true)
    //    {
    //        ThreadSleep(THREAD_EXIT_SAFETIME);
    //        bLoop = false;
    //        break;
    //    }
    //    TRACE(_T("[PWR] CPowerThread WaitThreadLoop Elapsed:%d[ms]\n"), lTimeChk);
    //    if (lTimeChk > THREAD_EXIT_TIMEOUT)
    //    {
    //        bLoop = false;
    //        break;
    //    }
    //    ThreadSleep(THREAD_ALIVE_CHECKTIME);
    //}
    //delete pTime;
    //pTime = NULL;
}

void CPowerThread::WaitForSingleObjectTime(HANDLE nHandle, int nTimeCount, int nMaxTime)
{
    //int nRet = 0;
    //int nWaitCount = 0;
    //while (true)
    //{
    //    nRet = WaitForSingleObject(nHandle, nMaxTime);
    //    if (nRet != WAIT_OBJECT_0)
    //    {
    //        nWaitCount++;
    //        if (nWaitCount > nTimeCount)
    //        {
    //            break;
    //        }
    //        continue;
    //    }
    //    else
    //    {
    //        break;
    //    }
    //}
}

void CPowerThread::StopThread()
{
    //int nRet = 0;
    //DWORD dwExitCode = 0;
    //HANDLE nHandle = GetThreadHandle();
    //SetThreadStatusError(false);
    //if (nHandle != NULL)
    //{
    //    //WaitForSingleObject(GetThreadTerminate(), INFINITE);
    //    WaitForSingleObjectTime(GetThreadTerminate(), 5, TIME1000MS);
    //    GetExitCodeThread(nHandle, &dwExitCode);
    //    CloseMutex();
    //    CloseThread();        
    //    InitThreadData();        
    //}
}

bool CPowerThread::ExitThreadLoop()
{
	ULONGLONG time = _time_get();

	if (IsEnd() == false)
	{
		TRACE(_T("[PWR] ExitThreadLoop start (%s, 0x%X)\n"), GetThreadName(), GetThreadID());

		SetEvent(GetThreadExternalTerminate());
		SetRunning(false);

		while (1)
		{
			if (IsEnd() == true)
			{
				TRACE(_T("[PWR] ExitThreadLoop complete (%s, 0x%X)\n"), GetThreadName(), GetThreadID());
				break;
			}
			else if (_time_elapsed(time) > TIME2000MS)
			{
				TRACE(_T("[PWR] ExitThreadLoop timeout (%s, 0x%X)\n"), GetThreadName(), GetThreadID());
				break;
			}

			ThreadSleep(TIME10MS);
		}
		ResetEvent(GetThreadExternalTerminate());
	}
	else
	{
		TRACE(_T("[PWR] ExitThreadLoop skip (%s, 0x%X)\n"), GetThreadName(), GetThreadID());
	}

	return IsEnd();
//    PowerThreadMessage* msgReceived = new PowerThreadMessage();
//    msgReceived->SetID(GetThreadID());
//    msgReceived->SetThreadMsg(_T(STRING_HOST_MSG_SINGLE_OUT));
//    msgQInfo->PushAfterCheckStatus(msgReceived);

}

bool CPowerThread::IsAliveThread(THREAD_STRUCT* threadInfo)
{
    if (threadInfo->m_Handle == NULL)
    {
        // 실행 중인 상태가 아니다.
        return false;
    }
    if (::WaitForSingleObject(threadInfo->m_Handle, 0) == WAIT_TIMEOUT)
    {
        // 현재 쓰레드가 실행 중.
        return true;
    }
    else
    {
        // 실행 중인 상태가 아니다.
        return false;
    }
}

bool CPowerThread::IsAliveThread()
{
    HANDLE threadHandle = GetThreadHandle();
    if (::WaitForSingleObject(threadHandle, 0) == WAIT_TIMEOUT)
    {
        // 현재 쓰레드가 실행 중.
        return true;
    }
    else
    {
        // 실행 중인 상태가 아니다.
        return false;
    }
}

void CPowerThread::CloseThread()
{
    //BOOL bClose = FALSE;
    //HANDLE threadHandle = GetThreadHandle();
    //try
    //{
    //    if (threadHandle != NULL)
    //    {
    //        bClose = CloseHandle(threadHandle);
    //        //ASSERT(bClose == TRUE);
    //        SetThreadHandle(NULL);
    //    }
    //}
    //catch (...)
    //{
    //    TRACE(_T("[PWR] CloseThread Exception\n"));
    //}
}

void CPowerThread::CloseMutex()
{
    //BOOL bClose = FALSE;
    //HANDLE lockHandle = GetThreadLock();
    //try
    //{
    //    if (lockHandle != NULL)
    //    {
    //        bClose = CloseHandle(lockHandle);
    //        //ASSERT(bClose == TRUE);
    //        SetThreadLock(NULL);
    //    }
    //}
    //catch (...)
    //{
    //    TRACE(_T("[PWR] CloseMutex Exception\n"));
    //}
}

CString CPowerThread::CheckMessageQueue(LPVOID wParam)
{
    void* pMsg = NULL;
    PowerThreadMessage* msgReceived;
    CString strMsg;
    int nTargetID = INITIALZE;
    CPowerThread* pThis = reinterpret_cast<CPowerThread*>(wParam);
    strMsg = _T("NULL");
    TRACE(_T("[PWR] CPowerThread %s \n"), pThis->GetThreadName());
    if (pThis->msgQInfo->IsEmpty() == false)
    {
        msgReceived = pThis->msgQInfo->m_Queue->front();
        pThis->msgQInfo->Pop();
        nTargetID = msgReceived->GetID();
        if (nTargetID != pThis->GetThreadID())
        {
            TRACE(_T("[PWR] CPowerThread GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, pThis->GetThreadID());
        }
        strMsg = msgReceived->GetThreadMsg();
        delete msgReceived;
        msgReceived = NULL;
        return strMsg;
    }
    return strMsg;
}

CString CPowerThread::CheckMessageQueue(LPVOID wParam, signed* arg)
{
    void* pMsg = NULL;
    PowerThreadMessage* msgReceived;
    CString strMsg;
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CPowerThread* pThis = reinterpret_cast<CPowerThread*>(wParam);
    strMsg = _T("NULL");
    if (pThis->msgQInfo->IsEmpty() == false)
    {
        msgReceived = pThis->msgQInfo->m_Queue->front();
        pThis->msgQInfo->Pop();
        nTargetID = msgReceived->GetID();
        if (nTargetID != pThis->GetThreadID())
        {
            TRACE(_T("[PWR] CPowerThread GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, pThis->GetThreadID());
        }        
        strMsg = msgReceived->GetThreadMsg();
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
            arg[indx] = nSubMsg[indx];
        }
        delete msgReceived;
        msgReceived = NULL;
        return strMsg;
    }
    return strMsg;
}

void CPowerThread::SetMsgQueueStatus(int nStatus)
{
    if (msgQInfo != NULL)
    {
        msgQInfo->SetStatus(nStatus);
    }
    else
    {
        TRACE(_T("[PWR] SetMsgQueueStatus msgQInfo is NULL\n"));
    }
}

void CPowerThread::PushAfterCheckStatus(PowerThreadMessage* pThMsg)
{
    if (msgQInfo != NULL)
    {
        msgQInfo->PushAfterCheckStatus(pThMsg);
    }
    else
    {
        TRACE(_T("[PWR] PushAfterCheckStatus msgQInfo is NULL\n"));
    }
}

bool CPowerThread::Lock()
{
    if (GetThreadLock() == INVALID_HANDLE_VALUE)
    {
        return false;
    }
    if (GetThreadLock() == NULL)
    {
        return false;
    }
    SEM_LOCK(GetThreadLock(), INFINITE);
    return true;
}

bool CPowerThread::Flush()
{
    if (GetThreadLock() == INVALID_HANDLE_VALUE)
    {
        return false;
    }
    if (GetThreadLock() == NULL)
    {
        return false;
    }
    SEM_FLUSH(GetThreadLock());
    return true;
}

bool CPowerThread::Unlock()
{
    if (GetThreadLock() == INVALID_HANDLE_VALUE)
    {
        return false;
    }
    if (GetThreadLock() == NULL)
    {
        return false;
    }
    SEM_UNLOCK(GetThreadLock());
    return true;
}

bool CPowerThread::IsThreadQuitMsg(CString strMsg)
{
    return false;
}

bool CPowerThread::GetRunning()
{
    return threadInfo.m_bRunning;
}

void CPowerThread::SetRunning(bool Running)
{
    threadInfo.m_bRunning = Running;
}

bool CPowerThread::GetThreadStatusError()
{
    bool bRet = false;
    bRet = threadInfo.m_bErrorStatus;
    return bRet;
}

void CPowerThread::SetThreadStatusError(bool bStatus)
{
    threadInfo.m_bErrorStatus = bStatus;
}

bool CPowerThread::ErrorPause()
{
    bool bLoop = true;
    while (bLoop)
    {
        if (GetThreadStatusError() == true)
        {
            TRACE(_T("[PWR] CPowerThread GetThreadStatusError()\n"));
            ThreadSleep(THREAD_ERROR_READTIME);
            continue;
        }
        if (GetGlobalStatusError() == true)
        {
            TRACE(_T("[PWR] CPowerThread GetGlobalStatusError()\n"));
            ThreadSleep(THREAD_GLOBAL_ERROR_READTIME);
            continue;
        }
        break;
    }
    return true;
}

bool CPowerThread::Pause()
{
    Lock();
    TRACE(_T("[PWR] CPowerThread ID:0x%04X(%s) Pause Complete\n"), GetThreadID(), GetThreadName());
    SetLockComplete();
    return true;
}

bool CPowerThread::Resume()
{
    TRACE(_T("[PWR] CPowerThread ID:0x%04X(%s) Resume Complete\n"), GetThreadID(), GetThreadName());
    Unlock();
    ClearLockComplete();
    return true;
}

bool CPowerThread::SetLockComplete()
{
    bool bRet = false;
    bRet = SetEvent(GetThreadLockComplete());
    ASSERT(bRet != false);
    return bRet;
}

bool CPowerThread::ClearLockComplete()
{
    bool bRet = false;
    bRet = ResetEvent(GetThreadLockComplete());
    ASSERT(bRet != false);
    return bRet;
}

bool CPowerThread::WaitLockComplete()
{
    bool bRet = false;
    DWORD ret;
    ret = WaitForSingleObject(GetThreadLockComplete(), TIME100MS);
    if (ret == WAIT_FAILED)
    {
        return bRet;
    }
    else if (ret == WAIT_TIMEOUT)
    {
        return bRet;
    }
    else
    {
        SetEvent(GetThreadTerminate());
        Unlock();
        return true;
    }
}

bool CPowerThread::IsTerminated(unsigned loopTime)
{
    bool bRet = false;
    DWORD ret;
    ret = WaitForSingleObject(GetThreadExternalTerminate(), loopTime);
    if (ret == WAIT_FAILED)
    {
        return bRet;
    }
    else if (ret == WAIT_TIMEOUT)
    {
        return bRet;
    }
    else
    {
        return true;
    }
}

void CPowerThread::SetEnd(bool bEnd)
{
    Lock();
    threadInfo.m_bEnd = bEnd;
    Unlock();
}

bool CPowerThread::IsEnd()
{
    bool bRet = threadInfo.m_bEnd;
    return bRet;
}

void CPowerThread::SetCreate(bool bCreate)
{
    Lock();
    threadInfo.m_bCreate = bCreate;
    Unlock();
}

bool CPowerThread::IsCreate()
{
    bool bRet = threadInfo.m_bCreate;
    return bRet;
}

void CPowerThread::SetStart(bool bStart)
{
    Lock();
    threadInfo.m_bStart = bStart;
    Unlock();
}

bool CPowerThread::IsStart()
{
    bool bRet = threadInfo.m_bStart;
    return bRet;
}

void CPowerThread::SetRun(bool bRun)
{
    Lock();
    threadInfo.m_bRun = bRun;
    Unlock();
}

bool CPowerThread::IsRunning()
{
    bool bRet = threadInfo.m_bRun;
    return bRet;
}

void CPowerThread::SetSuspend(bool bSuspend)
{
    Lock();
    threadInfo.m_bSuspend = bSuspend;
    Unlock();
}

bool CPowerThread::IsSuspend()
{
    bool bRet = threadInfo.m_bSuspend;
    return bRet;
}

bool CPowerThread::SelfThreadTerminate()
{
    CThreadException* terminate = new CThreadException(T_TERMINATE);
    THROW(terminate);
    return true;
}

void CPowerThread::SetSuspendEnd(bool bSuspend)
{
    Lock();
    threadInfo.m_bSuspendEnd = bSuspend;
    Unlock();
}

bool CPowerThread::IsSuspendEnd()
{
    bool bRet = threadInfo.m_bSuspendEnd;
    return bRet;
}
