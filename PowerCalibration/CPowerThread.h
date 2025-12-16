#pragma once
#include "GlobalDefine.h"
#include "CApplicationTime.h"

class CPowerThread
{
public:
	CPowerThread();
	CPowerThread(HANDLE h_Terminate);
	CPowerThread(HANDLE h_Terminate, HANDLE h_HostMutex);
	CPowerThread(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	~CPowerThread();
	//HANDLE GetThreadStart();
	//HANDLE GetThreadWaitEnd();
	//void WorkStart();
	//void WaitWorkStart();
	//void WorkEnd(DWORD dwMili);
	//void WaitWorkEnd(DWORD dwMili);
	void SetThreadHandle(HANDLE nHandle);
	HANDLE GetThreadHandle();
	void SetThreadStartSync(HANDLE nStartSync);
	void SetThreadTerminate(HANDLE nTerminated);
	void SetThreadExternalTerminate(HANDLE nTerminated);
	HANDLE GetThreadStartSync();
	HANDLE GetThreadTerminate();
	HANDLE GetThreadLockComplete();
	HANDLE GetThreadExternalTerminate();
	void SetThreadID(int nID);
	unsigned GetThreadID();
	void SetThreadName(CString strName);
	CString GetThreadName();
	void InitThreadVariable();
	bool Lock();
	bool Unlock();
	bool Flush();
	void InitThreadData();
	void InitThreadData(HANDLE h_Terminate);
	void InitThreadData(HANDLE h_Terminate, HANDLE h_HostMutex);
	void InitThreadData(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	void WaitThreadLoop();
	void StopThread();
	void WaitForSingleObjectTime(HANDLE nHandle, int nTimeCount, int nMaxTime);
	bool ExitThreadLoop();
	bool IsAliveThread();
	bool IsAliveThread(THREAD_STRUCT* threadInfo);
	void CloseThread();
	void CloseMutex();
	CString CheckMessageQueue(LPVOID wParam);
	CString CheckMessageQueue(LPVOID wParam, signed* arg);
	bool IsThreadQuitMsg(CString strMsg);
	void SetMsgQueueStatus(int nStatus);
	void PushAfterCheckStatus(PowerThreadMessage* pThMsg);
	bool Pause();
	bool ErrorPause();
	bool Resume();
	bool SetLockComplete();
	bool ClearLockComplete();
	bool WaitLockComplete();
	bool IsTerminated(unsigned loopTime);
	void SetEnd(bool bEnd);
	bool IsEnd();
	void SetCreate(bool bStart);
	bool IsCreate();
	void SetStart(bool bStart);
	bool IsStart();
	void SetRun(bool bRun);
	bool IsRunning();
	void SetSuspend(bool bSuspend);
	bool IsSuspend();
	void SetSuspendEnd(bool bSuspend);
	bool IsSuspendEnd();
	bool SelfThreadTerminate();
	void SetThreadStatusError(bool bStatus);
	bool GetThreadStatusError();
	bool GetRunning();
	void SetRunning(bool Runnig);

private:
	void SetThreadLock(HANDLE nLock);
	HANDLE GetThreadLock();
	THREAD_STRUCT threadInfo;
	PowerThreadMessageQInfo* msgQInfo;
};

