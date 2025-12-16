#pragma once
#include <chrono>
#include "GlobalDefine.h"
#include <queue>

using namespace std;
using namespace std::chrono;

class CApplicationTime
{
private:
	high_resolution_clock::time_point m_startTime;
public:
	long TimeElapsed();
	void TimeGet();
	bool IsTimeOut(long TimeOut);
	CApplicationTime();
};

class PowerThreadMessage
{
public:
	~PowerThreadMessage();
	PowerThreadMessage();
	void Unlock();
	void Lock();
	CString GetThreadMsg();
	void SetThreadMsg(CString strMsg);
	void SetThreadSubMsg(unsigned nSub1);
	void SetThreadSubMsg(unsigned nSub1, unsigned nSub2);
	void SetThreadSubMsg(unsigned nSub1, unsigned nSub2, unsigned nSub3);
	unsigned GetThreadArgMsg(unsigned arguNo);
	int GetID();
	void SetID(unsigned nID);
	HANDLE m_Lock;
private:
	unsigned m_ID;
	CString m_Msg;
	unsigned m_MsgSub[MAXSUBMSGNO];
};

class PowerThreadMessageQInfo
{
public:
	PowerThreadMessageQInfo();
	~PowerThreadMessageQInfo();
	HANDLE GetThreadLock();
	bool Unlock();
	bool Lock();
	void SetStatus(int nStatus);
	int GetStatus();
	void PushAfterCheckStatus(PowerThreadMessage* pThMsg);
	bool IsEmpty();
	void Pop();
	HANDLE m_Lock;
	queue<PowerThreadMessage*>* m_Queue;
	int m_Status;
};