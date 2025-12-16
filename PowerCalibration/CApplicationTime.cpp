#include "pch.h"
#include "CApplicationTime.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"

#include <ctime>
#include <string>
#include <chrono>

using namespace std;
using namespace std::chrono;

CApplicationTime::CApplicationTime()
{
	TimeGet();
}

long CApplicationTime::TimeElapsed()
{
	milliseconds currentSeconds = duration_cast<milliseconds>(high_resolution_clock::now() - m_startTime);
	return static_cast<long>(currentSeconds.count());
}

void CApplicationTime::TimeGet()
{
	m_startTime = high_resolution_clock::now();
}

bool CApplicationTime::IsTimeOut(long TimeOut)
{
	if (TimeElapsed() > TimeOut)
	{
		return true;
	}
	return false;
}

PowerThreadMessage::PowerThreadMessage()
{
	m_ID = INITIALZE;
	m_Lock = CreateMutex(NULL, FALSE, NULL);
	m_Msg.Empty();
	ZeroMemory(&m_MsgSub, sizeof(m_MsgSub));
}

PowerThreadMessage::~PowerThreadMessage()
{
	CloseHandle(m_Lock);
}

CString PowerThreadMessage::GetThreadMsg()
{
	CString strRet;
	Lock();
	if (m_Msg.IsEmpty() == true)
	{
		strRet = _T("EMPTY");
	}
	else
	{
		strRet = m_Msg;
	}
	Unlock();
	return strRet;
}

void PowerThreadMessage::Lock()
{
	SEM_LOCK(m_Lock, INFINITE);
}

void PowerThreadMessage::Unlock()
{
	SEM_UNLOCK(m_Lock);
}

void PowerThreadMessage::SetThreadMsg(CString strMsg)
{
	Lock();
	m_Msg = strMsg;
	Unlock();
}

void PowerThreadMessage::SetThreadSubMsg(unsigned nSub1)
{
	Lock();
	m_MsgSub[0] = nSub1;
	Unlock();
}

void PowerThreadMessage::SetThreadSubMsg(unsigned nSub1, unsigned nSub2)
{
	Lock();
	m_MsgSub[0] = nSub1;
	m_MsgSub[1] = nSub2;
	Unlock();
}

void PowerThreadMessage::SetThreadSubMsg(unsigned nSub1, unsigned nSub2, unsigned nSub3)
{
	Lock();
	m_MsgSub[0] = nSub1;
	m_MsgSub[1] = nSub2;
	m_MsgSub[2] = nSub3;
	Unlock();
}

unsigned PowerThreadMessage::GetThreadArgMsg(unsigned arguNo)
{
	unsigned arg = NULL;
	switch (arguNo)
	{
	case 0:
		arg = m_MsgSub[0];
		break;
	case 1:
		arg = m_MsgSub[1];
		break;
	case 2:
		arg = m_MsgSub[2];
		break;
	default:
		break;
	}	
	return arg;
}

int PowerThreadMessage::GetID()
{
	int retID;
	Lock();
	if (m_ID == NULL)
	{
		Unlock();
		return NULL;
	}
	retID = m_ID;
	Unlock();
	return retID;
}

void PowerThreadMessage::SetID(unsigned nID)
{
	Lock();
	m_ID = nID;
	Unlock();
}

PowerThreadMessageQInfo::PowerThreadMessageQInfo() 
{ 
	m_Status = INITIALZE; 
	m_Lock = CreateMutex(NULL, FALSE, NULL);
	m_Queue = new queue<PowerThreadMessage*>;
}

PowerThreadMessageQInfo::~PowerThreadMessageQInfo()
{
	delete m_Queue;
}

HANDLE PowerThreadMessageQInfo::GetThreadLock()
{
	if (m_Lock == NULL)
	{
		return NULL;
	}
	if (m_Lock == INVALID_HANDLE_VALUE)
	{
		return NULL;
	}
	return m_Lock;
}

bool PowerThreadMessageQInfo::Lock()
{
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	ASSERT(GetThreadLock() != INVALID_HANDLE_VALUE);
	ASSERT(GetThreadLock() != NULL);
	SEM_LOCK(GetThreadLock(), INFINITE);
	return true;
}

bool PowerThreadMessageQInfo::Unlock()
{
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	ASSERT(GetThreadLock() != INVALID_HANDLE_VALUE);
	ASSERT(GetThreadLock() != NULL);
	SEM_UNLOCK(GetThreadLock());
	return true;
}

void PowerThreadMessageQInfo::SetStatus(int nStatus)
{
	m_Status = nStatus;
}

int PowerThreadMessageQInfo::GetStatus()
{
	int retStatus = 0;
	retStatus = m_Status;
	return retStatus;
}

bool PowerThreadMessageQInfo::IsEmpty()
{
	bool bRet = false;
	if (m_Queue != NULL)
	{
		bRet = m_Queue->empty();
	}
	return bRet;
}

void PowerThreadMessageQInfo::Pop()
{
	m_Queue->pop();
}

void PowerThreadMessageQInfo::PushAfterCheckStatus(PowerThreadMessage* pThMsg)
{
	if (GetStatus() == INITIALZE)
	{
		//TRACE(_T("[PWR] PowerThreadMessageQInfo PushAfterCheckStatus Push Skip\n"));
	}
	else
	{
		m_Queue->push(pThMsg);
		//TRACE(_T("[PWR] PowerThreadMessageQInfo(0x%04X)(%s) PushAfterCheckStatus Push\n"), pThMsg->GetID(), GetThreadNameByID(pThMsg->GetID()));
	}
}
