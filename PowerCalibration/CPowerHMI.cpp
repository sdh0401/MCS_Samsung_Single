#include "pch.h"
#include "CPowerHMI.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "CPowerClient.h"
#include "CPowerLog.h"

CPowerHMI* gcCPowerHMI;
CPowerHMI::CPowerHMI()
{
	GetId(&m_id);
}

CPowerHMI::~CPowerHMI()
{
}

BOOL CPowerHMI::OnTask()
{
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CPowerHMI::OnTask Thread(0x%04X)\n", m_id);
	}
	return TRUE;
}

BOOL CPowerHMI::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	CString strMsg;
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CPowerHMI GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CPowerHMI(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}
		SendCommand(nSubMsg[0], nSubMsg[1], nSubMsg[2], strMsg);
		delete msgReceived;
	}
	return TRUE;
}

bool CPowerHMI::SendCommand(long Sub1, long Sub2, long Sub3, CString strCmd)
{
	bool bRet = false;
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] CPowerHMI SendCommand SubMsg(%d,%d,%d)(%s)\n"), Sub1, Sub2, Sub3, strCmd);
	}
	bRet = gcPowerClient->SendCommandTo(ID_HMI, Sub1, Sub2, Sub3, strCmd, NO_WAIT);
	return bRet;
}
