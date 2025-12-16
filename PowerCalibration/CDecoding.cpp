#include "pch.h"
#include "CDecoding.h"
#include "CDecoding1.h"
#include "CDecoding2.h"
#include "CDecoding3.h"
#include "CDecoding4.h"
#include "CPowerLog.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "AxisInformation.h"
#include "CPowerHMI.h"

CDecoding* gcDecoding;
CDecoding::CDecoding()
{
	GetId(&m_id);
}

CDecoding::~CDecoding()
{

}

BOOL CDecoding::OnTask()
{
	TRACE("[PWR] CDecoding::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CDecoding::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	long Ret = 0;
	CString strMsg;
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CDecoding GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}
		switch (nSubMsg[0])
		{
		case HMI_CMD1ST_1:
			Decoding1(strMsg, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			break;
		case HMI_CMD1ST_2:
			Decoding2(strMsg, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			break;
		case HMI_CMD1ST_3:
			Decoding3(strMsg, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			break;
		case HMI_CMD1ST_4:
			Decoding4(strMsg, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			break;
		}
		delete msgReceived;
	}
	return TRUE;
}

bool CDecoding::Decoding1(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcDecoding1)
	{
		gcDecoding1->GetId(&id);
		msgSend->SetID(id);
		if (gcDecoding1->PingThread(TIME1MS))
		{
			gcDecoding1->Event((LPVOID)msgSend);
		}
	}
	return true;
}

bool CDecoding::Decoding2(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcDecoding2)
	{
		gcDecoding2->GetId(&id);
		msgSend->SetID(id);
		if (gcDecoding2->PingThread(TIME1MS))
		{
			gcDecoding2->Event((LPVOID)msgSend);
		}
	}
	return true;
}

bool CDecoding::Decoding3(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcDecoding3)
	{
		gcDecoding3->GetId(&id);
		msgSend->SetID(id);
		if (gcDecoding3->PingThread(TIME1MS))
		{
			gcDecoding3->Event((LPVOID)msgSend);
		}
	}
	return true;
}

bool CDecoding::Decoding4(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcDecoding4)
	{
		gcDecoding4->GetId(&id);
		msgSend->SetID(id);
		if (gcDecoding4->PingThread(TIME1MS))
		{
			gcDecoding4->Event((LPVOID)msgSend);
		}
	}
	return true;
}