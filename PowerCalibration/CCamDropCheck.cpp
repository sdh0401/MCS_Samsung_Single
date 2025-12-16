#include "pch.h"
#include "CCamDropCheck.h"
#include "AxisInformation.h"
#include "DefineThreadLoopTime.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"
#include "VisionData.h"

CCamDropCheck* gcCamDropCheck[MAXGANTRY];
CCamDropCheck::CCamDropCheck(long CamTable)
{
	m_Table = CamTable;
	m_DropResult = NO_ERR;
	m_ProcessComplete = false;
	GetId(&m_id);

}

CCamDropCheck::~CCamDropCheck()
{
}

BOOL CCamDropCheck::OnTask()
{
	return 0;
}

BOOL CCamDropCheck::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	long Err = NO_ERR;
	CString strMsg, strSendMsg;
	CString strFunc(__func__);

	strSendMsg.Format(_T("0"));
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] %s GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), strFunc, nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}

		long CamNo[MAXCAMNO] = { 0, };
		long Result[MAXCAMNO] = { 1, };

		if (GetTable() == FRONT_STAGE)
		{
			CamNo[0] = CAM1;
			CamNo[1] = CAM2;
		}
		else
		{
			CamNo[0] = RCAM1;
			CamNo[1] = RCAM2;
		}

		InitDropResult();

		Err = gDropCheckProcess(CamNo, 2, Result);

		gLedOn(CamNo[0], 0, 0, 0);
		gLedOn(CamNo[1], 0, 0, 0);

		if (Err == NO_ERR)
		{
			if (IsAccTest() == true)
			{
				SetDropResult(NO_ERR);

			}
			else if (Result[0] == 0 && Result[1] == 0)
			{
				SetDropResult(NO_ERR);
			}
			else
			{
				SetDropResult(FCAMERA_DROP_ERROR + GetTable());
			}
		}
		delete msgReceived;
	}
	return TRUE;
}

long CCamDropCheck::GetTable()
{
	return m_Table;
}

void CCamDropCheck::InitDropResult()
{
	m_ProcessComplete = false;
}

void CCamDropCheck::SetDropResult(long result)
{
	m_DropResult = result;
	m_ProcessComplete = true;
}

bool CCamDropCheck::GetDropResult(long* result)
{
	*result = m_DropResult;
	return m_ProcessComplete;
}
