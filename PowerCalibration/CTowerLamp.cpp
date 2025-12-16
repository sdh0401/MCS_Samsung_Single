#include "pch.h"
#include "CTowerLamp.h"
#include "CApplicationTime.h"
#include "AxisInformation.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "Trace.h"
#include "Thread.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "LockDef.h"

CTowerLamp* gcTowerLamp;
CTowerLamp::CTowerLamp()
{
	GetId(&m_id);
	m_EntryFree = m_WorkFree = m_ExitFree = 0;
	m_Lock = CreateMutex(NULL, FALSE, NULL);
	m_OldLampMsg = 999;

}

CTowerLamp::~CTowerLamp()
{
	TRACE(_T("[PWR] ~CTowerLamp(0x%04X) Quit\n"), m_id);
}

BOOL CTowerLamp::OnTask()
{
	//TRACE("[PWR] CTowerLamp::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CTowerLamp::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	CString strAxis;
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CTowerLamp GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strAxis = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowTowerLampLog() == true)
		{
			TRACE("[PWR] CTowerLamp(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}
		if (nSubMsg[0] != LAMP_WAIT_PCB && GetRunModeNoLog() == PAUSE_MODE)
		{
			nSubMsg[0] = LAMP_WAIT_PCB;
		}
		switch (nSubMsg[0])
		{
		case LAMP_NORMAL:
			Normal();
			break;
		case LAMP_ALARM:
			Alarm();
			break;
		case LAMP_EMPTY:
			Empty();
			break;
		case LAMP_RUN:
        {
            Run(nSubMsg[0]);
            break;
        }
		case LAMP_IN_PCB:
		case LAMP_OUT_PCB:
			if (this->m_OldLampMsg == LAMP_EMPTY || this->m_OldLampMsg == LAMP_ALARM)
			{
				TRACE_FILE_FUNC_LINE_"function (CTowerLamp::Run) call in case (LAMP_IN_PCB || LAMP_OUT_PCB) is skipped due to (this->m_OldLampMsg == LAMP_EMPTY || LAMP_ALARM).");
				break;
			}
			Run(nSubMsg[0]);
			break;
		case LAMP_WAIT_PCB:
			if (this->m_OldLampMsg == LAMP_EMPTY || this->m_OldLampMsg == LAMP_ALARM)
			{
				TRACE_FILE_FUNC_LINE_"function (CTowerLamp::WaitPcb) call in case (LAMP_WAIT_PCB) is skipped due to (this->m_OldLampMsg == LAMP_EMPTY || LAMP_ALARM).");
				break;
			}
			WaitPcb(nSubMsg[1]);
			break;
		}

		if (m_OldLampMsg != nSubMsg[0])
		{
			m_OldLampMsg = nSubMsg[0];
			ThreadSleep(TIME10MS);

			if (GetTowerLampRed() == OUTON)
			{
				SendLampStatus(GetTowerLampGrn(), GetTowerLampYel(), GetTowerLampRed(), nSubMsg[2]);
			}
			else
			{
				SetMachineAlarmCode(NO_ERR);
				SendLampStatus(GetTowerLampGrn(), GetTowerLampYel(), GetTowerLampRed(), NO_ERR);
			}
		}

		delete msgReceived;
	}
	return TRUE;
}

void CTowerLamp::Normal()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLamp TowerNormal = GetTowerLamp().Normal;
	if(TowerNormal.Red == 0) 
		TowerLampRed(true);
	else 
		TowerLampRed(false);
	if (TowerNormal.Yel == 0) 
		TowerLampYel(true);
	else 
		TowerLampYel(false);
	if (TowerNormal.Grn == 0) 
		TowerLampGrn(true);
	else 
		TowerLampGrn(false);	
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Normal (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::Alarm()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLamp TowerAlarm = GetTowerLamp().Alarm;
	if (TowerAlarm.Red == 0)
		TowerLampRed(true);
	else
		TowerLampRed(false);
	if (TowerAlarm.Yel == 0)
		TowerLampYel(true);
	else
		TowerLampYel(false);
	if (TowerAlarm.Grn == 0)
		TowerLampGrn(true);
	else
		TowerLampGrn(false);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Alarm (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::Empty()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLamp TowerEmpty = GetTowerLamp().Empty;
	if (TowerEmpty.Red == 0)
		TowerLampRed(true);
	else
		TowerLampRed(false);
	if (TowerEmpty.Yel == 0)
		TowerLampYel(true);
	else
		TowerLampYel(false);
	if (TowerEmpty.Grn == 0)
		TowerLampGrn(true);
	else
		TowerLampGrn(false);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Empty (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::Run(long Msg2)
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLamp TowerRun = GetTowerLamp().Run;
	if (TowerRun.Red == 0)
		TowerLampRed(true);
	else
		TowerLampRed(false);
	if (TowerRun.Yel == 0)
		TowerLampYel(true);
	else
		TowerLampYel(false);
	if (TowerRun.Grn == 0)
		TowerLampGrn(true);
	else
		TowerLampGrn(false);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Run (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::WaitPcb(long Msg2)
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLamp TowerWait = GetTowerLamp().Wait;
	if (TowerWait.Red == 0)
		TowerLampRed(true);
	else
		TowerLampRed(false);
	if (TowerWait.Yel == 0)
		TowerLampYel(true);
	else
		TowerLampYel(false);
	if (TowerWait.Grn == 0)
		TowerLampGrn(true);
	else
		TowerLampGrn(false);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Wait (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::RedOn()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(true);
	TowerLampYel(false);
	TowerLampGrn(false);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** RedOn (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::YelOn()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(false);
	TowerLampGrn(false);
	TowerLampYel(true);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** YelOn (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::YelOn(long Msg2)
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(false);
	TowerLampGrn(false);
	TowerLampYel(true);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** YelOn (OK) Conv(%d) *****\n"), Msg2);
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::GrnOn()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(false);
	TowerLampYel(false);
	TowerLampGrn(true);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** GrnOn (OK) *****\n"));
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::GrnOn(long Msg1)
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(false);
	TowerLampYel(false);
	TowerLampGrn(true);
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** GrnOn (OK) From(%d) *****\n"), Msg1);
	}
	SEM_UNLOCK(m_Lock);
}

void CTowerLamp::AllOff()
{
	SEM_LOCK(m_Lock, INFINITE);
	TowerLampRed(false);
	TowerLampYel(false);
	TowerLampGrn(false);
	SEM_UNLOCK(m_Lock);
}