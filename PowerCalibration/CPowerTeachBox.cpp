#include "pch.h"
#include "CPowerTeachBox.h"
#include "CApplicationTime.h"
#include "AxisInformation.h"
#include "VisionData.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "Trace.h"
#include "Thread.h"
//#include "ErrorCode.h"

CPowerTeachBox* gcPowerTeachBox;
CPowerTeachBox::CPowerTeachBox()
{
	GetId(&m_id);
}

CPowerTeachBox::~CPowerTeachBox()
{
	TRACE(_T("[PWR] ~CPowerTeachBox(0x%04X) Quit\n"), m_id);
}

BOOL CPowerTeachBox::OnTask()
{
	TRACE("[PWR] CPowerTeachBox::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CPowerTeachBox::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	CString strAxis;
	// Dir(0:Stop, 1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High), Jog(0:Relative, 1:Velocity)
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CPowerTeachBox GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strAxis = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		TRACE("[PWR] CPowerTeachBox(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (nSubMsg[0] == 0) // Stop
		{
			StopAxis(strAxis);
		}
		else
		{
			if (nSubMsg[2] == 0) // Relative
			{
				StartMove(strAxis, nSubMsg[0], nSubMsg[1]); // Axis, Dir(1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High)
			}
			else // Jog
			{
				StartJog(strAxis, nSubMsg[0], nSubMsg[1]); // Axis, Dir(1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High)
			}
		}
		delete msgReceived;
	}
	return TRUE;
}



BOOL CPowerTeachBox::DirectControl(CString Msg, signed SubMsg1, signed SubMsg2, signed SubMsg3)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	CString strAxis;
	// Dir(0:Stop, 1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High), Jog(0:Relative, 1:Velocity)


		strAxis = Msg; // Axis
		nSubMsg[0] = SubMsg1;
		nSubMsg[1] = SubMsg2;
		nSubMsg[2] = SubMsg3;

		//dont' use cout here, output could be broken up due to threading
		TRACE("[PWR] CPowerTeachBox(0x%04X) DirectControl Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (nSubMsg[0] == 0) // Stop
		{
			StopAxis(strAxis);
		}
		else
		{
			if (nSubMsg[2] == 0) // Relative
			{
				StartMove(strAxis, nSubMsg[0], nSubMsg[1]); // Axis, Dir(1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High)
			}
			else // Jog
			{
				StartJog(strAxis, nSubMsg[0], nSubMsg[1]); // Axis, Dir(1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High)
			}
		}
	
	return TRUE;
}

/*
// Axis, Dir(1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High)
*/
void CPowerTeachBox::StartMove(CString strAxis, unsigned Dir, unsigned Speed) 
{
	long Err = NO_ERR;
	double dblPosition = 0.0, dblDir = -1, Inpos = 0.003, Ratio = 0.5;
	long TimeOut = TIME5000MS, Ms = TIME100MS;
	Point_XY LastPt;
	if (Dir == 2) // Plus
	{
		dblDir = 1;
	}
	if (Speed == 2) // High
	{
		dblPosition = 1.0 * dblDir;
	}
	else if (Speed == 1) // Middle
	{
		dblPosition = 0.1 * dblDir;
	}
	else // Low
	{
		dblPosition = 0.01 * dblDir;
	
	}
	Err = StartTeachMoveWaitDelayedInposition(strAxis, Ratio, TimeOut, dblPosition, Inpos, Ms, true);
	if (Err != NO_ERR)
	{
		Err = SendAlarm(Err, _T("StartMove StartTeachMoveWaitDelayedInposition"));
	}
	TRACE(_T("[PWR] StartMove(%s) Dir(%d) Speed(%d) Err:%d\n"), strAxis, Dir, Speed, Err);
	LastPt = gReadGantryPosition(FRONT_GANTRY);
	//if(IsGantryAxis(strAxis) == true)
	//{
	//	gLiveOn(FHCAM);
	//}
}

void CPowerTeachBox::StartJog(CString strAxis, unsigned Dir, unsigned Speed)
{
	long Err = NO_ERR;
	long AxisIndex = NON;
	double dblPosition = 0.0, dblDir = -1;
	if (Dir == 2) // Plus
	{
		dblDir = 1;
	}
	//if (IsGantryAxis(strAxis) == true)
	//{
	//	gLiveOn(FHCAM);
	//}


	if (Dir == 2) // Plus
	{
		if (IsNearSWLimitPlus(strAxis) == true)
		{
			TRACE(_T("[PWR] StartJog(%s) Dir(%d) Speed(%d) Skip : plus limit close  \n"), strAxis, Dir, Speed);
			return;
		}
	}
	else
	{
		if (IsNearSWLimitMinus(strAxis) == true)
		{
			TRACE(_T("[PWR] StartJog(%s) Dir(%d) Speed(%d) Skip : minus limit close  \n"), strAxis, Dir, Speed);
			return;
		}		
	}

	JogInfo* jogCmd = new JogInfo();
	if (strAxis == GetAxisX(FRONT_GANTRY) || strAxis == GetAxisY1(FRONT_GANTRY))
	{
		if (Speed == 2) // High
		{
			jogCmd->Acc = 300000.0;
			jogCmd->Dec = 300000.0;
			jogCmd->MaxVel = 30000.0 * dblDir;
		}
		else if (Speed == 1) // Middle
		{
			jogCmd->Acc = 150000.0;
			jogCmd->Dec = 150000.0;
			jogCmd->MaxVel = 15000.0 * dblDir;
		}
		else // Low
		{
			jogCmd->Acc = 100000.0;
			jogCmd->Dec = 100000.0;
			jogCmd->MaxVel = 10000.0 * dblDir;
		}
	}
	else
	{
		AxisIndex = GetAxisIndexFromAliasName(strAxis);
		if (AxisIndex >= (long)PowerAxis::FZ1 || AxisIndex <= (long)PowerAxis::FZ6)
		{
			if (Speed == 2) // High
			{
				jogCmd->Acc = 200000.0;
				jogCmd->Dec = 200000.0;
				jogCmd->MaxVel = 20000.0 * dblDir;
			}
			else if (Speed == 1) // Middle
			{
				jogCmd->Acc = 100000.0;
				jogCmd->Dec = 100000.0;
				jogCmd->MaxVel = 10000.0 * dblDir;
			}
			else // Low
			{
				jogCmd->Acc = 50000.0;
				jogCmd->Dec = 50000.0;
				jogCmd->MaxVel = 5000.0 * dblDir;
			}
		}
		else if (AxisIndex >= (long)PowerAxis::FW1 || AxisIndex <= (long)PowerAxis::FW6)
		{
			if (Speed == 2) // High
			{
				jogCmd->Acc = 1800000.0;
				jogCmd->Dec = 1800000.0;
				jogCmd->MaxVel = 180000.0 * dblDir;
			}
			else if (Speed == 1) // Middle
			{
				jogCmd->Acc = 900000.0;
				jogCmd->Dec = 900000.0;
				jogCmd->MaxVel = 90000.0 * dblDir;
			}
			else // Low
			{
				jogCmd->Acc = 300000.0;
				jogCmd->Dec = 300000.0;
				jogCmd->MaxVel = 30000.0 * dblDir;
			}
		}
		else if(AxisIndex == (long)PowerAxis::FCONV)
		{
			if (Speed == 2) // High
			{
				jogCmd->Acc = 500000.0;
				jogCmd->Dec = 500000.0;
				jogCmd->MaxVel = 50000.0 * dblDir;
			}
			else if (Speed == 1) // Middle
			{
				jogCmd->Acc = 200000.0;
				jogCmd->Dec = 200000.0;
				jogCmd->MaxVel = 20000.0 * dblDir;
			}
			else // Low
			{
				jogCmd->Acc = 100000.0;
				jogCmd->Dec = 100000.0;
				jogCmd->MaxVel = 10000.0 * dblDir;
			}
		}
		else if (AxisIndex == (long)PowerAxis::FPUZ)
		{
			if (Speed == 2) // High
			{
				jogCmd->Acc = 300000.0;
				jogCmd->Dec = 300000.0;
				jogCmd->MaxVel = 30000.0 * dblDir;
			}
			else if (Speed == 1) // Middle
			{
				jogCmd->Acc = 150000.0;
				jogCmd->Dec = 150000.0;
				jogCmd->MaxVel = 15000.0 * dblDir;
			}
			else // Low
			{
				jogCmd->Acc = 100000.0;
				jogCmd->Dec = 100000.0;
				jogCmd->MaxVel = 10000.0 * dblDir;
			}
		}
	}
	Err = StartOneJog(strAxis, *jogCmd);
	if (Err != NO_ERR)
	{
		Err = SendAlarm(Err, _T("StartJog StartOneJog"));
	}
	TRACE(_T("[PWR] StartJog(%s) Dir(%d) Speed(%d) Err:%d\n"), strAxis, Dir, Speed, Err);
	delete jogCmd;
	jogCmd = NULL;
}

void CPowerTeachBox::StopAxis(CString strAxis)
{
	unsigned err = ErrorCode::None;
	Point_XY LastPt;
	err = StopOne(strAxis);
	err = WaitOneIdle(strAxis, TIME5000MS);
	TRACE(_T("[PWR] Stop(%s) Ret:%d\n"), strAxis, err);
	LastPt = gReadGantryPosition(FRONT_GANTRY);	
}
