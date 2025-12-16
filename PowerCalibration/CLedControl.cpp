#include "pch.h"
#include <iostream>
#include "CApplicationTime.h"
#include "CLedControl.h"
#include "SerialCom.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "GlobalData.h"

CLedControl* gcLedControl;
CLedControl::CLedControl()
{
	GetId(&m_id);
	SetBaudRate(9600);
}

CLedControl::~CLedControl()
{
}

BOOL CLedControl::OnTask()
{
	TRACE("[PWR] CLedControl::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CLedControl::OnTask(LPVOID lpv)
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
			TRACE(_T("[PWR] CLedControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strAxis = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		TRACE("[PWR] CLedControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (nSubMsg[0] == 0) // All Off
		{
			LedAllOff();
		}
		else
		{

		}
		delete msgReceived;
	}
	return TRUE;
}

void CLedControl::SetBaudRate(int BaudRate)
{
	m_BaudRate = BaudRate;
}

int CLedControl::GetBaudRate()
{
	return m_BaudRate;
}

void CLedControl::OpenPort(CString strPort, int BaudRate)
{
	SetBaudRate(BaudRate);
	if (m_SerialComm.OpenPort(strPort, GetBaudRate(), 8, 0, 0, NULL, &CLedControl::ReadSerial) == FALSE)
	{
		TRACE(_T("[PWR] CLedControl Open failed.\n"));
	}
	else
	{
		TRACE(_T("[PWR] Success Comm Port(%s) %dbps Open(%d)\n"), strPort, GetBaudRate(), m_SerialComm.m_bConnected);
		//for (long indx = 0; indx < 3; ++indx)
		{
			LedAllOn();
			ThreadSleep(TIME500MS);
			LedAllOff();
			ThreadSleep(TIME500MS);
		}
	}
}

void CLedControl::ClosePort()
{
	m_SerialComm.ClosePort();
	TRACE(_T("[PWR] Success Comm Port(%s) Close\n"));
}

void CLedControl::ReadSerial(LPVOID owner, char* pBuffer, int nRead)
{
	CSerialComm* pComm = (CSerialComm*)owner;
	if (nRead < 999)
	{
		if (gcPowerLog->IsShowSerialLog() == true)
		{
			TRACE(_T("[PWR] %s ReadCount = %d (0x%X,0x%X,0x%X)\n"), pComm->m_sPortName, nRead, pBuffer[0], pBuffer[1], pBuffer[2]);
		}
	}
}

void CLedControl::WriteSerialValue(int ChNo, int Value, int All)
{
	char cSend[8];
	unsigned short uwValue1000 = 0, uwValue100 = 0, uwValue10 = 0, uwValue1 = 0;
	DWORD ret = 0;
	cSend[0] = 0x02;
	if (All > 0)
	{
		cSend[1] = 'z';
	}
	else
	{
		cSend[1] = 0x30 + ChNo;
	}
	cSend[2] = 'w';				
	uwValue1000 = Value / 1000;
	if (uwValue1000 >= 1)
	{
		cSend[3] = 0x30 + uwValue1000;
	}
	else
	{
		cSend[3] = '0';
	}
	uwValue100 = (Value - (uwValue1000 * 1000)) / 100;
	if (uwValue100 >= 1)
	{
		cSend[4] = 0x30 + uwValue100;
	}
	else
	{
		cSend[4] = '0';
	}
	uwValue10 = ((Value - (uwValue1000 * 1000)) - (uwValue100 * 100)) / 10;
	if (uwValue10 >= 1)
	{
		cSend[5] = 0x30 + uwValue10;
	}
	else
	{
		cSend[5] = '0';
	}
	uwValue1 = Value % 10;
	if (uwValue1 >= 1)
	{
		cSend[6] = 0x30 + uwValue1;
	}
	else
	{
		cSend[6] = '0';
	}
	cSend[7] = 0x03;
	ret = m_SerialComm.WriteComm(cSend, 8);
	if (gcPowerLog->IsShowSerialLog() == true)
	{
		TRACE(_T("[PWR] Write : 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x, Size:%d Ret:%d\n"),
			cSend[0], cSend[1], cSend[2], cSend[3],
			cSend[4], cSend[5], cSend[6], cSend[7], 8, ret);
	}
}

void CLedControl::WriteSerialOnOff(int ChNo, bool bOn)
{
	char cSend[4];
	DWORD ret = 0;
	cSend[0] = 0x02;		
	cSend[1] = 0x30 + ChNo;	
	if (bOn == true)
	{
		cSend[2] = 'o';		
	}
	else
	{
		cSend[2] = 'f';
	}
	cSend[3] = 0x03;	
	ret = m_SerialComm.WriteComm(cSend, 4);
	if (gcPowerLog->IsShowSerialLog() == true)
	{
		TRACE(_T("[PWR] Write : 0x%x,0x%x,0x%x,0x%x, Size:%d Ret:%d\n"), cSend[0], cSend[1], cSend[2], cSend[3], 4, ret);
	}
}

void CLedControl::LedOn(int CameraNo, int iValue1, int iValue2, int iValue3)
{
	if (GetModuleCamType(FRONT_STAGE) == 1) // 3ch and 2 Camera
	{
		if (CameraNo == CAM1)
		{
			WriteSerialValue(0, iValue1, 0);
			WriteSerialValue(1, iValue2, 0);
			WriteSerialValue(2, iValue3, 0);
		}
		else if (CameraNo == CAM2)
		{
			WriteSerialValue(3, iValue1, 0);
			WriteSerialValue(4, iValue2, 0);
			WriteSerialValue(5, iValue3, 0);
		}
	}
	else if (GetModuleCamType(FRONT_STAGE) == 2) // Bar Light
	{
		if (GetCameraCount(FRONT_STAGE) == 6) // 6 Camera
		{
			if (CameraNo == CAM1 || CameraNo == CAM2 || CameraNo == CAM3)
			{
				WriteSerialValue(0, iValue1, 0);
				WriteSerialValue(1, iValue1, 0);
			}
			else if (CameraNo == CAM4 || CameraNo == CAM5 || CameraNo == CAM6)
			{
				WriteSerialValue(2, iValue1, 0);
				WriteSerialValue(3, iValue1, 0);
			}
		}
		else // 2 Camera
		{
			if (CameraNo == CAM1)
			{
				WriteSerialValue(0, iValue1, 0);
				WriteSerialValue(1, iValue1, 0);
			}
			else if (CameraNo == CAM2)
			{
				WriteSerialValue(2, iValue1, 0);
				WriteSerialValue(3, iValue1, 0);
			}
		}
	}

	if (GetModuleCamType(REAR_STAGE) == 1) // 3ch and 2 Camera
	{
		if (CameraNo == RCAM1)
		{
			WriteSerialValue(6, iValue1, 0);
			WriteSerialValue(7, iValue2, 0);
			WriteSerialValue(8, iValue3, 0);
		}
		else if (CameraNo == RCAM2)
		{
			WriteSerialValue(9, iValue1, 0);
			WriteSerialValue(10, iValue2, 0);
			WriteSerialValue(11, iValue3, 0);
		}

	}
	else if (GetModuleCamType(REAR_STAGE) == 2) // Bar Light
	{
		if (GetCameraCount(REAR_STAGE) == 6) // 6 Camera
		{
			if (CameraNo == RCAM1 || CameraNo == RCAM2 || CameraNo == RCAM3)
			{
				WriteSerialValue(6, iValue1, 0);
				WriteSerialValue(7, iValue2, 0);
			}
			else if (CameraNo == RCAM4 || CameraNo == RCAM5 || CameraNo == RCAM6)
			{
				WriteSerialValue(8, iValue1, 0);
				WriteSerialValue(9, iValue2, 0);
			}
		}
		else // 2 Camera
		{
			if (CameraNo == RCAM1)
			{
				WriteSerialValue(6, iValue1, 0);
				WriteSerialValue(7, iValue2, 0);
			}
			else if (CameraNo == RCAM2)
			{
				WriteSerialValue(8, iValue1, 0);
				WriteSerialValue(9, iValue2, 0);
			}
		}
	}
	if (CameraNo == FHCAM)
	{
		WriteSerialValue(12, iValue1, 0);
		WriteSerialValue(13, iValue2, 0);
	}
	else if (CameraNo == RHCAM)
	{
		WriteSerialValue(14, iValue1, 0);
		WriteSerialValue(15, iValue2, 0);
	}
}

void CLedControl::LedOff(int CameraNo)
{
	LedOn(CameraNo, 0, 0, 0);
}

void CLedControl::LedAllOff()
{
	WriteSerialValue(0, 0, 1);
}

void CLedControl::LedAllOn()
{
	WriteSerialValue(0, 100, 1);
}