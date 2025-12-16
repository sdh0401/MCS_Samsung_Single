#include "pch.h"
#include <iostream>
#include "CApplicationTime.h"
#include "CVirtualSerialControl.h"
#include "SerialCom.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "GlobalData.h"

CVirtualSerialControl* gcVirtualSerialControl;
CVirtualSerialControl::CVirtualSerialControl()
{
	GetId(&m_id);
	SetBaudRate(9600);
}

CVirtualSerialControl::~CVirtualSerialControl()
{
}

BOOL CVirtualSerialControl::OnTask()
{
	TRACE("[PWR] CLedControl::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CVirtualSerialControl::OnTask(LPVOID lpv)
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
			TRACE(_T("[PWR] CVirtualSerialControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strAxis = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		TRACE("[PWR] CVirtualSerialControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);

		delete msgReceived;
	}
	return TRUE;
}

void CVirtualSerialControl::SetBaudRate(int BaudRate)
{
	m_BaudRate = BaudRate;
}

int CVirtualSerialControl::GetBaudRate()
{
	return m_BaudRate;
}

void CVirtualSerialControl::OpenPort(CString strPort, int BaudRate)
{
	SetBaudRate(BaudRate);
	if (m_SerialComm.OpenPort(strPort, GetBaudRate(), 8, 0, 0, NULL, &CVirtualSerialControl::ReadSerial) == FALSE)
	{
		TRACE(_T("[PWR] CVirtualSerialControl Open failed.\n"));
	}
	else
	{
		TRACE(_T("[PWR] Success Comm Port(%s) %dbps Open(%d)\n"), strPort, GetBaudRate(), m_SerialComm.m_bConnected);
	}
}

void CVirtualSerialControl::ClosePort()
{
	m_SerialComm.ClosePort();
	TRACE(_T("[PWR] Success Comm Port(%s) Close\n"));
}

void CVirtualSerialControl::ReadSerial(LPVOID owner, char* pBuffer, int nRead)
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

void CVirtualSerialControl::WriteSerialValue(CString strBarcode)
{
	DWORD ret = 0;
	CStringA strBarcodeA = CStringA(strBarcode);
	char* pChar = strBarcodeA.GetBuffer();
	long ArraySize = strBarcodeA.GetLength() + 1;
	char chBarcode[1024] = { 0 };
	memcpy(chBarcode, strBarcodeA.GetBuffer(), strBarcodeA.GetLength());
	ret = m_SerialComm.WriteComm(chBarcode, strBarcodeA.GetLength());
	if (gcPowerLog->IsShowSerialLog() == true)
	{
		TRACE(_T("[PWR] Write : 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x, Size:%d Ret:%d\n"),
			chBarcode[0], chBarcode[1], chBarcode[2], chBarcode[3],
			chBarcode[4], chBarcode[5], chBarcode[6], chBarcode[7], 
			strBarcodeA.GetLength(),
			ret);
	}
}

void CVirtualSerialControl::WriteBarcode(CString strBarcode)
{
	WriteSerialValue(strBarcode);
}