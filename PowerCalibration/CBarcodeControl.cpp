#include "pch.h"
#include <iostream>
#include "CApplicationTime.h"
#include "AxisInformation.h"
#include "CBarcodeControl.h"
#include "SerialCom.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "GlobalData.h"
#include "LockDef.h"

CBarcodeControl* gcBarcodeControl;
CBarcodeControl::CBarcodeControl()
{
	m_Lock = CreateMutex(NULL, FALSE, NULL);
	m_ReceiveBuffer.Empty();

	GetId(&m_id);
	SetBaudRate(BARCODE_CONTROL_BAUDRATE);
}

CBarcodeControl::~CBarcodeControl()
{
}

BOOL CBarcodeControl::OnTask()
{
	TRACE("[PWR] CBarcodeControl::OnTask Thread(0x%04X)\n", m_id);
	return TRUE;
}

BOOL CBarcodeControl::OnTask(LPVOID lpv)
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
			TRACE(_T("[PWR] CBarcodeControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strAxis = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		TRACE("[PWR] CBarcodeControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (nSubMsg[0] == 0) // All Off
		{
			TriggerOff();
		}
		delete msgReceived;
	}
	return TRUE;
}

void CBarcodeControl::Lock()
{
	SEM_LOCK(m_Lock, INFINITE);
}

void CBarcodeControl::Unlock()
{
	SEM_UNLOCK(m_Lock);
}

void CBarcodeControl::SetBaudRate(int BaudRate)
{
	m_BaudRate = BaudRate;
}

int CBarcodeControl::GetBaudRate()
{
	return m_BaudRate;
}

bool CBarcodeControl::OpenPort(CString strPort, int BaudRate)
{
	if (m_SerialComm.m_bConnected == FALSE)
	{
		SetBaudRate(BaudRate);
		if (m_SerialComm.OpenPort(strPort, GetBaudRate(), 8, 0, 0, NULL, &CBarcodeControl::ReadSerial) == FALSE)
		{
			TRACE(_T("[PWR] CBarcodeControl Open failed.\n"));
			return false;
		}
		else
		{
			TRACE(_T("[PWR] Success Comm Port(%s) %dbps Open(%d)\n"), strPort, GetBaudRate(), m_SerialComm.m_bConnected);
			ThreadSleep(TIME100MS);
			SetDefaultCognexParameter();
			return true;
		}
	}
	else
	{
		TRACE(_T("[PWR] CBarcodeControl Already Open.\n"));
		return true;
	}
}

void CBarcodeControl::ClosePort()
{
	SetDefaultCognexParameter();
	ThreadSleep(TIME100MS);
	m_SerialComm.ClosePort();
	TRACE(_T("[PWR] Comm Port(%s) Close\n"), m_SerialComm.m_sPortName);
}

void CBarcodeControl::ReadSerial(LPVOID owner, char* pBuffer, int nRead)
{
	CSerialComm* pComm = (CSerialComm*)owner;
	CString str;
	if (nRead < 999)
	{
		for (long Cnt = 0; Cnt < nRead; ++Cnt)
		{
			if (' ' <= pBuffer[Cnt] && pBuffer[Cnt] <= '~')
			{
				gcBarcodeControl->m_ReceiveBuffer.AppendFormat(_T("%c"), pBuffer[Cnt]);
				str.AppendFormat(_T("%c"), pBuffer[Cnt]);
			}
			else if(pBuffer[Cnt] == '\n')
			{
				gSetBarcodeString(gcBarcodeControl->m_ReceiveBuffer);
				gcBarcodeControl->m_ReceiveBuffer.Empty();
			}		
		}

		if (gcPowerLog->IsShowSerialLog() == true)
		{
			TRACE(_T("[PWR] %s ReadCount = %d(%s)\n"), pComm->m_sPortName, nRead, str);
		}
		
	}
}

void CBarcodeControl::WriteSerialValue(int ChNo, int Value, int All)
{
}

void CBarcodeControl::WriteSerialOnOff(int ChNo, bool bOn)
{
}

void CBarcodeControl::TriggerOn()
{
	//char cSend[3];
	//DWORD ret = 0;
	//cSend[0] = 0x16;
	//cSend[1] = 'T';
	//cSend[2] = '\r';
	//ret = m_SerialComm.WriteComm(cSend, 3);

	char cSend[50];

	cSend[0] = '|';
	cSend[1] = '|';
	cSend[2] = '>';
	cSend[3] = 't';
	cSend[4] = 'r';
	cSend[5] = 'i';
	cSend[6] = 'g';
	cSend[7] = 'g';
	cSend[8] = 'e';
	cSend[9] = 'r';
	cSend[10] = ' ';
	cSend[11] = 'o';
	cSend[12] = 'n';
	cSend[13] = '.';
	cSend[14] = '\r';
	cSend[15] = '\n';

	DWORD ret;
	ret = m_SerialComm.WriteComm(cSend, 16);
}

void CBarcodeControl::TriggerOff()
{
	char cSend[3];
	DWORD ret = 0;
	cSend[0] = 0x16;
	cSend[1] = 'U';
	cSend[2] = '\r';
	ret = m_SerialComm.WriteComm(cSend, 3);
}

void CBarcodeControl::SetTriggerType(long n)
{
	char cSend[100];
	DWORD ret = 0;


	if (n == 0)
	{
		cSend[0] = '|';
		cSend[1] = '|';
		cSend[2] = '>';
		cSend[3] = 'T';
		cSend[4] = 'R';
		cSend[5] = 'I';
		cSend[6] = 'G';
		cSend[7] = 'G';
		cSend[8] = 'E';
		cSend[9] = 'R';
		cSend[10] = ' ';
		cSend[11] = 'O';
		cSend[12] = 'F';
		cSend[13] = 'F';
		cSend[14] = '\r';
		cSend[15] = '\n';
		ret = m_SerialComm.WriteComm(cSend, 16);

	}
	else
	{
		cSend[0] = '|';
		cSend[1] = '|';
		cSend[2] = '>';
		cSend[3] = 'T';
		cSend[4] = 'R';
		cSend[5] = 'I';
		cSend[6] = 'G';
		cSend[7] = 'G';
		cSend[8] = 'E';
		cSend[9] = 'R';
		cSend[10] = ' ';
		cSend[11] = 'O';
		cSend[12] = 'N';
		cSend[13] = '\r';
		cSend[14] = '\n';
		ret = m_SerialComm.WriteComm(cSend, 15);
	}


	return;


	// 0 : single
	// 4 : self
	if (0 <= n && n <= 5) 
	{
		cSend[0] = '|';
		cSend[1] = '|';
		cSend[2] = '>';
		cSend[3] = 's';
		cSend[4] = 'e';
		cSend[5] = 't';
		cSend[6] = ' ';
		cSend[7] = 't';
		cSend[8] = 'r';
		cSend[9] = 'i';
		cSend[10] = 'g';
		cSend[11] = 'g';
		cSend[12] = 'e';
		cSend[13] = 'r';
		cSend[14] = '.';
		cSend[15] = 't';
		cSend[16] = 'y';
		cSend[17] = 'p';
		cSend[18] = 'e';
		cSend[19] = ' ';
		cSend[20] = '0' + (char)n;
		cSend[21] = '\r';
		cSend[22] = '\n';

		ret = m_SerialComm.WriteComm(cSend, 23);
	}
}

void CBarcodeControl::SetDataMatrixEnable(bool Enable)
{
	return;

	DWORD ret = 0;
	char strCmd[] = "||>SET SYMBOL.DATAMATRIX ";
	char strSend[1024] = "empty";
	long copySize = sizeof(strCmd) - 1;
	long sendSize = 0;
	for (sendSize = 0; sendSize < copySize; sendSize++)
	{
		strSend[sendSize] = strCmd[sendSize];
	}
	if (Enable == true)
	{
		strSend[sendSize++] = 'O';
		strSend[sendSize++] = 'N';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}
	else
	{
		strSend[sendSize++] = 'O';
		strSend[sendSize++] = 'F';
		strSend[sendSize++] = 'F';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}

	ret = m_SerialComm.WriteComm(strSend, sendSize++);
}

void CBarcodeControl::SetQRCodeEnable(bool Enable)
{
	return;

	DWORD ret = 0;
	char strCmd[] = "||>SET SYMBOL.QR ";
	char strSend[1024] = "empty";
	long copySize = sizeof(strCmd) - 1;
	long sendSize = 0;
	for (sendSize = 0; sendSize < copySize; sendSize++)
	{
		strSend[sendSize] = strCmd[sendSize];
	}
	if (Enable == true)
	{
		strSend[sendSize++] = 'O';
		strSend[sendSize++] = 'N';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}
	else
	{
		strSend[sendSize++] = 'O';
		strSend[sendSize++] = 'F';
		strSend[sendSize++] = 'F';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}

	ret = m_SerialComm.WriteComm(strSend, sendSize++);
}

void CBarcodeControl::LightOnOff(bool On)
{

	DWORD ret = 0;
	char strCmd[] = "||>SET LIGHT.AIMER ";
	char strSend[1024] = "empty";
	long copySize = sizeof(strCmd) - 1;
	long sendSize = 0;
	for (sendSize = 0; sendSize < copySize; sendSize++)
	{
		strSend[sendSize] = strCmd[sendSize];
	}
	if (On == true)
	{
		strSend[sendSize++] = '1';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}
	else
	{
		strSend[sendSize++] = '0';
		strSend[sendSize++] = '\r';
		strSend[sendSize++] = '\n';
	}

	ret = m_SerialComm.WriteComm(strSend, sendSize++);
}


void CBarcodeControl::SetBarcode(CString strBarcode)
{
	Lock();
	m_Barcode = strBarcode;
	TRACE(_T("[PWR] SetBarcode : %s"), strBarcode);
	Unlock();
}

CString CBarcodeControl::GetBarcode()
{
	CString strBar;

	Lock();
	strBar = m_Barcode;
	Unlock();

	return strBar;
}

void CBarcodeControl::InitBarcode()
{
	Lock();
	m_Barcode.Empty();
	Unlock();
}

void CBarcodeControl::SetDefaultCognexParameter()
{
	TRACE(_T("[PWR] SetDefaultCognexParameter\n"));
	SetTriggerType(0);
	SetDataMatrixEnable(true);
	SetQRCodeEnable(true);
	LightOnOff(true);	
}

bool CBarcodeControl::GetComPortStatus()
{
	return m_SerialComm.m_bConnected;
}

CString CBarcodeControl::InspectBarcodeCognex(long Type, long TimeOut)
{
	CString strBarcodeResult = _T(STRING_NULL_BARCODE);
	CString strBarcode1st = _T(STRING_NULL_BARCODE);
	CString strBarcode2nd = _T(STRING_NULL_BARCODE);
	ULONGLONG TimeGet = 0, ElapsedTime = 0;
	bool Read1st = false, Read2nd = false;	

	TimeGet = _time_get();

	SetTriggerType(0);
	//if (Type == 0)
	//{
	//	SetDataMatrixEnable(true);
	//	SetQRCodeEnable(false);
	//}
	//else
	//{
	//	SetDataMatrixEnable(false);
	//	SetQRCodeEnable(true);
	//}
	InitBarcode();

	SetTriggerType(4);
	ThreadSleep(TIME30MS);
	
	while (1)
	{
		if (_time_elapsed(TimeGet) > TIME5000MS)
		{
			TRACE(_T("[PWR] InspectBarcodeCognex Type:%d TimeOut\n"), Type);
			break;
		}

		if (Read1st == false)
		{
			strBarcode1st = GetBarcode();
			if (strBarcode1st.IsEmpty() == false)
			{
				InitBarcode();
				Read1st = true;
				Read2nd = true;
				break;
			}
		}
		ThreadSleep(TIME10MS);

		//else 
		//{
		//	strBarcode2nd = GetBarcode();
		//	if (strBarcode2nd.IsEmpty() == false)
		//	{
		//		InitBarcode();
		//		if (strBarcode1st.CompareNoCase(strBarcode2nd) == 0)
		//		{
		//			Read2nd = true;
		//			break;
		//		}
		//		else
		//		{
		//			TRACE(_T("[PWR] InspectBarcodeCognex Type:%d Compare fail. 1st:%s 2nd:%d \n"), Type, strBarcode1st, strBarcode2nd);
		//			Read1st = false;
		//		}
		//	}
		//}
	}
	SetTriggerType(0);

	if (Read1st == true && Read2nd == true)
	{
		strBarcodeResult = strBarcode1st;
		TRACE(_T("[PWR] InspectBarcodeCognex Type:%d Result:%s Time:%dms\n"), Type, strBarcodeResult, _time_elapsed(TimeGet));
	}
	else
	{
		strBarcodeResult = _T(STRING_NULL_BARCODE);
	}

	return strBarcodeResult;
}
