#include "pch.h"
#include "SerialCom.h"
#include "Trace.h"
#include "CPowerLog.h"

//--- 클래스 생성자
CSerialComm::CSerialComm()
{
	m_owner = NULL;
	m_serialByteFuncPtr = NULL;
	m_serialStringFuncPtr = NULL;
	m_hThreadSerialReceive = INVALID_HANDLE_VALUE;
	m_bConnected = FALSE;
}

CSerialComm::~CSerialComm()
{
	if(m_bConnected)
	{
		ClosePort();
	}
}

// 포트 sPortName을 dwBaud 속도로 연다.
// ThreadWatchComm 함수에서 포트에 무언가 읽혔을 때 MainWnd에 알리기
// 위해 WM_SERIAL_COMM_READ메시지를 보낼때 같이 보낼 wPortID값을 전달 받는다.
BOOL CSerialComm::OpenPort(CString strPortName, DWORD dwBaud, BYTE byData, BYTE byStop, BYTE byParity , LPVOID owner,  SERIALBYTEFUNCPTR funcptr, int nTimeOut)
{
	// Local 변수.
	COMMTIMEOUTS	timeouts;
	DCB				dcb;
	DWORD			dwThreadID;

	// overlapped structure 변수 초기화.
	m_osRead.Offset = 0;
	m_osRead.OffsetHigh = 0;
	//--> Read 이벤트 생성에 실패..
	if ( !(m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)) ) 	
	{
		return FALSE;
	}
	//m_nT1 = nTimeOut1;
	m_nT2 = nTimeOut;

	m_osWrite.Offset = 0;
	m_osWrite.OffsetHigh = 0;
	//--> Write 이벤트 생성에 실패..
	if (! (m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)))
	{
		return FALSE;
	}
	m_sPortName = strPortName;
	m_hComm = CreateFile(m_sPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	//===== 포트 상태 설정. =====

	// EV_RXCHAR event 설정...데이터가 들어오면.. 수신 이벤트가 발생하게끔..
	SetCommMask( m_hComm, EV_RXCHAR);	
	// InQueue, OutQueue 크기 설정.
	SetupComm( m_hComm, MAX_SERIAL_BUFF_SIZE, MAX_SERIAL_BUFF_SIZE);	
	// 포트 비우기.
	PurgeComm( m_hComm, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
	// timeout 설정.
	timeouts.ReadIntervalTimeout = 0xFFFFFFFF;
	//timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;

	timeouts.WriteTotalTimeoutMultiplier = 0;// 2 * CBR_9600 / dwBaud;
	timeouts.WriteTotalTimeoutConstant = 0;

	SetCommTimeouts(m_hComm, &timeouts);

	// dcb 설정.... 포트의 실제적인..제어를 담당하는 DCB 구조체값 셋팅..
	dcb.DCBlength = sizeof(DCB);
	GetCommState( m_hComm, &dcb);	
	//--> 보드레이트를 바꾸고..
	dcb.BaudRate = dwBaud;
	//--> Data 8 Bit
	dcb.ByteSize = byData;
	//--> Noparity
	dcb.Parity = byParity;
	//--> 1 Stop Bit
	dcb.StopBits = byStop;
	//--> 포트를 재..설정값으로.. 설정해보고..
	if( !SetCommState( m_hComm, &dcb) )	
	{
		return FALSE;
	}
	// 포트 감시 쓰레드 생성.
	m_bConnected = TRUE;
	//--> 포트 감시 쓰레드 생성.
	m_hThreadWatchComm = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)ThreadWatchComm, this, 0, &dwThreadID);
	if (! m_hThreadWatchComm)
	{
		ClosePort();
		return FALSE;
	}
	check = FALSE;
	if(funcptr != NULL) SetReceiveFunction(owner,funcptr);
	return TRUE;
}

BOOL CSerialComm::OpenPort(CString strPortName, DWORD dwBaud, BYTE byData, BYTE byStop, BYTE byParity, LPVOID owner,  SERIALSTRINGFUNCPTR funcptr, int nTimeOut)
{
	COMMTIMEOUTS	timeouts;
	DCB				dcb;
	DWORD			dwThreadID;

	// overlapped structure 변수 초기화.
	m_osRead.Offset = 0;
	m_osRead.OffsetHigh = 0;
	//--> Read 이벤트 생성에 실패..
	if ( !(m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)) ) 	
	{
		return FALSE;
	}
	//m_nT1 = nTimeOut1;
	m_nT2 = nTimeOut;
	m_osWrite.Offset = 0;
	m_osWrite.OffsetHigh = 0;
	//--> Write 이벤트 생성에 실패..
	if (! (m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL)))
	{
		return FALSE;
	}
	m_sPortName = strPortName;

	m_hComm = CreateFile(m_sPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
	if (m_hComm == INVALID_HANDLE_VALUE)
	{
		return FALSE;
	}

	//===== 포트 상태 설정. =====

	// EV_RXCHAR event 설정...데이터가 들어오면.. 수신 이벤트가 발생하게끔..
	SetCommMask( m_hComm, EV_RXCHAR);	
	// InQueue, OutQueue 크기 설정.
	SetupComm( m_hComm, MAX_SERIAL_BUFF_SIZE, MAX_SERIAL_BUFF_SIZE);	
	// 포트 비우기.
	PurgeComm( m_hComm,					
		PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
	// timeout 설정.
	timeouts.ReadIntervalTimeout = 0xFFFFFFFF;
	//timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;

	timeouts.WriteTotalTimeoutMultiplier = 0;// 2 * CBR_9600 / dwBaud;
	timeouts.WriteTotalTimeoutConstant = 0;

	SetCommTimeouts( m_hComm, &timeouts);

	// dcb 설정.... 포트의 실제적인..제어를 담당하는 DCB 구조체값 셋팅..
	dcb.DCBlength = sizeof(DCB);
	GetCommState( m_hComm, &dcb);	
	//--> 보드레이트를 바꾸고..
	dcb.BaudRate = dwBaud;
	//--> Data 8 Bit
	dcb.ByteSize = byData;
	//--> Noparity
	dcb.Parity = byParity;
	//--> 1 Stop Bit
	dcb.StopBits = byStop;
	//--> 포트를 재..설정값으로.. 설정해보고..
	if( !SetCommState( m_hComm, &dcb) )	
	{
		return FALSE;
	}
	// 포트 감시 쓰레드 생성.
	m_bConnected = TRUE;
	//--> 포트 감시 쓰레드 생성.
	m_hThreadWatchComm = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)ThreadWatchComm, this, 0, &dwThreadID);
	if (! m_hThreadWatchComm)
	{
		ClosePort();
		return FALSE;
	}
	check = FALSE;
	if(funcptr != NULL) SetReceiveFunction(owner,funcptr);
	return TRUE;
}

void CSerialComm::ClosePort()
{
	m_bConnected = FALSE;
	SetCommMask( m_hComm, 0);
	PurgeComm(m_hComm,	PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
	msgQueue.Flush();
	CloseHandle(m_hComm);
}

DWORD CSerialComm::WriteComm(char Buff)
{
	DWORD	dwWritten = 0;
	if (!m_bConnected)
	{
		return 0;
	}
	//--> 인자로 들어온 버퍼의 내용을 nToWrite 만큼 쓰고.. 쓴 갯수를.,dwWrite 에 넘김.
	if (WriteFile(m_hComm, &Buff, 1, &dwWritten, NULL) == 0)
	{
		if (gcPowerLog->IsShowSerialLog() == true)
		{
			TRACE(_T("[PWR] WriteComm NG %c,0x%X\n"), Buff, Buff);
		}

	}
	else
	{
		if (gcPowerLog->IsShowSerialLog() == true)
		{
			TRACE(_T("[PWR] WriteComm OK %c,0x%X\n"), Buff, Buff);
		}
	}
	return dwWritten;
}

// 포트에 pBuff의 내용을 nToWrite만큼 쓴다.
// 실제로 쓰여진 Byte수를 리턴한다.
DWORD CSerialComm::WriteComm(char *pBuff, DWORD nToWrite)
{
	DWORD	dwWritten, dwError, dwErrorFlags;
	COMSTAT	comstat;
	if( !m_bConnected )		
	{
		return 0;
	}
	if (gcPowerLog->IsShowSerialLog() == true)
	{
		TRACE(_T("[PWR] WriteComm(%c%c%c%c%c%c%c%c)"), pBuff[0], pBuff[1], pBuff[2], pBuff[3], pBuff[4], pBuff[5], pBuff[6], pBuff[7]);
	}
	//--> 인자로 들어온 버퍼의 내용을 nToWrite 만큼 쓰고.. 쓴 갯수를.,dwWrite 에 넘김.
	if( !WriteFile( m_hComm, pBuff, nToWrite, &dwWritten, &m_osWrite))
	{
		//--> 아직 전송할 문자가 남았을 경우..
		if (GetLastError() == ERROR_IO_PENDING)
		{
			// 읽을 문자가 남아 있거나 전송할 문자가 남아 있을 경우 Overapped IO의
			// 특성에 따라 ERROR_IO_PENDING 에러 메시지가 전달된다.
			//timeouts에 정해준 시간만큼 기다려준다.
			while (! GetOverlappedResult( m_hComm, &m_osWrite, &dwWritten, TRUE))
			{
				dwError = GetLastError();
				if (dwError != ERROR_IO_INCOMPLETE)
				{
					ClearCommError( m_hComm, &dwErrorFlags, &comstat);
					break;
				}
			}
		}
		else
		{
			dwWritten = 0;
			ClearCommError( m_hComm, &dwErrorFlags, &comstat);
		}
	}
	return dwWritten;
}

// 포트로부터 pBuff에 nToWrite만큼 읽는다.
// 실제로 읽혀진 Byte수를 리턴한다.
DWORD CSerialComm::ReadComm(char *pBuff, DWORD nToRead)
{
	DWORD	dwRead,dwError, dwErrorFlags;
	COMSTAT comstat;

	//--- system queue에 도착한 byte수만 미리 읽는다.
	ClearCommError( m_hComm, &dwErrorFlags, &comstat);

	//--> 시스템 큐에서 읽을 거리가 있으면..
	dwRead = comstat.cbInQue;
	if(dwRead > 0)
	{
		if( !ReadFile( m_hComm, pBuff, nToRead, &dwRead, &m_osRead) )
		{
			if (GetLastError() == ERROR_IO_PENDING)
			{
				//--------- timeouts에 정해준 시간만큼 기다려준다.
				while (! GetOverlappedResult( m_hComm, &m_osRead, &dwRead, TRUE))
				{
					dwError = GetLastError();
					if (dwError != ERROR_IO_INCOMPLETE)
					{
						ClearCommError( m_hComm, &dwErrorFlags, &comstat);
						break;
					}
				}
			}
			else
			{
				dwRead = 0;
				ClearCommError( m_hComm, &dwErrorFlags, &comstat);
			}
		}
	}
	return dwRead;

}

void CSerialComm::SetReceiveFunction(LPVOID owner, SERIALBYTEFUNCPTR funcptr)
{
//	DWORD dwThreadID;
	if(owner == NULL) m_owner = this;
	else m_owner = owner;
	m_serialStringFuncPtr = NULL;
	m_serialByteFuncPtr = funcptr;
}
void CSerialComm::SetReceiveFunction(LPVOID owner, SERIALSTRINGFUNCPTR funcptr)
{
	if(owner == NULL) m_owner = this;
	else m_owner = owner;
	m_serialStringFuncPtr = funcptr;
	m_serialByteFuncPtr = NULL;
}

// 포트를 감시하고, 읽힌 내용이 있으면
// m_ReadData에 저장한 뒤에 MainWnd에 메시지를 보내어 Buffer의 내용을
// 읽어가라고 신고한다.
DWORD ThreadWatchComm(CSerialComm* pComm)
{
	DWORD      dwEvent;
	OVERLAPPED os;
	BOOL       bOk = TRUE;
	char       buff[1024]; // 읽기 버퍼
	char       buffAllRead[MAX_SERIAL_BUFF_SIZE];
	DWORD      dwRead;  // 읽은 바이트수.
	DWORD      dwFullRead;
	
	// Event, OS 설정.
	memset( &os, 0, sizeof(OVERLAPPED));

	//--> 이벤트 설정..
	if( !(os.hEvent = CreateEvent( NULL, TRUE, FALSE, NULL)) )
	{
		bOk = FALSE;
	}

	//--> 이벤트 마스크..
	if( !SetCommMask( pComm->m_hComm, EV_RXCHAR) )
	{
		bOk = FALSE;
	}

	//--> 이벤트나..마스크 설정에 실패함..
	if( !bOk )
	{
		TRACE(_T("[PWR] Error while creating ThreadWatchComm(%s)"), pComm->m_sPortName);
		return FALSE;
	}
	while (pComm ->m_bConnected)//포트가 연결되면 무한 루프에 들어감
	{
		dwEvent = 0;

		// 포트에 읽을 거리가 올때까지 기다린다.
		WaitCommEvent( pComm->m_hComm, &dwEvent, NULL);
		dwFullRead = 0;
		ZeroMemory(buffAllRead,MAX_SERIAL_BUFF_SIZE);
		//--> 데이터가 수신되었다는 메세지가 발생하면..
		if ((dwEvent & EV_RXCHAR) == EV_RXCHAR)
		{
			// 포트에서 읽을 수 있는 만큼 읽는다.
			//--> buff 에 받아놓고..
			do
			{
				dwRead = pComm->ReadComm( buff, 1024); //들어온 데이터 읽어 오기 
				for(int i=0; i<(int)dwRead; i++)
				{
					buffAllRead[dwFullRead++] = buff[i];
				}
				Sleep(pComm->m_nT2);
			}while(dwRead);
		}
		if(dwFullRead > 0)
		{
		//	pSerialMessage = new SerialMessage();
			if(pComm->m_serialByteFuncPtr != NULL)
			{
				pComm->m_serialByteFuncPtr(pComm->m_owner,buffAllRead,dwFullRead);
			}
			else if(pComm->m_serialStringFuncPtr != NULL)
			{
				CStringA strRead;
				strRead.Format("%s",buffAllRead);
				pComm->m_serialStringFuncPtr(pComm->m_owner,strRead);
			}
		}
		Sleep(0);	// 받은 데이터를 화면에 보여줄 시간을 벌기 위해.
		// 데이터를 연속으로 받으면 cpu점유율이 100%가 되어 화면에 뿌려주는 작업이 잘 안되고. 결과적으로 
		// 큐 버퍼에 데이터가 쌓이게 됨
	}
	CloseHandle(os.hEvent);
	//--> 쓰레드 종료가 되겠죠?
	pComm->m_hThreadWatchComm = NULL;
	return TRUE;
}

void CSerialComm::SetWaitTime(int nT1, int nT2)
{
	m_nT1 = nT1;
	m_nT2 = nT2;
}
