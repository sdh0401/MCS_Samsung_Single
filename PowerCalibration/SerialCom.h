#pragma once
#include "GlobalDefine.h"
#include "CMessageQueue.h"

#define MAX_SERIAL_BUFF_SIZE			2048
typedef void(*SERIALBYTEFUNCPTR)(LPVOID owner, char* pReadBuffer, int nReadCount);
typedef void(*SERIALSTRINGFUNCPTR)(LPVOID owner, CStringA strRead);

class CSerialComm
{
public:
	CSerialComm();
	~CSerialComm();
	//--------- 환경 변수 -----------------------------------------//
	BOOL        check;
	OVERLAPPED	m_osRead, m_osWrite;	// 포트 파일 Overlapped structure
	WORD		m_wPortID;				// WM_SERIAL_COMM_READ와 함께 보내는 인수.
	int         m_nReadCount;
public:
	HANDLE		m_hComm;				// 통신 포트 파일 핸들
	CString		m_sPortName;			// 포트 이름 (COM1 ..)
	BOOL		m_bConnected;			// 포트가 열렸는지 유무를 나타냄.
	CMessageQueue	msgQueue;
	HANDLE		m_hThreadWatchComm;		// Watch함수 Thread 핸들.
	HANDLE      m_hThreadSerialReceive;
	
	//--------- 외부 사용 함수 ------------------------------------//
	BOOL	OpenPort(CString strPortName, DWORD dwBaud, BYTE byData, BYTE byStop, BYTE byParity, LPVOID owner, SERIALBYTEFUNCPTR funcptr, int nTimeOut=1);//포트 열기 
	BOOL	OpenPort(CString strPortName, DWORD dwBaud, BYTE byData, BYTE byStop, BYTE byParity, LPVOID owner, SERIALSTRINGFUNCPTR funcptr, int nTimeOut=1);//포트 열기 
	void    SetWaitTime(int nT1, int nT2);
	void	ClosePort();				//포트 닫기
	DWORD   WriteComm(char Buff);
	DWORD	WriteComm(char* pBuff, DWORD nToWrite);//포트에 데이터 쓰기

	//--------- 내부 사용 함수 ------------------------------------//
	DWORD	ReadComm(char* pBuff, DWORD nToRead);//포트에서 데이터 읽어오기

public:
	int m_nT1;
	int m_nT2;

	LPVOID m_owner;
	SERIALBYTEFUNCPTR m_serialByteFuncPtr;
	SERIALSTRINGFUNCPTR m_serialStringFuncPtr;
private:
	void SetReceiveFunction(LPVOID owner, SERIALBYTEFUNCPTR funcptr);
	void SetReceiveFunction(LPVOID owner, SERIALSTRINGFUNCPTR funcptr);
};

DWORD	ThreadWatchComm(CSerialComm* pComm);

