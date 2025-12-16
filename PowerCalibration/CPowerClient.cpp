#include "pch.h"
#include "CPowerClient.h"
#include "CLogFileSystem.h"
#include "PowerCalibration.h"
#include "CTokenizer.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "CPowerVision.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CDecoding.h"

CPowerClient* gcPowerClient;
CPowerClient::CPowerClient(void)
	: sockfd(INVALID_SOCKET),
	threadSockEvent(CreateEvent(NULL, TRUE, FALSE, NULL)),
	threadDataEvent(CreateEvent(NULL, TRUE, FALSE, NULL)),
	hReplyEvent(CreateEvent(NULL, TRUE, FALSE, NULL)),
	m_Status(CLIENT_DISCONNECTED),
	m_bStopThread(false),
	m_strServerIP(""),
	m_ServerPort(0),
	m_nToggleSend(1),
	m_nToggleReceive(-1),
	nCheckingTarget(-1),
	m_nReplyErrorCode(TARGET_NOERROR),
	m_Log(),
	m_errorCode(TARGET_NOERROR),
	m_dSendTime(0),
	m_ClientID(0),
	hSockThread(),
	hDataThread()
{
	threadSockInfo.m_CmdLock = CreateMutex(nullptr, FALSE, nullptr);
	threadDataInfo.m_CmdLock = CreateMutex(nullptr, FALSE, nullptr);
}

CPowerClient::~CPowerClient(void)
{
	Disconnect();
	if (threadSockEvent) CloseHandle(threadSockEvent);
	if (threadDataEvent) CloseHandle(threadDataEvent);
	CloseHandle(hReplyEvent);
}

void CPowerClient::DoCommunicationLog(CString strLogPath)
{
	m_Log = new CLogFileSystem();
	m_Log->InitializeLogging(strLogPath, _T("comm"), _T("CLIENT"));
}

BOOL CPowerClient::ConnectToServer()
{
	if(m_strServerIP.IsEmpty() == true) 
	{
		TRACE(_T("[PWR] IP Address is null\n"));
		return FALSE;
	}
	return ConnectToServer(m_ClientID, m_strServerIP, m_ServerPort);
}

void CPowerClient::TreatMessage(PowerMessage* msgReceived)
{
	CString strLogMsg;
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	switch(msgReceived->m_nSenderID)
	{
	case ID_MCS:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_MCS RCV CMD1(%d)\n"), msgReceived->GetSubCommand(0));
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[PWR] ID_MCS RCV CMD1(%d)"), msgReceived->GetSubCommand(0));
			m_Log->AddLogMsg(strLogMsg);
		}
		break;
	case ID_HMI:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_HMI RCV CMD(%d,%d,%d)\n"), msgReceived->GetSubCommand(0), msgReceived->GetSubCommand(1), msgReceived->GetSubCommand(2));
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[RCV] ID_HMI RCV CMD(%d,%d,%d)"), msgReceived->GetSubCommand(0), msgReceived->GetSubCommand(1), msgReceived->GetSubCommand(2));
			m_Log->AddLogMsg(strLogMsg);
		}
		strMsg = msgReceived->GetMsg();
		msgSend->SetThreadMsg(strMsg);
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetSubCommand(indx);
		}
		msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (gcDecoding)
		{
			gcDecoding->GetId(&id);
			msgSend->SetID(id);
			if (gcDecoding->PingThread(TIME1MS))
			{
				gcDecoding->Event((LPVOID)msgSend);
			}
		}
		break;
	case ID_SET:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_SET RCV CMD1(%d)\n"), msgReceived->GetSubCommand(0));
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[PWR] ID_SET RCV CMD1(%d)"), msgReceived->GetSubCommand(0));
			m_Log->AddLogMsg(strLogMsg);
		}
		break;
	case ID_VIS:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_VIS RCV CMD1(%d)\n"), msgReceived->GetSubCommand(0));
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[PWR] ID_VIS RCV CMD1(%d)"), msgReceived->GetSubCommand(0));
			m_Log->AddLogMsg(strLogMsg);
		}
		gcPowerVision->GetId(&id);
		msgSend->SetID(id);
		strMsg = msgReceived->GetMsg();
		msgSend->SetThreadMsg(strMsg);
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetSubCommand(indx);
		}
		nSubMsg[1] = FRONT_VISION;
		msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (gcPowerVision->PingThread(TIME1MS))
		{
			gcPowerVision->Event((LPVOID)msgSend);
		}
		break;
	case ID_VIS_REAR:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_VIS_REAR RCV CMD1(%d)\n"), msgReceived->GetSubCommand(0), (LPCTSTR)strMsg);
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[PWR] ID_VIS_REAR RCV CMD1(%d)"), msgReceived->GetSubCommand(0));
			m_Log->AddLogMsg(strLogMsg);
		}
		gcPowerVision->GetId(&id);
		msgSend->SetID(id);
		strMsg = msgReceived->GetMsg();
		msgSend->SetThreadMsg(strMsg);
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetSubCommand(indx);
		}
		nSubMsg[1] = REAR_VISION;
		msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		if (gcPowerVision->PingThread(TIME1MS))
		{
			gcPowerVision->Event((LPVOID)msgSend);
		}
		break;
	case ID_MES:
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] ID_MES RCV CMD(%d,%d,%d)\n"), msgReceived->GetSubCommand(0), msgReceived->GetSubCommand(1), msgReceived->GetSubCommand(2));
		}
		if (m_Log != NULL)
		{
			strLogMsg.Format(_T("[PWR] ID_MES RCV CMD(%d,%d,%d)"), msgReceived->GetSubCommand(0), msgReceived->GetSubCommand(1), msgReceived->GetSubCommand(2));
			m_Log->AddLogMsg(strLogMsg);
		}
		break;
	default:
		break;
	}
}

int CPowerClient::GetToggleSend()
{
	if (m_nToggleSend == 3) m_nToggleSend = 1;
	else m_nToggleSend++;
	return m_nToggleSend;
}

BOOL CPowerClient::ConnectToServer(int nID, CString strServerIPAddress, UINT nServerPort)
{
	BOOL bResult = FALSE;
	WSADATA wsa;
	char strIP[INET_ADDRSTRLEN];
	int retval = 0;
	int optval;// , threadID;
	CString strLog;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		TRACE(_T("[PWR] WSAStartup fail\n"));
	}
	sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//소켓 생성
	if (sockfd == INVALID_SOCKET)
	{
		TRACE(_T("[PWR] socket() INVALID_SOCKET\n"));
	}
	SOCKADDR_IN ServerAddr = { 0 };//소켓 주소
	ZeroMemory(&ServerAddr, sizeof(ServerAddr));
	ServerAddr.sin_family = AF_INET;
	//ServerAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
	ServerAddr.sin_port = htons(PORT_SERVER);
	inet_pton(AF_INET, STRING_SERVER_IP, &ServerAddr.sin_addr.s_addr);
	inet_ntop(AF_INET, &(ServerAddr.sin_addr), strIP, INET_ADDRSTRLEN);

	struct linger solinger = { 1, 0 };
	retval = setsockopt(sockfd, SOL_SOCKET, SO_LINGER, (char*)&solinger, sizeof(struct linger));
	if (retval == SOCKET_ERROR)
	{
		TRACE(_T("[PWR] setsockopt(SO_LINGER) SOCKET_ERROR\n"));
	}
	optval = -1;
	retval = setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, (char*)&optval, sizeof(optval));
	if (retval == SOCKET_ERROR)
	{
		TRACE(_T("[PWR] setsockopt(SO_KEEPALIVE) SOCKET_ERROR\n"));
	}
	optval = -1;
	retval = setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&optval, sizeof(optval));
	if (retval == SOCKET_ERROR)
	{
		TRACE(_T("[PWR] setsockopt(SO_REUSEADDR) SOCKET_ERROR\n"));
	}
	retval = connect(sockfd, (struct sockaddr*) & ServerAddr, sizeof(ServerAddr));//연결 요청
	if (retval == SOCKET_ERROR)
	{
		/*ErrQuit(_T("connect() SOCKET_ERROR\n"));*/
	}
	m_ClientID = nID;
	m_strServerIP.Format(_T("%s"), (LPCTSTR)strServerIPAddress);
	m_ServerPort = nServerPort;
	strLog.Format(_T("[PWR] ServerIP:%s Port:%d"), (LPCTSTR)m_strServerIP, (long)nServerPort);
	gcPowerLog->Logging(strLog);
	m_Status = CLIENT_CONNECTED;
	threadSockInfo.m_Handle = CreateThread( NULL, 0,(LPTHREAD_START_ROUTINE)ThreadSocketProc, this, 0, (LPDWORD )&threadSockInfo.m_ID);
	threadDataInfo.m_Handle = CreateThread( NULL, 0, (LPTHREAD_START_ROUTINE)ThreadDataProc, this, 0, (LPDWORD )&threadDataInfo.m_ID);
	TRACE(_T("[PWR] CPowerClient Sock:0x%04X Data:0x%04X\n"), threadSockInfo.m_ID, threadDataInfo.m_ID);
	m_dSendTime = _time_get();
	int nWait = 0;
	bResult = TRUE;
	while(m_Status != CLIENT_CHECKIN)
	{
		Sleep(1);
		if(nWait++ > 1000)
		{
			bResult =FALSE;
			break;
		}
	}
	return bResult;
}

DWORD CPowerClient::ThreadSocketProc(LPVOID pParam)
{
	char kbuff[MAX_BUFFER_SIZE];
	char kbuffRemain[MAX_BUFFER_SIZE * 2];
	CPowerClient* pThis = reinterpret_cast<CPowerClient*>(pParam);
	int fromlen = sizeof(sockaddr);
	fd_set  fdRead  = { 0 };
	timeval tv = { 0, 100 * 1000 }; // 100ms
	ZeroMemory(kbuffRemain, sizeof(kbuffRemain));
	int nRemainCount = 0;
	bool bLoop = true;
	while (bLoop && !pThis->m_bStopThread)
	{
		FD_ZERO(&fdRead);
		FD_SET(pThis->sockfd, &fdRead);

		int sel = select(0, &fdRead, nullptr, nullptr, &tv);
		if (sel > 0 && FD_ISSET(pThis->sockfd, &fdRead))
		{
			ZeroMemory(kbuff, sizeof(kbuff));
			int nRead = recv(pThis->sockfd, kbuff, MAX_BUFFER_SIZE, 0);
			if (nRead > 0)
			{
				nRemainCount = pThis->MakeFullBuff(kbuffRemain, nRemainCount, kbuff, nRead);
				nRemainCount = pThis->SplitCommand(kbuffRemain, nRemainCount);
			}
			else
			{
				pThis->m_Status = CLIENT_DISCONNECTED;
				break;
			}
		}
		if (pThis->threadSockEvent && WaitForSingleObject(pThis->threadSockEvent, 0) == WAIT_OBJECT_0) break;
	}
	TRACE(_T("[PWR] ThreadSocketProc ID:0x%04X Axis Quit\n"), pThis->threadSockInfo.m_ID);
	return 0;
}

DWORD CPowerClient::ThreadDataProc(LPVOID pParam)
{
	void* pMsg = NULL;
	PowerMessage* msgReceived;
	CPowerClient* pThis = reinterpret_cast<CPowerClient*>(pParam);
	bool bLoop = true;
	while (bLoop && !pThis->m_bStopThread)
	{
		if (pThis->msgQueue.Pop(pMsg, 1000) == TRUE)
		{
			msgReceived = static_cast<PowerMessage*>(pMsg);
			pThis->TreatMessage(msgReceived);
			if (msgReceived->m_atomic == CHECK_TREAT) pThis->SendReplayToServer(msgReceived->m_nMsgID);
			delete msgReceived;
			msgReceived = NULL;
			Sleep(1);
		}
		if (pThis->m_Status == CLIENT_EXCEPTION || pThis->m_Status == CLIENT_DISCONNECTED) break;
		if (pThis->threadDataEvent && WaitForSingleObject(pThis->threadDataEvent, 0) == WAIT_OBJECT_0) break;
	}
	TRACE(_T("[PWR] ThreadDataProc ID:0x%04X Axis Quit\n"), pThis->threadDataInfo.m_ID);
	return 0;
}

BOOL CPowerClient::AmICheckedIn()
{
	if(m_Status == CLIENT_CHECKIN) return TRUE;
	return FALSE;
}

int CPowerClient::MakeFullBuff(char* remain, int nRemainCount, char* receive, int nReceiveCount)
{
	for(int i=0; i<nReceiveCount; i++)
	{
		remain[nRemainCount++] = receive[i];
	}
	return nRemainCount;
}

int CPowerClient::SplitCommand(char* remain, int nFullCount)
{
//	int i;
	char byteRead;
	BOOL bSTX = FALSE;
	int nDoIndex, msgCount;

	char byteMessage[MAX_BUFFER_SIZE];
	ZeroMemory(byteMessage,MAX_BUFFER_SIZE);
	msgCount = 0;
	nDoIndex = 0;
	while(TRUE)
	{
		byteRead = remain[nDoIndex++];
		if(byteRead == CSTX) bSTX = TRUE;
		if(bSTX)
		{
			byteMessage[msgCount++] = byteRead;
			if(byteRead == CETX)
			{
				MakeCommand(byteMessage,msgCount);
				msgCount = 0;
				ZeroMemory(byteMessage,MAX_BUFFER_SIZE);
				bSTX = FALSE;
			}
		}
		if(nDoIndex == nFullCount) break;
	}
	ZeroMemory(remain,MAX_BUFFER_SIZE * 2);
	memcpy_s(remain,sizeof(remain),byteMessage,msgCount);
	return msgCount;

}

void CPowerClient::MakeCommand(char* cmdBytes, int nCount)
{
	CString strMsg;
	ASSERT(cmdBytes[0] == CSTX);
	ASSERT(cmdBytes[nCount-1] == CETX);
	PowerMessage* msgReceived = new PowerMessage((char*)cmdBytes,nCount);
	if(msgReceived->m_mainCommand == SC_REPLY)
	{
		m_nReplyErrorCode = msgReceived->GetSubCommand(0);
		m_MsgIDControl.AwakeMsgID(msgReceived->m_nMsgID);
		delete msgReceived;
		msgReceived = NULL;
	}
	else if(msgReceived->m_mainCommand == SC_CHECKIN)
	{
		if(msgReceived->GetSubCommand(0) == 1)
		{
			m_Status = CLIENT_CHECKIN;
		}
		else
		{
			if(msgReceived->GetMsg() != _T(STRING_SERVER_PROFILE))
			{
				strMsg.Format(_T("[PWR] Server profile(%s) is not match with this client(%s)"), (LPCTSTR)msgReceived->GetMsg(), (LPCTSTR)_T(STRING_SERVER_PROFILE));
				AfxMessageBox(strMsg);
				SocketClose();
			}
			else SendServerCommand(SC_CHECKIN, 0,0,_T(""));
		}
		delete msgReceived;
		msgReceived = NULL;
	}
	else if(msgReceived->m_mainCommand == SC_TARGETCHECK)
	{
		nCheckingTarget = msgReceived->GetSubCommand(0);
		delete msgReceived;
		msgReceived = NULL;
	}
	else if(msgReceived->m_mainCommand == SC_BAD)
	{
		delete msgReceived;
		msgReceived = NULL;
	}
	else
	{
		msgQueue.Push(msgReceived);
	}
}

bool CPowerClient::SendCommandTo(UINT nTargetID, UINT command1, UINT command2, UINT command3, CString sendMessage, BOOL bWait)
{
	CString strLogMessage;
	int sendStatus;
	CArray<int, int> cmdArray;
	cmdArray.Add(command1);
	cmdArray.Add(command2);
	cmdArray.Add(command3);

	if (bWait == FALSE)		sendStatus = SendCommand(SC_TARGET_MSG, nTargetID, &cmdArray, sendMessage, CHECK_EASY);
	else					sendStatus = SendCommand(SC_TARGET_MSG, nTargetID, &cmdArray, sendMessage, CHECK_SEND);
	CString strStatus = GetStatusString(sendStatus);
	if (m_Log != NULL)
	{
		strLogMessage.Format(_T("[SND][%s] CMD1(%d) CMD2(%d) CMD3(%d) (%s)"), (LPCTSTR)strStatus, (long)command1, (long)command2, (long)command3, (LPCTSTR)sendMessage);
		m_Log->AddLogMsg(strLogMessage);
	}
	cmdArray.RemoveAll();
	if (sendStatus == MSG_SEND_GOOD) return true;
	strLogMessage.Format(_T("[SND][%s] CMD1(%d) CMD2(%d) CMD3(%d) (%s)"), (LPCTSTR)strStatus, (long)command1, (long)command2, (long)command3, (LPCTSTR)sendMessage);
	TRACE(strLogMessage);
	return false;
}

int CPowerClient::SendCommand(int mainCommand, UINT nTargetID, CArray<int, int>* pCmdArray, CString sendMessage, int atomic)
{
	int nMsgID = 0;
	ULONGLONG nWaitTime;
	int nSend;
	int nSockResult;
	int sendResult = MSG_SEND_CANT;
	char byteToSend[MAX_BUFFER_SIZE] ={0};
	
	if (atomic != CHECK_EASY)
	{
		nMsgID = m_MsgIDControl.MakeOneMsgID();
		ASSERT(nMsgID != -1);
	}
	nSend = PowerMessage::ToByte(byteToSend, nMsgID, mainCommand, m_ClientID, nTargetID, pCmdArray, sendMessage, atomic);
	if (nSend > 0)
	{
		sendResult = MSG_SEND_CANT;
		if (m_Status == CLIENT_CONNECTED || m_Status == CLIENT_CHECKIN)
		{
			nWaitTime = _time_get();
			m_SendLock.Lock(INFINITE);
			if (atomic != CHECK_EASY) m_MsgIDControl.SleepMsgID(nMsgID);
			nSockResult = send(sockfd,(char*)byteToSend,nSend,0);
			//TRACE(_T("[PWR] ID_SET SND CMD Len(%d) Target(%d) Cmd(%d) SockResult:%d\n"), nSend, nTargetID, mainCommand, nSockResult);
			m_SendLock.Unlock();
			if (nSockResult == SOCKET_ERROR)
			{
				SocketClose();
				sendResult = MSG_SEND_FAIL;
			}
			else
			{
				sendResult = MSG_SEND_GOOD;
				if(atomic != CHECK_EASY)
				{
					sendResult = WaitResponse(nMsgID,500);
					TRACE(_T("[PWR] ID:%d Res:%d%d %d[ms]\n"), nMsgID, sendResult, _time_elapsed(nWaitTime));
				}
			}
		}
	}
	return sendResult;
}

int CPowerClient::SendServerCommand(int mainCommand,UINT command1, UINT command2,CString sendMessage)
{
	int nToggle = 0;
	int nWaitTime = 300;
	int nSend;
	int nSockResult;
	int nRetryCount = 0;
	int sendResult = MSG_SEND_CANT;
	CArray<int,int> cmdArray;
	cmdArray.Add(command1);
	cmdArray.Add(command2);
	char byteToSend[MAX_BUFFER_SIZE] ={0};
	nSend = PowerMessage::ToByte(byteToSend, 0 , mainCommand, m_ClientID, 0, &cmdArray, sendMessage, CHECK_EASY);
	if (nSend > 0)
	{
		if (m_Status == CLIENT_CONNECTED || m_Status == CLIENT_CHECKIN)
		{
			m_SendLock.Lock(INFINITE);
			if(nToggle > 0) ResetEvent(hReplyEvent);
			nSockResult = send(sockfd,(char*)byteToSend,nSend,0);
			m_SendLock.Unlock();
			if(nSockResult == SOCKET_ERROR)
			{
				SocketClose();
				sendResult = MSG_SEND_FAIL;
			}
			else sendResult = MSG_SEND_GOOD;
		}
	}
	cmdArray.RemoveAll();
	return sendResult;
}


int CPowerClient::WaitResponse(int nMsgID,int nMiliseconds)
{
	int status = MSG_SEND_TIMEOUT;
	
	if(m_MsgIDControl.WaitMsgIDNotify(nMsgID,nMiliseconds))
	{
		if(m_nReplyErrorCode == TARGET_NOERROR) status = MSG_SEND_GOOD;
		else if(m_nReplyErrorCode == TARGET_NOTEXIST) status = MSG_SEND_TARGET_NOTEXIST;
		else if(m_nReplyErrorCode == TARGET_NOTREPONSE) status = MSG_SEND_TIMEOUT;
		else if(m_nReplyErrorCode == TARGET_SOCKETERROR) status = MSG_SEND_FAIL;
	}
	return status;
}

void CPowerClient::Disconnect()
{
	DWORD dwExitCode = 0;
	closesocket(sockfd);
	m_bStopThread = true;
	if (threadSockEvent) SetEvent(threadSockEvent);
	if (threadDataEvent) SetEvent(threadDataEvent);
	if (threadSockInfo.m_Handle)
	{
		WaitForSingleObject(threadSockInfo.m_Handle, INFINITE);
		CloseHandle(threadSockInfo.m_Handle);
		threadSockInfo.m_Handle = NULL;
	}
	Sleep(100);
	if (threadDataInfo.m_Handle)
	{
		DWORD waitResult = WaitForSingleObject(threadDataInfo.m_Handle, 1000);
		if (waitResult != WAIT_OBJECT_0)
		{
			// 1초 이상 대기 후 종료되지 않으면 로그
			TRACE(_T("[PWR] Data thread did not exit in time\n"));
		}
		CloseHandle(threadDataInfo.m_Handle);
		threadDataInfo.m_Handle = NULL;
	}
	m_Status = CLIENT_DISCONNECTED;
	//if (WaitForSingleObject(threadDataInfo.m_Handle, 1000) != WAIT_OBJECT_0)
	//{
	//	TerminateThread(threadDataInfo.m_Handle, 0);
	//	while (::GetExitCodeThread(threadDataInfo.m_Handle, &dwExitCode) == STILL_ACTIVE) Sleep(1);
	//}
}

void CPowerClient::SocketClose()
{
	closesocket(sockfd);
	m_Status = CLIENT_DISCONNECTED;
}

void CPowerClient::OnClose(int nErrorCode)
{
}

void CPowerClient::SendReplayToServer(int nMsgID)
{
	SendServerCommand(SC_REPLY, nMsgID, 0, _T(""));
}

BOOL CPowerClient::CheckTargetExist(int nTargetID)
{
	BOOL bExist = FALSE;
	//m_SendLock.Lock(INFINITE);
	ULONGLONG nTime;// = _time_get();
	while(TRUE)
	{
		if(m_Status != CLIENT_CONNECTED && m_Status != CLIENT_CHECKIN) break;
		nCheckingTarget = -1;
		nTime = _time_get();
		if (SendServerCommand(SC_TARGETCHECK, nTargetID, 0, _T("")) == MSG_SEND_GOOD)
		{
			while(true)
			{
				Sleep(0);
				if(nCheckingTarget != -1) break;
				if(_time_elapsed(nTime) > 100)
				{
					nCheckingTarget = -1;
					break;
				}
			}
		}
		else 
			break;
		if(nCheckingTarget >= 0) 
			break;
	}
	if(nCheckingTarget == nTargetID) bExist = TRUE;
	return bExist;
}

UINT CPowerClient::GetMyID()
{
	return m_ClientID;
}

CString CPowerClient::GetStatusString(int status)
{
	CString strReturn = _T("");
	switch(status)
	{
	case MSG_SEND_CANT:
		strReturn = _T("CANT_SEND");
		break;
	case MSG_SEND_FAIL:
		strReturn = _T("SOCKETFAIL");
		break;
	case MSG_SEND_TIMEOUT:
		strReturn = _T("TIMEOUT");
		break;
	case MSG_SEND_TARGET_NOTEXIST:
		strReturn = _T("NO TARGET");
		break;
	case MSG_SEND_GOOD:
		strReturn = _T("GOOD");
		break;
	}
	return strReturn;
}

PowerMessage::PowerMessage()
{
	m_nSenderID = NULL;
	m_nTargetID = NULL;
	m_nMsgID = -1;
	m_nLength = 0;
	m_mainCommand = SC_BAD;
	m_pByteRead = NULL;
	m_atomic = CHECK_EASY;
}

PowerMessage::~PowerMessage()
{
	m_arrayCmdList.RemoveAll();
	if (m_pByteRead != NULL) delete[] m_pByteRead;	
}

PowerMessage::PowerMessage(char* byteRead,int nLength)
{
	size_t bufSize = static_cast<size_t>(nLength) + 1;
	m_pByteRead = new char[bufSize];
	ZeroMemory(m_pByteRead, bufSize);
	m_nMsgID = -1;
	m_nLength = nLength;
	memcpy_s(m_pByteRead, bufSize, byteRead, nLength);
	if (ExchangeToMsg() == FALSE) m_mainCommand = SC_BAD;
}

BOOL PowerMessage::ExchangeToMsg()
{
	int nIndex = 0;
	int msgLen;
	UINT nValue;
	char byteTemp, byteRead;
	char* cSend = NULL;
	UINT nLength = m_nLength;
	BYTE sdata[MAX_BUFFER_SIZE] = {0};
	m_mainCommand = SC_BAD;
	if (m_pByteRead[nIndex++] != CSTX) return FALSE;

	byteRead = m_pByteRead[nIndex++];
	byteTemp = byteRead & 0xC0;
	if (byteTemp == 0x80) m_atomic = CHECK_SEND;
	else if(byteTemp == 0xC0) m_atomic = CHECK_TREAT;
	byteTemp = byteRead & 0x3F;
	nValue = OneByteToUnsignedInt(byteTemp);

	m_mainCommand = nValue;
	nValue = OneByteToUnsignedInt(m_pByteRead[nIndex++]);
	m_nMsgID = nValue;
	nValue = OneByteToUnsignedInt(m_pByteRead[nIndex++]);
	m_nSenderID = nValue;
	nValue = OneByteToUnsignedInt(m_pByteRead[nIndex++]);
	m_nTargetID = nValue;
	nValue = OneByteToUnsignedInt(m_pByteRead[nIndex++]);
	for (UINT i=0; i<nValue; i++) m_arrayCmdList.Add(OneByteToUnsignedInt(m_pByteRead[nIndex++]));
	msgLen = m_nLength - nIndex - 1;
	if (msgLen > 0)
	{
		size_t bufSize = static_cast<size_t>(msgLen) + 1;
		cSend = new char[bufSize];
		ZeroMemory(cSend, bufSize);
		for(int i = 0; i < msgLen; i++)
		{
			cSend[i] = m_pByteRead[nIndex++];
		}
		string str(cSend);
		m_myMsg = str.c_str();
		if (cSend != NULL) delete[] cSend;
	}
	else m_myMsg = "";
	if (m_pByteRead[nIndex++] != CETX) return FALSE;
	return TRUE;
}

int PowerMessage::ToByte(char* sendByte, int msgID,int mainCommand,int nMyID, int nTargetID, CArray<int,int>* pCmdArray ,CString sendMessage, int atomic)
{
	int nIndex= 0;
	int msgLen, msgChangeLen = 0;
	CString sData;
	char byteResult;
	char* cSend = NULL;
	ASSERT(nMyID >= 0 && nMyID < 230);
	ASSERT(nTargetID >= 0 && nTargetID < 230);
	ZeroMemory(sendByte,MAX_BUFFER_SIZE);
	sendByte[nIndex++] = CSTX;
	byteResult = UnsignedIntToOneByte((int)mainCommand);
	if (atomic == CHECK_SEND) byteResult |= 0x80;
	else if (atomic == CHECK_TREAT) byteResult |= 0xC0;
	sendByte[nIndex++] = byteResult;
	//sendByte[nIndex++] = UnsignedIntToOneByte((int)mainCommand);
	sendByte[nIndex++] = UnsignedIntToOneByte((int)msgID);
	sendByte[nIndex++] = UnsignedIntToOneByte((int)nMyID);
	sendByte[nIndex++] = UnsignedIntToOneByte((int)nTargetID);
	sendByte[nIndex++] = UnsignedIntToOneByte((int)pCmdArray->GetCount());
	for (int i=0; i<pCmdArray->GetCount(); i++) sendByte[nIndex++] = UnsignedIntToOneByte(pCmdArray->GetAt(i));
	if (sendMessage == "") msgLen = 0;
	else
	{
		msgLen = sendMessage.GetLength();
		//TRACE(_T("[PWR] PowerMessage ToByte GetLength:%d\n"), msgLen);
		ASSERT(msgLen < MAX_BUFFER_SIZE);
		size_t bufSize = static_cast<size_t>(msgLen) + 1;
		cSend = new char[bufSize];
		ZeroMemory(cSend, bufSize);
		//memcpy(cSend, (char*)(LPCTSTR)sendMessage, msgLen);
		strcpy_s(cSend, bufSize, CT2A(sendMessage));
	}
	if (msgLen > 0)
	{
		for(int i=0; i<msgLen; i++)
		{
			sendByte[nIndex++] = (BYTE)cSend[i];
		}
	}
	if (cSend != NULL) delete[] cSend;
	sendByte[nIndex++] = CETX;
	return nIndex;
}

CString PowerMessage::GetMsg()
{
	return m_myMsg;
}

void PowerMessage::UnsingedIntToPByte(UINT nValue, char* byteValue, int nCount)
{
	int nReverse;
	long refValue;
	for (int i=0; i<nCount; i++) 
		byteValue[i] = (char)0x80;
	if (nValue == 0) return;
	nReverse = nCount;
	refValue = 0x000000007f;
	for (int i = 0; i < nCount; i++)
	{
		nReverse--; byteValue[nReverse] |= (char)((nValue&refValue) >> 7*i);
		refValue = refValue << 7; 
	}
}

UINT PowerMessage::PByteToUnsingedInt(char* byteValue, UINT nCount)
{
	int nReverse;
	long nlValue = 0;
	ASSERT(nCount > 0);
	nReverse = nCount;
	for (int i = 0; i < (int)nCount;i++)
	{
		nReverse--;
		nlValue |= (byteValue[nReverse] & 0x000000007f)<<(7*i);
	}
	return (UINT)nlValue;
}

UINT PowerMessage::OneByteToUnsignedInt(BYTE byteValue)
{
	UINT nValue;
	if (byteValue < 0x04)
	{
		TRACE(_T("[PWR] OneByteToUnsignedInt over number limitation only 0x05 over support\n"));
		return 0;
	}
	nValue = byteValue - 0x04;
	return nValue;
}

BYTE PowerMessage::UnsignedIntToOneByte(UINT nValue)
{
	BYTE byteValue;
	if (nValue > 240)
	{
		TRACE(_T("[PWR] UnsignedIntToOneByte over number limitation only 0~200 support\n"));
		return 0x00;
	}
	byteValue = nValue + 0x04;
	return byteValue;
}

int PowerMessage::GetSubCommand(int nIndex)
{
	int nResult = -1;
	ASSERT(nIndex < m_arrayCmdList.GetCount() && nIndex >= 0);
	return m_arrayCmdList[nIndex];
}

CToggle::CToggle()
{
	m_nMsgIndex = 0;
	hReplyEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (hReplyEvent > 0)
	{
		ResetEvent(hReplyEvent);
	}
}

CToggle::CToggle(int nMsgIndex)
{
	m_nMsgIndex = nMsgIndex;
	hReplyEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (hReplyEvent > 0)
	{
		ResetEvent(hReplyEvent);
	}
}

CToggle::~CToggle()
{
	CloseHandle(hReplyEvent);
}

CMsgIDControl::CMsgIDControl()
{
}

CMsgIDControl::~CMsgIDControl()
{
	m_cs.Lock(INFINITE);

	CToggle* pToggle = NULL;
	for(int i=0; i<m_MsgIDController.GetCount(); i++)
	{
		pToggle = (CToggle*)m_MsgIDController.GetAt(i);
		delete pToggle;
		pToggle = NULL;
	}
	m_cs.Unlock();
	m_MsgIDController.RemoveAll();
}

int CMsgIDControl::MakeOneMsgID()
{
	m_cs.Lock(INFINITE);
	int nID = m_msgIDGenerator.GetNewID();
	if(nID != -1)
	{
		CToggle* pToggle = new CToggle(nID);
		m_MsgIDController.Add(pToggle);
	}
	m_cs.Unlock();
	return nID;
}

BOOL CMsgIDControl::WaitMsgIDNotify(int nIndex, int nWaitTime)
{
	BOOL bResult = FALSE;
	CToggle* pToggle = FindToggle(nIndex);
	if(pToggle != NULL)
	{
		if(WaitForSingleObject(pToggle->hReplyEvent,nWaitTime) == WAIT_OBJECT_0)
		{
			bResult = TRUE;
		}
		DeleteToggle(nIndex);
	}
	return bResult;
}

void CMsgIDControl::SleepMsgID(int nIndex)
{
	CToggle* pToggle = FindToggle(nIndex);
	if(pToggle != NULL)
	{
		ResetEvent(pToggle->hReplyEvent);
	}
}

void CMsgIDControl::AwakeMsgID(int nIndex)
{
	CToggle* pToggle = FindToggle(nIndex);
	if(pToggle != NULL)
	{
		SetEvent(pToggle->hReplyEvent);
	}
	
}

CToggle* CMsgIDControl::FindToggle(int nIndex)
{
	m_cs.Lock(INFINITE);

	CToggle* pToggle = NULL;
	for(int i=0; i<m_MsgIDController.GetCount(); i++)
	{
		pToggle = (CToggle*)m_MsgIDController.GetAt(i);
		if(pToggle->m_nMsgIndex == nIndex) break;
		pToggle = NULL;
	}
	m_cs.Unlock();
	return pToggle;
}

void CMsgIDControl::DeleteToggle(int nIndex)
{
	m_cs.Lock(INFINITE);

	CToggle* pToggle = NULL;
	for(int i=0; i<m_MsgIDController.GetCount(); i++)
	{
		pToggle = (CToggle*)m_MsgIDController.GetAt(i);
		if(pToggle->m_nMsgIndex == nIndex)
		{
			m_msgIDGenerator.ReleaseID(nIndex);
			m_MsgIDController.RemoveAt(i);
			delete pToggle;
			pToggle = NULL;
			break;
		}
	}
	m_cs.Unlock();
}
