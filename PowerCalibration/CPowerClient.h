#pragma once
#include "GlobalDefine.h"
#include "CLogFileSystem.h"
#include "CMessageQueue.h"
#include "GeneralUtilClass.h"
#include "CApplicationTime.h"

//#define MAX_BUFFER_SIZE		2048
//#define TARGET_SERVER		0

#if _MSC_VER > 1000
//#pragma warning (disable: 4786)
//#pragma warning (disable: 4748)
//#pragma warning (disable: 4103)
#endif /* _MSC_VER > 1000 */

#define CLIENT_NONE			0
#define	CLIENT_NORMAL		1
#define	CLIENT_CONNECTED	2
#define	CLIENT_DISCONNECTED	3
#define	CLIENT_CHECKIN		4
#define	CLIENT_CHECKOUT		5
#define	CLIENT_EXCEPTION	6
#define	CLIENT_DUPLICATED	7

#define	SC_BAD				0
#define	SC_REPLY			1
#define	SC_CHECKIN			2
#define	SC_TARGETCHECK		3
#define	SC_ALIVE			4
#define	SC_TARGET_MSG		5
#define	SC_BROAD_MSG		6

#define	CHECK_EASY			0
#define CHECK_SEND			1
#define CHECK_TREAT			2

#define MAX_ONE_LOGLINE_SIZE 4096

#define MSG_SEND_CANT		0
#define MSG_SEND_TIMEOUT	1
#define MSG_SEND_FAIL		2
#define MSG_SEND_TARGET_NOTEXIST	3
#define MSG_SEND_GOOD		4

#define	TARGET_NOERROR		0
#define	TARGET_NOTEXIST		1
#define	TARGET_NOTREPONSE	2
#define	TARGET_SOCKETERROR	3

#define CSTX				0x01
#define CETX				0x02

class  CToggle
{
protected:
	CToggle();
public:
	CToggle(int nMsgID);
	~CToggle();
public:
	int m_nMsgIndex;
	HANDLE hReplyEvent;
};

class CMsgIDControl
{
public:
	CMsgIDControl();
	~CMsgIDControl();
public:
	int MakeOneMsgID();
	BOOL WaitMsgIDNotify(int nIndex, int nWaittime);
	void SleepMsgID(int nIndex);
	void AwakeMsgID(int nIndex);

private:
	CCriticalSection m_cs;
	CToggle* FindToggle(int nIndex);
	void DeleteToggle(int nIndex);
	CIDGenerator m_msgIDGenerator;
	CArray<CToggle*,CToggle*> m_MsgIDController;
};

class PowerMessage;
class CPowerClient
{
public:
	CPowerClient(void);
	~CPowerClient(void);
	int m_nReplyErrorCode;
	SOCKET sockfd;
	BOOL ConnectToServer();
	BOOL ConnectToServer(int nID, CString strServerIPAddress, UINT nServerPort = 10002);
	
	bool SendCommandTo(UINT nTargetID, UINT command1, UINT command2, UINT command3, CString sendMessage, BOOL bWait);
	int SendCommand(int mainCommand,UINT nTargetID, CArray<int,int>* pCmdArray ,CString sendMessage, int atomic);
	void TreatMessage(PowerMessage* msgReceived);
	int m_Status;
	int m_errorCode;
	//BOOL m_bIsAlive;
	UINT GetMyID();
	void Disconnect();
	void SocketClose();
	BOOL CheckTargetExist(int nTargetID);
	virtual void OnClose(int nErrorCode);
	CString GetStatusString(int status);
	void DoCommunicationLog(CString strLogPath);
	THREAD_STRUCT threadSockInfo;
	THREAD_STRUCT threadDataInfo;

	volatile bool m_bStopThread = false;
	HANDLE threadSockEvent = nullptr; // 소켓 스레드 종료 이벤트
	HANDLE threadDataEvent = nullptr; // 데이터 스레드 종료 이벤트
private:
	CMutex m_SendLock;
	CMessageQueue msgQueue;	
	HANDLE hSockThread;	
	HANDLE hDataThread;	
	ULONGLONG m_dSendTime;
	CMsgIDControl m_MsgIDControl;
	CLogFileSystem* m_Log;
	static DWORD ThreadSocketProc(LPVOID pParam);
	static DWORD ThreadDataProc(LPVOID pParam);
	
	BOOL AmICheckedIn();
	int SplitCommand(char* remain, int nFullCount);
	void MakeCommand(char* cmdBytes, int nCount);
	void SendReplayToServer(int nMsgID);
	int MakeFullBuff(char* remain, int nRemainCount, char* receive, int nReceiveCount);
	int WaitResponse(int nMsgID, int nMiliseconds);
	int SendServerCommand(int mainCommand,UINT command1, UINT command2,CString sendMessage);
	int GetToggleSend();
	CString m_strServerIP;
	UINT m_ServerPort;
	UINT m_ClientID;
	int nCheckingTarget;
	HANDLE hReplyEvent;
	int m_nToggleSend;
	int m_nToggleReceive;
};

class PowerMessage
{
public:
	unsigned m_nSenderID;
	unsigned m_nTargetID;
	int m_mainCommand;  //MainCommand type (login, logout, send message, etcetera)
	int m_atomic;
	CArray<int,int> m_arrayCmdList;
	int m_nLength;
	CString GetMsg();
	char *m_pByteRead;
	unsigned m_nMsgID;
	//Default constructor
	~PowerMessage();
	PowerMessage();
	PowerMessage(char* byteRead,int nLength);
	BOOL ExchangeToMsg();
private:
	CString m_myMsg;

public:
	static int ToByte(char* sendByte, int msgID,int mainCommand,int nMyID, int nTargetID, CArray<int,int>* pCmdArray ,CString sendMessage, int atomic);
private:
	static UINT PByteToUnsingedInt(char* byteValue, UINT nCount);
	static void UnsingedIntToPByte(UINT nValue, char* byteValue, int nCount);
	static UINT OneByteToUnsignedInt(BYTE byteValue);
	static BYTE UnsignedIntToOneByte(UINT nValue);
public:
	int  GetSubCommand(int nIndex);
	//Converts the Data structure into an array of bytes

};

extern CPowerClient* gcPowerClient;


