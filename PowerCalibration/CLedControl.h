#pragma once
#include "GlobalDefine.h"
#include "SerialCom.h"
#include "Thread.h"

class CLedControl : public CThread
{
public:
	CLedControl();
	~CLedControl();
	void OpenPort(CString strPort, int BaudRate);
	void ClosePort();
	void SetBaudRate(int BaudRate);
	int GetBaudRate();
	void LedOn(int CameraNo, int iValue1, int iValue2, int iValue3);
	void LedOff(int CameraNo);
	void LedAllOff();
	void LedAllOn();
	CSerialComm m_SerialComm;
private:
	int m_BaudRate;
	static void ReadSerial(LPVOID owner, char* pBuffer, int nRead);
	void WriteSerialValue(int ChNo, int Value, int All);
	void WriteSerialOnOff(int ChNo, bool bOn);
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
};

extern CLedControl* gcLedControl;