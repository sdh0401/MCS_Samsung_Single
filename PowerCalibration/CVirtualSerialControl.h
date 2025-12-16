#pragma once
#include "GlobalDefine.h"
#include "SerialCom.h"
#include "Thread.h"

class CVirtualSerialControl : public CThread
{
public:
	CVirtualSerialControl();
	~CVirtualSerialControl();
	void OpenPort(CString strPort, int BaudRate);
	void ClosePort();
	void SetBaudRate(int BaudRate);
	int GetBaudRate();
	CSerialComm m_SerialComm;
	void WriteBarcode(CString strBarcode);
private:
	int m_BaudRate;
	static void ReadSerial(LPVOID owner, char* pBuffer, int nRead);
	void WriteSerialValue(CString strBarcode);
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
};

extern CVirtualSerialControl* gcVirtualSerialControl;