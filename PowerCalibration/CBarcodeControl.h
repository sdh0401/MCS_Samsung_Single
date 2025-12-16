#pragma once
#include "GlobalDefine.h"
#include "SerialCom.h"
#include "Thread.h"

class CBarcodeControl : public CThread
{
public:
	CBarcodeControl();
	~CBarcodeControl();
	bool OpenPort(CString strPort, int BaudRate);
	void ClosePort();
	void SetBaudRate(int BaudRate);
	int GetBaudRate();
	void TriggerOn();
	void TriggerOff();
	void SetTriggerType(long no);
	void SetDataMatrixEnable(bool Enable);
	void SetQRCodeEnable(bool Enable);
	void LightOnOff(bool On);
	CSerialComm m_SerialComm;
	void SetBarcode(CString strBarcode);
	CString GetBarcode();
	void InitBarcode();
	void SetDefaultCognexParameter();
	bool GetComPortStatus();
	CString InspectBarcodeCognex(long Type, long TimeOut);
private:
	int m_BaudRate;
	CString m_Barcode;
	CString m_ReceiveBuffer;
	static void ReadSerial(LPVOID owner, char* pBuffer, int nRead);
	void WriteSerialValue(int ChNo, int Value, int All);
	void WriteSerialOnOff(int ChNo, bool bOn);
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	HANDLE m_Lock;
	void Lock();
	void Unlock();
};

extern CBarcodeControl* gcBarcodeControl;