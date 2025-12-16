#pragma once
#include "CPowerThread.h"
#include <WMX3Api.h>

using namespace wmx3Api;

class Cwmx3IO : public CPowerThread
{
public:
	Cwmx3IO(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	~Cwmx3IO();
	void InitialMemory();
	static UINT ReadWMX3IOStatus(LPVOID wParam);
	void Run();
private:
	unsigned char inData[WMX3_HW_MAXIO];
	unsigned char outData[WMX3_HW_MAXIO];

	//unsigned char GetOutData[wmx3Api::constants::maxIoOutSize];
	//unsigned char SetOutData[wmx3Api::constants::maxIoOutSize];
	long m_ShowID;
};

extern Cwmx3IO* gcwmx3IO;