#pragma once
#include "CPowerThread.h"

class CAnalogStatus : public CPowerThread
{
public:
	CAnalogStatus();
	~CAnalogStatus();
	void InitialMemory();
	static UINT ReadAnalogStatus(LPVOID wParam);
	void Run();
private:
	long m_ShowID;
};

extern CAnalogStatus* gcAnalogStatus;