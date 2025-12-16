#pragma once
#include "CPowerThread.h"
class CioStatus : public CPowerThread
{
public:
	CioStatus(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync);
	~CioStatus();
	
	void ReadDirectInput(unsigned index);
	void ReadDirectOutput(unsigned index);

	//void ReadDirectInput(unsigned index, unsigned mindx, unsigned char offset);
	//void SetDirectOutput(unsigned index, unsigned mindx, unsigned char offset);
	void Run();
	static UINT ReadIOStatus(LPVOID wParam);
	void InitInputTime();
private:
	long m_ShowID;
};

extern CioStatus* gcIOStatus;
