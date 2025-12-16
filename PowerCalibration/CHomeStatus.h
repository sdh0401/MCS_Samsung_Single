#pragma once
#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <ecApi.h>
#include "GlobalDefine.h"
#include "CMessageQueue.h"
#include "CApplicationTime.h"
#include "CPowerThread.h"

class CHomeStatus : public CPowerThread
{
public:
	CHomeStatus();
	~CHomeStatus();
	void WaitAllAxisHomingThread();
	void Run();
	static UINT StartAllHoming(LPVOID wParam);
	void AddAxisThreadID(THREAD_STRUCT* pThreadInfo);
	THREAD_STRUCT* GetAxisThreadByIndex(INT_PTR indx);
	INT_PTR GetAxisThreadCount();
	void RemoveAxisThread(INT_PTR indx);
	void RemoveAllAxis();
	int GetAxisThreadIDByIndex(INT_PTR indx);
	long Y2ShiftAutoInit(long Gantry);
private:
	CArray<THREAD_STRUCT*, THREAD_STRUCT*> m_HomingAxisIDArray;
	long m_ShowID;
};

extern CHomeStatus* gcHomeStatus;
