#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"

class CSyncGantryConveyor : public CPowerThread
{
public:
	CSyncGantryConveyor();
	~CSyncGantryConveyor();
	void Run();
	SyncGantryConveyorStep GetStep();

private:
	static UINT StartSyncGantryConveyor(LPVOID wParam);
	long m_ShowID;
	SyncGantryConveyorStep m_SyncGantryConveyorStep;
};

extern CSyncGantryConveyor* gcSyncGantryConveyor;