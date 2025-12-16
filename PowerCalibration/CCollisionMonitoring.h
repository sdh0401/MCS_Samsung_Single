#pragma once
#include "CPowerThread.h"
class CCollisionMonitoring : public CPowerThread
{
public:
	CCollisionMonitoring();
	~CCollisionMonitoring();
	//COLLISION_MONITORING GetMonitoringData(long Gantry);
	static UINT CollisionMonitoring(LPVOID wParam);
	void Run();
private:
	long m_ShowID;
};

extern CCollisionMonitoring* gcCollisionMonitoring;