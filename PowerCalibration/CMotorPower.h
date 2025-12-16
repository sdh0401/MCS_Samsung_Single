#pragma once
#include "CPowerThread.h"
class CMotorPower : public CPowerThread
{
public:
	CMotorPower();
	~CMotorPower();
    static const char* GetStepName(const MotorPowerStep& step);
	long WaitMotorPowerOn(long TimeOut);
	static UINT MotorPowerOn(LPVOID wParam);
	void Run();
	MotorPowerStep GetStep();
	void SetStep(MotorPowerStep MotorPowerStep);
private:
	long m_ShowID;
	MotorPowerStep m_MotorPowerStep;
};

extern CMotorPower* gcMotorPower;