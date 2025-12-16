#pragma once
#include "CPowerThread.h"
class CPowerSwitchPanel : public CPowerThread
{
public:
	CPowerSwitchPanel();
	~CPowerSwitchPanel();
	static UINT ReadSwitchPanel(LPVOID wParam);
	void Run();
	SwitchPanelStep GetStep();
	void SetStep(SwitchPanelStep SwitchPanelStep);
private:
	long m_ShowID;
	SwitchPanelStep m_SwitchPanelStep;
};

extern CPowerSwitchPanel* gcPowerSwitchPanel;