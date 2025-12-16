#pragma once
#include "CPowerThread.h"
#include "Cwmx3Axis.h"

class COriginSearch : public CPowerThread
{
public:
	COriginSearch(CString strAxis);
	~COriginSearch();
	void Run();
	static UINT StartOriginSearch(LPVOID wParam);
	CString GetAxisName();
	HomingStep GetStep();
	long GetHomingErr();
	bool isHomingFail();

private:
	HomingStep GetStartHomingStep();
	long SetHomingErr(long HomingErr);
	CString m_Axis;
	Cwmx3Axis* m_pAxis;
	HomingStep m_HomingStep;	// Axis Homing Step
	long m_ShowID;
	long m_HomingErr;
};

