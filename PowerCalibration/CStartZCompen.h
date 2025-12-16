#pragma once
#include "CPowerThread.h"
#include <CompensationApi.h>

using namespace wmx3Api;
class CStartZCompen : public CPowerThread
{
public:
	CStartZCompen();
	CStartZCompen(HANDLE h_Terminate);
	~CStartZCompen();
	void ClearZCompen();
	void SetZCompenMode(bool bApply);
	bool GetZCompenMode();
	void ClearStep();
	void SetZAxis(CString strAxisZ);
	CString GetZAxis();
	long CopyGlobalZCompensationData(long MaxCnt, double StartCompensation, double StartSensorHeight);
	void Run();
	static UINT StartZCompenCalibration(LPVOID wParam);
	CalibrationZCompenStep m_ZCompenStep;
	void SetPlusZDiff(unsigned indx, double dblDiff);
	void SetMinusZDiff(unsigned indx, double dblDiff);
	void ShowZMeanData(unsigned nCompenMax);
	void ShowZRawData(unsigned nCompenMax);
private:
	double dblMean[BUFSIZE];
	double dblPlusCompen[BUFSIZE];
	double dblMinusCompen[BUFSIZE];
	bool m_bApply;
	PitchErrorCompensationData oneDCompData;
	long m_ShowID;
	CString m_strAxisZ;
};

extern CStartZCompen* gcStartZCompen;