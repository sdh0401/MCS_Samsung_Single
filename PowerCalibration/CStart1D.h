#pragma once
#include "CPowerThread.h"
#include <CompensationApi.h>

using namespace wmx3Api;

class CStart1D : public CPowerThread
{
public:
	CStart1D();
	CStart1D(HANDLE h_Terminate);
	~CStart1D();
	void Clear1DCompen();
	void Set1DCompenMode(bool bApply);
	bool Get1DCompenMode();
	void ClearStep();
	//int Set1DCompensation();
	//int Set1DCompensation(unsigned nMaxCount);
	int CopyGlobalCompensationData();
	int CopyGlobalCompensationData(long MaxCnt);
	int PasteGlobalCompensationData();
	void Run();
	static UINT Start1DCalibration(LPVOID wParam);
	Calibration1DStep m_1DStep;
	void SetPlus1DDiff(unsigned indx, double dblDiff);
	void SetMinus1DDiff(unsigned indx, double dblDiff);
	void SetPlusTorque(unsigned indx, double dblDiff);
	void SetMinusTorque(unsigned indx, double dblDiff);
	void Show1DMeanData(unsigned nCompenMax);
	void Show1DRawData();
	void Show1DRawData(unsigned nCompenMax);
	void ShowTorqueData();
	void ShowTorqueData(unsigned nCompenMax);
private:
	long m_Gantry;
	double dblMean[BUFSIZE];
	double dblPlusCompen[BUFSIZE];
	double dblMinusCompen[BUFSIZE];
	double dblPlusTorque[BUFSIZE];
	double dblMinusTorque[BUFSIZE];
	bool m_bApply;
	PitchErrorCompensationData oneDCompData;
	CString m_StrX;
	CString m_StrY1;
	CString m_StrY2;
	long m_ShowID;
};

extern CStart1D* gcStart1D;