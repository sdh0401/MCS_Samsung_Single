#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"

class CFeeder : public CPowerThread
{
public:
	CFeeder(long FeederNo);
	~CFeeder();
	long GetNo();
	long GetReadyNo();
	long SetReadyNo(long ReadyNo);
	long GetReleaseNo();
	long SetReleaseNo(long ReadyNo);
	long SetReadyWaitTime(long WaitTime);
	FeederStep GetStep();
	void SetStep(FeederStep RunStep);
	void Run();
	static UINT StartFeeder(LPVOID wParam);
	bool IsEmpty();
	void SetEmtpy(bool Empty);
	long GetProdRunMode();
	void SetProdRunMode(long ProdRunMode);
private:
	FeederStep m_RunStep;
	long m_ShowID;
	long m_FeederNo;
	long m_ReadyNo;
	long m_ReleaseNo;
	bool m_Empty;
	long m_ProdRunMode;
	long m_ReadyWaitTime;
};

extern CFeeder* gcFeeder[MAXFEEDERNO];