#pragma once
#include "GlobalDefine.h"

class CDiscard
{
public:
	CDiscard(long Gantry);
	~CDiscard();
	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	double GetPickupZStandBy();
	long SetSendEmpty(long FeederNo, bool bSendEmpty);
	long GetMaxInsertOrder();
	long GetHeadNoFromInsertOrder(long insertOrd);
	long GetFdNoFromInsertOrder(long insertOrd);
	long GetVisionErrorEmpty(long FeederNo);
	long GetReadyTimeOutEmpty(long FeederNo);
	long GetReadyTimeOutEmptyByHeadNo(long HeadNo);
	long DiscardOneBeforePicking(long HeadNo, Ratio_XYRZ ratio);
	long DiscardAllBeforePicking(Ratio_XYRZ ratio);
	long DiscardOneBeforeAlignChecking(long FeederNo, long HeadNo);
	long DiscardAllBeforeAlignChecking();
	long DiscardOneAfterAlignChecking(long FeederNo, long HeadNo);
	long DiscardAllAfterAlignChecking();
	long DiscardOneAfterInserting(long FeederNo, long HeadNo);
	long DiscardAllAfterInserting();
	long GetVisionError(long insertOrd);
	//void PartDropLedOn(long CamTable);
	//void PartDropProcess(long CamTable);
	//long PartDropGetResult(long CamTable);
	//long PartDropCheckRear();
	void SetFeederEmptyDisplay(long FeederNo, bool set);
	bool GetFeederEmptyDisplay(long FeederNo);
	void SendDiscardStatics(long FeederNo, long HeadNo);

	long DiscardAllNormalMode(Ratio_XYRZ ratio);

private:
	long m_Gantry;
	long m_ProdRunMode;
};

extern CDiscard* gcDiscard;
