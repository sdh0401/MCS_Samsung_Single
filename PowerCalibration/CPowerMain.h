#pragma once
#include "CPowerThread.h"

class CPowerMain : public CPowerThread
{
public:
	CPowerMain();
	~CPowerMain();
	void Prepare();
	static UINT StartMain(LPVOID wParam);
	void Run();
	void SetStep(MainStep RunStep);
	long PrepareJob();
	long SetJobInfo();
	long MakeStepRun(long LastInsertNo);
	long CheckFeederAutoRefill();
	long CopyLastStep();
	long MoveStandBy();
	long GetRemainStartNo();
	long GetRemainStartNoWithEmpty();
	void Initial();
	void InitPcb();
	void InitPackage(long Count);
	void InitFeeder(long Count);
	void InitRetryLed(long Count);
	void ClearVisionError();
	void ClearEmptyError();
	bool GetAllVisionError();
	bool GetAllEmptyError();
	long SetRefill(long FeederNo, bool bRefill);
	long RefillDone();
	long MoveSaftyZBeforePickup();
	long MoveSaftyZAfterPickup();
	ULONGLONG GetConveyorLoadingTime();
	ULONGLONG GetConveyorLineOfBalance();
	long GetMaxInsertOrder();
	long GetFdNoFromInsertOrder(long insertOrd);
	long GetHeadNoFromInsertOrder(long insertOrd);
	double GetComponentHeight(long FdNo);
	double GetComponentLeadHeight(long FdNo);
	long GetInsertNoFromInsertOrder(long insertOrd);
	double GetMaxComponentHeight();
	Ratio_XYRZ GetComponentRatioByFdNo(long FdNo);
	long MoveZUpAfterPickup(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait);
	long WaitZUpAfterPickup(CString strAxis, double CmdPos, double Inpos, long TimeOut);
	long GetRecogTableBy1stInsertOrder();
	long PartDropCheck(long CamTable);
	void PrepareLastPickData();
	long GetFirstInsertNoLastPickData();
	Ratio_XYRZ GetMinRatioLastPickData();
	CString GetStepName(MainStep RunStep);
	void SetHMFirst(bool set);
	bool GetHMFirst();
	long SendHeightMeasureEnd();
	CString GetBarcodeCarrier(); 
	CString GetBarcodeBlock1(); 
	CString GetBarcodeBlock2(); 
	long StartPrevProdHeightMeasure();
	bool CheckBlockSkipHMUse();
	long MES_INIT();
	long MES_NG();
	long MES_OK();
	long MES_ABORT();
	long MES_GET();
	long SetBoardNo(long BoardNo);
	long MES_Disconnect();
	void SetBoardTimeExcute(bool Set);
	bool GetBoardTimeExcute();
	ULONGLONG GetBoardTimeElapsed();

private:
    long GetTable() const;
	HANDLE m_StepLock;
	long m_Gantry;
	long m_LastBlockNo, m_LastInsertNo;
	long m_RealMaxInsertCount;
	MainStep GetStep();
	long m_ShowID;
	MainStep m_RunStep;
	PRODUCTION m_Production;
	PCB m_Pcb;
	STANDBY m_StandBy;
	long m_MES_Result;
	bool m_HMfirst;
	ULONGLONG m_BoardStartTime;
	bool m_SetBoardStart;

    // √‚√≥ : https://vcpptips.wordpress.com/tag/getfileversioninfo/
    void traceMcsVersion();
	long calculateProductionTime(const long& ProdQuantity);
};

extern CPowerMain* gcPowerMain;
