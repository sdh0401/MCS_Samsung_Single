#pragma once
#include "GlobalDefine.h"

class CStep
{
public:
	CStep();
	~CStep();

	long GetPickupHeadNo(long PickOrd);
	long GetMaxPickOrder();
	long GetFdNoFromPickOrder(long PickOrd);
	long GetPickDelayFromFdNo(long FeederNo);
	Ratio_XYRZ GetRatioFromFdNo(long FeederNo);
	Point_XYRZ GetPickOffsetFromFdNo(long FeederNo);
	long GetFdNoFromInsertOrder(long insertOrd);
	MODULE_LED GetLed(long FeederNo);
	long GetMaxInsertOrder();
	long GetHeadNoFromInsertOrder(long insertOrd);
	double GetVAAngleFromFdNo(long FeederNo);
	double GetComponentHeightFromFdNo(long FeederNo);
	double GetComponentLeadHeightFromFdNo(long FeederNo);
	Point_XYRZ GetInsertPoint(long insertNo);
	long GetBlowDelayFromFdNo(long FeederNo);
	long GetReleaseDelayFromFdNo(long FeederNo);
	ORIGIN GetOrigin();
	long GetReturnHeadNo(long ReturnOrd);
	long GetMaxReturnComponentOrder();
	long GetFdNoFromReturnOrder(long ReturnOrd);
	long GetReturnDelayFromFdNo(long FeederNo);
	long GetManualCompensationUse();
	Point_XYRZ GetManualVisionResult();
	long GetRunStepNo(long insertNo);
	long GetUseFromInsertNo(long insertNo);
	long GetFeederUse(long FeederNo);
	long GetPickOrderFromInsertNo(long insertNo);
	long GetInsertOrderFromInsertNo(long insertNo);
	long GetHeadNoFromInsertNo(long insertNo);
	long GetFeederNoFromInsertNo(long insertNo);
	long GetInsertNoFromInsertOrder(long insertOrd);
	long GetReadyNoFromFeederNo(long FeederNo);
	long GetReleaseNoFromFeederNo(long FeederNo);
	long GetReadyTimeOutFromFeederNo(long FeederNo);
	long GetReadyWaitDelayFromFeederNo(long FeederNo);
	long GetDiscardMethod(long FeederNo);
	Point_XYRZ GetDiscardPoint(long FeederNo);
	long GetRecognitionTable(long insertNo);
	long GetPickRetryFromFdNo(long FeederNo);
	TwoStepMotion GetTwoStepPick(long FeederNo);
	TwoStepMotion GetTwoStepInsert(long FeederNo);
	double GetInsertZOffset(long FeederNo);
	double GetVAOffsetHeight(long FeederNo);
	long GetFeederReadyIOType(long FeederNo);
	long GetMaxInsertCount();
	double GetPickupZStandBy();
	double GetMaxComponentHeight();
	long GetSimultaneousLoading();
	double GetHighSpeedZOffset();
	long GetBoardNo();
	long SetNozzleNoFromInsertNo(long insertNo, long NozzleNo);
	long GetNozzleNoFromInsertNo(long insertNo);
	long GetAvoidCount(long insertNo);
	//////////////////////////////////////////////////////////////////////////////// RunTime
	long GetReadyTimeOutEmptyByHeadNo(long HeadNo);
	long GetReadyTimeOutEmpty(long FeederNo);
	long GetVisionErrorEmpty(long FeederNo);
	long GetVisionError(long FeederNo);
	long GetEmptyError(long insertOrd);
	bool GetSendEmpty(long FeederNo);
	User_PickOrder GetUserPickOrderFromIndex(long index);
	User_InsertOrder GetUserInsertOrderFromIndex(long index);
	bool GetRefill(long FeederNo);
	bool GetAllVisionError();
	bool GetAllEmptyError();
	void ClearVisionError();
	void ClearEmptyError();

	void SetMaxPickOrder(long MaxPickOrd);
	long SetPickupHeadNo(long PickOrd, long HeadNo);
	long SetFdNoFromPickOrder(long PickOrd, long FeederNo);
	long SetPickDelayFromFdNo(long FdNo, long PickDelay);
	long SetRatioFromFdNo(long FdNo, Ratio_XYRZ Ratio);
	long SetPickOffsetFromFdNo(long FdNo, Point_XYRZ pt);
	long SetLed(long FdNo, MODULE_LED Led);
	long SetMaxInsertOrder(long MaxInsertOrd);	
	long SetHeadNoFromInsertOrder(long insertOrd, long HeadNo);
	long SetFdNoFromInsertOrder(long insertOrd, long FeederNo);
	long SetInsertPoint(long insertNo, Point_XYRZ ptInsert);
	long SetVAAngleFromFdNo(long FdNo, double VAAngle);
	long SetComponentHeightFromFdNo(long FdNo, double Height);
	long SetComponentLeadHeightFromFdNo(long FdNo, double LeadHeight);
	long SetBlowDelayFromFdNo(long FdNo, long BlowDelay);
	long SetReleaseDelayFromFdNo(long FdNo, long ReleaseDelay);
	long SetOrigin(ORIGIN Origin);
	long SetReturnComponentHeadNo(long ReturnOrd, long ReturnHeadNo);
	long SetMaxReturnComponentOrder(long MaxReturnOrd);
	long SetFdNoFromReturnOrder(long PickOrd, long FeederNo);
	long SetManualCompensationUse(long ManualCompenUse);
	long SetManualVisionResult(Point_XYRZ VisionResult);
	long SetRunStepNo(long insertNo, long RunStepNo);
	long SetUseFromInsertNo(long insertNo, long Use);
	long SetFeederUse(long FeederNo, long Use);
	long SetPickOrderFromInsertNo(long insertNo, long PickOrder);
	long SetInsertOrderFromInsertNo(long insertNo, long InsertOrder);
	long SetHeadNoFromInsertNo(long insertNo, long InsertHeadNo);
	long SetFeederNoFromInsertNo(long insertNo, long InsertFeederNo);
	long SetInsertNoFromInsertOrder(long insertOrd, long insertNo);
	long SetReadyNoFromFeederNo(long FeederNo, long ReadyNo);
	long SetReleaseNoFromFeederNo(long FeederNo, long ReleaseNo);
	long SetReadyTimeOutFromFeederNo(long FeederNo, long TimeOut);
	long SetReadyWaitDelayFromFeederNo(long FeederNo, long ReadyWaitDelay);
	long SetDiscardMethod(long FeederNo, long Method);
	long SetDiscardPoint(long FeederNo, Point_XYRZ Point);
	long SetRecognitionTable(long insertNo, long RecogTable);
	long SetPickRetryFromFdNo(long FdNo, long PickRetry);
	long SetTwoStepPick(long FeederNo, TwoStepMotion TwoStepPick);
	long SetTwoStepInsert(long FeederNo, TwoStepMotion TwoStepInsert);
	long SetInsertZOffset(long FeederNo, double InsertZOffset);
	long SetMaxInsertCount(long MaxInsertCount);
	long SetPickupZStandBy(double PickupZStandBy);
	long SetMaxComponentHeight(double MaxComponentHeight);
	long SetSimultaneousLoading(long SimultaneousLoading);
	long SetBoardNo(long BoardNo);
	long SetVAOffsetHeight(long FeederNo, double VAOffsetHeight);
	long SetFeederReadyIOType(long FeederNo, long ReadyIOType);
	long SetAvoidCount(long insertNo, long AvoidCount);
	long SetPartEmptyStopByFeederNo(long FeederNo, long PartEmptyStop);
	long GetPartEmptyStopFromFeederNo(long FeederNo);
	//////////////////////////////////////////////////////////////////////////////// RunTime
	long SetReadyTimeOutEmptyByHeadNo(long HeadNo, long ReadyTimeOutEmpty);
	long SetReadyTimeOutEmpty(long FeederNo, long ReadyTimeOutEmpty);
	long ClearReadyTimeOutEmpty(long FeederNo);
	long SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty);
	long ClearVisionErrorEmpty(long FeederNo);
	long SetVisionError(long InsertOrd, long VisionError);
	long SetEmptyError(long insertOrd, long EmptyError);
	long SetSendEmpty(long FeederNo, bool bSendEmpty);
	long ClearSendEmpty(long FeederNo);
	long SetRefill(long FeederNo, bool bRefill);
	long ClearRefill(long FeederNo);
	long RefillDone();
	long SetUserPickOrderFromIndex(long index, User_PickOrder UserPickInfo);
	long ClearUserPickOrder();
	long SortUserPickOrder();
	long SetUserInsertOrderFromIndex(long index, User_InsertOrder UserInsertOrder);
	long ClearUserInsertOrder();
	long SortUserInsertOrder();
	long GetRecogTableBy1stInsertOrder();
	long SetMinRatio(Ratio_XYRZ MinRatio);
	Ratio_XYRZ GetMinRatio();
	long SetPickRatio(long PickOrder, Ratio_XYRZ Ratio);
	long SetInsertRatio(long InsertOrder, Ratio_XYRZ Ratio);
	Ratio_XYRZ GetPickRatio(long PickOrder);
	Ratio_XYRZ GetInsertRatio(long InsertOrder);

	void SetANCPrepare(long Gantry, long HeadNo, long NozzleNo);
	long GetANCPrepare(long Gantry, long HeadNo);
	long GetLaserControl(long FeederNo);
	long GetCatchDelay(long FeederNo);
	long SetLaserControl(long FeederNo, long LaserControl);
	long SetCatchDelay(long FeederNo, long CatchDelay);

	long SetRetryLed(long FeederNo, RETRY_LED RetryLed);
	RETRY_LED GetRetryLed(long FeederNo);

	long SetFeederType(long FeederNo, long Type);
	long GetFeederType(long FeederNo);
	long SetTray(long FeederNo, TRAY_INFO Tray);
	TRAY_INFO GetTray(long FeederNo);
	long SetTrayNowPocket(long FeederNo, long NowPocket);
	long GetTrayNowPocket(long FeederNo);
	long SetTrayMaxPocket(long FeederNo, long MaxPocket);
	long GetTrayMaxPocket(long FeederNo);
	long SetPickLevelCheck(long FeederNo, PICK_LEVEL_CHECK data);
	PICK_LEVEL_CHECK GetPickLevelCheck(long FeederNo);
	long SetTwoStepInsertUp(long FeederNo, TwoStepMotion TwoStepInsertUp);
	TwoStepMotion GetTwoStepInsertUp(long FeederNo);
	void SetFeederEmptyDisplay(long FeederNo, bool set);
	bool GetFeederEmptyDisplay(long FeederNo);
	void InitDivideInspect();
	long SetDivideInspect(long FeederNo, DIVIDE_INSPECT Divide);
	DIVIDE_INSPECT GetDivideInspect(long FeederNo);

	void SetPartTorqueLimit(long FeederNo, PARTTORQUELIMIT data);
	PARTTORQUELIMIT GetPartTorqueLimit(long FeederNo);
	
	long SetBarcode(BARCODE Barcode);
	BARCODE GetBarcode();
	long SetBarcodeInfoBlock1(BARCODE Barcode);
	BARCODE GetBarcodeInfoBlock1();
	long SetBarcodeInfoBlock2(BARCODE Barcode);
	BARCODE GetBarcodeInfoBlock2();
	long InitBarcode();
	long SetBarcodeCarrier(CString str);
	long SetBarcodeBlock1(CString str);
	long SetBarcodeBlock2(CString str);
	CString GetBarcodeCarrier();
	CString GetBarcodeBlock1();
	CString GetBarcodeBlock2();
	void SetBlockNoFromInsertNo(long InsertNo, long BlockNo);
	long GetBlockNoFromInsertNo(long InsertNo);
	void InitBlockSkipHM();
	void SetBlockSkipHM(long BlockNo, long Skip);
	long GetBlockSkipHM(long BlockNo);
	void SetBlockUse(long BlockNo, long Use);
	long GetBlockUse(long BlockNo);
	long GetBlockUseFromInsertNo(long InsertNo);
	void SetBlockSequence(long Seq);
	long GetBlockSequence();
	void SetAllBlockSkipHM(bool AllSkip);
	bool GetAllBlockSkipHM();
	long SetForming(long FeederNo, FORMING_COMPONENT Forming);
	FORMING_COMPONENT GetForming(long FeederNo);
	void SetFormingMotionDone(long insertOrd, long bDone);
	bool GetFormingMotionDone(long insertOrd);

private:
	long Initilize();
	/////////////////////////////////////////////////////////////// JobFile
	long m_MaxPickOrder;
	long m_MaxInsertOrder;
	long m_MaxReturnComponentOrder;
	double m_MaxComponentHeight;
	long m_SimultaneousLoading;
	long m_BoardNo;
	long m_UserPickOrderCount;
	long m_UserInsertOrderCount;
	long m_PickHeadNo[MAXUSEDHEADNO];				// Pick Order
	long m_PickFeederNo[MAXUSEDHEADNO];				// Pick Order
	User_PickOrder m_UserPickOrder[MAXUSEDHEADNO];	// Pick Order
	User_InsertOrder m_UserInsertOrder[MAXUSEDHEADNO];	// Insert Order
	long m_FeederUse[MAXFEEDERNO];					// Feeder No
	long m_PickupDelayTime[MAXFEEDERNO];			// Feeder No
	Ratio_XYRZ m_Ratio[MAXFEEDERNO];				// Feeder No
	Point_XYRZ m_PickOffset[MAXFEEDERNO];			// Feeder No
	long m_PickRetry[MAXFEEDERNO];					// Feeder No
	TwoStepMotion m_TwoStepPick[MAXFEEDERNO];		// Feeder No
	TwoStepMotion m_TwoStepInsert[MAXFEEDERNO];		// Feeder No
	TwoStepMotion m_TwoStepInsertUp[MAXFEEDERNO];		// Feeder No
	double m_InsertZOffset[MAXFEEDERNO];			// Feeder No
	long m_InsertHeadNo[MAXUSEDHEADNO];				// Insert Order
	long m_InsertNo[MAXUSEDHEADNO];					// Insert Order
	MODULE_LED m_Led[MAXFEEDERNO];					// Feeder No
	long m_InsertFeederNo[MAXUSEDHEADNO];			// Insert Order
	double m_VAAngle[MAXFEEDERNO];					// Feeder No
	double m_ComponentHeight[MAXFEEDERNO];			// Feeder No
	double m_ComponentLeadHeight[MAXFEEDERNO];		// Feeder No
	long m_RecogTable[MAXINSERTNO];					// Insert No
	Point_XYRZ m_Insert[MAXINSERTNO];				// Insert No
	long m_RunStepNo[MAXINSERTNO];					// Insert No
	long m_InsertUse[MAXINSERTNO];					// Insert No
	long m_PickOrder[MAXINSERTNO];					// Insert No
	long m_InsertOrder[MAXINSERTNO];				// Insert No
	long m_InsertHeadNoFromInsertNo[MAXINSERTNO];	// Insert No
	long m_InsertFeederNoFromInsertNo[MAXINSERTNO];	// Insert No
	long m_InsertBlowDelayTime[MAXFEEDERNO];		// Feeder No
	long m_InsertReleaseDelayTime[MAXFEEDERNO];		// Feeder No
	long m_ReadyNo[MAXFEEDERNO];					// Feeder No
	long m_ReleaseNo[MAXFEEDERNO];					// Feeder No
	long m_ReadyTimeOut[MAXFEEDERNO];				// Feeder No
	long m_ReadyWaitDelay[MAXFEEDERNO];				// Feeder No
	long m_DiscardMethod[MAXFEEDERNO];				// Feeder No
	Point_XYRZ m_DiscardPoint[MAXFEEDERNO];			// Feeder No
	double m_VARecognitionOffsetHeight[MAXFEEDERNO];// Feeder No
	long m_FeederReadyIOType[MAXFEEDERNO];			// Feeder No
	long m_AvoidCount[MAXINSERTNO];					// Insert No
	long m_PartEmptyStop[MAXFEEDERNO];				// Feeder No
	ORIGIN m_Origin;
	long m_ReturnComponentHeadNo[MAXUSEDHEADNO];	// Return Order
	long m_ReturnComponentFeederNo[MAXUSEDHEADNO];	// Return Order
	long m_ManualCompensationUse;
	Point_XYRZ m_ManualVisionResult;
	long m_MaxInsertCount;
	double m_PickupZStandBy;
	double m_HighSpeedOffset;
	long m_InsertNozzleNoFromInsertNo[MAXINSERTNO];	// Insert No
	/////////////////////////////////////////////////////////////// RunTime
	long m_ReadyTimeOutEmptyByHeadNo[MAXUSEDHEADNO];// Head No
	long m_ReadyTimeOutEmpty[MAXFEEDERNO];			// Feeder No
	long m_VisionErrorEmpty[MAXFEEDERNO];			// Feeder No
	long m_VisionError[MAXUSEDHEADNO];				// Insert Order
	long m_EmptyError[MAXUSEDHEADNO];				// Insert Order
	bool m_bSendEmpty[MAXFEEDERNO];					// 
	bool m_bFeederRefill[MAXFEEDERNO];				// 
	Ratio_XYRZ m_MinStepRatio;						// Minimum Ratio
	Ratio_XYRZ m_PickRatio[MAXUSEDHEADNO];			// Insert Order
	Ratio_XYRZ m_InsertRatio[MAXUSEDHEADNO];		// Insert Order

	long m_ANCPrepareFront[MAXUSEDHEADNO];
	long m_ANCPrepareRear[MAXUSEDHEADNO];

	long m_LaserControl[MAXFEEDERNO];				// Feeder No
	long m_CatchDelay[MAXFEEDERNO];					// Feeder No

	RETRY_LED m_RetryLed[MAXFEEDERNO];				// Feeder No

	TRAY_INFO m_Tray[MAXFEEDERNO];					// Feeder No
	long m_FeederType[MAXFEEDERNO];					// Feeder No
	long m_TrayNowPocket[MAXFEEDERNO];				// Feeder No
	long m_TrayMaxPocket[MAXFEEDERNO];				// Feeder No
	PICK_LEVEL_CHECK m_PickLevelCheck[MAXFEEDERNO];

	bool m_bFeederEmptyDisplay[MAXFEEDERNO];
	DIVIDE_INSPECT m_DivideInspect[MAXFEEDERNO];

	PARTTORQUELIMIT m_PartTorqueLimit[MAXFEEDERNO];

	BARCODE m_Barcode, m_BarcodeBlock1, m_BarcodeBlock2;
	CString strBarcodeCarrier, strBarcodeBlock1, strBarcodeBlock2;
	long m_BlockNo[MAXINSERTNO];
	long m_BlockSkipHM[MAXBLOCKNO]; // 0:Prod, 1:Skip
	long m_BlockUse[MAXBLOCKNO];
	long m_BlockSequence;
	bool m_AllBlockSkipHM;
	FORMING_COMPONENT m_Forming[MAXFEEDERNO];		// Feeder No
	bool m_bFormingMotionDone[MAXUSEDHEADNO];		// Insert Order
};

extern CStep* gcStep;
