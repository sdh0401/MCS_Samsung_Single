#include "pch.h"
#include "CStep.h"
#include "GlobalData.h"
#include "AxisInformation.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "Trace.h"

CStep* gcStep;
CStep::CStep()
{
	Initilize();
}

CStep::~CStep()
{
}

long CStep::Initilize()
{
	m_MaxInsertCount = 0;
	m_MaxPickOrder = 0;
	m_MaxInsertOrder = 0;
	m_MaxReturnComponentOrder = 0;
	m_UserPickOrderCount = 0;
	m_UserInsertOrderCount = 0;
	m_ManualCompensationUse = 0;
	m_MaxComponentHeight = 10.0;
	m_PickupZStandBy = 70.0;
	m_HighSpeedOffset = 5.0;
	ZeroMemory(&m_PickHeadNo, sizeof(m_PickHeadNo));			// Pick Order
	ZeroMemory(&m_UserPickOrder, sizeof(m_UserPickOrder));		// User Pick Order
	ZeroMemory(&m_PickFeederNo, sizeof(m_PickFeederNo));
	ZeroMemory(&m_PickupDelayTime, sizeof(m_PickupDelayTime));
	ZeroMemory(&m_Ratio, sizeof(m_Ratio));
	ZeroMemory(&m_PickOffset, sizeof(m_PickOffset));
	ZeroMemory(&m_PickRetry, sizeof(m_PickRetry));
	ZeroMemory(&m_InsertHeadNo, sizeof(m_InsertHeadNo));
	ZeroMemory(&m_InsertNo, sizeof(m_InsertNo));
	ZeroMemory(&m_Led, sizeof(m_Led));
	ZeroMemory(&m_InsertFeederNo, sizeof(m_InsertFeederNo));
	ZeroMemory(&m_VAAngle, sizeof(m_VAAngle));
	ZeroMemory(&m_ComponentHeight, sizeof(m_ComponentHeight));
	ZeroMemory(&m_ComponentLeadHeight, sizeof(m_ComponentLeadHeight));
	ZeroMemory(&m_RecogTable, sizeof(m_RecogTable));
	ZeroMemory(&m_Insert, sizeof(m_Insert));
	ZeroMemory(&m_InsertBlowDelayTime, sizeof(m_InsertBlowDelayTime));
	ZeroMemory(&m_InsertReleaseDelayTime, sizeof(m_InsertReleaseDelayTime));
	ZeroMemory(&m_ReturnComponentHeadNo, sizeof(m_ReturnComponentHeadNo));
	ZeroMemory(&m_ReturnComponentFeederNo, sizeof(m_ReturnComponentFeederNo));
	ZeroMemory(&m_ManualVisionResult, sizeof(m_ManualVisionResult));
	ZeroMemory(&m_InsertZOffset, sizeof(m_InsertZOffset));
	////////////////////////////////////////////////////////////////////////////////
	ZeroMemory(&m_ReadyTimeOutEmptyByHeadNo, sizeof(m_ReadyTimeOutEmptyByHeadNo));
	ZeroMemory(&m_ReadyTimeOutEmpty, sizeof(m_ReadyTimeOutEmpty));
	ZeroMemory(&m_VisionErrorEmpty, sizeof(m_VisionErrorEmpty));
	ZeroMemory(&m_VisionError, sizeof(m_VisionError));	
	ZeroMemory(&m_bFeederRefill, sizeof(m_bFeederRefill));
	ZeroMemory(&m_PartEmptyStop, sizeof(m_PartEmptyStop));
	ZeroMemory(&m_ANCPrepareFront, sizeof(m_ANCPrepareFront));
	ZeroMemory(&m_ANCPrepareRear, sizeof(m_ANCPrepareRear));
	ZeroMemory(&m_LaserControl, sizeof(m_LaserControl));
	ZeroMemory(&m_CatchDelay, sizeof(m_CatchDelay));

	m_MinStepRatio.xy = m_MinStepRatio.r = m_MinStepRatio.z = 1.0;
	for (long insertOrder = 0; insertOrder < MAXUSEDHEADNO; ++insertOrder)
	{
		m_PickRatio[insertOrder].xy = m_PickRatio[insertOrder].r = m_PickRatio[insertOrder].z = 1.0;
		m_InsertRatio[insertOrder].xy = m_InsertRatio[insertOrder].r = m_InsertRatio[insertOrder].z = 1.0;
	}	

	for (long feeder = 0; feeder < MAXFEEDERNO; feeder++)
	{
		m_Tray[feeder].Name.Format(_T("Tray%d"), feeder);
		m_RetryLed[feeder].PackageName.Format(_T("Package%d"), feeder);
	}
	ZeroMemory(&m_FeederType, sizeof(m_FeederType));
	ZeroMemory(&m_TrayNowPocket, sizeof(m_TrayNowPocket));
	ZeroMemory(&m_TrayMaxPocket, sizeof(m_TrayMaxPocket));
	ZeroMemory(&m_PickLevelCheck, sizeof(m_PickLevelCheck));

	ZeroMemory(&m_TwoStepPick, sizeof(m_TwoStepPick));
	ZeroMemory(&m_TwoStepInsert, sizeof(m_TwoStepInsert));
	ZeroMemory(&m_TwoStepInsertUp, sizeof(m_TwoStepInsertUp));
	ZeroMemory(&m_bFeederEmptyDisplay, sizeof(m_bFeederEmptyDisplay));

	InitDivideInspect();

	for (long feeder = 0; feeder < MAXFEEDERNO; feeder++)
	{
		m_PartTorqueLimit[feeder].PickDown.Use = false;
		m_PartTorqueLimit[feeder].PickDown2nd.Use = false;
		m_PartTorqueLimit[feeder].InsertDown.Use = false;
		m_PartTorqueLimit[feeder].InsertDown2nd.Use = false;

		m_PartTorqueLimit[feeder].PickDown.TorqueLimit = 0.0;
		m_PartTorqueLimit[feeder].PickDown2nd.TorqueLimit = 0.0;
		m_PartTorqueLimit[feeder].InsertDown.TorqueLimit = 0.0;
		m_PartTorqueLimit[feeder].InsertDown2nd.TorqueLimit = 0.0;
	}

	ClearEmptyError();
	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
	{
		ClearSendEmpty(FeederNo + 1);
	}

	ZeroMemory(&m_Forming, sizeof(m_Forming));
	ZeroMemory(&m_bFormingMotionDone, sizeof(m_bFormingMotionDone));

	return NO_ERR;
}

long CStep::GetPickupHeadNo(long PickOrd)
{
	long HeadNo = 0;
	if (PickOrd > 0 && PickOrd <= MAXUSEDHEADNO)
	{
		HeadNo = m_PickHeadNo[PickOrd - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetPickupHeadNo PickOrd:%d HeadNo:%d\n"), PickOrd, HeadNo);
	}
	return HeadNo;
}

long CStep::GetMaxPickOrder()
{
	return m_MaxPickOrder;
}

long CStep::GetFdNoFromPickOrder(long PickOrd)
{
	long FdNo = 0;
	if (PickOrd > 0 && PickOrd <= MAXUSEDHEADNO)
	{
		FdNo = m_PickFeederNo[PickOrd - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetFdNoFromPickOrder PickOrd:%d FeederNo:%d\n"), PickOrd, FdNo);
	}
	return FdNo;
}

long CStep::GetPickDelayFromFdNo(long FeederNo)
{
	long PickDelay = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		PickDelay = m_PickupDelayTime[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetPickDelayFromFdNo FeederNo:%d PIckDelay:%d\n"), FeederNo, PickDelay);
	}
	return PickDelay;
}

Ratio_XYRZ CStep::GetRatioFromFdNo(long FeederNo)
{
	Ratio_XYRZ Ratio;
	ZeroMemory(&Ratio, sizeof(Ratio));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Ratio = m_Ratio[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetRatioFromFdNo FeederNo:%d RatioXY,R,Z:%.1f,%.1f,%.1f\n"), FeederNo, Ratio.xy, Ratio.r, Ratio.z);
	}
	return Ratio;
}

Point_XYRZ CStep::GetPickOffsetFromFdNo(long FeederNo)
{
	Point_XYRZ PickOffset;
	ZeroMemory(&PickOffset, sizeof(PickOffset));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		PickOffset = m_PickOffset[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetPickOffsetFromFdNo FeederNo:%d PickOffsetX,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), FeederNo, PickOffset.x, PickOffset.y, PickOffset.r, PickOffset.z);
	}
	return PickOffset;
}

long CStep::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		FdNo = m_InsertFeederNo[insertOrd - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetFdNoFromInsertOrder insertOrd:%d Fd:%d\n"), insertOrd, FdNo);
	}
	return FdNo;
}

MODULE_LED CStep::GetLed(long FeederNo)
{
	MODULE_LED Led;
	ZeroMemory(&Led, sizeof(Led));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Led = m_Led[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetLed FeederNo:%d Led(Top,Mid,Bot)(%d,%d,%d)\n"), FeederNo, Led.Top, Led.Mid, Led.Bot);
	}
	return Led;
}

long CStep::GetMaxInsertOrder()
{
	return m_MaxInsertOrder;
}

long CStep::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		HeadNo =  m_InsertHeadNo[insertOrd - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder insertOrd:%d HeadNo:%d\n"), insertOrd, HeadNo);
	return HeadNo;
}

double CStep::GetVAAngleFromFdNo(long FeederNo)
{
	double VAAngle = 0.0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		VAAngle = m_VAAngle[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetVAAngleFromFdNo FeederNo:%d VAAngle:%.3f\n"), FeederNo, VAAngle);
	}
	return VAAngle;
}

double CStep::GetComponentHeightFromFdNo(long FeederNo)
{
	double Height = 12.345;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Height = m_ComponentHeight[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetComponentHeightFromFdNo FeederNo:%d ComponentHeight:%.3f\n"), FeederNo, Height);
	}
	return Height;
}

double CStep::GetComponentLeadHeightFromFdNo(long FeederNo)
{
	double LeadHeight = 3.456;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		LeadHeight = m_ComponentLeadHeight[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetComponentLeadHeightFromFdNo FeederNo:%d LeadHeight:%.3f\n"), FeederNo, LeadHeight);
	}
	return LeadHeight;
}

Point_XYRZ CStep::GetInsertPoint(long insertNo)
{
	Point_XYRZ pt;
	ZeroMemory(&pt, sizeof(pt));
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		pt = m_Insert[insertNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetInsertPoint insertNo:%04d Pt.X,Y,R,Z:%.3f,%.3f,%.3f,%.3f\n"), insertNo, pt.x, pt.y, pt.r, pt.z);
	}
	return pt;
}

long CStep::GetBlowDelayFromFdNo(long FeederNo)
{
	long BlowDelay = TIME1000MS;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		BlowDelay = m_InsertBlowDelayTime[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetBlowDelayFromFdNo FeederNo:%04d BlowDelay:%d\n"), FeederNo, BlowDelay);
	}
	return BlowDelay;
}

long CStep::GetReleaseDelayFromFdNo(long FeederNo)
{
	long ReleaseDelay = TIME1000MS;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReleaseDelay = m_InsertReleaseDelayTime[FeederNo - 1];
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReleaseDelayFromFdNo FeederNo:%04d ReleaseDelay:%d\n"), FeederNo, ReleaseDelay);
	}
	return ReleaseDelay;
}

ORIGIN CStep::GetOrigin()
{
	return m_Origin;
}

long CStep::GetReturnHeadNo(long ReturnOrd)
{
	long HeadNo = 0;
	if (ReturnOrd > 0 && ReturnOrd <= MAXUSEDHEADNO)
	{
		HeadNo = m_ReturnComponentHeadNo[ReturnOrd - 1];
	}
	return HeadNo;
}

long CStep::GetMaxReturnComponentOrder()
{
	return m_MaxReturnComponentOrder;
}

long CStep::GetFdNoFromReturnOrder(long ReturnOrd)
{
	long FdNo = 0;
	if (ReturnOrd > 0 && ReturnOrd <= MAXUSEDHEADNO)
	{
		FdNo = m_ReturnComponentFeederNo[ReturnOrd - 1];
	}
	return FdNo;
}

long CStep::GetReturnDelayFromFdNo(long FeederNo)
{
	long PickDelay = TIME1000MS;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		PickDelay = m_PickupDelayTime[FeederNo - 1];
	}
	return PickDelay;
}

long CStep::GetManualCompensationUse()
{
	long ManualCompenUse = 0;
	ManualCompenUse = m_ManualCompensationUse;
	return ManualCompenUse;
}

Point_XYRZ CStep::GetManualVisionResult()
{
	Point_XYRZ VisionResult;
	VisionResult = m_ManualVisionResult;
	return VisionResult;
}

long CStep::GetRunStepNo(long insertNo)
{
	long RunStepNo = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		RunStepNo = m_RunStepNo[insertNo - 1];
	}
	return RunStepNo;
}

long CStep::GetUseFromInsertNo(long insertNo)
{
	long UseInsertNo = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		UseInsertNo = m_InsertUse[insertNo - 1];
	}
	return UseInsertNo;
}

long CStep::GetFeederUse(long FeederNo)
{
	long UseFeeder = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		UseFeeder = m_FeederUse[FeederNo - 1];
	}
	return UseFeeder;
}

long CStep::GetPickOrderFromInsertNo(long insertNo)
{
	long PickOrder = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		PickOrder = m_PickOrder[insertNo - 1];
	}
	return PickOrder;
}

long CStep::GetInsertOrderFromInsertNo(long insertNo)
{
	long InsertOrder = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		InsertOrder = m_InsertOrder[insertNo - 1];
	}
	return InsertOrder;
}

long CStep::GetHeadNoFromInsertNo(long insertNo)
{
	long InsertHeadNoFromInsertNo = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		InsertHeadNoFromInsertNo = m_InsertHeadNoFromInsertNo[insertNo - 1];
	}
	return InsertHeadNoFromInsertNo;
}

long CStep::GetFeederNoFromInsertNo(long insertNo)
{
	long InsertFeederNoFromInsertNo = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		InsertFeederNoFromInsertNo = m_InsertFeederNoFromInsertNo[insertNo - 1];
	}
	return InsertFeederNoFromInsertNo;
}

long CStep::GetInsertNoFromInsertOrder(long insertOrd)
{
	long InsertNo = 0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		InsertNo = m_InsertNo[insertOrd - 1];
	}
	return InsertNo;
}

long CStep::GetReadyNoFromFeederNo(long FeederNo)
{
	long ReadyNo = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReadyNo = m_ReadyNo[FeederNo - 1];
	}
	return ReadyNo;
}

long CStep::GetReleaseNoFromFeederNo(long FeederNo)
{
	long ReleaseNo = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReleaseNo = m_ReleaseNo[FeederNo - 1];
	}
	return ReleaseNo;
}

long CStep::GetReadyTimeOutFromFeederNo(long FeederNo)
{
	long ReadyTimeOut = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReadyTimeOut = m_ReadyTimeOut[FeederNo - 1];
	}
	return ReadyTimeOut;
}

long CStep::GetReadyWaitDelayFromFeederNo(long FeederNo)
{
	long ReadyWaitDelay = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReadyWaitDelay = m_ReadyWaitDelay[FeederNo - 1];
	}
	return ReadyWaitDelay;
}

long CStep::GetDiscardMethod(long FeederNo)
{
	long Method = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Method = m_DiscardMethod[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetDiscardMethod FeederNo:%d Method:%d\n"), FeederNo, Method);
		}
	}
	return Method;
}

Point_XYRZ CStep::GetDiscardPoint(long FeederNo)
{
	Point_XYRZ Point;
	ZeroMemory(&Point, sizeof(Point));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Point = m_DiscardPoint[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetDiscardPoint FeederNo:%d X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), FeederNo, Point.x, Point.y, Point.r, Point.z);
		}
	}
	return Point;
}

long CStep::GetRecognitionTable(long insertNo)
{
	long RecogTable = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		RecogTable = m_RecogTable[insertNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetRecognitionTable insertNo:%d RecogTable:%d\n"), insertNo, RecogTable);
		}
	}
	return RecogTable;
}

long CStep::GetPickRetryFromFdNo(long FeederNo)
{
	long PickRetry = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		PickRetry = m_PickRetry[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
			TRACE(_T("[PWR] GetPickRetryFromFdNo FeederNo:%d PickRetry:%d\n"), FeederNo, PickRetry);
	}
	return PickRetry;
}

TwoStepMotion CStep::GetTwoStepPick(long FeederNo)
{
	TwoStepMotion TwoStepPick;
	ZeroMemory(&TwoStepPick, sizeof(TwoStepPick));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		TwoStepPick = m_TwoStepPick[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetTwoStepPick FdNo:%d TwoStepPick Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepPick.Use, TwoStepPick.Dist, TwoStepPick.Ratio);
		}
	}
	return TwoStepPick;
}

TwoStepMotion CStep::GetTwoStepInsert(long FeederNo)
{
	TwoStepMotion TwoStepInsert;
	ZeroMemory(&TwoStepInsert, sizeof(TwoStepInsert));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		TwoStepInsert = m_TwoStepInsert[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetTwoStepInsert FdNo:%d TwoStepInsert Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepInsert.Use, TwoStepInsert.Dist, TwoStepInsert.Ratio);
		}
	}
	return TwoStepInsert;
}

double CStep::GetInsertZOffset(long FeederNo)
{
	double InsertZOffset = 0.0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		InsertZOffset = m_InsertZOffset[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetInsertZOffset FeederNo:%d InsertZOffset:%.3f\n"), FeederNo, InsertZOffset);
		}
	}
	return InsertZOffset;
}

double CStep::GetVAOffsetHeight(long FeederNo)
{
	double VAOffsetHeight = 0.0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		VAOffsetHeight = m_VARecognitionOffsetHeight[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetVAOffsetHeight FeederNo:%d VAOffsetHeight:%.3f\n"), FeederNo, VAOffsetHeight);
		}
	}
	return VAOffsetHeight;
}

long CStep::GetMaxInsertCount()
{
	long MaxInsertCount = 0;
	MaxInsertCount = m_MaxInsertCount;
	return MaxInsertCount;
}

double CStep::GetPickupZStandBy()
{
	double PickupZStandBy = 50.0;
	PickupZStandBy = m_PickupZStandBy;
	return PickupZStandBy;
}

double CStep::GetHighSpeedZOffset()
{
	double HighSpeedZOffset = 5.0;
	HighSpeedZOffset = m_HighSpeedOffset;
	return HighSpeedZOffset;
}

long CStep::GetBoardNo()
{
	long BoardNo = 0;
	BoardNo = m_BoardNo;
	return BoardNo;
}

long CStep::GetFeederReadyIOType(long FeederNo)
{
	long ReadyIOType = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReadyIOType = m_FeederReadyIOType[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetFeederReadyIOType FeederNo:%d ReadyIOType:%d\n"), FeederNo, ReadyIOType);
		}
	}
	return ReadyIOType;
}

long CStep::GetAvoidCount(long insertNo)
{
	long AvoidCount = 0;
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		AvoidCount = m_AvoidCount[insertNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetAvoidCount insertNo:%d AvoidCount:%d\n"), insertNo, AvoidCount);
		}
	}
	return AvoidCount;
}

long CStep::SetNozzleNoFromInsertNo(long insertNo, long NozzleNo)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_InsertNozzleNoFromInsertNo[insertNo - 1] = NozzleNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFeederNoLastUsed insertNo:%d InsertFeederNo:%d\n"), insertNo, NozzleNo);
		}
	}
	return NO_ERR;
}

long CStep::GetNozzleNoFromInsertNo(long insertNo)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		return m_InsertNozzleNoFromInsertNo[insertNo - 1];
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
long CStep::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		ReadyTimeOutEmpty = m_ReadyTimeOutEmptyByHeadNo[HeadNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
			TRACE(_T("[PWR] GetReadyTimeOutEmptyByHeadNo HeadNo:%d Empty:%d\n"), HeadNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CStep::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		ReadyTimeOutEmpty = m_ReadyTimeOutEmpty[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
			TRACE(_T("[PWR] GetReadyTimeOutEmpty FeederNo:%d Empty:%d\n"), FeederNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CStep::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		VisionErrorEmpty = m_VisionErrorEmpty[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
			TRACE(_T("[PWR] GetVisionErrorEmpty FeederNo:%d VisionErrorEmpty:%d\n"), FeederNo, VisionErrorEmpty);
	}
	return VisionErrorEmpty;
}

long CStep::GetVisionError(long insertOrd)
{
	long VisionError = 0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		VisionError = m_VisionError[insertOrd - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetVisionError insertOrd:%d VisionError:%d\n"), insertOrd, VisionError);
		}
	}
	return VisionError;
}

long CStep::GetEmptyError(long insertOrd)
{
	long EmptyError = 0;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		EmptyError = m_EmptyError[insertOrd - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetEmptyError insertOrd:%d EmptyError:%d\n"), insertOrd, EmptyError);
		}
	}
	return EmptyError;
}

bool CStep::GetSendEmpty(long FeederNo)
{
	bool bSendEmpty = false;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		bSendEmpty = m_bSendEmpty[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetSendEmpty FeederNo:%d bSendEmpty:%d\n"), FeederNo, bSendEmpty);
		}
	}
	return bSendEmpty;
}

User_PickOrder CStep::GetUserPickOrderFromIndex(long index)
{	
	User_PickOrder UserPickOrder;
	UserPickOrder.InsertCurNo = 0;
	UserPickOrder.PickOrder = 99;
	if (index > 0 && index <= MAXUSEDHEADNO)
	{
		UserPickOrder = m_UserPickOrder[index - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetUserPickOrderFromIndex index:%d UserPickOrder:%d InsertNo:%d\n"), index, UserPickOrder.PickOrder, UserPickOrder.InsertCurNo);
		}
	}
	return UserPickOrder;
}

User_InsertOrder CStep::GetUserInsertOrderFromIndex(long index)
{
	User_InsertOrder UserInsertOrder;
	UserInsertOrder.InsertCurNo = 0;
	UserInsertOrder.InsertOrder = 99;
	if (index > 0 && index <= MAXUSEDHEADNO)
	{
		UserInsertOrder = m_UserInsertOrder[index - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetUserInsertOrderFromIndex index:%d UserInsertOrder:%d InsertNo:%d\n"), index, UserInsertOrder.InsertOrder, UserInsertOrder.InsertCurNo);
		}
	}
	return UserInsertOrder;
}

bool CStep::GetRefill(long FeederNo)
{
	bool bRefill = false;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		bRefill = m_bFeederRefill[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetRefill FeederNo:%d m_bFeederRefill:%s\n"), FeederNo, bRefill == true ? _T("Refill") : _T("Yet"));
		}
	}
	return bRefill;
}

bool CStep::GetAllVisionError()
{
	long FeederNo = 0;
	long VisionError = 0;
	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		if (GetVisionError(InsertOrd + 1) == 1)
		{
			VisionError++;
		}
	}
	if (GetMaxInsertOrder() > 0)
	{
		if (VisionError == GetMaxInsertOrder())
		{
			return true;
		}
	}
	return false;
}

bool CStep::GetAllEmptyError()
{
	long FeederNo = 0;
	long EmptyError = 0;
	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		if (GetEmptyError(InsertOrd + 1) == 1)
		{
			EmptyError++;
		}
	}
	if (GetMaxInsertOrder() > 0)
	{
		if (EmptyError == GetMaxInsertOrder())
		{
			return true;
		}
	}
	return false;
}


double CStep::GetMaxComponentHeight()
{
	double MaxComponentHeight = 10.0;
	MaxComponentHeight = m_MaxComponentHeight;
	return MaxComponentHeight;
}


long CStep::GetSimultaneousLoading()
{
	long SimultaneousLoading = 0;
	SimultaneousLoading = m_SimultaneousLoading;
	return SimultaneousLoading;
}


void CStep::ClearVisionError()
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] All InsertOrd ClearVisionError Start\n"));
	}
	for (long InsertOrd = 0; InsertOrd < MAXUSEDHEADNO; ++InsertOrd)
	{
		m_VisionError[InsertOrd] = 0;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] All InsertOrd ClearVisionError End\n"));
	}
}

void CStep::ClearEmptyError()
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] All InsertOrd ClearEmptyError Start\n"));
	}
	for (long InsertOrd = 0; InsertOrd < MAXUSEDHEADNO; ++InsertOrd)
	{
		m_EmptyError[InsertOrd] = 0;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] All InsertOrd ClearEmptyError End\n"));
	}
}

/////////////////////////////////////////////////////////////////////////////////

long CStep::SetPickupHeadNo(long PickOrd, long PickupHeadNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetPickupHeadNo PickOrd:%d PickupHeadNo:%d\n"), PickOrd, PickupHeadNo);
	}
	if (PickOrd > 0 && PickOrd <= MAXUSEDHEADNO)
	{
		m_PickHeadNo[PickOrd - 1] = PickupHeadNo;
	}
	return NO_ERR;
}

void CStep::SetMaxPickOrder(long MaxPickOrd)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetMaxPickOrder MaxPickOrd:%d\n"), MaxPickOrd);
	}
	m_MaxPickOrder = MaxPickOrd;
}

long CStep::SetFdNoFromPickOrder(long PickOrd, long FeederNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetFdNoFromPickOrder PickOrd:%d FeederNo:%d\n"), PickOrd, FeederNo);
	}
	if (PickOrd > 0 && PickOrd <= MAXUSEDHEADNO)
	{
		m_PickFeederNo[PickOrd - 1] = FeederNo;
	}
	return NO_ERR;
}

long CStep::SetPickDelayFromFdNo(long FeederNo, long PickDelay)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetPickDelayFromFdNo FeederNo:%d PickDelay:%d\n"), FeederNo, PickDelay);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PickupDelayTime[FeederNo - 1] = PickDelay;
	}
	return NO_ERR;
}

long CStep::SetRatioFromFdNo(long FeederNo, Ratio_XYRZ Ratio)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetRatioFromFdNo FeederNo:%d Ratio_XYRZ:%.1f,%.1f,%.1f\n"), FeederNo, Ratio.xy, Ratio.r, Ratio.z);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_Ratio[FeederNo - 1] = Ratio;
	}
	return NO_ERR;
}

long CStep::SetPickOffsetFromFdNo(long FeederNo, Point_XYRZ pt)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetPickOffsetFromFdNo FeederNo:%d Point_XYRZ:%.3f,%.3f,%.3f,%.3f\n"), FeederNo, pt.x, pt.y, pt.r, pt.z);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PickOffset[FeederNo - 1] = pt;
	}
	return NO_ERR;
}

long CStep::SetFdNoFromInsertOrder(long insertOrd, long FeederNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetFdNoFromInsertOrder insertOrd:%d FeederNo:%d\n"), insertOrd, FeederNo);
	}
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_InsertFeederNo[insertOrd - 1] = FeederNo;
	}
	return NO_ERR;
}

long CStep::SetLed(long FeederNo, MODULE_LED Led)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetLed FeederNo:%d Led Top,Mid,Bot(%d,%d,%d)\n"), FeederNo, Led.Top, Led.Mid, Led.Bot);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_Led[FeederNo - 1] = Led;
	}
	return NO_ERR;
}

long CStep::SetMaxInsertOrder(long MaxInsertOrd)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetMaxInsertOrder MaxInsertOrd:%d\n"), MaxInsertOrd);
	}
	m_MaxInsertOrder = MaxInsertOrd;
	return NO_ERR;
}

long CStep::SetHeadNoFromInsertOrder(long insertOrd, long HeadNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetHeadNoFromInsertOrder insertOrd:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_InsertHeadNo[insertOrd - 1] = HeadNo;
	}
	return NO_ERR;
}

long CStep::SetVAAngleFromFdNo(long FeederNo, double VAAngle)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetVAAngleFromFdNo FeederNo:%d VAAngle:%.1f\n"), FeederNo, VAAngle);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_VAAngle[FeederNo - 1] = VAAngle;
	}
	return NO_ERR;
}

long CStep::SetComponentHeightFromFdNo(long FeederNo, double Height)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetComponentHeightFromFdNo FeederNo:%d Height:%.1f\n"), FeederNo, Height);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ComponentHeight[FeederNo - 1] = Height;
	}
	return NO_ERR;
}

long CStep::SetComponentLeadHeightFromFdNo(long FeederNo, double LeadHeight)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetComponentLeadHeightFromFdNo FeederNo:%d LeadHeight:%.1f\n"), FeederNo, LeadHeight);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ComponentLeadHeight[FeederNo - 1] = LeadHeight;
	}
	return NO_ERR;
}

long CStep::SetInsertPoint(long insertNo, Point_XYRZ ptInsert)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_Insert[insertNo - 1] = ptInsert;
		if (m_InsertUse[insertNo - 1] > 0)
		{
			TRACE(_T("[PWR] SetInsertPoint insertNo:%d ptInsertXYRZ:%.3f,%.3f,%.3f,%.3f\n"), insertNo, ptInsert.x, ptInsert.y, ptInsert.r, ptInsert.z);
		}
	}
	return NO_ERR;
}

long CStep::SetBlowDelayFromFdNo(long FeederNo, long BlowDelay)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetBlowDelayFromFdNo FeederNo:%d BlowDelay:%d\n"), FeederNo, BlowDelay);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_InsertBlowDelayTime[FeederNo - 1] = BlowDelay;
	}
	return NO_ERR;
}

long CStep::SetReleaseDelayFromFdNo(long FeederNo, long ReleaseDelay)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetReleaseDelayFromFdNo FdNo:%d ReleaseDelay:%d\n"), FeederNo, ReleaseDelay);
	}
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_InsertReleaseDelayTime[FeederNo - 1] = ReleaseDelay;
	}
	return NO_ERR;
}

long CStep::SetOrigin(ORIGIN Origin) 
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetOrigin OriginXY,%.3f,%.3f\n"), Origin.pt.x, Origin.pt.y);
	}
	m_Origin = Origin;
	return NO_ERR;
}

long CStep::SetReturnComponentHeadNo(long ReturnOrd, long ReturnHeadNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetReturnComponentHeadNo ReturnOrd:%d ReturnHeadNo:%d\n"), ReturnOrd, ReturnHeadNo);
	}
	if (ReturnOrd > 0 && ReturnOrd <= MAXUSEDHEADNO)
	{
		m_ReturnComponentHeadNo[ReturnOrd - 1] = ReturnHeadNo;
	}
	return NO_ERR;
}

long CStep::SetMaxReturnComponentOrder(long MaxReturnOrd)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetMaxReturnComponentOrder MaxReturnOrd:%d\n"), MaxReturnOrd);
	}
	m_MaxReturnComponentOrder = MaxReturnOrd;
	return NO_ERR;
}

long CStep::SetFdNoFromReturnOrder(long ReturnOrd, long FeederNo)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetFdNoFromReturnOrder ReturnOrd:%d FeederNo:%d\n"), ReturnOrd, FeederNo);
	}
	if (ReturnOrd > 0 && ReturnOrd <= MAXUSEDHEADNO)
	{
		m_ReturnComponentFeederNo[ReturnOrd - 1] = FeederNo;
	}
	return NO_ERR;
}

long CStep::SetManualCompensationUse(long ManualCompenUse)
{
	m_ManualCompensationUse = ManualCompenUse;
	return NO_ERR;
}

long CStep::SetManualVisionResult(Point_XYRZ VisionResult)
{
	m_ManualVisionResult = VisionResult;
	return NO_ERR;
}

long CStep::SetRunStepNo(long insertNo, long RunStepNo)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_RunStepNo[insertNo - 1] = RunStepNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetRunStepNo insertNo:%d RunStepNo:%d\n"), insertNo, RunStepNo);
		}
	}	
	return NO_ERR;
}

long CStep::SetUseFromInsertNo(long insertNo, long Use)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_InsertUse[insertNo - 1] = Use;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetUseFromInsertNo insertNo:%d Use:%d\n"), insertNo, Use);
		}
	}	
	return NO_ERR;
}

long CStep::SetFeederUse(long FeederNo, long Use)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_FeederUse[FeederNo - 1] = Use;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFeederUse FeederNo:%d Use:%d\n"), FeederNo, Use);
		}
	}
	return NO_ERR;
}

long CStep::SetPickOrderFromInsertNo(long insertNo, long PickOrder)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_PickOrder[insertNo - 1] = PickOrder;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetPickOrderFromInsertNo insertNo:%d PickOrder:%d\n"), insertNo, PickOrder);
		}
	}
	return NO_ERR;
}

long CStep::SetInsertOrderFromInsertNo(long insertNo, long InsertOrder)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_InsertOrder[insertNo - 1] = InsertOrder;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetInsertOrderFromInsertNo insertNo:%d InsertOrder:%d\n"), insertNo, InsertOrder);
		}
	}
	return NO_ERR;
}

long CStep::SetHeadNoFromInsertNo(long insertNo, long InsertHeadNo)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_InsertHeadNoFromInsertNo[insertNo - 1] = InsertHeadNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetHeadNoFromInsertNo insertNo:%d InsertHeadNo:%d\n"), insertNo, InsertHeadNo);
		}
	}
	return NO_ERR;
}

long CStep::SetFeederNoFromInsertNo(long insertNo, long InsertFeederNo)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_InsertFeederNoFromInsertNo[insertNo - 1] = InsertFeederNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetHeadNoFromInsertNo insertNo:%d InsertFeederNo:%d\n"), insertNo, InsertFeederNo);
		}
	}
	return NO_ERR;
}

long CStep::SetInsertNoFromInsertOrder(long insertOrd, long insertNo)
{
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_InsertNo[insertOrd - 1] = insertNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetInsertNoFromInsertOrder insertOrd:%d insertNo:%d\n"), insertOrd, insertNo);
		}
	}
	return NO_ERR;
}

long CStep::SetReadyNoFromFeederNo(long FeederNo, long ReadyNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReadyNo[FeederNo - 1] = ReadyNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReadyNoFromFeederNo FeederNo:%d ReadyNo:%d\n"), FeederNo, ReadyNo);
		}
	}
	return NO_ERR;
}

long CStep::SetReleaseNoFromFeederNo(long FeederNo, long ReleaseNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReleaseNo[FeederNo - 1] = ReleaseNo;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReleaseNoFromFeederNo FeederNo:%d ReleaseNo:%d\n"), FeederNo, ReleaseNo);
		}
	}
	return NO_ERR;
}

long CStep::SetReadyTimeOutFromFeederNo(long FeederNo, long TimeOut)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReadyTimeOut[FeederNo - 1] = TimeOut;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReadyTimeOutFromFeederNo FeederNo:%d TimeOut:%d\n"), FeederNo, TimeOut);
		}
	}
	return NO_ERR;
}

long CStep::SetReadyWaitDelayFromFeederNo(long FeederNo, long ReadyWaitDelay)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReadyWaitDelay[FeederNo - 1] = ReadyWaitDelay;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReadyWaitDelayFromFeederNo FeederNo:%d ReadyWaitDelay:%d\n"), FeederNo, ReadyWaitDelay);
		}
	}
	return NO_ERR;
}

long CStep::SetDiscardMethod(long FeederNo, long Method)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_DiscardMethod[FeederNo - 1] = Method;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetDiscardMethod FeederNo:%d Method:%d\n"), FeederNo, Method);
		}
	}
	return NO_ERR;
}

long CStep::SetDiscardPoint(long FeederNo, Point_XYRZ Point)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_DiscardPoint[FeederNo - 1] = Point;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetDiscardPoint FeederNo:%d X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), FeederNo, Point.x, Point.y, Point.r, Point.z);
		}
	}
	return NO_ERR;
}

long CStep::SetRecognitionTable(long insertNo, long RecogTable)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_RecogTable[insertNo - 1] = RecogTable;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetRecognitionTable InsertNo:%d RecogTable:%d\n"), insertNo, RecogTable);
		}
	}
	return NO_ERR;
}

long CStep::SetPickRetryFromFdNo(long FeederNo, long PickRetry)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PickRetry[FeederNo - 1] = PickRetry;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetPickRetryFromFdNo FeederNo:%d PickRetry:%d\n"), FeederNo, PickRetry);
		}
	}
	return NO_ERR;
}

long CStep::SetTwoStepPick(long FeederNo, TwoStepMotion TwoStepPick)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_TwoStepPick[FeederNo - 1] = TwoStepPick;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTwoStepPick FeederNo:%d TwoStepPick Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepPick.Use, TwoStepPick.Dist, TwoStepPick.Ratio);
		}
	}
	return NO_ERR;
}

long CStep::SetTwoStepInsert(long FeederNo, TwoStepMotion TwoStepInsert)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_TwoStepInsert[FeederNo - 1] = TwoStepInsert;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTwoStepInsert FeederNo:%d TwoStepInsert Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepInsert.Use, TwoStepInsert.Dist, TwoStepInsert.Ratio);
		}
	}
	return NO_ERR;
}

long CStep::SetInsertZOffset(long FeederNo, double InsertZOffset)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_InsertZOffset[FeederNo - 1] = InsertZOffset;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetInsertZOffset FeederNo:%d InsertZOffset:%.3f\n"), FeederNo, InsertZOffset);
		}
	}
	return NO_ERR;
}

long CStep::SetVAOffsetHeight(long FeederNo, double VAOffsetHeight)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_VARecognitionOffsetHeight[FeederNo - 1] = VAOffsetHeight;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetVAOffsetHeight FeederNo:%d VAOffsetHeight:%.3f\n"), FeederNo, VAOffsetHeight);
		}
	}
	return NO_ERR;
}

long CStep::SetFeederReadyIOType(long FeederNo, long ReadyIOType)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_FeederReadyIOType[FeederNo - 1] = ReadyIOType;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFeederReadyIOType FeederNo:%d ReadyIOType:%d\n"), FeederNo, ReadyIOType);
		}
	}
	return NO_ERR;
}

long CStep::SetPartEmptyStopByFeederNo(long FeederNo, long PartEmptyStop)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PartEmptyStop[FeederNo - 1] = PartEmptyStop;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFeederPartEmptyStop FeederNo:%d PartEmptyStop:%d\n"), FeederNo, PartEmptyStop);
		}
	}
	return NO_ERR;
}

long CStep::GetPartEmptyStopFromFeederNo(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		return m_PartEmptyStop[FeederNo - 1];
	}
	return NO_ERR;
}

long CStep::SetMaxInsertCount(long MaxInsertCount)
{
	m_MaxInsertCount = MaxInsertCount;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetMaxInsertCount MaxInsertCount:%d\n"), MaxInsertCount);
	}
	return NO_ERR;
}

long CStep::SetPickupZStandBy(double PickupZStandBy)
{
	m_PickupZStandBy = PickupZStandBy;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetPickupZStandBy PickupZStandBy:%.3f\n"), PickupZStandBy);
	}
	return NO_ERR;
}

long CStep::SetMaxComponentHeight(double MaxComponentHeight)
{
	m_MaxComponentHeight = MaxComponentHeight;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetMaxComponentHeight MaxComponentHeight:%.3f\n"), MaxComponentHeight);
	}
	return NO_ERR;
}

long CStep::SetSimultaneousLoading(long SimultaneousLoading)
{
	m_SimultaneousLoading = SimultaneousLoading;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetSimultaneousLoading SimultaneousLoading:%d\n"), SimultaneousLoading);
	}
	return NO_ERR;
}

long CStep::SetBoardNo(long BoardNo)
{
	m_BoardNo = BoardNo;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetBoardNo BoardNo:%d\n"), BoardNo);
	}
	return NO_ERR;
}

long CStep::SetAvoidCount(long insertNo, long AvoidCount)
{
	if (insertNo > 0 && insertNo <= MAXINSERTNO)
	{
		m_AvoidCount[insertNo - 1] = AvoidCount;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetAvoidCount insertNo:%d AvoidCount:%d\n"), insertNo - 1, AvoidCount);
		}
	}
	return NO_ERR;
}

long CStep::SetLaserControl(long FeederNo, long LaserControl)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_LaserControl[FeederNo - 1] = LaserControl;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetLaserControl FeederNo:%d LaserControl:%d\n"), FeederNo, LaserControl);
		}
	}
	return NO_ERR;
}

long CStep::SetCatchDelay(long FeederNo, long CatchDelay)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_CatchDelay[FeederNo - 1] = CatchDelay;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetCatchDelay FeederNo:%d CatchDelay:%d\n"), FeederNo, CatchDelay);
		}
	}
	return NO_ERR;
}

long CStep::SetRetryLed(long FeederNo, RETRY_LED RetryLed)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_RetryLed[FeederNo - 1] = RetryLed;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetRetryLed FeederNo:%d MaxRetry:%d\n"), FeederNo, RetryLed.MaxRetry);
		}
	}
	return NO_ERR;
}

////////////////////////////////////////////////////////////////////////////////

long CStep::SetReadyTimeOutEmptyByHeadNo(long HeadNo, long ReadyTimeOutEmpty)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		m_ReadyTimeOutEmptyByHeadNo[HeadNo - 1] = ReadyTimeOutEmpty;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReadyTimeOutEmptyByHeadNo HeadNo:%d Empty:%d\n"), HeadNo, ReadyTimeOutEmpty);
		}
	}
	return NO_ERR;
}

long CStep::ClearReadyTimeOutEmpty(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReadyTimeOutEmpty[FeederNo - 1] = 0;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] ClearReadyTimeOutEmpty FeederNo:%d\n"), FeederNo);
		}
	}
	return NO_ERR;
}

long CStep::SetReadyTimeOutEmpty(long FeederNo, long ReadyTimeOutEmpty)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_ReadyTimeOutEmpty[FeederNo - 1] = ReadyTimeOutEmpty;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetReadyTimeOutEmpty FeederNo:%d Empty:%d\n"), FeederNo, ReadyTimeOutEmpty);
		}
	}
	return NO_ERR;
}

long CStep::ClearVisionErrorEmpty(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_VisionErrorEmpty[FeederNo - 1] = 0;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] ClearVisionErrorEmpty FeederNo:%d VisionErrorEmpty:%d\n"), FeederNo, m_VisionErrorEmpty[FeederNo - 1]);
		}
	}
	return NO_ERR;
}

long CStep::SetVisionErrorEmpty(long FeederNo, long VisionErrorEmpty)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_VisionErrorEmpty[FeederNo - 1] += VisionErrorEmpty;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetVisionErrorEmpty FeederNo:%d VisionErrorEmpty:%d\n"), FeederNo, m_VisionErrorEmpty[FeederNo - 1]);
		}
	}
	return NO_ERR;
}

long CStep::SetVisionError(long insertOrd, long VisionError)
{
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_VisionError[insertOrd - 1] = VisionError;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetVisionError InsertOrd:%d VisionError:%d\n"), insertOrd, m_VisionError[insertOrd - 1]);
		}
	}
	return NO_ERR;
}

long CStep::SetEmptyError(long insertOrd, long EmptyError)
{
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_EmptyError[insertOrd - 1] = EmptyError;
		if (gcPowerLog->IsShowFeederLog() == true)
		{
			TRACE(_T("[PWR] SetEmptyError insertOrd:%d EmptyError:%d\n"), insertOrd, EmptyError);
		}
	}
	return NO_ERR;
}

long CStep::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_bSendEmpty[FeederNo - 1] = bSendEmpty;
		if (gcPowerLog->IsShowFeederLog() == true)
		{
			TRACE(_T("[PWR] SetSendEmpty FeederNo:%d bSendEmpty:%d\n"), FeederNo, bSendEmpty);
		}
	}
	return NO_ERR;
}

long CStep::SetUserPickOrderFromIndex(long index, User_PickOrder UserPickOrder)
{
	if (index > 0 && index <= MAXUSEDHEADNO)
	{
		m_UserPickOrder[index - 1] = UserPickOrder;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetUserPickOrderFromIndex index:%d UserPickOrder:%d InsertNo:%d\n"), index, UserPickOrder.PickOrder, UserPickOrder.InsertCurNo);
		}
		m_UserPickOrderCount++;
	}
	return NO_ERR;
}

long CStep::ClearUserPickOrder()
{
	m_UserPickOrderCount = 0;
	for (long index = 0; index < MAXUSEDHEADNO; ++index)
	{
		m_UserPickOrder[index].PickOrder = 99;
	}
	return NO_ERR;
}

long CStep::SortUserPickOrder()
{
	long sort1st, sort2nd, min, buf;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < m_UserPickOrderCount; ++sort1st)
		{
			TRACE(_T("[PWR] Before Sorting Index(%d) UserPickOrder:%d InsertNo:%d\n"), sort1st + 1, m_UserPickOrder[sort1st].PickOrder, m_UserPickOrder[sort1st].InsertCurNo);
		}
	}
	for (sort1st = 0; sort1st < m_UserPickOrderCount - 1; ++sort1st)
	{
		min = sort1st;
		for (sort2nd = sort1st + 1; sort2nd < m_UserPickOrderCount; ++sort2nd)
		{
			if (m_UserPickOrder[sort2nd].PickOrder < m_UserPickOrder[sort1st].PickOrder)
			{
				//min = sort2nd;
				SWAP(m_UserPickOrder[sort1st].PickOrder, m_UserPickOrder[sort2nd].PickOrder, buf);
				SWAP(m_UserPickOrder[sort1st].InsertCurNo, m_UserPickOrder[sort2nd].InsertCurNo, buf);
			}
		}
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < m_UserPickOrderCount; ++sort1st)
		{
			TRACE(_T("[PWR] After  Sorting Index(%d) UserPickOrder:%d InsertNo:%d\n"), sort1st + 1, m_UserPickOrder[sort1st].PickOrder, m_UserPickOrder[sort1st].InsertCurNo);
		}
	}
	return NO_ERR;
}


long CStep::SetUserInsertOrderFromIndex(long index, User_InsertOrder UserInsertOrder)
{
	if (index > 0 && index <= MAXUSEDHEADNO)
	{
		m_UserInsertOrder[index - 1] = UserInsertOrder;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetUserInsertOrderFromIndex index:%d UserInsertOrder:%d InsertNo:%d\n"), index, UserInsertOrder.InsertOrder, UserInsertOrder.InsertCurNo);
		}
		m_UserInsertOrderCount++;
	}
	return NO_ERR;
}

long CStep::ClearUserInsertOrder()
{
	m_UserInsertOrderCount = 0;
	for (long index = 0; index < MAXUSEDHEADNO; ++index)
	{
		m_UserInsertOrder[index].InsertOrder = 99;
	}
	return NO_ERR;
}

long CStep::SortUserInsertOrder()
{
	long sort1st, sort2nd, min, buf;
	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < m_UserInsertOrderCount; ++sort1st)
		{
			TRACE(_T("[PWR] Before Sorting Index(%d) UserInsertOrder:%d InsertNo:%d\n"), sort1st + 1, m_UserInsertOrder[sort1st].InsertOrder, m_UserInsertOrder[sort1st].InsertCurNo);
		}
	}
	for (sort1st = 0; sort1st < m_UserInsertOrderCount - 1; ++sort1st)
	{
		min = sort1st;
		for (sort2nd = sort1st + 1; sort2nd < m_UserInsertOrderCount; ++sort2nd)
		{
			if (m_UserInsertOrder[sort2nd].InsertOrder < m_UserInsertOrder[sort1st].InsertOrder)
			{
				//min = sort2nd;
				SWAP(m_UserInsertOrder[sort1st].InsertOrder, m_UserInsertOrder[sort2nd].InsertOrder, buf);
				SWAP(m_UserInsertOrder[sort1st].InsertCurNo, m_UserInsertOrder[sort2nd].InsertCurNo, buf);
			}
		}
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		for (sort1st = 0; sort1st < m_UserInsertOrderCount; ++sort1st)
		{
			TRACE(_T("[PWR] After  Sorting Index(%d) UserInsertOrder:%d InsertNo:%d\n"), sort1st + 1, m_UserInsertOrder[sort1st].InsertOrder, m_UserInsertOrder[sort1st].InsertCurNo);
		}
	}
	return NO_ERR;
}

long CStep::ClearSendEmpty(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_bSendEmpty[FeederNo - 1] = false;
		if (gcPowerLog->IsShowFeederLog() == true)
		{
			TRACE(_T("[PWR] ClearSendEmpty FeederNo:%d\n"), FeederNo);
		}
	}
	return NO_ERR;
}

long CStep::SetRefill(long FeederNo, bool bRefill)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		RunStepLock();
		m_bFeederRefill[FeederNo - 1] = bRefill;
		RunStepUnLock();
		TRACE(_T("[PWR] SetRefill FeederNo:%d FeederRefill:%d\n"), FeederNo, m_bFeederRefill[FeederNo - 1]);
	}
	return NO_ERR;
}

long CStep::ClearRefill(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_bFeederRefill[FeederNo - 1] = false;
		if (gcPowerLog->IsShowFeederLog() == true)
		{
			TRACE(_T("[PWR] ClearRefill FeederNo:%d\n"), FeederNo);
		}
	}
	return NO_ERR;
}

long CStep::RefillDone()
{
	long RefillCount = 0;
	RunStepLock();
	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
	{
		if (GetFeederUse(FeederNo + 1) == 0) continue;
		if (GetSendEmpty(FeederNo + 1) == true && GetRefill(FeederNo + 1) == true)
		{
			ClearVisionErrorEmpty(FeederNo + 1);
			ClearReadyTimeOutEmpty(FeederNo + 1);
			SetSendEmpty(FeederNo + 1, false);
			SetFeederEmptyDisplay(FeederNo + 1, false);
			SetRefill(FeederNo + 1, false);
			SendFeederAutoRefill(FeederNo + 1);
			RefillCount++;
		}
	}
	RunStepUnLock();
	TRACE(_T("[PWR] RefillDone Feeder Count:%d\n"), RefillCount);
	return NO_ERR;
}

long CStep::GetRecogTableBy1stInsertOrder()
{
	long insertNo, RecognitionTableFirst, RecognitionTable;

	RecognitionTableFirst = RecognitionTable = FRONT_STAGE;

	for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
	{
		insertNo = GetInsertNoFromInsertOrder(InsertOrd + 1);

		if (InsertOrd == 0)
		{
			RecognitionTableFirst = GetRecognitionTable(insertNo);
		}

		RecognitionTable = GetRecognitionTable(insertNo);

		if (RecognitionTableFirst != RecognitionTable)
		{
			TRACE(_T("[PWR] --------------------------"));
			TRACE(_T("[PWR] Invalid Recognition Table. First:%d Diffrent InsertNo:%d"), RecognitionTableFirst, insertNo);

			return -1;
		}		
	}

	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetRecogTableBy1stInsertOrder Table:%d \n"), RecognitionTableFirst);
	}

	return RecognitionTableFirst;
}

// HarkDo 20210127-1
long CStep::SetMinRatio(Ratio_XYRZ MinRatio)
{
	m_MinStepRatio = MinRatio;
	return NO_ERR;
}

// HarkDo 20210127-1
Ratio_XYRZ CStep::GetMinRatio()
{
	return m_MinStepRatio;
}

// HarkDo 20210127-1
long CStep::SetPickRatio(long PickOrder, Ratio_XYRZ Ratio)
{
	if (PickOrder > 0 && PickOrder <= MAXUSEDHEADNO)
	{
		m_PickRatio[PickOrder - 1] = Ratio;
	}
	return NO_ERR;
}

// HarkDo 20210127-1
long CStep::SetInsertRatio(long InsertOrder, Ratio_XYRZ Ratio)
{
	if (InsertOrder > 0 && InsertOrder <= MAXUSEDHEADNO)
	{
		m_InsertRatio[InsertOrder - 1] = Ratio;
	}
	return NO_ERR;
}

// HarkDo 20210127-1
Ratio_XYRZ CStep::GetPickRatio(long PickOrder)
{
	Ratio_XYRZ Ratio;
	Ratio.xy = Ratio.r = Ratio.z = 0.5;
	if (PickOrder > 0 && PickOrder <= MAXUSEDHEADNO)
	{
		Ratio = m_PickRatio[PickOrder - 1];
	}
	return Ratio;
}

// HarkDo 20210127-1
Ratio_XYRZ CStep::GetInsertRatio(long InsertOrder)
{
	Ratio_XYRZ Ratio;
	Ratio.xy = Ratio.r = Ratio.z = 0.5;
	if (InsertOrder > 0 && InsertOrder <= MAXUSEDHEADNO)
	{
		Ratio = m_InsertRatio[InsertOrder - 1];
	}
	return Ratio;
}

void CStep::SetANCPrepare(long Gantry, long HeadNo, long NozzleNo)
{
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_ANCPrepareFront[HeadNo - 1] = NozzleNo;			
		}
		else
		{
			m_ANCPrepareRear[HeadNo - 1] = NozzleNo;
		}
		TRACE(_T("[PWR] SetANCPrepare Gantry:%d Head:%d Nozzle:%d"), Gantry, HeadNo, NozzleNo);
	}

}

long CStep::GetANCPrepare(long Gantry, long HeadNo)
{
	long NozzleNo = 0;
	if (HeadNo > 0 && HeadNo <= MAXUSEDHEADNO)
	{
		if (Gantry == FRONT_GANTRY)
		{
			NozzleNo = m_ANCPrepareFront[HeadNo - 1];
		}
		else
		{
			NozzleNo = m_ANCPrepareRear[HeadNo - 1];
		}
	}

	return NozzleNo;
}

long CStep::GetLaserControl(long FeederNo)
{
	long LaserControl = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		LaserControl = m_LaserControl[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetLaserControl FeederNo:%d LaserControl:%d\n"), FeederNo, LaserControl);
		}
	}
	return LaserControl;
}

long CStep::GetCatchDelay(long FeederNo)
{
	long CatchDelay = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		CatchDelay = m_CatchDelay[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetCatchDelay FeederNo:%d CatchDelay:%d\n"), FeederNo, CatchDelay);
		}
	}
	return CatchDelay;
}

RETRY_LED CStep::GetRetryLed(long FeederNo)
{
	RETRY_LED RetryLed;
	RetryLed.MaxRetry = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		RetryLed = m_RetryLed[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetRetryLed FeederNo:%d MaxRetry:%d\n"), FeederNo, RetryLed.MaxRetry);
		}
	}
	return RetryLed;
}

long CStep::SetFeederType(long FeederNo, long Type)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_FeederType[FeederNo - 1] = Type;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFeederType FeederNo:%d Type:%d\n"), FeederNo, Type);
		}
	}
	return NO_ERR;
}

long CStep::GetFeederType(long FeederNo)
{
	long FeederType = 0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		FeederType = m_FeederType[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetFeederType FeederNo:%d Type:%d\n"), FeederNo, FeederType);
		}
	}
	return FeederType;
}

long CStep::SetTray(long FeederNo, TRAY_INFO Tray)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_Tray[FeederNo - 1] = Tray;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTray FeederNo:%d TrayName:%s\n"), FeederNo, Tray.Name);
		}
	}
	return NO_ERR;
}

TRAY_INFO CStep::GetTray(long FeederNo)
{
	TRAY_INFO Tray;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Tray = m_Tray[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetTray FeederNo:%d TrayName:%s\n"), FeederNo, Tray.Name);
		}
	}
	return Tray;
}

long CStep::SetTrayNowPocket(long FeederNo, long NowPocket)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_TrayNowPocket[FeederNo - 1] = NowPocket;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTrayNowPocket FeederNo:%d Now:%d\n"), FeederNo, NowPocket);
		}
		//SendTrayLastPocket(FeederNo, NowPocket);
	}
	return NO_ERR;
}

long CStep::GetTrayNowPocket(long FeederNo)
{
	long TrayNowPocket = 1;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		TrayNowPocket = m_TrayNowPocket[FeederNo - 1];
	}
	return TrayNowPocket;
}

long CStep::SetTrayMaxPocket(long FeederNo, long MaxPocket)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_TrayMaxPocket[FeederNo - 1] = MaxPocket;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTrayMaxPocket FeederNo:%d Now:%d\n"), FeederNo, MaxPocket);
		}
	}
	return NO_ERR;
}

long CStep::GetTrayMaxPocket(long FeederNo)
{
	long TrayMaxPocket = 1;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		TrayMaxPocket = m_TrayMaxPocket[FeederNo - 1];
	}
	return TrayMaxPocket;
}

long CStep::SetPickLevelCheck(long FeederNo, PICK_LEVEL_CHECK data)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PickLevelCheck[FeederNo - 1] = data;
	}

	return 0;
}

PICK_LEVEL_CHECK CStep::GetPickLevelCheck(long FeederNo)
{
	PICK_LEVEL_CHECK temp;
	ZeroMemory(&temp, sizeof(temp));

	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		temp = m_PickLevelCheck[FeederNo - 1];
	}

	return temp;
}

long CStep::SetTwoStepInsertUp(long FeederNo, TwoStepMotion TwoStepInsertUp)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_TwoStepInsertUp[FeederNo - 1] = TwoStepInsertUp;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetTwoStepInsertUp FeederNo:%d Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepInsertUp.Use, TwoStepInsertUp.Dist, TwoStepInsertUp.Ratio);
		}
	}
	return NO_ERR;
}

TwoStepMotion CStep::GetTwoStepInsertUp(long FeederNo)
{
	TwoStepMotion TwoStepInsertUp;
	ZeroMemory(&TwoStepInsertUp, sizeof(TwoStepInsertUp));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		TwoStepInsertUp = m_TwoStepInsertUp[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetTwoStepInsertUp FdNo:%d Use:%d Dist:%.3f Ratio:%.1f\n"), FeederNo, TwoStepInsertUp.Use, TwoStepInsertUp.Dist, TwoStepInsertUp.Ratio);
		}
	}
	return TwoStepInsertUp;
}


void CStep::SetFeederEmptyDisplay(long FeederNo, bool set)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		if (m_bFeederEmptyDisplay[FeederNo - 1] != set)
		{
			TRACE(_T("[PWR] SetFeederEmptyDisplay FeederNo:%d Set:%d\n"), FeederNo, set);
		}

		m_bFeederEmptyDisplay[FeederNo - 1] = set;
	}
}

bool CStep::GetFeederEmptyDisplay(long FeederNo)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		return m_bFeederEmptyDisplay[FeederNo - 1];
	}
	return false;
}

void CStep::InitDivideInspect()
{
	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; FeederNo++)
	{
		m_DivideInspect[FeederNo].Use = false;
		m_DivideInspect[FeederNo].FeederNo = FeederNo + 1;
		m_DivideInspect[FeederNo].PackageName.Format(_T("D%d"), FeederNo + 1);
		m_DivideInspect[FeederNo].Count = 0;

		for (long i = 0; i < DIVIDE_INSPECT_MAX_COUNT; i++)
		{
			m_DivideInspect[FeederNo].Offset[i].x = 0.000;
			m_DivideInspect[FeederNo].Offset[i].y = 0.000;
			m_DivideInspect[FeederNo].Offset[i].r = 0.000;
			m_DivideInspect[FeederNo].Offset[i].z = 0.000;
		}
	}
}

long CStep::SetDivideInspect(long FeederNo, DIVIDE_INSPECT Divide)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_DivideInspect[FeederNo - 1] = Divide;
	}
	return NO_ERR;
}

DIVIDE_INSPECT CStep::GetDivideInspect(long FeederNo)
{
	DIVIDE_INSPECT Divide;

	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Divide = m_DivideInspect[FeederNo - 1];
	}

	return Divide;
}

void CStep::SetPartTorqueLimit(long FeederNo, PARTTORQUELIMIT data)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_PartTorqueLimit[FeederNo - 1] = data;
	}
}

PARTTORQUELIMIT CStep::GetPartTorqueLimit(long FeederNo)
{
	PARTTORQUELIMIT data;

	data.PickDown.Use = data.PickDown2nd.Use = data.InsertDown.Use = data.InsertDown2nd.Use = false;

	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		data = m_PartTorqueLimit[FeederNo - 1];
	}
	return data;
}

BARCODE CStep::GetBarcode()
{
	return m_Barcode;
}

long CStep::SetBarcode(BARCODE Barcode)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetBarcode XY,%.3f,%.3f Mes.Use:%d\n"), Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Mes.Use);
	}
	m_Barcode = Barcode;
	return NO_ERR;
}

long CStep::SetBarcodeInfoBlock1(BARCODE Barcode)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetBarcodeBlock1 XY,%.3f,%.3f\n"), Barcode.pt[0].x, Barcode.pt[0].y);
	}
	m_BarcodeBlock1 = Barcode;
	return NO_ERR;
}

BARCODE CStep::GetBarcodeInfoBlock1()
{
	return m_BarcodeBlock1;
}

long CStep::SetBarcodeInfoBlock2(BARCODE Barcode)
{
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetBarcodeBlock2 XY,%.3f,%.3f\n"), Barcode.pt[0].x, Barcode.pt[0].y);
	}
	m_BarcodeBlock2 = Barcode;
	return NO_ERR;
}

BARCODE CStep::GetBarcodeInfoBlock2()
{
	return m_BarcodeBlock2;
}

long CStep::InitBarcode()
{
	strBarcodeCarrier.Empty();
	strBarcodeBlock1.Empty();
	strBarcodeBlock2.Empty();
	return NO_ERR;
}

long CStep::SetBarcodeCarrier(CString str)
{
	strBarcodeCarrier.Format(_T("%s"), LPCTSTR(str));
	TRACE(_T("[PWR] SetBarcodeCarrier %s\n"), strBarcodeCarrier);
	return strBarcodeCarrier.GetLength();
}

long CStep::SetBarcodeBlock1(CString str)
{
	strBarcodeBlock1.Format(_T("%s"), LPCTSTR(str));
	TRACE(_T("[PWR] SetBarcodeBlock1 %s\n"), strBarcodeBlock1);
	return strBarcodeBlock1.GetLength();
}

long CStep::SetBarcodeBlock2(CString str)
{
	strBarcodeBlock2.Format(_T("%s"), LPCTSTR(str));
	TRACE(_T("[PWR] SetBarcodeBlock2 %s\n"), strBarcodeBlock2);
	return strBarcodeBlock2.GetLength();
}

CString CStep::GetBarcodeCarrier()
{
	TRACE(_T("[PWR] GetBarcodeCarrier %s\n"), strBarcodeCarrier);
	return strBarcodeCarrier;
}

CString CStep::GetBarcodeBlock1()
{
	TRACE(_T("[PWR] GetBarcodeBlock1 %s\n"), strBarcodeBlock1);
	return strBarcodeBlock1;
}

CString CStep::GetBarcodeBlock2()
{
	TRACE(_T("[PWR] GetBarcodeBlock2 %s\n"), strBarcodeBlock2);
	return strBarcodeBlock2;
}

void CStep::SetBlockNoFromInsertNo(long InsertNo, long BlockNo)
{
	if (InsertNo > 0 && InsertNo <= MAXINSERTNO)
	{
		m_BlockNo[InsertNo - 1] = BlockNo;
	}
}

void CStep::InitBlockSkipHM()
{
	//for (long i = 0; i < MAXBLOCKNO; i++)
	//{
	//	m_BlockSkipHM[i] = BLOCK_PROD;
	//}

	RtlFillMemory(&m_BlockSkipHM, sizeof(m_BlockSkipHM), BLOCK_PROD);
	m_AllBlockSkipHM = false;
}

void CStep::SetBlockSkipHM(long BlockNo, long Skip)
{
	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		m_BlockSkipHM[BlockNo - 1] = Skip;
	}
}

long CStep::GetBlockSkipHM(long BlockNo)
{
	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		return m_BlockSkipHM[BlockNo - 1];
	}

	return 0;
}

void CStep::SetBlockUse(long BlockNo, long Use)
{
	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		m_BlockUse[BlockNo - 1] = Use;
	}
}

long CStep::GetBlockUse(long BlockNo)
{
	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		return m_BlockUse[BlockNo - 1];
	}

	return 0;
}

long CStep::GetBlockUseFromInsertNo(long InsertNo)
{
	long BlockNo = GetBlockNoFromInsertNo(InsertNo);

	if (GetBlockSequence() != BLOCK_SEQ_OVERALL)
	{
		return 1;
	}
	else if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		return m_BlockUse[BlockNo - 1];
	}

	return 0;
}

void CStep::SetBlockSequence(long Seq)
{
	m_BlockSequence = Seq;
}

long CStep::GetBlockSequence()
{
	return m_BlockSequence;
}

void CStep::SetAllBlockSkipHM(bool AllSkip)
{
	m_AllBlockSkipHM = AllSkip;
}

bool CStep::GetAllBlockSkipHM()
{
	return m_AllBlockSkipHM;
}

long CStep::GetBlockNoFromInsertNo(long InsertNo)
{
	long BlockNo = 0;

	if (InsertNo > 0 && InsertNo <= MAXINSERTNO)
	{
		BlockNo = m_BlockNo[InsertNo - 1];
	}

	return BlockNo;
}


long CStep::SetForming(long FeederNo, FORMING_COMPONENT Forming)
{
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		m_Forming[FeederNo - 1] = Forming;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] JobInfo FormingUse:%d FdNo:%d InsertCase:%.3f~%.3f Case1:%.3f~%.3f Case2:%.3f~%.3f Pt1:%.3f,%.3f,%.3f,%.3f Pt2:%.3f,%.3f,%.3f,%.3f\n"),
				Forming.Use, Forming.FeederNo,
				Forming.InsertCase.Min, Forming.InsertCase.Max,
				Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
				Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z);
		}
	}
	return NO_ERR;
}

FORMING_COMPONENT CStep::GetForming(long FeederNo)
{
	FORMING_COMPONENT Forming;
	ZeroMemory(&Forming, sizeof(Forming));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Forming = m_Forming[FeederNo - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] JobInfo FormingUse:%d FdNo:%d InsertCase:%.3f~%.3f Case1:%.3f~%.3f Case2:%.3f~%.3f Pt1:%.3f,%.3f,%.3f,%.3f Pt2:%.3f,%.3f,%.3f,%.3f\n"),
				Forming.Use, Forming.FeederNo,
				Forming.InsertCase.Min, Forming.InsertCase.Max,
				Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
				Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z);
		}
	}
	return Forming;
}

void CStep::SetFormingMotionDone(long insertOrd, long bDone)
{
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		m_bFormingMotionDone[insertOrd - 1] = bDone;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetFormingMotionDone InsertOrd:%d Done:%d\n"), insertOrd, bDone);
		}
	}
}

bool CStep::GetFormingMotionDone(long insertOrd)
{
	bool bDone = false;
	if (insertOrd > 0 && insertOrd <= MAXUSEDHEADNO)
	{
		bDone = m_bFormingMotionDone[insertOrd - 1];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetFormingMotionDone InsertOrd:%d Done:%d\n"), insertOrd, bDone);
		}
	}
	return bDone;
}