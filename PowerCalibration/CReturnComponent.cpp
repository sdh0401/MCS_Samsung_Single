#include "pch.h"
#include "CReturnComponent.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CStep.h"

CReturnComponent* gcReturnComponent;
CReturnComponent::CReturnComponent()
{
	m_MaxReturnOrder = 1;
	m_ProdRunMode = RUN_REAL;
}

CReturnComponent::~CReturnComponent()
{
}

void CReturnComponent::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CReturnComponent::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CReturnComponent::GetReturnComponentHeadNo(long ReturnOrd)
{
	long HeadNo = TBL_HEAD1;
	HeadNo = gcStep->GetReturnHeadNo(ReturnOrd);
	return HeadNo;
}

long CReturnComponent::GetMaxReturnComponentOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxReturnComponentOrder();
	return RetMaxOrder;
}

void CReturnComponent::SetMaxReturnComponentOrder(long MaxPickOrd)
{
	//if (MaxPickOrd > 0 && MaxPickOrd <= MAXUSEDHEADNO)
	//{
	//	m_MaxPickOrder = MaxPickOrd;
	//}
}

long CReturnComponent::GetFdNoFromReturnComponentOrder(long ReturnOrd)
{
	long FdNo = 30;
	FdNo = gcStep->GetFdNoFromReturnOrder(ReturnOrd);
	return FdNo;
}

long CReturnComponent::GetReturnDelayFromFdNo(long FdNo)
{
	long PickDelay;
	PickDelay = gcStep->GetReturnDelayFromFdNo(FdNo);
	return PickDelay;
}

Ratio_XYRZ CReturnComponent::GetRatioFromFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = gcStep->GetRatioFromFdNo(FdNo);
	return ratio;
}

Point_XYRZ CReturnComponent::GetPickOffsetFromFdNo(long FdNo)
{
	Point_XYRZ pt;
	pt = gcStep->GetPickOffsetFromFdNo(FdNo);
	return pt;
}

long CReturnComponent::Run(long Gantry)
{
	long Err = NO_ERR, NozzleNo = 0;
	CString strZAxis, strRAxis;
	Ratio_XYRZ CompRatio[MAXUSEDHEADNO];
	long Ms = TIME30MS, TimeOut = TIME5000MS, HeadNo[MAXUSEDHEADNO], JobPickDelay[MAXUSEDHEADNO], FdNo[MAXUSEDHEADNO];
	double InposR = 1.000, InposZ = 0.050, RatioXY = 0.3, RatioR = 0.7, RatioZ = 1.0, Pitch = 1.0, ComponentHeight = 0.0, Angle = 0.0;
	long NozzleType = 0;
	Point_XY pt, RefPt;
	Point_XYRZ ReturnOffset[MAXUSEDHEADNO];
	NOZZLE Nozzle[MAXUSEDHEADNO];
	//CApplicationTime* pTime = new CApplicationTime();
	ULONGLONG GetTime = 0, Elapsed = 0;
	ZeroMemory(&FdNo, sizeof(FdNo));
	ZeroMemory(&HeadNo, sizeof(HeadNo));
	ZeroMemory(&Nozzle, sizeof(Nozzle));
	ZeroMemory(&ReturnOffset, sizeof(ReturnOffset));
	ZeroMemory(&JobPickDelay, sizeof(JobPickDelay));
	TRACE(_T("[PWR] Return Run Step1\n"));
	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] Measure Height WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), RatioZ);
	if (Err != NO_ERR)
	{
		return Err;
	}
	TRACE(_T("[PWR] Return Run Step2 Err:%d\n"), Err);
	ZeroMemory(&FdNo, sizeof(FdNo));
	ZeroMemory(&HeadNo, sizeof(HeadNo));
	ZeroMemory(&ReturnOffset, sizeof(ReturnOffset));
	ZeroMemory(&JobPickDelay, sizeof(JobPickDelay));
	Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), RatioZ);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Return StartAllZAxisWaitMotion Err:%d\n"), Err);
		return Err;
	}
	for (long ReturnOrd = 0; ReturnOrd < GetMaxReturnComponentOrder(); ++ReturnOrd)
	{
		HeadNo[ReturnOrd] = GetReturnComponentHeadNo(ReturnOrd + 1);
		FdNo[ReturnOrd] = GetFdNoFromReturnComponentOrder(ReturnOrd + 1);
		if (HeadNo[ReturnOrd] == 0) continue;		
		if (FdNo[ReturnOrd] == 0) continue;
		strRAxis = GetRAxisFromHeadNo(Gantry, HeadNo[ReturnOrd]);
		strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo[ReturnOrd]);
		RefPt = GetFeederPosition(Gantry, FdNo[ReturnOrd]);
		ReturnOffset[ReturnOrd] = GetPickOffsetFromFdNo(FdNo[ReturnOrd]);
		CompRatio[ReturnOrd] = GetRatioFromFdNo(FdNo[ReturnOrd]);
		JobPickDelay[ReturnOrd] = GetReturnDelayFromFdNo(FdNo[ReturnOrd]);
		NozzleNo = GetGlobalNozzleNo(HeadNo[ReturnOrd]);
		Nozzle[ReturnOrd] = GetGlobalNozzleInformation(NozzleNo); //GetNozzle(FdNo[ReturnOrd]);
		pt.x = RefPt.x + ReturnOffset[ReturnOrd].x;
		pt.y = RefPt.y + ReturnOffset[ReturnOrd].y;
		Err = MoveZStandy(Gantry, GetStandByZ(Gantry), RatioZ);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return MoveZStandy Err:%d\n"), Err);
			return Err;
		}

		Err = StartAllRAxisWaitMotion(Gantry, GetStandByR(Gantry), RatioR, TIME5000MS);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return StartAllRAxisWaitMotion Err:%d\n"), Err);
			return Err;
		}
		Err = LinearIntplPosWaitMotion(Gantry, HeadNo[ReturnOrd], pt, RatioXY, TIME5000MS);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return LinearIntplPosWaitMotion Err:%d\n"), Err);
			return Err;
		}
		Err = StartPosWaitDelayedInposition(strRAxis, CompRatio[ReturnOrd].r, TimeOut, ReturnOffset[ReturnOrd].r, InposR, Ms, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return StartPosWaitDelayedInposition Err:%d\n"), Err);
			return Err;
		}
		ReturnOffset[ReturnOrd].z = ReturnOffset[ReturnOrd].z - Nozzle[ReturnOrd].TipHeight + Nozzle[ReturnOrd].PusherHeight;
		TRACE(_T("[PWR] ReturnOrd(%d) Run Step3 TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
			ReturnOrd,
			Nozzle[ReturnOrd].TipHeight, Nozzle[ReturnOrd].PusherHeight, ReturnOffset[ReturnOrd].z);
		Err = StartPosWaitDelayedInposition(strZAxis, CompRatio[ReturnOrd].z, TimeOut, ReturnOffset[ReturnOrd].z, InposZ, Ms, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return StartPosWaitDelayedInposition Err:%d\n"), Err);
			return Err;
		}
		Err = SuctionOne(Gantry, HeadNo[ReturnOrd], false);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return SuctionOne Err:%d\n"), Err);
			return Err;
		}
		ThreadSleep(JobPickDelay[ReturnOrd]);
		Err = MoveZStandy(Gantry, GetStandByZ(Gantry), RatioZ);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Return MoveZStandy Err:%d\n"), Err);
			return Err;
		}
	}
	return Err;
}

