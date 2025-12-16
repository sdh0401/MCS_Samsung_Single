#include "pch.h"
#include "CDiscard.h"
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
#include "CReadJobFile.h"
#include "CCamDropCheck.h"
#include <typeinfo>
#include "CTrayDumpBox.h"

CDiscard* gcDiscard;
CDiscard::CDiscard(long Gantry)
{
	m_Gantry = Gantry;
	m_ProdRunMode = RUN_REAL;
}

CDiscard::~CDiscard()
{
}

void CDiscard::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
}

long CDiscard::GetProdRunMode()
{
	return m_ProdRunMode;
}

long CDiscard::SetSendEmpty(long FeederNo, bool bSendEmpty)
{
	gcStep->SetSendEmpty(FeederNo, bSendEmpty);
	return NO_ERR;
}

long CDiscard::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = gcStep->GetMaxInsertOrder();
	return RetMaxOrder;
}

long CDiscard::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadNo = 0;
	HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	}
	return HeadNo;
}

long CDiscard::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo = 0;
	FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
	return FdNo;
}

long CDiscard::GetReadyTimeOutEmpty(long FeederNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmpty(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmpty FeederNo:%d ReadyTimeOutEmpty:%d\n"), FeederNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CDiscard::GetReadyTimeOutEmptyByHeadNo(long HeadNo)
{
	long ReadyTimeOutEmpty = 0;
	ReadyTimeOutEmpty = gcStep->GetReadyTimeOutEmptyByHeadNo(HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetReadyTimeOutEmptyByHeadNo HeadNo:%d ReadyTimeOutEmpty:%d\n"), HeadNo, ReadyTimeOutEmpty);
	}
	return ReadyTimeOutEmpty;
}

long CDiscard::GetVisionErrorEmpty(long FeederNo)
{
	long VisionErrorEmpty = 0;
	VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetVisionErrorEmpty FeederNo:%d VisionErrorEmpty:%d\n"), FeederNo, VisionErrorEmpty);
	}
	return VisionErrorEmpty;
}

double CDiscard::GetPickupZStandBy()
{
	double StandByZ = 50.0;
	StandByZ = gcStep->GetPickupZStandBy();
	return StandByZ;
}

#pragma warning (suppress: 6262)
long CDiscard::DiscardOneBeforePicking(long HeadNo, Ratio_XYRZ Ratio)
{
	long Err = NO_ERR, Ms = TIME100MS, TimeOut = TIME5000MS;
	double InposXY = 0.1, InposR = 1.000, InposZ = 0.050;
	Point_XY pt;
	//Ratio_XYRZ Ratio;
	Point_XYRZ Discard;
	CString strRAxis, strZAxis;
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	ZeroMemory(&Discard, sizeof(Discard));
	strRAxis = GetRAxisFromHeadNo(m_Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(m_Gantry, HeadNo);
	Ratio.xy = Ratio.r = Ratio.z = 0.7;
	Discard = GetGlobalDiscardPosition();
    Discard = gcLastPickFront->getDiscardInfo(HeadNo).getDiscardPoint(true);
    const long feederNumber = gcLastPickFront->getDiscardInfo(HeadNo).getFeederNumber();
    const long discardMode = gcReadJobFile->GetDiscard(feederNumber).Mode;
    const bool useTrayDiscard = (discardMode == 2 || discardMode == 3);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        const Point_XY discardRefPosition = { dumpTrayData.ptRef.x , dumpTrayData.ptRef.y };//
        const TRAY_INFO trayInfo = dumpTrayData.trayInfo;//잡파일 어딘가에서... 얘도 const 가능?
        const long trayDiscardNowPocket = dumpTrayData.NowPocket;//
        if (trayInfo.MaxPocket <= trayDiscardNowPocket)//꽉 찼는지 검사
        {
            Err = SendAlarm(ERR_TRAYDUMPBOX_FULL(discardMode == 2 ? 0 : 1), L"TrayDumpBox is full. (trayInfo.MaxPocket <= trayDiscardNowPocket)");
            return Err;
        }

        //discardRefPosition + trayInfo.pt
        const Point_XY goal = { discardRefPosition.x + trayInfo.pt[trayDiscardNowPocket].x, discardRefPosition.y + trayInfo.pt[trayDiscardNowPocket].y };

        //여기는 pt가 아니라 Discard로 해야 정상적으로 처리됨.
        Discard.x = goal.x;
        Discard.y = goal.y;
        Discard.z = dumpTrayData.ptRef.z;

        CString temp; temp.Format(L"x : %.3f, y : %.3f, z : %.3f, r : %.3f due to useTrayDiscard == true.", Discard.x, Discard.y, Discard.z, dumpTrayData.ptRef.r);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);

        //R축 돌리기..
        //Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, dumpTrayData.ptRef.r, InposR, Ms, true);
        //if (Err != NO_ERR)
        //{
        //    TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "MoveR failed.");
        //    return Err;
        //}
    }
	pt.x = Discard.x;
	pt.y = Discard.y;
	double StandByZ = GetRuntimeSafetyZ(m_Gantry);
	GetTime = _time_get();
	Err = WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitAllZIdle Err:%d\n"), Err);
		return Err;
	}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	GetTime = _time_get();
	Err = WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitAllRIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitAllRIdle Err:%d\n"), Err);
		return Err;
	}
	
	// 대형 자재 때문에 R축 구동 삭제.
	//Err = StartAllRAxisWaitMotion(m_Gantry, GetStandByR(m_Gantry), Ratio.r, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneBeforePicking StartAllRAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}

	Err = LinearIntplPosWaitDelayedPosSet(m_Gantry, HeadNo, pt, Ratio.xy, InposXY, Ms, TimeOut);
	//Err = LinearIntplPosWaitMotion(m_Gantry, HeadNo, pt, Ratio.xy, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking LinearIntplPosWaitMotion Err:%d\n"), Err);
		return Err;
	}

	//Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, Discard.r, InposR, Ms, true);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneBeforePicking StartPosWaitDelayedInposition Err:%d\n"), Err);
	//	return Err;
	//}

    const long NozzleNo = GetGlobalNozzleNo(HeadNo);
    const NOZZLE Nozzle = GetGlobalNozzleInformation(NozzleNo);
    Discard.z = Discard.z - Nozzle.TipHeight + Nozzle.PusherHeight;
    TRACE(_T("[PWR] DiscardOneBeforePicking TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
          Nozzle.TipHeight, Nozzle.PusherHeight, Discard.z);
	Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
	if (limit.minus > Discard.z)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking Target position,%.3f is under minus limit,%.3f\n"), Discard.z, limit.minus);
		Discard.z = limit.minus + 1.0;
	}
	Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, Discard.z, InposZ, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}
	Err = SuctionOne(m_Gantry, HeadNo, false);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        dumpTrayData.NowPocket++;

        Err = gcTrayDumpBox->SetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->SetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        gcTrayDumpBox->sendTrayDumpBoxLastPocket(discardMode == 2 ? 0 : 1, dumpTrayData.NowPocket);
    }
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking SuctionOne Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME500MS);

	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforePicking MoveZStandy Err:%d\n"), Err);
		return Err;
	}

	return Err;
}

long CDiscard::DiscardAllBeforePicking(Ratio_XYRZ ratio)
{
	double maxRatio = 0.5;

	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	if (ratio.xy > maxRatio)
	{
		ratio.xy = maxRatio;
	}
	if (ratio.r > maxRatio)
	{
		ratio.r = maxRatio;
	}
	if (ratio.z > maxRatio)
	{
		ratio.z = maxRatio;
	}

	long Err = NO_ERR, Gantry = FRONT_GANTRY, SuctionIONo = IO_NOUSE;
	UBYTE ucStatus = OUTOFF;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] DiscardAllBeforePicking GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] DiscardAllBeforePicking GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		SuctionIONo = GetSuctionIONo(Gantry, HeadNo + TBL_HEAD1);
		if(SuctionIONo != IO_NOUSE)
		{
			ucStatus = ReadOutputOne(SuctionIONo);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] DiscardAllBeforePicking HeadNo:%02d Suction:%d\n"), HeadNo + TBL_HEAD1, ucStatus);
			}
			if (ucStatus == OUTON)
			{
				if (gcLastPickFront->GetHeadData(HeadNo + TBL_HEAD1).Enable == true)
				{
					TRACE(_T("[PWR] DiscardAllBeforePicking Skip LastPick %s\n"), GetZAxisFromHeadNo(Gantry, HeadNo + TBL_HEAD1));

					Err = MoveZStandy(m_Gantry, GetRuntimeSafetyZ(m_Gantry), ratio.z);
					if (Err != NO_ERR)
					{
						return Err;
					}

					Err = WaitZStandy(m_Gantry, GetRuntimeSafetyZ(Gantry));
					if (Err != NO_ERR)
					{
						return Err;
					}
				}
				else
				{
					TRACE(_T("[PWR] DiscardAllBeforePicking Start %s RatioXYRZ %.1f %.1f %.1f\n"), GetZAxisFromHeadNo(Gantry, HeadNo + TBL_HEAD1), ratio.xy, ratio.r, ratio.z);

					Err = DiscardOneBeforePicking(HeadNo + TBL_HEAD1, ratio);

					if (Err != NO_ERR)
					{
						TRACE(_T("[PWR] DiscardOneBeforePicking Err:%d\n"), Err);
						return Err;
					}
				}
			}
		}
	}
	return Err;
}

#pragma warning (suppress: 6262)
long CDiscard::DiscardOneBeforeAlignChecking(long FeederNo, long HeadNo)
{
	long Err = NO_ERR, Ms = TIME100MS, TimeOut = TIME5000MS, PickRetry = 0, NozzleNo = 0;
	double InposXY = 0.1, InposR = 1.000, InposZ = 0.050;
	Point_XY pt;
	Ratio_XYRZ Ratio;
	Point_XYRZ Discard;
	CString strRAxis, strZAxis;
	//NOZZLE Nozzle = gcStep->GetNozzle(FeederNo);
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	ZeroMemory(&Discard, sizeof(Discard));
	Discard = gcStep->GetDiscardPoint(FeederNo);
    Discard = gcLastPickFront->getDiscardInfo(HeadNo).getDiscardPoint(true);
    const long feederNumber = gcLastPickFront->getDiscardInfo(HeadNo).getFeederNumber();
    const long discardMode = gcReadJobFile->GetDiscard(feederNumber).Mode;
    const bool useTrayDiscard = (discardMode == 2 || discardMode == 3);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        const Point_XY discardRefPosition = { dumpTrayData.ptRef.x , dumpTrayData.ptRef.y };//
        const TRAY_INFO trayInfo = dumpTrayData.trayInfo;//잡파일 어딘가에서... 얘도 const 가능?
        const long trayDiscardNowPocket = dumpTrayData.NowPocket;//
        if (trayInfo.MaxPocket <= trayDiscardNowPocket)//꽉 찼는지 검사
        {
            Err = SendAlarm(ERR_TRAYDUMPBOX_FULL(discardMode == 2 ? 0 : 1), L"TrayDumpBox is full. (trayInfo.MaxPocket <= trayDiscardNowPocket)");
            return Err;
        }

        //discardRefPosition + trayInfo.pt
        const Point_XY goal = { discardRefPosition.x + trayInfo.pt[trayDiscardNowPocket].x, discardRefPosition.y + trayInfo.pt[trayDiscardNowPocket].y };

        //여기는 pt가 아니라 Discard로 해야 정상적으로 처리됨.
        Discard.x = goal.x;
        Discard.y = goal.y;
        Discard.z = dumpTrayData.ptRef.z;

        CString temp; temp.Format(L"x : %.3f, y : %.3f, z : %.3f, r : %.3f due to useTrayDiscard == true.", Discard.x, Discard.y, Discard.z, dumpTrayData.ptRef.r);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);

        //R축 돌리기..
        //Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, dumpTrayData.ptRef.r, InposR, Ms, true);
        //if (Err != NO_ERR)
        //{
        //    TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "MoveR failed.");
        //    return Err;
        //}
    }
	strRAxis = GetRAxisFromHeadNo(m_Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(m_Gantry, HeadNo);
	Ratio = gcStep->GetRatioFromFdNo(FeederNo);
	PickRetry = gcStep->GetPickRetryFromFdNo(FeederNo);
	pt.x = Discard.x;
	pt.y = Discard.y;
	NozzleNo = GetGlobalNozzleNo(HeadNo);
	NOZZLE Nozzle = GetGlobalNozzleInformation(NozzleNo);
	GetTime = _time_get();
	double StandByZ = GetStandByZ(m_Gantry);
	Err = WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitAllZIdle Err:%d\n"), Err);
		return Err;
	}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking MoveZStandy Err:%d\n"), Err);
		return Err;
	}

	GetTime = _time_get();
	Err = WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitAllRIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitAllRIdle Err:%d\n"), Err);
		return Err;
	}

	// 대형 자재 때문에 R축 구동 삭제.
	//Err = StartAllRAxisWaitMotion(m_Gantry, GetStandByR(m_Gantry), Ratio.r, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneBeforeAlignChecking StartAllRAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}

	Err = LinearIntplPosWaitMotion(m_Gantry, HeadNo, pt, Ratio.xy, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking LinearIntplPosWaitMotion Err:%d\n"), Err);
		return Err;
	}
	//Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, Discard.r, InposR, Ms, true);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneBeforeAlignChecking StartPosWaitDelayedInposition Err:%d\n"), Err);
	//	return Err;
	//}
	Discard.z = Discard.z - Nozzle.TipHeight + Nozzle.PusherHeight;
	TRACE(_T("[PWR] DiscardOneBeforeAlignChecking TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
		Nozzle.TipHeight, Nozzle.PusherHeight, Discard.z);
	Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
	if (limit.minus > Discard.z)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking Target position,%.3f is under minus limit,%.3f\n"), Discard.z, limit.minus);
		Discard.z = limit.minus + 1.0;
	}
	Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, Discard.z, InposZ, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}
	Err = SuctionOne(m_Gantry, HeadNo, false);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        dumpTrayData.NowPocket++;

        Err = gcTrayDumpBox->SetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->SetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        gcTrayDumpBox->sendTrayDumpBoxLastPocket(discardMode == 2 ? 0 : 1, dumpTrayData.NowPocket);
    }
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking SuctionOne Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME500MS);
	if (GetProdRunMode() == RUN_REAL)
	{
		if (GetReadyTimeOutEmpty(FeederNo) == 1)
		{
			if (GetFeederEmptyDisplay(FeederNo) == false)
			{
				SetFeederEmptyDisplay(FeederNo, true);
				SendEmpty(FeederNo, EMPTY_READY_TIMEOUT, _T("Ready IO TimeOut"));
			}
			SetSendEmpty(FeederNo, true);
		}
	}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneBeforeAlignChecking MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	return Err;
}

long CDiscard::DiscardAllBeforeAlignChecking()
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, FeederNo = 0, HeadNo = 0;
	TRACE(_T("[PWR] DiscardAllBeforeAlignChecking MaxInsertOrder:%d\n"), GetMaxInsertOrder());
	for (long insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] DiscardAllBeforeAlignChecking GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] DiscardAllBeforeAlignChecking GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		HeadNo = GetHeadNoFromInsertOrder(insertOrd + 1);
		FeederNo = GetFdNoFromInsertOrder(insertOrd + 1);
		TRACE(_T("[PWR] DiscardAllBeforeAlignChecking FdNo:%d HdNo:%d\n"), FeederNo, HeadNo);
		if (GetReadyTimeOutEmptyByHeadNo(HeadNo) == 1)
		{
			Err = DiscardOneBeforeAlignChecking(FeederNo, HeadNo);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] DiscardAllBeforeAlignChecking HeadNo:%d FeederNo:%d Err:%d\n"), HeadNo, FeederNo, Err);
				break;
			}
		}
	}

	//Err = AllBeforeAlignChecking();

	return Err;
}

#pragma warning (suppress: 6262)
long CDiscard::DiscardOneAfterAlignChecking(long FeederNo, long HeadNo)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, Ms = TIME100MS, TimeOut = TIME5000MS, PickRetry = 0, NozzleNo = 0;
	double InposXY = 0.1, InposR = 1.000, InposZ = 0.050;
	Point_XY pt;
	Ratio_XYRZ Ratio;
	Point_XYRZ Discard;
	CString strRAxis, strZAxis;
	//NOZZLE Nozzle = gcStep->GetNozzle(FeederNo);
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	NozzleNo = GetGlobalNozzleNo(HeadNo);
	NOZZLE Nozzle = GetGlobalNozzleInformation(NozzleNo);
	ZeroMemory(&Discard, sizeof(Discard));
	Discard = gcStep->GetDiscardPoint(FeederNo);
    Discard = gcLastPickFront->getDiscardInfo(HeadNo).getDiscardPoint(true);
    const long feederNumber = gcLastPickFront->getDiscardInfo(HeadNo).getFeederNumber();
    const long discardMode = gcReadJobFile->GetDiscard(feederNumber).Mode;
    const bool useTrayDiscard = (discardMode == 2 || discardMode == 3);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        const Point_XY discardRefPosition = { dumpTrayData.ptRef.x , dumpTrayData.ptRef.y };//
        const TRAY_INFO trayInfo = dumpTrayData.trayInfo;//잡파일 어딘가에서... 얘도 const 가능?
        const long trayDiscardNowPocket = dumpTrayData.NowPocket;//
        if (trayInfo.MaxPocket <= trayDiscardNowPocket)//꽉 찼는지 검사
        {
            Err = SendAlarm(ERR_TRAYDUMPBOX_FULL(discardMode == 2 ? 0 : 1), L"TrayDumpBox is full. (trayInfo.MaxPocket <= trayDiscardNowPocket)");
            return Err;
        }

        //discardRefPosition + trayInfo.pt
        const Point_XY goal = { discardRefPosition.x + trayInfo.pt[trayDiscardNowPocket].x, discardRefPosition.y + trayInfo.pt[trayDiscardNowPocket].y };

        //여기는 pt가 아니라 Discard로 해야 정상적으로 처리됨.
        Discard.x = goal.x;
        Discard.y = goal.y;
        Discard.z = dumpTrayData.ptRef.z;

        CString temp; temp.Format(L"x : %.3f, y : %.3f, z : %.3f, r : %.3f due to useTrayDiscard == true.", Discard.x, Discard.y, Discard.z, dumpTrayData.ptRef.r);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);

        //R축 돌리기..
        //Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, dumpTrayData.ptRef.r, InposR, Ms, true);
        //if (Err != NO_ERR)
        //{
        //    TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "MoveR failed.");
        //    return Err;
        //}
    }
	strRAxis = GetRAxisFromHeadNo(m_Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(m_Gantry, HeadNo);
	Ratio = gcStep->GetRatioFromFdNo(FeederNo);
	PickRetry = gcStep->GetPickRetryFromFdNo(FeederNo);
	pt.x = Discard.x;
	pt.y = Discard.y;
	double StandByZ = GetStandByZ(m_Gantry);
	GetTime = _time_get();
	Err = WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitAllZIdle Err:%d\n"), Err);
		return Err;
	}
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterAlignChecking StartAllZAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	GetTime = _time_get();
	Err = WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitAllRIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitAllRIdle Err:%d\n"), Err);
		return Err;
	}

	// 대형 자재 때문에 R축 구동 삭제.
	//Err = StartAllRAxisWaitMotion(m_Gantry, GetStandByR(m_Gantry), Ratio.r, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterAlignChecking StartAllRAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	if (_time_elapsed(GetTime) > 0)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	}
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}
	
	Err = LinearIntplPosWaitMotion(m_Gantry, HeadNo, pt, Ratio.xy, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking LinearIntplPosWaitMotion Err:%d\n"), Err);
		return Err;
	}
	//Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, Discard.r, InposR, Ms, true);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterAlignChecking StartPosWaitDelayedInposition Err:%d\n"), Err);
	//	return Err;
	//}
	Discard.z = Discard.z - Nozzle.TipHeight + Nozzle.PusherHeight;
	TRACE(_T("[PWR] DiscardOneAfterAlignChecking TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
		Nozzle.TipHeight, Nozzle.PusherHeight, Discard.z);
	Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
	if (limit.minus > Discard.z)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking Target position,%.3f is under minus limit,%.3f\n"), Discard.z, limit.minus);
		Discard.z = limit.minus + 1.0;
	}
	Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, Discard.z, InposZ, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}

	SendDiscardStatics(FeederNo, HeadNo);

	Err = SuctionOne(m_Gantry, HeadNo, false);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        dumpTrayData.NowPocket++;

        Err = gcTrayDumpBox->SetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->SetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        gcTrayDumpBox->sendTrayDumpBoxLastPocket(discardMode == 2 ? 0 : 1, dumpTrayData.NowPocket);
    }
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        dumpTrayData.NowPocket++;

        Err = gcTrayDumpBox->SetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->SetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        gcTrayDumpBox->sendTrayDumpBoxLastPocket(discardMode == 2 ? 0 : 1, dumpTrayData.NowPocket);
    }
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking SuctionOne Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME500MS);
	if (GetProdRunMode() == RUN_REAL)
	{
		if (GetReadyTimeOutEmpty(FeederNo) == 1)
		{
			if (GetFeederEmptyDisplay(FeederNo) == false)
			{
				SetFeederEmptyDisplay(FeederNo, true);
				SendEmpty(FeederNo, EMPTY_READY_TIMEOUT, _T("Ready IO TimeOut"));
			}
			SetSendEmpty(FeederNo, true);
		}
		else if (GetVisionErrorEmpty(FeederNo) > PickRetry)
		{
			if (GetFeederEmptyDisplay(FeederNo) == false)
			{
				SetFeederEmptyDisplay(FeederNo, true);
				SendEmpty(FeederNo, EMPTY_PICKRETRY_VISERROR, _T("Over Pick Retry"));
			}
			SetSendEmpty(FeederNo, true);
		}
	}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterAlignChecking MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	return Err;
}

long CDiscard::DiscardAllAfterAlignChecking()
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, FeederNo = 0, HeadNo = 0;
	for (long insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] DiscardAllAfterAlignChecking GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] DiscardAllAfterAlignChecking GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		HeadNo = GetHeadNoFromInsertOrder(insertOrd + 1);
		FeederNo = GetFdNoFromInsertOrder(insertOrd + 1);
		if (GetVisionErrorEmpty(FeederNo) > 0 || GetReadyTimeOutEmptyByHeadNo(HeadNo) == 1)
		{
			Err = DiscardOneAfterAlignChecking(FeederNo, HeadNo);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] DiscardAllAfterAlignChecking HeadNo:%d FeederNo:%d Err:%d\n"), HeadNo, FeederNo, Err);
				break;
			}
		}
	}



	return Err;
}

#pragma warning (suppress: 6262)
long CDiscard::DiscardOneAfterInserting(long FeederNo, long HeadNo)//here
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, Ms = TIME100MS, TimeOut = TIME5000MS, PickRetry = 0, NozzleNo = 0;
	double InposXY = 0.5, InposR = 1.000, InposZ = 0.050, MaxComponentHeight = 30.0;
	Point_XY pt;
	Ratio_XYRZ Ratio;
	Point_XYRZ Discard;
	CString strRAxis, strZAxis;
	//NOZZLE Nozzle = gcStep->GetNozzle(FeederNo);
	ULONGLONG GetTime = _time_get(), Elapsed = 0;
	ZeroMemory(&Discard, sizeof(Discard));
	NozzleNo = GetGlobalNozzleNo(HeadNo);
	NOZZLE Nozzle = GetGlobalNozzleInformation(NozzleNo);
	Discard = gcStep->GetDiscardPoint(FeederNo);
    Discard = gcLastPickFront->getDiscardInfo(HeadNo).getDiscardPoint(true);
    const long feederNumber = gcLastPickFront->getDiscardInfo(HeadNo).getFeederNumber();
    const long discardMode = gcReadJobFile->GetDiscard(feederNumber).Mode;
    const bool useTrayDiscard = (discardMode == 2 || discardMode == 3);
	strRAxis = GetRAxisFromHeadNo(m_Gantry, HeadNo);
	strZAxis = GetZAxisFromHeadNo(m_Gantry, HeadNo);
	Ratio = gcStep->GetRatioFromFdNo(FeederNo);
	PickRetry = gcStep->GetPickRetryFromFdNo(FeederNo);
	MaxComponentHeight = gcStep->GetMaxComponentHeight();
	double StandByZ = GetStandByZ(m_Gantry) - MaxComponentHeight;

	//long suctionIO = GetSuctionIONo(m_Gantry, HeadNo);

	//if (ReadOutputOne(suctionIO) == OUTOFF)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterInserting %s Suction Off Skip"), strZAxis);
	//	return NO_ERR;
	//}

	TRACE(_T("[PWR] DiscardOneAfterInserting FeederNo:%d HeadNo:%d Ratio XY:%.1f R:%.1f Z:%.1f\n"),
		FeederNo, HeadNo, Ratio.xy, Ratio.r, Ratio.z);
	pt.x = Discard.x;
	pt.y = Discard.y;

	GetTime = _time_get();
	Err = WaitAllZIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] DiscardOneAfterInserting WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting WaitAllZIdle Err:%d\n"), Err);
		return Err;
	}
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterInserting StartAllZAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	GetTime = _time_get();
	Err = WaitAllRIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] DiscardOneAfterInserting WaitAllRIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting WaitAllRIdle Err:%d\n"), Err);
		return Err;
	}

	// 대형 자재 때문에 R축 구동 삭제.
	//Err = StartAllRAxisWaitMotion(m_Gantry, GetStandByR(m_Gantry), Ratio.r, TIME5000MS);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterInserting StartAllRAxisWaitMotion Err:%d\n"), Err);
	//	return Err;
	//}

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] DiscardOneAfterInserting WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}

    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::l%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        const Point_XY discardRefPosition = { dumpTrayData.ptRef.x , dumpTrayData.ptRef.y };//

        const TRAY_INFO trayInfo = dumpTrayData.trayInfo;//잡파일 어딘가에서... 얘도 const 가능?

        const long trayDiscardNowPocket = dumpTrayData.NowPocket;//

        if (trayInfo.MaxPocket <= trayDiscardNowPocket)//꽉 찼는지 검사
        {
            Err = SendAlarm(ERR_TRAYDUMPBOX_FULL(discardMode == 2 ? 0 : 1), L"TrayDumpBox is full. (trayInfo.MaxPocket <= trayDiscardNowPocket)");
            return Err;
        }

        //discardRefPosition + trayInfo.pt
        pt = Point_XY{ discardRefPosition.x + trayInfo.pt[trayDiscardNowPocket].x, discardRefPosition.y + trayInfo.pt[trayDiscardNowPocket].y };

        Discard.z = dumpTrayData.ptRef.z;

        CString temp; temp.Format(L"x : %.3f, y : %.3f, z : %.3f, r : %.3f due to useTrayDiscard == true.", Discard.x, Discard.y, Discard.z, dumpTrayData.ptRef.r);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);

        //R축 돌리기..
        //Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, dumpTrayData.ptRef.r, InposR, Ms, true);
        //if (Err != NO_ERR)
        //{
        //    TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "MoveR failed.");
        //    return Err;
        //}
    }

	Err = LinearIntplPosWaitMotion(m_Gantry, HeadNo, pt, Ratio.xy, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting LinearIntplPosWaitDelayedPosSet Err:%d\n"), Err);
		return Err;
	}
	//Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, Discard.r, InposR, Ms, true);
	//if (Err != NO_ERR)
	//{
	//	TRACE(_T("[PWR] DiscardOneAfterInserting StartPosWaitDelayedInposition Err:%d\n"), Err);
	//	return Err;
	//}
	Discard.z = Discard.z - Nozzle.TipHeight + Nozzle.PusherHeight;
	TRACE(_T("[PWR] DiscardOneAfterInserting TipHeight:%.3f PusherHeight:%.3f Target:%.3f\n"),
		Nozzle.TipHeight, Nozzle.PusherHeight, Discard.z);
	Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
	if (limit.minus > Discard.z)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting Target position,%.3f is under minus limit,%.3f\n"), Discard.z, limit.minus);
		Discard.z = limit.minus + 1.0;
	}
	Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, Discard.z, InposZ, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}

	SendDiscardStatics(FeederNo, HeadNo);

	Err = SuctionOne(m_Gantry, HeadNo, false);
    if (useTrayDiscard == true)
    {
        DUMP_TRAY dumpTrayData = DUMP_TRAY();
        Err = gcTrayDumpBox->GetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->GetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        dumpTrayData.NowPocket++;

        Err = gcTrayDumpBox->SetBoxInfo(discardMode == 2 ? 0 : 1, dumpTrayData);
        if (Err != NO_ERR)
        {
            TRACE("[PWR] %s::%s -> %s", typeid(CDiscard).name(), __func__, "gcTrayDumpBox->SetBoxInfo(..) != NO_ERR");//휴먼에러
            return Err;
        }

        gcTrayDumpBox->sendTrayDumpBoxLastPocket(discardMode == 2 ? 0 : 1, dumpTrayData.NowPocket);
    }
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting SuctionOne Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME500MS);
	if (GetProdRunMode() == RUN_REAL)
	{
		if (GetReadyTimeOutEmpty(FeederNo) == 1)
		{
			if (GetFeederEmptyDisplay(FeederNo) == false)
			{
				SetFeederEmptyDisplay(FeederNo, true);
				SendEmpty(FeederNo, EMPTY_READY_TIMEOUT, _T("Ready IO TimeOut"));
			}
			SetSendEmpty(FeederNo, true);
		}
		else if (GetVisionErrorEmpty(FeederNo) > PickRetry)
		{
			if (GetFeederEmptyDisplay(FeederNo) == false)
			{
				SetFeederEmptyDisplay(FeederNo, true);
				SendEmpty(FeederNo, EMPTY_PICKRETRY_VISERROR, _T("Over Pick Retry"));
			}
			SetSendEmpty(FeederNo, true);
		}
	}
	Err = MoveZStandy(m_Gantry, StandByZ, Ratio.z);
	//Err = StartAllZAxisWaitMotion(m_Gantry, StandByZ, Ratio.z, TIME5000MS);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	return Err;
}

long CDiscard::DiscardAllAfterInserting()
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, FeederNo = 0, HeadNo = 0;
	for (long insertOrd = 0; insertOrd < GetMaxInsertOrder(); ++insertOrd)
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] DiscardAllAfterInserting GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] DiscardAllAfterInserting GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			break;
		}
		HeadNo = GetHeadNoFromInsertOrder(insertOrd + 1);
		FeederNo = GetFdNoFromInsertOrder(insertOrd + 1);
		TRACE(_T("[PWR] DiscardAllAfterInserting(%d) Hd:%d Fd:%d GetVisionErrorEmptyByFeeder:%d GetReadyTimeOutEmptyByHeadNo:%d GetVisionErrorByInsertOrd:%d\n"),
			insertOrd, HeadNo, FeederNo, GetVisionErrorEmpty(FeederNo), GetReadyTimeOutEmptyByHeadNo(HeadNo), GetVisionError(insertOrd + 1));
		if (GetReadyTimeOutEmptyByHeadNo(HeadNo) == 1 || GetVisionError(insertOrd + 1) == 1)
		{
			//gcLastPickFront->ClearHeadData(HeadNo);

			Err = DiscardOneAfterInserting(FeederNo, HeadNo);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] DiscardAllAfterInserting HeadNo:%d FeederNo:%d Err:%d\n"), HeadNo, FeederNo, Err);
				break;
			}
		}

		SuctionOne(FRONT_GANTRY, HeadNo, false);
	}
	return Err;
}

long CDiscard::GetVisionError(long insertOrd)
{
	long VisionError = 0;
	VisionError = gcStep->GetVisionError(insertOrd);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetVisionError insertOrd:%d VisionError:%d\n"), insertOrd, VisionError);
	}
	return VisionError;
}

//void CDiscard::PartDropLedOn(long CamTable)
//{
//	if (gcReadJobFile->GetPartDrop().Use == false) return;
//
//	MODULE_LED led = gcReadJobFile->GetPartDrop().led;
//
//	if (CamTable == FRONT_GANTRY)
//	{
//		gLedOn(CAM1, led.Top, led.Mid, led.Bot);
//		gLedOn(CAM2, led.Top, led.Mid, led.Bot);
//	}
//	else
//	{
//		gLedOn(RCAM1, led.Top, led.Mid, led.Bot);
//		gLedOn(RCAM2, led.Top, led.Mid, led.Bot);
//	}
//
//	TRACE(_T("[PWR] PartDropLedOn(%d) Led:%d,%d,%d\n"), CamTable, led.Top, led.Mid, led.Bot);
//}
//
//void CDiscard::PartDropProcess(long CamTable)
//{
//	if (gcReadJobFile->GetPartDrop().Use == false) return;
//
//	PowerThreadMessage* msgSend = new PowerThreadMessage();
//	ThreadId_t id;
//	msgSend->SetThreadMsg(_T("0"));
//	msgSend->SetThreadSubMsg(0, 0, 0);
//	if (gcCamDropCheck[CamTable])
//	{
//		gcCamDropCheck[CamTable]->InitDropResult();
//
//		gcCamDropCheck[CamTable]->GetId(&id);
//		msgSend->SetID(id);
//		if (gcCamDropCheck[CamTable]->PingThread(TIME1MS))
//		{
//			gcCamDropCheck[CamTable]->Event((LPVOID)msgSend);
//			TRACE(_T("[PWR] PartDropProcess(%d) Send ProcessMsg\n"), CamTable);
//		}
//	}
//}
//
//long CDiscard::PartDropGetResult(long CamTable)
//{
//	if (gcReadJobFile->GetPartDrop().Use == false) return NO_ERR;
//
//	ULONGLONG time = _time_get();
//	long result = NO_ERR;
//	long timeOut = TIME1000MS;
//
//	while (1)
//	{
//		if (_time_elapsed(time) > timeOut)
//		{
//			TRACE(_T("[PWR] PartDropGetResult(%d) TimeOut\n"), CamTable);
//
//			return FCAMERA_DROP_ERROR + CamTable;
//		}
//
//		if (gcCamDropCheck[CamTable]->GetDropResult(&result) == true)
//		{
//			TRACE(_T("[PWR] PartDropGetResult(%d) Result:%d WaitTime:%d\n"), CamTable, result, _time_elapsed(time));
//
//			return result;
//		}
//
//		ThreadSleep(TIME1MS);
//	}
//
//	return NO_ERR;
//}

//long CDiscard::PartDropCheckRear()
//{
//	long Gantry = FRONT_GANTRY;
//	long Err = NO_ERR;
//	double ratio = 1.0;
//	long Stage = gcStep->GetRecogTableBy1stInsertOrder();
//	
//	if (gcReadJobFile->GetPartDrop().Use == false) return NO_ERR;
//	if (Stage != REAR_STAGE) return NO_ERR;
//
//	TRACE(_T("[PWR] Discard PartDropCheckRear start\n"));
//
//	gPartDropLedOn(Stage);
//	STANDBY standbyPt = gcReadJobFile->GetStandyBy();
//
//	Err = MoveZStandy(Gantry, standbyPt.pt.z, ratio);
//	if (Err != NO_ERR)
//	{
//		return Err;
//	}
//
//	Point_XY ptXY;
//	ptXY.x = standbyPt.pt.x;
//	ptXY.y = standbyPt.pt.y;
//
//	Err = MoveStandByXY(Gantry, FHCAM, ptXY, ratio, TIME5000MS);
//	if (Err != NO_ERR)
//	{
//		return Err;
//	}
//
//	gPartDropProcess(Stage);
//
//	Err = gPartDropGetResult(Stage);
//	if (Err != NO_ERR)
//	{
//		Err = SendAlarm(Err, _T("Drop object on the camera"));
//		return Err;
//	}
//
//	return Err;
//}
void CDiscard::SetFeederEmptyDisplay(long FeederNo, bool set)
{
	gcStep->SetFeederEmptyDisplay(FeederNo, set);
}

bool CDiscard::GetFeederEmptyDisplay(long FeederNo)
{
	return gcStep->GetFeederEmptyDisplay(FeederNo);
}

void CDiscard::SendDiscardStatics(long FeederNo, long HeadNo)
{
	long errCode = 0;
	long OutputIO = 0;
	long InsertOrder = 0;

	if (GetProdRunMode() != REAL_RUN)
	{
		return;
	}

	OutputIO = GetSuctionIONo(m_Gantry, HeadNo);

	if (ReadOutputOne(OutputIO) == OUTOFF)
	{
		TRACE(_T("[PWR] SendDiscardStatics Skip Suction. Ord:%d Fd:%d Head:%d ErrCode:%d\n"), InsertOrder, FeederNo, HeadNo, errCode);
		return;
	}


	for (InsertOrder = 1; InsertOrder <= GetMaxInsertOrder(); InsertOrder++)
	{
		if (GetHeadNoFromInsertOrder(InsertOrder) == HeadNo)
		{
			errCode = gGetRunVisionErrorCode(m_Gantry, InsertOrder - 1);

			if (errCode == 0 || errCode == 503)
			{
				TRACE(_T("[PWR] SendDiscardStatics Skip Errocode. Ord:%d Fd:%d Head:%d ErrCode:%d\n"), InsertOrder, FeederNo, HeadNo, errCode);
				return;
			}

			TRACE(_T("[PWR] SendDiscardStatics. Ord:%d Fd:%d Head:%d ErrCode:%d\n"), InsertOrder, FeederNo, HeadNo, errCode);
			SendToRunTimeDiscardStatistics(FeederNo, GetGlobalNozzleNo(HeadNo));
		}
	}

	return;
}

long CDiscard::DiscardAllNormalMode(Ratio_XYRZ ratio)
{
	long Err = NO_ERR;
	long SuctionIONo = IO_NOUSE;

	for (long headNo = TBL_HEAD1; headNo <= TBL_HEAD6; headNo++)
	{
		SuctionIONo = GetSuctionIONo(m_Gantry, headNo);

		if (ReadOutputOne(SuctionIONo) == OUTON)
		{
			TRACE(_T("[PWR] DiscardAllNormalMode Head:%d\n"), headNo);

			Err = DiscardOneBeforePicking(headNo, ratio);
			if (Err != NO_ERR)
			{
				break;
			}
		}
	}

	return Err;
}
