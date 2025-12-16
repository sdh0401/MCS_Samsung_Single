#include "pch.h"
#include "CDecoding4.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "AxisInformation.h"
#include "CPowerHMI.h"
#include "VisionData.h"
#include "vision.h"
//#include "ErrorCode.h"
#include "CStep.h"
#include "CPick.h"
#include "CRecognition.h"
#include "CRecognitionCamera6.h"
#include "CReturnComponent.h"
#include "CDiscard.h"
#include "CPowerLog.h"
#include "CAdvancedMotionFile.h"
#include "CAutoNozzleChange.h"
#include "CRecognitionRetry.h"
#include "CBarcodeControl.h"
#include "CReadJobFile.h"
#include "CRecognitionDivide.h"
#include "CMark.h"
#include "CForming.h"
#include "CRecognitionFormingRetry.h"
#include "CFormingDRBCoil.h"
#include "CWorkConveyor.h"
#include "CInsert.h"

CDecoding4* gcDecoding4;
CDecoding4::CDecoding4()
{
	GetId(&m_id);
}

CDecoding4::~CDecoding4()
{

}

BOOL CDecoding4::OnTask()
{
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding4::OnTask Thread(0x%04X)\n", m_id);
	}
	return TRUE;
}

BOOL CDecoding4::OnTask(LPVOID lpv)
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	long Ret = 0;
	CString strMsg, strSendMsg;
	ThreadId_t id;
	strSendMsg.Format(_T("0"));
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CDecoding4 GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}
		switch (nSubMsg[1])
		{
		case HMI_CMD2ND_00:
			if (nSubMsg[2] == HMI_CMD3RD_42)
				strSendMsg = DisplayCameraRoi(strMsg);
			break;
		case HMI_CMD2ND_20:
			strSendMsg = CameraMode(strMsg);
			break;
		case HMI_CMD2ND_30:
			if(nSubMsg[2] == HMI_CMD3RD_20)
				strSendMsg = TrainingMark(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_30)
				strSendMsg = InspectionMark(strMsg);
			break;
		case HMI_CMD2ND_40:
			if (nSubMsg[2] == HMI_CMD3RD_30)
				strSendMsg = PartRecognition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_31)
				strSendMsg = PartReturn(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_32)
				strSendMsg = PartDiscard(strMsg);
			break;
		case HMI_CMD2ND_50:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = InspectionBarcode(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = SendBarcodeComPortStatus(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_06)
				strSendMsg = BarcodeComPortOpen(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_07)
				strSendMsg = BarcodeComPortClose(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_08)
				strSendMsg = ReceiveMESResult(strMsg);
			break;
		case HMI_CMD2ND_60:
			if (nSubMsg[2] == HMI_CMD3RD_01)
			{
				strSendMsg = DropCheckImageSave(strMsg);
			}
			else if (nSubMsg[2] == HMI_CMD3RD_30)
			{
				strSendMsg = DropCheckProcess(strMsg);
			}
			break;
		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 ReturnMsg(%s)\n", (LPCTSTR)strSendMsg);
		}
		if (strSendMsg.GetLength() > 0)
		{
			PowerThreadMessage* msgSend = new PowerThreadMessage();
			msgSend->SetThreadMsg(strSendMsg);
			msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
			if (gcCPowerHMI)
			{
				gcCPowerHMI->GetId(&id);
				msgSend->SetID(id);
				if (gcCPowerHMI->PingThread(TIME1MS))
				{
					gcCPowerHMI->Event((LPVOID)msgSend);
				}
			}
		}
		delete msgReceived;
	}
	return TRUE;
}

CString CDecoding4::DisplayCameraRoi(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 DisplayCameraRoi TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue;
		int iValue[10];
		WindowSize Win;
		long CameraNo = FHCAM;
		ZeroMemory(&iValue, sizeof(iValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			CameraNo = iValue[0] - 1;
			Win.x1 = iValue[1];
			Win.y1 = iValue[2];
			Win.x2 = iValue[3];
			Win.y2 = iValue[4];
			gShowRoiBase(gGetVisionTable(CameraNo), Win);
			strRetMsg.Format(_T("0"));
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding4::CameraMode(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 CameraMode TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue;
		int iValue[10];
		long CamNo = FHCAM, Mode = 0;
		long LedTop, LedMid, LedBot;
		ZeroMemory(&iValue, sizeof(iValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			gLedAllOff();
			gLaserAllOff();
			//if(iValue[0] == 4)
			//	CamNo = FHCAM;
			//else
			if (iValue[0] > 0)
			{
				CamNo = iValue[0] - 1;
			}
			Mode = iValue[1];
			LedTop = iValue[2];
			LedMid = iValue[3];
			LedBot = iValue[4];
			gLedOn(CamNo, LedTop, LedMid, LedBot);
			if (gcPowerLog->IsShowCommunicationLog() == true)
			{
				TRACE("[PWR] CDecoding4 CamNo:%d Mode:%d LED Top:%d Mid:%d Bot:%d\n", CamNo, Mode, LedTop, LedMid, LedBot);
			}
			if (Mode == 0)
				gLiveOn(CamNo);
			else
				gLiveOff(CamNo);
			strRetMsg.Format(_T("0"));
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding4::TrainingMark(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 CameraMode TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue;
		int iValue[10];
		long CamNo = FHCAM;
		long MarkNo, MarkType, MarkColor, Area = 0;
		long LedTop, LedMid;
		double RoiRatioW = 0.5, RoiRatioH = 0.5;
		WindowSize Win;
		Point_XYRE Res;
		ZeroMemory(&Win, sizeof(Win));
		ZeroMemory(&Res, sizeof(Res));
		ZeroMemory(&iValue, sizeof(iValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			gLedAllOff();
			gLaserAllOff();
			CamNo = iValue[0] - 1;
			MarkType = iValue[1];
			MarkColor = iValue[2];
			LedTop = iValue[3];
			LedMid = iValue[4];
			Win.x1 = iValue[5];
			Win.y1 = iValue[6];
			Win.x2 = iValue[7];
			Win.y2 = iValue[8];
			TRACE(_T("[PWR] CamNo:%d Mark Type:%d Color:%d LED Top:%d Mid:%d Win x1:%d y1:%d x2:%d y2:%d\n"),
				CamNo, MarkType, MarkColor, LedTop, LedMid, Win.x1, Win.y1, Win.x2, Win.y2);
			gLedOn(CamNo, LedTop, LedMid, 0);
			gLiveOn(CamNo);
			MarkNo = TMPMARK;
			markInitialize(MarkNo, RoiRatioW, RoiRatioH, MarkType, MarkColor, Win);
			markTraining(CamNo, MarkNo, 0, &Res);
			Area = gGetMarkArea(FRONT_VISION, MarkNo);
			strRetMsg.Format(_T("0,%d"), Area);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0"));
	}
	return strRetMsg;
}

CString CDecoding4::InspectionMark(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 CameraMode TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue;
		int iValue[10];
		long CamNo = FHCAM, Err = 0;
		long MarkNo;
		//long LedTop, LedMid;
		Point_XYRE Res;
		ZeroMemory(&Res, sizeof(Res));
		ZeroMemory(&iValue, sizeof(iValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			if (iValue[1] > 0)
			{
				CamNo = iValue[0] - 1;
				MarkNo = iValue[1] - 1;
				TRACE(_T("[PWR] CamNo:%d MarkNo:%d\n"), CamNo, MarkNo);
				Res = gCatchMark(CamNo, MarkNo);
				Err = gGetVisionErrorCode(gGetVisionTable(CamNo));
				TRACE(_T("[PWR] Catch Err:%d ResXYR,%.3f,%.3f,%.3f\n"), Err, Res.x, Res.y, Res.r);
				strRetMsg.Format(_T("%d,%.3f,%.3f,%.3f"), Err, Res.x, Res.y, Res.r);
			}
			else
			{
				strRetMsg.Format(_T("%d"), INVALID_MARKNO);
			}
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0.0,0.0,0.0"));
	}
	return strRetMsg;
}

CString CDecoding4::PartRecognition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 PartRecognition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		Ratio_XYRZ Ratio;
		Point_XYRZ PickupOffset, RecogOffset, VisionResult;
		Point_XYRE Res;
		ZeroMemory(&Res, sizeof(Res));
		ZeroMemory(&Ratio, sizeof(Ratio));
		ZeroMemory(&RecogOffset, sizeof(RecogOffset));
		ZeroMemory(&PickupOffset, sizeof(PickupOffset));		
		ZeroMemory(&VisionResult, sizeof(VisionResult));
		CString strValue;
        constexpr int ARRAY_SIZE = 50;
		int iValue[ARRAY_SIZE];
		double dValue[ARRAY_SIZE];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
                if (dCnt < ARRAY_SIZE)
                {
                    dValue[dCnt] = cTokenizer->GetDouble(i);
                }
				dCnt++;
			}
			else
			{
                if (iCnt < ARRAY_SIZE)
                {
                    iValue[iCnt] = cTokenizer->GetInt(i);
                }
				iCnt++;
			}
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Gantry = FRONT_GANTRY;
			long Ord = 1;
			long FeederNo = iValue[0];
			long CamNo = iValue[1];
			long HeadNo = iValue[2];
			long OnlyRecog = iValue[3];
			long RecogTable = CamNo < RCAM1 ? FRONT_STAGE : REAR_STAGE;
			TRACE(_T("[PWR] FeederNo:%d Cam:%d HeadNo:%d OnlyRecog:%d RecogTable:%d\n"), FeederNo, CamNo, HeadNo, OnlyRecog, RecogTable);
			Ratio.xy = (double)(iValue[4] / 100.0);
			Ratio.z = (double)(iValue[5] / 100.0);
			Ratio.r = (double)(iValue[6] / 100.0);
			TRACE(_T("[PWR] RatioXYRZ,%.1f,%.1f,%.1f\n"), Ratio.xy, Ratio.r, Ratio.z);
			double Length = dValue[0];
			double Width = dValue[1];
			double Height = dValue[2];
			double LeadHeight = dValue[3];
			TRACE(_T("[PWR] Component Length,Width,Height,%.3f,%.3f,%.3f Lead(%.3f)\n"), Length, Width, Height, LeadHeight);
			NOZZLE Nozzle;
			Nozzle.No = iValue[7];
			Nozzle.Type = iValue[8];
			Nozzle.Empty = iValue[9];
			Nozzle.EmptyDiff = iValue[10];
			Nozzle.Exist = iValue[11];
			Nozzle.ExistDiff = iValue[12];
			Nozzle.TipHeight = dValue[4];
			Nozzle.PusherHeight = dValue[5];
			TRACE(_T("[PWR] Nozzle No,Type(%d,%d) TipHeight,PusherHeight,%.3f,%.3f Empty,Diff,Exist,Diff(%d,%d,%d,%d)\n"),
				Nozzle.No, Nozzle.Type,
				Nozzle.TipHeight, Nozzle.PusherHeight,
				Nozzle.Empty, Nozzle.EmptyDiff,
				Nozzle.Exist, Nozzle.ExistDiff);
			long ReadyIO = iValue[13];
			long ReleaseIO = iValue[14];
			long TimeOut = iValue[15];
			long ReadyWaitDelay = iValue[16];
			long PickupDelay = iValue[17];
			TRACE(_T("[PWR] IO Ready(%d) Release(%d) TimeOut(%d) ReadyWaityDelay(%d) PickupDelay(%d)\n"),
				ReadyIO, ReleaseIO, TimeOut, ReadyWaitDelay, PickupDelay);
			MODULE_LED Led;
			Led.Top = iValue[18];
			Led.Mid = iValue[19];
			Led.Bot = iValue[20];
			TRACE(_T("[PWR] Led Top(%d) Mid(%d) Bot(%d)\n"), Led.Top, Led.Mid, Led.Bot);
			PickupOffset.x = dValue[6];
			PickupOffset.y = dValue[7];
			PickupOffset.r = dValue[8];
			PickupOffset.z = dValue[9];
			TRACE(_T("[PWR] PickupXYRZ,%.3f,%.3f,%.3f,%.3f\n"), PickupOffset.x, PickupOffset.y, PickupOffset.r, PickupOffset.z);
			double VAAngle = dValue[10];
			RecogOffset.x = dValue[11];
			RecogOffset.y = dValue[12];
			RecogOffset.r = dValue[13];
			RecogOffset.z = dValue[14];
			TRACE(_T("[PWR] RecogAngle(%.3f) Offset,%.3f,%.3f,%.3f,%.3f\n"), VAAngle, RecogOffset.x, RecogOffset.y, RecogOffset.r, RecogOffset.z);
			long UseVisionCompensation = iValue[21];
			VisionResult.x = dValue[15];
			VisionResult.y = dValue[16];
			VisionResult.r = dValue[17];
			VisionResult.z = 0.000;
			TRACE(_T("[PWR] UseVisionCompensation(%d) VisionResult,%.3f,%.3f,%.3f,%.3f\n"), UseVisionCompensation, VisionResult.x, VisionResult.y, VisionResult.r, VisionResult.z);
			double VAOffsetHeight = dValue[18];
			TRACE(_T("[PWR] VAOffsetHeight:%.3f\n"), VAOffsetHeight);

			long FeederType = iValue[22];
			long ReadyIOType = iValue[23];
			long PalletNo = iValue[24];
			TRACE(_T("[PWR] FeederType:%d ReadyIOType:%d Pallet:%d\n"), FeederType, ReadyIOType, PalletNo);

			ROTATIONHEIGHTOFFSET RecogRotateZOffset;
			RecogRotateZOffset.Use = iValue[25];
			RecogRotateZOffset.ZOffset = dValue[19];
			TRACE(_T("[PWR] Recog Rotation Height Use:%d Height:%.3f\n"), RecogRotateZOffset.Use, RecogRotateZOffset.ZOffset);

			long LaserControl = iValue[26];
			TRACE(_T("[PWR] LaserControl Use:%d\n"), LaserControl);

			long CatchDelay = iValue[27];
			TRACE(_T("[PWR] CatchDelay:%d\n"), CatchDelay);

			FORMING_COMPONENT Forming;
			ZeroMemory(&Forming, sizeof(Forming));
			Forming.Use = iValue[28];
			Forming.FeederNo = FeederNo;
			Forming.HeadNo = HeadNo;
			Forming.BodyHeight = Height;
			Forming.LeadHeight = LeadHeight;
			Forming.RecognitionAngle = VAAngle;
			Forming.Ratio = Ratio;
			Forming.Delay = TIME300MS;
			Forming.InsertCase.Min = dValue[20];
			Forming.InsertCase.Max = dValue[21];
			Forming.CaseNo[0].Min = dValue[22];
			Forming.CaseNo[0].Max = dValue[23];
			Forming.CaseNo[1].Min = dValue[24];
			Forming.CaseNo[1].Max = dValue[25];
			Forming.ptNo[0].x = dValue[26];
			Forming.ptNo[0].y = dValue[27];
			Forming.ptNo[0].r = dValue[28];
			Forming.ptNo[0].z = dValue[29];
			Forming.ptNo[1].x = dValue[30];
			Forming.ptNo[1].y = dValue[31];
			Forming.ptNo[1].r = dValue[32];
			Forming.ptNo[1].z = dValue[33];

			TRACE(_T("[PWR] FormingUse:%d FdNo:%d InsertCase:%.3f~%.3f Case1:%.3f~%.3f Case2:%.3f~%.3f Pt1:%.3f,%.3f,%.3f,%.3f Pt2:%.3f,%.3f,%.3f,%.3f\n"),
				Forming.Use, Forming.FeederNo,
				Forming.InsertCase.Min, Forming.InsertCase.Max,
				Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
				Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z);

			long manualInsertUse = 0;
			long manualInsertNo = 0;

			manualInsertUse = iValue[29];
			//manualInsertNo = iValue[30];

			gcReadJobFile->AddBlockSequence();

			long i = 0;
			while (i < 5)
			{
				manualInsertNo = gcReadJobFile->GetInsertNo(i, iValue[30]);
				if (manualInsertNo != 0)
					break;
				i++;
			}

			Err = gcAdvancedMotionFile->ReadFile();

			TRACE(_T("[PWR] AdvancedMotionFile Err(%d) \n"), Err);

			gLedAllOff();
			gLaserAllOff();
			if (IsAliveCStep(FRONT_GANTRY) == true)
			{
				DeleteCStep(FRONT_GANTRY);
			}
			CreateCStep(FRONT_GANTRY);

			TRACE(_T("[PWR] Step1\n"));
			gcStep->SetMaxPickOrder(Ord);
			TRACE(_T("[PWR] Step2\n"));
			gcStep->SetPickupHeadNo(Ord, HeadNo);
			TRACE(_T("[PWR] Step3\n"));
			gcStep->SetFdNoFromPickOrder(Ord, FeederNo);
			TRACE(_T("[PWR] Step4\n"));
			gcStep->SetPickDelayFromFdNo(FeederNo, PickupDelay);
			TRACE(_T("[PWR] Step5\n"));
			gcStep->SetReadyNoFromFeederNo(FeederNo, ReadyIO);
			TRACE(_T("[PWR] Step6\n"));
			gcStep->SetReleaseNoFromFeederNo(FeederNo, ReleaseIO);
			TRACE(_T("[PWR] Step7\n"));
			gcStep->SetReadyTimeOutFromFeederNo(FeederNo, TimeOut);
			TRACE(_T("[PWR] Step8\n"));
			gcStep->SetReadyWaitDelayFromFeederNo(FeederNo, ReadyWaitDelay);
			TRACE(_T("[PWR] Step9\n"));
			gcStep->SetRatioFromFdNo(FeederNo, Ratio);
			TRACE(_T("[PWR] Step10\n"));
			gcStep->SetPickOffsetFromFdNo(FeederNo, PickupOffset);
			TRACE(_T("[PWR] Step11\n"));
			gcStep->SetFdNoFromInsertOrder(Ord, FeederNo);
			TRACE(_T("[PWR] Step12\n"));
			gcStep->SetLed(FeederNo, Led);
			TRACE(_T("[PWR] Step13\n"));
			gcStep->SetMaxInsertOrder(Ord);
			TRACE(_T("[PWR] Step14\n"));
			gcStep->SetHeadNoFromInsertOrder(Ord, HeadNo);
			TRACE(_T("[PWR] Step15\n"));
			gcStep->SetVAAngleFromFdNo(FeederNo, VAAngle);
			TRACE(_T("[PWR] Step16\n"));
			gcStep->SetComponentHeightFromFdNo(FeederNo, Height);
			TRACE(_T("[PWR] Step17\n"));
			gcStep->SetComponentLeadHeightFromFdNo(FeederNo, LeadHeight);
			TRACE(_T("[PWR] Step18\n"));
			gcStep->SetManualCompensationUse(UseVisionCompensation);
			TRACE(_T("[PWR] Step19\n"));
			gcStep->SetManualVisionResult(VisionResult);
			TRACE(_T("[PWR] Step20\n"));
			gcStep->SetRecognitionTable(Ord, RecogTable);
			TRACE(_T("[PWR] Step21\n"));
			gcStep->SetInsertNoFromInsertOrder(Ord, Ord);
			TRACE(_T("[PWR] Step22\n"));
			gcStep->SetVAOffsetHeight(FeederNo, VAOffsetHeight);
			TRACE(_T("[PWR] Step23\n"));
			gcStep->SetFeederReadyIOType(FeederNo, ReadyIOType);
			TRACE(_T("[PWR] Step24\n"));

			gcStep->SetLaserControl(FeederNo, LaserControl);
			TRACE(_T("[PWR] Step25\n"));

			gcStep->SetCatchDelay(FeederNo, CatchDelay);
			TRACE(_T("[PWR] Step26\n"));

			gcStep->SetFeederType(FeederNo, FeederType);
			if (IsLabelTypeFeeder(FeederType) == true)
			{
				PICK_LEVEL_CHECK data;
				data.Use = true;
				gcStep->SetPickLevelCheck(FeederNo, data);
			}

			double MaxComponent = gcReadJobFile->GetPcb().MaxComponentHeight;
			gcStep->SetMaxComponentHeight(MaxComponent);
			TRACE(_T("[PWR] Step SetDivideInspect\n"));
			DIVIDE_INSPECT Divide = gcReadJobFile->GetDivideInspect(FeederNo);
			gcStep->SetDivideInspect(FeederNo, Divide);

			TRAY_INFO TempTray;
			TempTray.pt[0].x = 0.000;
			TempTray.pt[0].y = 0.000;
			gcStep->SetTray(FeederNo, TempTray);
			gcStep->SetTrayMaxPocket(FeederNo, 1);
			gcStep->SetTrayNowPocket(FeederNo, 0);

			//RETRY_LED RetryLed;
			//RetryLed.MaxRetry = MAXRETRYCNT_5;
			//for (long index = 0; index < MAXRETRYCNT_5; ++index)
			//{
			//	RetryLed.Led[index].Top = Led.Top;
			//	RetryLed.Led[index].Mid = Led.Mid;
			//	RetryLed.Led[index].Bot = Led.Bot;
			//}
			//gcStep->SetRetryLed(FeederNo, RetryLed);

			gcStep->SetForming(FeederNo, Forming);

			gcStep->SetMinRatio(Ratio);

			long PackageIndex = gcReadJobFile->GetPackgeIndexFromFeederNo(FeederNo);
			if (PackageIndex > 0)
			{
				PARTTORQUELIMIT PartTorque = gcReadJobFile->GetPackage(PackageIndex).PartTorqueLimit;
				gcStep->SetPartTorqueLimit(FeederNo, PartTorque);

				TwoStepMotion TwoStepPick = gcReadJobFile->GetPackage(PackageIndex).TwoStepPick;
				gcStep->SetTwoStepPick(FeederNo, TwoStepPick);
			}

			SetMachineState(STATE_RUN);
			SetGlobalStatusError(false);
			gcDiscard = new CDiscard(Gantry);
			gcPick = new CPick(Gantry);
			TRACE(_T("[PWR] Step26\n"));
			gcPick->SetProdRunMode(RUN_REAL);
			gcDiscard->SetProdRunMode(RUN_DRY);
			TRACE(_T("[PWR] Step27\n"));
			gcRecognition = new CRecognition(Gantry);
			gcRecognitionCamera6 = new CRecognitionCamera6(Gantry);
			gcRecognitionDivide = new CRecognitionDivide(Gantry);
			TRACE(_T("[PWR] Step Create RecognitionDivide\n"));

			//gcRecognitionRetry = new CRecognitionRetry(Gantry);
			TRACE(_T("[PWR] Step28\n"));
			gcRecognition->SetProdRunMode(RUN_REAL);
			gcRecognitionCamera6->SetProdRunMode(RUN_REAL);
			gcRecognitionDivide->SetProdRunMode(RUN_REAL);
			gcLastPickFront->SetAllHeadDataEnable(false);

			CForming* FormingRun = new CForming(Gantry);
			CFormingDRBCoil* FormingDRBCoilRun = new CFormingDRBCoil(Gantry);

			TRACE(_T("[PWR] Step30\n"));
			CRecognitionFormingRetry* RecognitionFormingRetry = new CRecognitionFormingRetry(Gantry);

			if (OnlyRecog == 0)
			{
				Ratio_XYRZ ratioDiscard;
				ratioDiscard.xy = ratioDiscard.r = ratioDiscard.z = 0.5;

				Err = gcDiscard->DiscardAllBeforePicking(ratioDiscard);
				if (Err == NO_ERR)
				{
					TRACE(_T("[PWR] HeadNo:%02d NozzleNo:%d CurNozzleNo:%d\n"), HeadNo, Nozzle.No, GetGlobalNozzleNo(HeadNo));
					if (Nozzle.No != GetGlobalNozzleNo(HeadNo))
					{
						Err = gcCAutoNozzleChange->NozzleRelease(Gantry, HeadNo, false);
						if (Err == NO_ERR)
						{
							Err = gcCAutoNozzleChange->NozzleHold(Gantry, HeadNo, Nozzle.No, false);
						}
					}
					if (Err == NO_ERR)
					{
						MachineReferenceMark MachineRefMark = GetGlobalMachineReferenceMark();
						if (MachineRefMark.Use == 1)
						{
							CMark* MarkFront = new CMark(FRONT_GANTRY);
							Ratio_XYRZ ratio;
							ratio.xy = ratio.z = ratio.r = 0.7;

							MarkFront->MachineReferenceMarkCheckingAutoCompen(MachineRefMark, ratio);
							delete MarkFront;
						}

						SendCameraRecognitionOffset(FRONT_GANTRY);
						ThreadSleep(TIME100MS);

						SetFeederProdRunMode(RUN_DRY);
						TRACE(_T("[PWR] Step29\n"));
						StartFeeder(FeederNo, ReadyIO, ReleaseIO);
						TRACE(_T("[PWR] Step30\n"));
						SetInfoFeeder(FeederNo, ReadyIO, ReleaseIO);
						TRACE(_T("[PWR] Step31\n"));
						ThreadSleep(TIME10MS);
						TRACE(_T("[PWR] Step32\n"));
						RunFeeder(FeederNo);
						TRACE(_T("[PWR] Step33\n"));
						ThreadSleep(TIME10MS);
						TRACE(_T("[PWR] Step34\n"));
						Err = gcPick->Run(true, true);
						TRACE(_T("[PWR] Step35\n"));
					}
				}
			}
			if (Err == NO_ERR)
			{
				TRACE(_T("[PWR] Step36\n"));
				//if(RecogTable == FRONT_STAGE)
				//	Err = gcRecognitionCamera6->Run(true);
				//else
				//	Err = gcRecognition->Run(true);

				SendCameraRecognitionOffset(FRONT_GANTRY);
				ThreadSleep(TIME100MS);


				if (gcStep->GetDivideInspect(FeederNo).Use == true)
				{
					Err = gcRecognitionDivide->Run(true);
				}
				else
				{
					if (GetCameraCount(RecogTable) == CAMERA_COUNT_2)
						Err = gcRecognition->Run(true);
					else
						Err = gcRecognitionCamera6->Run(true);
				}

				if (Err == NO_ERR)
				{
					TRACE(_T("[PWR] Step37\n"));
					Res = gGetRunVisionResult(Gantry, Ord - 1);
					TRACE(_T("[PWR] Step38\n"));
					Err = gGetVisionErrorCode(Gantry);
					TRACE(_T("[PWR] Step39\n"));
					if (gGetVisionTimeOut() == true)
					{
						Err = 999;
					}

					if ((Forming.Use == 1) && (Res.exe == 1 || GetSimulationForming() == true))
					{
						TRACE(_T("[PWR] Step42\n"));
						if (GetSimulationForming() == true)
						{
							Res.exe = 1;
							Res.x = 0.1;
							Res.y = 0.1;
							Res.r = 1.000;
							Forming.VisionResult = Res;
							Forming.VisionResultPitch = 26.0;
						}
						else
						{
							Forming.VisionResult = Res;
							Forming.VisionResultPitch = gGetVisionPartPitchResult(Gantry);
						}

						if (GetFormingDRBCoilUse() == true)
						{
							Err = FormingDRBCoilRun->Run(0, Forming, FormingType::Forming1st);
							if (Err == NO_ERR)
							{
								Err = FormingDRBCoilRun->Run(0, Forming, FormingType::Forming2nd);
							}
						}
						else
						{
							Err = FormingRun->Run(0, Forming);

						}
						TRACE(_T("[PWR] Step43\n"));
						if (Err == NO_ERR)
						{
							Err = RecognitionFormingRetry->Run(true);
							if (Err == NO_ERR)
							{
								TRACE(_T("[PWR] Step44\n"));
								Res = gGetRunVisionResult(Gantry, Ord - 1);
								TRACE(_T("[PWR] Step45\n"));
								Err = gGetVisionErrorCode(Gantry);
								TRACE(_T("[PWR] Step46\n"));
								if (gGetVisionTimeOut() == true)
								{
									Err = 999;
								}
							}
						}
						if (Err == NO_NEED_FORMING)
						{
							Err = NO_ERR;
						}
					}


					//if ((Err != NO_ERR) && fabs(VisionResult.x) < 0.001 && fabs(VisionResult.y) < 0.001 && fabs(VisionResult.r) < 0.001) // Compensation 인 경우에는 Skip
					//{
					//	gcRecognitionRetry->Run(true);
					//	TRACE(_T("[PWR] Step40\n"));
					//	Res = gGetRunVisionResult(Gantry, Ord - 1);
					//	TRACE(_T("[PWR] Step41\n"));
					//	Err = gGetVisionErrorCode(Gantry);
					//	TRACE(_T("[PWR] Step42\n"));
					//	if (gGetVisionTimeOut() == true)
					//	{
					//		Err = 999;
					//	}
					//}
				}

				//if (GetFirstPickup() == 1)
				//{
				//	manualInsertUse = 1;
				//	manualInsertNo = 1;
				//}

				bool manualInsertOK = false;
				if (Err == NO_ERR && manualInsertUse > 0 && manualInsertNo > 0 && IsExistSet(WORK1_CONV) == true && Res.exe == 1)
				{
					manualInsertOK = true;
				}
				else
				{
					TRACE(_T("[PWR] Step47 InsertNo:%d PCB Exist:%d VisRes:%d\n"), manualInsertNo, IsExistSet(WORK1_CONV), Res.exe);
				}

				if (manualInsertOK == true)
				{
					double PCBThickness = 0.0, RatioPusherZ = 0.5;
					PCBThickness = gcReadJobFile->GetPcb().Thickness;
					if (gcWorkConveyor->GetPusherPlateZControlType() == PusherZType::Motor)
					{
						Err = StartPosWaitMotion(GetPusherZName(FRONT_GANTRY), RatioPusherZ, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
						if (Err == NO_ERR)
						{
							WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
						}
					}

					gcStep->SetUseFromInsertNo(manualInsertNo, 1);

					if (PackageIndex > 0)
					{
						TwoStepMotion TwoStepInsert = gcReadJobFile->GetPackage(PackageIndex).TwoStepInsert;
						gcStep->SetTwoStepInsert(FeederNo, TwoStepInsert);

						long blowDelay = gcReadJobFile->GetPackage(PackageIndex).BlowDelay;
						gcStep->SetBlowDelayFromFdNo(FeederNo, blowDelay);

						long releaseDelay = gcReadJobFile->GetPackage(PackageIndex).ReleaseDelay;
						gcStep->SetReleaseDelayFromFdNo(FeederNo, releaseDelay);

						double zOffset = gcReadJobFile->GetPackage(PackageIndex).InsertZOffset;
						gcStep->SetInsertZOffset(FeederNo, zOffset);

						gcStep->SetInsertNoFromInsertOrder(Ord, manualInsertNo);

						INSERT Insert = gcReadJobFile->GetInsert(manualInsertNo);
						gcStep->SetInsertPoint(manualInsertNo, Insert.pt);

						long AvoidCount = gcReadJobFile->GetAvoidMotion(manualInsertNo).Count;
						gcStep->SetAvoidCount(manualInsertNo, AvoidCount);
					}

					ORIGIN origin = gcReadJobFile->GetOrigin();
					gcStep->SetOrigin(origin);

					CMark* objMark = new CMark(Gantry);
					CInsert* objInsert = new CInsert(Gantry);

					Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio.z);
					if (Err == NO_ERR)
					{
						Err = objMark->FiducialMarkChecking();
					}

					if (Err == NO_ERR)
					{
						objInsert->SetProdRunMode(RUN_REAL);
						Err = objInsert->Run(true);
					}

					delete objMark;
					delete objInsert;
				}
			}
			if (OnlyRecog == 0)
			{
				StopFeeder(FeederNo);
			}
			SetMachineState(STATE_IDLE);
			//DeleteCStep(FRONT_GANTRY);
			delete gcDiscard;
			delete gcPick;
			delete gcRecognition;
			delete gcRecognitionCamera6;
			delete gcRecognitionDivide;
			//delete gcRecognitionRetry;
			delete FormingRun;
			delete RecognitionFormingRetry;
			delete FormingDRBCoilRun;
			strRetMsg.Format(_T("%d,%.3f,%.3f,%.3f,%d,%d"), Err, Res.x, Res.y, Res.r, Nozzle.Empty, Nozzle.Exist);
		}
		else
		{
			strRetMsg.Format(_T("%d,0.0,0.0,0.0,0,0"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0.0,0.0,0.0,0,0"));
	}
	return strRetMsg;
}

CString CDecoding4::PartReturn(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 PartReturn TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		Ratio_XYRZ Ratio;
		Point_XYRZ PickupOffset;
		ZeroMemory(&Ratio, sizeof(Ratio));
		ZeroMemory(&PickupOffset, sizeof(PickupOffset));
		CString strValue;
		int iValue[20];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Gantry = FRONT_GANTRY;
			long Ord = 1;
			long FeederNo = iValue[0];
			long HeadNo = iValue[1];
			TRACE(_T("[PWR] FeederNo:%d HeadNo:%d\n"), FeederNo, HeadNo);
			Ratio.xy = (double)(iValue[2] / 100.0);
			Ratio.r = (double)(iValue[3] / 100.0);
			Ratio.z = (double)(iValue[4] / 100.0);
			TRACE(_T("[PWR] RatioXYRZ,%.1f,%.1f,%.1f\n"), Ratio.xy, Ratio.r, Ratio.z);
			long PickupDelay = iValue[5];
			TRACE(_T("[PWR] PickupDelay(%d)\n"), PickupDelay);
			PickupOffset.x = dValue[0];
			PickupOffset.y = dValue[1];
			PickupOffset.r = dValue[2];
			PickupOffset.z = dValue[3];
			TRACE(_T("[PWR] PickupXYRZ,%.3f,%.3f,%.3f,%.3f\n"), PickupOffset.x, PickupOffset.y, PickupOffset.r, PickupOffset.z);
			NOZZLE Nozzle;
			Nozzle.No = iValue[6];
			Nozzle.Type = iValue[7];
			Nozzle.Empty = iValue[8];
			Nozzle.EmptyDiff = iValue[9];
			Nozzle.Exist = iValue[10];
			Nozzle.ExistDiff = iValue[11];
			Nozzle.TipHeight = dValue[4];
			Nozzle.PusherHeight = dValue[5];
			TRACE(_T("[PWR] Nozzle No,Type(%d,%d) TipHeight,PusherHeight,%.3f,%.3f Empty,Diff,Exist,Diff(%d,%d,%d,%d)\n"),
				Nozzle.No, Nozzle.Type,
				Nozzle.TipHeight, Nozzle.PusherHeight,
				Nozzle.Empty, Nozzle.EmptyDiff,
				Nozzle.Exist, Nozzle.ExistDiff);

			if (IsAliveCStep(FRONT_GANTRY) == true)
			{
				DeleteCStep(FRONT_GANTRY);
			}
			CreateCStep(FRONT_GANTRY);
			gcLastPickFront->SetAllHeadDataEnable(false);

			gLedAllOff();
			gLaserAllOff();

			TRACE(_T("[PWR] Step1\n"));
			gcStep->SetMaxReturnComponentOrder(Ord);
			TRACE(_T("[PWR] Step2\n"));
			gcStep->SetReturnComponentHeadNo(Ord, HeadNo);
			TRACE(_T("[PWR] Step3\n"));
			gcStep->SetFdNoFromReturnOrder(Ord, FeederNo);
			TRACE(_T("[PWR] Step4\n"));
			gcStep->SetPickOffsetFromFdNo(FeederNo, PickupOffset);
			TRACE(_T("[PWR] Step5\n"));
			gcStep->SetRatioFromFdNo(FeederNo, Ratio);
			TRACE(_T("[PWR] Step6\n"));
			gcStep->SetPickDelayFromFdNo(FeederNo, PickupDelay);
			TRACE(_T("[PWR] Step7\n"));
			gcReturnComponent = new CReturnComponent();
			TRACE(_T("[PWR] Step8\n"));
			Err = gcReturnComponent->Run(Gantry);
			TRACE(_T("[PWR] Step9\n"));
			//DeleteCStep(FRONT_GANTRY);
			delete gcReturnComponent;
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d,0.0,0.0,0.0,0,0"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0.0,0.0,0.0"));
	}
	return strRetMsg;
}

CString CDecoding4::PartDiscard(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 PartDiscard TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		Ratio_XYRZ Ratio;
		Point_XYRZ DiscardPos;
		ZeroMemory(&Ratio, sizeof(Ratio));
		ZeroMemory(&DiscardPos, sizeof(DiscardPos));
		CString strValue;
		int iValue[20];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Gantry = FRONT_GANTRY;
			long Ord = 1;
			long FeederNo = iValue[0];
			long HeadNo = iValue[1];
			TRACE(_T("[PWR] FeederNo:%d HeadNo:%d\n"), FeederNo, HeadNo);
			Ratio.xy = (double)(iValue[2] / 100.0);
			Ratio.r = (double)(iValue[3] / 100.0);
			Ratio.z = (double)(iValue[4] / 100.0);
			TRACE(_T("[PWR] RatioXYRZ,%.1f,%.1f,%.1f\n"), Ratio.xy, Ratio.r, Ratio.z);
			DiscardPos.x = dValue[0];
			DiscardPos.y = dValue[1];
			DiscardPos.r = dValue[2];
			DiscardPos.z = dValue[3];
			TRACE(_T("[PWR] DiscardPosXYRZ,%.3f,%.3f,%.3f,%.3f\n"), DiscardPos.x, DiscardPos.y, DiscardPos.r, DiscardPos.z);
			NOZZLE Nozzle;
			Nozzle.No = iValue[6];
			Nozzle.Type = iValue[7];
			Nozzle.Empty = iValue[8];
			Nozzle.EmptyDiff = iValue[9];
			Nozzle.Exist = iValue[10];
			Nozzle.ExistDiff = iValue[11];
			Nozzle.TipHeight = dValue[4];
			Nozzle.PusherHeight = dValue[5];
			TRACE(_T("[PWR] Nozzle No,Type(%d,%d) TipHeight,PusherHeight,%.3f,%.3f Empty,Diff,Exist,Diff(%d,%d,%d,%d)\n"),
				Nozzle.No, Nozzle.Type,
				Nozzle.TipHeight, Nozzle.PusherHeight,
				Nozzle.Empty, Nozzle.EmptyDiff,
				Nozzle.Exist, Nozzle.ExistDiff);

			if (IsAliveCStep(FRONT_GANTRY) == true)
			{
				DeleteCStep(FRONT_GANTRY);
			}
			CreateCStep(FRONT_GANTRY);
			gcLastPickFront->SetAllHeadDataEnable(false);

			gLedAllOff();
			gLaserAllOff();

			TRACE(_T("[PWR] Step1\n"));
			gcStep->SetDiscardPoint(FeederNo, DiscardPos);
			TRACE(_T("[PWR] Step2\n"));
			gcStep->SetRatioFromFdNo(FeederNo, Ratio);
			TRACE(_T("[PWR] Step3\n"));
			gcStep->SetPickRetryFromFdNo(FeederNo, DEFAULT_PICKRETRY);
			TRACE(_T("[PWR] Step4\n"));
			gcDiscard = new CDiscard(Gantry);
			TRACE(_T("[PWR] Step5\n"));
			gcDiscard->SetProdRunMode(RUN_DRY);
			TRACE(_T("[PWR] Step6\n"));
			Err = gcDiscard->DiscardOneAfterAlignChecking(FeederNo, HeadNo);
			TRACE(_T("[PWR] Step7\n"));
			//DeleteCStep(FRONT_GANTRY);
			delete gcDiscard;
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d,0.0,0.0,0.0,0,0"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0.0,0.0,0.0"));
	}
	return strRetMsg;
}


CString CDecoding4::DropCheckImageSave(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 DropCheckImageSave TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0;
		CString strValue;
		int iValue[10];
		long CamNo[MAXCAMNO] = { 0, };
		long Err = NO_ERR;

		ZeroMemory(&iValue, sizeof(iValue));

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}

		if (GetRunMode() == NORMAL_MODE)
		{
			iValue[0]--;

			if (iValue[0] == CAM1 || iValue[0] == RCAM1)
			{
				CamNo[0] = iValue[0];
				Err = gDropCheckSaveImage(CamNo, 1);

			}
			else if (iValue[0] == CAM2 || iValue[0] == RCAM2)
			{
				CamNo[0] = iValue[0];
				Err = gDropCheckSaveImage(CamNo, 1);
			}
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding4::DropCheckProcess(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 DropCheckProcess TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0;
		CString strValue;
		int iValue[10];
		long CamNo[MAXCAMNO] = { 0, };
		long Result[MAXCAMNO] = { 1, };
		long Err = NO_ERR;

		ZeroMemory(&iValue, sizeof(iValue));

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}

		if (GetRunMode() == NORMAL_MODE)
		{
			iValue[0]--;

			if (iValue[0] == CAM1 || iValue[0] == RCAM1)
			{
				CamNo[0] = iValue[0];
				Err = gDropCheckProcess(CamNo, 1, Result);
				if (Result[0] != 0)
				{
					Err = FCAMERA_DROP_ERROR;
				}
				else
				{
					Err = NO_ERR;
				}

			}
			else if (iValue[0] == CAM2 || iValue[0] == RCAM2)
			{
				CamNo[0] = iValue[0];
				Err = gDropCheckProcess(CamNo, 1, Result);

				if (Result[1] != 0)
				{
					Err = FCAMERA_DROP_ERROR;
				}
				else
				{
					Err = NO_ERR;
				}
				
			}
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding4::InspectionBarcode(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 InspectionBarcode TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue, strBarcode;
		int iValue[10];
		long CamNo = FHCAM, Err = 0, BarCodeType = 0, BarCodeReaderType = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		// 20210415 HarkDo
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			BarCodeType = iValue[0]; // 20210415 HarkDo
			BarCodeReaderType = iValue[1];
			if (BarCodeReaderType == 1)
			{
				gInspectBarcode(FHCAM, BarCodeType); // 20210415 HarkDo
				ULONGLONG TimeGet = 0, ElapsedTime = 0;
				TimeGet = _time_get();
				while (_time_elapsed(TimeGet) < TIME5000MS)
				{
					strBarcode = gGetCameraBarcode(FRONT_VISION);
					if (strBarcode.IsEmpty() == false)
					{
						break;
					}
				}
				if (strBarcode.IsEmpty() == true)
				{
					strBarcode.Format(_T("NULL_BARCODE"));
				}
			}
			else if (GetHeadBarcodeCognexUse() == true && GetBarcodeTypeCognexRead(BarCodeType) == true)
			{
				strBarcode = gcBarcodeControl->InspectBarcodeCognex(BarCodeType, TIME5000MS);
			}
			else
			{
				gInspectBarcode(FHCAM, BarCodeType); // 20210415 HarkDo
				ULONGLONG TimeGet = 0, ElapsedTime = 0;
				TimeGet = _time_get();
				while (_time_elapsed(TimeGet) < TIME5000MS)
				{
					strBarcode = gGetCameraBarcode(FRONT_VISION);
					if (strBarcode.IsEmpty() == false)
					{
						break;
					}
				}
				if (strBarcode.IsEmpty() == true)
				{
					strBarcode.Format(_T("NULL_BARCODE"));
				}
			}

			strRetMsg.Format(_T("%d,%s"), Err, (LPCTSTR)strBarcode);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000,0.0,0.0,0.0"));
	}
	return strRetMsg;
}

CString CDecoding4::SendBarcodeComPortStatus(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Status = 0;

	if (gcBarcodeControl->GetComPortStatus() == false)
	{
		Status = 0;
	}
	else
	{
		Status = 1;
	}

	strRetMsg.Format(_T("%d"), Status);

	return strRetMsg;
}

CString CDecoding4::BarcodeComPortOpen(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Status = 0;
	CString strPort;
	strPort.Format(_T(BARCODE_CONTROL_COM2));
	if (gcBarcodeControl->OpenPort(strPort, BARCODE_CONTROL_BAUDRATE) == true)
	{
		Status = 0;
	}
	else
	{
		Status = 1;
	}

	strRetMsg.Format(_T("%d"), Status);

	return strRetMsg;
}

CString CDecoding4::BarcodeComPortClose(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Status = 0;

	gcBarcodeControl->ClosePort();
	Status = 0;
	strRetMsg.Format(_T("%d"), Status);

	return strRetMsg;
}

CString CDecoding4::ReceiveMESResult(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	bool bRerverse = false;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding4 ReceiveMESResult TokenCount:%d\n", cTokenizer->GetCount());
		}
		int iValue[20];
		long BarcodeResult = INIT_EIMES, iCnt = 0;
		CString Barcode;

		ZeroMemory(&iValue, sizeof(iValue));

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}

		TRACE(_T("[PWR] *********************************************\n"));
		TRACE(_T("[PWR] ******* MES Result :%d (0:OK, 1:NG) *******\n"), iValue[0]);
		TRACE(_T("[PWR] *********************************************\n"));
		BarcodeResult = iValue[0];

		if (BarcodeResult == 0)
		{
			gMainMESResultOK();
		}
		else
		{
			gMainMESResultNG();
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}
