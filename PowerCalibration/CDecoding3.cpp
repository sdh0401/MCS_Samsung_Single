#include "pch.h"
#include "CDecoding3.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "AxisInformation.h"
#include "vision.h"
#include "VisionData.h"
#include "CPowerHMI.h"
//#include "ErrorCode.h"
#include "CPowerTeachBox.h"
#include "CReadJobFile.h"
#include "CHomeStatus.h"
#include "CPowerConveyorControl.h"
#include "CPowerLog.h"
#include "CTorqueMonitor.h"
#include "CPowerCalibrationData.h"
#include "CMeasureHeight.h"
#include "CAutoNozzleChange.h"
#include "CAvoidMotion.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "CPowerIO.h"
#include "CMachineInformation.h"
#include "CPowerBuzzer.h"
#include "CStartCalibrationFunc.h"
#include "CMachineFileDB.h"
#include "CDiscard.h"
#include "CylinderOscillator.h"
#include "Io_state_detector.h"
#include "CTrayDumpBox.h"

#define MAXGANTRYAXISNO_1ST	20

CDecoding3* gcDecoding3;
CDecoding3::CDecoding3() : cylinderOscillateInfo(new CylinderOscillateInfo())
{
	GetId(&m_id);
	m_HMRefPosFront.x = m_HMRefPosFront.y = m_HMRefPosRear.x = m_HMRefPosRear.y = -9999.0;
	m_updateInsertOffset = false;
}

CDecoding3::~CDecoding3()
{

}

BOOL CDecoding3::OnTask()
{
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding3::OnTask Thread(0x%04X)\n", m_id);
	}
	return TRUE;
}

BOOL CDecoding3::OnTask(LPVOID lpv)
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
			TRACE(_T("[PWR] CDecoding3 GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg();
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}

		strSendMsg.Format(_T(STRING_UNDEFINED_MESSAGE));
		
		switch (nSubMsg[1])
		{
		case HMI_CMD2ND_00:
			strSendMsg = HomingControl(strMsg);
			SetMachineManualActionRunninSign(false);
			break;
		case HMI_CMD2ND_01:
			strSendMsg = RequestDoorState(strMsg);
			break;
		case HMI_CMD2ND_02:
			if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = RequestServoState(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_30)
			{
				strSendMsg = ServoControl(strMsg);
				SetMachineManualActionRunninSign(false);
			}
			break;
		case HMI_CMD2ND_03:
			if (nSubMsg[2] == HMI_CMD3RD_14)
				strSendMsg = TeachTargetPosition(strMsg);
			break;
		case HMI_CMD2ND_04: // Move Gantry
			if(nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = MovePcbConfig(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = TeachPendent(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = ConfirmInsertPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03)
				strSendMsg = TeachInsertPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_04)
				strSendMsg = MoveGantry(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = ConfirmMeasureHeightPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_06)
			//	strSendMsg = TeachHeightMeasurementCurrentPosition(strMsg);
				strSendMsg = TeachMeasureHeightPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_08)
				strSendMsg = MoveAvoidGantry(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_09)
				strSendMsg = MoveAvoidSequenceGantry(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_10)
				strSendMsg = TeachAvoidGantry(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_11)
				strSendMsg = ManualMeasureHeightTry(strMsg);
            else if (nSubMsg[2] == HMI_CMD3RD_30)
                strSendMsg = MoveGantryToJig(strMsg);
            else if (nSubMsg[2] == HMI_CMD3RD_31)
                strSendMsg = TeachGantry(strMsg);
            else if (nSubMsg[2] == HMI_CMD3RD_34)
                strSendMsg = SetDumpTrayInfo(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_50)
				strSendMsg = InsertTorqueAlarmAction(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_60)
			{
				strSendMsg = AllDumpNormalMode(strMsg);
			}
			break;
		case HMI_CMD2ND_05: // Emergency
			if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = RequestEmergencyState(strMsg);
			break;
		case HMI_CMD2ND_06: // LOTO Key
			if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = RequestLotoKeyState(strMsg);
			break;
		case HMI_CMD2ND_07: // Global Data
			if (nSubMsg[2] == HMI_CMD3RD_00) // Discard Position Data Send
				strSendMsg = TeachGlobalDiscardPosition(strMsg);
			//	else if (nSubMsg[2] == HMI_CMD3RD_01) // Head Default Data Send
			//		strSendMsg = TeachGlobalNozzleNo(strMsg);
			// 3,60,1 명령으로 변경
			else if (nSubMsg[2] == HMI_CMD3RD_02) // Using Nozzle Data
				strSendMsg = TeachGlobalNozzleInformation(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_04) // Set TowerLamp Change Status Time
				strSendMsg = TeachGlobalTowerYellowLampTime(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_06)
				strSendMsg = TeachGlobalMachineReferenceMark(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_07)
				strSendMsg = TeachGlobalEmptyBuzzerTime(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_08)
				strSendMsg = ConveyorBeltManualControl(strMsg);
			break;
		case HMI_CMD2ND_08: // Feeder Ready IO
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadFeederReadyIO(strMsg);
			break;
		case HMI_CMD2ND_09: // Temperature IO
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadGantryHighIO(strMsg);
			break;
		case HMI_CMD2ND_10:
			strSendMsg = TeachPcbOffset(strMsg);
			break;
		case HMI_CMD2ND_20: // Conveyor
			if (nSubMsg[2] == HMI_CMD3RD_00)	// Manual Loading/Unloading
				strSendMsg = ManualPcbControl(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = SetConveyorStopDelay(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03) // Motor Profile
				strSendMsg = SetConveyorProfile(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_04)
				strSendMsg = MoveConveyorWidthRelative(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = MoveConveyorWidth(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_11) // PCB Position
				strSendMsg = ReadPcbSensor(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_12) // Read Conveyor Width
				strSendMsg = ReadConveyorWidth(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_20) // Set Conveyor Width
				strSendMsg = SetConveyorWidth(strMsg);
			break;
		case HMI_CMD2ND_30:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadMotorPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = ReadMotorPlusLimit(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = ReadMotorMiusLimit(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03)
				strSendMsg = ReadMotorTorque(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_04)
				strSendMsg = ReadServoState(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = ReadServoHomeState(strMsg);
			break;
		case HMI_CMD2ND_40:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = MoveFeederConfig(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_16)
				strSendMsg = TeachFeederOffset(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_17)
				strSendMsg = TeachFeederZ(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_20)
				strSendMsg = SetBowlFeederOff(strMsg);
			break;
		case HMI_CMD2ND_50:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadIO(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = WriteIO(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03)
				strSendMsg = ReadHeightMeasure(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = ManualANCBaseUpDown(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_06)
				strSendMsg = SetBuzzerNoUse(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_07)//3 50 7 + (.,.,.)
			{
				this->startThreadForPusherCylinderTimeCheck(strMsg, strSendMsg);
			}
			else if (nSubMsg[2] == HMI_CMD3RD_08)//3 50 8 + ()
			{
				this->setStopSignalOnForCylinderTimeCheckOn(strMsg, strSendMsg);
			}
			//3 50 9 커맨드는 실린더 반복 동작에 대해 동작 완료시 MCS->HMI로 일방적으로 보내줄 때 사용합니다. (받아서 하는 일 없음)
			else if (nSubMsg[2] == HMI_CMD3RD_10)//3 50 10 + ()
			{
				this->inquiryFromHMI_IfCylinderIsBeingTested(strMsg, strSendMsg);
			}
			else if (nSubMsg[2] == HMI_CMD3RD_11)//3 50 11 -> IO MAP 시작/끝 신호 ( PANEL_SWITCH & TOWER_LAMP )
			{
				this->tower_lamp_panel_switch_communication_start_end(strMsg, strSendMsg);
			}
			break;
		case HMI_CMD2ND_60:
			if (nSubMsg[2] == HMI_CMD3RD_00)
			{
				strSendMsg = GetCurrentNozzleState();
				nSubMsg[2] = HMI_CMD3RD_02; // 응답 시 3rd 수정.

			}
			else if (nSubMsg[2] == HMI_CMD3RD_01)
			{
				strSendMsg = SetNozzleNoFromHMI(strMsg);
				nSubMsg[2] = HMI_CMD3RD_02; // 응답 시 3rd 수정.
			}
			else if (nSubMsg[2] == HMI_CMD3RD_03)
			{
				strSendMsg = ManualNozzleChange(strMsg);
				nSubMsg[2] = HMI_CMD3RD_02; // 응답 시 3rd 수정.
			}
			else if (nSubMsg[2] == HMI_CMD3RD_30)
				strSendMsg = SuctionControl(strMsg);
			break;
		case HMI_CMD2ND_70:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = ReadPusherZPosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = MovePusherZOffset(strMsg);
			break;
		case HMI_CMD2ND_110:
			if (nSubMsg[2] == HMI_CMD3RD_00)
				strSendMsg = CalculateHomePosition(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_01)
				strSendMsg = StartAlignOffset(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_02)
				strSendMsg = StartHeadOffset(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_03)
				strSendMsg = StartROriginOffset(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_04)
				strSendMsg = SaveHMReference(strMsg);
			else if (nSubMsg[2] == HMI_CMD3RD_05)
				strSendMsg = TeachHMReference(strMsg);
			break;
        case HMI_CMD2ND_120:
        {
            if (nSubMsg[2] == HMI_CMD3RD_01)
            {
                strSendMsg.Empty(); BuzzerOff(); break;
            }
        }
		}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReturnMsg(%s)\n", (LPCTSTR)(strSendMsg));
		}

		if (strSendMsg.CompareNoCase(_T(STRING_UNDEFINED_MESSAGE)) == 0)
		{
			TRACE("[PWR] CDecoding1 STRING_UNDEFINED_MESSAGE(%s)\n", (LPCTSTR)(strSendMsg));
		}
		else if (strSendMsg.GetLength() > 0)
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

CString CDecoding3::MovePcbConfig(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		Ratio_XYRZ Ratio;
		Point_XYRZ pt;
		Point_XY ptOriginOffset, ptGoal, ptPcbFix;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&ptPcbFix, sizeof(ptPcbFix));
		ZeroMemory(&Ratio, sizeof(Ratio));
		ZeroMemory(&ptOriginOffset, sizeof(ptOriginOffset));
		CString strValue;
		int iValue[10];
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
			pt.x = dValue[0];
			pt.y = dValue[1];
			pt.r = /*dValue[2]*/0.0;
            const double& BlockOriginRotation = dValue[2];
			pt.z = dValue[3];
			ptOriginOffset.x = dValue[4];
			ptOriginOffset.y = dValue[5];
			Ratio.xy = Ratio.r = Ratio.z = 0.3;
			if (gcPowerLog->IsShowCommunicationLog() == true)
			{
				TRACE("[PWR] CDecoding3 X,Y,R,Z Pos,%.3f,%.3f,%.3f,%.3f OriginOffsetX,Y,%.3f,%.3f\n", pt.x, pt.y, pt.r, pt.z, ptOriginOffset.x, ptOriginOffset.y);
				TRACE("[PWR] CDecoding3 X,Y,R,Z Ratio,%.1f,%.1f,%.1f\n", Ratio.xy, Ratio.r, Ratio.z);
			}
			Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio.z);
			TRACE("[PWR] CDecoding3 StartAllZAxisWaitMotion Err:%d\n", Err);
			if (Err == NO_ERR)
			{
				Err = StartAllRAxisWaitMotion(Gantry, GetStandByR(Gantry), Ratio.r, TIME5000MS);
				TRACE("[PWR] CDecoding3 StartAllRAxisWaitMotion Err:%d\n", Err);
			}
			if (Err == NO_ERR)
			{
				ptPcbFix = ReadPcbFixPosition(Gantry);
				ptGoal.x = ptPcbFix.x + pt.x + ptOriginOffset.x;
				ptGoal.y = ptPcbFix.y + pt.y + ptOriginOffset.y;
                if (EPSILON < std::abs(BlockOriginRotation))
                {
                    ptGoal = transformCoordinateFromBlockBasedToEquipment(Point_XY{ pt.x, pt.y }, Point_XY{ ptPcbFix.x + ptOriginOffset.x, ptPcbFix.y + ptOriginOffset.y }, BlockOriginRotation);
                }
				Err = LinearIntplPosWaitMotion(Gantry, FHCAM, ptGoal, Ratio.xy, TIME5000MS);
				TRACE("[PWR] CDecoding3 LinearIntplPosWaitMotion Err:%d\n", Err);
				gLedAllOff();
				gLiveOn(FHCAM);
				gLedOn(FHCAM, 100, 0, 0);
			}
			if (Err == NO_ERR)
			{
				//Err = StartAllRAxisWaitMotion(Gantry, pt.r, Ratio.r, TIME5000MS);//pt.r 에 쓰였던 dValue[2]가 BlockOriginRotation에 사용됨에 따라 R축 이동 삭제.
				//TRACE("[PWR] CDecoding3 StartAllRAxisWaitMotion Err:%d\n", Err);
			}
			if (pt.z < GetStandByZ(Gantry))
			{
				pt.z = GetStandByZ(Gantry);
			}
			if (Err == NO_ERR)
			{
				Err = MoveZStandy(Gantry, pt.z, Ratio.z);
				TRACE("[PWR] CDecoding3 MoveZStandy Err:%d\n", Err);
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

CString CDecoding3::TeachPendent(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachPendent TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0;
		CString strValue, strAxis;
		int iValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			iValue[i] = cTokenizer->GetInt(i);
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			if (iValue[0] > FRONT_GANTRY)
				Gantry = iValue[0] - 1;
			strAxis = GetAxisNameByAxisIndex(iValue[1]);
			// Dir(0:Stop, 1:Minus, 2:Plus), Speed(0:Low, 1:Mid, 2:High), Jog(0:Relative, 1:Velocity)
			unsigned nSub1, nSub2, nSub3;
			nSub1 = iValue[2];
			nSub2 = iValue[3];
			nSub3 = iValue[4];

			if (nSub1 != 0)
			{
				Err = CheckReadyToMachine(true);
			}

			if (Err != NO_ERR)
			{
				strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
			}
			else if (gcPowerTeachBox)
			{

				gcPowerTeachBox->DirectControl(strAxis, nSub1, nSub2, nSub3);
				//PowerThreadMessage* msgReceived = new PowerThreadMessage();
				//msgReceived->SetThreadMsg(strAxis);
				//ThreadId_t id;
				//gcPowerTeachBox->GetId(&id);
				//msgReceived->SetID(id);
				//msgReceived->SetThreadSubMsg(nSub1, nSub2, nSub3);
				//if (gcPowerTeachBox->PingThread(TIME10MS))
				//{
				//	gcPowerTeachBox->Event((LPVOID)msgReceived);
				//}
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

CString CDecoding3::TeachCurPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachCurPosition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
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
			TRACE(_T("[PWR] CDecoding3 TeachCurPosition (%d)\n"), iValue[0]);
			Gantry = FRONT_GANTRY;
			ReadAllPosition(Gantry);
			for (long indx = static_cast<unsigned>(PowerAxis::FX); indx < MAXGANTRYAXISNO; ++indx)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), ReadPosition(indx));
			}
			Length = strRetMsg.GetLength();
			strRetMsg.Delete(Length - 1, 1);
			TRACE(_T("[PWR] CDecoding3 TeachCurPosition %s"), strRetMsg);
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

CString CDecoding3::TeachGlobalDiscardPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalDiscardPosition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		Point_XYRZ ptDiscard;
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
			TRACE(_T("[PWR] CDecoding3 TeachGlobalDiscardPosition DiscardXYRZ:%.3f,%.3f,%.3f,%.3f\n"), dValue[0], dValue[1], dValue[2], dValue[3]);
			ptDiscard.x = dValue[0];
			ptDiscard.y = dValue[1];
			ptDiscard.r = dValue[2];
			ptDiscard.z = dValue[3];
			SetGlobalDiscardPosition(ptDiscard);
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 TeachGlobalDiscardPosition %s"), strRetMsg);
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

CString CDecoding3::TeachGlobalNozzleNo(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalNozzleNo TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[10];
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
			TRACE(_T("[PWR] CDecoding3 TeachGlobalNozzleNo %d %d %d %d %d %d\n"), iValue[0], iValue[1], iValue[2], iValue[3], iValue[4], iValue[5]);
			for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
			{
				SetGlobalNozzleNo(HeadNo + TBL_HEAD1, iValue[HeadNo]);
			}			
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 TeachGlobalNozzleNo %s"), strRetMsg);
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

CString CDecoding3::TeachGlobalNozzleInformation(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalNozzleInformation TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[100];
		double dValue[100];
		NOZZLE Nozzle;
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
			//Number, Use, Type, TipHeight, PusherHeight, EmptyLevel, EmptyDiff, ExistLevel, ExistDiff
			for (long StationNo = 0; StationNo < MAXUSEDHEADNO * 2; ++StationNo)
			{
				Nozzle.No = iValue[0 + (StationNo * 7)];
				Nozzle.Use = iValue[1 + (StationNo * 7)];
				Nozzle.Type = iValue[2 + (StationNo * 7)];
				Nozzle.TipHeight = dValue[0 + (2 * StationNo)];
				Nozzle.PusherHeight = dValue[1 + (2 * StationNo)];
				Nozzle.Empty = iValue[3 + (StationNo * 7)];
				Nozzle.EmptyDiff = iValue[4 + (StationNo * 7)];
				Nozzle.Exist = iValue[5 + (StationNo * 7)];
				Nozzle.ExistDiff = iValue[6 + (StationNo * 7)];
				SetGlobalNozzleInformation(StationNo + TBL_HEAD1, Nozzle);
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

CString CDecoding3::TeachGlobalTowerYellowLampTime(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalTowerYellowLampTime TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, YellowTowerLampTime = 60;
		CString strValue;
		int iValue[20];
		double dValue[10];
		TowerLampLed UserTowerLamp;
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
		YellowTowerLampTime = iValue[0];
		UserTowerLamp.Normal.Red = iValue[1];
		UserTowerLamp.Normal.Yel = iValue[2];
		UserTowerLamp.Normal.Grn = iValue[3];
		UserTowerLamp.Run.Red = iValue[4];
		UserTowerLamp.Run.Yel = iValue[5];
		UserTowerLamp.Run.Grn = iValue[6];
		UserTowerLamp.Wait.Red = iValue[7];
		UserTowerLamp.Wait.Yel = iValue[8];
		UserTowerLamp.Wait.Grn = iValue[9];
		UserTowerLamp.Alarm.Red = iValue[10];
		UserTowerLamp.Alarm.Yel = iValue[11];
		UserTowerLamp.Alarm.Grn = iValue[12];
		UserTowerLamp.Empty.Red = iValue[13];
		UserTowerLamp.Empty.Yel = iValue[14];
		UserTowerLamp.Empty.Grn = iValue[15];
		TRACE(_T("[PWR] TeachGlobalTowerYellowLampTime:%d[Sec]\n"), YellowTowerLampTime);
		if (GetRunMode() == NORMAL_MODE)
		{
			SetGlobalTowerYellowLampTime(YellowTowerLampTime);
			SetTowerLamp(UserTowerLamp);
			strRetMsg.Format(_T("%d"), Err);
			TowerLampNormal();
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

CString CDecoding3::TeachGlobalEmptyBuzzerTime(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalEmptyBuzzerTime TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, EmptyBuzzerTime = 60;
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
		EmptyBuzzerTime = iValue[0];
		TRACE(_T("[PWR] TeachGlobalEmptyBuzzerTime:%d[Sec]\n"), EmptyBuzzerTime);
		if (GetRunMode() == NORMAL_MODE)
		{
			SetGlobalEmptyBuzzerTime(EmptyBuzzerTime);
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

CString CDecoding3::TeachGlobalMachineReferenceMark(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachGlobalMachineReferenceMark TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		MachineReferenceMark ReferenceMark;
		CString strValue;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&ReferenceMark, sizeof(ReferenceMark));
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
		TRACE(_T("[PWR] TeachGlobalMachineReferenceMark Use:%d(0:Use, 1:NoUse)\n"), iValue[0]);
		if (GetRunMode() == NORMAL_MODE)
		{
			if (iValue[0] == 0)
			{
				ReferenceMark.Use = 1;
			}
			else
			{
				ReferenceMark.Use = 0;
			}
			ReferenceMark.CycleTime = iValue[1];
			for (long MarkNo = 0; MarkNo < MACHINE_REFERENCE_MARKNO; ++MarkNo)
			{
				ReferenceMark.Mark[MarkNo].x = dValue[0 + (2 * MarkNo)];
				ReferenceMark.Mark[MarkNo].y = dValue[1 + (2 * MarkNo)];
				TRACE(_T("[PWR] Mark(%d) XY,%.3f,%.3f\n"), MarkNo + 1, ReferenceMark.Mark[MarkNo].x, ReferenceMark.Mark[MarkNo].y);
			}
			SetGlobalMachineReferenceMark(ReferenceMark);
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 TeachGlobalMachineReferenceMark %s"), strRetMsg);
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

CString CDecoding3::TeachPcbOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachPcbOffset TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];

		Point_XY ptCur, ptOriginOffset, ptTeach, ptPcbFix;
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
			TRACE(_T("[PWR] CDecoding3 TeachPcbOffset Gantry:%d OriginOffsetXY,%.3f,%.3f\n"), iValue[0], dValue[0], dValue[1]);
			Gantry = FRONT_GANTRY;
			ReadAllPosition(Gantry);
			ptCur = gReadGantryPosition(Gantry);
			ptPcbFix = ReadPcbFixPosition(Gantry);
			ptOriginOffset.x = dValue[0];
			ptOriginOffset.y = dValue[1];
			ptTeach.x = ptCur.x - (ptPcbFix.x + ptOriginOffset.x);
			ptTeach.y = ptCur.y - (ptPcbFix.y + ptOriginOffset.y);
			strRetMsg.AppendFormat(_T("%.3f,"), ptTeach.x);
			strRetMsg.AppendFormat(_T("%.3f,"), ptTeach.y);
			strRetMsg.AppendFormat(_T("%.3f,"), ptTeach.y);
			for (long indx = static_cast<unsigned>(PowerAxis::FZ1); indx < MAXGANTRYAXISNO; ++indx)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), ReadPosition(indx));
			}
			Length = strRetMsg.GetLength();
			strRetMsg.Delete(Length - 1, 1);
			TRACE(_T("[PWR] CDecoding3 TeachPcbOffset %s"), strRetMsg);
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

CString CDecoding3::MoveFeederConfig(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

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
			TRACE("[PWR] CDecoding3 MoveFeederConfig TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Stage = FRONT_STAGE;
		Ratio_XYRZ Ratio;
		Point_XYRZ pt;
		Point_XY RefPt, ptGoal;
		long FeederNo, Target, FeederType, TimeOut = TIME5000MS, MsXy = TIME10MS, Ms = TIME30MS;
		long NozzleNo = 0;
		NOZZLE Nozzle;
		double InposXY = 0.010, InposR = 1.0, InposZ = 0.050;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue, strRAxis, strZAxis;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Nozzle, sizeof(Nozzle));
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
			FeederNo = iValue[0];
			Target = iValue[1];
			FeederType = iValue[2];
			pt.x = dValue[0];
			pt.y = dValue[1];
			pt.r = dValue[2];
			pt.z = dValue[3];
			Ratio.xy = Ratio.r = Ratio.z = 0.7;
			if (FeederNo > 0 && FeederNo < MAXFEEDERNO)
			{
				gLedAllOff();
				gLiveOn(FHCAM);
				gLedOn(FHCAM, 100, 0, 0);

				if (FeederNo <= MAXHALFFEEDNO)
					Stage = FRONT_STAGE;
				else
					Stage = REAR_STAGE;
				RefPt = GetFeederPosition(Stage, FeederNo);
				ptGoal.x = RefPt.x + pt.x;
				ptGoal.y = RefPt.y + pt.y;

				if (FeederNo <= MAXHALFFEEDNO)
				{
					pt.r += 0.0;
				}
				else
				{
					pt.r += 180.0;
					TRACE(_T("[PWR] FdNo%03d Add PickOffsetR:%.3f\n"), FeederNo, pt.r);
				}

				Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio.z);
				TRACE(_T("[PWR] MoveZStandy Err:%d\n"), Err);
				if (Err == NO_ERR)
				{
					Err = StartAllRAxisWaitMotion(Gantry, GetStandByR(FRONT_GANTRY), Ratio.r, TIME5000MS);
					TRACE(_T("[PWR] StartAllRAxisWaitMotion Err:%d\n"), Err);
				}
				if (Err == NO_ERR)
				{
					if (Target == 0)
					{
						Err = LinearIntplPosWaitMotion(Gantry, FHCAM, ptGoal, Ratio.xy, TIME5000MS);
					}
					else
					{
						NozzleNo = GetGlobalNozzleNo(Target);
						Nozzle = GetGlobalNozzleInformation(NozzleNo);
						Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, InposXY, MsXy, TimeOut);
						//Err = LinearIntplPosWaitMotion(Gantry, Target, ptGoal, Ratio.xy, TIME5000MS);
						if (Err == NO_ERR)
						{
							strRAxis = GetRAxisFromHeadNo(Gantry, Target);
							strZAxis = GetZAxisFromHeadNo(Gantry, Target);
							if (strRAxis.CompareNoCase(_T("NON)")) != 0)
							{
								Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, pt.r, InposR, Ms, true);
								if (Err == NO_ERR)
								{
									if (strZAxis.CompareNoCase(_T("NON)")) != 0)
									{
										pt.z = pt.z - Nozzle.TipHeight + Nozzle.PusherHeight;
										Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, pt.z, InposZ, Ms, true);
									}
								}
							}
						}
					}
					TRACE(_T("[PWR] LinearIntplPosWaitMotion Err:%d\n"), Err);
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

CString CDecoding3::TeachFeederOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachFeederOffset TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Stage = FRONT_STAGE;
		CString strValue;
		int iValue[10];
		double dValue[10];
		Point_XY ptPickOffset, ptCur, ptRefFd;
		long FeederNo, Target, FeederType;
		Point_XY HeadOffset;

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
			TRACE(_T("[PWR] CDecoding3 TeachFeederOffset FeederNo:%d\n"), iValue[0]);
			Gantry = FRONT_GANTRY;
			FeederNo = iValue[0];
			Target = iValue[1];
			FeederType = iValue[2];
			ptCur = gReadGantryPosition(Gantry);
			if (FeederNo <= MAXHALFFEEDNO)
				Stage = FRONT_STAGE;
			else
				Stage = REAR_STAGE;

			if (Target >= TBL_HEAD1 && Target <= TBL_HEAD10)
			{
				HeadOffset = GetHeadOffset(Gantry, Target);
			}
			else
			{
				HeadOffset.x = HeadOffset.y = 0.000;
			}

			ptRefFd = GetFeederPosition(Stage, FeederNo);
			ptPickOffset.x = ptCur.x - ptRefFd.x - HeadOffset.x;
			ptPickOffset.y = ptCur.y - ptRefFd.y - HeadOffset.y;
			strRetMsg.AppendFormat(_T("%.3f,"), ptPickOffset.x);
			strRetMsg.AppendFormat(_T("%.3f"), ptPickOffset.y);
			TRACE(_T("[PWR] CDecoding3 TeachFeederOffset %s"), strRetMsg);
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

CString CDecoding3::TeachFeederZ(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachFeederZ TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, HeadNo;
		CString strValue;
		int iValue[10];
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
			Gantry = iValue[0];
			HeadNo = iValue[1];
			CString strZ = GetZAxisFromHeadNo(Gantry, HeadNo);
			double posZ = ReadPosition(strZ);
			long NozzleNo = GetGlobalNozzleNo(HeadNo);
			NOZZLE NozzleInfo = GetGlobalNozzleInformation(NozzleNo);
			double pickupZ = posZ + NozzleInfo.TipHeight - NozzleInfo.PusherHeight;

			TRACE(_T("[PWR] CDecoding3 TeachFeederZ %s %.3f\n"), strZ, pickupZ);
			strRetMsg.Format(_T("%.3f"), pickupZ);
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

CString CDecoding3::ConfirmInsertPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		ULONGLONG GetTime = 0, Elapsed = 0;
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, order = 0, Ret = NO_ERR;
		Ratio_XYRZ Ratio;
		Point_XY ptGoal, Cur;
		Point_XYR ptXYR;
		Point_XYRZ pt;
		long Target = FHCAM, TimeOut = TIME5000MS, Ms = TIME30MS;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.001;
		double PCBThickness = 0.0, RatioPusherZ = 0.5;
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
			order = iValue[0];
			Ratio.xy = Ratio.r = Ratio.z = 0.5;
			SetMachineState(STATE_RUN);
			SetGlobalStatusError(false);
			if (order > 0)
			{
				if (gcReadJobFile)
				{
					PCBThickness = gcReadJobFile->GetPcb().Thickness;
					if (IsExistSet(WORK1_CONV) == true)
					{
						GetTime = _time_get();
						Err = StartPosWaitMotion(GetPusherZName(FRONT_GANTRY), RatioPusherZ, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
						if (Err == NO_ERR)
						{
							WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
							TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
						}
					}
					Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio.z);
					if (Err == NO_ERR)
					{
						PCB Pcb = gcReadJobFile->GetPcb();
						const ORIGIN Origin = gcReadJobFile->GetOrigin();
						INSERT Insert = gcReadJobFile->GetInsert(order);
						Point_XY OriginXY, InsertXY;
						OriginXY.x = Origin.pt.x;
						OriginXY.y = Origin.pt.y;
						InsertXY.x = Insert.pt.x;
						InsertXY.y = Insert.pt.y;
                        if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
                        {
                            const Point_XYR blockOrigin = GetOriginXYRFromJobfile(Insert.BlockNo);
                            OriginXY = Point_XY{ blockOrigin.x, blockOrigin.y };
                        }
						if (Pcb.UseFiducial > 0)
						{
							FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
							Point_XY Mark1, Mark2;
                            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                            {
                                Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                                Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                            }
                            else//Pcb.UseFiducial == FIDUCIAL_BLOCK
                            {
                                Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
                                Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);
                            }
							gSetMarkLed(FiducialMark.MarkNo[0], FiducialMark.Led[0]);
							gSetMarkLed(FiducialMark.MarkNo[1], FiducialMark.Led[1]);
							TRACE(_T("[PWR] MarkRecognition WaitZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
							Ret = gMarkPairRecognition(FRONT_GANTRY, FiducialMark.MarkNo[0], FiducialMark.MarkNo[1], Mark1, Mark2, Ratio);
							if (Ret == NO_ERR)
							{
                                constexpr long GantryJobfile = FRONT_GANTRY;//FRONT_GANTRY
                                if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                                {
                                    Ret = gGetMarkDelta(GantryJobfile, MK_1, MK_2, Mark1, Mark2, MK_PWB);
                                }
                                else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                                {
                                    Ret = gGetMarkDelta(GantryJobfile, MK_1, MK_2, Mark1, Mark2, Insert.BlockNo);
                                }
								ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
								TRACE(_T("[PWR] No(%03d) Goal X,Y,%.3f,%.3f\n"), order, ptGoal.x, ptGoal.y);
								ptXYR.x = ptGoal.x;
								ptXYR.y = ptGoal.y;
								ptXYR.r = 0.000;
                                if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                                {
                                    ptXYR = gMarkCompensation(GantryJobfile, ptXYR, MK_PWB);
                                }
                                else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                                {
                                    ptXYR = gMarkCompensation(GantryJobfile, ptXYR, Insert.BlockNo);
                                }
								ptGoal.x = ptXYR.x;
								ptGoal.y = ptXYR.y;
							}
							else
							{
								Ret = RECOGNITION_MARK_FAIL;
								Ret = SendAlarm(RECOGNITION_MARK_FAIL, _T("Mark Recognition Fail before Confirm Insert"));
							}
						}
						else
						{
							ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
						}
						if (Ret == NO_ERR)
						{
							gLedOn(FHCAM, 10, 10, 0);
							gLiveOn(FHCAM);

							Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, Inpos, Ms, TimeOut);
							if (Err == NO_ERR)
							{
								Cur = gReadGantryPosition(Gantry);
								WriteConfirmInsertBeforePosition(Cur);
								Cur = gReadGantryPosition(FRONT_GANTRY);
								TRACE(_T("[PWR] Go No(%03d) CmdXY,%.3f,%.3f FeedXY,%.3f,%.3f DiffXY,%.3f,%.3f\n"),
									order,
									ptGoal.x, ptGoal.y, Cur.x, Cur.y, Cur.x - ptGoal.x, Cur.y - ptGoal.y);
							}
						}
					}
				}
				SetMachineState(STATE_IDLE);
				strRetMsg.Format(_T("%d"), Err);
			}
			else
			{
				strRetMsg.Format(_T("10000"));
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::TeachInsertPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, PointNo = 0, Ret = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.001;
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
			PointNo = iValue[1];
			if (PointNo > 0)
			{
				if (gcReadJobFile)
				{
					const INSERT Insert = gcReadJobFile->GetInsert(PointNo - 1);
					Point_XY ptOld, ptNew;
					Point_XYR Diff, Loss;
					Point_XYRZE ptCad;
					ptCad.x = Insert.pt.x;
					ptCad.y = Insert.pt.y;
					ptNew = gReadGantryPosition(FRONT_GANTRY);	// Origin + CAD + 
					ptOld = ReadConfirmInsertBeforePosition();
					Diff.x = ptNew.x - ptOld.x;
					Diff.y = ptNew.y - ptOld.y;
					TRACE(_T("[PWR] No(%d) CadXY,%.3f,%.3f OldXY,%.3f,%.3f DiffXY,%.3f,%.3f\n"), PointNo,
						ptCad.x, ptCad.y, ptOld.x, ptOld.y, Diff.x, Diff.y);

                    const PCB Pcb = gcReadJobFile->GetPcb();
                    if (Pcb.UseFiducial > 0)
                    {
                        constexpr long GantryJobfile = FRONT_GANTRY;//FRONT_GANTRY
                        if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                        {
                            Loss = gMarkLoss(GantryJobfile, &Diff, MK_PWB);
                        }
                        else// if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                        {
                            Loss = gMarkLoss(GantryJobfile, &Diff, Insert.BlockNo);
                        }
                    }
                    else
                    {
                        Loss.x = Diff.x;
                        Loss.y = Diff.y;
                    }
					TRACE(_T("[PWR] No(%d) CadXY,%.3f,%.3f OldXY,%.3f,%.3f LossXY,%.3f,%.3f\n"), PointNo,
						ptCad.x, ptCad.y, ptOld.x, ptOld.y, Loss.x, Loss.y);
					if (abs(Loss.x) > 0.003 || abs(Loss.y) > 0.003)
					{
						ptCad.x = ptCad.x + Loss.x;
						ptCad.y = ptCad.y + Loss.y;
						strRetMsg.Format(_T("%.3f,%.3f"), Loss.x, Loss.y);
						ptNew.x = ptNew.x + +(Diff.x - Loss.x);
						ptNew.y = ptNew.y + +(Diff.y - Loss.y);
						WriteConfirmInsertBeforePosition(ptNew);
					}
					else // 응답 추가
					{
						strRetMsg.Format(_T("%.3f,%.3f"), Loss.x, Loss.y);
					}
                    if (EPSILON < std::abs(gcReadJobFile->GetBlockOrigin(Insert.BlockNo).pt.r))
                    {
                        const Point_XY transformedLoss = transformCoordinateFromEquipmentToBlockBased(Point_XY{ Loss.x, Loss.y }, Point_XY{ 0, 0 }, gcReadJobFile->GetBlockOrigin(Insert.BlockNo).pt.r);
                        strRetMsg.Format(L"%.3f,%.3f", transformedLoss.x, transformedLoss.y);
                    }
				}
			}
			else
			{
				strRetMsg.Format(_T("10000"));
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}




CString CDecoding3::ConfirmMeasureHeightPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	strRetMsg = gcMeasureHeight->ConfirmMeasureHeightPosition(strHostMsg);

	return strRetMsg;
}


CString CDecoding3::TeachHeightMeasurementCurrentPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachPcbOffset TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];

		Point_XY ptCur, ptOriginOffset, ptTeach, ptPcbFix;
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

			Point_XY HMoffset = gcPowerCalibrationData->GetHMOffset(FRONT_GANTRY);
		//	ORIGIN Origin = gcReadJobFile->GetOrigin();
			double currT = GetHeight(Gantry);

			ptCur = gReadGantryPosition(Gantry);
			ptPcbFix = ReadPcbFixPosition(Gantry);

			ptOriginOffset.x = dValue[0];
			ptOriginOffset.y = dValue[1];

			ptTeach.x = ptCur.x - (ptPcbFix.x + ptOriginOffset.x + HMoffset.x);
			ptTeach.y = ptCur.y - (ptPcbFix.y + ptOriginOffset.y + HMoffset.y);

			TRACE(_T("[PWR] CDecoding3 TeachHMOffset Gantry:%d OriginOffsetXY,%.3f,%.3f\n"), iValue[0], ptOriginOffset.x, ptOriginOffset.y);
			TRACE(_T("[PWR] CDecoding3 TeachHMOffset HMoffsetXY,%.3f,%.3f CurrHeight,%.3f\n"), HMoffset.x, HMoffset.y, currT);
			TRACE(_T("[PWR] CDecoding3 TeachHMOffset CurrentXY,%.3f,%.3f\n"), ptCur.x, ptCur.y);
			TRACE(_T("[PWR] CDecoding3 TeachHMOffset PCBFixXY,%.3f,%.3f\n"), ptPcbFix.x, ptPcbFix.y);

			strRetMsg.AppendFormat(_T("%.3f,"), ptTeach.x);
			strRetMsg.AppendFormat(_T("%.3f,"), ptTeach.y);
			strRetMsg.AppendFormat(_T("%.3f"), currT);
	
			Length = strRetMsg.GetLength();
			strRetMsg.Delete(Length - 1, 1);
			TRACE(_T("[PWR] CDecoding3 TeachHMOffset %s"), strRetMsg);
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



CString CDecoding3::TeachMeasureHeightPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;


	strRetMsg = gcMeasureHeight->TeachMeasureHeightPosition(strHostMsg);

	return strRetMsg;

}







CString CDecoding3::MoveGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}
	if (strMsg.GetLength() > 0)
	{
		ULONGLONG GetTime = 0, Elapsed = 0;
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MoveGantry TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		Ratio_XYRZ Ratio;
		Point_XYRZ pt;
		Point_XY ptXY;
		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptXY, sizeof(ptXY));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue, strRAxis, strZAxis;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.003, InposR = 1.0, InposZ = 0.010;
		long TimeOut = TIME10000MS, Ms = TIME100MS, Target = FHCAM;
		long NozzleNo = 0;
		NOZZLE Nozzle;
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
			pt.x = dValue[0];
			pt.y = dValue[1];
			pt.r = dValue[2];
			pt.z = dValue[3];
			ptXY.x = pt.x;
			ptXY.y = pt.y;
			Ratio.xy = 0.5;
			Ratio.r = Ratio.z = 0.7;
			Target = iValue[1];

			GetTime = _time_get();
			Err = WaitAllZIdle(FRONT_GANTRY, TimeOut);
			TRACE(_T("[PWR] MoveGantry WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
			if (Err == NO_ERR)
			{
				TRACE(_T("[PWR] MoveGantry WaitAllZIdle Err:%d\n"), Err);
				GetTime = _time_get();
				Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio.z);
				TRACE(_T("[PWR] MoveGantry MoveZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
			}
			if (Err == NO_ERR)
			{
				TRACE(_T("[PWR] MoveGantry MoveZStandy Err:%d\n"), Err);
				GetTime = _time_get();
				Err = MoveRStandy(Gantry, GetStandByR(Gantry), Ratio.r);
				TRACE(_T("[PWR] MoveGantry MoveRStandy Elapsed,%d\n"), _time_elapsed(GetTime));
			}

			if (Target == TBL_CAMERA)
			{
				if (Err == NO_ERR)
				{
					TRACE(_T("[PWR] MoveGantry MoveRStandy Err:%d\n"), Err);
					Err = MoveStandByXY(Gantry, FHCAM, ptXY, Ratio.xy, TimeOut);
					TRACE(_T("[PWR] MoveGantry MoveStandByXY Elapsed,%d\n"), _time_elapsed(GetTime));
				}
				if (Err == NO_ERR)
				{
					GetTime = _time_get();
					Err = WaitRStandBy(Gantry, GetStandByR(Gantry));
					if (gcPowerLog->IsShowElapsedLog() == true)
					{
						TRACE(_T("[PWR] MoveGantry WaitRStandBy Elapsed,%d\n"), _time_elapsed(GetTime));
					}
				}
				if (Err == NO_ERR)
				{
					TRACE(_T("[PWR] MoveGantry WaitRStandBy Err:%d\n"), Err);
					Err = MoveRStandy(Gantry, pt.r, Ratio.r);
					TRACE(_T("[PWR] MoveGantry MoveRStandy Err:%d\n"), Err);
					Err = WaitRStandBy(Gantry, pt.r);
					TRACE(_T("[PWR] MoveGantry WaitRStandBy Err:%d\n"), Err);
					Err = MoveZStandy(Gantry, pt.z, Ratio.z);
					TRACE(_T("[PWR] MoveGantry MoveZStandy Err:%d\n"), Err);
				}
			}
			else
			{
				if (Err == NO_ERR)
				{
					Err = MoveStandByXY(Gantry, Target, ptXY, Ratio.xy, TimeOut);
					TRACE(_T("[PWR] MoveGantry MoveStandByXY Elapsed,%d\n"), _time_elapsed(GetTime));
				}

				TRACE(_T("[PWR] MoveGantry MoveStandByXY Elapsed,%d\n"), _time_elapsed(GetTime));
				NozzleNo = GetGlobalNozzleNo(Target);
				Nozzle = GetGlobalNozzleInformation(NozzleNo);
				strRAxis = GetRAxisFromHeadNo(Gantry, Target);
				strZAxis = GetZAxisFromHeadNo(Gantry, Target);
				TRACE(_T("[PWR] MoveGantry R(%s) Z(%s)\n"), strRAxis, strZAxis);
				if (strRAxis.CompareNoCase(_T("NON)")) != 0)
				{
					Err = StartPosWaitDelayedInposition(strRAxis, Ratio.r, TimeOut, pt.r, InposR, Ms, true);
					if (Err == NO_ERR)
					{
						if (strZAxis.CompareNoCase(_T("NON)")) != 0)
						{
							pt.z = pt.z - Nozzle.TipHeight + Nozzle.PusherHeight;
							Err = StartPosWaitDelayedInposition(strZAxis, Ratio.z, TimeOut, pt.z, InposZ, Ms, true);
						}
					}
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

CString CDecoding3::MoveGantryToJig(CString strHostMsg)
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
        ULONGLONG GetTime = 0, Elapsed = 0;
        CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
        ASSERT(cTokenizer->GetCount() > 0);
        if (gcPowerLog->IsShowCommunicationLog() == true)
        {
            TRACE("[PWR] CDecoding3 MoveGantry TokenCount:%d\n", cTokenizer->GetCount());
        }
        long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
        long Target = TBL_CAMERA;

        Ratio_XYRZ Ratio;
        Point_XYRZ pt;
        Point_XY ptXY;
        ZeroMemory(&pt, sizeof(pt));
        ZeroMemory(&ptXY, sizeof(ptXY));
        ZeroMemory(&Ratio, sizeof(Ratio));
        CString strValue;
        int iValue[10];
        double dValue[10];
        double Inpos = 0.003;
        long TimeOut = TIME10000MS, Ms = TIME100MS;
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
            Gantry = iValue[0];
            Target = iValue[1];
            pt.x = dValue[0];
            pt.y = dValue[1];
            pt.r = dValue[2];
            pt.z = dValue[3];
            ptXY.x = pt.x;
            ptXY.y = pt.y;
            Ratio.xy = 0.5;
            Ratio.r = Ratio.z = 0.5;
            GetTime = _time_get();
            Err = WaitAllZIdle(Gantry, TimeOut);
            TRACE(_T("[PWR] MoveGantry WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
            if (Err == NO_ERR)
            {
                TRACE(_T("[PWR] MoveGantry WaitAllZIdle Err:%d\n"), Err);
                GetTime = _time_get();
                Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio.z);
                TRACE(_T("[PWR] MoveGantry MoveZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
            }
            if (Err == NO_ERR)
            {
                TRACE(_T("[PWR] MoveGantry MoveZStandy Err:%d\n"), Err);
                GetTime = _time_get();
                Err = WaitZStandy(Gantry, GetStandByZ(Gantry));
                TRACE(_T("[PWR] MoveGantry WaitZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
            }
            if (Err == NO_ERR)
            {
                TRACE(_T("[PWR] MoveGantry WaitZStandy Err:%d\n"), Err);
                GetTime = _time_get();
                Err = MoveRStandy(Gantry, pt.r, Ratio.r);
                TRACE(_T("[PWR] MoveGantry MoveRStandy Elapsed,%d\n"), _time_elapsed(GetTime));
            }
            if (Err == NO_ERR)
            {
                TRACE(_T("[PWR] MoveGantry MoveRStandy Err:%d\n"), Err);
                Err = MoveStandByXY(Gantry, Target, ptXY, Ratio.xy, TimeOut);
                TRACE(_T("[PWR] MoveGantry MoveStandByXY Elapsed,%d\n"), _time_elapsed(GetTime));
            }
            if (Err == NO_ERR)
            {
                GetTime = _time_get();
                Err = WaitRStandBy(Gantry, pt.r);
                if (gcPowerLog->IsShowElapsedLog() == true)
                {
                    TRACE(_T("[PWR] MoveGantry WaitRStandBy Elapsed,%d\n"), _time_elapsed(GetTime));
                }
            }
            if (Err == NO_ERR)
            {
                if (Target != TBL_CAMERA)
                {
                    MoveZStandyOneToJig(Gantry, Target, pt.z, Ratio.z);
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

CString CDecoding3::TeachGantry(CString strHostMsg)
{
    CString strRetMsg, strMsg = strHostMsg;
    long Length = 0;
    if (strMsg.GetLength() > 0)
    {
        CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
        ASSERT(cTokenizer->GetCount() > 0);
        if (gcPowerLog->IsShowCommunicationLog() == true)
        {
            TRACE("[PWR] CDecoding3 TeachGantry TokenCount:%d\n", cTokenizer->GetCount());
        }
        long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Target = TBL_CAMERA;
        CString strValue;
        int iValue[10];
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
            Gantry = iValue[0];
            Target = iValue[1];
            double posX = ReadPosition(GetAxisX(Gantry));
            double posY = ReadPosition(GetAxisY1(Gantry));

            if (Target > TBL_CAMERA)
            {
                posX = posX - GetHeadOffset(FRONT_GANTRY, Target).x;
                posY = posY - GetHeadOffset(FRONT_GANTRY, Target).y;
            }

            strRetMsg.Format(_T("%.3f, %.3f"), posX, posY);
            TRACE(_T("[PWR] CDecoding3 TeachGantry %s"), strRetMsg);
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

CString CDecoding3::TeachTargetPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 TeachTargetPosition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Target = TBL_CAMERA;
		CString strValue;
		Point_XY pt;
		int iValue[10];
		double dValue[10];
		long NozzleNo = 0, HeadNo = 0;
		NOZZLE Nozzle;
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
			TRACE(_T("[PWR] CDecoding3 TeachTargetPosition Gantry(%d) Target(%d)\n"), iValue[0], iValue[1]);
			Gantry = FRONT_GANTRY;
			Target = iValue[1];
			ReadAllPosition(Gantry);
			pt = gReadGantryPosition(Gantry);
			if (Target > TBL_CAMERA)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), pt.x - GetHeadOffset(FRONT_GANTRY, Target).x);
				strRetMsg.AppendFormat(_T("%.3f,"), pt.y - GetHeadOffset(FRONT_GANTRY, Target).y);
				strRetMsg.AppendFormat(_T("%.3f,"), pt.y - GetHeadOffset(FRONT_GANTRY, Target).y);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%.3f,"), pt.x);
				strRetMsg.AppendFormat(_T("%.3f,"), pt.y);
				strRetMsg.AppendFormat(_T("%.3f,"), pt.y);
			}
			for (long indx = static_cast<unsigned>(PowerAxis::FZ1); indx < static_cast<unsigned>(PowerAxis::FZ6); ++indx)
			{
				NozzleNo = GetGlobalNozzleNo(TBL_HEAD1 + HeadNo);
				Nozzle = GetGlobalNozzleInformation(NozzleNo);
				strRetMsg.AppendFormat(_T("%.3f,"), ReadPosition(indx) + Nozzle.TipHeight - Nozzle.PusherHeight);
				HeadNo++;
			}
			for (long indx = static_cast<unsigned>(PowerAxis::FW1); indx < MAXGANTRYAXISNO; ++indx)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), ReadPosition(indx));
			}
			Length = strRetMsg.GetLength();
			strRetMsg.Delete(Length - 1, 1);
			TRACE(_T("[PWR] CDecoding3 TeachTargetPosition %s"), strRetMsg);
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

CString CDecoding3::SuctionControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 SuctionControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Target = TBL_CAMERA, OnOff = OUTOFF, Err = NO_ERR;
		CString strValue;
		int iValue[10];
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 SuctionControl Gantry(%d) Target(%d) OnOff(%d)\n"), iValue[0], iValue[1], iValue[2]);
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Gantry = FRONT_GANTRY;
			Target = iValue[1];
			OnOff = iValue[2];
			if (OnOff == 1)
				Err = SuctionOne(Gantry, Target, true);
			else
				Err = SuctionOne(Gantry, Target, false);
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 SuctionControl %s"), strRetMsg);
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

CString CDecoding3::RequestDoorState(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 RequestDoorState TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Target = TBL_CAMERA, OnOff = OUTOFF, Err = NO_ERR;
		CString strValue;
		int iValue[10];
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 RequestDoorState Target(%d)\n"), iValue[0]);
		}
		Target = iValue[0];
		long DoorStatus = DOOR_OPENED;
		if (Target == 0) // FRONT_DOOR
		{
			if (InputOne(IN_FDOOR_KEY_OUT) == INOFF/* && InputOne(IN_FDOOR_KEY_UNLOCK) == INOFF*/)
			{
				DoorStatus = DOOR_CLOSED;
			}
			else
			{
				DoorStatus = DOOR_OPENED;
			}
			strRetMsg.Format(_T("%d,%d,%d"), Err, DoorStatus, Target);
		}
		else
		{
			if (InputOne(IN_RDOOR_KEY_OUT) == INOFF/* && InputOne(IN_RDOOR_KEY_UNLOCK) == INOFF*/)
			{
				DoorStatus = DOOR_CLOSED;
			}
			else
			{
				DoorStatus = DOOR_OPENED;
			}
			strRetMsg.Format(_T("%d,%d,%d"), Err, DoorStatus, Target);
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

CString CDecoding3::RequestServoState(CString strHostMsg)
{
	ULONGLONG GetTime = 0, Elapsed = 0;
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	INT_PTR AxisNo = 0, ServoOnAxis = 0, AxisCount = GetWmx3AxisCount();
	Cwmx3Axis* pAxis = NULL;
	GetTime = _time_get();
	for (AxisNo = 0; AxisNo < AxisCount; ++AxisNo)
	{
		pAxis = g_pWmx3AxisArray.GetAt(AxisNo);
		if (pAxis != NULL)
		{
			if (pAxis->CheckServoOn() == false)
			{
				break;
			}
			else
			{
				ServoOnAxis++;
			}
		}
	}
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] Check All Servo Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
	}
	if (AxisCount == 0)
	{
		strRetMsg.Format(_T("%d"), ALLSERVO_OFF);
	}
	else
	{
		if (ServoOnAxis == AxisCount)
		{
			strRetMsg.Format(_T("%d"), ALLSERVO_ON);
		}
		else
		{
			strRetMsg.Format(_T("%d"), ALLSERVO_OFF);
		}
	}
	return strRetMsg;
}

CString CDecoding3::RequestEmergencyState(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (InputOne(IN_FEMERGENCY) == INON)
	{
		strRetMsg.Format(_T("%d"), EMERGENCY_ON);
	}
	else
	{
		strRetMsg.Format(_T("%d"), EMERGENCY_OFF);
	}
	return strRetMsg;
}

CString CDecoding3::RequestLotoKeyState(CString strHostMsg)
{
    /*
    * 2024.11.04
    * LOCK - UNLOCK 자리를 바꿨지만 해당 전처리문의 값(0,1)도 똑같이 서로 바꿨기 때문에 실질적으로 코드 변화는 없다.
    */
	CString strRetMsg, strMsg = strHostMsg;
	if (InputOne(IN_LOTO_KEY_ON) == INOFF)
	{
		strRetMsg.Format(_T("%d"), LOTO_KEY_UNLOCK);
	}
	else
	{
		strRetMsg.Format(_T("%d"), LOTO_KEY_LOCK);
	}
	return strRetMsg;
}

CString CDecoding3::ServoControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	long Err = NO_ERR;

	//SendPopupMessage(_T("Wait Servo Control."));

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ServoControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Target = TBL_CAMERA, OnOff = OUTOFF;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Ratio = 0.5;
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
			TRACE(_T("[PWR] CDecoding3 ServoControl OnOff(%d)\n"), iValue[0]);
			Gantry = FRONT_GANTRY;
			OnOff = iValue[0];
			if (OnOff == ALLSERVO_ON)
			{
				Err = CheckReadyToMachine(false);
				if (Err != NO_ERR)
				{
					//SendPopupMessage(_T("Servo Control Error"));
					strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
					delete cTokenizer;
					cTokenizer = NULL;
					return strRetMsg;
				}

				Err = gServoAllOn();
				if (Err == NO_ERR)
				{
					Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio);
				}
			}
			else
			{
				Err = gServoAllOff();
			}
			if (Err == NO_ERR)
			{
				if (OnOff == ALLSERVO_ON)
					strRetMsg.Format(_T("%d,%d"), Err, ALLSERVO_ON);
				else
					strRetMsg.Format(_T("%d,%d"), Err, ALLSERVO_OFF);

				//SendPopupClose();
			}
			else
			{
				//SendPopupMessage(_T("Servo Control Error"));
				strRetMsg.Format(_T("%d,%d"), Err, ALLSERVO_OFF);
			}
			TRACE(_T("[PWR] CDecoding3 ServoControl %s"), strRetMsg);
		}
		else
		{
			//SendPopupMessage(_T("Servo Control Error"));
			strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		//SendPopupMessage(_T("Servo Control Error"));
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}


CString CDecoding3::HomingControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(false);
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
			TRACE("[PWR] CDecoding3 HomingControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, All = 0;
		bool bHomingFail = false;
		CString strValue, strAxis;
		int iValue[10];
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 HomingControl All(%d)\n"), iValue[0]);
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Gantry = FRONT_GANTRY;
			All = iValue[0];
			if (All > 0)
			{
				strAxis = GetAxisNameByAxisIndex(All - 1);
				if (GetConvName(FRONT_CONV).CompareNoCase(strAxis) == 0)
				{
					Err = PrepareMoveConvWidth();
					if (Err == NO_ERR)
					{
						double homepos = gGetHomePosition(All - 1);
						Err = OneOriginSearch(strAxis, homepos);
					}
				}
				else
				{
					Err = OneOriginSearch(strAxis, 0.000);
				}
			}
			else
			{
				gServoAllOff();
				oneDCompensationOff();
				twoDCompensationOff();
				AllZCompensationOff(FRONT_GANTRY);
				ThreadSleep(TIME1000MS);
				gcHomeStatus = new CHomeStatus();
				gcHomeStatus->Run();
				ThreadSleep(TIME500MS);
				while (IsAllAxisHomingComplete() == false)
				{
					if (IsAllAxisHomingFail() == true)
					{
						bHomingFail = true;
						break;
					}
					ThreadSleep(TIME1000MS);
				}
				if (bHomingFail == false)
				{
					TRACE(_T("[PWR] ########## Wait to All axis(%d) homing complete    ########## \n"), GetWmx3AxisCount());
					ThreadSleep(TIME500MS);
					TRACE(_T("[PWR] Get1DCompensationUse:%s\n"), Get1DCompensationUse() == true ? _T("Use") : _T("UnUse"));
					if (Get1DCompensationUse() == true)
					{
						oneDCompensationOn();
					}
					else
					{
						oneDCompensationOff();
					}
					TRACE(_T("[PWR] Get2DCompensationUse:%s\n"), Get2DCompensationUse() == true ? _T("Use") : _T("UnUse"));
					if (Get2DCompensationUse() == true)
					{
						twoDCompensationOn();
					}
					else
					{
						twoDCompensationOff();
					}
					TRACE(_T("[PWR] GetZCompensationUse:%s\n"), GetZCompensationUse() == true ? _T("Use") : _T("UnUse"));
					if (GetZCompensationUse() == true)
					{
						AllZCompensationOn(FRONT_GANTRY);
					}
					else
					{
						AllZCompensationOff(FRONT_GANTRY);
					}
					SendCameraRecognitionOffset(FRONT_GANTRY);
					gLiveOn(FHCAM);
				}
				else
				{
					TRACE(_T("[PWR] ########## Homing Failed ##########\n"));
				}
				gcHomeStatus->ExitThreadLoop();
			}
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 HomingControl %s"), strRetMsg);
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

CString CDecoding3::ReadFeederReadyIO(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadFeederReadyIO TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		bool bReady = false;
		long ReadyIO = IO_NOUSE, Length = 0, ReadyIONo = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadFeederReadyIO\n"));
		}
		Gantry = FRONT_GANTRY;
		for (ReadyIONo = 0; ReadyIONo < MAX_READYIO_NO; ++ReadyIONo)
		{
			bReady = false;
			ReadyIO = GetReadyIONoFromReadyNo(ReadyIONo + 1);
			if (ReadyIO != IO_NOUSE)
			{
				bReady = InputTimeOne(ReadyIO, INON, TIME5MS);
			}
			if (bReady == true)
			{
				strRetMsg.AppendFormat(_T("%d,"), INON);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), INOFF);
			}
		}
		for (ReadyIONo = MAX_FEEDER_READY_NO; ReadyIONo < MAX_FEEDER_READY_NO + MAX_READYIO_NO; ++ReadyIONo)
		{
			bReady = false;
			ReadyIO = GetReadyIONoFromReadyNo(ReadyIONo + 1);
			if (ReadyIO != IO_NOUSE)
			{
				bReady = InputTimeOne(ReadyIO, INON, TIME5MS);
			}
			if (bReady == true)
			{
				strRetMsg.AppendFormat(_T("%d,"), INON);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), INOFF);
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadGantryHighIO(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadGantryHighIO TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		bool bReady = false;
		long ReadyIO = IO_NOUSE, Length = 0, ReadyIONo = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadGantryHighIO\n"));
		}
		Gantry = FRONT_GANTRY;

		if (GetUseRTDSensorFX() == 0)
		{
			bReady = InputTimeOne(IN_FX_LMTEMP_LOW, INON, TIME5MS);
		}
		else
		{
			if (GetTemperature(_T("FX")) > HIGH_TEMPERATURE)
			{
				bReady = false;
			}
			else
			{
				bReady = true;
			}
		}

		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d,"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d,"), INON);
		}

		if (GetUseRTDSensorFY1() == 0)
		{
			bReady = InputTimeOne(IN_FY1_LMTEMP_LOW, INON, TIME5MS);
		}
		else
		{
			if (GetTemperature(_T("FY1")) > HIGH_TEMPERATURE)
			{
				bReady = false;
			}
			else
			{
				bReady = true;
			}
		}

		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d,"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d,"), INON);
		}

		if (GetUseRTDSensorFY2() == 0)
		{
			bReady = InputTimeOne(IN_FY2_LMTEMP_LOW, INON, TIME5MS);
		}
		else
		{
			if (GetTemperature(_T("FY2")) > HIGH_TEMPERATURE)
			{
				bReady = false;
			}
			else
			{
				bReady = true;
			}
		}
		
		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d,"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d,"), INON);
		}
		bReady = InputTimeOne(IN_RX_LMTEMP_LOW, INON, TIME5MS);
		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d,"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d,"), INON);
		}
		bReady = InputTimeOne(IN_RY1_LMTEMP_LOW, INON, TIME5MS);
		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d,"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d,"), INON);
		}
		bReady = InputTimeOne(IN_RY2_LMTEMP_LOW, INON, TIME5MS);
		if (bReady == true)
		{
			strRetMsg.AppendFormat(_T("%d"), INOFF);
		}
		else
		{
			strRetMsg.AppendFormat(_T("%d"), INON);
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

CString CDecoding3::ReadPcbSensor(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadPcbSensor TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		bool bExist = false;
		long Entry = 0, Work = 0, Exit = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadPcbSensor\n"));
		}
		Gantry = FRONT_GANTRY;
		bExist = IsExistAll(ENTRY_CONV);
		if (bExist == true) Entry = 1;
		bExist = IsExistAll(WORK1_CONV);
		if (bExist == true) Work = 1;
		bExist = IsExistAll(EXIT_CONV);
		if (bExist == true) Exit = 1;
		strRetMsg.Format(_T("%d,%d,%d"), Entry, Work, Exit);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadPcbSensor %s"), strRetMsg);
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

CString CDecoding3::ReadConveyorWidth(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadConveyorWidth TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Position = 0.0;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadConveyorWidth Conveyor:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		ReadAllPosition(Gantry);
		Position = ReadPosition(GetConvName(FRONT_CONV));
		strRetMsg.Format(_T("%d,%.3f"), Err, Position);
		TRACE(_T("[PWR] CDecoding3 ReadConveyorWidth %s"), strRetMsg);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::SetConveyorWidth(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 SetConveyorWidth TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Width = 0.0;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 SetConveyorWidth Conveyor:%d Width:%.3f\n"), iValue[0], dValue[0]);
		}
		if (GetRunMode() == NORMAL_MODE)
		{
			Width = dValue[0];
			Gantry = FRONT_GANTRY;
			Err = WritePosition(GetConvName(FRONT_CONV), Width);
			WriteWidth(FRONT_CONV, WORK1_CONV, Width);
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 SetConveyorWidth %s"), strRetMsg);
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

CString CDecoding3::ManualPcbControl(CString strHostMsg)
{
	ULONGLONG GetTime = 0, Elapsed = 0;
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ManualPcbControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long From = 0, To = 0, TimeOut = TIME10000MS;
		bool bExist = false;
		long Entry = 0, Work = 0, Exit = 0, UpRatio = 50, DownRatio = 50, PusherZRatio = 0;
		double PcbThickness = 0.0, PcbStandByZOffset = 10.0;
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
		From = iValue[0];
		To = iValue[1];
		//if(PcbThickness < dValue[0])
			PcbThickness = dValue[0];
		//if(PcbStandByZOffset < dValue[1])
			PcbStandByZOffset = dValue[1];
		UpRatio = iValue[2] / 10;
		DownRatio = iValue[3] / 10;
		TRACE(_T("[PWR] CDecoding3 ManualPcbControl From:%d To:%d Pcb Thickness:%.3f PusherZStandByOffset:%.3f PusherZ Up:%03d%% Down:%03d%%\n"), From, To,
			PcbThickness, PcbStandByZOffset, UpRatio * 10, DownRatio * 10);
		Gantry = FRONT_GANTRY;
		if (GetRunMode() == NORMAL_MODE)
		{
			//////////////////////////////////////////////////////////////////////////
			SetManualConveyorRunMode(LOCATION_MANUAL);
			StartManualLocationConveyor(From, To);
			ThreadSleep(TIME200MS);
			PusherZRatio = UpRatio << 4;
			PusherZRatio = PusherZRatio + DownRatio;
			SetPcbInfoManualLocationConveyor(PcbThickness, PcbStandByZOffset, PusherZRatio);
			ThreadSleep(TIME200MS);
			RunManualLocationConveyor(From, To);
			ThreadSleep(TIME200MS);

			//////////////////////////////////////////////////////////////////////////
			GetTime = _time_get();
			while (GetManualLocationConveyorEnd() == false)
			{
				Elapsed = _time_elapsed(GetTime);
				if (Elapsed > TimeOut)
				{
					TRACE(_T("[PWR] Manual Location Conveyor TimeOut\n"));
					break;
				}
				ThreadSleep(TIME1000MS);
			}
			//////////////////////////////////////////////////////////////////////////
			StopManualLocationConveyor(From, To);
			bExist = IsExistEnt(ENTRY_CONV);
			if (bExist == false)
			{
				bExist = IsExistSet(ENTRY_CONV);
			}
			if (bExist == true) Entry = 1;
			bExist = IsExistSet(WORK1_CONV);
			if (bExist == true) Work = 1;
			bExist = IsExistSet(EXIT_CONV);
			if (bExist == true) Exit = 1;
			strRetMsg.Format(_T("%d,%d,%d,%d"), Err, Entry, Work, Exit);
			TRACE(_T("[PWR] CDecoding3 ManualPcbControl %s"), strRetMsg);
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


CString CDecoding3::MoveConveyorWidth(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MoveConveyorWidth TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME20000MS, Ms = TIME100MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Width = 0.0, Ratio = 0.5, Inpos = 0.1;
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
			if (gcPowerLog->IsShowCommunicationLog() == true)
			{
				TRACE(_T("[PWR] CDecoding3 MoveConveyorWidth Conveyor:%d\n"), iValue[0]);
			}

			Width = dValue[0];
			Err = MoveConvWidth(FRONT_CONV, Width);

			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 MoveConveyorWidth %s"), strRetMsg);
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


CString CDecoding3::MoveConveyorWidthRelative(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MoveConveyorWidthRelative TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME20000MS, Ms = TIME100MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Width = 0.0, Ratio = 0.5, Inpos = 0.1;
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
			if (gcPowerLog->IsShowCommunicationLog() == true)
			{
				TRACE(_T("[PWR] CDecoding3 MoveConveyorWidthRelative Conveyor:%d\n"), iValue[0]);
			}

			double offset = dValue[0];
			double currWidth = ReadPosition(GetConvName(FRONT_CONV));

			TRACE(_T("[PWR] CDecoding3 MoveConveyorWidthRelative %.3f -> %.3f\n"), currWidth, offset + currWidth);

			Err = MoveConvWidth(FRONT_CONV, offset + currWidth);

			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 MoveConveyorWidthRelative %s"), strRetMsg);
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

CString CDecoding3::SetConveyorStopDelay(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 SetConveyorStopDelay TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long WorkStopLowDelay = TIME500MS;
		long WorkStopMidDelay = TIME500MS;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 SetConveyorStopDelay:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		if (GetRunMode() == NORMAL_MODE)
		{
			WorkStopMidDelay = iValue[0];
			WorkStopLowDelay = iValue[1];
			SetWorkConveyorStopMidDelay(WorkStopMidDelay);
			SetWorkConveyorStopLowDelay(WorkStopLowDelay);
			strRetMsg.Format(_T("%d"), Err);
			TRACE(_T("[PWR] CDecoding3 SetConveyorStopDelay %s"), strRetMsg);
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

CString CDecoding3::ReadMotorPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadMotorPosition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadMotorPosition:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		ReadAllPosition(Gantry);
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), 0.0);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%.3f,"), ReadPosition(AxisNo));
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadMotorPlusLimit(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadMotorPlusLimit TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadMotorPlusLimit:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%d,"), 1);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), IsPositiveLimitSwitchOn(GetAxisNameByAxisIndex(AxisNo)) == true ? 0 : 1);
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}


CString CDecoding3::ReadMotorMiusLimit(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadMotorMiusLimit TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadMotorMiusLimit:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%d,"), 1);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), IsNegativeLimitSwitchOn(GetAxisNameByAxisIndex(AxisNo)) == true ? 0 : 1);
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadMotorTorque(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadMotorTorque TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadMotorTorque:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%.3f,"), 0.000);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%.3f,"), ReadActualTorque(GetAxisNameByAxisIndex(AxisNo)));
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadServoState(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadServoState TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadServoState:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%d,"), 1);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), CheckServoOn(GetAxisNameByAxisIndex(AxisNo)) == true ? 0 : 1);
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadServoHomeState(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadServoHomeState TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Length = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadServoHomeState:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO_1ST; ++AxisNo)
		{
			if (GetAxisMap(GetAxisNameByAxisIndex(AxisNo)) == NON)
			{
				strRetMsg.AppendFormat(_T("%d,"), 1);
			}
			else
			{
				strRetMsg.AppendFormat(_T("%d,"), IsOneAxisHomingComplete(GetAxisNameByAxisIndex(AxisNo)) == true ? 0 : 1);
			}
		}
		Length = strRetMsg.GetLength();
		strRetMsg.Delete(Length - 1, 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadIO(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadIO TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		CString strValue;
		UBYTE ioStatus = INOFF;
		int iValue[10];
		double dValue[10];
		long level = IO_NOUSE;

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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadIO:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		IONum = iValue[0];
		if (IONum < MAXIODEFINE)
		{
			if (gcPowerIO->GetIOType(IONum) == IO_WMX3_ANALOG_EJECTOR)
			{
				ioStatus = INON;
				level = gcPowerIO->GetAnalogLevel(IONum);
			}
			else if (gcPowerIO->GetIOType(IONum) == IO_WMX3_ANALOG_SICK) // 일단 height 보내는건 고려.
			{
				ioStatus = INON;
				level = gcPowerIO->GetAnalogLevel(IONum);
			}
			else if (gcPowerIO->IsOutput(IONum) == 1)
			{
				ioStatus = ReadOutputOne(IONum);
				level = IO_NOUSE;
			}
			else
			{				
				ioStatus = InputOne(IONum);
				level = IO_NOUSE;				
			}
		}
		strRetMsg.Format(_T("%d,%d,%d,%d"), NO_ERR, IONum, ioStatus == INON ? 0 : 1, level);

		SendIOStatus(strRetMsg);
		strRetMsg.Empty();
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::WriteIO(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 WriteIO TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		CString strValue;
		int iValue[10];
		double dValue[10];
		UBYTE IOOutput = OUTOFF;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 WriteIO:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		if (GetRunMode() == NORMAL_MODE)
		{
			IONum = iValue[0];
			IOOutput = iValue[1];
			TRACE(_T("[PWR] WriteIO IONum:%d IOOutput:%d\n"), IONum, IOOutput);
			if (IONum < MAXIODEFINE)
			{
				OutputOne(IONum, IOOutput);
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::ReadPusherZPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadPusherZPosition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, Conveyor = FRONT_CONV, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double PusherZPosition = 0.0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//for (int i = 0; i < cTokenizer->GetCount(); i++)
		//{
		//	strValue = cTokenizer->GetString(i);
		//	if (strValue.Find(_T(".")) >= 0)
		//	{
		//		dValue[dCnt] = cTokenizer->GetDouble(i);
		//		dCnt++;
		//	}
		//	else
		//	{
		//		iValue[iCnt] = cTokenizer->GetInt(i);
		//		iCnt++;
		//	}
		//}
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadPusherZPosition:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		Conveyor = FRONT_CONV;
		PusherZPosition = ReadPosition(GetPusherZName(Conveyor));
		strRetMsg.Format(_T("%d,%.3f"), NO_ERR, PusherZPosition);
		SendInformation(HMI_CMD1ST_3, HMI_CMD2ND_70, HMI_CMD3RD_02, strRetMsg);
		strRetMsg.Empty();
		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::MovePusherZOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePusherZOffset TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, Ms = TIME50MS;
		long Conveyor = FRONT_CONV;
		CString strValue;
		int iValue[10];
		double dValue[10];
		long Dir = 0;
		double PusherZPosition = 0.0, MoveOffset = 0.0, Ratio = 0.7, Inpos = 0.1;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 MovePusherZOffset:%d\n"), iValue[0]);
		}
		Gantry = FRONT_GANTRY;
		Conveyor = FRONT_CONV;
		MoveOffset = dValue[0];
		Dir = iValue[0];
		TRACE(_T("[PWR] MovePusherZOffset Dir:%d Offset:%.1f\n"), Dir, MoveOffset);
		if (Dir == 0)
			MoveOffset = MoveOffset * -1.0;
		else
			MoveOffset = MoveOffset * 1.0;
		if (GetRunMode() == NORMAL_MODE)
		{
			PusherZPosition = ReadPosition(GetPusherZName(Conveyor));
			if (Dir == 0)
			{
				Err = StartPosWaitMotionSkipLimitCheck(GetPusherZName(Conveyor), Ratio, TimeOut, PusherZPosition + MoveOffset, true);
			}
			else
			{
				Err = StartPosWaitDelayedInposition(GetPusherZName(Conveyor), Ratio, TimeOut, PusherZPosition + MoveOffset, Inpos, Ms, true);
			}

			if (Err == NO_ERR)
			{
				strRetMsg.Format(_T("%d,%.3f"), Err, PusherZPosition + MoveOffset);
			}
			else
			{
				strRetMsg.Format(_T("%d,%.3f"), Err, PusherZPosition);
			}
			SendInformation(HMI_CMD1ST_3, HMI_CMD2ND_70, HMI_CMD3RD_02, strRetMsg);
			strRetMsg.Empty();
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

CString CDecoding3::SetConveyorProfile(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 SetConveyorProfile TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS;
		CString strValue;
		int iValue[20];
		double dValue[20];
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 SetConveyorProfile\n"));
		}
		Gantry = FRONT_GANTRY;
		if (GetRunMode() == NORMAL_MODE)
		{
			JogInfo LowSpd, MidSpd, HighSpd;
			LowSpd.MaxVel = dValue[0];
			MidSpd.MaxVel = dValue[1];
			HighSpd.MaxVel = dValue[2];
			LowSpd.Acc = dValue[3];
			MidSpd.Acc = dValue[4];
			HighSpd.Acc = dValue[5];
			LowSpd.Dec = dValue[6];
			MidSpd.Dec = dValue[7];
			HighSpd.Dec = dValue[8];
			TRACE(_T("[PWR] Low  Vel:%.1f Acc:%.1f Dec:%.1f\n"), LowSpd.MaxVel, LowSpd.Acc, LowSpd.Dec);
			TRACE(_T("[PWR] Mid  Vel:%.1f Acc:%.1f Dec:%.1f\n"), MidSpd.MaxVel, MidSpd.Acc, MidSpd.Dec);
			TRACE(_T("[PWR] High Vel:%.1f Acc:%.1f Dec:%.1f\n"), HighSpd.MaxVel, HighSpd.Acc, HighSpd.Dec);
			SetConveyorProfileLow(LowSpd.MaxVel, LowSpd.Acc, LowSpd.Dec);
			SetConveyorProfileMid(MidSpd.MaxVel, MidSpd.Acc, MidSpd.Dec);
			SetConveyorProfileHigh(HighSpd.MaxVel, HighSpd.Acc, HighSpd.Dec);
			long PrevTransferTimeOut, InWorkTransferTimeOut, WorkOutTransferTimeOut, NextTransferTimeOut;

			PrevTransferTimeOut = iValue[0] * TIME1000MS;
			InWorkTransferTimeOut = iValue[1] * TIME1000MS;
			WorkOutTransferTimeOut = iValue[2] * TIME1000MS;
			NextTransferTimeOut = iValue[3] * TIME1000MS;
			TRACE(_T("[PWR] TimeOut Prev:%d InWork:%d WorkOut:%d Next:%d\n"), PrevTransferTimeOut, InWorkTransferTimeOut, WorkOutTransferTimeOut, NextTransferTimeOut);

			SetConveyorTransferTimeOut(FRONT_CONV, ENTRY_CONV, PrevTransferTimeOut, InWorkTransferTimeOut);
			SetConveyorTransferTimeOut(REAR_CONV, ENTRY_CONV, PrevTransferTimeOut, InWorkTransferTimeOut);

			SetConveyorTransferTimeOut(FRONT_CONV, WORK1_CONV, InWorkTransferTimeOut, WorkOutTransferTimeOut);
			SetConveyorTransferTimeOut(REAR_CONV, WORK1_CONV, InWorkTransferTimeOut, WorkOutTransferTimeOut);

			SetConveyorTransferTimeOut(FRONT_CONV, EXIT_CONV, WorkOutTransferTimeOut, NextTransferTimeOut);
			SetConveyorTransferTimeOut(REAR_CONV, EXIT_CONV, WorkOutTransferTimeOut, NextTransferTimeOut);			
			
			long PrevIn, ExitIn, PrevWork, ExitWork, PrevOut, ExitOut;
			PrevIn = iValue[4];
			ExitIn = iValue[5];
			PrevWork = iValue[6];
			ExitWork = iValue[7];
			PrevOut = iValue[8];
			ExitOut = iValue[9];
			TRACE(_T("[PWR] PrevIn:%d   ExitIn:%d\n"), PrevIn, ExitIn);
			TRACE(_T("[PWR] PrevWork:%d ExitWork:%d\n"), PrevWork, ExitWork);
			TRACE(_T("[PWR] PrevOut:%d  ExitOut:%d\n"), PrevOut, ExitOut);
			SetConveyorProfileSpeed(ENTRY_CONV, PrevIn, ExitIn);
			SetConveyorProfileSpeed(WORK1_CONV, PrevWork, ExitWork);
			SetConveyorProfileSpeed(EXIT_CONV, PrevOut, ExitOut);
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


CString CDecoding3::SetNozzleNoFromHMI(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 SetGlobalNozzleNo TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR;
		CString strValue;
		int iValue[100];
		double dValue[100];

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
			for (long head = 0; head < GetZAxisCount(); head++)
			{
				SetGlobalNozzleNo(head + 1, iValue[head]);
				TRACE("[PWR] CDecoding3 SetNozzleNoFromHMI %s Nzl:%d\n", GetZAxisFromHeadNo(Gantry, head + 1), iValue[head]);
			}
		}
		else
		{

		}

		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{

	}

	strRetMsg = GetCurrentNozzleState();

	return strRetMsg;
}



CString CDecoding3::GetCurrentNozzleState()
{
	CString strRetMsg;

	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] CDecoding3 RequestNozzleState Target\n"));
	}

	strRetMsg.AppendFormat(_T("%d"), GetGlobalNozzleNo(TBL_HEAD1));
	for (long index = 1; index < GetZAxisCount(); ++index)
	{
		strRetMsg.AppendFormat(_T(",%d"), GetGlobalNozzleNo(TBL_HEAD1 + index));
	}

//	strRetMsg.Format(_T("%d,%d,%d,%d,%d,%d"),
//		GetGlobalNozzleNo(TBL_HEAD1), GetGlobalNozzleNo(TBL_HEAD2), GetGlobalNozzleNo(TBL_HEAD3), GetGlobalNozzleNo(TBL_HEAD4), GetGlobalNozzleNo(TBL_HEAD5), GetGlobalNozzleNo(TBL_HEAD6));

	return strRetMsg;
}


CString CDecoding3::ManualNozzleChange(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0, Err = NO_ERR;
	long HeadNo, NozzlNo, Action;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ManualNozzleChange TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[100];
		double dValue[100];

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
			HeadNo = iValue[0];
			NozzlNo = iValue[1];
			Action = iValue[2];

			if (Action == 0)
			{
				Err = gcCAutoNozzleChange->NozzleRelease(Gantry, HeadNo, true);
			}
			else
			{
				Err = gcCAutoNozzleChange->NozzleHold(Gantry, HeadNo, NozzlNo, true);
			}

			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("ANC Error."));
				strRetMsg = GetCurrentNozzleState();
			}
			else
			{
				strRetMsg = GetCurrentNozzleState();
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::MoveAvoidGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	gcAvoidMotion = new CAvoidMotion(FRONT_GANTRY);
	strRetMsg = gcAvoidMotion->MoveAvoidGantry(strHostMsg);
	delete gcAvoidMotion;
	return strRetMsg;
}

CString CDecoding3::MoveAvoidSequenceGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	gcAvoidMotion = new CAvoidMotion(FRONT_GANTRY);
	strRetMsg = gcAvoidMotion->MoveAvoidSequenceGantry(strHostMsg);
	delete gcAvoidMotion;
	return strRetMsg;
}

CString CDecoding3::TeachAvoidGantry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	gcAvoidMotion = new CAvoidMotion(FRONT_GANTRY);
	strRetMsg = gcAvoidMotion->TeachAvoidPosition(strHostMsg);
	delete gcAvoidMotion;
	return strRetMsg;
}

CString CDecoding3::ConveyorBeltManualControl(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Length = 0, Err = NO_ERR;
	long Reverse, Conveyor, Control;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ConveyorBeltManualControl TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[100];
		double dValue[100];

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
			Reverse = iValue[0];
			Conveyor = iValue[1];
			Control = iValue[2];

			if (Conveyor == 0) // InBuffer
			{
				if (Control == 1)
					gcEntryConveyor->BeltOn(1, Reverse);
				else
					gcEntryConveyor->BeltOff(1);
			}
			else if (Conveyor == 1) // Work
			{
				if (Control == 1)
					gcWorkConveyor->BeltOn(1, Reverse);
				else
					gcWorkConveyor->BeltOff(1);
			}
			else if (Conveyor == 2) // OutBuffer
			{
				if (Control == 1)
					gcExitConveyor->BeltOn(1, Reverse);
				else
					gcExitConveyor->BeltOff(1);
			}
			strRetMsg.Format(_T("%d"), NO_ERR);
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

CString CDecoding3::ReadHeightMeasure(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 ReadHeightMeasure TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		CString strValue;
		int iValue[10];
		double dValue[10];
		UBYTE IOOutput = OUTOFF;
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
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] CDecoding3 ReadHeightMeasure Gantry %d\n"), iValue[0]);
		}
		Gantry = iValue[0];
		double height = 0.0;
		if (GetRunMode() == NORMAL_MODE)
		{
			height = GetHeight(FRONT_GANTRY);
			strRetMsg.Format(_T("%d,%d,%.3f"), NO_ERR, Gantry, height);
		}
		else
		{
			strRetMsg.Format(_T("%d,%d,%.3f"), ISNOT_NORMALMODE, Gantry, height);
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



CString CDecoding3::ManualANCBaseUpDown(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		long Base, UpDown;
		CString strValue;
		int iValue[10];
		double dValue[10];
		UBYTE IOOutput = OUTOFF;
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

		if (iValue[0] == 0)
		{
			Base = FRONT_STAGE;
		}
		else
		{
			Base = REAR_STAGE;
		}

		if (iValue[1] == 0)
		{
			UpDown = BASE_UP;
		}
		else
		{
			UpDown = BASE_DOWN;
		}

		if (GetRunMode() == NORMAL_MODE)
		{
			gcCAutoNozzleChange->ActBaseUpDown(UpDown);

			ULONGLONG TimeAct = _time_get();

			while (1)
			{
				Err = gcCAutoNozzleChange->GetBaseUpDown(UpDown, TIME100MS);

				if (Err == NO_ERR)
				{
					break;
				}
				else if (_time_elapsed(TimeAct) > TIME5000MS)
				{
					break;
				}

				ThreadSleep(TIME10MS);
			}

			strRetMsg.Format(_T("%d,%d"), Err, iValue[0]);
			if (Err != NO_ERR)
			{
				SendAlarmForNormal(Err, _T("ANC UpDown Error"));
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

CString CDecoding3::InsertTorqueAlarmAction(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		long Action;
		CString strValue;
		int iValue[100];
		double dValue[100];
		UBYTE IOOutput = OUTOFF;
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

		if (iValue[0] >= INSERT_RETRY && iValue[0] <= INSERT_ALARM)
		{
			Action = iValue[0];
			SetInsertTorqueAlarm(true, Action);
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

CString CDecoding3::SetBowlFeederOff(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[100];
		double dValue[100];
		UBYTE IOOutput = OUTOFF;
		BOWLFEEDER_OFFTIME OffData;

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&OffData, sizeof(OffData));
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
			if (iValue[0] <= 0)
			{
				iValue[0] = 0;
			}

			for (long idx = 1; idx <= MAX_READYIO_NO * 2; idx++)
			{
				if (idx <= MAX_READYIO_NO)
				{
					if (iValue[idx] == 0)
					{
						OffData.ReadyOnTimeFront[idx - 1] = 0;
					}
					else
					{
						OffData.ReadyOnTimeFront[idx - 1] = iValue[0] * TIME1000MS;
					}
				}
				else
				{
					if (iValue[idx] == 0)
					{
						OffData.ReadyOnTimeRear[idx - 1 - MAX_READYIO_NO] = 0;
					}
					else
					{
						OffData.ReadyOnTimeRear[idx - 1 - MAX_READYIO_NO] = iValue[0] * TIME1000MS;
					}
				}
			}			

			gcMachineInformation->SetBowlFeederOffTime(OffData);
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

CString CDecoding3::SetBuzzerNoUse(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[10];
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
			if (iValue[0] == 1)
			{
				gcPowerBuzzer->SetBuzzserNoUse(true);
			}
			else
			{
				gcPowerBuzzer->SetBuzzserNoUse(false);
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
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}


CString CDecoding3::CalculateHomePosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[10];
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

		long axisNo = iValue[0];
		double actualPosition = dValue[0];

		if (GetRunMode() == NORMAL_MODE)
		{
			CString strAxis = GetAxisNameByAxisIndex(axisNo);
			Cwmx3Axis* pAxis = GetWmx3AxisByName(strAxis);

			if (pAxis != NULL && IsOneAxisHomingComplete(strAxis) == true)
			{
				//double currHome = pAxis->GetHomePosition();
				double currHome = gcPowerCalibrationData->GetHomePosition(axisNo);
				double currPosition = ReadPosition(strAxis);
				double different = actualPosition - currPosition;
				double newHome = currHome + different;
				Limit currLimit = gcPowerCalibrationData->GetLimit(axisNo);

				if (pAxis->IsZAxis() == true || pAxis->IsRAxis() == true || pAxis->IsConveyorAxis() == true || pAxis->IsPusherZAxis() == true)
				{
					TRACE(_T("[PWR] %s %s Old:%.3f  New:%.3f\n"), strFunc, strAxis, currHome, newHome);
					gcPowerCalibrationData->SetHomePosition(axisNo, newHome);
					gcPowerCalibrationData->WriteHomePosition(FRONT_GANTRY);
					WritePosition(strAxis, actualPosition);
					pAxis->SetInitializeEnd(false);
					pAxis->SetInitializeFail(true);
					strRetMsg.Format(_T("0,%d,%.3f"), axisNo, newHome);
				}
				else
				{
					TRACE(_T("[PWR] %s %s reject\n"), strFunc, strAxis);
					strRetMsg.Format(_T("%d,%d,9999.999"), ISNOT_NORMALMODE, axisNo);
				}
			}
		}
		else
		{
			strRetMsg.Format(_T("%d,%d,0.000"), ISNOT_NORMALMODE, axisNo);
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

CString CDecoding3::StartAlignOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[10];
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
			Gantry = iValue[0];

			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_ALIGN_OFFSET_CAL_START));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(Gantry, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strRetMsg.Empty();
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
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

CString CDecoding3::StartHeadOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		long head;
		CString strValue;
		int iValue[10];
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
			Gantry = iValue[0];
			head = iValue[1];
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_HEAD_OFFSET_CAL_START));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(Gantry, head, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strRetMsg.Empty();
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
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

CString CDecoding3::StartROriginOffset(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;
		long head;
		CString strValue;
		int iValue[10];
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
			Gantry = iValue[0];
			head = iValue[1];
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_R_CAL_START));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(Gantry, head, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strRetMsg.Empty();
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
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

CString CDecoding3::SaveHMReference(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[10];
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
			Gantry = iValue[0];
			Point_XY pt = gReadGantryPosition(Gantry);

			if (Gantry == FRONT_GANTRY)
			{
				m_HMRefPosFront = pt;
			}
			else
			{
				m_HMRefPosRear = pt;
			}

			HeightMeasurementControl(true);
			TRACE(_T("[PWR] Gantry(%d) Read Height Measurement OriginXY,%.3f,%.3f \n"), Gantry, pt.x, pt.y);
			strRetMsg.Format(_T("%d,%d"), 0, Gantry);
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
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

CString CDecoding3::TeachHMReference(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	if (strMsg.GetLength() > 0)
	{
		CString strFunc(__func__);

		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

		CString strValue;
		int iValue[10];
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
			Point_XY pt, Offset;
			long readHM;
			Gantry = iValue[0];
			pt = gReadGantryPosition(Gantry);

			if (Gantry == FRONT_GANTRY && m_HMRefPosFront.x > 0.0)
			{
				readHM = gcPowerIO->GetAnalogInput(IN_FHEAD_ZHMD_HEIGHT);
				Offset.x = pt.x - m_HMRefPosFront.x;
				Offset.y = pt.y - m_HMRefPosFront.y;

				gcPowerCalibrationData->SetHMOffset(Gantry, Offset);
				gcPowerCalibrationData->SetHMZero(Gantry, readHM);
				gcPowerCalibrationData->WriteHMOffset(Gantry);

				TRACE(_T("[PWR] Gantry(%d) Write Height Measurement OffsetXY,%.3f,%.3f Zero,%d\n"), Gantry, Offset.x, Offset.y, readHM);
			}
			else if (Gantry == REAR_GANTRY && m_HMRefPosRear.x > 0.0)
			{
				readHM = gcPowerIO->GetAnalogInput(IN_RHEAD_ZHMD_HEIGHT);
				Offset.x = pt.x - m_HMRefPosRear.x;
				Offset.y = pt.y - m_HMRefPosRear.y;

				gcPowerCalibrationData->SetHMOffset(Gantry, Offset);
				gcPowerCalibrationData->SetHMZero(Gantry, readHM);
				gcPowerCalibrationData->WriteHMOffset(Gantry);

				TRACE(_T("[PWR] Gantry(%d) Write Height Measurement OffsetXY,%.3f,%.3f Zero,%d\n"), Gantry, Offset.x, Offset.y, readHM);
			}
			else
			{
				Err = ISNOT_NORMALMODE;
			}
			HeightMeasurementControl(false);
			strRetMsg.Format(_T("%d,%d"), Err, Gantry);
		}
		else
		{
			strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
		}

		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}

	m_HMRefPosFront.x = m_HMRefPosFront.y = m_HMRefPosRear.x = m_HMRefPosRear.y = -9999.0;

	return strRetMsg;
}

CString CDecoding3::InsertResultAutoCheck()
{
	CString strRetMsg, strMsg;
	long Err = NO_ERR;
	strRetMsg.Format(_T("%d"), Err);

	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}

	if (GetRunMode() != NORMAL_MODE)
	{
		return strRetMsg;
	}

	ULONGLONG GetTime = 0, Elapsed = 0;

	gcReadJobFile->ReadFile();

	long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Ret = NO_ERR;
	Ratio_XYRZ Ratio;
	Point_XY ptGoal;
	Point_XYR ptXYR;
	Point_XYRZ pt;
	Point_XY Mark1, Mark2;

	long Target = FHCAM, TimeOut = TIME5000MS, Ms = TIME30MS;
	ZeroMemory(&pt, sizeof(pt));
	ZeroMemory(&ptGoal, sizeof(ptGoal));
	ZeroMemory(&Ratio, sizeof(Ratio));
	CString strValue;

	double Inpos = 0.001;
	double PCBThickness = 0.0, RatioPusherZ = 0.5;

	std::deque<Point_XYRE> MarkResult[MAXUSEDHEADNO];
	std::deque<Point_XYRE> MarkResultDegree[MAXUSEDHEADNO][4];

	Gantry = FRONT_GANTRY;
	Ratio.xy = Ratio.r = Ratio.z = 0.3;
	SetMachineState(STATE_RUN);
	SetGlobalStatusError(false);

	PCBThickness = gcReadJobFile->GetPcb().Thickness;
	if (IsExistSet(WORK1_CONV) == true)
	{
		GetTime = _time_get();
		Err = StartPosWaitMotion(GetPusherZName(FRONT_GANTRY), RatioPusherZ, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
		if (Err == NO_ERR)
		{
			WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
			TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
		}
	}

	Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio.z);
	if (Err != NO_ERR)
	{
		return strRetMsg;
	}

	PCB Pcb = gcReadJobFile->GetPcb();
	ORIGIN Origin = gcReadJobFile->GetOrigin();
	PRODUCTION Prod = gcReadJobFile->GetProduction();
	Point_XY OriginXY, InsertXY;
	OriginXY.x = Origin.pt.x;
	OriginXY.y = Origin.pt.y;

	FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
	Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
	Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);

	if (Pcb.UseFiducial > 0)
	{
		gSetMarkLed(FiducialMark.MarkNo[0], FiducialMark.Led[0]);
		gSetMarkLed(FiducialMark.MarkNo[1], FiducialMark.Led[1]);
		Ret = gMarkPairRecognition(FRONT_GANTRY, FiducialMark.MarkNo[0], FiducialMark.MarkNo[1], Mark1, Mark2, Ratio);
		if (Ret != NO_ERR)
		{
			Ret = RECOGNITION_MARK_FAIL;
			Ret = SendAlarm(RECOGNITION_MARK_FAIL, _T("Mark Recognition Fail before Confirm Insert"));
			return strMsg;
		}
	}

	gLedOn(FHCAM, 0, 0, 0);
	gLiveOn(FHCAM);

	long addCnt = 0;

	gcMachineFileDB->LoadInsertOffset4532FromDB();

	for (long insertNo = 1; insertNo <= Prod.TotalInsertCount; insertNo++)
	{
		INSERT Insert = gcReadJobFile->GetInsert(insertNo);

		if (Insert.Use == 0)
		{
			continue;
		}

		if (gcReadJobFile->GetPick(Insert.FeederNo).Use == 0)
		{
			continue;
		}

		InsertXY.x = Insert.pt.x;
		InsertXY.y = Insert.pt.y;

		if (Pcb.UseFiducial > 0)
		{
			Ret = gGetMarkDelta(FRONT_GANTRY, MK_1, MK_2, Mark1, Mark2, MK_PWB);
			ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
			ptXYR.x = ptGoal.x;
			ptXYR.y = ptGoal.y;
			ptXYR.r = 0.000;
			ptXYR = gMarkCompensation(FRONT_GANTRY, ptXYR, MK_PWB);
			ptGoal.x = ptXYR.x;
			ptGoal.y = ptXYR.y;
		}
		else
		{
			ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
		}

		Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, Inpos, Ms, TimeOut);
		if (Err != NO_ERR)
		{
			return strRetMsg;
		}

		ThreadSleep(TIME200MS);

		long MarkNo = CHIP4532;

		Point_XYRE markResult = gCatchMark(FHCAM, MarkNo);
		if (markResult.exe == 1)
		{
			if (Insert.HeadNo >= 1)
			{
				MarkResult[Insert.HeadNo - 1].push_back(markResult);

				long degreeIndex = gDegreeToIndex(Insert.pt.r);
				MarkResultDegree[Insert.HeadNo - 1][degreeIndex].push_back(markResult);

			}
			TRACE(_T("[PWR] CPK No %d %s InsertXYR %.3f %.3f %.3f RecogTable %d MarkXY %.3f %.3f\n"),
				insertNo, GetZAxisFromHeadNo(Gantry, Insert.HeadNo), Insert.pt.x, Insert.pt.y, Insert.pt.r, Insert.RecogTable, markResult.x, markResult.y);
		}
		else
		{
			break;
		}
	}

	if (Ret != NO_ERR)
	{
		return strRetMsg;
	}

	double tempMark[MAXINSERTNO];
	Point_XYRE CalcSigma;
	Point_XY meanXY;

	meanXY.x = meanXY.y = 0.000;

	for (long head = 0; head < MAXUSEDHEADNO; head++)
	{
		long dataCnt = (long)MarkResult[head].size();
		if (dataCnt > 0)
		{
			for (long copy = 0; copy < dataCnt; copy++)
			{
				tempMark[copy] = MarkResult[head].at(copy).x;
				meanXY.x += MarkResult[head].at(copy).x;
			}

			CalcSigma.x = gCalculate3SD(tempMark, dataCnt);

			for (long copy = 0; copy < dataCnt; copy++)
			{
				tempMark[copy] = MarkResult[head].at(copy).y;
				meanXY.y += MarkResult[head].at(copy).y;
			}

			CalcSigma.y = gCalculate3SD(tempMark, dataCnt);

			meanXY.x = meanXY.x / dataCnt;
			meanXY.y = meanXY.y / dataCnt;

			TRACE(_T("[PWR] CPK %s Count:%d 3sXY %.3f %.3f MeanXY %.3f %.3f \n"), GetZAxisFromHeadNo(Gantry, head + 1), dataCnt, CalcSigma.x, CalcSigma.y, meanXY.x, meanXY.y);
		}
	}

	for (long head = 0; head < MAXUSEDHEADNO; head++)
	{


		for (long degreeIndex = 0; degreeIndex < 4; degreeIndex++)
		{
			long dataCnt = (long)MarkResultDegree[head][degreeIndex].size();
			meanXY.x = meanXY.y = 0.000;

			if (dataCnt > 3)
			{
				for (long copy = 0; copy < dataCnt; copy++)
				{
					tempMark[copy] = MarkResultDegree[head][degreeIndex].at(copy).x;
					meanXY.x += MarkResultDegree[head][degreeIndex].at(copy).x;
				}

				CalcSigma.x = gCalculate3SD(tempMark, dataCnt);

				for (long copy = 0; copy < dataCnt; copy++)
				{
					tempMark[copy] = MarkResultDegree[head][degreeIndex].at(copy).y;
					meanXY.y += MarkResultDegree[head][degreeIndex].at(copy).y;
				}


				CalcSigma.y = gCalculate3SD(tempMark, dataCnt);

				meanXY.x = meanXY.x / (double)dataCnt;
				meanXY.y = meanXY.y / (double)dataCnt;

				TRACE(_T("[PWR] CPK %s Count %d Degree %3d 3sXY %.3f %.3f MeanXY %.3f %.3f \n"), GetZAxisFromHeadNo(Gantry, head + 1), dataCnt, degreeIndex * 90, CalcSigma.x, CalcSigma.y, meanXY.x, meanXY.y);

				if (m_updateInsertOffset == true)
				{
					Point_XY offsetXYMeasure;
					Point_XY offsetXYPrev;
					Point_XY offsetXYNew;
					offsetXYMeasure.x = -1.0 * meanXY.x;
					offsetXYMeasure.y = -1.0 * meanXY.y;

					offsetXYPrev = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head + 1, degreeIndex);

					offsetXYNew.x = offsetXYPrev.x + offsetXYMeasure.x;
					offsetXYNew.y = offsetXYPrev.y + offsetXYMeasure.y;

					gcPowerCalibrationData->SetInsertOffset4532(Gantry, head + 1, degreeIndex, offsetXYNew);
				}

			}
		}
	}


	std::deque<Point_XYRE> MarkResultAll;
	Point_XYRE tempAll;

	for (long head = 0; head < MAXUSEDHEADNO; head++)
	{
		for (long degreeIndex = 0; degreeIndex < 4; degreeIndex++)
		{
			long dataCnt = (long)MarkResultDegree[head][degreeIndex].size();
			if (dataCnt > 3)
			{
				for (long idx = 0; idx < dataCnt; idx++)
				{
					tempAll = MarkResultDegree[head][degreeIndex].at(idx);

					MarkResultAll.push_back(tempAll);
				}
			}
		}
	}


	double allX[MAXINSERTNO];
	double allY[MAXINSERTNO];
	meanXY.x = 0.000;
	meanXY.y = 0.000;

	if (MarkResultAll.size() <= MAXINSERTNO)
	{
		for (long idx = 0; idx < MarkResultAll.size(); idx++)
		{
			allX[idx] = MarkResultAll.at(idx).x;
			allY[idx] = MarkResultAll.at(idx).y;

			meanXY.x += MarkResultAll.at(idx).x;
			meanXY.y += MarkResultAll.at(idx).y;
		}

		CalcSigma.x = gCalculate3SD(allX, (long)MarkResultAll.size());
		CalcSigma.y = gCalculate3SD(allY, (long)MarkResultAll.size());

		meanXY.x = meanXY.x / (double)MarkResultAll.size();
		meanXY.y = meanXY.y / (double)MarkResultAll.size();

		TRACE(_T("[PWR] CPK All Count %d 3sXY %.3f %.3f MeanXY %.3f %.3f \n"), (long)MarkResultAll.size(), CalcSigma.x, CalcSigma.y, meanXY.x, meanXY.y);

	}

	if (m_updateInsertOffset == true)
	{
		gcMachineFileDB->SaveInsertOffset4532FromDB();
	}

	return strRetMsg;
}

void CDecoding3::SetInsertOffsetUpdate(bool set)
{
	if (m_updateInsertOffset != set)
	{
		TRACE("[PWR] SetInsertOffsetUpdate %d\n", set);
	}
	m_updateInsertOffset = set;
}


CString CDecoding3::AllDumpNormalMode(CString strHostMsg)
{
	CString strRetMsg;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Format(_T("%d"), NO_ERR);
		return strRetMsg;
	}

	CDiscard* DiscardF = new CDiscard(FRONT_GANTRY);
	Ratio_XYRZ ratio;
	long Err = NO_ERR;

	ratio.xy = ratio.r = ratio.z = 0.5;

	if (GetRunMode() == NORMAL_MODE)
	{
		Err = DiscardF->DiscardAllNormalMode(ratio);

		strRetMsg.Format(_T("%d"), Err);
	}
	else
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
	}

	delete DiscardF;

	return strRetMsg;
}

CString CDecoding3::ManualMeasureHeightTry(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err;

	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		strRetMsg.Empty();
		return strRetMsg;
	}

	if (GetRunMode() == NORMAL_MODE)
	{
		if (IsAliveCStep(FRONT_GANTRY) == true)
		{
			DeleteCStep(FRONT_GANTRY);
		}
		CreateCStep(FRONT_GANTRY);

		SetGlobalStatusError(false);
		//gcMeasureHeight = new CMeasureHeight();
		Err = gcMeasureHeight->Run(1);
		//delete gcMeasureHeight;
	}
	else
	{
		Err = ISNOT_NORMALMODE;
	}

	strMsg.Empty();
	return strMsg;
}

CString CDecoding3::SetDumpTrayInfo(CString strHostMsg)
{
    CString strRetMsg, strMsg = strHostMsg;
    if (strMsg.GetLength() > 0)
    {
        CString strFunc(__func__);

        CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
        ASSERT(cTokenizer->GetCount() > 0);
        if (gcPowerLog->IsShowCommunicationLog() == true)
        {
            TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", strFunc, cTokenizer->GetCount());
        }
        long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, sCnt = 0, Err = NO_ERR, TimeOut = TIME5000MS, IONum = IO_NOUSE;

        CString strValue;
        int iValue[100];
        double dValue[100];
        CString sValue[100] = { _T(""), };
        CString strZAxis;

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
            else if (cTokenizer->IsNumber(i) == TRUE)
            {
                iValue[iCnt] = cTokenizer->GetInt(i);
                iCnt++;
            }
            else
            {
                sValue[sCnt] = cTokenizer->GetString(i);
                sCnt++;
            }
        }

        if (GetRunMode() == NORMAL_MODE)
        {
            DUMP_TRAY dumpInfo;
            long dumpNo = iValue[0];
            dumpInfo.ptRef.x = dValue[0];
            dumpInfo.ptRef.y = dValue[1];
            dumpInfo.ptRef.r = dValue[2];
            dumpInfo.ptRef.z = dValue[3];
            CString trayName = sValue[0];
            dumpInfo.NowPocket = iValue[1];

            for (long headNo = 0; headNo < TBL_HEAD6; headNo++)
            {
                if (iValue[2 + headNo] == 1)
                {
                    strZAxis = GetZAxisFromHeadNo(FRONT_GANTRY, headNo + 1);
                    dumpInfo.AssignZAxis.push_back(strZAxis);
                }
            }

            for (long headNo = 0; headNo < TBL_HEAD6; headNo++)
            {
                if (iValue[8 + headNo] == 1)
                {
                    strZAxis = GetZAxisFromHeadNo(REAR_GANTRY, headNo + 1);
                    dumpInfo.AssignZAxis.push_back(strZAxis);
                }
            }

            if (gcReadJobFile->Read_TRAY_DIRECT(_T("C:\\Power\\i6.0\\HMI\\Config\\TrayFeederList.xml"), trayName, &dumpInfo.trayInfo) == true)
            {
                dumpInfo.Use = true;
                //dumpInfo.Refilled = false;
                //dumpInfo.ExistSensorNo = IN_DUMPBOX_CHECK_1 + dumpNo;

                gcTrayDumpBox->SetBoxInfo(dumpNo, dumpInfo);
            }
            else
            {
                TRACE("[PWR] CDecoding3 %s TrayFeederList.xml %s read fail\n", strFunc, trayName);

                Err = JOBFILE_OPEN_FAIL;
            }

            strRetMsg.Format(_T("%d"), Err);
        }
        else
        {
            strRetMsg.Format(_T("%d,%d"), ISNOT_NORMALMODE, Gantry);
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

void CDecoding3::startThreadForPusherCylinderTimeCheck(CString& receivedMessage, CString& messageToSend)//3 50 7 ('io번호','반복수','딜레이')
/*
* 반복 작업 시작하라는 명령
*/
{
	const int strMsgTokenizedCount_MUST_BE_ALWAYS_THREE = 3;//3개 보내주기로 약속함 (3개 다 중요한 정보)

	//#0. 초기화
	messageToSend.Empty();
	CTokenizer cTokenizerInstance = CTokenizer(receivedMessage, _T(","), FALSE);
	receivedMessage.Empty();
	int iValue[strMsgTokenizedCount_MUST_BE_ALWAYS_THREE];
	ZeroMemory(&iValue, sizeof(iValue));

	//다른 함수에서도 쓰길래
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", __func__, cTokenizerInstance.GetCount());//__func__ : CString 객체로 안감싸도 괜찮다.
	}

	//#1. 메시지 토큰 개수 확인
	if (cTokenizerInstance.GetCount() != strMsgTokenizedCount_MUST_BE_ALWAYS_THREE)
	{
		messageToSend.Append(L"20000");//근데 왜 2만?
		return;
	}

	//#2. MCS 초기화여부 확인
	if (GetRunMode() != NORMAL_MODE)
	{
		messageToSend.Format(L"%d", ISNOT_NORMALMODE);
		return;
	}

	//#3. 메시지 Parsing 및 유효성 검증
	for (int i = 0; i < cTokenizerInstance.GetCount(); i++)
	{
		iValue[i] = cTokenizerInstance.GetInt(i);//숫자 안 보내면 예외발생ex)(a,a)이런거 : 예외처리 필요?
	}
	const int ioNumber = iValue[0];
	const int repetitions = iValue[1];
	const int delay = iValue[2];
	ZeroMemory(&iValue, sizeof(iValue));

	if (repetitions < 1)//필요 없을지도.. 그러면 아예 삭제??
	{
		/*repetitions should be at least 1*/
		messageToSend.Append(L"20000");//
		return;
	}

	if (delay < 0)
	{
		/*delay should be at least 0*/
		messageToSend.Append(L"20000");//
		return;
	}

	//#3.9 편하게 하기 위해 단순 캐스팅 사용. 만약 아래의 ioNumber가 범위 밖이면 staticFunctionForGeneralThreading 안에서 fromCylinderToEventSets 호출할 때 false 리턴하면서 제대로 반복동작 안함.
	this->cylinderOscillateInfo->cylinderToMove = static_cast<CylinderOscillateInfo::T>(ioNumber);

	//#4. 동작 실시
	if (this->cylinderOscillateInfo->isBeingUsed == true)
	{
		/*already working*/
		messageToSend.Append(L"20000");//
		return;
	}
	this->cylinderOscillateInfo->repetitions = repetitions;
	this->cylinderOscillateInfo->delay = delay;

	CWinThread* cWinThreadPointer = NULL;

	//이게 핵심
	cWinThreadPointer = AfxBeginThread(CylinderOscillator::staticFunctionForGeneralThreading, (LPVOID)this->cylinderOscillateInfo);

	if (cWinThreadPointer == NULL)
	{
		/*worker thread create failed*/
		messageToSend.Append(L"20000");//
		this->cylinderOscillateInfo->isBeingUsed = false;
		return;
	}
	cWinThreadPointer->m_bAutoDelete = true;

	//#5. return
	messageToSend.Append(L"0");//정상이라는 뜻
	return;
}

void CDecoding3::setStopSignalOnForCylinderTimeCheckOn(CString& receivedMessage, CString& messageToSend)
/*
* 반복 작업 도중 정지 키라는 명령
*/
{
	receivedMessage.Empty();//메시지 따로 받지 않습니다.
	if (this->cylinderOscillateInfo->isBeingUsed == false)
	{
		/*it is not working now : nothing to stop*/
		messageToSend.Format(L"20000");//
		return;
	}

	SetMachineManualActionRunninSign(false);

	messageToSend.Empty();
	CylinderOscillator& wmx3ApiAdapter = CylinderOscillator::GetInstance();
	if (wmx3ApiAdapter.setEmergencyMemoryOn(messageToSend) == true)
	{
		messageToSend.Format(L"0");//정상종료
		return;
	}
	else
	{
		messageToSend.Format(L"20000");//
		return;
	}
}

void CDecoding3::inquiryFromHMI_IfCylinderIsBeingTested(CString& receivedMessage, CString& messageToSend)
/*
* 실린더 반복중인지, HMI가 물어봤을때 동작하는 함수입니다.
* 반복중이라면 0, 반복중이지 않으면 1을 CString 으로 돌려줍니다.
*/
{
	receivedMessage.Empty();//메시지 따로 받지 않아요.

	if (this->cylinderOscillateInfo->isBeingUsed == true)
	{
		messageToSend.Format(L"0");
	}
	else
	{
		messageToSend.Format(L"1");
	}
	return;
}

void CDecoding3::tower_lamp_panel_switch_communication_start_end(CString& in_message, CString& out_message)
{
	//#0. 초기화
	out_message.Empty();
	CTokenizer cTokenizerInstance = CTokenizer(in_message, _T(","), FALSE);
	int iValue[1];
	ZeroMemory(&iValue, sizeof(iValue));

	//다른 함수에서도 쓰길래
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CDecoding3 %s TokenCount:%d\n", __func__, cTokenizerInstance.GetCount());
	}

	//#1. MCS 초기화여부 확인 + 생산중일때는 아무 동작 안함.
	if (GetRunMode() != NORMAL_MODE)
	{
		out_message.Format(L"%d", ISNOT_NORMALMODE);
		return;
	}

	//#3. 메시지 Parsing 및 유효성 검사 (아래 3가지 커맨드만 받음)
	if (
		in_message.Compare(L"0,0") != 0
		&&
		in_message.Compare(L"0,1") != 0
		&&
		in_message.Compare(L"1,0") != 0
		)
	{
		out_message.Format(L"20000");
		return;
	}

	if (in_message.Compare(L"0,0") == 0)//패널 스위치 독점(기존 패널 스위치 동작 비활성화) 안함
	{
		Io_state_detector::GetInstance().Go().set_monopolize_panel_switch(false);
	}
	else if (in_message.Compare(L"0,1") == 0)//패널 스위치 독점(기존 패널 스위치 동작 비활성화)
	{
		Io_state_detector::GetInstance().Go().set_monopolize_panel_switch(true);
	}
	else if (in_message.Compare(L"1,0") == 0)//정지
	{
		Io_state_detector::GetInstance().Stop();
	}
	else
	{
		TRACE("[PWR] CDecoding3::%s -> wrong code!", __func__);//여기 걸리면 내 잘못
		Io_state_detector::GetInstance().Stop();
	}

	out_message.Append(in_message);//그냥 그대로 대답

	return;//끝
}//end of method.
