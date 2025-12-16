#pragma once
#include "Thread.h"
#include "GlobalDefine.h"
#include "CylinderOscillateInfo.h"
class CDecoding3 : public CThread
{
public:
	CDecoding3();
	~CDecoding3();
	CString InsertResultAutoCheck();
	void SetInsertOffsetUpdate(bool set);
    CString AllDumpNormalMode(CString strHostMsg);
private:
	CString MovePcbConfig(CString strHostMsg);
	CString TeachPendent(CString strHostMsg);
	CString TeachPcbOffset(CString strHostMsg);
	CString TeachCurPosition(CString strHostMsg);
	CString TeachGlobalDiscardPosition(CString strHostMsg);
	CString TeachGlobalNozzleNo(CString strHostMsg);
	CString TeachGlobalNozzleInformation(CString strHostMsg);
	CString TeachGlobalTowerYellowLampTime(CString strHostMsg);
	CString TeachGlobalEmptyBuzzerTime(CString strHostMsg);
	CString TeachGlobalMachineReferenceMark(CString strHostMsg);
	CString MoveFeederConfig(CString strHostMsg);
	CString TeachFeederOffset(CString strHostMsg);
	CString TeachFeederZ(CString strHostMsg);
	CString ConfirmInsertPosition(CString strHostMsg);
	CString TeachInsertPosition(CString strHostMsg);
	CString ConfirmMeasureHeightPosition(CString strHostMsg);
	CString TeachMeasureHeightPosition(CString strHostMsg);
	CString TeachHeightMeasurementCurrentPosition(CString strHostMsg);
	CString MoveGantry(CString strHostMsg);
    CString MoveGantryToJig(CString strHostMsg);
    CString TeachGantry(CString strHostMsg);
	CString TeachTargetPosition(CString strHostMsg);
	CString SuctionControl(CString strHostMsg);
	CString RequestDoorState(CString strHostMsg);
	CString RequestServoState(CString strHostMsg);
	CString RequestEmergencyState(CString strHostMsg);
    //3 6 1
	CString RequestLotoKeyState(CString strHostMsg);
	CString ServoControl(CString strHostMsg);
	CString HomingControl(CString strHostMsg);
	CString ReadPcbSensor(CString strHostMsg);
	CString ReadConveyorWidth(CString strHostMsg);
	CString MoveConveyorWidth(CString strHostMsg);
	CString MoveConveyorWidthRelative(CString strHostMsg);
	CString SetConveyorWidth(CString strHostMsg);
	CString ManualPcbControl(CString strHostMsg);
	CString ReadFeederReadyIO(CString strHostMsg);
	CString ReadGantryHighIO(CString strHostMsg);
	CString SetConveyorStopDelay(CString strHostMsg);
	CString ReadMotorPosition(CString strHostMsg);
	CString ReadMotorPlusLimit(CString strHostMsg);
	CString ReadMotorMiusLimit(CString strHostMsg);
	CString ReadMotorTorque(CString strHostMsg);
	CString ReadServoState(CString strHostMsg);
	CString ReadServoHomeState(CString strHostMsg);
	CString ReadIO(CString strHostMsg);
	CString WriteIO(CString strHostMsg);
	CString ReadHeightMeasure(CString strHostMsg);
	CString ReadPusherZPosition(CString strHostMsg);
	CString MovePusherZOffset(CString strHostMsg);
	CString SetConveyorProfile(CString strHostMsg);
	CString SetNozzleNoFromHMI(CString strHostMsg);
	CString GetCurrentNozzleState();
	CString ManualNozzleChange(CString strHostMsg);
	CString MoveAvoidGantry(CString strHostMsg);
	CString MoveAvoidSequenceGantry(CString strHostMsg);
	CString TeachAvoidGantry(CString strHostMsg);
	CString ConveyorBeltManualControl(CString strHostMsg);
	CString ManualANCBaseUpDown(CString strHostMsg);
	CString InsertTorqueAlarmAction(CString strHostMsg);
	CString SetBowlFeederOff(CString strHostMsg);
	CString SetBuzzerNoUse(CString strHostMsg);

	CString CalculateHomePosition(CString strHostMsg);
	CString StartAlignOffset(CString strHostMsg);
	CString StartHeadOffset(CString strHostMsg);
	CString StartROriginOffset(CString strHostMsg);
	CString SaveHMReference(CString strHostMsg);
	CString TeachHMReference(CString strHostMsg);

	CString ManualMeasureHeightTry(CString strHostMsg);
    CString SetDumpTrayInfo(CString strHostMsg);

	Point_XY m_HMRefPosFront;
	Point_XY m_HMRefPosRear;

	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;

	bool m_updateInsertOffset;

	CylinderOscillateInfo* cylinderOscillateInfo;
	void startThreadForPusherCylinderTimeCheck(CString& receivedMessage, CString& messageToSend);
	void setStopSignalOnForCylinderTimeCheckOn(CString& receivedMessage, CString& messageToSend);
	void inquiryFromHMI_IfCylinderIsBeingTested(CString& receivedMessage, CString& messageToSend);
	//3 50 11 + (0 or 1) 커맨드 동작. IO STATUS 기능관련. "타워램프 조작 및 패널스위치 눌림 여부 확인해서 HMI에게 보내주는 stage"에 들어가고 나온다는 메시지 처리.
	void tower_lamp_panel_switch_communication_start_end(CString& in_message, CString& out_message);
};

extern CDecoding3* gcDecoding3;