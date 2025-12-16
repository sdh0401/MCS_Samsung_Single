#pragma once
#include "GlobalDefine.h"
class CMachineInit
{
public:
	CMachineInit(bool bSimulation);
	~CMachineInit();
	void VariableInit();
	void SystemFileInit();
	void CalibrationDataInit();
	void ConveyorDataInit();
	long MotionInit(bool bReverse);
	void VisionInit();
	void ConveyorInit(bool bReverse);
	void MainInit();
	void TeachBoxInit();
	void FeederInit();
	void GantryInit();
	void LedControlInit();
	void BarcoodeControlInit();
	void CleanerInit();
	void MachineFileInit();
	void MotorPowerInit();
	void MonitoringMachineInformationInit();
	long WaitMotorPowerOn(long TimeOut);
	void SwitchPanelInit();
	void TowerLampInit();
	void BuzzerInit();
	long PauseMachine();
	long ResumeMachine();
	long ReleaseMachine();
	long StopMachine();
	void SetRunMode(long Mode);
	long GetRunMode();
	void SetStopMode(long Mode);
	long GetStopMode();
	void InitCStep(long Gantry);
	void DeleteCStep(long Gantry);
	void CreateCStep(long Gantry);
	bool IsAliveCStep(long Gantry);
	long SetTowerLamp(TowerLampLed UserLamp);
	TowerLampLed GetTowerLamp();
	long InitGantryPosition();
	void SetUseRearCamera(long UseRearCamera);
	long GetUseRearCamera();
	void VirtualSerialInit();

private:
	long m_RunMode;
	long m_StopMode;
	bool m_bSimulationMode;
	TowerLampLed m_UserTowerLamp;
	long m_UseRearCamera;
};

extern CMachineInit* gcMachineInit;
