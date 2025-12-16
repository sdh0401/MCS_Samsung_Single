#include "pch.h"
#include "CMachineInit.h"
#include "CStartCalibrationFunc.h"
#include "CMachineFile.h"
#include "CRunFile.h"
#include "CInsertEndFile.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorData.h"
#include "Cwmx3Init.h"
#include "CPowerVision.h"
#include "EthernetVision.h"
#include "CPowerConveyorControl.h"
#include "CPowerConveyorManualControl.h"
#include "CPowerFeederControl.h"
#include "CPowerMainControl.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "CPowerMain.h"
#include "CPcbExist.h"
#include "CTowerLamp.h"
#include "CSmemaControl.h"
#include "CReturnToEntrance.h"
#include "CPowerTeachBox.h"
#include "CPowerGantry.h"
#include "CLedControl.h"
#include "CBarcodeControl.h"
#include "Cwmx3LinearIntplPos.h"
#include "CPowerCleaner.h"
#include "AxisInformation.h"
#include "GlobalIODefine.h"
#include "Trace.h"
#include "CMotorPower.h"
#include "CMachineInformation.h"
#include "CPowerSwitchPanel.h"
#include "CReadJobFile.h"
#include "CAdvancedMotionFile.h"
#include "CMotorProfile.h"
#include "CBarcodeFile.h"
#include "CPowerBuzzer.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CAutoNozzleChange.h"
#include "CStep.h"
#include "CMachineFileDB.h"
#include "CRunFileConveyorDB.h"
#include "CVirtualSerialControl.h"

CMachineInit* gcMachineInit;
CMachineInit::CMachineInit(bool bSimulation)
{
	m_bSimulationMode = bSimulation;
	m_RunMode = 0;
	m_StopMode = 0;
	SetRunMode(NORMAL_MODE);
	TRACE(_T("[PWR] CMachineInit Start\n"));
	m_UserTowerLamp.Normal.Red = 1;
	m_UserTowerLamp.Normal.Yel = 0;
	m_UserTowerLamp.Normal.Grn = 1;

	m_UserTowerLamp.Run.Red = 1;
	m_UserTowerLamp.Run.Yel = 1;
	m_UserTowerLamp.Run.Grn = 0;

	m_UserTowerLamp.Wait.Red = 1;
	m_UserTowerLamp.Wait.Yel = 0;
	m_UserTowerLamp.Wait.Grn = 1;

	m_UserTowerLamp.Alarm.Red = 0;
	m_UserTowerLamp.Alarm.Yel = 1;
	m_UserTowerLamp.Alarm.Grn = 1;

	m_UserTowerLamp.Empty.Red = 1;
	m_UserTowerLamp.Empty.Yel = 0;
	m_UserTowerLamp.Empty.Grn = 1;

	m_UseRearCamera = 0;
}

CMachineInit::~CMachineInit()
{
	gQuitMachine();
}

void CMachineInit::VariableInit()
{
	ZeroMemory(Camera, sizeof(Camera));
}

void CMachineInit::SystemFileInit()
{
	gcMachineFile = new CMachineFile();
	gcRunFile = new CRunFile();
	gcInsertEndFile = new CInsertEndFile();
	gcCAutoNozzleChange = new CAutoNozzleChange();
	gcMachineFileDB = new CMachineFileDB();
	gcRuntimeConveyorDB = new CRunFileConveyorDB();
	//gcVisionFile = new CVisionFile();

	bool openCalDB = gcMachineFileDB->CheckDBOpen();
	bool openRunConvDB = gcRuntimeConveyorDB->CheckDBOpen();
	if (openCalDB == true && openRunConvDB == true)
	{
		gcMachineFileDB->AllLoadFromDB();
		gcMachineFileDB->InsertAutoDeleteConfig(_T("D:\\PowerMotion\\TorqueZ"), 7);
		gcRuntimeConveyorDB->LoadRuntimeConveyor();
	}
	else if (gcMachineFile)
	{
		gcMachineFile->ReadFile();

		//gcMachineFile->InitToDiskBlock0();		// Camera
		//gcMachineFile->InitToDiskBlock1();		// Machine - 1D
		//gcMachineFile->InitToDiskBlock2();		// Machine - 2D	Front
		//gcMachineFile->InitToDiskBlock3();		// Machine - 2D	Rear
		//gcMachineFile->InitToDiskBlock4();		// Machine - 
		//gcMachineFile->InitToDiskBlock5();		// Machine - 
		//gcMachineFile->InitToDiskBlock8();		// Camera Align
		//gcMachineFile->InitToDiskBlockA();		// Machine - Z
		//gcMachineFile->InitToDiskBlockB();		// Machine - Z Environment
		//gcMachineFile->SaveFile();	

		//WriteHomePosition(_T("FZ1"), 0.000);
		//WriteHomePosition(_T("FZ2"), 0.000);
		//WriteHomePosition(_T("FZ3"), 0.000);
		//WriteHomePosition(_T("FZ4"), 0.000);
		//WriteHomePosition(_T("FZ5"), 0.000);
		//WriteHomePosition(_T("FZ6"), 0.000);

		gcMachineFile->ReadToDiskBlock0();		// 
		gcMachineFile->ReadToDiskBlock1();		// Machine - 1D
		gcMachineFile->ReadToDiskBlock2();		// Machine - 2D	Front		
		gcMachineFile->ReadToDiskBlock3();		// Machine - 2D Rear
		gcMachineFile->ReadToDiskBlock4();		// 
		gcMachineFile->ReadToDiskBlock5();		// 
		gcMachineFile->ReadToDiskBlock6();		// Conveyor Option
		gcMachineFile->ReadToDiskBlock7();		// Conveyor Parameter
		gcMachineFile->ReadToDiskBlock8();		// Camera Align
		gcMachineFile->ReadToDiskBlock9();		// Reference Feeder No and Position
		gcMachineFile->ReadToDiskBlockA();		// Machine - Z
		gcMachineFile->ReadToDiskBlockB();		// Machine - Z Environment
		gcMachineFile->ReadToDiskBlockC();		// Aging
		gcMachineFile->ReadToDiskBlockD();		// Insert Done
		gcMachineFile->ReadToDiskBlockF();		// Reserved

		gcMachineFileDB->InitialAndCopyFromOld();
		gcMachineFileDB->AllLoadFromDB();

		gcRuntimeConveyorDB->InitialRuntimeConveyor();
		gcRuntimeConveyorDB->SaveRuntimeConveyor();
		gcRuntimeConveyorDB->LoadRuntimeConveyor();
	}
	if (gcRunFile)
	{
		gcRunFile->ReadFile();
		NozzleNoPerHeadStruct temp = gcRunFile->ReadNozzleNoPerHead(FRONT_GANTRY);

		TRACE(_T("[PWR] Read Nozzle %d %d %d %d %d %d"), temp.Head[0], temp.Head[1], temp.Head[2], temp.Head[3], temp.Head[4], temp.Head[5]);
	}
	if (gcInsertEndFile)
	{
		gcInsertEndFile->ReadFile();


	}

	//if (gcCAutoNozzleChange)
	//{
	//	gcCAutoNozzleChange->InitANC();
	//}

	//if (gcVisionFile)
	//{
	//	gcVisionFile->ReadFile();
	//}
}

void CMachineInit::CalibrationDataInit()
{
	gcPowerCalibrationData = new CPowerCalibrationData();
}

void CMachineInit::ConveyorDataInit()
{
	gcPowerConveyorData = new CPowerConveyorData();
}

long CMachineInit::MotionInit(bool bReverse)
{
	long Err = NO_ERR;
	gcWmx3Init = new Cwmx3Init(m_bSimulationMode, bReverse);
	if (gcWmx3Init)
	{
		Err = gcWmx3Init->InitializeWmx3(_T("PowerMCS"));
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Please Re-Initialize after shutdown cause fail to initializeWmx3"));
			TRACE(_T("[PWR] InitializeWmx3 Fail Err:%d\n"), Err);
			exit(0);
			return Err;
		}
		ThreadSleep(TIME100MS);
		gcWmx3Init->Run();
	}
	gcStartCalibrationFunc = new CStartCalibrationFunc();
	if (gcStartCalibrationFunc->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcStartCalibrationFunc->GetId(&id);
		TRACE(_T("[PWR] gcStartCalibrationFunc Id(0x%04X) Ready\n"), id);
	}
	gcWmx3LinearIntplPos = new Cwmx3LinearIntplPos();
	return Err;
}

void CMachineInit::VisionInit()
{
	ThreadId_t id;
	gcPowerVision = new CPowerVision(m_bSimulationMode);
	if (gcPowerVision->PingThread(TIME100MS))
	{		
		gcPowerVision->GetId(&id);
		TRACE(_T("[PWR] CPowerVision Id(0x%04X) Ready\n"), id);
	}
	gcEthernetVision = new CEthernetVision(m_bSimulationMode);
	if (gcEthernetVision->PingThread(TIME100MS))
	{
		gcEthernetVision->GetId(&id);
		TRACE(_T("[PWR] CEthernetVision Id(0x%04X) Ready\n"), id);
	}
	ThreadSleep(TIME500MS);
}

void CMachineInit::ConveyorInit(bool bReverse)
{
	ThreadId_t id;
	gcPcbExist = new CPcbExist();
	if (gcPcbExist)
	{
		//if (bReverse == true)
		//{								// Entry, Low, Exist, Out, Exit
		//	gcPcbExist->SetPCBSensorIO(ENTRY_CONV, IN_FCONV_EXIT_EXIST, IO_NOUSE, IN_FCONV_WORK1_OUT, IO_NOUSE, IO_NOUSE);
		//	gcPcbExist->SetPCBSensorIO(WORK1_CONV, IO_NOUSE, IN_FCONV_WORK1_LOW, IN_FCONV_WORK1_EXIST, IN_FCONV_ENTRY_EXIST, IO_NOUSE);
		//	gcPcbExist->SetPCBSensorIO(EXIT_CONV, IO_NOUSE, IO_NOUSE, IN_FCONV_ENTRY_ENT, IO_NOUSE, IO_NOUSE);
		//}
		//else
		{
			gcPcbExist->SetPCBSensorIO(ENTRY_CONV, IN_FCONV_ENTRY_ENT, IO_NOUSE, IN_FCONV_ENTRY_EXIST, IO_NOUSE, IO_NOUSE);
			gcPcbExist->SetPCBSensorIO(WORK1_CONV, IO_NOUSE, IN_FCONV_WORK1_LOW, IN_FCONV_WORK1_EXIST, IN_FCONV_WORK1_OUT, IO_NOUSE);
			gcPcbExist->SetPCBSensorIO(EXIT_CONV, IO_NOUSE, IO_NOUSE, IN_FCONV_EXIT_EXIST, IO_NOUSE, IO_NOUSE);
		}
		gcPcbExist->Run();
		ThreadSleep(TIME1000MS);
	}
	gcSmemaControl = new CSmemaControl();
	if (gcSmemaControl)
	{
		//if (bReverse == true)
		//{
		//	gcSmemaControl->SetIO(ENTRY_CONV, IN_FCONV_NEXT_IN, OUT_FCONV_NEXT_OUT, IN_FCONV_PREV_IN, OUT_FCONV_PREV_OUT);
		//	gcSmemaControl->SetIO(EXIT_CONV, IN_FCONV_NEXT_IN, OUT_FCONV_NEXT_OUT, IN_FCONV_PREV_IN, OUT_FCONV_PREV_OUT);
		//}
		//else
		{
			gcSmemaControl->SetIO(ENTRY_CONV, IN_FCONV_PREV_IN, OUT_FCONV_PREV_OUT, IN_FCONV_NEXT_IN, OUT_FCONV_NEXT_OUT);
			gcSmemaControl->SetIO(EXIT_CONV, IN_FCONV_PREV_IN, OUT_FCONV_PREV_OUT, IN_FCONV_NEXT_IN, OUT_FCONV_NEXT_OUT);
		}
		gcSmemaControl->ShowSmemaIO();
	}
	gcPowerConveyorControl = new CPowerConveyorControl();
	if (gcPowerConveyorControl)
	{
		if (gcPowerConveyorControl->PingThread(TIME100MS))
		{
			gcPowerConveyorControl->GetId(&id);
			TRACE(_T("[PWR] CPowerConveyorControl Id(0x%04X) Ready\n"), id);
		}
	}
	gcEntryConveyor = new CEntryConveyor(bReverse);
	gcWorkConveyor = new CWorkConveyor(bReverse);
	gcExitConveyor = new CExitConveyor(bReverse);
	gcPowerConveyorManualControl = new CPowerConveyorManualControl();
	if (gcPowerConveyorManualControl)
	{
		if (gcPowerConveyorManualControl->PingThread(TIME100MS))
		{
			gcPowerConveyorManualControl->GetId(&id);
			TRACE(_T("[PWR] CPowerConveyorManualControl Id(0x%04X) Ready\n"), id);
		}
	}
}

void CMachineInit::MainInit()
{
	gcPowerMainControl = new CPowerMainControl();
	if (gcPowerMainControl)
	{
		if (gcPowerMainControl->PingThread(TIME100MS))
		{
			ThreadId_t id;
			gcPowerMainControl->GetId(&id);
			TRACE(_T("[PWR] CPowerMainControl Id(0x%04X) Ready\n"), id);
		}
	}
	gcPowerMain = new CPowerMain();
}

void CMachineInit::FeederInit()
{
	gcPowerFeederControl = new CPowerFeederControl();
	if (gcPowerFeederControl)
	{
		if (gcPowerFeederControl->PingThread(TIME100MS))
		{
			ThreadId_t id;
			gcPowerFeederControl->GetId(&id);
			TRACE(_T("[PWR] CPowerFeederControl Id(0x%04X) Ready\n"), id);
		}
	}
}

void CMachineInit::TeachBoxInit()
{
	gcPowerTeachBox = new CPowerTeachBox();
	if (gcPowerTeachBox)
	{
		if (gcPowerTeachBox->PingThread(TIME100MS))
		{
			ThreadId_t id;
			gcPowerTeachBox->GetId(&id);
			TRACE(_T("[PWR] CPowerTeachBox Id(0x%04X) Ready\n"), id);
		}
	}
}

void CMachineInit::GantryInit()
{
	gcPowerGantry = new CPowerGantry();
	if (gcPowerGantry)
	{
		TRACE(_T("[PWR] CPowerGantry Ready\n"));
	}
	gcAdvancedMotionFile = new CAdvancedMotionFile();

	long Err = gcAdvancedMotionFile->ReadFile();
	TRACE(_T("[PWR] Init AdvancedMotionFile Err(%d) \n"), Err);

	ClearAllEvent();
	SetDefaultTorqueLimitEvent(FRONT_GANTRY);
	SetDefaultAreaSensorEvent();
}

void CMachineInit::LedControlInit()
{
	gcLedControl = new CLedControl();
	if (gcLedControl)
	{
		CString strPort;
		strPort.Format(_T(LED_CONTROL_COM));
		gcLedControl->OpenPort(strPort, LED_CONTROL_BAUDRATE);
	}
}

void CMachineInit::BarcoodeControlInit()
{
	gcBarcodeControl = new CBarcodeControl();
	if (gcBarcodeControl)
	{
		CString strPort;
		strPort.Format(_T(BARCODE_CONTROL_COM2));
		gcBarcodeControl->OpenPort(strPort, BARCODE_CONTROL_BAUDRATE);
	}
}

void CMachineInit::CleanerInit()
{
	gcPowerCleaner = new CPowerCleaner();
	if (gcPowerCleaner)
	{
		gcPowerCleaner->Run();
	}
}

void CMachineInit::MachineFileInit()
{
	gcReadJobFile = new CReadJobFile();
	//gcAdvancedMotionFile = new CAdvancedMotionFile();
	gcMotorProfile = new CMotorProfile();
}

void CMachineInit::MotorPowerInit()
{
	/////////////////////////////////////////////////////////////////////////
	gcMotorPower = new CMotorPower();
	gcMotorPower->Run();
	gcMotorPower->SetStep(MotorPowerStep::STOP);
	/////////////////////////////////////////////////////////////////////////
}

void CMachineInit::MonitoringMachineInformationInit()
{
	gcMachineInformation = new CMachineInformation();
	gcMachineInformation->Run();
}

long CMachineInit::WaitMotorPowerOn(long TimeOut)
{
	/////////////////////////////////////////////////////////////////////////
	long Err = NO_ERR;
	Err = gcMotorPower->WaitMotorPowerOn(TimeOut);
	return Err;
	/////////////////////////////////////////////////////////////////////////
}

void CMachineInit::SwitchPanelInit()
{
	gcPowerSwitchPanel = new CPowerSwitchPanel();
	gcPowerSwitchPanel->Run();
	gcPowerSwitchPanel->SetStep(SwitchPanelStep::STOP);
}

void CMachineInit::TowerLampInit()
{
	gcTowerLamp = new CTowerLamp();
	if (gcTowerLamp)
	{
		if (gcTowerLamp->PingThread(TIME100MS))
		{
			ThreadId_t id;
			gcTowerLamp->GetId(&id);
			TRACE(_T("[PWR] CTowerLamp Id(0x%04X) Ready\n"), id);
		}
	}
}

void CMachineInit::BuzzerInit()
{
	gcPowerBuzzer = new CPowerBuzzer();
	gcPowerBuzzer->Run();
}

long CMachineInit::PauseMachine()
{
	long Err = NO_ERR;
	if (gcPowerGantry)
	{
		CString strLog;
		CApplicationTime* pTime = new CApplicationTime();
		gcPowerGantry->PauseGantry();
		TRACE(_T("[PWR] Pause Front Gantry Elapsed:%d[ms]\n"), pTime->TimeElapsed());
		strLog.Format(_T("[PWR] Pause Front Gantry Elapsed:%d[ms]"), pTime->TimeElapsed());
		gcPowerLog->Logging(strLog);
		delete pTime;
		Err = NO_ERR;
	}
	else
	{
		Err = INVALID_GANTRY;
	}
	return Err;
}

long CMachineInit::ResumeMachine()
{
	long Err = NO_ERR;
	if (gcPowerGantry)
	{
		CString strLog;
		ULONGLONG GetTime = 0, Elapsed = 0;
		GetTime = _time_get();
		gcPowerGantry->ResumeGantry();
		Elapsed = _time_elapsed(GetTime);
		TRACE(_T("[PWR] Resume Front Gantry Elapsed:%d[ms]\n"), Elapsed);
		strLog.Format(_T("[PWR] Resume Front Gantry Elapsed:%d[ms]"), (int)Elapsed);
		gcPowerLog->Logging(strLog);
		Err = NO_ERR;
	}
	else
	{
		Err = INVALID_GANTRY;
	}
	return Err;
}

long CMachineInit::ReleaseMachine()
{
	long Err = NO_ERR;
	if (gcPowerGantry)
	{
		CString strLog;
		ULONGLONG GetTime = 0, Elapsed = 0;
		GetTime = _time_get();
		gcPowerGantry->ReleaseGantry();
		Elapsed = _time_elapsed(GetTime);
		TRACE(_T("[PWR] Release Front Gantry Elapsed:%d[ms]\n"), Elapsed);
		strLog.Format(_T("[PWR] Release Front Gantry Elapsed:%d[ms]"), (int)Elapsed);
		gcPowerLog->Logging(strLog);
		Err = NO_ERR;
	}
	else
	{
		Err = INVALID_GANTRY;
	}
	return Err;
}

long CMachineInit::StopMachine()
{
	long Err = NO_ERR;
	CString strLog;
	CPowerThread* pThread;
	TRACE(_T("[PWR] Start to stop Front Gantry\n"));
	strLog.Format(_T("[PWR] Start to stop Front Gantry"));
	gcPowerLog->Logging(strLog);
	ULONGLONG GetTime = 0, Elapsed = 0;
	GetTime = _time_get();
	for (INT_PTR indx = 0; indx < GetRunThreadCount(); ++indx)
	{
		pThread = GetRunThread(indx);
		if (pThread != NULL)
		{
			DestroyRunThread(indx);
		}
	}
	Elapsed = _time_elapsed(GetTime);
	TRACE(_T("[PWR] Stop Front Gantry Elapsed:%d[ms]\n"), Elapsed);
	strLog.Format(_T("[PWR] Stop Front Gantry Elapsed:%d[ms]"), (int)Elapsed);
	gcPowerLog->Logging(strLog);
	return Err;
}

void CMachineInit::SetRunMode(long Mode)
{
	SEM_LOCK(gRUN_MODE, INFINITE);
	if (m_RunMode != Mode)
	{
		if (Mode == PROD_RUN)
		{
			SendToChangeMachineState(TimeStatics::RUN);
		}
		else if (Mode == PAUSE_MODE)
		{
			SendToChangeMachineState(TimeStatics::PAUSE);
		}
		else if (Mode == NORMAL_MODE)
		{
			SendToChangeMachineState(TimeStatics::STOP);
		}
	}
	m_RunMode = Mode;
	SEM_UNLOCK(gRUN_MODE);
}

long CMachineInit::GetRunMode()
{
	long RunMode = 0;
	SEM_LOCK(gRUN_MODE, INFINITE);
	RunMode = m_RunMode;
	SEM_UNLOCK(gRUN_MODE);
	return RunMode;
}

void CMachineInit::SetStopMode(long Mode)
{
	SEM_LOCK(gRUN_MODE, INFINITE);
	m_StopMode = Mode;
	SEM_UNLOCK(gRUN_MODE);
}

long CMachineInit::GetStopMode()
{
	long mode = 0;
	SEM_LOCK(gRUN_MODE, INFINITE);
	mode = m_StopMode;
	SEM_UNLOCK(gRUN_MODE);
	return mode;
}

void CMachineInit::InitCStep(long Gantry)
{
	SEM_LOCK(gRUN_MODE, INFINITE);

	gcStep = NULL;
	TRACE(_T("[PWR] InitCStep Complete:%d\n"), Gantry);

	SEM_UNLOCK(gRUN_MODE);
}

void CMachineInit::DeleteCStep(long Gantry)
{
	SEM_LOCK(gRUN_MODE, INFINITE);

	if (gcStep != NULL)
	{
		delete gcStep;
		gcStep = NULL;
		TRACE(_T("[PWR] DeleteCStep Complete:%d\n"), Gantry);
	}
	else
	{
		TRACE(_T("[PWR] DeleteCStep Skip:%d\n"), Gantry);
	}
	SEM_UNLOCK(gRUN_MODE);
}

void CMachineInit::CreateCStep(long Gantry)
{
	SEM_LOCK(gRUN_MODE, INFINITE);

	if (gcStep == NULL)
	{
		gcStep = new CStep;
		TRACE(_T("[PWR] CreateCStep Complete:%d\n"), Gantry);
	}
	else
	{
		TRACE(_T("[PWR] CreateCStep Skip:%d\n"), Gantry);
	}
	SEM_UNLOCK(gRUN_MODE);
}

bool CMachineInit::IsAliveCStep(long Gantry)
{
	bool result = false;
	SEM_LOCK(gRUN_MODE, INFINITE);
	if (gcStep == NULL)
	{
		result = false;
		TRACE(_T("[PWR] IsAliveCStep false:%d\n"), Gantry);
	}
	else
	{
		result = true;
		TRACE(_T("[PWR] IsAliveCStep true:%d\n"), Gantry);
	}
	SEM_UNLOCK(gRUN_MODE);
	return result;
}
long CMachineInit::SetTowerLamp(TowerLampLed UserLamp)
{
	m_UserTowerLamp = UserLamp;
	TRACE(_T("[PWR] SetTowerLamp\n"));
	TRACE(_T("[PWR] Normal Red,Yel,Grn(%d,%d,%d)\n"), UserLamp.Normal.Red, UserLamp.Normal.Yel, UserLamp.Normal.Grn);
	TRACE(_T("[PWR] Run    Red,Yel,Grn(%d,%d,%d)\n"), UserLamp.Run.Red, UserLamp.Run.Yel, UserLamp.Run.Grn);
	TRACE(_T("[PWR] Wait   Red,Yel,Grn(%d,%d,%d)\n"), UserLamp.Wait.Red, UserLamp.Wait.Yel, UserLamp.Wait.Grn);
	TRACE(_T("[PWR] Alarm  Red,Yel,Grn(%d,%d,%d)\n"), UserLamp.Alarm.Red, UserLamp.Alarm.Yel, UserLamp.Alarm.Grn);
	TRACE(_T("[PWR] Empty  Red,Yel,Grn(%d,%d,%d)\n"), UserLamp.Empty.Red, UserLamp.Empty.Yel, UserLamp.Empty.Grn);
	return NO_ERR;
}

TowerLampLed CMachineInit::GetTowerLamp()
{
	TowerLampLed UserLamp;
	UserLamp = m_UserTowerLamp;
	return UserLamp;
}

long CMachineInit::InitGantryPosition()
{
	gcPowerGantry->InitializeValue();
	return 0;
}

void CMachineInit::SetUseRearCamera(long UseRearCamera)
{
	m_UseRearCamera = UseRearCamera;
	TRACE(_T("[PWR] SetUseRearCamera:%d\n"), m_UseRearCamera);
}

long CMachineInit::GetUseRearCamera()
{
	TRACE(_T("[PWR] GetUseRearCamera:%d\n"), m_UseRearCamera);
	return m_UseRearCamera;
}

void CMachineInit::VirtualSerialInit()
{
	gcVirtualSerialControl = new CVirtualSerialControl();
	if (gcVirtualSerialControl)
	{
		CString strPort;
		strPort.Format(_T(VIRTUAL_SERIAL_COM));
		gcVirtualSerialControl->OpenPort(strPort, VIRTUAL_SERIAL_CONTROL_BAUDRATE);
	}
}