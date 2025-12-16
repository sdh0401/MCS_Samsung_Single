#include "pch.h"
#include "Cwmx3Init.h"

#include "GlobalDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "DefineThreadLoopTime.h"
#include "DefineMotionLoopTime.h"

#include "Cwmx3IO.h"
#include "CioStatus.h"
#include "CAnalogStatus.h"
#include "CMasterMotion.h"
#include "Cwmx3Motor.h"
#include "CSlaveMotorStatus.h"
#include "CPowerHMI.h"
#include "Cwmx3SystemParam.h"
#include "CPowerIO.h"
#include "CApplicationTime.h"
//#include "ErrorCode.h"

#include <WMX3Api.h>
#include <IOApi.h>
#include <CoreMotionApi.h>
#include <AdvancedMotionApi.h>
#include <UserMemoryApi.h>

#include <thread>
/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace wmx3Api;
using namespace ecApi;
using namespace std;

Cwmx3Init* gcWmx3Init;
Cwmx3Init::Cwmx3Init(bool bSimulation, bool bReverse)
{
	long Err = NO_ERR;
	if (bSimulation == true)
		SetAsSimulation();
	else
		SetAsRealMachine();
	m_bSafeQuit = true;
	m_bError = false;
	m_MaxAxisNo = MAXAXISNO;
	m_MaxUsedAxisNo = MAXAXISNO;
	m_MaxAxisNo = m_MaxUsedAxisNo;
	m_MotorStartSyncEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_IOStartSyncEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_bReverse = bReverse;
	//m_TerminteEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	//g_ThreadPcbReady = CreateEvent(NULL, TRUE, FALSE, NULL);
}

Cwmx3Init::~Cwmx3Init()
{
	SetGlobalStatusError(false);
	DeleteAllAxis();
	if (m_bSafeQuit == true) // Warning!!! 사용자가 직접 종료를 누른 경우에만 Stop Communication을 수행한다~~~
	{
		StopDeviceComm();
	}
	CloseDevice();
	DeleteAllWmx3Handle();	
}

long Cwmx3Init::InitializeWmx3(CString strDeviceName)
{
	long Err = NO_ERR;
	Err = CreateDevice();
	if (Err != NO_ERR)
		return Err;
	Err = SetGlobalDevice();
	if (Err != NO_ERR)
		return Err;
	Err = CreateIO();
	if (Err != NO_ERR)
		return Err;
	Err = CreateAdvancedMotion();
	if (Err != NO_ERR)
		return Err;
	Err = CreateEventControl();
	if (Err != NO_ERR)
		return Err;
	Err = CreateWmx3LogControl();
	if (Err != NO_ERR)
		return Err;
	Err = CreateCoreMotion();
	if (Err != NO_ERR)
		return Err;
	Err = CreateCoreMotionStatus();
	if (Err != NO_ERR)
		return Err;
	Err = SetDeviceName(strDeviceName);
	if (Err != NO_ERR)
		return Err;
	Err = StartDeviceComm();
	if (Err != NO_ERR)
		return Err;
	Err = CreateEcat();
	if (Err != NO_ERR)
		return Err;
	Err = CreateCompensation();
	if (Err != NO_ERR)
		return Err;
	Err = SetGlobalWmx3Env();
	if (Err != NO_ERR)
		return Err;
	////////////////////////////////////////////////////////////////////////////////////////
	Err = CreateAllAxis();
	if (Err != NO_ERR)
		return Err;
	Err = SetInfoAllAxis();
	if (Err != NO_ERR)
		return Err;
	return Err;
}

long Cwmx3Init::CreateAllAxis()
{
	if (IsSimulationMode() == true) return true;
	Cwmx3Axis* pAxis;
	bool bSimulation = IsSimulationMode();
	for (int indx = 0; indx < m_MaxAxisNo; indx++)
	{
		if (PowerAxisMap[indx] == NON) continue;
		pAxis = new Cwmx3Axis(bSimulation);
		AddWmx3Axis(pAxis);
	}
	TRACE(_T("[PWR] CreateAllAxis Total GetWmx3AxisCount():%d\n"), (long)GetWmx3AxisCount());
	return NO_ERR;
}

long Cwmx3Init::SetInfoAllAxis()
{
	if (IsSimulationMode() == true) return true;
	Cwmx3Axis* pAxis;
	Profile* profile = new Profile();
	INT_PTR AxisAddCount = 0;
	for (INT_PTR indx = 0; indx < m_MaxAxisNo; indx++)
	{
		if (PowerAxisMap[indx] == NON) continue;
		pAxis = GetWmx3AxisByIndex(AxisAddCount);
		pAxis->SetAxisInformation(PowerAxisMap[indx], PowerAxisArray[indx]);
		pAxis->SetAxisName(PowerAxisAliasName[indx]);
		pAxis->SetResol(PowerAxisResol[indx]);
		pAxis->SetUnResol(PowerAxisUnResol[indx]);
		pAxis->SetUseSlaveAxis(PowerAxisUseMasterSlave[indx]);
		pAxis->SetSlaveAxisIndex(PowerAxisWhatMasterSlave[indx]);
		pAxis->SetMotionControlType(PowerAxisMotionControlType[indx]);
		pAxis->SetMotorType(PowerAxisMotorType[indx]);
		pAxis->SetAxisSlaveNo(PowerAxisSlaveID[indx]);
		if (pAxis->IsGantryAxis() == true)
		{
			for (int inx = 0; inx < MAX_SHORT_DIST_LEVEL; ++inx)
			{
				pAxis->SetShortDist(inx, gShortDist[inx]);
				pAxis->SetShortDistVel(inx, gShortDistVel[inx]);
				pAxis->SetShortDistAccDec(inx, gShortDistAccDec[inx]);
			}
		}
		// Profile Setting
		memcpy(profile, &PowerAxisMoveParam[indx], sizeof(Profile));
		pAxis->SetMoveProfile(*profile);
		memcpy(profile, &PowerAxisTeachMoveParam[indx], sizeof(Profile));
		pAxis->SetTeachBoxMoveProfile(*profile);
		memcpy(profile, &PowerAxisTeachJogParam[indx], sizeof(Profile));
		pAxis->SetTeachBoxJogProfile(*profile);

		if (pAxis->IsConveyorBeltAxis() == true && m_bReverse == true)
		{
			if (PowerAxisMovingDir[indx] == static_cast<signed>(Wmx3AxisMoveDir::Positive))
			{
				PowerAxisMovingDir[indx] = static_cast<signed>(Wmx3AxisMoveDir::Negative);
				TRACE(_T("[PWR] %s direction reverse Positive -> Negative"),pAxis->GetAxisName());
			}
			else
			{
				PowerAxisMovingDir[indx] = static_cast<signed>(Wmx3AxisMoveDir::Positive);
				TRACE(_T("[PWR] %s direction reverse Negative -> Positive"), pAxis->GetAxisName());
			}			
		}

		pAxis->SetMovingDir(PowerAxisMovingDir[indx]);
		pAxis->SetHomingMaxTimeOut(PowerHomingMaxTimeOut[indx]);
		AxisAddCount++;
	}
	delete profile;
	return NO_ERR;
}

long Cwmx3Init::DeleteAllAxis()
{
	if (IsSimulationMode() == true) return true;
	TRACE(_T("[PWR] DeleteAllAxis(%d) Start\n"), GetWmx3AxisCount());
	CApplicationTime* pTime = new CApplicationTime();
	while (GetWmx3AxisCount() > 0)
	{
		RemoveWmx3Axis(0);
		ThreadSleep(WAIT_REMOVE_ARRAY_WAITTIME);
	}
	TRACE(_T("[PWR] DeleteAllAxis. Elapsed:%d[ms]\n"), pTime->TimeElapsed());
	delete pTime;
	return NO_ERR;
}

void Cwmx3Init::Run()
{
	gcWmx3SystemParam = new Cwmx3SystemParam();
	ImportParameterAndSetAll();

	gcMasterMotion = new CMasterMotion();
	gcWmx3Motor = new Cwmx3Motor(NULL, NULL, m_MotorStartSyncEvent);
	gcSlaveMotorStatus = new CSlaveMotorStatus(NULL, NULL, m_MotorStartSyncEvent);
	gcwmx3IO = new Cwmx3IO(NULL, NULL, m_IOStartSyncEvent);
	gcIOStatus = new CioStatus(NULL, NULL, m_IOStartSyncEvent);
	gcAnalogStatus = new CAnalogStatus();
	gcPowerIO = new CPowerIO(m_bReverse);

	gcWmx3Motor->Run();
	gcwmx3IO->Run();
	gcSlaveMotorStatus->Run();
	gcMasterMotion->Run();
	gcIOStatus->Run();
	gcAnalogStatus->Run();
}

bool Cwmx3Init::IsSimulationMode()
{
	return m_bSimulation;
}

void Cwmx3Init::SetAsSimulation()
{
	m_bSimulation = true;
}

void Cwmx3Init::SetAsRealMachine()
{
	m_bSimulation = false;
}

long Cwmx3Init::CreateDevice()
{
	int err = ErrorCode::None;
	EngineStatus* engStatus = new EngineStatus;
	ModulesInfoA* moduleInfo = new ModulesInfoA;
	int id = 0;
	CString strModule;
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3Device = new WMX3Api();
	NewDevice();	
	err = m_wmx3Device->CreateDevice(_T("C:\\Program Files\\SoftServo\\WMX3"), DeviceType::DeviceTypeNormal);
	if (err != ErrorCode::None) 
	{
		DeviceErrorToString(err, _T("CreateDevice"));
		SetInitError(true);
		return err;
	}
	err = m_wmx3Device->GetDeviceID(&id);
	err = m_wmx3Device->GetEngineStatus(engStatus);
	m_wmx3Device->GetModulesInfo(moduleInfo);
	for (long Cnt = 0; Cnt < wmx3Api::constants::moduleNameLen; ++Cnt)
	{
		strModule.AppendFormat(_T("%c"), moduleInfo->modules->moduleName[Cnt]);
		//TRACE(_T("[PWR] Cnt:%d %c\n"), Cnt, moduleInfo->modules->moduleName[Cnt]);
	}
	TRACE(_T("[PWR] CreateDevice. Error=%d ID:0x%X State:%d ModuleName(%d):%s\n"), err, id, engStatus->state, strModule.GetLength(), (LPCTSTR)strModule);
	if (strModule.CompareNoCase(_T("Simulation")) == 0)
	{
		TRACE(_T("[PWR] ----------------------------------------------------\n"));
		TRACE(_T("[PWR] Simulation Mode                                     \n"));
		TRACE(_T("[PWR] ----------------------------------------------------\n"));
		SetSkipMotorPower(true);
		SetGlobalSimulationMode(true);

	}
	else
	{
		TRACE(_T("[PWR] ----------------------------------------------------\n"));
		TRACE(_T("[PWR] Real Machine Control Mode                           \n"));
		TRACE(_T("[PWR] ----------------------------------------------------\n"));
		SetGlobalSimulationMode(false);

	}
	delete engStatus;
	engStatus = NULL;
	delete moduleInfo;
	moduleInfo = NULL;
	return err;
}

long Cwmx3Init::CreateIO()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3IO = new Io(m_wmx3Device);
	NewIO();
	if (m_wmx3IO == NULL)
	{
		TRACE(_T("[PWR] Failed to new Io"));
	}
	TRACE(_T("[PWR] CreateIODevice.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateCoreMotion()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3CoreMotion = new CoreMotion(m_wmx3Device);
	NewCoreMotion();	
	if (m_wmx3CoreMotion == NULL || g_CoreMotion == NULL)
	{
		TRACE(_T("[PWR] Failed to new CreateCoreMotion"));
	}	
	TRACE(_T("[PWR] CreateCoreMotion.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateCoreMotionStatus()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3CoreMotionStatus = new CoreMotionStatus();
	NewCoreMotionStatus();
	if (m_wmx3CoreMotionStatus == NULL || g_CoreMotionStatus == NULL)
	{
		TRACE(_T("[PWR] Failed to new CoreMotionStatus"));
	}	
	TRACE(_T("[PWR] CreateCoreMotionStatus.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateAdvancedMotion()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3AdvancedMotion = new AdvancedMotion(m_wmx3Device);
	NewAdvancedMotion();
	if (m_wmx3AdvancedMotion == NULL || g_AdvancedMotion == NULL)
	{
		TRACE(_T("[PWR] Failed to new CreateAdvancedMotion"));
	}
	TRACE(_T("[PWR] CreateAdvancedMotion.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateEventControl()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3EventControl = new EventControl(m_wmx3Device);
	NewEventControl();
	if (m_wmx3EventControl == NULL || g_EventControl == NULL)
	{
		TRACE(_T("[PWR] Failed to new CreateEventControl"));
	}
	TRACE(_T("[PWR] CreateEventControl.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateWmx3LogControl()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3Log = new Log(m_wmx3Device);
	NewWmx3Log();
	if (m_wmx3Log == NULL || g_Wmx3Log == NULL)
	{
		TRACE(_T("[PWR] Failed to new CreateWmx3LogControl"));
	}
	TRACE(_T("[PWR] CreateWmx3LogControl.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateEcat()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3Ecat = new Ecat(m_wmx3Device);
	NewEcat();
	if (m_wmx3Ecat == NULL || g_Ecat == NULL)
	{
		TRACE(_T("[PWR] Failed to new Ecat"));
	}
	TRACE(_T("[PWR] CreateEcAPI.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateCompensation()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3Compensation = new Compensation(m_wmx3Device);
	NewCompensation();
	if (m_wmx3Compensation == NULL || g_Compensation == NULL)
	{
		TRACE(_T("[PWR] Failed to new Compensation"));
	}
	TRACE(_T("[PWR] CreateCompensation.\n"));
	return NO_ERR;
}

long Cwmx3Init::CreateUserMemory()
{
	if (IsSimulationMode() == true) return NO_ERR;
	m_wmx3UserMemory = new UserMemory(m_wmx3Device);
	NewUserMemory();
	if (m_wmx3UserMemory == NULL || g_UserMemory == NULL)
	{
		TRACE(_T("[PWR] Failed to new UserMemory"));
	}
	TRACE(_T("[PWR] CreateUserMemory.\n"));
	return NO_ERR;
}

long Cwmx3Init::SetGlobalDevice()
{
	if (IsSimulationMode() == true) return NO_ERR;
	SetDevice(m_wmx3Device);
	TRACE(_T("[PWR] SetGlobalDevice.\n"));
	return NO_ERR;
}

long Cwmx3Init::SetGlobalWmx3Env()
{
	if (IsSimulationMode() == true) return NO_ERR;
	SetCoreMotion(m_wmx3CoreMotion);
	SetCoreMotionStatus(m_wmx3CoreMotionStatus);
	SetIO(m_wmx3IO);
	SetEcat(m_wmx3Ecat);
	SetCompensation(m_wmx3Compensation);
	TRACE(_T("[PWR] SetGlobalWmx3Env.\n"));
	return NO_ERR;
}

long Cwmx3Init::SetDeviceName(CString strDevName)
{	
	if (IsSimulationMode() == true) return NO_ERR;
	int err = ErrorCode::None;
	char chDevName[BUFSIZE];
	CStringA strVersion(strDevName);
	ZeroMemory(chDevName, BUFSIZE);
	memcpy(chDevName, strVersion.GetBuffer(), strVersion.GetLength());
	TRACE(_T("[PWR] SetDeviceName. Start\n"));
	// Set Device Name.
	err = m_wmx3Device->SetDeviceName(chDevName);
	if (err != ErrorCode::None)
	{
		DeviceErrorToString(err, _T("SetDeviceName"));
		SetInitError(true);
		return err;
	}
	m_Name = strDevName;
	TRACE(_T("[PWR] SetDeviceName(%s) Error=%d\n"), m_Name, err);
	return err;
}

//long Cwmx3Init::StartDeviceComm()
//{
//	if (IsSimulationMode() == true) return NO_ERR;
//	int err = ErrorCode::None;
//	// Start Communication.
//	TRACE(_T("[PWR] StartCommunication. Start\n"));
//	CApplicationTime* pTime = new CApplicationTime();
//	err = m_wmx3Device->StartCommunication(TIME10000MS); //Wait up to 10 seconds for communication to start
//	if (err != ErrorCode::None)
//	{
//		DeviceErrorToString(err, _T("StartCommunication"));
//		SetInitError(true);
//		return err;
//	}
//	TRACE(_T("[PWR] StartCommunication. Error=%d Elapsed:%d[ms]\n"), err, pTime->TimeElapsed());
//	delete pTime;
//	//ThreadSleep(TIME3000MS);		// Force Delay
//	return err;
//}

long Cwmx3Init::StartDeviceComm()
{
	if (IsSimulationMode() == true) return NO_ERR;
	int err = ErrorCode::None;
	// Start Communication.
	TRACE(_T("[PWR] StartCommunication. Start\n"));
	CApplicationTime* pTime = new CApplicationTime();

	for (long idx = 0; idx < 5; idx++)
	{
		err = m_wmx3Device->StartCommunication(TIME10000MS); //Wait up to 10 seconds for communication to start
		DeviceErrorToString(err, _T("StartCommunication"));

		if (err == ErrorCode::None)
		{
			TRACE(_T("[PWR] StartCommunication Complete. Error=%d Elapsed:%d[ms]\n"), err, pTime->TimeElapsed());
			delete pTime;
			return err;
		}
		else
		{
			TRACE(_T("[PWR] StartCommunication Fail. Retry:%d Error=%d Elapsed:%d[ms]\n"), idx, err, pTime->TimeElapsed());
			m_wmx3Device->StopCommunication(TIME10000MS);
			DeviceErrorToString(err, _T("StopCommunication"));
			TRACE(_T("[PWR] StopCommunication Fail. Retry:%d Error=%d Elapsed:%d[ms]\n"), idx, err, pTime->TimeElapsed());

			ThreadSleep(TIME5000MS);
		}
	}

	SetInitError(true);
	delete pTime;
	return err;
}

long Cwmx3Init::StopDeviceComm()
{
	if (IsSimulationMode() == true) return NO_ERR;
	int err = ErrorCode::None;
	// Stop Communication.
	TRACE(_T("[PWR] StopCommunication. Start\n"));
	CApplicationTime* pTime = new CApplicationTime();
	err = m_wmx3Device->StopCommunication(INFINITE);
	if (err != ErrorCode::None)
	{
		DeviceErrorToString(err, _T("StopCommunication"));
		SetInitError(true);
		return err;
	}
	TRACE(_T("[PWR] StopCommunication. Error=%d Elapsed:%d[ms]\n"), err, pTime->TimeElapsed());
	delete pTime;
	return err;
}

long Cwmx3Init::CloseDevice()
{
	if (IsSimulationMode() == true) return NO_ERR;
	int err = ErrorCode::None;
	//close device.
	TRACE(_T("[PWR] CloseDevice. Start\n"));
	err = m_wmx3Device->CloseDevice();
	if (err != ErrorCode::None)
	{
		DeviceErrorToString(err, _T("CloseDevice"));
		SetInitError(true);
		return err;
	}
	TRACE(_T("[PWR] CloseDevice. Error=%d\n"), err);
	return err;
}

long Cwmx3Init::ImportParameter()
{
	char cWmx3XmlFile[BUFSIZE];
	int err = ErrorCode::None;
	sprintf_s(cWmx3XmlFile, "%s%s%swmx_parameters.xml", ROOT_PATH, MCS_PATH, WMX3_PATH);
	TRACE(_T("[PWR] ImportParameter. Start\n"));
	err = GetCoreMotion()->config->Import(cWmx3XmlFile, gcWmx3SystemParam->m_SystemParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ImportParameter"));
		SetInitError(true);
		return err;
	}
	TRACE(_T("[PWR] ImportParameter. Error=%d\n"), err);
	return err;
}

long Cwmx3Init::ImportParameterAndSetAll()
{
	char cWmx3XmlFile[BUFSIZE];
	int err = ErrorCode::None;
	sprintf_s(cWmx3XmlFile, "%s%s%swmx_parameters.xml", ROOT_PATH, MCS_PATH, WMX3_PATH);
	TRACE(_T("[PWR] ImportParameterAndSetAll. Start\n"));
	err = GetCoreMotion()->config->ImportAndSetAll(cWmx3XmlFile, gcWmx3SystemParam->m_SystemParam);
	if (err != ErrorCode::None)
	{
		CoreMotionErrorToString(err, _T("ImportParameterAndSetAll"));
		SetInitError(true);
		return err;
	}
	TRACE(_T("[PWR] ImportParameterAndSetAll. Error=%d\n"), err);
	return err;
}

long Cwmx3Init::ExportParameter()
{
	long Err = NO_ERR;
	return Err;
}

long Cwmx3Init::DeleteAllWmx3Handle()
{
	TRACE(_T("[PWR] DeleteAllWmx3Handle Start\n"));
	delete m_wmx3Device;
	delete m_wmx3IO;
	delete m_wmx3CoreMotion;
	delete m_wmx3CoreMotionStatus;
	delete m_wmx3Ecat;
	delete m_wmx3Compensation; 
	delete m_wmx3EventControl;
	delete m_wmx3Log;
	delete m_wmx3UserMemory;
	TRACE(_T("[PWR] DeleteAllWmx3Handle ing\n"));
	m_wmx3Device = NULL;
	m_wmx3IO = NULL;
	m_wmx3CoreMotion = NULL;
	m_wmx3CoreMotionStatus = NULL;
	m_wmx3Ecat = NULL;
	m_wmx3Compensation = NULL;
	m_wmx3EventControl = NULL;
	m_wmx3Log = NULL;
	m_wmx3UserMemory = NULL;
	TRACE(_T("[PWR] DeleteAllWmx3Handle End\n"));
	return NO_ERR;
}

//void Cwmx3Init::TerminateThread()
//{
	//TRACE(_T("[PWR] Cwmx3Init TerminateThread SetEvent done\n"));
	//SetEvent(m_TerminteEvent);
//}

bool Cwmx3Init::GetInitError()
{
	bool bRet = false;
	bRet = m_bError;
	return bRet;
}

long Cwmx3Init::GetAndExportAll()
{
	char cWmx3XmlFile[BUFSIZE];
	int err = ErrorCode::None;
	sprintf_s(cWmx3XmlFile, "%s%s%swmx_parameters.xml", ROOT_PATH, MCS_PATH, WMX3_PATH);
	TRACE(_T("[PWR] GetAndExportAll. Start\n"));
	err = GetCoreMotion()->config->GetAndExportAll(cWmx3XmlFile);
	if (err != ErrorCode::None)
	{
		AfxMessageBox(_T("WMX3 Parameter Save Error"));

		CoreMotionErrorToString(err, _T("GetAndExportAll"));
		return err;
	}
	TRACE(_T("[PWR] GetAndExportAll. Error=%d\n"), err);
	return err;
}

void Cwmx3Init::SetInitError(bool bStatus)
{
	m_bError = bStatus;
	SetGlobalStatusError(bStatus);
}
