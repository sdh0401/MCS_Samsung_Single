#pragma once
#include <WMX3Api.h>
#include <ecApi.h>
#include <IOApi.h>
#include <CoreMotionApi.h>
#include <EventApi.h>
#include <CompensationApi.h>
#include <AdvancedMotionApi.h>
#include <LogApi.h>
#include <UserMemoryApi.h>
#include "GlobalDefine.h"

using namespace wmx3Api;
using namespace ecApi;
using namespace std;

class Cwmx3Init
{
public:
	Cwmx3Init(bool bSimulation, bool bReverse);
	~Cwmx3Init();
	long InitializeWmx3(CString strDeviceName);
	void Run();
	bool IsSimulationMode();
	void SetAsSimulation();
	void SetAsRealMachine();
	void SetInitError(bool bStatus);
	bool GetInitError();
	long GetAndExportAll();

private:
	long CreateDevice();
	long CreateIO();
	long CreateCoreMotion();
	long CreateCoreMotionStatus();
	long CreateAdvancedMotion();
	long CreateEventControl();
	long CreateWmx3LogControl();
	long CreateEcat();
	long CreateCompensation();
	long CreateUserMemory();
	long SetGlobalDevice();
	long SetGlobalWmx3Env();
	long SetDeviceName(CString strDevName);
	long CloseDevice();
	long StartDeviceComm();
	long StopDeviceComm();
	long CreateAllAxis();
	long SetInfoAllAxis();
	long DeleteAllAxis();
	long DeleteAllWmx3Handle();
	long ImportParameter();
	long ImportParameterAndSetAll();
	long ExportParameter();
	CString m_Name;
	WMX3Api* m_wmx3Device;
	Io* m_wmx3IO;
	CoreMotion* m_wmx3CoreMotion;
	CoreMotionStatus* m_wmx3CoreMotionStatus;
	AdvancedMotion* m_wmx3AdvancedMotion;
	Log* m_wmx3Log;
	EventControl* m_wmx3EventControl;
	Ecat* m_wmx3Ecat;
	Compensation* m_wmx3Compensation;
	UserMemory* m_wmx3UserMemory;

	bool m_bSimulation;
	char m_ErrString[BUFSIZE];
	bool m_bSafeQuit;
	bool m_bError;
	bool m_bReverse;
	int m_MaxAxisNo;
	int m_MaxUsedAxisNo;
	HANDLE m_MotorStartSyncEvent;
	HANDLE m_IOStartSyncEvent;
};

extern Cwmx3Init* gcWmx3Init;