#pragma once

#include "pch.h"
#include "GlobalData.h"
#include "PowerCalibration.h"
#include "PowerCalibrationDlg.h"

#include "Cwmx3Axis.h"
#include "Cwmx3Init.h"
#include "CHomeStatus.h"
#include "CioStatus.h"
#include "CMasterMotion.h"
#include "CMessageQueue.h"
#include "CLogFileSystem.h"

#include "CPowerHomeDlg.h"
#include "CPowerCamCalDlg.h"
#include "CPowerMasterStatusDlg.h"
#include "CPowerStackWalker.h"

#include "LockDef.h"
#include "GlobalDefine.h"
#include "Trace.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CPowerReliability.h"
#include "AxisInformation.h"

#include <WMX3Api.h>
#include <IOApi.h>
#include <CoreMotionApi.h>
#include <CompensationApi.h>
#include <EventApi.h>
#include <AdvancedMotionApi.h>
#include <LogApi.h>
#include <UserMemoryApi.h>
#include <windows.h>
#include <dbghelp.h>
#include <thread>
#include <iostream>
#include <fstream>

#pragma comment(lib,"dbghelp")

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace wmx3Api;
using namespace ecApi;
using namespace std;

bool g_bSimulationMode;
bool g_bMachineInitialized;
bool g_bRecvInitializeMachine;
bool g_bMachineStatusError;
long g_bMachineAlarmCode;
bool g_bAutoConnectUse;
bool g_bAutoHomingUse;
bool g_b1DCompensationUse;
bool g_b2DCompensationUse;
long g_2DCompensationMethod;
long g_GantryCalibrationMethod;
long g_Gantry2DMethod;
bool g_bZCompensationUse;
bool g_bSkipMotorPower;
bool g_MotorOffDoorOpen = false;
bool g_bInitY2Shift;
long g_MachineState;
long g_InfiniteDryRun;
long g_SettlingDelay;
bool g_bUserDoorPush;
bool g_bUserDoorClose;
long g_FirstPickup;
long g_SkipVision;
long g_SimulLoading;
long g_UsePathLinearIntpl;
long g_Use2StepZMotion;
long g_UseLineOfBalance;
long g_ReadyToRun;
long g_UseAreaSensor;
long g_UseAreaSensor2nd;
long g_UseRearFeeder;
bool g_UseTTF;
bool g_ManualConveyor;
long g_WorkExistSkip;
long g_UseRTDSensorFX;
long g_UseRTDSensorFY1;
long g_UseRTDSensorFY2;
long g_UseANC[MAX_ANC_BASE];
bool g_bY2CompensationUse;
long g_FrontCamCount;
long g_RearCamCount;
long g_FrontCamType;
long g_RearCamType;
long g_FrontCamLedChannel;
long g_RearCamLedChannel;
long g_FrontCamLaser;
long g_RearCamLaser;
bool g_bUseExitConvStopper;
ULONGLONG g_UserOpenDoorGetTime = 0, g_UserOpenDoorElapsed = 0;
bool m_MoveOnceZREnable = false;
bool g_bMoveCheckANCDownFront = true;
bool g_bMoveCheckANCDownRear = true;
bool g_bOnlyConveyorMode = false;
long g_CameraVersion = 0;
bool g_bBarcodeSimulation;
bool g_bUseHeadBarcodeCognex;
INSERT_ALARM_ACTION g_InsertTorquteAlarm;
Y2SHIFT_AUTOINIT g_Y2ShiftAutoInit;
bool g_bSimulationForming;
bool g_HeadSkipFront[MAXUSEDHEADNO] = {false, };
bool g_HeadSkipRear[MAXUSEDHEADNO] = {false, };
bool g_FormingDRBCoilUse = false;

TwoDPitchErrorCompensationData g_2DCompDataX;
TwoDPitchErrorCompensationData g_2DCompDataY1;
TwoDPitchErrorCompensationData g_2DCompDataY2;

MachineStruct		gPowerConfig;
double				gdblCompen[MAXTABLENO][BUFSIZE];
CameraData			Camera[MAXCAMNO];
VisualAlignInfo		VA[MAXCOMPONENTNO];
MarkInfo			Mark[MAXVISIONFILENO];
VisualOddAlignInfo 	OddVA[MAXODDVADBNO + 1];
BGABallArrayInfo 	BGABall[MAXODDVADBNO + 1];
VisualOddAlignInfo 	OddInsertPartVA[MAXINSERTPARTVADBNO + 1];
InsertDBVisionInfo  InsertPartVA[MAXINSERTPARTVADBNO + 1];
UWORD OptionSet[MAXOPTIONNO];
float MachineFSet[MAXMACHINENO];
UWORD MachineISet[MAXMACHINENO];
float MachineFSet2[MAXMACHINENO];
UWORD MachineISet2[MAXMACHINENO];
UWORD ExtOptionSet[MAXOPTIONNO_EXT];
UWORD OptionSet2nd[MAXOPTIONNO];
float MachineFSet3rd[MAXOPTIONNO];
UWORD MachineISet3rd[MAXOPTIONNO];
HANDLE g_Wmx3AxisLock;
CArray<Cwmx3Axis*, Cwmx3Axis*> g_pWmx3AxisArray;
HANDLE g_Wmx3LinearIntplLock;
HANDLE g_Wmx3LinearIntplLockR;
HANDLE g_Wmx3LinearIntplLockZ;
CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArray;
CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArrayR;
CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArrayZ;
HANDLE g_ThreadArrayLock;
CArray<CPowerThread*, CPowerThread*> g_pThreadArray;
HANDLE g_ThreadCleanArrayLock;
CArray<CPowerThread*, CPowerThread*> g_ThreadCleanArray;
HANDLE g_ThreadRunLock;
CArray<CPowerThread*, CPowerThread*> g_pThreadRunArray;
IOMAPSTRUCT gioMapStruct;
WMX3MASTER_INFO gMasterInfo;
WMX3_AXIS_STATUS gMotorStatus[MAXAXISNO];
//HANDLE gMOTION_LOCK[MAXAXISNO];
HANDLE gMACHINE_CONTROL;
HANDLE gRUN_MODE;
HANDLE gRUN_MOVEONETIME;
bool g_PauseMoveOneTime;

WMX3Api* g_Device;
Io* g_IO;
CoreMotion* g_CoreMotion;
AdvancedMotion* g_AdvancedMotion;
EventControl* g_EventControl;
Log* g_Wmx3Log;
Compensation* g_Compensation;
CoreMotionStatus* g_CoreMotionStatus;
Ecat* g_Ecat;
DevicesInfoA* g_DevInfo;
HANDLE g_CycleStopLock[2];

CString g_strRoot;
CLogFileSystem* g_LogWriter;
UserMemory* g_UserMemory;
HANDLE g_RunStepLock;

unsigned char* DISK_RUN_ADDR;
unsigned int DISK_ADDR_OFFSET = 0;
unsigned int DISK_OFFSET = 0x40000;
unsigned int DISK_SADDR = 0x00;
unsigned int DISK_BYTE = 0x01;
unsigned int DISK_BLOCK = 0x04;
unsigned char MACHINE_BLOCKDATA[MAX_MACHINEFILE_SIZE];
unsigned char RUN_BLOCKDATA[MAX_RUNFILE_SIZE];
unsigned char INSERTEND_BLOCKDATA[MAX_INSERTENDFILE_SIZE];
//CString gLastPickPcbName;
//INSERT gLastPickDataFront[MAXUSEDHEADNO];
//INSERT gLastPickDataRear[MAXUSEDHEADNO];

long g_ProdQuantity = 0;

struct _PowerAxisConfig {
	unsigned PowerAxisArray[MAXAXISNO];
	unsigned PowerAxisMap[MAXAXISNO];
	double PowerAxisResol[MAXAXISNO];
	double PowerAxisUnResol[MAXAXISNO];
	bool PowerAxisUseMaster[MAXAXISNO];
	unsigned PowerAxisSlaveWhat[MAXAXISNO];
} PowerAxisConfig;

////////////////////////////////////////////////////////////////////////
void SetGlobalSimulationMode(bool bSimul)
{
	g_bSimulationMode = bSimul;
	TRACE(_T("[PWR] Simulation Mode:%s\n"), g_bSimulationMode==true?_T("On"):_T("Off"));
}

bool GetGlobalSimulationMode()
{
	bool bRet = false;
	bRet = g_bSimulationMode;
	return bRet;
}

void SetInitializedMachine(bool bInit)
{
	g_bMachineInitialized = bInit;
	TRACE(_T("[PWR] SetInitializedMachine(%d)\n"), bInit);
}

bool GetInitializedMachine()
{
	TRACE(_T("[PWR] GetInitializedMachine(%d)\n"), g_bMachineInitialized);
	return g_bMachineInitialized;
}

void SetRecvInitializeMachine(bool bRecv)
{
	g_bRecvInitializeMachine = bRecv;
}

bool GetRecvInitializeMachine()
{
	return g_bRecvInitializeMachine;
}

void SetAutoConnectUse(bool bUse)
{
	g_bAutoConnectUse = bUse;
}

bool GetAutoConnectUse()
{
	return g_bAutoConnectUse;
}

void SetAutoHomingUse(bool bUse)
{
	g_bAutoHomingUse = bUse;
}

bool GetAutoHomingUse()
{
	return g_bAutoHomingUse;
}

void Set1DCompensationUse(bool bUse)
{
	g_b1DCompensationUse = bUse;
}

bool Get1DCompensationUse()
{
	return g_b1DCompensationUse;
}

void Set2DCompensationMethod(long Method)
{
	g_2DCompensationMethod = Method;
	TRACE(_T("[PWR] Set2DCompensationMethod(%s)\n"), Method == 0 ? _T("WMX3") : _T("Software"));
}

long Get2DCompensationMethod()
{
	return g_2DCompensationMethod;
}

void Set2DCompensationUse(bool bUse)
{
	g_b2DCompensationUse = bUse;
}

bool Get2DCompensationUse()
{
	return g_b2DCompensationUse;
}

void SetGantryCalibrationMethod(long Method)
{
	g_GantryCalibrationMethod = Method;
	TRACE(_T("[PWR] SetGantryCalibrationMethod(%d)\n"), g_GantryCalibrationMethod);
}

long GetGantryCalibrationMethod()
{
	return g_GantryCalibrationMethod;
}

void SetGantry2DMethod(long Method)
{
	g_Gantry2DMethod = Method;
	TRACE(_T("[PWR] SetGantry2DMethod(%d)\n"), g_Gantry2DMethod);
}

long GetGantry2DMethod()
{
	return g_Gantry2DMethod;
}

void SetZCompensationUse(bool bUse)
{
	g_bZCompensationUse = bUse;
}

bool GetZCompensationUse()
{
	return g_bZCompensationUse;
}

void SetSkipMotorPower(bool bSkip)
{
	g_bSkipMotorPower = bSkip;
	TRACE(_T("[PWR] SetSkipMotorPower bSkip:%d\n"), bSkip);
}

bool GetSkipMotorPower()
{
	return g_bSkipMotorPower;
}

void SetMotorOffDoorOpen(bool set)
{
	g_MotorOffDoorOpen = set;
	TRACE(_T("[PWR] SetMotorOffDoorOpen :%d\n"), set);
}

bool GetMotorOffDoorOpen()
{
	return g_MotorOffDoorOpen;
}

void SetInitY2Shift(bool bInit)
{
	g_bInitY2Shift = bInit;
	TRACE(_T("[PWR] SetInitY2Shift bInit:%d\n"), bInit);
}

bool GetInitY2Shift()
{
	return g_bInitY2Shift;
}

void SetGlobalStatusError(bool bStatus)
{
	g_bMachineStatusError = bStatus;
}

bool GetGlobalStatusError()
{
	bool bRet = false;
	bRet = g_bMachineStatusError;
	return bRet;
}

void SetMachineAlarmCode(long AlarmCode)
{
	if (g_bMachineAlarmCode != AlarmCode)
	{
		TRACE(_T("[PWR] SetMachineAlarmCode(%d)n"), AlarmCode);
	}
	g_bMachineAlarmCode = AlarmCode;
}
long GetMachineAlarmCode()
{
	return g_bMachineAlarmCode;
}

void SetMachineState(long State)
{
	g_MachineState = State;
	TRACE(_T("[PWR] @@@@@ SetMachineState(%d) @@@@@\n"), g_MachineState);
}

long GetMachineState()
{
	//TRACE(_T("[PWR] ***** GetMachineState(%d) *****\n"), g_MachineState);
	return g_MachineState;
}

void SetInfiniteDryRun(long Infinite)
{
	g_InfiniteDryRun = Infinite;
}

long GetInfiniteDryRun()
{
	return g_InfiniteDryRun;
}

void SetGlobalSettlingDelay(long SettlingDelay)
{
	//if (SettlingDelay > TIME50MS + TIME30MS)
	//{
	//	g_SettlingDelay = SettlingDelay;
	//	TRACE(_T("[PWR] SetGlobalSettlingDelay,%d\n"), g_SettlingDelay);
	//}
	//else
	//{
	//	g_SettlingDelay = TIME200MS;
	//	TRACE(_T("[PWR] SetGlobalSettlingDelay Minimum Delay,%d\n"), g_SettlingDelay);
	//}

	g_SettlingDelay = SettlingDelay;
	TRACE(_T("[PWR] SetGlobalSettlingDelay,%d\n"), g_SettlingDelay);
}

long GetGlobalSettlingDelay()
{
	return g_SettlingDelay;
}

void SetFirstPickup(long FirstPickup)
{
	g_FirstPickup = FirstPickup;
	TRACE(_T("[PWR] SetFirstPickup,%d\n"), g_FirstPickup);
}

long GetFirstPickup()
{
	return g_FirstPickup;
}

void SetSkipVision(long SkipVision)
{
	g_SkipVision = SkipVision;
	TRACE(_T("[PWR] SetSkipVision,%d\n"), g_SkipVision);
}

long GetSkipVision()
{
	return g_SkipVision;
}

void SetSimulLoading(long SimulLoading)
{
	g_SimulLoading = SimulLoading;
	TRACE(_T("[PWR] SetSimulLoading,%d\n"), g_SimulLoading);
}

long GetSimulLoading()
{
	return g_SimulLoading;
}

void SetUsePathLinearIntpl(long PathLinear)
{
	g_UsePathLinearIntpl = PathLinear;
	TRACE(_T("[PWR] SetUsePathLinearIntpl,%d\n"), g_UsePathLinearIntpl);
}

long GetUsePathLinearIntpl()
{
	return g_UsePathLinearIntpl;
}

void SetUse2StepZMotion(long TwoStepZMotion)
{
	g_Use2StepZMotion = TwoStepZMotion;
	TRACE(_T("[PWR] SetUse2StepZMotion,%d\n"), TwoStepZMotion);
}

long GetUse2StepZMotion()
{
	return g_Use2StepZMotion;
}

void SetUseLineOfBalance(long LineOfBalance)
{
	g_UseLineOfBalance = LineOfBalance;
	TRACE(_T("[PWR] SetUseLineOfBalance,%d\n"), LineOfBalance);
}

long GetUseLineOfBalance()
{
	return g_UseLineOfBalance;
}

void SetHMIReadyToRun(long ReadyToRun)
{
	g_ReadyToRun = ReadyToRun;
	TRACE(_T("[PWR] SetHMIReadyToRun,%d\n"), g_ReadyToRun);
}

long GetHMIReadyToRun()
{
	return g_ReadyToRun;
}


void SetUseAreaSensor(long AreaSensor)
{
	g_UseAreaSensor = AreaSensor;
	TRACE(_T("[PWR] SetUseAreaSensor,%d\n"), g_UseAreaSensor);
}

long GetUseAreaSensor()
{
	return g_UseAreaSensor;
}

void SetUseAreaSensor2nd(long AreaSensor)
{
	g_UseAreaSensor2nd = AreaSensor;
	TRACE(_T("[PWR] SetUseAreaSensor2nd,%d\n"), g_UseAreaSensor2nd);
}

long GetUseAreaSensor2nd()
{
	return g_UseAreaSensor2nd;
}

void SetRearFeederUse(long FeederUse)
{
	g_UseRearFeeder = FeederUse;
	TRACE(_T("[PWR] SetRearFeederUse,%d\n"), g_UseRearFeeder);
}

long GetRearFeederUse()
{
	return g_UseRearFeeder;
}

void SetTTFUse(bool Use)
{
	g_UseTTF = Use;
	TRACE(_T("[PWR] SetTTFUse,%d\n"), g_UseTTF);
}

bool GetTTFUse()
{
	return g_UseTTF;
}

void SetManualConvUse(bool Use)
{
	g_ManualConveyor = Use;
	TRACE(_T("[PWR] SetHandConvUse,%d\n"), g_ManualConveyor);
}

bool GetManualConvUse()
{
	return g_ManualConveyor;
}

void SetWorkExistSkip(long skip)
{
	g_WorkExistSkip = skip;
	TRACE(_T("[PWR] set clear insert,%d\n"), skip);
}

long GetWorkExistSkip()
{
	return g_WorkExistSkip;
}

void SetUseRTDSensorFX(long UseRTDSensorX)
{
	g_UseRTDSensorFX = UseRTDSensorX;
	TRACE(_T("[PWR] SetUseRTDSensorX,%d\n"), g_UseRTDSensorFX);
}

long GetUseRTDSensorFX()
{
	return g_UseRTDSensorFX;
}

void SetUseRTDSensorFY1(long UseRTDSensorY1)
{
	g_UseRTDSensorFY1 = UseRTDSensorY1;
	TRACE(_T("[PWR] SetUseRTDSensorY1,%d\n"), g_UseRTDSensorFY1);
}

long GetUseRTDSensorFY1()
{
	return g_UseRTDSensorFY1;
}

void SetUseRTDSensorFY2(long UseRTDSensorY2)
{
	g_UseRTDSensorFY2 = UseRTDSensorY2;
	TRACE(_T("[PWR] SetUseRTDSensorY2,%d\n"), g_UseRTDSensorFY2);
}

long GetUseRTDSensorFY2()
{
	return g_UseRTDSensorFY2;
}

void SetUseANC(long Base, long Use)
{
	if (Base == FRONT_STAGE)
	{
		g_UseANC[FRONT_STAGE] = Use;
		TRACE(_T("[PWR] SetUseANC FRONT_STAGE,%d\n"), g_UseANC[FRONT_STAGE]);
	}
	else
	{
		g_UseANC[REAR_STAGE] = Use;
		TRACE(_T("[PWR] SetUseANC REAR_STAGE,%d\n"), g_UseANC[REAR_STAGE]);
	}
}

long GetUseANC(long Base)
{
	if (Base == FRONT_STAGE)
	{
		return g_UseANC[FRONT_STAGE];
	}
	else
	{
		return g_UseANC[REAR_STAGE];
	}
}

void SetUseExitConvStopper(bool bUse)
{
	g_bUseExitConvStopper = bUse;
	TRACE(_T("[PWR] SetUseExitConvStopper,%d\n"), g_bUseExitConvStopper);
}

bool GetUseExitConvStopper()
{
	return g_bUseExitConvStopper;
}

void SetY2CompensationUse(bool bUse)
{
	g_bY2CompensationUse = bUse;
	TRACE(_T("[PWR] SetY2CompensationUse,%d\n"), g_bY2CompensationUse);
}

bool GetY2CompensationUse()
{
	return g_bY2CompensationUse;
}

void SetCameraCount(long Stage, long CamCount)
{
	if (Stage == FRONT_VISION)
	{
		g_FrontCamCount = CamCount;
	}
	else
	{
		g_RearCamCount = CamCount;
	}
}

long GetCameraCount(long Stage)
{
	if (Stage == FRONT_VISION)
	{
		return g_FrontCamCount;
	}
	else
	{
		return g_RearCamCount;
	}
}

void SetModuleCamType(long Gantry, long Type) // (1:3Ch, 2 : Bar Light)
{
	if (Gantry == FRONT_GANTRY)
	{
		g_FrontCamType = Type;
	}
	else
	{
		g_RearCamType = Type;
	}

	TRACE(_T("[PWR] SetModuleCamType Gantry%d Type%d\n"), Gantry, Type);
}

long GetModuleCamType(long Gantry) // (1:3Ch, 2:Bar Light)
{
	if (Gantry == FRONT_GANTRY)
	{
		return g_FrontCamType;
	}
	else
	{
		return g_RearCamType;
	}
}

void SetModuleCamLedChannel(long Gantry, long Type) // 3:Channel, 1:Channel
{
	if (Gantry == FRONT_GANTRY)
	{
		g_FrontCamLedChannel = Type;
	}
	else
	{
		g_RearCamLedChannel = Type;
	}

	TRACE(_T("[PWR] SetModuleCamLedChannel Gantry%d Type%d\n"), Gantry, Type);
}

long GetModuleCamLedChannel(long Gantry) // 3:Channel, 1:Channel
{
	if (Gantry == FRONT_GANTRY)
	{
		return g_FrontCamLedChannel;
	}
	else
	{
		return g_RearCamLedChannel;
	}
}

void SetModuleCamLaserUse(long Gantry, long Use) // 1:Use, 2:NoUse
{
	if (Gantry == FRONT_GANTRY)
	{
		g_FrontCamLaser = Use;
	}
	else
	{
		g_RearCamLaser = Use;
	}

	TRACE(_T("[PWR] SetModuleCamLaserUse Gantry%d Use%d\n"), Gantry, Use);
}

long GetModuleCamLaserUse(long Gantry) // 1:Use, 2:NoUse
{
	if (Gantry == FRONT_GANTRY)
	{
		return g_FrontCamLaser;
	}
	else
	{
		return g_RearCamLaser;
	}
}

void SetOnlyConveyorMode(bool set)
{
	CString strFunc(__func__);
	g_bOnlyConveyorMode = set;
	TRACE(_T("[PWR] %s,%d\n"), strFunc, set);
}
bool GetOnlyConveyorMode()
{
	return g_bOnlyConveyorMode;
}

void SetSimulationForming(bool bUse)
{
	g_bSimulationForming = bUse;
	TRACE(_T("[PWR] SimulationForming,%d\n"), g_bSimulationForming);
}

bool GetSimulationForming()
{
	return g_bSimulationForming;
}


void TraceStack(void)
{
	unsigned int   i;
	void* stack[100];
	unsigned short frames;
	SYMBOL_INFO* symbol;
	HANDLE process;

	process = GetCurrentProcess();
	SymInitialize(process, NULL, TRUE);
	frames = CaptureStackBackTrace(0, 100, stack, NULL);
	symbol = (SYMBOL_INFO*)calloc(sizeof(SYMBOL_INFO) + 256 * sizeof(char), 1);
	symbol->MaxNameLen = 255;
	symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
	for (i = 0; i < frames; i++)
	{
		SymFromAddr(process, (DWORD64)(stack[i]), 0, symbol);
		TRACE(_T("[PWR] %i: %S - 0x%X\n"), frames - i - 1, symbol->Name, symbol->Address);
	}
	free(symbol);
}

char* ConvertWchartoChar(wchar_t* str)
{
	//반환할 char* 변수 선언
	char* pStr;

	//입력받은 wchar_t 변수의 길이를 구함
	int strSize = WideCharToMultiByte(CP_ACP, 0, str, -1, NULL, 0, NULL, NULL);
	//char* 메모리 할당
	pStr = new char[strSize];

	//형 변환 
	WideCharToMultiByte(CP_ACP, 0, str, -1, pStr, strSize, 0, 0);
	return pStr;
}

bool IsNumber(CString strNum)
{
	int nStart = 0;
	bool bRet = true;
	int nCount = 0;
	if (strNum.GetLength() == 0)
		bRet = false;
	else if (strNum.GetLength() == 1)
	{
		if (isdigit(strNum[0]) == 0)
			bRet = false;
	}
	else
	{
		if (strNum[0] == '-') // 음수인 경우 시작
			nStart = 1;
		for (int indx = nStart; indx < strNum.GetLength(); ++indx)
		{
			if (strNum[indx] != '.')
			{
				if (isdigit(strNum[indx]) == 0)
				{
					bRet = false;
					break;
				}
			}
			else nCount++;
		}
	}
	if (nCount > 1) bRet = false;
	return true;
}

int ConvertCStringToInt(CString strNum)
{
	int nRet;
	ASSERT(IsNumber(strNum));
	nRet = _ttoi(strNum);
	return nRet;
}

double ConvertCStringToDouble(CString strNum)
{
	double dblRet;
	ASSERT(strNum.GetLength() != 0);
	dblRet = _wtof(strNum);
	return dblRet;
}

float ConvertCStringToFloat(CString strNum)
{
	float flRet;
	ASSERT(strNum.GetLength() != 0);
	flRet = (float)_ttof(strNum);
	return flRet;
}

////////////////////////////////////////////////////////////////////////
void ThreadArrayLock()
{
	SEM_LOCK(g_ThreadArrayLock, INFINITE);
}

void ThreadArrayUnlock()
{
	SEM_UNLOCK(g_ThreadArrayLock);
}

bool IsDoubleThread(CPowerThread* pThreadInfo)
{
	//CPowerThread* pThread = NULL;
	//CString strLog, strThreadName, strThreadName2;
	//if (pThreadInfo != NULL)
	//{
	//	for (INT_PTR indx = 0; indx < AllThreadCount(); ++indx)
	//	{
	//		pThread = GetThreadByIndex(indx);
	//		if (pThread != NULL)
	//		{
	//			strThreadName = pThread->GetThreadName();
	//			strThreadName2 = pThreadInfo->GetThreadName();
	//			if (pThread->GetThreadHandle() == pThreadInfo->GetThreadHandle()) // Same Handle
	//			{
	//				TRACE(_T("[PWR] Double Handle Old:0x%x New:0x%x\n"), pThread->GetThreadHandle(), pThreadInfo->GetThreadHandle());
	//				strLog.Format(_T("[PWR] Double Handle Old(%s):0x%x New(%s):0x%x"), strThreadName, pThread->GetThreadHandle(), strThreadName2, pThreadInfo->GetThreadHandle());
	//				gcPowerLog->Logging(strLog);
	//				return true;
	//			}
	//			if (pThread->GetThreadID() == pThreadInfo->GetThreadID()) // Same ID
	//			{
	//				TRACE(_T("[PWR] Double ID Old:0x%x New:0x%x\n"), pThread->GetThreadID(), pThreadInfo->GetThreadID());
	//				strLog.Format(_T("[PWR] Double Handle Old(%s):0x%x New(%s):0x%x"), strThreadName, pThread->GetThreadID(), strThreadName2, pThreadInfo->GetThreadID());
	//				gcPowerLog->Logging(strLog);
	//				return true;
	//			}
	//			if (strThreadName.CompareNoCase(strThreadName2) == 0) // Same
	//			{
	//				TRACE(_T("[PWR] Double Thread Name\n"));
	//				strLog.Format(_T("[PWR] Double Handle Old(%s):0x%x New(%s):0x%x"), strThreadName, strThreadName2);
	//				gcPowerLog->Logging(strLog);
	//				return true;
	//			}
	//		}
	//	}
	//}
	return false;
}

void AddThread(CPowerThread* pThreadInfo)
{
	ThreadArrayLock();
	g_pThreadArray.Add(pThreadInfo);
	ThreadArrayUnlock();
	TRACE(_T("[PWR] (%d) AddThread %s ******************************************* \n"), AllThreadCount(), pThreadInfo->GetThreadName());
	
}

INT_PTR AllThreadCount()
{
	INT_PTR retCount;
	retCount = g_pThreadArray.GetCount();
	return retCount;
}

CString GetThreadNameByID(int nID)
{
	CPowerThread* pThread = NULL;
	CString strName = _T("NULL");
	strName.Empty();
	ThreadArrayLock();
	for (INT_PTR indx = 0; indx < g_pThreadArray.GetCount(); indx++)
	{
		pThread = g_pThreadArray.GetAt(indx);
		if (pThread != NULL)
		{
			if (pThread->GetThreadID() == nID)
			{
				strName = pThread->GetThreadName();
				break;
			}
		}
	}
	ThreadArrayUnlock();
	if (strName.IsEmpty() == true)
	{
		return strName;
	}
	return strName;
}

CPowerThread* GetThreadByIndex(INT_PTR indx)
{
	CPowerThread* pThread = NULL;
	ThreadArrayLock();
	//ASSERT(indx >= 0 && indx < g_pThreadArray.GetCount());
	pThread = g_pThreadArray.GetAt(indx);
	ThreadArrayUnlock();
	return pThread;
}

bool IsThreadByName(CString ThreadName)
{
	CPowerThread* pThread = NULL;
	CString strThreadName;
	ThreadArrayLock();
	TRACE(_T("[PWR] IsThreadByName GetCount:%d\n"), g_pThreadArray.GetCount());
	for (INT_PTR indx = 0; indx < g_pThreadArray.GetCount(); indx++)
	{
		pThread = g_pThreadArray.GetAt(indx);
		if (pThread != NULL)
		{
			strThreadName = pThread->GetThreadName();
			if (strThreadName.IsEmpty() == true) continue;
			TRACE(_T("[PWR] IsThreadByName %s\n"), strThreadName);
			if (strThreadName.CompareNoCase(ThreadName) == 0)
			{
				ThreadArrayUnlock();
				return true;
			}
		}
	}
	ThreadArrayUnlock();
	return false;
}

CPowerThread* GetThreadByName(CString ThreadName)
{
	CPowerThread* pThread = NULL;
	CPowerThread* retThread = NULL;
	CString strThreadName;
	ThreadArrayLock();
	for (INT_PTR indx = 0; indx < g_pThreadArray.GetCount(); indx++)
	{
		pThread = g_pThreadArray.GetAt(indx);
		if (pThread != NULL)
		{
			strThreadName = pThread->GetThreadName();
			if (strThreadName.CompareNoCase(ThreadName) == 0)
			{
				retThread = pThread;
				break;
			}
		}
	}
	ThreadArrayUnlock();
	return retThread;
}

void DeleteThread(int thID)
{
	INT_PTR nindx = 0; 
	//THREAD_STRUCT* pThread = NULL;
	nindx = GetIndexByThreadID(thID);
	if (nindx != NOT_FOUND_THREAD_ID)
	{
		//pThread = g_pThreadArray.GetAt(nindx);
		DeleteThreadByIndex(nindx);
		//delete pThread;
	}
}

INT_PTR GetIndexByThreadID(int thID)
{
	CPowerThread* pThread = NULL;
	INT_PTR indx = 0;
	for (indx = 0; indx < g_pThreadArray.GetCount(); indx++)
	{
		pThread = GetThreadByIndex(indx);
		if (pThread == NULL) continue;
		if (pThread->GetThreadID() == thID)
		{
			return indx;
		}
	}
	return NOT_FOUND_THREAD_ID;
}

void DeleteThreadByIndex(INT_PTR indx)
{
	ThreadArrayLock();
	g_pThreadArray.RemoveAt(indx);
	ThreadArrayUnlock();
}

bool gIsAliveThread(HANDLE mThread)
{
	HANDLE h_Handle = NULL;
	h_Handle = mThread;
	if (h_Handle == INVALID_HANDLE_VALUE)
	{
		TRACE(_T("[PWR] gIsAliveThread Handle is invalid\n"));
		return false;
	}
	if (::WaitForSingleObject(h_Handle, 0) == WAIT_TIMEOUT)
	{
		// 현재 쓰레드가 실행 중.
		//TRACE(_T("[PWR] Thread is running\n"));
		return true;
	}
	else
	{
		// 실행 중인 상태가 아니다.
		//TRACE(_T("[PWR] Thread is NOT running\n"));
		return false;
	}
}
////////////////////////////////////////////////////////////////////////
void ThreadCleanArrayLock()
{
	SEM_LOCK(g_ThreadCleanArrayLock, INFINITE);
}

void ThreadCleanArrayUnlock()
{
	SEM_UNLOCK(g_ThreadCleanArrayLock);
}

void AddCleanThread(CPowerThread* pThread)
{
	ThreadCleanArrayLock();
	g_ThreadCleanArray.Add(pThread);
	ThreadCleanArrayUnlock();
}

INT_PTR GetCleanThreadCount()
{
	ThreadCleanArrayLock();
	INT_PTR Count = 0;
	Count = g_ThreadCleanArray.GetCount();
	ThreadCleanArrayUnlock();
	return Count;
}

CPowerThread* GetCleanThread(INT_PTR index)
{
	CPowerThread* pThread = NULL;
	ThreadCleanArrayLock();
	ASSERT(index >= 0 && index < g_ThreadCleanArray.GetCount());
	pThread = g_ThreadCleanArray.GetAt(index);
	ThreadCleanArrayUnlock();
	return pThread;
}

void DestroyCleanThread(INT_PTR index)
{
	CPowerThread* pThread = NULL;
	ThreadCleanArrayLock();
	pThread = GetCleanThread(index);
	g_ThreadCleanArray.RemoveAt(index);
	delete pThread;
	pThread = NULL;
	ThreadCleanArrayUnlock();		
}

////////////////////////////////////////////////////////////////////////
void RunThreadLock()
{
	SEM_LOCK(g_ThreadRunLock, INFINITE);
}

void RunThreadUnlock()
{
	SEM_UNLOCK(g_ThreadRunLock);
}

void PCBWaitLock()
{
	//SEM_LOCK(g_ThreadPcbReady, INFINITE);
}

void PCBWaitUnLock()
{
	//SEM_UNLOCK(g_ThreadPcbReady);
}

void PCBWaitReset()
{
	//ResetEvent(g_ThreadPcbReady);
}

void AddRunThread(CPowerThread* pThread)
{
	RunThreadLock();
	g_pThreadRunArray.Add(pThread);
	RunThreadUnlock();
}

INT_PTR GetRunThreadCount()
{
	RunThreadLock();
	INT_PTR Count = 0;
	Count = g_pThreadRunArray.GetCount();
	RunThreadUnlock();
	return Count;
}

CPowerThread* GetRunThread(INT_PTR index)
{
	CPowerThread* pThread = NULL;
	RunThreadLock();
	ASSERT(index >= 0 && index < g_pThreadRunArray.GetCount());
	pThread = g_pThreadRunArray.GetAt(index);
	RunThreadUnlock();
	return pThread;
}

void DestroyRunThread(INT_PTR index)
{
	CPowerThread* pThread = NULL;
	RunThreadLock();
	pThread = GetRunThread(index);
	AddCleanThread(pThread);
	g_pThreadRunArray.RemoveAt(index);
	RunThreadUnlock();
}

////////////////////////////////////////////////////////////////////////
void Wmx3Lock()
{
	SEM_LOCK(g_Wmx3AxisLock, INFINITE);
}

void Wmx3Unlock()
{
	SEM_UNLOCK(g_Wmx3AxisLock);
}

bool AddWmx3Axis(Cwmx3Axis* pAxis)
{
	Wmx3Lock();
	g_pWmx3AxisArray.Add(pAxis);
	Wmx3Unlock();
	return true;
}

bool RemoveWmx3Axis(INT_PTR indx)
{
	Wmx3Lock();
	Cwmx3Axis* pAxis = NULL;
	pAxis = g_pWmx3AxisArray.GetAt(indx);
	if (pAxis != NULL)
	{
		g_pWmx3AxisArray.RemoveAt(indx);
		delete pAxis;
	}
	else
	{
		Wmx3Unlock();
		return false;
	}
	Wmx3Unlock();
	return true;
}

INT_PTR GetWmx3AxisCount()
{
	INT_PTR retCount = 0;
	Wmx3Lock();
	retCount = g_pWmx3AxisArray.GetCount();
	Wmx3Unlock();
	return retCount;
}

Cwmx3Axis* GetWmx3AxisByIndex(INT_PTR indx)
{
	Cwmx3Axis* pAxis = NULL;
	Wmx3Lock();
	ASSERT(indx >= 0 && indx < g_pWmx3AxisArray.GetCount());
	pAxis = g_pWmx3AxisArray.GetAt(indx);
	Wmx3Unlock();
	return pAxis;
}

Cwmx3Axis* GetWmx3AxisByName(CString AxisName)
{
	bool bFind = false;
	Cwmx3Axis* pAxis = NULL;
	Cwmx3Axis* retAxis = NULL;
	Wmx3Lock();
	for (INT_PTR indx = 0; indx < g_pWmx3AxisArray.GetCount(); indx++)
	{
		pAxis = g_pWmx3AxisArray.GetAt(indx);
		if (pAxis != NULL)
		{
			CString strGetAxis = pAxis->GetAxisName();
			if (AxisName.CompareNoCase(strGetAxis) == 0)
			{
				bFind = true;
				retAxis = pAxis;
				break;
			}
		}
	}
	Wmx3Unlock();
	return retAxis;
}

Cwmx3Axis* GetWmx3AxisByAxisNo(long AxisNo)
{
	bool bFind = false;
	Cwmx3Axis* pAxis = NULL;
	Cwmx3Axis* retAxis = NULL;
	Wmx3Lock();
	for (INT_PTR indx = 0; indx < g_pWmx3AxisArray.GetCount(); indx++)
	{
		pAxis = g_pWmx3AxisArray.GetAt(indx);
		if (pAxis != NULL)
		{
			long AxisIndex = pAxis->GetAxisIndex();
			if (AxisNo == AxisIndex)
			{
				bFind = true;
				retAxis = pAxis;
				break;
			}
		}
	}
	Wmx3Unlock();
	return retAxis;
}
////////////////////////////////////////////////////////////////////////
void Wmx3LinearIntpLock()
{
	SEM_LOCK(g_Wmx3LinearIntplLock, INFINITE);
}

void Wmx3LinearIntpUnlock()
{
	SEM_UNLOCK(g_Wmx3LinearIntplLock);
}

bool AddWmx3LinearIntpAxis(StartPosStruct* pStartPos)
{
	Wmx3LinearIntpLock();
	g_pWmx3LinearIntplAxisArray.Add(pStartPos);
	Wmx3LinearIntpUnlock();
	return true;
}

bool RemoveWmx3LinearIntpAxis(INT_PTR indx)
{
	Wmx3LinearIntpLock();
	StartPosStruct* pStartPos = NULL;
	pStartPos = g_pWmx3LinearIntplAxisArray.GetAt(indx);
	if (pStartPos != NULL)
	{
		g_pWmx3LinearIntplAxisArray.RemoveAt(indx);
		delete pStartPos;
	}
	Wmx3LinearIntpUnlock();
	return true;
}

INT_PTR GetWmx3LinearIntpAxisCount()
{
	INT_PTR retCount = 0;
	Wmx3LinearIntpLock();
	retCount = g_pWmx3LinearIntplAxisArray.GetCount();
	Wmx3LinearIntpUnlock();
	return retCount;
}

StartPosStruct* GetWmx3LinearIntpAxisByIndex(INT_PTR indx)
{
	Wmx3LinearIntpLock();
	StartPosStruct* pStartPos = NULL;
	ASSERT(indx >= 0 && indx < g_pWmx3LinearIntplAxisArray.GetCount());
	pStartPos = g_pWmx3LinearIntplAxisArray.GetAt(indx);
	Wmx3LinearIntpUnlock();
	return pStartPos;
}

StartPosStruct* GetWmx3LinearIntpAxisByName(CString AxisName)
{
	bool bFind = false;
	StartPosStruct* pStartPos = NULL;
	StartPosStruct* retAxis = NULL;
	Wmx3LinearIntpLock();
	for (INT_PTR indx = 0; indx < g_pWmx3LinearIntplAxisArray.GetCount(); indx++)
	{
		pStartPos = g_pWmx3LinearIntplAxisArray.GetAt(indx);
		if (pStartPos != NULL)
		{
			CString strGetAxis = pStartPos->StrAxis;
			if (AxisName.CompareNoCase(strGetAxis) == 0)
			{
				bFind = true;
				retAxis = pStartPos;
				break;
			}
		}
	}
	Wmx3LinearIntpUnlock();
	return retAxis;
}
////////////////////////////////////////////////////////////////////////
void Wmx3LinearIntpLockR()
{
	SEM_LOCK(g_Wmx3LinearIntplLockR, INFINITE);
}

void Wmx3LinearIntpUnlockR()
{
	SEM_UNLOCK(g_Wmx3LinearIntplLockR);
}

bool AddWmx3LinearIntpAxisR(StartPosStruct* pStartPos)
{
	Wmx3LinearIntpLockR();
	g_pWmx3LinearIntplAxisArrayR.Add(pStartPos);
	Wmx3LinearIntpUnlockR();
	return true;
}

bool RemoveWmx3LinearIntpAxisR(INT_PTR indx)
{
	Wmx3LinearIntpLockR();
	StartPosStruct* pStartPos = NULL;
	pStartPos = g_pWmx3LinearIntplAxisArrayR.GetAt(indx);
	if (pStartPos != NULL)
	{
		g_pWmx3LinearIntplAxisArrayR.RemoveAt(indx);
		delete pStartPos;
	}
	Wmx3LinearIntpUnlockR();
	return true;
}

INT_PTR GetWmx3LinearIntpAxisRCount()
{
	INT_PTR retCount = 0;
	Wmx3LinearIntpLockR();
	retCount = g_pWmx3LinearIntplAxisArrayR.GetCount();
	Wmx3LinearIntpUnlockR();
	return retCount;
}

StartPosStruct* GetWmx3LinearIntpAxisRByIndex(INT_PTR indx)
{
	Wmx3LinearIntpLockR();
	StartPosStruct* pStartPos = NULL;
	ASSERT(indx >= 0 && indx < g_pWmx3LinearIntplAxisArrayR.GetCount());
	pStartPos = g_pWmx3LinearIntplAxisArrayR.GetAt(indx);
	Wmx3LinearIntpUnlockR();
	return pStartPos;
}

StartPosStruct* GetWmx3LinearIntpAxisRByName(CString AxisName)
{
	bool bFind = false;
	StartPosStruct* pStartPos = NULL;
	StartPosStruct* retAxis = NULL;
	Wmx3LinearIntpLockR();
	for (INT_PTR indx = 0; indx < g_pWmx3LinearIntplAxisArrayR.GetCount(); indx++)
	{
		pStartPos = g_pWmx3LinearIntplAxisArrayR.GetAt(indx);
		if (pStartPos != NULL)
		{
			CString strGetAxis = pStartPos->StrAxis;
			if (AxisName.CompareNoCase(strGetAxis) == 0)
			{
				bFind = true;
				retAxis = pStartPos;
				break;
			}
		}
	}
	Wmx3LinearIntpUnlockR();
	return retAxis;
}
////////////////////////////////////////////////////////////////////////
void NewDevice()
{
	g_Device = new WMX3Api();
}

void SetDevice(WMX3Api* wmx3Api)
{
	*g_Device = *wmx3Api;
}

WMX3Api* GetDevice()
{
	WMX3Api* pDevice;
	if (g_Device->dev == NULL)
	{
		TRACE(_T("[PWR] g_Device->dev Handle is NULL\n"));
		return NULL;
	}
	pDevice = (WMX3Api*)g_Device;
	return pDevice;
}

void DeleteDeivce()
{
	delete g_Device;
	g_Device = NULL;
}

void NewIO()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	if (pDevice != NULL)
	{
		g_IO = new Io(pDevice);
	}
}

void SetIO(Io* wmx3Io)
{
	*g_IO = *wmx3Io;
}

Io* GetIO()
{
	Io* pIO;
	ASSERT(g_IO->IsDeviceValid() == true);
	pIO = (Io*)g_IO;
	return pIO;
}

void DeleteIO()
{
	delete g_IO;
	g_IO = NULL;
}

void NewCoreMotion()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	g_CoreMotion = new CoreMotion(pDevice);
}

void SetCoreMotion(CoreMotion* coreMotion)
{
	*g_CoreMotion = *coreMotion;
}

CoreMotion* GetCoreMotion()
{
	CoreMotion* pCoreMotion;
	ASSERT(g_CoreMotion->IsDeviceValid() == true);
	pCoreMotion = (CoreMotion*)g_CoreMotion;
	return pCoreMotion;
}

void DeleteCoreMotion()
{
	delete g_CoreMotion;
	g_CoreMotion = NULL;
}

void NewAdvancedMotion()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	g_AdvancedMotion = new AdvancedMotion(pDevice);
}

void SetAdvancedMotion(AdvancedMotion* coreMotion)
{
	*g_AdvancedMotion = *coreMotion;
}

AdvancedMotion* GetAdvancedMotion()
{
	AdvancedMotion* pAdvancedMotion;
	ASSERT(g_AdvancedMotion->IsDeviceValid() == true);
	pAdvancedMotion = (AdvancedMotion*)g_AdvancedMotion;
	return pAdvancedMotion;
}

void DeleteAdvancedMotion()
{
	delete g_AdvancedMotion;
	g_AdvancedMotion = NULL;
}

void NewEventControl()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	g_EventControl = new EventControl(pDevice);
}

void SetEventControl(EventControl* eventControl)
{
	*g_EventControl = *eventControl;
}

EventControl* GetEventControl()
{
	EventControl* pEventControl;
	ASSERT(g_EventControl->IsDeviceValid() == true);
	pEventControl = (EventControl*)g_EventControl;
	return pEventControl;
}

void DeleteEventControl()
{
	delete g_EventControl;
	g_EventControl = NULL;
}

long GetMaxEventControl()
{
	return 512;
}

void NewWmx3Log()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	g_Wmx3Log = new Log(pDevice);
}

void SetWmx3Log(Log* Wmx3Log)
{
	*g_Wmx3Log = *Wmx3Log;
}

Log* GetWmx3Log()
{
	Log* pWmx3Log;
	ASSERT(g_EventControl->IsDeviceValid() == true);
	pWmx3Log = (Log*)g_Wmx3Log;
	return pWmx3Log;
}

void DeleteWmx3Log()
{
	delete g_Wmx3Log;
	g_Wmx3Log = NULL;
}

void NewCoreMotionStatus()
{
	g_CoreMotionStatus = new CoreMotionStatus();
	TRACE(_T("[PWR] NewCoreMotionStatus:0x%X\n"), g_CoreMotionStatus);
}

void SetCoreMotionStatus(CoreMotionStatus* coreMotionStatus)
{
	memcpy(g_CoreMotionStatus, coreMotionStatus, sizeof(CoreMotionStatus*));
	TRACE(_T("[PWR] SetCoreMotionStatus:0x%X\n"), *coreMotionStatus);
}

CoreMotionStatus* GetCoreMotionStatus()
{
	return g_CoreMotionStatus;
}

void DeleteCoreMotionStatus()
{
	delete g_CoreMotionStatus;
	g_CoreMotionStatus = NULL;
}

void NewEcat()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	if (pDevice != NULL)
	{
		g_Ecat = new Ecat(pDevice);
	}
}

void SetEcat(Ecat* eCat)
{
	*g_Ecat = *eCat;
}

Ecat* GetEcat()
{
	return g_Ecat;
}

void DeleteEcat()
{
	delete g_Ecat;
	g_Ecat = NULL;
}

void NewCompensation()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	if (pDevice != NULL)
	{
		g_Compensation = new Compensation(pDevice);
		g_2DCompDataX = TwoDPitchErrorCompensationData();
		g_2DCompDataY1 = TwoDPitchErrorCompensationData();
		g_2DCompDataY2 = TwoDPitchErrorCompensationData();
	}
}

void SetCompensation(Compensation* pCompensation)
{
	*g_Compensation = *pCompensation;
}

Compensation* GetCompensation()
{
	Compensation* pCompensation;
	ASSERT(g_Compensation->IsDeviceValid() == true);
	pCompensation = (Compensation*)g_Compensation;
	return pCompensation;
}

void DeleteCompensation()
{
	delete g_Compensation;
	g_Compensation = NULL;
}

void NewUserMemory()
{
	WMX3Api* pDevice;
	pDevice = GetDevice();
	if (pDevice != NULL)
	{
		g_UserMemory = new UserMemory(pDevice);
	}
}

void SetUserMemory(UserMemory* pUserMemory)
{
	*g_UserMemory = *pUserMemory;
}

UserMemory* GetUserMemory()
{
	UserMemory* pUserMemory;
	ASSERT(g_Compensation->IsDeviceValid() == true);
	pUserMemory = (UserMemory*)g_UserMemory;
	return pUserMemory;
}

void DeleteUserMemory()
{
	delete g_UserMemory;
	g_UserMemory = NULL;
}

unsigned DeviceErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetDevice()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned IOErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetIO()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned EcatErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetEcat()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned CoreMotionErrorToString(int AxisMap, unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetCoreMotion()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] AxisNo:%d Failed to %s. Error=%d (%S)\n"), AxisMap, strCmd, errCode, chErrString);
	}
	return err;
}

unsigned CoreMotionErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetCoreMotion()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned AdvancedMotionErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetAdvancedMotion()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned EventControlErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetEventControl()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned LogErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetWmx3Log()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned CompensationErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetCompensation()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

unsigned UserMemoryErrorToString(unsigned errCode, CString strCmd)
{
	int err = ErrorCode::None;
	char chErrString[BUFSIZE];
	ZeroMemory(&chErrString, sizeof(chErrString));
	//if (GetGlobalSimulationMode() == true)
	//{
	//	return err;
	//}
	//else
	{
		err = GetUserMemory()->ErrorToString(errCode, chErrString, sizeof(chErrString));
		TRACE(_T("[PWR] Failed to %s. Error=%d (%S)\n"), strCmd, errCode, chErrString);
	}
	return err;
}

long Set2DCompensationData(long Channel, TwoDPitchErrorCompensationData* pitchErrCompData)
{
	long err = ErrorCode::None;
	err = GetCompensation()->Set2DPitchErrorCompensation(Channel, pitchErrCompData);
	if (err != ErrorCode::None)
	{
		CompensationErrorToString(err, _T("Set2DPitchErrorCompensation"));
	}
	return err;
}

long Enable2D(long Channel)
{
	long err = ErrorCode::None;
	err = GetCompensation()->Enable2DPitchErrorCompensation(Channel);
	if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsEnabled)
	{
		CompensationErrorToString(err, _T("Enable2DPitchErrorCompensation"));
	}
	return err;
}

long Disable2D(long Channel)
{
	long err = ErrorCode::None;
	err = GetCompensation()->Disable2DPitchErrorCompensation(Channel);
	if (err != ErrorCode::None && err != CompensationErrorCode::PitchErrorCompensationIsDisabled)
	{
		CompensationErrorToString(err, _T("Disable2DPitchErrorCompensation"));
	}
	return err;
}

CString GetOperatingState(unsigned OpState)
{
	CString strOP_State[16] = {
		{_T("Idle")}, {_T("Pos")}, {_T("Jog")} , {_T("Home")} , {_T("Sync")},  {_T("GantryHome")},
		{_T("Stop")}, {_T("Intpl")}, {_T("ConstLinearVelocity")} , {_T("Trq")} , {_T("DirectControl")},
		{_T("PVT")},  {_T("ECAM")}, {_T("SyncCatchUp")} , {_T("DancerControl")} , {_T("OperationStateMax")}
	};
	return strOP_State[OpState];
}

CString GetAxisNameByAxisIndex(unsigned Axis)
{
	return PowerAxisAliasName[Axis];
	CString strAxisName[MAXAXISNO] = {
		//////////////////////////////////////////////// Front
		{_T("FX")}, {_T("FY1")}, {_T("FY2")},	
		{_T("FZ1")}, {_T("FZ2")}, {_T("FZ3")}, {_T("FZ4")}, {_T("FZ5")}, {_T("FZ6")},
		{_T("FW1")}, {_T("FW2")}, {_T("FW3")}, {_T("FW4")}, {_T("FW5")}, {_T("FW6")}, 
		{_T("FCONV")}, {_T("FPUZ")},
		{_T("FBTIN")}, {_T("FBTWK")}, {_T("FBTOT")},
		//////////////////////////////////////////////// Rear
		{_T("RX")}, {_T("RY1")}, {_T("RY2")},
		{_T("RZ1")}, {_T("RZ2")}, {_T("RZ3")}, {_T("RZ4")}, {_T("RZ5")}, {_T("RZ6")},
		{_T("RW1")}, {_T("RW2")}, {_T("RW3")}, {_T("RW4")}, {_T("RW5")}, {_T("RW6")},
		{_T("RCONV")}, {_T("RPUZ")},
		{_T("RBTIN")}, {_T("RBTWK")}, {_T("RBTOT")},
		//////////////////////////////////////////////// Etc
		{_T("CLX")}, {_T("CLY")}, {_T("CLZ")}, {_T("CLR") }, {_T("CLC")},
		{_T("RES1")}, {_T("RES2")}, {_T("RES3")}, {_T("RES4") }, {_T("RES5")}
	};
	return strAxisName[Axis];
}

void PrintMotorStatus(CString strName, CoreMotionAxisStatus cmAxis)
{
	TRACE(_T("[PWR] %s  1.PosCmd    :%-11.3f\n"), strName, cmAxis.posCmd);          // 1. PosCmd
	TRACE(_T("[PWR] %s  2.ActPos    :%-11.3f\n"), strName, cmAxis.actualPos);       // 2. ActualPos
	TRACE(_T("[PWR] %s  3.ActVel    :%-11.3f\n"), strName, cmAxis.actualVelocity);  // 3. ActualVelocity
	TRACE(_T("[PWR] %s  4.ActTrq    :%-11.3f\n"), strName, cmAxis.actualTorque);    // 4. ActualTorque 
	TRACE(_T("[PWR] %s  5.AmpAlm    :%-3d\n"), strName, cmAxis.ampAlarm);           // 5. AmpAlarm
	TRACE(_T("[PWR] %s  6.AmpAlmCode:0x%05x\n"), strName, cmAxis.ampAlarmCode);     // 6. AmpAlarmCode 
	TRACE(_T("[PWR] %s  7.SrvOn     :%-3d\n"), strName, cmAxis.servoOn);            // 7. ServoOn
	TRACE(_T("[PWR] %s  8.HomeDone  :%-4d\n"), strName, cmAxis.homeDone);           // 8. HomeDone 
	TRACE(_T("[PWR] %s  9.InPos     :%-3d\n"), strName, cmAxis.inPos);              // 9. InPos
	TRACE(_T("[PWR] %s 10.NegLS     :%-3d\n"), strName, cmAxis.negativeLS);         // 10.negativeLS
	TRACE(_T("[PWR] %s 11.PosLS     :%-3d\n"), strName, cmAxis.positiveLS);         // 11.positiveLS
	TRACE(_T("[PWR] %s 12.HomeSw    :%-4d\n"), strName, cmAxis.homeSwitch);         // 12.homeSwitch
	TRACE(_T("[PWR] %s 13.OpState   :%s\n"), strName, GetOperatingState(cmAxis.opState)); // 13.OperationState
}

CString GetCurrentDateTime()
{
	time_t     now = time(0);
	struct tm  tstruct;
	CString str;
	if (localtime_s(&tstruct, &now) != 0)
	{
		TRACE(_T("[PWR] GetCurrentDateTime Failed"));
		return _T("");
	}
	str.Format(_T("%04d%02d%02d_%02d%02d%02d"), tstruct.tm_year + 1900, tstruct.tm_mon + 1, tstruct.tm_mday, tstruct.tm_hour, tstruct.tm_min, tstruct.tm_sec);
	return str;
}

ULONGLONG _time_get()
{
	return(GetTickCount64());
}

ULONGLONG _time_elapsed(ULONGLONG iTime)
{
	ULONGLONG iCurrentTime = GetTickCount64();
	if ((iCurrentTime - iTime) < 0) {
		return (0xffffffff - iTime) + iCurrentTime;
	}
	return(iCurrentTime - iTime);
}

bool FileExist(CString strFileName)
{
	CFileStatus fs;
	if (CFile::GetStatus(strFileName, fs))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool checkFileOpen(CString strFilename)
{
	ifstream ifs;
	bool bOpen = false;
	ifs.open(strFilename);
	if(ifs.is_open() == false)
	{
		ThreadSleep(TIME10MS);
		if (ifs.is_open() == false)
		{
			ThreadSleep(TIME10MS);
			if (ifs.is_open() == false)
			{
				ThreadSleep(TIME10MS);
				if (ifs.is_open() == false)
				{
					ThreadSleep(TIME10MS);
					if (ifs.is_open() == false)
					{
						TRACE(_T("\n-> [PWR] <File Management> File: <%s> Open Failed"), strFilename);
						return bOpen;
					}
				}
			}
		}
	}
	bOpen = true;
	return bOpen;
}

void getOnlyModelName(char* modelName)
{
	int i;
	for (i = 0; i < 13; i++)
	{
		if (gPowerConfig.cModelName[i] != ' ')
		{
			modelName[i] = gPowerConfig.cModelName[i];
		}
		else
		{
			modelName[i] = '\0';
		}
	}
}

//int gSkipLineFromFile(int fd, char* strname, int strsize)
//{
//	int err = 0;
//	char buf[1000];
//	while (err != 1)
//	{
//		//err = fioRdString(fd, buf, 500);
//		if (err != 1)
//		{
//			if (strncmp(buf, strname, strsize) == 0) break;
//		}
//		else
//			return ERROR;
//	}
//	if (err == -1)
//	{
//		printf("\n-> [FILE] File reading(%s) ERROR !", strname);
//		return ERROR;
//	}
//	else
//		return 0;
//}

////////////////////////////////////////////////////////////////////////
void ThreadSleep(int millisecond)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(millisecond));
}
long gDegreeToIndex(double degree)
{
	long remain = 0;
	long deg = 0;
	long thou = 1000;
	long index = 0;

	degree = degree * (double)thou;
	deg = (long)degree;

	while (deg < 0)
	{
		deg += (360 * thou);
	}

	remain = deg % (360 * thou);

	if (45 * thou < remain && remain <= (45 + 90) * thou)
	{
		index = 1;
	}
	else if ((45 + 90) * thou < remain && remain <= (45 + 180) * thou)
	{
		index = 2;
	}
	else if ((45 + 180) * thou < remain && remain <= (45 + 270) * thou)
	{
		index = 3;
	}
	else
	{
		index = 0;
	}

	return index;
}
////////////////////////////////////////////////////////////////////////
double gCalculate3SD(double data[], long MaxCount)
{
	double stdev3sigma = 0.0;
	if (gcPowerReliability)
	{
		stdev3sigma = gcPowerReliability->CalculateSD(data, MaxCount) * CAL_3SIGMA;
	}
	return stdev3sigma;
}

////////////////////////////////////////////////////////////////////////
void ReliabilityInit()
{
	if (gcPowerReliability)
	{
		gcPowerReliability->InitRaw();
	}
}

void ReliabilityRaw1(long index, double raw1)
{
	if (gcPowerReliability)
	{
		gcPowerReliability->SetRaw(index, raw1);
	}	
}
void ReliabilityRaw2(long index, double raw1, double raw2)
{
	if (gcPowerReliability)
	{
		gcPowerReliability->SetRaw(index, raw1, raw2);
	}
}
void ReliabilityRaw3(long index, double raw1, double raw2, double raw3)
{
	if (gcPowerReliability)
	{
		gcPowerReliability->SetRaw(index, raw1, raw2, raw3);
	}
}
void ReliabilityRaw4(long index, double raw1, double raw2, double raw3, double raw4)
{
	if (gcPowerReliability)
	{
		gcPowerReliability->SetRaw(index, raw1, raw2, raw3, raw4);
	}
}

void ReliabilityMakeStdev(CString strFileName, long MaxCount)
{
	if (gcPowerReliability)
	{
		gcPowerReliability->MakeStdev(strFileName, MaxCount);
	}
}

void RunStepLock()
{
	SEM_LOCK(g_RunStepLock, INFINITE);
}

void RunStepUnLock()
{
	SEM_UNLOCK(g_RunStepLock);
}

bool CycleStopLock(long Gantry)
{
	ULONGLONG ElapsedTime, GetTime;
	long Try = 0;
	GetTime = _time_get();
	while (IsUnlockCycleStop(Gantry, TIME1MS) == false)
	{
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TIME1000MS) // 보통 1초 이내에 반응한다~~~ 그렇지 않은 경우에는 확인하여 출력한다~~~
		{
			GetTime = _time_get();
			TRACE(_T("[PWR] Gantry(%d) CycleStopUnlock Wait Unlock Try(%d)\n"), Gantry, Try++);
		}
		ThreadSleep(TIME1MS);
	}
	return true;
}

void CycleStopUnlock(long Gantry)
{
	long Result = ReleaseSemaphore(g_CycleStopLock[Gantry], 1, NULL);

	CString strErr;

	strErr.Format(_T("[PWR] Gantry(%d) CycleStopUnlock Result(%d)"), Gantry, Result);
	TRACE(strErr);
}

void CycleStopFlush(long Gantry)
{
	//SEM_FLUSH(g_CycleStopLock[Gantry]);
}

bool IsUnlockCycleStop(long Gantry, unsigned loopTime)
{
	bool bRet = false;
	DWORD ret;
	ret = WaitForSingleObject(g_CycleStopLock[Gantry], loopTime);
	if (ret == WAIT_FAILED)
	{
		return bRet;
	}
	else if (ret == WAIT_TIMEOUT)
	{
		return bRet;
	}
	else
	{
		return true;
	}
}

void SetMoveCheckANCDown(long Gantry, bool bUse)
{
	if (Gantry == FRONT_GANTRY)
	{
		if (g_bMoveCheckANCDownFront != bUse)
		{
			TRACE(_T("[PWR] SetMoveCheckANCDown Front:%d\n"), bUse);
		}
		g_bMoveCheckANCDownFront = bUse;

	}
	else
	{
		if (g_bMoveCheckANCDownRear != bUse)
		{
			TRACE(_T("[PWR] SetMoveCheckANCDown Rear:%d\n"), bUse);
		}
		g_bMoveCheckANCDownRear = bUse;
	}
}

bool GetMoveCheckANCDown(long Gantry)
{
	if (Gantry == FRONT_GANTRY)
	{
		return g_bMoveCheckANCDownFront;
	}
	else
	{
		return g_bMoveCheckANCDownRear;
	}
}

bool IsAccTest()
{
	if (GetUse2StepZMotion() == 1)
	{

		return true;
	}
	else
	{
		return false;
	}
}

void SetInsertTorqueAlarm(bool Rcv, long Action)
{
	if (g_InsertTorquteAlarm.Rcv != Rcv || g_InsertTorquteAlarm.Action != Action)
	{
		TRACE(_T("[PWR] SetInsertTorqueAlarm Rcv,Action %d,%d->%d,%d\n"), g_InsertTorquteAlarm.Rcv, g_InsertTorquteAlarm.Action, Rcv, Action);
	}

	g_InsertTorquteAlarm.Action = Action;
	g_InsertTorquteAlarm.Rcv = Rcv;
}

INSERT_ALARM_ACTION GetInsertTorqueAlarm()
{
	return g_InsertTorquteAlarm;
}

void SetY2ShiftAutoInit(Y2SHIFT_AUTOINIT set)
{
	g_Y2ShiftAutoInit = set;
	TRACE(_T("[PWR] SetY2ShiftAutoInit Use:%d Torque:%.1f\n"), g_Y2ShiftAutoInit.Use, g_Y2ShiftAutoInit.TorqueLimit);
}

Y2SHIFT_AUTOINIT GetY2ShiftAutoInit()
{
	return g_Y2ShiftAutoInit;
}

void SetHeadSkip(long Gantry, long HeadNo, bool set)
{
	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			g_HeadSkipFront[HeadNo - 1] = set;
			TRACE(_T("[PWR] SetHeadSkip Front Head%d %d\n"), HeadNo, set);

		}
		else
		{
			g_HeadSkipRear[HeadNo - 1] = set;
			TRACE(_T("[PWR] SetHeadSkip Rear Head%d %d\n"), HeadNo, set);
		}
	}

}

bool GetHeadSkip(long Gantry, long HeadNo)
{
	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			return g_HeadSkipFront[HeadNo - 1];

		}
		else
		{
			return g_HeadSkipRear[HeadNo - 1];
		}
	}

	return false;
}

long GetHeadUseCount(long Gantry)
{
	long useCnt = 0;

	if (Gantry == FRONT_GANTRY)
	{
		for (long head = 0; head < MAXUSEDHEADNO; head++)
		{
			if (g_HeadSkipFront[head] == false)
			{
				useCnt++;
			}
		}
	}
	else
	{
		for (long head = 0; head < MAXUSEDHEADNO; head++)
		{
			if (g_HeadSkipRear[head] == false)
			{
				useCnt++;
			}
		}
	}

	return useCnt;
}

void SetProductionQuantity(long quatinty)
{
	TRACE(_T("[PWR] SetProductionQuantity:%d\n"), quatinty);

	g_ProdQuantity = quatinty;
}

long GetProductionQuantity()
{
	return g_ProdQuantity;
}

bool GetBarcodeSimulation()
{
	return g_bBarcodeSimulation;
}

void SetBarcodeSimulation(bool bSimul)
{
	g_bBarcodeSimulation = bSimul;
	TRACE(_T("[PWR] SetBarcodeSimulation:%d\n"), bSimul);
}

void SetHeadBarcodeCognexUse(bool bUse)
{
	g_bUseHeadBarcodeCognex = bUse;
	TRACE(_T("[PWR] SetHeadBarcodeCognexUse,%d\n"), g_bUseHeadBarcodeCognex);
}

bool GetHeadBarcodeCognexUse()
{
	return g_bUseHeadBarcodeCognex;
}

bool GetBarcodeTypeCognexRead(long Type)
{
	//return true;
	if (Type == 0) return true;
	else return false;
}

void SetFormingDRBCoilUse(bool use)
{
	g_FormingDRBCoilUse = use;
	TRACE(_T("[PWR] SetFormingDRBCoilUse,%d\n"), g_FormingDRBCoilUse);
}

bool GetFormingDRBCoilUse()
{
	return g_FormingDRBCoilUse;
}

atomic<bool> machineManualActionRunningSign = false;
void SetMachineManualActionRunninSign(bool isRun)
{
	machineManualActionRunningSign.store(isRun);
	return;
}

bool GetMachineManualActionRunninSign()
{
	bool ret = false;
	ret = machineManualActionRunningSign.load();
	return ret;
}
