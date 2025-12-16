#pragma once
/*
//	GlobalData.h
//
//	(C) Power. 2020 ~
*/
#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <CompensationApi.h>
#include <LogApi.h>
#include <ecApi.h>
#include "GlobalDefine.h"
#include "Cwmx3Axis.h"
#include "CPowerThread.h"
#include "CMessageQueue.h"
#include "CLogFileSystem.h"
#include "CLastPick.h"
#include <atomic>

using namespace wmx3Api;
using namespace ecApi;
using namespace std;

typedef int (*PfuncType)(void* funcPrm);

#define ROOT_PATH			"C:\\Power\\i6.0\\"
#define MCS_PATH			"\\MCS\\"
#define WMX3_PATH			"\\WMX3\\"
#define LOG_PATH			"\\HISTORY\\"
#define CONFIG_PATH			"\\Config\\"

#define EPSILON				1e-6f

#define	CHANNEL_1			0
#define	CHANNEL_2			1
#define	CHANNEL_3			2

#define SWAP(x,y,z) ((z)=(x), (x)=(y), (y)=(z))

extern bool g_bSimulationMode;
extern bool g_bMachineInitialized;
extern bool g_bRecvInitializeMachine;
extern bool g_bMachineStatusError;
extern long g_bMachineAlarmCode;
extern bool g_bAutoConnectUse;
extern bool g_bAutoHomingUse;
extern bool g_b1DCompensationUse;
extern bool g_b2DCompensationUse;
extern long g_2DCompensationMethod; 
extern long g_GantryCalibrationMethod;
extern long g_Gantry2DMethod;
extern bool g_bZCompensationUse;
extern bool g_bSkipMotorPower;
extern bool g_MotorOffDoorOpen;
extern bool g_bInitY2Shift;
extern long g_MachineState;
extern long g_InfiniteDryRun;
extern long g_SettlingDelay;
extern bool g_bUserDoorPush;
extern bool g_bUserDoorClose;
extern long g_FirstPickup;
extern long g_SkipVision;
extern long g_SimulLoading;
extern long g_UsePathLinearIntpl;
extern long g_Use2StepZMotion;
extern long g_UseLineOfBalance;
extern long g_ReadyToRun;
extern long g_UseAreaSensor;
extern long g_UseAreaSensor2nd;
extern long g_UseRearFeeder;
extern bool g_UseTTF;
extern bool g_ManualConveyor;
extern long g_WorkExistSkip;
extern long g_UseRTDSensorFX;
extern long g_UseRTDSensorFY1;
extern long g_UseRTDSensorFY2;
extern long g_UseANC[MAX_ANC_BASE];
extern bool g_bY2CompensationUse;
extern long g_FrontCamCount;
extern long g_RearCamCount;
extern long g_FrontCamType;
extern long g_RearCamType;
extern long g_FrontCamLedChannel;
extern long g_RearCamLedChannel;
extern long g_FrontCamLaser;
extern long g_RearCamLaser;
extern bool g_bUseExitConvStopper;
extern ULONGLONG g_UserOpenDoorGetTime, g_UserOpenDoorElapsed;
extern bool m_MoveOnceZREnable;
extern bool g_bMoveCheckANCDownFront;
extern bool g_bMoveCheckANCDownRear;
extern bool g_bOnlyConveyorMode;
extern long g_CameraVersion;
extern bool g_bBarcodeSimulation;
extern bool g_bUseHeadBarcodeCognex;
extern INSERT_ALARM_ACTION g_InsertTorquteAlarm;
extern Y2SHIFT_AUTOINIT g_Y2ShiftAutoInit;

extern bool g_HeadSkipFront[MAXUSEDHEADNO];
extern bool g_HeadSkipRear[MAXUSEDHEADNO];
extern bool g_FormingDRBCoilUse;

extern TwoDPitchErrorCompensationData g_2DCompDataX;
extern TwoDPitchErrorCompensationData g_2DCompDataY1;
extern TwoDPitchErrorCompensationData g_2DCompDataY2;
extern MachineStruct gPowerConfig;
extern double gdblCompen[MAXTABLENO][BUFSIZE];
extern CameraData Camera[MAXCAMNO];
extern VisualAlignInfo	VA[MAXCOMPONENTNO];
extern MarkInfo Mark[MAXVISIONFILENO];
extern VisualOddAlignInfo 	OddVA[MAXODDVADBNO + 1];
extern BGABallArrayInfo 	BGABall[MAXODDVADBNO + 1];
extern VisualOddAlignInfo 	OddInsertPartVA[MAXINSERTPARTVADBNO + 1];
extern InsertDBVisionInfo  InsertPartVA[MAXINSERTPARTVADBNO + 1];
extern UWORD OptionSet[MAXOPTIONNO];
extern float MachineFSet[MAXMACHINENO];
extern UWORD MachineISet[MAXMACHINENO];
extern float MachineFSet2[MAXMACHINENO];
extern UWORD MachineISet2[MAXMACHINENO];
extern UWORD ExtOptionSet[MAXOPTIONNO_EXT];
extern UWORD OptionSet2nd[MAXOPTIONNO];
extern float MachineFSet3rd[MAXOPTIONNO];
extern UWORD MachineISet3rd[MAXOPTIONNO];

extern HANDLE g_Wmx3AxisLock;
extern CArray<Cwmx3Axis*, Cwmx3Axis*> g_pWmx3AxisArray;
extern HANDLE g_Wmx3LinearIntplLock;
extern HANDLE g_Wmx3LinearIntplLockR;
extern HANDLE g_Wmx3LinearIntplLockZ;
extern CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArray;
extern CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArrayR;
extern CArray<StartPosStruct*, StartPosStruct*> g_pWmx3LinearIntplAxisArrayZ;
extern HANDLE g_ThreadArrayLock;
extern CArray<CPowerThread*, CPowerThread*> g_pThreadArray;
extern HANDLE g_ThreadCleanArrayLock;
extern CArray<CPowerThread*, CPowerThread*> g_ThreadCleanArray;
extern HANDLE g_ThreadRunLock;
extern CArray<CPowerThread*, CPowerThread*> g_pThreadRunArray;
extern IOMAPSTRUCT gioMapStruct;
extern WMX3MASTER_INFO gMasterInfo;
extern WMX3_AXIS_STATUS gMotorStatus[MAXAXISNO];
//extern HANDLE gMOTION_LOCK[MAXAXISNO];
extern HANDLE gMACHINE_CONTROL;
extern HANDLE gRUN_MODE;;
extern HANDLE gRUN_MOVEONETIME;
extern bool g_PauseMoveOneTime;
extern bool g_bSimulationForming;

extern WMX3Api* g_Device;
extern Io* g_IO;
extern CoreMotion* g_CoreMotion;
extern AdvancedMotion* g_AdvancedMotion;
extern EventControl* g_EventControl;
extern Log* g_Wmx3Log;
extern Compensation* g_Compensation;
extern CoreMotionStatus* g_CoreMotionStatus;
extern Ecat* g_Ecat;
extern DevicesInfoA* g_DevInfo;
extern HANDLE g_CycleStopLock[2];

extern CString g_strRoot;
extern CLogFileSystem* g_LogWriter;
extern UserMemory* g_UserMemory;
extern HANDLE g_RunStepLock;

extern long VARIOPORT[MAXIODEFINE];
extern unsigned char* DISK_RUN_ADDR;
extern unsigned int DISK_ADDR_OFFSET;
extern unsigned int DISK_OFFSET;
extern unsigned int DISK_SADDR;
extern unsigned int DISK_BYTE;
extern unsigned int DISK_BLOCK;
extern unsigned char MACHINE_BLOCKDATA[MAX_MACHINEFILE_SIZE];
extern unsigned char RUN_BLOCKDATA[MAX_RUNFILE_SIZE];
extern unsigned char INSERTEND_BLOCKDATA[MAX_INSERTENDFILE_SIZE];
//extern CString gLastPickPcbName;
//extern INSERT gLastPickDataFront[MAXUSEDHEADNO];
//extern INSERT gLastPickDataRear[MAXUSEDHEADNO];
extern long g_ProdQuantity;

////////////////////////////////////////////////////////////////////////
extern void SetGlobalSimulationMode(bool bSimul);
extern bool GetGlobalSimulationMode();
extern void SetInitializedMachine(bool bInit);
extern bool GetInitializedMachine();
extern void SetRecvInitializeMachine(bool bRecv);
extern bool GetRecvInitializeMachine();
extern void SetAutoHomingUse(bool bUse);
extern bool GetAutoHomingUse();
extern void Set1DCompensationUse(bool bUse);
extern bool Get1DCompensationUse();
extern void Set2DCompensationMethod(long Method);
extern long Get2DCompensationMethod();
extern void Set2DCompensationUse(bool bUse);
extern bool Get2DCompensationUse();
extern void SetGantryCalibrationMethod(long Method);
extern long GetGantryCalibrationMethod();
extern void SetGantry2DMethod(long Method);
extern long GetGantry2DMethod();
extern void SetZCompensationUse(bool bUse);
extern bool GetZCompensationUse();
extern void SetSkipMotorPower(bool bSkip);
extern bool GetSkipMotorPower();
extern void SetMotorOffDoorOpen(bool set);
extern bool GetMotorOffDoorOpen();
extern void SetInitY2Shift(bool bInit);
extern bool GetInitY2Shift();
extern void SetGlobalStatusError(bool bStatus);
extern bool GetGlobalStatusError();
extern void SetMachineAlarmCode(long AlarmCode);
extern long GetMachineAlarmCode();
extern void SetMachineState(long State);
extern long GetMachineState();
extern void SetInfiniteDryRun(long Infinite);
extern long GetInfiniteDryRun();
extern void SetGlobalSettlingDelay(long SettlingDelay);
extern long GetGlobalSettlingDelay();
extern void SetFirstPickup(long FirstPickup);
extern long GetFirstPickup();
extern void SetSkipVision(long SkipVision);
extern long GetSkipVision();
extern void SetSimulLoading(long SimulLoading);
extern long GetSimulLoading();
extern void SetUsePathLinearIntpl(long PathLinear);
extern long GetUsePathLinearIntpl();
extern void SetUse2StepZMotion(long TwoStepZMotion);
extern long GetUse2StepZMotion();
extern void SetUseLineOfBalance(long LineOfBalance);
extern long GetUseLineOfBalance();
extern void SetHMIReadyToRun(long ReadyToRun);
extern long GetHMIReadyToRun();
extern void SetUseAreaSensor(long AreaSensor);
extern long GetUseAreaSensor();
extern void SetUseAreaSensor2nd(long AreaSensor);
extern long GetUseAreaSensor2nd();
extern void SetRearFeederUse(long FeederUse);
extern long GetRearFeederUse();
extern void SetTTFUse(bool Use);
extern bool GetTTFUse();
extern void SetUseTTFAxis();
extern void SetManualConvUse(bool Use);
extern bool GetManualConvUse();
extern void SetWorkExistSkip(long skip);
extern long GetWorkExistSkip();
extern void SetUseRTDSensorFX(long UseRTDSensorX);
extern long GetUseRTDSensorFX();
extern void SetUseRTDSensorFY1(long UseRTDSensorY1);
extern long GetUseRTDSensorFY1();
extern void SetUseRTDSensorFY2(long UseRTDSensorY2);
extern long GetUseRTDSensorFY2();
extern void SetUseANC(long Base, long Use);
extern long GetUseANC(long Base);
extern void SetUseExitConvStopper(bool bUse);
extern bool GetUseExitConvStopper();
extern void SetClearConveyorAxis();
extern void SetAxisMapByOption();
extern void SetY2CompensationUse(bool bUse);
extern bool GetY2CompensationUse();
extern void SetCameraCount(long Stage, long CamCount);
extern long GetCameraCount(long Stage);
extern void SetModuleCamType(long Gantry, long Type);
extern long GetModuleCamType(long Gantry);
extern void SetModuleCamLedChannel(long Gantry, long Type);
extern long GetModuleCamLedChannel(long Gantry);
extern void SetModuleCamLaserUse(long Gantry, long Use);
extern long GetModuleCamLaserUse(long Gantry);
extern void SetOnlyConveyorMode(bool set);
extern bool GetOnlyConveyorMode();
extern void TraceStack(void);
extern char* ConvertWchartoChar(wchar_t* str);
extern bool IsNumber(CString strNum);
extern int ConvertCStringToInt(CString strNum);
extern double ConvertCStringToDouble(CString strNum);
extern float ConvertCStringToFloat(CString strNum);
extern ULONGLONG _time_get();
extern ULONGLONG _time_elapsed(ULONGLONG iTime);
extern CString GetCurrentDateTime();
extern void SetSimulationForming(bool bUse);
extern bool GetSimulationForming();
//extern void InitializeLog();
//extern void StartLogging();
//extern CString StartupPath();
//extern void PLog(CString strLog);
extern bool FileExist(CString strFileName);
extern bool checkFileOpen(CString strFilename);
extern void getOnlyModelName(char* modelName);
//extern int gSkipLineFromFile(int fd, char* strname, int strsize);
////////////////////////////////////////////////////////////////////////
extern void ThreadArrayLock();
extern void ThreadArrayUnlock();
extern bool IsDoubleThread(CPowerThread* pThreadInfo);
extern void AddThread(CPowerThread* pThreadInfo);
extern void DeleteThread(int thID);
extern INT_PTR AllThreadCount();
extern CString GetThreadNameByID(int nID);
extern CPowerThread* GetThreadByIndex(INT_PTR indx);
extern bool IsThreadByName(CString ThreadName);
extern CPowerThread* GetThreadByName(CString ThreadName);
extern INT_PTR GetIndexByThreadID(int thID);
extern void DeleteThreadByIndex(INT_PTR indx);
extern bool gIsAliveThread(HANDLE mThread);
extern void ThreadCleanArrayLock();
extern void ThreadCleanArrayUnlock();
extern void AddCleanThread(CPowerThread* pThread);
extern INT_PTR GetCleanThreadCount();
extern CPowerThread* GetCleanThread(INT_PTR index);
extern void DestroyCleanThread(INT_PTR index);
extern void RunThreadLock();
extern void RunThreadUnlock();
extern void PCBWaitLock();
extern void PCBWaitUnLock();
extern void PCBWaitReset();
extern void AddRunThread(CPowerThread* pThread);
extern INT_PTR GetRunThreadCount();
extern CPowerThread* GetRunThread(INT_PTR index);
extern void DestroyRunThread(INT_PTR index);
////////////////////////////////////////////////////////////////////////
extern void ThreadSleep(int millisecond);
extern long gDegreeToIndex(double degree);
////////////////////////////////////////////////////////////////////////
extern void Wmx3Lock();
extern void Wmx3Unlock();
extern bool AddWmx3Axis(Cwmx3Axis* pAxis);
bool RemoveWmx3Axis(INT_PTR indx);
extern INT_PTR GetWmx3AxisCount();
extern Cwmx3Axis* GetWmx3AxisByIndex(INT_PTR indx);
extern Cwmx3Axis* GetWmx3AxisByName(CString AxisName);
extern Cwmx3Axis* GetWmx3AxisByAxisNo(long AxisNo);
////////////////////////////////////////////////////////////////////////
extern void Wmx3LinearIntpLock();
extern void Wmx3LinearIntpUnlock();
extern bool AddWmx3LinearIntpAxis(StartPosStruct* pAxis);
bool RemoveWmx3LinearIntpAxis(INT_PTR indx);
extern INT_PTR GetWmx3LinearIntpAxisCount();
extern StartPosStruct* GetWmx3LinearIntpAxisByIndex(INT_PTR indx);
extern StartPosStruct* GetWmx3LinearIntpAxisByName(CString AxisName);
////////////////////////////////////////////////////////////////////////
extern void Wmx3LinearIntpLockR();
extern void Wmx3LinearIntpUnlockR();
extern bool AddWmx3LinearIntpAxisR(StartPosStruct* pAxis);
bool RemoveWmx3LinearIntpAxisR(INT_PTR indx);
extern INT_PTR GetWmx3LinearIntpAxisRCount();
extern StartPosStruct* GetWmx3LinearIntpAxisRByIndex(INT_PTR indx);
extern StartPosStruct* GetWmx3LinearIntpAxisRByName(CString AxisName);
////////////////////////////////////////////////////////////////////////
extern void NewDevice();
extern void SetDevice(WMX3Api* wmx3Api);
extern WMX3Api* GetDevice();
extern void DeleteDeivce();
extern void NewIO();
extern void SetIO(Io* wmx3Api);
extern Io* GetIO();
extern void DeleteIO();
extern void NewCoreMotion();
extern void SetCoreMotion(CoreMotion* coreMotion);
extern CoreMotion* GetCoreMotion();
extern void DeleteCoreMotion();

extern void NewAdvancedMotion();
extern void SetAdvancedMotion(AdvancedMotion* coreMotion);
extern AdvancedMotion* GetAdvancedMotion();
extern void DeleteAdvancedMotion();

extern void NewEventControl();
extern void SetEventControl(EventControl* eventControl);
extern EventControl* GetEventControl();
extern void DeleteEventControl();
extern long GetMaxEventControl();

extern void NewWmx3Log();
extern void SetWmx3Log(Log* Wmx3Log);
extern Log* GetWmx3Log();
extern void DeleteWmx3Log();

extern void NewCoreMotionStatus();
extern void SetCoreMotionStatus(CoreMotionStatus* coreMotionStatus);
extern CoreMotionStatus* GetCoreMotionStatus();
extern void DeleteCoreMotionStatus();
extern void NewEcat();
extern void SetEcat(Ecat* ecAPI);
extern Ecat* GetEcat();
extern void DeleteEcat();
extern void NewCompensation();
extern void SetCompensation(Compensation* pCompensation);
extern Compensation* GetCompensation();
extern void DeleteCompensation();
extern void NewUserMemory();
extern void SetUserMemory(UserMemory* pUserMemory);
extern UserMemory* GetUserMemory();
extern void DeleteUserMemory();
extern unsigned DeviceErrorToString(unsigned errCode, CString strCmd);
extern unsigned IOErrorToString(unsigned errCode, CString strCmd);
extern unsigned EcatErrorToString(unsigned errCode, CString strCmd);
extern unsigned CoreMotionErrorToString(unsigned errCode, CString strCmd);
extern unsigned CoreMotionErrorToString(int AxisMap, unsigned errCode, CString strCmd);
extern unsigned AdvancedMotionErrorToString(unsigned errCode, CString strCmd);
extern unsigned EventControlErrorToString(unsigned errCode, CString strCmd);
extern unsigned LogErrorToString(unsigned errCode, CString strCmd);
extern unsigned CompensationErrorToString(unsigned errCode, CString strCmd);
extern unsigned UserMemoryErrorToString(unsigned errCode, CString strCmd);
extern long Set2DCompensationData(long Channel, TwoDPitchErrorCompensationData* pitchErrCompData);
extern long Enable2D(long Channel);
extern long Disable2D(long Channel);
extern CString GetOperatingState(unsigned OpState);
extern CString GetAxisNameByAxisIndex(unsigned Axis);
extern void PrintMotorStatus(CString strName, CoreMotionAxisStatus cmAxis);
////////////////////////////////////////////////////////////////////////
extern double gCalculate3SD(double data[], long MaxCount);
////////////////////////////////////////////////////////////////////////
extern void ReliabilityInit();
extern void ReliabilityRaw1(long index, double raw1);
extern void ReliabilityRaw2(long index, double raw1, double raw2);
extern void ReliabilityRaw3(long index, double raw1, double raw2, double raw3);
extern void ReliabilityRaw4(long index, double raw1, double raw2, double raw3, double raw4);
extern void ReliabilityMakeStdev(CString strFileName, long MaxCount);
////////////////////////////////////////////////////////////////////////
extern void RunStepLock();
extern void RunStepUnLock();
extern bool CycleStopLock(long Gantry);
extern void CycleStopUnlock(long Gantry);
extern void CycleStopFlush(long Gantry);
extern bool IsUnlockCycleStop(long Gantry, unsigned loopTime);
extern void SetMoveCheckANCDown(long Gantry, bool bUse);
extern bool GetMoveCheckANCDown(long Gantry);
extern bool IsAccTest();
extern void SetInsertTorqueAlarm(bool Rcv, long Action);
extern INSERT_ALARM_ACTION GetInsertTorqueAlarm();
extern void SetY2ShiftAutoInit(Y2SHIFT_AUTOINIT set);
extern Y2SHIFT_AUTOINIT GetY2ShiftAutoInit();

extern void SetHeadSkip(long Gantry, long HeadNo, bool set);
extern bool GetHeadSkip(long Gantry, long HeadNo);
extern long GetHeadUseCount(long Gantry);

extern void SetProductionQuantity(long quatinty);
extern long GetProductionQuantity();

extern bool GetBarcodeSimulation();
extern void SetBarcodeSimulation(bool bSimul);

extern void SetHeadBarcodeCognexUse(bool bUse);
extern bool GetHeadBarcodeCognexUse();
extern bool GetBarcodeTypeCognexRead(long Type);
extern void SetFormingDRBCoilUse(bool use);
extern bool GetFormingDRBCoilUse();

extern atomic<bool> machineManualActionRunningSign;
extern void SetMachineManualActionRunninSign(bool isRun);
extern bool GetMachineManualActionRunninSign();