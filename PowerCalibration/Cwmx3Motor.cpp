#include "pch.h"
#include "Cwmx3Motor.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <ecApi.h>

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace wmx3Api;
using namespace std;

Cwmx3Motor* gcWmx3Motor;
Cwmx3Motor::Cwmx3Motor(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    m_ShowID = 0;
    InitThreadData(h_Terminate, h_HostMutex, h_RunStartSync);
}

Cwmx3Motor::~Cwmx3Motor()
{
}

void Cwmx3Motor::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);

    SetThreadName(_T("thWmx3MotorStatus"));
    lpStartAddress = (_beginthreadex_proc_type)StartWmx3MotorStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    m_ShowID = nID;
    TRACE(_T("[PWR] Wmx3 Motor Status Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] Wmx3 Motor Motion Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

int Cwmx3Motor::EStop()
{
    int err = ErrorCode::None;
    Lock();
    err = GetCoreMotion()->ExecEStop(EStopLevel::Final);
    Unlock();
    return err;
}

int Cwmx3Motor::EStopHigher()
{
    int err = ErrorCode::None;
    Lock();
    err = GetCoreMotion()->ExecEStop(EStopLevel::Level1);
    Unlock();
    return err;
}

int Cwmx3Motor::ReleaseEStop()
{
    int err = ErrorCode::None;
    Lock();
    err = GetCoreMotion()->ReleaseEStop();
    Unlock();
    return err;
}

bool Cwmx3Motor::WaitEStopStatus(bool bStatus)
{
    while (true)
    {
        if (GetEmergencyStop() == bStatus)
        {
            break;
        }
        ThreadSleep(TIME10MS);
    }
    return true;
}

bool Cwmx3Motor::GetEmergencyStop()
{
    return gMasterInfo.emergencyStop;
}

WMX3EmgStopLevel Cwmx3Motor::GetEmergencyStopLevel()
{
    return gMasterInfo.emergencyStopLevel;
}

int Cwmx3Motor::GetWmx3MotorStatus()
{
    int err = ErrorCode::None;
    //if (GetGlobalSimulationMode() == true) return err;
    err = GetCoreMotion()->GetStatus(g_CoreMotionStatus);
    if (err != ErrorCode::None)
    {
        CoreMotionErrorToString(err, _T("GetStatus"));
    }
    return err;
}

bool Cwmx3Motor::CopyWmx3MasterStatus(LPVOID wParam)
{
    //if (GetGlobalSimulationMode() == true) return true;
    gMasterInfo.invalidLicenseError = g_CoreMotionStatus->invalidLicenseError;
    gMasterInfo.engineState = (WMX3EngineState)g_CoreMotionStatus->engineState;
    gMasterInfo.cycleTimeMilliseconds[0] = g_CoreMotionStatus->cycleTimeMilliseconds[0];
    gMasterInfo.cycleTimeMilliseconds[1] = g_CoreMotionStatus->cycleTimeMilliseconds[1];
    gMasterInfo.cycleCounter[0] = g_CoreMotionStatus->cycleCounter[0];
    gMasterInfo.cycleCounter[1] = g_CoreMotionStatus->cycleCounter[1];
    gMasterInfo.emergencyStop = g_CoreMotionStatus->emergencyStop;
    gMasterInfo.emergencyStopLevel = (WMX3EmgStopLevel)g_CoreMotionStatus->emergencyStopLevel;
    return true;
}

bool Cwmx3Motor::CopyWmx3MotorStatus(LPVOID wParam)
{
    //if (GetGlobalSimulationMode() == true) return true;
    Cwmx3Motor* pThis = reinterpret_cast<Cwmx3Motor*>(wParam);
    for (int indx = 0; indx < MAXAXISNO; indx++)
    {
        gMotorStatus[indx].servoOn = g_CoreMotionStatus->axesStatus[indx].servoOn;
        gMotorStatus[indx].servoOffline = g_CoreMotionStatus->axesStatus[indx].servoOffline;
        gMotorStatus[indx].ampAlarm = g_CoreMotionStatus->axesStatus[indx].ampAlarm;
        gMotorStatus[indx].ampAlarmCode = g_CoreMotionStatus->axesStatus[indx].ampAlarmCode;
        gMotorStatus[indx].masterAxis = g_CoreMotionStatus->axesStatus[indx].masterAxis;
        gMotorStatus[indx].secondMasterAxis = g_CoreMotionStatus->axesStatus[indx].secondMasterAxis;
        gMotorStatus[indx].posCmd = g_CoreMotionStatus->axesStatus[indx].posCmd;
        gMotorStatus[indx].actualPos = g_CoreMotionStatus->axesStatus[indx].actualPos;
        gMotorStatus[indx].compPosCmd = g_CoreMotionStatus->axesStatus[indx].compPosCmd;
        gMotorStatus[indx].compActualPos = g_CoreMotionStatus->axesStatus[indx].compActualPos;
        gMotorStatus[indx].syncActualPos = g_CoreMotionStatus->axesStatus[indx].syncActualPos;
        gMotorStatus[indx].encoderCommand = g_CoreMotionStatus->axesStatus[indx].encoderCommand;
        gMotorStatus[indx].encoderFeedback = g_CoreMotionStatus->axesStatus[indx].encoderFeedback;
        gMotorStatus[indx].velocityCmd = g_CoreMotionStatus->axesStatus[indx].velocityCmd;
        gMotorStatus[indx].actualVelocity = g_CoreMotionStatus->axesStatus[indx].actualVelocity;
        gMotorStatus[indx].velocityLag = g_CoreMotionStatus->axesStatus[indx].velocityLag;
        gMotorStatus[indx].torqueCmd = g_CoreMotionStatus->axesStatus[indx].torqueCmd;
        gMotorStatus[indx].actualTorque = g_CoreMotionStatus->axesStatus[indx].actualTorque;
        gMotorStatus[indx].actualFollowingError = g_CoreMotionStatus->axesStatus[indx].actualFollowingError;
        gMotorStatus[indx].compensation.backlashCompensation = g_CoreMotionStatus->axesStatus[indx].compensation.backlashCompensation;
        gMotorStatus[indx].compensation.pitchErrorCompensation = g_CoreMotionStatus->axesStatus[indx].compensation.pitchErrorCompensation;
        gMotorStatus[indx].compensation.pitchErrorCompensation2D = g_CoreMotionStatus->axesStatus[indx].compensation.pitchErrorCompensation2D;
        gMotorStatus[indx].compensation.totalPosCompensation = g_CoreMotionStatus->axesStatus[indx].compensation.totalPosCompensation;
        gMotorStatus[indx].opState = (WMX3OperationState)g_CoreMotionStatus->axesStatus[indx].opState;
        gMotorStatus[indx].detailOpState = (WMX3DetailOperationState)g_CoreMotionStatus->axesStatus[indx].detailOpState;
        gMotorStatus[indx].axisCommandMode = (WMX3AxisCommandMode)g_CoreMotionStatus->axesStatus[indx].axisCommandMode;
        gMotorStatus[indx].axisSyncMode = (WMX3AxisSyncMode)g_CoreMotionStatus->axesStatus[indx].axisSyncMode;
        gMotorStatus[indx].followingErrorAlarm = g_CoreMotionStatus->axesStatus[indx].followingErrorAlarm;
        gMotorStatus[indx].commandReady = g_CoreMotionStatus->axesStatus[indx].commandReady;
        gMotorStatus[indx].motionPaused = g_CoreMotionStatus->axesStatus[indx].motionPaused;
        gMotorStatus[indx].profileTotalMilliseconds = g_CoreMotionStatus->axesStatus[indx].profileTotalMilliseconds;
        gMotorStatus[indx].profileAccMilliseconds = g_CoreMotionStatus->axesStatus[indx].profileAccMilliseconds;
        gMotorStatus[indx].profileCruiseMilliseconds = g_CoreMotionStatus->axesStatus[indx].profileCruiseMilliseconds;
        gMotorStatus[indx].profileDecMilliseconds = g_CoreMotionStatus->axesStatus[indx].profileDecMilliseconds;
        gMotorStatus[indx].profileRemainingMilliseconds = g_CoreMotionStatus->axesStatus[indx].profileRemainingMilliseconds;
        gMotorStatus[indx].profileTargetPos = g_CoreMotionStatus->axesStatus[indx].profileTargetPos;
        gMotorStatus[indx].profileTotalDistance = g_CoreMotionStatus->axesStatus[indx].profileTotalDistance;
        gMotorStatus[indx].profileRemainingDistance = g_CoreMotionStatus->axesStatus[indx].profileRemainingDistance;
        gMotorStatus[indx].intplVelocity = g_CoreMotionStatus->axesStatus[indx].intplVelocity;
        gMotorStatus[indx].intplSegment = g_CoreMotionStatus->axesStatus[indx].intplSegment;
        gMotorStatus[indx].cmdAcc = g_CoreMotionStatus->axesStatus[indx].cmdAcc;
        gMotorStatus[indx].accFlag = g_CoreMotionStatus->axesStatus[indx].accFlag;
        gMotorStatus[indx].decFlag = g_CoreMotionStatus->axesStatus[indx].decFlag;
        gMotorStatus[indx].inPos = g_CoreMotionStatus->axesStatus[indx].inPos;
        gMotorStatus[indx].inPos2 = g_CoreMotionStatus->axesStatus[indx].inPos2;
        gMotorStatus[indx].inPos3 = g_CoreMotionStatus->axesStatus[indx].inPos3;
        gMotorStatus[indx].inPos4 = g_CoreMotionStatus->axesStatus[indx].inPos4;
        gMotorStatus[indx].inPos5 = g_CoreMotionStatus->axesStatus[indx].inPos5;
        gMotorStatus[indx].cmdDistributionEnd = g_CoreMotionStatus->axesStatus[indx].cmdDistributionEnd;
        gMotorStatus[indx].posSet = g_CoreMotionStatus->axesStatus[indx].posSet;
        gMotorStatus[indx].delayedPosSet = g_CoreMotionStatus->axesStatus[indx].delayedPosSet;
        gMotorStatus[indx].cmdDistributionEndDelayedPosSetDiff = g_CoreMotionStatus->axesStatus[indx].cmdDistributionEndDelayedPosSetDiff;
        gMotorStatus[indx].positiveLS = g_CoreMotionStatus->axesStatus[indx].positiveLS;
        gMotorStatus[indx].negativeLS = g_CoreMotionStatus->axesStatus[indx].negativeLS;
        gMotorStatus[indx].nearPositiveLS = g_CoreMotionStatus->axesStatus[indx].nearPositiveLS;
        gMotorStatus[indx].nearNegativeLS = g_CoreMotionStatus->axesStatus[indx].nearNegativeLS;
        gMotorStatus[indx].externalPositiveLS = g_CoreMotionStatus->axesStatus[indx].externalPositiveLS;
        gMotorStatus[indx].externalNegativeLS = g_CoreMotionStatus->axesStatus[indx].externalNegativeLS;
        gMotorStatus[indx].positiveSoftLimit = g_CoreMotionStatus->axesStatus[indx].positiveSoftLimit;
        gMotorStatus[indx].negativeSoftLimit = g_CoreMotionStatus->axesStatus[indx].negativeSoftLimit;
        gMotorStatus[indx].homeState = (WMX3HomeState)g_CoreMotionStatus->axesStatus[indx].homeState;
        gMotorStatus[indx].homeError = (WMX3HomeError)g_CoreMotionStatus->axesStatus[indx].homeError;
        gMotorStatus[indx].homeSwitch = g_CoreMotionStatus->axesStatus[indx].homeSwitch;
        gMotorStatus[indx].homeDone = g_CoreMotionStatus->axesStatus[indx].homeDone;
        gMotorStatus[indx].homePaused = g_CoreMotionStatus->axesStatus[indx].homePaused;
        gMotorStatus[indx].homeOffset = g_CoreMotionStatus->axesStatus[indx].homeOffset;
        gMotorStatus[indx].cmdPosToFbPosFlag = g_CoreMotionStatus->axesStatus[indx].cmdPosToFbPosFlag;
        gMotorStatus[indx].execSuperimposedMotion = g_CoreMotionStatus->axesStatus[indx].execSuperimposedMotion;
        gMotorStatus[indx].singleTurnCounter = g_CoreMotionStatus->axesStatus[indx].singleTurnCounter;
        gMotorStatus[indx].motionComplete = g_CoreMotionStatus->axesStatus[indx].motionComplete;
    }
    return true;
}

UINT Cwmx3Motor::StartWmx3MotorStatus(LPVOID wParam)
{
    bool bLoop = true, bFirst = true;
    int err = ErrorCode::None;
    Cwmx3Motor* pThis = reinterpret_cast<Cwmx3Motor*>(wParam);
    AddThread(pThis);
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_WMX3_MOTOR_READTIME) == true)
        {
            TRACE(_T("[PWR] Cwmx3Motor(0x%X) Terminated\n"), pThis->GetThreadID());
            break;
        }
        err = pThis->GetWmx3MotorStatus();
        if (err != ErrorCode::None)
        {
            TRACE(_T("[PWR] failed to get status StartWmx3MotorStatus\n"));
            break;
        }
        pThis->CopyWmx3MasterStatus(wParam);
        pThis->CopyWmx3MotorStatus(wParam);
        //ThreadSleep(THREAD_WMX3_MOTOR_READTIME);
        if (bFirst == true)
        {
            TRACE(_T("[PWR] Motor Task Synchronize Set Event\n"));
            SetEvent(pThis->GetThreadStartSync());
            bFirst = false;
        }
    };
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    TRACE(_T("[PWR] Cwmx3Motor(0x%X) Quit\n"), pThis->GetThreadID());
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}
