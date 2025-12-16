#include "pch.h"
#include "CSlaveMotorStatus.h"

#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CPowerStackWalker.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/
using namespace wmx3Api;
using namespace std;

CSlaveMotorStatus* gcSlaveMotorStatus;
CSlaveMotorStatus::CSlaveMotorStatus(HANDLE h_Terminate, HANDLE h_HostMutex, HANDLE h_RunStartSync)
{
    InitialMemory();
}

void CSlaveMotorStatus::InitialMemory()
{
    ZeroMemory(&m_MotionStatus, sizeof(m_MotionStatus));
    ZeroMemory(&m_MasterInfo, sizeof(m_MasterInfo));
}

CSlaveMotorStatus::~CSlaveMotorStatus()
{
}

void CSlaveMotorStatus::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);

    SetThreadName(_T("thSlaveMotorStatus"));
    lpStartAddress = (_beginthreadex_proc_type)StartSlaveMotorStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);    
    TRACE(_T("[PWR] Slave Motor Status Thread ID:0x%04X(%s)\n"), GetThreadID(), GetThreadName());
    strLog.Format(_T("[PWR] Slave Motor Motion Thread ID:0x%04X(%s)"), GetThreadID(), (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

bool CSlaveMotorStatus::IsDiffMotorStatus(int iSelAxis)
{
    return true;
}

void CSlaveMotorStatus::CopyMasterStatus()
{
    m_MasterInfo.invalidLicenseError = gMasterInfo.invalidLicenseError;
    m_MasterInfo.engineState = gMasterInfo.engineState;
    m_MasterInfo.cycleTimeMilliseconds[0] = gMasterInfo.cycleTimeMilliseconds[0];
    m_MasterInfo.cycleTimeMilliseconds[1] = gMasterInfo.cycleTimeMilliseconds[1];
    m_MasterInfo.cycleCounter[0] = gMasterInfo.cycleCounter[0];
    m_MasterInfo.cycleCounter[1] = gMasterInfo.cycleCounter[1];
    m_MasterInfo.emergencyStop = gMasterInfo.emergencyStop;
    m_MasterInfo.emergencyStopLevel = gMasterInfo.emergencyStopLevel;
}

void CSlaveMotorStatus::CopyMotorStatus()
{
    for (int indx = 0; indx < MAXAXISNO; indx++)
    {
        m_MotionStatus[indx].servoOn = gMotorStatus[indx].servoOn;
        m_MotionStatus[indx].servoOffline = gMotorStatus[indx].servoOffline;
        m_MotionStatus[indx].ampAlarm = gMotorStatus[indx].ampAlarm;
        m_MotionStatus[indx].ampAlarmCode = gMotorStatus[indx].ampAlarmCode;
        m_MotionStatus[indx].masterAxis = gMotorStatus[indx].masterAxis;
        m_MotionStatus[indx].secondMasterAxis = gMotorStatus[indx].secondMasterAxis;
        m_MotionStatus[indx].posCmd = gMotorStatus[indx].posCmd;
        m_MotionStatus[indx].actualPos = gMotorStatus[indx].actualPos;
        m_MotionStatus[indx].compPosCmd = gMotorStatus[indx].compPosCmd;
        m_MotionStatus[indx].compActualPos = gMotorStatus[indx].compActualPos;
        m_MotionStatus[indx].syncActualPos = gMotorStatus[indx].syncActualPos;
        m_MotionStatus[indx].encoderCommand = gMotorStatus[indx].encoderCommand;
        m_MotionStatus[indx].encoderFeedback = gMotorStatus[indx].encoderFeedback;
        m_MotionStatus[indx].velocityCmd = gMotorStatus[indx].velocityCmd;
        m_MotionStatus[indx].actualVelocity = gMotorStatus[indx].actualVelocity;
        m_MotionStatus[indx].velocityLag = gMotorStatus[indx].velocityLag;
        m_MotionStatus[indx].torqueCmd = gMotorStatus[indx].torqueCmd;
        m_MotionStatus[indx].actualTorque = gMotorStatus[indx].actualTorque;
        m_MotionStatus[indx].actualFollowingError = gMotorStatus[indx].actualFollowingError;
        m_MotionStatus[indx].compensation.backlashCompensation = gMotorStatus[indx].compensation.backlashCompensation;
        m_MotionStatus[indx].compensation.pitchErrorCompensation = gMotorStatus[indx].compensation.pitchErrorCompensation;
        m_MotionStatus[indx].compensation.pitchErrorCompensation2D = gMotorStatus[indx].compensation.pitchErrorCompensation2D;
        m_MotionStatus[indx].compensation.totalPosCompensation = gMotorStatus[indx].compensation.totalPosCompensation;
        m_MotionStatus[indx].opState = gMotorStatus[indx].opState;
        m_MotionStatus[indx].detailOpState = gMotorStatus[indx].detailOpState;
        m_MotionStatus[indx].axisCommandMode = gMotorStatus[indx].axisCommandMode;
        m_MotionStatus[indx].axisSyncMode = gMotorStatus[indx].axisSyncMode;
        m_MotionStatus[indx].followingErrorAlarm = gMotorStatus[indx].followingErrorAlarm;
        m_MotionStatus[indx].commandReady = gMotorStatus[indx].commandReady;
        m_MotionStatus[indx].motionPaused = gMotorStatus[indx].motionPaused;
        m_MotionStatus[indx].profileTotalMilliseconds = gMotorStatus[indx].profileTotalMilliseconds;
        m_MotionStatus[indx].profileAccMilliseconds = gMotorStatus[indx].profileAccMilliseconds;
        m_MotionStatus[indx].profileCruiseMilliseconds = gMotorStatus[indx].profileCruiseMilliseconds;
        m_MotionStatus[indx].profileDecMilliseconds = gMotorStatus[indx].profileDecMilliseconds;
        m_MotionStatus[indx].profileRemainingMilliseconds = gMotorStatus[indx].profileRemainingMilliseconds;
        m_MotionStatus[indx].profileTargetPos = gMotorStatus[indx].profileTargetPos;
        m_MotionStatus[indx].profileTotalDistance = gMotorStatus[indx].profileTotalDistance;
        m_MotionStatus[indx].profileRemainingDistance = gMotorStatus[indx].profileRemainingDistance;
        m_MotionStatus[indx].intplVelocity = gMotorStatus[indx].intplVelocity;
        m_MotionStatus[indx].intplSegment = gMotorStatus[indx].intplSegment;
        m_MotionStatus[indx].cmdAcc = gMotorStatus[indx].cmdAcc;
        m_MotionStatus[indx].accFlag = gMotorStatus[indx].accFlag;
        m_MotionStatus[indx].decFlag = gMotorStatus[indx].decFlag;
        m_MotionStatus[indx].inPos = gMotorStatus[indx].inPos;
        m_MotionStatus[indx].inPos2 = gMotorStatus[indx].inPos2;
        m_MotionStatus[indx].inPos3 = gMotorStatus[indx].inPos3;
        m_MotionStatus[indx].inPos4 = gMotorStatus[indx].inPos4;
        m_MotionStatus[indx].inPos5 = gMotorStatus[indx].inPos5;
        m_MotionStatus[indx].cmdDistributionEnd = gMotorStatus[indx].cmdDistributionEnd;
        m_MotionStatus[indx].posSet = gMotorStatus[indx].posSet;
        m_MotionStatus[indx].delayedPosSet = gMotorStatus[indx].delayedPosSet;
        m_MotionStatus[indx].cmdDistributionEndDelayedPosSetDiff = gMotorStatus[indx].cmdDistributionEndDelayedPosSetDiff;
        m_MotionStatus[indx].positiveLS = gMotorStatus[indx].positiveLS;
        m_MotionStatus[indx].negativeLS = gMotorStatus[indx].negativeLS;
        m_MotionStatus[indx].nearPositiveLS = gMotorStatus[indx].nearPositiveLS;
        m_MotionStatus[indx].nearNegativeLS = gMotorStatus[indx].nearNegativeLS;
        m_MotionStatus[indx].externalPositiveLS = gMotorStatus[indx].externalPositiveLS;
        m_MotionStatus[indx].externalNegativeLS = gMotorStatus[indx].externalNegativeLS;
        m_MotionStatus[indx].positiveSoftLimit = gMotorStatus[indx].positiveSoftLimit;
        m_MotionStatus[indx].negativeSoftLimit = gMotorStatus[indx].negativeSoftLimit;
        m_MotionStatus[indx].homeState = gMotorStatus[indx].homeState;
        m_MotionStatus[indx].homeError = gMotorStatus[indx].homeError;
        m_MotionStatus[indx].homeSwitch = gMotorStatus[indx].homeSwitch;
        m_MotionStatus[indx].homeDone = gMotorStatus[indx].homeDone;
        m_MotionStatus[indx].homePaused = gMotorStatus[indx].homePaused;
        m_MotionStatus[indx].homeOffset = gMotorStatus[indx].homeOffset;
        m_MotionStatus[indx].cmdPosToFbPosFlag = gMotorStatus[indx].cmdPosToFbPosFlag;
        m_MotionStatus[indx].execSuperimposedMotion = gMotorStatus[indx].execSuperimposedMotion;
        m_MotionStatus[indx].singleTurnCounter = gMotorStatus[indx].singleTurnCounter;
        m_MotionStatus[indx].motionComplete = gMotorStatus[indx].motionComplete;
    }
}

UINT CSlaveMotorStatus::StartSlaveMotorStatus(LPVOID wParam)
{
    bool bLoop = true;
    int err = ErrorCode::None;
    CSlaveMotorStatus* pThis = reinterpret_cast<CSlaveMotorStatus*>(wParam);
    CString strHostMsg;
    //double dblActualTorqueOld = 0.0, dblActualTorqueNew = 5.1;
    TRACE(_T("[PWR] Slave Motor Task Synchronize Wait Event\n"));
    pThis->WaitForSingleObjectTime(pThis->GetThreadStartSync(), 5, TIME1000MS);
    TRACE(_T("[PWR] Slave Motor Synchronize Recv Event\n"));
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (pThis->GetRunning() == true)
    {        
        if (pThis->IsTerminated(TIME10MS) == true)
        {
            TRACE(_T("[PWR] CSlaveMotorStatus(0x%X) Terminated\n"), pThis->GetThreadID());
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_SLAVE_MOTOR_READTIME);
        //    continue;
        //}
        if (pThis->IsDiffMotorStatus() == true)
        {
            pThis->CopyMasterStatus();
            pThis->CopyMotorStatus();
        }
        ThreadSleep(THREAD_SLAVE_MOTOR_READTIME);
    };
    TRACE(_T("[PWR] CSlaveMotorStatus(0x%X) Quit\n"), pThis->GetThreadID());
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

bool CSlaveMotorStatus::GetWmx3MasterInvalidLicenseError()
{
    return m_MasterInfo.invalidLicenseError;
}

WMX3EngineState CSlaveMotorStatus::GetWmx3MasterEngineState()
{
    return m_MasterInfo.engineState;
}

double CSlaveMotorStatus::GetWmx3MasterCycleTimeMilliseconds(long no)
{
    return m_MasterInfo.cycleTimeMilliseconds[no];
}

long long CSlaveMotorStatus::GetWmx3MasterCycleCounter(long no)
{
    return m_MasterInfo.cycleCounter[no];
}

bool CSlaveMotorStatus::GetWmx3MasterEmergencyStop()
{
    return m_MasterInfo.emergencyStop;
}

WMX3EmgStopLevel CSlaveMotorStatus::GetWmx3MasterEmgStopLevel()
{
    return m_MasterInfo.emergencyStopLevel;
}

WMX3HomeState CSlaveMotorStatus::GetWmx3HomeState(int nAxis)
{
    return m_MotionStatus[nAxis].homeState;
}

bool CSlaveMotorStatus::GetWmx3AmpAlarm(int nAxis)
{
    return m_MotionStatus[nAxis].ampAlarm;
}

int CSlaveMotorStatus::GetAmpAlarmCode(int nAxis)
{
    return m_MotionStatus[nAxis].ampAlarmCode;
}

bool CSlaveMotorStatus::GetWmx3IsInpos(int nAxis)
{
    return m_MotionStatus[nAxis].inPos;
}

bool CSlaveMotorStatus::GetWmx3IsPosSet(int nAxis)
{
    return m_MotionStatus[nAxis].posSet;
}

bool CSlaveMotorStatus::GetWmx3IsDelayedPosSet(int nAxis)
{
    return m_MotionStatus[nAxis].delayedPosSet;
}

bool CSlaveMotorStatus::GetWmx3CheckServoOn(int nAxis)
{
    if (gcPowerLog->IsShowMotionLog() == true)
    {
        TRACE(_T("[PWR] GetWmx3CheckServoOn Axis(%d) %s\n"), nAxis, m_MotionStatus[nAxis].servoOn == true ? _T("On") : _T("Off"));
    }
    return m_MotionStatus[nAxis].servoOn;
}

bool CSlaveMotorStatus::GetWmx3IsAxisIdle(int nAxis)
{
    bool bRet = false;
    if (m_MotionStatus[nAxis].opState == WMX3OperationState::Idle)
    {
        bRet = true;
    }
    return bRet;
}

bool CSlaveMotorStatus::GetWmx3IsAxisHoming(int nAxis)
{
    bool bRet = false;
    if (m_MotionStatus[nAxis].opState == WMX3OperationState::Home)
    {
        bRet = true;
    }
    return bRet;
}

bool CSlaveMotorStatus::GetWmx3IsAxisHomeDone(int nAxis)
{
    return m_MotionStatus[nAxis].homeDone;
}

bool CSlaveMotorStatus::GetWmx3IsAxisMotionComplete(int nAxis)
{
    return m_MotionStatus[nAxis].motionComplete;
}

bool CSlaveMotorStatus::GetWmx3IsAxisBusy(int nAxis)
{
    bool bRet = false;
    if (m_MotionStatus[nAxis].opState != WMX3OperationState::Idle)
    {
        bRet = true;
    }
    return bRet;
}

bool CSlaveMotorStatus::GetWmx3IsNegativeLimitSwitchOn(int nAxis)
{
    return m_MotionStatus[nAxis].negativeLS;
}

bool CSlaveMotorStatus::GetWmx3IsPositiveLimitSwitchOn(int nAxis)
{
    return m_MotionStatus[nAxis].positiveLS;
}

bool CSlaveMotorStatus::GetWmx3IsNegativeVirtualLimitSwitchOn(int nAxis)
{
    return m_MotionStatus[nAxis].negativeSoftLimit;
}

bool CSlaveMotorStatus::GetWmx3IsPositiveVirtualLimitSwitchOn(int nAxis)
{
    return m_MotionStatus[nAxis].positiveSoftLimit;
}

bool CSlaveMotorStatus::GetWmx3IsHomeSwitchOn(int nAxis)
{
    return m_MotionStatus[nAxis].homeSwitch;
}

double CSlaveMotorStatus::GetWmx3ActualTorque(int nAxis)
{
    return m_MotionStatus[nAxis].actualTorque;
}

double CSlaveMotorStatus::GetWmx3oneDCompensation(int nAxis)
{
    return m_MotionStatus[nAxis].compensation.pitchErrorCompensation;
}

double CSlaveMotorStatus::GetWmx3twoDCompensation(int nAxis)
{
    return m_MotionStatus[nAxis].compensation.pitchErrorCompensation2D;
}

WMX3AxisSyncMode CSlaveMotorStatus::GetAxisSyncMode(int nAxis)
{
    return m_MotionStatus[nAxis].axisSyncMode;
}

double CSlaveMotorStatus::GetWmx3SyncOffset(int nAxis)
{
    return m_MotionStatus[nAxis].syncOffset;
}

double CSlaveMotorStatus::GetWmx3SyncPhaseOffset(int nAxis)
{
    return m_MotionStatus[nAxis].syncPhaseOffset;
}

double CSlaveMotorStatus::GetWmx3HomeOffset(int nAxis)
{
    return m_MotionStatus[nAxis].homeOffset;
}
int CSlaveMotorStatus::GetWmx3EncoderFeedback(int nAxis)
{
    return m_MotionStatus[nAxis].encoderFeedback;
}
