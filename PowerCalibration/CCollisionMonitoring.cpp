#include "pch.h"
#include "CCollisionMonitoring.h"
#include "GlobalIODefine.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "AxisInformation.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CPowerHMI.h"
//#include "ErrorCode.h"
#include "CPowerGantry.h"
#include <deque>

using namespace std;

#define LOG_MAX 200

CCollisionMonitoring* gcCollisionMonitoring;
CCollisionMonitoring::CCollisionMonitoring()
{
    m_ShowID = 0;
}

CCollisionMonitoring::~CCollisionMonitoring()
{
}

//COLLISION_MONITORING CCollisionMonitoring::GetMonitoringData(long Gantry)
//{
//    CString strAxis = GetAxisY1(Gantry);
//    COLLISION_MONITORING Data;
//
//    Data.FeedBackPosition = ReadPosition(strAxis);
//    Data.CommandVelocity = ReadOneVelocity(strAxis);
//    Data.Idle = GetOneIdle(strAxis);
//
//    if (Gantry == FRONT_GANTRY)
//    {
//        Data.Target = gcPowerFrontGantry->GetTargetPosition();
//    }
//    else
//    {
//        Data.Target = gcPowerRearGantry->GetTargetPosition();
//    }
//
//    Data.FeedbackVelocity = ReadOneVelocity(strAxis);
//    Data.CommandVelocity = ReadOneCommandVelocity(strAxis);
//    //Data.IntplVelocity = ReadintplVelocity(strAxis);
//    Data.Msg.Empty();
//
//    return Data;
//}

UINT CCollisionMonitoring::CollisionMonitoring(LPVOID wParam)
{
    CCollisionMonitoring* pThis = reinterpret_cast<CCollisionMonitoring*>(wParam); 
   
    CString strFX = GetAxisX(FRONT_GANTRY);
    CString strFY = GetAxisY1(FRONT_GANTRY);
	CString axisName;
	std::deque<CString> AxisNameList;
	long stopCnt = 0;

	AxisNameList.push_back(strFX);
	AxisNameList.push_back(strFY);

    COLLISION_MONITORING CurrentDataFY;
    COLLISION_MONITORING CurrentDataRY;
    COLLISION_MONITORING OldDataFY;
    COLLISION_MONITORING OldDataRY;

    //CurrentDataFY = OldDataFY = pThis->GetMonitoringData(FRONT_GANTRY);
    //CurrentDataRY = OldDataRY = pThis->GetMonitoringData(REAR_GANTRY);

    std::deque<COLLISION_MONITORING> LogDataFY;
    std::deque<COLLISION_MONITORING> LogDataRY;

    LogDataFY.assign(LOG_MAX, CurrentDataFY);
    LogDataRY.assign(LOG_MAX, CurrentDataRY);

    LogDataFY.clear();
    LogDataRY.clear();

    bool YCmdLock = false;
    bool DangerStatus = false;
    bool DangerFY = false;
    bool DangerRY = false; 
    bool PauseFY = false;
    bool PauseRY = false;

    bool IsCollisionAreaTargetFY = true;
    bool IsCollisionAreaTargetRY = true;
    bool IsCollisionAreaFeedBackFY = true;
    bool IsCollisionAreaFeedBackRY = true;
    
    double LogGap = 0.01;

    CString strAlarm = _T("Collision Error");
	bool showMsg = false;

    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_WMX3_MOTOR_READTIME) == true)
        {
            TRACE(_T("[PWR] CCollisionMonitoring(0x%x) Terminated\n"), pThis->m_ShowID);
            break;
        }

		if (GetRunModeNoLog() == NORMAL_MODE)
		{
			for (long cnt = 0; cnt < AxisNameList.size(); cnt++)
			{
				axisName = AxisNameList.at(cnt);
				if (IsAxisStatusJog(axisName) == true)
				{
					if (ReadOneCommandVelocity(axisName) > 0.0 && IsNearSWLimitPlus(axisName) == true)
					{
						StopOne(axisName);
						TRACE(_T("[PWR] Collision JogStop %s\n"), axisName);
					}
					else if (ReadOneCommandVelocity(axisName) < 0.0 && IsNearSWLimitMinus(axisName) == true)
					{
						StopOne(axisName);
						TRACE(_T("[PWR] Collision JogStop %s\n"), axisName);
					}
				}
			}
			ThreadSleep(TIME10MS);

		}
		else
		{
			ThreadSleep(TIME100MS);
		}
    }

    TRACE(_T("[PWR] CCollisionMonitoring(0x%x) Quit\n"), pThis->m_ShowID);
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CCollisionMonitoring::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thMachineInformation"));
    SetRunning(true);
    SetEnd(false);
    lpStartAddress = (_beginthreadex_proc_type)CollisionMonitoring;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] MonitoringCollision Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] MonitoringCollision Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
