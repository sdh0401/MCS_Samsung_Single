#include "pch.h"
#include "CPowerSwitchPanel.h"
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

CPowerSwitchPanel* gcPowerSwitchPanel;
CPowerSwitchPanel::CPowerSwitchPanel()
{
    m_ShowID = 0;
    m_SwitchPanelStep = SwitchPanelStep::INIT;
}

CPowerSwitchPanel::~CPowerSwitchPanel()
{
}

SwitchPanelStep CPowerSwitchPanel::GetStep()
{
    return m_SwitchPanelStep;
}

void CPowerSwitchPanel::SetStep(SwitchPanelStep SwitchPanelStep)
{
    m_SwitchPanelStep = SwitchPanelStep;
}

UINT CPowerSwitchPanel::ReadSwitchPanel(LPVOID wParam)
{
    int err = ErrorCode::None;
    bool bLoop = true, bStopFirst = true, bStartFirst = true, bDoorFirst = true, bResetFirst = true;
    long Gantry = FRONT_GANTRY;
    double Ratio = 1.0;
    CPowerSwitchPanel* pThis = reinterpret_cast<CPowerSwitchPanel*>(wParam);
    SwitchPanelStep OldStep = SwitchPanelStep::STOP;
    CApplicationTime* pTime = new CApplicationTime();
    CApplicationTime* pStopKeyTime = new CApplicationTime();
    CApplicationTime* pDoorKeyTime = new CApplicationTime();
    CApplicationTime* pStartKeyTime = new CApplicationTime();
    CApplicationTime* pResetKeyTime = new CApplicationTime();
    long StopKeyPushDnTime = 0, DoorKeyPushDnTime = 0, StartKeyPushDnTime = 0, ResetKeyPushDnTime = 0, StepTimeChk = 0;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_SWITCH_PANEL_READTIME) == true)
        {
            bLoop = false;
            TRACE(_T("[PWR] CPowerSwitchPanel(0x%x) Terminated\n"), pThis->m_ShowID);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_SWITCH_PANEL_READTIME);
        //    continue;
        //}
        if (OldStep != pThis->GetStep())
        {
            StepTimeChk = pTime->TimeElapsed();
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] CPowerSwitchPanel Step:%d Time:%d[ms]\n"), pThis->GetStep(), StepTimeChk);
            }
            OldStep = pThis->GetStep();
            pTime->TimeGet();
        }
        switch (pThis->GetStep())
        {
        case SwitchPanelStep::INIT:
            break;
        case SwitchPanelStep::STOP:
            pThis->SetStep(SwitchPanelStep::START);
            break;
        case SwitchPanelStep::START:
            bStopFirst = true;
            if (InputTimeOne(IN_STOP_PANEL_KEY, INON, TIME20MS) == true)
            {
                pStopKeyTime->TimeGet();
                OutputOne(OUT_STOP_PANEL_LED, OUTON);
                pThis->SetStep(SwitchPanelStep::STOPKEY_PUSHDN);
            }
            else if (InputTimeOne(IN_FDOOR_LOCK_PANEL_KEY, INON, TIME20MS) == true)
            {
                pDoorKeyTime->TimeGet();
                OutputOne(OUT_FDOOR_LOCK_PANEL_LED, OUTON);
                pThis->SetStep(SwitchPanelStep::DOORKEY_PUSHDN);
            }
            else if (InputTimeOne(IN_RDOOR_LOCK_PANEL_KEY, INON, TIME20MS) == true)
            {
                pDoorKeyTime->TimeGet();
                OutputOne(OUT_RDOOR_LOCK_PANEL_LED, OUTON);
                pThis->SetStep(SwitchPanelStep::DOORKEY_PUSHDN);
            }
            else if (InputTimeOne(IN_START_PANEL_KEY, INON, TIME20MS) == true)
            {
                pStartKeyTime->TimeGet();
                OutputOne(OUT_START_PANEL_LED, OUTON);
                pThis->SetStep(SwitchPanelStep::STARTKEY_PUSHDN);
            }
            else if (InputTimeOne(IN_RESET_PANEL_KEY, INON, TIME20MS) == true)
            {
                pResetKeyTime->TimeGet();
                OutputOne(OUT_RESET_PANEL_LED, OUTON);
                pThis->SetStep(SwitchPanelStep::RESETKEY_PUSHDN);
            }
            break;
        case SwitchPanelStep::STOPKEY_PUSHDN:
            if (InputOne(IN_STOP_PANEL_KEY) == INON)
            {
                StopKeyPushDnTime = pStopKeyTime->TimeElapsed();
                if (StopKeyPushDnTime > STOPKEY_PUSHDN_RECOGNITION_TIME)
                {
                    if (bStopFirst == true)
                    {
                        if (GetRunMode() == PROD_RUN)
                        {
                            TRACE(_T("[PWR] Machine is running but Push down stop key\n"));
                            CallbackHMI_Pause();
                        }
                        else if (GetRunMode() == PAUSE_MODE)
                        {
                            TRACE(_T("[PWR] Machine is Pause and want to stop\n"));
                            CallbackHMI_StopNow();
                        }
                        else
                        {
                            TRACE(_T("[PWR] Stop Key Push Down\n"));
                        }
                        pStopKeyTime->TimeGet();
                        bStopFirst = false;
                    }
                }
            }
            else
            {
                OutputOne(OUT_STOP_PANEL_LED, OUTOFF);
                pThis->SetStep(SwitchPanelStep::START);
                bStopFirst = true;
            }
            break;
        case SwitchPanelStep::DOORKEY_PUSHDN:
            if (InputOne(IN_FDOOR_LOCK_PANEL_KEY) == INON)
            {
                DoorKeyPushDnTime = pDoorKeyTime->TimeElapsed();
                if (DoorKeyPushDnTime > DOORKEY_PUSHDN_RECOGNITION_TIME)
                {
                    if (bDoorFirst == true)
                    {
                        if (GetRunMode() == NORMAL_MODE)
                        {
                            if (g_bUserDoorPush == false)
                            {
                                TRACE(_T("[PWR] Open Door After MoveZSafty Motion\n"));
								MoveZStandySkipServoOff(Gantry, GetStandByZ(Gantry), Ratio);
                                ThreadSleep(TIME200MS);
                            }
                            TRACE(_T("[PWR] Servo Off\n"));
                            //gServoAllOff();
                            gServoAllOffWithoutConv();
                            TRACE(_T("[PWR] Open Door\n"));
                            DoorLockingControl(false);
                            g_bUserDoorPush = true;
                            g_UserOpenDoorGetTime = _time_get();
                        }
                        else
                        {
                            TRACE(_T("[PWR] Machine is running, push Door key after Stop\n"));
                        }
                        pDoorKeyTime->TimeGet();
                        bDoorFirst = false;
                    }
                }
            }
            else
            {
                OutputOne(OUT_FDOOR_LOCK_PANEL_LED, OUTOFF);
                pThis->SetStep(SwitchPanelStep::START);
                bDoorFirst = true;
            }
            break;
        case SwitchPanelStep::STARTKEY_PUSHDN:
            if (InputOne(IN_START_PANEL_KEY) == INON)
            {
                StartKeyPushDnTime = pStartKeyTime->TimeElapsed();
                if (StartKeyPushDnTime > STARTKEY_PUSHDN_RECOGNITION_TIME)
                {
                    if (bStartFirst == true)
                    {
                        if (GetRunMode() == PAUSE_MODE)
                        {
                            TRACE(_T("[PWR] Machine resume to run\n"));
                            CallbackHMI_Resume();
                        }
                        else if (GetRunMode() == NORMAL_MODE)
                        {
                            TRACE(_T("[PWR] Machine is ready\n"));
                            if (GetHMIReadyToRun() == 0)
                            {
                                TRACE(_T("[PWR] Machine is ready to prepare\n"));
                                SendToPrepareRun();
                            }
                            else if (GetHMIReadyToRun() == 1)
                            {
                                TRACE(_T("[PWR] Machine is ready to run\n"));
                                SendToRun();
                            }
                        }
                        else
                        {
                            TRACE(_T("[PWR] Start Key Push Down but machine already is running\n"));
                        }
                        pStartKeyTime->TimeGet();
                        bStartFirst = false;
                    }
                }
            }
            else
            {
                OutputOne(OUT_START_PANEL_LED, OUTOFF);
                pThis->SetStep(SwitchPanelStep::START);
                bStartFirst = true;
            }
            break;
        case SwitchPanelStep::RESETKEY_PUSHDN:
            if (InputOne(IN_RESET_PANEL_KEY) == INON)
            {
                ResetKeyPushDnTime = pResetKeyTime->TimeElapsed();
                if (ResetKeyPushDnTime > RESETKEY_PUSHDN_RECOGNITION_TIME)
                {
                    if (bResetFirst == true)
                    {
                        TRACE(_T("[PWR] Reset Key Push Down\n"));
                        pResetKeyTime->TimeGet();
                        bResetFirst = false;
                    }
                }
            }
            else
            {
                OutputOne(OUT_RESET_PANEL_LED, OUTOFF);
                pThis->SetStep(SwitchPanelStep::START);
                bResetFirst = true;
            }
            break;
        case SwitchPanelStep::SELFQUIT:
            bLoop = false;
            break;
        case SwitchPanelStep::QUIT:
            break;
        }
        if (bLoop == false)
        {
            break;
        }
        ThreadSleep(THREAD_MOTOR_READTIME);
    }
    TRACE(_T("[PWR] CPowerSwitchPanel(0x%x) Quit\n"), pThis->m_ShowID);
    delete pTime;
    delete pStopKeyTime;
    delete pDoorKeyTime;
    delete pStartKeyTime;
    delete pResetKeyTime;
    return 0;
}

void CPowerSwitchPanel::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thSwitchPanel"));
    SetRunning(true);
    lpStartAddress = (_beginthreadex_proc_type)ReadSwitchPanel;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] SwitchPanel IO Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] SwitchPanel IO Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
