#include "pch.h"
#include "CPowerBuzzer.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CPowerMain.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CReadJobFile.h"
//#include "ErrorCode.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
//#include "CMotorPower.h"

CPowerBuzzer* gcPowerBuzzer;
CPowerBuzzer::CPowerBuzzer()
{
    m_bBuzzerStop = false;
	m_buzzserNoUse = false;
    this->m_BuzzerOnTime = 1;
    this->m_BuzzerStep = BuzzerStep::INIT;
    this->m_ShowID = 0;
    this->_lastSafetyAlarmCode = NO_ERR;
}

CPowerBuzzer::~CPowerBuzzer()
{
}

int CPowerBuzzer::GetLastSafetyAlarmCode()
{
    return this->_lastSafetyAlarmCode;
}

int CPowerBuzzer::SetLastSafetyAlarmCode(const int& safetyAlarmCode)
{
    CString temp; temp.Format(L"%d", safetyAlarmCode); TRACE_FILE_FUNC_LINE_(CStringA)temp);
    this->_lastSafetyAlarmCode = safetyAlarmCode;
    return 0;
}

BuzzerStep CPowerBuzzer::GetStep()
{
    return m_BuzzerStep;
}

void CPowerBuzzer::SetStep(BuzzerStep BuzzerStep)
{
    m_BuzzerStep = BuzzerStep;
}

void CPowerBuzzer::SetBuzzserNoUse(bool nouse)
{
	if (m_buzzserNoUse != nouse)
	{
		TRACE(_T("[PWR] SetBuzzserNoUse %d -> %d \n"), m_buzzserNoUse, nouse);
	}

	m_buzzserNoUse = nouse;
}

void CPowerBuzzer::SetBuzzerStop(bool BuzzerStop)
{
    m_bBuzzerStop = BuzzerStop;
}

bool CPowerBuzzer::GetBuzzerStop()
{
    return m_bBuzzerStop;
}

long CPowerBuzzer::SetBuzzerOnTime(long BuzzerOnTime)
{
    m_BuzzerOnTime = BuzzerOnTime;
    return NO_ERR;
}

long CPowerBuzzer::GetBuzzerOnTime()
{
    return m_BuzzerOnTime;
}

void CPowerBuzzer::BuzzerOn()
{
	if (m_buzzserNoUse == true)
	{
		return;
	}

    SetBuzzerStop(false);

	if (ReadOutputOne(OUT_FBUZZER) == OUTOFF)
	{
		TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
		TRACE(_T("[PWR] !!!!!!!!!!!!!!! BUZZER ON  !!!!!!!!!!!!!!\n"));
		TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
	}

    OutputOne(OUT_FBUZZER, OUTON);
}

void CPowerBuzzer::BuzzerOff()
{
	if (ReadOutputOne(OUT_FBUZZER) == OUTON)
	{
		TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
		TRACE(_T("[PWR] !!!!!!!!!!!!!!! BUZZER OFF !!!!!!!!!!!!!!\n"));
		TRACE(_T("[PWR] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"));
	}

    SetBuzzerStop(true);
    OutputOne(OUT_FBUZZER, OUTOFF);
}

UINT CPowerBuzzer::BuzzerControl(LPVOID wParam)
{
    CString strLog;
    bool bLoop = true;
    long Err = ErrorCode::None;
    CPowerBuzzer* pThis = reinterpret_cast<CPowerBuzzer*>(wParam);
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    ULONGLONG BuzzerOnGetTime = _time_get(), BuzzerOnElapsed = 0;
    BuzzerStep BuzStep = BuzzerStep::INIT, BuzOldStep = BuzzerStep::STOP;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_BUZZER_READTIME) == true)
        {
            TRACE(_T("[PWR] CPowerBuzzer(0x%x) Terminated\n"), pThis->m_ShowID);
            strLog.Format(_T("[PWR] CPowerBuzzer(0x%x) Terminated\n"), pThis->m_ShowID);
            gcPowerLog->Logging(strLog);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_BUZZER_READTIME);
        //    continue;
        //}
        if (BuzOldStep != pThis->GetStep())
        {
            Elapsed = _time_elapsed(GetTime);
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] CPowerBuzzer Step:%d Time:%d[ms]\n"), pThis->GetStep(), Elapsed);
            }
            BuzOldStep = pThis->GetStep();
            GetTime = _time_get();
        }
        switch (pThis->GetStep())
        {
        case BuzzerStep::INIT:
            break;
        case BuzzerStep::STOP:
            break;
        case BuzzerStep::START:
            break;
        case BuzzerStep::ON:
            pThis->BuzzerOn();
            if (pThis->GetBuzzerOnTime() > 0)
            {
                BuzzerOnGetTime = _time_get();
                BuzzerOnElapsed = 0;
                pThis->SetStep(BuzzerStep::TIMEOFF);
            }
            break;
        case BuzzerStep::TIMEOFF:
            BuzzerOnElapsed = _time_elapsed(BuzzerOnGetTime);
            if (BuzzerOnElapsed > pThis->GetBuzzerOnTime())
            {
                pThis->SetStep(BuzzerStep::OFF);
            }
            break;
        case BuzzerStep::OFF:
            pThis->BuzzerOff();
            pThis->SetStep(BuzzerStep::IDLE);
            break;
        case BuzzerStep::IDLE:
            break;
        case BuzzerStep::SELFQUIT:
            bLoop = false;
            break;
        case BuzzerStep::QUIT:
            break;
        }
        if (bLoop == false)
        {
            break;
        }
        ThreadSleep(THREAD_BUZZER_READTIME);
    }
    TRACE(_T("[PWR] CPowerBuzzer(0x%x) Quit\n"), pThis->m_ShowID);
    strLog.Format(_T("[PWR] CPowerBuzzer(0x%x) Quit\n"), pThis->m_ShowID);
    gcPowerLog->Logging(strLog);
	pThis->SetEnd(true);

    return 0;
}

void CPowerBuzzer::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetThreadName(_T("thBuzzer"));
    SetRunning(true);
    SetEnd(false);
    lpStartAddress = (_beginthreadex_proc_type)BuzzerControl;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] BuzzerControl Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] BuzzerControl Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}
