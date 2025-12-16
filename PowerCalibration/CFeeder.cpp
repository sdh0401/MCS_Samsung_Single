#include "pch.h"
#include "CFeeder.h"
#include "AxisInformation.h"
#include "DefineThreadLoopTime.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"

CFeeder* gcFeeder[MAXFEEDERNO];
CFeeder::CFeeder(long FeederNo)
{
    m_RunStep = FeederStep::STOP;
    m_ShowID = 0;
    m_FeederNo = FeederNo;
    m_ReadyNo = 1;
    m_ReleaseNo = 1;
    m_Empty = false;
    m_ProdRunMode = RUN_REAL;
    m_ReadyWaitTime = TIME50MS;
}

CFeeder::~CFeeder()
{
}

long CFeeder::GetNo()
{
    return m_FeederNo;
}

FeederStep CFeeder::GetStep()
{
    return m_RunStep;
}

void CFeeder::SetStep(FeederStep RunStep)
{
    m_RunStep = RunStep;
}

bool CFeeder::IsEmpty()
{
    return m_Empty;
}

void CFeeder::SetEmtpy(bool Empty)
{
    m_Empty = Empty;
    TRACE(_T("[PWR] Feeder(%03d) Empty(%d)\n"), GetNo(), Empty);
}

long CFeeder::GetReadyNo()
{
    return m_ReadyNo;
}

long CFeeder::GetReleaseNo()
{
    return m_ReleaseNo;
}

long CFeeder::GetProdRunMode()
{
    return m_ProdRunMode;
}

void CFeeder::SetProdRunMode(long ProdRunMode)
{
    m_ProdRunMode = ProdRunMode;
}

long CFeeder::SetReadyNo(long ReadyNo)
{
    m_ReadyNo = ReadyNo;
    TRACE(_T("[PWR] Feeder(%03d) SetReadyNo(%d)\n"), GetNo(), m_ReadyNo);
    return NO_ERR;
}

long CFeeder::SetReleaseNo(long ReleaseNo)
{
    m_ReleaseNo = ReleaseNo;
    TRACE(_T("[PWR] Feeder(%03d) SetReleaseNo(%d)\n"), GetNo(), m_ReleaseNo);
    return NO_ERR;
}
long CFeeder::SetReadyWaitTime(long WaitTime)
{
    m_ReadyWaitTime = WaitTime;
    return 0;
}

void CFeeder::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    CString strThreadName;
    strThreadName.Format(_T("thFeeder%03d"), GetNo());
    SetThreadName(strThreadName);
    lpStartAddress = (_beginthreadex_proc_type)StartFeeder;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] CFeeder Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CFeeder Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

UINT CFeeder::StartFeeder(LPVOID wParam)
{
    CFeeder* pThis = reinterpret_cast<CFeeder*>(wParam);
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    long TimeChk = 0, Gantry = FRONT_GANTRY;
    long Err = NO_ERR;
    long ReadyIO = IO_NOUSE;
    long ReadyCount = 0;
    bool bReady = false;
    long ReadyWaitTIme = pThis->m_ReadyWaitTime;
    FeederStep OldStep = FeederStep::STOP;
    CApplicationTime* pTime = new CApplicationTime();
    ULONGLONG GetTime = 0, Elapsed = 0;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_FEEDER_READTIME) == true)
        {
            TRACE(_T("[PWR] Feeder(%03d) Terminated\n"), pThis->GetNo());
            break;
        }
        if (GetGlobalStatusError() == true)
        {
            TRACE(_T("[PWR] Feeder(%03d) GetGlobalStatusError(%d)\n"), pThis->GetNo(), GetGlobalStatusError());
            break;
        }
        if (GetMachineState() == STATE_STOPNOW)
        {
            TRACE(_T("[PWR] Feeder(%03d) GetMachineState(%d)\n"), pThis->GetNo(), GetMachineState());
            Err = STOP_NOW;
            break;
        }
        if ((OldStep != pThis->GetStep()))
        {
            Elapsed = pTime->TimeElapsed();
            if (gcPowerLog->IsShowFeederLog() == true)
            {
                TRACE(_T("[PWR] Feeder(%03d) Step:%d Elapsed:%d[ms]\n"), pThis->GetNo(), pThis->GetStep(), Elapsed);
            }
            OldStep = pThis->GetStep();
            pTime->TimeGet();
        }
        switch (pThis->GetStep())
        {
        case FeederStep::STOP:
            break;
        case FeederStep::START:
            ReadyIO = GetReadyIONoFromReadyNo(pThis->GetReadyNo());
            if (ReadyIO != IO_NOUSE)
            {
                pThis->SetStep(FeederStep::NOTREADY);
            }
            break;
        case FeederStep::NOTREADY:
            bReady = InputTimeOne(ReadyIO, INON, ReadyWaitTIme);
            if (bReady == true)
            {
                ReadyCount = 0;
                pThis->SetStep(FeederStep::READY);
            }
            break;
        case FeederStep::READY:
            bReady = InputTimeOne(ReadyIO, INON, TIME5MS);
            if (bReady == false)
            {
                pThis->SetStep(FeederStep::NOTREADY);
            }
            else
            {
                if (pThis->GetProdRunMode() == RUN_REAL)
                {
					if (pThis->IsEmpty() == true && InputElapsedTimeOne(ReadyIO, INON, TIME1000MS) == true)
					{
						TRACE(_T("[PWR] Feeder(%03d) Auto Empty Clear\n"), pThis->GetNo());
						pThis->SetEmtpy(false);
						//if (ReadyCount > 100)
						//{
						//    TRACE(_T("[PWR] Feeder(%03d) Auto ReStart Refill done\n"), pThis->GetNo());
						//    gFeederRefill(pThis->GetNo());
						//    ThreadSleep(TIME10MS);
						//    gFeederRefillDone();
						//    ReadyCount = 0;
						//    pThis->SetEmtpy(false);
						//}
						//else
						//{
						//    ReadyCount++;
						//}
					}
                }
            }
            break;

		case FeederStep::WAIT_READYOFF:
			bReady = InputTimeOne(ReadyIO, INOFF, TIME5MS);
			if (bReady == true)
			{
				pThis->SetStep(FeederStep::NOTREADY);
			}
			break;

        case FeederStep::SELFQUIT:
            bLoop = false;
            break;
        }
        if (bLoop == false)
        {
            break;
        }
    }
    delete pTime;
    TRACE(_T("[PWR] Feeder(%03d)(0x%X) Quit\n"), pThis->GetNo(), pThis->m_ShowID);
    return 0;
}