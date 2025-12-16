#include "pch.h"
#include "CSmemaControl.h"
#include "AxisInformation.h"
#include "DefineThreadLoopTime.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "Trace.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

CSmemaControl* gcSmemaControl;
CSmemaControl::CSmemaControl()
{
    m_ShowID = 0xFFFF;
    for (unsigned indx = 0; indx < MAX_CONV; ++indx)
    {
        m_bSmema[indx].PrevIn.Use = false;
        m_bSmema[indx].PrevIn.IO = IO_NOUSE;
        m_bSmema[indx].PrevIn.Status = INOFF;
        m_bSmema[indx].PrevOut.Use = false;
        m_bSmema[indx].PrevOut.IO = IO_NOUSE;
        m_bSmema[indx].PrevOut.Status = OUTOFF;
        m_bSmema[indx].NextIn.Use = false;
        m_bSmema[indx].NextIn.IO = IO_NOUSE;
        m_bSmema[indx].NextIn.Status = INOFF;
        m_bSmema[indx].NextOut.Use = false;
        m_bSmema[indx].NextOut.IO = IO_NOUSE;
        m_bSmema[indx].NextOut.Status = OUTOFF;
    }
    InitSmema();
}

CSmemaControl::~CSmemaControl()
{
}

void CSmemaControl::InitSmema()
{
    for (long Conv = ENTRY_CONV; Conv < MAX_CONV; ++Conv)
    {
        PrevSmemaOut(Conv, SMEMA_BUSY);
        NextSmemaOut(Conv, SMEMA_NOT_READY);
    }
}

void CSmemaControl::SetIO(long Conv, long PrevIn, long PrevOut, long NextIn, long NextOut)
{
    if (Conv == ENTRY_CONV)
    {
        if (PrevIn != IO_NOUSE)
        {
            m_bSmema[Conv].PrevIn.IO = PrevIn;
            m_bSmema[Conv].PrevIn.Use = true;
        }
        if (PrevOut != IO_NOUSE)
        {
            m_bSmema[Conv].PrevOut.IO = PrevOut;
            m_bSmema[Conv].PrevOut.Use = true;
        }
    }
    if (Conv == EXIT_CONV)
    {
        if (NextIn != IO_NOUSE)
        {
            m_bSmema[Conv].NextIn.IO = NextIn;
            m_bSmema[Conv].NextIn.Use = true;
        }
        if (NextOut != IO_NOUSE)
        {
            m_bSmema[Conv].NextOut.IO = NextOut;
            m_bSmema[Conv].NextOut.Use = true;
        }
    }
    TRACE(_T("[PWR] SetIO Conv:%d PrevIn:%d PrevOut:%d NextIn:%d NextOut:%d\n"), Conv,
        PrevIn, PrevOut, NextIn, NextOut);
}

UINT CSmemaControl::StartSmemaControl(LPVOID wParam)
{
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    bool chkstatus = false;
    CSmemaControl* pThis = reinterpret_cast<CSmemaControl*>(wParam);
    SmemaControl OldSmema[MAX_CONV];
    OldSmema[ENTRY_CONV].PrevIn.Status = IN_INIT;
    OldSmema[ENTRY_CONV].NextIn.Status = IN_INIT;
    OldSmema[WORK1_CONV].PrevIn.Status = IN_INIT;
    OldSmema[WORK1_CONV].NextIn.Status = IN_INIT;
    OldSmema[EXIT_CONV].PrevIn.Status = IN_INIT;
    OldSmema[EXIT_CONV].NextIn.Status = IN_INIT;
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_SMEMA_READTIME) == true)
        {
            TRACE(_T("[PWR] CSmemaControl(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
        chkstatus = InputTimeOne(pThis->m_bSmema[ENTRY_CONV].PrevIn.IO, INON, TIME20MS);
        pThis->m_bSmema[ENTRY_CONV].PrevIn.Status = (chkstatus == true) ? INON : INOFF;
        pThis->m_bSmema[ENTRY_CONV].NextIn.Status = pThis->m_bSmema[WORK1_CONV].PrevOut.Status;
        pThis->m_bSmema[WORK1_CONV].PrevIn.Status = pThis->m_bSmema[ENTRY_CONV].NextOut.Status;
        pThis->m_bSmema[WORK1_CONV].NextIn.Status = pThis->m_bSmema[EXIT_CONV].PrevOut.Status;
        pThis->m_bSmema[EXIT_CONV].PrevIn.Status = pThis->m_bSmema[WORK1_CONV].NextOut.Status;        
        chkstatus = InputTimeOne(pThis->m_bSmema[EXIT_CONV].NextIn.IO, INON, TIME20MS);
        pThis->m_bSmema[EXIT_CONV].NextIn.Status = (chkstatus == true) ? INON : INOFF;
        if (gcPowerLog->IsShowConveyorLog() == true)
        {
            if ((OldSmema[ENTRY_CONV].PrevIn.Status != pThis->m_bSmema[ENTRY_CONV].PrevIn.Status) || (OldSmema[ENTRY_CONV].NextIn.Status != pThis->m_bSmema[ENTRY_CONV].NextIn.Status))
            {
                TRACE(_T("[PWR] ENTRY PrevIn(Old:%d New:%d) NextIn(Old:%d New:%d)\n"), 
                    OldSmema[ENTRY_CONV].PrevIn.Status, pThis->m_bSmema[ENTRY_CONV].PrevIn.Status, 
                    OldSmema[ENTRY_CONV].NextIn.Status, pThis->m_bSmema[ENTRY_CONV].NextIn.Status);
                if(OldSmema[ENTRY_CONV].PrevIn.Status != pThis->m_bSmema[ENTRY_CONV].PrevIn.Status)
                    OldSmema[ENTRY_CONV].PrevIn.Status = pThis->m_bSmema[ENTRY_CONV].PrevIn.Status;
                if (OldSmema[ENTRY_CONV].NextIn.Status != pThis->m_bSmema[ENTRY_CONV].NextIn.Status)
                    OldSmema[ENTRY_CONV].NextIn.Status = pThis->m_bSmema[ENTRY_CONV].NextIn.Status;
            }
            if ((OldSmema[WORK1_CONV].PrevIn.Status != pThis->m_bSmema[WORK1_CONV].PrevIn.Status) || (OldSmema[WORK1_CONV].NextIn.Status != pThis->m_bSmema[WORK1_CONV].NextIn.Status))
            {
                TRACE(_T("[PWR] WORK1 PrevIn(Old:%d New:%d) NextIn(Old:%d New:%d)\n"),
                    OldSmema[WORK1_CONV].PrevIn.Status, pThis->m_bSmema[WORK1_CONV].PrevIn.Status,
                    OldSmema[WORK1_CONV].NextIn.Status, pThis->m_bSmema[WORK1_CONV].NextIn.Status);
                if (OldSmema[WORK1_CONV].PrevIn.Status != pThis->m_bSmema[WORK1_CONV].PrevIn.Status)
                    OldSmema[WORK1_CONV].PrevIn.Status = pThis->m_bSmema[WORK1_CONV].PrevIn.Status;
                if (OldSmema[WORK1_CONV].NextIn.Status != pThis->m_bSmema[WORK1_CONV].NextIn.Status)
                    OldSmema[WORK1_CONV].NextIn.Status = pThis->m_bSmema[WORK1_CONV].NextIn.Status;
            }
            if ((OldSmema[EXIT_CONV].PrevIn.Status != pThis->m_bSmema[EXIT_CONV].PrevIn.Status) || (OldSmema[EXIT_CONV].NextIn.Status != pThis->m_bSmema[EXIT_CONV].NextIn.Status))
            {
                TRACE(_T("[PWR] EXIT  PrevIn(Old:%d New:%d) NextIn(Old:%d New:%d)\n"),
                    OldSmema[EXIT_CONV].PrevIn.Status, pThis->m_bSmema[EXIT_CONV].PrevIn.Status,
                    OldSmema[EXIT_CONV].NextIn.Status, pThis->m_bSmema[EXIT_CONV].NextIn.Status);
                if (OldSmema[EXIT_CONV].PrevIn.Status != pThis->m_bSmema[EXIT_CONV].PrevIn.Status)
                    OldSmema[EXIT_CONV].PrevIn.Status = pThis->m_bSmema[EXIT_CONV].PrevIn.Status;
                if (OldSmema[EXIT_CONV].NextIn.Status != pThis->m_bSmema[EXIT_CONV].NextIn.Status)
                    OldSmema[EXIT_CONV].NextIn.Status = pThis->m_bSmema[EXIT_CONV].NextIn.Status;
            }
        }
        ThreadSleep(THREAD_SMEMA_READTIME);
    };// while (pThis->IsTerminated(THREAD_SMEMA_READTIME) == false);
    //pThis->SetMsgQueueStatus(INITIALZE);
    //SetEvent(pThis->GetThreadTerminate());
    TRACE(_T("[PWR] CSmemaControl(0x%x) Quit\n"), pThis->m_ShowID);
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CSmemaControl::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);
    SetThreadName(_T("thSmema"));
    lpStartAddress = (_beginthreadex_proc_type)StartSmemaControl;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;    
    TRACE(_T("[PWR] CSmemaControl Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CSmemaControl Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

long CSmemaControl::CheckPrevSmema(long Conv)
{
    long status = SMEMA_NOT_READY;
    status = m_bSmema[Conv].PrevIn.Status;
    return status;
}

long CSmemaControl::CheckNextSmema(long Conv)
{
    long status = SMEMA_BUSY;
    status = m_bSmema[Conv].NextIn.Status;
    return status;
}

void CSmemaControl::PrevSmemaOut(long Conv, UBYTE output)
{
	switch (Conv)
	{
	case ENTRY_CONV:
		OutputOne(m_bSmema[ENTRY_CONV].PrevOut.IO, output);
		break;
	case WORK1_CONV:
    case WORK2_CONV:
    case EXIT_CONV:
        m_bSmema[Conv].PrevOut.Status = output;
		break;
	}
}

void CSmemaControl::NextSmemaOut(long Conv, UBYTE output)
{
	switch (Conv)
	{
	case ENTRY_CONV:
    case WORK1_CONV:
    case WORK2_CONV:
        m_bSmema[Conv].NextOut.Status = output;
		break;
	case EXIT_CONV:
        OutputOne(m_bSmema[EXIT_CONV].NextOut.IO, output);
		break;
	}
}

void CSmemaControl::ShowSmemaIO()
{
    for (long ConvNo = 0; ConvNo < MAX_CONV; ++ConvNo)
    {
        if (ConvNo == WORK2_CONV) continue;
        TRACE(_T("[PWR] Conv:%d PrevIn:%d(%d) PrevOut:%d NextIn:%d(%d) NextOut:%d\n"), 
            ConvNo,
            m_bSmema[ConvNo].PrevIn.IO, InputOne(m_bSmema[ConvNo].PrevIn.IO),
            m_bSmema[ConvNo].PrevOut.IO,
            m_bSmema[ConvNo].NextIn.IO, InputOne(m_bSmema[ConvNo].NextIn.IO),
            m_bSmema[ConvNo].NextOut.IO);
    }
}
