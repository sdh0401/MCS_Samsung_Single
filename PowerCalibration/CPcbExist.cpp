#include "pch.h"
#include "CPcbExist.h"
#include "AxisInformation.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "DefineThreadLoopTime.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CPowerLog.h"

CPcbExist* gcPcbExist;
CPcbExist::CPcbExist()
{
    for (int indx = 0; indx < MAX_CONV; ++indx)
    {
        m_PcbSensor[indx].Entr.Use = false;
        m_PcbSensor[indx].Entr.IO = IO_NOUSE;
        m_PcbSensor[indx].Low.Use = false;
        m_PcbSensor[indx].Low.IO = IO_NOUSE;        
        m_PcbSensor[indx].Set.Use = false;
        m_PcbSensor[indx].Set.IO = IO_NOUSE;
        m_PcbSensor[indx].Out.Use = false;
        m_PcbSensor[indx].Out.IO = IO_NOUSE;
        m_PcbSensor[indx].Exit.Use = false;
        m_PcbSensor[indx].Exit.IO = IO_NOUSE;

        m_PcbExist[indx].Entr = false;
        m_PcbExist[indx].Low = false;
        m_PcbExist[indx].Set = false;
        m_PcbExist[indx].Out = false;
        m_PcbExist[indx].Exit = false;
    }
    m_ShowID = 0;
}

CPcbExist::~CPcbExist()
{
}

UINT CPcbExist::StartPcbExist(LPVOID wParam)
{
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    unsigned indx = 0;
    PCBSensorPos OldPcbExist[MAX_CONV];
    CPcbExist* pThis = reinterpret_cast<CPcbExist*>(wParam);
    ZeroMemory(&OldPcbExist, sizeof(OldPcbExist));
    AddThread(pThis);
    //pThis->SetMsgQueueStatus(THREAD_RUN);
    while (pThis->GetRunning() == true)
    //do
    {
        if (pThis->IsTerminated(THREAD_PCBEXIST_READTIME) == true)
        {
            bLoop = false;
            TRACE(_T("[PWR] CPcbExist(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
        for (indx = 0; indx < MAX_CONV; ++indx)
        {
            pThis->IsExist(indx);
        }
        for (indx = 0; indx < MAX_CONV; ++indx)
        {
            if (pThis->m_PcbSensor[indx].Entr.IO != IO_NOUSE)
            {
                if (OldPcbExist[indx].Entr != pThis->m_PcbExist[indx].Entr)
                {
                    if (gcPowerLog->IsShowPcbSensorLog() == true)
                    {
                        TRACE(_T("[PWR] Conv%d Entr:%s\n"), indx, pThis->m_PcbExist[indx].Entr == true ? _T("Exist") : _T("Empty"));
                    }
                    OldPcbExist[indx].Entr = pThis->m_PcbExist[indx].Entr;
                }
            }
            if (pThis->m_PcbSensor[indx].Low.IO != IO_NOUSE)
            {
                if (OldPcbExist[indx].Low != pThis->m_PcbExist[indx].Low)
                {
                    if (gcPowerLog->IsShowPcbSensorLog() == true)
                    {
                        TRACE(_T("[PWR] Conv%d Low:%s\n"), indx, pThis->m_PcbExist[indx].Low == true ? _T("Exist") : _T("Empty"));
                    }
                    OldPcbExist[indx].Low = pThis->m_PcbExist[indx].Low;
                }
            }
            if (pThis->m_PcbSensor[indx].Set.IO != IO_NOUSE)
            {
                if (OldPcbExist[indx].Set != pThis->m_PcbExist[indx].Set)
                {
                    if (gcPowerLog->IsShowPcbSensorLog() == true)
                    {
                        TRACE(_T("[PWR] Conv%d Set:%s\n"), indx, pThis->m_PcbExist[indx].Set == true ? _T("Exist") : _T("Empty"));
                    }
                    OldPcbExist[indx].Set = pThis->m_PcbExist[indx].Set;
                }
            }
            if (pThis->m_PcbSensor[indx].Out.IO != IO_NOUSE)
            {
                if (OldPcbExist[indx].Out != pThis->m_PcbExist[indx].Out)
                {
                    if (gcPowerLog->IsShowPcbSensorLog() == true)
                    {
                        TRACE(_T("[PWR] Conv%d Out:%s\n"), indx, pThis->m_PcbExist[indx].Out == true ? _T("Exist") : _T("Empty"));
                    }
                    OldPcbExist[indx].Out = pThis->m_PcbExist[indx].Out;
                }
            }
            if (pThis->m_PcbSensor[indx].Exit.IO != IO_NOUSE)
            {
                if (OldPcbExist[indx].Exit != pThis->m_PcbExist[indx].Exit)
                {
                    if (gcPowerLog->IsShowPcbSensorLog() == true)
                    {
                        TRACE(_T("[PWR] Conv%d Out:%s\n"), indx, pThis->m_PcbExist[indx].Exit == true ? _T("Exist") : _T("Empty"));
                    }
                    OldPcbExist[indx].Exit = pThis->m_PcbExist[indx].Exit;
                }
            }
        }
        if (bLoop == false)
        {
            break;
        }
    };
    TRACE(_T("[PWR] CPcbExist(0x%x) Quit\n"), pThis->m_ShowID);
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

void CPcbExist::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);
    SetThreadName(_T("thPcbExist"));
    lpStartAddress = (_beginthreadex_proc_type)StartPcbExist;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;    
    TRACE(_T("[PWR] CPcbExist Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CPcbExist Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

bool CPcbExist::IsExist(long Conv)
{
    if (IsExistEnt(Conv) == true) return true;
    if (IsExistSet(Conv) == true) return true;
    if (IsExistExit(Conv) == true) return true;
    if (IsExistLow(Conv) == true) return true;
    if (IsExistOut(Conv) == true) return true;
    return false;
}

void CPcbExist::SetPCBSensorIO(long Conv, long Entry, long Low, long Exist, long Out, long Exit)
{
    if (Entry != IO_NOUSE)
    {
        m_PcbSensor[Conv].Entr.Use = true;
        m_PcbSensor[Conv].Entr.IO = Entry;
    }
    if (Low != IO_NOUSE)
    {
        m_PcbSensor[Conv].Low.Use = true;
        m_PcbSensor[Conv].Low.IO = Low;
    }
    if (Exist != IO_NOUSE)
    {
        m_PcbSensor[Conv].Set.Use = true;
        m_PcbSensor[Conv].Set.IO = Exist;
    }
    if (Out != IO_NOUSE)
    {
        m_PcbSensor[Conv].Out.Use = true;
        m_PcbSensor[Conv].Out.IO = Out;
    }
    if (Exit != IO_NOUSE)
    {
        m_PcbSensor[Conv].Exit.Use = true;
        m_PcbSensor[Conv].Exit.IO = Exit;
    }
    TRACE(_T("[PWR] CPcbExist(%d) Entr:%d Low:%d Exist:%d Out:%d Exit:%d\n"), Conv,
        m_PcbSensor[Conv].Entr.IO, m_PcbSensor[Conv].Low.IO, m_PcbSensor[Conv].Set.IO,
        m_PcbSensor[Conv].Out.IO, m_PcbSensor[Conv].Exit.IO);
}

bool CPcbExist::IsExistEnt(long Conv)
{
    UBYTE ucStatus = INOFF;
    if (m_PcbSensor[Conv].Entr.IO == IO_NOUSE)
    {
        return false;
    }

	if (InputElapsedTimeOne(m_PcbSensor[Conv].Entr.IO, INON, PCBEXIST_SENSOR_FILTERTIME) == true)
	{
		m_PcbExist[Conv].Entr = true;
		return true;
	}
	else
	{
		m_PcbExist[Conv].Entr = false;
		return false;
	}
}

bool CPcbExist::IsExistSet(long Conv)
{
    UBYTE ucStatus = INOFF;
    long InOffCnt = 0, InOnCnt = 0;
    if (m_PcbSensor[Conv].Set.IO == IO_NOUSE)
    {
        return false;
    }
    
	if (GetWorkExistSkip() == 1 && Conv == WORK1_CONV)
	{
		return true;
	}

	if (Conv == ENTRY_CONV || Conv == EXIT_CONV)
	{
		//if (InputElapsedTimeOne(m_PcbSensor[Conv].Set.IO, INON, PCBEXIST_SENSOR_FILTERTIME) == true)
		if (InputOne(m_PcbSensor[Conv].Set.IO) == INON)
		{
			m_PcbExist[Conv].Set = true;
			return true;
		}
		else
		{
			m_PcbExist[Conv].Set = false;
			return false;
		}
	}
	else
	{
		// Work의 set,low 감도 신뢰성 떨어져서, 없는걸 확실히 보기로.
		if (InputElapsedTimeOne(m_PcbSensor[Conv].Set.IO, INOFF, PCBEXIST_SENSOR_FILTERTIME) == true)
		{
			m_PcbExist[Conv].Set = false;
			return false;
		}
		else
		{
			m_PcbExist[Conv].Set = true;
			return true;
		}
	}
}

bool CPcbExist::GetStateSet(long Conv, long OnOff)
{
	UBYTE ucStatus = INOFF;
	long InOffCnt = 0, InOnCnt = 0;
	if (m_PcbSensor[Conv].Set.IO == IO_NOUSE)
	{
		return false;
	}

	if (OnOff == INON)
	{
		if (InputElapsedTimeOne(m_PcbSensor[Conv].Set.IO, INON, TIME100MS) == true)
		{
			m_PcbExist[Conv].Set = true;
			return true;
		}
		else
		{
			m_PcbExist[Conv].Set = false;
			return false;
		}
	}
	else
	{
		if (InputElapsedTimeOne(m_PcbSensor[Conv].Set.IO, INOFF, TIME100MS) == true)
		{
			m_PcbExist[Conv].Set = false;
			return true;
		}
		else
		{
			m_PcbExist[Conv].Set = true;
			return false;
		}
	}
}


bool CPcbExist::IsExistExit(long Conv)
{
    UBYTE ucStatus = INOFF;
    if (m_PcbSensor[Conv].Exit.IO == IO_NOUSE)
    {
        return false;
    }

	if (InputElapsedTimeOne(m_PcbSensor[Conv].Exit.IO, INON, PCBEXIST_SENSOR_FILTERTIME) == true)
	{
		m_PcbExist[Conv].Exit = true;
		return true;
	}
	else
	{
		m_PcbExist[Conv].Exit = false;
		return false;
	}
}

bool CPcbExist::IsExistLow(long Conv)
{
    UBYTE ucStatus = INOFF;
    if (m_PcbSensor[Conv].Low.IO == IO_NOUSE)
    {
        return false;
    }

	// Work의 set,low 감도 신뢰성 떨어져서, 없는걸 확실히 보기로.
	if (InputElapsedTimeOne(m_PcbSensor[Conv].Low.IO, INOFF, PCBEXIST_SENSOR_FILTERTIME) == true)
	{
		m_PcbExist[Conv].Low = false;
		return false;
	}
	else
	{
		m_PcbExist[Conv].Low = true;
		return true;
	}
}

bool CPcbExist::IsExistOut(long Conv)
{
    UBYTE ucStatus = INOFF;
    if (m_PcbSensor[Conv].Out.IO == IO_NOUSE)
    {
        return false;
    }
	if (InputElapsedTimeOne(m_PcbSensor[Conv].Out.IO, INOFF, PCBEXIST_SENSOR_FILTERTIME) == true)
	{
		m_PcbExist[Conv].Out = false;
		return false;
	}
	else
	{
		m_PcbExist[Conv].Out = true;
		return true;
	}
}

bool CPcbExist::GetStateOut(long Conv, long OnOff)
{
	UBYTE ucStatus = INOFF;
	if (m_PcbSensor[Conv].Out.IO == IO_NOUSE)
	{
		return false;
	}

	if (OnOff == INON)
	{
		if (InputElapsedTimeOne(m_PcbSensor[Conv].Out.IO, INON, TIME100MS) == true)
		{
			m_PcbExist[Conv].Out = true;
			return true;
		}
		else
		{
			m_PcbExist[Conv].Out = false;
			return false;
		}
	}
	else
	{
		if (InputElapsedTimeOne(m_PcbSensor[Conv].Out.IO, INOFF, TIME100MS) == true)
		{
			m_PcbExist[Conv].Out = false;
			return true;
		}
		else
		{
			m_PcbExist[Conv].Out = true;
			return false;
		}
	}
}

void CPcbExist::ShowPCBSensor()
{
    for (long ConvNo = 0; ConvNo < MAX_CONV; ++ConvNo)
    {
        if (ConvNo == WORK2_CONV) continue;
        TRACE(_T("[PWR] Conv:%d Ent:%d(%d) Low:%d(%d) Set:%d(%d) Out:%d(%d)\n"), ConvNo,
            m_PcbSensor[ConvNo].Entr.IO, InputOne(m_PcbSensor[ConvNo].Entr.IO),
            m_PcbSensor[ConvNo].Low.IO, InputOne(m_PcbSensor[ConvNo].Low.IO),
            m_PcbSensor[ConvNo].Set.IO, InputOne(m_PcbSensor[ConvNo].Set.IO),
            m_PcbSensor[ConvNo].Out.IO, InputOne(m_PcbSensor[ConvNo].Out.IO));
    }
}