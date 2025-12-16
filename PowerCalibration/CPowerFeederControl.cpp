#include "pch.h"
#include "CPowerFeederControl.h"
#include "CFeeder.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CReadJobFile.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CPowerFeederControl* gcPowerFeederControl;
CPowerFeederControl::CPowerFeederControl()
{
    m_ProdRunMode = 0;
    GetId(&m_id);
    SetStep(FeederControlStep::FEEDER_INIT);

	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
	{
		gcFeeder[FeederNo] = NULL;
	}
}

void CPowerFeederControl::SetStep(FeederControlStep nStep)
{
    m_Step = nStep;
    TRACE(_T("[PWR] CPowerFeederControl::SetStep:%d\n"), nStep);
}

FeederControlStep CPowerFeederControl::GetStep()
{
    return m_Step;
}

long CPowerFeederControl::GetThreadID()
{
    long retID;
    GetId(&m_id);
    retID = (long)m_id;
    return retID;
}

BOOL CPowerFeederControl::OnTask()
{
    TRACE("[PWR] CPowerFeederControl::OnTask Thread(0x%04X)\n", m_id);
    return TRUE;
}

BOOL CPowerFeederControl::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    FeederControlStep CtrlStep;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CPowerFeederControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        //dont' use cout here, output could be broken up due to threading
        TRACE("[PWR] CPowerFeederControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        CtrlStep = CheckFeederControlStep(strMsg);
        SetStep(CtrlStep);
        if (GetStep() >= FeederControlStep::FEEDER_INIT && GetStep() <= FeederControlStep::FEEDER_END)
        {
            FeederAutoControl(CtrlStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        }
        delete msgReceived;
    }
    return TRUE;
}

CPowerFeederControl::~CPowerFeederControl()
{
}

FeederControlStep CPowerFeederControl::CheckFeederControlStep(CString strHostMsg)
{
    FeederControlStep retStep = FeederControlStep::FEEDER_INIT;
    if (strHostMsg.CompareNoCase(_T(STRING_FEEDER_START)) == 0)
        retStep = FeederControlStep::FEEDER_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_FEEDER_INFO)) == 0)
        retStep = FeederControlStep::FEEDER_INFO;
    else if (strHostMsg.CompareNoCase(_T(STRING_FEEDER_RUN)) == 0)
        retStep = FeederControlStep::FEEDER_RUN;
    else if (strHostMsg.CompareNoCase(_T(STRING_FEEDER_END)) == 0)
        retStep = FeederControlStep::FEEDER_END;
    return retStep;
}

void CPowerFeederControl::StartFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] StartFeederCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long Use = 0, ReadyNo = 0, ReleaseNo = 0, ReadyWaitTime = TIME10MS;
    if (SubMsg1 == 0)
    {
        for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
        {
            Use = gcReadJobFile->GetPick(FeederNo + 1).Use;
            if (Use == 0)
            {
                continue;
            }
            ReadyNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIONo;
            ReleaseNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReleaseIONo;
            ReadyWaitTime = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIOWaitTime;
            if (ReadyNo > 0)
            {
				if (gcFeeder[FeederNo])
				{
					delete gcFeeder[FeederNo];
				}

                gcFeeder[FeederNo] = new CFeeder(FeederNo + 1);
                gcFeeder[FeederNo]->SetReadyNo(ReadyNo);
                gcFeeder[FeederNo]->SetReleaseNo(ReleaseNo);
                gcFeeder[FeederNo]->SetProdRunMode(GetProdRunMode());
                gcFeeder[FeederNo]->SetReadyWaitTime(ReadyWaitTime);
                gcFeeder[FeederNo]->Run();
            }
        }
    }
    else
    {
        ReadyNo = SubMsg2;
        ReleaseNo = SubMsg3;
        if (SubMsg1 > 0 && SubMsg1 <= MAXFEEDERNO)
        {
			if (gcFeeder[SubMsg1 - 1])
			{
				delete gcFeeder[SubMsg1 - 1];
			}

            gcFeeder[SubMsg1 - 1] = new CFeeder(SubMsg1);
            gcFeeder[SubMsg1 - 1]->SetReadyNo(ReadyNo);
            gcFeeder[SubMsg1 - 1]->SetReleaseNo(ReleaseNo);
            gcFeeder[SubMsg1 - 1]->SetProdRunMode(GetProdRunMode());
            gcFeeder[SubMsg1 - 1]->Run();
        }
    }
    TRACE(_T("[PWR] StartFeederCtrl Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerFeederControl::SetInfoFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] SetInfoFeederCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    delete pTime;
}

void CPowerFeederControl::RunFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] RunFeederCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long Use = 0, ReadyNo = 0, ReleaseNo = 0;
    if (SubMsg1 == 0)
    {
        for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
        {
            Use = gcReadJobFile->GetPick(FeederNo + 1).Use;
            if (Use == 0)
            {
                continue;
            }
            ReadyNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIONo;
            ReleaseNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReleaseIONo;
            if (ReadyNo > 0)
            {
                gcFeeder[FeederNo]->SetStep(FeederStep::START);
            }
        }
    }
    else
    {
        if (SubMsg1 > 0 && SubMsg1 <= MAXFEEDERNO)
        {
            if (gcFeeder[SubMsg1 - 1])
            {
                gcFeeder[SubMsg1 - 1]->SetStep(FeederStep::START);
            }
        }
    }
    TRACE(_T("[PWR] RunFeederCtrl Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerFeederControl::StopFeederCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    TRACE(_T("[PWR] StopFeederCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long Use = 0, ReadyNo = 0, ReleaseNo = 0;
    if (SubMsg1 == 0)
    {
        for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
        {
            Use = gcReadJobFile->GetPick(FeederNo + 1).Use;
            if (Use == 0)
            {
                continue;
            }
            ReadyNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIONo;
            ReleaseNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReleaseIONo;
            if (ReadyNo > 0)
            {
				if (gcFeeder[FeederNo])
				{
					gcFeeder[FeederNo]->ExitThreadLoop();
				}
            }
        }
    }
    else
    {
        if (SubMsg1 > 0 && SubMsg1 <= MAXFEEDERNO)
        {
            if (gcFeeder[SubMsg1 - 1])
            {
                gcFeeder[SubMsg1 - 1]->ExitThreadLoop();
				TRACE(_T("[PWR] StopFeederCtrl delete:%d\n"), SubMsg1);

            }
        }
    }

    TRACE(_T("[PWR] StopFeederCtrl Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerFeederControl::FeederAutoControl(FeederControlStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case FeederControlStep::FEEDER_START:
        StartFeederCtrl(nSub1, nSub2, nSub3);
        break;
    case FeederControlStep::FEEDER_INFO:
        SetInfoFeederCtrl(nSub1, nSub2, nSub3);
        break;
    case FeederControlStep::FEEDER_RUN:
        RunFeederCtrl(nSub1, nSub2, nSub3);
        break;
    case FeederControlStep::FEEDER_END:
        StopFeederCtrl(nSub1, nSub2, nSub3);
        break;
    default:
        break;
    }
}

long CPowerFeederControl::GetProdRunMode()
{
    return m_ProdRunMode;
}

void CPowerFeederControl::SetProdRunMode(long ProdRunMode)
{
    m_ProdRunMode = ProdRunMode;
}
