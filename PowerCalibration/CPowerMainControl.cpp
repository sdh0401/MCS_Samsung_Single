#include "pch.h"
#include "CPowerMainControl.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CPowerMain.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CReadJobFile.h"
#include "CMachineFileDB.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CPowerMainControl* gcPowerMainControl;
CPowerMainControl::CPowerMainControl()
{
    m_Step = MainRunCtrlStep::MAIN_INIT;
    m_ProdRunMode = RUN_REAL;
    m_ConveyorRunMode = LOCATION_STAY_WORK;
    GetId(&m_id);
}

void CPowerMainControl::SetStep(MainRunCtrlStep nStep)
{
    m_Step = nStep;
    TRACE(_T("[PWR] CPowerMainControl::SetStep:%d\n"), nStep);
}

MainRunCtrlStep CPowerMainControl::GetStep()
{
    return m_Step;
}

long CPowerMainControl::GetThreadID()
{
    long retID;
    GetId(&m_id);
    retID = (long)m_id;
    return retID;
}

BOOL CPowerMainControl::OnTask()
{
    TRACE("[PWR] CPowerMainControl::OnTask Thread(0x%04X)\n", m_id);
    return TRUE;
}

BOOL CPowerMainControl::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    MainRunCtrlStep CtrlStep;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CPowerMainControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        //dont' use cout here, output could be broken up due to threading
        TRACE("[PWR] CPowerMainControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        CtrlStep = CheckMainControlStep(strMsg);
        SetStep(CtrlStep);
        MainRunAutoControl(CtrlStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        delete msgReceived;
    }
    return TRUE;
}

CPowerMainControl::~CPowerMainControl()
{
}

MainRunCtrlStep CPowerMainControl::CheckMainControlStep(CString strHostMsg)
{
    MainRunCtrlStep retCalStep = MainRunCtrlStep::MAIN_INIT;
    if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_START)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_PREPARE)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_PREPARE;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_RUN)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_RUN;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_END)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_FEEDER_REFILL)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_FEEDER_REFILL;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_FEEDER_REFILL_DONE)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_FEEDER_REFILL_DONE;
    else if (strHostMsg.CompareNoCase(_T(STRING_MAINCONTROL_MOVE_STANDBY)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_MOVE_STANDBY;
    else if (strHostMsg.CompareNoCase(_T(STRING_MES_OK)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_MES_OK;
    else if (strHostMsg.CompareNoCase(_T(STRING_MES_NG)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_MES_NG;
    else if (strHostMsg.CompareNoCase(_T(STRING_MES_DISCONECT)) == 0)
        retCalStep = MainRunCtrlStep::MAIN_MES_DISCONNECT;
    return retCalStep;
}

void CPowerMainControl::MainRunAutoControl(MainRunCtrlStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case MainRunCtrlStep::MAIN_START:
        StartMainControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_PREPARE:
        PrepareMainControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_RUN:
        RunMainControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_END:
        StopMainControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_FEEDER_REFILL:
        FeederRefillControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_FEEDER_REFILL_DONE:
        FeederRefillDoneControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_MOVE_STANDBY:
        MoveStandByControl(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_MES_OK:
        GetResultMES_OK(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_MES_NG:
        GetResultMES_NG(nSub1, nSub2, nSub3);
        break;
    case MainRunCtrlStep::MAIN_MES_DISCONNECT:
        GetMES_Disconnect(nSub1, nSub2, nSub3);
        break;
    default:
        break;
    }
}

void CPowerMainControl::StartMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    gcPowerMain->SetStep(MainStep::STOP);
    gcPowerMain->Run();
}

void CPowerMainControl::PrepareMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    gcPowerMain->Prepare();
}

void CPowerMainControl::RunMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    if (gcPowerMain)
    {
        PRODUCTION Production = gcReadJobFile->GetProduction();
        PCB Pcb = gcReadJobFile->GetPcb();

		if (GetProdRunMode() != RUN_REAL || IsExistSet(WORK1_CONV) == false || Pcb.UseLastPickReUse != 1 || gcLastPickFront->GetPcbName().CompareNoCase(Pcb.Name) != 0)
		{
            gcLastPickFront->SetAllHeadDataEnable(false);
		}

		gcLastPickFront->SetPcbName(Pcb.Name);

        long InsertDone = 0;
        long PcbOutDone = 0;
        long Conveyor = FRONT_CONV;
        long LastBlockNo = 0, LastInsertNo = 0;
        LastBlockNo = GetRemainFirstBlock(Pcb.MaxBlockCount, Production.TotalInsertCount);
        LastInsertNo = GetRemainFirstNo(Pcb.MaxBlockCount, Production.TotalInsertCount);
        InsertDone = GetInsertDone(Conveyor);
        PcbOutDone = GetPcbOutDone(Conveyor);
        long HMDone = GetHeightMeasureDone();
        gcPowerMain->SetEnd(false);
        gcPowerMain->SetHMFirst(false);

        if (Pcb.UseHeightMeasurement == 0)
        {
            SetHeightMeasureDone(1);
        }

		gcMachineFileDB->LoadInsertOffset4532FromDB();

        if (GetProdRunMode() == RUN_REAL)
        {
            if (IsExistSet(WORK1_CONV) == true)
            {
                TRACE(_T("[PWR] RunMainControl Last Block:%d Point:%d InsertDone(%d) OutDone(%d)\n"), LastBlockNo, LastInsertNo, InsertDone, PcbOutDone);
                if (LastBlockNo == MAXINSERTDONECOUNT || LastInsertNo == MAXINSERTDONECOUNT || InsertDone == 1)
                {
                    gcPowerMain->SetStep(MainStep::PRODUCT_COMPLETE);
                }
                else
                {
                    gcPowerMain->SetStep(MainStep::START);
                }
            }
            else
            {
                SetWorkPcbReady(FRONT_CONV, false); // Run Start시 Work에 PCB가 없을 때
                SetInsertDone(0); // Run Start시 Work에 PCB가 없을 때
                SetPcbOutDone(1);
                ClearInsertEnd(Conveyor); // Run Start시 Work에 PCB가 없을 때
                SetHeightMeasureDone(0);
                gcPowerMain->SetStep(MainStep::START);
                TRACE(_T("[PWR] RunMainControl All Clear, Start New Job\n"), LastBlockNo, LastInsertNo, InsertDone);
            }
        }
        else
        {
            SetWorkPcbReady(FRONT_CONV, false); // Run Start시 Work에 PCB가 없을 때(DRY)
            SetInsertDone(0); // Run Start시 Work에 PCB가 없을 때(DRY)
            SetPcbOutDone(1); // (DRY)
            ClearInsertEnd(Conveyor); // Run Start시 Work에 PCB가 없을 때(DRY)
            SetHeightMeasureDone(0);
            gcPowerMain->SetStep(MainStep::START);
            TRACE(_T("[PWR] RunMainControl All Clear, Start New Job for DRY\n"), LastBlockNo, LastInsertNo, InsertDone);
        }
    }
}

void CPowerMainControl::StopMainControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    ULONGLONG GetTime = 0, Elapsed = 0;
    GetTime = _time_get();
    if (gcPowerMain)
    {
        gcPowerMain->ExitThreadLoop();
    }
    TRACE(_T("[PWR] StopMainControl Exit Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
}

long CPowerMainControl::GetProdRunMode()
{
    return m_ProdRunMode;
}

long CPowerMainControl::GetConveyorRunMode()
{
    return m_ConveyorRunMode;
}

void CPowerMainControl::SetProdRunMode(long ProdRunMode, long ConveyorRunMode)
{
    m_ProdRunMode = ProdRunMode;
    m_ConveyorRunMode = ConveyorRunMode;
}

void CPowerMainControl::FeederRefillControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] FeederRefillControl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    long FeederNo = SubMsg1;
    gcPowerMain->SetRefill(FeederNo, true);
}

void CPowerMainControl::FeederRefillDoneControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] FeederRefillDoneControl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    gcPowerMain->RefillDone();
}

void CPowerMainControl::MoveStandByControl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] MoveStandByControl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    gcPowerMain->MoveStandBy();
}

void CPowerMainControl::GetResultMES_OK(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] GetResultMES_OK RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    gcPowerMain->MES_OK();
}

void CPowerMainControl::GetResultMES_NG(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] GetResultMES_NG RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    gcPowerMain->MES_NG();
}

void CPowerMainControl::GetMES_Disconnect(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] GetResultMES_NG RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    gcPowerMain->MES_Disconnect();
}


