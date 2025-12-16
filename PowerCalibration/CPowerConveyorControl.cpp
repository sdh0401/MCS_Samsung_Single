#include "pch.h"
#include "CPowerConveyorControl.h"
#include "CApplicationTime.h"
#include "CThreadException.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "Trace.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "CMachineFile.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "CSmemaControl.h"
#include "CLoadingToWork.h"
#include "CReturnToEntrance.h"
#include "COutToExit.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CPowerConveyorData.h"
//#include "ErrorCode.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

CPowerConveyorControl* gcPowerConveyorControl;
CPowerConveyorControl::CPowerConveyorControl()
{
    m_ProdRunMode = 0;
    m_ConveyorRunMode = 0;
    m_EntryPcbCount = m_WorkPcbCount = m_ExitPcbCount = 0;
    m_EntryLoadingStartTimeGet = m_WorkLoadingElapsed = 0;
    GetId(&m_id);
    SetStep(ConveyorCtrlStep::CONVEYOR_INIT);
    SetProfileLow(10000.0, 300000.0, 300000.0);
    SetProfileMid(30000.0, 900000.0, 900000.0);
    SetProfileHigh(45000.0, 900000.0, 900000.0);
}

void CPowerConveyorControl::SetStep(ConveyorCtrlStep nStep)
{
    m_Step = nStep;
    if (gcPowerLog->IsShowConveyorLog() == true)
    {
        TRACE(_T("[PWR] CPowerConveyorControl::SetStep:%d\n"), nStep);
    }
}

ConveyorCtrlStep CPowerConveyorControl::GetStep()
{
    return m_Step;
}

long CPowerConveyorControl::GetThreadID()
{
    long retID;
    GetId(&m_id);
    retID = (long)m_id;
    return retID;
}

BOOL CPowerConveyorControl::OnTask()
{
    TRACE("[PWR] CPowerConveyorControl::OnTask Thread(0x%04X)\n", m_id);
    return TRUE;
}

BOOL CPowerConveyorControl::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    ConveyorCtrlStep CtrlStep;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CPowerConveyorControl GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        //dont' use cout here, output could be broken up due to threading
        if (gcPowerLog->IsShowConveyorLog() == true)
        {
            TRACE("[PWR] CPowerConveyorControl(0x%04X) Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        }
        CtrlStep = CheckCalibrationStep(strMsg);
        SetStep(CtrlStep);
        if (GetStep() >= ConveyorCtrlStep::CONVEYOR_INIT && GetStep() <= ConveyorCtrlStep::CONVEYOR_END)
        {
            ConveyorAutoControl(CtrlStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        }
        else 
        {
            ConveyorManualControl(CtrlStep, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
        }
        delete msgReceived;
    }
    return TRUE;
}

CPowerConveyorControl::~CPowerConveyorControl()
{
}

ConveyorCtrlStep CPowerConveyorControl::CheckCalibrationStep(CString strHostMsg)
{
    ConveyorCtrlStep retCalStep = ConveyorCtrlStep::CONVEYOR_INIT;
    if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_START)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_PCBINFO)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_PCBINFO;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_START_FREETIME)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_START_FREETIME;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_STOP_FREETIME)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_STOP_FREETIME;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_START_LOADING)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_START_LOADINGTIME;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_END_LOADING)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_END_LOADINGTIME;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_START_LOB)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_START_LOB;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_END_LOB)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_END_LOB;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_RUN)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_RUN;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_END)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_CONVEYOR_STOP)) == 0)
        retCalStep = ConveyorCtrlStep::CONVEYOR_STOP;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_CONVEYOR_START)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_CONVEYOR_START;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_CONVEYOR_PCBINFO)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_CONVEYOR_PCBINFO;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_CONVEYOR_END)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_CONVEYOR_END;
    else if (strHostMsg.CompareNoCase(_T(STRING_MANUAL_CONVEYOR_STOP)) == 0)
        retCalStep = ConveyorCtrlStep::MANUAL_CONVEYOR_STOP;
    return retCalStep;
}

void CPowerConveyorControl::StartConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    SetEntryPcbReady(false);
    SetWorkPcbReady(false);
    gcSmemaControl->InitSmema();
    gcSmemaControl->Run();
    gcEntryConveyor->Run(GetProdRunMode());
    gcWorkConveyor->Run(GetProdRunMode());
    gcExitConveyor->Run(GetProdRunMode());
}

void CPowerConveyorControl::SetPcbInfoConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    if (gcWorkConveyor)
    {
        TRACE(_T("[PWR] SetPcbInfoConveyorCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
        double PcbThickness = (double)(SubMsg1 / 10.0);
        double PcbStandByZOffset = (double)(SubMsg2 / 10.0);
        long PcbSimultaneousLoading = SubMsg3;
        TRACE(_T("[PWR] SetPcbInfoConveyorCtrl PcbThickness,%.3f,PcbStandByZOffset,%.3f,PcbSimultaneousLoading,%d\n"), PcbThickness, PcbStandByZOffset, PcbSimultaneousLoading);
        gcWorkConveyor->SetPcbThickness(PcbThickness);
        gcWorkConveyor->SetPcbStandByZOffset(PcbStandByZOffset);

    }
}

void CPowerConveyorControl::StartFreeTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    long Conv = SubMsg1;
    if (gcPowerLog->IsShowTowerLampLog() == true)
    {
        TRACE(_T("[PWR] StartFreeTime RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    if (Conv == ENTRY_CONV)
    {
        if (gcEntryConveyor)
        {
            gcEntryConveyor->StartGetFreeTime();
        }
    }
    else if (Conv == EXIT_CONV)
    {
        if (gcExitConveyor)
        {
            gcExitConveyor->StartGetFreeTime();
        }
    }
}

void CPowerConveyorControl::StopFreeTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    long Conv = SubMsg1;
    if (gcPowerLog->IsShowTowerLampLog() == true)
    {
        TRACE(_T("[PWR] StopFreeTime RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    if (Conv == ENTRY_CONV)
    {
        if (gcEntryConveyor)
        {
            gcEntryConveyor->StopGetFreeTime();
        }
    }
    else if (Conv == EXIT_CONV)
    {
        if (gcExitConveyor)
        {
            gcExitConveyor->StopGetFreeTime();
        }
    }
}

void CPowerConveyorControl::StartLoadingTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    long Conv = SubMsg1;
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] StartLoadingTime RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    m_EntryLoadingStartTimeGet = _time_get();
}

void CPowerConveyorControl::EndLoadingTime(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] EndLoadingTime RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    m_WorkLoadingElapsed = _time_get();
    if (m_WorkLoadingElapsed < m_EntryLoadingStartTimeGet)
        m_WorkLoadingElapsed = 0;
    else
        m_WorkLoadingElapsed = _time_elapsed(m_EntryLoadingStartTimeGet);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] ####################### EndLoadingTime,%d #######################\n"), m_WorkLoadingElapsed);
    }
}

void CPowerConveyorControl::StartLineOfBalance(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    long Conv = SubMsg1;
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] StartLineOfBalance RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    m_LineOfBalanceTimeGet = _time_get();
}

void CPowerConveyorControl::EndLineOfBalance(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] EndLineOfBalance RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    }
    m_LineOfBalanceElapsed = _time_get();
    if (m_LineOfBalanceElapsed < m_LineOfBalanceTimeGet)
        m_LineOfBalanceElapsed = 0;
    else
        m_LineOfBalanceElapsed = _time_elapsed(m_LineOfBalanceTimeGet);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] ####################### EndLineOfBalance,%d #######################\n"), m_LineOfBalanceElapsed);
    }
}

ULONGLONG CPowerConveyorControl::GetLoadingTime()
{
    return m_WorkLoadingElapsed;
}

ULONGLONG CPowerConveyorControl::GetLineOfBalance()
{
    return m_LineOfBalanceElapsed;
}


void CPowerConveyorControl::RunConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] RunConveyorCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    double NewWidth = (double)(SubMsg2 / 10.0);
    double CurWidth = ReadPosition(GetConvName(FRONT_CONV));
    bool bEntry = false, bWork = false, bExit = false;
    long TimeOut = TIME20000MS, Ms = TIME100MS, Err = NO_ERR;
    double Ratio = 0.7, Inpos = 0.5;
    bEntry = IsExistAll(ENTRY_CONV);
    bWork  = IsExistAll(WORK1_CONV);
    bExit  = IsExistAll(EXIT_CONV);
    // HarkDo Check
    // 남아있는 PCB를 배출하고 Conveyor 폭 조정하게 한다.
    if (bEntry == false && bWork == false && bExit == false)
    {
        if (abs(CurWidth - NewWidth) > 0.1)
        {
			CString strZ = GetPusherZName(FRONT_CONV);
			if (ReadPosition(strZ) < GetPusherByZ(FRONT_CONV) + 10.0)
			{
				Err = StartPosWaitDelayedInposition(strZ, Ratio, TimeOut, GetPusherByZ(FRONT_CONV) + 10.0, Inpos, Ms, true);
			}

			if (Err != NO_ERR)
			{
				Err = SendAlarm(Err, _T("PusherZ Moving is fail"));
				return;
			}

            Err = StartPosWaitDelayedInposition(GetConvName(FRONT_CONV), Ratio, TimeOut, NewWidth, Inpos, Ms, true);
            if (Err == NO_ERR)
            {
                WriteWidth(FRONT_CONV, WORK1_CONV, NewWidth);
                //WritePosition(GetConvName(FRONT_GANTRY), NewWidth);
            }
            else
            {
                double WidthCurrent = ReadPosition(GetConvName(FRONT_CONV));
                WriteWidth(FRONT_CONV, WORK1_CONV, WidthCurrent);
                Err = SendAlarm(Err, _T("Conveyor Width Moving is fail"));
                //WritePosition(GetConvName(FRONT_CONV), WidthCurrent);
            }
        }
        else
        {
            TRACE(_T("[PWR] Conveyor Width Cur:%.3f New:%.3f Moving is Skip\n"), CurWidth, NewWidth);
        }
    }
    else
    {
        TRACE(_T("[PWR] Conveyor Pcb Exist Entry:%d Work:%d Exit:%d\n"), bEntry, bWork, bExit);
		if (abs(CurWidth - NewWidth) > 0.1)
		{
			Err = CONVEYOR_CANNOT_MOVE_EXIST_PCB(FRONT_CONV);
			Err = SendAlarm(Err, _T("Conveyor Width Moving is fail"));

		}
    }
    if (Err == NO_ERR)
    {
        if (gcEntryConveyor)
        {
            gcEntryConveyor->SetStep(ConveyorStep::START);
        }
        if (gcWorkConveyor)
        {
            if (GetProdRunMode() == RUN_REAL)
            {
                if (SubMsg1 == 1) // Continue Run
                {
					if (GetConveyorRunMode() == LOCATION_STAY_WORK)
					{
						gcWorkConveyor->SetStep(ConveyorStep::WAIT_WORK_DONE);
						TRACE(_T("[PWR] ************ RunConveyorCtrl CWorkConveyor Continue Run & LOCATION_STAY_WORK ************\n"));

					}
					else
					{
						gcWorkConveyor->SetStep(ConveyorStep::DOWN_PUSHERZ);
						TRACE(_T("[PWR] ************ RunConveyorCtrl CWorkConveyor Continue Run ************\n"));
					}

                }
                else
                {
                    gcWorkConveyor->SetStep(ConveyorStep::START);
                    TRACE(_T("[PWR] ************ RunConveyorCtrl CWorkConveyor ReStart ************\n"));
                }
            }
            else
            {
                gcWorkConveyor->SetStep(ConveyorStep::START);
                TRACE(_T("[PWR] ************ RunConveyorCtrl CWorkConveyor ReStart(Dry) ************\n"));
            }
        }
        if (gcExitConveyor)
        {
            gcExitConveyor->SetStep(ConveyorStep::START);
        }
    }
}

void CPowerConveyorControl::StopConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    if (gcEntryConveyor)
    {
        gcEntryConveyor->BeltOff(BELT_SPEED_MID);
    }
    TRACE(_T("[PWR] gcEntryConveyor Stop Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    pTime->TimeGet();
    if (gcWorkConveyor)
    {
        gcWorkConveyor->BeltOff(BELT_SPEED_MID);
    }
    TRACE(_T("[PWR] gcWorkConveyor Stop Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    pTime->TimeGet();
    if (gcExitConveyor)
    {
        gcExitConveyor->BeltOff(BELT_SPEED_MID);
    }
    TRACE(_T("[PWR] gcExitConveyor Stop Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerConveyorControl::EndConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    CApplicationTime* pTime = new CApplicationTime();
    if (gcEntryConveyor)
    {
        gcEntryConveyor->ExitThreadLoop();
    }
    TRACE(_T("[PWR] gcEntryConveyor Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    pTime->TimeGet();
    if (gcWorkConveyor)
    {
        gcWorkConveyor->ExitThreadLoop();
    }
    TRACE(_T("[PWR] gcWorkConveyor Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    pTime->TimeGet();
    if (gcExitConveyor)
    {
        gcExitConveyor->ExitThreadLoop();
    }
    TRACE(_T("[PWR] gcExitConveyor Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    pTime->TimeGet();
    if (gcSmemaControl)
    {
        gcSmemaControl->ExitThreadLoop();
        gcSmemaControl->InitSmema();
    }
    TRACE(_T("[PWR] gcSmemaControl Exit Elapsed:%d[ms]\n"), pTime->TimeElapsed());
    delete pTime;
}

void CPowerConveyorControl::StartConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] StartConveyorManualCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    m_EntryPcbCount = m_WorkPcbCount = m_ExitPcbCount = 0;
    // Entrance
    if (IsExistEnt(ENTRY_CONV) == true)
    {
        m_EntryPcbCount++;
    }
    else if (IsExistSet(ENTRY_CONV) == true)
    {
        m_EntryPcbCount++;
    }
    // Work
    if (IsExistSet(WORK1_CONV) == true)
    {
        m_WorkPcbCount++;
    }
    else if (IsExistLow(WORK1_CONV) == true)
    {
        m_WorkPcbCount++;
    }
    else if (IsExistOut(WORK1_CONV) == true)
    {
        m_WorkPcbCount++;
    }
    // Exit
    if (IsExistSet(EXIT_CONV) == true)
    {
        m_ExitPcbCount++;
    }
    TRACE(_T("[PWR] PCB Count Entry:%d Work:%d Exit:%d\n"), m_EntryPcbCount, m_WorkPcbCount, m_ExitPcbCount);
    if (m_EntryPcbCount > 0 && m_WorkPcbCount == 0 && m_ExitPcbCount == 0)
    {
        gcLoadingToWork = new CLoadingToWork();
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount > 0 && m_ExitPcbCount == 0)
    {
        gcReturnToEntrance = new CReturnToEntrance();
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount == 0 && m_ExitPcbCount > 0)
    {
        gcReturnToEntrance = new CReturnToEntrance();
    }
}

void CPowerConveyorControl::SetPcbInfoManualConveyorCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    if (gcWorkConveyor)
    {
        TRACE(_T("[PWR] SetPcbInfoManualConveyorCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
        double PcbThickness = (double)(SubMsg1 / 10.0);
        double PcbStandByZOffset = (double)(SubMsg2 / 10.0);
        TRACE(_T("[PWR] SetPcbInfoManualConveyorCtrl PcbThickness,%.3f,PcbStandByZOffset,%.3f\n"), PcbThickness, PcbStandByZOffset);
        gcWorkConveyor->SetPcbThickness(PcbThickness);
        gcWorkConveyor->SetPcbStandByZOffset(PcbStandByZOffset);
    }
}

void CPowerConveyorControl::RunConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] RunConveyorManualCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    if (m_EntryPcbCount > 0 && m_WorkPcbCount == 0 && m_ExitPcbCount == 0)
    {
        gcLoadingToWork->Run();
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount > 0 && m_ExitPcbCount == 0)
    {
        gcReturnToEntrance->Run(WORK1_CONV);
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount == 0 && m_ExitPcbCount > 0)
    {
        gcReturnToEntrance->Run(EXIT_CONV);
    }
}


void CPowerConveyorControl::StopConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] StopConveyorManualCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    if (m_EntryPcbCount > 0 && m_WorkPcbCount == 0 && m_ExitPcbCount == 0)
    {        
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount > 0 && m_ExitPcbCount == 0)
    {
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount == 0 && m_ExitPcbCount > 0)
    {
    }
}


void CPowerConveyorControl::EndConveyorManualCtrl(unsigned SubMsg1, unsigned SubMsg2, unsigned SubMsg3)
{
    TRACE(_T("[PWR] EndConveyorManualCtrl RecvMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
    if (m_EntryPcbCount > 0 && m_WorkPcbCount == 0 && m_ExitPcbCount == 0)
    {
        gcLoadingToWork->ExitThreadLoop();
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount > 0 && m_ExitPcbCount == 0)
    {
        gcReturnToEntrance->ExitThreadLoop();
    }
    else if (m_EntryPcbCount == 0 && m_WorkPcbCount == 0 && m_ExitPcbCount > 0)
    {
        gcReturnToEntrance->ExitThreadLoop();
    }
    m_EntryPcbCount = m_WorkPcbCount = m_ExitPcbCount = 0;
}


void CPowerConveyorControl::ConveyorAutoControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case ConveyorCtrlStep::CONVEYOR_START:
        StartConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_PCBINFO:
        SetPcbInfoConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_START_FREETIME:
        StartFreeTime(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_STOP_FREETIME:
        StopFreeTime(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_START_LOADINGTIME:
        StartLoadingTime(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_END_LOADINGTIME:
        EndLoadingTime(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_START_LOB:
        StartLineOfBalance(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_END_LOB:
        EndLineOfBalance(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_RUN:
        RunConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_STOP:
        StopConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::CONVEYOR_END:
        EndConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    default:
        break;
    }
}

void CPowerConveyorControl::ConveyorManualControl(ConveyorCtrlStep CalStep, int nSub1, int nSub2, int nSub3)
{
    switch (CalStep)
    {
    case ConveyorCtrlStep::MANUAL_CONVEYOR_START:
        StartConveyorManualCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_CONVEYOR_PCBINFO:
        SetPcbInfoManualConveyorCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_CONVEYOR_RUN:
        RunConveyorManualCtrl(nSub1, nSub2, nSub3);
        break;
    case ConveyorCtrlStep::MANUAL_CONVEYOR_STOP:
        StopConveyorManualCtrl(nSub1, nSub2, nSub3);
    case ConveyorCtrlStep::MANUAL_CONVEYOR_END:
        EndConveyorManualCtrl(nSub1, nSub2, nSub3);
        break;
    default:
        break;
    }
}

void CPowerConveyorControl::SetEntryPcbReady(bool bReady)
{
    if (gcEntryConveyor)
    {
        gcEntryConveyor->SetPcbReady(bReady);
    }
}

void CPowerConveyorControl::SetWorkPcbReady(bool bReady)
{
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetPcbReady(bReady);
    }
}

void CPowerConveyorControl::SetWorkPcbOut(bool bReady)
{
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetPcbOut(bReady);
    }
}

bool CPowerConveyorControl::IsEntryPcbReady(long Conveyor)
{
    bool Ready = false;
    if (gcEntryConveyor)
    {
        Ready = gcEntryConveyor->IsPcbReady();
    }
    return Ready;
}

bool CPowerConveyorControl::IsWorkPcbReady(long Conveyor)
{
    bool Ready = false;
    if (gcWorkConveyor)
    {
        Ready = gcWorkConveyor->IsPcbReady();
    }
    return Ready;
}

bool CPowerConveyorControl::IsWorkPcbOut(long Conveyor)
{
    bool Out = false;
    if (gcWorkConveyor)
    {
        Out = gcWorkConveyor->IsPcbOut();
    }
    return Out;
}

void CPowerConveyorControl::SetInsertDone(long InsertDone)
{
    if(gcPowerConveyorData)
    {
        gcPowerConveyorData->SetInsertDone(FRONT_CONV, InsertDone);
        gcPowerConveyorData->WriteInsertDone(FRONT_CONV);
    }
}

long CPowerConveyorControl::GetInsertDone(long Conveyor)
{
    long InsertDone = 0;
    if (gcPowerConveyorData)
    {
        InsertDone = gcPowerConveyorData->GetInsertDone(Conveyor);
    }
    return InsertDone;
}

void CPowerConveyorControl::SetPcbOutDone(long OutDone)
{
    if (gcPowerConveyorData)
    {
        gcPowerConveyorData->SetPcbOutDone(FRONT_CONV, OutDone);
        gcPowerConveyorData->WritePcbOutDone(FRONT_CONV);
    }
}

long CPowerConveyorControl::GetPcbOutDone(long Conveyor)
{
    long OutDone = 0;
    if (gcPowerConveyorData)
    {
        OutDone = gcPowerConveyorData->GetPcbOutDone(Conveyor);
    }
    return OutDone;
}

bool CPowerConveyorControl::IsStopperUp(long Conveyor)
{
    bool bUp = false;
    if (gcWorkConveyor)
    {
        bUp = gcWorkConveyor->IsStopperUp(WORK1_CONV);
    }
    return bUp;
}

bool CPowerConveyorControl::IsStopperDn(long Conveyor)
{
    bool bDn = false;
    if (gcWorkConveyor)
    {
        bDn = gcWorkConveyor->IsStopperDn(WORK1_CONV);
    }
    return bDn;
}

long CPowerConveyorControl::GetProdRunMode()
{
    return m_ProdRunMode;
}

long CPowerConveyorControl::GetConveyorRunMode()
{
    return m_ConveyorRunMode;
}

void CPowerConveyorControl::SetConveyorRunMode(long ProdRunMode, long ConveyorRunMode)
{
    m_ProdRunMode = ProdRunMode;
    m_ConveyorRunMode = ConveyorRunMode;
    TRACE(_T("[PWR] SetConveyorRunMode ProdRunMode:%d ConveyorRunMode:%d\n"),
        m_ProdRunMode, m_ConveyorRunMode);
}

bool CPowerConveyorControl::IsEntryConveyorEnd()
{
    bool bEnd = false;
    if (gcEntryConveyor)
    {
        bEnd = gcEntryConveyor->IsEnd();
    }
    return bEnd;
}


bool CPowerConveyorControl::IsWorkConveyorEnd()
{
    bool bEnd = false;
    if (gcWorkConveyor)
    {
        bEnd = gcWorkConveyor->IsEnd();
    }
    return bEnd;
}

bool CPowerConveyorControl::IsExitConveyorEnd()
{
    bool bEnd = false;
    if (gcExitConveyor)
    {
        bEnd = gcExitConveyor->IsEnd();
    }
    return bEnd;
}

bool CPowerConveyorControl::IsAllConveyorEnd()
{
    if (IsEntryConveyorEnd() == false)
        return false;
    if (IsWorkConveyorEnd() == false)
        return false;
    if (IsExitConveyorEnd() == false)
        return false;
    return true;
}

bool CPowerConveyorControl::IsAllConveyorBeltStop()
{
	if (gcEntryConveyor)
	{
		if (gcEntryConveyor->GetBeltStopState() == false)
		{
			return false;
		}
	}

	if (gcWorkConveyor)
	{
		if (gcWorkConveyor->GetBeltStopState() == false)
		{
			return false;
		}
	}

	if (gcExitConveyor)
	{
		if (gcExitConveyor->GetBeltStopState() == false)
		{
			return false;
		}
	}
	return true;
}

long CPowerConveyorControl::SetWorkConveyorStopMidDelay(long WorkStopMidDelay)
{
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetMiddleTime(WorkStopMidDelay);
    }
    return NO_ERR;
}

long CPowerConveyorControl::SetWorkConveyorStopLowDelay(long WorkStopLowDelay)
{
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetLowTime(WorkStopLowDelay);
    }
    return NO_ERR;
}

long CPowerConveyorControl::SetProfileLow(double MaxVel, double Acc, double Dec)
{
    m_LowSpd.MaxVel = MaxVel;
    m_LowSpd.Acc = Acc;
    m_LowSpd.Dec = Dec;
    m_LowSpd.JerkRatio = 0.5;
    if (gcEntryConveyor)
    {
        gcEntryConveyor->SetBeltMotorSpeedLow(m_LowSpd);
    }
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetBeltMotorSpeedLow(m_LowSpd);
    }
    if (gcExitConveyor)
    {
        gcExitConveyor->SetBeltMotorSpeedLow(m_LowSpd);
    }
    TRACE(_T("[PWR] SetProfileLow Vel:%.1f Acc:%.1f Dec:%.1f\n"), MaxVel, Acc, Dec);
    return NO_ERR;
}

long CPowerConveyorControl::SetProfileMid(double MaxVel, double Acc, double Dec)
{
    m_MidSpd.MaxVel = MaxVel;
    m_MidSpd.Acc = Acc;
    m_MidSpd.Dec = Dec;
    m_MidSpd.JerkRatio = 0.5;
    if (gcEntryConveyor)
    {
        gcEntryConveyor->SetBeltMotorSpeedMid(m_MidSpd);
    }
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetBeltMotorSpeedMid(m_MidSpd);
    }
    if (gcExitConveyor)
    {
        gcExitConveyor->SetBeltMotorSpeedMid(m_MidSpd);
    }
    TRACE(_T("[PWR] SetProfileMid Vel:%.1f Acc:%.1f Dec:%.1f\n"), MaxVel, Acc, Dec);
    return NO_ERR;
}

long CPowerConveyorControl::SetProfileHigh(double MaxVel, double Acc, double Dec)
{
    m_HighSpd.MaxVel = MaxVel;
    m_HighSpd.Acc = Acc;
    m_HighSpd.Dec = Dec;
    m_HighSpd.JerkRatio = 0.5;
    if (gcEntryConveyor)
    {
        gcEntryConveyor->SetBeltMotorSpeedHigh(m_HighSpd);
    }
    if (gcWorkConveyor)
    {
        gcWorkConveyor->SetBeltMotorSpeedHigh(m_HighSpd);
    }
    if (gcExitConveyor)
    {
        gcExitConveyor->SetBeltMotorSpeedHigh(m_HighSpd);
    }
    TRACE(_T("[PWR] SetProfileHigh Vel:%.1f Acc:%.1f Dec:%.1f\n"), MaxVel, Acc, Dec);
    return NO_ERR;
}

long CPowerConveyorControl::SetConveyorSpeed(long Conv, long PrevInBeltSpd, long NextOutBeltSpd)
{
    if (Conv == ENTRY_CONV)
    {
        gcEntryConveyor->SetConveyorSpeed(PrevInBeltSpd, NextOutBeltSpd);
    }
    else if (Conv == WORK1_CONV)
    {
        gcWorkConveyor->SetConveyorSpeed(PrevInBeltSpd, NextOutBeltSpd);
    }
    else if (Conv == EXIT_CONV)
    {
        gcExitConveyor->SetConveyorSpeed(PrevInBeltSpd, NextOutBeltSpd);
    }
    return NO_ERR;
}

long CPowerConveyorControl::GetConveyorSpeedPrevIn(long Conv)
{
    long PrevIn = BELT_SPEED_MID;
    if (Conv == ENTRY_CONV)
    {
        PrevIn = gcEntryConveyor->GetPrevInBeltSpd();
    }
    else if (Conv == WORK1_CONV)
    {
        PrevIn = gcWorkConveyor->GetPrevInBeltSpd();
    }
    else if (Conv == EXIT_CONV)
    {
        PrevIn = gcExitConveyor->GetPrevInBeltSpd();
    }
    return PrevIn;
}

long CPowerConveyorControl::GetConveyorSpeedNextOut(long Conv)
{
    long NextOut = BELT_SPEED_MID;
    if (Conv == ENTRY_CONV)
    {
        NextOut = gcEntryConveyor->GetNextOutBeltSpd();
    }
    else if (Conv == WORK1_CONV)
    {
        NextOut = gcWorkConveyor->GetNextOutBeltSpd();
    }
    else if (Conv == EXIT_CONV)
    {
        NextOut = gcExitConveyor->GetNextOutBeltSpd();
    }
    return NextOut;
}

long CPowerConveyorControl::SetPcbTransferTimeOut(long ConvTable, long Conv, long LoadingTimeOut, long UnloadingTimeOut)
{
	if (Conv == ENTRY_CONV)
	{
		gcEntryConveyor->SetPrevTimeOut(LoadingTimeOut);
		gcEntryConveyor->SetNextTimeOut(UnloadingTimeOut);
	}
	else if (Conv == WORK1_CONV)
	{
		gcWorkConveyor->SetPrevTimeOut(LoadingTimeOut);
		gcWorkConveyor->SetNextTimeOut(UnloadingTimeOut);
	}
	else if (Conv == EXIT_CONV)
	{
		gcExitConveyor->SetPrevTimeOut(LoadingTimeOut);
		gcExitConveyor->SetNextTimeOut(UnloadingTimeOut);
	}

	return NO_ERR;
}

void CPowerConveyorControl::SetHeightMeasureDone(long done)
{
    if (gcPowerConveyorData)
    {
        gcPowerConveyorData->SetHeightMeasureDone(done);
    }
}

long CPowerConveyorControl::GetHeightMeasureDone()
{
    long done = 0;
    if (gcPowerConveyorData)
    {
        done = gcPowerConveyorData->GetHeightMeasureDone();
    }
    return done;
}