#include "pch.h"
#include "CPowerMain.h"
//#include "AxisInformation.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "LockDef.h"
#include "GlobalData.h"
#include "Trace.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"
#include "CReadJobFile.h"
#include "CStep.h"
#include "CMark.h"
#include "CPick.h"
#include "CRecognition.h"
#include "CRecognitionCamera6.h"
#include "CInsert.h"
#include "CDiscard.h"
#include "CMeasureHeight.h"
#include "vision.h"
#include "VisionData.h"
#include "CAutoNozzleChange.h"
#include "CAvoidMotion.h"
#include "CRecognitionRetry.h"
#include "CRecognitionDivide.h"
#include "CBarcode.h"
#include "CBarcodeFile.h"
#include "CVirtualSerialControl.h"
#include "CForming.h"
#include "CRecognitionFormingRetry.h"
#include "CFormingDRBCoil.h"

CPowerMain* gcPowerMain;
CPowerMain::CPowerMain()
{
    m_Gantry = FRONT_GANTRY;
    m_ShowID = 0;
    m_RunStep = MainStep::STOP;
    m_LastBlockNo = m_LastInsertNo = m_RealMaxInsertCount = 0;
    Initial();
}

CPowerMain::~CPowerMain()
{
}

long CPowerMain::GetTable() const
{
    return m_Gantry;
}

MainStep CPowerMain::GetStep()
{
    return m_RunStep;
}

void CPowerMain::SetStep(MainStep RunStep)
{
    m_RunStep = RunStep;
}

void CPowerMain::Initial()
{
}

void CPowerMain::InitPcb()
{
}

void CPowerMain::InitPackage(long index)
{
}

void CPowerMain::InitFeeder(long index)
{
}

void CPowerMain::InitRetryLed(long index)
{
}

void CPowerMain::Prepare()
{
    TRACE(_T("[PWR] CPowerMain Start to Prepare\n"));
    TRACE(_T("[PWR] CPowerMain End to Prepare\n"));
}

void CPowerMain::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
    SetEnd(false);
    SetThreadName(_T("thPowerMain"));
    lpStartAddress = (_beginthreadex_proc_type)StartMain;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] CPowerMain Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] CPowerMain Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

UINT CPowerMain::StartMain(LPVOID wParam)
{
    TRACE(_T("[PWR] CPowerMain Start-1\n"));
    CPowerMain* pThis = reinterpret_cast<CPowerMain*>(wParam);
    bool bLoop = true;
    signed nArg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    CString strHostMsg;
    long TimeChk = 0, Conveyor = FRONT_CONV, Gantry = FRONT_GANTRY;
    pThis->traceMcsVersion();
    ULONGLONG BoardTimeChk = 0, PartTimeChk = 0, PartPerHour = 0;
    ULONGLONG BoardTimeChkWithLoadingTime = 0, PartTimeChkWithLoadingTime = 0;
    long Err = NO_ERR, ProdRunMode = RUN_REAL, FirstPickingTiming = 0, InsertStartNo = 0;
    long PlanQuantity = 0, ProdQuantity = 0, MaxInsertCount = 0, FeederAll = 0, TotalInsertCount = 0;
    bool bFirstMark = true, MsgShow = true, bFirstSuctionCheck = true;
    bool bFirstPick = true, bManualRecog = false;
    bool bFirstEmpty = true, bBuzzerOn = true;
    bool bFirstMachineReferenceMark = true;
    long RecogTableBy1stInsert;
    bool bPCBExistCheckOK = false;
    bool bFirstBarcode = true;
    MainStep OldStep = MainStep::STOP;
    ULONGLONG StepGetTime = 0, StepElapsedTime = 0;
    ULONGLONG ProdGetTime = 0, ProdElapsedTime = 0;
    ULONGLONG EmptyMachineTime = 0, EmptyMachineElapsedTime = 0;
    ULONGLONG MachineRefMarkGetTime = 0, MachineRefMarkElapsedTime = 0, MachineRefMarkCycleTime = 0;
    MachineReferenceMark MachineRefMark;
    MainStep NextStepCycleStop = MainStep::STOP;
	CString stepName;
	CString stepNameOld;
    long MESResult = 0;
    ULONGLONG GetMesResultTime = 0, GetMesResultElapsedTime = 0;
    CString strBarcode, strBarcodeBlock1, strBarcodeBlock2;

	FORMING_COMPONENT Forming;
	ZeroMemory(&Forming, sizeof(Forming));

    TRACE(_T("[PWR] CPowerMain Start-2\n"));
    //gcStep = new CStep();
    gcMark = new CMark(pThis->m_Gantry);
    gcPick = new CPick(pThis->m_Gantry);
    gcRecognition = new CRecognition(pThis->m_Gantry);
    gcRecognitionCamera6 = new CRecognitionCamera6(pThis->m_Gantry);
    gcInsert = new CInsert(pThis->m_Gantry);
    gcDiscard = new CDiscard(pThis->m_Gantry);
    //gcMeasureHeight = new CMeasureHeight();
    gcAvoidMotion = new CAvoidMotion(pThis->m_Gantry);
    gcRecognitionRetry = new CRecognitionRetry(pThis->m_Gantry);
	gcRecognitionDivide = new CRecognitionDivide(pThis->m_Gantry);
    gcBarcode = new CBarcode(pThis->m_Gantry);

	CForming* FormingRun = new CForming(pThis->m_Gantry);
	CFormingDRBCoil* FormingDRBCoilRun = new CFormingDRBCoil(pThis->m_Gantry);
	CRecognitionFormingRetry* RecognitionFormingRetry = new CRecognitionFormingRetry(pThis->m_Gantry);

    TRACE(_T("[PWR] CPowerMain Start-3\n"));
    pThis->SetJobInfo();
    gcMark->SetStandBy(pThis->m_StandBy);
    ProdRunMode = pThis->m_Production.ProdMode;
    PlanQuantity = pThis->m_Production.PlanQuantity;
    ProdQuantity = pThis->m_Production.ProdQuantity;
    TotalInsertCount = pThis->m_Production.TotalInsertCount;
    FirstPickingTiming = pThis->m_Production.FirstPickingTiming;
    MachineRefMark = GetGlobalMachineReferenceMark();
    MachineRefMarkCycleTime = static_cast<ULONGLONG>(MachineRefMark.CycleTime) * ONE_MINUTE * TIME1000MS;
	SetProductionQuantity(ProdQuantity);

    gLedAllOff();
    gLaserAllOff();
    gcPick->SetProdRunMode(ProdRunMode);
    gcRecognition->SetProdRunMode(ProdRunMode);
    gcRecognitionCamera6->SetProdRunMode(ProdRunMode);
	gcRecognitionDivide->SetProdRunMode(ProdRunMode);
    gcInsert->SetProdRunMode(ProdRunMode);
    gcDiscard->SetProdRunMode(ProdRunMode);

    BARCODE Barcode = gcStep->GetBarcode();

    pThis->MES_INIT();

    if (pThis->CheckBlockSkipHMUse() == true)
    {
        FirstPickingTiming = 0;
        TRACE(_T("[PWR] First Picking Timing Change 0 by Force\n"));
    }
    //if (Barcode.Mes.Use == 3)
    //{
    //    FirstPickingTiming = 0;
    //    TRACE(_T("[PWR] First Picking Timing Change 0 by Force(MES)\n"));
    //}

 //   gcMeasureHeight->SetProdRunMode(ProdRunMode);
    InsertStartNo = pThis->GetRemainStartNo();
    HeightMeasurementControl(false);
    long MaxFreeTime = GetGlobalTowerYellowLampTime() * TIME1000MS;
    TRACE(_T("[PWR] CPowerMain Start Step:%d Running:%d StartNo:%d\n"), pThis->GetStep(), pThis->GetRunning(), InsertStartNo);
    StepGetTime = _time_get();
    MachineRefMarkGetTime = _time_get();

    pThis->SetBoardTimeExcute(false);

	pThis->PrepareLastPickData();
	long InsertStartNoLastPick = pThis->GetFirstInsertNoLastPickData();
	TRACE(_T("[PWR] CPowerMain Gantry %d StartNoLastPick :%d\n"), Gantry, InsertStartNoLastPick);

	if (InsertStartNoLastPick > 0)
	{
		InsertStartNo = InsertStartNoLastPick;
	}

    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_MAIN_READTIME) == true)
        {
            TRACE(_T("[PWR] CPowerMain(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }

		stepNameOld = pThis->GetStepName(OldStep);
		stepName = pThis->GetStepName(pThis->GetStep());

        if ((OldStep != pThis->GetStep()) && (MsgShow == true))
        {
            StepElapsedTime = _time_elapsed(StepGetTime);
            TRACE(_T("[PWR] CPowerMain(%d) Step:%s->%s Time:%d[ms]\n"), Gantry, stepNameOld, stepName, StepElapsedTime);
            OldStep = pThis->GetStep();
            StepGetTime = _time_get();
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_MAIN_READTIME);
        //    continue;
        //}
        if (GetGlobalStatusError() == true)
        {
            TRACE(_T("[PWR] CPowerMain(0x%X) GetGlobalStatusError(%d)\n"), pThis->m_ShowID, GetGlobalStatusError());
            Err = STOP_NOW;
            break;
        }
        if (GetMachineState() == STATE_STOPNOW)
        {
            TRACE(_T("[PWR] CPowerMain GetMachineState(%d)\n"), GetMachineState());
            Err = STOP_NOW;
            break;
        }
        switch (pThis->GetStep())
        {
        case MainStep::STOP:
            break;

        case MainStep::START:
            //if (pThis->GetHMFirst() == true)
            //{
            //    pThis->SetHMFirst(false);
            //    //pThis->StartPrevProdHeightMeasure(); // After Customer VOC
            //    pThis->SetStep(MainStep::PRODUCT_COMPLETE);
            //}
            //else
            {
                pThis->SetStep(MainStep::QUANTITY);
            }
            break;

        case MainStep::QUANTITY:
            if (PlanQuantity <= ProdQuantity)
            {
                TRACE(_T("[PWR] Plan(%010d) Production(%10d) Complete\n"), PlanQuantity, ProdQuantity);
                pThis->SetStep(MainStep::END);
            }
            else
            {
                Ratio_XYRZ raioLastPick = pThis->GetMinRatioLastPickData();
                Err = gcDiscard->DiscardAllBeforePicking(raioLastPick);
                if (Err != NO_ERR)
                {
                    SendAlarm(Err, _T("Discard Error"));
                    pThis->SetStep(MainStep::SELFQUIT);
                    break;
                }
                TRACE(_T("[PWR] MoveStandby QUANTITY\n"));

                pThis->MoveStandBy();

                if (pThis->CheckBlockSkipHMUse() == true)
                {
                    if (IsWorkPcbReady(Conveyor) == false)
                    {
                        gcMeasureHeight->MovePrepare();
                    }
                    pThis->SetStep(MainStep::BLOCKSKIP_HEIGHTMEASURE);
                }
                else
                {
                    pThis->SetStep(MainStep::PREPARE);
                }
            }
            break;

        case MainStep::BLOCKSKIP_HEIGHTMEASURE:
            if (IsWorkPcbReady(Conveyor) == true)
            {
                SetPcbOutDone(0);
                Err = gcMeasureHeight->BlockSkipCheck();
                if (Err == NO_ERR)
                {
                    if (gcStep->GetAllBlockSkipHM() == true)
                    {
                        pThis->SetStep(MainStep::PRODUCT_COMPLETE);
                    }
                    else
                    {
                        pThis->SetStep(MainStep::PREPARE);
                    }
                }
                else
                {
                    SendAlarm(Err, _T("Block Skip Check Fail"));
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;

        case MainStep::PREPARE:
            if (ProdRunMode == RUN_REAL)
            {
                Err = CheckAirPressureLow();
                if (Err != NO_ERR)
                {
                    SendAlarm(Err, _T("Air Pressure Low"));
                    pThis->SetStep(MainStep::SELFQUIT);
                    break;
                }
            }
            bPCBExistCheckOK = false;
            pThis->PrepareJob();
            pThis->SetBoardTimeExcute(false);
            pThis->SetStep(MainStep::DISCARD_BEFORE_RUN);
            break;

        case MainStep::DISCARD_BEFORE_RUN:
		{
			Ratio_XYRZ raioLastPick = pThis->GetMinRatioLastPickData();
			Err = gcDiscard->DiscardAllBeforePicking(raioLastPick);
			pThis->SetStep(MainStep::MACHINE_REFERENCE_MARK);
			break;
		}

        case MainStep::MACHINE_REFERENCE_MARK:
            if (MachineRefMark.Use == 1)
            {
				Ratio_XYRZ raioLastPick = pThis->GetMinRatioLastPickData();

                if (bFirstMachineReferenceMark == true)
                {
                    Err = gcMark->MachineReferenceMarkCheckingAutoCompen(MachineRefMark, raioLastPick);
                    MachineRefMarkGetTime = _time_get();
					SendCameraRecognitionOffset(FRONT_GANTRY);
                }
                else
                {
                    if (MachineRefMark.CycleTime == 0)
                    {
                        Err = gcMark->MachineReferenceMarkCheckingAutoCompen(MachineRefMark, raioLastPick);
						SendCameraRecognitionOffset(FRONT_GANTRY);
                    }
                    else
                    {
                        MachineRefMarkElapsedTime = _time_elapsed(MachineRefMarkGetTime);
                        if (MachineRefMarkElapsedTime > MachineRefMarkCycleTime)
                        {
                            Err = gcMark->MachineReferenceMarkCheckingAutoCompen(MachineRefMark, raioLastPick);
                            MachineRefMarkGetTime = _time_get();
							SendCameraRecognitionOffset(FRONT_GANTRY);
                        }
                    }
                }
                bFirstMachineReferenceMark = false;
            }
            pThis->SetStep(MainStep::DROPCHECK_BEFORE_RUN);
            break;

		case MainStep::DROPCHECK_BEFORE_RUN:

			Err = pThis->PartDropCheck(FRONT_STAGE);
			if (Err == NO_ERR)
			{
				Err = pThis->PartDropCheck(REAR_STAGE);
			}

			if (Err == NO_ERR)
			{
				pThis->SetStep(MainStep::STANDBY);
			}
			else
			{
				pThis->SetStep(MainStep::SELFQUIT);
			}
			break;

        case MainStep::STANDBY:
            pThis->MoveStandBy();
            if (Err == NO_ERR)
            {
                pThis->SetStep(MainStep::CONTINUE);
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::CONTINUE:
            pThis->SetStep(MainStep::STEP);
            break;

        case MainStep::STEP:
            pThis->ClearVisionError();
            pThis->ClearEmptyError();
            MaxInsertCount = pThis->MakeStepRun(InsertStartNo);
            pThis->SetStep(MainStep::IS_EMPTY);
            EmptyMachineTime = _time_get();
            GetMesResultTime = _time_get();
            break;

        case MainStep::IS_EMPTY:
            if (MaxInsertCount == 0)
            {
                InsertStartNo = pThis->GetRemainStartNoWithEmpty();
                if (InsertStartNo == MAXINSERTDONECOUNT)
                {
                    pThis->SetStep(MainStep::BOARDDONE);
                }
                else if (InsertStartNo == (TotalInsertCount + 1)) // Complete Empty
                {
                    if (bFirstEmpty == true)
                    {
                        TowerLampEmpty(WAIT_FEEDER_REFILL);
                        pThis->SetStep(MainStep::STANDBY);
                        bFirstEmpty = false;
						SendToChangeMachineState(TimeStatics::REFILL_WAIT_START);

                        TRACE(_T("[PWR] All Feeder Empty TotalInsertCount(%d) StepInsertCount(%d)\n"), InsertStartNo, MaxInsertCount);
                    }
                    EmptyMachineElapsedTime = _time_elapsed(EmptyMachineTime);
                    if (EmptyMachineElapsedTime > TIME1000MS)
                    {
                        TRACE(_T("[PWR] Machine Empty status continue InsertStartNo:%d TotalInsertCount:%d\n"), InsertStartNo, TotalInsertCount + 1);
                        EmptyMachineTime = _time_get();
                    }
                    if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE) // SNT모티브 ODBC 방식
                    {
                        GetMesResultElapsedTime = _time_elapsed(GetMesResultTime);
                        if (GetMesResultElapsedTime > TIME1000MS)
                        {
                            GetMesResultTime = _time_get();
                            TRACE(_T("[PWR] Checking Online from MES ... 1Sec \n"));
                            if (pThis->MES_GET() == DISCONECT_MES)
                            {
                                SendPopupClose();
                                SendAlarm(BARCODE_MES_FORCESTOP, _T("Barcode NG"));
                                pThis->SetStep(MainStep::SELFQUIT);
                            }
                        }
                    }
                }
                else // ReStart
                {
                    pThis->SetStep(MainStep::CONTINUE);
                }
            }
            else
            {
                if (bFirstEmpty == false)
                {
					SendToChangeMachineState(TimeStatics::REFILL_WAIT_END);

                    TRACE(_T("[PWR] MoveStandBy After Refill\n"));
                    pThis->MoveStandBy();
                }

                bFirstEmpty = true;
                pThis->SetStep(MainStep::SET_NOZZLE);
            }
            break;

        case MainStep::SET_NOZZLE:

            Err = gcCAutoNozzleChange->Run();

            if (Err != NO_ERR)
            {
                Err = SendAlarm(Err, _T("ANC Error."));
                pThis->SetStep(MainStep::SELFQUIT);
                break;
            }

            if (FirstPickingTiming > 0)
            {
                //if (bPCBExistCheckOK == true || IsExistSet(WORK1_CONV) == true)
                if (bPCBExistCheckOK == true || IsWorkPcbReady(Conveyor) == true || IsExistSet(WORK1_CONV) == true)
                {
                    bPCBExistCheckOK = true;
                    pThis->SetStep(MainStep::IS_WORK_EXIST);
                }
                else
                {
                    pThis->SetStep(MainStep::IS_ENTRY_PCB_READY_1ST);
                }
            }
            else
            {
                if (bFirstMark == true)
                {
                    pThis->SetStep(MainStep::MOVE_SAFTY_Z_BEFORE_PICKUP);
                }
                else
                {
                    pThis->SetStep(MainStep::IS_WORK_PCB_READY_1ST);
                }
            }
            break;

        case MainStep::IS_WORK_EXIST:
            if (GetInfiniteDryRun() == 1 && ProdRunMode == RUN_DRY)
            {
                pThis->SetStep(MainStep::IS_WORK_PCB_READY_1ST);
            }
            else
            {
                if (GetInsertDone(Conveyor) == 1) // 삽입을 다 했다.
                {
                    pThis->SetStep(MainStep::IS_ENTRY_PCB_READY_1ST);
                }
                else // PCB 가 있고 삽입이 남았다.
                {
                    pThis->SetStep(MainStep::IS_WORK_PCB_READY_1ST);
                }
            }
            break;

        case MainStep::IS_ENTRY_PCB_READY_1ST:
            if (IsEntryPcbReady(Conveyor) == true || FirstPickingTiming == 2)
            {
                StopFreeTimeConveyor(ENTRY_CONV);
                StopFreeTimeConveyor(EXIT_CONV);
                pThis->SetStep(MainStep::PICK);
            }
            break;

        case MainStep::MOVE_SAFTY_Z_BEFORE_PICKUP:
            Err = pThis->MoveSaftyZBeforePickup();
            pThis->SetStep(MainStep::IS_WORK_PCB_READY_1ST);
            break;

        case MainStep::IS_WORK_PCB_READY_1ST:
            if (IsWorkPcbReady(Conveyor) == true)
            {
                StopFreeTimeConveyor(ENTRY_CONV);
                StopFreeTimeConveyor(EXIT_CONV);
                SetPcbOutDone(0);
                if (FirstPickingTiming == 0)
                {
                    pThis->SetStep(MainStep::CHECK_FIDUCIAL_1ST);
                }
                else
                {
                    pThis->SetStep(MainStep::PICK);
                }
            }
            break;

        case MainStep::CHECK_FIDUCIAL_1ST:
            if (bFirstMark == true)
            {
                ProdGetTime = _time_get();
                pThis->SetBoardTimeExcute(true);
                Err = gcMark->FiducialMarkChecking();
                if (Err == NO_ERR)
                {
                    bFirstMark = false;
                }
                else
                {
                    Err = SendAlarm(Err, _T("Mark Recognition Fail"));
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            else
            {
                if (Err == NO_ERR)
                {
                    if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE)
                    {
                        pThis->SetStep(MainStep::CHECK_BARCODE);
                    }
                    else
                    {
                        pThis->SetStep(MainStep::PICK);
                    }
                }
                else
                {
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;

        case MainStep::CHECK_BARCODE:
            if (bFirstBarcode == true)
            {
                Err = gcBarcode->BarcodeChecking();
                if (Err == NO_ERR)
                {
                    bFirstBarcode = false;
                    pThis->SetStep(MainStep::SEND_BARCODE);
                }
                else
                {
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            else
            {
                pThis->SetStep(MainStep::PICK);
            }
            break;

        case MainStep::SEND_BARCODE:
            strBarcode = pThis->GetBarcodeCarrier();
            gSetBarcode(WORK1_CONV, strBarcode);
            if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE) // SNT모티브 ODBC 방식
            {
                MES_SendBarcodeSNTMotive(strBarcode);
                pThis->SetStep(MainStep::GET_BARCODE_RESULT);
                GetMesResultTime = _time_get();
                SendPopupMessage(_T("Waiting result from MES"));
            }
            else if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_DEFAULT) // 단방향(Single) : 파일만 생성
            {
                strBarcodeBlock1 = strBarcodeBlock2 = strBarcode;
                MES_SendBarcode(strBarcode, strBarcodeBlock1, strBarcodeBlock2);
                pThis->SetStep(MainStep::PICK);
            }
            else
            {
                pThis->SetStep(MainStep::PICK);
            }
            break;
            
        case MainStep::GET_BARCODE_RESULT:
            MESResult = pThis->MES_GET();
            if (MESResult == NG_MES)
            {
                SendPopupClose();
                if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE) // SNT모티브 ODBC 방식
                {
                    pThis->SetStep(MainStep::NG_BARCODE);
                }
                else // Please, Define Other Customer Action
                {
                    pThis->SetStep(MainStep::NG_BARCODE);
                }
            }
            else if (MESResult == OK_MES)
            {
                SendPopupClose();
                pThis->MES_INIT();
                pThis->SetStep(MainStep::PICK);
            }
            else // Waiting MES Result
            {
                GetMesResultElapsedTime = _time_elapsed(GetMesResultTime);
                if (GetMesResultElapsedTime > TIME3000MS)
                {
                    GetMesResultElapsedTime = 0;
                    GetMesResultTime = _time_get();
                    TRACE(_T("[PWR] Waiting result from MES ... 3Sec \n"));
                }
                if (pThis->MES_GET() == DISCONECT_MES)
                {
                    SendPopupClose();
                    SendAlarm(BARCODE_MES_FORCESTOP, _T("Barcode NG"));
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;

        case MainStep::OUT_BARCODE:
            pThis->SetStep(MainStep::PRODUCT_COMPLETE);
            break;

        case MainStep::NG_BARCODE:
            SendAlarm(BARCODE_MES_FORCESTOP, _T("Barcode NG"));
            pThis->SetStep(MainStep::SELFQUIT);
            break;

        case MainStep::PICK:
            StopFreeTimeConveyor(ENTRY_CONV);
            StopFreeTimeConveyor(EXIT_CONV);
            TowerLampRun();
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Pick Start\n"));
            }
            Err = gcPick->Run(bFirstPick, false);
            bFirstPick = false;
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Pick End Err:%d\n"), Err);
            }
            if (Err == NO_ERR)
            {
                if(pThis->GetAllEmptyError() == true)
                {
                    pThis->SetStep(MainStep::DISCARD_BEFORE_RECOGNITION);
                }
                else
                {
                    pThis->SetStep(MainStep::RECOGNITION);
                }
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::DISCARD_BEFORE_RECOGNITION:
            Err = gcDiscard->DiscardAllBeforeAlignChecking();
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] DiscardAllBeforeAlignChecking Err:%d\n"), Err);
            }
            if (Err == NO_ERR)
            {
                pThis->SetStep(MainStep::STEP);
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::RECOGNITION:
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Recognition Start\n"));
            }

            RecogTableBy1stInsert = pThis->GetRecogTableBy1stInsertOrder();
            if (RecogTableBy1stInsert == FRONT_STAGE)
            {
                if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_2)
                {
                    Err = gcRecognition->Run(bManualRecog);
                }
                else
                {
                    Err = gcRecognitionCamera6->Run(bManualRecog);
                }
            }
            else if (RecogTableBy1stInsert == REAR_STAGE)
            {
                if (GetCameraCount(REAR_STAGE) == CAMERA_COUNT_2)
                    Err = gcRecognition->Run(bManualRecog);
                else
                    Err = gcRecognitionCamera6->Run(bManualRecog);
            }
            else
            {
                SendAlarm(RECOGTABLE_FAIL, _T("Recognition Table Diffrent"));
                pThis->SetStep(MainStep::SELFQUIT);
            }

			if (IsAccTest() == true)
			{
				if (FirstPickingTiming > 0)
				{
					pThis->SetStep(MainStep::MOVE_SAFTY_Z_AFTER_PICKUP);
				}
				else
				{
					pThis->SetStep(MainStep::INSERT);
				}
				break;
			}

            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Recognition End Err:%d\n"), Err);
            }

            if (Err != NO_ERR)
            {
                SendAlarm(RECOGTABLE_FAIL, _T("RgcRecognition Error"));
                pThis->SetStep(MainStep::SELFQUIT);
            }
            else 
            {
                Err = gcRecognitionRetry->Run(bManualRecog);
            }

			if (Err == NO_ERR)
			{
				if (GetFormingDRBCoilUse() == true)
				{
					Err = FormingDRBCoilRun->Run(1, Forming, FormingType::Forming1st);
					if (Err == NO_ERR)
					{
						Err = FormingDRBCoilRun->Run(1, Forming, FormingType::Forming2nd);
					}
				}			
				else
				{
					Err = FormingRun->Run(1, Forming);
				}

			}
			if (Err == NO_ERR)
			{
				Err = RecognitionFormingRetry->Run(bManualRecog);
			}

			if (Err == NO_ERR)
			{
				Err = gcRecognitionDivide->Run(bManualRecog);
			}

            if (Err != NO_ERR)
            {
                SendAlarm(RECOGTABLE_FAIL, _T("RgcRecognitionRetry Error"));
                pThis->SetStep(MainStep::SELFQUIT);
            }

            if (FirstPickingTiming > 0)
            {
                pThis->SetStep(MainStep::MOVE_SAFTY_Z_AFTER_PICKUP);
            }
            else
            {
                if (Err == NO_ERR)
                {
					pThis->SetStep(MainStep::INSERT);
                }
                else
                {
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;
            
        case MainStep::MOVE_SAFTY_Z_AFTER_PICKUP:
            if (bFirstMark == true)
            {
                Err = pThis->MoveSaftyZAfterPickup();
            }
            if (Err == NO_ERR)
            {
                StartFreeTimeConveyor(ENTRY_CONV);
                StartFreeTimeConveyor(EXIT_CONV);
                if (GetInsertDone(Conveyor) == 1)
                {
                    pThis->SetStep(MainStep::OUT_BEFORE_PCB);
                }
                else
                {
                    pThis->SetStep(MainStep::IS_WORK_PCB_READY_2ND);
                }
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::OUT_BEFORE_PCB:
            if (GetInfiniteDryRun() == 1 && ProdRunMode == RUN_DRY)
            {
                pThis->SetStep(MainStep::IS_WORK_PCB_READY_2ND);
            }
            else
            {
                if (GetPcbOutDone(FRONT_CONV) == 1)
                {
                    pThis->SetStep(MainStep::IS_WORK_PCB_READY_2ND);
                    bFirstMark = true;
                    bFirstBarcode = true;
                }
            }
            break;

        case MainStep::DISCARD_AFTER_RECOGNITION:
            Err = gcDiscard->DiscardAllAfterAlignChecking();
            if (Err == NO_ERR)
            {
                pThis->SetStep(MainStep::STEP);
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::IS_WORK_PCB_READY_2ND:
            if (IsWorkPcbReady(Conveyor) == true)
            {
                SetPcbOutDone(0);
                pThis->SetStep(MainStep::CHECK_FIDUCIAL_2ND);
            }
            break;

        case MainStep::CHECK_FIDUCIAL_2ND:
            if (bFirstMark == true)
            {
                ProdGetTime = _time_get();
                pThis->SetBoardTimeExcute(true);
                Err = gcMark->FiducialMarkChecking();
                if (Err == NO_ERR)
                {
                    bFirstMark = false;
                }
                else
                {
                    Err = SendAlarm(Err, _T("Mark Recognition Fail"));
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            else
            {
                if (Err == NO_ERR)
                {
                    if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE)
                    {
                        pThis->SetStep(MainStep::CHECK_BARCODE_2ND);
                    }
                    else
                    {
                        pThis->SetStep(MainStep::INSERT);
                    }
                }
                else
                {
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;

        case MainStep::CHECK_BARCODE_2ND:
            if (bFirstBarcode == true)
            {
                Err = gcBarcode->BarcodeChecking();
                if (Err == NO_ERR)
                {
                    bFirstBarcode = false;
                    pThis->SetStep(MainStep::SEND_BARCODE_2ND);
                }
                else
                {
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            else
            {
                pThis->SetStep(MainStep::INSERT);
            }
            break;

        case MainStep::SEND_BARCODE_2ND:
            strBarcode = pThis->GetBarcodeCarrier();
            gSetBarcode(WORK1_CONV, strBarcode);
            if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE) // SNT모티브 ODBC 방식
            {
                MES_SendBarcodeSNTMotive(strBarcode);
                pThis->SetStep(MainStep::GET_BARCODE_RESULT_2ND);
                GetMesResultTime = _time_get();
                SendPopupMessage(_T("Waiting result from MES"));
            }
            else if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_DEFAULT) // 단방향(Single) : 파일만 생성
            {
                strBarcodeBlock1 = strBarcodeBlock2 = strBarcode;
                MES_SendBarcode(strBarcode, strBarcodeBlock1, strBarcodeBlock2);
                pThis->SetStep(MainStep::INSERT);
            }
            else
            {
                pThis->SetStep(MainStep::INSERT);
            }
            break;

        case MainStep::GET_BARCODE_RESULT_2ND:
            MESResult = pThis->MES_GET();
            if (MESResult == NG_MES)
            {
                SendPopupClose();
                if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE) // SNT모티브 ODBC 방식
                {
                    pThis->SetStep(MainStep::NG_BARCODE);
                }
                else // Please, Define Other Customer Action
                {
                    pThis->SetStep(MainStep::NG_BARCODE);
                }
            }
            else if (MESResult == OK_MES)
            {
                SendPopupClose();
                pThis->MES_INIT();
                pThis->SetStep(MainStep::INSERT);
            }
            else // Waiting MES Result
            {
                GetMesResultElapsedTime = _time_elapsed(GetMesResultTime);
                if (GetMesResultElapsedTime > TIME3000MS)
                {
                    GetMesResultElapsedTime = 0;
                    GetMesResultTime = _time_get();
                    TRACE(_T("[PWR] Waiting result from MES ... 3Sec \n"));
                }
                if (pThis->MES_GET() == DISCONECT_MES)
                {
                    SendAlarm(BARCODE_MES_FORCESTOP, _T("Barcode NG"));
                    pThis->SetStep(MainStep::SELFQUIT);
                }
            }
            break;

        case MainStep::INSERT:
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Insert Start\n"));
            }
			gcInsert->SetPartDrop(false);
            Err = gcInsert->Run();
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] Insert End Err:%d\n"), Err);
            }
            if (Err == NO_ERR)
            {
                pThis->SetStep(MainStep::DISCARD);
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::DISCARD:
            Err = gcDiscard->DiscardAllAfterInserting();
            if (Err == NO_ERR)
            {
				if (gcStep->GetRecogTableBy1stInsertOrder() == REAR_STAGE && gcInsert->GetPartDrop() == false)
				{
					//gcDiscard->PartDropCheckRear();
					Err = pThis->PartDropCheck(REAR_STAGE);
				}

				if (Err != NO_ERR)
				{
					pThis->SetStep(MainStep::SELFQUIT);

				}
                else if (GetStopMode() == HMI_RUN_CONTROL_STOPCYCLE)
                {
                    NextStepCycleStop = MainStep::STEP;
                    pThis->MoveStandBy();
                    pThis->SetStep(MainStep::CYCLESTOP_WAITOTHER);
                }
                else
                {
                    //if (Barcode.Use == 1 || Barcode.Use == 3) // 20210517 HarkDo Abort 추가
                    //{
                    //    MESResult = pThis->MES_GET();
                    //    if (MESResult == ABORT_EIMES || MESResult == NG_EIMES) // 20210601 HarkDo 중간에 작업자 Cancel을 눌렀다
                    //    {
                    //        pThis->SetStep(MainStep::OUT_BARCODE);
                    //    }
                    //    else
                    //    {
                    //        pThis->SetStep(MainStep::STEP);
                    //    }
                    //}
                    //else
                    if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_SNTMOTIVE)
                    {
                        if (pThis->MES_GET() == DISCONECT_MES)
                        {
                            SendAlarm(BARCODE_MES_FORCESTOP, _T("Barcode NG"));
                            pThis->SetStep(MainStep::SELFQUIT);
                        }
                        else
                        {
                            pThis->SetStep(MainStep::STEP);
                        }
                    }
                    else
                    {
                        pThis->SetStep(MainStep::STEP);
                    }
                }
            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::BOARDDONE:	

			if (pThis->m_RealMaxInsertCount == 0)
			{
				TRACE(_T("[PWR] *******************************************************************************\n"));
				TRACE(_T("[PWR] ********** Insert Pt is None!!!!!"));
				TRACE(_T("[PWR] *******************************************************************************\n"));
				pThis->SetStep(MainStep::SELFQUIT);
				break;
			}

            InsertStartNo = 1;

            //if (pThis->GetRemainStartNo() == MAXINSERTDONECOUNT)
            if (pThis->CheckBlockSkipHMUse() == true && gcStep->GetAllBlockSkipHM())
            {
                pThis->SetStep(MainStep::PRODUCT_COMPLETE);
            }
            else
            {
                ProdQuantity++;
                if (gcReadJobFile->GetPcb().UseBlockType == PCB_MAXTRIX || gcReadJobFile->GetPcb().UseBlockType == PCB_NON_MAXTRIX)
                {
                    ProdQuantity += gcReadJobFile->GetPcb().UseBlockCount - 1;
                }
				SetProductionQuantity(ProdQuantity);
                pThis->SetBoardNo(ProdQuantity);

                //SendBlockProdInfo(ProdQuantity, BlockProdQuantity);

                if (pThis->m_Pcb.UseHeightMeasurement == 1)
                {
                    pThis->SetStep(MainStep::MEASURE_HEIGHT);
                }
                else
                {
                    pThis->SetStep(MainStep::PRODUCT_COMPLETE);
                }
            }
            //else
            //{
            //    pThis->SetStep(MainStep::STEP);
            //}
            break;

        case MainStep::MEASURE_HEIGHT:
            Err = gcMeasureHeight->Run(0);
            if (Err == NO_ERR)
            {
                SetHeightMeasureDone(1);

                pThis->SendHeightMeasureEnd();

                pThis->SetStep(MainStep::PRODUCT_COMPLETE);
            }
            else if (Err == HEIGHT_MEASUREMENT_FAIL)
            {
                SetHeightMeasureDone(1);

                //if (GetNGBufferUse() == true && (Barcode.Use == 1 || Barcode.Use == 3))
                //{
                //    pThis->SendHeightMeasureEnd();
                //    TRACE(_T("[PWR] MEASURE_HEIGHT Err:%d\n"), Err);
                //    pThis->SetStep(MainStep::PRODUCT_COMPLETE);
                //}
                //else
                {
                    pThis->SendHeightMeasureEnd();
                    TRACE(_T("[PWR] MEASURE_HEIGHT Err:%d\n"), Err);
                    pThis->SetStep(MainStep::SELFQUIT);
                }

            }
            else
            {
                pThis->SetStep(MainStep::SELFQUIT);
            }
            break;

        case MainStep::PRODUCT_COMPLETE:

            SetHeightMeasureDone(1);
			SetInsertDone(1); // Main Board Insert End인 경우
			ClearInsertEnd(Conveyor); // Main Board Insert End인 경우

            if (GetStopMode() == HMI_RUN_CONTROL_STOPCYCLE || GetStopMode() == HMI_RUN_CONTROL_STOPBOARD)
            {
                NextStepCycleStop = MainStep::PRODUCT_COMPLETE;
                pThis->SetStep(MainStep::BOARDSTOP_SEND_PAUSE);
            }
            else
            {
                pThis->calculateProductionTime(ProdQuantity);
                pThis->SetStep(MainStep::RELEASE_NOZZLE);
            }

            break;

        case MainStep::RELEASE_NOZZLE:

            Err = gcCAutoNozzleChange->RunPrepare();

            if (Err != NO_ERR)
            {
                Err = SendAlarm(Err, _T("ANC Error."));
                pThis->SetStep(MainStep::SELFQUIT);
                break;
            }

            pThis->SetStep(MainStep::STANDBY_COMPLETE);
            break;

        case MainStep::STANDBY_COMPLETE:
            pThis->MoveStandBy();
            StartFreeTimeConveyor(ENTRY_CONV);
            StartFreeTimeConveyor(EXIT_CONV);

			if (IsAccTest() == true)
			{
				pThis->SetStep(MainStep::QUANTITY);
				bFirstMark = true;
                bFirstBarcode = true;
				break;
			}

            if (GetInfiniteDryRun() == 1 && ProdRunMode == RUN_DRY)
            {
                ThreadSleep(TIME1000MS);
                pThis->SetStep(MainStep::QUANTITY);
                bFirstMark = true;
                bFirstBarcode = true;
            }
            else
            {
				if (pThis->m_Production.BoardLocation == LOCATION_STAY_WORK)
				{
					pThis->SetStep(MainStep::SELFQUIT);
				}
                else if (FirstPickingTiming > 0)
                {
                    pThis->SetStep(MainStep::QUANTITY);
                    bFirstMark = true;
                    bFirstBarcode = true;
                }
                else
                {
                    pThis->SetStep(MainStep::OUT_PCB);
                }
            }
            break;
           
        case MainStep::CYCLESTOP_WAITOTHER:
            CallbackHMI_Pause();
            pThis->SetStep(MainStep::CYCLESTOP_WAITPAUSE);
            break;

        case MainStep::CYCLESTOP_WAITPAUSE:
            if (GetRunModeNoLog() == PAUSE_MODE)
            {
                IsUnlockCycleStop(Gantry, NO_WAIT);
                pThis->SetStep(MainStep::CYCLESTOP_WAITLOCK);

				if (GetAllInertEnd() == true)
				{
					CallbackHMI_BoardStop();
				}
            }
            break;

        case MainStep::CYCLESTOP_WAITLOCK:
            if (CycleStopLock(Gantry) == true)
            {
                pThis->SetStep(NextStepCycleStop);                
            }
            break;

        case MainStep::BOARDSTOP_SEND_PAUSE:
            CallbackHMI_Pause();
            pThis->SetStep(MainStep::BOARDSTOP_WAITPAUSE);            
            break;

        case MainStep::BOARDSTOP_WAITPAUSE:
            if (GetRunModeNoLog() == PAUSE_MODE)
            {
                IsUnlockCycleStop(Gantry, NO_WAIT);
                pThis->SetStep(MainStep::BOARDSTOP_WAITLOCK);
            }
            break;

        case MainStep::BOARDSTOP_WAITLOCK:
            //if (GetRunModeNoLog() != PAUSE_MODE)
            if (CycleStopLock(Gantry) == true)
            {
                pThis->SetStep(MainStep::PRODUCT_COMPLETE);
            }
            break;


        case MainStep::CYCLESTOP:
            break;

        case MainStep::BLOCKSTOP:
            break;

        case MainStep::BOARDSTOP:
            break;

        case MainStep::OUT_PCB:
            if (GetPcbOutDone(FRONT_CONV) == 1)
            {                
                pThis->SetStep(MainStep::QUANTITY);
                bFirstMark = true;
                bFirstBarcode = true;
            }
            break;

        case MainStep::END:
            bLoop = false;
            break;

        case MainStep::SELFQUIT:
            bLoop = false;
            break;
        }
        if (bLoop == false)
        {
            break;
        }
        ThreadSleep(THREAD_MAIN_READTIME);
    };

	gcLastPickFront->ShowAll();

	TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step1\n"), pThis->m_ShowID);
    //if(gcStep)
    //    delete gcStep;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step2\n"), pThis->m_ShowID);
    if (gcMark)
        delete gcMark;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step3\n"), pThis->m_ShowID);
    if (gcPick)
        delete gcPick;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step4\n"), pThis->m_ShowID);
    if (gcRecognition)
        delete gcRecognition;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step5\n"), pThis->m_ShowID);
    if (gcRecognitionCamera6)
        delete gcRecognitionCamera6;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step6\n"), pThis->m_ShowID);
    if (gcInsert)
        delete gcInsert;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step7\n"), pThis->m_ShowID);
    if (gcDiscard)
        delete gcDiscard;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step8\n"), pThis->m_ShowID);
    //if (gcMeasureHeight)
    //    delete gcMeasureHeight;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step9\n"), pThis->m_ShowID);
    if (gcAvoidMotion)
        delete gcAvoidMotion;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit Step10\n"), pThis->m_ShowID);
    if (gcRecognitionRetry)
        delete gcRecognitionRetry;
	if (gcRecognitionDivide)
		delete gcRecognitionDivide;
    if (gcBarcode)
        delete gcBarcode;
	TRACE(_T("[PWR] CPowerMain(%d)(0x%X) Quit Step11\n"), pThis->m_ShowID);
	if (FormingRun)
		delete FormingRun;
	TRACE(_T("[PWR] CPowerMain(%d)(0x%X) Quit Step12\n"), pThis->m_ShowID);
	if (RecognitionFormingRetry)
		delete RecognitionFormingRetry;
    TRACE(_T("[PWR] CPowerMain(0x%X) Quit\n"), pThis->m_ShowID);
    gLedAllOff();
    gLaserAllOff();

	pThis->SetEnd(true);

    return Err;
}    

long CPowerMain::SetJobInfo()
{
    long FeederNo = 0, PickupDelay = 0, BlowDelay = 0, ReleaseDelay = 0;
    long NozzleNo = 0, InsertNo = 0, RunStepNo = 0, Use = 0;
    long PickOrder = 0, InsertOrder = 0, InsertHeadNo = 0, InsertFeederNo = 0;
    long PackageIndex = 0, PackageNo = 0, DiscardMethod = 0, MaxInsertCount = 0;
    long ReadyNo = 0, ReleaseNo = 0, ReadyTimeOut = 0, ReadyWaitDelay = 0, PickRetry = 0;
    long RecogTable = 0, ReadyIOType = 0, AvoidCount = 0, LaserControl = 0, CatchDelay = 0;
    double Angle = 0.0, Height = 0.0, LeadHeight = 0.0, PickupZStandBy = 100.0, MaxComponentHeight = 10.0, InsertZOffset = 0.0;
    long PalletNo = 0, PartEmptyStop = 0;
    double VARecogOffsetHeight = 0.0;
    long Gantry = m_Gantry;
    TwoStepMotion TwoStepPick, TwoStepInsert, TwoStepInsertUp;
    CString PackageName;
    Ratio_XYRZ Ratio;
    Point_XYRZ PickupOffset;
    Point_XYRZ Discard;
    MODULE_LED Led;    
    NOZZLE Nozzle;
    INSERT Insert;
    RETRY_LED RetryLed;
	DIVIDE_INSPECT Divide;
	PARTTORQUELIMIT PartTorque;
    BARCODE Barcode;// , BarcodeBlock1, BarcodeBlock2;
	FORMING_COMPONENT Forming;

    TRACE(_T("[PWR] SetJobInfo Start\n"));
    ORIGIN Origin = gcReadJobFile->GetOrigin();
	long MaxPocket = 1, NowPocket = 1, FeederType = 0;
	long TrayIndex = 0, TrayNo = 0;
	CString TrayName;
	TRAY_INFO Tray;
	PICK_LEVEL_CHECK PickLevelInfo;

	m_Production = gcReadJobFile->GetProduction();
    m_Pcb = gcReadJobFile->GetPcb();
    m_StandBy = gcReadJobFile->GetStandyBy();

    Barcode = gcReadJobFile->GetBarcode();

    //BarcodeBlock1.Use = Barcode.Use;
    //BarcodeBlock1.pt = gcReadJobFile->GetBarcodeBlock1().pt;
    //BarcodeBlock1.Led = gcReadJobFile->GetBarcodeBlock1().Led;
    //BarcodeBlock1.Type = gcReadJobFile->GetBarcodeBlock1().Type;

    //BarcodeBlock2.Use = Barcode.Use;
    //BarcodeBlock2.pt = gcReadJobFile->GetBarcodeBlock2().pt;
    //BarcodeBlock2.Led = gcReadJobFile->GetBarcodeBlock2().Led;
    //BarcodeBlock2.Type = gcReadJobFile->GetBarcodeBlock2().Type;
    TRACE(_T("[PWR] SetJobInfo Step2\n"));

    //m_RealMaxInsertCount = m_Production.UseInsertCount;
    PickupZStandBy = gcReadJobFile->GetPickupZStandBy();
    gcStep->SetMaxInsertCount(m_Production.TotalInsertCount);
    gcStep->SetOrigin(Origin);
    if (PickupZStandBy > gcReadJobFile->GetPickupZStandBy())
    {
        PickupZStandBy = gcReadJobFile->GetPickupZStandBy();
    }
    gcStep->SetPickupZStandBy(PickupZStandBy);
    if (MaxComponentHeight < m_Pcb.MaxComponentHeight)
    {
        MaxComponentHeight = m_Pcb.MaxComponentHeight;
    }
    gcStep->SetMaxComponentHeight(MaxComponentHeight);

    gcStep->SetBarcode(Barcode);
    //gcStep->SetBarcodeInfoBlock1(BarcodeBlock1);
    //gcStep->SetBarcodeInfoBlock2(BarcodeBlock2);

    for (FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
    {
        PickRetry = PickupDelay = 0, BlowDelay = 0, ReleaseDelay = 0, Use = 0, PackageIndex = 0;
        ZeroMemory(&Ratio, sizeof(Ratio));
        ZeroMemory(&PickupOffset, sizeof(PickupOffset));
        ZeroMemory(&Led, sizeof(Led));
        VARecogOffsetHeight = Angle = Height = LeadHeight = 0.0;
        ZeroMemory(&Nozzle, sizeof(Nozzle));
        Use = gcReadJobFile->GetPick(FeederNo + 1).Use;
		FeederType = gcReadJobFile->GetFeeder(FeederNo + 1).Type;

        if (Use == 0)
        {
            gcStep->SetFeederUse(FeederNo + 1, Use);
            continue;
        }
        PackageName = gcReadJobFile->GetFeeder(FeederNo + 1).PackageName;
        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] Fd:%d PackageName:%s Start Find\n"), FeederNo, PackageName);
        }
        for (PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
        {
            if (PackageName.CompareNoCase(gcReadJobFile->GetPackage(PackageNo).Name) == 0)
            {
                PackageIndex = PackageNo;
                if (gcPowerLog->IsShowRunLog() == true)
                {
                    TRACE(_T("[PWR] Fd:%d PackageName:%s Found Index(%d)\n"), FeederNo, PackageName, PackageIndex);
                }
                break;
            }
        }
        if (PackageNo == MAXPACKAGENO)
        {
            TRACE(_T("[PWR] CANNOT find PackageName:%s from feeder No(%d)\n"), PackageName, FeederNo);
            return INVALID_PACKAGENAME(FeederNo + 1);
        }
        PickupDelay = gcReadJobFile->GetPackage(PackageIndex).PickDelay;
        BlowDelay = gcReadJobFile->GetPackage(PackageIndex).BlowDelay;
        ReleaseDelay = gcReadJobFile->GetPackage(PackageIndex).ReleaseDelay;
        Ratio = gcReadJobFile->GetPackage(PackageIndex).Ratio;
        PickRetry = gcReadJobFile->GetPackage(PackageIndex).PickRetry;
        PickupOffset = gcReadJobFile->GetPick(FeederNo + 1).Offset;
        DiscardMethod = gcReadJobFile->GetDiscard(FeederNo + 1).Mode;
        Discard = gcReadJobFile->GetDiscard(FeederNo + 1).pt;
        Led = gcReadJobFile->GetPackage(PackageIndex).Led;
        Height = gcReadJobFile->GetPackage(PackageIndex).Height;
        LeadHeight = gcReadJobFile->GetPackage(PackageIndex).LeadHeight;
        ReadyNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIONo;
        ReleaseNo = gcReadJobFile->GetFeeder(FeederNo + 1).ReleaseIONo;
        ReadyTimeOut = gcReadJobFile->GetFeeder(FeederNo + 1).TimeOut;
        ReadyWaitDelay = gcReadJobFile->GetFeeder(FeederNo + 1).ReadyIOWaitTime;
        TwoStepPick = gcReadJobFile->GetPackage(PackageIndex).TwoStepPick;
        TwoStepInsert = gcReadJobFile->GetPackage(PackageIndex).TwoStepInsert;
		TwoStepInsertUp = gcReadJobFile->GetPackage(PackageIndex).TwoStepInsertUp;
        InsertZOffset = gcReadJobFile->GetPackage(PackageIndex).InsertZOffset;
        Angle = gcReadJobFile->GetPackage(PackageIndex).RecogAngle;
        VARecogOffsetHeight = gcReadJobFile->GetPackage(PackageIndex).RecogOffsetHeight;
        PartEmptyStop = gcReadJobFile->GetPackage(PackageIndex).PartEmptyStop;
        LaserControl = gcReadJobFile->GetPackage(PackageIndex).LaserControl;
        CatchDelay = gcReadJobFile->GetPackage(PackageIndex).CatchDelay;
        RetryLed = gcReadJobFile->GetRetryLed(PackageIndex);

		PickLevelInfo = gcReadJobFile->GetFeeder(FeederNo + 1).PickLevel;
		Divide = gcReadJobFile->GetDivideInspect(FeederNo + 1);

		PartTorque = gcReadJobFile->GetPackage(PackageIndex).PartTorqueLimit;
		Forming = gcReadJobFile->GetForming(FeederNo + 1);

		if (IsTrayTypeFeeder(FeederType) == true || IsLabelTypeFeeder(FeederType) == true)
		{
			if (IsLabelTypeFeeder(FeederType) == true)
			{
				gcStep->SetPickLevelCheck(FeederNo + 1, PickLevelInfo);
			}

			NowPocket = gcReadJobFile->GetFeeder(FeederNo + 1).TrayNowPocket;
			MaxPocket = gcReadJobFile->GetFeeder(FeederNo + 1).TrayMaxPocket;
			TrayName = gcReadJobFile->GetFeeder(FeederNo + 1).TrayName;
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] Fd:%d TrayName:%s Start Find\n"), FeederNo, TrayName);
			}
			for (TrayNo = 0; TrayNo < MAXTRAYNO; ++TrayNo)
			{
				if (TrayName.CompareNoCase(gcReadJobFile->GetTray(TrayNo).Name) == 0)
				{
					TrayIndex = TrayNo;
					if (gcPowerLog->IsShowRunLog() == true)
					{
						TRACE(_T("[PWR] Fd:%d TrayName:%s Found Index(%d)\n"), FeederNo, TrayName, TrayIndex);
					}
					break;
				}
			}
			if (TrayNo == MAXTRAYNO)
			{
				TRACE(_T("[PWR] CANNOT find TrayName:%s from feeder No(%d)\n"), TrayName, FeederNo);
				return INVALID_TRAYNAME(FeederNo + 1);
			}
			Tray = gcReadJobFile->GetTray(TrayNo);
			gcStep->SetTray(FeederNo + 1, Tray);
			gcStep->SetTrayMaxPocket(FeederNo + 1, MaxPocket);
			gcStep->SetTrayNowPocket(FeederNo + 1, NowPocket);

			//TTFRatioY = gcReadJobFile->GetFeeder(FeederNo + 1).TTFRatio.y;
			//TTFRatioZ = gcReadJobFile->GetFeeder(FeederNo + 1).TTFRatio.z;

			//if (FeederType == TYPE_TTF1)
			//{
			//	gcTTF[TTF1]->SetRatioYZ(PalletNo, TTFRatioY, TTFRatioZ);
			//}
			//else if (FeederType == TYPE_TTF2)
			//{
			//	gcTTF[TTF2]->SetRatioYZ(PalletNo, TTFRatioY, TTFRatioZ);
			//}
		}


		gcStep->SetFeederType(FeederNo + 1, FeederType);
        gcStep->SetPickDelayFromFdNo(FeederNo + 1, PickupDelay);
        gcStep->SetBlowDelayFromFdNo(FeederNo + 1, BlowDelay);
        gcStep->SetReleaseDelayFromFdNo(FeederNo + 1, ReleaseDelay);
        gcStep->SetRatioFromFdNo(FeederNo + 1, Ratio);
        gcStep->SetPickRetryFromFdNo(FeederNo + 1, PickRetry);
        gcStep->SetPickOffsetFromFdNo(FeederNo + 1, PickupOffset);
        gcStep->SetLed(FeederNo + 1, Led);
        gcStep->SetVAAngleFromFdNo(FeederNo + 1, Angle);
        gcStep->SetComponentHeightFromFdNo(FeederNo + 1, Height);
        gcStep->SetComponentLeadHeightFromFdNo(FeederNo + 1, LeadHeight);        
        gcStep->SetFeederUse(FeederNo + 1, Use);
        gcStep->SetDiscardMethod(FeederNo + 1, DiscardMethod);
        gcStep->SetDiscardPoint(FeederNo + 1, Discard);
        gcStep->SetReadyNoFromFeederNo(FeederNo + 1, ReadyNo);
        gcStep->SetReleaseNoFromFeederNo(FeederNo + 1, ReleaseNo);
        gcStep->SetReadyTimeOutFromFeederNo(FeederNo + 1, ReadyTimeOut);
        gcStep->SetReadyWaitDelayFromFeederNo(FeederNo + 1, ReadyWaitDelay);
        gcStep->SetTwoStepPick(FeederNo + 1, TwoStepPick);
        gcStep->SetTwoStepInsert(FeederNo + 1, TwoStepInsert);
        gcStep->SetTwoStepInsertUp(FeederNo + 1, TwoStepInsertUp);
        gcStep->SetInsertZOffset(FeederNo + 1, InsertZOffset);
        gcStep->SetVAOffsetHeight(FeederNo + 1, VARecogOffsetHeight);
        gcStep->SetPartEmptyStopByFeederNo(FeederNo + 1, PartEmptyStop);
        gcStep->SetLaserControl(FeederNo + 1, LaserControl);
        gcStep->SetCatchDelay(FeederNo + 1, CatchDelay);
        gcStep->SetRetryLed(FeederNo + 1, RetryLed);
		gcStep->SetDivideInspect(FeederNo + 1, Divide);
		gcStep->SetPartTorqueLimit(FeederNo + 1, PartTorque);
		gcStep->SetForming(FeederNo + 1, Forming);
    }
    for (InsertNo = 0; InsertNo < MAXINSERTNO; ++InsertNo)
    {
        ZeroMemory(&Insert, sizeof(Insert));
        NozzleNo = RunStepNo = Use = PickOrder = InsertOrder = InsertHeadNo = 0, InsertFeederNo = 0;
        Insert = gcReadJobFile->GetInsert(InsertNo + 1);
        RunStepNo = gcReadJobFile->GetInsert(InsertNo + 1).Step;
        Use = gcReadJobFile->GetInsert(InsertNo + 1).Use;
        PickOrder = gcReadJobFile->GetInsert(InsertNo + 1).PickOrder;
        InsertOrder = gcReadJobFile->GetInsert(InsertNo + 1).InsertOrder;
        InsertHeadNo = gcReadJobFile->GetInsert(InsertNo + 1).HeadNo;
        InsertFeederNo = gcReadJobFile->GetInsert(InsertNo + 1).FeederNo;
        NozzleNo = gcReadJobFile->GetInsert(InsertNo + 1).NozzleNo;
        Nozzle = gcReadJobFile->GetNozzle(NozzleNo);
        AvoidCount = gcReadJobFile->GetAvoidMotion(InsertNo + 1).Count;
        if (gcReadJobFile->GetInsert(InsertNo + 1).RecogTable == 0)
        {
            if (InsertFeederNo <= MAXHALFFEEDNO)
            {
                RecogTable = FRONT_STAGE;
            }
            else
            {
                RecogTable = REAR_STAGE;
            }
        }
        else if (gcReadJobFile->GetInsert(InsertNo + 1).RecogTable == FRONT_STAGE + 1) // 1
        {
            RecogTable = FRONT_STAGE;
        }
        else // 2
        {
            RecogTable = REAR_STAGE;
        }

        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] InsertNo:%d HeadNo:%d NozzleNo:%d RecogTable:%d\n"), InsertNo + 1, InsertHeadNo, NozzleNo, RecogTable);
        }

        //gcStep->SetNozzle(InsertFeederNo, Nozzle);
        gcStep->SetUseFromInsertNo(InsertNo + 1, Use);
        gcStep->SetInsertPoint(InsertNo + 1, Insert.pt);
        gcStep->SetRunStepNo(InsertNo + 1, RunStepNo);        
        gcStep->SetPickOrderFromInsertNo(InsertNo + 1, PickOrder);
        gcStep->SetInsertOrderFromInsertNo(InsertNo + 1, InsertOrder); // 수정할 것.
        gcStep->SetHeadNoFromInsertNo(InsertNo + 1, InsertHeadNo);
        gcStep->SetFeederNoFromInsertNo(InsertNo + 1, InsertFeederNo);
        gcStep->SetRecognitionTable(InsertNo + 1, RecogTable);
        gcStep->SetNozzleNoFromInsertNo(InsertNo + 1, NozzleNo);
        gcStep->SetAvoidCount(InsertNo + 1, AvoidCount);
        gcStep->SetBlockNoFromInsertNo(InsertNo + 1, gcReadJobFile->GetInsert(this->GetTable(), InsertNo + 1).BlockNo);

        if (gcStep->GetFeederUse(InsertFeederNo) != 0 && Use != 0 && gcStep->GetANCPrepare(Gantry, InsertHeadNo) == 0)
        {
            gcStep->SetANCPrepare(Gantry, InsertHeadNo, NozzleNo);
        }
    }
    TRACE(_T("[PWR] SetJobInfo End\n"));
    return NO_ERR;
}

long CPowerMain::MakeStepRun(long LastInsertNo)
{
    const PCB pcb = gcReadJobFile->GetPcb();
    long StepNo = 0, insertCurNo = 0;
    long Loop1st = 0, MaxPickOrd = 0, MaxInsertOrd = 0;
    long PickOrd = 0, InsertOrd = 0;
    long HeadNo = 0, FeederNo = 0, Conveyor = FRONT_CONV, Gantry = m_Gantry;
    long InsertDone = 0, Block = 0;
    long UserPickOrd = 0, UserInsertOrd = 0;
    long MinOrd = 99, MaxOrd = 0;
    Ratio_XYRZ MinRatio, FeederRatio;
    User_PickOrder UserPickOrder;
    User_InsertOrder UserInsertOrder;
    //StepNo = gcStep->GetRunStepNo(insertCurNo);
    //for (Loop1st = insertCurNo; Loop1st < insertCurNo + MAXUSEDHEADNO; ++Loop1st)
    //{
    //    if (gcStep->GetUseFromInsertNo(Loop1st) == 0)
    //    {
    //        TRACE(_T("[PWR] MaxPickOrd InsertNo(%d) Use:%d\n"), Loop1st, gcStep->GetUseFromInsertNo(Loop1st));
    //        continue;
    //    }
    //    if (StepNo != gcStep->GetRunStepNo(Loop1st))
    //    {
    //        TRACE(_T("[PWR] MaxPickOrd MakeStepRun CurStep(%d):%d NextStep(%d):%d\n"), 
    //            insertCurNo, StepNo, Loop1st, gcStep->GetRunStepNo(Loop1st));
    //        break;
    //    }
    //    MaxPickOrd++;
    //}
    //TRACE(_T("[PWR] MakeStepRun StepPickCount(%d)\n"), MaxPickOrd);

	CheckFeederAutoRefill();

    RunStepLock();
    insertCurNo = LastInsertNo;
    PickOrd = 0;
    StepNo = gcStep->GetRunStepNo(insertCurNo);
    gcStep->ClearUserPickOrder();
    for (Loop1st = 0; Loop1st < MAXUSEDHEADNO; ++Loop1st)
    {
        HeadNo = 0, FeederNo = 0;
        if (StepNo != gcStep->GetRunStepNo(insertCurNo + Loop1st))
        {
            if (gcPowerLog->IsShowRunLog() == true)
            {
                TRACE(_T("[PWR] MaxPickOrd MakeStepRun CurStep(%d):%d NextStep(%d):%d\n"),
                    insertCurNo, StepNo, insertCurNo + Loop1st, gcStep->GetRunStepNo(insertCurNo + Loop1st));
            }
            break;
        }
        if (gcStep->GetUseFromInsertNo(insertCurNo + Loop1st) == 0)
        {
            continue;
        }
        Block = gcReadJobFile->GetInsert(this->GetTable(), insertCurNo + Loop1st).BlockNo;
        if (pcb.UseBlockType == PCB_MAXTRIX || pcb.UseBlockType == PCB_NON_MAXTRIX)
        {
            if (gcReadJobFile->GetBlockOrigin(Block).Use == 0)
            {
                CString temp; temp.Format(L"trying to skip.. (Loop1st:%d, Block:%d, Use:%d, insertCurNo:%d, StepNo:%d)", Loop1st, Block, gcReadJobFile->GetBlockOrigin(Block).Use, insertCurNo, StepNo);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
        }
        if (GetInsertEnd(Conveyor, Gantry, Block, insertCurNo + Loop1st) == INSERT_END)
        {
            continue;
        }
        HeadNo = gcStep->GetHeadNoFromInsertNo(insertCurNo + Loop1st);
        FeederNo = gcStep->GetFeederNoFromInsertNo(insertCurNo + Loop1st);
        UserPickOrd = gcStep->GetPickOrderFromInsertNo(insertCurNo + Loop1st);
        if (gcStep->GetFeederUse(FeederNo) == 0)
        {
            continue;
        }
        if (gcStep->GetSendEmpty(FeederNo) == true)
        {
            TRACE(_T("[PWR] Feeder%03d Pick Skip cause empty\n"), FeederNo);

            if (gcStep->GetPartEmptyStopFromFeederNo(FeederNo) != 0)
            {
                CString ErrStr;
                ErrStr.Format(_T("Feeder:%d Part Empty Stop"), FeederNo);
                SendAlarm(EMPTY_STOP(FeederNo), ErrStr);
                TRACE(_T("[PWR] %s"), ErrStr);
            }

            continue;
        }
        UserPickOrder.PickOrder = UserPickOrd;
        UserPickOrder.InsertCurNo = insertCurNo + Loop1st;
        gcStep->SetUserPickOrderFromIndex(PickOrd + 1, UserPickOrder);
        //gcStep->SetPickupHeadNo(PickOrd + 1, HeadNo);
        //gcStep->SetFdNoFromPickOrder(PickOrd + 1, FeederNo);
        PickOrd++;
    }
    gcStep->SortUserPickOrder();
    MinRatio.xy = MinRatio.r = MinRatio.z = 1.0;
    for (Loop1st = 0; Loop1st < PickOrd; ++Loop1st)
    {
        UserPickOrder = gcStep->GetUserPickOrderFromIndex(Loop1st + 1);
        HeadNo = gcStep->GetHeadNoFromInsertNo(UserPickOrder.InsertCurNo);
        FeederNo = gcStep->GetFeederNoFromInsertNo(UserPickOrder.InsertCurNo);
        gcStep->SetPickupHeadNo(Loop1st + 1, HeadNo);
        gcStep->SetFdNoFromPickOrder(Loop1st + 1, FeederNo);
        FeederRatio = gcStep->GetRatioFromFdNo(FeederNo);
        if (FeederRatio.xy < MinRatio.xy)
        {
            MinRatio.xy = FeederRatio.xy;
        }
        gcStep->SetPickRatio(Loop1st + 1, FeederRatio);
    }
    gcStep->SetMaxPickOrder(PickOrd);
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] MakeStepRun SetMaxPickOrder(%d)\n"), PickOrd);
    }
    //StepNo = gcStep->GetRunStepNo(insertCurNo);
    //for (Loop1st = insertCurNo; Loop1st < insertCurNo + MAXUSEDHEADNO; ++Loop1st)
    //{
    //    if (gcStep->GetUseFromInsertNo(Loop1st) == 0)
    //    {
    //        TRACE(_T("[PWR] MaxInsertOrd InsertNo(%d) Use:%d\n"), Loop1st, gcStep->GetUseFromInsertNo(Loop1st));
    //        continue;
    //    }
    //    if (StepNo != gcStep->GetRunStepNo(Loop1st))
    //    {
    //        TRACE(_T("[PWR] MaxInsertOrd MakeStepRun CurStep(%d):%d NextStep(%d):%d\n"),
    //            insertCurNo, StepNo, Loop1st, gcStep->GetRunStepNo(Loop1st));
    //        break;
    //    }
    //    MaxInsertOrd++;
    //}
    //TRACE(_T("[PWR] MakeStepRun StepInsertCount(%d)\n"), MaxInsertOrd);    
    InsertOrd = 0;
    StepNo = gcStep->GetRunStepNo(insertCurNo);
    gcStep->ClearUserInsertOrder();
    for (Loop1st = 0; Loop1st < MAXUSEDHEADNO; ++Loop1st)
    {
        if (StepNo != gcStep->GetRunStepNo(insertCurNo + Loop1st))
        {
            TRACE(_T("[PWR] MaxInsertOrd MakeStepRun CurStep(%d):%d NextStep(%d):%d\n"),
                insertCurNo, StepNo, insertCurNo + Loop1st, gcStep->GetRunStepNo(insertCurNo + Loop1st));
            break;
        }
        if (gcStep->GetUseFromInsertNo(insertCurNo + Loop1st) == 0)
        {
            continue;
        }
        Block = gcReadJobFile->GetInsert(this->GetTable(), insertCurNo + Loop1st).BlockNo;
        if (pcb.UseBlockType == PCB_MAXTRIX || pcb.UseBlockType == PCB_NON_MAXTRIX)
        {
            if (gcReadJobFile->GetBlockOrigin(Block).Use == 0)
            {
                CString temp; temp.Format(L"trying to skip.. (Loop1st:%d, Block:%d, Use:%d, insertCurNo:%d, StepNo:%d)", Loop1st, Block, gcReadJobFile->GetBlockOrigin(Block).Use, insertCurNo, StepNo);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
        }
        if (GetInsertEnd(Conveyor, Gantry, Block, insertCurNo + Loop1st) == INSERT_END)
        {
            continue;
        }
        HeadNo = gcStep->GetHeadNoFromInsertNo(insertCurNo + Loop1st);
        FeederNo = gcStep->GetFeederNoFromInsertNo(insertCurNo + Loop1st);
        UserInsertOrd = gcStep->GetInsertOrderFromInsertNo(insertCurNo + Loop1st);
        if (gcStep->GetFeederUse(FeederNo) == 0) continue;
        if (gcStep->GetSendEmpty(FeederNo) == true)
        {
            TRACE(_T("[PWR] Feeder%03d Insert Skip cause empty\n"), FeederNo);
            continue;
        }
        UserInsertOrder.InsertOrder = UserInsertOrd;
        UserInsertOrder.InsertCurNo = insertCurNo + Loop1st;
        gcStep->SetUserInsertOrderFromIndex(InsertOrd + 1, UserInsertOrder);
        //gcStep->SetHeadNoFromInsertOrder(InsertOrd + 1, HeadNo);
        //gcStep->SetFdNoFromInsertOrder(InsertOrd + 1, FeederNo);
        //gcStep->SetInsertNoFromInsertOrder(InsertOrd + 1, insertCurNo + Loop1st);
        InsertOrd++;
    }
    gcStep->SortUserInsertOrder();
    MinRatio.xy = MinRatio.r = MinRatio.z = 1.0;
    gcStep->SetMinRatio(MinRatio);
    for (Loop1st = 0; Loop1st < InsertOrd; ++Loop1st)
    {
        UserInsertOrder = gcStep->GetUserInsertOrderFromIndex(Loop1st + 1);
        HeadNo = gcStep->GetHeadNoFromInsertNo(UserInsertOrder.InsertCurNo);
        FeederNo = gcStep->GetFeederNoFromInsertNo(UserInsertOrder.InsertCurNo);
        gcStep->SetHeadNoFromInsertOrder(Loop1st + 1, HeadNo);
        gcStep->SetFdNoFromInsertOrder(Loop1st + 1, FeederNo);
        gcStep->SetInsertNoFromInsertOrder(Loop1st + 1, UserInsertOrder.InsertCurNo);
        FeederRatio = gcStep->GetRatioFromFdNo(FeederNo);
        if (FeederRatio.xy < MinRatio.xy)
        {
            MinRatio = FeederRatio;
        }
        gcStep->SetInsertRatio(Loop1st + 1, FeederRatio);
    }
    TRACE(_T("[PWR] MakeStepRun Find MinRatio XY:%d%%\n"), (long)(MinRatio.xy * 100));
    gcStep->SetMinRatio(MinRatio);
    gcStep->SetMaxInsertOrder(InsertOrd);
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] MakeStepRun SetMaxInsertOrder(%d)\n"), InsertOrd);
        TRACE(_T("[PWR] MakeStepRun StartInsertNo(%d) End InsertCount(%d)\n"), insertCurNo, InsertOrd);
    }
    RunStepUnLock();
    return InsertOrd;
}

long CPowerMain::CheckFeederAutoRefill()
{
	bool refilled = false;
	CFeeder* pFeeder;
	long startNo, lastNo;


	startNo = 1;
	lastNo = MAXFEEDERNO;

	for (long FeederNo = startNo; FeederNo <= lastNo; ++FeederNo)
	{
		if (gcStep->GetFeederUse(FeederNo) == 0)
		{
			continue;
		}

		if (gcStep->GetReadyNoFromFeederNo(FeederNo) == 0)
		{
			continue;
		}

		long VisionErrorEmpty = gcStep->GetVisionErrorEmpty(FeederNo);
		long PickRetry = gcStep->GetPickRetryFromFdNo(FeederNo);

		pFeeder = GetFeeder(FeederNo);
		if (gcStep->GetSendEmpty(FeederNo) == true && pFeeder != NULL && pFeeder->IsEmpty() == false)
		{
			if (VisionErrorEmpty > PickRetry)
			{
				//TRACE(_T("[PWR] CheckFeederAutoRefill Skip RetryOver Feeder:%d\n"), FeederNo);
			}
			else
			{
				TRACE(_T("[PWR] CheckFeederAutoRefill SetRefill Feeder:%d\n"), FeederNo);

				SetRefill(FeederNo, true);
				refilled = true;
			}
		}
	}

	if (refilled == true)
	{
		RefillDone();
	}

	return 0;
}

long CPowerMain::CopyLastStep()
{
    long StepNo = 0, insertCurNo = 0;
    long Loop1st = 0, Loop2nd = 0, MaxPickOrd = 0, MaxInsertOrd = 0;
    long PickOrd = 0, InsertOrd = 0;
    long HeadNo = 0, FeederNo = 0, Conveyor = FRONT_CONV;
    long InsertDone = 0;
    MaxPickOrd = gcStep->GetMaxPickOrder();
    MaxInsertOrd = gcStep->GetMaxInsertOrder();
    TRACE(_T("[PWR] CopyLasteStep MaxPickOrd(%d) MaxInsertOrd(%d)\n"), MaxPickOrd, MaxInsertOrd);
    return NO_ERR;
}

long CPowerMain::MoveStandBy()
{
    long Gantry = FRONT_GANTRY, Err = NO_ERR, TimeOut = TIME5000MS;
    double TargetZ = m_StandBy.pt.z, TargetR = m_StandBy.pt.r;
    ULONGLONG GetTime = _time_get(), Elapsed = 0;
    Point_XYRZ pt = m_StandBy.pt;
    Point_XY ptXY;
	Ratio_XYRZ Ratio = GetMinRatioLastPickData();
    ptXY.x = pt.x;
    ptXY.y = pt.y;
    GetTime = _time_get();


	if (IsAccTest() == true)
	{
		Err = MoveStandByXY(Gantry, FHCAM, ptXY, 1.0, TimeOut);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("MoveStandByXY"));
			return Err;
		}
		return Err;

	}

    Err = WaitAllZIdle(FRONT_GANTRY, TimeOut);
    if (_time_elapsed(GetTime) > 0)
    {
        TRACE(_T("[PWR] MoveStandBy WaitAllZIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
    }
    if (Err != NO_ERR)
    {
        TRACE(_T("[PWR] MoveStandBy WaitAllZIdle Err:%d\n"), Err);
        return Err;
    }

	double maxHeightLastpick= gcLastPickFront->GetMaxHeight();

	if (maxHeightLastpick > 0.001)
	{
		TRACE(_T("[PWR] MoveStandBy TargetZ Change %.3f -> %.3f\n"), TargetZ, TargetZ - maxHeightLastpick);
		TargetZ = TargetZ - maxHeightLastpick;
	}

    GetTime = _time_get();
    Err = MoveZStandy(Gantry, TargetZ, Ratio.z);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] MoveStandBy Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    if (Err != NO_ERR)
    {
        TRACE(_T("[PWR] MoveStandBy Err:%d\n"), Err);
        return Err;
    }

    GetTime = _time_get();
    Err = MoveRStandy(Gantry, TargetR, Ratio.r);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] MoveRStandy Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    if (Err != NO_ERR)
    {
        TRACE(_T("[PWR] MoveRStandy Err:%d\n"), Err);
        return Err;
    }
    Err = MoveStandByXY(Gantry, FHCAM, ptXY, Ratio.xy, TimeOut);
    if (Err != NO_ERR)
    {
        Err = SendAlarm(Err, _T("MoveStandByXY"));
        return Err;
    }
    GetTime = _time_get();
    Err = WaitRStandBy(Gantry, TargetR);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] WaitRStandBy Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    if (Err != NO_ERR)
    {
        TRACE(_T("[PWR] MoveRStandy Err:%d\n"), Err);
        return Err;
    }
    return Err;
}

long CPowerMain::GetRemainStartNoWithEmpty()
{
    long InsertNo = 0, RemainStartNo = 0, HeadNo = 0, FeederNo = 0, Conveyor = FRONT_CONV;
    long Block = 0, Gantry = m_Gantry;
    //m_LastBlockNo = GetRemainFirstBlock(m_Pcb.MaxBlockCount, m_Production.TotalInsertCount);
    RemainStartNo = GetRemainFirstNo(m_Pcb.MaxBlockCount, m_Production.TotalInsertCount);
    if (gcPowerLog->IsShowInsertEndLog() == true)
    {
        TRACE(_T("[PWR] GetRemainStartNoWithEmpty StartNo:%d TotalInsertCount:%d\n"), RemainStartNo, m_Production.TotalInsertCount);
    }

	CheckFeederAutoRefill();

    for (InsertNo = RemainStartNo; InsertNo < m_Production.TotalInsertCount + 1; ++InsertNo)
    {
        if (gcStep->GetUseFromInsertNo(InsertNo) == 0)
        {
            continue;
        }
        Block = gcReadJobFile->GetInsert(this->GetTable(), InsertNo).BlockNo;
        if (gcReadJobFile->GetPcb().UseBlockType == PCB_MAXTRIX || gcReadJobFile->GetPcb().UseBlockType == PCB_NON_MAXTRIX)
        {
            if (gcReadJobFile->GetBlockOrigin(Block).Use == NO_USE)
            {
                CString temp; temp.Format(L"trying to skip.. (InsertNo:%d, Block:%d, BlockOriginUse:%d)", InsertNo, Block, gcReadJobFile->GetBlockOrigin(Block).Use);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
        }
        if (GetInsertEnd(Conveyor, Gantry, Block, InsertNo) == INSERT_END)
        {
            continue;
        }
        FeederNo = gcStep->GetFeederNoFromInsertNo(InsertNo);
        if (gcStep->GetFeederUse(FeederNo) == 0)
        {
            if (gcPowerLog->IsShowInsertEndLog() == true)
            {
                TRACE(_T("[PWR] GetRemainStartNoWithEmpty Feeder%03d No Use\n"), FeederNo);
            }
            continue;
        }
        if (gcStep->GetSendEmpty(FeederNo) == true)
        {
            if (gcPowerLog->IsShowInsertEndLog() == true)
            {
                TRACE(_T("[PWR] GetRemainStartNoWithEmpty Feeder%03d Empty\n"), FeederNo);
            }
            continue;
        }
        break;
    }
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] GetRemainStartNoWithEmpty FoundNo:%d\n"), InsertNo);
    }
    RemainStartNo = InsertNo;
    return RemainStartNo;
}

long CPowerMain::GetRemainStartNo()
{
    long RemainStartNo = 0;
    ULONGLONG TimeGet = 0, ElapsedTime = 0;
    //TimeGet = _time_get();
    //m_LastBlockNo = GetRemainFirstBlock(m_Pcb.MaxBlockCount, m_Production.TotalInsertCount);
    //ElapsedTime = _time_elapsed(TimeGet);
    //TRACE(_T("[PWR] GetRemainStartNo() GetRemainFirstBlock Elapsed:%d\n"), ElapsedTime);
    TimeGet = _time_get();
    RemainStartNo = GetRemainFirstNo(m_Pcb.MaxBlockCount, m_Production.TotalInsertCount);
    ElapsedTime = _time_elapsed(TimeGet);
    TRACE(_T("[PWR] GetRemainStartNo() GetRemainFirstNo Elapsed:%d\n"), ElapsedTime);
    return RemainStartNo;
}

long CPowerMain::PrepareJob()
{
    long Err = NO_ERR, Conveyor = FRONT_CONV, Gantry = FRONT_GANTRY;
    long MaxBlockNo = m_Pcb.MaxBlockCount;
    long MaxInsertNo = m_Production.TotalInsertCount;
    long FeederNo = 0, NoUseInsertNo = 0, RealUseInsertNo = 0;
    if (MaxBlockNo < 1)
    {
        MaxBlockNo = 1;
    }
    TRACE(_T("[PWR] Prepare Job Start\n"));
    for (long Block = 0; Block <= MaxBlockNo; ++Block)
    {
        for (long Point = 0; Point < MaxInsertNo; ++Point)
        {
            if (gcStep->GetBlockNoFromInsertNo(Point + 1) != Block)
            {
                //temp.Format(L"BlockNo(Point+1(%d))(%d) != Block(%d). skipping..", Point + 1, gcStep->GetBlockNoFromInsertNo(Point + 1), Block); TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
            if (gcStep->GetUseFromInsertNo(Point + 1) == 0)
            {
                TRACE(_T("[PWR] Prepare Job Block(%d) Point(%d) Force Done\n"), Block, Point);
                SetInsertEnd(Conveyor, Gantry, Block, Point + 1);
                NoUseInsertNo++;
                continue;
            }
            if (Block != 0 && gcReadJobFile->GetBlockOrigin(Block).Use == 0)
            {
                //temp.Format(L"skipping.. (Point:%d, Block:%d, Use:%d)", Point, Block, gcReadJobFile->GetBlockOrigin(Block).Use); TRACE_FILE_FUNC_LINE_(CStringA)temp);
                NoUseInsertNo++;
                continue;
            }
            FeederNo = gcStep->GetFeederNoFromInsertNo(Point + 1);
            if (gcStep->GetFeederUse(FeederNo) == 0)
            {
                TRACE(_T("[PWR] Prepare Job Block(%d) Point(%d) FeederNo(%d) Force Done\n"), Block, Point, FeederNo);
                SetInsertEnd(Conveyor, Gantry, Block, Point + 1);
                NoUseInsertNo++;
                continue;
            }
            RealUseInsertNo++;
        }
    }
    m_RealMaxInsertCount = RealUseInsertNo;
    ClearVisionError();
    ClearEmptyError();
    TRACE(_T("[PWR] Prepare Job End NoUseInsertNo:%d RealUseInsertNo:%d\n"), NoUseInsertNo, RealUseInsertNo);
    return Err;
}

void CPowerMain::ClearVisionError()
{
    gcStep->ClearVisionError();
}

void CPowerMain::ClearEmptyError()
{
    gcStep->ClearEmptyError();
}

bool CPowerMain::GetAllVisionError()
{
    bool bAllVisionErr = false;
    bAllVisionErr = gcStep->GetAllVisionError();
    return bAllVisionErr;
}

bool CPowerMain::GetAllEmptyError()
{
    bool bAllEmptyError = false;
    bAllEmptyError = gcStep->GetAllEmptyError();
    return bAllEmptyError;
}

long CPowerMain::SetRefill(long FeederNo, bool bRefill)
{
    if (IsAliveCStep(FRONT_GANTRY) == true)
    {
        gcStep->SetRefill(FeederNo, bRefill);
    }
    return NO_ERR;
}

long CPowerMain::RefillDone()
{
    if (IsAliveCStep(FRONT_GANTRY) == true)
    {
        gcStep->RefillDone();
    }
    return NO_ERR;
}

long CPowerMain::MoveSaftyZBeforePickup()
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}
    //long Err = NO_ERR, Gantry = FRONT_GANTRY;
    //double TargetZ = gcStep->GetPickupZStandBy(), Ratio = 1.0;
    //ULONGLONG GetTime = 0, Elapsed = 0;

    long Err = NO_ERR, Gantry = FRONT_GANTRY;
    double Ratio = GetMinRatioLastPickData().z;
    double TargetZ_Standby = m_StandBy.pt.z;
    double TargetZ_PickupStandby = gcStep->GetPickupZStandBy();
    double TargetZ_MaxComponent = GetInsertByZ(FRONT_GANTRY) - GetMaxComponentHeight();
    double TargetZ;
    ULONGLONG GetTime = 0, Elapsed = 0;

    if (TargetZ_Standby < TargetZ_PickupStandby && TargetZ_Standby < TargetZ_MaxComponent)
    {
        TargetZ = TargetZ_Standby;
        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] MainStep MoveSaftyZBeforePickup TargetZ_Standby:%.3f\n"), TargetZ);
        }        
    }
    else if (TargetZ_MaxComponent < TargetZ_PickupStandby)
    {
        TargetZ = TargetZ_MaxComponent;
        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] MainStep MoveSaftyZBeforePickup TargetZ_MaxComponent:%.3f\n"), TargetZ);
        }        
    }
    else
    {
        TargetZ = TargetZ_PickupStandby;
        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] MainStep MoveSaftyZBeforePickup TargetZ_PickupStandby:%.3f\n"), TargetZ);
        }        
    }

	double maxHeightLastpick = gcLastPickFront->GetMaxHeight();

	if (maxHeightLastpick > 0.001)
	{
		TRACE(_T("[PWR] MoveSaftyZBeforePickup TargetZ Change %.3f -> %.3f\n"), TargetZ, TargetZ - maxHeightLastpick);
		TargetZ = TargetZ - maxHeightLastpick;
	}

    GetTime = _time_get();
    Err = MoveZStandy(Gantry, TargetZ, Ratio);
    if (gcPowerLog->IsShowElapsedLog() == true)
    {
        TRACE(_T("[PWR] MainStep MoveSaftyZBeforePickup Elapsed,%d\n"), _time_elapsed(GetTime));
    }
    return Err;
}

long CPowerMain::GetFdNoFromInsertOrder(long insertOrd)
{
    long FdNo = 0;
    FdNo = gcStep->GetFdNoFromInsertOrder(insertOrd);
    return FdNo;
}

long CPowerMain::GetMaxInsertOrder()
{
    long RetMaxOrder = 0;
    RetMaxOrder = gcStep->GetMaxInsertOrder();
    return RetMaxOrder;
}

long CPowerMain::GetHeadNoFromInsertOrder(long insertOrd)
{
    long HeadNo = 0;
    HeadNo = gcStep->GetHeadNoFromInsertOrder(insertOrd);
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
    }
    return HeadNo;
}

double CPowerMain::GetComponentHeight(long FdNo)
{
    double ComponentHeight = 10.0;
    ComponentHeight = gcStep->GetComponentHeightFromFdNo(FdNo);
    return ComponentHeight;
}

double CPowerMain::GetComponentLeadHeight(long FdNo)
{
    double ComponentLeadHeight = 10.0;
    ComponentLeadHeight = gcStep->GetComponentLeadHeightFromFdNo(FdNo);
    return ComponentLeadHeight;
}

long CPowerMain::GetInsertNoFromInsertOrder(long insertOrd)
{
    long InsertNo = 0;
    InsertNo = gcStep->GetInsertNoFromInsertOrder(insertOrd);
    return InsertNo;
}

double CPowerMain::GetMaxComponentHeight()
{
    double MaxComponentHeight = 10.0;
    MaxComponentHeight = gcStep->GetMaxComponentHeight();
    return MaxComponentHeight;
}

Ratio_XYRZ CPowerMain::GetComponentRatioByFdNo(long FdNo)
{
    Ratio_XYRZ ratio;
    ZeroMemory(&ratio, sizeof(ratio));
    ratio = gcStep->GetRatioFromFdNo(FdNo);
    return ratio;
}

long CPowerMain::MoveZUpAfterPickup(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
    long Err = NO_ERR;
    Err = StartPosWaitInposition(strAxis, Ratio, TimeOut, Pos, Inpos, Time, Wait);
    return Err;
}

long CPowerMain::WaitZUpAfterPickup(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
    long Err = NO_ERR;
    Err = WaitOneInPos(strAxis, CmdPos, Inpos, TimeOut);
    return Err;
}

long CPowerMain::MoveSaftyZAfterPickup()
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}
    long Err = NO_ERR, Gantry = FRONT_GANTRY, TimeOut = TIME5000MS;
    long HeadNo = 0, JobFdNo = 0, NozzleNo = 0, MsZUp = TIME30MS;
    double BodyHeight = 0.0, LeadHeight = 0.0, MaxComponentHeight = 0.0, InposZUp = 0.050;
    double TargetZ = GetStandByZ(Gantry), Ratio = 1.0, SaftyZUp = GetStandByZ(Gantry);
    ULONGLONG GetTime = 0, Elapsed = 0;
    Ratio_XYRZ CompRatio;
    NOZZLE Nozzle;
    CString strZAxis;
    Limit limit;
    CompRatio.xy = CompRatio.r = CompRatio.z = 1.0;
    for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
    {
        HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
        JobFdNo = GetFdNoFromInsertOrder(InsertOrd + 1);
        BodyHeight = GetComponentHeight(JobFdNo);
        LeadHeight = GetComponentLeadHeight(JobFdNo);
        CompRatio = GetComponentRatioByFdNo(JobFdNo);
        MaxComponentHeight = GetMaxComponentHeight();
        NozzleNo = GetGlobalNozzleNo(HeadNo);
        Nozzle = GetGlobalNozzleInformation(NozzleNo);
        SaftyZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
        strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
        limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
        if (SaftyZUp > SAFTY_ZHEIGHT)
        {
            SaftyZUp = GetStandByZ(Gantry);
            TRACE(_T("[PWR] MoveSaftyZAfterPickup Safty,%.3f\n"), SaftyZUp);
        }
        if (limit.minus > SaftyZUp)
        {
            TRACE(_T("[PWR] MoveSaftyZAfterPickup Target position(%.3f) is under minus limit(%.3f)\n"), SaftyZUp, limit.minus);
            SaftyZUp = limit.minus + 1.0;
        }
        if (gcPowerLog->IsShowRunLog() == true)
        {
            TRACE(_T("[PWR] MoveSaftyZAfterPickup:%.3f MaxCompoT:%.3f TipHeight:%.3f Body:%.3f Lead:%.3f\n"),
                SaftyZUp, MaxComponentHeight, Nozzle.TipHeight, BodyHeight, LeadHeight);
        }
        Err = MoveZUpAfterPickup(strZAxis, CompRatio.z, TimeOut, SaftyZUp, InposZUp, MsZUp, false);
        if (Err != NO_ERR)
        {
            Err = SendAlarm(Err, _T("MoveSaftyZAfterPickup Err"));
            TRACE(_T("[PWR] MoveSaftyZAfterPickup(%s) Err:%d\n"), strZAxis, Err);
            return Err;
        }
    }
    for (long InsertOrd = 0; InsertOrd < GetMaxInsertOrder(); ++InsertOrd)
    {
        HeadNo = GetHeadNoFromInsertOrder(InsertOrd + 1);
        JobFdNo = GetFdNoFromInsertOrder(InsertOrd + 1);
        BodyHeight = GetComponentHeight(JobFdNo);
        LeadHeight = GetComponentLeadHeight(JobFdNo);
        CompRatio = GetComponentRatioByFdNo(JobFdNo);
        MaxComponentHeight = GetMaxComponentHeight();
        NozzleNo = GetGlobalNozzleNo(HeadNo);
        Nozzle = GetGlobalNozzleInformation(NozzleNo);
        SaftyZUp = GetInsertByZ(Gantry) - MaxComponentHeight - Nozzle.TipHeight + Nozzle.PusherHeight - BodyHeight - LeadHeight;
        strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo);
        limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
        if (SaftyZUp > SAFTY_ZHEIGHT)
        {
            SaftyZUp = GetStandByZ(Gantry);
            TRACE(_T("[PWR] MoveSaftyZAfterPickup Safty,%.3f\n"), SaftyZUp);
        }
        if (limit.minus > SaftyZUp)
        {
            TRACE(_T("[PWR] MoveSaftyZAfterPickup Target position(%.3f) is under minus limit(%.3f)\n"), SaftyZUp, limit.minus);
            SaftyZUp = limit.minus + 1.0;
        }
        Err = WaitZUpAfterPickup(strZAxis, SaftyZUp, InposZUp, TimeOut);
        if (Err != NO_ERR)
        {
            Err = SendAlarm(Err, _T("WaitZUpAfterPickup Err"));
            TRACE(_T("[PWR] WaitZUpAfterPickup(%s) Err:%d\n"), strZAxis, Err);
            return Err;
        }
    }
    return Err;
}

ULONGLONG CPowerMain::GetConveyorLoadingTime()
{
    ULONGLONG LoadingTime = 0;
    if (m_Production.ProdMode == RUN_REAL)
    {
        LoadingTime = GetLoadingTime();
    }
    return LoadingTime;
}

ULONGLONG CPowerMain::GetConveyorLineOfBalance()
{
    ULONGLONG LineOfBalance = 0;
    if (m_Production.ProdMode == RUN_REAL)
    {
        LineOfBalance = GetLineOfBalance();
    }
    return LineOfBalance;
}

long CPowerMain::GetRecogTableBy1stInsertOrder()
{
    long ret = gcStep->GetRecogTableBy1stInsertOrder();

    return ret;
}

long CPowerMain::PartDropCheck(long CamTable)
{
	long Gantry = FRONT_GANTRY;
	long Err = NO_ERR;
	double ratio = 1.0;
	long Stage = CamTable;

	if (gcReadJobFile->GetPartDrop().Use == false) return NO_ERR;

	MoveStandBy();
	TRACE(_T("[PWR] PartDropCheck start\n"));

	gPartDropLedOn(Stage);
	STANDBY standbyPt = gcReadJobFile->GetStandyBy();

	Err = MoveZStandy(Gantry, standbyPt.pt.z, ratio);
	if (Err != NO_ERR)
	{
		return Err;
	}

	Point_XY ptXY;
	ptXY.x = standbyPt.pt.x;
	ptXY.y = standbyPt.pt.y;

	Err = MoveStandByXY(Gantry, FHCAM, ptXY, ratio, TIME5000MS);
	if (Err != NO_ERR)
	{
		return Err;
	}

	gPartDropProcess(Stage);

	Err = gPartDropGetResult(Stage);
	if (Err != NO_ERR)
	{
		Err = SendAlarm(Err, _T("Drop object on the camera"));
		return Err;
	}

	return Err;
}

void CPowerMain::PrepareLastPickData()
{
	long Gantry = m_Gantry;
	LASTPICK dataOld;
	LASTPICK dataNew;
    bool enableClear;
	long sucNo;
	ULONGLONG pickElaspedTime;
    CString packageNameOld;
    CString packageNameNew;
    long packageIndex;
    bool existClear;
    long InsertNo;

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
        enableClear = false;
        existClear = false;
		
		sucNo = GetSuctionIONo(Gantry, head);

		dataOld = gcLastPickFront->GetHeadData(head);

        InsertNo = gcReadJobFile->GetInsertNo(dataOld.Insert.BlockNo, dataOld.Insert.index);

		dataNew = dataOld;
		dataNew.Insert = gcReadJobFile->GetInsert(InsertNo);

        packageIndex = gcReadJobFile->GetPackgeIndexFromFeederNo(dataOld.Insert.FeederNo);
        packageNameNew = gcReadJobFile->GetPackage(packageIndex).Name;
        packageNameOld = dataOld.Package.Name;

		pickElaspedTime = _time_elapsed(dataOld.SetTime);

        if (ReadOutputOne(sucNo) == OUTOFF)
        {
            TRACE(_T("[PWR] LastPickData %s Clear Suction Off \n"), GetZAxisFromHeadNo(Gantry, head));
            existClear = true;
        }

        if (existClear)
        {
            gcLastPickFront->SetHeadDataExist(head, false);
            break;
        }

		if (dataOld.Enable == false)
		{
            enableClear = true;
		}
		else if (ReadOutputOne(sucNo) == OUTOFF)
		{
			TRACE(_T("[PWR] LastPickData %s Clear Suction Off \n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
		}
		else if (GetInsertEnd(FRONT_CONV, Gantry, dataOld.Insert.BlockNo, InsertNo) == INSERT_END)
		{
			TRACE(_T("[PWR] LastPickData %s Clear InsertEnd \n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
		}
		else if (GetGlobalNozzleNo(head) != dataOld.Insert.NozzleNo)
		{
			TRACE(_T("[PWR] LastPickData %s Clear Nozzle different \n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
		}
		else if (gcLastPickFront->IsSameHeadData(dataOld, dataNew) == false)
		{
			TRACE(_T("[PWR] LastPickData %s Clear Data different \n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
		}
		else if (OutputElapsedTimeOne(sucNo, OUTON, pickElaspedTime) == false)
		{
			TRACE(_T("[PWR] LastPickData %s Set time different\n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
		}
        else if (packageNameOld.CompareNoCase(packageNameNew) != 0)
        {
            TRACE(_T("[PWR] LastPickData %s Package different\n"), GetZAxisFromHeadNo(Gantry, head));
            enableClear = true;
        }

		if (enableClear == true)
		{
            gcLastPickFront->SetHeadDataEnable(head, false);
            break;
		}
	}
}

long CPowerMain::GetFirstInsertNoLastPickData()
{
	long Gantry = m_Gantry;
	LASTPICK data;
	long lastStep = 0;
	long insertNoMin = MAXINSERTNO;
	long MaxInsertNo = m_Production.TotalInsertCount;
    long insertNo;

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
		data = gcLastPickFront->GetHeadData(head);
        insertNo = gcReadJobFile->GetInsertNo(data.Insert.BlockNo, data.Insert.index);

		if (data.Enable == true && insertNo > 0)
		{
			lastStep = data.Insert.Step;
			break;
		}
	}

	if (lastStep == 0)
	{
		return 0;
	}


	for (long insert = 1; insert <= MaxInsertNo; insert++)
	{
		if (lastStep != gcStep->GetRunStepNo(insert)) continue;

		if (insertNoMin > insert)
		{
			insertNoMin = insert;
		}
	}

	if (insertNoMin == MAXINSERTNO)
	{
		return 0;
	}

	return insertNoMin;

}

Ratio_XYRZ CPowerMain::GetMinRatioLastPickData()
{
	long Gantry = m_Gantry;
	LASTPICK data;
	long insertNo = MAXINSERTNO;
	Ratio_XYRZ MinRatio;
	Ratio_XYRZ tempRatio;
	
	MinRatio.xy = MinRatio.z = MinRatio.r = 1.0;

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
		data = gcLastPickFront->GetHeadData(head);
        insertNo = gcReadJobFile->GetInsertNo(data.Insert.BlockNo, data.Insert.index);

		if (data.Enable == true && insertNo > 0)
		{
			tempRatio = gcStep->GetRatioFromFdNo(data.Insert.FeederNo);
			if (tempRatio.xy < MinRatio.xy)
			{
				MinRatio.xy = tempRatio.xy;
			}

			if (tempRatio.r < MinRatio.r)
			{
				MinRatio.r = tempRatio.r;
			}

			if (tempRatio.z < MinRatio.z)
			{
				MinRatio.z = tempRatio.z;
			}
		}
	}

	TRACE(_T("[PWR] LastPickData MinRatio XY %.1f R %.1f Z %.1f\n"), MinRatio.xy * 100.0, MinRatio.r * 100.0, MinRatio.z * 100.0);
	return MinRatio;
}


CString CPowerMain::GetStepName(MainStep RunStep)
{
	CString name;

	switch (RunStep)
	{
	case MainStep::STOP:
		name.Format(_T("STOP[%d]"), (long)RunStep);
		break;
	case MainStep::START:
		name.Format(_T("START[%d]"), (long)RunStep);
		break;
	case MainStep::QUANTITY:
		name.Format(_T("QUANTITY[%d]"), (long)RunStep);
		break;
	case MainStep::PREPARE:
		name.Format(_T("PREPARE[%d]"), (long)RunStep);
		break;
	case MainStep::DISCARD_BEFORE_RUN:
		name.Format(_T("DISCARD_BEFORE_RUN[%d]"), (long)RunStep);
		break;
	case MainStep::MACHINE_REFERENCE_MARK:
		name.Format(_T("MACHINE_REFERENCE_MARK[%d]"), (long)RunStep);
		break;
	case MainStep::STANDBY:
		name.Format(_T("STANDBY[%d]"), (long)RunStep);
		break;
	case MainStep::CONTINUE:
		name.Format(_T("CONTINUE[%d]"), (long)RunStep);
		break;
	case MainStep::STEP:
		name.Format(_T("STEP[%d]"), (long)RunStep);
		break;
	case MainStep::IS_EMPTY:
		name.Format(_T("IS_EMPTY[%d]"), (long)RunStep);
		break;
	case MainStep::SET_NOZZLE:
		name.Format(_T("SET_NOZZLE[%d]"), (long)RunStep);
		break;
	case MainStep::IS_WORK_EXIST:
		name.Format(_T("IS_WORK_EXIST[%d]"), (long)RunStep);
		break;
	case MainStep::MOVE_SAFTY_Z_BEFORE_PICKUP:
		name.Format(_T("MOVE_SAFTY_Z_BEFORE_PICKUP[%d]"), (long)RunStep);
		break;
	case MainStep::IS_ENTRY_PCB_READY_1ST:
		name.Format(_T("IS_ENTRY_PCB_READY_1ST[%d]"), (long)RunStep);
		break;
	case MainStep::IS_WORK_PCB_READY_1ST:
		name.Format(_T("IS_WORK_PCB_READY_1ST[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_FIDUCIAL_1ST:
		name.Format(_T("CHECK_FIDUCIAL_1ST[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_BARCODE:
		name.Format(_T("CHECK_BARCODE[%d]"), (long)RunStep);
		break;
	case MainStep::SEND_BARCODE:
		name.Format(_T("SEND_BARCODE[%d]"), (long)RunStep);
		break;
	case MainStep::GET_BARCODE_RESULT:
		name.Format(_T("GET_BARCODE_RESULT[%d]"), (long)RunStep);
		break;
	case MainStep::NG_BARCODE:
		name.Format(_T("NG_BARCODE[%d]"), (long)RunStep);
		break;
	case MainStep::OUT_BARCODE:
		name.Format(_T("OUT_BARCODE[%d]"), (long)RunStep);
		break;
	case MainStep::PICK:
		name.Format(_T("PICK[%d]"), (long)RunStep);
		break;
	case MainStep::DISCARD_BEFORE_RECOGNITION:
		name.Format(_T("DISCARD_BEFORE_RECOGNITION[%d]"), (long)RunStep);
		break;
	case MainStep::RECOGNITION:
		name.Format(_T("RECOGNITION[%d]"), (long)RunStep);
		break;
	case MainStep::OUT_BEFORE_PCB:
		name.Format(_T("OUT_BEFORE_PCB[%d]"), (long)RunStep);
		break;
	case MainStep::MOVE_SAFTY_Z_AFTER_PICKUP:
		name.Format(_T("MOVE_SAFTY_Z_AFTER_PICKUP[%d]"), (long)RunStep);
		break;
	case MainStep::DISCARD_AFTER_RECOGNITION:
		name.Format(_T("DISCARD_AFTER_RECOGNITION[%d]"), (long)RunStep);
		break;
	case MainStep::IS_WORK_PCB_READY_2ND:
		name.Format(_T("IS_WORK_PCB_READY_2ND[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_FIDUCIAL_2ND:
		name.Format(_T("CHECK_FIDUCIAL_2ND[%d]"), (long)RunStep);
		break;
	case MainStep::INSERT:
		name.Format(_T("INSERT[%d]"), (long)RunStep);
		break;
	case MainStep::DISCARD:
		name.Format(_T("DISCARD[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLEDONE:
		name.Format(_T("CYCLEDONE[%d]"), (long)RunStep);
		break;
	case MainStep::BLOCKDONE:
		name.Format(_T("BLOCKDONE[%d]"), (long)RunStep);
		break;
	case MainStep::BOARDDONE:
		name.Format(_T("BOARDDONE[%d]"), (long)RunStep);
		break;
	case MainStep::MEASURE_HEIGHT:
		name.Format(_T("MEASURE_HEIGHT[%d]"), (long)RunStep);
		break;
	case MainStep::PRODUCT_COMPLETE:
		name.Format(_T("PRODUCT_COMPLETE[%d]"), (long)RunStep);
		break;
	case MainStep::STANDBY_COMPLETE:
		name.Format(_T("STANDBY_COMPLETE[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLESTOP:
		name.Format(_T("CYCLESTOP[%d]"), (long)RunStep);
		break;
	case MainStep::BLOCKSTOP:
		name.Format(_T("BLOCKSTOP[%d]"), (long)RunStep);
		break;
	case MainStep::BOARDSTOP:
		name.Format(_T("BOARDSTOP[%d]"), (long)RunStep);
		break;
	case MainStep::OUT_PCB:
		name.Format(_T("OUT_PCB[%d]"), (long)RunStep);
		break;
	case MainStep::END:
		name.Format(_T("END[%d]"), (long)RunStep);
		break;
	case MainStep::RELEASE_NOZZLE:
		name.Format(_T("RELEASE_NOZZLE[%d]"), (long)RunStep);
		break;
	case MainStep::SELFQUIT:
		name.Format(_T("SELFQUIT[%d]"), (long)RunStep);
		break;
	case MainStep::LOCK_BEFORE_RECOG:
		name.Format(_T("LOCK_BEFORE_RECOG[%d]"), (long)RunStep);
		break;
	case MainStep::LOCK_BEFORE_INSERT:
		name.Format(_T("LOCK_BEFORE_INSERT[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_COLLISION_BEFORE_RECOG:
		name.Format(_T("CHECK_COLLISION_BEFORE_RECOG[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_COLLISION_BEFORE_INSERT:
		name.Format(_T("CHECK_COLLISION_BEFORE_INSERT[%d]"), (long)RunStep);
		break;
	case MainStep::MOVE_STANDBY_BEFORE_WAITOTHER:
		name.Format(_T("MOVE_STANDBY_BEFORE_WAITOTHER[%d]"), (long)RunStep);
		break;
	case MainStep::WAIT_OTHER:
		name.Format(_T("WAIT_OTHER[%d]"), (long)RunStep);
		break;
	case MainStep::LOCK_BEFORE_FID_1ST:
		name.Format(_T("LOCK_BEFORE_FID_1ST[%d]"), (long)RunStep);
		break;
	case MainStep::LOCK_BEFORE_FID_2ND:
		name.Format(_T("LOCK_BEFORE_FID_2ND[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_COLLISION_BEFORE_FID_1ST:
		name.Format(_T("CHECK_COLLISION_BEFORE_FID_1ST[%d]"), (long)RunStep);
		break;
	case MainStep::CHECK_COLLISION_BEFORE_FID_2ND:
		name.Format(_T("CHECK_COLLISION_BEFORE_FID_2ND[%d]"), (long)RunStep);
		break;
	case MainStep::START_GET_FIRST_CONV:
		name.Format(_T("START_GET_FIRST_CONV[%d]"), (long)RunStep);
		break;
	case MainStep::SELECT_CONV:
		name.Format(_T("SELECT_CONV[%d]"), (long)RunStep);
		break;
	case MainStep::BLOCKSKIP_HEIGHTMEASURE:
		name.Format(_T("BLOCKSKIP_HEIGHTMEASURE[%d]"), (long)RunStep);
		break;
	case MainStep::BOARDSTOP_SEND_PAUSE:
		name.Format(_T("BOARDSTOP_SEND_PAUSE[%d]"), (long)RunStep);
		break;
	case MainStep::BOARDSTOP_WAITPAUSE:
		name.Format(_T("BOARDSTOP_WAITPAUSE[%d]"), (long)RunStep);
		break;
	case MainStep::BOARDSTOP_WAITLOCK:
		name.Format(_T("BOARDSTOP_WAITLOCK[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLESTOP_WAITOTHER:
		name.Format(_T("CYCLESTOP_WAITOTHER[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLESTOP_WAITPAUSE:
		name.Format(_T("CYCLESTOP_WAITPAUSE[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLESTOP_WAITLOCK:
		name.Format(_T("CYCLESTOP_WAITLOCK[%d]"), (long)RunStep);
		break;
	case MainStep::CYCLESTOP_NEXTSTEP:
		name.Format(_T("CYCLESTOP_NEXTSTEP[%d]"), (long)RunStep);
		break;
	case MainStep::DROPCHECK_BEFORE_RUN:
		name.Format(_T("DROPCHECK_BEFORE_RUN[%d]"), (long)RunStep);
		break;
	case MainStep::WAIT_PREVPOINT_RECOGPASS:
		name.Format(_T("WAIT_PREVPOINT_RECOGPASS[%d]"), (long)RunStep);
		break;
	case MainStep::WAIT_PREVPOINT_INSERTEND:
		name.Format(_T("WAIT_PREVPOINT_INSERTEND[%d]"), (long)RunStep);
		break;
    case MainStep::ALIGN_COMPONENT:
        name.Format(_T("ALIGN_COMPONENT[%d]"), (long)RunStep);
        break;
    case MainStep::RETURN_AFTER_BOARDDONE:
        name.Format(_T("RETURN_AFTER_BOARDDONE[%d]"), (long)RunStep);
        break;
    case MainStep::CHECK_BARCODE_2ND:
        name.Format(_T("CHECK_BARCODE_2ND[%d]"), (long)RunStep);
        break;
    case MainStep::SEND_BARCODE_2ND:
        name.Format(_T("SEND_BARCODE_2ND[%d]"), (long)RunStep);
        break;
    case MainStep::GET_BARCODE_RESULT_2ND:
        name.Format(_T("GET_BARCODE_RESULT_2ND[%d]"), (long)RunStep);
        break;        
	default:
		name.Format(_T("UNDEFINE[%d]"), (long)RunStep);
		break;
	}

	return name;
}

bool CPowerMain::CheckBlockSkipHMUse()
{
    PCB Pcb = gcReadJobFile->GetPcb();

    for (long BlockNo = 1; BlockNo <= Pcb.MaxBlockCount; BlockNo++)
    {
        if (gcReadJobFile->GetBlockSkipHM(BlockNo).Use == 1)
        {
            return true;
        }
    }

    return false;
}

void CPowerMain::SetHMFirst(bool set)
{
    if (m_HMfirst != set)
    {
        TRACE(_T("[PWR] SetHMFirst:%d"), set);
    }
    m_HMfirst = set;
}

bool CPowerMain::GetHMFirst()
{
    return m_HMfirst;
}

long CPowerMain::SendHeightMeasureEnd()
{
    long Err;
    Err = gcMeasureHeight->SendHeightMeasureEnd();
    return Err;
}

long CPowerMain::StartPrevProdHeightMeasure()
{
    CString strFunc(__func__);
    MainStep Step = MainStep::STANDBY;
    MainStep OldStep = MainStep::STOP;
    BARCODE Barcode = gcStep->GetBarcode();

    long Gantry = FRONT_GANTRY;
    long Conveyor = FRONT_CONV;
    long Err = NO_ERR;
    long EIMESResult;

    ULONGLONG StepGetTime = _time_get();
    ULONGLONG StepElapsedTime = 0;

    CString strBarcode = GetBarcodeCarrier();
    //CString strBarcodeBlock1 = GetBarcodeBlock1();
    //CString strBarcodeBlock2 = GetBarcodeBlock2();

    while (GetRunning() == true)
    {
        if (OldStep != Step)
        {
            StepElapsedTime = _time_elapsed(StepGetTime);
            TRACE(_T("[PWR] %s Step:%d Time:%d[ms]\n"), strFunc, Step, StepElapsedTime);
            OldStep = Step;
            StepGetTime = _time_get();
        }

        if (GetGlobalStatusError() == true)
        {
            TRACE(_T("[PWR] %s GetGlobalStatusError(%d)\n"), strFunc, GetGlobalStatusError());
            Err = STOP_NOW;
            break;
        }

        if (GetMachineState() == STATE_STOPNOW)
        {
            TRACE(_T("[PWR] %s GetMachineState(%d)\n"), strFunc, GetMachineState());
            Err = STOP_NOW;
            break;
        }

        switch (Step)
        {
        case MainStep::STOP:
            break;

        case MainStep::STANDBY:
            Err = MoveStandBy();
            if (Err != NO_ERR)
            {
                Step = MainStep::SELFQUIT;
                break;
            }

            Step = MainStep::IS_WORK_PCB_READY_1ST;
            break;

        case MainStep::IS_WORK_PCB_READY_1ST:
            if (IsWorkPcbReady(Conveyor) == true)
            {
                Step = MainStep::BLOCKSKIP_HEIGHTMEASURE;
            }
            break;

        case MainStep::BLOCKSKIP_HEIGHTMEASURE:
            if (CheckBlockSkipHMUse() == false)
            {
                Step = MainStep::CHECK_FIDUCIAL_1ST;
                break;
            }

            Err = gcMeasureHeight->BlockSkipCheck();
            if (Err != NO_ERR)
            {
                Step = MainStep::SELFQUIT;
                break;
            }

            Step = MainStep::CHECK_FIDUCIAL_1ST;
            break;

        case MainStep::CHECK_FIDUCIAL_1ST:
            Err = gcMark->FiducialMarkChecking();
            if (Err != NO_ERR)
            {
                Step = MainStep::SELFQUIT;
                break;
            }

            Step = MainStep::CHECK_BARCODE;
            break;

        case MainStep::CHECK_BARCODE:

            MES_INIT();

            Err = gcBarcode->BarcodeChecking();
            if (Err != NO_ERR)
            {
                Step = MainStep::SELFQUIT;
                break;
            }

            Step = MainStep::SEND_BARCODE;
            break;

        case MainStep::SEND_BARCODE: // 20210415 HarkDo
            strBarcode = GetBarcodeCarrier();
            
            MES_SendBarcodeSNTMotive(strBarcode);

            if (CheckBlockSkipHMUse() == true && gcStep->GetAllBlockSkipHM() == true)
            {
                TRACE(_T("[PWR] Current PCB All Block Skip\n"));
                MoveStandBy();
                Step = MainStep::SELFQUIT;
            }
            else
            {
                Step = MainStep::GET_BARCODE_RESULT;
                SendPopupMessage(_T("Waiting result from MES"));
            }
            break;

        case MainStep::GET_BARCODE_RESULT:
            EIMESResult = MES_GET();

            if (EIMESResult == NG_EIMES)
            {
                SendPopupClose();
                Step = MainStep::SELFQUIT;
            }
            else if (EIMESResult == OK_EIMES)
            {
                SendPopupClose();
                Step = MainStep::MEASURE_HEIGHT;
            }
            else // Waiting MES Result
                ;
            break;

        case MainStep::MEASURE_HEIGHT:
            Err = gcMeasureHeight->Run(0);
            if (Err == NO_ERR || Err == HEIGHT_MEASUREMENT_FAIL)
            {
                SendHeightMeasureEnd();
                Err = NO_ERR;
            }
            else
            {
            }
            Step = MainStep::SELFQUIT;

            break;


        case MainStep::SELFQUIT:
            break;

        }

        if (Step == MainStep::SELFQUIT)
        {
            break;
        }

        ThreadSleep(TIME1MS);
    }

    TRACE(_T("[PWR] %s end(%d)\n"), strFunc, Err);

    if (Err == NO_ERR || Err == HEIGHT_MEASUREMENT_FAIL)
    {
    }
    else
    {
        SendAlarm(Err, _T("HeightMeasure Error"));
    }

    return Err;
}

CString CPowerMain::GetBarcodeCarrier()
{
    CString strBarcode;
    strBarcode.Format(_T("%s"), (LPCTSTR)(gcStep->GetBarcodeCarrier()));
    return strBarcode;
}

CString CPowerMain::GetBarcodeBlock1()
{
    CString strBarcode;
    strBarcode.Format(_T("%s"), (LPCTSTR)(gcStep->GetBarcodeBlock1()));
    return strBarcode;
}

CString CPowerMain::GetBarcodeBlock2()
{
    CString strBarcode;
    strBarcode.Format(_T("%s"), (LPCTSTR)(gcStep->GetBarcodeBlock2()));
    return strBarcode;
}

long CPowerMain::MES_INIT()
{
    m_MES_Result = INIT_MES;
    return NO_ERR;
}

long CPowerMain::MES_NG()
{
    m_MES_Result = NG_MES;
    return NO_ERR;
}

long CPowerMain::MES_OK()
{
    m_MES_Result = OK_MES;
    return NO_ERR;
}

long CPowerMain::MES_ABORT()
{
    m_MES_Result = ABORT_MES;
    return NO_ERR;
}

long CPowerMain::MES_GET()
{
    return m_MES_Result;
}

long CPowerMain::SetBoardNo(long BoardNo)
{
    if (gcStep)
    {
        gcStep->SetBoardNo(BoardNo);
    }
    return NO_ERR;
}

long CPowerMain::MES_Disconnect()
{
    m_MES_Result = DISCONECT_MES;
    return m_MES_Result;
}

void CPowerMain::traceMcsVersion()
{
    LPTSTR lpszFilePath = L"C:\\Power\\i6.0\\MCS\\PowerMCS.exe";

    DWORD dwDummy;
    DWORD dwFVISize = GetFileVersionInfoSize(lpszFilePath, &dwDummy);

    if (dwFVISize == 0)
    {
        CString temp;
        temp.Format(L"cannot find %s", lpszFilePath);
        TRACE("[PWR] %s::%s -> %s", typeid(CPowerMain).name(), __func__, (CStringA)temp);
        return;
    }

    LPBYTE lpVersionInfo = new BYTE[dwFVISize];

    GetFileVersionInfo(lpszFilePath, 0, dwFVISize, lpVersionInfo);

    UINT uLen;
    VS_FIXEDFILEINFO* lpFfi;

    VerQueryValue(lpVersionInfo, _T("\\"), (LPVOID*)&lpFfi, &uLen);

    DWORD dwFileVersionMS = lpFfi->dwFileVersionMS;
    DWORD dwFileVersionLS = lpFfi->dwFileVersionLS;

    delete[] lpVersionInfo;

    DWORD dwLeftMost = HIWORD(dwFileVersionMS);
    DWORD dwSecondLeft = LOWORD(dwFileVersionMS);
    DWORD dwSecondRight = HIWORD(dwFileVersionLS);
    DWORD dwRightMost = LOWORD(dwFileVersionLS);

    CString str;

    str.Format(L"Version: %lu.%lu.%lu.%lu\n"
               , dwLeftMost
               , dwSecondLeft
               , dwSecondRight
               , dwRightMost
    );

    TRACE("[PWR] %s::%s -> %s", typeid(CPowerMain).name(), __func__, (CStringA)str);
    return;
}

void CPowerMain::SetBoardTimeExcute(bool Set) {
    if (Set != m_SetBoardStart) {
        TRACE(_T("[PWR] SetBoardTimeExcute,%d,%d \n"), GetTable(), Set);
    }

    m_SetBoardStart = Set;
    m_BoardStartTime = _time_get();
}

bool CPowerMain::GetBoardTimeExcute() {
    return m_SetBoardStart;
}

ULONGLONG CPowerMain::GetBoardTimeElapsed() {
    if (m_SetBoardStart == false) {
        return 0;
    }

    return _time_elapsed(m_BoardStartTime);
}

long CPowerMain::calculateProductionTime(const long& ProdQuantity) {
    const ULONGLONG ProdElapsedTime = this->GetBoardTimeElapsed();
    ULONGLONG BoardTimeChk = ProdElapsedTime;
    const ULONGLONG PartTimeChk = BoardTimeChk / this->m_RealMaxInsertCount;

    ULONGLONG BoardTimeChkWithLoadingTime, PartTimeChkWithLoadingTime;
    if (GetUseLineOfBalance() == 1) {
        BoardTimeChkWithLoadingTime = GetConveyorLineOfBalance();
        PartTimeChkWithLoadingTime = BoardTimeChkWithLoadingTime / this->m_RealMaxInsertCount;
    }
    else {
        BoardTimeChkWithLoadingTime = BoardTimeChk + GetConveyorLoadingTime();
        PartTimeChkWithLoadingTime = BoardTimeChkWithLoadingTime / this->m_RealMaxInsertCount;
    }

    ULONGLONG PartPerHour = 0;
    if (PartTimeChk > 10)
        PartPerHour = 3600000 / PartTimeChk;
    else
        PartPerHour = 0;

    TRACE(_T("[PWR] *******************************************************************************\n"));
    TRACE(_T("[PWR] ********** ProdInfo Quantity(%04d) Board:%d Part:%d CPH:%d \n"),
        ProdQuantity, BoardTimeChk, PartTimeChk, PartPerHour);
    TRACE(_T("[PWR] *******************************************************************************\n"));

    BARCODE Barcode = gcStep->GetBarcode();
    if (GetCustomerSiteFromBarcodeSetting(Barcode) == CUSTOMER_DEFAULT) // 단뱡향(Single) : 파일만 생성
    {
        CString strBarcode = this->GetBarcodeCarrier();
        SendProdInfo(ProdQuantity, (long)BoardTimeChk, (long)PartTimeChk, (long)PartPerHour, (long)BoardTimeChkWithLoadingTime, (long)PartTimeChkWithLoadingTime, strBarcode);
    }
    else {
        SendProdInfo(ProdQuantity, (long)BoardTimeChk, (long)PartTimeChk, (long)PartPerHour, (long)BoardTimeChkWithLoadingTime, (long)PartTimeChkWithLoadingTime);
    }

    return 0;
}