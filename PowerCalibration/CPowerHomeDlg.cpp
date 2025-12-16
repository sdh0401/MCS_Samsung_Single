// CPowerHomeDlg.cpp: 구현 파일
//
#include "pch.h"
#include "PowerCalibration.h"
#include "PowerCalibrationDlg.h"
#include "CPowerHomeDlg.h"
#include "CStartCalibrationFunc.h"
/*****************************************************************************/
/* Header                                                                    */
/*****************************************************************************/
#include "GlobalDefine.h"
#include "Vision.h"
#include "Trace.h"
#include "EthernetVision.h"
#include "AxisInformation.h"
#include "CHomeStatus.h"
#include "CStart1D.h"
#include "CStart2D.h"
#include "CStartAlignOffset.h"
#include "CStartHeadOffset.h"
#include "CStartModuleCamera.h"
#include "CStartOffsetCamera.h"
#include "CPowerCalibrationData.h"
#include "CTokenizer.h"
#include "CPowerLog.h"
//#include "ErrorCode.h"
#include "CPowerIO.h"
#include "GlobalIODefine.h"
#include "CAutoNozzleChange.h"
#include "CMachineConfig.h"
#include "Cwmx3Init.h"
#include "CMachineFileDB.h"
#include "CDecoding3.h"

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/

// CPowerHomeDlg
IMPLEMENT_DYNCREATE(CPowerHomeDlg, CFormView)
CPowerHomeDlg::CPowerHomeDlg()
	: CFormView(IDD_FORM_HOME)
{
}

CPowerHomeDlg::~CPowerHomeDlg()
{
	if (gcHomeStatus)
	{
		delete gcHomeStatus;
		gcHomeStatus = NULL;
	}
}

void CPowerHomeDlg::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_R1D_SIMULON, m_1DSimulOn);
	DDX_Control(pDX, IDC_R1D_SIMULOFF, m_1DSimulOff);
	DDX_Control(pDX, IDC_RHDCAM_SIMULON, m_HdCamSimulOn);
	DDX_Control(pDX, IDC_RHDCAM_SIMULOFF, m_HdCamSimulOff);
	DDX_Control(pDX, IDC_R2D_SIMULON, m_2DSimulOn);
	DDX_Control(pDX, IDC_R2D_SIMULOFF, m_2DSimulOff);
	DDX_Control(pDX, IDC_R2D_WMX3, m_2DWmx3);
	DDX_Control(pDX, IDC_R2D_SW, m_2DSoftware);
	DDX_Control(pDX, IDC_CHECK_HOME_REPEAT, m_ChkBtnHomeRepeat);
	DDX_Control(pDX, IDC_CHECK_ALIGN_REPEAT, m_ChkBtnAlignRepeat);
	DDX_Control(pDX, IDC_CHECK_HEADOFFSET_REPEAT, m_ChkBtnHeadOffsetRepeat);
	DDX_Control(pDX, IDC_RB_ZNO1, m_RbZNo1);
	DDX_Control(pDX, IDC_RB_ZNO2, m_RbZNo2);
	DDX_Control(pDX, IDC_RB_ZNO3, m_RbZNo3);
	DDX_Control(pDX, IDC_RB_ZNO4, m_RbZNo4);
	DDX_Control(pDX, IDC_RB_ZNO5, m_RbZNo5);
	DDX_Control(pDX, IDC_RB_ZNO6, m_RbZNo6);
	DDX_Control(pDX, IDC_RB_FRONT, m_RbFrontGantry);
	DDX_Control(pDX, IDC_RB_REAR, m_RbRearGantry);
	DDX_Control(pDX, IDC_RB_CAM1, m_RbCam1);
	DDX_Control(pDX, IDC_RB_CAM2, m_RbCam2);
	DDX_Control(pDX, IDC_RADIO1, m_RbFX);
	DDX_Control(pDX, IDC_RADIO11, m_RbFY1);
	DDX_Control(pDX, IDC_RADIO12, m_RbFY2);
	DDX_Control(pDX, IDC_RADIO13, m_RbFZ1);
	DDX_Control(pDX, IDC_RADIO14, m_RbFZ2);
	DDX_Control(pDX, IDC_RADIO15, m_RbFZ3);
	DDX_Control(pDX, IDC_RADIO16, m_RbFZ4);
	DDX_Control(pDX, IDC_RADIO17, m_RbFZ5);
	DDX_Control(pDX, IDC_RADIO18, m_RbFZ6);
	DDX_Control(pDX, IDC_RADIO19, m_RbFW1);
	DDX_Control(pDX, IDC_RADIO20, m_RbFW2);
	DDX_Control(pDX, IDC_RADIO21, m_RbFW3);
	DDX_Control(pDX, IDC_RADIO22, m_RbFW4);
	DDX_Control(pDX, IDC_RADIO23, m_RbFW5);
	DDX_Control(pDX, IDC_RADIO24, m_RbFW6);
	DDX_Control(pDX, IDC_RADIO25, m_RbFCONV);
	DDX_Control(pDX, IDC_RADIO26, m_RbFPUZ);
	DDX_Control(pDX, IDC_RADIO27, m_RbFBTIN);
	DDX_Control(pDX, IDC_RADIO28, m_RbFBTWK);
	DDX_Control(pDX, IDC_RADIO29, m_RbFBTOT);
	DDX_Control(pDX, IDC_RADIO9, m_RbCam5);
	DDX_Control(pDX, IDC_RADIO10, m_RbCam6);
	DDX_Control(pDX, IDC_RB_FANC, m_RbFrontANC);
	DDX_Control(pDX, IDC_RB_RANC, m_RbRearANC);
	DDX_Control(pDX, IDC_RB_ANCR0, m_RbANC_R0);
	DDX_Control(pDX, IDC_RB_ANCR90, m_RbANC_R90);
	DDX_Control(pDX, IDC_CHECK_UPDATEINSERTOFFSET, m_ChkBtnUpdateinsertoffset);

}

BEGIN_MESSAGE_MAP(CPowerHomeDlg, CFormView)
	ON_BN_CLICKED(IDC_BTN_1D, &CPowerHomeDlg::OnBnClickedBtn1d)
	ON_BN_CLICKED(IDC_BTN_HOMING, &CPowerHomeDlg::OnBnClickedBtnHoming)
	ON_BN_CLICKED(IDC_BTN_CAL_CANCEL, &CPowerHomeDlg::OnBnClickedBtnCalCancel)
	ON_BN_CLICKED(IDC_R1D_SIMULON, &CPowerHomeDlg::OnBnClickedR1dSimulon)
	ON_BN_CLICKED(IDC_BTN_1D_ONOFF, &CPowerHomeDlg::OnBnClickedBtn1dOnoff)
	ON_BN_CLICKED(IDC_BTN_HEADCAM_CAL, &CPowerHomeDlg::OnBnClickedBtnHeadcamCal)
	ON_BN_CLICKED(IDC_R1D_SIMULOFF, &CPowerHomeDlg::OnBnClickedR1dSimuloff)
	ON_BN_CLICKED(IDC_RHDCAM_SIMULON, &CPowerHomeDlg::OnBnClickedRhdcamSimulon)
	ON_BN_CLICKED(IDC_RHDCAM_SIMULOFF, &CPowerHomeDlg::OnBnClickedRhdcamSimuloff)
	ON_BN_CLICKED(IDC_BTN_HDCAM_ONOFF, &CPowerHomeDlg::OnBnClickedBtnHdcamOnoff)
	ON_BN_CLICKED(IDC_BTN_2D, &CPowerHomeDlg::OnBnClickedBtn2d)
	ON_BN_CLICKED(IDC_BTN_2D_ONOFF, &CPowerHomeDlg::OnBnClickedBtn2dOnoff)
	ON_BN_CLICKED(IDC_BTN_GETVISIONVERSION, &CPowerHomeDlg::OnBnClickedBtnGetvisionversion)
	ON_BN_CLICKED(IDC_BUTTON1, &CPowerHomeDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_R2D_SIMULON, &CPowerHomeDlg::OnBnClickedR2dSimulon)
	ON_BN_CLICKED(IDC_R2D_SIMULOFF, &CPowerHomeDlg::OnBnClickedR2dSimuloff)
	ON_BN_CLICKED(IDC_R2D_WMX3, &CPowerHomeDlg::OnBnClickedR2dWmx3)
	ON_BN_CLICKED(IDC_R2D_SW, &CPowerHomeDlg::OnBnClickedR2dSw)
	ON_BN_CLICKED(IDC_BUTTON2, &CPowerHomeDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_CHECK_HOME_REPEAT, &CPowerHomeDlg::OnBnClickedCheckHomeRepeat)
	ON_BN_CLICKED(IDC_BTN_ALIGN_OFFSET, &CPowerHomeDlg::OnBnClickedBtnAlignOffset)
	ON_BN_CLICKED(IDC_CHECK_ALIGN_REPEAT, &CPowerHomeDlg::OnBnClickedCheckAlignRepeat)
	ON_BN_CLICKED(IDC_BTN_HEAD_OFFSET, &CPowerHomeDlg::OnBnClickedBtnHeadOffset)
	ON_BN_CLICKED(IDC_CHECK_HEADOFFSET_REPEAT, &CPowerHomeDlg::OnBnClickedCheckHeadoffsetRepeat)
	ON_BN_CLICKED(IDC_BTN_SETHOMEZ, &CPowerHomeDlg::OnBnClickedBtnSethomez)
	ON_BN_CLICKED(IDC_BTN_SETPCBFIX, &CPowerHomeDlg::OnBnClickedBtnSetpcbfix)
	ON_BN_CLICKED(IDC_BTN_SETFDREF, &CPowerHomeDlg::OnBnClickedBtnSetfdref)
	ON_BN_CLICKED(IDC_RB_FRONT, &CPowerHomeDlg::OnBnClickedRbFront)
	ON_BN_CLICKED(IDC_RB_REAR, &CPowerHomeDlg::OnBnClickedRbRear)
	ON_BN_CLICKED(IDC_RB_ZNO1, &CPowerHomeDlg::OnBnClickedRbZno1)
	ON_BN_CLICKED(IDC_RB_ZNO2, &CPowerHomeDlg::OnBnClickedRbZno2)
	ON_BN_CLICKED(IDC_RB_ZNO3, &CPowerHomeDlg::OnBnClickedRbZno3)
	ON_BN_CLICKED(IDC_RB_ZNO4, &CPowerHomeDlg::OnBnClickedRbZno4)
	ON_BN_CLICKED(IDC_RB_ZNO5, &CPowerHomeDlg::OnBnClickedRbZno5)
	ON_BN_CLICKED(IDC_RB_ZNO6, &CPowerHomeDlg::OnBnClickedRbZno6)
	ON_BN_CLICKED(IDC_BTN_SEND_CAMOFFSET, &CPowerHomeDlg::OnBnClickedBtnSendCamoffset)
	ON_BN_CLICKED(IDC_BTN_GOPCBFIX, &CPowerHomeDlg::OnBnClickedBtnGopcbfix)
	ON_BN_CLICKED(IDC_BTN_GOFDREF, &CPowerHomeDlg::OnBnClickedBtnGofdref)
	ON_BN_CLICKED(IDC_BTN_SET_FEEDER, &CPowerHomeDlg::OnBnClickedBtnSetFeeder)
	ON_WM_SHOWWINDOW()
	ON_BN_CLICKED(IDC_BTN_SET_ALIGN_OFFSET, &CPowerHomeDlg::OnBnClickedBtnSetAlignOffset)
	ON_BN_CLICKED(IDC_BTN_ZCOMPEN, &CPowerHomeDlg::OnBnClickedBtnZcompen)
	ON_BN_CLICKED(IDC_BTN_ZCOMPEN_ON, &CPowerHomeDlg::OnBnClickedBtnZcompenOn)
	ON_BN_CLICKED(IDC_BTN_MODULECAM_CAL, &CPowerHomeDlg::OnBnClickedBtnModulecamCal)
	ON_BN_CLICKED(IDC_RB_CAM2, &CPowerHomeDlg::OnBnClickedRbCam2)
	ON_BN_CLICKED(IDC_RB_CAM1, &CPowerHomeDlg::OnBnClickedRbCam1)
	ON_BN_CLICKED(IDC_BTN_INIT_HEAD_OFFSET, &CPowerHomeDlg::OnBnClickedBtnInitHeadOffset)
	ON_BN_CLICKED(IDC_BUTTON4, &CPowerHomeDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_RADIO1, &CPowerHomeDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_BTN_SET_LIMIT, &CPowerHomeDlg::OnBnClickedBtnSetLimit)
	ON_BN_CLICKED(IDC_RADIO11, &CPowerHomeDlg::OnBnClickedRadio11)
	ON_BN_CLICKED(IDC_RADIO12, &CPowerHomeDlg::OnBnClickedRadio12)
	ON_BN_CLICKED(IDC_RADIO13, &CPowerHomeDlg::OnBnClickedRadio13)
	ON_BN_CLICKED(IDC_RADIO14, &CPowerHomeDlg::OnBnClickedRadio14)
	ON_BN_CLICKED(IDC_RADIO15, &CPowerHomeDlg::OnBnClickedRadio15)
	ON_BN_CLICKED(IDC_RADIO16, &CPowerHomeDlg::OnBnClickedRadio16)
	ON_BN_CLICKED(IDC_RADIO17, &CPowerHomeDlg::OnBnClickedRadio17)
	ON_BN_CLICKED(IDC_RADIO18, &CPowerHomeDlg::OnBnClickedRadio18)
	ON_BN_CLICKED(IDC_RADIO19, &CPowerHomeDlg::OnBnClickedRadio19)
	ON_BN_CLICKED(IDC_RADIO20, &CPowerHomeDlg::OnBnClickedRadio20)
	ON_BN_CLICKED(IDC_RADIO21, &CPowerHomeDlg::OnBnClickedRadio21)
	ON_BN_CLICKED(IDC_RADIO22, &CPowerHomeDlg::OnBnClickedRadio22)
	ON_BN_CLICKED(IDC_RADIO23, &CPowerHomeDlg::OnBnClickedRadio23)
	ON_BN_CLICKED(IDC_RADIO24, &CPowerHomeDlg::OnBnClickedRadio24)
	ON_BN_CLICKED(IDC_RADIO25, &CPowerHomeDlg::OnBnClickedRadio25)
	ON_BN_CLICKED(IDC_RADIO26, &CPowerHomeDlg::OnBnClickedRadio26)
	ON_BN_CLICKED(IDC_RADIO27, &CPowerHomeDlg::OnBnClickedRadio27)
	ON_BN_CLICKED(IDC_RADIO28, &CPowerHomeDlg::OnBnClickedRadio28)
	ON_BN_CLICKED(IDC_RADIO29, &CPowerHomeDlg::OnBnClickedRadio29)
	ON_BN_CLICKED(IDC_BTN_SETHOMER, &CPowerHomeDlg::OnBnClickedBtnSethomer)
	ON_BN_CLICKED(IDC_BTN_TEACH_HM, &CPowerHomeDlg::OnBnClickedBtnTeachHm)
	ON_BN_CLICKED(IDC_BTN_SAVE_HM, &CPowerHomeDlg::OnBnClickedBtnSaveHm)
	ON_BN_CLICKED(IDC_RADIO9, &CPowerHomeDlg::OnBnClickedRadio9)
	ON_BN_CLICKED(IDC_RADIO10, &CPowerHomeDlg::OnBnClickedRadio10)
	ON_BN_CLICKED(IDC_BTN_INIT_HEAD_REAROFFSET, &CPowerHomeDlg::OnBnClickedBtnInitHeadRearoffset)
	ON_BN_CLICKED(IDC_RB_FANC, &CPowerHomeDlg::OnBnClickedRbFanc)
	ON_BN_CLICKED(IDC_RB_RANC, &CPowerHomeDlg::OnBnClickedRbRanc)
	ON_BN_CLICKED(IDC_BTN_ANC_TEACH1, &CPowerHomeDlg::OnBnClickedBtnAncTeach1)
	ON_BN_CLICKED(IDC_BTN_ANC_TEACH2, &CPowerHomeDlg::OnBnClickedBtnAncTeach2)
	ON_BN_CLICKED(IDC_BTN_ANC_APPLY, &CPowerHomeDlg::OnBnClickedBtnAncApply)
	ON_BN_CLICKED(IDC_BTN_ANC_APPLY_Z, &CPowerHomeDlg::OnBnClickedBtnAncApplyZ)
	ON_BN_CLICKED(IDC_BTN_ANC_MOVE2, &CPowerHomeDlg::OnBnClickedBtnAncMove2)
	ON_BN_CLICKED(IDC_RB_ANCR0, &CPowerHomeDlg::OnBnClickedRbAncr0)
	ON_BN_CLICKED(IDC_RB_ANCR90, &CPowerHomeDlg::OnBnClickedRbAncr90)
	ON_BN_CLICKED(IDC_BTN_SETHOMEX, &CPowerHomeDlg::OnBnClickedBtnSethomex)
	ON_BN_CLICKED(IDC_BTN_R_OFFSET, &CPowerHomeDlg::OnBnClickedBtnROffset)
	ON_BN_CLICKED(IDC_BTN_SETSELFZ, &CPowerHomeDlg::OnBnClickedBtnSetselfz)
	ON_BN_CLICKED(IDC_BTN_ANC_RELOADFILE, &CPowerHomeDlg::OnBnClickedBtnAncReloadfile)
    ON_BN_CLICKED(IDC_BTN_ANC_READFILE, &CPowerHomeDlg::OnBnClickedBtnAncReadfile)
    ON_BN_CLICKED(IDC_BTN_INSERTCHECK, &CPowerHomeDlg::OnBnClickedBtnInsertcheck)
END_MESSAGE_MAP()


// CPowerHomeDlg 진단

#ifdef _DEBUG
void CPowerHomeDlg::AssertValid() const
{
	CFormView::AssertValid();
}

#ifndef _WIN32_WCE
void CPowerHomeDlg::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif
#endif //_DEBUG

// CPowerHomeDlg 메시지 처리기
void CPowerHomeDlg::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();

	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
}


BOOL CPowerHomeDlg::Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	return CFormView::Create(lpszClassName, lpszWindowName, dwStyle, rect, pParentWnd, nID, pContext);
}


void CPowerHomeDlg::AddLogBox(CString strLog)
{
}


void CPowerHomeDlg::ResetLogBox()
{
}


void CPowerHomeDlg::OnBnClickedBtn1d()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcStartCalibrationFunc)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_1D, strBtnText);
			if (strBtnText.CompareNoCase(_T("1D Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_1D_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				if (m_1DSimulOn.GetCheck() == 1) // Simulation
				{
					nSub3 = 1;
				}
				else
				{
					nSub3 = 0;
				}
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("1D End");
				SetDlgItemText(IDC_BTN_1D, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("1D End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_1D_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("1D Start");
				SetDlgItemText(IDC_BTN_1D, strBtnText);
			}
		}
	}
}


UINT CPowerHomeDlg::Homing(LPVOID wParam)
{
	CString strBtnText;
	int nID = 0;
	CPowerHomeDlg* pThis = reinterpret_cast<CPowerHomeDlg*>(wParam);
	int RepeatHome = 0;
	//double X, Y;
	//Point_XYRE res;
	if (pThis->m_ChkBtnHomeRepeat.GetCheck() == 1)
	{
		pThis->GetDlgItemText(IDC_EDIT_HOME_REPEAT_COUNT, strBtnText);
		if (strBtnText.GetLength() > 0)
		{
			RepeatHome = ConvertCStringToInt(strBtnText);
		}
		//ReliabilityInit();
		if (RepeatHome < 1)
		{
			RepeatHome = 1;
		}
		for (int indx = 0; indx < RepeatHome; ++indx)
		{
			gcHomeStatus = new CHomeStatus();
			gcHomeStatus->Run();
			strBtnText = _T("Homing...");
			pThis->SetDlgItemText(IDC_BTN_HOMING, strBtnText);
			ThreadSleep(TIME500MS);
			while (IsAllAxisHomingComplete() == false)
			{
				ThreadSleep(TIME1000MS);
			}
			TRACE(_T("[PWR] ########## Wait to All axis(%d) homing complete    ########## \n"), GetWmx3AxisCount());
			ThreadSleep(TIME500MS);
			TRACE(_T("[PWR] Get1DCompensationUse:%s\n"), Get1DCompensationUse() == true ? _T("Use") : _T("UnUse"));
			if (Get1DCompensationUse() == true)
			{
				oneDCompensationOn();
			}
			else
			{
				oneDCompensationOff();
			}
			TRACE(_T("[PWR] Get2DCompensationUse:%s\n"), Get2DCompensationUse() == true ? _T("Use") : _T("UnUse"));
			if (Get2DCompensationUse() == true)
			{
				twoDCompensationOn();
			}
			else
			{
				twoDCompensationOff();
			}
			TRACE(_T("[PWR] GetZCompensationUse:%s\n"), GetZCompensationUse() == true ? _T("Use") : _T("UnUse"));
			if (GetZCompensationUse() == true)
			{
				AllZCompensationOn(FRONT_GANTRY);
			}
			else
			{
				AllZCompensationOff(FRONT_GANTRY);
			}
			SendCameraRecognitionOffset(FRONT_GANTRY);
			gLiveOn(FHCAM);
			//res = gCatchMark(FHCAM, ALIGNMARKWHT);
			//X = ReadPosition(_T("FX"));
			//Y = ReadPosition(_T("FY1"));
			//ReliabilityRaw4(indx, X, Y, res.x, res.y);
		}
		//ReliabilityMakeStdev(_T("CatchMarkAfterHoming"), RepeatHome);
	}
	return 0;
}


void CPowerHomeDlg::OnBnClickedBtnHoming()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strLog;
		HANDLE nHandle;
		DWORD nID = NULL;
		_beginthreadex_proc_type lpStartAddress;
		lpStartAddress = (_beginthreadex_proc_type)Homing;
		nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
		strLog.Format(_T("[PWR] CPowerHomeDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("Homing"));
		gcPowerLog->Logging(strLog);
	}
}


void CPowerHomeDlg::OnBnClickedBtnCalCancel()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strBtnText;
	if (gcStartCalibrationFunc)
	{
		PowerThreadMessage* msgReceived = new PowerThreadMessage();
		CalibrationStep nStep;
		nStep = gcStartCalibrationFunc->GetStep();
		TRACE(_T("[PWR] OnBnClickedBtnCalCancel Step:%d\n"), nStep);
		if (nStep >= CalibrationStep::CAL_1D_START && nStep <= CalibrationStep::CAL_1D_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_1D_CAL_END));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("1D Start");
			SetDlgItemText(IDC_BTN_1D, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_OFFSET_CAM_START && nStep <= CalibrationStep::CAL_OFFSET_CAM_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_OFFSETCAM_CAL_END));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("HCAM Start");
			SetDlgItemText(IDC_BTN_HEADCAM_CAL, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_2D_START && nStep <= CalibrationStep::CAL_2D_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_2D_CAL_END));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("2D Start");
			SetDlgItemText(IDC_BTN_2D, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_MODULE_CAM_START && nStep <= CalibrationStep::CAL_MODULE_CAM_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_MODULECAM_CAL_END));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("Module Start");
			SetDlgItemText(IDC_BTN_2D, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_ALIGN_START && nStep <= CalibrationStep::CAL_ALIGN_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_ALIGN_OFFSET_CAL_END));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("Align Offset Start");
			SetDlgItemText(IDC_BTN_2D, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_HEADOFFSET_START && nStep <= CalibrationStep::CAL_HEADOFFSET_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_HEAD_OFFSET_CAL_START));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("Head Offset Start");
			SetDlgItemText(IDC_BTN_2D, strBtnText);
		}
		else if (nStep >= CalibrationStep::CAL_Z_START && nStep <= CalibrationStep::CAL_Z_END)
		{
			msgReceived->SetThreadMsg(_T(STRING_Z_CAL_START));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("Z Start");
			SetDlgItemText(IDC_BTN_2D, strBtnText);
		}
		else
		{
			TRACE(_T("[PWR] OnBnClickedBtnCalCancel undefine step(%d)\n"), nStep);
		}
	}
}


void CPowerHomeDlg::OnBnClickedR1dSimulon()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_1DSimulOn.SetCheck(1);
	m_1DSimulOff.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedBtn1dOnoff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strBtnText;
		GetDlgItemText(IDC_BTN_1D_ONOFF, strBtnText);
		if (strBtnText.CompareNoCase(_T("1D On")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_1D_CAL_ON));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("1D Off");
			SetDlgItemText(IDC_BTN_1D_ONOFF, strBtnText);
		}
		else if (strBtnText.CompareNoCase(_T("1D Off")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_1D_CAL_OFF));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("1D On");
			SetDlgItemText(IDC_BTN_1D_ONOFF, strBtnText);
		}
	}
}



void CPowerHomeDlg::OnBnClickedBtnHeadcamCal()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcStartCalibrationFunc)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_HEADCAM_CAL, strBtnText);
			if (strBtnText.CompareNoCase(_T("HCAM Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_OFFSETCAM_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				if (m_HdCamSimulOn.GetCheck() == 1) // Simulation
				{
					nSub3 = 1;
				}
				else
				{
					nSub3 = 0;
				}
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("HCAM End");
				SetDlgItemText(IDC_BTN_HEADCAM_CAL, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("HCAM End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_OFFSETCAM_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("HCAM Start");
				SetDlgItemText(IDC_BTN_HEADCAM_CAL, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedR1dSimuloff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_1DSimulOn.SetCheck(0);
	m_1DSimulOff.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedRhdcamSimulon()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_HdCamSimulOff.SetCheck(0);
	m_HdCamSimulOn.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedRhdcamSimuloff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_HdCamSimulOff.SetCheck(1);
	m_HdCamSimulOn.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedBtnHdcamOnoff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strBtnText;
		GetDlgItemText(IDC_BTN_HDCAM_ONOFF, strBtnText);
		if (strBtnText.CompareNoCase(_T("HCAM On")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_OFFSETCAM_CAL_ON));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("HCAM Off");
			SetDlgItemText(IDC_BTN_HDCAM_ONOFF, strBtnText);
		}
		else if (strBtnText.CompareNoCase(_T("HCAM Off")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_OFFSETCAM_CAL_OFF));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("HCAM On");
			SetDlgItemText(IDC_BTN_HDCAM_ONOFF, strBtnText);
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtn2d()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcStartCalibrationFunc)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_2D, strBtnText);
			if (strBtnText.CompareNoCase(_T("2D Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_2D_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				if (m_2DSoftware.GetCheck() == 1)
				{
					nSub2 = 1;
				}
				else
				{
					nSub2 = 0;
				}
				if (m_2DSimulOn.GetCheck() == 1) // Simulation
				{
					nSub3 = 1;
				}
				else
				{
					nSub3 = 0;
				}
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("2D End");
				SetDlgItemText(IDC_BTN_2D, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("2D End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_2D_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("2D Start");
				SetDlgItemText(IDC_BTN_2D, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtn2dOnoff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strBtnText;
		int nSub1, nSub2, nSub3;
		nSub1 = nSub2 = nSub3 = 0;
		GetDlgItemText(IDC_BTN_2D_ONOFF, strBtnText);
		if (strBtnText.CompareNoCase(_T("2D On")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_2D_CAL_ON));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			if (m_2DSoftware.GetCheck() == 1)
			{
				nSub2 = 1;
			}
			else
			{
				nSub2 = 0;
			}
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("2D Off");
			SetDlgItemText(IDC_BTN_2D_ONOFF, strBtnText);
		}
		else if (strBtnText.CompareNoCase(_T("2D Off")) == 0)
		{
			PowerThreadMessage* msgReceived = new PowerThreadMessage();
			msgReceived->SetThreadMsg(_T(STRING_2D_CAL_OFF));
			msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
			if (m_2DSoftware.GetCheck() == 1)
			{
				nSub2 = 1;
			}
			else
			{
				nSub2 = 0;
			}
			msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
			gcStartCalibrationFunc->Event((LPVOID)msgReceived);
			strBtnText = _T("2D On");
			SetDlgItemText(IDC_BTN_2D_ONOFF, strBtnText);
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnGetvisionversion()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	gcEthernetVision->VisionVersion(0);
}


void CPowerHomeDlg::OnBnClickedButton1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcPowerCalibrationData != NULL)
		{
			//Point_XYRE xyre;
			Point_XY xy;
			ZeroMemory(&xy, sizeof(xy));
			//ZeroMemory(&xyre, sizeof(xyre));
			//gcPowerCalibrationData->Set1DStartCompensationData(FRONT_GANTRY, 0.000);
			//gcPowerCalibrationData->Set1DStartCompensationData(REAR_GANTRY, 0.000);
			//gcPowerCalibrationData->Write1DCompensationData(FRONT_GANTRY);
			for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
			{
				gcPowerCalibrationData->SetHeadOffset(REAR_GANTRY, HeadNo + 1, xy);
				gcPowerCalibrationData->SetCameraRecognitionPosition(REAR_GANTRY, HeadNo + 1, xy);
				gcPowerCalibrationData->SetCameraRecognitionOffset(REAR_GANTRY, HeadNo + 1, xy);
			}
			gcPowerCalibrationData->WriteCameraRecognitionOffset(REAR_GANTRY);
		}
	}
}

void CPowerHomeDlg::OnBnClickedR2dSimulon()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_2DSimulOn.SetCheck(1);
	m_2DSimulOff.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedR2dSimuloff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_2DSimulOn.SetCheck(0);
	m_2DSimulOff.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedR2dWmx3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_2DWmx3.SetCheck(1);
	m_2DSoftware.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedR2dSw()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_2DWmx3.SetCheck(0);
	m_2DSoftware.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedButton2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	double ZeroHomeShift = 0.0;
	gcPowerCalibrationData->SetHomeShiftDistance(FRONT_GANTRY, ZeroHomeShift);
	gcPowerCalibrationData->SetHomeShiftDistance(REAR_GANTRY, ZeroHomeShift);
	gcPowerCalibrationData->WriteHomeShiftDistance(FRONT_GANTRY);
}


void CPowerHomeDlg::OnBnClickedCheckHomeRepeat()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnHomeRepeat.GetCheck() == 1)
	{
		GetDlgItem(IDC_EDIT_HOME_REPEAT_COUNT)->EnableWindow(1);
	}
	else
	{
		GetDlgItem(IDC_EDIT_HOME_REPEAT_COUNT)->EnableWindow(0);
	}
}


void CPowerHomeDlg::OnBnClickedBtnAlignOffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	CString strBtnText;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_ChkBtnAlignRepeat.GetCheck() == 1)
		{
			GetDlgItemText(IDC_EDIT_ALIGN_REPEAT_COUNT, strBtnText);
			if (strBtnText.GetLength() > 0)
			{
				nSub3 = ConvertCStringToInt(strBtnText);
			}
		}
		if (gcStartCalibrationFunc)
		{
			GetDlgItemText(IDC_BTN_ALIGN_OFFSET, strBtnText);
			if (strBtnText.CompareNoCase(_T("Align Offset Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_ALIGN_OFFSET_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Align Offset End");
				SetDlgItemText(IDC_BTN_ALIGN_OFFSET, strBtnText);
			}
			else
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_ALIGN_OFFSET_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Align Offset Start");
				SetDlgItemText(IDC_BTN_ALIGN_OFFSET, strBtnText);
			}
		}
	}
}

void CPowerHomeDlg::OnBnClickedCheckAlignRepeat()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnAlignRepeat.GetCheck() == 1)
	{
		GetDlgItem(IDC_EDIT_ALIGN_REPEAT_COUNT)->EnableWindow(1);
	}
	else
	{
		GetDlgItem(IDC_EDIT_ALIGN_REPEAT_COUNT)->EnableWindow(0);
	}
}


void CPowerHomeDlg::OnBnClickedBtnHeadOffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		int nSub1, nSub2, nSub3;
		CString strBtnText;
		nSub1 = nSub2 = nSub3 = 0;
		if (m_RbZNo1.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD1;
		}
		else if (m_RbZNo2.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD2;
		}
		else if (m_RbZNo3.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD3;
		}
		else if (m_RbZNo4.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD4;
		}
		else if (m_RbZNo5.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD5;
		}
		else if (m_RbZNo6.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD6;
		}
		else
		{
			nSub2 = NON;
		}
		if (m_ChkBtnHeadOffsetRepeat.GetCheck() == 1)
		{
			GetDlgItemText(IDC_EDIT_HEADOFFSET_REPEAT_COUNT, strBtnText);
			if (strBtnText.GetLength() > 0)
			{
				nSub3 = ConvertCStringToInt(strBtnText);
			}
		}
		if (gcStartCalibrationFunc)
		{
			GetDlgItemText(IDC_BTN_HEAD_OFFSET, strBtnText);
			if (strBtnText.CompareNoCase(_T("Head Offset Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_HEAD_OFFSET_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Head Offset End");
				SetDlgItemText(IDC_BTN_HEAD_OFFSET, strBtnText);
			}
			else
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_HEAD_OFFSET_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Head Offset Start");
				SetDlgItemText(IDC_BTN_HEAD_OFFSET, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedCheckHeadoffsetRepeat()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnHeadOffsetRepeat.GetCheck() == 1)
	{
		GetDlgItem(IDC_EDIT_HEADOFFSET_REPEAT_COUNT)->EnableWindow(1);
	}
	else
	{
		GetDlgItem(IDC_EDIT_HEADOFFSET_REPEAT_COUNT)->EnableWindow(0);
	}
}


void CPowerHomeDlg::OnBnClickedBtnSethomez()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double ZHomePosition = 0.0, CurPosition = 0.0, Diff = 0.0;
		if (m_RbZNo1.GetCheck() == 1)
		{
			strMsg = _T("FZ1");
		}
		else if (m_RbZNo2.GetCheck() == 1)
		{
			strMsg = _T("FZ2");
		}
		else if (m_RbZNo3.GetCheck() == 1)
		{
			strMsg = _T("FZ3");
		}
		else if (m_RbZNo4.GetCheck() == 1)
		{
			strMsg = _T("FZ4");
		}
		else if (m_RbZNo5.GetCheck() == 1)
		{
			strMsg = _T("FZ5");
		}
		else if (m_RbZNo6.GetCheck() == 1)
		{
			strMsg = _T("FZ6");
		}
		else
		{
			strMsg = _T("NON");
		}
		CurPosition = ReadPosition(strMsg);
		Diff = GetInsertByZ(FRONT_GANTRY) - CurPosition;
		ZHomePosition = ReadHomePosition(strMsg) + Diff;
		TRACE(_T("[PWR] %s New Origin Offset %.3f Old %.3f Diff %.3f\n"), strMsg, ZHomePosition, ReadHomePosition(strMsg), Diff);

		CString strNewHome;

		strNewHome.Format(_T("%.3f"), ZHomePosition);
		SetDlgItemText(IDC_EDIT_HOME_Z, strNewHome);
	}
}

void CPowerHomeDlg::OnBnClickedBtnSetpcbfix()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		long Gantry = FRONT_GANTRY, Conveyor = FRONT_CONV;
		if (m_RbFrontGantry.GetCheck() == 1)
		{
			Conveyor = FRONT_CONV;
		}
		else
		{
			Conveyor = REAR_CONV;
		}
		pt = gReadGantryPosition(Gantry);
		WritePcbFixPosition(Conveyor, pt);
	}
}


void CPowerHomeDlg::OnBnClickedBtnSetfdref()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		long Gantry = FRONT_GANTRY, Stage = FRONT_STAGE;
		long RefFdNo = 0;
		CString str;
		GetDlgItemText(IDC_EDIT_REF_FEEDERNO, str);
		if (str.GetLength() > 0)
		{
			RefFdNo = ConvertCStringToInt(str);
		}
		if (RefFdNo > 0 && RefFdNo < MAXFEEDERNO)
		{
			if (m_RbFrontGantry.GetCheck() == 1)
			{
				Stage = FRONT_STAGE;
			}
			else
			{
				Stage = REAR_STAGE;
			}
			pt = gReadGantryPosition(Gantry);
			WriteReferenceFeederPosition(Stage, RefFdNo, pt);
		}
		else
		{
			TRACE(_T("[PWR] INVALID Reference Feeder No(%d)\n"), RefFdNo);
		}
	}
}


void CPowerHomeDlg::OnBnClickedRbFront()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbFrontGantry.SetCheck(1);
	m_RbRearGantry.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedRbRear()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbFrontGantry.SetCheck(0);
	m_RbRearGantry.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedRbZno1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(1);
	m_RbZNo2.SetCheck(0);
	m_RbZNo3.SetCheck(0);
	m_RbZNo4.SetCheck(0);
	m_RbZNo5.SetCheck(0);
	m_RbZNo6.SetCheck(0);
	CString strMsg = _T("FZ1");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW1");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}


void CPowerHomeDlg::OnBnClickedRbZno2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(0);
	m_RbZNo2.SetCheck(1);
	m_RbZNo3.SetCheck(0);
	m_RbZNo4.SetCheck(0);
	m_RbZNo5.SetCheck(0);
	m_RbZNo6.SetCheck(0);
	CString strMsg = _T("FZ2");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW2");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}


void CPowerHomeDlg::OnBnClickedRbZno3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(0);
	m_RbZNo2.SetCheck(0);
	m_RbZNo3.SetCheck(1);
	m_RbZNo4.SetCheck(0);
	m_RbZNo5.SetCheck(0);
	m_RbZNo6.SetCheck(0);
	CString strMsg = _T("FZ3");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW3");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}


void CPowerHomeDlg::OnBnClickedRbZno4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(0);
	m_RbZNo2.SetCheck(0);
	m_RbZNo3.SetCheck(0);
	m_RbZNo4.SetCheck(1);
	m_RbZNo5.SetCheck(0);
	m_RbZNo6.SetCheck(0);
	CString strMsg = _T("FZ4");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW4");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}


void CPowerHomeDlg::OnBnClickedRbZno5()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(0);
	m_RbZNo2.SetCheck(0);
	m_RbZNo3.SetCheck(0);
	m_RbZNo4.SetCheck(0);
	m_RbZNo5.SetCheck(1);
	m_RbZNo6.SetCheck(0);
	CString strMsg = _T("FZ5");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW5");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}


void CPowerHomeDlg::OnBnClickedRbZno6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbZNo1.SetCheck(0);
	m_RbZNo2.SetCheck(0);
	m_RbZNo3.SetCheck(0);
	m_RbZNo4.SetCheck(0);
	m_RbZNo5.SetCheck(0);
	m_RbZNo6.SetCheck(1);
	CString strMsg = _T("FZ6");
	double ZHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), ZHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
	strMsg = _T("FW6");
	double RHomePosition = ReadHomePosition(strMsg);
	strMsg.Format(_T("%.3f"), RHomePosition);
	SetDlgItemText(IDC_EDIT_HOME_R, strMsg);
}

void CPowerHomeDlg::OnBnClickedBtnSendCamoffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		gUpdateVisionData(FRONT_VISION, 0, 1);
		SendCameraRecognitionOffset(FRONT_GANTRY);
	}
}


void CPowerHomeDlg::OnBnClickedBtnGopcbfix()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		double Ratio = 0.5, Inpos = 0.001;
		long Ms = TIME30MS, Gantry = FRONT_GANTRY, Target = FHCAM, Conveyor = FRONT_CONV, Err = NO_ERR;
		ULONGLONG GetTime = 0, Elapsed = 0;
		// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
		if (m_RbFrontGantry.GetCheck() == 1)
		{
			Conveyor = FRONT_CONV;
		}
		else
		{
			Conveyor = REAR_CONV;
		}
		ZeroMemory(&pt, sizeof(pt));
		pt = ReadPcbFixPosition(Conveyor);
		GetTime = _time_get();
		Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio);
		TRACE(_T("[PWR] ReadPcbFixPosition StartAllZAxisWaitMotion Elapsed,%d Err,%d\n"), _time_elapsed(GetTime), Err);
		if (Err == NO_ERR)
		{
			GetTime = _time_get();
			Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TIME5000MS);
			TRACE(_T("[PWR] ReadPcbFixPosition LinearIntplPosWaitDelayedPosSet Elapsed,%d Err,%d\n"), _time_elapsed(GetTime), Err);
			gLedOn(FHCAM, 50, 0, 0);
			gLiveOn(FHCAM);
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnGofdref()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		double Ratio = 0.5, Inpos = 0.001;
		long Ms = TIME300MS, Gantry = FRONT_GANTRY, Target = FHCAM, Stage = FRONT_STAGE, Err = NO_ERR;
		ULONGLONG GetTime = 0, Elapsed = 0;
		if (m_RbFrontGantry.GetCheck() == 1)
		{
			Stage = FRONT_STAGE;
		}
		else
		{
			Stage = REAR_STAGE;
		}
		ZeroMemory(&pt, sizeof(pt));
		pt = ReadReferenceFeederPosition(Stage);
		GetTime = _time_get();
		Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio);
		TRACE(_T("[PWR] ReadFeederReferencePosition StartAllZAxisWaitMotion Elapsed,%d Err,%d\n"), _time_elapsed(GetTime), Err);
		if (Err == NO_ERR)
		{
			GetTime = _time_get();
			Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TIME5000MS);
			TRACE(_T("[PWR] ReadFeederReferencePosition LinearIntplPosWaitDelayedPosSet Elapsed,%d Err,%d\n"), _time_elapsed(GetTime), Err);
			gLedOn(FHCAM, 50, 0, 0);
			gLiveOn(FHCAM);
		}		
	}
}

void CPowerHomeDlg::OnBnClickedBtnSetFeeder()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString str;
		long RefFdNo = 0, Gantry = FRONT_GANTRY, Stage = FRONT_STAGE;
		double Pitch = 0.0;
		Point_XY pt;
		GetDlgItemText(IDC_EDIT_REF_FEEDERNO, str);
		if (str.GetLength() > 0)
		{
			RefFdNo = ConvertCStringToInt(str);
		}
		if (RefFdNo > 0 && RefFdNo < MAXFEEDERNO)
		{
			GetDlgItemText(IDC_EDIT_FEEDER_PITCH, str);
			if (str.GetLength() > 0)
			{
				Pitch = ConvertCStringToDouble(str);
			}
			if (m_RbFrontGantry.GetCheck() == 1)
			{
				Stage = FRONT_STAGE;
			}
			else
			{
				Stage = REAR_STAGE;
			}
			pt = gReadGantryPosition(Gantry);
			WriteReferenceFeederPosition(Stage, RefFdNo, pt);
			if (abs(Pitch) > MAX_MOTION_VALID_RANGE)
			{
				Pitch = 13.0;
			}
			WriteFeederPitch(Stage, Pitch);
		}
	}
}


void CPowerHomeDlg::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CFormView::OnShowWindow(bShow, nStatus);
	double Pitch = 0.0;
	long RefFdNo = 0, Gantry = FRONT_GANTRY;
	CString str;
	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	if (bShow == 1)
	{
		if (gcPowerCalibrationData)
		{
			if (m_RbFrontGantry.GetCheck() == 1)
			{
				Gantry = FRONT_GANTRY;
			}
			else
			{
				Gantry = REAR_GANTRY;
			}
			RefFdNo = ReadReferenceFeederNo(Gantry);
			Pitch = ReadFeederPitch(Gantry);
			str.Format(_T("%d"), RefFdNo);
			SetDlgItemText(IDC_EDIT_REF_FEEDERNO, str);
			str.Format(_T("%.3f"), Pitch);
			SetDlgItemText(IDC_EDIT_FEEDER_PITCH, str);
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnSetAlignOffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		long AlignCam = CAM1;
		pt = gReadGantryPosition(FRONT_GANTRY);
		if (gcPowerCalibrationData)
		{
			gcPowerCalibrationData->SetCameraAlignPosition(FRONT_GANTRY, pt);
			gcPowerCalibrationData->SetAlignCamera(FRONT_GANTRY, AlignCam);
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnZcompen()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		int nSub1, nSub2, nSub3;
		nSub1 = nSub2 = nSub3 = 0;
		if (gcStartCalibrationFunc)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_ZCOMPEN, strBtnText);
			if (strBtnText.CompareNoCase(_T("Z Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_Z_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				nSub1 = FRONT_GANTRY;
				if (m_RbZNo1.GetCheck() == 1) nSub2 = TBL_HEAD1;
				else if (m_RbZNo2.GetCheck() == 1) nSub2 = TBL_HEAD2;
				else if (m_RbZNo3.GetCheck() == 1) nSub2 = TBL_HEAD3;
				else if (m_RbZNo4.GetCheck() == 1) nSub2 = TBL_HEAD4;
				else if (m_RbZNo5.GetCheck() == 1) nSub2 = TBL_HEAD5;
				else if (m_RbZNo6.GetCheck() == 1) nSub2 = TBL_HEAD6;



				if (m_1DSimulOn.GetCheck() == 1) // Simulation
				{
					nSub3 = 1;
				}
				else
				{
					nSub3 = 0;
				}
				msgReceived->SetThreadSubMsg(nSub1, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Z End");
				SetDlgItemText(IDC_BTN_ZCOMPEN, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("Z End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_Z_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Z Start");
				SetDlgItemText(IDC_BTN_ZCOMPEN, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnZcompenOn()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long HeadNo = NON;
		CString strAxisZ = _T("NON");
		CString strBtnText;
		GetDlgItemText(IDC_BTN_ZCOMPEN_ON, strBtnText);
		if (m_RbZNo1.GetCheck() == 1) HeadNo = TBL_HEAD1;
		else if (m_RbZNo2.GetCheck() == 1) HeadNo = TBL_HEAD2;
		else if (m_RbZNo3.GetCheck() == 1) HeadNo = TBL_HEAD3;
		else if (m_RbZNo4.GetCheck() == 1) HeadNo = TBL_HEAD4;
		else if (m_RbZNo5.GetCheck() == 1) HeadNo = TBL_HEAD5;
		else if (m_RbZNo6.GetCheck() == 1) HeadNo = TBL_HEAD6;
		strAxisZ = GetZAxisFromHeadNo(FRONT_GANTRY, HeadNo);
		CString strAxisR = GetRAxisFromHeadNo(FRONT_GANTRY, HeadNo);
		Cwmx3Axis* pAxisZ = GetWmx3AxisByName(strAxisZ);
		Cwmx3Axis* pAxisR = GetWmx3AxisByName(strAxisR);

		if (strBtnText.CompareNoCase(_T("Z On")) == 0)
		{
			strBtnText.Format(_T("Z Off"));
			SetDlgItemText(IDC_BTN_ZCOMPEN_ON, strBtnText);
			//AxisZCompensationOn(strAxisZ);

			//pAxisZ->SetAxisSkip(true);
			//pAxisR->SetAxisSkip(true);
		}
		else
		{
			strBtnText.Format(_T("Z On"));
			SetDlgItemText(IDC_BTN_ZCOMPEN_ON, strBtnText);

			//pAxisZ->SetAxisSkip(false);
			//pAxisR->SetAxisSkip(false);

			//AxisZCompensationOff(strAxisZ);
		}
	}
}



void CPowerHomeDlg::OnBnClickedBtnModulecamCal()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		int nSub1, nSub2, nSub3;
		nSub1 = nSub2 = nSub3 = 0;
		if (gcStartCalibrationFunc)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_MODULECAM_CAL, strBtnText);
			if (strBtnText.CompareNoCase(_T("Module Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_MODULECAM_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				if (m_RbCam1.GetCheck() == 1)
				{
					nSub2 = CAM1;
				}
				else if (m_RbCam2.GetCheck() == 1)
				{
					nSub2 = CAM2;
				}
				else if (m_RbCam5.GetCheck() == 1)
				{
					nSub2 = RCAM1;
				}
				else if (m_RbCam6.GetCheck() == 1)
				{
					nSub2 = RCAM2;
				}
				if (m_HdCamSimulOn.GetCheck() == 1) // Simulation
				{
					nSub3 = 1;
				}
				else
				{
					nSub3 = 0;
				}
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Module End");
				SetDlgItemText(IDC_BTN_MODULECAM_CAL, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("Module End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_MODULECAM_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, 0, 0);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("Module Start");
				SetDlgItemText(IDC_BTN_MODULECAM_CAL, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedRbCam1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbCam1.SetCheck(1);
	m_RbCam2.SetCheck(0);
	m_RbCam5.SetCheck(0);
	m_RbCam6.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedRbCam2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbCam1.SetCheck(0);
	m_RbCam2.SetCheck(1);
	m_RbCam5.SetCheck(0);
	m_RbCam6.SetCheck(0);
}



void CPowerHomeDlg::OnBnClickedBtnInitHeadOffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long Gantry = FRONT_GANTRY;
		InitializeHeadMech(Gantry);
		InitializeCamPosMech(Gantry);
		InitializeCamOffsetMech(Gantry);
		SaveHeadOffsetCamPosOffsetValue(Gantry);
		gcPowerCalibrationData->WriteHeadOffset(Gantry);
		gcPowerCalibrationData->WriteCameraRecognitionPosition(Gantry);
	}
}


void CPowerHomeDlg::OnBnClickedButton4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt;
		double Ratio = 0.5, Inpos = 0.001;
		long Ms = TIME300MS, Gantry = FRONT_GANTRY, Target = FHCAM, Err = NO_ERR;
		// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
		if (m_RbFrontGantry.GetCheck() == 1)
		{
			Gantry = FRONT_GANTRY;
		}
		else
		{
			Gantry = REAR_GANTRY;
		}
		ZeroMemory(&pt, sizeof(pt));
		pt = ReadCameraAlignPosition(Gantry);
		CApplicationTime* pTime = new CApplicationTime();
		Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio);
		TRACE(_T("[PWR] ReadCameraAlignPosition StartAllZAxisWaitMotion %d[ms] Err:%d\n"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		if (Err == NO_ERR)
		{
			Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TIME5000MS);
			TRACE(_T("[PWR] ReadCameraAlignPosition LinearIntplPosWaitDelayedPosSet %d[ms] Err:%d\n"), pTime->TimeElapsed(), Err);
		}
		delete pTime;
	}	
}

void CPowerHomeDlg::UpdateLimitInfo()
{
	long AxisNo = NON;
	Limit limit;
	CString strLimit;
	if (m_RbFX.GetCheck() == 1)
	{
		AxisNo = 0;
	}
	else if (m_RbFY1.GetCheck() == 1)
	{
		AxisNo = 1;
	}
	else if (m_RbFY2.GetCheck() == 1)
	{
		AxisNo = 2;
	}
	else if (m_RbFZ1.GetCheck() == 1)
	{
		AxisNo = 3;
	}
	else if (m_RbFZ2.GetCheck() == 1)
	{
		AxisNo = 4;
	}
	else if (m_RbFZ3.GetCheck() == 1)
	{
		AxisNo = 5;
	}
	else if (m_RbFZ4.GetCheck() == 1)
	{
		AxisNo = 6;
	}
	else if (m_RbFZ5.GetCheck() == 1)
	{
		AxisNo = 7;
	}
	else if (m_RbFZ6.GetCheck() == 1)
	{
		AxisNo = 8;
	}
	else if (m_RbFW1.GetCheck() == 1)
	{
		AxisNo = 9;
	}
	else if (m_RbFW2.GetCheck() == 1)
	{
		AxisNo = 10;
	}
	else if (m_RbFW3.GetCheck() == 1)
	{
		AxisNo = 11;
	}
	else if (m_RbFW4.GetCheck() == 1)
	{ 
		AxisNo = 12;
	}
	else if (m_RbFW5.GetCheck() == 1)
	{
		AxisNo = 13;
	}
	else if (m_RbFW6.GetCheck() == 1)
	{
		AxisNo = 14;
	}
	else if (m_RbFCONV.GetCheck() == 1)
	{
		AxisNo = 15;
	}
	else if (m_RbFPUZ.GetCheck() == 1)
	{
		AxisNo = 16;
	}
	else if (m_RbFBTIN.GetCheck() == 1)
	{
		AxisNo = 17;
	}
	else if (m_RbFBTWK.GetCheck() == 1)
	{
		AxisNo = 18;
	}
	else if (m_RbFBTOT.GetCheck() == 1)
	{
		AxisNo = 19;
	}
	limit = gcPowerCalibrationData->GetLimit(AxisNo);
	strLimit.Format(_T("%.3f"), limit.minus);
	SetDlgItemText(IDC_EDIT_LIMIT_MINUS, strLimit);
	strLimit.Format(_T("%.3f"), limit.plus);
	SetDlgItemText(IDC_EDIT_LIMIT_PLUS, strLimit);
}

void CPowerHomeDlg::SetLimitInfo(long AxisNo, Limit limit)
{
	gcPowerCalibrationData->SetLimit(AxisNo, limit);
	gcPowerCalibrationData->WriteLimit(FRONT_GANTRY);
}

void CPowerHomeDlg::UnCheckAllAxis()
{
	m_RbFX.SetCheck(0);
	m_RbFY1.SetCheck(0);
	m_RbFY2.SetCheck(0);
	m_RbFZ1.SetCheck(0);
	m_RbFZ2.SetCheck(0);
	m_RbFZ3.SetCheck(0);
	m_RbFZ4.SetCheck(0);
	m_RbFZ5.SetCheck(0);
	m_RbFZ6.SetCheck(0);
	m_RbFW1.SetCheck(0);
	m_RbFW2.SetCheck(0);
	m_RbFW3.SetCheck(0);
	m_RbFW4.SetCheck(0);
	m_RbFW5.SetCheck(0);
	m_RbFW6.SetCheck(0);
	m_RbFCONV.SetCheck(0);
	m_RbFPUZ.SetCheck(0);
	m_RbFBTIN.SetCheck(0);
	m_RbFBTWK.SetCheck(0);
	m_RbFBTOT.SetCheck(0);
}

void CPowerHomeDlg::OnBnClickedRadio1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFX.SetCheck(1);
	UpdateLimitInfo();
}

void CPowerHomeDlg::OnBnClickedBtnSetLimit()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long AxisNo = 0;
		Limit limit;
		ZeroMemory(&limit, sizeof(limit));
		CString strLimit;
		GetDlgItemText(IDC_EDIT_LIMIT_MINUS, strLimit);
		limit.minus = ConvertCStringToDouble(strLimit);
		GetDlgItemText(IDC_EDIT_LIMIT_PLUS, strLimit);
		limit.plus = ConvertCStringToDouble(strLimit);
		if (m_RbFX.GetCheck() == 1)
		{
			AxisNo = 0;
		}
		else if (m_RbFY1.GetCheck() == 1)
		{
			AxisNo = 1;
		}
		else if (m_RbFY2.GetCheck() == 1)
		{
			AxisNo = 2;
		}
		else if (m_RbFZ1.GetCheck() == 1)
		{
			AxisNo = 3;
		}
		else if (m_RbFZ2.GetCheck() == 1)
		{
			AxisNo = 4;
		}
		else if (m_RbFZ3.GetCheck() == 1)
		{
			AxisNo = 5;
		}
		else if (m_RbFZ4.GetCheck() == 1)
		{
			AxisNo = 6;
		}
		else if (m_RbFZ5.GetCheck() == 1)
		{
			AxisNo = 7;
		}
		else if (m_RbFZ6.GetCheck() == 1)
		{
			AxisNo = 8;
		}
		else if (m_RbFW1.GetCheck() == 1)
		{
			AxisNo = 9;
		}
		else if (m_RbFW2.GetCheck() == 1)
		{
			AxisNo = 10;
		}
		else if (m_RbFW3.GetCheck() == 1)
		{
			AxisNo = 11;
		}
		else if (m_RbFW4.GetCheck() == 1)
		{
			AxisNo = 12;
		}
		else if (m_RbFW5.GetCheck() == 1)
		{
			AxisNo = 13;
		}
		else if (m_RbFW6.GetCheck() == 1)
		{
			AxisNo = 14;
		}
		else if (m_RbFCONV.GetCheck() == 1)
		{
			AxisNo = 15;
		}
		else if (m_RbFPUZ.GetCheck() == 1)
		{
			AxisNo = 16;
		}
		else if (m_RbFBTIN.GetCheck() == 1)
		{
			AxisNo = 17;
		}
		else if (m_RbFBTWK.GetCheck() == 1)
		{
			AxisNo = 18;
		}
		else if (m_RbFBTOT.GetCheck() == 1)
		{
			AxisNo = 19;
		}
		SetLimitInfo(AxisNo, limit);

		Cwmx3Axis* pAxis = GetWmx3AxisByIndex(AxisNo);


		if (pAxis->IsGantryAxis() == true || pAxis->IsSlaveAxis() == true)
		{
			Config::LimitParam* limitParam = new Config::LimitParam();
			int err = ErrorCode::None;

			err = pAxis->GetHWLimitParam(limitParam);
			if (err != ErrorCode::None)
			{
				TRACE(_T("[PWR] %s GetHWLimitParam Error\n"), pAxis->GetAxisName());
				delete limitParam;
				return;
			}

			double NegLimitPulse, PosLimitPulse;
			NegLimitPulse = pAxis->GetUnResol() * limit.minus;
			PosLimitPulse = pAxis->GetUnResol() * limit.plus;

			limitParam->softLimitNegativePos = NegLimitPulse;
			limitParam->softLimitPositivePos = PosLimitPulse;

			err = pAxis->SetHWLimitParam(limitParam);
			if (err != ErrorCode::None)
			{
				TRACE(_T("[PWR] %s SetHWLimitParam Error\n"), pAxis->GetAxisName());
				delete limitParam;
				return;
			}

			err = gcWmx3Init->GetAndExportAll();
			if (err != ErrorCode::None)
			{
				TRACE(_T("[PWR] %s GetAndExportAll Error\n"), pAxis->GetAxisName());
				delete limitParam;
				return;
			}

			TRACE(_T("[PWR] %s Set WMX3 Limit Parameter N:%.3f P:%.3f\n"), pAxis->GetAxisName(), limitParam->softLimitNegativePos, limitParam->softLimitPositivePos);

		}
	}
}


void CPowerHomeDlg::OnBnClickedRadio11()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFY1.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio12()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFY2.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio13()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ1.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio14()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ2.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio15()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ3.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio16()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ4.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio17()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ5.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio18()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFZ6.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio19()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW1.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio20()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW2.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio21()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW3.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio22()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW4.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio23()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW5.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio24()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFW6.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio25()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFCONV.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio26()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFPUZ.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio27()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFBTIN.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio28()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFBTWK.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedRadio29()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UnCheckAllAxis();
	m_RbFBTOT.SetCheck(1);
	UpdateLimitInfo();
}


void CPowerHomeDlg::OnBnClickedBtnSethomer()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strAxis, strMsg;
		double RHomePosition = 0.0;
		if (m_RbZNo1.GetCheck() == 1)
		{
			strAxis = _T("FW1");
		}
		else if (m_RbZNo2.GetCheck() == 1)
		{
			strAxis = _T("FW2");
		}
		else if (m_RbZNo3.GetCheck() == 1)
		{
			strAxis = _T("FW3");
		}
		else if (m_RbZNo4.GetCheck() == 1)
		{
			strAxis = _T("FW4");
		}
		else if (m_RbZNo5.GetCheck() == 1)
		{
			strAxis = _T("FW5");
		}
		else if (m_RbZNo6.GetCheck() == 1)
		{
			strAxis = _T("FW6");
		}
		else
		{
			strAxis = _T("NON");
		}
		GetDlgItemText(IDC_EDIT_HOME_R, strMsg);
		RHomePosition = ConvertCStringToDouble(strMsg);
		TRACE(_T("[PWR] %s New Origin Offset %.3f Old %.3f\n"), strAxis, RHomePosition, ReadHomePosition(strAxis));
		WriteHomePosition(strAxis, RHomePosition);
	}
}


void CPowerHomeDlg::OnBnClickedBtnTeachHm()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		Point_XY pt, Offset;
		long Gantry = FRONT_GANTRY;
		pt = gReadGantryPosition(Gantry);
		m_HMZero = gcPowerIO->GetAnalogInput(IN_FHEAD_ZHMD_HEIGHT);

		Offset.x = pt.x - m_HMOrigin.x;
		Offset.y = pt.y - m_HMOrigin.y;

		gcPowerCalibrationData->SetHMOffset(Gantry, Offset);
		gcPowerCalibrationData->SetHMZero(Gantry, m_HMZero);


		gcPowerCalibrationData->WriteHMOffset(Gantry);

		HeightMeasurementControl(false);
		TRACE(_T("[PWR] Write Height Measurement OffsetXY,%.3f,%.3f Zero,%d\n"), Offset.x, Offset.y, m_HMZero);
	}
}


void CPowerHomeDlg::OnBnClickedBtnSaveHm()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	Point_XY pt;
	pt = gReadGantryPosition(FRONT_GANTRY);
	m_HMOrigin = pt;


	HeightMeasurementControl(true);
	TRACE(_T("[PWR] Read Height Measurement OriginXY,%.3f,%.3f Zero,%d\n"), m_HMOrigin.x, m_HMOrigin.y, m_HMZero);
}

void CPowerHomeDlg::OnBnClickedRadio9()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbCam1.SetCheck(0);
	m_RbCam2.SetCheck(0);
	m_RbCam5.SetCheck(1);
	m_RbCam6.SetCheck(0);
}


void CPowerHomeDlg::OnBnClickedRadio10()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbCam1.SetCheck(0);
	m_RbCam2.SetCheck(0);
	m_RbCam5.SetCheck(0);
	m_RbCam6.SetCheck(1);
}


void CPowerHomeDlg::OnBnClickedBtnInitHeadRearoffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long Gantry = FRONT_GANTRY;
		InitializeRearCamPosMech(Gantry);
		InitializeRearCamOffsetMech(Gantry);
		SaveHeadOffsetCamPosOffsetValue(Gantry);
		gcPowerCalibrationData->WriteCameraRecognitionPosition(Gantry);
	}
}


void CPowerHomeDlg::OnBnClickedRbFanc()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	m_RbFrontANC.SetCheck(1);
	m_RbRearANC.SetCheck(0);

	CString str;
	str.Format(_T("%.3f"), gcPowerCalibrationData->GetANCZPosition(FRONT_STAGE));
	SetDlgItemText(IDC_EDIT_ANC_Z, str);

	if (gcPowerCalibrationData->GetANCRPosition(FRONT_STAGE) == 0)
	{
		m_RbANC_R0.SetCheck(1);
		m_RbANC_R90.SetCheck(0);
	}
	else
	{
		m_RbANC_R0.SetCheck(0);
		m_RbANC_R90.SetCheck(1);
	}
}


void CPowerHomeDlg::OnBnClickedRbRanc()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	m_RbFrontANC.SetCheck(0);
	m_RbRearANC.SetCheck(1);

	CString str;
	str.Format(_T("%.3f"), gcPowerCalibrationData->GetANCZPosition(REAR_STAGE));
	SetDlgItemText(IDC_EDIT_ANC_Z, str);

	if (gcPowerCalibrationData->GetANCRPosition(REAR_STAGE) == 0)
	{
		m_RbANC_R0.SetCheck(1);
		m_RbANC_R90.SetCheck(0);
	}
	else
	{
		m_RbANC_R0.SetCheck(0);
		m_RbANC_R90.SetCheck(1);
	}
}


void CPowerHomeDlg::OnBnClickedBtnAncTeach1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	Point_XY pt;
	ANC_MARK_STRUCT data;

	if (GetRunMode() == NORMAL_MODE)
	{
		pt = gReadGantryPosition(FRONT_GANTRY);

		if (m_RbFrontANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Front ANC Mark1 XY %.3f %.3f\n"), pt.x, pt.y);

			data.No = 1;
			data.pt.x = pt.x;
			data.pt.y = pt.y;
			if (gCMachineConfig->EditANCCalMark(FRONT_STAGE, data) == NO_ERR)
			{
				if (gcMachineFileDB->GetLoadComplete() == true)
				{
					gcMachineFileDB->SaveAncCofigFromXML();
				}
			}
			else
			{
				gcPowerCalibrationData->SetANCMarkPosition(FRONT_STAGE, 0, pt);
			}
		}
		else if (m_RbRearANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Rear ANC Mark1 XY %.3f %.3f\n"), pt.x, pt.y);

			data.No = 1;
			data.pt.x = pt.x;
			data.pt.y = pt.y;
			if (gCMachineConfig->EditANCCalMark(REAR_STAGE, data) == NO_ERR)
			{
				if (gcMachineFileDB->GetLoadComplete() == true)
				{
					gcMachineFileDB->SaveAncCofigFromXML();
				}
			}
			else
			{
				gcPowerCalibrationData->SetANCMarkPosition(REAR_STAGE, 0, pt);

			}
		}
	}
}

void CPowerHomeDlg::OnBnClickedBtnAncTeach2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	Point_XY pt;
	ANC_MARK_STRUCT data;

	if (GetRunMode() == NORMAL_MODE)
	{
		pt = gReadGantryPosition(FRONT_GANTRY);

		if (m_RbFrontANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Front ANC Mark2 XY %.3f %.3f\n"), pt.x, pt.y);
			data.No = 2;
			data.pt.x = pt.x;
			data.pt.y = pt.y;
			if (gCMachineConfig->EditANCCalMark(FRONT_STAGE, data) == NO_ERR)
			{
				if (gcMachineFileDB->GetLoadComplete() == true)
				{
					gcMachineFileDB->SaveAncCofigFromXML();
				}
			}
			else
			{
				gcPowerCalibrationData->SetANCMarkPosition(FRONT_STAGE, 1, pt);

			}
		}
		else if (m_RbRearANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Rear ANC Mark2 XY %.3f %.3f\n"), pt.x, pt.y);
			data.No = 2;
			data.pt.x = pt.x;
			data.pt.y = pt.y;
			if (gCMachineConfig->EditANCCalMark(REAR_STAGE, data) == NO_ERR)
			{
				if (gcMachineFileDB->GetLoadComplete() == true)
				{
					gcMachineFileDB->SaveAncCofigFromXML();
				}
			}
			else
			{
				gcPowerCalibrationData->SetANCMarkPosition(REAR_STAGE, 1, pt);

			}
		}
	}

}

void CPowerHomeDlg::OnBnClickedBtnAncApplyZ()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString str;
	GetDlgItemText(IDC_EDIT_ANC_Z, str);

	if (GetRunMode() == NORMAL_MODE)
	{
		if (str.GetLength() > 0)
		{
			double zpos = ConvertCStringToDouble(str);

			if (m_RbFrontANC.GetCheck() == 1)
			{
				TRACE(_T("[PWR] Front ANC z %.3f Apply.\n"), zpos);

				if (gCMachineConfig->ReadFileANCCal() == NO_ERR)
				{
					ANC_STRUCT ancData = gCMachineConfig->GetCalANC(FRONT_STAGE);
					for (long idx = 0; idx < ancData.Hole.size(); idx++)
					{
						ancData.Hole.at(idx).pt.z = zpos;
					}

					gCMachineConfig->SaveFileANCCal(ancData);

					if (gcMachineFileDB->GetLoadComplete() == true)
					{
						gcMachineFileDB->SaveAncFromXML(FRONT_STAGE);
						gcPowerCalibrationData->SetANCZPosition(FRONT_STAGE, zpos);
						gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);

					}

					return;
				}

				gcPowerCalibrationData->SetANCZPosition(FRONT_STAGE, zpos);
				//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
			}
			else if (m_RbRearANC.GetCheck() == 1)
			{
				TRACE(_T("[PWR] Rear ANC z %.3f Apply.\n"), zpos);

				if (gCMachineConfig->ReadFileANCCal() == NO_ERR)
				{
					ANC_STRUCT ancData = gCMachineConfig->GetCalANC(REAR_STAGE);
					for (long idx = 0; idx < ancData.Hole.size(); idx++)
					{
						ancData.Hole.at(idx).pt.z = zpos;
					}

					gCMachineConfig->SaveFileANCCal(ancData);

					if (gcMachineFileDB->GetLoadComplete() == true)
					{
						gcMachineFileDB->SaveAncFromXML(REAR_STAGE);
						gcPowerCalibrationData->SetANCZPosition(REAR_STAGE, zpos);
						gcPowerCalibrationData->EditANCHoleRealZPosition(REAR_STAGE);

					}
					return;
				}

				gcPowerCalibrationData->SetANCZPosition(REAR_STAGE, zpos);
				//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
			}
		}
	}

}

void CPowerHomeDlg::OnBnClickedBtnAncApply()
{

	if (GetRunMode() == NORMAL_MODE && (m_RbFrontANC.GetCheck() == 1 || m_RbRearANC.GetCheck() == 1))
	{
		if (gCMachineConfig->ReadFileANCConfig() == NO_ERR && gCMachineConfig->ReadFileANCCal() == NO_ERR)
		{
			Point_XY ptMark1, ptMark2, ptRefMark;
			double AngleRadian, AngleDegree, ca, sa;
			long hole;
			long Base;
			long holeStart, holeEnd;
			Point_XYRZ calcTemp;

			ptMark1.x = ptMark1.y = ptMark2.x = ptMark2.y = ptRefMark.x = ptRefMark.y = 0.000;

			if (m_RbFrontANC.GetCheck() == 1)
			{
				Base = FRONT_STAGE;
			}
			else
			{
				Base = REAR_STAGE;
			}

			ANC_STRUCT calData = gCMachineConfig->GetCalANC(Base);
			ANC_STRUCT configData = gCMachineConfig->GetConfigANC(Base);

			for (long idx = 0; idx < calData.Mark.size(); idx++)
			{
				if (calData.Mark.at(idx).No == 1)
				{
					ptMark1 = calData.Mark.at(idx).pt;
				}
				else if (calData.Mark.at(idx).No == 2)
				{
					ptMark2 = calData.Mark.at(idx).pt;
				}
			}

			AngleRadian = atan2((ptMark2.y - ptMark1.y), (ptMark2.x - ptMark1.x));
			ptRefMark = ptMark1;

			AngleDegree = A180_PIE * AngleRadian;

			if (fabs(AngleDegree) < 0.005)
			{
				AngleRadian = 0.000;
				AngleDegree = 0.000;
			}

			ca = cos(AngleRadian);
			sa = sin(AngleRadian);

			if (Base == FRONT_STAGE)
			{
				holeStart = 1;
				holeEnd = MAX_ANC_HOLE;
			}
			else
			{
				holeStart = REAR_ANC_1ST;
				holeEnd = REAR_ANC_1ST + MAX_ANC_HOLE - 1;
			}

			for (hole = holeStart; hole <= holeEnd; hole++)
			{
				for (long idx = 0; idx < configData.Hole.size(); idx++)
				{
					if (configData.Hole.at(idx).No == hole)
					{
						calcTemp.x = ca * configData.Hole.at(idx).pt.x - sa * configData.Hole.at(idx).pt.y + ptRefMark.x;
						calcTemp.y = sa * configData.Hole.at(idx).pt.x + ca * configData.Hole.at(idx).pt.y + ptRefMark.y;
						calcTemp.r = configData.Hole.at(idx).pt.r + AngleDegree;

						for (long idx2 = 0; idx2 < calData.Hole.size(); idx2++)
						{
							if (calData.Hole.at(idx2).No == hole)
							{
								calData.Hole.at(idx2).pt.x = calcTemp.x;
								calData.Hole.at(idx2).pt.y = calcTemp.y;
								calData.Hole.at(idx2).pt.r = calcTemp.r;

								TRACE(_T("[PWR] ANC Cal Base:%d Hole:%2d XYRZ:%.3f, %.3f, %.3f, %.3f\n"),
									Base, hole,
									calData.Hole.at(idx2).pt.x, calData.Hole.at(idx2).pt.y, calData.Hole.at(idx2).pt.r, calData.Hole.at(idx2).pt.z);

								break;
							}
						}
						break;
					}
				}
			}

			gCMachineConfig->SaveFileANCCal(calData);

			if (gcMachineFileDB->GetLoadComplete() == true)
			{
				gcMachineFileDB->SaveAncFromXML(Base);
			}

			return;

		}

	}

	if (GetRunMode() == NORMAL_MODE)
	{
		gcPowerCalibrationData->InitANCHoleCadPosition();

		if (m_RbFrontANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Front ANC XYR Apply.\n"));
			gcPowerCalibrationData->CalculateANCHoleRealXYRZ(FRONT_STAGE);
			gcPowerCalibrationData->WriteANCXYRZ();
		}
		else if (m_RbRearANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Rear ANC XYR Apply.\n"));
			gcPowerCalibrationData->CalculateANCHoleRealXYRZ(REAR_STAGE);
			gcPowerCalibrationData->WriteANCXYRZ();
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnAncMove2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	Point_XYRZ pt;
	Point_XY pt2;
	CString strRAxis;

	long Gantry = FRONT_GANTRY, Target = TBL_HEAD1;

	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_RbRearANC.GetCheck() == 1)
		{
			Gantry = REAR_GANTRY;
			if (gCMachineConfig->IsExistANCData() == true)
			{
				ANC_HOLE_STRUCT holeData;

				if (gCMachineConfig->GetCalANCHole(REAR_STAGE, REAR_ANC_1ST, &holeData) == true)
				{
					pt = holeData.pt;
				}
				else
				{
					return;
				}
			}
			else
			{
				gcPowerCalibrationData->GetANCHoleRealPosition(REAR_ANC_1ST, &pt);
			}
		}
		else
		{
			Gantry = FRONT_GANTRY;
			if (gCMachineConfig->IsExistANCData() == true)
			{
				ANC_HOLE_STRUCT holeData;

				if (gCMachineConfig->GetCalANCHole(FRONT_STAGE, 1, &holeData) == true)
				{
					pt = holeData.pt;
				}
				else
				{
					return;
				}
			}
			else
			{
				gcPowerCalibrationData->GetANCHoleRealPosition(1, &pt);
			}
		}

		pt2.x = pt.x;
		pt2.y = pt.y;
		LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt2, 0.3, 0.05, TIME10MS, TIME5000MS);

		strRAxis = GetRAxisFromHeadNo(Gantry, 1);
		StartPosWaitDelayedInposition(strRAxis, 1.0, TIME5000MS, pt.r, 1.0, TIME10MS, true);
	}
}


void CPowerHomeDlg::OnBnClickedRbAncr0()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	m_RbANC_R0.SetCheck(1);
	m_RbANC_R90.SetCheck(0);

	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_RbFrontANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Front ANC R 0.0 Apply.\n"));
			gcPowerCalibrationData->SetANCRPosition(FRONT_STAGE, 0);
			//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
		}
		else if (m_RbRearANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Rear ANC R 0.0 Apply.\n"));
			gcPowerCalibrationData->SetANCRPosition(REAR_STAGE, 0);
			//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
		}

	}
}


void CPowerHomeDlg::OnBnClickedRbAncr90()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	m_RbANC_R0.SetCheck(0);
	m_RbANC_R90.SetCheck(1);

	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_RbFrontANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Front ANC R 90.0 Apply.\n"));
			gcPowerCalibrationData->SetANCRPosition(FRONT_STAGE, 1);
			//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
		}
		else if (m_RbRearANC.GetCheck() == 1)
		{
			TRACE(_T("[PWR] Rear ANC R 90.0 Apply.\n"));
			gcPowerCalibrationData->SetANCRPosition(REAR_STAGE, 1);
			//	gcPowerCalibrationData->EditANCHoleRealZPosition(FRONT_STAGE);
		}
	}
}

void CPowerHomeDlg::OnBnClickedBtnSethomex()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strOrigin, strAxis = _T("FX");
	double origin;

	GetDlgItemText(IDC_EDIT_HOME_X, strOrigin);
	origin = ConvertCStringToDouble(strOrigin);

	TRACE(_T("[PWR] %s Set Origin Offset %.3f \n"), strAxis, origin);
	WriteHomePosition(strAxis, origin);
}


void CPowerHomeDlg::OnBnClickedButton6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

}


void CPowerHomeDlg::OnBnClickedBtnROffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		int nSub1, nSub2, nSub3;
		CString strBtnText;
		nSub1 = nSub2 = nSub3 = 0;
		if (m_RbZNo1.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD1;
		}
		else if (m_RbZNo2.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD2;
		}
		else if (m_RbZNo3.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD3;
		}
		else if (m_RbZNo4.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD4;
		}
		else if (m_RbZNo5.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD5;
		}
		else if (m_RbZNo6.GetCheck() == 1)
		{
			nSub2 = TBL_HEAD6;
		}
		else
		{
			nSub2 = NON;
		}
		if (gcStartCalibrationFunc)
		{
			GetDlgItemText(IDC_BTN_R_OFFSET, strBtnText);
			if (strBtnText.CompareNoCase(_T("R Offset Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_R_CAL_START));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("R Offset End");
				SetDlgItemText(IDC_BTN_R_OFFSET, strBtnText);
			}
			else
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_R_CAL_END));
				msgReceived->SetID(gcStartCalibrationFunc->GetThreadID());
				msgReceived->SetThreadSubMsg(FRONT_GANTRY, nSub2, nSub3);
				gcStartCalibrationFunc->Event((LPVOID)msgReceived);
				strBtnText = _T("R Offset Start");
				SetDlgItemText(IDC_BTN_R_OFFSET, strBtnText);
			}
		}
	}
}


void CPowerHomeDlg::OnBnClickedBtnSetselfz()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg, strAxis;
		double ZOriginNew = 0.0, ZOriginOrg = 0.0;

		if (m_RbZNo1.GetCheck() == 1)
		{
			strAxis = _T("FZ1");
		}
		else if (m_RbZNo2.GetCheck() == 1)
		{
			strAxis = _T("FZ2");
		}
		else if (m_RbZNo3.GetCheck() == 1)
		{
			strAxis = _T("FZ3");
		}
		else if (m_RbZNo4.GetCheck() == 1)
		{
			strAxis = _T("FZ4");
		}
		else if (m_RbZNo5.GetCheck() == 1)
		{
			strAxis = _T("FZ5");
		}
		else if (m_RbZNo6.GetCheck() == 1)
		{
			strAxis = _T("FZ6");
		}
		else
		{
			strAxis = _T("NON");
		}

		GetDlgItemText(IDC_EDIT_HOME_Z, strMsg);
		ZOriginNew = ConvertCStringToDouble(strMsg);
		if (abs(ZOriginNew) > 0.0)
		{
			ZOriginOrg = ReadHomePosition(strAxis);
			TRACE(_T("[PWR] %s New Origin Offset %.3f Old %.3f Diff %.3f\n"), strAxis, ZOriginNew, ZOriginOrg, ZOriginOrg - ZOriginNew);
			WriteHomePosition(strAxis, ZOriginNew);
		}

	}
}


void CPowerHomeDlg::OnBnClickedBtnAncReloadfile()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	long Err = gCMachineConfig->ReadFileANCCal();

}


void CPowerHomeDlg::OnBnClickedBtnAncReadfile()
{
    // TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (gCMachineConfig->ReadFileANCConfig() == NO_ERR)
	{
		gCMachineConfig->ReadFileANCCal();
	}

}


void CPowerHomeDlg::OnBnClickedBtnInsertcheck()
{
    // TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnUpdateinsertoffset.GetCheck() == 1)
	{
		gcDecoding3->SetInsertOffsetUpdate(true);
	}
	else
	{
		gcDecoding3->SetInsertOffsetUpdate(false);
	}

	gcDecoding3->InsertResultAutoCheck();
	SetMachineState(STATE_IDLE);

	m_ChkBtnUpdateinsertoffset.SetCheck(0);
	gcDecoding3->SetInsertOffsetUpdate(false);
}
