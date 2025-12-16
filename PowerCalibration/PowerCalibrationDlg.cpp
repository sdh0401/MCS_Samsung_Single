#pragma once
// PowerCalibrationDlg.cpp: 구현 파일
//
#include "pch.h"
#include "framework.h"
#include "PowerCalibration.h"
#include "PowerCalibrationDlg.h"
#include "CPowerLog.h"
#include "CPowerClient.h"
#include "CPowerReliability.h"

#include "CMachineInit.h"
#include "CHomeStatus.h"

#include "CPowerHomeDlg.h"
#include "CPowerCamCalDlg.h"
#include "CPowerMasterStatusDlg.h"
#include "Cwmx3Motor.h"
#include "CPowerStackWalker.h"
#include "CPowerCalibrationData.h"
#include "CApplicationTime.h"
#include "CPowerTeachBox.h"
//#include "CPowerConveyorData.h"

#include "GlobalDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "AxisInformation.h"
#include "VisionData.h"
#include "vision.h"
//#include "ErrorCode.h"
#include "CDecoding.h"
#include "CDecoding1.h"
#include "CDecoding2.h"
#include "CDecoding3.h"
#include "CDecoding4.h"
#include "CPowerHMI.h"
#include "CAdvancedMotionFile.h"
#include "CMachineFileDB.h"
#include "CInsertEndFile.h"

#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

//#include <crtdbg.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPowerCalibrationDlg 대화 상자
CPowerCalibrationDlg::CPowerCalibrationDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_FORM_MAIN, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDI_POWER);// (IDR_MAINFRAME);
	
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	CreateAllThreadLock();
}

CPowerCalibrationDlg::~CPowerCalibrationDlg()
{
	try
	{
		m_bStopHeight = true;
		TRACE(_T("[PWR] ~CPowerCalibrationDlg Start\n"));
		m_TrayIcon.DelTrayIcon(this->m_hWnd);
		TRACE(_T("[PWR] ~CPowerCalibrationDlg DelTrayIcon\n"));
		//SendAllThreadSelfKill();
		if (gcMachineInit)
		{
			delete gcMachineInit;
		}
		TRACE(_T("[PWR] ~CPowerCalibrationDlg delete gcMachineInit done\n"));
		if (gcPowerClient)
		{
			delete gcPowerClient;
		}
		TRACE(_T("[PWR] ~CPowerCalibrationDlg delete gcPowerClient done\n"));
		delete gcPowerLog;
		TRACE(_T("[PWR] ~CPowerCalibrationDlg delete gcPowerLog done\n"));
		TRACE(_T("[PWR] ~CPowerCalibrationDlg End\n"));
	}
	catch (...)
	{
		TRACE(_T("[PWR] ~CPowerCalibrationDlg Catch\n"));
	}
}

void CPowerCalibrationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_RADIO_SOCKET, m_rBtnSocket);
	DDX_Control(pDX, IDC_RADIO_HOME, m_rBtnHome);
	DDX_Control(pDX, IDC_RADIO_CAMCAL, m_rBtnCamCal);
	DDX_Control(pDX, IDC_SIMUL_ON, m_rSimulOn);
	DDX_Control(pDX, IDC_SIMUL_OFF, m_rSimulOff);
	DDX_Control(pDX, IDC_CB_TEACH, m_cbBtnTeach);
	DDX_Control(pDX, IDC_CHK_REPEAT, m_ChkBtnRepeat);
	DDX_Control(pDX, IDC_CB_SEL_CAMERA, m_cbBtnSelCamera);
	DDX_Control(pDX, IDC_RB_WAIT, m_RbWait);
	DDX_Control(pDX, IDC_RB_WAITPOSSET, m_RbWaitPosSet);
	DDX_Control(pDX, IDC_RB_WAITDELAYEDPOSSET, m_RbWaitDelayedPosSet);
	DDX_Control(pDX, IDC_CB_RATIO, m_cbBtnRatio);
	DDX_Control(pDX, IDC_CHK_CATCH_MARK, m_ChkBtnCatchMark);
	DDX_Control(pDX, IDC_RADIO2, m_RbMarkMachCal);
	DDX_Control(pDX, IDC_RADIO3, m_RbMarkMachCalRef);
	DDX_Control(pDX, IDC_RADIO4, m_RbMarkCamRef);
	DDX_Control(pDX, IDC_RADIO5, m_RbMarkPcb);
	DDX_Control(pDX, IDC_CHK_LOG1, m_ChkBtnLog1);
	DDX_Control(pDX, IDC_CHK_LOG2, m_ChkBtnLog2);
	DDX_Control(pDX, IDC_CHK_LOG3, m_ChkBtnLog3);
	DDX_Control(pDX, IDC_CHK_LOG4, m_ChkBtnLog4);
	DDX_Control(pDX, IDC_CHK_LOG5, m_ChkBtnLog5);
	DDX_Control(pDX, IDC_CHK_INPUT_MOVE, m_ChkBtnInputMove);
	DDX_Control(pDX, IDC_RADIO6, m_RbMarkAlignHead);
	DDX_Control(pDX, IDC_RADIO7, m_RbMarkAlignModule);
	DDX_Control(pDX, IDC_CHK_LOG0, m_ChkBtnLog0);
	DDX_Control(pDX, IDC_RB_TARGET_OFFSET, m_RbHeadCam);
	DDX_Control(pDX, IDC_RB_TARGET_HEAD1, m_RbHead1);
	DDX_Control(pDX, IDC_RB_TARGET_HEAD2, m_RbHead2);
	DDX_Control(pDX, IDC_CHK_LOG6, m_ChkBtnLog6);
	DDX_Control(pDX, IDC_CHK_LOG7, m_ChkBtnLog7);
	DDX_Control(pDX, IDC_CHK_LOG8, m_ChkBtnLog8);
	DDX_Control(pDX, IDC_CHK_LOG_ALL, m_ChkBtnLogAll);
	DDX_Control(pDX, IDC_CHK_MULTI_RECOG, m_ChkMultiPartRecog);
	DDX_Control(pDX, IDC_RADIO8, m_RbMarkPcb2);
	DDX_Control(pDX, IDC_CHK_LOG9, m_ChkBtnLog9);
	DDX_Control(pDX, IDC_RB_NO1, m_SelectNo1);
	DDX_Control(pDX, IDC_RB_NO2, m_SelectNo2);
	DDX_Control(pDX, IDC_CHK_USE1D, m_ChkBtnUse1D);
	DDX_Control(pDX, IDC_CHK_USE2D, m_ChkBtnUse2D);
	DDX_Control(pDX, IDC_CHK_USEZCOMPEN, m_ChkBtnUseZCompen);
	DDX_Control(pDX, IDC_CHK_LOG10, m_ChkBtnLog10);
	DDX_Control(pDX, IDC_CHK_SKIP_MOTORPOWER, m_ChkBtnSkipMotorPower);
	DDX_Control(pDX, IDC_CHK_INIT_Y2_SHIFT, m_ChkBtnInitY2Shift);
	DDX_Control(pDX, IDC_CHK_INFINITE, m_ChkBtnInfiniteDryRun);
	DDX_Control(pDX, IDC_CHK_LOG11, m_ChkBtnLog11);
	DDX_Control(pDX, IDC_CHK_LOG12, m_ChkBtnLog12);
	DDX_Control(pDX, IDC_CHK_LOG13, m_ChkBtnLog13);
	DDX_Control(pDX, IDC_CHK_LOG14, m_ChkBtnLog14);
	DDX_Control(pDX, IDC_CHK_LOG15, m_ChkBtnLog15);
	DDX_Control(pDX, IDC_CHK_FIRSTPICKUP, m_ChkBtn_FirstPickup);
	DDX_Control(pDX, IDC_CHK_SKIP_VISION, m_ChkBtnSkipVision);
	DDX_Control(pDX, IDC_CHK_SIMUL_LOADING, m_ChkBtnSimulLoading);
	DDX_Control(pDX, IDC_CHK_USE_PATH, m_ChkBtnUsePathIntpl);
	DDX_Control(pDX, IDC_CHK_2STEP_Z, m_ChkBtnUse2StepZMotion);
	DDX_Control(pDX, IDC_CHK_LOG16, m_ChkBtnLog16);
	DDX_Control(pDX, IDC_CHK_USE_LOB, m_ChkUseLineOfBalance);
	DDX_Control(pDX, IDC_CHK_WORKEXISTSKIP, m_ChkBtnWorkExistSkip);
	DDX_Control(pDX, IDC_CHK_LOG17, m_ChkBtnLog17);
}

BEGIN_MESSAGE_MAP(CPowerCalibrationDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_MESSAGE(WM_STARTCLIENT_RECV, RecvClientMsg)
	ON_MESSAGE(WM_STARTCLIENT_SEND, SendClientMsg)
	ON_MESSAGE(WM_STARTMASTER_STATUS, StartMaster)
	ON_MESSAGE(WM_MCS_TRAYICON, OnTrayIcon)
	ON_COMMAND(WM_MCS_APP_EXIT, OnAppExit)
	ON_COMMAND(WM_MCS_DIALOG_SHOW, OnDialogShow)
	ON_BN_CLICKED(IDC_RADIO_HOME, &CPowerCalibrationDlg::OnBnClickedRadioHome)
	ON_BN_CLICKED(IDC_RADIO_CAMCAL, &CPowerCalibrationDlg::OnBnClickedRadioCamcal)
	ON_BN_CLICKED(IDC_RADIO_SOCKET, &CPowerCalibrationDlg::OnBnClickedRadioSocket)
	ON_BN_CLICKED(IDC_RADIO_MASTER, &CPowerCalibrationDlg::OnBnClickedRadioMaster)
	ON_BN_CLICKED(IDOK, &CPowerCalibrationDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_CONN_SERVER, &CPowerCalibrationDlg::OnBnClickedBtnConnServer)
	ON_BN_CLICKED(IDC_BTN_HOMING, &CPowerCalibrationDlg::OnBnClickedBtnHoming)
	ON_WM_TIMER()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_BTN_FILE, &CPowerCalibrationDlg::OnBnClickedBtnFile)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BTN_INIT, &CPowerCalibrationDlg::OnBnClickedBtnInit)
	ON_BN_CLICKED(IDC_SIMUL_ON, &CPowerCalibrationDlg::OnBnClickedSimulOn)
	ON_BN_CLICKED(IDC_SIMUL_OFF, &CPowerCalibrationDlg::OnBnClickedSimulOff)
	ON_BN_CLICKED(IDC_BTN_ESTOP, &CPowerCalibrationDlg::OnBnClickedBtnEstop)
	ON_BN_CLICKED(IDC_BTN_VISINIT, &CPowerCalibrationDlg::OnBnClickedBtnVisinit)
	ON_BN_CLICKED(IDC_BTN_VISDEMO, &CPowerCalibrationDlg::OnBnClickedBtnVisdemo)
	ON_BN_CLICKED(IDC_BTN_YPLUS, &CPowerCalibrationDlg::OnBnClickedBtnYplus)
	ON_WM_KEYDOWN()
	ON_WM_KEYUP()
	ON_BN_CLICKED(IDC_BTN_YMINUS, &CPowerCalibrationDlg::OnBnClickedBtnYminus)
	ON_BN_CLICKED(IDC_BTN_XMINUS, &CPowerCalibrationDlg::OnBnClickedBtnXminus)
	ON_BN_CLICKED(IDC_BTN_XPLUS, &CPowerCalibrationDlg::OnBnClickedBtnXplus)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_BN_CLICKED(IDC_BTN_LIVE, &CPowerCalibrationDlg::OnBnClickedBtnLive)
	ON_BN_CLICKED(IDC_BTN_MARK, &CPowerCalibrationDlg::OnBnClickedBtnMark)
	ON_CBN_SELCHANGE(IDC_CB_TEACH, &CPowerCalibrationDlg::OnCbnSelchangeCbTeach)
	ON_BN_CLICKED(IDC_BTN_SERVO, &CPowerCalibrationDlg::OnBnClickedBtnServo)
	ON_BN_CLICKED(IDC_RB_WAIT, &CPowerCalibrationDlg::OnBnClickedRbWait)
	ON_BN_CLICKED(IDC_RB_WAITPOSSET, &CPowerCalibrationDlg::OnBnClickedRbWaitposset)
	ON_BN_CLICKED(IDC_RB_WAITDELAYEDPOSSET, &CPowerCalibrationDlg::OnBnClickedRbWaitdelayedposset)
	ON_BN_CLICKED(IDC_BTN_TEACH_MINUS, &CPowerCalibrationDlg::OnBnClickedBtnTeachMinus)
	ON_BN_CLICKED(IDC_BTN_TEACH_PLUS, &CPowerCalibrationDlg::OnBnClickedBtnTeachPlus)
	ON_BN_CLICKED(IDC_BTN_CLEAR_POINT, &CPowerCalibrationDlg::OnBnClickedBtnClearPoint)
	ON_BN_CLICKED(IDC_CHK_REPEAT, &CPowerCalibrationDlg::OnBnClickedChkRepeat)
	ON_BN_CLICKED(IDC_RADIO2, &CPowerCalibrationDlg::OnBnClickedRadio2)
	ON_BN_CLICKED(IDC_RADIO3, &CPowerCalibrationDlg::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO4, &CPowerCalibrationDlg::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_RADIO5, &CPowerCalibrationDlg::OnBnClickedRadio5)
	ON_BN_CLICKED(IDC_BTN_MOVEMCAL, &CPowerCalibrationDlg::OnBnClickedBtnMovemcal)
	ON_BN_CLICKED(IDC_BTN_MOVEMCALXY, &CPowerCalibrationDlg::OnBnClickedBtnMovemcalxy)
	ON_BN_CLICKED(IDC_CHK_LOG1, &CPowerCalibrationDlg::OnBnClickedChkLog1)
	ON_BN_CLICKED(IDC_CHK_LOG2, &CPowerCalibrationDlg::OnBnClickedChkLog2)
	ON_BN_CLICKED(IDC_CHK_LOG3, &CPowerCalibrationDlg::OnBnClickedChkLog3)
	ON_BN_CLICKED(IDC_CHK_LOG4, &CPowerCalibrationDlg::OnBnClickedChkLog4)
	ON_BN_CLICKED(IDC_CHK_LOG5, &CPowerCalibrationDlg::OnBnClickedChkLog5)
	ON_BN_CLICKED(IDC_BTN_READXY, &CPowerCalibrationDlg::OnBnClickedBtnReadxy)
	ON_BN_CLICKED(IDC_BTN_MOVEMCALX, &CPowerCalibrationDlg::OnBnClickedBtnMovemcalx)
	ON_BN_CLICKED(IDC_RADIO6, &CPowerCalibrationDlg::OnBnClickedRadio6)
	ON_BN_CLICKED(IDC_RADIO7, &CPowerCalibrationDlg::OnBnClickedRadio7)
	ON_BN_CLICKED(IDC_CHK_LOG0, &CPowerCalibrationDlg::OnBnClickedChkLog0)
	ON_BN_CLICKED(IDC_BTN_CONTROL, &CPowerCalibrationDlg::OnBnClickedBtnControl)
	ON_BN_CLICKED(IDC_BTN_STOP, &CPowerCalibrationDlg::OnBnClickedBtnStop)
	ON_BN_CLICKED(IDC_BTN_1ST_XY, &CPowerCalibrationDlg::OnBnClickedBtn1stXy)
	ON_BN_CLICKED(IDC_BTN_2ND_XY, &CPowerCalibrationDlg::OnBnClickedBtn2ndXy)
	ON_BN_CLICKED(IDC_RB_TARGET_OFFSET, &CPowerCalibrationDlg::OnBnClickedRbTargetOffset)
	ON_BN_CLICKED(IDC_RB_TARGET_HEAD1, &CPowerCalibrationDlg::OnBnClickedRbTargetHead1)
	ON_BN_CLICKED(IDC_RB_TARGET_HEAD2, &CPowerCalibrationDlg::OnBnClickedRbTargetHead2)
	ON_BN_CLICKED(IDC_BTN_PART, &CPowerCalibrationDlg::OnBnClickedBtnPart)
	ON_BN_CLICKED(IDC_CHK_LOG_ALL, &CPowerCalibrationDlg::OnBnClickedChkLogAll)
	ON_BN_CLICKED(IDC_CHK_LOG6, &CPowerCalibrationDlg::OnBnClickedChkLog6)
	ON_BN_CLICKED(IDC_CHK_LOG7, &CPowerCalibrationDlg::OnBnClickedChkLog7)
	ON_BN_CLICKED(IDC_CHK_LOG8, &CPowerCalibrationDlg::OnBnClickedChkLog8)
	ON_BN_CLICKED(DC_BTN_MARK_TRAIN, &CPowerCalibrationDlg::OnBnClickedBtnMarkTrain)
	ON_BN_CLICKED(IDC_RADIO8, &CPowerCalibrationDlg::OnBnClickedRadio8)
	ON_BN_CLICKED(IDC_BTN_GO_ORIGIN, &CPowerCalibrationDlg::OnBnClickedBtnGoOrigin)
	ON_BN_CLICKED(IDC_CHK_LOG9, &CPowerCalibrationDlg::OnBnClickedChkLog9)
	ON_BN_CLICKED(IDC_RB_NO1, &CPowerCalibrationDlg::OnBnClickedRbNo1)
	ON_BN_CLICKED(IDC_RB_NO2, &CPowerCalibrationDlg::OnBnClickedRbNo2)
	ON_BN_CLICKED(IDC_BTN_ZPLUS, &CPowerCalibrationDlg::OnBnClickedBtnZplus)
	ON_BN_CLICKED(IDC_BTN_ZMINUS, &CPowerCalibrationDlg::OnBnClickedBtnZminus)
	ON_BN_CLICKED(DC_BTN_RCW, &CPowerCalibrationDlg::OnBnClickedBtnRcw)
	ON_BN_CLICKED(DC_BTN_RCCW, &CPowerCalibrationDlg::OnBnClickedBtnRccw)
	ON_BN_CLICKED(IDC_CHK_LOG10, &CPowerCalibrationDlg::OnBnClickedChkLog10)
	ON_BN_CLICKED(IDC_BTN_SUCTION, &CPowerCalibrationDlg::OnBnClickedBtnSuction)
	ON_BN_CLICKED(IDC_BTN_BLOW, &CPowerCalibrationDlg::OnBnClickedBtnBlow)
	ON_BN_CLICKED(IDC_BTN_AXISSERVO, &CPowerCalibrationDlg::OnBnClickedBtnAxisservo)
	ON_BN_CLICKED(IDC_CHK_USE2D, &CPowerCalibrationDlg::OnBnClickedChkUse2d)
	ON_BN_CLICKED(IDC_CHK_USE1D, &CPowerCalibrationDlg::OnBnClickedChkUse1d)
	ON_CBN_SELCHANGE(IDC_CB_SEL_CAMERA, &CPowerCalibrationDlg::OnCbnSelchangeCbSelCamera)
	ON_BN_CLICKED(IDC_CHK_SKIP_MOTORPOWER, &CPowerCalibrationDlg::OnBnClickedChkSkipMotorpower)
	ON_BN_CLICKED(IDC_CHK_INIT_Y2_SHIFT, &CPowerCalibrationDlg::OnBnClickedChkInitY2Shift)
	ON_BN_CLICKED(IDC_CHK_INFINITE, &CPowerCalibrationDlg::OnBnClickedChkInfinite)
	ON_BN_CLICKED(IDC_CHK_LOG11, &CPowerCalibrationDlg::OnBnClickedChkLog11)
	ON_BN_CLICKED(IDC_CHK_LOG12, &CPowerCalibrationDlg::OnBnClickedChkLog12)
	ON_BN_CLICKED(IDC_CHK_LOG13, &CPowerCalibrationDlg::OnBnClickedChkLog13)
	ON_BN_CLICKED(IDC_CHK_LOG14, &CPowerCalibrationDlg::OnBnClickedChkLog14)
	ON_BN_CLICKED(IDC_CHK_LOG15, &CPowerCalibrationDlg::OnBnClickedChkLog15)
	ON_BN_CLICKED(IDC_CHK_FIRSTPICKUP, &CPowerCalibrationDlg::OnBnClickedChkFirstpickup)
	ON_BN_CLICKED(IDC_CHK_SKIP_VISION, &CPowerCalibrationDlg::OnBnClickedChkSkipVision)
		ON_BN_CLICKED(IDC_CHK_SIMUL_LOADING, &CPowerCalibrationDlg::OnBnClickedChkSimulLoading)
		ON_BN_CLICKED(IDC_CHK_USE_PATH, &CPowerCalibrationDlg::OnBnClickedChkUsePath)
		ON_BN_CLICKED(IDC_CHK_2STEP_Z, &CPowerCalibrationDlg::OnBnClickedChk2stepZ)
		ON_BN_CLICKED(IDC_CHK_LOG16, &CPowerCalibrationDlg::OnBnClickedChkLog16)
		ON_BN_CLICKED(IDC_CHK_USE_LOB, &CPowerCalibrationDlg::OnBnClickedChkUseLob)
		ON_BN_CLICKED(IDC_CHK_WORKEXISTSKIP, &CPowerCalibrationDlg::OnBnClickedChkWorkexistskip)
		ON_BN_CLICKED(IDC_CHK_LOG17, &CPowerCalibrationDlg::OnBnClickedChkLog17)
		ON_EN_CHANGE(IDC_EDIT_LEVEL, &CPowerCalibrationDlg::OnEnChangeEditLevel)
		END_MESSAGE_MAP()


// CPowerCalibrationDlg 메시지 처리기
BOOL CPowerCalibrationDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	gcPowerLog = new CPowerLog();

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	AllocForms();
	ShowForm(PowerCalibrationNo::WIN_SOCKET);

	m_rBtnSocket.SetCheck(TRUE);
	m_rBtnHome.SetCheck(FALSE);
	m_rBtnCamCal.SetCheck(FALSE);

	m_ChkBtnUse1D.SetCheck(1);
	m_ChkBtnUse2D.SetCheck(1);
	m_ChkBtnUseZCompen.SetCheck(0);

	m_rSimulOn.SetCheck(0);
	m_rSimulOff.SetCheck(1);
	m_bSimulationMode = false;

	SetGlobalSimulationMode(m_bSimulationMode);
	GetDlgItem(IDC_BTN_INIT)->EnableWindow(true);

	m_ChkBtnCatchMark.SetCheck(1);

	m_SelectNo1.SetCheck(1);
	m_SelectNo2.SetCheck(0);

	m_RbWait.SetCheck(0);
	m_RbWaitPosSet.SetCheck(0);
	m_RbWaitDelayedPosSet.SetCheck(1);

	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(1);
	m_RbMarkPcb2.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);

	m_ChkUseLineOfBalance.SetCheck(1);
	SetUseLineOfBalance(1);

	long ChkAll = 0;
	m_ChkBtnLog0.SetCheck(ChkAll);		// Homing
	m_ChkBtnLog1.SetCheck(ChkAll);		// Motion
	m_ChkBtnLog2.SetCheck(1);		// Conveyor
	m_ChkBtnLog3.SetCheck(ChkAll);		// Run
	m_ChkBtnLog4.SetCheck(ChkAll);		// Vision
	m_ChkBtnLog5.SetCheck(ChkAll);		// Communication
	m_ChkBtnLog6.SetCheck(ChkAll);		// Calibration
	m_ChkBtnLog7.SetCheck(ChkAll);		// Pcb Sensor
	m_ChkBtnLog8.SetCheck(ChkAll);		// Serial
	m_ChkBtnLog9.SetCheck(ChkAll);		// Compensation
	m_ChkBtnLog10.SetCheck(ChkAll);		// IO
	m_ChkBtnLog11.SetCheck(ChkAll);		// ShortDist
	m_ChkBtnLog12.SetCheck(ChkAll);		// Motion Lock
	m_ChkBtnLog13.SetCheck(ChkAll);		// Tower Lamp
	m_ChkBtnLog14.SetCheck(ChkAll);		// Feeder
	m_ChkBtnLog15.SetCheck(ChkAll);		// Elapsed
	m_ChkBtnLog16.SetCheck(ChkAll);		// Insert End
	m_ChkBtnLog17.SetCheck(ChkAll);		// Analog
	m_ChkBtnLogAll.SetCheck(ChkAll);	// Everything
	SetShowLog();
	
	m_RbHeadCam.SetCheck(1);
	m_RbHead1.SetCheck(0);
	m_RbHead2.SetCheck(0);

	CString str;
	for (int indx = 0; indx < 20; ++indx)
	{
		str.Format(_T("%d"), indx + 1);
		m_cbBtnTeach.InsertString(indx, str);
	}
	m_cbBtnTeach.SetCurSel(0);

	for (int indy = 0; indy < 10; ++indy)
	{
		str.Format(_T("%d"), (indy + 1) * 10);
		m_cbBtnRatio.InsertString(indy, str);
	}
	m_cbBtnRatio.SetCurSel(4);

	str.Format(_T("Camera%d"), 1);
	m_cbBtnSelCamera.InsertString(0, str);
	str.Format(_T("Camera%d"), 2);
	m_cbBtnSelCamera.InsertString(1, str);
	str.Format(_T("Camera%d"), 5);
	m_cbBtnSelCamera.InsertString(2, str);
	str.Format(_T("Camera%d"), 6);
	m_cbBtnSelCamera.InsertString(3, str);
	str.Format(_T("Camera%d"), 9);
	m_cbBtnSelCamera.InsertString(4, str);
	m_cbBtnSelCamera.SetCurSel(4);

	str.Format(_T("50"));
	SetDlgItemText(IDC_EDIT_LED1, str);
	str.Format(_T("0"));
	SetDlgItemText(IDC_EDIT_LED2, str);
	str.Format(_T("0"));
	SetDlgItemText(IDC_EDIT_LED3, str);

	str.Format(_T("50"));
	SetDlgItemText(IDC_EDIT_CATCH_DELAY, str);

	str.Format(_T("1000"));
	SetDlgItemText(IDC_EDIT_REPEAT_DELAY, str);
	str.Format(_T("10"));
	SetDlgItemText(IDC_EDIT_REPEAT_COUNT, str);
	str.Format(_T("1"));
	SetDlgItemText(IDC_EDIT_REPEAT_MARK, str);

	str.Format(_T("0.001"));
	SetDlgItemText(IDC_EDIT_INPOS, str);
	str.Format(_T("100"));
	SetDlgItemText(IDC_EDIT_INPOSMS, str);

	str.Format(_T("0.000"));
	SetDlgItemText(IDC_EDIT_CMD_X, str);
	SetDlgItemText(IDC_EDIT_CMD_Y, str);
	SetDlgItemText(IDC_EDIT_CMD_R, str);
	SetDlgItemText(IDC_EDIT_CMD_Z, str);
	SetDlgItemText(IDC_EDIT_FEED_X, str);
	SetDlgItemText(IDC_EDIT_FEED_Y, str);
	SetDlgItemText(IDC_EDIT_FEED_R, str);
	SetDlgItemText(IDC_EDIT_FEED_Z, str);
	SetDlgItemText(IDC_EDIT_VIS_X, str);
	SetDlgItemText(IDC_EDIT_VIS_Y, str);
	SetDlgItemText(IDC_EDIT_TORQUE_X, str);
	SetDlgItemText(IDC_EDIT_TORQUE_Y, str);
	SetDlgItemText(IDC_EDIT_TORQUE_R, str);
	SetDlgItemText(IDC_EDIT_TORQUE_Z, str);
	SetDlgItemText(IDC_EDIT_2D_X, str);
	SetDlgItemText(IDC_EDIT_2D_Y, str);
	SetDlgItemText(IDC_EDIT_2D_Y2, str);
	SetDlgItemText(IDC_EDIT_1D_Y2, str);
	SetDlgItemText(IDC_EDIT_1ST_X, str);
	SetDlgItemText(IDC_EDIT_1ST_Y, str);
	SetDlgItemText(IDC_EDIT_2ND_X, str);
	SetDlgItemText(IDC_EDIT_2ND_Y, str);
	SetDlgItemText(IDC_EDIT_DIST_X, str);
	SetDlgItemText(IDC_EDIT_DIST_Y, str);

	str.Format(_T("0.3"));
	SetDlgItemText(IDC_EDIT_ROI_RATIO, str);
	SetDlgItemText(IDC_EDIT_ROI_RATIO2, str);

	m_CmdLock = CreateMutex(NULL, FALSE, NULL);

	gcPowerReliability = new CPowerReliability();

	SetInitializedMachine(false);

	StartRetryConnect();

	m_bHide = true;
	m_TrayIcon.m_bTrayHide = m_bHide;
	m_TrayIcon.AddTrayIcon(this->m_hWnd);
	ShowWindow(SW_SHOWMINIMIZED);
	PostMessage(WM_SHOWWINDOW, FALSE, SW_OTHERUNZOOM);

	if (m_ChkBtnSkipMotorPower.GetCheck() == 1)
		SetSkipMotorPower(true);
	else
		SetSkipMotorPower(false);
	if (m_ChkBtnInitY2Shift.GetCheck() == 1)
		SetInitY2Shift(true);
	else
		SetInitY2Shift(false);
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CPowerCalibrationDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else if (nID == SC_MINIMIZE)
	{
		m_bHide = true;
		m_TrayIcon.m_bTrayHide = m_bHide;
		ShowWindow(SW_SHOWMINIMIZED);
		PostMessage(WM_SHOWWINDOW, FALSE, SW_OTHERUNZOOM);
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 애플리케이션의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CPowerCalibrationDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CPowerCalibrationDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CPowerCalibrationDlg::InitializeRaw()
{
}

HANDLE CPowerCalibrationDlg::GetThreadLock()
{
	ASSERT(m_CmdLock != INVALID_HANDLE_VALUE);
	return m_CmdLock;
}

bool CPowerCalibrationDlg::Lock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_LOCK(GetThreadLock(), INFINITE);
	return true;
}

bool CPowerCalibrationDlg::Unlock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_UNLOCK(GetThreadLock());
	return true;
}

afx_msg LRESULT CPowerCalibrationDlg::StartMaster(WPARAM wParam, LPARAM lParam)
{
	//ConnectServer();
	DoInit();
	return 0;
}

afx_msg LRESULT CPowerCalibrationDlg::RecvClientMsg(WPARAM wParam, LPARAM lParam)
{
	return 0;
}

afx_msg LRESULT CPowerCalibrationDlg::SendClientMsg(WPARAM wParam, LPARAM lParam)
{
	return 0;
}

void CPowerCalibrationDlg::AllocForms()
{
	CCreateContext context;
	ZeroMemory(&context, sizeof(context));

	CRect rectOfPanelArea;
	GetDlgItem(IDC_STATIC_RECT)->GetWindowRect(&rectOfPanelArea);
	ScreenToClient(&rectOfPanelArea);

	CString strMsg;
	m_pFormHome = new CPowerHomeDlg();
	m_pFormHome->Create(NULL, NULL, WS_CHILD | WS_VSCROLL | WS_HSCROLL, rectOfPanelArea, this, IDD_FORM_HOME, &context);
	m_pFormHome->OnInitialUpdate();
	m_pFormHome->ShowWindow(SW_HIDE);
	m_pFormHome->m_1DSimulOff.SetCheck(1);
	m_pFormHome->m_1DSimulOn.SetCheck(0);
	m_pFormHome->m_HdCamSimulOff.SetCheck(1);
	m_pFormHome->m_HdCamSimulOn.SetCheck(0);
	m_pFormHome->m_2DSimulOff.SetCheck(1);
	m_pFormHome->m_2DSimulOn.SetCheck(0);
	m_pFormHome->m_2DWmx3.SetCheck(1);
	m_pFormHome->m_2DSoftware.SetCheck(0);	
	m_pFormHome->m_ChkBtnHomeRepeat.SetCheck(1);
	strMsg.Format(_T("1"));
	m_pFormHome->SetDlgItemTextW(IDC_EDIT_HOME_REPEAT_COUNT, strMsg);
	m_pFormHome->m_ChkBtnAlignRepeat.SetCheck(1);
	m_pFormHome->SetDlgItemTextW(IDC_EDIT_ALIGN_REPEAT_COUNT, strMsg);
	m_pFormHome->m_ChkBtnHeadOffsetRepeat.SetCheck(1);
	m_pFormHome->SetDlgItemTextW(IDC_EDIT_HEADOFFSET_REPEAT_COUNT, strMsg);
	m_pFormHome->m_RbFrontGantry.SetCheck(1);
	m_pFormHome->m_RbCam1.SetCheck(1);
	m_pFormHome->m_RbZNo1.SetCheck(1);
	m_pFormHome->m_RbFX.SetCheck(1);

	m_pFormCamCal = new CPowerCamCalDlg();
	m_pFormCamCal->Create(NULL, NULL, WS_CHILD | WS_VSCROLL | WS_HSCROLL, rectOfPanelArea, this, IDD_FORM_CAMCAL, &context);
	m_pFormCamCal->OnInitialUpdate();
	m_pFormCamCal->ShowWindow(SW_HIDE);
	m_pFormCamCal->m_RbPlate10.SetCheck(0);
	m_pFormCamCal->m_RbPlate1.SetCheck(1);
	m_pFormCamCal->m_RbPlate01.SetCheck(0);
	m_pFormCamCal->m_RbWidth10.SetCheck(0);
	m_pFormCamCal->m_RbWidth1.SetCheck(1);
	m_pFormCamCal->m_RbWidth01.SetCheck(0);
	m_pFormCamCal->m_RbReverseOff.SetCheck(1);
	m_pFormCamCal->m_RbReverseOn.SetCheck(0);
	strMsg.Format(_T("Empty"));
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_IN_ENT, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_IN_SET, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_WORK_LOW, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_WORK_SET, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_WORK_OUT, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_OUT_SET, strMsg);
	strMsg.Format(_T("Update First"));
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_CONVWIDTH, strMsg);
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_CONVPUSHERZ, strMsg);
	strMsg.Format(_T("1.0"));
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_PCB_THICKNESS2, strMsg);
	strMsg.Format(_T("10.0"));
	m_pFormCamCal->SetDlgItemText(IDC_EDIT_PCB_STANDBYZ_OFFSET2, strMsg);

	m_pFormMasterStatus = new CPowerMasterStatusDlg();
	m_pFormMasterStatus->Create(NULL, NULL, WS_CHILD | WS_VSCROLL | WS_HSCROLL, rectOfPanelArea, this, IDD_FORM_CAMCAL, &context);
	m_pFormMasterStatus->OnInitialUpdate();
	m_pFormMasterStatus->m_ChkBtnAfterFiducial.SetCheck(1);
	m_pFormMasterStatus->ShowWindow(SW_HIDE);
	strMsg.Format(_T("0.000"));
	m_pFormMasterStatus->GetDlgItemTextW(IDC_EDIT_RES_X, strMsg);
	m_pFormMasterStatus->GetDlgItemTextW(IDC_EDIT_RES_Y, strMsg);
	m_pFormMasterStatus->GetDlgItemTextW(IDC_EDIT_RES_R, strMsg);
	GetDlgItem(IDC_STATIC_RECT)->DestroyWindow();
}

void CPowerCalibrationDlg::AllHideForm()
{
	SocketShowWindow(SW_HIDE);
	m_pFormMasterStatus->ShowWindow(SW_HIDE);
	m_pFormHome->ShowWindow(SW_HIDE);
	m_pFormCamCal->ShowWindow(SW_HIDE);
}

void CPowerCalibrationDlg::ShowForm(PowerCalibrationNo idx)
{
	AllHideForm();
	switch (idx)
	{
	case PowerCalibrationNo::WIN_SOCKET:
		SocketShowWindow(SW_SHOW);
		break;
	case PowerCalibrationNo::WIN_MASTER:
		m_pFormMasterStatus->ShowWindow(SW_SHOW);
		break;
	case PowerCalibrationNo::WIN_HOME:
		m_pFormHome->ShowWindow(SW_SHOW);
		break;
	case PowerCalibrationNo::WIN_CAMCAL:
		m_pFormCamCal->ShowWindow(SW_SHOW);
		break;
	}
}

void CPowerCalibrationDlg::OnBnClickedRadioHome()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ShowForm(PowerCalibrationNo::WIN_HOME);
}

void CPowerCalibrationDlg::OnBnClickedRadioCamcal()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ShowForm(PowerCalibrationNo::WIN_CAMCAL);
}

void CPowerCalibrationDlg::OnBnClickedRadioSocket()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ShowForm(PowerCalibrationNo::WIN_SOCKET);
}

void CPowerCalibrationDlg::SocketShowWindow(int bShow)
{
}

void CPowerCalibrationDlg::OnBnClickedRadioMaster()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	ShowForm(PowerCalibrationNo::WIN_MASTER);
}

void CPowerCalibrationDlg::OnBnClickedOk()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDialogEx::OnOK();
}

BOOL CPowerCalibrationDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	return CDialogEx::PreTranslateMessage(pMsg);
}

void CPowerCalibrationDlg::CreateAllThreadLock()
{
	g_Wmx3AxisLock = CreateMutex(NULL, FALSE, NULL);
	g_Wmx3LinearIntplLock = CreateMutex(NULL, FALSE, NULL);
	g_Wmx3LinearIntplLockR = CreateMutex(NULL, FALSE, NULL);
	g_Wmx3LinearIntplLockZ = CreateMutex(NULL, FALSE, NULL);
	g_ThreadArrayLock = CreateMutex(NULL, FALSE, NULL);
	g_ThreadCleanArrayLock = CreateMutex(NULL, FALSE, NULL);
	g_RunStepLock = CreateMutex(NULL, FALSE, NULL);
	//for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
	//{
	//	gMOTION_LOCK[AxisNo] = CreateMutex(NULL, FALSE, NULL);
	//}
	gMACHINE_CONTROL = CreateMutex(NULL, FALSE, NULL);
	gRUN_MODE = CreateMutex(NULL, FALSE, NULL);
	gRUN_MOVEONETIME = CreateSemaphore(NULL, 0, 1, NULL);
	g_PauseMoveOneTime = false;
	g_CycleStopLock[0] = CreateSemaphore(NULL, 0, 1, NULL);
	g_CycleStopLock[1] = CreateSemaphore(NULL, 0, 1, NULL);
}

void CPowerCalibrationDlg::MasterMotionSelfOut()
{
}

void CPowerCalibrationDlg::Wmx3MotorSelfOut()
{
}

void CPowerCalibrationDlg::SlaveMotorSelfOut()
{
}

void CPowerCalibrationDlg::Wmx3IOSelfOut()
{
}

void CPowerCalibrationDlg::IOStatusSelfOut()
{
}

void CPowerCalibrationDlg::AnalogStatusSelfOut()
{
}

void CPowerCalibrationDlg::HostMessageQueueSelfOut()
{
}

void CPowerCalibrationDlg::StartCalibrationSelfOut()
{
}

void CPowerCalibrationDlg::SendAllThreadSelfKill()
{
}

void CPowerCalibrationDlg::SendAllThreadRun()
{
}

void CPowerCalibrationDlg::SendMessageToThread(string strMsg)
{
}

void CPowerCalibrationDlg::OnBnClickedBtnConnServer()
{
}

void CPowerCalibrationDlg::OnBnClickedBtnHoming()
{

}

void CPowerCalibrationDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	switch (nIDEvent)
	{
		case THREAD_MONIOTR_TIMER:
			break;
	}
	CDialogEx::OnTimer(nIDEvent);
}

int CPowerCalibrationDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialogEx::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  여기에 특수화된 작성 코드를 추가합니다.
	//SetTimer(THREAD_MONIOTR_TIMER, 1000, NULL);
	return 0;
}

afx_msg LRESULT CPowerCalibrationDlg::OnTrayIcon(WPARAM wParam, LPARAM lParam)
{
	m_TrayIcon.ProcTrayMsg(this->m_hWnd, wParam, lParam);
	return 0;
}

void CPowerCalibrationDlg::OnAppExit(void)
{
	m_TrayIcon.DelTrayIcon(this->m_hWnd);
	CDialog::OnCancel();
}

void CPowerCalibrationDlg::OnDialogShow(void)
{
	if (!m_bHide) ShowWindow(false); 
	else ShowWindow(true);
	m_bHide = !m_bHide;
	m_TrayIcon.m_bTrayHide = m_bHide;
}

void CPowerCalibrationDlg::OnDestroy()
{
	CDialogEx::OnDestroy();
	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	if (m_bHide)
	{
		NOTIFYICONDATA  nid;
		nid.cbSize = sizeof(nid);
		nid.hWnd = this->m_hWnd;
		nid.uID = 0;
		Shell_NotifyIcon(NIM_DELETE, &nid);
	}
}

void CPowerCalibrationDlg::OnBnClickedBtnInit()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		if (m_rSimulOff.GetCheck() == 1)
		{
			//////////////////////////////////////////////////////////////////////////
			//m_bSimulationMode = false;
			//m_bAutoHoming = false;
			//m_bAutoConnect = false;
			m_bSimulationMode = false;
			m_bAutoConnect = true;
			m_bAutoHoming = true;
			if (m_ChkBtnUse1D.GetCheck() == 1)
				m_bAuto1DCompen = true;
			else
				m_bAuto1DCompen = false;
			if (m_ChkBtnUse2D.GetCheck() == 1)
				m_bAuto2DCompen = true;
			else
				m_bAuto2DCompen = false;
			if (m_ChkBtnUseZCompen.GetCheck() == 1)
				m_bAutoZCompen = true;
			else
				m_bAutoZCompen = false;
			m_GantryCalibrationMethod = 0;
			m_Gantry2DMethod = 0;
			SetGlobalSimulationMode(m_bSimulationMode);
			SetAutoHomingUse(m_bAutoHoming);
			Set1DCompensationUse(m_bAuto1DCompen);
			Set2DCompensationUse(m_bAuto2DCompen);
			SetZCompensationUse(m_bAutoZCompen);
			SetY2CompensationUse(true);
			SetGantryCalibrationMethod(m_GantryCalibrationMethod);
			SetGantry2DMethod(m_Gantry2DMethod);
			//////////////////////////////////////////////////////////////////////////
			if (gcMachineInit == NULL)
			{
				gcMachineInit = new CMachineInit(m_bSimulationMode);
				PostMessage(WM_STARTMASTER_STATUS);
				strMsg = _T("Machine Exit");
				SetDlgItemText(IDC_BTN_INIT, strMsg);
			}
			else
			{
				delete gcMachineInit;
				gcMachineInit = NULL;
				strMsg = _T("Machine Initialize");
				SetDlgItemText(IDC_BTN_INIT, strMsg);
			}
		}
	}
}

void CPowerCalibrationDlg::LinearIntplPos(long Gantry, Point_XYRZE pt, double Ratio, double Inpos, long Ms)
{
	long Err = NO_ERR;
	unsigned nWait = 0;
	long Target = TBL_CAMERA;
	Point_XY Point;
	Point.x = pt.x;
	Point.y = pt.y;
	if (m_RbWaitPosSet.GetCheck() == 1) 
		nWait = 1;
	else if (m_RbWaitDelayedPosSet.GetCheck() == 1) 
		nWait = 2;
	else
		nWait = 0;
	if (pt.z < 0.000)
		pt.z = 0.000;
	if (m_RbHead1.GetCheck() == 1)
		Target = TBL_HEAD1;
	else if (m_RbHead2.GetCheck() == 1)
		Target = TBL_HEAD2;
	else
		Target = TBL_CAMERA;
	CApplicationTime* pTime = new CApplicationTime();
	if (nWait == 1)	
	{		
		Err = StartAllZAxisWaitPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, TIME5000MS);
		TRACE(_T("[PWR] StartAllZAxisWaitPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		if (Err == NO_ERR)
		{
			//Err = StartPosWaitMotion(GetAxisX(FRONT_GANTRY), Ratio, TIME5000MS, Point.x, false);
			//Err = StartPosWaitMotion(GetAxisY1(FRONT_GANTRY), Ratio, TIME5000MS, Point.y, false);
			Err = LinearIntplPosWaitPosSet(Gantry, Target, Point, Ratio, Inpos, TIME5000MS);
			TRACE(_T("[PWR] LinearIntplPosWaitPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (Err == NO_ERR)
		{
			Err = StartAllRAxisWaitPosSet(Gantry, pt.r, Ratio, Inpos, TIME5000MS);
			TRACE(_T("[PWR] StartAllRAxisWaitPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (pt.z < GetStandByZ(Gantry))
			pt.z = GetStandByZ(Gantry);
		if (Err == NO_ERR)
		{
			Err = StartAllZAxisWaitPosSet(Gantry, pt.z, Ratio, Inpos, TIME5000MS);
		}
		TRACE(_T("[PWR] StartAllZAxisWaitPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err);
	}
	else if (nWait == 2)
	{
		Err = StartAllZAxisWaitDelayedPosSet(Gantry, GetStandByZ(Gantry), Ratio, Inpos, Ms, TIME5000MS);
		TRACE(_T("[PWR] StartAllZAxisWaitDelayedPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		if (Err == NO_ERR)
		{
			//Err = StartPosWaitMotion(GetAxisX(FRONT_GANTRY), Ratio, TIME5000MS, Point.x, false);
			//Err = StartPosWaitMotion(GetAxisY1(FRONT_GANTRY), Ratio, TIME5000MS, Point.y, false);
			Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, Point, Ratio, Inpos, Ms, TIME5000MS);
			TRACE(_T("[PWR] LinearIntplPosWaitDelayedPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (Err == NO_ERR)
		{
			Err = StartAllRAxisWaitDelayedPosSet(Gantry, pt.r, Ratio, Inpos, Ms, TIME5000MS);
			TRACE(_T("[PWR] StartAllRAxisWaitDelayedPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (pt.z < GetStandByZ(Gantry))
			pt.z = GetStandByZ(Gantry);
		if (Err == NO_ERR)
		{
			Err = StartAllZAxisWaitDelayedPosSet(Gantry, pt.z, Ratio, Inpos, Ms, TIME5000MS);
		}
		TRACE(_T("[PWR] StartAllZAxisWaitDelayedPosSet %d[ms] Err:%d"), pTime->TimeElapsed(), Err);
	}
	else
	{
		Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio);
		TRACE(_T("[PWR] StartAllZAxisWaitMotion %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		if (Err == NO_ERR)
		{
			//Err = StartPosWaitMotion(GetAxisX(FRONT_GANTRY), Ratio, TIME5000MS, Point.x, false);
			//Err = StartPosWaitMotion(GetAxisY1(FRONT_GANTRY), Ratio, TIME5000MS, Point.y, false);
			Err = LinearIntplPosWaitMotion(Gantry, Target, Point, Ratio, TIME5000MS);
			TRACE(_T("[PWR] LinearIntplPosWaitMotion %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (Err == NO_ERR)
		{
			Err = StartAllRAxisWaitMotion(Gantry, pt.r, Ratio, TIME5000MS);
			TRACE(_T("[PWR] StartAllRAxisWaitMotion %d[ms] Err:%d"), pTime->TimeElapsed(), Err); pTime->TimeGet();
		}
		if (pt.z < GetStandByZ(Gantry))
			pt.z = GetStandByZ(Gantry);
		if (Err == NO_ERR)
		{
			Err = MoveZStandy(Gantry, pt.z, Ratio);
			TRACE(_T("[PWR] StartAllZAxisWaitMotion %d[ms] Err:%d"), pTime->TimeElapsed(), Err);
		}
	}	
	delete pTime;
}

void CPowerCalibrationDlg::SingleMove(long Gantry, Point_XYRZE pt)
{
	long Err = NO_ERR;
	double dx, dy;
	Err = StartMoveWaitDelayedInposition(GetAxisX(Gantry), 0.5, TIME5000MS, pt.x, 0.005, TIME100MS, true);
	Err = StartMoveWaitDelayedInposition(GetAxisY1(Gantry), 0.5, TIME5000MS, pt.y, 0.005, TIME100MS, true);
	dx = ReadCommandPosition(GetAxisX(Gantry));
	dy = ReadCommandPosition(GetAxisY1(Gantry));
	TRACE(_T("[PWR] dxy:%.3f %.3f\n"), dx - pt.x, dy - pt.y);
}

void CPowerCalibrationDlg::OnBnClickedBtnFile()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	GetDlgItemText(IDC_BTN_FILE, strMsg);
	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_ChkBtnRepeat.GetCheck() == 1)
		{
			if (strMsg.CompareNoCase(_T("Go Point")) == 0)
			{
				strMsg.Format(_T("Stop"));
				DoExecute();
				SetDlgItemText(IDC_BTN_FILE, strMsg);
			}
			else
			{
				strMsg.Format(_T("Go Point"));
				Stop();
				SetDlgItemText(IDC_BTN_FILE, strMsg);
			}
		}
		else
		{
			DoExecute();
			strMsg.Format(_T("Go Point"));
			SetDlgItemText(IDC_BTN_FILE, strMsg);
		}
	}
}

void CPowerCalibrationDlg::OnBnClickedSimulOn()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_rSimulOn.GetCheck() == 1)
	{
		GetDlgItem(IDC_BTN_INIT)->EnableWindow(false);
	}
}

void CPowerCalibrationDlg::OnBnClickedSimulOff()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_rSimulOff.GetCheck() == 1)
	{
		GetDlgItem(IDC_BTN_INIT)->EnableWindow(true);
	}
}

void CPowerCalibrationDlg::OnBnClickedBtnEstop()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strBtnText;
	int ret = NULL;
	if(IsEmergencyStop() == true)
	{
		TRACE(_T("[PWR] Emg Status\n"));
		ret = gcWmx3Motor->ReleaseEStop();
		TRACE(_T("[PWR] ReleaseEStop Ret:%d\n"), ret);
		ret = gcWmx3Motor->WaitEStopStatus(false);
		TRACE(_T("[PWR] WaitEStopStatus Ret:%d\n"), ret);
		strBtnText = _T("E-STOP");
	}
	else
	{
		TRACE(_T("[PWR] Release Emg Status\n"));
		ret = gcWmx3Motor->EStop();
		TRACE(_T("[PWR] EStop Ret:%d\n"), ret);
		ret = gcWmx3Motor->WaitEStopStatus(true);
		TRACE(_T("[PWR] WaitEStopStatus Ret:%d\n"), ret);
		strBtnText = _T("Release E-STOP");
	}
	SetDlgItemText(IDC_BTN_ESTOP, strBtnText);
}

void CPowerCalibrationDlg::OnBnClickedBtnVisinit()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long index = 0;
		index = m_cbBtnTeach.GetCurSel();
		TRACE(_T("[PWR] CB Btn Select index:%d\n"), index);
		Point_XYRZE pt;
		ZeroMemory(&pt, sizeof(pt));
		pt.x = ReadCommandPosition(_T("FX"));
		pt.y = ReadCommandPosition(_T("FY1"));
		if (m_SelectNo1.GetCheck() == 1)
			pt.r = ReadCommandPosition(_T("FW1"));
		else if (m_SelectNo2.GetCheck() == 1)
			pt.r = ReadCommandPosition(_T("FW2"));
		if (m_SelectNo1.GetCheck() == 1)
			pt.z = ReadCommandPosition(_T("FZ1"));
		else if (m_SelectNo2.GetCheck() == 1)
			pt.z = ReadCommandPosition(_T("FZ2"));
		pt.exe = 1;
		gcPowerCalibrationData->SetAgingPosition(FRONT_GANTRY, index, pt);
		gcPowerCalibrationData->WriteAgingPosition(FRONT_GANTRY);
		TRACE(_T("[PWR] Set teached(%d) XYRZ:%.3f %.3f %.3f %.3f Exe:%d\n"), index, pt.x, pt.y, pt.r, pt.z, pt.exe);
	}
}

void CPowerCalibrationDlg::OnBnClickedBtnVisdemo()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		if (m_res.exe == 1)
		{
			Point_XYRZE pt;
			pt.x = m_res.x;
			pt.y = m_res.y;
			pt.r = m_res.r;
			pt.z = m_res.z;
			TRACE(_T("[PWR] Vision ResultXYRZ:%.3f %.3f %.3f %.3f\n"), pt.x, pt.y, pt.r, pt.z);
			SingleMove(FRONT_GANTRY, pt);
			ReadGantryInformation();
		}
	}
}

void CPowerCalibrationDlg::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CDialogEx::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CPowerCalibrationDlg::OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CDialogEx::OnKeyUp(nChar, nRepCnt, nFlags);
}

void CPowerCalibrationDlg::SendToTeachBox(CString strAxis, unsigned nSub1, unsigned nSub2, unsigned nSub3)
{
}

void CPowerCalibrationDlg::OnBnClickedBtnYplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedBtnYminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CallbackHMI(HMI_CMD1ST_4, HMI_CMD2ND_50, HMI_CMD3RD_08, _T("1"));
}


void CPowerCalibrationDlg::OnBnClickedBtnXminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CallbackHMI(HMI_CMD1ST_4, HMI_CMD2ND_50, HMI_CMD3RD_08, _T("0"));


}


void CPowerCalibrationDlg::OnBnClickedBtnXplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

void CPowerCalibrationDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CDialogEx::OnLButtonDown(nFlags, point);
}


void CPowerCalibrationDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CDialogEx::OnLButtonUp(nFlags, point);
}

void CPowerCalibrationDlg::LedOn(int CameraNo, int iValue1, int iValue2, int iValue3)
{	
	gLedOn(CameraNo, iValue1, iValue2, iValue3);
}

void CPowerCalibrationDlg::LedOff(int CameraNo)
{
	gLedOn(CameraNo, 0, 0, 0);
}

void CPowerCalibrationDlg::LiveCamera(int CameraNo)
{
	gLiveOn(CameraNo);
}

void CPowerCalibrationDlg::OnBnClickedBtnLive()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		GetDlgItemText(IDC_BTN_LIVE, strMsg);
		int iCameraNo = m_cbBtnSelCamera.GetCurSel();
		if (strMsg.CompareNoCase(_T("Live On")) == 0)
		{
			GetDlgItemText(IDC_EDIT_LED1, strMsg);
			int iValue1 = ConvertCStringToInt(strMsg);
			GetDlgItemText(IDC_EDIT_LED2, strMsg);
			int iValue2 = ConvertCStringToInt(strMsg);
			GetDlgItemText(IDC_EDIT_LED3, strMsg);
			int iValue3 = ConvertCStringToInt(strMsg);
			if (iCameraNo == 0)
			{
				LedOn(CAM1, iValue1, iValue2, iValue3);
				LiveCamera(CAM1);
			}
			else if (iCameraNo == 1)
			{
				LedOn(CAM2, iValue1, iValue2, iValue3);
				LiveCamera(CAM2);
			}
			else if (iCameraNo == 2)
			{
				LedOn(CAM5, iValue1, iValue2, iValue3);
				LiveCamera(CAM5);
			}
			else if (iCameraNo == 3)
			{
				LedOn(CAM6, iValue1, iValue2, iValue3);
				LiveCamera(CAM6);
			}
			else
			{
				LedOn(FHCAM, iValue1, iValue2, iValue3);
				LiveCamera(FHCAM);
			}
			strMsg.Format(_T("Live Off"));
			SetDlgItemText(IDC_BTN_LIVE, strMsg);
		}
		else
		{
			if (iCameraNo == 0)
			{
				LedOff(CAM1);
			}
			else if (iCameraNo == 1)
			{
				LedOff(CAM2);
			}
			else if (iCameraNo == 2)
			{
				LedOff(CAM5);
			}
			else if (iCameraNo == 3)
			{
				LedOff(CAM6);
			}
			else
			{
				LedOff(FHCAM);
			}
			strMsg.Format(_T("Live On"));
			SetDlgItemText(IDC_BTN_LIVE, strMsg);
		}
	}
}

void CPowerCalibrationDlg::CatchMachCalMark(int CameraNo, long MarkNo)
{
	if (GetRunMode() == NORMAL_MODE)
	{
		m_res = gCatchMachCalMark(CameraNo, MarkNo);
		if (m_res.exe == 1)
		{
			CString strMsg;
			strMsg.Format(_T("%.3f"), m_res.x);
			SetDlgItemText(IDC_EDIT_VIS_X, strMsg);
			strMsg.Format(_T("%.3f"), m_res.y);
			SetDlgItemText(IDC_EDIT_VIS_Y, strMsg);
		}
	}
}

void CPowerCalibrationDlg::CatchMachRefMark(int CameraNo, long MarkNo)
{
	if (GetRunMode() == NORMAL_MODE)
	{
		m_res = gCatchMachRefMark(CameraNo, MarkNo);
		if (m_res.exe == 1)
		{
			CString strMsg;
			strMsg.Format(_T("%.3f"), m_res.x);
			SetDlgItemText(IDC_EDIT_VIS_X, strMsg);
			strMsg.Format(_T("%.3f"), m_res.y);
			SetDlgItemText(IDC_EDIT_VIS_Y, strMsg);
		}
	}
}

void CPowerCalibrationDlg::CatchMark(int CameraNo, long MarkNo)
{
	if (GetRunMode() == NORMAL_MODE)
	{
		m_res = gCatchMark(CameraNo, MarkNo);
		if (m_res.exe == 1)
		{
			CString strMsg;
			strMsg.Format(_T("%.3f"), m_res.x);
			SetDlgItemText(IDC_EDIT_VIS_X, strMsg);
			strMsg.Format(_T("%.3f"), m_res.y);
			SetDlgItemText(IDC_EDIT_VIS_Y, strMsg);
		}
	}
}

void CPowerCalibrationDlg::OnBnClickedBtnMark()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		HANDLE nHandle;
		DWORD nID = NULL;
		m_bStop = false;
		_beginthreadex_proc_type lpStartAddress;
		lpStartAddress = (_beginthreadex_proc_type)RepeatCatchMark;
		nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
		strLog.Format(_T("[PWR] CPowerCalibrationDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("RepeatCatchMark"));
		gcPowerLog->Logging(strLog);
	}
}

UINT CPowerCalibrationDlg::RepeatCatchMark(LPVOID wParam)
{
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	int iCameraNo = pThis->m_cbBtnSelCamera.GetCurSel();
	long MarkNo = 0;
	long RepeatMark = 0;
	double RoiRatioW = 0.3, RoiRatioH = 0.3;
	CString str;
	pThis->GetDlgItemText(IDC_EDIT_REPEAT_MARK, str);
	if (str.GetLength() > 0)
	{
		RepeatMark = ConvertCStringToInt(str);
	}
	if (pThis->m_RbMarkMachCal.GetCheck() == 1)
	{
		MarkNo = MCALMARK;
	}
	else if (pThis->m_RbMarkMachCalRef.GetCheck() == 1)
	{
		MarkNo = MACHREFMARK;
	}
	else if (pThis->m_RbMarkCamRef.GetCheck() == 1)
	{
		MarkNo = MACHREFMARK;
	}
	else if (pThis->m_RbMarkAlignHead.GetCheck() == 1)
	{
		MarkNo = ALIGNMARKWHT;
	}
	else if (pThis->m_RbMarkAlignModule.GetCheck() == 1)
	{
		MarkNo = ALIGNMARKWHTCAM2;
	}
	else if (pThis->m_RbMarkPcb.GetCheck() == 1)
	{
		MarkNo = FIDMARK0;
	}
	else if (pThis->m_RbMarkPcb2.GetCheck() == 1)
	{
		MarkNo = FIDMARK1;
	}
	pThis->GetDlgItemText(IDC_EDIT_ROI_RATIO, str);
	if (str.GetLength() > 0)
	{
		RoiRatioW = ConvertCStringToDouble(str);
	}
	pThis->GetDlgItemText(IDC_EDIT_ROI_RATIO2, str);
	if (str.GetLength() > 0)
	{
		RoiRatioH = ConvertCStringToDouble(str);
	}
	pThis->InitializeRaw();
	ReliabilityInit();
	for (long rpt = 0; rpt < RepeatMark; ++rpt)
	{
		if (iCameraNo == 0)
		{
			pThis->CatchMark(CAM1, MarkNo);
		}
		else if (iCameraNo == 1)
		{
			pThis->CatchMark(CAM2, MarkNo);
		}
		else if (iCameraNo == 2)
		{
			pThis->CatchMark(CAM5, MarkNo);
		}
		else if (iCameraNo == 3)
		{
			pThis->CatchMark(CAM6, MarkNo);
		}
		else
		{
			if (MarkNo == MCALMARK)
			{
				pThis->CatchMachCalMark(FHCAM, MarkNo);
			}
			else if (MarkNo == MACHREFMARK)
			{
				pThis->CatchMachRefMark(FHCAM, MarkNo);
			}
			else if (MarkNo >= FIDMARK0 && MarkNo <= FIDMARK19)
			{
				gLiveOn(FHCAM);
				WindowSize Win;
				Win.x1 = 320;
				Win.y1 = 240;
				Win.x2 = 960;
				Win.y2 = 720;
				markInitialize(MarkNo, RoiRatioW, RoiRatioH, 0, 0, Win);
				pThis->CatchMark(FHCAM, MarkNo);
				TRACE(_T("[PWR] CatchMark MarkNo:%d ResXYR %.3f %.3f %.3f\n"), MarkNo, pThis->m_res.x, pThis->m_res.y, pThis->m_res.r);
			}
			else
			{
				pThis->CatchMark(FHCAM, MarkNo);
			}
		}
		ReliabilityRaw2(rpt, pThis->m_res.x, pThis->m_res.y);
		pThis->ReadGantryInformation();
	}
	ReliabilityMakeStdev(_T("JustMark"), RepeatMark);
	return 0;
}

void CPowerCalibrationDlg::OnCbnSelchangeCbTeach()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

UINT CPowerCalibrationDlg::ServoAllOn(LPVOID wParam)
{
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	gServoAllOn();
	gMoveAllLastPosition();
	CString strMsg;
	strMsg = _T("Servo Off");
	pThis->SetDlgItemText(IDC_BTN_SERVO, strMsg);
	return 0;
}

UINT CPowerCalibrationDlg::ServoAllOff(LPVOID wParam)
{
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	gServoAllOff();
	CString strMsg;
	strMsg = _T("Servo On");
	pThis->SetDlgItemText(IDC_BTN_SERVO, strMsg);
	return 0;
}

void CPowerCalibrationDlg::OnBnClickedBtnServo()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		HANDLE nHandle;
		DWORD nID = NULL;
		m_bStop = false;
		_beginthreadex_proc_type lpStartAddress;
		CString strMsg;
		GetDlgItemText(IDC_BTN_SERVO, strMsg);
		if (strMsg.CompareNoCase(_T("Servo On")) == 0)
		{
			lpStartAddress = (_beginthreadex_proc_type)ServoAllOn;
		}
		else
		{
			lpStartAddress = (_beginthreadex_proc_type)ServoAllOff;
		}
		nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	}
}

void CPowerCalibrationDlg::OnBnClickedRbWait()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWait.SetCheck(1);
	m_RbWaitPosSet.SetCheck(0);
	m_RbWaitDelayedPosSet.SetCheck(0);
}


void CPowerCalibrationDlg::OnBnClickedRbWaitposset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWait.SetCheck(0);
	m_RbWaitPosSet.SetCheck(1);
	m_RbWaitDelayedPosSet.SetCheck(0);
}


void CPowerCalibrationDlg::OnBnClickedRbWaitdelayedposset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWait.SetCheck(0);
	m_RbWaitPosSet.SetCheck(0);
	m_RbWaitDelayedPosSet.SetCheck(1);
}


void CPowerCalibrationDlg::OnBnClickedBtnTeachMinus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	unsigned index = 0;
	index = m_cbBtnTeach.GetCurSel();
	if (index > 0)
		index--;
	m_cbBtnTeach.SetCurSel(index);
}

void CPowerCalibrationDlg::OnBnClickedBtnTeachPlus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	unsigned index = 0;
	index = m_cbBtnTeach.GetCurSel();
	if (index < (BUFSIZE - 1))
		index++;
	m_cbBtnTeach.SetCurSel(index);
}

void CPowerCalibrationDlg::OnBnClickedBtnClearPoint()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long index = 0;
		index = m_cbBtnTeach.GetCurSel();
		TRACE(_T("[PWR] CB Btn Select index:%d\n"), index);
		Point_XYRZE pt;
		ZeroMemory(&pt, sizeof(pt));
		pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, index);
		if ((abs(pt.x) > 0.000 && abs(pt.x) < MAX_MOTION_VALID_RANGE) && (abs(pt.y) > 0.000 && abs(pt.y) < MAX_MOTION_VALID_RANGE))
		{
			if (pt.exe == 0)
			{
				pt.exe = 1;
			}
			else
			{
				pt.exe = 0;
			}
			gcPowerCalibrationData->SetAgingPosition(FRONT_GANTRY, index, pt);
			gcPowerCalibrationData->WriteAgingPosition(FRONT_GANTRY);
		}
		TRACE(_T("[PWR] Cleared teached(%d) XYRZ:%.3f %.3f %.3f %.3f Exe:%d\n"), index, pt.x, pt.y, pt.r, pt.z, pt.exe);
	}
}

void CPowerCalibrationDlg::OnBnClickedChkRepeat()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

void CPowerCalibrationDlg::Stop()
{
	m_bStop = true;
}

void CPowerCalibrationDlg::DoExecute()
{
	if (GetRunMode() == NORMAL_MODE)
	{
		HANDLE nHandle;
		DWORD nID = NULL;
		m_bStop = false;
		_beginthreadex_proc_type lpStartAddress;
		lpStartAddress = (_beginthreadex_proc_type)Execute;
		nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
		strLog.Format(_T("[PWR] CPowerCalibrationDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("Execute"));
		gcPowerLog->Logging(strLog);
	}
}

UINT CPowerCalibrationDlg::Execute(LPVOID wParam)
{
	bool bRet = false;
	long index = 0, RepeatDelay = 0, RepeatCount = 0, InposMs = TIME10MS, TimeChk = 0;
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	index = pThis->m_cbBtnTeach.GetCurSel();
	Point_XYRZE pt;
	CString strMsg;
	double Ratio = 0.0, Inpos = 0.100;
	double TorqueX, TorqueY1, TorqueY2;
	long MarkNo = MACHREFMARK;
	CApplicationTime* pTime = new CApplicationTime();
	TorqueX = TorqueY1 = TorqueY2 = 0.000;
	pThis->GetDlgItemText(IDC_EDIT_REPEAT_DELAY, strMsg);
	if (strMsg.GetLength() > 0)
	{
		RepeatDelay = ConvertCStringToInt(strMsg);
	}
	pThis->GetDlgItemText(IDC_EDIT_REPEAT_COUNT, strMsg);
	if (strMsg.GetLength() > 0)
	{
		RepeatCount = ConvertCStringToInt(strMsg);
	}
	pThis->GetDlgItemText(IDC_CB_RATIO, strMsg);
	if (strMsg.GetLength() > 0)
	{
		Ratio = ConvertCStringToDouble(strMsg) / 100.0;
		TRACE(_T("[PWR] Window Selected Ratio:%.1f\n"), Ratio);
		if (Ratio < 0.0 || Ratio > 1.0)
		{
			TRACE(_T("[PWR] Selected Invalid Ratio:%.1f Restore 0.5\n"), Ratio);
			Ratio = 0.5;
		}
	}
	pThis->GetDlgItemText(IDC_EDIT_INPOS, strMsg);
	if (strMsg.GetLength() > 0)
	{
		Inpos = ConvertCStringToDouble(strMsg);
		TRACE(_T("[PWR] Window Selected Inpos:%.3f\n"), Inpos);
	}
	pThis->GetDlgItemText(IDC_EDIT_INPOSMS, strMsg);
	if (strMsg.GetLength() > 0)
	{
		InposMs = ConvertCStringToInt(strMsg);
		TRACE(_T("[PWR] Window Selected InposMs:%d[ms]\n"), InposMs);
	}
	ZeroMemory(&pt, sizeof(pt));
	if (pThis->m_ChkBtnRepeat.GetCheck() == 1)
	{
		for (long rpt = 0; rpt < RepeatCount; ++rpt)
		{
			for (long point = 0; point < BUFSIZE; ++point)
			{
				if (gcPowerCalibrationData != NULL)
				{
					pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, point);
					if (pt.exe == 1)
					{
						pTime->TimeGet();
						TRACE(_T("[PWR] Cmd X,Y,R,Z %.3f %.3f %.3f %.3f\n"), pt.x, pt.y, pt.r, pt.z);
						pThis->LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
						TimeChk = pTime->TimeElapsed();
						if (pThis->m_RbMarkMachCal.GetCheck() == 1)
						{
							MarkNo = MCALMARK;
						}
						else if (pThis->m_RbMarkMachCalRef.GetCheck() == 1)
						{
							MarkNo = MACHREFMARK;
						}
						else if (pThis->m_RbMarkCamRef.GetCheck() == 1)
						{
							MarkNo = MACHREFMARK;
						}
						else if (pThis->m_RbMarkPcb.GetCheck() == 1)
						{
							MarkNo = MACHREFMARK;
						}
						if (pThis->m_ChkBtnCatchMark.GetCheck() == 1)
						{
							if (MarkNo == MCALMARK)
							{
								pThis->CatchMachCalMark(FHCAM, MarkNo);
							}
							else if (MarkNo == MACHREFMARK)
							{
								pThis->CatchMachRefMark(FHCAM, MarkNo);
							}
							TorqueX = ReadActualTorque(_T("FX"));
							TorqueY1 = ReadActualTorque(_T("FY1"));
							TorqueY2 = ReadActualTorque(_T("FY2"));
							TRACE(_T("[PWR] Repeat:%d Pt:%04d VisionXY %.3f %.3f TorqueX,Y1,Y2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), 
								rpt, point, pThis->m_res.x, pThis->m_res.y,	TorqueX, TorqueY1, TorqueY2, TimeChk);
						}
						pThis->ReadGantryInformation();
						ThreadSleep(RepeatDelay);						
					}
				}
				else
				{
					TRACE(_T("[PWR] gcPowerCalibrationData is NULL\n"));
				}
			}
			if (pThis->m_bStop == true) break;
		}
		strMsg.Format(_T("Go Point"));
		pThis->SetDlgItemText(IDC_BTN_FILE, strMsg);
	}
	else
	{
		if (pThis->m_ChkBtnInputMove.GetCheck() == 1)
		{
			pThis->GetDlgItemText(IDC_EDIT_CMD_X, strMsg);
			if (strMsg.GetLength() > 0)
			{
				pt.x = ConvertCStringToDouble(strMsg);
			}
			else
			{
				pt.x = 0.0;;
			}
			pThis->GetDlgItemText(IDC_EDIT_CMD_Y, strMsg);
			if (strMsg.GetLength() > 0)
			{
				pt.y = ConvertCStringToDouble(strMsg);
			}
			else
			{
				pt.y = 0.0;
			}		
			pThis->GetDlgItemText(IDC_EDIT_CMD_R, strMsg);
			if (strMsg.GetLength() > 0)
			{
				pt.r = ConvertCStringToDouble(strMsg);
			}
			else
			{
				pt.r = 0.0;
			}
			pThis->GetDlgItemText(IDC_EDIT_CMD_Z, strMsg);
			if (strMsg.GetLength() > 0)
			{
				pt.z = ConvertCStringToDouble(strMsg);
			}
			else
			{
				pt.z = 0.0;
			}
			pTime->TimeGet();
			TRACE(_T("[PWR] Cmd X,Y,R,Z %.3f %.3f %.3f %.3f\n"), pt.x, pt.y, pt.r, pt.z);
			pThis->LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
			TimeChk = pTime->TimeElapsed();
			TorqueX = ReadActualTorque(_T("FX"));
			TorqueY1 = ReadActualTorque(_T("FY1"));
			TorqueY2 = ReadActualTorque(_T("FY2"));
			TRACE(_T("[PWR] TorqueX,Y1,Y2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), TorqueX, TorqueY1, TorqueY2, TimeChk);
			pThis->ReadGantryInformation();
		}
		else
		{
			if (gcPowerCalibrationData != NULL)
			{
				pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, index);
				TRACE(_T("[PWR] pt.exe:%d X,Y,R,Z %.3f %.3f %.3f %.3f\n"), pt.exe, pt.x, pt.y, pt.r, pt.z);
				if (pt.exe == 1)
				{
					pTime->TimeGet();
					TRACE(_T("[PWR] Cmd X,Y,R,Z %.3f %.3f %.3f %.3f\n"), pt.x, pt.y, pt.r, pt.z);
					pThis->LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
					TimeChk = pTime->TimeElapsed();
					TorqueX = ReadActualTorque(_T("FX"));
					TorqueY1 = ReadActualTorque(_T("FY1"));
					TorqueY2 = ReadActualTorque(_T("FY2"));
					TRACE(_T("[PWR] Torque X,Y1,Y2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), TorqueX, TorqueY1, TorqueY2, TimeChk);
					pThis->ReadGantryInformation();
				}
			}
			else
			{
				TRACE(_T("[PWR] gcPowerCalibrationData is NULL\n"));
			}
		}
	}
	delete pTime;
	return 0;
}

void CPowerCalibrationDlg::DoInit()
{
	HANDLE nHandle;
	DWORD nID = NULL;
	_beginthreadex_proc_type lpStartAddress;
	lpStartAddress = (_beginthreadex_proc_type)Initialize;
	nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	strLog.Format(_T("[PWR] CPowerCalibrationDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("Initialize"));
	gcPowerLog->Logging(strLog);
}

UINT CPowerCalibrationDlg::Initialize(LPVOID wParam)
{
	return 0;
}

void CPowerCalibrationDlg::OnBnClickedRadio2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(1);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkPcb2.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}

void CPowerCalibrationDlg::OnBnClickedRadio3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(1);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkPcb2.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}

void CPowerCalibrationDlg::OnBnClickedRadio4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(1);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkPcb2.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}

void CPowerCalibrationDlg::OnBnClickedRadio5()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(1);
	m_RbMarkPcb2.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}

void CPowerCalibrationDlg::OnBnClickedBtnMovemcal()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long index = 0, InposMs = TIME10MS;
		double Ratio = 0.1, Inpos = 0.005;
		CString strMsg;
		long MarkNo = MCALMARK, TimeChk = 0, CatchDelay = 0;
		CApplicationTime* pTime = new CApplicationTime();
		if (m_RbMarkMachCal.GetCheck() == 1)
		{
			MarkNo = MCALMARK;
		}
		else if (m_RbMarkMachCalRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkCamRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkPcb.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		GetDlgItemText(IDC_CB_RATIO, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Ratio = ConvertCStringToDouble(strMsg) / 100.0;
			TRACE(_T("[PWR] Selected Ratio:%.1f\n"), Ratio);
			if (Ratio < 0.0 || Ratio > 1.0)
			{
				TRACE(_T("[PWR] Selected Invalid Ratio:%.1f Restore 0.5\n"), Ratio);
				Ratio = 0.5;
			}
		}
		GetDlgItemText(IDC_EDIT_INPOS, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Inpos = ConvertCStringToDouble(strMsg);
			TRACE(_T("[PWR] Selected Inpos:%.3f\n"), Inpos);
		}
		GetDlgItemText(IDC_EDIT_CATCH_DELAY, strMsg);
		if (strMsg.GetLength() > 0)
		{
			CatchDelay = ConvertCStringToInt(strMsg);
		}
		index = m_cbBtnTeach.GetCurSel();
		Point_XYRZE pt;
		double TorqueX, TorqueY1, TorqueY2;
		TorqueX = TorqueY1 = TorqueY2 = 0.000;
		ZeroMemory(&pt, sizeof(pt));
		if (gcPowerCalibrationData != NULL)
		{
			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, index);
			TRACE(_T("[PWR] Get Point(%d) XYRZ %.3f %.3f %.3f %.3f Exe:%d\n"), index, pt.x, pt.y, pt.r, pt.z, pt.exe);
			if (pt.exe == 1)
			{
				pt.y = pt.y + CAL_2D_MAX_YPOS;
				pTime->TimeGet();
				LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
				TimeChk = pTime->TimeElapsed();
				if (m_ChkBtnCatchMark.GetCheck() == 1)
				{
					ThreadSleep(CatchDelay);
					if (MarkNo == MCALMARK)
					{
						CatchMachCalMark(FHCAM, MarkNo);
					}
					else if (MarkNo == MACHREFMARK)
					{
						CatchMachRefMark(FHCAM, MarkNo);
					}
					TorqueX = ReadActualTorque(_T("FX"));
					TorqueY1 = ReadActualTorque(_T("FY1"));
					TorqueY2 = ReadActualTorque(_T("FY2"));
					TRACE(_T("[PWR] VisionXY %.3f %.3f TorqueX,Y1,Y2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), m_res.x, m_res.y,
						TorqueX, TorqueY1, TorqueY2, TimeChk);
				}
			}
		}
		delete pTime;
	}
}

void CPowerCalibrationDlg::OnBnClickedBtnMovemcalxy()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long index = 0, InposMs = TIME10MS;
		double Ratio = 0.1, Inpos = 0.005;
		CString strMsg;
		long MarkNo = MCALMARK, TimeChk = 0, CatchDelay = 0;
		CApplicationTime* pTime = new CApplicationTime();
		if (m_RbMarkMachCal.GetCheck() == 1)
		{
			MarkNo = MCALMARK;
		}
		else if (m_RbMarkMachCalRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkCamRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkPcb.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		GetDlgItemText(IDC_CB_RATIO, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Ratio = ConvertCStringToDouble(strMsg) / 100.0;
			TRACE(_T("[PWR] Selected Ratio:%.1f\n"), Ratio);
			if (Ratio < 0.0 || Ratio > 1.0)
			{
				TRACE(_T("[PWR] Selected Invalid Ratio:%.1f Restore 0.5\n"), Ratio);
				Ratio = 0.5;
			}
		}
		GetDlgItemText(IDC_EDIT_INPOS, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Inpos = ConvertCStringToDouble(strMsg);
			TRACE(_T("[PWR] Selected Inpos:%.3f\n"), Inpos);
		}
		GetDlgItemText(IDC_EDIT_CATCH_DELAY, strMsg);
		if (strMsg.GetLength() > 0)
		{
			CatchDelay = ConvertCStringToInt(strMsg);
		}
		index = m_cbBtnTeach.GetCurSel();
		Point_XYRZE pt;
		double TorqueX, TorqueY1, TorqueY2;
		TorqueX = TorqueY1 = TorqueY2 = 0.000;
		ZeroMemory(&pt, sizeof(pt));
		if (gcPowerCalibrationData != NULL)
		{
			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, index);
			TRACE(_T("[PWR] Get Point(%d) XYRZ %.3f %.3f %.3f %.3f Exe:%d\n"), index, pt.x, pt.y, pt.r, pt.z, pt.exe);
			if (pt.exe == 1)
			{
				pt.x = pt.x + CAL_2D_MAX_XPOS;
				pt.y = pt.y + CAL_2D_MAX_YPOS;
				pTime->TimeGet();
				LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
				TimeChk = pTime->TimeElapsed();
				if (m_ChkBtnCatchMark.GetCheck() == 1)
				{
					ThreadSleep(CatchDelay);
					if (MarkNo == MCALMARK)
					{
						CatchMachCalMark(FHCAM, MarkNo);
					}
					else if (MarkNo == MACHREFMARK)
					{
						CatchMachRefMark(FHCAM, MarkNo);
					}
					TorqueX = ReadActualTorque(_T("FX"));
					TorqueY1 = ReadActualTorque(_T("FY1"));
					TorqueY2 = ReadActualTorque(_T("FY2"));
					TRACE(_T("[PWR] VisionXY %.3f %.3f TorqueX,Y1,Y2,R2,Z2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), m_res.x, m_res.y, TorqueX, TorqueY1, TorqueY2, TimeChk);
				}
			}
		}
		delete pTime;
	}
}

void CPowerCalibrationDlg::SetShowLog()
{
	long m_ChkSum = 0;
	if (m_ChkBtnLog0.GetCheck() == 1)
	{
		gcPowerLog->SetShowHomingLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowHomingLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}
	if (m_ChkBtnLog1.GetCheck() == 1)
	{
		gcPowerLog->SetShowMotionLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowMotionLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog2.GetCheck() == 1)
	{
		gcPowerLog->SetShowConveyorLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowConveyorLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog3.GetCheck() == 1)
	{
		gcPowerLog->SetShowRunLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowRunLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog4.GetCheck() == 1)
	{
		gcPowerLog->SetShowVisionLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowVisionLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog5.GetCheck() == 1)
	{
		gcPowerLog->SetShowCommunicationLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowCommunicationLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog6.GetCheck() == 1)
	{
		gcPowerLog->SetShowCalibrationLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowCalibrationLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog7.GetCheck() == 1)
	{
		gcPowerLog->SetShowPcbSensorLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowPcbSensorLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog8.GetCheck() == 1)
	{
		gcPowerLog->SetShowSerialLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowSerialLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog9.GetCheck() == 1)
	{
		gcPowerLog->SetShowCompensationLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowCompensationLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog10.GetCheck() == 1)
	{
		gcPowerLog->SetShowIoLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowIoLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog11.GetCheck() == 1)
	{
		gcPowerLog->SetShowShortDistLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowShortDistLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog12.GetCheck() == 1)
	{
		gcPowerLog->SetShowMotionLockLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowMotionLockLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog13.GetCheck() == 1)
	{
		gcPowerLog->SetShowTowerLampLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowTowerLampLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog14.GetCheck() == 1)
	{
		gcPowerLog->SetShowFeederLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowFeederLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog15.GetCheck() == 1)
	{
		gcPowerLog->SetShowElapsedLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowElapsedLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog16.GetCheck() == 1)
	{
		gcPowerLog->SetShowInsertEndLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowInsertEndLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkBtnLog17.GetCheck() == 1)
	{
		gcPowerLog->SetShowAnalogLog(true);
		m_ChkSum++;
	}
	else
	{
		gcPowerLog->SetShowAnalogLog(false);
		m_ChkBtnLogAll.SetCheck(0);
	}

	if (m_ChkSum == 18)
	{
		m_ChkBtnLogAll.SetCheck(1);
	}
}

void CPowerCalibrationDlg::OnBnClickedChkLog1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::OnBnClickedChkLog2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::OnBnClickedChkLog3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::OnBnClickedChkLog5()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::OnBnClickedCheck6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::ReadGantryInformation()
{
	ReadGantryCommandPosition();
	ReadGantryPosition();
	ReadGantryTorque();
	Read1DCompensation();
	Read2DCompensation();
}

void CPowerCalibrationDlg::SendCamRecognitionOffset()
{
	SendCameraRecognitionOffset(FRONT_GANTRY);
}

void CPowerCalibrationDlg::ReadGantryCommandPosition()
{
	double X, Y, R, Z;
	CString strMsg;
	X = ReadCommandPosition(_T("FX"));
	Y = ReadCommandPosition(_T("FY1"));
	if (m_SelectNo1.GetCheck() == 1)
	{
		R = ReadCommandPosition(_T("FW1"));
		Z = ReadCommandPosition(_T("FZ1"));
	}
	else
	{
		R = ReadCommandPosition(_T("FW2"));
		Z = ReadCommandPosition(_T("FZ2"));
	}	
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_CMD_X, strMsg);
	strMsg.Format(_T("%.3f"), Y);
	SetDlgItemText(IDC_EDIT_CMD_Y, strMsg);
	strMsg.Format(_T("%.3f"), R);
	SetDlgItemText(IDC_EDIT_CMD_R, strMsg);
	strMsg.Format(_T("%.3f"), Z);
	SetDlgItemText(IDC_EDIT_CMD_Z, strMsg);
}

void CPowerCalibrationDlg::ReadGantryPosition()
{
	double X, Y, R, Z;
	CString strMsg;
	X = ReadPosition(_T("FX"));
	Y = ReadPosition(_T("FY1"));
	if (m_SelectNo1.GetCheck() == 1)
	{
		R = ReadPosition(_T("FW1"));
		Z = ReadPosition(_T("FZ1"));
	}
	else
	{
		R = ReadPosition(_T("FW2"));
		Z = ReadPosition(_T("FZ2"));
	}
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_FEED_X, strMsg);
	strMsg.Format(_T("%.3f"), Y);
	SetDlgItemText(IDC_EDIT_FEED_Y, strMsg);
	strMsg.Format(_T("%.3f"), R);
	SetDlgItemText(IDC_EDIT_FEED_R, strMsg);
	strMsg.Format(_T("%.3f"), Z);
	SetDlgItemText(IDC_EDIT_FEED_Z, strMsg);
}

void CPowerCalibrationDlg::ReadGantryTorque()
{
	double X, Y, R, Z;
	CString strMsg;
	X = ReadActualTorque(_T("FX"));
	Y = ReadActualTorque(_T("FY1"));
	if (m_SelectNo1.GetCheck() == 1)
	{
		R = ReadActualTorque(_T("FW1"));
		Z = ReadActualTorque(_T("FZ1"));
	}
	else
	{
		R = ReadActualTorque(_T("FW2"));
		Z = ReadActualTorque(_T("FZ2"));
	}
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_TORQUE_X, strMsg);
	strMsg.Format(_T("%.3f"), Y);
	SetDlgItemText(IDC_EDIT_TORQUE_Y, strMsg);
	strMsg.Format(_T("%.3f"), R);
	SetDlgItemText(IDC_EDIT_TORQUE_R, strMsg);
	strMsg.Format(_T("%.3f"), Z);
	SetDlgItemText(IDC_EDIT_TORQUE_Z, strMsg);
}

void CPowerCalibrationDlg::Read1DCompensation()
{
	double Y2;
	CString strMsg;
	Y2 = Read1DCompensationData(_T("FY2"));
	strMsg.Format(_T("%.3f"), Y2);
	SetDlgItemText(IDC_EDIT_1D_Y2, strMsg);
}

void CPowerCalibrationDlg::Read2DCompensation()
{
	double X, Y1, Y2;
	CString strMsg;
	X = Read2DCompensationData(_T("FX"));
	Y1 = Read2DCompensationData(_T("FY1"));
	Y2 = Read2DCompensationData(_T("FY2"));
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_2D_X, strMsg);
	strMsg.Format(_T("%.3f"), Y1);
	SetDlgItemText(IDC_EDIT_2D_Y, strMsg);
	strMsg.Format(_T("%.3f"), Y2);
	SetDlgItemText(IDC_EDIT_2D_Y2, strMsg);
}

void CPowerCalibrationDlg::OnBnClickedBtnReadxy()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	double X, Y, R, Z;
	ReadAllPosition(FRONT_GANTRY);
	X = ReadPosition(GetAxisX(FRONT_GANTRY));
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_CMD_X, strMsg);
	Y = ReadPosition(GetAxisY1(FRONT_GANTRY));
	strMsg.Format(_T("%.3f"), Y);
	SetDlgItemText(IDC_EDIT_CMD_Y, strMsg);
	if (m_SelectNo1.GetCheck() == 1)
	{
		R = ReadPosition(_T("FW1"));
		Z = ReadPosition(_T("FZ1"));
	}
	else
	{
		R = ReadPosition(_T("FW2"));
		Z = ReadPosition(_T("FZ2"));
	}
	strMsg.Format(_T("%.3f"), R);
	SetDlgItemText(IDC_EDIT_CMD_R, strMsg);
	strMsg.Format(_T("%.3f"), Z);
	SetDlgItemText(IDC_EDIT_CMD_Z, strMsg);
}


void CPowerCalibrationDlg::OnBnClickedBtnMovemcalx()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		long index = 0, InposMs = TIME300MS;
		double Ratio = 0.1, Inpos = 0.001;
		CString strMsg;
		long MarkNo = MCALMARK, TimeChk = 0, CatchDelay = 0;
		CApplicationTime* pTime = new CApplicationTime();
		if (m_RbMarkMachCal.GetCheck() == 1)
		{
			MarkNo = MCALMARK;
		}
		else if (m_RbMarkMachCalRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkCamRef.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		else if (m_RbMarkPcb.GetCheck() == 1)
		{
			MarkNo = MACHREFMARK;
		}
		GetDlgItemText(IDC_CB_RATIO, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Ratio = ConvertCStringToDouble(strMsg) / 100.0;
			TRACE(_T("[PWR] Selected Ratio:%.1f\n"), Ratio);
			if (Ratio < 0.0 || Ratio > 1.0)
			{
				TRACE(_T("[PWR] Selected Invalid Ratio:%.1f Restore 0.5\n"), Ratio);
				Ratio = 0.5;
			}
		}
		GetDlgItemText(IDC_EDIT_INPOS, strMsg);
		if (strMsg.GetLength() > 0)
		{
			Inpos = ConvertCStringToDouble(strMsg);
			TRACE(_T("[PWR] Selected Inpos:%.3f\n"), Inpos);
		}
		GetDlgItemText(IDC_EDIT_CATCH_DELAY, strMsg);
		if (strMsg.GetLength() > 0)
		{
			CatchDelay = ConvertCStringToInt(strMsg);
		}
		index = m_cbBtnTeach.GetCurSel();
		Point_XYRZE pt;
		double TorqueX, TorqueY1, TorqueY2;
		TorqueX = TorqueY1 = TorqueY2 = 0.000;
		ZeroMemory(&pt, sizeof(pt));
		if (gcPowerCalibrationData != NULL)
		{
			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, index);
			TRACE(_T("[PWR] Get Point(%d) XYRZ %.3f %.3f %.3f %.3f Exe:%d\n"), index, pt.x, pt.y, pt.r, pt.z, pt.exe);
			if (pt.exe == 1)
			{
				pt.x = pt.x + CAL_2D_MAX_XPOS;
				pTime->TimeGet();
				LinearIntplPos(FRONT_GANTRY, pt, Ratio, Inpos, InposMs);
				TimeChk = pTime->TimeElapsed();
				if (m_ChkBtnCatchMark.GetCheck() == 1)
				{
					ThreadSleep(CatchDelay);
					if (MarkNo == MCALMARK)
					{
						CatchMachCalMark(FHCAM, MarkNo);
					}
					else if (MarkNo == MACHREFMARK)
					{
						CatchMachRefMark(FHCAM, MarkNo);
					}
					TorqueX = ReadActualTorque(_T("FX"));
					TorqueY1 = ReadActualTorque(_T("FY1"));
					TorqueY2 = ReadActualTorque(_T("FY2"));
					TRACE(_T("[PWR] VisionXY %.3f %.3f TorqueX,Y1,Y2 %.3f %.3f %.3f Elapsed:%d[ms]\n"), m_res.x, m_res.y, TorqueX, TorqueY1, TorqueY2, TimeChk);
				}
			}
		}
		delete pTime;
	}
}


void CPowerCalibrationDlg::OnBnClickedRadio6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(1);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}


void CPowerCalibrationDlg::OnBnClickedRadio7()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(1);
	m_cbBtnSelCamera.SetCurSel(0);
}


void CPowerCalibrationDlg::OnBnClickedChkLog0()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedBtnControl()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//CMachineFileDB* test = new CMachineFileDB();
	//sqlite3* db = NULL;

	//test->LoadAxisParameterFromDB();
	//test->Load1dFromDB(FRONT_GANTRY);
	//test->Load1dFromDB(REAR_GANTRY);

	//test->Load2dFromDB(FRONT_GANTRY);
	//test->Load2dFromDB(REAR_GANTRY);

	//test->LoadAlignFromDB();
	//test->LoadRecogPositionFromDB();

	//test->LoadHeadOffsetFromDB();
	//test->LoadHeightMeasurementFromDB();

	//test->LoadAncCofigFromDB();
	//test->LoadAncFromDB(FRONT_STAGE);
	//test->LoadAncFromDB(REAR_STAGE);
	//test->LoadPcbFixFromDB();
	//test->LoadFeederReferenceFromDB();

	//delete test;


}


void CPowerCalibrationDlg::OnBnClickedBtnStop()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//CMachineFileDB* test = new CMachineFileDB();
	//sqlite3* db = NULL;

	//test->InitialAxisParameter();
	//test->SaveAxisParameter();

	//test->Initial1dConfig();
	//test->Initial1dTable(FRONT_GANTRY);
	//test->Initial1dTable(REAR_GANTRY);
	//test->Save1d(FRONT_GANTRY);
	//test->Save1d(REAR_GANTRY);

	//test->Initial2dConfig();
	//test->Initial2dTable(FRONT_GANTRY);
	//test->Initial2dTable(REAR_GANTRY);
	//test->Save2d(FRONT_GANTRY);
	//test->Save2d(REAR_GANTRY);

	//test->InitialAlign();
	//test->SaveAlign();

	//test->InitialRecognitionModule();
	//test->SaveRecogPosition();

	//test->InitialHeadOffset();
	//test->SaveHeadOffset();

	//test->InitialHeightMeasurement();
	//test->SaveHeightMeasurement();

	//test->InitialAncConfig();
	//test->SaveAncCofigFromXML();

	//test->InitialAnc(FRONT_STAGE);
	//test->InitialAnc(REAR_STAGE);
	//test->SaveAncFromXML(FRONT_STAGE);
	//test->SaveAncFromXML(REAR_STAGE);

	//test->InitialPcbFix();
	//test->SavePcbFix();

	//test->InitialFeederReference();
	//test->SaveFeederReference();

	//delete test;
}


void CPowerCalibrationDlg::OnBnClickedCheck7()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedCheck1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

}


void CPowerCalibrationDlg::OnBnClickedBtn1stXy()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	double X, Y;
	CString strMsg;
	X = ReadPosition(_T("FX"));
	Y = ReadPosition(_T("FY1"));
	strMsg.Format(_T("%.3f"), X);
	SetDlgItemText(IDC_EDIT_1ST_X, strMsg);
	strMsg.Format(_T("%.3f"), Y);
	SetDlgItemText(IDC_EDIT_1ST_Y, strMsg);
}


void CPowerCalibrationDlg::OnBnClickedBtn2ndXy()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	double X1, X2, Y1, Y2, DistX, DistY;
	CString strMsg;
	X1 = X2 = ReadPosition(_T("FX"));
	Y1 = Y2 = ReadPosition(_T("FY1"));
	strMsg.Format(_T("%.3f"), X2);
	SetDlgItemText(IDC_EDIT_2ND_X, strMsg);
	strMsg.Format(_T("%.3f"), Y2);
	SetDlgItemText(IDC_EDIT_2ND_Y, strMsg);	
	GetDlgItemText(IDC_EDIT_1ST_X, strMsg);
	if (strMsg.GetLength() > 0)
	{
		X1 = ConvertCStringToDouble(strMsg);
	}
	GetDlgItemText(IDC_EDIT_1ST_Y, strMsg);
	if (strMsg.GetLength() > 0)
	{
		Y1 = ConvertCStringToDouble(strMsg);
	}
	DistX = abs(X1 - X2);
	DistY = abs(Y1 - Y2);
	strMsg.Format(_T("%.3f"), DistX);
	SetDlgItemText(IDC_EDIT_DIST_X, strMsg);
	strMsg.Format(_T("%.3f"), DistY);
	SetDlgItemText(IDC_EDIT_DIST_Y, strMsg);
}


void CPowerCalibrationDlg::OnBnClickedRbTargetOffset()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbHeadCam.SetCheck(1);
	m_RbHead1.SetCheck(0);
	m_RbHead2.SetCheck(0);
}


void CPowerCalibrationDlg::OnBnClickedRbTargetHead1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbHeadCam.SetCheck(0);
	m_RbHead1.SetCheck(1);
	m_RbHead2.SetCheck(0);
}


void CPowerCalibrationDlg::OnBnClickedRbTargetHead2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbHeadCam.SetCheck(0);
	m_RbHead1.SetCheck(0);
	m_RbHead2.SetCheck(1);
}

void CPowerCalibrationDlg::OnBnClickedCheck2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedBtnPart()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	gInspectBarcode(FHCAM);
}

void CPowerCalibrationDlg::OnBnClickedChkLogAll()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	long ChkAll = 0;
	if (m_ChkBtnLogAll.GetCheck() == 1)
		ChkAll = 1;
	else
		ChkAll = 0;
	m_ChkBtnLog0.SetCheck(ChkAll);
	m_ChkBtnLog1.SetCheck(ChkAll);
	m_ChkBtnLog2.SetCheck(ChkAll);
	m_ChkBtnLog3.SetCheck(ChkAll);
	m_ChkBtnLog4.SetCheck(ChkAll);
	m_ChkBtnLog5.SetCheck(ChkAll);
	m_ChkBtnLog6.SetCheck(ChkAll);
	m_ChkBtnLog7.SetCheck(ChkAll);
	m_ChkBtnLog8.SetCheck(ChkAll);
	m_ChkBtnLog9.SetCheck(ChkAll);
	m_ChkBtnLog10.SetCheck(ChkAll);
	m_ChkBtnLog11.SetCheck(ChkAll);
	m_ChkBtnLog12.SetCheck(ChkAll);
	m_ChkBtnLog13.SetCheck(ChkAll);
	m_ChkBtnLog14.SetCheck(ChkAll);
	m_ChkBtnLog15.SetCheck(ChkAll);
	m_ChkBtnLog16.SetCheck(ChkAll);
	m_ChkBtnLog17.SetCheck(ChkAll);
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog7()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog8()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedBtnMarkTrain()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int iCameraSel = m_cbBtnSelCamera.GetCurSel(), iCameraNo = FHCAM;
	long MarkNo = 0, Ret = 0;
	long RepeatMark = 0;
	double RoiRatioW = 0.3, RoiRatioH = 0.3;
	CString str;
	GetDlgItemText(IDC_EDIT_REPEAT_MARK, str);
	if (str.GetLength() > 0)
	{
		RepeatMark = ConvertCStringToInt(str);
	}
	if (m_RbMarkMachCal.GetCheck() == 1)
	{
		MarkNo = MCALMARK;
	}
	else if (m_RbMarkMachCalRef.GetCheck() == 1)
	{
		MarkNo = MACHREFMARK;
	}
	else if (m_RbMarkCamRef.GetCheck() == 1)
	{
		MarkNo = MACHREFMARK;
	}
	else if (m_RbMarkAlignHead.GetCheck() == 1)
	{
		MarkNo = ALIGNMARKWHT;
	}
	else if (m_RbMarkAlignModule.GetCheck() == 1)
	{
		MarkNo = ALIGNMARKWHTCAM2;
	}
	else if (m_RbMarkPcb.GetCheck() == 1)
	{
		MarkNo = FIDMARK0;
	}
	else if (m_RbMarkPcb2.GetCheck() == 1)
	{
		MarkNo = FIDMARK1;
	}	
	GetDlgItemText(IDC_EDIT_ROI_RATIO, str);
	if (str.GetLength() > 0)
	{
		RoiRatioW = ConvertCStringToDouble(str);
	}
	GetDlgItemText(IDC_EDIT_ROI_RATIO2, str);
	if (str.GetLength() > 0)
	{
		RoiRatioH = ConvertCStringToDouble(str);
	}
	if (iCameraSel == 0)
		iCameraNo = CAM1;
	else if (iCameraSel == 1)
		iCameraNo = CAM2;
	else if (iCameraSel == 2)
		iCameraNo = CAM5;
	else if (iCameraSel == 3)
		iCameraNo = CAM6;
	else
		iCameraNo = FHCAM;
	InitializeRaw();
	ReliabilityInit();
	for (long rpt = 0; rpt < RepeatMark; ++rpt)
	{
		if (MarkNo == MCALMARK)
		{
			CatchMachCalMark(iCameraNo, MarkNo);
		}
		else if (MarkNo == MACHREFMARK)
		{
			CatchMachRefMark(iCameraNo, MarkNo);
		}
		else if (MarkNo >= FIDMARK0 && MarkNo <= FIDMARK19)
		{
			gLiveOn(iCameraNo);
			WindowSize Win;
			Win.x1 = 320;
			Win.y1 = 240;
			Win.x2 = 960;
			Win.y2 = 720;
			markInitialize(MarkNo, RoiRatioW, RoiRatioH, 0, 0, Win);
			Ret = markTraining(iCameraNo, MarkNo, 0, &m_res);
			TRACE(_T("[PWR] markTraining Cam:%d MarkNo:%d Ret(%d) ResXYR %.3f %.3f %.3f\n"), iCameraNo, MarkNo, Ret, m_res.x, m_res.y, m_res.r);
		}
		else
		{
			CatchMark(FHCAM, MarkNo);
		}
		ReliabilityRaw2(rpt, m_res.x, m_res.y);
		ReadGantryInformation();
	}
	ReliabilityMakeStdev(_T("TrainMark"), RepeatMark);
}


void CPowerCalibrationDlg::OnBnClickedRadio8()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbMarkMachCal.SetCheck(0);
	m_RbMarkMachCalRef.SetCheck(0);
	m_RbMarkCamRef.SetCheck(0);
	m_RbMarkPcb.SetCheck(0);
	m_RbMarkPcb2.SetCheck(1);
	m_RbMarkAlignHead.SetCheck(0);
	m_RbMarkAlignModule.SetCheck(0);
	m_cbBtnSelCamera.SetCurSel(4);
}


void CPowerCalibrationDlg::OnBnClickedBtnGoOrigin()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	Point_XY pt;
	long Gantry = FRONT_GANTRY, Err = NO_ERR;
	double Ratio = 0.5;
	pt.x = 0.0;
	pt.y = 0.0;
	if (GetRunMode() == NORMAL_MODE)
	{
		Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("StartAllZAxisWaitMotion Before GoOrigin"));
			return;
		}
		Err = StartAllRAxisWaitMotion(Gantry, GetStandByR(FRONT_GANTRY), Ratio, TIME5000MS);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("StartAllRAxisWaitMotion Before GoOrigin"));
			return;
		}
		Err = LinearIntplPosWaitMotion(Gantry, FHCAM, pt, Ratio, TIME5000MS);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("LinearIntplPosWaitMotion GoOrigin"));
			return;
		}
	}
}

void CPowerCalibrationDlg::OnBnClickedChkLog9()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::ShowHeightMeasurement()
{
	HANDLE nHandle;
	DWORD nID = NULL;
	m_bStopHeight = false;
	_beginthreadex_proc_type lpStartAddress;
	lpStartAddress = (_beginthreadex_proc_type)ThreadHeightMeasurement;
	nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	strLog.Format(_T("[PWR] CPowerCalibrationDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("ThreadHeightMeasurement"));
	gcPowerLog->Logging(strLog);
}

UINT CPowerCalibrationDlg::ThreadHeightMeasurement(LPVOID wParam)
{
	double Height = 0.0;
	CString strHeight;
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	while (1)
	{
		Height = GetHeight(FRONT_GANTRY);
		strHeight.Format(_T("%.3f"), Height);
		pThis->SetDlgItemText(IDC_EDIT_HEIGHT, strHeight);
		ThreadSleep(TIME1MS);
		if (pThis->m_bStopHeight == true)
		{
			break;
		}
	}
	return 0;
}

void CPowerCalibrationDlg::OnBnClickedRbNo1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedRbNo2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedBtnZplus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if (GetWorkExistSkip() == 1)
	{
		ClearInsertEnd(FRONT_CONV);
		SetInsertDone(0);
		SetPcbOutDone(1);
		//gcLastPickFront->ClearAllHeadData();
	}
}


void CPowerCalibrationDlg::OnBnClickedBtnZminus()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	ClearInsertEnd(FRONT_CONV);
	SetInsertDone(0);
	SetPcbOutDone(1);

	return;
	//CallbackHMI(HMI_CMD1ST_3, HMI_CMD2ND_40, HMI_CMD3RD_17, _T("0,1"));
}


void CPowerCalibrationDlg::OnBnClickedBtnRcw()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedBtnRccw()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//SetWorkPcbReady(FRONT_CONV, true);
	//SetInsertDone(0);
	//SetPcbOutDone(1);

	//if (gcInsertEndFile)
	//{
	//	TRACE(_T("[PWR] ***************************************** ClearInsertEnd Front Conveyor *****************************************\n"));
	//	gcInsertEndFile->ClearInsertEnd(FRONT_CONV);
	//}

	//CallbackHMI(3,4,60,_T(""));

	CallbackHMI(HMI_CMD1ST_1, HMI_CMD2ND_33, HMI_CMD3RD_00, _T("0"));
}


void CPowerCalibrationDlg::OnBnClickedChkLog10()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}

void CPowerCalibrationDlg::OnBnClickedBtnSuction()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedBtnBlow()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

void CPowerCalibrationDlg::OnBnClickedBtnAxisservo()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	//if (gcAdvancedMotionFile)
	//{
	//	long Err = NO_ERR;
	//	Err = gcAdvancedMotionFile->ReadFile();
	//	TRACE(_T("[PWR] gcAdvancedMotionFile ReadFile:%d\n"), Err);
	//}
	ShowSmemaIO();
	ShowPCBSensor();
	ShowSuctionIO();
	ShowBlowIO();
}

void CPowerCalibrationDlg::OnBnClickedChkUse2d()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnUse2D.GetCheck() == 1)
		m_bAuto2DCompen = true;
	else
		m_bAuto2DCompen = false;
	Set2DCompensationUse(m_bAuto2DCompen);
}

void CPowerCalibrationDlg::OnBnClickedChkUse1d()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnUse1D.GetCheck() == 1)
		m_bAuto1DCompen = true;
	else
		m_bAuto1DCompen = false;
	Set1DCompensationUse(m_bAuto1DCompen);
}

long CPowerCalibrationDlg::ConnectServer()
{
	CString strMsg, strPath;
	if (gcPowerClient == NULL)
	{
		gcPowerClient = new CPowerClient();
		if (gcPowerClient->ConnectToServer(ID_MCS, _T(STRING_SERVER_IP), PORT_SERVER) == TRUE)
		{
			TRACE(_T("[PWR] ConnectToServer Complete\n"));
			gcPowerLog->Logging(_T("[PWR] ConnectToServer Complete"));
			strMsg = _T("Disconnect");
			SetDlgItemText(IDC_BTN_CONN_SERVER, strMsg);
			strPath.Format(_T("%S%S%S\\Comm\\"), ROOT_PATH, MCS_PATH, LOG_PATH);
			gcPowerClient->DoCommunicationLog(strPath);
			InitInteraface();
			SetInitializeOption();
			ThreadSleep(TIME10MS);
			return NO_ERR;
		}
		else
		{
			TRACE(_T("[PWR] Err ConnectToServer fail\n"));
			gcPowerLog->Logging(_T("[PWR] Err ConnectToServer fail"));
			strMsg = _T("Connect");
			SetDlgItemText(IDC_BTN_CONN_SERVER, strMsg);
			delete gcPowerClient;
			gcPowerClient = NULL;
			return CONNECT_FAIL_TO_SERVER;
		}
	}
	else
	{
		strMsg = _T("Connect");
		SetDlgItemText(IDC_BTN_CONN_SERVER, strMsg);
		delete gcPowerClient;
		gcPowerClient = NULL;
		TRACE(_T("[PWR] Already exist Thread Connectect to server\n"));
		return CONNECT_FAIL_TO_SERVER;
	}
}

void CPowerCalibrationDlg::InitInteraface()
{
	gcCPowerHMI = new CPowerHMI();
	if (gcCPowerHMI->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcCPowerHMI->GetId(&id);
		TRACE(_T("[PWR] CPowerHMI Id(0x%04X) Ready\n"), id);
	}
	gcDecoding = new CDecoding();
	if (gcDecoding->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcDecoding->GetId(&id);
		TRACE(_T("[PWR] CDecoding Id(0x%04X) Ready\n"), id);
	}
	gcDecoding1 = new CDecoding1();
	if (gcDecoding1->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcDecoding1->GetId(&id);
		TRACE(_T("[PWR] CDecoding1 Id(0x%04X) Ready\n"), id);
	}
	gcDecoding2 = new CDecoding2();
	if (gcDecoding2->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcDecoding2->GetId(&id);
		TRACE(_T("[PWR] CDecoding2 Id(0x%04X) Ready\n"), id);
	}
	gcDecoding3 = new CDecoding3();
	if (gcDecoding3->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcDecoding3->GetId(&id);
		TRACE(_T("[PWR] CDecoding3 Id(0x%04X) Ready\n"), id);
	}
	gcDecoding4 = new CDecoding4();
	if (gcDecoding4->PingThread(TIME100MS))
	{
		ThreadId_t id;
		gcDecoding4->GetId(&id);
		TRACE(_T("[PWR] CDecoding4 Id(0x%04X) Ready\n"), id);
	}
}

void CPowerCalibrationDlg::SetInitializeOption()
{
	g_bUserDoorPush = false;
	g_bUserDoorClose = false;
	m_bSimulationMode = false;
	m_bAutoConnect = true;
	m_bAutoHoming = true;
	if (m_ChkBtnUse1D.GetCheck() == 1)
		m_bAuto1DCompen = true;
	else
		m_bAuto1DCompen = false;
	if (m_ChkBtnUse2D.GetCheck() == 1)
		m_bAuto2DCompen = true;
	else
		m_bAuto2DCompen = false;
	if (m_ChkBtnUseZCompen.GetCheck() == 1)
		m_bAutoZCompen = true;
	else
		m_bAutoZCompen = false;
	m_GantryCalibrationMethod = 0;
	m_Gantry2DMethod = 0;
	SetGlobalSimulationMode(m_bSimulationMode);
	SetAutoHomingUse(m_bAutoHoming);
	Set1DCompensationUse(m_bAuto1DCompen);
	Set2DCompensationUse(m_bAuto2DCompen);
	SetZCompensationUse(m_bAutoZCompen);
	SetY2CompensationUse(true);
	SetGantryCalibrationMethod(m_GantryCalibrationMethod);
	SetGantry2DMethod(m_Gantry2DMethod);
	SetMachineState(STATE_INIT);
	SetGlobalSettlingDelay(TIME200MS);
	SetFirstPickup(0);
	SetSkipVision(0);
	SetSimulLoading(0);
	SetUsePathLinearIntpl(0);
	SetUse2StepZMotion(0);
	SetUseAreaSensor(0);
	SetWorkExistSkip(0);
	SetUseRTDSensorFX(0);
	SetUseRTDSensorFY1(0);
	SetUseRTDSensorFY2(0);
	TRACE(_T("[PWR] SetInitializeOption Done\n"));
}

void CPowerCalibrationDlg::OnCbnSelchangeCbSelCamera()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

void CPowerCalibrationDlg::OnBnClickedChkSkipMotorpower()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnSkipMotorPower.GetCheck() == 1)
	{
		SetSkipMotorPower(true);
	}
	else
	{
		SetSkipMotorPower(false);
	}
}

void CPowerCalibrationDlg::StartRetryConnect()
{
	HANDLE nHandle;
	DWORD nID = NULL;
	m_bStop = false;
	_beginthreadex_proc_type lpStartAddress;
	lpStartAddress = (_beginthreadex_proc_type)RetryConnect;
	nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
	strLog.Format(_T("[PWR] CPowerCalibrationDlg Thread ID:0x%04X(%s)"), nID, (LPCTSTR)_T("RetryConnect"));
	gcPowerLog->Logging(strLog);
}

UINT CPowerCalibrationDlg::RetryConnect(LPVOID wParam)
{
	CPowerCalibrationDlg* pThis = reinterpret_cast<CPowerCalibrationDlg*>(wParam);
	long Err = NO_ERR;
	bool bConnectedCurrent = false;
	bool bConnectedFirst = false;

	while (1)
	{
		if (bConnectedCurrent == false)
		{
			if (bConnectedFirst == false)
			{
				Err = pThis->ConnectServer();
				if (Err == NO_ERR)
				{
					bConnectedCurrent = true;
					bConnectedFirst = true;
				}
			}
			else
			{
				if (gcPowerClient->ConnectToServer(ID_MCS, _T(STRING_SERVER_IP), PORT_SERVER) == TRUE)
				{
					TRACE(_T("[PWR] RetryConnect Done\n"));
					bConnectedCurrent = true;
				}
				else
				{
					TRACE(_T("[PWR] RetryConnect Fail\n"));
				}
			}
		}
		if (bConnectedCurrent == true)
		{
			if (GetRecvInitializeMachine() == false)
			{
				SendInitializeHMI(HMI_INITIALIZE_NOTYET);
			}
			else if (gcPowerClient->m_Status == CLIENT_DISCONNECTED)
			{
				gcPowerClient->Disconnect();
				bConnectedCurrent = false;
				TRACE(_T("[PWR] Disconnect Done\n"));
			}
		}
		ThreadSleep(TIME1000MS);
	}
	return 0;
}


void CPowerCalibrationDlg::OnBnClickedChkInitY2Shift()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnInitY2Shift.GetCheck() == 1)
		SetInitY2Shift(true);
	else
		SetInitY2Shift(false);
}


void CPowerCalibrationDlg::OnBnClickedChkInfinite()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnInfiniteDryRun.GetCheck() == 1)
		SetInfiniteDryRun(1);
	else
		SetInfiniteDryRun(0);
}


void CPowerCalibrationDlg::OnBnClickedChkLog11()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog12()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog13()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog14()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkLog15()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkFirstpickup()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtn_FirstPickup.GetCheck() == 1)
		SetFirstPickup(1);
	else
		SetFirstPickup(0);
}


void CPowerCalibrationDlg::OnBnClickedChkSkipVision()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnSkipVision.GetCheck() == 1)
		SetSkipVision(1);
	else
		SetSkipVision(0);
}


void CPowerCalibrationDlg::OnBnClickedChkSimulLoading()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnSimulLoading.GetCheck() == 1)
	{
		SetSimulLoading(1);
	}
	else
	{
		SetSimulLoading(0);
	}
}


void CPowerCalibrationDlg::OnBnClickedChkUsePath()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnUsePathIntpl.GetCheck() == 1)
	{
		SetUsePathLinearIntpl(1);
	}
	else
	{
		SetUsePathLinearIntpl(0);
	}
}


void CPowerCalibrationDlg::OnBnClickedChk2stepZ()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnUse2StepZMotion.GetCheck() == 1)
	{
		SetUse2StepZMotion(1);
	}
	else
	{
		SetUse2StepZMotion(0);
	}
}


void CPowerCalibrationDlg::OnBnClickedChkLog16()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnBnClickedChkUseLob()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkUseLineOfBalance.GetCheck() == 1)
	{
		SetUseLineOfBalance(1);
	}
	else
	{
		SetUseLineOfBalance(0);
	}
}


void CPowerCalibrationDlg::OnBnClickedChkUseLob2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCalibrationDlg::OnBnClickedChkWorkexistskip()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	if (m_ChkBtnWorkExistSkip.GetCheck() == 1)
	{
		SetWorkExistSkip(1);
	}
	else
	{
		SetWorkExistSkip(0);
	}
}


void CPowerCalibrationDlg::OnBnClickedChkLog17()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	SetShowLog();
}


void CPowerCalibrationDlg::OnEnChangeEditLevel()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CDialogEx::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
}
