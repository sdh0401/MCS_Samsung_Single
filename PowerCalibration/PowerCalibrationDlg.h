#pragma once
// PowerCalibrationDlg.h: 헤더 파일
//
#include "CLogFileSystem.h"
#include "CPowerHomeDlg.h"
#include "CPowerCamCalDlg.h"
#include "CPowerMasterStatusDlg.h"
#include "Cwmx3Init.h"
#include "CMessageQueue.h"
#include "CTrayIconManager.h"

using namespace std;

// CPowerCalibrationDlg 대화 상자
class CPowerCalibrationDlg : public CDialogEx
{
// 생성입니다.
public:
	CPowerCalibrationDlg(CWnd* pParent = nullptr);	// 표준 생성자입니다.
	~CPowerCalibrationDlg();
	bool m_bSimulationMode;
	bool m_bAutoHoming;
	bool m_bAutoConnect;
	bool m_bAuto1DCompen;
	bool m_bAuto2DCompen;
	bool m_bAutoZCompen;
	long m_GantryCalibrationMethod;
	long m_Gantry2DMethod;
	bool m_bStop;
	bool m_bStopHeight;
	Point_XYRE m_res;

	CPowerHomeDlg* m_pFormHome;
	CPowerCamCalDlg* m_pFormCamCal;
	CPowerMasterStatusDlg* m_pFormMasterStatus;
	CString strLog, strPath;
	HANDLE m_CmdLock;

	CTrayIconManager m_TrayIcon;
	bool m_bHide;
	void OnAppExit(void);
	void OnDialogShow(void);
	void InitializeRaw();
	void AllocForms();
	void ShowForm(PowerCalibrationNo idx);
	void AllHideForm();
	void SocketShowWindow(int bShow);
	void CreateAllThreadLock();
	void SendAllThreadSelfKill();
	void SendAllThreadRun();
	void MasterMotionSelfOut();
	void Wmx3MotorSelfOut();
	void SlaveMotorSelfOut();
	void Wmx3IOSelfOut();
	void IOStatusSelfOut();
	void AnalogStatusSelfOut();
	void HostMessageQueueSelfOut();
	void StartCalibrationSelfOut();
	void SendMessageToThread(string strMsg);
	void LedOn(int CameraNo, int iValue1, int iValue2, int iValue3);
	void LedOff(int CameraNo);
	void LiveCamera(int CameraNo);
	void CatchMachCalMark(int CameraNo, long MarkNo);
	void CatchMachRefMark(int CameraNo, long MarkNo);
	void CatchMark(int CameraNo, long MarkNo);
	void SendToTeachBox(CString strAxis, unsigned nSub1, unsigned nSub2, unsigned nSub3);
	void LinearIntplPos(long Gantry, Point_XYRZE pt, double Ratio, double Inpos, long Ms);
	void SingleMove(long Gantry, Point_XYRZE pt);
	HANDLE GetThreadLock();
	bool Lock();
	bool Unlock();
	static UINT RepeatCatchMark(LPVOID wParam);
	void DoExecute();
	static UINT Execute(LPVOID wParam);
	void Stop();
	void DoInit();
	static UINT Initialize(LPVOID wParam);
	void SetShowLog();
	void SendCamRecognitionOffset();
	void ReadGantryInformation();
	void ReadGantryCommandPosition();
	void ReadGantryPosition();
	void ReadGantryTorque();
	void Read1DCompensation();
	void Read2DCompensation();
	static UINT ServoAllOn(LPVOID wParam);
	static UINT ServoAllOff(LPVOID wParam);
	void ShowHeightMeasurement();
	static UINT ThreadHeightMeasurement(LPVOID wParam);
	long ConnectServer();
	void InitInteraface();
	void SetInitializeOption();
	void StartRetryConnect();
	static UINT RetryConnect(LPVOID wParam);


// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORM_MAIN };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.

// 구현입니다.
protected:
	HICON m_hIcon;
	afx_msg LRESULT RecvClientMsg(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT SendClientMsg(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT StartMaster(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnTrayIcon(WPARAM wParam, LPARAM lParam);

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedRadioHome();
	afx_msg void OnBnClickedRadioCamcal();
	afx_msg void OnBnClickedRadioSocket();
	CButton m_rBtnSocket;
	CButton m_rBtnHome;
	CButton m_rBtnCamCal;
	afx_msg void OnBnClickedRadioMaster();
	afx_msg void OnBnClickedOk();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedBtnConnServer();
	afx_msg void OnBnClickedBtnHoming();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnBnClickedBtnFile();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedBtnInit();
	CButton m_rSimulOn;
	CButton m_rSimulOff;
	afx_msg void OnBnClickedSimulOn();
	afx_msg void OnBnClickedSimulOff();
	afx_msg void OnBnClickedBtnEstop();
	afx_msg void OnBnClickedBtnVisinit();
	afx_msg void OnBnClickedBtnVisdemo();
	afx_msg void OnBnClickedBtnYplus();
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnKeyUp(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnBnClickedBtnYminus();
	afx_msg void OnBnClickedBtnXminus();
	afx_msg void OnBnClickedBtnXplus();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedBtnLive();
	afx_msg void OnBnClickedBtnMark();
	CComboBox m_cbBtnTeach;
	afx_msg void OnCbnSelchangeCbTeach();
	CButton m_ChkBtnRepeat;
	CComboBox m_cbBtnSelCamera;
	afx_msg void OnBnClickedBtnServo();
	CButton m_RbWait;
	CButton m_RbWaitPosSet;
	CButton m_RbWaitDelayedPosSet;
	afx_msg void OnBnClickedRbWait();
	afx_msg void OnBnClickedRbWaitposset();
	afx_msg void OnBnClickedRbWaitdelayedposset();
	afx_msg void OnBnClickedBtnTeachMinus();
	afx_msg void OnBnClickedBtnTeachPlus();
	CComboBox m_cbBtnRatio;
	afx_msg void OnBnClickedBtnClearPoint();
	CButton m_ChkBtnCatchMark;
	afx_msg void OnBnClickedChkRepeat();
	CButton m_RbMarkMachCal;
	CButton m_RbMarkMachCalRef;
	CButton m_RbMarkCamRef;
	CButton m_RbMarkPcb;
	afx_msg void OnBnClickedRadio2();
	afx_msg void OnBnClickedRadio3();
	afx_msg void OnBnClickedRadio4();
	afx_msg void OnBnClickedRadio5();
	afx_msg void OnBnClickedBtnMovemcal();
	afx_msg void OnBnClickedBtnMovemcalxy();
	CButton m_ChkBtnLog1;
	CButton m_ChkBtnLog2;
	CButton m_ChkBtnLog3;
	CButton m_ChkBtnLog4;
	CButton m_ChkBtnLog5;
	afx_msg void OnBnClickedChkLog1();
	afx_msg void OnBnClickedChkLog2();
	afx_msg void OnBnClickedChkLog3();
	afx_msg void OnBnClickedChkLog4();
	afx_msg void OnBnClickedChkLog5();
	afx_msg void OnBnClickedCheck6();
	CButton m_ChkBtnInputMove;
	afx_msg void OnBnClickedBtnReadxy();
	afx_msg void OnBnClickedBtnMovemcalx();
	afx_msg void OnBnClickedRadio6();
	afx_msg void OnBnClickedRadio7();
	CButton m_RbMarkAlignHead;
	CButton m_RbMarkAlignModule;
	afx_msg void OnBnClickedChkLog0();
	CButton m_ChkBtnLog0;
	afx_msg void OnBnClickedBtnControl();
	afx_msg void OnBnClickedBtnStop();
	afx_msg void OnBnClickedCheck7();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnBnClickedBtn1stXy();
	afx_msg void OnBnClickedBtn2ndXy();
	CButton m_RbHeadCam;
	CButton m_RbHead1;
	CButton m_RbHead2;
	afx_msg void OnBnClickedRbTargetOffset();
	afx_msg void OnBnClickedRbTargetHead1();
	afx_msg void OnBnClickedRbTargetHead2();
	afx_msg void OnBnClickedCheck2();
	afx_msg void OnBnClickedBtnPart();
	afx_msg void OnBnClickedChkLogAll();
	afx_msg void OnBnClickedChkLog6();
	afx_msg void OnBnClickedChkLog7();
	afx_msg void OnBnClickedChkLog8();
	CButton m_ChkBtnLog6;
	CButton m_ChkBtnLog7;
	CButton m_ChkBtnLog8;
	CButton m_ChkBtnLogAll;
	CButton m_ChkMultiPartRecog;
	afx_msg void OnBnClickedBtnMarkTrain();
	CButton m_RbMarkPcb2;
	afx_msg void OnBnClickedRadio8();
	afx_msg void OnBnClickedBtnGoOrigin();
	CButton m_ChkBtnLog9;
	afx_msg void OnBnClickedChkLog9();
	CButton m_SelectNo1;
	CButton m_SelectNo2;
	afx_msg void OnBnClickedRbNo1();
	afx_msg void OnBnClickedRbNo2();
	afx_msg void OnBnClickedBtnZplus();
	afx_msg void OnBnClickedBtnZminus();
	afx_msg void OnBnClickedBtnRcw();
	afx_msg void OnBnClickedBtnRccw();
	CButton m_ChkBtnUse1D;
	CButton m_ChkBtnUse2D;
	CButton m_ChkBtnUseZCompen;
	CButton m_ChkBtnLog10;
	afx_msg void OnBnClickedChkLog10();
	afx_msg void OnBnClickedBtnSuction();
	afx_msg void OnBnClickedBtnBlow();
	afx_msg void OnBnClickedBtnAxisservo();
	afx_msg void OnBnClickedChkUse2d();
	afx_msg void OnBnClickedChkUse1d();
	afx_msg void OnCbnSelchangeCbSelCamera();
	CButton m_ChkBtnSkipMotorPower;
	afx_msg void OnBnClickedChkSkipMotorpower();
	CButton m_ChkBtnInitY2Shift;
	afx_msg void OnBnClickedChkInitY2Shift();
	CButton m_ChkBtnInfiniteDryRun;
	afx_msg void OnBnClickedChkInfinite();
	afx_msg void OnBnClickedChkLog11();
	CButton m_ChkBtnLog11;
	CButton m_ChkBtnLog12;
	afx_msg void OnBnClickedChkLog12();
	CButton m_ChkBtnLog13;
	afx_msg void OnBnClickedChkLog13();
	CButton m_ChkBtnLog14;
	afx_msg void OnBnClickedChkLog14();
	CButton m_ChkBtnLog15;
	afx_msg void OnBnClickedChkLog15();
	CButton m_ChkBtn_FirstPickup;
	afx_msg void OnBnClickedChkFirstpickup();
	CButton m_ChkBtnSkipVision;
	afx_msg void OnBnClickedChkSkipVision();
	CButton m_ChkBtnSimulLoading;
	afx_msg void OnBnClickedChkSimulLoading();
	CButton m_ChkBtnUsePathIntpl;
	afx_msg void OnBnClickedChkUsePath();
	CButton m_ChkBtnUse2StepZMotion;
	afx_msg void OnBnClickedChk2stepZ();
	CButton m_ChkBtnLog16;
	afx_msg void OnBnClickedChkLog16();
	CButton m_ChkUseLineOfBalance;
	afx_msg void OnBnClickedChkUseLob();
	afx_msg void OnBnClickedChkUseLob2();

	CButton m_ChkBtnWorkExistSkip;
	afx_msg void OnBnClickedChkWorkexistskip();
	
	CButton m_ChkBtnLog17;
	afx_msg void OnBnClickedChkLog17();
    afx_msg void OnEnChangeEditLevel();
};
