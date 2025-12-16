#pragma once
#include <WMX3Api.h>
using namespace wmx3Api;

#include "GlobalDefine.h"
// CPowerMasterStatusDlg 폼 보기
class CPowerMasterStatusDlg : public CFormView
{
	DECLARE_DYNCREATE(CPowerMasterStatusDlg)
public:
	CPowerMasterStatusDlg();					// 동적 만들기에 사용되는 protected 생성자입니다.
	virtual ~CPowerMasterStatusDlg();

	Point_XY GetMarkPosition(long MarkNo);
	long GetFeederNo();
	long GetHeadNo();
	Point_XYRZ GetPickOffset();
	void SetPickOffset(Point_XYRZ PickOffset);
	Ratio_XYRZ GetPickRatio();
	Ratio_XYRZ GetMachineRatio();
	long GetHeadNoFromInsertOrder(long insertOrd);
	long GetFdNoFromPickOrder(long insertOrd);
	long GetFdNoFromInsertOrder(long insertOrd);
	long GetPickDelayFromFdNo(long FdNo);
	Ratio_XYRZ GetCompRatioFromFdNo(long FdNo);
	Ratio_XYRZ GetComponentRatioByFdNo(long FdNo);
	Point_XYRZ GetPickOffsetFromFdNo(long FdNo);
	double GetVAAngleFromInsertOrder(long insertOrd);
	void SetVAAngleOffsetFromInsertOrder(long insertOrd, double AngleOffset);
	double GetVAAngleOffsetFromInsertOrder(long insertOrd);
	double GetComponentHeight(long insertOrd);
	long GetMaxPickOrder();
	void SetMaxPickOrder(long MaxPickOrd);
	long GetMaxInsertOrder();
	void SetMaxInsertOrder(long MaxOrder);
	long GetPickDelay();
	long GetInsertDelayFromInsertOrder(long insertOrd);
	bool m_bShow;
	bool m_bLoop;
	void ReturnComponentToFeeder(long Gantry);
	void DiscardOneBeforePicking(long Gantry, long HeadNo);
	void DiscardAllBeforePicking(long Gantry);
	void DiscardOneAfterAlignChecking(long Gantry, long HeadNo);
	void DiscardAllAfterAlignChecking(long Gantry);
	void DiscardOneAfterInserting(long Gantry, long HeadNo);
	void DiscardAllAfterInserting(long Gantry);
	long Picking(long Gantry);
	long CheckingAlign(long Gantry);
	long WaitInPcb(long Conveyor);
	long CheckingMark(long Gantry);
	long Inserting(long Gantry);
	long GoOrigin();
	long GetConveyorRunMode();
	void SetConveyorRunMode(long ConveyorRunMode);
	static UINT Run(LPVOID wParam);
	void StartConveyor();
	void StartSyncGantryConveyor();
	void SetPcbInfoConveyor();
	bool IsRemainInsertion();
	void RunConveyor(bool ContinueRun);
	void RunSyncGantryConveyor();
	void StopConveyor();
	void StopSyncGantryConveyor();
	Point_XYRZ GetInsertPoint(long insertNo);
	long GetPickupHeadNo(long insertOrder);
	long GetInsertHeadNo(long PickOrd);
	Point_XYRE m_Res, m_Apply;
	long m_MaxInsertOrder;
	long m_MaxPickOrder;
	double m_VAAngleOffset[MAXUSEDHEADNO];
	long m_ConveyorRunMode;
protected:

public:
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORM_MASTER };
#endif
#ifdef _DEBUG
	virtual void AssertValid() const;
#ifndef _WIN32_WCE
	virtual void Dump(CDumpContext& dc) const;
#endif
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
public:
	virtual void OnInitialUpdate();
	virtual BOOL Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext = NULL);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedBtnTeachOrigin();
	afx_msg void OnBnClickedBtnTeachMark1();
	afx_msg void OnBnClickedBtnTeachMark2();
	afx_msg void OnBnClickedBtnTeachNo1();
	afx_msg void OnBnClickedBtnTeachNo2();
	afx_msg void OnBnClickedBtnTeachNo3();
	afx_msg void OnShowWindow(BOOL bShow, UINT nStatus);
	afx_msg void OnBnClickedBtnGoOrigin();
	afx_msg void OnBnClickedBtnGoMark1();
	afx_msg void OnBnClickedBtnGoMark2();
	afx_msg void OnBnClickedBtnGoNo1();
	afx_msg void OnBnClickedBtnGoNo2();
	afx_msg void OnBnClickedBtnGoNo3();
	CButton m_ChkBtnAfterFiducial;
	CComboBox m_cbBtnPickHead;
	CComboBox m_cbBtnPickSpeedXY;
	CComboBox m_cbBtnPickSpeedR;
	CComboBox m_cbBtnPickSpeedZ;
	CComboBox m_cbBtnMachineSpeed;
	//afx_msg void OnBnClickedBtnPickrecog();
	//afx_msg void OnBnClickedBtnGoFeeder();
	//afx_msg void OnBnClickedBtnTeachFeeder();
	CButton m_ChkBtnOnlyRecog;
	afx_msg void OnBnClickedBtnPickrecog();
	afx_msg void OnBnClickedBtnGoFeeder();
	afx_msg void OnBnClickedBtnTeachFeeder();
	afx_msg void OnBnClickedChkBtnOnlyRecog();
	afx_msg void OnCbnSelchangeCbSpeedXy();
	afx_msg void OnBnClickedBtnRun();
	CComboBox m_cbBtnInsertNo1;
	CComboBox m_cbBtnInsertNo2;
	CComboBox m_cbBtnInsertNo3;
	CButton m_ChkBtnApplyVisRes;
	afx_msg void OnBnClickedChkBtnApplyVis();
	afx_msg void OnBnClickedBtnTeachNo4();
	afx_msg void OnBnClickedBtnGoNo4();
	CComboBox m_cbBtnInsertNo4;
	CButton m_ChkBtnReturnComponentToFeeder;
	afx_msg void OnBnClickedChkBtnReturnFd();
	afx_msg void OnBnClickedBtnReturntofd();
	afx_msg void OnBnClickedCheck1();
	CButton m_ChkBtnUseRecogByInsertAngle;
	afx_msg void OnBnClickedBtnReadjobfile();
	afx_msg void OnCbnSelchangeCbMachineSpeed();
	afx_msg void OnEnChangeEditPickZ2();
	afx_msg void OnEnChangeEditHmDelay();
};


