#pragma once
#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <ecApi.h>
#include "GlobalDefine.h"

// CPowerHomeDlg 폼 보기
class CPowerHomeDlg : public CFormView
{
	DECLARE_DYNCREATE(CPowerHomeDlg)
public:
	CPowerHomeDlg();           // 동적 만들기에 사용되는 protected 생성자입니다.
	virtual ~CPowerHomeDlg();
	void AddLogBox(CString strLog);
	void ResetLogBox();
	static UINT Homing(LPVOID wParam);
	void UpdateLimitInfo();
	void SetLimitInfo(long AxisNo, Limit limit);
	CString m_StrLog;
	Point_XY m_HMOrigin;
	long m_HMZero;
	void UnCheckAllAxis();
private:

protected:

public:
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORM_HOME };
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
	afx_msg void OnBnClickedBtn1d();
	afx_msg void OnBnClickedBtnHoming();
	afx_msg void OnBnClickedBtnCalCancel();
	CButton m_1DSimulOn;
	CButton m_1DSimulOff;
	afx_msg void OnBnClickedR1dSimulon();
	afx_msg void OnBnClickedBtn1dOnoff();
	afx_msg void OnBnClickedBtnHeadcamCal();
	afx_msg void OnBnClickedR1dSimuloff();
	afx_msg void OnBnClickedRhdcamSimulon();
	afx_msg void OnBnClickedRhdcamSimuloff();
	CButton m_HdCamSimulOn;
	CButton m_HdCamSimulOff;
	afx_msg void OnBnClickedBtnHdcamOnoff();
	afx_msg void OnBnClickedBtn2d();
	afx_msg void OnBnClickedBtn2dOnoff();
	CButton m_2DSimulOn;
	CButton m_2DSimulOff;
	afx_msg void OnBnClickedBtnGetvisionversion();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedR2dSimulon();
	afx_msg void OnBnClickedR2dSimuloff();
	CButton m_2DWmx3;
	CButton m_2DSoftware;
	afx_msg void OnBnClickedR2dWmx3();
	afx_msg void OnBnClickedR2dSw();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedCheckHomeRepeat();
	CButton m_ChkBtnHomeRepeat;
	afx_msg void OnBnClickedBtnAlignOffset();
	CButton m_ChkBtnAlignRepeat;
	afx_msg void OnBnClickedCheckAlignRepeat();
	afx_msg void OnBnClickedBtnHeadOffset();
	CButton m_ChkBtnHeadOffsetRepeat;
	afx_msg void OnBnClickedCheckHeadoffsetRepeat();
	CButton m_RbZNo1;
	CButton m_RbZNo2;
	CButton m_RbZNo3;
	CButton m_RbZNo4;
	CButton m_RbZNo5;
	CButton m_RbZNo6;
	afx_msg void OnBnClickedBtnSethomez();
	CButton m_RbFrontGantry;
	CButton m_RbRearGantry;
	afx_msg void OnBnClickedBtnSetpcbfix();
	afx_msg void OnBnClickedBtnSetfdref();
	afx_msg void OnBnClickedRbFront();
	afx_msg void OnBnClickedRbRear();
	afx_msg void OnBnClickedRbZno1();
	afx_msg void OnBnClickedRbZno2();
	afx_msg void OnBnClickedRbZno3();
	afx_msg void OnBnClickedRbZno4();
	afx_msg void OnBnClickedRbZno5();
	afx_msg void OnBnClickedRbZno6();
	afx_msg void OnBnClickedBtnSendCamoffset();
	afx_msg void OnBnClickedBtnGopcbfix();
	afx_msg void OnBnClickedBtnGofdref();
	afx_msg void OnBnClickedBtnSetFeeder();
	afx_msg void OnShowWindow(BOOL bShow, UINT nStatus);
	afx_msg void OnBnClickedBtnSetAlignOffset();
	afx_msg void OnBnClickedBtnZcompen();
	afx_msg void OnBnClickedBtnZcompenOn();
	CButton m_RbCam1;
	CButton m_RbCam2;
	afx_msg void OnBnClickedBtnModulecamCal();
	afx_msg void OnBnClickedRbCam2();
	afx_msg void OnBnClickedRbCam1();
	afx_msg void OnBnClickedBtnInitHeadOffset();
	afx_msg void OnBnClickedButton4();
	CButton m_RbFX;
	CButton m_RbFY1;
	CButton m_RbFY2;
	CButton m_RbFZ1;
	CButton m_RbFZ2;
	CButton m_RbFZ3;
	CButton m_RbFZ4;
	CButton m_RbFZ5;
	CButton m_RbFZ6;
	CButton m_RbFW1;
	CButton m_RbFW2;
	CButton m_RbFW3;
	CButton m_RbFW4;
	CButton m_RbFW5;
	CButton m_RbFW6;
	CButton m_RbFCONV;
	CButton m_RbFPUZ;
	CButton m_RbFBTIN;
	CButton m_RbFBTWK;
	CButton m_RbFBTOT;
	afx_msg void OnBnClickedRadio1();
	afx_msg void OnBnClickedBtnSetLimit();
	afx_msg void OnBnClickedRadio11();
	afx_msg void OnBnClickedRadio12();
	afx_msg void OnBnClickedRadio13();
	afx_msg void OnBnClickedRadio14();
	afx_msg void OnBnClickedRadio15();
	afx_msg void OnBnClickedRadio16();
	afx_msg void OnBnClickedRadio17();
	afx_msg void OnBnClickedRadio18();
	afx_msg void OnBnClickedRadio19();
	afx_msg void OnBnClickedRadio20();
	afx_msg void OnBnClickedRadio21();
	afx_msg void OnBnClickedRadio22();
	afx_msg void OnBnClickedRadio23();
	afx_msg void OnBnClickedRadio24();
	afx_msg void OnBnClickedRadio25();
	afx_msg void OnBnClickedRadio26();
	afx_msg void OnBnClickedRadio27();
	afx_msg void OnBnClickedRadio28();
	afx_msg void OnBnClickedRadio29();
	afx_msg void OnBnClickedBtnSethomer();
	afx_msg void OnBnClickedBtnTeachHm();
	afx_msg void OnBnClickedBtnSaveHm();
	CButton m_RbCam5;
	CButton m_RbCam6;
	afx_msg void OnBnClickedRadio9();
	afx_msg void OnBnClickedRadio10();
	afx_msg void OnBnClickedBtnInitHeadRearoffset();
	afx_msg void OnBnClickedRbFanc();
	afx_msg void OnBnClickedRbRanc();
	CButton m_RbFrontANC;
	CButton m_RbRearANC;
	afx_msg void OnBnClickedBtnAncTeach1();
	afx_msg void OnBnClickedBtnAncTeach2();
	afx_msg void OnBnClickedBtnAncApply();
	afx_msg void OnBnClickedBtnAncApplyZ();
	afx_msg void OnBnClickedBtnAncMove2();

	CButton m_RbANC_R0;
	CButton m_RbANC_R90;
	afx_msg void OnBnClickedRbAncr0();
	afx_msg void OnBnClickedRbAncr90();
	afx_msg void OnBnClickedBtnSethomex();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedBtnROffset();
	afx_msg void OnBnClickedBtnSetselfz();
    afx_msg void OnBnClickedBtnAncReloadfile();
    afx_msg void OnBnClickedBtnAncReadfile();
    afx_msg void OnBnClickedBtnInsertcheck();

	CButton m_ChkBtnUpdateinsertoffset;

};


