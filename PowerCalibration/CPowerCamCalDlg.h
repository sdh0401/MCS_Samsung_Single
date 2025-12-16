#pragma once
#include <WMX3Api.h>
using namespace wmx3Api;

// CPowerCamCalDlg 폼 보기
class CPowerCamCalDlg : public CFormView
{
	DECLARE_DYNCREATE(CPowerCamCalDlg)
public:
	CPowerCamCalDlg();           // 동적 만들기에 사용되는 protected 생성자입니다.
	virtual ~CPowerCamCalDlg();
	void UpdateConvInfo();
	void SetPcbInfoManualConveyor();
protected:

public:
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_FORM_CAMCAL };
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
	afx_msg void OnBnClickedBtnReadfile();
	afx_msg void OnBnClickedBtnInbelt();
	afx_msg void OnBnClickedBtnWorkbelt();
	afx_msg void OnBnClickedBtnOutbelt();
	afx_msg void OnBnClickedBtnPlate();
	afx_msg void OnBnClickedBtnWidth();
	afx_msg void OnBnClickedBtnStopper();
	afx_msg void OnBnClickedBtnPlate2();
	afx_msg void OnBnClickedBtnWidth2();
	CButton m_RbPlate10;
	CButton m_RbPlate1;
	CButton m_RbPlate01;
	CButton m_RbWidth10;
	CButton m_RbWidth1;
	CButton m_RbWidth01;
	afx_msg void OnBnClickedRadio1();
	afx_msg void OnBnClickedRadio2();
	afx_msg void OnBnClickedRadio3();
	afx_msg void OnBnClickedRadio4();
	afx_msg void OnBnClickedRadio5();
	afx_msg void OnBnClickedRadio6();
	CButton m_RbReverseOff;
	CButton m_RbReverseOn;
	afx_msg void OnBnClickedRadio7();
	afx_msg void OnBnClickedRadio8();
	afx_msg void OnBnClickedBtnSetconvwidth();
	afx_msg void OnBnClickedBtnUpdateConv();
	afx_msg void OnBnClickedBtnSetconvpusherz();
	CButton m_RbPlate5;
	afx_msg void OnBnClickedRadio9();
	CButton m_RbWidth5;
	afx_msg void OnBnClickedRadio10();
	afx_msg void OnBnClickedBtnSetconvpuserzOrigin();
	afx_msg void OnBnClickedBtnFrontpcbfixpos();
	afx_msg void OnBnClickedBtnSetconvwidthOrigin();
	afx_msg void OnBnClickedBtnManual();
	afx_msg void OnBnClickedBtnCalcConvpuserzOrigin2();
};


