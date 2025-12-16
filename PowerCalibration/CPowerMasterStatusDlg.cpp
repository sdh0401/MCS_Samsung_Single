// CPowerMasterStatusDlg.cpp: 구현 파일
//
#include "pch.h"
#include "PowerCalibration.h"
#include "PowerCalibrationDlg.h"
#include "CPowerMasterStatusDlg.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CReadJobFile.h"
#include "CMeasureHeight.h"
/*****************************************************************************/
/* Header                                                                    */
/*****************************************************************************/

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/

IMPLEMENT_DYNCREATE(CPowerMasterStatusDlg, CFormView)

CPowerMasterStatusDlg::CPowerMasterStatusDlg()
	: CFormView(IDD_FORM_MASTER)
{
	ZeroMemory(&m_Res, sizeof(m_Res));
	ZeroMemory(&m_Apply, sizeof(m_Apply));
	m_bShow = true;
	m_bLoop = false;
	m_MaxInsertOrder = 0;
	m_MaxPickOrder = 0;
	m_ConveyorRunMode = LOCATION_OUT_NEXT;
	ZeroMemory(&m_VAAngleOffset, sizeof(m_VAAngleOffset));
}

CPowerMasterStatusDlg::~CPowerMasterStatusDlg()
{
}

void CPowerMasterStatusDlg::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHECK1, m_ChkBtnAfterFiducial);
	DDX_Control(pDX, IDC_CB_HEAD, m_cbBtnPickHead);
	DDX_Control(pDX, IDC_CB_SPEED_XY, m_cbBtnPickSpeedXY);
	DDX_Control(pDX, IDC_CB_SPEED_R, m_cbBtnPickSpeedR);
	DDX_Control(pDX, IDC_CB_SPEED_Z, m_cbBtnPickSpeedZ);
	DDX_Control(pDX, IDC_CB_MACHINE_SPEED, m_cbBtnMachineSpeed);
	DDX_Control(pDX, IDC_CHK_BTN_ONLY_RECOG, m_ChkBtnOnlyRecog);
	DDX_Control(pDX, IDC_CB_HEAD_NO1, m_cbBtnInsertNo1);
	DDX_Control(pDX, IDC_CB_HEAD_NO2, m_cbBtnInsertNo2);
	DDX_Control(pDX, IDC_CB_HEAD_NO3, m_cbBtnInsertNo3);
	DDX_Control(pDX, IDC_CHK_BTN_APPLY_VIS, m_ChkBtnApplyVisRes);
	DDX_Control(pDX, IDC_CB_HEAD_NO4, m_cbBtnInsertNo4);
	DDX_Control(pDX, IDC_CHK_BTN_RETURN_FD, m_ChkBtnReturnComponentToFeeder);
	DDX_Control(pDX, IDC_CHECK2, m_ChkBtnUseRecogByInsertAngle);

}

BEGIN_MESSAGE_MAP(CPowerMasterStatusDlg, CFormView)
	ON_BN_CLICKED(IDC_BTN_TEACH_ORIGIN, &CPowerMasterStatusDlg::OnBnClickedBtnTeachOrigin)
	ON_BN_CLICKED(IDC_BTN_TEACH_MARK1, &CPowerMasterStatusDlg::OnBnClickedBtnTeachMark1)
	ON_BN_CLICKED(IDC_BTN_TEACH_MARK2, &CPowerMasterStatusDlg::OnBnClickedBtnTeachMark2)
	ON_BN_CLICKED(IDC_BTN_TEACH_NO1, &CPowerMasterStatusDlg::OnBnClickedBtnTeachNo1)
	ON_BN_CLICKED(IDC_BTN_TEACH_NO2, &CPowerMasterStatusDlg::OnBnClickedBtnTeachNo2)
	ON_BN_CLICKED(IDC_BTN_TEACH_NO3, &CPowerMasterStatusDlg::OnBnClickedBtnTeachNo3)
	ON_WM_SHOWWINDOW()
	ON_BN_CLICKED(IDC_BTN_GO_ORIGIN, &CPowerMasterStatusDlg::OnBnClickedBtnGoOrigin)
	ON_BN_CLICKED(IDC_BTN_GO_MARK1, &CPowerMasterStatusDlg::OnBnClickedBtnGoMark1)
	ON_BN_CLICKED(IDC_BTN_GO_MARK2, &CPowerMasterStatusDlg::OnBnClickedBtnGoMark2)
	ON_BN_CLICKED(IDC_BTN_GO_NO1, &CPowerMasterStatusDlg::OnBnClickedBtnGoNo1)
	ON_BN_CLICKED(IDC_BTN_GO_NO2, &CPowerMasterStatusDlg::OnBnClickedBtnGoNo2)
	ON_BN_CLICKED(IDC_BTN_GO_NO3, &CPowerMasterStatusDlg::OnBnClickedBtnGoNo3)
	ON_BN_CLICKED(IDC_BTN_PICKRECOG, &CPowerMasterStatusDlg::OnBnClickedBtnPickrecog)
	ON_BN_CLICKED(IDC_BTN_GO_FEEDER, &CPowerMasterStatusDlg::OnBnClickedBtnGoFeeder)
	ON_BN_CLICKED(IDC_BTN_TEACH_FEEDER, &CPowerMasterStatusDlg::OnBnClickedBtnTeachFeeder)
	ON_BN_CLICKED(IDC_CHK_BTN_ONLY_RECOG, &CPowerMasterStatusDlg::OnBnClickedChkBtnOnlyRecog)
	ON_CBN_SELCHANGE(IDC_CB_SPEED_XY, &CPowerMasterStatusDlg::OnCbnSelchangeCbSpeedXy)
	ON_BN_CLICKED(IDC_BTN_RUN, &CPowerMasterStatusDlg::OnBnClickedBtnRun)
	ON_BN_CLICKED(IDC_CHK_BTN_APPLY_VIS, &CPowerMasterStatusDlg::OnBnClickedChkBtnApplyVis)
	ON_BN_CLICKED(IDC_BTN_TEACH_NO4, &CPowerMasterStatusDlg::OnBnClickedBtnTeachNo4)
	ON_BN_CLICKED(IDC_BTN_GO_NO4, &CPowerMasterStatusDlg::OnBnClickedBtnGoNo4)
	ON_BN_CLICKED(IDC_CHK_BTN_RETURN_FD, &CPowerMasterStatusDlg::OnBnClickedChkBtnReturnFd)
	ON_BN_CLICKED(IDC_BTN_RETURNTOFD, &CPowerMasterStatusDlg::OnBnClickedBtnReturntofd)
	ON_BN_CLICKED(IDC_CHECK1, &CPowerMasterStatusDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BTN_READJOBFILE, &CPowerMasterStatusDlg::OnBnClickedBtnReadjobfile)
	ON_CBN_SELCHANGE(IDC_CB_MACHINE_SPEED, &CPowerMasterStatusDlg::OnCbnSelchangeCbMachineSpeed)
//	ON_EN_CHANGE(IDC_EDIT_PICK_Z2, &CPowerMasterStatusDlg::OnEnChangeEditPickZ2)
	ON_EN_CHANGE(IDC_EDIT_HM_DELAY, &CPowerMasterStatusDlg::OnEnChangeEditHmDelay)
END_MESSAGE_MAP()


// CPowerMasterStatusDlg 진단

#ifdef _DEBUG
void CPowerMasterStatusDlg::AssertValid() const
{
	CFormView::AssertValid();
}

#ifndef _WIN32_WCE
void CPowerMasterStatusDlg::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif
#endif //_DEBUG


// CPowerMasterStatusDlg 메시지 처리기
void CPowerMasterStatusDlg::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();

	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
}


BOOL CPowerMasterStatusDlg::Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	return CFormView::Create(lpszClassName, lpszWindowName, dwStyle, rect, pParentWnd, nID, pContext);
}

BOOL CPowerMasterStatusDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	return CFormView::PreTranslateMessage(pMsg);
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachOrigin()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachMark1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachMark2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachNo1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachNo2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachNo3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachNo4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnShowWindow(BOOL bShow, UINT nStatus)
{
	CFormView::OnShowWindow(bShow, nStatus);
	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
	Point_XYRZE pt;
	CString strMsg;
	if (m_bShow == true)
	{
		strMsg.Format(_T("30"));
		SetDlgItemText(IDC_EDIT_FD_NO, strMsg);
		for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
		{
			strMsg.Format(_T("Head%02d"), indx + 1);
			m_cbBtnPickHead.InsertString(indx, strMsg);
			m_cbBtnInsertNo1.InsertString(indx, strMsg);
			m_cbBtnInsertNo2.InsertString(indx, strMsg);
			m_cbBtnInsertNo3.InsertString(indx, strMsg);
			m_cbBtnInsertNo4.InsertString(indx, strMsg);
		}
		m_cbBtnPickHead.SetCurSel(0);
		m_cbBtnInsertNo1.SetCurSel(1);
		m_cbBtnInsertNo2.SetCurSel(1);
		m_cbBtnInsertNo3.SetCurSel(1);
		m_cbBtnInsertNo4.SetCurSel(1);

		strMsg.Format(_T("0.000"));
		SetDlgItemText(IDC_EDIT_PICKOFFSET_X, strMsg);
		SetDlgItemText(IDC_EDIT_PICKOFFSET_Y, strMsg);
		SetDlgItemText(IDC_EDIT_RES_X, strMsg);
		SetDlgItemText(IDC_EDIT_RES_Y, strMsg);
		SetDlgItemText(IDC_EDIT_RES_R, strMsg);
		
		strMsg.Format(_T("0.000"));
		SetDlgItemText(IDC_EDIT_PICK_R, strMsg);

		strMsg.Format(_T("%.3f"), GetInsertByZ(FRONT_GANTRY));
		SetDlgItemText(IDC_EDIT_PICK_Z, strMsg);

		for (int indx = 0; indx < 10; ++indx)
		{
			strMsg.Format(_T("%03d%%"), (indx + 1) * 10);
			m_cbBtnPickSpeedXY.InsertString(indx, strMsg);
			m_cbBtnPickSpeedR.InsertString(indx, strMsg);
			m_cbBtnPickSpeedZ.InsertString(indx, strMsg);
			m_cbBtnMachineSpeed.InsertString(indx, strMsg);
		}
		m_cbBtnPickSpeedXY.SetCurSel(2);
		m_cbBtnPickSpeedR.SetCurSel(2);
		m_cbBtnPickSpeedZ.SetCurSel(2);
		m_cbBtnMachineSpeed.SetCurSel(9);
		strMsg.Format(_T("100"));
		SetDlgItemText(IDC_EDIT_PICK_DELAY, strMsg);
		strMsg.Format(_T("1.0"));
		SetDlgItemText(IDC_EDIT_PCB_THICKNESS, strMsg);
		strMsg.Format(_T("10.0"));
		SetDlgItemText(IDC_EDIT_PCB_STANDBYZ_OFFSET, strMsg);
		strMsg.Format(_T("20.0"));
		SetDlgItemText(IDC_EDIT_COMPONENT_HEIGHT, strMsg);
		m_ChkBtnUseRecogByInsertAngle.SetCheck(1);
		m_bShow = false;
	}

	if (bShow == 1)
	{
		if (gcPowerCalibrationData)
		{
			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_POINT_NO);
			TRACE(_T("[PWR] Origin Get X,Y,R,Z,%.3f,%.3f\n"), pt.x, pt.y);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_ORIGIN_X, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_ORIGIN_Y, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_MARK1);
			TRACE(_T("[PWR] Mark1 Get X,Y,%.3f,%.3f\n"), pt.x, pt.y);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_MARK_X1, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_MARK_Y1, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_MARK2);
			TRACE(_T("[PWR] Mark2 Get X,Y,%.3f,%.3f\n"), pt.x, pt.y);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_MARK_X2, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_MARK_Y2, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_INSERT_NO1);
			TRACE(_T("[PWR] No1 Get X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), pt.x, pt.y, pt.r, pt.z);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_NO_X1, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_NO_Y1, strMsg);
			strMsg.Format(_T("%.3f"), pt.r);
			SetDlgItemText(IDC_EDIT_NO_R1, strMsg);
			strMsg.Format(_T("%.3f"), pt.z);
			SetDlgItemText(IDC_EDIT_NO_Z1, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_INSERT_NO2);
			TRACE(_T("[PWR] No2 Get X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), pt.x, pt.y, pt.r, pt.z);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_NO_X2, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_NO_Y2, strMsg);
			strMsg.Format(_T("%.3f"), pt.r);
			SetDlgItemText(IDC_EDIT_NO_R2, strMsg);
			strMsg.Format(_T("%.3f"), pt.z);
			SetDlgItemText(IDC_EDIT_NO_Z2, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_INSERT_NO3);
			TRACE(_T("[PWR] No3 Get X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), pt.x, pt.y, pt.r, pt.z);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_NO_X3, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_NO_Y3, strMsg);
			strMsg.Format(_T("%.3f"), pt.r);
			SetDlgItemText(IDC_EDIT_NO_R3, strMsg);
			strMsg.Format(_T("%.3f"), pt.z);
			SetDlgItemText(IDC_EDIT_NO_Z3, strMsg);

			pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, ORIGIN_INSERT_NO4);
			TRACE(_T("[PWR] No4 Get X,Y,R,Z,%.3f,%.3f,%.3f,%.3f\n"), pt.x, pt.y, pt.r, pt.z);
			strMsg.Format(_T("%.3f"), pt.x);
			SetDlgItemText(IDC_EDIT_NO_X4, strMsg);
			strMsg.Format(_T("%.3f"), pt.y);
			SetDlgItemText(IDC_EDIT_NO_Y4, strMsg);
			strMsg.Format(_T("%.3f"), pt.r);
			SetDlgItemText(IDC_EDIT_NO_R4, strMsg);
			strMsg.Format(_T("%.3f"), pt.z);
			SetDlgItemText(IDC_EDIT_NO_Z4, strMsg);
		}
	}
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoOrigin()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoMark1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoMark2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

void CPowerMasterStatusDlg::OnBnClickedBtnGoNo1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoNo2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoNo3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoNo4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

Point_XY CPowerMasterStatusDlg::GetMarkPosition(long MarkNo)
{
	Point_XY pt;
	ZeroMemory(&pt, sizeof(pt));
	return pt;
}

long CPowerMasterStatusDlg::GetFeederNo()
{
	CString str;
	long FdNo = 0;
	GetDlgItemText(IDC_EDIT_FD_NO, str);
	if (str.GetLength() > 0)
	{
		FdNo = ConvertCStringToInt(str);
	}
	return FdNo;
}

long CPowerMasterStatusDlg::GetHeadNo()
{
	long HeadNo = TBL_CAMERA;
	if (m_cbBtnPickHead.GetCurSel() == 0)
		HeadNo = TBL_HEAD1;
	else if (m_cbBtnPickHead.GetCurSel() == 1)
		HeadNo = TBL_HEAD2;
	else if (m_cbBtnPickHead.GetCurSel() == 2)
		HeadNo = TBL_HEAD3;
	else if (m_cbBtnPickHead.GetCurSel() == 3)
		HeadNo = TBL_HEAD4;
	else if (m_cbBtnPickHead.GetCurSel() == 4)
		HeadNo = TBL_HEAD5;
	else if (m_cbBtnPickHead.GetCurSel() == 5)
		HeadNo = TBL_HEAD6;
	else
		HeadNo = TBL_CAMERA;
	return HeadNo;
}

Point_XYRZ CPowerMasterStatusDlg::GetPickOffset()
{
	CString str;
	Point_XYRZ PickOffset;
	ZeroMemory(&PickOffset, sizeof(PickOffset));
	GetDlgItemText(IDC_EDIT_PICKOFFSET_X, str);
	if (str.GetLength() > 0)
	{
		PickOffset.x = ConvertCStringToDouble(str);
	}
	GetDlgItemText(IDC_EDIT_PICKOFFSET_Y, str);
	if (str.GetLength() > 0)
	{
		PickOffset.y = ConvertCStringToDouble(str);
	}
	GetDlgItemText(IDC_EDIT_PICK_R, str);
	if (str.GetLength() > 0)
	{
		PickOffset.r = ConvertCStringToDouble(str);
	}
	GetDlgItemText(IDC_EDIT_PICK_Z, str);
	if (str.GetLength() > 0)
	{
		PickOffset.z = ConvertCStringToDouble(str);
	}
	return PickOffset;
}

void CPowerMasterStatusDlg::SetPickOffset(Point_XYRZ PickOffset)
{
	CString str;
	str.Format(_T("%.3f"), PickOffset.x);
	SetDlgItemText(IDC_EDIT_PICKOFFSET_X, str);
	str.Format(_T("%.3f"), PickOffset.y);
	SetDlgItemText(IDC_EDIT_PICKOFFSET_Y, str);
}

Ratio_XYRZ CPowerMasterStatusDlg::GetPickRatio()
{
	Ratio_XYRZ PickRatio;
	if (m_cbBtnPickSpeedXY.GetCurSel() == 0)
		PickRatio.xy = 0.1;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 1)
		PickRatio.xy = 0.2;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 2)
		PickRatio.xy = 0.3;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 3)
		PickRatio.xy = 0.4;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 4)
		PickRatio.xy = 0.5;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 5)
		PickRatio.xy = 0.6;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 6)
		PickRatio.xy = 0.7;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 7)
		PickRatio.xy = 0.8;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 8)
		PickRatio.xy = 0.9;
	else if (m_cbBtnPickSpeedXY.GetCurSel() == 9)
		PickRatio.xy = 1.0;
	else
		PickRatio.xy = 0.5;
	if (m_cbBtnPickSpeedR.GetCurSel() == 0)
		PickRatio.r = 0.1;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 1)
		PickRatio.r = 0.2;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 2)
		PickRatio.r = 0.3;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 3)
		PickRatio.r = 0.4;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 4)
		PickRatio.r = 0.5;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 5)
		PickRatio.r = 0.6;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 6)
		PickRatio.r = 0.7;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 7)
		PickRatio.r = 0.8;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 8)
		PickRatio.r = 0.9;
	else if (m_cbBtnPickSpeedR.GetCurSel() == 9)
		PickRatio.r = 1.0;
	else
		PickRatio.r = 0.5;
	if (m_cbBtnPickSpeedZ.GetCurSel() == 0)
		PickRatio.z = 0.1;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 1)
		PickRatio.z = 0.2;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 2)
		PickRatio.z = 0.3;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 3)
		PickRatio.z = 0.4;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 4)
		PickRatio.z = 0.5;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 5)
		PickRatio.z = 0.6;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 6)
		PickRatio.z = 0.7;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 7)
		PickRatio.z = 0.8;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 8)
		PickRatio.z = 0.9;
	else if (m_cbBtnPickSpeedZ.GetCurSel() == 9)
		PickRatio.z = 1.0;
	else
		PickRatio.z = 0.5;
	return PickRatio;
}


Ratio_XYRZ CPowerMasterStatusDlg::GetMachineRatio()
{
	Ratio_XYRZ PickRatio;
	if (m_cbBtnMachineSpeed.GetCurSel() == 0)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.1;		 
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 1)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.2;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 2)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.3;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 3)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.4;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 4)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.5;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 5)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.6;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 6)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.7;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 7)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.8;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 8)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.9;
	}
	else if (m_cbBtnMachineSpeed.GetCurSel() == 9)
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 1.0;
	}
	else
	{
		PickRatio.xy = PickRatio.z = PickRatio.r = 0.5;
	}

	return PickRatio;
}



long CPowerMasterStatusDlg::GetHeadNoFromInsertOrder(long insertOrd)
{
	long HeadSel = m_cbBtnInsertNo1.GetCurSel(), HeadNo = TBL_HEAD1;
	if (HeadSel == 0)
	{
		HeadNo = TBL_HEAD1;
	}
	else if (HeadSel == 1)
	{
		HeadNo = TBL_HEAD2;
	}
	TRACE(_T("[PWR] GetHeadNoFromInsertOrder:%d HeadNo:%d\n"), insertOrd, HeadNo);
	return HeadNo;
}

long CPowerMasterStatusDlg::GetPickDelayFromFdNo(long FdNo)
{
	long PickDelay;
	PickDelay = GetPickDelay();
	return PickDelay;
}

Ratio_XYRZ CPowerMasterStatusDlg::GetCompRatioFromFdNo(long FdNo)
{
	Ratio_XYRZ ratio;
	ratio = GetPickRatio();
	return ratio;
}

Point_XYRZ CPowerMasterStatusDlg::GetPickOffsetFromFdNo(long FdNo)
{
	Point_XYRZ pt;
	pt = GetPickOffset();
	return pt;
}

long CPowerMasterStatusDlg::GetFdNoFromPickOrder(long insertOrd)
{
	long FdNo;
	FdNo = GetFeederNo();
	return FdNo;
}

long CPowerMasterStatusDlg::GetFdNoFromInsertOrder(long insertOrd)
{
	long FdNo;
	FdNo = GetFeederNo();
	return FdNo;
}

long CPowerMasterStatusDlg::GetInsertDelayFromInsertOrder(long insertOrd)
{
	CString str;
	long PickDelay = TIME20MS;
	GetDlgItemText(IDC_EDIT_PICK_DELAY, str);
	if (str.GetLength() > 0)
	{
		PickDelay = ConvertCStringToInt(str);
	}
	return PickDelay;
}

Ratio_XYRZ CPowerMasterStatusDlg::GetComponentRatioByFdNo(long FdNo)
{
	Ratio_XYRZ Ratio;
	Ratio = GetPickRatio();
	return Ratio;
}

double CPowerMasterStatusDlg::GetComponentHeight(long insertOrd)
{
	double ComponentHeight = 5.0;
	CString strComponentHeight;
	GetDlgItemText(IDC_EDIT_COMPONENT_HEIGHT, strComponentHeight);
	ComponentHeight = ConvertCStringToDouble(strComponentHeight);
	return ComponentHeight;
}

double CPowerMasterStatusDlg::GetVAAngleFromInsertOrder(long insertOrd)
{
	double RecognitionAngle = 0.0;
	if (m_ChkBtnUseRecogByInsertAngle.GetCheck() == 1)
	{
		CString strR1;
		GetDlgItemText(IDC_EDIT_NO_R1, strR1);
		RecognitionAngle = ConvertCStringToDouble(strR1);
	}
	else
	{
		RecognitionAngle = 0.0;
	}
	return RecognitionAngle;
}

double CPowerMasterStatusDlg::GetVAAngleOffsetFromInsertOrder(long insertOrd)
{
	return m_VAAngleOffset[insertOrd];
}

void CPowerMasterStatusDlg::SetVAAngleOffsetFromInsertOrder(long insertOrd, double AngleOffset)
{

	m_VAAngleOffset[insertOrd] = AngleOffset;
}

long CPowerMasterStatusDlg::GetPickDelay()
{
	long PickDelay = TIME20MS;
	return PickDelay;
}

void CPowerMasterStatusDlg::ReturnComponentToFeeder(long Gantry)
{
}

void CPowerMasterStatusDlg::DiscardOneBeforePicking(long Gantry, long HeadNo)
{

}

void CPowerMasterStatusDlg::DiscardAllBeforePicking(long Gantry)
{

}

void CPowerMasterStatusDlg::DiscardOneAfterAlignChecking(long Gantry, long HeadNo)
{

}

void CPowerMasterStatusDlg::DiscardAllAfterAlignChecking(long Gantry)
{

}

void CPowerMasterStatusDlg::DiscardOneAfterInserting(long Gantry, long HeadNo)
{

}

void CPowerMasterStatusDlg::DiscardAllAfterInserting(long Gantry)
{

}

long CPowerMasterStatusDlg::Picking(long Gantry)
{
	long Err = NO_ERR;
	return Err;
}

long CPowerMasterStatusDlg::GetMaxPickOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = m_MaxPickOrder;
	return RetMaxOrder;
}

void CPowerMasterStatusDlg::SetMaxPickOrder(long MaxPickOrd)
{
	if (MaxPickOrd > 0 && MaxPickOrd <= MAXUSEDHEADNO)
	{
		m_MaxPickOrder = MaxPickOrd;
	}
}

long CPowerMasterStatusDlg::GetMaxInsertOrder()
{
	long RetMaxOrder = 0;
	RetMaxOrder = m_MaxInsertOrder;
	return RetMaxOrder;
}

void CPowerMasterStatusDlg::SetMaxInsertOrder(long MaxInsertOrd)
{
	if (MaxInsertOrd > 0 && MaxInsertOrd <= MAXUSEDHEADNO)
	{
		m_MaxInsertOrder = MaxInsertOrd;
	}
}

long CPowerMasterStatusDlg::CheckingAlign(long Gantry)
{
	long Err = NO_ERR;
	return Err;
}

long CPowerMasterStatusDlg::WaitInPcb(long Conveyor)
{
	long Err = NO_ERR;
	return Err;
}

long CPowerMasterStatusDlg::CheckingMark(long Gantry)
{
	long Ret = NO_ERR;
	return Ret;
}

long CPowerMasterStatusDlg::Inserting(long Gantry)
{
	long Err = NO_ERR;
	return Err;
}

void CPowerMasterStatusDlg::OnBnClickedBtnPickrecog()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnGoFeeder()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnTeachFeeder()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedChkBtnOnlyRecog()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnOnlyRecog.GetCheck() == 1)
	{
		m_ChkBtnReturnComponentToFeeder.SetCheck(0);
	}
}


void CPowerMasterStatusDlg::OnCbnSelchangeCbSpeedXy()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedBtnRun()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

long CPowerMasterStatusDlg::GetPickupHeadNo(long PickOrd)
{
	long HeadSel = m_cbBtnInsertNo1.GetCurSel(), HeadNo = TBL_HEAD1;
	if (HeadSel == 0)
	{
		HeadNo = TBL_HEAD1;
	}
	else if (HeadSel == 1)
	{
		HeadNo = TBL_HEAD2;
	}
	return HeadNo;
}

long CPowerMasterStatusDlg::GetInsertHeadNo(long InsertOrd)
{
	long HeadSel = m_cbBtnInsertNo1.GetCurSel(), HeadNo = TBL_CAMERA;
	if (HeadSel == 0)
	{
		HeadNo = TBL_HEAD1;
	}
	else if (HeadSel == 1)
	{
		HeadNo = TBL_HEAD2;
	}
	return HeadNo;
}

Point_XYRZ CPowerMasterStatusDlg::GetInsertPoint(long insertNo)
{
	Point_XYRZ ptRet;
	CString strMsg;
	ZeroMemory(&ptRet, sizeof(ptRet));
	if (insertNo == 0)
	{
		GetDlgItemText(IDC_EDIT_NO_X1, strMsg);
		ptRet.x = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Y1, strMsg);
		ptRet.y = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_R1, strMsg);
		ptRet.r = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Z1, strMsg);
		ptRet.z = ConvertCStringToDouble(strMsg);
	}
	else if (insertNo == 1)
	{
		GetDlgItemText(IDC_EDIT_NO_X2, strMsg);
		ptRet.x = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Y2, strMsg);
		ptRet.y = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_R2, strMsg);
		ptRet.r = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Z2, strMsg);
		ptRet.z = ConvertCStringToDouble(strMsg);
	}
	else if (insertNo == 2)
	{
		GetDlgItemText(IDC_EDIT_NO_X3, strMsg);
		ptRet.x = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Y3, strMsg);
		ptRet.y = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_R3, strMsg);
		ptRet.r = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Z3, strMsg);
		ptRet.z = ConvertCStringToDouble(strMsg);
	}
	else if (insertNo == 3)
	{
		GetDlgItemText(IDC_EDIT_NO_X4, strMsg);
		ptRet.x = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Y4, strMsg);
		ptRet.y = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_R4, strMsg);
		ptRet.r = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_NO_Z4, strMsg);
		ptRet.z = ConvertCStringToDouble(strMsg);
	}
	if (ptRet.z < GetStandByZ(FRONT_GANTRY))
	{
		ptRet.z = GetStandByZ(FRONT_GANTRY);
	}
	return ptRet;
}

void CPowerMasterStatusDlg::StartConveyor()
{
}

void CPowerMasterStatusDlg::StartSyncGantryConveyor()
{
}

void CPowerMasterStatusDlg::SetPcbInfoConveyor()
{
}

void CPowerMasterStatusDlg::RunConveyor(bool ContinueRun)
{
}

void CPowerMasterStatusDlg::RunSyncGantryConveyor()
{
}

void CPowerMasterStatusDlg::StopConveyor()
{
}

void CPowerMasterStatusDlg::StopSyncGantryConveyor()
{
}


bool CPowerMasterStatusDlg::IsRemainInsertion()
{
	return true;
}

UINT CPowerMasterStatusDlg::Run(LPVOID wParam)
{
	return 0;
}

void CPowerMasterStatusDlg::OnBnClickedChkBtnApplyVis()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedChkBtnReturnFd()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (m_ChkBtnReturnComponentToFeeder.GetCheck() == 1)
	{
		m_ChkBtnOnlyRecog.SetCheck(0);
	}
}


void CPowerMasterStatusDlg::OnBnClickedBtnReturntofd()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnBnClickedCheck1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

long CPowerMasterStatusDlg::GoOrigin()
{
	long Gantry = FRONT_GANTRY, Err = NO_ERR;
	return Err;
}

long CPowerMasterStatusDlg::GetConveyorRunMode()
{
	return m_ConveyorRunMode;
}

void CPowerMasterStatusDlg::SetConveyorRunMode(long ConveyorRunMode)
{
	m_ConveyorRunMode = ConveyorRunMode;
}

void CPowerMasterStatusDlg::OnBnClickedBtnReadjobfile()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	//gcReadJobFile->ReadFile();
}


void CPowerMasterStatusDlg::OnCbnSelchangeCbMachineSpeed()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.

	Ratio_XYRZ ratio = GetMachineRatio();
	gcMeasureHeight->SetRatio(ratio);


}


void CPowerMasterStatusDlg::OnEnChangeEditPickZ2()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CFormView::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerMasterStatusDlg::OnEnChangeEditHmDelay()
{
	// TODO:  RICHEDIT 컨트롤인 경우, 이 컨트롤은
	// CFormView::OnInitDialog() 함수를 재지정 
	//하고 마스크에 OR 연산하여 설정된 ENM_CHANGE 플래그를 지정하여 CRichEditCtrl().SetEventMask()를 호출하지 않으면
	// 이 알림 메시지를 보내지 않습니다.

	// TODO:  여기에 컨트롤 알림 처리기 코드를 추가합니다.

	CString strMsg;
	GetDlgItemText(IDC_EDIT_HM_DELAY, strMsg);

	if (strMsg.GetLength() > 0)
	{
		long indexR = ConvertCStringToInt(strMsg);
		gcMeasureHeight->SetDelay(indexR);
	}

	
}
