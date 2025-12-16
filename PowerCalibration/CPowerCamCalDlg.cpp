// CPowerCamCalDlg.cpp: 구현 파일
//

#include "pch.h"
#include "PowerCalibration.h"
#include "PowerCalibrationDlg.h"
#include "CPowerCamCalDlg.h"
#include "CTokenizer.h"

/*****************************************************************************/
/* Header                                                                    */
/*****************************************************************************/
#include "GlobalDefine.h"
#include "GlobalIODefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "EthernetVision.h"
#include "CEntryConveyor.h"
#include "CWorkConveyor.h"
#include "CExitConveyor.h"
#include "CPowerConveyorData.h"
#include "CReturnToEntrance.h"
#include "AxisInformation.h"
#include "CPowerConveyorControl.h"
//#include "ErrorCode.h"

/*****************************************************************************/
/* Name Space                                                                */
/*****************************************************************************/

// CPowerCamCalDlg
IMPLEMENT_DYNCREATE(CPowerCamCalDlg, CFormView)

CPowerCamCalDlg::CPowerCamCalDlg()
	: CFormView(IDD_FORM_CAMCAL)
{
}

CPowerCamCalDlg::~CPowerCamCalDlg()
{
}

void CPowerCamCalDlg::DoDataExchange(CDataExchange* pDX)
{
	CFormView::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_RADIO1, m_RbPlate10);
	DDX_Control(pDX, IDC_RADIO2, m_RbPlate1);
	DDX_Control(pDX, IDC_RADIO3, m_RbPlate01);
	DDX_Control(pDX, IDC_RADIO4, m_RbWidth10);
	DDX_Control(pDX, IDC_RADIO5, m_RbWidth1);
	DDX_Control(pDX, IDC_RADIO6, m_RbWidth01);
	DDX_Control(pDX, IDC_RADIO7, m_RbReverseOff);
	DDX_Control(pDX, IDC_RADIO8, m_RbReverseOn);
	DDX_Control(pDX, IDC_RADIO9, m_RbPlate5);
	DDX_Control(pDX, IDC_RADIO10, m_RbWidth5);
}

BEGIN_MESSAGE_MAP(CPowerCamCalDlg, CFormView)
	ON_BN_CLICKED(IDC_BTN_READFILE, &CPowerCamCalDlg::OnBnClickedBtnReadfile)
	ON_BN_CLICKED(IDC_BTN_INBELT, &CPowerCamCalDlg::OnBnClickedBtnInbelt)
	ON_BN_CLICKED(IDC_BTN_WORKBELT, &CPowerCamCalDlg::OnBnClickedBtnWorkbelt)
	ON_BN_CLICKED(IDC_BTN_OUTBELT, &CPowerCamCalDlg::OnBnClickedBtnOutbelt)
	ON_BN_CLICKED(IDC_BTN_PLATE, &CPowerCamCalDlg::OnBnClickedBtnPlate)
	ON_BN_CLICKED(IDC_BTN_WIDTH, &CPowerCamCalDlg::OnBnClickedBtnWidth)
	ON_BN_CLICKED(IDC_BTN_STOPPER, &CPowerCamCalDlg::OnBnClickedBtnStopper)
	ON_BN_CLICKED(IDC_BTN_PLATE2, &CPowerCamCalDlg::OnBnClickedBtnPlate2)
	ON_BN_CLICKED(IDC_BTN_WIDTH2, &CPowerCamCalDlg::OnBnClickedBtnWidth2)
	ON_BN_CLICKED(IDC_RADIO1, &CPowerCamCalDlg::OnBnClickedRadio1)
	ON_BN_CLICKED(IDC_RADIO2, &CPowerCamCalDlg::OnBnClickedRadio2)
	ON_BN_CLICKED(IDC_RADIO3, &CPowerCamCalDlg::OnBnClickedRadio3)
	ON_BN_CLICKED(IDC_RADIO4, &CPowerCamCalDlg::OnBnClickedRadio4)
	ON_BN_CLICKED(IDC_RADIO5, &CPowerCamCalDlg::OnBnClickedRadio5)
	ON_BN_CLICKED(IDC_RADIO6, &CPowerCamCalDlg::OnBnClickedRadio6)
	ON_BN_CLICKED(IDC_RADIO7, &CPowerCamCalDlg::OnBnClickedRadio7)
	ON_BN_CLICKED(IDC_RADIO8, &CPowerCamCalDlg::OnBnClickedRadio8)
	ON_BN_CLICKED(IDC_BTN_SETCONVWIDTH, &CPowerCamCalDlg::OnBnClickedBtnSetconvwidth)
	ON_BN_CLICKED(IDC_BTN_UPDATE_CONV, &CPowerCamCalDlg::OnBnClickedBtnUpdateConv)
	ON_BN_CLICKED(IDC_BTN_SETCONVPUSHERZ, &CPowerCamCalDlg::OnBnClickedBtnSetconvpusherz)
	ON_BN_CLICKED(IDC_RADIO9, &CPowerCamCalDlg::OnBnClickedRadio9)
	ON_BN_CLICKED(IDC_RADIO10, &CPowerCamCalDlg::OnBnClickedRadio10)
	ON_BN_CLICKED(IDC_BTN_SETCONVPUSERZ_ORIGIN, &CPowerCamCalDlg::OnBnClickedBtnSetconvpuserzOrigin)
	ON_BN_CLICKED(IDC_BTN_SETCONVWIDTH_ORIGIN, &CPowerCamCalDlg::OnBnClickedBtnSetconvwidthOrigin)
	ON_BN_CLICKED(IDC_BTN_MANUAL, &CPowerCamCalDlg::OnBnClickedBtnManual)
	ON_BN_CLICKED(IDC_BTN_CALC_CONVPUSERZ_ORIGIN2, &CPowerCamCalDlg::OnBnClickedBtnCalcConvpuserzOrigin2)
END_MESSAGE_MAP()


// CPowerCamCalDlg 진단

#ifdef _DEBUG
void CPowerCamCalDlg::AssertValid() const
{
	CFormView::AssertValid();
}

#ifndef _WIN32_WCE
void CPowerCamCalDlg::Dump(CDumpContext& dc) const
{
	CFormView::Dump(dc);
}
#endif
#endif //_DEBUG


// CPowerCamCalDlg 메시지 처리기


void CPowerCamCalDlg::OnInitialUpdate()
{
	CFormView::OnInitialUpdate();

	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
}

BOOL CPowerCamCalDlg::Create(LPCTSTR lpszClassName, LPCTSTR lpszWindowName, DWORD dwStyle, const RECT& rect, CWnd* pParentWnd, UINT nID, CCreateContext* pContext)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	return CFormView::Create(lpszClassName, lpszWindowName, dwStyle, rect, pParentWnd, nID, pContext);
}

void ReadCamCal()
{
	CStdioFile* cfile = new CStdioFile();
	CString strFileName;
	CString strRead;
	bool bFileRead = false;
	CString strCalibration;
	strCalibration.Empty();
	strFileName.Format(_T("%S%S\\%s"), ROOT_PATH, CONFIG_PATH, _T("newcamcal.sys"));
	if (strFileName.GetLength() > 0)
	{
		bool bOpend = false;
		bOpend = cfile->Open(strFileName, CFile::modeRead, NULL);
		long lFileLen = (long)cfile->GetLength();
		if (bOpend == false)
			TRACE(_T("[PWR] Error Open fail %s\n"), strFileName);
		else
			TRACE(_T("[PWR] Open success %s\n"), strFileName);
		CString strLine;
		while (true)
		{
			bFileRead = cfile->ReadString(strLine);
			if (strLine.CompareNoCase(_T("<New_Camera_Calibration_Data>")) == 0)
				continue;
			if (strLine.CompareNoCase(_T("@EOF")) == 0)
				continue;
			strCalibration += strLine;
			if (bFileRead == false)
			{
				break;
			}
			//TRACE(_T("[PWR] %s\n"), strLine);
		}
		cfile->Close();
	}
	CTokenizer* cTokenizer = new CTokenizer(strCalibration, _T(" "), FALSE);
	double dblValue[32];
	CString strValue;
	long lCamNo = 0, lCamCalNo = 0;
	ZeroMemory(&dblValue, sizeof(dblValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (strValue.Find(_T(".")) >= 0)
		{
			dblValue[lCamCalNo] = cTokenizer->GetDouble(i);
		}
		lCamCalNo++;
		if (lCamCalNo >= 32)
		{
			gcEthernetVision->SetCameraCalibration(lCamNo, dblValue);
			lCamNo++;
			lCamCalNo = 0;
			ZeroMemory(dblValue, sizeof(dblValue));
		}
	}
	delete cTokenizer;
	cTokenizer = NULL;
	delete cfile;
	cfile = NULL;
}

void CPowerCamCalDlg::OnBnClickedBtnReadfile()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcPowerConveyorControl)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_READFILE, strBtnText);
			if (strBtnText.CompareNoCase(_T("Start")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_CONVEYOR_START));
				msgReceived->SetID(gcPowerConveyorControl->GetThreadID());
				msgReceived->SetThreadSubMsg(0, 0, 0);
				gcPowerConveyorControl->Event((LPVOID)msgReceived);
				strBtnText = _T("End");
				SetDlgItemText(IDC_BTN_READFILE, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("End")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_CONVEYOR_END));
				msgReceived->SetID(gcPowerConveyorControl->GetThreadID());
				msgReceived->SetThreadSubMsg(0, 0, 0);
				gcPowerConveyorControl->Event((LPVOID)msgReceived);
				strBtnText = _T("Start");
				SetDlgItemText(IDC_BTN_READFILE, strBtnText);
			}
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnInbelt()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	GetDlgItemText(IDC_BTN_INBELT, strMsg);
	if (GetRunMode() == NORMAL_MODE)
	{
		if (strMsg.CompareNoCase(_T("In Belt On")) == 0)
		{
			JogInfo* jogCmd = new JogInfo();
			jogCmd->Acc = 50000.0;
			jogCmd->Dec = 50000.0;
			if (m_RbReverseOff.GetCheck() == 1)
			{
				jogCmd->MaxVel = 5000.0 * (1.0);
				StartOneJog(_T("FBTIN"), *jogCmd);
			}
			else
			{
				jogCmd->MaxVel = 5000.0 * (-1.0);
				StartOneJog(_T("FBTIN"), *jogCmd);
			}
			SetDlgItemText(IDC_BTN_INBELT, _T("In Belt Off"));
		}
		else
		{
			double Dec = 100000.0;
			StopOne(_T("FBTIN"), Dec);
			SetDlgItemText(IDC_BTN_INBELT, _T("In Belt On"));
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnWorkbelt()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	GetDlgItemText(IDC_BTN_WORKBELT, strMsg);
	if (GetRunMode() == NORMAL_MODE)
	{
		if (strMsg.CompareNoCase(_T("Work Belt On")) == 0)
		{
			JogInfo* jogCmd = new JogInfo();
			jogCmd->Acc = 50000.0;
			jogCmd->Dec = 50000.0;
			if (m_RbReverseOff.GetCheck() == 1)
			{
				jogCmd->MaxVel = 5000.0 * (1.0);
				StartOneJog(_T("FBTWK"), *jogCmd);
			}
			else
			{
				jogCmd->MaxVel = 5000.0 * (-1.0);
				StartOneJog(_T("FBTWK"), *jogCmd);
			}
			SetDlgItemText(IDC_BTN_WORKBELT, _T("Work Belt Off"));
		}
		else
		{
			double Dec = 100000.0;
			StopOne(_T("FBTWK"), Dec);
			SetDlgItemText(IDC_BTN_WORKBELT, _T("Work Belt On"));
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnOutbelt()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	GetDlgItemText(IDC_BTN_OUTBELT, strMsg);
	if (GetRunMode() == NORMAL_MODE)
	{
		if (strMsg.CompareNoCase(_T("Out Belt On")) == 0)
		{
			JogInfo* jogCmd = new JogInfo();
			jogCmd->Acc = 50000.0;
			jogCmd->Dec = 50000.0;
			if (m_RbReverseOff.GetCheck() == 1)
			{
				jogCmd->MaxVel = 5000.0 * (1.0);
				StartOneJog(_T("FBTOT"), *jogCmd);
			}
			else
			{
				jogCmd->MaxVel = 5000.0 * (-1.0);
				StartOneJog(_T("FBTOT"), *jogCmd);
			}
			SetDlgItemText(IDC_BTN_OUTBELT, _T("Out Belt Off"));
		}
		else
		{
			double Dec = 100000.0;
			StopOne(_T("FBTOT"), Dec);
			SetDlgItemText(IDC_BTN_OUTBELT, _T("Out Belt On"));
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnPlate()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		double Offset = 1.0, Ratio = 0.5;
		long TimeOut = TIME3000MS;
		if (m_RbPlate10.GetCheck() == 1)
			Offset = 10.0;
		else if (m_RbPlate5.GetCheck() == 1)
			Offset = 5.0;
		else if (m_RbPlate1.GetCheck() == 1)
			Offset = 1.0;
		else
			Offset = 0.1;
		CString strPusherZ = GetPusherZName(FRONT_CONV);
		double Position;
		Position = ReadPosition(strPusherZ);
		StartPosWaitMotionSkipLimitCheck(strPusherZ, Ratio, TimeOut, Position + Offset, true);
		WritePusherZ(FRONT_CONV, WORK1_CONV, Position + Offset);
		UpdateConvInfo();
	}
}

void CPowerCamCalDlg::OnBnClickedBtnWidth()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		double Offset = 1.0, Ratio = 0.5;
		long TimeOut = TIME3000MS, Err = NO_ERR;
		if (m_RbWidth10.GetCheck() == 1)
			Offset = 10.0;
		else if (m_RbWidth5.GetCheck() == 1)
			Offset = 5.0;
		else if (m_RbWidth1.GetCheck() == 1)
			Offset = 1.0;
		else
			Offset = 0.1;
		CString strConvWidth = _T("FCONV");
		double Position;
		Position = ReadPosition(strConvWidth);
		Err = StartPosWaitMotion(strConvWidth, Ratio, TimeOut, Position + Offset, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Width(+) %s Target:%.3f Err:%d\n"), strConvWidth, Position + Offset, Err);
		}
		else
		{
			WriteWidth(FRONT_CONV, WORK1_CONV, Position + Offset);
			UpdateConvInfo();
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnStopper()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CString strMsg;
	GetDlgItemText(IDC_BTN_STOPPER, strMsg);
	if (GetRunMode() == NORMAL_MODE)
	{
		if (gcWorkConveyor != NULL)
		{
			if (strMsg.CompareNoCase(_T("Stopper Up")) == 0)
			{
				gcWorkConveyor->UpStopper();
				SetDlgItemText(IDC_BTN_STOPPER, _T("Stopper Dn"));
			}
			else
			{
				gcWorkConveyor->DownStopper();
				SetDlgItemText(IDC_BTN_STOPPER, _T("Stopper Up"));
			}
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnPlate2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		double Offset = 1.0, Ratio = 0.5;
		long TimeOut = TIME3000MS;
		if (m_RbPlate10.GetCheck() == 1)
			Offset = 10.0;
		else if (m_RbPlate5.GetCheck() == 1)
			Offset = 5.0;
		else if (m_RbPlate1.GetCheck() == 1)
			Offset = 1.0;
		else
			Offset = 0.1;
		CString strPusherZ = _T("FPUZ");
		double Position;
		Position = ReadPosition(strPusherZ);
		StartPosWaitMotionSkipLimitCheck(strPusherZ, Ratio, TimeOut, Position - Offset, true);
		WritePusherZ(FRONT_CONV, WORK1_CONV, Position - Offset);
		UpdateConvInfo();
	}
}

void CPowerCamCalDlg::OnBnClickedBtnWidth2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		double Offset = 1.0, Ratio = 0.5;
		long TimeOut = TIME3000MS, Err = NO_ERR;
		if (m_RbWidth10.GetCheck() == 1)
			Offset = 10.0;
		else if (m_RbWidth5.GetCheck() == 1)
			Offset = 5.0;
		else if (m_RbWidth1.GetCheck() == 1)
			Offset = 1.0;
		else
			Offset = 0.1;
		CString strConvWidth = _T("FCONV");
		double Position;
		Position = ReadPosition(strConvWidth);
		Err = StartPosWaitMotion(strConvWidth, Ratio, TimeOut, Position - Offset, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Width(-) %s Target:%.3f Err:%d\n"), strConvWidth, Position - Offset, Err);
		}
		else
		{
			WriteWidth(FRONT_CONV, WORK1_CONV, Position - Offset);
			UpdateConvInfo();
		}
	}
}

void CPowerCamCalDlg::OnBnClickedRadio1()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbPlate10.SetCheck(1);
	m_RbPlate5.SetCheck(0);
	m_RbPlate1.SetCheck(0);
	m_RbPlate01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbPlate10.SetCheck(0);
	m_RbPlate5.SetCheck(0);
	m_RbPlate1.SetCheck(1);
	m_RbPlate01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio3()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbPlate10.SetCheck(0);
	m_RbPlate5.SetCheck(0);
	m_RbPlate1.SetCheck(0);
	m_RbPlate01.SetCheck(1);
}


void CPowerCamCalDlg::OnBnClickedRadio4()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWidth10.SetCheck(1);
	m_RbWidth5.SetCheck(0);
	m_RbWidth1.SetCheck(0);
	m_RbWidth01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio5()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWidth10.SetCheck(0);
	m_RbWidth5.SetCheck(0);
	m_RbWidth1.SetCheck(1);
	m_RbWidth01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio6()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWidth10.SetCheck(0);
	m_RbWidth5.SetCheck(0);
	m_RbWidth1.SetCheck(0);
	m_RbWidth01.SetCheck(1);
}


void CPowerCamCalDlg::OnBnClickedRadio7()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbReverseOff.SetCheck(1);
	m_RbReverseOn.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio8()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbReverseOff.SetCheck(0);
	m_RbReverseOn.SetCheck(1);
}


void CPowerCamCalDlg::OnBnClickedBtnSetconvwidth()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double Width = 0.0, ReadWidth = 999.999;
		GetDlgItemText(IDC_EDIT_CONVWIDTH, strMsg);
		Width = ConvertCStringToDouble(strMsg);
		if (gcPowerConveyorData->IsWidthValidRange(Width) == true)
		{
			gcPowerConveyorData->SetWidth(FRONT_CONV, WORK1_CONV, Width);
			WritePosition(GetConvName(FRONT_GANTRY), Width);
			ThreadSleep(TIME5MS);
			ReadWidth = ReadPosition(GetConvName(FRONT_GANTRY));
			if (abs(Width - ReadWidth) < 0.005)
			{
				WriteWidth(FRONT_CONV, WORK1_CONV, Width);
			}
			UpdateConvInfo();
		}
	}
}

void CPowerCamCalDlg::UpdateConvInfo()
{
	double Width = 0.0, PusherZ = 0.0;
	CString strWidth, strPusherZ, strOn = _T("Exist"), strOff = _T("Empty"), strUp = _T("Up"), strDn = _T("Dn");
	UBYTE OnOff = INOFF;
	if (gcPowerConveyorData)
	{
		PusherZ = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
		Width = gcPowerConveyorData->GetWidth(FRONT_CONV, WORK1_CONV);
	}
	strWidth.Format(_T("%.3f"), Width);
	strPusherZ.Format(_T("%.3f"), PusherZ);
	SetDlgItemText(IDC_EDIT_CONVWIDTH, strWidth);
	SetDlgItemText(IDC_EDIT_CONVPUSHERZ, strPusherZ);
	OnOff = InputOne(IN_FCONV_ENTRY_ENT);
	if(OnOff == INON)
		SetDlgItemText(IDC_EDIT_IN_ENT, strOn);
	else
		SetDlgItemText(IDC_EDIT_IN_ENT, strOff);
	OnOff = InputOne(IN_FCONV_ENTRY_EXIST);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_IN_SET, strOn);
	else
		SetDlgItemText(IDC_EDIT_IN_SET, strOff);
	OnOff = InputOne(IN_FCONV_WORK1_LOW);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_WORK_LOW, strOn);
	else
		SetDlgItemText(IDC_EDIT_WORK_LOW, strOff);
	OnOff = InputOne(IN_FCONV_WORK1_EXIST);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_WORK_SET, strOn);
	else
		SetDlgItemText(IDC_EDIT_WORK_SET, strOff);
	OnOff = InputOne(IN_FCONV_WORK1_OUT);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_WORK_OUT, strOn);
	else
		SetDlgItemText(IDC_EDIT_WORK_OUT, strOff);
	OnOff = InputOne(IN_FCONV_EXIT_EXIST);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_OUT_SET, strOn);
	else
		SetDlgItemText(IDC_EDIT_OUT_SET, strOff);
	OnOff = InputOne(IN_FCONV_WORK1_STOP_UP);
	if (OnOff == INON)
		SetDlgItemText(IDC_EDIT_STOPPER_UP, strUp);
	else
		SetDlgItemText(IDC_EDIT_STOPPER_UP, strDn);
}

void CPowerCamCalDlg::OnBnClickedBtnUpdateConv()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UpdateConvInfo();
}

void CPowerCamCalDlg::OnBnClickedBtnSetconvpusherz()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double PusherZ = 0.0, ReadPusherZ = 999.999;;
		GetDlgItemText(IDC_EDIT_CONVPUSHERZ, strMsg);
		PusherZ = ConvertCStringToDouble(strMsg);
		if (gcPowerConveyorData->IsPusherZValidRange(PusherZ) == true)
		{
			gcPowerConveyorData->SetPusherZ(FRONT_CONV, WORK1_CONV, PusherZ);
			WritePosition(GetPusherZName(FRONT_GANTRY), PusherZ);
			ThreadSleep(TIME5MS);
			ReadPusherZ = ReadPosition(GetPusherZName(FRONT_GANTRY));
			if (abs(PusherZ - ReadPusherZ) < 0.005)
			{
				WritePusherZ(FRONT_CONV, WORK1_CONV, PusherZ);
			}
			UpdateConvInfo();
		}
	}
}


void CPowerCamCalDlg::OnBnClickedRadio9()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbPlate10.SetCheck(0);
	m_RbPlate5.SetCheck(5);
	m_RbPlate1.SetCheck(0);
	m_RbPlate01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedRadio10()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	m_RbWidth10.SetCheck(0);
	m_RbWidth5.SetCheck(1);
	m_RbWidth1.SetCheck(0);
	m_RbWidth01.SetCheck(0);
}


void CPowerCamCalDlg::OnBnClickedBtnSetconvpuserzOrigin()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double PusherZ = 0.0;
		GetDlgItemText(IDC_EDIT_CONVPUSHERZ, strMsg);
		PusherZ = ConvertCStringToDouble(strMsg);
		if (gcPowerConveyorData->IsPusherZValidRange(PusherZ) == true)
		{
			WriteHomePosition(GetPusherZName(FRONT_GANTRY), PusherZ);
		}
	}
}


void CPowerCamCalDlg::OnBnClickedBtnFrontpcbfixpos()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CPowerCamCalDlg::OnBnClickedBtnSetconvwidthOrigin()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double Width = 0.0;
		GetDlgItemText(IDC_EDIT_CONVWIDTH, strMsg);
		Width = ConvertCStringToDouble(strMsg);
		if (gcPowerConveyorData->IsWidthValidRange(Width) == true)
		{
			WriteHomePosition(GetConvName(FRONT_GANTRY), Width);
		}
	}
}

void CPowerCamCalDlg::SetPcbInfoManualConveyor()
{
	if (GetRunMode() == NORMAL_MODE)
	{
		PowerThreadMessage* msgSend = new PowerThreadMessage();
		ThreadId_t id;
		CString strMsg;
		double PcbThickness, PcbStandByZOffset;
		unsigned SubMsg1, SubMsg2, SubMsg3;
		SubMsg1 = SubMsg2 = SubMsg3 = 0;
		msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_PCBINFO));
		GetDlgItemText(IDC_EDIT_PCB_THICKNESS2, strMsg);
		PcbThickness = ConvertCStringToDouble(strMsg);
		GetDlgItemText(IDC_EDIT_PCB_STANDBYZ_OFFSET2, strMsg);
		PcbStandByZOffset = ConvertCStringToDouble(strMsg);
		SubMsg1 = (unsigned)(PcbThickness * 10.0);
		SubMsg2 = (unsigned)(PcbStandByZOffset * 10.0);
		TRACE(_T("[PWR] SetPcbInfoManualConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
		msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
		if (gcPowerConveyorControl)
		{
			gcPowerConveyorControl->GetId(&id);
			msgSend->SetID(id);
			if (gcPowerConveyorControl->PingThread(TIME1MS))
			{
				gcPowerConveyorControl->Event((LPVOID)msgSend);
			}
		}
	}
}

void CPowerCamCalDlg::OnBnClickedBtnManual()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int nSub1, nSub2, nSub3;
	nSub1 = nSub2 = nSub3 = 0;
	if (GetRunMode() == NORMAL_MODE)
	{
		SetPcbInfoManualConveyor();
		if (gcPowerConveyorControl)
		{
			CString strBtnText;
			GetDlgItemText(IDC_BTN_MANUAL, strBtnText);
			if (strBtnText.CompareNoCase(_T("Manual")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_START));
				msgReceived->SetID(gcPowerConveyorControl->GetThreadID());
				msgReceived->SetThreadSubMsg(0, 0, 0);
				gcPowerConveyorControl->Event((LPVOID)msgReceived);
				strBtnText = _T("Stop");
				SetDlgItemText(IDC_BTN_MANUAL, strBtnText);
			}
			else if (strBtnText.CompareNoCase(_T("Stop")) == 0)
			{
				PowerThreadMessage* msgReceived = new PowerThreadMessage();
				msgReceived->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_END));
				msgReceived->SetID(gcPowerConveyorControl->GetThreadID());
				msgReceived->SetThreadSubMsg(0, 0, 0);
				gcPowerConveyorControl->Event((LPVOID)msgReceived);
				strBtnText = _T("Manual");
				SetDlgItemText(IDC_BTN_MANUAL, strBtnText);
			}
		}
	}
}


void CPowerCamCalDlg::OnBnClickedBtnCalcConvpuserzOrigin2()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (GetRunMode() == NORMAL_MODE)
	{
		CString strMsg;
		double PusherZ = 0.0;
		double Diff = 0.0, CurPosition = 0.0, CurHomePosition = 0.0, NewHomePosition = 0.0;
		CString strAxis = GetPusherZName(FRONT_GANTRY);
		CString strNewHome;
		double PcbThickness = 0.0;

		GetDlgItemText(IDC_EDIT_PCB_THICKNESS2, strMsg);
		PcbThickness = ConvertCStringToDouble(strMsg);

		CurPosition = ReadPosition(strAxis);
		Diff = (GetInsertByZ(FRONT_GANTRY) + PcbThickness) - CurPosition;
		CurHomePosition = ReadHomePosition(strAxis);
		NewHomePosition = CurHomePosition + Diff;

		if (gcPowerConveyorData->IsPusherZValidRange(NewHomePosition) == true)
		{
			TRACE(_T("[PWR] %s New Origin Offset %.3f Old %.3f Diff %.3f Thickness:%.3f\n"), strAxis, NewHomePosition, CurHomePosition, Diff, PcbThickness);
			strNewHome.Format(_T("%.3f"), NewHomePosition);
			SetDlgItemText(IDC_EDIT_CONVPUSHERZ, strNewHome);
		}
	}
}
