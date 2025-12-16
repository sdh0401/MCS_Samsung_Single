#include "pch.h"
#include "CAdvancedMotionFile.h"
#include "AxisInformation.h"
#include "CApplicationTime.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CStep.h"

CAdvancedMotionFile* gcAdvancedMotionFile;
CAdvancedMotionFile::CAdvancedMotionFile()
{
	Initial();
	SetDir(_T("C:\\Power\\i6.0\\MCS\\Advanced"));
	SetHeader(_T("Run"));
	SetExtension(_T("Motion"));
	MakeFileName();
}

CAdvancedMotionFile::~CAdvancedMotionFile()
{
}

void CAdvancedMotionFile::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CAdvancedMotionFile::GetFileName()
{
	return m_StrFileName;
}

void CAdvancedMotionFile::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CAdvancedMotionFile::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CAdvancedMotionFile::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CAdvancedMotionFile::Initial()
{
	m_PickXYMs = m_PickZDnMs = m_PickZUpMs = m_PickRMs = TIME30MS;
	m_PickXYInpos = 0.10;
	m_PickZDnInpos = m_PickZUpInpos = 0.050;
	m_PickRInpos = 1.000;

	m_RecogXYMs = m_RecogZDnMs = m_RecogZUpMs = m_RecogRMs = TIME30MS;
	m_RecogXYInpos = 0.010;
	m_RecogZDnInpos = m_RecogZUpInpos = 0.050;
	m_RecogRInpos = 1.000;

	m_InsertXYMs = m_InsertZDnMs = m_InsertZUpMs = m_InsertRMs = TIME30MS;
	m_InsertXYInpos = 0.010;
	m_InsertZDnInpos = m_InsertZUpInpos = 0.050;
	m_InsertRInpos = 0.100;

	m_SettlingTime = TIME200MS;

	m_UseAdvancedMotion = 0;
}

long CAdvancedMotionFile::Pick(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [PICK]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//30,30,30,30,0.10,0.050,0.050,1.0
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile Pick TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		m_PickXYMs = iValue[0];
		m_PickZDnMs = iValue[1];
		m_PickZUpMs = iValue[2];
		m_PickRMs = iValue[3];
		m_PickXYInpos = dValue[0];
		m_PickZDnInpos = dValue[1];
		m_PickZUpInpos = dValue[2];
		m_PickRInpos = dValue[3];
		TRACE(_T("[PWR] Pick Ms[ms] XY:%d ZDn:%d ZUp:%d R:%d\n"), m_PickXYMs, m_PickZDnMs, m_PickZUpMs, m_PickRMs);
		TRACE(_T("[PWR] Pick Inpos[mm] XY:%.3f ZDn:%.3f ZUp:%.3f R:%.3f\n"), m_PickXYInpos, m_PickZDnInpos, m_PickZUpInpos, m_PickRInpos);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_PICK]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::Recognition(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [RECOG]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//30,30,30,30,0.10,0.050,0.050,1.0
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile Recognition TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		m_RecogXYMs = iValue[0];
		m_RecogZDnMs = iValue[1];
		m_RecogZUpMs = iValue[2];
		m_RecogRMs = iValue[3];
		m_RecogXYInpos = dValue[0];
		m_RecogZDnInpos = dValue[1];
		m_RecogZUpInpos = dValue[2];
		m_RecogRInpos = dValue[3];
		TRACE(_T("[PWR] Recog Ms[ms] XY:%d ZDn:%d ZUp:%d R:%d\n"), m_RecogXYMs, m_RecogZDnMs, m_RecogZUpMs, m_RecogRMs);
		TRACE(_T("[PWR] Recog Inpos[mm] XY:%.3f ZDn:%.3f ZUp:%.3f R:%.3f\n"), m_RecogXYInpos, m_RecogZDnInpos, m_RecogZUpInpos, m_RecogRInpos);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_RECOG]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::Insert(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [INSERT]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//30,30,30,30,0.10,0.050,0.050,1.0
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile Insert TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		m_InsertXYMs = iValue[0];
		m_InsertZDnMs = iValue[1];
		m_InsertZUpMs = iValue[2];
		m_InsertRMs = iValue[3];
		m_InsertXYInpos = dValue[0];
		m_InsertZDnInpos = dValue[1];
		m_InsertZUpInpos = dValue[2];
		m_InsertRInpos = dValue[3];
		TRACE(_T("[PWR] Insert Ms[ms] XY:%d ZDn:%d ZUp:%d R:%d\n"), m_InsertXYMs, m_InsertZDnMs, m_InsertZUpMs, m_InsertRMs);
		TRACE(_T("[PWR] Insert Inpos[mm] XY:%.3f ZDn:%.3f ZUp:%.3f R:%.3f\n"), m_InsertXYInpos, m_InsertZDnInpos, m_InsertZUpInpos, m_InsertRInpos);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_INSERT]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::Settling(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [SETTLING]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile Settling TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[10];
		double dValue[10];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		m_SettlingTime = iValue[0];
		TRACE(_T("[PWR] Settling Ms[ms] XY:%d\n"), m_SettlingTime);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_SETTLING]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::ShortDistance(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [SHORTDIST]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile ShortDistance TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[30];
		double dValue[30];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}		
		for (long ShortDistNo = 0; ShortDistNo < MAX_SHORT_DIST_LEVEL; ++ShortDistNo)
		{
			SetShortDist(GetAxisX(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDist(GetAxisY1(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDist(GetAxisY2(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_SHORTDIST]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::ShortDistVelocity(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [SHORTVEL]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile ShortDistVelocity TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[30];
		double dValue[30];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		for (long ShortDistNo = 0; ShortDistNo < MAX_SHORT_DIST_LEVEL; ++ShortDistNo)
		{
			SetShortDistVel(GetAxisX(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDistVel(GetAxisY1(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDistVel(GetAxisY2(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_SHORTVEL]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::ShortDistAccDec(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [SHORTVEL]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			//
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile ShortDistAccDec TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[30];
		double dValue[30];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		for (long ShortDistNo = 0; ShortDistNo < MAX_SHORT_DIST_LEVEL; ++ShortDistNo)
		{
			SetShortDistAccDec(GetAxisX(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDistAccDec(GetAxisY1(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
			SetShortDistAccDec(GetAxisY2(FRONT_GANTRY), ShortDistNo, dValue[ShortDistNo]);
		}
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_SHORTVEL]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::DefaultHeadTorque(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [HEADTORQUE]

	if (bRet == false || str.CompareNoCase(_T("[HEADTORQUEALARM]")) != 0)
	{
		for (long head = 0; head < MAXUSEDHEADNO; head++)
		{
			SetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head, GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head));
			//SetEventToStopByOverTorque(EVENTID_FZ1_OVER_TORQUE, _T("FZ1"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head), GetStandByZ(FRONT_GANTRY));
		}

		return Err;
	}

	bRet = cFile->ReadString(str);			//
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	else
	{
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		//if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CAdvancedMotionFile DefaultHeadTorque TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[30];
		double dValue[30];

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}

		for (long head = 0; head < MAXUSEDHEADNO; head++)
		{
			SetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head, (double)iValue[head]);
			//SetEventToStopByOverTorque(EVENTID_FZ1_OVER_TORQUE, _T("FZ1"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head), GetStandByZ(FRONT_GANTRY));
		}

		//for (long head = 0; head < MAXUSEDHEADNO; head++)
		//{
		//	SetMaxZTorqueLimit(REAR_GANTRY, TBL_HEAD1 + head, (double)iValue[MAXUSEDHEADNO + head]);
		//	//SetEventToStopByOverTorque(EVENTID_FZ1_OVER_TORQUE, _T("FZ1"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1 + head), GetStandByZ(FRONT_GANTRY));
		//}

		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_SHORTVEL]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

long CAdvancedMotionFile::ReadFile()
{
	long Err = NO_ERR;
	//CApplicationTime* pTime = new CApplicationTime();
	ULONGLONG GetTime = 0, Elapsed = 0;
	Initial();
	CStdioFile* cFile = new CStdioFile();
	if (cFile->Open((LPCTSTR)GetFileName(), CFile::modeRead
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] CAdvancedMotionFile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] CAdvancedMotionFile(%s) Open Success\n"), (LPCTSTR)GetFileName());
	GetTime = _time_get();
	Err = Pick(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	//TRACE(_T("[PWR] CAdvancedMotionFile(%s) Pick Elapsed:%d[ms]\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime));
	Err = Recognition(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	//TRACE(_T("[PWR] CAdvancedMotionFile(%s) Recognition Elapsed:%d[ms]\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime));
	Err = Insert(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	//TRACE(_T("[PWR] CAdvancedMotionFile(%s) Insert Elapsed:%d[ms]\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime));
	Err = Settling(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	Err = ShortDistance(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	Err = ShortDistVelocity(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	Err = ShortDistAccDec(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	Err = DefaultHeadTorque(cFile);
	if (Err != NO_ERR)
	{
		//cFile->Close();
		//delete cFile;
		//return Err;
	}
	m_UseAdvancedMotion = 1;
	SetGlobalSettlingDelay(m_SettlingTime);
	TRACE(_T("[PWR] CAdvancedMotionFile(%s) Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cFile->Close();
	delete cFile;
	return Err;
}

void CAdvancedMotionFile::ClearAdvanceMotion()
{
	m_UseAdvancedMotion = 0;
	TRACE(_T("[PWR] ClearAdvanceMotion\n"));
}

long CAdvancedMotionFile::UseAdvanceMotion()
{
	return m_UseAdvancedMotion;
}

long CAdvancedMotionFile::GetPickDelayedMsXY()
{
	return m_PickXYMs;
}

long CAdvancedMotionFile::GetPickDelayedMsZDn()
{
	return m_PickZDnMs;
}

long CAdvancedMotionFile::GetPickDelayedMsZUp()
{
	return m_PickZUpMs;
}

long CAdvancedMotionFile::GetPickDelayedMsR()
{
	return m_PickRMs;
}

double CAdvancedMotionFile::GetPickInposXY()
{
	return m_PickXYInpos;
}

double CAdvancedMotionFile::GetPickInposZDn()
{
	return m_PickZDnInpos;
}

double CAdvancedMotionFile::GetPickInposZUp()
{
	return m_PickZUpInpos;
}

double CAdvancedMotionFile::GetPickInposR()
{
	return m_PickRInpos;
}

long CAdvancedMotionFile::GetRecogDelayedMsXY()
{
	return m_RecogXYMs;
}

long CAdvancedMotionFile::GetRecogDelayedMsZDn()
{
	return m_RecogZDnMs;
}

long CAdvancedMotionFile::GetRecogDelayedMsZUp()
{
	return m_RecogZUpMs;
}

long CAdvancedMotionFile::GetRecogDelayedMsR()
{
	return m_RecogRMs;
}

double CAdvancedMotionFile::GetRecogInposXY()
{
	return m_RecogXYInpos;
}

double CAdvancedMotionFile::GetRecogInposZDn()
{
	return m_RecogZDnInpos;
}

double CAdvancedMotionFile::GetRecogInposZUp()
{
	return m_RecogZUpInpos;
}

double CAdvancedMotionFile::GetRecogInposR()
{
	return m_RecogRInpos;
}

long CAdvancedMotionFile::GetInsertDelayedMsXY()
{
	return m_InsertXYMs;
}

long CAdvancedMotionFile::GetInsertDelayedMsZDn()
{
	return m_InsertZDnMs;
}

long CAdvancedMotionFile::GetInsertDelayedMsZUp()
{
	return m_InsertZUpMs;
}

long CAdvancedMotionFile::GetInsertDelayedMsR()
{
	return m_InsertRMs;
}

double CAdvancedMotionFile::GetInsertInposXY()
{
	return m_InsertXYInpos;
}

double CAdvancedMotionFile::GetInsertInposZDn()
{
	return m_InsertZDnInpos;
}

double CAdvancedMotionFile::GetInsertInposZUp()
{
	return m_InsertZUpInpos;
}

double CAdvancedMotionFile::GetInsertInposR()
{
	return m_InsertRInpos;
}

