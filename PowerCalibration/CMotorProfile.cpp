#include "pch.h"
#include "CMotorProfile.h"
#include "AxisInformation.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CStep.h"

CMotorProfile* gcMotorProfile;
CMotorProfile::CMotorProfile()
{
	Initial();
	SetDir(_T("C:\\Power\\i6.0\\MCS\\Advanced"));
	SetHeader(_T("Motor"));
	SetExtension(_T("Profile"));
	MakeFileName();
	if (GetFlieLineCount() != 62)
		ModifyFile();
	ReadFile();
}

CMotorProfile::~CMotorProfile()
{
}

void CMotorProfile::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CMotorProfile::GetFileName()
{
	return m_StrFileName;
}

void CMotorProfile::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CMotorProfile::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CMotorProfile::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CMotorProfile::Initial()
{
}

long CMotorProfile::GetFlieLineCount()
{
	long Err = NO_ERR;
	CString line;
	int lineCount = 0;
	Initial();
	CStdioFile* cFile = new CStdioFile();
	if (cFile->Open((LPCTSTR)GetFileName(), CFile::modeRead
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == TRUE)
	{
		while (cFile->ReadString(line)) {
			lineCount++;
		}
	}
	cFile->Close();
	delete cFile;
	TRACE(_T("[PWR] MotorProfile Line Count : %d\n"), lineCount);
	return lineCount;
}

long CMotorProfile::ReadFile()
{
	long Err = NO_ERR;
	ULONGLONG GetTime = 0, Elapsed = 0;
	Initial();
	CStdioFile* cFile = new CStdioFile();
	if (cFile->Open((LPCTSTR)GetFileName(), CFile::modeRead
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] CMotorProfile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] CMotorProfile(%s) Open Success\n"), (LPCTSTR)GetFileName());
	GetTime = _time_get();
	Err = ReadProfile(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	TRACE(_T("[PWR] CMotorProfile(%s) Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cFile->Close();
	delete cFile;
	return Err;
}

long CMotorProfile::WriteFile()
{
	long Err = NO_ERR;
	ULONGLONG GetTime = 0, Elapsed = 0;
	CString strProfile;
	Initial();
	CStdioFile* cFile = new CStdioFile();
	if (cFile->Open((LPCTSTR)GetFileName(), 
		CFile::modeWrite
		| CFile::modeCreate
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] CMotorProfile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("WriteFile Open Fail"));
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] CMotorProfile(%s) Open Success\n"), (LPCTSTR)GetFileName());
	GetTime = _time_get();
	cFile->WriteString(_T("[PROFILE]\n"));
	for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
	{
		strProfile.Format(_T("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n"),
			PowerAxisMoveParam[AxisNo].velocity,
			PowerAxisMoveParam[AxisNo].acc,
			PowerAxisMoveParam[AxisNo].dec,
			PowerAxisMoveParam[AxisNo].jerkAcc,
			PowerAxisMoveParam[AxisNo].jerkDec,
			PowerAxisMoveParam[AxisNo].jerkAccRatio,
			PowerAxisMoveParam[AxisNo].jerkDecRatio,
			PowerAxisMoveParam[AxisNo].accTimeMilliseconds,
			PowerAxisMoveParam[AxisNo].decTimeMilliseconds,
			PowerAxisMoveParam[AxisNo].startingVelocity,
			PowerAxisMoveParam[AxisNo].endVelocity,
			PowerAxisMoveParam[AxisNo].secondVelocity,
			PowerAxisMoveParam[AxisNo].movingAverageTimeMilliseconds);
		cFile->WriteString((LPCTSTR)strProfile);
	}
	cFile->WriteString(_T("[EOF_PROFILE]\n"));
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	TRACE(_T("[PWR] CMotorProfile(%s) Write Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cFile->Close();
	delete cFile;
	return Err;
}

long CMotorProfile::ModifyFile()
{
	long Err = NO_ERR;
	ULONGLONG GetTime = 0, Elapsed = 0;
	CString strProfile;
	CString line;
	long lineCount = 0;
	Initial();
	CStdioFile* cReadFile = new CStdioFile();
	CStdioFile* cWriteFile = new CStdioFile();
	if (cReadFile->Open((LPCTSTR)GetFileName(),
		CFile::modeRead
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] CMotorProfile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete cReadFile;
		return JOBFILE_OPEN_FAIL;
	}

	CString tempPath;
	tempPath.Format(_T("%s\\%s_.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
	if (cWriteFile->Open((LPCTSTR)tempPath,
		CFile::modeWrite
		| CFile::modeCreate
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] CMotorProfile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("WriteFile Open Fail"));
		delete cWriteFile;
		return JOBFILE_OPEN_FAIL;
	}

	GetTime = _time_get();

	while (cReadFile->ReadString(line))
	{
		if (line == "[EOF_PROFILE]")
			break;

		cWriteFile->WriteString(line + _T("\n"));
		lineCount++;
	}
	TRACE(_T("[PWR] MotorProfile Read Line Count : %d\n"), lineCount);
	if (lineCount == 0)
	{
		cWriteFile->WriteString(_T("[PROFILE]\n"));
		lineCount++;
	}

	for (long AxisNo = lineCount - 1; AxisNo < MAXAXISNO; ++AxisNo)
	{
		strProfile.Format(_T("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n"),
			PowerAxisMoveParam[AxisNo].velocity,
			PowerAxisMoveParam[AxisNo].acc,
			PowerAxisMoveParam[AxisNo].dec,
			PowerAxisMoveParam[AxisNo].jerkAcc,
			PowerAxisMoveParam[AxisNo].jerkDec,
			PowerAxisMoveParam[AxisNo].jerkAccRatio,
			PowerAxisMoveParam[AxisNo].jerkDecRatio,
			PowerAxisMoveParam[AxisNo].accTimeMilliseconds,
			PowerAxisMoveParam[AxisNo].decTimeMilliseconds,
			PowerAxisMoveParam[AxisNo].startingVelocity,
			PowerAxisMoveParam[AxisNo].endVelocity,
			PowerAxisMoveParam[AxisNo].secondVelocity,
			PowerAxisMoveParam[AxisNo].movingAverageTimeMilliseconds);
		cWriteFile->WriteString((LPCTSTR)strProfile);
	}
	cWriteFile->WriteString(_T("[EOF_PROFILE]\n"));
	if (Err != NO_ERR)
	{
		cReadFile->Close();
		cWriteFile->Close();
		delete cReadFile;
		delete cWriteFile;
		return Err;
	}
	TRACE(_T("[PWR] CMotorProfile(%s) Write Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cReadFile->Close();
	cWriteFile->Close();
	delete cReadFile;
	delete cWriteFile;

	CFile::Remove(GetFileName());            // 원본 삭제
	CFile::Rename(tempPath, GetFileName());
	return Err;
}

long CMotorProfile::ReadProfile(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [PROFILE]
	WMX3_AXIS_POSCOMMANDPROFILE Profile;

	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	for (long AxisNo = 0; AxisNo < MAXAXISNO; ++AxisNo)
	{
		bRet = cFile->ReadString(str);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CMotorProfile Profile TokenCount:%d\n", cTokenizer->GetCount());
		}
		long iCnt = 0, dCnt = 0;
		CString strValue;
		int iValue[20];
		double dValue[20];
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Profile, sizeof(Profile));
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
		TRACE(_T("[PWR] AxisNo:%d Vel:%.1f Acc:%.1f Dec:%.1f jerkAcc:%.1f jerkDec:%.1f jerkAccRatio:%.1f jerkDecRatio:%.1f\n"),
			AxisNo,	dValue[0], dValue[1], dValue[2], dValue[3], dValue[4], dValue[5], dValue[6]);
		Profile.velocity = dValue[0];
		Profile.acc = dValue[1];
		Profile.dec = dValue[2];
		Profile.jerkAcc = dValue[3];
		Profile.jerkDec = dValue[4];
		Profile.jerkAccRatio = dValue[5];
		Profile.jerkDecRatio = dValue[6];
		SetMoveProfile(GetAxisNameByAxisIndex(AxisNo), Profile);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	bRet = cFile->ReadString(str);			// [EOF_PROFILE]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}