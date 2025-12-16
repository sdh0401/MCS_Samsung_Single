#include "pch.h"
#include "CBarcodeFile.h"
#include "AxisInformation.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CStep.h"

CBarcodeFile* gcBarcodeFile;
CBarcodeFile::CBarcodeFile()
{
	Initial();
	SetDir(_T("C:\\Power\\i6.0\\MCS\\"));
	SetHeader(_T("Barcode"));
	SetExtension(_T("Sys"));
	MakeFileName();
	if (MakeFile() == true)
	{
		TRACE(_T("[PWR] MakeFile Barcode Complete\n"));
	}
	else
	{
		ReadFile();
	}
}

CBarcodeFile::~CBarcodeFile()
{
}

void CBarcodeFile::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CBarcodeFile::GetFileName()
{
	return m_StrFileName;
}

void CBarcodeFile::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CBarcodeFile::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CBarcodeFile::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CBarcodeFile::Initial()
{
}

bool CBarcodeFile::MakeFile()
{
	if (checkFileOpen(GetFileName()) == false)
	{
		FILE* fp;
		char filename[BUFSIZE];
		ZeroMemory(filename, sizeof(filename));
		CStringA strConverter(GetFileName());
		memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
		filename[strConverter.GetLength()] = 0;
		fopen_s(&fp, filename, "a");
		if (fp == NULL)
		{
			TRACE(_T("[PWR] CBarcodeFile open is null\n"));
			return false;
		}
		fclose(fp);
		return true;
	}
	else
	{
		return false;
	}
}

long CBarcodeFile::ReadFile()
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
		TRACE(_T("[PWR] CBarcodeFile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] CBarcodeFile(%s) Open Success\n"), (LPCTSTR)GetFileName());
	GetTime = _time_get();
	Err = ReadBarcode(cFile);
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	TRACE(_T("[PWR] CBarcodeFile(%s) Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cFile->Close();
	delete cFile;
	return Err;
}

long CBarcodeFile::WriteFile()
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
		TRACE(_T("[PWR] CBarcodeFile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] CBarcodeFile(%s) Open Success\n"), (LPCTSTR)GetFileName());
	GetTime = _time_get();
	cFile->WriteString(_T("[BARCODE]\n"));
	strProfile.Format(_T("%s\n"), (LPCTSTR)GetBarcode(ENTRY_CONV));
	cFile->WriteString((LPCTSTR)strProfile);
	strProfile.Format(_T("%s\n"), (LPCTSTR)GetBarcode(WORK1_CONV));
	cFile->WriteString((LPCTSTR)strProfile);
	strProfile.Format(_T("%s\n"), (LPCTSTR)GetBarcode(EXIT_CONV));
	cFile->WriteString((LPCTSTR)strProfile);
	cFile->WriteString(_T("[EOF_BARCODE]\n"));
	if (Err != NO_ERR)
	{
		cFile->Close();
		delete cFile;
		return Err;
	}
	TRACE(_T("[PWR] CBarcodeFile(%s) Write Complete Elapsed:%d[ms] Err:%d\n"), (LPCTSTR)GetFileName(), _time_elapsed(GetTime), Err);
	cFile->Close();
	delete cFile;
	return Err;
}

long CBarcodeFile::ReadBarcode(CStdioFile* cFile)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	bRet = cFile->ReadString(str);			// [BARCODE]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	SetBarcode(ENTRY_CONV, str);
	bRet = cFile->ReadString(str);
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	SetBarcode(WORK1_CONV, str);
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	SetBarcode(EXIT_CONV, str);
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	bRet = cFile->ReadString(str);			// [EOF_BARCODE]
	if (bRet == false)
	{
		return JOBFILE_READSTRING_FAIL;
	}
	return Err;
}

void CBarcodeFile::SetBarcode(long Conv, CString Barcode)
{
	if (Conv == ENTRY_CONV)
	{
		m_EntryBarcode = Barcode;
	}
	else if (Conv == WORK1_CONV)
	{
		m_WorkBarcode = Barcode;
	}
	else if (Conv == EXIT_CONV)
	{
		m_ExitBarcode = Barcode;
	}
	TRACE(_T("[PWR] SetBarcode Conv:%d %s\n"), Conv, Barcode);
}

CString CBarcodeFile::GetBarcode(long Conv)
{
	CString strBarcode;
	if (Conv == ENTRY_CONV)
	{
		strBarcode = m_EntryBarcode;
	}
	else if (Conv == WORK1_CONV)
	{
		strBarcode = m_WorkBarcode;
	}
	else if (Conv == EXIT_CONV)
	{
		strBarcode = m_ExitBarcode;
	}
	TRACE(_T("[PWR] GetBarcode Conv:%d %s\n"), Conv, strBarcode);
	return strBarcode;
}

