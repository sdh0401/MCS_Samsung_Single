#include "pch.h"
#include "CLogFileSystem.h"
#include "GlobalData.h"
#include "Trace.h"
#include <direct.h>
#include <io.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

CLogFileSystem::CLogFileSystem() : m_strExtension(_T(""))
{
	m_nUnicodeOrderMark = 0xfeff;  // 유니코드 바이트 오더마크.  
	m_cFileLog = NULL;
	m_nSizeLimit = 1;
	m_nCurLoggingCount = 0;
	m_cFileLog = new CFile();
	m_bUseLog = false;
	m_strSpecifier = "";
	m_strCurFileName ="";
	m_bEndOfLine = true;
	m_nType = NORM_LOG;
}

CLogFileSystem::CLogFileSystem(CString strFolderName, CString strExtension, CString strSpecifier) : m_strExtension(_T(""))
{
	m_cFileLog = NULL;
	m_nSizeLimit = 1;
	m_nCurLoggingCount = 0;
	m_cFileLog = new CFile();
	m_bUseLog = false;
	m_strSpecifier = "";
	m_strCurFileName ="";
	InitializeLogging(strFolderName,strExtension,strSpecifier);
	m_bEndOfLine = true;
}

CLogFileSystem::~CLogFileSystem()
{
	delete m_cFileLog;
	m_cFileLog = NULL;
}

bool CLogFileSystem::CheckDirectoryExistence(CString strDir)
{
	bool bRet = false;
	if ( GetFileAttributes(strDir) != -1)
	{
		bRet = true;
	}
	return bRet;
}

bool CLogFileSystem::RefreshLogFileName(SYSTEMTIME *pST)
{
	//strLogPath;
	ULONGLONG lSize;
	CString strFileName;
	CString strFileHead = GetFileHead(pST,m_strSpecifier);
	if(m_nType == NORM_LOG)
	{
		if(m_NewFileName == "")
		{
			TRACE(_T("[PWR] Make log file name first\n"));
			return false;
		}
		strFileName.Format(_T("%s\\%s"), (LPCTSTR)m_strLoggingDir, (LPCTSTR)m_NewFileName);
		m_strCurFileName =  strFileName;
		if(m_cFileLog->Open(m_strCurFileName, CFile::modeWrite | CFile::modeCreate | CFile::modeNoTruncate | CFile::shareDenyNone, NULL) == false) return false;
#ifdef _UNICODE
		m_cFileLog->Write(&m_nUnicodeOrderMark, 2);
#endif
		m_cFileLog->SeekToEnd();
	}
	else
	{
		strFileName.Format(_T("%s\\%s_%d.%s"), (LPCTSTR)m_strLoggingDir, (LPCTSTR)strFileHead, m_nCurLoggingCount, (LPCTSTR)m_strExtension);
		if (m_strCurFileName == strFileName)
		{
			if (m_cFileLog->Open(m_strCurFileName, CFile::modeWrite | CFile::modeCreate | CFile::modeNoTruncate | CFile::shareDenyNone, NULL) == false) return false;
			m_cFileLog->SeekToEnd();
			lSize = m_cFileLog->GetLength();
			if (lSize >= (m_nSizeLimit * ONEMEGABYTE))
			{
				m_cFileLog->Close();	
				//m_cFileLog = NULL;
				m_nCurLoggingCount++;
				strFileName.Format(_T("%s\\%s_%d.%s"), (LPCTSTR)m_strLoggingDir, (LPCTSTR)strFileHead, m_nCurLoggingCount, (LPCTSTR)m_strExtension);
				m_strCurFileName = strFileName;
				if(m_cFileLog->Open(m_strCurFileName, CFile::modeWrite | CFile::modeCreate | CFile::modeNoTruncate | CFile::shareDenyNone, NULL) == false) return false;
#ifdef _UNICODE
				m_cFileLog->Write(&m_nUnicodeOrderMark, 2);
#endif
			}
		}
		else
		{
			m_nCurLoggingCount = 0;
			m_strCurFileName =  strFileName;
			if(m_cFileLog->Open(m_strCurFileName, CFile::modeWrite | CFile::modeCreate | CFile::modeNoTruncate | CFile::shareDenyNone, NULL) == false) return false;
#ifdef _UNICODE
			m_cFileLog->Write(&m_nUnicodeOrderMark, 2);
#endif
			m_cFileLog->SeekToEnd();
		}
	}
	return true;
}

CString CLogFileSystem::GetFileHead(SYSTEMTIME* pST, CString specifier)
{
	CString strFileHead;
	if (specifier == "")
		strFileHead.Format(_T("M_%d%02d%02d"), (long)pST->wYear, (long)pST->wMonth, (long)pST->wDay);
	else
		strFileHead.Format(_T("%s_%d%02d%02d"), (LPCTSTR)specifier, (long)pST->wYear, (long)pST->wMonth, (long)pST->wDay);
	strFileHead.Replace(_T(" "), _T("0"));
	return strFileHead;
}

int CLogFileSystem::GetFileLogLastIndex(CString strPath)
{
	SYSTEMTIME st;
	int nCurCount = 0;
	HANDLE hFind;
	WIN32_FIND_DATA dataFind;
	bool bMoreFile = true;
	CString fileSearch;
	CString fileName;

	GetLocalTime(&st);
	CString strFileHead = GetFileHead(&st,m_strSpecifier);

	fileSearch.Format(_T("%s\\%s_*.%s"), (LPCTSTR)m_strLoggingDir, (LPCTSTR)strFileHead, (LPCTSTR)m_strExtension);
	hFind = FindFirstFile(strPath + fileSearch, &dataFind);
	while(hFind != INVALID_HANDLE_VALUE && bMoreFile == true)
	{
		if(dataFind.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			if(!(_tcscmp(dataFind.cFileName, _T(".")) == 0 || _tcscmp(dataFind.cFileName, _T("..")) == 0))
			{
				// 디렉토리입니다. 이름은.. dataFind.cFileName 을 참고하세요
			}
		}
		else
		{
			// 파일입니다. 이름은.. dataFind.cFileName 을 참고하세요
			fileName.Format(_T("%s"),dataFind.cFileName);
			fileName.Delete(fileName.GetLength() - 4, 4);
			int f = fileName.ReverseFind('_');
			fileName.Delete(0,f+1);
			f = _wtoi(fileName);
			if(nCurCount <= f) nCurCount = f;
		}
		bMoreFile = FindNextFile(hFind, &dataFind);
	}
	//	nCurCount = nCurCount;
	return nCurCount;
}

void CLogFileSystem::AddLogMsg(CString strMsg, bool bAddTime)
{
	return;

	// 마지막에 다음 라인을 넣어 보기 쉽게 바꾼다.
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	GetLocalTime(&st);
	m_cc.Lock();
	if(RefreshLogFileName(&st))
	{
		if(bAddTime == TRUE)
		{
			strTime.Format(_T("<%02d:%02d:%02d.%03d>"), (long)st.wHour, (long)st.wMinute, (long)st.wSecond, (long)st.wMilliseconds);
			strTime.Replace(_T(" "),_T("0"));
			strWriteMsg = strTime + _T(" ") + strMsg + _T("\r\n");
		}
		else
		{
			strWriteMsg = strMsg + _T("\r\n");
		}
		m_cFileLog->Write(strWriteMsg, strWriteMsg.GetLength()*2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddLogMsgTimeCritical(LPSYSTEMTIME pSt, CString strMsg, bool bAddTime)
{
	return;

	// 마지막에 다음 라인을 넣어 보기 쉽게 바꾼다.
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	m_cc.Lock();
	if(RefreshLogFileName(pSt))
	{
		if(bAddTime == TRUE)
		{
			strTime.Format(_T("<%02d:%02d:%02d.%03d>"), (long)pSt->wHour, (long)pSt->wMinute, (long)pSt->wSecond, (long)pSt->wMilliseconds);
			strTime.Replace(_T(" "), _T("0"));
			strWriteMsg = strTime + _T(" ") + strMsg + _T("\r\n");
		}
		else
		{
			strWriteMsg = strMsg + _T("\r\n");
		}
		m_cFileLog->Write(strWriteMsg,strWriteMsg.GetLength()*2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddTimeLogMsg(CString strMsg,...)
{
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	GetLocalTime(&st);
	TCHAR szBuf[MAX_ONE_LOGLINE_SIZE] = {0};
	va_list list;  
	va_start(list,strMsg); 
	_vsntprintf_s(szBuf,MAX_ONE_LOGLINE_SIZE,strMsg,list);
	va_end(list);
	strMsg.Format(_T("%s"),szBuf);
	m_cc.Lock();
	if(RefreshLogFileName(&st))
	{
		strTime.Format(_T("<%02d:%02d:%02d.%03d>"), (long)st.wHour, (long)st.wMinute, (long)st.wSecond, (long)st.wMilliseconds);
		strTime.Replace(_T(" "),_T("0"));
		strWriteMsg = strTime + _T(" ") + strMsg + _T("\r\n");
		m_cFileLog->Write(strWriteMsg,strWriteMsg.GetLength()*2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddNormalLogMsg(CString strMsg,...)
{
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	GetLocalTime(&st);
	TCHAR szBuf[MAX_ONE_LOGLINE_SIZE] = {0};
	va_list list;  
	va_start(list,strMsg); 
	_vsntprintf_s(szBuf,MAX_ONE_LOGLINE_SIZE,strMsg,list);
	va_end(list);
	strMsg.Format(_T("%s"),szBuf);
	m_cc.Lock();
	if(RefreshLogFileName(&st))
	{
		strWriteMsg = strMsg + _T("\r\n");
		m_cFileLog->Write(strWriteMsg,strWriteMsg.GetLength()*2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddTraceMsg(CString strMsg,...)
{
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	GetLocalTime(&st);
	TCHAR szBuf[MAX_ONE_LOGLINE_SIZE] = {0};
	va_list list;  
	va_start(list,strMsg); 
	_vsntprintf_s(szBuf, MAX_ONE_LOGLINE_SIZE, strMsg, list);
	va_end(list);
	strMsg.Format(_T("%s"),szBuf);
	m_cc.Lock();
	if(m_nType == NORM_LOG) 
	{
		if(RefreshLogFileName(&st))
		{
			m_cFileLog->Write(strMsg, strMsg.GetLength() * 2);
			m_cFileLog->Close();
		}
	}
	else
	{
		if(RefreshLogFileName(&st))
		{
			m_cFileLog->Write(strMsg, strMsg.GetLength() * 2);
			m_cFileLog->Close();
		}
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddTimeTraceMsg(CString strMsg,...)
{
	if(m_nType == NORM_LOG) 
	{
		TRACE(_T("[PWR] Log file type is not match\n"));
		return;
	}
	if(m_bUseLog == false) return;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	GetLocalTime(&st);
	TCHAR szBuf[MAX_ONE_LOGLINE_SIZE] = {0};
	va_list list;  
	va_start(list,strMsg); 
	_vsntprintf_s(szBuf, MAX_ONE_LOGLINE_SIZE, strMsg, list);
	va_end(list);
	strMsg.Format(_T("%s"),szBuf);
	m_cc.Lock();
	if(RefreshLogFileName(&st))
	{
		strTime.Format(_T("<%02d:%02d:%02d.%03d>"), (long)st.wHour, (long)st.wMinute, (long)st.wSecond, (long)st.wMilliseconds);
		strTime.Replace(_T(" "),_T("0"));
		strWriteMsg = strTime + strMsg;
		m_cFileLog->Write(strWriteMsg, strWriteMsg.GetLength() * 2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::AddTerminalMsg(CString strMsg)
{
	if(m_nType == NORM_LOG) 
	{
		TRACE(_T("[PWR] Log file type is not match\n"));
		return;
	}
	int length;
	wchar_t* strBuffer = NULL;
	CString strWriteMsg,strTime;
	SYSTEMTIME st;
	m_cc.Lock();
	GetLocalTime(&st);
	if(RefreshLogFileName(&st))
	{
		length = strMsg.GetLength();
		if(length == 0) return;
		strBuffer = strMsg.GetBuffer(length);
		strTime.Format(_T("<%02d:%02d:%02d.%03d>"), (long)st.wHour, (long)st.wMinute, (long)st.wSecond, (long)st.wMilliseconds);
		strTime.Replace(_T(" "),_T("0"));
		if(m_bEndOfLine) strWriteMsg = strTime + " " + strMsg;
		else			 strWriteMsg = strMsg;
		if( strBuffer[length-1] == '\n')
		{
			strWriteMsg.Delete(strWriteMsg.GetLength()-1, 1);
			strWriteMsg.Replace(_T("\n"), _T("\n") + strTime + " ");
			strWriteMsg += "\n";
			m_bEndOfLine = TRUE;
		}
		else
		{
			strWriteMsg.Replace(_T("\n"), _T("\n") + strTime + " ");
			m_bEndOfLine = FALSE;
		}
		m_cFileLog->Write(strWriteMsg, strWriteMsg.GetLength() * 2);
		m_cFileLog->Close();
	}
	m_cc.Unlock();
}

void CLogFileSystem::Close()
{
	if(CheckLogFileValid())
	{
		m_cFileLog->Close(); 
		m_cFileLog = NULL;
	}
	m_bUseLog = false;
}

bool CLogFileSystem::CheckLogFileValid()
{
	if(m_cFileLog != NULL && m_cFileLog->m_hFile != INVALID_HANDLE_VALUE)
	{
		return true;
	}
	return false;
}

void CLogFileSystem::SetLogUse(bool bUse)
{
	m_bUseLog = bUse;
}

void CLogFileSystem::SetLogFileName(CString fileName)
{
	if(m_nType != NORM_LOG)
	{
		TRACE(_T("[PWR] Log file type is not match\n"));
		return;
	}
	m_NewFileName = fileName;
}

bool CLogFileSystem::InitializeLogging(CString strFolderName)
{
	CString strMsg;
	if(m_bUseLog == true) 
	{
		TRACE(_T("[PWR] Log file system is already initialized\n"));
		return false;
	}
	m_nType = NORM_LOG;
	if(ChangeLogFolder(strFolderName) == false) return false;

	m_strSpecifier = "";
	m_strExtension = "";
	m_strFolderName= strFolderName;
	m_strExtension = "";
	m_bUseLog = true;
	m_NewFileName ="";
	return true;
}

bool CLogFileSystem::InitializeLogging(CString strFolderName, CString strExtension, CString strSpecifier)
{
	CString strMsg;
	if(m_bUseLog == true) 
	{
		TRACE(_T("[PWR] Log file system is already initialized\n"));
		return false;
	}
	m_nType = SPEC_LOG;
	if(ChangeLogFolder(strFolderName) == false) return false;

	m_strSpecifier = strSpecifier;
	m_strExtension = strExtension;
	m_strFolderName= strFolderName;
	m_bUseLog = true;
	m_NewFileName ="";
	return true;
}

bool CLogFileSystem::ChangeLogFolder(CString strFolderName)
{
	CString strMsg;
	if(strFolderName[strFolderName.GetLength() -1] != _T('\\') && strFolderName[strFolderName.GetLength() -1] != _T('/'))
	{
		TRACE(_T("[PWR] Log folder(%s) is not correct folder add \\ or after end of folder"),strFolderName);
		return false;
	}
	if(!CheckDirectoryExistence(strFolderName))
	{
		//if(_CreateDirectory(strFolderName) == false)
		//{
		//	TRACE(_T("[PWR] Log folder not exist make it first = %s"),strFolderName);
		//	return false;
		//}
	}
	m_cc.Lock();
	m_strLoggingDir = strFolderName;
	if(m_nType == SPEC_LOG) m_nCurLoggingCount = GetFileLogLastIndex(m_strLoggingDir);
	else                     m_nCurLoggingCount = -1;
	m_cc.Unlock();
	return true;
}

bool CLogFileSystem::_CreateDirectory(LPCWSTR lpszPath )
{
	wchar_t szPathBuffer[MAX_PATH];
	size_t len = wcslen( lpszPath );
	for ( size_t i = 0 ; i < len ; i++ )
	{
		szPathBuffer[i] = *( lpszPath + i );
		if ( szPathBuffer[i] == _T('\\') || szPathBuffer[i] == _T('/') )
		{
			szPathBuffer[i + 1] = NULL;
			if ( ! PathFileExists( szPathBuffer ) )
			{
				if ( ! ::CreateDirectory( szPathBuffer, NULL ) )
				{
					if ( GetLastError() != ERROR_ALREADY_EXISTS )
						return false;
				}
			}
		}
	}
	return true;
}

