#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

enum { SPEC_LOG, NORM_LOG };

class CLogFileSystem
{
public:
	void Close();
	void AddTimeLogMsg(CString strMsg,...);
	void AddNormalLogMsg(CString strMsg,...);
	void AddTraceMsg(CString strMsg,...);
	void AddTimeTraceMsg(CString strMsg,...);
	void AddLogMsg(CString strMsg, bool bAddTime = true);
	void AddLogMsgTimeCritical(LPSYSTEMTIME pSt, CString strMsg, bool bAddTime=true);
	void AddTerminalMsg(CString strMsg);
	bool InitializeLogging(CString strFolderName, CString strExtension, CString strSpecifier = _T(""));
	bool InitializeLogging(CString strFolderName);
	bool ChangeLogFolder(CString strFolderName);
	void SetLogFileName(CString fileName);
	bool _CreateDirectory(LPCWSTR lpszPath );
	CLogFileSystem();
	CLogFileSystem(CString strFolderName, CString strExtension, CString strSpecifier);
	virtual ~CLogFileSystem();
	CString m_strFolderName;
	void SetLogUse(bool bUse);

private:
	int m_nType;
	USHORT m_nUnicodeOrderMark;  // 유니코드 바이트 오더마크.  
	bool CheckLogFileValid();
	bool RefreshLogFileName(SYSTEMTIME* pST);
	bool CheckDirectoryExistence(CString strDir);
	CString GetFileHead(SYSTEMTIME* pST,CString specifier);
	int GetFileLogLastIndex(CString strPath);
	CFile* m_cFileLog;
	//FILE * m_cFileLogB;
	int m_nCurLoggingCount;
	int m_nSizeLimit;
	int m_nLogKeep;
	bool m_bUseLog;
	bool m_bEndOfLine;
	CString m_strCurFileName;
	CString m_NewFileName;
	CString m_strSpecifier;
	CString m_strLoggingDir;
	CString m_strLocalDir;
	CCriticalSection m_cc;
	CString m_strExtension;
};

