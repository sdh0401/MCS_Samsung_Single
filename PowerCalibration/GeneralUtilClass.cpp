#include "pch.h"
#include "GeneralUtilClass.h"
#include "GlobalDefine.h"
#include "Trace.h"
#include "GlobalData.h"

//#include <strsafe.h>

typedef struct _TIndexInfo
{
	int nCount;
	BYTE byteList[INDEXGEN_MAXBUFFER];
} TIndexInfo;

CProtect::CProtect(void)
{
	m_Occupier = -1;
	m_nOccupiedCount = 0;
	m_mutexProtect = CreateMutex( NULL, FALSE, NULL);
	m_nType = CPT_STEP_BY_STEP;
}

CProtect::~CProtect(void)
{
	CloseHandle(m_mutexProtect);
}

BOOL CProtect::CanIUse(DWORD nID)
{
	BOOL bResult = TRUE;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	if(m_Occupier != nID)
	{
		if(m_Occupier != -1)		bResult = FALSE;
		else						m_nOccupiedCount++;
	}
	else m_nOccupiedCount++;
	if(bResult == TRUE)	m_Occupier = nID;
	ReleaseMutex(m_mutexProtect);
	return bResult;
}

void CProtect::Release(DWORD nID)
{
	WaitForSingleObject(m_mutexProtect, INFINITE);
	if(m_Occupier == nID)
	{
		if(m_nType == CPT_STEP_BY_STEP)
		{
			m_nOccupiedCount--;
			if(m_nOccupiedCount == 0) m_Occupier = NO_RESULT;
			else if(m_nOccupiedCount < 0) ASSERT("occupier error minus!!!");
		}
		else
		{
			m_Occupier = NO_RESULT;
			m_nOccupiedCount = 0;
		}
	}
	else ASSERT("unknown id try to release!!!");
	ReleaseMutex(m_mutexProtect);
}

void CProtect::PrintProtectInfo(void)
{
	TRACE(_T("[PWR] Occupier:%d OccupierCount:%d\n"),m_Occupier,m_nOccupiedCount);
}

BOOL CProtect::IsOccupiedByMe(DWORD nOccupier, int* nOccupiedCount)
{
	BOOL bResult = FALSE;
	ASSERT(nOccupier != NO_RESULT);
	WaitForSingleObject(m_mutexProtect, INFINITE);
	if(m_Occupier == nOccupier)
	{
		if(nOccupiedCount != NULL) *nOccupiedCount = m_nOccupiedCount;
		bResult = TRUE;
	}
	else
	{
		if(nOccupiedCount != NULL) *nOccupiedCount = 0;
	}
	ReleaseMutex(m_mutexProtect);
	return bResult;
}

void CProtect::ReleaseAll(DWORD nID)
{
	WaitForSingleObject(m_mutexProtect, INFINITE);
	if(m_Occupier == nID)
	{
		if(m_nType == CPT_FORCE_RELEASE) ASSERT("Dont call this function when type is force release");
		m_Occupier = NO_RESULT;
		m_nOccupiedCount = 0;
	}
	else ASSERT("unknown id try to release!!!");
	ReleaseMutex(m_mutexProtect);
}

CIndexGenerator::CIndexGenerator(CString strName, int nMaxIndex)
{
	BOOL bAlreadyExist = FALSE;
	ASSERT(nMaxIndex >= 1);
	m_bCanUse = FALSE;
	CString strTemp;
	TIndexInfo* myPoint;
	ASSERT(nMaxIndex <= INDEXGEN_MAXINDEX);
	if(strName.IsEmpty() == FALSE)
	{
		strTemp.Format(_T("%s_MTX"), (LPCTSTR)strName);
		m_hCountProtect = CreateMutex(NULL,0,strTemp);
		if(m_hCountProtect != NULL)
		{
			strTemp.Format(_T("Local\\%s"), (LPCTSTR)strName);
			m_hMapFile = CreateFileMapping(
				INVALID_HANDLE_VALUE,	// Use paging file instead of existing file.
				// Pass file handle to share in a file.
				NULL,					// Default security 
				PAGE_READWRITE,			// Read/write access
				0,						// Max. object size 
				sizeof(TIndexInfo),			// Buffer size  
				strTemp			// Name of mapping object
				);
			if(m_hMapFile != NULL)
			{
				if(::GetLastError() == ERROR_ALREADY_EXISTS)
				{
					m_hMapFile = OpenFileMapping( 
						FILE_MAP_ALL_ACCESS,	// Read/write access
						FALSE,					// Do not inherit the name
						strTemp			// Name of mapping object 
						);
					if(m_hMapFile != NULL) m_bCanUse = TRUE;
				}
				else
				{
					myPoint = (TIndexInfo*)::MapViewOfFile(m_hMapFile,FILE_MAP_ALL_ACCESS, 
						0,0,sizeof(TIndexInfo));
					if (myPoint != NULL)
					{
						myPoint->nCount = nMaxIndex;
						for (int i = 0; i < INDEXGEN_MAXBUFFER; i++) myPoint->byteList[i] = 0x00;
						m_bCanUse = TRUE;
					}
				}
			}
			if(m_bCanUse == FALSE) CloseHandle(m_hCountProtect);
		}
	}
	if(m_bCanUse == FALSE) TRACE(_T("[PWR] Failed to make IndexGenerator\nCheck name or other things\n"));
}

CIndexGenerator::~CIndexGenerator(void)
{
	if(m_bCanUse)
	{
		CloseHandle(m_hCountProtect);
		CloseHandle(m_hMapFile);
	}
}

int CIndexGenerator::GetOneIndex()
{
	int nReturn = -1;
	ASSERT(m_bCanUse);
	TIndexInfo* myPoint;
	if(WaitForSingleObject(m_hCountProtect, WAIT_FOREVER) == WAIT_OBJECT_0)
	{
		myPoint = (TIndexInfo*)::MapViewOfFile(m_hMapFile,FILE_MAP_ALL_ACCESS, 0,0,sizeof(TIndexInfo));
		if(myPoint != NULL)
		{
			nReturn = FindAndGetEmptyIndex(myPoint->nCount,myPoint->byteList);
		}
		if (myPoint != NULL)
		{
			UnmapViewOfFile(myPoint);
		}
		ReleaseMutex(m_hCountProtect);
	}
	return nReturn;
}

BOOL CIndexGenerator::ReleaseIndex(int nIndex)
{
	BOOL bResult = FALSE;
	ASSERT(m_bCanUse);
	TIndexInfo* myPoint;
	if(WaitForSingleObject(m_hCountProtect, WAIT_FOREVER) == WAIT_OBJECT_0)
	{
		myPoint = (TIndexInfo*)::MapViewOfFile(m_hMapFile,FILE_MAP_ALL_ACCESS, 0,0,sizeof(TIndexInfo));
		if(myPoint != NULL)
		{
			bResult = FindAndReleaseIndex(myPoint->nCount,myPoint->byteList, nIndex);
		}
		if (myPoint != NULL)
		{
			UnmapViewOfFile(myPoint);
		}
		ReleaseMutex(m_hCountProtect);
	}
	return bResult;
}

void CIndexGenerator::ErrorExit(LPTSTR lpszFunction) 
{ 
	DWORD dw = GetLastError(); 
	ExitProcess(dw); 
}

int CIndexGenerator::FindAndGetEmptyIndex(int nMax,BYTE* pByte)
{
	int nIndex = -1;
	int nTempIndex;
	BYTE t,byteT;
	for(int i=0; i<INDEXGEN_MAXBUFFER; i++)
	{
		t=0x01;
		for(int nFlag = 0; nFlag < 8; nFlag++)
		{
			nTempIndex = (i*8) + nFlag + 1;
			if(nTempIndex > nMax) break;
			byteT = pByte[i];
			byteT = byteT & t;
			if(byteT == 0x00)
			{
				nIndex = nTempIndex;
				pByte[i] |= t;
				break;
			}
			t = t << 1;

		}
		if(nIndex != -1) break;
	}
	return nIndex;
}

BOOL CIndexGenerator::FindAndReleaseIndex(int nMax,BYTE* pByte, int nIndex)
{
	ASSERT(nIndex < nMax && nIndex > 0);
	BYTE t,byteT;
	int nTempIndex = nIndex -1;
	int nBytePos = nTempIndex / 8;
	int nBitPos = nTempIndex % 8;
	t = 0x01;
	t <<= nBitPos;
	byteT = pByte[nBytePos] & t;
	if(byteT != t)
	{
		TRACE(_T("[PWR] no index for release exist\n"));
		return FALSE;
	}
	t = ~t;
	pByte[nBytePos] = pByte[nBytePos] & t; 
	return TRUE;
}

BOOL CUtil::_CreateDirectory(LPCWSTR lpszPath)
{
	wchar_t szPathBuffer[MAX_PATH];
	size_t len = wcslen(lpszPath);
	for (size_t i = 0; i < len; i++)
	{
		szPathBuffer[i] = *(lpszPath + i);
		if (szPathBuffer[i] == _T('\\') || szPathBuffer[i] == _T('/'))
		{
			szPathBuffer[i + 1] = NULL;
			if (!PathFileExists(szPathBuffer))
			{
				if (!::CreateDirectory(szPathBuffer, NULL))
				{
					if (GetLastError() != ERROR_ALREADY_EXISTS)
						return false;
				}
			}
		}
	}
	return true;
}

CString CUtil::GetProgramPath()
{
	CString strResult;
	CString strPath;

	if( GetModuleFileName( nullptr, strPath.GetBuffer(_MAX_PATH + 1), MAX_PATH ) != FALSE )
	{
		strPath.ReleaseBuffer( );

		strResult = strPath.Left( strPath.ReverseFind( '\\' )+1 );
	}
	else
	{
	}

	return strResult;
}

//void CUtil::FileExists(LPCWSTR strFile, BOOL& isDirExist, BOOL& isFileEixst)
//{
//	isDirExist = TRUE;
//	isFileEixst = FALSE;
//	if(PathFileExists(strFile) == FALSE)
//	{
//		if(::GetLastError() == ERROR_PATH_NOT_FOUND) isDirExist = FALSE;
//		isFileEixst = FALSE;
//	}
//}
//
//BOOL CUtil::FileExists(LPCWSTR strFile)
//{
//	return PathFileExists(strFile);
//}

PATH_RESULT CUtil::CheckPath(CString sPath)
{
	DWORD dwAttr = GetFileAttributes(sPath);
	if (dwAttr == 0xffffffff) 
	{
		if (GetLastError() == ERROR_FILE_NOT_FOUND || GetLastError() == ERROR_PATH_NOT_FOUND) 
			return PATH_RET_NOT_FOUND;
		return PATH_RET_ERROR;
	}
	if (dwAttr & FILE_ATTRIBUTE_DIRECTORY) return PATH_RET_ISFOLDER;
	return PATH_RET_ISFILE;
}

FILETYPE CUtil::DetectEncoding(CString strFileName)
{
	//	FF FE: unicode
	//	FE FF: unicode big endian
	//	EF BB: UTF-8 coding

	FILETYPE myType = IS_ANSI;
	char byteRead[10];
	CFile file;
	file.Open(strFileName,CFile::modeRead | CFile::typeBinary);
	file.Read(byteRead,10);
	//int kk = String_GetEncoding(byteRead);
	if(byteRead[0] == 0xff && byteRead[1] == 0xfe) myType = IS_UNICODE;
	else if(byteRead[0] == 0xfe && byteRead[1] == 0xff)  myType = IS_UNICODE;
	else if(byteRead[0] == 0xef && byteRead[1] == 0xbb)  myType = IS_UTF8;
	file.Close();
	return myType;
}

CAnswer::CAnswer(int ID1, int ID2, int ID3)
{
	m_nType = ACT_ISNUMBER;
	m_ID1 = ID1;
	m_ID2 = ID2;
	m_ID3 = ID3;
	m_strAnswer = "";
	m_bAnswerGot =FALSE;
}

CAnswer::CAnswer(int ID1, int ID2, CString ID3)
{
	m_nType = ACT_ISSTRING;
	m_ID1 = ID1;
	m_ID2 = ID2;
	m_strID3 = ID3;
	m_strAnswer = "";
	m_bAnswerGot =FALSE;
}

CAnswer::~CAnswer()
{

}

CAnswerChecker::CAnswerChecker()
{
	m_mutexProtect = CreateMutex( NULL, FALSE, NULL);
}

CAnswerChecker::~CAnswerChecker()
{

}

BOOL CAnswerChecker::MakeAnswerChecker(int ID1, int ID2, int ID3, BOOL bIgnoreExistance)
{
	BOOL bResult = TRUE;
	BOOL bExist = FALSE;
	CAnswer* pAnswer = NULL;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	for(int i=0; i<m_AnswerList.GetCount(); i++)
	{
		pAnswer = m_AnswerList.GetAt(i);
		if(pAnswer->m_nType == CAnswer::ACT_ISNUMBER)
		{
			if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_ID3 == ID3)
			{
				bExist = TRUE;
				break;
			}
		}
	}
	if(bExist == FALSE)
	{
		pAnswer = new CAnswer(ID1,ID2,ID3);
		m_AnswerList.Add(pAnswer);
	}
	else
	{
		if(bIgnoreExistance == FALSE) bResult = FALSE;
		else
		{
			pAnswer->m_bAnswerGot = FALSE;
			pAnswer->m_strAnswer ="";
		}
	}
	ReleaseMutex(m_mutexProtect);
	return bResult;
}

BOOL CAnswerChecker::MakeAnswerChecker(int ID1, int ID2, CString ID3, BOOL bIgnoreExistance)
{
	BOOL bResult = TRUE;
	BOOL bExist = FALSE;
	CAnswer* pAnswer = NULL;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	for(int i=0; i<m_AnswerList.GetCount(); i++)
	{
		pAnswer = m_AnswerList.GetAt(i);
		if(pAnswer->m_nType == CAnswer::ACT_ISSTRING)
		{
			if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_strID3 == ID3)
			{
				bExist = TRUE;
				break;
			}
		}
	}
	if(bExist == FALSE)
	{
		pAnswer = new CAnswer(ID1,ID2,ID3);
		m_AnswerList.Add(pAnswer);
	}
	else
	{
		if(bIgnoreExistance == FALSE) bResult = FALSE;
		else
		{
			pAnswer->m_bAnswerGot = FALSE;
			pAnswer->m_strAnswer ="";
		}
	}
	ReleaseMutex(m_mutexProtect);
	return bResult;
}
	
BOOL CAnswerChecker::GetAnswer(CString& strResult,int ID1, int ID2, int ID3, DWORD nMiliseconds)
{
	BOOL bResult = TRUE;
	CAnswer* pAnswer = NULL;
	ULONGLONG nTime = _time_get();
	while(TRUE)
	{
		bResult = FALSE;
		WaitForSingleObject(m_mutexProtect, INFINITE);
		for(int i=0; i<m_AnswerList.GetCount(); i++)
		{
			pAnswer = m_AnswerList.GetAt(i);
			if(pAnswer->m_nType == CAnswer::ACT_ISNUMBER)
			{
				if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_ID3 == ID3)
				{
					if(pAnswer->m_bAnswerGot)
					{
						strResult = pAnswer->m_strAnswer;
						bResult = TRUE;
						m_AnswerList.RemoveAt(i);
						delete pAnswer;
						pAnswer = NULL;
					}
					break;
				}
			}
		}
		ReleaseMutex(m_mutexProtect);
		if(bResult == TRUE) break;
		if(bResult == FALSE && _time_elapsed(nTime) > nMiliseconds) break;
		Sleep(1);
	}
	return bResult;
}

BOOL CAnswerChecker::GetAnswer(CString& strResult,int ID1, int ID2, CString ID3, DWORD nMiliseconds)
{
	BOOL bResult = TRUE;
	CAnswer* pAnswer = NULL;
	ULONGLONG nTime = _time_get();
	while(TRUE)
	{
		bResult = FALSE;
		WaitForSingleObject(m_mutexProtect, INFINITE);
		for(int i=0; i<m_AnswerList.GetCount(); i++)
		{
			pAnswer = m_AnswerList.GetAt(i);
			if(pAnswer->m_nType == CAnswer::ACT_ISSTRING)
			{
				if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_strID3 == ID3)
				{
					if(pAnswer->m_bAnswerGot)
					{
						strResult = pAnswer->m_strAnswer;
						bResult = TRUE;
						m_AnswerList.RemoveAt(i);
						delete pAnswer;
						pAnswer = NULL;
					}
					break;
				}
			}
		}
		ReleaseMutex(m_mutexProtect);
		if(bResult == TRUE) break;
		if(bResult == FALSE && _time_elapsed(nTime) > nMiliseconds) break;
		Sleep(1);
	}
	return bResult;
}

BOOL CAnswerChecker::SetAnswer(CString strResult,int ID1, int ID2, int ID3)
{
	BOOL bResult = FALSE;
	CAnswer* pAnswer = NULL;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	for(int i=0; i<m_AnswerList.GetCount(); i++)
	{
		pAnswer = m_AnswerList.GetAt(i);
		if(pAnswer->m_nType == CAnswer::ACT_ISNUMBER)
		{
			if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_ID3 == ID3)
			{
				pAnswer->m_strAnswer = strResult;
				pAnswer->m_bAnswerGot = TRUE;
				bResult = TRUE;
				break;
			}
		}
	}
	ReleaseMutex(m_mutexProtect);
	return bResult;
}

BOOL CAnswerChecker::SetAnswer(CString strResult,int ID1, int ID2, CString ID3)
{
	BOOL bResult = FALSE;
	CAnswer* pAnswer = NULL;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	for(int i=0; i<m_AnswerList.GetCount(); i++)
	{
		pAnswer = m_AnswerList.GetAt(i);
		if(pAnswer->m_nType == CAnswer::ACT_ISSTRING)
		{
			if(pAnswer->m_ID1 == ID1 && pAnswer->m_ID2 == ID2 && pAnswer->m_strID3 == ID3)
			{
				pAnswer->m_strAnswer = strResult;
				pAnswer->m_bAnswerGot = TRUE;
				bResult = TRUE;
				break;
			}
		}
	}
	ReleaseMutex(m_mutexProtect);
	return bResult;
}


CIDGenerator::CIDGenerator()
{
	m_nStartID = 1;
	m_mutexProtect = CreateMutex( NULL, FALSE, NULL);
	m_nMaxIDCount = 100;
	m_nIDStatus = new int[m_nMaxIDCount];
	for(int i=0; i<m_nMaxIDCount; i++) m_nIDStatus[i] = CIDGenerator::NOT_ASSIGNED;
}
CIDGenerator::CIDGenerator(int nStartID, int nMaxIDCount)
{
	if(nMaxIDCount <= 0)
	{
		TRACE(_T("[PWR] Max Id Should over 0\n"));
		return;
	}
	if(nStartID <= 0)
	{
		TRACE(_T("[PWR] Start ID should over 0\n"));
		return;
	}
	m_nStartID = nStartID;
	m_mutexProtect = CreateMutex( NULL, FALSE, NULL);
	m_nMaxIDCount = nMaxIDCount;
	m_nIDStatus = new int[m_nMaxIDCount];
	for(int i=0; i<m_nMaxIDCount; i++) m_nIDStatus[i] = CIDGenerator::NOT_ASSIGNED;
}
CIDGenerator::~CIDGenerator()
{
	CloseHandle(m_mutexProtect);
	delete[] m_nIDStatus;
}

int CIDGenerator::GetNewID()
{
	int nID = -1;
	WaitForSingleObject(m_mutexProtect, INFINITE);
	for(int i=0; i<m_nMaxIDCount; i++)
	{
		if(m_nIDStatus[i] == CIDGenerator::NOT_ASSIGNED)
		{
			m_nIDStatus[i] = CIDGenerator::ASSIGNED;
			nID = i+m_nStartID;
			break;
		}
	}
	ReleaseMutex(m_mutexProtect);
	return nID;
}
void CIDGenerator::ReleaseID(int nID)
{
	int nIndex;
	ASSERT(nID >= m_nStartID && nID <= (m_nStartID + m_nMaxIDCount));
	WaitForSingleObject(m_mutexProtect, INFINITE);
	nIndex = nID - m_nStartID;
	ASSERT(m_nIDStatus[nIndex] != CIDGenerator::NOT_ASSIGNED);
	m_nIDStatus[nIndex] = CIDGenerator::NOT_ASSIGNED;
	ReleaseMutex(m_mutexProtect);

}
int CIDGenerator::ReadIDStatus(int nID)
{
	int nIndex;
	int nStatus;
	ASSERT(nID >= m_nStartID && nID <= (m_nStartID + m_nMaxIDCount));
	WaitForSingleObject(m_mutexProtect, INFINITE);
	nIndex = nID - m_nStartID;
	nStatus = m_nIDStatus[nIndex];
	ReleaseMutex(m_mutexProtect);
	return nStatus;
}
void CIDGenerator::ChangeStatus(int nID, int nStatus)
{
	int nIndex;
	ASSERT(nStatus > 0);
	ASSERT(nID >= m_nStartID && nID <= (m_nStartID + m_nMaxIDCount));
	WaitForSingleObject(m_mutexProtect, INFINITE);
	nIndex = nID - m_nStartID;
	ASSERT(m_nIDStatus[nIndex] != CIDGenerator::NOT_ASSIGNED);
	m_nIDStatus[nIndex] = nStatus;
	ReleaseMutex(m_mutexProtect);
}
BOOL CIDGenerator::IsValidID(int nID)
{
	if(nID < m_nStartID) return FALSE;
	if(nID > m_nStartID + m_nMaxIDCount) return FALSE;
	return TRUE;
}

