#pragma once

#include <windows.h>

#define INDEXGEN_MAXBUFFER 200
#define INDEXGEN_MAXINDEX INDEXGEN_MAXBUFFER*8
typedef enum {CPT_STEP_BY_STEP, CPT_FORCE_RELEASE } CPROTECT_TYPE ;
typedef enum {PATH_RET_ERROR=-1, PATH_RET_NOT_FOUND, PATH_RET_ISFILE, PATH_RET_ISFOLDER} PATH_RESULT;
enum FILETYPE {IS_ANSI, IS_UTF8, IS_UNICODE};

class CProtect
{
public:
	CProtect(void);
	~CProtect(void);
	CPROTECT_TYPE m_nType;
	DWORD m_Occupier;
	int m_nOccupiedCount;

	BOOL CanIUse(DWORD nID);
	void Release(DWORD nID);
	void PrintProtectInfo(void);
	BOOL IsOccupiedByMe(DWORD nOccupier, int* nOccupiedCount = NULL);
	void ReleaseAll(DWORD nID);
private:
	HANDLE m_mutexProtect;

};

class CIndexGenerator
{
public:
	CIndexGenerator(CString strName, int nMaxCount = 100);
	~CIndexGenerator(void);
	int GetOneIndex();
	BOOL ReleaseIndex(int nIndex);
	void ErrorExit(LPTSTR lpszFunction);
private:
	//BOOL m_bActiveServer;
	BOOL m_bCanUse;
	//HANDLE m_hCountKeep;
	HANDLE m_hMapFile;
	HANDLE m_hCountProtect;
	int m_nMaxIndex;
	CRITICAL_SECTION cs;
	//CArray<int,int> m_arList;
	int FindAndGetEmptyIndex(int nMax,BYTE* pByte);
	BOOL FindAndReleaseIndex(int nMax,BYTE* pByte, int nIndex);

};

class CUtil
{
public:
	static BOOL _CreateDirectory(LPCWSTR lpszPath );
	//static void FileExists(LPCWSTR strFile, BOOL& isDirExist, BOOL& isFileEixst);
	//static BOOL FileExists(LPCWSTR strFile);
	static PATH_RESULT CheckPath(CString sPath);
	static FILETYPE DetectEncoding(CString strFileName);
	static CString GetProgramPath();
};

class CAnswer
{
public:
	enum ANS_CHK_TYPE {ACT_ISNUMBER, ACT_ISSTRING};
protected:
	CAnswer();
public:
	CAnswer(int ID1, int ID2, int ID3);
	CAnswer(int ID1, int ID2, CString ID3);
	~CAnswer(void);
	int m_nType;
	int m_ID1;
	int m_ID2;
	int m_ID3;
	CString m_strID3;
	BOOL m_bAnswerGot;
	CString m_strAnswer;
};

class CAnswerChecker
{
public:
	CAnswerChecker();
	~CAnswerChecker(void);
private:
	HANDLE m_mutexProtect;
	CArray<CAnswer*, CAnswer*> m_AnswerList;
public:
	BOOL MakeAnswerChecker(int ID1, int ID2, int ID3, BOOL bIgnoreExistance = FALSE);
	BOOL MakeAnswerChecker(int ID1, int ID2, CString ID3, BOOL bIgnoreExistance = FALSE);
	BOOL GetAnswer(CString& strResult,int ID1, int ID2, int ID3, DWORD nMiliseconds);
	BOOL GetAnswer(CString& strResult,int ID1, int ID2, CString ID3, DWORD nMiliseconds);
	BOOL SetAnswer(CString strResult,int ID1, int ID2, int ID3);
	BOOL SetAnswer(CString strResult,int ID1, int ID2, CString ID3);
};

class CIDGenerator
{
public:
	enum ID_STATUS {NOT_ASSIGNED = -1, ASSIGNED};
public:
	CIDGenerator();
	CIDGenerator(int nStartID, int nMaxIDCount);
	~CIDGenerator();

private:
	int m_nStartID;
	int m_nMaxIDCount;
	HANDLE m_mutexProtect;
	int* m_nIDStatus;
	BOOL IsValidID(int nID);
public:
	int GetNewID();
	void ReleaseID(int nID);
	int ReadIDStatus(int nID);
	void ChangeStatus(int nID, int nStatus);
};

class GSyncObject
{

public:
    GSyncObject() { ::InitializeCriticalSection( &m_stCriticalSection ); }
	~GSyncObject() { ::DeleteCriticalSection( &m_stCriticalSection ); }

    // Lock critical section.
    bool Lock() { ::EnterCriticalSection( &m_stCriticalSection );return true; }

    // Unlock critical section.
    bool Unlock() { ::LeaveCriticalSection( &m_stCriticalSection ); return true; }

private:
    GSyncObject( const GSyncObject& );
    GSyncObject& operator = ( const GSyncObject& );

private:

    // Critical section object.
    CRITICAL_SECTION m_stCriticalSection;
};
/* usage
	declare "SyncObject m_LockWorkerThread;"
	in function call "AutoLock Lock(m_LockWorkerThread);"
*/
class AutoLock
{
public:
    AutoLock( GSyncObject& LockObj_i ) : m_pSyncObject( &LockObj_i )
    {
        if( NULL != m_pSyncObject )
        {
            m_pSyncObject->Lock();
        }
    }
    ~AutoLock()
    {
        if( NULL != m_pSyncObject )
        {
            m_pSyncObject->Unlock();
            m_pSyncObject = NULL;
        }
    }
private:
    GSyncObject* m_pSyncObject;
};
