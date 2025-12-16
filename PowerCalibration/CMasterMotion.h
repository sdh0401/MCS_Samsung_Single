#pragma once
#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include <ecApi.h>
#include "CPowerThread.h"

using namespace wmx3Api;
using namespace ecApi;

class CMasterMotionInfo {
public:
	CMasterMotionInfo();
	~CMasterMotionInfo();
	bool m_IsValid;
	char m_cMasterState;
	unsigned m_nSlaveCounts;
	int m_onLineSlaveCount;
	int m_offLineSlaveCount;
	int m_inAccessibleSlaveCount;
};

class CMasterMotionVersion {
public:
	CMasterMotionVersion();
	CMasterMotionVersion(char* lib, char* dll);
	~CMasterMotionVersion();
	char m_LibraryVersion[BUFSIZE];
	char m_ImDllVersion[BUFSIZE];
};

class CMasterMotion : public CPowerThread
{
public:
	CMasterMotion();
	~CMasterMotion();
	void Initialize();
	int GetWmx3Version();

	bool GetLibVersion(int* Err);
	bool GetIMDllVersion(int* Err);
	int GetDeviceCount(int* Err);
	
	int GetWmx3MasterInfo();
	int GetWmx3MasterState();
	int GetWmx3NumOfSlaves();
	int GetWmx3OnlineSlaveCount();
	int GetWmx3OfflineSlaveCount();
	int GetWmx3InaccessibleSlaveCount();
	
	bool IsWmx3Valid();

	void Run();
	static UINT StartMasterStatus(LPVOID wParam);

	CString m_StrLibVer;
	CString m_StrDllVer;

	char m_LibraryVersion[BUFSIZE];
	char m_ImDllVersion[BUFSIZE];
	char m_ErrString[BUFSIZE];

	EcMasterInfo* m_pMasterInfo;
	CMasterMotionInfo m_MasterMotionInfo;
private:
	long m_ShowID;
};

extern CMasterMotion* gcMasterMotion;