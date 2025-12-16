#include "pch.h"
#include "PowerCalibration.h"
#include "CMasterMotion.h"
#include "GlobalDefine.h"
#include "DefineThreadLoopTime.h"
#include "NetworkDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "CPowerStackWalker.h"
//#include "ErrorCode.h"
#include "CPowerLog.h"

#include <WMX3Api.h>
#include <EcApi.h>

using namespace wmx3Api;
using namespace ecApi;
using namespace std;

CMasterMotion* gcMasterMotion;
CMasterMotion::CMasterMotion()
{
    Initialize();
}

void CMasterMotion::Initialize()
{
    m_StrLibVer = _T("99.99.99.99");
    m_StrDllVer = _T("99.99");
    m_pMasterInfo = new EcMasterInfo();
    GetWmx3Version();
}

CMasterMotion::~CMasterMotion()
{
    delete m_pMasterInfo;
}

void CMasterMotion::Run()
{
    CString strLog;
    HANDLE nHandle;
    DWORD nID = NULL;
    _beginthreadex_proc_type lpStartAddress;
    SetRunning(true);
	SetEnd(false);
    SetThreadName(_T("thMasterStatus"));
    lpStartAddress = (_beginthreadex_proc_type)StartMasterStatus;
    nHandle = (HANDLE)_beginthreadex(nullptr, 0, lpStartAddress, this, 0, (unsigned*)&nID);
    if (nHandle != INVALID_HANDLE_VALUE)
        SetThreadHandle(nHandle);
    if (nID != NULL)
        SetThreadID(nID);
    m_ShowID = nID;
    TRACE(_T("[PWR] Master Motion Thread ID:0x%04X(%s)\n"), m_ShowID, GetThreadName());
    strLog.Format(_T("[PWR] Master Motion Thread ID:0x%04X(%s)"), m_ShowID, (LPCTSTR)GetThreadName());
    gcPowerLog->Logging(strLog);
}

bool CMasterMotion::GetIMDllVersion(int* Err)
{
    int iVersion, iRevision;
    int err = ErrorCode::None;
    //if (GetGlobalSimulationMode() == true)
    //{
    //    CStringA strVer(m_StrDllVer);
    //    memcpy(m_ImDllVersion, strVer.GetBuffer(), strVer.GetLength());
    //    m_ImDllVersion[strVer.GetLength()] = 0;
    //    return true;
    //}
    err = GetDevice()->GetIMDllVersion(&iVersion, &iRevision);
    if (err == ErrorCode::None)
    {
        m_StrDllVer.Format(_T("%02d.%02d"), iVersion, iRevision);
        CStringA strVer(m_StrDllVer);
        memcpy(m_ImDllVersion, strVer.GetBuffer(), strVer.GetLength());
        m_ImDllVersion[strVer.GetLength()] = 0;
    }
    *Err = err;
    return true;
}

bool CMasterMotion::GetLibVersion(int *Err)
{
    int iMajor, iMinor, iRevision, iFix;
    int err = ErrorCode::None;
    //if (GetGlobalSimulationMode() == true)
    //{
    //    CStringA strVer(m_StrLibVer);
    //    memcpy(m_LibraryVersion, strVer.GetBuffer(), strVer.GetLength());
    //    m_LibraryVersion[strVer.GetLength()] = 0;
    //    return true;
    //}
    err = GetDevice()->GetLibVersion(&iMajor, &iMinor, &iRevision, &iFix);
    if (err == ErrorCode::None)
    {
        m_StrLibVer.Format(_T("%02d.%02d.%02d.%02d"), iMajor, iMinor, iRevision, iFix);
        CStringA strVer(m_StrLibVer);
        memcpy(m_LibraryVersion, strVer.GetBuffer(), strVer.GetLength());
        m_LibraryVersion[strVer.GetLength()] = 0;
    }
    *Err = err;
    return true;
}

int CMasterMotion::GetDeviceCount(int* Err)
{
    int err = ErrorCode::None;
    int ret = 0;
    //if (GetGlobalSimulationMode() == true)
    //{
    //    *Err = err;
    //    return 0;
    //}
    err = GetDevice()->GetAllDevices(g_DevInfo);
    if (err == ErrorCode::None)
    {
        ret = g_DevInfo->count;
    }
    *Err = err;
    return ret;
}

int CMasterMotion::GetWmx3MasterInfo()
{
    int err = ErrorCode::None;
    //if (GetGlobalSimulationMode() == true) return ErrorCode::None;
    if (GetEcat() != NULL)
    {
        err = GetEcat()->GetMasterInfo(m_pMasterInfo);
    }
    return err;
}

int CMasterMotion::GetWmx3MasterState()
{
    return (int)(m_pMasterInfo->state);
}

int CMasterMotion::GetWmx3NumOfSlaves()
{
    return m_pMasterInfo->numOfSlaves;
}

int CMasterMotion::GetWmx3OnlineSlaveCount()
{
    return m_pMasterInfo->GetOnlineSlaveCount();
}

int CMasterMotion::GetWmx3OfflineSlaveCount()
{
    return m_pMasterInfo->GetOfflineSlaveCount();
}

int CMasterMotion::GetWmx3InaccessibleSlaveCount()
{
    return m_pMasterInfo->GetInaccessibleSlaveCount();
}

bool CMasterMotion::IsWmx3Valid()
{
    bool ret = false;
    ret = GetDevice()->IsDeviceValid();
    return ret;
}

int CMasterMotion::GetWmx3Version()
{
    int err = ErrorCode::None;
    GetLibVersion(&err);
    ASSERT(err == ErrorCode::None);
    GetIMDllVersion(&err);
    ASSERT(err == ErrorCode::None);
    return err;
}

UINT CMasterMotion::StartMasterStatus(LPVOID wParam)
{
    bool bLoop = true;
    int err = ErrorCode::None;
    CString strHostMsg;
    CMasterMotion* pThis = reinterpret_cast<CMasterMotion*>(wParam);
    AddThread(pThis);
    while (pThis->GetRunning() == true)
    {
        if (pThis->IsTerminated(THREAD_WMX3_MASTER_READTIME) == true)
        {
            TRACE(_T("[PWR] CMasterMotion(0x%X) Terminated\n"), pThis->m_ShowID);
            break;
        }
        //if (GetGlobalSimulationMode() == true)
        //{
        //    ThreadSleep(THREAD_WMX3_MASTER_READTIME);
        //    continue;
        //}
        pThis->m_MasterMotionInfo.m_IsValid = pThis->IsWmx3Valid();
        err = pThis->GetWmx3MasterInfo();
        if (err == ErrorCode::None)
        {
            pThis->m_MasterMotionInfo.m_cMasterState = pThis->GetWmx3MasterState();
            pThis->m_MasterMotionInfo.m_nSlaveCounts = pThis->GetWmx3NumOfSlaves();
            pThis->m_MasterMotionInfo.m_onLineSlaveCount = pThis->GetWmx3OnlineSlaveCount();
            pThis->m_MasterMotionInfo.m_offLineSlaveCount = pThis->GetWmx3OfflineSlaveCount();
            pThis->m_MasterMotionInfo.m_inAccessibleSlaveCount = pThis->GetWmx3InaccessibleSlaveCount();
        }
        ThreadSleep(THREAD_WMX3_MASTER_READTIME);
    };
    TRACE(_T("[PWR] CMasterMotion(0x%x) Quit\n"), pThis->m_ShowID);
    pThis->SetStart(false);
    pThis->SetEnd(true);
    return 0;
}

CMasterMotionInfo::CMasterMotionInfo()
{
    m_IsValid = false;
    m_cMasterState = 0;
    m_nSlaveCounts = 0;
    m_onLineSlaveCount = 0;
    m_offLineSlaveCount = 0;
    m_inAccessibleSlaveCount = 0;
}

CMasterMotionInfo::~CMasterMotionInfo()
{
}

CMasterMotionVersion::CMasterMotionVersion()
{
    ZeroMemory(m_LibraryVersion, sizeof(m_LibraryVersion));
    ZeroMemory(m_ImDllVersion, sizeof(m_LibraryVersion));
}

CMasterMotionVersion::CMasterMotionVersion(char* lib, char* dll)
{
    ZeroMemory(m_LibraryVersion, sizeof(m_LibraryVersion));
    ZeroMemory(m_ImDllVersion, sizeof(m_LibraryVersion));
    memcpy_s(m_LibraryVersion, sizeof(m_LibraryVersion), lib, sizeof(lib));
    memcpy_s(m_ImDllVersion, sizeof(m_ImDllVersion), dll, sizeof(dll));
}

CMasterMotionVersion::~CMasterMotionVersion()
{
}
