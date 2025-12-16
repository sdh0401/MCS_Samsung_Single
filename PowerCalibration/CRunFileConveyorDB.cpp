#include "pch.h"
#include "CRunFileConveyorDB.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorData.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
#include "CMachineConfig.h"
#include "CPowerConveyorData.h"

using namespace std;
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

CRunFileConveyorDB* gcRuntimeConveyorDB;
CRunFileConveyorDB::CRunFileConveyorDB()
{
	m_FilePath.Format(_T("C:\\Power\\i6.0\\MCS\\MachineConfig"));
	m_FileName.Format(_T("RuntimeConveyor.db"));
	m_FilePathName.Format(_T("%s\\%s"), (LPCTSTR)m_FilePath, (LPCTSTR)m_FileName);

	m_Conveyor.table = _T("RuntimeConveyor");
	m_Conveyor.column_Base = _T("Base");
	m_Conveyor.column_LastWidth = _T("LastWidth");
	m_Conveyor.column_LastPusherZ = _T("LastPusherZ");
	m_Conveyor.column_WidthOption = _T("WidthOption");
	m_Conveyor.column_InsertDone = _T("InsertDone");
	m_Conveyor.column_PcbOutDone = _T("PcbOutDone");
	m_Conveyor.column_PusherZOption = _T("PusherZOption");
}

CRunFileConveyorDB::~CRunFileConveyorDB()
{

}

bool CRunFileConveyorDB::CheckDBOpen()
{
	bool result;
	sqlite3* db = OpenDB(m_FilePathName, &result);
    sqlite3_close(db);

	return result;
}

bool CRunFileConveyorDB::InitialRuntimeConveyor()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	RUNTIME_CONVEYOR data = m_Conveyor;

	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' INT, '%s' INT, '%s' INT, '%s' INT);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Base,
		(LPCTSTR)data.column_LastWidth, (LPCTSTR)data.column_LastPusherZ, (LPCTSTR)data.column_WidthOption, 
		(LPCTSTR)data.column_InsertDone, (LPCTSTR)data.column_PcbOutDone, (LPCTSTR)data.column_PusherZOption);

	strSql = CT2CA(strTemp);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		strErr = (LPCSTR)(LPSTR)zErrMsg;
		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
		return false;
	}

	BeginTransaction(db);

	SqlInsertValue(db, data.table, data.column_Base, ToStringGantry(FRONT_STAGE));	

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}


bool CRunFileConveyorDB::SaveRuntimeConveyor()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	long widthoption;
	double widthLast;
	double pusherzLast;
	long insertDone;
	long pcbOutDone;
	long pusherzOption;

	Lock();
	db = OpenCreateDB(m_FilePathName , &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	widthoption = gcPowerConveyorData->GetWidthOption();
	widthLast = gcPowerConveyorData->GetWidth(base, WORK1_CONV);
	pusherzLast = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
	insertDone = gcPowerConveyorData->GetInsertDone(FRONT_CONV);
	pcbOutDone = gcPowerConveyorData->GetPcbOutDone(FRONT_CONV);
	pusherzOption = gcPowerConveyorData->GetPusherZOption();

	UpdateLastWidth(db, base, widthLast);
	UpdateLastPusherZ(db, base, pusherzLast);
	UpdateWidthOption(db, base, widthoption);
	UpdateInsertDone(db, base, insertDone);
	UpdatePcbOutDone(db, base, pcbOutDone);
	UpdatePusherZOption(db, base, pusherzOption);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::SaveLastWidth()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	double widthLast;

	Lock();
	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);
	widthLast = gcPowerConveyorData->GetWidth(base, WORK1_CONV);
	UpdateLastWidth(db, base, widthLast);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::SaveLastPushserZ()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	double pusherzLast;

	Lock();
	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	pusherzLast = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
	UpdateLastPusherZ(db, base, pusherzLast);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::SaveInsertDone()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	long insertDone;

	Lock();
	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	insertDone = gcPowerConveyorData->GetInsertDone(FRONT_CONV);
	UpdateInsertDone(db, base, insertDone);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::SavePcbOutDone()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	long pcbOutDone;

	Lock();
	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	pcbOutDone = gcPowerConveyorData->GetPcbOutDone(FRONT_CONV);
	UpdatePcbOutDone(db, base, pcbOutDone);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::SavePusherZOption()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	int rc;
	bool result = false;
	sqlite3* db;
	long base = FRONT_STAGE;
	long pusherZOption;

	Lock();
	db = OpenCreateDB(m_FilePathName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	pusherZOption = gcPowerConveyorData->GetPusherZOption();
	UpdatePusherZOption(db, base, pusherZOption);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	Unlock();
	return true;
}

bool CRunFileConveyorDB::LoadRuntimeConveyor()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	RUNTIME_CONVEYOR dataConfig = m_Conveyor;
	long base = FRONT_CONV;
	long widthOption = 0;
	double width = 0.0;
	double pusherz = 0.0;
	long insetDone = 0;
	long pcbOutDone = 0;
	long pusherZOption = 0;


	db = OpenDB(m_FilePathName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_WidthOption, dataConfig.column_Base, ToStringGantry(base), &widthOption) == true)
	{
		gcPowerConveyorData->SetWidthOption(widthOption);
	}

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_LastWidth, dataConfig.column_Base, ToStringGantry(base), &width) == true)
	{
		gcPowerConveyorData->SetWidth(base, WORK1_CONV, width);
	}

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_LastPusherZ, dataConfig.column_Base, ToStringGantry(base), &pusherz) == true)
	{
		gcPowerConveyorData->SetPusherZ(base, WORK1_CONV, pusherz);
	}

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_InsertDone, dataConfig.column_Base, ToStringGantry(base), &insetDone) == true)
	{
		gcPowerConveyorData->SetInsertDone(base, insetDone);
	}

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PcbOutDone, dataConfig.column_Base, ToStringGantry(base), &pcbOutDone) == true)
	{
		gcPowerConveyorData->SetPcbOutDone(base, pcbOutDone);
	}

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PusherZOption, dataConfig.column_Base, ToStringGantry(base), &pusherZOption) == true)
	{
		gcPowerConveyorData->SetPusherZOption(pusherZOption);
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	SetLoadComplete(true);

	return true;
}

long CRunFileConveyorDB::UpdateLastWidth(sqlite3* db, long Base, double Width)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_LastWidth, ToStringDouble(Width), data.column_Base, ToStringGantry(Base));
}

long CRunFileConveyorDB::UpdateLastPusherZ(sqlite3* db, long Base, double PusherZ)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_LastPusherZ, ToStringDouble(PusherZ), data.column_Base, ToStringGantry(Base));
}

long CRunFileConveyorDB::UpdateWidthOption(sqlite3* db, long Base, long WidthOption)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_WidthOption, ToStringInt(WidthOption), data.column_Base, ToStringGantry(Base));
}

long CRunFileConveyorDB::UpdateInsertDone(sqlite3* db, long Base, long InsertDone)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_InsertDone, ToStringInt(InsertDone), data.column_Base, ToStringGantry(Base));
}

long CRunFileConveyorDB::UpdatePcbOutDone(sqlite3* db, long Base, long PcbOutDone)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_PcbOutDone, ToStringInt(PcbOutDone), data.column_Base, ToStringGantry(Base));
}

long CRunFileConveyorDB::UpdatePusherZOption(sqlite3* db, long Base, long PusherZOption)
{
	RUNTIME_CONVEYOR data = m_Conveyor;

	return SqlupdateValue(db, data.table, data.column_PusherZOption, ToStringInt(PusherZOption), data.column_Base, ToStringGantry(Base));
}

