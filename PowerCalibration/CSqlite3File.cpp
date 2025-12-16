#include "pch.h"
#include "CSqlite3File.h"
#include "LockDef.h"
#include "Trace.h"
#include <string>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

CSqlite3File* gcSqlite3File;
CSqlite3File::CSqlite3File()
{
	m_LoadComplete = false;
	m_Lock = CreateMutex(NULL, FALSE, NULL);
}

CSqlite3File::~CSqlite3File()
{
	CloseHandle(m_Lock);
}

CString CSqlite3File::ToStringInt(long index)
{
	CString str;
	str.Format(_T("%d"), index);

	return str;
}

CString CSqlite3File::ToStringDouble(double value)
{
	CString str;
	str.Format(_T("%.3f"), value);
	return str;
}

CString CSqlite3File::ToStringGantry(long Gantry)
{
	CString str;

	if (Gantry == FRONT_GANTRY)
	{
		return _T("Front");
	}
	else
	{
		return _T("Rear");
	}
}

CString CSqlite3File::ToStringHead(long Gantry, long HeadNo)
{
	CString strHead;

	if (Gantry == FRONT_GANTRY)
	{
		strHead.Format(_T("FrontHead%d"), HeadNo);
	}
	else
	{
		strHead.Format(_T("RearHead%d"), HeadNo);
	}
	return strHead;
}

sqlite3* CSqlite3File::OpenCreateDB(CString fileName, bool* result)
{
	CString strFunc(__func__);
	std::string strFilePath = CT2CA(fileName);
	int rc;

	char* zErrMsg = 0;
	sqlite3* db;

	rc = sqlite3_open_v2(strFilePath.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
	if (rc)
	{
		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, fileName);
		sqlite3_free(zErrMsg);
		*result = false;
	}
	else
	{
		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, fileName);
		*result = true;
	}

	return db;
}

sqlite3* CSqlite3File::OpenDB(CString fileName, bool* result)
{
	CString strFunc(__func__);
	std::string strFilePath = CT2CA(fileName);
	int rc;

	char* zErrMsg = 0;
	sqlite3* db;

	rc = sqlite3_open_v2(strFilePath.c_str(), &db, SQLITE_OPEN_READWRITE, NULL);
	if (rc)
	{
		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, fileName);
		sqlite3_free(zErrMsg);
		*result = false;
	}
	else
	{
		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, fileName);
		*result = true;
	}

	return db;
}

void CSqlite3File::CloseDB(sqlite3* db)
{
	CString strFunc(__func__);
	int rc = sqlite3_close(db);

	if (rc)
	{
		TRACE(_T("[PWR] %s Error:%d)\n"), strFunc, rc );
	}
	else
	{
		TRACE(_T("[PWR] %s Success.\n"), strFunc);
	}

}

long CSqlite3File::BeginTransaction(sqlite3* db)
{
	char* zErrMsg = 0;
	return sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &zErrMsg);
}

long CSqlite3File::EndTransaction(sqlite3* db)
{
	char* zErrMsg = 0;
	return 	sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &zErrMsg);
}

//bool CSqlite3File::CheckTableExist(sqlite3* db, CString TableName)
//{
//	CString strFunc(__func__);
//	char* zErrMsg = 0;
//	int rc;
//	bool result = false;
//
//	sqlite3* db2;
//
//	rc = sqlite3_open_v2(strFilePath.c_str(), &db2, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, m_FileNameCalDB);
//		sqlite3_free(zErrMsg);
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, m_FileNameCalDB);
//		result = true;
//	}
//
//
//
//	CString strTemp;
//	strTemp.Format(_T("SELECT name FROM sqlite_master WHERE type='table' AND name='{%s}';"), (LPCTSTR)TableName);
//	CString sql = _T("CREATE TABLE IF NOT EXISTS TB_SETTING(SEQ INTEGER PRIMARY KEY AUTOINCREMENT, SET1 TEXT, SET2 TEXT, SET3 TEXT); ");
//	std::string strSql = CT2CA(sql);
//
//	rc = sqlite3_exec(db2, strSql.c_str(), 0, 0, &zErrMsg);
//
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Table:%s None.)\n"), strFunc, TableName);
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Table:%s Exist.)\n"), strFunc, TableName);
//		result = true;
//	}
//
//	return result;
//}

CString CSqlite3File::SqlCreateTable(CString TableName, CArray<CString>* ColumnName)
{
	CString strT;
	CString strC = _T("");
	CString strSql = _T("");

	for (long i = 0; i < ColumnName->GetCount(); i++)
	{
		strC.Format(_T(", %s TEXT PRIMARY KEY NOT NULL"), (LPCTSTR)ColumnName->GetAt(i));

		//if (i == 0)
		//{
		//	strC.Format(_T(", %s TEXT PRIMARY KEY NOT NULL"), (LPCTSTR)ColumnName->GetAt(i));
		//}
		//else
		//{
		//	strC.AppendFormat(_T(", %s TEXT"), (LPCTSTR)ColumnName->GetAt(i));
		//}
	}

	strSql.Format(_T("%s (%s);"), (LPCTSTR)strT, (LPCTSTR)strC);
	return strSql;
}

long CSqlite3File::SqlAddColumn(sqlite3* db, CString TableName, CString ColumnName, CString Type)
{
	std::string strSql;
	CString strTemp;
	char* zErrMsg = 0;
	CString strFunc(__func__);
	strTemp.Format(_T("ALTER TABLE '%s' ADD COLUMN '%s'[%s];"), (LPCTSTR)TableName, (LPCTSTR)ColumnName, (LPCTSTR)Type);
	strSql = CT2CA(strTemp);
	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc == SQLITE_OK)
	{
		TRACE(_T("[PWR] %s AddColumn %s %s\n"), strFunc, TableName, ColumnName);
	}
	return rc;
}

long CSqlite3File::SqlInsertValue(sqlite3* db, CString TableName, CString ColumnName, CString Value)
{
	std::string strSql;
	CString strTemp;
	char* zErrMsg = 0;
	CString strFunc(__func__);
	strTemp.Format(_T("INSERT INTO '%s' ('%s') VALUES ('%s');"), (LPCTSTR)TableName, (LPCTSTR)ColumnName, (LPCTSTR)Value);
	strSql = CT2CA(strTemp);
	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		TRACE(_T("[PWR] %s Err:%d %s %s %s\n"), strFunc, rc, TableName, ColumnName, Value);
	}


	return rc;
}

long CSqlite3File::SqlInsertValueMultiRow(sqlite3* db, CString TableName, CString ColumnName, CArray<CString>* Value)
{
	std::string strSql;
	CString strTemp;
	char* zErrMsg = 0;
	CString strFunc(__func__);
	CString strValue;

	for (long i = 0; i < Value->GetCount(); i++)
	{
		if (i == 0)
		{
			strValue.Format(_T("('%s')"), (LPCTSTR)Value->GetAt(i));
		}
		else
		{
			strValue.AppendFormat(_T(", ('%s')"), (LPCTSTR)Value->GetAt(i));
		}
	}

	strTemp.Format(_T("INSERT INTO '%s' ('%s') VALUES %s;"), (LPCTSTR)TableName, (LPCTSTR)ColumnName, (LPCTSTR)strValue);
	strSql = CT2CA(strTemp);
	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		TRACE(_T("[PWR] %s Err:%d %s\n"), strFunc, rc, TableName);
	}

	return rc;
}

long CSqlite3File::SqlupdateValue(sqlite3* db, CString Table, CString UpdateColumn, CString UpdateValue, CString ConditionColumn, CString ConditionValue)
{
	std::string strSql;
	CString strTemp;
	char* zErrMsg = 0;
	CString strFunc(__func__);
	strTemp.Format(_T("UPDATE '%s' SET %s = '%s' WHERE %s = '%s';"), 
		(LPCTSTR)Table, (LPCTSTR)UpdateColumn, (LPCTSTR)UpdateValue, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
	strSql = CT2CA(strTemp);
	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);

	if (rc != SQLITE_OK)
	{
		TRACE(_T("[PWR] %s Err:%d %s\n"), strFunc, rc, strTemp);

	}

	return rc;
}

long CSqlite3File::SqlupdateValueMulti(sqlite3* db, CString Table, CArray<CString>* UpdateColumn, CArray<CString>* UpdateValue, CString ConditionColumn, CString ConditionValue)
{
	std::string strSql;
	CString strTemp;
	char* zErrMsg = 0;
	CString strFunc(__func__);

	CString strUpdate;

	for (long i = 0; i < UpdateColumn->GetCount(); i++)
	{
		if (i == 0)
		{
			strUpdate.Format(_T("%s = '%s'"), (LPCTSTR)UpdateColumn->GetAt(i), (LPCTSTR)UpdateValue->GetAt(i));
		}
		else
		{
			strUpdate.AppendFormat(_T(",%s = '%s'"), (LPCTSTR)UpdateColumn->GetAt(i), (LPCTSTR)UpdateValue->GetAt(i));
		}
	}


	strTemp.Format(_T("UPDATE '%s' SET %s WHERE %s = '%s';"),
		(LPCTSTR)Table, (LPCTSTR)strUpdate, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
	strSql = CT2CA(strTemp);
	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);

	return rc;
}

bool CSqlite3File::SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, double* SearchResult)
{
	CString strFunc(__func__);
	char* zErrMsg = 0;
	int rc;
	std::string strSql;
	CString strTemp;
	char** results;
	char* error;
	char* end;
	double convert;

	strTemp.Format(_T("SELECT %s FROM %s WHERE %s = '%s';"),(LPCTSTR)SearchColumn, (LPCTSTR)SearchTable, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
	strSql = CT2CA(strTemp);
	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);

	if (rc != SQLITE_OK || results[1] == NULL)
	{
		TRACE(_T("[PWR] %s SqlErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	convert = std::strtold(results[1], &end);
	if (*end != '\0')
	{
		TRACE(_T("[PWR] %s ConvertErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	*SearchResult = convert;

	return true;
}

bool CSqlite3File::SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, long* SearchResult)
{
	CString strFunc(__func__);
	char* zErrMsg = 0;
	int rc;
	std::string strSql;
	CString strTemp;
	char** results;
	char* error;
	char* end;
	long convert;

	strTemp.Format(_T("SELECT %s FROM %s WHERE %s = '%s';"), (LPCTSTR)SearchColumn, (LPCTSTR)SearchTable, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
	strSql = CT2CA(strTemp);
	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);

	if (rc != SQLITE_OK || results[1] == NULL)
	{
		TRACE(_T("[PWR] %s SqlErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	convert = std::strtol(results[1], &end, 10);
	if (*end != '\0')
	{
		TRACE(_T("[PWR] %s ConvertErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	*SearchResult = convert;

	return true;
}

void CSqlite3File::SetLoadComplete(bool set)
{
	m_LoadComplete = set;
}

bool CSqlite3File::GetLoadComplete()
{
	return m_LoadComplete;
}

bool CSqlite3File::CheckColumnExist(sqlite3* db, CString TableName, CString ColumnName)
{
	CString strFunc(__func__);
	char* zErrMsg = 0;
	int rc;
	std::string strSql;
	CString strTemp;

	strTemp.Format(_T("SELECT %s FROM %s WHERE ROWID = 1;"), (LPCTSTR)ColumnName, (LPCTSTR)TableName);
	strSql = CT2CA(strTemp);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc == SQLITE_OK)
	{
		TRACE(_T("[PWR] %s Table:%s Column:%s Exist)\n"), strFunc, TableName, ColumnName);
		return true;
	}

	TRACE(_T("[PWR] %s Table:%s Column:%s None)\n"), strFunc, TableName, ColumnName);
	return false;
}

bool CSqlite3File::GetRowCount(sqlite3* db, CString TableName, CString ColumnName, long* count)
{
	CString strFunc(__func__);
	char* zErrMsg = 0;
	int rc;
	std::string strSql;
	CString strTemp;
	char** results;
	char* error;
	char* end;
	long convert;

	strTemp.Format(_T("SELECT count(%s) FROM %s;"), (LPCTSTR)ColumnName, (LPCTSTR)TableName);
	strSql = CT2CA(strTemp);
	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);

	if (rc != SQLITE_OK || results[1] == NULL)
	{
		TRACE(_T("[PWR] %s SqlErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	convert = std::strtol(results[1], &end, 10);
	if (*end != '\0')
	{
		TRACE(_T("[PWR] %s ConvertErr:%d (%s)\n"), strFunc, rc, strTemp);
		return false;
	}

	*count = convert;

	return true;
}

void CSqlite3File::Lock()
{
	SEM_LOCK(m_Lock, INFINITE);

}

void CSqlite3File::Unlock()
{
	SEM_UNLOCK(m_Lock);

}


