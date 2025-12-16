#pragma once

#include "GlobalDefine.h"

using namespace std;
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

class CSqlite3File
{
public:
	CSqlite3File();
	~CSqlite3File();

	CString ToStringInt(long index);
	CString ToStringDouble(double value);
	CString ToStringGantry(long Gantry);
	CString ToStringHead(long Gantry, long HeadNo);

	sqlite3* OpenCreateDB(CString fileName, bool* result);
	sqlite3* OpenDB(CString fileName, bool* result);
	void CloseDB(sqlite3* db);

	long BeginTransaction(sqlite3* db);
	long EndTransaction(sqlite3* db);
	//bool CheckTableExist(sqlite3* db, CString TableName);
	CString SqlCreateTable(CString TableName, CArray<CString>* ColumnName);
	long SqlAddColumn(sqlite3* db, CString TableName, CString ColumnName, CString Type);
	long SqlInsertValue(sqlite3* db, CString TableName, CString ColumnName, CString Value);
	long SqlInsertValueMultiRow(sqlite3* db, CString TableName, CString ColumnName, CArray<CString>* Value);
	long SqlupdateValue(sqlite3* db, CString Table, CString UpdateColumn, CString UpdateValue, CString ConditionColumn, CString ConditionValue);
	long SqlupdateValueMulti(sqlite3* db, CString Table, CArray<CString>* UpdateColumn, CArray<CString>* UpdateValue, CString ConditionColumn, CString ConditionValue);
	bool SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, double* SearchResult);
	bool SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, long* SearchResult);

	void SetLoadComplete(bool set);
	bool GetLoadComplete();
	bool _CreateDirectory(LPCWSTR lpszPath);
	bool CheckColumnExist(sqlite3* db, CString TableName, CString ColumnName);
	bool GetRowCount(sqlite3* db, CString TableName, CString ColumnName, long* count);
	//bool _CreateDirectory(CString Path);

private:

	HANDLE m_Lock;
	bool m_LoadComplete;

protected:
	void Lock();
	void Unlock();
};


extern CSqlite3File* gcSqlite3File;