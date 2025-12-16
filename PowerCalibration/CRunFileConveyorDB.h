#pragma once

#include "GlobalDefine.h"
#include "CSqlite3File.h"

using namespace std;
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

typedef struct {

	CString table;
	CString column_Base;
	CString column_LastWidth;
	CString column_LastPusherZ;
	CString column_WidthOption;
	CString column_InsertDone;
	CString column_PcbOutDone;
	CString column_PusherZOption;

}RUNTIME_CONVEYOR;

class CRunFileConveyorDB : public CSqlite3File
{
public:
	CRunFileConveyorDB();
	~CRunFileConveyorDB();

	bool CheckDBOpen();
	bool InitialRuntimeConveyor();

	bool SaveRuntimeConveyor();
	bool SaveLastWidth();
	bool SaveLastPushserZ();
	bool SaveInsertDone();
	bool SavePcbOutDone();
	bool SavePusherZOption();

	bool LoadRuntimeConveyor();

	long UpdateLastWidth(sqlite3* db, long Base, double Width);
	long UpdateLastPusherZ(sqlite3* db, long Base, double PusherZ);
	long UpdateWidthOption(sqlite3* db, long Base, long WidthOption);
	long UpdateInsertDone(sqlite3* db, long Base, long InsertDone);
	long UpdatePcbOutDone(sqlite3* db, long Base, long PcbOutDone);
	long UpdatePusherZOption(sqlite3* db, long Base, long PusherZOption);

private:
	CString m_FilePathName;
	CString m_FilePath;
	CString m_FileName;
	RUNTIME_CONVEYOR m_Conveyor;

};


extern CRunFileConveyorDB* gcRuntimeConveyorDB;