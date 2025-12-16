#include "pch.h"
#include "CMachineFileDB.h"
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

CMachineFileDB* gcMachineFileDB;
CMachineFileDB::CMachineFileDB()
{
	m_FileName.Format(_T("C:\\Power\\i6.0\\MCS\\MachineConfig\\CalibrationData.db"));
	m_ColumnDate = _T("Date");
	m_sqlMax = 500;

	// Axis Parameter
	m_AxisParameter.table =						_T("AxisParameter");
	m_AxisParameter.column_AxisNumber =			_T("AxisNumber");
	m_AxisParameter.column_AxisName =			_T("AxisName");
	m_AxisParameter.column_HomePosition =		_T("HomePosition");
	m_AxisParameter.column_MinusLimit =			_T("MinusLimit");
	m_AxisParameter.column_PlusLimit =			_T("PlusLimit");
	m_AxisParameter.column_HomeShiftDistance =	_T("HomeShiftDistance");

	// 1D
	m_1dConfig.table = _T("Compen1dConfig");
	m_1dConfig.column_Gantry = _T("Gantry");
	m_1dConfig.column_StartY =	_T("StartPositionY");

	m_1dTableFront.table = _T("Compen1dTableFront");
	m_1dTableFront.column_Index = _T("CompenIndex");
	m_1dTableFront.column_CompenY = _T("CompenY");

	m_1dTableRear = m_1dTableFront;
	m_1dTableRear.table = _T("Compen1dTableRear");

	// 2D
	m_2dConfig.table = _T("Compen2dConfig");
	m_2dConfig.column_Gantry = _T("Gantry");
	m_2dConfig.column_StartX = _T("StartPositionX");
	m_2dConfig.column_StartY = _T("StartPositionY");

	m_2dTableFront.table = _T("Compen2dTableFront");
	m_2dTableFront.column_Index = _T("CompenIndex");
	m_2dTableFront.column_CompenX = _T("CompenX");
	m_2dTableFront.column_CompenY = _T("CompenY");

	m_2dTableRear = m_2dTableFront;
	m_2dTableRear.table = _T("Compen2dTableRear");

	// Align
	m_Align.table = _T("Align");
	m_Align.column_Base = _T("Base");
	m_Align.column_AlignCamera = _T("AlignCamera");
	m_Align.column_PosiionX = _T("PositionX");
	m_Align.column_PosiionY = _T("PositionY");

	// Recog Position
	RecognitionModule.table = _T("RecognitionModule");
	RecognitionModule.column_Head = _T("HeadName");
	RecognitionModule.column_PositionX = _T("PositionX");
	RecognitionModule.column_PositionY = _T("PositionY");
	RecognitionModule.column_VisionOffsetX = _T("VisionOffsetX");
	RecognitionModule.column_VisionOffsetY = _T("VisionOffsetY");

	// Head Offset
	m_HeadOffset.table = _T("HeadOffset");
	m_HeadOffset.column_Head = _T("Head");
	m_HeadOffset.column_OffsetX = _T("OffsetX");
	m_HeadOffset.column_OffsetY = _T("OffsetY");
	m_HeadOffset.column_Radius = _T("Radius");

	// Height Measurement
	m_HeightMeasure.table = _T("HeightMeasurement");
	m_HeightMeasure.column_Gantry = _T("Gantry");
	m_HeightMeasure.column_OffsetX = _T("OffsetX");
	m_HeightMeasure.column_OffsetY = _T("OffsetY");
	m_HeightMeasure.column_OffsetZero = _T("OffsetZero");

	// ANC
	m_AncConfig.table = _T("AncConfig");
	m_AncConfig.column_Base = _T("Base");
	m_AncConfig.column_Mark1PositionX = _T("Mark1PositionX");
	m_AncConfig.column_Mark1PositionY = _T("Mark1PositionY");
	m_AncConfig.column_Mark2PositionX = _T("Mark2PositionX");
	m_AncConfig.column_Mark2PositionY = _T("Mark2PositionY");

	m_AncFront.table = _T("AncFront");
	m_AncFront.column_HoleNumber = _T("HoleNumber");
	m_AncFront.column_HolePositionX = _T("HolePositionX");
	m_AncFront.column_HolePositionY = _T("HolePositionY");
	m_AncFront.column_HolePositionR = _T("HolePositionR");
	m_AncFront.column_HolePositionZ = _T("HolePositionZ");

	m_AncRear = m_AncFront;
	m_AncRear.table = _T("AncRear");

	// PCB fix
	m_PcbFix.table = _T("PcbFixPosition");
	m_PcbFix.column_Base = _T("Base");
	m_PcbFix.column_PositionX = _T("FixPositionX");
	m_PcbFix.column_PositionY = _T("FixPositionY");

	// Feeder Refer
	m_FeederReference.table = _T("FeederReference");
	m_FeederReference.column_Base = _T("Base");
	m_FeederReference.column_FeederNumber = _T("ReferenceFeederNumber");
	m_FeederReference.column_PositionX = _T("ReferencePositionX");
	m_FeederReference.column_PositionY = _T("ReferencePositionY");
	m_FeederReference.column_Pitch = _T("Pitch");
	// Runtime Front Conveyor
	//m_RunTimeFrontConveyor.table = _T("RuntimeData");
	//m_RunTimeFrontConveyor.column_Base = _T("Base");
	//m_RunTimeFrontConveyor.column_LastWidth = _T("LastPositionWidth");
	//m_RunTimeFrontConveyor.column_LastPusherZ = _T("LastPositionPusherZ");
	// 
			// InsertOffset4532
	m_InsertOffset4532.table = _T("InsertOffset4532");
	m_InsertOffset4532.column_Head = _T("HeadName");
	m_InsertOffset4532.column_OffsetX_0 = _T("OffsetX_0");
	m_InsertOffset4532.column_OffsetY_0 = _T("OffsetY_0");
	m_InsertOffset4532.column_OffsetX_90 = _T("OffsetX_90");
	m_InsertOffset4532.column_OffsetY_90 = _T("OffsetY_90");
	m_InsertOffset4532.column_OffsetX_180 = _T("OffsetX_180");
	m_InsertOffset4532.column_OffsetY_180 = _T("OffsetY_180");
	m_InsertOffset4532.column_OffsetX_270 = _T("OffsetX_270");
	m_InsertOffset4532.column_OffsetY_270 = _T("OffsetY_270");
}

CMachineFileDB::~CMachineFileDB()
{
}

//
//CString CMachineFileDB::ToStringInt(long index)
//{
//	CString str;
//	str.Format(_T("%d"), index);
//
//	return str;
//}
//
//CString CMachineFileDB::ToStringDouble(double value)
//{
//	CString str;
//	str.Format(_T("%.3f"), value);
//	return str;
//}
//
//CString CMachineFileDB::ToStringGantry(long Gantry)
//{
//	CString str;
//
//	if (Gantry == FRONT_GANTRY)
//	{
//		return _T("Front");
//	}
//	else
//	{
//		return _T("Rear");
//	}
//}
//
//CString CMachineFileDB::ToStringHead(long Gantry, long HeadNo)
//{
//	CString strHead;
//
//	if (Gantry == FRONT_GANTRY)
//	{
//		strHead.Format(_T("FrontHead%d"), HeadNo);
//	}
//	else
//	{
//		strHead.Format(_T("RearHead%d"), HeadNo);
//	}
//	return strHead;
//}
//
//sqlite3* CMachineFileDB::OpenCreateCalibration(bool* result)
//{
//	CString strFunc(__func__);
//	std::string strFilePath = CT2CA(m_FileNameCalDB);
//	int rc;
//
//	char* zErrMsg = 0;	
//	sqlite3* db;
//
//	rc = sqlite3_open_v2(strFilePath.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, m_FileNameCalDB);
//		sqlite3_free(zErrMsg);
//		*result = false;
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, m_FileNameCalDB);
//		*result = true;
//	}
//
//	return db;
//}
//
//sqlite3* CMachineFileDB::OpenCreateFile(CString fileName, bool* result)
//{
//	CString strFunc(__func__);
//	std::string strFilePath = CT2CA(fileName);
//	int rc;
//
//	char* zErrMsg = 0;
//	sqlite3* db;
//
//	rc = sqlite3_open_v2(strFilePath.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, fileName);
//		sqlite3_free(zErrMsg);
//		*result = false;
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, fileName);
//		*result = true;
//	}
//
//	return db;
//}
//
//sqlite3* CMachineFileDB::OpenCalibration(bool* result)
//{
//	CString strFunc(__func__);
//	std::string strFilePath = CT2CA(m_FileNameCalDB);
//	int rc;
//
//	char* zErrMsg = 0;
//	sqlite3* db;
//
//	rc = sqlite3_open_v2(strFilePath.c_str(), &db, SQLITE_OPEN_READWRITE, NULL);
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, m_FileNameCalDB);
//		sqlite3_free(zErrMsg);
//		*result = false;
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, m_FileNameCalDB);
//		*result = true;
//	}
//
//	return db;
//}
//
//void CMachineFileDB::CloseDB(sqlite3* db)
//{
//	CString strFunc(__func__);
//	int rc = sqlite3_close(db);
//
//	if (rc)
//	{
//		TRACE(_T("[PWR] %s Error:%d, %s)\n"), strFunc, rc, m_FileNameCalDB);
//	}
//	else
//	{
//		TRACE(_T("[PWR] %s Success. %s\n"), strFunc, m_FileNameCalDB);
//	}
//
//}
//
//long CMachineFileDB::BeginTransaction(sqlite3* db)
//{
//	char* zErrMsg = 0;
//	return sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &zErrMsg);
//}
//
//long CMachineFileDB::EndTransaction(sqlite3* db)
//{
//	char* zErrMsg = 0;
//	return 	sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &zErrMsg);
//}
//
//bool CMachineFileDB::CheckTableExist(sqlite3* db, CString TableName)
//{
//	CString strFunc(__func__);
//	char* zErrMsg = 0;
//	int rc;
//	bool result = false;
//
//	std::string strFilePath = CT2CA(m_FileNameCalDB);
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
//
//CString CMachineFileDB::SqlCreateTable(CString TableName, CArray<CString>* ColumnName)
//{
//	CString strT;
//	CString strC = _T("");
//	CString strSql = _T("");
//
//	for (long i = 0; i < ColumnName->GetCount(); i++)
//	{
//		strC.Format(_T(", %s TEXT PRIMARY KEY NOT NULL"), (LPCTSTR)ColumnName->GetAt(i));
//
//		//if (i == 0)
//		//{
//		//	strC.Format(_T(", %s TEXT PRIMARY KEY NOT NULL"), (LPCTSTR)ColumnName->GetAt(i));
//		//}
//		//else
//		//{
//		//	strC.AppendFormat(_T(", %s TEXT"), (LPCTSTR)ColumnName->GetAt(i));
//		//}
//	}
//
//	strSql.Format(_T("%s (%s);"), (LPCTSTR)strT, (LPCTSTR)strC);
//	return strSql;
//}
//
//long CMachineFileDB::SqlAddColumn(sqlite3* db, CString TableName, CString ColumnName)
//{
//	std::string strSql;
//	CString strTemp;
//	char* zErrMsg = 0;
//	CString strFunc(__func__);
//	strTemp.Format(_T("ALTER TABLE '%s' ADD COLUMN '%s'[TEXT];"), (LPCTSTR)TableName, (LPCTSTR)ColumnName);
//	strSql = CT2CA(strTemp);
//	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//	if (rc == SQLITE_OK)
//	{
//		TRACE(_T("[PWR] %s AddColumn %s %s\n"), strFunc, TableName, ColumnName);
//	}
//	return rc;
//}
//
//long CMachineFileDB::SqlInsertValue(sqlite3* db, CString TableName, CString ColumnName, CString Value)
//{
//	std::string strSql;
//	CString strTemp;
//	char* zErrMsg = 0;
//	CString strFunc(__func__);
//	strTemp.Format(_T("INSERT INTO '%s' ('%s') VALUES ('%s');"), (LPCTSTR)TableName, (LPCTSTR)ColumnName, (LPCTSTR)Value);
//	strSql = CT2CA(strTemp);
//	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//
//	if (rc != SQLITE_OK)
//	{
//		TRACE(_T("[PWR] %s Err:%d %s %s %s\n"), strFunc, rc, TableName, ColumnName, Value);
//	}
//
//
//	return rc;
//}
//
//long CMachineFileDB::SqlInsertValueMultiRow(sqlite3* db, CString TableName, CString ColumnName, CArray<CString>* Value)
//{
//	std::string strSql;
//	CString strTemp;
//	char* zErrMsg = 0;
//	CString strFunc(__func__);
//	CString strValue;
//
//	for (long i = 0; i < Value->GetCount(); i++)
//	{
//		if (i == 0)
//		{
//			strValue.Format(_T("('%s')"), (LPCTSTR)Value->GetAt(i));
//		}
//		else
//		{
//			strValue.AppendFormat(_T(", ('%s')"), (LPCTSTR)Value->GetAt(i));
//		}
//	}
//
//	strTemp.Format(_T("INSERT INTO '%s' ('%s') VALUES %s;"), (LPCTSTR)TableName, (LPCTSTR)ColumnName, (LPCTSTR)strValue);
//	strSql = CT2CA(strTemp);
//	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//
//	if (rc != SQLITE_OK)
//	{
//		TRACE(_T("[PWR] %s Err:%d %s\n"), strFunc, rc, TableName);
//	}
//
//	return rc;
//}
//
//long CMachineFileDB::SqlupdateValue(sqlite3* db, CString Table, CString UpdateColumn, CString UpdateValue, CString ConditionColumn, CString ConditionValue)
//{
//	std::string strSql;
//	CString strTemp;
//	char* zErrMsg = 0;
//	CString strFunc(__func__);
//	strTemp.Format(_T("UPDATE '%s' SET %s = '%s' WHERE %s = '%s';"), 
//		(LPCTSTR)Table, (LPCTSTR)UpdateColumn, (LPCTSTR)UpdateValue, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
//	strSql = CT2CA(strTemp);
//	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//
//	if (rc != SQLITE_OK)
//	{
//		TRACE(_T("[PWR] %s Err:%d %s\n"), strFunc, rc, strTemp);
//
//	}
//
//	return rc;
//}
//
//long CMachineFileDB::SqlupdateValueMulti(sqlite3* db, CString Table, CArray<CString>* UpdateColumn, CArray<CString>* UpdateValue, CString ConditionColumn, CString ConditionValue)
//{
//	std::string strSql;
//	CString strTemp;
//	char* zErrMsg = 0;
//	CString strFunc(__func__);
//
//	CString strUpdate;
//
//	for (long i = 0; i < UpdateColumn->GetCount(); i++)
//	{
//		if (i == 0)
//		{
//			strUpdate.Format(_T("%s = '%s'"), (LPCTSTR)UpdateColumn->GetAt(i), (LPCTSTR)UpdateValue->GetAt(i));
//		}
//		else
//		{
//			strUpdate.AppendFormat(_T(",%s = '%s'"), (LPCTSTR)UpdateColumn->GetAt(i), (LPCTSTR)UpdateValue->GetAt(i));
//		}
//	}
//
//
//	strTemp.Format(_T("UPDATE '%s' SET %s WHERE %s = '%s';"),
//		(LPCTSTR)Table, (LPCTSTR)strUpdate, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
//	strSql = CT2CA(strTemp);
//	int rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//
//	return rc;
//}
//
//bool CMachineFileDB::SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, double* SearchResult)
//{
//	CString strFunc(__func__);
//	char* zErrMsg = 0;
//	int rc;
//	std::string strSql;
//	CString strTemp;
//	char** results;
//	char* error;
//	char* end;
//	double convert;
//
//	strTemp.Format(_T("SELECT %s FROM %s WHERE %s = '%s';"),(LPCTSTR)SearchColumn, (LPCTSTR)SearchTable, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
//	strSql = CT2CA(strTemp);
//	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);
//
//	if (rc != SQLITE_OK || results[1] == NULL)
//	{
//		TRACE(_T("[PWR] %s SqlErr:%d (%s)\n"), strFunc, rc, strTemp);
//		return false;
//	}
//
//	convert = std::strtold(results[1], &end);
//	if (*end != '\0')
//	{
//		TRACE(_T("[PWR] %s ConvertErr:%d (%s)\n"), strFunc, rc, strTemp);
//		return false;
//	}
//
//	*SearchResult = convert;
//
//	return true;
//}
//
//bool CMachineFileDB::SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, long* SearchResult)
//{
//	CString strFunc(__func__);
//	char* zErrMsg = 0;
//	int rc;
//	std::string strSql;
//	CString strTemp;
//	char** results;
//	char* error;
//	char* end;
//	long convert;
//
//	strTemp.Format(_T("SELECT %s FROM %s WHERE %s = '%s';"), (LPCTSTR)SearchColumn, (LPCTSTR)SearchTable, (LPCTSTR)ConditionColumn, (LPCTSTR)ConditionValue);
//	strSql = CT2CA(strTemp);
//	rc = sqlite3_get_table(db, strSql.c_str(), &results, NULL, NULL, &error);
//
//	if (rc != SQLITE_OK || results[1] == NULL)
//	{
//		TRACE(_T("[PWR] %s SqlErr:%d (%s)\n"), strFunc, rc, strTemp);
//		return false;
//	}
//
//	convert = std::strtol(results[1], &end, 10);
//	if (*end != '\0')
//	{
//		TRACE(_T("[PWR] %s ConvertErr:%d (%s)\n"), strFunc, rc, strTemp);
//		return false;
//	}
//
//	*SearchResult = convert;
//
//	return true;
//}

bool CMachineFileDB::CheckDBOpen()
{
	bool result;
	sqlite3* db = OpenDB(m_FileName, &result);
    sqlite3_close(db);

	return result;
}

long CMachineFileDB::InitialAndCopyFromOld()
{
	InitialAxisParameter();
	SaveAxisParameter();

	Initial1dConfig();
	Initial1dTable(FRONT_GANTRY);
	Initial1dTable(REAR_GANTRY);
	Save1d(FRONT_GANTRY);
	Save1d(REAR_GANTRY);

	Initial2dConfig();
	Initial2dTable(FRONT_GANTRY);
	Initial2dTable(REAR_GANTRY);
	Save2d(FRONT_GANTRY);
	Save2d(REAR_GANTRY);

	InitialAlign();
	SaveAlign();
	InitialRecognitionModule();
	SaveRecogPosition();
	SaveRecogOffset();

	InitialHeadOffset();
	SaveHeadOffset();

	InitialHeightMeasurement();
	SaveHeightMeasurement();

	InitialAncConfig();
	SaveAncCofigFromXML();

	InitialAnc(FRONT_STAGE);
	InitialAnc(REAR_STAGE);
	SaveAncFromXML(FRONT_STAGE);
	SaveAncFromXML(REAR_STAGE);

	InitialPcbFix();
	SavePcbFix();

	InitialFeederReference();
	SaveFeederReference();

	return 0;
}

long CMachineFileDB::AllLoadFromDB()
{
	LoadAxisParameterFromDB();
	Load1dFromDB(FRONT_GANTRY);
	Load1dFromDB(REAR_GANTRY);

	Load2dFromDB(FRONT_GANTRY);
	Load2dFromDB(REAR_GANTRY);

	LoadAlignFromDB();
	LoadRecogPositionFromDB();

	LoadHeadOffsetFromDB();
	LoadHeightMeasurementFromDB();

	LoadAncCofigFromDB();
	LoadAncFromDB(FRONT_STAGE);
	LoadAncFromDB(REAR_STAGE);
	LoadPcbFixFromDB();
	LoadFeederReferenceFromDB();

	InitialInsertOffset4532FromDB();
	LoadInsertOffset4532FromDB();

	SetLoadComplete(true);
	return 0;
}

bool CMachineFileDB::InitialAxisParameter()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	CString strTable = m_AxisParameter.table;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' INT PRIMARY KEY NOT NULL,'%s' TEXT, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)strTable, 
		(LPCTSTR)m_AxisParameter.column_AxisNumber,
		(LPCTSTR)m_AxisParameter.column_AxisName,
		(LPCTSTR)m_AxisParameter.column_HomePosition,
		(LPCTSTR)m_AxisParameter.column_MinusLimit,
		(LPCTSTR)m_AxisParameter.column_PlusLimit,
		(LPCTSTR)m_AxisParameter.column_HomeShiftDistance);

	strSql = CT2CA(strTemp);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		rc = sqlite3_close(db);
		strErr = (LPCSTR)(LPSTR)zErrMsg;
		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
		return false;
	}
	//SqlAddColumn(db, strTable, m_AxisParameter.column_AxisName);
	//SqlAddColumn(db, strTable, m_AxisParameter.column_HomePosition);
	//SqlAddColumn(db, strTable, m_AxisParameter.column_MinusLimit);
	//SqlAddColumn(db, strTable, m_AxisParameter.column_PlusLimit);
	//SqlAddColumn(db, strTable, m_AxisParameter.column_HomeShiftDistance);

	BeginTransaction(db);
	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		SqlInsertValue(db, strTable, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
	}
	EndTransaction(db);

	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::Initial1dConfig()
{	
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	CONFIG_1D dataConfig = m_1dConfig;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE);"),
		(LPCTSTR)dataConfig.table, (LPCTSTR)dataConfig.column_Gantry, (LPCTSTR)dataConfig.column_StartY);
	strSql = CT2CA(strTemp);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		strErr = (LPCSTR)(LPSTR)zErrMsg;
		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
		return false;
	}

	SqlInsertValue(db, dataConfig.table, dataConfig.column_Gantry, ToStringGantry(FRONT_GANTRY));
	SqlInsertValue(db, dataConfig.table, dataConfig.column_Gantry, ToStringGantry(REAR_GANTRY));

	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::Initial1dTable(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	CArray<CString> strIndex;
	TABLE_1D data;

	if (Gantry == FRONT_GANTRY)
	{
		data = m_1dTableFront;
	}
	else
	{
		data = m_1dTableRear;
	}

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}


	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' INT PRIMARY KEY NOT NULL, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Index, (LPCTSTR)data.column_CompenY);
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

	for (long i = 0; i < BUFSIZE; i++)
	{
		SqlInsertValue(db, data.table, data.column_Index, ToStringInt(i));
	}

	//for (long i = 0; i < m_sqlMax; i++)
	//{
	//	strIndex.Add(ConvertCompenIndex(i));
	//}

	//SqlInsertValueMultiRow(db, data.table, data.column_Index, &strIndex);

	//strIndex.RemoveAll();
	//for (long i = m_sqlMax; i < BUFSIZE; i++)
	//{
	//	strIndex.Add(ConvertCompenIndex(i));
	//}

	//SqlInsertValueMultiRow(db, data.table, data.column_Index, &strIndex);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}


bool CMachineFileDB::Initial2dConfig()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	CONFIG_2D dataConfig = m_2dConfig;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)dataConfig.table, (LPCTSTR)dataConfig.column_Gantry, (LPCTSTR)dataConfig.column_StartX, (LPCTSTR)dataConfig.column_StartY);
	strSql = CT2CA(strTemp);

	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		strErr = (LPCSTR)(LPSTR)zErrMsg;
		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
		return false;
	}

	SqlInsertValue(db, dataConfig.table, dataConfig.column_Gantry, ToStringGantry(FRONT_GANTRY));
	SqlInsertValue(db, dataConfig.table, dataConfig.column_Gantry, ToStringGantry(REAR_GANTRY));

	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::Initial2dTable(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	CArray<CString> strIndex;
	TABLE_2D data;

	if (Gantry == FRONT_GANTRY)
	{
		data = m_2dTableFront;
	}
	else
	{
		data = m_2dTableRear;
	}

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}


	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' INT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Index, (LPCTSTR)data.column_CompenX, (LPCTSTR)data.column_CompenY);
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

	for (long i = 0; i < CAL_2D_MAXDISKCOUNT; i++)
	{
		SqlInsertValue(db, data.table, data.column_Index, ToStringInt(i));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialAlign()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	ALIGN dataConfig = m_Align;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' INT, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)dataConfig.table, 
		(LPCTSTR)dataConfig.column_Base, (LPCTSTR)dataConfig.column_AlignCamera,
		(LPCTSTR)dataConfig.column_PosiionX, (LPCTSTR)dataConfig.column_PosiionY);
	strSql = CT2CA(strTemp);
	CString column_CameraBase;
	CString column_AlignCamera;
	CString column_PosiionX;
	CString column_PosiionY;
	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
	if (rc != SQLITE_OK)
	{
		sqlite3_close(db);
		strErr = (LPCSTR)(LPSTR)zErrMsg;
		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
		return false;
	}

	SqlInsertValue(db, dataConfig.table, dataConfig.column_Base, ToStringGantry(FRONT_GANTRY));
	SqlInsertValue(db, dataConfig.table, dataConfig.column_Base, ToStringGantry(REAR_GANTRY));

	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialRecognitionModule()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	RECOGNITION data = RecognitionModule;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table,
		(LPCTSTR)data.column_Head, 
		(LPCTSTR)data.column_PositionX, (LPCTSTR)data.column_PositionY,
		(LPCTSTR)data.column_VisionOffsetX, (LPCTSTR)data.column_VisionOffsetY);
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

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
		SqlInsertValue(db, data.table, data.column_Head, ToStringHead(FRONT_GANTRY, head));
	}

	for (long head = 1; head <= MAXUSEDHEADNO; head++)
	{
		SqlInsertValue(db, data.table, data.column_Head, ToStringHead(REAR_GANTRY, head));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}



bool CMachineFileDB::InitialHeadOffset()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	HEAD_OFFSET data = m_HeadOffset;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Head, (LPCTSTR)data.column_OffsetX, (LPCTSTR)data.column_OffsetY);
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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			SqlInsertValue(db, data.table, data.column_Head, ToStringHead(Gantry, head));
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialHeightMeasurement()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	HEIGHT_MEASUREMENT data = m_HeightMeasure;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' INT);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Gantry, (LPCTSTR)data.column_OffsetX, (LPCTSTR)data.column_OffsetY, (LPCTSTR)data.column_OffsetZero);
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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		SqlInsertValue(db, data.table, data.column_Gantry, ToStringGantry(Gantry));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialAncConfig()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	ANC_CONFIG data = m_AncConfig;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Base,
		(LPCTSTR)data.column_Mark1PositionX, (LPCTSTR)data.column_Mark1PositionY, 
		(LPCTSTR)data.column_Mark2PositionX, (LPCTSTR)data.column_Mark2PositionY);

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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		SqlInsertValue(db, data.table, data.column_Base, ToStringGantry(Gantry));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialAnc(long Base)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	ANC_HOLE data;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	if (Base == FRONT_STAGE)
	{
		data = m_AncFront;
	}
	else
	{
		data = m_AncRear;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' INT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_HoleNumber,
		(LPCTSTR)data.column_HolePositionX, (LPCTSTR)data.column_HolePositionY,
		(LPCTSTR)data.column_HolePositionR, (LPCTSTR)data.column_HolePositionZ);

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

	if (Base == FRONT_STAGE)
	{
		for (long holeNo = 1; holeNo <= MAX_ANC_HOLE; holeNo++)
		{
			SqlInsertValue(db, data.table, data.column_HoleNumber, ToStringInt(holeNo));
		}
	}
	else
	{
		for (long holeNo = REAR_ANC_1ST; holeNo < REAR_ANC_1ST + MAX_ANC_HOLE; holeNo++)
		{
			SqlInsertValue(db, data.table, data.column_HoleNumber, ToStringInt(holeNo));
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialPcbFix()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	PCB_FIX data = m_PcbFix;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Base,
		(LPCTSTR)data.column_PositionX, (LPCTSTR)data.column_PositionY);

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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		SqlInsertValue(db, data.table, data.column_Base, ToStringGantry(Gantry));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialFeederReference()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	FEEDER_REFRENCE data = m_FeederReference;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' INT, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Base,
		(LPCTSTR)data.column_FeederNumber, (LPCTSTR)data.column_PositionX, (LPCTSTR)data.column_PositionY);

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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		SqlInsertValue(db, data.table, data.column_Base, ToStringGantry(Gantry));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

bool CMachineFileDB::InitialInsertOffset4532FromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	std::string strFilePath = CT2CA(m_FileName);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strTemp;
	std::string strSql;
	CString strErr;
	INSERT_OFFSET data = m_InsertOffset4532;

	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
		return false;
	}

	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE, '%s' DOUBLE);"),
		(LPCTSTR)data.table, (LPCTSTR)data.column_Head,
		(LPCTSTR)data.column_OffsetX_0, (LPCTSTR)data.column_OffsetY_0,
		(LPCTSTR)data.column_OffsetX_90, (LPCTSTR)data.column_OffsetY_90,
		(LPCTSTR)data.column_OffsetX_180, (LPCTSTR)data.column_OffsetY_180,
		(LPCTSTR)data.column_OffsetX_270, (LPCTSTR)data.column_OffsetY_270);

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

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long HeadNo = 1; HeadNo <= MAXUSEDHEADNO; HeadNo++)
		{
			SqlInsertValue(db, data.table, data.column_Head, ToStringHead(Gantry, HeadNo));
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);
	return true;
}

//bool CMachineFileDB::InitialRuntimeConveyor()
//{
//	CString strFunc(__func__);
//
//	TRACE(_T("[PWR] %s start\n"), strFunc);
//
//	char* zErrMsg = 0;
//	int rc;
//	bool result = false;
//	sqlite3* db;
//	CString strTemp;
//	std::string strSql;
//	CString strErr;
//	RUNTIME_CONVEYOR data = m_RunTimeFrontConveyor;
//
//	db = OpenCreateDB(m_FileNameRuntimeFrontConveyor, &result);
//	if (result == false)
//	{
//		TRACE(_T("[PWR] %s CreateCalFile error.)\n"), strFunc);
//		return false;
//	}
//
//	strTemp.Format(_T("CREATE TABLE IF NOT EXISTS '%s' ('%s' TEXT PRIMARY KEY NOT NULL, '%s' INT, '%s' DOUBLE, '%s' DOUBLE);"),
//		(LPCTSTR)data.table, (LPCTSTR)data.column_Base,
//		(LPCTSTR)data.column_OptionWidth,(LPCTSTR)data.column_LastWidth, (LPCTSTR)data.column_LastPusherZ);
//
//	strSql = CT2CA(strTemp);
//
//	rc = sqlite3_exec(db, strSql.c_str(), 0, 0, &zErrMsg);
//	if (rc != SQLITE_OK)
//	{
//		sqlite3_close(db);
//		strErr = (LPCSTR)(LPSTR)zErrMsg;
//		TRACE(_T("[PWR] %s %s %s\n"), strFunc, strErr, strTemp);
//		return false;
//	}
//
//	BeginTransaction(db);
//
//	SqlInsertValue(db, data.table, data.column_Base, ToStringGantry(FRONT_STAGE));	
//
//	EndTransaction(db);
//	rc = sqlite3_close(db);
//
//	TRACE(_T("[PWR] %s end\n"), strFunc);
//	return true;
//}


bool CMachineFileDB::SaveAxisParameter()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName = m_AxisParameter.table;
	CString strColNameNameAxisNumber = m_AxisParameter.column_AxisNumber;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	CArray<CString> colUpdate;
	CArray<CString> valUpdate;

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		strAxisNo = ToStringInt(axisNo);
		strAxisName = GetAxisNameByAxisIndex(axisNo);

		colUpdate.RemoveAll();
		valUpdate.RemoveAll();

		colUpdate.Add(m_AxisParameter.column_AxisName);
		valUpdate.Add(strAxisName);
				
		strUpdateVal.Format(_T("%.3f"), gGetHomePosition(axisNo));
		colUpdate.Add(m_AxisParameter.column_HomePosition);
		valUpdate.Add(strUpdateVal);

		strUpdateVal.Format(_T("%.3f"), GetLimit(axisNo).minus);
		colUpdate.Add(m_AxisParameter.column_MinusLimit);
		valUpdate.Add(strUpdateVal);

		strUpdateVal.Format(_T("%.3f"), GetLimit(axisNo).plus);
		colUpdate.Add(m_AxisParameter.column_PlusLimit);
		valUpdate.Add(strUpdateVal);


		if (GetAxisY2(FRONT_GANTRY).CompareNoCase(strAxisName) == 0)
		{
			strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->GetHomeShiftDistance(FRONT_GANTRY));
			colUpdate.Add(m_AxisParameter.column_HomeShiftDistance);
			valUpdate.Add(strUpdateVal);

		}
		else if (GetAxisY2(REAR_GANTRY).CompareNoCase(strAxisName) == 0)
		{
			strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->GetHomeShiftDistance(REAR_GANTRY));
			colUpdate.Add(m_AxisParameter.column_HomeShiftDistance);
			valUpdate.Add(strUpdateVal);
		}
		else
		{
			colUpdate.Add(m_AxisParameter.column_HomeShiftDistance);
			valUpdate.Add(_T("0.000"));
		}
		SqlupdateValueMulti(db, strTableName, &colUpdate, &valUpdate, strColNameNameAxisNumber, strAxisNo);
	}

	EndTransaction(db);

	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::Save1d(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;

	CONFIG_1D dataConfig;
	TABLE_1D dataTable;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	if (Gantry == FRONT_GANTRY)
	{
		dataConfig = m_1dConfig;
		dataTable = m_1dTableFront;
	}
	else
	{
		dataConfig = m_1dConfig;
		dataTable = m_1dTableRear;
	}

	BeginTransaction(db);

	strUpdateVal = ToStringDouble(gcPowerCalibrationData->Get1DStartCompensationData(Gantry));
	SqlupdateValue(db, dataConfig.table, dataConfig.column_StartY, strUpdateVal, dataConfig.column_Gantry, ToStringGantry(Gantry));

	for (long index = 0; index < BUFSIZE; index++)
	{
		strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->Get1DCompensationData(Gantry, index));
		SqlupdateValue(db, dataTable.table, dataTable.column_CompenY, strUpdateVal, dataTable.column_Index, ToStringInt(index));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::Save2d(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	CONFIG_2D dataConfig;
	TABLE_2D dataTable;
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	dataConfig = m_2dConfig;

	if (Gantry == FRONT_GANTRY)
	{
		dataTable = m_2dTableFront;
	}
	else
	{
		dataTable = m_2dTableRear;
	}

	BeginTransaction(db);

	strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->Get2DStartPosition(Gantry).x);
	SqlupdateValue(db, dataConfig.table, dataConfig.column_StartX, strUpdateVal, dataConfig.column_Gantry, ToStringGantry(Gantry));

	strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->Get2DStartPosition(Gantry).y);
	SqlupdateValue(db, dataConfig.table, dataConfig.column_StartY, strUpdateVal, dataConfig.column_Gantry, ToStringGantry(Gantry));


	for (long index = 0; index < CAL_2D_MAXDISKCOUNT; index++)
	{
		strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->Get2DCompensationData(Gantry, index).x);
		SqlupdateValue(db, dataTable.table, dataTable.column_CompenX, strUpdateVal, dataTable.column_Index, ToStringInt(index));

		strUpdateVal.Format(_T("%.3f"), gcPowerCalibrationData->Get2DCompensationData(Gantry, index).y);
		SqlupdateValue(db, dataTable.table, dataTable.column_CompenY, strUpdateVal, dataTable.column_Index, ToStringInt(index));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveAlign()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	ALIGN dataConfig = m_Align;
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		strUpdateVal = ToStringInt(gcPowerCalibrationData->GetAlignCamera(Gantry));
		SqlupdateValue(db, dataConfig.table, dataConfig.column_AlignCamera, strUpdateVal, dataConfig.column_Base, ToStringGantry(Gantry));

		strUpdateVal = ToStringDouble(gcPowerCalibrationData->GetCameraAlignPosition(Gantry).x);
		SqlupdateValue(db, dataConfig.table, dataConfig.column_PosiionX, strUpdateVal, dataConfig.column_Base, ToStringGantry(Gantry));

		strUpdateVal = ToStringDouble(gcPowerCalibrationData->GetCameraAlignPosition(Gantry).y);
		SqlupdateValue(db, dataConfig.table, dataConfig.column_PosiionY, strUpdateVal, dataConfig.column_Base, ToStringGantry(Gantry));
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveRecogPosition()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	//RECOGNITION data;
	Point_XY posXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			posXY = gcPowerCalibrationData->GetCameraRecognitionPosition(Gantry, head);
			UpdateRecognitionPosition(db, Gantry, head, posXY);
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveRecogOffset()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	//RECOGNITION data;
	Point_XY posXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			posXY = gcPowerCalibrationData->GetCameraRecognitionOffset(Gantry, head);
			UpdateRecognitionOffset(db, Gantry, head, posXY);
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveHeadOffset()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	HEAD_OFFSET data;
	Point_XY posXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
	{
		posXY = gcPowerCalibrationData->GetHeadOffset(FRONT_GANTRY, head);
		UpdateHeadOffset(db, FRONT_GANTRY, head, posXY);

		posXY = gcPowerCalibrationData->GetHeadOffset(REAR_GANTRY, head);
		UpdateHeadOffset(db, REAR_GANTRY, head, posXY);
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveHeightMeasurement()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	HEAD_OFFSET data;
	Point_XY posXY;
	long zero;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	posXY = gcPowerCalibrationData->GetHMOffset(FRONT_GANTRY);
	zero = gcPowerCalibrationData->GetHMZero(FRONT_GANTRY);
	UpdateHeightMeasurement(db, FRONT_GANTRY, posXY, zero);

	posXY = gcPowerCalibrationData->GetHMOffset(REAR_GANTRY);
	zero = gcPowerCalibrationData->GetHMZero(REAR_GANTRY);
	UpdateHeightMeasurement(db, REAR_GANTRY, posXY, zero);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveAncCofigFromXML()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	ANC_MARK_STRUCT MarkData;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long base = 0; base <= REAR_STAGE; base++)
	{
		MarkData = gCMachineConfig->GetCalANC(base).Mark.at(0);
		UpdateAncMark(db, base, 1, MarkData.pt);
		MarkData = gCMachineConfig->GetCalANC(base).Mark.at(1);
		UpdateAncMark(db, base, 2, MarkData.pt);
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveAncFromXML(long Base)
{
	// ANC는 XML 한번 거치는걸로.
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	ANC_HOLE_STRUCT holeData;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	if (Base == FRONT_STAGE)
	{
		for (long holeNo = 1; holeNo <= MAX_ANC_HOLE; holeNo++)
		{
			if (gCMachineConfig->GetCalANCHole(Base, holeNo, &holeData) == true)
			{
				UpdateAncHole(db, Base, holeNo, holeData.pt);
			}
		}
	}
	else
	{
		for (long holeNo = REAR_ANC_1ST; holeNo < REAR_ANC_1ST + MAX_ANC_HOLE; holeNo++)
		{
			if (gCMachineConfig->GetCalANCHole(Base, holeNo, &holeData) == true)
			{
				UpdateAncHole(db, Base, holeNo, holeData.pt);
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SavePcbFix()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	Point_XY posXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	posXY = gcPowerCalibrationData->GetPcbFixPosition(FRONT_STAGE);
	UpdatePcbFix(db, FRONT_STAGE, posXY);

	posXY = gcPowerCalibrationData->GetPcbFixPosition(FRONT_STAGE);
	UpdatePcbFix(db, REAR_STAGE, posXY);

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::SaveFeederReference()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	Point_XY posXY;
	long feederRef;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);


	for (long base = 0; base <= REAR_STAGE; base++)
	{
		feederRef = gcPowerCalibrationData->GetReferenceFeederNo(base);
		posXY = gcPowerCalibrationData->GetReferenceFeederPosition(base);
		UpdateFeederReference(db, base, feederRef, posXY);
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}
bool CMachineFileDB::SaveInsertOffset4532FromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;
	INSERT_OFFSET dataConfig = m_InsertOffset4532;
	Point_XY offsetXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			offsetXY = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 0);
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_0, ToStringDouble(offsetXY.x), dataConfig.column_Head, ToStringHead(Gantry, head));
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_0, ToStringDouble(offsetXY.y), dataConfig.column_Head, ToStringHead(Gantry, head));

			offsetXY = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 1);
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_90, ToStringDouble(offsetXY.x), dataConfig.column_Head, ToStringHead(Gantry, head));
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_90, ToStringDouble(offsetXY.y), dataConfig.column_Head, ToStringHead(Gantry, head));

			offsetXY = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 2);
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_180, ToStringDouble(offsetXY.x), dataConfig.column_Head, ToStringHead(Gantry, head));
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_180, ToStringDouble(offsetXY.y), dataConfig.column_Head, ToStringHead(Gantry, head));

			offsetXY = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 3);
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_270, ToStringDouble(offsetXY.x), dataConfig.column_Head, ToStringHead(Gantry, head));
			SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_270, ToStringDouble(offsetXY.y), dataConfig.column_Head, ToStringHead(Gantry, head));
		}
	}


	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}
//bool CMachineFileDB::SaveRuntimeConveyor()
//{
//	CString strFunc(__func__);
//
//	TRACE(_T("[PWR] %s start\n"), strFunc);
//
//	char* zErrMsg = 0;
//	int rc;
//	bool result = false;
//	sqlite3* db;
//	CString strUpdateVal;
//	std::string strSql;
//	CString strErr;
//	CString strAxisName;
//	CString strAxisNo;
//	Point_XY posXY;
//	long feederRef;
//
//	db = OpenCreateDB(m_FileNameRuntimeFrontConveyor , &result);
//	if (result == false)
//	{
//		return false;
//	}
//
//	BeginTransaction(db);
//
//	feederRef = gcPowerConveyorData->GetWidth(base);
//	posXY = gcPowerCalibrationData->GetReferenceFeederPosition(base);
//	UpdateFeederReference(db, base, feederRef, posXY);	
//
//	EndTransaction(db);
//	rc = sqlite3_close(db);
//
//	TRACE(_T("[PWR] %s end\n"), strFunc);
//
//	return true;
//}

bool CMachineFileDB::LoadAxisParameterFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strReadTable = m_AxisParameter.table;
	CString strReadColumn;
	CString strColumnAxisNumber = m_AxisParameter.column_AxisNumber;
	CString strTemp;

	size_t convertSize = 0;
	std::deque<double> homepos;
	std::deque<double> minuslimit;
	std::deque<double> pluslimit;
	std::deque<double> homeshift;
	double readResult;
	bool searchSuccess = false;
	CString strSearchValue;

	Limit swlimit;
	long count;
	double defaultValue = 0.0;

	count = GetDBRowCount();

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long axisNo = count; axisNo < MAXAXISNO; axisNo++)
	{
		SqlInsertValue(db, strReadTable, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
	}

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		SqlupdateValue(db, strReadTable, m_AxisParameter.column_AxisName, GetAxisNameByAxisIndex(axisNo), m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
	}

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		if (SqlSearchValue(db, strReadTable, m_AxisParameter.column_HomePosition, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo), &readResult) == true)
		{
			homepos.push_back(readResult);
		}
		else
		{
			homepos.push_back(defaultValue);
			SqlupdateValue(db, strReadTable, m_AxisParameter.column_HomePosition, ToStringDouble(defaultValue), m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
		}

		if (SqlSearchValue(db, strReadTable, m_AxisParameter.column_MinusLimit, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo), &readResult) == true)
		{
			minuslimit.push_back(readResult);
		}
		else
		{
			minuslimit.push_back(defaultValue);
			SqlupdateValue(db, strReadTable, m_AxisParameter.column_MinusLimit, ToStringDouble(defaultValue), m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
		}

		if (SqlSearchValue(db, strReadTable, m_AxisParameter.column_PlusLimit, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo), &readResult) == true)
		{
			pluslimit.push_back(readResult);
		}
		else
		{
			pluslimit.push_back(defaultValue);
			SqlupdateValue(db, strReadTable, m_AxisParameter.column_PlusLimit, ToStringDouble(defaultValue), m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
		}

		if (SqlSearchValue(db, strReadTable, m_AxisParameter.column_HomeShiftDistance, m_AxisParameter.column_AxisNumber, ToStringInt(axisNo), &readResult) == true)
		{
			homeshift.push_back(readResult);
		}
		else
		{
			homeshift.push_back(defaultValue);
			SqlupdateValue(db, strReadTable, m_AxisParameter.column_HomeShiftDistance, ToStringDouble(defaultValue), m_AxisParameter.column_AxisNumber, ToStringInt(axisNo));
		}
	}

	if (homepos.size() != MAXAXISNO || minuslimit.size() != MAXAXISNO || pluslimit.size() != MAXAXISNO || homeshift.size() != MAXAXISNO)
	{
		TRACE(_T("[PWR] %s fail. (%d,%d,%d,%d)\n"), strFunc, homepos.size(), minuslimit.size(), pluslimit.size(), homeshift.size());
		return false;
	}

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		gcPowerCalibrationData->SetHomePosition(axisNo, homepos.at(axisNo));
	}

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		swlimit.minus = minuslimit.at(axisNo);
		swlimit.plus = pluslimit.at(axisNo);
		gcPowerCalibrationData->SetLimit(axisNo, swlimit);
	}

	for (long axisNo = 0; axisNo < MAXAXISNO; axisNo++)
	{
		if (GetAxisY2(FRONT_GANTRY).CompareNoCase(GetAxisNameByAxisIndex(axisNo)) == 0)
		{
			gcPowerCalibrationData->SetHomeShiftDistance(FRONT_GANTRY, homeshift.at(axisNo));
		}
		else if (GetAxisY2(REAR_GANTRY).CompareNoCase(GetAxisNameByAxisIndex(axisNo)) == 0)
		{
			gcPowerCalibrationData->SetHomeShiftDistance(REAR_GANTRY, homeshift.at(axisNo));
		}
	}

	EndTransaction(db);
	CloseDB(db);

	return true;
}

bool CMachineFileDB::Load1dFromDB(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	CString strUpdateVal;
	std::string strSql;
	CString strErr;
	CString strAxisName;
	CString strAxisNo;
	CString strTableName;

	CONFIG_1D dataConfig;
	TABLE_1D dataTable;

	double readResult;

	std::deque<double> compendata;


	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	if (Gantry == FRONT_GANTRY)
	{
		dataConfig = m_1dConfig;
		dataTable = m_1dTableFront;
	}
	else
	{
		dataConfig = m_1dConfig;
		dataTable = m_1dTableRear;
	}

	BeginTransaction(db);

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_StartY, dataConfig.column_Gantry, ToStringGantry(Gantry), &readResult) == true)
	{
		gcPowerCalibrationData->Set1DStartCompensationData(Gantry, readResult);
	}

	for (long index = 0; index < BUFSIZE; index++)
	{
		if (SqlSearchValue(db, dataTable.table, dataTable.column_CompenY, dataTable.column_Index, ToStringInt(index), &readResult) == true)
		{
			compendata.push_back(readResult);
		}
	}

	if (compendata.size() == BUFSIZE)
	{
		for (long index = 0; index < BUFSIZE; index++)
		{
			gcPowerCalibrationData->Set1DCompensationData(Gantry, index, compendata.at(index));
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::Load2dFromDB(long Gantry)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;

	bool result = false;
	sqlite3* db;

	CONFIG_2D dataConfig;
	TABLE_2D dataTable;

	std::deque<Point_XYRE> compendata;
	Point_XY ptXY;
	Point_XYRE ptXYRE;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	if (Gantry == FRONT_GANTRY)
	{
		dataConfig = m_2dConfig;
		dataTable = m_2dTableFront;
	}
	else
	{
		dataConfig = m_2dConfig;
		dataTable = m_2dTableRear;
	}

	BeginTransaction(db);

	if (SqlSearchValue(db, dataConfig.table, dataConfig.column_StartX, dataConfig.column_Gantry, ToStringGantry(Gantry), &ptXY.x) == true)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_StartY, dataConfig.column_Gantry, ToStringGantry(Gantry), &ptXY.y) == true)
		{
			gcPowerCalibrationData->Set2DStartPosition(Gantry, ptXY);
		}
	}

	for (long index = 0; index < CAL_2D_MAXDISKCOUNT; index++)
	{
		if (SqlSearchValue(db, dataTable.table, dataTable.column_CompenX, dataTable.column_Index, ToStringInt(index), &ptXYRE.x) == true)
		{
			if (SqlSearchValue(db, dataTable.table, dataTable.column_CompenY, dataTable.column_Index, ToStringInt(index), &ptXYRE.y) == true)
			{
				gcPowerCalibrationData->Set2DCompensationData(Gantry, index, ptXYRE);
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadAlignFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	ALIGN dataConfig = m_Align;
	Point_XY ptXY;
	long camera = 0;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long base = 0; base <= REAR_STAGE; base++)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PosiionX, dataConfig.column_Base, ToStringGantry(base), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PosiionY, dataConfig.column_Base, ToStringGantry(base), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetCameraAlignPosition(base, ptXY);
			}
		}

		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_AlignCamera, dataConfig.column_Base, ToStringGantry(base), &camera) == true)
		{
			gcPowerCalibrationData->SetAlignCamera(base, camera);
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadRecogPositionFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	RECOGNITION dataConfig = RecognitionModule;
	Point_XY ptXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionX, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true)
			{
				if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionY, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
				{
					gcPowerCalibrationData->SetCameraRecognitionPosition(Gantry, head, ptXY);
				}
			}	
		}
	}

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_VisionOffsetX, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true)
			{
				if (SqlSearchValue(db, dataConfig.table, dataConfig.column_VisionOffsetY, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
				{
					gcPowerCalibrationData->SetCameraRecognitionOffset(Gantry, head, ptXY);
				}
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadHeadOffsetFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	HEAD_OFFSET dataConfig = m_HeadOffset;
	Point_XY ptXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true)
			{
				if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
				{
					gcPowerCalibrationData->SetHeadOffset(Gantry, head, ptXY);
				}
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadHeightMeasurementFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	HEIGHT_MEASUREMENT dataConfig = m_HeightMeasure;
	Point_XY ptXY;
	long zeroset;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX, dataConfig.column_Gantry, ToStringGantry(Gantry), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY, dataConfig.column_Gantry, ToStringGantry(Gantry), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetHMOffset(Gantry, ptXY);
			}
		}

		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetZero, dataConfig.column_Gantry, ToStringGantry(Gantry), &zeroset) == true)
		{
			gcPowerCalibrationData->SetHMZero(Gantry, zeroset);
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadAncCofigFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	ANC_CONFIG dataConfig = m_AncConfig;
	Point_XY ptXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_Mark1PositionX, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_Mark1PositionY, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetANCMarkPosition(Gantry, 0, ptXY);
			}
		}

		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_Mark2PositionX, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_Mark2PositionY, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetANCMarkPosition(Gantry, 1, ptXY);
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadAncFromDB(long Base)
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	ANC_HOLE dataConfig;
	Point_XYRZ ptXYRZ;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	if (Base == FRONT_STAGE)
	{
		dataConfig = m_AncFront;
		for (long holeNo = 1; holeNo <= MAX_ANC_HOLE; holeNo++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionX, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.x) == true)
			{
				if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionY, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.y) == true)
				{
					if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionR, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.r) == true)
					{
						if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionZ, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.z) == true)
						{
							gcPowerCalibrationData->SetANCHoleRealPosition(holeNo, ptXYRZ);
						}
					}
				}
			}
		}
	}
	else
	{
		dataConfig = m_AncRear;

		for (long holeNo = REAR_ANC_1ST; holeNo < REAR_ANC_1ST + MAX_ANC_HOLE; holeNo++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionX, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.x) == true)
			{
				if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionY, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.y) == true)
				{
					if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionR, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.r) == true)
					{
						if (SqlSearchValue(db, dataConfig.table, dataConfig.column_HolePositionZ, dataConfig.column_HoleNumber, ToStringInt(holeNo), &ptXYRZ.z) == true)
						{
							gcPowerCalibrationData->SetANCHoleRealPosition(holeNo, ptXYRZ);
						}
					}
				}
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadPcbFixFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	PCB_FIX dataConfig = m_PcbFix;
	Point_XY ptXY;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionX, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionY, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetPcbFixPosition(Gantry, ptXY);
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadFeederReferenceFromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	FEEDER_REFRENCE dataConfig = m_FeederReference;
	Point_XY ptXY;
	long feederNo;
	double pitch = 0.0;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	if (CheckColumnExist(db, dataConfig.table, dataConfig.column_Pitch) == false)
	{
		SqlAddColumn(db, dataConfig.table, dataConfig.column_Pitch, _T("DOUBLE"));

		SqlupdateValue(db, dataConfig.table, dataConfig.column_Pitch, ToStringDouble(13.000), dataConfig.column_Base, ToStringGantry(FRONT_GANTRY));
		SqlupdateValue(db, dataConfig.table, dataConfig.column_Pitch, ToStringDouble(13.000), dataConfig.column_Base, ToStringGantry(REAR_GANTRY));
	}

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionX, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.x) == true)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_PositionY, dataConfig.column_Base, ToStringGantry(Gantry), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetReferenceFeederPosition(Gantry, ptXY);
			}
		}

		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_FeederNumber, dataConfig.column_Base, ToStringGantry(Gantry), &feederNo) == true)
		{
			gcPowerCalibrationData->SetReferenceFeederNo(Gantry, feederNo);
		}

		if (SqlSearchValue(db, dataConfig.table, dataConfig.column_Pitch, dataConfig.column_Base, ToStringGantry(Gantry), &pitch) == true)
		{
			gcPowerCalibrationData->SetFeederPitch(Gantry, pitch);
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

bool CMachineFileDB::LoadInsertOffset4532FromDB()
{
	CString strFunc(__func__);

	TRACE(_T("[PWR] %s start\n"), strFunc);

	char* zErrMsg = 0;
	int rc;
	bool result = false;
	sqlite3* db;
	INSERT_OFFSET dataConfig = m_InsertOffset4532;
	Point_XY ptXY;
	Point_XY offsetInit;
	bool readOK = false;

	db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		return false;
	}

	BeginTransaction(db);

	for (long Gantry = 0; Gantry <= REAR_GANTRY; Gantry++)
	{
		for (long head = TBL_HEAD1; head <= MAXUSEDHEADNO; head++)
		{
			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX_0, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true
				&& SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY_0, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetInsertOffset4532(Gantry, head, 0, ptXY);
			}
			else
			{
				offsetInit = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 0);
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_0, ToStringDouble(offsetInit.x), dataConfig.column_Head, ToStringHead(Gantry, head));
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_0, ToStringDouble(offsetInit.y), dataConfig.column_Head, ToStringHead(Gantry, head));
			}

			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX_90, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true
				&& SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY_90, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetInsertOffset4532(Gantry, head, 1, ptXY);
			}
			else
			{
				offsetInit = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 1);
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_90, ToStringDouble(offsetInit.x), dataConfig.column_Head, ToStringHead(Gantry, head));
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_90, ToStringDouble(offsetInit.y), dataConfig.column_Head, ToStringHead(Gantry, head));
			}


			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX_180, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true
				&& SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY_180, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetInsertOffset4532(Gantry, head, 2, ptXY);
			}
			else
			{
				offsetInit = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 2);
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_180, ToStringDouble(offsetInit.x), dataConfig.column_Head, ToStringHead(Gantry, head));
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_180, ToStringDouble(offsetInit.y), dataConfig.column_Head, ToStringHead(Gantry, head));
			}


			if (SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetX_270, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.x) == true
				&& SqlSearchValue(db, dataConfig.table, dataConfig.column_OffsetY_270, dataConfig.column_Head, ToStringHead(Gantry, head), &ptXY.y) == true)
			{
				gcPowerCalibrationData->SetInsertOffset4532(Gantry, head, 3, ptXY);
			}
			else
			{
				offsetInit = gcPowerCalibrationData->GetInsertOffset4532(Gantry, head, 3);
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetX_270, ToStringDouble(offsetInit.x), dataConfig.column_Head, ToStringHead(Gantry, head));
				SqlupdateValue(db, dataConfig.table, dataConfig.column_OffsetY_270, ToStringDouble(offsetInit.y), dataConfig.column_Head, ToStringHead(Gantry, head));
			}
		}
	}

	EndTransaction(db);
	rc = sqlite3_close(db);

	TRACE(_T("[PWR] %s end\n"), strFunc);

	return true;
}

long CMachineFileDB::UpdateRecognitionPosition(sqlite3* db, long Gantry, long Head, Point_XY Pos)
{	
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	RECOGNITION data = RecognitionModule;

	colUpdate.Add(data.column_PositionX);
	valUpdate.Add(ToStringDouble(Pos.x));

	colUpdate.Add(data.column_PositionY);
	valUpdate.Add(ToStringDouble(Pos.y));	

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Head, ToStringHead(Gantry, Head));
}

long CMachineFileDB::UpdateRecognitionOffset(sqlite3* db, long Gantry, long Head, Point_XY offset)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	RECOGNITION data = RecognitionModule;

	colUpdate.Add(data.column_VisionOffsetX);
	valUpdate.Add(ToStringDouble(offset.x));

	colUpdate.Add(data.column_VisionOffsetY);
	valUpdate.Add(ToStringDouble(offset.y));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Head, ToStringHead(Gantry, Head));
}

long CMachineFileDB::UpdateHeadOffset(sqlite3* db, long Gantry, long Head, Point_XY offset)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	HEAD_OFFSET data = m_HeadOffset;

	colUpdate.Add(data.column_OffsetX);
	valUpdate.Add(ToStringDouble(offset.x));

	colUpdate.Add(data.column_OffsetY);
	valUpdate.Add(ToStringDouble(offset.y));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Head, ToStringHead(Gantry, Head));
}

long CMachineFileDB::UpdateHeadRadius(long Gantry, long Head, double Radius)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	HEAD_OFFSET data = m_HeadOffset;
	bool result = false;
	long Err = 0;

	sqlite3* db = OpenDB(m_FileName, &result);
	if (result == false)
	{
		sqlite3_close(db);
		return false;
	}

	if (CheckColumnExist(db, data.table, data.column_Radius) == false)
	{
		SqlAddColumn(db, data.table, data.column_Radius, _T("DOUBLE"));
	}

	colUpdate.Add(data.column_Radius);
	valUpdate.Add(ToStringDouble(Radius));

	Err = SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Head, ToStringHead(Gantry, Head));

	sqlite3_close(db);

	return Err;
}

long CMachineFileDB::UpdateHeightMeasurement(sqlite3* db, long Gantry, Point_XY offsetXY, long offsetZero)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	HEIGHT_MEASUREMENT data = m_HeightMeasure;

	colUpdate.Add(data.column_OffsetX);
	valUpdate.Add(ToStringDouble(offsetXY.x));

	colUpdate.Add(data.column_OffsetY);
	valUpdate.Add(ToStringDouble(offsetXY.y));

	colUpdate.Add(data.column_OffsetZero);
	valUpdate.Add(ToStringDouble(offsetZero));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Gantry, ToStringGantry(Gantry));
}

long CMachineFileDB::UpdateAncMark(sqlite3* db, long Base, long MarkNo, Point_XY positionXY)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	ANC_CONFIG data = m_AncConfig;

	if (MarkNo == 1)
	{
		colUpdate.Add(data.column_Mark1PositionX);
		valUpdate.Add(ToStringDouble(positionXY.x));

		colUpdate.Add(data.column_Mark1PositionY);
		valUpdate.Add(ToStringDouble(positionXY.y));
	}
	else if (MarkNo == 2)
	{
		colUpdate.Add(data.column_Mark2PositionX);
		valUpdate.Add(ToStringDouble(positionXY.x));

		colUpdate.Add(data.column_Mark2PositionY);
		valUpdate.Add(ToStringDouble(positionXY.y));
	}
	else
	{
		return 0;
	}

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Base, ToStringGantry(Base));
}

long CMachineFileDB::UpdateAncHole(sqlite3* db, long Base, long HoleNo, Point_XYRZ positionXYRZ)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	ANC_HOLE data;

	if (Base == FRONT_STAGE)
	{
		data = m_AncFront;
	}
	else
	{
		data = m_AncRear;
	}

	colUpdate.Add(data.column_HolePositionX);
	valUpdate.Add(ToStringDouble(positionXYRZ.x));

	colUpdate.Add(data.column_HolePositionY);
	valUpdate.Add(ToStringDouble(positionXYRZ.y));

	colUpdate.Add(data.column_HolePositionR);
	valUpdate.Add(ToStringDouble(positionXYRZ.r));

	colUpdate.Add(data.column_HolePositionZ);
	valUpdate.Add(ToStringDouble(positionXYRZ.z));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_HoleNumber, ToStringInt(HoleNo));
}

long CMachineFileDB::UpdatePcbFix(sqlite3* db, long Base, Point_XY positionXY)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	PCB_FIX data = m_PcbFix;

	colUpdate.Add(data.column_PositionX);
	valUpdate.Add(ToStringDouble(positionXY.x));

	colUpdate.Add(data.column_PositionY);
	valUpdate.Add(ToStringDouble(positionXY.y));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Base, ToStringGantry(Base));

	return 0;
}

long CMachineFileDB::UpdateFeederReference(sqlite3* db, long Base, long FeederNo, Point_XY positionXY)
{
	CArray<CString> colUpdate;
	CArray<CString> valUpdate;
	FEEDER_REFRENCE data = m_FeederReference;

	colUpdate.Add(data.column_PositionX);
	valUpdate.Add(ToStringDouble(positionXY.x));

	colUpdate.Add(data.column_PositionY);
	valUpdate.Add(ToStringDouble(positionXY.y));

	colUpdate.Add(data.column_FeederNumber);
	valUpdate.Add(ToStringInt(FeederNo));

	return SqlupdateValueMulti(db, data.table, &colUpdate, &valUpdate, data.column_Base, ToStringGantry(Base));

	return 0;
}


void CMachineFileDB::InsertAutoDeleteConfig(CString DeletePath, long day)
{
	bool openReuslt = false;
	CString fileName = _T("C:\\Power\\i6.0\\HMI\\DataBase\\DataBase.db");
	std::string strSql;
	char* zErrMsg = 0;
	int rc;
	sqlite3* db;
	bool serachResult;
	long deleteDay;

	CSqlite3File* pConfigdDB = new CSqlite3File();

	db = OpenCreateDB(fileName, &openReuslt);
	if (openReuslt == false)
	{
		delete pConfigdDB;
		return;
	}


	serachResult = SqlSearchValue(db, _T("AutoDeleteTable"), _T("Period"), _T("FilePath"), DeletePath, &deleteDay);

	if (serachResult == true)
	{
		TRACE(_T("[PWR] InsertAutoDeleteConfig Already Exist (%s,%d)\n"), DeletePath, day);
	}
	else
	{
		CString strDay;
		strDay.Format(_T("%d"), day);

		SqlInsertValue(db, _T("AutoDeleteTable"), _T("FilePath"), DeletePath);
		SqlupdateValue(db, _T("AutoDeleteTable"), _T("Period"), strDay, _T("FilePath"), DeletePath);
	}

	rc = sqlite3_close(db);

	delete pConfigdDB;
}

long CMachineFileDB::GetDBRowCount()
{
	sqlite3* db;
	CString strReadTable = m_AxisParameter.table;
	long count;
	bool result;

	TRACE(_T("[PWR] GetDBRowCount Lock Start\n"));
	Lock();
	db = OpenCreateDB(m_FileName, &result);
	if (result == false)
	{
		Unlock();

		return false;
	}

	BeginTransaction(db);

	GetRowCount(db, strReadTable, m_AxisParameter.column_AxisNumber, &count);

	EndTransaction(db);
	sqlite3_close(db);
	Unlock();
	return count;
}
