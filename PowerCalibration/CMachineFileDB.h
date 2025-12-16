#pragma once

#include "GlobalDefine.h"
#include "CSqlite3File.h"

using namespace std;
#pragma comment(lib, "sqlite3.lib")
#include "sqlite3.h"

typedef struct {

	CString table;
	CString column_AxisNumber;
	CString column_AxisName;
	CString column_HomePosition;
	CString column_MinusLimit;
	CString column_PlusLimit;
	CString column_HomeShiftDistance;

}AXIS_PARAMETER;

typedef struct {

	CString table;
	CString column_Gantry;
	CString column_StartY;

}CONFIG_1D;

typedef struct {

	CString table;
	CString column_Index;
	CString column_CompenY;

}TABLE_1D;

typedef struct {

	CString table;
	CString column_Gantry;
	CString column_StartX;
	CString column_StartY;

}CONFIG_2D;

typedef struct {

	CString table;
	CString column_Index;
	CString column_CompenX;
	CString column_CompenY;

}TABLE_2D;

typedef struct {

	CString table;
	CString column_Base;
	CString column_AlignCamera;
	CString column_PosiionX;
	CString column_PosiionY;

}ALIGN;

typedef struct {

	CString table;
	CString column_Head;
	CString column_PositionX;
	CString column_PositionY;
	CString column_VisionOffsetX;
	CString column_VisionOffsetY;

}RECOGNITION;

typedef struct {

	CString table;
	CString column_Head;
	CString column_OffsetX;
	CString column_OffsetY;
	CString column_Radius;

}HEAD_OFFSET;

typedef struct {

	CString table;
	CString column_Gantry;
	CString column_OffsetX;
	CString column_OffsetY;
	CString column_OffsetZero;

}HEIGHT_MEASUREMENT;

typedef struct {

	CString table;
	CString column_Base;
	CString column_Mark1PositionX;
	CString column_Mark1PositionY;
	CString column_Mark2PositionX;
	CString column_Mark2PositionY;

}ANC_CONFIG;

typedef struct {

	CString table;
	CString column_HoleNumber;
	CString column_HolePositionX;
	CString column_HolePositionY;
	CString column_HolePositionR;
	CString column_HolePositionZ;

}ANC_HOLE;

typedef struct {

	CString table;
	CString column_Base;
	CString column_PositionX;
	CString column_PositionY;

}PCB_FIX;

typedef struct {

	CString table;
	CString column_Base;
	CString column_FeederNumber;
	CString column_PositionX;
	CString column_PositionY;
	CString column_Pitch;
}FEEDER_REFRENCE;

typedef struct {

	CString table;
	CString column_Head;
	CString column_OffsetX_0;
	CString column_OffsetY_0;
	CString column_OffsetX_90;
	CString column_OffsetY_90;
	CString column_OffsetX_180;
	CString column_OffsetY_180;
	CString column_OffsetX_270;
	CString column_OffsetY_270;

}INSERT_OFFSET;

class CMachineFileDB : public CSqlite3File
{
public:
	CMachineFileDB();
	~CMachineFileDB();

	//CString ToStringInt(long index);
	//CString ToStringDouble(double value);
	//CString ToStringGantry(long Gantry);
	//CString ToStringHead(long Gantry, long HeadNo);

	//sqlite3* OpenCreateCalibration(bool* result);
	//sqlite3* OpenCreateFile(CString fileName, bool* result);
	//sqlite3* OpenCalibration(bool* result);
	//void CloseDB(sqlite3* db);
	//long BeginTransaction(sqlite3* db);
	//long EndTransaction(sqlite3* db);
	//bool CheckTableExist(sqlite3* db, CString TableName);
	//CString SqlCreateTable(CString TableName, CArray<CString>* ColumnName);
	//long SqlAddColumn(sqlite3* db, CString TableName, CString ColumnName);
	//long SqlInsertValue(sqlite3* db, CString TableName, CString ColumnName, CString Value);
	//long SqlInsertValueMultiRow(sqlite3* db, CString TableName, CString ColumnName, CArray<CString>* Value);
	//long SqlupdateValue(sqlite3* db, CString Table, CString UpdateColumn, CString UpdateValue, CString ConditionColumn, CString ConditionValue);
	//long SqlupdateValueMulti(sqlite3* db, CString Table, CArray<CString>* UpdateColumn, CArray<CString>* UpdateValue, CString ConditionColumn, CString ConditionValue);
	////long SqlupdateValueMultiRow(sqlite3* db, CString Table, CString UpdateColumn, CArray<CString>* UpdateValue, CString ConditionColumn, CString ConditionValue);
	//bool SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, double* SearchResult);
	//bool SqlSearchValue(sqlite3* db, CString SearchTable, CString SearchColumn, CString ConditionColumn, CString ConditionValue, long* SearchResult);

	bool CheckDBOpen();
	long InitialAndCopyFromOld();
	long AllLoadFromDB();

	bool InitialAxisParameter();
	bool Initial1dConfig();
	bool Initial1dTable(long Gantry);
	bool Initial2dConfig();
	bool Initial2dTable(long Gantry);
	bool InitialAlign();
	bool InitialRecognitionModule();
	bool InitialHeadOffset();
	bool InitialHeightMeasurement();
	bool InitialAncConfig();
	bool InitialAnc(long Base);
	bool InitialPcbFix();
	bool InitialFeederReference();
	//bool InitialRuntimeConveyor();
	bool InitialInsertOffset4532FromDB();

	bool SaveAxisParameter();
	bool Save1d(long Gantry);
	bool Save2d(long Gantry);
	bool SaveAlign();
	bool SaveRecogPosition();
	bool SaveRecogOffset();
	bool SaveHeadOffset();
	bool SaveHeightMeasurement();
	bool SaveAncCofigFromXML();
	bool SaveAncFromXML(long Base);
	bool SavePcbFix();
	bool SaveFeederReference();
	bool SaveInsertOffset4532FromDB();
	//bool SaveRuntimeConveyor();

	bool LoadAxisParameterFromDB();
	bool Load1dFromDB(long Gantry);
	bool Load2dFromDB(long Gantry);
	bool LoadAlignFromDB();
	bool LoadRecogPositionFromDB();
	bool LoadHeadOffsetFromDB();
	bool LoadHeightMeasurementFromDB();
	bool LoadAncCofigFromDB();
	bool LoadAncFromDB(long Base);
	bool LoadPcbFixFromDB();
	bool LoadFeederReferenceFromDB();

	bool LoadInsertOffset4532FromDB();

	long UpdateRecognitionPosition(sqlite3* db, long Gantry, long Head, Point_XY Pos);
	long UpdateRecognitionOffset(sqlite3* db, long Gantry, long Head, Point_XY offset);
	long UpdateHeadOffset(sqlite3* db, long Gantry, long Head, Point_XY offset);
	long UpdateHeadRadius(long Gantry, long Head, double Radius);
	long UpdateHeightMeasurement(sqlite3* db, long Gantry, Point_XY offsetXY, long offsetZero);
	long UpdateAncMark(sqlite3* db, long Base, long MarkNo, Point_XY positionXY);
	long UpdateAncHole(sqlite3* db, long Base, long HoleNo, Point_XYRZ positionXYRZ);
	long UpdatePcbFix(sqlite3* db, long Base, Point_XY positionXY);
	long UpdateFeederReference(sqlite3* db, long Base, long FeederNo, Point_XY positionXY);
	void InsertAutoDeleteConfig(CString DeletePath, long day);

	long GetDBRowCount();

private:
	CString m_FileName;
	//CString m_FileNameRuntimeFrontConveyor;
	CString m_ColumnDate;

	AXIS_PARAMETER m_AxisParameter;

	CONFIG_1D m_1dConfig;
	TABLE_1D m_1dTableFront;
	TABLE_1D m_1dTableRear;

	CONFIG_2D m_2dConfig;
	TABLE_2D m_2dTableFront;
	TABLE_2D m_2dTableRear;

	ALIGN m_Align;
	RECOGNITION RecognitionModule;
	HEAD_OFFSET m_HeadOffset;
	HEIGHT_MEASUREMENT m_HeightMeasure;

	ANC_CONFIG m_AncConfig;
	ANC_HOLE m_AncFront;
	ANC_HOLE m_AncRear;

	PCB_FIX m_PcbFix;
	FEEDER_REFRENCE m_FeederReference;
	INSERT_OFFSET m_InsertOffset4532;

	//RUNTIME_CONVEYOR m_RunTimeFrontConveyor;
	long m_sqlMax;
};


extern CMachineFileDB* gcMachineFileDB;