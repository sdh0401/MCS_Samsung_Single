#pragma once
#include "GlobalDefine.h"


class CMeasureHeight
{

public:
	CMeasureHeight();
	~CMeasureHeight();
	long Run();
	long Run(long ManualMode);
	CString ConfirmMeasureHeightPosition(CString strHostMsg);
	CString TeachMeasureHeightPosition(CString strHostMsg);
	ORIGIN GetOrigin();
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	bool MakeFile();
	bool SaveFile();
	void AddResult(CString result);
//	CString GetResult(long index);
	void RemoveResult();

	void SetRatio(Ratio_XYRZ ratio);
	void SetDelay(long delay);
	long SendHeightMeasureEnd();
	long BlockSkipCheck();
	long SendHeightMeasureBeforeInsert(long Order, MEASUREHEIGHT measure, double realdata, double Gap, long measureReuslt);
	long MovePrepare();
	long HeightMeasureFromInsertNo(long InsertNo, double RatioXY, long BlockNo);

private:
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	CStringArray* m_StrArrResult;
	Ratio_XYRZ m_measureRatio;
	long m_measureDelay;
	
};

extern CMeasureHeight* gcMeasureHeight;
