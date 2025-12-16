#pragma once
#include "CPowerThread.h"

class CStartHeadOffset : public CPowerThread
{
public:
	CStartHeadOffset();
	~CStartHeadOffset();
	void Run();
	static UINT RunThread(LPVOID wParam);
	long StartHeadOffsetCalibration(void);
	void SetRepeatCount(int RepeatCount);
	void SetHeadOffsetMode(bool bApply);
	Point_XY GetCameraAlignPosition(long Gantry);
	long GetAlignCameraNo(long Gantry);
	void SetCalibrationHead(long HeadNo);

private:
	long GetCalibrationHead();
	void SetHeadOffset(long Gantry, long ZAxisNo, Point_XY pt);
	void SetCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY pt);
	void SetCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY pt);
	bool CopyHeadOffset(long Gantry);
	void WriteHeadOffset(long Gantry);
	long GetRepeatCount();
	void ClearStep();	
	bool GetHeadOffsetMode();
	CalibrationHeadOffsetStep m_Step;
	bool m_bApply;
	long m_RepeatCount;
	long m_CalibrationHead;
	long m_ShowID;
	Point_XY m_FrontHeadOffset[MAXUSEDHEADNO];
	Point_XY m_RearHeadOffset[MAXUSEDHEADNO];
	Point_XY m_FrontCameraRecogPosition[MAXUSEDHEADNO];
	Point_XY m_RearCameraRecogPosition[MAXUSEDHEADNO];
	Point_XY m_FrontCameraRecogOffset[MAXUSEDHEADNO];
	Point_XY m_RearCameraRecogOffset[MAXUSEDHEADNO];
};

extern CStartHeadOffset* gcStartHeadOffset;