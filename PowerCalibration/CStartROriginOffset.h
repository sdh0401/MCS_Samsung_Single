#pragma once
#include "CPowerThread.h"

class CStartROriginOffset : public CPowerThread
{
public:
	CStartROriginOffset();
	~CStartROriginOffset();
	void Run();
	static UINT RunThread(LPVOID wParam);
	long StartHeadROffsetCalibration();
	void SetHeadOffsetMode(bool bApply);
	bool GetHeadOffsetMode();
	Point_XY GetCameraAlignPosition(long Gantry);
	long GetAlignCameraNo(long Gantry);
	void SetCalibrationHead(long HeadNo);
	bool SetROriginOffset(long Gantry);
	void WriteROriginOffset(long Gantry);

private:
	long GetCalibrationHead();
	void ClearStep();
	bool m_bApply;
	long m_RepeatCount;
	long m_CalibrationHead;
	long m_ShowID;
};


extern CStartROriginOffset* gcStartROriginOffset;