#pragma once
#include "CPowerThread.h"

class CStartAlignOffset : public CPowerThread
{
public:
	CStartAlignOffset();
	~CStartAlignOffset();
	void ClearStep();
	void Run();
	static UINT RunThread(LPVOID wParam);
	long StartAlignOffsetCalibration();
	void SetRepeatCount(int RepeatCount);
	long GetRepeatCount();
	void SetAlignOffsetMode(bool bApply);
	bool GetAlignOffsetMode();
	void SetAlignOffsetCamera(long Camera);
	long GetAlignOffsetCamera();
	CString m_StrX;
	CString m_StrY;
private:
	long m_Gantry;
	void SetCameraAlignPosition(Point_XY pt);
	bool CopyCameraAlignPosition();
	bool WriteCameraAlignPosition();
	CalibrationAlignOffsetStep m_Step;
	bool m_bApply;
	long m_Camera;
	Point_XY m_CameraAlign;
	long m_RepeatCount;
	long m_ShowID;
};

extern CStartAlignOffset* gcStartAlignOffset;