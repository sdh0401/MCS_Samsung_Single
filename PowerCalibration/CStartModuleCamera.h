#pragma once
#include "CPowerThread.h"

class CStartModuleCamera : public CPowerThread
{
public:
	CStartModuleCamera();
	CStartModuleCamera(HANDLE h_Terminate);
	void InitializeValue();
	~CStartModuleCamera();
	void ClearStep();
	void Run();
	static UINT RunModuleCameraCalibration(LPVOID wParam);
	CalibrationCameraStep m_Step;
	void MakeCalibrationJigInfo();
	void RotationPoint(Point_XY* pt, double AngleErr);
	void ReadStartPosition();
	double GetJigPitchX();
	double GetJigPitchY();
	long GetCameraNo();
	void SetCameraNo(long CamNo);
private:	
	long m_Gantry;
	void SetJigPitchXY(Point_XY xy);
	Point_XY m_CalJigPos[70];
	Point_XY m_CalJigPitch;
	Point_XY m_CurPos;
	unsigned m_MaxCalJigNo;
	double m_AngleErr;
	unsigned m_Camera;
	unsigned m_Bdno;
	double m_PixelX, m_PixelY;
	double m_FovPerPixelX, m_FovPerPixelY;
	double m_FovX, m_FovY;
	CString m_StrX, m_StrY;
	long m_ShowID;
	long m_Cam;
};

extern CStartModuleCamera* gcStartModuleCamera;