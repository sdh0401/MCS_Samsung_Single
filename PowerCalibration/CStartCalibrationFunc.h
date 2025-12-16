#pragma once
#include "GlobalDefine.h"
#include "Thread.h"

class CStartCalibrationFunc : public CThread
{
public:
	CStartCalibrationFunc();
	~CStartCalibrationFunc();
	void StopCalibrationFunc();
	void StartOriginSearch(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void Start1DCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartOffsetCameraCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void Start2DCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartModuleCameraCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartAlignOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartHeadOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartZCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	void StartROriginOffsetCalibration(CalibrationStep CalStep, int nSub1, int nSub2, int nSub3);
	static UINT StartCalibrationFunc(LPVOID wParam);
	CalibrationStep CheckCalibrationStep(CString strHostMsg);
	void Run();
	void SetStep(CalibrationStep nStep);
	CalibrationStep GetStep();
	long GetThreadID();

private:
	CalibrationStep m_Step;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
};

extern CStartCalibrationFunc* gcStartCalibrationFunc;