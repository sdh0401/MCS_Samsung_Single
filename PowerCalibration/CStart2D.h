#pragma once
#include "CPowerThread.h"

class CStart2D : public CPowerThread
{
public:
	CStart2D();
	CStart2D(HANDLE h_Terminate);
	~CStart2D();
	void Run();
	static UINT Start2DCalibration(LPVOID wParam);
	void ClearStep();
	void Set2DCompenMode(bool bApply);
	void Set2DMethod(long Method);
	long Get2DMethod();
	bool Set2D(unsigned x, unsigned y, Point_XYRE pt);
	bool Show2D(unsigned MaxCntX, unsigned MaxCntY);
	bool Write2D();
	void Disable2D();
	void Enable2D();
	void Set2D(long Gantry, long MaxCntX, long MaxCntY);
	void SetStep(Calibration2DStep StartStep);

private:
	long m_Gantry;
	void SetStartPosition(Point_XY pt);
	Point_XY GetStartPosition();
	void Clear2DCompen();
	void Set2DBackup(long indx, Point_XYRE pt);
	Point_XYRE Get2DBackup(long indx);
	CString m_StrX;
	CString m_StrY;
	Calibration2DStep m_2DStep;
	Point_XY m_2DStartPosition;
	Point_XYRE m_2DPositionXY[CAL_2D_MAXCOUNT][CAL_2D_MAXCOUNT];
	Point_XYRE m_2DOffsetXY[CAL_2D_MAXCOUNT][CAL_2D_MAXCOUNT];
	Point_XYRE m_2DBackupXY[CAL_2D_MAXCOUNT * CAL_2D_MAXCOUNT];
	bool m_bApply;
	long m_Method;
	bool Get2DCompenMode();
	void Set2DPosition(unsigned x, unsigned y, Point_XYRE pt);
	void Set2DOffset(unsigned x, unsigned y, Point_XYRE pt);
	void Show2DPosition(long MaxCntX, long MaxCntY);
	void Show2DOffset(long MaxCntX, long MaxCntY);
	bool CopyGlobal2DCompensationData(long MaxCntX, long MaxCntY);
	long m_ShowID;
};

extern CStart2D* gcStart2D;