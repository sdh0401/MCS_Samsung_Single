#pragma once
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
#include "CStep.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "CReadJobFile.h"
#include "Trace.h"

#include "GlobalDefine.h"

class CBarcode
{
public:
	CBarcode(long Gantry);
	~CBarcode();
	Point_XY GetBarcodePosition(long MarkNo);
	ORIGIN GetOrigin();
	long BarcodeChecking();
	long MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut);
	Ratio_XYRZ GetMinRatio();
	long SetCarrier(CString str);
	long SetBlock1(CString str);
	long SetBlock2(CString str);
private:
	long m_Gantry;
};

extern CBarcode* gcBarcode;
