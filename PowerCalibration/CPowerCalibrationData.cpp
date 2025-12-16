#include "pch.h"
#include "CPowerCalibrationData.h"
#include "CMachineFile.h"
#include "GlobalDefine.h"
#include "LockDef.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CMachineFileDB.h"
#include "AxisInformation.h"

CPowerCalibrationData* gcPowerCalibrationData;
CPowerCalibrationData::CPowerCalibrationData()
{
	m_FrontGantry1DStart = 0.0;
	m_RearGantry1DStart = 0.0;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		ZeroMemory(&m_FrontGantry1D[indx], sizeof(m_FrontGantry1D[indx]));
		ZeroMemory(&m_RearGantry1D[indx], sizeof(m_RearGantry1D[indx]));
		ZeroMemory(&m_FrontAgingPoint[indx], sizeof(m_FrontAgingPoint[indx]));
		ZeroMemory(&m_RearAgingPoint[indx], sizeof(m_RearAgingPoint[indx]));
		ZeroMemory(&m_FrontGantry2D[indx], sizeof(m_FrontGantry2D[indx]));
		ZeroMemory(&m_RearGantry2D[indx], sizeof(m_RearGantry2D[indx]));
	}
	m_FrontHomeShiftDistance = 0.0;
	m_RearHomeShiftDistance = 0.0;
	ZeroMemory(&m_FrontGantry2DStart, sizeof(m_FrontGantry2DStart));
	ZeroMemory(&m_RearGantry2DStart, sizeof(m_RearGantry2DStart));
	ZeroMemory(&m_FrontCameraAlignPosition, sizeof(m_FrontCameraAlignPosition));
	ZeroMemory(&m_RearCameraAlignPosition, sizeof(m_RearCameraAlignPosition));
	for (int indx = 0; indx < MAXAXISNO; ++indx)
	{
		m_bUseAxis[indx] = false;
		m_HomePosition[indx] = 0.0;
	}
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		ZeroMemory(&m_FrontHeadOffset, sizeof(m_FrontHeadOffset));
		ZeroMemory(&m_RearHeadOffset, sizeof(m_RearHeadOffset));
	}
	ZeroMemory(&m_FrontZHMDOffset, sizeof(m_FrontZHMDOffset));
	ZeroMemory(&m_RearZHMDOffset, sizeof(m_RearZHMDOffset));
	m_FrontFeederPitch = 0.0;
	m_RearFeederPitch = 0.0;
	for (long indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		ZeroMemory(&m_FrontCompenZ[indx], sizeof(m_FrontCompenZ[indx]));
		ZeroMemory(&m_RearCompenZ[indx], sizeof(m_RearCompenZ[indx]));

		m_FrontCameraRecogOffsetAddMarkCompen[indx].x = 0.000;
		m_FrontCameraRecogOffsetAddMarkCompen[indx].y = 0.000;
		m_RearCameraRecogOffsetAddMarkCompen[indx].x = 0.000;
		m_RearCameraRecogOffsetAddMarkCompen[indx].y = 0.000;
	}
	m_FrontSensorOriginHeight = m_RearSensorOriginHeight = 0.0;
	m_EncoderDiffrent = 0;

	ZeroMemory(&m_InsertOffset4532Front, sizeof(m_InsertOffset4532Front));
	ZeroMemory(&m_InsertOffset4532Rear, sizeof(m_InsertOffset4532Rear));
}

CPowerCalibrationData::~CPowerCalibrationData()
{
}

void CPowerCalibrationData::SetYHomeEncoderDifferent(long Diff)
{
	TRACE(_T("[PWR] SetYHomeEncoderDifferent %d->%d"), m_EncoderDiffrent, Diff);
	m_EncoderDiffrent = Diff;
}
long CPowerCalibrationData::GetYHomeEncoderDifferent()
{
	TRACE(_T("[PWR] GetYHomeEncoderDifferent %d"), m_EncoderDiffrent);
	return m_EncoderDiffrent;
}

void CPowerCalibrationData::Load1DStartCompensationData(long Gantry, double CompenData)
{
	double Compen = CompenData;
	if (abs(Compen) > MAX_MOTION_VALID_RANGE)
		Compen = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry1DStart = Compen;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry1DStart = Compen;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Load1DStartCompensationData\n"));
	}
}

double CPowerCalibrationData::Get1DStartCompensationData(long Gantry)
{
	double Ret = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontGantry1DStart;
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearGantry1DStart;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Get1DStartCompensationData\n"));
	}
	if (abs(Ret) > MAX_MOTION_VALID_RANGE)
	{
		Ret = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::Set1DStartCompensationData(long Gantry, double StartCompenData)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f\n"), strFunc, Gantry, StartCompenData);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry1DStart = StartCompenData;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry1DStart = StartCompenData;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Set1DStartCompensationData\n"));
	}
}

void CPowerCalibrationData::Load1DCompensationData(long Gantry, long indx, double CompenData)
{
	double Compen = CompenData;
	if (abs(Compen) > MAX_MOTION_VALID_RANGE)
		Compen = 0.0;
	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry1D[indx] = Compen;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry1D[indx] = Compen;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Load1DCompensationData\n"));
	}
}

double CPowerCalibrationData::Get1DCompensationData(long Gantry, long indx)
{
	double Ret = 0.0;
	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontGantry1D[indx];
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearGantry1D[indx];
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Get1DCompensationData\n"));
	}
	if (abs(Ret) > MAX_MOTION_VALID_RANGE)
	{
		Ret = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::Set1DCompensationData(long Gantry, long indx, double CompenData)
{
	CString strFunc(__func__);
	if (indx % 10 == 0)
	{
		TRACE(_T("[PWR] %s Gantry:%d %d %.3f\n"), strFunc, Gantry, indx, CompenData);
	}

	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry1D[indx] = CompenData;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry1D[indx] = CompenData;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Set1DCompensationData\n"));
	}
}

bool CPowerCalibrationData::Write1DCompensationData(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->Save1d(Gantry);
	}

	bRet = gcMachineFile->WriteToDiskBlock1();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadAgingPosition(long Gantry, long index, Point_XYRZE AgingPosition)
{
	Point_XYRZE pt = AgingPosition;
	ASSERT(index < BUFSIZE);
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (abs(pt.r) > MAX_MOTION_VALID_RANGE)
		pt.r = 0.0;
	if (abs(pt.z) > MAX_MOTION_VALID_RANGE)
		pt.z = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontAgingPoint[index] = pt;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearAgingPoint[index] = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadAgingPosition\n"));
	}
}

void CPowerCalibrationData::SetAgingPosition(long Gantry, long index, Point_XYRZE pt)
{
	ASSERT(index < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontAgingPoint[index].x = pt.x;
		m_FrontAgingPoint[index].y = pt.y;
		m_FrontAgingPoint[index].r = pt.r;
		m_FrontAgingPoint[index].z = pt.z;
		m_FrontAgingPoint[index].exe = pt.exe;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearAgingPoint[index].x = pt.x;
		m_RearAgingPoint[index].y = pt.y;
		m_RearAgingPoint[index].r = pt.r;
		m_RearAgingPoint[index].z = pt.z;
		m_FrontAgingPoint[index].exe = pt.exe;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetAgingPosition\n"));
	}
}

Point_XYRZE CPowerCalibrationData::GetAgingPosition(long Gantry, long index)
{
	Point_XYRZE pt;
	ZeroMemory(&pt, sizeof(pt));
	ASSERT(index < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		pt = m_FrontAgingPoint[index];
	}
	else if (Gantry == REAR_GANTRY)
	{
		pt = m_RearAgingPoint[index];
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetAgingPosition\n"));
	}
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (abs(pt.r) > MAX_MOTION_VALID_RANGE)
		pt.r = 0.0;
	if (abs(pt.z) > MAX_MOTION_VALID_RANGE)
		pt.z = 0.0;
	return pt;
}

bool CPowerCalibrationData::WriteAgingPosition(long Gantry)
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlockC();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadHomeShiftDistance(long Gantry, double shiftDistance)
{
	double Dist = shiftDistance;
	if (abs(Dist) > MAX_MOTION_VALID_RANGE)
		Dist = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontHomeShiftDistance = Dist;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearHomeShiftDistance = Dist;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadHomeShiftDistance\n"));
	}
	TRACE(_T("[PWR] LoadHomeShiftDistance %s:%.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), Dist);
}

void CPowerCalibrationData::SetHomeShiftDistance(long Gantry, double shiftDistance)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f\n"), strFunc, Gantry, shiftDistance);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontHomeShiftDistance = shiftDistance;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearHomeShiftDistance = shiftDistance;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetHomeShiftDistance\n"));
	}
	//if (gcPowerLog->IsShowHomingLog() == true)
	//{
	//	TRACE(_T("[PWR] SetHomeShiftDistance %s:%.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), shiftDistance);
	//}
}

double CPowerCalibrationData::GetHomeShiftDistance(long Gantry)
{
	double ShiftDistance = 0.0f;
	if (Gantry == FRONT_GANTRY)
	{
		ShiftDistance = m_FrontHomeShiftDistance;
	}
	else if (Gantry == REAR_GANTRY)
	{
		ShiftDistance = m_RearHomeShiftDistance;		
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetHomeShiftDistance\n"));
	}
	if (abs(ShiftDistance) > MAX_MOTION_VALID_RANGE)
	{
		ShiftDistance = 0.0;
	}
	if (gcPowerLog->IsShowHomingLog() == true)
	{
		TRACE(_T("[PWR] GetHomeShiftDistance %s:%.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ShiftDistance);
	}
	return ShiftDistance;
}

bool CPowerCalibrationData::WriteHomeShiftDistance(long Gantry)
{
	bool bRet = false;

	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveAxisParameter();
	}

	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::Load2DCompensationData(long Gantry, long indx, Point_XYRE CompenData)
{
	Point_XYRE pt = CompenData;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
	{
		pt.x = 0.0;
	}
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
	{
		pt.y = 0.0;
	}
	ASSERT(indx < (CAL_2D_MAXCOUNT* CAL_2D_MAXCOUNT));
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry2D[indx] = pt;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry2D[indx] = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Load2DCompensationData\n"));
	}
}

Point_XYRE CPowerCalibrationData::Get2DCompensationData(long Gantry, long indx)
{
	ASSERT(indx < (CAL_2D_MAXCOUNT* CAL_2D_MAXCOUNT));
	Point_XYRE Ret;
	ZeroMemory(&Ret, sizeof(&Ret));
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontGantry2D[indx];
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearGantry2D[indx];
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Get2DCompensationData\n"));
	}
	if (abs(Ret.x) > MAX_MOTION_VALID_RANGE)
	{
		Ret.x = 0.0;
	}
	if (abs(Ret.y) > MAX_MOTION_VALID_RANGE)
	{
		Ret.y = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::Set2DCompensationData(long Gantry, long indx, Point_XYRE CompenData)
{
	CString strFunc(__func__);
	if (indx % 10 == 0)
	{
		TRACE(_T("[PWR] %s Gantry:%d %d %.3f %.3f\n"), strFunc, Gantry, indx, CompenData.x, CompenData.y);
	}

	ASSERT(indx < (CAL_2D_MAXCOUNT * CAL_2D_MAXCOUNT));
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry2D[indx] = CompenData;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry2D[indx] = CompenData;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Set2DCompensationData\n"));
	}
}

void CPowerCalibrationData::Load2DStartPosition(long Gantry, Point_XY Start)
{
	Point_XY pt = Start;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry2DStart = pt;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry2DStart = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Load2DStartPosition\n"));
	}
}

Point_XY CPowerCalibrationData::Get2DStartPosition(long Gantry)
{
	Point_XY Ret;
	ZeroMemory(&Ret, sizeof(&Ret));
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontGantry2DStart;
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearGantry2DStart;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Get2DStartPosition\n"));
	}
	if (abs(Ret.x) > MAX_MOTION_VALID_RANGE)
	{
		Ret.x = 0.0;
	}
	if (abs(Ret.y) > MAX_MOTION_VALID_RANGE)
	{
		Ret.y = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::Set2DStartPosition(long Gantry, Point_XY Start)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f %.3f\n"), strFunc, Gantry, Start.x, Start.y);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontGantry2DStart = Start;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearGantry2DStart = Start;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry Set2DStartPosition\n"));
	}
}

bool CPowerCalibrationData::Write2DCompensationData(long Gantry)
{
	bool bRet = false;

	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->Save2d(Gantry);
	}

	if(Gantry == FRONT_GANTRY)
		bRet = gcMachineFile->WriteToDiskBlock2();
	else if (Gantry == REAR_GANTRY)
		bRet = gcMachineFile->WriteToDiskBlock3();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadCameraAlignPosition(long Gantry, Point_XY CameraAlignXY)
{
	Point_XY pt = CameraAlignXY;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCameraAlignPosition = pt;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCameraAlignPosition = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadCameraAlignPosition\n"));
	}
}

Point_XY CPowerCalibrationData::GetCameraAlignPosition(long Gantry)
{
	Point_XY Ret;
	ZeroMemory(&Ret, sizeof(&Ret));
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontCameraAlignPosition;
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearCameraAlignPosition;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetCameraAlignPosition\n"));
	}
	if (abs(Ret.x) > MAX_MOTION_VALID_RANGE)
	{
		Ret.x = 0.0;
	}
	if (abs(Ret.y) > MAX_MOTION_VALID_RANGE)
	{
		Ret.y = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetCameraAlignPosition(long Gantry, Point_XY CameraAlignXY)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d XY:%.3f,%.3f\n"), strFunc, Gantry, CameraAlignXY.x, CameraAlignXY.y);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCameraAlignPosition = CameraAlignXY;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCameraAlignPosition = CameraAlignXY;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetCameraAlignPosition\n"));
	}
}

void CPowerCalibrationData::LoadAlignCamera(long Gantry, long Camera)
{
	long AlignCamera = Camera;
	if (Camera > MAXCAMNO)
	{
		AlignCamera = CAM1;
	}
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontAlignCamera = AlignCamera;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearAlignCamera = AlignCamera;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadAlignCamera\n"));
	}
}

long CPowerCalibrationData::GetAlignCamera(long Gantry)
{
	long AlignCamera = CAM1;
	if (Gantry == FRONT_GANTRY)
	{
		AlignCamera = m_FrontAlignCamera;
	}
	else if (Gantry == REAR_GANTRY)
	{
		AlignCamera = m_RearAlignCamera;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetAlignCamera\n"));
	}
	return AlignCamera;
}

void CPowerCalibrationData::SetAlignCamera(long Gantry, long Camera)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d Cam:%d\n"), strFunc, Gantry, Camera);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontAlignCamera = Camera;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearAlignCamera = Camera;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetAlignCamera\n"));
	}
}

bool CPowerCalibrationData::WriteCameraAlignPosition(long Gantry)
{
	bool bRet = false;

	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveAlign();
	}

	bRet = gcMachineFile->WriteToDiskBlock8();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadUseAxis(long AxisNo, bool bUse)
{
	if (AxisNo < MAXAXISNO)
	{
		m_bUseAxis[AxisNo] = bUse;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadUseAxis AxisNo:%d %s\n"), AxisNo, bUse == true ? _T("Use") : _T("NoUse"));
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadUseAxis Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

void CPowerCalibrationData::SetUseAxis(long AxisNo, bool bUse)
{
	if (AxisNo < MAXAXISNO)
	{
		m_bUseAxis[AxisNo] = bUse;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetUseAxis AxisNo:%d %s\n"), AxisNo, bUse == true ? _T("Use") : _T("NoUse"));
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetUseAxis Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

bool CPowerCalibrationData::GetUseAxis(long AxisNo)
{
	bool bUse = false;
	if (AxisNo < MAXAXISNO)
	{
		bUse = m_bUseAxis[AxisNo];
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetUseAxis AxisNo:%d %s\n"), AxisNo, bUse == true ? _T("Use") : _T("NoUse"));
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetUseAxis Invalid AxisNo:%d\n"), AxisNo);
		}
	}
	return bUse;
}

bool CPowerCalibrationData::WriteUseAxis(long Gantry)
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadLimit(long AxisNo, Limit limit)
{
	if (AxisNo < MAXAXISNO)
	{
		m_Limit[AxisNo] = limit;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadLimit AxisNo:%d Minus:%.3f Plus:%.3f\n"), AxisNo, limit.minus, limit.plus);
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadLimit Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

void CPowerCalibrationData::SetLimit(long AxisNo, Limit limit)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Axis:%d Minus:%.3f Plus:%.3f\n"), strFunc, AxisNo, limit.minus, limit.plus);

	if (AxisNo < MAXAXISNO)
	{
		m_Limit[AxisNo] = limit;
		//if (gcPowerLog->IsShowRunLog() == true)
		//{
		//	TRACE(_T("[PWR] SetLimit AxisNo:%d Minus:%.3f Plus:%.3f\n"), AxisNo, limit.minus, limit.plus);
		//}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetLimit Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

Limit CPowerCalibrationData::GetLimit(long AxisNo)
{
	Limit RetLimit;
	ZeroMemory(&RetLimit, sizeof(RetLimit));
	if (AxisNo < MAXAXISNO)
	{
		RetLimit = m_Limit[AxisNo];
		if (abs(RetLimit.minus) > MAX_MOTION_VALID_RANGE)
		{
			RetLimit.minus = 0;
		}
		if (abs(RetLimit.plus) > MAX_MOTION_VALID_RANGE)
		{
			RetLimit.plus = 0;
		}
		//TRACE(_T("[PWR] AxisNo:%d GetLimit Minus:%.3f Plus:%.3f\n"), AxisNo, RetLimit.minus, RetLimit.plus);
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetLimit Invalid AxisNo:%d\n"), AxisNo);
		}
	}
	return RetLimit;
}

bool CPowerCalibrationData::WriteLimit(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveAxisParameter();
	}
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadHomePosition(long AxisNo, double HomePosition)
{
	if (AxisNo < MAXAXISNO)
	{
		m_HomePosition[AxisNo] = HomePosition;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadHomePosition AxisNo:%d HomePosition:%.3f\n"), AxisNo, HomePosition);
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadHomePosition Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

void CPowerCalibrationData::SetHomePosition(long AxisNo, double HomePosition)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Axis:%d %.3f\n"), strFunc, AxisNo, HomePosition);

	if (AxisNo < MAXAXISNO)
	{
		m_HomePosition[AxisNo] = HomePosition;
		//if (gcPowerLog->IsShowRunLog() == true)
		//{
		//	TRACE(_T("[PWR] SetHomePosition AxisNo:%d HomePosition:%.3f\n"), AxisNo, HomePosition);
		//}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetHomePosition Invalid AxisNo:%d\n"), AxisNo);
		}
	}
}

double CPowerCalibrationData::GetHomePosition(long AxisNo)
{
	double HomePosition = 0.0;
	if (AxisNo < MAXAXISNO)
	{
		HomePosition = m_HomePosition[AxisNo];
		if (abs(HomePosition) > MAX_MOTION_VALID_RANGE)
		{
			HomePosition = 0.0;
		}
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetHomePosition AxisNo:%d HomePosition:%.3f\n"), AxisNo, HomePosition);
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetOriginOffset Invalid AxisNo:%d\n"), AxisNo);
		}
	}
	return HomePosition;
}

bool CPowerCalibrationData::WriteHomePosition(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveAxisParameter();
	}
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadHeadOffset(long Gantry, long ZAxisNo, Point_XY HeadOffset)
{
	if (abs(HeadOffset.x) > MAX_MOTION_VALID_RANGE)
		HeadOffset.x = 0.000;
	if (abs(HeadOffset.y) > MAX_MOTION_VALID_RANGE)
		HeadOffset.y = 0.000;
	if(ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontHeadOffset[ZAxisNo - 1] = HeadOffset;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearHeadOffset[ZAxisNo - 1] = HeadOffset;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry LoadHeadOffset\n"));
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] LoadHeadOffset Invalid AxisNo:%d\n"), ZAxisNo);
		}
	}
}

void CPowerCalibrationData::SetHeadOffset(long Gantry, long ZAxisNo, Point_XY HeadOffset)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d Head:%d %.3f %.3f\n"), strFunc, Gantry, ZAxisNo, HeadOffset.x, HeadOffset.y);

	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontHeadOffset[ZAxisNo - 1] = HeadOffset;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearHeadOffset[ZAxisNo - 1] = HeadOffset;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry SetHeadOffset\n"));
		}
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetHeadOffset %s ZAxis(%d) OffsetXY %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, HeadOffset.x, HeadOffset.y);
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetHeadOffset Invalid AxisNo:%d\n"), ZAxisNo);
		}
	}
}

Point_XY CPowerCalibrationData::GetHeadOffset(long Gantry, long ZAxisNo)
{
	Point_XY RetOffset;
	ZeroMemory(&RetOffset, sizeof(RetOffset));
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			RetOffset = m_FrontHeadOffset[ZAxisNo - 1];
		}
		else if (Gantry == REAR_GANTRY)
		{
			RetOffset = m_RearHeadOffset[ZAxisNo - 1];
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) GetHeadOffset\n"), Gantry);
		}
		if (abs(RetOffset.x) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.x = 0;
		}
		if (abs(RetOffset.y) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.y = 0;
		}
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetHeadOffset %s ZAxis(%d) OffsetXY %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, RetOffset.x, RetOffset.y);
		}
	}
	else
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetHeadOffset Invalid AxisNo:%d\n"), ZAxisNo);
		}
	}
	return RetOffset;
}

bool CPowerCalibrationData::WriteHeadOffset(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveHeadOffset();
	}
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadHMOffset(long Gantry, Point_XY HMOffset)
{
	Point_XY pt = HMOffset;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.000;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.000;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontZHMDOffset = pt;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearZHMDOffset = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry(%d) LoadHMOffset\n"), Gantry);
	}
}

void CPowerCalibrationData::SetHMOffset(long Gantry, Point_XY HMOffset)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f %.3f\n"), strFunc, Gantry, HMOffset.x, HMOffset.y);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontZHMDOffset = HMOffset;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearZHMDOffset = HMOffset;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry(%d) SetHMOffset\n"), Gantry);
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetHMOffset %s OffsetXY %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), HMOffset.x, HMOffset.y);
	}
}


void CPowerCalibrationData::SetHMZero(long Gantry, long zerolevel)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f\n"), strFunc, Gantry, zerolevel);

	if (Gantry == FRONT_GANTRY)
	{
		m_FrontHMZero = zerolevel;
	}
	else
	{
		m_RearHMZero = zerolevel;
	}

	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] SetHMZero %s level %d\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), zerolevel);
	}
}

long CPowerCalibrationData::GetHMZero(long Gantry)
{
	if (Gantry == FRONT_GANTRY)
	{
		return m_FrontHMZero;
	}
	else
	{
		return m_RearHMZero;
	}

	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHMZero %s level %d\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), m_FrontHMZero);
	}
}



Point_XY CPowerCalibrationData::GetHMOffset(long Gantry)
{
	Point_XY RetOffset;
	ZeroMemory(&RetOffset, sizeof(RetOffset));
	if (Gantry == FRONT_GANTRY)
	{
		RetOffset = m_FrontZHMDOffset;
	}
	else if (Gantry == REAR_GANTRY)
	{
		RetOffset = m_RearZHMDOffset;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry(%d) GetHMOffset\n"), Gantry);
	}
	if (abs(RetOffset.x) > MAX_MOTION_VALID_RANGE)
	{
		RetOffset.x = 0.0;
	}
	if (abs(RetOffset.y) > MAX_MOTION_VALID_RANGE)
	{
		RetOffset.y = 0.0;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetHMOffset %s OffsetXY %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), RetOffset.x, RetOffset.y);
	}
	return RetOffset;
}

bool CPowerCalibrationData::WriteHMOffset(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveHeightMeasurement();
	}
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY RecogPosition)
{
	Point_XY pt = RecogPosition;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.000;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.000;
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontCameraRecogPosition[ZAxisNo - 1] = pt;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearCameraRecogPosition[ZAxisNo - 1] = pt;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) LoadCameraRecognitionPosition\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] LoadCameraRecognitionPosition Invalid AxisNo:%d\n"), ZAxisNo);
	}
}

void CPowerCalibrationData::SetCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY RecogPosition)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d Head:%d %.3f %.3f\n"), strFunc, Gantry, ZAxisNo, RecogPosition.x, RecogPosition.y);

	Point_XY pt = RecogPosition;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.000;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.000;
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontCameraRecogPosition[ZAxisNo - 1] = pt;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearCameraRecogPosition[ZAxisNo - 1] = pt;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) SetCameraRecognitionPosition\n"), Gantry);
		}
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetCameraRecognitionPosition %s ZAxisNo(%d) RecognitionPosition %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetCameraRecognitionPosition Invalid AxisNo:%d\n"), ZAxisNo);
	}
}

Point_XY CPowerCalibrationData::GetCameraRecognitionPosition(long Gantry, long ZAxisNo)
{
	Point_XY RetOffset;
	ZeroMemory(&RetOffset, sizeof(RetOffset));
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			RetOffset = m_FrontCameraRecogPosition[ZAxisNo - 1];
		}
		else if (Gantry == REAR_GANTRY)
		{
			RetOffset = m_RearCameraRecogPosition[ZAxisNo - 1];
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) GetCameraRecognitionPosition\n"), Gantry);
		}
		if (abs(RetOffset.x) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.x = 0;
		}
		if (abs(RetOffset.y) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.y = 0;
		}
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetCameraRecognitionPosition %s ZAxisNo(%d) RecognitionPosition %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, RetOffset.x, RetOffset.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetCameraRecognitionPosition Invalid AxisNo:%d\n"), ZAxisNo);
	}
	return RetOffset;
}

bool CPowerCalibrationData::WriteCameraRecognitionPosition(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveRecogPosition();
	}
	bRet = gcMachineFile->WriteToDiskBlock4();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}


void CPowerCalibrationData::LoadCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY RecogPosition)
{
	Point_XY pt = RecogPosition;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.000;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.000;
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontCameraRecogOffset[ZAxisNo - 1] = pt;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearCameraRecogOffset[ZAxisNo - 1] = pt;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) LoadCameraRecognitionOffset\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] LoadCameraRecognitionOffset Invalid AxisNo:%d\n"), ZAxisNo);
	}
}

void CPowerCalibrationData::SetCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY RecogPosition)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d Head:%d %.3f %.3f\n"), strFunc, Gantry, ZAxisNo, RecogPosition.x, RecogPosition.y);

	Point_XY pt = RecogPosition;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.000;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.000;
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			m_FrontCameraRecogOffset[ZAxisNo - 1] = pt;
		}
		else if (Gantry == REAR_GANTRY)
		{
			m_RearCameraRecogOffset[ZAxisNo - 1] = pt;
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) SetCameraRecognitionOffset\n"), Gantry);
		}
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] SetCameraRecognitionOffset %s ZAxisNo(%d) RecognitionOffset %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, pt.x, pt.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] SetCameraRecognitionOffset Invalid AxisNo:%d\n"), ZAxisNo);
	}
}

Point_XY CPowerCalibrationData::GetCameraRecognitionOffset(long Gantry, long ZAxisNo)
{
	Point_XY RetOffset;
	ZeroMemory(&RetOffset, sizeof(RetOffset));
	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			RetOffset = m_FrontCameraRecogOffset[ZAxisNo - 1];
		}
		else if (Gantry == REAR_GANTRY)
		{
			RetOffset = m_RearCameraRecogOffset[ZAxisNo - 1];
		}
		else
		{
			TRACE(_T("[PWR] INVALID Gantry(%d) GetCameraRecognitionOffset\n"), Gantry);
		}
		if (abs(RetOffset.x) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.x = 0.000;
		}
		if (abs(RetOffset.y) > MAX_MOTION_VALID_RANGE)
		{
			RetOffset.y = 0.000;
		}
		//if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] GetCameraRecognitionOffset %s ZAxisNo(%d) GetCameraRecognitionOffset %.3f %.3f\n"), Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo, RetOffset.x, RetOffset.y);
		}
	}
	else
	{
		TRACE(_T("[PWR] GetCameraRecognitionOffset Invalid AxisNo:%d\n"), ZAxisNo);
	}
	return RetOffset;
}

bool CPowerCalibrationData::WriteCameraRecognitionOffset(long Gantry)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveRecogOffset();
	}
	bRet = gcMachineFile->WriteToDiskBlock4();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadReferenceFeederPosition(long Stage, Point_XY RefFdPos)
{
	Point_XY pt = RefFdPos;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (Stage == FRONT_STAGE)
	{
		m_FrontReferenceFeederPosition = pt;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearReferenceFeederPosition = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage LoadReferenceFeederPosition\n"));
	}
}

Point_XY CPowerCalibrationData::GetReferenceFeederPosition(long Stage)
{
	Point_XY Ret;
	ZeroMemory(&Ret, sizeof(&Ret));
	if (Stage == FRONT_STAGE)
	{
		Ret = m_FrontReferenceFeederPosition;
	}
	else if (Stage == REAR_STAGE)
	{
		Ret = m_RearReferenceFeederPosition;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage GetReferenceFeederPosition\n"));
	}
	if (abs(Ret.x) > MAX_MOTION_VALID_RANGE)
	{
		Ret.x = 0.0;
	}
	if (abs(Ret.y) > MAX_MOTION_VALID_RANGE)
	{
		Ret.y = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetReferenceFeederPosition(long Stage, Point_XY RefFdPos)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f,%.3f\n"), strFunc, Stage, RefFdPos.x, RefFdPos.y);

	if (Stage == FRONT_STAGE)
	{
		m_FrontReferenceFeederPosition = RefFdPos;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearReferenceFeederPosition = RefFdPos;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage SetReferenceFeederPosition\n"));
	}
}

void CPowerCalibrationData::LoadReferenceFeederNo(long Stage, long RefFdNo)
{
	long ReferenceFeederNo = RefFdNo;
	if (abs(ReferenceFeederNo) > MAXFEEDERNO)
	{
		if(Stage == FRONT_STAGE)
			ReferenceFeederNo = 30;
		else if (Stage == REAR_STAGE)
			ReferenceFeederNo = 130;
	}
	if (Stage == FRONT_STAGE)
	{
		m_FrontReferenceFeederNo = RefFdNo;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearReferenceFeederNo = RefFdNo;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage LoadReferenceFeederNo\n"));
	}
}

long CPowerCalibrationData::GetReferenceFeederNo(long Stage)
{
	long RefFdNo = 30;
	if (Stage == FRONT_STAGE)
	{
		RefFdNo = m_FrontReferenceFeederNo;
	}
	else if (Stage == REAR_STAGE)
	{
		RefFdNo = m_RearReferenceFeederNo;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage GetReferenceFeederNo\n"));
	}
	return RefFdNo;
}

void CPowerCalibrationData::SetReferenceFeederNo(long Stage, long RefFdNo)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %d\n"), strFunc, Stage, RefFdNo);

	if (Stage == FRONT_STAGE)
	{
		m_FrontReferenceFeederNo = RefFdNo;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearReferenceFeederNo = RefFdNo;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage SetReferenceFeederNo\n"));
	}
}

void CPowerCalibrationData::LoadFeederPitch(long Stage, double Pitch)
{
	double FeederPitch = Pitch;
	if (abs(FeederPitch) > MAX_MOTION_VALID_RANGE)
	{
		FeederPitch = DEFAULT_FEEDER_PITCH;
	}
	if (Stage == FRONT_STAGE)
	{
		m_FrontFeederPitch = FeederPitch;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearFeederPitch = FeederPitch;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage LoadFeederPitch\n"));
	}
}

double CPowerCalibrationData::GetFeederPitch(long Stage)
{
	double FeederPitch = 0.0;
	if (Stage == FRONT_STAGE)
	{
		FeederPitch = m_FrontFeederPitch;
	}
	else if (Stage == REAR_STAGE)
	{
		FeederPitch = m_RearFeederPitch;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage GetFeederPitch\n"));
	}
	return FeederPitch;
}

void CPowerCalibrationData::SetFeederPitch(long Stage, double Pitch)
{
	if (Stage == FRONT_STAGE)
	{
		m_FrontFeederPitch = Pitch;
	}
	else if (Stage == REAR_STAGE)
	{
		m_RearFeederPitch = Pitch;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Stage SetFeederPitch\n"));
	}
}

bool CPowerCalibrationData::WriteReferenceFeeder(long Stage)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SaveFeederReference();
	}
	bRet = gcMachineFile->WriteToDiskBlock9();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::LoadPcbFixPosition(long Conveyor, Point_XY PcbFixPos)
{
	Point_XY pt = PcbFixPos;
	if (abs(pt.x) > MAX_MOTION_VALID_RANGE)
		pt.x = 0.0;
	if (abs(pt.y) > MAX_MOTION_VALID_RANGE)
		pt.y = 0.0;
	if (Conveyor == FRONT_CONV)
	{
		m_FrontPCBFixPosition = pt;
	}
	else if (Conveyor == REAR_CONV)
	{
		m_RearPCBFixPosition = pt;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Conveyor LoadPcbFixPosition\n"));
	}
}

Point_XY CPowerCalibrationData::GetPcbFixPosition(long Conveyor)
{
	Point_XY Ret;
	ZeroMemory(&Ret, sizeof(&Ret));
	if (Conveyor == FRONT_CONV)
	{
		Ret = m_FrontPCBFixPosition;
	}
	else if (Conveyor == REAR_CONV)
	{
		Ret = m_RearPCBFixPosition;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Conveyor GetPcbFixPosition\n"));
	}
	if (abs(Ret.x) > MAX_MOTION_VALID_RANGE)
	{
		Ret.x = 0.0;
	}
	if (abs(Ret.y) > MAX_MOTION_VALID_RANGE)
	{
		Ret.y = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetPcbFixPosition(long Conveyor, Point_XY PcbFixPos)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Gantry:%d %.3f,%.3f\n"), strFunc, Conveyor, PcbFixPos.x, PcbFixPos.y);

	if (Conveyor == FRONT_CONV)
	{
		m_FrontPCBFixPosition = PcbFixPos;
	}
	else if (Conveyor == REAR_CONV)
	{
		m_RearPCBFixPosition = PcbFixPos;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetPcbFixPosition\n"));
	}
}

bool CPowerCalibrationData::WritePcbFixPosition(long Conveyor)
{
	bool bRet = false;
	if (gcMachineFileDB->GetLoadComplete() == true)
	{
		return gcMachineFileDB->SavePcbFix();
	}
	bRet = gcMachineFile->WriteToDiskBlock5();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}


void CPowerCalibrationData::LoadZCompensationData(long Gantry, long HeadNo, long indx, double CompenData)
{
	double Compen = CompenData;
	if (abs(Compen) > MAX_MOTION_VALID_RANGE)
		Compen = 0.0;
	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCompenZ[HeadNo][indx] = Compen;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCompenZ[HeadNo][indx] = Compen;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadZCompensationData\n"));
	}
}

double CPowerCalibrationData::GetZCompensationData(long Gantry, long HeadNo, long indx)
{
	double Ret = 0.0;
	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontCompenZ[HeadNo][indx];
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearCompenZ[HeadNo][indx];
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetZCompensationData\n"));
	}
	if (abs(Ret) > MAX_MOTION_VALID_RANGE)
	{
		Ret = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetZCompensationData(long Gantry, long HeadNo, long indx, double CompenData)
{
	double Compen = CompenData;
	if (abs(Compen) > MAX_MOTION_VALID_RANGE)
		Compen = 0.0;
	ASSERT(indx < BUFSIZE);
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCompenZ[HeadNo][indx] = Compen;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCompenZ[HeadNo][indx] = Compen;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetZCompensationData\n"));
	}
}

bool CPowerCalibrationData::WriteZCompensationData(long Gantry)
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlockA();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}


void CPowerCalibrationData::LoadZCompensationStartData(long Gantry, long HeadNo, double CompenStartData)
{
	double CompenStart = CompenStartData;
	if (abs(CompenStart) > MAX_MOTION_VALID_RANGE)
		CompenStart = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCompenStartZ[HeadNo] = CompenStart;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCompenStartZ[HeadNo] = CompenStart;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadZCompensationStartData\n"));
	}
}

double CPowerCalibrationData::GetZCompensationStartData(long Gantry, long HeadNo)
{
	double Ret = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontCompenStartZ[HeadNo];
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearCompenStartZ[HeadNo];
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetZCompensationStartData\n"));
	}
	if (abs(Ret) > MAX_MOTION_VALID_RANGE)
	{
		Ret = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetZCompensationStartData(long Gantry, long HeadNo, double CompenStartData)
{
	double CompenStart = CompenStartData;
	if (abs(CompenStart) > MAX_MOTION_VALID_RANGE)
		CompenStart = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontCompenStartZ[HeadNo] = CompenStart;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearCompenStartZ[HeadNo] = CompenStart;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetZCompensationStartData\n"));
	}
}


void CPowerCalibrationData::LoadSensorOriginHeight(long Gantry, double SensorHeight)
{
	double Height = SensorHeight;
	if (abs(Height) > MAX_MOTION_VALID_RANGE)
		Height = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontSensorOriginHeight = Height;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearSensorOriginHeight = Height;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry LoadSensorOriginHeight\n"));
	}
}

double CPowerCalibrationData::GetSensorOriginHeight(long Gantry)
{
	double Ret = 0.0;
	if (Gantry == FRONT_GANTRY)
	{
		Ret = m_FrontSensorOriginHeight;
	}
	else if (Gantry == REAR_GANTRY)
	{
		Ret = m_RearSensorOriginHeight;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry GetSensorOriginHeight\n"));
	}
	if (abs(Ret) > MAX_MOTION_VALID_RANGE)
	{
		Ret = 0.0;
	}
	return Ret;
}

void CPowerCalibrationData::SetSensorOriginHeight(long Gantry, double SensorHeight)
{
	if (Gantry == FRONT_GANTRY)
	{
		m_FrontSensorOriginHeight = SensorHeight;
	}
	else if (Gantry == REAR_GANTRY)
	{
		m_RearSensorOriginHeight = SensorHeight;
	}
	else
	{
		TRACE(_T("[PWR] INVALID Gantry SetSensorOriginHeight\n"));
	}
}

bool CPowerCalibrationData::WriteSensorOriginHeight(long Gantry)
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlockB();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerCalibrationData::SetANCMarkPosition(long Base, long MarkNo, Point_XY Mark)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Base:%d Mark%d %.3f,%.3f\n"), strFunc, Base, MarkNo, Mark.x, Mark.y);

	if (fabs(Mark.x) > 2000.0 || fabs(Mark.y) > 2000.0)
	{
		TRACE(_T("[PWR] Init SetANCMarkPosition Base:%d MarkXY:%.3f, %.3f\n"), Base, Mark.x, Mark.y);
		Mark.x = Mark.y = 0.000;
	}

	if (Base == FRONT_STAGE && MarkNo == 0)
	{
		m_FrontANCMark1 = Mark;
	}
	else if (Base == FRONT_STAGE && MarkNo == 1)
	{
		m_FrontANCMark2 = Mark;
	}
	else if (Base == REAR_STAGE && MarkNo == 0)
	{
		m_RearANCMark1 = Mark;
	}
	else if (Base == REAR_STAGE && MarkNo == 1)
	{
		m_RearANCMark2 = Mark;
	}
	else
	{
		TRACE(_T("[PWR] INVALID SetANCMarkPosition ANC Base:%d MarkNo:%d\n"), Base, MarkNo);
	}
}


Point_XY CPowerCalibrationData::GetANCMarkPosition(long Base, long MarkNo)
{
	Point_XY Mark;

	Mark.x = 0.000;
	Mark.y = 0.000;

	if (Base == FRONT_STAGE && MarkNo == 0)
	{
		Mark = m_FrontANCMark1;
	}
	else if (Base == FRONT_STAGE && MarkNo == 1)
	{
		Mark = m_FrontANCMark2;
	}
	else if (Base == REAR_STAGE && MarkNo == 0)
	{
		Mark = m_RearANCMark1;
	}
	else if (Base == REAR_STAGE && MarkNo == 1)
	{
		Mark = m_RearANCMark2;
	}
	else
	{
		TRACE(_T("[PWR] INVALID GetANCMarkPosition ANC Base:%d MarkNo:%d\n"), Base, MarkNo);
	}

	return Mark;
}


void CPowerCalibrationData::SetANCZPosition(long Base, double zPos)
{
	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Base:%d Z %.3f\n"), strFunc, Base, zPos);

	if (fabs(zPos) > 200.0)
	{
		TRACE(_T("[PWR] Init SetANCZPosition Base:%d zPos:%.3f\n"), Base, zPos);
		zPos = 0.000;
	}

	if (Base == FRONT_STAGE)
	{
		m_FrontANCZPos = zPos;
	}
	else if (Base == REAR_STAGE)
	{
		m_RearANCZPos = zPos;
	}
	else
	{
		TRACE(_T("[PWR] INVALID SetANCZPosition Base:%d zPos:%.3f\n"), Base, zPos);
	}
}

double CPowerCalibrationData::GetANCZPosition(long Base)
{
	if (Base == FRONT_STAGE)
	{
		return m_FrontANCZPos;
	}
	else if (Base == REAR_STAGE)
	{
		return m_RearANCZPos;
	}
	else
	{

	}

	return 0.0;
}



void CPowerCalibrationData::SetANCRPosition(long Base, long rPos)
{
	if (rPos != 0 && rPos != 1)
	{
		TRACE(_T("[PWR] Init SetANCZPosition Base:%d rPos:%d\n"), Base, rPos);
		rPos = 1;
	}

	if (Base == FRONT_STAGE)
	{
		m_FrontANCRPos = rPos;
	}
	else if (Base == REAR_STAGE)
	{
		m_RearANCRPos = rPos;
	}
	else
	{
		TRACE(_T("[PWR] INVALID SetANCZPosition Base:%d zPos:%d\n"), Base, rPos);
	}
}

long CPowerCalibrationData::GetANCRPosition(long Base)
{
	if (Base == FRONT_STAGE)
	{
		return m_FrontANCRPos;
	}
	else if (Base == REAR_STAGE)
	{
		return m_RearANCRPos;
	}
	else
	{

	}

	return 1;
}

bool CPowerCalibrationData::InitANCHoleCadPosition()
{
	long hole;

	m_ANCHoleCadPosition[0][0].x = m_ANCHoleCadPosition[1][0].x = -52.500;
	m_ANCHoleCadPosition[0][1].x = m_ANCHoleCadPosition[1][1].x = -52.500 + 39.000;
	m_ANCHoleCadPosition[0][2].x = m_ANCHoleCadPosition[1][2].x = -52.500 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][3].x = m_ANCHoleCadPosition[1][3].x = -52.500 + 39.000 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][4].x = m_ANCHoleCadPosition[1][4].x = -52.500 + 39.000 + 39.000 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][5].x = m_ANCHoleCadPosition[1][5].x = -52.500 + 39.000 + 39.000 + 39.000 + 39.000 + 39.000;

	m_ANCHoleCadPosition[0][6].x = m_ANCHoleCadPosition[1][6].x = -52.500;
	m_ANCHoleCadPosition[0][7].x = m_ANCHoleCadPosition[1][7].x = -52.500 + 39.000;
	m_ANCHoleCadPosition[0][8].x = m_ANCHoleCadPosition[1][8].x = -52.500 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][9].x = m_ANCHoleCadPosition[1][9].x = -52.500 + 39.000 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][10].x = m_ANCHoleCadPosition[1][10].x = -52.500 + 39.000 + 39.000 + 39.000 + 39.000;
	m_ANCHoleCadPosition[0][11].x = m_ANCHoleCadPosition[1][11].x = -52.500 + 39.000 + 39.000 + 39.000 + 39.000 + 39.000;

	for (hole = 0; hole < 6; hole++) // 0 ~ 5
	{
		m_ANCHoleCadPosition[0][hole].y = m_ANCHoleCadPosition[1][hole].y = -29.000;
	}
	for (hole = 6; hole < MAX_ANC_HOLE; hole++) // 6 ~ 11
	{
		m_ANCHoleCadPosition[0][hole].y = m_ANCHoleCadPosition[1][hole].y = 29.000;
	}

	for (hole = 0; hole < MAX_ANC_HOLE; hole++)
	{
		m_ANCHoleCadPosition[0][hole].r = m_ANCHoleCadPosition[1][hole].r = 90.000;
		if (gcPowerCalibrationData->GetANCRPosition(FRONT_STAGE) == 0)
		{
			m_ANCHoleCadPosition[0][hole].r = 0.0;
		}
		else
		{
			m_ANCHoleCadPosition[0][hole].r = 90.0;
		}

		if (gcPowerCalibrationData->GetANCRPosition(REAR_STAGE) == 0)
		{
			m_ANCHoleCadPosition[1][hole].r = 0.000;
		}
		else
		{
			m_ANCHoleCadPosition[1][hole].r = 90.000;
		}
	}

	return true;
}


bool CPowerCalibrationData::CalculateANCHoleRealXYRZ(long Base)
{
	Point_XYRZ temp[MAX_ANC_HOLE];
	Point_XY ptMark1, ptMark2;
	double AngleRadian, AngleDegree, ca, sa;
	long hole;
	double zpos;

	if (Base == FRONT_STAGE)
	{
		zpos = m_FrontANCZPos;
	}
	else if (Base == REAR_STAGE)
	{
		zpos = m_RearANCZPos;
	}
	else
	{
		TRACE(_T("[PWR] CalculateANCHoleRealPosition Invalid Base:%d\n"), Base);
		return false;
	}

	ptMark1 = gcPowerCalibrationData->GetANCMarkPosition(Base, 0);
	ptMark2 = gcPowerCalibrationData->GetANCMarkPosition(Base, 1);

	//for (hole = 0; hole < MAX_ANC_HOLE; hole++)
	//{
	//	temp[hole].x = m_ANCHoleCadPosition[Base][hole].x + ptMark1.x;
	//	temp[hole].y = m_ANCHoleCadPosition[Base][hole].y + ptMark1.y;
	//}

	AngleRadian = atan2((ptMark2.y - ptMark1.y), (ptMark2.x - ptMark1.x));
	AngleDegree = A180_PIE * AngleRadian;

	if (fabs(AngleDegree) < 0.005)
	{
		AngleRadian = 0.000;
		AngleDegree = 0.000;
	}

	ca = cos(AngleRadian);
	sa = sin(AngleRadian);

	for (hole = 0; hole < MAX_ANC_HOLE; hole++)
	{
		temp[hole].x = ca * m_ANCHoleCadPosition[Base][hole].x - sa * m_ANCHoleCadPosition[Base][hole].y + ptMark1.x;
		temp[hole].y = sa * m_ANCHoleCadPosition[Base][hole].x + ca * m_ANCHoleCadPosition[Base][hole].y + ptMark1.y;
		temp[hole].r = m_ANCHoleCadPosition[Base][hole].r + AngleDegree;
		temp[hole].z = zpos;

		m_ANCHoleRealPosition[Base][hole] = temp[hole];

		TRACE(_T("[PWR] CalculateANCHoleRealXYR Base:%d Hole:%2d XYRZ:%.3f, %.3f, %.3f, %.3f\n"),
			Base, hole,
			m_ANCHoleRealPosition[Base][hole].x, m_ANCHoleRealPosition[Base][hole].y, m_ANCHoleRealPosition[Base][hole].r, m_ANCHoleRealPosition[Base][hole].z);
	}

	return true;
}

bool CPowerCalibrationData::EditANCHoleRealZPosition(long Base)
{
	if (Base == FRONT_STAGE)
	{
		TRACE(_T("[PWR] EditANCHoleRealZPosition Front Z:%.3f -> %.3f,%.3f\n"), m_ANCHoleRealPosition[Base][0].z, m_FrontANCZPos);

		for (long hole = 0; hole < MAX_ANC_HOLE; hole++)
		{

			m_ANCHoleRealPosition[Base][hole].z = m_FrontANCZPos;
		}
	}
	else if (Base == REAR_STAGE)
	{
		TRACE(_T("[PWR] EditANCHoleRealZPosition Rear Z:%.3f -> %.3f,%.3f\n"), m_ANCHoleRealPosition[Base][0].z, m_FrontANCZPos);

		for (long hole = 0; hole < MAX_ANC_HOLE; hole++)
		{
			m_ANCHoleRealPosition[Base][hole].z = m_RearANCZPos;
		}
	}
	else
	{
		return false;
	}

	return true;
}

bool CPowerCalibrationData::GetANCHoleRealPosition(long Base, long HoleNo, Point_XYRZ* ptHole)
{
	if (Base != FRONT_STAGE && Base != REAR_STAGE)
	{
		return false;
	}

	if (HoleNo < 0 || HoleNo > MAX_ANC_HOLE)
	{
		return false;
	}

	*ptHole = m_ANCHoleRealPosition[Base][HoleNo];


	return true;
}


bool CPowerCalibrationData::GetANCHoleRealPosition(long NozzleNo, Point_XYRZ* ptHole)
{
	Point_XYRZ pt;

	pt.x = pt.y = pt.r = pt.z = 0.000;

	if (1 <= NozzleNo && NozzleNo <= MAX_ANC_HOLE)
	{
		*ptHole = m_ANCHoleRealPosition[FRONT_STAGE][NozzleNo - 1];
	}
	else if (REAR_ANC_1ST <= NozzleNo && NozzleNo <= (REAR_ANC_1ST + MAX_ANC_HOLE - 1))
	{
		*ptHole = m_ANCHoleRealPosition[REAR_STAGE][NozzleNo - REAR_ANC_1ST];
	}
	else
	{
		TRACE(_T("[PWR] Invalid GetANCHoleRealPosition Hold:%d"), NozzleNo);
		return false;
	}

	return true;
}

bool CPowerCalibrationData::SetANCHoleRealPosition(long NozzleNo, Point_XYRZ ptHole)
{
	Point_XYRZ pt;

	CString strFunc(__func__);
	TRACE(_T("[PWR] %s Nzl:%d %.3f,%.3f,%.3f,%.3f\n"), strFunc, NozzleNo, ptHole.x, ptHole.y, ptHole.r, ptHole.z);

	pt.x = pt.y = pt.r = pt.z = 0.000;

	if (1 <= NozzleNo && NozzleNo <= MAX_ANC_HOLE)
	{
		m_ANCHoleRealPosition[FRONT_STAGE][NozzleNo - 1] = ptHole;
	}
	else if (REAR_ANC_1ST <= NozzleNo && NozzleNo <= (REAR_ANC_1ST + MAX_ANC_HOLE - 1))
	{
		m_ANCHoleRealPosition[REAR_STAGE][NozzleNo - REAR_ANC_1ST] = ptHole;
	}
	else
	{
		TRACE(_T("[PWR] Invalid GetANCHoleRealPosition Hold:%d"), NozzleNo);
		return false;
	}

	return true;
}

bool CPowerCalibrationData::WriteANCXYRZ()
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlock0();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

bool CPowerCalibrationData::CalcRecogOffsetRefMarkCompen(long Gantry, long CameraBase, Point_XY MarkPos, Point_XYRE MarkResult)
{
	Point_XY RecogPosition[MAXUSEDHEADNO];
	Point_XY RecogOffset[MAXUSEDHEADNO];

	if (CameraBase == FRONT_STAGE)
	{
		CopyMemory(RecogPosition, m_FrontCameraRecogPosition, sizeof(RecogPosition));
		CopyMemory(RecogOffset, m_FrontCameraRecogPosition, sizeof(RecogOffset));
	}
	else
	{
		CopyMemory(RecogPosition, m_RearCameraRecogPosition, sizeof(RecogPosition));
		CopyMemory(RecogOffset, m_RearCameraRecogPosition, sizeof(RecogOffset));
	}


	Point_XY tempOffset[MAXUSEDHEADNO];
	double shoesThickness = 226.0;
	double mmPerShift;
	double headPos;
	double shiftY;

	if (abs(MarkPos.x + shoesThickness) < EPSILON)
	{
		TRACE(_T("[PWR] Invalid MarkPos X is zero"));
		return false;
	}

	mmPerShift = MarkResult.y / (MarkPos.x + shoesThickness);

	TRACE(_T("[PWR] RecogMarkCompen Gantry:%d CamBase:%d %.6f(shift/mm)"), Gantry, CameraBase, mmPerShift);

	for (long head = 0; head < MAXUSEDHEADNO; head++)
	{
		headPos = shoesThickness + RecogPosition[head].x - GetHeadOffset(Gantry, head + 1).x;
		shiftY = headPos * mmPerShift;

		tempOffset[head].x = MarkResult.x;
		tempOffset[head].y = shiftY;

		TRACE(_T("[PWR] RecogMarkCompen Gantry:%d CamBase:%d Head:%d Compen:%.3f,%.3f"), Gantry, CameraBase, head, tempOffset[head].x, tempOffset[head].y);
	}

	if (CameraBase == FRONT_STAGE)
	{
		CopyMemory(m_FrontCameraRecogOffsetAddMarkCompen, tempOffset, sizeof(m_FrontCameraRecogOffsetAddMarkCompen));
	}
	else
	{
		CopyMemory(m_RearCameraRecogOffsetAddMarkCompen, tempOffset, sizeof(m_RearCameraRecogOffsetAddMarkCompen));
	}

	//for (long head = 0; head < MAXUSEDHEADNO; head++)
	//{
	//	shiftY = GetHeadOffset(Gantry, head + 1).x * mmPerShift;

	//	tempOffset[head].x = GetHeadOffset(Gantry, head + 1).x;
	//	tempOffset[head].y = GetHeadOffset(Gantry, head + 1).y + shiftY;
	//	TRACE(_T("[PWR] RecogMarkCompen Gantry:%d Head:%d HeadOffset1:%.3f,%.3f"), Gantry, CameraBase, head, tempOffset[head].x, tempOffset[head].y);
	//}

	//for (long head = 0; head < MAXUSEDHEADNO; head++)
	//{
	//	shiftY = GetHeadOffset(Gantry, head + 1).x * mmPerShift;

	//	tempOffset[head].x = GetHeadOffset(Gantry, head + 1).x;
	//	tempOffset[head].y = GetHeadOffset(Gantry, head + 1).y - shiftY;
	//	TRACE(_T("[PWR] RecogMarkCompen Gantry:%d Head:%d HeadOffset2:%.3f,%.3f"), Gantry, CameraBase, head, tempOffset[head].x, tempOffset[head].y);
	//}

	return true;

}

Point_XY CPowerCalibrationData::GetCameraRecognitionOffsetRefMarkCompen(long Gantry, long ZAxisNo)
{
	Point_XY RetOffset;
	ZeroMemory(&RetOffset, sizeof(RetOffset));

	if (ZAxisNo >= TBL_HEAD1 && ZAxisNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			RetOffset.x = m_FrontCameraRecogOffset[ZAxisNo - 1].x - m_FrontCameraRecogOffsetAddMarkCompen[ZAxisNo - 1].x;
			RetOffset.y = m_FrontCameraRecogOffset[ZAxisNo - 1].y - m_FrontCameraRecogOffsetAddMarkCompen[ZAxisNo - 1].y;

			TRACE(_T("[PWR] GetCameraRecognitionOffsetRefMarkCompen %s ZAxisNo(%d) Compen (%.3f %.3f -> %.3f %.3f)\n"),
				Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo,
				m_FrontCameraRecogOffset[ZAxisNo - 1].x, m_FrontCameraRecogOffset[ZAxisNo - 1].y, RetOffset.x, RetOffset.y);			
		}
		else if (Gantry == REAR_GANTRY)
		{
			RetOffset.x = m_RearCameraRecogOffset[ZAxisNo - 1].x - m_RearCameraRecogOffsetAddMarkCompen[ZAxisNo - 1].x;
			RetOffset.y = m_RearCameraRecogOffset[ZAxisNo - 1].y - m_RearCameraRecogOffsetAddMarkCompen[ZAxisNo - 1].y;

			TRACE(_T("[PWR] GetCameraRecognitionOffsetRefMarkCompen %s ZAxisNo(%d) Compen (%.3f %.3f -> %.3f %.3f)\n"),
				Gantry == FRONT_GANTRY ? _T("Front") : _T("Rear"), ZAxisNo,
				m_RearCameraRecogOffset[ZAxisNo - 1].x, m_RearCameraRecogOffset[ZAxisNo - 1].y, RetOffset.x, RetOffset.y);
		}
		else
		{
			TRACE(_T("[PWR] GetCameraRecognitionOffsetRefMarkCompen INVALID Gantry(%d) \n"), Gantry);
		}

		if (abs(RetOffset.x) > MAX_MOTION_VALID_RANGE)
		{
			TRACE(_T("[PWR] GetCameraRecognitionOffsetRefMarkCompen INVALID offsetX(%.3f) \n"), RetOffset.x);
			RetOffset.x = 0.000;

		}
		if (abs(RetOffset.y) > MAX_MOTION_VALID_RANGE)
		{
			TRACE(_T("[PWR] GetCameraRecognitionOffsetRefMarkCompen INVALID offsetX(%.3f) \n"), RetOffset.y);
			RetOffset.y = 0.000;
		}
	}
	else
	{
		TRACE(_T("[PWR] GetCameraRecognitionOffset Invalid AxisNo:%d\n"), ZAxisNo);
	}
	return RetOffset;
}


void CPowerCalibrationData::SetInsertOffset4532(long CameraBase, long HeadNo, long DegreeIndex, Point_XY Offset)
{
	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (0 <= DegreeIndex && DegreeIndex < MAX_DEGREE_INDEX)
		{
			if (CameraBase == FRONT_STAGE)
			{
				m_InsertOffset4532Front[HeadNo - 1][DegreeIndex] = Offset;
				TRACE(_T("[PWR] SetInsertOffset4532 Base %d Head %d Degree %d Offset %.3f %.3f\n"),
					CameraBase, HeadNo, DegreeIndex, m_InsertOffset4532Front[HeadNo - 1][DegreeIndex].x, m_InsertOffset4532Front[HeadNo - 1][DegreeIndex].y);

			}
			else
			{
				m_InsertOffset4532Rear[HeadNo - 1][DegreeIndex] = Offset;
				TRACE(_T("[PWR] SetInsertOffset4532 Base %d Head %d Degree %d Offset %.3f %.3f\n"),
					CameraBase, HeadNo, DegreeIndex, m_InsertOffset4532Rear[HeadNo - 1][DegreeIndex].x, m_InsertOffset4532Rear[HeadNo - 1][DegreeIndex].y);

			}
		}
	}
}

Point_XY CPowerCalibrationData::GetInsertOffset4532(long CameraBase, long HeadNo, long DegreeIndex)
{
	Point_XY offset;
	offset.x = offset.y = 0.000;

	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (0 <= DegreeIndex && DegreeIndex < MAX_DEGREE_INDEX)
		{
			if (CameraBase == FRONT_STAGE)
			{
				offset = m_InsertOffset4532Front[HeadNo - 1][DegreeIndex];
			}
			else
			{
				offset = m_InsertOffset4532Rear[HeadNo - 1][DegreeIndex];
			}
		}
	}

	return offset;
}