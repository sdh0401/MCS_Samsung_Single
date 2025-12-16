#pragma once
#include "GlobalDefine.h"

class CPowerCalibrationData
{
public:
	CPowerCalibrationData();
	~CPowerCalibrationData();
	void SetYHomeEncoderDifferent(long Diff);
	long GetYHomeEncoderDifferent();
	void Load1DStartCompensationData(long Gantry, double CompenData);
	double Get1DStartCompensationData(long Gantry);
	void Set1DStartCompensationData(long Gantry, double StartCompenData);
	void Load1DCompensationData(long Gantry, long indx, double value);
	double Get1DCompensationData(long Gantry, long indx);
	void Set1DCompensationData(long Gantry, long indx, double value);
	bool Write1DCompensationData(long Gantry);
	void LoadAgingPosition(long Gantry, long index, Point_XYRZE AgingPosition);
	Point_XYRZE GetAgingPosition(long Gantry, long index);
	bool WriteAgingPosition(long Gantry);
	void SetAgingPosition(long Gantry, long index, Point_XYRZE pt);

	void LoadHomeShiftDistance(long Gantry, double shiftDistance);
	void SetHomeShiftDistance(long Gantry, double shiftDistance);
	double GetHomeShiftDistance(long Gantry);
	bool WriteHomeShiftDistance(long Gantry);

	void Load2DStartPosition(long Gantry, Point_XY Start);
	Point_XY Get2DStartPosition(long Gantry);
	void Set2DStartPosition(long Gantry, Point_XY Start);
	void Load2DCompensationData(long Gantry, long indx, Point_XYRE CompenData);
	Point_XYRE Get2DCompensationData(long Gantry, long indx);
	void Set2DCompensationData(long Gantry, long indx, Point_XYRE CompenData);
	bool Write2DCompensationData(long Gantry);

	void LoadCameraAlignPosition(long Gantry, Point_XY AlignPosition);
	void SetCameraAlignPosition(long Gantry, Point_XY AlignPosition);
	Point_XY GetCameraAlignPosition(long Gantry);
	void LoadAlignCamera(long Gantry, long Camera);
	long GetAlignCamera(long Gantry);
	void SetAlignCamera(long Gantry, long Camera);
	bool WriteCameraAlignPosition(long Gantry);

	void LoadUseAxis(long AxisNo, bool bUse);
	void SetUseAxis(long AxisNo, bool bUse); 
	bool GetUseAxis(long AxisNo);
	bool WriteUseAxis(long Gantry);

	void LoadLimit(long AxisNo, Limit limit);
	void SetLimit(long AxisNo, Limit limit);
	Limit GetLimit(long AxisNo);
	bool WriteLimit(long Gantry);

	void LoadHomePosition(long AxisNo, double OriginOffset);
	void SetHomePosition(long AxisNo, double OriginOffset);
	double GetHomePosition(long AxisNo);
	bool WriteHomePosition(long Gantry);

	void LoadHeadOffset(long Gantry, long ZAxisNo, Point_XY HeadOffset);
	void SetHeadOffset(long Gantry, long ZAxisNo, Point_XY HeadOffset);
	Point_XY GetHeadOffset(long Gantry, long ZAxisNo);
	bool WriteHeadOffset(long Gantry);

	void LoadHMOffset(long Gantry, Point_XY HMOffset);
	void SetHMOffset(long Gantry, Point_XY HMOffset);
	Point_XY GetHMOffset(long Gantry);
	bool WriteHMOffset(long Gantry);

	void SetHMZero(long Gantry, long zerolevel);
	long GetHMZero(long Gantry);

	void LoadCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY RecogPosition);
	void SetCameraRecognitionPosition(long Gantry, long ZAxisNo, Point_XY RecogPosition);
	Point_XY GetCameraRecognitionPosition(long Gantry, long ZAxisNo);
	bool WriteCameraRecognitionPosition(long Gantry);
	void LoadCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY RecogPosition);
	void SetCameraRecognitionOffset(long Gantry, long ZAxisNo, Point_XY RecogPosition);
	Point_XY GetCameraRecognitionOffset(long Gantry, long ZAxisNo);
	bool WriteCameraRecognitionOffset(long Gantry);

	void LoadReferenceFeederPosition(long Stage, Point_XY RefFdPos);
	Point_XY GetReferenceFeederPosition(long Stage);
	void SetReferenceFeederPosition(long Stage, Point_XY RefFdPos);
	void LoadReferenceFeederNo(long Stage, long RefFdNo);
	long GetReferenceFeederNo(long Stage);
	void SetReferenceFeederNo(long Stage, long RefFdNo);
	void LoadFeederPitch(long Stage, double Pitch);
	double GetFeederPitch(long Stage);
	void SetFeederPitch(long Stage, double Pitch);
	bool WriteReferenceFeeder(long Stage);

	void LoadPcbFixPosition(long Conveyor, Point_XY PcbFixPos);
	Point_XY GetPcbFixPosition(long Conveyor);
	void SetPcbFixPosition(long Conveyor, Point_XY PcbFixPos);
	bool WritePcbFixPosition(long Conveyor);
	
	void LoadZCompensationData(long Gantry, long HeadNo, long indx, double CompenData);
	double GetZCompensationData(long Gantry, long HeadNo, long indx);
	void SetZCompensationData(long Gantry, long HeadNo, long indx, double CompenData);
	bool WriteZCompensationData(long Gantry);
	void LoadZCompensationStartData(long Gantry, long HeadNo, double CompenStartData);
	double GetZCompensationStartData(long Gantry, long HeadNo);
	void SetZCompensationStartData(long Gantry, long HeadNo, double CompenStartData);

	void LoadSensorOriginHeight(long Gantry, double SensorHeight);
	double GetSensorOriginHeight(long Gantry);
	void SetSensorOriginHeight(long Gantry, double PcbFixPos);
	bool WriteSensorOriginHeight(long Gantry);

	void SetANCMarkPosition(long Base, long MarkNo, Point_XY Mark);
	Point_XY GetANCMarkPosition(long Base, long MarkNo);
	void SetANCZPosition(long Base, double zPos);
	double GetANCZPosition(long Base);
	void SetANCRPosition(long Base, long rPos);
	long GetANCRPosition(long Base);

	bool InitANCHoleCadPosition();
	bool CalculateANCHoleRealXYRZ(long Base);
	bool EditANCHoleRealZPosition(long Base);
	bool GetANCHoleRealPosition(long Base, long HoleNo, Point_XYRZ* ptHole);
	bool GetANCHoleRealPosition(long NozzleNo, Point_XYRZ* ptHole);
	bool SetANCHoleRealPosition(long NozzleNo, Point_XYRZ ptHole);
	bool WriteANCXYRZ();

	bool CalcRecogOffsetRefMarkCompen(long Gantry, long CameraBase, Point_XY MarkPos, Point_XYRE MarkResult);
	Point_XY GetCameraRecognitionOffsetRefMarkCompen(long Gantry, long ZAxisNo);

	void SetInsertOffset4532(long CameraBase, long HeadNo, long DegreeIndex, Point_XY Offset);
	Point_XY GetInsertOffset4532(long CameraBase, long HeadNo, long DegreeIndex);

private:
	double m_FrontGantry1DStart;
	double m_RearGantry1DStart;
	double	m_FrontGantry1D[BUFSIZE];
	double	m_RearGantry1D[BUFSIZE];
	Point_XYRZE m_FrontAgingPoint[BUFSIZE];
	Point_XYRZE m_RearAgingPoint[BUFSIZE];
	double m_FrontHomeShiftDistance;
	double m_RearHomeShiftDistance;

	Point_XY m_FrontGantry2DStart;
	Point_XY m_RearGantry2DStart;

	Point_XYRE m_FrontGantry2D[CAL_2D_MAXCOUNT * CAL_2D_MAXCOUNT];
	Point_XYRE m_RearGantry2D[CAL_2D_MAXCOUNT * CAL_2D_MAXCOUNT];

	long m_FrontAlignCamera;
	long m_RearAlignCamera;
	Point_XY m_FrontCameraAlignPosition;
	Point_XY m_RearCameraAlignPosition;

	bool m_bUseAxis[MAXAXISNO];
	Point_XY m_FrontHeadOffset[MAXUSEDHEADNO];
	Point_XY m_RearHeadOffset[MAXUSEDHEADNO];
	Point_XY m_FrontZHMDOffset;
	Point_XY m_RearZHMDOffset;
	long m_FrontHMZero;
	long m_RearHMZero;
	Point_XY m_FrontCameraRecogPosition[MAXUSEDHEADNO];
	Point_XY m_RearCameraRecogPosition[MAXUSEDHEADNO];
	Point_XY m_FrontCameraRecogOffset[MAXUSEDHEADNO];
	Point_XY m_RearCameraRecogOffset[MAXUSEDHEADNO];
	Point_XY m_FrontCameraRecogOffsetAddMarkCompen[MAXUSEDHEADNO];
	Point_XY m_RearCameraRecogOffsetAddMarkCompen[MAXUSEDHEADNO];
	double m_HomePosition[MAXAXISNO];
	Limit m_Limit[MAXAXISNO];
	
	Point_XY m_FrontPCBFixPosition;
	Point_XY m_RearPCBFixPosition;
	long m_FrontReferenceFeederNo;
	Point_XY m_FrontReferenceFeederPosition;
	long m_RearReferenceFeederNo;
	Point_XY m_RearReferenceFeederPosition;
	double m_FrontFeederPitch;
	double m_RearFeederPitch;

	double m_FrontCompenStartZ[MAXUSEDHEADNO];
	double m_RearCompenStartZ[MAXUSEDHEADNO];
	double m_FrontCompenZ[MAXUSEDHEADNO][BUFSIZE];
	double m_RearCompenZ[MAXUSEDHEADNO][BUFSIZE];

	double m_FrontSensorOriginHeight;
	double m_RearSensorOriginHeight;

	Point_XY m_FrontANCMark1;
	Point_XY m_FrontANCMark2;
	Point_XY m_RearANCMark1;
	Point_XY m_RearANCMark2;

	double m_FrontANCZPos;
	double m_RearANCZPos;

	Point_XYRZ m_ANCHoleCadPosition[MAX_ANC_BASE][MAX_ANC_HOLE];
	Point_XYRZ m_ANCHoleRealPosition[MAX_ANC_BASE][MAX_ANC_HOLE];

	long m_FrontANCRPos;
	long m_RearANCRPos;

	long m_EncoderDiffrent;

	Point_XY m_InsertOffset4532Front[MAXUSEDHEADNO][MAX_DEGREE_INDEX];
	Point_XY m_InsertOffset4532Rear[MAXUSEDHEADNO][MAX_DEGREE_INDEX];
};

extern CPowerCalibrationData* gcPowerCalibrationData;
