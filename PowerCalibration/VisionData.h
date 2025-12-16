#pragma once
#include "GlobalDefine.h"

#define	WRITE_BUFFSIZE		256 //command buffer size
#define	READ_BUFFSIZE		256 //command buffer size
#define	EOP					-1	// End of package
#define NEWMAXCAMNO			21  // PC Vision Max Camera Number
#define	PCODDDB_START		250		/* 150 ~ 169 */
#define MaxSystemMarkDB		20
#define PCVARECOGNUM		16
#define MAXQPVAHEAD			8
#define MAXFLEXNO			2
#define MAXHEADPERCAM		8
#define BASE3DDOT			5
#define SND_ETHERNET		1
#define MAXVISQUENO			30
#define	MAXVISMSGLENGTH		1024 // sizeof(COMMAND_DPRAM) * WRITE_BUFFSIZE
#define ZIGXMARGIN			50   //ROI 영역 x축 여백
#define ZIGYMARGIN			15   //ROI 영역 y축 여백
#define XGAWIDTH			1024
#define XGAHEIGHT			768
#define VGAWIDTH			648
#define VGAHEIGHT			494
#define SXGAWIDTH			1360
#define SXGAHEIGHT			1040
#define SXGA2WIDTH			1280			// 1.2MB
#define SXGA2HEIGHT			960				// 1.2MB
#define UXGAWIDTH			1600			// 2MB
#define UXGAHEIGHT			1200			// 2MB
#define BS4MEGAWIDTH		2048			// Balser acA2040-25gm 4MB
#define BS4MEGAHEIGHT		2048			// Basler acA2040-25gm 4MB
#define T5MEGAWIDTH			2592			// 5MB
#define T5MEGAHEIGHT		1944			// 5MB
#define	NO_USE_REVERSECHECK	-1

#define CHKETHERTMAXTIME	1200000 //20분 마다 vision server에 command 15(version 알아오는 것) 날린다
#define MAXEPNHOSTMSGLENG 	1024

/*
	structure
*/

////////////////////////////////////////////////////////////////////////////
typedef struct {
	int CamNo;
	float CamCoeff[7];
	float TranMtx[8];
}VisCalibDataInfo;

typedef struct {
	int CamNo;
	double CamCalCoeff[32];
}VisCalibDataInfoNew;

typedef struct {
	int DBNo;
	int Type;
	int Color;
	int Win[2][2];
	int Area;
}VisMarkDataInfo;

typedef struct {
	int DBNo;
	int Type;
	int Color;
	int Win[2][2];
	int Area;
	int InspectMethod;			/* 0=Old(Area), 1=New */
	float d1, d2, w1, w2, a1, a2;
	float angle;
}VisMarkDataInfoNew;

typedef struct {						//Fiducial Mark Data???
	int DBNo;
	int Type;
	int Color;
	int Win[2][2];
	int Area;
	int InspectMethod;			/* 0=Old(Area), 1=New */
	float d1, d2, w1, w2, a1, a2;
	float angle;
	int Tolerance;
	int Reserved[3];
}VisMarkDataInfoNewTwo;

typedef struct {						//26. Simple Component DB
	int DBNo;
	int PartType, LeadType;
	double SizeX, SizeY;
	int RecogOption;
	int Lead[4];					/* 0=Left,1=Right,2=Up,3=Down */
	float Length[4], Width[4], Pitch[4], Foot[4];
	int dwFilterOption;
	int dwReserved[10];
	float flROIOption;
	float flReserved[10];
}VisVADataInfoNew; //20180912 yscho

typedef struct {						//26. Simple Component DB
	int DBNo;
	int PartType, LeadType;
	float SizeX, SizeY;
	int RecogOption;
	int Lead[4];					/* 0=Left,1=Right,2=Up,3=Down */
	float Length[4], Width[4], Pitch[4], Foot[4];
}VisVADataInfo;

typedef struct {						//Odd Component DB Data???
	int DBNo;
	int PartType, LeadType;
	double SizeX, SizeY;
	int RecogOption;
	int NoGroup[4];				/* 0=Left,1=Right,2=Up,3=Down */
	float GroupPitch[4][3];			/* 4Side*3Group, 0=Left, 1=Right, 2=Up, 3=Down */
	int LeadNo[4][4];				/* 4Side*4Group, 0=Left, 1=Right, 2=Up, 3=Down */
	float Length[4][4];				/* 4Side*4Group, 0=Left, 1=Right, 2=Up, 3=Down */
	float Width[4][4];				/* 4Side*4Group, 0=Left, 1=Right, 2=Up, 3=Down */
	float Pitch[4][4];				/* 4Side*4Group, 0=Left, 1=Right, 2=Up, 3=Down */
	float Foot[4][4];				/* 4Side*4Group, 0=Left, 1=Right, 2=Up, 3=Down */
}VisOddVADataInfo;

typedef struct {						//BGA Component DB Data???
	int DBNo;
	int PartType, LeadType;
	double SizeX, SizeY;
	int RecogOption;
	int NoBall[2];				/* No of Balls */
	float BallDimension[4];			/* Radius,PitchX,Y,Height */
	int BallPitchTolerance[2];	/* X,Y Tolerence */
	int GridType;
	float GridDimension[3];			/* Angle,X,Y */
	int GridTolerance[3];			/* Angle,X,Y */
	int MissingBallFlag;
	int BallArray[32];
	int BallExtend[18];
	int Reserved[30];
}VisBGAVADataInfo;

typedef struct {					/* Pattern DB */
	int 	DBNo;
	int 	PartType, LeadType;
	double 	SizeX, SizeY;
	UDWORD 	RecogOption;
	double	CenterX, CenterY, CenterR;
	UDWORD	SizeOfDB;
	int 	Lead[4];				/* 0=Left, 1=Right, 2=Up, 3=Down */
	float 	Length[4], Width[4], Pitch[4], Foot[4];
	UDWORD	Features[72];			/* Pattern type features */
}VisPATVADataInfo;

typedef struct {					/* Pattern DB */
	UDWORD	Features[4096];
}VisFeaVADataInfo;

typedef struct {
	int CamNo;
	Point_XY Left, Right, CenterL, CenterR;
}VAOffsetDataInfo;
////////////////////////////////////////////////////////////////////////////

typedef struct {
	int CamNo;
	Point_XY Left, Right, CenterL, CenterR;
	Point_XY CenterExt1, CenterExt2;
	Point_XY Reserved1;
	Point_XY Reserved2;
	Point_XY Reserved3;
	Point_XY Reserved4;
	Point_XY DownSide[MAXHEADPERCAM];
	Point_XY Reserved5;
	Point_XY Reserved6;
	Point_XY DownCenter[MAXHEADPERCAM];
	Point_XY Reserved7;
	Point_XY Reserved8;
	Point_XY UpSide[MAXHEADPERCAM];
	Point_XY Reserved9;
	Point_XY Reserved10;
	Point_XY UpCenter[MAXHEADPERCAM];
	Point_XY Reserved11;
	Point_XY Reserved12;
}VAOffsetDataInfoNew;

typedef union COMMANDINFO {
	double  _dbl;
	long    _long;
}COMMAND_DPRAM;

extern int CamNoBySystem[3][MAXCAMNO][3];

typedef struct {
	struct __ETHERNET__VERSION {
		struct __MVL__VERSION {
			long	Major;
			long	Minor;
			long   Build;
			long	Patch;
		} MVL;
		struct __MVT__VERSION {
			long	Major;
			long	Minor;
			long   Build;
			long	Patch;
			long	Resol;
		} MVT;
		struct __LASER_VERSION {
			long	Major;
			long	Minor;
			long	Build;
			long	Patch;
		} LV;
	} Eth_Ver;
	struct __DPRAM__VERSION {
		long	Major;
		long	Minor;
		long	Major_Build;
		long	Minor_Build;
		long	Patch;
	} Vme_Ver;
	long Core;						// VisionVersion : 1866
	long Core1;
	long Core2;
	long Core3;
	long Core4;
	long Core5;
	long Grabber;
} VisionVersionInfo;

extern long SetMarkCameraCalibrationJigCenterMark(long cam);
extern void LoadMark(long MarkNo, MarkInfo mark);
extern void SetMark(long MarkNo, MarkInfo mark);
extern MarkInfo GetMark(long MarkNo);
extern void markInitialize(long MarkNo, double RoiRatioW, double RoiRatioH, long MarkType, long MarkColor, WindowSize Win);
extern long markTraining(int cam, int uwfileno, int calmark, Point_XYRE* pRes);
extern Point_XYRE markTrainingWithoutCamCal(int cam, int uwfileno, int calmark, int showrst, int useCal);
extern Point_XYRE GetVisionResult(unsigned bdno, unsigned index);
extern Point_XYRE GetVisionPartResult(long bdno);
extern int CameraCalibration(int cam, unsigned MaxCalJigMarkNo, Point_XY* calpos);
extern long GetVisionResultLong(unsigned bdno, unsigned indx);
extern double GetVisionResultDouble(unsigned bdno, unsigned indx);
extern int CameraRotateCalibration(long cam, long Catch);
extern int CameraRotateCalibrationPrepare(long cam, double JigPitchX);
extern long gLiveOn(long cam);
extern long gLiveOff(long cam);
extern Point_XYRE gCatchMachCalMark(int CameraNo, long MarkNo);
extern Point_XYRE gCatchMachRefMark(int CameraNo, long MarkNo);
extern Point_XYRE gCatchMark(int CameraNo, long MarkNo);
extern Point_XYRE ginspectROriginMark(int CameraNo, long MarkNo);
extern int gWindowMoving(int CameraNo, int Window, int Show, int Control, int pixel, int dir);
extern Point_XYRE markRecognitionRepeat(long CamNo, long file, long count, long delay);
extern long findRotateCenterWithRes(long Gantry, CString strZAXis, long CamNo, double* cx, double* cy, double* Res);
extern void Get3PointCircle(Point_XY* p, double* cx, double* cy, double* r);
extern void getCircleEstimate(long N, double* x, double* y, double* cx, double* cy, double* r);

extern long gPrepareCommand(long t, long* cam, long id, long* chk, long* useVA, long* dbNo, double* angleVA, long* Forming);
extern long gImageCatch(long t, long* cam, long id, long* chk, long* useVA, long uwDiv);
extern long gStartProcess(long cam, long id);
extern long gGetRecognitionResult(long cam, long loc, long id, long* Err, Point_XYRE* res, long dwFrameNo[], Point_XYT* ptXYRESize);
extern void gSetRunVisionResult(long Gantry, long InsertOrder, Point_XYRE Res);
extern Point_XYRE gGetRunVisionResult(long Gantry, long InsertOrder);
extern void gSetVisionTimeOut(bool bTimeOut);
extern bool gGetVisionTimeOut();
extern void gSetRunVisionErrorCode(long Gantry, long InsertOrder, long ErrorCode);
extern long gGetRunVisionErrorCode(long Gantry, long InsertOrder);
extern void gSetRunVisionAngle(long Gantry, long InsertOrder, double RecognitionAngle);
extern double gGetRunVisionAngle(long Gantry, long InsertOrder);
extern void gInitRunVisionAngle(long Gantry, long InsertOrder);
extern long gSendCameraRecognitionOffset(long cam, Point_XY CamOffset1, Point_XY CamOffset2, Point_XY CamOffset3);
extern long gInspectBarcode(long cam);
extern long gInspectBarcode(long cam, long BarcodeType);
extern long gMarkPairRecognition(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, Ratio_XYRZ Ratio);
extern long gGetMarkDelta(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, long Res);
extern Point_XYR gMarkCompensation(long Gantry, Point_XYR Pos, long Res);
extern Point_XYRZ gComponentCompensation(long Gantry, long ChkPos, Point_XYRZ Pos, long InsertOrd, long MarkRes);
extern Point_XYR gMarkLoss(long Gantry, Point_XYR* Pos, long Res);
extern void gUpdateVisionData(UWORD bdno, UWORD ret, UWORD data);
extern void gClearRunVisionResult(long bdno);
extern void gClearRunRecognitionAngle(long bdno);
extern void gShowRoiBase(long bdno, WindowSize Win);
extern long gGetVisionErrorCode(long bdno);
extern long gGetVisionCommandResult(long bdno);
extern long gGetMarkArea(long bdno, long MarkNo);
extern long gSetMarkLed(long MarkNo, OFFSET_LED Led);
extern void gSetPartRecognitionResult(long CamNo, long CamChk, Point_XYRE Res);
extern Point_XYRE gGetPartRecognitionResult(long CamNo, long CamChk);
extern long gDropCheckSaveImage(long* cam, long size);
extern long gDropCheckProcess(long* cam, long size, long* result);
extern long gGetVisionTable(long CamNo);
extern CString gGetCameraBarcode(long bdno);
extern Point_XYRE gGetPartRecognitionResult(long CamNo, long CamChk);
extern double gGetVisionPartPitchResult(long bdno);
extern void gSetRunVisionPitchResult(long Gantry, long InsertOrder, Point_XYT SizeResult);
extern Point_XYT gGetRunVisionPitchResult(long Gantry, long InsertOrder);