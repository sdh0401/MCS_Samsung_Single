#include "pch.h"
#include "VisionData.h"
#include "EthernetVision.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "Trace.h"
//#include "ErrorCode.h"
//------------------------------------------------------------------------------
int CamNoBySystem[3][MAXCAMNO][3] = { /*** Board,VisionCamID,FrameNo ***/
	/* Dual Vision  */
	{
		{0, 1,0},	//0 Front Module 1
		{0, 2,1},	//1 Front Module 2
		{0,99,2},	//2 Front Module 3
		{0,99,6},
		{1, 1,0},	//4 Rear Module 1
		{1, 2,1},	//5 Rear Module 2
		{1,99,2},	//6 Rear Module 3
		{1,99,6},
		{0, 4,3},	//8 Front Head
		{1, 4,3},	//9 Rear Head
		{1,99,33},	// Rear Left QFP
		{1,99,34},	// Rear Right QFP
		{0,99,33},	// Front Left QFP
		{0,99,34},	// Front Right QFP
		{0,99,0},
		{0,99,0},
		{0,99,0},
		{0,99,0},
		{0,99,0},
		{0,99,0}
	},
	/* Single Vision Two Camera */
	{
		{0, 1,0},	//0 FCAM1
		{0, 2,1},	//1 FCAM2
		{0,99,2},	//2 FCAM3
		{0,99,6},	//3 FCAM4
		{0, 5,33},	//4 RCAM1
		{0, 6,34},	//5 RCAM2
		{0,99,35},	//6 RCAM3
		{0,99,6},	//7 RCAM4
		{0, 4,3},	//8 FHEAD
		{0,99,36},	//9 RHEAD
		{0,99,36},	//10 Reserved
		{0,99,2},	//11 Reserved
		{0,99,35},	//12 
		{0,99,5},	//13
		{0,99,8},	//14
		{0,99,8},	//15
		{0,99,0},
		{0,99,0},
		{0,99,0},
		{0,99,0}
	},
	/* Single or Dual Vision Six Camera */
	{
		{0, 1,0},	//0 FCAM1
		{0, 2,1},	//1 FCAM2
		{0, 3,2},	//2 FCAM3
		{0, 5,3},	//3 FCAM4
		{0, 6,4},	//4 FCAM5
		{0, 7,5},	//5 FCAM6
		{0,99,6},	//6 Reserved
		{0,99,7},	//7 Reserved
		{0, 4,8},	//8 FHEAD
		{1, 4,9},	//9 RHEAD
		{1, 1,10},	//10 RCAM1
		{1, 2,11},	//11 RCAM2
		{1, 3,12},	//12 RCAM3
		{1, 5,13},	//13 RCAM4
		{1, 6,14},	//14 RCAM5
		{1, 7,15},	//15 RCAM6
		{0,99,16},  //16
		{0,99,17},  //17
		{0,99,18},  //18
		{0,99,19}   //19
	}
};

long SetMarkCameraCalibrationJigCenterMark(long cam)
{
	long MarkNo = 0;
	MarkNo = gcEthernetVision->SetMarkCameraCalibrationJigCenterMark(cam);
	return MarkNo;
}

void LoadMark(long MarkNo, MarkInfo mark)
{
	Mark[MarkNo] = mark;
}

void SetMark(long MarkNo, MarkInfo mark)
{
	Mark[MarkNo] = mark;
}

MarkInfo GetMark(long MarkNo)
{
	return Mark[MarkNo];
}

void markInitialize(long MarkNo, double RoiRatioW, double RoiRatioH, long MarkType, long MarkColor, WindowSize Win)
{
	Mark[MarkNo].Type = (UBYTE)MarkType; // Circle
	Mark[MarkNo].Color = (UBYTE)MarkColor; // White
	Mark[MarkNo].Win.x1 = Win.x1;
	Mark[MarkNo].Win.y1 = Win.y1;
	Mark[MarkNo].Win.x2 = Win.x2;
	Mark[MarkNo].Win.y2 = Win.y2;
	TRACE(_T("[PWR] MarkNo:%d Type:%d Color:%d Area:%d X1,Y1,X2,Y2(%d,%d,%d,%d)\n"), 
		MarkNo, Mark[MarkNo].Type, Mark[MarkNo].Color, Mark[MarkNo].Area,
		Mark[MarkNo].Win.x1, Mark[MarkNo].Win.y1, Mark[MarkNo].Win.x2, Mark[MarkNo].Win.y2);
}

long markTraining(int cam, int uwfileno, int calmark, Point_XYRE* pRes)
{
	long Ret = 0;
	if (gcEthernetVision)
	{
		Ret = gcEthernetVision->markTraining(cam, uwfileno, calmark, pRes);
	}
	return Ret;
}

Point_XYRE markTrainingWithoutCamCal(int cam, int uwfileno, int calmark, int showrst, int useCal)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		res = gcEthernetVision->markTrainingWithoutCamCal(cam, uwfileno, 0, 0, 1);
	}
	return res;
}

Point_XYRE GetVisionResult(unsigned bdno, unsigned index)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		res = gcEthernetVision->GetVisionMarkResult(bdno, 0);
	}
	return res;
}

Point_XYRE GetVisionPartResult(long bdno)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		res = gcEthernetVision->GetVisionPartResult(bdno);
	}
	return res;
}

int CameraCalibration(int cam, unsigned MaxCalJigMarkNo, Point_XY* calpos)
{
	int retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->CameraCalibration(cam, MaxCalJigMarkNo, calpos);
	}
	return retVis;
}

long GetVisionResultLong(unsigned bdno, unsigned indx)
{
	long Ret = 0;
	if (gcEthernetVision)
	{
		Ret = gcEthernetVision->GetVisionResultLong(bdno, indx);
	}
	return Ret;
}

double GetVisionResultDouble(unsigned bdno, unsigned indx)
{
	double Result = 0.0;
	if (gcEthernetVision)
	{
		Result = gcEthernetVision->GetVisionResultDouble(bdno, indx);
	}
	return Result;
}

int CameraRotateCalibration(long cam, long Catch)
{
	int retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->CameraRotateCalibration(cam, Catch);
	}
	return retVis;
}

int CameraRotateCalibrationPrepare(long cam, double JigPitchX)
{
	int retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->CameraRotateCalibrationPrepare(cam, JigPitchX);
	}
	return retVis;
}

long gLiveOn(long cam)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->LiveOn(cam);
	}
	return 0;
}

long gLiveOff(long cam)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->LiveOff(cam);
	}
	return 0;
}

Point_XYRE gCatchMachCalMark(int CameraNo, long MarkNo)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		gcEthernetVision->inspectMachCalMark(CameraNo, MarkNo, &res);
	}
	else
	{
		TRACE(_T("[PWR] CatchMachCalMark CameraNo:%d fail\n"), CameraNo);
	}
	return res;
}

Point_XYRE gCatchMachRefMark(int CameraNo, long MarkNo)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{		
		gcEthernetVision->inspectMachCalRefMark(CameraNo, MarkNo, &res);
	}
	else
	{
		TRACE(_T("[PWR] CatchMachRefMark CameraNo:%d fail\n"), CameraNo);
	}
	return res;
}

Point_XYRE gCatchMark(int CameraNo, long MarkNo)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		gcEthernetVision->inspectMark(CameraNo, MarkNo, &res);
	}
	else
	{
		TRACE(_T("[PWR] gCatchMark CameraNo:%d fail\n"), CameraNo);
	}
	return res;
}

Point_XYRE ginspectROriginMark(int CameraNo, long MarkNo)
{
	Point_XYRE res;
	ZeroMemory(&res, sizeof(res));
	if (gcEthernetVision)
	{
		gcEthernetVision->inspectROriginMark(CameraNo, MarkNo, &res);
	}
	else
	{
		TRACE(_T("[PWR] ginspectROriginMark CameraNo:%d fail\n"), CameraNo);
	}
	return res;
}

long GetCameraToGantry(long CamNo)
{
	long Gantry = FRONT_GANTRY;
	return Gantry;
}

int gWindowMoving(int CameraNo, int Window, int Show, int Control, int pixel, int dir)
{
	/*	
	win		: 0=All, 1=1st Win, 2=2nd Win
	Show	: 0=Hide, 1=Show
	cor		: 0=LeftTop, 1=RightDown, 2=Center, 3=NO Select
	pixel	: 0=0, 1=1, 2=5, 3=20, 4=50
	dir		: 0=Left, 1=Right, 2=Up, 3=Down	
	*/
	int retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->WindowMoving(CameraNo, Window, Show, Control, pixel, dir);
	}
	TRACE(_T("[PWR] WindowMoving Ret:%d\n"), retVis);
	return retVis;
}

Point_XYRE markRecognitionRepeat(long CamNo, long file, long count, long delay)
{
	/* cam_no : 0~8 */
	long inx, successCount;
	double x, y;
	Point_XYRE res;
	successCount = 0;
	x = 0.00;
	y = 0.00;
	gLedOn(CamNo, 60, 0, 0);
	for (inx = 0; inx < count; inx++)
	{
		if (gcEthernetVision)
		{
			gcEthernetVision->prepareMarkRecognition(CamNo, file, 0);
		}
		ThreadSleep(delay);
		res = gCatchMark(CamNo, file);
		if(res.exe == 1)
		{
			x += res.x;
			y += res.y;
			successCount++;
		}
	}
	if (successCount == 0)
	{
		res.x = res.y = res.r = 0.000;
		res.exe = 0;
		return res;
	}
	else 
	{
		res.x = x / successCount;
		res.y = y / successCount;
		res.exe = 1;
		return res;
	}
}

// 3점을 지나는 원의 원점과 반지름을 구한다.
// P1=P[0]과P[1]의 중심, P2=P[0]과 P[2]의 중심
// P1에서 P[0]와 P[1]이 이루는 선분의 직교과 P2에서 P[0]와 P[2]가 이루는 선분의 직교가 만나는 점이 Center
void Get3PointCircle(Point_XY* p, double* cx, double* cy, double* r) {
	double dy1, dy2, d, d2, yi;
	Point_XY p1, p2;

	p1.x = (p[0].x + p[1].x) / 2;
	p1.y = (p[0].y + p[1].y) / 2;
	p2.x = (p[0].x + p[2].x) / 2;
	p2.y = (p[0].y + p[2].y) / 2;
	dy1 = p[0].y - p[1].y;
	dy2 = p[0].y - p[2].y;
	*r = 0;
	if (dy1 != 0) {
		d = (p[1].x - p[0].x) / dy1;
		yi = p1.y - d * p1.x;
		if (dy2 != 0) {
			d2 = (p[2].x - p[0].x) / dy2;
			if (d != d2) *cx = (yi - (p2.y - d2 * p2.x)) / (d2 - d);
			else return;
		}
		else if (p[2].x - p[0].x == 0) return;
		else *cx = p2.x;
	}
	else if (dy2 != 0 && p[1].x - p[0].x != 0) {
		d = (p[2].x - p[0].x) / dy2;
		yi = p2.y - d * p2.x;
		*cx = p1.x;
	}
	else return;
	*cy = d * (*cx) + yi;
	*r = sqrt((p[0].x - (*cx)) * (p[0].x - (*cx)) + (p[0].y - (*cy)) * (p[0].y - (*cy)));
}
// JetLee-110525-01 End

void getCircleEstimate(long N, double* x, double* y, double* cx, double* cy, double* r)
{
	UWORD inx;
	double sumx, sumx2, sumx3, sumy, sumy2, sumy3, sumxy, sumx2y, sumxy2;
	//Point_XY p[3];
	//double ccx, ccy, rr;

	sumx = 0.0; sumx2 = 0.0; sumx3 = 0.0; sumy = 0.0; sumy2 = 0.0; sumy3 = 0.0; sumxy = 0.0; sumx2y = 0.0; sumxy2 = 0.0;

	for (inx = 0; inx < N; inx++)
	{
		sumx += x[inx];
		sumx2 += (x[inx] * x[inx]);
		sumx3 += (x[inx] * x[inx] * x[inx]);
		sumy += y[inx];
		sumy2 += (y[inx] * y[inx]);
		sumy3 += (y[inx] * y[inx] * y[inx]);
		sumxy += (x[inx] * y[inx]);
		sumx2y += (x[inx] * x[inx] * y[inx]);
		sumxy2 += (x[inx] * y[inx] * y[inx]);
	}
	// Make the average X, Y
	*cx = sumx / (double)N;
	*cy = sumy / (double)N;
	// Average - 0 Angle
	*r = sqrt((*cx - x[6]) * (*cx - x[6]) + (*cy - y[6]) * (*cy - y[6]));
	TRACE(_T("[PWR] Center of rotate XYR=(%7.3f, %7.3f, %7.3f)\n"), *cx, *cy, *r);
	//if (N == 12) 
	//{
	//	for (inx = 0; inx < 4; inx++) 
	//	{
	//		p[0].x = x[inx * 3]; 
	//		p[0].y = y[inx * 3]; 
	//		p[1].x = x[inx * 3 + 1]; 
	//		p[1].y = y[inx * 3 + 1]; 
	//		p[2].x = x[inx * 3 + 2]; 
	//		p[2].y = y[inx * 3 + 2];
	//		Get3PointCircle(p, &ccx, &ccy, &rr);
	//		TRACE(_T("[PWR] [HDOFF] Count=%d, Center=(%7.3f,%7.3f) r=%7.3f\n"), inx, ccx, ccy, rr);
	//	}
	//}
	return;
}

long findRotateCenterWithRes(long Gantry, CString strZAxis, long CamNo, double* cx, double* cy, double* Res)
{
	CString strAxis;
	long angle, TotalAngle = 40, Ms = TIME100MS, TimeOut = TIME10000MS, file = NZLCENTERMARK, RepeatCount = 1, retVis = 0, VisErr = 0, Err = NO_ERR;
	double resX[80], resY[80], cr;
	double Inpos = 0.1, Ratio = 0.1;
	Point_XYRE res;
	gLedOn(CamNo, 100, 100, 0);
	ThreadSleep(TIME100MS);
	for (angle = 0; angle < 80; angle++) 
	{ 
		resX[angle] = 0.0; 
		resY[angle] = 0.0; 
	}	
	strAxis = GetHeadToRotateAxis(Gantry, strZAxis);
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, -210.0, Inpos, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] findRotateCenterWithRes-1 StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}
	for (angle = 0; angle < 13; angle++)		// -180 ~ 180
	{
		Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, -180.0 + angle * 30.0, Inpos, Ms, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] findRotateCenterWithRes-2 StartPosWaitDelayedInposition Err:%d\n"), Err);
			return Err;
		}
		ThreadSleep(TIME100MS);
		res = markRecognitionRepeat(CamNo, file, RepeatCount, TIME10MS);
		retVis = gGetVisionCommandResult(Gantry);
		if (retVis != 0)
		{
			VisErr = gGetVisionErrorCode(Gantry);
			TRACE(_T("[PWR] markRecognitionRepeat Ret:%d Err:%d\n"), retVis, VisErr);
			return retVis;
		}
		/*HeadShiftRst[table][head].rst[angle].x = */resX[angle] = res.x;
		/*HeadShiftRst[table][head].rst[angle].y = */resY[angle] = res.y;
		TRACE(_T("[PWR] [ROTATE] HEAD(%s) RST(%.3f,%.3f) AT DEGREE(%7.3f)\n"), strZAxis, resX[angle], resY[angle], -180.0 + angle * 30.0);
	}
	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, 180.0, Inpos, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] findRotateCenterWithRes-3 StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME100MS);

	for (angle = 23; angle > 12; angle--)	// 150 ~ -150
	{
		double pos = 180.0 + (static_cast<double>(angle) - 24) * 30.0;
		Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, pos, Inpos, Ms, true);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] findRotateCenterWithRes-4 StartPosWaitDelayedInposition Err:%d\n"), Err);
			return Err;
		}
		ThreadSleep(TIME100MS);

		res = markRecognitionRepeat(CamNo, file, RepeatCount, TIME10MS);
		retVis = gGetVisionCommandResult(Gantry);
		if (retVis != 0)
		{
			VisErr = gGetVisionErrorCode(Gantry);
			TRACE(_T("[PWR] markRecognitionRepeat Ret:%d Err:%d\n"), retVis, VisErr);
			return retVis;
		}
		/*HeadShiftRst[table][head].rst[angle].x = */resX[angle] = res.x;
		/*HeadShiftRst[table][head].rst[angle].y = */resY[angle] = res.y;
		TRACE(_T("[PWR] [ROTATE] HEAD(%s) INDEX(%d) RST(%.3f,%.3f) AT DEGREE(%7.3f)\n"), strZAxis, angle, resX[angle], resY[angle], pos);
	}
	gLedAllOff();

	for (angle = 0; angle < 12; angle++)
	{
		resX[angle] = (resX[angle] + resX[angle + 12]) / 2.0;
		resY[angle] = (resY[angle] + resY[angle + 12]) / 2.0;
		//HeadShift[table][head].shift[angle].x = resX[angle];
		//HeadShift[table][head].shift[angle].y = resY[angle];
	}
	getCircleEstimate(12, resX, resY, cx, cy, &cr);
	//HeadShiftRst[table][head].cx = *cx;
	//HeadShiftRst[table][head].cy = *cy;
	//HeadShiftRst[table][head].cr = cr;
	*Res = cr;
	//HeadShift[table][head].shift[12].x = HeadShift[table][head].shift[0].x;
	//HeadShift[table][head].shift[12].y = HeadShift[table][head].shift[0].y;
	//for (angle = 0; angle < 13; angle++)
	//{
	//	HeadShift[table][head].shift[angle].x -= *cx;
	//	HeadShift[table][head].shift[angle].y -= *cy;
	//}
	//if (fabs(HeadShift[table][head].shift[6].x) < 0.001 && (HeadShift[table][head].shift[6].y >= 0.0))
	//	HeadShift[table][head].refAng = PIE * 0.5;
	//else if (fabs(HeadShift[table][head].shift[6].x) < 0.001 && (HeadShift[table][head].shift[6].y < 0.0))
	//	HeadShift[table][head].refAng = (-0.5) * PIE;
	//else if (fabs(HeadShift[table][head].shift[6].y) < 0.001 && (HeadShift[table][head].shift[6].x >= 0.0))
	//	HeadShift[table][head].refAng = 0.0;
	//else if (fabs(HeadShift[table][head].shift[6].y) < 0.001 && (HeadShift[table][head].shift[6].x < 0.0))
	//	HeadShift[table][head].refAng = PIE;
	//else { HeadShift[table][head].refAng = atan2((double)(HeadShift[table][head].shift[6].y), (double)(HeadShift[table][head].shift[6].x)); }
	//TRACE(_T("[PWR] Reference Angle = %7.3f(%7.3f deg)"), HeadShift[table][head].refAng, HeadShift[table][head].refAng * A180_PIE);
	//for (angle = 0; angle < 13; angle++)
	//{
	//	ca = cos(HeadShift[table][head].refAng + (-180.0 + angle * 30.0) * PIE_180);
	//	sa = sin(HeadShift[table][head].refAng + (-180.0 + angle * 30.0) * PIE_180);
	//	dx = cr * ca; dy = cr * sa;
	//	HeadShift[table][head].shift[angle].x = dx - HeadShift[table][head].shift[angle].x;
	//	HeadShift[table][head].shift[angle].y = dy - HeadShift[table][head].shift[angle].y;
	//}

	Err = StartPosWaitDelayedInposition(strAxis, Ratio, TimeOut, GetStandByR(FRONT_GANTRY), Inpos, Ms, true);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] findRotateCenterWithRes-5 StartPosWaitDelayedInposition Err:%d\n"), Err);
		return Err;
	}
	ThreadSleep(TIME100MS);

	//moveRHead(table, head, 0.00, INPOS_ON); taskDelay(TIME200MS);
	TRACE(_T("[PWR] *** Result(Cx,Cy,R) = (%7.3f, %7.3f, %7.3f) ***\n"), *cx, *cy, cr);
	//sprintf(buf, "RC_T%dH%dC%dS%d.dat", table + 1, head + 1, cam + 1, side);
	//if ((fd = creat(buf, O_RDWR | O_CREAT)) == ERROR)
	//{
	//	printf("\n-> File Creation Error !!!"); taskDelay(1000000);
	//}
	//else
	//{
	//	sprintf(buf, "*** Result(Cx,Cy,R) = (%7.3f, %7.3f, %7.3f) ***\n", *cx, *cy, cr); write(fd, buf, strlen(buf));
	//	sprintf(buf, "angle(deg), ResultX, ResultY, RotateX, RotateY, OffsetX, OffsetY\n"); write(fd, buf, strlen(buf));
	//	for (angle = 0; angle < 80; angle++)
	//	{
	//		if (fabs(resX[angle] - *cx) < 0.001 && fabs(resY[angle] - *cy) >= 0.0) rot = PIE * 0.5;
	//		else if (fabs(resX[angle] - *cx) < 0.001 && fabs(resY[angle] - *cy) < 0.0) rot = (-0.5) * PIE;
	//		else { rot = atan2((double)(resY[angle] - *cy), (double)(resX[angle] - *cx)); }
	//		ca = cos(rot); sa = sin(rot); 
	//		dx = cr * ca + *cx; dy = cr * sa + *cy;
	//		sprintf(buf, "%7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f\n",
	//			-180.0 + angle * 30.0,
	//			resX[angle], resY[angle], dx, dy,
	//			dx - resX[angle], dy - resY[angle]
	//		); write(fd, buf, strlen(buf));
	//	}
	//	close(fd);
	//}
	return 0;
}

long gPrepareCommand(long t, long* cam, long id, long* chk, long* useVA, long* dbNo, double* angleVA, long* Forming)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->PrepareCommand(t, cam, id, chk, useVA, dbNo, angleVA, Forming);
	}
	return retVis;
}

long gImageCatch(long t, long* cam, long id, long* chk, long* useVA, long uwDiv)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->ImageCatch(t, cam, id, chk, useVA, uwDiv);
	}
	return retVis;
}

long gStartProcess(long cam, long id)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->StartProcess(cam, id);
	}
	return retVis;
}

long gGetRecognitionResult(long cam, long loc, long id, long* err, Point_XYRE* res, long dwFrameNo[], Point_XYT* ptXYRESize)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->GetRecognitionResult(cam, loc, id, err, res, dwFrameNo, ptXYRESize);
	}
	return retVis;
}

void gSetRunVisionResult(long Gantry, long InsertOrder, Point_XYRE Res)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetRunVisionResult(Gantry, InsertOrder, Res);
	}
}

Point_XYRE gGetRunVisionResult(long Gantry, long InsertOrder)
{
	Point_XYRE retPt;
	ZeroMemory(&retPt, sizeof(retPt));
	if (gcEthernetVision)
	{
		retPt = gcEthernetVision->GetRunVisionResult(Gantry, InsertOrder);
	}
	return retPt;
}

void gSetVisionTimeOut(bool bTimeOut)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetVisionTimeOut(bTimeOut);
	}
}

bool gGetVisionTimeOut()
{
	bool bTimeOut = false;
	if (gcEthernetVision)
	{
		bTimeOut = gcEthernetVision->GetVisionTimeOut();
	}
	return bTimeOut;
}

void gSetRunVisionErrorCode(long Gantry, long InsertOrder, long ErrorCode)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetRunVisionErrorCode(Gantry, InsertOrder, ErrorCode);
	}
}

long gGetRunVisionErrorCode(long Gantry, long InsertOrder)
{
	long ErrorCode = 0;
	if (gcEthernetVision)
	{
		ErrorCode = gcEthernetVision->GetRunVisionErrorCode(Gantry, InsertOrder);
	}
	return ErrorCode;
}

void gInitRunVisionAngle(long Gantry, long InsertOrder)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->InitRunVisionAngle(Gantry, InsertOrder);
	}
}

void gSetRunVisionAngle(long Gantry, long InsertOrder, double RecognitionAngle)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetRunVisionAngle(Gantry, InsertOrder, RecognitionAngle);
	}
}

double gGetRunVisionAngle(long Gantry, long InsertOrder)
{
	double Angle = 0.0;
	if (gcEthernetVision)
	{
		Angle = gcEthernetVision->GetRunVisionAngle(Gantry, InsertOrder);
	}
	return Angle;
}

long gSendCameraRecognitionOffset(long cam, Point_XY CamOffset1, Point_XY CamOffset2, Point_XY CamOffset3)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->SendCameraRecognitionOffset(cam, CamOffset1, CamOffset2, CamOffset3);
	}
	return retVis;
}

long gInspectBarcode(long cam)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->InspectBarcode(cam);
	}
	return retVis;
}

long gInspectBarcode(long cam, long BarcodeType)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->InspectBarcode(cam, BarcodeType);
	}
	return retVis;
}

long gMarkPairRecognition(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, Ratio_XYRZ Ratio)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->MarkPairRecognition(Gantry, Mark1No, Mark2No, Mark1Pos, Mark2Pos, Ratio);
	}
	return retVis;
}

long gGetMarkDelta(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, long Res)
{
	long retVis = 0;
	if (gcEthernetVision)
	{
		retVis = gcEthernetVision->GetMarkDelta(Gantry, Mark1No, Mark2No, Mark1Pos, Mark2Pos, Res);
	}
	return retVis;
}

Point_XYR gMarkCompensation(long Gantry, Point_XYR Pos, long Res)
{
	Point_XYR retPt;
	ZeroMemory(&retPt, sizeof(retPt));
	if (gcEthernetVision)
	{
		retPt = gcEthernetVision->MarkCompensation(Gantry, Pos, Res);
	}
	return retPt;
}

Point_XYRZ gComponentCompensation(long Gantry, long ChkPos, Point_XYRZ Pos, long InsertOrd, long MarkRes)
{
	Point_XYRZ retPt;
	ZeroMemory(&retPt, sizeof(retPt));
	if (gcEthernetVision)
	{
		retPt = gcEthernetVision->ComponentCompensation(Gantry, ChkPos, Pos, InsertOrd, MarkRes);
	}
	return retPt;
}

Point_XYR gMarkLoss(long Gantry, Point_XYR* Pos, long Res)
{
	Point_XYR retPt;
	ZeroMemory(&retPt, sizeof(retPt));
	if (gcEthernetVision)
	{
		retPt = gcEthernetVision->MarkLoss(Gantry, Pos, Res);
	}
	return retPt;
}

void gUpdateVisionData(UWORD bdno, UWORD ret, UWORD data)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->updateVisionData(bdno, ret, data);
	}
}

void gClearRunVisionResult(long bdno)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->ClearRunVisionResult(bdno);
	}
}

void gClearRunRecognitionAngle(long bdno)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->ClearRunRecognitionAngle(bdno);
	}
}

void gShowRoiBase(long bdno, WindowSize Win)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->showRoiBase(bdno, 1, 3, 0, Win, 0);
	}
}

long gGetVisionErrorCode(long bdno)
{
	long VisErr = 0;
	if (gcEthernetVision)
	{
		VisErr = gcEthernetVision->GetVisionErrorCode(bdno);
	}
	return VisErr;
}

long gGetVisionCommandResult(long bdno)
{
	long RetVis = 0;
	if (gcEthernetVision)
	{
		RetVis = gcEthernetVision->GetVisionCommandResult(bdno);
	}
	return RetVis;
}

long gGetMarkArea(long bdno, long MarkNo)
{
	long MarkArea = 0;
	if (gcEthernetVision)
	{
		MarkArea = gcEthernetVision->GetMarkArea(bdno, MarkNo);
	}
	return MarkArea;
}

long gSetMarkLed(long MarkNo, OFFSET_LED Led)
{
	long Err = NO_ERR;
	if (gcEthernetVision)
	{
		Err = gcEthernetVision->SetMarkLed(MarkNo, Led);
	}
	return Err;
}

void gSetPartRecognitionResult(long CamNo, long CamChk, Point_XYRE Res)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetPartRecognitionResult(CamNo, CamChk, Res);
	}
}

Point_XYRE gGetPartRecognitionResult(long CamNo, long CamChk)
{
	Point_XYRE Res;
	ZeroMemory(&Res, sizeof(Res));
	if (gcEthernetVision)
	{
		Res = gcEthernetVision->GetPartRecognitionResult(CamNo, CamChk);
	}
	return Res;
}

long gDropCheckSaveImage(long* cam, long size)
{
	if (gcEthernetVision)
	{
		return gcEthernetVision->DropCheckSaveImage(cam, size);
	}

	return 0;
}

long gDropCheckProcess(long* cam, long size, long* result)
{
	if (gcEthernetVision)
	{
		return gcEthernetVision->DropCheckProcess(cam, size, result);
	}

	return 0;
}

long gGetVisionTable(long CamNo)
{
	if (CamNo < CAM6 || CamNo == FHCAM)
	{
		return FRONT_VISION;
	}
	else
	{
		return REAR_VISION;
	}
}

CString gGetCameraBarcode(long bdno)
{
	CString strBarcode;
	if (gcEthernetVision)
	{
		strBarcode = gcEthernetVision->GetBarcode();
	}
	return strBarcode;
}


double gGetVisionPartPitchResult(long bdno)
{
	double res;
	res = 0.000;
	if (gcEthernetVision)
	{
		res = gcEthernetVision->GetVisionPartPitchResult(bdno);
	}
	return res;
}

void gSetRunVisionPitchResult(long Gantry, long InsertOrder, Point_XYT SizeResult)
{
	if (gcEthernetVision)
	{
		gcEthernetVision->SetRunVisionPitchResult(Gantry, InsertOrder, SizeResult);
	}
}

Point_XYT gGetRunVisionPitchResult(long Gantry, long InsertOrder)
{
	Point_XYT SizeResult;
	ZeroMemory(&SizeResult, sizeof(SizeResult));
	if (gcEthernetVision)
	{
		SizeResult = gcEthernetVision->GetRunVisionPitchResult(Gantry, InsertOrder);
	}
	return SizeResult;
}