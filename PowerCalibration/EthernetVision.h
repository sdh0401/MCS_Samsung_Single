#pragma once
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "VisionData.h"
#include "LockDef.h"
//#include "CPowerThread.h"
#include "Thread.h"

class CEthernetVision : public CThread
{
public:
	CEthernetVision(bool bSimul);
	~CEthernetVision();
	long m_VisionCamNo[MAXCAMNO][3]; /*** Board,VisionCamID,FrameNo ***/
	VisionVersionInfo gVisionVer[MAXTABLENO];
	char BarcodeName[MAXTABLENO][MAXBARCODENAME];
	char OCRCharacter[MAXTABLENO][10][MAXBARCODENAME];
	int	headCamShift;
	int	m_bd;
	unsigned int crc_table[256];
	bool m_bSimulation;
	COMMAND_DPRAM VIS_CMD[MAXTABLENO][WRITE_BUFFSIZE];
	COMMAND_DPRAM VIS_RST[MAXTABLENO][WRITE_BUFFSIZE];
	Point_XYRE VisResult[MAXTABLENO][20];
	MarkDeltaInfo MarkDelta[MAXTABLENO][MAXBDFIDNO + 40];
	Point_XYRE m_VAResult[MAXGANTRY][MAXUSEDHEADNO];
	Point_XYRE m_VAPartRecognitionResult[MAXCAMNO][MAXUSEDHEADNO];
	long m_VAErrorCode[MAXGANTRY][MAXUSEDHEADNO];
	double m_VAAngleInsert[MAXGANTRY][MAXUSEDHEADNO];
	void VisionVersion(DWORD bdno);
	void SetVisionResult(long bd, long lVisCmd, COMMAND_DPRAM* VisionResult);
	void SetCameraCalibration(DWORD Cam, double* dblCamCal);
	void SetVisionTimeOut(bool bTimeOut);
	bool GetVisionTimeOut();
	void updateVisionData(long bdno, long ret, long data);
	Point_XYT m_VAResultPitch[MAXGANTRY][MAXUSEDHEADNO];
	/*
		function
	*/
	long GetVisionPosition(long CamNo);
	void ClearVisionCommand(long bd);
	bool TriggerEvent(long bd, long Size, long Cmd, CString strMsg);
	int sendVisCmd(long bd, long no, long cmd, CString strMsg);
	int CameraCalibration(int cam, unsigned MaxCalJigMarkNo, Point_XY* calpos);
	int CameraRotateCalibrationPrepare(long cam, double JigPitchX); // Ready
	int CameraRotateCalibration(long cam, long ImageCatch); // 1~5:Catch, 6:Get Result
	long SetMarkCameraCalibrationJigCenterMark(long cam);
	long markTraining(int cam, int uwfileno, int calmark, Point_XYRE* pRes);
	Point_XYRE markTrainingWithoutCamCal(int cam, int uwfileno, int calmark, int showrst, int useCal);
	long prepareMarkRecognition(long cam, long FileNo, long MarkNo);
	long catchMarkImage(long cam);
	long getMarkRecognition(long cam, Point_XYRE* res);
	bool GetVisionBusy(unsigned bdno, long TimeOut);
	long GetVisionCommandResult(unsigned bdno);
	long GetVisionErrorCode(unsigned bdno);
	long GetVisionBarcodeSize(unsigned bdno);
	long GetVisionFrmaeNo(unsigned bdno, unsigned FrameCount);
	Point_XYRE GetVisionMarkResult(unsigned bdno, unsigned index);
	Point_XYRE GetVisionMarkResult(unsigned bdno);
	Point_XYRE GetVisionPartResult(unsigned bdno);
	Point_XYT GetVisionPartSizeResult(unsigned bdno);
	long inspectMachCalMark(int CameraNo, long MarkNo, Point_XYRE* res);
	long inspectMachCalRefMark(int CameraNo, long MarkNo, Point_XYRE* res);
	long inspectMark(int CameraNo, long MarkNo, Point_XYRE* res);
	long inspectROriginMark(int CameraNo, long MarkNo, Point_XYRE* res);
	void ClearVisionResult(unsigned bdno, unsigned indx);
	void ClearRunVisionResult(unsigned bdno);
	void ClearRunRecognitionAngle(unsigned bdno);
	long GetVisionResultLong(unsigned bdno, unsigned indx);
	double GetVisionResultDouble(unsigned bdno, unsigned indx);
	int WindowMoving(int CameraNo, int Window, int Show, int Control, int pixel, int dir);
	long PrepareCommand(long t, long* cam, long id, long* chk, long* useVA, long* dbNo, double* angleVA, long* Forming);
	long ImageCatch(long t, long* cam, long id, long* chk, long* idx, long uwDiv);
	long StartProcess(long cam, long id);
	long GetRecognitionResult(long cam, long loc, long id, long* err, Point_XYRE* res, long dwFrameNo[], Point_XYT* ptXYRESize);
	long SendCameraRecognitionOffset(long cam, Point_XY CamOffset1, Point_XY CamOffset2, Point_XY CamOffset3);
	long InspectBarcode(long cam);
	long MarkPairRecognition(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, Ratio_XYRZ RatioXY);
	long GetMarkDelta(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, long Res);
	Point_XYR MarkCompensation(long Gantry, Point_XYR Pos, long Res);
	Point_XYRZ ComponentCompensation(long Gantry, long ChkPos, Point_XYRZ Pos, long InsertOrder, long MarkRes);
	Point_XYR MarkLoss(long Gantry, Point_XYR* Pos, long Res);
	void SetRunVisionResult(long Gantry, long InsertOrder, Point_XYRE res);
	Point_XYRE GetRunVisionResult(long Gantry, long InsertOrder);
	void SetRunVisionErrorCode(long Gantry, long InsertOrder, long ErrorCode);
	long GetRunVisionErrorCode(long Gantry, long InsertOrder);
	void InitRunVisionAngle(long Gantry, long InsertOrder);
	void SetRunVisionAngle(long Gantry, long InsertOrder, double Angle);
	double GetRunVisionAngle(long Gantry, long InsertOrder);
	long GetMarkArea(long Gantry, long fileNo);
	void SetPartRecognitionResult(long CamNo, long CamChk, Point_XYRE Res);
	Point_XYRE GetPartRecognitionResult(long CamNo, long CamChk);
	double GetVisionPartPitchResult(unsigned bdno);
	void SetRunVisionPitchResult(long Gantry, long InsertOrder, Point_XYT SizeResult);
	Point_XYT GetRunVisionPitchResult(long Gantry, long InsertOrder);

	long LiveOn(long cam);
	long LiveOff(long cam);
	long OverlayDisplayOnOff(long bdno, bool bOn);
	long CaptureImageAndFreeze(long cam);
	void DefineVisionCamera(void);

	void setMarkRoi(long file, int x1, int y1, int x2, int y2);
	void setMarkArea(long file, int Area);
	long SetMarkLed(long MarkNo, OFFSET_LED Led);
	int showRoiBase(long bd, long win, long uwShow, long cor, WindowSize Win1, long headCam);
	int showRoi2(long bd, long file);
	int showRoi(long bd, UDWORD x1, UDWORD y1, UDWORD x2, UDWORD y2, long headCam);
	int getCamNo(long bd, long visionCamNo);
	int visionResult(long bdno, long ret, long cmd);
	long visionESC(long bdno, long id);
	ULONGLONG m_GetTime, m_ElapsedTime = 0;
	long DropCheckSaveImage(long* cam, long size);
	long DropCheckProcess(long* cam, long size, long* result);
	long DropCheckGetResult(long bdno, long* result);
	long InspectBarcode(long cam, long BarcodeType);
	CString GetBarcode();

private:
	long GetVABoardNoFromGlobalCameraNo(long cam);
	long GetVACameraNoFromGlobalCameraNo(long cam);
	ThreadId_t m_id;
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	HANDLE GetThreadLock();
	bool Lock();
	bool Flush();
	bool Unlock();
	HANDLE m_CmdLock;
	HANDLE m_RecvEvent;
	bool IsReceived(unsigned loopTime);
	bool m_bTimeOut;
	long m_DropCheckResult[MAXCAMNO];
	long m_MaxWaitTime;
	CString m_Barcode;
};

extern CEthernetVision* gcEthernetVision;
