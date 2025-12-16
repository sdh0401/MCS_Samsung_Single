#include "pch.h"
#include "EthernetVision.h"
#include "VisionData.h"
#include "CPowerVision.h"
#include "CApplicationTime.h"
#include "DefineThreadLoopTime.h"
#include "GlobalData.h"
#include "GlobalDefine.h"
#include "Trace.h"
#include "vision.h"
#include "VisionData.h"
#include "CPowerLog.h"
#include "AxisInformation.h"
//#include "ErrorCode.h"
#include "CMachineConfig.h"

CEthernetVision* gcEthernetVision;
CEthernetVision::CEthernetVision(bool bSimul)
{
	m_bd = 0;
	GetId(&m_id);
	m_bSimulation = bSimul;
	ClearVisionCommand(0);
	ClearVisionCommand(1);
	DefineVisionCamera();
	m_CmdLock = CreateMutex(NULL, FALSE, NULL);
	m_RecvEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_bTimeOut = false;
	m_MaxWaitTime = TIME5000MS;
}

CEthernetVision::~CEthernetVision()
{
	TRACE(_T("[PWR] ~CEthernetVision() Start\n"));
	CloseHandle(m_CmdLock);
	m_CmdLock = NULL;
	CloseHandle(m_RecvEvent);
	m_RecvEvent = NULL;
	TRACE(_T("[PWR] ~CEthernetVision() End\n"));
}

long CEthernetVision::GetVABoardNoFromGlobalCameraNo(long cam)
{
	if (cam >= CAM1 && cam < MAXCAMNO)
	{
		return m_VisionCamNo[cam][0];
	}
	else
	{
		TRACE(_T("[PWR] GetVABoardNoFromGlobalCameraNo Camera no is over(%d) under %d"), cam, MAXCAMNO);
		return NON;
	}
}

long CEthernetVision::GetVACameraNoFromGlobalCameraNo(long cam)
{
	if (cam >= CAM1 && cam < MAXCAMNO)
	{
		return m_VisionCamNo[cam][1];
	}
	else
	{
		TRACE(_T("[PWR] GetVACameraNoFromGlobalCameraNo Camera no is over(%d) under %d"), cam, MAXCAMNO);
		return NON;
	}
}

HANDLE CEthernetVision::GetThreadLock()
{
	ASSERT(m_CmdLock != INVALID_HANDLE_VALUE);
	return m_CmdLock;
}

bool CEthernetVision::Lock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_LOCK(GetThreadLock(), INFINITE);
	return true;
}

bool CEthernetVision::Flush()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_FLUSH(GetThreadLock());
	return true;
}

bool CEthernetVision::Unlock()
{
	if (GetThreadLock() == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	if (GetThreadLock() == NULL)
	{
		return false;
	}
	SEM_UNLOCK(GetThreadLock());
	return true;
}

BOOL CEthernetVision::OnTask()
{
	return TRUE;
}

BOOL CEthernetVision::OnTask(LPVOID lpv) // bd, Size, Cmd
{
	signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
	signed nTargetID = INITIALZE;
	CString strMsg;
	if (lpv)
	{
		PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
		nTargetID = msgReceived->GetID();
		if (nTargetID != m_id)
		{
			TRACE(_T("[PWR] CEthernetVision GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
		}
		strMsg = msgReceived->GetThreadMsg(); // Axis
		for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
		{
			nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
		}
		//dont' use cout here, output could be broken up due to threading
		if (gcPowerLog->IsShowVisionLog() == true)
		{
			TRACE("[PWR] CEthernetVision(0x%04X) sendVisCmd Get Msg:%d %d %d\n", m_id, nSubMsg[0], nSubMsg[1], nSubMsg[2]);
		}
		sendVisCmd(nSubMsg[0], nSubMsg[1], nSubMsg[2], strMsg);
		delete msgReceived;
	}
	return TRUE;
}

bool CEthernetVision::TriggerEvent(long bd, long Size, long Cmd, CString strMsg)
{
	if (GetGlobalSimulationMode() == true || GetSkipVision() == 1)
	{
		//TRACE("[PWR] VisionResult CMD%02d Skip\n", VIS_CMD[bdno][0]);
		return 0;
	}


	if (Cmd == VIS_DROPCHECK)
	{
		TRACE(_T("[PWR] TriggerEvent VIS_DROPCHECK bd:%d Msg:%s\n"), bd, (LPCTSTR)strMsg);
	}

	PowerThreadMessage* msgReceived = new PowerThreadMessage();
	msgReceived->SetThreadMsg(strMsg);
	msgReceived->SetID(m_id);
	msgReceived->SetThreadSubMsg(bd, Size, Cmd);
	if (PingThread(TIME1MS))
	{
		Lock();
		ResetEvent(m_RecvEvent);
		Event((LPVOID)msgReceived);
		return true;
	}
	//sendVisCmd(bd, Size, Cmd, strMsg);
	return false;	
}

void CEthernetVision::ClearVisionCommand(long bd)
{
	ZeroMemory(&VIS_CMD[bd], sizeof(VIS_CMD[bd]));
	ZeroMemory(&VIS_RST[bd], sizeof(VIS_RST[bd]));
}

long CEthernetVision::GetVisionPosition(long CamNo)
{
	return m_VisionCamNo[CamNo][0];
}

int CEthernetVision::sendVisCmd(long bd, long no, long cmd, CString strMsg)
{
	bool bRet = false;
	m_GetTime = _time_get();
	if (bd == 0)
	{
		bRet = gcPowerVision->SendCommand(cmd, strMsg, FRONT_VISION);
	}
	else
	{
		bRet = gcPowerVision->SendCommand(cmd, strMsg, REAR_VISION);
	}
	if(bRet == false) 
	{
		TRACE("[PWR] Vision server[bd:%d][Write Failed]\n", bd);
		return ERROR;
	}
	return 0;
}

bool CEthernetVision::IsReceived(unsigned loopTime)
{
	bool bRet = false;
	DWORD ret;
	ret = WaitForSingleObject(m_RecvEvent, loopTime);
	if (ret == WAIT_FAILED)
	{
		return bRet;
	}
	else if (ret == WAIT_TIMEOUT)
	{
		return bRet;
	}
	else
	{
		return true;
	}
}

int CEthernetVision::visionResult(long bdno, long ret, long cmd)
{
	if (m_bSimulation)
	{
		return 0;
	}
	if (GetGlobalSimulationMode() == true || GetSkipVision() == 1)
	{
		//TRACE("[PWR] VisionResult CMD%02d Skip\n", VIS_CMD[bdno][0]);
		return 0;
	}
	ULONGLONG TotalGetTime, GetTime, ElapsedTime = 0;
	TotalGetTime = GetTime = _time_get();
	while (IsReceived(THREAD_VISION_RESULT_READTIME) == false)
	{
		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > m_MaxWaitTime) // 보통 1초 이내에 반응한다~~~ 그렇지 않은 경우에는 확인하여 출력한다~~~
		{
			GetTime = _time_get();
			TRACE("[PWR] VisionResult CMD%02d Elapsed:%d[ms] TimeOut\n", VIS_CMD[bdno][0], ElapsedTime);
			SetVisionTimeOut(true);
			Unlock();

			CString strErr;
			strErr.Format(_T("Vision timeout(%s)"), bdno == FRONT_GANTRY ? _T("Front") : _T("Rear"));
			SendAlarm(VISION_RECEIVE_TIMEOUT, strErr);

			return -1;
		}
		ThreadSleep(TIME1MS);
	}
	if (GetVisionResultLong(bdno, 0) != cmd)
	{
		TRACE("[PWR] <Vision Result bdno:%d> Return command mismatchs (MCS->VIS:%d VIS->MCS:%d)\n", bdno, cmd, GetVisionResultLong(bdno, 0));
	}
	ElapsedTime = _time_elapsed(TotalGetTime);
	if (ElapsedTime > TIME10MS)
	{
		if (gcPowerLog->IsShowVisionLog() == true)
		{
			m_ElapsedTime = _time_elapsed(m_GetTime);
			TRACE(_T("[PWR] CEthernetVision visionResult Elapsed:%d[ms](%d[ms])\n"), ElapsedTime, m_ElapsedTime);
		}
	}
	SetVisionTimeOut(false);
	Unlock();
	return 0;
}

long CEthernetVision::visionESC(long bdno, long id)
{
	CString strMsg;
	long retVis = 0, Err = 0;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_QPVA;
	VIS_CMD[bdno][1]._long = 99;
	VIS_CMD[bdno][2]._long = id;
	VIS_CMD[bdno][3]._long = VIS_EOP;
	VIS_CMD[bdno][4]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, 5, VIS_QPVA, strMsg);
	visionResult(bdno, 0, VIS_QPVA);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::DropCheckSaveImage(long* cam, long size)
{
	long retVis = 999, Err = 0;
	long bdno, i;
	long no = 0;
	long camidx[MAXVAHEAD] = { 0, };
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam[0]);

	for (i = 0; i < size; i++)
	{
		long tempidx = GetVACameraNoFromGlobalCameraNo(cam[i]);
		if (tempidx > 0 && tempidx <= MAXVAHEAD)
		{
			camidx[tempidx - 1] = 1;
		}
	}

	//ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);

	memset(VIS_CMD[bdno], 0, MAXVISMSGLENGTH);
	VIS_CMD[bdno][no++]._long = VIS_DROPCHECK;
	VIS_CMD[bdno][no++]._long = 0;
	for (i = 0; i < MAXVAHEAD; i++)
	{
		//VIS_CMD[bdno][no++]._long = GetVACameraNoFromGlobalCameraNo(cam[i]);
		VIS_CMD[bdno][no++]._long = camidx[i];
	}

	VIS_CMD[bdno][no++]._long = VIS_EOP;
	VIS_CMD[bdno][no]._long = 0;

	strMsg.AppendFormat(_T("%d"), VIS_CMD[bdno][1]._long);
	for (int indx = 2; indx <= no; ++indx)
	{
		strMsg.AppendFormat(_T(",%d"), VIS_CMD[bdno][indx]._long);
	}

	TriggerEvent(bdno, no + 1, VIS_DROPCHECK, strMsg);
	if (visionResult(bdno, cam[0] + 1, VIS_DROPCHECK) < 0)
	{
		return -2;
	}
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::DropCheckProcess(long* cam, long size, long* result)
{
	long retVis = 999, Err = 0;
	long bdno, i;
	long no = 0;
	long camidx[MAXVAHEAD] = { 0, };

	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam[0]);
	for (i = 0; i < size; i++)
	{
		long tempidx = GetVACameraNoFromGlobalCameraNo(cam[i]);
		if (tempidx > 0 && tempidx <= MAXVAHEAD)
		{
			camidx[tempidx - 1] = 1;
		}
	}
	//ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);

	memset(VIS_CMD[bdno], 0, MAXVISMSGLENGTH);
	VIS_CMD[bdno][no++]._long = VIS_DROPCHECK;
	VIS_CMD[bdno][no++]._long = 1;
	for (i = 0; i < MAXVAHEAD; i++)
	{
		//VIS_CMD[bdno][no++]._long = GetVACameraNoFromGlobalCameraNo(cam[i]);
		VIS_CMD[bdno][no++]._long = camidx[i];
	}

	VIS_CMD[bdno][no++]._long = VIS_EOP;
	VIS_CMD[bdno][no]._long = 0;

	strMsg.AppendFormat(_T("%d"), VIS_CMD[bdno][1]._long);
	for (int indx = 2; indx <= no; ++indx)
	{
		strMsg.AppendFormat(_T(",%d"), VIS_CMD[bdno][indx]._long);
	}

	TriggerEvent(bdno, no + 1, VIS_DROPCHECK, strMsg);
	if (visionResult(bdno, cam[0] + 1, VIS_DROPCHECK) < 0)
	{
		return -2;
	}

	retVis = GetVisionCommandResult(bdno);

	Err = DropCheckGetResult(bdno, result);
	return retVis;
}

long CEthernetVision::DropCheckGetResult(long bdno, long* result)
{
	result[0] = GetVisionResultLong(bdno, 2);
	result[1] = GetVisionResultLong(bdno, 3);
	return 0;
}



void CEthernetVision::DefineVisionCamera(void)
{
	long cam;
	int camBufIndex = 2;
	for (cam = 0; cam < MAXCAMNO; ++cam)
	{
		m_VisionCamNo[cam][0] = CamNoBySystem[camBufIndex][cam][0];
		m_VisionCamNo[cam][1] = CamNoBySystem[camBufIndex][cam][1];
		m_VisionCamNo[cam][2] = CamNoBySystem[camBufIndex][cam][2];
	}
}

void CEthernetVision::setMarkRoi(long file, int x1, int y1, int x2, int y2)
{
	Mark[file].Win.x1 = x1;
	Mark[file].Win.y1 = y1;
	Mark[file].Win.x2 = x2;
	Mark[file].Win.y2 = y2;
}

void CEthernetVision::setMarkArea(long file, int Area)
{
	Mark[file].Area = Area;
}

long CEthernetVision::SetMarkLed(long MarkNo, OFFSET_LED Led)
{
	if (MarkNo > 0 && MarkNo <= MAXVISIONFILENO)
	{
		Mark[MarkNo - 1].LEDLevel[0] = (UWORD)Led.Red;
		Mark[MarkNo - 1].LEDLevel[1] = (UWORD)Led.Blue;
		Mark[MarkNo - 1].LEDLevel[2] = 0;
	}
	return NO_ERR;
}

// 2:On, 3:Off
long CEthernetVision::OverlayDisplayOnOff(long bdno, bool bOn)
{
	long err = 0;
	CString strMsg;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_DT;
	if(bOn == true)
		VIS_CMD[bdno][1]._long = 2;
	else
		VIS_CMD[bdno][1]._long = 3;
	VIS_CMD[bdno][2]._long = VIS_EOP;
	VIS_CMD[bdno][3]._long = 0;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	TriggerEvent(bdno, 4, VIS_DT, strMsg);
	err = visionResult(bdno, 0, VIS_DT);
	return err;
}

// 1:Image Capture and Freeze Mode
long CEthernetVision::CaptureImageAndFreeze(long cam)
{
	long err = 0;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam);
	CString strMsg;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_DT;
	VIS_CMD[bdno][1]._long = 1;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._long = VIS_EOP;
	VIS_CMD[bdno][4]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, 5, VIS_DT, strMsg);
	err = visionResult(bdno, 0, VIS_DT);
	return err;
}

// Live Mode
long CEthernetVision::LiveOn(long cam)
{
	int bdno, rst;
	DWORD no = 0;
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][no++]._long = VIS_DT;
	VIS_CMD[bdno][no++]._long = 0;
	VIS_CMD[bdno][no++]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][no++]._long = VIS_EOP;
	VIS_CMD[bdno][no]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, no + 1, VIS_DT, strMsg);
	rst = visionResult(bdno, 0, VIS_DT);
	return rst;
}

// Live Off
long CEthernetVision::LiveOff(long cam)
{
	int bdno, rst;
	DWORD no = 0;
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][no++]._long = VIS_DT;
	VIS_CMD[bdno][no++]._long = 0;
	VIS_CMD[bdno][no++]._long = 0; // Live Off
	VIS_CMD[bdno][no++]._long = VIS_EOP;
	VIS_CMD[bdno][no]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, no + 1, VIS_DT, strMsg);
	rst = visionResult(bdno, 0, VIS_DT);
	return rst;
}

int CEthernetVision::showRoiBase(long bdno, long win, long uwShow, long cor, WindowSize Win1, long headCam)
{
	WindowSize _Win1;
	int _rst;
	CString strMsg;
	ClearVisionCommand(bdno);
	_Win1 = Win1;
	VIS_CMD[bdno][0]._long = VIS_WC;
	VIS_CMD[bdno][1]._long = win;
	VIS_CMD[bdno][2]._long = uwShow;
	VIS_CMD[bdno][3]._long = cor;
	VIS_CMD[bdno][4]._long = _Win1.x1;
	VIS_CMD[bdno][5]._long = _Win1.y1;
	VIS_CMD[bdno][6]._long = _Win1.x2;
	VIS_CMD[bdno][7]._long = _Win1.y2;
	VIS_CMD[bdno][8]._long = _Win1.x1;
	VIS_CMD[bdno][9]._long = _Win1.y1;
	VIS_CMD[bdno][10]._long = _Win1.x2;
	VIS_CMD[bdno][11]._long = _Win1.y2;
	VIS_CMD[bdno][12]._long = EOP;
	VIS_CMD[bdno][13]._long = 0;
	VIS_CMD[bdno][13]._dbl = 0.0;
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"),
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long,
		VIS_CMD[bdno][6]._long, VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long, VIS_CMD[bdno][10]._long,
		VIS_CMD[bdno][11]._long, VIS_CMD[bdno][12]._long, VIS_CMD[bdno][13]._long);
	TriggerEvent(bdno, 14, VIS_WC, strMsg);
	TRACE("[PWR] %d VIS_WC %d %d %d %d %d %d %d %d %d %d %d %d %d\n", bdno,
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, 
		VIS_CMD[bdno][6]._long, VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long, VIS_CMD[bdno][10]._long, 
		VIS_CMD[bdno][11]._long, VIS_CMD[bdno][12]._long, VIS_CMD[bdno][13]._long);
	_rst = visionResult(bdno, 0, VIS_WC);
	return _rst;
}

int CEthernetVision::showRoi2(long bd, long file)
{
	return showRoiBase(bd, bd+1, 1, 3, Mark[file].Win, 0);
}

int CEthernetVision::showRoi(long bd, UDWORD x1, UDWORD y1, UDWORD x2, UDWORD y2, long headCam)
{
	WindowSize Win1;
	Win1.x1 = x1; Win1.y1 = y1;
	Win1.x2 = x2; Win1.y2 = y2;
	return showRoiBase(bd, bd+1, 1, 3, Win1, headCam);
}

int CEthernetVision::getCamNo(long bd, long visionCamNo)
{
	long cam;
	for(cam = 0; cam < USEDCAMNO; cam++)
	{
		if (GetVACameraNoFromGlobalCameraNo(cam) == visionCamNo && GetVABoardNoFromGlobalCameraNo(cam) == bd)
		{
			return cam;
		}
	}
	return ERROR;
}

void CEthernetVision::VisionVersion(DWORD bdno)
{
	CString strMsg;
	if (m_bSimulation == true) return ;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_SV;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = VIS_EOP;
	VIS_CMD[bdno][3]._long = 0;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	CApplicationTime* pTime = new CApplicationTime();	
	TriggerEvent(bdno, 4, VIS_SV, strMsg);
	if (visionResult(bdno, 0, VIS_SV) != 0)
	{
		delete pTime;
		pTime = NULL;
	}
	TRACE(_T("[PWR] VisionVersion Elapsed:%d"), pTime->TimeElapsed());
	gVisionVer[bdno].Core  = (long)VIS_RST[bdno][3]._long;	/* 1835 */
	gVisionVer[bdno].Core1 = (long)VIS_RST[bdno][4]._long;	//Core Version1			/* 1	*/
	gVisionVer[bdno].Core2 = (long)VIS_RST[bdno][5]._long;	//Core Version2			/* 3	*/
	gVisionVer[bdno].Core3 = (long)VIS_RST[bdno][6]._long;	//Core Version3			/* 0	*/
	gVisionVer[bdno].Core4 = (long)VIS_RST[bdno][7]._long;	//Core Version4			/*  Sub Version		*/
	gVisionVer[bdno].Core5 = (long)VIS_RST[bdno][8]._long;	//Process BD Type		/* 0 : ± , 1 : TI, 2 :  */

	gVisionVer[bdno].Grabber = (long)VIS_RST[bdno][9]._long;
	gVisionVer[bdno].Eth_Ver.MVL.Major = (long)VIS_RST[bdno][10]._long;
	gVisionVer[bdno].Eth_Ver.MVL.Minor = (long)VIS_RST[bdno][11]._long;
	gVisionVer[bdno].Eth_Ver.MVL.Build = (long)VIS_RST[bdno][12]._long;
	gVisionVer[bdno].Eth_Ver.MVL.Patch = (long)VIS_RST[bdno][13]._long;

	gVisionVer[bdno].Eth_Ver.MVT.Major = (long)VIS_RST[bdno][14]._long;
	gVisionVer[bdno].Eth_Ver.MVT.Minor = (long)VIS_RST[bdno][15]._long;
	gVisionVer[bdno].Eth_Ver.MVT.Build = (long)VIS_RST[bdno][16]._long;
	gVisionVer[bdno].Eth_Ver.MVT.Patch = (long)VIS_RST[bdno][17]._long;
	gVisionVer[bdno].Eth_Ver.MVT.Resol = (long)VIS_RST[bdno][18]._long;
	TRACE("[PWR] Bd:%d Core %d Lib %d.%d.%d.%d Cmd %d.%d.%d.%d.%d\n", bdno, gVisionVer[bdno].Core,
		VIS_RST[bdno][10], VIS_RST[bdno][11], VIS_RST[bdno][12], VIS_RST[bdno][13],
		VIS_RST[bdno][14], VIS_RST[bdno][15], VIS_RST[bdno][16], VIS_RST[bdno][17], VIS_RST[bdno][18]);
	delete pTime;
	pTime = NULL;
}

/*
	bdno      : 0 or 1
	ret		: 0=Return ErrorCode, Other=Help Me
	data	: 0=All 1=Cal 2=Mark 3=VA 4=Nozzle 5=RecogPos
*/
void CEthernetVision::updateVisionData(long bdno, long ret, long data)
{
	CString strMsg;
	if (m_bSimulation == true) return;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_DB;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = (long)data;
	VIS_CMD[bdno][3]._long = 0; // FOV Mode(For Depth : 0 ~ 2)
	VIS_CMD[bdno][4]._long = VIS_EOP;
	VIS_CMD[bdno][5]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long);
	TriggerEvent(bdno, 6, VIS_DB, strMsg);
	visionResult(bdno, 0, VIS_DB);
	return;
}

void CEthernetVision::SetCameraCalibration(DWORD Cam, double* dblCamCal)
{
	for (int indz = 0; indz < 32; ++indz)
	{
		Camera[Cam].NewCamCal[indz] = dblCamCal[indz];
	}
}

void CEthernetVision::SetVisionTimeOut(bool bTimeOut)
{
	m_bTimeOut = bTimeOut;
}

bool CEthernetVision::GetVisionTimeOut()
{
	return m_bTimeOut;
}

void CEthernetVision::SetVisionResult(long bd, long lVisCmd, COMMAND_DPRAM* VisionResult)
{
	ZeroMemory(VIS_RST[bd], sizeof(VIS_RST[bd]));
	for (long index = 0; index < WRITE_BUFFSIZE - 1; ++index)
	{
		VIS_RST[bd][index]._long = VisionResult[index]._long;
		VIS_RST[bd][index]._dbl = VisionResult[index]._dbl;
	}
	VIS_RST[bd][0]._long = lVisCmd;
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] Recv Vision Result(Board:%d Cmd:%d Ret:%d Err:%d, XY:%.3f,%.3f)\n"), bd, VIS_RST[bd][0]._long, VIS_RST[bd][1]._long, VIS_RST[bd][2]._long, VIS_RST[bd][3]._dbl, VIS_RST[bd][4]._dbl);
	}
	SetEvent(m_RecvEvent);
}

int CEthernetVision::CameraCalibration(int cam, unsigned MaxCalJigMarkNo, Point_XY *calpos)
{
	unsigned bdno = GetVABoardNoFromGlobalCameraNo(cam);
	unsigned inx = 0, retVis = 0, VisErr = 0;
	CString strMsg;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_CA;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._long = 0;
	VIS_CMD[bdno][4]._long = 0;
	VIS_CMD[bdno][5]._long = 0;
	VIS_CMD[bdno][6]._long = 0;
	VIS_CMD[bdno][7]._long = 0;
	VIS_CMD[bdno][8]._long = 0;
	for (inx = 1; inx < 9; ++inx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][inx]._long);
	}
	for (inx = 0; inx < MaxCalJigMarkNo; ++inx)
	{
		VIS_CMD[bdno][9 + (inx * 2)]._dbl = calpos[inx].x;
		VIS_CMD[bdno][10 + (inx * 2)]._dbl = calpos[inx].y;
		strMsg.AppendFormat(_T("%f,%f,"), VIS_CMD[bdno][9 + (inx * 2)]._dbl, VIS_CMD[bdno][10 + (inx * 2)]._dbl);
	}
	VIS_CMD[bdno][9 + (inx * 2)]._long = VIS_EOP;
	VIS_CMD[bdno][10 + (inx * 2)]._long = 0;
	strMsg.AppendFormat(_T("%d,%d"), VIS_CMD[bdno][9 + (inx * 2)]._long, VIS_CMD[bdno][10 + (inx * 2)]._long);
	TriggerEvent(bdno, 11 + (inx * 2), VIS_CMD[bdno][0]._long, strMsg);
	visionResult(bdno, cam + 1, VIS_CA);
	retVis = GetVisionCommandResult(bdno);
	VisErr = GetVisionErrorCode(bdno);
	return retVis;
}

int CEthernetVision::CameraRotateCalibrationPrepare(long cam, double JigPitchX)
{
	unsigned bdno = GetVABoardNoFromGlobalCameraNo(cam);
	unsigned retVis = 0, VisErr = 0;
	CString strMsg;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_RA;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._long = Camera[cam].Win[0][0];
	VIS_CMD[bdno][4]._long = Camera[cam].Win[0][1] + 2 * Camera[cam].Winy;
	VIS_CMD[bdno][5]._long = Camera[cam].Win[1][0];
	VIS_CMD[bdno][6]._long = Camera[cam].Win[1][1] + 2 * Camera[cam].Winy;
	VIS_CMD[bdno][7]._long = Camera[cam].Winx;
	VIS_CMD[bdno][8]._dbl = (-1.0) * JigPitchX;
	VIS_CMD[bdno][9]._long = VIS_EOP;
	VIS_CMD[bdno][10]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d,%d,%f,%d,%d"), 
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, 
		VIS_CMD[bdno][4]._long,	VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long, 
		VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._dbl, 
		VIS_CMD[bdno][9]._long, VIS_CMD[bdno][10]._long);
	TriggerEvent(bdno, 11, VIS_CMD[bdno][0]._long, strMsg);
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] Cam:%d VIS_RA %d %d %d %d %d %d %d %f\n"), cam,
			VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long,
			VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long, VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._dbl);
	}
	visionResult(bdno, cam + 1, VIS_RA);
	retVis = GetVisionCommandResult(bdno);
	VisErr = GetVisionErrorCode(bdno);
	return retVis;
}

int CEthernetVision::CameraRotateCalibration(long cam, long ImageCatch)
{
	unsigned bdno = GetVABoardNoFromGlobalCameraNo(cam);
	unsigned retVis = 0, VisErr = 0;
	CString strMsg;
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_RA;
	VIS_CMD[bdno][1]._long = ImageCatch;		// 1~5
	VIS_CMD[bdno][2]._long = VIS_EOP;
	VIS_CMD[bdno][3]._long = 0;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	TriggerEvent(bdno, 4, VIS_RA, strMsg);
	ThreadSleep(THREAD_VISION_CALIBRATION_WAITTIME);
	visionResult(bdno, cam + 1, VIS_RA);
	retVis = GetVisionCommandResult(bdno);
	VisErr = GetVisionErrorCode(bdno);
	return retVis;
}


long CEthernetVision::GetVisionResultLong(unsigned bdno, unsigned indx)
{
	long Result = 0;
	if (isnan((double)(VIS_RST[bdno][indx]._long))) 
	{ 
		Result = 0; 
	}
	else
	{
		Result = VIS_RST[bdno][indx]._long;
	}
	return Result;
}

double CEthernetVision::GetVisionResultDouble(unsigned bdno, unsigned indx)
{
	double Result = 0.0;
	if (isnan(VIS_RST[bdno][indx]._dbl)) 
	{ 
		Result = 0.00; 
	}
	else if (isinf(VIS_RST[bdno][indx]._dbl)) 
	{ 
		Result = 0.00; 
	}
	else if (abs(VIS_RST[bdno][indx]._dbl) > MAX_VIS_VALID_RANGE) 
	{ 
		Result = 0.0; 
	}
	else
	{
		Result = VIS_RST[bdno][indx]._dbl;
	}
	return Result;
}

bool CEthernetVision::GetVisionBusy(unsigned bdno, long TimeOut)
{
	bool IsBusy = true;
	long Ret = 0;
	Ret = GetVisionResultLong(bdno, 1);
	if (Ret == 1)
		return true;
	else
		return false;
}

long CEthernetVision::GetVisionCommandResult(unsigned bdno)
{
	long Ret = 0;
	Ret = GetVisionResultLong(bdno, 1);
	return Ret;
}

long CEthernetVision::GetVisionErrorCode(unsigned bdno)
{
	long ErrCode = 0;
	ErrCode = GetVisionResultLong(bdno, 2);
	return ErrCode;
}

long CEthernetVision::GetVisionBarcodeSize(unsigned bdno)
{
	long Size = 0;
	Size = GetVisionResultLong(bdno, 3);
	return Size;
}

long CEthernetVision::GetVisionFrmaeNo(unsigned bdno, unsigned FrameCount)
{
	long ErrCode = 0;
	ErrCode = GetVisionResultLong(bdno, 9 + FrameCount);
	return ErrCode;
}

Point_XYRE CEthernetVision::GetVisionMarkResult(unsigned bdno)
{
	Point_XYRE res;
	res.x = GetVisionResultDouble(bdno, 3);
	res.y = GetVisionResultDouble(bdno, 4);
	res.r = GetVisionResultDouble(bdno, 5);
	return res;
}

Point_XYRE CEthernetVision::GetVisionPartResult(unsigned bdno)
{
	Point_XYRE res;
	res.x = GetVisionResultDouble(bdno, 6);
	res.y = GetVisionResultDouble(bdno, 7);
	res.r = GetVisionResultDouble(bdno, 8);
	return res;
}

Point_XYT CEthernetVision::GetVisionPartSizeResult(unsigned bdno)
{
	Point_XYT res;
	res.x = GetVisionResultDouble(bdno, 18);
	res.y = GetVisionResultDouble(bdno, 19);
	return res;
}

void CEthernetVision::ClearVisionResult(unsigned bdno, unsigned indx)
{
	VisResult[bdno][indx].x = 0.00;
	VisResult[bdno][indx].y = 0.00;
	VisResult[bdno][indx].r = 0.00;
}

void CEthernetVision::ClearRunVisionResult(unsigned bdno)
{
	long CamNo = 0, CamChk = 0;
	ZeroMemory(&m_VAResult[bdno], sizeof(m_VAResult[bdno]));
	if (bdno == FRONT_STAGE)
	{
		for (CamNo = CAM1; CamNo <= CAM6; ++CamNo)
		{
			for (CamChk = 0; CamChk < MAXUSEDHEADNO; ++CamChk)
			{
				m_VAPartRecognitionResult[CamNo][CamChk];
			}
		}
	}
	else
	{
		for (CamNo = RCAM1; CamNo <= RCAM8; ++CamNo)
		{
			for (CamChk = 0; CamChk < MAXUSEDHEADNO; ++CamChk)
			{
				m_VAPartRecognitionResult[CamNo][CamChk];
			}
		}
	}
}

void CEthernetVision::ClearRunRecognitionAngle(unsigned bdno)
{
	ZeroMemory(&m_VAAngleInsert[bdno], sizeof(m_VAAngleInsert[bdno]));
}

long CEthernetVision::SetMarkCameraCalibrationJigCenterMark(long cam)
{
	Mark[TMPMARK].InspectMethod = 0;
	Mark[TMPMARK].d1 = 0.0;
	Mark[TMPMARK].d2 = 0.0;
	Mark[TMPMARK].w1 = 0.0;
	Mark[TMPMARK].w2 = 0.0;
	Mark[TMPMARK].a1 = 0.0;
	Mark[TMPMARK].a2 = 0.0;	
	Mark[TMPMARK].Win.x1 = 590;
	Mark[TMPMARK].Win.x2 = 690;
	Mark[TMPMARK].Win.y1 = 430;
	Mark[TMPMARK].Win.y2 = 530;
	Mark[TMPMARK].Type = 0;
	Mark[TMPMARK].Color = MARK_BLACK;
	Mark[TMPMARK].Tolerance = 30;
	Mark[TMPMARK].InspectMethod = 1;

	int ver = 0;
	if (gCMachineConfig->GetCameraVersion(cam, &ver) == NO_ERR)
	{
		if (ver == 1)
		{
			Mark[TMPMARK].Win.x1 = 590;
			Mark[TMPMARK].Win.x2 = 690;
			Mark[TMPMARK].Win.y1 = 462;
			Mark[TMPMARK].Win.y2 = 562;
		}
	}

	return TMPMARK;
}

long CEthernetVision::markTraining(int cam, int uwfileno, int calmark, Point_XYRE* pRes)
{
	long retVis = 0;
	CString strMsg;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam), Err = 0;
	Point_XYRE Res;
	ZeroMemory(&Res, sizeof(Res));
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);	
	VIS_CMD[bdno][0]._long = VIS_FT; // markTraining
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = calmark;
	VIS_CMD[bdno][3]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][4]._long = Mark[uwfileno].Type;
	VIS_CMD[bdno][5]._long = Mark[uwfileno].Color;
	VIS_CMD[bdno][6]._long = Mark[uwfileno].Win.x1;
	VIS_CMD[bdno][7]._long = Mark[uwfileno].Win.y1;
	VIS_CMD[bdno][8]._long = Mark[uwfileno].Win.x2;
	VIS_CMD[bdno][9]._long = Mark[uwfileno].Win.y2;
	VIS_CMD[bdno][10]._long = 0; // Use Camera Calibration Data (0: Use 1: Not Use)
	VIS_CMD[bdno][11]._dbl = 0.0;
	VIS_CMD[bdno][12]._long = Mark[uwfileno].InspectMethod;
	VIS_CMD[bdno][13]._long = VIS_EOP;
	VIS_CMD[bdno][14]._long = 0;
	TRACE(_T("[PWR] markTraining %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d\n"),
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long,
		VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long,
		VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long,
		VIS_CMD[bdno][10]._long,
		VIS_CMD[bdno][11]._dbl,
		VIS_CMD[bdno][12]._long, VIS_CMD[bdno][13]._long, VIS_CMD[bdno][14]._long);
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d"),
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long,
		VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long,
		VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long,
		VIS_CMD[bdno][10]._long,
		VIS_CMD[bdno][11]._dbl,
		VIS_CMD[bdno][12]._long, VIS_CMD[bdno][13]._long, VIS_CMD[bdno][14]._long);
	TriggerEvent(bdno, 15, VIS_FT, strMsg);
	visionResult(bdno, cam + 1, VIS_FT);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	Mark[uwfileno].Area = GetVisionResultLong(bdno, 3);
	VisResult[bdno][0].x = GetVisionResultDouble(bdno, 4);
	VisResult[bdno][0].y = GetVisionResultDouble(bdno, 5);
	VisResult[bdno][0].r = GetVisionResultDouble(bdno, 6);
	Mark[uwfileno].d1 = GetVisionResultDouble(bdno, 7);
	Mark[uwfileno].d2 = GetVisionResultDouble(bdno, 8);
	Mark[uwfileno].w1 = GetVisionResultDouble(bdno, 9);
	Mark[uwfileno].w2 = GetVisionResultDouble(bdno, 10);
	Mark[uwfileno].a1 = GetVisionResultDouble(bdno, 11);
	Mark[uwfileno].a2 = GetVisionResultDouble(bdno, 12);
	Mark[uwfileno].Angle = GetVisionResultDouble(bdno, 13);
	Mark[uwfileno].Threshold = (UBYTE)GetVisionResultDouble(bdno, 14);
	Res = VisResult[bdno][0];
	TRACE(_T("[PWR] markTraining Ret(%d) Err(%d) Area:%d XYR %.3f %.3f %.3f\n"), retVis, Err, Mark[uwfileno].Area, Res.x, Res.y, Res.r);
	if (Err)
	{
		Res.exe = 2;
		Res.x = Res.y = Res.r = 0.000;
	}
	else
		Res.exe = 1;
	*pRes = Res;
	return retVis;
}

Point_XYRE CEthernetVision::markTrainingWithoutCamCal(int cam, int uwfileno, int calmark, int showrst, int useCal)
{
	long retVis = 999;
	CString strMsg;
	Point_XYRE Res;
	ZeroMemory(&Res, sizeof(Res));
	long bdno = GetVABoardNoFromGlobalCameraNo(cam), Err = 0;
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);	
	VIS_CMD[bdno][0]._long = VIS_FT; // Camera Calibration
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = calmark;
	VIS_CMD[bdno][3]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][4]._long = Mark[uwfileno].Type;
	VIS_CMD[bdno][5]._long = Mark[uwfileno].Color;
	VIS_CMD[bdno][6]._long = Mark[uwfileno].Win.x1; 
	VIS_CMD[bdno][7]._long = Mark[uwfileno].Win.y1;
	VIS_CMD[bdno][8]._long = Mark[uwfileno].Win.x2; 
	VIS_CMD[bdno][9]._long = Mark[uwfileno].Win.y2;
	VIS_CMD[bdno][10]._long = useCal;
	VIS_CMD[bdno][11]._dbl = 0.0;
	VIS_CMD[bdno][12]._long = Mark[uwfileno].InspectMethod;
	VIS_CMD[bdno][13]._long = VIS_EOP;
	VIS_CMD[bdno][14]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d"), 
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long,
		VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long,
		VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long,
		VIS_CMD[bdno][10]._long, 
		VIS_CMD[bdno][11]._dbl, 
		VIS_CMD[bdno][12]._long, VIS_CMD[bdno][13]._long, VIS_CMD[bdno][14]._long);
	TriggerEvent(bdno, 15, VIS_FT, strMsg);
	visionResult(bdno, cam + 1, VIS_FT);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	Mark[uwfileno].Area = GetVisionResultLong(bdno, 3);
	VisResult[bdno][0].x = GetVisionResultDouble(bdno, 4);
	VisResult[bdno][0].y = GetVisionResultDouble(bdno, 5);
	Res = VisResult[bdno][0];
	return Res;
}

Point_XYRE CEthernetVision::GetVisionMarkResult(unsigned bdno, unsigned index)
{
	return VisResult[bdno][index];
}

long CEthernetVision::prepareMarkRecognition(long cam, long FileNo, long MarkNo)
{
	long retVis = 0;
	CString strMsg;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam), Err = 0;
	ClearVisionCommand(bdno);	
	VIS_CMD[bdno][0]._long = VIS_FI;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = MarkNo;
	VIS_CMD[bdno][3]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][4]._long = FileNo;
	VIS_CMD[bdno][5]._dbl = 0.0;
	VIS_CMD[bdno][6]._long = VIS_EOP;
	VIS_CMD[bdno][7]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%f,%d,%d"), 
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long,
		VIS_CMD[bdno][5]._dbl, VIS_CMD[bdno][6]._long, VIS_CMD[bdno][7]._long);
	TriggerEvent(bdno, 8, VIS_FI, strMsg);
	visionResult(bdno, cam + 1, VIS_FI);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::catchMarkImage(long cam)
{
	CString strMsg;
	long retVis = 0, Err = 0;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionCommand(bdno);	
	VIS_CMD[bdno][0]._long = VIS_FI;
	VIS_CMD[bdno][1]._long = 1;
	VIS_CMD[bdno][2]._long = VIS_EOP;
	VIS_CMD[bdno][3]._long = 0;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	TriggerEvent(bdno, 4, VIS_FI, strMsg);
	visionResult(bdno, cam + 1, VIS_FI);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::getMarkRecognition(long cam, Point_XYRE* res)
{
	long retVis = 0, Err = 0;
	CString strMsg;
	bool bErrX, bErrY, bErrR, bErrZ;
	bErrX = bErrY = bErrR = bErrZ = false;
	res->exe = 0;
	res->x = res->y = res->r = res->z = 0.0;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_FI;
	VIS_CMD[bdno][1]._long = 2;
	VIS_CMD[bdno][2]._long = VIS_EOP;
	VIS_CMD[bdno][3]._long = 0;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	TriggerEvent(bdno, 4, VIS_FI, strMsg);
	visionResult(bdno, cam + 1, VIS_FI);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	if (Err)
	{
		TRACE(_T("[PWR] getMarkRecognition Cam:%d Ret(%d) Err(%d)\n"), cam, retVis, Err);
	}
	if (retVis == 0)
	{
		*res = GetVisionMarkResult(bdno);
		res->exe = 1;
		if (isnan(res->x)) { res->x = 0.00; Err = 999; }
		if (isnan(res->y)) { res->y = 0.00; Err = 999; }
		if (isnan(res->r)) { res->r = 0.00; Err = 999; }
		if (isnan(res->z)) { res->z = 0.00; Err = 999; }
		if (isinf(res->x)) { res->x = 0.00; Err = 999; }
		if (isinf(res->y)) { res->y = 0.00; Err = 999; }
		if (isinf(res->r)) { res->r = 0.00; Err = 999; }
		if (isinf(res->z)) { res->z = 0.00; Err = 999; }
		if (abs(res->x) > MAX_VIS_VALID_RANGE) { res->x = 0.0; Err = 998; bErrX = true; }
		if (abs(res->y) > MAX_VIS_VALID_RANGE) { res->y = 0.0; Err = 998; bErrY = true;	}
		if (abs(res->r) > MAX_VIS_VALID_RANGE) { res->r = 0.0; /*err = 998;*/ bErrR = true; }
		if (abs(res->z) > MAX_VIS_VALID_RANGE) { res->z = 0.0; bErrZ = true; }
		if (Err == 999)
		{
			res->exe = 0;
			TRACE(_T("[PWR] <Vision> Mark Result = NaN or INF\n"));
		}
		if (Err == 998)
		{
			res->exe = 0;
			if (bErrX == true)
				TRACE(_T("[PWR] <Vision> Mark Result X = Over valid range\n"));
			if (bErrY == true)
				TRACE(_T("[PWR] <Vision> Mark Result Y = Over valid range\n"));
		}

		return retVis;

	}
	else if (Err == 104)
	{	// Vision Result Packet Error
		visionESC(bdno, 0);
		res->x = res->y = res->r = res->z = 0.00;
		res->exe = 0;
	}
	else
	{
		res->x = res->y = res->r = res->z = 0.00;
		res->exe = 0;
	}
	return PCB_FIDUCIALMARK_RECOGNITION;
}

long CEthernetVision::inspectMachCalRefMark(int CameraNo, long MarkNo, Point_XYRE* res)
{
	long err = 0;
	err = prepareMarkRecognition(CameraNo, MarkNo, 0);
	ThreadSleep(TIME20MS);
	err = catchMarkImage(CameraNo);
	ThreadSleep(TIME10MS);
	err = getMarkRecognition(CameraNo, res);
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] inspectMachCalRefMark Cam:%d Err:%d XYR:%.3f %.3f %.3f\n"), CameraNo, err, res->x, res->y, res->r);
	}
	return err;
}

long CEthernetVision::inspectMachCalMark(int CameraNo, long MarkNo, Point_XYRE* res)
{
	long err = 0;
	err = prepareMarkRecognition(CameraNo, MarkNo, 0);
	ThreadSleep(TIME20MS);
	err = catchMarkImage(CameraNo);
	ThreadSleep(TIME10MS);
	err = getMarkRecognition(CameraNo, res);
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] inspectMachCalMark Cam:%d Err:%d XYR:%.3f %.3f %.3f\n"), CameraNo, err, res->x, res->y, res->r);
	}
	return err;
}

long CEthernetVision::inspectMark(int CameraNo, long MarkNo, Point_XYRE* res)
{
	long err = 0;
	err = prepareMarkRecognition(CameraNo, MarkNo, 0);
	ThreadSleep(TIME20MS);
	err = catchMarkImage(CameraNo);
	ThreadSleep(TIME10MS);
	err = getMarkRecognition(CameraNo, res);
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] inspectMark Cam:%d Err:%d XYR:%.3f %.3f %.3f\n"), CameraNo, err, res->x, res->y, res->r);
	}
	return err;
}

long CEthernetVision::inspectROriginMark(int CameraNo, long MarkNo, Point_XYRE* res)
{
	long err = 0;
	err = prepareMarkRecognition(CameraNo, MarkNo, 4);
	ThreadSleep(TIME20MS);
	err = catchMarkImage(CameraNo);
	ThreadSleep(TIME10MS);
	err = getMarkRecognition(CameraNo, res);
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] inspectROriginMark Cam:%d Err:%d XYR:%.3f %.3f %.3f\n"), CameraNo, err, res->x, res->y, res->r);
	}
	return err;
}

int CEthernetVision::WindowMoving(int CameraNo, int Window, int Show, int Control, int pixel, int dir)
{
	int retVis = 999;
	CString strMsg;
	long bdno = GetVABoardNoFromGlobalCameraNo(CameraNo);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_WC;
	VIS_CMD[bdno][1]._long = Window;
	VIS_CMD[bdno][2]._long = Show;
	VIS_CMD[bdno][3]._long = Control;
	VIS_CMD[bdno][4]._long = 0 + 100;
	VIS_CMD[bdno][5]._long = 0 + 100;
	VIS_CMD[bdno][6]._long = 1280 - 100;
	VIS_CMD[bdno][7]._long = 960 - 100;
	VIS_CMD[bdno][8]._long = 0 + 100;
	VIS_CMD[bdno][9]._long = 0 + 100;
	VIS_CMD[bdno][10]._long = 1280 - 100;
	VIS_CMD[bdno][11]._long = 960 - 100;
	VIS_CMD[bdno][12]._long = VIS_EOP;
	VIS_CMD[bdno][13]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"),
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long,
		VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long,
		VIS_CMD[bdno][7]._long, VIS_CMD[bdno][8]._long, VIS_CMD[bdno][9]._long,
		VIS_CMD[bdno][10]._long, VIS_CMD[bdno][11]._long, VIS_CMD[bdno][12]._long, 
		VIS_CMD[bdno][13]._long);
	TriggerEvent(bdno, 14, VIS_WC, strMsg);
	retVis = visionResult(bdno, CameraNo + 1, VIS_WC);
	return retVis;
}

long CEthernetVision::PrepareCommand(long t, long* cam, long id, long* chk, long* useVA, long* dbNo, double* angleVA, long* Forming)
{
	int retVis = 999;
	CString strMsg, strMsgI, strMsgF;
	long bdno = GetVABoardNoFromGlobalCameraNo(cam[0]), Err = 0;
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	long i, varecognum, pcDepth;
	varecognum = PCVARECOGNUM;
	pcDepth = 1;
	VIS_CMD[bdno][0]._long = VIS_QPVA;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = id;
	for (i = 0; i < MAXVAHEAD; i++)
	{
		if (useVA[i] == NON)
		{
			VIS_CMD[bdno][3 + i]._long = 0;
			VIS_CMD[bdno][3 + varecognum + i]._long = 0;					/* Only Center	*/
			VIS_CMD[bdno][3 + varecognum * 2 + i]._dbl = 0.0;				/* Depth for 3D cal */
			VIS_CMD[bdno][3 + varecognum * 3 + i]._long = 0;	/* DB Num */
			VIS_CMD[bdno][3 + varecognum * 4 + i]._dbl = 0.0;	/* Checking Algle */
			VIS_CMD[bdno][3 + varecognum * 5 + i + 4]._long = 0;
			continue;
		}
		VIS_CMD[bdno][3 + i]._long = GetVACameraNoFromGlobalCameraNo(cam[i]);
		VIS_CMD[bdno][3 + varecognum + i]._long = chk[i];
		VIS_CMD[bdno][3 + varecognum * 2 + i]._dbl = 0.0;
		VIS_CMD[bdno][3 + varecognum * 3 + i]._long = dbNo[i];
		VIS_CMD[bdno][3 + varecognum * 4 + i]._dbl = angleVA[i];
		VIS_CMD[bdno][3 + varecognum * 5 + i + 4]._long = Forming[i];
	}

	//no = 3 + varecognum * (4 + pcDepth);

	//if (VADivide[t][idx[0]] == 8)
	//{
	//	if (VA[VADBNum[t][idx[0]]].wDivideX == 4 && VA[VADBNum[t][idx[0]]].wDivideY == 2)
	//	{
	//		if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* Y방향 이동 */
	//		{
	//			VIS_CMD[bdno][no] = 2;
	//			VIS_CMD[bdno][no + 1] = 4;
	//			VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.5;
	//			VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.25;
	//		}
	//		else
	//		{
	//			VIS_CMD[bdno][no] = 4;
	//			VIS_CMD[bdno][no + 1] = 2;
	//			VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.25;
	//			VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.5;
	//		}
	//	}
	//	else if (VA[VADBNum[t][idx[0]]].wDivideX == 2 && VA[VADBNum[t][idx[0]]].wDivideY == 4)
	//	{
	//		if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* Y방향 이동 */
	//		{
	//			VIS_CMD[bdno][no] = 4;
	//			VIS_CMD[bdno][no + 1] = 2;
	//			VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.25;
	//			VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.5;
	//		}
	//		else
	//		{
	//			VIS_CMD[bdno][no] = 2;
	//			VIS_CMD[bdno][no + 1] = 4;
	//			VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.5;
	//			VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.25;
	//		}
	//	}
	//	else
	//	{
	//		// 20180314 HarkDo for New divide 8	(1x8)
	//		if (VA2DExtRecogMethod[t][idx[0]] == 1)
	//		{
	//			if (VA[VADBNum[t][idx[0]]].Size.x > VA[VADBNum[t][idx[0]]].Size.y)
	//			{
	//				if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* Y방향 이동 */
	//				{
	//					VIS_CMD[bdno][no] = 1;
	//					VIS_CMD[bdno][no + 1] = 8;//VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//					VIS_CMDF[bdno][no + 2] = 0.0;//VA[VADBNum[t][idx[0]]].Size.y*0.5;
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.125;
	//				}
	//				else	/* X방향 이동 */
	//				{
	//					VIS_CMD[bdno][no] = 8;//VADivide[t][idx[0]]; 
	//					VIS_CMD[bdno][no + 1] = 1;	    // 2,3분할시 0을 1로 변경. ytlee.
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.125;
	//					VIS_CMDF[bdno][no + 3] = 0.0;
	//				}
	//			}
	//			else
	//			{
	//				if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* X방향 이동 */
	//				{
	//					VIS_CMD[bdno][no] = 8;//VADivide[t][idx[0]]; 
	//					VIS_CMD[bdno][no + 1] = 1;    // 2,3분할시 0을 1로 변경. ytlee.
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.125;
	//					VIS_CMDF[bdno][no + 3] = 0.0;
	//				}
	//				else	/* Y방향 이동 */
	//				{
	//					VIS_CMD[bdno][no] = 1;
	//					VIS_CMD[bdno][no + 1] = 8;//VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//					VIS_CMDF[bdno][no + 2] = 0.0;
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.125;
	//				}
	//			}
	//		}
	//	}
	//}
	//else if (VADivide[t][idx[0]] == 4)
	//{
	//	if (VA2DExtRecogMethod[t][idx[0]] == 1)	// 20171107 HarkDo for New divide 4	(1x4)
	//	{
	//		if (VA[VADBNum[t][idx[0]]].Size.x > VA[VADBNum[t][idx[0]]].Size.y)
	//		{
	//			if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* Y방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 2;
	//				VIS_CMD[bdno][no + 1] = 2;//VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = 0.0;//VA[VADBNum[t][idx[0]]].Size.y*0.5;
	//				VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.25;
	//			}
	//			else	/* X방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 2;//VADivide[t][idx[0]]; 
	//				VIS_CMD[bdno][no + 1] = 2;	    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.25;
	//				VIS_CMDF[bdno][no + 3] = 0.0;
	//			}
	//		}
	//		else
	//		{
	//			if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* X방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 2;//VADivide[t][idx[0]]; 
	//				VIS_CMD[bdno][no + 1] = 2;    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.25;
	//				VIS_CMDF[bdno][no + 3] = 0.0;
	//			}
	//			else	/* Y방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 2;
	//				VIS_CMD[bdno][no + 1] = 2;//VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = 0.0;
	//				VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.25;
	//			}
	//		}
	//	}
	//	else // 20171107 HarkDo for Normal divide 4(2x2)
	//	{
	//		VIS_CMD[bdno][no] = 2;
	//		VIS_CMD[bdno][no + 1] = 2;
	//		VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.5;
	//		VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.5;
	//	}
	//}
	//else if (VADivide[t][idx[0]] > 1)
	//{
	//	// 20180403 HarkDo for Freeze Polarity Recognition
	//	if (VA2DExtFreezePolarity[t][idx[0]] == 1)
	//	{
	//		if (mode == 1)
	//		{
	//			if (uwStep == 0 && (gFormingData[VAFormingDB[t][0]].dwUsage[0] > 0 || gFormingData[VAFormingDB[t][0]].dwUsage[1] > 0) && (chkVisVersionForming(bdno) == OK))
	//				VIS_CMD[bdno][no] = 100 + VADivide[t][idx[0]];
	//			else
	//				VIS_CMD[bdno][no] = VADivide[t][idx[0]];
	//		}
	//		else
	//		{
	//			if (uwStep == 0 && (gFormingData[VADBNum[t][idx[0]]].dwUsage[0] > 0 || gFormingData[VADBNum[t][idx[0]]].dwUsage[1] > 0) && (chkVisVersionForming(bdno) == OK))
	//				VIS_CMD[bdno][no] = 100 + VADivide[t][idx[0]];
	//			else
	//				VIS_CMD[bdno][no] = VADivide[t][idx[0]];
	//		}
	//		VIS_CMD[bdno][no + 1] = 1;
	//		VIS_CMDF[bdno][no + 2] = 0.0;
	//		VIS_CMDF[bdno][no + 3] = 0.0;
	//	}
	//	else
	//	{
	//		if (VA[VADBNum[t][idx[0]]].Size.x > VA[VADBNum[t][idx[0]]].Size.y)
	//		{
	//			if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* Y방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 1;
	//				VIS_CMD[bdno][no + 1] = VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = 0.0;
	//				if (VADivide[t][idx[0]] == 2)
	//				{
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.5;
	//				}
	//				else
	//				{
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.x * 0.3333333333;
	//				}
	//			}
	//			else	/* X방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = VADivide[t][idx[0]];
	//				VIS_CMD[bdno][no + 1] = 1;	    // 2,3분할시 0을 1로 변경. ytlee.
	//				if (VADivide[t][idx[0]] == 2)
	//				{
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.5;
	//				}
	//				else
	//				{
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.x * 0.3333333333;
	//				}
	//				VIS_CMDF[bdno][no + 3] = 0.0;
	//			}
	//		}
	//		else
	//		{
	//			if (iround(VAAngle[t][idx[0]] / 90.0) % 2)	/* X방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = VADivide[t][idx[0]];
	//				VIS_CMD[bdno][no + 1] = 1;    // 2,3분할시 0을 1로 변경. ytlee.
	//				if (VADivide[t][idx[0]] == 2)
	//				{
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.5;
	//				}
	//				else
	//				{
	//					VIS_CMDF[bdno][no + 2] = VA[VADBNum[t][idx[0]]].Size.y * 0.3333333333;
	//				}
	//				VIS_CMDF[bdno][no + 3] = 0.0;
	//			}
	//			else	/* Y방향 이동 */
	//			{
	//				VIS_CMD[bdno][no] = 1;
	//				VIS_CMD[bdno][no + 1] = VADivide[t][idx[0]];    // 2,3분할시 0을 1로 변경. ytlee.
	//				VIS_CMDF[bdno][no + 2] = 0.0;
	//				if (VADivide[t][idx[0]] == 2)
	//				{
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.5;
	//				}
	//				else
	//				{
	//					VIS_CMDF[bdno][no + 3] = VA[VADBNum[t][idx[0]]].Size.y * 0.3333333333;
	//				}
	//			}
	//		}
	//	}
	//}
	//else
	{
		//VIS_CMD[bdno][no]._long = 100;		// Use Forming
		//VIS_CMD[bdno][no]._long = 0;
		//VIS_CMD[bdno][no + 1]._long = 0;
		//VIS_CMD[bdno][no + 2]._dbl = 0.0;
		//VIS_CMD[bdno][no + 3]._dbl = 0.0;
		//TRACE(_T("[PWR] no(%03d):%d no+1(%03d):%d no+2(%03d):%.3f no+3(%03d):%.3f\n"),
		//	no, VIS_CMD[bdno][no]._long,
		//	no + 1, VIS_CMD[bdno][no + 1]._long,
		//	no + 2, VIS_CMD[bdno][no + 2]._dbl,
		//	no + 3, VIS_CMD[bdno][no + 3]._dbl);
	}
	VIS_CMD[bdno][103]._long = VIS_EOP;
	VIS_CMD[bdno][104]._long = 0;
	strMsg.Format(_T("%d,%d,"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long);
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][3 + indx]._long);
	}
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][19 + indx]._long);
	}
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%.3f,"), VIS_CMD[bdno][35 + indx]._dbl);
	}
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][51 + indx]._long);
	}
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%.3f,"), VIS_CMD[bdno][67 + indx]._dbl);
	}
	strMsg.AppendFormat(_T("%d,%d,%.3f,%.3f,"),
		VIS_CMD[bdno][83]._long, VIS_CMD[bdno][84]._long, VIS_CMD[bdno][85]._dbl, VIS_CMD[bdno][86]._dbl);
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][87 + indx]._long);
	}
	strMsg.AppendFormat(_T("%d,%d"), VIS_CMD[bdno][103]._long, VIS_CMD[bdno][104]._long);
	TriggerEvent(bdno, 105, VIS_QPVA, strMsg);
	ThreadSleep(TIME2MS);
	if (visionResult(bdno, cam[0] + 1, VIS_QPVA) < 0)
	{
		return -2;
	}
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::ImageCatch(long t, long* cam, long id, long* chk, long* useVA, long uwDiv)
{
	long retVis = 999, Err = 0;
	long bdno, i;
	long varecognum, no;
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam[0]);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	varecognum = PCVARECOGNUM;
	memset(VIS_CMD[bdno], 0, MAXVISMSGLENGTH);
	VIS_CMD[bdno][0]._long = VIS_QPVA; 
	VIS_CMD[bdno][1]._long = 1;
	VIS_CMD[bdno][2]._long = id;
	for (i = 0; i < MAXVAHEAD; i++)
	{
		if (useVA[i] == NON)
		{ 
			VIS_CMD[bdno][3 + i]._long = 0; 
			VIS_CMD[bdno][3 + varecognum + i]._long = CHK_SIDE;
			continue; 
		}
		VIS_CMD[bdno][3 + i]._long = GetVACameraNoFromGlobalCameraNo(cam[i]);
		VIS_CMD[bdno][3 + varecognum + i]._long = chk[i]; //VAChkPos[t][idx[i]]; /* Checking Position */
	}
	no = 3 + varecognum * 2;
	//if (VADivide[t][idx[0]] == 0) 
	VIS_CMD[bdno][no++]._long = uwDiv; // Divide Capture Count
	//else					   
	//	VIS_CMD[bdno][no++] = uwDiv + 1;
	VIS_CMD[bdno][no++]._long = VIS_EOP;
	VIS_CMD[bdno][no]._long = 0;
	strMsg.Format(_T("%d,%d,"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long);
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][3 + indx]._long);
	}
	for (int indx = 0; indx < 16; ++indx)
	{
		strMsg.AppendFormat(_T("%d,"), VIS_CMD[bdno][19 + indx]._long);
	}
	strMsg.AppendFormat(_T("%d,%d,%d"), VIS_CMD[bdno][35]._long, VIS_CMD[bdno][36]._long, VIS_CMD[bdno][37]._long);
	TriggerEvent(bdno, 38, VIS_QPVA, strMsg);
	if(visionResult(bdno, cam[0] + 1, VIS_QPVA) < 0)
	{
		return -2;
	}
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

long CEthernetVision::StartProcess(long cam, long id)
{
	long retVis = 999, bdno, Err = 0;
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_QPVA;
	VIS_CMD[bdno][1]._long = 2;
	VIS_CMD[bdno][2]._long = id;
	VIS_CMD[bdno][3]._long = VIS_EOP;
	VIS_CMD[bdno][4]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, 5, VIS_QPVA, strMsg);
	if(visionResult(bdno, cam + 1, VIS_QPVA) < 0)
	{
		return -2;
	}
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

/*
//	cam	: 123/456/7/89
//	loc	: 0,1,2,3
//	res	: Point_XYRE *Result (0=Not Ready, 1=Ready(OK), 2=Error)
*/
long CEthernetVision::GetRecognitionResult(long cam, long loc, long id, long* err, Point_XYRE* res, long dwFrameNo[], Point_XYT* ptXYRESize)
{
	int TimeOut = 0;
	long retVis = 999;
	long bdno, VisErr = 0;
	CString strMsg;
	Point_XYRE VisRes;
	Point_XYT SizeRes;
	long FrameNo[MAXFRAMENO];
	ZeroMemory(&FrameNo, sizeof(FrameNo));
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_QPVA;
	VIS_CMD[bdno][1]._long = 3;
	VIS_CMD[bdno][2]._long = id;
	VIS_CMD[bdno][3]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][4]._long = loc;
	VIS_CMD[bdno][5]._long = VIS_EOP;
	VIS_CMD[bdno][6]._long = 0;
	strMsg.Format(_T("%d,%d,%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long, VIS_CMD[bdno][5]._long, VIS_CMD[bdno][6]._long);
	TriggerEvent(bdno, 7, VIS_QPVA, strMsg);
	ThreadSleep(TIME2MS);
	TimeOut = visionResult(bdno, cam + 1, VIS_QPVA);
	retVis = GetVisionCommandResult(bdno);
	VisErr = GetVisionErrorCode(bdno);
	VisRes = GetVisionPartResult(bdno);
	SizeRes.x = SizeRes.y = GetVisionPartPitchResult(bdno);
	for (long FrameCount = 0; FrameCount < MAXFRAMENO; ++FrameCount)
	{
		FrameNo[FrameCount] = GetVisionFrmaeNo(bdno, FrameCount);
	}
	if (VisErr || TimeOut < 0)
	{
		VisRes.x = VisRes.y = VisRes.r = 0.000;
		VisRes.exe = 2;
		if (TimeOut < 0)
		{
			VisErr = 999;
		}
	}
	else
	{
		if (isnan(VisRes.x)) { VisRes.x = 0.00; VisErr = 999; }
		if (isnan(VisRes.y)) { VisRes.y = 0.00; VisErr = 999; }
		if (isnan(VisRes.r)) { VisRes.r = 0.00; VisErr = 999; }
		if (isinf(VisRes.x)) { VisRes.x = 0.00; VisErr = 999; }
		if (isinf(VisRes.y)) { VisRes.y = 0.00; VisErr = 999; }
		if (isinf(VisRes.r)) { VisRes.r = 0.00; VisErr = 999; }
		if (VisErr == 0)
		{
			VisRes.exe = 1;
		}
		else
		{
			VisRes.x = VisRes.y = VisRes.r = 0.000;
			VisRes.exe = 2;
		}
	}
	*ptXYRESize = SizeRes;
	*res = VisRes;
	*err = VisErr;

	if (TimeOut < 0)
	{
		retVis = -2;
	}

	return retVis;
}

/*
*/
long CEthernetVision::SendCameraRecognitionOffset(long cam, Point_XY CamOffset1, Point_XY CamOffset2, Point_XY CamOffset3)
{
	long retVis = 999;
	long bdno, Err = 0;
	CString strMsg;
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_CAMOFFSET;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._dbl = CamOffset1.x;	// CHK_CENT
	VIS_CMD[bdno][4]._dbl = CamOffset1.y;	// CHK_CENT
	VIS_CMD[bdno][5]._dbl = CamOffset2.x;	// CHK_CENT2
	VIS_CMD[bdno][6]._dbl = CamOffset2.y;	// CHK_CENT2
	VIS_CMD[bdno][7]._dbl = CamOffset3.x;	// CHK_CENT3
	VIS_CMD[bdno][8]._dbl = CamOffset3.y;	// CHK_CENT3
	VIS_CMD[bdno][9]._long = VIS_EOP;
	VIS_CMD[bdno][10]._long = 0;

	strMsg.Format(_T("%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d"),
		VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, 
		VIS_CMD[bdno][3]._dbl, VIS_CMD[bdno][4]._dbl, 
		VIS_CMD[bdno][5]._dbl, VIS_CMD[bdno][6]._dbl, 
		VIS_CMD[bdno][7]._dbl, VIS_CMD[bdno][8]._dbl,
		VIS_CMD[bdno][9]._long, VIS_CMD[bdno][10]._long);
	TRACE(_T("[PWR] SendCameraRecognitionOffset Cam%02d Offset XY1 %.3f %.3f XY2 %.3f %.3f XY3 %.3f %.3f\n"), cam,
		CamOffset1.x, CamOffset1.y,	CamOffset2.x, CamOffset2.y, CamOffset3.x, CamOffset3.y);
	TriggerEvent(bdno, 11, VIS_CAMOFFSET, strMsg);
	ThreadSleep(TIME2MS);
	visionResult(bdno, cam + 1, VIS_CAMOFFSET);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

/*
*/
long CEthernetVision::InspectBarcode(long cam)
{
	long retVis = 999;
	long bdno, Err = 0, Size = 0;
	CString strMsg, strBarcode;
	char Barcode[BUFSIZE];
	ZeroMemory(&Barcode, sizeof(Barcode));
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_BARCODE;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._long = VIS_EOP;
	strMsg.Format(_T("%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long);
	TriggerEvent(bdno, 4, VIS_BARCODE, strMsg);
	ThreadSleep(TIME2MS);
	visionResult(bdno, cam + 1, VIS_BARCODE);
	Size = GetVisionBarcodeSize(bdno);
	TRACE(_T("[PWR] InspectBarcode Cam%02d Size:%d\n"), cam, Size);
	if (Size > 0)
	{
		for (long index = 0; index < Size; ++index)
		{
			Barcode[index] = (char)GetVisionResultLong(bdno, 4 + index);
			strBarcode.AppendFormat(_T("%c"), Barcode[index]);
		}
	}
	TRACE(_T("[PWR] Barcode:%s\n"), strBarcode);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

/*
*/
long CEthernetVision::MarkPairRecognition(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, Ratio_XYRZ RatioXY)
{
	long cam, repeat, correct, Err = NO_ERR;
	long nextdelay = 100, ret, moredelay = 100;
	Point_XYRE res;
	long settleerror = 0, viserror = 0, doneflag = 0;
	long nextMark = 0;
	Point_XY Mark1, Mark2, Cur;
	double TargetZ = GetStandByZ(Gantry), TargetR = GetStandByR(Gantry);
	double Ratio = RatioXY.xy, InposXY = 0.010, PositionZ = 0.0;
	ULONGLONG GetTime = 0, Elapsed = 0;
	long Ms = TIME30MS, TimeOut = TIME5000MS;
	bool bSaftyZ = false;
	Mark1 = Mark1Pos;
	Mark2 = Mark2Pos;
	double SpecIn = 0.020;

	ZeroMemory(&res, sizeof(res));
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] Mark1 XY,%.3f,%.3f Mark2 XY,%.3f,%.3f\n"), Mark1.x, Mark1.y, Mark2.x, Mark2.y);
	}

	if (Gantry == FRONT_GANTRY)
		cam = FHCAM;
	else
		cam = RHCAM;

	if (Mark1No < 1)
	{
		TRACE(_T("[PWR] MarkPairRecognition Mark1No is under 1\n"));
		return 0;
	}

	Mark1No--;
	if (Mark2No < 1)
	{
		TRACE(_T("[PWR] MarkPairRecognition Mark2No is under 1, Copy Mark1 to Mark2\n"));
		Mark2 = Mark1;
	}

	MarkDelta[Gantry][MK_1].Delta.x = MarkDelta[Gantry][MK_1].Delta.y = MarkDelta[Gantry][MK_1].Delta.r = 0.00;
	MarkDelta[Gantry][MK_2].Delta.x = MarkDelta[Gantry][MK_2].Delta.y = MarkDelta[Gantry][MK_2].Delta.r = 0.00;

	repeat = 0;
	correct = 0;
	moredelay = 30;		// Settling Delay


	if (IsAccTest() == true)
	{
		Err = LinearIntplPosWaitDelayedPosSet(Gantry, cam, Mark1, Ratio, InposXY, Ms, TimeOut);
		if (Err != NO_ERR)
		{
			return Err;
		}

		Err = LinearIntplPosWaitDelayedPosSet(Gantry, cam, Mark2, Ratio, InposXY, Ms, TimeOut);

		return Err;
	}

	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		PositionZ = ReadPosition(GetZAxisFromHeadNo(Gantry, HeadNo + 1));
		if (PositionZ > SAFTY_ZHEIGHT)
		{
			bSaftyZ = true;
			TRACE(_T("[PWR] MarkRecognition ZZZZZZZZZZZZZZZZ(%d) Position:%.3f\n"), HeadNo + 1, PositionZ);
			break;
		}
	}
	if (bSaftyZ == true)
	{
		GetTime = _time_get();
		MoveZStandy(Gantry, TargetZ, Ratio);
		TRACE(_T("[PWR] MarkRecognition MoveZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] Mark1(%d) Led On(%03d,%03d,%03d)\n"), Mark1No,
			Mark[Mark1No].LEDLevel[0], Mark[Mark1No].LEDLevel[1], Mark[Mark1No].LEDLevel[2]);
	}
	gLedOn(cam, Mark[Mark1No].LEDLevel[0], Mark[Mark1No].LEDLevel[1], Mark[Mark1No].LEDLevel[2]);
	if(gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE("[PWR] Gantry:%d P1 XY:%.3f %.3f P2 XY:%.3f %.3f\n", Gantry, Mark1Pos.x, Mark1Pos.y, Mark2Pos.x, Mark2Pos.y);
	}

	FOREVER
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] MarkPairRecognition-1 GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			return Err;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] MarkPairRecognition-1 GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			return Err;
		}
		res.x = res.y = 0.0;
		do
		{
			GetTime = _time_get();
			Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
			if (_time_elapsed(GetTime) > 0)
			{
				TRACE(_T("[PWR] MarkPairRecognition WaitGantryIdle-1 Elasped,%d\n"), _time_elapsed(GetTime));
			}
			if (Err != NO_ERR)
			{
				return Err;
			}
			Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, cam, Mark1, Ratio, InposXY, Ms, TimeOut);
			if (Err != NO_ERR)
			{
				return Err;
			}
			prepareMarkRecognition(cam, Mark1No, 0);
			ThreadSleep(TIME20MS + moredelay);
			if (gcPowerLog->IsShowVisionLog() == true)
			{
				Cur = gReadGantryPosition(FRONT_GANTRY);
				TRACE(_T("[PWR] <Mark Pos> (%s) Real XY (%7.3f, %7.3f),(Cmd Pos = %7.3f, %7.3f)\n"), (Gantry == FRONT_GANTRY) ? _T("Front") : _T("Rear"),
					Cur.x, Cur.y, Mark1.x, Mark1.y);
			}
			Cur = gReadGantryPosition(FRONT_GANTRY);
			catchMarkImage(cam);
			ThreadSleep(TIME10MS);
			ret = getMarkRecognition(cam, &res);
			if (gcPowerLog->IsShowVisionLog() == true)
			{
				TRACE(_T("[PWR] [Gantry=%d] 1st Mark Capture ResXYRE,%.3f,%.3f,%.3f,%d\n"), Gantry, res.x, res.y, res.r, res.exe);
			}
			if (ret != 0) viserror++;
			else if ((fabs(Mark1.x - Cur.x) >= SpecIn || fabs(Mark1.y - Cur.y) >= SpecIn)) {		// 2006.1.23
				settleerror++;
				moredelay += 20;
				ret = PCB_FIDUCIALMARK_RECOGNITION;
				TRACE(_T("[PWR] [Gantry=%d] XY Settling is NOT Sufficient during Fiducial 1st Mark Capture\n"), Gantry);
				TRACE(_T("[PWR] [ERR] TargetXY,%.3f,%.3f CurXY,%.3f,%.3f => DiffXY,%.3f,%.3f\n"), Mark1.x, Mark1.y, Cur.x, Cur.y, Mark1.x - Cur.x, Mark1.y - Cur.y);
			}
			else break;
			gLedOn(cam, Mark[Mark1No].LEDLevel[0], Mark[Mark1No].LEDLevel[1], Mark[Mark1No].LEDLevel[2]);
			if (settleerror >= 10 || viserror >= 3)
			{
				doneflag = 1;
				break;
			}
		} while (doneflag == 0);

		if (doneflag)
		{
			Point_XY pre;
			ThreadSleep(TIME200MS);
			Cur = gReadGantryPosition(FRONT_GANTRY);
			pre.x = Cur.x;
			pre.y = Cur.y;
			TRACE(_T("[PWR] <Mark> Fiducial MARK 1 NOT FOUND ERROR !!! (%s)\n"), ((Gantry == 0) ? _T("Front") : _T("Rear")));
			break;
		}
		else // Mark OK
		{
			break;
		}
		// Repeated Error -> Skip.
		//if (++repeat > 1 && correct == 1)
		//{
		//	break;
		//}
	}
	if (Err == STOP_NOW)
	{
		TRACE(_T("[PWR] MarkPairRecognition StopNow\n"));
		return Err;
	}
	// Final Mark 1 Result.
	MarkDelta[Gantry][MK_1].Delta.x += res.x;
	MarkDelta[Gantry][MK_1].Delta.y += res.y;
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] Gantry:%d 1st MarkDelta XY:%4.3f %4.3f\n"), Gantry, MarkDelta[Gantry][MK_1].Delta.x, MarkDelta[Gantry][MK_1].Delta.y);
	}
	if (doneflag)
	{
		TRACE(_T("[PWR] Gantry:%d 1st NG Err:%d\n"), Gantry, ret);
		return ret;
	}
	if (Mark2No < 1)
	{
		TRACE(_T("[PWR] MarkPairRecognition Mark2No is under 1\n"), Mark2No);
		MarkDelta[Gantry][MK_2].Delta = MarkDelta[Gantry][MK_1].Delta;
		gLedAllOff(); // Off
		return NO_ERR;
	}
	Mark2No--;
	
	// clear init.
	if (correct)
	{
		MarkDelta[Gantry][MK_2].Delta.x = MarkDelta[Gantry][MK_1].Delta.x;
		MarkDelta[Gantry][MK_2].Delta.y = MarkDelta[Gantry][MK_1].Delta.y;
		Mark2Pos.x += MarkDelta[Gantry][MK_2].Delta.x;
		Mark2Pos.y += MarkDelta[Gantry][MK_2].Delta.y;
	}

	if (viserror > 0 || settleerror > 0 || repeat > 0)
	{ 	
		// go to the next position. (except for first time).
		Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, cam, Mark2, Ratio, InposXY, Ms, TimeOut);
		if (Err != NO_ERR)
		{
			return Err;
		}
	}
	repeat = 0;
	correct = 0;
	moredelay = 30;		// Settling Delay
	settleerror = 0;
	viserror = 0;
	doneflag = 0;
	TRACE(_T("[PWR] Mark2(%d) Led On(%03d,%03d,%03d)\n"), Mark2No, 
		Mark[Mark2No].LEDLevel[0], Mark[Mark2No].LEDLevel[1], Mark[Mark2No].LEDLevel[2]);
	gLedOn(cam, Mark[Mark2No].LEDLevel[0], Mark[Mark2No].LEDLevel[1], Mark[Mark2No].LEDLevel[2]);

	FOREVER
	{
		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] MarkPairRecognition-2 GetGlobalStatusError(%d)\n"), GetGlobalStatusError());
			Err = STOP_NOW;
			return Err;
		}
		if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] MarkPairRecognition-2 GetMachineState(%d)\n"), GetMachineState());
			Err = STOP_NOW;
			return Err;
		}
		res.x = res.y = 0.0;
		do
		{
			GetTime = _time_get();
			Err = WaitGantryIdle(FRONT_GANTRY, TIME1000MS);
			if (_time_elapsed(GetTime) > 0)
			{
				TRACE(_T("[PWR] MarkPairRecognition WaitGantryIdle-2 Elasped:%d[ms]"), _time_elapsed(GetTime));
			}
			Err = LinearIntplPosWaitDelayedPosSet(FRONT_GANTRY, cam, Mark2, Ratio, InposXY, Ms, TimeOut);
			if (Err != NO_ERR)
			{
				return Err;
			}
			prepareMarkRecognition(cam, Mark2No, 0);
			ThreadSleep(TIME20MS + moredelay);	// more settling.
			catchMarkImage(cam);
			Cur = gReadGantryPosition(FRONT_GANTRY);
			ThreadSleep(TIME10MS);
			ret = getMarkRecognition(cam, &res);
			if (gcPowerLog->IsShowVisionLog() == true)
			{
				TRACE(_T("[PWR] [Gantry=%d] 2nd Mark Capture ResXYRE,%.3f,%.3f,%.3f,%d\n"), Gantry, res.x, res.y, res.r, res.exe);
			}
			if (ret != 0) viserror++;
			else if ((fabs(Mark2Pos.x - Cur.x) >= SpecIn || fabs(Mark2Pos.y - Cur.y) >= SpecIn)) {		// 2006.1.23
				settleerror++;
				moredelay += 20;
				ret = PCB_FIDUCIALMARK_RECOGNITION;
				TRACE(_T("[PWR] [Gantry=%d] XY Settling is NOT Sufficient during Fiducial 2nd Mark Capture\n"), Gantry);
				TRACE(_T("[PWR] [ERR] TargetXY,%.3f,%.3f CurXY,%.3f,%.3f => DiffXY,%.3f,%.3f\n"), Mark2.x, Mark2.y, Cur.x, Cur.y, Mark2.x - Cur.x, Mark2.y - Cur.y);
			}
			else break;
			nextdelay = 0;
			gLedOn(cam, Mark[Mark2No].LEDLevel[0], Mark[Mark2No].LEDLevel[1], Mark[Mark2No].LEDLevel[2]);
			if (settleerror >= 10 || viserror >= 3)
			{
				doneflag = 1;
				break;
			}
		} while (doneflag == 0);

		if (doneflag)
		{
			Point_XY pre;
			ThreadSleep(TIME100MS);
			Cur = gReadGantryPosition(FRONT_GANTRY);
			pre.x = Cur.x;
			pre.y = Cur.y;
			TRACE(_T("[PWR] <Mark> Fiducial MARK 2 NOT FOUND ERROR !!! (%s)\n"), ((Gantry == FRONT_GANTRY) ? "Front" : "Rear"));
			break;
		}
		// Mark OK.
		else
			break;
		// Repeated Error -> Skip.
		//if (++repeat > 1 && correct == 1)
		//{
		//	break;
		//}
	}
	if (Err == STOP_NOW)
	{
		return Err;
	}
	// Final Mark 2 Result.
	MarkDelta[Gantry][MK_2].Delta.x += res.x;
	MarkDelta[Gantry][MK_2].Delta.y += res.y;
	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] Gantry:%d 2nd MarkDelta XY:%4.3f %4.3f\n"), Gantry, MarkDelta[Gantry][MK_2].Delta.x, MarkDelta[Gantry][MK_2].Delta.y);
	}
	if (doneflag)
	{
		TRACE(_T("[PWR] Tbl:%d 2nd NG Err:%d\n"), Gantry, ret);
		return ret;
	}
	gLedAllOff();
	return NO_ERR;
}

long CEthernetVision::GetMarkDelta(long Gantry, long Mark1No, long Mark2No, Point_XY Mark1Pos, Point_XY Mark2Pos, long Res)
{
	double angle;
	double ca, sa;
	double dx1, dx2, dy1, dy2;
	double x_[5], y_[5], oridist, mesdist;
	long onept = 0;

	x_[2] = Mark1Pos.x;
	y_[2] = Mark1Pos.y;	// Informed Fiducial Mark1 XY
	x_[3] = Mark2Pos.x;
	y_[3] = Mark2Pos.y;	// Informed Fiducial Mark2 XY

	x_[0] = x_[2] + MarkDelta[Gantry][Mark1No].Delta.x;
	y_[0] = y_[2] + MarkDelta[Gantry][Mark1No].Delta.y;	// Real Fiducial Mark1 XY
	x_[1] = x_[3] + MarkDelta[Gantry][Mark2No].Delta.x;
	y_[1] = y_[3] + MarkDelta[Gantry][Mark2No].Delta.y;	// Real Fiducial Mark2 XY

	if (gcPowerLog->IsShowVisionLog() == true)
	{
		TRACE(_T("[PWR] (Mark1) Gantry:%d P1 XY (%.3f, %.3f) Delta1 XY (%.3f, %.3f)\n"), 
			Gantry, x_[2], y_[2], MarkDelta[Gantry][Mark1No].Delta.x, MarkDelta[Gantry][Mark1No].Delta.y);
		TRACE(_T("[PWR] (Mark2) Gantry:%d P2 XY (%.3f, %.3f) Delta2 XY (%.3f, %.3f)\n"), 
			Gantry, x_[3], y_[3], MarkDelta[Gantry][Mark2No].Delta.x, MarkDelta[Gantry][Mark2No].Delta.y);
	}

	MarkDelta[Gantry][Res].Delta.x = x_[0];
	MarkDelta[Gantry][Res].Delta.y = y_[0];

	x_[2] -= x_[0]; y_[2] -= y_[0];		// Informed Mark1 - Real Mark1
	x_[3] -= x_[0]; y_[3] -= y_[0];		// Informed Mark2 - Real Mark1
	x_[1] -= x_[0]; y_[1] -= y_[0];		// Real Mark2 - Real Mark1
	x_[0] = 0.00;  y_[0] = 0.00;
	
	onept = 0;
	if (fabs(x_[3] - x_[2]) < 0.5 && fabs(y_[3] - y_[2]) < 0.5)
	{
		angle = 0.0;
		onept = 1;
	}
	else
	{
		angle = atan2((y_[1] - y_[0]), (x_[1] - x_[0])) - atan2((y_[3] - y_[2]), (x_[3] - x_[2]));
	}
	ca = cos(angle);
	sa = sin(angle);

	dx1 = ca * x_[2] - sa * y_[2];				/* rotate cad x1 position */
	dx2 = ca * x_[3] - sa * y_[3];				/* rotate cad x2 position */
	MarkDelta[Gantry][Res].Dx = x_[0] - dx1;	/* 보상할 x방향 값 */

	dx1 = (dx2 - dx1);					/* rotated cad x length */
	dx2 = (x_[1] - x_[0]);				/* vision chk result x length */
	if (onept || (dx1 >= -2.0 && dx1 <= 2.0))
		MarkDelta[Gantry][Res].ScaX = 1.0;
	else
	{
		MarkDelta[Gantry][Res].ScaX = dx2 / dx1;            /* x scale factor */
	}

	dy1 = sa * x_[2] + ca * y_[2];				/* rotate cad y1 position */
	dy2 = sa * x_[3] + ca * y_[3];				/* rotate cad y2 position */
	MarkDelta[Gantry][Res].Dy = y_[0] - dy1;	/* 보상할 y방향 값 */

	dy1 = (dy2 - dy1);					/* rotated cad y length */
	dy2 = (y_[1] - y_[0]);				/* vision chk result x length */
	if (onept || (dy1 >= -2.0 && dy1 <= 2.0))
		MarkDelta[Gantry][Res].ScaY = 1.0;
	else
	{
		MarkDelta[Gantry][Res].ScaY = dy2 / dy1;            /* y scale factor */
	}

	MarkDelta[Gantry][Res].Angle = angle;	/* radian */

	oridist = sqrt((Mark1Pos.x - Mark2Pos.x) * (Mark1Pos.x - Mark2Pos.x) + (Mark1Pos.y - Mark2Pos.y) * (Mark1Pos.y - Mark2Pos.y));
	mesdist = sqrt(dx2 * dx2 + dy2 * dy2);

	if (fabs(oridist - mesdist) > 0.500)
	{
		TRACE(_T("[PWR] <MARK ERROR> Difference is Too Big ->(CAD:%4.3f,Measured:%4.3f)=%4.3f\n"), oridist, mesdist, fabs(oridist - mesdist));
	}
	else
	{
		if (gcPowerLog->IsShowVisionLog() == true)
		{
			TRACE(_T("[PWR] <MARK> CAD:%4.3f,Measured:%4.3f=%4.3f\n"), oridist, mesdist, fabs(oridist - mesdist));
		}
	}
	return 0;
}

Point_XYR CEthernetVision::MarkCompensation(long Gantry, Point_XYR Pos, long Res)
{
	double angle;
	double ca, sa;
	double x1, y1;
	Point_XYR retPt;

	angle = MarkDelta[Gantry][Res].Angle;    /* radian */
	ca = cos(angle);
	sa = sin(angle);

	retPt.x = Pos.x - MarkDelta[Gantry][Res].Delta.x;
	retPt.y = Pos.y - MarkDelta[Gantry][Res].Delta.y;

	x1 = ca * retPt.x - sa * retPt.y;
	y1 = sa * retPt.x + ca * retPt.y;
	x1 += MarkDelta[Gantry][Res].Dx;
	y1 += MarkDelta[Gantry][Res].Dy;

	retPt.x = x1 * MarkDelta[Gantry][Res].ScaX + MarkDelta[Gantry][Res].Delta.x;
	retPt.y = y1 * MarkDelta[Gantry][Res].ScaY + MarkDelta[Gantry][Res].Delta.y;
	retPt.r = Pos.r + MarkDelta[Gantry][Res].Angle * A180_PIE;

	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		if (Res == MK_PWB)
		{
			TRACE(_T("[PWR] Mark Delta XY %.3f %.3f\n"), MarkDelta[Gantry][Res].Dx, MarkDelta[Gantry][Res].Dy);
			TRACE(_T("[PWR] Mark Compen-Mark,DeltaX[%.3f],DeltaY[%.3f],Angle[%.3f]\n"), MarkDelta[Gantry][Res].Delta.x, MarkDelta[Gantry][Res].Delta.y, MarkDelta[Gantry][Res].Angle);
		}
		else
		{
			TRACE(_T("[PWR] Mark Compen-Mark[%d],DeltaX[%.3f],DeltaY[%.3f],Angle[%.3f]\n"), Res, MarkDelta[Gantry][Res].Delta.x, MarkDelta[Gantry][Res].Delta.y, MarkDelta[Gantry][Res].Angle);
		}
		TRACE(_T("[PWR] Inserting Pos  (Mark Compen) = (%.3f,%.3f,%.3f)\n"), retPt.x, retPt.y, retPt.r);
	}
	return retPt;
}

Point_XYRZ CEthernetVision::ComponentCompensation(long Gantry, long ChkPos, Point_XYRZ Pos, long InsertOrder, long MarkRes)
{
	double ang = 0.0, dx = 0.0, dy = 0.0, dr = 0.0;
	double ca, sa;
	long VACheckPos;
	Point_XYRZ retPt;
	Point_XYRE Res;

	VACheckPos = ChkPos;
	Res = GetRunVisionResult(Gantry, InsertOrder);
	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		TRACE(_T("[PWR] CAD Pos XYRZ = %4.3f,%4.3f,%4.3f,%4.3f\n"), Pos.x, Pos.y, Pos.r, Pos.z);
	}
	ang = Pos.r + Res.r + GetRunVisionAngle(Gantry, InsertOrder);
	if (MarkRes == W_INIT)
		ang = ang * PIE_180;
	else
	{
		if (Gantry >= 0 && Gantry < MAXTABLENO && MarkRes >= 0 && MarkRes < MAXBDFIDNO + 40) {
			ang = ang * PIE_180 + MarkDelta[Gantry][MarkRes].Angle;
		}
		else {
			ang = ang * PIE_180;
			TRACE(_T("[PWR] MarkDelta index out of range Gantry=%d, MarkRes=%d\n"), Gantry, MarkRes);
		}
	}
	ca = cos(ang);
	sa = sin(ang);
	dx = ca * Res.x - sa * Res.y;
	dy = sa * Res.x + ca * Res.y;
	retPt.x = Pos.x + dx;
	retPt.y = Pos.y + dy;
	retPt.r = Pos.r + Res.r;
	retPt.z = Pos.z;
	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		TRACE(_T("[PWR] Insert Pos   (Before)      = %4.3f,%4.3f,%4.3f\n"), Pos.x, Pos.y, Pos.r);
		if (Gantry >= 0 && Gantry < MAXTABLENO && MarkRes >= 0 && MarkRes < MAXBDFIDNO + 40)
			TRACE(_T("[PWR] Part Compens (VA result)   = %4.3f,%4.3f,%4.3f MarkR=%.3f\n"), Res.x, Res.y, Res.r, MarkDelta[Gantry][MarkRes].Angle * A180_PIE);
		else
			TRACE(_T("[PWR] MarkDelta index out of range Gantry=%d, MarkRes=%d\n"), Gantry, MarkRes);
		TRACE(_T("[PWR] Part Compens (Delta)       = %4.3f,%4.3f,%4.3f\n"), dx, dy, ang);
		TRACE(_T("[PWR] Insert Pos   (After Comps) = %4.3f,%4.3f,%4.3f,%4.3f\n"), retPt.x, retPt.y, retPt.r, retPt.z);
	}
	return retPt;
}

Point_XYR CEthernetVision::MarkLoss(long Gantry, Point_XYR* Pos, long Res)
{
	double angle, ca, sa, x1, y1;
	Point_XYR pt;
	ZeroMemory(&pt, sizeof(pt));
	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		TRACE(_T("[PWR] Gantry%d Blk:%d Dxyr,%.3f,%.3f,%.3f\n"), Gantry, Res, MarkDelta[Gantry][Res].Dx, MarkDelta[Gantry][Res].Dy, MarkDelta[Gantry][Res].Angle);
		TRACE(_T("[PWR] Gantry%d Blk:%d DeltaXY,%.3f,%.3f\n"), Gantry, Res, MarkDelta[Gantry][Res].Delta.x, MarkDelta[Gantry][Res].Delta.y);
		TRACE(_T("[PWR] Gantry%d Blk:%d ScaleXY,%.3f,%.3f\n"), Gantry, Res, MarkDelta[Gantry][Res].ScaX, MarkDelta[Gantry][Res].ScaY);
	}
	angle = -MarkDelta[Gantry][Res].Angle;    /* radian */
	ca = cos(angle);
	sa = sin(angle);
	pt.x = Pos->x / MarkDelta[Gantry][Res].ScaX;
	pt.y = Pos->y / MarkDelta[Gantry][Res].ScaY;
	x1 = ca * (pt.x) - sa * (pt.y);
	y1 = sa * (pt.x) + ca * (pt.y);
	pt.x = x1;
	pt.y = y1;
	TRACE(_T("[PWR] CAD Pos Mark Loss,%.3f,%.3f\n"), pt.x, pt.y);
	return pt;
}

void CEthernetVision::SetRunVisionResult(long Gantry, long InsertOrder, Point_XYRE res)
{
	m_VAResult[Gantry][InsertOrder].x = res.x;
	m_VAResult[Gantry][InsertOrder].y = res.y;
	m_VAResult[Gantry][InsertOrder].r = res.r;
	m_VAResult[Gantry][InsertOrder].exe = res.exe;
}

Point_XYRE CEthernetVision::GetRunVisionResult(long Gantry, long InsertOrder)
{
	return m_VAResult[Gantry][InsertOrder];
}

void CEthernetVision::SetRunVisionErrorCode(long Gantry, long InsertOrder, long ErrorCode)
{
	m_VAErrorCode[Gantry][InsertOrder] = ErrorCode;
}

long CEthernetVision::GetRunVisionErrorCode(long Gantry, long InsertOrder)
{
	return m_VAErrorCode[Gantry][InsertOrder];
}

void CEthernetVision::InitRunVisionAngle(long Gantry, long InsertOrder)
{
	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		TRACE(_T("[PWR] InitRunVisionAngle Gantry:%d InsertOrd:%d\n"), Gantry, InsertOrder);
	}
	m_VAAngleInsert[Gantry][InsertOrder] = 0.000;
}

void CEthernetVision::SetRunVisionAngle(long Gantry, long InsertOrder, double Angle)
{
	if (gcPowerLog->IsShowCompensationLog() == true)
	{
		TRACE(_T("[PWR] SetRunVisionAngle Gantry:%d InsertOrd:%d Angle:%.3f\n"), Gantry, InsertOrder, Angle); 
	}
	m_VAAngleInsert[Gantry][InsertOrder] -= Angle;
}

double CEthernetVision::GetRunVisionAngle(long Gantry, long InsertOrder)
{
	if (gcPowerLog->IsShowCompensationLog() == true)
		TRACE(_T("[PWR] GetRunVisionAngle Gantry:%d InsertOrd:%d Angle:%.3f\n"), Gantry, InsertOrder, m_VAAngleInsert[Gantry][InsertOrder]);
	if (abs(m_VAAngleInsert[Gantry][InsertOrder]) > 360.0)
	{
		m_VAAngleInsert[Gantry][InsertOrder] = 0.0;
		if (gcPowerLog->IsShowCompensationLog() == true)
			TRACE(_T("[PWR] GetRunVisionAngle Force InsertOrd:%d Changed:%.3f\n"), InsertOrder, m_VAAngleInsert[Gantry][InsertOrder]);
	}
	return m_VAAngleInsert[Gantry][InsertOrder];
}

long CEthernetVision::GetMarkArea(long Gantry, long MarkNo)
{
	return Mark[MarkNo].Area;
}

void CEthernetVision::SetPartRecognitionResult(long CamNo, long CamChk, Point_XYRE Res)
{
	m_VAPartRecognitionResult[CamNo][CamChk] = Res;
}

Point_XYRE CEthernetVision::GetPartRecognitionResult(long CamNo, long CamChk)
{
	Point_XYRE Res;
	ZeroMemory(&Res, sizeof(Res));
	Res = m_VAPartRecognitionResult[CamNo][CamChk];
	return Res;
}

/*
*/
long CEthernetVision::InspectBarcode(long cam, long BarcodeType)
{
	long retVis = 999;
	long bdno, Err = 0, Size = 0;
	CString strMsg, strBarcode;
	char Barcode[BUFSIZE];
	ZeroMemory(&Barcode, sizeof(Barcode));
	bdno = GetVABoardNoFromGlobalCameraNo(cam);
	ClearVisionResult(bdno, 0);
	ClearVisionCommand(bdno);
	VIS_CMD[bdno][0]._long = VIS_BARCODE;
	VIS_CMD[bdno][1]._long = 0;
	VIS_CMD[bdno][2]._long = GetVACameraNoFromGlobalCameraNo(cam);
	VIS_CMD[bdno][3]._long = BarcodeType;
	VIS_CMD[bdno][4]._long = VIS_EOP;
	strMsg.Format(_T("%d,%d,%d,%d"), VIS_CMD[bdno][1]._long, VIS_CMD[bdno][2]._long, VIS_CMD[bdno][3]._long, VIS_CMD[bdno][4]._long);
	TriggerEvent(bdno, 5, VIS_BARCODE, strMsg);
	ThreadSleep(TIME2MS);
	visionResult(bdno, cam + 1, VIS_BARCODE);
	Size = GetVisionBarcodeSize(bdno);
	TRACE(_T("[PWR] InspectBarcode Cam%02d Size:%d\n"), cam, Size);
	if (Size > 0)
	{
		for (long index = 0; index < Size; ++index)
		{
			Barcode[index] = (char)GetVisionResultLong(bdno, 4 + index);
			strBarcode.AppendFormat(_T("%c"), Barcode[index]);
		}
		if (strBarcode.GetLength() > 0)
		{
			m_Barcode = strBarcode;
		}
	}
	else
	{
		m_Barcode.Format(_T("NULL_BARCODE"));
	}
	TRACE(_T("[PWR] Barcode:%s\n"), strBarcode);
	retVis = GetVisionCommandResult(bdno);
	Err = GetVisionErrorCode(bdno);
	return retVis;
}

CString CEthernetVision::GetBarcode()
{
	return m_Barcode;
}


double CEthernetVision::GetVisionPartPitchResult(unsigned bdno)
{
	double PitchResult = 0.0;
	PitchResult = GetVisionResultDouble(bdno, 9);
	return PitchResult;
}

void CEthernetVision::SetRunVisionPitchResult(long Gantry, long InsertOrder, Point_XYT SizeResult)
{
	m_VAResultPitch[Gantry][InsertOrder] = SizeResult;
	TRACE(_T("[PWR] SetRunVisionPitchResult Gantry(%d) InsertOrd(%d) SizeXY,%.3f,%.3f\n"), Gantry, InsertOrder, SizeResult.x, SizeResult.y);
}

Point_XYT CEthernetVision::GetRunVisionPitchResult(long Gantry, long InsertOrder)
{
	TRACE(_T("[PWR] GetRunVisionPitchResult Gantry(%d) InsertOrd(%d) SizeXY,%.3f,%.3f\n"), Gantry, InsertOrder, m_VAResultPitch[Gantry][InsertOrder].x, m_VAResultPitch[Gantry][InsertOrder].y);
	return m_VAResultPitch[Gantry][InsertOrder];
}