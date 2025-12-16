#include "pch.h"
#include "CMeasureHeight.h"
#include "AxisInformation.h"
#include "Vision.h"
#include "VisionData.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "CPowerConveyorControl.h"
#include "GlobalDefine.h"
#include "CSyncGantryConveyor.h"
//#include "ErrorCode.h"
#include "GlobalData.h"
#include "Trace.h"
#include "CStep.h"
#include "CApplicationTime.h"
#include "CReadJobFile.h"
#include "CTokenizer.h"
#include <fstream>
#include <iostream>
#include "CMark.h"

CMeasureHeight* gcMeasureHeight;
CMeasureHeight::CMeasureHeight()
{
	m_measureDelay = TIME100MS;
	m_measureRatio.xy = m_measureRatio.z = m_measureRatio.r = 1.0;
	m_StrArrResult = new CStringArray();
}

CMeasureHeight::~CMeasureHeight()
{
	delete m_StrArrResult;
}

void CMeasureHeight::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CMeasureHeight::GetFileName()
{
	return m_StrFileName;
}

void CMeasureHeight::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CMeasureHeight::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CMeasureHeight::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CMeasureHeight::AddResult(CString result)
{
	m_StrArrResult->Add(result);
}

void CMeasureHeight::RemoveResult()
{
	m_StrArrResult->RemoveAll();
}


bool CMeasureHeight::MakeFile()
{

	FILE* fp;
	char filename[BUFSIZE];
	ZeroMemory(filename, sizeof(filename));
	CStringA strConverter(m_StrFileName);
	memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
	filename[strConverter.GetLength()] = 0;

	fopen_s(&fp, filename, "w");
	if (fp == NULL)
	{
		TRACE(_T("[PWR] Make HeightMesurement open is null\n"));
		return false;
	}
	fclose(fp);
	return true;

}


bool CMeasureHeight::SaveFile(void)
{
//	std::ofstream outfile(GetFileName());
	std::ofstream outfile(GetFileName(), std::ios::trunc);

	for (long i = 0; i < m_StrArrResult->GetCount(); i++)
	{
		CStringA writeResult(m_StrArrResult->GetAt(i));
		for (int k = 0; k < writeResult.GetLength(); k++) 
		{			
			outfile << writeResult[k];
		}
		outfile << std::endl;
	}


	outfile.close();

	return TRUE;

	//FILE* fp;
	//char filename[BUFSIZE];
	//ZeroMemory(filename, sizeof(filename));
	//CStringA strConverter(GetFileName());
	//memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
	//filename[strConverter.GetLength()] = 0;
	//fopen_s(&fp, filename, "w");

	//if (fp == NULL)
	//{
	//	TRACE(_T("[PWR] SaveInsertEndFile open is null\n"));
	//	return false;
	//}
	//if (gcPowerLog->IsShowRunLog() == true)
	//{
	//	TRACE(_T("[PWR] Height Inspect Data to File(%S)...\n"), filename);
	//}

	//for (long i = 0; i < m_StrArrResult.GetCount(); i++)
	//{
	//	CString str = m_StrArrResult.GetAt(i);
	//	CStringA strA(str);

	//	fwrite(strA, sizeof(TCHAR), sizeof(strA.GetLength()), fp);
	//	fwrite("\n", sizeof(char), 1, fp);
	//}

	//fclose(fp);

	//if (gcPowerLog->IsShowRunLog() == true)
	//{
	//	TRACE(_T("Done.\n"));
	//}


	//return true;
}


ORIGIN CMeasureHeight::GetOrigin()
{
	ORIGIN Origin;
	TRACE(_T("[PWR] CMeasureHeight GetOriginXY-1\n"));
	Origin = gcReadJobFile->GetOrigin();
	TRACE(_T("[PWR] CMeasureHeight GetOriginXY-2:%.3f %.3f\n"), Origin.pt.x, Origin.pt.y);
	return Origin;
}

long CMeasureHeight::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
	return Err;
}


void CMeasureHeight::SetRatio(Ratio_XYRZ ratio)
{
	m_measureRatio = ratio;
}
void CMeasureHeight::SetDelay(long delay)
{
	m_measureDelay = delay;
}

long SendHeightMeasureEnd()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_04;
	nSubMsg[2] = HMI_CMD3RD_07;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendHeightMeasureEnd Done\n"));
	}
	return 0;
}

long CMeasureHeight::Run()
{
	long Gantry = FRONT_GANTRY;
	long target = TBL_HEIGHTDEVICE;
	MEASUREHEIGHT measure;
	ORIGIN Origin;
	Point_XY goalXY, originXY, cadXY;
	Point_XYR compenXYR, cadXYR;
	long Err;
	double InposXY = 0.10, ratio = 0.5;
	long MsXy = TIME30MS;
	long TimeOut = TIME5000MS;
	double height, tolernace, gap;
	CString writeStr;
	ULONGLONG GetTime = 0, Elapsed = 0;
	long allPass = TRUE;
	long measureReuslt;


	PCB Pcb = gcReadJobFile->GetPcb();

	HeightMeasurementControl(TRUE);

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] CMeasureHeight Run WaitGantryIdle Elasped:%d[ms]"), _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] CMeasureHeight Run WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}

	Origin = GetOrigin();
	originXY.x = Origin.pt.x;
	originXY.y = Origin.pt.y;

	RemoveResult();

	for (long order = 0; order < MAXINSERTNO; order++)
	{
		measure = gcReadJobFile->GetMeasureHeight(order);
		
		if (measure.Use == 0) continue;

		cadXY.x = measure.pt.x; 
		cadXY.y = measure.pt.y;

		cadXY = ReadInsertFromOrigin(cadXY, originXY);

		cadXYR.x = cadXY.x;
		cadXYR.y = cadXY.y;
		cadXYR.r = 0.0;
		
		if (Pcb.UseFiducial > 0)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, MK_PWB);
		}
		else
		{
			compenXYR = cadXYR;
		}	

		goalXY.x = compenXYR.x;
		goalXY.y = compenXYR.y;

		ratio = 1.0;

		Err = MoveXY(Gantry, target, goalXY, ratio, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Height Inspect Order:%d LinearIntplPosWaitDelayedPosSet Err:%d\n"), order, Err);
			break;
		}

		ThreadSleep(TIME100MS);

		height = GetHeight(Gantry);

	//	height = GetInsertByZ(Gantry) - GetHeight(Gantry);

		tolernace = fabs(measure.Tolerance);
		gap = fabs(measure.pt.t - height);		

		if (gap > abs(measure.Tolerance))
		{
			measureReuslt = 0;
			allPass = FALSE;
			Err = HEIGHT_MEASUREMENT_FAIL;
			TRACE(_T("[PWR] HM Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Fail\n"), order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));
		}
		else
		{
			measureReuslt = 1;
			TRACE(_T("[PWR] HM Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Pass\n"), order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));

		}

		writeStr.Format(_T("%d,%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d"), 
			order+1,
			(LPCTSTR)measure.LocateID,
			measure.Use,
			measure.pt.x, measure.pt.y, measure.pt.t, tolernace, height, gap,
			measureReuslt);

	//	TRACE(writeStr);

		AddResult(writeStr);
	}

	HeightMeasurementControl(FALSE);

	SetDir(_T("C:\\Power\\i6.0\\RUN"));
	SetHeader(_T("HM_Result"));
	SetExtension(_T("txt"));
	MakeFileName();
//	MakeFile();
	SaveFile();

	SendHeightMeasureEnd();

	if (allPass == FALSE)
	{
		Err = HEIGHT_MEASUREMENT_FAIL;
		SendAlarm(HEIGHT_MEASUREMENT_FAIL,_T("Height Measurement fail."));
	}

	return Err;
}




CString CMeasureHeight::ConfirmMeasureHeightPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	long Err = NO_ERR;
	Err = CheckReadyToMachine(true);
	if (Err != NO_ERR)
	{
		strRetMsg.Format(_T("%d,%d"), Err, HMI_RUN_CONTROL_STOPNOW);
		return strRetMsg;
	}

	if (GetRunMode() != NORMAL_MODE)
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		ULONGLONG GetTime = 0, Elapsed = 0;
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}
		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, order = 0, Ret = NO_ERR;
		Ratio_XYRZ Ratio;
		Point_XY ptGoal, Cur;
		Point_XYR ptXYR;
		Point_XYRZ pt;
		Point_XYT CurXYT;


		long Target = TBL_CAMERA, TimeOut = TIME5000MS, Ms = TIME30MS;

		//		Target = FHCAM; 

		ZeroMemory(&pt, sizeof(pt));
		ZeroMemory(&ptGoal, sizeof(ptGoal));
		ZeroMemory(&Ratio, sizeof(Ratio));
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.003;
		double PCBThickness = 0.0, RatioPusherZ = 0.5;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}

		Gantry = FRONT_GANTRY;
		order = iValue[0] - 1;
		Ratio.xy = Ratio.r = Ratio.z = 0.5;

		SetMachineState(STATE_RUN);
		SetGlobalStatusError(false);
		if (order >= 0)
		{
			if (gcReadJobFile)
			{
				PCBThickness = gcReadJobFile->GetPcb().Thickness;
				if (IsExistSet(WORK1_CONV) == true)
				{
					GetTime = _time_get();
					Err = StartPosWaitMotion(GetPusherZName(FRONT_GANTRY), RatioPusherZ, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
					if (Err == NO_ERR)
					{
						WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
						TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
					}
				}
				Err = MoveZStandy(Gantry, GetStandByZ(FRONT_GANTRY), Ratio.z);
				if (Err == NO_ERR)
				{
					PCB Pcb = gcReadJobFile->GetPcb();
					const ORIGIN Origin = gcReadJobFile->GetOrigin();
					MEASUREHEIGHT measureHeight = gcReadJobFile->GetMeasureHeight(order);

					if (measureHeight.Use == 0)
					{
						TRACE(_T("[PWR] No(%03d) Height Measurement No Use!!!!\n"), order + 1);
						SetMachineState(STATE_IDLE);
						strRetMsg.Format(_T("%d"), Err);
						return strRetMsg;
					}

					Point_XY OriginXY, InsertXY;
					OriginXY.x = Origin.pt.x;
					OriginXY.y = Origin.pt.y;
					InsertXY.x = measureHeight.pt.x;
					InsertXY.y = measureHeight.pt.y;
                    const long BlockNo = gcReadJobFile->GetInsert(measureHeight.GantryNo, measureHeight.InsertNo).BlockNo;
                    if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
                    {
                        const Point_XYR BlockOrigin = GetOriginXYRFromJobfile(BlockNo);
                        OriginXY.x = BlockOrigin.x;
                        OriginXY.y = BlockOrigin.y;
                    }
					if (Pcb.UseFiducial > 0)
					{
						FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
						Point_XY Mark1, Mark2;
                        if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                        {
                            Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                            Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], Point_XY{ Origin.pt.x, Origin.pt.y }, true);
                        }
                        else//Pcb.UseFiducial == FIDUCIAL_BLOCK
                        {
                            Mark1 = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
                            Mark2 = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);
                        }
						gSetMarkLed(FiducialMark.MarkNo[0], FiducialMark.Led[0]);
						gSetMarkLed(FiducialMark.MarkNo[1], FiducialMark.Led[1]);
						TRACE(_T("[PWR] MarkRecognition WaitZStandy Elapsed,%d\n"), _time_elapsed(GetTime));
						Ret = gMarkPairRecognition(FRONT_GANTRY, FiducialMark.MarkNo[0], FiducialMark.MarkNo[1], Mark1, Mark2, Ratio);
						if (Ret == NO_ERR)
						{
                            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                            {
                                Ret = gGetMarkDelta(FRONT_GANTRY, MK_1, MK_2, Mark1, Mark2, MK_PWB);
                            }
                            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                            {
                                Ret = gGetMarkDelta(FRONT_GANTRY, MK_1, MK_2, Mark1, Mark2, BlockNo);
                            }
							ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
							TRACE(_T("[PWR] No(%03d) Goal X,Y,%.3f,%.3f\n"), order, ptGoal.x, ptGoal.y);
							ptXYR.x = ptGoal.x;
							ptXYR.y = ptGoal.y;
							ptXYR.r = 0.000;
                            if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
                            {
                                ptXYR = gMarkCompensation(FRONT_GANTRY, ptXYR, MK_PWB);
                            }
                            else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                            {
                                ptXYR = gMarkCompensation(FRONT_GANTRY, ptXYR, BlockNo);
                            }
							ptGoal.x = ptXYR.x;
							ptGoal.y = ptXYR.y;
						}
						else
						{
							Ret = RECOGNITION_MARK_FAIL;
							Ret = SendAlarm(RECOGNITION_MARK_FAIL, _T("Mark Recognition Fail before Confirm measureHeight"));
						}
					}
					else
					{
						ptGoal = ReadInsertFromOrigin(InsertXY, OriginXY);
					}
					if (Ret == NO_ERR)
					{
						HeightMeasurementControl(TRUE);
						Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptGoal, Ratio.xy, Inpos, Ms, TimeOut);
						if (Err == NO_ERR)
						{
							Cur = gReadGantryPosition(Gantry);
							CurXYT.x = Cur.x;
							CurXYT.y = Cur.y;

							CurXYT.t = measureHeight.pt.t;

							WriteConfirmMeasureHeightBeforePosition(CurXYT);

							gLedOn(FHCAM, 10, 10, 0);
							gLiveOn(FHCAM);

							//	Cur = gReadGantryPosition(FRONT_GANTRY);

							TRACE(_T("[PWR] HMConfirm MoveXY (%03d) JobXYT,%.3f,%.3f MoveXY,%.3f,%.3f\n"),
								order + 1,
								measureHeight.pt.x, measureHeight.pt.y, measureHeight.pt.t,
								ptGoal.x, ptGoal.y);
						}
					}
				}
			}
			SetMachineState(STATE_IDLE);
			strRetMsg.Format(_T("%d"), Err);
		}
		else
		{
			strRetMsg.Format(_T("10000"));
		}


		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}




CString CMeasureHeight::TeachMeasureHeightPosition(CString strHostMsg)
{
	CString strRetMsg, strMsg = strHostMsg;
	
	if (GetRunMode() != NORMAL_MODE)
	{
		strRetMsg.Format(_T("%d"), ISNOT_NORMALMODE);
		return strRetMsg;
	}

	if (strMsg.GetLength() > 0)
	{
		CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CDecoding3 MovePcbConfig TokenCount:%d\n", cTokenizer->GetCount());
		}

		long Gantry = FRONT_GANTRY, iCnt = 0, dCnt = 0, Err = NO_ERR, order = 0, Ret = NO_ERR;
		CString strValue;
		int iValue[10];
		double dValue[10];
		double Inpos = 0.003;
		double ratio = 0.5;
		long Target = TBL_HEIGHTDEVICE, TimeOut = TIME5000MS, Ms = TIME30MS;

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}

		Gantry = FRONT_GANTRY;
		order = iValue[1] - 1;
		if (order >= 0)
		{
			if (gcReadJobFile)
			{
				MEASUREHEIGHT measureHeight = gcReadJobFile->GetMeasureHeight(order);
				Point_XY ptCurrentGantryXY;
				Point_XYR Diff, Loss;
				Point_XYT ptOld, ptJobFile, ptReuslt, ptNew;
				double currT;

				if (measureHeight.Use == 0)
				{
					TRACE(_T("[PWR] No(%03d) Height Measurement No Use!!!!\n"), order + 1);
					SetMachineState(STATE_IDLE);
					strRetMsg.Format(_T("%d"), Err);
					return strRetMsg;
				}


				ptJobFile = measureHeight.pt;

				ptCurrentGantryXY = gReadGantryPosition(FRONT_GANTRY);	// Origin + CAD + 
				ptOld = ReadConfirmMeasureHeightBeforePosition();


				Diff.x = ptCurrentGantryXY.x - ptOld.x;
				Diff.y = ptCurrentGantryXY.y - ptOld.y;
				HeightMeasurementControl(TRUE);
				Target = TBL_HEIGHTDEVICE;
				Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptCurrentGantryXY, ratio, Inpos, Ms, TimeOut);

				ThreadSleep(TIME200MS);

				currT = GetHeight(Gantry);

				ThreadSleep(TIME200MS);

				Target = TBL_CAMERA;
				Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, ptCurrentGantryXY, ratio, Inpos, Ms, TimeOut);

				TRACE(_T("[PWR] HMTeach No(%d) CurrXYT,%.3f,%.3f OldXYT,%.3f,%.3f DiffXYT,%.3f,%.3f\n"), 
					order + 1,
					ptCurrentGantryXY.x, ptCurrentGantryXY.y, 
					ptOld.x, ptOld.y, ptOld.t, 
					Diff.x, Diff.y, currT - ptOld.t);


				const PCB Pcb = gcReadJobFile->GetPcb();
                const long BlockNo = gcReadJobFile->GetInsert(measureHeight.GantryNo ,measureHeight.InsertNo).BlockNo;
				if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
				{
					Loss = gMarkLoss(FRONT_GANTRY, &Diff, MK_PWB);
				}
                else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
                {
                    Loss = gMarkLoss(Gantry, &Diff, BlockNo);
                }
				else
				{
					Loss = Diff;
				}


				TRACE(_T("[PWR] HMTeach No(%d) LossXYT,%.3f,%.3f,%.3f\n"), order + 1, Loss.x, Loss.y, currT - ptOld.t);


				if (abs(Loss.x) > 0.003 || abs(Loss.y) > 0.003 || abs(currT - ptOld.t) > 0.010)
				{
                    ptNew.x = ptCurrentGantryXY.x + (Diff.x - Loss.x);
                    ptNew.y = ptCurrentGantryXY.y + (Diff.y - Loss.y);
                    ptNew.t = currT;

                    WriteConfirmMeasureHeightBeforePosition(ptNew);

                    TRACE(_T("[PWR] HMTeach (%d) OldJobXYT,%.3f,%.3f,%.3f NewJobXYT,%.3f,%.3f,%.3f \n"), 
                    order + 1,
                    ptJobFile.x, ptJobFile.y, measureHeight.pt.t, 
                    ptNew.x, ptNew.y, ptNew.t);

                    if (EPSILON < std::abs(gcReadJobFile->GetBlockOrigin(BlockNo).pt.r))
                    {
                        const Point_XY transformedLoss = transformCoordinateFromEquipmentToBlockBased(Point_XY{ Loss.x, Loss.y }, Point_XY{ 0, 0 }, gcReadJobFile->GetBlockOrigin(BlockNo).pt.r);
                        Loss.x = transformedLoss.x;
                        Loss.y = transformedLoss.y;
                    }

					ptReuslt.x = ptJobFile.x + Loss.x;
					ptReuslt.y = ptJobFile.y + Loss.y;
					ptReuslt.t = currT;
					strRetMsg.Format(_T("%.3f,%.3f,%.3f"), ptReuslt.x, ptReuslt.y, ptReuslt.t);
				}
			}
		}
		else
		{
			strRetMsg.Format(_T("10000"));
		}


		delete cTokenizer;
		cTokenizer = NULL;
	}
	else
	{
		strRetMsg.Format(_T("20000"));
	}
	return strRetMsg;
}

long CMeasureHeight::Run(long ManualMode)
{
	long Gantry = FRONT_GANTRY;
	long target = TBL_HEIGHTDEVICE;
	MEASUREHEIGHT measure;
	ORIGIN Origin;
	Point_XY goalXY, originXY, cadXY;
	Point_XYR compenXYR, cadXYR;
	long Err;
	double InposXY = 0.10, ratio = 0.5;
	long MsXy = TIME30MS;
	long TimeOut = TIME5000MS;
	double height, tolernace, gap;
	CString writeStr;
	ULONGLONG GetTime = 0, Elapsed = 0;
	long allPass = TRUE;
	long measureReuslt;
	long BlockNo = 1;
	long FeederNo = 0;
	PCB Pcb = gcReadJobFile->GetPcb();
	//Point_XYR OriginXYR;

	bool GroupUse = false;
	bool GroupAllPass = true;

	std::map<long, long> mapCountMeasure;
	std::map<long, long> mapCountPass;
	std::map<long, long> mapCountNG;

	std::map<long, long>::iterator iterCountMeasure;
	std::map<long, long>::iterator iterCountPass;
	std::map<long, long>::iterator iterCountNG;
	long Conveyor = FRONT_CONV;
	double Inpos = 0.003;
	double PCBThickness = 0.0, RatioPusherZ = 0.5;
	Ratio_XYRZ Ratio;

	Ratio.xy = Ratio.r = Ratio.z = 1.0;
	PCBThickness = gcReadJobFile->GetPcb().Thickness;


	HeightMeasurementControl(TRUE);

	GetTime = _time_get();
	Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
	TRACE(_T("[PWR] CMeasureHeight (ManualMode:%d) Run WaitGantryIdle Elasped:%d[ms]"), ManualMode, _time_elapsed(GetTime));
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] CMeasureHeight Run WaitGantryIdle Err:%d\n"), Err);
		return Err;
	}

	TRACE(_T("[PWR] CMeasureHeight (ManualMode:%d)-1 Run WaitGantryIdle Elasped:%d[ms]"), ManualMode, _time_elapsed(GetTime));

	Origin = GetOrigin();
	originXY.x = Origin.pt.x;
	originXY.y = Origin.pt.y;

	TRACE(_T("[PWR] CMeasureHeight (ManualMode:%d)-2 Run WaitGantryIdle Elasped:%d[ms]"), ManualMode, _time_elapsed(GetTime));

	RemoveResult();

	TRACE(_T("[PWR] CMeasureHeight (ManualMode:%d)-3 Run WaitGantryIdle Elasped:%d[ms]"), ManualMode, _time_elapsed(GetTime));

	if (ManualMode == 1)
	{
		TRACE(_T("[PWR] CMeasureHeight Run ManualMode\n"));
		if (IsExistSet(WORK1_CONV) == true)
		{
			GetTime = _time_get();
			Err = StartPosWaitMotion(GetPusherZName(FRONT_GANTRY), RatioPusherZ, TimeOut, GetPusherByZ(FRONT_GANTRY) + PCBThickness, true);
			if (Err == NO_ERR)
			{
				WritePusherZ(FRONT_CONV, WORK1_CONV, GetPusherByZ(FRONT_GANTRY) + PCBThickness);
			}
			TRACE(_T("[PWR] PusherZ Up Elapsed:%d[ms]\n"), _time_elapsed(GetTime));
		}
		else
		{
			SendAlarm(PCB_EXIST_CANNOT_CONV, _T("PCB Not Exist"));
			return PCB_EXIST_CANNOT_CONV;
		}

		Err = MoveZAllUpLimit(Gantry, Ratio.z);
		if (Err != NO_ERR)
		{
			SendAlarm(Err, _T("MoveZAllUpLimit error"));
			return Err;
		}

		Err = MoveZAllUpLimitWait(Gantry, TIME5000MS);
		if (Err != NO_ERR)
		{
			SendAlarm(Err, _T("MoveZAllUpLimit error"));
			return Err;
		}

		if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
		{
			FIDUCIAL Fiducial = gcReadJobFile->GetMark();
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] Fiducial Mark1XY,%.3f,%.3f Mark2XY,%.3f,%.3f\n"), Fiducial.pt[0].x, Fiducial.pt[0].y, Fiducial.pt[1].x, Fiducial.pt[1].y);
			}

			Point_XY OriginXY;
			OriginXY.x = Origin.pt.x;
			OriginXY.y = Origin.pt.y;

			Point_XY Mark1, Mark2;
			Mark1 = ReadMarkFromOrigin(Fiducial.pt[0], OriginXY, true);
			Mark2 = ReadMarkFromOrigin(Fiducial.pt[1], OriginXY, true);
			gSetMarkLed(Fiducial.MarkNo[0], Fiducial.Led[0]);
			gSetMarkLed(Fiducial.MarkNo[1], Fiducial.Led[1]);

			long Ret = gMarkPairRecognition(FRONT_GANTRY, Fiducial.MarkNo[0], Fiducial.MarkNo[1], Mark1, Mark2, Ratio);
			if (Ret != NO_ERR)
			{
				SendAlarm(Err, _T("Mark recognition error"));
				return Ret;
			}
			if (gcPowerLog->IsShowElapsedLog() == true)
			{
				TRACE(_T("[PWR] gMarkPairRecognition Elapsed,%d\n"), _time_elapsed(GetTime));
			}
			Ret = gGetMarkDelta(FRONT_GANTRY, MK_1, MK_2, Mark1, Mark2, MK_PWB);
			if (Ret != NO_ERR)
			{
				SendAlarm(Err, _T("Mark Delta error"));
				return Ret;
			}
		}
        else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
        {
            //SINGLE 에서는 CPowerMain에서 생성하고 소멸시킴. (생산중에만 사용)
            CMark cMarkInstance = CMark{ FRONT_GANTRY };

            for (int i = 1; i <= Pcb.MaxBlockCount; i++)//1 2 .. 블록개수.
            {
                if (gcReadJobFile->GetBlockOrigin(i).Use == 0)
                {
                    TRACE_FILE_FUNC_LINE_"skipped due to blockOrigin NO_USE.");
                    continue;
                }
                CString temp; temp.Format(L"block fiducial mark checking.. blockNo: %d", i); TRACE_FILE_FUNC_LINE_(CStringA)temp);
                cMarkInstance.blockFiducialMarkChecking(i);//[Block Origin]의 [Use]가 [0]이 아닌 것만 사용할거니깐 검사도 안함.
            }
        }
	}
	else
	{
		TRACE(_T("[PWR] CMeasureHeight Run ProdMode Conv:%d\n"), Conveyor);
	}

	for (long order = 0; order < MAXINSERTNO; order++)
	{
		measure = gcReadJobFile->GetMeasureHeight(order);

		if (measure.Use == 0)
		{
			//TRACE(_T("[PWR] CMeasureHeight order:%d measure.Method:%d\n"), order, measure.Method);
			continue;
		}
		if (measure.Method == 0 || measure.Method == 1)
		{
			TRACE(_T("[PWR] CMeasureHeight order:%d\n"), order);
		}
		else
		{
			//TRACE(_T("[PWR] CMeasureHeight order:%d measure.Method:%d\n"), order, measure.Method);
			continue;
		}

		//BlockNo = GetBlockNoFromInsert(measure.InsertNo);
		//OriginXYR = GetOriginXYRFromJobfile(BlockNo);
		//originXY.x = OriginXYR.x;
		//originXY.y = OriginXYR.y;

        //원점 좌표 찾기 (블록 타입에 따라)
        if (Pcb.UseBlockType == PCB_MAXTRIX || Pcb.UseBlockType == PCB_NON_MAXTRIX)
        {
            const INSERT insertInfo = gcReadJobFile->GetInsert(measure.GantryNo, measure.InsertNo);
            BlockNo = insertInfo.BlockNo;
            const Point_XYR blockOrigin = GetOriginXYRFromJobfile(BlockNo);
            originXY = Point_XY{ blockOrigin.x, blockOrigin.y };

            if (gcReadJobFile->GetBlockOrigin(BlockNo).Use == 0)//안쓰는 블록일 경우 검사 안함. (ReTest든 생산이든)
            {
                CString temp; temp.Format(L"skip due to BlockNo(%d) NO_USE", BlockNo);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                continue;
            }
        }

		cadXY.x = measure.pt.x;
		cadXY.y = measure.pt.y;

		cadXY = ReadInsertFromOrigin(cadXY, originXY);

		cadXYR.x = cadXY.x;
		cadXYR.y = cadXY.y;
		cadXYR.r = 0.0;

		if (ManualMode == 1)
		{
			FeederNo = gcReadJobFile->GetInsert(measure.InsertNo).FeederNo;
			if (gcReadJobFile->GetPick(FeederNo).Use == 0)
			{
				continue;
			}

			if (gcReadJobFile->GetInsert(measure.InsertNo).Use == 0)
			{
				continue;
			}
		}
		else
		{
			if (gcStep->GetUseFromInsertNo(measure.InsertNo) == 0)
			{
				continue;
			}

			FeederNo = gcStep->GetFeederNoFromInsertNo(measure.InsertNo);
			if (gcStep->GetFeederUse(FeederNo) == 0)
			{
				continue;
			}

			//if (Pcb.UseBlockType != PCB_SINGLE)
			//{
			//	if (gcStep[Gantry]->GetBlockUse(BlockNo) == 0 || gcStep[Gantry]->GetBlockSkipHM(BlockNo) == BLOCK_SKIP)
			//	{
			//		continue;
			//	}
			//}
		}

		if (measure.GroupNo > 0)
		{
			if (mapCountPass.find(measure.GroupNo) != mapCountPass.end())
			{
				TRACE(_T("[PWR] CMeasureHeight skip already pass Order:%d Group:%d\n"), order, measure.GroupNo);

				continue;
			}

			iterCountMeasure = mapCountMeasure.find(measure.GroupNo);
			if (iterCountMeasure == mapCountMeasure.end())
			{
				mapCountMeasure[measure.GroupNo] = 1;
			}
			else
			{
				mapCountMeasure[measure.GroupNo] = iterCountMeasure->second + 1;
			}
		}

		if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, MK_PWB);
		}
		else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, BlockNo);
		}
		else
		{
			compenXYR = cadXYR;
		}

		goalXY.x = compenXYR.x;
		goalXY.y = compenXYR.y;

		ratio = 1.0;

		Err = MoveXY(Gantry, target, goalXY, ratio, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] Height Inspect Order:%d LinearIntplPosWaitDelayedPosSet Err:%d\n"), order, Err);
			return Err;
		}

		for (long tryCnt = 0; tryCnt < 10; tryCnt++)
		{
			ThreadSleep(TIME10MS);
			height = GetHeight(Gantry);

			tolernace = fabs(measure.Tolerance);
			gap = fabs(measure.pt.t - height);

			if (gap > abs(measure.Tolerance))
			{

			}
			else
			{
				break;
			}
		}

		//ThreadSleep(TIME100MS);

		//height = GetHeight(Gantry);

		//tolernace = fabs(measure.Tolerance);
		//gap = fabs(measure.pt.t - height);		

		if (measure.GroupNo > 0)
		{
			iterCountMeasure = mapCountMeasure.find(measure.GroupNo);
			if (iterCountMeasure == mapCountMeasure.end())
			{
				mapCountMeasure[measure.GroupNo] = 1;
			}
			else
			{
				mapCountMeasure[measure.GroupNo] = iterCountMeasure->second + 1;
			}
		}

		if (gap > abs(measure.Tolerance))
		{
			if (measure.GroupNo > 0)
			{
				iterCountNG = mapCountNG.find(measure.GroupNo);

				if (iterCountNG == mapCountNG.end())
				{
					mapCountNG[measure.GroupNo] = 1;
				}
				else
				{
					mapCountNG[measure.GroupNo] = iterCountNG->second + 1;
				}
			}
			else
			{
				allPass = FALSE;
			}

			measureReuslt = 0;
			//Err = HEIGHT_MEASUREMENT_FAIL;
			TRACE(_T("[PWR] HM Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Fail\n"), order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));
		}
		else
		{
			if (measure.GroupNo > 0)
			{
				iterCountPass = mapCountPass.find(measure.GroupNo);

				if (iterCountPass == mapCountPass.end())
				{
					mapCountPass[measure.GroupNo] = 1;
				}
				else
				{
					mapCountPass[measure.GroupNo] = iterCountPass->second + 1;
				}
			}

			measureReuslt = 1;
			TRACE(_T("[PWR] HM Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Pass\n"), order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));
		}

		writeStr.Format(_T("%d,%s,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d"),
			order + 1,
			(LPCTSTR)measure.LocateID,
			measure.Use,
			measure.pt.x, measure.pt.y, measure.pt.t, tolernace, height, gap,
			measureReuslt, measure.GroupNo);

		//	TRACE(writeStr);

		AddResult(writeStr);
	}

	for (auto iter = mapCountMeasure.begin(); iter != mapCountMeasure.end(); iter++)
	{
		if (mapCountPass.find(iter->first) == mapCountPass.end())
		{
			GroupAllPass = false;
			TRACE(_T("[PWR] HM Group:%d All NG\n"), iter->first);
			break;
		}
	}

	for (auto iter = mapCountMeasure.begin(); iter != mapCountMeasure.end(); iter++)
	{
		TRACE(_T("[PWR] HM Count Measure Group:%d Cnt:%d\n"), iter->first, iter->second);
	}

	for (auto iter = mapCountPass.begin(); iter != mapCountPass.end(); iter++)
	{
		TRACE(_T("[PWR] HM Count Pass Group:%d Cnt:%d\n"), iter->first, iter->second);
	}

	for (auto iter = mapCountNG.begin(); iter != mapCountNG.end(); iter++)
	{
		TRACE(_T("[PWR] HM Count NG Group:%d Cnt:%d\n"), iter->first, iter->second);
	}

	HeightMeasurementControl(FALSE);

	SetDir(_T("C:\\Power\\i6.0\\RUN"));
	SetHeader(_T("HM_Result"));
	SetExtension(_T("txt"));
	MakeFileName();
	//	MakeFile();
	SaveFile();

	//SendHeightMeasureEnd();
	if (ManualMode == 1)
	{
		SendManualHeightMeasureEnd();
	}
	else
	{
		if (allPass == FALSE || GroupAllPass == false)
		{
			Err = HEIGHT_MEASUREMENT_FAIL;
			SendAlarm(HEIGHT_MEASUREMENT_FAIL, _T("Height Measurement fail."));
		}
	}

	return Err;
}

long CMeasureHeight::SendHeightMeasureEnd()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_04;
	nSubMsg[2] = HMI_CMD3RD_07;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendHeightMeasureEnd Done\n"));
	}
	return 0;
}

long CMeasureHeight::BlockSkipCheck()
{
	long Err = NO_ERR;
	long Gantry = FRONT_GANTRY;
	long target = TBL_HEIGHTDEVICE;
	Point_XY goalXY;
	double ratio = 1.0;
	double InposXY = 0.10;
	long MsXy = TIME30MS;
	long TimeOut = TIME5000MS;
	bool FirstMeasure = true;
	double height;
	long ExistCnt = 0;

	PCB Pcb = gcReadJobFile->GetPcb();
	BLOCKSKIP_HM BlockSkip;
	Point_XYR OriginXYR;
	Point_XY OriginXY;
	Point_XY OffsetXY;
	double StandbyZ = gcReadJobFile->GetStandyBy().pt.z;

	gcStep->InitBlockSkipHM();

	if (Pcb.UseBlockType == PCB_SINGLE)
	{
		return Err;
	}


	MoveZStandy(Gantry, StandbyZ, ratio);
	for (long BlockNo = 1; BlockNo <= Pcb.MaxBlockCount; BlockNo++)
	{
		BlockSkip = gcReadJobFile->GetBlockSkipHM(BlockNo);

		if (gcStep->GetBlockUse(BlockNo) == 0)
		{
			continue;
		}
		else if (BlockSkip.Use == 0)
		{
			ExistCnt++;
			continue;
		}

		OriginXYR = GetOriginXYRFromJobfile(BlockNo);
		OriginXY.x = OriginXYR.x;
		OriginXY.y = OriginXYR.y;

		OffsetXY.x = BlockSkip.Pt.x;
		OffsetXY.y = BlockSkip.Pt.y;

		goalXY = ReadInsertFromOrigin(OffsetXY, OriginXY);

		if (FirstMeasure == true)
		{
			FirstMeasure = false;
			HeightMeasurementControl(TRUE);
			Err = WaitGantryIdle(Gantry, TIME5000MS);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] BlockSkipCheck WaitGantryIdle Err:%d\n"), Err);
				return Err;
			}
		}

		Err = MoveXY(Gantry, target, goalXY, ratio, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] BlockSkipCheck Block:%d XY %.3f,%.3f Err:%d\n"), BlockNo, goalXY.x, goalXY.y, Err);
			return Err;
		}

		for (long retry = 0; retry < 100; retry++)
		{
			ThreadSleep(TIME10MS);
			height = GetHeight(Gantry);

			if (fabs(BlockSkip.Tolerence) < fabs(BlockSkip.Height - height))
			{
				gcStep->SetBlockSkipHM(BlockNo, BLOCK_SKIP);
				TRACE(_T("[PWR] SetBlockSkipHM Block:%d Ref:%.3f Measure:%.3f\n"), BlockNo, BlockSkip.Height, height);
				continue;
			}
			else
			{
				gcStep->SetBlockSkipHM(BlockNo, BLOCK_PROD);
				ExistCnt++;
				break;
			}
		}


		/*		if (fabs(BlockSkip.Tolerence) < fabs(BlockSkip.Height - height))
				{
					gcStep->SetBlockSkipHM(BlockNo, BLOCK_SKIP);
					TRACE(_T("[PWR] SetBlockSkipHM Block:%d Ref:%.3f Measure:%.3f\n"), BlockNo, BlockSkip.Height, height);
				}
				else
				{
					ExistCnt++;
				}	*/
	}


	if (ExistCnt == 0)
	{
		TRACE(_T("[PWR] SetBlockSkipHM All Block Skip(%d)\n"), ExistCnt);
		gcStep->SetAllBlockSkipHM(true);
	}
	else
	{
		TRACE(_T("[PWR] SetBlockSkipHM Block Exist(%d)\n"), ExistCnt);
		gcStep->SetAllBlockSkipHM(false);
	}

	HeightMeasurementControl(false);

	return Err;
}

long CMeasureHeight::MovePrepare()
{
	long Err = NO_ERR;
	long Gantry = FRONT_GANTRY;
	long target = TBL_HEIGHTDEVICE;
	Point_XY goalXY;
	double ratio = 1.0;
	double InposXY = 0.10;
	long MsXy = TIME30MS;
	long TimeOut = TIME5000MS;
	bool FirstMeasure = true;
	long ExistCnt = 0;

	PCB Pcb = gcReadJobFile->GetPcb();
	BLOCKSKIP_HM BlockSkip;
	Point_XYR OriginXYR;
	Point_XY OriginXY;
	Point_XY OffsetXY;
	double StandbyZ = gcReadJobFile->GetStandyBy().pt.z;

	gcStep->InitBlockSkipHM();

	if (Pcb.UseBlockType == PCB_SINGLE)
	{
		return Err;
	}

	TRACE(_T("[PWR] BlockSkipCheck MovePrepare\n"));


	MoveZStandy(Gantry, StandbyZ, ratio);
	for (long BlockNo = 1; BlockNo <= Pcb.MaxBlockCount; BlockNo++)
	{
		BlockSkip = gcReadJobFile->GetBlockSkipHM(BlockNo);

		if (gcStep->GetBlockUse(BlockNo) == 0)
		{
			continue;
		}
		else if (BlockSkip.Use == 0)
		{
			ExistCnt++;
			continue;
		}

		OriginXYR = GetOriginXYRFromJobfile(BlockNo);
		OriginXY.x = OriginXYR.x;
		OriginXY.y = OriginXYR.y;

		OffsetXY.x = BlockSkip.Pt.x;
		OffsetXY.y = BlockSkip.Pt.y;

		goalXY = ReadInsertFromOrigin(OffsetXY, OriginXY);

		if (FirstMeasure == true)
		{
			FirstMeasure = false;
			HeightMeasurementControl(TRUE);
			Err = WaitGantryIdle(Gantry, TIME5000MS);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] BlockSkipCheck WaitGantryIdle Err:%d\n"), Err);
				return Err;
			}
		}

		TRACE(_T("[PWR] BlockSkipCheck MovePrepare Block:%d XY %.3f,%.3f \n"), BlockNo, goalXY.x, goalXY.y, Err);
		Err = MoveXY(Gantry, target, goalXY, ratio, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] BlockSkipCheck Block:%d XY %.3f,%.3f Err:%d\n"), BlockNo, goalXY.x, goalXY.y, Err);
			return Err;
		}

		break;
	}

	return Err;
}

long CMeasureHeight::SendHeightMeasureBeforeInsert(long Order, MEASUREHEIGHT measure, double realdata, double Gap, long measureReuslt)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_14;

	strSendMsg.Format(_T("%d,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d"), Order, (LPCTSTR)measure.LocateID, measure.pt.x, measure.pt.y, measure.pt.t, fabs(measure.Tolerance), realdata, Gap, measureReuslt);

	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendHeightMeasureBeforeInsert Done\n"));
	}
	return 0;
}

long CMeasureHeight::HeightMeasureFromInsertNo(long InsertNo, double RatioXY, long BlockNo)
{
	typedef struct _SAVE_RESULT
	{
		long Order;
		MEASUREHEIGHT MeasureInfo;
		double realHeight;
		double gap;
		long result;
	} SAVE_RESULT;

	long Gantry = FRONT_GANTRY;
	long target = TBL_HEIGHTDEVICE;
	MEASUREHEIGHT measure;
	//ORIGIN Origin;
	Point_XY goalXY, originXY, cadXY;
	Point_XYR compenXYR, cadXYR;
	long Err = NO_ERR;
	double InposXY = 0.10, ratio = 0.5;
	long MsXy = TIME30MS;
	long TimeOut = TIME5000MS;
	double height, tolernace, gap;
	CString writeStr;
	ULONGLONG GetTime = 0, Elapsed = 0;
	long allPass = TRUE;
	long measureReuslt;
	bool ExcuteMeasure = false;
	//long MeasureTotal = gcReadJobFile->GetMeasureHeightTotal();
	PCB Pcb = gcReadJobFile->GetPcb();
	bool groupUse = false;
	bool groupGood = false;

	//Origin = GetOrigin();
	//originXY.x = Origin.pt.x;
	//originXY.y = Origin.pt.y;
	Point_XYR OriginXYR = GetOriginXYRFromJobfile(BlockNo);
	originXY.x = OriginXYR.x;
	originXY.y = OriginXYR.y;

	std::map<long, long> saveGroupList;
	std::map<long, SAVE_RESULT> saveResultOK;
	std::map<long, SAVE_RESULT> saveResultNG;
	SAVE_RESULT temp;

	for (long order = 1; order < MAXINSERTNO; order++)
	{
		measure = gcReadJobFile->GetMeasureHeight(order - 1);

		if (measure.Use == 0) continue;
		if (measure.Method != 2) continue;
		if (measure.InsertNo != InsertNo) continue;

		if (ExcuteMeasure == false)
		{
			ExcuteMeasure = true;
			HeightMeasurementControl(TRUE);
			GetTime = _time_get();
			Err = WaitGantryIdle(FRONT_GANTRY, TIME5000MS);
			if (Err != NO_ERR)
			{
				TRACE(_T("[PWR] HeightMeasureFromInsertNo WaitGantryIdle Err:%d\n"), Err);
				return Err;
			}
		}

		if (measure.GroupNo > 0)
		{
			if (saveResultOK.find(measure.GroupNo) != saveResultOK.end())
			{
				TRACE(_T("[PWR] HeightMeasureFromInsertNo skip already pass Order:%d Group:%d\n"), order, measure.GroupNo);

				continue;
			}
		}


		cadXY.x = measure.pt.x;
		cadXY.y = measure.pt.y;

		cadXY = ReadInsertFromOrigin(cadXY, originXY);

		cadXYR.x = cadXY.x;
		cadXYR.y = cadXY.y;
		cadXYR.r = 0.0;

		if (Pcb.UseFiducial == FIDUCIAL_SINGLE)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, MK_PWB);
		}
		else if (Pcb.UseFiducial == FIDUCIAL_BLOCK)
		{
			compenXYR = gMarkCompensation(Gantry, cadXYR, BlockNo);
		}
		else
		{
			compenXYR = cadXYR;
		}

		goalXY.x = compenXYR.x;
		goalXY.y = compenXYR.y;

		Err = MoveXY(Gantry, target, goalXY, RatioXY, InposXY, MsXy, TimeOut);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] HeightMeasureFromInsertNo InsertNo:%d Order:%d LinearIntplPosWaitDelayedPosSet Err:%d\n"), InsertNo, order, Err);
			return Err;
		}

		for (long tryCnt = 0; tryCnt < 10; tryCnt++)
		{
			ThreadSleep(TIME10MS);
			height = GetHeight(Gantry);

			tolernace = fabs(measure.Tolerance);
			gap = fabs(measure.pt.t - height);

			if (gap > abs(measure.Tolerance))
			{

			}
			else
			{
				break;
			}
		}

		//tolernace = fabs(measure.Tolerance);
		//gap = fabs(measure.pt.t - height);

		if (gap > abs(measure.Tolerance))
		{
			measureReuslt = 0;
			Err = HEIGHT_MEASUREMENT_FAIL;
			TRACE(_T("[PWR] HM Insert:%d Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Fail\n"), InsertNo, order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));

			if (measure.GroupNo == 0)
			{
				SendHeightMeasureBeforeInsert(order, measure, height, gap, measureReuslt);
				return Err;
			}
			else
			{
				temp.Order = order;
				temp.MeasureInfo = measure;
				temp.realHeight = height;
				temp.gap = gap;
				temp.result = measureReuslt;

				saveResultNG[measure.GroupNo] = temp;
				saveGroupList[measure.GroupNo] = measure.GroupNo;
			}
		}
		else
		{
			measureReuslt = 1;
			TRACE(_T("[PWR] HM Insert:%d Order:%d Loc:%s Ref:%.3f Measure:%.3f Diff:%.3f Tol:%.3f - Pass\n"), InsertNo, order, measure.LocateID, measure.pt.t, height, gap, abs(measure.Tolerance));

			if (measure.GroupNo == 0)
			{
				SendHeightMeasureBeforeInsert(order, measure, height, gap, measureReuslt);
			}
			else
			{
				temp.Order = order;
				temp.MeasureInfo = measure;
				temp.realHeight = height;
				temp.gap = gap;
				temp.result = measureReuslt;

				saveResultOK[measure.GroupNo] = temp;
				saveGroupList[measure.GroupNo] = measure.GroupNo;
			}

		}
	}

	TRACE(_T("[PWR] HM saveGroupList:%d saveResultOK:%d saveResultNG:%d"), saveGroupList.size(), saveResultOK.size(), saveResultNG.size());

	for (auto iter = saveGroupList.begin(); iter != saveGroupList.end(); iter++)
	{
		long grpNo = iter->first;

		if (saveResultOK.find(grpNo) != saveResultOK.end())
		{
			temp = saveResultOK.at(grpNo);

			SendHeightMeasureBeforeInsert(temp.Order, temp.MeasureInfo, temp.realHeight, temp.gap, temp.result);
		}
		else if (saveResultNG.find(grpNo) != saveResultNG.end())
		{
			temp = saveResultNG.at(grpNo);
			SendHeightMeasureBeforeInsert(temp.Order, temp.MeasureInfo, temp.realHeight, temp.gap, temp.result);

			Err = HEIGHT_MEASUREMENT_FAIL;
			return Err;

		}
	}

	HeightMeasurementControl(false);

	return Err;
}

