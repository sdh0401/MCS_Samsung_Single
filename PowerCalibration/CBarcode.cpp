#include "pch.h"
#include "CBarcode.h"
#include "CBarcodeFile.h"
//#include "AxisInformation.h"
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
#include "CAdvancedMotionFile.h"
#include "CBarcodeControl.h"

CBarcode* gcBarcode;
CBarcode::CBarcode(long Gantry)
{
    m_Gantry = Gantry;
}

CBarcode::~CBarcode()
{
}

ORIGIN CBarcode::GetOrigin()
{
    ORIGIN Origin;
    Origin = gcStep->GetOrigin();
    return Origin;
}

Point_XY CBarcode::GetBarcodePosition(long BarcodeNo)
{
    Point_XY pt, Goal, OriginXY;
    pt.x = 0.000;
    pt.y = 0.000;
    ORIGIN Origin = GetOrigin();
    OriginXY.x = Origin.pt.x;
    OriginXY.y = Origin.pt.y;
    Goal = ReadMarkFromOrigin(pt, OriginXY);
    return Goal;
}

long CBarcode::BarcodeChecking()
{
    long Ret = NO_ERR, Err = NO_ERR;
    PCB Pcb = gcReadJobFile->GetPcb();
    ORIGIN Origin = gcStep->GetOrigin();
    CString strBarcodeCarrier, strBarcodeBlock1, strBarcodeBlock2;
    long Cam = FHCAM;
    if (m_Gantry == REAR_GANTRY)
        Cam = RHCAM;
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] OriginXY,%.3f,%.3f\n"), Origin.pt.x, Origin.pt.y);
    }
    BARCODE Barcode = gcStep->GetBarcode();
    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] Barcode XY,%.3f,%.3f\n"), Barcode.pt[0].x, Barcode.pt[0].y);
    }

    Point_XY OriginXY;
    Point_XY OriginXYCarrier;
    OriginXYCarrier.x = OriginXYCarrier.y = 0.000;
    //BlockNoCarrier = MK_PWB;
    //BlockNoBlock1 = MK_PWB;
    //BlockNoBlock2 = MK_PWB;
    Point_XYR ptXYR;
    double InposXY = 0.010;
    long MsXy = TIME30MS, TimeOut = TIME5000MS;
    if (gcAdvancedMotionFile->UseAdvanceMotion() == 1)
    {
        MsXy = gcAdvancedMotionFile->GetInsertDelayedMsXY();
        InposXY = gcAdvancedMotionFile->GetInsertInposXY();
        TRACE(_T("[PWR] Insert Ms XY,%d Inpos XY,%.3f\n"), MsXy, InposXY);
    }
    OriginXY.x = Origin.pt.x + OriginXYCarrier.x;
    OriginXY.y = Origin.pt.y + OriginXYCarrier.y;
    Point_XY BarocdeXY;
    BarocdeXY = ReadMarkFromOrigin(Barcode.pt[0], OriginXY);
    ptXYR.x = BarocdeXY.x;
    ptXYR.y = BarocdeXY.y;
    ptXYR.r = 0.0;
    //ptXYR = gMarkCompensation(m_Gantry, ptXYR, BlockNoCarrier);
    BarocdeXY.x = ptXYR.x;
    BarocdeXY.y = ptXYR.y;
    Ratio_XYRZ MoveRatio = GetMinRatio();

    TRACE(_T("[PWR] BarcodeChecking XY %.3f %.3f MES.Use:%d\n"), BarocdeXY.x, BarocdeXY.y, Barcode.Mes.Use);
    gLedOn(Cam, 0, 0, 0);
    Ret = MoveXY(m_Gantry, Cam, BarocdeXY, MoveRatio.xy, InposXY, MsXy, TimeOut);
    if (Ret == NO_ERR)
    {
        if (GetBarcodeSimulation() == true)
        {
            strBarcodeCarrier.Format(_T("BARCODE_SAMPLE"));
            SetCarrier(strBarcodeCarrier);
            Ret = NO_ERR;
        }
        else
        {
            gLedOn(Cam, Barcode.Led.Red, Barcode.Led.Blue, 0);
            ULONGLONG GetTime = _time_get(), Elapsed = 0;
            Ret = gInspectBarcode(Cam, Barcode.Type); // 20210415 HarkDo
            if (gcPowerLog->IsShowElapsedLog() == true)
            {
                TRACE(_T("[PWR] gInspectBarcode Elapsed,%d\n"), _time_elapsed(GetTime));
            }
            if (Ret == 0)
            {
                strBarcodeCarrier = gGetCameraBarcode(FRONT_VISION);
                SetCarrier(strBarcodeCarrier);
            }
            else
            {
                SendAlarm(BARCODE_READ_FAIL(0), _T("Carrier Barcode Read fail"));
                return BARCODE_READ_FAIL(0);
            }
        }
    }

    gLedOn(Cam, 0, 0, 0);
    return Ret;
}

long CBarcode::MoveXY(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
    long Err = NO_ERR;
    Err = LinearIntplPosWaitDelayedPosSet(Gantry, Target, pt, Ratio, Inpos, Ms, TimeOut);
    return Err;
}

Ratio_XYRZ CBarcode::GetMinRatio()
{
    Ratio_XYRZ MinRatio;
    MinRatio.xy = MinRatio.r = MinRatio.z = 1.0;
    MinRatio = gcStep->GetMinRatio();
    TRACE(_T("[PWR] CBarcode MinRatioXYRZ,%.1f,%.1f,%.1f"), MinRatio.xy, MinRatio.r, MinRatio.z);
    return MinRatio;
}

long CBarcode::SetCarrier(CString str)
{
    CString Barcode;
    Barcode.Format(_T("%s"), (LPCTSTR)(str));
    gcStep->SetBarcodeCarrier(Barcode);
    return str.GetLength();
}

long CBarcode::SetBlock1(CString str)
{
    CString Barcode;
    Barcode.Format(_T("%s"), (LPCTSTR)(str));
    gcStep->SetBarcodeBlock1(Barcode);
    return str.GetLength();
}

long CBarcode::SetBlock2(CString str)
{
    CString Barcode;
    Barcode.Format(_T("%s"), (LPCTSTR)(str));
    gcStep->SetBarcodeBlock2(Barcode);
    return str.GetLength();
}

