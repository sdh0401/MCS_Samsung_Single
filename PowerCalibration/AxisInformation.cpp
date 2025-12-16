#pragma once
#include "pch.h"
#include "AxisInformation.h"
#include "DefineMotionLoopTime.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "GlobalIODefine.h"
#include "GlobalDefine.h"
#include "Trace.h"
//#include "ErrorCode.h"
#include "CMachineInit.h"
#include "CPowerGantry.h"
#include "Cwmx3Motor.h"
#include "CSlaveMotorStatus.h"
#include "CMasterMotion.h"
#include "Cwmx3IO.h"
#include "CPowerIO.h"
#include "CioStatus.h"
#include "CAnalogStatus.h"
#include "CPowerConveyorData.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorControl.h"
#include "CPowerConveyorManualControl.h"
#include "CPowerFeederControl.h"
#include "CSyncGantryConveyor.h"
#include "CPowerMainControl.h"
#include "CPowerMain.h"
#include "CPcbExist.h"
#include "CPowerLog.h"
#include "CInsertEndFile.h"
#include "CPowerHMI.h"
#include "VisionData.h"
#include "CLedControl.h"
#include "CPowerTeachBox.h"
#include "CSmemaControl.h"
#include "CFeeder.h"
#include "CMachineFile.h"
#include "CRunFile.h"
#include "CPowerVision.h"
#include "EthernetVision.h"
#include "CMotorPower.h"
#include "CMachineInformation.h"
#include "CPowerSwitchPanel.h"
#include "CPowerCleaner.h"
#include "CTowerLamp.h"
#include "CDecoding.h"
#include "CPowerBuzzer.h"
#include <iostream>
#include "CAutoNozzleChange.h"
#include "CReadJobFile.h"
#include "CCamDropCheck.h"
#include "CCollisionMonitoring.h"
#include "CMachineConfig.h"
#include "CBarcodeControl.h"
#include "CBarcodeFile.h"
#include "CTrayDumpBox.h"
#include "Io_state_detector.h"

Point_XY ReadBlockTarget(const Point_XY& targetPoint, const Point_XY& originPoint)
{
    const long useBlockType = gcReadJobFile->GetPcb().UseBlockType;
    const Point_XY PcbFix = ReadPcbFixPosition(FRONT_CONV);

    if (useBlockType != PCB_MAXTRIX && useBlockType != PCB_NON_MAXTRIX)
    {
        TRACE_FILE_FUNC_LINE_"BlockType is neither PCB_MATRIX nor PCB_NON_MATRIX");
        return Point_XY{ PcbFix.x + originPoint.x + targetPoint.x, PcbFix.y + originPoint.y + targetPoint.y };
    }

    ORIGIN blockOrigin = ORIGIN{};
    Point_XYR blockOriginXYR = Point_XYR{};
    bool originFound = false;
    for (int i = 1; i <= gcReadJobFile->GetPcb().MaxBlockCount; i++)
    {
        const ORIGIN _blockOrigin = gcReadJobFile->GetBlockOrigin(i);//숫자 그대로 리턴.
        const Point_XYR _blockOriginXYR = GetOriginXYRFromJobfile(i);//PCB Origin + Block Origin 계산해서 리턴.
        if (originPoint.x == _blockOriginXYR.x && originPoint.y == _blockOriginXYR.y)
        {
            blockOrigin = _blockOrigin;
            blockOriginXYR = _blockOriginXYR;
            originFound = true;
            break;
        }
        continue;
    }
    if (originFound == false)
    {
        TRACE_FILE_FUNC_LINE_"origin not found!! (human error?)");
        return Point_XY{ PcbFix.x + originPoint.x + targetPoint.x, PcbFix.y + originPoint.y + targetPoint.y };
    }

    if (EPSILON < std::abs(blockOrigin.pt.r))
    {
        return transformCoordinateFromBlockBasedToEquipment(targetPoint, Point_XY{ PcbFix.x + blockOriginXYR.x, PcbFix.y + blockOriginXYR.y }, blockOrigin.pt.r);
    }
    else
    {
        //TRACE_FILE_FUNC_LINE_"blockOrigin.pt.r == 0.0");
        return Point_XY{ PcbFix.x + originPoint.x + targetPoint.x, PcbFix.y + originPoint.y + targetPoint.y };
    }
}

constexpr double convertDegreeToRadian(const double& degree)
{
    return degree * M_PI / 180.0;
}

constexpr double convertRadianToDegree(const double& radian)
{
    return radian * 180.0 / M_PI;
}

Point_XY transformCoordinate(const Point_XY& source, const double& rotateAngleInDegree, const Point_XY& origin)
{
    const double rotateAngleInRadian = convertDegreeToRadian(rotateAngleInDegree);

    //1. 회전 -> 2차원 회전행렬 적용. (반시계방향)
    const Point_XY rotateResult = {
        source.x * cos(rotateAngleInRadian) + source.y * sin(rotateAngleInRadian) * (-1)  //x
        ,
        source.x * sin(rotateAngleInRadian) + source.y * cos(rotateAngleInRadian)         //y
    };

    //2. 평행이동
    const Point_XY parallelTranslationResult = {
        rotateResult.x + origin.x
        ,
        rotateResult.y + origin.y
    };

    //TRACE("[PWR] [BLOCK] global::%s -> source : %.3f %.3f, origin : %.3f %.3f, rotateAngleInDegree : %.3f(%.3f radian), result : %.3f %.3f"
    //      , __func__, source.x, source.y, origin.x, origin.y, rotateAngleInDegree, rotateAngleInRadian, parallelTranslationResult.x, parallelTranslationResult.y);

    CString temp;
    temp.Format(L"source : %.3f %.3f, origin : %.3f %.3f, rotateAngleInDegree : %.3f(%.3f radian), result : %.3f %.3f"
                , source.x, source.y, origin.x, origin.y, rotateAngleInDegree, rotateAngleInRadian, parallelTranslationResult.x, parallelTranslationResult.y);
    TRACE_FILE_FUNC_LINE_(CStringA)temp);

    return parallelTranslationResult;
}

Point_XY transformCoordinateFromBlockBasedToEquipment(const Point_XY& source, const Point_XY& origin, const double& rotateAngleInDegree)
{
    return transformCoordinate(source, rotateAngleInDegree, origin);
}

Point_XY transformCoordinateFromEquipmentToBlockBased(const Point_XY& source, const Point_XY& origin, const double& rotateAngleInDegree)
{
    //시계방향으로 돌려야 하니까 각도에 -1 곱해서 넘긴다.
    return transformCoordinate(source, -rotateAngleInDegree, origin);
}

void getDegree(const CString& strMsg)
{
    static Point_XY referencePoint = { 0.0, 0.0 };
    CString temp;

    const Point_XY currentGantryPosition = gReadGantryPosition(FRONT_GANTRY);
    temp.Format(L"currentGantryPosition : %.3f %.3f", currentGantryPosition.x, currentGantryPosition.y);
    TRACE_FILE_FUNC_LINE_(CStringA)temp);

    if (strMsg.Compare(L"0") == 0)
    {
        referencePoint = currentGantryPosition;
        temp.Format(L"referencePoint set to %.3f %.3f", referencePoint.x, referencePoint.y);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
    }
    else if (strMsg.Compare(L"1") == 0)
    {
        temp.Format(L"referencePoint : %.3f %.3f", referencePoint.x, referencePoint.y);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
        const Point_XY positionOff = { currentGantryPosition.x - referencePoint.x, currentGantryPosition.y - referencePoint.y };
        temp.Format(L"positionOff : %.3f %.3f", positionOff.x, positionOff.y);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
        /*const*/ double angleInRadian = atan(positionOff.y / positionOff.x);
        if (positionOff.x < -0.0009 || positionOff.y < -0.0009)
        {
            if (positionOff.x < -0.0009 && 0.0 <= positionOff.y)
            {
                TRACE_FILE_FUNC_LINE_"plus 180");
                angleInRadian += M_PI;
            }
            else if (positionOff.y < -0.0009 && 0.0 <= positionOff.x)
            {
                TRACE_FILE_FUNC_LINE_"plus 360");
                angleInRadian += M_PI * 2;
            }
            else if (positionOff.x < -0.0009 && positionOff.y < -0.0009)
            {
                TRACE_FILE_FUNC_LINE_"plus 180");
                angleInRadian += M_PI;
            }
        }
        temp.Format(L"angleInRadian : %.3f", angleInRadian);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
        const double angleInDegree = convertRadianToDegree(angleInRadian);
        temp.Format(L"angleInDegree : %.3f", angleInDegree);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
    }
    else
    {
        TRACE_FILE_FUNC_LINE_"strMsg is neither 0, 1 nor 2.");
    }
    return;
}

double calculateBlockRotationOffset(const double& initialRotation, const long& blockNo)
{
    const Point_XYR originXYR = GetOriginXYRFromJobfile(blockNo);
    if (EPSILON < std::abs(originXYR.r))//origin R값이 0이 아닐때만 계산.
    {
        const double result = initialRotation + originXYR.r;
        CString temp; temp.Format(L"%.3f -> %.3f", initialRotation, result); TRACE_FILE_FUNC_LINE_(CStringA)temp);
        return result;
    }
    else//0이면 계산안하고 그대로 리턴.
    {
        return initialRotation;
    }
}

CString getFileNameFromFullPath(const std::string& fileName)
{
    const CString fullPath(fileName.c_str());

    constexpr int MAX_CYCLE = 10;
    int currentCycle = 0;

    int iStart = 0;
    CString lastTokenizeResult = fullPath;
    while (true)
    {
        if (currentCycle++ == MAX_CYCLE)
        {
            break;
        }

        const CString tokenizeResult = fullPath.Tokenize(L"\\", iStart);

        if (tokenizeResult.IsEmpty() == true)
        {
            break;
        }
        else
        {
            lastTokenizeResult = tokenizeResult;
            continue;
        }
        continue;
    }

    lastTokenizeResult = lastTokenizeResult.Left(lastTokenizeResult.GetLength() - 4);//항상 .cpp 에서 로그 출력하기 때문에 뒤 4글자(.cpp)는 삭제할게요.

    return lastTokenizeResult;
}

unsigned PowerAxisArray[MAXAXISNO] =
{
	static_cast<unsigned>(PowerAxis::FX),			// 0
	static_cast<unsigned>(PowerAxis::FY1),			// 1
	static_cast<unsigned>(PowerAxis::FY2),			// 2
	static_cast<unsigned>(PowerAxis::FZ1),			// 3
	static_cast<unsigned>(PowerAxis::FZ2),			// 4
	static_cast<unsigned>(PowerAxis::FZ3),			// 5
	static_cast<unsigned>(PowerAxis::FZ4),			// 6
	static_cast<unsigned>(PowerAxis::FZ5),			// 7
	static_cast<unsigned>(PowerAxis::FZ6),			// 8
	static_cast<unsigned>(PowerAxis::FW1),			// 9
	static_cast<unsigned>(PowerAxis::FW2),			// 10
	static_cast<unsigned>(PowerAxis::FW3),			// 11
	static_cast<unsigned>(PowerAxis::FW4),			// 12
	static_cast<unsigned>(PowerAxis::FW5),			// 13
	static_cast<unsigned>(PowerAxis::FW6),			// 14
	static_cast<unsigned>(PowerAxis::FCONV),		// 15
	static_cast<unsigned>(PowerAxis::FPUZ),			// 16
	static_cast<unsigned>(PowerAxis::FBTIN),		// 17
	static_cast<unsigned>(PowerAxis::FBTWK),		// 18
	static_cast<unsigned>(PowerAxis::FBTOT),		// 19
	static_cast<unsigned>(PowerAxis::RX),			// 20
	static_cast<unsigned>(PowerAxis::RY1),			// 21
	static_cast<unsigned>(PowerAxis::RY2),			// 22
	static_cast<unsigned>(PowerAxis::RZ1),			// 23
	static_cast<unsigned>(PowerAxis::RZ2),			// 24
	static_cast<unsigned>(PowerAxis::RZ3),			// 25
	static_cast<unsigned>(PowerAxis::RZ4),			// 26
	static_cast<unsigned>(PowerAxis::RZ5),			// 27
	static_cast<unsigned>(PowerAxis::RZ6),			// 28
	static_cast<unsigned>(PowerAxis::RW1),			// 29
	static_cast<unsigned>(PowerAxis::RW2),			// 30
	static_cast<unsigned>(PowerAxis::RW3),			// 31
	static_cast<unsigned>(PowerAxis::RW4),			// 32
	static_cast<unsigned>(PowerAxis::RW5),			// 33
	static_cast<unsigned>(PowerAxis::RW6),			// 34
	static_cast<unsigned>(PowerAxis::RCONV),		// 35
	static_cast<unsigned>(PowerAxis::RPUZ),			// 36
	static_cast<unsigned>(PowerAxis::RBTIN),		// 37
	static_cast<unsigned>(PowerAxis::RBTWK),		// 38
	static_cast<unsigned>(PowerAxis::RBTOT),		// 39
	static_cast<unsigned>(PowerAxis::TTF1_Y),		// 40
	static_cast<unsigned>(PowerAxis::TTF1_Z),		// 41
	static_cast<unsigned>(PowerAxis::TTF2_Y),		// 42
	static_cast<unsigned>(PowerAxis::TTF2_Z),		// 43
	static_cast<unsigned>(PowerAxis::CLI_X),		// 44
	static_cast<unsigned>(PowerAxis::CLI_Y),		// 45
	static_cast<unsigned>(PowerAxis::CLI_Z),		// 46
	static_cast<unsigned>(PowerAxis::CLI_R),		// 47
	static_cast<unsigned>(PowerAxis::CLI_C),		// 48
	static_cast<unsigned>(PowerAxis::STF_Y),		// 49
	static_cast<unsigned>(PowerAxis::STF_UPPER_Z),	// 50
	static_cast<unsigned>(PowerAxis::STF_LOWER_Z),	// 51
	static_cast<unsigned>(PowerAxis::ROLLCONV),		// 52
	static_cast<unsigned>(PowerAxis::ROLLBT),		// 53
	static_cast<unsigned>(PowerAxis::RESERVED1),	// 54
	static_cast<unsigned>(PowerAxis::RESERVED2),	// 55
	static_cast<unsigned>(PowerAxis::RESERVED3),	// 56
	static_cast<unsigned>(PowerAxis::RESERVED4),	// 57
	static_cast<unsigned>(PowerAxis::RESERVED5),	// 58
	static_cast<unsigned>(PowerAxis::RESERVED6)		// 59
};

unsigned PowerAxisMap[MAXAXISNO] =
{
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_00),//(PowerAxis::FX),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_01),//(PowerAxis::FY1),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_02),//(PowerAxis::FY2),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_03),//(PowerAxis::FZ1),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_05),//(PowerAxis::FZ2),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_07),//(PowerAxis::FZ3),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_09),//(PowerAxis::FZ4),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_11),//(PowerAxis::FZ5),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_13),//(PowerAxis::FZ6),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_04),//(PowerAxis::FW1),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_06),//(PowerAxis::FW2),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_08),//(PowerAxis::FW3),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_10),//(PowerAxis::FW4),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_12),//(PowerAxis::FW5),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_14),//(PowerAxis::FW6),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_19),//(PowerAxis::FCONV),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_18),//(PowerAxis::FPUZ),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_15),//(PowerAxis::FBTIN),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_16),//(PowerAxis::FBTWK),
	static_cast<unsigned>(PowerAxisMap::SLAVE_ID_17),//(PowerAxis::FBTOT),
	NON,//(PowerAxis::RX),
	NON,//(PowerAxis::RY1),
	NON,//(PowerAxis::RY2),
	NON,//(PowerAxis::RZ1),
	NON,//(PowerAxis::RZ2),
	NON,//(PowerAxis::RZ3),
	NON,//(PowerAxis::RZ4),
	NON,//(PowerAxis::RZ5),
	NON,//(PowerAxis::RZ6),
	NON,//(PowerAxis::RW1),
	NON,//(PowerAxis::RW2),
	NON,//(PowerAxis::RW3),
	NON,//(PowerAxis::RW4),
	NON,//(PowerAxis::RW5),
	NON,//(PowerAxis::RW6),
	NON,//(PowerAxis::RCONV),
	NON,//(PowerAxis::RPUZ),
	NON,//(PowerAxis::RBTIN),
	NON,//(PowerAxis::RBTWK),
	NON,//(PowerAxis::RBTOT),
	NON,//(PowerAxis::TTF1_Y),
	NON,//(PowerAxis::TTF1_Z),
	NON,//(PowerAxis::TTF2_Y),
	NON,//(PowerAxis::TTF2_Z),
	NON,//(PowerAxis::CLI_X),
	NON,//(PowerAxis::CLI_Y),
	NON,//(PowerAxis::CLI_Z),
	NON,//(PowerAxis::CLI_R),
	NON,//(PowerAxis::CLI_C),
	NON,//(PowerAxis::STF_Y),
	NON,//(PowerAxis::STF_UPPER_Z),
	NON,//(PowerAxis::STF_LOWER_Z),
	NON,//(PowerAxis::ROLLCONV),
	NON,//(PowerAxis::ROLLBT),
	NON,//(PowerAxis::RESERVED1),
	NON,//(PowerAxis::RESERVED2),
	NON,//(PowerAxis::RESERVED3),
	NON,//(PowerAxis::RESERVED4),
	NON,//(PowerAxis::RESERVED5),
	NON,//(PowerAxis::RESERVED6)
};

unsigned PowerAxisSlaveID[MAXAXISNO] =
{
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_01),//(PowerAxis::FX),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_02),//(PowerAxis::FY1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_03),//(PowerAxis::FY2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_04),//(PowerAxis::FZ1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_05),//(PowerAxis::FZ2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_06),//(PowerAxis::FZ3),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_07),//(PowerAxis::FZ4),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_08),//(PowerAxis::FZ5),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_09),//(PowerAxis::FZ6),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_10),//(PowerAxis::FW1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_11),//(PowerAxis::FW2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_12),//(PowerAxis::FW3),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_13),//(PowerAxis::FW4),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_14),//(PowerAxis::FW5),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_15),//(PowerAxis::FW6),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_16),//(PowerAxis::FCONV),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_17),//(PowerAxis::FPUZ),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_18),//(PowerAxis::FBTIN),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_19),//(PowerAxis::FBTWK),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_20),//(PowerAxis::FBTOT),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_21),//(PowerAxis::RX),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_22),//(PowerAxis::RY1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_23),//(PowerAxis::RY2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_24),//(PowerAxis::RZ1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_25),//(PowerAxis::RZ2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_26),//(PowerAxis::RZ3),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_27),//(PowerAxis::RZ4),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_28),//(PowerAxis::RZ5),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_29),//(PowerAxis::RZ6),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_30),//(PowerAxis::RW1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_31),//(PowerAxis::RW2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_32),//(PowerAxis::RW3),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_33),//(PowerAxis::RW4),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_34),//(PowerAxis::RW5),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_35),//(PowerAxis::RW6),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_36),//(PowerAxis::RCONV),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_37),//(PowerAxis::RPUZ),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_38),//static_cast<unsigned>(PowerAxisMap::SLAVE_ID_18),//(PowerAxis::RBTIN),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_39),//static_cast<unsigned>(PowerAxisMap::SLAVE_ID_19),//(PowerAxis::RBTWK),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_40),//static_cast<unsigned>(PowerAxisMap::SLAVE_ID_20),//(PowerAxis::RBTOT),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_41),//(PowerAxis::TTF1_Y),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_42),//(PowerAxis::TTF1_Z),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_43),//(PowerAxis::TTF2_Y),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_44),//(PowerAxis::TTF2_Z),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_45),//(PowerAxis::CLI_X),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_46),//(PowerAxis::CLI_Y),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_47),//(PowerAxis::CLI_Z),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_48),//(PowerAxis::CLI_R),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_49),//(PowerAxis::CLI_C),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_50),//(PowerAxis::STF_Y),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_51),//(PowerAxis::STF_UPPER_Z),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_52),//(PowerAxis::STF_LOWER_Z),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_53),//(PowerAxis::ROLLCONV),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_54),//(PowerAxis::ROLLBT),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_55),//(PowerAxis::RESERVED1),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_56),//(PowerAxis::RESERVED2),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_57),//(PowerAxis::RESERVED3),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_58),//(PowerAxis::RESERVED4),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_59),//(PowerAxis::RESERVED5),
	static_cast<unsigned>(PowerAxisSlaveNo::SLAVE_NO_60),//(PowerAxis::RESERVED6)
};

void SetUseTTFAxis()
{
	long Axis;

	Axis = (long)PowerAxis::TTF1_Y;
	PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_20);

	Axis = (long)PowerAxis::TTF1_Z;
	PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_21);

	Axis = (long)PowerAxis::TTF2_Y;
	PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_22);

	Axis = (long)PowerAxis::TTF2_Z;
	PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_23);


	TRACE(_T("[PWR] TTF Axis Use!!!!!!!!!!!!!!!!!!"));

}

void SetClearConveyorAxis()
{
	long Axis;

	Axis = (long)PowerAxis::FCONV;
	PowerAxisMap[Axis] = NON;

	TRACE(_T("[PWR] Front Conveyor Axis Clear!!!!!!!!!!!!!!!!!!"));
}

void SetAxisMapByOption()
{
	long Axis;

	if (GetManualConvUse() == true)
	{
		Axis = (long)PowerAxis::FCONV;
		PowerAxisMap[Axis] = NON;

		TRACE(_T("[PWR] Front Conveyor Axis Clear!!!!!!!!!!!!!!!!!!"));

		if (GetTTFUse() == true)
		{
			Axis = (long)PowerAxis::TTF1_Y;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_20);

			Axis = (long)PowerAxis::TTF1_Z;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_21);

			Axis = (long)PowerAxis::TTF2_Y;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_22);

			Axis = (long)PowerAxis::TTF2_Z;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_23);

			TRACE(_T("[PWR] TTF Axis Use (20~23)!!!!!!!!!!!!!!!!!!"));
		}
	}
	else
	{
		if (GetTTFUse() == true)
		{
			Axis = (long)PowerAxis::TTF1_Y;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_20);

			Axis = (long)PowerAxis::TTF1_Z;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_21);

			Axis = (long)PowerAxis::TTF2_Y;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_22);

			Axis = (long)PowerAxis::TTF2_Z;
			PowerAxisMap[Axis] = static_cast<unsigned>(PowerAxisMap::SLAVE_ID_23);

			TRACE(_T("[PWR] TTF Axis Use (20~23)!!!!!!!!!!!!!!!!!!"));
		}
	}
}

signed PowerAxisMovingDir[MAXAXISNO] =
{
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FX),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FY1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FY2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ3),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ4),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ5),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FZ6),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW3),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW4),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW5),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FW6),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FCONV),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FPUZ),

	//static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::FBTIN),
	//static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FBTWK),
	//static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::FBTOT),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FBTIN),
	static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::FBTWK),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::FBTOT),

	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RX),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RY1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RY2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ3),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ4),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ5),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RZ6),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW3),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW4),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW5),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RW6),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RCONV),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RPUZ),
	static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::RBTIN),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RBTWK),
	static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::RBTOT),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::TTF1_Y),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::TTF1_Z),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::TTF2_Y),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::TTF2_Z),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::CLI_X),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::CLI_Y),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::CLI_Z),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::CLI_R),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::CLI_C),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::STF_Y),
	//50
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::STF_UPPER_Z),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::STF_LOWER_Z),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::ROLLCONV),
	static_cast<signed>(Wmx3AxisMoveDir::Negative),//(PowerAxis::ROLLBT),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED1),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED2),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED3),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED4),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED5),
	static_cast<signed>(Wmx3AxisMoveDir::Positive),//(PowerAxis::RESERVED6)
};

CString PowerAxisAliasName[MAXAXISNO] = {
	_T("FX"),
	_T("FY1"),
	_T("FY2"),
	_T("FZ1"),
	_T("FZ2"),
	_T("FZ3"),
	_T("FZ4"),
	_T("FZ5"),
	_T("FZ6"),
	_T("FW1"),
	_T("FW2"),
	_T("FW3"),
	_T("FW4"),
	_T("FW5"),
	_T("FW6"),
	_T("FCONV"),
	_T("FPUZ"),
	_T("FBTIN"),
	_T("FBTWK"),
	_T("FBTOT"),
	_T("RX"),
	_T("RY1"),
	_T("RY2"),
	_T("RZ1"),
	_T("RZ2"),
	_T("RZ3"),
	_T("RZ4"),
	_T("RZ5"),
	_T("RZ6"),
	_T("RW1"),
	_T("RW2"),
	_T("RW3"),
	_T("RW4"),
	_T("RW5"),
	_T("RW6"),
	_T("RCONV"),
	_T("RPUZ"),
	_T("RBTIN"),
	_T("RBTWK"),
	_T("RBTOT"),
	_T("TTF1_Y"),
	_T("TTF1_Z"),
	_T("TTF2_Y"),
	_T("TTF2_Z"),
	_T("CLI_X"),
	_T("CLI_Y"),
	_T("CLI_Z"),
	_T("CLI_R"),
	_T("CLI_C"),
	_T("STF_Y"),
	_T("STF_UPPER_Z"),
	_T("STF_LOWER_Z"),
	_T("ROLLCONV"),
	_T("ROLLBT"),
	_T("RESERVED1"),
	_T("RESERVED2"),
	_T("RESERVED3"),
	_T("RESERVED4"),
	_T("RESERVED5"),
	_T("RESERVED6")
};

CString PowerAxisName[MAXAXISNO] = {
	_T("Front X"),
	_T("Front Y1"),
	_T("Front Y2"),
	_T("Front Z1"),
	_T("Front Z2"),
	_T("Front Z3"),
	_T("Front Z4"),
	_T("Front Z5"),
	_T("Front Z6"),
	_T("Front W1"),
	_T("Front W2"),
	_T("Front W3"),
	_T("Front W4"),
	_T("Front W5"),
	_T("Front W6"),
	_T("Front Conv"),
	_T("Front Pusher Z"),
	_T("Front Belt InBuffer"),
	_T("Front Belt Work"),
	_T("Front Belt OutBuffer"),
	_T("Rear X"),
	_T("Rear Y1"),
	_T("Rear Y2"),
	_T("Rear Z1"),
	_T("Rear Z2"),
	_T("Rear Z3"),
	_T("Rear Z4"),
	_T("Rear Z5"),
	_T("Rear Z6"),
	_T("Rear W1"),
	_T("Rear W2"),
	_T("Rear W3"),
	_T("Rear W4"),
	_T("Rear W5"),
	_T("Rear W6"),
	_T("Rear Conv"),
	_T("Rear Pusher Z"),
	_T("Rear Belt InBuffer"),
	_T("Rear Belt Work"),
	_T("Rear Belt OutBuffer"),
	_T("TTF1 Y"),
	_T("TTF1 Z"),
	_T("TTF2 Y"),
	_T("TTF2 Z"),
	_T("Clinching X"),
	_T("Clinching Y"),
	_T("Clinching Z"),
	_T("Clinching R"),
	_T("Clinching C"),
	_T("STF_Y"),
	_T("STF Upper Z"),
	_T("STF Lower Z"),
	_T("Roller Conv"),
	_T("Roller Belt"),
	_T("RESERVED1"),
	_T("RESERVED2"),
	_T("RESERVED3"),
	_T("RESERVED4"),
	_T("RESERVED5"),
	_T("RESERVED6")
};

double PowerAxisResol[MAXAXISNO] =
{
	0.001,	// FX									2 / 1
	0.001,	// FY1									2 / 1
	0.001,	// FY2									2 / 1
	0.001,	//0.00000095367431640625, // FZ1		8.0[mm] / 8388608 pulse
	0.001,	//0.00000095367431640625, // FZ2
	0.001,	//0.00000095367431640625, // FZ3
	0.001,	//0.00000095367431640625, // FZ4
	0.001,	//0.00000095367431640625, // FZ5
	0.001,	//0.00000095367431640625, // FZ6
	0.01, // FW1									360.0 [Angle] / 8388608 pulse
	0.01, // FW2
	0.01, // FW3
	0.01, // FW4
	0.01, // FW5
	0.01, // FW6
	0.0005, // FCONV								5.0 [mm] / 10000 pulse
	0.00097656250, // FPUZ							10.0 [mm] / 10240 pulse
	0.00034332275390625, // FBTIN
	0.00034332275390625, // FBTWK
	0.00034332275390625, // FBTOT
	0.001, // RX
	0.001, // RY1
	0.001, // RY2
	0.001,//0.00000095367431640625, // RZ1			8.0[mm] / 8388608 pulse
	0.001,//0.00000095367431640625, // RZ2
	0.001,//0.00000095367431640625, // RZ3
	0.001,//0.00000095367431640625, // RZ4
	0.001,//0.00000095367431640625, // RZ5
	0.001,//0.00000095367431640625, // RZ6
	0.01, // RW1									360.0 [Angle] / 8388608 pulse
	0.01, // RW2
	0.01, // RW3
	0.01, // RW4
	0.01, // RW5
	0.01, // RW6
	0.0005, // RCONV								5.0 [mm] / 10240 pulse
	0.00097656250, // RPUZ							10.0 [mm] / 10240 pulse
	0.00034332275390625, // RBTIN
	0.00034332275390625, // RBTWK
	0.00034332275390625, // RBTOT
	0.001, // TTF1_Y
	0.001, // TTF1_Z
	0.001, // TTF2_Y
	0.001, // TTF2_Z
	0.001, // CLI_X
	0.001, // CLI_Y
	0.001, // CLI_Z
	0.01,  // CLI_R
	0.001, // CLI_C
	0.001, // STF_Y
	0.001, // STF_UPPER_Z
	0.001, // STF_LOWER_Z
	0.001, // ROLLCONV							2.0 [mm] / 10000 pulse
	0.00034332275390625, // ROLLBT
	0.001,	// RESERVED1
	0.001,	// RESERVED2
	0.001,	// RESERVED3
	0.001,	// RESERVED4
	0.001,	// RESERVED5
	0.001	// RESERVED6
};

double PowerAxisUnResol[MAXAXISNO] =
{
	1000.0000000000000000000000000000, // FX		2 / 1
	1000.0000000000000000000000000000, // FY1
	1000.0000000000000000000000000000, // FY2
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ1
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ2 
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ3
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ4
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ5
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // FZ6
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW1
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW2
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW3
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW4 
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW5 
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // FW6
	2000.0000000000000000000000000000, // FCONV
	1024.0000000000000000000000000000, // FPUZ
	2912.7111111111111111111111111111, // FBTIN
	2912.7111111111111111111111111111, // FBTWK
	2912.7111111111111111111111111111, // FBTOT
	1000.0000000000000000000000000000, // RX
	1000.0000000000000000000000000000, // RY1
	1000.0000000000000000000000000000, // RY2
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ1
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ2
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ3
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ4
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ5
	1000.0000000000000000000000000000,//1048576.0000000000000000000000000, // RZ6
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW1
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW2
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW3
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW4
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW5
	100.00000000000000000000000000000,//23301.688888888888888888888888889, // RW6
	2000.0000000000000000000000000000, // RCONV
	1024.0000000000000000000000000000, // RPUZ
	2912.7111111111111111111111111111, // RBTIN
	2912.7111111111111111111111111111, // RBTWK
	2912.7111111111111111111111111111, // RBTOT
	1000.0000000000000000000000000000, // TTF1_Y
	1000.0000000000000000000000000000, // TTF1_Z
	1000.0000000000000000000000000000, // TTF2_Y
	1000.0000000000000000000000000000, // TTF2_Z
	1000.0000000000000000000000000000, // CLI_X
	1000.0000000000000000000000000000, // CLI_Y
	1000.0000000000000000000000000000, // CLI_Z
	100.0000000000000000000000000000, // CLI_R
	1000.0000000000000000000000000000, // CLI_C
	1000.0000000000000000000000000000, // STF_Y
	1000.0000000000000000000000000000, // STF_UPPER_Z
	1000.0000000000000000000000000000, // STF_LOWER_Z
	1000.0000000000000000000000000000, // ROLLCONV
	2912.7111111111111111111111111111, // ROLLBT
	1000.0000000000000000000000000000,	// RESERVED1
	1000.0000000000000000000000000000,	// RESERVED2
	1000.0000000000000000000000000000,	// RESERVED3
	1000.0000000000000000000000000000,	// RESERVED4
	1000.0000000000000000000000000000,	// RESERVED5
	1000.0000000000000000000000000000	// RESERVED6
};

bool PowerAxisUseMasterSlave[MAXAXISNO] =
{
	false, true, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, true, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false,
	false, false, false, false, false
};

unsigned PowerAxisWhatMasterSlave[MAXAXISNO] =
{
	static_cast<unsigned>(PowerAxis::FX),
	static_cast<unsigned>(PowerAxis::FY2),
	static_cast<unsigned>(PowerAxis::FY2),
	static_cast<unsigned>(PowerAxis::FZ1),
	static_cast<unsigned>(PowerAxis::FZ2),
	static_cast<unsigned>(PowerAxis::FZ3),
	static_cast<unsigned>(PowerAxis::FZ4),
	static_cast<unsigned>(PowerAxis::FZ5),
	static_cast<unsigned>(PowerAxis::FZ6),
	static_cast<unsigned>(PowerAxis::FW1),
	static_cast<unsigned>(PowerAxis::FW2),
	static_cast<unsigned>(PowerAxis::FW3),
	static_cast<unsigned>(PowerAxis::FW4),
	static_cast<unsigned>(PowerAxis::FW5),
	static_cast<unsigned>(PowerAxis::FW6),
	static_cast<unsigned>(PowerAxis::FCONV),
	static_cast<unsigned>(PowerAxis::FPUZ),
	static_cast<unsigned>(PowerAxis::FBTIN),
	static_cast<unsigned>(PowerAxis::FBTWK),
	static_cast<unsigned>(PowerAxis::FBTOT),
	static_cast<unsigned>(PowerAxis::RX),
	static_cast<unsigned>(PowerAxis::RY2),
	static_cast<unsigned>(PowerAxis::RY2),
	static_cast<unsigned>(PowerAxis::RZ1),
	static_cast<unsigned>(PowerAxis::RZ2),
	static_cast<unsigned>(PowerAxis::RZ3),
	static_cast<unsigned>(PowerAxis::RZ4),
	static_cast<unsigned>(PowerAxis::RZ5),
	static_cast<unsigned>(PowerAxis::RZ6),
	static_cast<unsigned>(PowerAxis::RW1),
	static_cast<unsigned>(PowerAxis::RW2),
	static_cast<unsigned>(PowerAxis::RW3),
	static_cast<unsigned>(PowerAxis::RW4),
	static_cast<unsigned>(PowerAxis::RW5),
	static_cast<unsigned>(PowerAxis::RW6),
	static_cast<unsigned>(PowerAxis::RCONV),
	static_cast<unsigned>(PowerAxis::RPUZ),
	static_cast<unsigned>(PowerAxis::RBTIN),
	static_cast<unsigned>(PowerAxis::RBTWK),
	static_cast<unsigned>(PowerAxis::RBTOT),
	static_cast<unsigned>(PowerAxis::TTF1_Y),
	static_cast<unsigned>(PowerAxis::TTF1_Z),
	static_cast<unsigned>(PowerAxis::TTF2_Y),
	static_cast<unsigned>(PowerAxis::TTF2_Z),
	static_cast<unsigned>(PowerAxis::CLI_X),
	static_cast<unsigned>(PowerAxis::CLI_Y),
	static_cast<unsigned>(PowerAxis::CLI_Z),
	static_cast<unsigned>(PowerAxis::CLI_R),
	static_cast<unsigned>(PowerAxis::CLI_C),
	static_cast<unsigned>(PowerAxis::STF_Y),
	static_cast<unsigned>(PowerAxis::STF_UPPER_Z),
	static_cast<unsigned>(PowerAxis::STF_LOWER_Z),
	static_cast<unsigned>(PowerAxis::ROLLCONV),
	static_cast<unsigned>(PowerAxis::ROLLBT),
	static_cast<unsigned>(PowerAxis::RESERVED1),
	static_cast<unsigned>(PowerAxis::RESERVED2),
	static_cast<unsigned>(PowerAxis::RESERVED3),
	static_cast<unsigned>(PowerAxis::RESERVED4),
	static_cast<unsigned>(PowerAxis::RESERVED5),
	static_cast<unsigned>(PowerAxis::RESERVED6)
};

CMotionControlType PowerAxisMotionControlType[MAXAXISNO] =
{
	CMotionControlType::Wmx3Motion,//(PowerAxis::FX),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FY1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FY2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ3),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ4),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ5),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FZ6),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW3),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW4),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW5),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FW6),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FCONV),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FPUZ),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FBTIN),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FBTWK),
	CMotionControlType::Wmx3Motion,//(PowerAxis::FBTOT),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RX),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RY1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RY2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ3),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ4),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ5),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RZ6),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW3),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW4),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW5),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RW6),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RCONV),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RPUZ),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RBTIN),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RBTWK),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RBTOT),
	CMotionControlType::Wmx3Motion,//(PowerAxis::TTF1_Y),
	CMotionControlType::Wmx3Motion,//(PowerAxis::TTF1_Z),
	CMotionControlType::Wmx3Motion,//(PowerAxis::TTF2_Y),
	CMotionControlType::Wmx3Motion,//(PowerAxis::TTF2_Z),
	CMotionControlType::Wmx3Motion,//(PowerAxis::CLI_X),
	CMotionControlType::Wmx3Motion,//(PowerAxis::CLI_Y),
	CMotionControlType::Wmx3Motion,//(PowerAxis::CLI_Z),
	CMotionControlType::Wmx3Motion,//(PowerAxis::CLI_R),
	CMotionControlType::Wmx3Motion,//(PowerAxis::CLI_C),
	CMotionControlType::Wmx3Motion,//(PowerAxis::STF_Y),
	CMotionControlType::Wmx3Motion,//(PowerAxis::STF_UPPER_Z),
	CMotionControlType::Wmx3Motion,//(PowerAxis::STF_LOWER_Z),
	CMotionControlType::Wmx3Motion,//(PowerAxis::ROLLCONV),
	CMotionControlType::Wmx3Motion,//(PowerAxis::ROLLBT),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RESERVED1),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RESERVED2),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RESERVED3),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RESERVED4),
	CMotionControlType::Wmx3Motion,//(PowerAxis::RESERVED5),
	CMotionControlType::Wmx3Motion //(PowerAxis::RESERVED6)
};

CMotorType PowerAxisMotorType[MAXAXISNO] =
{
	CMotorType::ServoMotor,//(PowerAxis::FX),
	CMotorType::ServoMotor,//(PowerAxis::FY1),
	CMotorType::ServoMotor,//(PowerAxis::FY2),
	CMotorType::ServoMotor,//(PowerAxis::FZ1),
	CMotorType::ServoMotor,//(PowerAxis::FZ2),
	CMotorType::ServoMotor,//(PowerAxis::FZ3),
	CMotorType::ServoMotor,//(PowerAxis::FZ4),
	CMotorType::ServoMotor,//(PowerAxis::FZ5),
	CMotorType::ServoMotor,//(PowerAxis::FZ6),
	CMotorType::ServoMotor,//(PowerAxis::FW1),
	CMotorType::ServoMotor,//(PowerAxis::FW2),
	CMotorType::ServoMotor,//(PowerAxis::FW3),
	CMotorType::ServoMotor,//(PowerAxis::FW4),
	CMotorType::ServoMotor,//(PowerAxis::FW5),
	CMotorType::ServoMotor,//(PowerAxis::FW6),
	CMotorType::StepServoMotor,//(PowerAxis::FCONV),
	CMotorType::StepServoMotor,//(PowerAxis::FPUZ),
	CMotorType::StepMotor,//(PowerAxis::FBTIN),
	CMotorType::StepMotor,//(PowerAxis::FBTWK),
	CMotorType::StepMotor,//(PowerAxis::FBTOT),
	CMotorType::ServoMotor,//(PowerAxis::RX),
	CMotorType::ServoMotor,//(PowerAxis::RY1),
	CMotorType::ServoMotor,//(PowerAxis::RY2),
	CMotorType::ServoMotor,//(PowerAxis::RZ1),
	CMotorType::ServoMotor,//(PowerAxis::RZ2),
	CMotorType::ServoMotor,//(PowerAxis::RZ3),
	CMotorType::ServoMotor,//(PowerAxis::RZ4),
	CMotorType::ServoMotor,//(PowerAxis::RZ5),
	CMotorType::ServoMotor,//(PowerAxis::RZ6),
	CMotorType::ServoMotor,//(PowerAxis::RW1),
	CMotorType::ServoMotor,//(PowerAxis::RW2),
	CMotorType::ServoMotor,//(PowerAxis::RW3),
	CMotorType::ServoMotor,//(PowerAxis::RW4),
	CMotorType::ServoMotor,//(PowerAxis::RW5),
	CMotorType::ServoMotor,//(PowerAxis::RW6),
	CMotorType::StepServoMotor,//(PowerAxis::RCONV),
	CMotorType::StepServoMotor,//(PowerAxis::RPUZ),
	CMotorType::StepMotor,//(PowerAxis::RBTIN),
	CMotorType::StepMotor,//(PowerAxis::RBTWK),
	CMotorType::StepMotor,//(PowerAxis::RBTOT),
	CMotorType::ServoMotor,//(PowerAxis::TTF1_Y),
	CMotorType::ServoMotor,//(PowerAxis::TTF1_Z),
	CMotorType::ServoMotor,//(PowerAxis::TTF2_Y),
	CMotorType::ServoMotor,//(PowerAxis::TTF2_Z),
	CMotorType::ServoMotor,//(PowerAxis::CLI_X),
	CMotorType::ServoMotor,//(PowerAxis::CLI_Y),
	CMotorType::ServoMotor,//(PowerAxis::CLI_Z),
	CMotorType::ServoMotor,//(PowerAxis::CLI_R),
	CMotorType::ServoMotor,//(PowerAxis::CLI_C),
	CMotorType::ServoMotor,//(PowerAxis::STF_Y),
	CMotorType::ServoMotor,//(PowerAxis::STF_UPPER_Z),
	CMotorType::ServoMotor,//(PowerAxis::STF_LOWER_Z),
	CMotorType::StepServoMotor,//(PowerAxis::ROLLCONV),
	CMotorType::StepMotor,//(PowerAxis::ROLLBT),
	CMotorType::StepMotor,//(PowerAxis::RESERVED1),
	CMotorType::StepMotor,//(PowerAxis::RESERVED2),
	CMotorType::StepMotor,//(PowerAxis::RESERVED3),
	CMotorType::StepMotor,//(PowerAxis::RESERVED4),
	CMotorType::StepMotor,//(PowerAxis::RESERVED5),
	CMotorType::StepMotor//(PowerAxis::RESERVED6)
};

/// <summary>
/// Y1, Y2 축 - 최대속도 1.1m / s, 가속도 1.3G, 휴지타임 500msec
/// X축 - 최대속도 1.1m / s, 가속도 1.3G, 휴지타임 500msec
/// 세우산전 모터부 온도센서는 B접점으로(온도 변화량 체크용이 아닌) 100도씨 이하에서 접점이 붙어 있다가, 
/// 100도씨 초과시 접점이 떨어지는, 온도 조건에 따른 사용 유 / 무 확인용 센서 입니다.
/// </summary>

WMX3_AXIS_POSCOMMANDPROFILE PowerAxisMoveParam[MAXAXISNO] =
{
	//Profile Type, velocity, acc,dec,	jerkAcc, jerkDec, jerkAccRatio, jerkDecRatio, accTimeMilliseconds, decTimeMilliseconds, startingVelocity, endVelocity, secondVelocity, movingAverageTimeMilliseconds
	{WMX3ProfileType::JerkRatio, 1500000, 15000000, 15000000, 1000, 1000, 0.3, 0.3, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 1500000, 15000000, 15000000, 1000, 1000, 0.3, 0.3, 0, 0, 0, 0, 0, 0},//(PowerAxis::FX),//{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FX),
	{WMX3ProfileType::JerkRatio, 1500000, 15000000, 15000000, 1000, 1000, 0.3, 0.3, 0, 0, 0, 0, 0, 0},//(PowerAxis::FY1),//{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FY1),
	{WMX3ProfileType::JerkRatio, 1500000, 15000000, 15000000, 1000, 1000, 0.3, 0.3, 0, 0, 0, 0, 0, 0},//(PowerAxis::FY2),//{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FY2),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ1),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ2),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ3),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ4),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ5),
	{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 800000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FZ6),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FW1),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FW2),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0,0},//(PowerAxis::FW3),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FW4),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FW5),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FW6),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FCONV),
	{WMX3ProfileType::JerkRatio, 1200000, 1200000, 1200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FPUZ),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FBTIN),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FBTWK),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::FBTOT),
	{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RX),
	{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RY1),
	{WMX3ProfileType::JerkRatio, 1000000, 10000000, 10000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RY2),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ1),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ2),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ3),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ4),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ5),
	{WMX3ProfileType::JerkRatio, 500000, 5000000, 5000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//{WMX3ProfileType::JerkRatio, 524288000, 188743680000, 1258291200, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RZ6),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW1),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW2),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW3),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW4),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW5),
	{WMX3ProfileType::JerkRatio, 1800000, 18000000, 18000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RW6),
	{WMX3ProfileType::JerkRatio, 50000, 500000, 500000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RCONV),
	{WMX3ProfileType::JerkRatio, 1000000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RPUZ),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RBTIN),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RBTWK),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RBTOT),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::TTF1_Y),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::TTF1_Z),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::TTF2_Y),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::TTF2_Z),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::CLI_X),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::CLI_Y),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::CLI_Z),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::CLI_R),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::CLI_C),
	{WMX3ProfileType::SCurve,    2500000, 150000, 150000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},//(PowerAxis::STF_Y),
	{WMX3ProfileType::SCurve,    200000, 600000, 600000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},//(PowerAxis::STF_UPPER_Z),
	{WMX3ProfileType::SCurve,    200000, 600000, 600000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},//(PowerAxis::STF_LOWER_Z),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::ROLLCONV),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::ROLLBT),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RESERVED1),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RESERVED2),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RESERVED3),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RESERVED4),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},//(PowerAxis::RESERVED5),
	{WMX3ProfileType::JerkRatio, 100000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}//(PowerAxis::RESERVED6)
};

WMX3_AXIS_POSCOMMANDPROFILE PowerAxisTeachMoveParam[MAXAXISNO] =
{
	//Profile Type, velocity, acc, dec,	jerkAcc, jerkDec, jerkAccRatio, jerkDecRatio, accTimeMilliseconds, decTimeMilliseconds, startingVelocity, endVelocity, secondVelocity, movingAverageTimeMilliseconds
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FX),
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FY1),
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FY2),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ1),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ2),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ3),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ4),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ5),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FZ6),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW1),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW2),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW3),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW4),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW5),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FW6),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FCONV),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FPUZ),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FBTIN),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FBTWK),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::FBTOT),
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RX),
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RY1),
	{WMX3ProfileType::JerkRatio, 75000, 750000, 750000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RY2),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ1),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ2),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ3),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ4),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ5),
	{WMX3ProfileType::JerkRatio, 100000, 1000000, 1000000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RZ6),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW1),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW2),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW3),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW4),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW5),
	{WMX3ProfileType::JerkRatio, 180000, 1800000, 1800000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RW6),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RCONV),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RPUZ),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RBTIN),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RBTWK),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RBTOT),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::TTF1_Y),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::TTF1_Z),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::TTF2_Y),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::TTF2_Z),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::CLI_X),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::CLI_Y),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::CLI_Z),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::CLI_R),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::CLI_C),
	{WMX3ProfileType::SCurve,    250000, 1250000, 1250000,  1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},//(PowerAxis::STF_Y),
	{WMX3ProfileType::SCurve,    100000, 300000, 300000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},   //(PowerAxis::STF_UPPER_Z),
	{WMX3ProfileType::SCurve,    100000, 300000, 300000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},   //(PowerAxis::STF_LOWER_Z),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::ROLLCONV),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::ROLLBT),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RESERVED1),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RESERVED2),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RESERVED3),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RESERVED4),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0},	//(PowerAxis::RESERVED5),
	{WMX3ProfileType::JerkRatio, 200000, 200000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}	//(PowerAxis::RESERVED6)
};

WMX3_AXIS_POSCOMMANDPROFILE PowerAxisTeachJogParam[MAXAXISNO] =
{
	//Profile Type, velocity, acc, dec,	jerkAcc, jerkDec, jerkAccRatio, jerkDecRatio, accTimeMilliseconds, decTimeMilliseconds, startingVelocity, endVelocity, secondVelocity, movingAverageTimeMilliseconds
	{WMX3ProfileType::JerkRatio, 10000, 100000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FX),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FY1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 200000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FY2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ3),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ4),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ5),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FZ6),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW3),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW4),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW5),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FW6),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FCONV),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FPUZ),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FBTIN),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FBTWK),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::FBTOT),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RX),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RY1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RY2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ3),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ4),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ5),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RZ6),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW3),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW4),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW5),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RW6),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RCONV),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RPUZ),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RBTIN),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RBTWK),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RBTOT),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::TTF1_Y),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::TTF1_Z),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::TTF2_Y),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::TTF2_Z),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::CLI_X),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::CLI_Y),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::CLI_Z),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::CLI_R),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::CLI_C),
	{WMX3ProfileType::SCurve,    250000, 1250000, 1250000,  1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},//(PowerAxis::STF_Y),
	{WMX3ProfileType::SCurve,    100000, 300000, 300000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},   //(PowerAxis::STF_UPPER_Z),
	{WMX3ProfileType::SCurve,    100000, 300000, 300000, 1000, 1000, 1.0, 1.0, 0, 0, 0, 0, 0, 0},   //(PowerAxis::STF_LOWER_Z),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::ROLLCONV),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::ROLLBT),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RESERVED1),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RESERVED2),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RESERVED3),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RESERVED4),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}, //(PowerAxis::RESERVED5),
	{WMX3ProfileType::JerkRatio, 10000, 100000, 100000, 1000, 1000, 0.5, 0.5, 0, 0, 0, 0, 0, 0}  //(PowerAxis::RESERVED6)
};

#if EARTH
double gShortDist[MAX_SHORT_DIST_LEVEL]
{
	2.0,
	4.0,
	6.0,
	8.0,
	10.0,
	12.0,
	14.0,
	16.0,
	18.0,
	20.0,
	22.0,
	24.0,
	26.0,
	28.0,
	30.0,
	40.0,
	50.0,
	100.0,
	150.0,
	160.0
};

double gShortDistVel[MAX_SHORT_DIST_LEVEL]
{
	82500,
	123750,
	165000,
	206250,
	247500,
	288750,
	330000,
	371250,
	412500,
	453750,
	495000,
	536250,
	577500,
	618750,
	660000,
	742500,
	800000,
	1000000,
	1300000,
	1400000
};

double gShortDistAccDec[MAX_SHORT_DIST_LEVEL] =
{
	825000,
	1237500,
	1650000,
	2062500,
	2475000,
	2887500,
	3300000,
	3712500,
	4125000,
	4537500,
	4950000,
	5362500,
	5775000,
	6187500,
	6600000,
	7425000,
	8000000,
	10000000,
	13000000,
	14000000
};

#else // Original
	double gShortDist[MAX_SHORT_DIST_LEVEL]
	{
		2.0,
		4.0,
		6.0,
		8.0,
		10.0,
		12.0,
		14.0,
		16.0,
		18.0,
		20.0,
		22.0,
		24.0,
		26.0,
		28.0,
		30.0,
		40.0,
		50.0,
		100.0,
		150.0,
		160.0
	};

	double gShortDistVel[MAX_SHORT_DIST_LEVEL]
	{
		41250,
		82500,
		123750,
		165000,
		206250,
		247500,
		288750,
		330000,
		371250,
		412500,
		453750,
		495000,
		536250,
		577500,
		618750,
		660000,
		742500,
		1237500,
		1451250,
		1500000
	};

	double gShortDistAccDec[MAX_SHORT_DIST_LEVEL] =
	{
		904110,
		1783784,
		2675676,
		3520000,
		4400000,
		5280000,
		5923077,
		6683544,
		7156627,
		7951807,
		8541176,
		9317647,
		10094118,
		10620690,
		11379310,
		11115789,
		11880000,
		15230769,
		14980645,
		15000000
	};
#endif

ULONGLONG PowerHomingMaxTimeOut[MAXAXISNO]
{
	30000,//(PowerAxis::FX),
	60000,//(PowerAxis::FY1),
	60000,//(PowerAxis::FY2),
	10000,//(PowerAxis::FZ1),
	10000,//(PowerAxis::FZ2),
	10000,//(PowerAxis::FZ3),
	10000,//(PowerAxis::FZ4),
	10000,//(PowerAxis::FZ5),
	10000,//(PowerAxis::FZ6),
	5000 ,//(PowerAxis::FW1),
	5000 ,//(PowerAxis::FW2),
	5000 ,//(PowerAxis::FW3),
	5000 ,//(PowerAxis::FW4),
	5000 ,//(PowerAxis::FW5),
	5000 ,//(PowerAxis::FW6),
	30000,//(PowerAxis::FCONV),
	10000,//(PowerAxis::FPUZ),
	5000 ,//(PowerAxis::FBTIN),
	5000 ,//(PowerAxis::FBTWK),
	5000 ,//(PowerAxis::FBTOT),
	30000,//(PowerAxis::RX),
	60000,//(PowerAxis::RY1),
	60000,//(PowerAxis::RY2),
	10000,//(PowerAxis::RZ1),
	10000,//(PowerAxis::RZ2),
	10000,//(PowerAxis::RZ3),
	10000,//(PowerAxis::RZ4),
	10000,//(PowerAxis::RZ5),
	10000,//(PowerAxis::RZ6),
	5000 ,//(PowerAxis::RW1),
	5000 ,//(PowerAxis::RW2),
	5000 ,//(PowerAxis::RW3),
	5000 ,//(PowerAxis::RW4),
	5000 ,//(PowerAxis::RW5),
	5000 ,//(PowerAxis::RW6),
	30000,//(PowerAxis::RCONV),
	10000,//(PowerAxis::RPUZ),
	5000 ,//(PowerAxis::RBTIN),
	5000 ,//(PowerAxis::RBTWK),
	5000 ,//(PowerAxis::RBTOT),
	30000,//(PowerAxis::TTF1_Y),
	30000,//(PowerAxis::TTF1_Z),
	30000,//(PowerAxis::TTF2_Y),
	30000,//(PowerAxis::TTF2_Z),
	10000,//(PowerAxis::CLI_X),
	10000,//(PowerAxis::CLI_Y),
	10000,//(PowerAxis::CLI_Z),
	10000,//(PowerAxis::CLI_R),
	10000,//(PowerAxis::CLI_C),
	30000,//(PowerAxis::STF_Y),
	30000,//(PowerAxis::STF_UPPER_Z),
	30000,//(PowerAxis::STF_LOWER_Z),
	30000,//(PowerAxis::ROLLCONV),
	5000,//(PowerAxis::ROLLBT),
	10000,//(PowerAxis::RESERVED1),
	10000,//(PowerAxis::RESERVED2),
	10000,//(PowerAxis::RESERVED3),
	10000,//(PowerAxis::RESERVED4),
	10000,//(PowerAxis::RESERVED5),
	10000//(PowerAxis::RESERVED6)
};

//void SetShortDist(long ShortDistNo, double ShortDistance)
//{
//	if (ShortDistNo < MAX_SHORT_DIST_LEVEL)
//	{
//		gShortDist[ShortDistNo] = ShortDistance;
//		TRACE(_T("[PWR] SetShortDist ShortNo:%d Dist:%.3f\n"), ShortDistNo, ShortDistance);
//	}
//}
//
//void SetShortDistMaxVel(long ShortDistNo, double ShortDistMaxVel)
//{
//	if (ShortDistNo < MAX_SHORT_DIST_LEVEL)
//	{
//		gShortDistVel[ShortDistNo] = ShortDistMaxVel;
//		TRACE(_T("[PWR] SetShortDistMaxVel ShortNo:%d Dist:%.3f\n"), ShortDistNo, ShortDistMaxVel);
//	}
//}

long GetAxisMap(CString strAxis)
{
	long AxisMap = NON;
	AxisMap = gcPowerGantry->GetAxisMap(strAxis);
	return AxisMap;
}

CString GetAxisX(long Gantry)
{
	CString strAxisX = _T("NON");
	strAxisX = gcPowerGantry->GetAxisX(Gantry);
	return strAxisX;
}

CString GetAxisY1(long Gantry)
{
	CString strAxisY = _T("NON");
	strAxisY = gcPowerGantry->GetAxisY1(Gantry);
	return strAxisY;
}

CString GetAxisY2(long Gantry)
{
	CString strAxisY = _T("NON");
	strAxisY = gcPowerGantry->GetAxisY2(Gantry);
	return strAxisY;
}

CString GetConvName(long Conveyor)
{
	CString strConv = _T("NON");
	strConv = gcPowerGantry->GetConvName(Conveyor);
	return strConv;
}

CString GetPusherZName(long Conveyor)
{
	CString strPusherZ = _T("NON");
	strPusherZ = gcPowerGantry->GetPusherZName(Conveyor);
	return strPusherZ;
}

CString GetRAxisFromHeadNo(long Gantry, long HeadNo)
{
	CString strZAxis = _T("NON");
	strZAxis = gcPowerGantry->GetRAxisFromHeadNo(Gantry, HeadNo);
	return strZAxis;
}

CString GetZAxisFromHeadNo(long Gantry, long HeadNo)
{
	CString strZAxis = _T("NON");
	strZAxis = gcPowerGantry->GetZAxisFromHeadNo(Gantry, HeadNo);
	return strZAxis;
}

CString GetHeadToRotateAxis(long Gantry, CString strZAxis)
{
	CString strRAxis = _T("NON");
	strRAxis = gcPowerGantry->GetHeadToRotateAxis(Gantry, strZAxis);
	return strRAxis;
}

CString GetRotateAxisToHead(long Gantry, CString strRAxis)
{
	CString strZAxis = _T("NON");
	strZAxis = gcPowerGantry->GetRotateAxisToHead(Gantry, strRAxis);
	return strZAxis;
}

void SetOneRatio(CString strAxis, double Ratio)
{
	gcPowerGantry->SetOneRatio(strAxis, Ratio);
}

void SetOne2ndRatio(CString strAxis, double Ratio2nd)
{
	gcPowerGantry->SetOne2ndRatio(strAxis, Ratio2nd);
}

void InitOneRatio(CString strAxis)
{
	gcPowerGantry->InitOneRatio(strAxis);
}

long SetOnePosSet(CString strAxis, double Inpos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetOnePosSet(strAxis, Inpos);
	return Err;
}

long SetOneInPos(CString strAxis, double Inpos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetOneInPos(strAxis, Inpos);
	return Err;
}

long SetOneDelayedPosSet(CString strAxis, double Inpos, long Time)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetOneDelayedPosSet(strAxis, Inpos, Time);
	return Err;
}

long StartOneJog(CString strAxis, JogInfo jogInfo)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOneJog(strAxis, jogInfo);
	return Err;
}

long StartOnePosition(CString strAxis, double Position)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOnePosition(strAxis, Position);
	return Err;
}

long StartOnePositionSkipLimitCheck(CString strAxis, double Position)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOnePositionSkipLimitCheck(strAxis, Position);
	return Err;
}
long StartOne2StepPosition(CString strAxis, double Position, double Position2nd)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOne2StepPosition(strAxis, Position, Position2nd);
	return Err;
}

long StartPositionX(CString strAxis, double Position, double Ratio)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOnePositionX(strAxis, Position, Ratio);
	return Err;
}

long StartPositionY(CString strAxis, double Position, double Ratio)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOnePositionY(strAxis, Position, Ratio);
	return Err;
}

long StartOnePositionWithoutSlave(CString strAxis, double Position)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOnePositionWithoutSlave(strAxis, Position);
	return Err;
}

long StartOneMove(CString strAxis, double Offset)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOneMove(strAxis, Offset);
	return Err;
}

long StartOneMoveWithoutSlave(CString strAxis, double Offset)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOneMoveWithoutSlave(strAxis, Offset);
	return Err;
}

long StartOneTeachMove(CString strAxis, double Offset)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartOneTeachMove(strAxis, Offset);
	return Err;
}

long StopOne(CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StopOne(strAxis);
	return Err;
}

long StopOne(CString strAxis, double Dec)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StopOne(strAxis, Dec);
	return Err;
}

long GetOneOnlyIdle(CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->GetOneOnlyIdle(strAxis);
	return Err;
}


long WaitOneIdle(CString strAxis, long TimeOut)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->WaitOneIdle(strAxis, TimeOut);
	return Err;
}

long WaitOneMotion(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->IsWaitOnePos(strAxis, CmdPos, Inpos, TimeOut);
	if (Err == NO_ERR) // 20210225 HarkDo
	{
		Err = gcPowerGantry->WaitOneMotion(strAxis, CmdPos, TimeOut);
	}
	return Err;
}

long WaitOnePosSet(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->IsWaitOnePos(strAxis, CmdPos, Inpos, TimeOut);
	if (Err == NO_ERR) // 20210225 HarkDo
	{
		Err = gcPowerGantry->WaitOnePosSet(strAxis, CmdPos, TimeOut);
	}
	return Err;
}

long WaitOneInPos(CString strAxis, double CmdPos, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->IsWaitOnePos(strAxis, CmdPos, Inpos, TimeOut);
	//if (Err == NO_ERR) // 20210225 HarkDo
	//{
	//	Err = gcPowerGantry->WaitOneInPos(strAxis, CmdPos, TimeOut);
	//}
	return Err;
}

long WaitOneDelayedPosSet(CString strAxis, double CmdPos, double Inpos, long TimeOut, bool UseSendAlarm)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->IsWaitOnePos(strAxis, CmdPos, Inpos, TimeOut, UseSendAlarm);
	if (Err == NO_ERR) // 20210225 HarkDo
	{
		Err = gcPowerGantry->WaitOneDelayedPosSet(strAxis, CmdPos, TimeOut, UseSendAlarm);
	}
	return Err;
}

long OneOriginSearch(CString strAxis, double forceOffset)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->OneOriginSearch(strAxis, forceOffset);
	return Err;
}

long SetInitializeEnd(CString strAxis, bool bEnd)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetInitializeEnd(strAxis, bEnd);
	return Err;
}

long GetTorqueLimit(CString strAxis, double* pTorqueLimit)
{
	long Err = NO_ERR;
	double TorqueLimit = 0.0;
	Err = gcPowerGantry->GetTorqueLimit(strAxis, &TorqueLimit);
	*pTorqueLimit = TorqueLimit;
	return Err;
}

long SetTorqueLimit(CString strAxis, double Torque)
{
	long Err = NO_ERR;
	double TorqueLimit = Torque;
	Err = gcPowerGantry->SetTorqueLimit(strAxis, TorqueLimit);
	return Err;
}

long SetEventToStopByOverTorque(long EventId, CString strAxis, double Torque, double SaftyPosition)
{
	long Err = NO_ERR;
	double MaxTorque = Torque;
	Err = gcPowerGantry->SetEventToStopByOverTorque(EventId, strAxis, MaxTorque, SaftyPosition);
	return Err;
}

long SetEventToOverrideVelByPosition(long EventId, CString strAxis, double position, double ratio)
{
	long Err = NO_ERR;
	double Position = position;
	double Ratio = ratio;
	Err = gcPowerGantry->SetEventToOverrideVelByPosition(EventId, strAxis, Position, Ratio);
	return Err;
}

long SetEventToStopByAreaSensor(long EventID, long AreaSensor, CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetEventToStopByAreaSensor(EventID, AreaSensor, strAxis);
	return Err;
}

long ClearAllEvent()
{
	long Err = NO_ERR;
	Err = gcPowerGantry->ClearAllEvent();
	return Err;
}

long RemoveEvent(long EventId)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->RemoveEvent(EventId);
	return Err;
}

long EnableEvent(long EventId)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->EnableEvent(EventId);
	return Err;
}

long StartMonitor(CString strAxis, long BoardCount, long BlockNo, long InsertNo, CString Action)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StartWmx3Monitor(strAxis, BoardCount, BlockNo, InsertNo, Action);
	return Err;
}

CString GetTorqueMonitorFileName(CString strAxis)
{
	CString strFileName;
	strFileName = gcPowerGantry->GetTorqueMonitorFileName(strAxis);
	return strFileName;
}

long StopMonitor(CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->StopWmx3Monitor(strAxis);
	return Err;
}

long WaitStopMonitor(CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->WaitStopMonitor(strAxis);
	return Err;
}

long ResetMonitor(CString strAxis)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->ResetMonitor(strAxis);
	return Err;
}

void SetShortDist(CString strAxis, long index, double ShortDist)
{
	gcPowerGantry->SetShortDist(strAxis, index, ShortDist);
}

void SetShortDistVel(CString strAxis, long index, double ShortDistMaxVel)
{
	gcPowerGantry->SetShortDistVel(strAxis, index, ShortDistMaxVel);
}

void SetShortDistAccDec(CString strAxis, long index, double ShortDistMaxAccDec)
{
	gcPowerGantry->SetShortDistAccDec(strAxis, index, ShortDistMaxAccDec);
}

void SetMoveProfile(CString strAxis, WMX3_AXIS_POSCOMMANDPROFILE profile)
{
	gcPowerGantry->SetMoveProfile(strAxis, profile);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double GetUnResol(CString strAxis)
{
	double Position = 0.0;
	Position = gcPowerGantry->GetUnResol(strAxis);
	return Position;
}

double GetDryRunZHeightOffset()
{
	return gcPowerGantry->GetDryRunZHeightOffset();
}

void SetInsertByZ(long Gantry, double InsertByZ)
{
	gcPowerGantry->SetInsertByZ(FRONT_GANTRY, InsertByZ);
}

double GetInsertByZ(long Gantry)
{
	double InsertByZ = 0.0;
	InsertByZ = gcPowerGantry->GetInsertByZ(FRONT_GANTRY);
	return InsertByZ;
}

void SetPusherByZ(long Gantry, double PusherByZ)
{
	gcPowerGantry->SetPusherByZ(FRONT_GANTRY, PusherByZ);
}

double GetPusherByZ(long Conveyor)
{
	double PusherByZ = 0.0;
	PusherByZ = gcPowerGantry->GetPusherByZ(Conveyor);
	return PusherByZ;
}

void SetStandByZ(long Gantry, double StandByZ)
{
	gcPowerGantry->SetStandByZ(FRONT_GANTRY, StandByZ);
}

double GetStandByZ(long Gantry)
{
	double StandByZ = 0.0;
	double maxComp = 0.000;

	StandByZ = gcPowerGantry->GetStandByZ(FRONT_GANTRY);

	maxComp = gcLastPickFront->GetMaxHeight();

	return StandByZ - maxComp;
}

double GetRuntimeSafetyZ(long Gantry)
{
	double StandByZ = 0.0;
	double maxComp = gcLastPickFront->GetMaxHeight();

	if (GetRunModeNoLog() != PROD_RUN)
	{
		return GetTorqueOverSafetyZ();
	}
	else
	{
		double TargetZ_Standby = gcReadJobFile->GetStandyBy().pt.z;
		double TargetZ_PickupStandby = gcReadJobFile->GetPickupZStandBy();
		double TargetZ_MaxComponent = GetInsertByZ(FRONT_GANTRY) - gcReadJobFile->GetPcb().MaxComponentHeight;
		double TargetZ;

		if (TargetZ_Standby < TargetZ_PickupStandby && TargetZ_Standby < TargetZ_MaxComponent)
		{
			TargetZ = TargetZ_Standby;
		}
		else if (TargetZ_MaxComponent < TargetZ_PickupStandby)
		{
			TargetZ = TargetZ_MaxComponent;
		}
		else
		{
			TargetZ = TargetZ_PickupStandby;
		}
		return TargetZ - maxComp;
	}
}

double GetTorqueOverSafetyZ()
{
	double minusLimit = 0.0;

	for (long HeadNo = TBL_HEAD1; HeadNo <= GetZAxisCount(); HeadNo++)
	{
		CString strZAxis = GetZAxisFromHeadNo(FRONT_GANTRY, HeadNo);

		if (HeadNo == TBL_HEAD1)
		{
			minusLimit = GetLimit(GetAxisIndexFromAliasName(strZAxis)).minus;
		}

		if (minusLimit < GetLimit(GetAxisIndexFromAliasName(strZAxis)).minus)
		{
			minusLimit = GetLimit(GetAxisIndexFromAliasName(strZAxis)).minus;
		}
	}

	return minusLimit + 5.0;
}

void SetMaxZTorqueLimit(long Gantry, long Head, double MaxZTorqueLimit)
{
	gcPowerGantry->SetMaxZTorqueLimit(Gantry, Head, MaxZTorqueLimit);
}

double GetMaxZTorqueLimit(long Gantry, long Head)
{
	double MaxZTorqueLimit = 80.0;
	MaxZTorqueLimit = gcPowerGantry->GetMaxZTorqueLimit(FRONT_GANTRY, Head);
	return MaxZTorqueLimit;
}

void SetStandByR(long Gantry, double StandByR)
{
	gcPowerGantry->SetStandByR(FRONT_GANTRY, StandByR);
}

double GetStandByR(long Gantry)
{
	double StandByR = 0.0;
	StandByR = gcPowerGantry->GetStandByR(FRONT_GANTRY);
	return StandByR;
}

void SetHeadOffset(long Gantry, long HeadNo, Point_XY Offset)
{
	gcPowerGantry->SetHeadOffset(HeadNo, Offset);
}

Point_XY GetHeadOffset(long Gantry, long HeadNo)
{
	Point_XY Offset;
	Offset = gcPowerGantry->GetHeadOffset(HeadNo);
	return Offset;
}

void SetCameraRecognitionPosition(long Gantry, long HeadNo, Point_XY Position)
{
	if (Gantry == FRONT_GANTRY)
		gcPowerGantry->SetCameraRecognitionPosition(HeadNo, Position);
	else
		gcPowerGantry->SetRearCameraRecognitionPosition(HeadNo, Position);
}

void SetCameraRecognitionOffset(long Gantry, long HeadNo, Point_XY Offset)
{
	if (Gantry == FRONT_GANTRY)
		gcPowerGantry->SetCameraRecognitionOffset(HeadNo, Offset);
	else
		gcPowerGantry->SetRearCameraRecognitionOffset(HeadNo, Offset);
}

Point_XY GetCameraRecognitionPosition(long Gantry, long HeadNo)
{
	Point_XY pt;
	if(Gantry == FRONT_GANTRY)
		pt = gcPowerGantry->GetCameraRecognitionPosition(HeadNo);
	else
		pt = gcPowerGantry->GetRearCameraRecognitionPosition(HeadNo);
	return pt;
}

long GetCameraHeadFromHeadNo(long HeadNo)
{
	long RetHeadNo = 0;
	RetHeadNo = gcPowerGantry->GetCameraHeadFromHeadNo(HeadNo);
	return RetHeadNo;
}

long GetCamera6HeadFromHeadNo(long HeadNo)
{
	long RetHeadNo = 0;
	RetHeadNo = gcPowerGantry->GetCamera6HeadFromHeadNo(HeadNo);
	return RetHeadNo;
}

bool IsCameraCenterPositionByHeadNo(long HeadNo)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsCameraCenterPositionByHeadNo(HeadNo);
	return bRet;
}

bool IsCamera6CenterPositionByHeadNo(long HeadNo)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsCamera6CenterPositionByHeadNo(HeadNo);
	return bRet;
}

long GetCameraNoByHead(long Gantry, long HeadNo)
{
	long CameraNo = MAXCAMNO;
	CameraNo = gcPowerGantry->GetCameraNoByHead(Gantry, HeadNo);
	return CameraNo;
}

long GetCamera6NoByHead(long Gantry, long HeadNo)
{
	long CameraNo = MAXCAMNO;
	CameraNo = gcPowerGantry->GetCamera6NoByHead(Gantry, HeadNo);
	return CameraNo;
}

long GetCameraChkPosByHead(long HeadNo)
{
	long ChkPos = 0;
	ChkPos = gcPowerGantry->GetCameraChkPosByHead(HeadNo);
	return ChkPos;
}

long GetCamera6ChkPosByHead(long HeadNo)
{
	long ChkPos = 0;
	ChkPos = gcPowerGantry->GetCamera6ChkPosByHead(HeadNo);
	return ChkPos;
}

Point_XY gReadGantryPosition(long Gantry)
{
	Point_XY pt;
	pt = gcPowerGantry->ReadGantryPosition(Gantry);
	return pt;
}

long WaitGantryRZIdle(long TimeOut)
{
	long Err = NO_ERR;
	long Gantry = FRONT_GANTRY;

	Err = WaitAllZIdle(Gantry, TimeOut);
	if (Err != NO_ERR)
	{
		return Err;
	}

	Err = WaitAllRIdle(Gantry, TimeOut);
	if (Err != NO_ERR)
	{
		return Err;
	}

	Err = WaitGantryIdle(Gantry, TimeOut);
	if (Err != NO_ERR)
	{
		return Err;
	}
	return Err;	
}

long WaitAllZIdle(long Gantry, long TimeOut)
{
	long Ret = 0;
	Ret = gcPowerGantry->WaitAllZIdle(Gantry, TimeOut);
	return Ret;
}

long WaitAllRIdle(long Gantry, long TimeOut)
{
	long Ret = 0;
	Ret = gcPowerGantry->WaitAllRIdle(Gantry, TimeOut);
	return Ret;
}

long WaitGantryIdle(long Gantry, long TimeOut)
{
	long Ret = 0;
	Ret = gcPowerGantry->WaitGantryIdle(Gantry, TimeOut);
	return Ret;
}

double ReadPosition(CString strAxis)
{
	double Position = 0.0;
	Position = gcPowerGantry->ReadOnePosition(strAxis);
	return Position;
}

double ReadPosition(long AxisNo)
{
	double Position = 0.0;
	Position = gcPowerGantry->ReadPosition(AxisNo);
	return Position;
}

void ReadAllPosition(long Gantry)
{
	gcPowerGantry->ReadAllPosition(Gantry);
}

double ReadCommandPosition(CString strAxis)
{
	double Position = 0.0;
	Position = gcPowerGantry->ReadCommandPosition(strAxis);
	return Position;
}

double ReadOneCommandVelocity(CString strAxis)
{
	double vel = 0.0;
	vel = gcPowerGantry->ReadOneCommandVelocity(strAxis);
	return vel;
}

double ReadProfileTargetPosition(CString strAxis)
{
	double Position = 0.0;
	Position = gcPowerGantry->ReadProfileTargetPosition(strAxis);
	return Position;
}

long WritePosition(CString strAxis, double Position)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->WriteOnePosition(strAxis, Position);
	return Err;
}

double ReadMotorActualPulse(CString strAxis)
{
	double ActualPulse = 0.0;
	ActualPulse = gcPowerGantry->ReadMotorActualPosition(strAxis);
	return ActualPulse;
}

double ReadActualTorque(CString strAxis)
{
	if (GetGlobalSimulationMode() == true && GetUse2StepZMotion() == 1)
	{
		return 30.1;
	}

	double Torque = 0.0;
	Torque = gcPowerGantry->ReadActualTorque(strAxis);
	return Torque;
}

double Read1DCompensationData(CString strAxis)
{
	double Torque = 0.0;
	Torque = gcPowerGantry->Read1DCompensationData(strAxis);
	return Torque;
}

double Read2DCompensationData(CString strAxis)
{
	double Torque = 0.0;
	Torque = gcPowerGantry->Read2DCompensationData(strAxis);
	return Torque;
}

double ReadVirtualPositiveLimit(CString strAxis)
{
	double pos = 0.0;
	pos = gcPowerGantry->ReadVirtualPositiveLimit(strAxis);
	return pos;
}

double ReadVirtualNegativeLimit(CString strAxis)
{
	double pos = 0.0;
	pos = gcPowerGantry->ReadVirtualNegativeLimit(strAxis);
	return pos;
}

bool IsNegativeLimitSwitchOn(CString strAxis)
{
	bool bMinus = false;
	bMinus = gcPowerGantry->IsNegativeLimitSwitchOn(strAxis);
	return bMinus;
}

bool IsPositiveLimitSwitchOn(CString strAxis)
{
	bool bPlus = false;
	bPlus = gcPowerGantry->IsPositiveLimitSwitchOn(strAxis);
	return bPlus;
}

bool IsOneAxisHomingComplete(CString strAxis)
{
	bool bHomeDone = false;
	bHomeDone = gcPowerGantry->IsOneAxisHomingComplete(strAxis);
	return bHomeDone;
}

long StartPosWaitMotion(CString strAxis, double Ratio, long TimeOut, double Pos, bool Wait)
{
	long Err = NO_ERR;
	double Inpos = 0.1;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitMotion Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	//if (strAxis.CompareNoCase(GetAxisX(FRONT_GANTRY)) == 0)
	//{
	//	TRACE(_T("[PWR] StartPosWaitMotion StartPositionX Pos:%.3f Ratio:%.1f\n"), Pos, Ratio);
	//	Err = StartPositionX(strAxis, Pos, Ratio);
	//}
	//else if (strAxis.CompareNoCase(GetAxisY1(FRONT_GANTRY)) == 0)
	//{
	//	TRACE(_T("[PWR] StartPosWaitMotion StartPositionY Pos:%.3f Ratio:%.1f\n"), Pos, Ratio);
	//	Err = StartPositionY(strAxis, Pos, Ratio);
	//}
	//else
	{
		Err = StartOnePosition(strAxis, Pos);
	}
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitMotion:%.3f\n"), strAxis, Pos);
		}
		if (Wait == true)
		{
			Err = WaitOneMotion(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartPosWaitMotionSkipLimitCheck(CString strAxis, double Ratio, long TimeOut, double Pos, bool Wait)
{
	long Err = NO_ERR;
	double Inpos = 0.1;

	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitMotion Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	//if (strAxis.CompareNoCase(GetAxisX(FRONT_GANTRY)) == 0)
	//{
	//	TRACE(_T("[PWR] StartPosWaitMotion StartPositionX Pos:%.3f Ratio:%.1f\n"), Pos, Ratio);
	//	Err = StartPositionX(strAxis, Pos, Ratio);
	//}
	//else if (strAxis.CompareNoCase(GetAxisY1(FRONT_GANTRY)) == 0)
	//{
	//	TRACE(_T("[PWR] StartPosWaitMotion StartPositionY Pos:%.3f Ratio:%.1f\n"), Pos, Ratio);
	//	Err = StartPositionY(strAxis, Pos, Ratio);
	//}
	//else
	{
		Err = StartOnePositionSkipLimitCheck(strAxis, Pos);
	}
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitMotion:%.3f\n"), strAxis, Pos);
		}
		if (Wait == true)
		{
			Err = WaitOneMotion(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long Start2StepPosWaitMotion(CString strAxis, double Ratio, double Ratio2nd, long TimeOut, double Pos, double Pos2nd, bool Wait)
{
	long Err = NO_ERR;
	double Inpos = 0.1;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s Start2StepPosWaitMotion Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOne2ndRatio(strAxis, Ratio2nd);
	Err = StartOne2StepPosition(strAxis, Pos, Pos2nd);
	if (Err == NO_ERR)
	{
		//if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s Start2StepPosWaitMotion,%.3f,%.3f Ratio,%.1f%%,%.1f%%\n"), strAxis, Pos, Pos2nd, Ratio, Ratio2nd);
		}
		if (Wait == true)
		{
			Err = WaitOneMotion(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartPosWaitInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, bool Wait)
{
	long Err = NO_ERR;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitInposition Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOnePosSet(strAxis, Inpos);
	Err = StartOnePosition(strAxis, Pos);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitInposition:%.3f Inpos:%.3f\n"), strAxis, Pos, Inpos);
		}
		if (Wait == true)
		{
			Err = WaitOnePosSet(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartPosWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitDelayedInposition Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOnePosition(strAxis, Pos);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitDelayedInposition:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Pos, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long Start2StepPosWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait, double Ratio2nd, double Pos2nd)
{
	long Err = NO_ERR;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitDelayedInposition Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOne2ndRatio(strAxis, Ratio2nd);
	SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOnePosition(strAxis, Pos);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitDelayedInposition:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Pos, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartPosWaitDelayedInpositionWihtoutSlave(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitDelayedInpositionWihtoutSlave Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOnePositionWithoutSlave(strAxis, Pos);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitDelayedInpositionWihtoutSlave:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Pos, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartPosWaitInposition(CString strAxis, double Ratio, long TimeOut, double Pos, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	Err = CheckLimitOver(strAxis, Pos);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartPosWaitInposition Command:%.3f Limit Error(%d)\n"), strAxis, Pos, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOneInPos(strAxis, Inpos);
	Err = StartOnePosition(strAxis, Pos);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartPosWaitInposition:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Pos, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneInPos(strAxis, Pos, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartMoveWaitMotion(CString strAxis, double Ratio, long TimeOut, double Offset, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis), Inpos = 0.1;
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartMoveWaitMotion Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	Err = StartOneMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartMoveWaitMotion:%.3f\n"), strAxis, Offset);
		}
		if (Wait == true)
		{
			Err = WaitOneMotion(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartMoveWaitInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis);
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartMoveWaitInposition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOnePosSet(strAxis, Inpos);
	Err = StartOneMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartMoveWaitInposition:%.3f Inpos:%.3f\n"), strAxis, Offset, Inpos);
		}
		if (Wait == true)
		{
			Err = WaitOnePosSet(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartMoveWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis);
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartMoveWaitDelayedInposition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOneMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartMoveWaitInposition:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Offset, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartMoveWaitDelayedInpositionWithoutSlave(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis);
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartMoveWaitDelayedInpositionWithoutSlave Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOneMoveWithoutSlave(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartMoveWaitDelayedInpositionWithoutSlave:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Offset, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartTeachMoveWaitMotion(CString strAxis, double Ratio, long TimeOut, double Offset, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis), Inpos = 0.1;
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartTeachMoveWaitMotion Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	Err = StartOneTeachMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartTeachMoveWaitMotion:%.3f\n"), strAxis, Offset);
		}
		if (Wait == true)
		{
			Err = WaitOneMotion(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartTeachMoveWaitInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis);
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartTeachMoveWaitInposition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	Err = SetOnePosSet(strAxis, Inpos);
	Err = StartOneTeachMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartTeachMoveWaitInposition:%.3f Inpos:%.3f\n"), strAxis, Offset, Inpos);
		}
		if (Wait == true)
		{
			Err = WaitOnePosSet(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

long StartTeachMoveWaitDelayedInposition(CString strAxis, double Ratio, long TimeOut, double Offset, double Inpos, long Time, bool Wait)
{
	long Err = NO_ERR;
	double Cur = ReadPosition(strAxis);
	double Cmd = Cur + Offset;
	Err = CheckLimitOver(strAxis, Cmd);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] %s StartTeachMoveWaitDelayedInposition Command:%.3f Limit Error(%d)\n"), strAxis, Cmd, Err);
		return Err;
	}
	SetOneRatio(strAxis, Ratio);
	Err = SetOneDelayedPosSet(strAxis, Inpos, Time);
	Err = StartOneTeachMove(strAxis, Offset);
	if (Err == NO_ERR)
	{
		if (gcPowerLog->IsShowMotionLog() == true)
		{
			TRACE(_T("[PWR] %s StartTeachMoveWaitDelayedInposition:%.3f Inpos:%.3f Delay:%d[ms]\n"), strAxis, Offset, Inpos, Time);
		}
		if (Wait == true)
		{
			Err = WaitOneDelayedPosSet(strAxis, Cmd, Inpos, TimeOut);
			InitOneRatio(strAxis);
		}
	}
	else
	{
		StopOne(strAxis);
		InitOneRatio(strAxis);
	}
	return Err;
}

INT_PTR AddLinearIntpAxis(long Gantry)
{
	INT_PTR Count;
	Count = gcPowerGantry->AddLinearIntpAxis(Gantry);
	return Count;
}

INT_PTR AddAllRAxis()
{
	INT_PTR Count;
	Count = gcPowerGantry->AddAllRIntpAxis();
	return Count;
}

INT_PTR AddSomeRAxis(SomeTarget Target)
{
	INT_PTR Count;
	Count = gcPowerGantry->AddSomeRIntpAxis(Target);
	return Count;
}

INT_PTR AddAllZAxis()
{
	INT_PTR Count;
	Count = gcPowerGantry->AddAllZIntpAxis();
	return Count;
}

INT_PTR AddSomeZAxis(SomeTarget Target)
{
	INT_PTR Count;
	Count = gcPowerGantry->AddSomeZIntpAxis(Target);
	return Count;
}

long SetMultiCommand(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiCommand(Gantry, Target);
	return Err;
}

long SetMultiCommand(long Gantry, long Target, double x, double y)
{
	long Err = NO_ERR;
	double X, Y;
	X = x;
	Y = y;
	Err = gcPowerGantry->SetMultiCommand(Gantry, Target, X, Y);
	return Err;
}

long SetMultiCommand(long Gantry, double pos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiCommand(Gantry, pos);
	return Err;
}

void InitMultiRatio()
{
	gcPowerGantry->InitMultiRatio();
}

void SetMultiRatio(SomeTarget Target)
{
	gcPowerGantry->SetMultiRatio(Target);
}

void SetMultiRatio(double Ratio)
{
	gcPowerGantry->SetMultiRatio(Ratio);
}

long StartMultiPosition(long Gantry, SomeTarget Target, bool Wait)
{
	long Err;
	Err = gcPowerGantry->StartMultiPosition(Target);
	if (Wait == true)
	{
		Err = WaitMultiMotion(Gantry, Target);
	}
	return Err;
}

long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, bool Wait)
{
	long Err;
	Err = gcPowerGantry->StartMultiPosition(Target);
	if (Wait == true)
	{
		Err = WaitMultiMotion(Gantry, TimeOut);
	}
	return Err;
}

long StartMultiPosition(long Gantry, Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal, long TimeOut, bool Wait)
{
	long Err;
	Err = gcPowerGantry->StartMultiPosition(Linear, StartCenter, EndCenter, Goal);
	if (Wait == true)
	{
		Err = WaitMultiMotion(Gantry, TimeOut);
	}
	return Err;
}

long SetMultiPosSet(SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiPosSet(Target);
	return Err;
}

long SetMultiPosSet(double Inpos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiPosSet(Inpos);
	return Err;
}

long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, bool Wait)
{
	long Err = NO_ERR;
	if (Wait == true)
	{
		SetMultiRatio(Ratio);
		SetMultiPosSet(Inpos);
	}
	Err = gcPowerGantry->StartMultiPosition(Target);
	if (Wait == true)
	{
		Err = WaitMultiPosSet(Gantry, TimeOut);
	}
	return Err;
}

long SetMultiDelayedPosSet(SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiDelayedPosSet(Target);
	return Err;
}

long SetMultiDelayedPosSet(double Inpos, long Ms)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiDelayedPosSet(Inpos, Ms);
	return Err;
}

long StartMultiPosition(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, long Ms, bool Wait)
{
	long Err = NO_ERR;
	if (Wait == true)
	{
		SetMultiRatio(Ratio);
		Err = SetMultiDelayedPosSet(Inpos, Ms);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPosition SetMultiDelayedPosSet Err:%d\n"), Err);
			return Err;
		}
	}
	if (Err == NO_ERR)
	{
		Err = gcPowerGantry->StartMultiPosition(Target);
	}
	if (Wait == true)
	{
		Err = WaitMultiDelayedPosSet(Gantry, TimeOut);
	}
	return Err;
}

long WaitMultiMotion(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiMotion(Target);
	return Err;
}

long WaitMultiMotion(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiMotion(TimeOut);
	return Err;
}

long WaitMultiPosSet(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiPosSet(Target);
	return Err;
}

long WaitMultiPosSet(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiPosSet(TimeOut);
	return Err;
}

long WaitMultiDelayedPosSet(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiDelayedPosSet(Target);
	return Err;
}

long WaitMultiDelayedPosSet(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiDelayedPosSet(TimeOut);
	return Err;
}

long SetMultiCommandR(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiCommandR(Gantry, Target);
	return Err;
}

long SetMultiCommandR(long Gantry, long Target, double x, double y)
{
	long Err = NO_ERR;
	double X, Y;
	X = x;
	Y = y;
	Err = gcPowerGantry->SetMultiCommandR(Gantry, Target, X, Y);
	return Err;
}

long SetMultiCommandR(long Gantry, double pos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiCommandR(Gantry, pos);
	return Err;
}

void InitMultiRatioR()
{
	gcPowerGantry->InitMultiRatioR();
}

void SetMultiRatioR(SomeTarget Target)
{
	gcPowerGantry->SetMultiRatioR(Target);
}

void SetMultiRatioR(double Ratio)
{
	gcPowerGantry->SetMultiRatioR(Ratio);
}

long StartMultiPositionR(long Gantry, SomeTarget Target, bool Wait)
{
	long Err;
	Err = gcPowerGantry->StartMultiPositionR(Target);
	if (Wait == true)
	{
		Err = WaitMultiMotionR(Gantry, Target);
	}
	return Err;
}

long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, bool Wait)
{
	long Err;
	Err = gcPowerGantry->StartMultiPositionR(Target);
	if (Wait == true)
	{
		Err = WaitMultiMotionR(Gantry, TimeOut);
	}
	return Err;
}

long SetMultiPosSetR(SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiPosSetR(Target);
	return Err;
}

long SetMultiPosSetR(double Inpos)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiPosSetR(Inpos);
	return Err;
}

long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, bool Wait)
{
	long Err = NO_ERR;
	if (Wait == true)
	{
		SetMultiRatioR(Ratio);
		SetMultiPosSetR(Inpos);
	}
	Err = gcPowerGantry->StartMultiPositionR(Target);
	if (Wait == true)
	{
		Err = WaitMultiPosSetR(Gantry, TimeOut);
	}
	return Err;
}

long SetMultiDelayedPosSetR(SomeTarget Target)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiDelayedPosSetR(Target);
	return Err;
}

long SetMultiDelayedPosSetR(double Inpos, long Ms)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SetMultiDelayedPosSetR(Inpos, Ms);
	return Err;
}

long StartMultiPositionR(long Gantry, long Target, double Ratio, long TimeOut, double Inpos, long Ms, bool Wait)
{
	long Err = NO_ERR;
	if (Wait == true)
	{
		SetMultiRatioR(Ratio);
		Err = SetMultiDelayedPosSetR(Inpos, Ms);
		if (Err != NO_ERR)
		{
			TRACE(_T("[PWR] StartMultiPosition SetMultiDelayedPosSet Err:%d\n"), Err);
			return Err;
		}
	}
	if (Err == NO_ERR)
	{
		Err = gcPowerGantry->StartMultiPositionR(Target);
	}
	if (Wait == true)
	{
		Err = WaitMultiDelayedPosSetR(Gantry, TimeOut);
	}
	return Err;
}

long WaitMultiMotionR(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiMotionR(Target);
	return Err;
}

long WaitMultiMotionR(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiMotionR(TimeOut);
	return Err;
}

long WaitMultiPosSetR(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiRPosSet(Target);
	return Err;
}

long WaitMultiPosSetR(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiRPosSet(TimeOut);
	return Err;
}


long WaitMultiDelayedPosSetR(long Gantry, SomeTarget Target)
{
	long Err;
	Err = gcPowerGantry->WaitMultiDelayedRPosSet(Target);
	return Err;
}


long WaitMultiDelayedPosSetR(long Gantry, long TimeOut)
{
	long Err;
	Err = gcPowerGantry->WaitMultiDelayedRPosSet(TimeOut);
	return Err;
}

void ReadMultiPosition()
{
	gcPowerGantry->ReadMultiPosition();
}

void ReadMultiPositionR()
{
	gcPowerGantry->ReadMultiPositionR();
}

void RemoveLinearIntpAxis()
{
	gcPowerGantry->RemoveLinearIntpAxis();
}

void RemoveLinearIntpAxisR()
{
	gcPowerGantry->RemoveLinearIntpAxisR();
}

long gMoveAllLastPosition()
{
	long Err = NO_ERR;
	INT_PTR index = 0;
	double PusherZ = 0.0, Ratio = 0.1, Inpos = 0.1;
	long TimeOut = TIME5000MS, Ms = TIME100MS;
	Cwmx3Axis* pAxis = NULL;
	for (index = 0; index < GetWmx3AxisCount(); ++index)
	{
		pAxis = g_pWmx3AxisArray.GetAt(index);
		if (pAxis->CheckServoOn() == true)
		{
			if (pAxis->IsPusherZAxis() == true)
			{
				PusherZ = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
				if (gcPowerLog->IsShowHomingLog() == true)
				{
					TRACE(_T("[PWR] Move lastest Pusher Z Pos:%.3f\n"), PusherZ);
				}
				Err = StartPosWaitDelayedInposition(_T("FPUZ"), Ratio, TimeOut, PusherZ, Inpos, Ms, true);
				if (Err != NO_ERR)
				{
					return false;
				}
			}
		}
	}
	//Err = StartAllZAxisWaitMotion(FRONT_GANTRY, GetStandByZ(FRONT_GANTRY), Ratio, TimeOut);
	Err = MoveZStandy(FRONT_GANTRY, GetStandByZ(FRONT_GANTRY), Ratio);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] DiscardOneAfterInserting MoveZStandy Err:%d\n"), Err);
		return Err;
	}
	return Err;
}

long gServoAllOn()
{
	long Err = NO_ERR;;

	SendPopupMessage(_T("Wait Servo On."));

	Err = gcPowerGantry->ServoAllOn();

	SendPopupClose();

	return Err;
}

long gServoOnWithAlarmClear(CString strAxis)
{
	long Err = NO_ERR;;
	Err = gcPowerGantry->ServoOnWithAlarmClear(strAxis);
	return Err;
}

long gServoAllOff()
{
	long Err = NO_ERR;;
	SendPopupMessage(_T("Wait Servo Off."));
	Err = gcPowerGantry->ServoAllOff();
	ThreadSleep(TIME200MS);
	SendPopupClose();
	return Err;
}

long gServoAllOffWithoutConv()
{
	long Err = NO_ERR;;
	Err = gcPowerGantry->ServoAllOffWithoutConvAxis();
	return Err;
}

long GetAllAxisHomingInComplete()
{
	long InCompleteCount = 0;
	InCompleteCount = gcPowerGantry->GetAllAxisHomingInComplete();
	return InCompleteCount;
}

bool IsAllAxisHomingComplete()
{
	bool bRet = false;
	bRet = gcPowerGantry->IsAllAxisHomingComplete();
	return bRet;
}

long GetHomingCompleteError()
{
	INT_PTR indx = 0;
	for (indx = 0; indx < GetWmx3AxisCount(); indx++)
	{
		if (IsOneAxisHomingComplete(indx) == false)
		{
			return HOMING_FAIL((long)indx);
		}
	}

	return NO_ERR;
}

bool IsAllZAxisHomingComplete()
{
	bool bRet = false;
	bRet = gcPowerGantry->IsAllZAxisHomingComplete();
	return bRet;
}

bool IsAllAxisHomingFail()
{
	bool bRet = false;
	bRet = gcPowerGantry->IsAllAxisHomingFail();
	return bRet;
}

bool IsOneAxisHomingFail(CString strAxis)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsOneAxisHomingFail(strAxis);
	return bRet;
}

bool IsAllRAxisHomingComplete()
{
	bool bRet = false;
	bRet = gcPowerGantry->IsAllRAxisHomingComplete();
	return bRet;
}

bool IsOneAxisHomingComplete(INT_PTR indx)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsOneAxisHomingComplete(indx);
	return bRet;
}

bool IsGantryAxis(CString strAxis)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsGantryAxis(strAxis);
	return bRet;
}

bool IsUseSlaveAxis(CString strAxis)
{
	bool bRet = false;
	bRet = gcPowerGantry->IsUseSlaveAxis(strAxis);
	return bRet;
}

bool SetSyncMasterSlave()
{
	bool bRet = false;
	bRet = gcPowerGantry->SetSyncMasterSlave();
	return bRet;
}

long GetZAxisCount()
{
	long ZCount = 0;
	ZCount = (long)gcPowerGantry->GetZAxisCount();
	return ZCount;
}

CString GetZAxisByIndex(INT_PTR indx)
{
	CString StrAxis;
	StrAxis = gcPowerGantry->GetZAxisByIndex(indx);
	return StrAxis;
}

long GetZAxisIndexByZName(CString strZAxis)
{
	long ZAxisNo = 0;
	ZAxisNo = gcPowerGantry->GetZAxisIndexByZName(strZAxis);
	return ZAxisNo;
}

long GetRAxisCount()
{
	long RCount = 0;
	RCount = (long)gcPowerGantry->GetRAxisCount();
	return RCount;
}

CString GetRAxisByIndex(INT_PTR indx)
{
	CString StrAxis;
	StrAxis = gcPowerGantry->GetRAxisByIndex(indx);
	return StrAxis;
}

long LinearIntplPosWaitMotion(long Gantry, long Target, Point_XY pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	if (AddLinearIntpAxis(Gantry) > 0)
	{
		Err = SetMultiCommand(Gantry, Target, pt.x, pt.y);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			Err = StartMultiPosition(Gantry, Target, Ratio, TimeOut, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiMotion(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("LinearIntplPosWaitMotion WaitMultiMotion"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("LinearIntplPosWaitMotion StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else
	{
		TRACE(_T("[PWR] LinearIntplPosWaitMotion AddLinearIntpAxis is Zero\n"));
		Err = EMPTY_XY;
		Err = SendAlarm(Err, _T("LinearIntplPosWaitMotion AddLinearIntpAxis"));
	}
	ThreadSleep(GetGlobalSettlingDelay());
	return Err;
}

long LinearIntplPosWaitPosSet(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	if (AddLinearIntpAxis(Gantry) > 0)
	{
		Err = SetMultiCommand(Gantry, Target, pt.x, pt.y);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			SetMultiPosSet(Inpos);
			Err = StartMultiPosition(Gantry, Target, Ratio, TimeOut, Inpos, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiPosSet(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("LinearIntplPosWaitPosSet WaitMultiPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("LinearIntplPosWaitPosSet StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else
	{
		TRACE(_T("[PWR] LinearIntplPosWaitPosSet AddLinearIntpAxis is Zero\n"));
		Err = EMPTY_XY;
		Err = SendAlarm(Err, _T("LinearIntplPosWaitPosSet AddLinearIntpAxis"));
	}
	ThreadSleep(GetGlobalSettlingDelay());
	return Err;
}

long LinearIntplPosWaitDelayedPosSet(long Gantry, long Target, Point_XY pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	if (AddLinearIntpAxis(Gantry) > 0)
	{
		Err = SetMultiCommand(Gantry, Target, pt.x, pt.y);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			SetMultiDelayedPosSet(Inpos, Ms);
			Err = StartMultiPosition(Gantry, Target, Ratio, TimeOut, Inpos, Ms, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiDelayedPosSet(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("LinearIntplPosWaitDelayedPosSet WaitMultiDelayedPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("LinearIntplPosWaitDelayedPosSet WaitMultiPosSet"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else
	{
		TRACE(_T("[PWR] LinearIntplPosWaitDelayedPosSet AddLinearIntpAxis is Zero\n"));
		Err = EMPTY_XY;
		Err = SendAlarm(Err, _T("LinearIntplPosWaitDelayedPosSet AddLinearIntpAxis"));
	}
	ThreadSleep(GetGlobalSettlingDelay());
	return Err;
}

long LinearPathIntplPos(long Gantry, long Target, Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	if (AddLinearIntpAxis(Gantry) > 0)
	{
		Err = SetMultiCommand(Gantry, Target, Linear.x, Linear.y);
		if (Err != NO_ERR) return Err;
		Err = SetMultiCommand(Gantry, Target, StartCenter.x, StartCenter.y);
		if (Err != NO_ERR) return Err;
		Err = SetMultiCommand(Gantry, Target, EndCenter.x, EndCenter.y);
		if (Err != NO_ERR) return Err;
		Err = SetMultiCommand(Gantry, Target, Goal.x, Goal.y);
		if (Err != NO_ERR) return Err;
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			Err = StartMultiPosition(Gantry, Linear, StartCenter, EndCenter, Goal, TimeOut, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiMotion(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("LinearPathIntplPos WaitMultiMotion"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("LinearPathIntplPos StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else
	{
		TRACE(_T("[PWR] LinearPathIntplPos AddLinearIntpAxis is Zero\n"));
		Err = EMPTY_XY;
		Err = SendAlarm(Err, _T("LinearPathIntplPos AddLinearIntpAxis"));
	}
	ThreadSleep(GetGlobalSettlingDelay());
	return Err;
}

long StartSomeRAxisWaitMotion(long Gantry, SomeTarget Target, bool Wait)
{
	long Err = NO_ERR;
	if (AddSomeRAxis(Target) > 0)
	{
		Err = SetMultiCommandR(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Target);
			Err = StartMultiPositionR(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				if (Wait == true)
				{
					Err = WaitMultiMotionR(Gantry, Target);
					if (Err != NO_ERR)
					{
						Err = SendAlarm(Err, _T("StartSomeRAxisWaitMotion WaitMultiMotionR"));
					}
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeRAxisWaitMotion StartMultiPositionR"));
			}
			if (Wait == true)
			{
				InitMultiRatioR();
			}
		}
		else
		{
			Err = SendAlarm(Err, _T("StartSomeRAxisWaitMotion SetMultiCommandR"));
		}
		if (Wait == true)
		{
			RemoveLinearIntpAxisR();
		}
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeRAxisWaitMotion AddSomeRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartSomeRAxisWaitMotion AddSomeRAxis"));
	}
	return Err;
}

long StartSomeRAxisWaitPosSet(long Gantry, SomeTarget Target, bool Wait)
{
	long Err = NO_ERR;
	if (AddSomeRAxis(Target) > 0)
	{
		Err = SetMultiCommandR(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Target);
			SetMultiPosSetR(Target);
			Err = StartMultiPositionR(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				if (Wait == true)
				{
					Err = WaitMultiPosSetR(Gantry, Target);
					if (Err != NO_ERR)
					{
						Err = SendAlarm(Err, _T("StartSomeRAxisWaitPosSet WaitMultiPosSetR"));
					}
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeRAxisWaitPosSet StartMultiPositionR"));
			}
			if (Wait == true)
			{
				InitMultiRatioR();
			}
		}
		else
		{
			Err = SendAlarm(Err, _T("StartSomeRAxisWaitPosSet SetMultiCommandR"));
		}
		if (Wait == true)
		{
			RemoveLinearIntpAxisR();
		}
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeRAxisWaitPosSet AddSomeRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartSomeRAxisWaitPosSet AddSomeRAxis"));
	}
	return Err;
}

long StartSomeRAxisWaitDelayedPosSet(long Gantry, SomeTarget Target, bool Wait)
{
	long Err = NO_ERR;
	if (AddSomeRAxis(Target) > 0)
	{
		Err = SetMultiCommandR(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Target);
			SetMultiDelayedPosSetR(Target);
			Err = StartMultiPositionR(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				if (Wait == true)
				{
					Err = WaitMultiDelayedPosSetR(Gantry, Target);
					if (Err != NO_ERR)
					{
						Err = SendAlarm(Err, _T("StartSomeRAxisWaitDelayedPosSet WaitMultiDelayedPosSetR"));
					}
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeRAxisWaitDelayedPosSet StartMultiPositionR"));
			}
			if (Wait == true)
			{
				InitMultiRatioR();
			}
		}
		else
		{
			Err = SendAlarm(Err, _T("StartSomeRAxisWaitDelayedPosSet SetMultiCommandR"));
		}
		if (Wait == true)
		{
			RemoveLinearIntpAxisR();
		}
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeRAxisWaitDelayedPosSet AddSomeRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartSomeRAxisWaitDelayedPosSet AddSomeRAxis"));
	}
	return Err;
}

long StartAllRAxisWaitMotion(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllRAxis() > 0)
	{
		Err = SetMultiCommandR(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Ratio);
			Err = StartMultiPositionR(Gantry, TBL_CAMERA, Ratio, TimeOut, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiMotionR(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllRAxisWaitMotion WaitMultiMotionR"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllRAxisWaitMotion StartMultiPositionR"));
			}
			InitMultiRatioR();
		}
		else
		{
			Err = SendAlarm(Err, _T("StartAllRAxisWaitMotion SetMultiCommandR"));
		}
		RemoveLinearIntpAxisR();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllRAxisWaitMotion AddAllRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartAllRAxisWaitMotion AddAllRAxis"));
	}
	return Err;
}

long StartAllRAxisWaitPosSet(long Gantry, double pt, double Ratio, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllRAxis() > 0)
	{
		Err = SetMultiCommandR(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Ratio);
			SetMultiPosSetR(Inpos);
			Err = StartMultiPositionR(Gantry, TBL_CAMERA, Ratio, TimeOut, Inpos, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiPosSetR(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllRAxisWaitPosSet WaitMultiPosSetR"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllRAxisWaitPosSet StartMultiPositionR"));
			}
			InitMultiRatioR();
		}
		else
		{
			Err = SendAlarm(Err, _T("StartAllRAxisWaitPosSet SetMultiCommandR"));
		}
		RemoveLinearIntpAxisR();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllRAxisWaitPosSet AddAllRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartAllRAxisWaitPosSet AddAllRAxis"));
	}
	return Err;
}

long StartAllRAxisWaitDelayedPosSet(long Gantry, double pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllRAxis() > 0)
	{
		Err = SetMultiCommandR(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatioR(Ratio);
			SetMultiDelayedPosSetR(Inpos, Ms);
			Err = StartMultiPositionR(Gantry, TBL_CAMERA, Ratio, TimeOut, Inpos, Ms, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiDelayedPosSetR(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllRAxisWaitDelayedPosSet WaitMultiDelayedPosSetR"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllRAxisWaitDelayedPosSet StartMultiPositionR"));
			}
			InitMultiRatioR();
		}
		else
		{
			Err = SendAlarm(Err, _T("StartAllRAxisWaitDelayedPosSet SetMultiCommandR"));
		}
		RemoveLinearIntpAxisR();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllRAxisWaitDelayedPosSet AddAllRAxis is Zero\n"));
		Err = EMPTY_ALLR;
		Err = SendAlarm(Err, _T("StartAllRAxisWaitDelayedPosSet AddAllRAxis"));
	}
	return Err;
}

long StartSomeZAxisWaitMotion(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	if (AddSomeZAxis(Target) > 0)
	{
		Err = SetMultiCommand(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Target);
			Err = StartMultiPosition(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiMotion(Gantry, Target);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartSomeZAxisWaitMotion WaitMultiMotion"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeZAxisWaitMotion StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeZAxisWaitMotion AddSomeZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartSomeZAxisWaitMotion AddSomeZAxis"));
	}
	return Err;
}

long StartSomeZAxisWaitPosSet(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	if (AddSomeZAxis(Target) > 0)
	{
		Err = SetMultiCommand(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Target);
			SetMultiPosSet(Target);
			Err = StartMultiPosition(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiPosSet(Gantry, Target);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartSomeZAxisWaitPosSet WaitMultiPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeZAxisWaitPosSet StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeZAxisWaitPosSet AddSomeZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartSomeZAxisWaitPosSet AddSomeZAxis"));
	}
	return Err;
}

long StartSomeZAxisWaitDelayedPosSet(long Gantry, SomeTarget Target)
{
	long Err = NO_ERR;
	if (AddSomeZAxis(Target) > 0)
	{
		Err = SetMultiCommand(Gantry, Target);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Target);
			SetMultiDelayedPosSet(Target);
			Err = StartMultiPosition(Gantry, Target, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiDelayedPosSet(Gantry, Target);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartSomeZAxisWaitDelayedPosSet WaitMultiDelayedPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartSomeZAxisWaitDelayedPosSet StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartSomeZAxisWaitDelayedPosSet AddSomeZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartSomeZAxisWaitDelayedPosSet AddSomeZAxis"));
	}
	return Err;
}

long StartAllZAxisWaitMotion(long Gantry, double pt, double Ratio, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllZAxis() > 0)
	{
		Err = SetMultiCommand(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			Err = StartMultiPosition(Gantry, TBL_CAMERA, Ratio, TimeOut, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiMotion(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllZAxisWaitMotion WaitMultiMotion"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllZAxisWaitMotion StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllZAxisWaitMotion AddAllZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartAllZAxisWaitMotion AddAllZAxis"));
	}
	return Err;
}

long StartAllZAxisWaitPosSet(long Gantry, double pt, double Ratio, double Inpos, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllZAxis() > 0)
	{
		Err = SetMultiCommand(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			SetMultiPosSet(Inpos);
			Err = StartMultiPosition(Gantry, TBL_CAMERA, Ratio, TimeOut, Inpos, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiPosSet(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllZAxisWaitPosSet WaitMultiPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllZAxisWaitPosSet StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllZAxisWaitPosSet AddAllZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartAllZAxisWaitPosSet AddAllZAxis"));
	}
	return Err;
}

long StartAllZAxisWaitDelayedPosSet(long Gantry, double pt, double Ratio, double Inpos, long Ms, long TimeOut)
{
	long Err = NO_ERR;
	if (AddAllZAxis() > 0)
	{
		Err = SetMultiCommand(Gantry, pt);
		if (Err == NO_ERR)
		{
			SetMultiRatio(Ratio);
			SetMultiDelayedPosSet(Inpos, Ms);
			Err = StartMultiPosition(Gantry, TBL_CAMERA, Ratio, TimeOut, Inpos, Ms, false);
			if (Err == NO_ERR)
			{
				Err = WaitMultiDelayedPosSet(Gantry, TimeOut);
				if (Err != NO_ERR)
				{
					Err = SendAlarm(Err, _T("StartAllZAxisWaitDelayedPosSet WaitMultiDelayedPosSet"));
				}
			}
			else
			{
				Err = SendAlarm(Err, _T("StartAllZAxisWaitDelayedPosSet StartMultiPosition"));
			}
			InitMultiRatio();
		}
		RemoveLinearIntpAxis();
	}
	else if (GetHeadUseCount(Gantry) > 0)
	{
		TRACE(_T("[PWR] StartAllZAxisWaitDelayedPosSet AddAllZAxis is Zero\n"));
		Err = EMPTY_ALLZ;
		Err = SendAlarm(Err, _T("StartAllZAxisWaitDelayedPosSet AddAllZAxis"));
	}
	return Err;
}


long PrepareMoveConvWidth()
{
	bool bEntry = IsExistAll(ENTRY_CONV);
	bool bWork = IsExistAll(WORK1_CONV);
	bool bExit = IsExistAll(EXIT_CONV);
	double Ratio = 0.5, Inpos = 0.1;
	long TimeOut = TIME5000MS, Ms = TIME30MS, Err = NO_ERR;

	if (bEntry == true || bWork == true || bExit == true)
	{
		TRACE(_T("[PWR] PrepareMoveConvWidth Pcb Exist Entry:%d Work:%d Exit:%d\n"), bEntry, bWork, bExit);
		return CONVEYOR_CANNOT_MOVE_EXIST_PCB(FRONT_CONV);
	}

	CString strZ = GetPusherZName(FRONT_CONV);
	if (ReadPosition(strZ) < GetPusherByZ(FRONT_CONV) + 10.0)
	{
		Err = StartPosWaitDelayedInposition(strZ, Ratio, TimeOut, GetPusherByZ(FRONT_CONV) + 10.0, Inpos, Ms, true);
		if (Err != NO_ERR)
		{
			return Err;
		}
	}

	return NO_ERR;
}

long MoveConvWidth(long Conv, double Width)
{
	long Err = NO_ERR;
	double Ratio = 0.5, Inpos = 0.1;
	long TimeOut = TIME20000MS, Ms = TIME100MS;

	Err = PrepareMoveConvWidth();
	if (Err != NO_ERR)
	{
		Err = SendAlarm(Err, _T("Prepare conveyor move Error"));
		return Err;
	}

	Err = StartPosWaitDelayedInposition(GetConvName(Conv), Ratio, TimeOut, Width, Inpos, Ms, true);
	if (Err == NO_ERR)
	{
		WriteWidth(FRONT_CONV, WORK1_CONV, Width);
	}
	else
	{
		StopOne(GetConvName(Conv));
		ThreadSleep(TIME500MS);
		double WidthCurrent = ReadPosition(GetConvName(Conv));
		WriteWidth(Conv, WORK1_CONV, WidthCurrent);

		Err = SendAlarm(Err, _T("Conveyor move Error"));
	}

	return Err;
}

bool CheckServoOn(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->CheckServoOn(strAxis);
	}
	return bRet;	
}

long GetServoOnError(CString strAxis)
{
	long Err = NO_ERR;
	if (gcPowerGantry != NULL)
	{
		Err = gcPowerGantry->GetServoOnError(strAxis);
	}
	return Err;
}


long GetAxisID(CString strAxis)
{
	long axisID = 0;
	axisID = gcPowerGantry->GetAxisID(strAxis);
	return axisID;
}

bool CheckSlaveServoOn(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->CheckSlaveServoOn(strAxis);
	}
	return bRet;
}

long GetSlaveAxisIndex(CString strAxis)
{
	long slave = 0;
	slave = gcPowerGantry->GetSlaveAxisIndex(strAxis);
	return slave;
}

long GetSlaveAxisSlaveID(CString strAxis)
{
	long slaveID = 0;
	slaveID = gcPowerGantry->GetSlaveAxisSlaveID(strAxis);
	return slaveID;
}

bool CheckAmpAlarm(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->CheckAmpAlarm(strAxis);
	}
	return bRet;
}

bool CheckSlaveAmpAlarm(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->CheckSlaveAmpAlarm(strAxis);
	}
	return bRet;
}

bool AlarmClear(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->AlarmClear(strAxis);
	}
	return bRet;
}

bool ClearSlaveAmpAlarm(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->ClearSlaveAmpAlarm(strAxis);
	}
	return bRet;
}

bool ServoOn(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->ServoOn(strAxis);
	}
	return bRet;
}

bool ServoOff(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->ServoOff(strAxis);
	}
	return bRet;
}

bool SlaveServoOn(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->SlaveServoOn(strAxis);
	}
	return bRet;
}

bool SlaveServoOff(CString strAxis)
{
	bool bRet = false;
	if (gcPowerGantry != NULL)
	{
		bRet = gcPowerGantry->SlaveServoOff(strAxis);
	}
	return bRet;
}

long CheckLimitOver(CString strAxis, double Command)
{
	long Err = NO_ERR;
	if (gcPowerGantry != NULL)
	{
		Err = gcPowerGantry->CheckLimitOver(strAxis, Command);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("Over Limit"));
		}
	}
	return Err;
}

long GetSuctionIONo(long Gantry, long HeadNo)
{
	long SuctionIONo = IO_NOUSE;
	SuctionIONo = gcPowerGantry->GetSuctionIONo(Gantry, HeadNo);
	return SuctionIONo;
}

long GetBlowIONo(long Gantry, long HeadNo)
{
	long BlowIONo = IO_NOUSE;
	BlowIONo = gcPowerGantry->GetBlowIONo(Gantry, HeadNo);
	return BlowIONo;
}

long CheckAirPressureLow()
{
	long Err = NO_ERR;
	Err = gcPowerGantry->CheckAirPressureLow();
	return Err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

UBYTE ReadOutputOne(long no)
{
	UBYTE ucStatus = OUTOFF;
	if (gcPowerIO != NULL)
	{
		ucStatus = gcPowerIO->Getdo1(no);
	}
	return ucStatus;
}

bool ReadOutputTimeOne(long port, long onoff, long dwTime)
{
	bool bRet = false;
	if (gcPowerIO != NULL)
	{
		bRet = gcPowerIO->Getdot1(port, onoff, dwTime);
	}
	return bRet;
}

void OutputOne(long no, long onoff)
{
	if (gcPowerIO != NULL)
	{
		gcPowerIO->do1(no, onoff);
	}
}

UBYTE InputOne(long no)
{
	UBYTE ucStatus = INOFF;
	if (gcPowerIO != NULL)
	{
		ucStatus = gcPowerIO->di1(no);
	}
	return ucStatus;
}

long GetAnalogOne(long no)
{
	long level = IO_NOUSE;
	if (gcPowerIO != NULL)
	{
		level = gcPowerIO->di1(no);
	}
	return level;
}

bool InputTimeOne(long port, long onoff, long dwTime)
{
	bool bRet = false;
	if (gcPowerIO != NULL)
	{
		bRet = gcPowerIO->dit1(port, onoff, dwTime);
	}
	return bRet;
}

bool InputElapsedTimeOne(long port, long onoff, ULONGLONG dwTime)
{
	bool bRet = false;
	if (gcPowerIO != NULL)
	{
		if (gcPowerIO->di1ElapsedTime(port, onoff) > dwTime)
		{
			bRet = true;
		}
		else
		{
			bRet = false;
		}
	}

	return bRet;
}

bool InputElapsedTimeWait(long port, long onoff, ULONGLONG dwTime, ULONGLONG timeOut)
{
	if (GetGlobalSimulationMode() == true)
	{
		return true;
	}

	ULONGLONG time = _time_get();
	while (true)
	{
		if (InputElapsedTimeOne(port, onoff, TIME500MS) == true)
		{
			return true;
		}

		if (_time_elapsed(time) > timeOut)
		{
			return false;
		}

		ThreadSleep(TIME1MS);
	}

	return true;
}

bool OutputElapsedTimeOne(long port, long onoff, ULONGLONG dwTime)
{
	bool bRet = false;
	if (gcPowerIO != NULL)
	{
		if (gcPowerIO->do1ElapsedTime(port, onoff) > dwTime)
		{
			bRet = true;
		}
		else
		{
			bRet = false;
		}
	}

	return bRet;
}


double GetTemperature(CString strAxis)
{
	double Temperature = 25.0;
	Temperature = gcPowerIO->GetTemperature(strAxis);
	return Temperature;
}

double GetHeight(long Gantry)
{
	double Height = 0.0;
	Height = gcPowerIO->GetHeight(Gantry);
	return Height;
}

long GetAnalogLevel(long Gantry, long HeadNo)
{
	long Level = 0;
	Level = gcPowerIO->GetAnalogLevel(Gantry, HeadNo);
	return Level;
}

long SuctionOne(long Gantry, long HeadNo, bool bSuction)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SuctionOne(Gantry, HeadNo, bSuction);
	return Err;
}

long BlowOne(long Gantry, long HeadNo, bool bBlow)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->BlowOne(Gantry, HeadNo, bBlow);
	return Err;
}

long SuctionAll(long Gantry, bool bSuction)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->SuctionAll(Gantry, bSuction);
	return Err;
}

long BlowAll(long Gantry, bool bBlow)
{
	long Err = NO_ERR;
	Err = gcPowerGantry->BlowAll(Gantry, bBlow);
	return Err;
}

bool GetOneSuction(long Gantry, long HeadNo)
{
	bool bSuction = gcPowerGantry->GetOneSuction(Gantry, HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetOneSuction Gantry(%d) HeadNo(%d) %s\n"), Gantry, HeadNo, bSuction == true ? _T("On") : _T("Off"));
	}
	return bSuction;
}

bool GetAllSuction(long Gantry)
{
	bool bSuction = gcPowerGantry->GetAllSuction(Gantry);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetAllSuction Gantry(%d) %s\n"), Gantry, bSuction == true ? _T("On") : _T("Off"));
	}
	return bSuction;
}

bool GetOneBlow(long Gantry, long HeadNo)
{
	bool bBlow = gcPowerGantry->GetOneBlow(Gantry, HeadNo);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetOneBlow Gantry(%d) HeadNo(%d) %s\n"), Gantry, HeadNo, bBlow == true ? _T("On") : _T("Off"));
	}
	return bBlow;
}

bool GetAllBlow(long Gantry)
{
	bool bBlow = gcPowerGantry->GetAllBlow(Gantry);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] GetAllBlow Gantry(%d) %s\n"), Gantry, bBlow == true ? _T("On") : _T("Off"));
	}
	return bBlow;
}

long GetReadyIONoFromReadyNo(long ReadyNo)
{
	long ReadyIONo = IO_NOUSE;
	ReadyIONo = gcPowerIO->GetReadyIONoFromReadyNo(ReadyNo);
	if (gcPowerLog->IsShowFeederLog() == true)
	{
		TRACE(_T("[PWR] GetReadyIONoFromReadyNo No(%d) IO(%d)\n"), ReadyNo, ReadyIONo);
	}
	return ReadyIONo;
}

long GetReleaseIONoFromReleaseNo(long ReleaseNo)
{
	long ReleaseIONo = IO_NOUSE;
	ReleaseIONo = gcPowerIO->GetReleaseIONoFromReleaseNo(ReleaseNo);
	if (gcPowerLog->IsShowFeederLog() == true)
	{
		TRACE(_T("[PWR] GetReleaseIONoFromReleaseNo No(%d) IO(%d)\n"), ReleaseNo, ReleaseIONo);
	}
	return ReleaseIONo;
}

long GetIOAddr(long IONum)
{
	long Address = gcPowerIO->GetIOAddr(IONum);
	return Address;
}

char GetIOBit(long IONum)
{
	char IOBit = gcPowerIO->GetIOBit(IONum);
	return IOBit;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

bool WriteHomePosition(CString strAxis, double HomePosition)
{
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		return false;
	}
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteHomePosition(strAxis, HomePosition);
		return true;
	}
	else
	{
		return false;
	}
}

double ReadHomePosition(CString strAxis)
{
	double HomePosition = 0.0;
	if (strAxis.CompareNoCase(_T("NON")) == 0)
	{
		return HomePosition;
	}
	if (gcPowerGantry != NULL)
	{
		HomePosition = gcPowerGantry->ReadHomePosition(strAxis);
	}
	return HomePosition;
}

bool WriteCameraAlignPosition(long Gantry, Point_XY pt)
{
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteCameraAlignPosition(Gantry, pt);
	}
	return true;
}

Point_XY ReadCameraAlignPosition(long Gantry)
{
	Point_XY PcbAlign;
	ZeroMemory(&PcbAlign, sizeof(PcbAlign));
	if (gcPowerGantry != NULL)
	{
		PcbAlign = gcPowerGantry->ReadCameraAlignPosition(Gantry);
	}
	return PcbAlign;
}

bool WritePcbFixPosition(long Conveyor, Point_XY pt)
{
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WritePcbFixPosition(Conveyor, pt);
	}
	return true;
}

Point_XY ReadPcbFixPosition(long Conveyor)
{
	Point_XY PcbFix;
	ZeroMemory(&PcbFix, sizeof(PcbFix));
	if (gcPowerGantry != NULL)
	{
		PcbFix = gcPowerGantry->ReadPcbFixPosition(Conveyor);
	}
	return PcbFix;
}

void WriteReferenceFeederPosition(long Stage, long RefFdNo, Point_XY RefFdPos)
{
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteReferenceFeederPosition(Stage, RefFdNo, RefFdPos);
	}
}

void WriteFeederPitch(long Stage, double Pitch)
{
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteFeederPitch(Stage, Pitch);
	}
}

Point_XY ReadReferenceFeederPosition(long Stage)
{
	Point_XY RefFdPos;
	ZeroMemory(&RefFdPos, sizeof(RefFdPos));
	if (gcPowerGantry != NULL)
	{
		RefFdPos = gcPowerGantry->ReadReferenceFeederPosition(Stage);
	}
	return RefFdPos;
}

long ReadReferenceFeederNo(long Stage)
{
	long RefFdNo = 0;
	if (gcPowerGantry != NULL)
	{
		RefFdNo = gcPowerGantry->ReadReferenceFeederNo(Stage);
	}
	return RefFdNo;
}

double ReadFeederPitch(long Stage)
{
	double Pitch = 0.0;
	if (gcPowerGantry != NULL)
	{
		Pitch = gcPowerGantry->ReadFeederPitch(Stage);
	}
	return Pitch;
}

Point_XY ReadDistOriginFromPcbFix(Point_XY Origin)
{
	Point_XY Dist;
	ZeroMemory(&Dist, sizeof(Dist));
	if (gcPowerGantry != NULL)
	{
		Dist = gcPowerGantry->ReadOriginDist(Origin);
	}
	return Dist;
}

Point_XY ReadDistMarkFromOrigin(Point_XY MarkPt, Point_XY Origin)
{
	Point_XY Dist;
	ZeroMemory(&Dist, sizeof(Dist));
	if (gcPowerGantry != NULL)
	{
		Dist = gcPowerGantry->ReadMarkDist(MarkPt, Origin);
	}
	return Dist;
}

Point_XY ReadDistInsertFromOrigin(Point_XY Insert, Point_XY Origin)
{
	Point_XY Dist;
	ZeroMemory(&Dist, sizeof(Dist));
	if (gcPowerGantry != NULL)
	{
		Dist = gcPowerGantry->ReadInsertDist(Insert, Origin);
	}
	return Dist;
}

Point_XY ReadOriginFromPcbFix(Point_XY Origin)
{
	Point_XY Goal;
	ZeroMemory(&Goal, sizeof(Goal));
	if (gcPowerGantry != NULL)
	{
		Goal = gcPowerGantry->ReadOriginTarget(Origin);
	}
	return Goal;
}

Point_XY ReadMarkFromOrigin(Point_XY MarkPt, Point_XY Origin, const bool& isSkipReadBlockTarget)
{
	Point_XY Goal;
	ZeroMemory(&Goal, sizeof(Goal));
	if (gcPowerGantry != NULL)
	{
        Goal = gcPowerGantry->ReadMarkTarget(MarkPt, Origin, isSkipReadBlockTarget);
	}
	return Goal;
}

Point_XY ReadInsertFromOrigin(Point_XY Insert, Point_XY Origin)
{
	Point_XY Goal;
	ZeroMemory(&Goal, sizeof(Goal));
	if (gcPowerGantry != NULL)
	{
		Goal = gcPowerGantry->ReadInsertTarget(Insert, Origin);
	}
	return Goal;
}


bool ReadBlockMarkFromOrigin(long BlockNo, Point_XY* Mark1Pt, Point_XY* Mark2Pt)
{
	FIDUCIAL FiducialMark = gcReadJobFile->GetMark();
	ORIGIN Origin = gcReadJobFile->GetOrigin();
	ORIGIN BlockOrigin = gcReadJobFile->GetBlockOrigin(BlockNo);
	Point_XY OriginXY;

	TRACE(_T("[PWR] ReadBlockMarkFromOrigin Blk:%d Origin,%.3f,%.3f BlkOrigin,%.3f,%.3f\n"), BlockNo, Origin.pt.x, Origin.pt.y, BlockOrigin.pt.x, BlockOrigin.pt.y);

	OriginXY.x = Origin.pt.x + BlockOrigin.pt.x;
	OriginXY.y = Origin.pt.y + BlockOrigin.pt.y;

	*Mark1Pt = ReadMarkFromOrigin(FiducialMark.pt[0], OriginXY);
	*Mark2Pt = ReadMarkFromOrigin(FiducialMark.pt[1], OriginXY);

	return true;
}

Point_XYR GetInsertXYFromJobfile(long BlockNo, long InsertNo)
{
	Point_XYR Result;
	PCB Pcb = gcReadJobFile->GetPcb();
	Point_XY OriginXY;
	ORIGIN Origin = gcReadJobFile->GetOrigin();
	ORIGIN BlockOrigin;
	Point_XYRZ PtInsert = gcReadJobFile->GetInsert(InsertNo).pt;

	if (Pcb.UseBlockType == PCB_SINGLE)
	{
		BlockOrigin.pt.x = BlockOrigin.pt.y = BlockOrigin.pt.r = 0.000;
	}
	else
	{
		BlockOrigin = gcReadJobFile->GetBlockOrigin(BlockNo);
	}

	OriginXY.x = Origin.pt.x + BlockOrigin.pt.x;
	OriginXY.y = Origin.pt.y + BlockOrigin.pt.y;

	Result.x = OriginXY.x + PtInsert.x;
	Result.y = OriginXY.y + PtInsert.y;
	Result.r = 0.000;

	return Result;
}

Point_XYR GetOriginXYRFromJobfile(long BlockNo)
{
	Point_XYR OriginXYR;
	ORIGIN Origin = gcReadJobFile->GetOrigin();
	PCB Pcb = gcReadJobFile->GetPcb();
	ORIGIN BlockOrigin;

	if (Pcb.UseBlockType == PCB_SINGLE)
	{
		BlockOrigin.pt.x = BlockOrigin.pt.y = BlockOrigin.pt.r = 0.000;
	}
	else
	{
		BlockOrigin = gcReadJobFile->GetBlockOrigin(BlockNo);
	}

	OriginXYR.x = Origin.pt.x + BlockOrigin.pt.x;
	OriginXYR.y = Origin.pt.y + BlockOrigin.pt.y;
	OriginXYR.r = BlockOrigin.pt.r;

	return OriginXYR;
}

void WriteConfirmInsertBeforePosition(Point_XY Before)
{
	TRACE(_T("[PWR] WriteConfirmInsertBeforePosition Pt.XY,%.3f,%.3f\n"), Before.x, Before.y);
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteConfirmInsertBeforePosition(Before);
	}
}

void WriteConfirmInsertBeforePosition(const long& gantry, const Point_XY& Before)
{
    if (gantry != FRONT_GANTRY)
    {
        TRACE_FILE_FUNC_LINE_"bad request. (gantry != FRONT_GANTRY)");
        return;
    }
    WriteConfirmInsertBeforePosition(Before);
}

Point_XY ReadConfirmInsertBeforePosition()
{
	Point_XY Before;
	ZeroMemory(&Before, sizeof(Before));
	if (gcPowerGantry != NULL)
	{
		Before = gcPowerGantry->ReadConfirmInsertBeforePosition();
	}
	return Before;
}


void WriteConfirmMeasureHeightBeforePosition(Point_XYT Before)
{
	TRACE(_T("[PWR] WriteConfirmInsertBeforePosition Pt.XY,%.3f,%.3f,%.3f\n"), Before.x, Before.y,Before.t);
	if (gcPowerGantry != NULL)
	{
		gcPowerGantry->WriteConfirmMeasureHeightBeforePosition(Before);
	}
}

Point_XYT ReadConfirmMeasureHeightBeforePosition()
{
	Point_XYT Before;
	ZeroMemory(&Before, sizeof(Before));
	if (gcPowerGantry != NULL)
	{
		Before = gcPowerGantry->ReadConfirmMeasureHeightBeforePosition();
	}
	return Before;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
void WritePusherZ(long Conveyor, long Type, double Position)
{
	if (gcPowerConveyorData != NULL)
	{
		gcPowerConveyorData->SetPusherZ(Conveyor, Type, Position);
		gcPowerConveyorData->WritePusherZ(Conveyor);
	}
}

void WriteWidth(long Conveyor, long Type, double Position)
{
	if (gcPowerConveyorData != NULL)
	{
		gcPowerConveyorData->SetWidth(Conveyor, Type, Position);
		gcPowerConveyorData->WriteWidth(Conveyor);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long gDisable1DCompensation()
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Disable1DCompensation();
	return err;
}

long gEnable1DCompensation()
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Enable1DCompensation();
	return err;
}

long gSet1DCompensation()
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Set1DCompensation();
	return err;
}

void gShow1DData()
{
	for (unsigned indx = CAL_1D_INIT; indx < CAL_1D_MAXCOUNT; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d New Compensation %.3f\n"), indx, gcPowerCalibrationData->Get1DCompensationData(FRONT_GANTRY, indx));
	}
}

void gShow1DData(unsigned nCompenMax)
{
	for (unsigned indx = CAL_1D_INIT; indx < nCompenMax; ++indx)
	{
		TRACE(_T("[PWR] 1D indx:%d New Compensation %.3f\n"), indx, gcPowerCalibrationData->Get1DCompensationData(FRONT_GANTRY, indx));
	}
}

long oneDCompensationOn()
{
	long err = ErrorCode::None;
	err = gDisable1DCompensation();
	ThreadSleep(TIME100MS);
	err = gSet1DCompensation();
	ThreadSleep(TIME100MS);
	err = gEnable1DCompensation();
	ThreadSleep(TIME500MS);
	return err;
}

long oneDCompensationOff()
{
	long err = ErrorCode::None;
	err = gDisable1DCompensation();
	ThreadSleep(TIME100MS);
	return err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long gDisable2DCompensation(long Channel)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Disable2DCompensation(Channel);
	return err;
}

long gEnable2DCompensation(long Channel)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Enable2DCompensation(Channel);
	return err;
}

long gSet2DCompensation_ForXY(long Gantry)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Set2DCompensation(Gantry);
	return err;
}


long gSet2DCompensation_ForXY(long Gantry, long MarkXCount, long MarkYCount)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->Set2DCompensation(Gantry, MarkXCount, MarkYCount);
	return err;
}

long twoDCompensationOn()
{
	long err = ErrorCode::None;
	long Gantry = FRONT_GANTRY;
	Set2DCompensationUse(true);
	if (Get2DCompensationMethod() == 1)
	{
		TRACE(_T("[PWR] twoDCompensationOn Start\n"));
		TRACE(_T("[PWR] twoDCompensationOn End\n"));
	}
	else
	{
		TRACE(_T("[PWR] twoDCompensationOff Start\n"));
		TRACE(_T("[PWR] gDisable2DCompensation Start\n"));
		err = gDisable2DCompensation(CHANNEL_1);
		err = gDisable2DCompensation(CHANNEL_2);
		err = gDisable2DCompensation(CHANNEL_3);
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] gDisable2DCompensation End\n"));
		err = gSet2DCompensation_ForXY(Gantry);
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] gSet2DCompensation_ForXY End\n"));
		err = gEnable2DCompensation(CHANNEL_1);
		err = gEnable2DCompensation(CHANNEL_2);
		err = gEnable2DCompensation(CHANNEL_3);
		TRACE(_T("[PWR] gEnable2DCompensation End\n"));
		ThreadSleep(TIME500MS);
		TRACE(_T("[PWR] twoDCompensationOff End\n"));
	}
	return err;
}

long twoDCompensationOff()
{
	long err = ErrorCode::None;
	if (Get2DCompensationMethod() == 1)
	{
		TRACE(_T("[PWR] twoDCompensationOff Start\n"));
		TRACE(_T("[PWR] twoDCompensationOff End\n"));
	}
	else
	{
		TRACE(_T("[PWR] twoDCompensationOff Start\n"));
		err = gDisable2DCompensation(CHANNEL_1);
		err = gDisable2DCompensation(CHANNEL_2);
		err = gDisable2DCompensation(CHANNEL_3);
		ThreadSleep(TIME100MS);
		TRACE(_T("[PWR] twoDCompensationOff End\n"));
	}
	return err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
double gGetHomePosition(long AxisNo)
{
	double HomePosition = 0.0;
	HomePosition = gcPowerCalibrationData->GetHomePosition(AxisNo);
	return HomePosition;
}

void gSetHomePosition(long AxisNo, double Origin)
{
	gcPowerCalibrationData->SetHomePosition(AxisNo, Origin);
}

void gWriteHomePosition(long Gantry)
{
	gcPowerCalibrationData->WriteHomePosition(Gantry);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

long gDisableZCompensation(CString strAxisZ)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->DisableZCompensation(strAxisZ);
	return err;
}

long gEnableZCompensation(CString strAxisZ)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->EnableZCompensation(strAxisZ);
	return err;
}

long gSetZCompensation(CString strAxisZ)
{
	long err = ErrorCode::None;
	err = gcPowerGantry->SetZCompensation(strAxisZ);
	return err;
}

long AxisZCompensationOn(CString strAxisZ)
{
	long err = ErrorCode::None;
	err = gDisableZCompensation(strAxisZ);
	ThreadSleep(TIME100MS);
	err = gSetZCompensation(strAxisZ);
	ThreadSleep(TIME100MS);
	err = gEnableZCompensation(strAxisZ);
	ThreadSleep(TIME500MS);
	return err;
}

long AxisZCompensationOff(CString strAxisZ)
{
	long err = ErrorCode::None;
	err = gDisableZCompensation(strAxisZ);
	ThreadSleep(TIME100MS);
	return err;
}

long AllZCompensationOn(long Gantry)
{
	long err = ErrorCode::None;
	err = AxisZCompensationOn(_T("FZ1"));
	err = AxisZCompensationOn(_T("FZ2"));
	return err;
}

long AllZCompensationOff(long Gantry)
{
	long err = ErrorCode::None;
	err = AxisZCompensationOff(_T("FZ1"));
	err = AxisZCompensationOff(_T("FZ2"));
	return err;
}
//////////////////////////////////////////////////////////////////////////////////////////////////

long gPauseMachine()
{
	long Err = NO_ERR;
	//SEM_LOCK(gMACHINE_CONTROL, INFINITE);
	Err = gcMachineInit->PauseMachine();
	//SEM_UNLOCK(gMACHINE_CONTROL);
	return Err;
}

long gResumeMachine()
{
	long Err = NO_ERR;
	//SEM_LOCK(gMACHINE_CONTROL, INFINITE);
	Err = gcMachineInit->ResumeMachine();
	//SEM_UNLOCK(gMACHINE_CONTROL);
	return Err;
}

long gReleaseMachine()
{
	long Err = NO_ERR;
	//SEM_LOCK(gMACHINE_CONTROL, INFINITE);
	Err = gcMachineInit->ReleaseMachine();
	//SEM_UNLOCK(gMACHINE_CONTROL);
	return Err;
}

long gStopMachine()
{
	long Err = NO_ERR;
	Err = gcMachineInit->StopMachine();
	return Err;
}

void SetRunMode(long Mode)
{
	CString strLog;
	gcMachineInit->SetRunMode(Mode);
	TRACE(_T("[PWR] ************************** SetRunMode(%d) **************************\n"), Mode);
	strLog.Format(_T("[PWR] ************************** SetRunMode(%d) **************************"), Mode);
	gcPowerLog->Logging(strLog);
}

long GetRunMode()
{
	CString strLog;
	long Mode = NORMAL_MODE;
	Mode = gcMachineInit->GetRunMode();
	TRACE(_T("[PWR] ************************** GetRunMode(%d) **************************\n"), Mode);
	strLog.Format(_T("[PWR] ************************** GetRunMode(%d) **************************"), Mode);
	gcPowerLog->Logging(strLog);
	return Mode;
}

long GetRunModeNoLog()
{
	CString strLog;
	long Mode = NORMAL_MODE;
	Mode = gcMachineInit->GetRunMode();
	return Mode;
}

void SetStopMode(long Mode)
{
	gcMachineInit->SetStopMode(Mode);
	TRACE(_T("[PWR] ************************** SetStopMode(%d) **************************\n"), Mode);
}

long GetStopMode()
{
	long Mode = NORMAL_MODE;
	Mode = gcMachineInit->GetStopMode();
	return Mode;
}

bool WaitRunMode(long Mode, long TimeOut)
{
	ULONGLONG time = _time_get();

	while (1)
	{
		if (GetRunModeNoLog() == Mode)
		{
			TRACE(_T("[PWR] WaitRunMode(%d) OK\n"), Mode);
			return true;
		}
		else if (_time_elapsed(time) > TimeOut)
		{
			TRACE(_T("[PWR] WaitRunMode(%d) TimeOut\n"), Mode);
			return false;
		}

		ThreadSleep(TIME1MS);
	}

	return false;
}

void InitCStep(long Gantry)
{
	gcMachineInit->InitCStep(Gantry);
}

void DeleteCStep(long Gantry)
{
	gcMachineInit->DeleteCStep(Gantry);
}

void CreateCStep(long Gantry)
{
	gcMachineInit->CreateCStep(Gantry);

}

bool IsAliveCStep(long Gantry)
{
	return 	gcMachineInit->IsAliveCStep(Gantry);
}

long SetTowerLamp(TowerLampLed UserLamp)
{
	long Err = NO_ERR;
	Err = gcMachineInit->SetTowerLamp(UserLamp);
	return Err;
}

TowerLampLed GetTowerLamp()
{
	TowerLampLed UserLamp;
	UserLamp = gcMachineInit->GetTowerLamp();
	return UserLamp;
}

void SetUseRearCamera(long UseRearCamera)
{
	gcMachineInit->SetUseRearCamera(UseRearCamera);
}

long GetUseRearCamera()
{
	long UseRearCamera = 0;
	UseRearCamera = gcMachineInit->GetUseRearCamera();
	return UseRearCamera;
}

long CallbackHMI(unsigned Msg1, unsigned Msg2, unsigned Msg3, CString SendMsg)
{
	long Err = NO_ERR;
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = Msg1;
	SubMsg2 = Msg2;
	SubMsg3 = Msg3;
	strMsg = SendMsg;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcDecoding)
	{
		gcDecoding->GetId(&id);
		msgSend->SetID(id);
		if (gcDecoding->PingThread(TIME1MS))
		{
			gcDecoding->Event((LPVOID)msgSend);
		}
	}
	return Err;
}

void CallbackHMI_Pause()
{
	CallbackHMI(HMI_CMD1ST_2, HMI_CMD2ND_00, HMI_CMD3RD_01, _T("1"));
}
void CallbackHMI_Resume()
{
	CallbackHMI(HMI_CMD1ST_2, HMI_CMD2ND_00, HMI_CMD3RD_01, _T("2"));
}
void CallbackHMI_StopNow()
{
	CallbackHMI(HMI_CMD1ST_2, HMI_CMD2ND_00, HMI_CMD3RD_01, _T("3"));
}

void CallbackHMI_BoardStop()
{
	CallbackHMI(HMI_CMD1ST_2, HMI_CMD2ND_00, HMI_CMD3RD_01, _T("6"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////

long InitializeHeadMech(long Gantry)
{
	gcPowerGantry->InitializeHeadMech();
	return 0;
}

long InitializeHeadMech(long Gantry, long HeadNo)
{
	gcPowerGantry->InitializeHeadMech(HeadNo);
	return 0;
}

long InitializeCamPosMech(long Gantry)
{
	gcPowerGantry->InitializeCamPosMech();
	return 0;
}

long InitializeCamOffsetMech(long Gantry)
{
	gcPowerGantry->InitializeCamOffsetMech();
	return 0;
}

long InitializeRearCamPosMech(long Gantry)
{
	gcPowerGantry->InitializeRearCamPosMech();
	return 0;
}

long InitializeRearCamOffsetMech(long Gantry)
{
	gcPowerGantry->InitializeRearCamOffsetMech();
	return 0;
}

long LoadHeadOffsetCamPosOffsetValue(long Gantry)
{
	gcPowerGantry->LoadValue();
	return 0;
}

long SaveHeadOffsetCamPosOffsetValue(long Gantry)
{
	gcPowerGantry->SaveValue();
	return 0;
}

long SendCameraRecognitionOffset(long Gantry)
{
	gcPowerGantry->SendCameraRecognitionOffset(Gantry);
	return 0;
}

void SetGlobalDiscardPosition(Point_XYRZ Discard)
{
	gcPowerGantry->SetGlobalDiscardPosition(Discard);
}

Point_XYRZ GetGlobalDiscardPosition()
{
	Point_XYRZ Discard;
	Discard = gcPowerGantry->GetGlobalDiscardPosition();
	return Discard;
}

void SetGlobalNozzleNo(long HeadNo, long NozzleNo)
{
	gcPowerGantry->SetGlobalNozzleNo(HeadNo, NozzleNo);
}

long GetGlobalNozzleNo(long HeadNo)
{
	long GlobalNozzleNo;
	GlobalNozzleNo = gcPowerGantry->GetGlobalNozzleNo(HeadNo);
	return GlobalNozzleNo;
}

void SetGlobalNozzleInformation(long StationNo, NOZZLE NozzleInfo)
{
	gcPowerGantry->SetGlobalNozzleInformation(StationNo, NozzleInfo);
}

NOZZLE GetGlobalNozzleInformation(long StationNo)
{
	NOZZLE GlobalNozzleInfo;
	GlobalNozzleInfo = gcPowerGantry->GetGlobalNozzleInformation(StationNo);
	return GlobalNozzleInfo;
}

void SetGlobalTowerYellowLampTime(long YellowTowerLampTime)
{
	gcPowerGantry->SetGlobalTowerYellowLampTime(YellowTowerLampTime);
}

long GetGlobalTowerYellowLampTime()
{
	long YellowTowerLampTime;
	YellowTowerLampTime = gcPowerGantry->GetGlobalTowerYellowLampTime();
	return YellowTowerLampTime;
}

void SetGlobalEmptyBuzzerTime(long EmptyBuzzerTime)
{
	gcPowerGantry->SetGlobalEmptyBuzzerTime(EmptyBuzzerTime);
}

long GetGlobalEmptyBuzzerTime()
{
	long EmptyBuzzerTime;
	EmptyBuzzerTime = gcPowerGantry->GetGlobalEmptyBuzzerTime();
	return EmptyBuzzerTime;
}

void SetGlobalMachineReferenceMark(MachineReferenceMark ReferenceMark)
{
	gcPowerGantry->SetGlobalMachineReferenceMark(ReferenceMark);
}

MachineReferenceMark GetGlobalMachineReferenceMark()
{
	MachineReferenceMark ReferenceMark;
	ReferenceMark = gcPowerGantry->GetGlobalMachineReferenceMark();
	return ReferenceMark;
}

Point_XY GetFeederPosition(long Stage, long FdNo)
{
	Point_XY pt, RefPt;
	double Pitch = 0.0;
	long RefFd = 0;
	ZeroMemory(&pt, sizeof(pt));
	ZeroMemory(&RefPt, sizeof(RefPt));
	if (FdNo > 0)
	{
		RefFd = ReadReferenceFeederNo(Stage);
		if(FdNo <= MAXHALFFEEDNO)
			RefPt = ReadReferenceFeederPosition(FRONT_GANTRY);
		else
			RefPt = ReadReferenceFeederPosition(REAR_GANTRY);
		Pitch = ReadFeederPitch(Stage);
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] Reference FdNo:%d X,Y,%.3f,%.3f Pitch,%.3f\n"), RefFd, RefPt.x, RefPt.y, Pitch);
		}
		pt.x = (RefPt.x + ((static_cast<double>(FdNo) - RefFd) * Pitch));
		pt.y = (RefPt.y);
	}
	return pt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

bool IsEmergencyStop()
{
	if (gcWmx3Motor != NULL)
	{
		if (gcWmx3Motor->GetEmergencyStop() == true)
		{
			return true;
		}
	}
	return false;
}

double GetRotateAngle(double dest)
{
	long sign = 0;
	double origRot;
	origRot = dest;
	if (dest < 0.0) {
		sign = 1;
		dest = -dest;
	}
	dest = dest - ((long)(dest / 360.0) * 360.0);
	if (sign)
		dest = -dest;							/* -360.0 ~ 360.0 */
	dest = dest + 360.0;						/* 0.0 ~ 720.0 */
	dest = dest - ((long)(dest / 360.0) * 360.0);	/* 0.0 ~ 360.0 */
	if (dest > 190.0)
		dest = dest - 360.0;					/* -180.0 ~ 180.0 */
	return dest;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

bool IsStopperUp(long Conveyor)
{
	bool bUp = false;
	if (gcPowerConveyorControl)
	{
		bUp = gcPowerConveyorControl->IsStopperUp(Conveyor);
	}
	return bUp;
}

bool IsStopperDn(long Conveyor)
{
	bool bDown = false;
	if (gcPowerConveyorControl)
	{
		bDown = gcPowerConveyorControl->IsStopperDn(Conveyor);
	}
	return bDown;
}

void SetEntryPcbReady(long Conveyor, bool bReady)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetEntryPcbReady(bReady);
	}
}

void SetWorkPcbReady(long Conveyor, bool bReady)
{
	if (GetInfiniteDryRun() == false)
	{
		TRACE(_T("[PWR] SetWorkPcbReady Ready:%d\n"), bReady);
	}
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetWorkPcbReady(bReady);
	}
}

void SetWorkPcbOut(long Conveyor, bool Out)
{
	TRACE(_T("[PWR] SetWorkPcbOut Out:%d\n"), Out);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetWorkPcbOut(Out);
	}
}

bool IsEntryPcbReady(long Conveyor)
{
	bool Ready = false;
	if (gcPowerConveyorControl)
	{
		Ready = gcPowerConveyorControl->IsEntryPcbReady(Conveyor);
	}
	return Ready;
}

bool IsWorkPcbReady(long Conveyor)
{
	//if (GetWorkExistSkip() == 1)
	//{
	//	return true;
	//}

	bool Ready = false;
	if (gcPowerConveyorControl)
	{
		Ready = gcPowerConveyorControl->IsWorkPcbReady(Conveyor);
	}
	return Ready;
}

bool IsWorkPcbOut(long Conveyor)
{
	bool Ready = false;
	if (gcPowerConveyorControl)
	{
		Ready = gcPowerConveyorControl->IsWorkPcbOut(Conveyor);
	}
	return Ready;
}

void SetInsertDone(long InsertDone)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetInsertDone(InsertDone);
	}
}

long GetInsertDone(long Conveyor)
{
	long InsertDone = true;
	if (gcPowerConveyorControl)
	{
		InsertDone = gcPowerConveyorControl->GetInsertDone(Conveyor);
	}
	return InsertDone;
}

void SetPcbOutDone(long OutDone)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetPcbOutDone(OutDone);
	}
}

long GetPcbOutDone(long Conveyor)
{
	long OutDone = true;
	if (gcPowerConveyorControl)
	{
		OutDone = gcPowerConveyorControl->GetPcbOutDone(Conveyor);
	}
	return OutDone;
}

long GetConveyorRunMode()
{
	long ConveyorRunMode = LOCATION_OUT_NEXT;
	if (gcPowerConveyorControl)
	{
		ConveyorRunMode = gcPowerConveyorControl->GetConveyorRunMode();
	}
	return ConveyorRunMode;
}

void SetConveyorRunMode(long ConveyorRunMode)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetConveyorRunMode(RUN_DRY, ConveyorRunMode);
	}
	TRACE(_T("[PWR] SetConveyorRunMode ProdRun:%d ConveyorRun:%d\n"), RUN_DRY, ConveyorRunMode);
}

void SetProdRunMode(long ProdRunMode, long ConveyorRunMode)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetConveyorRunMode(ProdRunMode, ConveyorRunMode);
	}
	if (gcPowerMainControl)
	{
		gcPowerMainControl->SetProdRunMode(ProdRunMode, ConveyorRunMode);
	}
	TRACE(_T("[PWR] SetRunMode ProdRun:%d ConveyorRun:%d\n"), ProdRunMode, ConveyorRunMode);
}

long GetManualConveyorRunMode()
{
	long ManualConveyorRunMode = LOCATION_OUT_NEXT;
	if (gcPowerConveyorManualControl)
	{
		ManualConveyorRunMode = gcPowerConveyorManualControl->GetConveyorRunMode();
		TRACE(_T("[PWR] GetManualConveyorRunMode ConveyorRun:%d\n"), ManualConveyorRunMode);
	}
	return ManualConveyorRunMode;
}

void SetManualConveyorRunMode(long ConveyorRunMode)
{
	if (gcPowerConveyorManualControl)
	{
		gcPowerConveyorManualControl->SetConveyorRunMode(ConveyorRunMode);
	}
	TRACE(_T("[PWR] SetManualConveyorRunMode ConveyorRun:%d\n"), ConveyorRunMode);
}

void StartConveyor(unsigned Msg1, unsigned Msg2, unsigned Msg3)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = Msg1;
	SubMsg2 = Msg2;
	SubMsg3 = Msg3;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_START));
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_START));
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_START));
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void SetPcbInfoConveyor(double Thickness, double StandByZOffset, long SimultaneousLoading)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = (unsigned)(Thickness * 10.0);
	SubMsg2 = (unsigned)(StandByZOffset * 10.0);
	SubMsg3 = SimultaneousLoading;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_PCBINFO));
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_PCBINFO));
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_PCBINFO));
	}
	TRACE(_T("[PWR] SetPcbInfoConveyor SendMsg1~3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void SetRunInfoConveyor(long BarcodeType, long UseMes, long Reserved2)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = BarcodeType;
	SubMsg2 = UseMes;
	SubMsg3 = Reserved2;
	msgSend->SetThreadMsg(_T(STRING_CONVEYOR_RUNINFO));
	TRACE(_T("[PWR] SetRunInfoConveyor SendMsg1~3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void SetBarcodeResultConveyor(long Conveyor, long BarcodeResult, long Reserved1)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = Conveyor;
	SubMsg2 = BarcodeResult;
	SubMsg3 = Reserved1;
	msgSend->SetThreadMsg(_T(STRING_CONVEYOR_BARCODE_RESULT));
	TRACE(_T("[PWR] SetBarcodeResultConveyor SendMsg1~3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void RunConveyor(long ContinueRun, long From, long To)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_RUN));
	//	SubMsg3 = LOCATION_STAY_WORK;
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_RUN));
	//	SubMsg3 = LOCATION_RETURN_ENTRY;
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_RUN));
		if (ContinueRun == 1)
		{
			SubMsg1 = 1;
		}
		SubMsg3 = LOCATION_OUT_NEXT;
	}
	TRACE(_T("[PWR] RunConveyor SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void RunConveyor(long ContinueRun, double Width)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_RUN));
	//	SubMsg3 = LOCATION_STAY_WORK;
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_RUN));
	//	SubMsg3 = LOCATION_RETURN_ENTRY;
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_RUN));
		if (ContinueRun == 1)
		{
			SubMsg1 = 1;
		}
		SubMsg3 = LOCATION_OUT_NEXT;
	}
	SubMsg2 = (unsigned)(Width * 10.0);
	TRACE(_T("[PWR] RunConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void EndConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_END));
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_END));
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_END));
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void StopConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_STOP));
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//	msgSend->SetThreadMsg(_T(STRING_MANUAL_CONVEYOR_STOP));
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_STOP));
	}	
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void StartFreeTimeConveyor(long Conv)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = Conv;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_START_FREETIME));
	}
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] StartFreeTimeConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void StopFreeTimeConveyor(long Conv)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = Conv;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_STOP_FREETIME));
	}
	TRACE(_T("[PWR] StopFreeTimeConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void StartLoadingTimeConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_START_LOADING));
	}
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		TRACE(_T("[PWR] StartLoadingTimeConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void EndLoadingTimeConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_END_LOADING));
	}
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		TRACE(_T("[PWR] EndLoadingTimeConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void StartLineOfBalanceConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_START_LOB));
	}
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		TRACE(_T("[PWR] StartLineOfBalanceConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

void EndLineOfBalanceConveyor()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//if (GetConveyorRunMode() == LOCATION_STAY_WORK)
	//{
	//}
	//else if (GetConveyorRunMode() == LOCATION_RETURN_ENTRY)
	//{
	//}
	//else
	{
		msgSend->SetThreadMsg(_T(STRING_CONVEYOR_END_LOB));
	}
	if (gcPowerLog->IsShowElapsedLog() == true)
	{
		TRACE(_T("[PWR] EndLineOfBalanceConveyor SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	}
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorControl->PingThread(TIME1MS))
		{
			gcPowerConveyorControl->Event((LPVOID)msgSend);
		}
	}
}

ULONGLONG GetLoadingTime()
{
	ULONGLONG LoadingTime = 0;
	if (gcPowerConveyorControl)
	{
		LoadingTime = gcPowerConveyorControl->GetLoadingTime();
	}
	return LoadingTime;
}

ULONGLONG GetLineOfBalance()
{
	ULONGLONG LineOfBalance = 0;
	if (gcPowerConveyorControl)
	{
		LineOfBalance = gcPowerConveyorControl->GetLineOfBalance();
	}
	return LineOfBalance;
}

void SetWorkConveyorStopMidDelay(long WorkStopMidDelay)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetWorkConveyorStopMidDelay(WorkStopMidDelay);
	}
}

void SetWorkConveyorStopLowDelay(long WorkStopLowDelay)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetWorkConveyorStopLowDelay(WorkStopLowDelay);
	}
}

void SetConveyorProfileLow(double Vel, double Acc, double Dec)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetProfileLow(Vel, Acc, Dec);
	}
}

void SetConveyorProfileMid(double Vel, double Acc, double Dec)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetProfileMid(Vel, Acc, Dec);
	}
}

void SetConveyorProfileHigh(double Vel, double Acc, double Dec)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetProfileHigh(Vel, Acc, Dec);
	}
}

void SetConveyorProfileSpeed(long Conv, long PrevInBeltSpd, long NextOutBeltSpd)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetConveyorSpeed(Conv, PrevInBeltSpd, NextOutBeltSpd);
	}
}

long GetConveyorProfileSpeedPrevIn(long Conv)
{
	long PrevIn = BELT_SPEED_MID;
	if (gcPowerConveyorControl)
	{
		PrevIn = gcPowerConveyorControl->GetConveyorSpeedPrevIn(Conv);
	}
	return PrevIn;
}

long GetConveyorProfileSpeedNextOut(long Conv)
{
	long NextOut = BELT_SPEED_MID;
	if (gcPowerConveyorControl)
	{
		NextOut = gcPowerConveyorControl->GetConveyorSpeedNextOut(Conv);
	}
	return NextOut;
}

long SetConveyorTransferTimeOut(long Table, long Conv, long PrevInBeltSpd, long NextOutBeltSpd)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetPcbTransferTimeOut(Table, Conv, PrevInBeltSpd, NextOutBeltSpd);
	}
	return NO_ERR;
}
//////////////////////////////////////////////////////////////////////////////////////////////////

void StartManualLocationConveyor(long From, long To)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MANUAL_LOCATION_CONVEYOR_START));
	SubMsg1 = From;
	SubMsg2 = To;
	SubMsg3 = LOCATION_MANUAL;
	TRACE(_T("[PWR] StartManualLocationConveyor SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorManualControl)
	{
		gcPowerConveyorManualControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorManualControl->PingThread(TIME1MS))
		{
			gcPowerConveyorManualControl->Event((LPVOID)msgSend);
		}
	}
}

void SetPcbInfoManualLocationConveyor(double PcbThickness, double PcbStandByZOffset, long PusherzRatio)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = (unsigned)(PcbThickness * 10.0);
	SubMsg2 = (unsigned)(PcbStandByZOffset * 10.0);
	SubMsg3 = PusherzRatio;
	msgSend->SetThreadMsg(_T(STRING_MANUAL_LOCATION_CONVEYOR_PCBINFO));
	TRACE(_T("[PWR] SetPcbInfoManualLocationConveyor SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorManualControl)
	{
		gcPowerConveyorManualControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorManualControl->PingThread(TIME1MS))
		{
			gcPowerConveyorManualControl->Event((LPVOID)msgSend);
		}
	}
}

void RunManualLocationConveyor(long From, long To)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MANUAL_LOCATION_CONVEYOR_RUN));
	SubMsg1 = From;
	SubMsg2 = To;
	SubMsg3 = LOCATION_MANUAL;
	TRACE(_T("[PWR] RunManualLocationConveyor SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorManualControl)
	{
		gcPowerConveyorManualControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorManualControl->PingThread(TIME1MS))
		{
			gcPowerConveyorManualControl->Event((LPVOID)msgSend);
		}
	}
}

void StopManualLocationConveyor(long From, long To)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MANUAL_LOCATION_CONVEYOR_END));
	SubMsg1 = From;
	SubMsg2 = To;
	SubMsg3 = LOCATION_MANUAL;
	TRACE(_T("[PWR] StopManualLocationConveyor SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerConveyorManualControl)
	{
		gcPowerConveyorManualControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerConveyorManualControl->PingThread(TIME1MS))
		{
			gcPowerConveyorManualControl->Event((LPVOID)msgSend);
		}
	}
}


void StartSyncGantryConveyor()
{
	gcSyncGantryConveyor = new CSyncGantryConveyor();
}

void RunSyncGantryConveyor()
{
	if (gcSyncGantryConveyor)
	{
		gcSyncGantryConveyor->Run();
	}
}

void StopSyncGantryConveyor()
{
	if (gcSyncGantryConveyor)
	{
		gcSyncGantryConveyor->ExitThreadLoop();
		delete gcSyncGantryConveyor;
		gcSyncGantryConveyor = NULL;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void StartMain()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_START));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void PrepareMain()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_PREPARE));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void RunMain(bool ContinueRun)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_RUN));
	if (ContinueRun == true)
	{
		SubMsg1 = 1;
	}
	TRACE(_T("[PWR] RunMain SendMsg1,2,3,%d,%d,%d\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void StopMain(long StopTiming)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = StopTiming;
	SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_END));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void gFeederRefill(long FeederNo)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = FeederNo;
	SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_FEEDER_REFILL));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void gFeederRefillDone()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_FEEDER_REFILL_DONE));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void gMainMoveStandBy()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MAINCONTROL_MOVE_STANDBY));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

bool GetMainEnd()
{
	bool bEnd = false;
	if (gcPowerMain)
	{
		bEnd = gcPowerMain->IsEnd();
	}
	return bEnd;
}

bool GetConveyorEnd()
{
	bool bEnd = false;
	if (gcPowerConveyorControl)
	{
		bEnd = gcPowerConveyorControl->IsAllConveyorEnd();
	}
	return bEnd;
}

bool GetConveyorBeltStop()
{
	bool bEnd = false;
	if (gcPowerConveyorControl)
	{
		bEnd = gcPowerConveyorControl->IsAllConveyorBeltStop();
	}
	return bEnd;
}

bool GetManualLocationConveyorEnd()
{
	bool bEnd = false;
	if (gcPowerConveyorManualControl)
	{
		bEnd = gcPowerConveyorManualControl->IsAllConveyorEnd();
	}
	return bEnd;
}

long SendFeederAutoRefill(long FeederNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_23;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d"), FeederNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	//if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendFeederAutoRefill Done(%03d)\n"), FeederNo);
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

bool IsExistEnt(long Conv)
{
	bool bExist = false;
	if (gcPcbExist)
	{
		bExist = gcPcbExist->IsExistEnt(Conv);
	}
	return bExist;
}

bool IsExistLow(long Conv)
{
	bool bExist = false;
	if (gcPcbExist)
	{
		bExist = gcPcbExist->IsExistLow(Conv);
	}
	return bExist;
}

bool IsExistSet(long Conv)
{
	//if (GetWorkExistSkip() == 1)
	//{
	//	return true;
	//}

	bool bExist = false;
	if (gcPcbExist)
	{
		bExist = gcPcbExist->IsExistSet(Conv);
	}
	return bExist;
}

bool IsExistExit(long Conv)
{
	bool bExist = false;
	if (gcPcbExist)
	{
		bExist = gcPcbExist->IsExistExit(Conv);
	}
	return bExist;
}

bool IsExistOut(long Conv)
{
	bool bExist = false;
	if (gcPcbExist)
	{
		bExist = gcPcbExist->IsExistOut(Conv);
	}
	return bExist;
}

bool IsExistAll(long Conv)
{
	bool bExist = false;
	bExist = IsExistEnt(Conv);
	if (bExist == true) return true;
	bExist = IsExistLow(Conv);
	if (bExist == true) return true;
	bExist = IsExistSet(Conv);
	if (bExist == true) return true;
	bExist = IsExistExit(Conv);
	if (bExist == true) return true;
	bExist = IsExistOut(Conv);
	if (bExist == true) return true;
	return bExist;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void SaveInsertEnd()
{
	if (gcInsertEndFile)
	{
		gcInsertEndFile->SaveFile();
	}
}

void ClearInsertEnd(long Conv)
{
	SendToNewBoard();
	if (gcInsertEndFile)
	{
		TRACE(_T("[PWR] ***************************************** ClearInsertEnd Conveyor:%d *****************************************\n"), Conv);
		gcInsertEndFile->ClearInsertEnd(Conv);
		SaveInsertEnd();
	}
}

void SetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	if (gcInsertEndFile)
	{
		if (Point > 0 && Point < MAXINSERTNO)
		{
			gcInsertEndFile->SetInsertEnd(Conv, Gantry, Block, Point - 1);
			SaveInsertEnd();
		}
	}
}

void SetInsertClear(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	if (gcInsertEndFile)
	{
		if (Point > 0 && Point < MAXINSERTNO)
		{
			gcInsertEndFile->SetInsertStatus(Conv, Gantry, Block, Point - 1, INSERT_CLEAR);
			SaveInsertEnd();
		}
	}
}

unsigned GetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	unsigned InsertEnd = INSERT_CLEAR;
	if (gcInsertEndFile)
	{
		if (Point > 0 && Point < MAXINSERTNO)
		{
			InsertEnd = gcInsertEndFile->GetInsertEnd(Conv, Gantry, Block, Point - 1);
		}
	}
	return InsertEnd;
}

long GetRemainFirstNo(long MaxBlockNo, long MaxInsertNo)
{
	long Gantry = FRONT_GANTRY, Conveyor = FRONT_CONV, Block = 0, Point = 0, InsertEnd = INSERT_CLEAR;
	long MaxBlock = MaxBlockNo;
	if (MaxBlock < 1)
	{
		MaxBlock = 1;
	}
	if (gcPowerLog->IsShowInsertEndLog() == true)
	{
		TRACE(_T("[PWR] GetRemainFirstNo MaxBlock:%d MaxInsertNo:%d\n"), MaxBlock, MaxInsertNo);
	}
	if (IsExistSet(WORK1_CONV) == true)
	{
		//for (Block = 0; Block < MaxBlock; ++Block)
		{
			for (Point = 0; Point < MaxInsertNo; ++Point)
			{
                Block = gcReadJobFile->GetInsert(Gantry, Point + 1).BlockNo;
                if (
                    (gcReadJobFile->GetPcb().UseBlockType == PCB_MAXTRIX || gcReadJobFile->GetPcb().UseBlockType == PCB_NON_MAXTRIX)
                    &&
                    gcReadJobFile->GetBlockOrigin(Block).Use == NO_USE
                    )
                {
                    CString temp; temp.Format(L"skipping.. Point:%d, BlockNo:%d, BlockOriginUse:%d", Point, Block, gcReadJobFile->GetBlockOrigin(Block).Use);
                    TRACE_FILE_FUNC_LINE_(CStringA)temp);
                    continue;
                }
				if (GetInsertEnd(Conveyor, Gantry, Block, Point + 1) == INSERT_END)
				{
					if (gcPowerLog->IsShowInsertEndLog() == true)
					{
						TRACE(_T("[PWR] Already GetRemainFirstNo Insert End Conveyor:%d Gantry:%d Block:%d Point:%d\n"), Conveyor, Gantry, Block, Point);
					}
				}
				else
				{
					if (gcPowerLog->IsShowInsertEndLog() == true)
					{
						TRACE(_T("[PWR] GetRemainFirstNo1 Reamin Conveyor:%d Gantry:%d Block:%d Start Point:%d\n"), Conveyor, Gantry, Block, Point);
					}
					return Point + 1;
				}
			}
		}
		return MAXINSERTDONECOUNT;
	}
	if (gcPowerLog->IsShowInsertEndLog() == true)
	{
		TRACE(_T("[PWR] GetRemainFirstNo2 Conveyor:%d Gantry:%d Block:%d Point:%d\n"), Conveyor, Gantry, Block, Point);
	}
	return Point + 1;
}

bool GetAllInertEnd()
{
	PRODUCTION Prod = gcReadJobFile->GetProduction();
	PCB Pcb = gcReadJobFile->GetPcb();

	if (MAXINSERTDONECOUNT != GetRemainFirstNo(Pcb.MaxBlockCount, Prod.TotalInsertCount))
	{
		return false;
	}

	return true;
}

long GetRemainFirstBlock(long MaxBlockNo, long MaxInsertNo)
{
	long Gantry = FRONT_GANTRY, Conveyor = FRONT_CONV, Block = 0, Point = 0, InsertEnd = INSERT_CLEAR;
	long MaxBlock = MaxBlockNo;
	if (MaxBlock < 1)
	{
		MaxBlock = 1;
	}
	if (gcPowerLog->IsShowInsertEndLog() == true)
	{
		TRACE(_T("[PWR] GetRemainFirstBlock MaxBlockNo:%d MaxInsertNo:%d\n"), MaxBlock, MaxInsertNo);
	}
	if (IsExistSet(WORK1_CONV) == true)
	{
		for (Block = 0; Block < MaxBlock; ++Block)
		{
			for (Point = 0; Point < MaxInsertNo; ++Point)
			{
				if (GetInsertEnd(Conveyor, Gantry, Block, Point + 1) == INSERT_END)
				{
					if (gcPowerLog->IsShowInsertEndLog() == true)
					{
						TRACE(_T("[PWR] Already GetRemainFirstBlock Insert End Conveyor:%d Gantry:%d Block:%d Point:%d\n"), Conveyor, Gantry, Block, Point);
					}
				}
				else
				{
					if (gcPowerLog->IsShowInsertEndLog() == true)
					{
						TRACE(_T("[PWR] GetRemainFirstBlock1 Reamin Conveyor:%d Gantry:%d Block:%d Point:%d\n"), Conveyor, Gantry, Block, Point);
					}
					return Block + 1;
				}
			}
		}
		return MAXINSERTDONECOUNT;
	}
	if (gcPowerLog->IsShowInsertEndLog() == true)
	{
		TRACE(_T("[PWR] GetRemainFirstBlock2 Conveyor:%d Gantry:%d Block:%d Point:%d\n"), Conveyor, Gantry, Block, Point);
	}
	return Block + 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

long GetAxisIndexFromAliasName(CString strAxis)
{
	long AxisIndx = NON;
	if (strAxis.GetLength() > 0)
	{
		for (long indx = 0; indx < MAXAXISNO; ++indx)
		{
			if (strAxis.CompareNoCase(PowerAxisAliasName[indx]) == 0)
			{
				AxisIndx = indx;
				break;
			}
		}
	}
	return AxisIndx;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long SendToHMI(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = SubMsg1;
	nSubMsg[1] = SubMsg2;
	nSubMsg[2] = SubMsg3;
	strSendMsg.Format(_T("%s"), (LPCTSTR)strMsg);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
	return NO_ERR;
}

long SendInitializeHMI(long Status)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_02;
	nSubMsg[2] = HMI_CMD3RD_01;
	strSendMsg.Format(_T("%d"), Status);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendInitializeHMI Done\n"));
	}
	return 0;
}

long SendAlarm(long AlarmCode, CString AlarmMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));


	if (AlarmCode == NO_ERR || AlarmCode < 0)
	{
		TRACE(_T("[PWR] SendAlarm(%d) InvalidCode (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
		AlarmCode = ALARM_STATUS;
	}

	if (AlarmCode == STOP_NOW && GetRunMode() == PAUSE_MODE)
	{
		SetGlobalStatusError(true);
		TRACE(_T("[PWR] SendAlarm(%d) SkipStopNow,Pause (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
		return AlarmCode;
	}

	if (AlarmCode == STOP_NOW && GetRunMode() == NORMAL_MODE)
	{
		TRACE(_T("[PWR] SendAlarm(%d) SkipStopNow,Normal (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
		return AlarmCode;
	}

	strSendMsg.Format(_T("%d,%s"), AlarmCode, (LPCTSTR)AlarmMsg);
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_00;

	if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] SendAlarm(%d) SkipAlreadyAlarmState (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
		return AlarmCode;
	}

	TRACE(_T("[PWR] SendAlarm(%d) SendDone (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	SendToChangeMachineState(TimeStatics::ALARM);

	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendAlarm Done()%s\n"), (LPCTSTR)AlarmMsg);
	}
	//if (GetRunMode() == PROD_RUN)
	if (GetRunMode() != NORMAL_MODE)
	{
		TowerLampAlarm(AlarmCode);
		long EmptyBuzzerTime = GetGlobalEmptyBuzzerTime();
		BuzzerOn(EmptyBuzzerTime); // 10 Sec
		SetGlobalStatusError(true);
	}

	if (GetRunMode() == NORMAL_MODE)
	{
		TRACE(_T("[PWR] SendAlarm(%d) CodeChange (%s)\n"), AlarmCode, (LPCTSTR)AlarmMsg);
		AlarmCode = STOP_NOW;
	}

	return AlarmCode;
}

long SendForceAlarm(long AlarmCode, CString AlarmMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%d,%s"), AlarmCode, (LPCTSTR)AlarmMsg);
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_00;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendForceAlarm Done()%s\n"), (LPCTSTR)AlarmMsg);
	}
	//if (GetRunMode() == PROD_RUN)
	{
		TowerLampAlarm(AlarmCode);
		long EmptyBuzzerTime = 1000000000;
		BuzzerOn(CPowerBuzzer::GlobalBuzzerTimeInSeconds); // 999 Sec
        (void)gcPowerBuzzer->SetLastSafetyAlarmCode(AlarmCode);
		SetGlobalStatusError(true);
	}
	return 0;
}

long SendAlarmForNormal(long AlarmCode, CString AlarmMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));

	strSendMsg.Format(_T("%d,%s"), AlarmCode, (LPCTSTR)AlarmMsg);
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_00;

	TRACE(_T("[PWR] SendAlarmForNormal (%d,%d,%d,%s)\n"), nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendAlarmForNormal Done\n"));
	}
	return 0;
}

long SendAlarmOnlyBuzzer(long AlarmCode, CString AlarmMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));

	if (AlarmCode == STOP_NOW && GetRunMode() == PAUSE_MODE)
	{
		SetGlobalStatusError(true);
		TRACE(_T("[PWR] SendAlarmOnlyBuzzer STOP_NOW is NoAlarm\n"));
		return 0;
	}

	//if (GetGlobalStatusError() == false)
	{
		strSendMsg.Format(_T("%d,%s"), AlarmCode, (LPCTSTR)AlarmMsg);
		nSubMsg[0] = HMI_CMD1ST_1;
		nSubMsg[1] = HMI_CMD2ND_00;
		nSubMsg[2] = HMI_CMD3RD_00;

		TRACE(_T("[PWR] SendAlarmOnlyBuzzer (%d,%d,%d,%s)\n"), nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
		SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE(_T("[PWR] SendAlarmOnlyBuzzer Done()%s\n"), (LPCTSTR)AlarmMsg);
		}

		if (GetRunMode() != NORMAL_MODE)
		{
			TowerLampAlarm(AlarmCode);
			long EmptyBuzzerTime = GetGlobalEmptyBuzzerTime();
			BuzzerOn(EmptyBuzzerTime);
		}
	}
	//else
	//{
	//	TRACE(_T("[PWR] SendAlarmOnlyBuzzer Already Alarm State, New AlarmCode:%d Msg:%s\n"), AlarmCode, (LPCTSTR)AlarmMsg);
	//}

	return 0;
}

long GetZTorqueAlarmcode(long Gantry, long HeadNo)
{
	long torqueErr = NO_ERR;

	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			torqueErr = MAX_TORQUE_LIMIT_FZ1 + HeadNo - 1;
		}
		else if (Gantry == REAR_GANTRY)
		{
			torqueErr = MAX_TORQUE_LIMIT_RZ1 + HeadNo - 1;
		}
	}

	return torqueErr;
}

long GetZTorqueInsertFailAlarmcode(long Gantry, long HeadNo)
{
	long torqueErr = NO_ERR;

	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			torqueErr = INSERT_FAIL_TORQUE_FZ1 + HeadNo - 1;
		}
		else if (Gantry == REAR_GANTRY)
		{
			torqueErr = INSERT_FAIL_TORQUE_RZ1 + HeadNo - 1;
		}
	}

	return torqueErr;
}

long GetZTorquePickFailAlarmcode(long Gantry, long HeadNo)
{
	long torqueErr = NO_ERR;

	if (TBL_HEAD1 <= HeadNo && HeadNo <= TBL_HEAD6)
	{
		if (Gantry == FRONT_GANTRY)
		{
			torqueErr = PICK_FAIL_TORQUE_FZ1 + HeadNo - 1;
		}
		else if (Gantry == REAR_GANTRY)
		{
			torqueErr = PICK_FAIL_TORQUE_RZ1 + HeadNo - 1;
		}
	}

	return torqueErr;
}

long SendEmpty(long Feeder, long Type, CString EmptyMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	strSendMsg.Format(_T("%d,%d,%s"), Feeder, Type, (LPCTSTR)EmptyMsg);
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_20;
	nSubMsg[2] = HMI_CMD3RD_00;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] Feeder%03d SendEmpty Done\n"), Feeder);
	}
	return 0;
}

long SendRequestResetButton()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_01;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendRequestResetButton Done\n"));
	}
	return 0;
}

long SendResetDone()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_02;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendResetDone Done\n"));
	}
	return 0;
}

long SendInitializeHome(long Timing)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_03;
	strSendMsg.Format(_T("%d"), Timing);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendInitializeHome Done\n"));
	}
	return 0;
}

long AlarmSelfQuit(long StartMode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_02;
	strSendMsg.Format(_T("%d"), StartMode);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] AlarmSelfQuit Done StartMode:%d\n"), StartMode);
	}
	return 0;
}

long SendDoorStatus(long Status, long FrontOrRear) // 0:Open 1:Close
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d,%d"), Status, FrontOrRear);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendDoorStatus Done\n"));
	}
    if (Status == 0)
    {
        CString temp; temp.Format(L"%s Door Open", (FrontOrRear == FRONT_GANTRY) ? L"FRONT" : L"REAR");
        SendForceAlarm(OPEN_DOOR, temp);
    }
	return 0;
}

long SendServoStatus(long Status) // 0:On, 1:Off
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_02;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d"), Status);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendServoStatus Done\n"));
	}
	return 0;
}

long SendEmergencyStatus(long Status) // 0:On, 1:Off
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_05;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d"), Status);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendEmergencyStatus Done\n"));
	}
	if (Status == 0)
	{
		SendForceAlarm(PUSH_EMERGENCY, _T("Emergency"));
	}
	return 0;
}

long SendLotoKeyStatus(long Status)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_06;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d"), Status);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendLotoKeyStatus Done\n"));
	}
	if (Status == LOTO_KEY_LOCK)
	{
		SendForceAlarm(OPEN_LOTOKEY, _T("LotoKey"));
	}
	return 0;
}

long SendMachineStartStopStatus(long Status)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_02;
	strSendMsg.Format(_T("%d"), Status);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendMachineStartStopStatus Done\n"));
	}
	return 0;
}

long SendProdInfo(long ProdCompleteCount, long ProdTime, long TactTime, long ChipPerHour, long ProdTimeWithLoadingTime, long TactTimeWithLoadingTime)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_04;
	strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d"), ProdCompleteCount, ProdTime, TactTime, ChipPerHour, ProdTimeWithLoadingTime, TactTimeWithLoadingTime);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendProdInfo Done\n"));
	}
	return 0;
}

long SendProdInfo(long ProdCompleteCount, long ProdTime, long TactTime, long ChipPerHour, long ProdTimeWithLoadingTime, long TactTimeWithLoadingTime, CString strBarcode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_04;
	strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d,%s"), ProdCompleteCount, ProdTime, TactTime, ChipPerHour, ProdTimeWithLoadingTime, TactTimeWithLoadingTime, (LPCTSTR)strBarcode);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendProdInfo Done\n"));
	}
	return 0;
}

long SendBlockProdInfo(long ProdCompleteCount, long BlockProdCompleteCount)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD2ND_19;
	strSendMsg.Format(_T("%d,%d"), ProdCompleteCount, BlockProdCompleteCount);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendBlockProdInfo %d %d Done\n"), ProdCompleteCount, BlockProdCompleteCount);
	}
	return 0;
}

long SendReadyIOStatus(CString ReadyIOStatus)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	long Length = 0;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%s"), (LPCTSTR)ReadyIOStatus);
	Length = strSendMsg.GetLength();
	if (Length > 0)
	{
		strSendMsg.Delete(Length - 1, 1);
	}
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_08;
	nSubMsg[2] = HMI_CMD3RD_01;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendReadyIOStatus Done\n"));
	}
	return 0;
}

long SendGantryTempStatus(CString TempStatus)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%s"), (LPCTSTR)TempStatus);
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_09;
	nSubMsg[2] = HMI_CMD3RD_01;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendGantryTempStatus Done\n"));
	}
	return 0;
}

long SendPcbSensorStatus(CString PcbSensor)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%s"), (LPCTSTR)PcbSensor);
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_20;
	nSubMsg[2] = HMI_CMD3RD_01;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendPcbSensorStatus Done\n"));
	}
	return 0;
}

long SendToPrepareRun()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_05;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToPrepareRun Done\n"));
	}
	return 0;
}

long SendToRun()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_06;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToRun Done\n"));
	}
	return 0;
}

long SendToInsertComplete(long BlockNo, long InsertNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_08;
	strSendMsg.Format(_T("%d,%d"), BlockNo, InsertNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToInsertComplete Done\n"));
	}
	return 0;
}

long SendToNewBoard()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_09;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToNewBoard Done\n"));
	}
	return 0;
}


long SendToZTorqueMonitorFile(CString strFileName, long TorqueLimit)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%s,%d"), (LPCTSTR)strFileName, TorqueLimit);
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_00;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToZTorqueMonitorFile Done\n"));
	}
	return 0;
}


long SendInformation(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	long Length = 0;
	bool bReady = false;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%s"), (LPCTSTR)strMsg);
	nSubMsg[0] = SubMsg1;
	nSubMsg[1] = SubMsg2;
	nSubMsg[2] = SubMsg3;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendInformation Done\n"));
	}
	return 0;
}

long SendIOStatus(CString strMsg)
{
	SendInformation(HMI_CMD1ST_3, HMI_CMD2ND_50, HMI_CMD3RD_02, strMsg);
	return 0;
}

long SendTrayLastPocket(long TrayNo, long LastPocket)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%d,%d"), TrayNo, LastPocket);
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_02;
	nSubMsg[2] = HMI_CMD2ND_00;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendTrayLastPocket Done\n"));
	}
	return 0;
}

long SendNozzleNoStatus()
{
	unsigned nSubMsg[3];
	CString strSendMsg;

	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	strSendMsg.Format(_T("%d,%d,%d,%d,%d,%d"),
		GetGlobalNozzleNo(TBL_HEAD1), GetGlobalNozzleNo(TBL_HEAD2), GetGlobalNozzleNo(TBL_HEAD3), GetGlobalNozzleNo(TBL_HEAD4), GetGlobalNozzleNo(TBL_HEAD5), GetGlobalNozzleNo(TBL_HEAD6));

	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_60;
	nSubMsg[2] = HMI_CMD3RD_02;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendNozzleNoStatus Done\n"));
	}

	return 0;
}

long SendToPickupStatisticsComplete(long FeederNo, long NozzleNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_11;
	strSendMsg.Format(_T("%d,%d"), FeederNo, NozzleNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToPickupStatisticsComplete Done\n"));
	}
	return 0;
}

long SendToInsertStatisticsComplete(long InsertNo, long FeederNo, long NozzleNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_10;
	strSendMsg.Format(_T("%d,%d,%d"), InsertNo, FeederNo, NozzleNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToInsertStatisticsComplete Done\n"));
	}
	return 0;
}
long SendToNoComponentStatistics(long FeederNo, long NozzleNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_12;
	strSendMsg.Format(_T("%d,%d"), FeederNo, NozzleNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToNoComponentStatistics Done\n"));
	}
	return 0;
}

long SendToRunTimeDiscardStatistics(long FeederNo, long NozzleNo)
{
	TRACE(_T("[PWR] SendToRunTimeDiscardStatistics Feeder:%d Nozzle:%d\n"), FeederNo, NozzleNo);

	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_10;
	strSendMsg.Format(_T("%d,%d"), FeederNo, NozzleNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	return 0;
}

long SendToPickSkipByLastPick(long FeederNo)
{
	TRACE(_T("[PWR] SendToPickSkipByLastPick Feeder:%d \n"), FeederNo);

	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_11;
	strSendMsg.Format(_T("%d"), FeederNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	return 0;
}

long SendToVisionResult(long Gantry, long InsertNo, long ErrorCode)
{
	TRACE(_T("[PWR] SendToVisionResult Gantry:%d InsertNo:%d ErrorCode:%d \n"), Gantry, InsertNo, ErrorCode);

	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_13;
	strSendMsg.Format(_T("%d,%d,%d"), Gantry, InsertNo, ErrorCode);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	return 0;
}

long SendPopupMessage(CString Message)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	long Show = 1;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_30;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d,%s"), Show, (LPCTSTR)Message);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	//if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendPopupMessage Done(%s)\n"), Message);
	}
	return 0;
}

long SendPopupClose()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	CString Message = _T("Close");
	long Show = 2;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_30;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d,%s"), Show, (LPCTSTR)Message);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendPopupClose Done\n"));
	}
	return 0;
}
long SendToForming1Statistics(long FeederNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_17;
	strSendMsg.Format(_T("%d"), FeederNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToForming1Statistics Done\n"));
	}
	return 0;
}

long SendToForming2Statistics(long FeederNo)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_00;
	nSubMsg[2] = HMI_CMD3RD_18;
	strSendMsg.Format(_T("%d"), FeederNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendToForming2Statistics Done\n"));
	}
	return 0;
}
long SendSuctionDifferentLevel(long FeederNo, long Level)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	CString Message = _T("Close");
	long Show = 2;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_60;
	nSubMsg[2] = HMI_CMD3RD_08;
	strSendMsg.Format(_T("%d,%d"), FeederNo, Level);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendSuctionDifferentLevel Done\n"));
	}
	return 0;
}

long SendToLedRetrytatistics(long FeederNo)
{
	TRACE(_T("[PWR] SendToLedRetrytatistics Feeder:%d\n"), FeederNo);

	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_2;
	nSubMsg[1] = HMI_CMD2ND_01;
	nSubMsg[2] = HMI_CMD3RD_12;
	strSendMsg.Format(_T("%d"), FeederNo);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	return 0;
}

long SendToChangeMachineState(TimeStatics state)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	CString strFunc(__func__);
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_26;
	nSubMsg[2] = HMI_CMD3RD_00;
	strSendMsg.Format(_T("%d"), (long)state);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	//if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] %s %s\n"), strFunc, GetTimeStaticsName(state));
	}
	return 0;
}

long SendManualHeightMeasureEnd()
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_3;
	nSubMsg[1] = HMI_CMD2ND_04;
	nSubMsg[2] = HMI_CMD3RD_11;
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendManualHeightMeasureEnd Done\n"));
	}
	return 0;
}


long SendLampStatus(long Green, long Yellow, long Red, long AlarmCode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));

	long GreenState = 0, YellowState = 0, RedState = 0;
	CString GreenLog = _T("Off"), YellowLog = _T("Off"), RedLog = _T("Off");

	if (Green == OUTON)
	{
		GreenState = 1;
		GreenLog = _T("On");
	}

	if (Yellow == OUTON)
	{
		YellowState = 1;
		YellowLog = _T("On");
	}

	if (Red == OUTON)
	{
		RedState = 1;
		RedLog = _T("On");
	}

	nSubMsg[0] = HMI_CMD1ST_1;
	nSubMsg[1] = HMI_CMD2ND_25;
	nSubMsg[2] = HMI_CMD3RD_00;

	strSendMsg.Format(_T("%d%d%d,%d"), GreenState, YellowState, RedState, AlarmCode);
	SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);
	//if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] SendLampStatus Green:%s Yellow:%s Red:%s AlarmCode:%d\n"), GreenLog, YellowLog, RedLog, AlarmCode);

	}
	return 0;
}

CString GetTimeStaticsName(TimeStatics set)
{
	CString strName = _T("Undefined");

	switch (set)
	{
	case TimeStatics::STOP:
		strName = _T("STOP");
		break;

	case TimeStatics::RUN:
		strName = _T("RUN");
		break;

	case TimeStatics::PAUSE:
		strName = _T("PAUSE");
		break;

	case TimeStatics::ALARM:
		strName = _T("ALARM");
		break;

	case TimeStatics::REFILL_WAIT_START:
		strName = _T("REFILL_WAIT_START");
		break;

	case TimeStatics::REFILL_WAIT_END:
		strName = _T("REFILL_WAIT_END");
		break;

	case TimeStatics::ENTRY_WAIT_START:
		strName = _T("ENTRY_WAIT_START");
		break;

	case TimeStatics::ENTRY_WAIT_END:
		strName = _T("ENTRY_WAIT_END");
		break;

	case TimeStatics::EXIT_WAIT_START:
		strName = _T("EXIT_WAIT_START");
		break;

	case TimeStatics::EXIT_WAIT_END:
		strName = _T("EXIT_WAIT_END");
		break;

	default:
		break;
	}

	return strName;
}


//////////////////////////////////////////////////////////////////////////////////////////////////

long CheckReadyToMachine(bool ServoCheck)
{
	long RetMsg = NO_ERR;
	CString strMsg;
	CString strAxis;
	strMsg.Format(_T("NO Err"));

	SetDefaultTorqueLimitEvent(FRONT_GANTRY);

	GantryEventEnable();

	if (InputOne(IN_FEMERGENCY) == INON || InputOne(IN_REMERGENCY) == INON)
	{
		RetMsg = PUSH_EMERGENCY;				
		strMsg.Format(_T("Emergency Push"));

	}
	if (InputOne(IN_FDOOR_KEY_OUT) == INON || InputOne(IN_RDOOR_KEY_OUT) == INON)
	{
		RetMsg = OPEN_DOOR;
		strMsg.Format(_T("Door Open"));

	}
	if (InputOne(IN_LOTO_KEY_ON) == INON)
	{
		RetMsg = OPEN_LOTOKEY;
		strMsg.Format(_T("LOTOKey On"));

	}

	if (ServoCheck == true)
	{
		for (long AxisNo = 0; AxisNo < MAXGANTRYAXISNO; ++AxisNo)
		{
			strAxis = GetAxisNameByAxisIndex(AxisNo);
			if (GetAxisMap(strAxis) == NON)
			{
				continue;
			}
			if (CheckServoOn(strAxis) == false)
			{
				RetMsg = SERVO_ON_TIMEOUT(AxisNo);
				strMsg.Format(_T("%s Servo Off"), (LPCTSTR)PowerAxisAliasName[AxisNo]);

				RetMsg = SendAlarm(RetMsg, _T("Machine is not Ready"));
				return RetMsg;
			}
		}

		RetMsg = WaitGantryRZIdle(TIME5000MS);
		if (RetMsg != NO_ERR)
		{
			RetMsg = SendAlarm(RetMsg, _T("Idle TimeOut"));
			return RetMsg;
		}

		double torqueMax = 30.0;
		double TorqueX = ReadActualTorque(GetAxisX(FRONT_GANTRY));
		double TorqueY1 = ReadActualTorque(GetAxisY1(FRONT_GANTRY));
		double TorqueY2 = ReadActualTorque(GetAxisY2(FRONT_GANTRY));

		if (GetY2ShiftAutoInit().Use == true)
		{
			torqueMax = GetY2ShiftAutoInit().TorqueLimit;
		}

		if (fabs(TorqueX) > torqueMax)
		{
			RetMsg = LM_TORQUE_HIGH_FX;
			strMsg.Format(_T("FX Torque Over. Max:%.1f Actual:%.1f)"), torqueMax, TorqueX);
			SendAlarm(RetMsg, strMsg);
		}

		if (fabs(TorqueY1) > torqueMax)
		{
			RetMsg = LM_TORQUE_HIGH_FY1;
			strMsg.Format(_T("FY1 Torque Over. Max:%.1f Actual:%.1f)"), torqueMax, TorqueY1);
			SendAlarm(RetMsg, strMsg);
		}

		if (fabs(TorqueY2) > torqueMax)
		{
			RetMsg = LM_TORQUE_HIGH_FY2;
			strMsg.Format(_T("FY2 Torque Over. Max:%.1f Actual:%.1f)"), torqueMax, TorqueY2);
			SendAlarm(RetMsg, strMsg);
		}

		if (RetMsg != NO_ERR)
		{
			gServoAllOff();
			return RetMsg;
		}
	}

	if (GetSkipMotorPower() == true)
	{
		TRACE(_T("[PWR] Skip to Servo Enable Check\n"));
	}
	else
	{
		if (GetUseRTDSensorFX() == 1)
		{
			TRACE(_T("[PWR] CheckReadyToMachine GetTemperature FX:%.1f\n"), GetTemperature(GetAxisX(FRONT_GANTRY)));
			if (GetTemperature(GetAxisX(FRONT_GANTRY)) > HIGH_TEMPERATURE)
			{
				RetMsg = LM_TEMP_HIGH_FX;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisX(FRONT_GANTRY));

			}
		}
		else
		{
			if (InputOne(IN_FX_LMTEMP_LOW) == INOFF)
			{
				RetMsg = LM_TEMP_HIGH_FX;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisX(FRONT_GANTRY));

			}
		}
		if (GetUseRTDSensorFY1() == 1)
		{
			TRACE(_T("[PWR] CheckReadyToMachine GetTemperature FY1:%.1f\n"), GetTemperature(GetAxisY1(FRONT_GANTRY)));
			if (GetTemperature(GetAxisY1(FRONT_GANTRY)) > HIGH_TEMPERATURE)
			{
				RetMsg = LM_TEMP_HIGH_FY1;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY1(FRONT_GANTRY));

			}
		}
		else
		{
			if (InputOne(IN_FY1_LMTEMP_LOW) == INOFF)
			{
				RetMsg = LM_TEMP_HIGH_FY1;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY1(FRONT_GANTRY));

			}
		}
		if (GetUseRTDSensorFY2() == 1)
		{
			TRACE(_T("[PWR] CheckReadyToMachine GetTemperature FY2:%.1f\n"), GetTemperature(GetAxisY2(FRONT_GANTRY)));
			if (GetTemperature(GetAxisY2(FRONT_GANTRY)) > HIGH_TEMPERATURE)
			{
				RetMsg = LM_TEMP_HIGH_FY2;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY2(FRONT_GANTRY));

			}
		}
		else
		{
			if (InputOne(IN_FY2_LMTEMP_LOW) == INOFF)
			{
				RetMsg = LM_TEMP_HIGH_FY2;
				strMsg.Format(_T("%s Temperature high"), (LPCTSTR)GetAxisY2(FRONT_GANTRY));

			}
		}
		if (InputOne(IN_SAFETY_ON) == INOFF)
		{
			RetMsg = DISABLE_MOTOR_POWER;
			strMsg.Format(_T("Motor Power Input Off"));

		}

		if (GetUseAreaSensor() == 1)
		{
			if (InputElapsedTimeOne(IN_FRONT_AREA_SENSOR, INOFF, TIME20MS) == true)
			{
				RetMsg = DETECTED_AREA_SENSOR_FRONT;
				strMsg.Format(_T("Front Area Sensor Detected"));
			}
			else if (InputElapsedTimeOne(IN_REAR_AREA_SENSOR, INOFF, TIME20MS) == true)
			{
				RetMsg = DETECTED_AREA_SENSOR_REAR;
				strMsg.Format(_T("Rear Area Sensor Detected"));
			}
			else
			{
				for (long EventID = EVENTID_FX_FRONT_AREA_SENSOR; EventID <= EVENTID_FY1_REAR_AREA_SENSOR; EventID++)
				{
					if (gcPowerGantry->GetEventAvailable(EventID) == true)
					{
						TRACE(_T("[PWR] EnableEvent EVENTID:%d\n"), EventID);
						EnableEvent(EventID);
					}
				}
			}
		}

		if (GetUseAreaSensor2nd() == 1)
		{
			if (InputElapsedTimeOne(IN_FRONT_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
			{
				RetMsg = DETECTED_AREA_SENSOR_FRONT;
				strMsg.Format(_T("Front Area Sensor2nd Detected"));
			}
			else if (InputElapsedTimeOne(IN_REAR_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
			{
				RetMsg = DETECTED_AREA_SENSOR_REAR;
				strMsg.Format(_T("Rear Area Sensor2nd Detected"));
			}
			else
			{
				for (long EventID = EVENTID_FX_FRONT_AREA_SENSOR_2ND; EventID <= EVENTID_FY1_REAR_AREA_SENSOR_2ND; EventID++)
				{
					if (gcPowerGantry->GetEventAvailable(EventID) == true)
					{
						TRACE(_T("[PWR] EnableEvent EVENTID:%d\n"), EventID);
						EnableEvent(EventID);
					}
				}
			}
		}
	}
	if (RetMsg != NO_ERR)
	{
		RetMsg = SendAlarm(RetMsg, strMsg);
	}
	return RetMsg;
}


long CheckMachineSafety()
{
	if (GetSkipMotorPower() == true)
	{
		TRACE(_T("[PWR] Skip to Servo Enable Check\n"));
		return NO_ERR;
	}

	if (InputOne(IN_FEMERGENCY) == INON || InputOne(IN_REMERGENCY) == INON)
	{
		return PUSH_EMERGENCY;
	}
	if (InputOne(IN_FDOOR_KEY_OUT) == INON || InputOne(IN_RDOOR_KEY_OUT) == INON)
	{
		return OPEN_DOOR;
	}
	if (InputOne(IN_LOTO_KEY_ON) == INON)
	{
		return OPEN_LOTOKEY;
	}

	if (GetUseAreaSensor() == 1)
	{
		if (InputTimeOne(IN_FRONT_AREA_SENSOR, INOFF, TIME20MS) == true)
		{
			return DETECTED_AREA_SENSOR_FRONT;
		}
		if (InputTimeOne(IN_REAR_AREA_SENSOR, INOFF, TIME20MS) == true)
		{
			return DETECTED_AREA_SENSOR_REAR;
		}
	}

	if (GetUseAreaSensor2nd() == 1)
	{
		if (InputTimeOne(IN_FRONT_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
		{
			return DETECTED_AREA_SENSOR_FRONT;
		}
		if (InputTimeOne(IN_REAR_AREA_SENSOR_2ND, INOFF, TIME20MS) == true)
		{
			return DETECTED_AREA_SENSOR_REAR;
		}
	}

	return NO_ERR;
}

long GantryEventEnable()
{
	return gcPowerGantry->GantryEventEnable(FRONT_GANTRY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

//void BlinkingIO(bool blink)
//{
//	long Out = OUTOFF;
//	if (blink == true)
//	{
//		Out = OUTON;
//	}
//	OutputOne(OUT_STOP_PANEL_LED, Out);
//	OutputOne(OUT_FDOOR_LOCK_PANEL_LED, Out);
//	OutputOne(OUT_START_PANEL_LED, Out);
//	OutputOne(OUT_RESET_PANEL_LED, Out);
//	OutputOne(OUT_RDOOR_LOCK_PANEL_LED, Out);
//	OutputOne(OUT_FBUZZER, Out);
//	OutputOne(OUT_FLAMP_RED, Out);
//	OutputOne(OUT_FLAMP_YEL, Out);
//	OutputOne(OUT_FLAMP_GRN, Out);
//}

//////////////////////////////////////////////////////////////////////////////////////////////////

void gLedOn(int CameraNo, int iValue1, int iValue2, int iValue3)
{
	if (gcLedControl)
	{
		gcLedControl->LedOn(CameraNo, iValue1, iValue2, iValue3);
	}
}

void gLedAllOff()
{
	if (gcLedControl)
	{
		gcLedControl->LedAllOff();
	}
}

void gLedAllOn()
{
	if (gcLedControl)
	{
		gcLedControl->LedAllOn();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void gLaserOn(int CameraNo)
{
	if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
	{
		switch (CameraNo)
		{
		case CAM1:
		case CAM2:
		case CAM3:
			OutputOne(OUT_FCAM1_LASER_ON, OUTON);
			OutputOne(OUT_FCAM3_LASER_ON, OUTON);
			break;
		case CAM4:
		case CAM5:
		case CAM6:
			OutputOne(OUT_FCAM4_LASER_ON, OUTON);
			OutputOne(OUT_FCAM6_LASER_ON, OUTON);
			break;
		}
	}
	else
	{
		if (gCMachineConfig->GetOptionCameraLaser() == 1)
		{
			switch (CameraNo)
			{
			case CAM1:
				OutputOne(OUT_FCAM1_LASER_ON, OUTON);
				break;
			case CAM2:
				OutputOne(OUT_FCAM2_LASER_ON, OUTON);
				break;
			}
		}
		else
		{
			switch (CameraNo)
			{
			case CAM1:
				OutputOne(OUT_FCAM1_LASER_ON, OUTON);
				OutputOne(OUT_FCAM2_LASER_ON, OUTON);
				break;
			case CAM2:
				OutputOne(OUT_FCAM3_LASER_ON, OUTON);
				OutputOne(OUT_FCAM4_LASER_ON, OUTON);
				break;
			}
		}

	}
	if (GetCameraCount(REAR_STAGE) == CAMERA_COUNT_6)
	{
		switch (CameraNo)
		{
		case RCAM1:
		case RCAM2:
		case RCAM3:
			OutputOne(OUT_RCAM1_LASER_ON, OUTON);
			OutputOne(OUT_RCAM3_LASER_ON, OUTON);
			break;
		case RCAM4:
		case RCAM5:
		case RCAM6:
			OutputOne(OUT_RCAM4_LASER_ON, OUTON);
			OutputOne(OUT_RCAM6_LASER_ON, OUTON);
			break;
		}
	}
	else
	{
		if (gCMachineConfig->GetOptionCameraLaser() == 1)
		{
			switch (CameraNo)
			{
			case RCAM1:
				OutputOne(OUT_RCAM1_LASER_ON, OUTON);
				break;
			case RCAM2:
				OutputOne(OUT_RCAM2_LASER_ON, OUTON);
				break;
			default:
				break;
			}
		}
		else
		{
			switch (CameraNo)
			{
				// 20210617 HarkDo
			case RCAM1:
				OutputOne(OUT_RCAM1_LASER_ON, OUTON);
				OutputOne(OUT_RCAM2_LASER_ON, OUTON);
				break;
			case RCAM2:
				OutputOne(OUT_RCAM3_LASER_ON, OUTON);
				OutputOne(OUT_RCAM4_LASER_ON, OUTON);
				break;
			default:
				break;
			}
		}

	}
}

void gLaserOff(int CameraNo)
{
	if (GetCameraCount(FRONT_STAGE) == CAMERA_COUNT_6)
	{
		switch (CameraNo)
		{
		case CAM1:
		case CAM2:
		case CAM3:
			OutputOne(OUT_FCAM1_LASER_ON, OUTOFF);
			OutputOne(OUT_FCAM3_LASER_ON, OUTOFF);
			break;
		case CAM4:
		case CAM5:
		case CAM6:
			OutputOne(OUT_FCAM4_LASER_ON, OUTOFF);
			OutputOne(OUT_FCAM6_LASER_ON, OUTOFF);
			break;
		}
	}
	else
	{
		switch (CameraNo)
		{
		case CAM1:
			OutputOne(OUT_FCAM1_LASER_ON, OUTOFF);
			OutputOne(OUT_FCAM2_LASER_ON, OUTOFF);
			break;
		case CAM2:
			OutputOne(OUT_FCAM3_LASER_ON, OUTOFF);
			OutputOne(OUT_FCAM4_LASER_ON, OUTOFF);
			break;
		}
	}
	if (GetCameraCount(REAR_STAGE) == CAMERA_COUNT_6)
	{
		switch (CameraNo)
		{
		case RCAM1:
		case RCAM2:
		case RCAM3:
			OutputOne(OUT_RCAM1_LASER_ON, OUTOFF);
			OutputOne(OUT_RCAM3_LASER_ON, OUTOFF);
			break;
		case RCAM4:
		case RCAM5:
		case RCAM6:
			OutputOne(OUT_RCAM4_LASER_ON, OUTOFF);
			OutputOne(OUT_RCAM6_LASER_ON, OUTOFF);
			break;
		}
	}
	else
	{
		switch (CameraNo)
		{
			// 20210617 HarkDo
		case RCAM1:
			OutputOne(OUT_RCAM1_LASER_ON, OUTOFF);
			OutputOne(OUT_RCAM2_LASER_ON, OUTOFF);
			break;
		case RCAM2:
			OutputOne(OUT_RCAM3_LASER_ON, OUTOFF);
			OutputOne(OUT_RCAM4_LASER_ON, OUTOFF);
			break;
		default:
			break;
		}
	}
}

void gLaserAllOff()
{
	OutputOne(OUT_FCAM1_LASER_ON, OUTOFF);
	OutputOne(OUT_FCAM2_LASER_ON, OUTOFF);
	OutputOne(OUT_FCAM3_LASER_ON, OUTOFF);
	OutputOne(OUT_FCAM4_LASER_ON, OUTOFF);
	OutputOne(OUT_FCAM5_LASER_ON, OUTOFF);
	OutputOne(OUT_FCAM6_LASER_ON, OUTOFF);

	OutputOne(OUT_RCAM1_LASER_ON, OUTOFF);
	OutputOne(OUT_RCAM2_LASER_ON, OUTOFF);
	OutputOne(OUT_RCAM3_LASER_ON, OUTOFF);
	OutputOne(OUT_RCAM4_LASER_ON, OUTOFF);
	OutputOne(OUT_RCAM5_LASER_ON, OUTOFF);
	OutputOne(OUT_RCAM6_LASER_ON, OUTOFF);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void StartFeeder(long FeederNo, long ReadyIONo, long ReleaseIONo)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = FeederNo;
	SubMsg2 = ReadyIONo;
	SubMsg3 = ReleaseIONo;
	msgSend->SetThreadMsg(_T(STRING_FEEDER_START));
	TRACE(_T("[PWR] StartFeeder SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerFeederControl)
	{
		gcPowerFeederControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerFeederControl->PingThread(TIME1MS))
		{
			gcPowerFeederControl->Event((LPVOID)msgSend);
		}
	}
}

void SetInfoFeeder(long FeederNo, long ReadyIONo, long ReleaseIONo)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = FeederNo;
	SubMsg2 = ReadyIONo;
	msgSend->SetThreadMsg(_T(STRING_FEEDER_INFO));
	TRACE(_T("[PWR] SetInfoFeeder SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerFeederControl)
	{
		gcPowerFeederControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerFeederControl->PingThread(TIME1MS))
		{
			gcPowerFeederControl->Event((LPVOID)msgSend);
		}
	}
}

void RunFeeder(long FeederNo)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	CString strMsg;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = FeederNo;
	msgSend->SetThreadMsg(_T(STRING_FEEDER_RUN));
	TRACE(_T("[PWR] RunFeeder SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerFeederControl)
	{
		gcPowerFeederControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerFeederControl->PingThread(TIME1MS))
		{
			gcPowerFeederControl->Event((LPVOID)msgSend);
		}
	}
}

void StopFeeder(long FeederNo)
{
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	SubMsg1 = FeederNo;
	TRACE(_T("[PWR] StopFeeder SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	gcPowerFeederControl->StopFeederCtrl(SubMsg1, SubMsg2, SubMsg3);


	//PowerThreadMessage* msgSend = new PowerThreadMessage();
	//ThreadId_t id;
	//unsigned SubMsg1, SubMsg2, SubMsg3;
	//SubMsg1 = SubMsg2 = SubMsg3 = 0;
	//SubMsg1 = FeederNo;
	//msgSend->SetThreadMsg(_T(STRING_FEEDER_END));
	//TRACE(_T("[PWR] StopFeeder SendMsg1~3(%d,%d,%d)\n"), SubMsg1, SubMsg2, SubMsg3);
	//msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	//if (gcPowerFeederControl)
	//{
	//	gcPowerFeederControl->GetId(&id);
	//	msgSend->SetID(id);
	//	if (gcPowerFeederControl->PingThread(TIME1MS))
	//	{
	//		gcPowerFeederControl->Event((LPVOID)msgSend);
	//	}
	//}
}

CFeeder* GetFeeder(long FeederNo)
{
	CFeeder* pFeeder = NULL;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		pFeeder = gcFeeder[FeederNo - 1];
	}
	return pFeeder;
}

void SetFeederProdRunMode(long RunMode)
{
	if (gcPowerFeederControl)
	{
		gcPowerFeederControl->SetProdRunMode(RunMode);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void ClearY2Shift(long Gantry)
{
	double ZeroHomeShift = 0.0;
	if(Gantry == FRONT_GANTRY)
		gcPowerCalibrationData->SetHomeShiftDistance(FRONT_GANTRY, ZeroHomeShift);
	else if (Gantry == REAR_GANTRY)
		gcPowerCalibrationData->SetHomeShiftDistance(REAR_GANTRY, ZeroHomeShift);
	gcPowerCalibrationData->WriteHomeShiftDistance(FRONT_GANTRY);
}

void ClearCameraRecognitionPosition(long Gantry)
{
	Point_XY pt;
	gcPowerGantry->InitializeRearCamPosMech();
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		pt = gcPowerGantry->GetRearCameraRecognitionPosition(HeadNo + 1);
		gcPowerCalibrationData->SetCameraRecognitionPosition(REAR_GANTRY, HeadNo + 1, pt);
	}
	gcPowerCalibrationData->WriteCameraRecognitionPosition(FRONT_GANTRY);
}

void ClearCameraRecognitionOffset(long Gantry)
{
	Point_XY pt;
	gcPowerGantry->InitializeRearCamOffsetMech();
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		pt = gcPowerGantry->GetRearCameraRecognitionOffset(HeadNo + 1);
		gcPowerCalibrationData->SetCameraRecognitionOffset(REAR_GANTRY, HeadNo + 1, pt);
	}
	gcPowerCalibrationData->WriteCameraRecognitionOffset(FRONT_GANTRY);
}

Limit GetLimit(long AxisNo)
{
	Limit limit;
	limit = gcPowerCalibrationData->GetLimit(AxisNo);
	return limit;
}

long WriteHeadOffset(long Gantry)
{
	gcPowerCalibrationData->WriteHeadOffset(FRONT_GANTRY);
	return NO_ERR;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void HeightMeasurementControl(bool On)
{
	if (On == true)
	{
		OutputOne(OUT_FHEAD_ZHMD_OFF, OUTOFF);
	}
	else
	{
		OutputOne(OUT_FHEAD_ZHMD_OFF, OUTON);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void MotorPowerControl(bool On)
{
	if (GetOnlyConveyorMode() == true)
	{
		OutputOne(OUT_MOTOR_POWER, OUTOFF);
		return;
	}

	if (On == true)
	{
		OutputOne(OUT_MOTOR_POWER, OUTON);
	}
	else
	{
		OutputOne(OUT_MOTOR_POWER, OUTOFF);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void DoorLockingControl(bool Lock)
{
	if (Lock == true)
	{
		OutputOne(OUT_FDOOR_KEY_LOCK, OUTON);
		OutputOne(OUT_RDOOR_KEY_LOCK, OUTON);
	}
	else
	{
		OutputOne(OUT_FDOOR_KEY_LOCK, OUTOFF);
		OutputOne(OUT_RDOOR_KEY_LOCK, OUTOFF);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void TowerLampRed(bool On)
{
	if(On == true)
		OutputOne(OUT_FLAMP_RED, OUTON);
	else
		OutputOne(OUT_FLAMP_RED, OUTOFF);
}

void TowerLampYel(bool On)
{
	if (On == true)
		OutputOne(OUT_FLAMP_YEL, OUTON);
	else
		OutputOne(OUT_FLAMP_YEL, OUTOFF);
}

void TowerLampGrn(bool On)
{
	if (On == true)
		OutputOne(OUT_FLAMP_GRN, OUTON);
	else
		OutputOne(OUT_FLAMP_GRN, OUTOFF);
}
long GetTowerLampRed()
{
	return ReadOutputOne(OUT_FLAMP_RED);
}
long GetTowerLampYel()
{
	return ReadOutputOne(OUT_FLAMP_YEL);
}
long GetTowerLampGrn()
{
	return ReadOutputOne(OUT_FLAMP_GRN);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void gQuitMachine()
{
	TRACE(_T("[PWR] QuitMachine Start\n"));

	gLedAllOff();
	gLaserAllOff();

	Io_state_detector::GetInstance().Stop();

    if (gcTrayDumpBox)
    {
        delete gcTrayDumpBox;
    }
    TRACE(_T("[PWR] ~QuitMachine delete gcTrayDumpBox done\n"));

	if (gcMachineInformation)
	{
		gcMachineInformation->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcMachineInformation done\n"));

	if (gcCollisionMonitoring)
	{
		gcCollisionMonitoring->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcCollisionMonitoring done\n"));

	if (gcPowerSwitchPanel)
	{
		gcPowerSwitchPanel->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerSwitchPanel done\n"));


	if (gcPowerBuzzer)
	{
		gcPowerBuzzer->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerBuzzer done\n"));
	if (gcAnalogStatus)
	{
		gcAnalogStatus->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcAnalogStatus done\n"));
	if (gcIOStatus)
	{
		gcIOStatus->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcIOStatus done\n"));
	if (gcMasterMotion)
	{
		gcMasterMotion->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcMasterMotion done\n"));
	if (gcSlaveMotorStatus)
	{
		gcSlaveMotorStatus->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcSlaveMotorStatus done\n"));
	if (gcwmx3IO)
	{
		gcwmx3IO->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcwmx3IO done\n"));
	if (gcWmx3Motor)
	{
		gcWmx3Motor->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcWmx3Motor done\n"));
	if (gcMotorPower)
	{
		gcMotorPower->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcMotorPower done\n"));

	if (gcPcbExist)
	{
		gcPcbExist->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPcbExist done\n"));

	if (gcSmemaControl)
	{
		gcSmemaControl->ExitThreadLoop();
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcSmemaControl done\n"));

	ThreadSleep(TIME100MS);

	if (gcCAutoNozzleChange)
	{
		delete gcCAutoNozzleChange;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcCAutoNozzleChange done\n"));

	if (gcLedControl)
	{
		delete gcLedControl;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcLedControl done\n"));
	if (gcPowerTeachBox)
	{
		delete gcPowerTeachBox;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerTeachBox done\n"));
	if (gcPowerConveyorControl)
	{
		delete gcPowerConveyorControl;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerConveyorControl done\n"));
	if (gcPowerConveyorManualControl)
	{
		delete gcPowerConveyorManualControl;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerConveyorManualControl done\n"));
	if (gcPowerFeederControl)
	{
		delete gcPowerFeederControl;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerFeederControl done\n"));

	if (gcPowerConveyorData)
	{
		delete gcPowerConveyorData;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerConveyorData done\n"));
	if (gcMachineFile)
	{
		delete gcMachineFile;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcMachineFile done\n"));
	if (gcRunFile)
	{
		delete gcRunFile;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcRunFile done\n"));
	if (gcInsertEndFile)
	{
		delete gcInsertEndFile;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcInsertEndFile done\n"));
	//if (gcPowerVision)
	//{
	//	delete gcPowerVision;
	//}
	//TRACE(_T("[PWR] ~QuitMachine delete gcPowerVision done\n"));
	//if (gcEthernetVision)
	//{
	//	delete gcEthernetVision;
	//}
	//TRACE(_T("[PWR] ~QuitMachine delete gcEthernetVision done\n"));

	//if (gcTowerLamp)
	//{
	//	delete gcTowerLamp;
	//}
	//TRACE(_T("[PWR] ~QuitMachine delete gcTowerLamp done\n"));

	if (gcPowerMainControl)
	{
		delete gcPowerMainControl;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerMainControl done\n"));

	if (gcWmx3Init)
	{
		delete gcWmx3Init;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcWmx3Init done\n"));
	//if (gcPowerCleaner)
	//{
	//	delete gcPowerCleaner;
	//}
	//TRACE(_T("[PWR] ~QuitMachine delete gcPowerCleaner done\n"));

	if (gcPowerCalibrationData)
	{
		delete gcPowerCalibrationData;
	}
	TRACE(_T("[PWR] ~QuitMachine delete gcPowerCalibrationData done\n"));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void TowerLampMsg(long SubMsg1, long SubMsg2, long SubMsg3, CString strMsg)
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned nSubMsg1, nSubMsg2, nSubMsg3;
	nSubMsg1 = SubMsg1;
	nSubMsg2 = SubMsg2;
	nSubMsg3 = SubMsg3;
	msgSend->SetThreadMsg(strMsg);
	msgSend->SetThreadSubMsg(nSubMsg1, nSubMsg2, nSubMsg3);
	if (gcTowerLamp)
	{
		gcTowerLamp->GetId(&id);
		msgSend->SetID(id);
		if (gcTowerLamp->PingThread(TIME1MS))
		{
			gcTowerLamp->Event((LPVOID)msgSend);
		}
	}
}

void TowerLampAlarm(long AlarmCode)
{
	TowerLampMsg(LAMP_ALARM, 0, AlarmCode, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampEmpty(long AlarmCode)
{
	long EmptyBuzzerTime = GetGlobalEmptyBuzzerTime();
	BuzzerOn(EmptyBuzzerTime); // 10 Sec
	TowerLampMsg(LAMP_EMPTY, 0, AlarmCode, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampWaitPcb(long Conv, long AlarmCode)
{
	TowerLampMsg(LAMP_WAIT_PCB, Conv, AlarmCode, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampInPcb()
{
	TowerLampMsg(LAMP_IN_PCB, 0, NO_ERR, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampOutPcb()
{
	TowerLampMsg(LAMP_OUT_PCB, 0, NO_ERR, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampRun()
{
	BuzzerOff();
	TowerLampMsg(LAMP_RUN, 0, 0, _T(STRING_TOWERLAMP_CONTROL));
}

void TowerLampNormal()
{
	BuzzerOff();
	TowerLampMsg(LAMP_NORMAL, 0, 0, _T(STRING_TOWERLAMP_CONTROL));
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void ShowSmemaIO()
{
	if (gcSmemaControl)
	{
		gcSmemaControl->ShowSmemaIO();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void ShowPCBSensor()
{
	if (gcPcbExist)
	{
		gcPcbExist->ShowPCBSensor();
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void ShowSuctionIO()
{
	UBYTE ucStatus = OUTOFF;
	long OutputIO = IO_NOUSE;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		switch (HeadNo)
		{
		case 0:
			OutputIO = OUT_FHEAD1_SUC;
			break;
		case 1:
			OutputIO = OUT_FHEAD2_SUC;
			break;
		case 2:
			OutputIO = OUT_FHEAD3_SUC;
			break;
		case 3:
			OutputIO = OUT_FHEAD4_SUC;
			break;
		case 4:
			OutputIO = OUT_FHEAD5_SUC;
			break;
		case 5:
			OutputIO = OUT_FHEAD6_SUC;
			break;
		}
		if (OutputIO != IO_NOUSE)
		{
			ucStatus = ReadOutputOne(OutputIO);
			TRACE(_T("[PWR] ShowSuctionIO HeadNo:%d Status:%d\n"), HeadNo + 1, ucStatus);
		}
	}
}

void ShowBlowIO()
{
	UBYTE ucStatus = OUTOFF;
	long OutputIO = IO_NOUSE;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		switch (HeadNo)
		{
		case 0:
			OutputIO = OUT_FHEAD1_BLO;
			break;
		case 1:
			OutputIO = OUT_FHEAD2_BLO;
			break;
		case 2:
			OutputIO = OUT_FHEAD3_BLO;
			break;
		case 3:
			OutputIO = OUT_FHEAD4_BLO;
			break;
		case 4:
			OutputIO = OUT_FHEAD5_BLO;
			break;
		case 5:
			OutputIO = OUT_FHEAD6_BLO;
			break;
		}
		if (OutputIO != IO_NOUSE)
		{
			ucStatus = ReadOutputOne(OutputIO);
			TRACE(_T("[PWR] ShowBlowIO HeadNo:%d Status:%d\n"), HeadNo + 1, ucStatus);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long MoveZStandy(long Gantry, double StandByZ, double Ratio)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	NOZZLE Nozzle;
	double InposXY = 0.010, InposZ = 0.100, TargetZ, Ratio2nd = 0.1;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		NozzleNo = GetGlobalNozzleNo(HeadNo + TBL_HEAD1);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		if ((StandByZ - Nozzle.TipHeight) > StandByZ)
		{
			TargetZ = StandByZ;
		}
		else
		{
			TargetZ = StandByZ - Nozzle.TipHeight;
		}

		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > TargetZ)
		{
			TRACE(_T("[PWR] MoveZStandy Z Standby position(%.3f) is under minus limit(%.3f)\n"), TargetZ, limit.minus);
			TargetZ = limit.minus + 1.0;
		}

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveStandBy %s NozzleNo:%d Target:%.3f StandByZ:%.3f Height Tip,%.3f Pusher,%.3f\n"),
				GetZAxisByIndex(HeadNo), NozzleNo, TargetZ, StandByZ, Nozzle.TipHeight, Nozzle.PusherHeight);
		}
		Err = StartPosWaitInposition(GetZAxisByIndex(HeadNo), Ratio, TimeOut, TargetZ, InposZ, Ms, false);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("StartPosWaitInposition WaitZStandy"));
			return Err;
		}
	}

	Err = WaitZStandy(Gantry, StandByZ);

	return Err;
}

long MoveZStandyOneToJig(long Gantry, long Target, double StandByZ, double Ratio)
{
    if (Target == 0)
    {
        TRACE(_T("[PWR] MoveZStandyOneToJig invalid target:%d\n"), Target);

        return NO_ERR;
    }

    long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
    NOZZLE Nozzle;
    double InposXY = 0.010, InposZ = 0.100, TargetZ, Ratio2nd = 0.1;
    long HeadNo = Target - 1;

    NozzleNo = GetGlobalNozzleNo(Target);
    Nozzle = GetGlobalNozzleInformation(NozzleNo);
    if ((StandByZ - Nozzle.TipHeight) > StandByZ)
    {
        TargetZ = StandByZ;
    }
    else
    {
        TargetZ = StandByZ - Nozzle.TipHeight + Nozzle.PusherHeight;
    }

    CString strZAxis = GetZAxisFromHeadNo(Gantry, Target);
    Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
    if (limit.minus > TargetZ)
    {
        TRACE(_T("[PWR] MoveZStandyOneToJig Z Standby position(%.3f) is under minus limit(%.3f)\n"), TargetZ, limit.minus);
        TargetZ = limit.minus + 1.0;
    }

    if (gcPowerLog->IsShowRunLog() == true)
    {
        TRACE(_T("[PWR] MoveZStandyOneToJig %s NozzleNo:%d Target:%.3f StandByZ:%.3f Height Tip,%.3f Pusher,%.3f\n"),
              GetZAxisByIndex(HeadNo), NozzleNo, TargetZ, StandByZ, Nozzle.TipHeight, Nozzle.PusherHeight);
    }
    Err = StartPosWaitInposition(GetZAxisByIndex(HeadNo), Ratio, TimeOut, TargetZ, InposZ, Ms, true);
    if (Err != NO_ERR)
    {
        Err = SendAlarm(Err, _T("StartPosWaitInposition WaitZStandy"));
        return Err;
    }

    return Err;
}

long MoveZStandySkipServoOff(long Gantry, double StandByZ, double Ratio)
{
	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	NOZZLE Nozzle;
	double InposXY = 0.010, InposZ = 0.100, TargetZ, Ratio2nd = 0.1;
	for (long HeadNo = 0; HeadNo < GetZAxisCount(); ++HeadNo)
	{
		NozzleNo = GetGlobalNozzleNo(HeadNo + TBL_HEAD1);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		if ((StandByZ - Nozzle.TipHeight) > StandByZ)
		{
			TargetZ = StandByZ;
		}
		else
		{
			TargetZ = StandByZ - Nozzle.TipHeight;// +Nozzle.PusherHeight;
		}

		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);

		if (CheckServoOn(strZAxis) == false)
		{
			TRACE(_T("[PWR] MoveZStandySkipServoOff Skip. %s Servo Off\n"), strZAxis);
			continue;
		}

		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > TargetZ)
		{
			TRACE(_T("[PWR] MoveZStandySkipServoOff Z Standby position(%.3f) is under minus limit(%.3f)\n"), TargetZ, limit.minus);
			TargetZ = limit.minus + 1.0;
		}

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveZStandySkipServoOff %s NozzleNo:%d Target:%.3f StandByZ:%.3f Height Tip,%.3f Pusher,%.3f\n"),
				GetZAxisByIndex(HeadNo), NozzleNo, TargetZ, StandByZ, Nozzle.TipHeight, Nozzle.PusherHeight);
		}
		Err = StartPosWaitInposition(GetZAxisByIndex(HeadNo), Ratio, TimeOut, TargetZ, InposZ, Ms, false);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("StartPosWaitInposition WaitZStandy"));
			//return Err;
		}
	}
	return Err;
}
long WaitZStandy(long Gantry, double StandByZ)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	NOZZLE Nozzle;
	double InposZ = 0.100, TargetZ = GetStandByZ(Gantry);
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		NozzleNo = GetGlobalNozzleNo(HeadNo + TBL_HEAD1);
		Nozzle = GetGlobalNozzleInformation(NozzleNo);
		if ((StandByZ - Nozzle.TipHeight) > StandByZ)
		{
			TargetZ = StandByZ;
		}
		else
		{
			TargetZ = StandByZ - Nozzle.TipHeight;
		}

		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		if (limit.minus > TargetZ)
		{
			TRACE(_T("[PWR] MoveZStandy Z Standby position(%.3f) is under minus limit(%.3f)\n"), TargetZ, limit.minus);
			TargetZ = limit.minus + 1.0;
		}

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] WaitZStandy %s NozzleNo:%d Target:%.3f StandByZ:%.3f TipHeight:%.3f\n"),
				GetZAxisByIndex(HeadNo), NozzleNo, TargetZ, StandByZ, Nozzle.TipHeight);
		}
		Err = WaitOneInPos(GetZAxisByIndex(HeadNo), TargetZ, InposZ, TimeOut);
		InitOneRatio(GetZAxisByIndex(HeadNo));
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("WaitOneInPos WaitZStandy"));
			return Err;
		}
	}
	return Err;
}

long MoveRStandy(long Gantry, double TargetR, double Ratio)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	double InposR = 3.0;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveStandBy %s StandByR:%.3f\n"), GetRAxisByIndex(HeadNo), TargetR);
		}
		Err = StartPosWaitInposition(GetRAxisByIndex(HeadNo), Ratio, TimeOut, TargetR, InposR, Ms, false);
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("StartPosWaitInposition MoveRStandy"));
			return Err;
		}
	}
	return Err;
}

long WaitRStandBy(long Gantry, double TargetR)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	double Ratio = 1.0, InposR = 3.0;
	for (long HeadNo = 0; HeadNo < MAXUSEDHEADNO; ++HeadNo)
	{
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveStandBy %s StandByR:%.3f\n"), GetRAxisByIndex(HeadNo), TargetR);
		}
		Err = WaitOneInPos(GetRAxisByIndex(HeadNo), TargetR, InposR, TimeOut);
		InitOneRatio(GetRAxisByIndex(HeadNo));
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("WaitOneInPos WaitRStandBy"));
			return Err;
		}
	}
	return Err;
}

long MoveStandByXY(long Gantry, long Target, Point_XY NewPt, double Ratio, long TimeOut)
{
	if (IsAccTest() == true)
	{
		return NO_ERR;

	}

	long Err = NO_ERR;
	Point_XY CurPt;
	CurPt = gReadGantryPosition(Gantry);
	if (GetUsePathLinearIntpl() == 1)
	{
		if ((NewPt.y - CurPt.y) > 500.0)
		{
			Point_XY Linear, StartCenter, EndCenter, Goal;
			StartCenter = GetCameraRecognitionPosition(FRONT_STAGE, TBL_HEAD1);
			EndCenter = GetCameraRecognitionPosition(FRONT_STAGE, TBL_HEAD1);
			Linear.x = StartCenter.x - 300.0;
			Linear.y = 0.0;
			StartCenter.x = StartCenter.x - 300.0;
			Goal = NewPt;
			TRACE(_T("[PWR] MoveStandByXY LinearXY,%.3f,%.3f CenterXY Start,%.3f,%.3f End,%.3f,%.3f GoalXY,%.3f,%.3f\n"),
				Linear.x, Linear.y, StartCenter.x, StartCenter.y, EndCenter.x, EndCenter.y, Goal.x, Goal.y);
			Err = LinearPathIntplPos(Gantry, Target, Linear, StartCenter, EndCenter, Goal, Ratio, TimeOut);
		}
		else
		{
			Err = LinearIntplPosWaitMotion(Gantry, Target, NewPt, Ratio, TimeOut);
		}
	}
	else
	{
		Err = LinearIntplPosWaitMotion(Gantry, Target, NewPt, Ratio, TimeOut);
	}
	return Err;
}

long MoveZAllUpLimitWithOutOneHead(long Gantry, double Ratio, CString WithoutHead)
{
	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	double InposXY = 0.010, InposZ = 0.100, TargetZ;

	for (long HeadNo = 0; HeadNo < GetZAxisCount(); ++HeadNo)
	{
		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		TargetZ = limit.minus + 1.0;

		if (WithoutHead.CompareNoCase(strZAxis) == 0)
		{
			TRACE(_T("[PWR] MoveZAllUpLimitWithOutOneHead %s skip \n"), WithoutHead);
			continue;
		}

		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveZAllUpLimit %s Target:%.3f \n"), GetZAxisByIndex(HeadNo), TargetZ);
		}

		Err = StartPosWaitInposition(GetZAxisByIndex(HeadNo), Ratio, TimeOut, TargetZ, InposZ, Ms, false);
		if (Err != NO_ERR)
		{
			//SendAlarm(Err, _T("MoveZAllUpLimit Err"));
			return Err;
		}
	}
	return Err;
}

long MoveZAllUpLimit(long Gantry, double Ratio)
{
	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS, TimeOut = TIME5000MS;
	double InposXY = 0.010, InposZ = 0.100, TargetZ;

	if (IsAccTest() == true)
	{
		return NO_ERR;
	}

	for (long HeadNo = 0; HeadNo < GetZAxisCount(); ++HeadNo)
	{
		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		TargetZ = limit.minus + 1.0;
		if (gcPowerLog->IsShowRunLog() == true)
		{
			TRACE(_T("[PWR] MoveZAllUpLimit %s Target:%.3f \n"), GetZAxisByIndex(HeadNo), TargetZ);
		}

		Err = StartPosWaitInposition(GetZAxisByIndex(HeadNo), Ratio, TimeOut, TargetZ, InposZ, Ms, false);
		if (Err != NO_ERR)
		{
			SendAlarm(Err, _T("MoveZAllUpLimit Err"));
			return Err;
		}
	}
	return Err;
}

long MoveZAllUpLimitWait(long Gantry, long TimeOut)
{
	long Err = NO_ERR, NozzleNo = 0, Ms = TIME30MS;
	double InposZ = 0.100, TargetZ = GetStandByZ(Gantry);
	for (long HeadNo = 0; HeadNo < GetZAxisCount(); ++HeadNo)
	{
		CString strZAxis = GetZAxisFromHeadNo(Gantry, HeadNo + 1);
		Limit limit = GetLimit(GetAxisIndexFromAliasName(strZAxis));
		TargetZ = limit.minus + 1.0;

		if (gcPowerLog->IsShowRunLog() == true)
		{
			CString str(__func__);
			TRACE(_T("[PWR] %s %s Target:%.3f \n"), str, strZAxis, TargetZ);
		}

		Err = WaitOneInPos(GetZAxisByIndex(HeadNo), TargetZ, InposZ, TimeOut);
		InitOneRatio(GetZAxisByIndex(HeadNo));
		if (Err != NO_ERR)
		{
			Err = SendAlarm(Err, _T("WaitOneInPos WaitZStandy"));
			return Err;
		}
	}
	return Err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/*
	BuzzerOnTime - 0 is infinite, other is On time(sec)
*/
void BuzzerOn(long BuzzerOnTime)
{
	long BuzzerOn = BuzzerOnTime * TIME1000MS;
	gcPowerBuzzer->SetBuzzerOnTime(BuzzerOn);
	gcPowerBuzzer->SetStep(BuzzerStep::ON);
}

void BuzzerOff()
{
    const int lastSafetyAlarmCode = gcPowerBuzzer->GetLastSafetyAlarmCode();
    if (lastSafetyAlarmCode != NO_ERR && lastSafetyAlarmCode != OPEN_DOOR)
    {
        TRACE_FILE_FUNC_LINE_"skipped due to (gcPowerBuzzer->GetLastSafetyAlarmCode()(= %d) != NO_ERR).", lastSafetyAlarmCode);
        return;
    }
	gcPowerBuzzer->SetStep(BuzzerStep::OFF);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long GantryMoveStandByZ(long Gantry, double Ratio)
{
	long Err = NO_ERR;
	Err = MoveZStandy(Gantry, GetStandByZ(Gantry), Ratio);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] GantryMoveStandByZ(%d) MoveZStandy Ratio:%.1f Err:%d\n"), Gantry, Ratio, Err);
	}
	return Err;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
long SetDefaultTorqueLimitEvent(long Gantry)
{
	//if (Gantry == FRONT_GANTRY)
	//{
	//	SetEventToStopByOverTorque(EVENTID_FZ1_OVER_TORQUE, _T("FZ1"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_FZ2_OVER_TORQUE, _T("FZ2"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_FZ3_OVER_TORQUE, _T("FZ3"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_FZ4_OVER_TORQUE, _T("FZ4"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_FZ5_OVER_TORQUE, _T("FZ5"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_FZ6_OVER_TORQUE, _T("FZ6"), GetMaxZTorqueLimit(FRONT_GANTRY), GetTorqueOverSafetyZ());
	//}
	//else
	//{
	//	SetEventToStopByOverTorque(EVENTID_RZ1_OVER_TORQUE, _T("RZ1"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_RZ2_OVER_TORQUE, _T("RZ2"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_RZ3_OVER_TORQUE, _T("RZ3"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_RZ4_OVER_TORQUE, _T("RZ4"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_RZ5_OVER_TORQUE, _T("RZ5"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//	SetEventToStopByOverTorque(EVENTID_RZ6_OVER_TORQUE, _T("RZ6"), GetMaxZTorqueLimit(REAR_GANTRY), GetTorqueOverSafetyZ());
	//}

	//SetEventToStopByOverTorque(EVENTID_FZ1_OVER_TORQUE, _T("FZ1"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD1), GetTorqueOverSafetyZ());
	//SetEventToStopByOverTorque(EVENTID_FZ2_OVER_TORQUE, _T("FZ2"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD2), GetTorqueOverSafetyZ());
	//SetEventToStopByOverTorque(EVENTID_FZ3_OVER_TORQUE, _T("FZ3"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD3), GetTorqueOverSafetyZ());
	//SetEventToStopByOverTorque(EVENTID_FZ4_OVER_TORQUE, _T("FZ4"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD4), GetTorqueOverSafetyZ());
	//SetEventToStopByOverTorque(EVENTID_FZ5_OVER_TORQUE, _T("FZ5"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD5), GetTorqueOverSafetyZ());
	//SetEventToStopByOverTorque(EVENTID_FZ6_OVER_TORQUE, _T("FZ6"), GetMaxZTorqueLimit(FRONT_GANTRY, TBL_HEAD6), GetTorqueOverSafetyZ());

	for (long head = TBL_HEAD1; head <= GetZAxisCount(); head++)
	{
		RemoveSetHeadTorqueLimitEvent(Gantry, head, GetMaxZTorqueLimit(Gantry, head), GetTorqueOverSafetyZ());
	}

	return NO_ERR;
}

long SetDefaultAreaSensorEvent()
{
	if (GetUseAreaSensor() == 1)
	{
		SetEventToStopByAreaSensor(EVENTID_FX_FRONT_AREA_SENSOR, IN_FRONT_AREA_SENSOR, GetAxisX(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FY1_FRONT_AREA_SENSOR, IN_FRONT_AREA_SENSOR, GetAxisY1(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FX_REAR_AREA_SENSOR, IN_REAR_AREA_SENSOR, GetAxisX(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FY1_REAR_AREA_SENSOR, IN_REAR_AREA_SENSOR, GetAxisY1(FRONT_GANTRY));
	}

	if (GetUseAreaSensor2nd() == 1)
	{
		SetEventToStopByAreaSensor(EVENTID_FX_FRONT_AREA_SENSOR_2ND, IN_FRONT_AREA_SENSOR_2ND, GetAxisX(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FY1_FRONT_AREA_SENSOR_2ND, IN_FRONT_AREA_SENSOR_2ND, GetAxisY1(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FX_REAR_AREA_SENSOR_2ND, IN_REAR_AREA_SENSOR_2ND, GetAxisX(FRONT_GANTRY));
		SetEventToStopByAreaSensor(EVENTID_FY1_REAR_AREA_SENSOR_2ND, IN_REAR_AREA_SENSOR_2ND, GetAxisY1(FRONT_GANTRY));
	}

	return NO_ERR;
}

long RemoveHeadTorqueLimitEvent(long Gantry, long HeadNo)
{
	long EventID = GetMaxEventControl(), Err = NO_ERR;
	if (Gantry == FRONT_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_FZ1_OVER_TORQUE; break;
		case TBL_HEAD2: EventID = EVENTID_FZ2_OVER_TORQUE; break;
		case TBL_HEAD3: EventID = EVENTID_FZ3_OVER_TORQUE; break;
		case TBL_HEAD4: EventID = EVENTID_FZ4_OVER_TORQUE; break;
		case TBL_HEAD5: EventID = EVENTID_FZ5_OVER_TORQUE; break;
		case TBL_HEAD6: EventID = EVENTID_FZ6_OVER_TORQUE; break;
		default:
			TRACE(_T("[PWR] RemoveHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = RemoveEvent(EventID);

			if (Err == ErrorCode::None)
			{
				TRACE(_T("[PWR] RemoveHeadTorqueLimitEvent Gantry:%d HeadNo:%d Complete\n"), Gantry, HeadNo);
			}

		}
		else
		{
			Err = EventID;
		}
	}
	else
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_RZ1_OVER_TORQUE; break;
		case TBL_HEAD2: EventID = EVENTID_RZ2_OVER_TORQUE; break;
		case TBL_HEAD3: EventID = EVENTID_RZ3_OVER_TORQUE; break;
		case TBL_HEAD4: EventID = EVENTID_RZ4_OVER_TORQUE; break;
		case TBL_HEAD5: EventID = EVENTID_RZ5_OVER_TORQUE; break;
		case TBL_HEAD6: EventID = EVENTID_RZ6_OVER_TORQUE; break;
		default:
			TRACE(_T("[PWR] RemoveHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = RemoveEvent(EventID);
			if (Err == ErrorCode::None)
			{
				TRACE(_T("[PWR] RemoveHeadTorqueLimitEvent Gantry:%d HeadNo:%d Complete\n"), Gantry, HeadNo);
			}
		}
		else
		{
			Err = EventID;
		}
	}
	return Err;
}

long SetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit)
{
	long EventID = GetMaxEventControl(), Err = NO_ERR;
	CString strZAxis = _T("NON");
	if (Gantry == FRONT_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_FZ1_OVER_TORQUE; strZAxis.Format(_T("FZ1")); break;
		case TBL_HEAD2: EventID = EVENTID_FZ2_OVER_TORQUE; strZAxis.Format(_T("FZ2")); break;
		case TBL_HEAD3: EventID = EVENTID_FZ3_OVER_TORQUE; strZAxis.Format(_T("FZ3")); break;
		case TBL_HEAD4: EventID = EVENTID_FZ4_OVER_TORQUE; strZAxis.Format(_T("FZ4")); break;
		case TBL_HEAD5: EventID = EVENTID_FZ5_OVER_TORQUE; strZAxis.Format(_T("FZ5")); break;
		case TBL_HEAD6: EventID = EVENTID_FZ6_OVER_TORQUE; strZAxis.Format(_T("FZ6")); break;
		default:
			TRACE(_T("[PWR] SetHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = SetEventToStopByOverTorque(EventID, strZAxis, TorqueLimit, GetTorqueOverSafetyZ());
		}
		else
		{
			Err = EventID;
		}
	}
	else
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_RZ1_OVER_TORQUE; strZAxis.Format(_T("RZ1")); break;
		case TBL_HEAD2: EventID = EVENTID_RZ2_OVER_TORQUE; strZAxis.Format(_T("RZ2")); break;
		case TBL_HEAD3: EventID = EVENTID_RZ3_OVER_TORQUE; strZAxis.Format(_T("RZ3")); break;
		case TBL_HEAD4: EventID = EVENTID_RZ4_OVER_TORQUE; strZAxis.Format(_T("RZ4")); break;
		case TBL_HEAD5: EventID = EVENTID_RZ5_OVER_TORQUE; strZAxis.Format(_T("RZ5")); break;
		case TBL_HEAD6: EventID = EVENTID_RZ6_OVER_TORQUE; strZAxis.Format(_T("RZ6")); break;
		default:
			TRACE(_T("[PWR] SetHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = SetEventToStopByOverTorque(EventID, strZAxis, TorqueLimit, GetTorqueOverSafetyZ());
		}
		else
		{
			Err = EventID;
		}
	}
	return Err;
}

long SetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit, double SafetyPosition)
{
	long EventID = GetMaxEventControl(), Err = NO_ERR;
	CString strZAxis = _T("NON");
	if (Gantry == FRONT_GANTRY)
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_FZ1_OVER_TORQUE; strZAxis.Format(_T("FZ1")); break;
		case TBL_HEAD2: EventID = EVENTID_FZ2_OVER_TORQUE; strZAxis.Format(_T("FZ2")); break;
		case TBL_HEAD3: EventID = EVENTID_FZ3_OVER_TORQUE; strZAxis.Format(_T("FZ3")); break;
		case TBL_HEAD4: EventID = EVENTID_FZ4_OVER_TORQUE; strZAxis.Format(_T("FZ4")); break;
		case TBL_HEAD5: EventID = EVENTID_FZ5_OVER_TORQUE; strZAxis.Format(_T("FZ5")); break;
		case TBL_HEAD6: EventID = EVENTID_FZ6_OVER_TORQUE; strZAxis.Format(_T("FZ6")); break;
		default:
			TRACE(_T("[PWR] SetHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = SetEventToStopByOverTorque(EventID, strZAxis, TorqueLimit, SafetyPosition);
		}
		else
		{
			Err = EventID;
		}
	}
	else
	{
		switch (HeadNo)
		{
		case TBL_HEAD1: EventID = EVENTID_RZ1_OVER_TORQUE; strZAxis.Format(_T("RZ1")); break;
		case TBL_HEAD2: EventID = EVENTID_RZ2_OVER_TORQUE; strZAxis.Format(_T("RZ2")); break;
		case TBL_HEAD3: EventID = EVENTID_RZ3_OVER_TORQUE; strZAxis.Format(_T("RZ3")); break;
		case TBL_HEAD4: EventID = EVENTID_RZ4_OVER_TORQUE; strZAxis.Format(_T("RZ4")); break;
		case TBL_HEAD5: EventID = EVENTID_RZ5_OVER_TORQUE; strZAxis.Format(_T("RZ5")); break;
		case TBL_HEAD6: EventID = EVENTID_RZ6_OVER_TORQUE; strZAxis.Format(_T("RZ6")); break;
		default:
			TRACE(_T("[PWR] SetHeadTorqueLimitEvent Gantry:%d Invalid HeadNo:%d\n"), Gantry, HeadNo);
			break;
		}
		if (EventID < GetMaxEventControl())
		{
			Err = SetEventToStopByOverTorque(EventID, strZAxis, TorqueLimit, SafetyPosition);
		}
		else
		{
			Err = EventID;
		}
	}
	return Err;
}

long RemoveSetHeadTorqueLimitEvent(long Gantry, long HeadNo, double TorqueLimit, double SafetyPosition)
{
	RemoveHeadTorqueLimitEvent(Gantry, HeadNo);
	SetHeadTorqueLimitEvent(Gantry, HeadNo, TorqueLimit, SafetyPosition);

	return 0;
}

long InitialHeadTorqueLimit(long Gantry, long HeadNo)
{
	long Err = NO_ERR;
	Err = RemoveHeadTorqueLimitEvent(Gantry, HeadNo);
	if (Err != NO_ERR)
	{
		return Err;
	}
	Err = SetHeadTorqueLimitEvent(Gantry, HeadNo, GetMaxZTorqueLimit(Gantry, HeadNo));
	if (Err != NO_ERR)
	{
		return Err;
	}
	return Err;
}


void MoveOneTimeUnlock()
{
	ReleaseSemaphore(gRUN_MOVEONETIME, 1, NULL);
}
//
//bool StepMoveLock()
//{
//	SEM_LOCK(gRUN_MOVEONETIME, INFINITE);
//
//	return true;
//}

bool MoveOneTimeLockWait(long WaitTime)
{
	bool bRet = false;
	DWORD ret;

	ret = WaitForSingleObject(gRUN_MOVEONETIME, WaitTime);

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

long MoveOnceLockPauseMode(CString strAxis)
{
	long mode = GetRunModeNoLog();
	long Err = NO_ERR;

	if (gcPowerGantry->IsMoveOnceAxis(strAxis) == false)
	{
		return Err;
	}

	if (GetRunModeNoLog() != PAUSE_MODE)
	{
		return Err;
	}

	TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Lock Try"), strAxis);

	while (1)
	{
		if (MoveOneTimeLockWait(TIME1MS) == true)
		{
			gcPowerGantry->SetAxisPause(strAxis, false);
			gcPowerGantry->SetLockOKAxisName(strAxis);
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Lock OK"), strAxis);
			break;
		}
		else if (GetRunModeNoLog() != PAUSE_MODE)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Mode(%d->%d)"), strAxis, mode, PAUSE_MODE);
			break;
		}
		else if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetMachineState(%d)"), strAxis, GetMachineState());
			Err = STOP_NOW;
			break;
		}
		else if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetGlobalStatusError(%d)"), strAxis, GetGlobalStatusError());
			Err = STOP_NOW;
		}

		ThreadSleep(TIME1MS);
	}

	if (Err == STOP_NOW)
	{

	}
	else if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetGlobalStatusError(%d)"), strAxis, GetGlobalStatusError());
		Err = STOP_NOW;
	}
	else if (GetMachineState() == STATE_STOPNOW)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetMachineState(%d)"), strAxis, GetMachineState());
		Err = STOP_NOW;
	}

	return Err;
}

long MoveOnceLockPauseMode(CString strAxis, double CmdPosition)
{
	long mode = GetRunModeNoLog();
	long Err = NO_ERR;
	double shift = 0.000;

	//gcPowerGantry->SetAxisPause(strAxis, false);

	if (gcPowerGantry->IsMoveOnceAxis(strAxis) == false)
	{
		return Err;
	}

	if (GetRunModeNoLog() != PAUSE_MODE)
	{
		return Err;
	}

	shift = ReadProfileTargetPosition(strAxis) - CmdPosition;
	if (fabs(shift) < EPSILON)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Skip by TargetPosition(%.3f)"), strAxis, CmdPosition);
		return Err;
	}

	TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Lock Try"), strAxis);
	gcPowerGantry->SetAxisPause(strAxis, true);

	while (1)
	{
		if (MoveOneTimeLockWait(TIME1MS) == true)
		{
			gcPowerGantry->SetLockOKAxisName(strAxis);
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Lock OK"), strAxis);
			break;
		}
		else if (GetRunModeNoLog() != PAUSE_MODE)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. Mode(%d->%d)"), strAxis, mode, PAUSE_MODE);
			break;
		}
		else if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetMachineState(%d)"), strAxis, GetMachineState());
			Err = STOP_NOW;
			break;
		}
		else if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetGlobalStatusError(%d)"), strAxis, GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}

		ThreadSleep(TIME1MS);
	}

	if (Err == STOP_NOW)
	{

	}
	else if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetGlobalStatusError(%d)"), strAxis, GetGlobalStatusError());
		Err = STOP_NOW;
	}
	else if (GetMachineState() == STATE_STOPNOW)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseMode %s. GetMachineState(%d)"), strAxis, GetMachineState());
		Err = STOP_NOW;
	}

	return Err;
}


long MoveOnceLockPauseModeTTF(CString TTFName)
{
	long mode = GetRunModeNoLog();
	long Err = NO_ERR;
	double shift = 0.000;

	if (GetRunModeNoLog() != PAUSE_MODE)
	{
		return Err;
	}

	TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. Lock Try"), TTFName);

	while (1)
	{
		if (MoveOneTimeLockWait(TIME1MS) == true)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. Lock OK"), TTFName);
			break;
		}
		else if (GetRunModeNoLog() != PAUSE_MODE)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. Mode(%d->%d)"), TTFName, mode, PAUSE_MODE);
			break;
		}
		else if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. GetMachineState(%d)"), TTFName, GetMachineState());
			Err = STOP_NOW;
			break;
		}
		else if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. GetGlobalStatusError(%d)"), TTFName, GetGlobalStatusError());
			Err = STOP_NOW;
			break;
		}

		ThreadSleep(TIME1MS);
	}

	if (Err == STOP_NOW)
	{

	}
	else if (GetGlobalStatusError() == true)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. GetGlobalStatusError(%d)"), TTFName, GetGlobalStatusError());
		Err = STOP_NOW;
	}
	else if (GetMachineState() == STATE_STOPNOW)
	{
		TRACE(_T("[PWR] MoveOnceLockPauseModeTTF %s. GetMachineState(%d)"), TTFName, GetMachineState());
		Err = STOP_NOW;
	}

	return Err;
}

void SetZRMoveOnce(bool Set)
{
	gcPowerGantry->SetMoveOnceAxisRZ(Set);
}

bool GetZRMoveOnce()
{
	return gcPowerGantry->GetMoveOnceAxisRZ();
}

bool WaitAxisPauseState(long TimeOut)
{
	return gcPowerGantry->WaitAxisPauseState(TimeOut);
}

void ResetAxisPauseState()
{
	gcPowerGantry->ResetAxisPause();
}



bool WaitAxisPauseStateByLock(long TimeOut)
{
	CString strAxis;
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();

	while (1)
	{
		if (MoveOneTimeLockWait(TIME1MS) == true)
		{
			MoveOneTimeUnlock();
		}
		else
		{
			TRACE(_T("[PWR] WaitPausedByLock OK\n"));
			return true;
		}

		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] WaitAxisPauseStateByLock. GetGlobalStatusError(%d)"), GetGlobalStatusError());
			return false;
		}
		else if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] WaitAxisPauseStateByLock. GetMachineState(%d)"), GetMachineState());
			return false;
		}

		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitAxisPauseState TimeOut(%d)\n"), TimeOut);
			return false;
		}
		ThreadSleep(TIME1MS);
	}

	return false;
}

void ResetLockOKAxisName()
{
	gcPowerGantry->SetLockOKAxisName(_T("NON"));
}

bool WaitLockOKAxisIdle(long TimeOut)
{
	bool IdleResult = false;
	bool LockResult = false;
	CString strAxis = _T("NON");
	ULONGLONG GetTime = 0, ElapsedTime = 0;
	GetTime = _time_get();

	while (1)
	{
		if (LockResult == false)
		{
			if (MoveOneTimeLockWait(TIME1MS) == true)
			{
				MoveOneTimeUnlock();
			}
			else
			{
				TRACE(_T("[PWR] WaitLockOKAxisIdle LockOK\n"));
				LockResult = true;
			}
		}
		else
		{
			strAxis = gcPowerGantry->GetLockOKAxisName();
			if (strAxis.CompareNoCase(_T("NON")) == 0)
			{

			}
			else
			{
				IdleResult = gcPowerGantry->WaitOneIdle(strAxis, TIME5000MS);
				TRACE(_T("[PWR] WaitLockOKAxisIdle IdleEnd %s Err:%d\n"), strAxis, IdleResult);
				return true;
			}

		}


		if (GetGlobalStatusError() == true)
		{
			TRACE(_T("[PWR] WaitLockOKAxisIdle. GetGlobalStatusError(%d)"), GetGlobalStatusError());
			return false;
		}
		else if (GetMachineState() == STATE_STOPNOW)
		{
			TRACE(_T("[PWR] WaitLockOKAxisIdle. GetMachineState(%d)"), GetMachineState());
			return false;
		}

		ElapsedTime = _time_elapsed(GetTime);
		if (ElapsedTime > TimeOut)
		{
			TRACE(_T("[PWR] WaitLockOKAxisIdle TimeOut(%d)\n"), TimeOut);
			return false;
		}
		ThreadSleep(TIME1MS);
	}

	return false;
}


long ANCDownBeforeMove(long Gantry)
{
	ULONGLONG GetTime = 0;
	long Base;
	CString strFunc(__func__);


	if (Gantry == FRONT_GANTRY)
	{
		Base = FRONT_STAGE;
	}
	else
	{
		Base = REAR_STAGE;
	}

	if (GetGlobalSimulationMode() == true) // Simulation
	{
		return NO_ERR;
	}

	if (GetMoveCheckANCDown(Gantry) == true && gcCAutoNozzleChange->IsUpDownType() == true && gcCAutoNozzleChange->GetUseANC() == true && gcCAutoNozzleChange->GetBaseUpDown(BASE_DOWN, TIME100MS) != NO_ERR)
	{
		TRACE(_T("[PWR] %s Gantry:%d Base:%d Try ActBaseDown"), strFunc, Gantry, Base);
		gcCAutoNozzleChange->ActBaseUpDown(BASE_DOWN);

		GetTime = _time_get();
		while (1)
		{
			if (gcCAutoNozzleChange->GetBaseUpDown(BASE_DOWN, TIME100MS) == NO_ERR)
			{
				TRACE(_T("[PWR] %s Gantry:%d Base:%d ActBaseDown Pass"), strFunc, Gantry, Base);
				return NO_ERR;
			}
			else if (_time_elapsed(GetTime) > TIME5000MS)
			{
				TRACE(_T("[PWR] %s Gantry:%d Base:%d ActBaseDown fail"), strFunc, Gantry, Base);

				return ANC_BASE_DN_TIMEOUT(Base);
			}

			ThreadSleep(TIME1MS);
		}
	}
	return NO_ERR;
}

bool IsTrayTypeFeeder(long FeederType)
{
	if (FeederType == TYPE_MANUALTRAY) return true;
	if (FeederType == TYPE_TTF1) return true;
	if (FeederType == TYPE_TTF2) return true;
	if (FeederType == TYPE_AMTF_F) return true;
	if (FeederType == TYPE_AMTF_R) return true;

	return false;
}
bool IsLabelTypeFeeder(long FeederType)
{
	if (FeederType == TYPE_LABEL) return true;

	return false;
}

void gPartDropLedOn(long CamTable)
{
	if (gcReadJobFile->GetPartDrop().Use == false) return;

	MODULE_LED led = gcReadJobFile->GetPartDrop().led;

	if (CamTable == FRONT_GANTRY)
	{
		gLedOn(CAM1, led.Top, led.Mid, led.Bot);
		gLedOn(CAM2, led.Top, led.Mid, led.Bot);
	}
	else
	{
		gLedOn(RCAM1, led.Top, led.Mid, led.Bot);
		gLedOn(RCAM2, led.Top, led.Mid, led.Bot);
	}

	TRACE(_T("[PWR] PartDropLedOn(%d) Led:%d,%d,%d\n"), CamTable, led.Top, led.Mid, led.Bot);
}

void gPartDropProcess(long CamTable)
{
	if (gcReadJobFile->GetPartDrop().Use == false) return;

	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	msgSend->SetThreadMsg(_T("0"));
	msgSend->SetThreadSubMsg(0, 0, 0);
	if (gcCamDropCheck[CamTable])
	{
		gcCamDropCheck[CamTable]->InitDropResult();

		gcCamDropCheck[CamTable]->GetId(&id);
		msgSend->SetID(id);
		if (gcCamDropCheck[CamTable]->PingThread(TIME1MS))
		{
			gcCamDropCheck[CamTable]->Event((LPVOID)msgSend);
			TRACE(_T("[PWR] PartDropProcess(%d) Send ProcessMsg\n"), CamTable);
		}
	}
}

long gPartDropGetResult(long CamTable)
{
	if (gcReadJobFile->GetPartDrop().Use == false) return NO_ERR;

	ULONGLONG time = _time_get();
	long result = NO_ERR;
	long timeOut = TIME1000MS;

	while (1)
	{
		if (_time_elapsed(time) > timeOut)
		{
			TRACE(_T("[PWR] PartDropGetResult(%d) TimeOut\n"), CamTable);

			return FCAMERA_DROP_ERROR + CamTable;
		}

		if (gcCamDropCheck[CamTable]->GetDropResult(&result) == true)
		{
			TRACE(_T("[PWR] PartDropGetResult(%d) Result:%d WaitTime:%d\n"), CamTable, result, _time_elapsed(time));

			return result;
		}

		ThreadSleep(TIME1MS);
	}

	return NO_ERR;
}



double GetManualSafetyDistanceSWLimit()
{
	return 5.0;
}

bool IsNearSWLimitMinus(CString strAxis)
{
	Limit swLimit = GetLimit(GetAxisIndexFromAliasName(strAxis));
	double safeDistance = GetManualSafetyDistanceSWLimit();
	double currentPosition = ReadPosition(strAxis);

	//if (fabs(swLimit.minus - currentPosition) < safeDistance)
	if (currentPosition < swLimit.minus + safeDistance)
	{
		return true;
	}

	return false;
}

bool IsNearSWLimitPlus(CString strAxis)
{
	Limit swLimit = GetLimit(GetAxisIndexFromAliasName(strAxis));
	double safeDistance = GetManualSafetyDistanceSWLimit();
	double currentPosition = ReadPosition(strAxis);

	if (currentPosition > swLimit.plus - safeDistance)
	{
		return true;
	}

	return false;
}

bool IsDangerSWLimit(CString strAxis)
{
	Limit swLimit = GetLimit(GetAxisIndexFromAliasName(strAxis));
	double safeDistance = 0.5;
	double currentPosition = ReadPosition(strAxis);

	if (currentPosition < swLimit.minus + safeDistance)
	{
		return true;
	}

	if (currentPosition > swLimit.plus - safeDistance)
	{
		return true;
	}

	return false;
}

bool IsAxisStatusJog(CString strAxis)
{
	return gcPowerGantry->IsAxisStatusJog(strAxis);
}

void SetHeightMeasureDone(long done)
{
	if (gcPowerConveyorControl)
	{
		gcPowerConveyorControl->SetHeightMeasureDone(done);
	}
}

long GetHeightMeasureDone()
{
	long done = true;
	if (gcPowerConveyorControl)
	{
		done = gcPowerConveyorControl->GetHeightMeasureDone();
	}
	return done;
}

void gBarcodeTriggerOn()
{
	if (gcBarcodeControl)
	{
		gcBarcodeControl->TriggerOn();
	}
}

void gBarcodeTriggerOff()
{
	if (gcBarcodeControl)
	{
		gcBarcodeControl->TriggerOff();
	}
}

CString gGetBarcodeString()
{
	CString strBarcode;
	strBarcode.Empty();
	if (gcBarcodeControl)
	{
		strBarcode = gcBarcodeControl->GetBarcode();
	}
	return strBarcode;
}

void gSetBarcodeString(CString str)
{
	if (gcBarcodeControl)
	{
		gcBarcodeControl->SetBarcode(str);
	}
}

void gInitBarcodeString()
{
	if (gcBarcodeControl)
	{
		gcBarcodeControl->InitBarcode();
	}
}

void EIMES_SendProtocol(long SubMsg2, long SubMsg3, CString strMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_5;
	nSubMsg[1] = SubMsg2;
	nSubMsg[2] = SubMsg3;
	strSendMsg.Format(_T("%s"), (LPCTSTR)strMsg);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
}

void MES_SendBarcode(CString strBarcode)
{
	EIMES_SendProtocol(HMI_CMD2ND_00, HMI_CMD3RD_00, strBarcode);
}

void MES_SendResult(long Result, CString strPath)
{
	CString strResult;
	strResult.Format(_T("%d,%s"), Result, (LPCTSTR)strPath);
	EIMES_SendProtocol(HMI_CMD2ND_02, HMI_CMD3RD_00, strResult);
}

void EIMES_GetResult(long Result) // CanDo, CanNotDo
{

}

void MES_SendBarcode(CString strBarcode, CString strBarcodeBlock1, CString strBarcodeBlock2)
{
	CString strSend;
	strSend.Format(_T("%s,%s,%s"), (LPCTSTR)(strBarcode), (LPCTSTR)(strBarcodeBlock1), (LPCTSTR)(strBarcodeBlock2));
	MES_SendProtocol(HMI_CMD2ND_50, HMI_CMD3RD_03, strSend);
}

void MES_SendBarcodeSNTMotive(CString strBarcode)
{
	CString strSend;
	strSend.Format(_T("%s"), (LPCTSTR)(strBarcode));
	MES_SendProtocol(HMI_CMD2ND_50, HMI_CMD3RD_08, strSend);
}

void MES_SendProtocol(long SubMsg2, long SubMsg3, CString strMsg)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_4;
	nSubMsg[1] = SubMsg2;
	nSubMsg[2] = SubMsg3;
	strSendMsg.Format(_T("%s"), (LPCTSTR)strMsg);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
}

void gSetBarcode(long Conv, CString strBarcode)
{
	if (gcBarcodeFile)
	{
		gcBarcodeFile->SetBarcode(Conv, strBarcode);
		gcBarcodeFile->WriteFile();
	}
}

CString gGetBarcode(long Conv)
{
	CString strBarcode;
	strBarcode.Empty();
	if (gcBarcodeFile)
	{
		strBarcode = gcBarcodeFile->GetBarcode(Conv);
	}
	return strBarcode;
}

void gCopyBarcode(long SrcConv, long DestConv)
{
	CString strSrc;
	strSrc = gGetBarcode(SrcConv);
	gSetBarcode(DestConv, strSrc);
	strSrc.Format(_T("EMPTY"));
	gSetBarcode(SrcConv, strSrc);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void SendConveyorBarcode(long Conv, CString strBarcode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_4;
	nSubMsg[1] = HMI_CMD2ND_50;
	nSubMsg[2] = HMI_CMD3RD_01;
	strSendMsg.Format(_T("%d,%s"), Conv, (LPCTSTR)strBarcode);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
}

void SendOutBufferConveyorBarcode(CString strBarcode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_4;
	nSubMsg[1] = HMI_CMD2ND_50;
	nSubMsg[2] = HMI_CMD3RD_04;
	strSendMsg.Format(_T("%s"), (LPCTSTR)strBarcode);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
	gSetBarcode(EXIT_CONV, strBarcode);
}

void SendNextMachineConveyorBarcode(CString strBarcode)
{
	unsigned nSubMsg[3];
	CString strSendMsg;
	ThreadId_t id;
	ZeroMemory(&nSubMsg, sizeof(nSubMsg));
	nSubMsg[0] = HMI_CMD1ST_4;
	nSubMsg[1] = HMI_CMD2ND_50;
	nSubMsg[2] = HMI_CMD3RD_05;
	strSendMsg.Format(_T("%s"), (LPCTSTR)strBarcode);
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	msgSend->SetThreadMsg(strSendMsg);
	msgSend->SetThreadSubMsg(nSubMsg[0], nSubMsg[1], nSubMsg[2]);
	if (gcCPowerHMI)
	{
		gcCPowerHMI->GetId(&id);
		msgSend->SetID(id);
		if (gcCPowerHMI->PingThread(TIME1MS))
		{
			gcCPowerHMI->Event((LPVOID)msgSend);
		}
	}
}

void gMainMESResultOK()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MES_OK));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void gMainMESResultNG()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MES_NG));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

long GetCustomerSiteFromBarcodeSetting(BARCODE Barcode)
{
	long CustomerSite = CUSTOMER_NONE;
	if (Barcode.Use == 1 && Barcode.Mes.Use == 3) // Single Interface
	{
		CustomerSite = CUSTOMER_DEFAULT;
	}
	else if (Barcode.Use == 1 && Barcode.Mes.Use == 4) // SNT Motiv 
	{
		CustomerSite = CUSTOMER_SNTMOTIVE;
	}

	//CustomerSite = CUSTOMER_SNTMOTIVE;

	//TRACE(_T("[PWR] GetCustomerSiteFromBarcodeSetting(%d)\n"), CustomerSite);
	return CustomerSite;
}

void gMainMES_Disconnect()
{
	PowerThreadMessage* msgSend = new PowerThreadMessage();
	ThreadId_t id;
	unsigned SubMsg1, SubMsg2, SubMsg3;
	SubMsg1 = SubMsg2 = SubMsg3 = 0;
	msgSend->SetThreadMsg(_T(STRING_MES_DISCONECT));
	msgSend->SetThreadSubMsg(SubMsg1, SubMsg2, SubMsg3);
	if (gcPowerMainControl)
	{
		gcPowerMainControl->GetId(&id);
		msgSend->SetID(id);
		if (gcPowerMainControl->PingThread(TIME1MS))
		{
			gcPowerMainControl->Event((LPVOID)msgSend);
		}
	}
}

void ptRot(double* x, double* y, double rad)
{
	double 	tmp;
	double 	ca, sa;

	ca = cos(rad);
	sa = sin(rad);
	tmp = *x;
	*x = tmp * ca - (*y) * sa;
	*y = tmp * sa + (*y) * ca;
}

void ptRotDeg(double* x, double* y, double deg)
{
	double 	tmp, rad;
	double 	ca, sa;

	rad = (PIE_180)*deg;
	ca = cos(rad);
	sa = sin(rad);
	tmp = *x;
	*x = tmp * ca - (*y) * sa;
	*y = tmp * sa + (*y) * ca;
}

long GetMaxBoardCount()
{
	return gcPowerConveyorControl->GetMaxBoardCount();
}

void SetMaxBoardCount(long MaxBoardCount)
{
	gcPowerConveyorControl->SetMaxBoardCount(MaxBoardCount);
}

bool IsLoadable()
{
	return gcPowerConveyorControl->GetLoadable();
}

void SetLoadable(bool Loadable)
{
	gcPowerConveyorControl->SetLoadable(Loadable);
}