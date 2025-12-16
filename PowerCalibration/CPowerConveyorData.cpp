#include "pch.h"
#include "CPowerConveyorData.h"
#include "CMachineFile.h"
#include "GlobalDefine.h"
#include "LockDef.h"
#include "Trace.h"
#include "ConveyorInformation.h"
#include "CRunFileConveyorDB.h"
//#include "ErrorCode.h"

CPowerConveyorData* gcPowerConveyorData;
CPowerConveyorData::CPowerConveyorData()
{
	ZeroMemory(&m_FrontConveyorWidth, sizeof(m_FrontConveyorWidth));
	ZeroMemory(&m_RearConveyorWidth, sizeof(m_RearConveyorWidth));
	ZeroMemory(&m_FrontConveyorPusherZ, sizeof(m_FrontConveyorPusherZ));
	ZeroMemory(&m_RearConveyorPusherZ, sizeof(m_RearConveyorPusherZ));
}

CPowerConveyorData::~CPowerConveyorData()
{
}

void CPowerConveyorData::LoadWidth(long Conveyor, long Type, double Position)
{
	if (abs(Position) > MAX_MOTION_VALID_RANGE)
		Position = 0.0;
	if (Type < MAX_CONV)
	{
		if (Conveyor == FRONT_CONV)
		{
			m_FrontConveyorWidth[Type] = Position;
		}
		else if (Conveyor == REAR_CONV)
		{
			m_RearConveyorWidth[Type] = Position;
		}
		else
		{
			TRACE(_T("[PWR] CPowerConveyor::LoadWidth\n"));
		}
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyorData::LoadWidth Invalid Type:%d\n"), Type);
	}
}

void CPowerConveyorData::SetWidth(long Conveyor, long Type, double Position)
{
	if (Type < MAX_CONV)
	{
		if (Conveyor == FRONT_CONV)
		{
			m_FrontConveyorWidth[Type] = Position;
			TRACE(_T("[PWR] SetWidth Front Type(%d):%.3f\n"), Type, Position);
		}
		else if (Conveyor == REAR_CONV)
		{
			m_RearConveyorWidth[Type] = Position;
			TRACE(_T("[PWR] SetWidth Rear Type(%d):%.3f\n"), Type, Position);
		}
		else
		{
			TRACE(_T("[PWR] CPowerConveyorData::SetWidth\n"));
		}
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyorData::SetWidth Invalid Type:%d\n"), Type);
	}
}

double CPowerConveyorData::GetWidth(long Conveyor, long Type)
{
	double Position = 0.0f;
	if (Conveyor == FRONT_CONV)
	{
		Position = m_FrontConveyorWidth[Type];

	}
	else if (Conveyor == REAR_CONV)
	{
		Position = m_RearConveyorWidth[Type];
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyor::GetWidth Invalid Conveyor:%d\n"), Conveyor);
	}
	if (abs(Position) > MAX_MOTION_VALID_RANGE)
		Position = 0.0;
	TRACE(_T("[PWR] GetWidth %s Type(%d):%.3f\n"), Conveyor==FRONT_CONV?_T("Front"):_T("Rear"), Type, Position);
	return Position;
}

bool CPowerConveyorData::WriteWidth(long Conveyor)
{
	bool bRet = false;

	if (gcRuntimeConveyorDB->GetLoadComplete() == true)
	{
		return gcRuntimeConveyorDB->SaveLastWidth();
	}

	bRet = gcMachineFile->WriteToDiskBlock7();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerConveyorData::LoadPusherZ(long Conveyor, long Type, double Position)
{
	if (abs(Position) > MAX_MOTION_VALID_RANGE)
		Position = 0.0;
	if (Conveyor == FRONT_CONV)
	{
		m_FrontConveyorPusherZ[Type] = Position;
	}
	else if (Conveyor == REAR_CONV)
	{
		m_RearConveyorPusherZ[Type] = Position;
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyor::LoadPusherZ Invalid Conveyor:%d\n"), Conveyor);
	}
}

void CPowerConveyorData::SetPusherZ(long Conveyor, long Type, double Position)
{
	if (Conveyor == FRONT_CONV)
	{
		m_FrontConveyorPusherZ[Type] = Position;
		TRACE(_T("[PWR] SetPuhserZ Front Type(%d):%.3f\n"), Type, Position);
	}
	else if (Conveyor == REAR_CONV)
	{
		m_RearConveyorPusherZ[Type] = Position;
		TRACE(_T("[PWR] SetPuhserZ Rear Type(%d):%.3f\n"), Type, Position);
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyor::SetPusherZ Invalid Conveyor:%d\n"), Conveyor);
	}
}

double CPowerConveyorData::GetPusherZ(long Conveyor, long Type)
{
	double Position = 0.0f;
	if (Conveyor == FRONT_CONV)
	{
		Position = m_FrontConveyorPusherZ[Type];
	}
	else if (Conveyor == REAR_CONV)
	{
		Position = m_RearConveyorPusherZ[Type];
	}
	else
	{
		TRACE(_T("[PWR] CPowerConveyor::GetPusherZ Invalid Conveyor:%d\n"), Conveyor);
	}
	if (abs(Position) > MAX_MOTION_VALID_RANGE)
		Position = 0.0;
	TRACE(_T("[PWR] GetPusherZ %s Type(%d):%.3f\n"), Conveyor == FRONT_CONV ? _T("Front") : _T("Rear"), Type, Position);
	return Position;
}

bool CPowerConveyorData::WritePusherZ(long Conveyor)
{
	bool bRet = false;
	if (gcRuntimeConveyorDB->GetLoadComplete() == true)
	{
		return gcRuntimeConveyorDB->SaveLastPushserZ();
	}

	bRet = gcMachineFile->WriteToDiskBlock7();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerConveyorData::LoadWidthOption(long Option)
{
	long Opt = Option;
	if (abs(Opt) > MAX_OPTION_VALID_RANGE)
		Opt = 0;
	m_OptionWidth = Opt;
}

void CPowerConveyorData::SetWidthOption(long Option)
{
	m_OptionWidth = Option;
}

long CPowerConveyorData::GetWidthOption()
{
	TRACE(_T("[PWR] GetWidthOption:%d\n"), m_OptionWidth);
	return m_OptionWidth;
}

bool CPowerConveyorData::WriteWidthOption()
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlock6();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}


double CPowerConveyorData::GetMinWidth(long Option)
{
	double retMin = 0.0;
	switch (GetWidthOption())
	{
	case CONV_SINGLE_350MM:
		retMin = 40.0;
		break;
	case CONV_DUAL_250MM:
		retMin = 40.0;
		break;
	default:
		TRACE(_T("[PWR] GetMinWidth Invalid Option:%d\n"), Option);
		break;
	}
	return retMin;
}

double CPowerConveyorData::GetMaxWidth(long Option)
{
	double retMax = 0.0;
	switch (GetWidthOption())
	{
	case CONV_SINGLE_350MM:
		retMax = 355.0;
		break;
	case CONV_DUAL_250MM:
		retMax = 251.0;
		break;
	default:
		TRACE(_T("[PWR] GetMaxWidth Invalid Option:%d\n"), Option);
		break;
	}
	return retMax;
}

bool CPowerConveyorData::IsWidthValidRange(double Width)
{
	long Option = GetWidthOption();
	if (Width >= GetMinWidth(Option) && Width <= GetMaxWidth(Option))
	{
		return true;
	}
	else
	{
		TRACE(_T("[PWR] Width Range Over(%.3f~%.3f)"), GetMinWidth(Option), GetMaxWidth(Option));
		return false;
	}
}

void CPowerConveyorData::LoadPusherZOption(long Option)
{
	if (abs(Option) > MAX_OPTION_VALID_RANGE)
		Option = 0;
	m_OptionPusherZ = Option;
}

void CPowerConveyorData::SetPusherZOption(long Option)
{
	m_OptionPusherZ = Option;
}

long CPowerConveyorData::GetPusherZOption()
{
	TRACE(_T("[PWR] GetPusherZOption:%d\n"), m_OptionPusherZ);
	return m_OptionPusherZ;
}

bool CPowerConveyorData::WritePusherZOption()
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlock6();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

double CPowerConveyorData::GetMinPuhserZ(long Option)
{
	double retMin = 0.0;
	switch (GetPusherZOption())
	{
	case PUSHERZ_TYPE0:
		retMin = 118.999;
		break;
	case PUSHERZ_TYPE1:
		retMin = 118.999;
		break;
	default:
		TRACE(_T("[PWR] GetMinPuhserZ Invalid Option:%d\n"), Option);
		break;
	}
	return retMin;
}

double CPowerConveyorData::GetMaxPusherZ(long Option)
{
	double retMax = 0.0;
	switch (GetPusherZOption())
	{
	case PUSHERZ_TYPE0:
		retMax = 150.001;
		break;
	case PUSHERZ_TYPE1:
		retMax = 150.001;
		break;
	default:
		TRACE(_T("[PWR] GetMaxPusherZ Invalid Option:%d\n"), Option);
		break;
	}
	return retMax;
}

bool CPowerConveyorData::IsPusherZValidRange(double PusherZ)
{
	long Option = GetPusherZOption();
	if (PusherZ >= GetMinPuhserZ(Option) && PusherZ <= GetMaxPusherZ(Option))
	{
		return true;
	}
	else
	{
		TRACE(_T("[PWR] PuhserZ Range Over(%.3f~%.3f)"), GetMinPuhserZ(Option), GetMaxPusherZ(Option));
		return false;
	}
}

void CPowerConveyorData::LoadInsertDone(long Conv, long InsertDone)
{
	long insertDone = InsertDone;
	if (abs(insertDone) > MAX_OPTION_VALID_RANGE)
	{
		insertDone = 0;
	}
	m_InsertDone[Conv] = insertDone;
}

long CPowerConveyorData::SetInsertDone(long Conv, long InsertDone)
{
	if (m_InsertDone[Conv] != InsertDone)
	{
		TRACE(_T("[PWR] SetInsertDone(%d,%d)"), Conv, InsertDone);
	}
	m_InsertDone[Conv] = InsertDone;
	//WriteInsertDone(Conv);
	return NO_ERR;
}

long CPowerConveyorData::GetInsertDone(long Conv)
{
	return m_InsertDone[Conv];
}

bool CPowerConveyorData::WriteInsertDone(long Conv)
{
	bool bRet = false;

	if (gcRuntimeConveyorDB->GetLoadComplete() == true)
	{
		return gcRuntimeConveyorDB->SaveInsertDone();
	}

	bRet = gcMachineFile->WriteToDiskBlockD();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerConveyorData::LoadPcbOutDone(long Conv, long PcbOutDone)
{
	long OutDone = PcbOutDone;
	if (abs(OutDone) > MAX_OPTION_VALID_RANGE)
	{
		OutDone = 0;
	}
	m_PcbOutDone[Conv] = OutDone;
}

long CPowerConveyorData::SetPcbOutDone(long Conv, long PcbOutDone)
{
	m_PcbOutDone[Conv] = PcbOutDone;
	//WritePcbOutDone(Conv);
	return NO_ERR;
}

long CPowerConveyorData::GetPcbOutDone(long Conv)
{
	return m_PcbOutDone[Conv];
}

bool CPowerConveyorData::WritePcbOutDone(long Conv)
{
	bool bRet = false;
	if (gcRuntimeConveyorDB->GetLoadComplete() == true)
	{
		return gcRuntimeConveyorDB->SavePcbOutDone();
	}

	bRet = gcMachineFile->WriteToDiskBlockD();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}

void CPowerConveyorData::LoadHeightMeasureDone(long done)
{
	if (done == 0 || done == 1)
	{
		if (m_HeighitMeasureDone != done)
		{
			TRACE(_T("[PWR] LoadHeightMeasureDone:%d->%d"), m_HeighitMeasureDone, done);
		}

		m_HeighitMeasureDone = done;
	}
	else
	{
		TRACE(_T("[PWR] LoadHeightMeasureDone Invalid:%d"), done);

		m_HeighitMeasureDone = 1;
	}
}

long CPowerConveyorData::SetHeightMeasureDone(long done)
{
	LoadHeightMeasureDone(done);
	WriteHeightMeasureDone();
	return NO_ERR;
}

long CPowerConveyorData::GetHeightMeasureDone()
{
	return m_HeighitMeasureDone;
}

bool CPowerConveyorData::WriteHeightMeasureDone()
{
	bool bRet = false;
	bRet = gcMachineFile->WriteToDiskBlockD();
	bRet = gcMachineFile->SaveFile();
	return bRet;
}