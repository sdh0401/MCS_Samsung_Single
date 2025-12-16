#include "pch.h"
#include "CPowerConveyor.h"
#include "CPcbExist.h"
#include "CSmemaControl.h"
#include "GlobalData.h"
#include "Trace.h"
#include "LockDef.h"
#include "CApplicationTime.h"
#include "AxisInformation.h"
#include "CPowerLog.h"
//#include "ErrorCode.h"

CPowerConveyor::CPowerConveyor()
{
	InitVariable();
}

CPowerConveyor::CPowerConveyor(bool bReverse)
{
	InitVariable();
	m_Reverse = bReverse;
}

CPowerConveyor::~CPowerConveyor()
{
}

bool CPowerConveyor::IsSimulationOn()
{
	return m_IsSimulationOn;
}

void CPowerConveyor::SetSimulationOn(bool SimulationOn)
{
	m_IsSimulationOn = SimulationOn;
}

void CPowerConveyor::SetTaskID(long ID)
{
	m_ShowID = ID;
}

long CPowerConveyor::GetTaskID(void)
{
	return m_ShowID;
}

void CPowerConveyor::SetReverse(bool bReverse)
{
	m_Reverse = bReverse;
}

void CPowerConveyor::InitVariable()
{
	m_IsSimulationOn = false;
	m_BeltControlType = BeltControl::None;
	m_BeltMotorName.Empty();
	m_PusherZMotorName.Empty();
	m_ConveyorWidthMotorName.Empty();
	SetStep(ConveyorStep::STOP);
	m_Reverse = false;
	m_PcbReady = false, m_OldPcbReady = false;
	m_PcbOut = false, m_OldPcbOut = false;
	m_PcbThickness = 1.0;
	m_PcbStandByZOffset = 10.0;
	m_ProdRunMode = 0;
	m_FromPcb = 0;
	m_FreeTime = 60;
	m_bStartFreeTime = true;
	m_PrevInBeltSpd = m_NextOutBeltSpd = BELT_SPEED_MID;
	m_PrevTransferTimeOut = TIME10000MS;
	m_NextTransferTimeOut = TIME10000MS;
	m_RatioPusherUp = m_RatioPusherDown = 0.5;

}

bool CPowerConveyor::IsReverse()
{
	return m_Reverse;
}

double CPowerConveyor::GetConveyorDir()
{
	if (IsReverse() == true)
		return -1.0;
	else
		return 1.0;
}

ConveyorStep CPowerConveyor::GetStep()
{
	return m_Step;
}

void CPowerConveyor::SetStep(ConveyorStep Step)
{
	m_Step = Step;
}

void CPowerConveyor::SetPcbReady(bool Ready)
{
	m_PcbReady = Ready;
	if (m_OldPcbReady != m_PcbReady)
	{
		TRACE(_T("[PWR] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"));
		if (Ready == true)
			TRACE(_T("[PWR] <<<<<<<<<<<<<<<<<<<< Pcb(%d) Ready <<<<<<<<<<<<<<<<<<<<<\n"), GetConv());
		else
			TRACE(_T("[PWR] <<<<<<<<<<<<<<<<<<<< Pcb(%d) Clear <<<<<<<<<<<<<<<<<<<<<\n"), GetConv());
		TRACE(_T("[PWR] <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"));
		m_OldPcbReady = m_PcbReady;
	}
}

void CPowerConveyor::SetPcbOut(bool Out)
{
	m_PcbOut = Out;
	if (m_OldPcbOut != m_PcbOut)
	{
		TRACE(_T("[PWR] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"));
		TRACE(_T("[PWR] >>>>>>>>>>>>>>>>>>>>> Pcb(%d) Out >>>>>>>>>>>>>>>>>>>>>\n"), GetConv());
		TRACE(_T("[PWR] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"));
		m_OldPcbOut = m_PcbOut;
	}
}

bool CPowerConveyor::IsPcbReady()
{
	return m_PcbReady;
}

bool CPowerConveyor::IsPcbOut()
{
	return m_PcbOut;
}

void CPowerConveyor::SetPcbInsertDone(bool InsertDone)
{
	m_PcbInsertDone = InsertDone;
}

bool CPowerConveyor::IsPcbInsertDone(long Conveyor)
{
	return m_PcbInsertDone;
}

long CPowerConveyor::GetHighTime()
{
	return m_HighTime;
}

void CPowerConveyor::SetHighTime(long HighTime)
{
	m_HighTime = HighTime;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) HighTime:%d[ms]\n"), GetPos(), GetConv(), HighTime);
}

long CPowerConveyor::GetMiddleTime()
{
	return m_MiddleTime;
}

void CPowerConveyor::SetMiddleTime(long MiddleTime)
{
	m_MiddleTime = MiddleTime;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) MiddleTime:%d[ms]\n"), GetPos(), GetConv(), MiddleTime);
}

long CPowerConveyor::GetLowTime()
{
	return m_LowTime;
}

void CPowerConveyor::SetLowTime(long LowTime)
{
	m_LowTime = LowTime;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) LowTime:%d[ms]\n"), GetPos(), GetConv(), LowTime);
}

long CPowerConveyor::GetFromPcb()
{
	return m_FromPcb;
}

void CPowerConveyor::SetFromPcb(long From)
{
	m_FromPcb = From;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) SetFromPcb:%d\n"), GetPos(), GetConv(), From);
}

void CPowerConveyor::SetConveyorSpeed(long PrevInBeltSpd, long NextOutBeltSpd)
{
	m_PrevInBeltSpd = PrevInBeltSpd;
	m_NextOutBeltSpd = NextOutBeltSpd;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) BeltSpd PrevInBeltSpd(%d) NextOutBeltSpd(%d)\n"), GetPos(), GetConv(), m_PrevInBeltSpd, m_NextOutBeltSpd);
}

double CPowerConveyor::GetPcbThickness()
{
	return m_PcbThickness;
}

void CPowerConveyor::SetPcbThickness(double PcbThickness)
{
	m_PcbThickness = PcbThickness;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) PcbThickness:%.3f[mm]\n"), GetPos(), GetConv(), PcbThickness);
}

double CPowerConveyor::GetPcbStandByZOffset()
{
	return m_PcbStandByZOffset;
}

void CPowerConveyor::SetPcbStandByZOffset(double PcbStandByZOffset)
{
	m_PcbStandByZOffset = PcbStandByZOffset;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) PcbStandByZOffset:%.3f[mm]\n"), GetPos(), GetConv(), PcbStandByZOffset);
}

void CPowerConveyor::SetBeltMotorType(BeltControl Type)
{
	m_BeltControlType = Type;
}

BeltControl CPowerConveyor::GetBeltMotorType()
{
	return m_BeltControlType;
}

void CPowerConveyor::SetBeltMotor(CString strName)
{
	m_BeltMotorName = strName;
	TRACE(_T("[PWR] Pos(%d) Conv(%d) Motor:%s\n"), GetPos(), GetConv(), strName);
}

CString CPowerConveyor::GetBeltMotor()
{
	return m_BeltMotorName;
}

void CPowerConveyor::SetPusherPlateZMotor(CString strName)
{
	m_PusherZMotorName = strName;
}

CString CPowerConveyor::GetPusherPlateZMotor()
{
	return m_PusherZMotorName;
}

void CPowerConveyor::SetPusherPlateZControlType(PusherZType type)
{
	m_PusherZControlType = type;
}

PusherZType CPowerConveyor::GetPusherPlateZControlType()
{
	return m_PusherZControlType;
}

void CPowerConveyor::SetConveyorWidthMotorType(ConveyorMotorType type)
{
	m_ConveyorWidthType = type;
}

ConveyorMotorType CPowerConveyor::GetConveyorWidthMotorType()
{
	return m_ConveyorWidthType;
}


void CPowerConveyor::SetConveyorWidthMotor(CString strName)
{
	m_ConveyorWidthMotorName = strName;
}

CString CPowerConveyor::GetConveyorWidthMotor()
{
	return m_ConveyorWidthMotorName;
}

void CPowerConveyor::SetBeltMotorSpeedLow(JogInfo spd)
{
	m_Conveyor.LowSpd = spd;
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB LOW Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), spd.Acc, spd.Dec, spd.MaxVel);
}

void CPowerConveyor::SetBeltMotorSpeedMid(JogInfo spd)
{
	m_Conveyor.MidSpd = spd;
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB Mid Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), spd.Acc, spd.Dec, spd.MaxVel);
}

void CPowerConveyor::SetBeltMotorSpeedHigh(JogInfo spd)
{
	m_Conveyor.HighSpd = spd;
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB High Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), spd.Acc, spd.Dec, spd.MaxVel);
}

void CPowerConveyor::SetStopperIO(long oUp, long iUp, long oDn, long iDn)
{
	m_Conveyor.PcbStopper.iUp = iUp;
	m_Conveyor.PcbStopper.iDn = iDn;
	m_Conveyor.PcbStopper.oUp = oUp;
	m_Conveyor.PcbStopper.oDn = oDn;
	if (m_Conveyor.PcbStopper.oUp != IO_NOUSE && m_Conveyor.PcbStopper.oDn != IO_NOUSE)
		m_Conveyor.PcbStopper.SolType = SolType::Double;
	else if (m_Conveyor.PcbStopper.oUp == IO_NOUSE && m_Conveyor.PcbStopper.oDn == IO_NOUSE)
		m_Conveyor.PcbStopper.SolType = SolType::None;
	else
		m_Conveyor.PcbStopper.SolType = SolType::Single;
	TRACE(_T("[PWR] Pos(%d) Conveyor(%d) Stopper iUp:%d iDn:%d oUp:%d oDn:%d SolType:%d\n"), 
		GetPos(), GetConv(),
		m_Conveyor.PcbStopper.iUp, m_Conveyor.PcbStopper.iDn, 
		m_Conveyor.PcbStopper.oUp, m_Conveyor.PcbStopper.oDn, m_Conveyor.PcbStopper.SolType);
}

void CPowerConveyor::SetPusherPlateIO(PusherZType Type, long oUp, long iUp, long oDn, long iDn)
{
	m_PusherZControlType = Type;
	m_Conveyor.PcbPusherPlate.iUp = iUp;
	m_Conveyor.PcbPusherPlate.iDn = iDn;
	m_Conveyor.PcbPusherPlate.oUp = oUp;
	m_Conveyor.PcbPusherPlate.oDn = oDn;
	m_Conveyor.PcbPusherPlate.SolType = SolType::Single;
	if (m_Conveyor.PcbPusherPlate.oUp != IO_NOUSE && m_Conveyor.PcbPusherPlate.oDn != IO_NOUSE)
		m_Conveyor.PcbPusherPlate.SolType = SolType::Double;
	else if (m_Conveyor.PcbPusherPlate.oUp == IO_NOUSE && m_Conveyor.PcbPusherPlate.oDn == IO_NOUSE)
		m_Conveyor.PcbPusherPlate.SolType = SolType::None;
	else
		m_Conveyor.PcbPusherPlate.SolType = SolType::Single;
	TRACE(_T("[PWR] Pos(%d) Conveyor(%d) Pusher(%d) iUp:%d iDn:%d oUp:%d oDn:%d SolType:%d\n"),
		GetPos(), GetConv(),
		m_PusherZControlType,
		m_Conveyor.PcbPusherPlate.iUp, m_Conveyor.PcbPusherPlate.iDn,
		m_Conveyor.PcbPusherPlate.oUp, m_Conveyor.PcbPusherPlate.oDn, m_Conveyor.PcbPusherPlate.SolType);
}

bool CPowerConveyor::BeltOn(long BeltSpd)
{
	JogInfo BeldSpd;
	if (CheckAmpAlarm(GetBeltMotor()) == true)
	{
		AlarmClear(GetBeltMotor());
	}
	if (CheckServoOn(GetBeltMotor()) == false)
	{
		ServoOn(GetBeltMotor());
	}
	if (BeltSpd == 0)
	{
		BeldSpd = GetBeltMotorSpeedLow();
	}
	else if (BeltSpd == 1)
	{
		BeldSpd = GetBeltMotorSpeedMid();
	}
	else
	{
		BeldSpd = GetBeltMotorSpeedHigh();
	}
	StartBelt(BeldSpd);
	return true;
}

bool CPowerConveyor::BeltOn(long BeltSpd, long BeltDir)
{
	JogInfo BeldSpd;
	double BeldDir;
	if (CheckAmpAlarm(GetBeltMotor()) == true)
	{
		AlarmClear(GetBeltMotor());
	}
	if (CheckServoOn(GetBeltMotor()) == false)
	{
		ServoOn(GetBeltMotor());
	}
	if (BeltSpd == 0)
	{
		BeldSpd = GetBeltMotorSpeedLow();
	}
	else if (BeltSpd == 1)
	{
		BeldSpd = GetBeltMotorSpeedMid();
	}
	else
	{
		BeldSpd = GetBeltMotorSpeedHigh();
	}
	if (BeltDir == 0)
	{
		BeldDir = 1.0;
	}
	else
	{
		BeldDir = -1.0;
	}
	StartBelt(BeldSpd, BeldDir);
	return true;
}

bool CPowerConveyor::BeltOff(long BeltSpd)
{
	JogInfo BeldSpd;
	if (BeltSpd == 0)
	{
		BeldSpd = GetBeltMotorSpeedLow();
	}
	else if (BeltSpd == 1)
	{
		BeldSpd = GetBeltMotorSpeedMid();
	}
	else
	{
		BeldSpd = GetBeltMotorSpeedHigh();
	}
	StopBelt(BeldSpd.Dec);
	return true;
}

bool CPowerConveyor::StartBelt(JogInfo BeldSpd)
{
	if (m_BeltControlType == BeltControl::Motor)
	{
		JogInfo* jog = new JogInfo();
		jog->Acc = BeldSpd.Acc;
		jog->Dec = BeldSpd.Dec;
		jog->MaxVel = BeldSpd.MaxVel * GetConveyorDir();
		if (gcPowerLog->IsShowConveyorLog() == true)
		{
			TRACE(_T("[PWR] Pos(%d) Conveyor(%d) Vel:%.1f Dir:%.3f\n"),
				GetPos(), GetConv(), BeldSpd.MaxVel, GetConveyorDir());
		}
		StartOneJog(GetBeltMotor(), *jog);
		delete jog;
		return true;
	}
	else if (m_BeltControlType == BeltControl::IO)
	{
		return false;
	}
	else
	{
		return false;
	}
}

bool CPowerConveyor::StartBelt(JogInfo BeldSpd, double BeldDir)
{
	if (m_BeltControlType == BeltControl::Motor)
	{
		JogInfo* jog = new JogInfo();
		jog->Acc = BeldSpd.Acc;
		jog->Dec = BeldSpd.Dec;
		jog->MaxVel = BeldSpd.MaxVel * BeldDir;
		if (gcPowerLog->IsShowConveyorLog() == true)
		{
			TRACE(_T("[PWR] Pos(%d) Conveyor(%d) Vel:%.1f Dir:%.3f\n"),
				GetPos(), GetConv(), BeldSpd.MaxVel, BeldDir);
		}
		StartOneJog(GetBeltMotor(), *jog);
		delete jog;
		return true;
	}
	else if (m_BeltControlType == BeltControl::IO)
	{
		return false;
	}
	else
	{
		return false;
	}
}

bool CPowerConveyor::StopBelt(double Dec)
{
	if (m_BeltControlType == BeltControl::Motor)
	{
		StopOne(GetBeltMotor(), Dec);
		WaitOneIdle(GetBeltMotor(), TIME3000MS);
		return true;
	}
	else if (m_BeltControlType == BeltControl::IO)
	{
		return false;
	}
	else
	{
		return false;
	}
}

bool CPowerConveyor::UpStopper()
{
	if (m_Conveyor.PcbStopper.SolType == SolType::Double)
	{
		OutputOne(m_Conveyor.PcbStopper.oUp, OUTON);
		OutputOne(m_Conveyor.PcbStopper.oDn, OUTOFF);
	}
	else // Single
	{
		if (m_Conveyor.PcbStopper.oUp != IO_NOUSE)
		{
			OutputOne(m_Conveyor.PcbStopper.oUp, OUTON);
		}
		else if (m_Conveyor.PcbStopper.oDn != IO_NOUSE)
		{
			OutputOne(m_Conveyor.PcbStopper.oDn, OUTOFF);
		}
	}
	return true;
}

bool CPowerConveyor::DownStopper()
{
	if (m_Conveyor.PcbStopper.SolType == SolType::Double)
	{
		OutputOne(m_Conveyor.PcbStopper.oUp, OUTOFF);
		OutputOne(m_Conveyor.PcbStopper.oDn, OUTON);
	}
	else // Single
	{
		if (m_Conveyor.PcbStopper.oUp != IO_NOUSE)
		{
			OutputOne(m_Conveyor.PcbStopper.oUp, OUTOFF);
		}
		else if (m_Conveyor.PcbStopper.oDn != IO_NOUSE)
		{
			OutputOne(m_Conveyor.PcbStopper.oDn, OUTON);
		}
	}
	return true;
}

bool CPowerConveyor::IsExist(long Conv)
{
	if (IsExistEnt(Conv) == true) return true;
	if (IsExistSet(Conv) == true) return true;
	if (IsExistExit(Conv) == true) return true;
	if (IsExistLow(Conv) == true) return true;
	if (IsExistOut(Conv) == true) return true;
	return false;
}

bool CPowerConveyor::IsExistEnt(long Conv)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->IsExistEnt(Conv);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::IsExistSet(long Conv)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->IsExistSet(Conv);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::GetStateSet(long Conv, long OnOff)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->GetStateSet(Conv, OnOff);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::IsExistExit(long Conv)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->IsExistExit(Conv);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::IsExistLow(long Conv)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->IsExistLow(Conv);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::IsExistOut(long Conv)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->IsExistOut(Conv);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::GetStateOut(long Conv, long OnOff)
{
	bool bStatus = false;
	if (gcPcbExist)
	{
		bStatus = gcPcbExist->GetStateOut(Conv, OnOff);
		return bStatus;
	}
	return false;
}

bool CPowerConveyor::IsStopperUp(long Conv)
{
	bool bStatus = false;
	if (m_Conveyor.PcbStopper.SolType == SolType::Double)
	{
		if (InputTimeOne(m_Conveyor.PcbStopper.iUp, INON, TIME20MS) == true && InputTimeOne(m_Conveyor.PcbStopper.iDn, INOFF, TIME20MS) == true)
		{
			bStatus = true;
		}
		else
		{
			bStatus = false;
		}
	}
	else // Single
	{
		if (m_Conveyor.PcbStopper.iUp != IO_NOUSE)
		{
			if (InputTimeOne(m_Conveyor.PcbStopper.iUp, INON, TIME20MS) == true)
			{
				bStatus = true;
			}
			else
			{
				bStatus = false;
			}
		}
		else if(m_Conveyor.PcbStopper.iDn != IO_NOUSE)
		{
			if (InputTimeOne(m_Conveyor.PcbStopper.iDn, INOFF, TIME20MS) == true)
			{
				bStatus = true;
			}
			else
			{
				bStatus = false;
			}
		}
		else
		{
			TRACE(_T("[PWR] IsStopperUp Conv Undefine IO\n"));
			bStatus = false;
		}
	}
	return bStatus;
}

bool CPowerConveyor::IsStopperDn(long Conv)
{
	bool bStatus = false;
	if (m_Conveyor.PcbStopper.SolType == SolType::Double)
	{
		if (InputTimeOne(m_Conveyor.PcbStopper.iUp, INOFF, TIME20MS) == true && InputTimeOne(m_Conveyor.PcbStopper.iDn, INON, TIME20MS) == true)
		{
			bStatus = true;
		}
		else
		{
			bStatus = false;
		}
	}
	else // Single
	{
		if (m_Conveyor.PcbStopper.iUp != IO_NOUSE)
		{
			if (InputTimeOne(m_Conveyor.PcbStopper.iUp, INOFF, TIME20MS) == true)
			{
				bStatus = true;
			}
			else
			{
				bStatus = false;
			}
		}
		else if (m_Conveyor.PcbStopper.iDn != IO_NOUSE)
		{
			if (InputTimeOne(m_Conveyor.PcbStopper.iDn, INON, TIME20MS) == true)
			{
				bStatus = true;
			}
			else
			{
				bStatus = false;
			}
		}
		else
		{
			TRACE(_T("[PWR] IsStopperDn Conv Undefine IO\n"));
			bStatus = false;
		}
	}
	return bStatus;
}

unsigned CPowerConveyor::GetPos()
{
	return m_Conveyor.Status.Pos;
}

unsigned CPowerConveyor::GetConv()
{
	return m_Conveyor.Status.Conv;
}

void CPowerConveyor::SetInfo(long Pos, long Conv)
{
	m_Conveyor.Status.Pos = Pos;
	m_Conveyor.Status.Conv = Conv;
}

long CPowerConveyor::CheckPrevSmema(long Conv)
{
	long status = SMEMA_NOT_READY;
	status = gcSmemaControl->CheckPrevSmema(Conv);
	return status;
}

long CPowerConveyor::CheckNextSmema(long Conv)
{
	long status = SMEMA_BUSY;
	status = gcSmemaControl->CheckNextSmema(Conv);
	return status;
}

void CPowerConveyor::PrevSmemaOut(long Conv, UBYTE output)
{
	gcSmemaControl->PrevSmemaOut(Conv, output);
}

void CPowerConveyor::NextSmemaOut(long Conv, UBYTE output)
{
	gcSmemaControl->NextSmemaOut(Conv, output);
}

void CPowerConveyor::InitSmema()
{
	PrevSmemaOut(GetConv(), SMEMA_BUSY);
	NextSmemaOut(GetConv(), SMEMA_NOT_READY);
}

void CPowerConveyor::InitSmema(long Conv)
{
	PrevSmemaOut(Conv, SMEMA_BUSY);
	NextSmemaOut(Conv, SMEMA_NOT_READY);
}

void CPowerConveyor::SetProdRunMode(long ProdRunMode)
{
	m_ProdRunMode = ProdRunMode;
	TRACE(_T("[PWR] Pos:%d Conv:%d SetProdRunMode:%d"), GetPos(), GetConv(), ProdRunMode);
}

long CPowerConveyor::GetProdRunMode()
{
	return m_ProdRunMode;
}

void CPowerConveyor::SetMaxFreeTime(long FreeTime)
{
	m_FreeTime = FreeTime;
}

long CPowerConveyor::GetMaxFreeTime()
{
	return m_FreeTime;
}

void CPowerConveyor::StartGetFreeTime()
{
	m_bStartFreeTime = true;
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Conv:%d StartGetFreeTime *****\n"), GetConv());
	}
}

void CPowerConveyor::StopGetFreeTime()
{
	m_bStartFreeTime = false;
	if (gcPowerLog->IsShowTowerLampLog() == true)
	{
		TRACE(_T("[PWR] ***** Conv:%d StopGetFreeTime *****\n"), GetConv());
	}
}

bool CPowerConveyor::IsStartGetFreeTime()
{
	return m_bStartFreeTime;
}

void CPowerConveyor::EntryLoadingStart()
{
	if (GetConv() == ENTRY_CONV)
	{
		StartLoadingTimeConveyor();
	}
}

void CPowerConveyor::WorkLoadingElapsed()
{
	if (GetConv() == WORK1_CONV)
	{
		EndLoadingTimeConveyor();
	}
}

void CPowerConveyor::SetTransferTimeOut(long TimeOut)
{
	m_TransferTimeOut = TimeOut;
}

long CPowerConveyor::GetTransferTimeOut()
{
	return m_TransferTimeOut;
}

void CPowerConveyor::SetPrevTimeOut(long TimeOut)
{
	m_PrevTransferTimeOut = TimeOut;
}

long CPowerConveyor::GetPrevTimeOut()
{
	return m_PrevTransferTimeOut;
}

void CPowerConveyor::SetNextTimeOut(long TimeOut)
{
	m_NextTransferTimeOut = TimeOut;
}

long CPowerConveyor::GetNextTimeOut()
{
	return m_NextTransferTimeOut;
}

long CPowerConveyor::GetPrevInBeltSpd()
{
	return m_PrevInBeltSpd;
}

long CPowerConveyor::GetNextOutBeltSpd()
{
	return m_NextOutBeltSpd;
}

JogInfo CPowerConveyor::GetBeltMotorSpeedLow()
{
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB LOW Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), m_Conveyor.LowSpd.Acc, m_Conveyor.LowSpd.Dec, m_Conveyor.LowSpd.MaxVel);
	return m_Conveyor.LowSpd;
}

JogInfo CPowerConveyor::GetBeltMotorSpeedMid()
{
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB Mid Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), m_Conveyor.MidSpd.Acc, m_Conveyor.MidSpd.Dec, m_Conveyor.MidSpd.MaxVel);
	return m_Conveyor.MidSpd;
}

JogInfo CPowerConveyor::GetBeltMotorSpeedHigh()
{
	TRACE(_T("[PWR] Pos(%d) Conv:%d PCB High Acc:%.3f Dec:%.3f Vel:%.3f\n"), GetPos(), GetConv(), m_Conveyor.HighSpd.Acc, m_Conveyor.HighSpd.Dec, m_Conveyor.HighSpd.MaxVel);
	return m_Conveyor.HighSpd;
}

bool CPowerConveyor::GetBeltStopState()
{
	if (GetOneOnlyIdle(GetBeltMotor()) == NO_ERR)
	{
		return true;
	}

	return false;
}

CString CPowerConveyor::GetBarcode()
{
	TRACE(_T("[PWR] GetBarcode Pos(%d) Conv:%d %s\n"), GetPos(), GetConv(), (LPCTSTR)m_Barcode);
	return m_Barcode;
}

void CPowerConveyor::SetBarcode(CString strBarcode)
{
	TRACE(_T("[PWR] SetBarcode Pos(%d) Conv:%d %s\n"), GetPos(), GetConv(), (LPCTSTR)strBarcode);
	m_Barcode = strBarcode;
}

long CPowerConveyor::GetBarcodeType()
{
	TRACE(_T("[PWR] GetBarcodeType Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), m_BarcodeType);
	return m_BarcodeType;
}

void CPowerConveyor::SetBarcodeType(long BarcodeType)
{
	TRACE(_T("[PWR] SetBarcodeType Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), BarcodeType);
	m_BarcodeType = BarcodeType;
}

long CPowerConveyor::GetBarcodeResult()
{
	TRACE(_T("[PWR] GetBarcodeResult Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), m_BarcodeResult);
	return m_BarcodeResult;
}

void CPowerConveyor::SetBarcodeResult(long BarcodeResult)
{
	TRACE(_T("[PWR] SetBarcodeResult Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), BarcodeResult);
	m_BarcodeResult = BarcodeResult;
}

long CPowerConveyor::GetMesUse()
{
	TRACE(_T("[PWR] GetMesUse Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), m_UseMes);
	return m_UseMes;
}

void CPowerConveyor::SetMesUse(long UseMes)
{
	TRACE(_T("[PWR] SetMesUse Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), UseMes);
	m_UseMes = UseMes;
}

void CPowerConveyor::SetBeltStopState(bool set)
{
	TRACE(_T("[PWR] SetBeltStopState Pos(%d) Conv:%d %d\n"), GetPos(), GetConv(), set);
	m_BeltStop = set;
}

void CPowerConveyor::SetPusherDownRatio(long Ratio)
{
	m_RatioPusherDown = Ratio * 0.1;
}

void CPowerConveyor::SetPusherUpRatio(long Ratio)
{
	m_RatioPusherUp = Ratio * 0.1;
}

double CPowerConveyor::GetPusherDownRatio()
{
	return m_RatioPusherDown;
}

double CPowerConveyor::GetPusherUpRatio()
{
	return m_RatioPusherUp;
}