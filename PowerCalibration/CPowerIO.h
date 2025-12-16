#pragma once
#include "GlobalDefine.h"

class CPowerIO
{
public:
	CPowerIO(bool bReverse);
	~CPowerIO();
	void ReadIOConfig(bool bReverse);
	UBYTE di1(long no);
	ULONGLONG di1ElapsedTime(long port, long onoff);
	ULONGLONG do1ElapsedTime(long port, long onoff);
	void do1(long no, long onoff);
	UBYTE Getdo1(long no);
	bool dit1(long port, long onoff, long dwTime);
	bool Getdot1(long port, long onoff, long dwTime);
	long GetReadyIONoFromReadyNo(long ReadyNo);
	long GetReleaseIONoFromReleaseNo(long ReleaseNo);
	long GetIOUsage(long No);
	long GetIOType(long No);
	long GetIOAddr(long No);
	char GetIOBit(long No);
    long IsOutput(long No);
	void SetAnalogInput(long No, long Level);
	long GetAnalogInput(long No);
	long GetAnalogLevel(long Gantry, long HeadNo);
	long GetAnalogLevel(long IONum);
	double GetHeight(long Gantry);
	double GetTemperature(CString strAxis);
	long GetInOutType(long PortNo);
	bool IsUsedIO(long inout, long addr, unsigned char bit, CString* description);
	bool IsUsedIO(const int IONUMBER_ON_IODEFINE_HEADER) const;

private:
	void ReadAncIO();
	void ReadDoorIO();
	void ReadSmemaIO();
	void ReadTowerIO();
	void ReadSafetyIO();
	void ReadFeederIO();
	void ReadUserKeyIO();
	void ReadHeadAnalogIO();
	void ReadHeadSuctionBlowIO();
	void ReadHeightMeasurementIO();
	void ReadConveyorIO(bool bReverse);
	void ReadMachineEnvironmentIO();
	void ReadLaserIO();
	void ReadPusherPlateIO();
	void ReadFormingDRBCoilIO();
	IOStruct m_IOConfig[MAXIODEFINE];
	bool m_Simulation[MAXIODEFINE];
	void ReadIOMapRSA1st();
	void EditIOAddress(long addr, unsigned char bit, long ioNum);
	long m_IOVer;

	void ReadAncIO_Weid1st();
	void ReadDoorIO_Weid1st();
	void ReadSmemaIO_Weid1st();
	void ReadTowerIO_Weid1st();
	void ReadSafetyIO_Weid1st();
	void ReadFeederIO_Weid1st();
	void ReadUserKeyIO_Weid1st();
	void ReadHeadAnalogIO_Weid1st();
	void ReadHeadSuctionBlowIO_Weid1st();
	void ReadHeightMeasurementIO_Weid1st();
	void ReadConveyorIO_Weid1st(bool bReverse);
	void ReadMachineEnvironmentIO_Weid1st();
	void ReadLaserIO_Weid1st();
	void ReadPusherPlateIO_Weid1st();
};

extern CPowerIO* gcPowerIO;
