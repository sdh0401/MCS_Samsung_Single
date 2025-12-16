#include "pch.h"
#include "CMachineFile.h"
#include "CPowerCalibrationData.h"
#include "CPowerConveyorData.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"
#include "CPowerLog.h"

CMachineFile* gcMachineFile;
CMachineFile::CMachineFile()
{
	SetDir(_T("C:\\Power\\i6.0\\MCS"));
	SetHeader(_T("i6.0"));
	SetExtension(_T("Setup"));
	MakeFileName();
	if (MakeFile() == true)
	{
		InitializeFile();
		SaveFile();
	}
}

CMachineFile::~CMachineFile()
{
}

void CMachineFile::InitializeFile(void)
{
	Lock();
	for (int indx = 0; indx < MAX_MACHINEFILE_SIZE; ++indx)
	{
		MACHINE_BLOCKDATA[indx] = 0x0;
	}
	Unlock();
}

bool CMachineFile::MakeFile()
{
	if (checkFileOpen(GetFileName()) == false)
	{
		FILE* fp;
		char filename[BUFSIZE];
		ZeroMemory(filename, sizeof(filename));
		CStringA strConverter(GetFileName());
		memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
		filename[strConverter.GetLength()] = 0;
		fopen_s(&fp, filename, "a");
		if (fp == NULL)
		{
			TRACE(_T("[PWR] MakeMachineFile open is null\n"));
			return false;
		}
		fclose(fp);
		return true;
	}
	else
	{
		return false;
	}
}

bool CMachineFile::ReadFile(void)
{
	FILE* fp;
	char filename[BUFSIZE];
	ZeroMemory(filename, sizeof(filename));
	CStringA strConverter(GetFileName());
	memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
	filename[strConverter.GetLength()] = 0;
	fopen_s(&fp, filename, "r");
	if (fp == NULL)
	{
		TRACE(_T("[PWR] ReadMachineFile open is null\n"));
		return false;
	}
	TRACE(_T("[PWR] DISK Read Flash Data from File(%S)..."), filename);
	fread((LPVOID)MACHINE_BLOCKDATA, sizeof(char), sizeof(MACHINE_BLOCKDATA), fp);
	fclose(fp);
	TRACE(_T("Done.\n"));
	return true;
}

bool CMachineFile::SaveFile(void)
{
	FILE* fp;
	char filename[BUFSIZE];
	ZeroMemory(filename, sizeof(filename));
	CStringA strConverter(GetFileName());
	memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
	filename[strConverter.GetLength()] = 0;
	fopen_s(&fp, filename, "w");
	if (fp == NULL)
	{
		TRACE(_T("[PWR] SaveMachineFile open is null\n"));
		return false;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] DISK Save Flash Data to File(%S)...\n"), filename);
	}
	fwrite(MACHINE_BLOCKDATA, sizeof(char), sizeof(MACHINE_BLOCKDATA), fp);
	fclose(fp);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("Done.\n"));
	}
	return true;
}

bool CMachineFile::InitToDiskBlock0()
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_0;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_1 / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_1)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock1()
{
	unsigned char* addr;
	double dblValue = 0.0;
	Lock();
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addr = &MACHINE_BLOCKDATA[DISK_BLOCK_1 + indx * 0x10];
		WriteDoubleToDISK(dblValue, addr);
	}
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addr = &MACHINE_BLOCKDATA[DISK_BLOCK_1 + (indx * 0x10) + (0x10 * BUFSIZE)];
		WriteDoubleToDISK(dblValue, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock2()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0;
	Lock();
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		addrPitch = DISK_BLOCK_2 + (indx * 0x20);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(dblValue, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock3()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0;
	Lock();
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		addrPitch = DISK_BLOCK_3 + (indx * 0x20);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(dblValue, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock4()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0;
	Lock();
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		addrPitch = DISK_BLOCK_4 + (indx * 0x20);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(dblValue, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock5()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0;
	Lock();
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		addrPitch = DISK_BLOCK_5 + (indx * 0x20);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(dblValue, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock6() // Conveyor Option
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_6;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_7 / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_7)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock7() // Conveyor Parameter
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_7;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_8/0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_8)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock8() // Camera Align
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_8;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_9 / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_9)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlock9() // 
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_9;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_A / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_A)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlockA() // 
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_A;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_B / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_B)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlockB() // 
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_B;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_C / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_C)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlockC()
{
	unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	Point_XYRZE pt;
	Lock();
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		ZeroMemory(&pt, sizeof(pt));
		addrPitch = DISK_BLOCK_C + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		WriteDoubleToDISK(pt.r, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		WriteDoubleToDISK(pt.z, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		WriteDWordToDISK(pt.exe, addr);
	}
	addrLast = addrPitch + 0x50;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		ZeroMemory(&pt, sizeof(pt));
		addrPitch = addrLast + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		WriteDoubleToDISK(pt.r, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		WriteDoubleToDISK(pt.z, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		WriteDWordToDISK(pt.exe, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlockD() // 
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_D;
	double Value = 0.0;
	Lock();
	for (unsigned index = 0; index < (DISK_BLOCK_E / 0x10); ++index)
	{
		if (baseAddr + (index * 0x0010) >= DISK_BLOCK_E)
			break;
		addr = &MACHINE_BLOCKDATA[baseAddr + (index * 0x0010)];
		WriteDoubleToDISK(Value, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::InitToDiskBlockF()
{
	//unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	Lock();

	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock0()
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	double dblValue = 0.0;
	double dblValue2 = 0.0;
	long addrPitch = 0, addrLast = 0;
	long lValue = 0, lValue2 = 0;
	Limit limit;
	Point_XY pt;
	Lock();
	addrPitch = DISK_BLOCK_0;
	addr = &MACHINE_BLOCKDATA[addrPitch];
	dblValue = gcPowerCalibrationData->GetHomeShiftDistance(FRONT_GANTRY);
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Front Gantry Addr:0x%X GetHomeShiftDistance:%.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x0010;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = gcPowerCalibrationData->GetHomeShiftDistance(REAR_GANTRY);
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Rear Gantry Addr:0x%X GetHomeShiftDistance:%.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x0020;
	for (int indx = 0; indx < MAXAXISNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		if (gcPowerCalibrationData->GetUseAxis(indx) == true)
			lValue = 1;
		else
			lValue = 0;
		WriteDWordToDISK(lValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock0 Use AxisNo(%d) Addr:0x%X GetUseZAxis:%d\n"), indx, addr, lValue);
		}
	}
	addrLast = addrPitch + 0x0010;
	for (int indx = 0; indx < MAXAXISNO; ++indx)
	{
		limit = gcPowerCalibrationData->GetLimit(indx);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(limit.minus, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(limit.plus, addr2);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock0 GetLimit AxisNo:%04d Addr:0x%X,0x%X Minus,Plus,%.3f,%.3f\n"), indx, addr1, addr2, limit.minus, limit.plus);
		}
	}
	addrLast = addrPitch + 0x0020;
	for (int indx = 0; indx < MAXAXISNO; ++indx)
	{
		dblValue = gcPowerCalibrationData->GetHomePosition(indx);
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock0 GetHomePosition AxisNo:%04d Addr:0x%X HomePosition:%.3f\n"), indx, addr, dblValue);
		}
	}
	addrLast = addrPitch + 0x0010;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetHeadOffset(FRONT_GANTRY, indx + TBL_HEAD1);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock0 Front GetHeadOffset ZAxisNo:%04d Addr:0x%X,0x%X HeadOffsetXY:%.3f,%.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x0020;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetHeadOffset(REAR_GANTRY, indx + TBL_HEAD1);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock0 Rear GetHeadOffset ZAxisNo:%04d Addr:0x%X,0x%X HeadOffsetXY:%.3f,%.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x0020;
	pt = gcPowerCalibrationData->GetHMOffset(FRONT_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Front GetHMOffset Addr:0x%X,0x%X ZHMDOffsetXY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}


	addrLast = addrPitch + 0x0040;
	pt = gcPowerCalibrationData->GetHMOffset(REAR_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Rear GetHMOffset Addr:0x%X,0x%X ZHMDOffsetXY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addrLast = addrPitch + 0x0060;
	lValue = gcPowerCalibrationData->GetHMZero(FRONT_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	WriteDWordToDISK(lValue, addr1);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Front GetHMOffset Addr:0x%X Zero:%d\n"), addr1, lValue);
	}

	lValue = gcPowerCalibrationData->GetHMZero(REAR_GANTRY);
	addr2 = &MACHINE_BLOCKDATA[addrLast+0x10];
	WriteDWordToDISK(lValue, addr2);

	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Rear GetHMOffset Addr:0x%X Zero:%d\n"), addr2, lValue);
	}


	addrLast = addrPitch + 0x0080;

	pt = gcPowerCalibrationData->GetANCMarkPosition(FRONT_STAGE, 0);
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Front ANC Mark1 Addr:0x%X,0x%X XY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	pt = gcPowerCalibrationData->GetANCMarkPosition(FRONT_STAGE, 1);
	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x20];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x30];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Front ANC Mark2 Addr:0x%X,0x%X XY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	pt = gcPowerCalibrationData->GetANCMarkPosition(REAR_STAGE, 0);
	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x40];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x50];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Rear ANC Mark1 Addr:0x%X,0x%X XY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	pt = gcPowerCalibrationData->GetANCMarkPosition(REAR_STAGE, 1);
	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x60];
	WriteDoubleToDISK(pt.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x70];
	WriteDoubleToDISK(pt.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 Rear ANC Mark2 Addr:0x%X,0x%X XY:%.3f,%.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	dblValue = gcPowerCalibrationData->GetANCZPosition(FRONT_STAGE);
	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x80];
	WriteDoubleToDISK(dblValue, addr1);

	dblValue2 = gcPowerCalibrationData->GetANCZPosition(REAR_STAGE);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x90];
	WriteDoubleToDISK(dblValue2, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 ANC Z Addr:0x%X,0x%X Front:%.3f Rear:%.3f\n"), addr1, addr2, dblValue, dblValue2);
	}

	lValue = gcPowerCalibrationData->GetANCRPosition(FRONT_STAGE);
	addr1 = &MACHINE_BLOCKDATA[addrLast + 0xA0];
	WriteDWordToDISK(lValue, addr1);

	lValue2 = gcPowerCalibrationData->GetANCRPosition(REAR_STAGE);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0xB0];
	WriteDWordToDISK(lValue2, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock0 ANC R Addr:0x%X,0x%X Front:%.3f Rear:%.3f\n"), addr1, addr2, lValue, lValue2);
	}


	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock1()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0, addrLast = 0;
	Lock();
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = DISK_BLOCK_1 + (indx * 0x10);
		dblValue = gcPowerCalibrationData->Get1DCompensationData(FRONT_GANTRY, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock1 Front Gantry Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}	
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		dblValue = gcPowerCalibrationData->Get1DCompensationData(REAR_GANTRY, indx);
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		//WriteDoubleToDISK(dblValue, addr);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		//{
		//	TRACE(_T("[PWR] WriteToDiskBlock1 Rear Gantry Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		//}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->Get1DStartCompensationData(FRONT_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock1 Front Gantry Addr:0x%X StartCompensation:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	dblValue = gcPowerCalibrationData->Get1DStartCompensationData(REAR_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	//if (gcPowerLog->IsShowCalibrationLog() == true)
	//{
	//	TRACE(_T("[PWR] WriteToDiskBlock1 Rear Gantry Addr:0x%X StartCompensation:%.3f\n"), addr, dblValue);
	//}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock2()
{
	unsigned char* addr1;
	unsigned char* addr2;
	Point_XYRE pt;
	Point_XY Start;
	long addrPitch = 0, addrLast = 0;
	Lock();
	addrPitch = DISK_BLOCK_2;
	Start = gcPowerCalibrationData->Get2DStartPosition(FRONT_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(Start.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	WriteDoubleToDISK(Start.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock2 Front Gantry Addr:0x%X,0x%X, Start XY %.3f %.3f\n"), addr1, addr2, Start.x, Start.y);
	}
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		ZeroMemory(&pt, sizeof(&pt));
		addrPitch = DISK_BLOCK_2 + 0x20 + (indx * 0x20);
		pt = gcPowerCalibrationData->Get2DCompensationData(FRONT_GANTRY, indx);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock2 Front Gantry Addr:0x%X,0x%X, indx:%d 2DCompensation XY %.3f %.3f\n"), addr1, addr2, indx, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock3()
{
	unsigned char* addr1;
	unsigned char* addr2;
	Point_XYRE pt;
	Point_XY Start;
	long addrPitch = 0, addrLast = 0;
	Lock();
	addrPitch = DISK_BLOCK_3;
	Start = gcPowerCalibrationData->Get2DStartPosition(REAR_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(Start.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	WriteDoubleToDISK(Start.y, addr2);
	//if (gcPowerLog->IsShowCalibrationLog() == true)
	//{
	//	TRACE(_T("[PWR] WriteToDiskBlock3 Rear Gantry Addr:0x%X,0x%X, Start XY %.3f %.3f\n"), addr1, addr2, Start.x, Start.y);
	//}
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		ZeroMemory(&pt, sizeof(&pt));
		addrPitch = DISK_BLOCK_3 + 0x20 + (indx * 0x20);
		pt = gcPowerCalibrationData->Get2DCompensationData(REAR_GANTRY, indx);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		//{
		//	TRACE(_T("[PWR] WriteToDiskBlock3 Rear Gantry Addr:0x%X,0x%X, indx:%d 2DCompensation XY %.3f %.3f\n"), addr1, addr2, indx, pt.x, pt.y);
		//}
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock4()
{
	unsigned char* addr1;
	unsigned char* addr2;
	Point_XY pt;
	long addrPitch = DISK_BLOCK_4, addrLast = 0;
	Lock();
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetCameraRecognitionPosition(FRONT_GANTRY, indx + TBL_HEAD1);
		addrPitch = DISK_BLOCK_4 + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock4 Front GetCameraRecognitionPosition ZAxisNo:%04d Addr:0x%X,0x%X RecogPosition:%.3f,%.3f\n"), 
				indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetCameraRecognitionPosition(REAR_GANTRY, indx + TBL_HEAD1);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock4 Rear GetCameraRecognitionPosition ZAxisNo:%04d Addr:0x%X,0x%X RecogPosition:%.3f,%.3f\n"), 
				indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetCameraRecognitionOffset(FRONT_GANTRY, indx + TBL_HEAD1);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock4 Front GetCameraRecognitionOffset ZAxisNo:%04d Addr:0x%X,0x%X RecogOffset:%.3f,%.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		pt = gcPowerCalibrationData->GetCameraRecognitionOffset(REAR_GANTRY, indx + TBL_HEAD1);
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock4 Rear GetCameraRecognitionOffset ZAxisNo:%04d Addr:0x%X,0x%X RecogOffset:%.3f,%.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock5()
{
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = DISK_BLOCK_5, addrLast = 0;
	Lock();
	Point_XY pt;
	for (int indx = 0; indx < MAXLANENO; ++indx)
	{
		pt = gcPowerCalibrationData->GetPcbFixPosition(FRONT_GANTRY);
		addrPitch = addrPitch + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr2);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlock5 %s GetPcbFixPosition Addr:0x%X,0x%X RecogPosition:%.3f,%.3f\n"), 
				indx == FRONT_CONV ? _T("Front") : _T("Rear"), addr1, addr2, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock6() // Conveyor Option
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_6;
	long Value = 0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = gcPowerConveyorData->GetWidthOption();
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock6 GetWidthOption Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = gcPowerConveyorData->GetPusherZOption();
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock6 GetPusherZOption Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock7() // Conveyor Parameter
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_7;
	double Value = 0.0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = gcPowerConveyorData->GetWidth(FRONT_CONV, ENTRY_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Entry Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = gcPowerConveyorData->GetPusherZ(FRONT_CONV, ENTRY_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Entry PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0020];
	Value = gcPowerConveyorData->GetWidth(REAR_CONV, ENTRY_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Entry Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0030];
	Value = gcPowerConveyorData->GetPusherZ(REAR_CONV, ENTRY_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Entry PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0040];
	Value = gcPowerConveyorData->GetWidth(FRONT_CONV, WORK1_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Work1 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0050];
	Value = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK1_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Work1 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0060];
	Value = gcPowerConveyorData->GetWidth(REAR_CONV, WORK1_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Work1 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0070];
	Value = gcPowerConveyorData->GetPusherZ(REAR_CONV, WORK1_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Work1 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0080];
	Value = gcPowerConveyorData->GetWidth(FRONT_CONV, WORK2_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Work2 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0090];
	Value = gcPowerConveyorData->GetPusherZ(FRONT_CONV, WORK2_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Work2 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00A0];
	Value = gcPowerConveyorData->GetWidth(REAR_CONV, WORK2_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Work2 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00B0];
	Value = gcPowerConveyorData->GetPusherZ(REAR_CONV, WORK2_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Work2 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00C0];
	Value = gcPowerConveyorData->GetWidth(FRONT_CONV, EXIT_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Exit Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00D0];
	Value = gcPowerConveyorData->GetPusherZ(FRONT_CONV, EXIT_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Front Exit PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00E0];
	Value = gcPowerConveyorData->GetWidth(REAR_CONV, EXIT_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Exit Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00F0];
	Value = gcPowerConveyorData->GetPusherZ(REAR_CONV, EXIT_CONV);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock7 Rear Exit PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	WriteDoubleToDISK(Value, addr);
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock8()
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	Point_XY CameraAlign;
	long addrPitch = 0, AlignCamera = 0;
	Lock();
	addrPitch = DISK_BLOCK_8;
	CameraAlign = gcPowerCalibrationData->GetCameraAlignPosition(FRONT_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(CameraAlign.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	WriteDoubleToDISK(CameraAlign.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock8 Front Gantry Addr:0x%X,0x%X, CameraAlign XY %.3f %.3f\n"), addr1, addr2, CameraAlign.x, CameraAlign.y);
	}
	CameraAlign = gcPowerCalibrationData->GetCameraAlignPosition(REAR_GANTRY);
	addr1 = &MACHINE_BLOCKDATA[addrPitch + 0x20];
	WriteDoubleToDISK(CameraAlign.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x30];
	WriteDoubleToDISK(CameraAlign.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock8 Rear Gantry Addr:0x%X,0x%X, CameraAlign XY %.3f %.3f\n"), addr1, addr2, CameraAlign.x, CameraAlign.y);
	}
	AlignCamera = gcPowerCalibrationData->GetAlignCamera(FRONT_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
	WriteDWordToDISK(AlignCamera, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock8 Front Gantry Addr:0x%X AlignCamera:%d\n"), addr, AlignCamera);
	}
	AlignCamera = gcPowerCalibrationData->GetAlignCamera(REAR_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x50];
	WriteDWordToDISK(AlignCamera, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock8 Rear Gantry Addr:0x%X AlignCamera:%d\n"), addr, AlignCamera);
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlock9()
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	Point_XY ReferenceFeeder;
	double FeederPitch = 0.0;
	long addrPitch = 0, ReferenceFeederNo = 0;
	Lock();
	addrPitch = DISK_BLOCK_9;
	ReferenceFeeder = gcPowerCalibrationData->GetReferenceFeederPosition(FRONT_STAGE);
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(ReferenceFeeder.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	WriteDoubleToDISK(ReferenceFeeder.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Front Stage Addr:0x%X,0x%X, ReferenceFeeder XY %.3f %.3f\n"), addr1, addr2, ReferenceFeeder.x, ReferenceFeeder.y);
	}
	ReferenceFeeder = gcPowerCalibrationData->GetReferenceFeederPosition(REAR_STAGE);
	addr1 = &MACHINE_BLOCKDATA[addrPitch + 0x20];
	WriteDoubleToDISK(ReferenceFeeder.x, addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x30];
	WriteDoubleToDISK(ReferenceFeeder.y, addr2);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Rear Stage Addr:0x%X,0x%X, ReferenceFeeder XY %.3f %.3f\n"), addr1, addr2, ReferenceFeeder.x, ReferenceFeeder.y);
	}
	ReferenceFeederNo = gcPowerCalibrationData->GetReferenceFeederNo(FRONT_STAGE);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
	WriteDWordToDISK(ReferenceFeederNo, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Front Stage Addr:0x%X ReferenceFeederNo:%d\n"), addr, ReferenceFeederNo);
	}
	ReferenceFeederNo = gcPowerCalibrationData->GetReferenceFeederNo(REAR_STAGE);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x50];
	WriteDWordToDISK(ReferenceFeederNo, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Rear Stage Addr:0x%X ReferenceFeederNo:%d\n"), addr, ReferenceFeederNo);
	}
	FeederPitch = gcPowerCalibrationData->GetFeederPitch(FRONT_STAGE);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x60];
	WriteDoubleToDISK(FeederPitch, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Front Stage Addr:0x%X FeederPitch:%.3f\n"), addr, FeederPitch);
	}
	FeederPitch = gcPowerCalibrationData->GetFeederPitch(REAR_STAGE);
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x70];
	WriteDoubleToDISK(FeederPitch, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlock9 Rear Stage Addr:0x%X FeederPitch:%.3f\n"), addr, FeederPitch);
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlockA()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0, addrLast = 0;
	Lock();
	addrPitch = DISK_BLOCK_A;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 0);
	addr = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z1 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 0, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z1 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 1);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z2 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 1, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z2 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 2);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z3 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 2, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z3 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 3);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z4 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 3, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z4 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 4);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z5 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 4, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z5 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetZCompensationStartData(FRONT_GANTRY, 5);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockA Front Z6 Addr:0x%X GetZCompensationStartData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		dblValue = gcPowerCalibrationData->GetZCompensationData(FRONT_GANTRY, 5, indx);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(dblValue, addr);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] WriteToDiskBlockA Front Z6 Addr:0x%X indx:%d Compensation:%.3f\n"), addr, indx, dblValue);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlockB()
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0, addrLast = 0;
	Lock();
	addrPitch = DISK_BLOCK_B;
	dblValue = gcPowerCalibrationData->GetSensorOriginHeight(FRONT_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrPitch];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockB Front Addr:0x%X GetSensorOriginHeight:%.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x10;
	dblValue = gcPowerCalibrationData->GetSensorOriginHeight(REAR_GANTRY);
	addr = &MACHINE_BLOCKDATA[addrLast];
	WriteDoubleToDISK(dblValue, addr);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockB Rear Addr:0x%X GetSensorOriginHeight:%.3f\n"), addr, dblValue);
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlockC()
{
	unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	Point_XYRZE pt;
	Lock();
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		pt = gcPowerCalibrationData->GetAgingPosition(FRONT_GANTRY, indx);
		addrPitch = DISK_BLOCK_C + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		WriteDoubleToDISK(pt.r, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		WriteDoubleToDISK(pt.z, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		WriteDWordToDISK(pt.exe, addr);
	}
	addrLast = addrPitch + 0x50;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		pt = gcPowerCalibrationData->GetAgingPosition(REAR_GANTRY, indx);
		addrPitch = addrLast + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		WriteDoubleToDISK(pt.x, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		WriteDoubleToDISK(pt.y, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		WriteDoubleToDISK(pt.r, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		WriteDoubleToDISK(pt.z, addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		WriteDWordToDISK(pt.exe, addr);
	}
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlockD()
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_D;
	long Value = 0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = gcPowerConveyorData->GetInsertDone(FRONT_CONV);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockD GetInsertDone(FRONT) Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = gcPowerConveyorData->GetInsertDone(REAR_CONV);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockD GetInsertDone(REAR) Addr:0x%X Option:%d\n"), addr, Value);
	}
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0020];
	Value = gcPowerConveyorData->GetPcbOutDone(FRONT_CONV);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockD GetPcbOutDone(FRONT) Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0030];
	Value = gcPowerConveyorData->GetPcbOutDone(REAR_CONV);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockD GetPcbOutDone(REAR) Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0040];
	Value = gcPowerConveyorData->GetHeightMeasureDone();
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] WriteToDiskBlockD GetHeightMeasureDone() Addr:0x%X Option:%d\n"), addr, Value);
	}
	WriteDWordToDISK(Value, addr);
	Unlock();
	return true;
}

bool CMachineFile::WriteToDiskBlockF()
{
	Lock();
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock0(void)
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = DISK_BLOCK_0, addrLast = 0;
	double dblValue = 0.0;
	double dblValue2 = 0.0;
	long lValue = 0, lValue2 = 0;
	Limit limit;
	Point_XY pt;
	long maxAxisNo = MAXAXISNO;
	Lock();
	addr = &MACHINE_BLOCKDATA[addrPitch];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadHomeShiftDistance(FRONT_GANTRY, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Front Gantry Addr:0x%X HomeShiftDistance %.3f\n"), addr, dblValue);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadHomeShiftDistance(REAR_GANTRY, dblValue);
	//if (gcPowerLog->IsShowCalibrationLog() == true)
	//{
	//	TRACE(_T("[PWR] ReadToDiskBlock0 Rear Gantry Addr:0x%X HomeShiftDistance %.3f\n"), addr, dblValue);
	//}
	addrLast = DISK_BLOCK_0 + 0x0020;
	for (int indx = 0; indx < MAXAXISNO; ++indx)
	{
		addr = &MACHINE_BLOCKDATA[addrLast + (indx * 0x10)];
		lValue = ReadDWordFromDISK(addr);
		if (lValue != 0 && lValue != 1)
			maxAxisNo = indx + 1;
	}
	for (int indx = 0; indx < maxAxisNo; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		lValue = ReadDWordFromDISK(addr);
		if (lValue == 1)
			gcPowerCalibrationData->LoadUseAxis(indx, true);
		else
			gcPowerCalibrationData->LoadUseAxis(indx, false);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock0 AxisNo(%d) Addr:0x%X LoadUseAxis:%d\n"), indx, addr, lValue);
		}
	}
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < maxAxisNo; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		limit.minus = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		limit.plus = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadLimit(indx, limit);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock0 LoadLimit index:%04d Addr:0x%X,0x%X Minus,Plus,%.3f,%.3f\n"), indx, addr1, addr2, limit.minus, limit.plus);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < maxAxisNo; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadHomePosition(indx, dblValue);		
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock0 LoadHomePosition AxisNo:%04d Addr:0x%X HomePosition:%.3f\n"), indx, addr, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadHeadOffset(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock0 LoadHeadOffset Front ZAxisNo:%04d Addr:0x%X,0x%X HeadOffsetXY %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadHeadOffset(REAR_GANTRY, indx + TBL_HEAD1, pt);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		//{
		//	TRACE(_T("[PWR] ReadToDiskBlock0 LoadHeadOffset Rear ZAxisNo:%04d Addr:0x%X,0x%X HeadOffsetXY %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		//}
	}
	addrLast = addrPitch + 0x20;
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadHMOffset(FRONT_GANTRY, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 LoadHMOffset Front Addr:0x%X,0x%X HMOffsetXY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}
	addrLast = addrPitch + 0x40;
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadHMOffset(REAR_GANTRY, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 LoadHMOffset Rear Addr:0x%X,0x%X HMOffsetXY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addrLast = addrPitch + 0x60;
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	lValue = ReadDWordFromDISK(addr1);
	gcPowerCalibrationData->SetHMZero(FRONT_GANTRY, lValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Front SetHMZero Addr:0x%X Zero %d\n"), addr1, lValue);
	}

	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	lValue = ReadDWordFromDISK(addr2);
	gcPowerCalibrationData->SetHMZero(REAR_GANTRY, lValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Rear SetHMZero Addr:0x%X Zero %d\n"), addr2, lValue);
	}



	addrLast = addrPitch + 0x80;
	addr1 = &MACHINE_BLOCKDATA[addrLast];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x10];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->SetANCMarkPosition(FRONT_STAGE, 0, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Front ANC Mark1 Addr:0x%X,0x%X XY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x20];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x30];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->SetANCMarkPosition(FRONT_STAGE, 1, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Front ANC Mark2 Addr:0x%X,0x%X XY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x40];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x50];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->SetANCMarkPosition(REAR_STAGE, 0, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Rear ANC Mark1 Addr:0x%X,0x%X XY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x60];
	pt.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x70];
	pt.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->SetANCMarkPosition(REAR_STAGE, 1, pt);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Rear ANC Mark2 Addr:0x%X,0x%X XY %.3f %.3f\n"), addr1, addr2, pt.x, pt.y);
	}

	addr1 = &MACHINE_BLOCKDATA[addrLast + 0x80];
	dblValue = ReadDoubleFromDISK(addr1);
	gcPowerCalibrationData->SetANCZPosition(FRONT_STAGE, dblValue);

	addr2 = &MACHINE_BLOCKDATA[addrLast + 0x90];
	dblValue2 = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->SetANCZPosition(REAR_STAGE, dblValue2);

	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 Rear ANC Z Addr:0x%X,0x%X Front:%.3f Rear:%.3f\n"), addr1, addr2, gcPowerCalibrationData->GetANCZPosition(FRONT_STAGE), gcPowerCalibrationData->GetANCZPosition(REAR_STAGE));
	}

	addr1 = &MACHINE_BLOCKDATA[addrLast + 0xA0];
	lValue = ReadDWordFromDISK(addr1);
	gcPowerCalibrationData->SetANCRPosition(FRONT_STAGE, lValue);

	addr2 = &MACHINE_BLOCKDATA[addrLast + 0xB0];
	lValue2 = ReadDWordFromDISK(addr2);
	gcPowerCalibrationData->SetANCRPosition(REAR_STAGE, lValue2);

	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock0 ANC R Addr:0x%X,0x%X Front:%.3f Rear:%.3f\n"), addr1, addr2, gcPowerCalibrationData->GetANCRPosition(FRONT_STAGE), gcPowerCalibrationData->GetANCRPosition(REAR_STAGE));
	}


	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock1(void)
{
	unsigned char* addr;
	double dblValue = 0.0;
	long addrPitch = 0, addrLast = 0;
	Lock();
	addrLast = DISK_BLOCK_1;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->Load1DCompensationData(FRONT_GANTRY, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock1 Front Gantry Addr:0x%X Load1DCompensationData %04d,%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->Load1DCompensationData(REAR_GANTRY, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock1 Rear Gantry Addr:0x%X Load1DCompensationData %04d,%.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->Load1DStartCompensationData(FRONT_GANTRY, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock1 Front Gantry Addr:0x%X Load1DStartCompensationData:%.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->Load1DStartCompensationData(REAR_GANTRY, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock1 Rear Gantry Addr:0x%X Load1DStartCompensationData:%.3f\n"), addr, dblValue);
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock2(void)
{
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = 0;
	Point_XYRE pt;
	Point_XY Start;
	Lock();
	addrPitch = DISK_BLOCK_2;
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	Start.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	Start.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->Load2DStartPosition(FRONT_GANTRY, Start);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock2 Front Gantry Addr:0x%X,0x%X, Start XY %.3f %.3f\n"), addr1, addr2, Start.x, Start.y);
	}
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		ZeroMemory(&pt, sizeof(&pt));
		addrPitch = DISK_BLOCK_2 + 0x20 + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->Load2DCompensationData(FRONT_GANTRY, indx, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock2 Front Load2DCompensationData Addr:0x%X,0x%X, index:%04d XY %.3f %.3f\n"), addr1, addr2, indx, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock3(void)
{
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = 0;
	Point_XYRE pt;
	Point_XY Start;
	Lock();
	addrPitch = DISK_BLOCK_3;
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	Start.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	Start.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->Load2DStartPosition(REAR_GANTRY, Start);
	//if (gcPowerLog->IsShowCalibrationLog() == true)
	//{
	//	TRACE(_T("[PWR] ReadToDiskBlock3 Rear Gantry Addr:0x%X,0x%X Start XY %.3f %.3f\n"), addr1, addr2, Start.x, Start.y);
	//}
	for (int indx = 0; indx < CAL_2D_MAXDISKCOUNT; ++indx)
	{
		ZeroMemory(&pt, sizeof(&pt));
		addrPitch = DISK_BLOCK_3 + 0x20 + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->Load2DCompensationData(REAR_GANTRY, indx, pt);
		//if (gcPowerLog->IsShowCalibrationLog() == true)
		//{
		//	TRACE(_T("[PWR] ReadToDiskBlock3 Rear Load2DCompensationData Addr:0x%X,0x%X index:%04d XY %.3f %.3f\n"), addr1, addr2, indx, pt.x, pt.y);
		//}
	}
	Unlock();
	return true;
}


bool CMachineFile::ReadToDiskBlock4(void)
{
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = DISK_BLOCK_4, addrLast = 0;
	Point_XY pt;
	Lock();
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = DISK_BLOCK_4 + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadCameraRecognitionPosition(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock4 LoadCameraRecognitionPosition Front ZAxisNo:%04d Addr:0x%X,0x%X RecogPosition %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadCameraRecognitionPosition(REAR_GANTRY, indx + TBL_HEAD1, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock4 LoadCameraRecognitionPosition Rear ZAxisNo:%04d Addr:0x%X,0x%X RecogPosition %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadCameraRecognitionOffset(FRONT_GANTRY, indx + TBL_HEAD1, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock4 LoadCameraRecognitionOffset Front ZAxisNo:%04d Addr:0x%X,0x%X RecogOffset %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	addrLast = addrPitch + 0x20;
	for (int indx = 0; indx < MAXUSEDHEADNO; ++indx)
	{
		addrPitch = addrLast + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadCameraRecognitionOffset(REAR_GANTRY, indx + TBL_HEAD1, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock4 LoadCameraRecognitionOffset Rear ZAxisNo:%04d Addr:0x%X,0x%X RecogOffset %.3f %.3f\n"), indx, addr1, addr2, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock5(void)
{
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = DISK_BLOCK_5, addrLast = 0;
	Point_XY pt;
	Lock();
	for (int indx = 0; indx < MAXLANENO; ++indx)
	{
		addrPitch = addrPitch + (indx * 0x20);
		addr1 = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr1);
		addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr2);
		gcPowerCalibrationData->LoadPcbFixPosition(FRONT_GANTRY, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlock5 LoadPcbFixPosition %s Addr:0x%X,0x%X PcbFixPosition %.3f %.3f\n"), indx == FRONT_CONV ? _T("Front") : _T("Rear"),
				addr1, addr2, pt.x, pt.y);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock6(void)
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_6;
	long Value = 0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = ReadDWordFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock6 Width Option:%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidthOption(Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = ReadDWordFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock6 PusherZ Option:%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZOption(Value);
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock7(void)
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_7;
	double Value = 0.0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Entry Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(FRONT_CONV, ENTRY_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Entry PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(FRONT_CONV, ENTRY_CONV, Value);
	
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0020];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Entry Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(REAR_CONV, ENTRY_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0030];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Entry PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(REAR_CONV, ENTRY_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0040];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Work1 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(FRONT_CONV, WORK1_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0050];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Work1 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(FRONT_CONV, WORK1_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0060];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Work1 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(REAR_CONV, WORK1_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0070];
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Work1 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	Value = ReadDoubleFromDISK(addr);
	gcPowerConveyorData->LoadPusherZ(REAR_CONV, WORK1_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0080];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Work2 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(FRONT_CONV, WORK2_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0090];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Work2 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(FRONT_CONV, WORK2_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00A0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Work2 Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(REAR_CONV, WORK2_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00B0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Work2 PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(REAR_CONV, WORK2_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00C0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Exit Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(FRONT_CONV, EXIT_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00D0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Front Exit PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(FRONT_CONV, EXIT_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00E0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Exit Width:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadWidth(REAR_CONV, EXIT_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x00F0];
	Value = ReadDoubleFromDISK(addr);
	if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock7 Rear Exit PusherZ:%.3f Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPusherZ(REAR_CONV, EXIT_CONV, Value);
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock8(void)
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = 0;
	Point_XY CameraAlign;
	long AlignCamera;
	Lock();
	addrPitch = DISK_BLOCK_8;
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	CameraAlign.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	CameraAlign.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadCameraAlignPosition(FRONT_GANTRY, CameraAlign);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock8 Front Gantry Addr:0x%X,0x%X, Align XY %.3f %.3f\n"), addr1, addr2, CameraAlign.x, CameraAlign.y);
	}
	addr1 = &MACHINE_BLOCKDATA[addrPitch + 0x20];
	CameraAlign.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x30];
	CameraAlign.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadCameraAlignPosition(REAR_GANTRY, CameraAlign);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock8 Rear Gantry Addr:0x%X,0x%X, Align XY %.3f %.3f\n"), addr1, addr2, CameraAlign.x, CameraAlign.y);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
	AlignCamera = ReadDWordFromDISK(addr);
	gcPowerCalibrationData->LoadAlignCamera(FRONT_GANTRY, AlignCamera);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock8 Front Gantry Addr:0x%X Align Camera:%d\n"), addr, AlignCamera);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x50];
	AlignCamera = ReadDWordFromDISK(addr);
	gcPowerCalibrationData->LoadAlignCamera(REAR_GANTRY, AlignCamera);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock8 Rear Gantry Addr:0x%X Align Camera:%d\n"), addr, AlignCamera);
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlock9(void)
{
	unsigned char* addr;
	unsigned char* addr1;
	unsigned char* addr2;
	long addrPitch = 0, ReferenceFeederNo = 0;
	Point_XY ReferenceFeeder;	
	double FeederPitch = 0.0;
	Lock();
	addrPitch = DISK_BLOCK_9;
	addr1 = &MACHINE_BLOCKDATA[addrPitch];
	ReferenceFeeder.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x10];
	ReferenceFeeder.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadReferenceFeederPosition(FRONT_GANTRY, ReferenceFeeder);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Front Gantry Addr:0x%X,0x%X, ReferenceFeeder XY %.3f %.3f\n"), addr1, addr2, ReferenceFeeder.x, ReferenceFeeder.y);
	}
	addr1 = &MACHINE_BLOCKDATA[addrPitch + 0x20];
	ReferenceFeeder.x = ReadDoubleFromDISK(addr1);
	addr2 = &MACHINE_BLOCKDATA[addrPitch + 0x30];
	ReferenceFeeder.y = ReadDoubleFromDISK(addr2);
	gcPowerCalibrationData->LoadReferenceFeederPosition(REAR_GANTRY, ReferenceFeeder);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Rear Gantry Addr:0x%X,0x%X, ReferenceFeeder XY %.3f %.3f\n"), addr1, addr2, ReferenceFeeder.x, ReferenceFeeder.y);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
	ReferenceFeederNo = ReadDWordFromDISK(addr);
	gcPowerCalibrationData->LoadReferenceFeederNo(FRONT_GANTRY, ReferenceFeederNo);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Front Gantry Addr:0x%X ReferenceFeederNo:%d\n"), addr, ReferenceFeederNo);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x50];
	ReferenceFeederNo = ReadDWordFromDISK(addr);
	gcPowerCalibrationData->LoadReferenceFeederNo(REAR_GANTRY, ReferenceFeederNo);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Rear Gantry Addr:0x%X ReferenceFeederNo:%d\n"), addr, ReferenceFeederNo);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x60];
	FeederPitch = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadFeederPitch(FRONT_GANTRY, FeederPitch);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Front Gantry Addr:0x%X FeederPitch:%.3f\n"), addr, FeederPitch);
	}
	addr = &MACHINE_BLOCKDATA[addrPitch + 0x70];
	FeederPitch = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadFeederPitch(REAR_GANTRY, FeederPitch);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlock9 Rear Gantry Addr:0x%X FeederPitch:%.3f\n"), addr, FeederPitch);
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlockA(void)
{
	unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	double dblValue = 0.0;
	Lock();
	addrPitch = DISK_BLOCK_A;
	addr = &MACHINE_BLOCKDATA[addrPitch];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 0, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z1 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 0, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z1 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 1, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z2 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 1, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z2 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 2, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z3 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 2, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z3 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 2, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z4 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 3, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z4 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 2, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z5 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 4, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z5 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadZCompensationStartData(FRONT_GANTRY, 2, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z6 Addr:0x%X LoadZCompensationStartData %.3f\n"), addr, dblValue);
	}
	addrLast = addrLast + 0x10;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		addrPitch = addrLast + (indx * 0x10);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		dblValue = ReadDoubleFromDISK(addr);
		gcPowerCalibrationData->LoadZCompensationData(FRONT_GANTRY, 5, indx, dblValue);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			TRACE(_T("[PWR] ReadToDiskBlockA Front Gantry Z6 Addr:0x%X indx:%04d LoadZCompensationData %.3f\n"), addr, indx, dblValue);
		}
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlockB(void)
{
	unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	double dblValue = 0.0;
	Lock();
	addrPitch = DISK_BLOCK_B;
	addr = &MACHINE_BLOCKDATA[addrPitch];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadSensorOriginHeight(FRONT_GANTRY, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockB Front Gantry Addr:0x%X LoadSensorOriginHeight %.3f\n"), addr, dblValue);
	}
	addrLast = addrPitch + 0x10;
	addr = &MACHINE_BLOCKDATA[addrLast];
	dblValue = ReadDoubleFromDISK(addr);
	gcPowerCalibrationData->LoadSensorOriginHeight(REAR_GANTRY, dblValue);
	if (gcPowerLog->IsShowCalibrationLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockB Rear Gantry Addr:0x%X LoadSensorOriginHeight %.3f\n"), addr, dblValue);
	}
	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlockC(void)
{
	unsigned char* addr;
	long addrPitch = 0, addrLast = 0;
	double dblValueX = 0.0, dblValueY = 0.0, dblValueR = 0.0, dblValueZ = 0.0;
	double nValue = 0;
	Point_XYRZE pt;
	Lock();
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		ZeroMemory(&pt, sizeof(pt));
		addrPitch = DISK_BLOCK_C + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		pt.r = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		pt.z = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		pt.exe = ReadDWordFromDISK(addr);
		gcPowerCalibrationData->LoadAgingPosition(FRONT_GANTRY, indx, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			if (pt.exe == 1)
			{
				TRACE(_T("[PWR] ReadToDiskBlockC Front Gantry Aging Position(%d) X,Y,R,Z,Exe %.3f %.3f %.3f %.3f %d\n"), indx, pt.x, pt.y, pt.r, pt.z, pt.exe);
			}
		}
	}
	addrLast = addrPitch + 0x50;
	for (int indx = 0; indx < BUFSIZE; ++indx)
	{
		ZeroMemory(&pt, sizeof(pt));
		addrPitch = addrLast + (indx * 0x50);
		addr = &MACHINE_BLOCKDATA[addrPitch];
		pt.x = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x10];
		pt.y = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x20];
		pt.r = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x30];
		pt.z = ReadDoubleFromDISK(addr);
		addr = &MACHINE_BLOCKDATA[addrPitch + 0x40];
		pt.exe = ReadDWordFromDISK(addr);
		gcPowerCalibrationData->LoadAgingPosition(REAR_GANTRY, indx, pt);
		if (gcPowerLog->IsShowCalibrationLog() == true)
		{
			if (pt.exe == 1)
			{
				TRACE(_T("[PWR] ReadToDiskBlockC Rear Gantry Aging Position(%d) X,Y,R,Z,Exe %.3f %.3f %.3f %.3f %d\n"), indx, pt.x, pt.y, pt.r, pt.z, pt.exe);
			}
		}
	}
	Unlock();
	TRACE(_T("[PWR] ReadToDiskBlockC End Addr:0x%X\n"), addr);
	return true;
}

bool CMachineFile::ReadToDiskBlockD(void)
{
	unsigned char* addr;
	long baseAddr = DISK_BLOCK_D;
	long Value = 0;
	Lock();
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0000];
	Value = ReadDWordFromDISK(addr);
	//if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockD LoadInsertDone(FRONT):%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadInsertDone(FRONT_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0010];
	Value = ReadDWordFromDISK(addr);
	//if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockD LoadInsertDone(REAR):%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadInsertDone(REAR_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0020];
	Value = ReadDWordFromDISK(addr);
	//if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockD LoadPcbOutDone(FRONT):%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPcbOutDone(FRONT_CONV, Value);
	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0030];
	Value = ReadDWordFromDISK(addr);
	//if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockD LoadPcbOutDone(REAR):%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadPcbOutDone(REAR_CONV, Value);

	addr = &MACHINE_BLOCKDATA[baseAddr + 0x0040];
	Value = ReadDWordFromDISK(addr);
	//if (gcPowerLog->IsShowConveyorLog() == true)
	{
		TRACE(_T("[PWR] ReadToDiskBlockD LoadHeightMeasureDone:%d Addr:0x%X\n"), Value, addr);
	}
	gcPowerConveyorData->LoadHeightMeasureDone(Value);

	Unlock();
	return true;
}

bool CMachineFile::ReadToDiskBlockF(void)
{
	unsigned char* addr;
	Lock();
	addr = &MACHINE_BLOCKDATA[DISK_BLOCK_F + 0x00];
	Unlock();
	return true;
}
