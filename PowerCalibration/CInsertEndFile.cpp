#include "pch.h"
#include "CInsertEndFile.h"
#include "CApplicationTime.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "CPowerLog.h"
#include "LockDef.h"
#include "Trace.h"

CInsertEndFile* gcInsertEndFile;
CInsertEndFile::CInsertEndFile()
{
	SetDir(_T("C:\\Power\\i6.0\\MCS"));
	SetHeader(_T("i6.0"));
	SetExtension(_T("InsertEnd"));
	MakeFileName();
	if (MakeFile() == true)
	{
		InitializeFile();
		SaveFile();		
	}
}

CInsertEndFile::~CInsertEndFile()
{
}

bool CInsertEndFile::ReadFile(void)
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
		TRACE(_T("[PWR] ReadInsertEndFile open is null\n"));
		return false;
	}
	TRACE(_T("[PWR] DISK Read Flash Data from File(%S)..."), filename);
	fread((LPVOID)INSERTEND_BLOCKDATA, sizeof(char), sizeof(INSERTEND_BLOCKDATA), fp);
	fclose(fp);
	TRACE(_T("Done.\n"));
	return true;
}

bool CInsertEndFile::SaveFile(void)
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
		TRACE(_T("[PWR] SaveInsertEndFile open is null\n"));
		return false;
	}
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("[PWR] DISK Save Flash Data to File(%S)...\n"), filename);
	}
	fwrite(INSERTEND_BLOCKDATA, sizeof(char), sizeof(INSERTEND_BLOCKDATA), fp);
	fclose(fp);
	if (gcPowerLog->IsShowRunLog() == true)
	{
		TRACE(_T("Done.\n"));
	}
	return true;
}

void CInsertEndFile::InitializeFile(void)
{
	Lock();
	ZeroMemory(&INSERTEND_BLOCKDATA, sizeof(INSERTEND_BLOCKDATA));
	Unlock();
}

unsigned CInsertEndFile::GetBaseAddr(int nGantrynPCB, unsigned Block, unsigned Point)
{
	unsigned retAddr = NULL;
	ASSERT(nGantrynPCB <= REAR_CONV_N_REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	retAddr = (MAX_INSERTENDFILE_BLK_PITCH * nGantrynPCB) + (Block * MAXINSERTNO * DISK_PITCH) + (Point * DISK_PITCH);
	Unlock();
	return retAddr;
}

unsigned CInsertEndFile::GetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	unsigned InsertEnd = INSERT_CLEAR;
	BYTE* addr;
	ASSERT(Conv <= REAR_CONV);
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	if (Conv == FRONT_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_FRONT_GANTRY, Block, Point)];
			InsertEnd = ReadDWordFromDISK(addr);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEnd Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_REAR_GANTRY, Block, Point)];
			InsertEnd = ReadDWordFromDISK(addr);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEnd Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::GetInsertEnd Front Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else if (Conv == REAR_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_FRONT_GANTRY, Block, Point)];
			InsertEnd = ReadDWordFromDISK(addr);
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_REAR_GANTRY, Block, Point)];
			InsertEnd = ReadDWordFromDISK(addr);
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::GetInsertEnd Rear Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::GetInsertEnd Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
	return InsertEnd;
}

void CInsertEndFile::SetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	int InsertEnd = INSERT_END;
	BYTE* addr;
	ASSERT(Conv <= REAR_CONV);
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	if (Conv == FRONT_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_FRONT_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEnd Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_REAR_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEnd Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertEnd Front Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else if (Conv == REAR_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_FRONT_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_REAR_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertEnd Rear Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::SetInsertEnd Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
}

void CInsertEndFile::SetInsertStatus(int Conv, int Gantry, unsigned Block, unsigned Point, int Status)
{
	int InsertEnd = Status;
	BYTE* addr;
	ASSERT(Conv <= REAR_CONV);
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	if (Conv == FRONT_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_FRONT_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertStatus Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X Status:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(FRONT_CONV_N_REAR_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
			//if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertStatus Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X Status:%d"),
					Conv, Gantry, Block, Point, addr, InsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertStatus Front Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else if (Conv == REAR_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_FRONT_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = &INSERTEND_BLOCKDATA[GetBaseAddr(REAR_CONV_N_REAR_GANTRY, Block, Point)];
			WriteDWordToDISK(InsertEnd, addr);
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertStatus Rear Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::SetInsertStatus Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
}

void CInsertEndFile::ClearInsertEnd(int Conv)
{
	int iEnd = MAXBLOCKNO;
	long lTimeChk = 0;
	ASSERT(Conv <= REAR_CONV);
	Lock();
	if (Conv == FRONT_CONV)
	{
		for (int indx=0; indx < iEnd; ++indx)
		{
			ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(0, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		}
		for (int indx = 0; indx < iEnd; ++indx)
		{
			ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(1, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		}
	}
	else if (Conv == REAR_CONV)
	{
		for (int indx = 0; indx < iEnd; ++indx)
		{
			ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(2, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		}
		for (int indx = 0; indx < iEnd; ++indx)
		{
			ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(3, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::ClearInsertEnd Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
}

unsigned CInsertEndFile::GetInsertEndToUserMemory(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	unsigned InsertEnd = INSERT_CLEAR;
	unsigned addr;
	unsigned char pInsertEnd = INSERT_CLEAR;
	ASSERT(Conv <= REAR_CONV);
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	if (Conv == FRONT_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = GetBaseAddr(FRONT_CONV_N_FRONT_GANTRY, Block, Point);
			GetUserMemory()->GetMBit(addr, 0, &pInsertEnd);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = GetBaseAddr(FRONT_CONV_N_REAR_GANTRY, Block, Point);
			GetUserMemory()->GetMBit(addr, 0, &pInsertEnd);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::GetInsertEndToUserMemory Front Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else if (Conv == REAR_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = GetBaseAddr(REAR_CONV_N_FRONT_GANTRY, Block, Point);
			GetUserMemory()->GetMBit(addr, 0, &pInsertEnd);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = GetBaseAddr(REAR_CONV_N_REAR_GANTRY, Block, Point);
			GetUserMemory()->GetMBit(addr, 0, &pInsertEnd);
			if (gcPowerLog->IsShowInsertEndLog() == true)
			{
				TRACE(_T("[PWR] GetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::GetInsertEndToUserMemory Rear Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::GetInsertEndToUserMemory Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
	return InsertEnd;
}

void CInsertEndFile::SetInsertEndToUserMemory(int Conv, int Gantry, unsigned Block, unsigned Point)
{
	//int InsertEnd = INSERT_END;
	unsigned addr;
	unsigned char pInsertEnd = INSERT_END;
	ASSERT(Conv <= REAR_CONV);
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Block <= MAXBLOCKNO);
	ASSERT(Point <= MAXINSERTNO);
	Lock();
	if (Conv == FRONT_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = GetBaseAddr(FRONT_CONV_N_FRONT_GANTRY, Block, Point);
			GetUserMemory()->SetMBit(addr, 0, pInsertEnd);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = GetBaseAddr(FRONT_CONV_N_REAR_GANTRY, Block, Point);
			GetUserMemory()->SetMBit(addr, 0, pInsertEnd);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertEndToUserMemory Front Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else if (Conv == REAR_CONV)
	{
		if (Gantry == FRONT_GANTRY)
		{
			addr = GetBaseAddr(REAR_CONV_N_FRONT_GANTRY, Block, Point);
			GetUserMemory()->SetMBit(addr, 0, pInsertEnd);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else if (Gantry == REAR_GANTRY)
		{
			addr = GetBaseAddr(REAR_CONV_N_REAR_GANTRY, Block, Point);
			if (gcPowerLog->IsShowRunLog() == true)
			{
				TRACE(_T("[PWR] SetInsertEndToUserMemory Conv:%d Gantry:%d Block:%d Point:%d addr:0x%X InsertEnd:%d"),
					Conv, Gantry, Block, Point, addr, pInsertEnd);
			}
		}
		else
		{
			TRACE(_T("[PWR] CInsertEndFile::SetInsertEndToUserMemory Rear Conv Invalid Gantry(%d)\n"), Gantry);
		}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::SetInsertEndToUserMemory Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
}

void CInsertEndFile::ClearInsertEndToUserMemory(int Conv)
{
	int iEnd = MAXBLOCKNO;
	long lTimeChk = 0;
	ASSERT(Conv <= REAR_CONV);
	Lock();
	if (Conv == FRONT_CONV)
	{
		//GetUserMemory()->
		//for (int indx = 0; indx < iEnd; ++indx)
		//{
		//	ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(0, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		//}
		//for (int indx = 0; indx < iEnd; ++indx)
		//{
		//	ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(1, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		//}
	}
	else if (Conv == REAR_CONV)
	{
		//for (int indx = 0; indx < iEnd; ++indx)
		//{
		//	ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(2, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		//}
		//for (int indx = 0; indx < iEnd; ++indx)
		//{
		//	ZeroMemory(&INSERTEND_BLOCKDATA[GetBaseAddr(3, 0, 0)], sizeof(char) * MAX_INSERTENDFILE_BLK_PITCH);
		//}
	}
	else
	{
		TRACE(_T("[PWR] CInsertEndFile::ClearInsertEndToUserMemory Invalid Conv(%d)\n"), Conv);
	}
	Unlock();
}