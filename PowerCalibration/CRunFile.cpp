#include "pch.h"
#include "CRunFile.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"

CRunFile* gcRunFile;
CRunFile::CRunFile()
{
	SetDir(_T("C:\\Power\\i6.0\\MCS"));
	SetHeader(_T("i6.0"));
	SetExtension(_T("Run"));
	MakeFileName();
	if (MakeFile() == true)
	{
		SaveFile();
		InitializeFile();
	}
}

CRunFile::~CRunFile()
{
}

bool CRunFile::MakeFile()
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
			TRACE(_T("[PWR] MakeRunFile open is null\n"));
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

bool CRunFile::ReadFile(void)
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
		TRACE(_T("[PWR] ReadRunFile open is null\n"));
		return false;
	}
	TRACE(_T("[PWR] DISK Read Flash Data from File(%S)..."), filename);
	fread((LPVOID)RUN_BLOCKDATA, sizeof(char), sizeof(RUN_BLOCKDATA), fp);
	fclose(fp);
	TRACE(_T("Done.\n"));
	return true;
}

bool CRunFile::SaveFile(void)
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
		TRACE(_T("[PWR] SaveRunFile open is null\n"));
		return false;
	}
	TRACE(_T("[PWR] DISK Save Flash Data to File(%S)...\n"), filename);
	fwrite(RUN_BLOCKDATA, sizeof(char), sizeof(RUN_BLOCKDATA), fp);
	fclose(fp);
	TRACE(_T("Done.\n"));
	return true;
}

void CRunFile::InitializeFile(void)
{
	Lock();
	ZeroMemory(&RUN_BLOCKDATA, sizeof(RUN_BLOCKDATA));
	Unlock();
}

unsigned CRunFile::GetBaseAddressNozzlePerHead(unsigned Gantry, unsigned Head)
{
	unsigned retAddr = NULL;
	ASSERT(Gantry <= REAR_GANTRY);
	ASSERT(Head <= MAXUSEDHEADNO);
	Lock();
	Head--;
	retAddr = (Gantry * 0x00A0) + (Head * DISK_PITCH);
	Unlock();
	return retAddr;
}

void CRunFile::WriteNozzleNoPerHead(unsigned Gantry, NozzleNoPerHeadStruct NzlNo)
{
	BYTE* addr;
	int hdinx = 0;
	ASSERT(Gantry <= REAR_GANTRY);
	Lock();
	if (Gantry == FRONT_GANTRY)
	{
		for (int indx = TBL_HEAD1; indx <= MAXUSEDHEADNO; ++indx)
		{
			addr = &RUN_BLOCKDATA[GetBaseAddressNozzlePerHead(FRONT_GANTRY, indx)];
			ASSERT(addr != NULL);
			WriteDWordToDISK(NzlNo.Head[hdinx], addr);
			hdinx++;
		}
	}
	else if (Gantry == REAR_GANTRY)
	{
		for (int indx = TBL_HEAD1; indx <= MAXUSEDHEADNO; ++indx)
		{
			addr = &RUN_BLOCKDATA[GetBaseAddressNozzlePerHead(REAR_GANTRY, indx)];
			ASSERT(addr != NULL);
			WriteDWordToDISK(NzlNo.Head[hdinx], addr);
			hdinx++;
		}
	}
	else
	{
		TRACE(_T("[PWR] CRunFile::WriteNozzleNoPerHead Invalid Gantry:%d\n"), Gantry);
	}
	Unlock();
}

NozzleNoPerHeadStruct CRunFile::ReadNozzleNoPerHead(unsigned Gantry)
{
	ASSERT(Gantry <= REAR_GANTRY);
	NozzleNoPerHeadStruct retNzlNo = NozzleNoPerHeadStruct();
	ZeroMemory(&retNzlNo, sizeof(retNzlNo));
	BYTE* addr;
	int hdinx = 0;
	Lock();
	if (Gantry == FRONT_GANTRY)
	{
		for (int indx = TBL_HEAD1; indx <= MAXUSEDHEADNO; ++indx)
		{
			addr = &RUN_BLOCKDATA[GetBaseAddressNozzlePerHead(FRONT_GANTRY, indx)];
			ASSERT(addr != NULL);
			retNzlNo.Head[hdinx] = ReadDWordFromDISK(addr);
			hdinx++;
		}
	}
	else if (Gantry == REAR_GANTRY)
	{
		for (int indx = TBL_HEAD1; indx <= MAXUSEDHEADNO; ++indx)
		{
			addr = &RUN_BLOCKDATA[GetBaseAddressNozzlePerHead(REAR_GANTRY, indx)];
			ASSERT(addr != NULL);
			retNzlNo.Head[hdinx] = ReadDWordFromDISK(addr);
			hdinx++;
		}
	}
	else
	{
		TRACE(_T("[PWR] CRunFile::ReadNozzleNoPerHead Invalid Gantry:%d\n"), Gantry);
	}
	Unlock();
	return retNzlNo;
}

