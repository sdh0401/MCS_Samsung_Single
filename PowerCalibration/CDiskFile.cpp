#include "pch.h"
#include "CDiskFile.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "LockDef.h"

CDiskFile::CDiskFile()
{
	m_Lock = CreateMutex(NULL, FALSE, NULL);
}

CDiskFile::~CDiskFile()
{
	CloseHandle(m_Lock);
}

void CDiskFile::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CDiskFile::GetFileName()
{
	return m_StrFileName;
}

void CDiskFile::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CDiskFile::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CDiskFile::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CDiskFile::Lock()
{
	SEM_LOCK(m_Lock, INFINITE);
}

void CDiskFile::Unlock()
{
	SEM_UNLOCK(m_Lock);
}

bool CDiskFile::MakeFile()
{
	if (checkFileOpen(m_StrFileName) == false)
	{
		FILE* fp;
		char filename[BUFSIZE];
		ZeroMemory(filename, sizeof(filename));
		CStringA strConverter(m_StrFileName);
		memcpy(filename, strConverter.GetBuffer(), strConverter.GetLength());
		filename[strConverter.GetLength()] = 0;
		fopen_s(&fp, filename, "a");
		if (fp == NULL)
		{
			TRACE(_T("[PWR] MakeInsertEndFile open is null\n"));
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

void CDiskFile::WriteDWordToDISK(unsigned Target, BYTE* addr)
{
	Lock();
	*addr = (BYTE)(Target & 0x0ff);
	addr += DISK_BLOCK;
	*addr = (BYTE)(Target >>= 8) & 0xff;
	addr += DISK_BLOCK;
	*addr = (BYTE)(Target >>= 8) & 0xff;
	addr += DISK_BLOCK;
	*addr = (BYTE)(Target >>= 8) & 0xff;
	Unlock();
}

unsigned CDiskFile::ReadDWordFromDISK(BYTE* addr)
{
	unsigned t1, temp;
	Lock();
	t1 = (int)(*addr & 0xff);
	addr += DISK_BLOCK; temp = (unsigned)(*addr & 0xff);
	t1 = (temp << 8) | t1;
	addr += DISK_BLOCK; temp = (unsigned)(*addr & 0xff);
	t1 = (temp << 16) | t1;
	addr += DISK_BLOCK; temp = (unsigned)(*addr & 0xff);
	t1 = (temp << 24) | t1;
	Unlock();
	return t1;
}

void CDiskFile::WriteDoubleToDISK(double d, BYTE* addr)
{
	char buf[20];
	WORD n;
	Lock();
	sprintf_s(buf, "%7.10f ", d);
	for (n = 0; n < 20; n++)
	{
		*addr = (buf[n] & 0x0ff);
		addr += DISK_BYTE;
	}
	Unlock();
}

double CDiskFile::ReadDoubleFromDISK(BYTE* addr)
{
	char buf[20];
	WORD n;
	double value;
	Lock();
	for (n = 0; n < 20; n++)
	{
		buf[n] = (char)(*addr & 0x0ff);
		addr += DISK_BYTE;
	}
	sscanf_s(buf, "%lf", &value);
	Unlock();
	return value;
}