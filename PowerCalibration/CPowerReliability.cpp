#include "pch.h"
#include "CPowerReliability.h"
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "TRace.h"
#include <cmath>
using namespace std;

CPowerReliability* gcPowerReliability;
CPowerReliability::CPowerReliability()
{
	SetDir(_T("C:\\Power\\i6.0\\Report"));
	SetExtension(_T("Report"));
	InitRaw();
}

CPowerReliability::~CPowerReliability()
{

}

double CPowerReliability::CalculateSD(double data[], long MaxCount)
{
	double sum = 0.0, mean, standardDeviation = 0.0;
	long i, Max = MaxCount;
	for (i = 0; i < Max; ++i)
	{
		sum += data[i];
	}
	mean = sum / Max;
	for (i = 0; i < Max; ++i)
	{
		standardDeviation += pow(data[i] - mean, 2);
	}
	return sqrt(standardDeviation / Max);
}

void CPowerReliability::MakeStdev(CString strFileName, double* raw1, long MaxCount)
{
	if (strFileName.GetLength() > 0)
	{
		SetHeader(strFileName);
		if (MakeFileName() == true)
		{
			TRACE(_T("[PWR] MakeStdev FileName:%s\n"), m_StrFileName);
			CreateUserFile();
			for (long count = 0; count < MaxCount; ++count)
			{
				AddUserFile(count, raw1[count]);
			}
			raw1[BUFSIZE - 1] = CalculateSD(raw1, MaxCount) * CAL_3SIGMA;
			TRACE(_T("[PWR] Stdev Raw(1) %.3f\n"), raw1[BUFSIZE - 1]);
			AddUserFile(MaxCount, raw1[BUFSIZE - 1]);
		}
	}
}

void CPowerReliability::MakeStdev(CString strFileName, double* raw1, double* raw2, long MaxCount)
{
	if (strFileName.GetLength() > 0)
	{
		SetHeader(strFileName);
		if (MakeFileName() == true)
		{
			TRACE(_T("[PWR] MakeStdev FileName:%s\n"), m_StrFileName);
			CreateUserFile();
			for (long count = 0; count < MaxCount; ++count)
			{
				AddUserFile(count, raw1[count], raw2[count]);
			}
			raw1[BUFSIZE - 1] = CalculateSD(raw1, MaxCount) * CAL_3SIGMA;
			raw2[BUFSIZE - 1] = CalculateSD(raw2, MaxCount) * CAL_3SIGMA;
			TRACE(_T("[PWR] Stdev Raw(1, 2) %.3f %.3f\n"), raw1[BUFSIZE - 1], raw2[BUFSIZE - 1]);
			AddUserFile(MaxCount, raw1[BUFSIZE - 1], raw2[BUFSIZE - 1]);
		}
	}
}

void CPowerReliability::MakeStdev(CString strFileName, double* raw1, double* raw2, double* raw3, long MaxCount)
{
	if (strFileName.GetLength() > 0)
	{
		SetHeader(strFileName);
		if (MakeFileName() == true)
		{
			TRACE(_T("[PWR] MakeStdev FileName:%s\n"), m_StrFileName);
			CreateUserFile();
			for (long count = 0; count < MaxCount; ++count)
			{
				AddUserFile(count, raw1[count], raw2[count], raw3[count]);
			}
			raw1[BUFSIZE - 1] = CalculateSD(raw1, MaxCount) * CAL_3SIGMA;
			raw2[BUFSIZE - 1] = CalculateSD(raw2, MaxCount) * CAL_3SIGMA;
			raw3[BUFSIZE - 1] = CalculateSD(raw3, MaxCount) * CAL_3SIGMA;
			TRACE(_T("[PWR] Stdev Raw(1, 2, 3) %.3f %.3f %.3f\n"), raw1[BUFSIZE - 1], raw2[BUFSIZE - 1], raw3[BUFSIZE - 1]);
			AddUserFile(MaxCount, raw1[BUFSIZE - 1], raw2[BUFSIZE - 1], raw3[BUFSIZE - 1]);
		}
	}
}

void CPowerReliability::MakeStdev(CString strFileName, double* raw1, double* raw2, double* raw3, double* raw4, long MaxCount)
{
	if (strFileName.GetLength() > 0)
	{
		SetHeader(strFileName);
		if (MakeFileName() == true)
		{
			TRACE(_T("[PWR] MakeStdev FileName:%s\n"), m_StrFileName);
			CreateUserFile();
			for (long count = 0; count < MaxCount; ++count)
			{
				AddUserFile(count, raw1[count], raw2[count], raw3[count], raw4[count]);
			}
			raw1[BUFSIZE - 1] = CalculateSD(raw1, MaxCount) * CAL_3SIGMA;
			raw2[BUFSIZE - 1] = CalculateSD(raw2, MaxCount) * CAL_3SIGMA;
			raw3[BUFSIZE - 1] = CalculateSD(raw3, MaxCount) * CAL_3SIGMA;
			raw4[BUFSIZE - 1] = CalculateSD(raw4, MaxCount) * CAL_3SIGMA;
			TRACE(_T("[PWR] Stdev Raw(1, 2, 3, 4) %.3f %.3f %.3f %.3f\n"), raw1[BUFSIZE - 1], raw2[BUFSIZE - 1], raw3[BUFSIZE - 1], raw4[BUFSIZE - 1]);
			AddUserFile(MaxCount, raw1[BUFSIZE - 1], raw2[BUFSIZE - 1], raw3[BUFSIZE - 1], raw4[BUFSIZE - 1]);
		}
	}
}

void CPowerReliability::MakeStdev(CString strFileName, long MaxCount)
{
	if (strFileName.GetLength() > 0)
	{
		SetHeader(strFileName);
		if (MakeFileName() == true)
		{
			TRACE(_T("[PWR] MakeStdev FileName:%s\n"), m_StrFileName);
			CreateUserFile();
			for (long count = 0; count < MaxCount; ++count)
			{
				AddUserFile(count, m_raw1[count], m_raw2[count], m_raw3[count], m_raw4[count]);
			}
			m_raw1[BUFSIZE - 1] = CalculateSD(m_raw1, MaxCount) * CAL_3SIGMA;
			m_raw2[BUFSIZE - 1] = CalculateSD(m_raw2, MaxCount) * CAL_3SIGMA;
			m_raw3[BUFSIZE - 1] = CalculateSD(m_raw3, MaxCount) * CAL_3SIGMA;
			m_raw4[BUFSIZE - 1] = CalculateSD(m_raw4, MaxCount) * CAL_3SIGMA;
			TRACE(_T("[PWR] Stdev Raw(1, 2, 3, 4) %.3f %.3f %.3f %.3f\n"), m_raw1[BUFSIZE - 1], m_raw2[BUFSIZE - 1], m_raw3[BUFSIZE - 1], m_raw4[BUFSIZE - 1]);
			AddUserFile(MaxCount, m_raw1[BUFSIZE - 1], m_raw1[BUFSIZE - 1], m_raw1[BUFSIZE - 1], m_raw1[BUFSIZE - 1]);
		}
	}
}

bool CPowerReliability::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s_%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)GetCurrentDateTime(), (LPCTSTR)m_StrExtension);
	if (m_StrFileName.GetLength() > 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

CString CPowerReliability::GetFileName()
{
	return m_StrFileName;
}

void CPowerReliability::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CPowerReliability::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CPowerReliability::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

long CPowerReliability::CreateUserFile()
{
	CString strFile_Location = GetFileName();
	if (FileExist(strFile_Location) == false)
	{
		CFile myFile;
		CFileException fileException;
		if (!myFile.Open(strFile_Location, CFile::modeCreate | CFile::shareDenyNone, &fileException))
		{
			TRACE(_T("Can't open file %s, error = %u\n"), strFile_Location, fileException.m_cause);
		}
		myFile.Close();
	}
	return 0;
}

int CPowerReliability::AddUserFile(long index, double RawData1)
{
	CString str;
	CStringA strA;

	str.Format(_T("%04d %.3f\n"), index, RawData1);
	strA = CStringA(str);

	char chArray[1024] = { 0 };
	memcpy(chArray, strA.GetBuffer(), strA.GetLength());

	CFile myFile;
	CFileException fileException;
	CString strFile_Location = GetFileName();
	if (!myFile.Open(strFile_Location, CFile::modeWrite, &fileException))
	{
		TRACE(_T("Can't open file %s, error = %u\n"), strFile_Location, fileException.m_cause);
	}
	ULONGLONG value = myFile.SeekToEnd();
	myFile.Write(chArray, strA.GetLength());
	myFile.Close();
	return 0;
}

int CPowerReliability::AddUserFile(long index, double RawData1, double RawData2)
{
	CString str;
	CStringA strA;

	str.Format(_T("%04d %.3f %.3f\n"), index, RawData1, RawData2);
	strA = CStringA(str);

	char chArray[1024] = { 0 };
	memcpy(chArray, strA.GetBuffer(), strA.GetLength());

	CFile myFile;
	CFileException fileException;
	CString strFile_Location = GetFileName();
	if (!myFile.Open(strFile_Location, CFile::modeWrite, &fileException))
	{
		TRACE(_T("Can't open file %s, error = %u\n"), strFile_Location, fileException.m_cause);
	}
	ULONGLONG value = myFile.SeekToEnd();
	myFile.Write(chArray, strA.GetLength());
	myFile.Close();
	return 0;
}

int CPowerReliability::AddUserFile(long index, double RawData1, double RawData2, double RawData3)
{
	CString str;
	CStringA strA;

	str.Format(_T("%04d %.3f %.3f %.3f\n"), index, RawData1, RawData2, RawData3);
	strA = CStringA(str);

	char chArray[1024] = { 0 };
	memcpy(chArray, strA.GetBuffer(), strA.GetLength());

	CFile myFile;
	CFileException fileException;
	CString strFile_Location = GetFileName();
	if (!myFile.Open(strFile_Location, CFile::modeWrite, &fileException))
	{
		TRACE(_T("Can't open file %s, error = %u\n"), strFile_Location, fileException.m_cause);
	}
	ULONGLONG value = myFile.SeekToEnd();
	myFile.Write(chArray, strA.GetLength());
	myFile.Close();
	return 0;
}

int CPowerReliability::AddUserFile(long index, double RawData1, double RawData2, double RawData3, double RawData4)
{
	CString str;
	CStringA strA;
	
	str.Format(_T("%04d %.3f %.3f %.3f %.3f\n"), index, RawData1, RawData2, RawData3, RawData4);
	strA = CStringA(str);

	char chArray[1024] = { 0 };
	memcpy(chArray, strA.GetBuffer(), strA.GetLength());

	CFile myFile;
	CFileException fileException;
	CString strFile_Location = GetFileName();
	if (!myFile.Open(strFile_Location, CFile::modeWrite, &fileException))
	{
		TRACE(_T("Can't open file %s, error = %u\n"), strFile_Location, fileException.m_cause);
	}
	ULONGLONG value = myFile.SeekToEnd();
	myFile.Write(chArray, strA.GetLength());
	myFile.Close();
	return 0;
}

void CPowerReliability::InitRaw()
{
	ZeroMemory(&m_raw1, sizeof(m_raw1));
	ZeroMemory(&m_raw2, sizeof(m_raw2));
	ZeroMemory(&m_raw3, sizeof(m_raw3));
	ZeroMemory(&m_raw4, sizeof(m_raw4));
}

void CPowerReliability::SetRaw(long index, double RawData1)
{
	m_raw1[index] = RawData1;
}

void CPowerReliability::SetRaw(long index, double RawData1, double RawData2)
{
	m_raw1[index] = RawData1;
	m_raw2[index] = RawData2;
}

void CPowerReliability::SetRaw(long index, double RawData1, double RawData2, double RawData3)
{
	m_raw1[index] = RawData1;
	m_raw2[index] = RawData2;
	m_raw3[index] = RawData3;
}

void CPowerReliability::SetRaw(long index, double RawData1, double RawData2, double RawData3, double RawData4)
{
	m_raw1[index] = RawData1;
	m_raw2[index] = RawData2;
	m_raw3[index] = RawData3;
	m_raw4[index] = RawData4;
}
