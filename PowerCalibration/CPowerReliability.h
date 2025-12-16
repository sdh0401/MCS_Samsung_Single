#pragma once
#include "GlobalDefine.h"

class CPowerReliability
{
public:
	CPowerReliability();
	~CPowerReliability();
	CString GetFileName();
	bool MakeFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	long CreateUserFile();
	
	double CalculateSD(double data[], long MaxCount);
	int AddUserFile(long index, double RawData1);
	int AddUserFile(long index, double RawData1, double RawData2);
	int AddUserFile(long index, double RawData1, double RawData2, double RawData3);
	int AddUserFile(long index, double RawData1, double RawData2, double RawData3, double RawData4);
	void MakeStdev(CString strFileName, double* raw1, long MaxCount);
	void MakeStdev(CString strFileName, double* raw1, double* raw2, long MaxCount);
	void MakeStdev(CString strFileName, double* raw1, double* raw2, double* raw3, long MaxCount);
	void MakeStdev(CString strFileName, double* raw1, double* raw2, double* raw3, double* raw4, long MaxCount);
	void MakeStdev(CString strFileName, long MaxCount);

	void InitRaw();
	void SetRaw(long index, double RawData1);
	void SetRaw(long index, double RawData1, double RawData2);
	void SetRaw(long index, double RawData1, double RawData2, double RawData3);
	void SetRaw(long index, double RawData1, double RawData2, double RawData3, double RawData4);

private:
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	double m_raw1[BUFSIZE], m_raw2[BUFSIZE], m_raw3[BUFSIZE], m_raw4[BUFSIZE];
};

extern CPowerReliability* gcPowerReliability;