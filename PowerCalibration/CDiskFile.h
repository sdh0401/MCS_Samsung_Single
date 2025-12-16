#pragma once
class CDiskFile
{
public:
	CDiskFile();
	~CDiskFile();
	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	bool MakeFile();
	void Lock();
	void Unlock();
	void WriteDWordToDISK(unsigned Target, BYTE* addr);
	unsigned ReadDWordFromDISK(BYTE* addr);
	void WriteDoubleToDISK(double d, BYTE* addr);
	double ReadDoubleFromDISK(BYTE* addr);

private:
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	HANDLE m_Lock;
};

