#pragma once
#include "GlobalDefine.h"
class CBarcodeFile
{
public:
	CBarcodeFile();
	~CBarcodeFile();
	long ReadFile();
	long WriteFile();
	bool MakeFile();
	long ReadBarcode(CStdioFile* cFile);
	void SetBarcode(long Conv, CString Barcode);
	CString GetBarcode(long Conv);
	
private:
	void Initial();
	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	CString m_EntryBarcode, m_WorkBarcode, m_ExitBarcode;
};

extern CBarcodeFile* gcBarcodeFile;
