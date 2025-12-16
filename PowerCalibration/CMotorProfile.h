#pragma once

#include "GlobalDefine.h"
class CMotorProfile
{
public:
	CMotorProfile();
	~CMotorProfile();
	long GetFlieLineCount();
	long ReadFile();
	long WriteFile();
	long ModifyFile();
	long ReadProfile(CStdioFile* cFile);
private:
	void Initial();
	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
};

extern CMotorProfile* gcMotorProfile;