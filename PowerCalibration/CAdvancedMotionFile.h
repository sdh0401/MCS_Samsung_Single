#pragma once

#include "GlobalDefine.h"
class CAdvancedMotionFile
{
public:
	CAdvancedMotionFile();
	~CAdvancedMotionFile();
	long UseAdvanceMotion();
	long GetPickDelayedMsXY();
	long GetPickDelayedMsZDn();
	long GetPickDelayedMsZUp();
	long GetPickDelayedMsR();
	double GetPickInposXY();
	double GetPickInposZDn();
	double GetPickInposZUp();
	double GetPickInposR();

	long GetRecogDelayedMsXY();
	long GetRecogDelayedMsZDn();
	long GetRecogDelayedMsZUp();
	long GetRecogDelayedMsR();
	double GetRecogInposXY();
	double GetRecogInposZDn();
	double GetRecogInposZUp();
	double GetRecogInposR();

	long GetInsertDelayedMsXY();
	long GetInsertDelayedMsZDn();
	long GetInsertDelayedMsZUp();
	long GetInsertDelayedMsR();
	double GetInsertInposXY();
	double GetInsertInposZDn();
	double GetInsertInposZUp();
	double GetInsertInposR();
	long ReadFile();
	void ClearAdvanceMotion();

private:
	void Initial();
	long Pick(CStdioFile* cFile);
	long Recognition(CStdioFile* cFile);
	long Insert(CStdioFile* cFile);
	long Settling(CStdioFile* cFile);
	long ShortDistance(CStdioFile* cFile);
	long ShortDistVelocity(CStdioFile* cFile);
	long ShortDistAccDec(CStdioFile* cFile);
	long DefaultHeadTorque(CStdioFile* cFile);
	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	long m_UseAdvancedMotion;
	long m_PickXYMs, m_PickZDnMs, m_PickZUpMs, m_PickRMs;
	double m_PickXYInpos, m_PickZDnInpos, m_PickZUpInpos, m_PickRInpos;

	long m_RecogXYMs, m_RecogZDnMs, m_RecogZUpMs, m_RecogRMs;
	double m_RecogXYInpos, m_RecogZDnInpos, m_RecogZUpInpos, m_RecogRInpos;

	long m_InsertXYMs, m_InsertZDnMs, m_InsertZUpMs, m_InsertRMs;
	double m_InsertXYInpos, m_InsertZDnInpos, m_InsertZUpInpos, m_InsertRInpos;

	long m_SettlingTime;
};

extern CAdvancedMotionFile* gcAdvancedMotionFile;