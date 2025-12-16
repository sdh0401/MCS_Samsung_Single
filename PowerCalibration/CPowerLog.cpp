#include "pch.h"
#include "CPowerLog.h"
#include "GlobalData.h"
#include "TRACE.h"

CPowerLog* gcPowerLog;

CPowerLog::CPowerLog()
{
	m_HomingLog = false;
	m_MotionLog = false;
	m_ConveyorLog = false;
	m_RunLog = false;
	m_VisionLog = false;
	m_CommunicationLog = false;
	m_CalibrationLog = false;
	m_PcbSensorLog = false;
	m_SerialLog = false;
	m_CompensationLog = false;
	m_IoLog = false;
	m_FeederLog = false;
	m_ShortDistLog = false;
	m_MotionLockLog = false;
	m_TowerLampLog = false;
	m_ElapsedLog = false;
	m_InsertEndLog = false;
	m_AnalogLog = false;
	InitializeLog();
}
CPowerLog::~CPowerLog() 
{
	delete g_LogWriter;
}

void CPowerLog::InitializeLog()
{
	CString strLog;
	g_strRoot = StartupPath();
	strLog.Format(_T("%s\\LOG\\"), (LPCTSTR)g_strRoot);
	g_LogWriter = new CLogFileSystem();
	g_LogWriter->InitializeLogging(strLog, _T("log"), _T("CALMCS"));
	g_LogWriter->SetLogUse(TRUE);
	StartLogging();
}

void CPowerLog::StartLogging()
{
	CString strLog;
	strLog.Empty();
	strLog.Format(_T("------------------------------------------------------------------------------------"));
	Logging(strLog);
	strLog.Empty();
	strLog.Format(_T("[PWR] Start to Logging"));
	Logging(strLog);
}

CString CPowerLog::StartupPath()
{
	wchar_t module_name[MAX_PATH];
	GetModuleFileName(0, module_name, MAX_PATH);
	CString path(module_name);
	int k = path.ReverseFind('\\');
	CString strResult = _T("");
	if (k != -1) strResult = path.Left(k);
	return strResult;
}

void CPowerLog::Logging(CString strLog)
{
	if (strLog.GetLength() > 0)
	{
		if (g_LogWriter != NULL)
		{
			g_LogWriter->AddLogMsg(strLog);
		}
	}
}

bool CPowerLog::IsShowHomingLog()
{
	return m_HomingLog;;
}

void CPowerLog::SetShowHomingLog(bool bShow)
{
	m_HomingLog = bShow;
}

bool CPowerLog::IsShowCalibrationLog()
{
	return m_CalibrationLog;;
}

void CPowerLog::SetShowCalibrationLog(bool bShow)
{
	m_CalibrationLog = bShow;
}

bool CPowerLog::IsShowCommunicationLog()
{
	return m_CommunicationLog;
}

void CPowerLog::SetShowCommunicationLog(bool bShow)
{
	m_CommunicationLog = bShow;
}

bool CPowerLog::IsShowMotionLog()
{
	return m_MotionLog;
}

void CPowerLog::SetShowMotionLog(bool bShow)
{
	m_MotionLog = bShow;
}

bool CPowerLog::IsShowConveyorLog()
{
	return m_ConveyorLog;
}

void CPowerLog::SetShowConveyorLog(bool bShow)
{
	m_ConveyorLog = bShow;
}

bool CPowerLog::IsShowRunLog()
{
	return m_RunLog;
}

void CPowerLog::SetShowRunLog(bool bShow)
{
	m_RunLog = bShow;
}

bool CPowerLog::IsShowVisionLog()
{
	return m_VisionLog;
}

void CPowerLog::SetShowVisionLog(bool bShow)
{
	m_VisionLog = bShow;
}

bool CPowerLog::IsShowPcbSensorLog()
{
	return m_PcbSensorLog;
}

void CPowerLog::SetShowPcbSensorLog(bool bShow)
{
	m_PcbSensorLog = bShow;
}

bool CPowerLog::IsShowSerialLog()
{
	return m_SerialLog;
}

void CPowerLog::SetShowSerialLog(bool bShow)
{
	m_SerialLog = bShow;
}

bool CPowerLog::IsShowCompensationLog()
{
	return m_CompensationLog;
}

void CPowerLog::SetShowCompensationLog(bool bShow)
{
	m_CompensationLog = bShow;
}

bool CPowerLog::IsShowIoLog()
{
	return m_IoLog;
}

void CPowerLog::SetShowIoLog(bool bShow)
{
	m_IoLog = bShow;
}

bool CPowerLog::IsShowShortDistLog()
{
	return m_ShortDistLog;
}

void CPowerLog::SetShowShortDistLog(bool bShow)
{
	m_ShortDistLog = bShow;
}

bool CPowerLog::IsShowMotionLockLog()
{
	return m_MotionLockLog;
}

void CPowerLog::SetShowMotionLockLog(bool bShow)
{
	m_MotionLockLog = bShow;
}

bool CPowerLog::IsShowTowerLampLog()
{
	return m_TowerLampLog;
}

void CPowerLog::SetShowTowerLampLog(bool bShow)
{
	m_TowerLampLog = bShow;
}

bool CPowerLog::IsShowFeederLog()
{
	return m_FeederLog;
}

void CPowerLog::SetShowFeederLog(bool bShow)
{
	m_FeederLog = bShow;
}

bool CPowerLog::IsShowElapsedLog()
{
	return m_ElapsedLog;
}

void CPowerLog::SetShowElapsedLog(bool bShow)
{
	m_ElapsedLog = bShow;
}

bool CPowerLog::IsShowInsertEndLog()
{
	return m_InsertEndLog;
}

void CPowerLog::SetShowInsertEndLog(bool bShow)
{
	m_InsertEndLog = bShow;
}

bool CPowerLog::IsShowAnalogLog()
{
	return m_AnalogLog;
}

void CPowerLog::SetShowAnalogLog(bool bShow)
{
	m_AnalogLog = bShow;
}
