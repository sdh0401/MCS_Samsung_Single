#pragma once
class CPowerLog
{
public:
	CPowerLog();
	~CPowerLog();
	
	bool IsShowHomingLog();
	void SetShowHomingLog(bool bShow);

	bool IsShowCalibrationLog();
	void SetShowCalibrationLog(bool bShow);

	bool IsShowCommunicationLog();
	void SetShowCommunicationLog(bool bShow);

	bool IsShowMotionLog();
	void SetShowMotionLog(bool bShow);

	bool IsShowConveyorLog();
	void SetShowConveyorLog(bool bShow);

	bool IsShowRunLog();
	void SetShowRunLog(bool bShow);

	bool IsShowVisionLog();
	void SetShowVisionLog(bool bShow);
	
	bool IsShowPcbSensorLog();
	void SetShowPcbSensorLog(bool bShow);

	bool IsShowSerialLog();
	void SetShowSerialLog(bool bShow);

	bool IsShowCompensationLog();
	void SetShowCompensationLog(bool bShow);

	bool IsShowIoLog();
	void SetShowIoLog(bool bShow);

	bool IsShowShortDistLog();
	void SetShowShortDistLog(bool bShow);

	bool IsShowMotionLockLog();
	void SetShowMotionLockLog(bool bShow);

	bool IsShowTowerLampLog();
	void SetShowTowerLampLog(bool bShow);

	bool IsShowFeederLog();
	void SetShowFeederLog(bool bShow);

	bool IsShowElapsedLog();
	void SetShowElapsedLog(bool bShow);

	bool IsShowInsertEndLog();
	void SetShowInsertEndLog(bool bShow);

	bool IsShowAnalogLog();
	void SetShowAnalogLog(bool bShow);

	void Logging(CString strLog);

private:
	bool m_HomingLog;
	bool m_MotionLog;
	bool m_ConveyorLog;
	bool m_RunLog;
	bool m_VisionLog;
	bool m_CommunicationLog;
	bool m_CalibrationLog;
	bool m_PcbSensorLog;
	bool m_SerialLog;
	bool m_CompensationLog;
	bool m_IoLog;
	bool m_FeederLog;
	bool m_ShortDistLog;
	bool m_MotionLockLog;
	bool m_TowerLampLog;
	bool m_ElapsedLog;
	bool m_InsertEndLog;
	bool m_AnalogLog;
	void InitializeLog();
	void StartLogging();
	CString StartupPath();
};

extern CPowerLog* gcPowerLog;