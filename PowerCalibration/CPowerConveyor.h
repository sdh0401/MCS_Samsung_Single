#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"

class CPowerConveyor : public CPowerThread
{
public:
	CPowerConveyor();
	CPowerConveyor(bool bReverse);
	~CPowerConveyor();
	bool IsSimulationOn();
	void SetSimulationOn(bool SimulationOn);
	void InitVariable();
	void SetReverse(bool bReverse);
	bool BeltOn(long InLowOut);
	bool BeltOn(long InLowOut, long Dir);
	bool BeltOff(long InLowOut);
	bool StartBelt(JogInfo BeldSpd);
	bool StartBelt(JogInfo BeldSpd, double Dir);
	bool StopBelt(double Dec);
	
	bool UpStopper();
	bool DownStopper();

	bool IsExist(long Conv);
	bool IsExistEnt(long Conv);
	bool IsExistSet(long Conv);
	bool GetStateSet(long Conv, long OnOff);
	bool IsExistExit(long Conv);
	bool IsExistLow(long Conv);
	bool IsExistOut(long Conv);
	bool GetStateOut(long Conv, long OnOff);
	bool IsStopperUp(long Conv);
	bool IsStopperDn(long Conv);
	void SetStopperIO(long oUp, long iUp, long oDn, long iDn);
	void SetPusherPlateIO(PusherZType Type, long oUp, long iUp, long oDn, long iDn);
	long CheckPrevSmema(long Conv);
	long CheckNextSmema(long Conv);
	void PrevSmemaOut(long Conv, UBYTE output);
	void NextSmemaOut(long Conv, UBYTE output);
	void InitSmema(long Conv);
	void InitSmema();

	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	void SetMaxFreeTime(long FreeTime);
	long GetMaxFreeTime();
	void StartGetFreeTime();
	void StopGetFreeTime();
	bool IsStartGetFreeTime();
	void EntryLoadingStart();
	void WorkLoadingElapsed();

	void SetTransferTimeOut(long TimeOut);
	long GetTransferTimeOut();
	void SetPrevTimeOut(long TimeOut);
	long GetPrevTimeOut();
	void SetNextTimeOut(long TimeOut);
	long GetNextTimeOut();
	long GetPrevInBeltSpd();
	long GetNextOutBeltSpd();
	JogInfo GetBeltMotorSpeedLow();
	JogInfo GetBeltMotorSpeedMid();
	JogInfo GetBeltMotorSpeedHigh();
	bool GetBeltStopState();
	CString GetBarcode();
	void SetBarcode(CString strBarcode);
	long GetBarcodeType();
	void SetBarcodeType(long BarcodeType);
	long GetBarcodeResult();
	void SetBarcodeResult(long BarcodeResult);
	long GetMesUse();
	void SetMesUse(long UseMes);
	void SetBeltStopState(bool set);
	void SetPusherDownRatio(long Ratio);
	void SetPusherUpRatio(long Ratio);
	double GetPusherDownRatio();
	double GetPusherUpRatio();

	void SetInfo(long Pos, long Conv);
	unsigned GetPos();
	unsigned GetConv();
	void SetBeltMotorType(BeltControl Type);
	BeltControl GetBeltMotorType();
	void SetBeltMotor(CString strName);
	CString GetBeltMotor();
	void SetBeltMotorSpeedLow(JogInfo spd);
	void SetBeltMotorSpeedMid(JogInfo spd);
	void SetBeltMotorSpeedHigh(JogInfo spd);
	void SetPusherPlateZMotor(CString strName);
	CString GetPusherPlateZMotor();
	void SetPusherPlateZControlType(PusherZType type);
	PusherZType GetPusherPlateZControlType();
	void SetConveyorWidthMotor(CString strName);
	CString GetConveyorWidthMotor();
	void SetConveyorWidthMotorType(ConveyorMotorType type);
	ConveyorMotorType GetConveyorWidthMotorType();
	ConveyorStep GetStep();
	bool IsReverse();
	double GetConveyorDir();
	void SetStep(ConveyorStep step);
	void SetPcbReady(bool Ready);
	void SetPcbOut(bool Out);
	bool IsPcbReady();
	bool IsPcbOut();
	void SetPcbInsertDone(bool InsertDone);
	bool IsPcbInsertDone(long Conveyor);
	double GetPcbThickness();
	void SetPcbThickness(double PcbThickness);
	double GetPcbStandByZOffset();
	void SetPcbStandByZOffset(double PcbStandByZOffset);
	void SetTaskID(long ID);
	long GetTaskID(void);
	long GetHighTime();
	void SetHighTime(long HighTime);
	long GetMiddleTime();
	void SetMiddleTime(long MiddleTime);
	long GetLowTime();
	void SetLowTime(long LowTime);
	long GetFromPcb();
	void SetFromPcb(long From);
	void SetConveyorSpeed(long PrevInBeltSpd, long NextOutBeltSpd);

private:
	CString m_Barcode;
	long m_UseMes;
	long m_BarcodeType;
	long m_BarcodeResult;
	BeltControl m_BeltControlType;
	CString m_BeltMotorName;

	PusherZType m_PusherZControlType;
	CString m_PusherZMotorName;

	ConveyorMotorType m_ConveyorWidthType;
	CString m_ConveyorWidthMotorName;
	
	ConveyorStruct m_Conveyor;
	ConveyorStep m_Step;
	double m_BeltHWMotorDir;
	bool m_Reverse;
	bool m_PcbReady, m_OldPcbReady;
	bool m_PcbOut, m_OldPcbOut;
	bool m_PcbInsertDone;
	bool m_IsSimulationOn;
	long m_ShowID;
	double m_PcbThickness;
	double m_PcbStandByZOffset;
	long m_HighTime;
	long m_MiddleTime;
	long m_LowTime;
	long m_ProdRunMode;
	long m_FromPcb;
	long m_FreeTime;
	bool m_bStartFreeTime;
	long m_TransferTimeOut;
	long m_PrevTransferTimeOut;
	long m_NextTransferTimeOut;
	long m_PrevInBeltSpd;
	long m_NextOutBeltSpd;
	bool m_BeltStop;
	double m_RatioPusherUp, m_RatioPusherDown;
};