#pragma once
#include "GlobalDefine.h"
class CPowerConveyorData
{
public:
	CPowerConveyorData();
	~CPowerConveyorData();
	void LoadWidth(long Conveyor, long Type, double Position);
	void SetWidth(long Conveyor, long Type, double Position);
	double GetWidth(long Conveyor, long Type);
	bool WriteWidth(long Conveyor);
	void LoadPusherZ(long Conveyor, long Type, double Position);
	void SetPusherZ(long Conveyor, long Type, double Position);
	double GetPusherZ(long Conveyor, long Type);
	bool WritePusherZ(long Conveyor);
	double GetMinWidth(long Option);
	double GetMaxWidth(long Option);
	bool IsWidthValidRange(double Width);
	double GetMinPuhserZ(long Option);
	double GetMaxPusherZ(long Option);
	bool IsPusherZValidRange(double PusherZ);
	void LoadWidthOption(long Option);
	void SetWidthOption(long Option);
	long GetWidthOption();
	bool WriteWidthOption();
	void LoadPusherZOption(long Option);
	void SetPusherZOption(long Option);
	long GetPusherZOption();
	bool WritePusherZOption();
	void LoadInsertDone(long Conv, long InsertDone);
	long SetInsertDone(long Conv, long InsertDone);
	long GetInsertDone(long Conv);
	bool WriteInsertDone(long Conv);
	void LoadPcbOutDone(long Conv, long PcbOutDone);
	long SetPcbOutDone(long Conv, long PcbOutDone);
	long GetPcbOutDone(long Conv);
	bool WritePcbOutDone(long Conv);
	void LoadHeightMeasureDone(long done);
	long SetHeightMeasureDone(long done);
	long GetHeightMeasureDone();
	bool WriteHeightMeasureDone();

private:
	double m_FrontConveyorWidth[MAX_CONV];
	double m_RearConveyorWidth[MAX_CONV];
	double m_FrontConveyorPusherZ[MAX_CONV];
	double m_RearConveyorPusherZ[MAX_CONV];
	long m_OptionWidth;							// Single 350 or Dual 250 what else
	long m_OptionPusherZ;						// 
	long m_InsertDone[MAXTABLENO];				// Front & Rear
	long m_PcbOutDone[MAXTABLENO];				// Front & Rear
	long m_HeighitMeasureDone;
};

extern CPowerConveyorData* gcPowerConveyorData;