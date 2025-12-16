#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"

class CPcbExist : public CPowerThread
{
public:
	CPcbExist();
	~CPcbExist();
	void Run();
	bool IsExist(long Conv);
	void SetPCBSensorIO(long Conv, long Entry, long Low, long Exist, long Out, long Exit);
	bool IsExistEnt(long Conv);
	bool IsExistSet(long Conv);
	bool GetStateSet(long Conv, long OnOff);
	bool IsExistExit(long Conv);
	bool IsExistLow(long Conv);
	bool IsExistOut(long Conv);
	bool GetStateOut(long Conv, long OnOff);
	void ShowPCBSensor();
private:
	static UINT StartPcbExist(LPVOID wParam);
	PCBSensorInfo m_PcbSensor[MAX_CONV];
	PCBSensorPos m_PcbExist[MAX_CONV];
	long m_ShowID;
};

extern CPcbExist* gcPcbExist;