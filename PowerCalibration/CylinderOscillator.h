#pragma once

#include <WMX3Api.h>
#include <IOApi.h>
#include <SimuApi.h>
#include <EventApi.h>
#include <UserMemoryApi.h>

#include <map>

//Using Singleton Pattern
class CylinderOscillator
{
public:
	static CylinderOscillator& GetInstance();
	static UINT staticFunctionForGeneralThreading(LPVOID parameter);
	bool setEmergencyMemoryOn(CString& cStringForErrorMessage);
	static bool getIOAddressAndIOBitFromgcPowerIO(const int index, int& ioAddress, int& ioBit);

	static const int						BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY	= 119;
	static const int						BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY	= 1;

private:
	wmx3Api::WMX3Api						wmx3Api;
	wmx3Api::Io								io;
	wmx3Api::simuApi::Simu					simu;
	wmx3Api::EventControl					eventControl;
	wmx3Api::UserMemory						userMemory;

	const wchar_t*							deviceNameForIOSignalSimulation = _T("IO Simulating..");
	const char*								deviceName;//"Cylinder Helper"

	CylinderOscillator();
	~CylinderOscillator();
	CylinderOscillator& operator=(const CylinderOscillator& ref);
	/// <summary>
	/// 시뮬레이션에서 실린더 반복기능 할 때 0초만에 반복 끝나면 안되니까 일정 시간 이후에 센서 들어오도록 하기위한 스레드
	/// </summary>
	static UINT delayedIOSetForSimuApi(LPVOID parameter);
	struct delayedIOSet {
		int timeInMilliseconds;
		int byteAddress;
		int bitAddress;
		int setToThis_OneOrZero;
		bool copyCompleted;//멤버 int 값 복사를 끝냈다는 뜻
	};
};
