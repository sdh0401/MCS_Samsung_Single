#pragma once

#include <map>
#include <afxstr.h>
#include "IoApi.h"
//#include "../../../../Header/IODefine.h"
#include "GlobalIODefine.h"

class CylinderOscillateInfo
{
public:

	/// <summary>
	/// 본 enum에 항목이 추가되는 경우 static bool CylinderOscillateInfo::CylinderToEverything 함수에도 항목을 추가해야함. (switch-case 분기 추가하라는 뜻)
	/// DO specify enum value for each of them
	/// ->HMI와의 프로토콜 약속이기 때문에 직접 지정하고 CDecoding3 (3 50 7)에서 받은 숫자를 그대로 캐스팅해서 함수 실행할 수 있도록 구성
	/// 구체적으로 어떻게 할당할지는 바로 아래 설명 참고.
	/// 모든 실린더의 전진에 대한 SENSOR 신호의 IODefine 숫자가 해당 enum 상수의 value 입니다 (아래 IN으로 시작하는 보라색 상수 참고)
	/// ★ ANC LOCK UNLOCK 의 경우 LOCK 에 대한 신호가 해당 value가 되도록 약속했다. (전진, 후진 헷갈리니까)
	/// </summary>
	enum class T
	{
		FRONT_STOPPER						= IN_FCONV_WORK1_STOP_UP,
		FRONT_PUSHER_PLATE					= IN_FCONV_WORK1_PUSH_UP,
		FRONT_ANC_UP_DOWN					= IN_FANC_BASE_UP,
		FRONT_ANC_LOCK_UNLOCK				= IN_FANC_CLAMP_LOCK,
		TURNUNIT_LEFT_POCKET_UP_DOWN		= IN_TURNUNIT_LEFT_POCKET_UP,
		TURNUNIT_RIGHT_POCKET_UP_DOWN		= IN_TURNUNIT_RIGHT_POCKET_UP,
		TURNUNIT_PUSHER_FORWARD_BACKWARD	= IN_TURNUNIT_PUSHER_FORWARD,
		REAR_STOPPER						= IN_RCONV_WORK1_STOP_UP,
		REAR_PUSHER_PLATE					= IN_RCONV_WORK1_PUSH_UP,
		REAR_ANC_UP_DOWN					= IN_RANC_BASE_UP,
		REAR_ANC_LOCK_UNLOCK				= IN_RANC_CLAMP_LOCK,
	};

	struct CylinderOscillateInfo::InputEventSet
	{
		wmx3Api::IoEventInput::IoEventInputType inputFunction;
		int byteAddress;
		int bitAddress;
		int delay;
		unsigned char invert;
	};

	struct CylinderOscillateInfo::OutputEventSet
	{
		wmx3Api::IoEventOutput::IoEventOutputType outputFunction;
		int byteAddress;
		int bitAddress;
		unsigned char invert;
	};

	struct CylinderOscillateInfo::EventSetsForEvent
	{
		struct CylinderOscillateInfo::InputEventSet inputEventSet;
		struct CylinderOscillateInfo::OutputEventSet outputEventSet;
	};

	static void CylinderToCString(const CylinderOscillateInfo::T& cylinder, CString& cString);

	static bool CylinderToEverything(const CylinderOscillateInfo::T& cylinder, CString& cString, 
									 int& eventIdForMovingForward, int& eventIdForMovingBackward,
									 CylinderOscillateInfo::EventSetsForEvent& eventSetsForForwardEvent,
									 CylinderOscillateInfo::EventSetsForEvent& eventSetsForBackwardEvent
	);

	static bool fromCylinderTo2EventSets(const CylinderOscillateInfo::T& cylinder, 
										 struct CylinderOscillateInfo::EventSetsForEvent& eventSetsForForwardEvent, 
										 struct CylinderOscillateInfo::EventSetsForEvent& eventSetsForBackwardEvent);

	CylinderOscillateInfo();
	~CylinderOscillateInfo();

private:
	long double getStandardDeviation(const unsigned long long* numArray, const int arraySize);


public:
	CylinderOscillateInfo::T cylinderToMove;
	int repetitions;
	unsigned long long* timeToMoveForward;
	unsigned long long* timeToMoveBackward;
	int delay;
	//isBeingUsed는 조심히 다루어야 합니다. 서로다른 스레드간의 데이터접근을 막기 위함이므로.
	bool isBeingUsed;

	bool createXML(CString& cStringForEveryError, CString& cStringForFullFilePath);
};
