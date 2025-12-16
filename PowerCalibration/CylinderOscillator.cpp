#include "pch.h"
#include "CylinderOscillator.h"
#include "CPowerIO.h"
//#include "../../../../Header/IODefine.h"
#include "GlobalIODefine.h"
#include <thread>
#include <UserMemoryApi.h>
#include "GlobalData.h"
#include "CylinderOscillateInfo.h"
#include "AxisInformation.h"
#include "CTokenizer.h"
#include "Trace.h"

CylinderOscillator::CylinderOscillator()
	: wmx3Api(wmx3Api::WMX3Api())
	, io(wmx3Api::Io(&this->wmx3Api))
	, simu(wmx3Api::simuApi::Simu(&this->wmx3Api))
	, eventControl(wmx3Api::EventControl(&this->wmx3Api))
	, userMemory(wmx3Api::UserMemory(&this->wmx3Api))
	//, isRealRun(false)
	//, ID_FOR_EVENT_STOPPER_CYLINDER_FORWARD(100)//static으로 바뀌었다.
	//, ID_FOR_EVENT_STOPPER_CYLINDER_BACKWARD(101)
	//, ID_FOR_EVENT_PUSHER_CYLINDER_UPWARD(102)
	//, ID_FOR_EVENT_PUSHER_CYLINDER_DOWNWARD(103)
	//, BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY(119)
	//, BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY(1)
	//, deviceNameForStopperSequence(_T("Stopper Testing.."))
	//, deviceNameForIOSignalSimulation(_T("IO Simulating.."))
	, deviceName("Cylinder Helper")
{

	//CylinderNameAnd4IOInfos cylinderNameAnd4IOInfos(cylinderNameInCStringForCylinder0, mapOfSignalsForCylinderForwardForCylinder0, mapOfSignalsForCylinderForwardForCylinder0);

	//std::pair<int, CylinderNameAnd4IOInfos> firstPair = std::pair<int, CylinderNameAnd4IOInfos>(indexNumberForCylinder0, cylinderNameAnd4IOInfos);
	
	//this->mapOfSomething.insert();
}

CylinderOscillator::~CylinderOscillator() {}

CylinderOscillator& CylinderOscillator::operator=(const CylinderOscillator& ref)
{
	return *this; 
}

CylinderOscillator& CylinderOscillator::GetInstance()
{
	static CylinderOscillator cylinderOscillator;
	return cylinderOscillator;
}

bool CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(const int index, int& ioAddress, int& ioBit)
{
	if (gcPowerIO == NULL)
	{
		ioAddress = -1;
		ioBit = -1;
		return false;
	}
	ioAddress = gcPowerIO->GetIOAddr(index);
	ioBit = gcPowerIO->GetIOBit(index);
	return true;
}

UINT CylinderOscillator::staticFunctionForGeneralThreading(LPVOID parameter)
/*
* 편솔밸브/단동실린더 관련 대응 방안
* 문제 : 만약에 invert설정 인경우 (후진 신호가 곧 전진 OFF 신호인 경우) 전진/후진 정보를 얻을 때 그냥 아무생각없이 GetOutBitEx 하면 실패한다.
* 해결방안 : 후진 OUT신호가 0 인게 정상이기 때문에, invert 가 1 인 경우 후진신호 정보에 대한 지역변수의 값을 0이면 1, 1이면 0 으로 바꿔주는 코드를 추가해야한다. (GetOutBitEx 직후에)
* + 후진이 invert = 0 이고, 전진이 invert = 1 인 경우도 있기 때문에 GetOutBitEx 직후에 if문이 두개 있어야해.
*
* 231213 추가된 기능 : 장비 비상정지 스위치 작동시 정지(앞뒤로 달려있는거) + 타임아웃(5초) + 공압체크(처음에)
* 231214 추가된 기능 : 실린더가 만약에 올라가있으면 다 끝나고 올려놓고 나오기 (반대인경우도 마찬가지)
* 231215 : 231214에 추가한 기능에 대해서 오해가 있었고 무조건 후진으로 끝나는게 맞다.관련 내용 수정중.
*/
{
	//#0. param 불러오기
	CylinderOscillateInfo* param = reinterpret_cast<CylinderOscillateInfo*>(parameter);
	CylinderOscillateInfo& cylinderOscillateInfo = *param;
	if (
		//정상적으로 초기화되지 않거나.. 이미 isBeingUsed가 true인경우 바로 종료.
		cylinderOscillateInfo.isBeingUsed == true ||
		cylinderOscillateInfo.repetitions == -1 ||
		cylinderOscillateInfo.delay == -1
		)
	{
		return 1;
	}

	/*
	* 20250530_중복 동작 방지
	* NEW_DEVELOP, Machine manual Action Running Sign, Lock
	*/
	if (GetMachineManualActionRunninSign() == true) {
		TRACE("[PWR] CDecoding3 MachineManualAction Err, Other is Running!\n");
		return 1;
	}
	else {
		SetMachineManualActionRunninSign(true);
	}

	//#1. 정상적으로 불러왔으므로 사용중이라는 정보 저장, 실행 관련 정보 안헷갈리게 const 지역변수로 저장.
	cylinderOscillateInfo.isBeingUsed = true;
	const CylinderOscillateInfo::T cylinder = cylinderOscillateInfo.cylinderToMove;
	const int repetitions = cylinderOscillateInfo.repetitions;
	const int delay = cylinderOscillateInfo.delay;
	const int timeOut_5000MilliSeconds = 5000;//타임아웃 5초 안에 전진/후진 완료 못하면 비정상 종료할거야.

	//#2. 필요한 배열 초기화
	cylinderOscillateInfo.timeToMoveForward = new unsigned long long[repetitions];
	cylinderOscillateInfo.timeToMoveBackward = new unsigned long long[repetitions];
	for (int i = 0; i < repetitions; i++)//쓰레기값 대신에 0으로 초기화
	{
		cylinderOscillateInfo.timeToMoveForward[i] = 0;
		cylinderOscillateInfo.timeToMoveBackward[i] = 0;
	}

	//#3. 기타 지역변수 선언
	bool emergencyStopSignal = false;//중간에 문제 발생 or 사용자의 정지 명령 시 해당 정보를 저장하기 위한 지역변수 (emergency를 변수이름에서 빼는게 적절. 진짜 emergency switch랑 헷갈려)
	CylinderOscillateInfo::EventSetsForEvent eventSetsButJustForIOAddressForForward, eventSetsButJustForIOAddressForBackward;//io주소저장용 (EventSetsForEvent 라는 타입 이름 부적절(의미가 맞지않음))
	CylinderOscillateInfo::InputEventSet justForIOAddressFor_FrontMachineEmergencySwitchInput, justForIOAddressFor_RearMachineEmergencySwitchInput;//FRONT또는REAR Machine Emergency Switch Input의 io주소 저장용 (진짜 장비에 달려있는 비상 스위치. 마찬가지로 타입 이름 부적절)
	CylinderOscillateInfo::InputEventSet justForIOAddressFor_IN_AIR_PRESS_LOW;//맨첨에 공압체크 하기위한 IO주소 저장.
	CString cStringForEveryError;//디버그용 에러내용 저장..

	//#4. Emergency Switch (Front Rear) 주소 불러오기
	if (
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_FEMERGENCY,
														  justForIOAddressFor_FrontMachineEmergencySwitchInput.byteAddress,
														  justForIOAddressFor_FrontMachineEmergencySwitchInput.bitAddress) == false
		||
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_REMERGENCY,
														  justForIOAddressFor_RearMachineEmergencySwitchInput.byteAddress,
														  justForIOAddressFor_RearMachineEmergencySwitchInput.bitAddress) == false
		)
	{
		cStringForEveryError.Append(L"failed : CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_(F/R)EMERGENCY)\n");
		emergencyStopSignal = true;
	}

	//#5. 실린더 input output 주소 불러오기
	if (CylinderOscillateInfo::fromCylinderTo2EventSets(cylinder, eventSetsButJustForIOAddressForForward, eventSetsButJustForIOAddressForBackward) == false)
	{
		cStringForEveryError.Append(L"failed : CylinderOscillateInfo::fromCylinderTo2EventSets\n");
		emergencyStopSignal = true;
	}

	//#6. 공압 체크 의 주소 불러오기
	if (CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_AIR_PRESS_LOW,
														  justForIOAddressFor_IN_AIR_PRESS_LOW.byteAddress,
														  justForIOAddressFor_IN_AIR_PRESS_LOW.bitAddress) == false
		)
	{
		cStringForEveryError.Append(L"failed : CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_AIR_PRESS_LOW)\n");
		emergencyStopSignal = true;
	}
	//불러오기는 끝..? 응.

	//#7. 시뮬레이션인지 확인하기
	const bool isSimulation = GetGlobalSimulationMode();
	const int maxCycleCount = repetitions;//이거는 좀 쓸데없긴한데 그냥 추가했어요..

	//#8. 멀티스레드니까 전용 디바이스 wmx3Api 변수 선언
	wmx3Api::WMX3Api		deviceForThread;
	wmx3Api::Io				ioApiForThread(&deviceForThread);
	wmx3Api::EventControl	eventControlApiForThread(&deviceForThread);
	wmx3Api::simuApi::Simu	simuApiForThread(&deviceForThread);
	wmx3Api::UserMemory		userMemoryApiForThread(&deviceForThread);

	//#9. CreateDevice, SetDeviceName
	if (emergencyStopSignal == true ||
		deviceForThread.CreateDevice(_T("C:\\Program Files\\SoftServo\\WMX3")) != wmx3Api::ErrorCode::None ||
		deviceForThread.SetDeviceName("CylinderTesting:") != wmx3Api::ErrorCode::None
		)
	{
		cStringForEveryError.Append(L"createDeviceFailed\n");
		emergencyStopSignal = true;
	}
	if (isSimulation == true)
	{
		deviceForThread.SetDeviceName("CylinderTesting:Simulation");
	}
	else
	{
		deviceForThread.SetDeviceName("CylinderTesting:RealRun");
	}

	//#10. 시작전에 공압 들어오는지 확인 하기 -> 시뮬이면 봐준다.
	/*
	* : 원래 신호가 1일 때 비정상, 0일 때 정상이라고 판단하고 그렇게 짜서 테스트 해보니까 실제로는 반대로 되어있어서(?) 해당 내용 레드마인에 댓글로 남겨놓았고
	* 아래 소스코드에서도 값이 0일 때 비정상(압력낮음)이라고 판단하는 동작으로 구성해따.
	*/
	unsigned char is_0_IfAirPressureIsLow = -1;
	if (ioApiForThread.GetInBitEx(justForIOAddressFor_IN_AIR_PRESS_LOW.byteAddress, justForIOAddressFor_IN_AIR_PRESS_LOW.bitAddress, &is_0_IfAirPressureIsLow) != wmx3Api::ErrorCode::None)
	{
		cStringForEveryError.Append(L"ioApiForThread.GetInBitEx\n");
		emergencyStopSignal = true;
	}
	bool _isAirPressureGood = true;
	if (is_0_IfAirPressureIsLow == 0)//0일 때 비정상....
	{
		if (isSimulation == false)
		{
			cStringForEveryError.Append(L"air pressure is low!\n");
			_isAirPressureGood = false;
			emergencyStopSignal = true;
		}
		else
		{
			cStringForEveryError.Append(L"air pressure is low! but its simulation..\n");
			_isAirPressureGood = true;
		}
	}
	else
	{
		_isAirPressureGood = true;
	}
	const bool isAirPressureGood = _isAirPressureGood;

	//#11. input 신호 저장용 변수 선언
	unsigned char forwardMoveIsCompletedIfIts1 = -1;
	unsigned char backwardMoveIsCompletedIfIts1 = -1;

	/*
	//#12. 전진인지 후진인지 기억했다가 마지막에 원상복구 해놓고 나오기 위해 최초의 센서 값 읽는 작업 -> 이거 필요없어 .. ㅇㅇ
	if (emergencyStopSignal == true ||
		ioApiForThread.GetInBitEx(eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress,
								  eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress,
								  &forwardMoveIsCompletedIfIts1) != wmx3Api::ErrorCode::None ||
		ioApiForThread.GetInBitEx(eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress,
								  eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress,
								  &backwardMoveIsCompletedIfIts1) != wmx3Api::ErrorCode::None)
	{
		cStringForEveryError.Append(L"ioApiForThread.GetInBitEx\n");
		emergencyStopSignal = true;
	}
	*/

	//#13. 마지막에 어떻게 끝나야하는지 구분하기 위한 상수 저장. ANC LOCK UNLOCK 만 전진으로 끝나고 나머지는 후진으로 끝나면 됩니다. 아마?
	bool _thisCylinderShouldEndWithForwardMove = false;
	if (cylinder == CylinderOscillateInfo::T::FRONT_ANC_LOCK_UNLOCK
		||
		cylinder == CylinderOscillateInfo::T::REAR_ANC_LOCK_UNLOCK)
	{
		_thisCylinderShouldEndWithForwardMove = true;
	}
	const bool thisCylinderShouldEndWithForwardMove = _thisCylinderShouldEndWithForwardMove;

	//#14. 일단 무조건 전진 끄고 후진 켜기
	if (emergencyStopSignal == true ||
		ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
								   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
								   (eventSetsButJustForIOAddressForForward.outputEventSet.invert == 0 ? 0 : 1)) != wmx3Api::ErrorCode::None || //끄는거니까 invert가 0이면 0, 1 이면 1.
		ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
								   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
								   (eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 0 ? 1 : 0)) != wmx3Api::ErrorCode::None)//켜는거니까 invert가 0이면 1, 1 이면 0.
	{
		cStringForEveryError.Append(L"ioApiForThread.SetOutBitEx\n");
		emergencyStopSignal = true;
	}

	//#15. [시뮬레이션 모드 전용] 일정시간이후에 input 신호 만들기 위해 ..
	delayedIOSet structInstanceForDelayedForwardCompletion;
	structInstanceForDelayedForwardCompletion.byteAddress = eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress;
	structInstanceForDelayedForwardCompletion.bitAddress = eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress;
	structInstanceForDelayedForwardCompletion.setToThis_OneOrZero = eventSetsButJustForIOAddressForBackward.inputEventSet.invert == 0 ? 1 : 0;//delayed completion 이면 켜져야되니까 0 이면 1, 1 이면 0.
	structInstanceForDelayedForwardCompletion.timeInMilliseconds = 500;//내맘대로 밀리초 (전진시간) (시뮬에서 쓰는 값)
	structInstanceForDelayedForwardCompletion.copyCompleted = false;
	delayedIOSet structInstanceForDelayedBackwardCompletion;
	structInstanceForDelayedBackwardCompletion.byteAddress = eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress;
	structInstanceForDelayedBackwardCompletion.bitAddress = eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress;
	structInstanceForDelayedBackwardCompletion.setToThis_OneOrZero = eventSetsButJustForIOAddressForForward.inputEventSet.invert == 0 ? 1 : 0;//delayed completion 이면 켜져야되니까 0 이면 1, 1 이면 0.
	structInstanceForDelayedBackwardCompletion.timeInMilliseconds = 600;//내맘대로 밀리초 (후진시간) (시뮬에서 쓰는 값)
	structInstanceForDelayedBackwardCompletion.copyCompleted = false;

	//#16. output 직전 신호 정보 저장용 변수 선언
	unsigned char forwardOutputWas_1_or_0_Before = -1;
	unsigned char backwardOutputWas_1_or_0_Before = -1;

	//#17. [시뮬레이션모드] 가상의 후진시작 input + 가상의 후진완료 input 만들기
	if (isSimulation == true && emergencyStopSignal == false)
	{
		//전진완료에 대한 input 신호 끄기
		if (simuApiForThread.SetInBit(eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress,
									  eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress,
									  eventSetsButJustForIOAddressForBackward.inputEventSet.invert == 0 ? 0 : 1) != wmx3Api::ErrorCode::None)//invert가 0이면 꺼야되니까 그대로 0
		{
			cStringForEveryError.Append(L"simuApiForThread.SetInBit\n");
			emergencyStopSignal = true;
		}

		if (ioApiForThread.GetInBitEx(eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress,
									  eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress,
									  &backwardMoveIsCompletedIfIts1) != wmx3Api::ErrorCode::None)//후진 완료 in 신호
		{
			cStringForEveryError.Append(L"ioApiForThread.GetInBitEx\n");
			emergencyStopSignal = true;
		}
		else
		{
			if (backwardMoveIsCompletedIfIts1 != 1)//후진완료상태가 아니라면..
			{
				//@. additional threading for simuapi : 일정시간 뒤 후진 완료 : 후진 완료가 켜져있지 않을 때만
				CWinThread* cWinThreadPointer;
				cWinThreadPointer = AfxBeginThread(CylinderOscillator::delayedIOSetForSimuApi, &structInstanceForDelayedBackwardCompletion);

				if (cWinThreadPointer == NULL)
				{
					cStringForEveryError.Append(L"additional thread is null\n");
					emergencyStopSignal = true;
				}
				else
				{
					cWinThreadPointer->m_bAutoDelete = true;
				}
			}
			backwardOutputWas_1_or_0_Before = 1;//이미 후진 시작했으므로 1로 재할당.
		}
	}

	//#18. 현재 움직임 정보 저장 : 전진중인지 후진중인지
	unsigned char is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff = -1;//현재의 움직임 정보 저장.
	unsigned char is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff = -1;//현재의 움직임 정보 저장.

	//#19. 도중 정지 명령들어왔는지 저장하는 변수 선언, 장비의 비상 스위치 정보 저장 위한 변수 선언
	unsigned char is_1_IfEmergencyM_BitIsOn = -1;//wmx3Api GetMBitEx를 쓸 때 넘겨주기위한 변수
	unsigned char is_1_IfFrontMachineEmergencySwitchInputIsOn = -1, is_1_IfRearMachineEmergencySwitchInputIsOn = -1;//wmx3Api GetMBitEx를 쓸 때 넘겨주기위한 변수

	//#20. 후진 켜졌는지 확인하고 넘어가기
	while (true)
	/*
	* 이 함수의 모든 while(true)루프(총 2개.. 아마) 에는 항상 매 루프마다
	* 사용자 정지명령(MBit)신호가 들어오는지, FRONT REAR 의 MACHINE EMERGENCY SWITCH INPUT 신호가 들어오는지
	* 체크해서 감지될 경우 cStringForEveryError에 해당 내용 추가하고
	* emergencyStopSignal을 true로 바꾼다. + break.
	*/
	{
		if (emergencyStopSignal == true)
		{
			break;
		}

		//GetInBitEx (앞 뒤 EMERGENCY SWITCH 신호 감지)
		if (
			ioApiForThread.GetInBitEx(justForIOAddressFor_FrontMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_FrontMachineEmergencySwitchInput.bitAddress, &is_1_IfFrontMachineEmergencySwitchInputIsOn) != wmx3Api::ErrorCode::None
			||
			ioApiForThread.GetInBitEx(justForIOAddressFor_RearMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_RearMachineEmergencySwitchInput.bitAddress, &is_1_IfRearMachineEmergencySwitchInputIsOn) != wmx3Api::ErrorCode::None
			)
		{
			cStringForEveryError.AppendFormat(L"ioApiForThread.GetInBitEx(..) != wmx3Api::ErrorCode::None\n");
			emergencyStopSignal = true;
			break;
		}
		//방금 GetInBitEx한 결과 확인해서 신호 들어왔으면 처리
		if (is_1_IfFrontMachineEmergencySwitchInputIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"FrontMachineEmergencySwitchInputIsOn\n");
			emergencyStopSignal = true;
			break;
		}
		if (is_1_IfRearMachineEmergencySwitchInputIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"RearMachineEmergencySwitchInputIsOn\n");
			emergencyStopSignal = true;
			break;
		}

		//GetMBitEx (사용자 정지 명령 신호 감지)
		if (userMemoryApiForThread.GetMBitEx(CylinderOscillator::BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, CylinderOscillator::BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, &is_1_IfEmergencyM_BitIsOn) != wmx3Api::ErrorCode::None)
		{
			cStringForEveryError.AppendFormat(L"userMemoryApiForThread.GetMBitEx != wmx3Api::ErrorCode::None\n");
			emergencyStopSignal = true;
			break;
		}
		if (is_1_IfEmergencyM_BitIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"userMemory is ON (1)\n");
			emergencyStopSignal = true;
			break;
		}


		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		if (ioApiForThread.GetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
									   &is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff) != wmx3Api::ErrorCode::None ||
			ioApiForThread.GetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
									   &is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff) != wmx3Api::ErrorCode::None)
		{
			cStringForEveryError.Append(L"ioApiForThread.GetInBitEx\n");
			emergencyStopSignal = true;
			break;
		}
		if (eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 1)/*forward-output-invert 가 켜져있는 경우(편솔 또는 단동실린더)*/
		{
			/*1이면 0, 0이면 1로 바꾸는 코드*/
			is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff = 1 - is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff;
		}
		else if (eventSetsButJustForIOAddressForForward.outputEventSet.invert == 1)/*backward-output-invert 가 켜져있는 경우(편솔 또는 단동실린더)*/
		{
			/*1이면 0, 0이면 1로 바꾸는 코드*/
			is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff = 1 - is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff;
		}

		if (is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff == 0 &&//전진신호가 꺼져있고,
			is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff == 1)//후진신호가 켜져있으면
		{
			break;//다음단계로 넘어가
		}
		else
		{
			continue;
		}
	}

	//#21. 본게임 전 필요한 변수 선언
	int currentCycleCount = -1;//현재 바퀴수 저장
	unsigned long long timeBefore = _time_get();//이전시점의 시간 저장.
	unsigned long long timeElapsedMovingForward;//전진하는데 걸린 시간 저장.
	unsigned long long timeElapsedMovingBackward;//후진하는데 걸린 시간 저장.
	bool isTimeOutDetected = false;//타임아웃 발생 여부 저장. 타임아웃이 발생했을 경우에만 true로 재정의.

	//#22. 본게임 시작
	/*
	* 필요할지도 모르는 추가기능 : thread.sleep 시간(delay)이 굉장히 길 경우 중간 중지 명령을 보내도 sleep 도중일 경우 예정된대로 sleep 이후에 전진/후진을 할 가능성이 있다.
	* 해결방안 : sleep 이후에 직접 GetMBitEx를 해서 켜져있으면 아무것도 안 바꾸고 바로 나올 수 있도록 하는게 필요할지도? 또는 sleep의 최댓값을 MCS 맘대로 1초 지정해버리던가
	*/
	while (true)
	{
		if (emergencyStopSignal == true)
		{
			break;
		}

		if (userMemoryApiForThread.GetMBitEx(CylinderOscillator::BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, CylinderOscillator::BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, &is_1_IfEmergencyM_BitIsOn) != wmx3Api::ErrorCode::None)
		{
			cStringForEveryError.AppendFormat(L"userMemoryApiForThread.GetMBitEx\n");
			emergencyStopSignal = true;
			break;
		}
		if (is_1_IfEmergencyM_BitIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"userMemory is ON (1)\n");
			emergencyStopSignal = true;
			break;
		}

		//GetInBitEx (앞 뒤 EMERGENCY SWITCH 신호 감지)
		if (
			ioApiForThread.GetInBitEx(justForIOAddressFor_FrontMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_FrontMachineEmergencySwitchInput.bitAddress, &is_1_IfFrontMachineEmergencySwitchInputIsOn) != wmx3Api::ErrorCode::None
			||
			ioApiForThread.GetInBitEx(justForIOAddressFor_RearMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_RearMachineEmergencySwitchInput.bitAddress, &is_1_IfRearMachineEmergencySwitchInputIsOn) != wmx3Api::ErrorCode::None
			)
		{
			cStringForEveryError.AppendFormat(L"ioApiForThread.GetInBitEx(..) != wmx3Api::ErrorCode::None\n");
			emergencyStopSignal = true;
			break;
		}
		//방금 GetInBitEx한 결과 확인해서 신호 들어왔으면 처리
		if (is_1_IfFrontMachineEmergencySwitchInputIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"FrontMachineEmergencySwitchInputIsOn\n");
			emergencyStopSignal = true;
			break;
		}
		if (is_1_IfRearMachineEmergencySwitchInputIsOn == 1)
		{
			cStringForEveryError.AppendFormat(L"RearMachineEmergencySwitchInputIsOn\n");
			emergencyStopSignal = true;
			break;
		}

		//전/후진에 대한 SOLENOID, SENSOR 총 4가지(OUT IN) 신호 확인
		if (ioApiForThread.GetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
									   &is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff) != wmx3Api::ErrorCode::None ||//전진 out 신호
			ioApiForThread.GetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
									   &is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff) != wmx3Api::ErrorCode::None ||//후진 out 신호
			ioApiForThread.GetInBitEx(eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress,
									  eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress,
									  &backwardMoveIsCompletedIfIts1) != wmx3Api::ErrorCode::None || //후진 완료 in 신호
			ioApiForThread.GetInBitEx(eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress,
									  eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress,
									  &forwardMoveIsCompletedIfIts1) != wmx3Api::ErrorCode::None)//전진 완료 in 신호
		{
			cStringForEveryError.Append(L"ioApiForThread.Get(In/Out)BitEx\n");
			emergencyStopSignal = true;
			break;
		}
		if (eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 1)/*forward-output-invert 가 켜져있는 경우(편솔 또는 단동실린더)*/
		{
			/*1이면 0, 0이면 1로 바꾸는 코드*/
			is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff = 1 - is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff;
		}
		else if (eventSetsButJustForIOAddressForForward.outputEventSet.invert == 1)/*backward-output-invert 가 켜져있는 경우(편솔 또는 단동실린더)*/
		{
			/*1이면 0, 0이면 1로 바꾸는 코드*/
			is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff = 1 - is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff;
		}

		if (is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff == 1 &&//전진 ON
			is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff == 0)//후진 OFF 이면.. < 전진 >
		{
			if (forwardOutputWas_1_or_0_Before != is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff &&
				backwardOutputWas_1_or_0_Before != is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff)//방금 막 전진을 시작한 상황
			{
				//전진/후진 정보 갱신
				forwardOutputWas_1_or_0_Before = is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff;
				backwardOutputWas_1_or_0_Before = is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff;

				if (isSimulation == true)//시뮬레이션일때
				{
					//가상의 후진완료센서 떨어짐 생성
					if (simuApiForThread.SetInBit(eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress,
												  eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress,
												  (eventSetsButJustForIOAddressForForward.inputEventSet.invert == 0 ? 0 : 1)//꺼야되니까 0 일때 0.
					) != wmx3Api::ErrorCode::None)
					{
						cStringForEveryError.Append(L"simuApiForThread.SetInBit\n");
						emergencyStopSignal = true;
					}

					//@. additional threading for simuapi : 일정시간 뒤 전진 완료
					CWinThread* cWinThreadPointer = AfxBeginThread(CylinderOscillator::delayedIOSetForSimuApi, &structInstanceForDelayedForwardCompletion);

					if (cWinThreadPointer == NULL)
					{
						cStringForEveryError.Append(L"additional thread is null\n");
						emergencyStopSignal = true;
						break;
					}
					else
					{
						cWinThreadPointer->m_bAutoDelete = true;
					}
				}//if (isSimulation == true)
			}
			else//이미 전진 하고있던 상황
			{
				//전진 완료 신호가 들어왔으면, 딜레이 이후에 전진 out 신호 끄고, 후진 out 신호 켜기
				if (forwardMoveIsCompletedIfIts1 == 1)
				{
					timeElapsedMovingForward = _time_elapsed(timeBefore);
					cylinderOscillateInfo.timeToMoveForward[currentCycleCount] = timeElapsedMovingForward;
					//cStringForTimeDebug.AppendFormat(L"%llu ", timeElapsedMovingForward);//

					//신호 바꾸기 전에 설정된 딜레이만큼 대기
					std::this_thread::sleep_for(std::chrono::milliseconds(delay));//설정한 대기시간만큼 sleep

					if (ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
												   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
												   (eventSetsButJustForIOAddressForForward.outputEventSet.invert == 0 ? 0 : 1)) != wmx3Api::ErrorCode::None || //꺼야되니까 0이면 0.
						ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
												   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
												   (eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 0 ? 1 : 0)) != wmx3Api::ErrorCode::None)//켜야되니까 0이면 1.
					{
						cStringForEveryError.Append(L"ioApiForThread.SetOutBitEx\n");
						emergencyStopSignal = true;
						break;
					}

					timeBefore = _time_get();
				}
				else
				{
					//5초동안 못했으면 나가야돼
					if (_time_elapsed(timeBefore) > timeOut_5000MilliSeconds)
					{
						cStringForEveryError.Append(L"timeOut!\n");
						emergencyStopSignal = true;
						isTimeOutDetected = true;
						break;
					}
				}
			}
		}// < 전진 > 끝.
		else if (is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff == 0 &&//전진 OFF
				 is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff == 1)//후진 ON 이면.. < 후진 >
		{
			if (forwardOutputWas_1_or_0_Before != is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff &&
				backwardOutputWas_1_or_0_Before != is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff)//방금 막 후진을 시작한 상황
			{
				//전진/후진 정보 갱신
				forwardOutputWas_1_or_0_Before = is_1_IfItsForwardOutputIsOn_0_IfItsForwardOutputIsOff;
				backwardOutputWas_1_or_0_Before = is_1_IfItsBackwardOutputIsOn_0_IfItsBackwardOutputIsOff;

				//시뮬이면 일정시간 delayed 후진완료 설정
				if (isSimulation == true)//시뮬레이션이면
				{
					//가상의 전진완료센서 떨어짐 생성
					if (simuApiForThread.SetInBit(eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress,
												  eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress,
												  (eventSetsButJustForIOAddressForBackward.inputEventSet.invert == 0 ? 0 : 1)//꺼야되니까 0 일때 0.
					) != wmx3Api::ErrorCode::None)
					{
						cStringForEveryError.Append(L"simuApiForThread.SetInBit\n");
						emergencyStopSignal = true;
					}
					//@. 일정시간뒤 후진 완료 생성
					CWinThread* cWinThreadPointer = AfxBeginThread(CylinderOscillator::delayedIOSetForSimuApi, &structInstanceForDelayedBackwardCompletion);

					if (cWinThreadPointer == NULL)
					{
						cStringForEveryError.Append(L"additional thread is null\n");
						emergencyStopSignal = true;
						break;
					}
					else
					{
						cWinThreadPointer->m_bAutoDelete = true;
					}
				}
			}
			else//이미 후진 하고있던 상황
			{
				//후진 완료 신호가 들어왔으면, 바로 후진 out 신호 끄고, 전진 out 신호 켜기
				if (backwardMoveIsCompletedIfIts1 == 1)
				{
					if (currentCycleCount != -1)
					{
						timeElapsedMovingBackward = _time_elapsed(timeBefore);
						cylinderOscillateInfo.timeToMoveBackward[currentCycleCount] = timeElapsedMovingBackward;
						//cStringForTimeDebug.AppendFormat(L"%llu ", timeElapsedMovingBackward);//
					}

					currentCycleCount++;
					if (maxCycleCount == currentCycleCount)
					{
						break;
					}

					//신호 바꾸기 전에 설정된 딜레이만큼 대기
					std::this_thread::sleep_for(std::chrono::milliseconds(delay));//설정한 대기시간만큼 sleep

					if (ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
												   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
												   eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 0 ? 0 : 1) != wmx3Api::ErrorCode::None ||//꺼야되니 0이면 0.
						ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
												   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
												   eventSetsButJustForIOAddressForForward.outputEventSet.invert == 0 ? 1 : 0) != wmx3Api::ErrorCode::None)//켜야되니 0이면 1.
					{
						cStringForEveryError.Append(L"ioApiForThread.SetOutBitEx\n");
						emergencyStopSignal = true;
						break;
					}

					timeBefore = _time_get();
				}
				else
				{
					//5초동안 못했으면 나가야돼
					if (_time_elapsed(timeBefore) > timeOut_5000MilliSeconds)
					{
						cStringForEveryError.Append(L"timeOut!\n");
						emergencyStopSignal = true;
						isTimeOutDetected = true;
						break;
					}
				}
			}
		}// < 후진 > 끝.
		else//둘다 켜져있거나 둘다 꺼져있는 경우.. (불가능?)
		{
			//deviceForThread.SetDeviceName("neither forward nor backward");
			//nothing..yet?
		}
		std::this_thread::sleep_for(std::chrono::microseconds(300));//300마이크로세컨드 쉬기
	}// while(true) 본게임 끝

	//#23. 반복 완료 후 원래 있어야 하는 상태로 전진 또는 후진 하는 동작 (실린더 별로 다르기 때문에 저- 위에서 상수에 저장함.)
	if (thisCylinderShouldEndWithForwardMove == true)
	{
		//전진으로 끝나야되는 애는 전진
		if (
			ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.invert == 0 ? 1 : 0) != wmx3Api::ErrorCode::None//전진 켜기
			||
			ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 0 ? 0 : 1) != wmx3Api::ErrorCode::None//후진 끄기
			)
		{
			cStringForEveryError.Append(L"ioApiForThread.SetOutBitEx != ErrorCode::None : backToWhereTheCylinderWasBefore\n");
		}
		if (isSimulation == true)
		{
			//가상의 후진완료센서 떨어짐 생성
			if (simuApiForThread.SetInBit(eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress,
										  eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress,
										  (eventSetsButJustForIOAddressForForward.inputEventSet.invert == 0 ? 0 : 1)//꺼야되니까 0 일때 0.
			) != wmx3Api::ErrorCode::None)
			{
				cStringForEveryError.Append(L"simuApiForThread.SetInBit\n");
				emergencyStopSignal = true;
			}

			//@. additional threading for simuapi : 일정시간 뒤 전진 완료
			CWinThread* cWinThreadPointer = AfxBeginThread(CylinderOscillator::delayedIOSetForSimuApi, &structInstanceForDelayedForwardCompletion);

			if (cWinThreadPointer == NULL)
			{
				cStringForEveryError.Append(L"additional thread is null\n");
				emergencyStopSignal = true;
			}
			else
			{
				cWinThreadPointer->m_bAutoDelete = true;
			}
		}
	}
	else
	{
		//후진으로 끝나야되는 애는 후진
		if (
			ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress,
									   eventSetsButJustForIOAddressForForward.outputEventSet.invert == 0 ? 0 : 1) != wmx3Api::ErrorCode::None//전진 끄기
			||
			ioApiForThread.SetOutBitEx(eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress,
									   eventSetsButJustForIOAddressForBackward.outputEventSet.invert == 0 ? 1 : 0) != wmx3Api::ErrorCode::None//후진 켜기
			)
		{
			cStringForEveryError.Append(L"ioApiForThread.SetOutBitEx != ErrorCode::None : backToWhereTheCylinderWasBefore\n");
		}
		if (isSimulation == true)//시뮬이면 일정시간 delayed 후진완료 설정(본게임안에서 쓴거랑 동일하게 가져다 썼다.)
		{
			//가상의 전진완료센서 떨어짐 생성
			if (simuApiForThread.SetInBit(eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress,
										  eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress,
										  (eventSetsButJustForIOAddressForBackward.inputEventSet.invert == 0 ? 0 : 1)//꺼야되니까 0 일때 0.
			) != wmx3Api::ErrorCode::None)
			{
				cStringForEveryError.Append(L"simuApiForThread.SetInBit\n");
				emergencyStopSignal = true;
			}
			//@. 일정시간뒤 후진 완료 생성
			CWinThread* cWinThreadPointer = AfxBeginThread(CylinderOscillator::delayedIOSetForSimuApi, &structInstanceForDelayedBackwardCompletion);

			if (cWinThreadPointer == NULL)
			{
				cStringForEveryError.Append(L"additional thread is null\n");
				emergencyStopSignal = true;
			}
			else
			{
				cWinThreadPointer->m_bAutoDelete = true;
			}
		}
	}

	//#24. 본게임 끝나고 : createXML xml 파일 저장!
	CString cStringForFullFilePath;
	bool _createXMLResult;
	if (isTimeOutDetected == false)//타임아웃 발생 안했을 경우 정상적으로 xml 파일 생성 시도합니다.
	{
		_createXMLResult = cylinderOscillateInfo.createXML(cStringForEveryError, cStringForFullFilePath);
		if (_createXMLResult == false)
		{
			cStringForEveryError.AppendFormat(L"cylinderOscillateInfo.createXML -> false");
			emergencyStopSignal = true;
		}
	}
	else//타임아웃 발생시 xml 파일 생성하지 않고 넘어갑니다.
	{
		cStringForFullFilePath.Empty();//이거 없어도 되긴 하지만 그냥..
		_createXMLResult = false;
	}
	const bool createXMLResult = _createXMLResult;//XML 성공 했는지 여부 저장용 변수 선언.

	//#25. [시뮬레이션 모드 전용] 시뮬일 때 만든 스레드 종료 기다리기 wait (무한루프 가능성 있으)
	if (isSimulation == true)
	{
		CString cstemp;
		wmx3Api::DevicesInfoA devicesInfoA;
		bool stillWait = false;
		while (true)
		{
			//추가로 생성한 디바이스 이름을 확인해서 여기에서 만든 스레드들이 닫혔는지 체크
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			if (deviceForThread.GetAllDevices(&devicesInfoA) != wmx3Api::ErrorCode::None)
			{
				cStringForEveryError.AppendFormat(L"%s", L"deviceForThread.GetAllDevices");
				emergencyStopSignal = true;
				break;
			}
			for (unsigned int j = 0; j < devicesInfoA.count; j++)
			{
				for (int i = 0; i < wmx3Api::constants::maxDeviceName; i++)
				{
					cstemp.AppendFormat(L"%c", devicesInfoA.devices[j].name[i]);//이름 완성하기
				}
				if (cstemp.Compare(CylinderOscillator::GetInstance().deviceNameForIOSignalSimulation) == 0)
				{
					stillWait = true;
				}
				cstemp.Empty();
			}
			if (stillWait == true)
			{
				stillWait = false;
				continue;
			}
			else
			{
				//closeDevice -> 함수 종료 까지 0.01초만 마지막에 더 기다리기
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				break;
			}
		}//while(true)
	}

	//#26. SendToHMI 하기
	CString cStringForSendingToHMI;
	if (emergencyStopSignal == true)
	{
		if (isAirPressureGood == false)//공압문제 발생 : 특별히 (1,0,0)+(ㅁ,ㅁ) 먼저 보내준다 (HMI-MCS Protocol 엑셀파일 참고)
		{
			CString cs;
			cs.Append(L"48,LOW_AIR");
			SendToHMI(1, 0, 0, cs);
			cs.Empty();
			cStringForSendingToHMI.Format(L"20000");
		}
		else if (isTimeOutDetected == true)//타임아웃 발생
		{
			cStringForSendingToHMI.Format(L"1");
		}
		else if (createXMLResult == false)//타임아웃도 아닌데 알수없는 문제로 xml 파일 생성 실패
		{
			cStringForSendingToHMI.Format(L"20000");
		}
		else//그냥 중간에 멈춘경우(정지 명령 또는 비상정지 스위치 동작) 또는 초기(IO주소 불러오기 등) 문제.
		{
			cStringForSendingToHMI.Format(L"0");
		}
	}
	else//정상적으로 정해진 반복수 다 채우고 종료
	{
		cStringForSendingToHMI.Format(L"0");
	}
	cStringForSendingToHMI.Append(L",");
	cStringForSendingToHMI.Append(cStringForFullFilePath);
	SendToHMI(3, 50, 9, cStringForSendingToHMI);// 3 50 9 ok? 응
	cStringForSendingToHMI.Empty();

	//#27. 디버그용 CString 내용 저장
	CString cStringForDebug;
	cStringForDebug.AppendFormat(L"* emergencyStopSignal : %s *\n", emergencyStopSignal == true ? L"true" : L"false");
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* IN_AIR_PRESS_LOW address : (%d.%d) *", justForIOAddressFor_IN_AIR_PRESS_LOW.byteAddress, justForIOAddressFor_IN_AIR_PRESS_LOW.bitAddress);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* IN_FEMERGENCY address : (%d.%d) *", justForIOAddressFor_FrontMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_FrontMachineEmergencySwitchInput.bitAddress);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* IN_REMERGENCY address : (%d.%d) *", justForIOAddressFor_RearMachineEmergencySwitchInput.byteAddress, justForIOAddressFor_RearMachineEmergencySwitchInput.bitAddress);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* cylinderToMove *");
	cStringForDebug.AppendFormat(L"\n");
	CString cStringCylinderInString;
	CylinderOscillateInfo::CylinderToCString(cylinder, cStringCylinderInString);
	cStringForDebug.AppendFormat(L"%s", cStringCylinderInString.GetString());
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* this cylinder Should End With This Position..(%s) *", thisCylinderShouldEndWithForwardMove == true ? L"forward" : L"backward");
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* eventSetsButJustForIOAddressForForward *");
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"input : (%d.%d) invert : (%u)", eventSetsButJustForIOAddressForForward.inputEventSet.byteAddress, eventSetsButJustForIOAddressForForward.inputEventSet.bitAddress, eventSetsButJustForIOAddressForForward.inputEventSet.invert);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"output : (%d.%d) invert : (%u)", eventSetsButJustForIOAddressForForward.outputEventSet.byteAddress, eventSetsButJustForIOAddressForForward.outputEventSet.bitAddress, eventSetsButJustForIOAddressForForward.outputEventSet.invert);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* eventSetsButJustForIOAddressForBackward *");
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"input : (%d.%d) invert : (%u)", eventSetsButJustForIOAddressForBackward.inputEventSet.byteAddress, eventSetsButJustForIOAddressForBackward.inputEventSet.bitAddress, eventSetsButJustForIOAddressForBackward.inputEventSet.invert);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"output : (%d.%d) invert : (%u)", eventSetsButJustForIOAddressForBackward.outputEventSet.byteAddress, eventSetsButJustForIOAddressForBackward.outputEventSet.bitAddress, eventSetsButJustForIOAddressForBackward.outputEventSet.invert);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* repetitions : %d *", repetitions);
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"* full path of xml file , created? (%s) * ", createXMLResult == true ? L"yes!" : L"no..");
	cStringForDebug.AppendFormat(L"\n");
	cStringForDebug.AppendFormat(L"%s", cStringForFullFilePath.GetString());
	cStringForDebug.AppendFormat(L"\n");

	//#28. 도중 정지 했는지 여부확인
	if (emergencyStopSignal == true)
	{
		cStringForEveryError.Append(L"emergency stoped\n");
		/* 여기 대신에 마지막에 끌거야
		if (userMemoryApiForThread.SetMBitEx(CylinderOscillator::BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, CylinderOscillator::BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, 0) != wmx3Api::ErrorCode::None)
		{
			//TRACE(_T("[PWR] [CYLINDER_TESTER %s] %s"), (LPCTSTR)cStringCylinderInString, L"userMemoryApiForThread.SetMBitEx : failed!");//
			cStringForEveryError.Append(L"!userMemoryApiForThread.SetMBitEx : failed!\n");
		}
		*/
	}

	//#29. 디버그용 마지막에 TRACE 하기
	CString cStringInManyLinesForTrace;
	cStringInManyLinesForTrace.Format(L"**ForDebugging\n%s\n**end of ForDebugging\n**error logs\n%s\n**end of error logs\n", (LPCTSTR)cStringForDebug, (LPCTSTR)cStringForEveryError);
	//CTokenizer + for문 이용해서 여러줄의 문장을 TRACE 여러번 호출해서 해결 (한번에 여러줄 TRACE 못하는 문제)
	CTokenizer cTokenizerInstance = CTokenizer(cStringInManyLinesForTrace, _T("\n"), FALSE);
	cStringInManyLinesForTrace.Empty();
	for (int i = 0; i < cTokenizerInstance.GetCount(); i++)
	{
		CString cStringForTRACE;
		cStringForTRACE.Format(L"[PWR] [CYLINDER_TESTER %s]", (LPCTSTR)cStringCylinderInString);
		cStringForTRACE.AppendFormat(L" %s", (LPCTSTR)cTokenizerInstance.GetString(i));
		TRACE(_T("%s"), cStringForTRACE);
		cStringForTRACE.Empty();
	}
	cStringForDebug.Empty();
	cStringForEveryError.Empty();
	cStringForFullFilePath.Empty();

	//#30. closeDevice, return : 30단계나 되네요 만들다보니.. 약 800줄
	if (userMemoryApiForThread.SetMBitEx(CylinderOscillator::BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, CylinderOscillator::BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, 0) != wmx3Api::ErrorCode::None)
	{
		//CloseDevice직전에 MBit 무조건 끄고 종료.
		TRACE(_T("[PWR] [CYLINDER_TESTER %s] %s"), (LPCTSTR)cStringCylinderInString, L"userMemoryApiForThread.SetMBitEx : failed!");
		//cStringForEveryError.Append(L"!userMemoryApiForThread.SetMBitEx : failed!\n");
	}

	/*
	* 20250530_중복 동작 방지
	* NEW_DEVELOP, Machine manual Action Running Sign, UnLock
	*/
	SetMachineManualActionRunninSign(false);

	cylinderOscillateInfo.isBeingUsed = false;//다 썼으니까 false라고 알려주기.
	cStringCylinderInString.Empty();
	deviceForThread.CloseDevice();
	delete cylinderOscillateInfo.timeToMoveForward;//위에서 new 한 메모리 해제.
	delete cylinderOscillateInfo.timeToMoveBackward;
	return 0;//이 함수는 return 분기가 딱 2개입니다. 여기 아니면 함수 맨-위(#0단계 param 값 이상할 때)
}//end of CylinderOscillator::staticFunctionForGeneralThreading.

UINT CylinderOscillator::delayedIOSetForSimuApi(LPVOID parameter)
{
	delayedIOSet* delayedIoSetInstance = reinterpret_cast<delayedIOSet*>(parameter);
	const int byteAddress = delayedIoSetInstance->byteAddress;
	const int bitAddress = delayedIoSetInstance->bitAddress;
	const int setToThis_OneOrZero = delayedIoSetInstance->setToThis_OneOrZero;
	const int timeInMilliseconds = delayedIoSetInstance->timeInMilliseconds;
	delayedIoSetInstance->copyCompleted = true;

	wmx3Api::WMX3Api		deviceForAdditionalThread;
	wmx3Api::simuApi::Simu	simuApiForAdditionalThread(&deviceForAdditionalThread);

	deviceForAdditionalThread.CreateDevice(_T("C:\\Program Files\\SoftServo\\WMX3"));
	//deviceForAdditionalThread.SetDeviceName(strdup(CylinderOscillator::GetInstance().deviceNameForIOSignalSimulation));//
	deviceForAdditionalThread.SetDeviceName(const_cast<wchar_t*>(CylinderOscillator::GetInstance().deviceNameForIOSignalSimulation));//

	std::this_thread::sleep_for(std::chrono::milliseconds(timeInMilliseconds));

	simuApiForAdditionalThread.SetInBit(byteAddress, bitAddress, setToThis_OneOrZero);

	deviceForAdditionalThread.CloseDevice();
	return 0;
}

bool CylinderOscillator::setEmergencyMemoryOn(CString& cStringForErrorMessage)
{
	if (this->wmx3Api.IsDeviceValid() == false)
	//디바이스가 없는경우 : 단발성으로 디바이스 만들고 바로 디바이스 닫기.
	{
		if (this->wmx3Api.CreateDevice(L"C:\\Program Files\\SoftServo\\WMX3") != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Append(L"this->wmx3Api.CreateDevice failed");
			return false;
		}
		if (this->wmx3Api.SetDeviceName(strdup(this->deviceName)) != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Append(L"this->wmx3Api.SetDeviceName failed");
			return false;
		}
		/*
		* 이거 만약에 연속으로 3 50 8 이 날라와서 스레드가 종료 후에 바로 또 켜버려가지구 문제될까봐 또 못키게 하려고 했는데 이것 보다는 차라리 스레드에서 TRACE 이후에 끄도록 해서 즉시 종료되도록 하는게 더 낫다.
		unsigned char getMBitExResult;
		if (this->userMemory.GetMBitEx(this->BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, this->BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, &getMBitExResult) != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Format(L"GetMBitEx Failed : CylinderOscillator::setEmergencyMemoryOn");
			return false;
		}
		if (getMBitExResult == 1)//이미 켜져있으면 또 키는 명령은 금지?
		{
			cStringForErrorMessage.Format(L"MBit is already ON...");
			return false;
		}
		*/
		if (this->userMemory.SetMBitEx(this->BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, this->BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, 1) != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Format(L"SetMBitEx Failed : CylinderOscillator::setEmergencyMemoryOn");
			return false;
		}
		if (this->wmx3Api.CloseDevice() != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Format(L"this->wmx3Api.CloseDevice() Failed : CylinderOscillator::setEmergencyMemoryOn");
			return false;
		}
		return true;
	}
	else
	//디바이스가 있는경우 : 그대로 열어놓고 setMBitEx만 켜기.
	{
		if (this->userMemory.SetMBitEx(this->BYTE_ADDRESS_FOR_EMERGENCY_USER_MEMORY, this->BIT_ADDRESS_FOR_EMERGENCY_USER_MEMORY, 1) != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Format(L"SetMBitEx Failed : CylinderOscillator::setEmergencyMemoryOn");
			return false;
		}
		if (this->wmx3Api.CloseDevice() != wmx3Api::ErrorCode::None)
		{
			cStringForErrorMessage.Format(L"this->wmx3Api.CloseDevice() Failed : CylinderOscillator::setEmergencyMemoryOn");
			return false;
		}
		return true;
	}
}
