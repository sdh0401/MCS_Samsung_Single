#include "pch.h"
#include "CylinderOscillateInfo.h"
#include "CylinderOscillator.h"
//#include "../../../../Header/IODefine.h"
#include "GlobalIODefine.h"

#include "tinyxml2.h"//for xml
#include <filesystem>//for createing directory/folder
#include "CPowerIO.h"

CylinderOscillateInfo::CylinderOscillateInfo()
	: cylinderToMove(CylinderOscillateInfo::T::FRONT_STOPPER)
	, repetitions(-1)
	, timeToMoveForward(NULL)
	, timeToMoveBackward(NULL)
	, isBeingUsed(false)
	, delay(-1)
{
}

CylinderOscillateInfo::~CylinderOscillateInfo()
{
	delete this->timeToMoveForward;
	delete this->timeToMoveBackward;
}

bool CylinderOscillateInfo::CylinderToEverything(const CylinderOscillateInfo::T& cylinder, CString& cString, 			//#1.
										   int& eventIdForMovingForward, int& eventIdForMovingBackward, 	//#2.
										   CylinderOscillateInfo::EventSetsForEvent& eventSetsForForwardEvent,	//#3-1.
										   CylinderOscillateInfo::EventSetsForEvent& eventSetsForBackwardEvent	//#3-2.
)
/*
* input : CylinderOscillateInfo::T 실린더.
* output : CString, EventId(2개), CylinderOscillateInfo::EventSetsForEvent(2개)
* 
* #1. UP, DOWN에 대한 IO INPUT 신호가 하나라도 정의되어있지 않다면, false 를 반환합니다.
* 다시말해 IOStruct::Usage가 NO_USE인 경우입니다. (CPowerIO class의 private 인스턴스 변수 IOStruct array 타입의 m_IOConfig 참고)
* 
* #2. OUTPUT 신호의 경우 전진이 정의되어있지 않다면 false 반환. 다만 후진의 경우 정의되지 않은경우 전진신호를 끄는 방식으로 세팅합니다.
* 
*/
{
	if (gcPowerIO == NULL)
	{
		return false;
	}
	cString.Empty();

	//#0.0 기본적인 초기화
	/*
	* 실질적으로 현재 struct CylinderOscillateInfo::EventSetsForEvent에서 쓰는건 byteAddress bitAddress invert 세가지이다..
	* inputFunction, delay 는 사용 안함 (삭제예정)
	*/
	eventSetsForForwardEvent.inputEventSet.inputFunction = wmx3Api::IoEventInput::IOBit;
	eventSetsForForwardEvent.inputEventSet.byteAddress;//그냥 쓴거 (동작 없음)
	eventSetsForForwardEvent.inputEventSet.bitAddress;//
	eventSetsForForwardEvent.inputEventSet.delay = -1;
	eventSetsForForwardEvent.inputEventSet.invert = 0;
	eventSetsForForwardEvent.outputEventSet.outputFunction = wmx3Api::IoEventOutput::SetIOOutBit;
	eventSetsForForwardEvent.outputEventSet.byteAddress;//
	eventSetsForForwardEvent.outputEventSet.bitAddress;//
	eventSetsForForwardEvent.outputEventSet.invert = 0;

	eventSetsForBackwardEvent.inputEventSet.inputFunction = wmx3Api::IoEventInput::IOBit;
	eventSetsForBackwardEvent.inputEventSet.byteAddress;//
	eventSetsForBackwardEvent.inputEventSet.bitAddress;//
	eventSetsForBackwardEvent.inputEventSet.delay = -1;
	eventSetsForBackwardEvent.inputEventSet.invert = 0;
	eventSetsForBackwardEvent.outputEventSet.outputFunction = wmx3Api::IoEventOutput::SetIOOutBit;
	eventSetsForBackwardEvent.outputEventSet.byteAddress;//
	eventSetsForBackwardEvent.outputEventSet.bitAddress;//
	eventSetsForBackwardEvent.outputEventSet.invert = 0;

	//#0.1 IN/OUT 신호 4가지 저장할 지역변수 ( 매개변수 cylinder에 따라 switch-case문 내부에서 다르게 할당. )
	int _IN_SENSOR_FORWARD, _IN_SENSOR_BACKWARD, _OUT_SOLENOID_FORWARD, _OUT_SOLENOID_BACKWARD;

	_IN_SENSOR_FORWARD = _IN_SENSOR_BACKWARD = _OUT_SOLENOID_FORWARD = _OUT_SOLENOID_BACKWARD = -1;//-1로 초기화

/*
* 모든 case는 다음과 같은 순서로 처리

		//#1. cString 정의

		//#2. 지역변수 값 할당

		//#3. break; 끝.

		//@. 아래는 case문 지나서

		//#. 센서 INPUT IO 양쪽중에 하나라도 없으면(OR조건) false.

		//#. 솔레노이드 Out 신호의 경우, 전진과 후진 둘중에 하나만 살아있어도 (편솔레노이드 밸브) 전/후진이 가능하기 때문에, 둘다 정의되지 않은 경우 (AND조건)에만 false.

		//#. getIOAddressAndIOBitFromgcPowerIO 호출. (4번)

		//#. 전/후진 OUT신호 중 정의되지 않은게 있으면 해당 신호를 반대 신호의 끄는신호로 바꿔준다. + 대신에 INVERT는 1 해줘야지. (둘다 정의되지 않은경우는 위에서 걸렀기 때문에 여기에서 고려 안함)
*
*	IN/OUT 신호가 전진/후진 총 4가지라서 헷갈리지않게 조심할것.
*/
	switch (cylinder)
	{

	case CylinderOscillateInfo::T::FRONT_STOPPER:
	{
		//#1. cString 정의
		cString.Format(L"FRONT_STOPPER");

		//#2. 밖에서 처리하기위한 지역변수 값 할당.
		_IN_SENSOR_FORWARD = IN_FCONV_WORK1_STOP_UP;
		_IN_SENSOR_BACKWARD = IN_FCONV_WORK1_STOP_DOWN;
		_OUT_SOLENOID_FORWARD = OUT_FCONV_WORK1_STOP_UP;
		_OUT_SOLENOID_BACKWARD = OUT_FCONV_WORK1_STOP_DN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::FRONT_PUSHER_PLATE:
	{
		//#1 cString 정의
		cString.Format(L"FRONT_PUSHER_PLATE");

		//#3~6을 밖에서 코드반복없이 처리하기위한 지역변수 값 할당.
		_IN_SENSOR_FORWARD = IN_FCONV_WORK1_PUSH_UP;
		_IN_SENSOR_BACKWARD = IN_FCONV_WORK1_PUSH_DN;
		_OUT_SOLENOID_FORWARD = OUT_FCONV_WORK1_PUSH_UP;
		_OUT_SOLENOID_BACKWARD = OUT_FCONV_WORK1_PUSH_DN;

		//#7. break;
		break;
	}//case CylinderOscillateInfo::T::FRONT_PUSHER_PLATE:

	case CylinderOscillateInfo::T::FRONT_ANC_UP_DOWN:
	{
		//#1. cString 정의
		cString.Format(L"FRONT_ANC_UP_DOWN");

		//#3~6을 코드반복 줄이기) 밖에서 처리하기위한 지역변수 값 할당.
		_IN_SENSOR_FORWARD = IN_FANC_BASE_UP;
		_IN_SENSOR_BACKWARD = IN_FANC_BASE_DN;
		_OUT_SOLENOID_FORWARD = OUT_FANC_BASE_UP;
		_OUT_SOLENOID_BACKWARD = OUT_FANC_BASE_DN;

		//#7. 마지막에 꼭 break;
		break;
	}//case CylinderOscillateInfo::T::FRONT_ANC_UP_DOWN:

	case CylinderOscillateInfo::T::FRONT_ANC_LOCK_UNLOCK:
	{
		//#1. cString 정의
		cString.Format(L"FRONT_ANC_LOCK_UNLOCK");

		//FORWARD(전진) : LOCK 동작, BACKWARD(후진) : UNLOCK 동작 입니다. (HMI랑 약속)

		//#3~6을 코드반복 줄이기) 밖에서 처리하기위한 지역변수 값 할당.
		_IN_SENSOR_FORWARD = IN_FANC_CLAMP_LOCK;
		_IN_SENSOR_BACKWARD = IN_FANC_CLAMP_UNLOCK;
		_OUT_SOLENOID_FORWARD = OUT_FANC_CLAMP_LOCK;
		_OUT_SOLENOID_BACKWARD = OUT_FANC_CLAMP_UNLOCK;

		break;//마지막에 꼭 break;
	}//case CylinderOscillateInfo::T::FRONT_ANC_LOCK_UNLOCK:

	case CylinderOscillateInfo::T::TURNUNIT_LEFT_POCKET_UP_DOWN:
	{
		//#1. cString 정의
		cString.Format(L"TURNUNIT_LEFT_POCKET_UP_DOWN");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_TURNUNIT_LEFT_POCKET_UP;
		_IN_SENSOR_BACKWARD = IN_TURNUNIT_LEFT_POCKET_DOWN;
		_OUT_SOLENOID_FORWARD = OUT_TURNUNIT_POCKET_UP;
		_OUT_SOLENOID_BACKWARD = OUT_TURNUNIT_POCKET_DOWN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::TURNUNIT_RIGHT_POCKET_UP_DOWN:
	{
		//#1. cString 정의
		cString.Format(L"TURNUNIT_RIGHT_POCKET_UP_DOWN");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_TURNUNIT_RIGHT_POCKET_UP;
		_IN_SENSOR_BACKWARD = IN_TURNUNIT_RIGHT_POCKET_DOWN;
		_OUT_SOLENOID_FORWARD = OUT_TURNUNIT_POCKET_UP;
		_OUT_SOLENOID_BACKWARD = OUT_TURNUNIT_POCKET_DOWN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::TURNUNIT_PUSHER_FORWARD_BACKWARD:
	{
		//#1. cString 정의
		cString.Format(L"TURNUNIT_PUSHER_FORWARD_BACKWARD");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_TURNUNIT_PUSHER_FORWARD;
		_IN_SENSOR_BACKWARD = IN_TURNUNIT_PUSHER_BACKWARD;
		_OUT_SOLENOID_FORWARD = OUT_TURNUNIT_PUSHER_FORWARD;
		const int OUT_TURNUNIT_PUSHER_BACKWARD = MAXIODEFINE;//IODefine.h 에 이거 자체가 없어서 그냥 임의 추가 (아래 IsUsedIO 에서 false 걸리는 값으로..) 만약 나중에 생기면(안생기겠지만) 이거 한줄만 지우거나 주석처리하면 됩니다.
		_OUT_SOLENOID_BACKWARD = OUT_TURNUNIT_PUSHER_BACKWARD;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::REAR_STOPPER:
	{
		//#1. cString 정의
		cString.Format(L"REAR_STOPPER");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_RCONV_WORK1_STOP_UP;
		_IN_SENSOR_BACKWARD = IN_RCONV_WORK1_STOP_DOWN;
		_OUT_SOLENOID_FORWARD = OUT_RCONV_WORK1_STOP_UP;
		_OUT_SOLENOID_BACKWARD = OUT_RCONV_WORK1_STOP_DN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::REAR_PUSHER_PLATE:
	{
		//#1. cString 정의
		cString.Format(L"REAR_PUSHER_PLATE");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_RCONV_WORK1_PUSH_UP;
		_IN_SENSOR_BACKWARD = IN_RCONV_WORK1_PUSH_DN;
		_OUT_SOLENOID_FORWARD = OUT_RCONV_WORK1_PUSH_UP;
		_OUT_SOLENOID_BACKWARD = OUT_RCONV_WORK1_PUSH_DN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::REAR_ANC_UP_DOWN:
	{
		//#1. cString 정의
		cString.Format(L"REAR_ANC_UP_DOWN");

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD = IN_RANC_BASE_UP;
		_IN_SENSOR_BACKWARD = IN_RANC_BASE_DN;
		_OUT_SOLENOID_FORWARD = OUT_RANC_BASE_UP;
		_OUT_SOLENOID_BACKWARD = OUT_RANC_BASE_DN;

		//#3. 마지막에 꼭 break;
		break;
	}

	case CylinderOscillateInfo::T::REAR_ANC_LOCK_UNLOCK:
	{
		//#1. cString 정의
		cString.Format(L"REAR_ANC_LOCK_UNLOCK");

		//FORWARD(전진) : LOCK 동작, BACKWARD(후진) : UNLOCK 동작 입니다. (개발자 마음)

		//#2. 지역변수4개 값 할당.
		_IN_SENSOR_FORWARD		= IN_RANC_CLAMP_LOCK;
		_IN_SENSOR_BACKWARD		= IN_RANC_CLAMP_UNLOCK;
		_OUT_SOLENOID_FORWARD	= OUT_RANC_CLAMP_LOCK;
		_OUT_SOLENOID_BACKWARD	= OUT_RANC_CLAMP_UNLOCK;

		//#3. 마지막에 꼭 break;
		break;
	}

	default://여기 걸리면 무조건 false 리턴
	{
		cString.Format(L"**undefined_cylinder");
		return false;
	}
	}//switch-case 끝.

	//(없겠지만) 하나라도 지역변수값 재정의가 안된 부분이 있으면 false return (여기 걸리면 개발자 실수)
	if (_IN_SENSOR_FORWARD == -1 || _IN_SENSOR_BACKWARD == -1 || _OUT_SOLENOID_FORWARD == -1 || _OUT_SOLENOID_BACKWARD == -1)
	{
		return false;
	}

	const int IN_SENSOR_FORWARD		= _IN_SENSOR_FORWARD;
	const int IN_SENSOR_BACKWARD	= _IN_SENSOR_BACKWARD;
	const int OUT_SOLENOID_FORWARD	= _OUT_SOLENOID_FORWARD;
	const int OUT_SOLENOID_BACKWARD = _OUT_SOLENOID_BACKWARD;

	//#3. 센서 INPUT IO 양쪽중에 하나라도 없으면(OR조건) false.
	if (gcPowerIO->IsUsedIO(IN_SENSOR_FORWARD) == false || gcPowerIO->IsUsedIO(IN_SENSOR_BACKWARD) == false)
	{
		return false;
	}

	//#4. 솔레노이드 Out 신호의 경우, 전진과 후진 둘중에 하나만 살아있어도 (편솔레노이드 밸브) 전/후진이 가능하기 때문에, 둘다 정의되지 않은 경우 (AND조건)에만 false.
	if (gcPowerIO->IsUsedIO(OUT_SOLENOID_FORWARD) == false && gcPowerIO->IsUsedIO(OUT_SOLENOID_BACKWARD) == false)
	{
		return false;
	}

	//#5. getIOAddressAndIOBitFromgcPowerIO 호출. (4번)
	if (
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_SENSOR_BACKWARD,		eventSetsForForwardEvent.inputEventSet.byteAddress,		eventSetsForForwardEvent.inputEventSet.bitAddress)		== false 
		||
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(OUT_SOLENOID_FORWARD,		eventSetsForForwardEvent.outputEventSet.byteAddress,	eventSetsForForwardEvent.outputEventSet.bitAddress)		== false 
		||
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(IN_SENSOR_FORWARD,		eventSetsForBackwardEvent.inputEventSet.byteAddress,	eventSetsForBackwardEvent.inputEventSet.bitAddress)		== false 
||
		CylinderOscillator::getIOAddressAndIOBitFromgcPowerIO(OUT_SOLENOID_BACKWARD,	eventSetsForBackwardEvent.outputEventSet.byteAddress,	eventSetsForBackwardEvent.outputEventSet.bitAddress)	== false
		)
	{
		return false;
	}

	//#6. 전/후진 OUT신호 중 정의되지 않은게 있으면 해당 신호를 반대 신호의 끄는신호로 바꿔준다. + 그리고 INVERT는 1 해줘야지. (둘다 정의되지 않은경우는 위에서 걸렀기 때문에 여기에서 고려 안함)
	if (gcPowerIO->IsUsedIO(OUT_SOLENOID_FORWARD) == false)
	{
		eventSetsForForwardEvent.outputEventSet.byteAddress = eventSetsForBackwardEvent.outputEventSet.byteAddress;
		eventSetsForForwardEvent.outputEventSet.bitAddress = eventSetsForBackwardEvent.outputEventSet.bitAddress;
		eventSetsForForwardEvent.outputEventSet.invert = 1;
	}
	else if (gcPowerIO->IsUsedIO(OUT_SOLENOID_BACKWARD) == false)
	{
		eventSetsForBackwardEvent.outputEventSet.byteAddress = eventSetsForForwardEvent.outputEventSet.byteAddress;
		eventSetsForBackwardEvent.outputEventSet.bitAddress = eventSetsForForwardEvent.outputEventSet.bitAddress;
		eventSetsForBackwardEvent.outputEventSet.invert = 1;
	}

	/*마지막에 (input / output 신호 유효성 검증) 정의되지 않은 OUTPUT도 반대신호 끄기로 등록을 마쳤기 때문에 모든 값이 WOS의 IO신호범위 내 이어야 한다.*/
	if ( eventSetsForForwardEvent.inputEventSet.byteAddress   < 0 || 7999 < eventSetsForForwardEvent.inputEventSet.byteAddress ||
		 eventSetsForForwardEvent.inputEventSet.bitAddress    < 0 ||    7 < eventSetsForForwardEvent.inputEventSet.bitAddress ||
		 eventSetsForBackwardEvent.inputEventSet.byteAddress  < 0 || 7999 < eventSetsForBackwardEvent.inputEventSet.byteAddress ||
		 eventSetsForBackwardEvent.inputEventSet.bitAddress   < 0 ||    7 < eventSetsForBackwardEvent.inputEventSet.bitAddress || 
		 eventSetsForForwardEvent.outputEventSet.byteAddress  < 0 || 7999 < eventSetsForForwardEvent.outputEventSet.byteAddress || 
		 eventSetsForForwardEvent.outputEventSet.bitAddress   < 0 ||    7 < eventSetsForForwardEvent.outputEventSet.bitAddress ||
		 eventSetsForBackwardEvent.outputEventSet.byteAddress < 0 || 7999 < eventSetsForBackwardEvent.outputEventSet.byteAddress ||
		 eventSetsForBackwardEvent.outputEventSet.bitAddress  < 0 ||    7 < eventSetsForBackwardEvent.outputEventSet.bitAddress )
	{
		//input, output 신호 하나라도 범위 바깥의 것이 나오면 false (OR 연산)
		return false;
	}

	//#99. true return(정상종료)
	return true;
}//end of CylinderOscillateInfo::CylinderToEverything

void CylinderOscillateInfo::CylinderToCString(const CylinderOscillateInfo::T& cylinder, CString& cString)
{
	int a, b;
	CylinderOscillateInfo::EventSetsForEvent c, d;
	if (CylinderOscillateInfo::CylinderToEverything(cylinder, cString, a, b, c, d) == false)
	{
		/*
		* 이 부분이 없으면 MCS.exe를 초기화(1 1 0 커맨드) 하지 않고 cylinder helper dialog를 열었을 때 무한로딩 걸립니다. (중요한 문제)
		*/
		//cString.Format(L"**undefined_cylinder");

		cString.Empty();//지금은 괜찮아.
	}
}

bool CylinderOscillateInfo::fromCylinderTo2EventSets(const CylinderOscillateInfo::T& cylinder, 
											   CylinderOscillateInfo::EventSetsForEvent& eventSetsForForwardEvent, 
											   CylinderOscillateInfo::EventSetsForEvent& eventSetsForBackwardEvent)
{
	int a, b;
	CString cs;
	if (CylinderOscillateInfo::CylinderToEverything(cylinder, cs, a, b, eventSetsForForwardEvent, eventSetsForBackwardEvent) == false)
	{
		return false;
	}
	return true;
}

bool CylinderOscillateInfo::createXML(CString& cStringForEveryError, CString& cStringForFullFilePath)
{
	//NewElement -> Set -> Insert
	
	//#0. 초기화
	int _realSizeOfForwardArray = this->repetitions;
	for (int i = 0; i < this->repetitions; i++)
	{
		if (this->timeToMoveForward[i] == 0)
		{
			_realSizeOfForwardArray = i;//중간에 중지한 경우 유효하지 않은 0이 들어가서 통계자료를 해치지 않기위해 size를 수정.
			break;
		}
		else
		{
			continue;
		}
	}
	int _realSizeOfBackwardArray = this->repetitions;
	for (int i = 0; i < this->repetitions; i++)
	{
		if (this->timeToMoveBackward[i] == 0)
		{
			_realSizeOfBackwardArray = i;//중간에 중지한 경우 유효하지 않은 0이 들어가서 통계자료를 해치지 않기위해 size를 수정.
			break;
		}
		else
		{
			continue;
		}
	}
	//0으로 나누는 불상사 방지하기위해 최솟값 1 준다.
	if (_realSizeOfForwardArray == 0)
	{
		_realSizeOfForwardArray = 1;
	}
	if (_realSizeOfBackwardArray == 0)
	{
		_realSizeOfBackwardArray = 1;
	}
	const int realSizeOfForwardArray = _realSizeOfForwardArray;
	const int realSizeOfBackwardArray = _realSizeOfBackwardArray;

	//#1. XMLDocument 객체 생성
	tinyxml2::XMLDocument xmlDocumentInstance;

	//#2.root Node
	tinyxml2::XMLNode* rootNode = xmlDocumentInstance.NewElement("root");
	xmlDocumentInstance.InsertFirstChild(rootNode);

	//input Node
	tinyxml2::XMLNode* inputNode = xmlDocumentInstance.NewElement("input");
	rootNode->InsertFirstChild(inputNode);

	//name Element
	tinyxml2::XMLElement* cylinderElement = xmlDocumentInstance.NewElement("cylinder");
	CString cStringForName;
	CylinderOscillateInfo::CylinderToCString(this->cylinderToMove, cStringForName);

	if (cStringForName.IsEmpty() == true)
	{
		cStringForEveryError.Append(L"CylinderOscillateInfo::createXML -> CylinderOscillateInfo::CylinderToCString failed : cStringForName is empty.");
		cStringForFullFilePath.Append(L"0");
		return false;//중간에 return. 유효하지 않은 데이터 거르기.
	}

	cylinderElement->SetText((CStringA)cStringForName);//아래에서 empty 함.
	inputNode->InsertFirstChild(cylinderElement);

	//reps Element
	tinyxml2::XMLElement* repsElement = xmlDocumentInstance.NewElement("repetitions");
	repsElement->SetText((int)this->repetitions);
	inputNode->InsertAfterChild(cylinderElement, repsElement);

	//output Node
	tinyxml2::XMLNode* outputNode = xmlDocumentInstance.NewElement("output");
	rootNode->InsertAfterChild(inputNode, outputNode);

	//"mean" Node
	tinyxml2::XMLNode* meanNode = xmlDocumentInstance.NewElement("mean");
	outputNode->InsertFirstChild(meanNode);

	//meanForwardElement Element
	tinyxml2::XMLElement* meanForwardElement = xmlDocumentInstance.NewElement("up");//mean 양쪽에 대해서 수정사항 있음.
	unsigned long long totalForwardTime = 0;
	for (int i = 0; i < this->repetitions; i++)
	{
		totalForwardTime += this->timeToMoveForward[i];
	}
	const float meanForwardTime = static_cast<float>(totalForwardTime) / static_cast<float>(realSizeOfForwardArray);
	CString cStringForMeanForwardTime;
	cStringForMeanForwardTime.Format(L"%0.3f", meanForwardTime);
	meanForwardElement->SetText((CStringA)cStringForMeanForwardTime);
	cStringForMeanForwardTime.Empty();
	meanNode->InsertFirstChild(meanForwardElement);

	//meanBackwardElement Element
	tinyxml2::XMLElement* meanBackwardElement = xmlDocumentInstance.NewElement("down");
	unsigned long long totalBackwardTime = 0;
	for (int i = 0; i < this->repetitions; i++)
	{
		totalBackwardTime += this->timeToMoveBackward[i];
	}
	const float meanBackwardTime = static_cast<float>(totalBackwardTime) / static_cast<float>(realSizeOfBackwardArray);
	CString cStringForMeanBackwardTime;
	cStringForMeanBackwardTime.Format(L"%0.3f", meanBackwardTime);
	meanBackwardElement->SetText((CStringA)cStringForMeanBackwardTime);
	cStringForMeanBackwardTime.Empty();
	meanNode->InsertAfterChild(meanForwardElement, meanBackwardElement);

	//"Standard Deviation" Node
	tinyxml2::XMLNode* standardDeviationElement = xmlDocumentInstance.NewElement("standard_deviation");
	outputNode->InsertAfterChild(meanNode, standardDeviationElement);

	//standardDeviationForward Element
	tinyxml2::XMLElement* standardDeviationForward = xmlDocumentInstance.NewElement("up");
	CString cStringForStandardDeviationForward;
	cStringForStandardDeviationForward.Format(L"%0.3Lf", this->getStandardDeviation(this->timeToMoveForward, realSizeOfForwardArray));
	standardDeviationForward->SetText((CStringA)cStringForStandardDeviationForward);
	cStringForStandardDeviationForward.Empty();
	standardDeviationElement->InsertFirstChild(standardDeviationForward);

	//standardDeviationBackward Element
	tinyxml2::XMLElement* standardDeviationBackward = xmlDocumentInstance.NewElement("down");
	CString cStringForStandardDeviationBackward;
	cStringForStandardDeviationBackward.Format(L"%0.3Lf", this->getStandardDeviation(this->timeToMoveBackward, realSizeOfBackwardArray));
	standardDeviationBackward->SetText((CStringA)cStringForStandardDeviationBackward);
	cStringForStandardDeviationBackward.Empty();
	standardDeviationElement->InsertAfterChild(standardDeviationForward, standardDeviationBackward);

	//details Node
	tinyxml2::XMLNode* detailsNode = xmlDocumentInstance.NewElement("details");
	outputNode->InsertAfterChild(standardDeviationElement, detailsNode);

	//details Element
	for (int i = 0; i < this->repetitions; i++)
	{
		tinyxml2::XMLElement* detailsElement = xmlDocumentInstance.NewElement("detail");
		detailsElement->SetAttribute("number", i);
		detailsElement->SetAttribute("up", this->timeToMoveForward[i]);
		detailsElement->SetAttribute("down", this->timeToMoveBackward[i]);
		detailsNode->InsertEndChild(detailsElement);
	}

	//#4. 디렉토리 생성
	struct tm newtime;
	time_t now = time(0);
	localtime_s(&newtime, &now);

	//cStringForFullFilePath.Format(L"C:/Power/i6.0/CylinderTestResult");//삭제예정.
	cStringForFullFilePath.Format(L"D:/CylinderTestResult");
	cStringForFullFilePath.AppendFormat(L"/%04d%02d%02d", newtime.tm_year + 1900, newtime.tm_mon + 1, newtime.tm_mday);

	std::filesystem::path path = ((CStringA)cStringForFullFilePath).GetBuffer();
	std::filesystem::create_directories(path);

	cStringForFullFilePath.AppendFormat(L"/%02d%02d%02d_", newtime.tm_hour, newtime.tm_min, newtime.tm_sec);
	cStringForFullFilePath.Append(cStringForName);
	cStringForFullFilePath.Append(L".xml");

	cStringForName.Empty();

	//#5. SaveFile
	tinyxml2::XMLError error = xmlDocumentInstance.SaveFile(((CStringA)cStringForFullFilePath).GetBuffer());
	if (error == tinyxml2::XMLError::XML_SUCCESS)
	{
		return true;
	}
	else
	{
		const char* string = tinyxml2::XMLDocument::ErrorIDToName(error);
		cStringForEveryError.Append((CString)string);
		cStringForEveryError.Append(L"\n");
		return false;
	}

}//end of CylinderOscillateInfo::createXML.

//표준편차를 return 하는 함수입니다.
long double CylinderOscillateInfo::getStandardDeviation(const unsigned long long* numArray, const int arraySize)
{
	//#0 sum 구하기
	unsigned long long _sum = 0;

	for (int i = 0; i < arraySize; i++)
	{
		_sum += numArray[i];
	}

	const unsigned long long sum = _sum;

	//#1. mean 구하기
	const long double mean = static_cast<long double>(sum) / static_cast<long double>(arraySize);

	//#2. variance (분산) = (편차의 제곱의 총합 / 전체개수) 구하기
	long double _variance = 0;

	for (int i = 0; i < arraySize; i++)
	{
		_variance += (mean - numArray[i]) * (mean - numArray[i]);
	}

	_variance = _variance / arraySize;

	const long double variance = _variance;

	//#3. standard deviation (표준편차) = (분산의 제곱근) 구하기
	long double _standardDeviation = variance;

	_standardDeviation = sqrt(_standardDeviation);

	const long double standardDeviation = _standardDeviation;

	return standardDeviation;
}