#include "pch.h"
#include "CReadJobFile.h"
#include "AxisInformation.h"
#include "CApplicationTime.h"
#include "GlobalDefine.h"
//#include "ErrorCode.h"
#include "CTokenizer.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "CStep.h"
#include <list>
#include "CMachineConfig.h"
#include "CLastPick.h"

CReadJobFile* gcReadJobFile;
CReadJobFile::CReadJobFile()
{
	Initial();
	SetDir(_T("C:\\Power\\i6.0\\RUN"));
	SetHeader(_T("power"));
	SetExtension(_T("run"));
	MakeFileName();
}

CReadJobFile::~CReadJobFile()
{
}

INSERT CReadJobFile::GetInsert(const long& gantry, const long& insertNo)
{
    if (gantry == FRONT_GANTRY)
    {
        return this->GetInsert(insertNo);
    }
    TRACE_FILE_FUNC_LINE_"bad request");
    return INSERT{};
}

long CReadJobFile::AddBlockSequence()
{
    long Err = NO_ERR;
    const PCB Pcb = this->GetPcb();//GetPcb();
    const int totalBlockCount = Pcb.MaxBlockCount;

    if (Pcb.UseBlockType == PCB_SINGLE)//기존방식인 PCB_SINGLE 블록타입이면 할게없다.
    {
        TRACE_FILE_FUNC_LINE_"early return due to (Pcb.UseBlockType == PCB_SINGLE).");
        return Err;
    }
    if (totalBlockCount < 2)//총 블록 개수가 2개미만이면 할게없다.
    {
        TRACE_FILE_FUNC_LINE_"early return due to 'totalBlockCount < 2'.");
        return Err;
    }
    if (Pcb.BlockSequenceMode != BLOCK_SEQ_PATTERN && Pcb.BlockSequenceMode != BLOCK_SEQ_STEP)//패턴이랑 스텝일 때만 동작.
    {
        TRACE_FILE_FUNC_LINE_"early return due to (Pcb.BlockSequenceMode != BLOCK_SEQ_PATTERN && Pcb.BlockSequenceMode != BLOCK_SEQ_STEP).");
        return Err;
    }

    const int totalInsertCount = this->GetProduction().TotalInsertCount;

    if (totalInsertCount < 1)
    {
        TRACE_FILE_FUNC_LINE_"early return due to (totalInsertCount < 1)");
        return Err;//NO_ERR
    }

    CString temp;
    temp.Format(L"starts. Mode: %d (%d: PATTERN, %d: STEP) before add, existing totalInsertCount: %d, totalBlockCount: %d"
                , Pcb.BlockSequenceMode, BLOCK_SEQ_PATTERN, BLOCK_SEQ_STEP, totalInsertCount, totalBlockCount);
    TRACE_FILE_FUNC_LINE_(CStringA)temp);

    if (MAXINSERTNO < Pcb.MaxBlockCount * totalInsertCount)
    {
        Err = ERR_TOTAL_INSERT_COUNT_INCLUDING_BLOCK_OVER_LIMIT;
        temp.Format(L"[blockCount] * [totalFrontInsertCount] is over MAXINSERTNO(%d)", MAXINSERTNO);
        Err = SendAlarm(Err, temp);
        return Err;
    }

    std::deque<INSERT> _existingFrontGantryInsertDeque = std::deque<INSERT>{};//기존의 모든 삽입설정 정보 저장.
    for (int i = 1; i <= totalInsertCount; i++)//일단 기존 데이터 저장. -> 싱글이니까 totalInsertCount 사용.
    {
        _existingFrontGantryInsertDeque.push_back(GetInsert(FRONT_GANTRY, i));
    }
    const std::deque<INSERT>& existingFrontGantryInsertDeque = _existingFrontGantryInsertDeque;

    std::deque<AVOIDMOTION> existingFrontAvoidMotionDeque = std::deque<AVOIDMOTION>();//기존의 모든 AVOIDMOTION 설정 정보 저장. 데크 사이즈는 삽입설정 행의 개수와 동일하다. (주의) -> 그래서 사실 개수 초과 검사할 필요 없음.
    for (int i = 0; i < existingFrontGantryInsertDeque.size(); i++)
    {
        const AVOIDMOTION avoidMotion = GetAvoidMotion(FRONT_GANTRY, existingFrontGantryInsertDeque.at(i).index);
        existingFrontAvoidMotionDeque.push_back(avoidMotion);
        //temp.Format(L"existingFrontGantryInsertDeque.index : %d, count : %d, insertNo : %d, locateID : %s", existingFrontGantryInsertDeque.at(i).index, avoidMotion.Count, avoidMotion.insertNo, (LPCTSTR)avoidMotion.LocateID);
        //TRACE_FILE_FUNC_LINE_(CStringA)temp);//꼭 필요해?
    }

    //AVOID 설정 정보 개수 초과 검사. -> 삽입설정 개수와 같아서 검사할 필요가 없다.

    //#. 기존 모든 높이검사 정보 저장. (쓰는것만저장..)->밖으로뺀다.
    std::deque<MEASUREHEIGHT> existingMeasureHeightDeque = std::deque<MEASUREHEIGHT>();//기존의 모든 MEASUREHEIGHT 설정 정보 저장.
    for (int i = 0; i < MAXINSERTNO; i++)
    {
        const MEASUREHEIGHT measureHeight = GetMeasureHeight(i);
        if (measureHeight.Use == YES_USE)
        {
            existingMeasureHeightDeque.push_back(measureHeight);
        }
    }

    //MEASUREHEIGHT 설정 개수 초과 검사
    if (MAXINSERTNO < Pcb.MaxBlockCount * existingMeasureHeightDeque.size())
    {
        Err = ERR_TOTAL_INSERT_COUNT_INCLUDING_BLOCK_OVER_LIMIT;
        temp.Format(L"[blockCount] * [existingMeasureHeightDeque.size()] is over MAXINSERTNO(%d)", MAXINSERTNO);
        Err = SendAlarm(Err, temp);
        return Err;
    }

    (void)addInsertSequence(existingFrontGantryInsertDeque, FRONT_GANTRY);

    (void)addAvoidSequence(existingFrontGantryInsertDeque, existingFrontAvoidMotionDeque, FRONT_GANTRY);//avoid 개수는 삽입설정 행의 개수와 동일

    (void)addHeightMeasureSequence(existingFrontGantryInsertDeque, existingMeasureHeightDeque);//heightMeasure 개수는..

                                                                                         //#. 끝.
    TRACE_FILE_FUNC_LINE_"returns.");
    return Err;//NO_ERR
}

int CReadJobFile::addInsertSequence(const std::deque<INSERT>& existingInsertDeque, const long& gantry)
{
    if (gantry == REAR_GANTRY)
    {
        TRACE_FILE_FUNC_LINE_"early return due to (gantry == REAR_GANTRY).");
        return 0;
    }

    TRACE_FILE_FUNC_LINE_"starts.");

    int stepNo = -1;//getNextStepDeque 전용.

    int setInsertIndex = 1;//SetInsertInfo 전용. 당연히 1번부터 사용.

    if (this->GetPcb().BlockSequenceMode == BLOCK_SEQ_PATTERN)//패턴일때 한스텝짜리 복사하는걸 기존 전체 삽입설정 가지고 복사하면 돼. 함수 안에서 약간의 코드중복..
    {
        for (int i = 1; i <= this->GetPcb().MaxBlockCount; i++)//i = 1, 2, .., MaxBlockCount
        {
            const int addStepNo = ((existingInsertDeque.back().Step) - (existingInsertDeque.front().Step) + 1) * (i - 1);//BLOCK_SEQ_STEP일때 step 번호에 얼마나 더해야할지. 기존 의 스텝 개수 ex) 기존에 2 3 5 스텝으로 해놨으면 매 블록마다 [4]를 더해야한다 (5 - 2 + 1 = [4])
            const int blockNo = i;//1, 2, .., MaxBlockCount
            for (int j = 0; j < existingInsertDeque.size(); j++)
            {
                INSERT insert = existingInsertDeque.at(j);
                insert.BlockNo = blockNo;
                insert.Step += addStepNo;
                //insert.index = setInsertIndex;
                (gantry == FRONT_GANTRY) ? SetInsert(insert, setInsertIndex) : SetInsert2(insert, setInsertIndex);//how?
                setInsertIndex++;
            }
        }
    }
	else {
		int setStepNo = existingInsertDeque.at(0).Step;//SetInsertInfo 전용. 기존 삽입설정에서 쓰는 스텝번호부터 사용.

		while (true) {
			//#. 한스텝짜리 삽입설정 데크 구하기.
			std::deque<INSERT> stepInsertDeque = std::deque<INSERT>();
			stepNo = getNextStepDeque(existingInsertDeque, stepInsertDeque, stepNo);
			if (stepNo == -1 && stepInsertDeque.empty() == true) {
				//TRACE_FILE_FUNC_LINE_"break by (stepNo == -1 && stepInsertDeque.empty() == true).");
				break;
			}

			//#. 해당 데크 가지고 복사.
			for (int i = 1; i <= GetPcb().MaxBlockCount; i++)//i = 1, 2, .., MaxBlockCount
			{
				const int blockNo = i;//1, 2, .., MaxBlockCount
				for (int j = 0; j < stepInsertDeque.size(); j++) {
					INSERT insert = stepInsertDeque.at(j);
					insert.BlockNo = blockNo;
					insert.Step = setStepNo;
					//insert.index = setInsertIndex;
					(gantry == FRONT_GANTRY) ? SetInsert(insert, setInsertIndex) : SetInsert2(insert, setInsertIndex);
					setInsertIndex++;
				}
				setStepNo++;
			}
		}//while(true)
	}

    //#. SetProduction (TotalInsertCount, UseInsertCount 갱신)
    setInsertIndex--;//마지막에 1 더하기 때문에 그거 빼야지 실제 총 삽입 포인트 개수다.
    PRODUCTION production = this->GetProduction();
    if (gantry == FRONT_GANTRY)
    {
        production.TotalInsertCount = setInsertIndex;//싱글갠트리에서는 실질적으로 TotalInsertCount를 쓴다.
        production.TotalFrontInsertCount = setInsertIndex;
        production.UseFrontInsertCount *= this->GetPcb().MaxBlockCount;
    }
    else//gantry == REAR_GANTRY
    {
        production.TotalRearInsertCount = setInsertIndex;
        production.UseRearInsertCount *= this->GetPcb().MaxBlockCount;
    }
    SetProduction(production);

    TRACE_FILE_FUNC_LINE_"returns.");
    return 0;
}

int CReadJobFile::addAvoidSequence(const std::deque<INSERT>& existingInsertDeque, const std::deque<AVOIDMOTION>& existingAvoidMotionDeque, const long& gantry)
{
    if (gantry == REAR_GANTRY)
    {
        TRACE_FILE_FUNC_LINE_"early return due to (gantry == REAR_GANTRY).");
        return 0;
    }

    TRACE_FILE_FUNC_LINE_"starts.");
    int stepNo = -1;
    int insertIndex = 1;

    if (GetPcb().BlockSequenceMode == BLOCK_SEQ_PATTERN)//패턴일때 한스텝짜리 복사하는걸 기존 전체 삽입설정 가지고 복사하면 돼. 약간의 코드중복..
    {
        for (int k = 1; k <= GetPcb().MaxBlockCount; k++)
        {
            for (int i = 0; i < existingAvoidMotionDeque.size(); i++)
            {
                AVOIDMOTION avoidMotionToCopy = existingAvoidMotionDeque.at(i);
                avoidMotionToCopy.insertNo = insertIndex;
                for (int j = 0; j < MAX_AVOID_COUNT; j++)
                {
                    (gantry == FRONT_GANTRY)
                        ? (void)SetAvoidMotion (avoidMotionToCopy.insertNo, avoidMotionToCopy.Order[j], avoidMotionToCopy.Use[j], Point_XYRZ{ avoidMotionToCopy.pt[j].x, avoidMotionToCopy.pt[j].y, avoidMotionToCopy.pt[j].r, avoidMotionToCopy.Height[j] })
                        : (void)SetAvoidMotion2(avoidMotionToCopy.insertNo, avoidMotionToCopy.Order[j], avoidMotionToCopy.Use[j], Point_XYRZ{ avoidMotionToCopy.pt[j].x, avoidMotionToCopy.pt[j].y, avoidMotionToCopy.pt[j].r, avoidMotionToCopy.Height[j] });
                }
                insertIndex++;
            }
        }
        TRACE_FILE_FUNC_LINE_"returns.");
        return 0;
    }//if (GetPcb().BlockSequenceMode == BLOCK_SEQ_STEP)

    while (true)
    {
        //#. 기존 삽입설정에서 스텝별로 나누기.
        std::deque<INSERT> stepInsertDeque = std::deque<INSERT>();
        stepNo = getNextStepDeque(existingInsertDeque, stepInsertDeque, stepNo);
        if (stepNo == -1 && stepInsertDeque.empty() == true)
        {
            //TRACE_FILE_FUNC_LINE_"break due to (stepNo == -1 && stepInsertDeque.empty() == true).");
            break;
        }

        //#. 스텝별 회피설정 구하기. : GetAvoidMotion 대신에 기존 저장한 데크에서 가져오려면 어떻게해? -> 그냥 .at(index-1) 하면 해결.
        std::deque<AVOIDMOTION> stepAvoidMotionDeque;
        for (int i = 0; i < stepInsertDeque.size(); i++)
        {
            stepAvoidMotionDeque.push_back(existingAvoidMotionDeque.at((size_t)stepInsertDeque.at(i).index - 1));
        }

        //#. 스텝별 삽입설정 + 스텝별 회피설정 가지고 복사. + LocateID [Block00] 붙이기 추가
        for (int k = 1; k <= GetPcb().MaxBlockCount; k++)
        {
            for (int i = 0; i < stepAvoidMotionDeque.size(); i++)
            {
                AVOIDMOTION avoidMotionToCopy = stepAvoidMotionDeque.at(i);
                avoidMotionToCopy.insertNo = insertIndex;
                avoidMotionToCopy.LocateID.AppendFormat(L"[Block%02d]", k);//사실 여기서 하면 안되고 Insert 개수 늘릴때 거기서 한번에 해야되는데 이거 어쩔 수 없어보인다. 서로 맞춰야할듯. 위험하지만..
                for (int j = 0; j < MAX_AVOID_COUNT; j++)
                {
                    //if (j < avoidMotionToCopy.Count)//기존 count 개수만큼만 다시 set. 안그러면 10줄씩 로그가 너무 많아서.. -> Use 가 YES_USE일때만 로그 출력하게 해서 해결.
                    {
                        (gantry == FRONT_GANTRY)
                            ? (void)SetAvoidMotion (avoidMotionToCopy.insertNo, avoidMotionToCopy.Order[j], avoidMotionToCopy.Use[j], Point_XYRZ{ avoidMotionToCopy.pt[j].x, avoidMotionToCopy.pt[j].y, avoidMotionToCopy.pt[j].r, avoidMotionToCopy.Height[j] })
                            : (void)SetAvoidMotion2(avoidMotionToCopy.insertNo, avoidMotionToCopy.Order[j], avoidMotionToCopy.Use[j], Point_XYRZ{ avoidMotionToCopy.pt[j].x, avoidMotionToCopy.pt[j].y, avoidMotionToCopy.pt[j].r, avoidMotionToCopy.Height[j] });
                        //잠재적 문제 의심 : 쓰는 부분만 Set하면 블록이 늘어나면서 앞에 Avoid 안해야되는 부분에 대해서 안하도록 set을 안할경우 하게될 가능성이 보인다.(오동작) -> 가능성이 보이는게 아니라 실제로 그렇게 오동작하게 되어있다. 이거 고쳐야돼 -> 고치는중. -> 해결.
                    }
                }
                insertIndex++;
            }
        }
    }//while(true)

    TRACE_FILE_FUNC_LINE_"returns.");
    return 0;
}

int CReadJobFile::addHeightMeasureSequence(const std::deque<INSERT>& existingInsertDeque, const std::deque<MEASUREHEIGHT>& existingMeasureHeightDeque)
{
    TRACE_FILE_FUNC_LINE_"starts.");

    int orderToSet = 1;//1부터 차례차례 높이검사 정보 Set 하기 위해서. + STEP 방식도 같이 사용.

    /*STEP-PATTERN 서로 바꾸기(정상화)*/if (GetPcb().BlockSequenceMode == BLOCK_SEQ_PATTERN)
    {
        for (int j = 0; j < GetPcb().MaxBlockCount; j++)
        {
            const int valueToAddOnInsertNo = j * (int)existingInsertDeque.size();
            for (int i = 0; i < existingMeasureHeightDeque.size(); i++)
            {
                MEASUREHEIGHT measureHeightToSet = existingMeasureHeightDeque.at(i);
                measureHeightToSet.Order = orderToSet;
                orderToSet++;
                measureHeightToSet.InsertNo += valueToAddOnInsertNo;
                if (measureHeightToSet.GroupNo != 0)
                {
                    //GroupNo 최대가 20이라서 블럭간 완전히 구별되도록 그냥 MEASUREHEIGHT::MAX_GROUP_NO을 더했다.
                    measureHeightToSet.GroupNo += j * MEASUREHEIGHT::MAX_GROUP_NO;
                }
                measureHeightToSet.LocateID.AppendFormat(L"[Block%02d]", j + 1);//높이 검사 이력 볼때 이름 같아서 뭐가뭔지 모르니깐 이렇게 표시해서 일단 구분 가능하도록 한다.
                SetMeasureHeight(measureHeightToSet);//SetMeasureHeight 할때 매개변수의 order 속성이 1 이상이어야 한다. 그럼 그거를 배열에 저장하는데 order-1 자리에 저장한다. 근데 order이 뭔데?
            }
        }
        TRACE_FILE_FUNC_LINE_"returns.");
        return 0;
    }//if (GetPcb().BlockSequenceMode == BLOCK_SEQ_STEP)

    int stepNo = -1;//스텝번호.
    int cumulativeInsertCountIncludingBlockFactor = 0;//다음 스텝의 높이검사 정보 InsertNo Set 할때 앞에 삽입설정 개수랑 블록개수 계산해서 그만큼 띄워야되니까 그거 계산용.
    while (true)
    {
        //#. 한 스텝짜리 INSERT_INFO 데크 구하기.
        std::deque<INSERT> stepInsertInfoDeque = std::deque<INSERT>{};
        stepNo = getNextStepDeque(existingInsertDeque, stepInsertInfoDeque, stepNo);
        if (stepNo == -1 && stepInsertInfoDeque.size() == 0)
        {
            //TRACE_FILE_FUNC_LINE_"break due to (stepNo == -1 && stepInsertInfoDeque.size() == 0).");//정상
            break;
        }

        //#. 한 스텝짜리 HEIGHTMEASURE 데크 구하기.
        std::deque<MEASUREHEIGHT> stepMeasureHeightDeque = std::deque<MEASUREHEIGHT>();
        for (int i = 0; i < existingMeasureHeightDeque.size(); i++)
        {
            const MEASUREHEIGHT& currentMeasureHeight = existingMeasureHeightDeque.at(i);
            const INSERT& currentInsertInfo = existingInsertDeque.at((size_t)currentMeasureHeight.InsertNo - 1);
            if (currentInsertInfo.Step == stepNo)
            {
                stepMeasureHeightDeque.push_back(currentMeasureHeight);
            }
        }

        //#. 한 스텝자리 INSERT_INFO 데크 가지고 복사하기.
        for (int j = 0; j < GetPcb().MaxBlockCount; j++)
        {
            const int valueToAddOnInsertNo = j * (int)stepInsertInfoDeque.size() + cumulativeInsertCountIncludingBlockFactor;
            for (int i = 0; i < stepMeasureHeightDeque.size(); i++)
            {
                MEASUREHEIGHT measureHeightToSet = stepMeasureHeightDeque.at(i);
                measureHeightToSet.Order = orderToSet;
                orderToSet++;
                measureHeightToSet.InsertNo += valueToAddOnInsertNo;
                if (measureHeightToSet.GroupNo != 0)
                {
                    //GroupNo 최대가 20이라서 완전히 구별되도록 그냥 MEASUREHEIGHT::MAX_GROUP_NO을 더했다.
                    measureHeightToSet.GroupNo += j * MEASUREHEIGHT::MAX_GROUP_NO;
                }
                measureHeightToSet.LocateID.AppendFormat(L"[Block%02d]", j + 1);//높이 검사 이력 볼때 이름 같아서 뭐가뭔지 모르니깐 이렇게 표시해서 일단 구분 가능하도록 한다.
                SetMeasureHeight(measureHeightToSet);//SetMeasureHeight 할때 매개변수의 order 속성이 1 이상이어야 한다. 그럼 그거를 배열에 저장하는데 order-1 자리에 저장한다. 근데 order이 뭔데?
            }
        }
        cumulativeInsertCountIncludingBlockFactor += (int)stepInsertInfoDeque.size() * (GetPcb().MaxBlockCount - 1);
    }//while(true)

    TRACE_FILE_FUNC_LINE_"returns.");
    return 0;
}

int CReadJobFile::getNextStepDeque(const std::deque<INSERT>& existingInsertDeque, std::deque<INSERT>& stepInsertDeque, const int& previousStepNo)
{
    stepInsertDeque.clear();
    int stepNoToSave = -1;
    if (existingInsertDeque.size() == 0)
    {
        TRACE_FILE_FUNC_LINE_"early return due to 'existingInsertDeque.size() == 0'.");
        return stepNoToSave;
    }

    if (GetPcb().BlockSequenceMode == BLOCK_SEQ_PATTERN && false)//이래도 되나? -> 이거 별로야.
    {
        if (previousStepNo == -1)//맨처음에 들어왔을때
        {
            stepNoToSave = existingInsertDeque.back().Step;//이거 그렇게 정확한 정보 아니다.. 하지만 BLOCK_SEQ_STEP으로 더블체크 하기때문에 일단 그냥 넘어갈게. 현재 논리상으로 -1 만 아니면 작동되기 때문에.
            stepInsertDeque = existingInsertDeque;
            CString temp;
            temp.Format(L"returning with stepNoToSave : %d, stepInsertDeque.size() : %llu due to 'GetPcb().BlockSequenceMode == BLOCK_SEQ_STEP'.", stepNoToSave, stepInsertDeque.size());
            TRACE_FILE_FUNC_LINE_(CStringA)temp);
            return stepNoToSave;
        }
        else//그 다음에 들어왔을 때 -> 계속하게 하면 안된다.
        {
            return stepNoToSave;//-1 그대로 바로 리턴.
        }
    }

    bool previousStepNoFound = false;
    for (int i = 0; i < existingInsertDeque.size(); i++)
    {
        const INSERT& currentInsert = existingInsertDeque.at(i);
        if (stepNoToSave == -1)//아직 저장할 스텝 못 찾았을 때 만 여기 걸림.
        {
            if (previousStepNo == -1)//처음부터 저장하기로 했으면.
            {
                previousStepNoFound = true;//이전 스텝번호 이미 만났다고 표시하기.
                stepNoToSave = currentInsert.Step;//저장할 스텝번호를 현재(첫) 스텝번호로.
            }
            else if (previousStepNo == currentInsert.Step)//현재 스텝번호가 previousStepNo 와 일치
            {
                previousStepNoFound = true;
                continue;//다시 일치하지 않을때까지 (저장시작할때까지) 스킵.
            }
            else//아직 일치하기전 또는 일치하고 난 후 저장할 차례.
            {
                if (previousStepNoFound == false)//일치하기전이면
                {
                    continue;//일치할때까지 스킵.
                }
                else//(previousStepNoFound == true) 일치하고 난 후 저장할 차례면
                {
                    stepNoToSave = currentInsert.Step;//저장할 스텝번호를 현재 스텝번호로.
                }
            }
        }

        if (stepNoToSave == currentInsert.Step)//저장할 스텝번호면?
        {
            stepInsertDeque.push_back(existingInsertDeque.at(i));//저장.
        }
        else//여기는 언제걸림? 걸릴일 없을듯? 저장 다하고 다른 스텝 만나서 빠져나올 때.
        {
            CString temp;
            temp.Format(L"returning with stepNoToSave : %d, previousStepNo : %d, stepInsertDeque.size() : %llu, (stopped due to currentInsert.Step : %d)", stepNoToSave, previousStepNo, stepInsertDeque.size(), currentInsert.Step);
            TRACE_FILE_FUNC_LINE_(CStringA)temp);
            break;//break 이지만 바로 리턴.
        }
        if (i == existingInsertDeque.size() - 1)//그냥 다 읽어서 빠졌을 때도 TRACE 해야된다.
        {
            CString temp;
            temp.Format(L"returning with stepNoToSave : %d, previousStepNo : %d, stepInsertDeque.size() : %llu, (stopped due to encountering end of deque)", stepNoToSave, previousStepNo, stepInsertDeque.size());
            TRACE_FILE_FUNC_LINE_(CStringA)temp);
            break;//break 안해도 빠져나와진다.
        }
    }//for(i)
    if (stepNoToSave == -1)
    {
        CString temp;
        temp.Format(L"returning with stepNoToSave : %d, previousStepNo : %d, stepInsertDeque.size() : %llu", stepNoToSave, previousStepNo, stepInsertDeque.size());
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
    }
    return stepNoToSave;
}

void CReadJobFile::SetInsert2(const INSERT& insert, const long& insertNo)
{
    TRACE_FILE_FUNC_LINE_"bad request");
    return;
}

AVOIDMOTION CReadJobFile::GetAvoidMotion(const long& gantry, const long& insertNo)
{
    if (gantry == FRONT_GANTRY)
    {
        return this->GetAvoidMotion(insertNo);
    }
    TRACE_FILE_FUNC_LINE_"bad request");
    return AVOIDMOTION{};
}

void CReadJobFile::SetAvoidMotion2(const long& InsertNo, const long& Order, const long& Use, const Point_XYRZ& xyrz)
{
    TRACE_FILE_FUNC_LINE_"bad request");
    return;
}

void CReadJobFile::MakeFileName()
{
	m_StrFileName.Format(_T("%s\\%s.%s"), (LPCTSTR)m_StrDir, (LPCTSTR)m_StrHeader, (LPCTSTR)m_StrExtension);
}

CString CReadJobFile::GetFileName()
{
	return m_StrFileName;
}

void CReadJobFile::SetDir(CString Dir)
{
	m_StrDir = Dir;
}

void CReadJobFile::SetHeader(CString Header)
{
	m_StrHeader = Header;
}

void CReadJobFile::SetExtension(CString Ext)
{
	m_StrExtension = Ext;
}

void CReadJobFile::Initial()
{
	ZeroMemory(&m_Production, sizeof(m_Production));
	InitPcb();
	ZeroMemory(&m_Origin, sizeof(m_Origin));
	ZeroMemory(&m_BlockOrigin, sizeof(m_BlockOrigin[MAXBLOCKNO]));
	ZeroMemory(&m_Mark, sizeof(m_Mark));
	ZeroMemory(&m_BlockMark, sizeof(m_BlockMark[MAXBLOCKNO]));
	ZeroMemory(&m_Discard, sizeof(m_Discard[MAXFEEDERNO]));
	for (long index = 0; index < MAXFEEDERNO; ++index)
	{
		InitPick(index);
		InitPackage(index);
		InitFeeder(index);
		InitRetryLed(index);
		InitForming(index);
	}
	for (long NzlCount = 0; NzlCount < MAXNOZZLENO; ++NzlCount)
	{
		m_Nozzle[NzlCount].No = 0;
		m_Nozzle[NzlCount].Type = 0;
		m_Nozzle[NzlCount].TipHeight = 0.0;
		m_Nozzle[NzlCount].PusherHeight = 0.0;
		m_Nozzle[NzlCount].Empty = 0;
		m_Nozzle[NzlCount].EmptyDiff = 0;
		m_Nozzle[NzlCount].Exist = 0;
		m_Nozzle[NzlCount].ExistDiff = 0;
	}
	for (long InsertNo = 0; InsertNo < MAXINSERTNO; ++InsertNo)
	{
		InitInsert(InsertNo);
		InitMeasureHeight(InsertNo);
		InitAvoidMotion(InsertNo);;
	}
	m_PickupZStandby = 0;
	m_PartDrop.Use = false;
	m_PartDrop.led.Top = m_PartDrop.led.Mid = m_PartDrop.led.Bot = 0;
}

void CReadJobFile::InitPcb()
{
	m_Pcb.Name.Format(_T("PwrPcb"));
	m_Pcb.Length = 200.0;
	m_Pcb.Width = 200.0;
	m_Pcb.Thickness = 1.0;						// PCB 고정단의 고정 위치 + Thickness
	m_Pcb.MaxComponentHeight = 10.0;			// Max Component Height
	m_Pcb.StandByPusherZOffsetHeight = 10.0;	// PCB 대기 중에 정지 위치 + InsertPos
	m_Pcb.UseBlockType = 0;						// 0:Single 1:Matrix 2:Non-Matrix
	m_Pcb.UseFiducial = 0;						// 0:No Use 1:Single 2:Block
	m_Pcb.MaxBlockCount = 0;					// Max Block Count
	m_Pcb.UseBlockCount = 0;					// Use Block Count
	m_Pcb.MaxPcbLoadingCount = 0;				// 0:Three 1:Only One
	m_Pcb.BlockSequenceMode = 0;				// 0:Step 1:Pattern 2:Overall
	m_Pcb.UseHeightMeasurement = 0;				// 0:No Use 1:Use
	m_Pcb.UseSimultaneousLoading = 0;			// 0:No Use 1:Use
	m_Pcb.SimulLoadType = 0;
}

void CReadJobFile::InitPick(long index)
{
	m_Pick[index].FeederNo = 0;
	m_Pick[index].NextFeedNo = 0;
	m_Pick[index].Use = 0;
	m_Pick[index].UseSimultaneous = 0;
	m_Pick[index].Offset.x = m_Pick[index].Offset.y = m_Pick[index].Offset.r = m_Pick[index].Offset.z = 0.0;
}

void CReadJobFile::InitPackage(long index)
{
	m_Package[index].Name.Format(_T("PwrPackage%03d"), index + 1);
	m_Package[index].Length = 10.0;
	m_Package[index].Width = 10.0;
	m_Package[index].Height = 10.0;
	ZeroMemory(&m_Package[index].Led, sizeof(m_Package[index].Led));
	m_Package[index].PickDelay = TIME100MS;
	m_Package[index].BlowDelay = TIME100MS;
	m_Package[index].ReleaseDelay = TIME100MS;
	m_Package[index].Ratio.xy = m_Package[index].Ratio.r = m_Package[index].Ratio.z = 0;
	m_Package[index].PickRetry = DEFAULT_PICKRETRY;
	m_Package[index].TwoStepPick.Use = 0;
	m_Package[index].TwoStepPick.Dist = 1.0;
	m_Package[index].TwoStepPick.Ratio = 1.0;
	m_Package[index].TwoStepInsert.Use = 0;
	m_Package[index].TwoStepInsert.Dist = 1.0;
	m_Package[index].TwoStepInsert.Ratio = 1.0;
	m_Package[index].RecogAngle = 90.0;
	m_Package[index].PartEmptyStop = 0;
	m_Package[index].LaserControl = 0;
	m_Package[index].CatchDelay = 0;
	m_Package[index].TwoStepInsertUp.Use = 0;
	m_Package[index].TwoStepInsertUp.Dist = 1.0;
	m_Package[index].TwoStepInsertUp.Ratio = 1.0;
	m_Package[index].PartTorqueLimit.PickDown.Use = false;
	m_Package[index].PartTorqueLimit.PickDown.TorqueLimit = 0.0;
	m_Package[index].PartTorqueLimit.PickDown2nd.Use = false;
	m_Package[index].PartTorqueLimit.PickDown2nd.TorqueLimit = 0.0;
	m_Package[index].PartTorqueLimit.InsertDown.Use = false;
	m_Package[index].PartTorqueLimit.InsertDown.TorqueLimit = 0.0;
	m_Package[index].PartTorqueLimit.InsertDown2nd.Use = false;
	m_Package[index].PartTorqueLimit.InsertDown2nd.TorqueLimit = 0.0;
}

void CReadJobFile::InitFeeder(long index)
{
	m_Feeder[index].PackageName.Format(_T("PwrPackage%03d"), index + 1);
	m_Feeder[index].No = 0;
	m_Feeder[index].Type = 0;
	m_Feeder[index].TimeOut = TIME5000MS;
	m_Feeder[index].ReadyIONo = 0;
	m_Feeder[index].ReleaseIONo = 0;
	m_Feeder[index].ReadyIOWaitTime = TIME500MS;
}

void CReadJobFile::InitRetryLed(long index)
{
	m_RetryLed[index].PackageName.Format(_T("PwrPackage%03d"), index + 1);
	m_RetryLed[index].MaxRetry = 0;
	for (long LedCount = 0; LedCount < MAXRETRYCNT_5; ++LedCount)
	{
		ZeroMemory(&m_RetryLed[index].Led[LedCount], sizeof(m_RetryLed[index].Led[LedCount]));
	}
}

void CReadJobFile::InitInsert(long insertNo)
{
	m_Insert[insertNo].index = 0;
	m_Insert[insertNo].Step = 0;
	m_Insert[insertNo].Use = 0;
	m_Insert[insertNo].FeederNo = 0;
	m_Insert[insertNo].PickOrder = 0;
	m_Insert[insertNo].InsertOrder = 0;
	m_Insert[insertNo].HeadNo = 0;
	m_Insert[insertNo].pt.x = m_Insert[insertNo].pt.y = m_Insert[insertNo].pt.r = m_Insert[insertNo].pt.z = 0.0;
	m_Insert[insertNo].BlockNo = 0;
	m_Insert[insertNo].NozzleNo = 0;
	m_Insert[insertNo].RecogTable = FRONT_GANTRY;

}

void CReadJobFile::InitMeasureHeight(long order)
{
	m_MeasureHeight[order].Use = 0;
	m_MeasureHeight[order].LocateID = "";
	m_MeasureHeight[order].Order = 0;
	m_MeasureHeight[order].pt.x = 0.0;
	m_MeasureHeight[order].pt.y = 0.0;
	m_MeasureHeight[order].pt.t = 0.0;
	m_MeasureHeight[order].Tolerance = 0.0;
	m_MeasureHeight[order].InsertNo = 0;
	m_MeasureHeight[order].GroupNo = 0;
	m_MeasureHeight[order].GantryNo = 0;
}

long CReadJobFile::ReadFile()
{
	long Err = NO_ERR;
	CApplicationTime* pTime = new CApplicationTime();
	CStdioFile* cFile = new CStdioFile();
	CFileException ex;
	Initial();
	if (cFile->Open((LPCTSTR)GetFileName(), CFile::modeRead
		| CFile::modeNoInherit
		//| CFile::shareExclusive, 
		| CFile::shareDenyNone,
		NULL) == false)
	{
		TRACE(_T("[PWR] ReadJobFile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
		delete pTime;
		delete cFile;
		return JOBFILE_OPEN_FAIL;
	}
	TRACE(_T("[PWR] ReadJobFile(%s) Open Success\n"), (LPCTSTR)GetFileName());

	CStringArray* AllData = new CStringArray();
	CString str;

	while (1)
	{
		if (cFile->ReadString(str) == false)
		{
			break;
		}
		else
		{
			AllData->Add(str);
		}
	}

	cFile->Close();
	delete cFile;

	TRACE(_T("[PWR] ReadJobFile(%s) Read all line(%d) Success\n"), (LPCTSTR)GetFileName(), AllData->GetCount());

	Err = Read_PRODUCTION_INFO(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_PCB(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_ORIGIN(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_FIDUCIAL(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_PICK(AllData);
	if (Err != NO_ERR)
	{

		delete AllData;
		return Err;
	}

	Err = Read_PACKAGE(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_INSERT(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}
	
	Err = Read_FEEDER(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_LED_VALUE(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_NOZZLE(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_DISCARD(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_STANDBY(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_PICKUPZSTANDBY(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_BARCODE(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_MEASUREHEIGHT(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_TRAY(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_AVOIDMOTION(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_PartDrop(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_DivideInspect(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_BlockSkipHM(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = Read_FORMING(AllData);
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}

	Err = CheckJobFileValid();
	if (Err != NO_ERR)
	{
		delete AllData;
		return Err;
	}
	TRACE(_T("[PWR] ReadFile(%s) Complete Elapsed:%d[ms]\n"), (LPCTSTR)GetFileName(), pTime->TimeElapsed());

	delete AllData;
	delete pTime;

	return Err;
}

//long CReadJobFile::ReadFile_org()
//{
//	long Err = NO_ERR;
//	CApplicationTime* pTime = new CApplicationTime();
//	CStdioFile* cFile = new CStdioFile();
//	CFileException ex;
//	Initial();
//	if (cFile->Open((LPCTSTR)GetFileName(), CFile::modeRead
//		| CFile::modeNoInherit
//		//| CFile::shareExclusive, 
//		| CFile::shareDenyNone,
//		NULL) == false)
//	{
//		TRACE(_T("[PWR] ReadJobFile(%s) Open Failed\n"), (LPCTSTR)GetFileName());
//		SendAlarm(JOBFILE_OPEN_FAIL, _T("ReadFile Open Fail"));
//		delete pTime;
//		delete cFile;
//		return JOBFILE_OPEN_FAIL;
//	}
//	TRACE(_T("[PWR] ReadJobFile(%s) Open Success\n"), (LPCTSTR)GetFileName());
//	Err = Read_PRODUCTION_INFO(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_PCB(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_ORIGIN(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_FIDUCIAL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_PICK(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_PACKAGE(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_INSERT(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_FEEDER(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_LED_VALUE(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NOZZLE(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_DISCARD(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_STANDBY(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	Err = Read_PICKUPZSTANDBY(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}	
//
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	
//	Err = Read_BARCODE(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{ 
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = Read_MEASUREHEIGHT(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = Read_TRAY(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = Read_NULL(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = Read_AVOIDMOTION(cFile);
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//
//	Err = CheckJobFileValid();
//	if (Err != NO_ERR)
//	{
//		cFile->Close();
//		delete cFile;
//		return Err;
//	}
//	TRACE(_T("[PWR] ReadFile(%s) Complete Elapsed:%d[ms]\n"), (LPCTSTR)GetFileName(), pTime->TimeElapsed());
//	cFile->Close();
//	delete cFile;
//	return Err;
//}

ORIGIN CReadJobFile::ParseOriginFromStr(CString str)
{
	ORIGIN Origin;
	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile ParseOriginFromStr TokenCount:%d\n", cTokenizer->GetCount());
	}
	CString strValue;
	double dValue[20];
	ZeroMemory(&dValue, sizeof(dValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		dValue[i] = cTokenizer->GetDouble(i);
	}
	Origin.pt.x = dValue[0];
	Origin.pt.y = dValue[1];
	Origin.pt.r = 0.0;
	Origin.Use = 1;
	delete cTokenizer;
	return Origin;
}

ORIGIN CReadJobFile::ParseBlockOriginFromStr(CString str)
{
	ORIGIN Origin;
	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile ParseBlockOriginFromStr TokenCount:%d\n", cTokenizer->GetCount());
	}
	long iCnt = 0, dCnt = 0;
	CString strValue;
	int iValue[20];
	double dValue[20];
	ZeroMemory(&iValue, sizeof(iValue));
	ZeroMemory(&dValue, sizeof(dValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (strValue.Find(_T(".")) >= 0)
		{
			dValue[dCnt] = cTokenizer->GetDouble(i);
			dCnt++;
		}
		else
		{
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
	}
	Origin.Use = iValue[0];
	Origin.pt.x = dValue[0];
	Origin.pt.y = dValue[1];
	Origin.pt.r = dValue[2];
	delete cTokenizer;
	return Origin;
}

FIDUCIAL CReadJobFile::ParseFiducialFromStr(CString str)
{
	FIDUCIAL Fiducial;
	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile ParseFiducialFromStr TokenCount:%d\n", cTokenizer->GetCount());
	}
	long iCnt = 0, dCnt = 0;
	CString strValue;
	int iValue[20];
	double dValue[20];
	ZeroMemory(&iValue, sizeof(iValue));
	ZeroMemory(&dValue, sizeof(dValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (strValue.Find(_T(".")) >= 0)
		{
			dValue[dCnt] = cTokenizer->GetDouble(i);
			dCnt++;
		}
		else
		{
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
	}
	Fiducial.MarkNo[0] = iValue[0] + 1;
	Fiducial.pt[0].x = dValue[0];
	Fiducial.pt[0].y = dValue[1];
	Fiducial.Led[0].Red = iValue[1];
	Fiducial.Led[0].Blue = iValue[2];

	Fiducial.MarkNo[1] = iValue[3] + 1;
	Fiducial.pt[1].x = dValue[2];
	Fiducial.pt[1].y = dValue[3];
	Fiducial.Led[1].Red = iValue[4];
	Fiducial.Led[1].Blue = iValue[5];
	delete cTokenizer;
	return Fiducial;
}
//
//long CReadJobFile::Read_NULL(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			//
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			//
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_PRODUCTION_INFO(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [PRODUCTION_INFO]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			// i6.0_v1.0,0,0,0,0,0,1,1,0,0
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE("[PWR] CReadJobFile Read_PRODUCTION_INFO TokenCount:%d\n", cTokenizer->GetCount());
//		}
//		long Gantry = FRONT_GANTRY;
//		CString strValue;
//		int iValue[20];
//		ZeroMemory(&iValue, sizeof(iValue));
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			iValue[i] = cTokenizer->GetInt(i);
//		}
//		PRODUCTION Prod;
//		Prod.JobFileVersion = iValue[0];
//		Prod.PlanQuantity = iValue[1];
//		Prod.ProdQuantity = iValue[2];
//		Prod.ProdMode = iValue[3];
//		Prod.BoardLocation = iValue[4];
//		Prod.FirstPickingTiming = iValue[5];
//		Prod.TotalFeederCount = iValue[6];
//		Prod.UseFeederCount = iValue[7];
//		Prod.TotalInsertCount = iValue[8];
//		Prod.UseInsertCount = iValue[9];
//		SetProduction(Prod);
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//	bRet = cFile->ReadString(str);			// [EOF_PRODUCTION_INFO]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_PCB(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [PCB]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			// i6.0_v1.0,0,0,0,0,0,1,1,0,0
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE("[PWR] CReadJobFile Read_PCB TokenCount:%d\n", cTokenizer->GetCount());
//		}
//		long iCnt = 0, dCnt = 0;
//		CString strValue;
//		int iValue[20];
//		double dValue[20];
//		PCB Pcb;
//		ZeroMemory(&iValue, sizeof(iValue));
//		ZeroMemory(&dValue, sizeof(dValue));
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			if (i == 0)
//			{
//				Pcb.Name = strValue;
//			}
//			else
//			{
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//		}		
//		Pcb.Length = dValue[0];
//		Pcb.Width = dValue[1];
//		Pcb.Thickness = dValue[2];
//
//		double MaxCompSafe;
//		if (gCMachineConfig->IsANCUpType(FRONT_STAGE) == true && gCMachineConfig->IsANCUpType(REAR_STAGE) == true)
//		{
//			MaxCompSafe = SAFTY_MAXCOMPONENTHEIGHT_REV1;
//		}
//		else
//		{
//			MaxCompSafe = SAFTY_MAXCOMPONENTHEIGHT;
//		}
//
//		if (dValue[3] < MaxCompSafe)
//		{
//			Pcb.MaxComponentHeight = MaxCompSafe;
//			TRACE(_T("[PWR] User Input MaxComponentHeight:%.3f Change to %.3f\n"), dValue[3], MaxCompSafe);
//		}
//		else
//		{
//			Pcb.MaxComponentHeight = dValue[3];
//		}
//		Pcb.StandByPusherZOffsetHeight = dValue[4];
//		Pcb.UseBlockType = iValue[0];
//		Pcb.UseFiducial = iValue[1];
//		Pcb.MaxBlockCount = iValue[2];
//		Pcb.UseBlockCount = iValue[3];
//		Pcb.MaxPcbLoadingCount = iValue[4];
//		Pcb.BlockSequenceMode = iValue[5];
//		Pcb.UseHeightMeasurement = iValue[6];
//		Pcb.UseSimultaneousLoading = iValue[7];
//		Pcb.PusherRatioUp = iValue[8];
//		Pcb.PusherRatioDown = iValue[9];
//		Pcb.SimulLoadType = iValue[10];
//		SetPcb(Pcb);
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//	bRet = cFile->ReadString(str);			// [EOF_PCB]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_ORIGIN(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [ORIGIN]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			// 
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		ORIGIN Origin = ParseOriginFromStr(str);
//		SetOrigin(Origin);
//		PCB Pcb = GetPcb();
//		for (long blk = 0; blk < Pcb.MaxBlockCount; ++blk)
//		{
//			bRet = cFile->ReadString(str);
//			if (bRet == false)
//			{
//				return JOBFILE_READSTRING_FAIL;
//			}
//			else
//			{
//				Origin = ParseBlockOriginFromStr(str);
//				SetBlockOrigin(Origin, Origin.Use);
//			}
//		}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_ORIGIN]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_FIDUCIAL(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [FIDUCIAL]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			// 0,10.000,13.000,9,10,1,165.000,223.000,9,10
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		FIDUCIAL Fiducial = ParseFiducialFromStr(str);
//		PCB Pcb = GetPcb();
//		if(Pcb.UseFiducial > 0)
//		{ 
//			Fiducial.Use[0] = Fiducial.Use[1] = 1;
//		}
//		else
//		{
//			Fiducial.Use[0] = Fiducial.Use[1] = 0;
//		}
//		SetMark(Fiducial);
//		//for (long blk = 0; blk < Pcb.MaxBlockCount; ++blk)
//		//{
//		//	bRet = cFile->ReadString(str);	// 0,10.000,13.000,9,10,1,165.000,223.000,9,10
//		//	if (bRet == false)
//		//	{
//		//		return JOBFILE_READSTRING_FAIL;
//		//	}
//		//	else
//		//	{
//		//		Fiducial = ParseFiducialFromStr(str);
//		//		if (Pcb.UseFiducial > 0)
//		//		{
//		//			Fiducial.Use[0] = Fiducial.Use[1] = blk + 1;
//		//		}
//		//		else
//		//		{
//		//			Fiducial.Use[0] = Fiducial.Use[1] = 0;
//		//		}
//		//		SetBlockMark(Fiducial, blk + 1);
//		//	}
//		//}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_FIDUCIAL]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_PICK(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [PICK]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	PICK Pick;
//	for (long PickCount = 0; PickCount < Prod.TotalFeederCount; ++PickCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_PICK(%d) TokenCount:%d\n", PickCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			ZeroMemory(&Pick, sizeof(Pick));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//			Pick.FeederNo = iValue[0];
//			Pick.NextFeedNo = iValue[1];
//			Pick.Use = iValue[2];
//			Pick.UseSimultaneous = iValue[3];
//			Pick.Offset.x = dValue[0];
//			Pick.Offset.y = dValue[1];
//			Pick.Offset.r = dValue[2];
//			Pick.Offset.z = dValue[3];
//			SetPick(Pick, Pick.FeederNo);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_PICK]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_PACKAGE(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [PACKAGE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	PACKAGE Package;
//	for (long PackageCount = 0; PackageCount < MAXPACKAGENO; ++PackageCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//		if (str.CompareNoCase(_T("[EOF_PACKAGE]")) == 0) // [EOF_PACKAGE]
//		{
//			break;
//		}
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_PACKAGE(%d) TokenCount:%d\n", PackageCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (i == 0)
//				{
//					Package.Name.Format(_T("%s"), (LPCTSTR)strValue);
//				}
//				else
//				{
//					if (strValue.Find(_T(".")) >= 0)
//					{
//						dValue[dCnt] = cTokenizer->GetDouble(i);
//						dCnt++;
//					}
//					else
//					{
//						iValue[iCnt] = cTokenizer->GetInt(i);
//						iCnt++;
//					}
//				}
//			}
//			Package.Length = dValue[0];
//			Package.Width = dValue[1];
//			Package.Height = dValue[2];
//			Package.LeadHeight = dValue[3];
//			Package.Led.Top = iValue[0];
//			Package.Led.Mid = iValue[1];
//			Package.Led.Bot = iValue[2];
//			Package.PickDelay = iValue[3];
//			Package.BlowDelay = iValue[4];
//			Package.ReleaseDelay = iValue[5];
//			Package.Ratio.xy = (double)(iValue[6] / 100.0);
//			Package.Ratio.r = (double)(iValue[7] / 100.0);
//			Package.Ratio.z = (double)(iValue[8] / 100.0);
//			Package.PickRetry = iValue[9];
//			Package.TwoStepPick.Use = iValue[10];
//			Package.TwoStepPick.Dist = dValue[4];
//			Package.TwoStepPick.Ratio = (double)(iValue[11] / 100.0);
//			Package.TwoStepInsert.Use = iValue[12];
//			Package.TwoStepInsert.Dist = dValue[5];
//			Package.TwoStepInsert.Ratio = (double)(iValue[13] / 100.0);
//			Package.InsertZOffset = dValue[6];
//			Package.RecogAngle = dValue[7];
//			Package.RecogOffsetHeight = dValue[8];
//			Package.PartEmptyStop = iValue[14];
//			Package.RotationHeightOffset_Use = iValue[15];
//			Package.RotationHeightOffset_ZOffset = dValue[9];
//			Package.LaserControl = iValue[16];
//			Package.CatchDelay = iValue[17];
//			SetPackage(Package, PackageCount + 1);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	//bRet = cFile->ReadString(str);			// [EOF_PACKAGE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_INSERT(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [INSERT]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[10];
//	double dValue[10];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	INSERT Insert;
//	for (long InsertCount = 0; InsertCount < Prod.TotalInsertCount; ++InsertCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_INSERT(%d) TokenCount:%d\n", InsertCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			ZeroMemory(&Insert, sizeof(Insert));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//			Insert.index = iValue[0];
//			Insert.Step = iValue[1];
//			Insert.Use = iValue[2];
//			Insert.FeederNo = iValue[3];
//			Insert.PickOrder = iValue[4];
//			Insert.InsertOrder = iValue[5];
//			Insert.HeadNo = iValue[6];
//			Insert.pt.x = dValue[0];
//			Insert.pt.y = dValue[1];
//			Insert.pt.r = dValue[2];
//			Insert.pt.z = dValue[3];
//			Insert.BlockNo = iValue[7];
//			Insert.NozzleNo = iValue[8];
//			Insert.RecogTable = iValue[9];
//			Err = SetInsert(Insert, InsertCount + 1);
//			delete cTokenizer;
//			cTokenizer = NULL;
//			if (Err != NO_ERR)
//			{
//				return Err;
//			}
//		}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_INSERT]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_MEASUREHEIGHT(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	int iValue[10];
//	double dValue[10];
//	CString strValue;
//	long iCnt = 0, dCnt = 0;
//
//	bRet = cFile->ReadString(str);			// []
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//
//	if (str.CompareNoCase(_T("[EOF_HEIGHTMEASUREMENT]")) == 0)
//	{
//		return NO_ERR;
//	}
//
//	//list<MEASUREHEIGHT> data;
//	MEASUREHEIGHT temp;
//
//	for (long inspectCount = 0; inspectCount < MAXINSERTNO; inspectCount++)
//	{
//		bRet = cFile->ReadString(str);
//
//		if (str.CompareNoCase(_T("[EOF_HEIGHTMEASUREMENT]")) == 0)
//		{
//			return NO_ERR;
//		}
//
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE("[PWR] CReadJobFile Read_MEASUREHEIGHT(%d) TokenCount:%d\n", inspectCount, cTokenizer->GetCount());
//		}
//
//		ZeroMemory(&iValue, sizeof(iValue));
//		ZeroMemory(&dValue, sizeof(dValue));
//		iCnt = dCnt = 0;
//
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			if (i == 1)
//			{
//				temp.LocateID.Format(_T("%s"), (LPCTSTR)strValue);
//			}
//			else
//			{
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//		}
//
//		temp.Order = iValue[0];
//		temp.pt.x = dValue[0];
//		temp.pt.y = dValue[1];
//		temp.pt.t = dValue[2];
//		temp.Tolerance = dValue[3];
//		temp.Use = iValue[1];
//		//data.push_back(temp);
//
//		SetMeasureHeight(temp);
//
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//
//	return Err;
//}
//
//long CReadJobFile::Read_FEEDER(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [FEEDER]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	FEEDER Feeder;
//	for (long FeederCount = 0; FeederCount < Prod.TotalFeederCount; ++FeederCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_FEEDER(%d) TokenCount:%d\n", FeederCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//		//	InitFeeder(FeederCount);
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (i == 0)
//				{
//					Feeder.PackageName = strValue;
//				}
//				else
//				{
//					if (strValue.Find(_T(".")) >= 0)
//					{
//						dValue[dCnt] = cTokenizer->GetDouble(i);
//						dCnt++;
//					}
//					else
//					{
//						iValue[iCnt] = cTokenizer->GetInt(i);
//						iCnt++;
//					}
//				}
//			}
//			Feeder.No = iValue[0];
//			Feeder.Type = iValue[1];
//			Feeder.TimeOut = iValue[2];
//			Feeder.ReadyIONo = iValue[3];
//			Feeder.ReleaseIONo = iValue[4];
//			Feeder.ReadyIOWaitTime = iValue[5];
//
//			if (Feeder.Type == 1) // Tray
//			{
//				if (Feeder.ReadyIONo != 0 || Feeder.ReleaseIONo != 0)
//				{
//					TRACE(_T("[PWR] Tray Feeder No(%d) Ready IO Clear(%d,%d) \n"), Feeder.No, Feeder.ReadyIONo, Feeder.ReleaseIONo);
//					Feeder.ReadyIONo = Feeder.ReleaseIONo = 0;
//				}
//			}
//
//			SetFeeder(Feeder, Feeder.No);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_FEEDER]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_LED_VALUE(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [LED_VALUE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	long iCnt = 0;
//	CString strValue;
//	RETRY_LED RetryLed;
//	for (long RetryCount = 0; RetryCount < MAXPACKAGENO; ++RetryCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,1,80,80,80
//		if (str.CompareNoCase(_T("[EOF_LED_VALUE]")) == 0) // [EOF_LED_VALUE]
//		{
//			break;
//		}
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_LED_VALUE(%d) TokenCount:%d\n", RetryCount, cTokenizer->GetCount());
//			}
//			iCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (i == 0)
//				{
//					RetryLed.PackageName = strValue;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//			RetryLed.MaxRetry = iValue[0];
//			for (long Count = 0; Count < MAXRETRYCNT_5; ++Count)
//			{
//				RetryLed.Led[Count].Top = RetryLed.Led[Count].Mid = RetryLed.Led[Count].Bot = 0;
//			}
//			for (long Count = 0; Count < RetryLed.MaxRetry; ++Count)
//			{
//				RetryLed.Led[Count].Top = iValue[(Count * 3) + 1]; // 1, 4, 7, 10, 13
//				RetryLed.Led[Count].Mid = iValue[(Count * 3) + 2]; // 2, 5, 8, 11, 14
//				RetryLed.Led[Count].Bot = iValue[(Count * 3) + 3]; // 3, 6, 9, 12, 15
//			}
//			
//			SetRetryLed(RetryLed, RetryCount + 1);
//
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	//bRet = cFile->ReadString(str);			// [EOF_LED_VALUE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_NOZZLE(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [NOZZLE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0;
//	CString strValue;
//	NOZZLE Nozzle;
//	for (long NozzleCount = 0; NozzleCount < MAXNOZZLENO; ++NozzleCount)
//	{
//		bRet = cFile->ReadString(str);			// TEST NOZZLE,3.000,7.000
//		if (str.CompareNoCase(_T("[EOF_NOZZLE]")) == 0) // [EOF_NOZZLE]
//		{
//			break;
//		}
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_NOZZLE(%d) TokenCount:%d\n", NozzleCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&Nozzle, sizeof(Nozzle));
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//			Nozzle.No = iValue[0];
//			Nozzle.Type = iValue[1];
//			Nozzle.TipHeight = dValue[0];
//			Nozzle.PusherHeight = dValue[1];
//			Nozzle.Empty = iValue[2];
//			Nozzle.EmptyDiff = iValue[3];
//			Nozzle.Exist = iValue[4];
//			Nozzle.EmptyDiff = iValue[5];
//			SetNozzle(Nozzle, Nozzle.No);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	//bRet = cFile->ReadString(str);			// [EOF_LED_VALUE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_DISCARD(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [DISCARD]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	DISCARD Discard;
//	for (long DiscardCount = 0; DiscardCount < Prod.TotalFeederCount; ++DiscardCount)
//	{
//		bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_DISCARD(%d) TokenCount:%d\n", DiscardCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			ZeroMemory(&Discard, sizeof(Discard));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//			Discard.FeederNo = iValue[0];
//			Discard.Mode = iValue[1];
//			Discard.pt.x = dValue[0];
//			Discard.pt.y = dValue[1];
//			Discard.pt.r = dValue[2];
//			Discard.pt.z = dValue[3];
//			SetDiscard(Discard, Discard.FeederNo);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	bRet = cFile->ReadString(str);			// [EOF_DISCARD]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_BARCODE(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [BARCODE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	bRet = cFile->ReadString(str);			// [EOF_BARCODE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//
//	return Err;
//}
//
//long CReadJobFile::Read_STANDBY(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [STANDBY]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0;
//	CString strValue;
//	STANDBY StandBy;
//	bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE(_T("[PWR] CReadJobFile Read_STANDBY TokenCount:%d\n"), cTokenizer->GetCount());
//		}
//		iCnt = dCnt = 0;
//		ZeroMemory(&iValue, sizeof(iValue));
//		ZeroMemory(&dValue, sizeof(dValue));
//		ZeroMemory(&StandBy, sizeof(StandBy));
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			if (strValue.Find(_T(".")) >= 0)
//			{
//				dValue[dCnt] = cTokenizer->GetDouble(i);
//				dCnt++;
//			}
//			else
//			{
//				iValue[iCnt] = cTokenizer->GetInt(i);
//				iCnt++;
//			}
//		}
//		StandBy.pt.x = dValue[0];
//		StandBy.pt.y = dValue[1];
//		StandBy.pt.r = dValue[2];
//		StandBy.pt.z = dValue[3];
//		if (StandBy.pt.z > SAFTY_ZHEIGHT)
//		{
//			TRACE(_T("[PWR] INVALID StandBy Z,%.3f\n"), StandBy.pt.z);
//			Err = INVALID_STANDBYZ;
//		}
//		else
//		{
//			SetStandyBy(StandBy);
//		}		
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//	bRet = cFile->ReadString(str);			// [EOF_STANDBY]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_PICKUPZSTANDBY(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [PICKUPZSTANDBY]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	int iValue[20];
//	double dValue[20];
//	long iCnt = 0, dCnt = 0;
//	double PickupZStandBy = 70.0;
//	CString strValue;	
//	bRet = cFile->ReadString(str);			// 30,0,1,0,0.000,0.000,0.000,0.000,100,100,100,0
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	else
//	{
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE(_T("[PWR] CReadJobFile Read_PICKUPZSTANDBY TokenCount:%d\n"), cTokenizer->GetCount());
//		}
//		iCnt = dCnt = 0;
//		ZeroMemory(&iValue, sizeof(iValue));
//		ZeroMemory(&dValue, sizeof(dValue));
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			if (strValue.Find(_T(".")) >= 0)
//			{
//				dValue[dCnt] = cTokenizer->GetDouble(i);
//				dCnt++;
//			}
//			else
//			{
//				iValue[iCnt] = cTokenizer->GetInt(i);
//				iCnt++;
//			}
//		}
//		PickupZStandBy = dValue[0];
//		if (PickupZStandBy > SAFTY_ZHEIGHT)
//		{
//			TRACE(_T("[PWR] INVALID Pickup Z StandBy,%.3f\n"), PickupZStandBy);
//			Err = INVALID_PICKUPZSTANDBY;
//		}
//		else
//		{
//			SetPickupZStandBy(PickupZStandBy);
//		}
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//	if (Err != NO_ERR)
//	{
//		return Err;
//	}
//	bRet = cFile->ReadString(str);			// [EOF_PICKUPZSTANDBY]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_TRAY(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	bRet = cFile->ReadString(str);			// [TRAY]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	PRODUCTION Prod = GetProduction();
//	int iValue[20];
//	double dValue[1500];
//	long iCnt = 0, dCnt = 0, FeederNo = 0;
//	CString strValue;
//	TRAY_INFO Tray;
//	for (long TrayCount = 0; TrayCount < MAXTRAYNO; ++TrayCount)
//	{
//		bRet = cFile->ReadString(str);
//		if (str.CompareNoCase(_T("[EOF_TRAY]")) == 0)
//		{
//			break;
//		}
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//		else
//		{
//			CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//			ASSERT(cTokenizer->GetCount() > 0);
//			if (gcPowerLog->IsShowCommunicationLog() == true)
//			{
//				TRACE("[PWR] CReadJobFile Read_TRAY(%d) TokenCount:%d\n", TrayCount, cTokenizer->GetCount());
//			}
//			iCnt = dCnt = 0;
//			ZeroMemory(&iValue, sizeof(iValue));
//			ZeroMemory(&dValue, sizeof(dValue));
//			for (int i = 0; i < cTokenizer->GetCount(); i++)
//			{
//				strValue = cTokenizer->GetString(i);
//				if (i == 0)
//				{
//					Tray.Name.Format(_T("%s"), (LPCTSTR)strValue);
//				}
//				else
//				{
//					if (strValue.Find(_T(".")) >= 0)
//					{
//						dValue[dCnt] = cTokenizer->GetDouble(i);
//						dCnt++;
//					}
//					else
//					{
//						iValue[iCnt] = cTokenizer->GetInt(i);
//						iCnt++;
//					}
//				}
//			}
//			long Pocket = 0, MaxPocket = iValue[0];
//			for (long Count = 0; Count < MaxPocket; ++Count)
//			{
//				Tray.pt[Count].x = dValue[Pocket++];
//				Tray.pt[Count].y = dValue[Pocket++];
//				TRACE(_T("[PWR] PocketCount:%03d X,Y,%.3f,%.3f\n"), Count + 1, Tray.pt[Count].x, Tray.pt[Count].y);
//			}
//			SetTray(Tray, TrayCount + 1);
//			delete cTokenizer;
//			cTokenizer = NULL;
//		}
//	}
//	//bRet = cFile->ReadString(str);			// [EOF_PACKAGE]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//	return Err;
//}
//
//long CReadJobFile::Read_AVOIDMOTION(CStdioFile* cFile)
//{
//	long Err = NO_ERR;
//	BOOL bRet = false;
//	CString str;
//	int iValue[10];
//	double dValue[10];
//	CString strValue;
//	long iCnt = 0, dCnt = 0;
//
//	bRet = cFile->ReadString(str);			// [AVOID]
//	if (bRet == false)
//	{
//		return JOBFILE_READSTRING_FAIL;
//	}
//
//	if (str.CompareNoCase(_T("[EOF_AVOID]")) == 0)
//	{
//		return NO_ERR;
//	}
//
//	CString strLocateID;
//	Point_XYRZ xyrz;
//	long insertNo = 0, Order = 0, Use = 0;
//	ZeroMemory(&xyrz, sizeof(xyrz));
//	for (long avoidCount = 0; avoidCount < MAXINSERTNO; avoidCount++)
//	{
//		bRet = cFile->ReadString(str);
//
//		if (str.CompareNoCase(_T("[EOF_AVOID]")) == 0)
//		{
//			return NO_ERR;
//		}
//
//		if (bRet == false)
//		{
//			return JOBFILE_READSTRING_FAIL;
//		}
//
//		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
//		ASSERT(cTokenizer->GetCount() > 0);
//		if (gcPowerLog->IsShowCommunicationLog() == true)
//		{
//			TRACE("[PWR] CReadJobFile Read_AVOIDMOTION(%d) TokenCount:%d\n", avoidCount, cTokenizer->GetCount());
//		}
//
//		ZeroMemory(&iValue, sizeof(iValue));
//		ZeroMemory(&dValue, sizeof(dValue));
//		iCnt = dCnt = 0;
//
//		for (int i = 0; i < cTokenizer->GetCount(); i++)
//		{
//			strValue = cTokenizer->GetString(i);
//			if (i == 1)
//			{
//				strLocateID.Format(_T("%s"), (LPCTSTR)strValue);
//			}
//			else
//			{
//				if (strValue.Find(_T(".")) >= 0)
//				{
//					dValue[dCnt] = cTokenizer->GetDouble(i);
//					dCnt++;
//				}
//				else
//				{
//					iValue[iCnt] = cTokenizer->GetInt(i);
//					iCnt++;
//				}
//			}
//		}
//		Order = iValue[0];
//		insertNo = iValue[1];
//		xyrz.x = dValue[0];
//		xyrz.y = dValue[1];
//		xyrz.r = dValue[2];
//		xyrz.z = dValue[3];
//		if (xyrz.z < 5.0) // 최소 Avoid 안전 높이
//		{
//			xyrz.z = 5.0;
//		}
//		Use = iValue[2];
//		SetAvoidMotion(insertNo, Order, Use, xyrz);
//		delete cTokenizer;
//		cTokenizer = NULL;
//	}
//	return Err;
//}




long CReadJobFile::FindJobBlock(CStringArray* AllData, CString strString, CString strEnd, CStringArray* ResultData)
{
	INT_PTR line;
	CString strTemp;
	bool Find = false;

	ResultData->RemoveAll();

	for (line = 0; line < AllData->GetCount(); line++)
	{
		strTemp = AllData->GetAt(line);

		if (strTemp.CompareNoCase(strString) == 0)
		{
			Find = true;
			break;
		}
	}

	if (Find == true)
	{
		line++;

		for (; line < AllData->GetCount(); line++)
		{
			strTemp = AllData->GetAt(line);

			if (strTemp.CompareNoCase(strEnd) == 0)
			{
				break;
			}

			ResultData->Add(strTemp);
		}
	}

	return NO_ERR;
}

long CReadJobFile::Read_PRODUCTION_INFO(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;

	CStringArray* Data = new CStringArray;
	FindJobBlock(AllData, _T("[PRODUCTION_INFO]"), _T("[EOF_PRODUCTION_INFO]"), Data);

	if (Data->GetCount() != 1)
	{
		TRACE("[PWR] Read_PRODUCTION_INFO Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);
	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile Read_PRODUCTION_INFO TokenCount:%d\n", cTokenizer->GetCount());
	}
	long Gantry = FRONT_GANTRY;
	CString strValue;
	int iValue[20];
	ZeroMemory(&iValue, sizeof(iValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		iValue[i] = cTokenizer->GetInt(i);
	}
	PRODUCTION Prod;
	Prod.JobFileVersion = iValue[0];
	Prod.PlanQuantity = iValue[1];
	Prod.ProdQuantity = iValue[2];
	Prod.ProdMode = iValue[3];
	Prod.BoardLocation = iValue[4];
	Prod.FirstPickingTiming = iValue[5];
	Prod.TotalFeederCount = iValue[6];
	Prod.UseFeederCount = iValue[7];
	Prod.TotalInsertCount = iValue[8];
	Prod.UseInsertCount = iValue[9];
	SetProduction(Prod);
	delete cTokenizer;
	cTokenizer = NULL;
	delete Data;
	return Err;
}

long CReadJobFile::Read_PCB(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[PCB]"), _T("[EOF_PCB]"), Data);

	if (Data->GetCount() != 1)
	{
		TRACE("[PWR] Read_PCB Invalid Data Count:%d\n", Data->GetCount());
		delete Data;

		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);

	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile Read_PCB TokenCount:%d\n", cTokenizer->GetCount());
	}
	long iCnt = 0, dCnt = 0;
	CString strValue;
	int iValue[100];
	double dValue[100];
	PCB Pcb;
	ZeroMemory(&iValue, sizeof(iValue));
	ZeroMemory(&dValue, sizeof(dValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (i == 0)
		{
			Pcb.Name = strValue;
		}
		else
		{
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
	}
	Pcb.Length = dValue[0];
	Pcb.Width = dValue[1];
	Pcb.Thickness = dValue[2];

	double MaxCompSafe;
	if (gCMachineConfig->IsANCUpType(FRONT_STAGE) == true && gCMachineConfig->IsANCUpType(REAR_STAGE) == true)
	{
		MaxCompSafe = SAFTY_MAXCOMPONENTHEIGHT_REV1;
	}
	else
	{
		MaxCompSafe = SAFTY_MAXCOMPONENTHEIGHT;
	}

	if (dValue[3] < MaxCompSafe)
	{
		Pcb.MaxComponentHeight = MaxCompSafe;
		TRACE(_T("[PWR] User Input MaxComponentHeight:%.3f Change to %.3f\n"), dValue[3], MaxCompSafe);
	}
	else
	{
		Pcb.MaxComponentHeight = dValue[3];
	}
	Pcb.StandByPusherZOffsetHeight = dValue[4];
	Pcb.UseBlockType = iValue[0];
	Pcb.UseFiducial = iValue[1];
	Pcb.MaxBlockCount = iValue[2];
	Pcb.UseBlockCount = iValue[3];
	Pcb.MaxPcbLoadingCount = iValue[4];
	Pcb.BlockSequenceMode = iValue[5];
	Pcb.UseHeightMeasurement = iValue[6];
	Pcb.UseSimultaneousLoading = iValue[7];
	Pcb.PusherRatioUp = iValue[8];
	Pcb.PusherRatioDown = iValue[9];
	Pcb.SimulLoadType = iValue[10];
	Pcb.UseLastPickReUse = iValue[11];

	SetPcb(Pcb);
	delete cTokenizer;
	cTokenizer = NULL;
	delete Data;

	return Err;
}

long CReadJobFile::Read_ORIGIN(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	CStringArray* Data = new CStringArray;
	PCB Pcb = GetPcb();

	FindJobBlock(AllData, _T("[ORIGIN]"), _T("[EOF_ORIGIN]"), Data);

	if (Data->GetCount() != 1 + (INT_PTR)Pcb.MaxBlockCount)
	{
		TRACE("[PWR] Read_PCB Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);
	ORIGIN Origin = ParseOriginFromStr(str);
	SetOrigin(Origin);
	for (long blk = 1; blk <= Pcb.MaxBlockCount; ++blk)
	{
		str = Data->GetAt(blk);
		Origin = ParseBlockOriginFromStr(str);
		SetBlockOrigin(Origin, blk);
	}
	delete Data;

	return Err;
}

long CReadJobFile::Read_FIDUCIAL(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	PCB Pcb = GetPcb();
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[FIDUCIAL]"), _T("[EOF_FIDUCIAL]"), Data);

	if (Data->GetCount() != 1/* + (INT_PTR)Pcb.MaxBlockCount*/)
	{
		TRACE("[PWR] Read_FIDUCIAL Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);

	FIDUCIAL Fiducial = ParseFiducialFromStr(str);
	if (Pcb.UseFiducial > 0)
	{
		Fiducial.Use[0] = Fiducial.Use[1] = 1;
	}
	else
	{
		Fiducial.Use[0] = Fiducial.Use[1] = 0;
	}
	SetMark(Fiducial);

	delete Data;
	return Err;
}

long CReadJobFile::Read_PICK(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	INT_PTR line = 0;

	//	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	PICK Pick;

	CStringArray* Data = new CStringArray;
	FindJobBlock(AllData, _T("[PICK]"), _T("[EOF_PICK]"), Data);

	if (Data->GetCount() != Prod.TotalFeederCount)
	{
		TRACE("[PWR] Read_PICK Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	for (long PickCount = 0; PickCount < Prod.TotalFeederCount; ++PickCount)
	{
		str = Data->GetAt(PickCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_PICK(%d) TokenCount:%d\n", PickCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Pick, sizeof(Pick));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Pick.FeederNo = iValue[0];
		Pick.NextFeedNo = iValue[1];
		Pick.Use = iValue[2];
		Pick.UseSimultaneous = iValue[3];
		Pick.Offset.x = dValue[0];
		Pick.Offset.y = dValue[1];
		Pick.Offset.r = dValue[2];
		Pick.Offset.z = dValue[3];
		SetPick(Pick, Pick.FeederNo);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;

	return Err;
}

long CReadJobFile::Read_PACKAGE(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	INT_PTR line = 0;
	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[100];
	double dValue[100];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	PACKAGE Package;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[PACKAGE]"), _T("[EOF_PACKAGE]"), Data);

	if (Data->GetCount() == 0 || Data->GetCount() > MAXPACKAGENO)
	{
		TRACE("[PWR] Read_PACKAGE Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	for (long PackageCount = 0; PackageCount < Data->GetCount(); ++PackageCount)
	{
		str = Data->GetAt(PackageCount);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_PACKAGE(%d) TokenCount:%d\n", PackageCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 0)
			{
				Package.Name.Format(_T("%s"), (LPCTSTR)strValue);
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}
		Package.Length = dValue[0];
		Package.Width = dValue[1];
		Package.Height = dValue[2];
		Package.LeadHeight = dValue[3];
		Package.Led.Top = iValue[0];
		Package.Led.Mid = iValue[1];
		Package.Led.Bot = iValue[2];
		Package.PickDelay = iValue[3];
		Package.BlowDelay = iValue[4];
		Package.ReleaseDelay = iValue[5];
		Package.Ratio.xy = (double)(iValue[6] / 100.0);
		Package.Ratio.r = (double)(iValue[7] / 100.0);
		Package.Ratio.z = (double)(iValue[8] / 100.0);
		Package.PickRetry = iValue[9];
		Package.TwoStepPick.Use = iValue[10];
		Package.TwoStepPick.Dist = dValue[4];
		Package.TwoStepPick.Ratio = (double)(iValue[11] / 100.0);
		Package.TwoStepInsert.Use = iValue[12];
		Package.TwoStepInsert.Dist = dValue[5];
		Package.TwoStepInsert.Ratio = (double)(iValue[13] / 100.0);
		Package.InsertZOffset = dValue[6];
		Package.RecogAngle = dValue[7];
		Package.RecogOffsetHeight = dValue[8];
		Package.PartEmptyStop = iValue[14];
		Package.RotationHeightOffset_ZOffset = dValue[9];

		Package.LaserControl = iValue[16];
		Package.CatchDelay = iValue[17];

		Package.PickErrorCond.Use = iValue[18];
		Package.PickErrorCond.ReferenceCount = iValue[19];
		Package.PickErrorCond.AlarmCount = iValue[20];

		Package.InsertCompleteMode = iValue[21];

		Package.TwoStepInsertUp.Use = iValue[22];
		Package.TwoStepInsertUp.Dist = dValue[10];
		Package.TwoStepInsertUp.Ratio = (double)(iValue[23] / 100.0);

		Package.PartTorqueLimit.PickDown.Use = iValue[24];
		Package.PartTorqueLimit.PickDown.TorqueLimit = dValue[11];	
		Package.PartTorqueLimit.PickDown2nd.Use = iValue[25];
		Package.PartTorqueLimit.PickDown2nd.TorqueLimit = dValue[12];

		Package.PartTorqueLimit.InsertDown.Use = iValue[26];
		Package.PartTorqueLimit.InsertDown.TorqueLimit = dValue[13];
		Package.PartTorqueLimit.InsertDown2nd.Use = iValue[27];
		Package.PartTorqueLimit.InsertDown2nd.TorqueLimit = dValue[14];

		SetPackage(Package, PackageCount + 1);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;
	return Err;

}

long CReadJobFile::Read_INSERT(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strStart, strEnd;
	CStringArray* Data = new CStringArray;

	//if (Gantry == FRONT_GANTRY)
	//{
	//	FindJobBlock(AllData, _T("[INSERT]"), _T("[EOF_INSERT]"), Data);
	//	if (Data->GetCount() != Prod.UseFrontInsertCount)
	//	{
	//		TRACE("[PWR] Read_INSERT Invalid Data Count:%d\n", Data->GetCount());
	//		delete Data;
	//		return JOBFILE_READSTRING_FAIL;
	//	}
	//}
	//else
	//{
	//	FindJobBlock(AllData, _T("[INSERT2]"), _T("[EOF_INSERT2]"), Data);
	//	if (Data->GetCount() != Prod.UseRearInsertCount)
	//	{
	//		TRACE("[PWR] Read_INSERT Invalid Data Count:%d\n", Data->GetCount());
	//		delete Data;
	//		return JOBFILE_READSTRING_FAIL;
	//	}
	//}
	FindJobBlock(AllData, _T("[INSERT]"), _T("[EOF_INSERT]"), Data);

	if (Data->GetCount() != Prod.TotalInsertCount)
	{
		TRACE("[PWR] Read_INSERT Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}


	INSERT Insert;

	for (long InsertCount = 0; InsertCount < Data->GetCount(); ++InsertCount)
	{
		str = Data->GetAt(InsertCount);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_INSERT(%d) TokenCount:%d\n", InsertCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Insert, sizeof(Insert));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Insert.index = iValue[0];
		Insert.Step = iValue[1];
		Insert.Use = iValue[2];
		Insert.FeederNo = iValue[3];
		Insert.PickOrder = iValue[4];
		Insert.InsertOrder = iValue[5];
		Insert.HeadNo = iValue[6];
		Insert.pt.x = dValue[0];
		Insert.pt.y = dValue[1];
		Insert.pt.r = dValue[2];
		Insert.pt.z = dValue[3];
		Insert.BlockNo = iValue[7];
		Insert.NozzleNo = iValue[8];
		Insert.RecogTable = iValue[9];
		Err = SetInsert(Insert, InsertCount + 1);
		delete cTokenizer;
		cTokenizer = NULL;
		if (Err != NO_ERR)
		{
			return Err;
		}
	}
	delete Data;

	return Err;
}


long CReadJobFile::Read_MEASUREHEIGHT(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	//	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	//PICK Pick;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[HEIGHTMEASUREMENT]"), _T("[EOF_HEIGHTMEASUREMENT]"), Data);

	if (Data->GetCount() > MAXINSERTNO)
	{
		TRACE("[PWR] Read_MEASUREHEIGHT Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	MEASUREHEIGHT temp;

	for (long inspectCount = 0; inspectCount < Data->GetCount(); inspectCount++)
	{
		str = Data->GetAt(inspectCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_MEASUREHEIGHT(%d) TokenCount:%d\n", inspectCount, cTokenizer->GetCount());
		}

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		iCnt = dCnt = 0;

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 1)
			{
				temp.LocateID.Format(_T("%s"), (LPCTSTR)strValue);
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}

		temp.Order = iValue[0];
		temp.pt.x = dValue[0];
		temp.pt.y = dValue[1];
		temp.pt.t = dValue[2];
		temp.Tolerance = dValue[3];
		temp.Use = iValue[1];
		temp.Method = iValue[2];
		temp.InsertNo = iValue[3];
		temp.GroupNo = iValue[4];
        temp.GantryNo = FRONT_GANTRY;

		//data.push_back(temp);

		SetMeasureHeight(temp);

		delete cTokenizer;
		cTokenizer = NULL;
	}

	delete Data;
	return Err;
}

long CReadJobFile::Read_FEEDER(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;

	//	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[50];
	double dValue[50];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	CStringArray* Data = new CStringArray;
	bool CheckTTF = true;

	FindJobBlock(AllData, _T("[FEEDER]"), _T("[EOF_FEEDER]"), Data);

	if (Data->GetCount() != Prod.TotalFeederCount)
	{
		TRACE("[PWR] Read_FEEDER Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	FEEDER Feeder;
	for (long FeederCount = 0; FeederCount < Prod.TotalFeederCount; ++FeederCount)
	{
		str = Data->GetAt(FeederCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_FEEDER(%d) TokenCount:%d\n", FeederCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		//	InitFeeder(FeederCount);
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 0)
			{
				Feeder.PackageName = strValue;
			}
			else if (i == 7)
			{
				Feeder.TrayName = strValue;
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}
		Feeder.No = iValue[0];
		Feeder.Type = iValue[1];
		Feeder.TimeOut = iValue[2];
		Feeder.ReadyIONo = iValue[3];
		Feeder.ReleaseIONo = iValue[4];
		Feeder.ReadyIOWaitTime = iValue[5];
		Feeder.TrayMaxPocket = iValue[6];
		Feeder.TrayNowPocket = iValue[7] - 1;



		//Feeder.PalletNo = iValue[8];
		//if (10 <= iValue[9] && iValue[9] <= 100)
		//{
		//	Feeder.TTFRatio.y = (double)(iValue[9] / 100.0);
		//}
		//else
		//{
		//	Feeder.TTFRatio.y = TTF_RATIO_DEFAULT;
		//}

		//if (10 <= iValue[10] && iValue[10] <= 100)
		//{
		//	Feeder.TTFRatio.z = (double)(iValue[10] / 100.0);
		//}
		//else
		//{
		//	Feeder.TTFRatio.z = TTF_RATIO_DEFAULT;
		//}

		//Feeder.ReadyIOType = iValue[11];
		//Feeder.AutoTeaching.Use = iValue[12];
		//Feeder.AutoTeaching.Timing = iValue[13];


		Feeder.PickLevel.Use = false;
		Feeder.PickLevel.FirstPocketFind = false;
		Feeder.PickLevel.DifferentLevel = iValue[14];
		//Feeder.PickLevel.AfterLevel = iValue[15];

		if (IsLabelTypeFeeder(Feeder.Type) == true)
		{
			Feeder.ReleaseIONo = 0;
			Feeder.ReadyIOType = 0;

			if (Feeder.PickLevel.DifferentLevel > 0)
			{
				Feeder.PickLevel.Use = true;
				Feeder.TrayNowPocket = 0;
			}
		}
		else if (IsTrayTypeFeeder(Feeder.Type) == true)
		{
			if (Feeder.ReadyIONo != 0 || Feeder.ReleaseIONo != 0 || Feeder.ReadyIOType != 0)
			{
				TRACE(_T("[PWR] Tray Feeder No(%d) Ready IO Clear(%d,%d) \n"), Feeder.No, Feeder.ReadyIONo, Feeder.ReleaseIONo);
				Feeder.ReadyIONo = Feeder.ReleaseIONo = Feeder.ReadyIOType = 0;
			}
		}

		SetFeeder(Feeder, Feeder.No);
		delete cTokenizer;
		cTokenizer = NULL;
	}

	delete Data;
	return Err;
}

long CReadJobFile::Read_LED_VALUE(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[LED_VALUE]"), _T("[EOF_LED_VALUE]"), Data);

	if (Data->GetCount() > MAXPACKAGENO)
	{
		TRACE("[PWR] Read_LED_VALUE Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	RETRY_LED RetryLed;

	for (long RetryCount = 0; RetryCount < Data->GetCount(); ++RetryCount)
	{
		str = Data->GetAt(RetryCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_LED_VALUE(%d) TokenCount:%d\n", RetryCount, cTokenizer->GetCount());
		}
		iCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 0)
			{
				RetryLed.PackageName = strValue;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		RetryLed.MaxRetry = iValue[0];
		for (long Count = 0; Count < RetryLed.MaxRetry; ++Count)
		{
			RetryLed.Led[Count].Top = iValue[(Count * 3) + 1]; // 1, 4, 7, 10, 13
			RetryLed.Led[Count].Mid = iValue[(Count * 3) + 2]; // 2, 5, 8, 11, 14
			RetryLed.Led[Count].Bot = iValue[(Count * 3) + 3]; // 3, 6, 9, 12, 15
		}
		SetRetryLed(RetryLed, RetryCount + 1);

		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;
	return Err;
}
long CReadJobFile::Read_NOZZLE(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	//PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[NOZZLE]"), _T("[EOF_NOZZLE]"), Data);

	if (Data->GetCount() > MAXNOZZLENO)
	{
		TRACE("[PWR] Read_NOZZLE Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	NOZZLE Nozzle;

	for (long NozzleCount = 0; NozzleCount < Data->GetCount(); ++NozzleCount)
	{
		str = Data->GetAt(NozzleCount);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_NOZZLE(%d) TokenCount:%d\n", NozzleCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&Nozzle, sizeof(Nozzle));
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Nozzle.No = iValue[0];
		Nozzle.Type = iValue[1];
		Nozzle.TipHeight = dValue[0];
		Nozzle.PusherHeight = dValue[1];
		Nozzle.Empty = iValue[2];
		Nozzle.EmptyDiff = iValue[3];
		Nozzle.Exist = iValue[4];
		Nozzle.EmptyDiff = iValue[5];
		SetNozzle(Nozzle, Nozzle.No);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;
	return Err;
}

long CReadJobFile::Read_DISCARD(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[DISCARD]"), _T("[EOF_DISCARD]"), Data);

	if (Data->GetCount() != Prod.TotalFeederCount)
	{
		TRACE("[PWR] Read_DISCARD Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	DISCARD Discard;

	for (long DiscardCount = 0; DiscardCount < Prod.TotalFeederCount; ++DiscardCount)
	{
		str = Data->GetAt(DiscardCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_DISCARD(%d) TokenCount:%d\n", DiscardCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Discard, sizeof(Discard));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Discard.FeederNo = iValue[0];
		Discard.Mode = iValue[1];
		Discard.pt.x = dValue[0];
		Discard.pt.y = dValue[1];
		Discard.pt.r = dValue[2];
		Discard.pt.z = dValue[3];
		SetDiscard(Discard, Discard.FeederNo);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;
	return Err;
}


long CReadJobFile::Read_BARCODE(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	//	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	BARCODE Barcode, BarcodeBlock1, BarcodeBlock2;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[BARCODE]"), _T("[EOF_BARCODE]"), Data);

	if (Data->GetCount() > MAXTRAYNO)
	{
		TRACE("[PWR] Read_BARCODE Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	for (long Count = 0; Count < Data->GetCount(); ++Count)
	{
		iCnt = dCnt = 0;
		str = Data->GetAt(Count);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Barcode.pt, sizeof(Barcode.pt));
		ZeroMemory(&BarcodeBlock1.pt, sizeof(Barcode.pt));
		ZeroMemory(&BarcodeBlock2.pt, sizeof(Barcode.pt));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		Barcode.Use = iValue[0];
		Barcode.pt[0].x = dValue[0];
		Barcode.pt[0].y = dValue[1];
		Barcode.Led.Blue = iValue[1];
		Barcode.Led.Red = iValue[2];
		Barcode.Mes.Use = iValue[3];
		Barcode.Type = iValue[4]; // 20210415 HarkDo
		TRACE(_T("[PWR] ReadBarcode Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), iValue[0], dValue[0], dValue[1], iValue[1], iValue[2], iValue[3], iValue[4]);
		SetBarcode(Barcode);

		// 20210415 HarkDo
		BarcodeBlock1.Use = Barcode.Use;
		BarcodeBlock1.pt[0].x = dValue[2];
		BarcodeBlock1.pt[0].y = dValue[3];
		BarcodeBlock1.Led.Blue = iValue[5];
		BarcodeBlock1.Led.Red = iValue[6];
		BarcodeBlock1.Mes.Use = Barcode.Mes.Use;
		BarcodeBlock1.Type = iValue[7];
		TRACE(_T("[PWR] ReadBarcodeBlock1 Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), BarcodeBlock1.Use, BarcodeBlock1.pt[0].x, BarcodeBlock1.pt[0].y, BarcodeBlock1.Led.Blue, BarcodeBlock1.Led.Red, BarcodeBlock1.Mes.Use, BarcodeBlock1.Type);
		SetBarcodeBlock1(BarcodeBlock1);

		BarcodeBlock2.Use = Barcode.Use;
		BarcodeBlock2.pt[0].x = dValue[4];
		BarcodeBlock2.pt[0].y = dValue[5];
		BarcodeBlock2.Led.Blue = iValue[8];
		BarcodeBlock2.Led.Red = iValue[9];
		BarcodeBlock2.Mes.Use = Barcode.Mes.Use;
		BarcodeBlock2.Type = iValue[10];
		TRACE(_T("[PWR] ReadBarcodeBlock2 Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), BarcodeBlock2.Use, BarcodeBlock2.pt[0].x, BarcodeBlock2.pt[0].y, BarcodeBlock2.Led.Blue, BarcodeBlock2.Led.Red, BarcodeBlock2.Mes.Use, BarcodeBlock2.Type);
		SetBarcodeBlock2(BarcodeBlock2);

		delete cTokenizer;
		cTokenizer = NULL;
	}

	delete Data;

	return Err;
}


long CReadJobFile::Read_STANDBY(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	//PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	STANDBY StandBy;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[STANDBY]"), _T("[EOF_STANDBY]"), Data);

	if (Data->GetCount() != 1)
	{
		TRACE("[PWR] Read_STANDBY Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);

	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] CReadJobFile Read_STANDBY TokenCount:%d\n"), cTokenizer->GetCount());
	}
	iCnt = dCnt = 0;
	ZeroMemory(&iValue, sizeof(iValue));
	ZeroMemory(&dValue, sizeof(dValue));
	ZeroMemory(&StandBy, sizeof(StandBy));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (strValue.Find(_T(".")) >= 0)
		{
			dValue[dCnt] = cTokenizer->GetDouble(i);
			dCnt++;
		}
		else
		{
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
	}
	StandBy.pt.x = dValue[0];
	StandBy.pt.y = dValue[1];
	StandBy.pt.r = dValue[2];
	StandBy.pt.z = dValue[3];
	if (StandBy.pt.z > SAFTY_ZHEIGHT)
	{
		TRACE(_T("[PWR] INVALID StandBy Z,%.3f\n"), StandBy.pt.z);
		Err = INVALID_STANDBYZ;
	}
	else
	{
		SetStandyBy(StandBy);
	}
	delete cTokenizer;
	cTokenizer = NULL;
	delete Data;

	return Err;
}

long CReadJobFile::Read_PICKUPZSTANDBY(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str, strTemp;
	//	PCB Pcb = GetPcb();
	//PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue;
	double PickupZStandBy = 70.0;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[PICKUPZSTANDBY]"), _T("[EOF_PICKUPZSTANDBY]"), Data);

	if (Data->GetCount() != 1)
	{
		TRACE("[PWR] Read_PICKUPZSTANDBY Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);

	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE(_T("[PWR] CReadJobFile Read_PICKUPZSTANDBY TokenCount:%d\n"), cTokenizer->GetCount());
	}
	iCnt = dCnt = 0;
	ZeroMemory(&iValue, sizeof(iValue));
	ZeroMemory(&dValue, sizeof(dValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		strValue = cTokenizer->GetString(i);
		if (strValue.Find(_T(".")) >= 0)
		{
			dValue[dCnt] = cTokenizer->GetDouble(i);
			dCnt++;
		}
		else
		{
			iValue[iCnt] = cTokenizer->GetInt(i);
			iCnt++;
		}
	}
	PickupZStandBy = dValue[0];
	if (PickupZStandBy > SAFTY_ZHEIGHT)
	{
		TRACE(_T("[PWR] INVALID Pickup Z StandBy,%.3f\n"), PickupZStandBy);
		Err = INVALID_PICKUPZSTANDBY;
	}
	else
	{
		SetPickupZStandBy(PickupZStandBy);
	}
	delete cTokenizer;
	cTokenizer = NULL;
	delete Data;

	return Err;
}

long CReadJobFile::Read_TRAY(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[1500];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	TRAY_INFO Tray;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[TRAY]"), _T("[EOF_TRAY]"), Data);

	if (Data->GetCount() > MAXTRAYNO)
	{
		TRACE("[PWR] Read_TRAY Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	for (long TrayCount = 0; TrayCount < Data->GetCount(); ++TrayCount)
	{
		str = Data->GetAt(TrayCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_TRAY(%d) TokenCount:%d\n", TrayCount, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 0)
			{
				Tray.Name.Format(_T("%s"), (LPCTSTR)strValue);
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}
		long Pocket = 0, MaxPocket = iValue[0];
		for (long Count = 0; Count < MaxPocket; ++Count)
		{
			Tray.pt[Count].x = dValue[Pocket++];
			Tray.pt[Count].y = dValue[Pocket++];
			TRACE(_T("[PWR] PocketCount:%03d X,Y,%.3f,%.3f\n"), Count + 1, Tray.pt[Count].x, Tray.pt[Count].y);
		}
		SetTray(Tray, TrayCount + 1);
		delete cTokenizer;
		cTokenizer = NULL;

	}
	delete Data;
	return Err;
}

long CReadJobFile::Read_AVOIDMOTION(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false;
	CString str;
	int iValue[10];
	double dValue[10];
	CString strValue;
	long iCnt = 0, dCnt = 0;

	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[AVOID]"), _T("[EOF_AVOID]"), Data);

	if (Data->GetCount() > MAXINSERTNO)
	{
		TRACE("[PWR] Read_AVOIDMOTION Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}


	CString strLocateID;
	Point_XYRZ xyrz;
	long insertNo = 0, Order = 0, Use = 0;
	ZeroMemory(&xyrz, sizeof(xyrz));
	for (long avoidCount = 0; avoidCount < Data->GetCount(); avoidCount++)
	{
		str = Data->GetAt(avoidCount);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_AVOIDMOTION(%d) TokenCount:%d\n", avoidCount, cTokenizer->GetCount());
		}

		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		iCnt = dCnt = 0;

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 1)
			{
				strLocateID.Format(_T("%s"), (LPCTSTR)strValue);
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}
		Order = iValue[0];
		insertNo = iValue[1];
		xyrz.x = dValue[0];
		xyrz.y = dValue[1];
		xyrz.r = dValue[2];
		xyrz.z = dValue[3];
		if (xyrz.z < 2.0) // 최소 Avoid 안전 높이
		{
			xyrz.z = 2.0;
		}
		Use = iValue[2];
		SetAvoidMotion(insertNo, Order, Use, xyrz);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;

	return Err;
}

long CReadJobFile::Read_PartDrop(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	int iValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[CAM_DROPPED_OBJECT_INS]"), _T("[EOF_CAM_DROPPED_OBJECT_INS]"), Data);

	if (Data->GetCount() == 0)
	{
		delete Data;
		return NO_ERR;
	}

	if (Data->GetCount() > 1)
	{
		TRACE("[PWR] Read_PartDrop Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	str = Data->GetAt(0);
	CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
	ASSERT(cTokenizer->GetCount() > 0);
	if (gcPowerLog->IsShowCommunicationLog() == true)
	{
		TRACE("[PWR] CReadJobFile Read_PartDrop TokenCount:%d\n", cTokenizer->GetCount());
	}

	ZeroMemory(&iValue, sizeof(iValue));
	for (int i = 0; i < cTokenizer->GetCount(); i++)
	{
		iValue[i] = cTokenizer->GetInt(i);
	}

	PART_DROP drop;
	drop.Use = iValue[0];
	drop.led.Top = iValue[1];
	drop.led.Mid = iValue[2];
	drop.led.Bot = iValue[3];

	SetPartDrop(drop);
	delete cTokenizer;
	cTokenizer = NULL;
	delete Data;
	return Err;
}


long CReadJobFile::Read_DivideInspect(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	PRODUCTION Prod = GetProduction();
	int iValue[200];
	double dValue[200];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	DIVIDE_INSPECT Divide;
	CStringArray* Data = new CStringArray;
	CString strFunc(__func__);

	FindJobBlock(AllData, _T("[DIVISION_INFO]"), _T("[EOF_DIVISION_INFO]"), Data);

	if (Data->GetCount() > MAXFEEDERNO)
	{
		TRACE(_T("[PWR] %s Invalid Data Count:%d\n"), strFunc, Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	InitDivideInspect();

	for (long Count = 0; Count < Data->GetCount(); ++Count)
	{
		str = Data->GetAt(Count);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_InsertPriority(%d) TokenCount:%d\n", Count, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		ZeroMemory(&Divide.Offset, sizeof(Divide.Offset));

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (i == 0)
			{
				Divide.FeederNo = cTokenizer->GetInt(i);
			}
			else if (i == 1)
			{
				Divide.PackageName.Format(_T("%s"), (LPCTSTR)strValue);
			}
			else if (i == 2)
			{
				Divide.Count = cTokenizer->GetInt(i);
			}
			else
			{
				if (strValue.Find(_T(".")) >= 0)
				{
					dValue[dCnt] = cTokenizer->GetDouble(i);
					dCnt++;
				}
				else
				{
					iValue[iCnt] = cTokenizer->GetInt(i);
					iCnt++;
				}
			}
		}

		for (long count = 0; count < Divide.Count; count++)
		{
			Divide.Offset[count].x = dValue[count * 4];
			Divide.Offset[count].y = dValue[count * 4 + 1];
			Divide.Offset[count].r = dValue[count * 4 + 2];
			Divide.Offset[count].z = dValue[count * 4 + 3];
		}

		Divide.Use = true;
		SetDivideInspect(Divide);

		delete cTokenizer;
		cTokenizer = NULL;

	}
	delete Data;
	return Err;
}


void CReadJobFile::SetProduction(PRODUCTION Prod)
{
	m_Production = Prod;
	TRACE(_T("[PWR] Version(%d) Plan(%04d) Prod(%04d) Mode(%d) Location(%d) FirstPicking(%d) TotalFeeder(%d) UsedFeeder(%d) TotalInsert(%d) UsedInsert(%d)\n"),
		Prod.JobFileVersion, Prod.PlanQuantity, Prod.ProdQuantity, Prod.ProdMode, Prod.BoardLocation,
		Prod.FirstPickingTiming, Prod.TotalFeederCount, Prod.UseFeederCount, Prod.TotalInsertCount, Prod.UseInsertCount);
}

void CReadJobFile::SetPcb(PCB Pcb)
{
	if (Pcb.SimulLoadType > SIMULLOAD_OUTON)
	{
		TRACE(_T("[PWR] SimulLoadType Reset(%d)\n"), Pcb.SimulLoadType);
		Pcb.SimulLoadType = SIMULLOAD_NOUSE;
	}

	m_Pcb = Pcb;
	TRACE(_T("[PWR] PCB Name(%s) Length(%.3f) Width(%.3f) Thickness(%.3f) MaxComponentHeight(%.3f) StandByPusherZOffset(%.3f)\n"),
		Pcb.Name, Pcb.Length, Pcb.Width, Pcb.Thickness, Pcb.MaxComponentHeight, Pcb.StandByPusherZOffsetHeight);
	TRACE(_T("[PWR] PCB BlockType(%d) Fiducial(%d) MaxBlock(%d) UseBlock(%d) MaxPcbLoadingCount(%d) BlockSequenceMode(%d)\n"),
		Pcb.UseBlockType, Pcb.UseFiducial, Pcb.MaxBlockCount,
		Pcb.UseBlockCount, Pcb.MaxPcbLoadingCount, Pcb.BlockSequenceMode);
	TRACE(_T("[PWR] Use MeasureHeight:%d Simultaneous Loading:%d\n"), Pcb.UseHeightMeasurement, Pcb.UseSimultaneousLoading);
	TRACE(_T("[PWR] Pusher Up%03d%% Down%03d%% SimulLoadType:%d LastPickUse:%d\n"), Pcb.PusherRatioUp, Pcb.PusherRatioDown, Pcb.SimulLoadType, Pcb.UseLastPickReUse);
}

void CReadJobFile::SetOrigin(ORIGIN Origin)
{
	m_Origin = Origin;
	TRACE(_T("[PWR] OriginXY(%.3f,%.3f)\n"), Origin.pt.x, Origin.pt.y);
}

void CReadJobFile::SetBlockOrigin(ORIGIN BlockOrigin, long BlockNo)
{
	if (BlockNo > 0)
	{
		m_BlockOrigin[BlockNo - 1] = BlockOrigin;
		TRACE(_T("[PWR] Block(%03d) OriginXYR(%.3f,%.3f,%.3f) Use(%d)\n"), BlockNo, BlockOrigin.pt.x, BlockOrigin.pt.y, BlockOrigin.pt.r, BlockOrigin.Use);
	}
}

void CReadJobFile::SetMark(FIDUCIAL FidMark)
{
	m_Mark = FidMark;
	TRACE(_T("[PWR] Mark Use(%d,%d) No(%d,%d) XY1(%.3f,%.3f) XY2(%.3f,%.3f) LED1(%03d,%03d) LED2(%03d,%03d)\n"), 
		FidMark.Use[0], FidMark.Use[1],
		FidMark.MarkNo[0], FidMark.MarkNo[1],
		FidMark.pt[0].x, FidMark.pt[0].y,
		FidMark.pt[1].x, FidMark.pt[1].y,
		FidMark.Led[0].Red, FidMark.Led[0].Blue,
		FidMark.Led[1].Red, FidMark.Led[1].Blue);
}

void CReadJobFile::SetBlockMark(FIDUCIAL FidMark, long BlockNo)
{
	if (BlockNo > 0)
	{
		m_BlockMark[BlockNo - 1] = FidMark;
		TRACE(_T("[PWR] Block Mark(%d,%d) Use(%d,%d) No(%d,%d) XYXY1(%.3f,%.3f) XY2(%.3f,%.3f) LED1(%03d,%03d) LED2(%03d,%03d)\n"), 
			BlockNo, 
			FidMark.Use[0], FidMark.Use[1],
			FidMark.MarkNo[0], FidMark.MarkNo[1],
			FidMark.pt[0].x, FidMark.pt[0].y,
			FidMark.pt[1].x, FidMark.pt[1].y,
			FidMark.Led[0].Red, FidMark.Led[0].Blue,
			FidMark.Led[1].Red, FidMark.Led[1].Blue);
	}
}

void CReadJobFile::SetPick(PICK Pick, long FeederNo)
{
	if (FeederNo > 0)
	{
		m_Pick[FeederNo - 1] = Pick;
		TRACE(_T("[PWR] Pick FeederNo(%d) Next(%d) Use(%d) Simul(%d) OffsetXYRZ(%.3f,%.3f,%.3f,%.3f)\n"),
			Pick.FeederNo, Pick.NextFeedNo, Pick.Use, Pick.UseSimultaneous, Pick.Offset.x, Pick.Offset.y, Pick.Offset.r, Pick.Offset.z);
	}
}

void CReadJobFile::SetPackage(PACKAGE Package, long PackageNo)
{
	if (PackageNo > 0)
	{
		m_Package[PackageNo - 1] = Package;
		TRACE(_T("[PWR] Package(%d,%s) Length,Width,Height,Lead(%.3f,%.3f,%.3f,%.3f) Led(Top,Mid,Bot)(%d,%d,%d) Pick,Blow,Release(%04d,%04d,%04d) Speed(%03d%%,%03d%%,%03d%%) Retry(%d)\n"),
			PackageNo,
			Package.Name,
			Package.Length, Package.Width, Package.Height, Package.LeadHeight,
			Package.Led.Top, Package.Led.Mid, Package.Led.Bot,
			Package.PickDelay, Package.BlowDelay, Package.ReleaseDelay, 
			(long)(Package.Ratio.xy * 100), (long)(Package.Ratio.r * 100), (long)(Package.Ratio.z * 100), Package.PickRetry);
		TRACE(_T("[PWR] TwoStepPick Use:%d Dist:%.3f Ratio:%03d%% TwoStepInsert Use:%d Dist:%.3f Ratio:%03d%% InsertZOffset:%.3f\n"),
			Package.TwoStepPick.Use, Package.TwoStepPick.Dist, (long)(Package.TwoStepPick.Ratio * 100),
			Package.TwoStepInsert.Use, Package.TwoStepInsert.Dist, (long)(Package.TwoStepInsert.Ratio * 100),
			Package.InsertZOffset);
		TRACE(_T("[PWR] Recog Angle:%.3f RecogHeight:%.3f EmptyStop:%d\n"), Package.RecogAngle, Package.RecogOffsetHeight, Package.PartEmptyStop);
		TRACE(_T("[PWR] LaserControl:%d CatchDelay:%d\n"), Package.LaserControl, Package.CatchDelay);

		TRACE(_T("[PWR] TwoStepInsertUp Use:%d Dist:%.3f Ratio:%03d%%\n"),
			Package.TwoStepInsertUp.Use, Package.TwoStepInsertUp.Dist, (long)(Package.TwoStepInsertUp.Ratio * 100));

		TRACE(_T("[PWR] PartTorque PickDown:%d,%.1f PickDown2nd:%d,%.1f InsertDown:%d,%.1f InsertDown2nd:%d,%.1f\n"),
			Package.PartTorqueLimit.PickDown.Use, Package.PartTorqueLimit.PickDown.TorqueLimit,
			Package.PartTorqueLimit.PickDown2nd.Use, Package.PartTorqueLimit.PickDown2nd.TorqueLimit,
			Package.PartTorqueLimit.InsertDown.Use, Package.PartTorqueLimit.InsertDown.TorqueLimit,
			Package.PartTorqueLimit.InsertDown2nd.Use, Package.PartTorqueLimit.InsertDown2nd.TorqueLimit);
	}
}

long CReadJobFile::SetInsert(INSERT Insert, long InsertNo)
{
	long Err = NO_ERR;
	if (InsertNo > 0)
	{
        if (this->GetPcb().UseBlockType == PCB_SINGLE)
        {
            if (Insert.BlockNo != 0)
            {
                const long oldBlockNo = Insert.BlockNo;
                Insert.BlockNo = 0;
                CString temp; temp.Format(L"BlockNo changed %d->%d due to (this->GetPcb().UseBlockType == PCB_SINGLE).", oldBlockNo, Insert.BlockNo);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
            }
        }
        else
        {
            if (Insert.BlockNo == 0)
            {
                TRACE_FILE_FUNC_LINE_"early return due to (Insert.BlockNo == 0) while (this->GetPcb().UseBlockType != PCB_SINGLE).");
                (void)SendAlarm(JOBFILE_READSTRING_FAIL, L"Insert.BlockNo is wrong(0)");
                return JOBFILE_READSTRING_FAIL;
            }
        }

		if (Insert.PickOrder == 0)
		{
			Err = INVALID_PICKORDER(InsertNo);
			TRACE(_T("[PWR] Err(%d) Insert(%d) index(%d) Step(%d) Use(%d) PickOrder(%d)\n"),
				Err, InsertNo, Insert.index, Insert.Step, Insert.Use, Insert.PickOrder);
		}
		else if (Insert.InsertOrder == 0)
		{
			Err = INVALID_INSERTORDER(InsertNo);
			TRACE(_T("[PWR] Err(%d) Insert(%d) index(%d) Step(%d) Use(%d) InsertOrder(%d)\n"),
				Err, InsertNo, Insert.index, Insert.Step, Insert.Use, Insert.InsertOrder);
		}
		else
		{
			m_Insert[InsertNo - 1] = Insert;
			m_Insert[InsertNo - 1].pt.z = GetInsertByZ(FRONT_GANTRY);
			TRACE(_T("[PWR] Insert(%d) index(%d) Step(%d) Use(%d) FeederNo(%03d) PickOrder(%d) InsertOrder(%d)\n"),
				InsertNo, Insert.index, Insert.Step, Insert.Use, Insert.FeederNo, Insert.PickOrder, Insert.InsertOrder);
			TRACE(_T("[PWR] HeadNo(%d) XYRZ(%.3f,%.3f,%.3f,%.3f) BlockNo:%03d NozzleNo:%d RecogTable:%d\n"),
				Insert.HeadNo, Insert.pt.x, Insert.pt.y, Insert.pt.r, Insert.pt.z, Insert.BlockNo, Insert.NozzleNo, Insert.RecogTable);
		}
	}
	return Err;
}

long CReadJobFile::SetMeasureHeight(MEASUREHEIGHT inspect)
{
	long Err = NO_ERR;
	long order = inspect.Order;
	if (order > 0 && order <= MAXINSERTNO)
	{
		m_MeasureHeight[order - 1] = inspect;
        TRACE(_T("[PWR] SetMeasureHeight Gantry:%d LocateID:%s InsertNo:%d Order:%d XY:%.3f,%.3f Height:%.3f Tol:%.3f Use:%d Method:%d Group:%d\n")
              , inspect.GantryNo, inspect.LocateID, inspect.InsertNo, inspect.Order, inspect.pt.x, inspect.pt.y, inspect.pt.t, inspect.Tolerance, inspect.Use, inspect.Method, inspect.GroupNo);
	}
	return Err;
}

void CReadJobFile::SetMeasureHeightTotal(long Count)
{
	m_MeasureHeightTotal = Count;
	TRACE(_T("[PWR] SetMeasureHeightTotal(%d)\n"), Count);
}

long CReadJobFile::GetMeasureHeightTotal()
{
	return m_MeasureHeightTotal;
}

void CReadJobFile::SetFeeder(FEEDER Feeder, long FeederNo)
{
	if (FeederNo > 0)
	{
		m_Feeder[FeederNo - 1] = Feeder;
		TRACE(_T("[PWR] Feeder No(%d) PackageName(%s) Type(%d) TimeOut(%d) IO(%d,%d) IOWaitTime(%d) TrayName(%s) Max(%d) Now(%d) ReadyIOType(%d) Level(%d,%d)\n"),
			Feeder.No, Feeder.PackageName, Feeder.Type,
			Feeder.TimeOut, Feeder.ReadyIONo, Feeder.ReleaseIONo, Feeder.ReadyIOWaitTime,
			Feeder.TrayName, Feeder.TrayMaxPocket, Feeder.TrayNowPocket, Feeder.ReadyIOType,
			Feeder.PickLevel.BeforeLevel, Feeder.PickLevel.AfterLevel);
	}
}

void CReadJobFile::SetRetryLed(RETRY_LED RetryLed, long PackageNo)
{
	if (PackageNo > 0)
	{
		RETRY_LED RetryLedLocal;
		RetryLedLocal.MaxRetry = RetryLed.MaxRetry;
		RetryLedLocal.PackageName.Format(_T("%s"), (LPCTSTR)(RetryLed.PackageName));
		for (long index = 0; index < RetryLedLocal.MaxRetry; ++index)
		{
			RetryLedLocal.Led[index].Top = RetryLed.Led[index].Top;
			RetryLedLocal.Led[index].Mid = RetryLed.Led[index].Mid;
			RetryLedLocal.Led[index].Bot = RetryLed.Led[index].Bot;
		}
		m_RetryLed[PackageNo - 1] = RetryLedLocal;
		TRACE(_T("[PWR] Retry Pacakge(%s) Retry(%d) LED(%d,%d,%d)(%d,%d,%d)(%d,%d,%d)(%d,%d,%d)(%d,%d,%d)\n"),
			RetryLedLocal.PackageName, RetryLedLocal.MaxRetry,
			RetryLedLocal.Led[0].Top, RetryLedLocal.Led[0].Mid, RetryLedLocal.Led[0].Bot,
			RetryLedLocal.Led[1].Top, RetryLedLocal.Led[1].Mid, RetryLedLocal.Led[1].Bot,
			RetryLedLocal.Led[2].Top, RetryLedLocal.Led[2].Mid, RetryLedLocal.Led[2].Bot,
			RetryLedLocal.Led[3].Top, RetryLedLocal.Led[3].Mid, RetryLedLocal.Led[3].Bot,
			RetryLedLocal.Led[4].Top, RetryLedLocal.Led[4].Mid, RetryLedLocal.Led[4].Bot);
	}
}

void CReadJobFile::SetNozzle(NOZZLE Nozzle, long NozzleNo)
{
	if (NozzleNo > 0)
	{
		m_Nozzle[NozzleNo - 1] = Nozzle;
		TRACE(_T("[PWR] Nozzle No,Type(%d,%d) Tip,Pusher,%.3f,%.3f Empty,Diff,Exist,Diff(%d,%d,%d,%d)\n"),
			Nozzle.No, Nozzle.Type, 
			Nozzle.TipHeight, Nozzle.PusherHeight,
			Nozzle.Empty, Nozzle.EmptyDiff, Nozzle.Exist, Nozzle.ExistDiff);
	}
}

void CReadJobFile::SetDiscard(DISCARD Discard, long FeederNo)
{
	if (FeederNo > 0)
	{
		m_Discard[FeederNo - 1] = Discard;
		TRACE(_T("[PWR] Discard FeederNo(%04d) Mode(%d) XY(%.3f,%.3f)\n"), Discard.FeederNo,
			Discard.Mode, Discard.pt.x, Discard.pt.y);
        (void)CLastPick::updateDiscardPoint(FeederNo, Discard.pt);
	}
}

void CReadJobFile::SetStandyBy(STANDBY Standby)
{
	m_StandBy = Standby;
	TRACE(_T("[PWR] Standby XYRZ(%.3f,%.3f,%.3f,%.3f)\n"), m_StandBy.pt.x, m_StandBy.pt.y, m_StandBy.pt.r, m_StandBy.pt.z);
}

void CReadJobFile::SetPickupZStandBy(double PickupZStandby)
{
	m_PickupZStandby = PickupZStandby;
	TRACE(_T("[PWR] PickupZStandby (%.3f)\n"), m_PickupZStandby);
}

void CReadJobFile::SetTray(TRAY_INFO Tray, long FeederNo)
{
	if (FeederNo > 0)
	{
		m_Tray[FeederNo - 1] = Tray;
		TRACE(_T("[PWR] Tray Name(%s)\n"), Tray.Name, Tray.MaxPocket);
		for (long Count = 0; Count < Tray.MaxPocket; ++Count)
		{
			TRACE(_T("[PWR] Tray Count(%03d) X,Y,%.3f,%.3f\n"), Count + 1, Tray.pt[Count].x, Tray.pt[Count].y);
		}
	}
}

long CReadJobFile::SetAvoidMotion(long InsertNo, long Order, long Use, Point_XYRZ xyrz)
{
	long Err = NO_ERR;
	if(InsertNo > 0)
	{
		m_AvoidMotion[InsertNo - 1].insertNo = InsertNo;
		if (Order > 0)
		{
			m_AvoidMotion[InsertNo - 1].Order[Order - 1] = Order;
			m_AvoidMotion[InsertNo - 1].Use[Order - 1] = Use;
			m_AvoidMotion[InsertNo - 1].pt[Order - 1].x = xyrz.x;
			m_AvoidMotion[InsertNo - 1].pt[Order - 1].y = xyrz.y;
			m_AvoidMotion[InsertNo - 1].pt[Order - 1].r = xyrz.r;
			m_AvoidMotion[InsertNo - 1].Height[Order - 1] = xyrz.z;
			m_AvoidMotion[InsertNo - 1].Count++;
            m_AvoidMotion[InsertNo - 1].Count = 0;
            for (int i = 0; i < MAX_AVOID_COUNT; i++)
            {
                if (m_AvoidMotion[InsertNo - 1].Use[i] == YES_USE)
                {
                    m_AvoidMotion[InsertNo - 1].Count++;
                }
            }
            if (Use == YES_USE)
            {
                TRACE(_T("[PWR] Avoid(%03d) Order:%d Use:%d XYRH,%.3f,%.3f,%.3f,%.3f Count:%d\n"),
                      InsertNo, Order, Use, xyrz.x, xyrz.y, xyrz.r, xyrz.z, m_AvoidMotion[InsertNo - 1].Count);
            }
		}
	}
	return Err;
}

void CReadJobFile::SetPartDrop(PART_DROP data)
{
	m_PartDrop = data;
	TRACE(_T("[PWR] SetPartDrop Use:%d Led:%d,%d,%d)\n"), m_PartDrop.Use, m_PartDrop.led.Top, m_PartDrop.led.Mid, m_PartDrop.led.Bot);

}

PART_DROP CReadJobFile::GetPartDrop()
{
	return m_PartDrop;
}

PRODUCTION CReadJobFile::GetProduction()
{
	PRODUCTION ProdInfo;
	ProdInfo = m_Production;
	return ProdInfo;
}

PCB CReadJobFile::GetPcb()
{
	PCB PcbInfo;
	PcbInfo = m_Pcb;
	return PcbInfo;
}

ORIGIN CReadJobFile::GetOrigin()
{
	ORIGIN Origin;
	Origin = m_Origin;
	return Origin;
}

ORIGIN CReadJobFile::GetBlockOrigin(long BlockNo)
{
	ORIGIN BlockOrigin;
	ZeroMemory(&BlockOrigin, sizeof(BlockOrigin));
	if (BlockNo > 0 && BlockNo <= MAXBLOCKNO)
	{
		BlockOrigin = m_BlockOrigin[BlockNo - 1];
	}
	return BlockOrigin;
}

FIDUCIAL CReadJobFile::GetMark()
{
	FIDUCIAL Mark;
	Mark = m_Mark;
	return Mark;
}

FIDUCIAL CReadJobFile::GetBlockMark(long BlockNo)
{
	FIDUCIAL BlockMark;
	ZeroMemory(&BlockMark, sizeof(BlockMark));
	if (BlockNo > 0 && BlockNo <= MAXBLOCKNO)
	{
		BlockMark = m_BlockMark[BlockNo - 1];
	}
	return BlockMark;
}

PICK CReadJobFile::GetPick(long FeederNo)
{
	PICK Pick;
	Pick.FeederNo = 0;
	Pick.NextFeedNo = 0;
	Pick.Use = 0;
	Pick.UseSimultaneous = 0;
	Pick.Offset.x = Pick.Offset.y = Pick.Offset.r = Pick.Offset.z = 0.0;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Pick = m_Pick[FeederNo - 1];
	}
	return Pick;
}

PACKAGE CReadJobFile::GetPackage(long PackageNo)
{
	PACKAGE Package;
	Package.Name.Format(_T("PwrPackage%03d"), PackageNo + 1);
	Package.Length = 10.0;
	Package.Width = 10.0;
	Package.Height = 10.0;
	Package.LeadHeight = 3.0;
	ZeroMemory(&Package.Led, sizeof(Package.Led));
	//Package.NozzleNo = 0;
	Package.PickDelay = TIME100MS;
	Package.BlowDelay = TIME100MS;
	Package.ReleaseDelay = TIME100MS;
	Package.Ratio.xy = Package.Ratio.r = Package.Ratio.z = 0;
	Package.PickRetry = 0;
	Package.RecogAngle = 0.0;
	Package.RecogOffsetHeight = 0.0;
	if (PackageNo > 0 && PackageNo <= MAXPACKAGENO)
	{
		Package = m_Package[PackageNo - 1];
	}
	return Package;
}

INSERT CReadJobFile::GetInsert(long InsertNo)
{
	INSERT Insert;
	Insert.index = 0;
	Insert.Step = 0;
	Insert.Use = 0;
	Insert.FeederNo = 0;
	Insert.PickOrder = 0;
	Insert.InsertOrder = 0;
	Insert.HeadNo = 0;
	Insert.pt.x = Insert.pt.y = Insert.pt.r = Insert.pt.z = 0.0;
	Insert.BlockNo = 0;
	Insert.NozzleNo = 0;
	Insert.RecogTable = FRONT_GANTRY;
	if (InsertNo > 0 && InsertNo <= MAXINSERTNO)
	{
		Insert = m_Insert[InsertNo - 1];
	}
	return Insert;
}

long CReadJobFile::GetInsertNo(long blockNo, long index)
{
	long insertNo = 0;
	for (int i = 0; i < MAXINSERTNO; i++)
	{
		if (m_Insert[i].BlockNo == blockNo && m_Insert[i].index == index)
		{
			insertNo = i + 1;
			break;
		}
	}

	return insertNo;
}

FEEDER CReadJobFile::GetFeeder(long FeederNo)
{
	FEEDER Feeder;
	Feeder.PackageName.Format(_T("PwrPackage%03d"), FeederNo + 1);
	Feeder.No = 0;
	Feeder.Type = 0;
	Feeder.TimeOut = TIME5000MS;
	Feeder.ReadyIONo = 0;
	Feeder.ReleaseIONo = 0;
	Feeder.ReadyIOWaitTime = TIME500MS;
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Feeder = m_Feeder[FeederNo - 1];
	}
	return Feeder;
}

MEASUREHEIGHT CReadJobFile::GetMeasureHeight(long order)
{
	long Err = NO_ERR;

	MEASUREHEIGHT temp;

	temp.Use = 0;

	if (order >= 0 && order < MAXINSERTNO)
	{
		temp = m_MeasureHeight[order];
	}

	return temp;
}

RETRY_LED CReadJobFile::GetRetryLed(long PackageNo)
{
	RETRY_LED RetryLed;
	RetryLed.PackageName.Format(_T("PwrPackage%03d"), PackageNo + 1);
	RetryLed.MaxRetry = 0;
	RetryLed.Led[0].Top = RetryLed.Led[0].Mid = RetryLed.Led[0].Bot = 0;
	RetryLed.Led[1].Top = RetryLed.Led[1].Mid = RetryLed.Led[1].Bot = 0;
	RetryLed.Led[2].Top = RetryLed.Led[2].Mid = RetryLed.Led[2].Bot = 0;
	RetryLed.Led[3].Top = RetryLed.Led[3].Mid = RetryLed.Led[3].Bot = 0;
	RetryLed.Led[4].Top = RetryLed.Led[4].Mid = RetryLed.Led[4].Bot = 0;
	if (PackageNo > 0 && PackageNo <= MAXPACKAGENO)
	{
		RetryLed = m_RetryLed[PackageNo - 1];
	}
	return RetryLed;
}

NOZZLE CReadJobFile::GetNozzle(long NozzleNo)
{
	NOZZLE Nozzle;
	ZeroMemory(&Nozzle, sizeof(Nozzle));
	if (NozzleNo > 0 && NozzleNo <= MAXNOZZLENO)
	{
		Nozzle = m_Nozzle[NozzleNo - 1];
	}
	return Nozzle;
}

DISCARD CReadJobFile::GetDiscard(long FeederNo)
{
	DISCARD Discard;
	ZeroMemory(&Discard, sizeof(Discard));
	if (FeederNo > 0 && FeederNo <= MAXFEEDERNO)
	{
		Discard = m_Discard[FeederNo - 1];
	}
	return Discard;
}

STANDBY CReadJobFile::GetStandyBy()
{
	STANDBY StandBy;
	ZeroMemory(&StandBy, sizeof(StandBy));
	StandBy = m_StandBy;
	TRACE(_T("[PWR] StandBy XYRZ(%.3f,%.3f,%.3f,%.3f)\n"), StandBy.pt.x, StandBy.pt.y, StandBy.pt.r, StandBy.pt.z);
	return StandBy;
}

double CReadJobFile::GetPickupZStandBy()
{
	double PickupZStandBy = 0.0;
	PickupZStandBy = m_PickupZStandby;
	TRACE(_T("[PWR] PickupZStandBy (%.3f)\n"), PickupZStandBy);
	return PickupZStandBy;
}

long CReadJobFile::CheckJobFileValid()
{
    //동일한 x y 좌표의 origin이 여러개 있으면 안된다. -> 수정된 ReadInsertFromOrigin 함수 동작 때문에.
    std::deque<ORIGIN> blockOrigins = std::deque<ORIGIN>{};
    for (int i = 1; i <= this->GetPcb().MaxBlockCount; i++)
    {
        const ORIGIN blockOrigin = GetBlockOrigin(i);
        for (int j = 0; j < blockOrigins.size(); j++)
        {
            if (
                blockOrigin.pt.x == blockOrigins.at(j).pt.x 
                && 
                blockOrigin.pt.y == blockOrigins.at(j).pt.y
                )
            {
                TRACE_FILE_FUNC_LINE_"early return due to Multiple block origins with the same x and y coordinates are not allowed.");
                return ERR_IDENTICAL_XY_BLOCK_ORIGIN_FOUND;
                break;
            }
            continue;
        }
        blockOrigins.push_back(blockOrigin);
        continue;
    }

	long Err = NO_ERR, Use = 0, PackageNo = 0, PackageIndex = 0, TrayNo = 0, TrayIndex = 0;
	CString PackageName, TrayName;
	long FeederType = 0;

	TRACE(_T("[PWR] CheckJobFileValid Start\n"));
	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; ++FeederNo)
	{
		Use = gcReadJobFile->GetPick(FeederNo + 1).Use;
		if (Use == 0)
		{
			continue;
		}
		PackageName = gcReadJobFile->GetFeeder(FeederNo + 1).PackageName;
		TRACE(_T("[PWR] CheckJobFileValid Fd:%d PackageName:%s Start Find\n"), FeederNo + 1, PackageName);
		for (PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
		{
			if (PackageName.CompareNoCase(gcReadJobFile->GetPackage(PackageNo).Name) == 0)
			{
				PackageIndex = PackageNo;
				TRACE(_T("[PWR] CheckJobFileValid Fd:%d PackageName:%s Found Index(%d)\n"), FeederNo + 1, PackageName, PackageIndex);
				break;
			}
		}
		if (PackageNo == MAXPACKAGENO)
		{
			return INVALID_PACKAGENAME(FeederNo + 1);
		}

		FeederType = gcReadJobFile->GetFeeder(FeederNo + 1).Type;
		if (IsTrayTypeFeeder(FeederType) == true || IsLabelTypeFeeder(FeederType) == true) // Tray
		{
			TrayName = gcReadJobFile->GetFeeder(FeederNo + 1).TrayName;
			TRACE(_T("[PWR] CheckJobFileValid Fd:%d TrayName:%s Start Find\n"), FeederNo + 1, TrayName);
			for (TrayNo = 0; TrayNo < MAXTRAYNO; ++TrayNo)
			{
				if (TrayName.CompareNoCase(gcReadJobFile->GetTray(TrayNo).Name) == 0)
				{
					TrayIndex = TrayNo;
					TRACE(_T("[PWR] CheckJobFileValid Fd:%d TrayName:%s Found Index(%d)\n"), FeederNo + 1, TrayName, TrayIndex);
					break;
				}
			}
			if (TrayNo == MAXPACKAGENO)
			{
				return INVALID_TRAYNAME(FeederNo + 1);
			}
		}

	}
	TRACE(_T("[PWR] CheckJobFileValid End Err:%d\n"), Err);
	return Err;
}

long CReadJobFile::GetPackgeIndexFromFeederNo(long FeederNo)
{
	long PackageIndex = 0;
	CString PackageName = GetFeeder(FeederNo).PackageName;


	for (long PackageNo = 0; PackageNo < MAXPACKAGENO; ++PackageNo)
	{
		if (PackageName.CompareNoCase(GetPackage(PackageNo).Name) == 0)
		{
			PackageIndex = PackageNo;
			break;
		}
	}

	return PackageIndex;
}

AVOIDMOTION CReadJobFile::GetAvoidMotion(long InsertNo)
{
	long Err = NO_ERR;
	AVOIDMOTION temp;
	temp.insertNo = InsertNo;
	temp.Count = 0;
	temp.LocateID.Format(_T("PwrLoc%04d"), InsertNo + 1);
	for (long index = 0; index < MAX_AVOID_COUNT; ++index)
	{
		temp.Order[index] = index;
		temp.pt[index].x = temp.pt[index].y = temp.pt[index].r = 0.0;
		temp.Height[index] = 10.0;
		temp.Use[index] = 0;
		temp.Timing[index] = 0;
	}
	if (InsertNo > 0 && InsertNo <= MAXINSERTNO)
	{
		temp = m_AvoidMotion[InsertNo - 1];
		//TRACE(_T("[PWR] GetAvoidMotion InsertNo:%d Count:%d\n"), InsertNo, m_AvoidMotion[InsertNo - 1].Count);
		for (long order = 0; order < MAX_AVOID_COUNT; ++order)
		{
			if (m_AvoidMotion[InsertNo - 1].Use[order] == 1)
			{
				TRACE(_T("[PWR] GetAvoidMotion Order:%d XYRH,%.3f,%.3f,%.3f,%.3f\n"), order + 1, m_AvoidMotion[InsertNo - 1].pt[order].x, m_AvoidMotion[InsertNo - 1].pt[order].y,
					m_AvoidMotion[InsertNo - 1].pt[order].r, m_AvoidMotion[InsertNo - 1].Height[order]);
			}
		}
	}
	return temp;
}

void CReadJobFile::InitAvoidMotion(long insertNo)
{
	for (long index = 0; index < MAX_AVOID_COUNT; ++index)
	{
		m_AvoidMotion[insertNo].Order[index] = index + 1;
		m_AvoidMotion[insertNo].LocateID.Format(_T("PwrLoc%04dOrd%02d"), insertNo + 1, index + 1);
		m_AvoidMotion[insertNo].insertNo = insertNo + 1;
		m_AvoidMotion[insertNo].Count = 0;
		m_AvoidMotion[insertNo].pt[index].x = m_AvoidMotion[insertNo].pt[index].y = m_AvoidMotion[insertNo].pt[index].r = 0.0;
		m_AvoidMotion[insertNo].Height[index] = 10.0;
		m_AvoidMotion[insertNo].Use[index] = 0;
		m_AvoidMotion[insertNo].Timing[index] = 0;
	}
}

TRAY_INFO CReadJobFile::GetTray(long FeederNo)
{
	TRAY_INFO Tray;
	Tray.Name.Format(_T("PwrTray%03d"), FeederNo + 1);
	Tray.MaxPocket = 1;
	for (long Count = 0; Count < MAXPOCKETNO; ++Count)
	{
		ZeroMemory(&Tray.pt[Count], sizeof(Tray.pt[Count]));
	}
	if (FeederNo > 0 && FeederNo <= MAXTRAYNO)
	{
		Tray = m_Tray[FeederNo - 1];
	}
	return Tray;
}


void CReadJobFile::SetDivideInspect(DIVIDE_INSPECT Data)
{
	long FeederNo = Data.FeederNo;
	CString strFunc(__func__);
	CString strLog;

	strLog.Empty();
	if (0 < FeederNo && FeederNo <= MAXFEEDERNO)
	{
		m_DividePackage[FeederNo - 1] = Data;
		strLog.Format(_T("[PWR] %s Feeder:%d Cnt:%d Package:%s"), (LPCTSTR)strFunc, Data.FeederNo, Data.Count, (LPCTSTR)Data.PackageName);

		for (long Cnt = 0; Cnt < Data.Count; Cnt++)
		{
			strLog.AppendFormat(_T("(%.3f %.3f)"), Data.Offset[Cnt].x, Data.Offset[Cnt].y);
		}

		TRACE(_T("%s \n"), strLog);
	}
}

void CReadJobFile::InitDivideInspect()
{
	for (long FeederNo = 0; FeederNo < MAXFEEDERNO; FeederNo++)
	{
		m_DividePackage[FeederNo].Use = false;
		m_DividePackage[FeederNo].FeederNo = FeederNo + 1;
		m_DividePackage[FeederNo].PackageName.Format(_T("D%d"), FeederNo + 1);
		m_DividePackage[FeederNo].Count = 0;

		for (long i = 0; i < DIVIDE_INSPECT_MAX_COUNT; i++)
		{
			m_DividePackage[FeederNo].Offset[i].x = 0.000;
			m_DividePackage[FeederNo].Offset[i].y = 0.000;
			m_DividePackage[FeederNo].Offset[i].r = 0.000;
			m_DividePackage[FeederNo].Offset[i].z = 0.000;
		}
	}
}

DIVIDE_INSPECT CReadJobFile::GetDivideInspect(long FeederNo)
{
	DIVIDE_INSPECT Data;

	if (0 < FeederNo && FeederNo <= MAXFEEDERNO)
	{
		Data = m_DividePackage[FeederNo - 1];
	}

	return Data;
}

bool CReadJobFile::Read_TRAY_DIRECT(CString fileName, CString trayName, TRAY_INFO* data)
{
	CString strFunc(__func__);
	CString strTemp;

	tinyxml2::XMLError ErrXML = tinyxml2::XMLError::XML_SUCCESS;
	tinyxml2::XMLDocument DocJobFile;
	tinyxml2::XMLElement* elementTray = NULL;

	bool readResult = false;
	long trayCnt = 0;

	TRACE(_T("[PWR] %s Start\n"), strFunc);

	ErrXML = DocJobFile.LoadFile(ToStdString(fileName).c_str());
	if (ErrXML != tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] %s Error(%d,%s)\n"), strFunc, ErrXML, GetXMLError(ErrXML));
		return readResult;
	}

	tinyxml2::XMLHandle docHandle(DocJobFile);


	strTemp = trayName;
	elementTray = docHandle.FirstChildElement("TrayFeederList").ToElement();
	if (elementTray == NULL)
	{
		TRACE(_T("[PWR] %s %s Null\n"), strFunc, strTemp);
		return readResult;
	}

	for (tinyxml2::XMLElement* elementTrayInfo = elementTray->FirstChildElement("TrayFeeder"); elementTrayInfo != 0; elementTrayInfo = elementTrayInfo->NextSiblingElement("TrayFeeder"))
	{
		if (elementTrayInfo->Attribute("TrayName", ToStdString(trayName).c_str()))
		{
			data->Name = trayName;
			for (tinyxml2::XMLElement* elementTrayPosition = elementTrayInfo->FirstChildElement("TrayPosition"); elementTrayPosition != 0; elementTrayPosition = elementTrayPosition->NextSiblingElement("TrayPosition"))
			{
				data->pt[trayCnt].x = elementTrayPosition->FindAttribute("OffsetX")->DoubleValue();
				data->pt[trayCnt].y = elementTrayPosition->FindAttribute("OffsetY")->DoubleValue();

				trayCnt++;
			}

			data->MaxPocket = trayCnt;

			TRACE(_T("[PWR] Tray Name:%s Count:%d\n"), data->Name, data->MaxPocket);

			for (size_t i = 0; i < trayCnt; i++)
			{
				TRACE(_T("[PWR] Tray XY:%.3f, %.3f\n"), data->pt[i].x, data->pt[i].y);

			}

			readResult = true;
			break;
		}
	}

	TRACE(_T("[PWR] %s Complete\n"), strFunc);

	return readResult;
}

std::string CReadJobFile::ToStdString(CString strText)
{
	std::string strTemp = CT2CA(strText);
	return strTemp;
}

CString CReadJobFile::GetXMLError(tinyxml2::XMLError ErrXML)
{
	CString strErr(tinyxml2::XMLDocument::ErrorIDToName(ErrXML));
	return strErr;
}

void CReadJobFile::SetBarcode(BARCODE Barcode)
{
	m_Barcode = Barcode;
	TRACE(_T("[PWR] SetBarcode Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use, Barcode.Type);
}

void CReadJobFile::SetBarcodeBlock1(BARCODE Barcode) // 20210415 HarkDo
{
	m_BarcodeBlock1 = Barcode;
	TRACE(_T("[PWR] SetBarcodeBlock1 Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use, Barcode.Type);
}

void CReadJobFile::SetBarcodeBlock2(BARCODE Barcode) // 20210415 HarkDo
{
	m_BarcodeBlock2 = Barcode;
	TRACE(_T("[PWR] SetBarcodeBlock2 Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d BarcodeType:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use, Barcode.Type);
}

BARCODE CReadJobFile::GetBarcode()
{
	BARCODE Barcode;
	Barcode = m_Barcode;
	TRACE(_T("[PWR] GetBarcode Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use);
	return Barcode;
}

long CReadJobFile::Read_BlockSkipHM(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	int iValue[20];
	double dValue[20];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	CStringArray* Data = new CStringArray;
	BLOCKSKIP_HM BlockSkip;
	bool BlockSkipUse = false;

	FindJobBlock(AllData, _T("[BLOCKSKIPINFO]"), _T("[EOF_BLOCKSKIPINFO]"), Data);

	if (Data->GetCount() > MAXINSERTNO)
	{
		TRACE("[PWR] Read_BlockSkipHM Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}

	for (long Count = 0; Count < Data->GetCount(); ++Count)
	{
		str = Data->GetAt(Count);

		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_BlockSkipHM(%d) TokenCount:%d\n", Count, cTokenizer->GetCount());
		}
		iCnt = dCnt = 0;
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);

			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}

		BlockSkip.BlockNo = iValue[0];
		BlockSkip.Use = iValue[1];
		BlockSkip.Pt.x = dValue[0];
		BlockSkip.Pt.y = dValue[1];
		BlockSkip.Height = dValue[2];
		BlockSkip.Tolerence = fabs(dValue[3]);

		SetBlockSkipHM(BlockSkip);

		if (BlockSkip.Use == 1)
		{
			BlockSkipUse = true;
		}

		delete cTokenizer;
		cTokenizer = NULL;
	}

	PRODUCTION Prod = GetProduction();
	if (BlockSkipUse == true && Prod.FirstPickingTiming == 1)
	{
		TRACE(_T("[PWR] SetBlockSkipHM FirstPicking 1->0 \n"));
		Prod.FirstPickingTiming = 0;
		SetProduction(Prod);
	}

	delete Data;
	return Err;
}

BARCODE CReadJobFile::GetBarcodeBlock1()
{
	BARCODE Barcode;
	Barcode = m_BarcodeBlock1;
	TRACE(_T("[PWR] GetBarcode Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d Type:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use, Barcode.Type);
	return Barcode;
}

BARCODE CReadJobFile::GetBarcodeBlock2()
{
	BARCODE Barcode;
	Barcode = m_BarcodeBlock2;
	TRACE(_T("[PWR] GetBarcode Use:%d XY:%.3f,%.3f Blue:%d Red:%d Mes.Use:%d Type:%d\n"), Barcode.Use, Barcode.pt[0].x, Barcode.pt[0].y, Barcode.Led.Blue, Barcode.Led.Red, Barcode.Mes.Use, Barcode.Type);
	return Barcode;
}

void CReadJobFile::SetBlockSkipHM(BLOCKSKIP_HM Data)
{
	long BlockNo = Data.BlockNo;

	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		m_BlockSkipHM[BlockNo - 1] = Data;
		TRACE(_T("[PWR] SetBlockSkipHM Block:%d Use:%d XY:%.3f,%.3f Height:%.3f Tol:%.3f\n"), Data.BlockNo, Data.Use, Data.Pt.x, Data.Pt.y, Data.Height, Data.Tolerence);
	}
}

BLOCKSKIP_HM CReadJobFile::GetBlockSkipHM(long BlockNo)
{
	BLOCKSKIP_HM data;
	ZeroMemory(&data, sizeof(data));

	if (0 < BlockNo && BlockNo <= MAXBLOCKNO)
	{
		data = m_BlockSkipHM[BlockNo - 1];
	}

	return data;
}


void CReadJobFile::InitForming(long index)
{
	ZeroMemory(&m_Forming[index], sizeof(m_Forming[index]));
}

long CReadJobFile::Read_FORMING(CStringArray* AllData)
{
	long Err = NO_ERR;
	BOOL bRet = false, Find = false;
	CString str;
	PRODUCTION Prod = GetProduction();
	int iValue[20];
	double dValue[100];
	long iCnt = 0, dCnt = 0, FeederNo = 0;
	CString strValue, strTemp;
	TRAY_INFO Tray;
	CStringArray* Data = new CStringArray;

	FindJobBlock(AllData, _T("[FORMING]"), _T("[EOF_FORMING]"), Data);

	if (Data->GetCount() > MAXFEEDERNO)
	{
		TRACE("[PWR] Read_FORMING Invalid Data Count:%d\n", Data->GetCount());
		delete Data;
		return JOBFILE_READSTRING_FAIL;
	}
	Point_XYRZ xyrz;
	ZeroMemory(&xyrz, sizeof(xyrz));
	for (long FeederCount = 0; FeederCount < Data->GetCount(); FeederCount++)
	{
		str = Data->GetAt(FeederCount);
		CTokenizer* cTokenizer = new CTokenizer(str, _T(","), FALSE);
		ASSERT(cTokenizer->GetCount() > 0);
		if (gcPowerLog->IsShowCommunicationLog() == true)
		{
			TRACE("[PWR] CReadJobFile Read_FORMING(%d) TokenCount:%d\n", FeederCount, cTokenizer->GetCount());
		}
		ZeroMemory(&iValue, sizeof(iValue));
		ZeroMemory(&dValue, sizeof(dValue));
		iCnt = dCnt = 0;

		for (int i = 0; i < cTokenizer->GetCount(); i++)
		{
			strValue = cTokenizer->GetString(i);
			if (strValue.Find(_T(".")) >= 0)
			{
				dValue[dCnt] = cTokenizer->GetDouble(i);
				dCnt++;
			}
			else
			{
				iValue[iCnt] = cTokenizer->GetInt(i);
				iCnt++;
			}
		}
		FORMING_COMPONENT Forming;
		ZeroMemory(&Forming, sizeof(Forming));
		Forming.Use = iValue[1];
		Forming.FeederNo = iValue[0];
		Forming.Delay = TIME300MS;
		Forming.InsertCase.Min = dValue[0];
		Forming.InsertCase.Max = dValue[1];
		Forming.ptNo[0].x = dValue[2];
		Forming.ptNo[0].y = dValue[3];
		Forming.ptNo[0].r = dValue[4];
		Forming.ptNo[0].z = dValue[5];
		Forming.CaseNo[0].Min = dValue[6];
		Forming.CaseNo[0].Max = dValue[7];
		Forming.ptNo[1].x = dValue[8];
		Forming.ptNo[1].y = dValue[9];
		Forming.ptNo[1].r = dValue[10];
		Forming.ptNo[1].z = dValue[11];
		Forming.CaseNo[1].Min = dValue[12];
		Forming.CaseNo[1].Max = dValue[13];
		SetForming(Forming.FeederNo, Forming);
		delete cTokenizer;
		cTokenizer = NULL;
	}
	delete Data;
	return Err;
}

long CReadJobFile::SetForming(long FeederNo, FORMING_COMPONENT Forming)
{
	long Err = NO_ERR;
	if (FeederNo > 0)
	{
		m_Forming[FeederNo - 1] = Forming;
		//if (Forming.Use == 1)
		{
			TRACE(_T("[PWR] FormingUse:%d FdNo:%d InsertCase:%.3f~%.3f Case1:%.3f~%.3f Case2:%.3f~%.3f Pt1:%.3f,%.3f,%.3f,%.3f Pt2:%.3f,%.3f,%.3f,%.3f\n"),
				Forming.Use, Forming.FeederNo,
				Forming.InsertCase.Min, Forming.InsertCase.Max,
				Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
				Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z);
		}
	}
	return Err;
}

FORMING_COMPONENT CReadJobFile::GetForming(long FeederNo)
{
	FORMING_COMPONENT Forming;
	ZeroMemory(&Forming, sizeof(Forming));
	if (FeederNo > 0)
	{
		Forming = m_Forming[FeederNo - 1];
		//if (Forming.Use == 1)
		{
			TRACE(_T("[PWR] GetForming:%d FdNo:%d InsertCase:%.3f~%.3f Case1:%.3f~%.3f Case2:%.3f~%.3f Pt1:%.3f,%.3f,%.3f,%.3f Pt2:%.3f,%.3f,%.3f,%.3f\n"),
				Forming.Use, Forming.FeederNo,
				Forming.InsertCase.Min, Forming.InsertCase.Max,
				Forming.CaseNo[0].Min, Forming.CaseNo[0].Max,
				Forming.CaseNo[1].Min, Forming.CaseNo[1].Max,
				Forming.ptNo[0].x, Forming.ptNo[0].y, Forming.ptNo[0].r, Forming.ptNo[0].z,
				Forming.ptNo[1].x, Forming.ptNo[1].y, Forming.ptNo[1].r, Forming.ptNo[1].z);
		}
	}
	return Forming;
}