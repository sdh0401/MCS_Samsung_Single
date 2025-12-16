#include "pch.h"
#include "CLastPick.h"
#include "GlobalData.h"
#include "AxisInformation.h"
#include "Trace.h"

CLastPick::DiscardInfo::DiscardInfo(const long& feederNumber, const Point_XYRZ& discardPoint)
    : feederNumber{ feederNumber }
    , discardPoint{ discardPoint } {}

long CLastPick::DiscardInfo::getFeederNumber() const
{
    return this->feederNumber;
}

Point_XYRZ CLastPick::DiscardInfo::getDiscardPoint(const bool& trace) const
{
    if (trace == true)
    {
        CString temp; temp.Format(L"feederNumber: %d, XYRZ: %.3f %.3f %.3f %.3f", this->getFeederNumber(), this->discardPoint.x, this->discardPoint.y, this->discardPoint.r, this->discardPoint.z);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
    }
    return this->discardPoint;
}

long CLastPick::updateDiscardPoint(const long& feederNumber, const Point_XYRZ& discardPoint)
{
    for (size_t i = 0; i < 6; i++)
    {
        if (gcLastPickFront->discardInfos[i]->getFeederNumber() == feederNumber)
        {
            (void)gcLastPickFront->setDiscardInfo(i + 1, CLastPick::DiscardInfo{ feederNumber, discardPoint }, true);
        }
        //if (gcLastPickRear->discardInfos[i]->getFeederNumber() == feederNumber)
        //{
        //    (void)gcLastPickRear->setDiscardInfo(i + 1, CLastPick::DiscardInfo{ feederNumber, discardPoint }, true);
        //}
    }
    return 0;
}

long CLastPick::initializeDiscardInfo()
{
    if (CLastPick::initalizeDiscardInfoTextFile() != NO_ERR)
    {
        TRACE_FILE_FUNC_LINE_"(CLastPick::initalizeDiscardInfoTextFile() != NO_ERR).");
        return SAVE_FAIL_MACHINE_CONFIG;
    }
    if (CLastPick::loadDiscardInfoFromTextFile() != NO_ERR)
    {
        TRACE_FILE_FUNC_LINE_"(CLastPick::loadDiscardInfoFromTextFile() != NO_ERR).");
        return READ_FAIL_MACHINE_CONFIG;
    }
    return 0;
}

long CLastPick::setDiscardInfo(const size_t& headNumber, const DiscardInfo& discardInfo, const bool saveTextFile)
{
    //아예 동일하면 생략가능해..
    const bool isDiscardInfoSame
        =
        ( discardInfo.getFeederNumber()     == this->discardInfos[headNumber - 1]->getFeederNumber()    )
        &&
        ( discardInfo.getDiscardPoint().x   == this->discardInfos[headNumber - 1]->getDiscardPoint().x  )
        &&
        ( discardInfo.getDiscardPoint().y   == this->discardInfos[headNumber - 1]->getDiscardPoint().y  )
        &&
        ( discardInfo.getDiscardPoint().r   == this->discardInfos[headNumber - 1]->getDiscardPoint().r  )
        &&
        ( discardInfo.getDiscardPoint().z   == this->discardInfos[headNumber - 1]->getDiscardPoint().z  );

    if (isDiscardInfoSame == true)
    {
        CString temp; temp.Format(L"early return due to same discardInfo. gantry: %s, head: %llu, feeder: %03d, discard: %.3f %.3f %.3f %.3f"
                                  , (this->m_Gantry == FRONT_GANTRY) ? L"FRONT" : L" REAR", headNumber, discardInfo.getFeederNumber()
                                  , discardInfo.getDiscardPoint().x, discardInfo.getDiscardPoint().y, discardInfo.getDiscardPoint().r, discardInfo.getDiscardPoint().z);
        TRACE_FILE_FUNC_LINE_(CStringA)temp);
        return 0;
    }

    CString temp;
    temp.Format(L"gantry: %s, head: %llu, feeder: %03d, discard: %.3f %.3f %.3f %.3f, saveTextFile: %s"
                , (this->m_Gantry == FRONT_GANTRY) ? L"FRONT" : L" REAR", headNumber, discardInfo.getFeederNumber()
                , discardInfo.getDiscardPoint().x, discardInfo.getDiscardPoint().y, discardInfo.getDiscardPoint().r, discardInfo.getDiscardPoint().z
                , (saveTextFile == true) ? L" TRUE" : L"FALSE");
    TRACE_FILE_FUNC_LINE_(CStringA)temp);

    if (CLastPick::isValidHeadNumber(headNumber) == false)
    {
        TRACE_FILE_FUNC_LINE_"early return due to invalid head number !!!!!");
        return INVALID_HEADNO;
    }

    delete this->discardInfos[headNumber - 1];
    this->discardInfos[headNumber - 1] = NULL;
    this->discardInfos[headNumber - 1] = new DiscardInfo{ discardInfo.getFeederNumber(), discardInfo.getDiscardPoint() };

    long Err = (saveTextFile == true) ? CLastPick::saveDiscardInfoIntoTextFile() : NO_ERR;//이거 여기 있으면 안된다? -> 옵션화 처리.
    if (Err != NO_ERR)
    {
        TRACE_FILE_FUNC_LINE_"this->saveDiscardInfoIntoTextFile() != NO_ERR");
        return Err;
    }

    return 0;
}

CLastPick::DiscardInfo CLastPick::getDiscardInfo(const size_t& headNumber) const
{
    if (CLastPick::isValidHeadNumber(headNumber) == false)
    {
        return *this->discardInfos[0];
    }
    return *this->discardInfos[headNumber - 1];
}

std::mutex CLastPick::runtimeDiscardInfoFileMutex = std::mutex{};

long CLastPick::initalizeDiscardInfoTextFile()
{
    CStdioFile cStdioFile;
    if (cStdioFile.Open(CString(CLastPick::RUNTIME_DISCARD_INFO_FULL_PATH), CStdioFile::OpenFlags::modeRead) == TRUE)//이미 해당 파일이 있으면 초기화가 필요 없음.
    {
        TRACE_FILE_FUNC_LINE_"early return due to cStdioFile.Open(.., CStdioFile::OpenFlags::modeRead) == TRUE.");
        return NO_ERR;
    }

    if (cStdioFile.Open(CString(CLastPick::RUNTIME_DISCARD_INFO_FULL_PATH), CStdioFile::OpenFlags::modeCreate/* | CStdioFile::OpenFlags::modeWrite*/) == FALSE)
    {
        TRACE_FILE_FUNC_LINE_"early return due to cStdioFile.Open(.., CStdioFile::OpenFlags::modeCreate) == FALSE.");
        return SAVE_FAIL_MACHINE_CONFIG;
    }

    cStdioFile.Close();
    return CLastPick::saveDiscardInfoIntoTextFile();
}

constexpr bool CLastPick::isValidHeadNumber(const size_t& headNumber)
{
    if (1 <= headNumber && headNumber <= MAXUSEDHEADNO)
    {
        return true;
    }
    TRACE_FILE_FUNC_LINE_"returns false.");
    return false;
}

constexpr bool CLastPick::isValidFeederNumber(const size_t& feederNumber)
{
    if (1 <= feederNumber && feederNumber <= MAXFEEDERNO)
    {
        return true;
    }
    TRACE_FILE_FUNC_LINE_"returns false.");
    return false;
}

long CLastPick::saveDiscardInfoIntoTextFile()
{
    std::lock_guard<std::mutex> lockGuard = std::lock_guard<std::mutex>{ CLastPick::runtimeDiscardInfoFileMutex };//동시접근 방지.

    CStdioFile cStdioFile{};
    if (cStdioFile.Open(CString(CLastPick::RUNTIME_DISCARD_INFO_FULL_PATH), CStdioFile::OpenFlags::modeWrite) == FALSE)
    {
        TRACE_FILE_FUNC_LINE_"cStdioFile.Open(..) == FALSE.");
        long Err = SAVE_FAIL_MACHINE_CONFIG;
        Err = SendAlarm(Err, L"file open failed..");
        return SAVE_FAIL_MACHINE_CONFIG;
    }

    for (int j = 0; j < 1; j++)//싱글갠트리니깐 0 한번만 한다. 듀얼때만 필요했던 반복문,,
    {
        const CLastPick* const currentLastPick = gcLastPickFront;
        gcLastPickFront->ClearAllHeadData();
        gcLastPickFront->SetHeadData(1, LASTPICK{});

        for (int i = 1; i <= MAXUSEDHEADNO; i++)//1~6
        {
            CString headNumber{}, feederNumber{}, discardPosition{};
            headNumber.Format(L"%d", i);
            feederNumber.Format(L"%d", currentLastPick->getDiscardInfo(i).getFeederNumber());
            discardPosition.Format(L"%.3f,%.3f,%.3f,%.3f"
                                   , currentLastPick->getDiscardInfo(i).getDiscardPoint().x
                                   , currentLastPick->getDiscardInfo(i).getDiscardPoint().y
                                   , currentLastPick->getDiscardInfo(i).getDiscardPoint().r
                                   , currentLastPick->getDiscardInfo(i).getDiscardPoint().z
            );

            cStdioFile.WriteString(CString(CLastPick::STRING_HEAD_NUMBER));
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(headNumber);
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(CString(CLastPick::STRING_FEEDER_NUBMER));
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(feederNumber);
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(CString(CLastPick::STRING_DISCARD_POSITION));
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(discardPosition);
            cStdioFile.WriteString(CString("\n"));
            cStdioFile.WriteString(CString("\n"));
        }
    }

    cStdioFile.Close();
    return 0;
}

long CLastPick::loadDiscardInfoFromTextFile()
{
    std::lock_guard<std::mutex> lockGuard = std::lock_guard<std::mutex>{ CLastPick::runtimeDiscardInfoFileMutex };//동시접근 방지.

    long Err = NO_ERR;

    CStdioFile cStdioFile{};
    if (cStdioFile.Open(CString(CLastPick::RUNTIME_DISCARD_INFO_FULL_PATH), CStdioFile::OpenFlags::modeRead) == FALSE)
    {
        Err = READ_FAIL_MACHINE_CONFIG;
        Err = SendAlarm(Err, L"cStdioFile.Open(..) == FALSE");
        return Err;
    }

    int readLineCount = 0;
    bool isRearGantry = false;
    bool isHeadNumberOnNextLine = false;
    bool isFeederNumberOnNextLine = false;
    bool isDiscardPositionOnNextLine = false;
    long currentGantry = FRONT_GANTRY;
    long currentHeadNumber = 1;
    long currentFeederNumber = 1;

    CLastPick* const currentCLastPick = gcLastPickFront;//12.0 에는 없었는데 여기에만 추가.

    while (true)
    {
        CString readStringResult{};
        if (cStdioFile.ReadString(readStringResult) == FALSE)
        {
            if (readLineCount < CLastPick::RUNTIME_DISCARD_INFO_TEXT_LINE)
            {
                TRACE_FILE_FUNC_LINE_"(readLineCount < CLastPick::RUNTIME_DISCARD_INFO_TEXT_LINE).");
                Err = READ_FAIL_MACHINE_CONFIG;
                Err = SendAlarm(Err, L"readLineCount is not enough..");
                cStdioFile.Close();
                return Err;
                break;
            }
            break;
        }
        readLineCount++;

        if (readStringResult.Compare(CString{CLastPick::STRING_REAR_GANTRY}) == 0)//싱글 갠트리라 여기 걸릴일 없음..
        {
            isRearGantry = true;
            currentGantry = REAR_GANTRY;
            continue;
        }

        if (readStringResult.Compare(CString{CLastPick::STRING_HEAD_NUMBER}) == 0)
        {
            isHeadNumberOnNextLine = true;
            continue;
        }

        if (isHeadNumberOnNextLine == true)
        {
            isHeadNumberOnNextLine = false;

            const int headNumber = _ttoi(readStringResult);
            if (CLastPick::isValidHeadNumber(headNumber) == false)
            {
                CString temp; temp.Format(L"invalid head number! headNumber: %d, readLineCount: %d, readStringResult: %s", headNumber, readLineCount, (LPCTSTR)readStringResult);
                TRACE_FILE_FUNC_LINE_(CString)temp);
                Err = READ_FAIL_MACHINE_CONFIG;
                Err = SendAlarm(Err, L"invalid head number..");
                cStdioFile.Close();
                return Err;
                break;
            }

            currentHeadNumber = headNumber;
            continue;
        }

        if (readStringResult.Compare(CString{CLastPick::STRING_FEEDER_NUBMER}) == 0)
        {
            isFeederNumberOnNextLine = true;
            continue;
        }

        if (isFeederNumberOnNextLine == true)
        {
            isFeederNumberOnNextLine = false;

            const int feederNumber = _ttoi(readStringResult);
            if (CLastPick::isValidFeederNumber(feederNumber) == false)
            {
                CString temp; temp.Format(L"invalid feederNumber number! feederNumber: %d, readLineCount: %d, readStringResult: %s", feederNumber, readLineCount, (LPCTSTR)readStringResult);
                TRACE_FILE_FUNC_LINE_(CStringA)temp);
                Err = READ_FAIL_MACHINE_CONFIG;
                Err = SendAlarm(Err, L"invalid feeder Number..");
                cStdioFile.Close();
                return Err;
                break;
            }

            currentFeederNumber = feederNumber;
            continue;
        }

        if (readStringResult.Compare(CString{CLastPick::STRING_DISCARD_POSITION}) == 0)
        {
            isDiscardPositionOnNextLine = true;
            continue;
        }

        if (isDiscardPositionOnNextLine == true)
        {
            isDiscardPositionOnNextLine = false;

            int iStart = 0;
            std::deque<CString> tokens = std::deque<CString>{};
            while (true)
            {
                const CString tokenizeResult = readStringResult.Tokenize(L",", iStart);

                if (tokenizeResult.IsEmpty() == true)
                {
                    break;
                }

                tokens.push_back(tokenizeResult);

                continue;
            }

            if (tokens.size() != 4)//반드시 XYRZ 4개이어야..
            {
                CString allTokens{};
                for (int i = 0; i < tokens.size(); i++)
                {
                    allTokens.AppendFormat(L"%s ", (LPCTSTR)tokens.at(i));
                }
                CString temp; temp.Format(L"invalid discardPosition token count! allTokens: %s, tokens.size(): %llu readLineCount: %d, readStringResult: %s", (LPCTSTR)allTokens, tokens.size(), readLineCount, (LPCTSTR)readStringResult);
                TRACE_FILE_FUNC_LINE_(CString)temp);
                Err = READ_FAIL_MACHINE_CONFIG;
                Err = SendAlarm(Err, L"invalid discard position..");
                cStdioFile.Close();
                return Err;
                break;
            }

            const double discardPositionX = _wtof(tokens.at(0));
            const double discardPositionY = _wtof(tokens.at(1));
            const double discardPositionR = _wtof(tokens.at(2));
            const double discardPositionZ = _wtof(tokens.at(3));

            //파일에서 읽은거라 파일에 다시 저장할 필요 없다.
            (void)currentCLastPick->setDiscardInfo(currentHeadNumber, CLastPick::DiscardInfo{ currentFeederNumber, Point_XYRZ{ discardPositionX, discardPositionY, discardPositionR, discardPositionZ } }, false);

            continue;
        }

        continue;
    }//while(true)

    cStdioFile.Close();
    return 0;
}

CLastPick* gcLastPickFront;

CLastPick::CLastPick(const long& m_Gantry)
    : discardInfos{
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
    new DiscardInfo{ 1, Point_XYRZ{ 0.000, 0.000, 0.000, 0.000 } },
}
, m_Gantry{ m_Gantry }
{
    ClearAllHeadData();
    m_PCBName.Empty();
}

CLastPick::~CLastPick()
{
}

LASTPICK CLastPick::DefaultHeadData(long headNo)
{
    LASTPICK data;

    data.Enable = false;
    data.SetTime = _time_get();

    data.Insert.index = 0;
    data.Insert.Step = 0;
    data.Insert.Use = 0;
    data.Insert.FeederNo = 0;
    data.Insert.PickOrder = 0;
    data.Insert.InsertOrder = 0;
    data.Insert.HeadNo = headNo;
    data.Insert.pt.x = 0.000;
    data.Insert.pt.y = 0.000;
    data.Insert.pt.r = 0.000;
    data.Insert.pt.z = 0.000;
    data.Insert.BlockNo = 0;
    data.Insert.NozzleNo = 0;
    data.Insert.RecogTable = 0;
    data.Insert.InsertOffset = 0.000;

    data.Package.Name.Empty();

    return data;
}

void CLastPick::ClearAllHeadData()
{
    for (long headNo = 1; headNo <= MAXUSEDHEADNO; headNo++)
    {
        ClearHeadData(headNo);
    }	
}

void CLastPick::ClearHeadData(long headNo)
{
    LASTPICK data = DefaultHeadData(headNo);
    SetHeadData(headNo, data);
}

bool CLastPick::IsSameHeadData(LASTPICK data1, LASTPICK data2)
{
    //if (data1.Enable != data2.Enable) return false;
    //if (data1.SetTime != data2.SetTime) return false;

    if (data1.Insert.index != data2.Insert.index) return false;
    if (data1.Insert.Step != data2.Insert.Step) return false;
    if (data1.Insert.Use != data2.Insert.Use) return false;
    if (data1.Insert.FeederNo != data2.Insert.FeederNo) return false;
    if (data1.Insert.PickOrder != data2.Insert.PickOrder) return false;
    if (data1.Insert.InsertOrder != data2.Insert.InsertOrder) return false;
    if (data1.Insert.HeadNo != data2.Insert.HeadNo) return false;
    //if (data1.Insert.pt.x != data2.Insert.pt.x) return false;
    //if (data1.Insert.pt.y != data2.Insert.pt.y) return false;
    //if (data1.Insert.pt.r != data2.Insert.pt.r) return false;
    //if (data1.Insert.pt.z != data2.Insert.pt.z) return false;
    if (data1.Insert.BlockNo != data2.Insert.BlockNo) return false;
    if (data1.Insert.NozzleNo != data2.Insert.NozzleNo) return false;
    //if (data1.Insert.RecogTable != data2.Insert.RecogTable) return false;
    //if (data1.Insert.InsertOffset != data2.Insert.InsertOffset) return false;

    return true;
}

void CLastPick::ShowHead(long headNo)
{
    LASTPICK data = GetHeadData(headNo);

    TRACE(_T("[PWR] LastPick Head %d Enable %d index %d Feeder %d Nozzle %d Package %s\n"), 
          data.Insert.HeadNo, data.Enable, data.Insert.index, data.Insert.FeederNo, data.Insert.NozzleNo, data.Package.Name);
}

void CLastPick::ShowAll()
{
    TRACE(_T("[PWR] LastPick PcbName(%s)\n"), m_PCBName);

    for (long headNo = 1; headNo <= MAXUSEDHEADNO; headNo++)
    {
        ShowHead(headNo);
    }
}

void CLastPick::SetHeadData(long headNo, LASTPICK data)
{
    if (headNo != data.Insert.HeadNo || headNo < 1 || headNo > MAXUSEDHEADNO)
    {
        TRACE(_T("[PWR] LastPick SetHeadData Invalid Head (%d, %d)\n"), headNo, data.Insert.HeadNo);
        return;
    }

    bool same = IsSameHeadData(m_HeadData[headNo - 1], data);
    m_HeadData[headNo - 1] = data;
    m_HeadData[headNo - 1].SetTime = _time_get();

    if (same == false)
    {
        ShowHead(headNo);
    }
}

LASTPICK CLastPick::GetHeadData(long headNo)
{
    LASTPICK data = DefaultHeadData(headNo - 1);

    if (headNo < 1 || headNo > MAXUSEDHEADNO)
    {
        TRACE(_T("[PWR] LastPick SetHeadData Invalid Head (%d)\n"), headNo);
        return data;
    }

    data = m_HeadData[headNo - 1];

    if (headNo != data.Insert.HeadNo)
    {
        TRACE(_T("[PWR] LastPick SetHeadData Invalid Head (%d, %d)\n"), headNo, data.Insert.HeadNo);

        data = DefaultHeadData(headNo - 1);
        return data;
    }

    return data;
}

void CLastPick::SetHeadDataExist(long headNo, bool exist)
{
    CString strFunc(__func__);

    if (headNo < 1 || headNo > MAXUSEDHEADNO)
    {
        TRACE(_T("[PWR] LastPick %s Invalid Head(%d)\n"), strFunc, headNo);
        return;
    }

    if (m_HeadData[headNo - 1].Exist != exist)
    {
        TRACE(_T("[PWR] LastPick %s Head:%d Exist:%d->%d\n"), strFunc, headNo, m_HeadData[headNo - 1].Exist, exist);
    }

    m_HeadData[headNo - 1].Exist = exist;

    if (exist == false && m_HeadData[headNo - 1].Enable == true)
    {
        m_HeadData[headNo - 1].Enable = false;
        TRACE(_T("[PWR] LastPick %s Head:%d Enable:%d->%d\n"), strFunc, headNo, m_HeadData[headNo - 1].Enable, exist);
    }
}

void CLastPick::SetHeadDataEnable(long headNo, bool enable)
{
    CString strFunc(__func__);

    if (headNo < 1 || headNo > MAXUSEDHEADNO)
    {
        TRACE(_T("[PWR] LastPick %s Invalid Head(%d)\n"), strFunc, headNo);
        return;
    }

    if (m_HeadData[headNo - 1].Enable != enable)
    {
        TRACE(_T("[PWR] LastPick %s Head:%d Enable:%d->%d\n"), strFunc, headNo, m_HeadData[headNo - 1].Enable, enable);
    }

    m_HeadData[headNo - 1].Enable = enable;
}

void CLastPick::SetAllHeadDataEnable(bool enable)
{
    for (long headNo = 1; headNo <= MAXUSEDHEADNO; headNo++)
    {
        SetHeadDataEnable(headNo, enable);
    }
}

void CLastPick::SetPcbName(CString name)
{
    if (m_PCBName.CompareNoCase(name) != 0)
    {
        TRACE(_T("[PWR] LastPick SetPcbName (%s -> %s)\n"), m_PCBName, name);
    }

    m_PCBName = name;
}

CString CLastPick::GetPcbName()
{
    return m_PCBName;
}

double CLastPick::GetMaxHeight()
{
    double max = 0.000;
    double height;

    for (long head = 1; head <= MAXUSEDHEADNO; head++)
    {
        if (GetHeadData(head).Enable == false) continue;

        height = GetHeadData(head).Package.Height + GetHeadData(head).Package.LeadHeight;
        if (max < height)
        {
            max = height;
        }
    }

    if (max > 0.001)
    {
        TRACE(_T("[PWR] LastPick GetMaxHeight %.3f\n"), max);
    }

    return max;
}
