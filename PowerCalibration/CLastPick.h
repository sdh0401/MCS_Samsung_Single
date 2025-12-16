#pragma once
#include "GlobalDefine.h"
#include <array>
#include <mutex>

class CLastPick
{
public:
    class DiscardInfo
    {
    private:
        const long feederNumber;
        const Point_XYRZ discardPoint;
    public:
        DiscardInfo(const long& feederNumber, const Point_XYRZ& discardPoint);
        long getFeederNumber() const;
        Point_XYRZ getDiscardPoint(const bool& trace = false) const;
    };
    static long updateDiscardPoint(const long& feederNumber, const Point_XYRZ& discardPoint);
    static long initializeDiscardInfo();
    /// <summary>
    /// <para>단순히 멤버 변수에 접근해서 새로 할당하는 동작. SETTER.</para>
    /// </summary>
    /// <param name="headNumber">1~6</param>
    /// <param name="saveTextFile">true면 내부적으로 saveDiscardInfoIntoTextFile 호출.</param>
    long setDiscardInfo(const size_t& headNumber, const DiscardInfo& discardInfo, const bool saveTextFile = false);
    /// <summary>
    /// <para>단순히 멤버 변수에 접근해서 읽어오는 동작. GETTER.</para>
    /// </summary>
    /// <param name="headNumber">1~6</param>
    DiscardInfo getDiscardInfo(const size_t& headNumber) const;
private:

    static std::mutex runtimeDiscardInfoFileMutex;
    static long initalizeDiscardInfoTextFile();
    static constexpr bool isValidHeadNumber(const size_t& headNumber);
    static constexpr bool isValidFeederNumber(const size_t& feederNumber);
    std::array<DiscardInfo*, 6> discardInfos;
    /// <summary>
    /// <para>GETTER 를 통해 멤버변수에 접근해서 파일에 쓰는 동작.</para>
    /// 실패시 내부에서 SendAlarm.
    /// </summary>
    static long saveDiscardInfoIntoTextFile();
    /// <summary>
    /// 파일에서 읽어서 내부적으로 SETTER 호출하는 동작.
    /// </summary>
    /// <returns></returns>
    static long loadDiscardInfoFromTextFile();
    static constexpr const char* RUNTIME_DISCARD_INFO_FULL_PATH
        = "C:/Power/i6.0/MCS/MachineConfig/RuntimeDiscardInfo.txt";
    static constexpr const char* STRING_FRONT_GANTRY = "*****[FRONT_GANTRY]*****";
    static constexpr const char* STRING_REAR_GANTRY = "*****[REAR_GANTRY]*****";
    static constexpr const char* STRING_HEAD_NUMBER = "[HEAD_NUMBER]";
    static constexpr const char* STRING_FEEDER_NUBMER = "[FEEDER_NUMBER]";
    static constexpr const char* STRING_DISCARD_POSITION = "[DISCARD_POSITION]";
    static constexpr const int RUNTIME_DISCARD_INFO_TEXT_LINE = 41;//싱글갠트리는 적어도 41줄은 있어야 정상적인 파일이다.

public:
    CLastPick(const long& m_Gantry);
    ~CLastPick();
    LASTPICK DefaultHeadData(long headNo = 0);
    void ClearAllHeadData();
    void ClearHeadData(long headNo);
    bool IsSameHeadData(LASTPICK data1, LASTPICK data2);
    void ShowHead(long headNo);
    void ShowAll();
    void SetHeadData(long headNo, LASTPICK data);
    LASTPICK GetHeadData(long headNo);

    void SetHeadDataExist(long headNo, bool exist);
    void SetHeadDataEnable(long headNo, bool enable);

    void SetAllHeadDataEnable(bool enable);

    void SetPcbName(CString name);
    CString GetPcbName();
    double GetMaxHeight();
private:
    LASTPICK m_HeadData[MAXUSEDHEADNO];
    CString m_PCBName;
    const long m_Gantry;
};

extern CLastPick* gcLastPickFront;