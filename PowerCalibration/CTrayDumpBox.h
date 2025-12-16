#pragma once
#include "GlobalDefine.h"

class CTrayDumpBox
{
public:
    CTrayDumpBox();

    /// <summary>
    /// 
    /// </summary>
    /// <param name="DumpNo">0 또는 1만 받는다</param>
    /// <param name="Data"></param>
    /// <returns>성공시 0</returns>
    long SetBoxInfo(long DumpNo, DUMP_TRAY Data);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="DumpNo">0 또는 1만 받는다</param>
    /// <param name="Data"></param>
    /// <returns>성공시 0</returns>
    long GetBoxInfo(long DumpNo, DUMP_TRAY& Data);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="dumpNo"></param>
    /// <param name="lastPocket"></param>
    /// <returns></returns>
    long sendTrayDumpBoxLastPocket(long dumpNo, long lastPocket);

private:
    DUMP_TRAY m_dumpbox[2];

};
extern CTrayDumpBox* gcTrayDumpBox;
