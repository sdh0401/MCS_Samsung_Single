#include "pch.h"
#include "CTrayDumpBox.h"
#include "Trace.h"
#include <typeinfo>
#include "AxisInformation.h"

CTrayDumpBox* gcTrayDumpBox;

#define TRACE_CLASS_AND_METHOD_AND_THIS_    TRACE("[PWR] %s::%s -> %s", typeid(CTrayDumpBox).name(), __func__, 

CTrayDumpBox::CTrayDumpBox()
    : m_dumpbox{ DUMP_TRAY(), DUMP_TRAY() }
{
    return;
}

long CTrayDumpBox::SetBoxInfo(long DumpNo, DUMP_TRAY Data)
{
    if (DumpNo != 0 && DumpNo != 1)
    {
        TRACE_CLASS_AND_METHOD_AND_THIS_"invalid DumpNo.");
        return 1;
    }

    m_dumpbox[DumpNo] = Data;

    CString temp;
    temp.Format(L"No:%d Use:%d TrayName:%s Max:%d Now:%d", DumpNo, m_dumpbox[DumpNo].Use == true ? 1 : 0, (LPCTSTR)m_dumpbox[DumpNo].trayInfo.Name, m_dumpbox[DumpNo].trayInfo.MaxPocket, m_dumpbox[DumpNo].NowPocket);
    TRACE_CLASS_AND_METHOD_AND_THIS_(CStringA)temp);

    return NO_ERR;//0
}

long CTrayDumpBox::GetBoxInfo(long DumpNo, DUMP_TRAY& Data)
{
    if (DumpNo != 0 && DumpNo != 1)
    {
        TRACE_CLASS_AND_METHOD_AND_THIS_"invalid DumpNo.");
        return 1;
    }

    Data = m_dumpbox[DumpNo];
    return NO_ERR;//0
}

long CTrayDumpBox::sendTrayDumpBoxLastPocket(long dumpNo, long lastPocket)
{
    unsigned nSubMsg[3];
    CString strSendMsg;
    ZeroMemory(&nSubMsg, sizeof(nSubMsg));
    strSendMsg.Format(_T("%d,%d"), dumpNo, lastPocket);
    nSubMsg[0] = HMI_CMD1ST_1;
    nSubMsg[1] = HMI_CMD2ND_32;
    nSubMsg[2] = HMI_CMD2ND_00;
    SendToHMI(nSubMsg[0], nSubMsg[1], nSubMsg[2], strSendMsg);

    TRACE(_T("[PWR] SendTrayDumpBoxLastPocket Dump:%d Pocket:%d\n"), dumpNo, lastPocket);

    return 0;
}

#undef TRACE_CLASS_AND_METHOD_AND_THIS_