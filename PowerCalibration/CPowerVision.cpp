#include "pch.h"
#include "CPowerVision.h"
#include "CPowerClient.h"
#include "EthernetVision.h"
#include "CApplicationTime.h"
#include "CTokenizer.h"
#include "DefineThreadLoopTime.h"
#include "Vision.h"
#include "Trace.h"
#include "CPowerLog.h"

CPowerVision* gcPowerVision;
CPowerVision::CPowerVision(bool bSimul)
{
    GetId(&m_id);
    m_bSimulation = bSimul;
    ZeroMemory(&m_SubCmd, sizeof(m_SubCmd));
}

CPowerVision::~CPowerVision()
{
    TRACE(_T("[PWR] ~CPowerVision(0x%04X) Quit\n"), m_id);
}

void CPowerVision::ClearSubCmd()
{
    ZeroMemory(&m_SubCmd, sizeof(m_SubCmd));
}

bool CPowerVision::SendCommand(int nCmd, CString strCmd, long Table)
{
	bool bRet = false;
    if(Table == FRONT_VISION)
	    bRet = gcPowerClient->SendCommandTo(ID_VIS, nCmd, 0, 0, strCmd, NO_WAIT);
    else if (Table == REAR_VISION)
        bRet = gcPowerClient->SendCommandTo(ID_VIS_REAR, nCmd, 0, 0, strCmd, NO_WAIT);
    else
    {
        TRACE(_T("[PWR] INVALID Table(%d) SendCommand\n"), Table);
    }
	return bRet;
}

void CPowerVision::ParsingVisionMessage(long lVisCmd, CString strMsg, long Table)
{
    CTokenizer* cTokenizer = new CTokenizer(strMsg, _T(","), FALSE);
    CString strValue;
    int iValue;
    double dblValue;
    ASSERT(cTokenizer->GetCount() > 0);
    for (int i = 0; i < cTokenizer->GetCount(); i++)
    {
        if (i == WRITE_BUFFSIZE - 1) break;
        strValue = cTokenizer->GetString(i);
        if (strValue.Find(_T(".")) >= 0)
        {
            VIS_RST[Table][i + 1]._dbl = cTokenizer->GetDouble(i);
        }
        else
        {
            iValue = cTokenizer->GetInt(i);
            memcpy(&dblValue, &iValue, sizeof(int));
            VIS_RST[Table][i + 1]._dbl = dblValue;
            VIS_RST[Table][i + 1]._long = iValue;
        }
    }
    gcEthernetVision->SetVisionResult(Table, lVisCmd, VIS_RST[Table]);
    delete cTokenizer;
    cTokenizer = NULL;
}

BOOL CPowerVision::OnTask(LPVOID lpv)
{
    signed nSubMsg[MAXSUBMSGNO] = { INITIALZE, INITIALZE , INITIALZE };
    signed nTargetID = INITIALZE;
    CString strMsg;
    if (lpv)
    {
        PowerThreadMessage* msgReceived = reinterpret_cast<PowerThreadMessage*>(lpv);
        nTargetID = msgReceived->GetID();
        if (nTargetID != m_id)
        {
            TRACE(_T("[PWR] CPowerVision GetThreadMsg() not match TargetID:0x%04X OwnerID:0x%04X\n"), nTargetID, m_id);
        }
        strMsg = msgReceived->GetThreadMsg(); // Axis
        for (int indx = 0; indx < MAXSUBMSGNO; ++indx)
        {
            nSubMsg[indx] = msgReceived->GetThreadArgMsg(indx);
        }
        switch (nSubMsg[0])
        {
        case VIS_CA: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Auto Camera Calibration\n"));            
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_FT: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Fiducial Mark Training\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_FI:
            if(gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Fiducial Mark Inspection\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_DB: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Vision Data Download\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_DT: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Digitizer Control\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_WC: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Window Control\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_ESC: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Vision Escape\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_RA: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Rotate Angle Calibration\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_SV: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Vision software version\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_VA8:
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Component Inspection\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_BARCODE:
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Barcode Inspection\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
		case VIS_DROPCHECK:
			if (gcPowerLog->IsShowVisionLog() == true)
				TRACE(_T("[PWR] PartDrop Inspection\n"));
			ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
			break;
        case VIS_REQDB: 
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Component DB MCS <-> Vision Communication\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        case VIS_CAMOFFSET:
            if (gcPowerLog->IsShowVisionLog() == true)
                TRACE(_T("[PWR] Send Camera Recognition Offset\n"));
            ParsingVisionMessage(nSubMsg[0], strMsg, nSubMsg[1]);
            break;
        default:
            TRACE(_T("[PWR] Invalid Command\n"));
            break;
        }
        delete msgReceived;
    }
    return TRUE;
}

void CPowerVision::Run()
{
}

