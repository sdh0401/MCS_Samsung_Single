#pragma once
#include "GlobalDefine.h"
#include "GlobalData.h"
#include "Thread.h"
#include "EthernetVision.h"

class CPowerVision : public CThread
{
public:
	CPowerVision(bool bSimul);
	~CPowerVision();
	void Run();
	bool SendCommand(int nCmd, CString strCmd, long Table);
	void ParsingVisionMessage(long lVisCmd, CString strMsg, long Table);

private:
	BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	void ClearSubCmd();
	//static UINT ProcessVisionResult(LPVOID wParam);
	char m_SubCmd[BUFSIZE];
	bool m_bSimulation;
	COMMAND_DPRAM VIS_RST[MAXTABLENO][WRITE_BUFFSIZE];
};

extern CPowerVision* gcPowerVision;