#pragma once
#include "Thread.h"

class CDecoding2 : public CThread
{
public:
	CDecoding2();
	~CDecoding2();
	long SetHMIRunMode(long RunMode);
	long GetHMIRunMode();
	CString ReadRecipeFile(CString strHostMsg);
	CString RunControl(CString strHostMsg);
	CString FeedBackRunControl(CString strHostMsg);
	CString ReadRunMode(CString strHostMsg);
	CString StartRun();
	CString PauseRun();
	CString ResumeRun();
	CString ResumeMoveOnce(CString strHostMsg);
	CString StopNow();
	CString StopStep();
	CString StopBlock();
	CString StopBoard();
	CString StopCycle();
	CString GetReadyPrepareRun(CString strHostMsg);
	CString GetReadyRun(CString strHostMsg);
	CString RunBypass(CString strHostMsg);

private:
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	long m_HMIRunMode;
};

extern CDecoding2* gcDecoding2;