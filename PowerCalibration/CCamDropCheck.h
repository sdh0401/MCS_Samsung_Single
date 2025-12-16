#pragma once
#include "Thread.h"
#include "GlobalDefine.h"

class CCamDropCheck : public CThread
{
public:
	CCamDropCheck(long CamTable);
	~CCamDropCheck();
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	long GetTable();
	static UINT StartDropCheck(LPVOID wParam);
	void InitDropResult();
	void SetDropResult(long result);
	bool GetDropResult(long* result);
private:
	long m_Table;
	long m_DropResult;
	bool m_ProcessComplete;
	ThreadId_t m_id;

};

extern CCamDropCheck* gcCamDropCheck[MAXGANTRY];
