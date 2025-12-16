#pragma once
#include "Thread.h"

class CDecoding : public CThread
{
public:
	CDecoding();
	~CDecoding();
private:
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	bool Decoding1(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3);
	bool Decoding2(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3);
	bool Decoding3(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3);
	bool Decoding4(CString strMsg, long SubMsg1, long SubMsg2, long SubMsg3);
};

extern CDecoding* gcDecoding;