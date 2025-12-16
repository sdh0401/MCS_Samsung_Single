#pragma once
#include "Thread.h"
class CPowerTeachBox : public CThread
{
public:
	CPowerTeachBox();
	~CPowerTeachBox();
	void StopAxis(CString strAxis);
	void StartMove(CString strAxis, unsigned Dir, unsigned Speed);
	void StartJog(CString strAxis, unsigned Dir, unsigned Speed);
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	BOOL DirectControl(CString Msg, signed SubMsg1, signed SubMsg2, signed SubMsg3);
private:
	ThreadId_t m_id;
};

extern CPowerTeachBox* gcPowerTeachBox;