#pragma once
#include "CPowerThread.h"
#include "GlobalDefine.h"

class CSmemaControl : public CPowerThread
{
public:
	CSmemaControl();
	~CSmemaControl();
	void InitSmema();
	void Run();
	void SetIO(long Conv, long PrevIn, long PrevOut, long NextIn, long NextOut);
	long CheckPrevSmema(long Conv);
	long CheckNextSmema(long Conv);
	void PrevSmemaOut(long Conv, UBYTE output);
	void NextSmemaOut(long Conv, UBYTE output);
	void ShowSmemaIO();

private:
	static UINT StartSmemaControl(LPVOID wParam);
	SmemaControl m_bSmema[MAX_CONV];
	long m_ShowID;
};

extern CSmemaControl* gcSmemaControl;