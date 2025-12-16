#pragma once
#include "CPowerConveyor.h"

class COutToExit : public CPowerConveyor
{
public:
	COutToExit();
	~COutToExit();
	void Run(long From);
private:
	static UINT StartOutToExit(LPVOID wParam);
};

extern COutToExit* gcOutToExit;