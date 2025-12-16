#pragma once
#include "CPowerConveyor.h"

class CLoadingToWork : public CPowerConveyor
{
public:
	CLoadingToWork();
	~CLoadingToWork();
	void Run();
private:
	static UINT StartLoadingToWork(LPVOID wParam);
};

extern CLoadingToWork* gcLoadingToWork;