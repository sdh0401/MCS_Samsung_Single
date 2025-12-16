#pragma once
#include "CPowerConveyor.h"

class CReturnToWork : public CPowerConveyor
{
public:
	CReturnToWork();
	~CReturnToWork();
	void Run();
private:
	static UINT StartReturnToWork(LPVOID wParam);
};

extern CReturnToWork* gcReturnToWork;