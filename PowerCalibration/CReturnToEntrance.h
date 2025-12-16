#pragma once
#include "CPowerConveyor.h"

class CReturnToEntrance : public CPowerConveyor
{
public:
	CReturnToEntrance();
	~CReturnToEntrance();
	void Run(long From);
private:
	static UINT StartReturnToEntrance(LPVOID wParam);
};

extern CReturnToEntrance* gcReturnToEntrance;