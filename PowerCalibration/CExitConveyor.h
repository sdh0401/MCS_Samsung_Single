#pragma once
#include "CPowerConveyor.h"

class CExitConveyor : public CPowerConveyor
{
public:
	CExitConveyor(bool bReverse);
	~CExitConveyor();
	bool IsSaftyStep(ConveyorStep Step);
	static UINT StartExitConveyor(LPVOID wParam);
	void Run(long ProdMode);
};

extern CExitConveyor* gcExitConveyor;
