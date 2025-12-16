#pragma once
#include "CPowerConveyor.h"

class CEntryConveyor : public CPowerConveyor
{
public:
	CEntryConveyor(bool bReverse);
	~CEntryConveyor();
	bool IsSaftyStep(ConveyorStep Step);
	static UINT StartEntryConveyor(LPVOID wParam);
	void Run(long ProdMode);
};

extern CEntryConveyor* gcEntryConveyor;
