#pragma once
#include "CPowerConveyor.h"
class CWorkConveyor : public CPowerConveyor
{
public:
	CWorkConveyor(bool bReverse);
	~CWorkConveyor();
	bool IsSaftyStep(ConveyorStep Step, long SimulLoadType);
	static UINT StartWorkConveyor(LPVOID wParam);
	void Run(long ProdMode);
};

extern CWorkConveyor* gcWorkConveyor;
