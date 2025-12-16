#pragma once
#include "GlobalDefine.h"

class CReturnComponent
{
public:
	CReturnComponent();
	~CReturnComponent();
	void SetProdRunMode(long ProdRunMode);
	long GetProdRunMode();
	long GetReturnComponentHeadNo(long ReturnOrd);
	long GetMaxReturnComponentOrder();
	void SetMaxReturnComponentOrder(long MaxPickOrd);
	long GetFdNoFromReturnComponentOrder(long ReturnOrd);
	long GetReturnDelayFromFdNo(long FdNo);
	Ratio_XYRZ GetRatioFromFdNo(long FdNo);
	Point_XYRZ GetPickOffsetFromFdNo(long FdNo);
	long Run(long Gantry);

private:
	long m_MaxReturnOrder;
	long m_ProdRunMode;
};

extern CReturnComponent* gcReturnComponent;