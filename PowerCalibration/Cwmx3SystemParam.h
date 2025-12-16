#pragma once
#include <WMX3Api.h>
#include <CoreMotionApi.h>
#include "GlobalDefine.h"

using namespace wmx3Api;

class Cwmx3SystemParam
{
public:
	Cwmx3SystemParam();
	~Cwmx3SystemParam();
	Config::SystemParam m_SystemParam[MAXAXISNO];
};

extern Cwmx3SystemParam* gcWmx3SystemParam;
