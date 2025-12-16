#include "pch.h"
#include "Cwmx3SystemParam.h"
#include <WMX3Api.h>
#include <CoreMotionApi.h>

using namespace wmx3Api;

Cwmx3SystemParam* gcWmx3SystemParam;
Cwmx3SystemParam::Cwmx3SystemParam()
{
	ZeroMemory(&m_SystemParam, sizeof(m_SystemParam));
}

Cwmx3SystemParam::~Cwmx3SystemParam()
{
}

