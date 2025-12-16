#pragma once
#include "GlobalDefine.h"
class Cwmx3LinearIntplPos
{
public:
	Cwmx3LinearIntplPos();
	~Cwmx3LinearIntplPos();
	long StartLinearIntplPos();
	long StartLinearIntplPosR();
	long StartPathLinearIntplPos(Point_XY Linear, Point_XY StartCenter, Point_XY EndCenter, Point_XY Goal);
};

extern Cwmx3LinearIntplPos* gcWmx3LinearIntplPos;