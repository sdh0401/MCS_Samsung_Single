#pragma once
#include "GlobalDefine.h"
class CMark
{
public:
	CMark(long Gantry);
	~CMark();
    long blockFiducialMarkChecking(const long& blockNo);
	Point_XY GetMarkPosition(long MarkNo);
	ORIGIN GetOrigin();
	long FiducialMarkChecking();
	long MachineReferenceMarkChecking(MachineReferenceMark MachineRefMark);
	long MachineReferenceMarkCheckingAutoCompen(MachineReferenceMark MachineRefMark, Ratio_XYRZ ratio);
	long SetStandBy(STANDBY StandBy);
	Ratio_XYRZ GetMinRatio();
private:
	long m_Gantry;
	STANDBY m_StandBy;
};

extern CMark* gcMark;
