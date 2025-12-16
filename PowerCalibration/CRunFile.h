#pragma once
#include "GlobalDefine.h"
#include "CDiskFile.h"

class CRunFile : public CDiskFile
{
public:
	CRunFile();
	~CRunFile();
	bool MakeFile();
	bool ReadFile(void);
	bool SaveFile(void);
	void InitializeFile(void);
	unsigned GetBaseAddressNozzlePerHead(unsigned Gantry, unsigned Head);
	void WriteNozzleNoPerHead(unsigned Gantry, NozzleNoPerHeadStruct NzlNo);
	NozzleNoPerHeadStruct ReadNozzleNoPerHead(unsigned Gantry);
};

extern CRunFile* gcRunFile;