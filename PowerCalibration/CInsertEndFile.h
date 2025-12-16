#pragma once
#include "GlobalDefine.h"
#include "CDiskFile.h"

class CInsertEndFile : public CDiskFile
{
public:
	CInsertEndFile();
	~CInsertEndFile();
	bool ReadFile(void);
	bool SaveFile(void);
	void InitializeFile(void);
	unsigned GetBaseAddr(int nGantrynPCB, unsigned Block, unsigned Point);
	unsigned GetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point);
	void SetInsertEnd(int Conv, int Gantry, unsigned Block, unsigned Point);
	void SetInsertStatus(int Conv, int Gantry, unsigned Block, unsigned Point, int Status);
	void ClearInsertEnd(int Conv);
	unsigned GetInsertEndToUserMemory(int Conv, int Gantry, unsigned Block, unsigned Point);
	void SetInsertEndToUserMemory(int Conv, int Gantry, unsigned Block, unsigned Point);
	void ClearInsertEndToUserMemory(int Conv);
};

extern CInsertEndFile* gcInsertEndFile;