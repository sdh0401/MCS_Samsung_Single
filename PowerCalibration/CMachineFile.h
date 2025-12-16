#pragma once

#include "GlobalDefine.h"
#include "CDiskFile.h"

class CMachineFile : public CDiskFile
{
public:
	CMachineFile();
	~CMachineFile();
	void InitializeFile(void);
	bool MakeFile();
	bool ReadFile(void);
	bool SaveFile(void);
	
	bool InitToDiskBlock0();		// Y1 & Y2 Home Offset, Use Axis, 
	bool InitToDiskBlock1();		// Front & Rear 1D Compensation
	bool InitToDiskBlock2();		// Front 2D Compensation
	bool InitToDiskBlock3();		// Rear 2D Compensation
	bool InitToDiskBlock4();		// Front Software 2D Compensation
	bool InitToDiskBlock5();		// Rear Software 2D Compensation
	bool InitToDiskBlock6();		// Conveyor Option
	bool InitToDiskBlock7();		// Conveyor Parameter
	bool InitToDiskBlock8();		// Camera Align
	bool InitToDiskBlock9();		// Feeder Reference No, Position
	bool InitToDiskBlockA();		// Front Z Compensation
	bool InitToDiskBlockB();		// Height Measurement
	bool InitToDiskBlockC();		// Aging
	bool InitToDiskBlockD();		// Thrash
	//bool InitToDiskBlockE();
	bool InitToDiskBlockF();

	bool WriteToDiskBlock0();		// Y1 & Y2 Home Offset, Use Axis, 
	bool WriteToDiskBlock1();		// Front & Rear 1D Compensation
	bool WriteToDiskBlock2();		// Front 2D Compensation
	bool WriteToDiskBlock3();		// Rear 2D Compensation
	bool WriteToDiskBlock4();		// Front Software 2D Compensation
	bool WriteToDiskBlock5();		// Rear Software 2D Compensation
	bool WriteToDiskBlock6();		// Conveyor Option
	bool WriteToDiskBlock7();		// Conveyor Parameter
	bool WriteToDiskBlock8();		// Camera Align
	bool WriteToDiskBlock9();		// Feeder Reference No, Position
	bool WriteToDiskBlockA();		// Front Z Compensation
	bool WriteToDiskBlockB();		// Height Measurement
	bool WriteToDiskBlockC();		// Aging
	bool WriteToDiskBlockD();
	//bool WriteToDiskBlockE();
	bool WriteToDiskBlockF();

	bool ReadToDiskBlock0(void);	// Y1 & Y2 Home Offset, Use Axis, 
	bool ReadToDiskBlock1(void);	// Front & Rear 1D Compensation
	bool ReadToDiskBlock2(void);	// Front 2D Compensation
	bool ReadToDiskBlock3(void);	// Rear 2D Compensation
	bool ReadToDiskBlock4(void);	// Front Software 2D Compensation
	bool ReadToDiskBlock5(void);	// Rear Software 2D Compensation
	bool ReadToDiskBlock6(void);	// Conveyor Option
	bool ReadToDiskBlock7(void);	// Conveyor Parameter
	bool ReadToDiskBlock8(void);	// Camera Align
	bool ReadToDiskBlock9(void);	// Feeder Reference No, Position
	bool ReadToDiskBlockA(void);	// Front Z Compensation
	bool ReadToDiskBlockB(void);	// Height Measurement
	bool ReadToDiskBlockC(void);	// Aging
	bool ReadToDiskBlockD(void);
	//bool ReadToDiskBlockE(void);
	bool ReadToDiskBlockF(void);
};

extern CMachineFile* gcMachineFile;