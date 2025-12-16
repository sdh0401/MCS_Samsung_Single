#pragma once
#include "Thread.h"

class CDecoding4 : public CThread
{
public:
	CDecoding4();
	~CDecoding4();
private:
	virtual BOOL OnTask();
	virtual BOOL OnTask(LPVOID lpv);
	ThreadId_t m_id;
	CString DisplayCameraRoi(CString strHostMsg);
	CString CameraMode(CString strHostMsg);
	CString TrainingMark(CString strHostMsg);
	CString InspectionMark(CString strHostMsg);
	CString PartRecognition(CString strHostMsg);
	CString PartReturn(CString strHostMsg);
	CString PartDiscard(CString strHostMsg);
	CString DropCheckImageSave(CString strHostMsg);
	CString DropCheckProcess(CString strHostMsg);
	CString InspectionBarcode(CString strHostMsg);
	CString SendBarcodeComPortStatus(CString strHostMsg);
	CString BarcodeComPortOpen(CString strHostMsg);
	CString BarcodeComPortClose(CString strHostMsg);

	CString ReceiveMESResult(CString strHostMsg);
};

extern CDecoding4* gcDecoding4;