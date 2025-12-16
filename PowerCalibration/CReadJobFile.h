#pragma once
#include "GlobalDefine.h"
#include "tinyxml2.h"
#include <string>

class CReadJobFile
{
public: 
    INSERT GetInsert(const long& gantry, const long& insertNo);

    /// <summary>
    /// BLOCK_SEQ_PATTERN, BLOCK_SEQ_STEP 모드로 생산 시 회피, 삽입, 누름, 높이검사, 클린칭에 대해서 1번블록 기준으로 다른 블록도 동일하게 생산하도록 데이터 복사.
    /// </summary>
    /// <returns>성공 시 0</returns>
    long AddBlockSequence();

private:
    /// <summary>
    /// BLOCK_SEQ_STEP 일때 : 전체 가지고 복사
    /// BLOCK_SEQ_PATTERN 일때 : 한 스텝씩 복사.
    /// #1. 한스텝짜리 삽입설정 데크 구하기.
    /// #2. 해당 데크 가지고 복사.
    /// 공통 : 마지막에 SetProduction (TotalInsertCount, UseInsertCount 갱신)
    /// </summary>
    /// <param name="existingInsertDeque">복제하기 전 기존의 삽입설정 정보를 저장한 deque</param>
    /// <returns>항상 0</returns>
    int addInsertSequence(const std::deque<INSERT>& existingInsertDeque, const long& gantry);

    /// <summary>
    /// addInsertSequence와 같은 원리.
    /// </summary>
    /// <param name="existingInsertDeque">기존 삽입설정</param>
    /// <param name="existingAvoidMotionDeque">기존 회피장착설정</param>
    /// <returns></returns>
    int addAvoidSequence(const std::deque<INSERT>& existingInsertDeque, const std::deque<AVOIDMOTION>& existingAvoidMotionDeque, const long& gantry);

    /// <summary>
    /// addInsertSequence와 같은 원리.
    /// </summary>
    /// <param name="existingInsertDeque"></param>
    /// <param name="existingMeasureHeightDeque"></param>
    /// <returns></returns>
    int addHeightMeasureSequence(const std::deque<INSERT>& existingInsertDeque, const std::deque<MEASUREHEIGHT>& existingMeasureHeightDeque);

    /// <summary>
    /// 전달받은 INSERT 데크에서 previousStepNo 스텝 이후에 처음으로 등장하는 스텝에 해당하는 데크를 stepInsertDeque에 담는다.
    /// stepInsertDeque.clear() 하면서 시작.
    /// 반복문 안에서 stepNo = getNextStepDeque(deque, deque, stepNo); 이런식으로 사용하면 된다. (기본값 -1 인 stepNo 가 -1이 될때까지 반복하는식)
    /// </summary>
    /// <param name="existingInsertDeque"></param>
    /// <param name="stepInsertDeque"></param>
    /// <param name="previousStepNo">이 스텝번호 다음 스텝번호의 삽입설정 데이터들을 데크에 담아준다. -1이면 맨 처음걸로 저장.</param>
    /// <returns>stepInsertDeque의 스텝번호(저장한스텝번호)를 리턴. 데이터가 없을 경우 -1</returns>
    int getNextStepDeque(const std::deque<INSERT>& existingInsertDeque, std::deque<INSERT>& stepInsertDeque, const int& previousStepNo);

    void SetInsert2(const INSERT& insert, const long& insertNo);

    AVOIDMOTION GetAvoidMotion(const long& gantry, const long& insertNo);

    void SetAvoidMotion2(const long& InsertNo, const long& Order, const long& Use, const Point_XYRZ& xyrz);

public:
	CReadJobFile();
	~CReadJobFile();
	void Initial();
	void InitPcb();
	void InitPick(long index);
	void InitPackage(long index);
	void InitFeeder(long index);
	void InitRetryLed(long index);
	void InitInsert(long insertNo);
	void InitMeasureHeight(long order);
	void InitForming(long index);

	void SetProduction(PRODUCTION Prod);
	void SetPcb(PCB Pcb);
	void SetOrigin(ORIGIN Origin);
	void SetBlockOrigin(ORIGIN BlockOrigin, long BlockNo);
	void SetMark(FIDUCIAL FidMark);
	void SetBlockMark(FIDUCIAL FidMark, long BlockNo);
	void SetPick(PICK Pick, long FeederNo);
	void SetDiscard(DISCARD Discard, long FeederNo);
	void SetPackage(PACKAGE Package, long FeederNo);
	long SetInsert(INSERT Insert, long InsertNo);
	long SetMeasureHeight(MEASUREHEIGHT inspect);
	void SetMeasureHeightTotal(long Count);
	long GetMeasureHeightTotal();
	void SetFeeder(FEEDER Feeder, long FeederNo);
	void SetRetryLed(RETRY_LED RetryLed, long FeederNo);
	void SetNozzle(NOZZLE Nozzle, long NozzleNo);
	void SetStandyBy(STANDBY Standby);
	void SetPickupZStandBy(double PickupZStandby);
	void SetTray(TRAY_INFO Tray, long FeederNo);;
	long SetAvoidMotion(long InsertNo, long Order, long Use, Point_XYRZ xyrz);
	void SetPartDrop(PART_DROP data);
	long SetForming(long FeederNo, FORMING_COMPONENT Forming);

	PART_DROP GetPartDrop();
	PRODUCTION GetProduction();
	PCB GetPcb();
	ORIGIN GetOrigin();
	ORIGIN GetBlockOrigin(long BlockNo);
	FIDUCIAL GetMark();
	FIDUCIAL GetBlockMark(long BlockNo);
	PICK GetPick(long FeederNo);
	DISCARD GetDiscard(long FeederNo);
	PACKAGE GetPackage(long PackageNo);
	INSERT GetInsert(long InsertNo);
	long GetInsertNo(long blockNo, long index);
	MEASUREHEIGHT GetMeasureHeight(long order);
	FEEDER GetFeeder(long FeederNo);
	RETRY_LED GetRetryLed(long PackageNo);
	NOZZLE GetNozzle(long NozzleNo);
	STANDBY GetStandyBy();
	double GetPickupZStandBy();
	AVOIDMOTION GetAvoidMotion(long InsertNo);
	void InitAvoidMotion(long insertNo);
	TRAY_INFO GetTray(long FeederNo);
	FORMING_COMPONENT GetForming(long FeederNo);

	void SetDivideInspect(DIVIDE_INSPECT Data);
	void InitDivideInspect();
	DIVIDE_INSPECT GetDivideInspect(long FeederNo);

	long ReadFile();
	long CheckJobFileValid();
	long GetPackgeIndexFromFeederNo(long FeederNo);

    bool Read_TRAY_DIRECT(CString fileName, CString trayName, TRAY_INFO* data);

	void SetBarcode(BARCODE Barcode);
	void SetBarcodeBlock1(BARCODE Barcode); // 20210415 HarkDo
	void SetBarcodeBlock2(BARCODE Barcode); // 20210415 HarkDo
	BARCODE GetBarcode();

	BLOCKSKIP_HM GetBlockSkipHM(long BlockNo);
	BARCODE GetBarcodeBlock1();
	BARCODE GetBarcodeBlock2();
	void SetBlockSkipHM(BLOCKSKIP_HM Data);

private:

	long FindJobBlock(CStringArray* AllData, CString strString, CString strEnd, CStringArray* ResultData);
	long Read_PRODUCTION_INFO(CStringArray* AllData);
	long Read_PCB(CStringArray* AllData);
	long Read_ORIGIN(CStringArray* AllData);
	long Read_FIDUCIAL(CStringArray* AllData);
	long Read_PICK(CStringArray* AllData);
	long Read_PACKAGE(CStringArray* AllData);
	long Read_INSERT(CStringArray* AllData);
	long Read_MEASUREHEIGHT(CStringArray* AllData);
	long Read_FEEDER(CStringArray* AllData);
	long Read_LED_VALUE(CStringArray* AllData);
	long Read_NOZZLE(CStringArray* AllData);
	long Read_DISCARD(CStringArray* AllData);
	long Read_STANDBY(CStringArray* AllData);
	long Read_PICKUPZSTANDBY(CStringArray* AllData);
	long Read_BARCODE(CStringArray* AllData);
	long Read_TRAY(CStringArray* AllData);
	long Read_AVOIDMOTION(CStringArray* AllData);
	long Read_PartDrop(CStringArray* AllData);
	long Read_DivideInspect(CStringArray* AllData);
	long Read_BlockSkipHM(CStringArray* AllData);
	long Read_FORMING(CStringArray* AllData);

	ORIGIN ParseOriginFromStr(CString str);
	ORIGIN ParseBlockOriginFromStr(CString str);
	FIDUCIAL ParseFiducialFromStr(CString str);

    std::string ToStdString(CString strText);
    CString GetXMLError(tinyxml2::XMLError ErrXML);


	void MakeFileName();
	CString GetFileName();
	void SetDir(CString Dir);
	void SetHeader(CString Header);
	void SetExtension(CString Ext);
	PRODUCTION m_Production;
	PCB m_Pcb;
	ORIGIN m_Origin;
	ORIGIN m_BlockOrigin[MAXBLOCKNO];
	FIDUCIAL m_Mark;
	FIDUCIAL m_BlockMark[MAXBLOCKNO];
	PICK m_Pick[MAXFEEDERNO];
	PACKAGE m_Package[MAXPACKAGENO];
	INSERT m_Insert[MAXINSERTNO];
	MEASUREHEIGHT m_MeasureHeight[MAXINSERTNO];
	long m_MeasureHeightTotal;
	FEEDER m_Feeder[MAXFEEDERNO];
	RETRY_LED m_RetryLed[MAXPACKAGENO];
	NOZZLE m_Nozzle[MAXNOZZLENO];
	DISCARD m_Discard[MAXFEEDERNO];
	STANDBY m_StandBy;
	double m_PickupZStandby;
	CString m_StrFileName, m_StrDir, m_StrHeader, m_StrExtension;
	TRAY_INFO m_Tray[MAXFEEDERNO];
	AVOIDMOTION m_AvoidMotion[MAXINSERTNO];
	PART_DROP m_PartDrop;
	DIVIDE_INSPECT m_DividePackage[MAXFEEDERNO];	
	
	BARCODE m_Barcode;
	BARCODE m_BarcodeBlock1, m_BarcodeBlock2;
	BLOCKSKIP_HM m_BlockSkipHM[MAXBLOCKNO];
	FORMING_COMPONENT m_Forming[MAXFEEDERNO];
};

extern CReadJobFile* gcReadJobFile;
