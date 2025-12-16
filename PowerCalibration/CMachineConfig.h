#pragma once

#include "GlobalDefine.h"
#include "tinyxml2.h"
#include <string>
using namespace std;

class CMachineConfig
{
public:
	CMachineConfig();
	~CMachineConfig();
	CString GetFilePath(CString fileName);
	long ReadConfigFile();
	long GetRevisionFileName(CString item, CString* fileName);
	long GetHWOptionIOMapVersion(long* Version);
	long GetHWOptionCameraLaser();

	long ReadFileANCConfig();
	long ReadFileANCCal();
	long SaveFileANCCal(ANC_STRUCT data);
	long ParsingAncConfig(tinyxml2::XMLElement* eleANC, CString ancName, ANC_STRUCT* data);
	long ParsingAncCal(tinyxml2::XMLElement* eleANC, CString ancName, ANC_STRUCT* data);
	long UpdateANCCal(tinyxml2::XMLElement* eleANC, ANC_STRUCT data);
	long EditANCCalMark(long base, ANC_MARK_STRUCT data);

	bool GetOpenConfigFile();
	bool IsExistANCData();
	ANC_STRUCT GetConfigANC(long Base);
	bool GetConfigANC(long Base, ANC_STRUCT* data);
	ANC_STRUCT GetCalANC(long Base);
	bool GetConfigANCHole(long Base, long holeNo, ANC_HOLE_STRUCT* holeData);
	bool GetCalANCHole(long Base, long holeNo, ANC_HOLE_STRUCT* holeData);
	bool IsANCUpType(long Base);
	long GetCameraVersion(long CamNo, int* Version);
	long GetOptionCameraLaser();
private:
	bool m_OpenConfigFile;
	tinyxml2::XMLDocument m_DocumentHW;

	CString m_strDirectoryConfig;
	CString m_strDirectoryConfigANC;
	CString m_strDirectoryCal;
	CString m_strConfigXml;
	CString m_strCalibration_ANCxml;

	CString m_strROOT;
	CString m_strCONFIG_FILE;
	CString m_strANC;

	CString m_strFRONT_ANC;
	CString m_strREAR_ANC;

	bool m_OpenANCCalFile;
	bool m_OpenANCConfigFile;
	ANC_STRUCT m_ConfigANCFront;
	ANC_STRUCT m_ConfigANCRear;
	ANC_STRUCT m_CalANCFront;
	ANC_STRUCT m_CalANCRear;

	CString m_strHARDWARE_OPTION;
	CString m_strIO_MAP_VERSION;
	CString m_strCAMERA_VERSION;
	CString m_strCAMERA_LASER;
	long m_OptionCameraLaser;

};

extern CMachineConfig* gCMachineConfig;