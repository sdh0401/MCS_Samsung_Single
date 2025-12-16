#include "pch.h"
#include "CMachineConfig.h"
#include "CPowerCalibrationData.h"
#include "GlobalData.h"
#include "LockDef.h"
#include "Trace.h"
#include "CPowerLog.h"
#include "tinyxml2.h"
#include <vector>
#include <algorithm>
#include <string>
using namespace std;

CMachineConfig* gCMachineConfig;
CMachineConfig::CMachineConfig()
{
	m_OpenConfigFile = false;
	m_OpenANCCalFile = false;
	m_OpenANCConfigFile = false;

	m_strDirectoryConfig = _T("C:\\Power\\i6.0\\MCS\\MachineConfig");
	m_strDirectoryConfigANC = _T("\\ANC");

	m_strDirectoryCal.Format(_T("%s\\Calibration"), (LPCTSTR)m_strDirectoryConfig);
	m_strConfigXml = _T("MachineConfig.xml");
	m_strCalibration_ANCxml = _T("Calibration_ANC.xml");
	m_strCONFIG_FILE = _T("CONFIG_FILE");
	m_strROOT = _T("ROOT");
	m_strANC = _T("ANC");

	m_strFRONT_ANC = _T("FRONT_ANC");
	m_strREAR_ANC = _T("REAR_ANC");

	m_ConfigANCFront.Use = m_ConfigANCRear.Use = m_CalANCFront.Use = m_CalANCRear.Use = false;
	m_ConfigANCFront.UpdownType = m_ConfigANCRear.UpdownType = m_CalANCFront.UpdownType = m_CalANCRear.UpdownType = false;
	m_ConfigANCFront.Name = m_ConfigANCRear.Name = m_CalANCFront.Name = m_CalANCRear.Name = _T("");

	m_strHARDWARE_OPTION = _T("HARDWARE_OPTION");
	m_strIO_MAP_VERSION = _T("IO_MAP_VERSION");
	m_strCAMERA_VERSION = _T("CAMERA_VERSION");
	m_strCAMERA_LASER = _T("CAMERA_LASER");

	m_OptionCameraLaser = 0;
}

CMachineConfig::~CMachineConfig()
{
}

CString CMachineConfig::GetFilePath(CString fileName)
{
	CString strFileName;
	strFileName.Format(_T("%s\\%s"), (LPCTSTR)m_strDirectoryConfig, (LPCTSTR)fileName);

	return strFileName;
}
long CMachineConfig::ReadConfigFile()
{
	long Err = READ_FAIL_MACHINE_CONFIG;
	CString filePath;
	CString strANCRevFile;
	tinyxml2::XMLError ErrXML;
	ANC_STRUCT ancFront;
	long version = 0;

	m_OpenConfigFile = false;
	filePath = GetFilePath(m_strConfigXml);
	CT2A aFilePathHW(filePath);
	ErrXML = m_DocumentHW.LoadFile(aFilePathHW);

	if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config Open complete. %s\n"), filePath);
		m_OpenConfigFile = true;
	}
	else if (ErrXML == tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND)
	{
		TRACE(_T("[PWR] Config not found. %s\n"), filePath);
		return NO_ERR;
	}
	else
	{
		TRACE(_T("[PWR] Config Open fail. Err %d %s\n"), READ_FAIL_MACHINE_CONFIG, filePath);
		return READ_FAIL_MACHINE_CONFIG;
	}

	Err = ReadFileANCConfig();
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Config ANC Open fail.\n"));
		return Err;
	}

	Err = ReadFileANCCal();
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Config ANC cal Open fail.\n"));
		return Err;
	}	

	return NO_ERR;
}

long CMachineConfig::GetRevisionFileName(CString item, CString* fileName)
{
	long Err = READ_FAIL_MACHINE_CONFIG;

	CString strFunc(__func__);
	CString strReturn;
	CT2A aItem(item);

	tinyxml2::XMLElement* element = NULL;
	tinyxml2::XMLHandle docHandle(&m_DocumentHW);

	element = docHandle.FirstChildElement("ROOT").FirstChildElement("CONFIG_FILE").FirstChildElement(aItem).ToElement();
	if (element == NULL)
	{
		TRACE(_T("[PWR] Config %s %s read fail. Element null\n"), strFunc, item);
		return Err;
	}

	strReturn = (CString)element->GetText();

	if (strReturn.IsEmpty() == true)
	{
		TRACE(_T("[PWR] Config %s %s read fail. text empty\n"), strFunc, item);
		return Err;
	}

	*fileName = strReturn;
	TRACE(_T("[PWR] Config %s %s filename %s.\n"), strFunc, item, strReturn);
	return NO_ERR;
}

long CMachineConfig::GetHWOptionIOMapVersion(long* Version)
{
	long Err = READ_FAIL_MACHINE_CONFIG;

	CString strFunc(__func__);
	CString strReturn;
	std::string strROOT = CT2CA(m_strROOT);
	std::string strHARDWARE_OPTION = CT2CA(m_strHARDWARE_OPTION);
	std::string strIO_MAP_VERSION = CT2CA(m_strIO_MAP_VERSION);

	tinyxml2::XMLElement* element = NULL;
	tinyxml2::XMLHandle docHandle(&m_DocumentHW);
	int OptionVal = 0;

	CString filePath = GetFilePath(m_strConfigXml);
	CT2A aFilePathHW(filePath);

	element = docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strHARDWARE_OPTION.c_str()).FirstChildElement(strIO_MAP_VERSION.c_str()).ToElement();
	if (element == NULL)
	{
		//docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strHARDWARE_OPTION.c_str()).ToElement()->InsertNewChildElement(strIO_MAP_VERSION.c_str())->InsertNewText("0");

		//m_DocumentHW.SaveFile(aFilePathHW);

		TRACE(_T("[PWR] Config %s %s read fail. Element null\n"), strFunc, m_strIO_MAP_VERSION);
		return Err;
	}

	if (element->QueryIntText(&OptionVal) != tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config %s %s read fail. Element null\n"), strFunc, m_strIO_MAP_VERSION);
		return Err;
	}

	*Version = OptionVal;
	TRACE(_T("[PWR] Config %s %s Value %d.\n"), strFunc, m_strIO_MAP_VERSION, *Version);

	return NO_ERR;
}

long CMachineConfig::GetHWOptionCameraLaser()
{
	long Err = READ_FAIL_MACHINE_CONFIG;

	CString strFunc(__func__);
	CString strReturn;
	std::string strROOT = CT2CA(m_strROOT);
	std::string strHARDWARE_OPTION = CT2CA(m_strHARDWARE_OPTION);
	std::string strCAMERA_LASER = CT2CA(m_strCAMERA_LASER);

	tinyxml2::XMLElement* element = NULL;
	tinyxml2::XMLHandle docHandle(&m_DocumentHW);
	int OptionVal = 0;

	CString filePath = GetFilePath(m_strConfigXml);
	CT2A aFilePathHW(filePath);

	element = docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strHARDWARE_OPTION.c_str()).FirstChildElement(strCAMERA_LASER.c_str()).ToElement();
	if (element == NULL)
	{
		//docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strHARDWARE_OPTION.c_str()).ToElement()->InsertNewChildElement(strIO_MAP_VERSION.c_str())->InsertNewText("0");

		//m_DocumentHW.SaveFile(aFilePathHW);

		TRACE(_T("[PWR] Config %s %s read fail. Element null\n"), strFunc, m_strCAMERA_LASER);
		return Err;
	}

	if (element->QueryIntText(&OptionVal) != tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config %s %s read fail. Element null\n"), strFunc, m_strCAMERA_LASER);
		return Err;
	}

	m_OptionCameraLaser = OptionVal;
	TRACE(_T("[PWR] Config %s %s Value %d.\n"), strFunc, m_strCAMERA_LASER, m_OptionCameraLaser);

	return NO_ERR;
}

long CMachineConfig::ReadFileANCConfig()
{
	long Err = READ_FAIL_ANC_CONFIG;
	m_OpenANCConfigFile = false;

	CString strFunc(__func__);
	CString fileName;

	std::string strROOT = CT2CA(m_strROOT);
	std::string strANC = CT2CA(m_strANC);
	std::string strFRONT_ANC = CT2CA(m_strFRONT_ANC);
	std::string strREAR_ANC = CT2CA(m_strREAR_ANC);

	//if (GetRevisionFileName(CString(strANC.c_str()), &fileName) != NO_ERR)
	//{
	//	TRACE(_T("[PWR] Config %s read fail. filename error\n"), strFunc);
	//	return Err;
	//}

	if (GetRevisionFileName(CString(strFRONT_ANC.c_str()), &fileName) != NO_ERR)
	{
		TRACE(_T("[PWR] Config %s read fail. filename error\n"), strFunc);
		return Err;
	}

	CString strTemp;
	strTemp.Format(_T("%s%s\\%s"), (LPCTSTR)m_strDirectoryConfig, (LPCTSTR)m_strDirectoryConfigANC, (LPCTSTR)fileName);
	std::string strFileName = CT2CA(strTemp);

	ANC_STRUCT readDataF;

	tinyxml2::XMLDocument DocumentXML;
	tinyxml2::XMLError ErrXML = DocumentXML.LoadFile(strFileName.c_str());

	if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config Open complete. %s\n"), fileName);
	}
	else if (ErrXML == tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND)
	{
		TRACE(_T("[PWR] Config not found. %s\n"), fileName);
		return NOT_FOUND_ANC_CONFIG;
	}
	else
	{
		TRACE(_T("[PWR] Config Open fail. Err %d %s\n"), READ_FAIL_ANC_CONFIG, fileName);
		return READ_FAIL_ANC_CONFIG;
	}

	tinyxml2::XMLHandle docHandle(&DocumentXML);
	tinyxml2::XMLElement* eleANC = docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strANC.c_str()).ToElement();

	if (ParsingAncConfig(eleANC, m_strFRONT_ANC, &readDataF) != NO_ERR)
	{
		TRACE(_T("[PWR] Config parsing fail. Err %d %s\n"), READ_FAIL_ANC_CONFIG, fileName);
		return READ_FAIL_ANC_CONFIG;
	}

	m_ConfigANCFront = readDataF;

	if (GetRevisionFileName(CString(strREAR_ANC.c_str()), &fileName) != NO_ERR)
	{
		TRACE(_T("[PWR] Config %s read fail. filename error\n"), strFunc);
		return Err;
	}

	strTemp.Format(_T("%s%s\\%s"), (LPCTSTR)m_strDirectoryConfig, (LPCTSTR)m_strDirectoryConfigANC, (LPCTSTR)fileName);
	strFileName = CT2CA(strTemp);

	ANC_STRUCT readDataR;

	tinyxml2::XMLDocument DocumentXML_Rear;
	tinyxml2::XMLError ErrXML_Rear = DocumentXML_Rear.LoadFile(strFileName.c_str());

	if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config Open complete. %s\n"), fileName);
	}
	else
	{
		TRACE(_T("[PWR] Config Open fail. Err %d %s\n"), READ_FAIL_ANC_CONFIG, fileName);
		return READ_FAIL_ANC_CONFIG;
	}

	tinyxml2::XMLHandle docHandle_Rear(&DocumentXML_Rear);
	tinyxml2::XMLElement* eleANC_Rear = docHandle_Rear.FirstChildElement(strROOT.c_str()).FirstChildElement(strANC.c_str()).ToElement();

	if (ParsingAncConfig(eleANC_Rear, m_strREAR_ANC, &readDataR) != NO_ERR)
	{
		TRACE(_T("[PWR] Config parsing fail. Err %d %s\n"), READ_FAIL_ANC_CONFIG, fileName);
		return READ_FAIL_ANC_CONFIG;
	}

	m_ConfigANCRear = readDataR;

	m_OpenANCConfigFile = true;
	return NO_ERR;
}

long CMachineConfig::ReadFileANCCal()
{
	m_OpenANCCalFile = false;

	long Err = READ_FAIL_ANC_CALIBRATION;
	CString strFunc(__func__);

	tinyxml2::XMLDocument docXML;

	std::string strROOT = CT2CA(m_strROOT);
	std::string strANC = CT2CA(m_strANC);
	//std::string strFRONT_ANC = CT2CA(m_strFrontANC);
	//std::string strREAR_ANC = CT2CA(m_strRearANC);

	CString strTemp;
	strTemp.Format(_T("%s\\%s"), (LPCTSTR)m_strDirectoryCal, (LPCTSTR)m_strCalibration_ANCxml);
	std::string strFileName = CT2CA(strTemp);

	ANC_STRUCT readDataF;
	ANC_STRUCT readDataR;
	tinyxml2::XMLError ErrXML = docXML.LoadFile(strFileName.c_str());

	if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] %s Open complete. %s\n"), strFunc, m_strCalibration_ANCxml);
	}
	else if (ErrXML == tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND)
	{
		TRACE(_T("[PWR] %s not found. %s\n"), strFunc, m_strCalibration_ANCxml);
		return NOT_FOUND_ANC_CALIBRATION;
	}
	else
	{
		TRACE(_T("[PWR] %s Open fail. Err %d %s\n"), strFunc, Err, m_strCalibration_ANCxml);
		return READ_FAIL_ANC_CALIBRATION;
	}

	tinyxml2::XMLHandle docHandle(&docXML);
	tinyxml2::XMLElement* eleANC = docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strANC.c_str()).ToElement();

	if (ParsingAncCal(eleANC, m_strFRONT_ANC, &readDataF) != NO_ERR)
	{
		TRACE(_T("[PWR] %s parsing fail. Err %d %s\n"), strFunc, Err, m_strFRONT_ANC);
		return READ_FAIL_ANC_CALIBRATION;
	}
	
	if (ParsingAncCal(eleANC, m_strREAR_ANC, &readDataR) != NO_ERR)
	{
		TRACE(_T("[PWR] %s parsing fail. Err %d %s\n"), strFunc, Err, m_strREAR_ANC);
		return READ_FAIL_ANC_CALIBRATION;
	}

	m_CalANCFront = readDataF;
	m_CalANCRear = readDataR;
	m_OpenANCCalFile = true;
	return NO_ERR;
}
long CMachineConfig::SaveFileANCCal(ANC_STRUCT data)
{
	long Err = SAVE_FAIL_ANC_CALIBRATION;

	CString strFunc(__func__);
	tinyxml2::XMLDocument docXML;

	std::string strROOT = CT2CA(m_strROOT);
	std::string strANC = CT2CA(m_strANC);

	CString strTemp;
	strTemp.Format(_T("%s\\%s"), (LPCTSTR)m_strDirectoryCal, (LPCTSTR)m_strCalibration_ANCxml);
	std::string strFileName = CT2CA(strTemp);

	ANC_STRUCT readData;
	tinyxml2::XMLError ErrXML = docXML.LoadFile(strFileName.c_str());

	if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config Open complete. %s\n"), m_strCalibration_ANCxml);
	}
	else
	{
		TRACE(_T("[PWR] Config Open fail. Err %d %s\n"), Err, m_strCalibration_ANCxml);
		return Err;
	}

	tinyxml2::XMLHandle docHandle(&docXML);
	tinyxml2::XMLElement* eleANC = docHandle.FirstChildElement(strROOT.c_str()).FirstChildElement(strANC.c_str()).ToElement();

	Err = UpdateANCCal(eleANC, data);
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Config update fail. Err %d %s\n"), Err, m_strCalibration_ANCxml);
		return Err;
	}

	ErrXML = docXML.SaveFile(strFileName.c_str());
	if (ErrXML != tinyxml2::XMLError::XML_SUCCESS)
	{
		TRACE(_T("[PWR] Config save fail. Err %d %s\n"), Err, m_strCalibration_ANCxml);
		return Err;
	}

	Err = ReadFileANCCal();
	if (Err != NO_ERR)
	{
		TRACE(_T("[PWR] Config reload fail. Err %d %s\n"), Err, m_strCalibration_ANCxml);
		return Err;
	}

	TRACE(_T("[PWR] Save complete. %s\n"), m_strCalibration_ANCxml);

	return NO_ERR;
}
long CMachineConfig::ParsingAncConfig(tinyxml2::XMLElement* eleANC, CString ancName, ANC_STRUCT* data)
{
	long Err = READ_FAIL_ANC_CONFIG;
	ANC_STRUCT readData;

	CString strFunc(__func__);

	std::string strROOT = "ROOT";
	std::string strANC = "ANC";
	std::string strNAME = "NAME";
	std::string strANC_USE = "ANC_USE";
	std::string strUPDOWN = "UPDOWN";
	std::string strANGLE_COMPEN = "ANGLE_COMPEN";
	std::string strHEAD_TORQUE = "HEAD_TORQUE";
	std::string strBASE = "BASE";
	std::string strHOLE_INFO = "HOLE_INFO";
	std::string strHOLE_NUMBER = "HOLE_NUMBER";
	std::string strHOLE_USE = "HOLE_USE";
	std::string strOFFSET_X = "OFFSET_X";
	std::string strOFFSET_Y = "OFFSET_Y";
	std::string strOFFSET_R = "OFFSET_R";
	std::string strFRONT = "FRONT";
	std::string strREAR = "REAR";

	tinyxml2::XMLElement* eleTemp;
	tinyxml2::XMLElement* eleTemp2;
	std::string strTemp, strTemp2;

	if (eleANC == NULL)
	{
		TRACE(_T("[PWR] Config %s %s is NULL.\n"), strFunc, ancName);
		return Err;
	}

	readData.Name.Empty();
	for (tinyxml2::XMLElement* ele = eleANC; ele != 0; ele = ele->NextSiblingElement(strANC.c_str()))
	{
		strTemp = strNAME;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->GetText() == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		if (ancName.CompareNoCase((CString)eleTemp->GetText()) != 0)
		{
			continue;
		}

		readData.Name = (CString)eleTemp->GetText();

		strTemp = strANC_USE;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->QueryBoolText(&readData.Use) != tinyxml2::XMLError::XML_SUCCESS)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		strTemp = strUPDOWN;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->QueryBoolText(&readData.UpdownType) != tinyxml2::XMLError::XML_SUCCESS)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		strTemp = strANGLE_COMPEN;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->QueryBoolText(&readData.AngleCompen) != tinyxml2::XMLError::XML_SUCCESS)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		strTemp = strHEAD_TORQUE;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->QueryDoubleText(&readData.HeadTorque) != tinyxml2::XMLError::XML_SUCCESS)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		strTemp = strBASE;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->GetText() == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		if (strcmp(eleTemp->GetText(), strFRONT.c_str()) == 0)
		{
			readData.base = FRONT_STAGE;
		}
		else if (strcmp(eleTemp->GetText(), strREAR.c_str()) == 0)
		{
			readData.base = REAR_STAGE;
		}
		else
		{
			TRACE(_T("[PWR] Config %s %s %s %s is invalid.\n"), strFunc, ancName, (CString)strTemp.c_str(), (CString)eleTemp->GetText());
			return Err;
		}

		strTemp = strHOLE_INFO;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		for (tinyxml2::XMLElement* eleHole = eleTemp; eleHole != 0; eleHole = eleHole->NextSiblingElement(strTemp.c_str()))
		{
			ANC_HOLE_STRUCT holeData;

			strTemp2 = strHOLE_NUMBER;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryIntText(&holeData.No) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strHOLE_USE;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryBoolText(&holeData.Use) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strOFFSET_X;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.x) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strOFFSET_Y;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.y) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strOFFSET_R;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.r) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			holeData.pt.z = 0.000;

			readData.Hole.push_back(holeData);
		}
	}

	if (readData.Name.IsEmpty() == true)
	{
		TRACE(_T("[PWR] Config %s %s not found.\n"), strFunc, ancName);
		return Err;
	}

	*data = readData;

	TRACE(_T("[PWR] Config %s %s read complete.\n"), strFunc, ancName);
	TRACE(_T("[PWR] %s %s %s %d %s %d %s %d.\n"), 
		(CString)strNAME.c_str(), data->Name, 
		(CString)strANC_USE.c_str(), data->Use, 
		(CString)strUPDOWN.c_str(), data->UpdownType,
		(CString)strBASE.c_str(), data->base);

	for (long idx = 0; idx < data->Hole.size(); idx++)
	{
		TRACE(_T("[PWR] %s %d %s %d %s %.3f %s %.3f %s %.3f.\n"),
			(CString)strHOLE_NUMBER.c_str(), data->Hole.at(idx).No,
			(CString)strHOLE_USE.c_str(), data->Hole.at(idx).Use,
			(CString)strOFFSET_X.c_str(), data->Hole.at(idx).pt.x,
			(CString)strOFFSET_Y.c_str(), data->Hole.at(idx).pt.y,
			(CString)strOFFSET_R.c_str(), data->Hole.at(idx).pt.r);
	}

	return NO_ERR;
}

long CMachineConfig::ParsingAncCal(tinyxml2::XMLElement* eleANC, CString ancName, ANC_STRUCT* data)
{
	long Err = READ_FAIL_ANC_CALIBRATION;
	ANC_STRUCT readData;

	CString strFunc(__func__);

	std::string strROOT = "ROOT";
	std::string strANC = "ANC";
	std::string strNAME = "NAME";
	std::string strANC_USE = "ANC_USE";
	std::string strUPDOWN = "UPDOWN";
	std::string strBASE = "BASE";
	std::string strHOLE_INFO = "HOLE_INFO";
	std::string strHOLE_NUMBER = "HOLE_NUMBER";
	std::string strHOLE_USE = "HOLE_USE";
	std::string strPOSITION_X = "POSITION_X";
	std::string strPOSITION_Y = "POSITION_Y";
	std::string strPOSITION_R = "POSITION_R";
	std::string strPOSITION_Z = "POSITION_Z";
	std::string strMARK_INFO = "MARK_INFO";
	std::string strMARK_NUMBER = "MARK_NUMBER";
	std::string strFRONT = "FRONT";
	std::string strREAR = "REAR";

	tinyxml2::XMLElement* eleTemp;
	tinyxml2::XMLElement* eleTemp2;
	std::string strTemp, strTemp2;

	if (eleANC == NULL)
	{
		TRACE(_T("[PWR] Config %s %s is NULL.\n"), strFunc, ancName);
		return Err;
	}

	readData.Name.Empty();
	for (tinyxml2::XMLElement* ele = eleANC; ele != 0; ele = ele->NextSiblingElement(strANC.c_str()))
	{
		strTemp = strNAME;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->GetText() == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		if (ancName.CompareNoCase((CString)eleTemp->GetText()) != 0)
		{
			continue;
		}

		readData.Name = (CString)eleTemp->GetText();

		strTemp = strHOLE_INFO;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		for (tinyxml2::XMLElement* eleHole = eleTemp; eleHole != 0; eleHole = eleHole->NextSiblingElement(strTemp.c_str()))
		{
			ANC_HOLE_STRUCT holeData;

			strTemp2 = strHOLE_NUMBER;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryIntText(&holeData.No) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			//strTemp2 = strHOLE_USE;
			//eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			//if (eleTemp2 == 0 || eleTemp2->QueryBoolText(&holeData.Use) != tinyxml2::XMLError::XML_SUCCESS)
			//{
			//	TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
			//	return Err;
			//}

			strTemp2 = strPOSITION_X;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.x) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strPOSITION_Y;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.y) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strPOSITION_R;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.r) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strPOSITION_Z;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&holeData.pt.z) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			readData.Hole.push_back(holeData);
		}



		strTemp = strMARK_INFO;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp.c_str());
			return Err;
		}

		for (tinyxml2::XMLElement* eleHole = eleTemp; eleHole != 0; eleHole = eleHole->NextSiblingElement(strTemp.c_str()))
		{
			ANC_MARK_STRUCT markData;

			strTemp2 = strMARK_NUMBER;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryIntText(&markData.No) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strPOSITION_X;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&markData.pt.x) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			strTemp2 = strPOSITION_Y;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryDoubleText(&markData.pt.y) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, ancName, (CString)strTemp2.c_str());
				return Err;
			}

			readData.Mark.push_back(markData);
		}
	}

	if (readData.Name.IsEmpty() == true)
	{
		TRACE(_T("[PWR] Config %s %s not found.\n"), strFunc, ancName);
		return Err;
	}

	*data = readData;

	TRACE(_T("[PWR] Config %s %s read complete.\n"), strFunc, ancName);
	for (long idx = 0; idx < data->Hole.size(); idx++)
	{
		TRACE(_T("[PWR] %s Hole %d XYRZ %.3f %.3f %.3f %.3f\n"),
			data->Name,
			data->Hole.at(idx).No,
			data->Hole.at(idx).pt.x, data->Hole.at(idx).pt.y, data->Hole.at(idx).pt.r, data->Hole.at(idx).pt.z);
	}

	for (long idx = 0; idx < data->Mark.size(); idx++)
	{
		TRACE(_T("[PWR] %s Mark %d XYRZ %.3f %.3f %.3f %.3f\n"),
			data->Name,
			data->Mark.at(idx).No,
			data->Mark.at(idx).pt.x, data->Mark.at(idx).pt.y);
	}

	return NO_ERR;
}
long CMachineConfig::UpdateANCCal(tinyxml2::XMLElement* eleEditANC, ANC_STRUCT data)
{
	long Err = SAVE_FAIL_ANC_CALIBRATION;
	ANC_STRUCT readData;

	CString strFunc(__func__);

	std::string strROOT = "ROOT";
	std::string strANC = "ANC";
	std::string strNAME = "NAME";
	std::string strANC_USE = "ANC_USE";
	std::string strUPDOWN = "UPDOWN";
	std::string strBASE = "BASE";
	std::string strHOLE_INFO = "HOLE_INFO";
	std::string strHOLE_NUMBER = "HOLE_NUMBER";
	std::string strHOLE_USE = "HOLE_USE";
	std::string strOFFSET_X = "POSITION_X";
	std::string strOFFSET_Y = "POSITION_Y";
	std::string strOFFSET_R = "POSITION_R";
	std::string strOFFSET_Z = "POSITION_Z";
	std::string strMARK_INFO = "MARK_INFO";
	std::string strMARK_NUMBER = "MARK_NUMBER";
	std::string strFRONT = "FRONT";
	std::string strREAR = "REAR";

	tinyxml2::XMLElement* eleTemp;
	tinyxml2::XMLElement* eleTemp2;
	std::string strTemp, strTemp2;

	if (eleEditANC == NULL)
	{
		TRACE(_T("[PWR] Config %s %s is NULL.\n"), strFunc, data.Name);
		return Err;
	}

	for (tinyxml2::XMLElement* ele = eleEditANC; ele != 0; ele = ele->NextSiblingElement(strANC.c_str()))
	{
		strTemp = strNAME;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0 || eleTemp->GetText() == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp.c_str());
			return Err;
		}

		if (data.Name.CompareNoCase((CString)eleTemp->GetText()) != 0)
		{
			continue;
		}

		strTemp = strHOLE_INFO;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp.c_str());
			return Err;
		}

		for (tinyxml2::XMLElement* eleHole = eleTemp; eleHole != 0; eleHole = eleHole->NextSiblingElement(strTemp.c_str()))
		{
			ANC_HOLE_STRUCT holeData;
			int holeNo;
			strTemp2 = strHOLE_NUMBER;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryIntText(&holeNo) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
				return Err;
			}

			for (int i = 0; i < data.Hole.size(); i++)
			{
				if (data.Hole.at(i).No == holeNo)
				{
					holeData = data.Hole.at(i);

					CString strX, strY, strR, strZ;
					strX.Format(_T("%.3f"), holeData.pt.x);
					strY.Format(_T("%.3f"), holeData.pt.y);
					strR.Format(_T("%.3f"), holeData.pt.r);
					strZ.Format(_T("%.3f"), holeData.pt.z);

					CT2A cstrX(strX);
					CT2A cstrY(strY);
					CT2A cstrR(strR);
					CT2A cstrZ(strZ);

					strTemp2 = strOFFSET_X;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}					
					eleTemp2->SetText(cstrX);

					strTemp2 = strOFFSET_Y;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}
					eleTemp2->SetText(cstrY);

					strTemp2 = strOFFSET_R;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}
					eleTemp2->SetText(cstrR);

					strTemp2 = strOFFSET_Z;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}
					eleTemp2->SetText(cstrZ);

					
					TRACE(_T("[PWR] %s Hole %d XYRZ %.3f %.3f %.3f %.3f\n"),
						data.Name,
						holeData.No,
						holeData.pt.x, holeData.pt.y, holeData.pt.r, holeData.pt.z);					

					break;
				}
			}
		}


		strTemp = strMARK_INFO;
		eleTemp = ele->FirstChildElement(strTemp.c_str());
		if (eleTemp == 0)
		{
			TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp.c_str());
			return Err;
		}

		for (tinyxml2::XMLElement* eleHole = eleTemp; eleHole != 0; eleHole = eleHole->NextSiblingElement(strTemp.c_str()))
		{
			ANC_MARK_STRUCT MarkData;
			int MarkNo;
			strTemp2 = strMARK_NUMBER;
			eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
			if (eleTemp2 == 0 || eleTemp2->QueryIntText(&MarkNo) != tinyxml2::XMLError::XML_SUCCESS)
			{
				TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
				return Err;
			}

			for (int i = 0; i < data.Mark.size(); i++)
			{
				if (MarkNo != 1 && MarkNo != 2)
				{
					TRACE(_T("[PWR] Config %s %s %s mark number error.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
					return Err;
				}

				if (data.Mark.at(i).No == MarkNo)
				{
					MarkData = data.Mark.at(i);

					CString strX, strY, strR, strZ;
					strX.Format(_T("%.3f"), MarkData.pt.x);
					strY.Format(_T("%.3f"), MarkData.pt.y);	

					CT2A cstrX(strX);
					CT2A cstrY(strY);
	
					strTemp2 = strOFFSET_X;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}
					eleTemp2->SetText(cstrX);

					strTemp2 = strOFFSET_Y;
					eleTemp2 = eleHole->FirstChildElement(strTemp2.c_str());
					if (eleTemp2 == 0)
					{
						TRACE(_T("[PWR] Config %s %s %s is NULL.\n"), strFunc, data.Name, (CString)strTemp2.c_str());
						return Err;
					}
					eleTemp2->SetText(cstrY);

					TRACE(_T("[PWR] %s Mark %d XYRZ %.3f %.3f %.3f %.3f\n"),
						data.Name,
						MarkData.No,
						MarkData.pt.x, MarkData.pt.y);

					break;
				}
			}
		}
	}

	return NO_ERR;
}

long CMachineConfig::EditANCCalMark(long base, ANC_MARK_STRUCT data)
{
	long Err;

	Err = ReadFileANCCal();
	if (Err != NO_ERR)
	{
		return Err;
	}

	ANC_STRUCT calData = GetCalANC(base);

	for (long idx = 0; idx < calData.Mark.size(); idx++)
	{
		if (calData.Mark.at(idx).No == data.No)
		{
			calData.Mark.at(idx) = data;
			Err = SaveFileANCCal(calData);
			return Err;
		}
	}

	return SAVE_FAIL_ANC_CALIBRATION;
}

bool CMachineConfig::GetOpenConfigFile()
{
	return m_OpenConfigFile;
}

bool CMachineConfig::IsExistANCData()
{
	if (m_OpenANCConfigFile == true && m_OpenANCCalFile == true)
	{
		return true;
	}

	return false;
}

ANC_STRUCT CMachineConfig::GetConfigANC(long Base)
{
	if (Base == FRONT_STAGE)
	{
		return m_ConfigANCFront;
	}

	return m_ConfigANCRear;

}

bool CMachineConfig::GetConfigANC(long Base, ANC_STRUCT* data)
{
	if (IsExistANCData() == true)
	{
		if (Base == FRONT_STAGE)
		{
			*data = m_ConfigANCFront;
		}
		else
		{
			*data = m_ConfigANCRear;

		}

		return true;
	}

	return false;
}

ANC_STRUCT CMachineConfig::GetCalANC(long Base)
{
	if (Base == FRONT_STAGE)
	{
		return m_CalANCFront;
	}

	return m_CalANCRear;
}

bool CMachineConfig::GetConfigANCHole(long Base, long holeNo, ANC_HOLE_STRUCT* holeData)
{
	ANC_STRUCT temp;

	if (Base == FRONT_STAGE)
	{
		temp = m_ConfigANCFront;
	}
	else
	{
		temp = m_ConfigANCRear;
	}

	for (long idx = 0; idx < temp.Hole.size(); idx++)
	{
		if (temp.Hole.at(idx).No == holeNo)
		{
			*holeData = temp.Hole.at(idx);
			return true;
		}
	}

	return false;
}

 bool CMachineConfig::GetCalANCHole(long Base, long holeNo, ANC_HOLE_STRUCT* holeData)
{
	ANC_STRUCT temp;

	if (Base == FRONT_STAGE)
	{
		temp = m_CalANCFront;
	}
	else
	{
		temp = m_CalANCRear;
	}

	for (long idx = 0; idx < temp.Hole.size(); idx++)
	{
		if (temp.Hole.at(idx).No == holeNo)
		{
			*holeData = temp.Hole.at(idx);
			return true;
		}
	}

	return false;
}
 
 bool CMachineConfig::IsANCUpType(long Base)
 {
	 if (IsExistANCData() == true)
	 {
		 return GetConfigANC(Base).UpdownType;
	 }

	 return false;
 }


 long CMachineConfig::GetCameraVersion(long CamNo, int* Version)
 {
	 long Err = READ_FAIL_ANC_CALIBRATION;
	 int CamNoAttribute = 0;
	 CString strFunc(__func__);

	 tinyxml2::XMLDocument docXML;
	 tinyxml2::XMLElement* eleTemp2;
	 CString strTemp;
	 strTemp.Format(_T("%s\\Camera\\Camera.xml"), (LPCTSTR)m_strDirectoryConfig);
	 std::string strFileName = CT2CA(strTemp);
	 tinyxml2::XMLError ErrXML = docXML.LoadFile(strFileName.c_str());

	 int value;

	 if (ErrXML == tinyxml2::XMLError::XML_SUCCESS)
	 {
		 TRACE(_T("[PWR] %s Open complete. %s\n"), strFunc, strTemp);
	 }
	 else if (ErrXML == tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND)
	 {
		 TRACE(_T("[PWR] %s not found. %s\n"), strFunc, strTemp);
		 return NOT_FOUND_ANC_CALIBRATION;
	 }
	 else
	 {
		 TRACE(_T("[PWR] %s Open fail. Err %d %s\n"), strFunc, Err, strTemp);
		 return Err;
	 }

	 tinyxml2::XMLHandle docHandle(&docXML);
	 tinyxml2::XMLElement* eleANC = docHandle.FirstChildElement("ROOT").FirstChildElement("CAMERA").ToElement();

	 for (tinyxml2::XMLElement* ele = eleANC; ele != 0; ele = ele->NextSiblingElement("CAMERA"))
	 {
		 if (ele->QueryAttribute("NUMBER", &CamNoAttribute) == tinyxml2::XMLError::XML_SUCCESS && CamNoAttribute == CamNo)
		 {
			 eleTemp2 = ele->FirstChildElement("VERSION");
			 if (eleTemp2 == 0 || eleTemp2->QueryIntText(&value) != tinyxml2::XMLError::XML_SUCCESS)
			 {
				 TRACE(_T("[PWR] %s VERSION is NULL.\n"), strFunc);
				 return Err;
			 }

			 TRACE(_T("[PWR] %s Cam:%d version:%d\n"), strFunc, CamNo, value);
			 *Version = value;
			 return NO_ERR;
		 }
	 }

	 return Err;
 }

 long CMachineConfig::GetOptionCameraLaser()
 {
	 return m_OptionCameraLaser;
 }
