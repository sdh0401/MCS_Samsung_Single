#include "pch.h"
#include "CTokenizer.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

CTokenizer::CTokenizer()
{
}

CTokenizer::~CTokenizer(void)
{
	arToken.RemoveAll();
}

CTokenizer::CTokenizer(CString sourceLine, CString strDelimiter, BOOL bAddEmpty, BOOL bTrim)
{
	arToken.RemoveAll();
	Split(sourceLine,strDelimiter,arToken,bAddEmpty,bTrim);
}

CTokenizer::CTokenizer(CString sourceLine, CStringArraySafe* deliminatorList, BOOL bAddEmpty, BOOL bTrim)
{
	arToken.RemoveAll();
	Split(sourceLine,deliminatorList,arToken,bAddEmpty,bTrim);
}
void CTokenizer::DoTokenizing(CString sourceLine, CString strDelimiter, BOOL bAddEmpty, BOOL bTrim)
{
	arToken.RemoveAll();
	Split(sourceLine,strDelimiter,arToken,bAddEmpty,bTrim);
}

void CTokenizer::DoTokenizing(CString sourceLine, CStringArraySafe* deliminatorList, BOOL bAddEmpty, BOOL bTrim)
{
	arToken.RemoveAll();
	Split(sourceLine,deliminatorList,arToken,bAddEmpty,bTrim);
}

int CTokenizer::GetCount(void)
{
	return arToken.GetCount();
}

BOOL CTokenizer::IsNumber(int nIndex)
{
	int nStartIndex = 0;
	BOOL bResult = TRUE;
	int nPointCount = 0;
	CString strToken = GetString(nIndex);
	if(strToken.GetLength() == 0)
	{
		bResult = FALSE;
	}
	else if(strToken.GetLength() == 1)
	{
		if(isdigit(strToken[0]) == FALSE) bResult = FALSE;
	}
	else
	{
		if(strToken[0] == '-') nStartIndex = 1;
		for(int i=nStartIndex; i< strToken.GetLength(); i++)
		{
			if(strToken[i] != '.')
			{
				if(isdigit(strToken[i]) == FALSE) 
				{
					bResult = FALSE;
					break;
				}
			}
			else nPointCount++;
		}
	}
	if(nPointCount > 1) bResult = FALSE;
	return bResult;
}

void CTokenizer::GetItem(int nIndex, CString& strValue)
{
	ASSERT(nIndex < arToken.GetCount());
	strValue = arToken.GetAt(nIndex);
}

void CTokenizer::GetItem(int nIndex, int& nValue)
{
	int nPointCount = 0;
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	nValue = _ttoi(strToken);
}

void CTokenizer::GetItem(int nIndex, UINT& nValue)
{
	int nPointCount = 0;
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	nValue = (UINT)_ttoi(strToken);
}

void CTokenizer::GetItem(int nIndex, double& dValue)
{
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	dValue = (double)_wtof(strToken);
}

void CTokenizer::GetItem(int nIndex, float& fValue)
{
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	fValue = (float)_tstof(strToken);
}

void CTokenizer::GetItem(int nIndex, CTime& tValue)
{
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	ASSERT(strToken.GetLength() == 14);
	int nYear = _wtoi(strToken.Left(4)); //_wtoi(strValue.Left(4));
	int nMon  = _wtoi(strToken.Mid(4,2));
	int nDay  = _wtoi(strToken.Mid(6,2));
	int nHour = _wtoi(strToken.Mid(8,2));
	int nMin  = _wtoi(strToken.Mid(10,2));
	int nSec  = _wtoi(strToken.Mid(12,2));
	CTime tiTime(nYear, nMon, nDay, nHour, nMin, nSec);
	tValue = tiTime;
}

CString CTokenizer::GetString(int nIndex)
{
	ASSERT(nIndex < arToken.GetCount());
	return arToken.GetAt(nIndex);
}

int CTokenizer::GetInt(int nIndex)
{
	int nValue;
	BOOL bResult = TRUE;
	int nPointCount = 0;
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	nValue = _ttoi(strToken);
	return nValue;
}

float CTokenizer::GetFloat(int nIndex)
{
	float dValue;
	BOOL bResult = TRUE;
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	dValue = (float)_tstof(strToken);
	return dValue;
}

double CTokenizer::GetDouble(int nIndex)
{
	double dValue;
	BOOL bResult = TRUE;
	CString strToken = GetString(nIndex);
	ASSERT(IsNumber(nIndex));
	dValue = _wtof(strToken);
	return dValue;
}

CTime CTokenizer::GetTime(int nIndex)
{
//	int nValue;
	BOOL bResult = TRUE;
	int nPointCount = 0;
	ASSERT(IsNumber(nIndex));
	CString strValue = GetString(nIndex);
	ASSERT(strValue.GetLength() == 14);
	int nYear = _ttoi(strValue.Left(4));//_wtoi(strValue.Left(4));
	int nMon = _ttoi(strValue.Mid(4,2));
	int nDay = _ttoi(strValue.Mid(6,2));
	int nHour = _ttoi(strValue.Mid(8,2));
	int nMin = _ttoi(strValue.Mid(10,2));
	int nSec = _ttoi(strValue.Mid(12,2));
	CTime tiTime(nYear, nMon, nDay, nHour, nMin, nSec);
	return tiTime;
}

BOOL CTokenizer::RemoveToken(int nIndex)
{
	ASSERT(nIndex < arToken.GetCount());
	arToken.RemoveAt(nIndex);
	return TRUE;
}

void CTokenizer::RemoveAll(void)
{
	arToken.RemoveAll();
}

void CTokenizer::AddOneItem(UINT nValue)
{
	CString strValue;
	strValue.Format(_T("%ud"),nValue);
	arToken.Add(strValue);
}
void CTokenizer::AddOneItem(int nValue)
{
	CString strValue;
	strValue.Format(_T("%d"),nValue);
	arToken.Add(strValue);
}
void CTokenizer::AddOneItem(double dValue)
{
	CString strValue;
	strValue.Format(_T("%f"),dValue);
	arToken.Add(strValue);
}
void CTokenizer::AddOneItem(float dValue)
{
	CString strValue;
	strValue.Format(_T("%f"),dValue);
	arToken.Add(strValue);
}
void CTokenizer::AddOneItem(CString strValue)
{
	arToken.Add(strValue);
}
void CTokenizer::AddOneItem(CTime tValue)
{
	CString strValue;
	strValue = tValue.Format(_T("%Y%m%d%H%M%S"));
	arToken.Add(strValue);
}
BOOL CTokenizer::MakeAsOneString(CString strDelimiter, CString& strResult)
{
	BOOL bResult = TRUE;
	CString strTemp;
	strResult = "";
	for(int i=0; i<arToken.GetCount(); i++)
	{
		strTemp = arToken.GetAt(i);
		if(strTemp.FindOneOf(strDelimiter) != -1)
		{
			bResult = FALSE;
			break;
		}
		if(i != arToken.GetCount() - 1) strTemp += strDelimiter;
		strResult += strTemp;
	}
//	arToken.RemoveAll();
	return bResult;
}

CString CTokenizer::MakeAsOneString(CString strDelimiter)
{
	CString strTemp;
	CString strResult = _T("");
	for(int i=0; i<arToken.GetCount(); i++)
	{
		strTemp = arToken.GetAt(i);
		if(strTemp.FindOneOf(strDelimiter) != -1)
		{
			strResult  = "";
			break;
		}
		if(i != arToken.GetCount() - 1) strTemp += strDelimiter;
		strResult += strTemp;
	}
	return strResult;
}

CString CTokenizer::FindString(CString&tmp, CString Search, BOOL bAddEmpty)
{
	int		nFound=-1;

	CString csRet = _T("");
	CString csTmp1 = tmp;

	if (Search.IsEmpty()) 	Search = ","; 
	if (!bAddEmpty) 
	{
		if (tmp.Left(Search.GetLength()) == Search) 
		{
			do 
			{
				// get the 
				tmp = csTmp1.Mid(Search.GetLength());
				csTmp1 = tmp;
			} while (tmp.Left(Search.GetLength()) == Search);
			tmp = csTmp1.Mid(Search.GetLength() - 1);
		}
	}
	CString csTmp = tmp;
	nFound = tmp.Find(Search,0);
	if (nFound >= 0) 
	{
		csRet = csTmp.Left(nFound);
		tmp = csTmp.Mid(nFound + (Search.GetLength()));
	}
	else 
	{
		csRet = csTmp;
		tmp = "";
	}
	return csRet;
}


CString CTokenizer::Join(CString strDelimiter, CStringArraySafe& listOfResult)
{
	CString csReturn;
	CString csTmp;

	if (strDelimiter.IsEmpty()) 		strDelimiter = ","; 
	for( int iNum = 0; iNum < listOfResult.GetCount(); iNum++ ) 
	{
		csTmp += listOfResult.GetAt(iNum);
		csTmp += strDelimiter;
	}
	csReturn = csTmp.Left(csTmp.GetLength() - 1);
	return csReturn;
}

void CTokenizer::Split(CString sourceLine, CString strDelimiter, CStringArraySafe& listOfResult, BOOL bAddEmpty,BOOL bTrim)
{
	// initialize the variables
	CString		 newCString;// = Source;
	CString		 tmpCString = _T("");
	CString		 AddCString = _T("");

	int pos1 = 0;
	int pos = 0;
	listOfResult.RemoveAll();
	if (strDelimiter.IsEmpty()) 		strDelimiter = ",";
	newCString = sourceLine;
	do 
	{
		pos1 = 0;
		pos = newCString.Find(strDelimiter, pos1);
		if ( pos != -1 ) 
		{
			AddCString = newCString.Left(pos);
			if(bTrim) AddCString.Trim();
			if (!AddCString.IsEmpty()) 	listOfResult.Add(AddCString);
			else if (bAddEmpty) 		listOfResult.Add(AddCString);
			ASSERT(strDelimiter.GetLength() > 0);
			tmpCString = newCString.Mid(pos + strDelimiter.GetLength());
			newCString = tmpCString;
		}
	} while ( pos != -1 );

	if (!newCString.IsEmpty()) 		listOfResult.Add(newCString);
}

void CTokenizer::Split(CString sourceLine, CString strDelimiter, CStringArraySafe* listOfResult, BOOL bAddEmpty,BOOL bTrim)
{
	// initialize the variables
	CString		 newCString;// = Source;
	CString		 tmpCString = _T("");
	CString		 AddCString = _T("");

	int pos1 = 0;
	int pos = 0;
	ASSERT(listOfResult != NULL);
	listOfResult->RemoveAll();
	if (strDelimiter.IsEmpty()) 		strDelimiter = ",";
	newCString = sourceLine;
	do 
	{
		pos1 = 0;
		pos = newCString.Find(strDelimiter, pos1);
		if ( pos != -1 ) 
		{
			AddCString = newCString.Left(pos);
			if(bTrim) AddCString.Trim();
			if (!AddCString.IsEmpty()) 	listOfResult->Add(AddCString);
			else if (bAddEmpty) 		listOfResult->Add(AddCString);
			tmpCString = newCString.Mid(pos + strDelimiter.GetLength());
			newCString = tmpCString;
		}
	} while ( pos != -1 );

	if (!newCString.IsEmpty()) 		listOfResult->Add(newCString);
}

void CTokenizer::Split(CString sourceLine, CStringArraySafe* deliminatorList, CStringArraySafe& listOfResult, BOOL bAddEmpty,BOOL bTrim)
{
	// initialize the variables
	CString		 newCString;// = Source;
	CString		 tmpCString = _T("");
	CString		 AddCString = _T("");
	CString      strDelimitor = _T("");
	CString      strTemp;
	int pos1 = 0;
	int pos = 0;
	ASSERT(deliminatorList->GetCount() > 0);
	for(int i=0; i<deliminatorList->GetCount(); i++)
	{
		strTemp = deliminatorList->GetAt(i);
		if(strTemp.IsEmpty()) continue;
		if(strDelimitor == "") strDelimitor = strTemp;
		else if(strDelimitor != strTemp)
		{
			sourceLine.Replace(strTemp, strDelimitor);
		}
	}
	ASSERT(!strDelimitor.IsEmpty());
	newCString = sourceLine;
	do 
	{
		pos1 = 0;
		pos = newCString.Find(strDelimitor, pos1);
		if ( pos != -1 ) 
		{
			AddCString = newCString.Left(pos);
			if(bTrim) AddCString.Trim();
			if (!AddCString.IsEmpty()) 	listOfResult.Add(AddCString);
			else if (bAddEmpty) 		listOfResult.Add(AddCString);
			tmpCString = newCString.Mid(pos + strDelimitor.GetLength());
			newCString = tmpCString;
		}
	} while ( pos != -1 );

	if (!newCString.IsEmpty()) 		listOfResult.Add(newCString);
}

//void CTokenizer::SplitPath (CString Path, CString& Drive, CString& Dir, CString& FName, CString& Ext)
//{
//	TCHAR path_buffer[_MAX_PATH];
//	TCHAR drive[_MAX_DRIVE];
//	TCHAR dir[_MAX_DIR];
//	TCHAR fname[_MAX_FNAME];
//	TCHAR ext[_MAX_EXT];
//	// Copy the path
//	_tcsncpy_s( path_buffer, Path, sizeof( path_buffer ) - 1 );
//	// Split the given path
//	_tsplitpath_s( path_buffer, drive, dir, fname, ext );
//	Drive.Format(_T("%s"),drive);
//	Dir.Format(_T("%s"),dir);
//	FName.Format(_T("%s"),fname);
//	Ext.Format(_T("%s"),ext);
//}

CStringArraySafe::CStringArraySafe()
{

}
CStringArraySafe::~CStringArraySafe()
{
	arToken.RemoveAll();
}
CString CStringArraySafe::GetAt(int nIndex)
{
	return arToken.GetAt(nIndex);
}
void CStringArraySafe::Add(CString str)
{
	arToken.Add(str);
}
void CStringArraySafe::RemoveAll()
{
	arToken.RemoveAll();
}
void CStringArraySafe::RemoveAt(int nIndex)
{
	arToken.RemoveAt(nIndex);
}
int CStringArraySafe::GetCount()
{
	return (int)arToken.GetCount();
}