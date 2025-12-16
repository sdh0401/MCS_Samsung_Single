#pragma once

class CStringArraySafe
{
public:
	CStringArraySafe();
	~CStringArraySafe();
private:
	CStringArray arToken;
public:
	CString GetAt(int nIndex);
	void Add(CString str);
	void RemoveAll();
	void RemoveAt(int nIndex);
	int GetCount();
};

class CTokenizer
{
public:
	CTokenizer();
	CTokenizer(CString sourceLine, CString strDelimiter,  BOOL bAddEmpty=TRUE, BOOL bTrim = TRUE);
	CTokenizer(CString sourceLine, CStringArraySafe* deliminatorList, BOOL bAddEmpty=TRUE, BOOL bTrim = TRUE);
	void DoTokenizing(CString sourceLine, CString strDelimiter,  BOOL bAddEmpty=TRUE, BOOL bTrim = TRUE);
	void DoTokenizing(CString sourceLine, CStringArraySafe* deliminatorList, BOOL bAddEmpty=TRUE, BOOL bTrim = TRUE);
	int GetCount(void);
	CString GetString(int nIndex);
	BOOL IsNumber(int nIndex);
	int GetInt(int nIndex);
	float GetFloat(int nIndex);
	double GetDouble(int nIndex);
	CTime GetTime(int nIndex);
	void GetItem(int nIndex, CString& strValue);
	void GetItem(int nIndex, int& nValue);
	void GetItem(int nIndex, UINT& nValue);
	void GetItem(int nIndex, double& dValue);
	void GetItem(int nIndex, float& fValue);
	void GetItem(int nIndex, CTime& tValue);
	BOOL RemoveToken(int nIndex);
	void RemoveAll(void);

	void AddOneItem(UINT nValue);
	void AddOneItem(int nValue);
	void AddOneItem(float dValue);
	void AddOneItem(double dValue);
	void AddOneItem(CString strValue);
	void AddOneItem(CTime tValue);
	BOOL MakeAsOneString(CString strDelimiter, CString& strResult);
	CString MakeAsOneString(CString strDelimiter);
	~CTokenizer();
	//static void SplitPath(CString Path, CString& Drive, CString& Dir, CString& FName, CString& Ext);
	static void Split(CString sourceLine, CString strDelimiter, CStringArraySafe& listOfResult, BOOL bAddEmpty = TRUE, BOOL bTrim = TRUE);
	static void Split(CString sourceLine, CString strDelimiter, CStringArraySafe* listOfResult, BOOL bAddEmpty = TRUE, BOOL bTrim = TRUE);
	static void Split(CString sourceLine, CStringArraySafe* deliminatorList, CStringArraySafe& listOfResult, BOOL bAddEmpty = TRUE, BOOL bTrim = TRUE);
	static CString Join(CString strDelimiter, CStringArraySafe& listOfResult);
	static CString FindString(CString& csRefered, CString strDelimiter, BOOL bAddEmpty);
private:
	CStringArraySafe arToken;
protected:
};
