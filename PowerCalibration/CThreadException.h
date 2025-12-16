#pragma once
#include <afx.h>
enum TEXCEPTION_CODE { T_NONECODE, T_TERMINATE };
class CThreadException :
	public CException
{
public:
	CThreadException(int nCode);
	int m_nThreadErrorCode;
};

