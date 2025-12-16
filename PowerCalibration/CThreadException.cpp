#include "pch.h"
#include "CThreadException.h"

CThreadException::CThreadException(int nCode)
{
	m_nThreadErrorCode = nCode;
}