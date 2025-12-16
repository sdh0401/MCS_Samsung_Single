#pragma once
#include "StackWalker.h"
class CPowerStackWalker : public StackWalker
{
public:
    CPowerStackWalker() : StackWalker() {}
    CPowerStackWalker(DWORD dwProcessId, HANDLE hProcess) : StackWalker(dwProcessId, hProcess) {}
    virtual void OnOutput(LPCSTR szText) { printf(szText); StackWalker::OnOutput(szText); }
};

extern int DemoStackWalker();
extern LONG WINAPI ExpFilter(EXCEPTION_POINTERS* pExp, DWORD dwExpCode);
