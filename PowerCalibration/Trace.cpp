#include "pch.h"
#include <string>

void CustomTrace(const char* format, ...)
{
	const int TraceBufferSize = 1024;
	char buffer[TraceBufferSize];
	va_list argptr; va_start(argptr, format);
	vsprintf_s(buffer, format, argptr);
	va_end(argptr);
	::OutputDebugStringA(buffer);
}
void CustomTrace(const wchar_t* format, ...)
{
	const int TraceBufferSize = 1024;
	wchar_t buffer[TraceBufferSize];
	va_list argptr; va_start(argptr, format);
	vswprintf_s(buffer, format, argptr);
	va_end(argptr);
	::OutputDebugStringW(buffer);
}
void CustomTrace(int dwCategory, int line, const char* format, ...)
{
	va_list argptr; va_start(argptr, format);
	CustomTrace(format, argptr);
	va_end(argptr);
}
void CustomTrace(int dwCategory, int line, const wchar_t* format, ...)
{
	va_list argptr; va_start(argptr, format);
	CustomTrace(format, argptr);
	va_end(argptr);
}

