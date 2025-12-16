#pragma once
class CPowerCleaner
{
public:
	CPowerCleaner();
	~CPowerCleaner();
	static UINT StartCleaner(LPVOID wParam);
	void Run();
private:
	bool m_bLoop;
};

extern CPowerCleaner* gcPowerCleaner;
