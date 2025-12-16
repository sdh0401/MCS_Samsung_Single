#pragma once
class CTrayIconManager :
	public CObject
{
public:
	bool m_bTrayIconAdded, m_bTrayHide;
	void MakePopupMenu(HWND hWnd, int x, int y);
    void ProcTrayMsg(HWND hWnd, WPARAM wParam, LPARAM lParam);
    bool AddTrayIcon(HWND hWnd);
    bool DelTrayIcon(HWND hWnd);
};

