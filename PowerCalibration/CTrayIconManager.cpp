#include "pch.h"
#include "GlobalDefine.h"
#include "PowerCalibration.h"
#include "CTrayIconManager.h"
#include "GlobalData.h"
#include "Trace.h"

void CTrayIconManager::MakePopupMenu(HWND hWnd, int x, int y)
{
    HMENU hMenu = CreatePopupMenu();
    if (m_bTrayHide)
        AppendMenu(hMenu, MF_STRING, WM_MCS_DIALOG_SHOW, _T("Show"));
    else
        AppendMenu(hMenu, MF_STRING, WM_MCS_DIALOG_SHOW, _T("Hide"));
    AppendMenu(hMenu, MF_STRING, WM_MCS_APP_EXIT, _T("Exit"));
    SetForegroundWindow(hWnd);
    TrackPopupMenu(hMenu, TPM_LEFTALIGN | TPM_RIGHTBUTTON, x, y, 0, hWnd, NULL);
}

void CTrayIconManager::ProcTrayMsg(HWND hWnd, WPARAM wParam, LPARAM lParam)
{
    HMENU hMenu = NULL;
    POINT pos;
    if (lParam == WM_LBUTTONDOWN)
    {
        GetCursorPos(&pos);
        MakePopupMenu(hWnd, pos.x, pos.y);
    }
}

bool CTrayIconManager::AddTrayIcon(HWND hWnd)
{
    if (m_bTrayIconAdded)
        return FALSE;
    NOTIFYICONDATA nid;    //아이콘을생성하여설정
    nid.cbSize = sizeof(NOTIFYICONDATA);
    nid.hWnd = hWnd;
    nid.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
    lstrcpy(nid.szTip, _T("PowerMCS"));
    nid.uID = 0;
    nid.uCallbackMessage = WM_MCS_TRAYICON;
    nid.hIcon = theApp.LoadIcon(IDR_MAINFRAME);
    if (Shell_NotifyIcon(NIM_ADD, &nid) == 0)
        return FALSE;
    m_bTrayIconAdded = true;
    return TRUE;
}

bool CTrayIconManager::DelTrayIcon(HWND hWnd)
{
    NOTIFYICONDATA nid;
    unsigned retry = 0;
    ZeroMemory(&nid, sizeof(NOTIFYICONDATA));
    nid.cbSize = sizeof(NOTIFYICONDATA);
    if (hWnd != NULL)
    {
        nid.hWnd = hWnd;
        nid.uFlags = NULL;
        nid.uID = 0x100;
        for (retry = 0; retry < 10; ++retry)
        {
            if (Shell_NotifyIcon(NIM_DELETE, &nid) == 0)
            {
                TRACE(_T("[PWR] DelTrayIcon return 0\n"));
            }
            else
            {
                TRACE(_T("[PWR] DelTrayIcon return else 0\n"));
                break;
            }
            ThreadSleep(TIME100MS);
        }
        if (retry == 10)
            return FALSE;
        return TRUE;
    }
    TRACE(_T("[PWR] hWnd is NULL\n"));
    return FALSE;
}
