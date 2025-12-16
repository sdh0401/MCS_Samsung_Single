#include "pch.h"
#include "Io_state_detector.h"
#include "Trace.h"
#include <typeinfo>
#include "GlobalIODefine.h"
#include "CPowerIO.h"
#include "CPowerSwitchPanel.h"
#include "AxisInformation.h"
#include <string>
#include "GlobalData.h"
Io_state_detector& Io_state_detector::GetInstance()
{
	static Io_state_detector iostatus;
	return iostatus;
}

Io_state_detector::Io_state_detector()
	: mtx(std::mutex())
	, isRunning(false)
	, isEnd(true)
	, is_monopolize_panel_switch(false)
{

}

Io_state_detector::~Io_state_detector()
{

}

Io_state_detector& Io_state_detector::operator=(const Io_state_detector& ref)
{
	return *this;
}

UINT Io_state_detector::Run(LPVOID parameter)
{
	//null pointer exception
	if (gcPowerIO == NULL)
	{
		TRACE("[PWR] %s::%s -> gcPowerIO is NULL -> wrong code!", typeid(Io_state_detector).name(), __func__);
		return 0;//여기 걸리면 내 잘못
	}
	if (gcPowerSwitchPanel == NULL)
	{
		TRACE("[PWR] %s::%s -> gcPowerSwitchPanel is NULL -> wrong code!", typeid(Io_state_detector).name(), __func__);
		return 0;//여기 걸리면 내 잘못
	}

	//pThis 찾기
	Io_state_detector& pThis = Io_state_detector::GetInstance();

	//이미 도는지 체크
	if (pThis.getRunning() == true)
	{
		TRACE("[PWR] %s::%s -> already running", typeid(Io_state_detector).name(), __func__);
		return 0;//이미 돌고있으면 종료
	}

	//아무도 안돌면 내가 돌거라는 뜻
	pThis.setRunning(true);
	pThis.setEnd(false);
	TRACE("[PWR] %s::%s -> START!", typeid(Io_state_detector).name(), __func__);

	//처음 한번 읽기
	char  stop = gcPowerIO->di1(IN_STOP_PANEL_KEY)
		, oldStop = stop

		, door = gcPowerIO->di1(IN_FDOOR_LOCK_PANEL_KEY)
		, oldDoor = door

		, start = gcPowerIO->di1(IN_START_PANEL_KEY)
		, oldStart = start

		, reset = gcPowerIO->di1(IN_RESET_PANEL_KEY)
		, oldReset = reset

		, red = gcPowerIO->Getdo1(OUT_FLAMP_RED)
		, oldRed = red

		, green = gcPowerIO->Getdo1(OUT_FLAMP_GRN)
		, oldGreen = green

		, yellow = gcPowerIO->Getdo1(OUT_FLAMP_YEL)
		, oldYellow = yellow

		, in_fconv_entry_ent = gcPowerIO->di1(IN_FCONV_ENTRY_ENT)
		, old_in_fconv_entry_ent = in_fconv_entry_ent

		, in_fconv_entry_exist = gcPowerIO->di1(IN_FCONV_ENTRY_EXIST)
		, old_in_fconv_entry_exist = in_fconv_entry_exist

		, in_fconv_work1_low = gcPowerIO->di1(IN_FCONV_WORK1_LOW)
		, old_in_fconv_work1_low = in_fconv_work1_low

		, in_fconv_work1_exist = gcPowerIO->di1(IN_FCONV_WORK1_EXIST)
		, old_in_fconv_work1_exist = in_fconv_work1_exist

		, in_fconv_work1_out = gcPowerIO->di1(IN_FCONV_WORK1_OUT)
		, old_in_fconv_work1_out = in_fconv_work1_out

		, in_fconv_exit_exist = gcPowerIO->di1(IN_FCONV_EXIT_EXIST)
		, old_in_fconv_exit_exist = in_fconv_exit_exist

		;

	//조건부로 CPowerSwitchPanel 클래스 일 못하게 하기 -> CPowerSwitchPanel::ReadSwitchPanel 함수 동작 참고.
	bool is_monopolizing_panel_switch;
	if (pThis.get_monopolize_panel_switch() == true)
	{
		gcPowerSwitchPanel->SetStep(SwitchPanelStep::INIT);
		is_monopolizing_panel_switch = true;
	}
	else
	{
		is_monopolizing_panel_switch = false;
	}

	ULONGLONG timeElapsed = 0;
	//돌기
	while (true)
	{
		//break condition
		if (pThis.getRunning() == false)//외부에서 Stop 호출했을때 여기 걸립니다.
		{
			TRACE("[PWR] %s::%s -> STOP!", typeid(Io_state_detector).name(), __func__);
			break;
		}

		if (timeElapsed == 0 || _time_elapsed(timeElapsed) > TIME500MS)
		{
			timeElapsed = _time_get();
		}
		else
		{
			ThreadSleep(TIME10MS);
			continue;
		}

		//스위치 독점 여부 재확인
		if (is_monopolizing_panel_switch != pThis.get_monopolize_panel_switch())
		{
			if (is_monopolizing_panel_switch == true)//독점중이었으면 놓아주고
			{
				gcPowerSwitchPanel->SetStep(SwitchPanelStep::STOP);
			}
			else//독점중이지 않았으면 독점 시작
			{
				gcPowerSwitchPanel->SetStep(SwitchPanelStep::INIT);
			}

			is_monopolizing_panel_switch = !is_monopolizing_panel_switch;//갱신
		}

		//한번 읽기
		stop = gcPowerIO->di1(IN_STOP_PANEL_KEY);
		door = gcPowerIO->di1(IN_FDOOR_LOCK_PANEL_KEY);
		start = gcPowerIO->di1(IN_START_PANEL_KEY);
		reset = gcPowerIO->di1(IN_RESET_PANEL_KEY);

		red = gcPowerIO->Getdo1(OUT_FLAMP_RED);
		green = gcPowerIO->Getdo1(OUT_FLAMP_GRN);
		yellow = gcPowerIO->Getdo1(OUT_FLAMP_YEL);

		in_fconv_entry_ent = gcPowerIO->di1(IN_FCONV_ENTRY_ENT);
		in_fconv_entry_exist = gcPowerIO->di1(IN_FCONV_ENTRY_EXIST);
		in_fconv_work1_low = gcPowerIO->di1(IN_FCONV_WORK1_LOW);
		in_fconv_work1_exist = gcPowerIO->di1(IN_FCONV_WORK1_EXIST);
		in_fconv_work1_out = gcPowerIO->di1(IN_FCONV_WORK1_OUT);
		in_fconv_exit_exist = gcPowerIO->di1(IN_FCONV_EXIT_EXIST);

		//비교해서 다르면(스위치)? #1.원래처럼 스위치 LED켜기, #2.(3 50 2)커맨드날리기.
		if (oldStop != stop)
		{
			TRACE("[PWR] %s::%s -> stop   CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldStop, stop);
			oldStop = stop;
			OutputOne(OUT_STOP_PANEL_LED, (stop == INON) ? (OUTON) : (OUTOFF));
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_STOP_PANEL_KEY, stop, IO_NOUSE);
			SendIOStatus(c);
		}
		if (oldDoor != door)
		{
			TRACE("[PWR] %s::%s -> door   CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldDoor, door);
			oldDoor = door;
			OutputOne(OUT_FDOOR_LOCK_PANEL_LED, (door == INON) ? (OUTON) : (OUTOFF));
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FDOOR_LOCK_PANEL_KEY, door, IO_NOUSE);
			SendIOStatus(c);
		}
		if (oldStart != start)
		{
			TRACE("[PWR] %s::%s -> start  CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldStart, start);
			oldStart = start;
			OutputOne(OUT_START_PANEL_LED, (start == INON) ? (OUTON) : (OUTOFF));
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_START_PANEL_KEY, start, IO_NOUSE);
			SendIOStatus(c);
		}
		if (oldReset != reset)
		{
			TRACE("[PWR] %s::%s -> reset  CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldReset, reset);
			oldReset = reset;
			OutputOne(OUT_RESET_PANEL_LED, (reset == INON) ? (OUTON) : (OUTOFF));
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_RESET_PANEL_KEY, reset, IO_NOUSE);
			SendIOStatus(c);
		}

		//비교해서 다르면(타워램프)? #1.(3 50 2)커맨드 날리기.
		if (oldRed != red)
		{
			TRACE("[PWR] %s::%s -> red    CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldRed, red);
			oldRed = red;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, OUT_FLAMP_RED, red, IO_NOUSE);
			SendIOStatus(c);
		}
		if (oldGreen != green)
		{
			TRACE("[PWR] %s::%s -> green  CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldGreen, green);
			oldGreen = green;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, OUT_FLAMP_GRN, green, IO_NOUSE);
			SendIOStatus(c);
		}
		if (oldYellow != yellow)
		{
			TRACE("[PWR] %s::%s -> yellow CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, oldYellow, yellow);
			oldYellow = yellow;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, OUT_FLAMP_YEL, yellow, IO_NOUSE);
			SendIOStatus(c);
		}

		//비교해서 다르면(컨베이어 센서)? #1. (3 50 2)커맨드 날리기.
		if (old_in_fconv_entry_ent != in_fconv_entry_ent)
		{
			TRACE("[PWR] %s::%s -> in_fconv_entry_ent   CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_entry_ent = in_fconv_entry_ent;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_ENTRY_ENT, in_fconv_entry_ent, IO_NOUSE);
			SendIOStatus(c);
		}
		if (old_in_fconv_entry_exist != in_fconv_entry_exist)
		{
			TRACE("[PWR] %s::%s -> in_fconv_entry_exist CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_entry_exist = in_fconv_entry_exist;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_ENTRY_EXIST, in_fconv_entry_exist, IO_NOUSE);
			SendIOStatus(c);
		}
		if (old_in_fconv_work1_low != in_fconv_work1_low)
		{
			TRACE("[PWR] %s::%s -> in_fconv_work1_low   CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_work1_low = in_fconv_work1_low;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_WORK1_LOW, in_fconv_work1_low, IO_NOUSE);
			SendIOStatus(c);
		}
		if (old_in_fconv_work1_exist != in_fconv_work1_exist)
		{
			TRACE("[PWR] %s::%s -> in_fconv_work1_exist CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_work1_exist = in_fconv_work1_exist;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_WORK1_EXIST, in_fconv_work1_exist, IO_NOUSE);
			SendIOStatus(c);
		}
		if (old_in_fconv_work1_out != in_fconv_work1_out)
		{
			TRACE("[PWR] %s::%s -> in_fconv_work1_out   CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_work1_out = in_fconv_work1_out;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_WORK1_OUT, in_fconv_work1_out, IO_NOUSE);
			SendIOStatus(c);
		}
		if (old_in_fconv_exit_exist != in_fconv_exit_exist)
		{
			TRACE("[PWR] %s::%s -> in_fconv_exit_exist  CHANGED : %d -> %d (0 : ON, 1 : OFF)", typeid(Io_state_detector).name(), __func__, old_in_fconv_entry_ent, in_fconv_entry_ent);
			old_in_fconv_exit_exist = in_fconv_exit_exist;
			CString c;
			c.Format(L"%d,%d,%d,%d", NO_ERR, IN_FCONV_EXIT_EXIST, in_fconv_exit_exist, IO_NOUSE);
			SendIOStatus(c);
		}

		//쉬면서 돌기
		//std::this_thread::sleep_for(std::chrono::microseconds(500));

	}//end of while(true)

	//원래 타워램프로 되돌리기
	TowerLampNormal();

	//START 같은 경우 독점을 하지 않으면 생산창으로 넘어가버리면서 PANEL_LED를 끄기 전에 이 스레드가 끝나버리므로 끝나기전에 무조건 그냥 한번 끄도록 했다.
	OutputOne(OUT_START_PANEL_LED, OUTOFF);

	//다시 CPowerSwitchPanel 클래스가 일 할 수 있게 하기
	gcPowerSwitchPanel->SetStep(SwitchPanelStep::STOP);
	TRACE("[PWR] %s::%s -> Quit!", typeid(Io_state_detector).name(), __func__);

	pThis.setEnd(true);
	//종료
	return 0;
}//end of method

bool Io_state_detector::getRunning()
{
	this->mtx.lock();
	const bool result = this->isRunning;
	this->mtx.unlock();
	return result;
}

void Io_state_detector::setRunning(const bool isRunning)
{
	this->mtx.lock();
	this->isRunning = isRunning;
	this->mtx.unlock();
	return;
}

void Io_state_detector::setEnd(const bool isEnd)
{
	this->mtx.lock();

	if (this->isEnd != isEnd)
	{
		TRACE("[PWR] %s::%s -> setEnd:%d", typeid(Io_state_detector).name(), __func__, isEnd);
	}

	this->isEnd = isEnd;
	this->mtx.unlock();
	return;
}

bool Io_state_detector::getEnd()
{
	this->mtx.lock();
	const bool result = this->isEnd;
	this->mtx.unlock();
	return result;
}

void Io_state_detector::Stop()
{
	if (this->getRunning() == false && this->getEnd() == true)
	{
		TRACE("[PWR] %s::%s -> already stopped", typeid(Io_state_detector).name(), __func__);
	}
	else
	{
		this->setRunning(false);

		ULONGLONG timeElapsed = _time_get();

		while (true)
		{
			if (this->getEnd() == true)
			{
				TRACE("[PWR] %s::%s -> stop complete", typeid(Io_state_detector).name(), __func__);
				break;
			}
			else if (_time_elapsed(timeElapsed) > TIME500MS)
			{
				TRACE("[PWR] %s::%s -> stop time over", typeid(Io_state_detector).name(), __func__);
				break;
			}
		}
	}
}

Io_state_detector& Io_state_detector::Go() const
{
	CWinThread* cWinThreadPointer = NULL;

	cWinThreadPointer = AfxBeginThread(Io_state_detector::Run, (LPVOID)0);

	if (cWinThreadPointer == NULL)
	{
		TRACE("[PWR] %s::%s -> worker thread create failed", typeid(Io_state_detector).name(), __func__);
		return Io_state_detector::GetInstance();
	}

	cWinThreadPointer->m_bAutoDelete = true;

	return Io_state_detector::GetInstance();
}

void Io_state_detector::set_monopolize_panel_switch(const bool is_monopolize_panel_switch)
{
	this->mtx.lock();
	this->is_monopolize_panel_switch = is_monopolize_panel_switch;
	TRACE("[PWR] %s::%s -> %s", typeid(Io_state_detector).name(), __func__, (is_monopolize_panel_switch == true) ? ("true") : ("false"));
	this->mtx.unlock();
	return;
}

bool Io_state_detector::get_monopolize_panel_switch()
{
	this->mtx.lock();
	const bool result = this->is_monopolize_panel_switch;
	this->mtx.unlock();
	return result;
}