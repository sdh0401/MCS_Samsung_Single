#pragma once
#include <mutex>

class Io_state_detector
{

public:
	static Io_state_detector& GetInstance();

private:
	Io_state_detector();
	~Io_state_detector();
	Io_state_detector& operator=(const Io_state_detector& ref);

public:
	Io_state_detector& Go() const;
	void Stop();
	void set_monopolize_panel_switch(const bool is_monopolize);

private:
	std::mutex mtx;
	bool isRunning;
	bool is_monopolize_panel_switch;
	bool isEnd;
private:
	static UINT Run(LPVOID parameter);
	void setRunning(const bool isRunning);
	bool getRunning();
	void setEnd(const bool isEnd);
	bool getEnd();
	bool get_monopolize_panel_switch();

};
