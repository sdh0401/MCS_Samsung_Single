#pragma once
/*
//	NetworkDefine.h
//
//	(C) Power. 2020 ~
*/

enum class WMX3StateMachine {
	None = 0x00,
	Init = 0x01,
	Preop = 0x02,
	Boot = 0x03,
	Safeop = 0x04,
	Op = 0x08
};
