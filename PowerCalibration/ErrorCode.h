#pragma once
/*
//	ErrorCode.h
//
//	(C) Power. 2020 ~
*/
///////////////////////////////////////////////////////// Motion
#define NO_ERR							0
#define CONNECT_FAIL_TO_SERVER			1
#define LM_TEMP_HIGH_FX					2
#define LM_TEMP_HIGH_FY1				3
#define LM_TEMP_HIGH_FY2				4
#define LM_TEMP_HIGH_RX					5
#define LM_TEMP_HIGH_RY1				6
#define LM_TEMP_HIGH_RY2				7
#define INVALID_GANTRY					8
#define INVALID_HEADNO					9
#define INVALID_INSERTNO				10
#define UNDEFINED_AXIS_NULL				11
#define UNDEFINED_AXIS_TIMEOUT			12
#define UNDEFINED_MINUS_LIMIT			13
#define UNDEFINED_PLUS_LIMIT			14
#define UNDER_MIN_LINEAR_INTP_AXIS		15
#define NOT_READY_SYNC_Y				16
#define EMPTY_XY						17
#define EMPTY_ALLZ						18
#define EMPTY_ALLR						19
#define UNDEIFNED_IO					20
#define PCB_FIDUCIALMARK_RECOGNITION	21
#define JOBFILE_OPEN_FAIL				22
#define JOBFILE_READSTRING_FAIL			23
#define ISNOT_NORMALMODE				24
#define ISNOT_PRODRUNMODE				25
#define ISNOT_PAUSEMODE					26
#define ALREADY_INITIALIZED_MACHINE		27
#define UNDERZERO_1DCOUNT				28
#define WORK1_STOPPER_UP_TIMEOUT		29
#define WORK1_STOPPER_DN_TIMEOUT		30
#define STOP_NOW						31
#define STOP_STEP						32
#define STOP_BLOCK						33
#define STOP_BOARD						34
#define OPEN_DOOR						35
#define PUSH_EMERGENCY					36
#define OPEN_LOTOKEY					37
#define DISABLE_MOTOR_POWER				38
#define INVALID_MARKNO					39
#define INVALID_PICKUPZSTANDBY			40
#define INVALID_STANDBYZ				41
#define RECOGNITION_MARK_FAIL			42
#define DETECTED_AREA_SENSOR_FRONT		43
#define DETECTED_AREA_SENSOR_REAR		44
#define EIMES_CANNOTDO					45
#define EIMES_TIMEOUT					46
#define HEIGHT_MEASUREMENT_FAIL			47
#define LOW_AIR							48
#define RECOGTABLE_FAIL					49
#define LM_TEMP_LOW_FX					50
#define LM_TEMP_LOW_FY1					51
#define LM_TEMP_LOW_FY2					52
#define LM_TEMP_LOW_RX					53
#define LM_TEMP_LOW_RY1					54
#define LM_TEMP_LOW_RY2					55

#define EMPTY_NOZZLE_FZ1				100
#define EMPTY_NOZZLE_FZ2				101
#define EMPTY_NOZZLE_FZ3				102
#define EMPTY_NOZZLE_FZ4				103
#define EMPTY_NOZZLE_FZ5				104
#define EMPTY_NOZZLE_FZ6				105

#define ANC_HOLD_FZ1					110
#define ANC_HOLD_FZ2					111
#define ANC_HOLD_FZ3					112
#define ANC_HOLD_FZ4					113
#define ANC_HOLD_FZ5					114
#define ANC_HOLD_FZ6					115

#define ANC_RELEASE_FZ1					120
#define ANC_RELEASE_FZ2					121
#define ANC_RELEASE_FZ3					122
#define ANC_RELEASE_FZ4					123
#define ANC_RELEASE_FZ5					124

#define EMPTY_NOZZLE_RZ1				200
#define EMPTY_NOZZLE_RZ2				201
#define EMPTY_NOZZLE_RZ3				202
#define EMPTY_NOZZLE_RZ4				203
#define EMPTY_NOZZLE_RZ5				204
#define EMPTY_NOZZLE_RZ6				205

#define ANC_HOLD_RZ1					210
#define ANC_HOLD_RZ2					211
#define ANC_HOLD_RZ3					212
#define ANC_HOLD_RZ4					213
#define ANC_HOLD_RZ5					214
#define ANC_HOLD_RZ6					215

#define ANC_RELEASE_RZ1					220
#define ANC_RELEASE_RZ2					221
#define ANC_RELEASE_RZ3					222
#define ANC_RELEASE_RZ4					223
#define ANC_RELEASE_RZ5					224
#define ANC_RELEASE_RZ6					225


#define MAX_TORQUE_LIMIT_FZ1			300
#define MAX_TORQUE_LIMIT_FZ2			301
#define MAX_TORQUE_LIMIT_FZ3			302
#define MAX_TORQUE_LIMIT_FZ4			303
#define MAX_TORQUE_LIMIT_FZ5			304
#define MAX_TORQUE_LIMIT_FZ6			305

#define AXIS_NULL(X)					1000 + (X)	// FX 1000 ~ 1019
													// RX 1020 ~ 1039
#define MOTION_TIMEOUT(X)				2000 + (X)	// FX 2000 ~ 2019
													// RX 2020 ~ 2039
#define MINUS_LIMIT(X)					3000 + (X)	// FX 3000 ~ 3019
													// RX 3020 ~ 3039
#define PLUS_LIMIT(X)					4000 + (X)	// FX 4000 ~ 4019
													// RX 4020 ~ 4039
#define SERVO_ON_TIMEOUT(X)				5000 + (X)	// FX 5000 ~ 5019
													// RX 5020 ~ 5039
#define SERVO_OFF_TIMEOUT(X)			6000 + (X)	// FX 6000 ~ 6019
													// RX 6020 ~ 6039
#define HOMING_TIMOUT(X)				7000  + (X) // FX 7000 ~ 7019
													// RX 7020 ~ 7039
#define HOMING_FAIL(X)					7100  + (X) // 

#define INVALID_PICKORDER(x)			(8000 + (x))	// PickOrder
#define INVALID_INSERTORDER(x)			(9000 + (x))	// InsertOrder
#define INVALID_PACKAGENAME(x)			(10000 + (x))	// Feeder No
#define READYTIMEOUTEMPTY(x)			(11000 + (x))	// Feeder No
#define INVALID_TRAYNAME(x)				(12000 + (x))	// Feeder No

// 13000 ~ 13999 TTF1
// 14000 ~ 14999 TTF2 
#define TTF_PALLET_EXIST(x)					(13000 + (x)*1000)
#define TTF_MAGAZINE_EXIST(x)				(13001 + (x)*1000)
#define TTF_MAGAZINE_EMPETY(x)				(13002 + (x)*1000)
#define TTF_FCLAMP_LOCK_TIMEOUT(x)			(13003 + (x)*1000)
#define TTF_FCLAMP_UNLOCK_TIMEOUT(x)		(13004 + (x)*1000)
#define TTF_RCLAMP_LOCK_TIMEOUT(x)			(13005 + (x)*1000)
#define TTF_RCLAMP_UNLOCK_TIMEOUT(x)		(13006 + (x)*1000)
#define TTF_Z_SAFETY_ON(x)					(13007 + (x)*1000)
#define TTF_PALLET_LOCK_TIMEOUT(x)			(13008 + (x)*1000)
#define TTF_PALLET_UNLOCK_TIMEOUT(x)		(13009 + (x)*1000)
#define TTF_PALLET_NUMBER_ERROR(x)			(13010 + (x)*1000)
#define TTF_SUPPLY_TIMEOUT(x)				(13011 + (x)*1000)
#define TTF_DOOR_LOCK_TIMEOUT(x)			(13012 + (x)*1000)
#define TTF_DOOR_UNLOCK_TIMEOUT(x)			(13013 + (x)*1000)

// 15000~150200 
#define EMPTY_STOP(x)						(15000 + (x))	// Feeder No													
///////////////////////////////////////////////////////// Ethercat



///////////////////////////////////////////////////////// Socket


///////////////////////////////////////////////////////// Message Queue


///////////////////////////////////////////////////////// IO


///////////////////////////////////////////////////////// Thread
#define NOT_FOUND_THREAD_ID			-1000


