#pragma once
/*
//	DefineThreadLoopTime.h
//
//	(C) Power. 2020 ~
*/
///////////////////////////////////////////////////////// Ethercat
#define THREAD_WMX3_MASTER_READTIME					TIME10MS
#define THREAD_WMX3_MOTOR_READTIME					TIME1MS
#define THREAD_SLAVE_MOTOR_READTIME					TIME3MS
///////////////////////////////////////////////////////// Socket
#define THREAD_CLIENT_SOCKET_RECV					TIME1MS
#define THREAD_CLIENT_SOCKET_SEND					TIME1MS

///////////////////////////////////////////////////////// Message Queue
#define THREAD_MSGQ_SOCKET_READ						TIME1MS
#define THREAD_MSGQ_HOST_READ						TIME1MS
///////////////////////////////////////////////////////// IO
#define THREAD_ANALOG_READTIME						TIME10MS
#define THREAD_WMX3_IO_READTIME						TIME5MS
#define THREAD_IO_READTIME							TIME5MS

/////////////////////////////////////////////////////////
#define THREAD_MOTOR_READTIME						TIME10MS
#define THREAD_MACHINE_MONITORINGTIME				TIME20MS
#define THREAD_BUZZER_READTIME						TIME10MS

///////////////////////////////////////////////////////// Vision
#define THREAD_VISION_CHECK_CONNECT					TIME100MS
#define THREAD_VISION_RESULT_READTIME				TIME1MS
#define THREAD_VISION_DOWNLOAD_TIME					TIME100MS
#define THREAD_VISION_UPDATE_DATE_TIME				TIME100MS
#define THREAD_VISION_CALIBRATION_WAITTIME			TIME1000MS

///////////////////////////////////////////////////////// Thread
#define THREAD_RUN_WAITTIME							TIME100MS
#define THREAD_SUSPEND_READTIME						TIME500MS
#define THREAD_CHILDTHREAD_DO_WAIT					TIME100MS
#define THREAD_SOCKET_START_WAIT					TIME1MS
#define THREAD_ORIGIN_SEARCH_TIME					TIME10MS
#define THREAD_HOST_MSGQ_READTIME					TIME50MS
#define THREAD_ALIVE_CHECKTIME						TIME10MS
#define THREAD_EXIT_SAFETIME						TIME30MS
#define THREAD_EXIT_TIMEOUT							TIME500MS
#define THREAD_CAL_FUNC_READTIME					TIME100MS
#define THREAD_ERROR_READTIME						TIME100MS
#define THREAD_GLOBAL_ERROR_READTIME				TIME100MS
#define THREAD_CLEANER_READTIME						TIME1MS

///////////////////////////////////////////////////////// Homing
#define THREAD_HOMING_DONE_WAITTIME					TIME1000MS

///////////////////////////////////////////////////////// Calibration
#define THREAD_Z_CALIBRATION_READTIME				TIME10MS
#define THREAD_1D_CALIBRATION_READTIME				TIME10MS
#define THREAD_1D_CATCH_DELAY						TIME500MS
#define THREAD_2D_CALIBRATION_READTIME				TIME100MS
#define THREAD_ALIGNOFFSET_CALIBRATION_READTIME		TIME10MS
#define THREAD_HEADOFFSET_CALIBRATION_READTIME		TIME100MS
#define THREAD_OFFSETCAMERA_CALIBRATION_READTIME	TIME100MS
#define THREAD_MODULECAMERA_CALIBRATION_READTIME	TIME100MS

///////////////////////////////////////////////////////// Conveyor
#define THREAD_ENTRYCONV_READTIME					TIME20MS
#define THREAD_WORKCONV_READTIME					TIME20MS
#define THREAD_EXITCONV_READTIME					TIME20MS
#define THREAD_PCBEXIST_READTIME					TIME5MS
#define THREAD_SMEMA_READTIME						TIME3MS
#define THREAD_RETURN_TO_ENTRANCE_READTIME			TIME5MS
#define THREAD_LOADINGTOWORK_READTIME				TIME5MS
#define THREAD_OUTTOEXIT_READTIME					TIME5MS
///////////////////////////////////////////////////////// Filtering
#define PCBEXIST_SENSOR_FILTERTIME					TIME100MS

///////////////////////////////////////////////////////// Sync Gantry & Conveyor
#define THREAD_SYNC_READTIME						TIME10MS

///////////////////////////////////////////////////////// Main
#define THREAD_MAIN_READTIME						TIME1MS

///////////////////////////////////////////////////////// Main
#define THREAD_FEEDER_READTIME						TIME5MS

///////////////////////////////////////////////////////// Button
#define THREAD_SWITCH_PANEL_READTIME				TIME5MS

///////////////////////////////////////////////////////// Torque Monitor
#define THREAD_TORQUE_MONITOR_READTIME				TIME1MS