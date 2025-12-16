/*
//	Vision.h
*/

#ifndef VISION_H
#define VISION_H

/* crc polynomial & precalculated crc table. */
#define CRC_POLYNOMIAL				0x04C11DB7L    			/* 0xEDB88320L */

#define	VIS_BASE_ORG			0x80000000

#define VIS_TAKE				0
#define VIS_GIVE				1

#define VIS_DI					0			/* Diagnostic Command */
#define VIS_CA					1			/* Auto Camera Calibration Command */
//#define VIS_MCA					2
#define VIS_FT					3			/* Fiducial Mark Training Command */
#define VIS_FI					4			/* Fiducial Mark Inspection Command */
//#define VIS_VA					5			/* Visual Alignment (Gerenal Header) Command */
//#define VIS_VA4					6			/* Visual Alignment (Precision Header) Command */
#define VIS_DB					7			/* Data Download Command */
#define VIS_DT					8			/* Digtizer Control Command */
#define VIS_WC					9			/* Window Control Command */
#define VIS_ESC					10			/* Escape Command */
#define VIS_RA					11			/* Rotate Angle Calibration Command */
#define VIS_CC					12			/* Camera Configuration Command */
//#define	VIS_PD					13			/* Program Download */
#define	VIS_IMG					14			/* Image Data Upload */
#define	VIS_SV					15			/* Vision software version */
#define VIS_IT					16			/* Vision Image Test */
#define	VIS_DW					17			/* Component DB drawing */
#define	VIS_AT					18			/* Part Auto teaching */
#define VIS_MODX				19			/* Mode Parameter Change */
#define VIS_TOLE				19			/* Tolerance Change */
#define	VIS_FAT					21			/* Feeder Position Auto Teaching Command */
#define VIS_NZL					22			/* Nozzle Dust Checking */
#define	VIS_ANC					23			/* ANC Nozzle Rotation */
#define	VIS_VA8					24			/* Component Inspection(New) Command */
#define	VIS_QPVA				VIS_VA8		/* QP Machine Vision Alignment */
#define	VIS_LN					25			/* Line Camera */
#define	VIS_CL					26			/* Calibration Data Write */
#define VIS_3DCA				28			/* 3D Camera Calibration Command*/
#define VIS_MMARK				29			/* Multi Mark Inspection */
#define VIS_OCR_LIB				30			/* Use Letters inspection for Open eVision */
#define VIS_OCR					31			/* Letters inspection */
#define VIS_BARCODE				32			/* Barcode inspection */
#define VIS_NZLCHK				33			/* Nozzle State Check */
#define VIS_DROPCHECK			40			
#define VIS_REQDB				50			/* Component DB MCS <---> VS Communication */
#define VIS_SURFACE				52			/* Camera Surface Inspection */
#define VIS_RGB					53			/* Color Camera Command */
#define VIS_LASER				54			/* Laser Vision */
#define VIS_COLOR				55			/* Camera Color Mode */
#define VIS_CAMOFFSET			56			/* Camera Recognition Offset */
#define VIS_EOP					-1

#define VIS_SUB_CMD0			0
#define VIS_SUB_CMD1			1
#define VIS_SUB_CMD2			2
#define VIS_SUB_CMD3			3


/*** Some Other Cmd(10xx) ***/
#define MAXVAHEAD 				8
#define MAXFRAMENO				9
#define VIS_BUSY				1

/*** Mark Color Define */
#define MARK_WHITE				0
#define MARK_BLACK				1

/*** Threshold Method Define */
#define THRES_AUTO				0
#define THRES_MANUAL			1

#define MAXWINDOWBOX			2

/*** window select define */
#define WINALL					0
#define WIN1ST					1
#define WIN2ND					2

/*** window edge select define */
#define WIN_LT					0
#define WIN_RD					1
#define WIN_CENT				2

/*** Part Checking Location Define ***/
#define	CHK_SIDE				0
#define	CHK_SIDE1				0
#define CHK_CENT				2
#define	CHK_CENT2				3
#define CHK_CENT3				4
#define CHK_CENT4				5
#define CHK_CENT5				6
#define CHK_CENT6				7

#define WIN_LEFT				0
#define WIN_RIGHT				1
#define WIN_UP					2
#define WIN_DOWN				3

#define FIDMARK0				0
#define FIDMARK1				1
#define FIDMARK2				2
#define FIDMARK3				3
#define FIDMARK4				4
#define FIDMARK5				5
#define FIDMARK6				6
#define FIDMARK7				7
#define FIDMARK8				8
#define FIDMARK9				9
#define FIDMARK10				10
#define FIDMARK11				11
#define FIDMARK12				12
#define FIDMARK13				13
#define FIDMARK14				14
#define FIDMARK15				15
#define FIDMARK16				16
#define FIDMARK17				17
#define FIDMARK18				18
#define FIDMARK19				19


/* 특수한 Mark의 DB File Number 지정 */
#define TMPMARK					(USEDMARKDBNOBYJOB)			/* 20 */
#define TESTMARK(x)				(USEDMARKDBNOBYJOB+(x))		/* 20~ */
#define FDREFMARK				(USEDMARKDBNOBYJOB+1)		/* 21 */
#define ANCREFMARK				(USEDMARKDBNOBYJOB+2)		/* 22 */
#define MCALMARK				(USEDMARKDBNOBYJOB+3)		/* 23 */
#define MACHREFMARK 			(USEDMARKDBNOBYJOB+4)		/* 24 */
#define NZLCNTMARK_FCSP			(USEDMARKDBNOBYJOB+5)		/* 25 */
#define NZLCENTERMARK			(USEDMARKDBNOBYJOB+6)		/* 26 */
#define NZLCNTMARK_RCSP			(USEDMARKDBNOBYJOB+6)		/* 26 */
#define CONVPWBMARK				(USEDMARKDBNOBYJOB+7)		/* 27 */
#define CONVPOSMARK				(USEDMARKDBNOBYJOB+8)		/* 28 */

#define	MACHREFMARKSIDE(x)		(USEDMARKDBNOBYJOB-(x)-1)		// 19 ~ 11. (machine reference mark)

#define CHIP4532				(USEDMARKDBNOBYJOB+9)
#define NZLCENTERMARK_RP		(USEDMARKDBNOBYJOB+10)
#define NZLCENTERMARK_CSP		(USEDMARKDBNOBYJOB+11)
#define ALIGNMARKBLK			(USEDMARKDBNOBYJOB+12)
#define ALIGNMARKWHT			(USEDMARKDBNOBYJOB+13)
#define SMALLMCALMARK			(USEDMARKDBNOBYJOB+14)
#define ALIGNMARKWHTCAM2		(USEDMARKDBNOBYJOB+15)
#define TESTMARK1				(USEDMARKDBNOBYJOB+16)
#define TESTMARK2				(USEDMARKDBNOBYJOB+17)
#define CHIP1005				(USEDMARKDBNOBYJOB+18)
#define CHIP1608				(USEDMARKDBNOBYJOB+18)
#define GLASSQFP				(USEDMARKDBNOBYJOB+19)	// 이것을 추가하면 Vision쪽에 Mark BD 갯수를 늘려 줘야 합니다.

/* 특수한 부품의 DB File Number 지정 */
#define TMPPART					USEDVISIONFILENO
#define TESTPART(x)				(TMPPART+(x)+1)	/* x=0~10 */
#define TMPODDPART				(MAXODDVADBNO-1)
#define TMPINSERTPART			(MAXINSERTPARTVADBNO-1)
#define NOZZLETESTDB(x)			(97+(x))		/* x=0~9 */

#define P_CHIP					1
#define P_SEMI					2
#define P_BGA					3
#define P_CONN					4
#define P_ODD					5

#define COMP_RELAY				117
#define COMP_120				120
#define COMP_121				121
#define COMP_122				122
#define COMP_123				123
#define COMP_124				124
#define COMP_125				125
#define COMP_126				126
#define COMP_127				127
#define COMP_128				128
#define COMP_160                160
#define COMP_161                161
#define COMP_162				162

#define COMP_177				177		// Multi Pin Component DB Possible User VA Offset Function

#define COMP_LASERPIN			196
#define COMP_INSERTPIN			197
#define COMP_HEADERPIN			199

#define SEMI_QFP				201
#define SEMI_QFP_BUMPER			202
#define SEMI_SOP				203
#define SEMI_SOJ				204
#define SEMI_PLCC				205

#define ODD_QFP					501
#define ODD_SOP					502
#define ODD_SOJ					503
#define ODD_PLCC				504
#define ODD_CONN				505
#define ODD_TR					506
#define ODD_USER				509
#define ODD_INSERT				511

/* Define index for Temporary(Only) Offset Mark Compensation Value. Temporary !!! */
#define MK_PWB					MAXBDFIDNO
#define MK_0					(MAXBDFIDNO+1)
#define MK_1					(MAXBDFIDNO+2)
#define MK_2					(MAXBDFIDNO+3)
#define MK_3					(MAXBDFIDNO+4)
#define MK_4					(MAXBDFIDNO+5)
#define MK_5					(MAXBDFIDNO+6)
#define MK_6					(MAXBDFIDNO+7)
#define MK_7					(MAXBDFIDNO+8)
#define	MK_ANC					(MAXBDFIDNO+9)
#define MK_S(sec,num)			(MAXBDFIDNO+((sec)x2)+(num)+10)		// maximum 20
#define MK(head)				(MAXBDFIDNO+(head)+30)	// for local part mark.

#define LV_DEFAULT_FRAMENO		20

#define MAX_VIS_VALID_RANGE		9999.999

#endif

/* End of file */
