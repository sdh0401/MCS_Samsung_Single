#pragma once
/*
//	Wmx3MotorDefine.h
//
//	(C) Power. 2020 ~
*/

enum class WMX3AxisDir {
	wmx3PLUS_LIM,
	wmx3MINUS_LIM,
	wmx3HOME_SENSOR
};

enum class WMX3AxisHomeDir {
	Positive,
	Negative
};

enum class Wmx3AxisMoveDir {
	Positive = 1,
	Negative = -1
};

enum class WMX3AxisHomeType {
	CurrentPos,
	ZPulse,
	HS,
	HSHS,
	HSZPulse,
	HSReverseZPulse,
	LSReverseZPulse,
	NearLSReverseZPulse,
	ExternalLSReverseZPulse,
	TrqLimit,
	TouchProbe,
	HSTouchProbe,
	LS,
	NearLS,
	ExternalLS,
	MechanicalEndDetection,
	MechanicalEndDetectionHS,
	MechanicalEndDetectionLS,
	MechanicalEndDetectionReverseZPulse
};

enum class WMX3AxisLimitSwitchType {
	None,
	ServoOff,
	DecServoOff,
	Dec,
	SlowDecServoOff,
	SlowDec,
	SeparatePositiveLSNegativeLS
};

enum class WMX3HomeState {
	Idle,
	ZPulseSearch,
	ZPulseSearchReverse,
	ZPulseSearchPauseReverse,
	HSSearch,
	HSSearchPause,
	HSAndZPulseSearch,
	HSAndZPulseSearchPause,
	LSSearch,
	LSSearchPause,
	HSClearReverse,
	HSClearReversePause,
	HSFallingEdgeSearchReverse,
	HSFallingEdgeSearchReversePause,
	LSFallingEdgeSearchReverse,
	LSFallingEdgeSearchReversePause,
	TouchProbeSearch,
	TouchProbeSearchPause,
	SecondHSSearch,
	SecondHSSearchPause,
	SecondTouchProbeSearch,
	SecondTouchProbeSearchPause,
	MechanicalEndDetection,
	HomeShift,
	HomeShiftPause,
	Cancel,
	Other
};

enum class WMX3HomeError {
	NoError,
	LSTriggered,
	MaxLSRevDistanceTraveled,
	MaxHSOnAtStartRevDistanceTraveled,
	ZPulseDistanceCheckError
};

enum class WMX3OperationState {
	Idle,
	Pos,
	Jog,
	Home,
	Sync,
	GantryHome,
	Stop,
	Intpl,
	Velocity,
	ConstLinearVelocity,
	Trq,
	DirectControl,
	PVT,
	ECAM,
	SyncCatchUp,
	DancerControl
};

enum class WMX3DetailOperationState {
	Idle = 0,
	Pos = 200,
	Pos_OverrideSetup,
	Pos_WaitingForTrigger,
	Jog = 300,
	Jog_OverrideSetup,
	Home = 400,
	Sync = 500,
	Sync_PhaseShift,
	Sync_GearShift,
	GantryHome = 600,
	Stop = 700,
	Stop_QStop,
	Stop_EStop,
	Intpl = 800,
	Intpl_Linear,
	Intpl_Circular,
	Intpl_Helical,
	Intpl_Spline,
	Intpl_Path,
	Intpl_PathWithRotation,
	Intpl_OverrideSetup,
	Intpl_OverrideSmoothing,
	Velocity = 900,
	Velocity_OverrideSetup,
	ConstLinearVelocity = 1100,
	Trq = 1200,
	DirectControl = 1300,
	PVT = 1400,
	ECAM = 1500,
	SyncCatchUp = 1600,
	DancerControl = 1700
};

enum class WMX3AxisCommandMode {
	Position,
	Velocity,
	Torque
};

enum class WMX3ProfileType {
	Trapezoidal,
	SCurve,
	JerkRatio,
	Parabolic,
	Sin,
	AdvancedS,
	TrapezoidalMAT,
	JerkLimited,
	JerkLimitedSCurve,
	JerkLimitedAdvancedS,
	TwoVelocityTrapezoidal,
	TwoVelocitySCurve,
	TwoVelocityJerkRatio,
	TimeAccTrapezoidal,
	TimeAccSCurve,
	TimeAccJerkRatio,
	TimeAccParabolic,
	TimeAccSin,
	TimeAccAdvancedS,
	ConstantDec,
	JerkRatioFixedVelocityT,
	JerkRatioFixedVelocityS,
	JerkLimitedFixedVelocityT,
	JerkLimitedFixedVelocityS
};

enum class WMX3AxisSyncMode {
	NoSync,
	NoOffset,
	VelocityOffset,
	SymmetricVelocityOffset
};

enum class WMX3EngineState {
	Idle,
	Running,
	Communicating,
	Shutdown,

	Unknown
};

enum class WMX3EmgStopLevel {
	Final,
	Level1
};

enum class WMX3LinearIntplProfileCalcMode {
	AxisLimit,
	MatchSlowestAxis,
	MatchFarthestAxis
};

//////////////////////////////////////////////////////////////////////////////////////////////////////

enum class CMotionControlType {
	SimulMotion,
	Wmx3Motion,
	MaxMotion
};

enum class CMotorType {
	SimulMotor,
	ServoMotor,
	StepMotor,
	StepServoMotor,
};

//////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct _WMX3MASTER_INFO {
	bool invalidLicenseError;
	WMX3EngineState engineState;
	double cycleTimeMilliseconds[2];
	long long cycleCounter[2];
	bool emergencyStop;
	WMX3EmgStopLevel emergencyStopLevel;
} WMX3MASTER_INFO;

typedef struct _WMX3AXISCOMPENSATION {
	double pitchErrorCompensation;
	double pitchErrorCompensation2D;
	double backlashCompensation;
	double totalPosCompensation;
} WMX3AXISCOMPENSATION;

typedef struct _WMX3PROFILE {
	double velocity;
	double acc;
	double dec;
	double jerkAcc;
	double jerkDec;
	double jerkAccRatio;
	double jerkDecRatio;
	double accTimeMilliseconds;
	double decTimeMilliseconds;
	double startingVelocity;
	double endVelocity;
	double secondVelocity;
	double movingAverageTimeMilliseconds;
} WMX3PROFILE;

typedef struct _WMX3_MOTOR_HOME_PARAM {
	WMX3AxisHomeType homeType;
	WMX3AxisHomeDir homeDirection;
	double homingVelocitySlow;
	double homingVelocitySlowAcc;
	double homingVelocitySlowDec;
	double homingVelocityFast;
	double homingVelocityFastAcc;
	double homingVelocityFastDec;
	double homingReverseDistance;
	double homeShiftVelocity;
	double homeShiftAcc;
	double homeShiftDec;
	double homeShiftDistance;
	bool invertHSPolarity;
	unsigned int multipleZPulse;
	bool roundPosCmdAfterHoming;
	bool pauseMode;
	double maxHSOnAtStartReverseDistance;
	double maxLSReverseDistance;
	unsigned int zPulseDistanceCheck;
	double homePosition;
	bool gantryHomingUseSlaveHS;
	bool gantryHomingUseSlaveLS;
	bool gantryHomingUseSlaveZPulse;
	bool gantryHomingUseSlaveTouchProbe;
	bool gantryHomingUseSlaveMechanicalEnd;
	bool gantryHomingRetainSyncOffset;
	bool immediateStopAtLS;
	double mechanicalEndDetectionPosDiff;
	double mechanicalEndDetectionTimeMilliseconds;
	bool mechanicalEndDetectionIgnoreLS;
	double mechanicalEndDetectionFirstTorqueLimit;
	double mechanicalEndDetectionSecondTorqueLimit;
	bool openLoopHoming;
	bool clearHomeDoneOnServoOff;
	bool clearHomeDoneOnCommStop;
} WMX3_MOTOR_HOME_PARAM;

typedef struct _WMX3_MOTOR_MOVE_PARAM {
	int SpdMode;
	int HomeMode;
	int HomeDir;
	int TouchPF;
	double WorkSpd;
	double Acc;
	double Dec;
	double HomeSpec;
	double HomeOffset;
	double EscDist;
} WMX3_MOTOR_MOVE_PARAM;

typedef struct _WMX3_AXISCOMPENSATION {
	double pitchErrorCompensation;
	double pitchErrorCompensation2D;
	double backlashCompensation;
	double totalPosCompensation;
} WMX3_AXISCOMPENSATION;

typedef struct _WMX3_AXISSUPPORTEDFUNCTION {
	bool posFeedbackSupport;
	bool posCommandSupport;
	bool velocityFeedbackSupport;
	bool velocityCommandSupport;
	bool velocityOffsetSupport;
	bool trqFeedbackSupport;
	bool trqCommandSupport;
	bool maxTrqLimitSupport;
	bool positiveTrqLimitSupport;
	bool negativeTrqLimitSupport;
	bool maxMotorSpeedSupport;
} WMX3_AXISSUPPORTEDFUNCTION;

typedef struct _WMX3_AXIS_STATUS {
	bool servoOn;
	bool servoOffline;
	bool ampAlarm;
	int ampAlarmCode;
	int masterAxis;
	int secondMasterAxis;
	double posCmd;
	double actualPos;
	double compPosCmd;
	double compActualPos;
	double syncPosCmd;
	double syncActualPos;
	int encoderCommand;
	int encoderFeedback;
	double velocityCmd;
	double actualVelocity;
	double velocityLag;
	double torqueCmd;
	double actualTorque;
	double actualFollowingError;
	WMX3_AXISCOMPENSATION compensation;
	WMX3_AXISSUPPORTEDFUNCTION axisSupportedFunction;
	WMX3OperationState opState;
	WMX3DetailOperationState detailOpState;
	WMX3AxisCommandMode axisCommandMode;
	WMX3AxisSyncMode axisSyncMode;
	double syncOffset;
	double syncPhaseOffset;
	double syncGearRatio;
	bool followingErrorAlarm;
	bool commandReady;
	bool motionPaused;
	double profileTotalMilliseconds;
	double profileAccMilliseconds;
	double profileCruiseMilliseconds;
	double profileDecMilliseconds;
	double profileRemainingMilliseconds;
	double profileTargetPos;
	double profileTotalDistance;
	double profileRemainingDistance;
	double intplVelocity;
	int intplSegment;
	double cmdAcc;
	bool accFlag;
	bool decFlag;
	bool inPos;
	bool inPos2;
	bool inPos3;
	bool inPos4;
	bool inPos5;
	bool cmdDistributionEnd;
	bool posSet;
	bool delayedPosSet;
	unsigned int cmdDistributionEndDelayedPosSetDiff;
	bool positiveLS;
	bool negativeLS;
	bool nearPositiveLS;
	bool nearNegativeLS;
	bool externalPositiveLS;
	bool externalNegativeLS;
	bool positiveSoftLimit;
	bool negativeSoftLimit;
	WMX3HomeState homeState;
	WMX3HomeError homeError;
	bool homeSwitch;
	bool homeDone;
	bool homePaused;
	double homeOffset;
	bool cmdPosToFbPosFlag;
	bool execSuperimposedMotion;
	unsigned int singleTurnCounter;
	bool motionComplete;
} WMX3_AXIS_STATUS;

typedef struct _WMX3_AXIS_MOTOR_STATUS {
	bool servoOn;
	bool servoOffline;
	bool ampAlarm;
	int ampAlarmCode;
	int masterAxis;
	int secondMasterAxis;
	WMX3OperationState opState;
	WMX3DetailOperationState detailOpState;
	bool followingErrorAlarm;
	bool positiveLS;
	bool negativeLS;
	bool nearPositiveLS;
	bool nearNegativeLS;
	bool externalPositiveLS;
	bool externalNegativeLS;
	bool positiveSoftLimit;
	bool negativeSoftLimit;
	unsigned int singleTurnCounter;
} WMX3_AXIS_MOTOR_STATUS;

typedef struct _WMX3_AXIS_MOTION_STATUS {
	double posCmd;
	double actualPos;
	double compPosCmd;
	double compActualPos;
	double syncPosCmd;
	double syncActualPos;
	int encoderCommand;
	int encoderFeedback;
	double velocityCmd;
	double actualVelocity;
	double velocityLag;
	double torqueCmd;
	double actualTorque;
	double actualFollowingError;
	WMX3OperationState opState;
	WMX3DetailOperationState detailOpState;
	WMX3AxisCommandMode axisCommandMode;
	WMX3AxisSyncMode axisSyncMode;
	double syncOffset;
	double syncPhaseOffset;
	double syncGearRatio;
	bool commandReady;
	bool motionPaused;
	double profileTotalMilliseconds;
	double profileAccMilliseconds;
	double profileCruiseMilliseconds;
	double profileDecMilliseconds;
	double profileRemainingMilliseconds;
	double profileTargetPos;
	double profileMoveDistance;
	double profileRemainingDistance;
	double intplVelocity;
	int intplSegment;
	double cmdAcc;
	bool accFlag;
	bool decFlag;
	bool inPos;
	bool inPos2;
	bool inPos3;
	bool inPos4;
	bool inPos5;
	bool cmdDistributionEnd;
	bool posSet;
	bool delayedPosSet;
	unsigned int cmdDistributionEndDelayedPosSetDiff;
	bool cmdPosToFbPosFlag;
	bool execSuperimposedMotion;
	bool motionComplete;
} WMX3_AXIS_MOTION_STATUS;

typedef struct _WMX3_AXIS_HOME_STATUS {
	WMX3HomeState homeState;
	WMX3HomeError homeError;
	bool homeSwitch;
	bool homeDone;
	bool homePaused;
	double homeOffset;
} WMX3_AXIS_HOME_STATUS;

typedef struct _WMX3_AXIS_POSCOMMANDPROFILE
{
	WMX3ProfileType type;
	double velocity;
	double acc;
	double dec;
	double jerkAcc;
	double jerkDec;
	double jerkAccRatio;
	double jerkDecRatio;
	double accTimeMilliseconds;
	double decTimeMilliseconds;
	double startingVelocity;
	double endVelocity;
	double secondVelocity;
	double movingAverageTimeMilliseconds;
} WMX3_AXIS_POSCOMMANDPROFILE;

typedef struct _WMX3_POSCOMMAND {
	int axis;
	double target;
	WMX3_AXIS_POSCOMMANDPROFILE profile;
} WMX3_POSCOMMAND;

