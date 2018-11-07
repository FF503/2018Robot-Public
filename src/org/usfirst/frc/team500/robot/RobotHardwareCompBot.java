package org.usfirst.frc.team500.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class RobotHardwareCompBot {
	public static final int leftMasterID = 3; // front left == 1
	public static final int leftSlaveID = 1; // back left == 2
	public static final int rightMasterID = 4; // front right == 3
	public static final int rightSlaveID = 2; // back right == 4

	public static final int leftIntakeID = 4;
	public static final int rightIntakeID = 5;
	public static final int elevatorID = 5;

	public static final int intakeLiftForward = 4;
	public static final int intakeLiftReverse = 5;
	public static final int INTAKE_STALL_THRESHOLD = 10;

	public static final int carriageTopID = 1;
	public static final int carriageBottomID = 2;
	public static final int carriageIR_ID = 2;
	public static final int carriageReversed = -1;

	@Deprecated
	public final double WHEEL_DIAMETER = 8.0;
	public static final double WHEEL_BASE_INCHES = 25.375;
	public static final double WHEEL_BASE_FEET = WHEEL_BASE_INCHES / 12;
	public static final double WHEEL_BASE_METERS = .6858;

	public final SerialPort.Port laserPort = SerialPort.Port.kUSB2;

	public final double kMaxElevatorVelocityUp = 15;
	public final double kMaxElevatorAcceleration = 100;
	public final double kMaxElevatorJerk = 1000;

	public final double kMaxElevatorVelocityDown = 25;

	public final double CYCLE_TIME = 0.05;

	public final int brakeID = 6;
	public static final int liftID = 7;

	// Wheels
	public static final double kDriveWheelDiameterInches = 6.125;
	public static final double kDriveWheelDiameterFeet = kDriveWheelDiameterInches / 12;
	public static final double kDriveWheelDiameterMeters = 0.18542;
	@Deprecated
	public static final int kEncoderTicksperRev = 360; // was 360 changed to 256 maybe should be 256 ??
	@Deprecated
	public static final int kEncoderReadsperRev = 4;
	@Deprecated
	public static final int kEncoderCountsperRev = kEncoderTicksperRev * kEncoderReadsperRev;

	public static final int kEncoderUnitsPerRev = 4096;

	public final double elevatorSpoolDiameter = 1.5;

	// Low gear values
	public final double kMaxVelocityInchesPerSec = 120;
	public final double kMaxAccelerationInchesPerSec = 125;
	public final double kMaxJerkInchesPerSec = 5000;

	public static final double kMaxVelocityInchesPerSecHigh = 225;

	public static final boolean leftMasterReverseOutput = true;
	public static final boolean leftSlaveReverseOutput = true;
	public static final boolean rightMasterReverseOutput = true;
	public static final boolean rightSlaveReverseOutput = true;
	public static final boolean leftSensorPhase = false;
	public static final boolean rightSensorPhase = true;

	public final double lowGearing = 90.0 / 7.0;
	public final double highGearing = 6.25;
	public final double upShiftRPM = 3811.28972;
	public final double downShiftRPM = 1852.71028;
	public final double shiftToleranceRPM = 100.0;
	public final double motorShiftRPM = 296.4336449;

	// Drive Position PID Coefficients

	public int kDrivePositionIZone = 200;
	public double kDrivePositionRampRate = 256.0;
	public int kDrivePositionAllowableError = 2;

	public static final boolean REVERSE_LEFT_SENSOR = false;
	public static final boolean REVERSE_RIGHT_SENSOR = true;
	public static final boolean REVERSE_LEFT_OUTPUT = false;
	public static final boolean REVERSE_RIGHT_OUTPUT = true;

	public static final int driveSolenoidID1 = 0;
	public static final int driveSolenoidID2 = 1;
	public static final int ptoID1 = 2;
	public static final int ptoID2 = 3;
	public static final double driverJoystickTolerance = 0.1;

	public static final double elevatorSlowDownUp = 1.0;
	public static final double elevatorSlowDownDown = 1.0;
	public static final double elevatorJoystickThreshold = 0.15;
	public static final boolean elevatorMotorInverted = true;
	public static final boolean elevatorSensorPhase = true;
	public static final double elevatorEncoderOffset = 7.5;
	public static final double carriageLaserOffset = -12.5;
	public static final double ElevatorResetHeight = 8.0;
	public static final double ElevatorResetTolerance = 0.3;
	public static final double ElevatorResetMotorPower = 0.8; // positive constant means it's going down
	public static final double ElevatorJammedVelocityTolerance = 0.1;
	public static final double elevatorSafetyShiftHeight = 30;

	public final double sensorMarginThreshold = 20.0;
	public static final double laserLowThreshold = 7.5;
	public static final double elevatorZeroVelocityThreshold = 0.5;

	public static final double intakeFlipped = 1.0;
	public static final double leftIntakeFlipped = 1.0;
	public static final double rightIntakeFlipped = 1.0;
	public static final double intakePower = 0.6;

	public static final double carriageIRThreshold = 2.2;
	public static final double carriageShootTime = 1.0;
	public static final double intakeEjectTime = 2.0;
	public static final double intakeRaiseTime = 1.5;

	public static final int servoPort1 = 0;
	public static final int servoPort2 = 1;
	public static final double blockEngagePos = 0.10; // 0.16
	public static final double blockDisengagePos = 0.90; // 0.90
	public static final double PIVOT_P = 0.1;
	public static final double PIVOT_I = 0;
	public static final double PIVOT_D = 0;
	public static final double PIVOT_TOLERANCE = 5.0;

	public void initialize() {
	}

	public boolean hasSecondIntake() {
		return false;
	}

	public boolean hasTurret() {
		return false;
	}

	public boolean hasIndexer() {
		return false;
	}

	public boolean hasDriveCamera() {
		return false;
	}

	public static String getName() {
		return "CompBot";
	}
}
