package org.usfirst.frc.team500.robot;

public class RobotHardwareProgrammingBot extends RobotHardware {
	public static final double SHOOT_TOLERANCE = 0;
	public static final double DEFLECTOR_P = 0;
	public static final double DEFLECTOR_I = 0;
	public static final double DEFLECTOR_D = 0;
	public static final boolean DEFLECTOR_REVERSE_SENSOR = false;
	public static final double DEFLECTOR_TOLERANCE = 0;
	public static final double DEFLECTOR_MAX = 0;
	public static final double TURRET_CYCLE_TIME = 0;
	public static final double TURRET_P = 0;
	public static final double TURRET_I = 0;
	public static final double TURRET_D = 0;
	public static final double REVERSE_INDEXER = 0;
	public static final double GYRO_P = 0;
	public static final double GYRO_I = 0;
	public static final double GYRO_D = 0;
	public static final double GYRO_TOLERANCE = 0;

	public static final int leftMasterID = 1; // front left == 1
	public static final int leftSlaveID = 2; // back left == 2
	public static final int rightMasterID = 3; // front right == 3
	public static final int rightSlaveID = 4; // back right == 4
	public static final int shooterID = 5;
	public static final int intakeID = 0;
	public static final int leftUltrasonicPort = 0;
	public static final int rightUltrasonicPort = 1;

	public static final double DISTANCE_BETWEEN_ULTRASONICS = 13.7;
	public static final double LEFT_ULTRASONIC_VOLTS_PER_INCH = 0.00966389875;
	public static final double RIGHT_ULTRASONIC_VOLTS_PER_INCH = 0.0095621735;

	@Deprecated
	public final double WHEEL_DIAMETER = 8.0;
	public final double WHEEL_BASE_INCHES = 27.0;
	public final double WHEEL_BASE_FEET = WHEEL_BASE_INCHES / 12;
	public final double WHEEL_BASE_METERS = .6858;

	public final double CYCLE_TIME = 0.05;

	// Wheels
	public final double kDriveWheelDiameterInches = 6.25;// 7.3
	public final double kDriveWheelDiameterFeet = kDriveWheelDiameterInches / 12;
	@Deprecated
	public final int kEncoderTicksperRev = 360; // was 360 changed to 256 maybe should be 256 ??
	@Deprecated
	public final int kEncoderReadsperRev = 4;
	@Deprecated
	public final int kEncoderCountsperRev = kEncoderTicksperRev * kEncoderReadsperRev;

	public final int kEncoderUnitsPerRev = 4096;

	public final double kMaxVelocityInchesPerSec = 100.0; // 110
	public final double kMaxAccelerationInchesPerSec = 100.0; // 650
	public final double kMaxJerkInchesPerSec = 8000.0; // 10000

	public final double turnMPConstant = -.03;// -.027 with no elevator
	public final boolean leftMasterReverseOutput = true;
	public final boolean leftSlaveReverseOutput = true;
	public final boolean rightMasterReverseOutput = true;
	public final boolean rightSlaveReverseOutput = true;
	public final boolean leftSensorPhase = false;
	public final boolean rightSensorPhase = true;

	public static final boolean REVERSE_LEFT_SENSOR = false;
	public static final boolean REVERSE_RIGHT_SENSOR = true;
	public static final boolean REVERSE_LEFT_OUTPUT = false;
	public static final boolean REVERSE_RIGHT_OUTPUT = true;

	public final double SHOOT_P = 0;
	public final double SHOOT_I = 0;
	public final double SHOOT_D = 0;
	public final double SHOOT_F = 0;

	public int driveSolenoidID1;
	public int driveSolenoidID2;
	public int deflectorID;
	public int outerGearSolenoidID1;
	public int outerGearSolenoidID2;
	public int innerGearSolenoidID1;
	public int innerGearSolenoidID2;
	public int indexerID;
	public int upperIntakeID;
	public int lowerIntakeID;
	public int turretID = 6;

	@Override
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

	@Override
	public String getName() {
		return "ProgrammingBot";
	}
}
