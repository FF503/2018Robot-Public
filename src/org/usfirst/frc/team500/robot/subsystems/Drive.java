package org.usfirst.frc.team500.robot.subsystems;

import org.usfirst.frc.team500.robot.Robot;
import org.usfirst.frc.team500.robot.RobotState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/****************************************************************
 * Class Drive.java Description Provides all services for the robot drive train.
 * 
 * @author Frog Force 503 Programming team
 * @version 0.1 Change History 0.0 Initial Load
 * 
 ****************************************************************/

public class Drive extends Subsystem {

	private TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;

	private static double[] motorCurrents = new double[2];
	private static double lastTime, curTime, lastVelR, curVelR, curAccelR, differenceCurrent, lastAccelR, curJerkR,
			lastVelL, curVelL, curAccelL, lastAccelL, curJerkL;

	/***************************************************************************************
	 * Drive - Constructor
	 ***************************************
	 ************************************************/
	public Drive() {
		curTime = System.currentTimeMillis() / 1000;
		lastTime = curTime;
		curAccelR = 0;
		lastAccelR = 0;
		lastVelR = 0;
		curVelR = 0;
		curAccelL = 0;
		lastAccelL = 0;
		lastVelL = 0;
		curVelL = 0;
		motorCurrents[0] = 0;
		motorCurrents[1] = 0;

		System.out.println("constructing drive train");
		// System.out.println(Robot.bot.driveSolenoidID1 + ", " +
		// Robot.bot.driveSolenoidID2);

		leftMaster = new TalonSRX(Robot.bot.leftMasterID);
		leftSlave = new TalonSRX(Robot.bot.leftSlaveID);
		rightMaster = new TalonSRX(Robot.bot.rightMasterID);
		rightSlave = new TalonSRX(Robot.bot.rightSlaveID);

		// leftMaster.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);
		// rightMaster.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

		// leftMaster.configEncoderCodesPerRev(Robot.bot.kEncoderTicksperRev);
		// rightMaster.configEncoderCodesPerRev(Robot.bot.kEncoderTicksperRev);

		leftSlave.set(ControlMode.Follower, Robot.bot.leftMasterID);
		rightSlave.set(ControlMode.Follower, Robot.bot.rightMasterID);

		leftMaster.setSensorPhase(Robot.bot.leftSensorPhase);
		leftMaster.setInverted(Robot.bot.leftMasterReverseOutput);
		leftSlave.setInverted(Robot.bot.leftSlaveReverseOutput);

		// leftMaster.reverseSensor(false);
		// leftMaster.reverseOutput(true);

		// rightMaster.reverseSensor(true);
		// rightMaster.reverseOutput(false);

		rightMaster.setSensorPhase(Robot.bot.rightSensorPhase);
		rightMaster.setInverted(Robot.bot.rightMasterReverseOutput);
		rightSlave.setInverted(Robot.bot.rightSlaveReverseOutput);
		// rightMaster.reverseSensor(true);
		// rightMaster.reverseOutput(true);

		// leftMaster.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, 20);
		// rightMaster.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, 20);

		System.out.println("constructing drive train completed...");
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	private static Drive instance = new Drive();

	public static Drive getInstance() {
		return instance;
	}

	/********************************************************************************
	 * Section - Routines for Driving Robot
	 ******************************************************************************/

	/*
	 * Arcade Drive Routine
	 */
	public void arcadeDrive(double moveValue, double rotateValue) {
		double leftMotorSpeed;
		double rightMotorSpeed;

		moveValue = limit(moveValue);
		rotateValue = limit(rotateValue);

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}
		setMotorOutputs(leftMotorSpeed, rightMotorSpeed);
	}

	/*
	 * Tank Drive Routine
	 */
	public void tankDrive(double leftValue, double rightValue) {

		leftValue = limit(leftValue);
		rightValue = limit(rightValue);

		setMotorOutputs(leftValue, rightValue);
	}

	/*
	 * Limit values from -1 to +1
	 */
	private static double limit(double num) {
		if (num > 1.0) {
			num = 1.0;
		} else if (num < -1.0) {
			num = -1.0;
		}
		return num;
	}

	/*
	 * Send command to drive motors
	 */
	public void setMotorOutputs(double leftSpeed, double rightSpeed) {

		if (RobotState.getInstance().getDriveMode() == RobotState.RobotDriveMode.CLIMB
				&& ((leftSpeed < 0.0) || (rightSpeed < 0.0))) {
			leftMaster.set(ControlMode.PercentOutput, 0.0);
			rightMaster.set(ControlMode.PercentOutput, 0.0);
		} else {
			leftMaster.set(ControlMode.PercentOutput, -leftSpeed);
			rightMaster.set(ControlMode.PercentOutput, rightSpeed);
		}

	}

	/********************************************************************************
	 * Section - Routines to interface with Encoders
	 *******************************************************************************/

	/*
	 * Reset both encoder positions to 0
	 */
	public void resetEncoders() {
		/*
		 * leftMaster.setPosition(0); rightMaster.setPosition(0);
		 * 
		 * leftMaster.setEncPosition(0); rightMaster.setEncPosition(0);
		 */

		leftMaster.getSensorCollection().setPulseWidthPosition(0, 10);
		rightMaster.getSensorCollection().setPulseWidthPosition(0, 10);
	}

	/*
	 * Read Left Encoder in rotations and convert to inches
	 */
	private double getLeftDistanceInches() {
		int encPos = leftMaster.getSelectedSensorPosition(0);
		return ticksToInches(encPos);
	}

	/**
	 * Gets left motor speed
	 * 
	 * @return left speed in inches/sec
	 */
	public double getLeftVelocityInches() {
		return ticksToInches(leftMaster.getSelectedSensorVelocity(0) * 10.0);
	}

	/**
	 * Gets right motor speed
	 * 
	 * @return right speed in inches/sec
	 */
	public double getRightVelocityInches() {
		return ticksToInches(rightMaster.getSelectedSensorVelocity(0) * 10.0);
	}

	/*
	 * Get difference of drive motor currents
	 */
	public double getCurrentDiff(boolean abs) {
		motorCurrents[0] = leftMaster.getOutputCurrent();
		motorCurrents[1] = rightMaster.getOutputCurrent();

		if (abs) {
			differenceCurrent = Math.abs(Math.abs(motorCurrents[0]) - Math.abs(motorCurrents[1]));
		} else
			differenceCurrent = motorCurrents[0] - motorCurrents[1];

		return differenceCurrent;
	}

	/**
	 * Read Right Encoder in rotations and convert to inches
	 */
	private double getRightDistanceInches() {
		int encPos = rightMaster.getSelectedSensorPosition(0);
		return ticksToInches(encPos);
	}

	public double getAverageEncoderCounts() {
		return (getLeftPosition() + getRightPosition()) / 2.0;
	}

	/**
	 * Return average of left and right encoder values in inches
	 */
	public double getAverageDistanceInches() {
		double encPos = (getLeftDistanceInches() + getRightDistanceInches()) / 2;
		return encPos;
	}

	/**
	 * Read Left Encoder and return absolute click count
	 * 
	 * @return Native units of encoder
	 */
	public int getLeftPosition() {
		// the talon optical encoder returns position in revolutions-multiply times
		// counts/rev to get to position
		return leftMaster.getSelectedSensorPosition(0);
	}

	/**
	 * Read Right Encoder and return absolute click count
	 * 
	 * @return Native units of encoder
	 */
	public int getRightPosition() {
		// the talon optical encoder returns position in revolutions-multiply times
		// counts/rev to get to position
		return rightMaster.getSelectedSensorPosition(0); // getposition = enc rotations
	}

	/********************************************************************************
	 * Section - Encoder Conversion Routines
	 *******************************************************************************/

	private static double ticksToInches(double ticks) {
		return rotationsToInches(ticksToRotations(ticks));
		// return(ticks / (double) Robot.bot.kEncoderCountsperRev) * ((double)
		// Robot.bot.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rotationsToInches(double rotations) {
		return rotations * (Robot.bot.kDriveWheelDiameterInches * Math.PI);
	}

	public static double ticksToRotations(double ticks) {
		return ticks / Robot.bot.kEncoderUnitsPerRev;
		// return (ticks / Robot.bot.kEncoderCountsperRev);
	}

	private static double inchesToRotations(double inches) {
		return inches / (Robot.bot.kDriveWheelDiameterInches * Math.PI);
	}

	public void setBrakeMode(boolean on) {
		if (on) {
			leftMaster.setNeutralMode(NeutralMode.Brake);
			leftSlave.setNeutralMode(NeutralMode.Brake);
			rightMaster.setNeutralMode(NeutralMode.Brake);
			rightSlave.setNeutralMode(NeutralMode.Brake);
		} else {
			leftMaster.setNeutralMode(NeutralMode.Coast);
			leftSlave.setNeutralMode(NeutralMode.Coast);
			rightMaster.setNeutralMode(NeutralMode.Coast);
			rightSlave.setNeutralMode(NeutralMode.Coast);
		}
	}

	public void calculateKinematicData() {
		curVelR = getRightVelocityInches();
		curVelL = getLeftVelocityInches();
		curTime = Timer.getFPGATimestamp();
		curAccelR = derivative(curVelR, lastVelR, curTime, lastTime);
		curAccelL = derivative(curVelL, lastVelL, curTime, lastTime);
		lastVelR = curVelR;
		lastVelL = curVelL;
		curJerkR = derivative(curAccelR, lastAccelR, curTime, lastTime);
		curJerkL = derivative(curAccelL, lastAccelL, curTime, lastTime);
		lastAccelR = curAccelR;
		lastAccelL = curAccelL;
		lastTime = curTime;
		System.out.println("Min velocity: " + Math.min(curVelR, curVelL));
		System.out.println("Min acceleration: " + Math.min(curAccelR, curAccelL));
		System.out.println("Min jerk: " + Math.min(curJerkR, curJerkL));
	}

	private static double rpmToInchesPerSecond(int rpm) {
		return rotationsToInches(rpm) / 60;
	}

	public double derivative(double vel2, double vel1, double t2, double t1) {
		return (vel2 - vel1) / (t2 - t1);
	}

	public void sendDashboardData() {
		SmartDashboard.putNumber("Motor current difference", Drive.getInstance().getCurrentDiff(true));
		SmartDashboard.putNumber("Left master current", leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("Right master current", rightMaster.getOutputCurrent());
		SmartDashboard.putNumber("Left encoder counts", leftMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right encoder counts", rightMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Left motor output", leftMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Right motor output", rightMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Left encoder inches", getLeftDistanceInches());
		SmartDashboard.putNumber("Right encoder inches", getRightDistanceInches());
		SmartDashboard.putNumber("Average encoder counts", getAverageEncoderCounts());
		SmartDashboard.putNumber("Right Motor Velocity", getRightVelocityInches());
		SmartDashboard.putNumber("Left Motor Velocity", getLeftVelocityInches());
		SmartDashboard.putNumber("Min Motor Vel", Math.min(getRightVelocityInches(), getLeftVelocityInches()));
		SmartDashboard.putNumber("Min Motor Accel", Math.min(curAccelR, curAccelL));
		SmartDashboard.putNumber("Min Motor Jerk", Math.min(curJerkL, curJerkR));
		SmartDashboard.putNumber("Left master bus voltage", leftMaster.getBusVoltage());
		SmartDashboard.putNumber("Right master bus voltage", rightMaster.getBusVoltage());
		SmartDashboard.putNumber("Left master output voltage", leftMaster.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right master output voltage", rightMaster.getMotorOutputVoltage());
		SmartDashboard.putBoolean("Drive motion profile done", RobotState.getInstance().getDriveProfileDone());
		SmartDashboard.putBoolean("Robot is reversed", RobotState.getInstance().getDriveReversed());
	}

}
