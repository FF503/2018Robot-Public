package org.usfirst.frc.team500.robot.subsystems;

import org.usfirst.frc.team500.robot.Robot;
import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.commands.ElevatorLaserThread;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elevator extends Subsystem {

	private static Elevator instance = new Elevator();
	private ElevatorLaserThread laserThread;
	public boolean hasLaser = false;
	private double startTime, lastVel, lastAccel, accel, jerk, startAccelTime, startJerkTime;

	public static Elevator getInstance() {
		return instance;
	}

	private TalonSRX elevatorMotor;

	private double height;

	private Solenoid brake;
	private Solenoid lift;

	public Elevator() {
		try {
			laserThread = new ElevatorLaserThread();
			Notifier notifier = new Notifier(laserThread);
			hasLaser = true;
			notifier.startPeriodic(0.01);
		} catch (java.lang.RuntimeException e) {
			System.out.println("Laser object not constructed");
			e.printStackTrace();
		}
		if (!Robot.bot.getName().equals("ProgrammingBot")) {
			elevatorMotor = new TalonSRX(Robot.bot.elevatorID);
			elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
			elevatorMotor.setInverted(Robot.bot.elevatorMotorInverted);
			elevatorMotor.setSensorPhase(Robot.bot.elevatorSensorPhase);
			elevatorMotor.setNeutralMode(NeutralMode.Brake);
			brake = new Solenoid(Robot.bot.brakeID);
			if (!Robot.bot.getName().equals("PracticeBot")) {
				lift = new Solenoid(Robot.bot.liftID);
			}

		}
	}

//	public int getLaserHeight() {
//		return laser.getBytesReceived();
//	}	

//	public double getElevatorHeight(){
//		return filter(getLaserHeight(), getEncoderVal());
//	}

//	public double filter(double laser, double enc){
//		return laser + enc;
//	}

	public double getEncoderCounts() {
		return elevatorMotor.getSelectedSensorPosition(0);
	}

	public double getEncoderVelCounts() {
		return elevatorMotor.getSelectedSensorVelocity(0);
	}

	/**
	 * Takes sensor inputs to return a fused value representing the functions state
	 * estimation of elevator postion.
	 * 
	 * @param e  raw encoder inches
	 * @param l  raw laser inches
	 * @param de encoder velocity inches/sec
	 * @param dl laser velocity inches/sec
	 * @return filtered state estimate of elevator pose inches
	 */
	public double filter(double e, double l, double de, double dl) {
		double stateEstimate;
		if (Math.abs(e - l) < Robot.bot.sensorMarginThreshold) {
			stateEstimate = (e + l) / 2.0;
		} else if (l < Robot.bot.laserLowThreshold) {
			stateEstimate = e;
		}
		/*
		 * else if ((Math.abs(de) >= Robot.bot.elevatorZeroVelocityThreshold) &&
		 * (Math.abs(dl) <= Robot.bot.elevatorZeroVelocityThreshold)){ stateEstimate =
		 * e; } else if ((Math.abs(de) <= Robot.bot.elevatorZeroVelocityThreshold) &&
		 * (Math.abs(dl) >= Robot.bot.elevatorZeroVelocityThreshold)){ stateEstimate =
		 * l; }
		 */
		else {
			stateEstimate = l;
		}
		return stateEstimate;
	}

	public synchronized void setElevatorOutput(double outputVal) {
		elevatorMotor.set(ControlMode.PercentOutput, outputVal);
		if (outputVal < 0) {
			Carriage.getInstance().setCubeBlock(true);
		}
	}

	public void resetEncoder() {
		elevatorMotor.getSensorCollection().setPulseWidthPosition(0, 10);
	}

	public double getEncoderHeight() {
		return ((getEncoderCounts() / Robot.bot.kEncoderUnitsPerRev) * Math.PI * Robot.bot.elevatorSpoolDiameter * 1.0)
				+ Robot.bot.elevatorEncoderOffset;
	}

	public double getEncoderVelocity() {
		return getEncoderVelCounts() / Robot.bot.kEncoderUnitsPerRev * 10.0 * Math.PI * Robot.bot.elevatorSpoolDiameter;
	}

	public void calculateKinematics() {
		double vel = getEncoderVelocity();
		double time = Timer.getFPGATimestamp();
		accel = (vel - lastVel) / (time - startTime);
		jerk = (accel - lastAccel) / (time - startTime);
		lastAccel = accel;
		startTime = time;
		lastVel = vel;
	}

	public double getEncoderAccel() {
		double vel = getEncoderVelocity();
		double time = Timer.getFPGATimestamp();
		double accel = (vel - lastVel) / (time - startAccelTime);
		lastVel = vel;
		startAccelTime = time;
		return accel;
	}

	public double getEncoderJerk() {
		double accel = getEncoderAccel();
		double time = Timer.getFPGATimestamp();
		double jerk = (accel - lastAccel) / (time - startJerkTime);
		lastAccel = accel;
		startJerkTime = time;
		return jerk;
	}

	public double getElevatorStateEstimate() {
		return getEncoderHeight();
		// return laserThread.getLaserHeight();
		// return filter(getEncoderHeight(), laserThread.getLaserHeight(),
		// getEncoderVelocity(), laserThread.getLaserVelocity());
	}

	public boolean hasReset() {
		if (getElevatorStateEstimate() <= Robot.bot.ElevatorResetHeight) { ///////// motor
			return true;
		} else {
			return false;
		}
	}

	public boolean elevatorStopped() {
		if (Math.abs(laserThread.getLaserVelocity()) < Robot.bot.ElevatorJammedVelocityTolerance) {
			System.out.println("Elevator Jammed");
			return true;
		} else {
			return false;
		}
	}

	public void sendDashboardData() {
		if (!Robot.bot.getName().equals("ProgrammingBot")) {
			SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getMotorOutputPercent());
			SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
			SmartDashboard.putNumber("Elevator encoder height", getEncoderHeight());
			SmartDashboard.putNumber("Elevator encoder velocity", getEncoderVelocity());
			SmartDashboard.putNumber("Elevator encoder acceleration", accel);
			SmartDashboard.putNumber("Elevator encoder jerk", jerk);
			SmartDashboard.putString("Elevator location", RobotState.getInstance().getElevatorLocation().name());
			SmartDashboard.putNumber("Elevator filtered Height", getElevatorStateEstimate());
			SmartDashboard.putBoolean("Elevator brake engaged", RobotState.getInstance().getBrakeEngaged());
			SmartDashboard.putBoolean("Elevator motion profile done",
					RobotState.getInstance().getElevatorProfileDone());
			if (hasLaser) {
				laserThread.putDashboardData();
			}
		}
	}

	public void brakeOn() {
		brake.set(false);
		RobotState.getInstance().setBrakeEngaged(true);
	}

	public void brakeOff() {
		brake.set(true);
		RobotState.getInstance().setBrakeEngaged(false);
	}

	public void deployForks() {
		if (!Robot.bot.getName().equals("PracticeBot")) {
			lift.set(true);
		}
	}

	public void lockForks() {
		if (!Robot.bot.getName().equals("PracticeBot")) {
			lift.set(false);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

}