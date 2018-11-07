/*
 * Name:		 IntakeSubsystem 
 * Purpose:		 Subsystem for intake 
 * Author:		 Roshan Raj
 * Date:		 January 2018
 * Comments:
 */

package org.usfirst.frc.team500.robot.subsystems;

import org.usfirst.frc.team500.robot.Robot;
//import org.usfirst.frc.team500.robot.RobotHardwareProgrammingBot;
import org.usfirst.frc.team500.robot.RobotState;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.PIDOutput;
//import edu.wpi.first.wpilibj.PIDSource;
//import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends Subsystem /* implements PIDSource, PIDOutput */ {

	private Spark intakeLeftMotor, intakeRightMotor;
	private DoubleSolenoid intakeLift;
	private PowerDistributionPanel pdp;

	// private TalonSRX intakeLiftMotor;
	// double pidOutput;
	// private PIDController Controller;

	public Intake() {
		intakeLeftMotor = new Spark(Robot.bot.leftIntakeID);
		intakeRightMotor = new Spark(Robot.bot.rightIntakeID);
		// intakeIRSensor = new AnalogInput(Robot.bot.intakeIRPort);
		intakeLift = new DoubleSolenoid(Robot.bot.intakeLiftForward, Robot.bot.intakeLiftReverse);
		// pdp = new PowerDistributionPanel();
		// pdp = new PDPJNI();

	}

	private static Intake instance = new Intake();

	public static Intake getInstance() {
		return instance;
	}

	private void setMotorPower(double power) {
		intakeLeftMotor.set(power * Robot.bot.leftIntakeFlipped);
		intakeRightMotor.set(power * Robot.bot.rightIntakeFlipped);
	}

	public void lowerIntake() {
		intakeLift.set(DoubleSolenoid.Value.kForward);
	}

	public void raiseIntake() {
		intakeLift.set(DoubleSolenoid.Value.kReverse);
	}

	public void intakeCube() {
		setMotorPower(Robot.bot.intakePower);
	}

	public void ejectCube() {
		setMotorPower(-0.6);
	}

	public void stopIntake() {
		setMotorPower(0.0);
	}

	/*
	 * public boolean isStall() { return ((pdp.getCurrent(5) >
	 * Robot.bot.INTAKE_STALL_THRESHOLD) || (pdp.getCurrent(6) >
	 * Robot.bot.INTAKE_STALL_THRESHOLD)); }
	 * 
	 * public boolean getStalled() { if (isStall() && (pdp.getCurrent(5) >
	 * Robot.bot.INTAKE_STALL_THRESHOLD) || (pdp.getCurrent(6) >
	 * Robot.bot.INTAKE_STALL_THRESHOLD)) { return true; } else { return false; } }
	 */
	public void sendDashboardData() {
		SmartDashboard.putBoolean("Intaking cube", RobotState.getInstance().getIntakeRunning());
		SmartDashboard.putBoolean("Ejecting cube", RobotState.getInstance().getReverseIntakeRunning());
		SmartDashboard.putNumber("Intake motor power", intakeLeftMotor.get());
		SmartDashboard.putBoolean("Intake raised", RobotState.getInstance().getIntakeRaised());
		// SmartDashboard.putNumber("Left Intake current",
		// pdp.getPDPChannelCurrent((byte) 4, 0));
		// SmartDashboard.putNumber("Left Intake voltage", pdp.getPDPVoltage(9));
		/*
		 * SmartDashboard.putBoolean("Intake Stall Status", isStall());
		 * SmartDashboard.putString("Intake Stalling Motors", getStalled());
		 */
		// SmartDashboard.putNumber("Left Intake motor current", pdp.getCurrent(9));
		// SmartDashboard.putNumber("Right Intake motor current", pdp.getCurrent(4));
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here i.e. setDefaultCommand(new
		// MySpecialCommand());
	}

}