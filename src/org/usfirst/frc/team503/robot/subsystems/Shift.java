package org.usfirst.frc.team503.robot.subsystems;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

public class Shift extends Subsystem {

	private DoubleSolenoid driveShift, pto;

	/***************************************************************************************
	 * Drive - Constructor
	 ***************************************
	 ************************************************/
	public Shift() {

		System.out.println("constructing shift train");
		// System.out.println(Robot.bot.driveSolenoidID1 + ", " +
		// Robot.bot.driveSolenoidID2);
		driveShift = new DoubleSolenoid(0, 1);
		pto = new DoubleSolenoid(2, 3);

		System.out.println("constructing shift train completed...");
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	private static Shift instance = new Shift();

	public static Shift getInstance() {
		return instance;
	}

	public void updateDriveMode(RobotState.RobotDriveMode mode) {
		RobotState.getInstance().setDriveMode(mode);
		switch (mode) {
		case LOW:
			System.out.println("going low");
			// if(Robot.bot.getName().equals("PracticeBot")) {
			// driveShift.set(DoubleSolenoid.Value.kForward);
			// pto.set(DoubleSolenoid.Value.kForward);
			// }
			// else {
			driveShift.set(DoubleSolenoid.Value.kReverse);
			pto.set(DoubleSolenoid.Value.kForward);
			// }

			break;
		case HIGH:
			System.out.println("going high");
			// if(Robot.bot.getName().equals("PracticeBot")) {
			// driveShift.set(DoubleSolenoid.Value.kReverse);
			// pto.set(DoubleSolenoid.Value.kForward);
			// }
			// else {
			driveShift.set(DoubleSolenoid.Value.kForward);
			pto.set(DoubleSolenoid.Value.kForward);
			// }
			break;
		case CLIMB:
			System.out.println("going climb");
			driveShift.set(DoubleSolenoid.Value.kReverse);
			pto.set(DoubleSolenoid.Value.kReverse);
			break;
		}
	}

	public void upShift() {
		if (RobotState.getInstance().getDriveMode() == RobotState.RobotDriveMode.LOW
				&& Drive.getInstance().ticksToRotations(Drive.getInstance()
						.getAverageEncoderCounts()) > (Robot.bot.motorShiftRPM + Robot.bot.shiftToleranceRPM)) {
			updateDriveMode(RobotState.RobotDriveMode.HIGH);
		}
	}

	public void downShift() {
		if (RobotState.getInstance().getDriveMode() == RobotState.RobotDriveMode.HIGH
				&& Drive.getInstance().ticksToRotations(Drive.getInstance()
						.getAverageEncoderCounts()) < (Robot.bot.motorShiftRPM - Robot.bot.shiftToleranceRPM)) {
			updateDriveMode(RobotState.RobotDriveMode.LOW);
		}
	}

	public void sendDashboardData() {
		SmartDashboard.putString("Robot drive mode", RobotState.getInstance().getDriveMode().name());
	}

}
