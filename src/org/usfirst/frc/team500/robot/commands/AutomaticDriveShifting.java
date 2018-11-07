package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Shift;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutomaticDriveShifting extends Command {
	// When climbing, cancel this method.
	public AutomaticDriveShifting() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.LOW);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Shift.getInstance().upShift();
		Shift.getInstance().downShift();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !DriverStation.getInstance().isOperatorControl();
	}

	// Called once after isFinished returns true
	protected void end() {
		Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.HIGH);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
