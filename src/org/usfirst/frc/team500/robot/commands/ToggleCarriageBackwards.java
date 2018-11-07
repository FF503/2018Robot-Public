package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ToggleCarriageBackwards extends Command {

	public ToggleCarriageBackwards() {

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Carriage.getInstance());

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (RobotState.getInstance().getCarriageReverse()) {
			Carriage.getInstance().stopCarriage();
			RobotState.getInstance().setCarriageReverse(false);
		} else {
			Carriage.getInstance().shootCubeBackwardFast();
			RobotState.getInstance().setCarriageReverse(true);
		}
		RobotState.getInstance().setCarriageForward(false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
