package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ToggleCarriageForwards extends Command {

	public ToggleCarriageForwards() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Carriage.getInstance());

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (RobotState.getInstance().getCarriageForward()) {
			Carriage.getInstance().stopCarriage();
			RobotState.getInstance().setCarriageForward(false);
		} else {
			Carriage.getInstance().shootCubeForwardFast();
			RobotState.getInstance().setCarriageForward(true);
		}
		RobotState.getInstance().setCarriageReverse(false);
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
