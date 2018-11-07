package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CloseIndexer extends Command {

	public CloseIndexer() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		Carriage.getInstance().setCubeBlock(true);
		if (RobotState.getInstance().getState() == RobotState.State.TELEOP) {
			Intake.getInstance().raiseIntake();
			RobotState.getInstance().setIntakeRaised(true);
		}
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
