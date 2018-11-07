package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ToggleIntakeCube extends Command {

	public ToggleIntakeCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (RobotState.getInstance().getIntakeRunning()) {
			Intake.getInstance().stopIntake();
//    		Carriage.getInstance().setCubeBlock(true);
			RobotState.getInstance().setIntakeRunning(false);
		} else {
//    		Carriage.getInstance().setCubeBlock(true);
			Intake.getInstance().intakeCube();
			RobotState.getInstance().setIntakeRunning(true);
		}
		RobotState.getInstance().setReverseIntakeRunning(false);
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
