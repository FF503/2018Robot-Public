package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.OI;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RumbleJoystickCommand extends Command {
	double startTime;
	boolean finished = false;

	public RumbleJoystickCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		startTime = Timer.getFPGATimestamp();
		OI.setDriveRumble(0.4);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	public void finish() {
		finished = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finished;// Timer.getFPGATimestamp() - startTime > 1.0;
	}

	// Called once after isFinished returns true
	protected void end() {
		OI.setDriveRumble(0.0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
