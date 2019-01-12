package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForwardTimeCommand extends Command {
	double power, time;
	double start;

	public DriveForwardTimeCommand(double power, double time) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.power = power;
		this.time = time;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		start = Timer.getFPGATimestamp();
		Drive.getInstance().tankDrive(power, power);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Timer.getFPGATimestamp() - start) > time;
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
