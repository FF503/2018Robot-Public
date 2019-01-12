package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Gyro;
import org.usfirst.frc.team503.robot.utils.SynchronousPID;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PivotCommand extends Command {
	SynchronousPID pidController;
	double angle;

	public PivotCommand(double setpoint) {
		pidController = new SynchronousPID(Robot.bot.PIVOT_P, Robot.bot.PIVOT_I, Robot.bot.PIVOT_D);
		pidController.setSetpoint(setpoint);
		pidController.setInputRange(-180, 180);
		pidController.setOutputRange(-1, 1);
		angle = setpoint;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Gyro.getInstance().resetGyro();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double currAngle = Gyro.getInstance().getAngle();
		/*
		 * if(currAngle < angle) { Drive.getInstance().tankDrive(0.8, -0.8); }
		 */
		double power = pidController.calculate(Gyro.getInstance().getAngle());
		Drive.getInstance().tankDrive(power, -power);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return Gyro.getInstance().getAngle() >= angle;
		return pidController.onTarget(Robot.bot.PIVOT_TOLERANCE);
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
