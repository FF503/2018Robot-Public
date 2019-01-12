package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.VisionFollower;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * 
 */
public class FollowCube extends Command {

	double basePower = 0.2;
	double kp = 0.1;
	double turn, left, right;
	double stopProportion = 0.0001;
	double stopTolerance = 18;
	double stop;
	double power;
	double startTime;
	double timeout;

	public FollowCube(double timeout) {
		// Use requires() s(chassis);
		requires(VisionFollower.getInstance());
		// requires(Drive.getInstance());
		this.timeout = timeout;
	}

	public FollowCube(double stopTolerance, boolean nothing) {
		this.stopTolerance = stopTolerance;
	}

	public FollowCube() {
		this(3.0);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		SmartDashboard.putBoolean("Follow Cube ended", false);
		SmartDashboard.putBoolean("Follow cube timed out", false);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		turn = kp * VisionFollower.getInstance().getAngleOffsetHorizontal();
		stop = stopProportion * VisionFollower.getInstance().getAreaOfTarget();
		power = basePower - stop;
		left = power + turn;
		right = power - turn;

		if (left < 0) {
			left = 0;
		}

		if (right < 0) {
			right = 0;
		}
		SmartDashboard.putNumber("Left follow power", left);
		SmartDashboard.putNumber("Right follow power", right);
		Drive.getInstance().tankDrive(left, right);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (VisionFollower.getInstance().getAreaOfTarget() > stopTolerance)
				|| ((Timer.getFPGATimestamp() - startTime) > timeout);
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().tankDrive(0, 0);
		if ((Timer.getFPGATimestamp() - startTime) > timeout) {
			SmartDashboard.putBoolean("Follow cube timed out", true);
		} else {
			SmartDashboard.putBoolean("Follow Cube ended", true);
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}