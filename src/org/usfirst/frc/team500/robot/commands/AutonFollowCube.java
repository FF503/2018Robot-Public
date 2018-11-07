package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Drive;
import org.usfirst.frc.team500.robot.subsystems.Gyro;
import org.usfirst.frc.team500.robot.subsystems.VisionFollower;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonFollowCube extends Command {

	double basePower = 0.4;
	double kp = 0.02;
	double turn, left, right, stopPower, power;

	public double steer = 0.7;
	double stopProportion = 0.002;
	double stopTolerance = 18;
	double foundThresh = 2.0;
	double turnAngle = -503;

	boolean turnRight = false;
	boolean found = false;
	boolean runp = false;
	int pipeline = 0;
	String p = "";

	/**
	 * @param right - true if you want to go right, false if otherwise.
	 */
	public AutonFollowCube(boolean right) {
		// Use requires() s(chassis);
		requires(VisionFollower.getInstance());
		turnRight = right;
		// requires(Drive.getInstance());
	}

	/**
	 * 
	 * @param right - true if you want to go right, false if otherwise.
	 * @param stop  - Area at which the robot will stop in
	 */
	public AutonFollowCube(boolean right, double stop) {
		requires(VisionFollower.getInstance());
		turnRight = right;
		stopTolerance = stop;
	}

	/**
	 * 
	 * @param right - true if you want to go right, false if otherwise.
	 * @param stop  - Area at which the robot will stop in.
	 * @param found - The area which the cube has to be for the robot to follow it.
	 */
	public AutonFollowCube(boolean right, double stop, double found) {
		requires(VisionFollower.getInstance());
		turnRight = right;
		stopTolerance = stop;
		foundThresh = found;
	}

	/**
	 * 
	 * @param right - true if you want to go right, false if otherwise.
	 * @param stop  - Area at which the robot will stop in.
	 * @param found - The area which the cube has to be for the robot to follow it.
	 * @param p     - Name of alternate p
	 */
	public AutonFollowCube(boolean right, double stop, double found, int pipeline) {
		requires(VisionFollower.getInstance());
		turnRight = right;
		stopTolerance = stop;
		foundThresh = found;
		this.pipeline = pipeline;
	}

	/**
	 * 
	 * @param right - true if you want to go right, false if otherwise.
	 * @param stop  - Area at which the robot will stop in.
	 * @param found - The area which the cube has to be for the robot to follow it.
	 * @param p     - Name of p
	 */
	public AutonFollowCube(boolean right, double stop, double found, String p) {
		requires(VisionFollower.getInstance());
		turnRight = right;
		stopTolerance = stop;
		foundThresh = found;
		this.p = p;
	}

	/**
	 * 
	 * @param right - true if you want to go right, false if otherwise.
	 * @param stop  - Area at which the robot will stop in.
	 * @param found - The area which the cube has to be for the robot to follow it.
	 * @param angle - The angle the robot has to be before the robot goes to follow
	 *              the cube.
	 */
	public AutonFollowCube(boolean right, double stop, double found, double angle) {
		requires(VisionFollower.getInstance());
		turnRight = right;
		stopTolerance = stop;
		foundThresh = found;
		turnAngle = angle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		VisionFollower.getInstance().setPipeline(pipeline);
		if (turnRight) {
			steer = -steer; // Comment this line out if you want to spin left if target not visible
		}
		SmartDashboard.putBoolean("Follow Cube ended", false);
		Gyro.getInstance().resetGyro();
		if (VisionFollower.getInstance().getAreaOfTarget() < foundThresh && !p.equals("")) {
			runp = true;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/*
		 * if(FRCVision2018.getInstance().getAreaOfTarget() >= stopTolerance) {
		 * DriveSubsystem.getInstance().tankDrive(0, 0);
		 * 
		 * } else if(FRCVision2018.getInstance().getAreaOfTarget() < stopTolerance) {
		 */
		// if(OI.getVisionButton()) {
		// VisionFollower.getInstance().setPipeline(3);
		if (VisionFollower.getInstance().getAreaOfTarget() > foundThresh
				&& Math.abs(Gyro.getInstance().getAngle()) > turnAngle) {
			found = true;
		}

		if (VisionFollower.getInstance().hasTarget() && VisionFollower.getInstance().getAreaOfTarget() > foundThresh
				&& Math.abs(Gyro.getInstance().getAngle()) > turnAngle) {
			turn = kp * VisionFollower.getInstance().getAngleOffsetHorizontal();
			stopPower = stopProportion * VisionFollower.getInstance().getAreaOfTarget();
			power = basePower - stopPower;
			left = power + turn;
			right = power - turn;

			if (left < 0) {
				left = 0;
			}

			if (right < 0) {
				right = 0;
			}
		} else {
			right = steer;
			left = -steer;
		}
		SmartDashboard.putNumber("Left follow power", left);
		SmartDashboard.putNumber("Right follow power", right);
		Drive.getInstance().tankDrive(left, right);
		// System.out.println("execute");
	}
	// }

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return VisionFollower.getInstance().getAreaOfTarget() > stopTolerance
				|| Carriage.getInstance().carriageHasCube() || runp;
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().tankDrive(0, 0);
		SmartDashboard.putBoolean("Follow Cube ended", true);

//    	if(RobotState.getInstance().getState() == RobotState.State.TELEOP) {
//    		VisionFollower.getInstance().setPipeline(1);
//    	}else if(RobotState.getInstance().getState() == RobotState.State.AUTON) {
//    		VisionFollower.getInstance().setPipeline(0);
//    	}
		// System.out.println("end");

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
