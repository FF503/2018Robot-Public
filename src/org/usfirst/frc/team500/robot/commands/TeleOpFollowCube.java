package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Drive;
import org.usfirst.frc.team500.robot.subsystems.VisionFollower;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TeleOpFollowCube extends Command {

	double basePower = 0.8;
	double kp = 0.07;
	double turn, left, right;
	double stopProportion = 0.003;
	double stopTolerance = 28;
	double goneTolerance = 2;
	double stop;
	double power;

	public TeleOpFollowCube() {
		// Use requires() s(chassis);
		requires(VisionFollower.getInstance());
		// requires(Drive.getInstance());
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// VisionFollower.getInstance().setPipeline(0);
		SmartDashboard.putBoolean("Follow Cube ended", false);
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
		// System.out.println("execute");
	}
	// }

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return /* (VisionFollower.getInstance().getAreaOfTarget() >= stopTolerance) || */ Carriage.getInstance()
				.carriageHasCube();
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