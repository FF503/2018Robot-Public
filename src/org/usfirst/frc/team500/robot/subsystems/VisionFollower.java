package org.usfirst.frc.team500.robot.subsystems;

import org.usfirst.frc.team500.robot.OI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionFollower extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	// double targetHeightAboveField = 0; //inches
	double cameraToTargetHight = 2.5;
	double cameraHight = 14; // inches
	double mountingAngle = 0; // in degrees

	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	public double getAngleOffsetHorizontal() {
		return table.getEntry("tx").getDouble(0.0);
	}

	public boolean hasTarget() {
		boolean targetVisible;
		if (table.getEntry("tv").getDouble(0.0) == 0) {
			targetVisible = false;
		} else {
			targetVisible = true;
		}
		return targetVisible;
	}

	public double getDistanceFromTarget() {
		return Math.abs((cameraToTargetHight)
				/ (Math.tan((Math.abs(Math.toRadians(getAngleOffsetVertical()) + mountingAngle)))));
	}

	public double getAreaOfTarget() {
		return table.getEntry("ta").getDouble(0.0);
	}

	public double getAngleOffsetVertical() {
		return table.getEntry("ty").getDouble(0.0);
	}

	public void setPipeline(int x) {
		table.getEntry("pipeline").forceSetNumber(x);
	}

	// 0 is on. 1 is off. 2 is blink.
	public void setLedMode(int x) {
		table.getEntry("ledMode").forceSetNumber(x);
	}

	private static VisionFollower instance = new VisionFollower();

	public static VisionFollower getInstance() {
		return instance;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void sendDashboardData() {
		SmartDashboard.putNumber("Horizontal angle offset", VisionFollower.getInstance().getAngleOffsetHorizontal());
		SmartDashboard.putNumber("Vertical angle offset", VisionFollower.getInstance().getAngleOffsetVertical());
		SmartDashboard.putNumber("Distance from target", VisionFollower.getInstance().getDistanceFromTarget());
		SmartDashboard.putNumber("Area of target", VisionFollower.getInstance().getAreaOfTarget());
		SmartDashboard.putBoolean("Vision Button State", OI.getVisionButton());
	}
}