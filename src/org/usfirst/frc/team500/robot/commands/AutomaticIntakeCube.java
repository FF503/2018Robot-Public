package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.OI;
import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutomaticIntakeCube extends Command {
	boolean firstPress;
	boolean hasStopped;
	boolean rumble;
	boolean outOff = false;
	double stopTime, startTime;

	public AutomaticIntakeCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Intake.getInstance());
		requires(Carriage.getInstance());

	}

	public AutomaticIntakeCube(boolean turnOffOut) {
		outOff = turnOffOut;
		requires(Intake.getInstance());
		requires(Carriage.getInstance());
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// firstPress = true;
		hasStopped = false;
		stopTime = Timer.getFPGATimestamp();
		startTime = Timer.getFPGATimestamp();
		RobotState.getInstance().setCarriageReverse(true);
		RobotState.getInstance().setIntakeRunning(true);
		Carriage.getInstance().setCubeBlock(false);
//    	if(RobotState.getInstance().getIntakeRaised()) {
//    		RobotState.getInstance().setIntakeRaised(false);
//    		Intake.getInstance().lowerIntake();
//    	}
		Intake.getInstance().lowerIntake();
		RobotState.getInstance().setIntakeRaised(false);

		Carriage.getInstance().shootCubeBackwardFast();
		Intake.getInstance().intakeCube();
		if (RobotState.getInstance().getState() == RobotState.State.TELEOP) {
			OI.setDriveRumble(0.4);
			rumble = true;
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/*
		 * if (!OI.getIntakeButton()){ firstPress = false; }
		 */
		if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
			if ((Timer.getFPGATimestamp() - stopTime > 2.5) && !hasStopped) {
				if (outOff) {
					Intake.getInstance().stopIntake();
				} else {
					Intake.getInstance().ejectCube();
				}
				Carriage.getInstance().shootCubeForwardFast();
				hasStopped = true;
				stopTime = Timer.getFPGATimestamp();
			} else if (Timer.getFPGATimestamp() - stopTime > 0.2 && hasStopped) {
				Intake.getInstance().intakeCube();
				Carriage.getInstance().shootCubeBackwardFast();
				hasStopped = false;
				stopTime = Timer.getFPGATimestamp();
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		if (RobotState.getInstance().getState() == RobotState.State.TELEOP) {
			if (OI.getIntakeOverride()) {
				return OI.getEjectButton();
			} else {
				return Carriage.getInstance().carriageHasCube() || OI.getEjectButton();
			}
		} else {
			return Carriage.getInstance().carriageHasCube();
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		Intake.getInstance().stopIntake();
		// Carriage.getInstance().setCubeBlock(true);
		Carriage.getInstance().stopCarriage();
		RobotState.getInstance().setIntakeRunning(false);
		RobotState.getInstance().setCarriageReverse(false);
		if (RobotState.getInstance().getState() == RobotState.State.TELEOP) {
			rumble = false;
			OI.setDriveRumble(0.0);
		} else {
			Carriage.getInstance().setCubeBlock(true);
		}
//	   	Intake.getInstance().raiseIntake();
//	    RobotState.getInstance().setIntakeRaised(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
