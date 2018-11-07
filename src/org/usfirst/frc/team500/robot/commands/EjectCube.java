/*
 * Name:		 ReverseIntakeCommand
 * Purpose:		 Turns on the intake when button is pressed
 * Author:		 Roshan Raj
 * Date:		 January 2018
 * Comments:
 */

package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.Robot;
import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class EjectCube extends Command {
	double time, initTime;

	public EjectCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//    	if(RobotState.getInstance().getIntakeRaised()) {
//    		RobotState.getInstance().setIntakeRaised(false);
//    		Intake.getInstance().lowerIntake();
//    	}
		Intake.getInstance().ejectCube();
		Carriage.getInstance().shootCubeForwardFast();
		RobotState.getInstance().setCarriageReverse(false);
		RobotState.getInstance().setCarriageForward(true);
		RobotState.getInstance().setIntakeRunning(false);
		RobotState.getInstance().setReverseIntakeRunning(true);
		initTime = Timer.getFPGATimestamp();
		Carriage.getInstance().setCubeBlock(false);
		time = Robot.bot.intakeEjectTime;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

//    	if(Timer.getFPGATimestamp() - initTime > time) {
//    		
//    		initTime = Timer.getFPGATimestamp();
//    		time = Robot.bot.intakeEjectTime;
//    		Intake.getInstance().ejectCube();
//    		
//    	}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
			return (Timer.getFPGATimestamp() - initTime) > time && !Carriage.getInstance().carriageHasCube();
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotState.getInstance().setCarriageForward(false);
		RobotState.getInstance().setReverseIntakeRunning(false);
		Intake.getInstance().stopIntake();
		Carriage.getInstance().stopCarriage();
//		RobotState.getInstance().setIntakeRaised(true);
//		Intake.getInstance().raiseIntake();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}