package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CarriageForwards extends Command {
	boolean slow = false;

	public CarriageForwards() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		slow = false;
	}

	public CarriageForwards(boolean slow) {
		this.slow = slow;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (slow) {
			Carriage.getInstance().shootCubeForwardSlow();
		} else {
			Carriage.getInstance().shootCubeForwardFast();
		}
		Carriage.getInstance().setCubeBlock(false);
		RobotState.getInstance().setCarriageForward(true);
		RobotState.getInstance().setCarriageReverse(false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Carriage.getInstance().stopCarriage();
		;
		RobotState.getInstance().setCarriageForward(false);
		RobotState.getInstance().setCarriageReverse(false);
		Carriage.getInstance().setCubeBlock(true);
	}
}
