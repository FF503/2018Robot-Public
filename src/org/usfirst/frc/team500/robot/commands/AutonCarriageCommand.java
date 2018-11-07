package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.Robot;
import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutonCarriageCommand extends Command {
	boolean reverse;
	boolean slow;
	Timer timer;
	boolean started;

	public AutonCarriageCommand(boolean reverse) {
		this.reverse = reverse;
		this.slow = false;
		timer = new Timer();
		requires(Carriage.getInstance());
	}

	public AutonCarriageCommand(boolean reverse, boolean slow) {
		this.reverse = reverse;
		this.slow = slow;
		timer = new Timer();
		requires(Carriage.getInstance());
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Carriage.getInstance().stopCarriage();
		RobotState.getInstance().setCarriageReverse(false);
		RobotState.getInstance().setCarriageForward(false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (RobotState.getInstance().getDriveProfileDone() && RobotState.getInstance().getElevatorProfileDone()) {
			if (!started) {
				Carriage.getInstance().setCubeBlock(false);
				if (reverse) {
					if (slow) {
						Carriage.getInstance().shootCubeBackwardSlow();
					} else {
						Carriage.getInstance().shootCubeBackwardFast();

					}
					RobotState.getInstance().setCarriageReverse(true);
					RobotState.getInstance().setCarriageForward(false);
				} else {
					if (slow) {
						Carriage.getInstance().shootCubeForwardSlow();
					} else {
						Carriage.getInstance().shootCubeForwardFast();
					}
					Carriage.getInstance().shootCubeForwardFast();
					RobotState.getInstance().setCarriageReverse(false);
					RobotState.getInstance().setCarriageForward(true);
				}
				started = true;
				timer.start();
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() > Robot.bot.carriageShootTime;
	}

	// Called once after isFinished returns true
	protected void end() {
		Carriage.getInstance().stopCarriage();
		RobotState.getInstance().setCarriageReverse(false);
		RobotState.getInstance().setCarriageForward(false);
		Carriage.getInstance().setCubeBlock(true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
