package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopElevatorCommand extends Command {

	public TeleopElevatorCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(Elevator.getInstance());
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/*
		 * if(Elevator.getInstance().getElevatorStateEstimate() >
		 * Robot.bot.elevatorSafetyShiftHeight &&
		 * RobotState.getInstance().getDriveMode() != RobotState.RobotDriveMode.LOW) {
		 * Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.LOW); }
		 */
		if (Math.abs(OI.getOperatorLeftYValue()) > Robot.bot.elevatorJoystickThreshold) {
			RobotState.getInstance().setElevatorProfileDone(true);
			if (Elevator.getInstance()
					.getElevatorStateEstimate() < Robot.bot.elevatorEncoderOffset) /*
																					 * && !Elevator.getInstance().
																					 * elevatorStopped()
																					 */ {
				if (OI.getOperatorLeftYValue() < 0) {
					Elevator.getInstance().brakeOff();
					Elevator.getInstance().setElevatorOutput(OI.getOperatorLeftYValue() * Robot.bot.elevatorSlowDownUp);
				} else if (OI.overrideElevator()) {
					Elevator.getInstance().brakeOff();
					Elevator.getInstance()
							.setElevatorOutput(OI.getOperatorLeftYValue() * Robot.bot.elevatorSlowDownDown);
				} else {
					Elevator.getInstance().setElevatorOutput(0);
					Elevator.getInstance().brakeOn();
				}
			} else {
				if (OI.getOperatorLeftYValue() < 0) {
					Elevator.getInstance().brakeOff();
					Elevator.getInstance().setElevatorOutput(OI.getOperatorLeftYValue() * Robot.bot.elevatorSlowDownUp);
				} else {
					Elevator.getInstance().brakeOff();
					Elevator.getInstance()
							.setElevatorOutput(OI.getOperatorLeftYValue() * Robot.bot.elevatorSlowDownDown);
				}
			}
		}
		/*
		 * else if (!RobotState.getInstance().getElevatorProfileDone()){
		 * Elevator.getInstance().brakeOff(); }
		 */
		else if (!RobotState.getInstance().getElevatorResetting()
				&& RobotState.getInstance().getElevatorProfileDone()) {
			Elevator.getInstance().setElevatorOutput(0);
			Elevator.getInstance().brakeOn();
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !DriverStation.getInstance().isOperatorControl();
	}

	// Called once after isFinished returns true
	protected void end() {
		Elevator.getInstance().setElevatorOutput(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}