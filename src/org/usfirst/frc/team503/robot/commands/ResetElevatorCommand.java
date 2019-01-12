package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.OI;
import org.usfirst.frc.team503.robot.Robot;
import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ResetElevatorCommand extends Command {

	public ResetElevatorCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		/// requires(Elevator.getInstance());
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotState.getInstance().setElevatorResetting(true);
		Elevator.getInstance().brakeOff();
		Elevator.getInstance().setElevatorOutput(Robot.bot.ElevatorResetMotorPower);
		SmartDashboard.getEntry("Elevator reset").forceSetBoolean(false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (RobotState.getInstance().getState() == RobotState.State.AUTON) {
			return Elevator.getInstance().hasReset();
		} else {
			return (Elevator.getInstance().hasReset() || (Math
					.abs(OI.getOperatorLeftYValue()) > Robot.bot.elevatorJoystickThreshold)); /* elevatorStopped() */

		}
	}
	// sydney is really cool :))))
	// r-ted is not :((((

	// Called once after isFinished returns true
	protected void end() {
		Elevator.getInstance().setElevatorOutput(0.0);
		Elevator.getInstance().brakeOn();
		SmartDashboard.getEntry("Elevator reset").forceSetBoolean(true);
		RobotState.getInstance().setElevatorResetting(false);
		RobotState.getInstance().setElevatorLocation(RobotState.ElevatorLocation.INTAKE);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}