package org.usfirst.frc.team503.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
@Deprecated
public class IntakeStallWatcher extends Command {

    public IntakeStallWatcher() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
	protected void initialize() {

	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	/*if(Intake.getInstance().getStalled()) {
    		Intake.getInstance().stopIntake();
    		RobotState.getInstance().setIntakeRunning(false);
    	}*/
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
    }
}
