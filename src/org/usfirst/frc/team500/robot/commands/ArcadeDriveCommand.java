package org.usfirst.frc.team500.robot.commands;

import org.usfirst.frc.team500.robot.OI;
import org.usfirst.frc.team500.robot.Robot;
import org.usfirst.frc.team500.robot.RobotState;
import org.usfirst.frc.team500.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArcadeDriveCommand extends Command {
	
    public ArcadeDriveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Drive.getInstance());
    	
    }

	// Called just before this Command runs the first time
    protected void initialize() {
   
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(RobotState.getInstance().getDriveReversed()) {
        	Drive.getInstance().arcadeDrive(OI.getDriverLeftYValue(), -OI.getDriverLeftXValue());
    	}
    	else {
        	Drive.getInstance().arcadeDrive(-OI.getDriverLeftYValue(), -OI.getDriverLeftXValue());
    	}
   
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !DriverStation.getInstance().isOperatorControl();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Drive.getInstance().tankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
