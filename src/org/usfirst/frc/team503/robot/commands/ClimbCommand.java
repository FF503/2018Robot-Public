package org.usfirst.frc.team503.robot.commands;

import org.usfirst.frc.team503.robot.RobotState;
import org.usfirst.frc.team503.robot.subsystems.Drive;
import org.usfirst.frc.team503.robot.subsystems.Shift;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbCommand extends Command {
	Timer timer;
	Compressor c;

	public ClimbCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		requires(Drive.getInstance());
		timer = new Timer();
		c = new Compressor();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.CLIMB);
		timer.start();
		c.stop();
		Drive.getInstance().setMotorOutputs(1.0, 1.0);

		// System.out.println("init");
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
//    	if(timer.get() > 0.5) {
//        	Drive.getInstance().setMotorOutputs(0.0, 1.0);
//    	}
//    	else if(timer.get() > 1.0) {
//    		Drive.getInstance().setMotorOutputs(1.0, 0.0);
//    		timer.reset();
		Drive.getInstance().setMotorOutputs(1.0, 1.0);
		// System.out.println("execute");

//    	}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Drive.getInstance().setMotorOutputs(0.0, 0.0);
		// System.out.println("end");

		(new ArcadeDriveCommand()).start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
