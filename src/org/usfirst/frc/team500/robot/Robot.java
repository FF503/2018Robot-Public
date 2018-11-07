package org.usfirst.frc.team500.robot;

import org.usfirst.frc.team500.robot.commands.ArcadeDriveCommand;
import org.usfirst.frc.team500.robot.commands.TeleopElevatorCommand;
import org.usfirst.frc.team500.robot.subsystems.Carriage;
import org.usfirst.frc.team500.robot.subsystems.Drive;
import org.usfirst.frc.team500.robot.subsystems.Elevator;
import org.usfirst.frc.team500.robot.subsystems.Gyro;
import org.usfirst.frc.team500.robot.subsystems.Intake;
import org.usfirst.frc.team500.robot.subsystems.Shift;
import org.usfirst.frc.team500.robot.subsystems.VisionFollower;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// public static final ExampleSubsystem exampleSubsystem = new
	// ExampleSubsystem();
	public static RobotHardwareCompBot bot = null;
	public static NetworkTable universalGrapherTable;

	/*
	 * CvSink cvSink; CvSource outputStream;
	 * 
	 * Mat source = new Mat(); Mat output = new Mat();
	 */

	// private Compressor c;
	// Command autonomousCommand;
	// SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotState.getInstance().setState(RobotState.State.DISABLED);

		NetworkTableInstance.create();
		NetworkTableInstance.getDefault().deleteAllEntries();
		universalGrapherTable = NetworkTableInstance.getDefault().getTable("UniversalGrapher");
		// c = new Compressor(0);
		// c.start();

		/* Initialize a new bot object */
		bot = new RobotHardwareCompBot();
		// bot.initialize();
		// bot.logSmartDashboard();
		OI.initialize();

		Gyro.getInstance().resetGyro();
		Drive.getInstance().resetEncoders();
		Elevator.getInstance().resetEncoder();

		VisionFollower.getInstance().setLedMode(0);
		VisionFollower.getInstance().setLedMode(1);
		Elevator.getInstance().lockForks();
		// chooser.addDefault("Default Auto", new TankDriveCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		// SmartDashboard.putData("Auto mode", chooser);
		// s = new SerialPort(115200, SerialPort.Port.kOnboard, 8,
		// SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
//		s.setReadBufferSize(8);
//		s.setFlowControl(FlowControl.kXonXoff);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Elevator.getInstance().lockForks();
		RobotState.getInstance().setState(RobotState.State.DISABLED);
		Elevator.getInstance().brakeOn();
	}

	@Override
	public void disabledPeriodic() {
		// System.out.println("working");
		Drive.getInstance().sendDashboardData();
		Gyro.getInstance().sendDashboardData();
		Elevator.getInstance().sendDashboardData();
		Carriage.getInstance().sendDashboardData();
		Shift.getInstance().sendDashboardData();
		Intake.getInstance().sendDashboardData();
		VisionFollower.getInstance().sendDashboardData();
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		RobotState.getInstance().setState(RobotState.State.AUTON);
		RobotState.getInstance().resetGameData();
		Elevator.getInstance().lockForks();
		Carriage.getInstance().setCubeBlock(true);
		Drive.getInstance().setBrakeMode(true);
		// (new FollowCube()).start();
		// (new DriveForwardCommand(0.6, 3)).start();
		// Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.LOW);
		Drive.getInstance().resetEncoders();
		Gyro.getInstance().resetGyro();
		VisionFollower.getInstance().setLedMode(0);
		VisionFollower.getInstance().setLedMode(1);
		VisionFollower.getInstance().setPipeline(0);

		System.out.println("Time before auton start call:" + Timer.getFPGATimestamp());
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Elevator.getInstance().sendDashboardData();
		Drive.getInstance().sendDashboardData();
		Gyro.getInstance().sendDashboardData();
		Carriage.getInstance().sendDashboardData();
		Shift.getInstance().sendDashboardData();
		Intake.getInstance().sendDashboardData();
		VisionFollower.getInstance().sendDashboardData();
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Carriage.getInstance().setCubeBlock(true);
		Elevator.getInstance().lockForks();
		if (RobotState.getInstance().getDriveReversed()) {
			RobotState.getInstance().toggleDriveReversed();
		}
		Drive.getInstance().setBrakeMode(false);

		RobotState.getInstance().setState(RobotState.State.TELEOP);
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null)
		// autonomousCommand.cancel();
		Drive.getInstance().resetEncoders();
		Gyro.getInstance().resetGyro();
		VisionFollower.getInstance().setLedMode(0);
		VisionFollower.getInstance().setLedMode(1);
		VisionFollower.getInstance().setPipeline(0);

		Carriage.getInstance().setCarriageMotorOutput(0.0);
		RobotState.getInstance().setCarriageForward(false);
		RobotState.getInstance().setCarriageReverse(false);
		Intake.getInstance().stopIntake();
		RobotState.getInstance().setIntakeRunning(false);
		RobotState.getInstance().setReverseIntakeRunning(false);
		Shift.getInstance().updateDriveMode(RobotState.RobotDriveMode.LOW);

		/*
		 * cvSink = CameraServer.getInstance().getVideo(); outputStream =
		 * CameraServer.getInstance().putVideo("Blur", 640, 480);
		 */

		(new TeleopElevatorCommand()).start();
		(new ArcadeDriveCommand()).start();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Elevator.getInstance().calculateKinematics();
		Drive.getInstance().sendDashboardData();
		Gyro.getInstance().sendDashboardData();
		Elevator.getInstance().sendDashboardData();
		Carriage.getInstance().sendDashboardData();
		Shift.getInstance().sendDashboardData();
		Intake.getInstance().sendDashboardData();
		VisionFollower.getInstance().sendDashboardData();

		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
