package org.usfirst.frc.team503.robot;

import org.usfirst.frc.team503.robot.commands.AutomaticIntakeCube;
import org.usfirst.frc.team503.robot.commands.CarriageBackwards;
import org.usfirst.frc.team503.robot.commands.CarriageBackwardsGroup;
import org.usfirst.frc.team503.robot.commands.CarriageForwards;
import org.usfirst.frc.team503.robot.commands.CarriageForwardsGroup;
import org.usfirst.frc.team503.robot.commands.ClimbCommand;
import org.usfirst.frc.team503.robot.commands.CloseIndexer;
import org.usfirst.frc.team503.robot.commands.DeployLiftsCommand;
import org.usfirst.frc.team503.robot.commands.EjectCube;
import org.usfirst.frc.team503.robot.commands.FollowCube;
import org.usfirst.frc.team503.robot.commands.ResetElevatorCommand;
import org.usfirst.frc.team503.robot.commands.ReverseDriveCommand;
import org.usfirst.frc.team503.robot.commands.ShiftDriveGear;
import org.usfirst.frc.team503.robot.commands.TeleOpFollowCube;
import org.usfirst.frc.team503.robot.commands.ToggleIntakePosition;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	private static XboxController driverJoystick = new XboxController(0);
	private static XboxController operatorJoystick = new XboxController(1);
	
	private static JoystickButton intakeCube = new JoystickButton(driverJoystick, 1);
	private static JoystickButton ejectCube = new JoystickButton(driverJoystick, 2);
	private static JoystickButton toggleIntakePosition = new JoystickButton(driverJoystick, 3);
	private static JoystickButton climb = new JoystickButton(driverJoystick, 4);
	private static JoystickButton shiftToLow = new JoystickButton(driverJoystick, 5);
	private static JoystickButton shiftToHigh= new JoystickButton(driverJoystick, 6);
	private static JoystickButton intakeOverride = new JoystickButton(driverJoystick, 7);
	private static JoystickButton ptoShift = new JoystickButton(driverJoystick, 8);
	

	private static JoystickButton carriageBackward = new JoystickButton(operatorJoystick, 1);
	private static JoystickButton resetElevator = new JoystickButton(operatorJoystick, 3);
	private static JoystickButton carriageForward = new JoystickButton(operatorJoystick, 4);
	private static JoystickButton elevatorToIntake = new JoystickButton(operatorJoystick, 5);	
	private static JoystickButton elevatorToClimb = new JoystickButton(operatorJoystick, 6);
	private static JoystickButton overrideElevator = new JoystickButton(operatorJoystick, 7);
	private static JoystickButton deployLifts = new JoystickButton(operatorJoystick, 8);
	private static JoystickButton closeIndexer = new JoystickButton(operatorJoystick, 2);
	
	
	public static Button elevatorToSwitch = new Button(){
		@Override
		public boolean get(){
			return operatorJoystick.getRawAxis(2) >= 0.9;
		}
	};
	
	public static Button carriageForwardSlow = new Button(){	
		@Override
		public boolean get(){
			return operatorJoystick.getPOV(0) == 0 || operatorJoystick.getPOV(0) == 45 || operatorJoystick.getPOV(0) == 315;
		}
		
	};
	
	public static Button carriageBackwardSlow = new Button(){	
		@Override
		public boolean get(){
			return ((operatorJoystick.getPOV(0) == 180) || (operatorJoystick.getPOV(0) == 225) || operatorJoystick.getPOV(0) == 135);
		}
		
		
	};
	public static  Button elevatorToScale = new Button(){
		@Override 
		public boolean get(){
			return operatorJoystick.getRawAxis(3) >= 0.9;
		}
	};
	
	public static Button reverseDrive = new Button() {
		@Override
		public boolean get() {
			return driverJoystick.getRawAxis(2) >= 0.9;			
		}
	};
	
	public static Button followCube = new Button() {
		@Override
		public boolean get() {
			return driverJoystick.getRawAxis(3) >= 0.9;
		}
	};
	
	public static void initialize() {
		followCube.whileHeld(new TeleOpFollowCube());
		shiftToLow.whenPressed(new ShiftDriveGear(RobotState.RobotDriveMode.LOW));
		shiftToHigh.whenPressed(new ShiftDriveGear(RobotState.RobotDriveMode.HIGH));
		//climb.whileHeld(new ClimbCommand());
		reverseDrive.whenPressed(new ReverseDriveCommand());
		intakeCube.toggleWhenPressed(new AutomaticIntakeCube());
		ejectCube.whileHeld(new EjectCube()); 
		toggleIntakePosition.whenPressed(new ToggleIntakePosition());
		followCube.whenPressed(new FollowCube());
		carriageForward.whileHeld(new CarriageForwardsGroup(false));
		carriageBackward.whileHeld(new CarriageBackwardsGroup(false));
		carriageForwardSlow.whileHeld(new CarriageForwards(true));
		carriageBackwardSlow.whileHeld(new CarriageBackwards(true));
		resetElevator.whenPressed(new ResetElevatorCommand());
		ptoShift.whenPressed(new ShiftDriveGear(RobotState.RobotDriveMode.CLIMB));
		deployLifts.whenPressed(new DeployLiftsCommand());
		closeIndexer.whenPressed(new CloseIndexer());
		
		
	}
	
	public static boolean getIntakeButton(){
		return intakeCube.get(); 
	}
	
	public static boolean getEjectButton(){
		return ejectCube.get();
	}

	public static double getDriverLeftYValue(){
		return driverJoystick.getRawAxis(1);
	}
	public static double getDriverLeftXValue(){
		return driverJoystick.getRawAxis(0);
	}
	public static double getDriverRightYValue(){
		return driverJoystick.getRawAxis(5);
	}
	public static double getDriverRightXValue(){
		return driverJoystick.getRawAxis(4);
	}
	public static boolean getDriverLeftTrigger() {
		return driverJoystick.getRawAxis(2) >= 0.9;
	}
	public static boolean getDriverRightTrigger() {
		return driverJoystick.getRawAxis(3) >= 0.9;
	}
	
	public static double getOperatorLeftYValue() {
		return operatorJoystick.getRawAxis(1);
	}
	public static boolean getOperatorLeftTrigger() {
		return operatorJoystick.getRawAxis(2) >= 0.9;
	}
	public static boolean getOperatorRightTrigger() {
		return operatorJoystick.getRawAxis(3) >= 0.9;
	}
	
	public static boolean overrideElevator() {
		return overrideElevator.get();
	}
	
	public static void setDriveRumble(double rumble) {
		driverJoystick.setRumble(GenericHID.RumbleType.kLeftRumble, rumble);
		driverJoystick.setRumble(GenericHID.RumbleType.kRightRumble, rumble);
	}

	public static boolean getVisionButton(){             
		return followCube.get();
	}
	
	public static boolean getIntakeOverride() {
		return intakeOverride.get();
	}
}
