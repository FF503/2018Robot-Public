package org.usfirst.frc.team503.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

public class RobotState extends Subsystem {

	// Put methods for controlling this subsystem
	private boolean driveReversed;
	private boolean climberIsRunning;
	private String gameData;
	private State robotState;
	private ElevatorLocation elevatorState;
	private double gyroAngle;
	private boolean intakeRunning;
	private boolean reverseIntakeRunning;
	private boolean intakeRaised;
	private RobotDriveMode driveMode;
	private boolean carriageRunning;
	private boolean carriageReverse;
	private boolean brakeEngaged;
	private boolean elevatorResetting;
	private boolean elevatorProfileDone;
	private boolean driveProfileDone;

	public RobotState() {
		driveReversed = false;
		climberIsRunning = false;
		robotState = State.DISABLED;
		gyroAngle = 0.0;
		gameData = "";
		intakeRunning = false;
		reverseIntakeRunning = false;
		intakeRaised = true;
		carriageRunning = false;
		carriageReverse = false;
		driveMode = RobotDriveMode.LOW;
		elevatorState = ElevatorLocation.INTAKE;
		brakeEngaged = true;
		elevatorResetting = false;
		driveProfileDone = true;
		elevatorProfileDone = true;
		/*
		 * //LL, LR, RL, RR centerToSwitch = new ArrayList<CommandGroup>();
		 * centerToSwitch.add(new CtoLSwitch()); centerToSwitch.add(new CtoLSwitch());
		 * centerToSwitch.add(new CtoRSwitch()); centerToSwitch.add(new CtoRSwitch());
		 * 
		 * leftToSwitchOnly = new ArrayList<CommandGroup>(); leftToSwitchOnly.add(new
		 * LtoLSwitch()); leftToSwitchOnly.add(new LtoLSwitch());
		 * leftToSwitchOnly.add(new LtoLScale()); leftToSwitchOnly.add(new LtoRScale());
		 * 
		 * leftToSwitchThenScale = new ArrayList<CommandGroup>();
		 * leftToSwitchThenScale.add(new LSwitchToLScale());
		 */
	}

	private static RobotState instance = new RobotState();

	public static RobotState getInstance() {
		return instance;
	}

	public String getGameData() {
		int retries = 1000;
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		while (gameData.length() < 2 && retries > 0) {
			retries--;
			try {
				Thread.sleep(5);
			} catch (InterruptedException ie) {
				// Just ignore the interrupted exception
			}
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		if (gameData.length() > 0) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		} else if (retries == 0) {
			gameData = "503";
		}
		return gameData.substring(0, 2);
	}

	public void resetGameData() {
		gameData = "";
	}

	public enum State {
		DISABLED, AUTON, TELEOP, TEST, ENABLED;
	}

	public enum AllianceColor {
		RED, BLUE;
	}

	public void setCarriageForward(boolean running) {
		carriageRunning = running;
	}

	public boolean getCarriageForward() {
		return carriageRunning;
	}

	public boolean getElevatorResetting() {
		return elevatorResetting;
	}

	public void setElevatorResetting(boolean reset) {
		elevatorResetting = reset;
	}

	public enum ElevatorLocation {
		INTAKE, SWITCH, SCALE, CLIMB
	}

	public ElevatorLocation getElevatorLocation() {
		return elevatorState;
	}

	public void setElevatorLocation(ElevatorLocation location) {
		elevatorState = location;
	}

	public boolean getBrakeEngaged() {
		return brakeEngaged;
	}

	public void setBrakeEngaged(boolean engaged) {
		brakeEngaged = engaged;
	}

	public void setCarriageReverse(boolean reverse) {
		carriageReverse = reverse;
	}

	public boolean getCarriageReverse() {
		return carriageReverse;
	}

	public void setReverseIntakeRunning(boolean running) {
		reverseIntakeRunning = running;
	}

	public boolean getReverseIntakeRunning() {
		return reverseIntakeRunning;
	}

	public void setIntakeRunning(boolean running) {
		intakeRunning = running;
	}

	public boolean getIntakeRunning() {
		return intakeRunning;
	}

	public void setGyroAngle(double angle) {
		gyroAngle = angle;
	}

	public double getGyroAngle() {
		return gyroAngle;
	}

	public State getState() {
		return robotState;
	}

	public void setState(State state) {
		robotState = state;
	}

	public void setClimberRunning(boolean status) {
		climberIsRunning = status;
	}

	public boolean getClimberRunning() {
		return climberIsRunning;
	}

	public enum RobotDriveMode {
		LOW, HIGH, CLIMB;
	}

	public RobotDriveMode getDriveMode() {
		return driveMode;
	}

	public void setDriveMode(RobotDriveMode mode) {
		driveMode = mode;

	}

	public boolean getDriveReversed() {
		return driveReversed;
	}

	public void toggleDriveReversed() {
		driveReversed = !driveReversed;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void setIntakeRaised(boolean raised) {
		intakeRaised = raised;
	}

	public boolean getIntakeRaised() {
		return intakeRaised;
	}

	public boolean getElevatorProfileDone() {
		return elevatorProfileDone;
	}

	public void setElevatorProfileDone(boolean done) {
		elevatorProfileDone = done;
	}

	public boolean getDriveProfileDone() {
		return driveProfileDone;
	}

	public void setDriveProfileDone(boolean done) {
		driveProfileDone = done;
	}
}
