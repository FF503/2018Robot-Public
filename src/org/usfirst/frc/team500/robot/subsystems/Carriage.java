package org.usfirst.frc.team500.robot.subsystems;

import org.usfirst.frc.team500.robot.Robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Carriage extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private Spark carriageBottom;
	// private Spark carriageTop;
	private int irLen = 10;
	private int filterCalls = 0;
	private double[] irVals = new double[irLen];
	private Servo cubeBlock1;
	private Servo cubeBlock2;

	AnalogInput carriageIR;

	public Carriage() {
		carriageBottom = new Spark(Robot.bot.carriageBottomID);
		// carriageTop = new Spark(Robot.bot.carriageTopID);

		carriageIR = new AnalogInput(Robot.bot.carriageIR_ID);
		if (!Robot.bot.getName().equals("ProgrammingBot")) {
			cubeBlock1 = new Servo(Robot.bot.servoPort1);
			cubeBlock2 = new Servo(Robot.bot.servoPort2);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	private static Carriage instance = new Carriage();

	public static Carriage getInstance() {
		return instance;
	}

	public void setCarriageMotorOutput(double power) {
		carriageBottom.set(power * Robot.bot.carriageReversed);
		// carriageTop.set(power * Robot.bot.carriageReversed);

	}

	public void shootCubeForwardFast() {
		// setCubeBlock(false);
		setCarriageMotorOutput(1.0);
	}

	public void shootCubeBackwardFast() {
		// setCubeBlock(false);
		setCarriageMotorOutput(-1.0);
	}

	public void shootCubeForwardSlow() {
		// setCubeBlock(false);
		setCarriageMotorOutput(0.6);
	}

	public void shootCubeBackwardSlow() {
		// setCubeBlock(false);
		setCarriageMotorOutput(-0.6);
	}

	public void stopCarriage() {
		setCarriageMotorOutput(0.0);
		// setCubeBlock(true);
	}

	public void setCubeBlock(boolean state) {
		if (state) {
			setServoPos(Robot.bot.blockEngagePos);
		} else {
			setServoPos(Robot.bot.blockDisengagePos);
		}
	}

	private void setServoPos(double position) {
		cubeBlock1.set(position);
		cubeBlock2.set(1 - position);
	}

	public double getServoPos() {
		return cubeBlock1.get();
	}

	public double getCarriageIRFiltered() {

		if (filterCalls < irLen) {
			irVals[(filterCalls) % irLen] = getCarriageIR();
			filterCalls++;

			return 0.503;
		}
		irVals[(filterCalls) % irLen] = getCarriageIR();
		double sd, sum, average;
		sum = 0;

		for (int i = 0; i < irVals.length; i++) {
			sum += irVals[i];
		}

		average = sum / irVals.length;
		sd = 0;

		for (int i = 0; i < irVals.length; i++) {
			sd = sd + Math.pow(irVals[i] - average, 2);
		}
		sd = Math.sqrt(sd / (irVals.length - 1));

		double sumFiltered = 0.0;
		double goodCount = 0.0;
		double goodAvg = 0.0;
		for (int i = 0; i < irVals.length; i++) {
			if (Math.abs(irVals[i] - average) <= sd) {
				sumFiltered += irVals[i];
				// System.out.println(sumFiltered);
				goodCount++;
			} else {
				// System.out.println("threw out" + i);
			}
		}
		// System.out.println(goodCount);
		if (goodCount != 0) {
			goodAvg = sumFiltered / goodCount;
		}

		filterCalls++;
		// System.out.println(goodAvg);
		return goodAvg;
	}

	public double getCarriageIR() {
		return carriageIR.getVoltage();
	}

	public boolean carriageHasCube() {
		return getCarriageIRFiltered() > Robot.bot.carriageIRThreshold;
	}

	public void sendDashboardData() {
		SmartDashboard.putNumber("Carriage Power Bottom", carriageBottom.get());
		// SmartDashboard.putNumber("Carriage Power Top", carriageTop.get());
		SmartDashboard.putNumber("Carriage IR Sensor Voltage", getCarriageIR());
		SmartDashboard.putBoolean("Carriage has cube", carriageHasCube());
		SmartDashboard.putNumber("Filtered IR Value", getCarriageIRFiltered());
	}
}
