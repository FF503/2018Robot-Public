package org.usfirst.frc.team503.robot.commands;

import java.util.Arrays;

import org.usfirst.frc.team503.robot.Robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorLaserThread implements Runnable {
	double dist;
	double vel;
	double lastDist;
	double dist_cm;
	double timeStart;
	double timeCur;
	int check = 0;
	int strength;
	int[] data = new int[9];
	int header = 0x59;
	private SerialPort s;
	boolean laserFound;

	public ElevatorLaserThread() {
		s = new SerialPort(115200, Robot.bot.laserPort, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
	}

	public void run() {
		try {
			if (s.getBytesReceived() > 8) {
				// System.out.println("pass recieve check");
				if (s.read(1)[0] == header) {
					data[0] = header;
					// System.out.println("pass check 1");
					if (s.read(1)[0] == header) {
						data[1] = header;
						// System.out.println("pass check 2");
						for (int i = 2; i < 9; i++) {
							data[i] = Byte.toUnsignedInt((s.read(1)[0]));
							// System.out.println("data collected");
						}
					}
				}
				s.reset();
			}

			for (int i = 0; i < 8; i++) {
				check += data[i];
			}
			check = check & 0xff;

			dist_cm = data[2] + data[3] * 256;
			dist = dist_cm / 2.54;
			strength = data[4] + data[5] * 256;
			// System.out.println(Arrays.toString(data));
			/*
			 * timeCur = Timer.getFPGATimestamp(); vel = (dist-lastDist)/(0.05); timeStart =
			 * timeCur; //System.out.println("Dist: " + dist); lastDist = dist;
			 * //System.out.println("Last Dist: " + lastDist);
			 */ laserFound = true;
		} catch (Exception e) {
			laserFound = false;
			e.printStackTrace();
		}
		// putDashboardData();
	}

	public synchronized double getLaserHeight() {
		if (laserFound) {
			return dist;// + Robot.bot.carriageLaserOffset;
		}
		return 0.0;
	}

	public synchronized double getLaserVelocity() {
		if (laserFound) {
			return vel;
		}
		return 0.0;
	}

	public synchronized void putDashboardData() {
		/*
		 * SmartDashboard.putNumber("data 3", data[2]);
		 * SmartDashboard.putNumber("data 4", data[3]);
		 * SmartDashboard.putNumber("data 5", data[4]);
		 * SmartDashboard.putNumber("data 6", data[5]);
		 */
		/*
		 * SmartDashboard.putNumber("Laser check", check);
		 * SmartDashboard.putNumber("Laser checkActual", data[8]);
		 */
		double timeStarted = Timer.getFPGATimestamp();
		if (laserFound) {
			SmartDashboard.putBoolean("Laser found", true);
			SmartDashboard.putNumber("Laser strength", strength);
			// if(dist != 0) {
			// System.out.println(String.format("%.2f", dist) + " cm: " +
			// String.format("%.2f", dist_cm));
			double distance = getLaserHeight();
			double time = Timer.getFPGATimestamp();
			double velocity = (distance - lastDist) / (time - timeStart);
			SmartDashboard.putNumber("Laser distance", distance);
			SmartDashboard.putNumber("Laser velocity", velocity);
			SmartDashboard.putNumber("Laser last distance", lastDist);
			SmartDashboard.putNumber("Laser distance difference", distance - lastDist);
			lastDist = distance;
			timeStart = time;

			// SmartDashboard.putNumber("Laser distance cm", dist_cm);
			// }
		} else {
			SmartDashboard.putBoolean("Laser found", false);
			SmartDashboard.putNumber("Laser distance", 0.0);
			SmartDashboard.putNumber("Laser velocity", 0.0);
		}
		double timeEnded = Timer.getFPGATimestamp();
		SmartDashboard.putNumber("Method time", Timer.getFPGATimestamp() - timeStarted);
	}

}
