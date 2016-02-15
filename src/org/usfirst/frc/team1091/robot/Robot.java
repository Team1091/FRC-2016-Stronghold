
package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends SampleRobot {

	CameraServer server;
	// ReEnable soon
	// SerialPort.Port port = new Port(3);
	// SerialPort sonic = new SerialPort(19200, port);

	RobotDrive myRobot;
	AutoDrive autoDrive;
	final Joystick xbox; // xbox controller

	// final Joystick cyborg; // cyborg controller
	// // Joysticks below used together in unison, separated in code for
	// usability
	// final Joystick leftJoy; // left joystick controller
	// final Joystick rightJoy; // right joystick controller

	final int rightBumperButtenNumber = 6;

	final Victor lShoot;
	final Victor rShoot;
	final Victor lift;
	DigitalInput limit;

	Encoder lEncod; // 20 per rotation
	Encoder rEncod; // 20 per rotation
	Encoder liftEncod;

	Solenoid in;
	Solenoid out;

	final double deadZone = 0.02;

	AngleCalc calc = new AngleCalc();
	double angle;
	double RPM;

	final DriverStation.Alliance color;

	boolean xboxBut1, xboxBut2, xboxBut3;
	/**
	 * 1: Right Joystick Arcade Drive 2: Left Joystick Arcade Drive 3: Both
	 * Joystick Tank Drive 4: Trigger Tank Drive
	 */
	boolean cyborgBut1, cyborgBut2, cyborgBut3;
	/**
	 * 1: Joystick drive / lever shooter 2: Joystick drive / hat shooter 3:
	 * Joystick drive / scroll shooter
	 */
	boolean joyBut1, joyBut2, joyBut3, joyBut4;

	/**
	 * 1: Dual stick drive / hat shooter 2: Dual stick drive / side button
	 * shooter 3: Left stick drive / Right shooter 4: Right stick drive / Left
	 * shooter
	 **/

	// SerialPort serialPort;

	public Robot() {

		// serialPort = new SerialPort(19200, Port.kUSB);

		color = DriverStation.getInstance().getAlliance();
		System.out.print(color.name());
		myRobot = new RobotDrive(0, 1, 2, 3);
		myRobot.setExpiration(0.1);
		autoDrive = new AutoDrive(myRobot);
		lShoot = new Victor(5);
		rShoot = new Victor(6);
		lift = new Victor(4);

		limit = new DigitalInput(0); // normally closed

		lEncod = new Encoder(1, 2, true);
		rEncod = new Encoder(3, 4);
		liftEncod = new Encoder(5, 6);

		in = new Solenoid(0);
		out = new Solenoid(1);

		xbox = new Joystick(0);

		xboxBut1 = true;

		server = CameraServer.getInstance();
		server.setQuality(100);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");
	}

	// MAIN AUTONOMOUS METHOD

	public void autonomous() {
		autoDrive.autoChoose();
	}

	// SMOOTHS CONTROLS
	private double driveConvert(double x) {
		if (x < -1)
			return -1;
		else if (x > 1)
			return 1;
		else {
			if (x < 0) {
				x *= 1;
				return (-1 * x * x * x * (x * (x * 6 - 15) + 10));
			} else {
				x *= -1;
				return (x * x * x * (x * (x * 6 - 15) + 10));
			}

		}
	}

	// SETS CONTROLLER SENSITIVITY
	private double setSensitivity(double x) {
		x *= -1;
		x += 1;
		x /= 2; // sets range of (0,1)

		final double low = 0.3; // minimum sensitivity
		final double high = 0.7; // maximum sensitivity

		final double constant = 1 / (high - low); // constant for use in
													// equation
		return (((x * x) / constant) + low) * -1;
	}

	// GET DISTANCE TO WALL
	private double getDistance() {
		double dist = 0; // replace with ultrasonic code
		return dist;
	}

	long lLastEncoderVal = 0;
	long rLastEncoderVal = 0;
	long lastTime = System.currentTimeMillis();

	// UPDATE CONTROLS AND SENSORS
	private void refresh() throws InterruptedException {
		long currentTime = System.currentTimeMillis();
		long changeTime = currentTime - lastTime;

		long lCurrentEncod = Math.abs(lEncod.get());
		double lChangeEncod = (double) lCurrentEncod - lLastEncoderVal;

		long rCurrentEncod = Math.abs(rEncod.get());
		double rChangeEncod = (double) rCurrentEncod - rLastEncoderVal;

		// was What is bellow
		// long lCurrentRPM = (long) (((lChangeEncod / 20) / (changeTime)) *
		// 60000);
		// long rCurrentRPM = (long) (((rChangeEncod / 20) / (changeTime)) *
		// 60000);

		long lCurrentRPM = (long) ((lChangeEncod / 20) / (changeTime));
		long rCurrentRPM = (long) ((rChangeEncod / 20) / (changeTime));

		// Figure out what we are getting from serial
		// byte[] data = serialPort.read(serialPort.getBytesReceived());
		//
		// for (int i = 0; i < data.length; i++) {
		// System.out.print((int) data[i]);
		// }
		//

		// Pnumatics of the sole stuff
		if (xbox.getRawButton(rightBumperButtenNumber)) {
			in.set(true);
			out.set(false);
			// put a sleep here
		} else {
			in.set(false);
			out.set(true);
		}
		// System.out.println("lRPM: " + lCurrentRPM);
		// System.out.println("rRPM: " + rCurrentRPM);
		System.out.println("liftEncod: " + liftEncod.get());
		// System.out.println("Limit: " + limit.get());

		calc.setAngle(getDistance()); // check dist and perform calculations
		angle = calc.getAngle(); // update angle val from dist
		RPM = calc.getRPM(); // update RPM val from dist

		xboxDrive(); // For xbox controls
		xboxShoot(); // For xbox shooting

		lastTime = currentTime;
		lLastEncoderVal = lEncod.get();
		rLastEncoderVal = rEncod.get();
	}

	// MAIN WHILE LOOP
	public void operatorControl() {//
		myRobot.setSafetyEnabled(true);

		while (isOperatorControl() && isEnabled()) {
			try {
				refresh();
			} catch (InterruptedException e) {
				e.printStackTrace();
			} // Update controls and sensors
			Timer.delay(0.001); // wait for a motor update time
		}
	}

	// XBOX SHOOTING CONTROLS

	int deg45 = -30;

	// 177.5 in theary

	private void xboxShoot() {
		double yAxis = xbox.getRawAxis(5);
		double trigger = xbox.getRawAxis(2);
		boolean homeButten = DriverStation.getInstance().getStickButton(0, (byte) 8);
		boolean degSet45 = DriverStation.getInstance().getStickButton(0, (byte) 4);

		if (!(Math.abs(trigger) < deadZone)) {
			lShoot.set(-trigger);
			rShoot.set(trigger);
			// launch solenoid
		} else if (xbox.getRawButton(5) == true) {
			double var = 0.5;
			lShoot.set(var);
			rShoot.set(-var);
		} else {
			lShoot.set(0);
			rShoot.set(0);
		}

//		if(limit.get()){
//			// We are at the top, so reset it
//			liftEncod.reset();
//			System.out.println("resetting encoder");
//		}
		
		if (homeButten) {
			System.out.println("Homies");
			if (!limit.get()) {
				lift.set(-.5);
			} else {
				//lift.set(Math.max(yAxis, 0));
				lift.set(0);
				System.out.println("0 motor");
			}

		} else if (degSet45) {
			System.out.println("45 deg");
			int liftDiffToTar = (liftEncod.get() - deg45);
			if (liftDiffToTar < 0) {
				lift.set(-.5);
			} else {
				lift.set(.5);
			}

		}
	}

	// XBOX DRIVING CONTROLS
	private void xboxDrive() {
		if (xboxBut1) // Right Joy Arcade Drive
		{
			double yAxis = driveConvert(xbox.getRawAxis(1)) * .50;
			double xAxis = driveConvert(xbox.getRawAxis(0)) * .50;
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone))
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
	}
}