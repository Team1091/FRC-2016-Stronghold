package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
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

	final int rightBumperButtonNumber = 6;

	final Victor lShoot;
	final Victor rShoot;
	final Victor lift;
	DigitalInput limit;

	Encoder lEncod; // 20 per rotation
	Encoder rEncod; // 20 per rotation
	Encoder liftEncod;

	boolean first;

	Solenoid in;
	Solenoid out;

	final double deadZone = 0.02;

	AngleCalc calc = new AngleCalc();
	double angle;
	double RPM;

	final DriverStation.Alliance color;

	// SerialPort serialPort;

	public Robot() {

		// serialPort = new SerialPort(19200, Port.kUSB);

		color = DriverStation.getInstance().getAlliance();
		System.out.print(color.name());
		myRobot = new RobotDrive(0, 1, 2, 3);
		myRobot.setExpiration(0.1);
		autoDrive = new AutoDrive(myRobot, lift, limit);
		lShoot = new Victor(5);
		rShoot = new Victor(6);
		lift = new Victor(4);

		limit = new DigitalInput(0); // normally open

		lEncod = new Encoder(1, 2, true);
		rEncod = new Encoder(3, 4);
		liftEncod = new Encoder(5, 6);
		shooterLift = new ShooterLift(liftEncod);

		in = new Solenoid(0);
		out = new Solenoid(1);

		xbox = new Joystick(0);


		server = CameraServer.getInstance();
		server.setQuality(100);
		// the camera name (ex "cam0") can be found through the roboRIO web
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

	// GET DISTANCE TO WALL
	private double getDistance() {
		double dist = 0; // replace with distince code
		return dist;
	}

	long lLastEncoderVal = 0;
	long rLastEncoderVal = 0;
	long lastTime = System.currentTimeMillis();

	// UPDATE CONTROLS AND SENSORS
	private void refresh() throws InterruptedException {
		long currentTime = System.currentTimeMillis();
		// long changeTime = currentTime - lastTime;

		// long lCurrentEncod = Math.abs(lEncod.get());
		// double lChangeEncod = (double) lCurrentEncod - lLastEncoderVal;

		// long rCurrentEncod = Math.abs(rEncod.get());
		// double rChangeEncod = (double) rCurrentEncod - rLastEncoderVal;

		// was What is bellow
		// long lCurrentRPM = (long) (((lChangeEncod / 20) / (changeTime)) *
		// 60000);
		// long rCurrentRPM = (long) (((rChangeEncod / 20) / (changeTime)) *
		// 60000);

		// long lCurrentRPM = (long) ((lChangeEncod / 20) / (changeTime)) *
		// 60000;
		// long rCurrentRPM = (long) ((rChangeEncod / 20) / (changeTime)) *
		// 60000;

		// System.out.println("lRPM: " + lCurrentRPM);
		// System.out.println("rRPM: " + rCurrentRPM);

		// Figure out what we are getting from serial
		// byte[] data = serialPort.read(serialPort.getBytesReceived());
		//
		// for (int i = 0; i < data.length; i++) {
		// System.out.print((int) data[i]);
		// }
		//

		// System.out.println("lRPM: " + lCurrentRPM);
		// System.out.println("rRPM: " + rCurrentRPM);
		// System.out.println("liftEncod: " + liftEncod.get());
		// System.out.println("Limit: " + limit.get());

		calc.setAngle(getDistance()); // check dist and perform calculations
		angle = calc.getAngle(); // update angle val from dist
		RPM = calc.getRPM(); // update RPM val from dist

		xboxDrive(); // For xbox controls
		xboxShoot(); // For xbox shooting
		// xboxAutoShoot(angle, RPM);
		xboxAutoShoot(Math.PI / 4, 0);

		lastTime = currentTime;
		lLastEncoderVal = Math.abs(lEncod.get());
		rLastEncoderVal = Math.abs(rEncod.get());
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

	private void kick() {
		if (xbox.getRawButton(rightBumperButtonNumber)) {
			out.set(false);
			in.set(true);
		} else {
			out.set(true);
			in.set(false);
		}
	}

	// XBOX SHOOTING CONTROLS

	private final int deg0 = 130; // This is an estimation
	private final int deg45 = 59;
	private final int deg60 = 10;
	private final int deg90 = 0;

	public int moveToDeg;
	ShooterLift shooterLift;

	private void xboxShoot() {
		double yAxis = xbox.getRawAxis(5);
		double trigger = xbox.getRawAxis(2);
		boolean isHomeButtonPushed = xbox.getRawButton(8);
		boolean isYButtonPushed = DriverStation.getInstance().getStickButton(0, (byte) 4);
		boolean isBButtonPushed = DriverStation.getInstance().getStickButton(0, (byte) 2);
		boolean isBackPushed = DriverStation.getInstance().getStickButton(0, (byte) 7);

		// Firing Wheels
		if (!(Math.abs(trigger) < deadZone)) {
			lShoot.set(-trigger);
			rShoot.set(trigger);
		} else if (xbox.getRawButton(5) == true) {
			double var = 0.5;
			lShoot.set(var);
			rShoot.set(-var);
		} else {
			lShoot.set(0);
			rShoot.set(0);
		}

		kick(); // hits ball so hard

		System.out.println("Lift: " + liftEncod.get());

		double liftPower = 0;
		if (isHomeButtonPushed) {
			liftPower = -0.8;
			System.out.print("HOMING");
		} else {
		
			if (isYButtonPushed) { // Check if the Y button is pressed
				shooterLift.setTarget(deg60);
			} else {
				// Freeform lifting - this assumes y goes from 0 to 1,
				// and you want to be at deg0 at y=0
				// and deg90 at y=1
				shooterLift.setTarget(lerp(deg90, deg0, ((double) (yAxis + 1)) / 2.0));
			}

			liftPower = shooterLift.update();
		}
		
		if (limit.get()) {
			// We are at the top, so reset it and don't go negative any more			
			shooterLift.reset();
			liftPower = Math.max(0, liftPower);
		}
		System.out.print("POWER: " + liftPower);
		lift.set(-liftPower);
	}

	// https://en.wikipedia.org/wiki/Linear_interpolation
	private double lerp(double v0, double v1, double t) {
		return v0 + t * (v1 - v0);
	}

	private void liftUp() {
		lift.set(-0.6 * Math.cos(Math.toRadians(liftEncod.get() * 0.755)));
		System.out.println("Lifting");
	}

	private double getAngle() {
		double angle = Math.toRadians(90 - (-liftEncod.get() * (18 / 71)));
		System.out.println("Angle: " + angle);
		return angle;
	}

	// PREREQ: Home shooter prior to auto-shooting
	private void xboxAutoShoot(double angle, double RPM) {
		if (xbox.getRawButton(1) == true) {
			while (Math.abs(getAngle() - angle) < (Math.PI / 71) && xbox.getRawButton(8) == false) {
				if (getAngle() > angle)
					liftUp();
				else
					lift.set(0.6);
			}
			lift.set(0);

		}
	}

	// XBOX DRIVING CONTROLS
	private void xboxDrive() {
			double yAxis = xbox.getRawAxis(1) * -.85;
			double xAxis = xbox.getRawAxis(0) * -.85;
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone))
				myRobot.arcadeDrive(yAxis, xAxis, true);
	}
}