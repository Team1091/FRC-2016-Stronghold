
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
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Robot extends SampleRobot {

	CameraServer server;
	// SerialPort.Port port = new Port(3);
	// SerialPort sonic = new SerialPort(19200, port);

	RobotDrive myRobot;

	final Joystick xbox; // xbox controller
	final Joystick cyborg; // cyborg controller
	// Joysticks below used together in unison, separated in code for usability
	final Joystick leftJoy; // left joystick controller
	final Joystick rightJoy; // right joystick controller

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

	//SerialPort serialPort;

	public Robot() {

	//	serialPort = new SerialPort(19200, Port.kUSB);

		color = DriverStation.getInstance().getAlliance();
		System.out.print(color.name());
		myRobot = new RobotDrive(0, 1, 2, 3);
		myRobot.setExpiration(0.1);

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
		cyborg = new Joystick(1);
		leftJoy = new Joystick(2);
		rightJoy = new Joystick(3);

		xboxBut1 = true;
		// cyborgBut1 = true;
		// joyBut1 = true;

		server = CameraServer.getInstance();
		server.setQuality(100);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");
	}

	// MAIN AUTONOMOUS METHOD
	public void autonomous() {
		myRobot.setSafetyEnabled(false);
		try {
			// autoPortcullis(-1);
			// autoChevaldefrise(-1);
			// autoRampards(-1);
			// autoMoat(-1);
			// autoDrawbridge(-1);
			// autoSallyport(-1);
			autoRockwall(-1);
			// autoRoughterrain(-1);
			// autoLowbar();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		stop();
	}

	private void left(double num) {
		myRobot.setLeftRightMotorOutputs(num, 0);
	}

	private void right(double num) {
		myRobot.setLeftRightMotorOutputs(0, num);
	}

	private void forward(double num) {
		myRobot.setLeftRightMotorOutputs(num, num);
	}

	private void stop() {
		myRobot.stopMotor();
	}

	private void autoPortcullis(int pos) {
		autoCenter(pos);
	}

	private void autoChevaldefrise(int pos) {
		autoCenter(pos);
	}

	private void autoRampards(int pos) throws InterruptedException {
		forward(0.4);
		Thread.sleep(2000);
		left(0.1);
		right(0.6);
		Thread.sleep(500);
		autoCenter(pos);
	}

	private void autoMoat(int pos) throws InterruptedException {
		forward(0.3);
		Thread.sleep(1500);
		stop();
		Thread.sleep(1000);
		forward(0.5);
		Thread.sleep(1000);
		stop();
		forward(0.4);
		Thread.sleep(750);
		right(0.3);
		left(0.1);
		Thread.sleep(250);
		stop();
		autoCenter(pos);
	}

	private void autoDrawbridge(int pos) {
		autoCenter(pos);
	}

	private void autoSallyport(int pos) {
		autoCenter(pos);
	}

	private void autoRockwall(int pos) throws InterruptedException { // run
																		// backwards
		forward(-0.3);
		Thread.sleep(1500);
		forward(-0.5);
		Thread.sleep(1000);
		forward(-0.2);
		Thread.sleep(200);
		forward(-0.5);
		Thread.sleep(800);
		autoCenter(pos);
	}

	private void autoRoughterrain(int pos) throws InterruptedException {
		forward(0.3);
		Thread.sleep(3000);
		autoCenter(pos);
	}

	private void autoLowbar() {
		autoCenter(5);
	}

	// AUTOMOUSLY CENTERS ROBOT
	private void autoCenter(int pos) {

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
	private void refresh() {
		long currentTime = System.currentTimeMillis();
		long changeTime = currentTime - lastTime;

		long lCurrentEncod = Math.abs(lEncod.get());
		double lChangeEncod = (double) lCurrentEncod - lLastEncoderVal;

		long rCurrentEncod = Math.abs(rEncod.get());
		double rChangeEncod = (double) rCurrentEncod - rLastEncoderVal;

		long lCurrentRPM = (long) (((lChangeEncod / 20) / (changeTime)) * 60000);
		long rCurrentRPM = (long) (((rChangeEncod / 20) / (changeTime)) * 60000);

		// Figure out what we are getting from serial
		// byte[] data = serialPort.read(serialPort.getBytesReceived());
		//
		// for (int i = 0; i < data.length; i++) {
		// System.out.print((int) data[i]);
		// }
		//
		
		//Pnumatics of the sole stuff
		if (xbox.getRawButton(rightBumperButtenNumber)) {
			in.set(true);
			out.set(false);
		} else {
			in.set(false);
			out.set(true);
		}
		System.out.println("lRPM: " + lCurrentRPM);
		System.out.println("rRPM: " + rCurrentRPM);
		System.out.println("liftEncod: " + liftEncod.get());
		System.out.println("Limit: " + limit.get());

		calc.setAngle(getDistance()); // check dist and perform calculations
		angle = calc.getAngle(); // update angle val from dist
		RPM = calc.getRPM(); // update RPM val from dist

		xboxDrive(); // For xbox controls
		xboxShoot(); // For xbox shooting
		cyborgDrive(); // For cyclops controls
		// cyborgShoot(); //For cyclops shooting
		// joyDrive(); // For dual joystick controls
		// joyShoot(); //For dual joystick shooting

		lastTime = currentTime;
		lLastEncoderVal = lEncod.get();
		rLastEncoderVal = rEncod.get();
	}

	// MAIN WHILE LOOP
	public void operatorControl() {//
		myRobot.setSafetyEnabled(true);

		while (isOperatorControl() && isEnabled()) {
			refresh(); // Update controls and sensors
			Timer.delay(0.001); // wait for a motor update time
		}
	}

	// XBOX SHOOTING CONTROLS
	private void xboxShoot() {
		if (xboxBut1) // Right Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(5); //was 1
			double trigger = xbox.getRawAxis(2);

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
			if (!(Math.abs(yAxis) < deadZone) && limit.get())
				lift.set(yAxis * 0.75);
			else if (!(Math.abs(yAxis) < deadZone) && yAxis > 0)
				lift.set(yAxis * 0.5);
			else
				lift.set(-0.4);

		}
		if (xboxBut2) // Left Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(5);
			double trigger = xbox.getRawAxis(3);

			if (!(Math.abs(trigger) < deadZone)) {
				lShoot.set(-trigger);
				rShoot.set(trigger);
			} else if (xbox.getRawButton(6) == true) {
				double var = 0.5;
				lShoot.set(var);
				rShoot.set(-var);
			} else {
				lShoot.set(0);
				rShoot.set(0);
			}
			if (!(Math.abs(yAxis) < deadZone) && limit.get())
				lift.set(yAxis * -0.50);
			else if (!(Math.abs(yAxis) < deadZone) && yAxis > 0)
				lift.set(yAxis * -0.50);
			else
				lift.set(0.4);
		}
		if (xboxBut3) // Joystick Tank Drive
		{
			double yAxis = xbox.getRawAxis(1);
			double lTrigger = xbox.getRawAxis(2);
			double rTrigger = xbox.getRawAxis(3);

			if (!(Math.abs(lTrigger) < deadZone)) {
				lShoot.set(-lTrigger);
				rShoot.set(lTrigger);
			} else if (!(Math.abs(rTrigger) < deadZone)) {
				lShoot.set(rTrigger);
				rShoot.set(-rTrigger);
			} else {
				lShoot.set(0);
				rShoot.set(0);
			}
			if (xbox.getRawButton(5) == true)
				lift.set(-0.50);
			else if (xbox.getRawButton(6) == true)
				lift.set(0.50);
			else
				lift.set(0.4);
		}
	}

	// XBOX DRIVING CONTROLS
	private void xboxDrive() {
		if (xboxBut1) // Right Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(1) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = xbox.getRawAxis(0) * setSensitivity(cyborg.getRawAxis(4));
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (xboxBut2) // Left Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(1) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = xbox.getRawAxis(0) * setSensitivity(cyborg.getRawAxis(4));
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (xboxBut3) // Joystick Tank Drive
		{
			double rightAxis = xbox.getRawAxis(5) * setSensitivity(cyborg.getRawAxis(4));
			double leftAxis = xbox.getRawAxis(1) * setSensitivity(cyborg.getRawAxis(4));
			if (!(Math.abs(rightAxis) < deadZone) || !(Math.abs(leftAxis) < deadZone)) // deadzone
				myRobot.tankDrive(leftAxis, rightAxis, true);
		}

	}

	// //CYBORG SHOOTING CONTROLS
	// private void cyborgShoot() {
	//
	// }
	// CYBORG DRIVING CONTROLS
	private void cyborgDrive() {
		if (cyborgBut1) // Joystick drive / lever shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			double xAxis = driveConvert(cyborg.getRawAxis(3) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (cyborgBut2) // Joystick drive / hat shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			double xAxis = driveConvert((cyborg.getRawAxis(3)) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (cyborgBut3) // Joystick drive / scroll shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			double xAxis = driveConvert((cyborg.getRawAxis(3)) * -1) * setSensitivity(cyborg.getRawAxis(4)) * 0.5;
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
	}
	// //DUAL JOYSTICK SHOOTING CONTROLS
	// private void joyShoot() {
	//
	// }
	// //DUAL JOYSTICK CONROLS
	// private void joyDrive() {
	// if (joyBut1) // Dual stick drive / hat shooter
	// {
	// double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5;
	// double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5;
	// if (!(Math.abs(rightY) < deadZone) || !(Math.abs(leftY) < deadZone)) //
	// deadZone
	// myRobot.tankDrive(leftY, rightY, true);
	// }
	// if (joyBut2) // Dual stick drive / side button shooter
	// {
	// double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5;
	// double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5;
	// if (!(Math.abs(rightY) < deadZone) || !(Math.abs(leftY) < deadZone))
	// myRobot.tankDrive(leftY, rightY, true);
	// }
	// if (joyBut3) // Left stick drive / Right shooter
	// {
	// double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) *
	// setSensitivity(leftJoy.getRawAxis(3)) * 0.5;
	// double leftX = driveConvert(leftJoy.getRawAxis(2)* -1) *
	// setSensitivity(leftJoy.getRawAxis(3)) * 0.5; //twist
	// if (!(Math.abs(leftX) < deadZone) || !(Math.abs(leftY) < deadZone))
	// myRobot.arcadeDrive(leftY, leftX, true);
	// }
	// if (joyBut4) // Right stick drive / Left shooter
	// {
	// double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5;
	// double rightX = driveConvert(rightJoy.getRawAxis(2) * -1) *
	// setSensitivity(rightJoy.getRawAxis(3)) * 0.5; //twist
	// if (!(Math.abs(rightY) < deadZone) || !(Math.abs(rightX) < deadZone))
	// myRobot.arcadeDrive(rightY, rightX, true);
	// }
	// }
	//
	//
	// public void test() {}
}
