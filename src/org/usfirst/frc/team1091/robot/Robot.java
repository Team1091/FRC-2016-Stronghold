
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

		limit = new DigitalInput(0); // normally open

		lEncod = new Encoder(1, 2, true);
		rEncod = new Encoder(3, 4);
		liftEncod = new Encoder(5, 6);

		in = new Solenoid(0);
		out = new Solenoid(1);

		xbox = new Joystick(0);

		xboxBut1 = true;

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

		// System.out.println("lRPM: " + lCurrentRPM);
		// System.out.println("rRPM: " + rCurrentRPM);
		// System.out.println("liftEncod: " + liftEncod.get());
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

	public final int deg45 = 40;
	public final int deg60 = 55;
	public int moveToDeg;
	private void xboxShoot() {
		double yAxis = xbox.getRawAxis(5);
		double trigger = xbox.getRawAxis(2);
		boolean isHomeButtonPushed = DriverStation.getInstance().getStickButton(0, (byte) 8);
		boolean isYButtonPushed = DriverStation.getInstance().getStickButton(0, (byte) 4);
		boolean isBButtenPushed = DriverStation.getInstance().getStickButton(0, (byte) 2);
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

		// Pneumatic Kicker
		if (xbox.getRawButton(rightBumperButtonNumber)) {
			in.set(true);
			out.set(false);
			// TODO put a sleep here
		} else {
			in.set(false);
			out.set(true);
		}
		
	//LIFTER CODE
//	if (isBackPushed) {					// TODO I think that this is all wrong
//		if (yAxis > 0.3){
//			moveToDeg = (int) (moveToDeg + .5);
//		}		
//		if (yAxis < -0.3){					//THis may be correct however i'm at home and can't check it may just float back
//			moveToDeg = (int) (moveToDeg - .5);
//		}
//	}	

		System.out.println("Encoder" + liftEncod.get());
		double liftPower = (moveToDeg); //was yAxis

		if (isBButtenPushed){ //Check if the B butten is pressed
			int liftDiffToTar = (deg60 + liftEncod.get()); 
			if (liftDiffToTar > 4) {
				liftPower = -0.6;
				
			}else{
				liftPower = (float) liftDiffToTar * (0.5/4.0);
			}
		}
		
		if (isYButtonPushed) { // Check if the Y butten is pressed
			int liftDiffToTar = (deg45 + liftEncod.get());
			if (liftDiffToTar < -4) {
				liftPower = -0.6;
			} else if( liftDiffToTar > 4){
				liftPower = 0.6;
			}else{
				liftPower = (float)liftDiffToTar * (0.5/4.0);
			}
		}

		if (limit.get()) {
			// We are at the top, so reset it and dont go negative any more

			if (liftEncod.get() != 0)
				liftEncod.reset();
			liftPower = Math.max(0, liftPower);
		} else {
			if (isHomeButtonPushed) {
				liftPower = -0.6;
			}
		}
		lift.set(liftPower);
	}


	// XBOX DRIVING CONTROLS
	private void xboxDrive() {
		if (xboxBut1) // Right Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(1) * -.75;
			double xAxis = xbox.getRawAxis(0) * -.75;
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone))
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
	}
}