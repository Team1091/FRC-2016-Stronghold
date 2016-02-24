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

	private CameraServer server;

	// ReEnable soon
	// SerialPort.Port port = new Port(3);
	// SerialPort sonic = new SerialPort(19200, port);
	// SerialPort serialPort;

	private RobotDrive myRobot;
	private AutoDrive autoDrive;
	private final Joystick xbox; // xbox controller

	private final int rightBumperButtonNumber = 6;

	private final Victor lShoot, rShoot, lift;
	private DigitalInput limit;

	private Encoder lEncod, rEncod, liftEncod; // 20 per rotation
	
	private Solenoid in, out; //kicker

	final double deadZone = 0.02;

	AngleCalc calc = new AngleCalc();
	double angle, RPM;

	final DriverStation.Alliance color;

	private Thread thread;
	
	public Robot() {

		// serialPort = new SerialPort(19200, Port.kUSB);

		color = DriverStation.getInstance().getAlliance();
		System.out.print(color.name());
		myRobot = new RobotDrive(0, 1, 2, 3);
		myRobot.setExpiration(0.1);

		lShoot = new Victor(5);
		rShoot = new Victor(6);
		lift = new Victor(4);

		limit = new DigitalInput(0); // normally open
		xbox = new Joystick(0);
		liftEncod = new Encoder(5, 6);
		shooterLift = new ShooterLift(liftEncod, xbox, lift,lShoot,rShoot, limit);
		autoDrive = new AutoDrive(myRobot, shooterLift);
		
		
		
		lEncod = new Encoder(1, 2, true);
		rEncod = new Encoder(3, 4);
		
		

		in = new Solenoid(0);
		out = new Solenoid(1);

		server = CameraServer.getInstance();
		server.setQuality(100);
		// the camera name (ex "cam0") can be found through the roboRIO web
		// interface
		server.startAutomaticCapture("cam0");
		
		//thread = new Thread(shooterLift);
	}

	// MAIN AUTONOMOUS METHOD

	public void autonomous() {
		shooterLift.enable(thread);
		autoDrive.autoChoose();
		shooterLift.disable(thread);
	}



	long lLastEncoderVal = 0;
	long rLastEncoderVal = 0;
	long lastTime = System.currentTimeMillis();

	// UPDATE CONTROLS AND SENSORS
	private void refresh() throws InterruptedException {
		long currentTime = System.currentTimeMillis();
		xboxDrive(); // For xbox controls
		xboxShoot(); // For xbox shooting
		// xboxAutoShoot(angle, RPM);

		lastTime = currentTime;
		lLastEncoderVal = Math.abs(lEncod.get());
		rLastEncoderVal = Math.abs(rEncod.get());
	}

	// MAIN WHILE LOOP
	public void operatorControl() {//
		myRobot.setSafetyEnabled(true);
		
		shooterLift.enable(thread);
		shooterLift.auto = false;
		
		System.out.println("");
		System.out.println("<<ROBOT TELEOP -- CONTROLS ACTIVATED>>");
		System.out.println("");
		
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

	public int moveToDeg;
	ShooterLift shooterLift;
	
	//XBOX SHOOTING CONTROLS
	private void xboxShoot() {
		
		double trigger = xbox.getRawAxis(2);

		// Firing Wheels
		if (!(Math.abs(trigger) < deadZone)) {
			lShoot.set(-trigger * .90);
			rShoot.set(trigger * .90);
		} else if (xbox.getRawButton(5) == true) {
			double var = 0.5;
			lShoot.set(var);
			rShoot.set(-var);
		} else {
			lShoot.set(0);
			rShoot.set(0);
		}

		kick(); // hits ball so hard		
	}

	// XBOX DRIVING CONTROLS
	private void xboxDrive() {
			double yAxis = xbox.getRawAxis(1) * -.85;
			double xAxis = xbox.getRawAxis(0) * -.85;
			if (!(Math.abs(yAxis) < deadZone) || !(Math.abs(xAxis) < deadZone))
				myRobot.arcadeDrive(yAxis, xAxis, true);
	}
	
	@Override
	public void disabled()
	{
		shooterLift.disable(thread);	
		thread = new Thread(shooterLift);
	}
}