
package org.usfirst.frc.team1091.robot;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.AnalogGyro;
public class Robot extends SampleRobot {

	CameraServer server;

	RobotDrive myRobot;
	Joystick xbox; // xbox controller
	Joystick cyborg; // cyborg controller

	// Joysticks below used together in unison, separated in code for usability
	Joystick leftJoy; // left joystick controller
	Joystick rightJoy; // right joystick controller
	
	Encoder encoder;

	final double deadZone = 0.01;
	
	AngleCalc calc = new AngleCalc();
	double angle;
	double RPM;
	
	final DriverStation.Alliance color;

	boolean xboxBut1, xboxBut2, xboxBut3;
	/**
	 * 1: Right Joystick Arcade Drive
	 * 2: Left Joystick Arcade Drive
	 * 3: Both Joystick Tank Drive
	 * 4: Trigger Tank Drive 
	 */
	boolean cyborgBut1, cyborgBut2, cyborgBut3;
	/**
	 * 1: Joystick drive / lever shooter
	 * 2: Joystick drive / hat shooter
	 * 3: Joystick drive / scroll shooter
	 */
	boolean joyBut1, joyBut2, joyBut3, joyBut4;
	/**
	 * 1: Dual stick drive / hat shooter
	 * 2: Dual stick drive / side button shooter
	 * 3: Left stick drive / Right shooter
	 * 4: Right stick drive / Left shooter
	**/
	public Robot() {
		color = DriverStation.getInstance().getAlliance();
		System.out.print(color.name());
		myRobot = new RobotDrive(0,1,2,3);
		myRobot.setExpiration(0.1);
		xbox = new Joystick(0);
		cyborg = new Joystick(1);
		leftJoy = new Joystick(2);
		rightJoy = new Joystick(3);
		
		//xboxBut2 = true;
		//cyborgBut1 = true;
		joyBut4 = true;
		
		
		server = CameraServer.getInstance();
		server.setQuality(100);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");
	}

	//MAIN AUTONOMOUS METHOD 
	public void autonomous() {
		myRobot.setSafetyEnabled(false);
		// autoPortcullis(-1);
		// autoChevaldefrise(-1);
		// autoRampards(-1);
		// autoMoat(-1);
		// autoDrawbridge(-1);
		// autoSallyport(-1);
		// autoRockwall(-1);
		// autoRoughterrain(-1);
		// autoLowbar();
		myRobot.drive(0.0, 0.0);
	}

	private void autoPortcullis(int pos) {
		autoCenter(pos);
	}

	private void autoChevaldefrise(int pos) {
		autoCenter(pos);
	}

	private void autoRampards(int pos) {
		autoCenter(pos);
	}

	private void autoMoat(int pos) {
		autoCenter(pos);
	}

	private void autoDrawbridge(int pos) {
		autoCenter(pos);
	}

	private void autoSallyport(int pos) {
		autoCenter(pos);
	}

	private void autoRockwall(int pos) {
		autoCenter(pos);
	}

	private void autoRoughterrain(int pos) {
		autoCenter(pos);
	}

	private void autoLowbar() {
		autoCenter(5);
	}

	//AUTOMOUSLY CENTERS ROBOT
	private void autoCenter(int pos) {

	}
	
	//SMOOTHS CONTROLS
	private double driveConvert(double x) 
	{
		if(x < -1)
			return -1;
		else if(x > 1)
			return 1;
		else
		{
			if(x < 0)
			{
				x *= 1;
				return (-1 * x * x * x * (x * (x * 6 -15) + 10)) ;
			} else
			{
				x *= -1;
				return (x * x * x * (x * (x * 6 -15) + 10)) ;
			}
				
		}
	}
	
	//SETS CONTROLLER SENSITIVITY
	private double setSensitivity(double x)
	{
		x -= 1;
		return ((x/-8) + 0.5) * -1; //Was .25 chaged for xboxremote
	}
	
	//GET DISTANCE TO WALL
	private double getDistance()
	{
		double dist = 0; //replace with ultrasonic code
		return dist;
	}
	
	//UPDATE CONTROLS AND SENSORS
	private void refresh()
	{
		calc.setAngle(getDistance()); //check dist and perform calculations
		angle = calc.getAngle(); //update angle val from dist
		RPM = calc.getRPM(); //update RPM val from dist
		
		xboxDrive(); // For xbox controls
		cyborgDrive(); // For cyclops controls
		joyDrive(); // For dual joystick controls
	}
	
	//MAIN WHILE LOOP
	public void operatorControl() {//
		myRobot.setSafetyEnabled(true);
		
		while (isOperatorControl() && isEnabled()) {
			refresh(); //Update controls and sensors
			Timer.delay(0.001); // wait for a motor update time
		}
	}
	
	//XBOX CONTROLS
	private void xboxDrive() {
		if (xboxBut1) // Right Joy Arcade Drive
		{
			double yAxis = xbox.getRawAxis(5) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = xbox.getRawAxis(4) * setSensitivity(cyborg.getRawAxis(4));
			System.out.println("Y: " + yAxis);
			System.out.print("X: " + xAxis);
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
	//CYBORG CONTROLS
	private void cyborgDrive() {
		if (cyborgBut1) // Joystick drive / lever shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = driveConvert(cyborg.getRawAxis(3) * -1) * setSensitivity(cyborg.getRawAxis(4));;
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (cyborgBut2) // Joystick drive / hat shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = driveConvert((cyborg.getRawAxis(3)) * -1) * setSensitivity(cyborg.getRawAxis(4));
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
		if (cyborgBut3) // Joystick drive / scroll shooter
		{
			double yAxis = driveConvert(cyborg.getRawAxis(1) * -1) * setSensitivity(cyborg.getRawAxis(4));
			double xAxis = driveConvert((cyborg.getRawAxis(3)) * -1) * setSensitivity(cyborg.getRawAxis(4));
			if (!(Math.abs(cyborg.getRawAxis(1)) < deadZone) || !(Math.abs(cyborg.getRawAxis(3)) < deadZone)) // deadzone
				myRobot.arcadeDrive(yAxis, xAxis, true);
		}
	}
	//DUAL JOYSTICK CONROLS
	private void joyDrive() {
		if (joyBut1) // Dual stick drive / hat shooter
		{
			double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) * setSensitivity(rightJoy.getRawAxis(3));
			double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) * setSensitivity(rightJoy.getRawAxis(3));
			if (!(Math.abs(rightY) < deadZone) || !(Math.abs(leftY) < deadZone)) // deadZone
				myRobot.tankDrive(leftY, rightY, true);
		}
		if (joyBut2) // Dual stick drive / side button shooter
		{
			double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) * setSensitivity(rightJoy.getRawAxis(3));
			double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) * setSensitivity(rightJoy.getRawAxis(3));
			if (!(Math.abs(rightY) < deadZone) || !(Math.abs(leftY) < deadZone))
				myRobot.tankDrive(leftY, rightY, true);
		}
		if (joyBut3) // Left stick drive / Right shooter
		{
			double leftY = driveConvert(leftJoy.getRawAxis(1) * -1) * setSensitivity(leftJoy.getRawAxis(3));
			double leftX = driveConvert(leftJoy.getRawAxis(2)* -1) * setSensitivity(leftJoy.getRawAxis(3)); //twist
			if (!(Math.abs(leftX) < deadZone) || !(Math.abs(leftY) < deadZone))
				myRobot.arcadeDrive(leftY, leftX, true);
		}
		if (joyBut4) // Right stick drive / Left shooter
		{
			double rightY = driveConvert(rightJoy.getRawAxis(1) * -1) * setSensitivity(rightJoy.getRawAxis(3));
			double rightX = driveConvert(rightJoy.getRawAxis(2) * -1) * setSensitivity(rightJoy.getRawAxis(3)); //twist
			if (!(Math.abs(rightY) < deadZone) || !(Math.abs(rightX) < deadZone))
				myRobot.arcadeDrive(rightY, rightX, true);
		}
	}

	
	public void test() {}
}
