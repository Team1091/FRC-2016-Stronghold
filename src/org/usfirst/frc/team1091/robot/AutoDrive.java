package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;

public class AutoDrive {

	RobotDrive myRobot;
	Victor lifter;
	DigitalInput limit;
	boolean first;
	
	long startTime;
	long currentTime;

	public AutoDrive(RobotDrive inputDrive, Victor lifter, DigitalInput limit) {
		myRobot = inputDrive;
		this.lifter = lifter;
		this.limit = limit;
		first = true;
	}

	public void autoChoose() {
		myRobot.setSafetyEnabled(false);
		if (first) {
			startTime = System.currentTimeMillis();
			first = false;
		}

		currentTime = System.currentTimeMillis();
		long deltaTime = currentTime- startTime;

			// autoPortcullis(-1);
			// autoChevaldefrise(-1);
			// autoRampards(-1);
			// autoMoat(-1);
			// autoDrawbridge(-1);
			// autoSallyport(-1);
			autoRockwall(deltaTime);
			// autoRoughterrain(-1);
			// autoLowbar();

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

	public void stop() {
		myRobot.stopMotor();
	}

	private void autoPortcullis(int pos) {

	}

	private void autoChevaldefrise(int pos) {

	}

	private void liftBack() {
		if (!limit.get())
			lifter.set(0.4);
	}

//	private void autoRampards(int pos) {
//		liftBack();
//		forward(0.4);
//		Thread.sleep(2000);
//		left(0.1);
//		right(0.6);
//		Thread.sleep(500);
//
//	}
//
//	private void autoMoat(int pos) {
//		liftBack();
//		forward(0.3);
//		Thread.sleep(1500);
//		stop();
//		Thread.sleep(1000);
//		forward(0.5);
//		Thread.sleep(1000);
//		stop();
//		forward(0.4);
//		Thread.sleep(750);
//		right(0.3);
//		left(0.1);
//		Thread.sleep(250);
//		stop();
//
//	}

	private void autoDrawbridge(int pos) {

	}

	private void autoSallyport(int pos) {

	}

	private void autoRockwall(long time) { // run
																	// backwards
		liftBack();
		if(time<1500)
			forward(-0.3);
		else if(time<2500)
			forward(-0.5);
		else if(time<2700)
			forward(-0.2);
		else if(time<3500)
			forward(-0.5);
		else
			forward(0.0);

	}

	private void autoRoughterrain(int pos) {
		liftBack();
		forward(0.3);
//		Thread.sleep(3000);

	}

	private void autoLowbar(int pos) { // This will
																	// still
																	// need to
																	// be tested
																	// USE WITH
																	// CARE
		if (currentTime < 5000)
			forward(.5);

	}

}
