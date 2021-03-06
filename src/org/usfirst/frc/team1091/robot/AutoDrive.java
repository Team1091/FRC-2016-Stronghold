package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.RobotDrive;

public class AutoDrive {

	private RobotDrive myRobot;
	private ShooterLift shooterLift;
	private long startTime;
	private long currentTime;
	private long deltaTime;

	public AutoDrive(RobotDrive inputDrive, ShooterLift shooterLift) {
		myRobot = inputDrive;
		this.shooterLift = shooterLift;
	}

	public void autoChoose() {
		myRobot.setSafetyEnabled(false);
		startTime = System.currentTimeMillis();

		shooterLift.auto = true;

		System.out.println("");
		System.out.println("<<ROBOT AUTO -- FUNCTION ACTIVATED>>");
		System.out.println("");

		while (deltaTime < 15000) {
			currentTime = System.currentTimeMillis();
			deltaTime = currentTime - startTime;

			// autoPortcullis();
			// autoChevaldefrise();
			// autoRampards();
			// autoMoat();
			// autoDrawbridge();
			// autoSallyport();
			 autoRockwall();
			// autoRoughterrain();
			// autoLowbar();
		}

		shooterLift.auto = false;
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

	private void liftTo(int ang) {
		shooterLift.autoTarget = ang;
	}

	private void autoPortcullis(int pos) {

	}

	private void autoChevaldefrise(int pos) {

	}

	private void autoRampards() {
		liftTo(30);
		if (deltaTime < 2000)
			forward(0.4);
		if (deltaTime < 2500) {
			left(0.1);
			right(0.6);
		} else
			stop();
	}

	private void autoMoat() {
		liftTo(30);
		if (deltaTime < 1500)
			forward(0.3);
		else if (deltaTime < 3000)
			stop();
		else if (deltaTime < 4000)
			forward(0.5);
		else if (deltaTime < 5000)
			forward(0.4);
		else if (deltaTime < 5750) {
			right(0.3);
			left(0.1);
		} else
			stop();

	}

	private void autoDrawbridge() { // Need attachment

	}

	private void autoSallyport() { // Need attachment

	}

	private void autoRockwall() { // RUN BACKWARDSSSSSSSSSSSSSSSSSSS
		liftTo(30);
		if (deltaTime < 1500)
		forward(-.85);
	else if (deltaTime < 19000)		
		stop();

	}

	private void autoRoughterrain() {
		liftTo(30);
		if (deltaTime < 3000)
			forward(0.3);
		else
			stop();
	}

	private void autoLowbar() { // RUN BACKWARDS
		stop();

	}

}
