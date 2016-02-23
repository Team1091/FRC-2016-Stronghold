package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;

public class AutoDrive {

	private RobotDrive myRobot;
	private Victor lifter;
	private DigitalInput limit;
	private boolean first;
	
	private long startTime;
	private long currentTime;
	private long deltaTime;

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
		deltaTime = currentTime- startTime;

			// autoPortcullis();
			// autoChevaldefrise();
			// autoRampards();
			// autoMoat();
			// autoDrawbridge();
			// autoSallyport();
			autoRockwall();
			// autoRoughterrain();
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

	private void autoRampards() {
		liftBack();	
		if(deltaTime < 2000)
		forward(0.4);
		if(deltaTime < 2500)
		{
			left(0.1);
			right(0.6);
		}
		else
		stop();
	}

	private void autoMoat() {
		liftBack();
		if(deltaTime < 1500)
		forward(0.3);
		else if(deltaTime < 3000)
		stop();
		else if(deltaTime < 4000)
		forward(0.5);
		else if(deltaTime < 5000)
		forward(0.4);
		else if(deltaTime < 5750)
		{
			right(0.3);
			left(0.1);
		}
		else
		stop();

	}

	private void autoDrawbridge() { //Need attachment
		
	}

	private void autoSallyport() { //Need attachment
		
	}

	
	private void autoRockwall() { // RUN BACKWARDSSSSSSSSSSSSSSSSSSS
		liftBack();
		if(deltaTime<1500)
			forward(-0.3);
		else if(deltaTime<2500)
			forward(-0.5);
		else if(deltaTime<2700)
			forward(-0.2);
		else if(deltaTime<3500)
			forward(-0.5);
		else
			stop();

	}

	private void autoRoughterrain() {
		liftBack();
		if(deltaTime < 3000)
		forward(0.3);
		else
		stop();
	}

	private void autoLowbar() { // This will still need to be tested USE WITH CARE
		if (currentTime < 5000)
			forward(.5);

	}

}
