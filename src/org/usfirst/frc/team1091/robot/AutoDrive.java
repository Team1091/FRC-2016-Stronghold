package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.RobotDrive;
public class AutoDrive {

    RobotDrive myRobot;
    Victor lifter;
    
    public AutoDrive(RobotDrive inputDrive, Victor lifter){
        myRobot = inputDrive;
        this.lifter = lifter;
    }
    
    public void autoChoose(){
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

	public void stop() {
		myRobot.stopMotor();
	}

	private void autoPortcullis(int pos) {
		autoCenter(pos);
	}

	private void autoChevaldefrise(int pos) {
		autoCenter(pos);
	}
    
    private void liftBack() {
        lifter.set(0.4);
    }

	private void autoRampards(int pos) throws InterruptedException {
        liftBack();
        forward(0.4);
		Thread.sleep(2000);
		left(0.1);
		right(0.6);
		Thread.sleep(500);
		autoCenter(pos);
	}

	private void autoMoat(int pos) throws InterruptedException {
        liftBack();
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

	public void autoRockwall(int pos) throws InterruptedException { // run backwards
        liftBack();
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
        liftBack():
        forward(0.3);
		Thread.sleep(3000);
		autoCenter(pos);
	}

	private void autoLowbar(int pos) throws InterruptedException { // This will
																	// still
																	// need to
																	// be tested
																	// USE WITH
																	// CARE
		
        forward(.5);
		Thread.sleep(5000);
		autoCenter(pos);
	}

	// AUTOMOUSLY CENTERS ROBOT
	private void autoCenter(int pos) {

	}


}
