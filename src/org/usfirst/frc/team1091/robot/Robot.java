
package org.usfirst.frc.team1091.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot{
    RobotDrive myRobot;
    Joystick stick;

    public Robot() {
        myRobot = new RobotDrive(0, 1);
        myRobot.setExpiration(0.1);
        stick = new Joystick(0);
    }

  
    
    public void autonomous() {
        myRobot.setSafetyEnabled(false);
//        autoPortcullis(-1);
//        autoChevaldefrise(-1);
//        autoRampards(-1);
//        autoMoat(-1);
//        autoDrawbridge(-1);
//        autoSallyport(-1);
//        autoRockwall(-1);
//        autoRoughterrain(-1);
//        autoLowbar();
        myRobot.drive(0.0, 0.0);
    }

    private void autoPortcullis(int pos)
    {
    	autoCenter(pos);
    }
    private void autoChevaldefrise(int pos)
    {
    	autoCenter(pos);
    }
    private void autoRampards(int pos)
    {
    	autoCenter(pos);
    }
    private void autoMoat(int pos)
    {
    	autoCenter(pos);
    }
    private void autoDrawbridge(int pos)
    {
    	autoCenter(pos);
    }
    private void autoSallyport(int pos)
    {
    	autoCenter(pos);
    }
    private void autoRockwall(int pos)
    {
    	autoCenter(pos);
    }
    private void autoRoughterrain(int pos)
    {
    	autoCenter(pos);
    }
    private void autoLowbar()
    {
    	autoCenter(5);
    }
    private void autoCenter(int pos)
    {
    	
    }
    
    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
            myRobot.arcadeDrive(stick); // drive with arcade style (use right stick)
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
