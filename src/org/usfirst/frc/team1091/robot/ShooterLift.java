package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

public class ShooterLift implements Runnable{

	Encoder liftEncoder;
	Joystick xbox;
	Victor lift;
	DigitalInput limit;
	
	private final int deg0 = 130; // This is an estimation
	private final int deg45 = 59;
	private final int aimAng = 8;
	private final int deg90 = 0;

	private final double maxAnglularVelocity = 50; // Max ticks per second
	private final int fudgeFactor = 8; // Size of that ramp. Smaller is more
										// accurate, larger reduces oscillations
	private double targetAngle = 0; // The eventual destination in ticks
	private double currentAngle = 0; //
	private long lastTime = System.currentTimeMillis();

	public ShooterLift(Encoder liftEncoder, Joystick joy, Victor lift, DigitalInput limit) {
		this.liftEncoder = liftEncoder;
		this.xbox = joy;
		this.lift = lift;
		this.limit = limit;
	}

	public void setTarget(double angle) {
		targetAngle = angle;
	}

	/**
	 * Do you even lift, bro?
	 */
	public double update() {

		double liftPower = 0;

		long nowTime = System.currentTimeMillis();
		double deltaTime = (double) (nowTime-lastTime) / 1000.0;
		lastTime = nowTime;

		if (currentAngle < targetAngle) {
			currentAngle = Math.min(targetAngle, currentAngle + maxAnglularVelocity * deltaTime);
		} else {
			currentAngle = Math.max(targetAngle, currentAngle - maxAnglularVelocity * deltaTime);
		}

		int liftDiffToTar = ((int) currentAngle + liftEncoder.get());
		if (liftDiffToTar < -fudgeFactor) {
			liftPower = -1;
		} else if (liftDiffToTar > fudgeFactor) {
			liftPower = 1;
		} else {
			liftPower = (double) liftDiffToTar * (1.0 / (fudgeFactor * 2.0));
		}

		System.out.println("current: " + currentAngle + " target:" + targetAngle + " p: " + liftPower);

		return liftPower;

	}

	public void reset() {
		if (liftEncoder.get() != 0)
			liftEncoder.reset();

		currentAngle = 0;
		
	}

	// https://en.wikipedia.org/wiki/Linear_interpolation
		private double lerp(double v0, double v1, double t) {
			return v0 + t * (v1 - v0);
		}
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		boolean isHomeButtonPushed = xbox.getRawButton(8);
		boolean isYButtonPushed = xbox.getRawButton(4);
		double yAxis = xbox.getRawAxis(5);
		
		System.out.println("Lift: " + liftEncoder.get());

		double liftPower = 0;
		if (isHomeButtonPushed) {
			liftPower = -0.8;
			System.out.print("HOMING");
		} else {
		
			if (isYButtonPushed) { // Check if the Y button is pressed
				setTarget(aimAng);
			} else {
				// Freeform lifting - this assumes y goes from 0 to 1,
				// and you want to be at deg0 at y=0
				// and deg90 at y=1
				setTarget(lerp(deg90, deg0, ((double) (yAxis + 1)) / 2.0));
			}

			liftPower = update();
		}
		
		if (limit.get()) {
			// We are at the top, so reset it and don't go negative any more			
			reset();
			liftPower = Math.max(0, liftPower);
		}
		System.out.print("POWER: " + liftPower);
		lift.set(-liftPower);
	}

}
