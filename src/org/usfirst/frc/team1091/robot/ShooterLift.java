package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

public class ShooterLift implements Runnable {

	Encoder liftEncoder;
	Joystick xbox;
	Victor lift;
	DigitalInput limit;
	boolean isDisabled;

	private final int deg0 = 130; // This is an estimation
	private final int deg45 = 59;
	private final int aimAng = 59;
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

	private void setTarget(double angle) {
		targetAngle = angle;
	}

	/**
	 * Do you even lift, bro?
	 */
	private double update() {

		double liftPower = 0;

		long nowTime = System.currentTimeMillis();
		double deltaTime = (double) (nowTime - lastTime) / 1000.0;
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


		return liftPower;

	}

	private void reset() {
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
		while (!Thread.interrupted() && !isDisabled) {
			boolean isHomeButtonPushed = xbox.getRawButton(8);
			boolean isYButtonPushed = xbox.getRawButton(4);
			double yAxis = xbox.getRawAxis(5);


			double liftPower = 0;
			if (isHomeButtonPushed) {
				liftPower = -0.3;
			} else {

				if (isYButtonPushed) { // Check if the Y button is pressed
					setTarget(aimAng);
				} 

				liftPower = update();
			}

			if (limit.get()) {
				// We are at the top, so reset it and don't go negative any more
				reset();
				liftPower = Math.max(0, liftPower);
			}
			lift.set(-liftPower);
			
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	public void disabled()
	{
		isDisabled = true;
	}

}
