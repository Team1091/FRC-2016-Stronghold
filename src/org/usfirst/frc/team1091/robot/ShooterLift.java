package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;

public class ShooterLift {

	Encoder liftEncoder;

	private final double maxAnglularVelocity = 0.5; // Max ticks per second
	private final int fudgeFactor = 8; // Size of that ramp. Smaller is more
										// accurate, larger reduces oscillations
	private double targetAngle = 0; // The eventual destination in ticks
	private double currentAngle = 0; //
	private long lastTime = System.currentTimeMillis();

	public ShooterLift(Encoder liftEncoder) {
		this.liftEncoder = liftEncoder;
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
		double deltaTime = (double) (lastTime - nowTime) / 1000.0;
		lastTime = nowTime;

		if (currentAngle < targetAngle) {
			currentAngle = Math.max(targetAngle, currentAngle + maxAnglularVelocity * deltaTime);
		} else {
			currentAngle = Math.min(targetAngle, currentAngle - maxAnglularVelocity * deltaTime);
		}

		int liftDiffToTar = ((int) currentAngle + liftEncoder.get());
		if (liftDiffToTar < -fudgeFactor) {
			liftPower = -0.75;
		} else if (liftDiffToTar > fudgeFactor) {
			liftPower = 0.75;
		} else {
			liftPower = (double) liftDiffToTar * (1.0 / (fudgeFactor * 2));
		}

		System.out.println("current: " + currentAngle + " p: " + liftPower);

		return liftPower;

	}

}
