package org.usfirst.frc.team1091.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

public class ShooterLift implements Runnable {

	private Encoder liftEncoder;
	private Joystick xbox;
	private Victor lift;
	private Victor lShoot;
	private Victor rShoot;
	private DigitalInput limit;
	private boolean isDisabled;

	// Time to config all of the ABXY Stuffs "Yay"
	private final int A = 0; // This all is an estimation
	private final int B = 150;
	private final int X = 130;
	private final int Y = 26;

	private final double maxAnglularVelocity = 30; // Max ticks per second
	private final int fudgeFactor = 6; // Size of that ramp. Smaller is more
										// accurate, larger reduces oscillations
	private double targetAngle = 0; // The eventual destination in ticks
	private double currentAngle = 0; //
	private long lastTime = System.currentTimeMillis();

	public ShooterLift(Encoder liftEncoder, Joystick joy, Victor lift, Victor lShoot, Victor rShoot,
			DigitalInput limit) {
		this.liftEncoder = liftEncoder;
		this.xbox = joy;
		this.lift = lift;
		this.limit = limit;
		this.lShoot = lShoot;
		this.rShoot = rShoot;
	}

	private void setTarget(double angle) {
		targetAngle = angle;
	}

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

		currentAngle = Math.max(0, currentAngle);

	}

	public boolean auto = false;
	public int autoTarget = 0;

	@Override
	public void run() {

		try {
			while (!Thread.interrupted() && !isDisabled) {
				double liftPower = 0;
				if (!auto) {
					boolean isStartButtenPushed = xbox.getRawButton(8);
					boolean isYButtonPushed = xbox.getRawButton(4);
					boolean isBButtenPushed = xbox.getRawButton(2);
					boolean isAButtenPushed = xbox.getRawButton(1);
					boolean isXButtenPushed = xbox.getRawButton(3);
					
					if (isYButtonPushed) {
						setTarget(Y);
					} else if (isBButtenPushed) {
						setTarget(B);
						lShoot.set(.5);
						rShoot.set(-.5);
					} else if (isXButtenPushed) {
						setTarget(X);
					} else if (isAButtenPushed) {
						setTarget(A);
					} else if (isStartButtenPushed) {
						setTarget(-9000000);
					}
				} else {

					setTarget(autoTarget);
					
				}

			

				liftPower = update();

				if (limit.get()) {
					// We are at the top, so reset it and don't go negative any
					// more
					reset();
					liftPower = Math.max(0, liftPower);
				}

				System.out.println("liftPower: " + liftPower);
				System.out.println("encoder: " + liftEncoder.get());
				lift.set(-liftPower);

				Thread.sleep(10);
			}
		} catch (InterruptedException e) {
			// Interrupted when disabled called, drop out of here
			e.printStackTrace(); 
			return;
		}
	}

	public void disable(Thread thread) {
		isDisabled = true;
		// thread.interrupt();
		System.out.println("");
		System.out.println("<<ROBOT DISABLED -- THREADS ENDED>>");
		System.out.println("");
	}

	public void enable(Thread thread) {
		isDisabled = false;
		thread.start();
	}
}
