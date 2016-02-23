package org.usfirst.frc.team1091.robot;

public class AngleCalc {

	private static final double height = 2.46; // height of tower
	private static final double gravity = -9.8; // force of gravity (m/s/s)
	private static final double diam = 3.875; // diameter of shooting wheels
												// (in.)
	// circumference of shooting wheels (m)
	
	private static final double circ = (diam * Math.PI * 0.0254); 
																
																	
	private static final double vY = Math.sqrt(-2 * gravity * height); // initial
																		// vertical
																		// velocity
																		// of
																		// ball
																		// (m/s)
	private static final double time = Math.sqrt((height * -2) / gravity); // time
																			// to
																			// reach
																			// apex
	private double angle, rpm;

	public AngleCalc(double dist) // creates angle + rpm values
	{
		double vX = dist / time; // initial horizontal velocity of ball (m/s)
		double vI = Math.sqrt((vY * vY) + (vX * vX)); // initial velocity of
														// ball (m/s)
		angle = Math.atan(vY / vX);
		rpm = (vI / circ) * 60;
	}

	public AngleCalc() {

	}

	public void setAngle(double dist) {
		double vX = dist / time; // initial horizontal velocity of ball (m/s)
		double vI = Math.sqrt((vY * vY) + (vX * vX)); // initial velocity of
														// ball (m/s)
		angle = Math.atan(vY / vX);
		rpm = (vI / circ) * 60;
	}

	public double getAngle() // returns angle in radians
	{
		return angle;
	}

	public double getAngleDeg() // returns angle in degrees
	{
		return Math.toDegrees(angle);
	}

	public double getRPM() // returns RPM to shoot
	{
		return rpm;
	}

	public double getTimeAir() // returns time ball is in air
	{
		return time;
	}

}
