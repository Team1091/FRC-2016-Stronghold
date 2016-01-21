package org.usfirst.frc.team1091.robot;

public class AngleCalc {
	
	private static final double height = 2.46; //height of tower
	private static final double gravity = -9.8; //force of gravity (m/s/s)
	private static final double diam = 3.875; //diameter of shooting wheels (in.)
	private static final double circ = (diam * Math.PI * 0.0254); //circumference of shooting wheel (m.)
	private static final double  vY = Math.sqrt(-2*gravity*height); //initial vertical velocity of ball (m/s)
	private static final double time = Math.sqrt((height*-2)/gravity);  //time to reach apex
	private double angle, rpm;
	
	public AngleCalc(double dist) //creates angle + rpm values
	{
		double vX = dist/time; //initial horizontal velocity of ball (m/s)
		double vI = Math.sqrt((vY * vY) + (vX * vX)); //initial velocity of ball (m/s)
		angle = Math.tanh(vY/vX);
		rpm = (vI/circ) * 60; 
	}
	
	public double getAngle()
	{
		return angle;
	}
	
	public double getRPM()
	{
		return rpm;
	}

}
