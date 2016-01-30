package org.usfirst.frc.team1091.robot;

public class test {
	public static void main(String[] args)
	{
		AngleCalc testy = new AngleCalc(2);
		System.out.println("AngleRad: " + testy.getAngle());
		System.out.println("AngleDeg: " + testy.getAngleDeg());
		System.out.println("RPM: " + testy.getRPM());
		System.out.println("Time: " + testy.getTimeAir());
	}

}
