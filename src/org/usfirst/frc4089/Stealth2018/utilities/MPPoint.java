package org.usfirst.frc4089.Stealth2018.utilities;

public class MPPoint {
	double ticksL;
	double ticksR;
	double heading;
	double timeTaken;
	
	public MPPoint(double TicksL, double TicksR, double Heading, double time)
	{
		ticksL = TicksL;
		ticksR = TicksR;
		heading = Heading;
		timeTaken = time;
	}
	
	@Override
	public String toString()
	{
		return "L:" + ticksL + " R:" + ticksR + " H:" + heading + " T: " + timeTaken; 
	}
}
