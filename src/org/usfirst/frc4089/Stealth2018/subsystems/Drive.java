// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.commands.*;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 *
 */
public class Drive extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	private final WPI_TalonSRX sRX1DriveLR = RobotMap.driveSRX1DriveLR;
	private final WPI_TalonSRX sRX2DriveLF = RobotMap.driveSRX2DriveLF;
	private final WPI_TalonSRX sRX3DriveRR = RobotMap.driveSRX3DriveRR;
	private final WPI_TalonSRX sRX4DriveRF = RobotMap.driveSRX4DriveRF;
    private final DifferentialDrive robotDrive41 = RobotMap.driveRobotDrive41;
    private final Encoder quadEncLeftDrive = RobotMap.driveQuadEncLeftDrive;
    private final Encoder quadEncRightDrive = RobotMap.driveQuadEncRightDrive;

    
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    
    private double mError = 0.0;
    private double mLastError = 0.0;
    private double mAcumError = 0.0;
 
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    	

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    	System.out.println("Drive init");
    	setDefaultCommand(new UserDrive());
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    /**
     * 
     * @param speed -1.0 (reverse) : 1.0 (forward) 
     * @param rotation -1.0 (counter clockwise) : 1.0 (clockwise)
     */
    public void arcDrive (double speed, double rotation) {
       	robotDrive41.arcadeDrive(speed, rotation);
    }
    
    public void drive (double leftSpeed, double rightSpeed, boolean squaredInputs) {
    	robotDrive41.tankDrive(leftSpeed, rightSpeed, squaredInputs);
    }
    
    @Override
    public void periodic() {
    }

    public void Drive(Joystick driveJoystick) {
    	double y = driveJoystick.getRawAxis(1);
    	double x = driveJoystick.getRawAxis(0)*-1;
    	
    	if(true != driveJoystick.getRawButton(4))
    	{
        	if(true == driveJoystick.getRawButton(1))
        	{
        		y = y/2.5;
        		x = x/2.8;
        	}
        	else
        	{
        		y = y/1.5;
        		x = x/1.8;
        	}
    	}
    	Drive(y,x);
    }	
    
    public void Drive(double y, double x) {
    	//TODO update this to nav class
    	double rate = RobotMap.navigationAnalogGyro1.getRate()/2;
    	
    	mError = (Constants.drivekAngleSetpoint - rate);
    	mAcumError += mError;
    	
    	if(mAcumError>Constants.drivekMaxAcum )
    	{
    		mAcumError = Constants.drivekMaxAcum;
    	}
    	
    	if(mAcumError<(Constants.drivekMaxAcum*-1))
    	{
    		mAcumError = Constants.drivekMaxAcum *-1;
    	}
    	
		double turningValue = (mError * Constants.drivekP)+(mAcumError * Constants.drivekI) + (mLastError * Constants.drivekD);
   	
		mLastError = mError;
		
    	if(Math.abs(x)>0.2)
    	{
    		robotDrive41.arcadeDrive(y,x);
    		mAcumError = 0;
    	}
    	else
    	{
			// Invert the direction of the turn if we are going backwards
			if(y<0)
			{
				turningValue *= -1;
			}
			
			robotDrive41.curvatureDrive(y, turningValue, false);
    	}
    	/*
    	RobotMap.netTable. .putDouble("gyroRate", rate);
    	RobotMap.netTable.putNumber("gyroAngle", angle);
    	RobotMap.netTable.putNumber("error", mError*kP);
    	RobotMap.netTable.putNumber("lastError", mLastError*kD);
    	RobotMap.netTable.putNumber("acumError", mAcumError*kI);    	
    	RobotMap.netTable.putNumber("speed", y);
    	RobotMap.netTable.putNumber("turning", turningValue);
    	RobotMap.netTable.putNumber("LeftEnc", leftMain.getEncPosition());
    	RobotMap.netTable.putNumber("RightEnc", rightMain.getEncPosition());
    	*/
    	
    	//RobotMap.driveBaseRobotDrive41.arcadeDrive(y,x);
    	//driveBaseLeftMain.set(0.5);
    	/*
    	System.out.print("Left Encoder:"+driveBaseLeftMain.getSelectedSensorPosition(0));
    	System.out.print(" Left Encoder:"+driveBaseLeftMain.getSelectedSensorVelocity(0));
    	System.out.print(" Right Encoder:"+driveBaseRightMain.getSelectedSensorPosition(0));
    	System.out.println(" Right Encoder:"+driveBaseRightMain.getSelectedSensorVelocity(0));
        */
     }
}

