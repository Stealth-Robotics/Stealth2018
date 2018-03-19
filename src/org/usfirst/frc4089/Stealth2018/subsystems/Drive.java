//----------------------------------------------------------------------------
//
//  $Workfile: Drive.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                               Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.RobotMap;

import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc4089.Stealth2018.Constants;
import org.usfirst.frc4089.Stealth2018.commands.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc4089.Stealth2018.utilities.*;


//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: Drive
//
//Purpose:
//Handle getting us from here to there
//
//----------------------------------------------------------------------------
public class Drive extends Subsystem {
  //----------------------------------------------------------------------------
  //  Class Constants 
  //----------------------------------------------------------------------------
  static final double kPgain = 0.003; /* percent throttle per degree of error */
  static final double kDgain = 0.0002; /* percent throttle per angular velocity dps */
  static final double kMaxCorrectionRatio = 0.20; /* cap corrective turning throttle to 30 percent of forward throttle */
  static final double kSpeedGain = 0.05; // The ramp for the speed
  
  //----------------------------------------------------------------------------
  //  ENUM Constants 
  //----------------------------------------------------------------------------
  // The robot drivetrain's various states.
  public enum DriveControlState {
      OPEN_LOOP, // open loop voltage control
      VELOCITY_SETPOINT, // velocity PID control
      PATH_FOLLOWING, // used for autonomous driving
      AIM_TO_GOAL, // turn to face the boiler
      TURN_TO_HEADING, // turn in place
      DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
      DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
  }
  
  //----------------------------------------------------------------------------
  //  Class Attributes 
  //----------------------------------------------------------------------------
  double mTargetAngle = 0;
  boolean mSendJoystickCommands = true;   // send the joystick to the drive, we surpress this in auto
  double mCurrentAngle = 0.0;
  double mActualSpeed = 0.0;
  StopWatch mDisplay = new StopWatch(500);
  DriveControlState mState = DriveControlState.OPEN_LOOP;
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Set the system to path following
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------   
  public void SetAuto()
  {
    mState = DriveControlState.PATH_FOLLOWING;
  }
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Set the system to open drive
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------   
  public void SetTele()
  {
    mState = DriveControlState.OPEN_LOOP;
  }
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Init the command
  //
  // Notes:
  //     None.
  //--------------------------------------------------------------------   
    public void initDefaultCommand() {
    	setDefaultCommand(new UserDrive());
    }

    //--------------------------------------------------------------------
    // Purpose:
    //     Calls from every tick
    //
    // Notes:
    //     None.
    //--------------------------------------------------------------------  
    @Override
    public void periodic() {
      // Do Nothing
    }

    //--------------------------------------------------------------------
    // Purpose:
    //     Sets the target angle 
    //
    // Notes:
    //     Usually used in auto when we want to reset things
    //--------------------------------------------------------------------  
    public void SetTargetAngle(double targetAngle)
    {
      mTargetAngle = targetAngle;
    }
    
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Clear the angle 
    //
    // Notes:
    //     Usually used in auto when we want to reset things
    //--------------------------------------------------------------------  
    public void ClearCurrentAngle()
    {
      mCurrentAngle = 0.0;
      RobotMap.pigeonIMU.setFusedHeading(0, 20);
    }
    
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive the robot in auto 
  //
  // Notes:
  //    
  //--------------------------------------------------------------------  
  public void AutoDrive(
      double speedInPerTenthSec,
      double speedL, 
      double speedR, 
      double heading,
      FileWriter logFile)
  {
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    double [] xyz_dps = new double [3];
    RobotMap.pigeonIMU.getRawGyro(xyz_dps);
    RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
    
    
    
    mCurrentAngle = fusionStatus.heading;
  
    if(heading>180)
    {
      heading -= 360.0;
    }
    
    double targetSpeedL = speedL*-25.5;
    double targetSpeedR = speedR*-25.5;
   
    double angle_difference = heading - mCurrentAngle;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
  
    double turn = 3.0*angle_difference;
  
      targetSpeedL += turn;
      targetSpeedR -= turn;
    
    RobotMap.driveSRXDriveLF.set(ControlMode.Velocity, targetSpeedR);
    RobotMap.driveSRXDriveRF.set(ControlMode.Velocity, targetSpeedL);
  
    try {
      logFile.write(
          Timer.getFPGATimestamp() + ", " +
          speedL + ", " +
          speedR + ", " +
          heading + ", " +
          turn + ", " +
          targetSpeedL + ", " +
          targetSpeedR + ", " +
          RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0) + ", " +
          RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0) + ", " +
          RobotMap.driveSRXDriveLF.getMotorOutputVoltage() + ", " +
          RobotMap.driveSRXDriveRF.getMotorOutputVoltage() +", " +
          angle_difference + "\n"
      );
    }
    catch(IOException e) {
        e.printStackTrace();
    }
  
  
}

    
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Drive using the joystick 
    //
    // Notes:
    //     none
    //--------------------------------------------------------------------  
    public void DriveRobot(Joystick driveJoystick) {
      
      double y = driveJoystick.getRawAxis(1);
      double x = driveJoystick.getRawAxis(2);

      x = DriveMath.DeadBand(x,0.05);
      y = DriveMath.DeadBand(y,0.05);
          
      // Adjust for speed, check if the fast button is pushed
      if (true == driveJoystick.getRawButton(Constants.kFastButton)) {
        // Do Nothing
      } else {
        // Is the slow button pushed
        if (true == driveJoystick.getRawButton(Constants.kSlowButton)) {
        	y *= Constants.kNormalSpeed;
        	x *= Constants.kNormalTurnSpeed;
        } else {
        	y *= Constants.kSlowSpeed;
            x *= Constants.kSlowTurnSpeed;
        }
      }
      
      if(DriveControlState.OPEN_LOOP == mState)
      {
        DriveRobot(y, x);
    	//DriveRobotWithOutGyro(y,x);
      }
    }
    
    
    public void DriveRobotWithOutGyro(double speed, double turn) {
    	RawDriveRobot(speed*0.5, turn*0.5);
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Drive the robot using throttle and turn 
    //
    // Notes:
    //     Usually used in auto when we want to reset things
    //--------------------------------------------------------------------  
    public void DriveRobot(double speed, double turn) {      
      PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
      double [] xyz_dps = new double [3];
      RobotMap.pigeonIMU.getRawGyro(xyz_dps);
      RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
      
      mCurrentAngle = fusionStatus.heading;
      double currentAngularRate = xyz_dps[2];
      double turnThrottle = turn;
    
      //System.out.format("%f\n", mCurrentAngle);
      
      
      // IF we are turning, turn off the gyro
      if (Math.abs(turn) > 0.2) {
        RawDriveRobot(speed*.5, turn*.5);
        mTargetAngle = mCurrentAngle;
      } else {
        if (Math.abs(speed) > 0.1) {
          double angleError = (mTargetAngle - mCurrentAngle);
          turnThrottle = angleError * kPgain - (currentAngularRate) * kDgain;
          double maxThrot = DriveMath.MaxCorrection(speed, kMaxCorrectionRatio);
          turnThrottle =  -1*DriveMath.Cap(turnThrottle, maxThrot);
          RawDriveRobot(speed,turnThrottle);
        }
        else
        {
          RawDriveRobot(0, 0);
          mTargetAngle = mCurrentAngle;
        }
      }
    }
    
    //--------------------------------------------------------------------
    // Purpose:
    //     Drive the robot using throttle and turnm it adds the gyro 
    //
    // Notes:
    //     Usually used in auto when we want to reset things
    //--------------------------------------------------------------------  
    public void DriveRobotWithGyro(double speed, double turn) {
      PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
      double [] xyz_dps = new double [3];
      RobotMap.pigeonIMU.getRawGyro(xyz_dps);
      RobotMap.pigeonIMU.getFusedHeading(fusionStatus);
      
      mCurrentAngle = fusionStatus.heading;
      double currentAngularRate = xyz_dps[Constants.kGyroZ];
      double turnThrottle = turn;
      
      // IF we are turning, turn off the gyro
      if (Math.abs(turn) > Constants.kGyroDeadband) {
        RawDriveRobot(speed/1.3, turn/1.3);
        mTargetAngle = mCurrentAngle;
      } else {
        if (Math.abs(speed) >Constants.kGyroDeadband) {
          double angleError = (mTargetAngle - mCurrentAngle);
          /* very simple Proportional and Derivative   (PD) loop with a cap,
           * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
          turnThrottle = angleError * kPgain - (currentAngularRate) * kDgain;
          /* the max correction is the forward throttle times a scalar,
           * This can be done a number of ways but basically only apply small turning correction when we are moving slow
           * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
          double maxThrot = DriveMath.MaxCorrection(speed, kMaxCorrectionRatio);
          turnThrottle =  DriveMath.Cap(turnThrottle, maxThrot);
          
          RobotMap.netTable.putNumber("gyroRate", currentAngularRate);
          RobotMap.netTable.putNumber("angleError", angleError);
          RobotMap.netTable.putNumber("turnAdj", turnThrottle);
          
          RawDriveRobot(speed,turnThrottle);
        }
        else
        {
          RawDriveRobot(0, 0);
          mTargetAngle = mCurrentAngle;
        }
      }
    }

    //--------------------------------------------------------------------
    // Purpose:
    //     Set the velocity of the robot 
    //
    // Notes:
    //     None
    //--------------------------------------------------------------------  
    protected void RawDriveRobot(double speed, double turn) {
      // Ramp the speed
      if(mActualSpeed != speed)
      {
        if(mActualSpeed<speed)
        {
          mActualSpeed = Math.min(mActualSpeed+kSpeedGain, speed);
        }
        else
        {
          mActualSpeed = Math.max(mActualSpeed-kSpeedGain, speed);
        }
      }
      
      double targetSpeedL = (mActualSpeed + turn) * 4000;
      double targetSpeedR = (mActualSpeed - turn) * 4000;
      
      RobotMap.driveSRXDriveLF.set(ControlMode.Velocity, targetSpeedL);
      RobotMap.driveSRXDriveRF.set(ControlMode.Velocity, targetSpeedR);
/*
      if(true == mDisplay.isExpired())
      {
        mDisplay.reset();
        double timestamp = Timer.getFPGATimestamp();    
        System.out.format("%f %6.2f %6.2f %6d %6d %6d %6d\n",
            timestamp,targetSpeedL,targetSpeedR,
          RobotMap.driveSRXDriveLF.getSelectedSensorPosition(0),
          RobotMap.driveSRXDriveLF.getSelectedSensorVelocity(0),
          RobotMap.driveSRXDriveRF.getSelectedSensorPosition(0),
          RobotMap.driveSRXDriveRF.getSelectedSensorVelocity(0));
      }
      */
    }
}

