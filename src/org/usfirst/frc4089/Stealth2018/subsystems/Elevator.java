//----------------------------------------------------------------------------
//
//  $Workfile: Elevator.java$
//
//  $Revision: X$
//
//  Project:    Stealth Libraries
//
//                            Copyright (c) 2018
//                           Cedarcrest High School
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//----------------------------------------------------------------------------
package org.usfirst.frc4089.Stealth2018.subsystems;

import org.usfirst.frc4089.Stealth2018.Robot;
import org.usfirst.frc4089.Stealth2018.RobotMap;
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.utilities.DriveMath;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;

//----------------------------------------------------------------------------
//Class Declarations
//----------------------------------------------------------------------------
//
//Class Name: Elevator
//
//Purpose:
//  Moves both the Elevator and Picker Elevator up and down
//
//----------------------------------------------------------------------------
public class Elevator extends Subsystem {

  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the default command 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void initDefaultCommand() {
    //setDefaultCommand(new UserElevator());
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive using the Joystick 
  //--------------------------------------------------------------------  
  public void DriveElevator(Joystick driveJoystick) {
    
	  
	//TODO Move these button mapings to Robot.oi
    if (driveJoystick.getRawButton(9) || Robot.oi.dsJoystick.getRawButton(5)) {
      RobotMap.overrideElevator = true;
    }
    if (driveJoystick.getRawButton(10) || Robot.oi.dsJoystick.getRawButton(1)) {
      RobotMap.overridePickerElevator = true;
    }
    
    HandleElevator(driveJoystick.getRawAxis(1));
    HandlePickerElevator(driveJoystick.getRawAxis(5));
  }
    
  
  //--------------------------------------------------------------------
  // Main Elevator Functions
  //--------------------------------------------------------------------
  
  public boolean isElevatorAtTarget = false;
  
  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------
  final double elevatorKp = 0.035;
  final double elevatorKi = 0;
  final double elevatorKd = 0;

  
  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidElevatorTarget;
  private double elevatorPreviousError = 0;
  private double elevatorIntegral = 0;

  
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive elevator using the joystick 
  //--------------------------------------------------------------------  
  private void HandleElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator, 0.15);
    int change = (int)(-yElevator * 35);
    boolean topSwitch = RobotMap.elevatorSensors.isFwdLimitSwitchClosed();
    boolean bottomSwitch = RobotMap.elevatorSensors.isRevLimitSwitchClosed();
    
    if (RobotMap.overrideElevator) {
      
      if(topSwitch) {
        if(-yElevator > 0) {
          yElevator = 0;
        }
      }
      
      if (bottomSwitch) {
        if (-yElevator < 0) {
          yElevator = 0;
        }
      }
      
      
      
      RobotMap.elevatorMotor.set(-yElevator);
      
      System.out.println("Override " + RobotMap.elevatorMotor.getSelectedSensorPosition(0) + " " + RobotMap.elevatorMotor.get());
    } else {
    	// only change things if the user wants to to avoid messing with auto
        if(change != 0)
        {
          // if nothing is pressed move the elevator
          if((!topSwitch) && (!bottomSwitch)) {
            AddElevatorTarget(change);
          } else {
           // Don't change the drive if the switches are set
           if((bottomSwitch) && (change > 0)) {
              AddElevatorTarget(change);
           } else if((topSwitch) && (change < 0)) {
             AddElevatorTarget(change);
           }
         }
        }
    }
  }   
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the elevator to a known position 
  //--------------------------------------------------------------------  
  public void SetElevatorTarget(int target)
  {
    pidElevatorTarget = target;  
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Adds an amount to the current position 
  //--------------------------------------------------------------------  
  public void AddElevatorTarget(int amount)
  {
    pidElevatorTarget += amount;  
  }
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Runs the motion control on the elevator 
  //--------------------------------------------------------------------  
  public void MoveElevatorToTarget() {
      //get current ticks
      int elevatorEncoderTicks = RobotMap.elevatorMotor.getSelectedSensorPosition(0);
	  boolean topSwitch = RobotMap.elevatorSensors.isFwdLimitSwitchClosed();
	  boolean bottomSwitch = RobotMap.elevatorSensors.isRevLimitSwitchClosed();
	  
	  //check to see target vs limit switch
	  if (topSwitch && elevatorEncoderTicks < pidElevatorTarget) {
		  SetElevatorTarget(elevatorEncoderTicks);
	  }
	  if (bottomSwitch && elevatorEncoderTicks > pidElevatorTarget) {
		  SetElevatorTarget(elevatorEncoderTicks);
	  }
    
	  //calculate error
	  double error = pidElevatorTarget - elevatorEncoderTicks;
	  //set flags
	  if (error < 10) {
		  isElevatorAtTarget = true;
	  } else {
		  isElevatorAtTarget = false;
	  }
	  
	  //calculate Integral
	  elevatorIntegral = elevatorIntegral + error;
	  //clamp Integral
	  elevatorIntegral = Math.max(0, Math.min(1440, elevatorIntegral));
	  //Calculate derivative
	  double derivative = (error - elevatorPreviousError);
	  //Calculate Motor Power using PID loop
	  double motorOutput = elevatorKp*error + elevatorKi*elevatorIntegral + elevatorKd*derivative;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  //record previous error
	  elevatorPreviousError = error;
	  
	  //print out debugging statements
	  System.out.format("!override %d %d %f %f %f\n", pidElevatorTarget, elevatorEncoderTicks, error, derivative, motorOutput);
	  
	  //set the motor to the correct value
	  RobotMap.elevatorMotor.set(motorOutput);
  }

  
  //--------------------------------------------------------------------
  // Picker Elevator Functions
  //--------------------------------------------------------------------
  
  public boolean isPickerElevatorAtTarget = false;
  
  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------
  final double pickerElevatorKp = 0.005;
  final double pickerElevatorKi = 0;
  final double pickerElevatorKd = 0;

  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidPickerElevatorTarget;
  private double pickerElevatorPreviousError = 0;
  private double pickerElevatorIntegral = 0;
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the setpoint from the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  private void HandlePickerElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator,0.1);
    int change = (int)(-yElevator * 35);
    boolean topSwitch = RobotMap.pickerElevatorSensors.isFwdLimitSwitchClosed();
    boolean bottomSwitch = RobotMap.pickerElevatorSensors.isRevLimitSwitchClosed();
    
    if (RobotMap.overridePickerElevator) {
      
      if(topSwitch) {
        if(-yElevator > 0) {
          yElevator = 0;
        }
      }
      
      if (bottomSwitch) {
        if (-yElevator < 0) {
          yElevator = 0;
        }
      }
      
      RobotMap.pickerElevatorMotor.set(-yElevator);
      
      //print out debugging information
      System.out.println("Override " + RobotMap.pickerElevatorMotor.getSelectedSensorPosition(0) + " " + RobotMap.pickerElevatorMotor.get());
    } else {
      // only change things if the user wants to to avoid messing with auto
      if(change != 0)
      {
        // if nothing is pressed move the elevator
        if((!topSwitch) && (!bottomSwitch)) {
          AddPickerElevatorTarget(change);
        } else {  
         // Don't change the drive if the switches are set
         if((bottomSwitch) && (change > 0)) {
            AddPickerElevatorTarget(change);
         } else if((topSwitch) && (change < 0)) {
           AddPickerElevatorTarget(change);
         }
       }
      }
    }
  }
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the picker elevator target
  //--------------------------------------------------------------------  
  public void SetPickerElevatorTarget(int target) {
    pidPickerElevatorTarget = target;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Adds to the value to picker target
  //--------------------------------------------------------------------  
  public void AddPickerElevatorTarget(int value) {
    pidPickerElevatorTarget += value;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Picker Elevator PID loop 
  //--------------------------------------------------------------------  
  public void MovePickerElevatorToTarget() {
	  //get current ticks
	  int pickerElevatorEncoderTicks = RobotMap.pickerElevatorMotor.getSelectedSensorPosition(0);
	  boolean topSwitch = RobotMap.pickerElevatorSensors.isFwdLimitSwitchClosed();
	  boolean bottomSwitch = RobotMap.pickerElevatorSensors.isRevLimitSwitchClosed();
	  
	  //check to see target vs limit switch
	  if (topSwitch && pickerElevatorEncoderTicks < pidPickerElevatorTarget) {
		  SetPickerElevatorTarget(pickerElevatorEncoderTicks);
	  }
	  if (bottomSwitch && pickerElevatorEncoderTicks > pidPickerElevatorTarget) {
		  SetPickerElevatorTarget(pickerElevatorEncoderTicks);
	  }
	  
	  //calculate error
	  double error = pidPickerElevatorTarget - pickerElevatorEncoderTicks;
	  //set flags
	  if (error < 10) {
		  isPickerElevatorAtTarget = true;
	  } else {
		  isPickerElevatorAtTarget = false;
	  }
	  //calculate Integral
	  pickerElevatorIntegral = pickerElevatorIntegral + error;
	  //clamp Integral between 0 and max ticks for elevator (1440)
	  pickerElevatorIntegral = Math.max(0, Math.min(1440, pickerElevatorIntegral));
	  //Calculate derivative
	  double derivative = (error - pickerElevatorPreviousError);
	  //calculate motor output power
	  double motorOutput = pickerElevatorKp*error + pickerElevatorKi*pickerElevatorIntegral + pickerElevatorKd*derivative;
	  //record the error for next loop
	  pickerElevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  //print out debugging statements
	  //System.out.format("!override %d %d %f %f %f\n", pidPickerElevatorTarget, pickerElevatorEncoderTicks, error, derivative, motorOutput);
	  
	  //set the motor to the correct value
	  RobotMap.pickerElevatorMotor.set(motorOutput);
 }
}

