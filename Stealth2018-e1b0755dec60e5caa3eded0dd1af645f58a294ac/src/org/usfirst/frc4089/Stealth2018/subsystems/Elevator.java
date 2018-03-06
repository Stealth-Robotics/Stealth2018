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
    setDefaultCommand(new UserElevator());
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive using the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void DriveElevator(Joystick driveJoystick) {
    
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
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // Main Elevator Functions
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------
  final double elevatorKp = 0.035;
  final double elevatorKi = 0.00069;
  final double elevatorKd = 0;

  
  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidElevatorTarget;
  private double elevatorPreviousError = 0;
  private double elevatorIntegral = 0;
  private double elevatorLastEncoderTicks = 0;
  private double elevatorEncoderCount = 0;

  
  //--------------------------------------------------------------------
  // Purpose:
  //     Drive elevator using the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  private void HandleElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator, 0.20);
    int change = (int)(-yElevator * 10);
    boolean topSwitch = RobotMap.elevatorSwitchTop.get();
    boolean bottomSwitch = RobotMap.elevatorSwitchBottom.get();
    
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
    } else {
      // only change things if the user wants to to advoid messing with auto
      if(0 != change)
      {
        // if nothing is pressed move the elevator
        if((false == topSwitch)&&
            (false == bottomSwitch))
        {
          AddElevatorTarget(change);
        }
        else
        {  
          // Don't change the drive if the switches are set
          if((true == bottomSwitch)&&
              (change>0))
           {
             AddElevatorTarget(change);
           }
          else
          {
              if(change<0)
             {
               AddElevatorTarget(change);
             }
          }
        }
      }
    }
  }   
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the elevator to a known position 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void SetElevatorTarget(int target)
  {
    pidElevatorTarget = target;  
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Adds an amount to the current position 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void AddElevatorTarget(int amount)
  {
    pidElevatorTarget += amount;  
  }
    
  //--------------------------------------------------------------------
  // Purpose:
  //     Get Elevator Current Posiotn 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public int GetElevatorPosition() {
    return RobotMap.elevatorEncoder.get();
   }
  //--------------------------------------------------------------------
  // Purpose:
  //     Runs the motion control on the elevator 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void MoveElevatorToTarget() {
    //get current ticks
    int elevatorEncoderTicks = RobotMap.elevatorEncoder.get();

    //make sure it doesn't go past limit switches
    if (RobotMap.elevatorSwitchBottom.get() == true) {
      RobotMap.elevatorEncoder.reset();
      
      SetElevatorTarget(25);
      elevatorIntegral = 0.0;
      elevatorPreviousError = 0.0;
    }
    
    if (RobotMap.elevatorSwitchTop.get() == true) {
      SetElevatorTarget(elevatorEncoderTicks-25);
      elevatorIntegral = 0.0;
      elevatorPreviousError = elevatorEncoderTicks;
    }
    
	  //calculate error
	  double error = pidElevatorTarget - elevatorEncoderTicks;
	  //Calculate Motor Power using PID loop
	  elevatorIntegral = elevatorIntegral + error;
	  elevatorIntegral = Math.max(0, Math.min(1440, elevatorIntegral));
	  double derivative = (error - elevatorPreviousError);
	  double motorOutput = elevatorKp*error + elevatorKi*elevatorIntegral + elevatorKd*derivative;
	  elevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  elevatorLastEncoderTicks = elevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  RobotMap.elevatorMotor.set(motorOutput);

	  RobotMap.netTable.putNumber("elevatorMotorOutput", motorOutput);
    RobotMap.netTable.putNumber("elevatorError", error);
    RobotMap.netTable.putNumber("elevatorSetPoint", pidElevatorTarget);
    RobotMap.netTable.putNumber("elevatorIntegral", elevatorIntegral);
    RobotMap.netTable.putNumber("elevatorDerivative", derivative);
  }

  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  // Picker Elevator Functions
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------
  //--------------------------------------------------------------------

  // --------------------------------------------------------------------
  // Constants:
  // --------------------------------------------------------------------
  final double pickerElevatorKp = 0.03;
  final double pickerElevatorKi = 0;
  final double pickerElevatorKd = 0;

  // --------------------------------------------------------------------
  // Attributes:
  // --------------------------------------------------------------------
  private int pidPickerElevatorTarget;
  double pickerElevatorPreviousError = 0;
  double pickerElevatorIntegral = 0;
  double pickerElevatorLastEncoderTicks = 0;
  double pickerElevatorEncoderCount = 0;
  
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the setpoint from the joystick 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  private void HandlePickerElevator(double yElevator) {
    yElevator = DriveMath.DeadBand(yElevator,0.25);
    int change = (int)(-yElevator * 10);
    boolean topSwitch = RobotMap.pickerElevatorSwitchTop.get();
    boolean bottomSwitch = RobotMap.pickerElevatorSwitchBottom.get();
    
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
    } else {
      // only change things if the user wants to to advoid messing with auto
      if(0 != change)
      {
        // if nothing is pressed move the elevator
        if((false == topSwitch)&&
            (false == bottomSwitch))
        {
          AddPickerElevatorTarget(change);
        }
        else
        {  
          // Don't change the drive if the switches are set
          if((true == bottomSwitch)&&
              (change>0))
           {
             AddPickerElevatorTarget(change);
           }
          else
          {
              if(change<0)
             {
               AddPickerElevatorTarget(change);
             }
          }
        }
      }
    }
  }
 
  //--------------------------------------------------------------------
  // Purpose:
  //     Sets the setpoint 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void SetPickerElevatorTarget(int target) {
    pidPickerElevatorTarget = target;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Adds to the setpoint 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void AddPickerElevatorTarget(int target) {
    pidPickerElevatorTarget += target;
   }

  //--------------------------------------------------------------------
  // Purpose:
  //     Picker Elevator PID loop 
  //
  // Notes:
  //     none
  //--------------------------------------------------------------------  
  public void MovePickerElevatorToTarget() {
        //get current ticks
   int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
   boolean pickerElevatorSwitchTotalBottom = RobotMap.pickerElevatorTotalBottom.get();

/*
   if (RobotMap.pickerElevatorSwitchBottom.get() == true) {
     RobotMap.pickerElevatorEncoder.reset();
     SetPickerElevatorTarget(0);
     pickerElevatorIntegral = 0.0;
     pickerElevatorPreviousError = 0.0;
   }
   
   if (RobotMap.elevatorSwitchTop.get() == true) {
     SetPickerElevatorTarget(pickerElevatorEncoderTicks);
     pickerElevatorIntegral = 0.0;
     pickerElevatorPreviousError = pickerElevatorEncoderTicks;
   }
	*/
   
	  //calculate error
	  double error = pidPickerElevatorTarget - pickerElevatorEncoderTicks;
	  //calculate motor output power
	  pickerElevatorIntegral = pickerElevatorIntegral + error;
	  pickerElevatorIntegral = Math.max(0, Math.min(1440, pickerElevatorIntegral));
	  double derivative = (error - pickerElevatorPreviousError);
	  double motorOutput = pickerElevatorKp*error + pickerElevatorKi*pickerElevatorIntegral + pickerElevatorKd*derivative;
	  pickerElevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  pickerElevatorLastEncoderTicks = pickerElevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  RobotMap.pickerElevatorMotor.set(motorOutput);
	  
	  RobotMap.netTable.putNumber("pickerElevatorMotorOutput", motorOutput);
    RobotMap.netTable.putNumber("pickerError", error);
    RobotMap.netTable.putNumber("pickerSetPoint", pidPickerElevatorTarget);
    RobotMap.netTable.putNumber("pickerElevatorIntegral", pickerElevatorIntegral);
    RobotMap.netTable.putNumber("pickerElevatorDerivative", derivative);
 }

 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 // Climb Hook Functions
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 //--------------------------------------------------------------------
 //TODO: Move to it's own command not under the elevator subsystem
  public void EngageClimbHook() {
	  //1. Lower elevator to bottom (limit switch 1 = true)
	  SetElevatorTarget(0);
	  SetPickerElevatorTarget(0);
	  
	  //2. Engage climb hook pneumatic 
	  
	  
	  //3. Lower elevator to retract grabber
	  SetPickerElevatorTarget(-100000); 
	  
	  //4. Engage pneumatic to lock grabber in upright position
	  
	  
	  //5. Return to user drive mode
	  
	  
  }
}

