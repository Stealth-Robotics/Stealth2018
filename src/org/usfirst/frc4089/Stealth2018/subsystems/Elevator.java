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
import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.utilities.DriveMath;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {
    private int elevatorTargetTick;
    private int pickerElevatorTargetTick;
  	private int pidElevatorTarget;
  	private int pidPickerElevatorTarget;

	
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
      HandleElevator(driveJoystick.getRawAxis(1));
      HandlePickerElevator(driveJoystick.getRawAxis(5));
    }
    
    
    public void HandleElevator(double yElevator) {
		  
	  yElevator = DriveMath.DeadBand(yElevator, 0.25);
	  
	  elevatorTargetTick += -yElevator * 8;
	  	 
	  SetElevatorTarget(elevatorTargetTick);
    }   
 
    
    public void HandlePickerElevator(double yElevator) {
      
      yElevator = DriveMath.DeadBand(yElevator,0.25);

	  pickerElevatorTargetTick += -yElevator * 6;

	  SetPickerElevatorTarget(pickerElevatorTargetTick);
    }
  

  	public void SetElevatorTarget(int target) {
		pidElevatorTarget = target;
	}
  
  	
  
  
  final double elevatorKp = 0.035;
  final double elevatorKi = 0.00069;
  final double elevatorKd = 0;
  double elevatorPreviousError = 0;
  double elevatorIntegral = 0;
  double elevatorLastEncoderTicks = 0;
  double elevatorEncoderCount = 0;
  public void MoveElevatorToTarget() {
	  //get limit switches
	  boolean elevatorSwitchTop = RobotMap.elevatorSwitchTop.get();
    boolean elevatorSwitchBottom = RobotMap.elevatorSwitchBottom.get();
      
      //reset encoder if at bottom switch
	  if (elevatorSwitchBottom == true) {
    	  RobotMap.elevatorEncoder.reset();
      }
	  
	  //get current ticks
	  int elevatorEncoderTicks = RobotMap.elevatorEncoder.get();
	  
	  RobotMap.netTable.putNumber("ElevatorEncoderTicks", elevatorEncoderTicks);
	  
	  if (elevatorLastEncoderTicks != elevatorEncoderTicks && RobotMap.elevatorMotor.get() != 0)
	  {
		  elevatorEncoderCount += 1;
	  } else {
		  elevatorEncoderCount = 0;
	  }
      
	  //calculate error
	  double error = pidElevatorTarget - elevatorEncoderTicks;
	  RobotMap.netTable.putNumber("elevatorError", error);
	  RobotMap.netTable.putNumber("elevatorSetPoint", pidElevatorTarget);
	  //Calculate Motor Power using PID loop
	  elevatorIntegral = elevatorIntegral + error;
	  elevatorIntegral = Math.max(0, Math.min(1440, elevatorIntegral));
	  RobotMap.netTable.putNumber("elevatorIntegral", elevatorIntegral);
	  double derivative = (error - elevatorPreviousError);
	  RobotMap.netTable.putNumber("elevatorDerivative", derivative);
	  double motorOutput = elevatorKp*error + elevatorKi*elevatorIntegral + elevatorKd*derivative;
	  elevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  
	  //make sure it doesn't go past limit switches
	  if (elevatorSwitchBottom == true) {
		  //motorOutput = 0;
	    SetElevatorTarget(elevatorEncoderTicks);
	  }
	  
	  if (elevatorSwitchTop == true) {
		  //motorOutput = 0;
		  SetElevatorTarget(elevatorEncoderTicks);
	  }
	  
	  if (elevatorEncoderCount > 20) {
		  motorOutput = 0;
	  }
	  
	  System.out.println("elevatorEncoderCount : " + elevatorEncoderCount);
	  
	  elevatorLastEncoderTicks = elevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  RobotMap.elevatorMotor.set(motorOutput);
	  RobotMap.netTable.putNumber("elevatorMotorOutput", motorOutput);
	  
	  
	  //Debugin stuff
	  System.out.println("elevatorEncoderTicks: " + elevatorEncoderTicks);
	  System.out.println("ElevatorEncoderTargetTicks: " + pidElevatorTarget);
	  //System.out.println("elevatorMotorPower: " + RobotMap.elevatorMotor.get());
 }
 
 public void SetPickerElevatorTarget(int target) {
 	pidPickerElevatorTarget = target;
 }

 final double pickerElevatorKp = 0.03;
 final double pickerElevatorKi = 0;
 final double pickerElevatorKd = 0;
 double pickerElevatorPreviousError = 0;
 double pickerElevatorIntegral = 0;
 double pickerElevatorLastEncoderTicks = 0;
 double pickerElevatorEncoderCount = 0;
 public void MovePickerElevatorToTarget() {
	 //get limit switches
	 boolean elevatorSwitchTop = RobotMap.pickerElevatorSwitchTop.get();
     boolean pickerElevatorSwitchBottom = RobotMap.pickerElevatorSwitchBottom.get();
     boolean pickerElevatorSwitchTotalBottom = RobotMap.pickerElevatorTotalBottom.get();
     
      //reset encoder if at bottom switch
	  if (pickerElevatorSwitchTotalBottom == true) {
   	  RobotMap.pickerElevatorEncoder.reset();
      }
	  
	  //get current ticks
	  int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
	  if (pickerElevatorLastEncoderTicks != pickerElevatorEncoderTicks && RobotMap.pickerElevatorMotor.get() != 0)
	  {
		  pickerElevatorEncoderCount += 1;
	  } else {
		  pickerElevatorEncoderCount = 0;
	  }
	  
	  
	  //calculate error
	  double error = pidPickerElevatorTarget - pickerElevatorEncoderTicks;
	  RobotMap.netTable.putNumber("pickerError", error);
	  RobotMap.netTable.putNumber("pickerSetPoint", pidPickerElevatorTarget);
	  //calculate motor output power
	  pickerElevatorIntegral = pickerElevatorIntegral + error;
	  pickerElevatorIntegral = Math.max(0, Math.min(1440, pickerElevatorIntegral));
	  RobotMap.netTable.putNumber("pickerElevatorIntegral", pickerElevatorIntegral);
	  double derivative = (error - pickerElevatorPreviousError);
	  RobotMap.netTable.putNumber("pickerElevatorDerivative", derivative);
	  double motorOutput = pickerElevatorKp*error + pickerElevatorKi*pickerElevatorIntegral + pickerElevatorKd*derivative;
	  pickerElevatorPreviousError = error;
	  //clamp the value between -1 and 1
	  motorOutput = Math.max(-1, Math.min(1, motorOutput));
	  
	  
	  //make sure it doesn't go past limit switches
	  if (pickerElevatorSwitchBottom == true) {
		  //motorOutput = 0;
	    SetPickerElevatorTarget(pickerElevatorEncoderTicks);
	  }
	  
	  if (pickerElevatorSwitchTotalBottom == true) {
	    //motorOutput = 0;
	    SetPickerElevatorTarget(pickerElevatorEncoderTicks);
	  }
	  
	  if (elevatorSwitchTop == true) {
		  //motorOutput = 0;
		  SetPickerElevatorTarget(pickerElevatorEncoderTicks);
	  }
	  
	  if (pickerElevatorEncoderCount > 20) {
		  motorOutput = 0;
	  }
	  
	  pickerElevatorLastEncoderTicks = pickerElevatorEncoderTicks;
	  
	  //set the motor to the correct value
	  RobotMap.pickerElevatorMotor.set(motorOutput);
	  
	  RobotMap.netTable.putNumber("pickerElevatorMotorOutput", motorOutput);
	  
	  //Debugin stuff
	  //System.out.println("pickerElevatorEncoderTicks: " + pickerElevatorEncoderTicks);
	  //System.out.println("pickerElevatorMotorPower: " + RobotMap.pickerElevatorMotor.get());
	 
	 /*boolean elevatorSwitchTop = RobotMap.pickerElevatorSwitchTop.get();
     boolean elevatorSwitchBottom = RobotMap.pickerElevatorSwitchBottom.get();
     
     if (elevatorSwitchBottom == true) {
    	 RobotMap.pickerElevatorEncoder.reset();
     }
	 
	 // when -100000, lower until bottom limit 2 == true
	 
	  int pickerElevatorEncoderTicks = RobotMap.pickerElevatorEncoder.get();
	  if(pidPickerElevatorTarget == pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(0);
	  }
	  if(pidPickerElevatorTarget < pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(1);
	  }
	  else if(pidPickerElevatorTarget > pickerElevatorEncoderTicks)
	  {
		  RobotMap.pickerElevatorMotor.set(-1);
	  }
	  
	  System.out.println("pickerElevatorEncoderTicks: " + pickerElevatorEncoderTicks);*/
 } 
   
 
}

