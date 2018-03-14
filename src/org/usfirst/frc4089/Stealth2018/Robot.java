// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4089.Stealth2018;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc4089.Stealth2018.commands.*;
import org.usfirst.frc4089.Stealth2018.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
    
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;

    public static Drive drive;
    public static Elevator elevator;
    public static Picker picker;
    public static Utilities utilities;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        
        if(Constants.UseCamera) {
	        //start camera server
	        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	        camera.setResolution(320,240);
	        camera.setFPS(30);
        }
        
        drive = new Drive();
        elevator = new Elevator();
        picker = new Picker();
        utilities = new Utilities();

        Robot.elevator.SetElevatorTarget(0);
        Robot.elevator.SetPickerElevatorTarget(0);
        RobotMap.elevatorEncoder.reset();
        RobotMap.pickerElevatorEncoder.reset();
        
        System.out.println("robot init");
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        
        
        // Add commands to Autonomous Sendable Chooser

        chooser.addObject("1 Position One", new PositionOne());
        //chooser.addObject("2 Position Two", new PositionTwo());
        chooser.addDefault("3 Position Three", new PositionThree());
        //chooser.addObject("4 Position Four", new PositionFour());
        chooser.addObject("5 Position Five", new PositionFive());
        SmartDashboard.putData("Auto mode", chooser);
        
    }


    @Override
    public void robotPeriodic(){
    }
    
    @Override
    public void disabledInit(){
      if (mTestCommand != null) mTestCommand.cancel();
      //picker.hugBlock();
      //RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
    }

    private void DisplaySensors()
    {
      System.out.format("%b %b %b %b %b %d %d\n", 
          RobotMap.elevatorSwitchTop.get(),
          RobotMap.elevatorSwitchBottom.get(),
          RobotMap.pickerElevatorSwitchTop.get(),
          RobotMap.pickerElevatorSwitchBottom.get(),
          RobotMap.pickerElevatorTotalBottom.get(),
          RobotMap.elevatorEncoder.get(),
          RobotMap.pickerElevatorEncoder.get()
          );

    }
    
    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        
        if(true == Robot.oi.mechJoystick.getRawButton(9))
        {
          DisplaySensors();
        }
    }

    Command mTestCommand;
    
    @Override
    public void autonomousInit() {
      if (mTestCommand != null) mTestCommand.cancel();
      Robot.picker.ungrabClimber();
      
      RobotMap.SetUpTalonsForAuto();
      drive.ClearCurrentAngle();

      drive.SetAuto();
      
      Robot.elevator.SetElevatorTarget(0);
      Robot.elevator.SetPickerElevatorTarget(0);
      

      // When we used the auto stuff for this the autonomous we running twice.
      // So we are doing this the long way.  We need to research why it was running twice.
      if(true == chooser.getSelected().getName().equals("PositionOne"))
      {
        mTestCommand = new PositionOne();
        Scheduler.getInstance().add(mTestCommand);
      }
      
      if(true == chooser.getSelected().getName().equals("PositionThree"))
      {
        mTestCommand = new PositionThree();
        Scheduler.getInstance().add(mTestCommand);
      }
        
      if(true == chooser.getSelected().getName().equals("PositionFive"))
      {
        mTestCommand = new PositionFive();
        Scheduler.getInstance().add(mTestCommand);
      }
      
      /* Should be that
      autonomousCommand = chooser.getSelected();
      // schedule the autonomous command (example)
      if (autonomousCommand != null)
      {
        autonomousCommand.start();
      }
      */
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
       Scheduler.getInstance().run();
       
       if(!RobotMap.overrideElevator) {
           Robot.elevator.MoveElevatorToTarget();
         }
         if(!RobotMap.overridePickerElevator) {
           Robot.elevator.MovePickerElevatorToTarget();
         }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (mTestCommand != null) mTestCommand.cancel();
        System.out.println("tele init");
        RobotMap.SetUpTalonsForTele();
        Robot.drive.SetTele();
        Robot.picker.ungrabClimber();
        RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
        Robot.elevator.SetElevatorTarget(Robot.elevator.GetElevatorPosition());
        
        
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
        //DisplaySensors();
        
        Robot.drive.DriveRobot(oi.driveJoystick);
        Robot.elevator.DriveElevator(oi.mechJoystick);
        Robot.picker.DrivePickerWheels(oi.mechJoystick);
        
        if(!RobotMap.overrideElevator) {
          Robot.elevator.MoveElevatorToTarget();
        }
        if(!RobotMap.overridePickerElevator) {
          Robot.elevator.MovePickerElevatorToTarget();
        }
    }
    
    
    @Override
    public void testInit() {
      
    }   

    @Override
    public void testPeriodic() {
      
    }   
}
