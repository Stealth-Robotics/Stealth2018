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

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;

    public static Drive drive;
    public static Elevator elevator;
    public static Picker picker;
    public static Climber climber;
    public static Navigation navigation;
    public static Utilities utilities;
    public static PickerPWM pickerPWM;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        
        //start camera server
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(160,120);
        camera.setFPS(24);
        
        
        //for Hololens Debug
        SmartDashboard.putBoolean("Robot Conected", true);
        
        //put SmartDashboard Auto setup values in network tables
        //set defaults here
        SmartDashboard.putBoolean("RedAlience", false);
        SmartDashboard.putString("StartingPos", "1");
        SmartDashboard.putString("AutoPath_LL", "");
        SmartDashboard.putString("AutoPath_LR", "");
        SmartDashboard.putString("AutoPath_RR", "");
        SmartDashboard.putString("AutoPath_RL", "");
        
        drive = new Drive();
        elevator = new Elevator();
        picker = new Picker();
        climber = new Climber();
        navigation = new Navigation();
        utilities = new Utilities();
        pickerPWM = new PickerPWM();

        System.out.println("robot init");
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        
        picker.unlockPciker();

        // Add commands to Autonomous Sendable Chooser

        chooser.addDefault("Autonomous Command", new AutonomousCommand());

        SmartDashboard.putData("Auto mode", chooser);
    }


    @Override
    public void robotPeriodic(){
    }
    
    
    
    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){

    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        
        System.out.format("%b %b %b %b %b %f %f %d %d\n", 
            RobotMap.elevatorSwitchTop.get(),
            RobotMap.elevatorSwitchBottom.get(),
            RobotMap.pickerElevatorSwitchTop.get(),
            RobotMap.pickerElevatorSwitchBottom.get(),
            RobotMap.pickerElevatorTotalBottom.get(),
            oi.mechJoystick.getRawAxis(1),
            oi.mechJoystick.getRawAxis(5),
            RobotMap.elevatorEncoder.get(),
            RobotMap.pickerElevatorEncoder.get()
            );
        
        
    }

    Command mTestCommand;
    
    @Override
    public void autonomousInit() {
      RobotMap.SetUpTalonsForAuto();
      drive.ClearCurrentAngle();
//      mTestCommand = new ScoreInSwitch();
      mTestCommand = new PositionThree();
      Scheduler.getInstance().add(mTestCommand);
      drive.SetAuto();
        
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        System.out.println("tele init");
        RobotMap.SetUpTalonsForTele();
        Robot.drive.SetTele();
        Robot.picker.ungrabClimber();
        RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
        Robot.elevator.SetElevatorTarget(0);
        Robot.elevator.SetPickerElevatorTarget(0);
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
        Robot.drive.DriveRobot(oi.driveJoystick);
        Robot.elevator.DriveElevator(oi.mechJoystick);
        
<<<<<<< HEAD
        
        if(oi.mechJoystick.getRawAxis(3)>0)
        {
          System.out.println(oi.mechJoystick.getRawAxis(3));
          System.out.println("<<");
          RobotMap.pickerLeftMotor.set(oi.mechJoystick.getRawAxis(3));
          RobotMap.pickerRightMotor.set(oi.mechJoystick.getRawAxis(3)*-1);
        }
        else
        {
          if(oi.mechJoystick.getRawAxis(4)>0)
          {
            System.out.println("4");
            RobotMap.pickerLeftMotor.set(oi.mechJoystick.getRawAxis(4)*-1);
            RobotMap.pickerRightMotor.set(oi.mechJoystick.getRawAxis(4));
          }
          else
          {
            RobotMap.pickerLeftMotor.set(0.2);
            RobotMap.pickerRightMotor.set(-0.2);
          }
        }
        
        Robot.elevator.MoveElevatorToTarget();
        Robot.elevator.MovePickerElevatorToTarget();
=======

        //System.out.format("%s %s %f\n", enabled?"true":"false", pressureSwitch?"true":"false",current);
        
        Robot.elevator.MoveElevatorToTarget();
        Robot.elevator.MovePickerElevatorToTarget();

        RobotMap.pickerLeftMotor.set(oi.mechJoystick.getRawAxis(0));
        RobotMap.pickerRightMotor.set(oi.mechJoystick.getRawAxis(0));
>>>>>>> 3273125b796a71d51cdfaf4a801bfa9b5e709152
    }
    
    
    @Override
    public void testPeriodic() {
    }   
}
