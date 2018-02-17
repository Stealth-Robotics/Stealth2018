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
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Drive drive;
    public static Elevator elevator;
    public static Picker picker;
    public static Climber climber;
    public static Navigation navigation;
    public static Utilities utilities;
    public static PickerPWM pickerPWM;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        
        //start camera server
     //   UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
     //   camera.setResolution(160,120);
     //   camera.setFPS(24);
        
        
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
        
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drive = new Drive();
        elevator = new Elevator();
        picker = new Picker();
        climber = new Climber();
        navigation = new Navigation();
        utilities = new Utilities();
        pickerPWM = new PickerPWM();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        System.out.println("robot init");
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.addDefault("Autonomous Command", new AutonomousCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        SmartDashboard.putData("Auto mode", chooser);
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
    }

    @Override
    public void autonomousInit() {
    	 System.out.println("auto init");
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
        
        //Get Game Setup String as RRR or LLL or RLR ect. R = 0  L = 1 und = -1
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        
        String closeSwitchString = gameData.substring(0,1);
        double closeSwitch = -1;
        if (closeSwitchString.equals("R")) {
        	closeSwitch = 0;
        } else if (closeSwitchString.equals("L")){
        	closeSwitch = 1;
        }
        //put in NT for Hololens to use
        SmartDashboard.putNumber("CloseSwitch", closeSwitch);
        
        String scaleString = gameData.substring(1,2);
        double scale = -1;
        if (scaleString.equals("R")) {
        	scale = 0;
        } else if (scaleString.equals("L")){
        	scale = 1;
        }
        //put in NT for hololens to use
        SmartDashboard.putNumber("Scale", scale);
        
        String farSwitchString = gameData.substring(2,3);
        double farSwitch = -1;
        if (farSwitchString.equals("R")) {
        	farSwitch = 0;
        } else if (farSwitchString.equals("L")){
        	farSwitch = 1;
        }
        //put in NT for hololens to use
        SmartDashboard.putNumber("FarSwitch", farSwitch);
        
        //select correct string to pass to path finding algorithm
        String selectedPath = "";
        if (closeSwitch == 0 && scale == 0) {
        	selectedPath = SmartDashboard.getString("AutoPath_RR", "");
        } else if (closeSwitch == 0 && scale == 1) {
        	selectedPath = SmartDashboard.getString("AutoPath_RL", "");
        } else if (closeSwitch == 1 && scale == 0) {
        	selectedPath = SmartDashboard.getString("AutoPath_LR", "");
        } else if (closeSwitch == 1 && scale == 1) {
        	selectedPath = SmartDashboard.getString("AutoPath_LL", "");
        }
        //print out selected path finding string
        System.out.println("Using : " + selectedPath + " for Autonomus");
        
        RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
        RobotMap.SetUpTalonsForAuto();
        Robot.drive.SetAuto();
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
        RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        //RobotMap.utilitiesPCMCompressor.setClosedLoopControl(true);
        RobotMap.utilitiesPCMCompressor.enabled();
        RobotMap.utilitiesPCMCompressor.start();
        
        Robot.drive.DriveRobot(oi.joystick1);
        Robot.elevator.DriveElevator(oi.joystick2);
        
        boolean enabled = RobotMap.utilitiesPCMCompressor.enabled();
        boolean pressureSwitch = RobotMap.utilitiesPCMCompressor.getPressureSwitchValue();
        double current = RobotMap.utilitiesPCMCompressor.getCompressorCurrent();
        
        System.out.format("%s %s %f\n", enabled?"true":"false",
            pressureSwitch?"true":"false",current);
    }
    
    
    @Override
    public void testPeriodic() {
      System.out.format("%b %b %d\n", 
          RobotMap.elevatorSwitchTop.get(),
          RobotMap.pickerSwitchTop.get(),
          RobotMap.elevatorEncoder.get());
    }   
}
