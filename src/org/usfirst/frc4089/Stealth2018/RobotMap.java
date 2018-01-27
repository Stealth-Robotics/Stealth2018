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

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static WPI_TalonSRX driveSRX1DriveLR;
    public static WPI_TalonSRX driveSRX2DriveLF;
    public static WPI_TalonSRX driveSRX3DriveRR;
    public static WPI_TalonSRX driveSRX4DriveRF;
    public static DifferentialDrive driveRobotDrive41;
    public static Encoder driveQuadEncLeftDrive;
    public static Encoder driveQuadEncRightDrive;
    public static Encoder elevatorQuadEncElevator;
    public static WPI_TalonSRX elevatorSRX5Elevator;
    public static PIDController elevatorPIDController1;
    public static DigitalInput elevatorSwitch7ElevatorTop;
    public static DigitalInput elevatorSwitch8ElevatorBottom;
    public static WPI_TalonSRX pickerSRX6PickerL;
    public static WPI_TalonSRX pickerSRX7PickerR;
    public static DigitalInput pickerSwitch1Picker;
    public static DoubleSolenoid pickerDoubleSolenoid1Pick;
    public static DoubleSolenoid pickerDSolenoidArticulationR;
    public static DoubleSolenoid pickerDoubleSolenoid1;
    public static WPI_TalonSRX climberSRR8Climb;
    public static DoubleSolenoid climberDoubleSolenoid2Climb;
    public static AnalogGyro navigationAnalogGyro1;
    public static Ultrasonic navigationUltrasonic1;
    public static DigitalInput navigationDigitalInput1NullZoneColor;
    public static PowerDistributionPanel utilitiesPowerDistributionPanel1;
    public static Compressor utilitiesPCMCompressor;
    public static SpeedController pickerPWMTalonSRRF;
    public static SpeedController pickerPWMTalonSRLR;
    public static SpeedController pickerPWMTalonSRLF;
    public static SpeedController pickerPWMTalonSRRR;
    public static AnalogInput pickerPWMSharpDistPickCube;
    public static SpeedControllerGroup leftDriveGroup;
    public static SpeedControllerGroup rightDriveGroup;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveSRX1DriveLR = new WPI_TalonSRX(1);
        
        
        driveSRX2DriveLF = new WPI_TalonSRX(2);
        
        
        driveSRX3DriveRR = new WPI_TalonSRX(3);
        
        
        driveSRX4DriveRF = new WPI_TalonSRX(4);
        
        leftDriveGroup = new SpeedControllerGroup(driveSRX1DriveLR,driveSRX2DriveLF);
        rightDriveGroup = new SpeedControllerGroup(driveSRX3DriveRR,driveSRX4DriveRF);
        
        driveRobotDrive41 = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
        
        
        
        driveRobotDrive41.setSafetyEnabled(true);
        driveRobotDrive41.setExpiration(0.1);
        
        driveRobotDrive41.setMaxOutput(1.0);
       
        driveQuadEncLeftDrive = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "Quad Enc LeftDrive", driveQuadEncLeftDrive);
        driveQuadEncLeftDrive.setDistancePerPulse(1.0);
        driveQuadEncLeftDrive.setPIDSourceType(PIDSourceType.kRate);
        driveQuadEncRightDrive = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive", "QuadEnc RightDrive", driveQuadEncRightDrive);
        driveQuadEncRightDrive.setDistancePerPulse(1.0);
        driveQuadEncRightDrive.setPIDSourceType(PIDSourceType.kRate);
        elevatorQuadEncElevator = new Encoder(4, 5, false, EncodingType.k4X);
        LiveWindow.addSensor("Elevator", "Quad Enc Elevator", elevatorQuadEncElevator);
        elevatorQuadEncElevator.setDistancePerPulse(1.0);
        elevatorQuadEncElevator.setPIDSourceType(PIDSourceType.kRate);
        elevatorSRX5Elevator = new WPI_TalonSRX(5);
        
        
        elevatorPIDController1 = new PIDController(1.0, 0.0, 0.0, 0.0, elevatorQuadEncElevator, elevatorSRX5Elevator, 0.02);
        LiveWindow.addActuator("Elevator", "PID Controller 1", elevatorPIDController1);
        elevatorPIDController1.setContinuous(false);
        elevatorPIDController1.setAbsoluteTolerance(0.2);

        elevatorPIDController1.setOutputRange(-1.0, 1.0);
        elevatorSwitch7ElevatorTop = new DigitalInput(7);
        LiveWindow.addSensor("Elevator", "Switch7ElevatorTop", elevatorSwitch7ElevatorTop);
        
        elevatorSwitch8ElevatorBottom = new DigitalInput(8);
        LiveWindow.addSensor("Elevator", "Switch8ElevatorBottom", elevatorSwitch8ElevatorBottom);
        
        pickerSRX6PickerL = new WPI_TalonSRX(6);
        
        
        pickerSRX7PickerR = new WPI_TalonSRX(7);
        
        
        pickerSwitch1Picker = new DigitalInput(6);
        LiveWindow.addSensor("Picker", "Switch1Picker", pickerSwitch1Picker);
        
        pickerDoubleSolenoid1Pick = new DoubleSolenoid(0, 0, 1);
        LiveWindow.addActuator("Picker", "Double Solenoid 1 Pick", pickerDoubleSolenoid1Pick);
        
        pickerDSolenoidArticulationR = new DoubleSolenoid(0, 4, 5);
        LiveWindow.addActuator("Picker", "DSolenoidArticulationR", pickerDSolenoidArticulationR);
        
        pickerDoubleSolenoid1 = new DoubleSolenoid(0, 6, 7);
        LiveWindow.addActuator("Picker", "Double Solenoid 1", pickerDoubleSolenoid1);
        
        climberSRR8Climb = new WPI_TalonSRX(8);
        
        
        climberDoubleSolenoid2Climb = new DoubleSolenoid(0, 2, 3);
        LiveWindow.addActuator("Climber", "Double Solenoid 2 Climb", climberDoubleSolenoid2Climb);
        
        navigationAnalogGyro1 = new AnalogGyro(0);
        LiveWindow.addSensor("Navigation", "AnalogGyro 1", navigationAnalogGyro1);
        navigationAnalogGyro1.setSensitivity(0.007);
        navigationUltrasonic1 = new Ultrasonic(9, 10);
        LiveWindow.addSensor("Navigation", "Ultrasonic 1", navigationUltrasonic1);
        
        navigationDigitalInput1NullZoneColor = new DigitalInput(11);
        LiveWindow.addSensor("Navigation", "Digital Input 1 Null Zone Color", navigationDigitalInput1NullZoneColor);
        
        utilitiesPowerDistributionPanel1 = new PowerDistributionPanel(18);
        LiveWindow.addSensor("Utilities", "PowerDistributionPanel 1", utilitiesPowerDistributionPanel1);
        
        utilitiesPCMCompressor = new Compressor(16);
        LiveWindow.addActuator("Utilities", "PCMCompressor", utilitiesPCMCompressor);
        
        pickerPWMTalonSRRF = new Talon(1);
        LiveWindow.addActuator("PickerPWM", "TalonSRRF", (Talon) pickerPWMTalonSRRF);
        pickerPWMTalonSRRF.setInverted(false);
        pickerPWMTalonSRLR = new Talon(2);
        LiveWindow.addActuator("PickerPWM", "TalonSRLR", (Talon) pickerPWMTalonSRLR);
        pickerPWMTalonSRLR.setInverted(false);
        pickerPWMTalonSRLF = new Talon(3);
        LiveWindow.addActuator("PickerPWM", "TalonSRLF", (Talon) pickerPWMTalonSRLF);
        pickerPWMTalonSRLF.setInverted(false);
        pickerPWMTalonSRRR = new Talon(0);
        LiveWindow.addActuator("PickerPWM", "TalonSRRR", (Talon) pickerPWMTalonSRRR);
        pickerPWMTalonSRRR.setInverted(false);
        pickerPWMSharpDistPickCube = new AnalogInput(1);
        LiveWindow.addSensor("PickerPWM", "SharpDistPickCube", pickerPWMSharpDistPickCube);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
