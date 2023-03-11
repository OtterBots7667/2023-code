// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 *  the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // private MotorControllerGroup rightDrives = new MotorControllerGroup( new WPI_VictorSPX(), new WPI_VictorSPX());
  // private MotorControllerGroup leftDrives = new MotorControllerGroup( new WPI_VictorSPX(), new WPI_VictorSPX());
  // private TalonFX pivot = new motorController(new TalonFX());                                     //Replace Victors with the
  // private VictorSPX FAslide = new motorController(new VictorSPX()); //FAslide = Fore-Aft Slide    //correct motor controllers
  // private VictorSPX armExtension = new motorController(new VictorSPX());                          //

  private DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid leftGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private DoubleSolenoid rightGrabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  int teleopCounter = 0;

  private Joystick joystickButtons = new Joystick(0);
  private Joystick joystickDriver = new Joystick(1);
  private Joystick joystickManual = new Joystick(2);

Double pivotPos = 0.0;
Double armExtensionPos = 0.0;
Double FAslidePos = 0.0;
Boolean pivotGood = false;
Boolean armExtensionGood = false;
Boolean FAslideGood = false;

Boolean posGroundButton = joystickButtons.getRawButton(8);//Low Yellow
Boolean posMidButton = joystickButtons.getRawButton(7);//Low-Mid Yellow
Boolean posHighButton = joystickButtons.getRawButton(6);//High-Mid Yellow
Boolean coneButton = joystickButtons.getRawButton(2);//X-Blue
Boolean cubeButton = joystickButtons.getRawButton(3);//B-Red
Boolean releaseButton = joystickButtons.getRawButton(4);//A-Green
Boolean brakeButton = joystickButtons.getRawButton(1);//Y-Yellow
Boolean goingGround = false;
Boolean goingMid = false;
Boolean goingHigh = false;
Boolean grabbingCone = false;
Boolean grabbingCube = false;

// public void positionSetter(String position) {
// if(position == "ground" && goingGround){
//   goingMid = false;
//   goingHigh = false;
//   goingPickUp = false;

// } 
// if(position == "mid" && goingMid){
//   goingGround = false;
//   goingHigh = false;
//   goingPickUp = false;

// } 
// if(position == "high" && goingHigh){
//   goingGround = false;
//   goingMid = false;
//   goingPickUp = false;

// } 
// if(position == "pickUp" && goingPickUp){
//   goingGround = false;
//   goingMid = false;
//   goingHigh = false;

// }

// }

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  // DRIVE CODE
  double leftStick = joystickDriver.getRawAxis(1);
  double rightStick = joystickDriver.getRawAxis(5);
  leftStick = leftStick * -1;

// squares the motor power; easier to use low speeds, but high speed is uneffected
  if(leftStick > 0 ){
  leftStick = leftStick * leftStick;
  }else if (leftStick < 0){
    leftStick = leftStick * -leftStick;
  }
  if(rightStick > 0 ){
    rightStick = rightStick * rightStick;
    }else if (rightStick < 0){
      rightStick = rightStick * -rightStick;
    }
  
if(leftStick > -0.05 && leftStick < 0.05){
  leftStick = 0.0;
}                                               //Controller Deadzone up to 22.4%
if(rightStick > -0.05 && rightStick < 0.05){
  rightStick = 0.0;
}

  // leftDrives.set(leftStick);
  
  // // if you press A, all wheels are controled by left stick (makes sure that you drive straight)
  // if(joystickDriver.getRawButton(4) || joystickDriver.getRawButton(1)){
  //   rightDrives.set(-leftStick);
  // }else{
  // rightDrives.set(rightStick);
  // }

  
  //Grabber Code (っ◕‿◕)っ ❒  ⃤

  if(coneButton){
    grabbingCube = false;
    if(!grabbingCone){teleopCounter = 0;}
    grabbingCone = true;
  }
  
  if(cubeButton){
    grabbingCone = false;
    if(!grabbingCube){teleopCounter = 0;}
    grabbingCube = true;
  }

  if(releaseButton){
    grabbingCone = false;
    grabbingCube = false;
    rightGrabber.set(DoubleSolenoid.Value.kReverse);
    leftGrabber.set(DoubleSolenoid.Value.kReverse);
  }

if(grabbingCone){
  if(teleopCounter<5){
  rightGrabber.set(DoubleSolenoid.Value.kForward);
  leftGrabber.set(DoubleSolenoid.Value.kForward);
  }else{
    rightGrabber.set(DoubleSolenoid.Value.kOff);
    leftGrabber.set(DoubleSolenoid.Value.kOff);
    grabbingCone = false;
  }
}

if(grabbingCube){
  if(teleopCounter<3){
    rightGrabber.set(DoubleSolenoid.Value.kForward);
    leftGrabber.set(DoubleSolenoid.Value.kForward);
    }else{
      rightGrabber.set(DoubleSolenoid.Value.kOff);
      leftGrabber.set(DoubleSolenoid.Value.kOff);
      grabbingCube = false;
    }
}


if(posGroundButton) {
}

if(posMidButton) {
  goingMid = true;
}

if(posHighButton) {
  goingHigh = true;
}

// if(joystickButtons.getRawButton(posHighButton)) {
//   goingPickUp = true;
// }

// pivotPos = pivot.getSelectedSensorPosition;
// armExtensionPos = armExtension.getSelectedSensorPosition;
// FAslidePos = FAslide.getSelectedSensorPosition;

// if(goingGround){
//   goingMid = false;
//   goingHigh = false;
//   // goingPickUp = false;
//   if(pivotPos<){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, -0.5);
//   }else{
//     pivot.set(controlMode.PercentOutput, 0);
//     pivotGood = true;
//   }

//   if(armExtensionPos<){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, -0.5);
//   }else{
//     armExtension.set(controlMode.PercentOutput, 0);
//     armExtensionGood = true;
//   }

//   if(FAslideGood<){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, -0.5);
//   }else{
//     FAslide.set(controlMode.PercentOutput, 0);
//     FAslideGood = true;
//   }
//   if(pivotGood && armExtensionGood && FAslideGood){
// pivotGood = false;
// armExtensionGood = false;
// FAslideGood = false;
// goingGround = false;
//   }
// } 
// if(goingMid){
//   goingGround = false;
//   goingHigh = false;
//   // goingPickUp = false;
//   if(pivotPos<){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, -0.5);
//   }else{
//     pivot.set(controlMode.PercentOutput, 0);
//     pivotGood = true;
//   }

//   if(armExtensionPos<){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, -0.5);
//   }else{
//     armExtension.set(controlMode.PercentOutput, 0);
//     armExtensionGood = true;
//   }

//   if(FAslide<){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, -0.5);
//   }else{
//     FAslide.set(controlMode.PercentOutput, 0);
//     FAslideGood = true;
//   }
//   if(pivotGood && armExtensionGood && FAslideGood){
// pivotGood = false;
// armExtensionGood = false;
// FAslideGood = false;
// goingMid = false;
//   }
// } 

// if(goingHigh){
//   goingGround = false;
//   goingMid = false;
//   // goingPickUp = false;
//   if(pivotPos<){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     pivotGood = false;
//     pivot.set(controlMode.PercentOutput, -0.5);
//   }else{
//     pivot.set(controlMode.PercentOutput, 0);
//     pivotGood = true;
//   }

//   if(armExtensionPos<){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     armExtensionGood = false;
//     armExtension.set(controlMode.PercentOutput, -0.5);
//   }else{
//     armExtension.set(controlMode.PercentOutput, 0);
//     armExtensionGood = true;
//   }

//   if(FAslidePos<){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, 0.5);
//   }else if(pivotPos>){
//     FAslideGood = false;
//     FAslide.set(controlMode.PercentOutput, -0.5);
//   }else{
//     FAslide.set(controlMode.PercentOutput, 0);
//     FAslideGood = true;
//   }
//   if(pivotGood && armExtensionGood && FAslideGood){
// pivotGood = false;
// armExtensionGood = false;
// FAslideGood = false;
// goingHigh = false;
//   }
// } 


teleopCounter++;

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}