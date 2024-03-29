// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 *  the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "NO AUTO";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private MotorControllerGroup rightDrives = new MotorControllerGroup( new WPI_VictorSPX(3), new WPI_VictorSPX(7));
  private MotorControllerGroup leftDrives = new MotorControllerGroup( new WPI_VictorSPX(4), new WPI_VictorSPX(1));
  private TalonFX pivot = new TalonFX(5);                                     //Replace Victors with the
  private CANSparkMax FASlide = new CANSparkMax(6,MotorType.kBrushless); //FAslide = Fore-Aft Slide
  private RelativeEncoder FAEncoder = FASlide.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private VictorSPX armExtension = new VictorSPX(2);
  private PIDController pidf = new PIDController(0, 0, 0);
  // private Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

  

  DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,5,4);
  DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,7,6);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  int teleopCounter = 0;

  private Joystick joystickButtons = new Joystick(0);
  private Joystick joystickDriver = new Joystick(1);
  private Joystick joystickManual = new Joystick(2);

  Double integral = 0.0;
 Double previous_error = 0.0;
 Double setpoint = 0.0;
 int safty = 0;

 int resetClock = 0;
 int restCount = 0;
 Double restVar1 = 0.0;
 int restVar2 = 0;

Boolean posGroundButton = false;
Boolean posMidButton = false;
Boolean posHighButton = false;
Boolean posShelfButton = false;
Boolean posRestButton = false;
Boolean grabberButton = false;
Boolean releaseButton = false;
Boolean manualButton = false;
Double  manualStick = 0.0;
Boolean manualFASlideButton = false;
Boolean manualExtensionButton = false;
Boolean manualPivotButton = false;

Boolean goGround = false;
Boolean goMid = false;
Boolean goHigh = false;
Boolean goShelf = false;
Boolean goRest = false;
Boolean goManual = false;

Boolean highPosBypass = false;
Boolean midPosBypass = false;
Boolean shelfPosBypass = false;

Double redness = 0.0;
Double blueness = 0.0;
Double greenness = 0.0;
String posColor = null;
Boolean colorDisabled = false;

Boolean auto = false;
Boolean noTarget = true;
Double manualTimer = 0.0;
Boolean manualTT1 = false;  // manual target timer variables
Double manualTT2 = 0.0;
Double manualTT3 = 0.0;

Double p = 0.00008;
Double i = 0.00002;
Double d = 0.00001;
// Double f = 0.08;
Double target = 0.0;
Double armPos = 0.0;
Double pidConstant = 81920.0/360.0;
Double time = 0.0;
Double pidPowerLimitUp = 0.16;
Double pidPowerLimitDown = 0.16;
int autoTime1 = 0;
int autoTime2 = 0;
Boolean highPowerHighPos = false;
Boolean stopPowerGoingDown = false;
Double pid = 0.0;
Double pidRaw = 0.0;
int countThing = 0;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    FAEncoder.setPosition(0.0);
    pivot.setSelectedSensorPosition(0.0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //PID using WPILIB PID method

pidf.setPID(p, i, d);
armPos = (pivot.getSelectedSensorPosition());
pidRaw = pidf.calculate(armPos, target);

  pidPowerLimitDown = 0.25;
  pidPowerLimitUp = 0.25;

  if(goMid || goShelf || goHigh){
    if((pivot.getSelectedSensorPosition() > target -800) && (pivot.getSelectedSensorPosition() < target + 800)){
 if(countThing > 25){
      pid = pid;
 }
  countThing ++;
    }else{
      pid = pidRaw;
      countThing = 0;
    }
  }else{
pid = pidRaw;
countThing = 0;
  }

if(pid > pidPowerLimitUp){
  pid = pidPowerLimitUp;
}else if(pid < -pidPowerLimitDown){
  pid = -pidPowerLimitDown;
}
if(!noTarget){        //In rest position, PID position updating is disabled
pivot.set(TalonFXControlMode.PercentOutput, pid);
}else{
  target = 0.0;
}

SmartDashboard.putNumber("PID target", target);
SmartDashboard.putNumber("PID_target", target);
SmartDashboard.putNumber("Arm Position", armPos);



//Arm extension uses a color sensor to sense the extender's position
    Color detectedColor = colorSensor.getColor();

    redness = detectedColor.red*1.3;    // Red and Blue are given an extra push
    blueness = detectedColor.blue*1.3;  // because the sensor is more sensitive to green
    greenness = detectedColor.green;
    
    SmartDashboard.putNumber("Rojo: ", redness);

    SmartDashboard.putNumber("azul: ", blueness);

    SmartDashboard.putNumber("verde: ", greenness);

if(blueness == 0.0){
  colorDisabled = true;
  System.out.println("COLOR SENSOR IS DISABLED");
}

if(redness<0.4 && blueness<0.35 && greenness<0.55){
posColor = "gray";
  }else{
    if(redness >= blueness){
      if(greenness >= redness){
        if(greenness > 0.0){        // blue is all the way retracted
          posColor = "green";       // green is shelf position
        }                           // red is high position
      }else{                        // gray means it's part-way through an extension/retraction
        posColor = "red";
      }
    }else{
      posColor = "blue";
    }
  }

    SmartDashboard.putString("Color: ", posColor);

    SmartDashboard.putNumber("FASlide position", FAEncoder.getPosition());

    SmartDashboard.putNumber("FalconPosition (degrees)", pivot.getSelectedSensorPosition()/228);
    SmartDashboard.putNumber("FalconPosition", pivot.getSelectedSensorPosition());

 posGroundButton = joystickButtons.getRawButton(1);//Low Yellow
 posMidButton = joystickButtons.getRawButton(2);//Low-Mid Yellow
 posHighButton = joystickButtons.getRawButton(3);//High-Mid Yellow
 posShelfButton = joystickButtons.getRawButton(4);//High Yellow
 posRestButton = joystickButtons.getRawButton(7);//Y-Yellow
 grabberButton = joystickButtons.getRawButton(12);//X-Blue
 releaseButton = joystickButtons.getRawButton(11);//B-Red
//  manualButton = joystickManual.getRawButton(11);//Switch Up
//  manualStick = joystickManual.getRawAxis(1);
//  manualFASlideButton = joystickManual.getRawButton(2);
//  manualExtensionButton = joystickManual.getRawButton(3);
//  manualPivotButton = joystickManual.getRawButton(4);

// SmartDashboard.updateValues();

 time++;
  }

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
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", "awdbsKJ");
    // System.out.println("Auto selected: " + m_autoSelected);

    // auto = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if(autoTime1 < 300){
//Go to mid position
noTarget = false;
goMid = true;
    }else if(autoTime1 < 350){
      grabber.set(kReverse);
    }else if(autoTime1 <525){
goMid = false;
noTarget = true;
goRest = true;
    }else{
leftDrives.set(-0.3);
rightDrives.set(0.3);
    }


// Go to rest posiotion

//Drive backwards

if(goRest){
  if(posColor != "blue"){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
        if(FAEncoder.getPosition()>-0.5){
          FASlide.set(0.0);
          if(pivot.getSelectedSensorPosition() < -500.0){
          pivot.set(TalonFXControlMode.PercentOutput, 0.20);
          }else{
            pivot.set(TalonFXControlMode.PercentOutput, 0.1);
          }
          if((restCount + 6)%6 == 0){
          restVar2 = restCount;
          restVar1 = pivot.getSelectedSensorPosition();
          }
          restPosChange(restVar2, restVar1);
          restCount++;
        }else{
          FASlide.set(0.25);
        }
      }
}



if(goMid){
  if(posColor != "blue"){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
           target = -8400.0;
            if(FAEncoder.getPosition()<-32.5){
              FASlide.set(0.0);
            }else{
              FASlide.set(-0.25);
            } 
      }
}


// if(goHigh){
//   if(posColor != "blue"&&!highPosBypass){
//     armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
//       }else{
//         if(!highPosBypass){
//         armExtension.set(VictorSPXControlMode.PercentOutput, 0);
//         }
//         target = -10000.0;
//         if(FAEncoder.getPosition()<-32.5){
//           FASlide.set(0.0);
//         }else{
//           FASlide.set(-0.25);
//         }
//           highPosBypass = true;
//           if(posColor != "red"){
//             armExtension.set(VictorSPXControlMode.PercentOutput, 0.3);
//             if(posColor == "green"){
//             }
//           }else{
//             armExtension.set(VictorSPXControlMode.PercentOutput, 0);
//           }
//         }
// }

autoTime1++;




//     switch (m_autoSelected) {
//       case kCustomAuto:
//         // Put custom auto code here
// System.out.println("NO AUTO");
//       if(autoTime2 < 150){

//         leftDrives.set(-0.3);
//         rightDrives.set(0.3);
//       }
//       //left negative
//       //right positive
// autoTime2++;
//         break;
//       case kDefaultAuto:
//       default:
//         // Put default auto code here

//         //Go to high position

//         grabber.set(kReverse);

//         // Go to rest posiotion

//         //Drive backwards

//         System.out.println("YES AUTO");


// autoTime1++;
//         break;
//     }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    auto = false;
  }

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

  leftDrives.set(leftStick);
  
  // if you press A, all wheels are controled by left stick (makes sure that you drive straight)
  if(joystickDriver.getRawButton(4) || joystickDriver.getRawButton(1)){
    rightDrives.set(-leftStick);
  }else{
  rightDrives.set(rightStick);
  }


  // Grabber Code - a pair of pneumatics cylanders actuated by a single solanoid

if(grabberButton){
grabber.set(kForward);
}else if(releaseButton){
  grabber.set(kReverse);
}

  if(manualButton){
  noTarget = false;
  highPosBypass = false;
  shelfPosBypass = false;
  goRest = false;
  goShelf = false;
  goHigh = false;
  goMid = false;
  goGround = false;
  goManual = true;
}else if(posRestButton){
  noTarget = true;
  highPosBypass = false;
  shelfPosBypass = false;
  goRest = true;
  goShelf = false;
  goHigh = false;
  goMid = false;
  goGround = false;
  goManual = false;
}else if(posShelfButton){
  noTarget = false;
  highPosBypass = false;
  shelfPosBypass = false;
  goRest = false;
  goShelf = true;
  goHigh = false;
  goMid = false;
  goGround = false;
  goManual = false;
}else if(posHighButton){
  noTarget = false;
  shelfPosBypass = false;
  goRest = false;
  goShelf = false;
  goHigh = true;
  goMid = false;
  goGround = false;
  goManual = false;
}else if(posMidButton){
  noTarget = false;
  highPosBypass = false;
  shelfPosBypass = false;
  goRest = false;
  goShelf = false;
  goHigh = false;
  goMid = true;
  goGround = false;
  goManual = false;
}else if(posGroundButton){
  noTarget = false;
  highPosBypass = false;
  shelfPosBypass = false;
  goRest = false;
  goShelf = false;
  goHigh = false;
  goMid = false;
  goGround = true;
  goManual = false;
}

if(!goRest){
  resetClock = 0;
  restCount = 0;
  restVar1 = 0.0;
  restVar2 = 0;
}

if(!manualPivotButton){
  manualTT1 = true;
}

// Manual override can be used if the robot behaves unexpectedly, if any of the sensors/encoders
// aren't working, or if the robot needs to do something beyond its usual functions.

if(goManual){
  if(manualExtensionButton){
    armExtension.set(VictorSPXControlMode.PercentOutput, manualStick*0.5);
    FASlide.set(0.0);
  }else if(manualFASlideButton){
    FASlide.set(manualStick*0.35);
    armExtension.set(VictorSPXControlMode.PercentOutput, 0.0);
  }else if(manualPivotButton){

if(manualStick > 0.5){
    manualTimer = 120.0;
}else if(manualStick < -0.5){
manualTimer = -120.0;
}else{
  manualTimer = 0.0;
}

    target = target + manualTimer;

    armExtension.set(VictorSPXControlMode.PercentOutput, 0.0);
    FASlide.set(0.0);
  }else{
    armExtension.set(VictorSPXControlMode.PercentOutput, 0.0);
    FASlide.set(0.0);
  }
}

if(goRest){
  if((posColor != "blue") && !colorDisabled){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
        if(FAEncoder.getPosition()>-0.5){
          FASlide.set(0.0);
          if(pivot.getSelectedSensorPosition() < -500.0){
          pivot.set(TalonFXControlMode.PercentOutput, 0.20);
          }else{
            pivot.set(TalonFXControlMode.PercentOutput, 0.1);
          }
          if((restCount + 6)%6 == 0){
          restVar2 = restCount;
          restVar1 = pivot.getSelectedSensorPosition();
          }
          restPosChange(restVar2, restVar1);
          restCount++;
        }else{
          FASlide.set(0.25);
        }
      }
}

if(goShelf && !colorDisabled){
  if(posColor != "blue" && !shelfPosBypass){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        if(!shelfPosBypass){
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
        }
        if(FAEncoder.getPosition()>-1||shelfPosBypass){
          if(!shelfPosBypass){
          FASlide.set(0.0);
          }
          target =-12500.0;
          shelfPosBypass = true;
          if(posColor != "green" && !colorDisabled){
            armExtension.set(VictorSPXControlMode.PercentOutput, 0.3);
          }else{
            armExtension.set(VictorSPXControlMode.PercentOutput, 0.0);
          }

        }else{
          FASlide.set(0.25);
        }
      }
}

if(goGround){
  if(posColor != "blue" && !colorDisabled){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
        if(FAEncoder.getPosition()>-0.5){
          FASlide.set(0.0);
          target = -27000.0;
        }else{
          FASlide.set(0.25);
        }
      }
}

if(goMid){
  if(posColor != "blue" && !colorDisabled){
    armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
      }else{
        armExtension.set(VictorSPXControlMode.PercentOutput, 0);
           target = -11400.0;
            if(FAEncoder.getPosition()<-32.5){
              FASlide.set(0.0);
            }else{
              FASlide.set(-0.25);
            } 
      }
}

// if(goHigh){
//   if(posColor != "blue" && !highPosBypass){
//     armExtension.set(VictorSPXControlMode.PercentOutput, -0.45);
//       }else{
//         if(!highPosBypass){
//         armExtension.set(VictorSPXControlMode.PercentOutput, 0);
//         }
//         target = -9000.0;
//         if(FAEncoder.getPosition()<-32.5){
//           FASlide.set(0.0);
//         }else{
//           FASlide.set(-0.25);
//         }
//           highPosBypass = true;
//           if(posColor != "red"){
//             armExtension.set(VictorSPXControlMode.PercentOutput, 0.3);
//             if(posColor == "green"){
//             }
//           }else{
//             armExtension.set(VictorSPXControlMode.PercentOutput, 0);
//           }
//       }
//   //make sure arm is retracted
//   //make sure FASlide is all the way back
//   //set pivot angle
//   //extend
//   //set FASlide
// }

//Brake - helps prevent falling off the charge station

// if(joystickButtons.getRawButton(2) && joystickButtons.getRawAxis(1)<-0.5){
// brake.set(kForward);
// System.out.println("Brake Deployed");
// }else if(joystickButtons.getRawButton(2) && joystickButtons.getRawAxis(1)>0.5){
//   brake.set(kReverse);
//   System.out.println("Brake Retracted");
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

  public void restPosChange(int thing1, Double thing2){

    if((thing1 + 5) == restCount){

      if((thing2 > pivot.getSelectedSensorPosition()-10) && (thing2 < pivot.getSelectedSensorPosition()+10)){
            
        pivot.setSelectedSensorPosition(0.0);
      }
    }
  }
}