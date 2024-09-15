// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;




public class Wrist extends SubsystemBase {

  double kS = 0;
  double kG = 0;
  double kV = 0;

  Timer time;
  ArmFeedforward wristFF = new ArmFeedforward(kS, kG, kV);

  public static enum moveJoystick { //States to check what direction the joystick is moving
    Up,
    Down
  }

  public static enum wristPositionStates { //Different position states for the arm to move
    FLOOR,
    AMP,
    HOOK,
    SPEAKER,
    REST,
    DUMMY,
    SNIPE
  }

  public static enum WristStates { //Different states that determines what stage the arm is in.
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED
  }
  



  // motors

  static TalonFX wristMotor = new TalonFX(Constants.wristMotorPort, "CANivore1");
 


  // limit switches

  DigitalInput limitSwitch = new DigitalInput(Constants.limitPortWrist);

  // values

  double currentArmEncoderPos;
  static double AuxiliaryFF;
  static double wristAngle;


  static double targetPosition = 0;
  static wristPositionStates currentPos = wristPositionStates.REST;
  WristStates WristState = WristStates.NOT_INITIALIZED;
  static wristPositionStates positionState = wristPositionStates.REST;

  // PID
  GenericEntry pWEntry = Shuffleboard.getTab("WRIST").add("Proportional Wrist", 0).getEntry();
  GenericEntry iWEntry = Shuffleboard.getTab("WRIST").add("Integral Wrist", 0).getEntry();
  GenericEntry dWEntry = Shuffleboard.getTab("WRIST").add("Derivative Wrist",0).getEntry();
  GenericEntry FFWEntry = Shuffleboard.getTab("WRIST").add("Feed Forward Wrist",0).getEntry();
  GenericEntry WAngleEntry = Shuffleboard.getTab("WRIST").add("Wrist Angle",0).getEntry();

  // motion magic velocity and acceleration
  //GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration ", 0).getEntry();

  //GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity", 0).getEntry();

  // position
  GenericEntry currentPosWEntry = Shuffleboard.getTab("WRIST").add("Current Position Wrist", 0).getEntry();
  GenericEntry targetPosWEntry = Shuffleboard.getTab(  "WRIST").add("Target Position Wrist", 0).getEntry();
  GenericEntry PosErrWEntry = Shuffleboard.getTab("WRIST").add("Pos Err Wrist", 0).getEntry();

  GenericEntry kSEntry = Shuffleboard.getTab("WRIST").add("kS", 0).getEntry();
  GenericEntry kGEntry = Shuffleboard.getTab("WRIST").add("kG", 0).getEntry();
  GenericEntry kVEntry = Shuffleboard.getTab("WRIST").add("kV", 0).getEntry();


  // states

  GenericEntry wristStateEntry = Shuffleboard.getTab("Game").add("Wrist State",  WristState.toString()).getEntry();

  // limit switch
  
  GenericEntry limitSwitchWEntry = Shuffleboard.getTab("WRIST").add("Limit Switch Wrist", true).getEntry();


  GenericEntry stateEntry = Shuffleboard.getTab("WRIST").add("state",0).getEntry();
  MotionMagicVoltage m_position = new MotionMagicVoltage(0);
  

  public Wrist() {
   
   
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    var slot0Configs = talonFXConfigs.Slot0; 
    
    //tune PID and also tune configurations

    slot0Configs.kP = 30;
    slot0Configs.kI = 0; 
    slot0Configs.kD = 0;
    slot0Configs.kS = 0;
    slot0Configs.kG = 0;
    slot0Configs.kV = 0; 
    
    motionMagicConfigs.MotionMagicAcceleration = 70; 
    motionMagicConfigs.MotionMagicCruiseVelocity = 75; 

    wristMotor.getConfigurator().apply(talonFXConfigs);
    // targetPosition=0;
    // wristMotor.setPosition(0);
    // wristMotor.setControl(m_position.withPosition(0));
  }

  public double getPos() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public void setPos(double pos) {
    wristMotor.setPosition(pos);
  }

  // public void changePID (double KP, double KI, double KD){
  //   TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  //   var slot0Configs = talonFXConfigs.Slot0;
  //   slot0Configs.kP = KP; 
  //   slot0Configs.kI = KI; 
  //   slot0Configs.kD = KD; 
  // }

  public void movePos(double pos) {
    wristMotor.setControl(m_position.withPosition(pos * Constants.gearRatioWrist));
    //wristMotor. (m_position.withFeedForward(AuxiliaryFF));
    targetPosition = pos;
    setTargetPos(targetPosition);
    targetPosWEntry.setDouble(targetPosition);
  }

  public void setWristState(WristStates state) {
    WristState = state; 
  }

  public void setWristPositionState(wristPositionStates state) {
    positionState = state;
  }

  public void reset() {
    setPos(0);
    movePos(0);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public double returnTargetPos() {
    return targetPosition;
  }

  // public double getVelocity() {
  //   return velEntry.getDouble(0);
  // }

  // public void setAcceleration(double val) { 
  //   accelEntry.setDouble(val);
  // }

  // public double getAccel() {
  //   return accelEntry.getDouble(0);
  // }

  public static wristPositionStates armPlacement(double ctrlValue) {

    if (ctrlValue <= -0.95 && ctrlValue >= -0.98)
      return wristPositionStates.FLOOR;
    else if (ctrlValue <= -0.06 && ctrlValue >= -0.1)
      return wristPositionStates.AMP;
    else if (ctrlValue <= -0.53 && ctrlValue >= -0.56)
      return wristPositionStates.REST; 
    else if (ctrlValue <= -0.27 && ctrlValue >= -0.29)
      return wristPositionStates.SPEAKER;
    else
      return currentPos;
  }

  public static wristPositionStates WristgetPositionState() {
    return currentPos;
  }

  public WristStates getWristState() {
    return WristState;
  }

  public void setTargetPos(double pos) {
    targetPosition = pos;
  }

  public void runMotor() {
    wristMotor.setControl(new DutyCycleOut(0.0));
  }

  public void disableMotor() {
    wristMotor.disable();
  }

  public void setFF(double val) {
    FFWEntry.setDouble(val);
  }

  @Override
  public void periodic() {
    //System.out.println("WRIST"+ getWristState());
    //change the feed forward to correspond the wrist formula
    wristAngle = -34 + ((getPos()*360/Constants.gearRatioWrist) - (RobotContainer.m_armLift.getPos()*360/Constants.gearRatioArm)); //try removing gear ratios
    AuxiliaryFF = /*FFWEntry.getDouble(-0.15)*/ -0.15 * Math.sin(Math.toRadians(wristAngle)); //-0.128 
    wristMotor.setControl(m_position.withFeedForward(AuxiliaryFF));
    
    //System.out.println(WristgetPositionState());
    //FFWEntry.setDouble(AuxiliaryFF);
    // wristFF = new ArmFeedforward(kS, kG, kV);

    WAngleEntry.setDouble(wristAngle);
    limitSwitchWEntry.setBoolean(getLimitSwitch());
    currentPosWEntry.setDouble(getPos()/72.0167590726);
    PosErrWEntry.setDouble(targetPosWEntry.getDouble(0) - currentPosWEntry.getDouble(0));
    //targetPosWEntry.setDouble(targetPosition*Constants.gearRatioWrist);

    wristStateEntry.setString(WristState.toString());

    if(DriverStation.isEnabled()) {
    switch(WristState) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:
          currentPos = armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ENCODER_PORT));
        }
      }
    }

 
  }
