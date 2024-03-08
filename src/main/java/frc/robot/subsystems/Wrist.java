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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;




public class Wrist extends SubsystemBase {

  Timer time;

  public static enum moveJoystick { //States to check wwhat direction the joystick is moving
    Up,
    Down
  }

  public static enum WristpositionStates { //Different position states for the arm to move
    FLOOR,
    AMP,
    HOOK,
    SPEAKER,
    REST,
    DUMMY
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


  static double targetPostion = 0;
  static WristpositionStates currentPos = WristpositionStates.REST;
  WristStates WristState = WristStates.NOT_INITIALIZED;
  static WristpositionStates positionState = WristpositionStates.REST;

  // PID
  // GenericEntry pWEntry = Shuffleboard.getTab("WRIST").add("Proportional Wrist", 0).getEntry();
  // GenericEntry iWEntry = Shuffleboard.getTab("WRIST").add("Integral Wrist", 0).getEntry();
  // GenericEntry dWEntry = Shuffleboard.getTab("WRIST").add("Derivative Wrist",0).getEntry();
  GenericEntry FFWEntry = Shuffleboard.getTab("WRIST").add("Feed Forward Wrist",0).getEntry();
  GenericEntry WAngleEntry = Shuffleboard.getTab("WRIST").add("Wrist Angle",0).getEntry();

  // motion magic velocity and acceleration
  //GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration ", 0).getEntry();

  //GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity", 0).getEntry();

  // position
  GenericEntry currentPosWEntry = Shuffleboard.getTab("WRIST").add("Current Position Wrist", 0).getEntry();
  GenericEntry targetPosWEntry = Shuffleboard.getTab(  "WRIST").add("Target Position Wrist", 0).getEntry();
  GenericEntry PosErrWEntry = Shuffleboard.getTab("WRIST").add("Pos Err Wrist", 0).getEntry();

  // states

  GenericEntry wristStateEntry = Shuffleboard.getTab("Game").add("Wrist State", 0).getEntry();

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

    slot0Configs.kV = 0; 
    
    motionMagicConfigs.MotionMagicAcceleration = 50; 
    motionMagicConfigs.MotionMagicCruiseVelocity = 55; 

    wristMotor.getConfigurator().apply(talonFXConfigs);
    // targetPostion=0;
    // wristMotor.setPosition(0);
    // wristMotor.setControl(m_position.withPosition(0));
  }

  public double getPos() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public void setPos(double pos) {
    wristMotor.setPosition(pos);
  }

  public void changeP(double KP){
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = KP; 
  }

  public void movePos(double pos) {
    wristMotor.setControl(m_position.withPosition(pos * Constants.gearRatioWrist));
    //wristMotor.setControl(m_position.withFeedForward(AuxiliaryFF));
    targetPostion = pos;
    setTargetPos(targetPostion);
    targetPosWEntry.setDouble(targetPostion);
  }

  public void setWristState(WristStates state) {
    WristState = state; 
  }

  public void setWristPositionState(WristpositionStates state) {
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
    return targetPostion;
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

  public static WristpositionStates armPlacement(double ctrlValue) {

    if (ctrlValue <= -0.95 && ctrlValue >= -0.98)
      return WristpositionStates.AMP;
    else if (ctrlValue <= -0.06 && ctrlValue >= -0.1)
      return WristpositionStates.REST;
    else if (ctrlValue <= -0.53 && ctrlValue >= -0.56)
      return WristpositionStates.SPEAKER; 
    else if (ctrlValue <= -0.27 && ctrlValue >= -0.29)
      return WristpositionStates.REST;
    else if (ctrlValue == 1.00)
      return WristpositionStates.FLOOR;
    else
      return currentPos;
  }

  public static WristpositionStates WristgetPositionState() {
    return currentPos;
  }

  public WristStates getWristState() {
    return WristState;
  }

  public void setTargetPos(double pos) {
    targetPostion = pos;
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
    AuxiliaryFF = -0.128 * Math.sin(Math.toRadians(wristAngle)); 
    wristMotor.setControl(m_position.withFeedForward(AuxiliaryFF));


    
    //System.out.println(WristgetPositionState());
    FFWEntry.setDouble(AuxiliaryFF);
    WAngleEntry.setDouble(wristAngle);
    limitSwitchWEntry.setBoolean(getLimitSwitch());
    currentPosWEntry.setDouble(getPos());
    PosErrWEntry.setDouble(targetPosWEntry.getDouble(0) - currentPosWEntry.getDouble(0));
    targetPosWEntry.setDouble(targetPostion*Constants.gearRatioWrist);

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
