// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

  Timer time;

  public static enum moveJoystick { //States to check what direction the joystick is moving
    Up,
    Down
  }

  public static enum armPositionStates { //Different position states for the arm to move
    FLOOR,
    AMP,
    HOOK,
    SPEAKER,
    REST,
    DUMMY,
    SNIPE
  }

  public static enum ArmLiftStates { //Different states that determines what stage the arm is in.
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED
  }
  
  //Configurations:


  // motors

  TalonFX leftArmMotor = new TalonFX(Constants.leftmotorPort);
  TalonFX rightArmMotor = new TalonFX(Constants.rightmotorPort);



  // limit switches

  DigitalInput limitSwitch = new DigitalInput(Constants.limitPort);

  // values
  static double armAngle;
  static double armWristAngle;
  static double effectiveArmLength;
  static double armOffsetAngle;
  static double effectiveArmMass;
  static double targetPos;

  double currentArmEncoderPos;
  double AuxiliaryFF;
  static armPositionStates currentPos = armPositionStates.REST;
  ArmLiftStates ArmLiftState = ArmLiftStates.INITIALIZING;
  static armPositionStates positionState = armPositionStates.REST;

  // PID
  // GenericEntry pEntry = Shuffleboard.getTab("ARM").add("Arm Proportional", 0).getEntry();
  // GenericEntry iEntry = Shuffleboard.getTab("ARM").add("Arm Integral", 0).getEntry();
  // GenericEntry dEntry = Shuffleboard.getTab("ARM").add("Arm Derivative",0).getEntry();
  GenericEntry FFEntry = Shuffleboard.getTab("ARM").add("Arm Feed Forward",0).getEntry();


  // motion magic velocity and acceleration
  // GenericEntry accelEntry = Shuffleboard.getTab("ARM").add("Arm Acceleration", 0).getEntry();
  // GenericEntry velEntry = Shuffleboard.getTab("ARM").add("Arm Velocity", 0).getEntry();

  // position
  GenericEntry currentPosEntry = Shuffleboard.getTab("ARM").add("Arm Current Position", 0).getEntry();
  GenericEntry targetPosEntry = Shuffleboard.getTab(  "ARM").add("Arm Target Position", 0).getEntry();
  GenericEntry PosErrEntry = Shuffleboard.getTab("ARM").add("Arm Pos Err", 0).getEntry();

  // state

  GenericEntry armStateEntry = Shuffleboard.getTab("Game").add("Arm State", ArmLiftState.toString()).getEntry();
  GenericEntry stateEncoderEntry = Shuffleboard.getTab("Game").add("State Encoder", positionState.toString()).getEntry();

  // limit switch
  
  GenericEntry limitSwitchEntry = Shuffleboard.getTab("ARM").add("Arm Limit Switch", true).getEntry();

  // requests
  final MotionMagicVoltage m_position = new MotionMagicVoltage(0);

  public Arm() {

    //PID/Velocity configurations make sure that the motionMagic and slot0 configs are under var!
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    var slot0Configs = talonFXConfigs.Slot0;    

    slot0Configs.kP = 6; //Proportional 2.5 seems good for actual robot
    slot0Configs.kI = 0; //Integral
    slot0Configs.kD = 0; //Derivative
    slot0Configs.kV = 0; //Velocity
    
    motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration 
    motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target velocity

    leftArmMotor.getConfigurator().apply(talonFXConfigs);


    //rightArmMotor.setControl(new Follower(Constants.leftmotorPort, true)); //Follower master the rightArmMotor is reversed b/c double linked arm
    Follower rightFollower = new Follower(Constants.leftmotorPort, true);
    //rightFollower.withUpdateFreqHz(20);
    rightArmMotor.setControl(rightFollower);
    leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
    rightArmMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getPos() {
    return leftArmMotor.getPosition().getValueAsDouble();
  }

  public void setPos(double pos) {
    leftArmMotor.setPosition(pos);
  }

  // public void setSpeed(double speed) { Doesn't work
  //   leftArmMotor.set(speed);
  // }

  public void movePos(double pos) {
    leftArmMotor.setControl(m_position.withPosition(pos * Constants.gearRatioArm)); //
    //leftArmMotor.setControl(m_position.withFeedForward(AuxiliaryFF));
    targetPos = pos;
    targetPosEntry.setDouble(targetPos);
    
  }

  public void setArmLiftState(ArmLiftStates state) {
    ArmLiftState = state; 
  }
  public double returnTargetPos() {
    return targetPos;
  } 
  public void reset() {
    setPos(0);
    movePos(0);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  // public void setVelocity(double val) { Doesn't work
  //   leftArmMotor.setControl(m_velocity.withVelocity(val));
    // var motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = val;
    // leftArmMotor.getConfigurator().apply(motionMagicConfigs);
  // }

  // public void setAcceleration(double val) { Doesn't work
  //   var motionMagicConfigs = new MotionMagicConfigs();
  //   motionMagicConfigs.MotionMagicAcceleration = val;
  //   leftArmMotor.getConfigurator().apply(motionMagicConfigs);
  // }

  // public double getVelocity() {
  //   return velEntry.getDouble(0);
  // }

  // public void setAcceleration(double val) { 
  //   accelEntry.setDouble(val);
  // }

  // public double getAccel() {
  //   return accelEntry.getDouble(0);
  // }

  public void setArmLiftPositionState(armPositionStates state) {
    positionState = state;
  }
  public static armPositionStates armPlacement(double ctrlValue) { //Checks the guh??!!??!
    // if (ctrlValue <= 0.05 && ctrlValue >= 0) 
    //   return armPositionStates.HOOK; what hook
   if (ctrlValue <= -0.95 && ctrlValue >= -0.98)
      return armPositionStates.FLOOR;
    else if (ctrlValue <= -0.06 && ctrlValue >= -0.1)
      return armPositionStates.AMP;
    else if (ctrlValue <= -0.53 && ctrlValue >= -0.56)
      return armPositionStates.REST; 
    else if (ctrlValue <= -0.27 && ctrlValue >= -0.29){
      return armPositionStates.SPEAKER;
    }
    else
      return currentPos;
      
  }

  public static armPositionStates getPositionState() {
    return currentPos;
  }

  public ArmLiftStates getArmLiftState() {
    return ArmLiftState;
  }

  public void setTargetPos(double pos) {
    targetPosEntry.setDouble(pos);
  }

  public void runMotor() {
    leftArmMotor.setControl(new DutyCycleOut(0.0));
  }

  public void disableMotor() {
    leftArmMotor.disable();
  }

  public void setFF(double val) {
    FFEntry.setDouble(val);
  }
  @Override
  public void periodic() {

    armAngle = 19.5243 - getPos() * 360 / Constants.gearRatioArm;

    armWristAngle = 180 - armAngle - Wrist.wristAngle;

    effectiveArmLength = Math.sqrt(Math.pow(Constants.armLength, 2) + Math.pow(Constants.wristLength, 2) - 2 * Constants.armLength * Constants.wristLength * Math.cos(armWristAngle));
    armOffsetAngle = Math.sin(Math.sin(Math.toRadians(armWristAngle) * Constants.wristLength / Constants.armLength));
    effectiveArmMass = 0.17 * (Constants.armLength + Constants.wristLength);

    AuxiliaryFF = effectiveArmLength * effectiveArmMass * Math.sin(Math.toRadians(armAngle + armOffsetAngle));// *9.81
    leftArmMotor.setControl(m_position.withFeedForward(AuxiliaryFF));

    FFEntry.setDouble(AuxiliaryFF);

    // System.out.println(limitSwitch.get()); Limit switch testing
    // This method will be called once per scheduler run

    stateEncoderEntry.setString(currentPos.toString());
    limitSwitchEntry.setBoolean(getLimitSwitch());
    currentPosEntry.setDouble(getPos()/151);
    PosErrEntry.setDouble(targetPosEntry.getDouble(0) - currentPosEntry.getDouble(0));

    armStateEntry.setString(getArmLiftState().toString());
    // movePos(targetPosEntry.getDouble(0)); Ruins the positionState change

    if (DriverStation.isEnabled()) {
      switch (ArmLiftState) {
        case NOT_INITIALIZED:
          break;
        case INITIALIZING:
          // CommandScheduler.getInstance().schedule(new ResetPosition()); Only works in
          // RobotContainer (Configure bindings) and Robot
          break;
        case INITIALIZED:
          currentPos = armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ENCODER_PORT));
      }
    }
  }
}
