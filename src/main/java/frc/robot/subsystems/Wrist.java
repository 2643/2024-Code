// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.Timer;




public class Wrist extends SubsystemBase {

  Timer time;

  public static enum moveJoystick { //States to check wwhat direction the joystick is moving
    Up,
    Down
  }

  public static enum positionStates { //Different position states for the arm to move
    FLOOR,
    AMP,
    HOOK,
    SPEAKER,
    REST
  }

  public static enum WristStates { //Different states that determines what stage the arm is in.
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED
  }
  
  //Configurations:


  // motors

  TalonFX leftarmMotor = new TalonFX(Constants.leftmotorPort);
  TalonFX rightarmMotor = new TalonFX(Constants.rightmotorPort);


  // limit switches

  DigitalInput limitSwitch = new DigitalInput(Constants.limitPort);

  // values

  double currentArmEncoderPos;
  double AuxiliaryFF;
  static positionStates currentPos = positionStates.REST;
  WristStates WristState = WristStates.INITIALIZING;
  static positionStates positionState = positionStates.REST;

  // PID
  GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", 0).getEntry();
  GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral", 0).getEntry();
  GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",0).getEntry();
  GenericEntry FFEntry = Shuffleboard.getTab("PID").add("Feed Forward",0).getEntry();


  // motion magic velocity and acceleration
  GenericEntry accelEntry = Shuffleboard.getTab("PID").add("Acceleration", 0).getEntry();
  GenericEntry velEntry = Shuffleboard.getTab("PID").add("Velocity", 0).getEntry();

  // position
  GenericEntry currentPosEntry = Shuffleboard.getTab("PID").add("Current Position", 0).getEntry();
  GenericEntry targetPosEntry = Shuffleboard.getTab(  "PID").add("Target Position", 0).getEntry();
  GenericEntry PosErrEntry = Shuffleboard.getTab("PID").add("Pos Err", 0).getEntry();

  // limit switch
  
   GenericEntry limitSwitchEntry = Shuffleboard.getTab("PID").add("Limit Switch", true).getEntry();

  // requests
  final MotionMagicVoltage m_position = new MotionMagicVoltage(0);

  public Wrist() {

    //PID/Velocity configurations make sure that the motionMagic and slot0 configs are under var!
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    var slot0Configs = talonFXConfigs.Slot0;    

    slot0Configs.kP = 3; //Proportional 2.5 seems good for actual robot
    slot0Configs.kI = 0; //Integral
    slot0Configs.kD = 0; //Derivative
    slot0Configs.kV = 0; //Velocity
    
    motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration 
    motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target velocity

    leftarmMotor.getConfigurator().apply(talonFXConfigs);


    rightarmMotor.setControl(new Follower(Constants.leftmotorPort, true)); //Follower master the rightarmMotor is reversed b/c double linked arm
  }

  public double getPos() {
    return leftarmMotor.getPosition().getValueAsDouble();
  }

  public void setPos(double pos) {
    leftarmMotor.setPosition(pos);
  }

  // public void setSpeed(double speed) { Doesn't work
  //   leftarmMotor.set(speed);
  // }

  public void movePos(double pos) {
    leftarmMotor.setControl(m_position.withPosition(pos * Constants.gearRatio));
    leftarmMotor.setControl(m_position.withFeedForward(AuxiliaryFF));
    setTargetPos(pos);
  }

  public void setWristState(WristStates state) {
    WristState = state; 
  }

  public void reset() {
    setPos(0);
    movePos(0);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  // public void setVelocity(double val) { Doesn't work
  //   leftarmMotor.setControl(m_velocity.withVelocity(val));
    // var motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = val;
    // leftarmMotor.getConfigurator().apply(motionMagicConfigs);
  // }

  // public void setAcceleration(double val) { Doesn't work
  //   var motionMagicConfigs = new MotionMagicConfigs();
  //   motionMagicConfigs.MotionMagicAcceleration = val;
  //   leftarmMotor.getConfigurator().apply(motionMagicConfigs);
  // }

  public double getVelocity() {
    return velEntry.getDouble(0);
  }

  public void setAcceleration(double val) { 
    accelEntry.setDouble(val);
  }

  public double getAccel() {
    return accelEntry.getDouble(0);
  }

  public static positionStates armPlacement(double ctrlValue) { //Checks the 
    // if (ctrlValue <= 0.05 && ctrlValue >= 0) 
    //   return positionStates.HOOK;
    if (ctrlValue <= -0.95 && ctrlValue >= -0.98)
      return positionStates.HOOK;
    else if (ctrlValue <= -0.06 && ctrlValue >= -0.1)
      return positionStates.REST;
    else if (ctrlValue <= -0.53 && ctrlValue >= -0.56)
      return positionStates.AMP; 
    else if (ctrlValue <= -0.27 && ctrlValue >= -0.29)
      return positionStates.SPEAKER;
    else
      return currentPos;
  }

  public static positionStates getPositionState() {
    return currentPos;
  }

  public WristStates getWristState() {
    return WristState;
  }

  public void setTargetPos(double pos) {
    targetPosEntry.setDouble(pos);
  }

  public void runMotor() {
    leftarmMotor.setControl(new DutyCycleOut(0.0));
  }

  public void disableMotor() {
    leftarmMotor.disable();
  }

  public void setFF(double val) {
    FFEntry.setDouble(val);
  }

  @Override
  public void periodic() {

    AuxiliaryFF = 0.17 * Math.sin(Math.toRadians((getPos()+19.5243))); //19.5243 is the offset
    setFF(AuxiliaryFF);

    // System.out.println(limitSwitch.get()); Limit switch testing  
    // This method will be called once per scheduler run

    limitSwitchEntry.setBoolean(getLimitSwitch());
    currentPosEntry.setDouble(getPos());
    PosErrEntry.setDouble(targetPosEntry.getDouble(0) - currentPosEntry.getDouble(0));
    // movePos(targetPosEntry.getDouble(0)); Ruins the positionState change

    if(DriverStation.isEnabled()) {
    switch(WristState) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        // CommandScheduler.getInstance().schedule(new ResetPosition()); Only works in RobotContainer (Configure bindings) and Robot
        break;
      case INITIALIZED:
          currentPos = armPlacement(RobotContainer.operatorBoard.getRawAxis(Constants.ENCODER_PORT));
        }
      }
    }
  }
