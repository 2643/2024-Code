// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ArmLift extends SubsystemBase {

  public static enum positionStates { //Learn how to make state machine
    FLOOR,
    PICKUP,
    AMP,
    HOOK,
    SPEAKER,
    REST
  }
  
    public static enum ArmLiftStates {
      NOT_INITIALIZED,
      INITIALIZING_CALLED,
      INITIALIZING,
      INITIALIZED
    }

    //motors

    ArmLiftStates ArmLiftState = ArmLiftStates.NOT_INITIALIZED;

    TalonFX leftarmMotor = new TalonFX(Constants.motor1Port);
    TalonFX rightarmMotor = new TalonFX(Constants.motor2Port);

    //limit switches
    DigitalInput limit = new DigitalInput(Constants.limitPort);

  /** Creates a new ArmLift. */
  public ArmLift() {
    leftarmMotor.setControl(new Follower(Constants.motor2Port, true));
  }

  public void getPos() {
    leftarmMotor.getPosition();
  }

  public void setPos(double pos) {
    leftarmMotor.setControl(new PositionDutyCycle(pos));
  }

  public void movePos(double pos) {
    leftarmMotor.setControl(new MotionMagicDutyCycle(pos));
  }

  public ArmLiftStates getArmLiftState() {
    return ArmLiftState;
  }

  public void setArmLiftState (ArmLiftStates state) {
    ArmLiftState = state;
  }

  public void reset(){
    setPos(0);
    movePos(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

