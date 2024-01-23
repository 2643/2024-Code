// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public static enum positionStates { //Learn how to make state machine
    FLOOR,
    AMP,
    HOOK,
}

public class ArmLift extends SubsystemBase {
    TalonFX leftMotor = new TalonFX(Constants.motor1Port);
    Follower rightMotor = new Follower(Constants.motor1Port, true);
    DigitalInput limit = new DigitalInput(Constants.limitPort);

  /** Creates a new ArmLift. */
  public ArmLift() {
    //Insert Configurations (ask rayrith)
  }

  public void getPos() {
    leftMotor.getPosition();
  }

  public void setPos(double pos) {
    leftMotor.setPosition(pos);
  }

  public void destroyMotor() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

