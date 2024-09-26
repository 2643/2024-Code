// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  TalonFX motor = new TalonFX(Constants.motorID);
  CANSparkMax neoMotor = new CANSparkMax(Constants.neoMotorID, CANSparkLowLevel.MotorType.kBrushless);
  DigitalInput sensor = new DigitalInput(Constants.sensorID);
  Timer time = new Timer();
  Timer timeOuttake = new Timer();
  double curFalconVelocity;
  double curNeoVelocity;
  ShuffleboardTab tab = Shuffleboard.getTab("Velocity");
  GenericEntry currentFalconEntry = tab.add("Falcon Velocity", curFalconVelocity).getEntry();
  GenericEntry currentNeoEntry = tab.add("Neo Velocity", curNeoVelocity).getEntry();
  GenericEntry didWeIntake = Shuffleboard.getTab("Driver").add("Intake Limit Switch", false).getEntry();
  boolean isActive = false;
  Timer timeOut = new Timer();
  double neoSpeed;

  
  //TOD: VELOCITY LIMIT DOES NOT WORK, but it works so who really cares

  public Grabber() {
    // ... why is this here?
    // motor.setControl(new PositionDutyCycle(100));
  }

  public void grabberIntake(double falconSpeed, double neoSpeed) {
    // limit velocity
    // if(curFalconVelocity < Constants.falconVelocityLimit ||
    // curNeoVelocity < Constants.neoVelocityLimit) {
    //   falconSpeed = 0;
    //   neoSpeed = 0;
    // }
    
    if(sensor.get() && Arm.getPositionState() == Arm.armPositionStates.FLOOR) {
      falconSpeed = 0;
      time.start();
      if(time.get() > 0.125) {
        neoSpeed = 0;
        neoMotor.setIdleMode(IdleMode.kBrake);
      }
  }
    motor.setControl(new DutyCycleOut(falconSpeed));
    neoMotor.set(neoSpeed);
  }
  

  public void grabberOuttake(double falconSpeed, double neoSpeed) {
    // limit velocity
    // if(curFalconVelocity < Constants.falconVelocityLimit ||
    // curNeoVelocity < Constants.neoVelocityLimit) {
    //   falconSpeed = 0;
    //   neoSpeed = 0;
    // }

    // limit time
    // time1.start();
    // System.out.println("start");
    // if(time1.get() > 5) {
    //   stopGrabber();
    // }
    this.neoSpeed = neoSpeed;
    if (!isActive) {
      timeOut.start();
      isActive = true;
      neoMotor.set(0.15);
      motor.setControl(new DutyCycleOut(falconSpeed));
      timeOuttake.start();
    }
  }

  public void stopGrabber() {
    motor.setControl(new DutyCycleOut(0));
    neoMotor.set(0);
    time.stop();
    time.reset();
  }

  public boolean getLimitSwitch() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (timeOut.get() > 1.35) {
      timeOut.stop();
      timeOut.reset();
      stopGrabber();
      isActive = false;
    }
    if(timeOuttake.get() > 0.85) {
      neoMotor.set(neoSpeed);
      timeOuttake.stop();
      timeOuttake.reset();
    }
    didWeIntake.setBoolean(sensor.get());
    curFalconVelocity = motor.getVelocity().getValueAsDouble();
    curNeoVelocity = neoMotor.getEncoder().getVelocity();
    currentFalconEntry.setDouble(curFalconVelocity);
    currentNeoEntry.setDouble(curNeoVelocity);
  }
}
