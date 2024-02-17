// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  TalonFX motor = new TalonFX(Constants.motorID, "CANivore1");
  CANSparkMax neoMotor = new CANSparkMax(Constants.neoMotorID, CANSparkLowLevel.MotorType.kBrushless);
  DigitalInput sensor = new DigitalInput(Constants.sensorID);
  Timer time = new Timer();
  Timer time1 = new Timer();
  
  public Grabber() {
    motor.setControl(new PositionDutyCycle(100));
  }

  public void grabberIntake(double falconSpeed, double neoSpeed) {
    time1.start();
    if(time1.get() > 7) {
      falconSpeed = 0;
      neoSpeed = 0;
    }

    if(sensor.get()) {
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
    time1.start();
    if(time1.get() > 7) {
      falconSpeed = 0;
      neoSpeed = 0;
    }

    motor.setControl(new DutyCycleOut(falconSpeed));
    time.start();
    if(time.get() > 1) {
      neoMotor.set(neoSpeed);
    }
  }

  public void stopGrabber() {
    motor.setControl(new DutyCycleOut(0));
    neoMotor.set(0);
    time.reset();
    time1.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
