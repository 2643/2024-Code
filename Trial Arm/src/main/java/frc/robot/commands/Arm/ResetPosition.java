// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.ArmLiftStates;

public class ResetPosition extends Command {
  boolean finish = false;
  boolean flag = false;
  /** Creates a new ResetPosition. */
  public ResetPosition() {
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     RobotContainer.m_armLift.setVelocity(0.5);
    // RobotContainer.m_armLift.setAcceleration(2);
     if(RobotContainer.m_armLift.getLimitSwitch()) {
      RobotContainer.m_armLift.movePos(-0.3);
    } else {
      System.out.println("Hey man put arm on >:(");
      RobotContainer.m_armLift.disableMotor();
      RobotContainer.m_armLift.setArmLiftState(ArmLiftStates.NOT_INITIALIZED);
      flag = true;
    }
    
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(RobotContainer.m_armLift.getLimitSwitch()) && flag == false) {
      RobotContainer.m_armLift.setArmLiftState(ArmLift.ArmLiftStates.INITIALIZED);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armLift.setPos(0);
    RobotContainer.m_armLift.movePos(0);
    // RobotContainer.m_armLift.setVelocity(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
