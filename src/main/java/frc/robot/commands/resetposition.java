// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristStates;

public class resetposition extends Command {
  boolean finish = false;
  /** Creates a new resetposition. */
  public resetposition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_wrist.getLimitSwitch()){
        RobotContainer.m_wrist.movePos(2000);}
    finish = true;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_wrist.setPos(0);
    RobotContainer.m_wrist.setWristState(WristStates.INITIALIZED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
