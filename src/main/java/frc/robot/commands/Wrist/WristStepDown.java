// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WristStepDown extends Command {
  /** Creates a new wristDownCom. */
  Boolean finish = false;
  public WristStepDown() {
    addRequirements(RobotContainer.m_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.m_wrist.getPos() <= 0){
      RobotContainer.m_wrist.movePos(RobotContainer.m_wrist.returnTargetPos() - 0.01);
      finish = true;
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
