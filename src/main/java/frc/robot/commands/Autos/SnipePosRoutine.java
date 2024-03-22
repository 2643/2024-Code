// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.Arm.armPositionStates;

public class SnipePosRoutine extends Command {
  Timer time = new Timer();
  boolean finish = false;
  /** Creates a new SnipePos. */
  public SnipePosRoutine() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (time.get() < 1.2) {
      RobotContainer.m_armLift.movePos(RobotContainer.m_vision.autoArm());
      RobotContainer.m_wrist.movePos(0);
    }
    if (time.get() > 1.2) {
      time.stop();
      time.reset();
      finish = true;
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    end(finish);
    return finish;
  }
}
