// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.armPositionStates;

public class Outtake extends Command {
  /** Creates a new Outtake. */
  boolean finish = false;
  // Timer time = new Timer();
  // boolean isActive = false;
  public Outtake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Arm.getPositionState() == armPositionStates.AMP)
      RobotContainer.m_Grabber.grabberOuttake(0.5, -0.4);
    else
      RobotContainer.m_Grabber.grabberOuttake(1, -0.8);
    // if (!isActive) {
    //   isActive = true;
    //   time.start();
    //   System.out.println("Starting");
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (time.get() > 5) {
    //   time.stop();
    //   time.reset();
    //   isActive = false;
    //   System.out.println("Finished" + time.get());
    // }
    // if (!isActive)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
