// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;

public class ResetPosition extends Command {
  boolean finish = false;
  /** Creates a new ResetPosition. */
  public ResetPosition() {
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     System.out.println("hi");
     if(RobotContainer.m_armLift.getLimitSwitch()) {
      System.out.println("Reach!!!");
      RobotContainer.m_armLift.movePos(1.5);
      // if(RobotContainer.m_armLift.getPos()== 1.5){
      //    RobotContainer.m_armLift.setArmLiftState(ArmLift.ArmLiftStates.INITIALIZED);
      // }
     
    } else {
      System.out.println("Hey man put arm on >:(");
    }
    
    finish = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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
