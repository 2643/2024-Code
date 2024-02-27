// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCom;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristStates;

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
    // RobotContainer.m_Wrist.setVelocity(0.5); Doesn't work
    // RobotContainer.m_Wrist.setAcceleration(2); Doesn't work
     if(!RobotContainer.m_wrist.getLimitSwitch()) { //If the arm is on the limit switch
      // RobotContainer.m_Wrist.movePos(-0.3);
      System.out.println("Arm not on limit switch. Disabling motor.");
      RobotContainer.m_wrist.disableMotor();
      RobotContainer.m_wrist.setWristState(WristStates.NOT_INITIALIZED);
      flag = true;
      } 
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(RobotContainer.m_wrist.getLimitSwitch())) { //Checks if the arm moved out of the limit switch
      RobotContainer.m_wrist.setWristState(Wrist.WristStates.INITIALIZED);
      finish = true;
    }
    else{
      RobotContainer.m_wrist.movePos(-0.03);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_wrist.setPos(0); //Makes sure that the arm is at zero
    RobotContainer.m_wrist.movePos(0);
    // RobotContainer.m_Wrist.setVelocity(2); Doesn't work
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
