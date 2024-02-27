// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.WristCom;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.Wrist.positionStates;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.positionStates;



public class WristMove extends Command {
  Wrist.positionStates state;

  /** Creates a new Wrist. */
  public WristMove(Wrist.positionStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist); //Used to add parameters to the subsystem
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println(state); //Check what state it is in
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(state == positionStates.AMP) { //Checks what state the arm is set to right now and moves to the corresponding position
       RobotContainer.m_wrist.movePos(Constants.AMP);
    }
    // else if(state == Wrist.positionStates.FLOOR) {
    //     RobotContainer.m_Wrist.movePos(Constants.FLOOR);
    // }
    else if(state == positionStates.SPEAKER) {
      RobotContainer.m_wrist.movePos(Constants.SPEAKER );
    }
    else if(state == positionStates.REST ) {
      RobotContainer.m_wrist.movePos(Constants.REST);
    }
    else if(state == positionStates.HOOK) {
        RobotContainer.m_wrist.movePos(Constants.HOOK);
    }
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
