// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.ArmLift;
// import frc.robot.subsystems.Wrist.positionStates;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.wristPositionStates;

public class WristMove extends Command {
  Wrist.wristPositionStates state;

  // Wrist.wristPositionStates dummy;
  /** Creates a new Wrist. */
  public WristMove(Wrist.wristPositionStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_wrist); // Used to add parameters to the subsystem
    this.state = state;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // dummy = state;

    // System.out.println(state); //Check what state it is in
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println(state);
    // if(dummy!=state){
    // dummy = Wrist.wristPositionStates.DUMMY;
    // if (RobotContainer.m_wrist.getWristState() == Wrist.WristStates.INITIALIZED)
    // {
    
    if (state == wristPositionStates.SPEAKER) {
      RobotContainer.m_wrist.movePos(Constants.SPEAKER_WRIST);
    } else if (state == wristPositionStates.REST) {
      RobotContainer.m_wrist.movePos(Constants.REST_WRIST);
    } else if (state == wristPositionStates.AMP) {
      RobotContainer.m_wrist.movePos(Constants.AMP_WRIST);
    } else if (state == wristPositionStates.FLOOR) {
      RobotContainer.m_wrist.movePos(Constants.FLOOR_WRIST);
    }
      else if (state == wristPositionStates.SNIPE) {
        RobotContainer.m_wrist.movePos(Constants.SPEAKER_WRIST);
    }

  }
  // RobotContainer.m_wrist.setWristPositionState(wristPositionStates.REST);

  // else if(state == Wrist.positionStates.FLOOR) {
  // RobotContainer.m_Wrist.movePos(Constants.FLOOR);
  // }
  // }
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
