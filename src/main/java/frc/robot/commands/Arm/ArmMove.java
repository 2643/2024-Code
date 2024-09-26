// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.ArmLift.ArmLiftStates;
// import frc.robot.subsystems.Wrist.WristpositionStates;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmMove extends Command {
  Arm.armPositionStates state;
  // ArmLift.armPositionStates dummy;

  /** Creates a new ArmLift. */
  public ArmMove(Arm.armPositionStates state) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_armLift); // Used to add parameters to the subsystem
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // dummy = state;
    //System.out.println(state); // Check what state it is in
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (RobotContainer.autoAngleButton.getAsBoolean()) {
    //   RobotContainer.m_armLift.movePos(RobotContainer.m_vision.autoArm());
    // }
    //System.out.println(state);
    // dummy = ArmLift.armPositionStates.DUMMY;
     if (state == Arm.armPositionStates.AMP) { 
      RobotContainer.m_armLift.movePos(Constants.AMP);
    } else if (state == Arm.armPositionStates.FLOOR) {
      RobotContainer.m_armLift.movePos(Constants.FLOOR);
    } else if (state == Arm.armPositionStates.SPEAKER) {
      RobotContainer.m_armLift.movePos(Constants.SPEAKER);
    } else if (state == Arm.armPositionStates.REST) {
      RobotContainer.m_armLift.movePos(Constants.REST);
    } else if (state == Arm.armPositionStates.SNIPE) {
      RobotContainer.m_armLift.movePos(RobotContainer.m_vision.autoArm());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
