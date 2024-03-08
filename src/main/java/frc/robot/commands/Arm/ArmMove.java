// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmLift.ArmLiftStates;
import frc.robot.subsystems.Wrist.WristpositionStates;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmMove extends Command {
  ArmLift.positionStates state;
  // ArmLift.positionStates dummy;

  /** Creates a new ArmLift. */
  public ArmMove(ArmLift.positionStates state) {
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

    //System.out.println(state);
    // dummy = ArmLift.positionStates.DUMMY;
    if (state == ArmLift.positionStates.AMP) { // Checks what state the arm is set to right now and moves to the
                                               // corresponding position
      RobotContainer.m_armLift.movePos(Constants.AMP);
    } else if (state == ArmLift.positionStates.FLOOR) {
      RobotContainer.m_armLift.movePos(Constants.FLOOR);
    } else if (state == ArmLift.positionStates.SPEAKER) {
      RobotContainer.m_armLift.movePos(Constants.SPEAKER);
    } else if (state == ArmLift.positionStates.REST) {
      RobotContainer.m_armLift.movePos(Constants.REST);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}