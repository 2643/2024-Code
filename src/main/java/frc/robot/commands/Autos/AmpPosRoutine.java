// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.subsystems.Arm.armPositionStates;
import frc.robot.subsystems.Wrist.wristPositionStates;
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpPosRoutine extends ParallelCommandGroup {
  /** Creates a new AmpPosRoutine. */
  public AmpPosRoutine() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
        addCommands(new ArmMove(armPositionStates.AMP), new WristMove(wristPositionStates.AMP));
  }
}
