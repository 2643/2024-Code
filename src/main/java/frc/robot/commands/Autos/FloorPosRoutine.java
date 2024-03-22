package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.subsystems.Arm.armPositionStates;
import frc.robot.subsystems.Wrist.wristPositionStates;

public class FloorPosRoutine extends ParallelCommandGroup {
    public FloorPosRoutine() {
        addCommands(new ArmMove(armPositionStates.FLOOR), new WristMove(wristPositionStates.FLOOR));
    }
}