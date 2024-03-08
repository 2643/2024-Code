package frc.robot.commands.autos;

import frc.robot.commands.Grabber.Outtake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.WristCom.WristMove;
import frc.robot.subsystems.ArmLift.positionStates;
import frc.robot.subsystems.Wrist.WristpositionStates;

public class AmpRoutine extends SequentialCommandGroup {
    public AmpRoutine() {
        addCommands(new ArmMove(positionStates.AMP), new WristMove(WristpositionStates.AMP), new Outtake());
    }
}