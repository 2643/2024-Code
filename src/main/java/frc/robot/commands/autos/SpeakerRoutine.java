package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Grabber.Outtake;
import frc.robot.commands.WristCom.WristMove;
import frc.robot.subsystems.ArmLift.positionStates;
import frc.robot.subsystems.Wrist.WristpositionStates;

public class SpeakerRoutine extends SequentialCommandGroup {
    public SpeakerRoutine() {

        addCommands(new WaitCommand(.5),new ArmMove(positionStates.SPEAKER), new WristMove(WristpositionStates.SPEAKER), new WaitCommand(1),
                    new Outtake().withTimeout(1), new WaitCommand(1), new ArmMove(positionStates.REST), new WristMove(WristpositionStates.REST));
    }
}