package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Arm.ResetPositionArm;
import frc.robot.commands.Grabber.Outtake;
import frc.robot.commands.Wrist.ResetPositionWrist;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.subsystems.Arm.armPositionStates;
import frc.robot.subsystems.Wrist.wristPositionStates;

public class SpeakerRoutine extends SequentialCommandGroup {
    public SpeakerRoutine() {

        addCommands(new WaitCommand(0.5), new ResetPositionArm(), new ResetPositionWrist(), new WaitCommand(.5),new ArmMove(armPositionStates.SPEAKER), new WristMove(wristPositionStates.SPEAKER), new WaitCommand(1),
                    new Outtake().withTimeout(1), new WaitCommand(1), new ArmMove(armPositionStates.REST), new WristMove(wristPositionStates.REST));
    }
}