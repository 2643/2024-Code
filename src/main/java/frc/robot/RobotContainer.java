// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Arm.ArmStepDown;
import frc.robot.commands.Arm.ArmStepUp;
import frc.robot.commands.Grabber.Intake;
import frc.robot.commands.Grabber.Outtake;
import frc.robot.commands.Grabber.StopGrabber;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.commands.Wrist.WristStepDown;
import frc.robot.commands.Wrist.WristStepUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.armPositionStates;
import frc.robot.subsystems.Wrist.wristPositionStates;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final static Grabber m_Grabber = new Grabber();

  public static Joystick driver = new Joystick(1);
  public  JoystickButton zeroGyro = new JoystickButton(driver, 1);
  public static JoystickButton robotCentric = new JoystickButton(driver, 12);

  public static Arm m_armLift = new Arm();
  public static Wrist m_wrist = new Wrist();
  public static Vision m_vision = new Vision();

  // The robot's subsystems and commands are defined here...
  // public static final Joystick test = new Joystick(0);
  public static final Joystick operatorBoard = new Joystick(0);
  public static JoystickButton wristUp = new JoystickButton(operatorBoard, 8);
  public static JoystickButton wristDown = new JoystickButton(operatorBoard, 9);

  public static JoystickButton armUp = new JoystickButton(operatorBoard, 5);
  public static JoystickButton armDown = new JoystickButton(operatorBoard, 14);

  public JoystickButton intakeButton = new JoystickButton(operatorBoard, Constants.intakeButtonID);
  public JoystickButton outtakeButton = new JoystickButton(operatorBoard, Constants.outtakeButtonID);
  public static JoystickButton turnSwitch = new JoystickButton(operatorBoard, 15);
  public static JoystickButton autoAngleButton = new JoystickButton(operatorBoard, 11);
  public static JoystickButton snipeButton = new JoystickButton(operatorBoard, 4);

  public static Swerve s_Swerve = new Swerve();

  // private final JoystickButton first = new JoystickButton(test, 1);
  // private final JoystickButton second = new JoystickButton(test, 2);  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver").add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2).withPosition(0, 2);

  // public final Command AmpRoutine = new AmpRoutine();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("Reset", new ResetArmAndWrist());
    NamedCommands.registerCommand("Floor", new FloorPosRoutine());
    NamedCommands.registerCommand("AmpWrist", new WristMove(wristPositionStates.SPEAKER));
    NamedCommands.registerCommand("AmpArm", new ArmMove(armPositionStates.SPEAKER));
    NamedCommands.registerCommand("Outtake", new Outtake().withTimeout(1));
    NamedCommands.registerCommand("StopGrab", new StopGrabber().withTimeout(0.1));
    NamedCommands.registerCommand("Intake", new Intake());
    NamedCommands.registerCommand("Speaker", new SpeakerPosRoutine());
    NamedCommands.registerCommand("Rest", new RestPosRoutine());
    NamedCommands.registerCommand("Amp", new AmpPosRoutine());
    NamedCommands.registerCommand("Snipe", new SnipePosRoutine().withTimeout(1.3));
    //NamedCommands.registerCommand("Thing", new FloorRoutine());
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("SPEAKER Routine", new SpeakerRoutine());
    autoChooser.addOption("SidePathAway", new PathPlannerAuto("side"));
    autoChooser.addOption("MiddlePath", new PathPlannerAuto("simpleAuto"));
    autoChooser.addOption("guh", new PathPlannerAuto("guh"));
    autoChooser.addOption("SidePathStage", new PathPlannerAuto("sideAutoStage"));
    autoChooser.addOption("MiddlePathStage", new PathPlannerAuto("Copy of simpleAuto"));
    autoChooser.addOption("Nothing", new PathPlannerAuto("nothing"));
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    wristDown.onTrue(new WristStepDown());
    wristUp.onTrue(new WristStepUp());

    armUp.onTrue(new ArmStepUp());
    armDown.onTrue(new ArmStepDown());
    //ArmStick.onTrue(new ArmMove(armPositionStates.SPEAKER));
    // first.onTrue(new first());
    // second.onTrue(new second());

    intakeButton.onTrue(new Intake());
    outtakeButton.onTrue(new Outtake());
    snipeButton.onTrue(new SnipePosRoutine());

    intakeButton.onFalse(new StopGrabber());
    // outtakeButton.onFalse(new StopGrabber());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
