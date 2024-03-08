// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Arm.armDownCom;
import frc.robot.commands.Arm.armUpCom;
import frc.robot.commands.Grabber.Intake;
import frc.robot.commands.Grabber.Outtake;
import frc.robot.commands.Grabber.StopGrabber;
import frc.robot.commands.WristCom.wristDownCom;
import frc.robot.commands.WristCom.wristUpCom;
import frc.robot.commands.autos.SpeakerRoutine;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

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
  public static JoystickButton robotCentric = new JoystickButton(driver, 2);

  public static ArmLift m_armLift = new ArmLift();
  public static Wrist m_wrist = new Wrist();

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // public static final Joystick test = new Joystick(0);
  public static final Joystick operatorBoard = new Joystick(0);
  public static JoystickButton wristUp = new JoystickButton(operatorBoard, 8);
  public static JoystickButton wristDown = new JoystickButton(operatorBoard, 9);

  public static JoystickButton armUp = new JoystickButton(operatorBoard, 5);
  public static JoystickButton armDown = new JoystickButton(operatorBoard, 14);

  public JoystickButton intakeButton = new JoystickButton(operatorBoard, Constants.intakeButtonID);
  public JoystickButton outtakeButton = new JoystickButton(operatorBoard, Constants.outtakeButtonID);
  public static JoystickButton turnSwitch = new JoystickButton(operatorBoard, 15);

  public static Swerve s_Swerve = new Swerve();

  // private final JoystickButton first = new JoystickButton(test, 1);
  // private final JoystickButton second = new JoystickButton(test, 2);  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver").add("Autonoumous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2).withPosition(0, 2);
  public final Command SpeakerRoutine = new SpeakerRoutine();
  // public final Command AmpRoutine = new AmpRoutine();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    autoChooser.setDefaultOption("SPEAKER Routine", new SpeakerRoutine());
    
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

    wristDown.onTrue(new wristDownCom());
    wristUp.onTrue(new wristUpCom());

    armUp.onTrue(new armUpCom());
    armDown.onTrue(new armDownCom());
    //ArmStick.onTrue(new ArmMove(positionStates.SPEAKER));
    // first.onTrue(new first());
    // second.onTrue(new second());

    intakeButton.onTrue(new Intake());
    outtakeButton.onTrue(new Outtake());

    intakeButton.onFalse(new StopGrabber());
    outtakeButton.onFalse(new StopGrabber());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
