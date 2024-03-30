// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Arm.ResetPositionArm;
//import frc.robot.commands.Grabber.Outtake;
import frc.robot.commands.Swerve.AutoSwerve;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.commands.Wrist.ResetPositionWrist;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmLiftStates;
import frc.robot.subsystems.Arm.armPositionStates;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristStates;
import frc.robot.subsystems.Wrist.wristPositionStates;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private armPositionStates isCurrentPos = armPositionStates.REST;

  private wristPositionStates isCurrentWristPos = wristPositionStates.REST;

  // private boolean hazNote;
  // private wristPositionStates isCurrentEncoderPos;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // if (RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZING) {
    //   CommandScheduler.getInstance().schedule(new ResetPositionArm());
    // }
    // if (RobotContainer.m_wrist.getWristState() == WristStates.NOT_INITIALIZED) {
    //   CommandScheduler.getInstance().schedule(new ResetpositionWrist());

    // }

    // CommandScheduler.getInstance().schedule(new ResetPosition());
    CommandScheduler.getInstance().schedule(new AutoSwerve());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      // System.out.println("hi");
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // MYSTERY ACTOR HERE
    // isCurrentEncoderPos = Wrist.WristgetPositionState();
    isCurrentPos = Arm.getPositionState();
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.s_Swerve, new TeleopSwerve());

    if (RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZING) {
      CommandScheduler.getInstance().schedule(new ResetPositionArm());
    }
    if (RobotContainer.m_wrist.getWristState() == WristStates.NOT_INITIALIZED) {
      CommandScheduler.getInstance().schedule(new ResetPositionWrist());
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // if (RobotContainer.autoAngleButton.getAsBoolean()) {
    //   CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new ArmMove(armPositionStates.SNIPE), new WristMove(wristPositionStates.SNIPE)));
    // }
    
    
    if (isCurrentPos != Arm.getPositionState()
        && RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED
        && RobotContainer.m_wrist.getWristState() == WristStates.INITIALIZED
        && !RobotContainer.autoAngleButton.getAsBoolean()) {
      isCurrentPos = Arm.getPositionState();
      CommandScheduler.getInstance().schedule(new ArmMove(isCurrentPos));
    }

    if (isCurrentWristPos != Wrist.WristgetPositionState()
        && RobotContainer.m_wrist.getWristState() == WristStates.INITIALIZED
        && RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED
        && !RobotContainer.autoAngleButton.getAsBoolean()) {
      isCurrentWristPos = Wrist.WristgetPositionState();
      CommandScheduler.getInstance().schedule(new WristMove(isCurrentWristPos));
    }


    if (RobotContainer.autoAngleButton.getAsBoolean() && RobotContainer.m_armLift.getArmLiftState() == ArmLiftStates.INITIALIZED && RobotContainer.m_wrist.getWristState() == WristStates.INITIALIZED) //{
      CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new ArmMove(armPositionStates.SNIPE), new WristMove(wristPositionStates.SNIPE)));
    //   if (RobotContainer.m_Grabber.getLimitSwitch()) {
    //     hazNote = true;
    //   }
    //   if (RobotContainer.m_vision.getError() < 2 && hazNote && RobotContainer.m_vision.getError() != 0) {
    //     hazNote = false;
    //     CommandScheduler.getInstance().schedule(new Outtake());
    //   }
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
