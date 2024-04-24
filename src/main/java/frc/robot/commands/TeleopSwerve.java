package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.RobotContainer;

// IMPORTS THAT ARE NOT CURRENTLY USED
// import frc.robot.Robot;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;

public class TeleopSwerve extends Command {

  // Flag used to prevent spin up at the start
  boolean initFlag = true;

  // flag used for sync
  double encoderOffset;

  // Double suppliers for the movement values for swerve
  static private DoubleSupplier translationSup;
  static private DoubleSupplier strafeSup;
  static private DoubleSupplier rotationSup;
  static private BooleanSupplier robotCentricSup;

  // Encoder/rotational pid values, constraints, and controller
  static double encoderkP = 0.08;
  static double encoderkI = 0.00;
  static double encoderkD = 0.005;
  static double ff = 0.005;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(50, 10);
  ProfiledPIDController rotPid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, constraints);

  // Shuffleboard entries
  static GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", encoderkP).getEntry();
  static GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",encoderkI).getEntry();
  static GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",encoderkD).getEntry();
  static GenericEntry ffEntry = Shuffleboard.getTab("PID").add("ff",ff).getEntry();
  static GenericEntry rotValEntry = Shuffleboard.getTab("PID").add("outVal",0).getEntry();
  static GenericEntry diffValEntry = Shuffleboard.getTab("PID").add("diffVal",0).getEntry();
  static GenericEntry betterEncoderEntry = Shuffleboard.getTab("Swerve").add("Encoder Target Angle",0).getEntry();
  // static GenericEntry ffEntry = Shuffleboard.getTab("PID").add("Feed Forward",encoderkD).getEntry();
  // static GenericEntry velEntry = Shuffleboard.getTab("PID").add("VelTOl",encoderkD).getEntry();
  // static GenericEntry posEntry = Shuffleboard.getTab("PID").add("PostTOl",encoderkD).getEntry();  

  // The variable that holds the target angle for swerve to rotate to

  static double betterEncoderAngle = 0;//(RobotContainer.operatorBoard.getRawAxis(3) + 1) * 180;


  // Default swerve command

  public TeleopSwerve() {
    addRequirements(RobotContainer.s_Swerve);
    rotPid.enableContinuousInput(0, 360);
  }

  // Logarithmic control for swerve so that small movements remain small and large movements are large

  private static double logAxis(double value) {
    value = Math.copySign(Math.log((Math.abs(value) + 1)) / Math.log(2), value);
    return value;
  }

  // Squares for the same reason as the logarithm earlier, just for better control

  private static double squareAxis(double value, double deadband) {
    value = MathUtil.applyDeadband(value, deadband);
    value = Math.copySign(value * value, value);
    return value;
  }

  public double getBetterEncoderAngle() {
    return betterEncoderAngle;
  }

  @Override
  public void execute() {

    // Shuffleboard entries for rotational PID

    rotPid.setP(pEntry.getDouble(0));
    rotPid.setI(iEntry.getDouble(0));
    rotPid.setD(dEntry.getDouble(0));
    betterEncoderEntry.setDouble(betterEncoderAngle);

    // Shuffleboard entries for encoder

    // Switches modes between joystick/encoder rotational control TODO: change this to swap between flight controller and xbox

    if (!RobotContainer.turnSwitch.getAsBoolean()) {
      rotationSup = () -> RobotContainer.operatorBoard.getRawAxis(3);
    } else {
      rotationSup = () -> RobotContainer.driver.getRawAxis(2);
    }

    // Initializes movement suppliers to controller axis

    translationSup = () -> -RobotContainer.driver.getRawAxis(1);
    strafeSup = () -> -RobotContainer.driver.getRawAxis(0);
    robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean();

    // Attempts to zero the robot at startup and wait for a new angle to go to

    if (initFlag && !RobotContainer.turnSwitch.getAsBoolean()) {
      RobotContainer.s_Swerve.zeroHeading();
      encoderOffset = RobotContainer.operatorBoard.getRawAxis(3);
      initFlag = false;
    }
    else if (rotationSup.getAsDouble() + 1 - encoderOffset < 0 && !RobotContainer.turnSwitch.getAsBoolean()) {
      //System.out.println("im below 0");
      betterEncoderAngle = (2 + rotationSup.getAsDouble() + 1 - encoderOffset) * 180;
    }
    else if (rotationSup.getAsDouble() + 1 - encoderOffset > 2 && !RobotContainer.turnSwitch.getAsBoolean()) {
      //System.out.println("im above 2");
      betterEncoderAngle = (rotationSup.getAsDouble() + 1 - encoderOffset - 2) * 180;
    }
    else if (!RobotContainer.turnSwitch.getAsBoolean())
      betterEncoderAngle = (rotationSup.getAsDouble() + 1 - encoderOffset) * 180;
    betterEncoderAngle -= 180;
    // System.out.println(betterEncoderAngle);
    // System.out.println("encoder offset" + encoderOffset);
    // else {
    //   betterEncoderAngle = (rotationSup.getAsDouble() + 1) * 180;
    // }
    
    // Not sure what this is, leftover testing statements

      /*
     * betterEncoderAngle = ((rotationSup.getAsDouble() + 1) * )%360
     */
    // System.out.println("Better encoder angle: " + betterEncoderAngle);
    // System.out.println("RotationSup: " + rotationSup.getAsDouble());
    
    
    // Convert double supplier to double

    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();
    double rotationVal = rotationSup.getAsDouble();

    // Calculate the difference between the encoder angle and the gyro angle

    double new_Var = ((RobotContainer.s_Swerve.getGyroYaw().getDegrees()) - betterEncoderAngle);
    
    // Optimization; for example, if the robot is commanded to spin 190 degrees, it will instead spin 170 degrees in the other direction.
    // can cause bugs so comment out while testing
    // dont use, rot pid already implements a sort of optimization

    // if(new_Var > 180){
    //   new_Var -= 180;
    // }
    // else if(new_Var < -180){
    //   new_Var += 180;
    // }

    // Applying deadband and smoother control

    translationVal = squareAxis(logAxis(translationVal), Constants.stickDeadband);
    strafeVal = squareAxis(logAxis(strafeVal), Constants.stickDeadband);

    // Check what control to use based on switch (whether joystick or encoder rotation)

    if (!RobotContainer.turnSwitch.getAsBoolean() && RobotContainer.operatorBoard.isConnected()) {
      rotationVal = rotPid.calculate((RobotContainer.s_Swerve.getGyroYaw().getDegrees()), betterEncoderAngle); // m_feedforward.calculate(rotPid.getSetpoint().velocity, rotationVal, rotationVal, rotationVal, rotationVal);
      rotationVal += (-ffEntry.getDouble(ff) /* new_Var*/);  
    }
    else {
      rotationVal = squareAxis(logAxis(rotationVal), Constants.stickRotationDeadband) * Constants.Swerve.maxAngularVelocity / 4;
    }

    // Updating shuffleboard entries for rotation values

    rotValEntry.setDouble(rotationVal);
    diffValEntry.setDouble(new_Var);

    // Calling drive method in subsystem to move robot

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed/10),
        rotationVal,
        !robotCentricSup.getAsBoolean(),
        true);
  }

  public boolean isFinished() {
    return false;
  }
}