package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TeleopSwerve extends Command {

  static private DoubleSupplier translationSup;
  static private DoubleSupplier strafeSup;
  static private DoubleSupplier rotationSup;
  static private BooleanSupplier robotCentricSup;

  static double encoderkP = 0.08;
  static double encoderkI = 0.00;
  static double encoderkD = 0.005;

 static double ff = 0.005;

  
 

  static GenericEntry pEntry = Shuffleboard.getTab("PID").add("Proportional", encoderkP).getEntry();
  static GenericEntry iEntry = Shuffleboard.getTab("PID").add("Integral",encoderkI).getEntry();
  static GenericEntry dEntry = Shuffleboard.getTab("PID").add("Derivative",encoderkD).getEntry();
  static GenericEntry ffEntry = Shuffleboard.getTab("PID").add("ff",ff).getEntry();
  static GenericEntry rotValEntry = Shuffleboard.getTab("PID").add("outVal",0).getEntry();
  static GenericEntry diffValEntry = Shuffleboard.getTab("PID").add("diffVal",0).getEntry();
  //static GenericEntry ffEntry = Shuffleboard.getTab("PID").add("Feed Forward",encoderkD).getEntry();
  // static GenericEntry velEntry = Shuffleboard.getTab("PID").add("VelTOl",encoderkD).getEntry();
  // static GenericEntry posEntry = Shuffleboard.getTab("PID").add("PostTOl",encoderkD).getEntry();
 

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(100, 100);
  ProfiledPIDController rotational_pid = new ProfiledPIDController(encoderkP, encoderkI, encoderkD, constraints);
  static double betterEncoderAngle;

  public TeleopSwerve() {
    addRequirements(RobotContainer.s_Swerve);
    rotational_pid.enableContinuousInput(0, 360);
  }

  private static double logAxis(double value) {
    value = Math.copySign(Math.log((Math.abs(value) + 1)) / Math.log(2), value);
    return value;
  }

  private static double squareAxis(double value, double deadband) {
    value = MathUtil.applyDeadband(value, deadband);
    value = Math.copySign(value * value, value);
    return value;
  }

  @Override
  public void execute() {

   rotational_pid.setP(pEntry.getDouble(0));
   rotational_pid.setI(iEntry.getDouble(0));
   rotational_pid.setD(dEntry.getDouble(0));

   
  //rotational_pid.setTolerance(posEntry.getDouble(0), velEntry.getDouble(0));

    if (!RobotContainer.turnSwitch.getAsBoolean()) {
      rotationSup = () -> RobotContainer.operatorBoard.getRawAxis(3);
    } else {
      rotationSup = () -> RobotContainer.driver.getRawAxis(2);
    }

    translationSup = () -> -RobotContainer.driver.getRawAxis(1);
    strafeSup = () -> -RobotContainer.driver.getRawAxis(0);
    robotCentricSup = () -> RobotContainer.robotCentric.getAsBoolean();

    betterEncoderAngle = (rotationSup.getAsDouble() + 1) * 180;
    /*
     * betterEncoderAngle = ((rotationSup.getAsDouble() + 1) * )%360
     */
    // System.out.println("Better encoder angle: " + betterEncoderAngle);
    // System.out.println("RotationSup: " + rotationSup.getAsDouble());
    double translationVal = translationSup.getAsDouble();
    double strafeVal = strafeSup.getAsDouble();
    double rotationVal = rotationSup.getAsDouble();
    double new_Var = ((RobotContainer.s_Swerve.getGyroYaw().getDegrees()) - betterEncoderAngle);
    
    // if(new_Var > 180){
    //   new_Var -= 180;
    // }
    // else if(new_Var < -180){
    //   new_Var += 180;
    // }

    translationVal = squareAxis(logAxis(translationVal), Constants.stickDeadband);
    strafeVal = squareAxis(logAxis(strafeVal), Constants.stickDeadband);

    if (!RobotContainer.turnSwitch.getAsBoolean() && RobotContainer.operatorBoard.isConnected()) {

      rotationVal = rotational_pid.calculate((RobotContainer.s_Swerve.getGyroYaw().getDegrees()) , betterEncoderAngle ) ; // m_feedforward.calculate(rotational_pid.getSetpoint().velocity, rotationVal, rotationVal, rotationVal, rotationVal);
      rotationVal += (-ffEntry.getDouble(ff) * new_Var);
      
    } else {
      rotationVal = squareAxis(logAxis(rotationVal), Constants.stickRotationDeadband)
          * Constants.Swerve.maxAngularVelocity / 4;
    }

    rotValEntry.setDouble(rotationVal);
    diffValEntry.setDouble(new_Var);

    RobotContainer.s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal,
        !robotCentricSup.getAsBoolean(),
        true);
  }

  public boolean isFinished() {
    return false;
  }
}