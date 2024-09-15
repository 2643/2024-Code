// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  double[] botPoseBlueTable;
  double targetX;
  double x;
  double y;
  double autoP = 0.05;
  double autoI = 0.005;
  double autoD = 0;
  final double maxPos = -0.205;
  final double minPos = -0.15;
  final double maxDis = -4;
  final double minDis = 21;
  double armPercent;
  double offset = Units.degreesToRadians(98);
  double trig;
  double yDisShooterSpeaker = Units.inchesToMeters(66.5);

  // GenericEntry autoPEntry = Shuffleboard.getTab("autoPID").add("P", 0).getEntry();
  // GenericEntry autoIEntry = Shuffleboard.getTab("autoPID").add("I", 0).getEntry();
  // GenericEntry autoDEntry = Shuffleboard.getTab("autoPID").add("D", 0).getEntry();
  GenericEntry angleOffset = Shuffleboard.getTab("autoPID").add("angle", 100).getEntry();
  GenericEntry errorEntry = Shuffleboard.getTab("autoPID").add("Error", 0).getEntry();
  //GenericEntry turnEntry = Shuffleboard.getTab("autoPID").add("Turn", 0).getEntry();
  LinearFilter autoAimFilter = LinearFilter.movingAverage(10);
  MedianFilter outlierFilter = new MedianFilter(7);



  PIDController autoAnglePID = new PIDController(0.01, 0, 0);
  PIDController angularLockPID = new PIDController(0.05, 0.0005, 0.04);

  /** Creates a new Vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    //offset = Units.degreesToRadians(angleOffset.getDouble(100));

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-smokey");
    NetworkTableEntry botPoseBlue = table.getEntry("botpose_wpiblue");

    // autoAnglePID.setP(autoPEntry.getDouble(0));
    // autoAnglePID.setI(autoIEntry.getDouble(0));
    // autoAnglePID.setD(autoDEntry.getDouble(0));

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);

    botPoseBlueTable = botPoseBlue.getDoubleArray(new double[11]);

//post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    errorEntry.setDouble(x);
    // This method will be called once per scheduler run
  }

  // leaving here, but i don't think its needed

  public double autoArm() {
    trig = (Math.atan(yDisShooterSpeaker/outlierFilter.calculate(botPoseBlueTable[9])) - offset)/(2*Math.PI);
    return autoAimFilter.calculate(trig);
  }

  // needs to be readjusted

  public double autoAngle() {
    targetX = x;
    if (targetX < 0) {
      //turnEntry.setDouble(autoAnglePID.calculate(targetX) - 0.1);
      return autoAnglePID.calculate(targetX - 0.2);
    }
    else
      //turnEntry.setDouble(autoAnglePID.calculate(targetX + 2));
      return autoAnglePID.calculate(targetX + 0.2);
  }

  public double getTagPose() {
    System.out.println(angularLockPID.calculate(botPoseBlueTable[5]));
    return angularLockPID.calculate(botPoseBlueTable[5]);
  }

  public boolean isApriltag() {
    if (x == 0 && y == 0)
      return false;
    else
      return true;
  }

  public double getError() {
    return targetX;
  }
}
