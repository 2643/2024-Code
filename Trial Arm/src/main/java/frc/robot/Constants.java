// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int gearRatio = 1; //change to 151 once you get on the robot  
  public static final int leftmotorPort = 4;  
  public static final int rightmotorPort = 0;
  public static final int limitPort = 1;
  public static final int ENCODER_PORT = 2;

  public static final double AMP = 0.08;
  public static final double REST = 0;
  public static final double SPEAKER = 0.32;
  public static final double HOOK = 0.16;
  public static final double FLOOR = 0.04;
  public static final double COUNT_PER_DEGREES = 0;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
