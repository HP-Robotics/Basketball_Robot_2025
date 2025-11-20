// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kShooterButtonId = 1;
  }

  public static class ShooterConstants {
    public static final int shooterLeaderMotorId = 41;
    public static final int shooterFollowerMotorId = 42;
    public static final double shooterAllowedError = 5;
    public static final double shooterSpeed = -50;
    public static final double shooterkP = 0.035;
    public static final double shooterkI = 0.01;
    public static final double shooterkD = 0;
    public static final double shooterkS = 0.253906;
    public static final double shooterkV = 0.0091;
  }

}
