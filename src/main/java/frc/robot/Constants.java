// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.PIDGains;

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

  }

  public static final int MIN_COMPRESSOR_PRESSURE = 100;
  public static final int MAX_COMPRESSOR_PRESSURE = 120;

  // Elevator
  public static final PIDGains DRIVE_TALON_POSITION_ELEVATOR_GAINS = new PIDGains(0.25, 0.0, 0.0, 0.0);
  public static final int ElevatorTalon_CAN_ID = 0;
  public static final int kTimeoutMs = 30;
  public static final int kPIDLoopIdx = 0;
  public static final int LIMIT_SWITCH_PORT = 0;
}
