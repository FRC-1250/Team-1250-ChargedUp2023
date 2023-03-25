// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.PIDGains;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final String CANIVORE_BUS_NAME = "Party Bus";
  public static final int CONFIG_TIMEOUT_MS = 10;
  public static final int TALONFX_PRIMARY_PID_LOOP_ID = 0;
  public static final int TALONFX_SECONDARY_PID_LOOP_ID = 1;
  public static final int TALONFX_INTEGRATED_SENSOR_RESOLUTION = 2048;
  public static final double TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR = TALONFX_INTEGRATED_SENSOR_RESOLUTION * 0.05;
  public static final double CONTROLLER_DEADBAND = 0.1;
  public static final double TALONFX_MAX_ROTATION_PER_100MS = 21777;

  public static final class PneumaticHubCalibrations {
    public static final int PNEUMATIC_HUB_ID = 20;
    public static final int MIN_COMPRESSOR_PRESSURE = 100;
    public static final int MAX_COMPRESSOR_PRESSURE = 120;
  }

  public static final class ArmCalibrations {
    public static final PIDGains PID_GAINS = new PIDGains(0.65, 0.0, 6.5, 1023 / TALONFX_MAX_ROTATION_PER_100MS);
    public static final int TALON_CAN_ID = 13;
    public static final double CLOSED_LOOP_RAMP_RATE = 1;
    public static final double PEAK_OUTPUT_FORWARD = 1;
    public static final double PEAK_OUTPUT_REVERSE = -1;
    public static final int BRAKE_SOLENOID_PORT = 0;
    public static final int ANGLE_SOLENOID_FORWARD_PORT = 2;
    public static final int ANGLE_SOLENOID_REVERSE_PORT = 7;
    public static final int AMP_RESET_THRESHOLD = 100;
  }

  public static final class EndEffectorCalibrations {
    public static final int TALON_CAN_ID = 16;
  }

  public final static class ElevatorCalibrations {
    public static final PIDGains PID_GAINS = new PIDGains(0.75, 0.0, 7.5, 1023 / TALONFX_MAX_ROTATION_PER_100MS);
    public static final int TALON_CAN_ID = 14;
    public static final double CLOSED_LOOP_RAMP_RATE = 1.5;
    public static final double OPEN_LOOP_RAMP_RATE = 0.5;
    public static final double PEAK_OUTPUT_FORWARD = 0.75;
    public static final double PEAK_OUTPUT_REVERSE = 0;
    public static final int BRAKE_SOLENOID_PORT = 1;
  }

  public final static class DrivetrainCalibration {
    /**
     * The maxmimum velocity that the swerve modules is capable of in meters per
     * This value can be derived mathimatically OR will be available from the
     * manufacturer of the
     * swerve module.
     * 
     * @see <a href=
     *      "https://www.swervedrivespecialties.com/products/mk4-swerve-module">MK4
     *      Swerve Module L1 - Standard </a>
     */
    public static final double MAX_DRIVE_SPEED = 4.115;

    /**
     * @see MAX_DRIVE_SPEED
     */
    public static final double MAX_AUTO_DRIVE_SPEED = 4.115;

     /**
     * The acceleration of the swerve module.
     */
    public static final double MAX_DRIVE_ACCELERATION = 3;

    /**
     * The maxmimum angular velocity that the swerve module is capable of in
     * rotations per second.
     * This value is easy to represent as some multiple of PI.
     * <p>
     * For example: 2 * Math.PI is 1 rotation per second.
     * Other options (2, 3, 4, 6, 2pi)
     */
    public static final double MAX_TURNING_SPEED = Math.PI; // rotation per second

    /**
     * @see MAX_TURNING_SPEED
     */
    public static final double MAX_AUTO_TURNING_SPEED = Math.PI;
    public static final double MAX_TURNING_ACCELERATION = Math.PI;
   

    /**
     * The disatance between the centers of the right and left wheels on the robot.
     * This value must be in the same unit as {@wheelBase}.
     */
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);

    /**
     * The distance between the front and back wheels on the robot.
     * This value must be in the same unit as {@trackWidth}.
     */
    public static final double WHEELBASE = Units.inchesToMeters(25);

    public static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(WHEELBASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d REAR_LEFT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d REAR_RIGHT_MODULE_LOCATION = new Translation2d(-WHEELBASE / 2, -TRACK_WIDTH / 2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_MODULE_LOCATION,
        FRONT_RIGHT_MODULE_LOCATION,
        REAR_LEFT_MODULE_LOCATION,
        REAR_RIGHT_MODULE_LOCATION);

    public static final TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(MAX_AUTO_DRIVE_SPEED,
        MAX_DRIVE_ACCELERATION).setKinematics(KINEMATICS);

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
    new TrapezoidProfile.Constraints(
        MAX_TURNING_SPEED, MAX_TURNING_ACCELERATION);
  
    public static final int FRONT_LEFT_TURNING_TALON_CAN_ID = 11;
    public static final int FRONT_LEFT_DRIVE_TALON_CAN_ID = 12;
    public static final int FRONT_LEFT_CANCODER_CAN_ID = 10;
    public static final double FRONT_LEFT_CANCODER_OFFSET = -93.955;

    public static final int FRONT_RIGHT_TURNING_TALON_CAN_ID = 2;
    public static final int FRONT_RIGHT_DRIVE_TALON_CAN_ID = 3;
    public static final int FRONT_RIGHT_CANCODER_CAN_ID = 1;
    public static final double FRONT_RIGHT_CANCODER_OFFSET = 7.119;

    public static final int REAR_LEFT_TURNING_TALON_CAN_ID = 8;
    public static final int REAR_LEFT_DRIVE_TALON_CAN_ID = 9;
    public static final int REAR_LEFT_CANCODER_CAN_ID = 7;
    public static final double REAR_LEFT_CANCODER_OFFSET = -5.186;

    public static final int REAR_RIGHT_TURNING_TALON_CAN_ID = 5;
    public static final int REAR_RIGHT_DRIVE_TALON_CAN_ID = 6;
    public static final int REAR_RIGHT_CANCODER_CAN_ID = 4;
    public static final double REAR_RIGHT_CANCODER_OFFSET = -65.654;

    public static final int PIDGEON_CAN_ID = 15;

    public static final PIDGains DRIVE_TALON_VELOCITY_GAINS = new PIDGains(0.1, 0.0, 1, 0.0);
    public static final PIDGains TURNING_TALON_POSITION_GAINS = new PIDGains(0.5, 0.0, 5, 0.0);
    public static final double METERS_PER_SECOND_TO_TALON_TICKS_CONVERSION_FACTOR = 5293;
    public static final double DEGRESS_TO_TALON_TICKS_CONVERSION_FACTOR = 4096 / 360;
  }
}