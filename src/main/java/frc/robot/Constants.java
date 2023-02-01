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
  public static final int CONFIG_TIMEOUT_MS = 30;
  public static final int TALONFX_PRIMARY_PID_LOOP_ID = 0;
  public static final int TALONFX_SECONDARY_PID_LOOP_ID = 1;

  public static final class ArmCalibrations {
    public static final PIDGains PID_GAINS = new PIDGains(0.25, 0.0, 0.0, 0.0);
    public static final int TALON_CAN_ID = 46;
    public static final int BRAKE_SOLENOID_PORT = 0;
    public static final int ANGLE_SOLENOID_PORT = 1;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final int AMP_RESET_THRESHOLD = 10;
  }

  public final static class RevPneumaticModuleCalibration {
    public static final int MIN_COMPRESSOR_PRESSURE = 100;
    public static final int MAX_COMPRESSOR_PRESSURE = 120;
  }

  public final static class ElevatorCalibration {
    public static final PIDGains PID_GAINS = new PIDGains(0.25, 0.0, 0.0, 0.0);
    public static final int TALON_CAN_ID = 0;
    public static final int BRAKE_SOLENOID_PORT = 2;
    public static final int LIMIT_SWITCH_PORT = 3;
  }

  public final static class DrivetrainCalibration {
    /**
     * The maxmimum velocity that the swerve modules is capable of in meters per
     * second (m/s).
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
    private static final double TRACK_WIDTH = Units.inchesToMeters(32);

    /**
     * The distance between the front and back wheels on the robot.
     * This value must be in the same unit as {@trackWidth}.
     */
    private static final double WHEELBASE = Units.inchesToMeters(28);

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
  
    public static final int FRONT_LEFT_TURNING_TALON_CAN_ID = 17;
    public static final int FRONT_LEFT_DRIVE_TALON_CAN_ID = 18;
    public static final int FRONT_LEFT_CANCODER_CAN_ID = 19;
    public static final double FRONT_LEFT_CANCODER_OFFSET = -46.14258;

    public static final int FRONT_RIGHT_TURNING_TALON_CAN_ID = 14;
    public static final int FRONT_RIGHT_DRIVE_TALON_CAN_ID = 15;
    public static final int FRONT_RIGHT_CANCODER_CAN_ID = 16;
    public static final double FRONT_RIGHT_CANCODER_OFFSET = -83.14453;

    public static final int REAR_LEFT_TURNING_TALON_CAN_ID = 20;
    public static final int REAR_LEFT_DRIVE_TALON_CAN_ID = 21;
    public static final int REAR_LEFT_CANCODER_CAN_ID = 22;
    public static final double REAR_LEFT_CANCODER_OFFSET = -94.57031;

    public static final int REAR_RIGHT_TURNING_TALON_CAN_ID = 11;
    public static final int REAR_RIGHT_DRIVE_TALON_CAN_ID = 12;
    public static final int REAR_RIGHT_CANCODER_CAN_ID = 13;
    public static final double REAR_RIGHT_CANCODER_OFFSET = 85.78125;

    public static final int PIDGEON_CAN_ID = 23;

    public static final PIDGains DRIVE_TALON_VELOCITY_GAINS = new PIDGains(0.15, 0.0, 0.2, 0.0);
    public static final PIDGains TURNING_TALON_POSITION_GAINS = new PIDGains(0.7, 0.0, 0.1, 0.0);
    public static final double METERS_PER_SECOND_TO_TALON_TICKS_CONVERSION_FACTOR = 5293;
    public static final double DEGRESS_TO_TALON_TICKS_CONVERSION_FACTOR = 4096 / 360;
  }
}