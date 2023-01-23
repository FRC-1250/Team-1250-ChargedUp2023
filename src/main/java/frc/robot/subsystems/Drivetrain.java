// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.SwerveModuleTalonFX;

public class Drivetrain extends SubsystemBase {
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
  public static final double maxDriveSpeed = 4.115;

  /**
   * The maxmimum angular velocity that the swerve module is capable of in
   * rotations per second.
   * This value is easy to represent as some multiple of PI.
   * <p>
   * For example: 2 * Math.PI is 1 rotation per second.
   * Other options (2, 3, 4, 6, 2pi)
   */
  public static final double maxTurningSpeed = Math.PI; // rotation per second

  /**
   * The acceleration of the swerve module.
   */
  public static final double maxDriveAcceleration = 2.0575;

  private final SwerveModuleTalonFX frontLeftModule = new SwerveModuleTalonFX(
      Constants.FRONT_LEFT_DRIVE_TALON_CAN_ID,
      Constants.FRONT_LEFT_TURNING_TALON_CAN_ID,
      Constants.FRONT_LEFT_CANCODER_CAN_ID,
      Constants.FRONT_LEFT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX frontRightModule = new SwerveModuleTalonFX(
      Constants.FRONT_RIGHT_DRIVE_TALON_CAN_ID,
      Constants.FRONT_RIGHT_TURNING_TALON_CAN_ID,
      Constants.FRONT_RIGHT_CANCODER_CAN_ID,
      Constants.FRONT_RIGHT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX rearLeftModule = new SwerveModuleTalonFX(
      Constants.REAR_LEFT_DRIVE_TALON_CAN_ID,
      Constants.REAR_LEFT_TURNING_TALON_CAN_ID,
      Constants.REAR_LEFT_CANCODER_CAN_ID,
      Constants.REAR_LEFT_CANCODER_OFFSET);

  private final SwerveModuleTalonFX rearRightModule = new SwerveModuleTalonFX(
      Constants.REAR_RIGHT_DRIVE_TALON_CAN_ID,
      Constants.REAR_RIGHT_TURNING_TALON_CAN_ID,
      Constants.REAR_RIGHT_CANCODER_CAN_ID,
      Constants.REAR_RIGHT_CANCODER_OFFSET);

  private final WPI_Pigeon2 pidgey = new WPI_Pigeon2(
      Constants.PIDGEON_CAN_ID,
      Constants.CANIVORE_BUS_NAME);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      Constants.FRONT_LEFT_MODULE_LOCATION,
      Constants.FRONT_RIGHT_MODULE_LOCATION,
      Constants.REAR_LEFT_MODULE_LOCATION,
      Constants.REAR_RIGHT_MODULE_LOCATION);

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      kinematics, pidgey.getRotation2d(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearLeftModule.getPosition()
      }, new Pose2d(0, 0, new Rotation2d()));

  public Drivetrain() {
    pidgey.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    ChassisSpeeds speeds;

    if (fieldRelative) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, pidgey.getRotation2d());
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rotation);
    }

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxDriveSpeed);

    frontLeftModule.setDesiredState(desiredStates[0]);
    SmartDashboard.putString("2.0 Front left module current state", frontLeftModule.getState().toString());
    SmartDashboard.putString("2.0 Front left module desired state", desiredStates[0].toString());

    frontRightModule.setDesiredState(desiredStates[1]);
    SmartDashboard.putString("2.1 Front right module current state", frontRightModule.getState().toString());
    SmartDashboard.putString("2.1 Front right module desired state", desiredStates[1].toString());

    rearLeftModule.setDesiredState(desiredStates[2]);
    SmartDashboard.putString("2.2 Rear left module current state", rearLeftModule.getState().toString());
    SmartDashboard.putString("2.2 Rear left module desired state", desiredStates[2].toString());

    rearRightModule.setDesiredState(desiredStates[3]);
    SmartDashboard.putString("2.3 Rear right module current state", rearRightModule.getState().toString());
    SmartDashboard.putString("2.3 Rear right module desired state", desiredStates[3].toString());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pidgey.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pidgey.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return pidgey.getRate();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    /* odometry.resetPosition(pose, pidgey.getRotation2d()); */
  }

  @Override
  public void periodic() { // Update the pose
    Pose2d m_pose = m_odometry.update(pidgey.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(), frontRightModule.getPosition(),
            rearLeftModule.getPosition(), rearRightModule.getPosition()
        });

    SmartDashboard.putData("Pigeon heading", pidgey);
    SmartDashboard.putString("Pose",
        String.format("X: %.2f, Y: %.2f, Deg: %.2f ", m_pose.getX(), m_pose.getY(), m_pose.getRotation().getDegrees()));
    SmartDashboard.putString("Pose in inches",
        String.format("X: %.2f, Y: %.2f, Deg: %.2f ", Units.metersToInches(m_pose.getX()), Units.metersToInches(m_pose.getY()), m_pose.getRotation().getDegrees()));
  }
}