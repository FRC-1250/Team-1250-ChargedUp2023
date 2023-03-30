// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class WaitForDriveCoordinates extends CommandBase {

  private final Pose2d cmd_targetPose;
  private final Drivetrain cmd_drivetrain;
  private final double cmd_acceptanceThreshold;

  public WaitForDriveCoordinates(Drivetrain drivetrain, Pose2d targetPose, double acceptanceThreshold) {
    cmd_drivetrain = drivetrain;
    cmd_targetPose = targetPose;
    cmd_acceptanceThreshold = acceptanceThreshold;
  }

  @Override
  public boolean isFinished() {
    var pose = cmd_drivetrain.getPose();
    return Math.sqrt((Math.pow(cmd_targetPose.getX() - pose.getX(), 2)
        + Math.pow(cmd_targetPose.getY() - pose.getY(), 2))) < cmd_acceptanceThreshold;
  }
}
