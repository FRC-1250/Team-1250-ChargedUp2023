// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class ResetPoseAndHeading extends InstantCommand {
  private final Drivetrain drivetrain;

  public ResetPoseAndHeading(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void initialize() {
    drivetrain.resetDriveTalonPosition();
    drivetrain.resetOdometry(new Pose2d());
    drivetrain.zeroHeading();
  }
}
