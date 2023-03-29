// Copyright (c) FIRST and other WPILib contriutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainCalibration;
import frc.robot.subsystems.Drivetrain;

public class DriveSwerveOffsetCenter extends CommandBase {

  private final Drivetrain drivetrain;
  private final IntSupplier centerOfRotationSupplier;
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier rotationThrottleSupplier;
  private final DoubleSupplier yInputSupplier;
  private final DoubleSupplier xInputSupplier;
  private final DoubleSupplier rotationInputSupplier;
  private final boolean fieldRelative;

  public DriveSwerveOffsetCenter(
      IntSupplier centerOfRotationSupplier,
      DoubleSupplier throttleSupplier,
      DoubleSupplier rotationThrottleSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      DoubleSupplier rotationInputSupplier,
      boolean fieldRelative,
      Drivetrain drivetrain) {
    this.centerOfRotationSupplier = centerOfRotationSupplier;
    this.throttleSupplier = throttleSupplier;
    this.rotationThrottleSupplier = rotationThrottleSupplier;
    this.yInputSupplier = yInputSupplier;
    this.xInputSupplier = xInputSupplier;
    this.rotationInputSupplier = rotationInputSupplier;
    this.drivetrain = drivetrain;
    this.fieldRelative = fieldRelative;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(
        drivetrain.calculateSpeed(-yInputSupplier.getAsDouble(), throttleSupplier.getAsDouble()),
        drivetrain.calculateSpeed(-xInputSupplier.getAsDouble(), throttleSupplier.getAsDouble()),
        drivetrain.calculateRotationSpeed(-rotationInputSupplier.getAsDouble(), rotationThrottleSupplier.getAsDouble()),
        new Translation2d(DrivetrainCalibration.WHEELBASE / 2, 0)
            .rotateBy(Rotation2d.fromDegrees(centerOfRotationSupplier.getAsInt())),
        fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }
}
