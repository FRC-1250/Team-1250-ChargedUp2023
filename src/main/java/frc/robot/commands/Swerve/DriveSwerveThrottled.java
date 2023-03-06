// Copyright (c) FIRST and other WPILib contriutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSwerveThrottled extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier rotationThrottleSupplier;
  private final DoubleSupplier yInputSupplier;
  private final DoubleSupplier xInputSupplier;
  private final DoubleSupplier rotationInputSupplier;
  private final boolean fieldRelative;

  public DriveSwerveThrottled(
      DoubleSupplier throttleSupplier,
      DoubleSupplier rotationThrottleSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      DoubleSupplier rotationInputSupplier,
      boolean fieldRelative,
      Drivetrain drivetrain) {
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
        fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }
}
