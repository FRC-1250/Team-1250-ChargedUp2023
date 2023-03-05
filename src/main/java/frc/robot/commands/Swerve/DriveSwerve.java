// Copyright (c) FIRST and other WPILib contriutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainCalibration;
import frc.robot.subsystems.Drivetrain;

public class DriveSwerve extends CommandBase {

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);
  private final Drivetrain drivetrain;
  private final IntSupplier centerOfRotationSupplier;
  private final BooleanSupplier boostInputSupplier;
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier rotationThrottleSupplier;
  private final DoubleSupplier yInputSupplier;
  private final DoubleSupplier xInputSupplier;
  private final DoubleSupplier rotationInputSupplier;
  private final boolean fieldRelative;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  public DriveSwerve(
      IntSupplier centerOfRotationSupplier,
      BooleanSupplier boostInputSupplier,
      DoubleSupplier throttleSupplier,
      DoubleSupplier rotationThrottleSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      DoubleSupplier rotationInputSupplier,
      boolean fieldRelative,
      Drivetrain drivetrain) {
    this.centerOfRotationSupplier = centerOfRotationSupplier;
    this.boostInputSupplier = boostInputSupplier;
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
    xSpeed = -yInputSupplier.getAsDouble();
    ySpeed = -xInputSupplier.getAsDouble();
    rotSpeed = -rotationInputSupplier.getAsDouble();

    if (!boostInputSupplier.getAsBoolean()) {
      ySpeed = ySpeed * throttleSupplier.getAsDouble();
      xSpeed = xSpeed * throttleSupplier.getAsDouble();
      rotSpeed = rotSpeed * rotationThrottleSupplier.getAsDouble();
    }

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
    rotSpeed = MathUtil.applyDeadband(rotSpeed, 0.1);

    xSpeed = m_xspeedLimiter.calculate(xSpeed);
    ySpeed = m_yspeedLimiter.calculate(ySpeed);
    rotSpeed = m_rotLimiter.calculate(rotSpeed);

    xSpeed = xSpeed * Constants.DrivetrainCalibration.MAX_DRIVE_SPEED;
    ySpeed = ySpeed * Constants.DrivetrainCalibration.MAX_DRIVE_SPEED;
    rotSpeed = rotSpeed * Constants.DrivetrainCalibration.MAX_DRIVE_SPEED;

    var centerOfRotation = centerOfRotationSupplier.getAsInt();
    if (centerOfRotation != -1) {
      drivetrain.drive(
          xSpeed,
          ySpeed,
          rotSpeed,
          new Translation2d(DrivetrainCalibration.WHEELBASE / 2, 0)
              .rotateBy(Rotation2d.fromDegrees(centerOfRotation)),
          fieldRelative);
    } else {
      drivetrain.drive(xSpeed, ySpeed, rotSpeed, fieldRelative);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }
}
