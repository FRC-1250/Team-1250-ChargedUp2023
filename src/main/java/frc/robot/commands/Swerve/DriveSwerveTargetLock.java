// Copyright (c) FIRST and other WPILib contriutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveSwerveTargetLock extends CommandBase {

  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier yInputSupplier;
  private final DoubleSupplier xInputSupplier;
  private final boolean fieldRelative;
  private double rotSpeed;
  private long fid;
  private Alliance alliance;
  private final PIDController rotController = new PIDController(0.05, 0, 0);

  public DriveSwerveTargetLock(
      DoubleSupplier throttleSupplier,
      DoubleSupplier rotationThrottleSupplier,
      DoubleSupplier yInputSupplier,
      DoubleSupplier xInputSupplier,
      DoubleSupplier rotationInputSupplier,
      boolean fieldRelative,
      Drivetrain drivetrain,
      Limelight limelight) {
    this.throttleSupplier = throttleSupplier;
    this.yInputSupplier = yInputSupplier;
    this.xInputSupplier = xInputSupplier;
    this.drivetrain = drivetrain;
    this.fieldRelative = fieldRelative;
    this.limelight = limelight;
    rotController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain, limelight);
  }

  @Override
  public void initialize() {
    alliance = DriverStation.getAlliance();
  }

  @Override
  public void execute() {
    fid = limelight.getfid();
    if (fid == -1) {
      rotSpeed = 0;
    } else if (alliance == Alliance.Blue) {
      if (fid == 6 || fid == 7 || fid == 8) {
        rotSpeed = rotController.calculate(drivetrain.getHeading(), 180);
      } else if (fid == 4) {
        rotSpeed = rotController.calculate(drivetrain.getHeading(), 0);
      }
    } else if (alliance == Alliance.Red) {
      if (fid == 1 || fid == 2 || fid == 3) {
        rotSpeed = rotController.calculate(drivetrain.getHeading(), 180);
      } else if (fid == 5) {
        rotSpeed = rotController.calculate(drivetrain.getHeading(), 0);
      }
    }

    drivetrain.drive(
        drivetrain.calculateSpeed(-yInputSupplier.getAsDouble(), throttleSupplier.getAsDouble()),
        drivetrain.calculateSpeed(-xInputSupplier.getAsDouble(), throttleSupplier.getAsDouble()),
        rotSpeed,
        fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }
}
