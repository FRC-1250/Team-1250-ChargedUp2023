// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TrackTarget extends CommandBase {
  private final Limelight limelight;
  private final Drivetrain drivetrain;

  private double kP = -0.15;
  private double KProt = 0.01;
  private double xCorrect = 0;
  private double yCorrect = 0;
  private double rotCorrect = 0;
  private double tx;
  private double ty;

  public TrackTarget(Limelight limelight, Drivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
  }

  @Override
  public void execute() {
    tx = limelight.getXOffset();
    ty = limelight.getYOffset();
    if (isXOutsideBounds()) {
      xCorrect = tx * kP;
    } else {
      xCorrect = 0;
    }

    if (isYOutsideBounds()) {
      yCorrect = ty * kP;
    } else {
      yCorrect = 0;
    }

    var tempID = limelight.getfid();
    if (tempID == 1 || tempID == 2 || tempID == 3) {
      rotCorrect = -drivetrain.getHeading() * KProt;
    }
    drivetrain.drive(yCorrect, xCorrect, rotCorrect, true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return !isXOutsideBounds() && !isYOutsideBounds();
  }

  private boolean isXOutsideBounds() {
    return tx < -1 || tx > 1;
  }

  private boolean isYOutsideBounds() {
    return ty < -1 || ty > 1;
  }
}
