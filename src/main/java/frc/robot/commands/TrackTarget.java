// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private Double[] targetposeincameraspace;
  private double tx;
  private double ty;

  public TrackTarget(Limelight limelight, Drivetrain drivetrain) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetposeincameraspace = limelight.getcamtran();
    tx = limelight.getXOffset();
    ty = limelight.getYOffset();
    if (tx < -1 || tx > 1) {
      xCorrect = tx * kP;
    } else {
      xCorrect = 0;
    }
    if (ty < -1 || ty > 1) {
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
