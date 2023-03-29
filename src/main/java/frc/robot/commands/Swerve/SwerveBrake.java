// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveBrake extends CommandBase {
  private final Drivetrain cmd_drivetrain;
  private final SwerveModuleState[] cmd_desiredStates = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
  };
  private final Timer timer = new Timer();

  public SwerveBrake(Drivetrain drivetrain) {
    cmd_drivetrain = drivetrain;
    addRequirements(cmd_drivetrain);
  }

  @Override
  public void initialize() {
    cmd_drivetrain.setModuleStates(cmd_desiredStates);
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    cmd_drivetrain.setModuleStates(cmd_desiredStates);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_drivetrain.drive(0, 0, 0, false);
  }
}
