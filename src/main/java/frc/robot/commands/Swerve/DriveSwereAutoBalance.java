// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSwereAutoBalance extends CommandBase {

  private final PIDController xController = new PIDController(0.035, 0, 0.01);
  private final PIDController yController = new PIDController(0.035, 0, 0);
  private final Drivetrain drivetrain;

  public DriveSwereAutoBalance(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    xController.setTolerance(5);
    yController.setTolerance(2.5);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(
        xController.calculate(drivetrain.getPitch(), 0),
        0,
        0, 
        false);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint();
  }
}
