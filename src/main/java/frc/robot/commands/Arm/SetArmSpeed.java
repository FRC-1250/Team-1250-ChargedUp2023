// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmSpeed extends CommandBase {
  private final Arm cmd_arm;
  private final Double cmd_talonSpeed;

  public SetArmSpeed(Arm arm, Double talonSpeed) {
    cmd_arm = arm;
    cmd_talonSpeed = talonSpeed;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    cmd_arm.disableBrake();
  }

  @Override
  public void execute() {
    cmd_arm.setSpeed(cmd_talonSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_arm.setSpeed(0.0);
    cmd_arm.enableBrake();
  }
}
