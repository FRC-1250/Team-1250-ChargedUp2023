// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class SetEndEffectorSpeed extends CommandBase {
  private final EndEffector cmd_endEffector;
  private final double cmd_speed;

  public SetEndEffectorSpeed(EndEffector endEffector, double speed) {
    cmd_endEffector = endEffector;
    cmd_speed = speed;
    addRequirements(endEffector);
  }

  @Override
  public void execute() {
    cmd_endEffector.setPercentOutput(cmd_speed);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_endEffector.setPercentOutput(0);
  }
}
