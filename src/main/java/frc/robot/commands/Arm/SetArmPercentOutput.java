// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPercentOutput extends CommandBase {
  private final Arm cmd_arm;
  private final double cmd_speed;
  private final boolean cmd_override;

  public SetArmPercentOutput(Arm arm, double speed, boolean override) {
    cmd_arm = arm;
    cmd_speed = speed;
    cmd_override = override;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    cmd_arm.disableBrake();
  }

  @Override
  public void execute() {
    cmd_arm.setPercentOutput(cmd_speed, cmd_override);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_arm.stop();
    cmd_arm.enableBrake();
  }
}
