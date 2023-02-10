// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ExtendArm extends InstantCommand {

  private final Arm cmd_arm;

  public ExtendArm(Arm arm) {
    cmd_arm = arm;
  }

  @Override
  public void initialize() {
    cmd_arm.extendArm();
  }
}
