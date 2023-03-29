// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class WaitForArm extends CommandBase {

  double cmd_targetPosition;
  double cmd_startingPosition;
  double cmd_acceptableThreshold;
  Arm cmd_arm;

  public WaitForArm(Arm arm, double targetPosition, double acceptableThreshold) {
    cmd_targetPosition = targetPosition;
    cmd_acceptableThreshold = acceptableThreshold;
    cmd_arm = arm;
  }

  public WaitForArm(Arm arm, ArmPosition targetPosition, double acceptableThreshold) {
    this(arm, targetPosition.positionInTicks, acceptableThreshold);
  }

  @Override
  public void initialize() {
    cmd_startingPosition = cmd_arm.getPosition();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(cmd_startingPosition - cmd_arm.getPosition())
        / Math.abs(cmd_startingPosition - cmd_targetPosition) < cmd_acceptableThreshold;
  }
}
