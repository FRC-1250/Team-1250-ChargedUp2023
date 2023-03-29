// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.SystemStateHandler;
import frc.robot.subsystems.Arm;

public class ExtendArmBySystemState extends CommandBase {
  private final Arm cmd_arm;

  public ExtendArmBySystemState(Arm arm) {
    cmd_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    cmd_arm.disableBrake();
  }

  @Override
  public void execute() {
    cmd_arm.setPositionMotionMagic(SystemStateHandler.getInstance().getSystemState().armActionExtension.positionInTicks);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_arm.stop();
    cmd_arm.enableBrake();
  }

  @Override
  public boolean isFinished() {
    return cmd_arm.isAtSetPoint(SystemStateHandler.getInstance().getSystemState().armActionExtension.positionInTicks);
  }
}
