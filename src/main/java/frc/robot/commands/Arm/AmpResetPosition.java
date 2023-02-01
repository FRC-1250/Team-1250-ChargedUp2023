// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class AmpResetPosition extends CommandBase {

  private final Arm cmd_arm;

  public AmpResetPosition(Arm arm) {
    cmd_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    cmd_arm.setSpeed(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_arm.setSpeed(0.0);
    cmd_arm.resetPosition();
  }

  @Override
  public boolean isFinished() {
    return cmd_arm.isAmpThresholdReached();
  }
}
