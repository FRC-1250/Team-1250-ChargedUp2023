// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;

public class SetArmPosition extends CommandBase {
  private final Arm cmd_arm;
  private final Double cmd_positionInTicks;

  public SetArmPosition(Arm arm, ArmPosition armPosition) {
    this(arm, armPosition.positionInTicks);
  }

  public SetArmPosition(Arm arm, Double positionInTicks) {
    cmd_arm = arm;
    cmd_positionInTicks = positionInTicks;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    cmd_arm.setPosition(cmd_positionInTicks);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_arm.setSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(cmd_arm.getPosition() - cmd_positionInTicks) < 100 ;
  }
}
