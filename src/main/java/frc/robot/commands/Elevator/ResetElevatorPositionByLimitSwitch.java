// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ResetElevatorPositionByLimitSwitch extends CommandBase {
  private final Elevator cmd_elevator;

  public ResetElevatorPositionByLimitSwitch(Elevator elevator) {
    addRequirements(elevator);
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_elevator.disableBrake();
  }

  @Override
  public void execute() {
    cmd_elevator.setPercentOutput(-0.2);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_elevator.setPercentOutput(0);
    cmd_elevator.enableBrake();
  }

  @Override
  public boolean isFinished() {
    return cmd_elevator.isRevLimitSwitchClosed();
  }
}
