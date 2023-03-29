// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class WaitForElevator extends CommandBase {

  double cmd_targetPosition;
  double cmd_startingPosition;
  double cmd_acceptableThreshold;
  Elevator cmd_elevator;

  public WaitForElevator(Elevator elevator, double targetPosition, double acceptableThreshold) {
    cmd_targetPosition = targetPosition;
    cmd_acceptableThreshold = acceptableThreshold;
    cmd_elevator = elevator;
  }

  public WaitForElevator(Elevator elevator, ElevatorPosition targetPosition, double acceptableThreshold) {
    this(elevator, targetPosition.positionInTicks, acceptableThreshold);
  }

  @Override
  public void initialize() {
    cmd_startingPosition = cmd_elevator.getPosition();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(cmd_startingPosition - cmd_elevator.getPosition())
        / Math.abs(cmd_startingPosition - cmd_targetPosition) < cmd_acceptableThreshold;
  }
}
