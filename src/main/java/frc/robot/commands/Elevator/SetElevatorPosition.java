// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.SystemStateHandler;
import frc.robot.modules.SystemStateHandler.SystemState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SetElevatorPosition extends CommandBase {

  private final Elevator cmd_elevator;
  private final double cmd_targetPosition;
  private double cmd_startingPosition;
  private SystemState cmd_superstructureState;

  public SetElevatorPosition(Elevator elevator, ElevatorPosition elevatorPosition) {
    this(elevator, elevatorPosition.positionInTicks);
  }

  public SetElevatorPosition(Elevator elevator, SystemState superstructureState) {
    this(elevator, superstructureState.elevatorPosition.positionInTicks);
    cmd_superstructureState = superstructureState;
  }

  public SetElevatorPosition(Elevator elevator, double positionInTicks) {
    addRequirements(elevator);
    cmd_targetPosition = positionInTicks;
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_startingPosition = cmd_elevator.getPosition();
    cmd_elevator.disableBrake();
  }

  @Override
  public void execute() {
    cmd_elevator.setPositionMotionMagic(cmd_targetPosition);
    if (cmd_superstructureState != null) {
      if (Math.abs(cmd_startingPosition - cmd_elevator.getPosition())
          / Math.abs(cmd_startingPosition - cmd_targetPosition) < 0.8) {
        SystemStateHandler.getInstance().setSystemState(cmd_superstructureState);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    cmd_elevator.enableBrake();
    cmd_elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return cmd_elevator.isAtSetPoint(cmd_targetPosition);
  }
}
