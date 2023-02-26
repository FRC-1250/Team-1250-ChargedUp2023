// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.modules.SystemStateHandler;
import frc.robot.modules.SystemStateHandler.SystemState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class SetElevatorPosition extends CommandBase {
  
  private final Elevator cmd_elevator;
  private final double cmd_positionInTicks;
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
    cmd_positionInTicks = positionInTicks;
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_elevator.disableBrake();
    cmd_elevator.setPIDProfile(cmd_positionInTicks);
  }

  @Override
  public void execute() {
    cmd_elevator.SetPosition(cmd_positionInTicks);
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted == false && cmd_superstructureState != null) {
      SystemStateHandler.getInstance().setSuperstructureState(cmd_superstructureState);
    }

    cmd_elevator.stop();
    cmd_elevator.enableBrake();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(cmd_elevator.getPosition() - cmd_positionInTicks) < Constants.TALONFX_INTEGRATED_SENSOR_RESOLUTION * 0.25;
  }
}
