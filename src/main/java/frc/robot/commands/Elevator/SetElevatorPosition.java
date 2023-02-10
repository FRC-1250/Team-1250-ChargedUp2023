// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends CommandBase {
  
  private final Elevator cmd_elevator;
  private final double cmd_position;

  public SetElevatorPosition(Elevator elevator, double position) {
    addRequirements(elevator);
    cmd_position = position;
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_elevator.disableBrake();
    cmd_elevator.setPIDProfile(cmd_position);
  }

  @Override
  public void execute() {
    cmd_elevator.SetPosition(cmd_position);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_elevator.setPercentOutput(0);
    cmd_elevator.enableBrake();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(cmd_elevator.getPosition() - cmd_position) < Constants.TALONFX_INTEGRATED_SENSOR_RESOLUTION * 0.05;
  }
}
