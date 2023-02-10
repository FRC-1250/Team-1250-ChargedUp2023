// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class ResetElevatorPosition extends InstantCommand {

  private final Elevator cmd_elevator;

  public ResetElevatorPosition(Elevator elevator) {
    addRequirements(elevator);
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_elevator.resetPosition();
  }
}
