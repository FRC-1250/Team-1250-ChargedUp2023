// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class SetElevatorPercentOutput extends CommandBase {
  
  private final Elevator cmd_elevator;
  private final double cmd_speed;

  public SetElevatorPercentOutput(Elevator elevator, double speed) {
    addRequirements(elevator);
    cmd_speed = speed;
    cmd_elevator = elevator;
  }

  @Override
  public void initialize() {
    cmd_elevator.disableBrake();
  }

  @Override
  public void execute() {
    cmd_elevator.setPercentOutput(cmd_speed);
  }

  @Override
  public void end(boolean interrupted) {
    cmd_elevator.setPercentOutput(0);
    cmd_elevator.enableBrake();
  }
}
