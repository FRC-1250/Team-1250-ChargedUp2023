// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorCalibrations;

public class EndEffector extends SubsystemBase {
  WPI_TalonSRX talon = new WPI_TalonSRX(EndEffectorCalibrations.TALON_CAN_ID);

  public EndEffector() {
    TalonSRXConfiguration talonSRXConfiguration = new TalonSRXConfiguration();
    talonSRXConfiguration.continuousCurrentLimit = 20;
    talonSRXConfiguration.peakCurrentLimit = 0;
    talon.configAllSettings(talonSRXConfiguration, Constants.CONFIG_TIMEOUT_MS);
    talon.setNeutralMode(NeutralMode.Brake);
  }

  public void setPercentOutput(double speed) {
    talon.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
  }
}
