// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    TOP_NODE(42424),
    MID_NODE(30883),
    LOW_NODE(10000),
    HOME(0);

    public final double positionInTicks;

    ElevatorPosition(double e_positionInTicks) {
      positionInTicks = e_positionInTicks;
    }
  }

  public enum PIDProfile {
    UP(0),
    DOWN(1);

    public final int id;

    PIDProfile(int e_id) {
      id = e_id;
    }
  }

  private WPI_TalonFX talon = new WPI_TalonFX(Constants.ElevatorCalibration.TALON_CAN_ID);
  private Solenoid airBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ElevatorCalibration.BRAKE_SOLENOID_PORT);

  public Elevator() {
    talon.setNeutralMode(NeutralMode.Brake);

    SlotConfiguration upSlotConfiguration = new SlotConfiguration();
    upSlotConfiguration.kP = Constants.ElevatorCalibration.PID_GAINS.kP;
    upSlotConfiguration.kI = Constants.ElevatorCalibration.PID_GAINS.kI;
    upSlotConfiguration.kD = Constants.ElevatorCalibration.PID_GAINS.kD;
    upSlotConfiguration.closedLoopPeakOutput = 0.5;

    SlotConfiguration downSlotConfiguration = new SlotConfiguration();
    downSlotConfiguration.kP = Constants.ElevatorCalibration.PID_GAINS.kP;
    downSlotConfiguration.kI = Constants.ElevatorCalibration.PID_GAINS.kI;
    downSlotConfiguration.kD = Constants.ElevatorCalibration.PID_GAINS.kD;
    downSlotConfiguration.closedLoopPeakOutput = 0.2;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.slot0 = upSlotConfiguration;
    talonFXConfiguration.slot1 = downSlotConfiguration;
    talonFXConfiguration.closedloopRamp = 0.5;
    talonFXConfiguration.clearPositionOnLimitR = true;
    talonFXConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    talon.configAllSettings(talonFXConfiguration, Constants.CONFIG_TIMEOUT_MS);
  }

  private void setPIDProfile(PIDProfile pidProfile) {
    talon.selectProfileSlot(pidProfile.id, 0);
  }

  public void setPIDProfile(double targetPosition) {
    var direction = Math.signum(targetPosition - getPosition());
    if (direction == 1) {
      setPIDProfile(Elevator.PIDProfile.UP);
    } else if (direction == -1) {
      setPIDProfile(Elevator.PIDProfile.DOWN);
    }
  }

  public void setPercentOutput(double speed) {
    talon.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void SetPosition(Double tickcount) {
    talon.set(TalonFXControlMode.Position, tickcount);
  }

  public double getPosition() {
    return talon.getSelectedSensorPosition();
  }

  public void resetPosition() {
    talon.setSelectedSensorPosition(0);
  }

  public void enableBrake() {
    airBrake.set(true);
  }

  public void disableBrake() {
    airBrake.set(false);
  }

  public boolean isRevLimitSwitchClosed() {
    return talon.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
