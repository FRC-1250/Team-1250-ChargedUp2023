// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    LIMIT(47500),
    TOP_CONE(44250),
    SUBSTATION(41500),
    TOP_CUBE(37200),
    MID_CONE(34100),
    MID_CUBE(23500),
    HYBRID(2300),
    FLOOR(2300),
    HOME(2300);

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

  private final WPI_TalonFX talon = new WPI_TalonFX(Constants.ElevatorCalibration.TALON_CAN_ID);
  private final Solenoid airBrake;
  private final PneumaticHub pneumaticHub;

  public Elevator(PneumaticHub subPneumaticHub) {
    pneumaticHub = subPneumaticHub;
    airBrake = pneumaticHub.makeSolenoid(Constants.ElevatorCalibration.BRAKE_SOLENOID_PORT);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    SlotConfiguration upSlotConfiguration = new SlotConfiguration();
    upSlotConfiguration.kP = Constants.ElevatorCalibration.PID_GAINS.kP;
    upSlotConfiguration.kI = Constants.ElevatorCalibration.PID_GAINS.kI;
    upSlotConfiguration.kD = Constants.ElevatorCalibration.PID_GAINS.kD;
    upSlotConfiguration.closedLoopPeakOutput = 0.5;
    upSlotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;

    SlotConfiguration downSlotConfiguration = new SlotConfiguration();
    downSlotConfiguration.kP = Constants.ElevatorCalibration.PID_GAINS.kP;
    downSlotConfiguration.kI = Constants.ElevatorCalibration.PID_GAINS.kI;
    downSlotConfiguration.kD = Constants.ElevatorCalibration.PID_GAINS.kD;
    downSlotConfiguration.closedLoopPeakOutput = 0.5;
    downSlotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.peakOutputForward = 1;
    talonFXConfiguration.peakOutputReverse = 0;
    talonFXConfiguration.slot0 = upSlotConfiguration;
    talonFXConfiguration.slot1 = downSlotConfiguration;
    talonFXConfiguration.closedloopRamp = 1;
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

  public void setPercentOutput(double speed, boolean override) {
    var direction = Math.signum(speed);

    if(override) {
      talon.set(speed);
    } else if(direction == 1 && talon.getSelectedSensorPosition() < ElevatorPosition.LIMIT.positionInTicks) {
      talon.set(TalonFXControlMode.PercentOutput, speed);
    } else {
      talon.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void SetPosition(Double tickcount) {
    talon.set(TalonFXControlMode.Position, tickcount);
  }

  public boolean isAtSetPoint(double targetPosition) {
    return Math.abs(targetPosition - talon.getSelectedSensorPosition()) < Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;
  }

  public void stop() {
    talon.set(0);
  }

  public double getPosition() {
    return talon.getSelectedSensorPosition();
  }

  public void resetPosition() {
    talon.setSelectedSensorPosition(0);
  }

  public void enableBrake() {
    airBrake.set(false);
  }

  public void disableBrake() {
    airBrake.set(true);
  }

  public boolean isRevLimitSwitchClosed() {
    return talon.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator position", talon.getSelectedSensorPosition());
  }
}
