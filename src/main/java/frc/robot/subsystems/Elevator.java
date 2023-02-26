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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorCalibration;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    HARD_LIMIT(58),
    SOFT_LIMIT(50),
    TOP_CONE(46),
    SUBSTATION(37.375),
    TOP_CUBE(35.5),
    MID_CONE(34),
    MID_CUBE(23.5),
    HYBIRD(15),
    FLOOR(15),
    HOME(1);

    public final double positionInTicks;
    public final double positionInInches;

    ElevatorPosition(double e_positionInInches) {
      positionInInches = e_positionInInches;
      positionInTicks = e_positionInInches * ElevatorCalibration.INCHES_TO_TICK_CONVERSION;
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

    SlotConfiguration downSlotConfiguration = new SlotConfiguration();
    downSlotConfiguration.kP = Constants.ElevatorCalibration.PID_GAINS.kP;
    downSlotConfiguration.kI = Constants.ElevatorCalibration.PID_GAINS.kI;
    downSlotConfiguration.kD = Constants.ElevatorCalibration.PID_GAINS.kD;
    downSlotConfiguration.closedLoopPeakOutput = 0.1;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.peakOutputForward = 1;
    talonFXConfiguration.peakOutputReverse = 0;
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

  public void setPercentOutput(double speed, boolean override) {
    var direction = Math.signum(speed);

    if(override) {
      talon.set(speed);
    } else if(direction == 1 && talon.getSelectedSensorPosition() < ElevatorPosition.SOFT_LIMIT.positionInTicks) {
      talon.set(TalonFXControlMode.PercentOutput, speed);
    } else {
      talon.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void SetPosition(Double tickcount) {
    talon.set(TalonFXControlMode.Position, tickcount);
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
    // This method will be called once per scheduler run
  }
}
