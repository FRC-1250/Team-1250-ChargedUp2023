// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    TOP_CUBE(37200),
    MID_CONE(34100),
    MID_CUBE(23500),
    SINGLE_SUBSTATION_CONE(3567),
    SINGLE_SUBSTATION_CUBE(12450),
    DOUBLE_SUBSTATION_CONE(34430),
    DOUBLE_SUBSTATION_CUBE(42500),//Was 41500
    CARRY(3000),
    FLOOR_CONE(2700),
    FLOOR_CUBE(2300),
    HOME(500);

    public final double positionInTicks;

    ElevatorPosition(double e_positionInTicks) {
      positionInTicks = e_positionInTicks;
    }
  }

  private final WPI_TalonFX talon = new WPI_TalonFX(Constants.ElevatorCalibrations.TALON_CAN_ID);
  private final Solenoid airBrake;
  private final PneumaticHub pneumaticHub;

  public Elevator(PneumaticHub subPneumaticHub) {
    pneumaticHub = subPneumaticHub;
    airBrake = pneumaticHub.makeSolenoid(Constants.ElevatorCalibrations.BRAKE_SOLENOID_PORT);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    SlotConfiguration slotConfiguration = new SlotConfiguration();
    slotConfiguration.kP = Constants.ElevatorCalibrations.PID_GAINS.kP;
    slotConfiguration.kI = Constants.ElevatorCalibrations.PID_GAINS.kI;
    slotConfiguration.kD = Constants.ElevatorCalibrations.PID_GAINS.kD;
    slotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.slot0 = slotConfiguration;
    talonFXConfiguration.peakOutputForward = Constants.ElevatorCalibrations.PEAK_OUTPUT_FORWARD;
    talonFXConfiguration.peakOutputReverse = Constants.ElevatorCalibrations.PEAK_OUTPUT_REVERSE;
    talonFXConfiguration.openloopRamp = Constants.ElevatorCalibrations.OPEN_LOOP_RAMP_RATE;
    talonFXConfiguration.clearPositionOnLimitR = true;
    talonFXConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    /*
     * Motion magic
     */
    talonFXConfiguration.motionAcceleration = Constants.TALONFX_MAX_ROTATION_PER_100MS * 0.75;
    talonFXConfiguration.motionCruiseVelocity = Constants.TALONFX_MAX_ROTATION_PER_100MS * 0.75;
    talonFXConfiguration.motionCurveStrength = 0;

    talon.configAllSettings(talonFXConfiguration, Constants.CONFIG_TIMEOUT_MS);
    SmartDashboard.putBoolean("elevatorMotion", false);
  }

  public void setPercentOutput(double speed, boolean override) {
    var direction = Math.signum(speed);
    SmartDashboard.putBoolean("elevatorMotion", true);
    if(override) {
      talon.set(speed);
    } else if(direction == 1 && talon.getSelectedSensorPosition() < ElevatorPosition.LIMIT.positionInTicks) {
      talon.set(speed);
    } else {
      talon.set(0);
    }
  }

  public void SetPosition(Double tickcount) {
    talon.set(ControlMode.Position, tickcount);
    SmartDashboard.putBoolean("elevatorMotion", true);
  }

  public void setPositionMotionMagic(double targetPosition) {
      talon.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, 0.1);
  }

  public boolean isAtSetPoint(double targetPosition) {
    return Math.abs(targetPosition - talon.getSelectedSensorPosition()) < Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;
  }

  public void stop() {
    talon.set(0);
    SmartDashboard.putBoolean("elevatorMotion", false);
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
    SmartDashboard.putNumber("Elevator sensor position", talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator percent output", talon.get());
    SmartDashboard.putNumber("Elevator sensor velocity", talon.getSelectedSensorVelocity());
    if (talon.getControlMode() == ControlMode.Position || talon.getControlMode() == ControlMode.MotionMagic) {
      SmartDashboard.putNumber("Elevator closed loop error", talon.getClosedLoopError());
      SmartDashboard.putNumber("Elevator closed loop target", talon.getClosedLoopTarget());
    }
  }
}
