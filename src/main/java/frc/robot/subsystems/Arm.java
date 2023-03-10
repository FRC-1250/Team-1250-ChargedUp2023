// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    LIMIT(192000),
    TOP_CONE(150000),
    TOP_CUBE(150000),
    MID_CONE(73920),
    MID_CUBE(73920),
    HOME(500),
    PAST_BUMPER(41600),
    AT_BUMPER(10000);

    public final double positionInTicks;

    ArmPosition(double e_positionInTicks) {
      positionInTicks = e_positionInTicks;
    }
  }

  private final WPI_TalonFX talon = new WPI_TalonFX(Constants.ArmCalibrations.TALON_CAN_ID);
  private final Solenoid airBrake;
  private final DoubleSolenoid angleToggle;
  private final PneumaticHub pneumaticHub;
  private boolean reverseLocked = false;
  private Timer reverseLockTimer = new Timer();

  public Arm(PneumaticHub subPneumaticHub) {
    pneumaticHub = subPneumaticHub;
    angleToggle = pneumaticHub.makeDoubleSolenoid(Constants.ArmCalibrations.ANGLE_SOLENOID_FORWARD_PORT,
        Constants.ArmCalibrations.ANGLE_SOLENOID_REVERSE_PORT);
    airBrake = pneumaticHub.makeSolenoid(Constants.ArmCalibrations.BRAKE_SOLENOID_PORT);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talon.setInverted(true);

    SlotConfiguration slotConfiguration = new SlotConfiguration();
    slotConfiguration.kP = Constants.ArmCalibrations.PID_GAINS.kP;
    slotConfiguration.kI = Constants.ArmCalibrations.PID_GAINS.kI;
    slotConfiguration.kD = Constants.ArmCalibrations.PID_GAINS.kD;
    slotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.slot0 = slotConfiguration;
    talonFXConfiguration.peakOutputForward = Constants.ArmCalibrations.PEAK_OUTPUT_FORWARD;
    talonFXConfiguration.peakOutputReverse = Constants.ArmCalibrations.PEAK_OUTPUT_REVERSE;
    talonFXConfiguration.closedloopRamp = Constants.ArmCalibrations.CLOSED_LOOP_RAMP_RATE;
    talonFXConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    talon.configAllSettings(talonFXConfiguration, Constants.CONFIG_TIMEOUT_MS);
  }

  public void setPercentOutput(double speed, boolean override) {
    var direction = Math.signum(speed);

    if (override) {
      talon.set(speed);
    } else if (direction == 1 && talon.getSelectedSensorPosition() < ArmPosition.LIMIT.positionInTicks) {
      talon.set(speed);
    } else if (direction == -1 && talon.getSelectedSensorPosition() > ArmPosition.HOME.positionInTicks) {
      talon.set(speed);
    } else {
      talon.set(0);
    }
  }

  public void stop() {
    talon.set(0);
  }

  public void setPosition(double targetPositon) {
    talon.set(ControlMode.Position, targetPositon);
  }

  public boolean isAtSetPoint(double targetPosition) {
    return Math.abs(targetPosition - talon.getSelectedSensorPosition()) < Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;
  }

  public double getPosition() {
    return talon.getSelectedSensorPosition();
  }

  public void resetPosition() {
    talon.setSelectedSensorPosition(0, 0, Constants.CONFIG_TIMEOUT_MS);
  }

  public void enableBrake() {
    airBrake.set(false);
  }

  public void disableBrake() {
    airBrake.set(true);
  }

  public void rotateArmDown() {
    angleToggle.set(DoubleSolenoid.Value.kForward);
  }

  public void rotateArmUp() {
    angleToggle.set(DoubleSolenoid.Value.kReverse);
  }

  public void normalizeArm() {
    angleToggle.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverseLocked() {
    if (talon.getSupplyCurrent() > Constants.ArmCalibrations.AMP_RESET_THRESHOLD) {
      reverseLockTimer.start();
    } else {
      reverseLockTimer.stop();
      reverseLockTimer.reset();
    }

    if (reverseLockTimer.hasElapsed(0.1)) {
      reverseLocked = true;
      resetPosition();
    }
  }

  public boolean isReverseLocked() {
    return reverseLocked;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm sensor position", talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm percent output", talon.get());
    if (talon.getControlMode() == ControlMode.Position) {
      SmartDashboard.putNumber("Arm closed loop error", talon.getClosedLoopError());
      SmartDashboard.putNumber("Arm closed loop target", talon.getClosedLoopTarget());
    } else {
      SmartDashboard.putNumber("Arm closed loop error", 0);
      SmartDashboard.putNumber("Arm closed loop target", 0);
    }
  }
}
