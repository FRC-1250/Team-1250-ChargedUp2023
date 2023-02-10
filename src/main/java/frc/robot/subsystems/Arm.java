// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    // Max 104183
    TOP_NODE(85544),
    MID_NODE(50736),
    HOME(500);

    public final double positionInTicks;

    ArmPosition(double e_positionInTicks) {
      this.positionInTicks = e_positionInTicks;
    }
  }

  public enum PIDProfile {
    EXTEND(0),
    RETRACT(1);

    public final int id;

    PIDProfile(int e_id) {
      id = e_id;
    }
  }

  private WPI_TalonFX talon = new WPI_TalonFX(Constants.ArmCalibrations.TALON_CAN_ID);
  private Solenoid airBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmCalibrations.BRAKE_SOLENOID_PORT);
  private DoubleSolenoid angleToggle = new DoubleSolenoid(
    PneumaticsModuleType.REVPH, 
    Constants.ArmCalibrations.ANGLE_SOLENOID_FORWARD_PORT,
    Constants.ArmCalibrations.ANGLE_SOLENOID_FORWARD_PORT);
  private boolean reverseLocked = false;

  public Arm() {
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    SlotConfiguration extendSlotConfiguration = new SlotConfiguration();
    extendSlotConfiguration.kP = Constants.ArmCalibrations.PID_GAINS.kP;
    extendSlotConfiguration.kI = Constants.ArmCalibrations.PID_GAINS.kI;
    extendSlotConfiguration.kD = Constants.ArmCalibrations.PID_GAINS.kD;
    extendSlotConfiguration.closedLoopPeakOutput = 0.4;
    
    SlotConfiguration retractSlotConfiguration = new SlotConfiguration();
    retractSlotConfiguration.kP = Constants.ArmCalibrations.PID_GAINS.kP;
    retractSlotConfiguration.kI = Constants.ArmCalibrations.PID_GAINS.kI;
    retractSlotConfiguration.kD = Constants.ArmCalibrations.PID_GAINS.kD;
    retractSlotConfiguration.closedLoopPeakOutput = 0.7;

    // TODO: Arm amp usage is very high when all springs are engaged closed to home. This creates stall points on the arm itself. This sometimes will trigger the reverse lock.
    StatorCurrentLimitConfiguration statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration();
    statorCurrentLimitConfiguration.enable = true;
    statorCurrentLimitConfiguration.currentLimit = 60;
    statorCurrentLimitConfiguration.triggerThresholdCurrent = 60;
    statorCurrentLimitConfiguration.triggerThresholdTime = 0.1;

    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.slot0 = extendSlotConfiguration;
    talonFXConfiguration.slot1 = retractSlotConfiguration;
    talonFXConfiguration.statorCurrLimit = statorCurrentLimitConfiguration;
    talonFXConfiguration.closedloopRamp = 0.5;
    talonFXConfiguration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    talon.configAllSettings(talonFXConfiguration, Constants.CONFIG_TIMEOUT_MS);
  }

  private void setPIDProfile(PIDProfile pidProfile) {
    talon.selectProfileSlot(pidProfile.id, 0);
  }

  public void setPIDProfile(double targetPosition) {
    var direction = Math.signum(targetPosition - getPosition());
    if (direction == 1) {
      setPIDProfile(Arm.PIDProfile.EXTEND);
    } else if (direction == -1) {
      setPIDProfile(Arm.PIDProfile.RETRACT);
    }
  }

  public void setSpeed(Double speed) {
    var direction = Math.signum(speed);
    if (direction == -1 && !reverseLocked || direction == 1) {
      talon.set(speed);
      reverseLocked = false;
    } else {
      talon.set(0);
    }
  }

  public void setPosition(double targetPositon) {
    var direction = Math.signum(targetPositon - getPosition());
    if (direction == -1 && !reverseLocked || direction == 1) {
      talon.set(TalonFXControlMode.Position, targetPositon);
      reverseLocked = false;
    }
  }

  public double getPosition() {
    return talon.getSelectedSensorPosition();
  }

  public void resetPosition() {
    talon.setSelectedSensorPosition(0, 0, Constants.CONFIG_TIMEOUT_MS);
  }

  public void enableBrake() {
    airBrake.set(true);
  }

  public void disableBrake() {
    airBrake.set(false);
  }

  public void extendArm() {
    angleToggle.set(DoubleSolenoid.Value.kForward);
  }

  public void retractArm() {
    angleToggle.set(DoubleSolenoid.Value.kReverse);
  }

  public void normalizeArm() {
    angleToggle.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverseLocked() {
    if (talon.getStatorCurrent() > Constants.ArmCalibrations.AMP_RESET_THRESHOLD) {
      reverseLocked = true;
      resetPosition();
    }
  }

  public boolean isReverseLocked() {
    return reverseLocked;
  }

  @Override
  public void periodic() {
    setReverseLocked();
    SmartDashboard.putNumber("Stator current", talon.getStatorCurrent());
    SmartDashboard.putNumber("Supply Current", talon.getSupplyCurrent());
  }
}
