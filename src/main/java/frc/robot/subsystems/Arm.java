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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    LIMIT(100000),
    TOP_CONE(78600),
    SUBSTATION(21700),
    TOP_CUBE(78600),
    MID_CONE(38500),
    MID_CUBE(38500),
    HYBRID(21700),
    FLOOR(21700),
    HOME(21700),
    PRE_EXTEND(21700);

    public final double positionInTicks;

    ArmPosition(double e_positionInTicks) {
      positionInTicks = e_positionInTicks;
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

  private final WPI_TalonFX talon = new WPI_TalonFX(Constants.ArmCalibrations.TALON_CAN_ID);
  private final Solenoid airBrake;
  private final DoubleSolenoid angleToggle;
  private final PneumaticHub pneumaticHub;
  private boolean reverseLocked = false;
  private Timer reverseLockTimer = new Timer();

  public Arm(PneumaticHub subPneumaticHub) {
    pneumaticHub = subPneumaticHub;
    angleToggle = pneumaticHub.makeDoubleSolenoid(Constants.ArmCalibrations.ANGLE_SOLENOID_FORWARD_PORT, Constants.ArmCalibrations.ANGLE_SOLENOID_REVERSE_PORT);
    airBrake = pneumaticHub.makeSolenoid(Constants.ArmCalibrations.BRAKE_SOLENOID_PORT);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talon.setInverted(true);

    SlotConfiguration extendSlotConfiguration = new SlotConfiguration();
    extendSlotConfiguration.kP = Constants.ArmCalibrations.EXTEND_PID_GAINS.kP;
    extendSlotConfiguration.kI = Constants.ArmCalibrations.EXTEND_PID_GAINS.kI;
    extendSlotConfiguration.kD = Constants.ArmCalibrations.EXTEND_PID_GAINS.kD;
    extendSlotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;
    
    SlotConfiguration retractSlotConfiguration = new SlotConfiguration();
    retractSlotConfiguration.kP = Constants.ArmCalibrations.RETRACT_PID_GAINS.kP;
    retractSlotConfiguration.kI = Constants.ArmCalibrations.RETRACT_PID_GAINS.kI;
    retractSlotConfiguration.kD = Constants.ArmCalibrations.RETRACT_PID_GAINS.kD;
    retractSlotConfiguration.allowableClosedloopError = Constants.TALONFX_ALLOWABLE_CLOSED_LOOP_ERROR;
    
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.slot0 = extendSlotConfiguration;
    talonFXConfiguration.slot1 = retractSlotConfiguration;
    talonFXConfiguration.closedloopRamp = 1;
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

  public void setPercentOutput(double speed, boolean override) {
    var direction = Math.signum(speed);

    if(override) {
      talon.set(speed);
    } else if(direction == 1 && talon.getSelectedSensorPosition() < ArmPosition.LIMIT.positionInTicks) {
      talon.set(TalonFXControlMode.PercentOutput, speed);
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
      talon.set(TalonFXControlMode.Position, targetPositon);
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

    if(reverseLockTimer.hasElapsed(0.1)) {
      reverseLocked = true;
      resetPosition();
    }
  }

  public boolean isReverseLocked() {
    return reverseLocked;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm position", talon.getSelectedSensorPosition());
  }
}
