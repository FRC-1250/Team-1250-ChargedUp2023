// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public Arm() {
    talon.configFactoryDefault(Constants.CONFIG_TIMEOUT_MS);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.ArmCalibrations.PID_GAINS.kP, Constants.CONFIG_TIMEOUT_MS);
    talon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.ArmCalibrations.PID_GAINS.kI, Constants.CONFIG_TIMEOUT_MS);
    talon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.ArmCalibrations.PID_GAINS.kD, Constants.CONFIG_TIMEOUT_MS);
  }

  public enum ArmPosition {
    TOP_NODE(100),
    MID_NODE(50),
    LOW_NODE(0),
    HOME(0);

    public final double positionInTicks;

    ArmPosition(double positionInTickets) {
      this.positionInTicks = positionInTickets;
    }
  }

  private WPI_TalonFX talon = new WPI_TalonFX(Constants.ArmCalibrations.TALON_CAN_ID);
  private Solenoid airBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmCalibrations.BRAKE_SOLENOID_PORT);
  private DigitalInput limitSwitch = new DigitalInput(Constants.ArmCalibrations.LIMIT_SWITCH_PORT);
  private Solenoid angleToggle = new Solenoid(PneumaticsModuleType.REVPH, Constants.ArmCalibrations.ANGLE_SOLENOID_PORT);

  public boolean getLimitSwitchState() {
    return limitSwitch.get();
  }

  public void setPosition(Double tickcount) {
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

  public void extendArm() {
    angleToggle.set(true);
  }

  public void retractArm() {
    angleToggle.set(false);
  }

  public void setSpeed(Double speed) {
    talon.set(speed);
  }

  private double getOutputCurrent() {
    return talon.getStatorCurrent();
  }

  public boolean isAmpThresholdReached() {
    return getOutputCurrent() > Constants.ArmCalibrations.AMP_RESET_THRESHOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
