// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public Arm() {
    ArmTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Arm.PID_GAINS.kP, Constants.CONFIG_TIMEOUT_MS);
    ArmTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Arm.PID_GAINS.kI, Constants.CONFIG_TIMEOUT_MS);
    ArmTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Arm.PID_GAINS.kD, Constants.CONFIG_TIMEOUT_MS);
  }

  private WPI_TalonFX ArmTalon = new WPI_TalonFX(Constants.Arm.TALON_CAN_ID);
  private Solenoid ArmBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.BRAKE_SOLENOID_PORT);
  private DigitalInput LimitSwitch = new DigitalInput(Constants.Arm.LIMIT_SWITCH_PORT);
  private Solenoid ArmAngle = new Solenoid(PneumaticsModuleType.REVPH, Constants.Arm.ANGLE_SOLENOID_PORT);

  public boolean GetLimitSwitchState() {
    return LimitSwitch.get();
  }

  public void SetPosition(Double tickcount) {
    ArmTalon.set(TalonFXControlMode.Position, tickcount);
  }

  public void enableBrake() {
    ArmBrake
        .set(true);
  }

  public void disableBrake() {
    ArmBrake
        .set(false);
  }

  public void extendArm() {
    ArmAngle.set(true);

  }

  public void retractArm() {
    ArmAngle.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
