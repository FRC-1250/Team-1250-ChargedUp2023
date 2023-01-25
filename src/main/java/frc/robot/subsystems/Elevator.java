// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  public enum ElevatorHeight {
    TOP_NODE(100),
    MID_NODE(50),
    LOW_NODE(0);

    public final double heightInTicks;

    ElevatorHeight(double heightInTicks) {
      this.heightInTicks = heightInTicks;
    }
  }

  private WPI_TalonFX ElevatorTalon = new WPI_TalonFX(Constants.Elevator.TALON_CAN_ID);
  private Solenoid ElevatorBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.Elevator.BRAKE_SOLENOID_PORT);
  private DigitalInput ElevatorLimitswitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_PORT);

  /** Creates a new Elevator. */
  public Elevator() {
    ElevatorTalon.config_kP(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Elevator.PID_GAINS.kP,
        Constants.CONFIG_TIMEOUT_MS);
    ElevatorTalon.config_kI(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Elevator.PID_GAINS.kI,
        Constants.CONFIG_TIMEOUT_MS);
    ElevatorTalon.config_kD(Constants.TALONFX_PRIMARY_PID_LOOP_ID, Constants.Elevator.PID_GAINS.kD,
        Constants.CONFIG_TIMEOUT_MS);
  }

  public boolean GetLimitSwitchState() {
    return ElevatorLimitswitch.get();
  }

  public void SetPosition(Double tickcount) {
    ElevatorTalon.set(TalonFXControlMode.Position, tickcount);
  }

  public void enableBrake() {
    ElevatorBrake
        .set(true);
  }

  public void disableBrake() {
    ElevatorBrake
        .set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
