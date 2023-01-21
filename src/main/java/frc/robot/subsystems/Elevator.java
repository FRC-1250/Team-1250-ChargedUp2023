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

  private WPI_TalonFX ElevatorTalon = new WPI_TalonFX(Constants.ElevatorTalon_CAN_ID);

  private Solenoid ElevatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

  private DigitalInput ElevatorLimitswitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);

  /** Creates a new Elevator. */
  public Elevator() {
    ElevatorTalon.config_kP(0, Constants.DRIVE_TALON_POSITION_ELEVATOR_GAINS.kP, Constants.kTimeoutMs);
    ElevatorTalon.config_kI(Constants.kPIDLoopIdx, Constants.DRIVE_TALON_POSITION_ELEVATOR_GAINS.kI, Constants.kTimeoutMs);
    ElevatorTalon.config_kD(Constants.kPIDLoopIdx, Constants.DRIVE_TALON_POSITION_ELEVATOR_GAINS.kD, Constants.kTimeoutMs);
  }

  public void ToggleSolenoid() {
    ElevatorSolenoid.toggle();
  }

  public void SetSolenoid(boolean value) {
    ElevatorSolenoid.set(value);
  }

  public boolean GetLimitSwitchState() {
    return ElevatorLimitswitch.get();
  }

  public void SetPosition(Double tickcount) {
    ElevatorTalon.set(TalonFXControlMode.Position, tickcount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
