// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {}

  CANSparkMax ElevatorSpark = new CANSparkMax(Constants.ElevatorSpark_CAN_ID,MotorType.kBrushless);

  public enum ElevatorHeight{
    TOP_NODE(100),
    MID_NODE(50),
    LOW_NODE(0); 

    public final double heightInTicks; 

    ElevatorHeight(double heightInTicks){
      this.heightInTicks = heightInTicks; 
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
