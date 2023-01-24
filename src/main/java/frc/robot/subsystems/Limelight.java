// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
// Owned by Eli M. and Caleb S.
  private final NetworkTable table;

  /*
   * tv Whether the limelight has any valid targets (0 or 1)
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   * ta Target Area (0% of image to 100% of image)
   */
  private double tx, ty, tv, ta, ts = -1;
  private String json = "" ;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public Limelight(String LimelightTable) {
    table = NetworkTableInstance.getDefault().getTable(LimelightTable);
  }

  public double getXOffset() {
    return tx;
  }

  public double getYOffset() {
    return ty;
  }

  public double getTargetAreaPercent() {
    return ta;
  }

  public double getSkew() {
    return ts;
    
    
  }
public String getjsonString() {
return json;
}
  /**
   * ledMode Sets limelight’s LED state
   * 0 use the LED Mode set in the current pipeline
   * 1 force off
   * 2 force blink
   * 3 force on
   */

  public void setLEDMode(int value) {
    if (value > 3 || value < 0)
      value = 0;
    table.getEntry("ledMode").setNumber(value);
  }

  public void setPipeline(int value) {
    if (value > 9 || value < 0)
     value = 0;
     table.getEntry("pipeline").setNumber(value);
  }
  public boolean isTargetSeen() {
    if (tv == 1) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    tx = table.getEntry("tx").getDouble(-1);
    ty = table.getEntry("ty").getDouble(-1);
    tv = table.getEntry("tv").getDouble(-1);
    ts = table.getEntry("ts").getDouble(-1);
   json = table.getEntry("json").getString("");
  }
}
