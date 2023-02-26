// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RevPneumaticModule extends SubsystemBase {

  /*
   * Hide the compressor so that the disable functionality is not directly
   * available through this class!
   */
  private final Compressor compressor = new Compressor(20, PneumaticsModuleType.REVPH);

  /**
   * Enable the rev module in DIGITAL mode.
   */
  public RevPneumaticModule() {
    compressor.enableDigital();
  }

  /**
   * Enable the rev module in ANALOG mode. The ANALOG wiring must be
   * placed into port 0 of the rev pnumatics module in order for the compressor to
   * work.
   * 
   * @param minPressure The minimum pressure in PSI. The compressor will turn on
   *                    when the pressure drops below this value.
   * @param maxPressure The maximum pressure in PSI. The compressor will turn off
   *                    when the pressure reaches this value.
   */
  public RevPneumaticModule(double minPressure, double maxPressure) {
    compressor.enableAnalog(minPressure, maxPressure);
  }

  public double getPressure() {
    return compressor.getPressure();
  }

  public double getCurrent() {
    return compressor.getCurrent();
  }

  public boolean isPressureLimitReached() {
    return compressor.getPressureSwitchValue();
  }

}