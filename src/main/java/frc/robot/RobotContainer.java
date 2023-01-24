// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RevPneumaticModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveSwerve;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final RevPneumaticModule revPneumaticModule = new RevPneumaticModule(
      Constants.RevPneumaticModule.MIN_COMPRESSOR_PRESSURE,
      Constants.RevPneumaticModule.MAX_COMPRESSOR_PRESSURE);
  private final Limelight limelight = new Limelight();

  private final XboxController xboxController = new XboxController(0);

  public RobotContainer() {
    configureButtonBindings();
    drivetrain.setDefaultCommand(
        new DriveSwerve(
            xboxController::getRightBumper,
            0.75,
            xboxController::getLeftY,
            xboxController::getLeftX,
            xboxController::getRightX,
            true,
            drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
